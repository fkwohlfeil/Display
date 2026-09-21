#!/usr/bin/env python3
"""Prueft die Ablaufsteuerung von urdf_gui2_17 unter xvfb.

move_group laeuft nicht. Statt die Bewegungsebene nachzubauen, wird die
Sicht darauf ausgetauscht: der Ablauf bekommt eine Attrappe untergeschoben,
und geprueft wird, was die Oberflaeche darum herum tut -- Knoepfe, Anzeige,
Sperre, Takt, Abbruch beim Umbau des Modells.

Die Programmebene selbst ist in test_programm_ablauf.py geprueft, ohne
Fenster und mit gestellter Uhr. Hier geht es nur um die Naht.
"""

import importlib.util
import math
import os
import sys
import tempfile

HIER = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HIER)

import programm as prg

URDF = """<?xml version="1.0"?>
<robot name="pruefarm">
  <link name="base_link"/>
  <link name="glied1"/>
  <link name="glied2"/>
  <link name="werkzeug"/>
  <joint name="achse1" type="revolute">
    <parent link="base_link"/><child link="glied1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="achse2" type="revolute">
    <parent link="glied1"/><child link="glied2"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-1.5" upper="1.5" effort="10" velocity="1"/>
  </joint>
  <joint name="flansch" type="fixed">
    <parent link="glied2"/><child link="werkzeug"/>
    <origin xyz="0.15 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""

CHECKS = []


def check(label, bedingung, detail=""):
    CHECKS.append((label, bool(bedingung), detail))


class Attrappe:
    """Bewegungsebene, die der Test von Hand weiterstellt."""

    def __init__(self):
        self.frei = True
        self.plan = None
        self.plaene = []
        self.fahrten = 0
        self.abbrueche = 0

    def bereit(self):
        return self.frei

    def planen(self, werte, bewegung, tempo):
        self.plaene.append((dict(werte), bewegung, tempo))
        self.frei = False
        return prg.AUFTRAG_LAEUFT

    def plan_vorhanden(self):
        return self.plan is not None

    def fahren(self):
        self.fahrten += 1
        self.frei = False
        self.plan = None
        return prg.AUFTRAG_LAEUFT

    def abbrechen(self):
        self.abbrueche += 1
        self.plan = None

    def planung_fertig(self):
        self.plan = "bahn"
        self.frei = True

    def fahrt_fertig(self):
        self.frei = True


def main():
    ordner = tempfile.mkdtemp()
    urdf = os.path.join(ordner, "pruefarm.urdf")
    open(urdf, "w").write(URDF)

    spec = importlib.util.spec_from_file_location(
        "gui2_17", os.path.join(HIER, "urdf_gui2_17.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)

    gui = m.UrdfViewer(urdf)
    gui.update()

    protokoll = []
    echt = gui._log
    gui._log = lambda t, l="info": (protokoll.append((l, t)), echt(t, l))[1]

    antworten = {"text": ""}
    gui._ask_text = lambda t, f, v="": antworten["text"]
    m.messagebox.askyesno = lambda *a, **k: True

    # Die Bewegungsebene austauschen. Der Takt der Oberflaeche laeuft weiter
    # und ruft dieselbe Programmebene -- nur sieht sie jetzt die Attrappe.
    attrappe = Attrappe()
    gui.ablauf.maschine = attrappe

    gui._open_program()
    gui.update()

    # -- Programm aufbauen -------------------------------------------------
    sig = gui._program_signature()
    datei = gui.program_file
    datei.punkte.teachen("A", {"achse1": 0.0, "achse2": 0.0}, sig)
    datei.punkte.teachen("B", {"achse1": math.radians(30.0),
                               "achse2": math.radians(20.0)}, sig)
    datei.programm.einfuegen(prg.Schritt("fahren", "B", "PTP", 50))
    datei.programm.einfuegen(prg.Schritt("warten", dauer=0.2))
    datei.programm.einfuegen(prg.Schritt("fahren", "A", "PTP", 50))
    gui._program_refresh()
    gui.update()

    check("Anzeige zeigt anfangs kein Programm",
          gui.program_run_var.get() == prg.PROG_LEER,
          gui.program_run_var.get())
    check("Nichts gesperrt", gui.lock_reason is None, gui.lock_reason)

    # -- Einzelschritt -----------------------------------------------------
    gui._program_step()
    gui.update()
    check("Ablauf laeuft", gui.ablauf.zustand == prg.PROG_LAEUFT,
          gui.ablauf.zustand)
    check("Anzeige nennt Schritt 1 von 3",
          "Schritt 1 von 3" in gui.program_run_var.get(),
          gui.program_run_var.get())
    check("Modell waehrend des Laufs gesperrt",
          gui.lock_reason == m.PROGRAM_LOCK, gui.lock_reason)
    check("Sperre greift auch, wenn die Bewegungsebene ruht",
          gui.state == m.STATE_READY, gui.state)

    def takte(n=6):
        """Den Takt der Oberflaeche n-mal durchlaufen lassen."""
        for _ in range(n):
            gui.ablauf.takt()
            gui._program_drain()
            gui._program_run_state()
            gui.update()

    takte()
    check("Planauftrag herausgegangen", len(attrappe.plaene) == 1,
          attrappe.plaene)
    check("Sechs Takte erzeugen genau einen Auftrag",
          len(attrappe.plaene) == 1 and attrappe.fahrten == 0)
    check("Auftrag traegt die Achswerte von B",
          abs(attrappe.plaene[0][0]["achse1"] - math.radians(30.0)) < 1e-12,
          attrappe.plaene[0][0])

    attrappe.planung_fertig()
    takte()
    check("Danach wird gefahren", attrappe.fahrten == 1, attrappe.fahrten)
    attrappe.fahrt_fertig()
    takte()
    check("Schritt 1 fertig, Anzeige auf Schritt 2",
          "Schritt 2 von 3" in gui.program_run_var.get(),
          gui.program_run_var.get())
    check("Einzelschritt haelt an", gui.ablauf.zustand == prg.PROG_PAUSE,
          gui.ablauf.zustand)
    check("Pause haelt die Sperre", gui.lock_reason == m.PROGRAM_LOCK)
    check("Fertigmeldung im Protokoll",
          any("Schritt 1 fertig" in t for _l, t in protokoll), protokoll[-3:])

    # -- Umbau des Modells bricht den Lauf ab -------------------------------
    stand = len(protokoll)
    gui._program_recheck("Test")
    gui.update()
    check("Umbau bricht den Ablauf ab",
          gui.ablauf.zustand == prg.PROG_BEREIT, gui.ablauf.zustand)
    check("Abbruch gemeldet",
          any("abgebrochen" in t for _l, t in protokoll[stand:]),
          protokoll[stand:])
    check("Bewegungsebene geraeumt", attrappe.abbrueche == 1)
    check("Sperre wieder weg", gui.lock_reason is None, gui.lock_reason)

    # -- Warteschritt ------------------------------------------------------
    gui._program_step()
    takte()
    attrappe.planung_fertig()
    takte()
    attrappe.fahrt_fertig()
    takte()
    check("Wieder bei Schritt 2", gui.ablauf.schritt == 1, gui.ablauf.schritt)
    gui._program_step()
    takte(2)
    check("Restzeit steht in der Anzeige",
          "noch" in gui.program_run_var.get(), gui.program_run_var.get())
    check("Warteschritt legt keinen Auftrag hinein",
          len(attrappe.plaene) == 2, attrappe.plaene)

    import time
    ende = time.monotonic() + 1.0
    while gui.ablauf.schritt == 1 and time.monotonic() < ende:
        takte(1)
    check("Warteschritt laeuft ab", gui.ablauf.schritt == 2,
          gui.ablauf.schritt)
    check("Und haelt danach an", gui.ablauf.zustand == prg.PROG_PAUSE)
    check("Restzeit wieder weg", "noch" not in gui.program_run_var.get(),
          gui.program_run_var.get())

    # -- Abbruch von Hand ---------------------------------------------------
    stand = len(protokoll)
    gui._program_abort()
    gui.update()
    check("Abbruch setzt zurueck",
          gui.ablauf.zustand == prg.PROG_BEREIT and gui.ablauf.schritt == 0,
          (gui.ablauf.zustand, gui.ablauf.schritt))
    check("Abbruch nennt Schritt und Grund",
          any("Schritt 3" in t and "Bediener" in t
              for _l, t in protokoll[stand:]), protokoll[stand:])
    check("Sperre gefallen", gui.lock_reason is None)
    gui._program_abort()
    check("Abbruch ohne Lauf meldet nur",
          any("kein Ablauf" in t for _l, t in protokoll[-1:]), protokoll[-1])

    # -- Programm nicht startbar ---------------------------------------------
    datei.programm.einfuegen(prg.Schritt("fahren", "Weggeworfen"))
    gui._program_refresh()
    stand = len(protokoll)
    gui._program_step()
    check("Nicht startbares Programm startet nicht",
          gui.ablauf.zustand == prg.PROG_BEREIT, gui.ablauf.zustand)
    check("Grund nennt den fehlenden Punkt",
          any("Weggeworfen" in t for _l, t in protokoll[stand:]),
          protokoll[stand:])
    datei.programm.entfernen(len(datei.programm) - 1)
    gui._program_refresh()

    # -- Programm waehrend eines Laufs nicht austauschen ----------------------
    gui._program_step()
    takte(2)
    stand = len(protokoll)
    gui._program_new()
    check("Neues Programm waehrend des Laufs abgelehnt",
          len(gui.program_file.programm) == 3
          and any("Erst Abbruch" in t for _l, t in protokoll[stand:]),
          protokoll[stand:])
    gui._program_abort()

    # -- Hinweis auf die noch nicht angeschlossene Bewegungsart --------------
    # Der Hinweis sitzt in _Bewegungssicht.planen, also an der echten Naht
    # zur Bewegungsebene -- die Attrappe kommt dort nicht vorbei. Geprueft
    # wird deshalb die Sicht selbst.
    sicht = m._Bewegungssicht(gui)
    gui._program_unwired_noted = False
    stand = len(protokoll)
    antwort = sicht.planen({"achse1": math.radians(30.0),
                            "achse2": math.radians(20.0)}, "LIN", 30)
    check("LIN und Tempo werden als noch nicht angeschlossen gemeldet",
          any("erreichen den Planer" in t for _l, t in protokoll[stand:]),
          protokoll[stand:])
    check("Ohne move_group wird der Auftrag nicht angenommen",
          antwort == prg.AUFTRAG_FEHLER, antwort)

    stand2 = len(protokoll)
    sicht.planen({"achse1": math.radians(30.0),
                  "achse2": math.radians(20.0)}, "LIN", 30)
    check("Der Hinweis wiederholt sich nicht im selben Lauf",
          not any("erreichen den Planer" in t for _l, t in protokoll[stand2:]),
          [t for _l, t in protokoll[stand2:] if "Planer" in t])

    check("Ein schon erreichtes Ziel gilt als erledigt",
          sicht.planen({"achse1": gui.positions["achse1"],
                        "achse2": gui.positions["achse2"]}, "PTP", 50)
          == prg.AUFTRAG_ERLEDIGT)
    check("Und schreibt kein Ziel ins Fahrfenster",
          gui.state == m.STATE_READY, gui.state)

    # -- Takt ueberlebt eine Ausnahme ---------------------------------------
    kaputt = Attrappe()
    def sprengen(*_a, **_k):
        raise RuntimeError("absichtlich")
    kaputt.planen = sprengen
    gui.ablauf.maschine = kaputt
    gui._program_step()
    stand = len(protokoll)
    gui._program_tick()
    gui.update()
    check("Ausnahme im Takt bricht den Ablauf ab",
          gui.ablauf.zustand == prg.PROG_BEREIT, gui.ablauf.zustand)
    check("Und wird gemeldet",
          any("unerwarteter Fehler" in t for _l, t in protokoll[stand:]),
          protokoll[stand:])
    gui._program_tick()
    check("Der Takt laeuft danach weiter", True)

    gui.destroy()

    breite = max(len(label) for label, _ok, _d in CHECKS)
    fehl = 0
    for label, ok, detail in CHECKS:
        marke = "ok  " if ok else "FEHL"
        zeile = f"  {marke}  {label.ljust(breite)}"
        if not ok:
            fehl += 1
            zeile += f"   -> {detail}"
        print(zeile)
    print(f"\n{len(CHECKS) - fehl} von {len(CHECKS)} Pruefungen bestanden")
    return 1 if fehl else 0


if __name__ == "__main__":
    sys.exit(main())
