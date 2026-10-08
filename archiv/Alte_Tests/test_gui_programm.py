#!/usr/bin/env python3
"""Prueft das Programmfenster von urdf_gui2_17 unter xvfb.

Gebaut wird die echte UrdfViewer-Instanz gegen eine kleine URDF in einem
Ordner unter /tmp. ROS wird nicht gestartet -- RosBridge.start meldet nur,
dass rclpy fehlt, und die Oberflaeche kommt damit zurecht. Geprueft wird
alles, was ohne move_group entscheidbar ist: Teachen, Verweise, Anfahren
in die Zielfelder, Modellwechsel beim Oeffnen und die Sperren.
"""

import importlib.util
import math
import os
import sys
import tempfile

HIER = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HIER)

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

# Zweites Modell: achse2 hat eine engere obere Grenze und eine dritte Achse
# kommt dazu. Beides muss beim Laden einer Programmdatei auffallen.
URDF_ANDERS = URDF.replace('lower="-1.5" upper="1.5"',
                           'lower="-1.5" upper="0.3"').replace(
    "</robot>",
    """  <link name="glied3"/>
  <joint name="achse3" type="revolute">
    <parent link="glied2"/><child link="glied3"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>""")

CHECKS = []


def check(label, bedingung, detail=""):
    CHECKS.append((label, bool(bedingung), detail))


def laden(pfad):
    spec = importlib.util.spec_from_file_location("gui2_17", pfad)
    modul = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modul)
    return modul


def main():
    ordner = tempfile.mkdtemp()
    urdf = os.path.join(ordner, "pruefarm.urdf")
    urdf2 = os.path.join(ordner, "anders.urdf")
    open(urdf, "w").write(URDF)
    open(urdf2, "w").write(URDF_ANDERS)

    m = laden(os.path.join(HIER, "urdf_gui2_17.py"))
    import programm as prg

    # Fenster aufbauen. ROS bleibt aus; das meldet die Oberflaeche selbst.
    gui = m.UrdfViewer(urdf)
    gui.update()

    protokoll = []
    echtes_log = gui._log
    gui._log = lambda text, level="info": (protokoll.append((level, text)),
                                           echtes_log(text, level))[1]

    # Statt Dialogen feste Antworten. Ohne das wartet der Test auf einen
    # Menschen, den es unter xvfb nicht gibt.
    antworten = {"text": "", "janein": True, "janeinabbruch": True,
                 "datei_auf": "", "datei_ab": ""}
    gui._ask_text = lambda t, f, v="": antworten["text"]
    m.messagebox.askyesno = lambda *a, **k: antworten["janein"]
    m.messagebox.askyesnocancel = lambda *a, **k: antworten["janeinabbruch"]
    m.messagebox.showwarning = lambda *a, **k: None
    m.messagebox.showerror = lambda *a, **k: None
    m.filedialog.askopenfilename = lambda *a, **k: antworten["datei_auf"]
    m.filedialog.asksaveasfilename = lambda *a, **k: antworten["datei_ab"]

    # -- Fenster -----------------------------------------------------------
    gui._open_program()
    gui.update()
    check("Programmfenster existiert", gui._program_alive())
    check("Punktetabelle gebaut", gui.program_points is not None)
    check("Schritttabelle gebaut", gui.program_steps is not None)
    check("Titel ohne Stern", "*" not in gui.program_window.title(),
          gui.program_window.title())

    def zeilen(tabelle):
        return [tabelle.item(k, "values") for k in tabelle.get_children()]

    # -- Teachen -----------------------------------------------------------
    gui.positions["achse1"] = math.radians(20.0)
    gui.positions["achse2"] = math.radians(30.0)
    antworten["text"] = "Aufnahme"
    gui._program_teach()
    gui.update()
    check("Punkt angelegt", "Aufnahme" in gui.program_file.punkte,
          gui.program_file.punkte.namen())
    check("Punkt steht in der Tabelle", len(zeilen(gui.program_points)) == 1,
          zeilen(gui.program_points))
    check("Achswerte in grad angezeigt",
          "20" in zeilen(gui.program_points)[0][1],
          zeilen(gui.program_points)[0][1])
    check("Status gueltig", zeilen(gui.program_points)[0][3] == "gueltig",
          zeilen(gui.program_points)[0][3])
    check("TCP-Lage mitgeteacht",
          gui.program_file.punkte.get("Aufnahme").lage is not None)
    check("Stern nach dem Teachen", gui.program_file.geaendert)
    check("Titel zeigt den Stern", "*" in gui.program_window.title(),
          gui.program_window.title())

    antworten["text"] = "Aufnahme"
    gui._program_teach()
    check("Doppelter Name wird gemeldet",
          any("schon einen Punkt" in t for _l, t in protokoll[-2:]),
          protokoll[-1])
    check("Kein zweiter Punkt angelegt", len(gui.program_file.punkte) == 1)

    gui.positions["achse1"] = math.radians(-40.0)
    antworten["text"] = "Ablage"
    gui._program_teach()
    gui.update()
    check("Zweiter Punkt angelegt", len(gui.program_file.punkte) == 2)
    check("Auswahl steht auf dem neuen Punkt",
          gui.program_points.item(gui.program_points.selection()[0],
                                  "values")[0] == "Ablage")

    # -- Schritte ----------------------------------------------------------
    gui.program_points.selection_set(gui.program_points.get_children()[0])
    gui.program_speed_var.set("60")
    gui.program_move_var.set("LIN")
    gui._program_add_move()
    gui.update()
    check("Fahrschritt angehaengt", len(gui.program_file.programm) == 1)
    check("Schritt traegt Punkt, Bewegung und Tempo",
          zeilen(gui.program_steps)[0][2:5] == ("Aufnahme", "LIN", "60"),
          zeilen(gui.program_steps)[0])

    gui.program_wait_var.set("2.5")
    gui._program_add_wait()
    gui.program_points.selection_set(gui.program_points.get_children()[1])
    gui._program_add_move()
    gui.update()
    check("Drei Schritte", len(gui.program_file.programm) == 3)
    check("Warteschritt zeigt keine Bewegungsart",
          zeilen(gui.program_steps)[1][3] == "", zeilen(gui.program_steps)[1])

    gui.program_speed_var.set("999")
    gui._program_add_move()
    check("Tempo ausserhalb 1..100 abgelehnt",
          len(gui.program_file.programm) == 3
          and any("zwischen 1 und 100" in t for _l, t in protokoll[-1:]),
          protokoll[-1])
    gui.program_speed_var.set("50")

    gui.program_wait_var.set("-1")
    gui._program_add_wait()
    check("Negative Wartezeit abgelehnt", len(gui.program_file.programm) == 3)
    gui.program_wait_var.set("2.0")

    # -- Umsortieren -------------------------------------------------------
    gui.program_steps.selection_set(gui.program_steps.get_children()[2])
    gui._program_move_step(-1)
    gui.update()
    check("Schritt nach oben verschoben",
          [s.art for s in gui.program_file.programm]
          == ["fahren", "fahren", "warten"],
          [s.art for s in gui.program_file.programm])
    check("Auswahl folgt dem Schritt",
          gui.program_steps.item(gui.program_steps.selection()[0],
                                 "values")[0] == "2")
    gui._program_move_step(1)
    gui.update()

    # -- Umbenennen zieht Schritte mit --------------------------------------
    gui.program_points.selection_set(gui.program_points.get_children()[0])
    antworten["text"] = "Greifen"
    gui._program_rename()
    gui.update()
    check("Punkt umbenannt", "Greifen" in gui.program_file.punkte)
    check("Schritt zeigt auf den neuen Namen",
          zeilen(gui.program_steps)[0][2] == "Greifen",
          zeilen(gui.program_steps)[0])
    check("Mitgezogene Schritte gemeldet",
          any("mitgezogen" in t for _l, t in protokoll[-2:]), protokoll[-2:])

    # -- Loeschen mit Verweis ----------------------------------------------
    gui.program_points.selection_set(gui.program_points.get_children()[0])
    gui._program_delete()
    check("Benutzter Punkt wird nicht geloescht",
          "Greifen" in gui.program_file.punkte)
    check("Grund nennt den Schritt",
          any("wird in Schritt" in t for _l, t in protokoll[-1:]), protokoll[-1])

    # -- Rueckgaengig ------------------------------------------------------
    vorher = len(gui.program_file.programm)
    gui._program_undo()
    gui.update()
    check("Rueckgaengig nimmt das Umbenennen zurueck",
          "Aufnahme" in gui.program_file.punkte,
          gui.program_file.punkte.namen())
    check("Rueckgaengig stellt auch die Schritte her",
          zeilen(gui.program_steps)[0][2] == "Aufnahme",
          zeilen(gui.program_steps)[0])
    check("Schrittzahl unveraendert",
          len(gui.program_file.programm) == vorher)

    # -- Anfahren geht durch das Fahrfenster --------------------------------
    gui.program_points.selection_set(gui.program_points.get_children()[0])
    gui._program_goto()
    gui.update()
    check("Fahrfenster offen", gui.drive_window is not None
          and gui.drive_window.winfo_exists())
    check("Zielraum steht auf Achsraum",
          gui.drive_space_var.get() == m.SPACE_JOINT)
    ziel = gui.drive_entries["achse2"].get()
    check("Achswert im Zielfeld des Fahrfensters",
          abs(float(ziel) - 30.0) < 0.01, ziel)
    check("Anfahren faehrt nicht von selbst",
          gui.state == m.STATE_READY, gui.state)
    check("Planen wurde versucht und scheitert ohne move_group",
          any("move_group" in t for _l, t in protokoll[-2:]), protokoll[-2:])

    # -- Wiederholungen ----------------------------------------------------
    gui.program_repeat_var.set("20")
    gui._program_apply_repeat()
    check("Wiederholungen uebernommen",
          gui.program_file.programm.wiederholungen == 20)
    gui.program_repeat_var.set("0")
    gui._program_apply_repeat()
    check("Null Wiederholungen abgelehnt",
          gui.program_file.programm.wiederholungen == 20)
    gui.program_repeat_var.set("2")
    gui._program_apply_repeat()

    # -- Speichern ---------------------------------------------------------
    pfad = os.path.join(ordner, "ablauf.csv")
    antworten["datei_ab"] = pfad
    gui._program_save_as()
    gui.update()
    check("Datei geschrieben", os.path.isfile(pfad))
    check("Kein Stern nach dem Speichern", not gui.program_file.geaendert)
    check("Titel ohne Stern", "*" not in gui.program_window.title(),
          gui.program_window.title())
    text = open(pfad).read()
    check("Modellpfad relativ mitgeschrieben",
          "pruefarm.urdf" in text, text[:200])
    check("Signatur mitgeschrieben", "achse;achse1;revolute" in text)
    check("Lage mitgeschrieben", "\nlage;Aufnahme" in text)

    # -- Laden mit passendem Modell ----------------------------------------
    antworten["datei_auf"] = pfad
    gui._program_new()
    check("Neues Programm ist leer", gui.program_file.leer)
    gui._program_load()
    gui.update()
    check("Wieder zwei Punkte", len(gui.program_file.punkte) == 2)
    check("Wieder drei Schritte", len(gui.program_file.programm) == 3)
    check("Wiederholungen gelesen",
          gui.program_file.programm.wiederholungen == 2)
    check("Passendes Modell wird gemeldet",
          any("Modell passt" in t for _l, t in protokoll), protokoll[-3:])
    check("Kein Stern nach dem Laden", not gui.program_file.geaendert)

    # Bei gleicher Signatur ist ein offener Stern am Modell kein Grund zu
    # sperren: es steht kein Wechsel an. Gesperrt wird erst, wenn die Datei
    # ein anderes Modell verlangt und dabei Arbeit verloren ginge.
    gui._set_dirty(True)
    gui._program_load()
    check("Gleiches Modell laedt trotz Stern am Modell",
          any("Modell passt" in t for _l, t in protokoll[-4:]), protokoll[-2:])
    gui._set_dirty(False)

    # -- Laden mit anderem Modell, Antwort "nur Programm" -------------------
    gui._load(urdf2)
    gui.update()

    gui._set_dirty(True)
    gui._program_load()
    check("Offener Stern am Modell sperrt den Wechsel",
          any("ungespeicherte Aenderungen" in t for _l, t in protokoll[-2:]),
          protokoll[-1])
    check("Programm dabei nicht angetastet",
          len(gui.program_file.punkte) == 2, len(gui.program_file.punkte))
    gui._set_dirty(False)
    check("Punkte nach Modellwechsel geprueft",
          any("passen nicht mehr zum Modell" in t for _l, t in protokoll[-4:]),
          protokoll[-4:])
    ungueltig = [z for z in zeilen(gui.program_points) if z[3] != "gueltig"]
    check("Ungueltiger Punkt steht rot in der Tabelle", len(ungueltig) >= 1,
          zeilen(gui.program_points))
    check("Grund nennt die fehlende Achse oder die Grenze",
          any("achse3" in z[3] or "ausserhalb" in z[3] for z in ungueltig),
          [z[3] for z in ungueltig])
    check("Der Wert wurde nicht geklemmt",
          abs(gui.program_file.punkte.get("Aufnahme").werte["achse2"]
              - math.radians(30.0)) < 1e-12)

    # Die wichtigste Sperre im ganzen Fenster: ein Punkt, der nicht mehr zum
    # Modell passt, darf nicht ins Fahrfenster. Ohne sie stuende ein
    # veraltetes Ziel in denselben Feldern wie ein geprueftes, und der
    # naechste Druck auf Fahrt starten faehrt dorthin.
    gui.drive_entries.get("achse2") and gui.drive_entries["achse2"].delete(0, "end")
    gui.program_points.selection_set(
        [k for k in gui.program_points.get_children()
         if gui.program_points.item(k, "values")[3] != "gueltig"][0])
    stand = len(protokoll)
    gui._program_goto()
    gui.update()
    check("Ungueltiger Punkt wird nicht ins Fahrfenster uebernommen",
          any("passt nicht zum Modell" in t
              for _l, t in protokoll[stand:]), protokoll[stand:])
    check("Und es steht kein Ziel in den Zielfeldern",
          not gui.drive_entries.get("achse2")
          or gui.drive_entries["achse2"].get() == "",
          gui.drive_entries.get("achse2")
          and gui.drive_entries["achse2"].get())

    antworten["janeinabbruch"] = False              # nur das Programm laden
    gui._program_load()
    gui.update()
    check("Nur-Programm-Laden laesst das Modell stehen",
          gui.robot_name == "pruefarm" and len(gui.joints) == 4,
          (gui.robot_name, len(gui.joints)))
    check("Hinweis auf die roten Punkte",
          any("stehen rot" in t for _l, t in protokoll[-3:]), protokoll[-3:])

    # -- Laden mit anderem Modell, Antwort "abbrechen" ----------------------
    antworten["janeinabbruch"] = None
    vorher = len(gui.program_file.punkte)
    gui._program_load()
    check("Abbrechen laedt nichts",
          any("nicht geoeffnet" in t for _l, t in protokoll[-1:]),
          protokoll[-1])
    check("Programm unveraendert", len(gui.program_file.punkte) == vorher)

    # -- Laden mit anderem Modell, Antwort "wechseln" -----------------------
    antworten["janeinabbruch"] = True
    gui._program_load()
    gui.update()
    check("Modellwechsel laedt die URDF aus der Datei",
          len(gui.joints) == 3, len(gui.joints))
    check("Punkte danach wieder gueltig",
          all(z[3] == "gueltig" for z in zeilen(gui.program_points)),
          [z[3] for z in zeilen(gui.program_points)])

    # -- Startsperre -------------------------------------------------------
    gruende = gui.program_file.programm.startbar(gui.program_file.punkte,
                                                 gui._program_signature())
    check("Programm ist jetzt startbar", gruende == [], gruende)

    # -- Sperren waehrend der Fahrt -----------------------------------------
    gui._set_state(m.STATE_MOVING, "Test")
    gui.update()
    zustaende = [str(b.cget("state")) for b in gui.program_buttons
                 if b.winfo_exists()]
    check("Programmknoepfe waehrend der Fahrt gesperrt",
          zustaende and all(z == "disabled" for z in zustaende), zustaende)
    gui._program_load()
    check("Oeffnen waehrend der Fahrt gesperrt",
          any("beschaeftigt" in t for _l, t in protokoll[-2:]), protokoll[-1])
    gui._set_state(m.STATE_READY)
    gui.update()
    check("Nach der Fahrt wieder frei",
          all(str(b.cget("state")) == "normal" for b in gui.program_buttons
              if b.winfo_exists()))

    # -- Fenster schliessen und wieder oeffnen ------------------------------
    gui._close_program()
    check("Fenster zu", not gui._program_alive())
    gui._open_program()
    gui.update()
    check("Wieder offen mit denselben Punkten",
          gui._program_alive() and len(zeilen(gui.program_points)) == 2,
          len(zeilen(gui.program_points)))

    gui.destroy()

    # -- Ausgabe ------------------------------------------------------------
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
