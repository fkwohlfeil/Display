#!/usr/bin/env python3
"""Prueft die Programmebene aus programm.py.

Die Bewegungsebene ist eine Attrappe: sie zaehlt die Auftraege, laesst sich
schrittweise weiterstellen und kann auf Verlangen scheitern. Damit ist der
ganze Ablauf durchspielbar, ohne dass move_group laeuft -- und ohne dass ein
Test drei Sekunden auf eine Bahn wartet.

Die Uhr wird ebenfalls gestellt. Ein Warteschritt von zwei Sekunden darf
einen Test nicht zwei Sekunden kosten, und eine echte Uhr macht den Ausgang
vom Zufall des Taktes abhaengig.
"""

import sys

import programm as P

CHECKS = []


def check(label, bedingung, detail=""):
    CHECKS.append((label, bool(bedingung), detail))


# ==========================================================================
# Attrappe
# ==========================================================================

class Maschine:
    """Bewegungsebene, wie die Programmebene sie sieht -- und sonst nichts.

    `zustand` ist "bereit" oder "beschaeftigt". Weitergestellt wird von Hand:
    der Test bestimmt, wann eine Planung durch ist und wann eine Fahrt endet.
    """

    def __init__(self):
        self.zustand = "bereit"
        self.plan = None
        self.plan_auftraege = []                       # (werte, bewegung, tempo)
        self.fahr_auftraege = 0
        self.abbrueche = 0
        self.plan_antwort = P.AUFTRAG_LAEUFT
        self.fahr_antwort = P.AUFTRAG_LAEUFT
        self.plan_glueckt = True

    # -- Die fuenf Fragen --------------------------------------------------

    def bereit(self):
        return self.zustand == "bereit"

    def planen(self, werte, bewegung, tempo):
        self.plan_auftraege.append((dict(werte), bewegung, tempo))
        if self.plan_antwort == P.AUFTRAG_LAEUFT:
            self.zustand = "beschaeftigt"
        return self.plan_antwort

    def plan_vorhanden(self):
        return self.plan is not None

    def fahren(self):
        self.fahr_auftraege += 1
        if self.fahr_antwort == P.AUFTRAG_LAEUFT:
            self.zustand = "beschaeftigt"
            self.plan = None
        return self.fahr_antwort

    def abbrechen(self):
        self.abbrueche += 1
        self.plan = None

    # -- Vom Test gestellt --------------------------------------------------

    def planung_fertig(self):
        self.plan = "bahn" if self.plan_glueckt else None
        self.zustand = "bereit"

    def fahrt_fertig(self):
        self.zustand = "bereit"


class Uhr:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def weiter(self, sekunden):
        self.t += sekunden


# ==========================================================================
# Modell und Programm von Hand
# ==========================================================================

def gelenk(name):
    return {"name": name, "type": "revolute", "axis": [0.0, 0.0, 1.0],
            "lower": -3.14, "upper": 3.14, "has_limit": True, "mimic": None,
            "parent": "a", "child": "b"}


def aufbau(mit_warten=True):
    sig = P.Modellsignatur.aus_gelenken("pruefarm",
                                        [gelenk("j1"), gelenk("j2")])
    datei = P.Programmdatei()
    datei.signatur = sig
    datei.punkte.teachen("A", {"j1": 0.0, "j2": 0.0}, sig)
    datei.punkte.teachen("B", {"j1": 1.0, "j2": 0.5}, sig)
    datei.programm.einfuegen(P.Schritt("fahren", "A", "PTP", 50))
    if mit_warten:
        datei.programm.einfuegen(P.Schritt("warten", dauer=2.0))
    datei.programm.einfuegen(P.Schritt("fahren", "B", "LIN", 30))
    return datei, sig


def fahrschritt(ablauf, maschine):
    """Einen Fahrschritt vollstaendig durchtakten."""
    ablauf.takt()                                      # planen
    maschine.planung_fertig()
    ablauf.takt()                                      # fahren
    maschine.fahrt_fertig()
    ablauf.takt()                                      # angekommen


# ==========================================================================

def main():
    datei, sig = aufbau()
    maschine, uhr = Maschine(), Uhr()
    ablauf = P.Ablauf(maschine, jetzt=uhr)

    # -- Ausgangslage ------------------------------------------------------
    check("Anfangs leer", ablauf.zustand == P.PROG_LEER, ablauf.zustand)
    check("Leer ist nicht aktiv", not ablauf.aktiv)
    check("Takt ohne Programm tut nichts", ablauf.takt() is False)
    check("Abbruch ohne Programm tut nichts", ablauf.abbrechen("x") is False)

    # -- Start mit ungueltigem Programm ------------------------------------
    kaputt = P.Programmdatei()
    kaputt.signatur = sig
    gruende = ablauf.starten(kaputt, sig)
    check("Leeres Programm startet nicht", gruende, gruende)
    check("Zustand bleibt leer", ablauf.zustand == P.PROG_LEER)

    # -- Einzelschritt ueber den ersten Fahrschritt -------------------------
    check("Start ohne Gruende", ablauf.starten(datei, sig, einzeln=True) == [])
    check("Zustand laeuft", ablauf.zustand == P.PROG_LAEUFT)
    check("Aktiv sperrt das Modell", ablauf.aktiv)
    check("Zeiger auf Schritt 1", ablauf.schritt == 0)

    geaendert = ablauf.takt()
    check("Erster Takt legt einen Planauftrag hinein",
          geaendert and len(maschine.plan_auftraege) == 1,
          maschine.plan_auftraege)
    check("Auftrag traegt die Achswerte des Punkts",
          maschine.plan_auftraege[0][0] == {"j1": 0.0, "j2": 0.0},
          maschine.plan_auftraege[0])
    check("Auftrag traegt Bewegungsart und Tempo",
          maschine.plan_auftraege[0][1:] == ("PTP", 50),
          maschine.plan_auftraege[0])

    # Der Kern: zehn Takte ohne Zustandswechsel duerfen genau nichts tun.
    for _ in range(10):
        ablauf.takt()
    check("Zehn Takte erzeugen keinen zweiten Auftrag",
          len(maschine.plan_auftraege) == 1, maschine.plan_auftraege)
    check("Und keine Fahrt", maschine.fahr_auftraege == 0)

    maschine.planung_fertig()
    ablauf.takt()
    check("Nach der Planung wird gefahren", maschine.fahr_auftraege == 1)
    for _ in range(10):
        ablauf.takt()
    check("Zehn Takte erzeugen keine zweite Fahrt",
          maschine.fahr_auftraege == 1, maschine.fahr_auftraege)

    maschine.fahrt_fertig()
    ablauf.takt()
    check("Schritt fertig, Zeiger auf Schritt 2", ablauf.schritt == 1,
          ablauf.schritt)
    check("Einzelschritt haelt an", ablauf.zustand == P.PROG_PAUSE,
          ablauf.zustand)
    check("Pause ist weiterhin aktiv", ablauf.aktiv)
    check("In der Pause tut der Takt nichts", ablauf.takt() is False)

    arten = [a for a, _t in ablauf.meldungen]
    check("Fertigmeldung abgesetzt",
          any("Schritt 1 fertig" in t for _a, t in ablauf.meldungen),
          ablauf.meldungen)
    check("Meldungen lassen sich abholen",
          ablauf.abholen() and ablauf.meldungen == [])

    # -- Einzelschritt ueber den Warteschritt --------------------------------
    check("Fortsetzen ohne Gruende",
          ablauf.starten(datei, sig, einzeln=True) == [])
    ablauf.takt()
    check("Wartefrist gesetzt", abs(ablauf.rest() - 2.0) < 1e-9, ablauf.rest())
    check("Warten legt keinen Auftrag hinein",
          len(maschine.plan_auftraege) == 1)

    uhr.weiter(1.0)
    ablauf.takt()
    check("Nach der Haelfte laeuft der Warteschritt noch",
          ablauf.zustand == P.PROG_LAEUFT and ablauf.schritt == 1,
          (ablauf.zustand, ablauf.schritt))
    check("Restzeit stimmt", abs(ablauf.rest() - 1.0) < 1e-9, ablauf.rest())

    uhr.weiter(1.0)
    ablauf.takt()
    check("Warteschritt zu Ende", ablauf.schritt == 2, ablauf.schritt)
    check("Danach Pause", ablauf.zustand == P.PROG_PAUSE)
    check("Restzeit wieder null", ablauf.rest() == 0.0)

    # -- Letzter Schritt beendet das Programm -------------------------------
    ablauf.starten(datei, sig, einzeln=True)
    fahrschritt(ablauf, maschine)
    check("Nach dem letzten Schritt bereit",
          ablauf.zustand == P.PROG_BEREIT, ablauf.zustand)
    check("Bereit ist nicht aktiv", not ablauf.aktiv)
    check("Zeiger zurueck auf Schritt 1", ablauf.schritt == 0)
    check("Ende gemeldet",
          any("Programm zu Ende" in t for _a, t in ablauf.meldungen),
          ablauf.meldungen)
    check("Zweiter Punkt wurde angefahren",
          maschine.plan_auftraege[-1][0] == {"j1": 1.0, "j2": 0.5},
          maschine.plan_auftraege[-1])
    check("Bewegungsart des zweiten Schritts",
          maschine.plan_auftraege[-1][1:] == ("LIN", 30),
          maschine.plan_auftraege[-1])
    ablauf.abholen()

    # -- Durchlauf ohne Einzelschritt ---------------------------------------
    # einzeln=False ist die Grundlage fuer den Dauerbetrieb spaeter. Hier
    # wird nur geprueft, dass nach einem Schritt nicht angehalten wird.
    maschine2, uhr2 = Maschine(), Uhr()
    ablauf2 = P.Ablauf(maschine2, jetzt=uhr2)
    datei2, sig2 = aufbau(mit_warten=False)
    ablauf2.starten(datei2, sig2, einzeln=False)
    fahrschritt(ablauf2, maschine2)
    check("Ohne Einzelschritt laeuft es weiter",
          ablauf2.zustand == P.PROG_LAEUFT and ablauf2.schritt == 1,
          (ablauf2.zustand, ablauf2.schritt))
    fahrschritt(ablauf2, maschine2)
    check("Und endet nach dem letzten Schritt",
          ablauf2.zustand == P.PROG_BEREIT, ablauf2.zustand)

    # -- Planung scheitert ---------------------------------------------------
    maschine3 = Maschine()
    ablauf3 = P.Ablauf(maschine3, jetzt=Uhr())
    datei3, sig3 = aufbau(mit_warten=False)
    maschine3.plan_glueckt = False
    ablauf3.starten(datei3, sig3, einzeln=False)
    ablauf3.takt()
    maschine3.planung_fertig()                         # ohne Plan
    ablauf3.takt()
    check("Gescheiterte Planung fuehrt nach PROG_FEHLER",
          ablauf3.zustand == P.PROG_FEHLER, ablauf3.zustand)
    check("Grund nennt den Schritt", "Schritt 1" in ablauf3.grund,
          ablauf3.grund)
    check("Es wurde nicht gefahren", maschine3.fahr_auftraege == 0)
    check("Die Bewegungsebene wurde geraeumt", maschine3.abbrueche == 1)
    check("Im Fehler tut der Takt nichts", ablauf3.takt() is False)
    check("Fehler ist nicht aktiv", not ablauf3.aktiv)
    check("Text nennt den Fehler", P.PROG_FEHLER in ablauf3.text(),
          ablauf3.text())

    check("Abbruch quittiert den Fehler", ablauf3.abbrechen("quittiert"))
    check("Danach bereit", ablauf3.zustand == P.PROG_BEREIT)

    # -- Auftrag wird gar nicht erst angenommen ------------------------------
    maschine4 = Maschine()
    ablauf4 = P.Ablauf(maschine4, jetzt=Uhr())
    datei4, sig4 = aufbau(mit_warten=False)
    maschine4.plan_antwort = P.AUFTRAG_FEHLER
    ablauf4.starten(datei4, sig4)
    ablauf4.takt()
    check("Abgelehnter Planauftrag fuehrt nach PROG_FEHLER",
          ablauf4.zustand == P.PROG_FEHLER, ablauf4.zustand)

    # -- Ziel schon erreicht -------------------------------------------------
    maschine5 = Maschine()
    ablauf5 = P.Ablauf(maschine5, jetzt=Uhr())
    datei5, sig5 = aufbau(mit_warten=False)
    maschine5.plan_antwort = P.AUFTRAG_ERLEDIGT
    ablauf5.starten(datei5, sig5, einzeln=True)
    ablauf5.takt()
    check("Schon erreichtes Ziel gilt als erledigt",
          ablauf5.schritt == 1 and ablauf5.zustand == P.PROG_PAUSE,
          (ablauf5.schritt, ablauf5.zustand))
    check("Ohne Fahrauftrag", maschine5.fahr_auftraege == 0)
    check("Und mit Hinweis",
          any("schon erreicht" in t for _a, t in ablauf5.meldungen),
          ablauf5.meldungen)

    # -- Abbruch mitten in der Fahrt -----------------------------------------
    maschine6 = Maschine()
    ablauf6 = P.Ablauf(maschine6, jetzt=Uhr())
    datei6, sig6 = aufbau()
    ablauf6.starten(datei6, sig6, einzeln=False)
    ablauf6.takt()
    maschine6.planung_fertig()
    ablauf6.takt()                                     # faehrt jetzt
    check("Abbruch waehrend der Fahrt", ablauf6.abbrechen("Bediener"))
    check("Zustand bereit", ablauf6.zustand == P.PROG_BEREIT)
    check("Zeiger auf Anfang", ablauf6.schritt == 0)
    check("Bewegungsebene abgebrochen", maschine6.abbrueche == 1)
    check("Abbruch nennt Schritt und Grund",
          any("Schritt 1" in t and "Bediener" in t
              for _a, t in ablauf6.meldungen), ablauf6.meldungen)
    maschine6.fahrt_fertig()
    check("Nach dem Abbruch taktet nichts nach", ablauf6.takt() is False)
    check("Und es geht kein Auftrag mehr heraus",
          len(maschine6.plan_auftraege) == 1)

    # -- Schritt verschwindet waehrend des Laufs ------------------------------
    maschine7 = Maschine()
    ablauf7 = P.Ablauf(maschine7, jetzt=Uhr())
    datei7, sig7 = aufbau(mit_warten=False)
    ablauf7.starten(datei7, sig7, einzeln=False)
    fahrschritt(ablauf7, maschine7)
    datei7.programm.entfernen(1)                       # der naechste Schritt
    ablauf7.takt()
    check("Verschwundener Schritt fuehrt nach PROG_FEHLER",
          ablauf7.zustand == P.PROG_FEHLER, ablauf7.zustand)

    # -- Punkt verschwindet waehrend des Laufs --------------------------------
    maschine8 = Maschine()
    ablauf8 = P.Ablauf(maschine8, jetzt=Uhr())
    datei8, sig8 = aufbau(mit_warten=False)
    ablauf8.starten(datei8, sig8, einzeln=False)
    fahrschritt(ablauf8, maschine8)
    del datei8.punkte.punkte["B"]
    ablauf8.takt()
    check("Verschwundener Punkt fuehrt nach PROG_FEHLER",
          ablauf8.zustand == P.PROG_FEHLER, ablauf8.zustand)
    check("Grund nennt den Punkt", "'B'" in ablauf8.grund, ablauf8.grund)

    # -- Doppelter Start ------------------------------------------------------
    maschine9 = Maschine()
    ablauf9 = P.Ablauf(maschine9, jetzt=Uhr())
    datei9, sig9 = aufbau(mit_warten=False)
    ablauf9.starten(datei9, sig9)
    ablauf9.takt()
    gruende = ablauf9.starten(datei9, sig9)
    check("Zweiter Start waehrend des Laufs abgelehnt", gruende, gruende)
    check("Kein zweiter Auftrag durch den zweiten Start",
          len(maschine9.plan_auftraege) == 1)

    # -- vergessen ------------------------------------------------------------
    ablauf9.vergessen()
    check("Vergessen setzt auf leer", ablauf9.zustand == P.PROG_LEER)
    check("Vergessen bricht ab", maschine9.abbrueche == 1)
    check("Text ohne Programm", ablauf9.text() == P.PROG_LEER, ablauf9.text())

    # -- Ausgabe -------------------------------------------------------------
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
