#!/usr/bin/env python3
"""Prueft programm.py. Ohne Fenster, ohne ROS, ohne URDF-Datei.

Das Modell kommt als Gelenkliste in derselben Form, die parse_root in der
Oberflaeche liefert -- aufgebaut von Hand, damit sich ein Modellwechsel
durchspielen laesst, ohne dass je eine Datei gelesen wird.
"""

import math
import os
import sys
import tempfile

import programm as P

CHECKS = []


def check(label, bedingung, detail=""):
    CHECKS.append((label, bool(bedingung), detail))


def erwarte_fehler(label, funktion, teil=""):
    """Prueft, dass eine Handlung mit ValueError abgelehnt wird."""
    try:
        funktion()
    except ValueError as fehler:
        if teil and teil not in str(fehler):
            CHECKS.append((label, False, f"Grund war: {fehler}"))
        else:
            CHECKS.append((label, True, ""))
        return
    CHECKS.append((label, False, "kein Fehler ausgeloest"))


# ==========================================================================
# Modell von Hand
# ==========================================================================

def gelenk(name, typ="revolute", achse=(0.0, 0.0, 1.0), untere=-3.14, obere=3.14,
           grenze=True, mimic=None):
    return {"name": name, "type": typ, "axis": list(achse),
            "lower": untere, "upper": obere, "has_limit": grenze,
            "mimic": mimic, "parent": "a", "child": "b"}


def modell():
    return [
        gelenk("joint1"),
        gelenk("joint2", untere=-1.5, obere=1.5),
        gelenk("joint3", typ="prismatic", achse=(1.0, 0.0, 0.0),
               untere=0.0, obere=0.2),
        gelenk("flansch", typ="fixed"),
        gelenk("joint4", mimic={"joint": "joint2", "multiplier": 0.5,
                                "offset": 0.01}),
    ]


def stellung(**werte):
    grund = {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0, "joint4": 0.0}
    grund.update(werte)
    return grund


LAGE = ((0.4123, -0.0881, 0.2357), (0.0, math.pi / 2.0, 0.0))


# ==========================================================================

def main():
    sig = P.Modellsignatur.aus_gelenken("pruefarm", modell())

    # -- Signatur ----------------------------------------------------------
    check("Feste Gelenke stehen nicht in der Signatur",
          "flansch" not in sig.nach_name(), sig.nach_name().keys())
    check("Signatur hat vier bewegliche Achsen", len(sig) == 4, len(sig))
    check("Gekoppelte Achse ist nicht befehlbar",
          sig.frei == ["joint1", "joint2", "joint3"], sig.frei)

    # -- Teachen -----------------------------------------------------------
    datei = P.Programmdatei()
    datei.signatur = sig
    punkte, prog = datei.punkte, datei.programm

    aufnahme = punkte.teachen("Aufnahme",
                              stellung(joint2=math.radians(30.0), joint3=0.05),
                              sig, LAGE)
    check("Punkt speichert nur befehlbare Achsen",
          list(aufnahme.werte) == ["joint1", "joint2", "joint3"],
          list(aufnahme.werte))
    check("Gekoppelte Achse nicht mitgespeichert",
          "joint4" not in aufnahme.werte)
    check("Wert in rad abgelegt",
          abs(aufnahme.werte["joint2"] - math.radians(30.0)) < 1e-15,
          aufnahme.werte["joint2"])
    check("Lage mitgeteacht", aufnahme.lage == LAGE, aufnahme.lage)

    # Der Umweg ueber stand() und _aus_text ist der Weg, den jedes
    # Rueckgaengig geht. Verliert er Stellen, verliert sie auch die Historie
    # -- und zwar unbemerkt, weil danach beide Seiten jedes spaeteren
    # Vergleichs gleich gerundet sind. Deshalb wird hier gegen den
    # Ursprungswert geprueft und nicht gegen einen, der schon durch die
    # Wandlung gelaufen ist.
    genau = math.radians(31.5)
    datei.punkte.teachen("Genau", stellung(joint2=genau), sig)
    datei._aus_text(datei.stand())
    check("Historie haelt den Achswert bitgleich",
          datei.punkte.get("Genau").werte["joint2"] == genau,
          (genau, datei.punkte.get("Genau").werte["joint2"]))
    datei.punkte.loeschen("Genau")
    punkte, prog = datei.punkte, datei.programm

    erwarte_fehler("Doppelter Name abgelehnt",
                   lambda: punkte.teachen("Aufnahme", stellung(), sig),
                   "schon einen Punkt")
    erwarte_fehler("Leerer Name abgelehnt",
                   lambda: punkte.teachen("   ", stellung(), sig), "leer")
    erwarte_fehler("Name mit Trennzeichen abgelehnt",
                   lambda: punkte.teachen("A;B", stellung(), sig), "';'")
    erwarte_fehler("Name mit Raute abgelehnt",
                   lambda: punkte.teachen("#A", stellung(), sig), "'#'")

    punkte.teachen("Ablage", stellung(joint1=math.radians(20.0)), sig)
    punkte.teachen("Heimat", stellung(), sig)
    check("Vorschlag umgeht vergebene Namen",
          punkte.vorschlag() == "P1", punkte.vorschlag())

    # -- Nachteachen -------------------------------------------------------
    prog.einfuegen(P.Schritt("fahren", "Aufnahme", "PTP", 50))
    prog.einfuegen(P.Schritt("warten", dauer=2.0))
    prog.einfuegen(P.Schritt("fahren", "Ablage", "LIN", 30))
    prog.einfuegen(P.Schritt("fahren", "Aufnahme", "PTP", 80))

    punkte.neu_teachen("Aufnahme", stellung(joint2=math.radians(31.5)), sig)
    check("Nachteachen aendert die Werte",
          abs(punkte.get("Aufnahme").werte["joint2"]
              - math.radians(31.5)) < 1e-15)
    check("Nachteachen laesst Verweise unberuehrt",
          prog.verweise("Aufnahme") == [0, 3], prog.verweise("Aufnahme"))
    check("Nachteachen loescht die alte Lage",
          punkte.get("Aufnahme").lage is None)

    # -- Umbenennen --------------------------------------------------------
    betroffen = punkte.umbenennen("Aufnahme", "Greifen", prog)
    check("Umbenennen zieht zwei Schritte mit", betroffen == 2, betroffen)
    check("Alter Name weg", "Aufnahme" not in punkte)
    check("Schritte zeigen auf den neuen Namen",
          prog.verweise("Greifen") == [0, 3], prog.verweise("Greifen"))
    check("Reihenfolge der Punkte bleibt",
          punkte.namen() == ["Greifen", "Ablage", "Heimat"], punkte.namen())
    erwarte_fehler("Umbenennen auf vergebenen Namen abgelehnt",
                   lambda: punkte.umbenennen("Ablage", "Heimat", prog),
                   "schon einen Punkt")

    # -- Loeschen ----------------------------------------------------------
    erwarte_fehler("Benutzter Punkt wird nicht geloescht",
                   lambda: punkte.loeschen("Greifen", prog), "Schritt 1, 4")
    punkte.loeschen("Heimat", prog)
    check("Unbenutzter Punkt wird geloescht", "Heimat" not in punkte)

    # -- Pruefung gegen unveraendertes Modell ------------------------------
    pruefung = punkte.pruefen(sig)
    check("Alle Punkte gueltig bei gleichem Modell",
          all(not g for g in pruefung.values()), pruefung)
    check("Programm ist startbar",
          prog.startbar(punkte, sig) == [], prog.startbar(punkte, sig))

    # -- Fall 1: Achse fehlt im Modell -------------------------------------
    ohne3 = [j for j in modell() if j["name"] != "joint3"]
    gruende = punkte.pruefen(P.Modellsignatur.aus_gelenken("pruefarm", ohne3))
    check("Fehlende Achse macht Punkt ungueltig",
          any("gibt es im Modell nicht" in g for g in gruende["Greifen"]),
          gruende["Greifen"])

    # -- Fall 2: neue freie Achse ------------------------------------------
    plus = modell() + [gelenk("joint5")]
    gruende = punkte.pruefen(P.Modellsignatur.aus_gelenken("pruefarm", plus))
    check("Neue Achse macht Punkt ungueltig",
          any("fehlt im Punkt" in g for g in gruende["Greifen"]),
          gruende["Greifen"])

    # -- Fall 3: Wert ausserhalb der Grenzen -------------------------------
    eng = modell()
    eng[1]["upper"] = math.radians(20.0)
    gruende = punkte.pruefen(P.Modellsignatur.aus_gelenken("pruefarm", eng))
    check("Wert ausserhalb der Grenze wird gemeldet",
          any("liegt ausserhalb" in g for g in gruende["Greifen"]),
          gruende["Greifen"])
    check("Meldung nennt die Werte in grad",
          any("31.5" in g and "grad" in g for g in gruende["Greifen"]),
          gruende["Greifen"])
    check("Der Wert wird nicht stillschweigend geklemmt",
          abs(punkte.get("Greifen").werte["joint2"]
              - math.radians(31.5)) < 1e-15)

    # -- Fall 4: Achse jetzt gekoppelt -------------------------------------
    koppel = modell()
    koppel[0]["mimic"] = {"joint": "joint2", "multiplier": 1.0, "offset": 0.0}
    gruende = punkte.pruefen(P.Modellsignatur.aus_gelenken("pruefarm", koppel))
    check("Neu gekoppelte Achse macht Punkt ungueltig",
          any("folgt jetzt joint2" in g for g in gruende["Greifen"]),
          gruende["Greifen"])

    # -- Fall 5: Schubgelenk ohne Anschlag ---------------------------------
    offen = modell()
    offen[2]["has_limit"] = False
    gruende = punkte.pruefen(P.Modellsignatur.aus_gelenken("pruefarm", offen))
    check("Ohne Anschlag keine Grenzverletzung",
          not any("ausserhalb" in g for g in gruende["Greifen"]),
          gruende["Greifen"])

    # -- Signaturvergleich -------------------------------------------------
    arten = {u.art for u in sig.vergleichen(
        P.Modellsignatur.aus_gelenken("pruefarm", eng))}
    check("Vergleich erkennt geaenderte Grenze", arten == {"grenze"}, arten)

    gedreht = modell()
    gedreht[0]["axis"] = [0.0, 1.0, 0.0]
    arten = {u.art for u in sig.vergleichen(
        P.Modellsignatur.aus_gelenken("pruefarm", gedreht))}
    check("Vergleich erkennt gedrehte Achse", arten == {"richtung"}, arten)

    unterschiede = sig.vergleichen(P.Modellsignatur.aus_gelenken("pruefarm", plus))
    check("Vergleich erkennt neue Achse",
          [u.art for u in unterschiede] == ["neu"], unterschiede)
    check("Vergleich nennt die Achse beim Namen",
          unterschiede[0].achse == "joint5", unterschiede[0])
    check("Gleiches Modell ergibt keinen Unterschied",
          sig.vergleichen(P.Modellsignatur.aus_gelenken("pruefarm", modell())) == [])

    # -- Lageabweichung ----------------------------------------------------
    punkte.neu_teachen("Ablage", stellung(joint1=math.radians(20.0)), sig, LAGE)
    verschoben = ((LAGE[0][0] + 0.0314, LAGE[0][1], LAGE[0][2]), LAGE[1])
    abstand, winkel = punkte.lage_abweichung("Ablage", verschoben)
    check("Lageabweichung in m gerechnet", abs(abstand - 0.0314) < 1e-9, abstand)
    check("Kein Winkelunterschied", winkel < 1e-12, winkel)

    meldungen = punkte.lage_meldungen({"Ablage": verschoben})
    check("Verschobene Lage wird gemeldet", len(meldungen) == 1, meldungen)
    check("Meldung in Millimetern",
          "31.4 mm" in meldungen[0], meldungen[0])
    check("Unveraenderte Lage wird nicht gemeldet",
          punkte.lage_meldungen({"Ablage": LAGE}) == [])
    check("Punkt ohne Lage liefert keine Abweichung",
          punkte.lage_abweichung("Greifen", LAGE) is None)

    # -- Startsperre bei ungueltigem Punkt ---------------------------------
    gruende = prog.startbar(punkte, P.Modellsignatur.aus_gelenken("pruefarm", eng))
    check("Ungueltiger Punkt sperrt den Start", gruende, gruende)
    check("Sperrmeldung nennt Schritt und Punkt",
          any("Schritt 1 (Greifen)" in g for g in gruende), gruende)

    prog.einfuegen(P.Schritt("fahren", "Weggeworfen"))
    gruende = prog.startbar(punkte, sig)
    check("Offener Verweis sperrt den Start",
          any("gibt es nicht" in g for g in gruende), gruende)
    prog.entfernen(len(prog) - 1)

    # -- Schritte ----------------------------------------------------------
    check("Schritttext fahren",
          prog.schritte[0].text() == "fahren Greifen (PTP, 50 %)",
          prog.schritte[0].text())
    check("Schritttext warten",
          prog.schritte[1].text() == "warten 2 s", prog.schritte[1].text())
    erwarte_fehler("Unbekannte Schrittart abgelehnt",
                   lambda: P.Schritt("springen"), "Schrittart")
    erwarte_fehler("Unbekannte Bewegungsart abgelehnt",
                   lambda: P.Schritt("fahren", "Greifen", "KREIS"),
                   "Bewegungsart")

    prog.verschieben(1, 0)
    check("Verschieben ordnet um",
          [s.art for s in prog] == ["warten", "fahren", "fahren", "fahren"],
          [s.art for s in prog])
    prog.verschieben(0, 1)
    erwarte_fehler("Verschieben ausserhalb abgelehnt",
                   lambda: prog.verschieben(99, 0), "gibt es nicht")

    # -- Historie ----------------------------------------------------------
    vorher = len(punkte)
    datei.aendern("Punkt anlegen",
                  lambda: punkte.teachen("Zwischen", stellung(), sig))
    check("Aenderung wirkt", len(punkte) == vorher + 1, len(punkte))
    check("Historie hat einen Eintrag", len(datei.historie) == 1)

    beschreibung = datei.zurueck()
    check("Rueckgaengig meldet die Beschreibung",
          beschreibung == "Punkt anlegen", beschreibung)
    check("Rueckgaengig nimmt den Punkt zurueck",
          "Zwischen" not in datei.punkte, datei.punkte.namen())
    check("Historie ist wieder leer", not datei.historie)

    datei.aendern("Nichts tun", lambda: None)
    check("Handlung ohne Wirkung kommt nicht in die Historie",
          not datei.historie)

    # Umbenennen beruehrt Punkte und Schritte -- ein Rueckgaengig muss beides
    # zuruecknehmen, sonst zeigten die Schritte ins Leere.
    datei.aendern("Punkt umbenennen",
                  lambda: datei.punkte.umbenennen("Greifen", "Pick",
                                                  datei.programm))
    check("Umbenannt", "Pick" in datei.punkte, datei.punkte.namen())
    datei.zurueck()
    check("Rueckgaengig stellt den Punktnamen her",
          "Greifen" in datei.punkte, datei.punkte.namen())
    check("Rueckgaengig stellt auch die Schritte her",
          datei.programm.verweise("Greifen") == [0, 3],
          datei.programm.verweise("Greifen"))

    # -- Datei schreiben und lesen -----------------------------------------
    datei.programm.name = "Palettieren"
    datei.programm.wiederholungen = 20
    datei.modell_relativ = "./bmKinematic.urdf"
    datei.modell_absolut = "/home/frederik/Display/bmKinematic.urdf"

    pfad = os.path.join(tempfile.mkdtemp(), "palettieren.csv")
    anzahl_punkte, anzahl_schritte = datei.schreiben(pfad)
    check("Schreiben meldet die Anzahlen",
          (anzahl_punkte, anzahl_schritte) == (len(datei.punkte),
                                               len(datei.programm)))
    check("Nach dem Schreiben kein Stern", not datei.geaendert)

    with open(pfad) as handle:
        text = handle.read()
    check("Kommentarzeilen vorhanden", "#erzeugt" in text and "#p_grad" in text)
    check("Einheit steht in der Datei", "\neinheit;rad" in text)
    check("Achswerte als Name=Wert", "joint2=" in text, text[:0])

    gelesen = P.Programmdatei()
    meldungen = gelesen.lesen(pfad)
    check("Lesen ohne Meldung", meldungen == [], meldungen)
    check("Stand identisch nach dem Zurueckschreiben",
          gelesen.stand() == datei.stand())
    check("Programmname gelesen", gelesen.programm.name == "Palettieren")
    check("Wiederholungen gelesen", gelesen.programm.wiederholungen == 20)
    check("Modellpfade gelesen",
          gelesen.modell_relativ == "./bmKinematic.urdf"
          and gelesen.modell_absolut.endswith("bmKinematic.urdf"))
    check("Signatur gelesen",
          gelesen.signatur == datei.signatur, len(gelesen.signatur))

    # Verglichen wird gegen math.radians(31.5) und nicht gegen den Wert im
    # Speicher: der ist beim Rueckgaengig weiter oben schon einmal durch
    # stand() gelaufen. Haette die Wandlung dort Stellen verloren, waeren
    # beide Seiten gleich falsch und die Pruefung bliebe gruen.
    zurueck = gelesen.punkte.get("Greifen").werte["joint2"]
    check("Achswert kommt bitgleich aus der Datei",
          zurueck == math.radians(31.5), (math.radians(31.5), zurueck))
    check("Lage kommt zurueck",
          gelesen.punkte.get("Ablage").lage == datei.punkte.get("Ablage").lage)
    check("Schritte kommen zurueck",
          [s.text() for s in gelesen.programm]
          == [s.text() for s in datei.programm])
    check("Frisch gelesene Datei hat keinen Stern", not gelesen.geaendert)

    gelesen.punkte.neu_teachen("Ablage", stellung(joint1=1.0), sig)
    check("Aenderung setzt den Stern", gelesen.geaendert)

    # -- Datei mit kaputter und unbekannter Zeile --------------------------
    krumm = os.path.join(os.path.dirname(pfad), "krumm.csv")
    with open(krumm, "w") as handle:
        handle.write("programm;Krumm\n")
        handle.write("punkt;Gut;joint1=0.0;joint2=0.5;joint3=0.0;\n")
        handle.write("punkt;Kaputt;joint1=xyz\n")
        handle.write("schleife;3\n")
        handle.write("schritt;fahren;Gut;PTP;50;0.0;\n")
    zweite = P.Programmdatei()
    meldungen = zweite.lesen(krumm)
    check("Kaputte Zeile wird gemeldet",
          any("Zeile 3" in m for m in meldungen), meldungen)
    check("Unbekannte Satzart wird gemeldet",
          any("Zeile 4" in m and "schleife" in m for m in meldungen), meldungen)
    check("Der Rest der Datei wird trotzdem gelesen",
          "Gut" in zweite.punkte and len(zweite.programm) == 1,
          (zweite.punkte.namen(), len(zweite.programm)))

    # -- Entscheidung beim Modellwechsel -----------------------------------
    andere = P.Modellsignatur.aus_gelenken("pruefarm", plus)

    def wechsel(**kwargs):
        vorgabe = {"datei_signatur": sig, "jetzige_signatur": sig,
                   "modell_geladen": True, "urdf_geaendert": False,
                   "maschine_beschaeftigt": False}
        vorgabe.update(kwargs)
        return P.modellwechsel(**vorgabe)

    check("Kein Modell geladen: laden",
          wechsel(modell_geladen=False).art == "laden")
    check("Gleiches Modell: passt", wechsel().art == "passt")
    check("Anderes Modell: fragen",
          wechsel(jetzige_signatur=andere).art == "fragen")
    check("Rueckfrage nennt den Unterschied",
          "joint5" in wechsel(jetzige_signatur=andere).text,
          wechsel(jetzige_signatur=andere).text)
    check("Offener Stern am Modell: sperren",
          wechsel(jetzige_signatur=andere, urdf_geaendert=True).art == "sperren")
    check("Maschine beschaeftigt: sperren",
          wechsel(maschine_beschaeftigt=True).art == "sperren")
    check("Beschaeftigt hat Vorrang vor allem anderen",
          wechsel(modell_geladen=False, maschine_beschaeftigt=True).art
          == "sperren")

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
