#!/usr/bin/env python3
"""Pruefungen der Lesesch1cht von bahn_viewer3.py.

Ohne Tkinter und ohne Anzeige: der Datenteil wird aus der Quelle geschnitten
und einzeln ausgefuehrt. So laeuft die Pruefung auch dort, wo kein Display
haengt, und sie bricht nicht ab, wenn sich am Fenster etwas aendert.
"""

import math
import os
import sys
import tempfile

QUELLE = os.environ.get(
    "VIEWER_QUELLE",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "bahn_viewer3.py"))


def lade_datenteil():
    """Nur den Abschnitt von den Helfern bis vor `class Room` ausfuehren."""
    text = open(QUELLE).read()
    kopf = text.index("def clamp(")
    schnitt = text.index("class Room:")
    raum = {"__name__": "bahn_viewer3_daten"}
    exec("import csv, math, os\n" + text[kopf:schnitt], raum)
    return raum


M = lade_datenteil()

FEHLER = []


def pruefe(bedingung, text):
    if bedingung:
        print(f"  ok    {text}")
    else:
        print(f"  FEHLER {text}")
        FEHLER.append(text)


def schreibe(zeilen):
    handle = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False,
                                         newline="")
    handle.write("\r\n".join(zeilen) + "\r\n")
    handle.close()
    return handle.name


# --------------------------------------------------------------------------
# Bausteine fuer kuenstliche Dateien
# --------------------------------------------------------------------------

KOPF_META = ["#roboter;pruefling",
             "#kette;link_1;base_link;joint1",
             "#kette;link_2;link_1;joint2"]


def datei_neu(konvention="ZYX", mit_kennung=True):
    """Datei in neuer Schreibweise, zwei Frames, eine Achse."""
    namen = M["winkelnamen"](konvention)
    meta = list(KOPF_META)
    if mit_kennung:
        meta.insert(1, f"#drehkonvention;{konvention}")
    kopf = (["t_s", "x_m", "y_m", "z_m"] + list(namen) + ["joint1_deg"])
    for frame in ("base_link", "link_1", "link_2"):
        kopf += [f"{frame}_x_m", f"{frame}_y_m", f"{frame}_z_m"]
        kopf += list(M["winkelnamen"](konvention, f"{frame}_"))
    zeilen = list(meta) + [";".join(kopf)]
    for schritt in range(3):
        werte = [f"{schritt * 0.1:.4f}", "1.000", "2.000", "3.000",
                 "10.0", "20.0", "30.0", "5.0"]
        for k in range(3):
            werte += [f"{k:.3f}", "0.000", "0.000", "10.0", "20.0", "30.0"]
        zeilen.append(";".join(werte))
    return schreibe(zeilen)


def datei_alt():
    """Datei in der Schreibweise von Fassung 2: roll/pitch/yaw, keine Kennung."""
    kopf = ["t_s", "x_m", "y_m", "z_m",
            "roll_deg", "pitch_deg", "yaw_deg", "joint1_deg"]
    for frame in ("base_link", "link_1", "link_2"):
        kopf += [f"{frame}_x_m", f"{frame}_y_m", f"{frame}_z_m",
                 f"{frame}_roll_deg", f"{frame}_pitch_deg", f"{frame}_yaw_deg"]
    zeilen = list(KOPF_META) + [";".join(kopf)]
    for schritt in range(3):
        werte = [f"{schritt * 0.1:.4f}", "1.000", "2.000", "3.000",
                 "30.0", "20.0", "10.0", "5.0"]
        for k in range(3):
            werte += [f"{k:.3f}", "0.000", "0.000", "30.0", "20.0", "10.0"]
        zeilen.append(";".join(werte))
    return schreibe(zeilen)


def erwarte_fehler(pfad, text, stichwort):
    try:
        M["Track"].load(pfad)
    except ValueError as exc:
        treffer = stichwort.lower() in str(exc).lower()
        pruefe(treffer, f"{text} -> Abbruch ({str(exc).splitlines()[0][:60]})")
        return
    pruefe(False, f"{text} -> kein Abbruch, aber einer war faellig")


# --------------------------------------------------------------------------
print("Drehmatrix")
# --------------------------------------------------------------------------


def alt_rotation(rpy):
    """Die Formel aus Fassung 2, als unabhaengiger Vergleich."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, sy * cp, -sp),
            (cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr),
            (cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr))


def gleich(a, b, toleranz=1e-12):
    return all(abs(x - y) <= toleranz
               for spalte_a, spalte_b in zip(a, b)
               for x, y in zip(spalte_a, spalte_b))


for probe in ((0.3, -0.7, 1.1), (0.0, math.pi / 2, 0.0), (-2.0, 0.4, 2.9)):
    roll, pitch, yaw = probe
    neu = M["rotation_from_abc"]((yaw, pitch, roll), ("z", "y", "x"))
    pruefe(gleich(neu, alt_rotation(probe)),
           f"ZYX(A,B,C) gleich RPY(roll,pitch,yaw) bei {probe}")

# Orthonormal und rechtshaendig fuer alle drei Konventionen.
for name, achsen in M["KONVENTIONEN"].items():
    spalten = M["rotation_from_abc"]((0.4, 0.9, -1.3), achsen)
    laengen = [math.sqrt(sum(v * v for v in s)) for s in spalten]
    skalar = [sum(a * b for a, b in zip(spalten[i], spalten[j]))
              for i, j in ((0, 1), (0, 2), (1, 2))]
    kreuz = (spalten[0][1] * spalten[1][2] - spalten[0][2] * spalten[1][1],
             spalten[0][2] * spalten[1][0] - spalten[0][0] * spalten[1][2],
             spalten[0][0] * spalten[1][1] - spalten[0][1] * spalten[1][0])
    pruefe(all(abs(l - 1.0) < 1e-12 for l in laengen)
           and all(abs(s) < 1e-12 for s in skalar)
           and all(abs(a - b) < 1e-12 for a, b in zip(kreuz, spalten[2])),
           f"{name}: Spalten orthonormal und rechtshaendig")

# ZYZ und ZXZ duerfen sich nicht wie ZYX verhalten.
abc = (0.4, 0.9, -1.3)
pruefe(not gleich(M["rotation_from_abc"](abc, ("z", "y", "x")),
                  M["rotation_from_abc"](abc, ("z", "y", "z"))),
       "ZYX und ZYZ liefern verschiedene Matrizen")

# --------------------------------------------------------------------------
print("Spaltennamen")
# --------------------------------------------------------------------------

pruefe(M["winkelnamen"]("ZYX") == ("a_z_deg", "b_y_deg", "c_x_deg"),
       "ZYX heisst a_z_deg, b_y_deg, c_x_deg")
pruefe(M["winkelnamen"]("ZYZ") == ("a_z_deg", "b_y_deg", "c_z_deg"),
       "ZYZ heisst a_z_deg, b_y_deg, c_z_deg")
pruefe(M["winkelnamen"]("ZXZ", "link_1_")
       == ("link_1_a_z_deg", "link_1_b_x_deg", "link_1_c_z_deg"),
       "Praefix wird durchgereicht")
namen = [M["winkelnamen"](k) for k in M["KONVENTIONEN"]]
pruefe(len(set(namen)) == len(namen),
       "die drei Konventionen sind an den Spaltennamen unterscheidbar")

# --------------------------------------------------------------------------
print("Neue Dateien")
# --------------------------------------------------------------------------

for name in M["KONVENTIONEN"]:
    pfad = datei_neu(name)
    track = M["Track"].load(pfad)
    pruefe(track.konvention == name and track.has_model
           and len(track.frame_names) == 3 and track.joint_names == ["joint1_deg"],
           f"{name}: Modell mit 3 Frames und 1 Achse gelesen")
    pruefe(abs(math.degrees(track.angles[0][0]) - 10.0) < 1e-9
           and abs(math.degrees(track.angles[0][2]) - 30.0) < 1e-9,
           f"{name}: A = 10, C = 30 in der Reihenfolge der Datei")
    os.unlink(pfad)

pfad = datei_neu("ZYX", mit_kennung=False)
track = M["Track"].load(pfad)
pruefe(track.konvention == "ZYX" and track.konvention_quelle == "Spaltennamen",
       "ohne Kennung entscheiden die Spaltennamen")
os.unlink(pfad)

# --------------------------------------------------------------------------
print("Alte Dateien")
# --------------------------------------------------------------------------

pfad = datei_alt()
track = M["Track"].load(pfad)
pruefe(track.konvention == "ZYX" and track.has_model
       and len(track.frame_names) == 3,
       "roll/pitch/yaw: Modell wird weiterhin gelesen")
pruefe(abs(math.degrees(track.angles[0][0]) - 10.0) < 1e-9
       and abs(math.degrees(track.angles[0][1]) - 20.0) < 1e-9
       and abs(math.degrees(track.angles[0][2]) - 30.0) < 1e-9,
       "A = yaw, B = pitch, C = roll beim Umsetzen")
pruefe(track.joint_names == ["joint1_deg"],
       "keine Frame-Spalten in der Achsenliste")
_ort, abc = track.frame_pose(0, 1)
pruefe(abs(math.degrees(abc[0]) - 10.0) < 1e-9,
       "auch die Frameblocks werden umgesetzt")
os.unlink(pfad)

# --------------------------------------------------------------------------
print("Widersprueche werden gemeldet")
# --------------------------------------------------------------------------

# Kennung ZYZ, Spalten ZYX.
zeilen = open(datei_neu("ZYX")).read().splitlines()
zeilen = [z.replace("#drehkonvention;ZYX", "#drehkonvention;ZYZ") for z in zeilen]
erwarte_fehler(schreibe(zeilen), "Kennung ZYZ bei ZYX-Spalten", "Kennung")

# Kennung ZYZ, alte Namen.
zeilen = open(datei_alt()).read().splitlines()
zeilen.insert(1, "#drehkonvention;ZYZ")
erwarte_fehler(schreibe(zeilen), "Kennung ZYZ bei roll/pitch/yaw", "ZYX")

# Unbekannte Kennung.
zeilen = open(datei_neu("ZYX")).read().splitlines()
zeilen = [z.replace("#drehkonvention;ZYX", "#drehkonvention;XYZ") for z in zeilen]
erwarte_fehler(schreibe(zeilen), "unbekannte Kennung XYZ", "Unbekannte")

# Frame ohne Winkelspalten.
zeilen = open(datei_neu("ZYX")).read().splitlines()
kopf = zeilen[4].split(";")
weg = [i for i, n in enumerate(kopf) if n.startswith("link_1_")
       and n.endswith("_deg")]
for i in sorted(weg, reverse=True):
    for z, zeile in enumerate(zeilen[4:], start=4):
        teile = zeile.split(";")
        del teile[i]
        zeilen[z] = ";".join(teile)
erwarte_fehler(schreibe(zeilen), "Frame link_1 ohne Winkelspalten", "link_1")

# Halbe alte Namen.
zeilen = open(datei_alt()).read().splitlines()
zeilen[3] = zeilen[3].replace("pitch_deg;", "", 1)
for z in range(4, len(zeilen)):
    teile = zeilen[z].split(";")
    del teile[5]
    zeilen[z] = ";".join(teile)
erwarte_fehler(schreibe(zeilen), "roll und yaw ohne pitch", "pitch_deg")

# Beide Schreibweisen im selben Kopf.
zeilen = open(datei_neu("ZYX")).read().splitlines()
zeilen[4] = zeilen[4].replace("joint1_deg", "roll_deg;pitch_deg;yaw_deg", 1)
for z in range(5, len(zeilen)):
    zeilen[z] = zeilen[z].replace(";5.0;", ";1.0;2.0;3.0;", 1)
erwarte_fehler(schreibe(zeilen), "beide Schreibweisen im Kopf", "beide")

# --------------------------------------------------------------------------
print("Bahn ohne Orientierung")
# --------------------------------------------------------------------------

zeilen = ["#roboter;flach", "t_s;x_m;y_m;z_m",
          "0.0;1.0;2.0;3.0", "0.1;1.1;2.0;3.0"]
track = M["Track"].load(schreibe(zeilen))
pruefe(not track.angles_known and track.konvention == "ZYX",
       "ohne Winkelspalten wird das gemeldet statt null angezeigt")

# --------------------------------------------------------------------------
print("Echte Aufzeichnungen")
# --------------------------------------------------------------------------

for pfad, erwartet in (
        ("/mnt/user-data/uploads/bahn_20260827_110053.csv", "alte Namen"),
        ("/mnt/user-data/uploads/bahn_20260902_064034.csv", "Kennung")):
    if not os.path.exists(pfad):
        continue
    track = M["Track"].load(pfad)
    pruefe(track.has_model and len(track.frame_names) == 12
           and len(track.joint_names) == 7,
           f"{os.path.basename(pfad)}: 12 Frames, 7 Achsen, Modell da")
    pruefe(track.konvention == "ZYX" and erwartet in track.konvention_quelle,
           f"{os.path.basename(pfad)}: ZYX aus {track.konvention_quelle}")

print()
if FEHLER:
    print(f"{len(FEHLER)} Pruefung(en) fehlgeschlagen")
    sys.exit(1)
print("alle Pruefungen bestanden")
