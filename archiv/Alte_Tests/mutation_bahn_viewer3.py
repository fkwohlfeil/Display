#!/usr/bin/env python3
"""Mutationssonden zu test_bahn_viewer3.py.

Jede Sonde baut genau einen Schutz aus der Quelle aus und laesst danach die
Pruefungen laufen. Bestehen sie trotzdem, prueft die Sonde nichts -- dann ist
nicht der Schutz gut, sondern der Test blind.
"""

import os
import subprocess
import sys
import tempfile

QUELLE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "bahn_viewer3.py")
TEST = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "test_bahn_viewer3.py")
ORIGINAL = open(QUELLE).read()

SONDEN = [
    ("Kennung und Spaltennamen duerfen sich widersprechen",
     """        if kennung and kennung != name:
            raise ValueError(
                f"Die Kennung im Kopf sagt {kennung}, die Spaltennamen sagen "
                f"{name}.\\nEine der beiden Angaben ist falsch; welche, ist der "
                "Datei nicht zu entnehmen.")""",
     "        pass"),

    ("beide Schreibweisen im Kopf werden hingenommen",
     """    if treffer and alt_vollstaendig:
        raise ValueError(
            "Der Spaltenkopf enthaelt beide Schreibweisen der Winkel "
            f"({', '.join(winkelnamen(treffer[0][0]))} und roll/pitch/yaw).\\n"
            "Welche der beiden gilt, steht nirgends.")""",
     "    pass"),

    ("halbe alte Namen werden hingenommen",
     """    if alt_teilweise:
        fehlen = [gruppe[0] for gruppe, spalte in zip(ALT_NAMEN, alt)
                  if spalte is None]
        raise ValueError("Von den alten Winkelspalten fehlen: "
                         + ", ".join(fehlen))""",
     "    pass"),

    ("unbekannte Kennung wird durchgelassen",
     """        if kennung and kennung not in KONVENTIONEN:
            raise ValueError(
                f"Unbekannte Drehkonvention im Kopf: {kennung}\\n"
                "Bekannt sind: " + ", ".join(KONVENTIONEN))""",
     "        kennung = kennung if kennung in KONVENTIONEN else \"\""),

    ("Frame ohne Winkelspalten wird uebersprungen statt gemeldet",
     """            if treffer is None:
                erwartet = ", ".join(stem + gruppe[0] for gruppe in suffixe)
                raise ValueError(
                    f"Zum Frame '{stem}' fehlen Winkelspalten.\\n"
                    f"Erwartet: {erwartet}")""",
     "            if treffer is None:\n                columns = None\n                break"),

    ("alte Namen in der Reihenfolge roll, pitch, yaw gedeutet",
     """ALT_NAMEN = (("yaw_deg", "yaw", "gier_deg"),           # A
             ("pitch_deg", "pitch", "p_deg"),          # B
             ("roll_deg", "roll", "r_deg"))            # C""",
     """ALT_NAMEN = (("roll_deg", "roll", "r_deg"),
             ("pitch_deg", "pitch", "p_deg"),
             ("yaw_deg", "yaw", "gier_deg"))"""),

    ("Drehung immer nach ZYX statt nach der Kennung",
     """    matrix = matmul3(matmul3(elementary_rotation(achsen[0], abc[0]),
                             elementary_rotation(achsen[1], abc[1])),
                     elementary_rotation(achsen[2], abc[2]))""",
     """    achsen = ("z", "y", "x")
    matrix = matmul3(matmul3(elementary_rotation(achsen[0], abc[0]),
                             elementary_rotation(achsen[1], abc[1])),
                     elementary_rotation(achsen[2], abc[2]))"""),

    ("fehlende Winkelspalten als Nulllage ausgegeben",
     "                   konvention, quelle, winkel_cols is not None)",
     "                   konvention, quelle, True)"),
]


def lauf(quelle_text):
    handle = tempfile.NamedTemporaryFile("w", suffix=".py", delete=False)
    handle.write(quelle_text)
    handle.close()
    umgebung = dict(os.environ, VIEWER_QUELLE=handle.name)
    ergebnis = subprocess.run([sys.executable, TEST], env=umgebung,
                              capture_output=True, text=True)
    os.unlink(handle.name)
    return ergebnis


blind = []
print("Ausgangslage")
if lauf(ORIGINAL).returncode != 0:
    print("  FEHLER die unveraenderte Quelle besteht die Pruefungen nicht")
    sys.exit(1)
print("  ok    unveraendert bestanden\n")

for text, alt, neu in SONDEN:
    if ORIGINAL.count(alt) != 1:
        print(f"  FEHLER Sonde greift nicht: {text}")
        blind.append(text)
        continue
    ergebnis = lauf(ORIGINAL.replace(alt, neu))
    if ergebnis.returncode == 0:
        print(f"  BLIND  {text}")
        blind.append(text)
    else:
        zeilen = [z.strip() for z in ergebnis.stdout.splitlines()
                  if z.strip().startswith("FEHLER")]
        print(f"  ok     {text}")
        for z in zeilen[:2]:
            print(f"           faellt auf: {z[7:][:70]}")

print()
if blind:
    print(f"{len(blind)} Sonde(n) unbemerkt -- der Test prueft das nicht")
    sys.exit(1)
print("alle Sonden werden bemerkt")
