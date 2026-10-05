#!/usr/bin/env python3
"""Mutationsproben fuer intrinsic_folge.py.

    python3 mutation_intrinsic_folge.py

Jede Probe veraendert eine Stelle in einer Kopie von intrinsic_folge.py und
laesst test_intrinsic_folge.py dagegen laufen. Ueberlebt eine Mutation,
prueft der Test die Stelle nicht -- dann ist der Test zu schwach, nicht der
Code gut. Das Original bleibt unberuehrt.
"""

import os
import shutil
import subprocess
import sys
import tempfile

HIER = os.path.dirname(os.path.abspath(__file__))
DATEIEN = ("sequenz.py", "totg_folge.py", "test_intrinsic_folge.py")

PROBEN = (
    ("rundung_rot untere Grenze",
     "if rot is not None and not RUNDUNG_ROT_MIN <= rot <= RUNDUNG_ROT_MAX:",
     "if rot is not None and not RUNDUNG_ROT_MIN < rot <= RUNDUNG_ROT_MAX:"),
    ("rundung_rot obere Grenze",
     "if rot is not None and not RUNDUNG_ROT_MIN <= rot <= RUNDUNG_ROT_MAX:",
     "if rot is not None and not RUNDUNG_ROT_MIN <= rot:"),
    ("resample_dt nicht geprueft",
     "        befunde.append(befund)\n    return tuple(befunde)",
     "        pass\n    return tuple(befunde)"),
    ("Wrapper nicht geprueft",
     "if not (os.path.isfile(wrapper) and os.access(wrapper, os.X_OK)):",
     "if not os.path.exists(wrapper):"),
    ("Tip nicht geprueft",
     "    if not tip:\n",
     "    if False:\n"),
    ("Override gemittelt statt Fehler",
     "    if len(paare) > 1:\n",
     "    if len(paare) > 9:\n"),
    ("CIRC erlaubt",
     'FAHRARTEN = ("LIN", "PTP")',
     'FAHRARTEN = ("LIN", "PTP", "CIRC")'),
    ("Fahrartfehler ohne Abbruch",
     "                stelle=index))\n"
     "    if any(befund.stufe == sequenz.FEHLER for befund in befunde):\n"
     "        return None, tuple(befunde)\n",
     "                stelle=index))\n"),
    ("Wechsel ist keine Abschnittsgrenze",
     "grenzen = sorted(set(halte) | set(wechsel))",
     "grenzen = sorted(set(halte))"),
    ("Wechsel mit Radius haelt",
     "uebergaenge = [i for i in wechsel if i not in halte]",
     "uebergaenge = []"),
    ("Wechsel mit Radius 0 verschliffen",
     "uebergaenge = [i for i in wechsel if i not in halte]",
     "uebergaenge = list(wechsel)"),
    ("Wechsel haelt immer (alte Regel)",
     "halte, umkehr = totg_folge.abschnitte(wegpunkte, genau)",
     "halte, umkehr = totg_folge.abschnitte(wegpunkte, "
     "sorted(set(genau) | set(wechsel)))"),
    ("Verschleifen nicht gemeldet",
     "    if uebergaenge:\n",
     "    if uebergaenge[:0]:\n"),
    ("Verschleifen nicht im Auftrag",
     '"verschleifen": list(uebergaenge),',
     '"verschleifen": [],'),
    ("Verschliffen nicht gelesen",
     "verschliffen=[str(v.get(\"punkt\")) for v in",
     "verschliffen=[str(v.get(\"x\")) for v in"),
    ("PTP-Radius zaehlt als LIN-Radius",
     'and arten[index] == arten[index + 1] == "LIN"})',
     '})'),
    ("Fahrart des Abschnitts vertauscht",
     '"fahrarten": [arten[a] for a, _b in stuecke],',
     '"fahrarten": [arten[a] for a, _b in stuecke][::-1],'),
    ("Fahrart immer LIN",
     '"fahrarten": [arten[a] for a, _b in stuecke],',
     '"fahrarten": ["LIN"] * len(stuecke),'),
    ("PTP-Ueberschleifen nicht gemeldet",
     "    if ptp_rund:\n",
     "    if ptp_rund[:0]:\n"),
    ("Ueberschleifen falsch",
     "UEBERSCHLEIFEN_RAD = 0.2",
     "UEBERSCHLEIFEN_RAD = 0.02"),
    ("Fahrarten nicht gelesen",
     'fahrarten=[str(a.get("fahrart") or "LIN") for a in lin],',
     'fahrarten=[],'),
    ("letzter Radius zaehlt mit",
     "for index in range(len(posten) - 1)\n                         if radien",
     "for index in range(len(posten))\n                         if radien"),
    ("verschiedene Radien erlaubt",
     "    if len(ueber_null) > 1:\n",
     "    if len(ueber_null) > 2:\n"),
    ("groesster statt einziger Radius",
     "rundung_trans = ueber_null[0] if ueber_null else",
     "rundung_trans = ueber_null[-1] * 2 if ueber_null else"),
    ("Minimum ohne Radius",
     "RUNDUNG_TRANS_OHNE_RADIUS = 0.001",
     "RUNDUNG_TRANS_OHNE_RADIUS = 0.01"),
    ("Genauhalt vergessen",
     "             if radius == 0.0]",
     "             if radius < 0.0]"),
    ("Abschnitte versetzt",
     '"abschnitte": [[a, b] for a, b in stuecke],',
     '"abschnitte": [[0, grenzen[-1]]],'),
    ("Umkehr nicht gemeldet",
     "    for index in umkehr:\n",
     "    for index in umkehr[:0]:\n"),
    ("Grenzfehler verschluckt",
     "        return None, tuple(befunde)\n    for hinweis in grenzhinweise:",
     "        pass\n    for hinweis in grenzhinweise:"),
    ("Grenzhinweise fehlen",
     "        befunde.append(sequenz.Befund(sequenz.HINWEIS, hinweis))",
     "        pass"),
    ("Rotation nicht uebergeben",
     '"rundung_rotation_rad": float(rundung_rot),',
     '"rundung_rotation_rad": RUNDUNG_ROT_STANDARD,'),
    ("Startname falsch",
     "namen = [totg_folge.START_NAME]",
     'namen = ["P0"]'),
    ("Stand fehlt",
     '"stand": AUFTRAG_STAND,',
     '"stand": 0,'),
    ("Iterationen nicht gelesen",
     'toppra_iterationen=[a.get("toppra_iterationen") for a in abschnitte],',
     'toppra_iterationen=[],'),
    ("roh und abgetastet vertauscht",
     'v_faktor_max=roh.get("v_faktor_max"),',
     'v_faktor_max=abgetastet.get("v_faktor_max"),'),
    ("Sprung nicht das Maximum",
     "ik_sprung_max_rad=max(",
     "ik_sprung_max_rad=min("),
    ("Hinweise in der CSV fehlen",
     '        zeilen.append(["#intrinsic_hinweis", str(hinweis)])',
     '        pass'),
    ("Liste in der CSV als repr",
     '                text = ",".join(str(w) for w in wert)',
     '                text = str(wert)'),
)


def lauf(ordner):
    ergebnis = subprocess.run(
        [sys.executable, "-B", "test_intrinsic_folge.py"], cwd=ordner,
        capture_output=True, text=True, timeout=120,
        # Ohne Bytecode: eine Mutation in derselben Sekunde saehe sonst
        # die zwischengespeicherte Fassung der vorigen.
        env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))
    return ergebnis.returncode


def main():
    original = open(os.path.join(HIER, "intrinsic_folge.py"),
                    encoding="utf-8").read()
    ordner = tempfile.mkdtemp(prefix="mutation_intrinsic_")
    try:
        for name in DATEIEN:
            shutil.copy(os.path.join(HIER, name), ordner)
        ziel = os.path.join(ordner, "intrinsic_folge.py")
        with open(ziel, "w", encoding="utf-8") as datei:
            datei.write(original)
        if lauf(ordner) != 0:
            print("Die Tests scheitern schon am Original.")
            return 2
        ueberlebt = []
        for name, alt, neu in PROBEN:
            if original.count(alt) != 1:
                print(f"PROBE KAPUTT  {name}: Stelle {original.count(alt)}x")
                ueberlebt.append(name)
                continue
            with open(ziel, "w", encoding="utf-8") as datei:
                datei.write(original.replace(alt, neu))
            code = lauf(ordner)
            print(f"{'erkannt   ' if code else 'UEBERLEBT '} {name}")
            if not code:
                ueberlebt.append(name)
        print(f"\n{len(PROBEN) - len(ueberlebt)} von {len(PROBEN)} erkannt")
        return 1 if ueberlebt else 0
    finally:
        shutil.rmtree(ordner, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
