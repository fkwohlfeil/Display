#!/usr/bin/env python3
"""Patch intrinsic_lin_rechner.py Stand 2 -> Stand 3: gekoppelte Gelenke in der FK.

Aufruf (im Ordner des Rechners):
    python3 patch_lin_rechner_mimic_3.py [intrinsic_lin_rechner.py]

Fehlerbild
    "LIN-Pfad endet 0.23 rad neben dem Zielwegpunkt", obwohl die erste
    IK-Loesung passt. Die Abweichung waechst mit dem Weg vom Start.

Ursache (nachgemessen am bmKinematic-Modell, joint6 mimic joint5 * -1)
    _setzen schrieb die ganze Grundstellung in den Zustand, auch das
    gekoppelte (mimic) Gelenk mit seinem Startwert. moveit_py setzt die
    Gelenke einzeln in Namensreihenfolge; jedes Setzen eines Leitgelenks
    zieht seine Mimic-Gelenke nach. Kommt das Mimic-Gelenk alphabetisch nach
    seinem Leitgelenk, ueberschreibt der alte Startwert den richtigen Wert.
    fk() rechnet dann mit falscher Kopplung -> falsche Soll-Posen fuer die
    Kurve. Am Start stimmt es (Startwert = richtiger Wert), zum Ziel hin
    nicht mehr. Gemessen an P3: joint5 0.2648, joint6 blieb 0.0 statt
    -0.2648, FK 83.9 mm / 0.265 rad daneben.

Aenderung
    Gelenke der Gruppe, die nicht aktiv sind (Mimic), gehen nicht mehr in
    den Zustand; MoveIt leitet sie aus dem Leitgelenk ab. Nichts wird
    korrigiert: die Kopplung wird nur nicht mehr verfaelscht.

Fassung 2 dieses Patches: Anker des Stand-Kommentars an den Rechner mit
Stand 2 = "fahrarten" (PTP) angepasst. Die erste Fassung erwartete einen
Stand 2 mit "ziel_ast", den es hier nicht gibt, und brach mit 0 Treffern ab.
"""

import ast
import sys

DATEI = sys.argv[1] if len(sys.argv) > 1 else "intrinsic_lin_rechner.py"
MARKE = "# [STAND 3] Mimic-Gelenke nicht aus der Grundstellung setzen"

SEGMENTE = [
    ("stand",
     "\nSTAND = 2\n",
     "\nSTAND = 3\n"),
    ("stand_kommentar",
     "# Seit 2: \"fahrarten\" je Abschnitt (LIN, PTP).\n",
     "# Seit 2: \"fahrarten\" je Abschnitt (LIN, PTP).\n"
     "# Seit 3: gekoppelte (mimic) Gelenke nicht mehr aus der Grundstellung\n"
     "#    ueberschreiben -- verfaelschte die FK der Wegpunkte.\n"),
    ("init_ausblenden",
     "        self.grundstellung = {k: float(v) for k, v in auftrag[\"start\"].items()}\n",
     "        " + MARKE + "\n"
     "        # Alle Gelenke der Gruppe, die nicht aktiv sind: gekoppelte\n"
     "        # (mimic) und feste. Ihr Wert folgt in MoveIt dem Leitgelenk.\n"
     "        self.abgeleitet = set(self.modell.get_joint_model_group(\n"
     "            self.gruppe).joint_model_names) - set(aktiv)\n"
     "        self.grundstellung = {k: float(v) for k, v in auftrag[\"start\"].items()\n"
     "                              if k not in self.abgeleitet}\n"),
    ("setzen_kommentar",
     "    def _setzen(self, lagen):\n",
     "    def _setzen(self, lagen):\n"
     "        # Ohne Mimic-Gelenke: moveit_py setzt die Eintraege einzeln in\n"
     "        # Namensreihenfolge und zieht bei jedem Leitgelenk dessen\n"
     "        # Mimic-Gelenke nach. Ein Mimic-Eintrag hier wuerde das -- je nach\n"
     "        # Name -- mit dem Startwert wieder ueberschreiben.\n"),
]


def main():
    with open(DATEI, encoding="utf-8") as datei:
        text = datei.read()
    if MARKE in text:
        print(f"{DATEI}: Patch ist schon drin, nichts geaendert.")
        return 0
    for name, alt, neu in SEGMENTE:
        anzahl = text.count(alt)
        if anzahl != 1:
            print(f"Segment {name}: {anzahl} Treffer statt 1 -- falscher "
                  f"Stand der Datei? Nichts geschrieben.")
            return 1
        text = text.replace(alt, neu)
    try:
        ast.parse(text)
    except SyntaxError as fehler:
        print(f"Syntaxfehler nach dem Patch: {fehler}. Nichts geschrieben.")
        return 1
    with open(DATEI, "w", encoding="utf-8") as datei:
        datei.write(text)
    print(f"{DATEI}: Stand 3 geschrieben ({len(SEGMENTE)} Segmente).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
