"""Pruefungen fuer konvention.py.

Aufbau in fuenf Gruppen:

  1  Die ausmultiplizierten Formeln gegen das Produkt dreier Elementardrehungen
  2  Die ZYX-Formel gegen die Fassung, die bis 2_19 in der GUI stand
  3  Hin und zurueck ueber ein Winkelraster
  4  Die Sonderfaelle an den entarteten Stellungen
  5  Bereiche, Kennungen, Fehlerfaelle

Gruppe 1 ist die eigentliche Absicherung. Die ausmultiplizierten Formeln sind
von Hand hergeleitet, und ein Vorzeichenfehler darin faellt beim Hin- und
Zurueckrechnen nicht auf, weil die Umkehrung denselben Fehler noch einmal
macht. Erst der Vergleich gegen ein unabhaengig gebildetes Produkt findet ihn.

Aufruf:  python3 test_konvention.py
"""

import math
import sys

import konvention as k


BESTANDEN = 0
GEFALLEN = []


def pruefung(name, bedingung, zusatz=""):
    global BESTANDEN
    if bedingung:
        BESTANDEN += 1
    else:
        GEFALLEN.append(name + (("  " + zusatz) if zusatz else ""))


# ==========================================================================
# Hilfsmittel: Elementardrehungen und ihr Produkt
# ==========================================================================

def elementar(achse, winkel):
    c, s = math.cos(winkel), math.sin(winkel)
    if achse == "X":
        return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
    if achse == "Y":
        return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def mal(a, b):
    return [[sum(a[i][m] * b[m][j] for m in range(3)) for j in range(3)]
            for i in range(3)]


def produkt(abc, konvention):
    """R = R_a1(A) R_a2(B) R_a3(C), unabhaengig von der Formel im Modul."""
    achsen = k.ACHSEN[konvention]
    return mal(mal(elementar(achsen[0], abc[0]), elementar(achsen[1], abc[1])),
               elementar(achsen[2], abc[2]))


def dreh(matrix):
    return [zeile[:3] for zeile in matrix[:3]]


def abstand(a, b):
    return max(abs(a[i][j] - b[i][j]) for i in range(3) for j in range(3))


# Referenz: die Formel, wie sie bis Stand 2_19 in der GUI stand.
def alt_from_rpy_xyz(rpy, xyz):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp,     cp * sr,                cp * cr,                xyz[2]],
        [0.0,     0.0,                    0.0,                    1.0],
    ]


def alt_to_xyz_rpy(matrix):
    xyz = (matrix[0][3], matrix[1][3], matrix[2][3])
    sp = -matrix[2][0]
    if abs(sp) >= 1.0 - 1e-9:
        pitch = math.copysign(math.pi / 2.0, sp)
        roll = 0.0
        yaw = math.atan2(-matrix[0][1], matrix[1][1])
    else:
        pitch = math.asin(sp)
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    return xyz, (roll, pitch, yaw)


RASTER = [-170.0, -125.0, -90.0, -47.0, -1.0, 0.0, 1.0, 33.0, 90.0, 118.0, 180.0]
RASTER_B = {
    "ZYX": [-89.0, -60.0, -13.0, 0.0, 13.0, 60.0, 89.0],
    "ZYZ": [1.0, 17.0, 62.0, 90.0, 121.0, 179.0],
    "ZXZ": [1.0, 17.0, 62.0, 90.0, 121.0, 179.0],
}


# ==========================================================================
# Gruppe 1: Formel gegen Matrizenprodukt
# ==========================================================================

for konv in k.KONVENTIONEN:
    schlimmster = 0.0
    for a in RASTER:
        for b in RASTER_B[konv]:
            for c in RASTER:
                abc = (math.radians(a), math.radians(b), math.radians(c))
                schlimmster = max(schlimmster,
                                  abstand(dreh(k.from_abc_xyz(abc, (0, 0, 0), konv)),
                                          produkt(abc, konv)))
    pruefung("1.%s Formel gleich Produkt der Elementardrehungen" % konv,
             schlimmster < 1e-14, "Abweichung %.3e" % schlimmster)

# Die Verschiebung darf die Drehung nicht anfassen und muss unveraendert
# in der vierten Spalte stehen.
m = k.from_abc_xyz((0.3, 0.4, 0.5), (1.5, -2.25, 0.125), "ZYZ")
pruefung("1.4 Verschiebung unveraendert",
         (m[0][3], m[1][3], m[2][3]) == (1.5, -2.25, 0.125))
pruefung("1.5 letzte Zeile homogen", m[3] == [0.0, 0.0, 0.0, 1.0])

# Orthonormal und rechtshaendig: R R^T = I und det R = +1.
for konv in k.KONVENTIONEN:
    r = dreh(k.from_abc_xyz((0.7, 0.9, -1.1), (0, 0, 0), konv))
    rt = [[r[j][i] for j in range(3)] for i in range(3)]
    eins = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
    det = (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
           - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
           + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))
    pruefung("1.6.%s orthonormal" % konv, abstand(mal(r, rt), eins) < 1e-14)
    pruefung("1.7.%s Determinante +1" % konv, abs(det - 1.0) < 1e-14)


# ==========================================================================
# Gruppe 2: ZYX bitgleich zur alten Fassung
# ==========================================================================

ungleich = 0
for a in RASTER:
    for b in RASTER_B["ZYX"]:
        for c in RASTER:
            rpy = (math.radians(c), math.radians(b), math.radians(a))
            neu = k.from_rpy_xyz(rpy, (0.1, 0.2, 0.3))
            alt = alt_from_rpy_xyz(rpy, (0.1, 0.2, 0.3))
            if neu != alt:                             # bitgleich, nicht nur nah
                ungleich += 1
pruefung("2.1 from_rpy_xyz bitgleich zu 2_19", ungleich == 0,
         "%d Abweichungen" % ungleich)

ungleich = 0
for a in RASTER:
    for b in RASTER_B["ZYX"]:
        for c in RASTER:
            rpy = (math.radians(c), math.radians(b), math.radians(a))
            m = alt_from_rpy_xyz(rpy, (0.0, 0.0, 0.0))
            if k.to_xyz_rpy(m) != alt_to_xyz_rpy(m):
                ungleich += 1
pruefung("2.2 to_xyz_rpy bitgleich zu 2_19", ungleich == 0,
         "%d Abweichungen" % ungleich)

# A ist Gieren, C ist Rollen -- nicht umgekehrt. Eine Drehung um Z allein
# muss in A landen.
_, abc = k.to_xyz_abc(k.from_rpy_xyz((0.0, 0.0, math.radians(35.0)),
                                     (0, 0, 0)), "ZYX")
pruefung("2.3 A ist die Drehung um Z",
         abs(math.degrees(abc[0]) - 35.0) < 1e-9 and abs(abc[1]) < 1e-12
         and abs(abc[2]) < 1e-12)
_, abc = k.to_xyz_abc(k.from_rpy_xyz((math.radians(35.0), 0.0, 0.0),
                                     (0, 0, 0)), "ZYX")
pruefung("2.4 C ist die Drehung um X",
         abs(math.degrees(abc[2]) - 35.0) < 1e-9 and abs(abc[0]) < 1e-12)


# ==========================================================================
# Gruppe 3: hin und zurueck
# ==========================================================================

for konv in k.KONVENTIONEN:
    schlimmster = 0.0
    ausserhalb = 0
    b_min, b_max = k.B_BEREICH_DEG[konv]
    for a in RASTER:
        for b in RASTER_B[konv]:
            for c in RASTER:
                abc = (math.radians(a), math.radians(b), math.radians(c))
                m = k.from_abc_xyz(abc, (0.4, -0.5, 0.6), konv)
                xyz, zurueck = k.to_xyz_abc(m, konv)
                # Nicht die Winkel vergleichen, sondern die Matrix: zu jeder
                # Drehung gehoeren zwei gueltige Tripel.
                schlimmster = max(schlimmster,
                                  abstand(dreh(k.from_abc_xyz(zurueck, (0, 0, 0), konv)),
                                          dreh(m)))
                if not (b_min - 1e-9 <= math.degrees(zurueck[1]) <= b_max + 1e-9):
                    ausserhalb += 1
                if xyz != (0.4, -0.5, 0.6):
                    ausserhalb += 1000
    pruefung("3.1.%s Matrix nach Rueckrechnung gleich" % konv,
             schlimmster < 1e-12, "Abweichung %.3e" % schlimmster)
    pruefung("3.2.%s B im Bereich %s" % (konv, k.B_BEREICH_DEG[konv]),
             ausserhalb == 0, "%d Verstoesse" % ausserhalb)

# Sehr kleine und sehr grosse B. Hier trennt sich atan2 von acos: acos(r22)
# waere fuer diese Werte um Groessenordnungen ungenauer, weil das Argument
# dicht an +-1 liegt und dort die halbe Mantisse verlorengeht. Die Schwelle
# unten ist so gewaehlt, dass acos sie reisst und atan2 sie haelt.
for konv in ("ZYZ", "ZXZ"):
    schlimmster = 0.0
    for b in (1e-3, 1e-5, 1e-7, 180.0 - 1e-3, 180.0 - 1e-5, 180.0 - 1e-7):
        abc = (math.radians(25.0), math.radians(b), math.radians(-80.0))
        m = k.from_abc_xyz(abc, (0, 0, 0), konv)
        _, zurueck = k.to_xyz_abc(m, konv)
        schlimmster = max(schlimmster,
                          abstand(dreh(k.from_abc_xyz(zurueck, (0, 0, 0), konv)),
                                  dreh(m)))
    pruefung("3.4.%s B dicht an der Entartung genau" % konv,
             schlimmster < 1e-12, "Abweichung %.3e" % schlimmster)

# Zweimal rueckrechnen muss dasselbe Tripel liefern. Sonst waere die Anzeige
# von der Vorgeschichte abhaengig.
for konv in k.KONVENTIONEN:
    stabil = True
    for a in RASTER:
        for b in RASTER_B[konv]:
            abc = (math.radians(a), math.radians(b), math.radians(-47.0))
            erst = k.to_xyz_abc(k.from_abc_xyz(abc, (0, 0, 0), konv), konv)[1]
            zweit = k.to_xyz_abc(k.from_abc_xyz(erst, (0, 0, 0), konv), konv)[1]
            if max(abs(x - y) for x, y in zip(erst, zweit)) > 1e-12:
                stabil = False
    pruefung("3.3.%s Rueckrechnung ist ein Festpunkt" % konv, stabil)


# ==========================================================================
# Gruppe 4: die entarteten Stellungen
# ==========================================================================

SONDER = {
    "ZYX": (-90.0, 90.0),
    "ZYZ": (0.0, 180.0),
    "ZXZ": (0.0, 180.0),
}

for konv in k.KONVENTIONEN:
    for b in SONDER[konv]:
        for a in (-150.0, -30.0, 0.0, 75.0, 180.0):
            for c in (-120.0, 0.0, 95.0):
                abc = (math.radians(a), math.radians(b), math.radians(c))
                m = k.from_abc_xyz(abc, (0, 0, 0), konv)
                _, zurueck = k.to_xyz_abc(m, konv)
                wieder = k.from_abc_xyz(zurueck, (0, 0, 0), konv)
                pruefung("4.%s B=%g A=%g C=%g Matrix erhalten"
                         % (konv, b, a, c), abstand(dreh(wieder), dreh(m)) < 1e-12)
                pruefung("4.%s B=%g A=%g C=%g C auf Null gelegt"
                         % (konv, b, a, c), abs(zurueck[2]) < 1e-12)

# Knapp neben der Entartung darf nicht der Sonderfallzweig greifen, und die
# Matrix muss trotzdem stimmen.
#
# Die Schwelle EPS wirkt in den Konventionen verschieden weit. Bei ZYX steht
# sie am Kosinus des Abstands und greift deshalb schon rund 0.0026 Grad neben
# der Entartung; bei ZYZ und ZXZ steht sie am Sinus und greift erst rund
# 6e-8 Grad davor. Beides ist richtig gerechnet -- die Zahl 1e-9 bedeutet in
# den beiden Faellen nur nicht dasselbe. 89.99 liegt bei ZYX sicher ausserhalb.
for konv, b in (("ZYX", 89.99), ("ZYZ", 0.0001), ("ZXZ", 179.9999)):
    abc = (math.radians(20.0), math.radians(b), math.radians(-70.0))
    m = k.from_abc_xyz(abc, (0, 0, 0), konv)
    _, zurueck = k.to_xyz_abc(m, konv)
    pruefung("4.9.%s knapp neben der Entartung" % konv,
             abstand(dreh(k.from_abc_xyz(zurueck, (0, 0, 0), konv)),
                     dreh(m)) < 1e-9)


# ==========================================================================
# Gruppe 5: Bereiche, Kennungen, Fehlerfaelle
# ==========================================================================

pruefung("5.1 Abstand null in der Entartung",
         k.singularitaetsabstand((0.0, math.radians(90.0), 0.0), "ZYX") < 1e-12
         and k.singularitaetsabstand((0.0, 0.0, 0.0), "ZYZ") < 1e-12
         and k.singularitaetsabstand((0.0, math.pi, 0.0), "ZXZ") < 1e-12)

pruefung("5.2 Grundstellung ist nur bei Euler entartet",
         not k.ist_singulaer((0.0, 0.0, 0.0), "ZYX")
         and k.ist_singulaer((0.0, 0.0, 0.0), "ZYZ")
         and k.ist_singulaer((0.0, 0.0, 0.0), "ZXZ"))

pruefung("5.3 Abstand ist eine Winkeldifferenz",
         abs(math.degrees(k.singularitaetsabstand(
             (0.0, math.radians(60.0), 0.0), "ZYX")) - 30.0) < 1e-9)

pruefung("5.4 Kennung schreiben und lesen",
         all(k.lies_kennung("# " + k.kennung(x)) == x for x in k.KONVENTIONEN))
pruefung("5.5 fehlende Kennung ergibt None",
         k.lies_kennung("# t_s,x_m,y_m") is None)

pruefung("5.6 Spaltennamen tragen die Achse",
         k.spaltennamen("tcp_", "ZYZ") == ("tcp_a_z_deg", "tcp_b_y_deg",
                                           "tcp_c_z_deg"))
pruefung("5.7 Spaltennamen unterscheiden die Konventionen",
         k.spaltennamen("", "ZYX") != k.spaltennamen("", "ZXZ"))

for aufruf, name in ((lambda: k.from_abc_xyz((0, 0, 0), (0, 0, 0), "XYZ"),
                      "5.8 from_abc_xyz lehnt XYZ ab"),
                     (lambda: k.to_xyz_abc(k.from_abc_xyz((0, 0, 0), (0, 0, 0)),
                                           "zyx"),
                      "5.9 to_xyz_abc lehnt Kleinschreibung ab"),
                     (lambda: k.lies_kennung("drehkonvention=ZZZ"),
                      "5.10 lies_kennung lehnt Unsinn ab")):
    try:
        aufruf()
        pruefung(name, False, "kein Fehler geworfen")
    except ValueError:
        pruefung(name, True)

pruefung("5.11 Standard ist ZYX", k.STANDARD == "ZYX")
# Fest eingetragen und nicht aus dem Modul gelesen. Wer die Erwartung aus dem
# Pruefling holt, prueft nichts.
pruefung("5.13 Bereich von B fest",
         k.B_BEREICH_DEG == {"ZYX": (-90.0, 90.0), "ZYZ": (0.0, 180.0),
                             "ZXZ": (0.0, 180.0)})
pruefung("5.14 entartete Stellungen fest",
         k.B_SINGULAER_DEG == {"ZYX": (-90.0, 90.0), "ZYZ": (0.0, 180.0),
                               "ZXZ": (0.0, 180.0)})
pruefung("5.15 Achsen fest",
         k.ACHSEN == {"ZYX": ("Z", "Y", "X"), "ZYZ": ("Z", "Y", "Z"),
                      "ZXZ": ("Z", "X", "Z")})
pruefung("5.12 jede Konvention hat Titel und Beschreibung",
         all(x in k.TITEL and x in k.BESCHREIBUNG and x in k.ACHSEN
             and x in k.B_BEREICH_DEG for x in k.KONVENTIONEN))


# ==========================================================================

print("%d Pruefungen bestanden, %d gefallen" % (BESTANDEN, len(GEFALLEN)))
for eintrag in GEFALLEN:
    print("   GEFALLEN  " + eintrag)
sys.exit(1 if GEFALLEN else 0)
