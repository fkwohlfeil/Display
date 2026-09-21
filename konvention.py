"""Drehkonventionen fuer die Winkel A, B, C.

Eine Drehung im Raum hat drei Freiheitsgrade. Welche drei Zahlen man dafuer
hinschreibt, ist eine Wahl -- und zwar eine, die man treffen muss, bevor eine
Zahl einen Sinn hat. Dieses Modul haelt die drei Wahlen, die die GUI anbietet,
an einer Stelle zusammen.

    Konvention   erste Drehung   zweite   dritte   Art
    ZYX          um Z (A)        um Y (B) um X (C) Kardan-/Tait-Bryan-Winkel
    ZYZ          um Z (A)        um Y (B) um Z (C) eigentliche Eulerwinkel
    ZXZ          um Z (A)        um X (B) um Z (C) eigentliche Eulerwinkel

Alle drei sind *intrinsisch* zu lesen: jede Drehung dreht um eine Achse des
bereits mitgedrehten Systems. Als Matrizenprodukt steht das rechtsmultipliziert
da, und in dieser Form ist es in jeder Konvention derselbe Satz:

    R = R_a1(A) @ R_a2(B) @ R_a3(C)

Deshalb heissen die Winkel A, B und C und nicht Roll, Pitch, Yaw: A ist immer
die erste Drehung, C immer die letzte. Der Name sagt die Reihenfolge, die
Konvention sagt die Achsen. Bei ZYX faellt das mit der Belegung nach DIN 66217
zusammen, wie sie KUKA verwendet -- A ist Gieren, B ist Nicken, C ist Rollen.

Achtung: die URDF kennt nur ZYX. Ihr Attribut rpy ist als feste XYZ-Drehung
definiert, was derselben Matrix entspricht wie intrinsisch ZYX mit
(A, B, C) = (yaw, pitch, roll). Zum Schreiben einer URDF muss also immer nach
ZYX zurueckgerechnet werden; dafuer stehen unten from_rpy_xyz und to_xyz_rpy.
Die ZYX-Formel ist hier zeichengleich zu der Fassung abgeschrieben, die bis
Stand 2_19 in der GUI stand. Gleiche Summationsreihenfolge heisst gleiches
Rundungsergebnis, und damit lesen sich alte Dateien bitgleich wie vorher.

Kein NumPy. Die Summationsreihenfolge liegt damit fest und ist zwischen
Rechnern reproduzierbar.
"""

import math


# ==========================================================================
# Die Konventionen
# ==========================================================================

KONVENTIONEN = ("ZYX", "ZYZ", "ZXZ")

# Was die GUI anzeigt, wenn nichts anderes gewaehlt wurde. ZYX, weil das die
# Konvention der URDF ist und die der Industriesteuerungen: eine aus einem
# KUKA-Handbuch abgeschriebene Pose laesst sich damit ohne Umrechnung eintippen.
STANDARD = "ZYX"

# Die drei Drehachsen in der Reihenfolge A, B, C.
ACHSEN = {
    "ZYX": ("Z", "Y", "X"),
    "ZYZ": ("Z", "Y", "Z"),
    "ZXZ": ("Z", "X", "Z"),
}

# Kurzform fuer die Kopfzeile des TCP-Fensters.
TITEL = {
    "ZYX": "ZYX  Kardan",
    "ZYZ": "ZYZ  Euler",
    "ZXZ": "ZXZ  Euler",
}

# Langform fuer Statuszeile, Dateikopf und Fehlermeldung.
BESCHREIBUNG = {
    "ZYX": "Kardanwinkel, R = Rz(A) Ry(B) Rx(C), entspricht URDF rpy",
    "ZYZ": "Eulerwinkel, R = Rz(A) Ry(B) Rz(C)",
    "ZXZ": "Eulerwinkel, R = Rz(A) Rx(B) Rz(C)",
}

# Wertebereich von B in Grad. Bei ZYX ist B ein Nickwinkel und laeuft
# symmetrisch um Null; bei den Eulerkonventionen ist B ein Oeffnungswinkel
# zwischen zwei Achsen und kann nicht negativ werden. A und C laufen immer
# ueber (-180, 180].
B_BEREICH_DEG = {
    "ZYX": (-90.0, 90.0),
    "ZYZ": (0.0, 180.0),
    "ZXZ": (0.0, 180.0),
}

# Werte von B in Grad, an denen die Konvention entartet: zwei der drei
# Drehungen wirken dort um dieselbe Achse, A und C sind nicht mehr einzeln
# bestimmt. Bei ZYX liegt das an einer aufrecht stehenden Werkzeugachse und
# wird selten erreicht. Bei ZYZ und ZXZ liegt es bei B = 0, also genau dann,
# wenn die Werkzeugachse parallel zur Basis-Z-Achse steht -- der ueblichen
# Grundstellung. Das ist keine Eigenschaft des Roboters, sondern der Zahlen,
# mit denen man ihn beschreibt.
B_SINGULAER_DEG = {
    "ZYX": (-90.0, 90.0),
    "ZYZ": (0.0, 180.0),
    "ZXZ": (0.0, 180.0),
}

# Ab hier wird der Sonderfallzweig genommen. Der Wert ist der der GUI bis 2_19.
EPS = 1e-9


def pruefe(konvention):
    """Namen pruefen und zurueckgeben. Wirft, statt still auf ZYX zu fallen."""
    if konvention not in KONVENTIONEN:
        raise ValueError(
            "Unbekannte Drehkonvention %r, erlaubt sind %s"
            % (konvention, ", ".join(KONVENTIONEN)))
    return konvention


# ==========================================================================
# Winkel -> Matrix
# ==========================================================================

def from_abc_xyz(abc, xyz, konvention=STANDARD):
    """(A, B, C) in rad und (x, y, z) in m als 4x4.

    Ausmultipliziert statt als Produkt dreier Matrizen. Das spart nicht nur
    Rechnung, es legt auch die Summationsreihenfolge fest.
    """
    pruefe(konvention)
    a, b, c = abc
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cc, sc = math.cos(c), math.sin(c)

    if konvention == "ZYX":
        # Zeichengleich zu from_rpy_xyz bis Stand 2_19, mit
        # (a, b, c) = (yaw, pitch, roll).
        rot = [
            [ca * cb, ca * sb * sc - sa * cc, ca * sb * cc + sa * sc],
            [sa * cb, sa * sb * sc + ca * cc, sa * sb * cc - ca * sc],
            [-sb,     cb * sc,                cb * cc],
        ]
    elif konvention == "ZYZ":
        rot = [
            [ca * cb * cc - sa * sc, -ca * cb * sc - sa * cc, ca * sb],
            [sa * cb * cc + ca * sc, -sa * cb * sc + ca * cc, sa * sb],
            [-sb * cc,               sb * sc,                 cb],
        ]
    else:                                              # ZXZ
        rot = [
            [ca * cc - sa * cb * sc, -ca * sc - sa * cb * cc, sa * sb],
            [sa * cc + ca * cb * sc, -sa * sc + ca * cb * cc, -ca * sb],
            [sb * sc,                sb * cc,                 cb],
        ]

    return [
        [rot[0][0], rot[0][1], rot[0][2], xyz[0]],
        [rot[1][0], rot[1][1], rot[1][2], xyz[1]],
        [rot[2][0], rot[2][1], rot[2][2], xyz[2]],
        [0.0,       0.0,       0.0,       1.0],
    ]


# ==========================================================================
# Matrix -> Winkel
# ==========================================================================

def to_xyz_abc(matrix, konvention=STANDARD):
    """4x4 -> ((x, y, z), (A, B, C)). Umkehrung von from_abc_xyz.

    Zu jeder Drehung gehoeren zwei Tripel, die dieselbe Matrix ergeben. Welches
    davon herauskommt, ist eine Festlegung und keine Rechnung: genommen wird
    das mit B im Bereich aus B_BEREICH_DEG. Ohne diese Festlegung waere die
    Anzeige nicht reproduzierbar, weil beide Tripel gleich richtig sind.

    Im Sonderfall sind A und C nicht einzeln bestimmt, nur ihre Summe oder
    Differenz. Dann wird C auf Null gesetzt und die ganze Drehung in A gelegt.
    Das ist dieselbe Wahl, die die GUI bis 2_19 fuer den Rollwinkel getroffen
    hat.
    """
    pruefe(konvention)
    xyz = (matrix[0][3], matrix[1][3], matrix[2][3])

    if konvention == "ZYX":
        sb = -matrix[2][0]
        if abs(sb) >= 1.0 - EPS:
            b = math.copysign(math.pi / 2.0, sb)
            c = 0.0
            a = math.atan2(-matrix[0][1], matrix[1][1])
        else:
            b = math.asin(sb)
            c = math.atan2(matrix[2][1], matrix[2][2])
            a = math.atan2(matrix[1][0], matrix[0][0])
        return xyz, (a, b, c)

    if konvention == "ZYZ":
        # atan2 mit der Laenge der Nebenachsenanteile statt acos(r22): acos
        # verliert nahe 0 und 180 Grad Stellen, weil dort die Ableitung
        # ueber alle Grenzen waechst. Die Wurzel ist nie negativ, damit
        # liegt B von selbst in [0, pi].
        laenge = math.sqrt(matrix[0][2] * matrix[0][2]
                           + matrix[1][2] * matrix[1][2])
        b = math.atan2(laenge, matrix[2][2])
        if laenge <= EPS:
            c = 0.0
            if matrix[2][2] >= 0.0:                    # B = 0, R = Rz(A + C)
                a = math.atan2(matrix[1][0], matrix[0][0])
            else:                                      # B = pi, R = Rz(A - C) Ry(pi)
                a = math.atan2(-matrix[1][0], -matrix[0][0])
        else:
            a = math.atan2(matrix[1][2], matrix[0][2])
            c = math.atan2(matrix[2][1], -matrix[2][0])
        return xyz, (a, b, c)

    # ZXZ
    laenge = math.sqrt(matrix[0][2] * matrix[0][2]
                       + matrix[1][2] * matrix[1][2])
    b = math.atan2(laenge, matrix[2][2])
    if laenge <= EPS:
        c = 0.0
        a = math.atan2(matrix[1][0], matrix[0][0])     # gilt fuer B = 0 und B = pi
    else:
        a = math.atan2(matrix[0][2], -matrix[1][2])
        c = math.atan2(matrix[2][0], matrix[2][1])
    return xyz, (a, b, c)


# ==========================================================================
# Bruecke zur URDF
# ==========================================================================

def from_rpy_xyz(rpy, xyz):
    """URDF-origin als 4x4. rpy = (roll, pitch, yaw) = (C, B, A) bei ZYX."""
    roll, pitch, yaw = rpy
    return from_abc_xyz((yaw, pitch, roll), xyz, "ZYX")


def to_xyz_rpy(matrix):
    """4x4 -> ((x, y, z), (roll, pitch, yaw)). Fuer den URDF-Schreibpfad."""
    xyz, (a, b, c) = to_xyz_abc(matrix, "ZYX")
    return xyz, (c, b, a)


# ==========================================================================
# Naehe zur Entartung
# ==========================================================================

def singularitaetsabstand(abc, konvention=STANDARD):
    """Abstand von B zur naechsten entarteten Stellung, in rad.

    Faellt der Wert gegen Null, verlieren A und C ihre Bedeutung: sie drehen
    dann um dieselbe Achse und lassen sich beliebig gegeneinander verschieben,
    ohne dass sich die Lage aendert. Die Anzeige springt dort, und beim
    Joggen laufen zwei Felder auf dieselbe Bewegung.

    Das ist eine Eigenschaft der Zahlen, nicht der Maschine. Der Roboter steht
    dort voellig unauffaellig. Die kinematische Singularitaet, bei der ein
    Bahnmillimeter beliebig viel Achsweg kostet, ist etwas anderes und wird
    an anderer Stelle gemeldet.
    """
    pruefe(konvention)
    b_deg = math.degrees(abc[1])
    return math.radians(min(abs(b_deg - marke)
                            for marke in B_SINGULAER_DEG[konvention]))


def ist_singulaer(abc, konvention=STANDARD, schwelle_deg=1.0):
    """True, wenn B naeher als schwelle_deg an einer entarteten Stellung steht."""
    return singularitaetsabstand(abc, konvention) < math.radians(schwelle_deg)


# ==========================================================================
# Kennung fuer Dateikoepfe
# ==========================================================================

# Alte Dateien tragen keine Kennung. Sie sind in ZYX geschrieben, aber mit
# der Spaltenfolge roll, pitch, yaw -- also C, B, A. Wer sie ohne Ruecksicht
# als A, B, C liest, bekommt eine gueltige, aber falsche Lage und keine
# Fehlermeldung. Deshalb gehoert die Kennung in jeden Kopf, und deshalb muss
# eine fehlende Kennung abgelehnt und nicht geraten werden.
KENNUNG_SCHLUESSEL = "drehkonvention"


def kennung(konvention=STANDARD):
    """Zeile fuer den Dateikopf."""
    pruefe(konvention)
    return "%s=%s" % (KENNUNG_SCHLUESSEL, konvention)


def lies_kennung(zeile):
    """Kennung aus einer Kopfzeile lesen. None, wenn keine drinsteht."""
    text = zeile.strip().lstrip("#").strip()
    if not text.startswith(KENNUNG_SCHLUESSEL + "="):
        return None
    return pruefe(text.split("=", 1)[1].strip())


def spaltennamen(praefix="", konvention=STANDARD):
    """CSV-Spaltenkoepfe fuer die drei Winkel, in Grad."""
    pruefe(konvention)
    achsen = ACHSEN[konvention]
    return tuple("%s%s_%s_deg" % (praefix, name, achse.lower())
                 for name, achse in zip(("a", "b", "c"), achsen))
