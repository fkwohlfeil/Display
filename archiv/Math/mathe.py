# -*- coding: utf-8 -*-
"""mathe.py -- Elementare Vektor-, Matrix- und Rotationsmathematik.

Reines Rechenmodul.  Keine Abhaengigkeit auf Tkinter, ROS, NumPy oder auf
urdf_gui2_*.py.  Einzige Fremdabhaengigkeit ist das Standardmodul math.

Konventionen dieses Moduls
--------------------------
* Alle Groessen in SI-Einheiten: Meter, Radiant.  Keine Grad, keine
  Millimeter.  Umrechnung findet ausschliesslich an der Anzeigegrenze statt.
* Vektoren sind Listen aus drei float.  Matrizen sind Listen von Zeilen
  (zeilenweise, row-major).  Quaternionen sind Tupel (w, x, y, z).
* Die Summationsreihenfolge in allen Produkten ist fest.  Es wird keine
  Kahan-Summation verwendet -- entweder ueberall oder nirgends, und die
  Entscheidung lautet hier: nirgends.  Damit sind alle Ergebnisse bei
  gleicher Eingabe bitgenau reproduzierbar.
* Keine Funktion veraendert ihre Eingabeargumente.  Rueckgaben sind stets
  neue Objekte.
* Dieses Modul kennt keine Eulerwinkel und damit keine Drehkonvention.
  Orientierungsfehler werden als Drehvektor (Achse mal Winkel) dargestellt,
  nicht als Winkeldifferenz.  Die Konventionsfrage aus konvention.py stellt
  sich hier deshalb bewusst nicht.

Autor: erstellt fuer das URDF-GUI-Projekt, Ebene 0 und 1.
"""

import math

__all__ = [
    "EPS", "KLEIN",
    "add3", "sub3", "skaliere3", "skal", "kreuz", "norm", "normiere3",
    "null_matrix", "einheit", "diag", "transponiert",
    "mat_mal_mat", "mat_mal_vec", "mat_plus", "mat_mal_skalar",
    "spur", "schief", "frobenius",
    "winkel_normiert", "klemme", "nahe", "nahe_vektor",
    "rot_x", "rot_y", "rot_z",
    "exp_so3", "log_so3",
    "quat_von_matrix", "matrix_von_quat", "quat_mal_quat", "quat_konjugiert",
    "quat_normiert", "quat_von_achse_winkel", "achse_winkel_von_quat",
    "ist_rotation", "twist_diff",
]

# Schwelle fuer Gleitkommavergleiche.  Bewusst kein == auf float.
EPS = 1.0e-12
# Schwelle, unterhalb derer auf Reihenentwicklung umgeschaltet wird.
KLEIN = 1.0e-8


# ---------------------------------------------------------------------------
# Ebene 0a -- Vektoroperationen
# ---------------------------------------------------------------------------

def add3(a, b):
    """Summe zweier 3-Vektoren."""
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def sub3(a, b):
    """Differenz a - b zweier 3-Vektoren."""
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def skaliere3(a, s):
    """3-Vektor mit Skalar multiplizieren."""
    return [a[0] * s, a[1] * s, a[2] * s]


def skal(a, b):
    """Skalarprodukt zweier 3-Vektoren.  Feste Summationsreihenfolge."""
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def kreuz(a, b):
    """Kreuzprodukt a x b."""
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def norm(a):
    """Euklidische Norm (L2) eines Vektors beliebiger Laenge."""
    s = 0.0
    for x in a:
        s += x * x
    return math.sqrt(s)


def norm1(a):
    """Betragssummennorm (L1).  Wird als Abbruchkriterium auf Delta-q
    verwendet -- so macht es auch der KDL-Loeser."""
    s = 0.0
    for x in a:
        s += abs(x)
    return s


def normiere3(a):
    """3-Vektor auf Laenge 1 bringen.

    Wirft ValueError bei Nullvektor.  Ein stillschweigend zurueckgegebener
    Nullvektor waere hier die gefaehrlichere Variante, weil er sich als
    plausible Achsrichtung tarnen wuerde.
    """
    n = norm(a)
    if n < EPS:
        raise ValueError("normiere3: Vektor hat Laenge 0, keine Richtung definiert")
    return [a[0] / n, a[1] / n, a[2] / n]


# ---------------------------------------------------------------------------
# Ebene 0b -- Matrixoperationen
# ---------------------------------------------------------------------------

def null_matrix(m, n):
    """m x n Nullmatrix."""
    return [[0.0] * n for _ in range(m)]


def einheit(n):
    """n x n Einheitsmatrix."""
    e = [[0.0] * n for _ in range(n)]
    for i in range(n):
        e[i][i] = 1.0
    return e


def diag(werte):
    """Diagonalmatrix aus einer Liste von Werten."""
    n = len(werte)
    d = [[0.0] * n for _ in range(n)]
    for i in range(n):
        d[i][i] = float(werte[i])
    return d


def transponiert(A):
    """Transponierte einer Matrix."""
    if not A:
        return []
    m = len(A)
    n = len(A[0])
    return [[A[i][j] for i in range(m)] for j in range(n)]


def mat_mal_mat(A, B):
    """Matrixprodukt A*B mit fester Summationsreihenfolge."""
    m = len(A)
    k = len(A[0]) if m else 0
    if len(B) != k:
        raise ValueError("mat_mal_mat: Dimensionen passen nicht (%d Spalten gegen %d Zeilen)"
                         % (k, len(B)))
    n = len(B[0]) if B else 0
    C = [[0.0] * n for _ in range(m)]
    for i in range(m):
        Ai = A[i]
        Ci = C[i]
        for j in range(n):
            s = 0.0
            for t in range(k):
                s += Ai[t] * B[t][j]
            Ci[j] = s
    return C


def mat_mal_vec(A, v):
    """Matrix mal Vektor."""
    if not A:
        return []
    if len(A[0]) != len(v):
        raise ValueError("mat_mal_vec: Dimensionen passen nicht (%d Spalten gegen %d Elemente)"
                         % (len(A[0]), len(v)))
    out = []
    for zeile in A:
        s = 0.0
        for t in range(len(v)):
            s += zeile[t] * v[t]
        out.append(s)
    return out


def mat_plus(A, B):
    """Summe zweier gleich grosser Matrizen."""
    if len(A) != len(B) or len(A[0]) != len(B[0]):
        raise ValueError("mat_plus: Dimensionen passen nicht")
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]


def mat_mal_skalar(A, s):
    """Matrix mit Skalar multiplizieren."""
    return [[x * s for x in zeile] for zeile in A]


def spur(A):
    """Spur einer quadratischen Matrix."""
    s = 0.0
    for i in range(len(A)):
        s += A[i][i]
    return s


def frobenius(A):
    """Frobeniusnorm einer Matrix."""
    s = 0.0
    for zeile in A:
        for x in zeile:
            s += x * x
    return math.sqrt(s)


def schief(v):
    """Schiefsymmetrische Matrix K zu einem 3-Vektor, so dass K*a = v x a."""
    return [[0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0]]


# ---------------------------------------------------------------------------
# Ebene 0c -- Vergleiche und Winkel
# ---------------------------------------------------------------------------

def winkel_normiert(a):
    """Winkel auf das halboffene Intervall (-pi, pi] abbilden."""
    a = math.fmod(a, 2.0 * math.pi)
    if a > math.pi:
        a -= 2.0 * math.pi
    elif a <= -math.pi:
        a += 2.0 * math.pi
    return a


def klemme(x, lo, hi):
    """Wert auf [lo, hi] begrenzen.

    Wird insbesondere vor acos verwendet, wo ein Argument von 1.0000000002
    durch Rundung sonst zu einem Domainfehler fuehrt.
    """
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def nahe(a, b, rel=1.0e-9, abso=1.0e-12):
    """Vergleich zweier Gleitkommazahlen, relativ und absolut."""
    d = abs(a - b)
    if d <= abso:
        return True
    return d <= rel * max(abs(a), abs(b))


def nahe_vektor(a, b, rel=1.0e-9, abso=1.0e-12):
    """Elementweiser Vergleich zweier gleich langer Vektoren."""
    if len(a) != len(b):
        return False
    for i in range(len(a)):
        if not nahe(a[i], b[i], rel, abso):
            return False
    return True


# ---------------------------------------------------------------------------
# Ebene 1a -- Elementardrehungen
# ---------------------------------------------------------------------------

def rot_x(w):
    """Drehmatrix um die x-Achse, Winkel in Radiant."""
    c = math.cos(w)
    s = math.sin(w)
    return [[1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c]]


def rot_y(w):
    """Drehmatrix um die y-Achse, Winkel in Radiant."""
    c = math.cos(w)
    s = math.sin(w)
    return [[c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c]]


def rot_z(w):
    """Drehmatrix um die z-Achse, Winkel in Radiant."""
    c = math.cos(w)
    s = math.sin(w)
    return [[c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0]]


# ---------------------------------------------------------------------------
# Ebene 1b -- Exponentialabbildung und Logarithmus auf SO(3)
# ---------------------------------------------------------------------------

def exp_so3(v):
    """Drehvektor (Achse mal Winkel) -> Drehmatrix.  Rodrigues-Formel.

        R = I + a*K + b*K*K     mit K = schief(v)

    Fuer grosse Winkel gilt a = sin(t)/t und b = (1-cos(t))/t^2.  Fuer
    t -> 0 werden beide Ausdruecke 0/0; dort wird auf die abgebrochene
    Reihenentwicklung umgeschaltet.  Damit ist die Funktion ueber den
    gesamten Bereich stabil, inklusive v = 0.
    """
    t = norm(v)
    if t < KLEIN:
        t2 = t * t
        a = 1.0 - t2 / 6.0
        b = 0.5 - t2 / 24.0
    else:
        a = math.sin(t) / t
        b = (1.0 - math.cos(t)) / (t * t)
    K = schief(v)
    KK = mat_mal_mat(K, K)
    R = einheit(3)
    for i in range(3):
        for j in range(3):
            R[i][j] += a * K[i][j] + b * KK[i][j]
    return R


def log_so3(R):
    """Drehmatrix -> Drehvektor (Achse mal Winkel), Betrag in [0, pi].

    Bewusst NICHT ueber t = acos((spur(R)-1)/2) und Achse aus (R-R^T):
    dieser direkte Weg bricht bei t nahe 0 (0/0) und bei t nahe pi
    (sin t -> 0) zusammen.  Stattdessen der Umweg ueber das Quaternion:

        t = 2*atan2(|vektorteil|, w)

    atan2 ist an beiden Enden gutartig.  Das Quaternion wird zuvor auf
    w >= 0 gebracht, damit stets der kuerzere der beiden aequivalenten
    Drehwege geliefert wird.
    """
    w, x, y, z = quat_von_matrix(R)
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    nv = math.sqrt(x * x + y * y + z * z)
    if nv < KLEIN:
        # Kleinwinkelfall: t ~ 2*nv/w, Achse = v/nv, also t*Achse = 2*v/w.
        # Die Division durch nv entfaellt damit vollstaendig.
        if w < EPS:
            raise ValueError("log_so3: entartetes Quaternion")
        return [2.0 * x / w, 2.0 * y / w, 2.0 * z / w]
    t = 2.0 * math.atan2(nv, w)
    f = t / nv
    return [f * x, f * y, f * z]


# ---------------------------------------------------------------------------
# Ebene 1c -- Quaternionen
# ---------------------------------------------------------------------------

def quat_von_matrix(R):
    """Drehmatrix -> Quaternion (w, x, y, z).

    Verfahren nach Shepperd: es wird derjenige der vier Zweige gewaehlt,
    dessen Nenner betragsmaessig am groessten ist.  Die naive Formel ueber
    die Spur allein verliert Stellen, sobald die Spur nahe -1 liegt.
    """
    s00, s01, s02 = R[0][0], R[0][1], R[0][2]
    s10, s11, s12 = R[1][0], R[1][1], R[1][2]
    s20, s21, s22 = R[2][0], R[2][1], R[2][2]
    sp = s00 + s11 + s22
    if sp > 0.0:
        s = math.sqrt(sp + 1.0) * 2.0
        w = 0.25 * s
        x = (s21 - s12) / s
        y = (s02 - s20) / s
        z = (s10 - s01) / s
    elif s00 > s11 and s00 > s22:
        s = math.sqrt(1.0 + s00 - s11 - s22) * 2.0
        w = (s21 - s12) / s
        x = 0.25 * s
        y = (s01 + s10) / s
        z = (s02 + s20) / s
    elif s11 > s22:
        s = math.sqrt(1.0 + s11 - s00 - s22) * 2.0
        w = (s02 - s20) / s
        x = (s01 + s10) / s
        y = 0.25 * s
        z = (s12 + s21) / s
    else:
        s = math.sqrt(1.0 + s22 - s00 - s11) * 2.0
        w = (s10 - s01) / s
        x = (s02 + s20) / s
        y = (s12 + s21) / s
        z = 0.25 * s
    return (w, x, y, z)


def matrix_von_quat(q):
    """Quaternion (w, x, y, z) -> Drehmatrix.  Das Quaternion wird zuvor
    normiert, damit auch leicht driftende Eingaben eine echte Drehmatrix
    ergeben."""
    w, x, y, z = quat_normiert(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [[1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)]]


def quat_mal_quat(a, b):
    """Hamilton-Produkt zweier Quaternionen."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_konjugiert(q):
    """Konjugiertes Quaternion.  Bei Einheitsquaternionen zugleich das
    Inverse."""
    return (q[0], -q[1], -q[2], -q[3])


def quat_normiert(q):
    """Quaternion auf Betrag 1 bringen."""
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < EPS:
        raise ValueError("quat_normiert: Quaternion hat Betrag 0")
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_von_achse_winkel(achse, winkel):
    """Einheitsachse und Winkel -> Quaternion."""
    a = normiere3(achse)
    h = 0.5 * winkel
    s = math.sin(h)
    return (math.cos(h), a[0] * s, a[1] * s, a[2] * s)


def achse_winkel_von_quat(q):
    """Quaternion -> (Achse, Winkel) mit Winkel in [0, pi].

    Bei verschwindendem Drehwinkel ist die Achse nicht definiert; es wird
    dann die z-Achse und Winkel 0 geliefert, damit Aufrufer nicht mit einem
    Nullvektor weiterrechnen.
    """
    w, x, y, z = quat_normiert(q)
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    nv = math.sqrt(x * x + y * y + z * z)
    if nv < KLEIN:
        return ([0.0, 0.0, 1.0], 0.0)
    winkel = 2.0 * math.atan2(nv, w)
    return ([x / nv, y / nv, z / nv], winkel)


# ---------------------------------------------------------------------------
# Ebene 1d -- Pruefung und Posendifferenz
# ---------------------------------------------------------------------------

def ist_rotation(R, tol=1.0e-9):
    """Prueft R^T*R = I und det(R) = +1.

    Diagnosefunktion.  Wird in den Tests als Invariante nach jeder
    Rotationsoperation aufgerufen.
    """
    if len(R) != 3 or any(len(z) != 3 for z in R):
        return False
    P = mat_mal_mat(transponiert(R), R)
    for i in range(3):
        for j in range(3):
            soll = 1.0 if i == j else 0.0
            if abs(P[i][j] - soll) > tol:
                return False
    d = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
         - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
         + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
    return abs(d - 1.0) <= tol


def twist_diff(R_a, p_a, R_b, p_b):
    """Posendifferenz von Pose a nach Pose b als 6-Vektor.

    Rueckgabe: [dx, dy, dz, wx, wy, wz]

    Der Translationsanteil ist die schlichte Differenz.  Der Rotationsanteil
    ist log(R_b * R_a^T), also ein Drehvektor im BASISFRAME -- nicht im
    Werkzeugframe und nicht als Eulerwinkeldifferenz.

    Die Wahl des Basisframes ist kein Geschmack, sondern Zwang: die unteren
    drei Zeilen der geometrischen Jacobi-Matrix sind die Gelenkachsen im
    Basisframe.  Wuerde der Orientierungsfehler in einem anderen Frame
    ausgedrueckt, passten Fehler und Jacobi-Matrix nicht zusammen und jede
    Iteration liefe in die falsche Richtung.

    Dies ist das Gegenstueck zu KDLs diff() auf KDL::Frame.
    """
    dp = sub3(p_b, p_a)
    dR = mat_mal_mat(R_b, transponiert(R_a))
    dw = log_so3(dR)
    return [dp[0], dp[1], dp[2], dw[0], dw[1], dw[2]]
