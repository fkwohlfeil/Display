# -*- coding: utf-8 -*-
"""test_mathe_jacobi.py -- Testsuite fuer mathe.py und jacobi.py.

Schicht 1 des Dreischichtmusters: reine Logiktests ohne Tk, ohne ROS,
ohne URDF.  Die Kinematik wird von einer Mock-Kette im Testmodul selbst
gestellt.

Aufruf:   python3 -B test_mathe_jacobi.py
"""

import math
import sys

import mathe as M
import jacobi as JA


# ---------------------------------------------------------------------------
# Mock-Kinematik: reine Testkette, kein URDF
# ---------------------------------------------------------------------------

class Kette:
    """Serielle Drehgelenkkette.

    Jedes Segment ist (R_versatz, p_versatz): die feste Lage des
    Gelenkframes gegenueber dem vorigen Gelenkframe.  Anschliessend dreht
    das Gelenk um seine lokale z-Achse.  Am Ende ein festes Werkzeug.
    """

    def __init__(self, segmente, werkzeug=None):
        self.segmente = segmente
        self.werkzeug = werkzeug or (M.einheit(3), [0.0, 0.0, 0.0])

    def durchlauf(self, q):
        """Liefert (R_tcp, p_tcp, gelenke) mit gelenke im Format von
        jacobi.jacobi()."""
        if len(q) != len(self.segmente):
            raise ValueError("Kette: %d Winkel, %d Segmente" % (len(q), len(self.segmente)))
        R = M.einheit(3)
        p = [0.0, 0.0, 0.0]
        gelenke = []
        for i, (R_v, p_v) in enumerate(self.segmente):
            p = M.add3(p, M.mat_mal_vec(R, p_v))
            R = M.mat_mal_mat(R, R_v)
            z = M.mat_mal_vec(R, [0.0, 0.0, 1.0])
            gelenke.append((JA.DREH, z, list(p)))
            R = M.mat_mal_mat(R, M.rot_z(q[i]))
        R_w, p_w = self.werkzeug
        p = M.add3(p, M.mat_mal_vec(R, p_w))
        R = M.mat_mal_mat(R, R_w)
        return R, p, gelenke

    def fk(self, q):
        R, p, _ = self.durchlauf(q)
        return R, p


def kette_planar_2r(a1, a2):
    """Planarer Zweigelenkarm in der xy-Ebene, beide Achsen z."""
    return Kette([(M.einheit(3), [0.0, 0.0, 0.0]),
                  (M.einheit(3), [a1, 0.0, 0.0])],
                 werkzeug=(M.einheit(3), [a2, 0.0, 0.0]))


def kette_7achs():
    """Siebenachsiger Arm mit abwechselnden Achsrichtungen.

    Bewusst mit krummen Massen, damit zufaellige Symmetrien keine Fehler
    verdecken.
    """
    seg = [
        (M.einheit(3), [0.000, 0.000, 0.1725]),
        (M.rot_x(math.pi / 2), [0.000, 0.000, 0.0000]),
        (M.rot_x(-math.pi / 2), [0.000, -0.213, 0.0000]),
        (M.rot_x(math.pi / 2), [0.000, 0.000, 0.0000]),
        (M.rot_x(-math.pi / 2), [0.000, -0.197, 0.0000]),
        (M.rot_x(math.pi / 2), [0.000, 0.000, 0.0000]),
        (M.rot_x(-math.pi / 2), [0.000, -0.081, 0.0000]),
    ]
    return Kette(seg, werkzeug=(M.rot_z(0.3), [0.011, 0.023, 0.1070]))


# ---------------------------------------------------------------------------
# Testgeruest
# ---------------------------------------------------------------------------

_FEHLER = []
_ANZAHL = [0]


def pruefe(bedingung, text):
    _ANZAHL[0] += 1
    if not bedingung:
        _FEHLER.append(text)


def pruefe_nahe(a, b, text, rel=1.0e-9, abso=1.0e-12):
    _ANZAHL[0] += 1
    if not M.nahe(a, b, rel, abso):
        _FEHLER.append("%s: %.17g gegen %.17g (Abweichung %.3g)" % (text, a, b, abs(a - b)))


# ---------------------------------------------------------------------------
# Ebene 0/1 -- Rotationen
# ---------------------------------------------------------------------------

def test_rotationsinvarianten():
    for w in (0.0, 0.1, 1.0, 2.0, 3.0, math.pi, -0.7):
        for f in (M.rot_x, M.rot_y, M.rot_z):
            pruefe(M.ist_rotation(f(w)), "ist_rotation fuer %s(%g)" % (f.__name__, w))


def test_exp_log_umkehr():
    """log(exp(v)) = v ueber den gesamten Winkelbereich, inklusive der
    beiden kritischen Enden 0 und pi."""
    achse = M.normiere3([0.3, -0.6, 0.74])
    winkel = [0.0, 1e-14, 1e-9, 1e-6, 1e-3, 0.5, 1.0, 2.0, 3.0,
              math.pi - 1e-6, math.pi - 1e-9]
    for t in winkel:
        v = M.skaliere3(achse, t)
        R = M.exp_so3(v)
        pruefe(M.ist_rotation(R, 1e-12), "exp_so3 liefert Rotation bei t=%g" % t)
        v2 = M.log_so3(R)
        pruefe(M.nahe_vektor(v, v2, rel=1e-7, abso=1e-12),
               "log(exp(v)) bei t=%g: %r gegen %r" % (t, v, v2))


def test_log_bei_pi():
    """Bei genau pi sind +v und -v aequivalent; geprueft wird der Betrag
    und die Achsrichtung bis aufs Vorzeichen."""
    achse = M.normiere3([1.0, 0.0, 0.0])
    v = M.skaliere3(achse, math.pi)
    R = M.exp_so3(v)
    v2 = M.log_so3(R)
    pruefe_nahe(M.norm(v2), math.pi, "Betrag bei t=pi", rel=1e-7)
    kos = abs(M.skal(M.normiere3(v2), achse))
    pruefe_nahe(kos, 1.0, "Achsrichtung bei t=pi", rel=1e-7)


def test_log_kuerzester_weg():
    """log_so3 muss stets den kuerzeren der beiden aequivalenten Drehwege
    liefern, also |log(R)| <= pi.

    Sondenluecke: die Achsrichtung entscheidet ueber das Vorzeichen von w
    im Quaternion.  Bei einer Achse mit negativer dominanter Komponente
    und Drehwinkel oberhalb von etwa 2 rad liefert quat_von_matrix ein
    Quaternion mit w < 0.  Ohne die Umschaltung auf w >= 0 kaeme dann der
    lange Weg heraus, also 2*pi - t statt t.  Mit Achsen, deren dominante
    Komponente positiv ist, faellt das nie auf.
    """
    achsen = [[-0.9, 0.2, 0.3], [0.2, -0.95, 0.1], [0.1, 0.2, -0.97],
              [-0.6, -0.6, -0.53], [0.9, 0.2, 0.3]]
    for a in achsen:
        achse = M.normiere3(a)
        for t in (2.0, 2.5, 3.0, 3.1):
            R = M.exp_so3(M.skaliere3(achse, t))
            v = M.log_so3(R)
            pruefe(M.norm(v) <= math.pi + 1.0e-9,
                   "log_so3 liefert langen Drehweg: |v|=%.6f bei t=%g, Achse %r"
                   % (M.norm(v), t, a))
            pruefe(M.nahe_vektor(v, M.skaliere3(achse, t), rel=1e-7, abso=1e-12),
                   "log_so3 bei t=%g, Achse %r: %r" % (t, a, v))


def test_quaternion_umkehr():
    achse = M.normiere3([-0.2, 0.9, 0.35])
    for t in (0.0, 1e-9, 0.4, 1.9, 3.0, math.pi):
        R = M.exp_so3(M.skaliere3(achse, t))
        q = M.quat_von_matrix(R)
        pruefe_nahe(math.sqrt(sum(x * x for x in q)), 1.0,
                    "Quaternionnorm bei t=%g" % t, rel=1e-12)
        R2 = M.matrix_von_quat(q)
        for i in range(3):
            for j in range(3):
                pruefe_nahe(R[i][j], R2[i][j], "quat-Umkehr [%d][%d] bei t=%g" % (i, j, t),
                            rel=1e-9, abso=1e-12)


def test_quaternion_produkt():
    """Das Quaternionprodukt muss der Matrizenmultiplikation entsprechen."""
    Ra = M.exp_so3([0.3, -0.2, 0.9])
    Rb = M.exp_so3([-1.1, 0.4, 0.15])
    qa = M.quat_von_matrix(Ra)
    qb = M.quat_von_matrix(Rb)
    Rc = M.mat_mal_mat(Ra, Rb)
    Rc2 = M.matrix_von_quat(M.quat_mal_quat(qa, qb))
    for i in range(3):
        for j in range(3):
            pruefe_nahe(Rc[i][j], Rc2[i][j], "quat-Produkt [%d][%d]" % (i, j), rel=1e-9)


def test_twist_diff():
    """twist_diff muss null liefern, wenn beide Posen gleich sind, und der
    Rotationsanteil muss im Basisframe liegen."""
    R = M.exp_so3([0.2, 0.5, -0.3])
    p = [0.1, -0.2, 0.35]
    d = M.twist_diff(R, p, R, p)
    for k in range(6):
        pruefe_nahe(d[k], 0.0, "twist_diff auf sich selbst, Komponente %d" % k, abso=1e-14)
    # Reine Drehung um Basis-z um 0.4 rad
    Rb = M.mat_mal_mat(M.rot_z(0.4), R)
    d = M.twist_diff(R, p, Rb, p)
    pruefe(M.nahe_vektor(d[3:], [0.0, 0.0, 0.4], rel=1e-9, abso=1e-12),
           "twist_diff Rotationsanteil im Basisframe: %r" % (d[3:],))


def test_winkel_normiert():
    faelle = [(0.0, 0.0), (math.pi, math.pi), (-math.pi, math.pi),
              (3.0 * math.pi, math.pi), (2.0 * math.pi + 0.3, 0.3),
              (-2.0 * math.pi - 0.3, -0.3)]
    for ein, soll in faelle:
        pruefe_nahe(M.winkel_normiert(ein), soll,
                    "winkel_normiert(%g)" % ein, rel=1e-12, abso=1e-12)


# ---------------------------------------------------------------------------
# Ebene 2 -- Jacobi-Matrix
# ---------------------------------------------------------------------------

def test_jacobi_gegen_differenz():
    """Kernpruefung: analytische gegen numerische Jacobi-Matrix.

    Prueft Vorzeichen, Achsreihenfolge, Bezugspunkt und Framewahl in einem
    Zug, ohne handeingetragene Sollwerte.
    """
    k = kette_7achs()
    stellungen = [
        [0.1, -0.4, 0.7, 0.9, -1.2, 0.35, 0.6],
        [0.0, 0.3, 0.0, -1.1, 0.0, 0.8, 0.0],
        [-1.3, 0.85, -0.25, 1.4, 0.6, -0.9, 1.1],
        [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
    ]
    for q in stellungen:
        _, p_tcp, gelenke = k.durchlauf(q)
        Ja = JA.jacobi(gelenke, p_tcp)
        Jn = JA.jacobi_numerisch(k.fk, q)
        skala = max(abs(x) for zeile in Ja for x in zeile)
        for r in range(6):
            for c in range(len(q)):
                abw = abs(Ja[r][c] - Jn[r][c])
                pruefe(abw <= 1.0e-7 * max(skala, 1.0),
                       "Jacobi [%d][%d] bei q=%r: analytisch %.12g, numerisch %.12g, "
                       "Abweichung %.3g" % (r, c, q, Ja[r][c], Jn[r][c], abw))


def test_jacobi_schubgelenk():
    """Ein Schubgelenk hat keinen Rotationsanteil und ist lageunabhaengig."""
    gelenke = [(JA.SCHUB, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0])]
    J1 = JA.jacobi(gelenke, [0.0, 0.0, 0.0])
    J2 = JA.jacobi(gelenke, [5.0, -3.0, 2.0])
    for r in range(6):
        pruefe_nahe(J1[r][0], J2[r][0], "Schubgelenk lageunabhaengig, Zeile %d" % r,
                    abso=1e-15)
    pruefe(J1[3][0] == 0.0 and J1[4][0] == 0.0 and J1[5][0] == 0.0,
           "Schubgelenk ohne Rotationsanteil")


def test_jacobi_unveraendert():
    """Reinheit: die Eingabe darf nicht veraendert werden."""
    k = kette_7achs()
    q = [0.2, 0.4, -0.3, 0.9, 0.1, -0.6, 0.25]
    _, p, gelenke = k.durchlauf(q)
    kopie = [(t, list(z), list(pp)) for (t, z, pp) in gelenke]
    p_kopie = list(p)
    JA.jacobi(gelenke, p)
    for i in range(len(gelenke)):
        pruefe(gelenke[i][1] == kopie[i][1] and gelenke[i][2] == kopie[i][2],
               "jacobi veraendert Gelenk %d" % i)
    pruefe(p == p_kopie, "jacobi veraendert den Bezugspunkt")


# ---------------------------------------------------------------------------
# Ebene 3 -- Kennzahlen
# ---------------------------------------------------------------------------

def test_manipulierbarkeit_analytisch():
    """Planarer 2R: w = a1 * a2 * |sin(q2)| in der xy-Ebene.

    Das ist der einzige Sollwert in der ganzen Suite, der von Hand
    hergeleitet wurde -- und deswegen der wertvollste.
    """
    a1, a2 = 0.37, 0.29
    k = kette_planar_2r(a1, a2)
    for q2 in (0.2, 0.7, 1.4, 2.6, -0.9):
        q = [0.31, q2]
        _, p, gelenke = k.durchlauf(q)
        J = JA.jacobi(gelenke, p)
        w = JA.manipulierbarkeit(JA.zeilen(J, (0, 1)))
        soll = a1 * a2 * abs(math.sin(q2))
        pruefe_nahe(w, soll, "planare Manipulierbarkeit bei q2=%g" % q2, rel=1e-9)


def test_manipulierbarkeit_singulaer():
    """In der Streckstellung (q2 = 0) muss w exakt gegen null gehen."""
    k = kette_planar_2r(0.37, 0.29)
    _, p, gelenke = k.durchlauf([0.5, 0.0])
    J = JA.jacobi(gelenke, p)
    w = JA.manipulierbarkeit(JA.zeilen(J, (0, 1)))
    pruefe(w < 1.0e-15, "Manipulierbarkeit in Streckstellung: %g" % w)


def test_manipulierbarkeit_zeilenpruefung():
    """Mehr Zeilen als Spalten muss einen Fehler werfen, nicht still null
    liefern."""
    k = kette_planar_2r(0.37, 0.29)
    _, p, gelenke = k.durchlauf([0.5, 0.6])
    J = JA.jacobi(gelenke, p)
    try:
        JA.manipulierbarkeit(J)          # 6 Zeilen, 2 Spalten
    except ValueError:
        pruefe(True, "Zeilenpruefung greift")
    else:
        pruefe(False, "manipulierbarkeit haette ValueError werfen muessen")


def test_determinante():
    """Determinante gegen von Hand gerechnete Faelle."""
    pruefe_nahe(JA.det_lu([[2.0]]), 2.0, "det 1x1")
    pruefe_nahe(JA.det_lu([[1.0, 2.0], [3.0, 4.0]]), -2.0, "det 2x2")
    pruefe_nahe(JA.det_lu([[6.0, 1.0, 1.0], [4.0, -2.0, 5.0], [2.0, 8.0, 7.0]]),
                -306.0, "det 3x3", rel=1e-12)
    pruefe_nahe(JA.det_lu(M.einheit(6)), 1.0, "det Einheit 6x6")
    pruefe_nahe(JA.det_lu([[1.0, 2.0], [2.0, 4.0]]), 0.0, "det singulaer", abso=1e-15)


def test_produkt_jjt_symmetrie():
    k = kette_7achs()
    _, p, gelenke = k.durchlauf([0.3, -0.5, 0.8, 1.0, -0.2, 0.44, 0.7])
    J = JA.jacobi(gelenke, p)
    P = JA.produkt_jjt(J)
    for i in range(6):
        for j in range(6):
            pruefe(P[i][j] == P[j][i],
                   "J*J^T exakt symmetrisch bei [%d][%d]" % (i, j))


def test_eigenwerte():
    """Eigenwertsumme gleich Spur, Rekonstruktion V*D*V^T gleich M,
    Eigenvektoren orthonormal."""
    k = kette_7achs()
    _, p, gelenke = k.durchlauf([0.3, -0.5, 0.8, 1.0, -0.2, 0.44, 0.7])
    J = JA.jacobi(gelenke, p)
    P = JA.produkt_jjt(J)
    werte, V = JA.eigen_symmetrisch(P)
    pruefe_nahe(sum(werte), M.spur(P), "Eigenwertsumme gleich Spur", rel=1e-9)
    for w in werte:
        pruefe(w >= -1.0e-12, "Eigenwert nicht negativ: %g" % w)
    # absteigend sortiert
    for i in range(len(werte) - 1):
        pruefe(werte[i] >= werte[i + 1], "Eigenwerte absteigend an Position %d" % i)
    # Orthonormalitaet
    VT = M.transponiert(V)
    I = M.mat_mal_mat(VT, V)
    for i in range(6):
        for j in range(6):
            soll = 1.0 if i == j else 0.0
            pruefe_nahe(I[i][j], soll, "Eigenvektoren orthonormal [%d][%d]" % (i, j),
                        rel=1e-8, abso=1e-9)
    # Rekonstruktion
    D = M.diag(werte)
    R = M.mat_mal_mat(M.mat_mal_mat(V, D), VT)
    skala = max(abs(x) for zeile in P for x in zeile)
    for i in range(6):
        for j in range(6):
            pruefe(abs(R[i][j] - P[i][j]) <= 1e-9 * max(skala, 1.0),
                   "Eigen-Rekonstruktion [%d][%d]: %g gegen %g" % (i, j, R[i][j], P[i][j]))


def test_singulaerwerte_gegen_determinante():
    """Produkt der Singulaerwerte muss gleich sqrt(det(J*J^T)) sein.

    Zwei voellig verschiedene Rechenwege auf dasselbe Ergebnis -- die
    schaerfste Kontrolle, die ohne Fremdbibliothek zu haben ist.
    """
    k = kette_7achs()
    for q in ([0.3, -0.5, 0.8, 1.0, -0.2, 0.44, 0.7],
              [1.1, 0.2, -0.9, 0.4, 0.75, -1.3, 0.05]):
        _, p, gelenke = k.durchlauf(q)
        J = JA.jacobi(gelenke, p)
        Jt = JA.zeilen(J, (0, 1, 2))
        s = JA.singulaerwerte(Jt)
        produkt = 1.0
        for x in s:
            produkt *= x
        pruefe_nahe(produkt, JA.manipulierbarkeit(Jt),
                    "Produkt der Singulaerwerte gleich Yoshikawa-Index", rel=1e-7)


def test_isotropie_und_kondition():
    k = kette_7achs()
    _, p, gelenke = k.durchlauf([0.3, -0.5, 0.8, 1.0, -0.2, 0.44, 0.7])
    Jt = JA.zeilen(JA.jacobi(gelenke, p), (0, 1, 2))
    kappa = JA.konditionszahl(Jt)
    iso = JA.isotropie(Jt)
    pruefe(kappa >= 1.0, "Konditionszahl mindestens 1: %g" % kappa)
    pruefe_nahe(iso, 1.0 / kappa, "Isotropie ist Kehrwert der Konditionszahl", rel=1e-9)


# ---------------------------------------------------------------------------
# Ebene 4 -- Cholesky und DLS
# ---------------------------------------------------------------------------

def test_cholesky():
    A = [[4.0, 12.0, -16.0], [12.0, 37.0, -43.0], [-16.0, -43.0, 98.0]]
    L = JA.cholesky(A)
    R = M.mat_mal_mat(L, M.transponiert(L))
    for i in range(3):
        for j in range(3):
            pruefe_nahe(R[i][j], A[i][j], "Cholesky-Rekonstruktion [%d][%d]" % (i, j),
                        rel=1e-12)
    b = [1.0, -2.0, 0.5]
    y = JA.loese_spd(L, b)
    Ay = M.mat_mal_vec(A, y)
    for i in range(3):
        pruefe_nahe(Ay[i], b[i], "loese_spd Rueckprobe %d" % i, rel=1e-10)


def test_cholesky_nicht_definit():
    """Sondenluecke: der indefinite Fall [[1,2],[2,1]] loest ohne eigene
    Pruefung einen Domainfehler in math.sqrt aus -- ebenfalls ein
    ValueError.  Ein blosses 'except ValueError' merkt also nicht, ob die
    eigene Pruefung noch da ist.  Deshalb wird hier zusaetzlich der
    SEMIDEFINITE Fall geprueft, der ohne Pruefung voellig fehlerfrei
    durchlaeuft und eine unbrauchbare Zerlegung liefert, sowie der Text
    der Meldung.
    """
    for A, kennung in (([[1.0, 2.0], [2.0, 1.0]], "indefinit"),
                       ([[1.0, 1.0], [1.0, 1.0]], "semidefinit"),
                       ([[0.0, 0.0], [0.0, 1.0]], "Nullzeile")):
        try:
            JA.cholesky(A)
        except ValueError as e:
            pruefe("nicht positiv definit" in str(e),
                   "Cholesky-Meldung stammt aus der eigenen Pruefung (%s): %s"
                   % (kennung, e))
        else:
            pruefe(False, "Cholesky haette bei %s einen ValueError werfen muessen"
                   % kennung)


def test_dls_fern_der_singularitaet():
    """Fern der Singularitaet und mit kleinem lambda muss J*dq ~ dx gelten."""
    k = kette_7achs()
    q = [0.3, -0.5, 0.8, 1.0, -0.2, 0.44, 0.7]
    _, p, gelenke = k.durchlauf(q)
    J = JA.jacobi(gelenke, p)
    dx = [0.001, -0.0005, 0.002, 0.0, 0.0, 0.0]
    dq = JA.dls_schritt(J, dx, lam=1.0e-6)
    erreicht = M.mat_mal_vec(J, dq)
    for i in range(6):
        pruefe(abs(erreicht[i] - dx[i]) <= 1.0e-7,
               "DLS erreicht dx, Komponente %d: %g gegen %g" % (i, erreicht[i], dx[i]))


def test_dls_daempfung_begrenzt():
    """Nahe der Singularitaet muss ein groesseres lambda den Gelenkschritt
    verkleinern.  Das ist die eigentliche Zusage der Daempfung."""
    k = kette_planar_2r(0.37, 0.29)
    _, p, gelenke = k.durchlauf([0.4, 1.0e-4])       # fast gestreckt
    J = JA.zeilen(JA.jacobi(gelenke, p), (0, 1))
    dx = [0.001, 0.001]
    n_klein = M.norm(JA.dls_schritt(J, dx, lam=1.0e-8))
    n_gross = M.norm(JA.dls_schritt(J, dx, lam=1.0e-2))
    pruefe(n_gross < n_klein,
           "groesseres lambda begrenzt den Schritt: %g gegen %g" % (n_gross, n_klein))
    pruefe(n_klein > 1.0,
           "ohne Daempfung entsteht ein grosser Schritt: %g" % n_klein)


def test_dls_gewichtung_orientierung():
    """Mit kleinem Orientierungsgewicht muss der Positionsfehler kleiner und
    der Orientierungsfehler groesser werden als bei gleicher Gewichtung.

    Das ist die Kernzusage fuer 'Position exakt, Orientierung darf leiden'.
    """
    k = kette_7achs()
    q = [0.0, 0.9, 0.0, 1.2, 0.0, 0.6, 0.0]
    _, p, gelenke = k.durchlauf(q)
    J = JA.jacobi(gelenke, p)
    dx = [0.002, 0.001, -0.001, 0.02, -0.015, 0.01]
    lam = 5.0e-2
    dq_gleich = JA.dls_schritt(J, dx, lam=lam)
    dq_pos = JA.dls_schritt(J, dx, lam=lam, gew_kart=[1.0, 1.0, 1.0, 0.05, 0.05, 0.05])
    e_gleich = M.mat_mal_vec(J, dq_gleich)
    e_pos = M.mat_mal_vec(J, dq_pos)
    fehler_pos_gleich = M.norm([e_gleich[i] - dx[i] for i in range(3)])
    fehler_pos_gew = M.norm([e_pos[i] - dx[i] for i in range(3)])
    fehler_rot_gleich = M.norm([e_gleich[i] - dx[i] for i in range(3, 6)])
    fehler_rot_gew = M.norm([e_pos[i] - dx[i] for i in range(3, 6)])
    pruefe(fehler_pos_gew < fehler_pos_gleich,
           "Positionsfehler sinkt durch Gewichtung: %g gegen %g"
           % (fehler_pos_gew, fehler_pos_gleich))
    pruefe(fehler_rot_gew > fehler_rot_gleich,
           "Orientierungsfehler steigt durch Gewichtung: %g gegen %g"
           % (fehler_rot_gew, fehler_rot_gleich))


def test_dls_gelenkgewichtung():
    """Die Gelenkgewichtung darf die erreichte kartesische Bewegung nicht
    veraendern, nur ihre Aufteilung auf die Gelenke.

    Sondenluecke: die Gewichtung geht zweimal ein -- einmal beim Aufbau von
    A und einmal beim abschliessenden dq = W_q * A^T * y.  Faellt der
    zweite Faktor weg, bleibt die Rechnung formal gueltig, liefert aber
    eine um den Gewichtsfaktor verfaelschte Bewegung.  Mit einheitlichem
    Gewicht ungleich 1 wird das sofort sichtbar.
    """
    k = kette_7achs()
    q = [0.3, -0.5, 0.8, 1.0, -0.2, 0.44, 0.7]
    _, p, gelenke = k.durchlauf(q)
    J = JA.jacobi(gelenke, p)
    dx = [0.001, -0.0005, 0.002, 0.0003, -0.0002, 0.0001]
    for g in (0.5, 2.0, 0.25):
        dq = JA.dls_schritt(J, dx, lam=1.0e-7, gew_gelenk=[g] * len(q))
        erreicht = M.mat_mal_vec(J, dq)
        for i in range(6):
            pruefe(abs(erreicht[i] - dx[i]) <= 1.0e-7,
                   "einheitliche Gelenkgewichtung %g veraendert die Bewegung, "
                   "Komponente %d: %g gegen %g" % (g, i, erreicht[i], dx[i]))
    # Semantik: ein niedrig gewichtetes Gelenk soll sich weniger bewegen.
    gew = [1.0] * len(q)
    gew[3] = 0.05
    dq_gleich = JA.dls_schritt(J, dx, lam=1.0e-4)
    dq_gebremst = JA.dls_schritt(J, dx, lam=1.0e-4, gew_gelenk=gew)
    pruefe(abs(dq_gebremst[3]) < abs(dq_gleich[3]),
           "niedriges Gelenkgewicht bremst Achse 4: %g gegen %g"
           % (dq_gebremst[3], dq_gleich[3]))


def test_lambda_chiaverini():
    lam_max, lam_basis, ws = 0.05, 1.0e-4, 0.01
    pruefe_nahe(JA.lambda_chiaverini(0.0, ws, lam_max, lam_basis), lam_max + lam_basis,
                "lambda bei w=0")
    pruefe_nahe(JA.lambda_chiaverini(ws, ws, lam_max, lam_basis), lam_basis,
                "lambda an der Schwelle")
    pruefe_nahe(JA.lambda_chiaverini(10.0 * ws, ws, lam_max, lam_basis), lam_basis,
                "lambda weit oberhalb der Schwelle")
    a = JA.lambda_chiaverini(0.25 * ws, ws, lam_max, lam_basis)
    b = JA.lambda_chiaverini(0.75 * ws, ws, lam_max, lam_basis)
    pruefe(a > b, "lambda faellt monoton mit wachsendem w")


def test_nullraum():
    """Bei sieben Achsen ist der Nullraum eindimensional: J*N*v muss
    verschwinden."""
    k = kette_7achs()
    _, p, gelenke = k.durchlauf([0.3, -0.5, 0.8, 1.0, -0.2, 0.44, 0.7])
    J = JA.jacobi(gelenke, p)
    N = JA.nullraum_projektor(J, lam=1.0e-8)
    v = [1.0, -0.4, 0.7, 0.2, -1.1, 0.3, 0.9]
    dq = M.mat_mal_vec(N, v)
    rest = M.mat_mal_vec(J, dq)
    pruefe(M.norm(rest) < 1.0e-5,
           "J*N*v verschwindet, Restnorm %g" % M.norm(rest))
    pruefe(M.norm(dq) > 1.0e-3,
           "Nullraumbewegung ist nicht trivial null, Norm %g" % M.norm(dq))


# ---------------------------------------------------------------------------
# Determinismus
# ---------------------------------------------------------------------------

def test_determinismus():
    """Zweimaliger Aufruf mit identischer Eingabe muss bitgleiche Ausgabe
    liefern.  Geprueft ueber repr(), nicht ueber Toleranzvergleich."""
    k = kette_7achs()
    q = [0.3, -0.5, 0.8, 1.0, -0.2, 0.44, 0.7]
    _, p, gelenke = k.durchlauf(q)
    J1 = JA.jacobi(gelenke, p)
    J2 = JA.jacobi(gelenke, p)
    pruefe(repr(J1) == repr(J2), "jacobi bitgleich bei Wiederholung")
    w1 = JA.manipulierbarkeit_trans(J1)
    w2 = JA.manipulierbarkeit_trans(J2)
    pruefe(repr(w1) == repr(w2), "manipulierbarkeit_trans bitgleich")
    s1 = JA.singulaerwerte(JA.zeilen(J1, (0, 1, 2)))
    s2 = JA.singulaerwerte(JA.zeilen(J2, (0, 1, 2)))
    pruefe(repr(s1) == repr(s2), "singulaerwerte bitgleich")
    d1 = JA.dls_schritt(J1, [0.001, 0.0, 0.0, 0.0, 0.0, 0.0], lam=1e-3)
    d2 = JA.dls_schritt(J2, [0.001, 0.0, 0.0, 0.0, 0.0, 0.0], lam=1e-3)
    pruefe(repr(d1) == repr(d2), "dls_schritt bitgleich")


# ---------------------------------------------------------------------------
# Ablauf
# ---------------------------------------------------------------------------

def alle_tests():
    return [f for name, f in sorted(globals().items())
            if name.startswith("test_") and callable(f)]


def main():
    for f in alle_tests():
        try:
            f()
        except Exception as e:                       # noqa: BLE001
            _FEHLER.append("%s hat eine Ausnahme ausgeloest: %r" % (f.__name__, e))
    print("Pruefungen: %d" % _ANZAHL[0])
    if _FEHLER:
        print("FEHLGESCHLAGEN: %d" % len(_FEHLER))
        for t in _FEHLER[:40]:
            print("  - " + t)
        if len(_FEHLER) > 40:
            print("  ... und %d weitere" % (len(_FEHLER) - 40))
        return 1
    print("alle gruen")
    return 0


if __name__ == "__main__":
    sys.exit(main())
