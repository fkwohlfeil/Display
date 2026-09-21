# -*- coding: utf-8 -*-
"""jacobi.py -- Jacobi-Matrix, Singularitaetskennzahlen und gedaempfte
Ausgleichsrechnung.

Reines Rechenmodul.  Keine Abhaengigkeit auf Tkinter, ROS, NumPy oder auf
urdf_gui2_*.py.  Baut ausschliesslich auf mathe.py auf.

Abgrenzung
----------
Dieses Modul ist DIAGNOSTISCH und liefert Rechenbausteine.  Es berechnet
keine Posen, es fuehrt keine Vorwaertskinematik aus und es ist keine
alternative inverse Kinematik.  Gelenkwinkel bleiben die Wahrheit, die
Vorwaertskinematik der GUI bleibt die einzige Quelle fuer Posen, und
Zielstellungen werden weiterhin von MoveIt bestimmt.

Die Funktion dls_schritt() liefert einen einzelnen Gelenkschritt.  Wer
daraus eine Bahn integriert, verlaesst diese Abgrenzung bewusst und muss
jede erzeugte Gelenkstellung mit der eigenen Vorwaertskinematik
zurueckrechnen und gegen die Toleranz pruefen.

Einheiten
---------
Alles in SI.  Daraus folgt eine Eigenschaft, die in den Unterlagen stehen
muss: die oberen drei Zeilen der Jacobi-Matrix haben die Einheit m/rad, die
unteren drei sind dimensionslos.  Ein Manipulierbarkeitsindex ueber alle
sechs Zeilen ist deshalb NICHT skaleninvariant -- in Millimetern gerechnet
aendert sich sein Zahlenwert um den Faktor 1e9.  Deswegen sind
manipulierbarkeit_trans() und manipulierbarkeit_rot() die empfohlene
Schnittstelle und manipulierbarkeit() nur der allgemeine Unterbau.
"""

import math

from mathe import (EPS, KLEIN, kreuz, sub3, norm, einheit, transponiert,
                   mat_mal_mat, mat_mal_vec, twist_diff)

__all__ = [
    "DREH", "SCHUB",
    "jacobi", "jacobi_numerisch", "zeilen",
    "produkt_jjt", "det_lu", "cholesky", "loese_spd",
    "manipulierbarkeit", "manipulierbarkeit_trans", "manipulierbarkeit_rot",
    "eigen_symmetrisch", "singulaerwerte", "konditionszahl", "isotropie",
    "ellipsoid",
    "lambda_chiaverini", "dls_schritt", "nullraum_projektor",
]

DREH = "dreh"
SCHUB = "schub"


# ---------------------------------------------------------------------------
# Ebene 2 -- Jacobi-Matrix
# ---------------------------------------------------------------------------

def jacobi(gelenke, p_bezug):
    """Geometrische Jacobi-Matrix im Basisframe, 6 x n.

    Parameter
    ---------
    gelenke : Liste von (typ, z, p)
        Ein Eintrag je aktivem Gelenk, in Kettenreihenfolge.
        typ ist DREH oder SCHUB.
        z   ist die Gelenkachse als Einheitsvektor im Basisframe.
        p   ist der Gelenkursprung im Basisframe.
        Beide Groessen fallen im Durchlauf der Vorwaertskinematik ohnehin
        an; dieses Modul berechnet sie bewusst nicht selbst, damit es keine
        zweite Quelle fuer die Kinematik gibt.
    p_bezug : Bezugspunkt im Basisframe, ueblicherweise der TCP.

    Rueckgabe
    ---------
    6 x n Matrix.  Zeilen 0..2 translatorisch, Zeilen 3..5 rotatorisch.

    Spalte i fuer ein Drehgelenk:   ( z_i x (p_bezug - p_i) ,  z_i )
    Spalte i fuer ein Schubgelenk:  ( z_i                   ,  0   )

    Achtung Bezugspunkt: das Kreuzprodukt haengt davon ab, ob auf den
    Flansch oder auf den TCP gerechnet wird.  Der gewaehlte Bezugspunkt
    gehoert deshalb in jede Anzeige und in jede gespeicherte Kennzahl,
    sonst ist spaeter nicht erklaerbar, warum sich der Wert beim
    Werkzeugwechsel aendert, obwohl die Achsen stehen.
    """
    n = len(gelenke)
    J = [[0.0] * n for _ in range(6)]
    for i in range(n):
        typ, z, p = gelenke[i]
        if typ == DREH:
            r = sub3(p_bezug, p)
            oben = kreuz(z, r)
            J[0][i] = oben[0]
            J[1][i] = oben[1]
            J[2][i] = oben[2]
            J[3][i] = z[0]
            J[4][i] = z[1]
            J[5][i] = z[2]
        elif typ == SCHUB:
            J[0][i] = z[0]
            J[1][i] = z[1]
            J[2][i] = z[2]
            # Zeilen 3..5 bleiben null
        else:
            raise ValueError("jacobi: unbekannter Gelenktyp %r an Position %d" % (typ, i))
    return J


def jacobi_numerisch(fk, q, h=1.0e-6):
    """Jacobi-Matrix durch zentrale Differenz.  Nur zur Pruefung.

    Parameter
    ---------
    fk : Funktion q -> (R, p) der Bezugspose im Basisframe.
    q  : Gelenkwinkel.
    h  : Schrittweite.  1e-6 ist der Kompromiss zwischen Abbruch- und
         Rundungsfehler bei doppelter Genauigkeit.

    Der Rotationsanteil wird als Twist-Differenz gebildet, nicht als
    Differenz von Eulerwinkeln.  Eine Winkeldifferenz waere
    konventionsabhaengig und wuerde an der Konventionsgrenze auseinander-
    fallen -- der Test wuerde dann Fehler melden, wo keine sind.

    Diese Funktion ist die eigentliche Absicherung der analytischen
    Jacobi-Matrix: sie prueft Vorzeichen, Achsreihenfolge, Bezugspunkt und
    Framewahl in einem Zug, ohne dass irgendwo ein Sollwert von Hand
    eingetragen werden muss.
    """
    n = len(q)
    J = [[0.0] * n for _ in range(6)]
    for i in range(n):
        qp = list(q)
        qm = list(q)
        qp[i] += h
        qm[i] -= h
        Rp, pp = fk(qp)
        Rm, pm = fk(qm)
        d = twist_diff(Rm, pm, Rp, pp)
        for r in range(6):
            J[r][i] = d[r] / (2.0 * h)
    return J


def zeilen(J, indizes):
    """Teilmatrix aus ausgewaehlten Zeilen.

    zeilen(J, (0,1,2)) liefert den translatorischen Block,
    zeilen(J, (3,4,5)) den rotatorischen.
    """
    return [list(J[i]) for i in indizes]


# ---------------------------------------------------------------------------
# Ebene 3a -- Determinante und Produkt J*J^T
# ---------------------------------------------------------------------------

def produkt_jjt(J):
    """J * J^T, symmetrisch und positiv semidefinit.

    Es wird nur das obere Dreieck berechnet und gespiegelt.  Das halbiert
    nicht nur den Aufwand, sondern garantiert exakte Symmetrie -- sonst
    koennen J[i]*J[j] und J[j]*J[i] durch unterschiedliche
    Summationsreihenfolge im letzten Bit auseinanderlaufen, was das
    Eigenwertverfahren stoert.
    """
    m = len(J)
    n = len(J[0]) if m else 0
    M = [[0.0] * m for _ in range(m)]
    for i in range(m):
        Ji = J[i]
        for j in range(i, m):
            Jj = J[j]
            s = 0.0
            for t in range(n):
                s += Ji[t] * Jj[t]
            M[i][j] = s
            M[j][i] = s
    return M


def det_lu(M):
    """Determinante ueber LU-Zerlegung mit Teilpivotisierung.

    Die Zeilenvertauschungen sind eine deterministische Funktion der
    Eingabe; das Verfahren ist damit trotz Pivotisierung bitgenau
    reproduzierbar.
    """
    n = len(M)
    if n == 0:
        return 1.0
    if any(len(z) != n for z in M):
        raise ValueError("det_lu: Matrix ist nicht quadratisch")
    A = [list(z) for z in M]          # Eingabe bleibt unveraendert
    vorzeichen = 1.0
    for k in range(n):
        # Teilpivotisierung: betragsgroesstes Element der Restspalte
        p = k
        groesst = abs(A[k][k])
        for i in range(k + 1, n):
            v = abs(A[i][k])
            if v > groesst:
                groesst = v
                p = i
        if groesst < EPS:
            return 0.0               # Matrix ist singulaer
        if p != k:
            A[k], A[p] = A[p], A[k]
            vorzeichen = -vorzeichen
        pivot = A[k][k]
        for i in range(k + 1, n):
            f = A[i][k] / pivot
            if f != 0.0:
                Ai = A[i]
                Ak = A[k]
                for j in range(k, n):
                    Ai[j] -= f * Ak[j]
    d = vorzeichen
    for k in range(n):
        d *= A[k][k]
    return d


def cholesky(M):
    """Cholesky-Zerlegung M = L * L^T fuer symmetrisch positiv definite M.

    Wirft ValueError, wenn M nicht positiv definit ist.  Das ist Absicht:
    im vorgesehenen Einsatz (Ebene 4) garantiert der Daempfungsterm
    lambda^2 * I die positive Definitheit, ein Fehlschlag ist also ein
    echter Programmfehler und kein zu behandelnder Betriebsfall.
    """
    n = len(M)
    L = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            s = 0.0
            for k in range(j):
                s += L[i][k] * L[j][k]
            if i == j:
                rest = M[i][i] - s
                if rest <= 0.0:
                    raise ValueError("cholesky: Matrix nicht positiv definit "
                                     "(Zeile %d, Restwert %g)" % (i, rest))
                L[i][j] = math.sqrt(rest)
            else:
                L[i][j] = (M[i][j] - s) / L[j][j]
    return L


def loese_spd(L, b):
    """Loest L*L^T*y = b durch Vorwaerts- und Rueckwaertseinsetzen.

    Es wird nie eine Matrix invertiert, sondern stets ein
    Gleichungssystem geloest -- stabiler, schneller und kuerzer.
    """
    n = len(L)
    if len(b) != n:
        raise ValueError("loese_spd: rechte Seite hat Laenge %d statt %d" % (len(b), n))
    # Vorwaerts: L*z = b
    z = [0.0] * n
    for i in range(n):
        s = b[i]
        for k in range(i):
            s -= L[i][k] * z[k]
        z[i] = s / L[i][i]
    # Rueckwaerts: L^T*y = z
    y = [0.0] * n
    for i in range(n - 1, -1, -1):
        s = z[i]
        for k in range(i + 1, n):
            s -= L[k][i] * y[k]
        y[i] = s / L[i][i]
    return y


# ---------------------------------------------------------------------------
# Ebene 3b -- Kennzahlen
# ---------------------------------------------------------------------------

def manipulierbarkeit(J):
    """Yoshikawa-Index w = sqrt(det(J * J^T)).

    Voraussetzung: Zeilenzahl <= Spaltenzahl.  Sonst ist J*J^T zwangslaeufig
    singulaer und der Index waere immer null -- ein haeufiger Anfaengerfehler
    beim planaren Zweigelenkarm, wo man den vollen 3 x 2 Block einsetzt
    statt der beiden ebenen Zeilen.

    Der Index ist proportional zum Volumen des Geschwindigkeitsellipsoids.
    Daraus folgt seine bekannte Schwaeche: das Volumen kann gross bleiben,
    obwohl eine einzelne Halbachse bereits gegen null geht, wenn die
    anderen lang sind.  Als Naeheanzeige zur Singularitaet ist er deshalb
    ein spaetes Warnsignal.  Die schaerfere Groesse ist der kleinste
    Singulaerwert aus singulaerwerte().
    """
    m = len(J)
    n = len(J[0]) if m else 0
    if m > n:
        raise ValueError("manipulierbarkeit: %d Zeilen bei nur %d Spalten; "
                         "J*J^T waere zwangslaeufig singulaer" % (m, n))
    d = det_lu(produkt_jjt(J))
    if d <= 0.0:
        return 0.0                   # kleine negative Werte sind Rundung
    return math.sqrt(d)


def manipulierbarkeit_trans(J):
    """Translatorische Manipulierbarkeit aus den Zeilen 0..2.  Einheit m^3.

    Geometrisch das Volumen des translatorischen Geschwindigkeits-
    ellipsoids.  Diese Groesse faellt bei der Streckstellung.
    """
    return manipulierbarkeit(zeilen(J, (0, 1, 2)))


def manipulierbarkeit_rot(J):
    """Rotatorische Manipulierbarkeit aus den Zeilen 3..5.  Dimensionslos.

    Diese Groesse faellt bei der Handachsensingularitaet.  Die Trennung von
    der translatorischen Kennzahl trennt damit zugleich die
    Singularitaetstypen, was einer gemischten Zahl didaktisch ueberlegen
    ist -- und nebenbei das Einheitenproblem aufloest.
    """
    return manipulierbarkeit(zeilen(J, (3, 4, 5)))


def eigen_symmetrisch(M, max_umlaeufe=60, tol=1.0e-14):
    """Eigenwerte und Eigenvektoren einer symmetrischen Matrix.

    Zyklisches Jacobi-Verfahren mit fester Durchlaufreihenfolge (p < q,
    zeilenweise).  Liefert (eigenwerte, eigenvektoren), wobei die
    Eigenvektoren SPALTEN der zurueckgegebenen Matrix sind, absteigend nach
    Eigenwert sortiert.

    Kein NumPy, keine SVD.  Fuer die hier auftretenden 3x3- und
    6x6-Matrizen ist das Verfahren schnell, kurz und quadratisch
    konvergent.
    """
    n = len(M)
    A = [list(z) for z in M]
    V = einheit(n)
    fro = 0.0
    for zeile in A:
        for x in zeile:
            fro += x * x
    fro = math.sqrt(fro)
    schranke = tol * max(fro, 1.0)

    for _ in range(max_umlaeufe):
        ausser = 0.0
        for i in range(n):
            for j in range(i + 1, n):
                ausser += A[i][j] * A[i][j]
        if math.sqrt(2.0 * ausser) <= schranke:
            break
        for p in range(n - 1):
            for q in range(p + 1, n):
                apq = A[p][q]
                if abs(apq) <= schranke * 1.0e-3:
                    continue
                theta = (A[q][q] - A[p][p]) / (2.0 * apq)
                if theta >= 0.0:
                    t = 1.0 / (theta + math.sqrt(theta * theta + 1.0))
                else:
                    t = -1.0 / (-theta + math.sqrt(theta * theta + 1.0))
                c = 1.0 / math.sqrt(t * t + 1.0)
                s = t * c
                # Zeilen und Spalten p, q drehen
                for k in range(n):
                    akp = A[k][p]
                    akq = A[k][q]
                    A[k][p] = c * akp - s * akq
                    A[k][q] = s * akp + c * akq
                for k in range(n):
                    apk = A[p][k]
                    aqk = A[q][k]
                    A[p][k] = c * apk - s * aqk
                    A[q][k] = s * apk + c * aqk
                for k in range(n):
                    vkp = V[k][p]
                    vkq = V[k][q]
                    V[k][p] = c * vkp - s * vkq
                    V[k][q] = s * vkp + c * vkq

    paare = [(A[i][i], i) for i in range(n)]
    # Deterministische Sortierung: absteigend nach Eigenwert, bei Gleichstand
    # nach urspruenglichem Index.
    paare.sort(key=lambda pi: (-pi[0], pi[1]))
    werte = [p[0] for p in paare]
    vektoren = [[V[z][paare[s][1]] for s in range(n)] for z in range(n)]
    return werte, vektoren


def singulaerwerte(J):
    """Singulaerwerte von J, absteigend sortiert.

    Berechnet als Wurzeln der Eigenwerte von J*J^T.  Keine SVD des
    rechteckigen J noetig.  Negative Eigenwerte aus Rundung werden auf null
    geklemmt.
    """
    werte, _ = eigen_symmetrisch(produkt_jjt(J))
    return [math.sqrt(w) if w > 0.0 else 0.0 for w in werte]


def konditionszahl(J):
    """kappa = sigma_max / sigma_min.

    Liefert float('inf') bei exakter Singularitaet.  Das ist die Groesse,
    auf die MoveIt Servo seine Schwellwerte legt (dort voreingestellt 17
    zum Abbremsen und 30 zum Halt).
    """
    s = singulaerwerte(J)
    if not s:
        return float("inf")
    smin = s[-1]
    if smin < EPS:
        return float("inf")
    return s[0] / smin


def isotropie(J):
    """sigma_min / sigma_max, also der Kehrwert der Konditionszahl.

    Wert zwischen 0 und 1.  Achtung bei der Benennung in Unterlagen: MoveIt
    nennt genau diese Groesse getManipulability(), waehrend
    getManipulabilityIndex() den Yoshikawa-Index meint.  Zwei verschiedene
    Groessen mit fast gleichem Namen -- eine bekannte Verwechslungsquelle.
    """
    s = singulaerwerte(J)
    if not s or s[0] < EPS:
        return 0.0
    return s[-1] / s[0]


def ellipsoid(J):
    """Hauptachsen des Geschwindigkeitsellipsoids.

    Rueckgabe (halbachsen, richtungen): halbachsen sind die Singulaerwerte,
    richtungen ist eine Matrix, deren Spalten die zugehoerigen
    Achsrichtungen sind.  Direkt zeichenbar, etwa im Bahnviewer.
    """
    werte, vektoren = eigen_symmetrisch(produkt_jjt(J))
    halb = [math.sqrt(w) if w > 0.0 else 0.0 for w in werte]
    return halb, vektoren


# ---------------------------------------------------------------------------
# Ebene 4 -- Gedaempfte gewichtete Ausgleichsrechnung
# ---------------------------------------------------------------------------

def lambda_chiaverini(w, w_schwelle, lam_max, lam_basis=0.0):
    """Adaptive Daempfung nach Chiaverini.

        lambda(q) = lam_max * (1 - min(w/w_schwelle, 1))^2 + lam_basis

    Fern der Singularitaet ist lambda ~ lam_basis, es wird also kaum
    Genauigkeit verschenkt.  In Singularitaetsnaehe waechst die Daempfung
    quadratisch an.  Ein konstantes lambda kostet dagegen ueberall
    Genauigkeit, auch dort, wo es nicht noetig waere.

    w ist ueblicherweise manipulierbarkeit_trans() oder _rot(), w_schwelle
    ist robotersspezifisch und muss ueber den Arbeitsraum ausgemessen
    werden -- es gibt dafuer keinen allgemeingueltigen Wert, weil die obere
    Schranke von w von den Gliedlaengen abhaengt.
    """
    if w_schwelle <= 0.0:
        raise ValueError("lambda_chiaverini: w_schwelle muss positiv sein")
    v = w / w_schwelle
    if v > 1.0:
        v = 1.0
    rest = 1.0 - v
    return lam_max * rest * rest + lam_basis


def dls_schritt(J, dx, lam, gew_kart=None, gew_gelenk=None):
    """Ein Gelenkschritt aus gedaempfter gewichteter Ausgleichsrechnung.

        A  = W_x * J * W_q
        M  = A * A^T + lambda^2 * I
        y  = M^-1 * (W_x * dx)          (ueber Cholesky, nicht invertiert)
        dq = W_q * A^T * y

    Mit W_x = W_q = I ergibt das die klassische Form
    dq = J^T (J J^T + lambda^2 I)^-1 dx.

    Parameter
    ---------
    J   : 6 x n Jacobi-Matrix.
    dx  : gewuenschte kartesische Aenderung, 6-Vektor in der Reihenfolge
          [dx, dy, dz, wx, wy, wz], Rotationsanteil als Drehvektor im
          Basisframe (Ausgabe von twist_diff).
    lam : Daempfung.  Sie begrenzt den Kehrwert kleiner Singulaerwerte und
          verhindert damit, dass in Singularitaetsnaehe grosse
          Gelenkschritte entstehen.
    gew_kart : 6 Gewichte.  Voreinstellung alle 1.  Fuer "Position exakt,
          Orientierung darf nachgeben" waehlt man [1,1,1,wo,wo,wo] mit
          kleinem wo, etwa 0.05 bis 0.2.  Der Ausgleich schiebt den Fehler
          dann dorthin, wo er billiger ist.
    gew_gelenk : n Gewichte.  Ein Gewicht kleiner 1 bedeutet: dieses Gelenk
          soll sich weniger bewegen.  KDL setzt an dieser Stelle 0.01 fuer
          ein Gelenk, das gerade gegen seinen Anschlag laeuft.

    WICHTIG: Diese Funktion garantiert nichts ueber die erreichte Pose.  Wer
    daraus eine Bahn integriert, muss jede Stellung mit der eigenen
    Vorwaertskinematik zurueckrechnen und die Positionsabweichung gegen die
    Toleranz pruefen.  Der lokale Rechenweg darf nie die letzte Instanz
    sein.
    """
    m = len(J)
    n = len(J[0]) if m else 0
    if len(dx) != m:
        raise ValueError("dls_schritt: dx hat Laenge %d, erwartet %d" % (len(dx), m))
    if lam <= 0.0:
        raise ValueError("dls_schritt: lambda muss positiv sein, sonst ist die "
                         "positive Definitheit nicht garantiert")
    wx = [1.0] * m if gew_kart is None else [float(v) for v in gew_kart]
    wq = [1.0] * n if gew_gelenk is None else [float(v) for v in gew_gelenk]
    if len(wx) != m:
        raise ValueError("dls_schritt: gew_kart hat Laenge %d, erwartet %d" % (len(wx), m))
    if len(wq) != n:
        raise ValueError("dls_schritt: gew_gelenk hat Laenge %d, erwartet %d" % (len(wq), n))

    # A = W_x * J * W_q, ohne die Diagonalmatrizen wirklich aufzubauen
    A = [[wx[i] * J[i][j] * wq[j] for j in range(n)] for i in range(m)]

    # M = A*A^T + lambda^2 * I
    M = produkt_jjt(A)
    l2 = lam * lam
    for i in range(m):
        M[i][i] += l2

    b = [wx[i] * dx[i] for i in range(m)]
    L = cholesky(M)
    y = loese_spd(L, b)

    # dq = W_q * A^T * y
    dq = [0.0] * n
    for j in range(n):
        s = 0.0
        for i in range(m):
            s += A[i][j] * y[i]
        dq[j] = wq[j] * s
    return dq


def nullraum_projektor(J, lam):
    """N = I - J^+ * J mit gedaempftem J^+ = J^T (J J^T + lambda^2 I)^-1.

    n x n Matrix.  Eine Zusatzaufgabe dq_zusatz wirkt ueber N * dq_zusatz,
    ohne die kartesische Hauptaufgabe zu stoeren.  Erst fuer redundante
    Achszahlen sinnvoll; bei sechs Achsen ist N bis auf Rundung null.

    Vorgesehen fuer spaeter: Ausweichen vor Gelenkgrenzen oder Hindernissen
    beim siebenachsigen Arm.  MoveIt Pro nutzt an dieser Stelle eine
    Abstossung aus einem Signed Distance Field.
    """
    m = len(J)
    n = len(J[0]) if m else 0
    M = produkt_jjt(J)
    l2 = lam * lam
    for i in range(m):
        M[i][i] += l2
    L = cholesky(M)
    # Spaltenweise: J^+ = J^T * M^-1, also loese M * y_k = e_k
    Minv_spalten = []
    for k in range(m):
        e = [0.0] * m
        e[k] = 1.0
        Minv_spalten.append(loese_spd(L, e))
    # Jplus[j][k] = sum_i J[i][j] * Minv[i][k]
    Jplus = [[0.0] * m for _ in range(n)]
    for j in range(n):
        for k in range(m):
            s = 0.0
            for i in range(m):
                s += J[i][j] * Minv_spalten[k][i]
            Jplus[j][k] = s
    P = mat_mal_mat(Jplus, J)
    N = einheit(n)
    for i in range(n):
        for j in range(n):
            N[i][j] -= P[i][j]
    return N
