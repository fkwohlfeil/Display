#!/usr/bin/env python3
"""Kartesische Bahngeometrie mit Ueberschleifen. Reine Rechnung.

Erster Baustein des eigenen Bahnplaners. Dieses Modul kennt kein Tkinter,
kein ROS und kein NumPy -- dieselbe Trennung wie sequenz.py und
totg_folge.py, und aus demselben Grund: es soll ohne Bildschirm und ohne
gesourctes Environment pruefbar sein, und zweimal gerufen soll es bitgleich
dasselbe liefern. Summen stehen deshalb in fester Reihenfolge da.

Was hier entsteht, ist nur die Form der Bahn, keine Zeit. Die Bahn ist
ueber ihre Bogenlaenge s parametriert: zu jedem s gibt es den Ort p(s), die
Tangente p'(s) mit Betrag 1, den Kruemmungsvektor p''(s) und die
Orientierung q(s). Die Zeitplanung (bahn_zeit.py) sucht darauf spaeter nur
noch das Geschwindigkeitsprofil s'(s) -- ein eindimensionales Problem,
weil die Form hier festgelegt ist.

Was die Geometrie kann (Stufe 1)

    LIN     Strecke im Raum.
    CIRC    Kreisbogen durch Start, Hilfspunkt und Ziel, wie bei Pilz mit
            interim-Punkt.
    Radius  An jedem Zwischenpunkt mit Radius r > 0 wird ueberschliffen:
            die Bahn verlaesst die Strecke dort, wo sie in die Kugel mit
            Radius r um den Punkt eintritt, und kehrt dort zurueck, wo sie
            die Kugel verlaesst. Dazwischen liegt ein Kreisbogen, der beide
            Strecken tangential beruehrt. Das ist dieselbe Bedeutung wie
            blend_radius bei Pilz und zonedata bei ABB -- der Radius ist
            die Kugel, nicht der Bogen. Der Bogenradius folgt aus ihr:

                R = r / tan(theta / 2)

            theta ist der Richtungswechsel an der Ecke. Die Abweichung von
            der Ecke steht in jeder Ecke mit dabei:

                e = R / cos(theta / 2) - R

    r = 0   Genauhalt. Die Bahn hat dort einen Knick; die Zeitplanung muss
            anhalten. Die Stelle steht in Bahn.halte.

Was Stufe 1 bewusst nicht kann, und was sie dann tut

    Ueberschleifen an einem CIRC-Ende. Der Uebergang Strecke-Kreis
    braucht einen anderen Blender. Ein Radius ueber null dort ist ein
    Fehler, kein stilles r = 0.

    PTP. Eine PTP-Bewegung laeuft im Gelenkraum und hat keine kartesische
    Form. Sie gehoert nicht hierher; die Folge muss an ihr geteilt werden.

    Reine Umorientierung. Ohne Weg gibt es keine Bogenlaenge, an der die
    Drehung haengen koennte. Fehler.

    Kruemmungsstetigkeit. Der Kreisbogen ist tangentenstetig, aber an
    seinen Enden springt die Kruemmung von 0 auf 1/R. Das ist ein Sprung in
    der Normalbeschleunigung und damit im Ruck. Fuer eine ruckbegrenzte
    Zeitplanung kommt in Stufe 2 ein Bogen 5. Ordnung dazu; bis dahin steht
    der Sprung in jeder Ecke als kruemmung_sprung.

Umkehr

    Kehrt die Bahn an einem Punkt um (Richtungswechsel nahe 180 grad),
    wird R null und die Kruemmung unendlich. Ueberschleifen geht dort
    nicht. Mit Radius ueber null ist das eine Warnung, und die Ecke wird
    zum Genauhalt -- gemeldet, weil der eingetragene Radius dort nicht
    wirkt. Dieselbe Grenze wie in totg_folge (cos <= -1 + 1e-5).

Orientierung

    Jede Bewegung dreht von ihrer Startlage zu ihrer Ziellage, per Slerp
    ueber den Anteil der zurueckgelegten Strecke. Im Ueberschleifbereich
    laufen beide Bewegungen weiter, jede mit ihrer eigenen Rate, und
    werden mit einem Gewicht 5. Ordnung ineinander ueberblendet:

        f1 = min(1, (L1 - r + sigma) / L1)
        f2 = max(0, (r - La + sigma) / L2)
        q  = slerp(q1(f1), q2(f2), w(sigma / La))

    Am Eintritt ist das die Lage der ersten Bewegung, am Austritt die der
    zweiten, und weil w dort die Steigung null hat, springt die Drehrate
    nicht. Dasselbe Prinzip wie bei Pilz, nur ueber die Bogenlaenge statt
    ueber die Zeit.

Eingabe

    start = {"xyz": (x, y, z), "quat": (qx, qy, qz, qw)}
    ziele = [{"name", "fahrart", "xyz", "quat", "radius", "hilfspunkt"}, ...]

Quaternionen statt A, B, C: welche Achsen A, B und C meinen, entscheidet
die Anzeigekonvention der Oberflaeche, und die gehoert nicht hierher. Die
Umrechnung macht die Oberflaeche mit konvention.py.
"""

import bisect
import math

import sequenz

STAND = 1

# Unter dieser Laenge gilt eine Bewegung als ohne Weg.
LAENGE_MIN = 1e-9

# Unterhalb dieses Richtungswechsels ist eine Ecke keine: die Bahn laeuft
# geradeaus durch, ein Bogen waere unendlich gross.
KOLLINEAR_RAD = 1e-9

# Wie in totg_folge und Path::create: Umkehr ab cos(theta) <= -1 + 1e-5.
UMKEHR_TOLERANZ = 1e-05

# Kollinearitaet der drei CIRC-Punkte, bezogen auf die Seitenlaengen.
KREIS_KOLLINEAR = 1e-9

# Abtastung: Schrittweite auf Strecken und in Boegen, in Metern.
DS_GERADE = 0.005
DS_BOGEN = 0.0005

GERADE = "gerade"
KREIS = "kreis"
UEBERSCHLIFF = "ueberschliff"


# ----------------------------------------------------------------------------
# Vektoren und Quaternionen, ohne NumPy
# ----------------------------------------------------------------------------

def plus(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def minus(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def mal(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def punkt(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def kreuz(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def betrag(a):
    return math.sqrt(punkt(a, a))


def einheit(a):
    n = betrag(a)
    if n <= 0.0:
        raise ValueError("Nullvektor hat keine Richtung")
    return mal(a, 1.0 / n)


def quat_norm(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n <= 0.0:
        raise ValueError("Quaternion ist null")
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_punkt(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]


def slerp(a, b, t):
    """Kuerzester Weg von a nach b. a und b normiert, t in [0, 1]."""
    d = quat_punkt(a, b)
    if d < 0.0:
        b = (-b[0], -b[1], -b[2], -b[3])
        d = -d
    if d > 0.9995:
        # Fast gleich: linear und normiert. Vermeidet 0/0 in sin(omega).
        return quat_norm(tuple(x + t * (y - x) for x, y in zip(a, b)))
    omega = math.acos(min(1.0, d))
    s = math.sin(omega)
    ka = math.sin((1.0 - t) * omega) / s
    kb = math.sin(t * omega) / s
    return (ka * a[0] + kb * b[0], ka * a[1] + kb * b[1],
            ka * a[2] + kb * b[2], ka * a[3] + kb * b[3])


def drehwinkel(a, b):
    """Winkel der Drehung von a nach b, 0 .. pi."""
    d = min(1.0, abs(quat_punkt(a, b)))
    return 2.0 * math.acos(d)


def glatt5(u):
    """Gewicht 5. Ordnung: 0 -> 1, erste und zweite Ableitung am Rand 0."""
    u = min(1.0, max(0.0, u))
    return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)


# ----------------------------------------------------------------------------
# Stuecke der Bahn
# ----------------------------------------------------------------------------

class Bewegung:
    """Eine programmierte Bewegung, unabhaengig davon, wie viel von ihr
    nach dem Ueberschleifen uebrig bleibt. Traegt die Orientierung."""

    def __init__(self, name, art, laenge, q_start, q_ziel):
        self.name = name
        self.art = art                                 # GERADE oder KREIS
        self.laenge = laenge
        self.q_start = q_start
        self.q_ziel = q_ziel

    def lage(self, anteil):
        return slerp(self.q_start, self.q_ziel, min(1.0, max(0.0, anteil)))


class Stueck:
    """Ein Teil der Bahn mit eigener Form. sigma laeuft von 0 bis laenge."""

    art = None

    def __init__(self, s0, laenge):
        self.s0 = s0
        self.laenge = laenge

    def ort(self, sigma):
        raise NotImplementedError

    def tangente(self, sigma):
        raise NotImplementedError

    def kruemmung(self, sigma):
        """p''(s): Richtung zum Kruemmungsmittelpunkt, Betrag 1/R."""
        raise NotImplementedError

    def lage(self, sigma):
        raise NotImplementedError


class Gerade(Stueck):
    art = GERADE

    def __init__(self, s0, anfang, richtung, laenge, bewegung, versatz):
        super().__init__(s0, laenge)
        self.anfang = anfang
        self.richtung = richtung
        self.bewegung = bewegung
        self.versatz = versatz                         # Weg der Bewegung vor dem Stueck

    def ort(self, sigma):
        return plus(self.anfang, mal(self.richtung, sigma))

    def tangente(self, sigma):
        return self.richtung

    def kruemmung(self, sigma):
        return (0.0, 0.0, 0.0)

    def lage(self, sigma):
        b = self.bewegung
        return b.lage((self.versatz + sigma) / b.laenge)


class Bogen(Stueck):
    """Kreisbogen mit Mittelpunkt, Radius und zwei Basisvektoren.

    ort(sigma) = m + R (cos(phi) e1 + sin(phi) e2),  phi = sigma / R
    """

    def __init__(self, s0, mitte, radius, e1, e2, winkel):
        super().__init__(s0, radius * winkel)
        self.mitte = mitte
        self.radius = radius
        self.e1 = e1
        self.e2 = e2
        self.winkel = winkel

    def _cs(self, sigma):
        phi = sigma / self.radius
        return math.cos(phi), math.sin(phi)

    def ort(self, sigma):
        c, s = self._cs(sigma)
        return plus(self.mitte, plus(mal(self.e1, self.radius * c),
                                     mal(self.e2, self.radius * s)))

    def tangente(self, sigma):
        c, s = self._cs(sigma)
        return plus(mal(self.e1, -s), mal(self.e2, c))

    def kruemmung(self, sigma):
        c, s = self._cs(sigma)
        return mal(plus(mal(self.e1, c), mal(self.e2, s)), -1.0 / self.radius)


class Kreis(Bogen):
    """Eine CIRC-Bewegung."""

    art = KREIS

    def __init__(self, s0, mitte, radius, e1, e2, winkel, bewegung):
        super().__init__(s0, mitte, radius, e1, e2, winkel)
        self.bewegung = bewegung

    def lage(self, sigma):
        return self.bewegung.lage(sigma / self.bewegung.laenge)


class Ueberschliff(Bogen):
    """Kreisbogen an einer Ecke zwischen zwei Strecken."""

    art = UEBERSCHLIFF

    def __init__(self, s0, mitte, radius, e1, e2, winkel,
                 vorher, nachher, kugel):
        super().__init__(s0, mitte, radius, e1, e2, winkel)
        self.vorher = vorher
        self.nachher = nachher
        self.kugel = kugel                             # eingetragener Radius r

    def lage(self, sigma):
        l1 = self.vorher.laenge
        l2 = self.nachher.laenge
        f1 = min(1.0, (l1 - self.kugel + sigma) / l1)
        f2 = max(0.0, (self.kugel - self.laenge + sigma) / l2)
        w = glatt5(sigma / self.laenge)
        return slerp(self.vorher.lage(f1), self.nachher.lage(f2), w)


def kreis_durch(a, b, c):
    """Mittelpunkt, Radius, e1, e2 und Winkel des Bogens a -> b -> c.

    e1 zeigt vom Mittelpunkt auf a, e2 in Laufrichtung. Der Winkel ist der
    von a bis c ueber b, 0 .. 2 pi. None, wenn die Punkte auf einer Geraden
    liegen.
    """
    ab = minus(b, a)
    ac = minus(c, a)
    n = kreuz(ab, ac)
    nn = punkt(n, n)
    skala = punkt(ab, ab) * punkt(ac, ac)
    if skala <= 0.0 or nn <= (KREIS_KOLLINEAR ** 2) * skala:
        return None
    teil1 = mal(kreuz(n, ab), punkt(ac, ac))
    teil2 = mal(kreuz(ac, n), punkt(ab, ab))
    mitte = plus(a, mal(plus(teil1, teil2), 1.0 / (2.0 * nn)))
    radius = betrag(minus(a, mitte))
    e1 = einheit(minus(a, mitte))
    e2 = kreuz(einheit(n), e1)

    def winkel_von(p):
        d = minus(p, mitte)
        w = math.atan2(punkt(d, e2), punkt(d, e1))
        return w if w >= 0.0 else w + 2.0 * math.pi

    return mitte, radius, e1, e2, winkel_von(c), winkel_von(b)


def eckbogen(ecke, rein, raus, kugel):
    """Tangentialer Kreisbogen an einer Ecke. rein und raus sind Einheiten.

    Liefert (mitte, R, e1, e2, theta, eintritt, austritt, abweichung) oder
    None, wenn die Ecke keine ist (kollinear).
    """
    kos = max(-1.0, min(1.0, punkt(rein, raus)))
    theta = math.acos(kos)
    if theta < KOLLINEAR_RAD:
        return None
    halb = 0.5 * theta
    radius = kugel / math.tan(halb)
    eintritt = minus(ecke, mal(rein, kugel))
    austritt = plus(ecke, mal(raus, kugel))
    innen = einheit(minus(raus, rein))
    mitte = plus(ecke, mal(innen, radius / math.cos(halb)))
    e1 = einheit(minus(eintritt, mitte))
    e2 = rein
    abweichung = radius / math.cos(halb) - radius
    return mitte, radius, e1, e2, theta, eintritt, austritt, abweichung


# ----------------------------------------------------------------------------
# Die Bahn
# ----------------------------------------------------------------------------

class Auswertung:
    __slots__ = ("s", "ort", "tangente", "kruemmung", "lage", "art", "stueck")

    def __init__(self, s, ort, tangente, kruemmung, lage, art, stueck):
        self.s = s
        self.ort = ort
        self.tangente = tangente
        self.kruemmung = kruemmung
        self.lage = lage
        self.art = art
        self.stueck = stueck


class Bahn:
    """Die fertige Bahn: Stuecke hintereinander, Laenge, Halte, Ecken."""

    def __init__(self, stuecke, halte, ecken, bewegungen):
        self.stuecke = list(stuecke)
        self.halte = list(halte)                       # s-Werte mit Stillstand
        self.ecken = list(ecken)                       # je Zwischenpunkt ein Eintrag
        self.bewegungen = list(bewegungen)
        self._anfaenge = [st.s0 for st in self.stuecke]
        letztes = self.stuecke[-1]
        self.laenge = letztes.s0 + letztes.laenge

    def stueck_bei(self, s):
        """Index des Stuecks, in dem s liegt. Grenzen gehoeren nach vorn."""
        if s <= 0.0:
            return 0
        index = bisect.bisect_left(self._anfaenge, s) - 1
        # bisect_left liefert hoechstens len, index also hoechstens len - 1.
        return max(0, index)

    def auswerten(self, s):
        s = min(self.laenge, max(0.0, float(s)))
        index = self.stueck_bei(s)
        st = self.stuecke[index]
        sigma = min(st.laenge, max(0.0, s - st.s0))
        return Auswertung(s, st.ort(sigma), st.tangente(sigma),
                          st.kruemmung(sigma), st.lage(sigma), st.art, index)

    def drehrate(self, s, h=1e-5):
        """Drehwinkel je Meter Bahn an der Stelle s, numerisch."""
        a = max(0.0, s - h)
        b = min(self.laenge, s + h)
        if b <= a:
            return 0.0
        return drehwinkel(self.auswerten(a).lage, self.auswerten(b).lage) / (b - a)

    def abtasten(self, ds_gerade=DS_GERADE, ds_bogen=DS_BOGEN):
        """s-Werte fuer die Abtastung. Jede Stueckgrenze ist dabei.

        Auf Strecken grob, in Boegen fein: dort aendern sich die
        Achswerte schneller, und dort entscheidet sich die Eckgeschwindigkeit.
        Die Schrittweite wird je Stueck so gewaehlt, dass die Teilung
        aufgeht -- kein kurzer Rest am Ende.
        """
        werte = [0.0]
        for st in self.stuecke:
            schritt = ds_gerade if st.art == GERADE else ds_bogen
            if st.laenge <= 0.0:
                continue
            anzahl = max(1, int(math.ceil(st.laenge / schritt - 1e-9)))
            for k in range(1, anzahl + 1):
                werte.append(st.s0 + st.laenge * k / anzahl)
        werte[-1] = self.laenge
        return werte


def _befund(stufe, text, stelle=None):
    return sequenz.Befund(stufe, text, stelle=stelle)


def baue_bahn(start, ziele):
    """Bahn aus Startlage und Zielen. Liefert (bahn oder None, befunde).

    Fehler werden gesammelt, nicht geworfen. Traegt die Liste einen
    Fehler, gibt es keine Bahn.
    """
    befunde = []
    punkte = [tuple(float(v) for v in start["xyz"])]
    lagen = [quat_norm(tuple(float(v) for v in start["quat"]))]
    namen = ["Start"]
    arten = [None]
    radien = [0.0]
    hilfen = [None]

    if not ziele:
        return None, (_befund(sequenz.FEHLER, "Keine Ziele"),)

    for index, ziel in enumerate(ziele):
        name = str(ziel.get("name") or f"P{index + 1}")
        art = str(ziel.get("fahrart", "LIN")).upper()
        if art not in ("LIN", "CIRC"):
            befunde.append(_befund(
                sequenz.FEHLER,
                f"{name}: {art} hat keine kartesische Form. Die Bahngeometrie "
                f"nimmt nur LIN und CIRC; an PTP muss die Folge geteilt werden.",
                index))
        q = quat_norm(tuple(float(v) for v in ziel["quat"]))
        if quat_punkt(q, lagen[-1]) < 0.0:            # gleiche Hemisphaere
            q = (-q[0], -q[1], -q[2], -q[3])
        radius = float(ziel.get("radius", 0.0))
        if radius < 0.0:
            befunde.append(_befund(sequenz.FEHLER,
                                   f"{name}: Radius {radius:g} ist negativ", index))
        hilfe = ziel.get("hilfspunkt")
        if art == "CIRC" and hilfe is None:
            befunde.append(_befund(sequenz.FEHLER,
                                   f"{name}: CIRC braucht einen Hilfspunkt", index))
        punkte.append(tuple(float(v) for v in ziel["xyz"]))
        lagen.append(q)
        namen.append(name)
        arten.append(art)
        radien.append(radius)
        hilfen.append(None if hilfe is None else tuple(float(v) for v in hilfe))

    if radien[-1] > 0.0:
        befunde.append(_befund(
            sequenz.FEHLER,
            f"{namen[-1]}: der letzte Punkt wird angefahren und braucht "
            f"Radius 0", len(ziele) - 1))

    # -- Bewegungen: Laenge, Richtung, Kreisdaten ------------------------
    bewegungen = []
    richtungen = []                                    # Ein- und Austrittsrichtung
    kreise = []
    for k in range(1, len(punkte)):
        a, b = punkte[k - 1], punkte[k]
        if arten[k] == "CIRC" and hilfen[k] is not None:
            daten = kreis_durch(a, hilfen[k], b)
            if daten is None:
                befunde.append(_befund(
                    sequenz.FEHLER,
                    f"{namen[k]}: Start, Hilfspunkt und Ziel liegen auf einer "
                    f"Geraden -- kein Kreis", k - 1))
                bewegungen.append(None)
                richtungen.append(None)
                kreise.append(None)
                continue
            mitte, rho, e1, e2, winkel, winkel_hilfe = daten
            laenge = rho * winkel
            rein = e2
            raus = plus(mal(e1, -math.sin(winkel)), mal(e2, math.cos(winkel)))
            kreise.append((mitte, rho, e1, e2, winkel))
        else:
            laenge = betrag(minus(b, a))
            if laenge <= LAENGE_MIN:
                rein = raus = None
            else:
                rein = raus = einheit(minus(b, a))
            kreise.append(None)
        if laenge <= LAENGE_MIN:
            text = (f"{namen[k]}: liegt auf {namen[k - 1]}. "
                    + ("Reine Umorientierung wird in Stufe 1 nicht unterstuetzt."
                       if drehwinkel(lagen[k - 1], lagen[k]) > 1e-9
                       else "Die Bewegung hat keinen Weg."))
            befunde.append(_befund(sequenz.FEHLER, text, k - 1))
            bewegungen.append(None)
            richtungen.append(None)
            continue
        art = KREIS if arten[k] == "CIRC" else GERADE
        bewegungen.append(Bewegung(namen[k], art, laenge, lagen[k - 1], lagen[k]))
        richtungen.append((rein, raus))

    if any(b.stufe == sequenz.FEHLER for b in befunde):
        return None, tuple(befunde)

    # -- Ecken: Radius pruefen, Bogen bestimmen ---------------------------
    # kugel[k] ist der wirksame Radius an Punkt k (0 = Genauhalt).
    kugel = [0.0] * len(punkte)
    ecken = []
    boegen = {}
    for k in range(1, len(punkte) - 1):
        r = radien[k]
        rein = richtungen[k - 1][1]
        raus = richtungen[k][0]
        kos = max(-1.0, min(1.0, punkt(rein, raus)))
        theta = math.acos(kos)
        eintrag = {"name": namen[k], "index": k, "radius": r, "winkel": theta,
                   "bogenradius": None, "abweichung": 0.0,
                   "kruemmung_sprung": 0.0, "halt": False}
        umkehr = kos <= -1.0 + UMKEHR_TOLERANZ
        if r > 0.0 and (arten[k] == "CIRC" or arten[k + 1] == "CIRC"):
            befunde.append(_befund(
                sequenz.FEHLER,
                f"{namen[k]}: Ueberschleifen an einem CIRC-Ende kommt in "
                f"Stufe 2. Radius hier 0 setzen.", k - 1))
        elif r > 0.0 and umkehr:
            befunde.append(_befund(
                sequenz.WARNUNG,
                f"{namen[k]}: die Bahn kehrt dort um. Ueberschleifen geht "
                f"nicht; angehalten wird trotz Radius {r:g} m.", k - 1))
            eintrag["halt"] = True
        elif r > 0.0:
            if r >= bewegungen[k - 1].laenge or r >= bewegungen[k].laenge:
                befunde.append(_befund(
                    sequenz.FEHLER,
                    f"{namen[k]}: Radius {r:g} m reicht ueber eine der "
                    f"angrenzenden Bewegungen hinaus "
                    f"({bewegungen[k - 1].laenge:.4f} / "
                    f"{bewegungen[k].laenge:.4f} m).", k - 1))
            else:
                bogen = eckbogen(punkte[k], rein, raus, r)
                kugel[k] = r
                if bogen is not None:
                    boegen[k] = bogen
                    eintrag["bogenradius"] = bogen[1]
                    eintrag["abweichung"] = bogen[7]
                    eintrag["kruemmung_sprung"] = 1.0 / bogen[1]
        else:
            eintrag["halt"] = True
        ecken.append(eintrag)

    for k in range(1, len(punkte) - 1):
        if kugel[k - 1] + kugel[k] >= bewegungen[k - 1].laenge and kugel[k] > 0.0 \
                and kugel[k - 1] > 0.0:
            befunde.append(_befund(
                sequenz.FEHLER,
                f"{namen[k - 1]} -> {namen[k]}: die Ueberschleifkugeln "
                f"ueberlappen ({kugel[k - 1]:g} + {kugel[k]:g} >= "
                f"{bewegungen[k - 1].laenge:.4f} m).", k - 1))

    if any(b.stufe == sequenz.FEHLER for b in befunde):
        return None, tuple(befunde)

    # -- Stuecke zusammensetzen -------------------------------------------
    stuecke = []
    halte = [0.0]
    s = 0.0
    for k in range(1, len(punkte)):
        bewegung = bewegungen[k - 1]
        vorne = kugel[k - 1]
        hinten = kugel[k]
        if bewegung.art == GERADE:
            richtung = richtungen[k - 1][0]
            rest = bewegung.laenge - vorne - hinten
            if rest > 0.0:
                stuecke.append(Gerade(s, plus(punkte[k - 1], mal(richtung, vorne)),
                                      richtung, rest, bewegung, vorne))
                s += rest
        else:
            mitte, rho, e1, e2, winkel = kreise[k - 1]
            stuecke.append(Kreis(s, mitte, rho, e1, e2, winkel, bewegung))
            s += bewegung.laenge

        if k == len(punkte) - 1:
            break
        if hinten > 0.0 and k in boegen:
            mitte, radius, e1, e2, theta, _ein, _aus, _abw = boegen[k]
            stueck = Ueberschliff(s, mitte, radius, e1, e2, theta,
                                  bewegung, bewegungen[k], hinten)
            stuecke.append(stueck)
            ecken_eintrag = next(e for e in ecken if e["index"] == k)
            ecken_eintrag["s_mitte"] = s + 0.5 * stueck.laenge
            s += stueck.laenge
        elif hinten > 0.0:
            # Kollinear durchfahren: die Geraden stossen direkt aneinander.
            # Der Weg durch die Kugel gehoert zu beiden Bewegungen; er wird
            # als zwei kurze Geraden eingesetzt, damit jede ihre Drehung
            # behaelt.
            richtung = richtungen[k - 1][1]
            stuecke.append(Gerade(s, minus(punkte[k], mal(richtung, hinten)),
                                  richtung, hinten, bewegung,
                                  bewegung.laenge - hinten))
            s += hinten
            ecken_eintrag = next(e for e in ecken if e["index"] == k)
            ecken_eintrag["s_mitte"] = s
            richtung2 = richtungen[k][0]
            stuecke.append(Gerade(s, punkte[k], richtung2, hinten,
                                  bewegungen[k], 0.0))
            s += hinten
        else:
            halte.append(s)
            ecken_eintrag = next(e for e in ecken if e["index"] == k)
            ecken_eintrag["s_mitte"] = s
    halte.append(s)

    return Bahn(stuecke, halte, ecken, bewegungen), tuple(befunde)


def ecken_bericht(bahn):
    """Die Ecken als Textzeilen, fuer den Bericht der Oberflaeche."""
    zeilen = []
    for e in bahn.ecken:
        if e["halt"]:
            art = "Genauhalt"
        elif e["bogenradius"] is None:
            art = "geradeaus"
        else:
            art = (f"R {e['bogenradius'] * 1000.0:8.2f} mm  "
                   f"Abweichung {e['abweichung'] * 1000.0:7.3f} mm")
        zeilen.append(f"{e['name']:<10} Radius {e['radius'] * 1000.0:7.2f} mm  "
                      f"Wechsel {math.degrees(e['winkel']):6.1f} deg  {art}")
    return zeilen
