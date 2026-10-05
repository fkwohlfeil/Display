#!/usr/bin/env python3
"""Punktfolgen mit Ueberschleifen. Reine Rechnung, keine Oberflaeche.

Dieses Modul kennt kein Tkinter, kein ROS und kein NumPy. Es nimmt eine
Liste von Punkten entgegen, loest den Modalzustand auf, prueft die Folge und
baut daraus die Zahlen, die die Bruecke spaeter in einen
moveit_msgs/MotionSequenceRequest umsetzt. Es ruft nichts und wartet auf
nichts; damit laeuft es ohne Bildschirm und ohne laufenden move_group.

Warum ueberhaupt eine Folge und nicht dreimal planen und fahren

    Ein einzelner Auftrag endet mit Geschwindigkeit null. Drei Punkte
    nacheinander sind deshalb drei Beschleunigungs- und drei Bremsrampen,
    auch wenn der mittlere Punkt nur ein Durchfahrpunkt ist. Der Pilz-Planer
    kann die Teilbahnen stattdessen verschmelzen: liegt der Ueberschleif-
    radius eines Punktes ueber null, haelt der Roboter dort nicht an,
    sondern verlaesst die Bahn innerhalb einer Kugel um den Punkt und kehrt
    danach auf die Bahn zurueck, die er ohne Ueberschleifen genommen haette.

    Das ist dasselbe, was ABB als zonedata, KUKA als C_DIS und Fanuc als
    CNT anbietet. Der Radius ist damit die eine Zahl, an der sich Zykluszeit
    gegen Bahntreue tauschen laesst, und der Vergleichsmassstab gegenueber
    einer industriellen Steuerung.

Modalzustand

    Eine Punktzeile darf Fahrart, Tempo, Beschleunigung und Radius
    mitbringen. Laesst sie ein Feld weg, gilt der zuletzt gesetzte Wert.
    Steht ein Wert in der Zeile, gilt er ab dort auch fuer alle folgenden
    Punkte, die nichts sagen -- wie F1000 im G-Code weiterwirkt, bis es
    jemand aendert.

    Genau das ist die uebliche Fehlerquelle: ein Tempo, das drei Punkte
    weiter noch gilt, obwohl niemand mehr daran denkt. Das Modul versteckt
    das nicht, sondern schreibt zu jedem aufgeloesten Feld dazu, woher der
    Wert stammt. aufloesen() liefert die Herkunft mit, bericht() macht sie
    lesbar. Wer die Zeile sieht, sieht auch, dass sie geerbt ist.

Achsziele

    Eine Zeile, die mit J beginnt, traegt keine Lage, sondern je bewegliche
    Achse einen Wert, in Grad (Schubachsen in Metern), in der Reihenfolge
    der beweglichen Gelenke des Modells:

        J 0 0 0 0 0 0 0   tempo=50 name=Null

    Sie wird immer PTP und ohne Ueberschleifen angefahren. fahrart= und
    radius= stehen auf ihr nicht, auch nicht passend als PTP und 0: dort
    wirkten sie nicht weiter, sahen aber so aus. Der Modalzustand von
    Fahrart und Radius geht deshalb unberuehrt ueber die Zeile hinweg --
    die naechste kartesische Zeile erbt, was vor der J-Zeile galt. tempo
    und beschleunigung wirken wie auf jeder anderen Zeile.

    Warum es das braucht: bei sieben Achsen ist eine Lage nicht eindeutig,
    und an einer Stellung, in der zwei Achsen fluchten -- die
    Nullstellung ist eine --, hat die Rueckwaertskinematik unendlich viele
    Loesungen. Wer eine Messung wiederholbar beginnen will, braucht dort
    Achswerte und keine Lage.

    Das Modul ordnet die Werte den Gelenken zu und prueft Anzahl und
    Grenzen (loese_achsziel). Begrenzt wird nichts. Die Lage des Punktes
    rechnet es nicht; sie kommt aus der Vorwaertskinematik der Oberflaeche
    und wird nur gebraucht, um die Ueberschleifkugel des Nachbarn zu
    pruefen.

Was hier geprueft wird und warum es hier steht

    Zwei aufeinander folgende Ueberschleifkugeln duerfen sich nicht
    ueberlappen: radius(i) + radius(i+1) muss kleiner sein als der Abstand
    zwischen Ziel i und Ziel i+1. MoveIt lehnt das ab, sagt aber nicht,
    welches Paar es reisst. Bei acht Punkten ist das der Unterschied
    zwischen einer Meldung und einer Suche. Fuer Intrinsic ist das kein
    Fehler (pruefe(kugeln_pruefen=False)): Intrinsic kappt den Radius auf
    einen Teil der Strecke, statt abzulehnen. Der letzte Punkt muss Radius
    null haben -- ueberschliffen wird zum naechsten Punkt hin, und nach dem
    letzten kommt keiner.

Was hier ausdruecklich nicht steht

    Rueckwaertskinematik. Achswerte kommen aus MoveIt, nie aus einer
    zweiten Rechnung. Ein Punkt traegt seine Achswerte, wenn sie schon
    aufgeloest sind, und sonst nicht.
"""

import math

# Fahrarten des Pilz-Planers. Dieselben Zeichenketten wie PLANNER_IDS in der
# Oberflaeche; sie wandern unveraendert in planner_id.
FAHRARTEN = ("PTP", "LIN", "CIRC")

# Die vier Felder, die modal wirken. Reihenfolge ist die Anzeigereihenfolge.
MODALFELDER = ("fahrart", "tempo", "beschleunigung", "radius")

# Was ein Achsziel festlegt, statt es aus dem Modalzustand zu nehmen. Die
# Felder bleiben in MODALFELDER; nur ihre Herkunft heisst "achsziel", und der
# Modalzustand laeuft an ihnen vorbei.
ACHSZIEL_FEST = {"fahrart": "PTP", "radius": 0.0}

# Kennung am Zeilenanfang. Gross oder klein.
ACHSZIEL_KENNUNG = "J"

# Voreinstellung, solange niemand etwas anderes gesetzt hat. Langsam und
# ohne Ueberschleifen: die vorsichtige Wahl ist die richtige Voreinstellung.
MODAL_START = {
    "fahrart": "PTP",
    "tempo": 10.0,                                     # Prozent
    "beschleunigung": 10.0,                            # Prozent
    "radius": 0.0,                                     # Meter
}

# Zuschlag auf die Summe der Planungszeiten, deckt Hin- und Rueckweg und
# das, was move_group vor dem eigentlichen Planen tut. Dieselbe Ueberlegung
# wie PLAN_TIMEOUT_MARGIN_S in der Oberflaeche, hier eigenstaendig, damit
# das Modul nichts importieren muss.
FRIST_ZUSCHLAG_S = 5.0

# Ab hier gelten zwei Punkte als derselbe Ort. Ein Ueberschleifradius um
# einen Punkt, der auf seinem Vorgaenger liegt, hat keinen Sinn.
ORT_GLEICH_M = 1e-9

# Stufen eines Befundes. fehler haelt die Folge an, warnung nicht.
FEHLER = "fehler"
WARNUNG = "warnung"
HINWEIS = "hinweis"


class Befund:
    """Ein einzelner Prueferfolg. Traegt die Stelle mit, nicht nur den Text."""

    def __init__(self, stufe, text, stelle=None, paar=None):
        self.stufe = stufe
        self.text = text
        self.stelle = stelle                           # Index des Punktes
        self.paar = paar                               # (i, i+1) bei Paaren

    def __repr__(self):
        ort = ""
        if self.paar is not None:
            ort = f" [{self.paar[0]}->{self.paar[1]}]"
        elif self.stelle is not None:
            ort = f" [{self.stelle}]"
        return f"<{self.stufe}{ort} {self.text}>"

    def __eq__(self, other):
        if not isinstance(other, Befund):
            return NotImplemented
        return (self.stufe, self.text, self.stelle, self.paar) == (
            other.stufe, other.text, other.stelle, other.paar)


class Punkt:
    """Eine Zeile der Punktliste.

    xyz ist Pflicht und steht in Metern. Ohne Ort laesst sich der
    Ueberschleifradius nicht pruefen, und ohne diese Pruefung ist die Folge
    nur eine Liste, die MoveIt spaeter ablehnt.

    abc steht in Grad und in der gerade gewaehlten Anzeigekonvention. Das
    Modul rechnet damit nicht, es reicht die Zahlen durch; die Bedeutung
    haengt an der Konvention, und die gehoert nicht hierher.

    achsen ist ein Wortbuch Gelenkname -> Bogenmass oder None. Es steht
    hier, wenn die Rueckwaertskinematik den Punkt schon aufgeloest hat.
    Berechnet wird es hier nie.
    """

    def __init__(self, xyz, abc=(0.0, 0.0, 0.0), achsen=None, name="",
                 fahrart=None, tempo=None, beschleunigung=None, radius=None,
                 hilfspunkt=None, achsziel=None):
        # Seit 3.5: achsziel sind die Zahlen einer J-Zeile, so wie sie dort
        # standen (Grad bzw. Meter). Solange die Oberflaeche die Lage nicht
        # nachgetragen hat, ist xyz None -- nur hier darf es das sein.
        self.achsziel = (None if achsziel is None
                         else tuple(float(wert) for wert in achsziel))
        if xyz is None and self.achsziel is None:
            raise ValueError("xyz ist Pflicht, ausser bei einem Achsziel")
        self.xyz = None if xyz is None else tuple(float(wert) for wert in xyz)
        self.abc = None if abc is None else tuple(float(wert) for wert in abc)
        self.achsen = dict(achsen) if achsen else None
        self.name = str(name)
        self.fahrart = fahrart if fahrart is None else str(fahrart).upper()
        self.tempo = None if tempo is None else float(tempo)
        self.beschleunigung = (None if beschleunigung is None
                               else float(beschleunigung))
        self.radius = None if radius is None else float(radius)
        # CIRC braucht einen dritten Ort auf dem Bogen. PTP und LIN nicht.
        self.hilfspunkt = (None if hilfspunkt is None
                           else tuple(float(wert) for wert in hilfspunkt))

    def gesetzt(self):
        """Die Felder, die diese Zeile selbst mitbringt."""
        return tuple(feld for feld in MODALFELDER
                     if getattr(self, feld) is not None)

    def ergaenze_lage(self, xyz, abc):
        """Die Lage eines Achsziels nachtragen. Nur bei einem Achsziel.

        Die Lage kommt aus der Vorwaertskinematik der Oberflaeche und dient
        der Pruefung der Ueberschleifkugeln und dem Bericht. Bei einem Punkt
        mit eigener Lage waere das Ueberschreiben ein stiller Tausch der
        Zielvorgabe -- dort ist es deshalb ein Fehler.
        """
        if self.achsziel is None:
            raise ValueError("Lage nachtragen geht nur bei einem Achsziel")
        self.xyz = tuple(float(wert) for wert in xyz)
        self.abc = tuple(float(wert) for wert in abc)

    def __repr__(self):
        if self.achsziel is not None:
            return f"<Punkt {self.name or '?'} J{self.achsziel}>"
        return f"<Punkt {self.name or '?'} {self.xyz}>"


class Schritt:
    """Ein Punkt mit aufgeloestem Modalzustand.

    herkunft sagt je Feld, ob der Wert in der Zeile stand oder geerbt ist.
    Das ist der Grund, aus dem aufloesen() nicht einfach ein Wortbuch
    zurueckgibt: der Wert allein beantwortet die Frage nicht, die im
    Zweifelsfall gestellt wird, naemlich warum dieser Punkt langsam faehrt.
    """

    def __init__(self, index, punkt, werte, herkunft):
        self.index = index
        self.punkt = punkt
        self.fahrart = werte["fahrart"]
        self.tempo = werte["tempo"]
        self.beschleunigung = werte["beschleunigung"]
        self.radius = werte["radius"]
        self.herkunft = dict(herkunft)                 # Feld -> "zeile"/"modal"

    @property
    def xyz(self):
        return self.punkt.xyz

    @property
    def name(self):
        return self.punkt.name or f"P{self.index + 1}"

    def geerbt(self):
        """Die Felder, die aus dem Modalzustand kommen."""
        return tuple(feld for feld in MODALFELDER
                     if self.herkunft[feld] == "modal")

    def __repr__(self):
        return (f"<Schritt {self.name} {self.fahrart} "
                f"v={self.tempo} r={self.radius}>")


def abstand(a, b):
    """Euklidischer Abstand zweier Orte. Feste Summenreihenfolge, kein NumPy."""
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    dz = float(a[2]) - float(b[2])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class Folge:
    """Eine Punktfolge samt Modalzustand.

    Der Modalzustand am Anfang gehoert zur Folge und nicht zum Programm, das
    sie faehrt: dieselbe Liste zweimal gefahren muss dieselbe Bahn ergeben.
    Ein Zustand, der aus der vorigen Fahrt uebrig ist, waere genau die
    Nichtwiederholbarkeit, die hier nicht vorkommen soll.
    """

    def __init__(self, punkte=(), modal=None):
        self.punkte = list(punkte)
        self.modal = dict(MODAL_START)
        if modal:
            for feld, wert in modal.items():
                if feld not in MODALFELDER:
                    raise KeyError(f"kein modales Feld: {feld}")
                self.modal[feld] = wert

    # -- Aufloesen ---------------------------------------------------------

    def aufloesen(self):
        """Die Folge einmal durchgehen und den Modalzustand mitfuehren.

        Liest nichts an self ausser den Punkten und dem Anfangszustand und
        aendert nichts daran. Zweimal gerufen liefert es zweimal dasselbe.
        """
        zustand = dict(self.modal)
        schritte = []
        for index, punkt in enumerate(self.punkte):
            werte = {}
            herkunft = {}
            fest = ACHSZIEL_FEST if punkt.achsziel is not None else {}
            for feld in MODALFELDER:
                if feld in fest:
                    # Das Achsziel legt das Feld fest und laesst den
                    # Modalzustand stehen, wo er war.
                    werte[feld] = fest[feld]
                    herkunft[feld] = "achsziel"
                    continue
                eigen = getattr(punkt, feld)
                if eigen is None:
                    werte[feld] = zustand[feld]
                    herkunft[feld] = "modal"
                else:
                    werte[feld] = eigen
                    herkunft[feld] = "zeile"
                    zustand[feld] = eigen              # wirkt ab hier weiter
            schritte.append(Schritt(index, punkt, werte, herkunft))
        return tuple(schritte)

    def endzustand(self):
        """Der Modalzustand nach dem letzten Punkt. Fuer die Anzeige."""
        zustand = dict(self.modal)
        for punkt in self.punkte:
            fest = ACHSZIEL_FEST if punkt.achsziel is not None else {}
            for feld in MODALFELDER:
                if feld in fest:
                    continue
                eigen = getattr(punkt, feld)
                if eigen is not None:
                    zustand[feld] = eigen
        return zustand

    # -- Pruefen -----------------------------------------------------------

    def pruefe(self, schritte=None, kugeln_pruefen=True):
        """Alle Befunde zur Folge. Fehler zuerst gesammelt, nicht geworfen.

        Eine Ausnahme beim ersten Fehler waere bequem und fuer den, der die
        Liste eingibt, laestig: er bekaeme sieben Meldungen nacheinander
        statt einmal sieben. Gesammelt wird deshalb alles, was sich ohne
        MoveIt sagen laesst.

        kugeln_pruefen=False: sich ueberlappende Ueberschleifkugeln sind kein
        Fehler, sondern ein Hinweis. Fuer Verfahren, die den Radius selbst
        auf einen Teil der Strecke kappen (Intrinsic), statt ihn wie Pilz
        abzulehnen.
        """
        if schritte is None:
            schritte = self.aufloesen()
        befunde = []

        if not schritte:
            befunde.append(Befund(FEHLER, "Die Punktfolge ist leer"))
            return tuple(befunde)
        if len(schritte) == 1:
            befunde.append(Befund(
                HINWEIS,
                "Nur ein Punkt. Eine Folge lohnt sich ab zwei; "
                "ueberschliffen wird erst ab drei."))

        befunde.extend(self._pruefe_felder(schritte))
        befunde.extend(self._pruefe_radien(schritte, kugeln_pruefen))
        return tuple(befunde)

    def _pruefe_felder(self, schritte):
        """Jeden Schritt fuer sich. Nichts hier schaut auf den Nachbarn."""
        befunde = []
        for schritt in schritte:
            name = schritt.name

            if schritt.fahrart not in FAHRARTEN:
                befunde.append(Befund(
                    FEHLER,
                    f"{name}: Fahrart {schritt.fahrart!r} gibt es nicht. "
                    f"Moeglich: {', '.join(FAHRARTEN)}",
                    stelle=schritt.index))

            for feld in ("tempo", "beschleunigung"):
                wert = getattr(schritt, feld)
                if not 0.0 < wert <= 100.0:
                    befunde.append(Befund(
                        FEHLER,
                        f"{name}: {feld} {wert} liegt nicht in (0, 100] Prozent",
                        stelle=schritt.index))

            if schritt.radius < 0.0:
                befunde.append(Befund(
                    FEHLER,
                    f"{name}: Ueberschleifradius {schritt.radius} ist negativ",
                    stelle=schritt.index))

            if schritt.fahrart == "CIRC" and schritt.punkt.hilfspunkt is None:
                befunde.append(Befund(
                    FEHLER,
                    f"{name}: CIRC braucht einen Hilfspunkt auf dem Bogen",
                    stelle=schritt.index))
            if schritt.fahrart != "CIRC" and schritt.punkt.hilfspunkt is not None:
                befunde.append(Befund(
                    WARNUNG,
                    f"{name}: Hilfspunkt steht da, wird bei "
                    f"{schritt.fahrart} aber nicht gebraucht",
                    stelle=schritt.index))

            if schritt.punkt.achsziel is not None:
                # lies_zeile laesst diese Felder auf einer J-Zeile gar nicht
                # erst zu. Ein Punkt kann aber auch anders entstehen, und
                # dann soll die Folge nicht mit einem Feld hinausgehen, das
                # stillschweigend nicht gilt.
                for feld in ACHSZIEL_FEST:
                    if getattr(schritt.punkt, feld) is not None:
                        befunde.append(Befund(
                            FEHLER,
                            f"{name}: {feld}= steht auf einem Achsziel. "
                            f"Achsziele fahren immer PTP ohne "
                            f"Ueberschleifen; das Feld wirkte weder dort "
                            f"noch weiter.",
                            stelle=schritt.index))
                if schritt.punkt.achsen is None or schritt.punkt.xyz is None:
                    befunde.append(Befund(
                        FEHLER,
                        f"{name}: Achsziel nicht aufgeloest. Es fehlen die "
                        f"Achswerte oder die Lage aus der "
                        f"Vorwaertskinematik.",
                        stelle=schritt.index))
        return befunde

    def _pruefe_radien(self, schritte, kugeln_pruefen=True):
        """Die Bedingungen, die zwei Punkte zusammen betreffen.

        Der letzte Radius wird eigens geprueft und nicht stillschweigend auf
        null gesetzt. Wer ihn setzt, meint etwas damit, und was er meint,
        gibt es nicht: nach dem letzten Punkt ist keiner mehr, zu dem hin
        ueberschliffen werden koennte.
        """
        befunde = []
        letzter = schritte[-1]
        if letzter.radius > 0.0:
            befunde.append(Befund(
                FEHLER,
                f"{letzter.name}: letzter Punkt traegt Radius "
                f"{letzter.radius} m. Der letzte Punkt wird angefahren, "
                f"nicht ueberschliffen; er braucht Radius 0.",
                stelle=letzter.index))

        for eins, zwei in zip(schritte, schritte[1:]):
            if eins.xyz is None or zwei.xyz is None:
                # Ein Achsziel ohne Lage. Gemeldet ist es in _pruefe_felder;
                # hier ist ohne Ort nichts zu rechnen.
                continue
            weg = abstand(eins.xyz, zwei.xyz)
            paar = (eins.index, zwei.index)

            if weg <= ORT_GLEICH_M:
                stufe = FEHLER if (eins.radius > 0.0 or zwei.radius > 0.0) \
                    else WARNUNG
                befunde.append(Befund(
                    stufe,
                    f"{eins.name} und {zwei.name} liegen am selben Ort"
                    + (". Ein Ueberschleifradius hat dort keinen Sinn."
                       if stufe == FEHLER else "."),
                    paar=paar))
                continue

            summe = eins.radius + zwei.radius
            if summe >= weg and kugeln_pruefen:
                befunde.append(Befund(
                    FEHLER,
                    f"{eins.name} -> {zwei.name}: die Ueberschleifkugeln "
                    f"ueberlappen. {eins.radius:.4f} + {zwei.radius:.4f} = "
                    f"{summe:.4f} m, Abstand nur {weg:.4f} m. "
                    f"Zusammen unter {weg:.4f} m bleiben.",
                    paar=paar))
            elif summe >= weg:
                befunde.append(Befund(
                    HINWEIS,
                    f"{eins.name} -> {zwei.name}: Radien {eins.radius:.4f} + "
                    f"{zwei.radius:.4f} = {summe:.4f} m bei nur {weg:.4f} m "
                    f"Abstand. Fuer dieses Verfahren kein Fehler: es kappt "
                    f"den Radius selbst auf einen Teil der Strecke, der "
                    f"eingetragene Wert wirkt dann nicht voll.",
                    paar=paar))
        return befunde

    def fehler(self, befunde=None):
        """Nur die Befunde, die die Folge anhalten."""
        if befunde is None:
            befunde = self.pruefe()
        return tuple(b for b in befunde if b.stufe == FEHLER)

    def tragfaehig(self, befunde=None):
        """True, wenn die Folge an MoveIt gehen darf."""
        return not self.fehler(befunde)

    # -- Nutzlast ----------------------------------------------------------

    def nutzlast(self, gruppe, start, toleranz, planungszeit=1.0,
                 versuche=1, tag=None, tcp=None, kugeln_pruefen=True):
        """Reine Zahlen und Zeichenketten fuer /plan_sequence_path.

        Gebaut wird hier, gesendet in der Bruecke. Der GUI-Faden baut keine
        ROS-Nachrichten, und dieses Modul erst recht nicht.

        Zwei Regeln des Pilz-Planers stecken darin und sind der Grund, aus
        dem die Liste nicht einfach eine Aneinanderreihung der Einzelplaene
        ist:

            Nur der erste Auftrag einer Gruppe darf einen Startzustand
            tragen. Jede folgende Teilbahn beginnt am vorigen Ziel. Ein
            zweiter Startzustand ist kein zusaetzlicher Hinweis, sondern
            ein Widerspruch, und MoveIt lehnt die ganze Folge ab.

            Der letzte blend_radius ist null. Das prueft pruefe(); hier wird
            es nicht noch einmal stillschweigend erzwungen, sonst faende die
            Pruefung einen Fehler, den es gar nicht mehr gaebe.

        Schlaegt das Planen eines einzigen Auftrags fehl, wird keiner der
        Auftraege ausgefuehrt. Die Folge ist ganz oder gar nicht.

        kugeln_pruefen wie bei pruefe(). Muss zu dem passen, womit die Folge
        vorher geprueft wurde -- sonst geht sie durch die Pruefung und
        scheitert erst hier, nach der IK.
        """
        schritte = self.aufloesen()
        befunde = self.pruefe(schritte, kugeln_pruefen=kugeln_pruefen)
        harte = tuple(b for b in befunde if b.stufe == FEHLER)
        if harte:
            raise ValueError("Folge traegt " + str(len(harte))
                             + " Fehler: " + harte[0].text)

        start = {name: float(wert) for name, wert in start.items()}
        posten = []
        for schritt in schritte:
            eintrag = {
                "group": gruppe,
                "planner_id": schritt.fahrart,
                "tolerance": float(toleranz),
                "velocity_scaling": float(schritt.tempo) / 100.0,
                "acceleration_scaling": float(schritt.beschleunigung) / 100.0,
                "planning_time": float(planungszeit),
                "attempts": max(1, int(versuche)),
                "blend_radius": float(schritt.radius),
                "name": schritt.name,
            }
            if schritt.punkt.achsen:
                eintrag["goal"] = {name: float(wert) for name, wert
                                   in schritt.punkt.achsen.items()}
            elif schritt.punkt.achsziel is not None:
                # Unerreichbar, solange pruefe() gilt. Steht trotzdem hier:
                # ein Achsziel als Lage hinauszuschicken, waere genau der
                # stille Tausch, den die J-Zeile verhindern soll.
                raise ValueError(f"{schritt.name}: Achsziel ohne Achswerte")
            else:
                # Ohne aufgeloeste Achswerte geht das Ziel als Lage hinaus
                # und move_group rechnet die Rueckwaertskinematik. Bei einer
                # redundanten Kette heisst das, dass der Konfigurationszweig
                # nicht gesichert ist -- deshalb steht es hier ausdruecklich
                # als eigener Fall und nicht als stiller Zweig.
                eintrag["pose"] = {"xyz": schritt.punkt.xyz,
                                   "abc": schritt.punkt.abc,
                                   "tcp": tcp}
            if schritt.punkt.hilfspunkt is not None:
                eintrag["hilfspunkt"] = schritt.punkt.hilfspunkt
            posten.append(eintrag)

        posten[0]["start"] = start                     # nur der erste
        frist = (float(planungszeit) * max(1, int(versuche)) * len(posten)
                 + FRIST_ZUSCHLAG_S)
        return {
            "group": gruppe,
            "items": posten,
            "timeout": frist,
            "tag": tag,
        }

    # -- Anzeigen ----------------------------------------------------------

    def bericht(self, schritte=None):
        """Die aufgeloeste Folge als Text, mit Herkunft je Feld.

        Geerbte Werte stehen in Klammern. Das ist die ganze Absicht: wer
        sieht, dass drei Punkte hintereinander (10 %) tragen, sieht auch,
        dass er das Tempo einmal gesetzt und dreimal vergessen hat.
        """
        if schritte is None:
            schritte = self.aufloesen()
        zeilen = []
        for schritt in schritte:
            teile = [f"{schritt.index + 1:>3}  {schritt.name:<10}"]
            if schritt.xyz is None:
                teile.append(f"{'--':>8} {'--':>8} {'--':>8}")
            else:
                x, y, z = schritt.xyz
                teile.append(f"{x:8.3f} {y:8.3f} {z:8.3f}")
            for feld, form in (("fahrart", "{}"), ("tempo", "{:g}%"),
                               ("beschleunigung", "{:g}%"),
                               ("radius", "{:g} m")):
                text = form.format(getattr(schritt, feld))
                if schritt.herkunft[feld] == "modal":
                    text = "(" + text + ")"
                elif schritt.herkunft[feld] == "achsziel":
                    text = "[" + text + "]"
                teile.append(f"{text:>9}")
            zeilen.append("  ".join(teile))
        zeilen.append("")
        zeilen.append("Werte in Klammern stammen aus dem Modalzustand, "
                      "nicht aus der Zeile.")
        if any(schritt.punkt.achsziel is not None for schritt in schritte):
            zeilen.append("Werte in eckigen Klammern legt das Achsziel fest; "
                          "der Modalzustand geht darueber hinweg.")
        return "\n".join(zeilen)

    def spielraum(self, schritte=None):
        """Je Punktpaar der Abstand und der noch freie Radius.

        Fuer die Anzeige beim Eintragen: sie sagt nicht nur, dass ein Radius
        zu gross ist, sondern wie gross er sein duerfte.
        """
        if schritte is None:
            schritte = self.aufloesen()
        zeilen = []
        for eins, zwei in zip(schritte, schritte[1:]):
            if eins.xyz is None or zwei.xyz is None:
                continue
            weg = abstand(eins.xyz, zwei.xyz)
            zeilen.append({
                "paar": (eins.index, zwei.index),
                "namen": (eins.name, zwei.name),
                "abstand": weg,
                "belegt": eins.radius + zwei.radius,
                "frei": weg - (eins.radius + zwei.radius),
            })
        return tuple(zeilen)


# ----------------------------------------------------------------------------
# Antwortseite
# ----------------------------------------------------------------------------

def bereinige_zeitreihe(punkte, mindestschritt=0.0):
    """Stuetzpunkte mit nicht wachsender Zeit entfernen.

    Der Abtaster interpoliert ueber die Zeit. Zwei Stuetzpunkte mit
    demselben time_from_start sind dort eine Division durch null, und drei
    davon sind ein Sprung. In MoveIt gibt es dazu einen offenen Fehler: beim
    Erzeugen einer Folge mit Ueberschleifradius entstehen mehrfach
    Stuetzpunkte mit gleicher Zeit; ohne Ueberschleifen tritt es nicht auf.

    Verworfen wird still, aber nicht heimlich: die Anzahl und die Stellen
    gehen zurueck, damit die Oberflaeche es protokollieren kann. Eine Bahn,
    an der etwas ausgebessert wurde, muss das sagen -- sonst waere der
    Fehler weg und niemand wuesste, dass er da war.

    punkte ist eine Folge von (sekunden, ...). Der Rest jedes Eintrags wird
    nicht angefasst.
    """
    behalten = []
    verworfen = []
    letzte = None
    for index, eintrag in enumerate(punkte):
        zeit = float(eintrag[0])
        if letzte is not None and zeit - letzte <= float(mindestschritt):
            verworfen.append(index)
            continue
        behalten.append(eintrag)
        letzte = zeit
    return behalten, tuple(verworfen)


def pruefe_bahn(punkte, erwartete_dauer=None, toleranz=1e-6):
    """Befunde zu einer zurueckgegebenen Bahn. Ohne sie zu aendern."""
    befunde = []
    if not punkte:
        befunde.append(Befund(FEHLER, "Die Antwort enthaelt keine Bahn"))
        return tuple(befunde)

    _, verworfen = bereinige_zeitreihe(punkte)
    if verworfen:
        befunde.append(Befund(
            WARNUNG,
            f"{len(verworfen)} Stuetzpunkte tragen keine wachsende Zeit "
            f"(Stellen {verworfen[0]} ff.). Bekanntes Verhalten des "
            f"Ueberschleifens; sie werden vor dem Abtasten verworfen."))

    if float(punkte[0][0]) > toleranz:
        befunde.append(Befund(
            WARNUNG,
            f"Die Bahn beginnt bei t = {float(punkte[0][0]):.6f} s "
            f"und nicht bei null"))

    if erwartete_dauer is not None:
        dauer = float(punkte[-1][0])
        if dauer > float(erwartete_dauer) + toleranz:
            befunde.append(Befund(
                HINWEIS,
                f"Die Bahn dauert {dauer:.3f} s, erwartet waren "
                f"{float(erwartete_dauer):.3f} s"))
    return tuple(befunde)


# ----------------------------------------------------------------------------
# Achsziele
# ----------------------------------------------------------------------------

def loese_achsziel(punkt, gelenke, index=0):
    """Die Werte eines Achsziels den Gelenken zuordnen und pruefen.

    gelenke ist eine Folge von Wortbuechern mit name, drehend (bool), unten
    und oben -- die Grenzen in Bogenmass bzw. Metern, None ohne Grenze. Der
    erste Wert der Zeile gehoert dem ersten Gelenk.

    Gibt (achsen, befunde) zurueck; achsen ist None, sobald ein Befund
    vorliegt. Begrenzt wird nichts. Ein Wert jenseits der Grenze ist ein
    Fehler und nicht ein Wert an der Grenze: eine still begrenzte Achse
    fuehre woanders hin, als die Liste sagt, und die Messung stuende auf
    einem Ziel, das niemand eingetragen hat. Verglichen wird exakt, ohne
    Toleranz; die Meldung nennt die Grenze deshalb mit sechs Stellen.
    """
    if punkt.achsziel is None:
        return None, ()
    name = punkt.name or f"P{int(index) + 1}"
    werte = punkt.achsziel
    gelenke = tuple(gelenke)
    befunde = []

    if len(werte) != len(gelenke):
        namen = " ".join(gelenk["name"] for gelenk in gelenke) or "keine"
        befunde.append(Befund(
            FEHLER,
            f"{name}: {len(werte)} Achswert(e), das Modell hat "
            f"{len(gelenke)} bewegliche Achse(n): {namen}",
            stelle=index))
        return None, tuple(befunde)

    achsen = {}
    for wert, gelenk in zip(werte, gelenke):
        drehend = bool(gelenk["drehend"])
        einheit = "deg" if drehend else "m"
        if not math.isfinite(wert):
            befunde.append(Befund(
                FEHLER,
                f"{name}: {gelenk['name']} = {wert!r} ist keine endliche Zahl",
                stelle=index))
            continue
        ziel = math.radians(wert) if drehend else wert
        unten = gelenk.get("unten")
        oben = gelenk.get("oben")
        if (unten is not None and ziel < unten) or \
                (oben is not None and ziel > oben):
            def zeige(grenze):
                if grenze is None:
                    return "offen"
                grenze = math.degrees(grenze) if drehend else grenze
                return f"{grenze:.6f}"
            befunde.append(Befund(
                FEHLER,
                f"{name}: {gelenk['name']} = {wert!r} {einheit} liegt "
                f"ausserhalb der Grenzen {zeige(unten)} .. {zeige(oben)} "
                f"{einheit}",
                stelle=index))
            continue
        achsen[gelenk["name"]] = ziel

    if befunde:
        return None, tuple(befunde)
    return achsen, ()


# ----------------------------------------------------------------------------
# Lesen und Schreiben
# ----------------------------------------------------------------------------

def lies_zeile(text):
    """Eine Punktzeile lesen. Gibt einen Punkt zurueck oder wirft ValueError.

    Format, Felder durch Leerraum getrennt:

        X Y Z A B C [schluessel=wert ...]

    Die sechs Zahlen sind Pflicht, alles weitere ist benannt und modal:

        1.0 1.0 1.0 45 0 10   fahrart=LIN tempo=80 radius=0.05 name=Anfahrt

    Benannt statt der Stelle nach, weil eine Zeile mit zehn Zahlen
    hintereinander niemand mehr liest und ein vergessenes Feld die
    folgenden verschiebt, ohne dass es auffaellt.
    """
    stuecke = text.split()
    if not stuecke:
        raise ValueError("leere Zeile")
    if stuecke[0].upper() == ACHSZIEL_KENNUNG:
        return _lies_achsziel(stuecke[1:])
    zahlen, rest = _trenne(stuecke)
    if len(zahlen) != 6:
        raise ValueError(f"sechs Zahlen erwartet (X Y Z A B C), "
                         f"{len(zahlen)} gefunden")
    werte = _zahlen(zahlen)
    felder = _lies_felder(rest)
    return Punkt(xyz=werte[0:3], abc=werte[3:6], **felder)


def _trenne(stuecke):
    """Zahlen vorn, benannte Felder ab dem ersten Stueck mit =."""
    zahlen = []
    rest = []
    for index, stueck in enumerate(stuecke):
        if "=" in stueck:
            rest = stuecke[index:]
            break
        zahlen.append(stueck)
    return zahlen, rest


def _zahlen(zahlen):
    try:
        return [float(zahl) for zahl in zahlen]
    except ValueError as fehler:
        raise ValueError(f"keine Zahl: {fehler}") from None


def _lies_felder(rest):
    """Benannte Felder lesen. Dieselben Regeln fuer beide Zeilenarten."""
    felder = {}
    for stueck in rest:
        if "=" not in stueck:
            raise ValueError(f"benanntes Feld erwartet, gefunden: {stueck!r}")
        schluessel, wert = stueck.split("=", 1)
        schluessel = schluessel.strip().lower()
        if schluessel not in MODALFELDER + ("name",):
            raise ValueError(f"kein Feld dieses Namens: {schluessel!r}")
        if schluessel in felder:
            raise ValueError(f"Feld doppelt: {schluessel!r}")
        if schluessel in ("name", "fahrart"):
            felder[schluessel] = wert
        else:
            try:
                felder[schluessel] = float(wert)
            except ValueError:
                raise ValueError(f"{schluessel}: keine Zahl: {wert!r}") from None
    return felder


def _lies_achsziel(stuecke):
    """Den Rest einer J-Zeile lesen: Achswerte, danach benannte Felder.

    Die Anzahl der Werte prueft loese_achsziel, denn nur dort ist das Modell
    bekannt. Hier steht nur, was ohne Modell zu sagen ist.
    """
    zahlen, rest = _trenne(stuecke)
    if not zahlen:
        raise ValueError("Achsziel ohne Werte: nach J steht je bewegliche "
                         "Achse ein Wert")
    werte = _zahlen(zahlen)
    felder = _lies_felder(rest)
    verboten = [feld for feld in ACHSZIEL_FEST if feld in felder]
    if verboten:
        raise ValueError(f"{verboten[0]}= steht nicht auf einer J-Zeile: "
                         f"Achsziele fahren immer PTP ohne Ueberschleifen")
    return Punkt(xyz=None, abc=None, achsziel=werte, **felder)


def lies_liste(text):
    """Mehrere Zeilen lesen. Leerzeilen und # werden uebergangen.

    Gibt (punkte, befunde) zurueck. Eine unlesbare Zeile haelt das Lesen
    nicht an, sondern wird gemeldet und uebersprungen: wer zwanzig Punkte
    eintippt, will alle Tippfehler auf einmal sehen.
    """
    punkte = []
    befunde = []
    for nummer, zeile in enumerate(text.splitlines(), start=1):
        blank = zeile.split("#", 1)[0].strip()
        if not blank:
            continue
        try:
            punkte.append(lies_zeile(blank))
        except ValueError as fehler:
            befunde.append(Befund(FEHLER, f"Zeile {nummer}: {fehler}",
                                  stelle=nummer))
    return punkte, tuple(befunde)
