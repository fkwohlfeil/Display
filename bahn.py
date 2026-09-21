"""bahn -- Bahn abfahren und aufzeichnen: Zustaende, Tippbetrieb-Konstanten,
MoveIt-Fehlertexte, hermite, TrajectoryPlayer, PathRecorder.

Seit urdf_gui2_1 ein eigenes Modul. Der Inhalt ist zeilengleich aus
urdf_gui3_7.py verschoben (dort Zeilen 2606-2614, 3418-3969); neu sind
nur dieser Kopf, die Importe und STAND.
"""

import csv
import collections
import math
import time
import konvention
from datetime import datetime

from einstellungen import (RECORD_MAX_POINTS, RECORD_MIN_ANGLE_DEG,
                           RECORD_MIN_DIST_M, RECORD_MIN_JOINT_DEG,
                           RECORD_RATE_HZ)

# Muss zu MODUL_STAENDE in urdf_gui2_1.py passen.
STAND = 1


# ==========================================================================
# Bahnrekorder
# ==========================================================================

def angle_delta(a, b):
    """Kleinster Winkelabstand in rad, ueber +-pi hinweg."""
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)



# ==========================================================================
# Bahn abfahren
# ==========================================================================

# So viele Ausnahmen vertraegt der ROS-Takt, bevor er aufgibt. Einer allein
# darf ihn nicht beenden, endloses Wiederholen aber auch nicht.
SPIN_ERROR_LIMIT = 20

# Ab diesem Alter der letzten TF-Ausgabe meldet der Waechter. Beim Takt von
# TF_RATE_HZ ist das etwa das Dreissigfache des Erwarteten.
TF_STALL_MS = 1000.0

# Takt, in dem der Executor-Thread nach neuen Auftraegen sieht.
JOB_RATE_HZ = 20.0

# Interpolationstakt der GUI. Reale Steuerungen liegen bei 4 bis 12 ms; hier
# waeren die schoen, aber Tk-after, der Python-GIL, DDS ueber Sockets und
# unter WSL der Hypervisor liegen dazwischen. 20 ms sind unter diesen
# Bedingungen stabil zu halten -- und der Abstand zwischen Soll- und Ist-Takt
# ist genau das, was in der Fusszeile steht und im Unterricht etwas wert ist.
IPO_TICK_MS = 20
IPO_DT = IPO_TICK_MS / 1000.0

# Zeit, in der die Bahn beim Anhalten von voller Rate auf null gefuehrt wird.
# Ein Sprung auf null waere die Verletzung genau der Grenzen, um die es hier
# geht. Die Rampe ist linear in der Zeitrate: einfach, erklaerbar, und sie
# begrenzt die Achsbeschleunigung auf einen bekannten Faktor. Sie ist keine
# ruckbegrenzte Bremsung -- das waere ein eigener Schritt.
BRAKE_TIME_S = 0.40

# Takt des Wachhunds am Halte-Taster. Er fragt nicht nach Ereignissen,
# sondern nach dem, was Tk beantworten kann: steht der Zeiger noch auf der
# Taste. 100 ms sind der Ausgleich zwischen zwei Fehlern -- kuerzer belastet
# den GUI-Thread ohne Gewinn, laenger laesst den Roboter nach einem
# verlorenen ButtonRelease sichtbar zu lange weiterfahren. Bei 100 ms liegt
# der schlimmste Fall unter der Bremszeit selbst.
HOLD_WATCH_MS = 100

# Zustaende der Ablaufsteuerung.
STATE_READY = "BEREIT"
# Die Rueckwaertskinematik ist ein eigener Zustand und nicht der Anfang von
# PLANEN. Sie beantwortet eine andere Frage -- "gibt es zu dieser Lage
# ueberhaupt eine Achsstellung" statt "gibt es einen Weg dorthin" --, und sie
# scheitert aus anderen Gruenden. Beides in einem Zustand zu fuehren hiesse,
# den Unterschied im Fehlercode zu verstecken.
STATE_IK = "IK"
STATE_PLANNING = "PLANEN"
# Eine Punktfolge ist ein eigener Zustand und nicht PLANEN. Sie geht an
# einen anderen Dienst, sie scheitert an anderen Dingen -- ueberlappenden
# Radien etwa, die es bei einem einzelnen Ziel gar nicht gibt --, und ihr
# Ergebnis ist eine Liste von Teilbahnen statt einer Bahn. In einem
# Zustand gefuehrt waere der Unterschied nur noch am Fehlertext zu sehen.
# Die Aufloesung der Punkte vor dem Auftrag. Ein eigener Zustand und nicht
# STATE_IK: hier laufen n Anfragen nacheinander, und die Antwort auf die
# dritte darf nicht in denselben Zweig fallen wie die eine Anfrage des
# Fahrfensters. Wer sie zusammenlegte, muesste an jeder Stelle in
# _on_ik_result zusaetzlich fragen, wofuer die Antwort gedacht war.
STATE_SEQ_IK = "FOLGE IK"
STATE_SEQUENCE = "FOLGE"
STATE_MOVING = "FAHREN"
STATE_BRAKING = "BREMSEN"
STATE_ERROR = "FEHLER"

# Zustaende, in denen nichts veraendert werden darf.
STATES_BUSY = (STATE_IK, STATE_PLANNING, STATE_SEQ_IK, STATE_SEQUENCE,
               STATE_MOVING, STATE_BRAKING)

# Zielraum im Fahrfenster. Zwei Zeichenketten statt eines Wahrheitswerts,
# damit in Protokoll und Auswahl dasselbe Wort steht.
SPACE_JOINT = "joint"
SPACE_CART = "cart"

# Zuschlag auf die Zeitgrenze des Solvers, wie PLAN_TIMEOUT_MARGIN_S beim
# Planen. Deutlich kleiner: ein IK-Aufruf ist eine Rechnung von Millisekunden,
# und wenn nach Sekunden nichts kommt, liegt es nicht am Rechnen.
IK_TIMEOUT_MARGIN_S = 5.0

# ---------- Kartesisches Joggen ----------
# Ein Klick geht um eine feste Stufe, Halten faehrt mit fester Geschwindigkeit.
# Zwei Groessen, nicht eine: die Stufe soll gross genug sein, dass ein Klick
# etwas bewirkt, und die Geschwindigkeit klein genug, dass man beim Halten
# noch mitliest.
JOG_STEP_M = 0.01                                      # ein Klick
JOG_STEP_DEG = 0.1
JOG_VEL_M_S = 0.02                                     # gehalten
JOG_VEL_DEG_S = 0.2

# Ab hier wird aus dem Klick ein Halten. Kuerzer, und ein normaler Klick
# faehrt ungewollt weiter; laenger, und das Halten fuehlt sich tot an.
JOG_HOLD_DELAY_MS = 300

# Obergrenze fuer einen einzelnen Zeitschritt. Bleibt die Antwort einmal
# laenger aus -- Solver am Anschlag, Rechner unter Last --, waere der
# nachgeholte Weg sonst ein Sprung. Lieber langsamer als gerissen.
JOG_MAX_DT_S = 0.25

# Warnschwellen fuer die Achsauslastung, als Anteil der Achsgeschwindigkeit,
# die bei der eingestellten Jog-Geschwindigkeit gefordert waere.
#
# Zwei Werte und nicht einer: mit einer einzigen Schwelle kippte die Warnung
# bei jedem Schritt hin und her, sobald die Auslastung um sie herum pendelt --
# und pendeln tut sie, weil jeder Schritt eine eigene IK-Loesung ist. Das ist
# dieselbe Hysterese wie bei einem Schmitt-Trigger: eingeschaltet wird oben,
# ausgeschaltet erst deutlich darunter. Der Abstand ist der ganze Zweck.
# Takt der Achs-Dauerfahrt. Sie braucht keine IK und damit keine Antwort, auf
# die sie warten muesste -- deshalb eine Uhr, waehrend das kartesische Joggen
# an der Antwort haengt. Zwei Schleifen, weil es zwei verschiedene Aufgaben
# sind: die eine fragt einen Dienst, die andere rechnet nichts.
JOG_TICK_MS = 50

JOG_WARN_SHARE = 0.75                                  # ab hier warnen
JOG_CLEAR_SHARE = 0.65                                 # erst hier wieder still

# moveit_msgs/MoveItErrorCodes. Die nackte Zahl hilft niemandem, und der
# Umgang mit diesen Faellen ist ein guter Teil des Versuchs -- deshalb steht
# neben der Uebersetzung auch, woran es meistens liegt.
MOVEIT_ERRORS = {
    1: ("Erfolg", ""),
    # 0 ist UNDEFINED, der Vorbelegungswert des Feldes: niemand hat
    # geantwortet, der Auftrag ist bei keinem Planer angekommen.
    0: ("Keine Antwort", "Der Auftrag ist bei keinem Planer angekommen. "
                         "Pipeline geladen? Pilz installiert?"),
    # 99999 ist FAILURE, der Sammelwert -- und etwas ganz anderes als 0.
    # Hier hat sehr wohl jemand geantwortet: ein Planer hat gearbeitet und
    # eine Ausnahme geworfen, fuer die es keinen eigenen Code gibt. Das
    # Ueberschleifen benutzt ihn zum Beispiel, wenn die Abtastraster zweier
    # Teilbahnen nicht zusammenpassen. Bis 3.1 stand hier derselbe Text wie
    # bei 0, und der schickte die Suche in die falsche Richtung: nicht die
    # Pipeline fehlt, sondern der Grund steht in der Ausgabe von
    # move_group -- deshalb wird sie in diesem Fall selbst geholt.
    99999: ("Fehlgeschlagen", "Sammelfehler ohne eigenen Code. Ein Planer hat "
                              "gearbeitet und aufgegeben; der Grund steht nur "
                              "in der Ausgabe von move_group."),
    999999: ("Fehlgeschlagen", "Sammelfehler ohne eigenen Code. Der Grund "
                               "steht nur in der Ausgabe von move_group."),
    -1: ("Fehlgeschlagen", "Sammelfehler ohne naehere Angabe."),
    -2: ("Planung fehlgeschlagen",
         "Pilz weicht nicht aus. Ziel erreichbar? Achsgrenzen zu eng?"),
    -3: ("Bahn ungueltig",
         "Die gerechnete Bahn hat die Pruefung nicht bestanden."),
    -4: ("Bahn durch Umgebungsaenderung ungueltig", ""),
    -5: ("Ausfuehrung fehlgeschlagen",
         "Betrifft nur die Ausfuehrung durch move_group, nicht diese GUI."),
    -6: ("Keine Sensordaten", ""),
    -7: ("Zeitgrenze ueberschritten",
         "Planungszeit im Parameterfenster erhoehen."),
    -8: ("Abgebrochen", ""),
    -10: ("Startzustand in Kollision",
          "Der Roboter steht schon in sich selbst. SRDF-Paare pruefen."),
    -11: ("Startzustand verletzt Bahnbedingung", ""),
    -12: ("Ziel in Kollision", ""),
    -13: ("Ziel verletzt Bahnbedingung", ""),
    -14: ("Zielbedingungen verletzt",
          "Erreicht, aber ausserhalb der Toleranz. Zieltoleranz pruefen."),
    -15: ("Gruppenname ungueltig",
          "Die Planungsgruppe steht so nicht in der SRDF."),
    -16: ("Zielbedingungen ungueltig",
          "Gelenknamen im Ziel passen nicht zur Gruppe."),
    -17: ("Roboterzustand ungueltig",
          "Meist fehlt /joint_states oder es ist zu alt."),
    -18: ("Linkname ungueltig", ""),
    -19: ("Objektname ungueltig", ""),
    -21: ("Frame-Transformation fehlgeschlagen", ""),
    -22: ("Kollisionspruefung nicht verfuegbar", ""),
    -23: ("Roboterzustand veraltet",
          "/joint_states kommt nicht oder zu langsam."),
    -24: ("Sensordaten veraltet", ""),
    -25: ("Verbindungsfehler", ""),
    -31: ("Keine IK-Loesung",
          "Nur bei kartesischen Zielen. Lage ausserhalb des Arbeitsraums?"),
}


def moveit_error_text(code):
    """Fehlercode als Satz. Unbekannte Codes bleiben als Zahl sichtbar."""
    label, hint = MOVEIT_ERRORS.get(int(code), (None, ""))
    if label is None:
        return f"Unbekannter Fehlercode {int(code)}"
    if hint:
        return f"{label} ({int(code)}). {hint}"
    return f"{label} ({int(code)})"


def hermite(p0, v0, p1, v1, span, s):
    """Kubische Hermite-Interpolation zwischen zwei Stuetzpunkten.

    Pilz gibt zu jedem Punkt auch die Geschwindigkeit heraus. Wer die
    wegwirft und geradlinig interpoliert, bekommt an jedem Stuetzpunkt einen
    Knick in der Geschwindigkeit -- bei dichter Abtastung kaum sichtbar, im
    Geschwindigkeitsdiagramm aber sofort. Die kubische Form nimmt beide
    Randgeschwindigkeiten mit und ist damit stetig differenzierbar.

    span ist der Zeitabstand der Stuetzpunkte, s der Anteil dazwischen.
    """
    s2 = s * s
    s3 = s2 * s
    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2
    return h00 * p0 + h10 * span * v0 + h01 * p1 + h11 * span * v1


# Ab dieser Abweichung zwischen Achsstand und erstem Stuetzpunkt wird eine
# geplante Folge nicht gefahren. 1e-4 rad sind 0.0057 grad -- weit unter
# allem, was eine Achse aufloest, und weit ueber dem Rundungsrest, den der
# Weg durch ROS-Nachrichten und zurueck hinterlaesst.
FOLGE_START_TOL = 1e-4


def bahn_startabweichung(punkte, stand):
    """Wie weit steht der Roboter vom ersten Stuetzpunkt einer Bahn weg?

    punkte: [(zeit, {name: pos}, {name: vel}), ...] wie beim
    TrajectoryPlayer. stand: {name: pos}, der Achsstand.

    Liefert (name, betrag, fehlend): die Achse mit der groessten
    Abweichung, deren Betrag, und die Namen, die die Bahn traegt und der
    Stand nicht kennt. Ohne Punkte (None, 0.0, ()).

    Bewusst ohne Tk und ohne ROS und bewusst ohne Entscheidung: die
    Funktion vergleicht und urteilt nicht. Ob 1e-5 noch geht und 1e-3
    nicht mehr, ist eine Festlegung der Oberflaeche und steht in
    FOLGE_START_TOL -- hier stuende sie an der falschen Stelle, und beim
    Pruefen muesste man sie mitpruefen.

    Fehlende Achsen werden nicht als Abweichung null gezaehlt. Eine Bahn
    mit einer Achse, die der Stand nicht kennt, gehoert zu einem anderen
    Modell; das ist kein kleiner Unterschied, sondern ein anderer Roboter.
    """
    if not punkte:
        return (None, 0.0, ())
    soll = punkte[0][1]
    fehlend = tuple(sorted(name for name in soll if name not in stand))
    schlimmste = None
    betrag = 0.0
    for name, wert in soll.items():
        if name not in stand:
            continue
        ab = abs(float(stand[name]) - float(wert))
        if schlimmste is None or ab > betrag:
            schlimmste = name
            betrag = ab
    return (schlimmste, betrag, fehlend)


class TrajectoryPlayer:
    """Tastet eine geplante Bahn im festen Takt ab.

    Bewusst ohne Tk und ohne ROS: die Klasse bekommt Stuetzpunkte als reine
    Zahlen und gibt Sollwerte als reine Zahlen zurueck. Damit laesst sie sich
    ohne Fenster und ohne Roboter pruefen, und im Unterricht kann man sie in
    zehn Zeilen selbst aufrufen.

    Die Bahnzeit entsteht aus der Summe der Takte, nicht aus der Wanduhr.
    Das ist der Grund, warum derselbe Auftrag zweimal denselben CSV-Auszug
    liefert: Tk-after ist nicht echtzeitfaehig, und wer die Wanduhr in die
    Rechnung nimmt, nimmt deren Zittern gleich mit. Der Abstand zwischen
    Bahnzeit und Wanduhrzeit ist dadurch messbar -- er wird angezeigt statt
    versteckt.

    points: [(zeit, {name: pos}, {name: vel}), ...], nach Zeit sortiert.
    """

    def __init__(self, points, dt=IPO_DT):
        if len(points) < 2:
            raise ValueError("Eine Bahn braucht mindestens zwei Stuetzpunkte")
        self.points = points
        self.dt = dt
        self.names = list(points[0][1])
        self.duration = points[-1][0]
        self.t = 0.0                                   # Bahnzeit
        self.ticks = 0                                 # gezaehlte Takte
        self.rate = 1.0                                # Zeitrate, 1.0 = voll
        self.braking = False
        self._index = 0                                # Suchzeiger, laeuft vor

    # -- Zustand -----------------------------------------------------------

    @property
    def done(self):
        return self.t >= self.duration and not self.braking

    @property
    def stopped(self):
        return self.braking and self.rate <= 0.0

    @property
    def progress(self):
        if self.duration <= 0.0:
            return 1.0
        return min(1.0, self.t / self.duration)

    def wall_drift(self, elapsed):
        """Abstand zwischen gezaehlter Bahnzeit und vergangener Wanduhrzeit."""
        return elapsed - self.ticks * self.dt

    # -- Abtasten ----------------------------------------------------------

    def sample(self, t=None):
        """Sollwerte zum Zeitpunkt t. Ohne Angabe zur aktuellen Bahnzeit."""
        if t is None:
            t = self.t
        if t <= self.points[0][0]:
            return dict(self.points[0][1])
        if t >= self.points[-1][0]:
            return dict(self.points[-1][1])

        # Der Zeiger laeuft nur vorwaerts, solange die Zeit vorwaerts laeuft.
        # Beim Bremsen steht sie hoechstens still, sie geht nie zurueck.
        while (self._index + 1 < len(self.points) - 1
               and self.points[self._index + 1][0] <= t):
            self._index += 1
        while self._index > 0 and self.points[self._index][0] > t:
            self._index -= 1

        t0, p0, v0 = self.points[self._index]
        t1, p1, v1 = self.points[self._index + 1]
        span = t1 - t0
        if span <= 0.0:
            return dict(p1)
        s = (t - t0) / span
        return {name: hermite(p0.get(name, 0.0), v0.get(name, 0.0),
                              p1.get(name, 0.0), v1.get(name, 0.0), span, s)
                for name in self.names}

    def step(self):
        """Einen Takt weiter. Liefert die Sollwerte danach."""
        self.ticks += 1
        if self.braking and self.rate > 0.0:
            self.rate = max(0.0, self.rate - self.dt / BRAKE_TIME_S)
        self.t = min(self.duration, self.t + self.dt * self.rate)
        return self.sample()

    def brake(self):
        """Anhalten einleiten. Die Bahn wird nicht verlassen, nur langsamer."""
        self.braking = True


class PathRecorder:
    """Ringpuffer fuer die TCP-Bahn.

    Kein ROS, kein Tk: die Klasse haelt nur Zahlen. Die Abtastung stoesst die
    GUI ueber `sample` an, gefiltert wird hier drin.
    """

    def __init__(self, max_points=RECORD_MAX_POINTS,
                 min_dist=RECORD_MIN_DIST_M,
                 min_angle=math.radians(RECORD_MIN_ANGLE_DEG),
                 min_joint=math.radians(RECORD_MIN_JOINT_DEG)):
        self.points = collections.deque(maxlen=max_points)
        self.min_dist = min_dist
        self.min_angle = min_angle
        self.min_joint = min_joint
        self.joint_names = []                          # Reihenfolge der Achsspalten
        # Ruheschwelle je Achsspalte, in der Einheit dieser Spalte. Eine
        # gemeinsame Schwelle gibt es nicht: die Werte kommen so herein, wie
        # sie in die CSV gehoeren, und das sind grad bei einem Drehgelenk und
        # Meter bei einem Schubgelenk. min_joint bleibt als Rueckfall fuer den
        # Aufrufer, der nichts angibt.
        self.joint_tolerances = []
        self.frame_names = []                          # Reihenfolge der Frameblocks
        self.chain = []                                # [(kind, elter, gelenk)]
        self.couplings = []                            # [(gelenk, fuehrung, m, b)]
        self.robot_name = ""
        self.active = False
        # Die Konvention, in der die abgelegten Winkel gemeint sind. Sie
        # gehoert zu den Punkten und nicht zur Anzeige: eine Datei muss
        # deutbar bleiben, wenn die Oberflaeche laengst anders eingestellt
        # ist. Gesetzt wird sie beim Start und danach nicht mehr -- das ist
        # der Grund, aus dem das Umschalten waehrend einer Aufzeichnung
        # abgelehnt wird.
        self.konvention = konvention.STANDARD
        self.start_time = None                         # time.monotonic()
        self.length = 0.0                              # gefahrener Weg des TCP
        self.overflow = False                          # aeltester Punkt verworfen

    # -- Steuerung ---------------------------------------------------------

    def start(self, joint_names, frame_names, chain,
              couplings=(), robot_name="", joint_tolerances=(),
              drehkonvention=None):
        """Aufzeichnung beginnen oder fortsetzen.

        Wechselt der Achssatz, der Framesatz oder die Drehkonvention, sind
        die alten Punkte nicht mehr spaltenrichtig zu schreiben. Dann wird
        verworfen statt vermischt. Bei der Konvention ist der Grund ein
        anderer als bei den Achsen: die Spaltenzahl bliebe gleich, nur die
        Bedeutung waere ab der Umschaltzeile eine andere -- ein Unterschied,
        den die Datei nicht zeigt und den auch der Leser nicht finden kann.
        """
        joints, frames = list(joint_names), list(frame_names)
        gewaehlt = konvention.pruefe(drehkonvention or self.konvention)
        if (joints != self.joint_names or frames != self.frame_names
                or gewaehlt != self.konvention):
            self.clear()
            self.joint_names, self.frame_names = joints, frames
            self.konvention = gewaehlt
        self.joint_tolerances = list(joint_tolerances)
        self.chain = list(chain)
        self.couplings = list(couplings)
        self.robot_name = robot_name
        self.active = True
        return len(self.points)

    def stop(self):
        self.active = False

    def clear(self):
        """Nur die Punkte. Die Spaltenaufteilung bleibt, damit sich eine
        angehaltene Aufzeichnung fortsetzen laesst."""
        self.points.clear()
        self.start_time = None
        self.length = 0.0
        self.overflow = False

    def reset(self):
        """Punkte und Spaltenaufteilung verwerfen. Fuer den Modellwechsel:
        danach passt weder der Achssatz noch der Framesatz."""
        self.clear()
        self.joint_names = []
        self.joint_tolerances = []
        self.frame_names = []
        self.chain = []
        self.couplings = []
        self.konvention = konvention.STANDARD

    # -- Abtastung ---------------------------------------------------------

    def _tolerance(self, index):
        """Ruheschwelle der Achsspalte `index`, sonst der alte Rueckfallwert."""
        if index < len(self.joint_tolerances):
            return self.joint_tolerances[index]
        return self.min_joint

    def sample(self, xyz, abc, values, frames=(), now=None):
        """Einen Punkt anbieten. Gibt zurueck, ob er abgelegt wurde.

        `frames` ist flach: je Frame sechs Zahlen, x y z A B C, in der
        Reihenfolge von `frame_names`. Die Winkel sind in der Konvention
        gemeint, die `start` festgelegt hat -- der Rekorder rechnet nicht um
        und prueft es auch nicht nach, er kann es nicht.
        """
        if not self.active:
            return False
        now = time.monotonic() if now is None else now
        if self.start_time is None:
            self.start_time = now

        step = 0.0
        if self.points:
            _t, last_xyz, last_abc, last_values, _f = self.points[-1]
            step = math.sqrt(sum((a - b) ** 2 for a, b in zip(xyz, last_xyz)))
            # Die drei Winkel reichen als Ruhekriterium. Nahe der Entartung
            # springen sie, das erzeugt hoechstens einen Punkt zu viel, nie
            # zu wenig -- bei ZYZ und ZXZ ist das haeufiger als bei ZYX, weil
            # die Entartung dort in der Grundstellung liegt.
            turn = max(angle_delta(a, b) for a, b in zip(abc, last_abc))
            # Dritte Bedingung fuer den redundanten Fall: der TCP steht, der
            # Arm stellt sich trotzdem um. Die Schwelle steht in derselben
            # Einheit wie die Spalte, deshalb je Spalte eine eigene.
            moved = any(abs(a - b) >= self._tolerance(index)
                        for index, (a, b) in enumerate(zip(values, last_values)))
            if step < self.min_dist and turn < self.min_angle and not moved:
                return False

        if len(self.points) == self.points.maxlen:
            self.overflow = True
        self.points.append((now - self.start_time, tuple(xyz), tuple(abc),
                            tuple(values), tuple(frames)))
        self.length += step                            # zaehlt weiter, auch wenn
        return True                                    # vorne Punkte herausfallen

    # -- Auswertung --------------------------------------------------------

    @property
    def count(self):
        return len(self.points)

    @property
    def duration(self):
        """Zeit vom ersten bis zum letzten abgelegten Punkt."""
        if len(self.points) < 2:
            return 0.0
        return self.points[-1][0] - self.points[0][0]

    def header(self):
        """Spaltenkopf. Erst der TCP, dann die Achsen, dann je Frame ein Block.

        Die Winkelspalten tragen die Achse im Namen: a_z_deg, b_y_deg,
        c_x_deg bei ZYX. Nicht aus Sorgfalt allein -- eine Datei, deren
        Spalten roll_deg heissen und in ZYZ geschrieben sind, liest sich
        anstandslos und bedeutet etwas anderes. Der Name macht den Fehler
        sichtbar, die Kennung im Kopf macht ihn pruefbar.
        """
        columns = ["t_s", "x_m", "y_m", "z_m"]
        columns += list(konvention.spaltennamen("", self.konvention))
        columns += list(self.joint_names)
        for name in self.frame_names:
            columns += [f"{name}_x_m", f"{name}_y_m", f"{name}_z_m"]
            columns += list(konvention.spaltennamen(f"{name}_", self.konvention))
        return columns

    def write_csv(self, path, zusatz=()):
        """Punkte als CSV ablegen. Winkel in grad, Laengen in m.

        Vor dem Spaltenkopf stehen Kommentarzeilen mit Roboternamen, Kette und
        Kopplungen. Sie beginnen mit einem Rautenzeichen, halten aber dasselbe
        Trennzeichen ein und sind mit demselben Leser zu verarbeiten. So wandert
        der Aufbau des Modells mit der Datei mit und geht beim Kopieren nicht
        verloren, wie es bei einer Begleitdatei passieren wuerde.

        Die Kopplungen gehoeren dazu, weil ein gekoppeltes Gelenk in den Spalten
        wie eine freie Achse aussieht. Ohne den Hinweis waere spaeter nicht mehr
        zu erkennen, dass es einem anderen folgt.
        """
        with open(path, "w", newline="") as handle:
            writer = csv.writer(handle, delimiter=";")
            writer.writerow(["#roboter", self.robot_name])
            writer.writerow(["#erzeugt", datetime.now().strftime("%Y-%m-%d %H:%M:%S")])
            writer.writerow(["#rate_hz", f"{RECORD_RATE_HZ:g}"])
            # Ohne diese Zeile waeren die drei Winkelspalten nicht deutbar.
            # Sie steht vor dem Spaltenkopf, damit ein Leser sie kennt,
            # bevor er die Spalten aufteilt.
            writer.writerow(["#" + konvention.KENNUNG_SCHLUESSEL,
                             self.konvention])
            writer.writerow(["#tcp", self.frame_names[-1] if self.frame_names else ""])
            # Seit 3.6: die Herkunft einer geplanten Bahn -- Verfahren,
            # Parameter und die gemessenen Eckwerte. Eine Aufzeichnung
            # traegt keine. Die Bahnansicht ueberliest unbekannte Zeilen.
            for zeile in zusatz:
                writer.writerow(list(zeile))
            for child, parent, joint in self.chain:
                writer.writerow(["#kette", child, parent, joint])
            for name, source, multiplier, offset in self.couplings:
                writer.writerow(["#kopplung", name, source,
                                 f"{multiplier:.6g}", f"{offset:.6g}"])
            writer.writerow(self.header())

            for stamp, xyz, abc, values, frames in self.points:
                row = [f"{stamp:.4f}"]
                row += [f"{v:.6f}" for v in xyz]
                row += [f"{math.degrees(v):.4f}" for v in abc]
                row += [f"{v:.6f}" for v in values]
                for index in range(0, len(frames), 6):
                    block = frames[index:index + 6]
                    row += [f"{v:.6f}" for v in block[:3]]
                    row += [f"{math.degrees(v):.4f}" for v in block[3:]]
                writer.writerow(row)
        return len(self.points)
