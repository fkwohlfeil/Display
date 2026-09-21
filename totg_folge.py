#!/usr/bin/env python3
"""Punktfolge ueber TOTG statt ueber den Pilz-Blender. Reine Rechnung.

Dieses Modul kennt kein Tkinter, kein ROS und kein NumPy -- dieselbe
Trennung wie sequenz.py. Es baut aus der Nutzlast, die sequenz.Folge fuer
/plan_sequence_path zusammenstellt, den Auftrag fuer totg_rechner.py, liest
dessen Antwort und rechnet die Kennwerte, die ein Vergleich mit Pilz
braucht. Gerechnet wird die Bahn selbst nicht hier und auch nicht in der
Oberflaeche, sondern in MoveIt: totg_rechner.py ruft
TimeOptimalTrajectoryGeneration ueber moveit_py auf, mit derselben
Parameterdatei, mit der move_group laeuft. Eine zweite Implementierung von
TOTG gaebe es damit nicht -- aus demselben Grund, aus dem die
Rueckwaertskinematik nur aus MoveIt kommt.

Was TOTG anders macht als Pilz

    Pilz plant jede Teilbahn fuer sich, mit Stillstand am Ende, und
    mischt zwei Teilbahnen innerhalb der Ueberschleifkugel. TOTG bekommt
    die Achsziele als Polylinie im Gelenkraum, rundet jede Ecke mit einem
    Kreisbogen ab und legt darueber ein zeitoptimales Profil unter den
    Achsgrenzen. Daraus folgen vier Unterschiede, die hier geprueft und
    gemeldet werden und nicht stillschweigend passieren:

        Fahrart. Zwischen zwei Punkten faehrt TOTG eine Gerade im
        Gelenkraum. LIN und CIRC haben keine Wirkung; die TCP-Bahn ist
        keine Gerade mehr.

        Radius. Gelesen wird nur, ob er null ist. Radius 0 heisst wie bei
        Pilz Genauhalt: dort endet ein Abschnitt, und TOTG rechnet jeden
        Abschnitt fuer sich, von Stillstand zu Stillstand. Ein Radius ueber
        null heisst verschleifen, und zwar mit path_tolerance -- der Wert
        des Radius selbst hat bei TOTG keine Entsprechung.

        Umkehr. Kehrt die Polylinie an einem Punkt im Gelenkraum um, lehnt
        TOTG die ganze Bahn ab ("The path requires a 180 deg. turn"). Das
        ist keine Schwaeche der Einstellung: eine Umkehr ist ohne Anhalten
        nicht zu fahren. abschnitte findet diese Punkte vorher, nach
        derselben Regel wie Path::create, und legt dort ebenfalls eine
        Abschnittsgrenze -- gemeldet, weil ein Radius ueber null dort
        nicht wirkt.

        Override. TOTG kennt einen Geschwindigkeits- und einen
        Beschleunigungsfaktor fuer die ganze Bahn. Eine Folge mit
        wechselndem Tempo ist deshalb ein Fehler und wird nicht auf einen
        der Werte gerundet.

        Zu dichte Punkte. TOTG verwirft einen Wegpunkt, wenn keine Achse
        um mehr als min_angle_change vom zuletzt behaltenen abweicht.
        Welche das sind, rechnet doppelte_punkte nach derselben Regel vor
        -- gemeldet und gezaehlt, nicht verborgen.

path_tolerance ist eine Laenge im Gelenkraum: Bogenmass bei Drehachsen,
Meter bei Schubachsen, und bei gemischten Achsen beides zugleich. Die
Abweichung am TCP ist eine Folge davon und keine Eingabe. Sie wird
deshalb nach dem Rechnen aus der Bahn bestimmt (eckwerte) und so
protokolliert -- fuer die Pareto-Kurve zaehlt, was herauskam, nicht was
eingestellt war.
"""

import json
import math

import sequenz

VERFAHREN_PILZ = "Pilz"
VERFAHREN_TOTG = "TOTG"
VERFAHREN = (VERFAHREN_PILZ, VERFAHREN_TOTG)

# Voreinstellungen. path_tolerance und min_angle_change sind die Werte, die
# MoveIt selbst als Vorgabe traegt (default_response_adapter_params.yaml).
# resample_dt nicht: MoveIt nimmt 0.1 s, und das ist fuer eine Messung zu
# grob -- zwischen zwei Stuetzpunkten laege eine ganze Ecke.
PATH_TOLERANCE_STANDARD = 0.1
RESAMPLE_DT_STANDARD = 0.01
MIN_ANGLE_CHANGE = 0.001

# Grenzen der Eingabe. TOTG selbst lehnt path_tolerance <= 0 ab; die
# Oberflaeche sagt es vorher und mit dem Namen des Feldes.
PATH_TOLERANCE_MAX = 10.0
RESAMPLE_DT_MIN = 1.0e-4
RESAMPLE_DT_MAX = 1.0

# So lange darf der Hilfsprozess brauchen. MoveItPy laedt beim Start das
# Modell, die Pipeline und den Szenenmonitor; das dauert Sekunden, nicht
# Millisekunden.
FRIST_S = 90.0

# Name des Startpunkts in den Meldungen. Er steht nicht in der Punktliste.
START_NAME = "Start"

# Stand des Moduls. urdf_gui3_7 verlangt mindestens 2 (Abschnitte), und der
# Auftrag traegt die Zahl mit, damit ein aelterer totg_rechner ihn ablehnt,
# statt die Abschnitte zu uebergehen.
STAND = 2

# Wie angle_tolerance in Path::create (time_optimal_trajectory_generation.cpp):
# ab cos(Winkel) <= -1 + UMKEHR_TOLERANZ gilt eine Ecke als Umkehr. Die
# Normen werden gegen die Maschinengenauigkeit geprueft, wie dort.
UMKEHR_TOLERANZ = 1e-05
EPSILON = 2.220446049250313e-16


# ----------------------------------------------------------------------------
# Eingabe
# ----------------------------------------------------------------------------

def pruefe_parameter(path_tolerance, resample_dt):
    """Befunde zu den zwei Feldern im Folgenfenster."""
    befunde = []
    try:
        toleranz = float(path_tolerance)
    except (TypeError, ValueError):
        befunde.append(sequenz.Befund(
            sequenz.FEHLER, f"path_tolerance {path_tolerance!r} ist keine Zahl"))
        toleranz = None
    try:
        takt = float(resample_dt)
    except (TypeError, ValueError):
        befunde.append(sequenz.Befund(
            sequenz.FEHLER, f"resample_dt {resample_dt!r} ist keine Zahl"))
        takt = None

    if toleranz is not None and not 0.0 < toleranz <= PATH_TOLERANCE_MAX:
        befunde.append(sequenz.Befund(
            sequenz.FEHLER,
            f"path_tolerance {toleranz:g} liegt nicht in "
            f"(0, {PATH_TOLERANCE_MAX:g}]. TOTG verlangt einen Wert ueber null."))
    if takt is not None and not RESAMPLE_DT_MIN <= takt <= RESAMPLE_DT_MAX:
        befunde.append(sequenz.Befund(
            sequenz.FEHLER,
            f"resample_dt {takt:g} s liegt nicht in "
            f"[{RESAMPLE_DT_MIN:g}, {RESAMPLE_DT_MAX:g}] s"))
    return tuple(befunde)


def behaltene_punkte(wegpunkte, min_angle_change=MIN_ANGLE_CHANGE):
    """Indizes der Wegpunkte, die TOTG behaelt, in Reihenfolge.

    Der letzte Index ist immer der letzte Punkt: faellt er heraus, ersetzt
    er den zuletzt behaltenen. Siehe doppelte_punkte.
    """
    grenze = float(min_angle_change)
    behalten = []
    anzahl = len(wegpunkte)
    for index, punkt in enumerate(wegpunkte):
        if index == 0:
            behalten.append(index)
            continue
        letzter = wegpunkte[behalten[-1]]
        verschieden = False
        for wert, alt in zip(punkt, letzter):
            if abs(float(wert) - float(alt)) > grenze:
                verschieden = True
                break
        if verschieden:
            behalten.append(index)
        elif index == anzahl - 1:
            behalten[-1] = index
    return behalten


def ist_umkehr(a, b, c):
    """True, wenn die Polylinie a-b-c in b um 180 grad kehrt.

    Dieselbe Rechnung wie in Path::create: Kosinus des Winkels zwischen
    ankommender und abgehender Richtung, Summenreihenfolge fest.
    """
    rein = [float(y) - float(x) for x, y in zip(a, b)]
    raus = [float(y) - float(x) for x, y in zip(b, c)]
    norm_rein = math.sqrt(sum(w * w for w in rein))
    norm_raus = math.sqrt(sum(w * w for w in raus))
    if norm_rein <= EPSILON or norm_raus <= EPSILON:
        return False
    kosinus = sum(x * y for x, y in zip(rein, raus)) / (norm_rein * norm_raus)
    return kosinus <= -1.0 + UMKEHR_TOLERANZ


def abschnitte(wegpunkte, halte=(), min_angle_change=MIN_ANGLE_CHANGE):
    """Die Polylinie an Genauhalten und Umkehrpunkten zerlegen.

    halte: Indizes der Wegpunkte, an denen angehalten wird (Radius 0).
    Liefert (grenzen, umkehr): grenzen beginnt mit 0 und endet mit dem
    letzten Index, dazwischen jede Stelle, an der ein Abschnitt endet;
    umkehr sind die Stellen, die nur wegen einer Umkehr Grenze sind.

    Gerechnet wird Abschnitt fuer Abschnitt und nicht einmal ueber alles.
    TOTG filtert die zu dichten Punkte in jedem Abschnitt neu, beginnend
    an dessen erstem Punkt -- und die Umkehr prueft es an den gefilterten
    Punkten. Wer hier ueber die ganze Liste filterte, saehe in einem
    Grenzfall eine andere Punktfolge als TOTG.
    """
    anzahl = len(wegpunkte)
    if anzahl < 2:
        return [0] * anzahl, []
    halte = sorted({int(i) for i in halte if 0 < int(i) < anzahl - 1})
    grenzen = [0]
    umkehr = []
    anfang = 0
    while anfang < anzahl - 1:
        ende = next((h for h in halte if h > anfang), anzahl - 1)
        stueck = wegpunkte[anfang:ende + 1]
        behalten = behaltene_punkte(stueck, min_angle_change)
        for a, b, c in zip(behalten, behalten[1:], behalten[2:]):
            if ist_umkehr(stueck[a], stueck[b], stueck[c]):
                ende = anfang + b
                umkehr.append(ende)
                break
        if ende <= anfang:                         # kann nicht vorkommen
            raise ValueError(f"Abschnitt ohne Fortschritt bei {anfang}")
        grenzen.append(ende)
        anfang = ende
    return grenzen, umkehr


def doppelte_punkte(wegpunkte, min_angle_change=MIN_ANGLE_CHANGE):
    """Welche Wegpunkte TOTG verwirft. Dieselbe Regel wie in MoveIt.

    TimeOptimalTrajectoryGeneration::doTimeParameterizationCalculations
    behaelt den ersten Punkt immer und danach jeden, bei dem wenigstens
    eine Achse um mehr als min_angle_change vom zuletzt behaltenen
    abweicht. Faellt der letzte Punkt heraus, ersetzt er den zuletzt
    behaltenen -- das Ende der Bahn bleibt so das gewuenschte.

    Liefert [(verworfen, statt), ...]: den Index des Punktes, der nicht in
    die Bahn eingeht, und den Index des Punktes, der an seiner Stelle
    steht. Beim letzten Punkt ist das umgekehrt: verworfen ist der
    zuletzt behaltene, und statt seiner steht der letzte.
    """
    grenze = float(min_angle_change)
    behalten = []
    verworfen = []
    anzahl = len(wegpunkte)
    for index, punkt in enumerate(wegpunkte):
        if index == 0:
            behalten.append(index)
            continue
        letzter = wegpunkte[behalten[-1]]
        verschieden = False
        for wert, alt in zip(punkt, letzter):
            if abs(float(wert) - float(alt)) > grenze:
                verschieden = True
                break
        if verschieden:
            behalten.append(index)
        elif index == anzahl - 1:
            verworfen.append((behalten[-1], index))
            behalten[-1] = index
        else:
            verworfen.append((index, behalten[-1]))
    return verworfen


def anfrage(nutzlast, gelenke, drehend, params_datei,
            path_tolerance=PATH_TOLERANCE_STANDARD,
            resample_dt=RESAMPLE_DT_STANDARD,
            min_angle_change=MIN_ANGLE_CHANGE):
    """Auftrag fuer totg_rechner.py aus der Nutzlast der Folge.

    nutzlast: das Ergebnis von sequenz.Folge.nutzlast, alle Posten mit
    Achszielen. gelenke: die beweglichen, nicht gekoppelten Gelenke in der
    Reihenfolge des Fahrfensters. drehend: je Gelenk True bei Dreh-, False
    bei Schubachse.

    Liefert (auftrag, befunde). Traegt befunde einen Fehler, ist auftrag
    None -- ein halber Auftrag geht nicht hinaus.
    """
    befunde = list(pruefe_parameter(path_tolerance, resample_dt))
    gelenke = [str(name) for name in gelenke]
    drehend = [bool(wert) for wert in drehend]
    posten = list(nutzlast.get("items") or [])

    if len(drehend) != len(gelenke):
        befunde.append(sequenz.Befund(
            sequenz.FEHLER,
            f"{len(gelenke)} Gelenke, aber {len(drehend)} Achsarten"))
    if not gelenke:
        befunde.append(sequenz.Befund(sequenz.FEHLER, "Keine beweglichen Gelenke"))
    if not posten:
        befunde.append(sequenz.Befund(sequenz.FEHLER, "Die Folge ist leer"))
    if not params_datei:
        befunde.append(sequenz.Befund(
            sequenz.FEHLER,
            "Keine Parameterdatei. TOTG braucht die Achsgrenzen, mit denen "
            "move_group laeuft -- move_group muss aus dem Parameterfenster "
            "gestartet sein."))
    if any(befund.stufe == sequenz.FEHLER for befund in befunde):
        return None, tuple(befunde)

    start = posten[0].get("start") or {}
    fehlend = [name for name in gelenke if name not in start]
    if fehlend:
        befunde.append(sequenz.Befund(
            sequenz.FEHLER, f"Der Startzustand fuehrt {', '.join(fehlend)} nicht"))

    namen = [START_NAME]
    wegpunkte = [[float(start.get(name, 0.0)) for name in gelenke]]
    for index, eintrag in enumerate(posten):
        name = eintrag.get("name") or f"P{index + 1}"
        ziel = eintrag.get("goal")
        if not ziel:
            befunde.append(sequenz.Befund(
                sequenz.FEHLER, f"{name}: kein Achsziel. TOTG nimmt nur Achsziele.",
                stelle=index))
            continue
        luecke = [gelenk for gelenk in gelenke if gelenk not in ziel]
        if luecke:
            befunde.append(sequenz.Befund(
                sequenz.FEHLER,
                f"{name}: Achsziel ohne {', '.join(luecke)}", stelle=index))
            continue
        namen.append(name)
        wegpunkte.append([float(ziel[gelenk]) for gelenk in gelenke])

    # Ein Override fuer die ganze Bahn. Verglichen wird exakt: die Werte
    # stammen alle aus derselben Rechnung in sequenz.py, ein Rundungsrest
    # kann dort nicht entstehen.
    paare = []
    for eintrag in posten:
        paar = (float(eintrag["velocity_scaling"]),
                float(eintrag["acceleration_scaling"]))
        if paar not in paare:
            paare.append(paar)
    if len(paare) > 1:
        befunde.append(sequenz.Befund(
            sequenz.FEHLER,
            "TOTG kennt einen Override fuer die ganze Bahn. Die Folge traegt "
            + str(len(paare)) + " verschiedene: "
            + ", ".join(f"{v * 100:g} %/{a * 100:g} %" for v, a in paare)
            + ". Tempo und Beschleunigung einmal am Anfang setzen."))

    fahrarten = sorted({str(eintrag.get("planner_id", "")) for eintrag in posten}
                       - {"PTP"})
    if fahrarten:
        befunde.append(sequenz.Befund(
            sequenz.WARNUNG,
            f"Die Folge traegt {', '.join(fahrarten)}. TOTG verbindet die "
            f"Punkte im Gelenkraum; die TCP-Bahn dazwischen ist keine Gerade "
            f"und kein Bogen. Vergleichbar mit Pilz ist sie nur bei PTP."))

    # Wegpunkt i gehoert zu Posten i - 1; Wegpunkt 0 ist der Start.
    radien = [float(eintrag.get("blend_radius", 0.0)) for eintrag in posten]
    halte = [index + 1 for index, radius in enumerate(radien[:-1])
             if radius == 0.0]
    grenzen, umkehr = abschnitte(wegpunkte, halte, min_angle_change)
    innen = range(1, len(wegpunkte) - 1)
    verschliffen = [i for i in innen if i not in grenzen]
    if halte:
        befunde.append(sequenz.Befund(
            sequenz.HINWEIS,
            "Genauhalt (Radius 0) an " + ", ".join(namen[i] for i in halte)
            + ". TOTG haelt dort an, wie Pilz."))
    if verschliffen:
        befunde.append(sequenz.Befund(
            sequenz.HINWEIS,
            "Mit path_tolerance " + f"{float(path_tolerance):g}"
            + " verschliffen: " + ", ".join(namen[i] for i in verschliffen)
            + ". Gelesen wird nur, dass der Radius ueber null liegt, nicht "
              "sein Wert."))
    for index in umkehr:
        befunde.append(sequenz.Befund(
            sequenz.WARNUNG,
            f"{namen[index]}: die Bahn kehrt dort im Gelenkraum um. TOTG "
            f"kann eine Umkehr nicht verschleifen; angehalten wird trotz "
            f"Radius {radien[index - 1]:g} m.", stelle=index))

    if drehend and any(drehend) and not all(drehend):
        befunde.append(sequenz.Befund(
            sequenz.WARNUNG,
            "Dreh- und Schubachsen gemischt. path_tolerance gilt dann in rad "
            "und m zugleich; MoveIt warnt dabei selbst, dass die Toleranz "
            "nicht richtig wirkt."))

    # Je Abschnitt, wie TOTG filtert.
    for anfang, ende in zip(grenzen, grenzen[1:]):
        stueck = wegpunkte[anfang:ende + 1]
        for weg, statt in doppelte_punkte(stueck, min_angle_change):
            weg, statt = anfang + weg, anfang + statt
            befunde.append(sequenz.Befund(
                sequenz.WARNUNG,
                f"{namen[weg]} liegt innerhalb von {float(min_angle_change):g} "
                f"je Achse bei {namen[statt]}. TOTG verwirft ihn; die Bahn "
                f"fuehrt nicht eigens dorthin.", stelle=weg))

    if any(befund.stufe == sequenz.FEHLER for befund in befunde):
        return None, tuple(befunde)

    auftrag = {
        "params_datei": str(params_datei),
        "gruppe": str(nutzlast.get("group") or posten[0]["group"]),
        "gelenke": gelenke,
        "start": {str(k): float(v) for k, v in start.items()},
        "namen": namen,
        "wegpunkte": wegpunkte,
        "velocity_scaling": paare[0][0],
        "acceleration_scaling": paare[0][1],
        "path_tolerance": float(path_tolerance),
        "resample_dt": float(resample_dt),
        "min_angle_change": float(min_angle_change),
        "abschnitte": [[a, b] for a, b in zip(grenzen, grenzen[1:])],
        "umkehr": list(umkehr),
        "stand": STAND,
    }
    return auftrag, tuple(befunde)


def auftrag_text(auftrag):
    """Als JSON. repr-genau, damit die Zahlen unveraendert ankommen."""
    return json.dumps(auftrag, indent=1, sort_keys=True) + "\n"


# ----------------------------------------------------------------------------
# Antwort
# ----------------------------------------------------------------------------

def antwort_lesen(text, gelenke):
    """Antwort von totg_rechner.py in die Form von RosBridge._read_sequence.

    Liefert ein Woerterbuch mit points, teilbahnen, planning_time und dem
    Block totg. Wirft ValueError, wenn die Antwort nicht zum Auftrag passt
    -- eine Bahn mit anderen Achsen gehoert zu einem anderen Modell und
    wird nicht halb uebernommen.
    """
    try:
        daten = json.loads(text)
    except ValueError as fehler:
        raise ValueError(f"Antwort ist kein JSON: {fehler}") from None
    if not isinstance(daten, dict):
        raise ValueError("Antwort ist kein Objekt")
    if not daten.get("ok"):
        raise ValueError(str(daten.get("fehler") or "TOTG ohne Angabe gescheitert"))

    namen = list(daten.get("gelenke") or [])
    if namen != list(gelenke):
        raise ValueError(f"Antwort fuehrt die Achsen {namen}, erwartet "
                         f"{list(gelenke)}")

    punkte = []
    for nummer, roh in enumerate(daten.get("punkte") or []):
        if len(roh) != 3 or len(roh[1]) != len(namen) or len(roh[2]) != len(namen):
            raise ValueError(f"Stuetzpunkt {nummer} ist unvollstaendig")
        zeit = float(roh[0])
        if not math.isfinite(zeit):
            raise ValueError(f"Stuetzpunkt {nummer} traegt keine endliche Zeit")
        lagen = {name: float(wert) for name, wert in zip(namen, roh[1])}
        tempi = {name: float(wert) for name, wert in zip(namen, roh[2])}
        punkte.append((zeit, lagen, tempi))
    if len(punkte) < 2:
        raise ValueError("TOTG lieferte weniger als zwei Stuetzpunkte")

    return {
        "code": 1,
        "points": punkte,
        # TOTG liefert eine einzige Bahn. Die Zahl steht trotzdem da, damit
        # der Bericht fuer beide Verfahren derselbe bleibt.
        "teilbahnen": [int(n) for n in daten.get("teilbahnen") or [len(punkte)]],
        "planning_time": float(daten.get("rechenzeit_s", 0.0)),
        "totg": {
            "dauer_s": float(daten.get("dauer_s", punkte[-1][0])),
            "moveit_version": str(daten.get("moveit_version", "")),
            "moveit_pfad": str(daten.get("moveit_pfad", "")),
            "eingang": int(daten.get("eingang", 0)),
            "start_s": float(daten.get("start_s", 0.0)),
        },
    }


def endabweichung(punkte, auftrag):
    """Wie weit liegt das Ende der Bahn vom letzten Wegpunkt weg?

    TOTG wickelt Endlosachsen vorher ab (RobotTrajectory::unwind). Liegt ein
    Ziel mehr als eine halbe Umdrehung vom Vorgaenger weg, faehrt die Bahn
    den kuerzeren Weg und endet um 2 pi versetzt. Das ist keine Kleinigkeit
    und wird deshalb gemessen: (name, betrag) der groessten Abweichung.
    """
    if not punkte or not auftrag.get("wegpunkte"):
        return (None, 0.0)
    ende = punkte[-1][1]
    ziel = auftrag["wegpunkte"][-1]
    schlimmste, betrag = None, 0.0
    for name, soll in zip(auftrag["gelenke"], ziel):
        ab = abs(float(ende.get(name, 0.0)) - float(soll))
        if schlimmste is None or ab > betrag:
            schlimmste, betrag = name, ab
    return (schlimmste, betrag)


# ----------------------------------------------------------------------------
# Kennwerte
# ----------------------------------------------------------------------------

def _abstand_zur_strecke(p, a, b):
    """Abstand des Punktes p von der Strecke a-b und der Anteil darauf."""
    ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]]
    ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]]
    laenge2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]
    if laenge2 <= 0.0:
        anteil = 0.0
    else:
        anteil = (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / laenge2
        anteil = min(1.0, max(0.0, anteil))
    q = [a[0] + anteil * ab[0], a[1] + anteil * ab[1], a[2] + anteil * ab[2]]
    return sequenz.abstand(p, q), anteil


def eckwerte(zeiten, orte, ecken):
    """Wie nah kommt die Bahn an jede Ecke, und wie schnell ist sie dort?

    zeiten: Sekunden je Stuetzpunkt. orte: TCP-Ort je Stuetzpunkt in m.
    ecken: [(name, (x, y, z)), ...], die Zwischenpunkte der Folge.

    Gesucht wird ueber die Strecken zwischen den Stuetzpunkten und nicht
    nur ueber die Stuetzpunkte selbst: bei resample_dt = 0.01 s und 0.5 m/s
    liegen zwei Stuetzpunkte 5 mm auseinander, und das ist die Groessen-
    ordnung der Abweichung, die hier gemessen werden soll.

    Die Geschwindigkeit ist die mittlere auf der naechsten Strecke. Liefert
    [{"name", "abstand_m", "v_m_s", "zeit_s"}, ...].
    """
    if len(orte) != len(zeiten):
        raise ValueError("zeiten und orte sind verschieden lang")
    ergebnis = []
    for name, ecke in ecken:
        bester = None
        for index in range(len(orte) - 1):
            weite, anteil = _abstand_zur_strecke(ecke, orte[index], orte[index + 1])
            if bester is None or weite < bester[0]:
                bester = (weite, index, anteil)
        if bester is None:
            ergebnis.append({"name": name, "abstand_m": sequenz.abstand(ecke, orte[0])
                             if orte else float("nan"),
                             "v_m_s": 0.0, "zeit_s": zeiten[0] if zeiten else 0.0})
            continue
        weite, index, anteil = bester
        dauer = float(zeiten[index + 1]) - float(zeiten[index])
        weg = sequenz.abstand(orte[index], orte[index + 1])
        tempo = weg / dauer if dauer > 0.0 else 0.0
        zeit = float(zeiten[index]) + anteil * dauer
        ergebnis.append({"name": name, "abstand_m": weite,
                         "v_m_s": tempo, "zeit_s": zeit})
    return ergebnis


def zusatzzeilen(herkunft):
    """Kommentarzeilen fuer die CSV. Die Bahnansicht ueberliest sie.

    herkunft ist das Woerterbuch, das die Oberflaeche beim Planen ablegt.
    Geschrieben wird, womit gerechnet wurde, und dazu die gemessenen
    Eckwerte -- beides gehoert zur Datei und nicht in eine Begleitnotiz.
    """
    zeilen = [["#verfahren", str(herkunft.get("verfahren", ""))]]
    for schluessel in ("override_v", "override_a", "dauer_s",
                       "sampling_time", "path_tolerance", "resample_dt",
                       "min_angle_change", "ruckig", "teilbahnen", "verworfen",
                       "endabweichung", "umkehr", "moveit_version",
                       "moveit_pfad",
                       "hinweis"):
        if schluessel in herkunft:
            wert = herkunft[schluessel]
            text = repr(wert) if isinstance(wert, float) else str(wert)
            zeilen.append(["#" + schluessel, text])
    for name, fahrart, radius in herkunft.get("punkte", ()):
        zeilen.append(["#punkt", str(name), str(fahrart), repr(float(radius))])
    if herkunft.get("ecken"):
        zeilen.append(["#ecke_spalten", "name", "abstand_mm", "v_mm_s", "zeit_s"])
    for eintrag in herkunft.get("ecken", ()):
        zeilen.append(["#ecke", str(eintrag["name"]),
                       f"{eintrag['abstand_m'] * 1000.0:.4f}",
                       f"{eintrag['v_m_s'] * 1000.0:.3f}",
                       f"{eintrag['zeit_s']:.4f}"])
    return zeilen
