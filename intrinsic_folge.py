#!/usr/bin/env python3
"""Punktfolge ueber Intrinsic Core (LIN und PTP, Variante B). Reine Rechnung.

Dieses Modul kennt kein Tkinter, kein ROS und kein NumPy -- dieselbe
Trennung wie sequenz.py und totg_folge.py. Es baut aus der Nutzlast, die
sequenz.Folge fuer /plan_sequence_path zusammenstellt, den Auftrag fuer
intrinsic_lin_rechner.py und liest dessen Antwort. Gerechnet wird die Bahn
nicht hier: die verrundete kartesische Kurve und TOPP-RA kommen aus
Intrinsic Core (ueber den Wrapper), die Rueckwaertskinematik aus MoveIt
(ueber moveit_py im Rechner).

Was Intrinsic anders macht als Pilz und TOTG

    Fahrart. Sie wird je Zeile ausgewertet. LIN faehrt kartesisch linear
    (verrundete Kurve, IK-Kette), PTP gerade im Gelenkraum -- die Achsziele
    gehen ohne IK an TOPP-RA. CIRC kennt diese Rechnung nicht; eine Zeile
    mit CIRC ist ein Fehler.

    Wechsel der Fahrart. Radius 0 heisst Genauhalt. Radius ueber null heisst
    ueberschleifen, und zwar wie in Intrinsic nur im Gelenkraum mit 0,2 rad:
    die beiden Abschnitte gehen als ein Teilpfad an TOPP-RA. Der Wert des
    Radius wirkt dort nicht als Laenge, und nahe am Punkt verlaesst die
    Bahn die LIN-Gerade.

    Radius bei PTP. Ein Radius ueber null zwischen zwei PTP-Zeilen heisst
    ueberschleifen, im Gelenkraum mit 0,2 rad wie in Intrinsic. Der Wert
    des Radius wirkt dort nicht als Laenge.

    Radius bei LIN. Radius 0 heisst wie bei Pilz und TOTG Genauhalt: dort endet ein
    Abschnitt, und jeder Abschnitt beginnt und endet im Stillstand. Ein
    Radius ueber null heisst verrunden, und sein Wert geht als
    translatorische Eckenverrundung in Intrinsic ein -- aber als Abstand der
    Zwischenpunkte im Kontrollpolygon eines B-Splines, nicht als Kugel wie
    bei Pilz. Intrinsic kennt einen Wert fuer die ganze Kurve; verschiedene
    Radien ueber null in einer Folge sind deshalb ein Fehler und werden
    nicht gemittelt.

    Drehung. Dazu kommt eine rotatorische Verrundung (rad) aus dem
    Folgenfenster. Intrinsic wendet an jeder Ecke nur die strengere der
    beiden an (gemessen am Anteil der Strecke zum Nachbarpunkt).

    Umkehr. Kehrt die Folge im Gelenkraum um, wird dort angehalten, nach
    derselben Regel wie bei TOTG (totg_folge.abschnitte) -- auch an einem
    Wechsel der Fahrart mit Radius.

    Override. Ein Geschwindigkeits- und ein Beschleunigungsfaktor fuer die
    ganze Bahn, wie bei TOTG.

    Kein Ruck-Limit, keine TCP-Geschwindigkeitsgrenze (Intrinsic braeuchte
    dafuer ein eigenes Kinematikmodell), keine Kollisionspruefung. Das steht
    im Bericht und in der CSV.
"""

import json
import os

import sequenz
import totg_folge

VERFAHREN_INTRINSIC = "Intrinsic"

# Stand des Moduls. urdf_gui verlangt mindestens 1.
STAND = 1

# Stand des Auftrags fuer intrinsic_lin_rechner.py (dessen STAND). Seit 2
# mit "fahrarten" je Abschnitt; ein Rechner mit Stand 1 wuerde PTP still
# als LIN fahren und lehnt den Auftrag deshalb ab. Seit 4 mit
# "verschleifen" (Wechsel der Fahrart ohne Halt). Stand 3 hatte nur der
# Rechner (Mimic-Patch) -- ein solcher Rechner muss diesen Auftrag ablehnen,
# sonst hielte er an jedem Wechsel still an.
AUFTRAG_STAND = 4

# Fahrarten, die diese Rechnung kennt.
FAHRARTEN = ("LIN", "PTP")

# Ueberschleifen im Gelenkraum (rad) an den inneren Punkten eines
# Abschnitts -- der dichten LIN-Pfade wie der PTP-Wegpunkte. Wie
# kCartMotionLowLevelJointBlendingParameterRad in Intrinsic.
UEBERSCHLEIFEN_RAD = 0.2

# Rechner und Wrapper. Der Wrapper liegt im Bazel-Baum von intrinsic-core;
# INTRINSIC_WRAPPER in der Umgebung geht vor.
RECHNER = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "intrinsic_lin_rechner.py")
WRAPPER_STANDARD = os.path.expanduser(
    "~/intrinsic-core/bazel-bin/intrinsic_toppra_wrapper/"
    "intrinsic_toppra_wrapper")


def wrapper_pfad():
    return os.environ.get("INTRINSIC_WRAPPER") or WRAPPER_STANDARD


# Voreinstellungen im Folgenfenster.
RUNDUNG_ROT_STANDARD = 0.1          # rad
# Intrinsic verlangt eine Verrundung > 0. Ohne Radius ueber null in der
# Liste haelt die Bahn an jedem Punkt; der Wert wirkt dann nirgends und
# steht auf dem Minimum des Intrinsic Motion Planners.
RUNDUNG_TRANS_OHNE_RADIUS = 0.001   # m

RUNDUNG_ROT_MIN = 1.0e-6
RUNDUNG_ROT_MAX = 3.14159

# So lange darf der Rechner brauchen. MoveItPy faehrt hoch, und je
# Abschnitt laufen zwei IK-Ketten ueber einige hundert Kurvenposen.
FRIST_S = 300.0


# ----------------------------------------------------------------------------
# Eingabe
# ----------------------------------------------------------------------------

def pruefe_parameter(rundung_rot, resample_dt):
    """Befunde zu den Feldern im Folgenfenster."""
    befunde = []
    try:
        rot = float(rundung_rot)
    except (TypeError, ValueError):
        befunde.append(sequenz.Befund(
            sequenz.FEHLER, f"rundung_rot {rundung_rot!r} ist keine Zahl"))
        rot = None
    if rot is not None and not RUNDUNG_ROT_MIN <= rot <= RUNDUNG_ROT_MAX:
        befunde.append(sequenz.Befund(
            sequenz.FEHLER,
            f"rundung_rot {rot:g} rad liegt nicht in "
            f"[{RUNDUNG_ROT_MIN:g}, {RUNDUNG_ROT_MAX:g}]"))
    # resample_dt nach denselben Grenzen wie bei TOTG. path_tolerance wird
    # dort mitgeprueft; hier geht der Standardwert hinein, damit nur
    # Befunde zu resample_dt entstehen koennen.
    for befund in totg_folge.pruefe_parameter(
            totg_folge.PATH_TOLERANCE_STANDARD, resample_dt):
        befunde.append(befund)
    return tuple(befunde)


def anfrage(nutzlast, gelenke, params_datei, tip, rundung_rot, resample_dt,
            wrapper=None, grenzen_lesen=None):
    """Auftrag fuer intrinsic_lin_rechner.py aus der Nutzlast der Folge.

    nutzlast: sequenz.Folge.nutzlast, alle Posten mit Achszielen. gelenke:
    die beweglichen, nicht gekoppelten Gelenke in der Reihenfolge des
    Fahrfensters. tip: der Link, fuer den die Lagen der Liste gelten.

    grenzen_lesen(params_datei, gelenke) -> (grenzen, hinweise) liefert die
    Achsgrenzen aus der Parameterdatei von move_group; Standard ist
    intrinsic_lin_auftrag.grenzen_lesen. Fuer Tests austauschbar.

    Liefert (auftrag, befunde). Traegt befunde einen Fehler, ist auftrag
    None -- ein halber Auftrag geht nicht hinaus.
    """
    befunde = list(pruefe_parameter(rundung_rot, resample_dt))
    gelenke = [str(name) for name in gelenke]
    posten = list(nutzlast.get("items") or [])
    wrapper = wrapper or wrapper_pfad()

    if not gelenke:
        befunde.append(sequenz.Befund(sequenz.FEHLER,
                                      "Keine beweglichen Gelenke"))
    if not posten:
        befunde.append(sequenz.Befund(sequenz.FEHLER, "Die Folge ist leer"))
    if not params_datei:
        befunde.append(sequenz.Befund(
            sequenz.FEHLER,
            "Keine Parameterdatei. Die Rueckwaertskinematik kommt aus MoveIt "
            "mit derselben Datei wie move_group -- move_group muss aus dem "
            "Parameterfenster gestartet sein."))
    if not tip:
        befunde.append(sequenz.Befund(
            sequenz.FEHLER, "Kein Tip-Link. Im Parameterfenster eintragen."))
    if not (os.path.isfile(wrapper) and os.access(wrapper, os.X_OK)):
        befunde.append(sequenz.Befund(
            sequenz.FEHLER,
            f"Wrapper nicht ausfuehrbar: {wrapper}. In ~/intrinsic-core "
            f"bauen: bazel build "
            f"//intrinsic_toppra_wrapper:intrinsic_toppra_wrapper -- oder "
            f"INTRINSIC_WRAPPER setzen."))
    if any(befund.stufe == sequenz.FEHLER for befund in befunde):
        return None, tuple(befunde)

    start = posten[0].get("start") or {}
    fehlend = [name for name in gelenke if name not in start]
    if fehlend:
        befunde.append(sequenz.Befund(
            sequenz.FEHLER,
            f"Der Startzustand fuehrt {', '.join(fehlend)} nicht"))

    namen = [totg_folge.START_NAME]
    wegpunkte = [[float(start.get(name, 0.0)) for name in gelenke]]
    for index, eintrag in enumerate(posten):
        name = eintrag.get("name") or f"P{index + 1}"
        ziel = eintrag.get("goal")
        if not ziel:
            befunde.append(sequenz.Befund(
                sequenz.FEHLER,
                f"{name}: kein Achsziel. Intrinsic bekommt Achsziele; die "
                f"Lage entsteht daraus mit MoveIt.", stelle=index))
            continue
        luecke = [gelenk for gelenk in gelenke if gelenk not in ziel]
        if luecke:
            befunde.append(sequenz.Befund(
                sequenz.FEHLER,
                f"{name}: Achsziel ohne {', '.join(luecke)}", stelle=index))
            continue
        namen.append(name)
        wegpunkte.append([float(ziel[gelenk]) for gelenk in gelenke])

    paare = []
    for eintrag in posten:
        paar = (float(eintrag["velocity_scaling"]),
                float(eintrag["acceleration_scaling"]))
        if paar not in paare:
            paare.append(paar)
    if len(paare) > 1:
        befunde.append(sequenz.Befund(
            sequenz.FEHLER,
            "Intrinsic (TOPP-RA) kennt einen Override fuer die ganze Bahn. "
            "Die Folge traegt " + str(len(paare)) + " verschiedene: "
            + ", ".join(f"{v * 100:g} %/{a * 100:g} %" for v, a in paare)
            + ". Tempo und Beschleunigung einmal am Anfang setzen."))

    # Posten i faehrt von Wegpunkt i nach Wegpunkt i + 1; Wegpunkt 0 ist
    # der Start. Zwischen Posten i und i + 1 liegt also Wegpunkt i + 1.
    arten = [str(eintrag.get("planner_id", "")) for eintrag in posten]
    for index, art in enumerate(arten):
        if art not in FAHRARTEN:
            name = posten[index].get("name") or f"P{index + 1}"
            befunde.append(sequenz.Befund(
                sequenz.FEHLER,
                f"{name}: Fahrart {art or '(keine)'} kennt "
                f"Intrinsic hier nicht, nur {' und '.join(FAHRARTEN)}.",
                stelle=index))
    if any(befund.stufe == sequenz.FEHLER for befund in befunde):
        return None, tuple(befunde)

    radien = [float(eintrag.get("blend_radius", 0.0)) for eintrag in posten]
    wechsel = [index + 1 for index in range(len(posten) - 1)
               if arten[index] != arten[index + 1]]
    genau = [index + 1 for index, radius in enumerate(radien[:-1])
             if radius == 0.0]

    # Nur zwischen zwei LIN-Zeilen geht der Radius in die Kurve ein.
    ueber_null = sorted({radien[index] for index in range(len(posten) - 1)
                         if radien[index] > 0.0
                         and arten[index] == arten[index + 1] == "LIN"})
    if len(ueber_null) > 1:
        befunde.append(sequenz.Befund(
            sequenz.FEHLER,
            "Intrinsic verrundet die ganze Kurve mit einem Wert. Die Folge "
            "traegt zwischen LIN-Zeilen die Radien "
            + ", ".join(f"{r:g}" for r in ueber_null)
            + " m. Einen Radius fuer alle verschliffenen Punkte setzen "
              "(Radius 0 bleibt Genauhalt)."))
    rundung_trans = ueber_null[0] if ueber_null else RUNDUNG_TRANS_OHNE_RADIUS

    # Angehalten wird nur an Genauhalt (Radius 0) und Umkehr. Ein Wechsel
    # der Fahrart ist trotzdem Abschnittsgrenze -- jeder Abschnitt hat eine
    # Fahrart --, aber mit Radius ueber null wird dort nicht angehalten: der
    # Rechner haengt die beiden Abschnitte zu einem Teilpfad zusammen, und
    # TOPP-RA ueberschleift den Punkt im Gelenkraum.
    halte, umkehr = totg_folge.abschnitte(wegpunkte, genau)
    grenzen = sorted(set(halte) | set(wechsel))
    stuecke = list(zip(grenzen, grenzen[1:]))
    uebergaenge = [i for i in wechsel if i not in halte]
    innen = range(1, len(wegpunkte) - 1)
    verschliffen = [i for i in innen if i not in grenzen]
    lin_rund = [i for i in verschliffen if arten[i - 1] == "LIN"]
    ptp_rund = [i for i in verschliffen if arten[i - 1] == "PTP"]
    if genau:
        befunde.append(sequenz.Befund(
            sequenz.HINWEIS,
            "Genauhalt (Radius 0) an " + ", ".join(namen[i] for i in genau)
            + ". Intrinsic haelt dort an, wie Pilz."))
    if uebergaenge:
        befunde.append(sequenz.Befund(
            sequenz.HINWEIS,
            f"Wechsel der Fahrart ueberschliffen im Gelenkraum mit "
            f"{UEBERSCHLEIFEN_RAD:g} rad: "
            + ", ".join(f"{namen[i]} ({arten[i - 1]} -> {arten[i]})"
                        for i in uebergaenge)
            + ". Wie in Intrinsic; der Radius der Zeile wirkt dort nicht als "
              "Laenge, und die LIN-Strecke wird nahe am Punkt verlassen."))
    if lin_rund:
        befunde.append(sequenz.Befund(
            sequenz.HINWEIS,
            f"Verrundet mit {rundung_trans:g} m / {float(rundung_rot):g} rad: "
            + ", ".join(namen[i] for i in lin_rund)
            + ". Der Radius wirkt als Abstand im Kontrollpolygon des "
              "B-Splines, nicht als Kugel wie bei Pilz."))
    elif not ueber_null and "LIN" in arten:
        befunde.append(sequenz.Befund(
            sequenz.HINWEIS,
            "Kein Radius ueber null zwischen zwei LIN-Zeilen: keine "
            "kartesische Verrundung. Die Verrundung steht auf "
            f"{rundung_trans:g} m und wirkt nirgends."))
    if ptp_rund:
        befunde.append(sequenz.Befund(
            sequenz.HINWEIS,
            f"PTP ueberschliffen im Gelenkraum mit {UEBERSCHLEIFEN_RAD:g} "
            f"rad: " + ", ".join(namen[i] for i in ptp_rund)
            + ". Der Radius der Zeile wirkt dort nicht als Laenge."))
    for index in umkehr:
        befunde.append(sequenz.Befund(
            sequenz.WARNUNG,
            f"{namen[index]}: die Bahn kehrt dort im Gelenkraum um. "
            f"Angehalten wird trotz Radius {radien[index - 1]:g} m.",
            stelle=index))

    if any(befund.stufe == sequenz.FEHLER for befund in befunde):
        return None, tuple(befunde)

    if grenzen_lesen is None:
        import intrinsic_lin_auftrag
        grenzen_lesen = _grenzen_aus_datei(intrinsic_lin_auftrag)
    try:
        achsgrenzen, grenzhinweise = grenzen_lesen(params_datei, gelenke)
    except (OSError, ValueError, KeyError) as fehler:
        befunde.append(sequenz.Befund(
            sequenz.FEHLER, f"Achsgrenzen nicht lesbar: {fehler}"))
        return None, tuple(befunde)
    for hinweis in grenzhinweise:
        befunde.append(sequenz.Befund(sequenz.HINWEIS, hinweis))

    auftrag = {
        "stand": AUFTRAG_STAND,
        "wrapper": wrapper,
        "params_datei": str(params_datei),
        "gruppe": str(nutzlast.get("group") or posten[0]["group"]),
        "tip": str(tip),
        "gelenke": gelenke,
        "start": {str(k): float(v) for k, v in start.items()},
        "namen": namen,
        "wegpunkte": wegpunkte,
        "abschnitte": [[a, b] for a, b in stuecke],
        "fahrarten": [arten[a] for a, _b in stuecke],
        # Abschnittsgrenzen ohne Halt: dort werden die Nachbarabschnitte zu
        # einem Teilpfad verbunden.
        "verschleifen": list(uebergaenge),
        "ueberschleifen_rad": UEBERSCHLEIFEN_RAD,
        "umkehr": list(umkehr),
        "velocity_scaling": paare[0][0],
        "acceleration_scaling": paare[0][1],
        "resample_dt": float(resample_dt),
        "rundung_translation_m": float(rundung_trans),
        "rundung_rotation_rad": float(rundung_rot),
        "grenzen": achsgrenzen,
    }
    return auftrag, tuple(befunde)


def _grenzen_aus_datei(modul):
    """Achsgrenzen wie intrinsic_lin_auftrag: URDF und joint_limits."""
    def lesen(params_datei, gelenke):
        params = modul.parameter_lesen(params_datei)
        return modul.grenzen_lesen(params, gelenke)
    return lesen


def auftrag_text(auftrag):
    """Als JSON, wie totg_folge.auftrag_text."""
    return json.dumps(auftrag, indent=1, sort_keys=True) + "\n"


# ----------------------------------------------------------------------------
# Antwort
# ----------------------------------------------------------------------------

def antwort_lesen(text, gelenke):
    """Antwort von intrinsic_lin_rechner.py in die Form von _read_sequence.

    Das Grundgeruest ist dasselbe wie bei TOTG (punkte, teilbahnen,
    rechenzeit_s) und wird mit totg_folge.antwort_lesen gelesen -- mit
    denselben Pruefungen. Dazu kommt der Block "intrinsic" mit dem, was nur
    diese Rechnung liefert.
    """
    gelesen = totg_folge.antwort_lesen(text, gelenke)
    daten = json.loads(text)
    basis = gelesen.pop("totg")
    grenzpruefung = daten.get("grenzpruefung") or {}
    roh = grenzpruefung.get("roh") or {}
    abgetastet = grenzpruefung.get("abgetastet") or {}
    abschnitte = daten.get("abschnitte") or []
    lin = daten.get("lin") or []
    gelesen["intrinsic"] = dict(
        basis,
        intrinsic_release=str(daten.get("intrinsic_release", "")),
        wrapper_stand=daten.get("stand"),
        rechner=str(daten.get("rechner", "")),
        ik_aufrufe=daten.get("ik_aufrufe"),
        ik_rechenzeit_s=daten.get("ik_rechenzeit_s"),
        toppra_iterationen=[a.get("toppra_iterationen") for a in abschnitte],
        v_faktor_max=roh.get("v_faktor_max"),
        a_faktor_max=roh.get("a_faktor_max"),
        grenzen_roh_eingehalten=roh.get("eingehalten"),
        v_faktor_max_abgetastet=abgetastet.get("v_faktor_max"),
        a_faktor_max_abgetastet=abgetastet.get("a_faktor_max"),
        grenzen_abgetastet_eingehalten=abgetastet.get("eingehalten"),
        uebergang_max_rad=daten.get("uebergang_max_rad"),
        ik_sprung_max_rad=max([float(a.get("ik_sprung_max_dicht_rad") or 0.0)
                               for a in lin] or [0.0]),
        ik_posenfehler_max_m=max(
            [float(a.get("ik_posenfehler_max_m") or 0.0) for a in lin]
            or [0.0]),
        ik_posenfehler_max_rad=max(
            [float(a.get("ik_posenfehler_max_rad") or 0.0) for a in lin]
            or [0.0]),
        ziel_abweichung_max_rad=max(
            [float(a.get("ziel_abweichung_rad") or 0.0) for a in lin]
            or [0.0]),
        pfadpunkte=[a.get("pfadpunkte") for a in lin],
        # Ein Rechner mit Stand 1 kennt nur LIN und nennt die Fahrart nicht.
        fahrarten=[str(a.get("fahrart") or "LIN") for a in lin],
        # Wechsel der Fahrart, an denen nicht angehalten wurde.
        verschliffen=[str(v.get("punkt")) for v in
                      daten.get("verschliffen") or []],
        verschliffen_sprung_max_rad=max(
            [float(v.get("sprung_rad") or 0.0) for v in
             daten.get("verschliffen") or []] or [0.0]),
        hinweise=[str(h) for h in daten.get("hinweise") or []],
        lin_vorgaben=daten.get("lin_vorgaben") or {},
        feste_werte=daten.get("feste_werte_intrinsic") or {},
    )
    return gelesen


# ----------------------------------------------------------------------------
# CSV
# ----------------------------------------------------------------------------

# Schluessel aus herkunft, die zusaetzlich zu totg_folge.zusatzzeilen als
# Kommentarzeilen in die CSV gehen.
ZUSATZ_SCHLUESSEL = (
    "fahrarten", "verschliffen", "verschliffen_sprung_max_rad",
    "rundung_translation_m", "rundung_rotation_rad",
    "ueberschleifen_rad", "intrinsic_release",
    "wrapper_stand", "toppra_iterationen", "v_faktor_max", "a_faktor_max",
    "grenzen_roh_eingehalten", "v_faktor_max_abgetastet",
    "a_faktor_max_abgetastet", "grenzen_abgetastet_eingehalten",
    "ik_aufrufe", "ik_sprung_max_rad", "ik_posenfehler_max_m",
    "ik_posenfehler_max_rad", "ziel_abweichung_max_rad",
    "uebergang_max_rad", "ruck_begrenzt", "tcp_grenze", "kollision")


def zusatzzeilen(herkunft):
    """Kommentarzeilen fuer die CSV: die von TOTG und die von Intrinsic."""
    zeilen = totg_folge.zusatzzeilen(herkunft)
    for schluessel in ZUSATZ_SCHLUESSEL:
        if schluessel in herkunft:
            wert = herkunft[schluessel]
            if isinstance(wert, float):
                text = repr(wert)
            elif isinstance(wert, (list, tuple)):
                text = ",".join(str(w) for w in wert)
            else:
                text = str(wert)
            zeilen.append(["#" + schluessel, text])
    for hinweis in herkunft.get("intrinsic_hinweise", ()):
        zeilen.append(["#intrinsic_hinweis", str(hinweis)])
    return zeilen
