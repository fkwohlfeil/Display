#!/usr/bin/env python3
"""LIN-Bahn nach Intrinsic Core, mit MoveIt als Rueckwaertskinematik.

Hilfsprozess der Oberflaeche, Gegenstueck zu totg_rechner.py.

Aufruf:
    intrinsic_lin_rechner.py AUFTRAG.json ANTWORT.json
    intrinsic_lin_rechner.py --pruefen

Was gerechnet wird

    Nachgebaut ist der LIN-Planer von Intrinsic Core
    (LinearCartesianMotionPathPlanner und PathIkOnCartesianCurveAndResample-
    ForUniformJointSampling), mit einem Unterschied: die Rueckwaertskinematik
    kommt aus MoveIt, nicht aus Intrinsic. Je LIN-Abschnitt:

      1. MoveIt: Vorwaertskinematik der Wegpunkte -> Posen des Tip-Links
      2. Wrapper lin_kurve: verrundete kartesische Kurve (Intrinsic),
         gleichmaessig abgetastet
      3. MoveIt: IK an allen Kurvenposen, Startwert jeweils die vorige Loesung
      4. Wrapper lin_neuabtastung: Kurvenparameter, die im Gelenkraum etwa
         gleich weit auseinanderliegen (Intrinsic), zehnfach verdichtet
      5. MoveIt: IK an den dichten Posen; behalten werden die Indizes, die
         Intrinsic nennt
    Ein PTP-Abschnitt (Auftrag ab Stand 2, Feld "fahrarten") faehrt gerade
    im Gelenkraum. Seine Wegpunkte gehen unveraendert als Teilpfad weiter,
    ohne Kinematik und ohne IK-Kette. Sind alle Abschnitte PTP, faehrt
    MoveItPy gar nicht erst hoch. An Wechseln der Fahrart ohne Halt (ab
    Stand 4, Feld "verschleifen") werden die Gelenkpfade der Nachbar-
    abschnitte zu einem Teilpfad verbunden. Danach fuer alle Abschnitte
    zusammen

      6. Wrapper toppra: die Gelenkpfade als teilpfade, TOPP-RA, an den
         inneren Punkten mit "ueberschleifen_rad" ueberschliffen

Herkunft

    Selbst geschrieben. Aus Intrinsic Core kommt nur, was der Wrapper aufruft
    (siehe dessen Kopfkommentar). Aus MoveIt kommen Vorwaerts- und
    Rueckwaertskinematik, ueber moveit_py mit derselben Parameterdatei wie
    move_group -- dieselbe URDF, SRDF und kinematics.yaml. Mit [NACHGEBILDET]
    markiert sind Pruefungen, die der LIN-Planer von Intrinsic ebenso macht.

Was nicht passiert

    Keine Kollisionspruefung (der LIN-Planer von Intrinsic prueft danach auf
    Kollision, hier nicht). Keine kartesischen Grenzen in TOPP-RA -- dafuer
    braucht Intrinsic ein eigenes Kinematikmodell, das es hier nicht gibt.
    Der Startwert der IK ist die vorige Loesung; Intrinsic extrapoliert ihn
    zusaetzlich mit maximaler Verzoegerung (FinePathIk). Nichts wird still
    korrigiert: Abweichungen stehen in der Antwort unter "lin" und
    "hinweise".
"""

import json
import math
import os
import subprocess
import sys
import tempfile
import time

import totg_rechner

KNOTEN_PRAEFIX = "urdf_gui_intrinsic_lin_"

# Stand dieses Rechners. Ein Auftrag mit hoeherem Stand wird abgelehnt.
# Seit 2: "fahrarten" je Abschnitt (LIN, PTP).
# Seit 3: gekoppelte (mimic) Gelenke nicht mehr aus der Grundstellung
#    ueberschreiben -- verfaelschte die FK der Wegpunkte.
# Seit 4: "verschleifen" -- Wechsel der Fahrart ohne Halt.
STAND = 4

FAHRARTEN = ("LIN", "PTP")

# Stand des Wrappers, den dieser Rechner mindestens braucht (Modi lin_*).
WRAPPER_STAND = 2

# [NACHGEBILDET] path_ik.cc: groesster erlaubter Sprung einer Achse zwischen
# zwei aufeinanderfolgenden IK-Loesungen (kMaxJointLimitStep = pi/4).
MAX_IK_SPRUNG_RAD = math.pi / 4.0

# [NACHGEBILDET] path_ik.cc: die erste IK-Loesung muss dem Startwert
# entsprechen (kJointDeviationToleranceRad, L-unendlich-Norm).
START_TOLERANZ_RAD = 1e-3

# [NACHGEBILDET] linear_cartesian_motion_path_planner.cc: die letzte
# IK-Loesung muss den Zielwegpunkt treffen (quadrierte Norm <= 1e-3**2).
ZIEL_TOLERANZ_RAD = 1e-3

# Eigene Pruefung, nicht aus Intrinsic: so weit darf die Vorwaerts-
# kinematik einer IK-Loesung neben ihrer Kurvenpose liegen. MoveIt-KDL
# meldet nahe einer Singularitaet Erfolg, obwohl die Loesung die Pose um
# Millimeter verfehlt; ueber die IK-Kette waechst das dann weiter. Im
# Auftrag mit ik_toleranz_m und ik_toleranz_rad zu aendern.
IK_TOLERANZ_M = 1e-3
IK_TOLERANZ_RAD = 1e-2

# Ein Singulaerwert der Jacobi-Matrix unter diesem Anteil des groessten
# zaehlt als null. Nur fuer die Fehlermeldung bei einem Achssprung.
SINGULAER_RELATIV = 1e-6

# Standardwerte wie im Motion Planner von Intrinsic (motion_planner.cc).
STANDARD = {
    "abtastung_translation_m": 0.002,   # kDefaultTranslationalSamplingStepM
    "abtastung_rotation_rad": 0.005,    # kDefaultRotationalSamplingStepRad
    "gelenk_abtastung_rad": 0.05,       # kDefaultJointSpaceUniform...Rad
    "ueberschleifen_rad": 0.2,          # kCartMotionLowLevelJointBlending...
}


class LinFehler(RuntimeError):
    """Abbruch mit einer Meldung, die in die Antwort geht."""


# ==========================================================================
# Wrapper-Aufruf
# ==========================================================================

def wrapper_rufen(wrapper, auftrag, ordner, name):
    """Den C++-Wrapper mit einem Auftrag starten und seine Antwort lesen."""
    pfad_auftrag = os.path.join(ordner, f"{name}_auftrag.json")
    pfad_antwort = os.path.join(ordner, f"{name}_antwort.json")
    with open(pfad_auftrag, "w", encoding="utf-8") as datei:
        json.dump(auftrag, datei)
    lauf = subprocess.run([wrapper, pfad_auftrag, pfad_antwort],
                          capture_output=True, text=True)
    try:
        with open(pfad_antwort, encoding="utf-8") as datei:
            antwort = json.load(datei)
    except (OSError, ValueError) as fehler:
        raise LinFehler(f"Wrapper ({name}) ohne lesbare Antwort, Code "
                        f"{lauf.returncode}: {fehler}. Ausgabe: "
                        f"{lauf.stderr.strip()[-400:]}")
    if not antwort.get("ok"):
        raise LinFehler(f"Wrapper ({name}): {antwort.get('fehler')}")
    return antwort


# ==========================================================================
# Kinematik ueber MoveIt
# ==========================================================================

class MoveItKinematik:
    """Vorwaerts- und Rueckwaertskinematik aus moveit_py. Nichts Eigenes."""

    def __init__(self, auftrag, arbeitsordner):
        from moveit.planning import MoveItPy
        from moveit.core.robot_state import RobotState

        with open(auftrag["params_datei"], encoding="utf-8") as datei:
            abschrift = totg_rechner.params_fuer_moveitpy(datei.read())
        params = os.path.join(arbeitsordner, "lin_params.yaml")
        with open(params, "w", encoding="utf-8") as datei:
            datei.write(abschrift)

        beginn = time.monotonic()
        self.moveit = MoveItPy(node_name=f"{KNOTEN_PRAEFIX}{os.getpid()}",
                               launch_params_filepaths=[params],
                               provide_planning_service=False)
        self.start_s = time.monotonic() - beginn

        self.modell = self.moveit.get_robot_model()
        self.gruppe = auftrag["gruppe"]
        self.tip = auftrag["tip"]
        self.gelenke = list(auftrag["gelenke"])
        if not self.modell.has_joint_model_group(self.gruppe):
            raise LinFehler(f"Gruppe {self.gruppe} gibt es im Modell nicht")
        aktiv = list(self.modell.get_joint_model_group(
            self.gruppe).active_joint_model_names)
        if sorted(aktiv) != sorted(self.gelenke):
            raise LinFehler(f"Die Gruppe fuehrt {aktiv}, der Auftrag "
                            f"{self.gelenke}")
        # [STAND 3] Mimic-Gelenke nicht aus der Grundstellung setzen
        # Alle Gelenke der Gruppe, die nicht aktiv sind: gekoppelte
        # (mimic) und feste. Ihr Wert folgt in MoveIt dem Leitgelenk.
        self.abgeleitet = set(self.modell.get_joint_model_group(
            self.gruppe).joint_model_names) - set(aktiv)
        self.grundstellung = {k: float(v) for k, v in auftrag["start"].items()
                              if k not in self.abgeleitet}
        self.ik_timeout = float(auftrag.get("ik_timeout_s", 0.05))
        self.zustand = RobotState(self.modell)
        self.ik_rechenzeit = 0.0
        self.ik_aufrufe = 0

    def _setzen(self, lagen):
        # Ohne Mimic-Gelenke: moveit_py setzt die Eintraege einzeln in
        # Namensreihenfolge und zieht bei jedem Leitgelenk dessen
        # Mimic-Gelenke nach. Ein Mimic-Eintrag hier wuerde das -- je nach
        # Name -- mit dem Startwert wieder ueberschreiben.
        werte = dict(self.grundstellung)
        werte.update(zip(self.gelenke, (float(w) for w in lagen)))
        self.zustand.joint_positions = werte
        self.zustand.update()

    def _lagen(self):
        werte = self.zustand.joint_positions
        return [float(werte[name]) for name in self.gelenke]

    def fk(self, lagen):
        """Pose des Tip-Links als [x, y, z, qx, qy, qz, qw]."""
        self._setzen(lagen)
        p = self.zustand.get_pose(self.tip)
        return [p.position.x, p.position.y, p.position.z,
                p.orientation.x, p.orientation.y, p.orientation.z,
                p.orientation.w]

    def startwert(self, lagen):
        """Startwert der naechsten IK auf diese Gelenklagen setzen."""
        self._setzen(lagen)

    def rang(self, lagen):
        """Rang der Jacobi-Matrix in dieser Stellung und sein Hoechstwert.

        Nur fuer die Fehlermeldung. Setzt den Zustand um -- danach ist ein
        neuer Startwert noetig.
        """
        werte = self.singulaerwerte(lagen)
        grenze = SINGULAER_RELATIV * max(werte) if werte else 0.0
        return sum(1 for w in werte if w > grenze), len(werte)

    def singulaerwerte(self, lagen):
        """Singulaerwerte der Jacobi-Matrix in dieser Stellung, absteigend.

        Nur fuer Fehlermeldungen. Setzt den Zustand um.
        """
        import numpy
        self._setzen(lagen)
        jacobi = numpy.asarray(self.zustand.get_jacobian(
            self.gruppe, numpy.zeros(3)))
        return [float(w) for w in numpy.linalg.svd(jacobi, compute_uv=False)]

    def ik(self, pose):
        """IK zur Pose, Startwert ist der aktuelle Zustand. None bei Fehler."""
        from geometry_msgs.msg import Pose
        ziel = Pose()
        (ziel.position.x, ziel.position.y, ziel.position.z,
         ziel.orientation.x, ziel.orientation.y, ziel.orientation.z,
         ziel.orientation.w) = (float(w) for w in pose)
        beginn = time.monotonic()
        ok = self.zustand.set_from_ik(self.gruppe, ziel, self.tip,
                                      self.ik_timeout)
        self.ik_rechenzeit += time.monotonic() - beginn
        self.ik_aufrufe += 1
        if not ok:
            return None
        self.zustand.update()
        return self._lagen()


# ==========================================================================
# Die Kette je Abschnitt
# ==========================================================================

def max_abstand(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def pose_fehler(ist, soll):
    """Abstand zweier Posen [x, y, z, qx, qy, qz, qw]: (Meter, Winkel rad)."""
    lage = math.sqrt(sum((a - b) ** 2 for a, b in zip(ist[:3], soll[:3])))
    betrag = (math.sqrt(sum(w * w for w in ist[3:]))
              * math.sqrt(sum(w * w for w in soll[3:])))
    punkt = abs(sum(a * b for a, b in zip(ist[3:], soll[3:]))) / betrag
    return lage, 2.0 * math.acos(min(1.0, punkt))


def ik_kette(kinematik, posen, start, name, stufe, toleranz_m=IK_TOLERANZ_M,
             toleranz_rad=IK_TOLERANZ_RAD):
    """IK an allen Posen, Startwert jeweils die vorige Loesung.

    Prueft wie FinePathIk in Intrinsic: die erste Loesung muss dem Startwert
    entsprechen, und keine Achse darf zwischen zwei Posen um pi/4 oder mehr
    springen. Dazu (eigene Pruefung) muss jede Loesung ihre Pose treffen.
    Alles fuehrt zum Abbruch, nicht zu einer Korrektur.

    Gibt (loesungen, groesster_sprung, groesster_posenfehler) zurueck; der
    Posenfehler ist (Meter, rad).
    """
    kinematik.startwert(start)
    loesungen = []
    groesster_sprung = 0.0
    fehler_m = fehler_rad = 0.0
    for i, pose in enumerate(posen):
        q = kinematik.ik(pose)
        if q is None:
            raise LinFehler(f"{name} ({stufe}): MoveIt-IK ohne Loesung an "
                            f"Kurvenpose {i} von {len(posen)}")
        # fk setzt den Zustand auf q -- das ist ohnehin der Startwert der
        # naechsten IK.
        lage, winkel = pose_fehler(kinematik.fk(q), pose)
        fehler_m, fehler_rad = max(fehler_m, lage), max(fehler_rad, winkel)
        if lage > toleranz_m or winkel > toleranz_rad:
            raise LinFehler(
                f"{name} ({stufe}): MoveIt-IK meldet Erfolg an Kurvenpose {i} "
                f"von {len(posen)}, die Loesung liegt aber {lage * 1000:.3f} "
                f"mm / {winkel:.5f} rad daneben (erlaubt {toleranz_m * 1000:g}"
                f" mm / {toleranz_rad:g} rad). "
                + kondition_text(kinematik, q))
        if i == 0:
            abweichung = max_abstand(q, start)
            if abweichung >= START_TOLERANZ_RAD:           # [NACHGEBILDET]
                raise LinFehler(
                    f"{name} ({stufe}): erste IK-Loesung weicht um "
                    f"{abweichung:.6f} rad vom Startwegpunkt ab (erlaubt < "
                    f"{START_TOLERANZ_RAD}); anderer IK-Ast?")
        else:
            sprung = max_abstand(q, loesungen[-1])
            groesster_sprung = max(groesster_sprung, sprung)
            if sprung >= MAX_IK_SPRUNG_RAD:                 # [NACHGEBILDET]
                raise LinFehler(
                    f"{name} ({stufe}): Achssprung {sprung:.4f} rad zwischen "
                    f"Kurvenpose {i - 1} und {i} (erlaubt < "
                    f"{MAX_IK_SPRUNG_RAD:.4f}) "
                    + sprung_ursache(kinematik, loesungen[-1], q))
        loesungen.append(q)
    return loesungen, groesster_sprung, (fehler_m, fehler_rad)


def kondition_text(kinematik, lagen):
    """Wie nah die Stellung an einer Singularitaet liegt. Nur Auskunft."""
    try:
        werte = kinematik.singulaerwerte(lagen)
    except Exception:                                   # nur Auskunft
        return "Naehe zu Singularitaet oder Gelenkgrenze?"
    return (f"Kleinster Singulaerwert der Jacobi-Matrix dort "
            f"{min(werte):.4f} (groesster {max(werte):.4f}): nahe einer "
            f"Singularitaet ist die IK schlecht konditioniert und bricht "
            f"ungenau ab. Punkte weiter weg von der Singularitaet legen oder "
            f"diese Strecke mit PTP fahren.")


def sprung_ursache(kinematik, vorher, nachher):
    """Welche Achsen springen, und ist die Stellung davor singulaer?"""
    namen = getattr(kinematik, "gelenke", None) or [
        str(k) for k in range(len(vorher))]
    achsen = ", ".join(
        f"{n} {b - a:+.4f}" for n, a, b in zip(namen, vorher, nachher)
        if abs(b - a) >= MAX_IK_SPRUNG_RAD / 10.0)
    text = f"an {achsen}."
    try:
        rang, voll = kinematik.rang(vorher)
    except Exception:                                   # nur Auskunft
        return text + " Naehe zu Singularitaet oder Gelenkgrenze."
    if rang < voll:
        return (text + f" Die Stellung davor ist singulaer (Jacobi-Rang "
                f"{rang} von {voll}): dort legt die Pose die Achsen nicht "
                f"eindeutig fest, und die IK springt auf einen anderen Ast. "
                f"LIN kann nicht durch oder aus einer Singularitaet fahren "
                f"-- mit PTP herausfahren oder den Punkt verlegen.")
    return text + (f" Die Stellung davor ist nicht singulaer (Jacobi-Rang "
                   f"{rang} von {voll}); Naehe zu Singularitaet oder "
                   f"Gelenkgrenze?")


def lin_abschnitt(auftrag, kinematik, wrapper, ordner, wegpunkte, name,
                  nummer):
    """Ein Abschnitt als dichter Gelenkpfad. Gibt (pfad, info) zurueck."""
    posen = [kinematik.fk(q) for q in wegpunkte]
    kurve_def = {
        "stand": WRAPPER_STAND,
        "posen": posen,
        "rundung_translation_m": float(auftrag["rundung_translation_m"]),
        "rundung_rotation_rad": float(auftrag["rundung_rotation_rad"]),
        "abtastung_translation_m": float(auftrag.get(
            "abtastung_translation_m", STANDARD["abtastung_translation_m"])),
        "abtastung_rotation_rad": float(auftrag.get(
            "abtastung_rotation_rad", STANDARD["abtastung_rotation_rad"])),
    }

    # 2. verrundete Kurve (Intrinsic)
    kurve = wrapper_rufen(wrapper, dict(kurve_def, modus="lin_kurve"),
                          ordner, f"a{nummer}_lin_kurve")

    toleranz = (float(auftrag.get("ik_toleranz_m", IK_TOLERANZ_M)),
                float(auftrag.get("ik_toleranz_rad", IK_TOLERANZ_RAD)))

    # 3. IK an allen Kurvenposen (MoveIt)
    grob, sprung_grob, _ = ik_kette(kinematik, kurve["posen"], wegpunkte[0],
                                    name, "grob", *toleranz)

    # 4. gleichabstaendig im Gelenkraum, verdichtet (Intrinsic)
    neu = wrapper_rufen(wrapper, dict(
        kurve_def, modus="lin_neuabtastung",
        kurvenparameter=kurve["kurvenparameter"], gelenkpfad=grob,
        gelenk_abtastung_rad=float(auftrag.get(
            "gelenk_abtastung_rad", STANDARD["gelenk_abtastung_rad"]))),
        ordner, f"a{nummer}_lin_neuabtastung")

    # 5. IK an den dichten Posen (MoveIt), dann die behaltenen Indizes
    dicht, sprung_dicht, posenfehler = ik_kette(
        kinematik, neu["posen"], wegpunkte[0], name, "dicht", *toleranz)
    try:
        pfad = [dicht[i] for i in neu["behalten_indizes"]]
    except IndexError:
        raise LinFehler(f"{name}: behalten_indizes ausserhalb der "
                        f"{len(dicht)} dichten Loesungen")

    # [NACHGEBILDET] Der Pfad muss den Zielwegpunkt treffen. Weicht er ab,
    # hat die IK-Kette einen anderen Ast genommen -- Intrinsic bricht dann
    # ebenso ab.
    ziel_abweichung = math.sqrt(sum(
        (a - b) ** 2 for a, b in zip(pfad[-1], wegpunkte[-1])))
    if ziel_abweichung > ZIEL_TOLERANZ_RAD:
        # Gleiche Pose mit anderen Achsen ist ein anderer Ast. Liegt schon
        # die Pose daneben, war die IK ungenau -- das ist etwas anderes.
        lage, winkel = pose_fehler(kinematik.fk(pfad[-1]), posen[-1])
        if lage <= 1e-5 and winkel <= 1e-4:
            ursache = ("Die Pose stimmt, die Achsen nicht: anderer IK-Ast "
                       "-- Ziel als diese Stellung angeben oder Wegpunkte "
                       "aendern.")
        else:
            ursache = (f"Die Endpose liegt {lage * 1000:.3f} mm / "
                       f"{winkel:.5f} rad neben dem Ziel: kein anderer Ast, "
                       f"sondern eine ungenaue IK. "
                       + kondition_text(kinematik, wegpunkte[-1]))
        raise LinFehler(
            f"{name}: LIN-Pfad endet {ziel_abweichung:.6f} rad (Norm) neben "
            f"dem Zielwegpunkt (erlaubt {ZIEL_TOLERANZ_RAD}). Erreicht wurde "
            f"{[round(w, 6) for w in pfad[-1]]}. " + ursache)
    start_abweichung = max_abstand(pfad[0], wegpunkte[0])

    info = {
        "name": name,
        "fahrart": "LIN",
        "wegpunkte": len(wegpunkte),
        "kurve": kurve["kurve"],
        "kurvenposen_grob": len(kurve["posen"]),
        "gelenk_abtastschritt_rad": neu["gelenk_abtastschritt_rad"],
        "kurvenposen_dicht": len(neu["posen"]),
        "pfadpunkte": len(pfad),
        "ik_sprung_max_grob_rad": sprung_grob,
        "ik_sprung_max_dicht_rad": sprung_dicht,
        # Groesste Abweichung FK(IK-Loesung) zur dichten Kurvenpose.
        "ik_posenfehler_max_m": posenfehler[0],
        "ik_posenfehler_max_rad": posenfehler[1],
        "start_abweichung_rad": start_abweichung,
        "ziel_abweichung_rad": ziel_abweichung,
        "hinweise": list(kurve.get("hinweise", [])),
    }
    return pfad, info


def ptp_abschnitt(wegpunkte, name):
    """PTP: die Wegpunkte selbst sind der Gelenkpfad. Keine IK.

    Gerade Strecken im Gelenkraum zwischen den Achszielen; das
    Ueberschleifen an den inneren Punkten macht der Wrapper in TOPP-RA.
    """
    pfad = [list(q) for q in wegpunkte]
    info = {
        "name": name,
        "fahrart": "PTP",
        "wegpunkte": len(wegpunkte),
        "pfadpunkte": len(pfad),
        "hinweise": [],
    }
    return pfad, info


def verbinden(stuecke, teilpfade, teilnamen, verschleifen, namen):
    """Abschnitte an den Punkten in `verschleifen` zu einem Teilpfad fuegen.

    Der erste Punkt des hinteren Abschnitts faellt weg; er ist derselbe
    Wegpunkt wie der letzte des vorderen. Bei LIN treffen beide ihn nur bis
    auf die Zieltoleranz -- wie weit sie auseinanderliegen, steht je Punkt
    in der Rueckgabe. Gibt (teilpfade, teilnamen, verbunden) zurueck;
    verbunden ist [{"punkt", "sprung_rad"}, ...].
    """
    innere = {ende for _anfang, ende in stuecke[:-1]}
    for index in verschleifen:
        if index not in innere:
            raise LinFehler(f"verschleifen: Punkt {index} ist keine innere "
                            f"Abschnittsgrenze")
    offen = set(verschleifen)
    pfade, namen_neu, verbunden = [], [], []
    for (anfang, ende), pfad, name in zip(stuecke, teilpfade, teilnamen):
        if pfade and anfang in offen:
            sprung = max_abstand(pfade[-1][-1], pfad[0])
            pfade[-1] = pfade[-1] + [list(q) for q in pfad[1:]]
            namen_neu[-1] = f"{namen_neu[-1]} -> {namen[ende]}"
            verbunden.append({"punkt": namen[anfang], "sprung_rad": sprung})
        else:
            pfade.append([list(q) for q in pfad])
            namen_neu.append(name)
    return pfade, namen_neu, verbunden


def rechne(auftrag, arbeitsordner, kinematik=None):
    """Die ganze Rechnung. Gibt das Antwortwoerterbuch zurueck.

    `kinematik` laesst sich fuer Tests ersetzen; sonst MoveIt.
    """
    if int(auftrag.get("stand", 1)) > STAND:
        raise LinFehler(f"Auftrag hat Stand {auftrag.get('stand')}, "
                        f"intrinsic_lin_rechner.py kennt nur {STAND}")
    wrapper = auftrag["wrapper"]
    if not (os.path.isfile(wrapper) and os.access(wrapper, os.X_OK)):
        raise LinFehler(f"Wrapper nicht ausfuehrbar: {wrapper}")
    for schluessel in ("rundung_translation_m", "rundung_rotation_rad",
                       "velocity_scaling", "acceleration_scaling",
                       "resample_dt", "grenzen", "gelenke", "wegpunkte"):
        if schluessel not in auftrag:
            raise LinFehler(f"Auftrag ohne {schluessel}")

    wegpunkte = [[float(w) for w in q] for q in auftrag["wegpunkte"]]
    namen = auftrag.get("namen") or [str(i) for i in range(len(wegpunkte))]
    if len(namen) != len(wegpunkte):
        namen = [str(i) for i in range(len(wegpunkte))]
    stuecke = auftrag.get("abschnitte") or [[0, len(wegpunkte) - 1]]
    # Ein Auftrag mit Stand 1 kennt nur LIN.
    arten = auftrag.get("fahrarten") or ["LIN"] * len(stuecke)
    if len(arten) != len(stuecke):
        raise LinFehler(f"{len(arten)} Fahrarten fuer {len(stuecke)} "
                        f"Abschnitte")
    for art in arten:
        if art not in FAHRARTEN:
            raise LinFehler(f"Fahrart {art!r} unbekannt, nur "
                            f"{', '.join(FAHRARTEN)}")

    # MoveItPy nur, wenn eine IK-Kette laeuft -- PTP braucht keine.
    if kinematik is None and "LIN" in arten:
        kinematik = MoveItKinematik(auftrag, arbeitsordner)

    ordner = tempfile.mkdtemp(prefix="lin_", dir=arbeitsordner)
    teilpfade, teilnamen, lin_info = [], [], []
    for nummer, ((anfang, ende), art) in enumerate(zip(stuecke, arten)):
        if not (0 <= anfang < ende < len(wegpunkte)):
            raise LinFehler(f"Abschnitt [{anfang}, {ende}] ist ungueltig")
        name = f"{namen[anfang]} -> {namen[ende]}"
        if art == "PTP":
            pfad, info = ptp_abschnitt(wegpunkte[anfang:ende + 1], name)
        else:
            pfad, info = lin_abschnitt(auftrag, kinematik, wrapper, ordner,
                                       wegpunkte[anfang:ende + 1], name,
                                       nummer)
        teilpfade.append(pfad)
        teilnamen.append(name)
        lin_info.append(info)

    # Ab Stand 4: Wechsel der Fahrart ohne Halt. Die Nachbarabschnitte gehen
    # als ein Teilpfad an TOPP-RA; der Punkt dazwischen ist dann ein innerer
    # Punkt und wird im Gelenkraum mit ueberschleifen_rad ueberschliffen.
    teilpfade, teilnamen, verbunden = verbinden(
        stuecke, teilpfade, teilnamen, auftrag.get("verschleifen") or [],
        namen)

    # 6. TOPP-RA ueber alle Abschnitte (Intrinsic)
    antwort = wrapper_rufen(wrapper, {
        "stand": WRAPPER_STAND,
        "modus": "toppra",
        "gelenke": list(auftrag["gelenke"]),
        "teilpfade": teilpfade,
        "namen": teilnamen,
        "ueberschleifen_rad": float(auftrag.get(
            "ueberschleifen_rad", STANDARD["ueberschleifen_rad"])),
        "velocity_scaling": float(auftrag["velocity_scaling"]),
        "acceleration_scaling": float(auftrag["acceleration_scaling"]),
        "resample_dt": float(auftrag["resample_dt"]),
        "grenzen": auftrag["grenzen"],
        "roh_ausgeben": bool(auftrag.get("roh_ausgeben", False)),
    }, ordner, "toppra")

    fassung, pfad = totg_rechner.moveit_version()
    antwort["rechner"] = "intrinsic_lin_rechner"
    antwort["lin_stand"] = STAND
    antwort["fahrarten"] = list(arten)
    antwort["verschliffen"] = verbunden
    antwort["lin"] = lin_info
    antwort["lin_vorgaben"] = {
        schluessel: auftrag.get(schluessel, STANDARD.get(schluessel))
        for schluessel in ("rundung_translation_m", "rundung_rotation_rad",
                           "abtastung_translation_m", "abtastung_rotation_rad",
                           "gelenk_abtastung_rad", "ueberschleifen_rad")}
    antwort["lin_vorgaben"]["ik_toleranz_m"] = float(auftrag.get(
        "ik_toleranz_m", IK_TOLERANZ_M))
    antwort["lin_vorgaben"]["ik_toleranz_rad"] = float(auftrag.get(
        "ik_toleranz_rad", IK_TOLERANZ_RAD))
    antwort["herkunft"] = dict(
        antwort.get("herkunft", {}),
        ik="MoveIt (moveit_py, set_from_ik), Startwert = vorige Loesung",
        kurve="Intrinsic Core CartesianPolylineBSplinesCurve ueber Wrapper",
        ptp="Achsziele unveraendert als Gelenkpfad, ohne IK",
        lin_rechner="eigener Code (Ablauf, IK-Kette, Pruefungen)")
    antwort["ik_aufrufe"] = getattr(kinematik, "ik_aufrufe", None)
    antwort["ik_rechenzeit_s"] = getattr(kinematik, "ik_rechenzeit", None)
    antwort["start_s"] = getattr(kinematik, "start_s", 0.0)
    antwort["moveit_version"] = fassung
    antwort["moveit_pfad"] = pfad
    antwort["arbeitsordner_wrapper"] = ordner
    for info in lin_info:
        for hinweis in info["hinweise"]:
            antwort.setdefault("hinweise", []).append(
                f"{info['name']}: {hinweis}")
    return antwort


# ==========================================================================
# Aufruf
# ==========================================================================

def pruefen(wrapper=None):
    """Nachsehen, ob moveit_py und der Wrapper da sind. Kein Knoten."""
    code = totg_rechner.pruefen()
    if wrapper:
        if not os.path.isfile(wrapper):
            print(f"Wrapper fehlt: {wrapper}")
            return 1
        lauf = subprocess.run([wrapper, "--pruefen"], capture_output=True,
                              text=True)
        print(lauf.stdout.strip())
        if lauf.returncode != 0:
            return 1
        try:
            stand = int(lauf.stdout.split("Stand")[1].split(",")[0])
        except (IndexError, ValueError):
            stand = 0
        if stand < WRAPPER_STAND:
            print(f"Wrapper hat Stand {stand}, noetig ist {WRAPPER_STAND}. "
                  f"Neu bauen.")
            return 1
    return code


def main(argv):
    if len(argv) >= 2 and argv[1] == "--pruefen":
        return pruefen(argv[2] if len(argv) > 2 else None)
    if len(argv) != 3:
        print(__doc__)
        return 2
    auftrag_pfad, antwort_pfad = argv[1], argv[2]
    try:
        with open(auftrag_pfad, encoding="utf-8") as datei:
            auftrag = json.load(datei)
        daten = rechne(auftrag,
                       os.path.dirname(os.path.abspath(antwort_pfad)))
    except Exception as fehler:
        import traceback
        traceback.print_exc()
        daten = {"ok": False, "fehler": f"{type(fehler).__name__}: {fehler}"}
    totg_rechner.schreibe_antwort(antwort_pfad, daten)
    sys.stdout.flush()
    sys.stderr.flush()
    # Wie in totg_rechner: MoveItPy kann beim normalen Ende haengen.
    os._exit(0 if daten.get("ok") else 1)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
