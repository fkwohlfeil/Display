#!/usr/bin/env python3
"""TOTG aus MoveIt fuer eine Achszielfolge. Hilfsprozess der Oberflaeche.

Aufruf:
    totg_rechner.py AUFTRAG.json ANTWORT.json
    totg_rechner.py --pruefen

Warum ein eigener Prozess

    TimeOptimalTrajectoryGeneration ist C++ und haengt an keinem Dienst
    von move_group. Erreichbar ist es aus Python nur ueber moveit_py, und
    moveit_py bringt einen eigenen rclcpp-Knoten mit eigenem Executor mit.
    Das gehoert nicht in den Prozess der Oberflaeche, in dem schon rclpy
    laeuft: zwei Laufzeiten mit je eigenem init und shutdown in einem
    Prozess sind eine Fehlersuche, die niemand fuehren will. Ein kurzer
    Kindprozess je Auftrag kostet ein paar Sekunden und nimmt diese Klasse
    von Fehlern heraus.

Warum dieselbe Parameterdatei

    TOTG liest die Achsgrenzen aus dem RobotModel, und das RobotModel
    liest sie aus robot_description_planning. Genommen wird deshalb die
    Datei, mit der move_group gestartet wurde -- dieselben Grenzen, dieselbe
    URDF, dieselbe SRDF. Eine einzige Aenderung ist noetig und steht in
    params_fuer_moveitpy: move_group liest planning_pipelines als Liste,
    MoveItCpp dagegen als planning_pipelines.pipeline_names. Beides in
    einer Datei geht nicht, also bekommt dieser Prozess eine Abschrift.

Was nicht passiert

    Es wird nicht gefahren, nicht geglaettet und nicht nachgebessert. Die
    Antwort ist die Ausgabe von applyTOTGTimeParameterization, abgetastet
    im Takt resample_dt, wie MoveIt sie liefert. Scheitert TOTG, steht der
    Grund in der Ausgabe dieses Prozesses; die Oberflaeche zeigt deren
    letzte Zeilen.

Die Antwort wird erst in eine Nebendatei geschrieben und dann umbenannt.
Eine halb geschriebene Antwort kann die Oberflaeche damit nicht lesen.
"""

import json
import os
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

KNOTEN_PRAEFIX = "urdf_gui_totg_"

# Stand dieses Rechners. Ein Auftrag mit hoeherem Stand traegt Felder, die
# hier nicht verstanden wuerden -- er wird abgelehnt, nicht halb gerechnet.
STAND = 2


def params_fuer_moveitpy(text):
    """Parameterdatei von move_group in eine fuer MoveItPy umschreiben.

    Aus planning_pipelines: [a, b] wird planning_pipelines.pipeline_names.
    Alles andere bleibt, wie es ist. Der Wurzelschluessel /** gilt fuer
    jeden Knoten und damit auch fuer diesen.
    """
    import yaml

    daten = yaml.safe_load(text)
    if not isinstance(daten, dict) or not daten:
        raise ValueError("Parameterdatei ist leer oder kein Objekt")
    for knoten in daten.values():
        if not isinstance(knoten, dict):
            continue
        werte = knoten.get("ros__parameters")
        if not isinstance(werte, dict):
            continue
        pipelines = werte.get("planning_pipelines")
        if isinstance(pipelines, list):
            werte["planning_pipelines"] = {"pipeline_names": list(pipelines)}
        # Der Szenendienst gehoert move_group. Ein zweiter Anbieter unter
        # demselben Namen waere fuer RViz nicht zu unterscheiden.
        werte.setdefault("planning_scene_monitor_options", {})
        werte["planning_scene_monitor_options"].setdefault(
            "name", "planning_scene_monitor_totg")

    class Schreiber(yaml.SafeDumper):
        """URDF und SRDF als Literalblock, wie in der Vorlage."""

    def text_darstellen(dumper, wert):
        stil = "|" if "\n" in wert else None
        return dumper.represent_scalar("tag:yaml.org,2002:str", wert, style=stil)

    Schreiber.add_representer(str, text_darstellen)
    return yaml.dump(daten, Dumper=Schreiber, default_flow_style=False,
                     sort_keys=False, allow_unicode=True, width=4096)


def verketten(teile):
    """Abschnitte zu einer Bahn aneinanderhaengen.

    Jeder Abschnitt beginnt bei t = 0 am Ende des vorigen. Sein erster
    Stuetzpunkt ist derselbe Ort im selben Stillstand wie der letzte des
    vorigen und faellt deshalb weg -- dieselbe Regel wie bei den
    Teilbahnen von Pilz in RosBridge._read_sequence. Liefert (punkte,
    teilbahnen) mit der Zahl der uebernommenen Punkte je Abschnitt.
    """
    punkte = []
    teilbahnen = []
    versatz = 0.0
    for teil in teile:
        roh = list(teil)
        if punkte and roh:
            roh = roh[1:]
        for zeit, lagen, tempi in roh:
            punkte.append([float(zeit) + versatz, list(lagen), list(tempi)])
        teilbahnen.append(len(roh))
        if punkte:
            versatz = punkte[-1][0]
    return punkte, teilbahnen


def moveit_version():
    """Fassung und Herkunft von moveit_core, fuer die Protokollzeile."""
    try:
        from ament_index_python.packages import get_package_share_directory
        pfad = get_package_share_directory("moveit_core")
        wurzel = ET.parse(os.path.join(pfad, "package.xml")).getroot()
        fassung = (wurzel.findtext("version") or "").strip()
        return fassung, pfad
    except Exception as fehler:                        # nur Auskunft
        return f"unbekannt ({fehler})", ""


def schreibe_antwort(pfad, daten):
    ordner = os.path.dirname(os.path.abspath(pfad))
    griff, zwischen = tempfile.mkstemp(prefix=".antwort_", dir=ordner)
    with os.fdopen(griff, "w", encoding="utf-8") as datei:
        json.dump(daten, datei)
        datei.write("\n")
    os.replace(zwischen, pfad)


def rechne(auftrag, arbeitsordner):
    """Die eigentliche Rechnung. Gibt das Antwortwoerterbuch zurueck."""
    if int(auftrag.get("stand", 1)) > STAND:
        raise RuntimeError(f"Auftrag hat Stand {auftrag.get('stand')}, "
                           f"totg_rechner.py kennt nur {STAND}. Datei "
                           f"neben der GUI erneuern.")
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    from moveit.core.robot_trajectory import RobotTrajectory
    from moveit_msgs.msg import RobotTrajectory as BahnNachricht
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    with open(auftrag["params_datei"], encoding="utf-8") as datei:
        abschrift = params_fuer_moveitpy(datei.read())
    params = os.path.join(arbeitsordner, "totg_params.yaml")
    with open(params, "w", encoding="utf-8") as datei:
        datei.write(abschrift)

    beginn = time.monotonic()
    moveit = MoveItPy(node_name=f"{KNOTEN_PRAEFIX}{os.getpid()}",
                      launch_params_filepaths=[params],
                      provide_planning_service=False)
    start_s = time.monotonic() - beginn

    modell = moveit.get_robot_model()
    gruppe = auftrag["gruppe"]
    if not modell.has_joint_model_group(gruppe):
        raise RuntimeError(f"Gruppe {gruppe} gibt es im Modell nicht")
    gelenke = list(auftrag["gelenke"])
    aktiv = list(modell.get_joint_model_group(gruppe).active_joint_model_names)
    if sorted(aktiv) != sorted(gelenke):
        raise RuntimeError(f"Die Gruppe fuehrt {aktiv}, der Auftrag {gelenke}")

    wegpunkte = auftrag["wegpunkte"]
    stuecke = auftrag.get("abschnitte") or [[0, len(wegpunkte) - 1]]
    namen = auftrag.get("namen") or [str(i) for i in range(len(wegpunkte))]
    teile = []
    rechenzeit = 0.0
    for anfang, ende in stuecke:
        # Jeder Abschnitt beginnt im Stillstand an seinem ersten Wegpunkt.
        # Die uebrigen Gelenke (gekoppelte) stehen wie beim Start; TOTG
        # rechnet ohnehin nur mit den aktiven der Gruppe.
        werte = {k: float(v) for k, v in auftrag["start"].items()}
        werte.update(zip(gelenke, (float(w) for w in wegpunkte[anfang])))
        stand = RobotState(modell)
        stand.joint_positions = werte
        stand.update()

        nachricht = BahnNachricht()
        nachricht.joint_trajectory.joint_names = gelenke
        for nummer, lage in enumerate(wegpunkte[anfang:ende + 1]):
            punkt = JointTrajectoryPoint()
            punkt.positions = [float(w) for w in lage]
            # Platzhalter. TOTG liest nur die Lagen und setzt die Zeit neu.
            punkt.time_from_start = Duration(sec=nummer, nanosec=0)
            nachricht.joint_trajectory.points.append(punkt)

        bahn = RobotTrajectory(modell)
        bahn.set_robot_trajectory_msg(stand, nachricht)
        bahn.joint_model_group_name = gruppe

        rechnung = time.monotonic()
        ok = bahn.apply_totg_time_parameterization(
            float(auftrag["velocity_scaling"]),
            float(auftrag["acceleration_scaling"]),
            path_tolerance=float(auftrag["path_tolerance"]),
            resample_dt=float(auftrag["resample_dt"]),
            min_angle_change=float(auftrag["min_angle_change"]))
        rechenzeit += time.monotonic() - rechnung
        if not ok:
            raise RuntimeError(
                f"applyTOTGTimeParameterization lieferte false im Abschnitt "
                f"{namen[anfang]} -> {namen[ende]}. Grund in den Zeilen "
                f"darueber.")

        antwort = bahn.get_robot_trajectory_msg(joint_filter=gelenke)
        folge = antwort.joint_trajectory
        reihe = [list(folge.joint_names).index(name) for name in gelenke]
        teil = []
        for punkt in folge.points:
            zeit = (punkt.time_from_start.sec
                    + punkt.time_from_start.nanosec * 1e-9)
            lagen = [float(punkt.positions[i]) for i in reihe]
            tempi = ([float(punkt.velocities[i]) for i in reihe]
                     if len(punkt.velocities) == len(folge.joint_names)
                     else [0.0] * len(gelenke))
            teil.append([zeit, lagen, tempi])
        teile.append(teil)

    punkte, teilbahnen = verketten(teile)
    fassung, pfad = moveit_version()
    return {
        "ok": True,
        "gelenke": gelenke,
        "punkte": punkte,
        "dauer_s": float(punkte[-1][0]) if punkte else 0.0,
        "teilbahnen": teilbahnen,
        "eingang": len(auftrag["wegpunkte"]),
        "rechenzeit_s": rechenzeit,
        "start_s": start_s,
        "moveit_version": fassung,
        "moveit_pfad": pfad,
    }


def pruefen():
    """Nur nachsehen, ob moveit_py und yaml da sind. Kein Knoten."""
    fehlend = []
    for modul in ("yaml", "moveit.planning", "moveit.core.robot_trajectory",
                  "moveit_msgs.msg", "trajectory_msgs.msg"):
        try:
            __import__(modul)
        except ImportError as fehler:
            fehlend.append(f"{modul} ({fehler})")
    fassung, pfad = moveit_version()
    print(f"moveit_core {fassung} {pfad}")
    if fehlend:
        print("fehlt: " + "; ".join(fehlend))
        print("moveit_py: sudo apt install ros-jazzy-moveit-py, oder im "
              "Workspace ~/ws_moveit mitbauen und neu sourcen.")
        return 1
    print("ok")
    return 0


def main(argv):
    if len(argv) == 2 and argv[1] == "--pruefen":
        return pruefen()
    if len(argv) != 3:
        print(__doc__)
        return 2
    auftrag_pfad, antwort_pfad = argv[1], argv[2]
    try:
        with open(auftrag_pfad, encoding="utf-8") as datei:
            auftrag = json.load(datei)
        daten = rechne(auftrag, os.path.dirname(os.path.abspath(antwort_pfad)))
    except Exception as fehler:
        import traceback
        traceback.print_exc()
        daten = {"ok": False, "fehler": f"{type(fehler).__name__}: {fehler}"}
    schreibe_antwort(antwort_pfad, daten)
    sys.stdout.flush()
    sys.stderr.flush()
    # MoveItPy haengt seinen Executor-Thread ab. Ein normales Ende kann an
    # ihm haengen bleiben; die Antwort liegt zu diesem Zeitpunkt schon da.
    os._exit(0 if daten.get("ok") else 1)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
