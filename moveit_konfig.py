"""moveit_konfig -- MoveIt-Konfiguration, SRDF-Bau und move_group als
Kindprozess.

Seit urdf_gui2_1 ein eigenes Modul. Der Inhalt ist zeilengleich aus
urdf_gui3_7.py verschoben (dort Zeilen 2615-3417); neu sind
nur dieser Kopf, die Importe und STAND.
"""

import os
import shutil
import signal
import subprocess
import tempfile
import time

from kinematik import (leaf_links, root_links)

# Muss zu MODUL_STAENDE in urdf_gui.py passen.
# 2: PLAN_FIELDS ohne goal_position_tol, max_step, jump_threshold.
STAND = 2


# ==========================================================================
# MoveIt: Konfiguration, SRDF und Kindprozess
# ==========================================================================

# Name der Planungsgruppe in der erzeugten SRDF. Er muss zu dem passen, was
# spaeter im MotionPlanRequest steht, deshalb steht er hier einmal.
MOVEIT_GROUP = "manipulator"

# Unendlich, einmal benannt. Eine Achsgrenze kann so gemeint sein.
INF = float("inf")

# Wie lange nach dem Start auf move_group gewartet wird, bevor die Anzeige
# von "startet" auf "keine Antwort" wechselt.
MOVEIT_WAIT_S = 25.0

# Frist fuer eine Planungsanfrage. Laeuft sie ab, kommt ein Fehler in die
# Warteschlange und die Ablaufsteuerung verlaesst PLANEN. Ohne diese Frist
# haengt sie dort, wenn niemand antwortet: das Future wird dann nie fertig,
# und der Rueckweg in die GUI fuehrt allein ueber dessen Callback.
# Zuschlag auf die Planungszeit des Auftrags. Die Frist ergibt sich daraus,
# damit eine hochgesetzte Planungszeit nicht gegen eine feste Grenze laeuft.
PLAN_TIMEOUT_MARGIN_S = 10.0

# Vorsilbe der Arbeitsordner. Steht hier, weil beim Start auch die Reste
# frueherer Sitzungen damit gesucht werden.
WORKDIR_PREFIX = "urdf_gui_moveit_"

# Beschleunigung und Ruck stehen nicht in der URDF. Fehlen sie, werden sie
# aus der Geschwindigkeit abgeleitet. Bewusst grosszuegig: die Werte sind
# Startwerte zum Ueberschreiben, keine Behauptung ueber die Mechanik.
ACC_FROM_VEL = 2.0
JERK_FROM_ACC = 10.0
DEFAULT_JOINT_VELOCITY = 1.0                           # rad/s bzw. m/s

# Felder der Karteikarte Achsgrenzen. Ein leeres Feld heisst: Grenze aus,
# also has_*_limits false. Das spart je Achse vier Kaestchen.
LIMIT_FIELDS = [
    ("velocity",     "v",     "rad/s",  70),
    ("acceleration", "a",     "rad/s2", 70),
    ("deceleration", "dec",   "rad/s2", 70),
    ("jerk",         "jerk",  "rad/s3", 70),
]

# Kartesische Grenzen fuer Pilz. Eine eigene Rotationsbeschleunigung gibt es
# nicht, sie wird aus dem translatorischen Verhaeltnis abgeleitet.
CARTESIAN_FIELDS = [
    ("max_trans_vel", "Bahngeschwindigkeit", "m/s",   1.000),
    ("max_trans_acc", "Bahnbeschleunigung",  "m/s2",  2.250),
    ("max_trans_dec", "Bahnverzoegerung",    "m/s2", -5.000),
    ("max_rot_vel",   "Drehgeschwindigkeit", "rad/s", 1.570),
]

# Werte des Planers selbst, nicht des Roboters. Sie landen im Namensraum der
# Pipeline und heissen am Knoten deshalb pilz_industrial_motion_planner.<key>.
#
# Die Abtastzeit ist der Takt, in dem Pilz die kartesische Bahn abtastet und je
# Stuetzpunkt eine IK rechnet. Sie war bis 3.2 nicht erreichbar: der Wert stand
# ausschliesslich als Default-Argument von TrajectoryGenerator::generate() im
# Quellcode, und der einzige Aufrufer uebergab ihn nicht. Erst der Patch
# patch_pilz_sampling_time_1.py macht daraus einen echten ROS-Parameter.
#
# Ohne diesen Patch ist das Feld wirkungslos, nicht schaedlich: move_group
# ignoriert einen unbekannten Parameter im Pipeline-Namensraum. Ob der Patch
# laeuft, sagt die Zeile "PILZ-PATCH aktiv" im move_group-Protokoll.
PLANNER_FIELDS = [
    ("sampling_time", "Abtastzeit", "s", 0.100),
]

# Grenzen des Quellcode-Patches, hier gespiegelt. Die verbindliche Pruefung
# steht in CommandPlanner::initialize() und bricht den Start von move_group ab.
# Diese Kopie dient nur der frueheren Rueckmeldung im Fenster -- weicht sie
# vom Patch ab, gewinnt der Patch.
SAMPLING_TIME_MIN = 1.0e-4
SAMPLING_TIME_MAX = 1.0

KINEMATICS_SOLVERS = (
    "kdl_kinematics_plugin/KDLKinematicsPlugin",
    "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",
    "lma_kinematics_plugin/LMAKinematicsPlugin",
)

KINEMATICS_FIELDS = [
    ("kinematics_solver_timeout",           "Zeitgrenze je Aufruf", "s", 0.050),
    ("kinematics_solver_search_resolution", "Suchschrittweite",   "rad", 0.005),
]

# Parameter des KDL-Plugins, abgeschrieben aus kdl_kinematics_parameters.hpp
# (moveit_kinematics 2.12.4, Jazzy) samt Standardwerten. Sie stehen dort
# unter robot_description_kinematics.<gruppe>, neben kinematics_solver, und
# werden nur geschrieben, wenn KDL der Solver ist -- die anderen Plugins
# kennen sie nicht.
#
# max_solver_iterations ist dort int64. Es muss als Ganzzahl in die Datei:
# "500.0" deklariert rclcpp als double, und das Plugin bricht beim Laden ab
# (derselbe Fall wie in yaml_scalar, nur andersherum).
KDL_SOLVER = KINEMATICS_SOLVERS[0]
KDL_FIELDS = [
    ("max_solver_iterations",   "Iterationen je Versuch", "",  500),
    ("epsilon",                 "Konvergenzschwelle",     "",  1e-5),
    ("orientation_vs_position", "Gewicht Drehung : Lage", "",  1.0),
]
KDL_INT_KEYS = ("max_solver_iterations",)
KDL_POSITION_ONLY_DEFAULT = False

# Werte, die pro Auftrag in den MotionPlanRequest wandern. Sie brauchen
# keinen Neustart, deshalb stehen sie getrennt von allem anderen.
#
# Seit 2.3 ohne goal_position_tol, max_step und jump_threshold. Sie wurden
# geprueft und angezeigt, gingen aber in keinen Auftrag ein: Ziele gehen
# seit 3.2 nur noch als Achswerte hinaus, und die LIN-Abtastung von Pilz
# steuert sampling_time, nicht ein Stuetzpunktabstand. Ein Feld ohne
# Wirkung laedt zu Messreihen ein, die nichts messen.
#
# goal_orientation_tol behaelt seinen Schluessel, heisst aber, was er tut:
# die Toleranz jeder JointConstraint, also eine Achs-Zieltoleranz.
PLAN_FIELDS = [
    ("velocity_scaling",     "Override Geschwindigkeit", "%",   100.0),
    ("acceleration_scaling", "Override Beschleunigung",  "%",   100.0),
    ("planning_time",        "Planungszeit",             "s",     5.0),
    ("planning_attempts",    "Versuche",                 "",      1.0),
    ("goal_orientation_tol", "Zieltoleranz Achsen",      "deg",  0.100),
    ("blend_radius",         "Ueberschleifradius",       "m",   0.000),
]

PLANNER_IDS = ("PTP", "LIN", "CIRC")

# Was davon seit 2.22 zusaetzlich im Fahrfenster steht. Bewusst nur diese
# beiden und nicht die ganze Liste: Planungszeit, Toleranzen und
# Stuetzpunktabstand aendert man einmal und laesst sie dann stehen, die
# Overrides aendert man von Versuch zu Versuch. Das Fahrfenster traegt, was
# zum einzelnen Auftrag gehoert, nicht die Einrichtung der Anlage.
DRIVE_PLAN_KEYS = ("velocity_scaling", "acceleration_scaling")


def yaml_scalar(value):
    """Einen Wert als YAML schreiben. Nur die Faelle, die hier vorkommen."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        if value != value:                             # NaN ist ungleich sich selbst
            return ".nan"
        if value == INF:
            return ".inf"
        if value == -INF:
            return "-.inf"
        text = f"{value:.10g}"
        # Der Punkt muss stehen bleiben, auch wenn nichts dahinter kommt.
        # YAML liest 2 als Ganzzahl; rclcpp deklariert den Parameter dann als
        # integer, MoveIt erwartet dort ein double, und die Ausnahme daraus
        # wird nicht gefangen -- der Prozess bricht mit terminate() ab. Eine
        # ganze Zahl an dieser Stelle beendet also den Knoten. Das ist der
        # Grund, warum eine Zahl und ihre Schreibweise nicht dasselbe sind.
        if not any(c in text for c in ".eEn"):
            text += ".0"
        # Dasselbe mit Exponent: "1e-05" liest rclcpp als Zahl, PyYAML
        # (YAML 1.1) aber als Text -- dort braucht die Mantisse einen Punkt.
        # Ueber params_fuer_moveitpy kaeme der Wert so als String bei
        # MoveItPy an. "1.0e-05" lesen beide als double.
        if "e" in text and "." not in text:
            text = text.replace("e", ".0e", 1)
        return text
    if isinstance(value, int):
        return str(value)
    text = str(value)
    if text == "" or any(c in text for c in ":#{}[],&*?|-<>=!%@`\"'\n"):
        return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'
    return text


def yaml_block(text, indent):
    """Mehrzeiligen Text als Literalblock einruecken.

    Die URDF geht so unveraendert in die Parameterdatei. Sie ueber die
    Kommandozeile zu uebergeben scheitert an den Zeilenumbruechen, deshalb
    fuehrt der Weg immer ueber eine Datei.
    """
    pad = " " * indent
    lines = text.replace("\r\n", "\n").split("\n")
    while lines and not lines[-1].strip():
        lines.pop()
    return "|-\n" + "\n".join(pad + line for line in lines)


def build_srdf(robot_name, links, joints, group_name=MOVEIT_GROUP, tip=None):
    """SRDF aus dem geladenen Modell erzeugen.

    Von Hand gepflegt waere sie nach dem dritten Strukturbefehl kaputt: die
    GUI haengt Glieder an und nimmt sie heraus, und jeder dieser Schritte
    aendert Gruppe und Kollisionspaare. Deshalb entsteht sie hier neu, jedes
    Mal wenn das Modell sich aendert.

    Die Gruppe ist die Kette von der Wurzel zum Tip-Link. Benachbarte Links
    koennen sich nicht treffen, ihr Paar wird abgeschaltet; alles andere
    prueft MoveIt selbst.
    """
    roots = root_links(links, joints)
    base = roots[0] if roots else (links[0]["name"] if links else "base_link")

    if tip is None:
        leaves = leaf_links(links, joints)
        tip = leaves[0] if leaves else base

    lines = ['<?xml version="1.0"?>',
             f'<robot name="{robot_name or "robot"}">',
             f'  <group name="{group_name}">',
             f'    <chain base_link="{base}" tip_link="{tip}"/>',
             '  </group>']

    movable = [j for j in joints if j["type"] != "fixed" and j["mimic"] is None]
    if movable:
        lines.append(f'  <group_state name="null" group="{group_name}">')
        for joint in movable:
            lines.append(f'    <joint name="{joint["name"]}" value="0"/>')
        lines.append('  </group_state>')

    for joint in joints:
        if not joint["parent"] or not joint["child"]:
            continue
        lines.append(f'  <disable_collisions link1="{joint["parent"]}" '
                     f'link2="{joint["child"]}" reason="Adjacent"/>')

    lines.append('</robot>')
    return "\n".join(lines) + "\n"


class MoveItConfig:
    """Alle Werte, die move_group beim Hochfahren liest, plus die pro Auftrag.

    Bewusst ohne Tk und ohne ROS: die Klasse laesst sich ohne Fenster und
    ohne Roboter pruefen. Sie kennt drei Sorten von Werten und haelt sie
    getrennt, weil sie sich unterschiedlich verhalten:

        plan        wandert in den MotionPlanRequest, wirkt sofort
        limits      Achsgrenzen, werden beim Start gelesen
        cartesian   Bahngrenzen fuer Pilz, werden beim Start gelesen
        kinematics  Solver und dessen Parameter, wird beim Start gelesen
                    (dazu kdl und kdl_position_only, nur fuer KDL)

    Nur die erste Gruppe kommt ohne Neustart aus. Genau diese Unterscheidung
    zeigt die Oberflaeche dem Bediener an.
    """

    def __init__(self):
        self.plan = {key: default for key, _l, _u, default in PLAN_FIELDS}
        self.planner_id = "PTP"
        self.group = MOVEIT_GROUP
        self.tip = ""                                  # leer: Blatt automatisch
        self.limits = {}                               # {gelenk: {feld: wert|None}}
        self.cartesian = {key: default
                          for key, _l, _u, default in CARTESIAN_FIELDS}
        self.kinematics = {key: default
                           for key, _l, _u, default in KINEMATICS_FIELDS}
        self.planner = {key: default
                        for key, _l, _u, default in PLANNER_FIELDS}
        self.solver = KINEMATICS_SOLVERS[0]
        self.kdl = {key: default for key, _l, _u, default in KDL_FIELDS}
        self.kdl_position_only = KDL_POSITION_ONLY_DEFAULT
        self.use_ruckig = True
        # Kollisionspruefung der Bahn. An heisst: MoveIt prueft Startzustand
        # und jeden Stuetzpunkt gegen die Planungsszene und wirft die ganze
        # Bahn weg, sobald ein Paar sich beruehrt, das die SRDF nicht
        # freigegeben hat. Aus heisst nicht "es kollidiert nicht", sondern
        # "es wird nicht geprueft". Solange die Ersatzkoerper grob sind und
        # die Kollisionsmatrix nur benachbarte Paare kennt, ist das der
        # ehrlichere Zustand -- eine Pruefung, die falsche Treffer meldet,
        # ist keine Sicherheit, sondern nur ein Hindernis.
        self.check_collisions = True
        self.allow_execution = False                   # die GUI faehrt selbst

    # -- Achsgrenzen -------------------------------------------------------

    @staticmethod
    def defaults_for(joint):
        """Startwerte aus der URDF ableiten.

        Geschwindigkeit steht in <limit velocity=...>, Beschleunigung und Ruck
        stehen dort nicht und koennen es auch nicht: die URDF kennt sie nicht.
        Sie werden deshalb abgeleitet und sind zum Ueberschreiben gedacht.
        """
        velocity = joint.get("velocity")
        try:
            velocity = float(velocity)
        except (TypeError, ValueError):
            velocity = 0.0
        if velocity <= 0.0:
            velocity = DEFAULT_JOINT_VELOCITY
        acceleration = velocity * ACC_FROM_VEL
        return {"velocity": velocity,
                "acceleration": acceleration,
                "deceleration": -acceleration,
                "jerk": acceleration * JERK_FROM_ACC}

    def sync_joints(self, joints):
        """Eintraege an das Modell anpassen.

        Neue Gelenke bekommen Startwerte, verschwundene fallen heraus.
        Bestehende bleiben unveraendert -- ein Umbenennen des Gelenks setzt
        seine Grenzen also zurueck, und das ist richtig so: es ist aus Sicht
        der Konfiguration ein anderes Gelenk.
        """
        wanted = [j for j in joints
                  if j["type"] != "fixed" and j["mimic"] is None]
        names = {j["name"] for j in wanted}
        for name in list(self.limits):
            if name not in names:
                del self.limits[name]
        for joint in wanted:
            if joint["name"] not in self.limits:
                self.limits[joint["name"]] = self.defaults_for(joint)
        return [j["name"] for j in wanted]

    # -- Parameterdatei ----------------------------------------------------

    def limits_yaml(self, indent):
        pad = " " * indent
        out = []
        for name in sorted(self.limits):
            values = self.limits[name]
            out.append(f"{pad}{name}:")
            for key, has in (("velocity", "has_velocity_limits"),
                             ("acceleration", "has_acceleration_limits"),
                             ("jerk", "has_jerk_limits")):
                value = values.get(key)
                out.append(f"{pad}  {has}: {yaml_scalar(value is not None)}")
                if value is not None:
                    out.append(f"{pad}  max_{key}: {yaml_scalar(float(value))}")
            # Pilz-Erweiterung. Der Wert ist negativ, das ist dort so gewollt.
            value = values.get("deceleration")
            out.append(f"{pad}  has_deceleration_limits: "
                       f"{yaml_scalar(value is not None)}")
            if value is not None:
                out.append(f"{pad}  max_deceleration: {yaml_scalar(float(value))}")
        return "\n".join(out)

    def kdl_yaml(self, indent):
        """Die Zeilen der KDL-Parameter. Ganzzahlen bleiben Ganzzahlen."""
        pad = " " * indent
        out = []
        for key, _label, _unit, _default in KDL_FIELDS:
            value = self.kdl[key]
            value = int(value) if key in KDL_INT_KEYS else float(value)
            out.append(f"{pad}{key}: {yaml_scalar(value)}")
        out.append(f"{pad}position_only_ik: "
                   f"{yaml_scalar(bool(self.kdl_position_only))}")
        return out

    def params_yaml(self, urdf_xml, srdf_xml):
        """Die vollstaendige Parameterdatei fuer move_group.

        Die Pfade der Parameter haengen an der MoveIt-Fassung. Sie stehen
        deshalb hier an einer Stelle und nicht verteilt im Programm.
        """
        # Zwei Adapter tragen die Kollisionspruefung, je einer auf jeder
        # Seite: CheckStartStateCollision lehnt den Auftrag vorher ab,
        # ValidateSolution wirft die fertige Bahn nachher weg. Ein Schalter
        # fuer beide, sonst ist "aus" nur halb aus.
        requests = ["default_planning_request_adapters/ResolveConstraintFrames",
                    "default_planning_request_adapters/ValidateWorkspaceBounds",
                    "default_planning_request_adapters/CheckStartStateBounds"]
        if self.check_collisions:
            requests.append(
                "default_planning_request_adapters/CheckStartStateCollision")

        adapters = ["default_planning_response_adapters/DisplayMotionPath"]
        if self.check_collisions:
            adapters.insert(
                0, "default_planning_response_adapters/ValidateSolution")
        if self.use_ruckig:
            adapters.insert(
                0, "default_planning_response_adapters/AddRuckigTrajectorySmoothing")

        lines = [
            "/**:",
            "  ros__parameters:",
            "    robot_description: " + yaml_block(urdf_xml, 6),
            "    robot_description_semantic: " + yaml_block(srdf_xml, 6),
            "    robot_description_kinematics:",
            f"      {self.group}:",
            f"        kinematics_solver: {yaml_scalar(self.solver)}",
        ]
        for key, _label, _unit, _default in KINEMATICS_FIELDS:
            lines.append(f"        {key}: "
                         f"{yaml_scalar(float(self.kinematics[key]))}")
        if self.solver == KDL_SOLVER:
            lines += self.kdl_yaml(8)

        lines += [
            "    robot_description_planning:",
            "      joint_limits:",
            self.limits_yaml(8),
            "      cartesian_limits:",
        ]
        for key, _label, _unit, _default in CARTESIAN_FIELDS:
            lines.append(f"        {key}: "
                         f"{yaml_scalar(float(self.cartesian[key]))}")

        lines += [
            "    planning_pipelines: [\"pilz_industrial_motion_planner\"]",
            "    default_planning_pipeline: pilz_industrial_motion_planner",
            "    pilz_industrial_motion_planner:",
            "      planning_plugins:",
            "        - pilz_industrial_motion_planner/CommandPlanner",
            "      request_adapters:",
        ]
        for adapter in requests:
            lines.append(f"        - {adapter}")
        lines.append("      response_adapters:")
        for adapter in adapters:
            lines.append(f"        - {adapter}")

        # Sechs Leerzeichen: die Werte gehoeren unter
        # "pilz_industrial_motion_planner:" und heissen am Knoten damit
        # pilz_industrial_motion_planner.<key>. float() ist nicht kosmetisch --
        # "sampling_time: 1" waere in YAML ein Integer und wuerde in rclcpp
        # eine InvalidParameterTypeException ausloesen, "1.0" nicht.
        for key, _label, _unit, _default in PLANNER_FIELDS:
            lines.append(f"      {key}: "
                         f"{yaml_scalar(float(self.planner[key]))}")

        lines += [
            "    capabilities: \"pilz_industrial_motion_planner/MoveGroupSequenceAction "
            "pilz_industrial_motion_planner/MoveGroupSequenceService\"",
            # Ohne Controller waere jede Ausfuehrung ein Fehler. Die GUI
            # tastet die geplante Bahn selbst ab, deshalb bleibt das aus.
            f"    allow_trajectory_execution: {yaml_scalar(self.allow_execution)}",
            "    moveit_manage_controllers: false",
            "    publish_planning_scene: true",
            "    publish_geometry_updates: true",
            "    publish_state_updates: true",
            "    publish_transforms_updates: true",
        ]
        return "\n".join(lines) + "\n"

    # -- Vergleich ---------------------------------------------------------

    def startup_signature(self):
        """Alles, was einen Neustart erfordert, als vergleichbarer Text."""
        parts = [self.group, self.tip, self.solver, str(self.use_ruckig),
                 str(self.check_collisions)]
        for name in sorted(self.limits):
            values = self.limits[name]
            parts.append(name + "=" + ",".join(
                f"{key}:{values.get(key)}" for key, _l, _u, _w in LIMIT_FIELDS))
        for key, _l, _u, _d in CARTESIAN_FIELDS:
            parts.append(f"{key}:{self.cartesian[key]}")
        for key, _l, _u, _d in KINEMATICS_FIELDS:
            parts.append(f"{key}:{self.kinematics[key]}")
        for key, _l, _u, _d in KDL_FIELDS:
            parts.append(f"kdl.{key}:{self.kdl[key]}")
        parts.append(f"kdl.position_only_ik:{self.kdl_position_only}")
        # CommandPlanner::initialize() laeuft genau einmal beim Hochfahren und
        # legt die Abtastzeit in den LimitsContainer jedes Loaders. Ein
        # spaeteres Setzen des Parameters aendert nur den Parameterwert, nicht
        # die Abtastung. Deshalb gehoert das Feld in die Neustart-Kennung.
        for key, _l, _u, _d in PLANNER_FIELDS:
            parts.append(f"{key}:{self.planner[key]}")
        return "|".join(parts)


def find_move_group():
    """Pfad der move_group-Binary aus AMENT_PREFIX_PATH.

    Der Umweg ueber "ros2 run" kostet einen zusaetzlichen Prozess, und dieser
    eine Prozess ist der Grund fuer den Fehler, den 2.5 behebt: der Wrapper
    nimmt das Signal entgegen, das Kind ueberlebt ihn. Wer den Pfad kennt,
    braucht ihn nicht. Gefunden wird er ueber dieselbe Umgebungsvariable, die
    auch "ros2 run" durchsucht -- wer die GUI ohne gesourcetes Setup startet,
    findet hier nichts, und dann findet "ros2 run" auch nichts.
    """
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        path = os.path.join(prefix, "lib", "moveit_ros_move_group", "move_group")
        if os.access(path, os.X_OK):
            return path
    return None


def move_group_pids():
    """PIDs aller laufenden move_group-Prozesse, ueber /proc.

    Bewusst ohne pgrep: das ist ein weiterer Prozess, dessen Ausgabe geparst
    werden muesste, und unter WSL ist es nicht in jeder Ausbaustufe da. /proc
    ist da, sobald Linux laeuft.
    """
    found = []
    try:
        entries = os.listdir("/proc")
    except OSError:
        return found
    for entry in entries:
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as handle:
                parts = [p for p in handle.read().split(b"\0") if p]
        except OSError:                                # Prozess ist gerade weg
            continue
        if parts and parts[0].rsplit(b"/", 1)[-1] == b"move_group":
            found.append(int(entry))
    return found


class MoveItProcess:
    """move_group als Kindprozess der GUI.

    MoveIt liest Robotermodell, SRDF, Achsgrenzen und Solver beim Hochfahren
    und danach nicht mehr. Ein Modellwechsel zur Laufzeit ist dort nicht
    vorgesehen. Statt zu versuchen, den Knoten am Leben zu halten, macht die
    GUI den Neustart zur normalen Handlung: sie schreibt die Parameterdatei
    selbst und faehrt den Prozess damit neu hoch.

    Damit das trage, muss der alte Prozess auch wirklich weg sein. Er ist es
    seit 2.5: das Signal geht an die Prozessgruppe, und gestartet wird ohne
    Wrapper. Zusaetzlich traegt jeder Start einen eigenen Knotennamen, damit
    im Graphen zu erkennen ist, welcher Knoten zu dieser GUI gehoert.

    Zustaende: aus, startet, laeuft, beendet, fehler.
    """

    def __init__(self, log=None):
        self.log = log or (lambda text, level="info": None)
        self.process = None
        self.workdir = None
        self.started_at = None
        self.signature = None                          # Stand, mit dem gestartet wurde
        self.last_error = ""
        self.node_name = ""                            # Knoten des laufenden Starts
        self.logfile = None                            # Ausgabe von move_group
        self.command = []                              # zum Anzeigen im Protokoll
        self.pgid = None                               # Prozessgruppe des Starts
        self._serial = 0                               # zaehlt die Starts

    # -- Zustand -----------------------------------------------------------

    @property
    def alive(self):
        return self.process is not None and self.process.poll() is None

    def state(self):
        if self.process is None:
            return "aus"
        code = self.process.poll()
        if code is None:
            if self.started_at is not None and \
                    time.monotonic() - self.started_at < 3.0:
                return "startet"
            return "laeuft"
        return "beendet" if code == 0 else "fehler"

    def uptime(self):
        if not self.alive or self.started_at is None:
            return None
        return time.monotonic() - self.started_at

    def strays(self):
        """Fremde move_group-Prozesse: alle ausser dem eigenen.

        Der eigene ist nach dem Umbau derselbe Prozess, den Popen zurueckgibt.
        Bleibt hier trotzdem etwas uebrig, dann laeuft ein move_group, den
        diese GUI nicht gestartet hat -- aus einer frueheren Sitzung, aus
        einem Launchfile oder von Hand.
        """
        mine = self.process.pid if self.alive else None
        return [pid for pid in move_group_pids() if pid != mine]

    def log_tail(self, lines=12, window=65536):
        """Die letzten Zeilen der Ausgabe. Leer, solange nichts geschrieben ist.

        Gelesen wird nur das letzte Stueck der Datei, nicht die ganze. Der
        Unterschied ist nicht theoretisch: ValidateSolution schreibt zu jedem
        gefundenen Kontakt zwei Zeilen, ein gescheiterter Auftrag kommt so
        auf mehrere hundert, und nach einer Viertelstunde Fehlersuche liegt
        dort ein zweistelliger Megabytebetrag. readlines() zog den komplett
        in den Speicher, und zwar im GUI-Thread -- das Fenster stand still,
        genau in dem Augenblick, in dem es erklaeren sollte, was schiefging.

        Ein Fenster von 64 kB traegt bei diesen Zeilenlaengen rund 400
        Zeilen. Die erste davon ist womoeglich angeschnitten, deshalb faellt
        sie heraus, wenn nicht vom Dateianfang gelesen wurde.
        """
        if not self.logfile or not os.path.isfile(self.logfile):
            return []
        try:
            size = os.path.getsize(self.logfile)
            with open(self.logfile, "rb") as handle:
                start = max(0, size - window)
                handle.seek(start)
                raw = handle.read().decode("utf-8", errors="replace")
        except OSError:
            return []
        parts = raw.split("\n")
        if parts and parts[-1] == "":
            parts.pop()                                # Umbruch am Dateiende
        if start > 0 and parts:
            parts = parts[1:]                          # angeschnittene Zeile
        return [line.rstrip() for line in parts[-lines:] if line.strip()]

    def log_size(self):
        """Groesse der Protokolldatei in Byte, 0 wenn es sie nicht gibt."""
        if not self.logfile:
            return 0
        try:
            return os.path.getsize(self.logfile)
        except OSError:
            return 0

    # -- Lebenszyklus ------------------------------------------------------

    def start(self, config, urdf_xml, srdf_xml):
        """Parameterdatei schreiben und move_group starten.

        Gibt None zurueck oder eine Meldung. Ein laufender Prozess wird
        vorher beendet -- zwei move_group auf denselben Diensten waeren eine
        Fehlersuche ohne Ende, und genau die war es bis 2.4.
        """
        self.stop()

        self._serial += 1
        # Der Name muss ueber die Lebensdauer der GUI eindeutig sein, nicht
        # global. PID und laufende Nummer reichen dafuer und sind lesbar.
        self.node_name = f"move_group_gui{os.getpid()}_{self._serial}"

        try:
            self.workdir = tempfile.mkdtemp(prefix=WORKDIR_PREFIX)
            params = os.path.join(self.workdir, "move_group.yaml")
            with open(params, "w", encoding="utf-8") as handle:
                handle.write(config.params_yaml(urdf_xml, srdf_xml))
            # Zum Nachsehen, wenn etwas nicht passt. Kostet nichts.
            with open(os.path.join(self.workdir, "robot.urdf"), "w",
                      encoding="utf-8") as handle:
                handle.write(urdf_xml)
            with open(os.path.join(self.workdir, "robot.srdf"), "w",
                      encoding="utf-8") as handle:
                handle.write(srdf_xml)
        except OSError as exc:
            self.last_error = f"Parameterdatei nicht schreibbar: {exc}"
            return self.last_error

        arguments = ["--ros-args",
                     "--params-file", params,
                     "-r", f"__node:={self.node_name}"]
        binary = find_move_group()
        if binary:
            self.command = [binary] + arguments
        else:
            # Rueckfallweg. Er funktioniert, aber der Wrapper haengt wieder
            # dazwischen; deshalb steht es im Protokoll.
            self.command = ["ros2", "run", "moveit_ros_move_group",
                            "move_group"] + arguments
            self.log("move_group-Binary nicht in AMENT_PREFIX_PATH gefunden, "
                     "starte ueber ros2 run", "warn")

        # Die Ausgabe geht in eine Datei und nicht nach DEVNULL. Ohne sie ist
        # ein Fehlstart nicht aufklaerbar: der Prozess ist nach zwei Sekunden
        # weg, und die Meldung, die es erklaert, steht in seiner letzten Zeile.
        self.logfile = os.path.join(self.workdir, "move_group.log")
        try:
            handle = open(self.logfile, "wb")
        except OSError as exc:
            self.last_error = f"Protokolldatei nicht schreibbar: {exc}"
            return self.last_error

        try:
            self.process = subprocess.Popen(
                self.command,
                stdout=handle,
                stderr=subprocess.STDOUT,
                # Eigene Sitzung, damit ein Strg-C im Terminal der GUI nicht
                # gleich mit durchschlaegt -- und damit killpg unten eine
                # Gruppe hat, die nur diesen Baum enthaelt.
                start_new_session=True)
        except (OSError, ValueError) as exc:
            self.process = None
            self.last_error = f"move_group nicht startbar: {exc}"
            return self.last_error
        finally:
            handle.close()                             # Popen hat eine eigene Kopie

        # Wegen start_new_session ist die Gruppennummer gleich der PID. Sie
        # wird gemerkt, weil sie nach dem Einsammeln des Prozesses nicht mehr
        # zu erfragen ist -- die Gruppe kann dann noch Mitglieder haben.
        try:
            self.pgid = os.getpgid(self.process.pid)
        except OSError:
            self.pgid = self.process.pid

        self.started_at = time.monotonic()
        self.signature = config.startup_signature()
        self.last_error = ""
        self.log(f"move_group gestartet als {self.node_name} "
                 f"(PID {self.process.pid}), Parameter in {self.workdir}", "ok")
        return None

    def _signal_group(self, sig):
        """Ein Signal an die ganze Prozessgruppe.

        Der Kern der Sache. Wird ueber den Rueckfallweg "ros2 run" gestartet,
        ist move_group ein Enkelkind; ein Signal an self.process erreicht nur
        den Wrapper, und das Enkelkind laeuft weiter. start_new_session=True
        legt den ganzen Baum in eine eigene Gruppe, und diese Gruppe wird hier
        adressiert. Ohne Wrapper trifft es denselben einen Prozess wie zuvor.

        Die Gruppennummer stammt aus dem Start und nicht aus getpgid: sobald
        der Wrapper eingesammelt ist, gibt es seine PID nicht mehr, seine
        Gruppe aber schon.
        """
        if self.pgid is None:
            return
        try:
            os.killpg(self.pgid, sig)
        except OSError:
            if self.process is not None:
                try:
                    self.process.send_signal(sig)
                except OSError:
                    pass

    def _group_alive(self):
        """Lebt in der Prozessgruppe noch etwas?

        process.poll() beantwortet diese Frage nicht -- es sagt nur, ob das
        unmittelbare Kind weg ist. Genau dieser Unterschied ist der Fehler
        aus 2.4, und deshalb darf auch die Eskalation der Signale nicht an
        ihm haengen. Signal 0 stellt nichts zu, es prueft nur, ob es etwas
        zuzustellen gaebe.

        poll() davor sammelt den Wrapper ein: ein nicht abgeholter Zombie
        gehoert noch zur Gruppe und wuerde sie als lebendig ausweisen.
        """
        if self.pgid is None:
            return False
        if self.process is not None:
            self.process.poll()
        try:
            os.killpg(self.pgid, 0)
        except OSError:
            return False
        return True

    def _wait(self, timeout):
        """Rueckgabewert oder None, wenn die Zeit nicht gereicht hat."""
        if self.process is None:
            return None
        try:
            return self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    def _wait_group(self, timeout):
        """Warten, bis die Gruppe leer ist. Gibt zurueck, ob sie es wurde."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self._group_alive():
                return True
            time.sleep(0.05)
        return not self._group_alive()

    def stop(self, keep_workdir=True):
        """Beenden und aufraeumen. Erst freundlich, dann bestimmt.

        SIGINT zuerst: daran haengt rclcpp seinen geordneten Shutdown, der
        Knoten meldet sich beim Graphen ab und verschwindet sofort aus
        ros2 node list. SIGTERM danach, SIGKILL zuletzt.

        Weitergeschaltet wird, solange die Gruppe nicht leer ist. Am Wrapper
        allein liesse sich das nicht festmachen: der geht schon beim ersten
        Signal, und ein im Hintergrund gestartetes Kind ignoriert SIGINT
        ohnehin -- eine Shell setzt es dort auf ignorieren. Wer nur den
        Wrapper beobachtet, haelt die Sache fuer erledigt und laesst die
        Waise stehen.
        """
        for sig, grace in ((signal.SIGINT, 3.0),
                           (signal.SIGTERM, 3.0),
                           (signal.SIGKILL, 2.0)):
            if not self._group_alive():
                break
            self._signal_group(sig)
            self._wait_group(grace)
        if self._group_alive():
            self.last_error = ("move_group liess sich nicht beenden "
                               f"(Prozessgruppe {self.pgid})")
            self.log(self.last_error, "error")
        self._wait(0.5)                                # Zombie einsammeln
        self.process = None
        self.pgid = None
        self.started_at = None
        self.node_name = ""
        # Der Arbeitsordner bleibt liegen: er ist die einzige Spur, wenn der
        # Start misslungen ist. Aufgeraeumt wird beim naechsten Start der GUI,
        # siehe sweep_workdirs.
        if not keep_workdir and self.workdir and os.path.isdir(self.workdir):
            shutil.rmtree(self.workdir, ignore_errors=True)
            self.workdir = None
            self.logfile = None

    def kill_strays(self):
        """Fremde move_group-Prozesse beenden. Gibt die Zahl zurueck.

        Sie stammen aus einer Sitzung, die nicht sauber beendet wurde, oder
        aus einem Launchfile. Solange sie laufen, bedienen sie dieselben
        Dienste wie der eigene Knoten, und welcher antwortet, entscheidet
        nicht die GUI.
        """
        pids = self.strays()
        for pid in pids:
            for sig in (signal.SIGINT, signal.SIGTERM):
                try:
                    os.kill(pid, sig)
                except OSError:
                    break
                time.sleep(0.3)
                try:
                    os.kill(pid, 0)                    # lebt er noch?
                except OSError:
                    break
            else:
                try:
                    os.kill(pid, signal.SIGKILL)
                except OSError:
                    pass
        return len(pids)

    @staticmethod
    def sweep_workdirs(keep=None, max_age_s=0.0):
        """Liegengebliebene Arbeitsordner entfernen. Gibt die Zahl zurueck.

        Wird beim Start der GUI gerufen. Ordner, die juenger sind als
        max_age_s, bleiben stehen: es koennte eine zweite GUI daneben laufen.
        """
        base = tempfile.gettempdir()
        removed = 0
        now = time.time()
        try:
            entries = os.listdir(base)
        except OSError:
            return 0
        for name in entries:
            if not name.startswith(WORKDIR_PREFIX):
                continue
            path = os.path.join(base, name)
            if keep and os.path.abspath(path) == os.path.abspath(keep):
                continue
            try:
                if now - os.path.getmtime(path) < max_age_s:
                    continue
            except OSError:
                continue
            shutil.rmtree(path, ignore_errors=True)
            removed += 1
        return removed
