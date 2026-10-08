#!/usr/bin/env python3
"""URDF anzeigen, Achsen vorgeben, Vorwaertskinematik rechnen, TF senden.

Oberflaeche nach Skizze 1.4:

    Kopf    STOP-Taste, Robotername, Meldungsfenster
    Mitte   links die Gelenke (rad und grad), in der Mitte die Frames
            (X Y Z RPY), rechts die Sollwertvorgabe kartesisch oder
            achsspezifisch samt Jerk, Beschleunigung und Geschwindigkeit
    Fuss    Sitzungsdauer und Alter der letzten TF-Ausgabe

Alle Frames gehen zyklisch auf /tf, auch die fester Gelenke. Das ist Absicht:
ueber /tf_static abgelegte Frames verfallen nie und blieben nach einem
Modellwechsel als Geisterframes in RViz stehen.
Zusaetzlich wird robot_description latched gesendet, damit RViz das Modell findet.
Die Frames stehen als Baum entlang der kinematischen Kette und lassen sich
zweigweise einklappen; die ganze Karte laesst sich wegklappen, dann bekommen
Gelenkliste und Sollwerte den Platz.
Das Bearbeiten von Laengen, Winkeln und Gelenkgrenzen liegt in einem eigenen
Fenster; uebernommene Aenderungen wirken sofort, geschrieben wird erst per
Speichern.

Aufruf:
    ./urdf_gui12.py                       # Standardpfad aus URDF_PATH
    ./urdf_gui12.py ~/Display/bmKinematic.urdf
"""

import argparse
import copy
import math
import os
import re
import shutil
import sys
import tempfile
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
import xml.etree.ElementTree as ET
from datetime import datetime
from tkinter import ttk, filedialog, messagebox

# ---------- hier URDF eintragen ----------
URDF_PATH = "/home/fwohlfeil/simple_robot.urdf"
# ---------- Ablage fuer bearbeitete Staende ----------
# Absoluter Pfad oder relativ zur geladenen URDF. Wird bei Bedarf angelegt.
BACKUP_DIR = "/home/fwohlfeil/Display/URDF_Speicher"
# -----------------------------------------

TF_RATE_HZ = 30.0
FOOTER_TICK_MS = 50
# Beim Ziehen am Schieber nicht bei jedem Pixel die Tabellen neu bauen.
# Die TF-Ausgabe bekommt den Wert trotzdem sofort.
SLIDER_REFRESH_MS = 60
# Spanne der Schieber, wenn die URDF keinen Anschlag vorgibt.
FREE_ROTATION_DEG = 180.0
FREE_TRAVEL_M = 1.0

# ---------- Farben ----------
# Eine Palette fuer alles: Flaechen hell, Text dunkelgrau statt schwarz,
# Signalfarben nur dort, wo sie wirklich etwas bedeuten.
BG = "#eef1f5"                                         # Fensterhintergrund
SURFACE = "#ffffff"                                    # Karten und Tabellen
BORDER = "#d5dbe3"
TEXT = "#1f2933"
MUTED = "#6b7480"
FAINT = "#9aa3ad"
DISABLED = "#a8b0ba"
DISABLED_BG = "#c3c9d1"

ACCENT = "#2563c9"
OK_GREEN = "#1a7f37"
WARN = "#9a6700"

BUTTON = "#e3e8ef"
BUTTON_HOVER = "#d5dce6"
BUTTON_DOWN = "#c7cfdb"
HEADING = "#eaeef3"                                    # Tabellenkopf
SELECT = "#d3e3fb"
STRIPE = "#f7f9fb"                                     # jede zweite Zeile
SCROLL = "#ccd3dc"
TROUGH = "#dde3ea"                                     # Schienenfarbe der Schieber
LOG_BG = "#fbfcfd"

TINT_OK = "#e6f4ea"                                    # TCP-Zeile
TINT_BAD = "#fdeceb"                                   # nicht erreichbarer Frame
TINT_WARN = "#fdf3e2"

COLOR_STOP = "#d13438"
COLOR_STOP_HOVER = "#b32b2f"
COLOR_GO = "#2e7d32"
COLOR_GO_HOVER = "#256628"

BG_OK = SURFACE                                        # Eingabefeld in Ordnung
BG_CLAMPED = "#fdf3e2"                                 # auf das Limit begrenzt
BG_INVALID = "#fbe0e0"                                 # nicht lesbar


# ==========================================================================
# Matrizen: 4x4 als Liste von Zeilen
# ==========================================================================

def identity():
    return [[1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def from_rpy_xyz(rpy, xyz):
    """URDF-origin als 4x4. Feste Achsen: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp,     cp * sr,                cp * cr,                xyz[2]],
        [0.0,     0.0,                    0.0,                    1.0],
    ]


def from_axis_angle(axis, angle):
    """Rodrigues-Formel als 4x4. axis muss normiert sein."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    k = 1.0 - c
    return [
        [x * x * k + c,     x * y * k - z * s, x * z * k + y * s, 0.0],
        [y * x * k + z * s, y * y * k + c,     y * z * k - x * s, 0.0],
        [z * x * k - y * s, z * y * k + x * s, z * z * k + c,     0.0],
        [0.0,               0.0,               0.0,               1.0],
    ]


def from_translation(xyz):
    matrix = identity()
    matrix[0][3], matrix[1][3], matrix[2][3] = xyz
    return matrix


def to_xyz_rpy(matrix):
    """4x4 -> ((x, y, z), (roll, pitch, yaw)). Umkehrung von from_rpy_xyz."""
    xyz = (matrix[0][3], matrix[1][3], matrix[2][3])
    sp = -matrix[2][0]
    if abs(sp) >= 1.0 - 1e-9:                          # Gimbal Lock
        pitch = math.copysign(math.pi / 2.0, sp)
        roll = 0.0
        yaw = math.atan2(-matrix[0][1], matrix[1][1])
    else:
        pitch = math.asin(sp)
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    return xyz, (roll, pitch, yaw)


def to_quaternion(matrix):
    """4x4 -> (x, y, z, w), Shepperd-Methode."""
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2][1] - matrix[1][2]) / s
        y = (matrix[0][2] - matrix[2][0]) / s
        z = (matrix[1][0] - matrix[0][1]) / s
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / s
        x = 0.25 * s
        y = (matrix[0][1] + matrix[1][0]) / s
        z = (matrix[0][2] + matrix[2][0]) / s
    elif matrix[1][1] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / s
        x = (matrix[0][1] + matrix[1][0]) / s
        y = 0.25 * s
        z = (matrix[1][2] + matrix[2][1]) / s
    else:
        s = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / s
        x = (matrix[0][2] + matrix[2][0]) / s
        y = (matrix[1][2] + matrix[2][1]) / s
        z = 0.25 * s
    return x, y, z, w


# ==========================================================================
# URDF einlesen
# ==========================================================================

def num(elem, name, default=0.0):
    if elem is None or elem.get(name) is None:
        return default
    return float(elem.get(name))


def vec(elem, name, default=(0.0, 0.0, 0.0)):
    if elem is None or elem.get(name) is None:
        return list(default)
    return [float(v) for v in elem.get(name).split()]


def fmt(values, digits=4):
    return ", ".join(f"{v:.{digits}f}" for v in values)


def is_rotational(joint):
    return joint["type"] in ("revolute", "continuous")


def slider_range(joint):
    """Spanne des Schiebers in Anzeigeeinheit, also grad bzw. m.

    Ohne Anschlag in der URDF (continuous) braucht der Schieber trotzdem
    zwei Enden. Die Eingabe im Feld bleibt davon unbegrenzt.
    """
    if joint["has_limit"]:
        if is_rotational(joint):
            return math.degrees(joint["lower"]), math.degrees(joint["upper"])
        return joint["lower"], joint["upper"]
    if is_rotational(joint):
        return -FREE_ROTATION_DEG, FREE_ROTATION_DEG
    return -FREE_TRAVEL_M, FREE_TRAVEL_M


def parse_root(root):
    """Aus einem ElementTree-Baum lesen. Gibt (name, links, joints) zurueck."""
    links = []
    for link in root.findall("link"):
        inertial = link.find("inertial")
        links.append({
            "name": link.get("name"),
            "mass": num(inertial.find("mass") if inertial is not None else None, "value"),
            "com": vec(inertial.find("origin") if inertial is not None else None, "xyz"),
            "has_inertial": inertial is not None,
        })

    joints = []
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        limit = joint.find("limit")
        rpy = vec(origin, "rpy")
        xyz = vec(origin, "xyz")

        axis = vec(joint.find("axis"), "xyz", (1.0, 0.0, 0.0))
        length = math.sqrt(sum(a * a for a in axis))
        axis = [a / length for a in axis] if length > 1e-12 else [1.0, 0.0, 0.0]

        jtype = joint.get("type")
        joints.append({
            "name": joint.get("name"),
            "type": jtype,
            "parent": joint.find("parent").get("link") if joint.find("parent") is not None else None,
            "child": joint.find("child").get("link") if joint.find("child") is not None else None,
            "xyz": xyz,
            "rpy": rpy,
            "origin": from_rpy_xyz(rpy, xyz),          # konstant, vorab gerechnet
            "quat": to_quaternion(from_rpy_xyz(rpy, xyz)),
            "axis": axis,
            "lower": num(limit, "lower"),
            "upper": num(limit, "upper"),
            "effort": num(limit, "effort"),
            "velocity": num(limit, "velocity"),
            # continuous hat definitionsgemaess keinen Anschlag
            "has_limit": limit is not None and jtype != "continuous",
        })

    return root.get("name"), links, joints


def parse(path):
    """URDF-Datei einlesen. Gibt (root, name, links, joints, xml) zurueck."""
    with open(path, "r") as handle:
        urdf_xml = handle.read()
    root = ET.fromstring(urdf_xml)
    name, links, joints = parse_root(root)
    return root, name, links, joints, urdf_xml


def serialize(root):
    """Baum als eingerueckten XML-Text. ET.indent gibt es ab Python 3.9."""
    copied = copy.deepcopy(root)
    try:
        ET.indent(copied, space="  ")
    except AttributeError:
        pass
    return ET.tostring(copied, encoding="unicode")


def backup_dir_for(path):
    """Ablageordner, wird bei Bedarf angelegt."""
    if os.path.isabs(BACKUP_DIR):
        folder = BACKUP_DIR
    else:
        folder = os.path.join(os.path.dirname(os.path.abspath(path)), BACKUP_DIR)
    os.makedirs(folder, exist_ok=True)
    return folder


STAMP = re.compile(r"_\d{8}_\d{6}$")


def timestamped_name(path):
    """Dateiname mit Zeitstempel, Endung bleibt erhalten.

    Ein bereits vorhandener Stempel wird ersetzt, sonst haengt sich beim
    Speichern einer Ablagedatei ein zweiter an den Namen.
    """
    stem, suffix = os.path.splitext(os.path.basename(path))
    return f"{STAMP.sub('', stem)}_{datetime.now():%Y%m%d_%H%M%S}{suffix or '.urdf'}"


def save_urdf(path, xml_text):
    """Atomar schreiben. Wird eine vorhandene Datei ueberschrieben, kommt
    vorher eine Sicherungskopie in den Ablageordner."""
    backup = None
    if os.path.exists(path):
        backup = os.path.join(backup_dir_for(path), timestamped_name(path))
        shutil.copy2(path, backup)

    # Die temporaere Datei muss im Zielordner liegen. Ueber Dateisystemgrenzen
    # hinweg kopiert os.replace und ist dann nicht mehr atomar.
    handle_fd, tmp = tempfile.mkstemp(dir=os.path.dirname(os.path.abspath(path)),
                                      suffix=".tmp")
    try:
        with os.fdopen(handle_fd, "w") as handle:
            handle.write(xml_text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)                          # atomar im selben Dateisystem
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return backup


def validate_root(root):
    """Prueft den Baum. Gibt eine Liste von Fehlermeldungen zurueck."""
    errors = []
    joints = root.findall("joint")
    links = {l.get("name") for l in root.findall("link")}
    children = []

    for joint in joints:
        name = joint.get("name")
        for role in ("parent", "child"):
            element = joint.find(role)
            if element is None:
                errors.append(f"{name}: <{role}> fehlt")
                continue
            link = element.get("link")
            if link not in links:
                errors.append(f'{name}: Link "{link}" ist nicht deklariert')
            if role == "child":
                children.append(link)

        limit = joint.find("limit")
        if limit is not None and joint.get("type") in ("revolute", "prismatic"):
            lower = float(limit.get("lower", 0.0))
            upper = float(limit.get("upper", 0.0))
            if lower >= upper:
                errors.append(f"{name}: untere Grenze muss kleiner als die obere sein")

    if len(set(children)) != len(children):
        errors.append("Ein Link hat mehrere Eltern")

    roots = links - set(children)
    if len(roots) != 1:
        errors.append(f"Genau ein Wurzel-Link erwartet, gefunden: {len(roots)}")

    return errors


# ==========================================================================
# Vorwaertskinematik
# ==========================================================================

def local_transform(joint, value):
    """Parent -> Child bei Gelenkwert `value` (rad bzw. m)."""
    if joint["type"] in ("revolute", "continuous"):
        return multiply(joint["origin"], from_axis_angle(joint["axis"], value))
    if joint["type"] == "prismatic":
        return multiply(joint["origin"],
                        from_translation([a * value for a in joint["axis"]]))
    return joint["origin"]


def root_links(links, joints):
    """Links, die in keinem <child> vorkommen."""
    children = {j["child"] for j in joints}
    return [l["name"] for l in links if l["name"] not in children]


def leaf_links(links, joints):
    """Links, die in keinem <parent> vorkommen. Kandidaten fuer den TCP."""
    parents = {j["parent"] for j in joints}
    return [l["name"] for l in links if l["name"] not in parents]


def forward_kinematics(links, joints, positions):
    """{linkname: 4x4} relativ zum Wurzel-Link, in einem Durchlauf."""
    roots = root_links(links, joints)
    if not roots:
        return {}                                      # Zyklus, nichts berechenbar

    poses = {roots[0]: identity()}
    pending = list(joints)
    while pending:
        progressed = False
        for joint in list(pending):
            if joint["parent"] in poses:
                poses[joint["child"]] = multiply(
                    poses[joint["parent"]],
                    local_transform(joint, positions.get(joint["name"], 0.0)))
                pending.remove(joint)
                progressed = True
        if not progressed:
            break                                      # abgehaengter Teilbaum
    return poses


# ==========================================================================
# ROS-Anbindung
# ==========================================================================

class RosBridge:
    """Node, Spin-Thread und TF-Ausgabe. Alle ROS-Importe liegen hier drin."""

    def __init__(self):
        self.node = None
        self.error = None
        self.tick_count = 0
        self.enabled = True

        # Vom Executor-Thread geschrieben, vom GUI-Thread gelesen. Einzelne
        # float-Zuweisungen sind unter dem GIL atomar, ein Lock ist unnoetig.
        self.last_publish = None                       # time.monotonic()
        self.cycle_ms = None                           # geglaetteter Zyklusabstand

        self._lock = threading.Lock()
        self._broadcast = []                           # alle Gelenke, die auf /tf gehen
        self._positions = {}
        self._executor = None
        self._thread = None
        self._timer = None
        self._broadcaster = None
        self._description_pub = None

    @property
    def running(self):
        return self.node is not None

    # -- Lebenszyklus ------------------------------------------------------

    def start(self):
        """Node hochfahren. Gibt None zurueck oder eine Fehlermeldung."""
        if self.running:
            return None
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import (QoSProfile, DurabilityPolicy,
                                   ReliabilityPolicy, HistoryPolicy)
            from std_msgs.msg import String
            from tf2_ros import TransformBroadcaster
        except ImportError as exc:
            self.error = f"ROS 2 nicht verfuegbar ({exc.name}). Nur Anzeige und FK."
            return self.error

        rclpy.init()
        self.node = Node("urdf_gui_tf")
        self._broadcaster = TransformBroadcaster(self.node)

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._description_pub = self.node.create_publisher(String, "robot_description", qos)

        self._timer = self.node.create_timer(1.0 / TF_RATE_HZ, self._on_timer)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        return None

    def shutdown(self):
        if not self.running:
            return
        import rclpy
        self._executor.shutdown()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self.node = None

    # -- Modell und Werte --------------------------------------------------

    def set_model(self, joints, urdf_xml):
        """Neues Modell uebernehmen. Alle Frames laufen zyklisch ueber /tf.

        Feste Gelenke bewusst NICHT auf /tf_static: dort abgelegte Frames
        landen in einem Cache, der nie verfaellt. Nach einem Modellwechsel
        blieben sie als Geisterframes in RViz stehen, und loeschen laesst sich
        in tf2 nichts. Ueber /tf verfallen sie nach der Cache-Zeit von selbst.
        """
        if not self.running:
            return self.error or "Kein ROS-Node aktiv"

        from std_msgs.msg import String

        usable = [j for j in joints if j["parent"] and j["child"]]
        movable = [j for j in usable if j["type"] != "fixed"]

        self._description_pub.publish(String(data=urdf_xml))

        with self._lock:
            self._broadcast = usable
            self._positions = {j["name"]: 0.0 for j in movable}
        self.tick_count = 0
        self.last_publish = None
        self.cycle_ms = None

        self._publish_once()                           # nicht auf den Timer warten

        return (f"{len(usable)} Frames auf /tf mit {TF_RATE_HZ:.0f} Hz "
                f"({len(movable)} beweglich), robot_description gesendet")

    def set_positions(self, positions):
        """Sollwerte uebernehmen. Wird aus dem GUI-Thread aufgerufen."""
        with self._lock:
            self._positions = dict(positions)

    # -- Ausgabe -----------------------------------------------------------

    def _message(self, joint, matrix, stamp):
        from geometry_msgs.msg import TransformStamped
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = joint["parent"]
        msg.child_frame_id = joint["child"]
        msg.transform.translation.x = matrix[0][3]
        msg.transform.translation.y = matrix[1][3]
        msg.transform.translation.z = matrix[2][3]
        qx, qy, qz, qw = to_quaternion(matrix)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg

    def _publish_once(self):
        """Einen Zyklus senden. Feste Gelenke liefern dabei stets ihr origin."""
        with self._lock:
            frames = list(self._broadcast)
            positions = dict(self._positions)
        if not frames:
            return

        stamp = self.node.get_clock().now().to_msg()
        self._broadcaster.sendTransform([
            self._message(j, local_transform(j, positions.get(j["name"], 0.0)), stamp)
            for j in frames
        ])

        now = time.monotonic()
        if self.last_publish is not None:
            delta_ms = (now - self.last_publish) * 1000.0
            self.cycle_ms = (delta_ms if self.cycle_ms is None
                             else 0.9 * self.cycle_ms + 0.1 * delta_ms)
        self.last_publish = now
        self.tick_count += 1

    def _on_timer(self):
        """Laeuft im Executor-Thread. Nur lesen, was unter dem Lock geholt wurde."""
        if not self.enabled:
            return
        self._publish_once()


# ==========================================================================
# GUI
# ==========================================================================

# Spalten: Schluessel, Ueberschrift, Breite, Ausrichtung.
# Zahlen stehen rechts, damit die Kommastellen untereinander liegen.
# Der Framename steht in der Baumspalte #0, damit die Einrueckung dort
# sichtbar wird. Deshalb taucht er hier nicht mehr auf.
FRAME_TREE = ("Frame", 150)                            # Mindestbreite, waechst mit
TREE_INDENT = 14                                       # Einrueckung je Ebene
TREE_MAX_WIDTH = 330                                   # sonst ufert die Spalte aus
FRAME_COLUMNS = [
    ("x",     "X [m]",    64, "e"),
    ("y",     "Y [m]",    64, "e"),
    ("z",     "Z [m]",    64, "e"),
    ("roll",  "R [deg]",  66, "e"),
    ("pitch", "P [deg]",  66, "e"),
    ("yaw",   "Y [deg]",  66, "e"),
]

JOINT_COLUMNS = [
    ("name",  "Gelenk",       95, "w"),
    ("type",  "Typ",          84, "w"),
    ("rad",   "rad/m",        66, "e"),
    ("grad",  "grad/mm",      80, "e"),
    ("limit", "Grenzen",      92, "e"),
    ("chain", "Kette",       150, "w"),
]

LINK_COLUMNS = [
    ("name", "Link",        160, "w"),
    ("mass", "Masse [kg]",   95, "e"),
    ("com",  "COM xyz [m]", 175, "w"),
]

CART_FIELDS = [("x", "X", "m"), ("y", "Y", "m"), ("z", "Z", "m"),
               ("roll", "Roll", "deg"), ("pitch", "Pitch", "deg"),
               ("yaw", "Yaw", "deg")]

MOTION_FIELDS = [("jerk", "Jerk", "m/s3", 10.0),
                 ("acceleration", "Acceleration", "m/s2", 2.0),
                 ("speed", "Speed", "m/s", 0.5)]

PAD = 10                                               # ein Rastermass fuer alles


class UrdfViewer(tk.Tk):

    def __init__(self, urdf_path):
        super().__init__()
        self.title("URDF Viewer")
        self.geometry("1560x900")
        self.minsize(1220, 700)
        self.configure(background=BG)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.urdf_path = urdf_path
        self.robot_name = ""
        self.xml_root = None                           # bearbeiteter Baum, die Wahrheit
        self.links = []
        self.joints = []
        self.urdf_xml = ""
        self.dirty = False                             # geaendert, aber nicht gespeichert
        self.stopped = False                           # STOP gedrueckt
        self.positions = {}                            # {gelenkname: rad bzw. m}
        self.entries = {}                              # {gelenkname: tk.Entry}
        self.sliders = {}                              # {gelenkname: ttk.Scale}
        self.syncing = False                           # Schieber wird gerade gesetzt
        self.refresh_job = None                        # laufender after-Aufruf
        self.cart_entries = {}                         # Zielvorgabe kartesisch
        self.motion_entries = {}                       # Jerk, Acceleration, Speed
        self.motion = {key: default for key, _l, _u, default in MOTION_FIELDS}
        self.frames_shown = True                       # Frames-Karte sichtbar
        self.frame_items = {}                          # {linkname: iid im Baum}
        self.frame_layout = None                       # {linkname: elternname}
        self.editor = None                             # Bearbeiten-Fenster oder None
        self.edit_entries = {}
        self.go_buttons = []

        self.robot_var = tk.StringVar(value="kein Modell")
        self.info_var = tk.StringVar(value="")
        self.path_var = tk.StringVar(value=os.path.basename(urdf_path))
        self.tcp_var = tk.StringVar(value="")
        self.session_var = tk.StringVar(value="Session  00:00:00:000")
        self.refresh_var = tk.StringVar(value="Last Refresh  --")
        self.edit_joint_var = tk.StringVar(value="")
        self.edit_info_var = tk.StringVar(value="")
        self.edit_unit_var = tk.StringVar(value="")
        self.send_tf = tk.BooleanVar(value=True)

        self.session_start = time.monotonic()
        self.ros = RosBridge()

        self._setup_fonts()
        self._setup_style()
        self._build_ui()
        self._start_ros()
        self._load(urdf_path, initial=True)
        self._tick()

    # ----------------------------------------------------------------------
    # Aussehen
    # ----------------------------------------------------------------------

    def _setup_fonts(self):
        """Die benannten Tk-Schriften einmal zentral setzen.

        Der Standard ist auf vielen Linux-Systemen ein sehr kleines Bitmapfont.
        Es wird die erste Schrift genommen, die tatsaechlich vorhanden ist.
        """
        available = set(tkfont.families())

        base = tkfont.nametofont("TkDefaultFont")
        for family in ("Ubuntu", "Noto Sans", "DejaVu Sans", "Segoe UI", "Cantarell"):
            if family in available:
                base.configure(family=family)
                break
        base.configure(size=10)
        tkfont.nametofont("TkTextFont").configure(**base.actual())

        mono = tkfont.nametofont("TkFixedFont")
        for family in ("Ubuntu Mono", "DejaVu Sans Mono", "Noto Sans Mono",
                       "Liberation Mono", "Consolas"):
            if family in available:
                mono.configure(family=family)
                break
        mono.configure(size=10)

    def _setup_style(self):
        """Ein Farbschema fuer alle ttk-Elemente. clam laesst sich als
        einziges mitgeliefertes Thema durchgaengig umfaerben."""
        style = ttk.Style(self)
        style.theme_use("clam")

        style.configure(".", background=BG, foreground=TEXT,
                        fieldbackground=SURFACE, bordercolor=BORDER,
                        focuscolor=ACCENT)
        style.configure("TFrame", background=BG)
        style.configure("TLabel", background=BG, foreground=TEXT)
        style.configure("Card.TFrame", background=SURFACE)
        style.configure("Card.TLabel", background=SURFACE)
        style.configure("Title.TLabel", background=SURFACE, foreground=TEXT,
                        font=("TkDefaultFont", 17, "bold"))
        style.configure("Muted.TLabel", background=SURFACE, foreground=MUTED)
        style.configure("Section.TLabel", background=SURFACE, foreground=MUTED,
                        font=("TkDefaultFont", 9, "bold"))
        style.configure("Foot.TLabel", background=BG, foreground=MUTED,
                        font=("TkFixedFont", 10))
        style.configure("Tcp.TLabel", background=SURFACE, foreground=OK_GREEN,
                        font=("TkFixedFont", 9))

        style.configure("TButton", background=BUTTON, foreground=TEXT,
                        bordercolor=BORDER, lightcolor=BUTTON, darkcolor=BUTTON,
                        relief="flat", padding=(12, 7), focusthickness=0)
        style.map("TButton",
                  background=[("pressed", BUTTON_DOWN), ("active", BUTTON_HOVER),
                              ("disabled", BG)],
                  foreground=[("disabled", DISABLED)])

        style.configure("Mini.TButton", background=SURFACE, foreground=MUTED,
                        bordercolor=BORDER, lightcolor=SURFACE, darkcolor=SURFACE,
                        relief="flat", padding=(8, 2), focusthickness=0,
                        font=("TkDefaultFont", 9))
        style.map("Mini.TButton",
                  background=[("pressed", BUTTON_DOWN), ("active", BUTTON)],
                  foreground=[("active", TEXT)])

        style.configure("TCheckbutton", background=SURFACE, foreground=TEXT,
                        indicatorcolor=SURFACE, focuscolor=SURFACE)
        style.map("TCheckbutton", background=[("active", SURFACE)],
                  indicatorcolor=[("selected", ACCENT)])

        style.configure("TCombobox", fieldbackground=SURFACE, background=BUTTON,
                        arrowcolor=TEXT, bordercolor=BORDER, padding=4)

        style.configure("TNotebook", background=BG, borderwidth=0, tabmargins=(0, 0, 0, 0))
        style.configure("TNotebook.Tab", background=BG, foreground=MUTED,
                        bordercolor=BORDER, padding=(16, 8), font=("TkDefaultFont", 10))
        style.map("TNotebook.Tab",
                  background=[("selected", SURFACE)],
                  foreground=[("selected", TEXT)],
                  expand=[("selected", (0, 0, 0, 0))])

        style.configure("Treeview", background=SURFACE, fieldbackground=SURFACE,
                        foreground=TEXT, rowheight=26, borderwidth=0, relief="flat")
        self.tree_indent = 20                          # Vorgabe aelterer Tk-Fassungen
        try:
            # Eine serielle Kette staffelt sich sonst quer aus der Spalte.
            style.configure("Treeview", indent=TREE_INDENT)   # erst ab Tk 8.6.11
            self.tree_indent = TREE_INDENT
        except tk.TclError:
            pass
        style.configure("Treeview.Heading", background=HEADING, foreground=MUTED,
                        relief="flat", padding=(6, 7),
                        font=("TkDefaultFont", 9, "bold"))
        style.map("Treeview.Heading", background=[("active", BUTTON_HOVER)])
        style.map("Treeview", background=[("selected", SELECT)],
                  foreground=[("selected", TEXT)])

        for orient in ("Vertical", "Horizontal"):
            style.configure(f"{orient}.TScrollbar", background=SCROLL,
                            troughcolor=SURFACE, bordercolor=SURFACE,
                            lightcolor=SCROLL, darkcolor=SCROLL,
                            arrowcolor=MUTED, borderwidth=0)
            style.map(f"{orient}.TScrollbar", background=[("active", MUTED)])

        style.configure("Joint.Horizontal.TScale", background=SURFACE,
                        troughcolor=TROUGH, bordercolor=BORDER,
                        lightcolor=ACCENT, darkcolor=ACCENT, borderwidth=0)
        style.map("Joint.Horizontal.TScale",
                  background=[("disabled", SURFACE)],
                  troughcolor=[("disabled", DISABLED_BG)],
                  lightcolor=[("disabled", DISABLED)],
                  darkcolor=[("disabled", DISABLED)])

        style.configure("TSeparator", background=BORDER)

    # -- kleine Bausteine --------------------------------------------------

    def _card(self, parent, **grid):
        """Weisse Flaeche mit duennem Rand. Traegt alle Inhaltsbereiche."""
        frame = tk.Frame(parent, background=SURFACE, bd=0,
                         highlightthickness=1, highlightbackground=BORDER,
                         highlightcolor=BORDER)
        if grid:
            frame.grid(**grid)
        return frame

    def _entry(self, parent, width=10, value="", font=None):
        """Eingabefeld mit Rand in der Themenfarbe und blauem Fokusrahmen."""
        entry = tk.Entry(parent, width=width, justify="right",
                         font=font or ("TkFixedFont", 10),
                         background=SURFACE, foreground=TEXT,
                         disabledbackground=BG, disabledforeground=DISABLED,
                         insertbackground=TEXT, relief="flat",
                         highlightthickness=1, highlightbackground=BORDER,
                         highlightcolor=ACCENT)
        if value:
            entry.insert(0, value)
        return entry

    def _flat_button(self, parent, text, command, colour, hover, width=9,
                     height=1, size=11):
        button = tk.Button(parent, text=text, command=command,
                           background=colour, foreground="white",
                           activebackground=hover, activeforeground="white",
                           font=("TkDefaultFont", size, "bold"),
                           width=width, height=height, relief="flat",
                           bd=0, highlightthickness=0, cursor="hand2")
        button.bind("<Enter>", lambda _e: button.configure(
            background=button.hover_colour))
        button.bind("<Leave>", lambda _e: button.configure(
            background=button.base_colour))
        button.base_colour, button.hover_colour = colour, hover
        return button

    # ----------------------------------------------------------------------
    # Aufbau
    # ----------------------------------------------------------------------

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self._build_head()
        self._build_body()
        self._build_foot()

    def _build_head(self):
        """STOP, Robotername, Meldungsfenster und die Dateitasten."""
        outer = ttk.Frame(self, padding=(PAD, PAD, PAD, 0))
        outer.grid(row=0, column=0, sticky="ew")
        outer.columnconfigure(0, weight=1)

        card = self._card(outer, row=0, column=0, sticky="ew")
        card.columnconfigure(1, weight=1)
        inner = ttk.Frame(card, style="Card.TFrame", padding=(14, 12))
        inner.grid(row=0, column=0, sticky="nsew")
        card.columnconfigure(0, weight=1)
        inner.columnconfigure(1, weight=1)

        self.stop_button = self._flat_button(
            inner, "STOP", self._toggle_stop, COLOR_STOP, COLOR_STOP_HOVER,
            width=8, height=4, size=14)
        self.stop_button.grid(row=0, column=0, rowspan=2, sticky="ns", padx=(0, 16))

        title = ttk.Frame(inner, style="Card.TFrame")
        title.grid(row=0, column=1, sticky="ew")
        ttk.Label(title, textvariable=self.robot_var, style="Title.TLabel").pack(
            side="left")
        ttk.Label(title, textvariable=self.info_var, style="Muted.TLabel").pack(
            side="left", padx=(16, 0), pady=(6, 0))

        log = tk.Frame(inner, background=LOG_BG, bd=0, highlightthickness=1,
                       highlightbackground=BORDER)
        log.grid(row=1, column=1, sticky="nsew", pady=(10, 0))
        log.columnconfigure(0, weight=1)
        self.messages = tk.Text(log, height=7, wrap="none", state="disabled",
                                background=LOG_BG, foreground=TEXT,
                                relief="flat", highlightthickness=0,
                                padx=10, pady=8, font=("TkFixedFont", 10),
                                spacing1=1, spacing3=1)
        self.messages.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(log, orient="vertical", command=self.messages.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.messages.configure(yscrollcommand=scroll.set)
        self.messages.tag_configure("time", foreground=FAINT)
        for tag, colour in (("info", TEXT), ("ok", OK_GREEN),
                            ("warn", WARN), ("error", COLOR_STOP)):
            self.messages.tag_configure(tag, foreground=colour)

        tools = ttk.Frame(inner, style="Card.TFrame")
        tools.grid(row=0, column=2, rowspan=2, sticky="ne", padx=(16, 0))
        ttk.Button(tools, text="Laden...", command=self._choose,
                   width=14).pack(fill="x")
        ttk.Button(tools, text="Bearbeiten...", command=self._open_editor,
                   width=14).pack(fill="x", pady=(6, 0))
        self.save_button = ttk.Button(tools, text="Speichern", command=self._save,
                                      width=14, state="disabled")
        self.save_button.pack(fill="x", pady=(6, 0))
        ttk.Button(tools, text="Speichern unter...", command=self._save_as,
                   width=14).pack(fill="x", pady=(6, 0))
        ttk.Checkbutton(tools, text="TF senden", variable=self.send_tf,
                        command=self._toggle_tf).pack(anchor="w", pady=(10, 0))

    def _build_body(self):
        """Gelenke links, Frames in der Mitte, Sollwerte rechts."""
        body = ttk.Frame(self, padding=(PAD, PAD, PAD, 0))
        body.grid(row=1, column=0, sticky="nsew")
        body.rowconfigure(0, weight=1)
        body.columnconfigure(0, weight=1, minsize=420)
        body.columnconfigure(1, weight=2, minsize=430)   # Baum braucht Breite
        self.body = body

        left = ttk.Notebook(body)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, PAD))
        self.joint_table = self._make_table(left, JOINT_COLUMNS)
        self.link_table = self._make_table(left, LINK_COLUMNS)
        left.add(self.joint_table.master, text="Gelenke")
        left.add(self.link_table.master, text="Links")

        middle = self._card(body, row=0, column=1, sticky="nsew", padx=(0, PAD))
        middle.rowconfigure(1, weight=1)
        middle.columnconfigure(0, weight=1)
        self.frames_card = middle
        header = ttk.Frame(middle, style="Card.TFrame", padding=(12, 8, 10, 6))
        header.grid(row=0, column=0, sticky="ew")
        ttk.Label(header, text="FRAMES", style="Section.TLabel").pack(side="left")
        ttk.Button(header, text="Ausblenden", style="Mini.TButton",
                   command=self._toggle_frames).pack(side="right")
        ttk.Separator(header, orient="vertical").pack(
            side="right", fill="y", padx=8, pady=2)
        ttk.Button(header, text="Einklappen", style="Mini.TButton",
                   command=lambda: self._frames_open(False)).pack(side="right")
        ttk.Button(header, text="Aufklappen", style="Mini.TButton",
                   command=lambda: self._frames_open(True)).pack(
            side="right", padx=(0, 6))

        self.frame_table = self._make_table(middle, FRAME_COLUMNS, padding=(1, 0),
                                            tree=FRAME_TREE)
        self.frame_table.master.grid(row=1, column=0, sticky="nsew")
        self.frame_table.tag_configure("tcp", background=TINT_OK)
        self.frame_table.tag_configure("orphan", background=TINT_BAD)
        self.frame_table.tag_configure("stripe", background=STRIPE)

        # Schmaler Streifen, der die Karte zurueckholt. Liegt in derselben
        # Spalte und wird nur im ausgeblendeten Zustand eingeblendet.
        self.frames_strip = self._card(body)
        self.frames_strip.columnconfigure(0, weight=1)
        strip_button = tk.Button(
            self.frames_strip, text="\n".join("FRAMES"), command=self._toggle_frames,
            background=SURFACE, foreground=MUTED, activebackground=BUTTON,
            activeforeground=TEXT, font=("TkDefaultFont", 9, "bold"),
            relief="flat", bd=0, highlightthickness=0, cursor="hand2",
            padx=4, pady=10)
        strip_button.grid(row=0, column=0, sticky="new")

        # Die TCP-Lage steht bewusst ausserhalb der Karte: sie soll auch dann
        # sichtbar bleiben, wenn die Frames weggeklappt sind.
        tcp = self._card(body, row=1, column=0, columnspan=2, sticky="ew",
                         pady=(PAD, 0))
        tcp_inner = ttk.Frame(tcp, style="Card.TFrame", padding=(12, 7))
        tcp_inner.pack(fill="x")
        ttk.Label(tcp_inner, textvariable=self.tcp_var, style="Tcp.TLabel").pack(
            side="left")

        right = ttk.Frame(body, width=370)
        right.grid(row=0, column=2, rowspan=2, sticky="ns")
        right.grid_propagate(False)
        right.rowconfigure(0, weight=1)
        right.columnconfigure(0, weight=1)

        self.target_book = ttk.Notebook(right)
        self.target_book.grid(row=0, column=0, sticky="nsew")
        self.cart_tab = tk.Frame(self.target_book, background=SURFACE,
                                 padx=16, pady=14)
        self.joint_tab = tk.Frame(self.target_book, background=SURFACE,
                                  padx=16, pady=14)
        self.target_book.add(self.cart_tab, text="Kartesisch")
        self.target_book.add(self.joint_tab, text="Joint")
        self.target_book.select(self.joint_tab)
        self._build_cart_tab()

        motion = self._card(right, row=1, column=0, sticky="ew", pady=(PAD, 0))
        motion.columnconfigure(0, weight=1)
        inner = ttk.Frame(motion, style="Card.TFrame", padding=(16, 12))
        inner.grid(row=0, column=0, sticky="ew")
        inner.columnconfigure(1, weight=1)
        ttk.Label(inner, text="BEWEGUNG", style="Section.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        for row, (key, label, unit, default) in enumerate(MOTION_FIELDS, start=1):
            ttk.Label(inner, text=label, style="Card.TLabel").grid(
                row=row, column=0, sticky="w", pady=3)
            entry = self._entry(inner, width=9, value=f"{default:g}")
            entry.grid(row=row, column=1, sticky="e", padx=(10, 8), pady=3)
            ttk.Label(inner, text=unit, style="Muted.TLabel", width=5).grid(
                row=row, column=2, sticky="w")
            self.motion_entries[key] = entry
        ttk.Button(inner, text="Uebernehmen", command=self._apply_motion).grid(
            row=len(MOTION_FIELDS) + 1, column=0, columnspan=3, sticky="w",
            pady=(10, 0))

    def _build_cart_tab(self):
        """Zielvorgabe im Basiskoordinatensystem."""
        tab = self.cart_tab
        tab.columnconfigure(1, weight=1)

        ttk.Label(tab, text="ZIEL IM BASISSYSTEM", style="Section.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))

        for row, (key, label, unit) in enumerate(CART_FIELDS, start=1):
            pad = (10, 3) if key == "roll" else (0, 3)
            ttk.Label(tab, text=label, style="Card.TLabel").grid(
                row=row, column=0, sticky="w", pady=pad)
            entry = self._entry(tab, width=12, value="0.000")
            entry.grid(row=row, column=1, sticky="e", padx=(10, 8), pady=pad)
            ttk.Label(tab, text=unit, style="Muted.TLabel", width=4).grid(
                row=row, column=2, sticky="w", pady=pad)
            self.cart_entries[key] = entry

        buttons = tk.Frame(tab, background=SURFACE)
        buttons.grid(row=len(CART_FIELDS) + 1, column=0, columnspan=3,
                     sticky="w", pady=(16, 0))
        go = self._flat_button(buttons, "Go", self._cart_go, COLOR_GO,
                               COLOR_GO_HOVER)
        go.pack(side="left")
        self.go_buttons.append(go)
        ttk.Button(buttons, text="TCP uebernehmen",
                   command=self._cart_from_tcp).pack(side="left", padx=(8, 0))

        note = tk.Label(tab, background=TINT_WARN, foreground=WARN, justify="left",
                        text="Rueckwaertskinematik ist noch nicht hinterlegt. "
                             "Go prueft die Eingabe und meldet das Ziel.",
                        wraplength=290, padx=10, pady=8, anchor="w")
        note.grid(row=len(CART_FIELDS) + 2, column=0, columnspan=3,
                  sticky="ew", pady=(16, 0))

    def _build_joint_inputs(self):
        """Eingabefelder passend zur geladenen URDF neu aufbauen."""
        for child in self.joint_tab.winfo_children():
            child.destroy()
        self.entries.clear()
        self.sliders.clear()
        # Die Go-Taste dieses Tabs wurde eben mit zerstoert, aus der Liste werfen.
        self.go_buttons = [b for b in self.go_buttons if b.winfo_exists()]

        movable = [j for j in self.joints if j["type"] != "fixed"]
        if not movable:
            ttk.Label(self.joint_tab, text="keine beweglichen Gelenke",
                      style="Muted.TLabel").grid(row=0, column=0, sticky="w")
            return

        # Bei vielen Achsen laeuft die Liste sonst unten aus dem Fenster und
        # die Go-Taste waere nicht mehr erreichbar. Liste scrollt, Tasten nicht.
        self.joint_tab.columnconfigure(0, weight=1)
        self.joint_tab.rowconfigure(1, weight=1)
        ttk.Label(self.joint_tab, text="ACHSWERTE", style="Section.TLabel").grid(
            row=0, column=0, sticky="w", pady=(0, 10))

        area = tk.Frame(self.joint_tab, background=SURFACE)
        area.grid(row=1, column=0, sticky="nsew")
        area.rowconfigure(0, weight=1)
        area.columnconfigure(0, weight=1)

        canvas = tk.Canvas(area, background=SURFACE, highlightthickness=0, bd=0)
        canvas.grid(row=0, column=0, sticky="nsew")
        bar = ttk.Scrollbar(area, orient="vertical", command=canvas.yview)
        bar.grid(row=0, column=1, sticky="ns")
        canvas.configure(yscrollcommand=bar.set)

        rows = tk.Frame(canvas, background=SURFACE)
        window = canvas.create_window((0, 0), window=rows, anchor="nw")

        def resized(_event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))
            canvas.itemconfigure(window, width=canvas.winfo_width())
            needed = rows.winfo_reqheight()
            if needed > canvas.winfo_height() + 2:
                bar.grid()
            else:
                bar.grid_remove()
        rows.bind("<Configure>", resized)
        canvas.bind("<Configure>", resized)

        def wheel(event):
            step = -1 if getattr(event, "num", 0) == 4 or event.delta > 0 else 1
            canvas.yview_scroll(step, "units")
        for widget in (canvas, rows):
            widget.bind("<MouseWheel>", wheel)         # Windows und macOS
            widget.bind("<Button-4>", wheel)           # X11
            widget.bind("<Button-5>", wheel)

        rows.columnconfigure(1, weight=1)

        for index, joint in enumerate(movable):
            name = joint["name"]
            row = 2 * index                            # Zeile Wert, darunter Schieber

            ttk.Label(rows, text=name, style="Card.TLabel").grid(
                row=row, column=0, sticky="w")

            entry = self._entry(rows, width=10, value="0.00")
            entry.grid(row=row, column=1, sticky="e", padx=(10, 8))
            entry.bind("<Return>", lambda _event: self._apply())
            entry.bind("<FocusOut>", lambda _event: self._apply())
            self.entries[name] = entry

            ttk.Label(rows, text="deg" if is_rotational(joint) else "m",
                      style="Muted.TLabel", width=4).grid(
                row=row, column=2, sticky="w")

            low, high = slider_range(joint)
            ttk.Label(rows, text=f"{low:.0f} .. {high:.0f}",
                      style="Muted.TLabel").grid(row=row, column=3, sticky="e",
                                                 padx=(8, 0))

            slider = ttk.Scale(rows, from_=low, to=high,
                               orient="horizontal", style="Joint.Horizontal.TScale",
                               command=lambda value, n=name: self._on_slider(n, value))
            slider.grid(row=row + 1, column=0, columnspan=4, sticky="ew",
                        pady=(2, 10), padx=(0, 4))
            # Erst beim Loslassen ins Protokoll, sonst laeuft es beim Ziehen voll.
            slider.bind("<ButtonRelease-1>", lambda _e: self._slider_done())
            self.sliders[name] = slider

        buttons = tk.Frame(self.joint_tab, background=SURFACE)
        buttons.grid(row=2, column=0, sticky="w", pady=(10, 0))
        go = self._flat_button(buttons, "Go", self._apply, COLOR_GO, COLOR_GO_HOVER)
        go.pack(side="left")
        self.go_buttons.append(go)
        ttk.Button(buttons, text="Nullstellung", command=self._reset).pack(
            side="left", padx=(8, 0))
        self._sync_sliders()                           # sonst stehen sie am Anschlag
        self._update_stop_state()

    def _build_foot(self):
        foot = ttk.Frame(self, padding=(PAD + 4, 8))
        foot.grid(row=2, column=0, sticky="ew")
        foot.columnconfigure(1, weight=1)
        ttk.Label(foot, textvariable=self.session_var, style="Foot.TLabel").grid(
            row=0, column=0, sticky="w")
        ttk.Label(foot, textvariable=self.path_var, style="Foot.TLabel").grid(
            row=0, column=1)
        ttk.Label(foot, textvariable=self.refresh_var, style="Foot.TLabel").grid(
            row=0, column=2, sticky="e")

    def _make_table(self, parent, columns, padding=(0, 0), tree=None):
        """Treeview mit Karte drumherum. Der waagerechte Balken erscheint nur,
        wenn die Spalten wirklich breiter sind als das Fenster.

        Mit `tree` als (Ueberschrift, Breite) bekommt die Tabelle die Spalte #0
        und damit Einrueckung samt Klapppfeilen.
        """
        frame = tk.Frame(parent, background=SURFACE, bd=0,
                         highlightthickness=0)
        keys = [c[0] for c in columns]
        table = ttk.Treeview(frame, columns=keys, selectmode="browse",
                             show="tree headings" if tree else "headings")
        if tree:
            table.heading("#0", text=tree[0], anchor="w")
            table.column("#0", width=tree[1], minwidth=110, stretch=False)
        for key, heading, width, anchor in columns:
            table.heading(key, text=heading, anchor=anchor)
            table.column(key, width=width, minwidth=60, anchor=anchor,
                         stretch=(key == keys[-1]))

        yscroll = ttk.Scrollbar(frame, orient="vertical", command=table.yview)
        xscroll = ttk.Scrollbar(frame, orient="horizontal", command=table.xview)
        table.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)

        table.grid(row=0, column=0, sticky="nsew", padx=padding[0], pady=padding[1])
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll.grid(row=1, column=0, sticky="ew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        def fit(_event=None):
            needed = sum(table.column(k, "width") for k in keys)
            if tree:
                needed += table.column("#0", "width")
            if needed > table.winfo_width() + 4:
                xscroll.grid()
            else:
                xscroll.grid_remove()
            if table.yview() == (0.0, 1.0):
                yscroll.grid_remove()
            else:
                yscroll.grid()
        table.bind("<Configure>", fit)
        table.fit = fit
        table.tag_configure("stripe", background=STRIPE)
        return table

    def _insert_row(self, table, index, values, tags=()):
        """Jede zweite Zeile leicht getoent, aber nur ohne eigene Farbe."""
        if not tags:
            tags = ("stripe",) if index % 2 else ()
        table.insert("", "end", tags=tags, values=values)

    # ----------------------------------------------------------------------
    # Meldungsfenster
    # ----------------------------------------------------------------------

    def _log(self, text, level="info"):
        """Zeile ins Meldungsfenster schreiben. Ersetzt die alte Statuszeile."""
        self.messages.configure(state="normal")
        self.messages.insert("end", f"{datetime.now():%H:%M:%S}  ", "time")
        self.messages.insert("end", f"{text}\n", level)
        self.messages.see("end")
        self.messages.configure(state="disabled")

    # ----------------------------------------------------------------------
    # STOP
    # ----------------------------------------------------------------------

    def _toggle_stop(self):
        """STOP sperrt die Sollwertvorgabe. TF laeuft weiter, das Bild bleibt stehen."""
        self.stopped = not self.stopped
        if self.stopped:
            colour, hover, text = COLOR_GO, COLOR_GO_HOVER, "FREI"
            self._log("STOP ausgeloest, Sollwerte werden nicht mehr uebernommen",
                      "error")
        else:
            colour, hover, text = COLOR_STOP, COLOR_STOP_HOVER, "STOP"
            self._log("STOP aufgehoben", "ok")
        self.stop_button.base_colour, self.stop_button.hover_colour = colour, hover
        self.stop_button.configure(text=text, background=colour,
                                   activebackground=hover)
        self._update_stop_state()

    def _update_stop_state(self):
        state = "disabled" if self.stopped else "normal"
        for entry in self.entries.values():
            entry.configure(state=state)
        for entry in self.cart_entries.values():
            entry.configure(state=state)
        for slider in self.sliders.values():
            slider.configure(state=state)
        for button in self.go_buttons:
            if button.winfo_exists():
                button.configure(state=state,
                                 background=DISABLED_BG if self.stopped
                                 else button.base_colour)

    # ----------------------------------------------------------------------
    # Daten laden und speichern
    # ----------------------------------------------------------------------

    def _choose(self):
        path = filedialog.askopenfilename(
            title="URDF waehlen",
            initialdir=os.path.dirname(self.urdf_path) or ".",
            filetypes=[("URDF", "*.urdf *.xml"), ("Alle Dateien", "*.*")])
        if path:
            self._load(path)

    def _load(self, path, initial=False):
        try:
            root, name, links, joints, xml_text = parse(path)
        except FileNotFoundError:
            messagebox.showerror("URDF", f"Datei nicht gefunden:\n{path}")
            self._log(f"Datei nicht gefunden: {path}", "error")
            if initial:
                self._choose()
            return
        except ET.ParseError as exc:
            messagebox.showerror("URDF", f"XML fehlerhaft:\n{exc}")
            self._log(f"XML fehlerhaft: {exc}", "error")
            return

        self.urdf_path = path
        self.xml_root = root
        self.robot_name, self.links, self.joints, self.urdf_xml = name, links, joints, xml_text
        self.path_var.set(os.path.basename(path))
        self.positions = {j["name"]: 0.0 for j in joints if j["type"] != "fixed"}
        self._set_dirty(False)

        self._update_header()
        self._build_joint_inputs()
        self._fill_link_table()
        self._update_kinematics()
        if self.editor is not None:
            self._edit_fill_combo()

        self._log(f"Roboter {name} initialisiert, {len(links)} Links, "
                  f"{len(joints)} Gelenke", "ok")

        problems = validate_root(root)
        if problems:
            for problem in problems:
                self._log(problem, "warn")
            messagebox.showwarning("URDF", "Hinweise:\n\n" + "\n".join(problems))

        message = self.ros.set_model(joints, xml_text)
        if self.ros.running:                           # sonst steht der Hinweis
            self._log(message, "ok")                   # vom Start schon da
        self.ros.set_positions(self.positions)

    def _save(self):
        if not self.dirty:
            return
        try:
            backup = save_urdf(self.urdf_path, self.urdf_xml)
        except OSError as exc:
            messagebox.showerror("Speichern", f"Schreiben fehlgeschlagen:\n{exc}")
            self._log(f"Speichern fehlgeschlagen: {exc}", "error")
            return
        self._set_dirty(False)
        self._log(f"Gespeichert. Sicherung: {backup}" if backup
                  else "Gespeichert.", "ok")

    def _save_as(self):
        """Aktuellen Stand als neue Datei ablegen, Original bleibt unberuehrt.

        Danach zeigt die GUI auf die neue Datei: weitere Aenderungen landen
        dort und nicht mehr im Ausgangsmodell.
        """
        folder = backup_dir_for(self.urdf_path)
        path = filedialog.asksaveasfilename(
            title="URDF speichern unter",
            initialdir=folder,
            initialfile=timestamped_name(self.urdf_path),
            defaultextension=".urdf",
            filetypes=[("URDF", "*.urdf *.xml"), ("Alle Dateien", "*.*")])
        if not path:
            return

        try:
            save_urdf(path, self.urdf_xml)
        except OSError as exc:
            messagebox.showerror("Speichern", f"Schreiben fehlgeschlagen:\n{exc}")
            self._log(f"Speichern fehlgeschlagen: {exc}", "error")
            return

        self.urdf_path = path                          # ab jetzt gilt diese Datei
        self.path_var.set(os.path.basename(path))
        self._set_dirty(False)
        self._log(f"Gespeichert unter {path}", "ok")

    def _discard(self):
        if self.dirty and not messagebox.askyesno(
                "Neu laden", "Nicht gespeicherte Aenderungen verwerfen?"):
            return
        self._load(self.urdf_path)

    def _set_dirty(self, dirty):
        self.dirty = dirty
        self.save_button.configure(state="normal" if dirty else "disabled")
        self.title(f"URDF Viewer{'  *' if dirty else ''}")

    def _update_header(self):
        roots = root_links(self.links, self.joints)
        movable = sum(1 for j in self.joints if j["type"] != "fixed")
        self.robot_var.set(self.robot_name or "kein Modell")
        self.info_var.set(
            f"{len(self.links)} Links   |   {len(self.joints)} Gelenke, "
            f"{movable} beweglich   |   Wurzel "
            f"{roots[0] if len(roots) == 1 else roots}")

    # ----------------------------------------------------------------------
    # Tabellen
    # ----------------------------------------------------------------------

    def _fill_joint_table(self):
        """Gelenke mit dem aktuellen Sollwert in rad und grad."""
        self.joint_table.delete(*self.joint_table.get_children())
        for index, joint in enumerate(self.joints):
            value = self.positions.get(joint["name"], 0.0)
            if joint["type"] == "fixed":
                primary = secondary = "-"
            elif is_rotational(joint):
                primary = f"{value:.4f}"
                secondary = f"{math.degrees(value):.2f}"
            else:
                primary = f"{value:.4f}"
                secondary = f"{value * 1000.0:.1f}"

            if joint["has_limit"]:
                lo, up = joint["lower"], joint["upper"]
                if is_rotational(joint):
                    limit = f"{math.degrees(lo):.0f} .. {math.degrees(up):.0f}"
                else:
                    limit = f"{lo:.3f} .. {up:.3f}"
            elif joint["type"] == "continuous":
                limit = "unbegrenzt"
            else:
                limit = "-"

            self._insert_row(self.joint_table, index, (
                joint["name"], joint["type"], primary, secondary, limit,
                f"{joint['parent']} > {joint['child']}"))
        self.after_idle(self.joint_table.fit)

    def _fill_link_table(self):
        self.link_table.delete(*self.link_table.get_children())
        for index, link in enumerate(self.links):
            if link["has_inertial"]:
                mass, com = f"{link['mass']:.4f}", fmt(link["com"])
            else:
                mass = com = "-"
            self._insert_row(self.link_table, index, (link["name"], mass, com))
        self.after_idle(self.link_table.fit)

    # ----------------------------------------------------------------------
    # Sollwerte
    # ----------------------------------------------------------------------

    def _apply(self):
        """Eingaben lesen, pruefen, begrenzen und uebernehmen."""
        if self.stopped:
            return
        invalid = 0
        clamped = 0
        for joint in self.joints:
            entry = self.entries.get(joint["name"])
            if entry is None:
                continue

            try:
                typed = float(entry.get().replace(",", "."))
            except ValueError:
                entry.configure(background=BG_INVALID)
                invalid += 1
                continue

            value = math.radians(typed) if is_rotational(joint) else typed
            if joint["has_limit"]:
                limited = min(max(value, joint["lower"]), joint["upper"])
            else:
                limited = value

            if abs(limited - value) > 1e-12:
                clamped += 1
                entry.configure(background=BG_CLAMPED)
                shown = math.degrees(limited) if is_rotational(joint) else limited
                entry.delete(0, "end")
                entry.insert(0, f"{shown:.2f}")
            else:
                entry.configure(background=BG_OK)

            self.positions[joint["name"]] = limited

        self._sync_sliders()
        self._update_kinematics()
        self.ros.set_positions(self.positions)

        if invalid:
            self._log(f"{invalid} Eingabe(n) nicht lesbar, Wert unveraendert", "error")
        elif clamped:
            self._log(f"{clamped} Wert(e) auf das URDF-Limit begrenzt", "warn")
        else:
            self._log("Moved to  " + self._pose_text())

    def _joint_by_name(self, name):
        for joint in self.joints:
            if joint["name"] == name:
                return joint
        return None

    def _on_slider(self, name, value):
        """Laeuft bei jedem Pixel Zugweg. Muss deshalb billig bleiben."""
        if self.syncing or self.stopped:
            return
        joint = self._joint_by_name(name)
        if joint is None:
            return

        shown = float(value)
        entry = self.entries[name]
        entry.configure(background=BG_OK)
        entry.delete(0, "end")
        entry.insert(0, f"{shown:.2f}")

        # Der Schieber kann seine Enden nicht verlassen, ein Begrenzen
        # waere hier also nur Zierde.
        self.positions[name] = (math.radians(shown) if is_rotational(joint)
                                else shown)
        self.ros.set_positions(self.positions)         # TF sofort, ohne Umweg
        self._schedule_refresh()

    def _schedule_refresh(self):
        """Tabellen hoechstens alle SLIDER_REFRESH_MS neu rechnen."""
        if self.refresh_job is None:
            self.refresh_job = self.after(SLIDER_REFRESH_MS, self._refresh_now)

    def _refresh_now(self):
        self.refresh_job = None
        self._update_kinematics()

    def _slider_done(self):
        """Maustaste losgelassen: einmal sauber nachziehen und melden."""
        if self.stopped:
            return
        if self.refresh_job is not None:
            self.after_cancel(self.refresh_job)
            self.refresh_job = None
        self._update_kinematics()
        self._log("Moved to  " + self._pose_text())

    def _sync_sliders(self):
        """Schieber auf die aktuellen Sollwerte setzen, ohne sie auszuloesen."""
        self.syncing = True
        try:
            for joint in self.joints:
                slider = self.sliders.get(joint["name"])
                if slider is None:
                    continue
                value = self.positions.get(joint["name"], 0.0)
                shown = math.degrees(value) if is_rotational(joint) else value
                low, high = slider_range(joint)
                slider.set(min(max(shown, low), high))
        finally:
            self.syncing = False

    def _reset(self):
        for entry in self.entries.values():
            entry.configure(background=BG_OK)
            entry.delete(0, "end")
            entry.insert(0, "0.00")
        self._apply()

    def _sync_joint_entries(self):
        """Eingabefelder auf die aktuellen Sollwerte setzen."""
        for joint in self.joints:
            entry = self.entries.get(joint["name"])
            if entry is None:
                continue
            value = self.positions.get(joint["name"], 0.0)
            shown = math.degrees(value) if is_rotational(joint) else value
            entry.delete(0, "end")
            entry.insert(0, f"{shown:.2f}")
        self._sync_sliders()

    def _cart_from_tcp(self):
        """Zielfelder mit der aktuellen TCP-Lage vorbelegen."""
        pose = self._tcp_pose()
        if pose is None:
            self._log("Kein eindeutiger Endeffektor, nichts uebernommen", "warn")
            return
        xyz, rpy = pose
        for key, value in zip(("x", "y", "z"), xyz):
            self.cart_entries[key].delete(0, "end")
            self.cart_entries[key].insert(0, f"{value:.4f}")
        for key, value in zip(("roll", "pitch", "yaw"), rpy):
            self.cart_entries[key].delete(0, "end")
            self.cart_entries[key].insert(0, f"{math.degrees(value):.2f}")

    def _cart_go(self):
        """Kartesisches Ziel pruefen. Die Rueckwaertskinematik fehlt noch."""
        if self.stopped:
            return
        target = {}
        for key, label, _unit in CART_FIELDS:
            entry = self.cart_entries[key]
            try:
                target[key] = float(entry.get().replace(",", "."))
            except ValueError:
                entry.configure(background=BG_INVALID)
                self._log(f"{label} ist nicht lesbar", "error")
                return
            entry.configure(background=BG_OK)

        self._log("Ziel kartesisch   X {x:.4f}  Y {y:.4f}  Z {z:.4f} m   "
                  "R {roll:.2f}  P {pitch:.2f}  Y {yaw:.2f} deg".format(**target))
        self._log("Rueckwaertskinematik nicht hinterlegt, keine Achswerte berechnet",
                  "warn")

    def _apply_motion(self):
        """Jerk, Beschleunigung und Geschwindigkeit uebernehmen."""
        values = {}
        for key, label, unit, _default in MOTION_FIELDS:
            entry = self.motion_entries[key]
            try:
                value = float(entry.get().replace(",", "."))
            except ValueError:
                entry.configure(background=BG_INVALID)
                self._log(f"{label} ist nicht lesbar", "error")
                return
            if value <= 0.0:
                entry.configure(background=BG_INVALID)
                self._log(f"{label} muss groesser als null sein", "error")
                return
            entry.configure(background=BG_OK)
            values[key] = value

        self.motion = values
        self._log("Bewegung: Jerk {jerk:g}, Acceleration {acceleration:g}, "
                  "Speed {speed:g}".format(**values), "ok")

    # ----------------------------------------------------------------------
    # Kinematik
    # ----------------------------------------------------------------------

    def _tcp_pose(self):
        """Lage des einzigen Endeffektors oder None."""
        poses = forward_kinematics(self.links, self.joints, self.positions)
        roots = root_links(self.links, self.joints)
        base = roots[0] if roots else "?"
        tcps = [n for n in leaf_links(self.links, self.joints) if n != base]
        if len(tcps) != 1 or tcps[0] not in poses:
            return None
        return to_xyz_rpy(poses[tcps[0]])

    def _pose_text(self, pad=True):
        """TCP-Lage als Text. Mit pad stehen die Spalten im Protokoll untereinander."""
        pose = self._tcp_pose()
        if pose is None:
            return "Achswerte uebernommen"
        xyz, rpy = pose
        width, digits = (8, 2) if pad else (0, 1)
        return (f"X {xyz[0]:{width}.4f} Y {xyz[1]:{width}.4f} Z {xyz[2]:{width}.4f} m   "
                f"R {math.degrees(rpy[0]):{width}.{digits}f} "
                f"P {math.degrees(rpy[1]):{width}.{digits}f} "
                f"Y {math.degrees(rpy[2]):{width}.{digits}f} deg")

    def _frame_parents(self):
        """{linkname: elternname}. Leer, wenn der Link keinen Vater hat."""
        parents = {l["name"]: "" for l in self.links}
        for joint in self.joints:
            if joint["child"] in parents and joint["parent"]:
                parents[joint["child"]] = joint["parent"]
        return parents

    def _sync_frame_tree(self):
        """Baumgeruest nur neu bauen, wenn sich die Topologie geaendert hat.

        Beim Ziehen am Schieber laeuft das hier bis zu 16 Mal in der Sekunde.
        Wuerde jedes Mal geloescht und neu eingefuegt, klappten alle Zweige
        wieder auf und die Auswahl ginge verloren.
        """
        parents = self._frame_parents()
        if parents == self.frame_layout:
            return

        self.frame_table.delete(*self.frame_table.get_children())
        self.frame_items = {}
        depth = {}

        # Eltern muessen vor ihren Kindern eingefuegt sein.
        pending = [l["name"] for l in self.links]
        while pending:
            progressed = False
            for name in list(pending):
                parent = parents.get(name, "")
                if parent and parent in pending:
                    continue                           # Vater kommt noch
                target = self.frame_items.get(parent, "") if parent else ""
                depth[name] = depth.get(parent, -1) + 1 if target else 0
                self.frame_items[name] = self.frame_table.insert(
                    target, "end", text=name, open=True)
                pending.remove(name)
                progressed = True
            if not progressed:                         # Zyklus, Rest flach anhaengen
                for name in pending:
                    depth[name] = 0
                    self.frame_items[name] = self.frame_table.insert(
                        "", "end", text=name, open=True)
                break

        self._fit_tree_column(depth)
        self.frame_layout = parents

    def _fit_tree_column(self, depth):
        """Baumspalte so breit, dass auch der tiefste Name lesbar bleibt.

        Eine lange serielle Kette rueckt weit ein; mit fester Breite waeren
        die letzten Glieder nur noch als Stummel zu sehen.
        """
        font = tkfont.nametofont("TkDefaultFont")
        needed = max((28 + level * self.tree_indent + font.measure(name)
                      for name, level in depth.items()), default=0) + 14
        self.frame_table.column(
            "#0", width=max(FRAME_TREE[1], min(needed, TREE_MAX_WIDTH)))

    def _toggle_frames(self):
        """Frames-Karte weg- oder wieder einblenden.

        Die Spalte behaelt einen schmalen Streifen zum Zurueckholen. Ihr
        Gewicht faellt auf null, damit der frei werdende Platz an die
        Gelenkliste geht und nicht als Luecke stehen bleibt.
        """
        self.frames_shown = not self.frames_shown
        if self.frames_shown:
            self.frames_strip.grid_remove()
            self.body.columnconfigure(1, weight=2, minsize=430)
            self.frames_card.grid()
            self.after_idle(self.frame_table.fit)
        else:
            self.frames_card.grid_remove()
            self.body.columnconfigure(1, weight=0, minsize=0)
            self.frames_strip.grid(row=0, column=1, sticky="ns", padx=(0, PAD))

    def _frames_open(self, opened):
        """Alle Zweige auf- oder zuklappen."""
        for item in self.frame_items.values():
            self.frame_table.item(item, open=opened)

    def _update_kinematics(self):
        """FK fuer alle Frames neu rechnen und die Tabellen fuellen."""
        poses = forward_kinematics(self.links, self.joints, self.positions)
        leaves = set(leaf_links(self.links, self.joints))
        roots = root_links(self.links, self.joints)
        base = roots[0] if roots else "?"

        self._sync_frame_tree()
        for link in self.links:
            name = link["name"]
            item = self.frame_items.get(name)
            if item is None:
                continue
            matrix = poses.get(name)
            if matrix is None:                         # nicht mit der Basis verbunden
                self.frame_table.item(item, values=["-"] * 6, tags=("orphan",))
                continue

            xyz, rpy = to_xyz_rpy(matrix)
            tags = ("tcp",) if name in leaves and name != base else ()
            self.frame_table.item(item, tags=tags, values=(
                f"{xyz[0]:.4f}", f"{xyz[1]:.4f}", f"{xyz[2]:.4f}",
                f"{math.degrees(rpy[0]):.2f}",
                f"{math.degrees(rpy[1]):.2f}",
                f"{math.degrees(rpy[2]):.2f}"))

        self._fill_joint_table()
        self.after_idle(self.frame_table.fit)

        tcps = [n for n in leaves if n != base]
        if len(tcps) == 1 and tcps[0] in poses:
            self.tcp_var.set(f"TCP {tcps[0]}   {self._pose_text(pad=False)}")
        elif tcps:
            self.tcp_var.set(f"Mehrere Endeffektoren: {', '.join(sorted(tcps))}")
        else:
            self.tcp_var.set("Kein Endeffektor gefunden")

    # ----------------------------------------------------------------------
    # Bearbeiten in einem eigenen Fenster
    # ----------------------------------------------------------------------

    def _open_editor(self):
        """Nur Laengen, Winkel und Gelenkgrenzen sind aenderbar.

        Achse, Typ sowie parent/child bleiben bewusst gesperrt: sie definieren
        die Topologie und gehoeren in den CAD-Export, nicht in eine
        Bedienoberflaeche.
        """
        if self.editor is not None and self.editor.winfo_exists():
            self.editor.lift()
            return

        window = tk.Toplevel(self)
        window.title("URDF bearbeiten")
        window.configure(background=BG)
        window.transient(self)
        window.resizable(False, False)
        window.protocol("WM_DELETE_WINDOW", self._close_editor)
        self.editor = window

        card = self._card(window)
        card.pack(fill="both", expand=True, padx=PAD, pady=PAD)
        tab = ttk.Frame(card, style="Card.TFrame", padding=(18, 16))
        tab.pack(fill="both", expand=True)
        self.edit_entries = {}

        head = ttk.Frame(tab, style="Card.TFrame")
        head.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 16))
        ttk.Label(head, text="Gelenk", style="Card.TLabel").pack(side="left")
        self.edit_combo = ttk.Combobox(head, textvariable=self.edit_joint_var,
                                       state="readonly", width=24)
        self.edit_combo.pack(side="left", padx=(10, 16))
        self.edit_combo.bind("<<ComboboxSelected>>", lambda _e: self._edit_load_joint())
        ttk.Label(head, textvariable=self.edit_info_var, style="Muted.TLabel").pack(
            side="left")

        def field_row(row, title, specs):
            ttk.Label(tab, text=title, style="Section.TLabel").grid(
                row=row, column=0, sticky="w", pady=(8, 4))
            holder = ttk.Frame(tab, style="Card.TFrame")
            holder.grid(row=row, column=1, columnspan=3, sticky="w", pady=(8, 4))
            for key, label in specs:
                ttk.Label(holder, text=label, style="Muted.TLabel").pack(
                    side="left", padx=(0, 6))
                entry = self._entry(holder, width=11)
                entry.pack(side="left", padx=(0, 16))
                self.edit_entries[key] = entry

        field_row(1, "LAENGE  [m]", [("x", "X"), ("y", "Y"), ("z", "Z")])
        field_row(2, "WINKEL  [deg]", [("roll", "Roll"), ("pitch", "Pitch"),
                                       ("yaw", "Yaw")])
        field_row(3, "AUSLENKUNG", [("lower", "min"), ("upper", "max")])
        ttk.Label(tab, textvariable=self.edit_unit_var, style="Muted.TLabel").grid(
            row=3, column=1, columnspan=3, sticky="e", padx=(0, 20))

        actions = ttk.Frame(tab, style="Card.TFrame")
        actions.grid(row=4, column=0, columnspan=4, sticky="w", pady=(20, 0))
        ttk.Button(actions, text="Uebernehmen", command=self._edit_apply).pack(
            side="left")
        ttk.Button(actions, text="Feld zuruecksetzen",
                   command=self._edit_load_joint).pack(side="left", padx=(8, 0))

        ttk.Separator(tab, orient="horizontal").grid(
            row=5, column=0, columnspan=4, sticky="ew", pady=18)

        files = ttk.Frame(tab, style="Card.TFrame")
        files.grid(row=6, column=0, columnspan=4, sticky="w")
        ttk.Button(files, text="Speichern", command=self._save).pack(side="left")
        ttk.Button(files, text="Speichern unter...",
                   command=self._save_as).pack(side="left", padx=(8, 0))
        ttk.Button(files, text="Datei neu laden",
                   command=self._discard).pack(side="left", padx=(8, 0))
        ttk.Label(files, text=f"Ablage: {BACKUP_DIR}",
                  style="Muted.TLabel").pack(side="left", padx=(16, 0))

        self._edit_fill_combo()

    def _close_editor(self):
        if self.editor is not None:
            self.editor.destroy()
        self.editor = None
        self.edit_entries = {}

    def _edit_fill_combo(self):
        names = [j["name"] for j in self.joints]
        self.edit_combo.configure(values=names)
        if names:
            if self.edit_joint_var.get() not in names:
                self.edit_joint_var.set(names[0])
            self._edit_load_joint()

    def _edit_current_joint(self):
        name = self.edit_joint_var.get()
        for joint in self.joints:
            if joint["name"] == name:
                return joint
        return None

    def _edit_set(self, key, value, enabled=True):
        entry = self.edit_entries[key]
        entry.configure(state="normal", background=BG_OK)
        entry.delete(0, "end")
        entry.insert(0, value)
        if not enabled:
            entry.configure(state="disabled")

    def _edit_load_joint(self):
        """Felder aus dem aktuellen Stand des Modells fuellen."""
        if not self.edit_entries:
            return
        joint = self._edit_current_joint()
        if joint is None:
            return

        for axis_key, index in (("x", 0), ("y", 1), ("z", 2)):
            self._edit_set(axis_key, f"{joint['xyz'][index]:.5f}")
        for angle_key, index in (("roll", 0), ("pitch", 1), ("yaw", 2)):
            self._edit_set(angle_key, f"{math.degrees(joint['rpy'][index]):.3f}")

        if joint["has_limit"]:
            factor = math.degrees if is_rotational(joint) else (lambda v: v)
            self._edit_set("lower", f"{factor(joint['lower']):.3f}")
            self._edit_set("upper", f"{factor(joint['upper']):.3f}")
            self.edit_unit_var.set("deg" if is_rotational(joint) else "m")
        else:
            reason = ("continuous: kein Anschlag"
                      if joint["type"] == "continuous" else "fest: keine Auslenkung")
            self._edit_set("lower", "-", enabled=False)
            self._edit_set("upper", "-", enabled=False)
            self.edit_unit_var.set(reason)

        self.edit_info_var.set(
            f"{joint['type']}   {joint['parent']} -> {joint['child']}   "
            f"Achse [{fmt(joint['axis'], 1)}]")

    def _edit_read(self, key):
        """Feldwert als float. Faerbt bei Fehler und wirft ValueError."""
        entry = self.edit_entries[key]
        try:
            value = float(entry.get().replace(",", "."))
        except ValueError:
            entry.configure(background=BG_INVALID)
            raise
        entry.configure(background=BG_OK)
        return value

    def _edit_apply(self):
        """Werte in den XML-Baum schreiben, pruefen und live uebernehmen."""
        joint = self._edit_current_joint()
        if joint is None or self.xml_root is None:
            return

        try:
            xyz = [self._edit_read(k) for k in ("x", "y", "z")]
            rpy = [math.radians(self._edit_read(k)) for k in ("roll", "pitch", "yaw")]
            if joint["has_limit"]:
                factor = math.radians if is_rotational(joint) else (lambda v: v)
                lower = factor(self._edit_read("lower"))
                upper = factor(self._edit_read("upper"))
            else:
                lower = upper = None
        except ValueError:
            self._log("Eingabe nicht lesbar, nichts geaendert", "error")
            return

        # Auf einer Kopie arbeiten und erst nach der Pruefung tauschen.
        draft = copy.deepcopy(self.xml_root)
        element = next((j for j in draft.findall("joint")
                        if j.get("name") == joint["name"]), None)
        if element is None:
            self._log(f"Gelenk {joint['name']} nicht mehr im Baum", "error")
            return

        origin = element.find("origin")
        if origin is None:
            origin = ET.SubElement(element, "origin")
        origin.set("xyz", " ".join(f"{v:.6g}" for v in xyz))
        origin.set("rpy", " ".join(f"{v:.6g}" for v in rpy))

        if lower is not None:
            limit = element.find("limit")
            if limit is None:
                limit = ET.SubElement(element, "limit")
                limit.set("effort", "100")             # in der URDF Pflichtangaben
                limit.set("velocity", "1.0")
            limit.set("lower", f"{lower:.6g}")
            limit.set("upper", f"{upper:.6g}")

        problems = validate_root(draft)
        if problems:
            messagebox.showerror("URDF", "Nicht uebernommen:\n\n" + "\n".join(problems))
            for problem in problems:
                self._log(problem, "error")
            return

        self.xml_root = draft
        self._rebuild_from_root()
        self._set_dirty(True)
        self._log(f"{joint['name']} geaendert, noch nicht gespeichert", "warn")

    def _rebuild_from_root(self):
        """Modell, Tabellen, FK und TF-Ausgabe an den geaenderten Baum anpassen."""
        self.robot_name, self.links, self.joints = parse_root(self.xml_root)
        self.urdf_xml = serialize(self.xml_root)

        # Sollwerte behalten, aber an eventuell engere Grenzen anpassen.
        for joint in self.joints:
            if joint["name"] in self.positions and joint["has_limit"]:
                value = self.positions[joint["name"]]
                self.positions[joint["name"]] = min(max(value, joint["lower"]),
                                                    joint["upper"])

        self._update_header()
        self._build_joint_inputs()
        self._sync_joint_entries()
        self._fill_link_table()
        self._update_kinematics()
        self._edit_load_joint()

        self.ros.set_model(self.joints, self.urdf_xml)
        self.ros.set_positions(self.positions)

    # ----------------------------------------------------------------------
    # ROS und Fusszeile
    # ----------------------------------------------------------------------

    def _start_ros(self):
        error = self.ros.start()
        if error:
            self._log(error, "warn")

    def _toggle_tf(self):
        self.ros.enabled = self.send_tf.get()
        if not self.send_tf.get():
            self._log("TF-Ausgabe angehalten. RViz verliert nach der Cache-Zeit "
                      "von etwa 10 Sekunden alle Frames.", "warn")
        else:
            self._log("TF-Ausgabe laeuft wieder", "ok")

    def _tick(self):
        """Fusszeile aktualisieren. Laeuft im GUI-Thread, greift nie auf ROS zu."""
        elapsed = time.monotonic() - self.session_start
        whole = int(elapsed)
        self.session_var.set(
            f"Session  {whole // 3600:02d}:{whole // 60 % 60:02d}:"
            f"{whole % 60:02d}:{int((elapsed - whole) * 1000):03d}")

        last = self.ros.last_publish
        if not self.ros.running:
            self.refresh_var.set("Last Refresh  --   kein ROS")
        elif not self.send_tf.get():
            self.refresh_var.set("Last Refresh  --   angehalten")
        elif last is None:
            self.refresh_var.set("Last Refresh  --   wartet")
        else:
            age_ms = (time.monotonic() - last) * 1000.0
            cycle = self.ros.cycle_ms
            takt = f"   Takt {cycle:5.1f} ms" if cycle is not None else ""
            self.refresh_var.set(f"Last Refresh {age_ms:6.1f} ms{takt}")

        self.after(FOOTER_TICK_MS, self._tick)

    def _on_close(self):
        if self.dirty and not messagebox.askyesno(
                "Beenden", "Es gibt nicht gespeicherte Aenderungen. Trotzdem beenden?"):
            return
        self.ros.shutdown()
        self.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", nargs="?", default=URDF_PATH)
    args, _ = parser.parse_known_args()

    app = UrdfViewer(args.urdf)
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
