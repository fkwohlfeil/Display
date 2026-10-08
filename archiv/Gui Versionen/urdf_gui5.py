#!/usr/bin/env python3
"""URDF anzeigen, bearbeiten, Achsen vorgeben, Vorwaertskinematik rechnen, TF senden.

Bewegliche Gelenke gehen zyklisch auf /tf, feste einmalig auf /tf_static.
Zusaetzlich wird robot_description latched gesendet, damit RViz das Modell findet.
Im Tab "Bearbeiten" lassen sich Laengen, Winkel und Gelenkgrenzen aendern;
uebernommene Aenderungen wirken sofort, geschrieben wird erst per Speichern.

Aufruf:
    ./urdf_gui5.py                       # Standardpfad aus URDF_PATH
    ./urdf_gui5.py ~/Display/bmKinematic.urdf
"""

import argparse
import copy
import math
import os
import shutil
import sys
import tempfile
import threading
import time
import tkinter as tk
import xml.etree.ElementTree as ET
from datetime import datetime
from tkinter import ttk, filedialog, messagebox

# ---------- hier URDF eintragen ----------
URDF_PATH = "/home/fwohlfeil/simple_robot.urdf"
# -----------------------------------------

TF_RATE_HZ = 30.0
FOOTER_TICK_MS = 100

BG_OK = "white"
BG_CLAMPED = "#fff3c4"
BG_INVALID = "#ffd6d6"


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


def save_urdf(path, xml_text):
    """Atomar schreiben, vorher eine Sicherungskopie anlegen."""
    backup = None
    if os.path.exists(path):
        backup = f"{path}.{datetime.now():%Y%m%d_%H%M%S}.bak"
        shutil.copy2(path, backup)

    handle_fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path) or ".", suffix=".tmp")
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
        self._movable = []                             # bewegliche Gelenke fuer /tf
        self._positions = {}
        self._executor = None
        self._thread = None
        self._timer = None
        self._broadcaster = None
        self._static_broadcaster = None
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
        """Neues Modell uebernehmen: statische Frames und Beschreibung senden."""
        if not self.running:
            return self.error or "Kein ROS-Node aktiv"

        from std_msgs.msg import String
        from tf2_ros import StaticTransformBroadcaster

        # Alten statischen Broadcaster verwerfen, sonst latcht er Frames einer
        # zuvor geladenen URDF weiter, die es gar nicht mehr gibt.
        if self._static_broadcaster is not None:
            old = getattr(self._static_broadcaster, "pub_tf", None)
            if old is not None:
                self.node.destroy_publisher(old)
        self._static_broadcaster = StaticTransformBroadcaster(self.node)

        usable = [j for j in joints if j["parent"] and j["child"]]
        fixed = [j for j in usable if j["type"] == "fixed"]
        movable = [j for j in usable if j["type"] != "fixed"]

        if fixed:
            stamp = self.node.get_clock().now().to_msg()
            self._static_broadcaster.sendTransform(
                [self._message(j, j["origin"], stamp) for j in fixed])

        self._description_pub.publish(String(data=urdf_xml))

        with self._lock:
            self._movable = movable
            self._positions = {j["name"]: 0.0 for j in movable}
        self.tick_count = 0

        return (f"{len(fixed)} feste Frames auf /tf_static, "
                f"{len(movable)} bewegliche auf /tf mit {TF_RATE_HZ:.0f} Hz")

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

    def _on_timer(self):
        """Laeuft im Executor-Thread. Nur lesen, was unter dem Lock geholt wurde."""
        if not self.enabled:
            return
        with self._lock:
            movable = list(self._movable)
            positions = dict(self._positions)
        if not movable:
            return

        stamp = self.node.get_clock().now().to_msg()
        self._broadcaster.sendTransform([
            self._message(j, local_transform(j, positions.get(j["name"], 0.0)), stamp)
            for j in movable
        ])

        now = time.monotonic()
        if self.last_publish is not None:
            delta_ms = (now - self.last_publish) * 1000.0
            self.cycle_ms = (delta_ms if self.cycle_ms is None
                             else 0.9 * self.cycle_ms + 0.1 * delta_ms)
        self.last_publish = now
        self.tick_count += 1


# ==========================================================================
# GUI
# ==========================================================================

FRAME_COLUMNS = [
    ("link",     "Frame",            130),
    ("via",      "ueber Gelenk",     115),
    ("x",        "X [m]",             85),
    ("y",        "Y [m]",             85),
    ("z",        "Z [m]",             85),
    ("roll",     "Roll [deg]",        85),
    ("pitch",    "Pitch [deg]",       85),
    ("yaw",      "Yaw [deg]",         85),
    ("distance", "Abstand [m]",       90),
]

JOINT_COLUMNS = [
    ("name",     "Name",              125),
    ("type",     "Typ",                80),
    ("parent",   "Parent",            105),
    ("child",    "Child",             105),
    ("xyz",      "Origin xyz [m]",    165),
    ("rpy",      "Origin rpy [rad]",  165),
    ("axis",     "Achse",              85),
    ("limit",    "Limit [deg]",       125),
    ("effort",   "Effort",             70),
    ("velocity", "Velocity",           70),
]

LINK_COLUMNS = [
    ("name", "Name",        180),
    ("mass", "Masse [kg]",  100),
    ("com",  "COM xyz [m]", 180),
]


class UrdfViewer(tk.Tk):

    def __init__(self, urdf_path):
        super().__init__()
        self.title("URDF Viewer")
        self.geometry("1280x680")
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.urdf_path = urdf_path
        self.robot_name = ""
        self.xml_root = None                           # bearbeiteter Baum, die Wahrheit
        self.links = []
        self.joints = []
        self.urdf_xml = ""
        self.dirty = False                             # geaendert, aber nicht gespeichert
        self.positions = {}                            # {gelenkname: rad bzw. m}
        self.entries = {}                              # {gelenkname: tk.Entry}
        self.edit_entries = {}                         # Felder im Bearbeiten-Tab

        self.path_var = tk.StringVar(value=urdf_path)
        self.info_var = tk.StringVar(value="")
        self.tcp_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="")
        self.session_var = tk.StringVar(value="Sitzung 00:00:00")
        self.refresh_var = tk.StringVar(value="Last Refresh --")
        self.edit_joint_var = tk.StringVar(value="")
        self.edit_info_var = tk.StringVar(value="")
        self.edit_unit_var = tk.StringVar(value="")
        self.send_tf = tk.BooleanVar(value=True)

        self.session_start = time.monotonic()

        self.ros = RosBridge()

        self._build_ui()
        self._start_ros()
        self._load(urdf_path, initial=True)
        self._tick()

    # -- Aufbau ------------------------------------------------------------

    def _build_ui(self):
        top = ttk.Frame(self, padding=(10, 8))
        top.pack(fill="x")

        ttk.Label(top, text="Datei:").grid(row=0, column=0, sticky="w")
        ttk.Label(top, textvariable=self.path_var, foreground="#444").grid(
            row=0, column=1, sticky="w", padx=(6, 12))
        ttk.Button(top, text="Laden...", command=self._choose).grid(row=0, column=2)
        ttk.Checkbutton(top, text="TF senden", variable=self.send_tf,
                        command=self._toggle_tf).grid(row=0, column=3, padx=(12, 0))
        top.columnconfigure(1, weight=1)

        ttk.Label(top, textvariable=self.info_var,
                  font=("TkDefaultFont", 10, "bold")).grid(
            row=1, column=0, columnspan=4, sticky="w", pady=(8, 0))
        ttk.Label(top, textvariable=self.tcp_var, foreground="#0a5").grid(
            row=2, column=0, columnspan=4, sticky="w", pady=(2, 0))

        body = ttk.Frame(self, padding=(8, 4))
        body.pack(fill="both", expand=True)

        self.axis_frame = ttk.LabelFrame(body, text="Achsen", padding=(10, 8))
        self.axis_frame.pack(side="left", fill="y", padx=(0, 8))

        notebook = ttk.Notebook(body)
        notebook.pack(side="left", fill="both", expand=True)

        self.frame_table = self._make_table(notebook, FRAME_COLUMNS)
        self.joint_table = self._make_table(notebook, JOINT_COLUMNS)
        self.link_table = self._make_table(notebook, LINK_COLUMNS)
        notebook.add(self.frame_table.master, text="Frames (FK)")
        notebook.add(self.joint_table.master, text="Gelenke")
        notebook.add(self.link_table.master, text="Links")
        notebook.add(self._build_edit_tab(notebook), text="Bearbeiten")

        self.frame_table.tag_configure("tcp", background="#e8f5e9")
        self.frame_table.tag_configure("orphan", background="#ffebee")

        status = ttk.Frame(self, padding=(10, 6))
        status.pack(fill="x")
        # Meldungen links, Zeitanzeigen rechts. Der Tick ueberschreibt damit
        # keine Rueckmeldung aus der Eingabepruefung mehr.
        ttk.Label(status, textvariable=self.status_var).pack(side="left")
        ttk.Label(status, textvariable=self.refresh_var,
                  font=("TkFixedFont", 9)).pack(side="right")
        ttk.Separator(status, orient="vertical").pack(
            side="right", fill="y", padx=10)
        ttk.Label(status, textvariable=self.session_var,
                  font=("TkFixedFont", 9)).pack(side="right")

    def _build_edit_tab(self, parent):
        """Nur Laengen, Winkel und Gelenkgrenzen sind aenderbar.

        Achse, Typ sowie parent/child bleiben bewusst gesperrt: sie definieren
        die Topologie und gehoeren in den CAD-Export, nicht in eine Bedienoberflaeche.
        """
        tab = ttk.Frame(parent, padding=(14, 12))
        self.edit_entries = {}

        head = ttk.Frame(tab)
        head.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 14))
        ttk.Label(head, text="Gelenk:").pack(side="left")
        self.edit_combo = ttk.Combobox(head, textvariable=self.edit_joint_var,
                                       state="readonly", width=24)
        self.edit_combo.pack(side="left", padx=(8, 14))
        self.edit_combo.bind("<<ComboboxSelected>>", lambda _e: self._edit_load_joint())
        ttk.Label(head, textvariable=self.edit_info_var,
                  foreground="#666").pack(side="left")

        def field_row(row, title, specs):
            ttk.Label(tab, text=title, font=("TkDefaultFont", 9, "bold")).grid(
                row=row, column=0, sticky="w", pady=(6, 2))
            holder = ttk.Frame(tab)
            holder.grid(row=row, column=1, columnspan=3, sticky="w", pady=(6, 2))
            for key, label in specs:
                ttk.Label(holder, text=label).pack(side="left", padx=(0, 4))
                entry = tk.Entry(holder, width=10, justify="right",
                                 background=BG_OK, relief="solid", borderwidth=1)
                entry.pack(side="left", padx=(0, 14))
                self.edit_entries[key] = entry

        field_row(1, "Laenge  [m]", [("x", "X"), ("y", "Y"), ("z", "Z")])
        field_row(2, "Winkel  [deg]", [("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw")])
        field_row(3, "Auslenkung", [("lower", "min"), ("upper", "max")])
        ttk.Label(tab, textvariable=self.edit_unit_var, foreground="#666").grid(
            row=3, column=1, columnspan=3, sticky="e", padx=(0, 20))

        actions = ttk.Frame(tab)
        actions.grid(row=4, column=0, columnspan=4, sticky="w", pady=(18, 0))
        ttk.Button(actions, text="Uebernehmen", command=self._edit_apply).pack(side="left")
        ttk.Button(actions, text="Feld zuruecksetzen",
                   command=self._edit_load_joint).pack(side="left", padx=(6, 0))

        ttk.Separator(tab, orient="horizontal").grid(
            row=5, column=0, columnspan=4, sticky="ew", pady=16)

        files = ttk.Frame(tab)
        files.grid(row=6, column=0, columnspan=4, sticky="w")
        self.save_button = ttk.Button(files, text="Speichern", command=self._save,
                                      state="disabled")
        self.save_button.pack(side="left")
        ttk.Button(files, text="Datei neu laden",
                   command=self._discard).pack(side="left", padx=(6, 0))
        ttk.Label(files, text="Speichern legt eine Sicherungskopie mit Zeitstempel an.",
                  foreground="#777").pack(side="left", padx=(14, 0))

        return tab

    def _edit_fill_combo(self):
        names = [j["name"] for j in self.joints]
        self.edit_combo.configure(values=names)
        if names:
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
            f"Typ {joint['type']}   {joint['parent']} -> {joint['child']}   "
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
            self.status_var.set("Eingabe nicht lesbar, nichts geaendert")
            return

        # Auf einer Kopie arbeiten und erst nach der Pruefung tauschen.
        draft = copy.deepcopy(self.xml_root)
        element = next((j for j in draft.findall("joint")
                        if j.get("name") == joint["name"]), None)
        if element is None:
            self.status_var.set(f"Gelenk {joint['name']} nicht mehr im Baum")
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
                limit.set("effort", "100")              # in der URDF Pflichtangaben
                limit.set("velocity", "1.0")
            limit.set("lower", f"{lower:.6g}")
            limit.set("upper", f"{upper:.6g}")

        problems = validate_root(draft)
        if problems:
            messagebox.showerror("URDF", "Nicht uebernommen:\n\n" + "\n".join(problems))
            return

        self.xml_root = draft
        self._rebuild_from_root()
        self._set_dirty(True)
        self.status_var.set(f"{joint['name']} geaendert, noch nicht gespeichert")

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
        self._build_axis_inputs()
        self._sync_axis_entries()
        self._fill_joint_table()
        self._fill_link_table()
        self._update_kinematics()
        self._edit_load_joint()

        self.ros.set_model(self.joints, self.urdf_xml)
        self.ros.set_positions(self.positions)

    def _sync_axis_entries(self):
        """Eingabefelder auf die aktuellen Sollwerte setzen."""
        for joint in self.joints:
            entry = self.entries.get(joint["name"])
            if entry is None:
                continue
            value = self.positions.get(joint["name"], 0.0)
            shown = math.degrees(value) if is_rotational(joint) else value
            entry.delete(0, "end")
            entry.insert(0, f"{shown:.2f}")

    def _update_header(self):
        roots = root_links(self.links, self.joints)
        movable = sum(1 for j in self.joints if j["type"] != "fixed")
        self.info_var.set(
            f"Roboter: {self.robot_name}    Links: {len(self.links)}    "
            f"Gelenke: {len(self.joints)} ({movable} beweglich)    "
            f"Wurzel: {roots[0] if len(roots) == 1 else roots}")

    def _set_dirty(self, dirty):
        self.dirty = dirty
        self.save_button.configure(state="normal" if dirty else "disabled")
        self.title(f"URDF Viewer{' *' if dirty else ''}")

    def _save(self):
        if not self.dirty:
            return
        try:
            backup = save_urdf(self.urdf_path, self.urdf_xml)
        except OSError as exc:
            messagebox.showerror("Speichern", f"Schreiben fehlgeschlagen:\n{exc}")
            return
        self._set_dirty(False)
        self.status_var.set(
            f"Gespeichert. Sicherung: {os.path.basename(backup)}" if backup
            else "Gespeichert.")

    def _discard(self):
        if self.dirty and not messagebox.askyesno(
                "Neu laden", "Nicht gespeicherte Aenderungen verwerfen?"):
            return
        self._load(self.urdf_path)

    def _make_table(self, parent, columns):
        frame = ttk.Frame(parent)
        keys = [c[0] for c in columns]
        table = ttk.Treeview(frame, columns=keys, show="headings", selectmode="browse")
        for key, heading, width in columns:
            table.heading(key, text=heading)
            table.column(key, width=width, anchor="w", stretch=False)

        yscroll = ttk.Scrollbar(frame, orient="vertical", command=table.yview)
        xscroll = ttk.Scrollbar(frame, orient="horizontal", command=table.xview)
        table.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)

        table.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll.grid(row=1, column=0, sticky="ew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)
        return table

    def _build_axis_inputs(self):
        """Eingabefelder passend zur geladenen URDF neu aufbauen."""
        for child in self.axis_frame.winfo_children():
            child.destroy()
        self.entries.clear()

        movable = [j for j in self.joints if j["type"] != "fixed"]
        if not movable:
            ttk.Label(self.axis_frame, text="keine beweglichen Gelenke").grid(row=0, column=0)
            return

        for row, joint in enumerate(movable):
            unit = "deg" if is_rotational(joint) else "m"
            ttk.Label(self.axis_frame, text=joint["name"]).grid(
                row=row, column=0, sticky="w", pady=2)

            entry = tk.Entry(self.axis_frame, width=9, justify="right",
                             background=BG_OK, relief="solid", borderwidth=1)
            entry.insert(0, "0.00")
            entry.grid(row=row, column=1, padx=(8, 4), pady=2)
            entry.bind("<Return>", lambda _event: self._apply())
            entry.bind("<FocusOut>", lambda _event: self._apply())
            self.entries[joint["name"]] = entry

            ttk.Label(self.axis_frame, text=unit, width=4).grid(
                row=row, column=2, sticky="w")

            if joint["has_limit"]:
                lo, up = joint["lower"], joint["upper"]
                if is_rotational(joint):
                    lo, up = math.degrees(lo), math.degrees(up)
                hint = f"{lo:.1f} .. {up:.1f}"
            else:
                hint = "unbegrenzt"
            ttk.Label(self.axis_frame, text=hint, foreground="#777").grid(
                row=row, column=3, sticky="w", padx=(6, 0))

        buttons = ttk.Frame(self.axis_frame)
        buttons.grid(row=len(movable), column=0, columnspan=4, sticky="w", pady=(10, 0))
        ttk.Button(buttons, text="Uebernehmen", command=self._apply).pack(side="left")
        ttk.Button(buttons, text="Nullstellung", command=self._reset).pack(
            side="left", padx=(6, 0))

    # -- Daten -------------------------------------------------------------

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
            if initial:
                self._choose()
            return
        except ET.ParseError as exc:
            messagebox.showerror("URDF", f"XML fehlerhaft:\n{exc}")
            return

        self.urdf_path = path
        self.xml_root = root
        self.robot_name, self.links, self.joints, self.urdf_xml = name, links, joints, xml_text
        self.path_var.set(path)
        self.positions = {j["name"]: 0.0 for j in joints if j["type"] != "fixed"}
        self._set_dirty(False)

        roots = root_links(links, joints)
        self._update_header()

        self._build_axis_inputs()
        self._edit_fill_combo()
        self._fill_joint_table()
        self._fill_link_table()
        self._update_kinematics()

        if len(roots) != 1:
            messagebox.showwarning(
                "URDF", f"Genau ein Wurzel-Link erwartet, gefunden: {len(roots)}")

        problems = validate_root(root)
        if problems:
            messagebox.showwarning("URDF", "Hinweise:\n\n" + "\n".join(problems))

        self.status_var.set(self.ros.set_model(joints, xml_text))
        self.ros.set_positions(self.positions)

    def _fill_joint_table(self):
        self.joint_table.delete(*self.joint_table.get_children())
        for joint in self.joints:
            if joint["has_limit"]:
                limit = (f"{math.degrees(joint['lower']):.1f} .. "
                         f"{math.degrees(joint['upper']):.1f}")
            elif joint["type"] == "continuous":
                limit = "unbegrenzt"
            else:
                limit = "-"
            effort = f"{joint['effort']:.1f}" if joint["effort"] else "-"
            velocity = f"{joint['velocity']:.2f}" if joint["velocity"] else "-"
            self.joint_table.insert("", "end", values=(
                joint["name"], joint["type"], joint["parent"], joint["child"],
                fmt(joint["xyz"]), fmt(joint["rpy"]), fmt(joint["axis"], 1),
                limit, effort, velocity))

    def _fill_link_table(self):
        self.link_table.delete(*self.link_table.get_children())
        for link in self.links:
            if link["has_inertial"]:
                mass, com = f"{link['mass']:.4f}", fmt(link["com"])
            else:
                mass = com = "-"
            self.link_table.insert("", "end", values=(link["name"], mass, com))

    # -- Eingabe -----------------------------------------------------------

    def _apply(self):
        """Eingaben lesen, pruefen, begrenzen und uebernehmen."""
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

        self._update_kinematics()
        self.ros.set_positions(self.positions)

        if invalid:
            self.status_var.set(f"{invalid} Eingabe(n) nicht lesbar, Wert unveraendert")
        elif clamped:
            self.status_var.set(f"{clamped} Wert(e) auf das URDF-Limit begrenzt")

    def _reset(self):
        for entry in self.entries.values():
            entry.configure(background=BG_OK)
            entry.delete(0, "end")
            entry.insert(0, "0.00")
        self._apply()

    # -- Kinematik ---------------------------------------------------------

    def _update_kinematics(self):
        """FK fuer alle Frames neu rechnen und die Tabelle fuellen."""
        poses = forward_kinematics(self.links, self.joints, self.positions)
        via = {j["child"]: j["name"] for j in self.joints}
        leaves = set(leaf_links(self.links, self.joints))
        roots = root_links(self.links, self.joints)
        base = roots[0] if roots else "?"

        self.frame_table.delete(*self.frame_table.get_children())
        for link in self.links:
            name = link["name"]
            matrix = poses.get(name)
            if matrix is None:                          # nicht mit der Basis verbunden
                self.frame_table.insert("", "end", tags=("orphan",), values=(
                    name, via.get(name, "-"), *(["-"] * 6), "nicht erreichbar"))
                continue

            xyz, rpy = to_xyz_rpy(matrix)
            distance = math.sqrt(sum(v * v for v in xyz))
            tags = ("tcp",) if name in leaves and name != base else ()
            self.frame_table.insert("", "end", tags=tags, values=(
                name, via.get(name, "-"),
                f"{xyz[0]:.4f}", f"{xyz[1]:.4f}", f"{xyz[2]:.4f}",
                f"{math.degrees(rpy[0]):.2f}",
                f"{math.degrees(rpy[1]):.2f}",
                f"{math.degrees(rpy[2]):.2f}",
                f"{distance:.4f}"))

        tcps = [n for n in leaves if n != base]
        if len(tcps) == 1 and tcps[0] in poses:
            xyz, rpy = to_xyz_rpy(poses[tcps[0]])
            self.tcp_var.set(
                f"TCP {tcps[0]} in {base}:   "
                f"X {xyz[0]:.4f}   Y {xyz[1]:.4f}   Z {xyz[2]:.4f} m   |   "
                f"Roll {math.degrees(rpy[0]):.2f}   "
                f"Pitch {math.degrees(rpy[1]):.2f}   "
                f"Yaw {math.degrees(rpy[2]):.2f} deg")
        elif tcps:
            self.tcp_var.set(f"Mehrere Endeffektoren gefunden: {', '.join(sorted(tcps))}")
        else:
            self.tcp_var.set("Kein Endeffektor gefunden")

    # -- ROS ---------------------------------------------------------------

    def _start_ros(self):
        error = self.ros.start()
        if error:
            self.status_var.set(error)

    def _toggle_tf(self):
        self.ros.enabled = self.send_tf.get()
        if not self.send_tf.get():
            self.status_var.set("TF-Ausgabe angehalten. RViz behaelt die letzte Pose "
                                "etwa 10 Sekunden, dann meldet es fehlende Frames.")

    def _tick(self):
        """Fusszeile aktualisieren. Laeuft im GUI-Thread, greift nie auf ROS zu."""
        elapsed = int(time.monotonic() - self.session_start)
        self.session_var.set(
            f"Sitzung {elapsed // 3600:02d}:{elapsed // 60 % 60:02d}:{elapsed % 60:02d}")

        last = self.ros.last_publish
        if not self.ros.running:
            self.refresh_var.set("Last Refresh --  (kein ROS)")
        elif not self.send_tf.get():
            self.refresh_var.set("Last Refresh --  (angehalten)")
        elif last is None:
            self.refresh_var.set("Last Refresh --  (wartet)")
        else:
            age_ms = (time.monotonic() - last) * 1000.0
            cycle = self.ros.cycle_ms
            takt = f", Takt {cycle:5.1f} ms" if cycle is not None else ""
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
