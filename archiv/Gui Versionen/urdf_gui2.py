#!/usr/bin/env python3
"""URDF anzeigen, Vorwaertskinematik rechnen und auf /tf_static senden.

Aufruf:
    ./urdf_gui2.py                       # Standardpfad aus URDF_PATH
    ./urdf_gui2.py ~/Display/bmKinematic.urdf

Die Matrizenrechnung ist bewusst in reinem Python gehalten, damit das
Programm ohne numpy und ohne gesourctes ROS-Environment startet.
"""

import argparse
import math
import os
import sys
import threading
import tkinter as tk
import xml.etree.ElementTree as ET
from tkinter import ttk, filedialog, messagebox

# ---------- hier URDF eintragen ----------
URDF_PATH = "/home/fwohlfeil/simple_robot.urdf"
# -----------------------------------------


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


def parse(path):
    """URDF einlesen. Gibt (name, links, joints, xml) zurueck."""
    with open(path, "r") as handle:
        urdf_xml = handle.read()
    root = ET.fromstring(urdf_xml)

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

        joints.append({
            "name": joint.get("name"),
            "type": joint.get("type"),
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
            "has_limit": limit is not None,
        })

    return root.get("name"), links, joints, urdf_xml


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
    """{linkname: 4x4} relativ zum Wurzel-Link, in einem Durchlauf.

    positions: {gelenkname: wert}. Fehlende Gelenke gelten als 0.
    """
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
    """Haelt Node und Spin-Thread. Alle ROS-Importe liegen hier drin."""

    def __init__(self):
        self.node = None
        self.error = None
        self._executor = None
        self._thread = None
        self._broadcaster = None
        self._description_pub = None

    @property
    def running(self):
        return self.node is not None

    def start(self):
        if self.running:
            return None
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import (QoSProfile, DurabilityPolicy,
                                   ReliabilityPolicy, HistoryPolicy)
            from std_msgs.msg import String
        except ImportError as exc:
            self.error = f"ROS 2 nicht verfuegbar ({exc.name}). Nur Anzeige."
            return self.error

        rclpy.init()
        self.node = Node("urdf_gui_static_tf")

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._description_pub = self.node.create_publisher(String, "robot_description", qos)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        return None

    def publish(self, joints, urdf_xml, only_fixed):
        if not self.running:
            return self.error or "Kein ROS-Node aktiv"

        from geometry_msgs.msg import TransformStamped
        from std_msgs.msg import String
        from tf2_ros import StaticTransformBroadcaster

        # Broadcaster neu anlegen: der alte latcht sonst weiterhin Frames,
        # die bei "nur feste Gelenke" gar nicht mehr gesendet werden sollen.
        if self._broadcaster is not None:
            old = getattr(self._broadcaster, "pub_tf", None)
            if old is not None:
                self.node.destroy_publisher(old)
        self._broadcaster = StaticTransformBroadcaster(self.node)

        stamp = self.node.get_clock().now().to_msg()
        messages = []
        skipped = 0
        for joint in joints:
            if only_fixed and joint["type"] != "fixed":
                skipped += 1
                continue
            if not joint["parent"] or not joint["child"]:
                skipped += 1
                continue
            msg = TransformStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = joint["parent"]
            msg.child_frame_id = joint["child"]
            msg.transform.translation.x = joint["xyz"][0]
            msg.transform.translation.y = joint["xyz"][1]
            msg.transform.translation.z = joint["xyz"][2]
            qx, qy, qz, qw = joint["quat"]
            msg.transform.rotation.x = qx
            msg.transform.rotation.y = qy
            msg.transform.rotation.z = qz
            msg.transform.rotation.w = qw
            messages.append(msg)

        if messages:
            self._broadcaster.sendTransform(messages)
        self._description_pub.publish(String(data=urdf_xml))

        text = f"{len(messages)} Transformationen auf /tf_static, robot_description gesendet"
        if skipped:
            text += f"  ({skipped} ausgelassen)"
        return text

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


# ==========================================================================
# GUI
# ==========================================================================

FRAME_COLUMNS = [
    ("link",     "Frame",            140),
    ("via",      "ueber Gelenk",     120),
    ("x",        "X [m]",             90),
    ("y",        "Y [m]",             90),
    ("z",        "Z [m]",             90),
    ("roll",     "Roll [deg]",        90),
    ("pitch",    "Pitch [deg]",       90),
    ("yaw",      "Yaw [deg]",         90),
    ("distance", "Abstand [m]",       95),
    ("quat",     "Quaternion x y z w", 200),
]

JOINT_COLUMNS = [
    ("name",     "Name",              130),
    ("type",     "Typ",                80),
    ("parent",   "Parent",            110),
    ("child",    "Child",             110),
    ("xyz",      "Origin xyz [m]",    170),
    ("rpy",      "Origin rpy [rad]",  170),
    ("quat",     "Quaternion x y z w", 200),
    ("axis",     "Achse",              90),
    ("limit",    "Limit [deg]",       130),
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
        self.geometry("1180x640")
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.urdf_path = urdf_path
        self.robot_name = ""
        self.links = []
        self.joints = []
        self.urdf_xml = ""
        self.positions = {}                            # {gelenkname: wert in rad/m}

        self.path_var = tk.StringVar(value=urdf_path)
        self.info_var = tk.StringVar(value="")
        self.tcp_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="")
        self.only_fixed = tk.BooleanVar(value=False)

        self.ros = RosBridge()

        self._build_ui()
        self._start_ros()
        self._load(urdf_path, initial=True)

    # -- Aufbau ------------------------------------------------------------

    def _build_ui(self):
        top = ttk.Frame(self, padding=(10, 8))
        top.pack(fill="x")

        ttk.Label(top, text="Datei:").grid(row=0, column=0, sticky="w")
        ttk.Label(top, textvariable=self.path_var, foreground="#444").grid(
            row=0, column=1, sticky="w", padx=(6, 12))
        ttk.Button(top, text="Laden...", command=self._choose).grid(row=0, column=2)
        ttk.Button(top, text="Erneut senden", command=self._publish).grid(
            row=0, column=3, padx=(6, 0))
        ttk.Checkbutton(top, text="nur feste Gelenke senden",
                        variable=self.only_fixed,
                        command=self._publish).grid(row=0, column=4, padx=(12, 0))
        top.columnconfigure(1, weight=1)

        ttk.Label(top, textvariable=self.info_var,
                  font=("TkDefaultFont", 10, "bold")).grid(
            row=1, column=0, columnspan=5, sticky="w", pady=(8, 0))
        ttk.Label(top, textvariable=self.tcp_var, foreground="#0a5").grid(
            row=2, column=0, columnspan=5, sticky="w", pady=(2, 0))

        notebook = ttk.Notebook(self, padding=(8, 4))
        notebook.pack(fill="both", expand=True)

        self.frame_table = self._make_table(notebook, FRAME_COLUMNS)
        self.joint_table = self._make_table(notebook, JOINT_COLUMNS)
        self.link_table = self._make_table(notebook, LINK_COLUMNS)
        notebook.add(self.frame_table.master, text="Frames (FK)")
        notebook.add(self.joint_table.master, text="Gelenke")
        notebook.add(self.link_table.master, text="Links")

        self.frame_table.tag_configure("tcp", background="#e8f5e9")
        self.frame_table.tag_configure("orphan", background="#ffebee")

        status = ttk.Frame(self, padding=(10, 6))
        status.pack(fill="x")
        ttk.Label(status, textvariable=self.status_var).pack(side="left")

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
            name, links, joints, xml_text = parse(path)
        except FileNotFoundError:
            messagebox.showerror("URDF", f"Datei nicht gefunden:\n{path}")
            if initial:
                self._choose()
            return
        except ET.ParseError as exc:
            messagebox.showerror("URDF", f"XML fehlerhaft:\n{exc}")
            return

        self.urdf_path = path
        self.robot_name, self.links, self.joints, self.urdf_xml = name, links, joints, xml_text
        self.path_var.set(path)
        self.positions = {j["name"]: 0.0 for j in joints if j["type"] != "fixed"}

        roots = root_links(links, joints)
        movable = sum(1 for j in joints if j["type"] != "fixed")
        self.info_var.set(
            f"Roboter: {name}    Links: {len(links)}    "
            f"Gelenke: {len(joints)} ({movable} beweglich)    "
            f"Wurzel: {roots[0] if len(roots) == 1 else roots}")

        self._fill_joint_table()
        self._fill_link_table()
        self._update_kinematics()

        if len(roots) != 1:
            messagebox.showwarning(
                "URDF", f"Genau ein Wurzel-Link erwartet, gefunden: {len(roots)}")

        self._publish()

    def _fill_joint_table(self):
        self.joint_table.delete(*self.joint_table.get_children())
        for joint in self.joints:
            if joint["has_limit"]:
                limit = (f"{math.degrees(joint['lower']):.1f} .. "
                         f"{math.degrees(joint['upper']):.1f}")
                effort = f"{joint['effort']:.1f}"
                velocity = f"{joint['velocity']:.2f}"
            else:
                limit = effort = velocity = "-"
            self.joint_table.insert("", "end", values=(
                joint["name"], joint["type"], joint["parent"], joint["child"],
                fmt(joint["xyz"]), fmt(joint["rpy"]), fmt(joint["quat"], 6),
                fmt(joint["axis"], 1), limit, effort, velocity))

    def _fill_link_table(self):
        self.link_table.delete(*self.link_table.get_children())
        for link in self.links:
            if link["has_inertial"]:
                mass, com = f"{link['mass']:.4f}", fmt(link["com"])
            else:
                mass = com = "-"
            self.link_table.insert("", "end", values=(link["name"], mass, com))

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
                    name, via.get(name, "-"), *(["-"] * 7), "nicht erreichbar"))
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
                f"{distance:.4f}",
                fmt(to_quaternion(matrix), 6)))

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

    def _publish(self):
        if not self.joints:
            return
        try:
            self.status_var.set(
                self.ros.publish(self.joints, self.urdf_xml, self.only_fixed.get()))
        except Exception as exc:                        # noqa: BLE001
            self.status_var.set(f"Senden fehlgeschlagen: {exc}")

    def _on_close(self):
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
