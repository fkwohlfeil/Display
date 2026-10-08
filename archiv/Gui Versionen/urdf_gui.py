#!/usr/bin/env python3
"""URDF in einer GUI anzeigen und die Werte auf /tf_static senden.

Aufruf:
    ./urdf_gui.py                       # Standardpfad aus URDF_PATH
    ./urdf_gui.py ~/Display/bmKinematic.urdf

Die ROS-Importe liegen in RosBridge, damit die GUI auch ohne gesourctes
ROS-Environment startet. Ohne ROS werden die Werte nur angezeigt.
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


def rpy_to_quaternion(roll, pitch, yaw):
    """URDF-Konvention (feste Achsen, Rz*Ry*Rx) -> (x, y, z, w)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


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
        joints.append({
            "name": joint.get("name"),
            "type": joint.get("type"),
            "parent": joint.find("parent").get("link") if joint.find("parent") is not None else None,
            "child": joint.find("child").get("link") if joint.find("child") is not None else None,
            "xyz": vec(origin, "xyz"),
            "rpy": rpy,
            "quat": rpy_to_quaternion(*rpy),
            "axis": vec(joint.find("axis"), "xyz", (1.0, 0.0, 0.0)),
            "lower": num(limit, "lower"),
            "upper": num(limit, "upper"),
            "effort": num(limit, "effort"),
            "velocity": num(limit, "velocity"),
            "has_limit": limit is not None,
        })

    return root.get("name"), links, joints, urdf_xml


def root_links(links, joints):
    """Links, die in keinem <child> vorkommen."""
    children = {j["child"] for j in joints}
    return [l["name"] for l in links if l["name"] not in children]


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
        """Transformationen und robot_description senden. Gibt Statustext zurueck."""
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
    ("name", "Name",           180),
    ("mass", "Masse [kg]",     100),
    ("com",  "COM xyz [m]",    180),
]


class UrdfViewer(tk.Tk):

    def __init__(self, urdf_path):
        super().__init__()
        self.title("URDF Viewer")
        self.geometry("1150x600")
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.urdf_path = urdf_path
        self.robot_name = ""
        self.links = []
        self.joints = []
        self.urdf_xml = ""

        self.path_var = tk.StringVar(value=urdf_path)
        self.info_var = tk.StringVar(value="")
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

        ttk.Label(top, textvariable=self.info_var, font=("TkDefaultFont", 10, "bold")).grid(
            row=1, column=0, columnspan=5, sticky="w", pady=(8, 0))

        notebook = ttk.Notebook(self, padding=(8, 4))
        notebook.pack(fill="both", expand=True)

        self.joint_table = self._make_table(notebook, JOINT_COLUMNS)
        self.link_table = self._make_table(notebook, LINK_COLUMNS)
        notebook.add(self.joint_table.master, text="Gelenke")
        notebook.add(self.link_table.master, text="Links")

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
        self._fill_tables()

        roots = root_links(links, joints)
        movable = sum(1 for j in joints if j["type"] != "fixed")
        info = (f"Roboter: {name}    Links: {len(links)}    "
                f"Gelenke: {len(joints)} ({movable} beweglich)    "
                f"Wurzel: {roots[0] if len(roots) == 1 else roots}")
        self.info_var.set(info)

        if len(roots) != 1:
            messagebox.showwarning(
                "URDF", f"Genau ein Wurzel-Link erwartet, gefunden: {len(roots)}")

        self._publish()

    def _fill_tables(self):
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

        self.link_table.delete(*self.link_table.get_children())
        for link in self.links:
            if link["has_inertial"]:
                mass, com = f"{link['mass']:.4f}", fmt(link["com"])
            else:
                mass = com = "-"
            self.link_table.insert("", "end", values=(link["name"], mass, com))

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
        except Exception as exc:                       # noqa: BLE001
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
