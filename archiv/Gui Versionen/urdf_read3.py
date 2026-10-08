#!/usr/bin/env python3
"""Liest eine URDF, gibt Links und Gelenke aus und sendet sie auf /tf_static.

Aufrufbeispiele:
    ./urdf_read3.py                          # Standardpfad, alle Gelenke senden
    ./urdf_read3.py ~/Display/bmKinematic.urdf
    ./urdf_read3.py --only-fixed             # nur feste Gelenke senden
    ./urdf_read3.py --print-only             # kein ROS, nur Ausgabe wie bisher
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET

# ---------- hier URDF eintragen ----------
URDF_PATH = "/home/fwohlfeil/Display/bmKinematic.urdf"
# -----------------------------------------


def num(elem, name, default=0.0):
    """Einzelnes Attribut als float."""
    if elem is None or elem.get(name) is None:
        return default
    return float(elem.get(name))


def vec(elem, name, default=(0.0, 0.0, 0.0)):
    """xyz/rpy-Attribut als Liste aus floats."""
    if elem is None or elem.get(name) is None:
        return list(default)
    return [float(v) for v in elem.get(name).split()]


def fmt(values, digits=4):
    """Zahlenliste kompakt formatieren."""
    return "[" + ", ".join(f"{v:.{digits}f}" for v in values) + "]"


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
    """URDF einlesen und als Dictionaries mit numerischen Werten zurueckgeben."""
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


def report(name, links, joints):
    """Ausgabe wie bisher, ergaenzt um das Quaternion."""
    print(f"Roboter: {name}")

    print(f"\nLinks ({len(links)}):")
    for i, link in enumerate(links, 1):
        print(f"  {i}. {link['name']}")
        if link["has_inertial"]:
            print(f"       Masse:  {link['mass']:.4f} kg")
            print(f"       COM:    {fmt(link['com'])}")

    print(f"\nGelenke ({len(joints)}):")
    for i, joint in enumerate(joints, 1):
        print(f"  {i}. {joint['name']}  [{joint['type']}]")
        print(f"       Parent: {joint['parent']}")
        print(f"       Child:  {joint['child']}")
        print(f"       Origin: xyz={fmt(joint['xyz'])}  rpy={fmt(joint['rpy'])}")
        print(f"       Quat:   {fmt(joint['quat'], 6)}  (x, y, z, w)")
        print(f"       Achse:  {fmt(joint['axis'], 1)}")
        if joint["has_limit"]:
            lo, up = joint["lower"], joint["upper"]
            print(f"       Limit:  {lo:.4f} .. {up:.4f} rad"
                  f"  ({math.degrees(lo):.1f} .. {math.degrees(up):.1f} deg)"
                  f"  Spanne {math.degrees(up - lo):.1f} deg")
            print(f"       Effort: {joint['effort']:.2f}   Velocity: {joint['velocity']:.2f}")

    print("\nAuswertung:")
    print(f"  Gesamtmasse:      {sum(l['mass'] for l in links):.4f} kg")
    kette = [sum(j['xyz'][k] for j in joints) for k in range(3)]
    print(f"  Summe Offsets:    {fmt(kette)}")
    print(f"  Kettenlaenge:     {math.sqrt(sum(v * v for v in kette)):.4f} m")
    beweglich = [j for j in joints if j["type"] not in ("fixed",)]
    print(f"  Bewegliche Achsen: {len(beweglich)} von {len(joints)}")

    children = {j["child"] for j in joints}
    wurzeln = [l["name"] for l in links if l["name"] not in children]
    print(f"  Wurzel-Link:      {wurzeln[0] if len(wurzeln) == 1 else wurzeln}")
    if len(wurzeln) != 1:
        print("  WARNUNG: Es wird genau ein Wurzel-Link erwartet.")


def publish(name, joints, urdf_xml, only_fixed):
    """Node starten, /tf_static und robot_description senden, dann warten."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster

    class UrdfStaticTf(Node):

        def __init__(self):
            super().__init__("urdf_static_tf")
            self.broadcaster = StaticTransformBroadcaster(self)

            qos = QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.description_pub = self.create_publisher(String, "robot_description", qos)

            gewaehlt = [j for j in joints
                        if (j["type"] == "fixed" or not only_fixed)]
            uebersprungen = len(joints) - len(gewaehlt)

            stamp = self.get_clock().now().to_msg()
            nachrichten = []
            for joint in gewaehlt:
                if not joint["parent"] or not joint["child"]:
                    self.get_logger().warn(
                        f"Gelenk {joint['name']} ohne parent/child, uebersprungen")
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
                nachrichten.append(msg)

            self.broadcaster.sendTransform(nachrichten)
            self.description_pub.publish(String(data=urdf_xml))

            self.get_logger().info(
                f"{name}: {len(nachrichten)} Transformationen auf /tf_static"
                + (f", {uebersprungen} bewegliche ausgelassen" if uebersprungen else ""))
            self.get_logger().info(
                "robot_description gesendet (latched). Node laeuft weiter, "
                "damit spaete Subscriber die Daten noch bekommen.")

    rclpy.init()
    node = UrdfStaticTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", nargs="?", default=URDF_PATH, help="Pfad zur URDF")
    parser.add_argument("--only-fixed", action="store_true",
                        help="nur Gelenke vom Typ fixed senden")
    parser.add_argument("--print-only", action="store_true",
                        help="nur ausgeben, nichts senden")
    args, _ = parser.parse_known_args()

    try:
        name, links, joints, urdf_xml = parse(args.urdf)
    except FileNotFoundError:
        print(f"URDF nicht gefunden: {args.urdf}", file=sys.stderr)
        return 1
    except ET.ParseError as exc:
        print(f"URDF fehlerhaft: {exc}", file=sys.stderr)
        return 1

    report(name, links, joints)

    if args.print_only:
        return 0

    print()
    publish(name, joints, urdf_xml, args.only_fixed)
    return 0


if __name__ == "__main__":
    sys.exit(main())
