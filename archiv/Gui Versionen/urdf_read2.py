#!/usr/bin/env python3
"""Liest eine URDF, wandelt die Werte in Zahlen um und gibt Links und Gelenke aus."""

import math
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


def parse(path):
    """URDF einlesen und als Dictionaries mit numerischen Werten zurueckgeben."""
    root = ET.parse(path).getroot()

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
        joints.append({
            "name": joint.get("name"),
            "type": joint.get("type"),
            "parent": joint.find("parent").get("link") if joint.find("parent") is not None else None,
            "child": joint.find("child").get("link") if joint.find("child") is not None else None,
            "xyz": vec(origin, "xyz"),
            "rpy": vec(origin, "rpy"),
            "axis": vec(joint.find("axis"), "xyz", (1.0, 0.0, 0.0)),
            "lower": num(limit, "lower"),
            "upper": num(limit, "upper"),
            "effort": num(limit, "effort"),
            "velocity": num(limit, "velocity"),
            "has_limit": limit is not None,
        })

    return root.get("name"), links, joints


def main():
    name, links, joints = parse(URDF_PATH)

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
        print(f"       Achse:  {fmt(joint['axis'], 1)}")
        if joint["has_limit"]:
            lo, up = joint["lower"], joint["upper"]
            print(f"       Limit:  {lo:.4f} .. {up:.4f} rad"
                  f"  ({math.degrees(lo):.1f} .. {math.degrees(up):.1f} deg)"
                  f"  Spanne {math.degrees(up - lo):.1f} deg")
            print(f"       Effort: {joint['effort']:.2f}   Velocity: {joint['velocity']:.2f}")

   


if __name__ == "__main__":
    main()
