"""urdf_modell -- URDF einlesen, aendern, pruefen und speichern. Kein Tkinter,
kein ROS.

Seit urdf_gui2_1 ein eigenes Modul. Der Inhalt ist zeilengleich aus
urdf_gui3_7.py verschoben (dort Zeilen 1436-2498); neu sind
nur dieser Kopf, die Importe und STAND.
"""

import copy
import math
import os
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET
from datetime import datetime

from einstellungen import (BACKUP_DIR, FREE_ROTATION_DEG, FREE_TRAVEL_M,
                           SLIDER_STEPS,BAHN_DIR) 
from matrix import (from_rpy_xyz, invert, multiply, to_quaternion, to_xyz_rpy)



# Muss zu MODUL_STAENDE in urdf_gui2_1.py passen.
STAND = 1


# ==========================================================================
# URDF einlesen
# ==========================================================================

# Reihenfolge wie in der Auswahlliste. fixed steht hinten, weil der Wechsel
# dorthin Achse und Anschlaege verwirft.
JOINT_TYPES = ("revolute", "continuous", "prismatic", "fixed")
NEEDS_AXIS = ("revolute", "continuous", "prismatic")
NEEDS_LIMIT = ("revolute", "prismatic")

# Namen werden zu TF-Frames. Leerzeichen, Schraegstriche und Umlaute machen
# dort Aerger, deshalb bewusst eng gefasst.
NAME_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_.\-]*$")

# Stellen, mit denen ein <origin> geschrieben wird. Sechs waren es bis 1.17,
# und fuer getippte Werte reicht das auch. Einfuegen und Herausnehmen rechnen
# den Ursprung aber ueber eine Matrizenkette neu; dort schlaegt die Rundung
# als Lageabweichung durch, rund drei Mikrometer je Schritt und ueber mehrere
# Schritte summiert. Zehn Stellen druecken das auf Maschinengenauigkeit,
# ohne dass die Datei unleserlich wird: 0.3 bleibt 0.3 und wird nicht zu
# 0.30000000000000004. Achse und Anschlaege bleiben bei sechs Stellen, sie
# werden nie nachgerechnet.
ORIGIN_DIGITS = 10
# Eintrag in der Kopplungsliste fuer "keine Kopplung".
NO_COUPLING = "-- frei --"

# ---------- Frames und Werkzeuge ----------
# Die beiden Rollen, die ein fester Frame haben kann. In der URDF sind beide
# ein <joint type="fixed">; unterschieden werden sie allein durch die Stelle
# in der Kette. Deshalb steht die Rolle hier als Text und nicht als Typ: sie
# waehlt aus, welche Funktion schreibt, und ist danach nicht mehr abfragbar.
ROLE_TOOL = "Werkzeug (TCP)"
ROLE_BASE = "Bezug (Basis)"

# Eintrag in der TCP-Auswahl fuer "aus dem Modell ableiten". Leer waere
# zweideutig -- ein leeres Feld sieht aus wie ein vergessenes Feld.
TIP_AUTO = "-- einziges Blatt --"

# ---------- Aussehen der Links ----------
# Die vier Formen, die URDF in <geometry> kennt. Ein Netz laesst sich nur
# skalieren, nicht umformen: die Datei bestimmt seine Gestalt.
SHAPES = ("box", "cylinder", "sphere", "mesh")
SHAPE_LABELS = {"box": "Quader", "cylinder": "Zylinder",
                "sphere": "Kugel", "mesh": "Netz"}
SHAPE_FIELDS = {
    "box":      [("sx", "X", 0.100), ("sy", "Y", 0.100), ("sz", "Z", 0.100)],
    "cylinder": [("radius", "Radius", 0.030), ("length", "Hoehe", 0.200)],
    "sphere":   [("radius", "Radius", 0.050)],
    "mesh":     [("scale", "Massstab", 1.000)],
}
DEFAULT_RGBA = (0.60, 0.60, 0.62, 1.00)
# Felder im Aussehen-Fenster, die beim Umschalten der Form stehen bleiben.
VISUAL_KEEP = ("hex", "alpha", "ox", "oy", "oz", "oroll", "opitch", "oyaw")
# Formen mit einer Ausdehnung entlang z, die sich halbieren laesst.
HAS_HEIGHT = {"box": "sz", "cylinder": "length"}

DEFAULT_EFFORT = "100"
DEFAULT_VELOCITY = "1.0"
DEFAULT_AXIS = "0 0 1"


def num(elem, name, default=0.0):
    if elem is None or elem.get(name) is None:
        return default
    return float(elem.get(name))


def vec(elem, name, default=(0.0, 0.0, 0.0)):
    if elem is None or elem.get(name) is None:
        return list(default)
    return [float(v) for v in elem.get(name).split()]


def rgba_to_hex(rgba):
    """(r, g, b, a) mit 0..1 in #rrggbb. Die Deckung bleibt aussen vor."""
    return "#" + "".join(f"{max(0, min(255, round(c * 255))):02x}"
                         for c in rgba[:3])


def hex_to_rgb(text):
    """#rrggbb in (r, g, b) mit 0..1. Wirft ValueError bei Unsinn."""
    text = text.strip().lstrip("#")
    if len(text) != 6:
        raise ValueError("Farbe braucht sechs Stellen, zum Beispiel #a0a0a2")
    return tuple(int(text[i:i + 2], 16) / 255.0 for i in (0, 2, 4))


def fmt(values, digits=4):
    return ", ".join(f"{v:.{digits}f}" for v in values)


def fmt_origin(values):
    """Zahlentripel fuer ein <origin>-Attribut. Einzige Schreibstelle."""
    return " ".join(f"{v:.{ORIGIN_DIGITS}g}" for v in values)


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


def slider_step(joint):
    """Schrittweite des Schiebers, passend zur Spanne des Gelenks.

    Eine feste Schrittweite taugt nicht: dieselbe Zahl, die bei einer Achse
    mit 340 Grad sinnvoll ist, waere bei einem Hub von 40 mm die ganze
    Spanne. Deshalb aus der Spanne abgeleitet und auf 1, 2, 2.5 oder 5 mal
    Zehnerpotenz gerundet, damit runde Werte auf dem Raster liegen.
    """
    low, high = slider_range(joint)
    span = abs(high - low)
    if span < 1e-12:
        return 1e-3
    raw = span / SLIDER_STEPS
    exponent = math.floor(math.log10(raw))
    for nice in (1.0, 2.0, 2.5, 5.0):
        step = nice * 10.0 ** exponent
        if step >= raw:
            return step
    return 10.0 ** (exponent + 1)


def value_digits(step):
    """Nachkommastellen, die zur Schrittweite passen.

    Mindestens zwei, damit Winkel weiter wie gewohnt mit zwei Stellen
    dastehen; bei feinen Schritten entsprechend mehr.
    """
    if step >= 1.0:
        return 2
    return max(2, min(6, int(math.ceil(-math.log10(step)))))


def limit_digits(joint):
    """Stellen fuer die Anzeige der Anschlaege.

    Gedeckelt, damit die Spalte nicht ueberlaeuft; die Werte im Eingabefeld
    duerfen feiner sein als diese Kurzanzeige.
    """
    return min(4, value_digits(slider_step(joint)))


def snap(value, low, high, step):
    """Auf das Raster legen und in der Spanne halten.

    Verankert bei null, nicht am unteren Anschlag: sonst laege die
    Nullstellung zwischen zwei Stufen und waere mit dem Schieber nicht
    genau zu treffen. Die Anschlaege liegen meist neben dem Raster, weil sie
    aus gerundeten Radiant stammen; die letzte halbe Stufe faengt deshalb
    auf den Anschlag, damit er auch am Schieber genau erreichbar bleibt.
    """
    if value >= high - step / 2.0:
        return high
    if value <= low + step / 2.0:
        return low
    return min(max(round(value / step) * step, low), high)


def read_visual(root, link):
    """Form, Groesse und Farbe des ersten <visual> eines Links oder None.

    URDF erlaubt mehrere <visual> je Link, etwa um einen Koerper aus zwei
    Grundformen zusammenzusetzen. Gelesen und geschrieben wird nur das erste;
    alles andere waere ein eigener Editor. Die Anzahl wird mitgegeben, damit
    das Fenster warnen kann, statt stillschweigend nur die Haelfte zu zeigen.
    """
    visuals = link.findall("visual")
    if not visuals:
        return None
    visual = visuals[0]
    geometry = visual.find("geometry")
    origin = visual.find("origin")

    shape, size, filename = None, {}, ""
    for kind in SHAPES:
        node = geometry.find(kind) if geometry is not None else None
        if node is None:
            continue
        shape = kind
        if kind == "box":
            values = vec(node, "size", (0.1, 0.1, 0.1))
            size = dict(zip(("sx", "sy", "sz"), values))
        elif kind == "cylinder":
            size = {"radius": num(node, "radius", 0.03),
                    "length": num(node, "length", 0.2)}
        elif kind == "sphere":
            size = {"radius": num(node, "radius", 0.05)}
        else:
            filename = node.get("filename", "")
            # Ein Netz skaliert dreiachsig. Der Editor bietet einen Faktor an;
            # ungleiche Achsen bleiben erhalten, solange niemand ihn anfasst.
            size = {"scale": vec(node, "scale", (1.0, 1.0, 1.0))[0]}
        break

    rgba, material = colour_of(root, visual)
    return {"shape": shape, "size": size, "filename": filename,
            "xyz": vec(origin, "xyz"), "rpy": vec(origin, "rpy"),
            "rgba": rgba, "material": material, "count": len(visuals),
            "shared": material is not None and rgba is not None
                      and visual.find("material") is not None
                      and visual.find("material").find("color") is None}


def colour_of(root, visual):
    """Farbe eines <visual> als ((r, g, b, a), Materialname).

    Ein Material kann die Farbe selbst tragen oder nur auf eine oben im
    Dokument definierte verweisen. Der Verweis ist der Grund, warum diese
    Funktion den ganzen Baum braucht und nicht nur das <visual>.
    """
    material = visual.find("material")
    if material is None:
        return None, None
    name = material.get("name")
    colour = material.find("color")
    if colour is None and name:
        for shared in root.findall("material"):
            if shared.get("name") == name:
                colour = shared.find("color")
                break
    if colour is None or colour.get("rgba") is None:
        return None, name
    values = [float(v) for v in colour.get("rgba").split()]
    while len(values) < 4:
        values.append(1.0)
    return tuple(values[:4]), name


def set_visual(root, link_name, shape, size, rgba, filename,
               xyz, rpy, with_collision):
    """Form, Groesse und Farbe des ersten <visual> setzen. Legt es bei Bedarf an.

    Zur Farbe: geschrieben wird immer ein eigenes Material je Link mit
    eingebauter <color>. Der Grund ist Vorhersagbarkeit. Verweist ein Link auf
    ein gemeinsames Material, faerbt eine Aenderung sonst alle mit, die
    denselben Namen nennen -- wer die Farbe eines Links einstellt, erwartet
    das nicht. Der Preis ist ein Materialeintrag mehr je Link.

    Der Ursprung verschiebt und dreht den Koerper gegen den Linkframe. Er ist
    der Grund, warum ein Quader ohne Zutun mittig auf dem Frame sitzt und
    damit zur Haelfte nach hinten ragt: <origin> fehlt, und dann gilt die
    Null. Fuer ein Armsegment entlang z gehoert dort meist die halbe Hoehe.
    Mit der Kinematik hat das nichts zu tun -- der <origin> eines Gelenks
    bewegt die Kette, der eines <visual> nur das Bild.

    `with_collision` fuehrt dieselbe Form und denselben Ursprung in
    <collision> nach. Ohne das driften
    Anzeige und Kollisionskoerper auseinander, was spaetestens bei einer
    Bahnplanung auffaellt. Es ueberschreibt aber eine womoeglich bewusst
    groebere Kollisionsform, deshalb entscheidet der Aufrufer.
    """
    link = next((l for l in root.findall("link")
                 if l.get("name") == link_name), None)
    if link is None:
        raise ValueError(f"Link {link_name} ist nicht mehr im Baum")

    visual = link.find("visual")
    if visual is None:
        visual = ET.SubElement(link, "visual")

    geometry = visual.find("geometry")
    if geometry is None:
        geometry = ET.SubElement(visual, "geometry")
    write_shape(geometry, shape, size, filename)
    write_origin(visual, xyz, rpy)

    material = visual.find("material")
    if material is None:
        material = ET.SubElement(visual, "material")
    material.set("name", f"{link_name}_material")
    colour = material.find("color")
    if colour is None:
        colour = ET.SubElement(material, "color")
    colour.set("rgba", " ".join(f"{v:.4g}" for v in rgba))

    if with_collision:
        collision = link.find("collision")
        if collision is None:
            collision = ET.SubElement(link, "collision")
        target = collision.find("geometry")
        if target is None:
            target = ET.SubElement(collision, "geometry")
        write_shape(target, shape, size, filename)
        write_origin(collision, xyz, rpy)
    return visual


def write_origin(parent, xyz, rpy):
    """<origin> setzen oder anlegen. Dieselbe Genauigkeit wie bei Gelenken."""
    origin = parent.find("origin")
    if origin is None:
        origin = ET.SubElement(parent, "origin")
    origin.set("xyz", fmt_origin(xyz))
    origin.set("rpy", fmt_origin(rpy))
    return origin


def write_shape(geometry, shape, size, filename):
    """<geometry> auf genau eine Form setzen. Alte Formen fallen weg."""
    for kind in SHAPES:
        for old in geometry.findall(kind):
            geometry.remove(old)
    node = ET.SubElement(geometry, shape)
    if shape == "box":
        node.set("size", " ".join(f"{size[k]:.6g}"
                                  for k in ("sx", "sy", "sz")))
    elif shape == "cylinder":
        node.set("radius", f"{size['radius']:.6g}")
        node.set("length", f"{size['length']:.6g}")
    elif shape == "sphere":
        node.set("radius", f"{size['radius']:.6g}")
    else:
        node.set("filename", filename)
        node.set("scale", " ".join([f"{size['scale']:.6g}"] * 3))
    return node


def read_mimic(joint):
    """<mimic> eines Gelenks als dict oder None.

    Die mechanische Kopplung ist kein Zusatz der Oberflaeche, sondern ein
    Element der URDF:

        <mimic joint="joint5" multiplier="-0.6" offset="0"/>

    Damit gilt  q = multiplier * q(joint5) + offset. Der Wert wird also nicht
    mehr gestellt, sondern gerechnet -- das gekoppelte Gelenk verliert seinen
    Freiheitsgrad. Weil es in der Datei steht, halten sich auch
    robot_state_publisher und MoveIt daran und nicht nur diese Oberflaeche.
    """
    mimic = joint.find("mimic")
    if mimic is None or not mimic.get("joint"):
        return None
    return {"joint": mimic.get("joint"),
            "multiplier": num(mimic, "multiplier", 1.0),
            "offset": num(mimic, "offset", 0.0)}


def set_mimic(root, joint_name, source, multiplier, offset):
    """Kopplung setzen oder aendern."""
    element = next((j for j in root.findall("joint")
                    if j.get("name") == joint_name), None)
    if element is None:
        raise ValueError(f"Gelenk {joint_name} ist nicht mehr im Baum")
    mimic = element.find("mimic")
    if mimic is None:
        mimic = ET.SubElement(element, "mimic")
    mimic.set("joint", source)
    mimic.set("multiplier", f"{multiplier:.6g}")
    mimic.set("offset", fmt_origin([offset]))
    return mimic


def clear_mimic(root, joint_name):
    """Kopplung entfernen. Das Gelenk ist danach wieder frei stellbar."""
    element = next((j for j in root.findall("joint")
                    if j.get("name") == joint_name), None)
    if element is None:
        raise ValueError(f"Gelenk {joint_name} ist nicht mehr im Baum")
    for mimic in element.findall("mimic"):
        element.remove(mimic)


def drop_stale_mimics(root):
    """Kopplungen entfernen, die ins Leere zeigen oder sinnlos geworden sind.

    Wird nach jedem Loeschen gerufen. Ohne das bliebe nach dem Entfernen des
    Fuehrungsgelenks ein <mimic> stehen, das auf nichts mehr verweist -- die
    Pruefung faengt es zwar, aber dann waere das Loeschen selbst blockiert,
    obwohl der Bediener nichts falsch gemacht hat.
    """
    known = {j.get("name"): j.get("type") for j in root.findall("joint")}
    for joint in root.findall("joint"):
        mimic = joint.find("mimic")
        if mimic is None:
            continue
        source = mimic.get("joint")
        if (source not in known or source == joint.get("name")
                or known[source] == "fixed" or joint.get("type") == "fixed"):
            joint.remove(mimic)


def coupling_problem(joints, name, source):
    """Meldung zur geplanten Kopplung oder None.

    Ketten sind bewusst verboten: koppelte man A an B und B an C, muesste die
    Auswertung eine Reihenfolge finden, und tf2, KDL und MoveIt loesen solche
    Ketten unterschiedlich oder gar nicht auf. Eine Ebene deckt den Zweck ab
    -- ein Gelenk folgt einem angetriebenen Gelenk.
    """
    follower = next((j for j in joints if j["name"] == name), None)
    leader = next((j for j in joints if j["name"] == source), None)
    if follower is None:
        return f"Gelenk {name} gibt es nicht"
    if leader is None:
        return f"Gelenk {source} gibt es nicht"
    if name == source:
        return "Ein Gelenk kann nicht sich selbst folgen"
    if follower["type"] == "fixed" or leader["type"] == "fixed":
        return "Feste Gelenke lassen sich nicht koppeln"
    if leader["mimic"] is not None:
        return (f"{source} folgt selbst schon {leader['mimic']['joint']}. "
                f"Ketten von Kopplungen sind nicht vorgesehen.")
    if any(j["mimic"] is not None and j["mimic"]["joint"] == name
           for j in joints):
        return (f"{name} ist selbst Fuehrungsgelenk. Erst die Kopplung dort "
                f"loesen.")
    return None


def apply_couplings(joints, positions):
    """Gerechnete Werte der gekoppelten Gelenke eintragen. Aendert das dict.

    Ein Durchlauf genuegt, weil Ketten verboten sind: jedes Fuehrungsgelenk
    ist frei stellbar und steht bereits im dict.
    """
    for joint in joints:
        mimic = joint["mimic"]
        if mimic is None or joint["type"] == "fixed":
            continue
        driver = positions.get(mimic["joint"], 0.0)
        value = mimic["multiplier"] * driver + mimic["offset"]
        if joint["has_limit"]:
            value = min(max(value, joint["lower"]), joint["upper"])
        positions[joint["name"]] = value
    return positions


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
            "visual": read_visual(root, link),
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
            "mimic": read_mimic(joint),
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


def set_axis(element, axis):
    """<axis> setzen oder entfernen. axis None heisst: Typ braucht keine."""
    existing = element.find("axis")
    if axis is None:
        if existing is not None:
            element.remove(existing)
        return
    if existing is None:
        existing = ET.SubElement(element, "axis")
    existing.set("xyz", " ".join(f"{v:.6g}" for v in axis))


def set_limit(element, lower, upper, keep=False):
    """<limit> setzen oder entfernen.

    Bei `keep` bleibt das Element mit effort und velocity erhalten, nur die
    Anschlaege verschwinden. Das ist der Fall continuous: die Achse dreht
    durch, die Grenzwerte fuer Moment und Drehzahl gelten weiter.
    """
    existing = element.find("limit")
    if lower is None:
        if existing is None:
            if not keep:
                return
            # continuous ohne <limit>: Moment und Drehzahl gehoeren trotzdem
            # hinein, sonst fehlen sie nachgelagerten Werkzeugen.
            existing = ET.SubElement(element, "limit")
            existing.set("effort", DEFAULT_EFFORT)
            existing.set("velocity", DEFAULT_VELOCITY)
            return
        if not keep:
            element.remove(existing)
            return
        for attribute in ("lower", "upper"):
            existing.attrib.pop(attribute, None)
        return
    if existing is None:
        existing = ET.SubElement(element, "limit")
    # In der URDF Pflichtangaben, vorhandene Werte nicht ueberschreiben.
    existing.set("effort", existing.get("effort") or DEFAULT_EFFORT)
    existing.set("velocity", existing.get("velocity") or DEFAULT_VELOCITY)
    existing.set("lower", f"{lower:.6g}")
    existing.set("upper", f"{upper:.6g}")


def name_problem(name, kind, taken):
    """Meldung zum Namen oder None, wenn er brauchbar ist."""
    name = (name or "").strip()
    if not name:
        return f"{kind}: Name fehlt"
    if not NAME_PATTERN.match(name):
        return (f'{kind} "{name}": erlaubt sind Buchstabe oder Unterstrich am '
                f"Anfang, danach Buchstaben, Ziffern, _ . -")
    if name in taken:
        return f'{kind} "{name}" ist schon vergeben'
    return None


def subtree_links(joints, start):
    """`start` und alles, was daran haengt. Kreisfest."""
    kids = {}
    for joint in joints:
        kids.setdefault(joint["parent"], []).append(joint["child"])
    found, stack = [], [start]
    while stack:
        name = stack.pop()
        if name in found:                              # Kreis, nicht zweimal
            continue
        found.append(name)
        stack.extend(kids.get(name, []))
    return found


def append_limb(root, link_name, joint_name, jtype, parent,
                xyz, rpy, axis, lower, upper):
    """Neuen Link samt zufuehrendem Gelenk anhaengen.

    Beides zusammen, nie einzeln: ein Link ohne Gelenk waere ein zweiter
    Wurzel-Link, ein Gelenk ohne Link haette kein Ziel. Der Baum bleibt so in
    jedem Zwischenschritt gueltig.
    """
    link = ET.SubElement(root, "link")
    link.set("name", link_name)

    joint = ET.SubElement(root, "joint")
    joint.set("name", joint_name)
    joint.set("type", jtype)
    ET.SubElement(joint, "parent").set("link", parent)
    ET.SubElement(joint, "child").set("link", link_name)
    origin = ET.SubElement(joint, "origin")
    origin.set("xyz", fmt_origin(xyz))
    origin.set("rpy", fmt_origin(rpy))
    set_axis(joint, axis)
    set_limit(joint, lower, upper, keep=(jtype == "continuous"))
    return joint


def insert_limb(root, link_name, joint_name, jtype, below,
                xyz, rpy, axis, lower, upper, keep_pose):
    """Neues Glied zwischen zwei bestehende haengen.

    Aus   A --below--> B   wird   A --joint_name--> N --below--> B.

    Drei Eingriffe, die nur zusammen einen gueltigen Baum ergeben und deshalb
    im selben Entwurf laufen muessen:

        1. Link N und das zufuehrende Gelenk von A nach N anlegen,
        2. bei `below` den Elternteil von A auf N umschreiben,
        3. den Ursprung von `below` umrechnen, falls gewuenscht.

    Punkt 2 ist der Unterschied zu append_limb: dort entsteht immer ein Blatt,
    weil kein bestehendes Gelenk angefasst wird. Hier wandert ein vorhandener
    Teilbaum an eine neue Stelle, und genau deshalb bleibt die Kette eine
    Kette statt sich zu verzweigen.

    Zu Punkt 3: der Ursprung von `below` ist in A angeschrieben. Sitzt N mit
    T_neu unter A, so beschreibt derselbe Zahlensatz jetzt eine Lage relativ
    zu N und der ganze Teilbaum hinter B wandert um T_neu mit. Soll er
    stehenbleiben, bekommt `below` neu

        T_rest = T_neu^-1 * T_alt

    denn dann gilt wieder T_neu * T_rest = T_alt. Beides kann gewollt sein --
    ein Sensorframe zieht man nachtraeglich ein, ohne dass sich der Roboter
    bewegt, ein echtes Zwischenstueck verlaengert die Kette absichtlich --
    deshalb entscheidet der Aufrufer und nicht diese Funktion.
    """
    existing = next((j for j in root.findall("joint")
                     if j.get("name") == below), None)
    if existing is None:
        raise ValueError(f"Gelenk {below} ist nicht mehr im Baum")
    parent_element = existing.find("parent")
    if parent_element is None:
        raise ValueError(f"Gelenk {below} hat kein <parent>")

    # Erst das neue Glied, dann umhaengen. Andere Reihenfolge ginge auch, so
    # ist der Entwurf aber nie in einem Zustand ohne Ziel fuer <parent>.
    joint = append_limb(root, link_name, joint_name, jtype,
                        parent_element.get("link"),
                        xyz, rpy, axis, lower, upper)

    if keep_pose:
        origin = existing.find("origin")
        old = from_rpy_xyz(vec(origin, "rpy"), vec(origin, "xyz"))
        rest = multiply(invert(from_rpy_xyz(rpy, xyz)), old)
        rest_xyz, rest_rpy = to_xyz_rpy(rest)
        if origin is None:
            origin = ET.SubElement(existing, "origin")
        origin.set("xyz", fmt_origin(rest_xyz))
        origin.set("rpy", fmt_origin(rest_rpy))

    parent_element.set("link", link_name)
    return joint


def xml_root_links(root):
    """Wurzel-Links unmittelbar aus dem XML-Baum.

    Gegenstueck zu root_links, das auf den geparsten Listen arbeitet. Innerhalb
    einer mutate-Funktion liegt nur der Entwurf als XML vor; die geparste
    Fassung entsteht erst danach in _rebuild_from_root. Ohne diesen Helfer
    muesste man dort zwischendurch parsen, und der Entwurf ist genau in diesem
    Moment moeglicherweise noch ungueltig.
    """
    children = set()
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is not None and child.get("link"):
            children.add(child.get("link"))
    return [link.get("name") for link in root.findall("link")
            if link.get("name") not in children]


def insert_above_root(root, link_name, joint_name, xyz, rpy):
    """Neuen Link ueber die bisherige Wurzel setzen: der Basis-Offset.

    Aus   R --k--> ...   wird   N --joint_name--> R --k--> ...

    Der Gegenpol zu append_limb. Dort entsteht ein Blatt am unteren Ende der
    Kette, hier eine neue Wurzel am oberen. Beide schreiben ein
    <joint type="fixed">; welche Bedeutung es hat, entscheidet allein die
    Stelle.

    insert_limb deckt den Fall nicht ab, obwohl er verwandt aussieht: es
    braucht `below`, also ein bestehendes Gelenk, dessen Elternteil
    umgeschrieben wird. Ueber der Wurzel fuehrt kein Gelenk hin, es gaebe
    nichts umzuschreiben.

    Kein keep_pose. Die Frage stellt sich nur, wenn oberhalb etwas steht, zu
    dem eine Lage erhalten bleiben koennte; die Wurzel hat definitionsgemaess
    nichts ueber sich. Der ganze Baum haengt danach mit T_neu unter N, und
    genau das ist gemeint: der Roboter steht irgendwo in der Welt.

    Was sich dadurch aendert, ist nicht die Mechanik, sondern der Bezug.
    forward_kinematics rechnet gegen die Wurzel, und die heisst ab jetzt
    anders. Jede Zahl in der Frame-Tabelle ist danach gegen N angeschrieben
    statt gegen R -- fehlerfrei, unauffaellig und trotzdem eine andere
    Aussage. Deshalb nennt die TCP-Zeile ihren Bezug ausdruecklich.
    """
    tops = xml_root_links(root)
    if len(tops) != 1:
        raise ValueError(
            "Ein Basisframe braucht genau eine Wurzel. Gefunden: "
            + (", ".join(sorted(tops)) if tops else "keine"))

    link = ET.SubElement(root, "link")
    link.set("name", link_name)

    joint = ET.SubElement(root, "joint")
    joint.set("name", joint_name)
    joint.set("type", "fixed")
    ET.SubElement(joint, "parent").set("link", link_name)
    ET.SubElement(joint, "child").set("link", tops[0])
    origin = ET.SubElement(joint, "origin")
    origin.set("xyz", fmt_origin(xyz))
    origin.set("rpy", fmt_origin(rpy))
    return joint


def rename_link(root, old, new):
    """Einen Link umbenennen und alle Verweise darauf mitziehen.

    Ein Name steht in URDF an mehr Stellen als der einen, an der er vergeben
    wird. Wird nur <link name> geaendert, zeigen die Gelenke ins Leere und die
    Pruefung meldet einen zweiten Wurzel-Link -- der Baum zerfaellt.

    Mitgezogen werden:
        <link name>                    die Vergabe selbst
        <joint><parent link>           Verweis von oben
        <joint><child link>            Verweis von unten
        <gazebo reference>             Verweis aus der Simulation
        <material name>                nur das eigene, siehe unten

    Das Material wird nur umbenannt, wenn es der Regel {link}_material folgt,
    also von set_visual angelegt wurde und niemandem sonst gehoert. Ein
    gemeinsames Material bleibt unangetastet, sonst faerbte ein Umbenennen
    fremde Links um.
    """
    element = next((l for l in root.findall("link")
                    if l.get("name") == old), None)
    if element is None:
        raise ValueError(f"Link {old} ist nicht mehr im Baum")
    element.set("name", new)

    for joint in root.findall("joint"):
        for side in ("parent", "child"):
            hook = joint.find(side)
            if hook is not None and hook.get("link") == old:
                hook.set("link", new)

    for tag in root.findall("gazebo"):
        if tag.get("reference") == old:
            tag.set("reference", new)

    for material in element.iter("material"):
        if material.get("name") == f"{old}_material":
            material.set("name", f"{new}_material")
    return element


def rename_joint(root, old, new):
    """Ein Gelenk umbenennen und alle Verweise darauf mitziehen.

    Anders als beim Link haengt an einem Gelenknamen keine Topologie: parent
    und child stehen im Gelenk selbst. Verweise gibt es trotzdem, und sie sind
    leichter zu uebersehen, weil sie ausserhalb des Gelenks stehen:

        <joint name>                   die Vergabe selbst
        <mimic joint>                  ein Gelenk, das diesem folgt
        <transmission><joint name>     Zuordnung zum Antrieb
        <gazebo reference>             Verweis aus der Simulation
    """
    element = next((j for j in root.findall("joint")
                    if j.get("name") == old), None)
    if element is None:
        raise ValueError(f"Gelenk {old} ist nicht mehr im Baum")
    element.set("name", new)

    for mimic in root.iter("mimic"):
        if mimic.get("joint") == old:
            mimic.set("joint", new)

    for transmission in root.findall("transmission"):
        for joint in transmission.findall("joint"):
            if joint.get("name") == old:
                joint.set("name", new)

    for tag in root.findall("gazebo"):
        if tag.get("reference") == old:
            tag.set("reference", new)
    return element


def remove_limb(root, joint_name, keep_pose):
    """Ein einzelnes Glied aus der Kette nehmen, ohne den Rest zu verlieren.

    Aus   A --joint_name--> B --k--> C   wird   A --k--> C.

    Die Umkehrung von insert_limb und der Gegenpol zu remove_branch: dort
    faellt alles hinter dem Gelenk weg, hier nur das eine Glied. Was daran
    hing, wandert eine Stufe hoch.

    Link und Gelenk verschwinden zwangslaeufig gemeinsam, so wie sie in
    append_limb gemeinsam entstehen. Bliebe das Gelenk ohne Kindlink, haette
    es kein Ziel; bliebe der Link ohne zufuehrendes Gelenk, waere er ein
    zweiter Wurzel-Link. Beides waere keine gueltige URDF.

    Zu `keep_pose`: die Urspruenge der Kindgelenke sind in B angeschrieben.
    Faellt B weg, muessen sie in A stehen, also

        T_neu = T_joint_name * T_alt

    denn genau diese Verkettung lief vorher ueber B. Ist das Gelenk beweglich,
    gilt die Rechnung fuer Stellung null: mit dem Gelenk verschwindet auch
    sein Freiheitsgrad, eine andere Stellung liesse sich gar nicht erhalten.
    """
    element = next((j for j in root.findall("joint")
                    if j.get("name") == joint_name), None)
    if element is None:
        raise ValueError(f"Gelenk {joint_name} ist nicht mehr im Baum")
    parent_element = element.find("parent")
    child_element = element.find("child")
    if parent_element is None or child_element is None:
        raise ValueError(f"Gelenk {joint_name} hat kein <parent> oder <child>")

    above = parent_element.get("link")
    gone = child_element.get("link")
    origin = element.find("origin")
    lift = from_rpy_xyz(vec(origin, "rpy"), vec(origin, "xyz"))

    for other in root.findall("joint"):
        hook = other.find("parent")
        if hook is None or hook.get("link") != gone:
            continue
        if keep_pose:
            child_origin = other.find("origin")
            rest = multiply(lift, from_rpy_xyz(vec(child_origin, "rpy"),
                                               vec(child_origin, "xyz")))
            rest_xyz, rest_rpy = to_xyz_rpy(rest)
            if child_origin is None:
                child_origin = ET.SubElement(other, "origin")
            child_origin.set("xyz", fmt_origin(rest_xyz))
            child_origin.set("rpy", fmt_origin(rest_rpy))
        hook.set("link", above)

    root.remove(element)
    for link in list(root.findall("link")):
        if link.get("name") == gone:
            root.remove(link)
    drop_stale_mimics(root)
    return gone


def remove_branch(root, link_names):
    """Links und alle Gelenke entfernen, die dorthin fuehren."""
    names = set(link_names)
    for link in list(root.findall("link")):
        if link.get("name") in names:
            root.remove(link)
    for joint in list(root.findall("joint")):
        child = joint.find("child")
        if child is not None and child.get("link") in names:
            root.remove(joint)
    drop_stale_mimics(root)


def backup_dir_for(path):
    """Ablageordner, wird bei Bedarf angelegt."""
    if os.path.isabs(BACKUP_DIR):
        folder = BACKUP_DIR
    else:
        folder = os.path.join(os.path.dirname(os.path.abspath(path)), BACKUP_DIR)
    os.makedirs(folder, exist_ok=True)
    return folder

def bahn_dir_for(path):
    """Ablageordner fuer exportierte Bahnen, wird bei Bedarf angelegt.

    Ohne eigenen Eintrag landet die Bahn dort, wo auch die URDF-Staende
    liegen -- das war das Verhalten vor dieser Trennung.
    """
    if not BAHN_DIR:
        return backup_dir_for(path)
    if os.path.isabs(BAHN_DIR):
        folder = BAHN_DIR
    else:
        folder = os.path.join(os.path.dirname(os.path.abspath(path)), BAHN_DIR)
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


def duplicates(names):
    """Namen, die mehr als einmal vorkommen."""
    seen, twice = set(), set()
    for name in names:
        if name in seen:
            twice.add(name)
        seen.add(name)
    return sorted(twice)


def validate_root(root):
    """Prueft den Baum. Gibt eine Liste von Fehlermeldungen zurueck.

    Die Pruefung ist die einzige Instanz, die den Bediener vor einem kaputten
    Modell schuetzt: robot_state_publisher rechnet nur Vorwaertskinematik und
    beanstandet nichts.
    """
    errors = []
    joints = root.findall("joint")
    link_elements = root.findall("link")
    link_names = [l.get("name") for l in link_elements]
    links = {n for n in link_names if n}
    children = []

    # -- Namen ------------------------------------------------------------
    if sum(1 for n in link_names if not (n or "").strip()):
        errors.append("Ein Link hat keinen Namen")
    for name in duplicates([n for n in link_names if n]):
        errors.append(f'Linkname "{name}" ist mehrfach vergeben')
    for name in duplicates([j.get("name") for j in joints if j.get("name")]):
        errors.append(f'Gelenkname "{name}" ist mehrfach vergeben')

    # -- Kopplungen -------------------------------------------------------
    joint_names = {j.get("name") for j in joints}
    followers = {}
    for joint in joints:
        mimic = joint.find("mimic")
        if mimic is None:
            continue
        name, source = joint.get("name"), mimic.get("joint")
        if not source:
            errors.append(f"{name}: <mimic> ohne Gelenkangabe")
        elif source not in joint_names:
            errors.append(f'{name}: folgt "{source}", das es nicht gibt')
        elif source == name:
            errors.append(f"{name}: folgt sich selbst")
        else:
            followers[name] = source
    for name, source in followers.items():
        if source in followers:
            errors.append(f"{name}: folgt {source}, das selbst "
                          f"{followers[source]} folgt. Ketten sind nicht "
                          f"vorgesehen.")

    for joint in joints:
        name = joint.get("name")
        if not (name or "").strip():
            errors.append("Ein Gelenk hat keinen Namen")
            continue

        # -- Typ ----------------------------------------------------------
        jtype = joint.get("type")
        if jtype not in JOINT_TYPES:
            errors.append(f'{name}: Typ "{jtype}" ist keiner der erlaubten '
                          f'({", ".join(JOINT_TYPES)})')

        # -- Verknuepfung -------------------------------------------------
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

        # -- Anschlaege ---------------------------------------------------
        limit = joint.find("limit")
        if jtype in NEEDS_LIMIT:
            if limit is None:
                errors.append(f"{name}: {jtype} braucht <limit>, sonst ist die "
                              f"Achse unbegrenzt")
            else:
                for required in ("effort", "velocity"):
                    if limit.get(required) is None:
                        errors.append(f"{name}: <limit> ohne {required}")
                try:
                    lower = float(limit.get("lower", 0.0))
                    upper = float(limit.get("upper", 0.0))
                except ValueError:
                    errors.append(f"{name}: Grenzwert ist keine Zahl")
                else:
                    if lower >= upper:
                        errors.append(f"{name}: untere Grenze muss kleiner als "
                                      f"die obere sein")

        # -- Achse --------------------------------------------------------
        if jtype in NEEDS_AXIS:
            axis = joint.find("axis")
            if axis is not None:
                try:
                    values = [float(v) for v in (axis.get("xyz") or "").split()]
                except ValueError:
                    values = []
                if len(values) != 3:
                    errors.append(f"{name}: <axis> braucht drei Zahlen")
                elif math.sqrt(sum(v * v for v in values)) < 1e-9:
                    errors.append(f"{name}: <axis> ist der Nullvektor")

    if len(set(children)) != len(children):
        errors.append(f"Mehrere Eltern fuer: {', '.join(duplicates(children))}")

    roots = links - set(children)
    if len(roots) != 1:
        errors.append(f"Genau ein Wurzel-Link erwartet, gefunden: {len(roots)}")

    # -- Kreise -----------------------------------------------------------
    # Ein Kreis faellt sonst durch: jeder seiner Links hat genau einen Vater
    # und ist damit kein Wurzel-Link. Die FK laesst ihn stillschweigend weg.
    reachable = set(roots)
    pending = [j for j in joints
               if j.find("parent") is not None and j.find("child") is not None]
    while pending:
        progressed = False
        for joint in list(pending):
            if joint.find("parent").get("link") in reachable:
                reachable.add(joint.find("child").get("link"))
                pending.remove(joint)
                progressed = True
        if not progressed:
            break
    unreachable = sorted(links - reachable)
    if unreachable:
        errors.append(f"Nicht von der Wurzel erreichbar, vermutlich ein Kreis: "
                      f"{', '.join(unreachable)}")

    return errors
