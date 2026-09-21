"""kinematik -- Vorwaertskinematik und Layout des Gelenkbaums. Kein Tkinter,
kein ROS.

Seit urdf_gui2_1 ein eigenes Modul. Der Inhalt ist zeilengleich aus
urdf_gui3_7.py verschoben (dort Zeilen 2499-2605); neu sind
nur dieser Kopf, die Importe und STAND.
"""

from stil import (TREE_GAP_X, TREE_GAP_Y, TREE_MARGIN, TREE_NODE_H,
                  TREE_NODE_W)
from matrix import (from_axis_angle, from_translation, identity, multiply)

# Muss zu MODUL_STAENDE in urdf_gui2_1.py passen.
STAND = 1


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


def tree_layout(links, joints):
    """Mitte und Oberkante jedes Linkkastens. Gibt {linkname: (x, y)} zurueck.

    Zweipass, wie beim Setzen einer Tabelle:

        1. von unten nach oben: wie breit ist der Teilbaum unter einem Link,
        2. von oben nach unten: diese Breiten nebeneinander legen und jeden
           Link mittig ueber seine Kinder setzen.

    Ohne den ersten Durchlauf wuesste man beim Platzieren des Vaters noch
    nicht, wie viel Platz die Kinder brauchen; man muesste raten und spaeter
    verschieben. Die Tiefe bestimmt allein die Zeile, deshalb liegen alle
    Kinder eines Links garantiert auf einer Hoehe.

    `seen` bricht Kreise ab. Ein Kreis ist keine gueltige URDF, aber die
    Ansicht wird auch fuer eine geladene Datei gezeichnet, die die Pruefung
    nur bemaengelt hat. Links, die von keiner Wurzel aus erreichbar sind,
    tauchen nicht auf; der Aufrufer sieht das am Vergleich der Anzahlen.
    """
    kids = {}
    for joint in joints:
        kids.setdefault(joint["parent"], []).append(joint)

    width = {}

    def measure(link, seen):
        if link in seen:
            return TREE_NODE_W
        seen = seen | {link}
        spans = [measure(j["child"], seen) for j in kids.get(link, ())]
        total = sum(spans) + TREE_GAP_X * (len(spans) - 1) if spans else 0.0
        width[link] = max(TREE_NODE_W, total)
        return width[link]

    def place(link, left, depth, seen):
        if link in seen:
            return
        seen = seen | {link}
        nodes[link] = (left + width[link] / 2.0,
                       TREE_MARGIN + depth * (TREE_NODE_H + TREE_GAP_Y))
        offset = left
        for joint in kids.get(link, ()):
            child = joint["child"]
            if child in seen:
                continue
            place(child, offset, depth + 1, seen)
            offset += width[child] + TREE_GAP_X

    nodes = {}
    left = TREE_MARGIN
    # Mehrere Wurzeln sind ungueltig, kommen beim Laden einer fremden Datei
    # aber vor. Dann stehen die Baeume nebeneinander, statt dass die Ansicht
    # leer bleibt und der Fehler unsichtbar wird.
    for root in root_links(links, joints):
        measure(root, frozenset())
        place(root, left, 0, frozenset())
        left += width[root] + 3 * TREE_GAP_X
    return nodes


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
