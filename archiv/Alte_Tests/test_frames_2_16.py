#!/usr/bin/env python3
"""Pruefungen zu den Frames und Werkzeugen aus 2.16.

Logikschicht, ohne Tkinter und ohne ROS. Die beiden Methoden, die eine
Instanz brauchen, werden gegen einen Stub gerufen: sie lesen nur Listen und
schreiben ins Protokoll, beides laesst sich nachbilden.
"""

import math
import sys
import xml.etree.ElementTree as ET

import urdf_gui2_16 as G

CHECKS = []


def check(name, condition, detail=""):
    CHECKS.append((name, bool(condition), detail))


def near(a, b, tol=1e-9):
    return abs(a - b) <= tol


MINI = """<robot name="mini">
  <link name="base_link"/>
  <link name="arm"/>
  <link name="flansch"/>
  <joint name="a1" type="revolute">
    <parent link="base_link"/><child link="arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="100" velocity="1"/>
  </joint>
  <joint name="a2" type="revolute">
    <parent link="arm"/><child link="flansch"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="100" velocity="1"/>
  </joint>
</robot>"""


def tree():
    return ET.fromstring(MINI)


class Stub:
    """Traegt genau das, was _tcp_link und _sync_tip anfassen."""

    def __init__(self, links, joints, tip=""):
        self.links = links
        self.joints = joints
        self.moveit_config = type("C", (), {"tip": tip})()
        self.messages = []

    def _log(self, text, level="info"):
        self.messages.append((level, text))


def parsed(root):
    _name, links, joints = G.parse_root(root)
    return links, joints


# ==========================================================================
# 1..4  xml_root_links
# ==========================================================================

root = tree()
check("01 eine Wurzel erkannt", G.xml_root_links(root) == ["base_link"],
      G.xml_root_links(root))

check("02 Blaetter sind keine Wurzeln", "flansch" not in G.xml_root_links(root))

loose = tree()
ET.SubElement(loose, "link").set("name", "einzeln")
check("03 zweite Wurzel wird mitgezaehlt",
      sorted(G.xml_root_links(loose)) == ["base_link", "einzeln"])

empty = ET.fromstring('<robot name="leer"><link name="nur"/></robot>')
check("04 einzelner Link ist Wurzel", G.xml_root_links(empty) == ["nur"])


# ==========================================================================
# 5..14  insert_above_root
# ==========================================================================

root = tree()
G.insert_above_root(root, "world", "welt_basis", [1.0, 2.0, 0.5],
                    [0.0, 0.0, math.radians(90.0)])

check("05 neue Wurzel ist world", G.xml_root_links(root) == ["world"])

names = {l.get("name") for l in root.findall("link")}
check("06 alter Wurzel-Link bleibt", "base_link" in names)
check("07 neuer Link angelegt", "world" in names)

joint = next(j for j in root.findall("joint") if j.get("name") == "welt_basis")
check("08 Gelenktyp fest", joint.get("type") == "fixed")
check("09 Elternteil ist der neue Link",
      joint.find("parent").get("link") == "world")
check("10 Kind ist die alte Wurzel",
      joint.find("child").get("link") == "base_link")
check("11 kein Achseintrag", joint.find("axis") is None)
check("12 kein Anschlag", joint.find("limit") is None)

xyz = [float(v) for v in joint.find("origin").get("xyz").split()]
rpy = [float(v) for v in joint.find("origin").get("rpy").split()]
check("13 Laenge steht in der Datei", xyz == [1.0, 2.0, 0.5], xyz)
check("14 Winkel als Yaw geschrieben, Rundung unter 1e-9",
      near(rpy[2], math.radians(90.0)) and rpy[0] == 0.0 and rpy[1] == 0.0, rpy)


# ==========================================================================
# 15..17  insert_above_root verweigert mehrdeutige Baeume
# ==========================================================================

broken = tree()
ET.SubElement(broken, "link").set("name", "einzeln")
try:
    G.insert_above_root(broken, "world", "welt_basis", [0, 0, 0], [0, 0, 0])
    check("15 zwei Wurzeln werden abgewiesen", False, "keine Ausnahme")
except ValueError as exc:
    check("15 zwei Wurzeln werden abgewiesen", True)
    check("16 Meldung nennt beide Wurzeln",
          "base_link" in str(exc) and "einzeln" in str(exc), str(exc))

check("17 Entwurf bleibt bei Abweisung unveraendert",
      not any(j.get("name") == "welt_basis" for j in broken.findall("joint")))


# ==========================================================================
# 18..22  Baum bleibt gueltig, Kette bleibt Kette
# ==========================================================================

root = tree()
G.insert_above_root(root, "world", "welt_basis", [1.0, 0.0, 0.0], [0, 0, 0])
problems = G.validate_root(root)
check("18 Pruefung ohne Beanstandung", not problems, problems)

links, joints = parsed(root)
check("19 vier Links", len(links) == 4, len(links))
check("20 drei Gelenke", len(joints) == 3, len(joints))
check("21 genau ein Blatt ausser der Wurzel",
      [n for n in G.leaf_links(links, joints) if n != "world"] == ["flansch"])
check("22 keine Verzweigung entstanden",
      len(G.root_links(links, joints)) == 1)


# ==========================================================================
# 23..29  Basis-Offset multipliziert links
# ==========================================================================

base_links, base_joints = parsed(tree())
positions = {"a1": math.radians(30.0), "a2": math.radians(-20.0)}
before = G.forward_kinematics(base_links, base_joints, positions)["flansch"]

shifted = tree()
G.insert_above_root(shifted, "world", "welt_basis", [1.0, 2.0, 0.5],
                    [0.0, 0.0, math.radians(90.0)])
s_links, s_joints = parsed(shifted)
after = G.forward_kinematics(s_links, s_joints, positions)["flansch"]

offset = G.from_rpy_xyz([0.0, 0.0, math.radians(90.0)], [1.0, 2.0, 0.5])
expected = G.multiply(offset, before)

worst = max(abs(after[r][c] - expected[r][c])
            for r in range(4) for c in range(4))
check("23 T_neu = T_Basis * T_alt", worst < 1e-9, f"groesste Abweichung {worst:g}")

check("24 base_link liegt jetzt beim Offset",
      near(G.forward_kinematics(s_links, s_joints, positions)["base_link"][0][3],
           1.0))

# Der Roboter selbst hat sich nicht bewegt: die Lage des Flansches gegen
# base_link ist dieselbe wie vorher.
rel = G.multiply(G.invert(
    G.forward_kinematics(s_links, s_joints, positions)["base_link"]), after)
worst = max(abs(rel[r][c] - before[r][c]) for r in range(4) for c in range(4))
check("25 Lage gegen base_link unveraendert", worst < 1e-9,
      f"groesste Abweichung {worst:g}")

# Ohne Achsbewegung wirkt der Offset genauso: er haengt nicht von q ab.
zero = {"a1": 0.0, "a2": 0.0}
b0 = G.forward_kinematics(base_links, base_joints, zero)["flansch"]
a0 = G.forward_kinematics(s_links, s_joints, zero)["flansch"]
e0 = G.multiply(offset, b0)
worst = max(abs(a0[r][c] - e0[r][c]) for r in range(4) for c in range(4))
check("26 gilt auch in Nullstellung", worst < 1e-9, f"{worst:g}")

check("27 Achssatz unveraendert",
      {j["name"] for j in s_joints if j["type"] != "fixed"}
      == {"a1", "a2"})

check("28 der neue Frame traegt keinen Freiheitsgrad",
      next(j for j in s_joints if j["name"] == "welt_basis")["type"] == "fixed")

check("29 Wurzel heisst world", G.root_links(s_links, s_joints) == ["world"])


# ==========================================================================
# 30..37  Werkzeug: append_limb mit fixed
# ==========================================================================

tool = tree()
G.append_limb(tool, "tcp1", "frame1", "fixed", "flansch",
              [0.0, 0.0, 0.12], [0.0, math.radians(45.0), 0.0],
              None, None, None)
check("30 Werkzeugbaum gueltig", not G.validate_root(tool), G.validate_root(tool))

j = next(x for x in tool.findall("joint") if x.get("name") == "frame1")
check("31 fest", j.get("type") == "fixed")
check("32 keine Achse", j.find("axis") is None)
check("33 kein Anschlag", j.find("limit") is None)
check("34 haengt unter dem Flansch", j.find("parent").get("link") == "flansch")

t_links, t_joints = parsed(tool)
check("35 Wurzel unveraendert", G.root_links(t_links, t_joints) == ["base_link"])
check("36 neues Blatt ist der TCP",
      [n for n in G.leaf_links(t_links, t_joints) if n != "base_link"] == ["tcp1"])

# Werkzeug-Offset multipliziert rechts: die Lage des TCP ist die des
# Flansches mal dem Offset.
poses = G.forward_kinematics(t_links, t_joints, positions)
expected = G.multiply(poses["flansch"],
                      G.from_rpy_xyz([0.0, math.radians(45.0), 0.0],
                                     [0.0, 0.0, 0.12]))
worst = max(abs(poses["tcp1"][r][c] - expected[r][c])
            for r in range(4) for c in range(4))
check("37 T_TCP = T_Flansch * T_Werkzeug", worst < 1e-9, f"{worst:g}")


# ==========================================================================
# 38..46  _tcp_link: Wahl vor Ableitung
# ==========================================================================

t_links, t_joints = parsed(tool)
stub = Stub(t_links, t_joints, tip="")
check("38 ohne Wahl das einzige Blatt",
      G.UrdfViewer._tcp_link(stub) == "tcp1")

stub = Stub(t_links, t_joints, tip="flansch")
check("39 Wahl schlaegt die Ableitung",
      G.UrdfViewer._tcp_link(stub) == "flansch")

stub = Stub(t_links, t_joints, tip="gibtsnicht")
check("40 unbekannte Wahl faellt auf die Ableitung zurueck",
      G.UrdfViewer._tcp_link(stub) == "tcp1")

# Zweites Werkzeug: die Ableitung hat keine Grundlage mehr.
two = tree()
G.append_limb(two, "tcp1", "frame1", "fixed", "flansch",
              [0, 0, 0.12], [0, 0, 0], None, None, None)
G.append_limb(two, "kamera", "frame2", "fixed", "flansch",
              [0.05, 0, 0.02], [0, 0, 0], None, None, None)
two_links, two_joints = parsed(two)

stub = Stub(two_links, two_joints, tip="")
check("41 zwei Blaetter ohne Wahl: kein TCP",
      G.UrdfViewer._tcp_link(stub) is None)

stub = Stub(two_links, two_joints, tip="tcp1")
check("42 zwei Blaetter mit Wahl: der gewaehlte",
      G.UrdfViewer._tcp_link(stub) == "tcp1")

stub = Stub(two_links, two_joints, tip="kamera")
check("43 die andere Wahl ebenso",
      G.UrdfViewer._tcp_link(stub) == "kamera")

check("44 beide sind Blaetter",
      sorted(n for n in G.leaf_links(two_links, two_joints)
             if n != "base_link") == ["kamera", "tcp1"])

# Wurzel darf nie TCP sein, auch nicht bei einem Modell ohne Gelenke.
solo_links, solo_joints = parsed(
    ET.fromstring('<robot name="s"><link name="nur"/></robot>'))
stub = Stub(solo_links, solo_joints, tip="")
check("45 Modell ohne Gelenk hat keinen TCP",
      G.UrdfViewer._tcp_link(stub) is None)

stub = Stub(solo_links, solo_joints, tip="nur")
check("46 ausdrueckliche Wahl gilt auch dort",
      G.UrdfViewer._tcp_link(stub) == "nur")


# ==========================================================================
# 47..52  _sync_tip
# ==========================================================================

stub = Stub(t_links, t_joints, tip="tcp1")
G.UrdfViewer._sync_tip(stub, t_links)
check("47 vorhandener Tip bleibt", stub.moveit_config.tip == "tcp1")
check("48 kein Protokolleintrag", not stub.messages)

stub = Stub(t_links, t_joints, tip="verschwunden")
G.UrdfViewer._sync_tip(stub, t_links)
check("49 verschwundener Tip wird geleert", stub.moveit_config.tip == "")
check("50 mit Warnung", stub.messages and stub.messages[0][0] == "warn",
      stub.messages)
check("51 Warnung nennt den Namen",
      stub.messages and "verschwunden" in stub.messages[0][1])

stub = Stub(t_links, t_joints, tip="")
G.UrdfViewer._sync_tip(stub, t_links)
check("52 leerer Tip loest nichts aus",
      stub.moveit_config.tip == "" and not stub.messages)


# ==========================================================================
# 53..56  Genauigkeit der Schreibstelle
# ==========================================================================

check("53 zehn Stellen fuer Urspruenge", G.ORIGIN_DIGITS == 10)

value = math.radians(37.135)
text = G.fmt_origin([value, 0.0, 0.0]).split()[0]
check("54 Winkel ueberlebt den Weg durch die Datei",
      near(float(text), value, 1e-9), f"{value!r} -> {text}")

check("55 runde Zahl bleibt rund", G.fmt_origin([0.3, 0, 0]).split()[0] == "0.3",
      G.fmt_origin([0.3, 0, 0]))

# Ueber die Kette: anlegen, herausnehmen, wieder anlegen. Genau der Fall,
# fuer den ORIGIN_DIGITS von sechs auf zehn ging.
chain = tree()
G.insert_above_root(chain, "world", "welt_basis", [0.123456789, 0, 0],
                    [0, 0, math.radians(12.3456)])
c_links, c_joints = parsed(chain)
got = next(j for j in c_joints if j["name"] == "welt_basis")
check("56 Laenge und Winkel unter 1e-9 wiedergefunden",
      near(got["xyz"][0], 0.123456789, 1e-9)
      and near(got["rpy"][2], math.radians(12.3456), 1e-9),
      f"{got['xyz'][0]!r} {got['rpy'][2]!r}")


# ==========================================================================
# 57..70  _frame_apply, die GUI-Schicht gegen Attrappen
# ==========================================================================
#
# Ohne diese Gruppe blieb die Stelle ungeprueft, an der die Rolle entscheidet,
# welche Funktion mit welchen Argumenten geschrieben wird. Eine Mutationsprobe
# hat das aufgedeckt: ein Werkzeug mit Achse und Anschlaegen anzulegen fiel
# nirgends auf, weil die Tests append_limb unmittelbar riefen statt ueber die
# Oberflaeche.

class FakeEntry:
    """Was _read_entry und _set_entry von einem Eingabefeld brauchen."""

    def __init__(self, text=""):
        self.text = text

    def get(self):
        return self.text

    def configure(self, **_kw):
        pass

    def delete(self, *_a):
        self.text = ""

    def insert(self, _index, value):
        self.text = value


class Var:
    def __init__(self, value=""):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class ApplyStub(Stub):
    """Fuehrt _frame_apply auf einem echten XML-Entwurf aus."""

    def __init__(self, xml, role, values, link="neu", joint="neu_frame",
                 parent="flansch", tip=""):
        self.xml_root = xml
        links, joints = parsed(xml)
        Stub.__init__(self, links, joints, tip)
        self.frame_role_var = Var(role)
        self.frame_parent_var = Var(parent)
        self.frame_entries = {
            "link": FakeEntry(link), "joint": FakeEntry(joint),
            "x": FakeEntry(str(values[0])), "y": FakeEntry(str(values[1])),
            "z": FakeEntry(str(values[2])), "roll": FakeEntry(str(values[3])),
            "pitch": FakeEntry(str(values[4])), "yaw": FakeEntry(str(values[5])),
        }
        self.committed = None
        self.problems = None
        self.refreshed = 0

    def _commit(self, description, mutate):
        draft = ET.fromstring(ET.tostring(self.xml_root))
        try:
            mutate(draft)
        except ValueError as exc:
            self.problems = [str(exc)]
            return False
        self.problems = G.validate_root(draft)
        if self.problems:
            return False
        self.xml_root = draft
        self.links, self.joints = parsed(draft)
        self.committed = description
        return True

    def _frame_refresh(self):
        self.refreshed += 1

    # Aus der Klasse uebernommen statt nachgebaut: nachgebaut waere es eine
    # zweite Fassung derselben Regel, und die Probe pruefte dann sie statt
    # der Anwendung.
    _read_entry = G.UrdfViewer.__dict__["_read_entry"]
    _set_entry = G.UrdfViewer.__dict__["_set_entry"]
    _suggest_name = G.UrdfViewer._suggest_name
    _tcp_link = G.UrdfViewer._tcp_link


def joint_of(xml, name):
    return next((j for j in xml.findall("joint") if j.get("name") == name), None)


# -- Werkzeug ----------------------------------------------------------------
stub = ApplyStub(tree(), G.ROLE_TOOL, [0.0, 0.0, 0.15, 0.0, 45.0, 0.0])
G.UrdfViewer._frame_apply(stub)
check("57 Werkzeug uebernommen", stub.committed is not None, stub.problems)

j = joint_of(stub.xml_root, "neu_frame")
check("58 als festes Gelenk", j is not None and j.get("type") == "fixed")
check("59 ohne Achse", j is not None and j.find("axis") is None)
check("60 ohne Anschlag", j is not None and j.find("limit") is None)
check("61 unter dem gewaehlten Link",
      j is not None and j.find("parent").get("link") == "flansch")
check("62 Winkel von grad nach rad umgerechnet",
      near(float(j.find("origin").get("rpy").split()[1]), math.radians(45.0)),
      j.find("origin").get("rpy") if j is not None else "-")
check("63 Wurzel unveraendert", G.xml_root_links(stub.xml_root) == ["base_link"])
check("64 Namensvorschlaege wurden erneuert",
      stub.frame_entries["link"].get() != "neu"
      and stub.frame_entries["joint"].get() != "neu_frame",
      (stub.frame_entries["link"].get(), stub.frame_entries["joint"].get()))
check("65 Auswahlliste nachgezogen", stub.refreshed == 1, stub.refreshed)

# -- Bezug -------------------------------------------------------------------
stub = ApplyStub(tree(), G.ROLE_BASE, [1.0, 0.0, 0.0, 0.0, 0.0, 90.0],
                 link="world", joint="welt_basis")
G.UrdfViewer._frame_apply(stub)
check("66 Bezug uebernommen", stub.committed is not None, stub.problems)
check("67 neue Wurzel", G.xml_root_links(stub.xml_root) == ["world"])

j = joint_of(stub.xml_root, "welt_basis")
check("68 auch hier fest und ohne Achse",
      j is not None and j.get("type") == "fixed" and j.find("axis") is None)
check("69 Anschlusswahl wird bei Bezug nicht benutzt",
      j is not None and j.find("child").get("link") == "base_link")

# -- Doppelter Name ----------------------------------------------------------
_msgbox = G.messagebox
G.messagebox = type("M", (), {"showerror": staticmethod(lambda *a, **k: None)})()
stub = ApplyStub(tree(), G.ROLE_TOOL, [0, 0, 0, 0, 0, 0], link="arm")
G.UrdfViewer._frame_apply(stub)
check("70 vergebener Linkname wird abgewiesen", stub.committed is None)
G.messagebox = _msgbox


# ==========================================================================
# Auswertung
# ==========================================================================

failed = [c for c in CHECKS if not c[1]]
for name, ok, detail in CHECKS:
    if not ok:
        print(f"FEHLER  {name}" + (f"   [{detail}]" if detail else ""))
print(f"\n{len(CHECKS) - len(failed)} von {len(CHECKS)} Pruefungen bestanden")
sys.exit(1 if failed else 0)
