#!/usr/bin/env python3
"""Prueft die MoveIt-Seite von urdf_gui2_4 gegen eine beliebige URDF.

Ohne Fenster und ohne ROS. Geprueft werden MoveItConfig, build_srdf und die
erzeugte Parameterdatei -- genau die Teile, die bewusst weder Tk noch rclpy
brauchen.

Aufruf:

    ./test_logic.py                          # eingebautes Beispielmodell
    ./test_logic.py ~/Display/bmKinematic.urdf
    ./test_logic.py meine.urdf --dump /tmp/moveit_check

Mit einer eigenen Datei koennen keine festen Zahlen mehr geprueft werden --
niemand weiss, wie viele Achsen darin stehen. Geprueft werden deshalb
Eigenschaften, die fuer jedes Modell gelten muessen: dass genau die
beweglichen Gelenke Grenzen bekommen, dass die Verzoegerung negativ ist,
dass die SRDF gueltiges XML ergibt, dass URDF und SRDF die Parameterdatei
unversehrt durchlaufen. Beim eingebauten Modell kommen die festen Zahlen
zusaetzlich dazu.

--dump legt Parameterdatei, SRDF und URDF in ein Verzeichnis, damit man
nachsehen kann, was move_group tatsaechlich zu lesen bekaeme.
"""
import argparse
import os
import sys
import types
import xml.etree.ElementTree as ET

# tkinter wird beim Import des Moduls gebraucht, aber von nichts benutzt,
# was hier geprueft wird. Fehlt es, treten Attrappen an seine Stelle.
try:
    import tkinter                                     # noqa: F401
except ImportError:
    for name in ("tkinter", "tkinter.font", "tkinter.ttk",
                 "tkinter.filedialog", "tkinter.messagebox",
                 "tkinter.colorchooser"):
        module = types.ModuleType(name)
        module.__getattr__ = lambda attr: type(attr, (), {})
        sys.modules[name] = module
    sys.modules["tkinter"].Tk = type("Tk", (), {})
    for short in ("font", "ttk", "filedialog", "messagebox", "colorchooser"):
        setattr(sys.modules["tkinter"], short, sys.modules["tkinter." + short])

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import urdf_gui2_4 as gui                              # noqa: E402


# --------------------------------------------------------------------------
# Beispielmodell
# --------------------------------------------------------------------------

BEISPIEL = """<?xml version="1.0"?>
<robot name="bm">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="tool"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="2.5"/>
  </joint>
  <joint name="j2" type="prismatic">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="0.4" effort="80" velocity="0.3"/>
  </joint>
  <joint name="j_fix" type="fixed">
    <parent link="link2"/><child link="tool"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>
</robot>
"""


# --------------------------------------------------------------------------
# Protokoll
# --------------------------------------------------------------------------

class Report:
    def __init__(self):
        self.failed = []
        self.skipped = []
        self.warnings = []

    def part(self, title):
        print(f"\n{title}")

    def check(self, label, condition, detail=""):
        if condition:
            print(f"  ok    {label}")
        else:
            print(f"  FEHL  {label}" + (f"   {detail}" if detail else ""))
            self.failed.append(label)

    def skip(self, label, why):
        print(f"  --    {label}   ({why})")
        self.skipped.append(label)

    def warn(self, text):
        print(f"  !     {text}")
        self.warnings.append(text)

    def summary(self):
        print()
        if self.warnings:
            print(f"{len(self.warnings)} Hinweis(e)")
        if self.skipped:
            print(f"{len(self.skipped)} uebersprungen")
        if self.failed:
            print(f"{len(self.failed)} Pruefung(en) fehlgeschlagen:")
            for label in self.failed:
                print(f"    {label}")
            return 1
        print("alle Pruefungen bestanden")
        return 0


# --------------------------------------------------------------------------
# Modellbericht
# --------------------------------------------------------------------------

def describe(rep, name, links, joints, xml_text):
    """Erst zeigen, was im Modell steht. Danach erst pruefen.

    Bei einer fremden URDF ist der Bericht oft nuetzlicher als das Ergebnis:
    er sagt, welchen Tip-Link die SRDF nehmen wuerde und welche Achsen ohne
    Geschwindigkeitsangabe dastehen.
    """
    movable = [j for j in joints
               if j["type"] != "fixed" and j["mimic"] is None]
    mimics = [j for j in joints if j["mimic"] is not None]
    roots = gui.root_links(links, joints)
    leaves = gui.leaf_links(links, joints)

    rep.part("Modell")
    print(f"  Name            {name or '(ohne)'}")
    print(f"  Links           {len(links)}")
    print(f"  Gelenke         {len(joints)}  "
          f"({len(movable)} beweglich, {len(mimics)} gekoppelt)")
    print(f"  Wurzel          {', '.join(roots) or '(keine)'}")
    print(f"  Blaetter        {', '.join(leaves) or '(keine)'}")
    for joint in movable:
        limit = joint.get("velocity")
        print(f"    {joint['name']:<20} {joint['type']:<11} "
              f"v={limit if limit else '(fehlt)'}")

    if len(roots) != 1:
        rep.warn(f"{len(roots)} Wurzel-Links. MoveIt braucht genau einen "
                 "zusammenhaengenden Baum.")
    if len(leaves) > 1:
        rep.warn(f"{len(leaves)} Blaetter. Die SRDF nimmt automatisch "
                 f"'{leaves[0]}' als Tip-Link -- pruefen, ob das der TCP ist, "
                 "sonst im Parameterfenster setzen.")
    if not movable:
        rep.warn("Kein bewegliches Gelenk. Es gibt nichts zu planen.")
    ohne = [j["name"] for j in movable if not j.get("velocity")]
    if ohne:
        rep.warn(f"Ohne <limit velocity>: {', '.join(ohne)}. "
                 f"Startwert {gui.DEFAULT_JOINT_VELOCITY} wird eingesetzt.")
    if mimics:
        rep.warn(f"Gekoppelte Gelenke: {', '.join(j['name'] for j in mimics)}. "
                 "Sie bekommen keine eigenen Grenzen, gehen aber in "
                 "/joint_states mit heraus.")
    if "package://" in xml_text:
        rep.warn("Das Modell enthaelt package://-Verweise. move_group loest "
                 "die nur auf, wenn das Paket im Suchpfad liegt.")
    return movable, roots, leaves


# --------------------------------------------------------------------------
# Pruefungen
# --------------------------------------------------------------------------

def check_config(rep, joints, movable):
    rep.part("MoveItConfig")
    config = gui.MoveItConfig()
    names = config.sync_joints(joints)

    erwartet = [j["name"] for j in movable]
    rep.check("genau die beweglichen Gelenke", names == erwartet,
              f"{names} statt {erwartet}")
    rep.check("feste Gelenke nicht dabei",
              all(j["name"] not in config.limits
                  for j in joints if j["type"] == "fixed"))
    rep.check("gekoppelte Gelenke nicht dabei",
              all(j["name"] not in config.limits
                  for j in joints if j["mimic"] is not None))

    if not names:
        rep.skip("Grenzwerte plausibel", "kein bewegliches Gelenk")
        return config

    schlecht = []
    for name, values in config.limits.items():
        if not values["velocity"] > 0:
            schlecht.append(f"{name}.velocity={values['velocity']}")
        if not values["acceleration"] > 0:
            schlecht.append(f"{name}.acceleration={values['acceleration']}")
        if not values["deceleration"] < 0:
            schlecht.append(f"{name}.deceleration={values['deceleration']}")
        if not values["jerk"] > 0:
            schlecht.append(f"{name}.jerk={values['jerk']}")
    rep.check("Grenzwerte plausibel", not schlecht, "; ".join(schlecht))

    aus_urdf = []
    for joint in movable:
        try:
            wert = float(joint.get("velocity"))
        except (TypeError, ValueError):
            continue
        if wert > 0 and config.limits[joint["name"]]["velocity"] != wert:
            aus_urdf.append(joint["name"])
    rep.check("v aus der URDF uebernommen", not aus_urdf, ", ".join(aus_urdf))

    signature = config.startup_signature()
    config.plan["planning_time"] = 9.0
    rep.check("Auftragswerte aendern die Signatur nicht",
              config.startup_signature() == signature)
    config.cartesian["max_trans_vel"] = 0.4
    rep.check("Bahngrenzen aendern die Signatur",
              config.startup_signature() != signature)
    config.cartesian["max_trans_vel"] = gui.CARTESIAN_FIELDS[0][3]

    fehlt = names[0]
    config.sync_joints([j for j in joints if j["name"] != fehlt])
    rep.check("entferntes Gelenk faellt heraus", fehlt not in config.limits)
    config.sync_joints(joints)
    rep.check("wieder da mit Startwert", fehlt in config.limits)

    config.limits[names[0]]["jerk"] = None
    block = config.limits_yaml(0)
    rep.check("leere Grenze wird abgeschaltet",
              "has_jerk_limits: false" in block)
    config.limits[names[0]] = gui.MoveItConfig.defaults_for(
        next(j for j in movable if j["name"] == names[0]))
    return config


def check_srdf(rep, name, links, joints, leaves, roots):
    rep.part("SRDF")
    srdf = gui.build_srdf(name, links, joints)
    try:
        tree = ET.fromstring(srdf)
    except ET.ParseError as exc:
        rep.check("gueltiges XML", False, str(exc))
        return srdf
    rep.check("gueltiges XML", True)

    group = tree.find(f"group[@name='{gui.MOVEIT_GROUP}']")
    rep.check("Planungsgruppe vorhanden", group is not None)
    if group is not None:
        chain = group.find("chain")
        rep.check("Kette vorhanden", chain is not None)
        if chain is not None:
            bekannt = {link["name"] for link in links}
            base, tip = chain.get("base_link"), chain.get("tip_link")
            rep.check("base_link existiert", base in bekannt, str(base))
            rep.check("tip_link existiert", tip in bekannt, str(tip))
            if roots:
                rep.check("base_link ist die Wurzel", base == roots[0],
                          f"{base} statt {roots[0]}")
            if leaves:
                rep.check("tip_link ist ein Blatt", tip in leaves, str(tip))

    verbunden = sum(1 for j in joints if j["parent"] and j["child"])
    gezaehlt = len(tree.findall("disable_collisions"))
    rep.check("ein abgeschaltetes Paar je Gelenk", gezaehlt == verbunden,
              f"{gezaehlt} statt {verbunden}")

    if len(leaves) > 1:
        anderes = leaves[1]
        srdf2 = gui.build_srdf(name, links, joints, tip=anderes)
        rep.check("Tip-Link laesst sich setzen",
                  f'tip_link="{anderes}"' in srdf2)
    else:
        rep.skip("Tip-Link laesst sich setzen", "nur ein Blatt")
    return srdf


def check_params(rep, config, xml_text, srdf, root, movable):
    rep.part("Parameterdatei")
    params = config.params_yaml(xml_text, srdf)
    rep.check("URDF als Literalblock", "robot_description: |-" in params)
    rep.check("SRDF als Literalblock",
              "robot_description_semantic: |-" in params)
    rep.check("Solver eingetragen",
              f"kinematics_solver: {config.solver}" in params)
    rep.check("Pilz als Pipeline",
              "pilz_industrial_motion_planner/CommandPlanner" in params)
    rep.check("Ruckig angehaengt", "AddRuckigTrajectorySmoothing" in params)
    rep.check("Ausfuehrung aus", "allow_trajectory_execution: false" in params)

    config.use_ruckig = False
    rep.check("Ruckig abschaltbar",
              "AddRuckigTrajectorySmoothing" not in
              config.params_yaml(xml_text, srdf))
    config.use_ruckig = True

    try:
        import yaml
    except ImportError:
        rep.skip("YAML lesbar", "PyYAML fehlt")
        return params

    try:
        data = yaml.safe_load(params)
    except yaml.YAMLError as exc:
        rep.check("YAML lesbar", False, str(exc))
        return params
    rep.check("YAML lesbar", True)

    section = data["/**"]["ros__parameters"]

    # Der eigentliche Punkt: die mehrzeilige XML muss den Umweg ueber den
    # Literalblock unbeschadet ueberstehen. Verglichen wird nicht Text mit
    # Text, sondern Baum mit Baum -- Einrueckung darf sich aendern, Inhalt
    # nicht.
    try:
        zurueck = ET.fromstring(section["robot_description"])
        rep.check("URDF unversehrt durch die Parameterdatei",
                  gui.serialize(zurueck) == gui.serialize(root))
    except ET.ParseError as exc:
        rep.check("URDF unversehrt durch die Parameterdatei", False, str(exc))

    try:
        ET.fromstring(section["robot_description_semantic"])
        rep.check("SRDF unversehrt durch die Parameterdatei", True)
    except ET.ParseError as exc:
        rep.check("SRDF unversehrt durch die Parameterdatei", False, str(exc))

    limits = section["robot_description_planning"]["joint_limits"]
    rep.check("alle beweglichen Gelenke in den Grenzen",
              set(limits) == {j["name"] for j in movable},
              str(set(limits)))

    falsch = [name for name, values in limits.items()
              if values.get("has_deceleration_limits")
              and values.get("max_deceleration", 0) >= 0]
    rep.check("Achsverzoegerungen negativ", not falsch, ", ".join(falsch))

    cart = section["robot_description_planning"]["cartesian_limits"]
    rep.check("alle vier Bahngrenzen da",
              set(cart) == {key for key, _l, _u, _d in gui.CARTESIAN_FIELDS},
              str(set(cart)))
    rep.check("Bahnverzoegerung negativ", cart["max_trans_dec"] < 0,
              cart["max_trans_dec"])

    kin = section["robot_description_kinematics"][config.group]
    rep.check("Kinematik unter der Gruppe", "kinematics_solver" in kin)
    return params


def check_beispiel(rep, config, name, links):
    """Feste Zahlen, die nur fuer das eingebaute Modell gelten."""
    rep.part("Eingebautes Modell, feste Werte")
    rep.check("Name", name == "bm", name)
    rep.check("vier Links", len(links) == 4, len(links))
    rep.check("v von j1", config.limits["j1"]["velocity"] == 2.5,
              config.limits["j1"]["velocity"])
    rep.check("a von j1 abgeleitet", config.limits["j1"]["acceleration"] == 5.0,
              config.limits["j1"]["acceleration"])
    rep.check("dec von j1", config.limits["j1"]["deceleration"] == -5.0,
              config.limits["j1"]["deceleration"])
    rep.check("prismatic uebernommen", config.limits["j2"]["velocity"] == 0.3,
              config.limits["j2"]["velocity"])


def check_process(rep):
    rep.part("MoveItProcess ohne ROS")
    proc = gui.MoveItProcess()
    rep.check("Zustand aus", proc.state() == "aus", proc.state())
    rep.check("nicht am Leben", proc.alive is False)
    proc.stop()
    rep.check("Stoppen ohne Start bleibt still", proc.state() == "aus")


def bahn(namen, dauer=1.0, punkte=11):
    """Kuenstliche Bahn: alle Achsen von 0 nach 1, Geschwindigkeit sinusfoermig.

    Nicht physikalisch gemeint -- es geht um Stetigkeit und Reproduzierbarkeit
    des Abtasters, nicht um ein Profil.
    """
    import math
    out = []
    for i in range(punkte):
        s = i / (punkte - 1)
        t = s * dauer
        pos = 0.5 - 0.5 * math.cos(math.pi * s)
        vel = 0.5 * math.pi * math.sin(math.pi * s) / dauer
        out.append((t, {n: pos for n in namen}, {n: vel for n in namen}))
    return out


def check_player(rep):
    rep.part("TrajectoryPlayer")
    namen = ["a", "b"]
    punkte = bahn(namen, dauer=1.0)

    try:
        gui.TrajectoryPlayer([punkte[0]])
        rep.check("ein Punkt wird abgewiesen", False)
    except ValueError:
        rep.check("ein Punkt wird abgewiesen", True)

    player = gui.TrajectoryPlayer(punkte)
    rep.check("Dauer aus dem letzten Punkt", player.duration == 1.0,
              player.duration)
    rep.check("Start bei null", player.sample(0.0)["a"] == 0.0)
    rep.check("Ende bei eins", abs(player.sample(1.0)["a"] - 1.0) < 1e-12,
              player.sample(1.0)["a"])
    rep.check("vor dem Start wird gehalten", player.sample(-5.0)["a"] == 0.0)
    rep.check("nach dem Ende wird gehalten",
              abs(player.sample(99.0)["a"] - 1.0) < 1e-12)

    # Stetigkeit: kein Sprung zwischen zwei Takten. Bei 100 Takten auf eine
    # Sekunde darf ein Schritt nicht groesser sein als das Doppelte des
    # groessten erwarteten -- das faengt genau die Stuetzpunktknicke.
    letzte = player.sample(0.0)["a"]
    groesster = 0.0
    for i in range(1, 201):
        wert = player.sample(i / 200.0)["a"]
        groesster = max(groesster, abs(wert - letzte))
        letzte = wert
    rep.check("stetig ueber die Stuetzpunkte", groesster < 0.02, groesster)

    # Monotonie: die kuenstliche Bahn steigt durchgehend. Ein Knick oder ein
    # falscher Suchzeiger wuerde hier auffallen.
    rueckwaerts = 0
    letzte = -1.0
    for i in range(0, 401):
        wert = player.sample(i / 400.0)["a"]
        if wert < letzte - 1e-12:
            rueckwaerts += 1
        letzte = wert
    rep.check("keine Rueckwaertsbewegung", rueckwaerts == 0, rueckwaerts)

    rep.part("Takt und Reproduzierbarkeit")
    player = gui.TrajectoryPlayer(punkte)
    lauf1 = []
    while not player.done:
        lauf1.append(player.step()["a"])
    rep.check("Ziel erreicht", player.done is True)
    rep.check("Bahnzeit gleich Dauer", abs(player.t - 1.0) < 1e-12, player.t)
    rep.check("Takte passen zur Zeit",
              abs(player.ticks * player.dt - player.duration) < player.dt,
              f"{player.ticks} Takte")
    rep.check("Endwert erreicht", abs(lauf1[-1] - 1.0) < 1e-12, lauf1[-1])

    player = gui.TrajectoryPlayer(punkte)
    lauf2 = []
    while not player.done:
        lauf2.append(player.step()["a"])
    rep.check("zweiter Lauf bitgleich", lauf1 == lauf2,
              f"{len(lauf1)} gegen {len(lauf2)} Werte")

    rep.part("Bremsen")
    player = gui.TrajectoryPlayer(bahn(namen, dauer=4.0, punkte=41))
    for _ in range(25):                                # etwa eine halbe Sekunde
        player.step()
    vor_dem_bremsen = player.sample()["a"]
    player.brake()
    rep.check("Zustand bremsend", player.braking is True)

    schritte = 0
    letzter = vor_dem_bremsen
    groesster = 0.0
    while not player.stopped and schritte < 1000:
        wert = player.step()["a"]
        groesster = max(groesster, abs(wert - letzter))
        letzter = wert
        schritte += 1
    rep.check("kommt zum Stehen", player.stopped is True, schritte)
    erwartet = gui.BRAKE_TIME_S / player.dt
    rep.check("Rampe dauert BRAKE_TIME_S",
              abs(schritte - erwartet) <= 1, f"{schritte} statt {erwartet:.0f}")
    rep.check("Rate auf null", player.rate == 0.0, player.rate)
    rep.check("kein Sprung beim Bremsen", groesster < 0.02, groesster)
    rep.check("Bahn nicht verlassen", letzter >= vor_dem_bremsen - 1e-12)

    naechster = player.step()["a"]
    rep.check("steht danach still", abs(naechster - letzter) < 1e-15)

    rep.part("Hermite")
    # Mit Randgeschwindigkeit null liegt die Mitte genau dazwischen.
    mitte = gui.hermite(0.0, 0.0, 10.0, 0.0, 1.0, 0.5)
    rep.check("Mitte ohne Geschwindigkeit", abs(mitte - 5.0) < 1e-12, mitte)
    rep.check("Rand links", gui.hermite(2.0, 3.0, 7.0, 1.0, 1.0, 0.0) == 2.0)
    rep.check("Rand rechts", gui.hermite(2.0, 3.0, 7.0, 1.0, 1.0, 1.0) == 7.0)


def check_errors(rep):
    rep.part("Fehlertexte")
    rep.check("Erfolg", "Erfolg" in gui.moveit_error_text(1))
    rep.check("Gruppenname", "Gruppe" in gui.moveit_error_text(-15),
              gui.moveit_error_text(-15))
    rep.check("Zustand veraltet nennt joint_states",
              "joint_states" in gui.moveit_error_text(-23),
              gui.moveit_error_text(-23))
    rep.check("unbekannter Code bleibt sichtbar",
              "-9999" in gui.moveit_error_text(-9999),
              gui.moveit_error_text(-9999))
    fehlt = [code for code in (-1, -2, -7, -10, -14, -16, -17, -31)
             if code not in gui.MOVEIT_ERRORS]
    rep.check("die haeufigen Codes sind hinterlegt", not fehlt, str(fehlt))


# --------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="MoveIt-Seite von urdf_gui2_4 gegen eine URDF pruefen")
    parser.add_argument("urdf", nargs="?",
                        help="Pfad zur URDF. Ohne Angabe das Beispielmodell.")
    parser.add_argument("--dump", metavar="VERZEICHNIS",
                        help="Parameterdatei, SRDF und URDF dort ablegen")
    args = parser.parse_args()

    if args.urdf:
        if not os.path.isfile(args.urdf):
            print(f"Datei nicht gefunden: {args.urdf}")
            return 2
        try:
            root, name, links, joints, xml_text = gui.parse(args.urdf)
        except Exception as exc:
            print(f"URDF nicht lesbar: {exc}")
            return 2
        print(f"Geprueft wird {os.path.abspath(args.urdf)}")
        eigenes = True
    else:
        root = ET.fromstring(BEISPIEL)
        name, links, joints = gui.parse_root(root)
        xml_text = gui.serialize(root)
        print("Geprueft wird das eingebaute Beispielmodell "
              "(Pfad als Argument uebergeben fuer ein eigenes)")
        eigenes = False

    rep = Report()

    # Was die GUI beim Laden auch meldet. Es soll das Ergebnis nicht
    # verwerfen, aber sichtbar sein.
    for problem in gui.validate_root(root):
        rep.warn(f"URDF-Pruefung: {problem}")

    movable, roots, leaves = describe(rep, name, links, joints, xml_text)

    # Die Dateien entstehen VOR den Pruefungen. Sie sind der Grund, warum man
    # das Skript aufruft, wenn etwas nicht laeuft -- und genau dann ist die
    # Wahrscheinlichkeit am groessten, dass eine Pruefung darunter
    # ausgerechnet abbricht. Erst schreiben, dann pruefen.
    if args.dump:
        try:
            basis = gui.MoveItConfig()
            basis.sync_joints(joints)
            dump_srdf = gui.build_srdf(name, links, joints)
            dump_params = basis.params_yaml(xml_text, dump_srdf)
            ziel = os.path.abspath(args.dump)
            os.makedirs(ziel, exist_ok=True)
            for datei, inhalt in (("move_group.yaml", dump_params),
                                  ("robot.srdf", dump_srdf),
                                  ("robot.urdf", xml_text)):
                with open(os.path.join(ziel, datei), "w",
                          encoding="utf-8") as handle:
                    handle.write(inhalt)
            print(f"\nAbgelegt in {ziel}:")
            for datei in ("move_group.yaml", "robot.srdf", "robot.urdf"):
                pfad = os.path.join(ziel, datei)
                print(f"  {os.path.getsize(pfad):>8} Byte  {pfad}")
            print("\n  Start damit:")
            print("  ros2 run moveit_ros_move_group move_group "
                  f"--ros-args --params-file {ziel}/move_group.yaml")
        except OSError as exc:
            print(f"\nDump fehlgeschlagen: {exc}")
            return 2

    # Eine Ausnahme in einer Pruefung soll das Skript nicht mit einem nackten
    # Rueckverfolgungsprotokoll verlassen -- sie ist selbst ein Befund.
    try:
        config = check_config(rep, joints, movable)
        srdf = check_srdf(rep, name, links, joints, leaves, roots)
        check_params(rep, config, xml_text, srdf, root, movable)
        if not eigenes:
            check_beispiel(rep, config, name, links)
        check_process(rep)
        check_player(rep)
        check_errors(rep)
    except Exception:
        import traceback
        print()
        traceback.print_exc()
        rep.check("Pruefung ohne Absturz durchgelaufen", False,
                  "siehe Rueckverfolgung oben")

    return rep.summary()


if __name__ == "__main__":
    sys.exit(main())
