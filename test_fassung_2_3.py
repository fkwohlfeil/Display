#!/usr/bin/env python3
"""Tests fuer die Aufraeumarbeiten in urdf_gui 2.3. Ohne ROS, ohne Tk.

    python3 test_fassung_2_3.py

Geprueft wird:

    - sequenz.Folge.nutzlast lehnt eine Lage ohne Achswerte ab,
    - RosBridge._send_sequence nimmt nur Achsziele (mit Attrappen statt
      moveit_msgs),
    - PLAN_FIELDS ohne die Felder, die nichts bewirkten,
    - einstellungen.py ohne absolute Pfade, mit Umgebungsvariablen,
    - BAHN_VIEWER zeigt auf eine vorhandene Datei,
    - MODUL_STAENDE in urdf_gui.py passt zu den Modulen,
    - Karte BEWEGUNG: Felder aus CARTESIAN_FIELDS, Abgleich beim Start.

urdf_gui.py wird dabei nicht importiert, sondern als Quelltext gelesen --
wie in test_gui_intrinsic.py.
"""

import ast
import importlib
import os
import sys
import types
import unittest
from unittest import mock

import einstellungen
import moveit_konfig
import ros_bruecke
import sequenz

HIER = os.path.dirname(os.path.abspath(__file__))
GUI = os.path.join(HIER, "urdf_gui.py")


def gui_baum():
    with open(GUI, encoding="utf-8") as datei:
        return ast.parse(datei.read())


# ----------------------------------------------------------------------------
# Punktfolge: nur Achsziele
# ----------------------------------------------------------------------------

LISTE = """
0.4 0.0 0.5   0 90 0   fahrart=LIN radius=0.05 name=A
0.4 0.2 0.5   0 90 0
0.4 0.2 0.3   0 90 0   radius=0 name=C
"""


def folge(mit_achsen=True):
    punkte, fehler = sequenz.lies_liste(LISTE)
    assert not fehler, fehler
    if mit_achsen:
        for nummer, punkt in enumerate(punkte):
            punkt.achsen = {"j1": 0.1 * nummer, "j2": -0.2 * nummer}
    return sequenz.Folge(punkte)


class Nutzlast(unittest.TestCase):

    def test_lage_ohne_achswerte_ist_fehler(self):
        with self.assertRaisesRegex(ValueError, "A: Lage ohne Achswerte"):
            folge(mit_achsen=False).nutzlast("arm", {"j1": 0.0}, 0.01)

    def test_ein_punkt_ohne_achswerte_reicht(self):
        f = folge()
        f.punkte[1].achsen = None
        with self.assertRaisesRegex(ValueError, "P2: Lage ohne Achswerte"):
            f.nutzlast("arm", {"j1": 0.0}, 0.01)

    def test_nur_achsziele_hinaus(self):
        last = folge().nutzlast("arm", {"j1": 0.0, "j2": 0.0}, 0.01, tag=7)
        self.assertEqual(last["tag"], 7)
        for eintrag in last["items"]:
            self.assertIn("goal", eintrag)
            self.assertNotIn("pose", eintrag)
        self.assertEqual(last["items"][2]["goal"], {"j1": 0.2, "j2": -0.4})
        # Startzustand nur am ersten Posten.
        self.assertIn("start", last["items"][0])
        self.assertTrue(all("start" not in e for e in last["items"][1:]))

    def test_kein_tcp_parameter_mehr(self):
        with self.assertRaises(TypeError):
            folge().nutzlast("arm", {}, 0.01, tcp="tool0")


# ----------------------------------------------------------------------------
# RosBridge._send_sequence mit Attrappen
# ----------------------------------------------------------------------------

class _JointState:
    def __init__(self):
        self.name, self.position = [], []


class _RobotState:
    def __init__(self):
        self.joint_state = _JointState()
        self.is_diff = True


class _PlanRequest:
    def __init__(self):
        self.start_state = _RobotState()
        self.goal_constraints = []


class _Item:
    def __init__(self):
        self.blend_radius = 0.0
        self.req = _PlanRequest()


class _Constraints:
    def __init__(self):
        self.joint_constraints = []


class _JointConstraint:
    pass


class _GetMotionSequence:
    class Request:
        def __init__(self):
            self.request = types.SimpleNamespace(items=[])


def moveit_msgs_attrappe():
    srv = types.ModuleType("moveit_msgs.srv")
    srv.GetMotionSequence = _GetMotionSequence
    msg = types.ModuleType("moveit_msgs.msg")
    msg.Constraints = _Constraints
    msg.JointConstraint = _JointConstraint
    msg.MotionSequenceItem = _Item
    paket = types.ModuleType("moveit_msgs")
    paket.srv, paket.msg = srv, msg
    return {"moveit_msgs": paket, "moveit_msgs.srv": srv,
            "moveit_msgs.msg": msg}


class _Future:
    def __init__(self, anfrage):
        self.anfrage = anfrage
        self.rueckrufe = []

    def add_done_callback(self, rueckruf):
        self.rueckrufe.append(rueckruf)


class _Client:
    def __init__(self):
        self.futures = []

    def service_is_ready(self):
        return True

    def call_async(self, anfrage):
        self.futures.append(_Future(anfrage))
        return self.futures[-1]


class _Node:
    def __init__(self):
        self.client = _Client()

    def create_client(self, _typ, _name):
        return self.client


class Bruecke(unittest.TestCase):

    def setUp(self):
        self.bruecke = ros_bruecke.RosBridge()
        self.bruecke.node = _Node()
        attrappe = mock.patch.dict(sys.modules, moveit_msgs_attrappe())
        attrappe.start()
        self.addCleanup(attrappe.stop)

    def test_stand(self):
        self.assertEqual(ros_bruecke.STAND, 2)

    def test_posten_ohne_achsziel_ist_fehler(self):
        last = folge().nutzlast("arm", {"j1": 0.0, "j2": 0.0}, 0.01, tag=3)
        del last["items"][1]["goal"]
        last["items"][1]["pose"] = {"xyz": (0, 0, 0)}   # das alte Format
        self.bruecke._send_sequence(last)
        art, wert = self.bruecke.results.popleft()
        self.assertEqual(art, "sequence")
        self.assertEqual(wert["tag"], 3)
        self.assertIn("ohne Achsziel: P2", wert["error"])
        # Gestellt wurde nichts.
        self.assertEqual(self.bruecke.node.client.futures, [])
        self.assertIsNone(self.bruecke._pending)

    def test_achsziele_werden_jointconstraints(self):
        last = folge().nutzlast("arm", {"j1": 0.0, "j2": 0.0}, 0.01, tag=4)
        self.bruecke._send_sequence(last)
        self.assertEqual(len(self.bruecke.results), 0)
        anfrage = self.bruecke.node.client.futures[0].anfrage
        posten = anfrage.request.items
        self.assertEqual(len(posten), 3)
        bedingung = posten[2].req.goal_constraints[0].joint_constraints
        self.assertEqual([(c.joint_name, c.position) for c in bedingung],
                         [("j1", 0.2), ("j2", -0.4)])
        self.assertEqual(bedingung[0].tolerance_above, 0.01)
        self.assertEqual(posten[0].req.start_state.joint_state.name,
                         ["j1", "j2"])
        self.assertEqual(posten[1].req.start_state.joint_state.name, [])
        self.assertEqual(self.bruecke._pending[0], "sequence")
        self.assertEqual(self.bruecke._pending[4], 4)


# ----------------------------------------------------------------------------
# Reiter Fahrt
# ----------------------------------------------------------------------------

class PlanFelder(unittest.TestCase):

    def test_stand(self):
        self.assertEqual(moveit_konfig.STAND, 2)

    def test_felder(self):
        schluessel = [f[0] for f in moveit_konfig.PLAN_FIELDS]
        self.assertEqual(schluessel, [
            "velocity_scaling", "acceleration_scaling", "planning_time",
            "planning_attempts", "goal_orientation_tol", "blend_radius"])
        beschriftung = dict((f[0], f[1]) for f in moveit_konfig.PLAN_FIELDS)
        self.assertEqual(beschriftung["goal_orientation_tol"],
                         "Zieltoleranz Achsen")
        self.assertEqual(sorted(moveit_konfig.MoveItConfig().plan),
                         sorted(schluessel))

    def test_pruefregeln_ohne_alte_felder(self):
        text = open(GUI, encoding="utf-8").read()
        anfang = text.index("def _params_read(")
        rumpf = text[anfang:anfang + 1500]
        for alt in ("goal_position_tol", "max_step", "jump_threshold"):
            self.assertNotIn(alt, rumpf)


# ----------------------------------------------------------------------------
# einstellungen
# ----------------------------------------------------------------------------

class Einstellungen(unittest.TestCase):

    VARIABLEN = ("URDF_GUI_URDF", "URDF_GUI_SICHERUNG", "URDF_GUI_BAHNEN")

    def neu_laden(self, umgebung):
        sauber = {k: v for k, v in os.environ.items()
                  if k not in self.VARIABLEN}
        sauber.update(umgebung)
        with mock.patch.dict(os.environ, sauber, clear=True):
            return importlib.reload(einstellungen)

    def tearDown(self):
        self.neu_laden({})

    def test_stand(self):
        self.assertEqual(einstellungen.STAND, 2)

    def test_standard_neben_dem_modul(self):
        e = self.neu_laden({})
        self.assertEqual(e.HIER, HIER)
        self.assertEqual(e.URDF_PATH, os.path.join(HIER, "bmKinematic.urdf"))
        self.assertEqual(e.BACKUP_DIR, os.path.join(HIER, "URDF_Speicher"))
        self.assertEqual(e.BAHN_DIR, os.path.join(HIER, "TCP_Fahrten"))

    def test_umgebung_geht_vor(self):
        e = self.neu_laden({"URDF_GUI_URDF": "/x/robot.urdf",
                            "URDF_GUI_SICHERUNG": "/x/sicherung",
                            "URDF_GUI_BAHNEN": "~/bahnen"})
        self.assertEqual(e.URDF_PATH, "/x/robot.urdf")
        self.assertEqual(e.BACKUP_DIR, "/x/sicherung")
        self.assertEqual(e.BAHN_DIR, os.path.expanduser("~/bahnen"))

    def test_leere_variable_zaehlt_nicht(self):
        e = self.neu_laden({"URDF_GUI_BAHNEN": "  "})
        self.assertEqual(e.BAHN_DIR, os.path.join(HIER, "TCP_Fahrten"))

    def test_kein_absoluter_pfad_im_quelltext(self):
        text = open(os.path.join(HIER, "einstellungen.py"),
                    encoding="utf-8").read()
        self.assertNotIn("/home/", text)

    def test_bahn_viewer_vorhanden(self):
        self.assertTrue(os.path.isfile(
            os.path.join(HIER, einstellungen.BAHN_VIEWER)))

    def test_bahn_viewer_nicht_mehr_in_totg_prozess(self):
        import totg_prozess
        self.assertEqual(totg_prozess.STAND, 2)
        self.assertFalse(hasattr(totg_prozess, "BAHN_VIEWER"))


# ----------------------------------------------------------------------------
# urdf_gui.py als Quelltext
# ----------------------------------------------------------------------------

class Oberflaeche(unittest.TestCase):

    def test_modul_staende_passen(self):
        for knoten in gui_baum().body:
            if (isinstance(knoten, ast.Assign)
                    and getattr(knoten.targets[0], "id", "") == "MODUL_STAENDE"):
                staende = {k.id: v.value for k, v
                           in zip(knoten.value.keys, knoten.value.values)}
                break
        else:
            self.fail("MODUL_STAENDE nicht gefunden")
        self.assertEqual(len(staende), 10)
        for name, soll in staende.items():
            modul = importlib.import_module(name)
            self.assertEqual(modul.STAND, soll, name)

    def test_bewegung_ohne_jerk(self):
        for knoten in gui_baum().body:
            if (isinstance(knoten, ast.Assign)
                    and getattr(knoten.targets[0], "id", "") == "MOTION_FIELDS"):
                schluessel = [e.elts[0].value for e in knoten.value.elts]
                break
        else:
            self.fail("MOTION_FIELDS nicht gefunden")
        self.assertEqual(schluessel, ["speed", "acceleration"])

    def test_bewegung_beim_start_abgeglichen(self):
        klasse = next(k for k in gui_baum().body
                      if isinstance(k, ast.ClassDef) and k.name == "UrdfViewer")
        init = next(f for f in klasse.body
                    if isinstance(f, ast.FunctionDef) and f.name == "__init__")
        aufrufe = [n.value.func.attr for n in init.body
                   if isinstance(n, ast.Expr) and isinstance(n.value, ast.Call)
                   and isinstance(n.value.func, ast.Attribute)]
        self.assertIn("_sync_motion_from_config", aufrufe)
        self.assertLess(aufrufe.index("_build_ui"),
                        aufrufe.index("_sync_motion_from_config"))


if __name__ == "__main__":
    unittest.main(verbosity=1)
