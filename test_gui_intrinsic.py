#!/usr/bin/env python3
"""Die Intrinsic-Anbindung in urdf_gui.py, ohne Tk, ohne ROS.

    python3 test_gui_intrinsic.py

Die GUI selbst laesst sich hier nicht starten (Tk, rclpy, move_group).
Geprueft werden deshalb die neuen Methoden, herausgeloest aus der Datei
und an ein Ersatzobjekt gehaengt: Auftrag bauen, Kindprozess starten,
Antwort abholen, Bericht schreiben, Fehlerwege. Der Kindprozess ist echt
(TotgProzess), nur der Rechner ist ein Ersatz, der ohne MoveIt antwortet.
"""

import ast
import json
import os
import stat
import tempfile
import textwrap
import time
import unittest

import intrinsic_folge
import sequenz
import totg_folge
from moveit_konfig import MoveItConfig
from totg_prozess import TotgProzess

HIER = os.path.dirname(os.path.abspath(__file__))
GUI = os.path.join(HIER, "urdf_gui.py")
METHODEN = ("_intrinsic_meldung", "_intrinsic_send", "_intrinsic_poll",
            "_seq_intrinsic_bericht", "_seq_intrinsic_bereit",
            "_seq_totg_felder")

STATE_READY, STATE_ERROR, STATE_SEQUENCE = "bereit", "Fehler", "Folge"
FOLGE_START_TOL = 1e-3

PARAMS = """/**:
  ros__parameters:
    robot_description: |-
      <robot name="r">
        <link name="base"/><link name="l1"/><link name="l2"/><link name="tcp"/>
        <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" velocity="2" effort="1"/></joint>
        <joint name="j2" type="continuous"><parent link="l1"/><child link="l2"/><axis xyz="0 1 0"/></joint>
        <joint name="j3" type="fixed"><parent link="l2"/><child link="tcp"/></joint>
      </robot>
    robot_description_semantic: |-
      <robot name="r"><group name="arm"><chain base_link="base" tip_link="tcp"/></group></robot>
    robot_description_planning:
      joint_limits:
        j1: {has_velocity_limits: true, max_velocity: 1.5, has_acceleration_limits: true, max_acceleration: 4.0}
        j2: {has_velocity_limits: true, max_velocity: 2.5, has_acceleration_limits: true, max_acceleration: 5.0}
"""

# Ersatzrechner. Antwortet wie intrinsic_lin_rechner, ohne MoveIt: die Bahn
# laeuft linear im Gelenkraum durch die Wegpunkte. ART in der Umgebung
# waehlt den Fehlerfall.
RECHNER = r'''
import json, os, sys, time
auftrag = json.load(open(sys.argv[1]))
art = os.environ.get("ART", "gut")
print("Ersatzrechner:", art, flush=True)
if art == "haengt":
    time.sleep(30)
if art == "stumm":
    sys.exit(3)
if art == "abgelehnt":
    json.dump({"ok": False, "fehler": "LinFehler: IK ohne Loesung"},
              open(sys.argv[2], "w"))
    sys.exit(1)
w = auftrag["wegpunkte"]
punkte = [[0.1 * i, q, [0.0] * len(q)] for i, q in enumerate(w)]
json.dump({"ok": True, "stand": 2, "intrinsic_release": "20260922.0",
           "gelenke": auftrag["gelenke"], "punkte": punkte,
           "dauer_s": punkte[-1][0], "teilbahnen": [len(punkte)],
           "rechenzeit_s": 0.01, "moveit_version": "ersatz", "start_s": 0.0,
           "ik_aufrufe": 7, "ik_rechenzeit_s": 0.1,
           "abschnitte": [{"toppra_iterationen": 2}],
           "grenzpruefung": {"roh": {"v_faktor_max": 0.9, "a_faktor_max": 0.8,
                                     "eingehalten": True},
                             "abgetastet": {"v_faktor_max": 0.95,
                                            "a_faktor_max": 0.85,
                                            "eingehalten": True}},
           "lin": [{"ik_sprung_max_dicht_rad": 0.02,
                    "ziel_abweichung_rad": 1e-10, "pfadpunkte": 5}],
           "hinweise": ["Start -> P2: Hinweis aus dem Wrapper"],
           "lin_vorgaben": {"tcp_v_max": auftrag.get("tcp_v_max"),
                            "tcp_a_max": auftrag.get("tcp_a_max")},
           "tcp_pruefung": ({"v_max_m_s": 0.1249, "v_faktor_max": 0.9992,
                             "v_eingehalten": True, "massgeblich_proben": 40,
                             "proben": 50, "a_max_m_s2": 0.42,
                             "a_faktor_max": 1.05, "a_eingehalten": False,
                             "a_roh_max_m_s2": 0.39,
                             "a_roh_faktor_max": 0.975,
                             "a_roh_eingehalten": True}
                            if "tcp_v_max" in auftrag else None),
           "auftrag_echo": auftrag},
          open(sys.argv[2], "w"))
'''


def methoden_laden():
    """Die neuen Methoden aus urdf_gui.py als Klasse, samt Dekoratoren."""
    quelle = open(GUI, encoding="utf-8").read()
    zeilen = quelle.splitlines()
    baum = ast.parse(quelle)
    gefunden = {}
    for knoten in ast.walk(baum):
        if isinstance(knoten, ast.FunctionDef) and knoten.name in METHODEN:
            anfang = min([knoten.lineno]
                         + [d.lineno for d in knoten.decorator_list])
            gefunden[knoten.name] = "\n".join(
                zeilen[anfang - 1:knoten.end_lineno])
    fehlend = set(METHODEN) - set(gefunden)
    if fehlend:
        raise SystemExit(f"Nicht in urdf_gui.py:{sorted(fehlend)}")
    koerper = "\n\n".join(textwrap.dedent(t) for t in gefunden.values())
    text = "class Methoden:\n" + textwrap.indent(koerper, "    ")
    umgebung = {
        "intrinsic_folge": intrinsic_folge, "totg_folge": totg_folge,
        "sequenz": sequenz, "time": time, "os": os,
        "STATE_READY": STATE_READY, "STATE_ERROR": STATE_ERROR,
        "STATE_SEQUENCE": STATE_SEQUENCE, "FOLGE_START_TOL": FOLGE_START_TOL,
    }
    exec(compile(text, GUI + " (Auszug)", "exec"), umgebung)
    return umgebung["Methoden"], quelle


Methoden, QUELLE = methoden_laden()


class Feld:
    def __init__(self):
        self.state = None

    def winfo_exists(self):
        return True

    def configure(self, state):
        self.state = state


class Var:
    def __init__(self, wert):
        self.wert = wert

    def get(self):
        return self.wert


class Ersatz(Methoden):
    """Genau das, was die neuen Methoden vom Fenster brauchen."""

    def __init__(self, ordner, params):
        self.joints = [{"name": "j1", "type": "revolute", "mimic": None},
                       {"name": "j2", "type": "continuous", "mimic": None},
                       {"name": "j3", "type": "fixed", "mimic": None}]
        self.seq_ik_frames = ("base", "tcp")
        self.seq_verfahren_geplant = intrinsic_folge.VERFAHREN_INTRINSIC
        self.seq_verfahren_var = Var(intrinsic_folge.VERFAHREN_INTRINSIC)
        self.seq_rot_var = Var("0.2")
        self.seq_dt_var = Var("0.004")
        self.seq_ueberschleifen_var = Var("0.03")
        self.seq_abtastung_var = Var("0.005")
        self.seq_tcp_v_var = Var("")
        self.seq_tcp_a_var = Var("")
        self.seq_intrinsic_parameter = None
        self.seq_intrinsic_auftrag = None
        self.intrinsic = TotgProzess(rechner=os.path.join(ordner, "rechner.py"))
        self.params = params
        self.moveit_config = MoveItConfig()
        self.moveit_config.kinematics["kinematics_solver_timeout"] = 0.2
        self.state = STATE_READY
        self.besetzt = False
        self.geschrieben, self.protokoll, self.ergebnisse = [], [], []
        self.seq_started = None

    def _busy(self):
        return self.besetzt

    def _seq_params_datei(self):
        return self.params

    def _seq_write(self, text, level="info", clear=False):
        self.geschrieben.append((level, text))

    def _log(self, text, level="info"):
        self.protokoll.append((level, text))

    def _job_drop(self):
        pass

    def _seq_ik_reset(self):
        self.seq_ik_frames = None

    def _set_state(self, state, detail=""):
        self.state = state

    def _on_sequence_result(self, ergebnis):
        self.ergebnisse.append(ergebnis)

    def warten(self, frist=20.0):
        ende = time.monotonic() + frist
        while not self.ergebnisse and time.monotonic() < ende:
            self._intrinsic_poll()
            time.sleep(0.02)
        return self.ergebnisse[-1] if self.ergebnisse else None


def nutzlast():
    def posten(name, ziel, radius):
        return {"group": "arm", "planner_id": "LIN", "velocity_scaling": 0.5,
                "acceleration_scaling": 0.4, "blend_radius": radius,
                "name": name, "goal": {"j1": ziel[0], "j2": ziel[1]}}
    items = [posten("P1", (0.5, 0.0), 0.02), posten("P2", (0.5, 0.5), 0.0)]
    items[0]["start"] = {"j1": 0.0, "j2": 0.0, "j3": 0.0}
    return {"group": "arm", "items": items, "tag": 17}


class Anbindung(unittest.TestCase):

    def setUp(self):
        self.ordner = tempfile.mkdtemp()
        with open(os.path.join(self.ordner, "rechner.py"), "w") as datei:
            datei.write(RECHNER)
        self.params = os.path.join(self.ordner, "move_group.yaml")
        with open(self.params, "w") as datei:
            datei.write(PARAMS)
        self.wrapper = os.path.join(self.ordner, "wrapper")
        with open(self.wrapper, "w") as datei:
            datei.write("#!/bin/sh\n")
        os.chmod(self.wrapper, stat.S_IRWXU)
        self.alt = dict(os.environ)
        os.environ["INTRINSIC_WRAPPER"] = self.wrapper
        os.environ["ART"] = "gut"
        self.gui = Ersatz(self.ordner, self.params)

    def tearDown(self):
        self.gui.intrinsic.aufraeumen()
        os.environ.clear()
        os.environ.update(self.alt)

    def bereit_und_senden(self, tcp_v=None, tcp_a=None):
        rechner_alt = intrinsic_folge.RECHNER
        intrinsic_folge.RECHNER = self.gui.intrinsic.rechner
        try:
            self.assertTrue(self.gui._seq_intrinsic_bereit(),
                            self.gui.protokoll)
        finally:
            intrinsic_folge.RECHNER = rechner_alt
        self.assertEqual(self.gui.seq_intrinsic_parameter,
                         (0.2, 0.004, 0.03, 0.005, tcp_v, tcp_a))
        self.gui._intrinsic_send(nutzlast())

    def test_gut(self):
        self.bereit_und_senden()
        self.assertEqual(self.gui.state, STATE_SEQUENCE,
                         self.gui.geschrieben)
        auftrag = self.gui.seq_intrinsic_auftrag
        self.assertEqual(auftrag["tip"], "tcp")
        self.assertEqual(auftrag["gelenke"], ["j1", "j2"])
        self.assertEqual(auftrag["wrapper"], self.wrapper)
        self.assertEqual(auftrag["rundung_translation_m"], 0.02)
        # Zeitgrenze aus dem Parameterfenster, nicht die feste des Rechners.
        self.assertEqual(auftrag["ik_timeout_s"], 0.2)
        self.assertEqual(auftrag["rundung_rotation_rad"], 0.2)
        # Die Gelenkraum-Felder aus dem Folgenfenster, nicht die Standards.
        self.assertEqual(auftrag["ueberschleifen_rad"], 0.03)
        self.assertEqual(auftrag["gelenk_abtastung_rad"], 0.005)
        self.assertEqual(auftrag["grenzen"]["max_velocity"], [1.5, 2.5])
        self.assertEqual(auftrag["grenzen"]["max_position"][1], 1e9)
        self.assertTrue(any("continuous" in t for _, t in
                            self.gui.geschrieben))
        ergebnis = self.gui.warten()
        self.assertIsNotNone(ergebnis, self.gui.intrinsic.protokoll())
        self.assertNotIn("error", ergebnis)
        self.assertEqual(ergebnis["tag"], 17)
        self.assertEqual(len(ergebnis["points"]), 3)
        self.assertEqual(ergebnis["intrinsic"]["endabweichung"], 0.0)
        self.assertEqual(ergebnis["intrinsic"]["toppra_iterationen"], [2])
        # Der Rechner bekam genau den Auftrag, den die GUI baute.
        with open(os.path.join(self.gui.intrinsic.ordner,
                               "antwort.json")) as datei:
            antwort = json.load(datei)
        self.assertEqual(antwort["auftrag_echo"], auftrag)

        self.gui.geschrieben = []
        self.gui._seq_intrinsic_bericht(ergebnis, ergebnis["teilbahnen"])
        text = "\n".join(t for _, t in self.gui.geschrieben)
        self.assertIn("Intrinsic 20260922.0 (Wrapper Stand 2)", text)
        self.assertIn("TOPP-RA-Iterationen: 2.", text)
        self.assertIn("7 Aufrufe", text)
        self.assertIn("v bis 0.900, a bis 0.800", text)
        self.assertIn("Intrinsic: Start -> P2: Hinweis aus dem Wrapper", text)
        self.assertNotIn("VERLETZT", text)
        # Ohne v_bahn keine Grenze im Auftrag, und der Bericht sagt es.
        self.assertNotIn("tcp_v_max", auftrag)
        self.assertIn("keine Bahngeschwindigkeit", text)

    def test_bahngrenzen(self):
        self.gui.seq_tcp_v_var = Var("0.25")
        self.gui.seq_tcp_a_var = Var("0.8")
        self.bereit_und_senden(tcp_v=0.25, tcp_a=0.8)
        auftrag = self.gui.seq_intrinsic_auftrag
        self.assertEqual(auftrag["tcp_v_max"], 0.25)
        self.assertEqual(auftrag["tcp_a_max"], 0.8)
        self.assertTrue(any("Bahngeschwindigkeit hoechstens 0.25 m/s, "
                            "Bahnbeschleunigung hoechstens 0.8 m/s^2" in t
                            for _, t in self.gui.geschrieben),
                        self.gui.geschrieben)
        ergebnis = self.gui.warten()
        self.assertNotIn("error", ergebnis)
        intr = ergebnis["intrinsic"]
        self.assertEqual(intr["tcp_v_max"], 0.25)
        self.assertEqual(intr["tcp_v_gemessen_max"], 0.1249)
        self.gui.geschrieben = []
        self.gui._seq_intrinsic_bericht(ergebnis, ergebnis["teilbahnen"])
        text = "\n".join(t for _, t in self.gui.geschrieben)
        # Vorgabe mal Override 50 %.
        self.assertIn("Bahngeschwindigkeit am Tip tcp: Vorgabe 0.125 m/s, "
                      "gemessen bis 0.1249 m/s", text)
        self.assertIn("an 40 von 50 Pfadproben", text)
        self.assertNotIn("keine Bahngeschwindigkeit", text)
        self.assertNotIn("keine Bahnbeschleunigung", text)
        # Vorgabe 0.8 mal Override 40 %; roh gehalten, abgetastet darueber.
        self.assertIn("Bahnbeschleunigung am Tip tcp: Vorgabe 0.32 m/s^2, "
                      "TOPP-RA roh bis 0.3900 m/s^2 (Faktor 0.9750, "
                      "eingehalten); abgetastet bis 0.4200 m/s^2 (Faktor "
                      "1.0500, UEBERSCHRITTEN).", text)

    def test_abgelehnt(self):
        os.environ["ART"] = "abgelehnt"
        self.bereit_und_senden()
        ergebnis = self.gui.warten()
        self.assertEqual(ergebnis["error"],
                         "Intrinsic: LinFehler: IK ohne Loesung")

    def test_stumm(self):
        os.environ["ART"] = "stumm"
        self.bereit_und_senden()
        ergebnis = self.gui.warten()
        self.assertIn("intrinsic_lin_rechner endete mit Code 3",
                      ergebnis["error"])
        self.assertIn("Ersatzrechner: stumm",
                      "\n".join(self.gui.intrinsic.protokoll()))

    def test_frist(self):
        os.environ["ART"] = "haengt"
        self.bereit_und_senden()
        self.gui.intrinsic.frist = 0.3
        ergebnis = self.gui.warten()
        self.assertEqual(ergebnis["error"],
                         "intrinsic_lin_rechner antwortet seit 0 s nicht")
        self.assertFalse(self.gui.intrinsic.laeuft)

    def test_niemand_wartet(self):
        os.environ["ART"] = "haengt"
        self.bereit_und_senden()
        self.gui.state = STATE_READY
        self.gui._intrinsic_poll()
        self.assertFalse(self.gui.intrinsic.laeuft)
        self.assertEqual(self.gui.ergebnisse, [])
        self.assertIn("niemand", self.gui.protokoll[-1][1])

    def test_nicht_tragfaehig(self):
        last = nutzlast()
        last["items"][0]["velocity_scaling"] = 0.9
        self.gui.seq_intrinsic_parameter = (0.2, 0.004, 0.2, 0.05, None, None)
        self.gui._intrinsic_send(last)
        self.assertEqual(self.gui.state, STATE_ERROR)
        self.assertFalse(self.gui.intrinsic.laeuft)
        self.assertIsNone(self.gui.seq_ik_frames)
        self.assertIn("nicht tragfaehig", self.gui.protokoll[-1][1])

    def test_bereit_prueft(self):
        self.gui.seq_rot_var = Var("0")
        self.assertFalse(self.gui._seq_intrinsic_bereit())
        self.gui.seq_rot_var = Var("0.1")
        # Der Wrapper verlangt beide Gelenkraum-Werte > 0.
        for var in ("seq_ueberschleifen_var", "seq_abtastung_var"):
            alt = getattr(self.gui, var)
            for falsch in ("0", "-0.1", "abc"):
                setattr(self.gui, var, Var(falsch))
                self.assertFalse(self.gui._seq_intrinsic_bereit(), var)
            setattr(self.gui, var, alt)
        # v_bahn, a_bahn: leer ist gueltig (keine Grenze), Unsinn nicht.
        for var, zuviel in (("seq_tcp_v_var", "100"),
                            ("seq_tcp_a_var", "1000")):
            for falsch in ("0", "-0.1", "abc", zuviel):
                setattr(self.gui, var, Var(falsch))
                self.assertFalse(self.gui._seq_intrinsic_bereit(),
                                 (var, falsch))
            setattr(self.gui, var, Var(""))
        os.environ["INTRINSIC_WRAPPER"] = self.params      # nicht ausfuehrbar
        self.assertFalse(self.gui._seq_intrinsic_bereit())
        self.assertIn("Wrapper", self.gui.protokoll[-1][1])
        os.environ["INTRINSIC_WRAPPER"] = self.wrapper
        self.gui.params = None
        self.assertFalse(self.gui._seq_intrinsic_bereit())
        self.assertIn("Parameterdatei", self.gui.protokoll[-1][1])

    def test_felder(self):
        felder = {name: Feld() for name in ("tol", "dt", "rot")}
        self.gui.seq_totg_widgets = [
            (felder["tol"], (totg_folge.VERFAHREN_TOTG,)),
            (felder["dt"], (totg_folge.VERFAHREN_TOTG,
                            intrinsic_folge.VERFAHREN_INTRINSIC)),
            (felder["rot"], (intrinsic_folge.VERFAHREN_INTRINSIC,))]
        for verfahren, scharf in (
                ("Intrinsic", {"dt", "rot"}), ("TOTG", {"tol", "dt"}),
                ("Pilz", set())):
            self.gui.seq_verfahren_var = Var(verfahren)
            self.gui._seq_totg_felder()
            self.assertEqual({n for n, f in felder.items()
                              if f.state == "normal"}, scharf, verfahren)
        self.gui.besetzt = True
        self.gui.seq_verfahren_var = Var("Intrinsic")
        self.gui._seq_totg_felder()
        self.assertTrue(all(f.state == "disabled" for f in felder.values()))


class Verdrahtung(unittest.TestCase):
    """Die Stellen ausserhalb der neuen Methoden, als Text geprueft."""

    def test_stellen(self):
        for stelle in (
                "self.intrinsic = TotgProzess(rechner=intrinsic_folge.RECHNER)",
                "        self._intrinsic_poll()\n        self._moveit_watch()",
                "        self.intrinsic.aufraeumen()\n",
                "            self._intrinsic_send(payload)\n",
                "and not self._seq_intrinsic_bereit()",
                "zusatz = intrinsic_folge.zusatzzeilen(herkunft)",
                "elif verfahren == intrinsic_folge.VERFAHREN_INTRINSIC:",
                "self._seq_intrinsic_bericht(result, teilbahnen)",
                "+ [intrinsic_folge.VERFAHREN_INTRINSIC]",
                "self.intrinsic.abbrechen()",
                'sys.exit("Module passen nicht zu urdf_gui.py',
        ):
            self.assertEqual(QUELLE.count(stelle), 1 if stelle !=
                             "self.intrinsic.abbrechen()" else 2, stelle)


if __name__ == "__main__":
    unittest.main(verbosity=1)
