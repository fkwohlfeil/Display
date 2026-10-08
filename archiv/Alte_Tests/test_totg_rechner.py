#!/usr/bin/env python3
"""Pruefung von totg_rechner.py ohne MoveIt.

    python3 test_totg_rechner.py

Geprueft wird, was ohne moveit_py pruefbar ist: das Umschreiben der
Parameterdatei, das Schreiben der Antwort und das Verhalten, wenn die
Rechnung scheitert -- dann muss trotzdem eine lesbare Antwort mit ok=false
dastehen, sonst wartet die Oberflaeche bis zur Frist.
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

import yaml

HIER = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HIER)
import totg_rechner  # noqa: E402

VORLAGE = """/**:
  ros__parameters:
    robot_description: |-
      <robot name="r">
        <link name="a"/>
      </robot>
    robot_description_planning:
      joint_limits:
        j1:
          has_velocity_limits: true
          max_velocity: 1.0
          has_acceleration_limits: true
          max_acceleration: 2.0
          max_deceleration: -2.0
    planning_pipelines: ["pilz_industrial_motion_planner"]
    default_planning_pipeline: pilz_industrial_motion_planner
    pilz_industrial_motion_planner:
      planning_plugins:
        - pilz_industrial_motion_planner/CommandPlanner
      sampling_time: 0.1
    capabilities: "a b"
    allow_trajectory_execution: false
"""


class Params(unittest.TestCase):

    def setUp(self):
        self.alt = yaml.safe_load(VORLAGE)["/**"]["ros__parameters"]
        text = totg_rechner.params_fuer_moveitpy(VORLAGE)
        self.text = text
        self.neu = yaml.safe_load(text)["/**"]["ros__parameters"]

    def test_pipelines_als_map(self):
        self.assertEqual(self.neu["planning_pipelines"],
                         {"pipeline_names": ["pilz_industrial_motion_planner"]})

    def test_rest_unveraendert(self):
        alt = dict(self.alt)
        neu = dict(self.neu)
        for schluessel in ("planning_pipelines", "planning_scene_monitor_options"):
            alt.pop(schluessel, None)
            neu.pop(schluessel, None)
        self.assertEqual(alt, neu)

    def test_gleitkomma_bleibt_gleitkomma(self):
        grenzen = self.neu["robot_description_planning"]["joint_limits"]["j1"]
        self.assertIsInstance(grenzen["max_velocity"], float)
        self.assertIsInstance(grenzen["max_deceleration"], float)
        self.assertIn("max_velocity: 1.0", self.text)

    def test_urdf_als_block(self):
        self.assertIn("robot_description: |", self.text)
        self.assertEqual(self.neu["robot_description"], self.alt["robot_description"])

    def test_eigener_szenenname(self):
        self.assertEqual(self.neu["planning_scene_monitor_options"]["name"],
                         "planning_scene_monitor_totg")

    def test_schon_map_bleibt(self):
        text = VORLAGE.replace('planning_pipelines: ["pilz_industrial_motion_planner"]',
                               'planning_pipelines:\n      pipeline_names: [x]')
        neu = yaml.safe_load(totg_rechner.params_fuer_moveitpy(text))
        self.assertEqual(neu["/**"]["ros__parameters"]["planning_pipelines"],
                         {"pipeline_names": ["x"]})

    def test_leer(self):
        with self.assertRaises(ValueError):
            totg_rechner.params_fuer_moveitpy("")


class Verketten(unittest.TestCase):

    def test_zwei_abschnitte(self):
        eins = [[0.0, [0.0], [0.0]], [0.5, [0.5], [1.0]], [1.0, [1.0], [0.0]]]
        zwei = [[0.0, [1.0], [0.0]], [0.4, [0.8], [-1.0]], [0.8, [0.5], [0.0]]]
        punkte, teilbahnen = totg_rechner.verketten([eins, zwei])
        self.assertEqual([p[0] for p in punkte], [0.0, 0.5, 1.0, 1.4, 1.8])
        self.assertEqual([p[1][0] for p in punkte], [0.0, 0.5, 1.0, 0.8, 0.5])
        self.assertEqual(teilbahnen, [3, 2])

    def test_ein_abschnitt_unveraendert(self):
        eins = [[0.0, [0.0], [0.0]], [1.0, [1.0], [0.0]]]
        punkte, teilbahnen = totg_rechner.verketten([eins])
        self.assertEqual(punkte, eins)
        self.assertEqual(teilbahnen, [2])

    def test_abschnitt_ohne_bewegung(self):
        eins = [[0.0, [0.0], [0.0]], [1.0, [1.0], [0.0]]]
        still = [[0.0, [1.0], [0.0]]]
        drei = [[0.0, [1.0], [0.0]], [2.0, [3.0], [0.0]]]
        punkte, teilbahnen = totg_rechner.verketten([eins, still, drei])
        self.assertEqual([p[0] for p in punkte], [0.0, 1.0, 3.0])
        self.assertEqual(teilbahnen, [2, 0, 1])

    def test_leer(self):
        self.assertEqual(totg_rechner.verketten([]), ([], []))


class Stand(unittest.TestCase):

    def test_neuerer_auftrag_abgelehnt(self):
        with self.assertRaises(RuntimeError) as fall:
            totg_rechner.rechne({"stand": totg_rechner.STAND + 1}, "/tmp")
        self.assertIn("Stand", str(fall.exception))

    def test_stand_passt_zum_modul(self):
        import totg_folge
        self.assertEqual(totg_rechner.STAND, totg_folge.STAND)


class Antwort(unittest.TestCase):

    def test_atomar_und_lesbar(self):
        with tempfile.TemporaryDirectory() as ordner:
            pfad = os.path.join(ordner, "antwort.json")
            totg_rechner.schreibe_antwort(pfad, {"ok": True, "x": 0.1 + 0.2})
            with open(pfad, encoding="utf-8") as d:
                self.assertEqual(json.load(d)["x"], 0.1 + 0.2)
            self.assertEqual(os.listdir(ordner), ["antwort.json"])

    def test_scheitern_liefert_antwort(self):
        with tempfile.TemporaryDirectory() as ordner:
            auftrag = os.path.join(ordner, "auftrag.json")
            antwort = os.path.join(ordner, "antwort.json")
            with open(auftrag, "w", encoding="utf-8") as d:
                json.dump({"params_datei": os.path.join(ordner, "fehlt.yaml")}, d)
            umgebung = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
            lauf = subprocess.run([sys.executable,
                                   os.path.join(HIER, "totg_rechner.py"),
                                   auftrag, antwort],
                                  capture_output=True, text=True, timeout=60,
                                  env=umgebung)
            self.assertEqual(lauf.returncode, 1)
            with open(antwort, encoding="utf-8") as d:
                daten = json.load(d)
            self.assertFalse(daten["ok"])
            self.assertTrue(daten["fehler"])

    def test_falscher_aufruf(self):
        lauf = subprocess.run([sys.executable,
                               os.path.join(HIER, "totg_rechner.py")],
                              capture_output=True, text=True, timeout=30)
        self.assertEqual(lauf.returncode, 2)


if __name__ == "__main__":
    unittest.main(verbosity=1)
