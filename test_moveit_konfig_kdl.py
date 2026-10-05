#!/usr/bin/env python3
"""Tests fuer die KDL-Parameter in moveit_konfig.py. Ohne ROS, ohne Tk.

    python3 test_moveit_konfig_kdl.py
"""

import unittest

import yaml

import moveit_konfig as M
import totg_rechner

URDF = '<robot name="r"><link name="a"/></robot>'
SRDF = '<robot name="r"/>'


def kinematik(config, text=None):
    daten = yaml.safe_load(text or config.params_yaml(URDF, SRDF))
    return daten["/**"]["ros__parameters"]["robot_description_kinematics"][
        config.group]


class Kdl(unittest.TestCase):

    def test_standardwerte_wie_moveit(self):
        # kdl_kinematics_parameters.hpp, moveit_kinematics 2.12.4
        config = M.MoveItConfig()
        self.assertEqual(config.kdl, {"max_solver_iterations": 500,
                                      "epsilon": 1e-5,
                                      "orientation_vs_position": 1.0})
        self.assertIs(config.kdl_position_only, False)
        self.assertEqual(config.solver, M.KDL_SOLVER)

    def test_in_der_datei_mit_typen(self):
        config = M.MoveItConfig()
        config.kdl.update(max_solver_iterations=2000, epsilon=1e-7,
                          orientation_vs_position=2.0)
        config.kdl_position_only = True
        werte = kinematik(config)
        self.assertEqual(werte["kinematics_solver"], M.KDL_SOLVER)
        # Ganzzahl bleibt Ganzzahl, sonst bricht das Plugin beim Laden ab.
        self.assertIs(type(werte["max_solver_iterations"]), int)
        self.assertEqual(werte["max_solver_iterations"], 2000)
        self.assertIs(type(werte["epsilon"]), float)
        self.assertEqual(werte["epsilon"], 1e-7)
        self.assertIs(type(werte["orientation_vs_position"]), float)
        self.assertEqual(werte["orientation_vs_position"], 2.0)
        self.assertIs(werte["position_only_ik"], True)

    def test_float_eingabe_wird_ganzzahl(self):
        # Die GUI liest Felder als float; 2000.0 muss als 2000 hinaus.
        config = M.MoveItConfig()
        config.kdl["max_solver_iterations"] = 2000.0
        self.assertIn("        max_solver_iterations: 2000\n",
                      config.params_yaml(URDF, SRDF))
        # Ein ganzzahliges Gewicht dagegen als double.
        config.kdl["orientation_vs_position"] = 1
        self.assertIn("        orientation_vs_position: 1.0\n",
                      config.params_yaml(URDF, SRDF))

    def test_nur_fuer_kdl(self):
        config = M.MoveItConfig()
        config.solver = M.KINEMATICS_SOLVERS[1]
        werte = kinematik(config)
        for key in ("max_solver_iterations", "epsilon",
                    "orientation_vs_position", "position_only_ik"):
            self.assertNotIn(key, werte)

    def test_neustart_kennung(self):
        config = M.MoveItConfig()
        vorher = config.startup_signature()
        config.kdl["epsilon"] = 1e-6
        self.assertNotEqual(config.startup_signature(), vorher)
        mitte = config.startup_signature()
        config.kdl_position_only = True
        self.assertNotEqual(config.startup_signature(), mitte)

    def test_moveitpy_abschrift_behaelt_typen(self):
        # Der Intrinsic-Rechner liest die Datei ueber params_fuer_moveitpy.
        config = M.MoveItConfig()
        config.kdl["max_solver_iterations"] = 1500
        text = totg_rechner.params_fuer_moveitpy(
            config.params_yaml(URDF, SRDF))
        werte = kinematik(config, text)
        self.assertIs(type(werte["max_solver_iterations"]), int)
        self.assertEqual(werte["max_solver_iterations"], 1500)
        self.assertIs(type(werte["epsilon"]), float)


if __name__ == "__main__":
    unittest.main(verbosity=1)
