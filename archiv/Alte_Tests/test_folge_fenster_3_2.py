#!/usr/bin/env python3
"""Fenstertest zur Punktfolge in 3.2. Braucht einen Bildschirm.

    xvfb-run -a python3 test_folge_fenster_3_2.py

Neu gegenueber 3.1 ist die Kette: jeder Punkt wird einzeln ueber /compute_ik
aufgeloest, mit der Loesung des Vorgaengers als Startwert, und erst danach
geht die Folge als Achsziele hinaus. Genau das wird hier geprueft -- dass die
Kette laeuft, dass der Startwert wandert, dass sie am richtigen Punkt
abbricht und dass in der Nutzlast danach keine Lage mehr steht.

Die Bruecke ist eine Attrappe. Sie nimmt Anfragen entgegen und legt sie ab;
gesendet wird nichts, und die Antworten stellt der Test selbst. Damit laeuft
er ohne gesourctes Environment und ohne move_group.
"""

import math
import os
import tempfile
import unittest

os.environ.setdefault("DISPLAY", ":99")

import urdf_gui3_2 as gui                              # noqa: E402
import sequenz                                         # noqa: E402


MODELL = """<robot name="folgentest">
  <link name="base_link"/>
  <link name="l1"/><link name="l2"/><link name="tool0"/>
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="l1"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/></joint>
  <joint name="j3" type="revolute"><parent link="l2"/><child link="tool0"/>
    <origin xyz="0.4 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/></joint>
</robot>
"""

_handle, URDF = tempfile.mkstemp(suffix=".urdf", prefix="folgentest_")
os.write(_handle, MODELL.encode("utf-8"))
os.close(_handle)


class Attrappe:
    """Steht an der Stelle von RosBridge. Merkt sich, was hinausging."""

    def __init__(self, echt):
        self._echt = echt
        self.moveit_ready = True
        self.sequence_ready = True
        self.ik = []                                   # Anfragen an /compute_ik
        self.folgen = []                               # an /plan_sequence_path

    def request_ik(self, payload):
        self.ik.append(payload)

    def request_sequence(self, payload):
        self.folgen.append(payload)

    def __getattr__(self, name):
        return getattr(self._echt, name)


class FolgeTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.app = gui.UrdfViewer(URDF)
        cls.app.update()

    @classmethod
    def tearDownClass(cls):
        cls.app.ros.shutdown()
        cls.app.destroy()

    def setUp(self):
        self.app._set_state(gui.STATE_READY)
        self.app.stopped = False
        self.app.moveit_stale = False
        self.app._open_sequence()
        self.app.update()
        self.echt = self.app.ros
        self.ros = Attrappe(self.echt)
        self.app.ros = self.ros

    def tearDown(self):
        self.app.ros = self.echt
        self.app._seq_ik_reset()
        self.app.seq_started = None
        self.app._job_drop()
        self.app._set_state(gui.STATE_READY)
        self.app._close_sequence()
        self.app.update()

    # -- Hilfen ------------------------------------------------------------

    def setze(self, text):
        self.app.seq_text.delete("1.0", "end")
        self.app.seq_text.insert("1.0", text)

    def bericht(self):
        return self.app.seq_report.get("1.0", "end")

    def loesung(self, wert):
        return {"tag": self.app.job_tag, "code": 1,
                "values": {"j1": wert, "j2": 0.0, "j3": 0.0}}

    def kette(self, werte=(0.1, 0.2, 0.3)):
        """Planen anstossen und die Kette mit diesen Loesungen durchlaufen."""
        self.app._seq_plan()
        for wert in werte:
            self.app._on_seq_ik_result(self.loesung(wert))

    # -- Zustandsmaschine --------------------------------------------------

    def test_planen_geht_zuerst_in_die_ik(self):
        self.app._seq_plan()
        self.assertEqual(self.app.state, gui.STATE_SEQ_IK)
        self.assertEqual(len(self.ros.ik), 1)
        self.assertEqual(self.ros.folgen, [])

    def test_erst_nach_allen_punkten_geht_die_folge_hinaus(self):
        self.app._seq_plan()
        self.app._on_seq_ik_result(self.loesung(0.1))
        self.assertEqual(self.ros.folgen, [])
        self.app._on_seq_ik_result(self.loesung(0.2))
        self.assertEqual(self.ros.folgen, [])
        self.app._on_seq_ik_result(self.loesung(0.3))
        self.assertEqual(len(self.ros.folgen), 1)
        self.assertEqual(self.app.state, gui.STATE_SEQUENCE)

    def test_je_punkt_eine_ik_anfrage(self):
        self.kette()
        self.assertEqual(len(self.ros.ik), 3)

    def test_stop_haelt_die_kette_auf(self):
        self.app.stopped = True
        self.app._seq_plan()
        self.assertEqual(self.ros.ik, [])
        self.app.stopped = False

    def test_fehlende_capability_haelt_die_kette_auf(self):
        self.ros.sequence_ready = False
        self.app._seq_plan()
        self.assertEqual(self.ros.ik, [])

    def test_fehlerhafte_liste_geht_nicht_in_die_ik(self):
        self.setze("0 0 0 0 0 0 radius=0.9\n1 0 0 0 0 0 radius=0.9\n"
                   "2 0 0 0 0 0 radius=0\n")
        self.app._seq_plan()
        self.assertEqual(self.ros.ik, [])
        self.assertEqual(self.app.state, gui.STATE_READY)

    def test_bremsen_bricht_die_kette_ab(self):
        self.app._seq_plan()
        self.app._brake("Test")
        self.assertEqual(self.app.state, gui.STATE_READY)
        self.assertEqual(self.app.seq_schritte, ())
        # Eine danach eintreffende Antwort darf nichts mehr ausloesen.
        self.app._on_seq_ik_result({"tag": 1, "code": 1,
                                    "values": {"j1": 0.0, "j2": 0.0,
                                               "j3": 0.0}})
        self.assertEqual(self.ros.folgen, [])

    # -- Der Startwert wandert ---------------------------------------------

    def test_erster_startwert_ist_der_achsstand(self):
        self.app._seq_plan()
        seed = self.ros.ik[0]["seed"]
        for name, value in self.app.positions.items():
            self.assertAlmostEqual(seed[name], value)

    def test_zweiter_startwert_ist_die_erste_loesung(self):
        """Der Kern von 3.2. Genau das kann man Pilz nicht vorgeben."""
        self.app._seq_plan()
        self.app._on_seq_ik_result(self.loesung(0.77))
        self.assertAlmostEqual(self.ros.ik[1]["seed"]["j1"], 0.77)

    def test_dritter_startwert_ist_die_zweite_loesung(self):
        self.app._seq_plan()
        self.app._on_seq_ik_result(self.loesung(0.11))
        self.app._on_seq_ik_result(self.loesung(0.22))
        self.assertAlmostEqual(self.ros.ik[2]["seed"]["j1"], 0.22)

    def test_der_achsstand_bleibt_unberuehrt(self):
        vorher = dict(self.app.positions)
        self.kette()
        self.assertEqual(self.app.positions, vorher)

    def test_startzustand_der_folge_ist_der_achsstand(self):
        """Nicht die letzte IK-Loesung: gefahren wird von dort, wo er steht."""
        self.kette()
        posten = self.ros.folgen[0]["items"][0]
        for name, value in posten["start"].items():
            self.assertAlmostEqual(value, self.app.positions.get(name, 0.0))

    # -- Die Nutzlast ------------------------------------------------------

    def test_alle_posten_tragen_achsziele(self):
        self.kette()
        for posten in self.ros.folgen[0]["items"]:
            self.assertIn("goal", posten)
            self.assertNotIn("pose", posten)

    def test_die_achsziele_sind_die_ik_loesungen(self):
        self.kette(werte=(0.1, 0.2, 0.3))
        ziele = [p["goal"]["j1"] for p in self.ros.folgen[0]["items"]]
        self.assertEqual([round(z, 6) for z in ziele], [0.1, 0.2, 0.3])

    def test_nur_der_erste_posten_traegt_start(self):
        self.kette()
        posten = self.ros.folgen[0]["items"]
        self.assertIn("start", posten[0])
        self.assertNotIn("start", posten[1])
        self.assertNotIn("start", posten[2])

    def test_radien_wandern_durch(self):
        self.kette()
        posten = self.ros.folgen[0]["items"]
        self.assertAlmostEqual(posten[0]["blend_radius"], 0.05)
        self.assertEqual(posten[-1]["blend_radius"], 0.0)

    # -- Abbruch mit Namen -------------------------------------------------

    def test_abbruch_nennt_den_punkt(self):
        self.app._seq_plan()
        self.app._on_seq_ik_result(self.loesung(0.1))
        self.app._on_seq_ik_result({"tag": self.app.job_tag, "code": -31})
        self.assertEqual(self.app.state, gui.STATE_ERROR)
        self.assertIn("P2", self.bericht())
        self.assertEqual(self.ros.folgen, [])

    def test_abbruch_zeigt_die_sechs_zahlen(self):
        self.app._seq_plan()
        self.app._on_seq_ik_result({"tag": self.app.job_tag, "code": -31})
        self.assertIn("Gefragt war", self.bericht())

    def test_abbruch_bei_uebertragungsfehler(self):
        self.app._seq_plan()
        self.app._on_seq_ik_result({"tag": self.app.job_tag,
                                    "error": "Dienst weg"})
        self.assertEqual(self.app.state, gui.STATE_ERROR)
        self.assertIn("Dienst weg", self.bericht())

    def test_unvollstaendige_antwort_faellt_auf(self):
        self.app._seq_plan()
        self.app._on_seq_ik_result({"tag": self.app.job_tag, "code": 1,
                                    "values": {"j1": 0.0}})
        self.assertEqual(self.app.state, gui.STATE_ERROR)
        self.assertIn("j2", self.bericht())

    def test_verspaetete_antwort_faellt_weg(self):
        self.app._seq_plan()
        vorher = len(self.ros.ik)
        self.app._on_seq_ik_result({"tag": self.app.job_tag - 500, "code": 1,
                                    "values": {"j1": 0.0, "j2": 0.0,
                                               "j3": 0.0}})
        self.assertEqual(len(self.ros.ik), vorher)

    # -- Bericht -----------------------------------------------------------

    def test_bericht_zeigt_die_achswerte(self):
        self.kette()
        self.assertIn("groesster Schritt", self.bericht())

    def test_grosser_sprung_wird_gemeldet(self):
        self.app._seq_plan()
        self.app._on_seq_ik_result(self.loesung(math.radians(150.0)))
        self.assertIn("Konfigurationszweig", self.bericht())

    def test_kleiner_schritt_wird_nicht_gemeldet(self):
        self.app._seq_plan()
        self.app._on_seq_ik_result(self.loesung(math.radians(10.0)))
        self.assertNotIn("Konfigurationszweig", self.bericht())

    def test_ruckig_und_radien_werden_gewarnt(self):
        self.app.moveit_config.use_ruckig = True
        self.app._seq_plan()
        self.assertIn("Ruckig", self.bericht())

    def test_ohne_ruckig_keine_warnung(self):
        self.app.moveit_config.use_ruckig = False
        try:
            self.app._seq_plan()
            self.assertNotIn("Ruckig", self.bericht())
        finally:
            self.app.moveit_config.use_ruckig = True

    # -- Antwort der Folge -------------------------------------------------

    def bahn(self, punkte):
        return [(t, {"j1": t * 0.1, "j2": 0.0, "j3": 0.0},
                 {"j1": 0.0, "j2": 0.0, "j3": 0.0}) for t in punkte]

    def test_folgenantwort_wird_uebernommen(self):
        self.kette()
        self.app._on_sequence_result({
            "tag": self.app.job_tag, "code": 1, "teilbahnen": [4],
            "points": self.bahn([0.0, 0.1, 0.2, 0.3]), "planning_time": 0.0})
        self.assertEqual(len(self.app.seq_points), 4)
        self.assertEqual(self.app.state, gui.STATE_READY)

    def test_doppelte_zeitstempel_werden_verworfen(self):
        self.kette()
        self.app._on_sequence_result({
            "tag": self.app.job_tag, "code": 1, "teilbahnen": [4],
            "points": self.bahn([0.0, 0.1, 0.1, 0.2]), "planning_time": 0.0})
        self.assertEqual(self.app.seq_dropped, (2,))

    def test_fehlercode_holt_die_diagnose(self):
        self.kette()
        self.app._on_sequence_result({"tag": self.app.job_tag, "code": 99999,
                                      "points": [], "teilbahnen": []})
        self.assertEqual(self.app.state, gui.STATE_ERROR)

    # -- Fehlertexte -------------------------------------------------------

    def test_99999_meint_nicht_mehr_kein_planer(self):
        text = gui.moveit_error_text(99999)
        self.assertIn("Sammelfehler", text)
        self.assertNotIn("Kein Planer hat geantwortet", text)

    def test_null_meint_keine_antwort(self):
        self.assertIn("bei keinem Planer", gui.moveit_error_text(0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
