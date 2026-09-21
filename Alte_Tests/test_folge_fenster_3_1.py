#!/usr/bin/env python3
"""Fenstertest zum Folgenfenster in 3.1. Braucht einen Bildschirm.

    xvfb-run -a python3 test_folge_fenster_3_1.py

Geprueft wird, was sich ohne ROS pruefen laesst: dass sich das Fenster baut,
dass die Punktliste gelesen und geprueft wird, dass der Bericht das Geerbte
in Klammern zeigt, dass eine fehlerhafte Liste nicht hinausgeht und dass ein
Planungsversuch ohne /plan_sequence_path abgelehnt wird statt aufzulaufen.

Die Bruecke ist eine Attrappe. Sie nimmt Auftraege entgegen und legt sie ab;
gesendet wird nichts. Damit laeuft der Test ohne gesourctes Environment und
ohne move_group -- und was er findet, sind Fehler in der Oberflaeche und
nicht im Netz.
"""

import os
import sys
import tempfile
import unittest

os.environ.setdefault("DISPLAY", ":99")

import urdf_gui3_1 as gui                                # noqa: E402
import sequenz                                           # noqa: E402


# Ein eigenes Modell und nicht bmKinematic.urdf: der Test soll ueberall
# laufen und immer dasselbe pruefen. Haenge er am Arbeitsmodell, aenderte
# sich sein Ergebnis, sobald jemand ein Gelenk verschiebt -- und dann sagt
# ein roter Test nicht mehr, dass die Oberflaeche kaputt ist.
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
        self.auftraege = []

    def request_sequence(self, payload):
        self.auftraege.append(payload)

    def __getattr__(self, name):
        return getattr(self._echt, name)


class FolgenfensterTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.app = gui.UrdfViewer(URDF)
        cls.app.update()

    @classmethod
    def tearDownClass(cls):
        cls.app.ros.shutdown()
        cls.app.destroy()

    def setUp(self):
        self.app.state = gui.STATE_READY
        self.app.stopped = False
        self.app.moveit_stale = False
        self.app._open_sequence()
        self.app.update()

    def tearDown(self):
        self.app._close_sequence()
        self.app.update()

    # -- Aufbau ------------------------------------------------------------

    def test_fenster_baut_sich(self):
        self.assertIsNotNone(self.app.seq_window)
        self.assertTrue(self.app.seq_window.winfo_exists())
        self.assertIsNotNone(self.app.seq_text)
        self.assertIsNotNone(self.app.seq_report)

    def test_beispiel_steht_drin(self):
        text = self.app.seq_text.get("1.0", "end")
        self.assertIn("fahrart=LIN", text)
        self.assertIn("radius=0", text)

    def test_zweimal_oeffnen_baut_nicht_zweimal(self):
        erst = self.app.seq_window
        self.app._open_sequence()
        self.assertIs(self.app.seq_window, erst)

    def test_schliessen_raeumt_auf(self):
        self.app._close_sequence()
        self.app.update()
        self.assertIsNone(self.app.seq_window)
        self.assertIsNone(self.app.seq_text)

    # -- Lesen -------------------------------------------------------------

    def setze(self, text):
        self.app.seq_text.delete("1.0", "end")
        self.app.seq_text.insert("1.0", text)

    def bericht(self):
        return self.app.seq_report.get("1.0", "end")

    def test_beispiel_ist_tragfaehig(self):
        folge = self.app._seq_read()
        self.assertIsNotNone(folge)
        self.assertEqual(len(folge.punkte), 3)

    def test_bericht_zeigt_geerbtes_in_klammern(self):
        self.app._seq_read()
        self.assertIn("(LIN)", self.bericht())

    def test_bericht_zeigt_den_spielraum(self):
        self.app._seq_read()
        self.assertIn("frei", self.bericht())

    def test_ueberlappende_radien_gehen_nicht_durch(self):
        self.setze("0 0 0 0 0 0 radius=0.9\n"
                   "1 0 0 0 0 0 radius=0.9\n"
                   "2 0 0 0 0 0 radius=0\n")
        self.assertIsNone(self.app._seq_read())
        self.assertIn("ueberlappen", self.bericht())

    def test_letzter_radius_faellt_auf(self):
        self.setze("0 0 0 0 0 0 radius=0.05\n"
                   "1 0 0 0 0 0\n")
        self.assertIsNone(self.app._seq_read())
        self.assertIn("letzter Punkt", self.bericht())

    def test_kaputte_zeile_wird_gemeldet(self):
        self.setze("0 0 0 0 0\n1 0 0 0 0 0 radius=0\n")
        self.app._seq_read()
        self.assertIn("Zeile 1", self.bericht())

    def test_leere_liste(self):
        self.setze("# nur ein Kommentar\n")
        self.assertIsNone(self.app._seq_read())

    def test_modalzustand_kommt_aus_den_parametern(self):
        """Was im Parameterfenster steht, gilt auch fuer eine Folge."""
        self.app.moveit_config.plan["velocity_scaling"] = 33.0
        try:
            modal = self.app._seq_modal()
            self.assertEqual(modal["tempo"], 33.0)
            self.setze("0 0 0 0 0 0\n1 0 0 0 0 0\n")
            self.app._seq_read()
            self.assertIn("33%", self.bericht())
        finally:
            self.app.moveit_config.plan["velocity_scaling"] = 100.0

    # -- Lage anfuegen -----------------------------------------------------

    def test_lage_anfuegen(self):
        vorher = self.app.seq_text.get("1.0", "end").count("\n")
        self.app._seq_append_current()
        nachher = self.app.seq_text.get("1.0", "end").count("\n")
        self.assertGreater(nachher, vorher)
        letzte = [z for z in self.app.seq_text.get("1.0", "end").splitlines()
                  if z.strip()][-1]
        self.assertIn("radius=0", letzte)

    def test_angefuegte_lage_ist_lesbar(self):
        self.setze("")
        self.app._seq_append_current()
        self.app._seq_append_current()
        punkte, befunde = sequenz.lies_liste(
            self.app.seq_text.get("1.0", "end"))
        self.assertEqual(befunde, ())
        self.assertEqual(len(punkte), 2)

    # -- Planen ------------------------------------------------------------

    def test_planen_ohne_capability_wird_abgelehnt(self):
        echt, self.app.ros = self.app.ros, Attrappe(self.app.ros)
        self.app.ros.sequence_ready = False
        try:
            self.app._seq_plan()
            self.assertEqual(self.app.ros.auftraege, [])
            self.assertEqual(self.app.state, gui.STATE_READY)
        finally:
            self.app.ros = echt

    def test_planen_bei_stop_wird_abgelehnt(self):
        echt, self.app.ros = self.app.ros, Attrappe(self.app.ros)
        self.app.stopped = True
        try:
            self.app._seq_plan()
            self.assertEqual(self.app.ros.auftraege, [])
        finally:
            self.app.stopped = False
            self.app.ros = echt

    def test_auftrag_traegt_die_posten(self):
        echt, self.app.ros = self.app.ros, Attrappe(self.app.ros)
        try:
            self.app._seq_plan()
            self.assertEqual(len(self.app.ros.auftraege), 1)
            posten = self.app.ros.auftraege[0]["items"]
            self.assertEqual(len(posten), 3)
            self.assertEqual(self.app.state, gui.STATE_SEQUENCE)
        finally:
            self.app.ros = echt
            self.app._set_state(gui.STATE_READY)

    def test_nur_der_erste_posten_traegt_start(self):
        echt, self.app.ros = self.app.ros, Attrappe(self.app.ros)
        try:
            self.app._seq_plan()
            posten = self.app.ros.auftraege[0]["items"]
            self.assertIn("start", posten[0])
            self.assertNotIn("start", posten[1])
            self.assertNotIn("start", posten[2])
        finally:
            self.app.ros = echt
            self.app._set_state(gui.STATE_READY)

    def test_lage_wird_zu_quaternion_und_bezug(self):
        """Der GUI-Thread rechnet, die Bruecke baut nur Nachrichten."""
        echt, self.app.ros = self.app.ros, Attrappe(self.app.ros)
        try:
            self.app._seq_plan()
            lage = self.app.ros.auftraege[0]["items"][0]["pose"]
            self.assertEqual(set(lage), {"frame", "tip", "position",
                                         "orientation", "position_tolerance"})
            self.assertEqual(len(lage["orientation"]), 4)
            self.assertNotIn("abc", lage)
        finally:
            self.app.ros = echt
            self.app._set_state(gui.STATE_READY)

    def test_radien_wandern_in_den_auftrag(self):
        echt, self.app.ros = self.app.ros, Attrappe(self.app.ros)
        try:
            self.app._seq_plan()
            posten = self.app.ros.auftraege[0]["items"]
            self.assertAlmostEqual(posten[0]["blend_radius"], 0.05)
            self.assertAlmostEqual(posten[1]["blend_radius"], 0.05)
            self.assertEqual(posten[2]["blend_radius"], 0.0)
        finally:
            self.app.ros = echt
            self.app._set_state(gui.STATE_READY)

    def test_fehlerhafte_liste_geht_nicht_hinaus(self):
        self.setze("0 0 0 0 0 0 radius=0.9\n1 0 0 0 0 0 radius=0.9\n"
                   "2 0 0 0 0 0 radius=0\n")
        echt, self.app.ros = self.app.ros, Attrappe(self.app.ros)
        try:
            self.app._seq_plan()
            self.assertEqual(self.app.ros.auftraege, [])
            self.assertEqual(self.app.state, gui.STATE_READY)
        finally:
            self.app.ros = echt

    # -- Antwort -----------------------------------------------------------

    def bahn(self, punkte):
        return [(t, {"j1": t * 0.1, "j2": 0.0, "j3": 0.0},
                 {"j1": 0.0, "j2": 0.0, "j3": 0.0})
                for t in punkte]

    def plane(self):
        echt, self.app.ros = self.app.ros, Attrappe(self.app.ros)
        try:
            self.app._seq_plan()
            return self.app.job_tag
        finally:
            self.app.ros = echt

    def test_antwort_wird_uebernommen(self):
        tag = self.plane()
        self.app._on_sequence_result({
            "tag": tag, "code": 1, "teilbahnen": [5],
            "points": self.bahn([0.0, 0.1, 0.2, 0.3, 0.4]),
            "planning_time": 0.01})
        self.assertEqual(len(self.app.seq_points), 5)
        self.assertEqual(self.app.state, gui.STATE_READY)
        self.assertIn("Teilbahn", self.bericht())

    def test_eine_teilbahn_wird_als_erfolg_gemeldet(self):
        tag = self.plane()
        self.app._on_sequence_result({
            "tag": tag, "code": 1, "teilbahnen": [4],
            "points": self.bahn([0.0, 0.1, 0.2, 0.3]), "planning_time": 0.0})
        self.assertIn("verschmolzen", self.bericht())

    def test_mehrere_teilbahnen_werden_gemeldet(self):
        tag = self.plane()
        self.app._on_sequence_result({
            "tag": tag, "code": 1, "teilbahnen": [2, 2],
            "points": self.bahn([0.0, 0.1, 0.2, 0.3]), "planning_time": 0.0})
        self.assertIn("angehalten", self.bericht())

    def test_doppelte_zeitstempel_werden_verworfen(self):
        tag = self.plane()
        self.app._on_sequence_result({
            "tag": tag, "code": 1, "teilbahnen": [4],
            "points": self.bahn([0.0, 0.1, 0.1, 0.2]), "planning_time": 0.0})
        self.assertEqual(len(self.app.seq_points), 3)
        self.assertEqual(self.app.seq_dropped, (2,))

    def test_fehlercode_setzt_fehlerzustand(self):
        tag = self.plane()
        self.app._on_sequence_result({"tag": tag, "code": -1, "points": [],
                                      "teilbahnen": []})
        self.assertEqual(self.app.state, gui.STATE_ERROR)
        self.assertEqual(self.app.seq_points, [])
        self.app._set_state(gui.STATE_READY)

    def test_verspaetete_antwort_faellt_weg(self):
        tag = self.plane()
        self.app._on_sequence_result({
            "tag": tag - 1000, "code": 1, "teilbahnen": [2],
            "points": self.bahn([0.0, 0.1]), "planning_time": 0.0})
        self.assertEqual(self.app.seq_points, [])
        self.app._set_state(gui.STATE_READY)

    def test_antwort_ohne_zustand_faellt_weg(self):
        tag = self.plane()
        self.app._set_state(gui.STATE_READY)
        self.app._on_sequence_result({
            "tag": tag, "code": 1, "teilbahnen": [2],
            "points": self.bahn([0.0, 0.1]), "planning_time": 0.0})
        self.assertEqual(self.app.seq_points, [])

    def test_einzelner_punkt_ist_keine_bahn(self):
        tag = self.plane()
        self.app._on_sequence_result({
            "tag": tag, "code": 1, "teilbahnen": [1],
            "points": self.bahn([0.0]), "planning_time": 0.0})
        self.assertEqual(self.app.seq_points, [])
        self.assertEqual(self.app.state, gui.STATE_READY)

    # -- Ausgabe -----------------------------------------------------------

    def test_export_ohne_folge_schreibt_nichts(self):
        self.app.seq_points = []
        self.app._seq_export()                          # darf nur protokollieren

    def test_export_ruehrt_den_angezeigten_stand_nicht_an(self):
        tag = self.plane()
        self.app._on_sequence_result({
            "tag": tag, "code": 1, "teilbahnen": [3],
            "points": self.bahn([0.0, 0.1, 0.2]), "planning_time": 0.0})
        vorher = dict(self.app.positions)
        pfad = "/tmp/folge_test.csv"
        gui.filedialog.asksaveasfilename = lambda **kw: pfad
        self.app._seq_export()
        self.assertEqual(self.app.positions, vorher)
        self.assertTrue(os.path.exists(pfad))

    def test_export_schreibt_alle_stuetzpunkte(self):
        tag = self.plane()
        self.app._on_sequence_result({
            "tag": tag, "code": 1, "teilbahnen": [4],
            "points": self.bahn([0.0, 0.1, 0.2, 0.3]), "planning_time": 0.0})
        pfad = "/tmp/folge_test2.csv"
        gui.filedialog.asksaveasfilename = lambda **kw: pfad
        self.app._seq_export()
        with open(pfad) as handle:
            zeilen = [z for z in handle if not z.startswith("#")]
        self.assertEqual(len(zeilen), 5)                # Kopf plus vier Punkte

    def test_export_filtert_nicht(self):
        """Vier Stuetzpunkte am selben Ort: ein Rekorder mit Schwellen
        wuerde drei davon wegwerfen. Der Planer hat sie gesetzt, also
        gehoeren sie in die Datei."""
        tag = self.plane()
        steht = [(t, {"j1": 0.0, "j2": 0.0, "j3": 0.0},
                  {"j1": 0.0, "j2": 0.0, "j3": 0.0})
                 for t in (0.0, 0.1, 0.2, 0.3)]
        self.app._on_sequence_result({
            "tag": tag, "code": 1, "teilbahnen": [4],
            "points": steht, "planning_time": 0.0})
        pfad = "/tmp/folge_test4.csv"
        gui.filedialog.asksaveasfilename = lambda **kw: pfad
        self.app._seq_export()
        with open(pfad) as handle:
            zeilen = [z for z in handle if not z.startswith("#")]
        self.assertEqual(len(zeilen), 5)

    def test_export_traegt_die_konvention_im_kopf(self):
        tag = self.plane()
        self.app._on_sequence_result({
            "tag": tag, "code": 1, "teilbahnen": [2],
            "points": self.bahn([0.0, 0.1]), "planning_time": 0.0})
        pfad = "/tmp/folge_test3.csv"
        gui.filedialog.asksaveasfilename = lambda **kw: pfad
        self.app._seq_export()
        with open(pfad) as handle:
            kopf = handle.read(2000)
        self.assertIn("#drehkonvention", kopf)


if __name__ == "__main__":
    unittest.main(verbosity=2)
