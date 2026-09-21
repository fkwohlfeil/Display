#!/usr/bin/env python3
"""Pruefung der Folgenfahrt aus urdf_gui3_3.py.

Zwei Arten von Pruefungen stehen hier nebeneinander:

1. bahn_startabweichung als reine Funktion. Zahlen hinein, Zahlen heraus,
   kein Fenster und kein Roboter.

2. _seq_drive, _seq_drop und _seq_locks an einer Attrappe. Die drei
   Methoden haengen an der Oberflaeche, ruehren aber kein Tk-Widget an:
   sie lesen Zustand, entscheiden und rufen weiter. Deshalb lassen sie
   sich ungebunden aufrufen -- UrdfViewer._seq_drive(attrappe) -- und
   ohne Bildschirm pruefen. Was sie weiterrufen, zeichnet die Attrappe
   auf; gefahren wird dabei nichts.

Aufruf:
    python3 -m unittest test_folge_fahren -v
    GUI_MODUL=_mutant_gui python3 -m unittest test_folge_fahren   # Sonde
"""

import importlib
import os
import unittest


MODUL = os.environ.get("GUI_MODUL", "urdf_gui3_4")
gui = importlib.import_module(MODUL)


def pruefe_stand():
    """Abbrechen, wenn die falsche Fassung importiert wurde.

    Ohne diese Pruefung liefe die Datei gegen urdf_gui3_2.py mit einem
    Dutzend AttributeError durch und saehe aus wie ein kaputter Patch.
    Sie ist keiner -- es ist die falsche Datei.
    """
    fehlt = [name for name in ("bahn_startabweichung", "FOLGE_START_TOL")
             if not hasattr(gui, name)]
    fehlt += [name for name in ("_seq_drive", "_seq_drop", "_seq_locks")
              if not hasattr(gui.UrdfViewer, name)]
    if fehlt:
        raise SystemExit(
            f"{MODUL} kennt {', '.join(fehlt)} nicht. Das ist eine Fassung "
            f"vor der Folgenfahrt -- erst patch_folge_fahren_2.py anwenden.")


pruefe_stand()


# ==========================================================================
# Attrappen
# ==========================================================================

class Taste:
    """Gerade so viel Knopf, wie _seq_locks anfasst."""

    def __init__(self):
        self.state = "normal"
        self.da = True

    def winfo_exists(self):
        return self.da

    def configure(self, **kwargs):
        self.state = kwargs.get("state", self.state)


class Var:
    def __init__(self):
        self.wert = ""

    def set(self, wert):
        self.wert = wert

    def get(self):
        return self.wert


class Attrappe:
    """Ein Zustand ohne Fenster, ohne ROS und ohne Roboter."""

    def __init__(self, punkte=None, stand=None, hold=True):
        punkte = punkte if punkte is not None else BAHN
        self.state = gui.STATE_READY
        self.stopped = False
        self.positions = dict(stand if stand is not None else {"j1": 0.0,
                                                               "j2": 0.5})
        self.seq_points = list(punkte)
        self.seq_hold = gui.TrajectoryPlayer(punkte) if hold else None
        self.seq_hold_start = dict(self.positions) if hold else None
        self.seq_drive_button = Taste()
        self.seq_status_var = Var()
        self.protokoll = []                            # (text, stufe)
        self.bericht = []
        self.gefahren = []                             # uebergebene Player
        self.sperren = 0

    # -- was die Methoden weiterrufen --------------------------------------

    def _log(self, text, level="info"):
        self.protokoll.append((text, level))

    def _seq_write(self, text, level="info", clear=False):
        self.bericht.append((text, level))

    def _start_playing(self, player, from_hold=False):
        self.gefahren.append(player)

    def _seq_drop(self, grund=""):
        # Die echte Methode und keine Attrappe davon: _seq_drive ruft sie
        # in vier von fuenf Abweisungen, und was dabei mit der gehaltenen
        # Bahn geschieht, ist gerade das, was hier geprueft wird.
        gui.UrdfViewer._seq_drop(self, grund)

    def _update_locks(self):
        self.sperren += 1
        gui.UrdfViewer._seq_locks(self)

    def _busy(self):
        return self.state in gui.STATES_BUSY

    # -- Abkuerzungen fuer die Pruefungen -----------------------------------

    def fahren(self):
        gui.UrdfViewer._seq_drive(self)

    def verwerfen(self, grund=""):
        gui.UrdfViewer._seq_drop(self, grund)

    @property
    def fehler(self):
        return [t for t, s in self.protokoll if s == "error"]


# Zwei Stuetzpunkte genuegen fuer den TrajectoryPlayer. Der Startpunkt
# liegt genau auf dem Stand der Attrappe -- alles andere waere schon der
# Fall, den eine der Pruefungen unten sucht.
BAHN = [(0.0, {"j1": 0.0, "j2": 0.5}, {"j1": 0.0, "j2": 0.0}),
        (1.0, {"j1": 0.2, "j2": 0.5}, {"j1": 0.0, "j2": 0.0})]


# ==========================================================================
# Die reine Rechnung
# ==========================================================================

class StartabweichungTest(unittest.TestCase):

    def test_leere_bahn(self):
        self.assertEqual(gui.bahn_startabweichung([], {"j1": 1.0}),
                         (None, 0.0, ()))

    def test_groesste_gewinnt(self):
        name, ab, fehlend = gui.bahn_startabweichung(
            [(0.0, {"j1": 1.0, "j2": 2.0}, {})], {"j1": 1.1, "j2": 2.5})
        self.assertEqual(name, "j2")
        self.assertAlmostEqual(ab, 0.5)
        self.assertEqual(fehlend, ())

    def test_betrag_ohne_vorzeichen(self):
        """Eine Achse unter dem Sollwert weicht genauso ab wie darueber."""
        _n, ab, _f = gui.bahn_startabweichung(
            [(0.0, {"j1": 1.0}, {})], {"j1": 0.4})
        self.assertAlmostEqual(ab, 0.6)

    def test_fehlende_achse_wird_genannt(self):
        name, ab, fehlend = gui.bahn_startabweichung(
            [(0.0, {"j1": 1.0, "j9": 0.0}, {})], {"j1": 1.0})
        self.assertEqual(fehlend, ("j9",))
        self.assertEqual(name, "j1")
        self.assertAlmostEqual(ab, 0.0)

    def test_nur_erster_punkt_zaehlt(self):
        """Was hinten in der Bahn steht, ist kein Startpunkt."""
        _n, ab, _f = gui.bahn_startabweichung(
            [(0.0, {"j1": 1.0}, {}), (1.0, {"j1": 9.0}, {})], {"j1": 1.0})
        self.assertAlmostEqual(ab, 0.0)


# ==========================================================================
# Die Fahrt
# ==========================================================================

class FahrtTest(unittest.TestCase):

    def test_ohne_bahn_faehrt_nichts(self):
        a = Attrappe(hold=False)
        a.fahren()
        self.assertEqual(a.gefahren, [])
        self.assertTrue(a.fehler)

    def test_stop_haelt_die_fahrt_zurueck(self):
        a = Attrappe()
        a.stopped = True
        a.fahren()
        self.assertEqual(a.gefahren, [])
        self.assertIsNotNone(a.seq_hold)               # nicht verworfen

    def test_laufender_auftrag_haelt_die_fahrt_zurueck(self):
        a = Attrappe()
        a.state = gui.STATE_MOVING
        a.fahren()
        self.assertEqual(a.gefahren, [])
        self.assertIsNotNone(a.seq_hold)

    def test_bewegte_achse_verwirft_statt_zu_fahren(self):
        a = Attrappe()
        a.positions["j1"] = 0.01                       # jemand hat gestellt
        a.fahren()
        self.assertEqual(a.gefahren, [])
        self.assertIsNone(a.seq_hold)
        # Und zwar an der ersten Pruefung. Die zweite faenge denselben Fall
        # auch, aber mit einem Grund, der auf move_group zeigt statt auf die
        # gestellte Achse -- und wer den liest, sucht an der falschen Stelle.
        self.assertTrue(any("seit dem Planen" in t for t, _s in a.protokoll),
                        "verworfen, aber nicht wegen des Achsstands")

    def test_fremde_achse_in_der_bahn_verwirft(self):
        bahn = [(0.0, {"j1": 0.0, "j7": 0.0}, {}),
                (1.0, {"j1": 0.2, "j7": 0.0}, {})]
        a = Attrappe(punkte=bahn, stand={"j1": 0.0, "j2": 0.5})
        a.seq_hold_start = {"j1": 0.0, "j2": 0.5}
        a.fahren()
        self.assertEqual(a.gefahren, [])
        self.assertIsNone(a.seq_hold)

    def test_bahn_neben_dem_stand_verwirft(self):
        """Der Stand stimmt mit dem Planstand, die Bahn faengt woanders an."""
        versetzt = [(0.0, {"j1": 0.02, "j2": 0.5}, {}),
                    (1.0, {"j1": 0.2, "j2": 0.5}, {})]
        a = Attrappe(punkte=versetzt)
        a.seq_hold_start = dict(a.positions)           # niemand hat gestellt
        a.fahren()
        self.assertEqual(a.gefahren, [])
        self.assertIsNone(a.seq_hold)
        self.assertTrue(a.fehler)

    def test_rundungsrest_faehrt_trotzdem(self):
        """Unterhalb von FOLGE_START_TOL ist kein Sprung, sondern Rauschen."""
        rest = gui.FOLGE_START_TOL / 10.0
        knapp = [(0.0, {"j1": rest, "j2": 0.5}, {}),
                 (1.0, {"j1": 0.2, "j2": 0.5}, {})]
        a = Attrappe(punkte=knapp)
        a.seq_hold_start = dict(a.positions)
        a.fahren()
        self.assertEqual(len(a.gefahren), 1)

    def test_gute_bahn_faehrt(self):
        a = Attrappe()
        player = a.seq_hold
        a.fahren()
        self.assertEqual(a.gefahren, [player])
        self.assertIsNone(a.seq_hold)                  # uebergeben, nicht kopiert
        self.assertIsNone(a.seq_hold_start)

    def test_zweimal_fahren_geht_nicht(self):
        a = Attrappe()
        a.fahren()
        a.fahren()
        self.assertEqual(len(a.gefahren), 1)


# ==========================================================================
# Verwerfen und Sperren
# ==========================================================================

class VerwerfenTest(unittest.TestCase):

    def test_punkte_bleiben_liegen(self):
        a = Attrappe()
        vorher = list(a.seq_points)
        a.verwerfen("Grund")
        self.assertIsNone(a.seq_hold)
        self.assertEqual(a.seq_points, vorher)

    def test_grund_steht_im_protokoll(self):
        a = Attrappe()
        a.verwerfen("Testgrund")
        self.assertTrue(any("Testgrund" in t for t, _s in a.protokoll))

    def test_ohne_bahn_bleibt_es_still(self):
        a = Attrappe(hold=False)
        a.verwerfen("Testgrund")
        self.assertEqual(a.protokoll, [])

    def test_taste_ohne_bahn_stumpf(self):
        a = Attrappe(hold=False)
        gui.UrdfViewer._seq_locks(a)
        self.assertEqual(a.seq_drive_button.state, "disabled")

    def test_taste_mit_bahn_scharf(self):
        a = Attrappe()
        gui.UrdfViewer._seq_locks(a)
        self.assertEqual(a.seq_drive_button.state, "normal")

    def test_taste_bei_stop_stumpf(self):
        a = Attrappe()
        a.stopped = True
        gui.UrdfViewer._seq_locks(a)
        self.assertEqual(a.seq_drive_button.state, "disabled")

    def test_taste_waehrend_der_fahrt_stumpf(self):
        a = Attrappe()
        a.state = gui.STATE_MOVING
        gui.UrdfViewer._seq_locks(a)
        self.assertEqual(a.seq_drive_button.state, "disabled")


if __name__ == "__main__":
    unittest.main(verbosity=2)
