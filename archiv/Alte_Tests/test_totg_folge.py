#!/usr/bin/env python3
"""Pruefung von totg_folge.py. Ohne Bildschirm, ohne ROS, ohne MoveIt.

    python3 test_totg_folge.py

Das Modul wird ueber MODUL gewaehlt, damit mutation_totg_folge.py dieselben
Faelle gegen veraenderte Fassungen laufen lassen kann.
"""

import importlib
import json
import math
import os
import unittest

MODUL = os.environ.get("TOTG_FOLGE_MODUL", "totg_folge")
tf = importlib.import_module(MODUL)
import sequenz  # noqa: E402


def nutzlast(ziele, tempo=0.5, beschl=0.4, fahrart="PTP", radien=None):
    """Nutzlast wie sequenz.Folge.nutzlast, zwei Achsen a und b."""
    posten = []
    for index, ziel in enumerate(ziele):
        posten.append({
            "group": "manipulator", "planner_id": fahrart,
            "velocity_scaling": tempo, "acceleration_scaling": beschl,
            "blend_radius": (radien or [0.0] * len(ziele))[index],
            "name": f"P{index + 1}", "goal": {"a": ziel[0], "b": ziel[1]},
        })
    posten[0]["start"] = {"a": 0.0, "b": 0.0, "c_mimic": 0.0}
    return {"group": "manipulator", "items": posten, "tag": 7}


def stufen(befunde, stufe):
    return [b for b in befunde if b.stufe == stufe]


class Parameter(unittest.TestCase):

    def test_gueltig(self):
        self.assertEqual(tf.pruefe_parameter(0.1, 0.01), ())

    def test_toleranz_null(self):
        self.assertEqual(len(tf.pruefe_parameter(0.0, 0.01)), 1)

    def test_toleranz_negativ(self):
        self.assertEqual(len(tf.pruefe_parameter(-0.1, 0.01)), 1)

    def test_toleranz_obergrenze(self):
        self.assertEqual(tf.pruefe_parameter(tf.PATH_TOLERANCE_MAX, 0.01), ())
        self.assertEqual(len(tf.pruefe_parameter(tf.PATH_TOLERANCE_MAX * 1.01, 0.01)), 1)

    def test_takt_grenzen(self):
        self.assertEqual(tf.pruefe_parameter(0.1, tf.RESAMPLE_DT_MIN), ())
        self.assertEqual(tf.pruefe_parameter(0.1, tf.RESAMPLE_DT_MAX), ())
        self.assertEqual(len(tf.pruefe_parameter(0.1, tf.RESAMPLE_DT_MIN / 2)), 1)
        self.assertEqual(len(tf.pruefe_parameter(0.1, tf.RESAMPLE_DT_MAX * 2)), 1)

    def test_keine_zahl(self):
        befunde = tf.pruefe_parameter("x", "y")
        self.assertEqual(len(befunde), 2)
        self.assertTrue(all(b.stufe == sequenz.FEHLER for b in befunde))

    def test_standard_ist_gueltig(self):
        self.assertEqual(tf.pruefe_parameter(tf.PATH_TOLERANCE_STANDARD,
                                             tf.RESAMPLE_DT_STANDARD), ())
        self.assertEqual(tf.PATH_TOLERANCE_STANDARD, 0.1)
        self.assertEqual(tf.MIN_ANGLE_CHANGE, 0.001)


class DoppeltePunkte(unittest.TestCase):

    def test_alle_verschieden(self):
        self.assertEqual(tf.doppelte_punkte([[0, 0], [1, 0], [1, 1]]), [])

    def test_mitte_zu_nah(self):
        punkte = [[0, 0], [0.0005, 0], [1, 0]]
        self.assertEqual(tf.doppelte_punkte(punkte), [(1, 0)])

    def test_grenze_ist_ausschliesslich(self):
        # MoveIt behaelt nur bei > min_angle_change, nicht bei >=.
        punkte = [[0.0, 0.0], [0.001, 0.0], [1.0, 0.0]]
        self.assertEqual(tf.doppelte_punkte(punkte, 0.001), [(1, 0)])
        punkte = [[0.0, 0.0], [0.0011, 0.0], [1.0, 0.0]]
        self.assertEqual(tf.doppelte_punkte(punkte, 0.001), [])

    def test_eine_achse_genuegt(self):
        punkte = [[0, 0], [0, 0.5], [1, 0.5]]
        self.assertEqual(tf.doppelte_punkte(punkte), [])

    def test_letzter_ersetzt(self):
        punkte = [[0, 0], [1, 0], [1.0004, 0]]
        self.assertEqual(tf.doppelte_punkte(punkte), [(1, 2)])

    def test_vergleich_mit_behaltenem(self):
        # Kleine Schritte summieren sich: verglichen wird mit dem zuletzt
        # behaltenen, nicht mit dem direkten Vorgaenger.
        punkte = [[0.0], [0.0006], [0.0012], [0.0018]]
        self.assertEqual(tf.doppelte_punkte(punkte, 0.001), [(1, 0), (2, 3)])

    def test_eigene_grenze(self):
        punkte = [[0, 0], [0.05, 0], [1, 0]]
        self.assertEqual(tf.doppelte_punkte(punkte, 0.1), [(1, 0)])


class Abschnitte(unittest.TestCase):

    def test_behaltene(self):
        self.assertEqual(tf.behaltene_punkte([[0], [0.0005], [1], [1.0004]]),
                         [0, 3])
        self.assertEqual(tf.behaltene_punkte([[0], [1], [2]]), [0, 1, 2])
        self.assertEqual(tf.behaltene_punkte([]), [])

    def test_behaltene_grenze(self):
        self.assertEqual(tf.behaltene_punkte([[0.0], [0.001], [1.0]], 0.001),
                         [0, 2])
        self.assertEqual(tf.behaltene_punkte([[0.0], [0.0011], [1.0]], 0.001),
                         [0, 1, 2])

    def test_umkehr_gerade(self):
        self.assertTrue(tf.ist_umkehr([0, 0], [1, 0], [0.5, 0]))
        self.assertTrue(tf.ist_umkehr([0, 0], [1, 1], [-3, -3]))

    def test_keine_umkehr(self):
        self.assertFalse(tf.ist_umkehr([0, 0], [1, 0], [2, 0]))
        self.assertFalse(tf.ist_umkehr([0, 0], [1, 0], [1, 1]))
        self.assertFalse(tf.ist_umkehr([0, 0], [1, 0], [0, 0.01]))

    def test_umkehr_toleranz(self):
        # rein = (1, 0); raus unter dem Winkel w. Umkehr ab cos w <= -1 + 1e-5.
        def mit_kosinus(k):
            w = math.acos(k)
            return tf.ist_umkehr([0, 0], [1, 0], [1 + math.cos(w), math.sin(w)])
        self.assertTrue(mit_kosinus(-1.0))
        self.assertTrue(mit_kosinus(-1.0 + 0.5e-5))
        self.assertFalse(mit_kosinus(-1.0 + 2e-5))
        self.assertEqual(tf.UMKEHR_TOLERANZ, 1e-05)

    def test_nullstrecke_keine_umkehr(self):
        self.assertFalse(tf.ist_umkehr([0, 0], [0, 0], [1, 0]))
        self.assertFalse(tf.ist_umkehr([0, 0], [1, 0], [1, 0]))

    def test_ohne_alles(self):
        self.assertEqual(tf.abschnitte([[0], [1], [2]]), ([0, 2], []))

    def test_halte(self):
        self.assertEqual(tf.abschnitte([[0], [1], [2], [3]], [2]), ([0, 2, 3], []))

    def test_halte_am_rand_ignoriert(self):
        self.assertEqual(tf.abschnitte([[0], [1], [2]], [0, 2, 7]), ([0, 2], []))
        self.assertEqual(tf.abschnitte([[0], [1], [2]], [7]), ([0, 2], []))
        self.assertEqual(tf.abschnitte([[0], [1], [2]], [-1]), ([0, 2], []))

    def test_umkehr(self):
        punkte = [[0, 0], [1, 0], [0, 0], [0, 1]]
        self.assertEqual(tf.abschnitte(punkte), ([0, 1, 3], [1]))

    def test_zwei_umkehren(self):
        punkte = [[0], [1], [0], [1]]
        self.assertEqual(tf.abschnitte(punkte), ([0, 1, 2, 3], [1, 2]))

    def test_umkehr_hinter_dichtem_punkt(self):
        # Der Punkt bei 1.0005 faellt weg; die Umkehr liegt am behaltenen.
        punkte = [[0.0], [1.0], [1.0005], [0.0]]
        self.assertEqual(tf.abschnitte(punkte), ([0, 1, 3], [1]))

    def test_umkehr_am_ersetzten_ende(self):
        # Letzter Punkt ersetzt den davor, und genau dort kehrt nichts um.
        punkte = [[0.0], [1.0], [0.5], [0.5004]]
        self.assertEqual(tf.abschnitte(punkte), ([0, 1, 3], [1]))

    def test_leer_und_einzeln(self):
        self.assertEqual(tf.abschnitte([]), ([], []))
        self.assertEqual(tf.abschnitte([[1]]), ([0], []))

    def test_umkehr_nach_halt(self):
        punkte = [[0], [1], [2], [1]]
        self.assertEqual(tf.abschnitte(punkte, [1]), ([0, 1, 2, 3], [2]))


class Anfrage(unittest.TestCase):

    def bauen(self, last, **extra):
        argumente = dict(gelenke=["a", "b"], drehend=[True, True],
                         params_datei="/tmp/p.yaml")
        argumente.update(extra)
        return tf.anfrage(last, **argumente)

    def test_grundfall(self):
        auftrag, befunde = self.bauen(nutzlast([(1, 0), (1, 1)]))
        self.assertIsNotNone(auftrag)
        self.assertEqual(stufen(befunde, sequenz.FEHLER), [])
        self.assertEqual(auftrag["wegpunkte"], [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
        self.assertEqual(auftrag["namen"], [tf.START_NAME, "P1", "P2"])
        self.assertEqual(auftrag["gelenke"], ["a", "b"])
        self.assertEqual(auftrag["velocity_scaling"], 0.5)
        self.assertEqual(auftrag["acceleration_scaling"], 0.4)
        self.assertEqual(auftrag["gruppe"], "manipulator")
        self.assertEqual(auftrag["params_datei"], "/tmp/p.yaml")
        self.assertEqual(auftrag["start"]["c_mimic"], 0.0)
        self.assertEqual(auftrag["path_tolerance"], tf.PATH_TOLERANCE_STANDARD)
        self.assertEqual(auftrag["resample_dt"], tf.RESAMPLE_DT_STANDARD)
        self.assertEqual(auftrag["min_angle_change"], tf.MIN_ANGLE_CHANGE)

    def test_reihenfolge_der_gelenke(self):
        auftrag, _ = self.bauen(nutzlast([(1, 2)]), gelenke=["b", "a"])
        self.assertEqual(auftrag["wegpunkte"][1], [2.0, 1.0])

    def test_parameter_uebernommen(self):
        auftrag, _ = self.bauen(nutzlast([(1, 0), (1, 1)]),
                                path_tolerance=0.02, resample_dt=0.005)
        self.assertEqual(auftrag["path_tolerance"], 0.02)
        self.assertEqual(auftrag["resample_dt"], 0.005)

    def test_ohne_params_datei(self):
        auftrag, befunde = self.bauen(nutzlast([(1, 0)]), params_datei=None)
        self.assertIsNone(auftrag)
        self.assertTrue(stufen(befunde, sequenz.FEHLER))

    def test_schlechte_parameter(self):
        auftrag, _ = self.bauen(nutzlast([(1, 0)]), path_tolerance=0.0)
        self.assertIsNone(auftrag)

    def test_leer(self):
        auftrag, befunde = self.bauen({"items": []})
        self.assertIsNone(auftrag)
        self.assertTrue(stufen(befunde, sequenz.FEHLER))

    def test_achsarten_passen_nicht(self):
        auftrag, _ = self.bauen(nutzlast([(1, 0)]), drehend=[True])
        self.assertIsNone(auftrag)

    def test_ohne_achsziel(self):
        last = nutzlast([(1, 0), (1, 1)])
        del last["items"][1]["goal"]
        auftrag, befunde = self.bauen(last)
        self.assertIsNone(auftrag)
        self.assertTrue(any("P2" in b.text for b in stufen(befunde, sequenz.FEHLER)))

    def test_achsziel_unvollstaendig(self):
        last = nutzlast([(1, 0)])
        del last["items"][0]["goal"]["b"]
        auftrag, befunde = self.bauen(last)
        self.assertIsNone(auftrag)
        self.assertTrue(any("b" in b.text for b in stufen(befunde, sequenz.FEHLER)))

    def test_start_unvollstaendig(self):
        last = nutzlast([(1, 0)])
        del last["items"][0]["start"]["b"]
        auftrag, _ = self.bauen(last)
        self.assertIsNone(auftrag)

    def test_wechselnder_override(self):
        last = nutzlast([(1, 0), (1, 1)])
        last["items"][1]["velocity_scaling"] = 0.6
        auftrag, befunde = self.bauen(last)
        self.assertIsNone(auftrag)
        self.assertTrue(any("Override" in b.text
                            for b in stufen(befunde, sequenz.FEHLER)))

    def test_wechselnde_beschleunigung(self):
        last = nutzlast([(1, 0), (1, 1)])
        last["items"][1]["acceleration_scaling"] = 0.9
        auftrag, _ = self.bauen(last)
        self.assertIsNone(auftrag)

    def test_lin_ist_warnung(self):
        auftrag, befunde = self.bauen(nutzlast([(1, 0), (1, 1)], fahrart="LIN"))
        self.assertIsNotNone(auftrag)
        self.assertTrue(any("LIN" in b.text for b in stufen(befunde, sequenz.WARNUNG)))

    def test_ptp_ohne_warnung(self):
        _, befunde = self.bauen(nutzlast([(1, 0), (1, 1)]))
        self.assertEqual(stufen(befunde, sequenz.WARNUNG), [])

    def test_genauhalt_bei_radius_null(self):
        auftrag, befunde = self.bauen(nutzlast([(1, 0), (1, 1), (0, 1)],
                                               radien=[0.0, 0.05, 0.0]))
        self.assertEqual(auftrag["abschnitte"], [[0, 1], [1, 3]])
        self.assertEqual(auftrag["umkehr"], [])
        hinweise = stufen(befunde, sequenz.HINWEIS)
        self.assertEqual(len(hinweise), 2)
        self.assertIn("Genauhalt", hinweise[0].text)
        self.assertIn("P1", hinweise[0].text)
        self.assertNotIn("P2", hinweise[0].text)
        self.assertIn("verschliffen", hinweise[1].text)
        self.assertIn("P2", hinweise[1].text)
        self.assertNotIn("P1", hinweise[1].text)

    def test_alle_verschliffen(self):
        auftrag, befunde = self.bauen(nutzlast([(1, 0), (1, 1)], radien=[0.05, 0.0]))
        self.assertEqual(auftrag["abschnitte"], [[0, 2]])
        hinweise = stufen(befunde, sequenz.HINWEIS)
        self.assertEqual(len(hinweise), 1)
        self.assertNotIn("Genauhalt", hinweise[0].text)

    def test_alle_genauhalt(self):
        auftrag, befunde = self.bauen(nutzlast([(1, 0), (1, 1), (0, 1)]))
        self.assertEqual(auftrag["abschnitte"], [[0, 1], [1, 2], [2, 3]])
        hinweise = stufen(befunde, sequenz.HINWEIS)
        self.assertEqual(len(hinweise), 1)
        self.assertIn("P1, P2", hinweise[0].text)

    def test_letzter_radius_zaehlt_nicht(self):
        # Der letzte Punkt ist ohnehin das Ende, kein Genauhalt im Innern.
        auftrag, befunde = self.bauen(nutzlast([(1, 0), (1, 1)], radien=[0.05, 0.0]))
        self.assertFalse(any("Genauhalt" in b.text for b in befunde))

    def test_umkehr_wird_grenze(self):
        # Hin und zurueck auf derselben Geraden im Gelenkraum.
        auftrag, befunde = self.bauen(nutzlast([(1, 0), (0.5, 0)],
                                               radien=[0.05, 0.0]))
        self.assertEqual(auftrag["abschnitte"], [[0, 1], [1, 2]])
        self.assertEqual(auftrag["umkehr"], [1])
        warnungen = stufen(befunde, sequenz.WARNUNG)
        self.assertEqual(len(warnungen), 1)
        self.assertIn("P1", warnungen[0].text)
        self.assertIn("0.05", warnungen[0].text)
        self.assertEqual(warnungen[0].stelle, 1)
        self.assertEqual(stufen(befunde, sequenz.HINWEIS), [])

    def test_umkehr_bei_genauhalt_still(self):
        auftrag, befunde = self.bauen(nutzlast([(1, 0), (0.5, 0)]))
        self.assertEqual(auftrag["abschnitte"], [[0, 1], [1, 2]])
        self.assertEqual(auftrag["umkehr"], [])
        self.assertEqual(stufen(befunde, sequenz.WARNUNG), [])

    def test_stand_im_auftrag(self):
        auftrag, _ = self.bauen(nutzlast([(1, 0)]))
        self.assertEqual(auftrag["stand"], tf.STAND)
        self.assertGreaterEqual(tf.STAND, 2)
        self.assertEqual(auftrag["abschnitte"], [[0, 1]])

    def test_ein_punkt_ohne_hinweis(self):
        _, befunde = self.bauen(nutzlast([(1, 0)]))
        self.assertEqual(stufen(befunde, sequenz.HINWEIS), [])

    def test_gemischte_achsen(self):
        _, befunde = self.bauen(nutzlast([(1, 0), (1, 1)]), drehend=[True, False])
        self.assertTrue(any("gemischt" in b.text
                            for b in stufen(befunde, sequenz.WARNUNG)))

    def test_reine_schubachsen_ohne_warnung(self):
        _, befunde = self.bauen(nutzlast([(1, 0), (1, 1)]), drehend=[False, False])
        self.assertEqual(stufen(befunde, sequenz.WARNUNG), [])

    def test_doppelter_punkt_gemeldet(self):
        auftrag, befunde = self.bauen(nutzlast([(1, 0), (1.0002, 0), (1, 1)],
                                               radien=[0.05, 0.05, 0.0]))
        self.assertIsNotNone(auftrag)
        warnungen = stufen(befunde, sequenz.WARNUNG)
        self.assertEqual(len(warnungen), 1)
        self.assertIn("P2", warnungen[0].text)
        self.assertEqual(warnungen[0].stelle, 2)
        # Verworfen wird nichts: der Punkt geht mit hinaus, MoveIt entscheidet.
        self.assertEqual(len(auftrag["wegpunkte"]), 4)

    def test_doppelter_punkt_je_abschnitt(self):
        # P2 liegt auf P1, P1 ist Genauhalt: P2 beginnt einen eigenen
        # Abschnitt und wird dort nicht mit P1 verglichen, sondern ist sein
        # Anfang -- TOTG filtert je Abschnitt.
        auftrag, befunde = self.bauen(nutzlast([(1, 0), (1.0002, 0), (1, 1)],
                                               radien=[0.0, 0.05, 0.0]))
        self.assertEqual(auftrag["abschnitte"], [[0, 1], [1, 3]])
        warnungen = stufen(befunde, sequenz.WARNUNG)
        self.assertEqual(len(warnungen), 1)
        self.assertIn("P2 liegt", warnungen[0].text)
        self.assertEqual(warnungen[0].stelle, 2)

    def test_start_auf_erstem_punkt(self):
        _, befunde = self.bauen(nutzlast([(0, 0), (1, 1)]))
        warnungen = stufen(befunde, sequenz.WARNUNG)
        self.assertEqual(len(warnungen), 1)
        self.assertIn("P1", warnungen[0].text)

    def test_json_rundreise(self):
        auftrag, _ = self.bauen(nutzlast([(0.1 + 0.2, 1 / 3), (1, 1)]))
        zurueck = json.loads(tf.auftrag_text(auftrag))
        self.assertEqual(zurueck, auftrag)
        self.assertEqual(zurueck["wegpunkte"][1][0], 0.1 + 0.2)


def antwort(**extra):
    daten = {"ok": True, "gelenke": ["a", "b"],
             "punkte": [[0.0, [0.0, 0.0], [0.0, 0.0]],
                        [0.5, [0.5, 0.1], [1.0, 0.2]],
                        [1.0, [1.0, 1.0], [0.0, 0.0]]],
             "dauer_s": 1.0, "rechenzeit_s": 0.02, "start_s": 3.5,
             "eingang": 3, "moveit_version": "2.12.3", "moveit_pfad": "/x"}
    daten.update(extra)
    return json.dumps(daten)


class Antwort(unittest.TestCase):

    def test_grundfall(self):
        ergebnis = tf.antwort_lesen(antwort(), ["a", "b"])
        self.assertEqual(ergebnis["code"], 1)
        self.assertEqual(len(ergebnis["points"]), 3)
        zeit, lagen, tempi = ergebnis["points"][1]
        self.assertEqual(zeit, 0.5)
        self.assertEqual(lagen, {"a": 0.5, "b": 0.1})
        self.assertEqual(tempi, {"a": 1.0, "b": 0.2})
        self.assertEqual(ergebnis["teilbahnen"], [3])
        mit = tf.antwort_lesen(antwort(teilbahnen=[2, 1]), ["a", "b"])
        self.assertEqual(mit["teilbahnen"], [2, 1])
        self.assertEqual(ergebnis["planning_time"], 0.02)
        self.assertEqual(ergebnis["totg"]["moveit_version"], "2.12.3")
        self.assertEqual(ergebnis["totg"]["dauer_s"], 1.0)
        self.assertEqual(ergebnis["totg"]["start_s"], 3.5)
        self.assertEqual(ergebnis["totg"]["eingang"], 3)

    def test_kein_json(self):
        with self.assertRaises(ValueError):
            tf.antwort_lesen("{", ["a", "b"])

    def test_kein_objekt(self):
        with self.assertRaises(ValueError):
            tf.antwort_lesen("[1]", ["a", "b"])

    def test_fehler_durchgereicht(self):
        with self.assertRaises(ValueError) as fall:
            tf.antwort_lesen(json.dumps({"ok": False, "fehler": "keine Grenze"}),
                             ["a", "b"])
        self.assertIn("keine Grenze", str(fall.exception))

    def test_andere_achsen(self):
        with self.assertRaises(ValueError):
            tf.antwort_lesen(antwort(), ["a", "c"])

    def test_andere_reihenfolge(self):
        with self.assertRaises(ValueError):
            tf.antwort_lesen(antwort(), ["b", "a"])

    def test_unvollstaendiger_punkt(self):
        with self.assertRaises(ValueError):
            tf.antwort_lesen(antwort(punkte=[[0.0, [0.0], [0.0, 0.0]],
                                             [1.0, [1.0, 1.0], [0.0, 0.0]]]),
                             ["a", "b"])

    def test_unvollstaendige_tempi(self):
        with self.assertRaises(ValueError):
            tf.antwort_lesen(antwort(punkte=[[0.0, [0.0, 0.0], [0.0]],
                                             [1.0, [1.0, 1.0], [0.0, 0.0]]]),
                             ["a", "b"])

    def test_zu_wenig_punkte(self):
        with self.assertRaises(ValueError):
            tf.antwort_lesen(antwort(punkte=[[0.0, [0.0, 0.0], [0.0, 0.0]]]),
                             ["a", "b"])

    def test_unendliche_zeit(self):
        with self.assertRaises(ValueError):
            tf.antwort_lesen(antwort(punkte=[[0.0, [0.0, 0.0], [0.0, 0.0]],
                                             [float("inf"), [1.0, 1.0], [0.0, 0.0]]]),
                             ["a", "b"])


class Endabweichung(unittest.TestCase):

    def test_gleich(self):
        auftrag = {"gelenke": ["a", "b"], "wegpunkte": [[0, 0], [1, 2]]}
        punkte = [(0.0, {"a": 0.0, "b": 0.0}, {}), (1.0, {"a": 1.0, "b": 2.0}, {})]
        self.assertEqual(tf.endabweichung(punkte, auftrag), ("a", 0.0))

    def test_abgewickelt(self):
        auftrag = {"gelenke": ["a", "b"], "wegpunkte": [[0, 0], [1, 2]]}
        punkte = [(1.0, {"a": 1.0, "b": 2.0 - 2 * math.pi}, {})]
        name, betrag = tf.endabweichung(punkte, auftrag)
        self.assertEqual(name, "b")
        self.assertAlmostEqual(betrag, 2 * math.pi)

    def test_groesste(self):
        auftrag = {"gelenke": ["a", "b"], "wegpunkte": [[1, 1]]}
        punkte = [(1.0, {"a": 1.3, "b": 0.8}, {})]
        name, betrag = tf.endabweichung(punkte, auftrag)
        self.assertEqual(name, "a")
        self.assertAlmostEqual(betrag, 0.3)

    def test_leer(self):
        self.assertEqual(tf.endabweichung([], {"gelenke": [], "wegpunkte": []}),
                         (None, 0.0))


class Eckwerte(unittest.TestCase):

    def test_genau_durch_die_ecke(self):
        zeiten = [0.0, 1.0, 2.0]
        orte = [(0, 0, 0), (1, 0, 0), (1, 1, 0)]
        werte = tf.eckwerte(zeiten, orte, [("P1", (1, 0, 0))])
        self.assertEqual(werte[0]["name"], "P1")
        self.assertAlmostEqual(werte[0]["abstand_m"], 0.0)
        self.assertAlmostEqual(werte[0]["zeit_s"], 1.0)

    def test_abstand_zur_strecke_nicht_zum_punkt(self):
        # Die Ecke liegt zwischen zwei Stuetzpunkten, 1 cm neben der Strecke.
        zeiten = [0.0, 2.0]
        orte = [(0, 0, 0), (1, 0, 0)]
        werte = tf.eckwerte(zeiten, orte, [("E", (0.5, 0.01, 0))])
        self.assertAlmostEqual(werte[0]["abstand_m"], 0.01)
        self.assertAlmostEqual(werte[0]["v_m_s"], 0.5)
        self.assertAlmostEqual(werte[0]["zeit_s"], 1.0)

    def test_hinter_dem_ende(self):
        zeiten = [0.0, 1.0]
        orte = [(0, 0, 0), (1, 0, 0)]
        werte = tf.eckwerte(zeiten, orte, [("E", (2, 0, 0))])
        self.assertAlmostEqual(werte[0]["abstand_m"], 1.0)
        self.assertAlmostEqual(werte[0]["zeit_s"], 1.0)

    def test_vor_dem_anfang(self):
        zeiten = [0.0, 1.0]
        orte = [(0, 0, 0), (1, 0, 0)]
        werte = tf.eckwerte(zeiten, orte, [("E", (-1, 0, 0))])
        self.assertAlmostEqual(werte[0]["abstand_m"], 1.0)
        self.assertAlmostEqual(werte[0]["zeit_s"], 0.0)

    def test_naechste_strecke_gewinnt(self):
        zeiten = [0.0, 1.0, 3.0]
        orte = [(0, 0, 0), (1, 0, 0), (1, 1, 0)]
        werte = tf.eckwerte(zeiten, orte, [("E", (1.01, 0.5, 0))])
        self.assertAlmostEqual(werte[0]["abstand_m"], 0.01)
        self.assertAlmostEqual(werte[0]["v_m_s"], 0.5)
        self.assertAlmostEqual(werte[0]["zeit_s"], 2.0)

    def test_stillstand(self):
        zeiten = [0.0, 1.0, 1.0]
        orte = [(0, 0, 0), (1, 0, 0), (1, 0, 0)]
        werte = tf.eckwerte(zeiten, orte, [("E", (1, 0, 0))])
        self.assertAlmostEqual(werte[0]["abstand_m"], 0.0)

    def test_mehrere_ecken(self):
        zeiten = [0.0, 1.0, 2.0]
        orte = [(0, 0, 0), (1, 0, 0), (1, 1, 0)]
        werte = tf.eckwerte(zeiten, orte, [("A", (1, 0, 0)), ("B", (0.2, 0.9, 0))])
        self.assertEqual([w["name"] for w in werte], ["A", "B"])
        self.assertAlmostEqual(werte[1]["abstand_m"], 0.8)
        self.assertAlmostEqual(werte[1]["zeit_s"], 1.9)

    def test_ungleiche_laengen(self):
        with self.assertRaises(ValueError):
            tf.eckwerte([0.0], [(0, 0, 0), (1, 0, 0)], [])

    def test_ein_punkt(self):
        werte = tf.eckwerte([0.0], [(0, 0, 0)], [("E", (0, 3, 4))])
        self.assertAlmostEqual(werte[0]["abstand_m"], 5.0)
        self.assertEqual(werte[0]["v_m_s"], 0.0)


class Zusatzzeilen(unittest.TestCase):

    def test_reihenfolge_und_inhalt(self):
        herkunft = {
            "verfahren": "TOTG", "override_v": 0.5, "path_tolerance": 0.1,
            "teilbahnen": "120", "punkte": [("P1", "PTP", 0.0)],
            "ecken": [{"name": "P1", "abstand_m": 0.0084, "v_m_s": 0.15,
                       "zeit_s": 1.25}],
            "hinweis": "x",
        }
        zeilen = tf.zusatzzeilen(herkunft)
        self.assertEqual(zeilen[0], ["#verfahren", "TOTG"])
        self.assertIn(["#override_v", "0.5"], zeilen)
        self.assertIn(["#path_tolerance", "0.1"], zeilen)
        self.assertIn(["#teilbahnen", "120"], zeilen)
        self.assertIn(["#punkt", "P1", "PTP", "0.0"], zeilen)
        self.assertIn(["#ecke_spalten", "name", "abstand_mm", "v_mm_s", "zeit_s"],
                      zeilen)
        self.assertIn(["#ecke", "P1", "8.4000", "150.000", "1.2500"], zeilen)
        self.assertIn(["#hinweis", "x"], zeilen)
        self.assertTrue(all(z[0].startswith("#") for z in zeilen))
        self.assertLess(zeilen.index(["#ecke_spalten", "name", "abstand_mm",
                                      "v_mm_s", "zeit_s"]),
                        zeilen.index(["#ecke", "P1", "8.4000", "150.000", "1.2500"]))

    def test_float_repr_genau(self):
        zeilen = tf.zusatzzeilen({"verfahren": "Pilz", "sampling_time": 0.1 + 0.2})
        self.assertIn(["#sampling_time", repr(0.1 + 0.2)], zeilen)

    def test_leer(self):
        self.assertEqual(tf.zusatzzeilen({}), [["#verfahren", ""]])

    def test_ohne_ecken_keine_kopfzeile(self):
        zeilen = tf.zusatzzeilen({"verfahren": "Pilz", "ecken": []})
        self.assertEqual(zeilen, [["#verfahren", "Pilz"]])

    def test_ruckig_und_endabweichung(self):
        zeilen = tf.zusatzzeilen({"verfahren": "Pilz", "ruckig": False,
                                  "endabweichung": 0.0})
        self.assertIn(["#ruckig", "False"], zeilen)
        self.assertIn(["#endabweichung", "0.0"], zeilen)


if __name__ == "__main__":
    unittest.main(verbosity=1)
