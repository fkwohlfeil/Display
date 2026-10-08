#!/usr/bin/env python3
"""Tests fuer intrinsic_lin_rechner.py. Ohne ROS, ohne MoveIt, ohne Wrapper.

    python3 test_intrinsic_lin_rechner.py

Der Wrapper ist ein kleines Python-Skript, das die Modi lin_kurve,
lin_neuabtastung und toppra nur formal beantwortet; die Kinematik ist die
Identitaet (Pose = Gelenklagen). Geprueft wird der Ablauf: welche
Abschnitte durch die IK-Kette gehen und welche Teilpfade TOPP-RA bekommt.
"""

import json
import math
import os
import stat
import tempfile
import unittest

import intrinsic_lin_rechner as R

GELENKE = ["j1", "j2", "j3"]

ERSATZ_WRAPPER = r'''#!/usr/bin/env python3
import json, sys
auftrag = json.load(open(sys.argv[1]))
modus = auftrag.get("modus", "toppra")
if modus == "lin_kurve":
    antwort = {"ok": True, "posen": auftrag["posen"],
               "kurvenparameter": list(range(len(auftrag["posen"]))),
               "kurve": {"art": "ersatz"}, "hinweise": []}
elif modus == "lin_neuabtastung":
    antwort = {"ok": True, "posen": auftrag["posen"],
               "behalten_indizes": list(range(len(auftrag["posen"]))),
               "gelenk_abtastschritt_rad": 0.05}
elif modus == "toppra_proben":
    # Proben = die Punkte selbst, dq/ds = Abstand zum naechsten (am Ende 0),
    # d2q/ds2 = 0.
    teile = []
    for teil in auftrag["teilpfade"]:
        qp = [[b - a for a, b in zip(p, n)] for p, n in zip(teil, teil[1:])]
        qp.append([0.0] * len(teil[0]))
        teile.append({"q": teil, "qp": qp,
                      "qpp": [[0.0] * len(p) for p in teil],
                      "pruefsumme": sum(sum(p) for p in teil)})
    antwort = {"ok": True, "teilpfade": teile}
else:
    punkte = [p for teil in auftrag["teilpfade"] for p in teil]
    # Mit Bahngrenzen faehrt der Ersatz mit halber Vorgabe auf j1.
    v = (0.5 * auftrag["tcp_v_max"] * auftrag["velocity_scaling"]
         if "tcp_v_max" in auftrag else 0.0)
    a = (0.5 * auftrag["tcp_a_max"] * auftrag["acceleration_scaling"]
         if "tcp_a_max" in auftrag else 0.0)
    def null(p, w):
        return [w] + [0.0] * (len(p) - 1)
    antwort = {"ok": True, "stand": 3, "gelenke": auftrag["gelenke"],
               "punkte": [[0.1 * i, p, null(p, v)]
                          for i, p in enumerate(punkte)],
               "beschleunigungen": [null(p, a) for p in punkte],
               "teilbahnen": [len(t) for t in auftrag["teilpfade"]],
               "abschnitte": [dict({"toppra_iterationen": 1},
                                   **({"tcp_massgeblich_proben": len(t) - 1}
                                      if "tcp_v_max" in auftrag else {}))
                              for t in auftrag["teilpfade"]]}
    if auftrag.get("roh_ausgeben"):
        # Roh 10 % unter der abgetasteten Beschleunigung.
        antwort["roh"] = [[[0.1 * i, p, null(p, v), null(p, 0.9 * a)]
                           for i, p in enumerate(t)]
                          for t in auftrag["teilpfade"]]
json.dump(antwort, open(sys.argv[2], "w"))
'''


class Kinematik:
    """Pose = Gelenklagen plus Einheitsquaternion. IK ist die Umkehrung."""

    def __init__(self):
        self.ik_aufrufe = 0
        self.ik_rechenzeit = 0.0

    def fk(self, lagen):
        return [float(w) for w in lagen] + [0.0, 0.0, 0.0, 1.0]

    def startwert(self, lagen):
        pass

    def ik(self, pose):
        self.ik_aufrufe += 1
        return [float(w) for w in pose[:3]]


class KeinMoveIt:
    def __init__(self, *args, **kwargs):
        raise AssertionError("MoveItKinematik darf nicht hochfahren")


def auftrag(**aenderung):
    daten = {
        "stand": 2, "gelenke": GELENKE, "start": dict.fromkeys(GELENKE, 0.0),
        "namen": ["Start", "P1", "P2", "P3"],
        "wegpunkte": [[0.0, 0.0, 0.0], [0.5, 0.0, 0.0], [0.5, 0.5, 0.0],
                      [0.5, 0.5, 0.5]],
        "abschnitte": [[0, 1], [1, 3]],
        "fahrarten": ["LIN", "PTP"],
        "ueberschleifen_rad": 0.2,
        "velocity_scaling": 0.5, "acceleration_scaling": 0.5,
        "resample_dt": 0.01, "rundung_translation_m": 0.02,
        "rundung_rotation_rad": 0.1,
        "grenzen": {"min_position": [-3.0] * 3, "max_position": [3.0] * 3,
                    "max_velocity": [2.0] * 3, "max_acceleration": [5.0] * 3},
    }
    daten.update(aenderung)
    return daten


class Rechne(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.ordner = tempfile.mkdtemp()
        cls.wrapper = os.path.join(cls.ordner, "wrapper")
        with open(cls.wrapper, "w") as datei:
            datei.write(ERSATZ_WRAPPER)
        os.chmod(cls.wrapper, stat.S_IRWXU)

    def setUp(self):
        self.moveit_alt = R.MoveItKinematik
        R.MoveItKinematik = KeinMoveIt

    def tearDown(self):
        R.MoveItKinematik = self.moveit_alt

    def rechne(self, daten, kinematik=None):
        daten = dict(daten, wrapper=self.wrapper)
        antwort = R.rechne(daten, self.ordner, kinematik=kinematik)
        ordner = antwort["arbeitsordner_wrapper"]
        with open(os.path.join(ordner, "toppra_auftrag.json")) as datei:
            toppra = json.load(datei)
        return antwort, toppra, sorted(os.listdir(ordner))

    def test_nur_ptp_ohne_moveit(self):
        daten = auftrag(abschnitte=[[0, 2], [2, 3]], fahrarten=["PTP", "PTP"])
        antwort, toppra, dateien = self.rechne(daten)
        self.assertEqual(toppra["teilpfade"],
                         [daten["wegpunkte"][0:3], daten["wegpunkte"][2:4]])
        self.assertEqual(toppra["namen"], ["Start -> P2", "P2 -> P3"])
        self.assertEqual(toppra["ueberschleifen_rad"], 0.2)
        # Kein lin_kurve, keine lin_neuabtastung.
        self.assertEqual(dateien, ["toppra_antwort.json",
                                   "toppra_auftrag.json"])
        self.assertEqual(antwort["fahrarten"], ["PTP", "PTP"])
        self.assertEqual([a["fahrart"] for a in antwort["lin"]],
                         ["PTP", "PTP"])
        self.assertEqual([a["pfadpunkte"] for a in antwort["lin"]], [3, 2])
        self.assertIsNone(antwort["ik_aufrufe"])

    def test_gemischt(self):
        kinematik = Kinematik()
        daten = auftrag(ueberschleifen_rad=0.15)
        antwort, toppra, dateien = self.rechne(daten, kinematik)
        # LIN: durch die IK-Kette (Identitaet), PTP: die Wegpunkte selbst.
        self.assertEqual(toppra["teilpfade"],
                         [daten["wegpunkte"][0:2], daten["wegpunkte"][1:4]])
        self.assertEqual(toppra["ueberschleifen_rad"], 0.15)
        self.assertIn("a0_lin_kurve_auftrag.json", dateien)
        self.assertFalse(any(d.startswith("a1_") for d in dateien), dateien)
        # Zwei IK-Ketten ueber je zwei Posen, nur im LIN-Abschnitt.
        self.assertEqual(kinematik.ik_aufrufe, 4)
        self.assertEqual(antwort["ik_aufrufe"], 4)
        self.assertEqual([a["fahrart"] for a in antwort["lin"]],
                         ["LIN", "PTP"])

    def test_stand_1_ist_lin(self):
        kinematik = Kinematik()
        daten = auftrag(stand=1)
        del daten["fahrarten"]
        del daten["ueberschleifen_rad"]
        antwort, toppra, _ = self.rechne(daten, kinematik)
        self.assertEqual(antwort["fahrarten"], ["LIN", "LIN"])
        self.assertEqual(toppra["ueberschleifen_rad"],
                         R.STANDARD["ueberschleifen_rad"])
        self.assertEqual(kinematik.ik_aufrufe, 4 + 6)

    def test_abgelehnt(self):
        for aenderung, wort in (({"stand": R.STAND + 1}, "Stand"),
                                ({"fahrarten": ["LIN"]}, "Fahrarten"),
                                ({"fahrarten": ["LIN", "CIRC"]}, "CIRC"),
                                ({"tcp_v_max": 0.0}, "tcp_v_max"),
                                ({"tcp_v_max": "schnell"}, "tcp_v_max"),
                                ({"tcp_a_max": -1.0}, "tcp_a_max")):
            with self.assertRaisesRegex(R.LinFehler, wort, msg=aenderung):
                self.rechne(auftrag(**aenderung), Kinematik())

    def test_bahngeschwindigkeit(self):
        kinematik = Kinematik()
        antwort, toppra, dateien = self.rechne(auftrag(tcp_v_max=0.4),
                                               kinematik)
        # Erst die Proben, dann TOPP-RA mit x, dx/ds, d2x/ds2 daran.
        self.assertIn("toppra_proben_auftrag.json", dateien)
        self.assertEqual(toppra["tcp_v_max"], 0.4)
        self.assertNotIn("tcp_a_max", toppra)
        self.assertFalse(toppra["roh_ausgeben"])
        # Identitaet als FK: x = q, dx/ds = Abstand zum naechsten Punkt,
        # d2x/ds2 = 0.
        self.assertEqual(len(toppra["tcp_xp"]), 2)
        for teil, x, xp, xpp in zip(toppra["teilpfade"], toppra["tcp_x"],
                                    toppra["tcp_xp"], toppra["tcp_xpp"]):
            erwartet = [[b - a for a, b in zip(p, n)]
                        for p, n in zip(teil, teil[1:])] + [[0.0] * 3]
            for ist, soll in zip(x, teil):
                self.assertEqual(ist, soll)
            for ist, soll in zip(xp, erwartet):
                for a, b in zip(ist, soll):
                    self.assertAlmostEqual(a, b, places=9)
            for ist in xpp:
                for a in ist:
                    self.assertAlmostEqual(a, 0.0, places=6)
        self.assertEqual(toppra["tcp_pruefsummen"],
                         [sum(sum(p) for p in t) for t in toppra["teilpfade"]])
        # Nachgemessen an der Bahn: halbe wirksame Vorgabe (0.4 * 0.5).
        pruefung = antwort["tcp_pruefung"]
        self.assertAlmostEqual(pruefung["v_max_m_s"], 0.1, places=9)
        self.assertAlmostEqual(pruefung["v_vorgabe_m_s"], 0.2)
        self.assertTrue(pruefung["v_eingehalten"])
        self.assertNotIn("a_eingehalten", pruefung)
        self.assertNotIn("a_roh_max_m_s2", pruefung)
        self.assertEqual(pruefung["massgeblich_proben"], 1 + 2)
        self.assertEqual(antwort["lin_vorgaben"]["tcp_v_max"], 0.4)
        self.assertIsNone(antwort["lin_vorgaben"]["tcp_a_max"])
        self.assertIn("tcp_fk", antwort["herkunft"])

    def test_bahnbeschleunigung(self):
        antwort, toppra, dateien = self.rechne(auftrag(tcp_a_max=2.0),
                                               Kinematik())
        self.assertIn("toppra_proben_auftrag.json", dateien)
        self.assertEqual(toppra["tcp_a_max"], 2.0)
        self.assertNotIn("tcp_v_max", toppra)
        # Fuer die Messung vor der Abtastung -- aber nicht in der Antwort,
        # wenn sie keiner verlangt hat.
        self.assertTrue(toppra["roh_ausgeben"])
        self.assertNotIn("roh", antwort)
        pruefung = antwort["tcp_pruefung"]
        # Halbe wirksame Vorgabe (2.0 * 0.5), roh 10 % darunter.
        self.assertAlmostEqual(pruefung["a_vorgabe_m_s2"], 1.0)
        self.assertAlmostEqual(pruefung["a_max_m_s2"], 0.5, places=6)
        self.assertAlmostEqual(pruefung["a_roh_max_m_s2"], 0.45, places=6)
        self.assertTrue(pruefung["a_eingehalten"])
        self.assertTrue(pruefung["a_roh_eingehalten"])
        self.assertEqual(pruefung["roh_punkte"], 2 + 3)
        self.assertNotIn("massgeblich_proben", pruefung)
        self.assertEqual(antwort["lin_vorgaben"]["tcp_a_max"], 2.0)

    def test_roh_bleibt_wenn_verlangt(self):
        antwort, _, _ = self.rechne(auftrag(tcp_a_max=2.0, roh_ausgeben=True),
                                    Kinematik())
        self.assertIn("roh", antwort)

    def test_ohne_bahngrenze_keine_proben(self):
        antwort, toppra, dateien = self.rechne(auftrag(), Kinematik())
        self.assertNotIn("toppra_proben_auftrag.json", dateien)
        self.assertNotIn("tcp_v_max", toppra)
        self.assertNotIn("tcp_xp", toppra)
        self.assertNotIn("tcp_pruefung", antwort)
        self.assertIsNone(antwort["lin_vorgaben"]["tcp_v_max"])

    def test_nur_ptp_mit_bahngrenze_braucht_fk(self):
        for grenze in ({"tcp_v_max": 0.4}, {"tcp_a_max": 2.0}):
            daten = auftrag(abschnitte=[[0, 3]], fahrarten=["PTP"], **grenze)
            # Ohne Kinematik faehrt MoveItKinematik hoch (hier: KeinMoveIt).
            with self.assertRaises(AssertionError):
                self.rechne(daten)
            kinematik = Kinematik()
            antwort, toppra, _ = self.rechne(daten, kinematik)
            self.assertEqual(kinematik.ik_aufrufe, 0)
            self.assertIn("tcp_xp", toppra)


class Bahngeschwindigkeit(unittest.TestCase):

    def test_tip_ableitung(self):
        k = Kinematik()
        self.assertAlmostEqual(
            R.tip_ableitung(k, [1.0, 2.0, 3.0], [3.0, 4.0, 0.0]), 5.0,
            places=9)
        self.assertEqual(R.tip_ableitung(k, [1.0, 2.0, 3.0], [0, 0, 0]), 0.0)

    def test_tip_ableitung_mit_mimic(self):
        # Wie MoveIt: j2 folgt j1 mit Faktor -1 und geht in die Lage ein.
        class Gekoppelt(Kinematik):
            def fk(self, lagen):
                j1 = lagen[0]
                return [j1, -j1, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.assertAlmostEqual(
            R.tip_ableitung(Gekoppelt(), [0.3], [1.0]), math.sqrt(2.0),
            places=9)

    def test_zweite_ableitung_auf_dem_kreis(self):
        # x = (cos q, sin q): entlang q(h) = q0 + 2h + 3h^2/2 ist
        # |dx/dh| = 2 und d2x/dh2 = 3 tangential - 4 normal, Betrag 5.
        class Kreis(Kinematik):
            def fk(self, lagen):
                return [math.cos(lagen[0]), math.sin(lagen[0]), 0.0,
                        0.0, 0.0, 0.0, 1.0]
        x, erste, zweite = R.tip_ableitungen(Kreis(), [0.7], [2.0], [3.0])
        self.assertAlmostEqual(x[0], math.cos(0.7), places=12)
        self.assertAlmostEqual(math.hypot(*erste), 2.0, places=7)
        self.assertAlmostEqual(math.hypot(*zweite), 5.0, places=5)
        # Nur d2 (Anfahren aus dem Stillstand): d2x = 3 tangential.
        _x, erste, zweite = R.tip_ableitungen(Kreis(), [0.7], [0.0], [3.0])
        self.assertAlmostEqual(math.hypot(*erste), 0.0, places=7)
        self.assertAlmostEqual(math.hypot(*zweite), 3.0, places=5)
        # Ruht alles: beide null, ohne Teilen durch null.
        _x, erste, zweite = R.tip_ableitungen(Kreis(), [0.7], [0.0], [0.0])
        self.assertEqual((erste, zweite), ([0.0] * 3, [0.0] * 3))

    def test_pruefung_zaehlt_darueber(self):
        k = Kinematik()
        punkte = [[0.0, [0, 0, 0], [0.1, 0, 0]],
                  [0.1, [0, 0, 0], [0.25, 0, 0]],
                  [0.2, [0, 0, 0], [0.2001, 0, 0]]]
        beschleunigungen = [[0, 0, 0], [0, 3.0, 0], [0, 0, 0]]
        roh = [[[0.0, [0, 0, 0], [0, 0, 0], [0, 0, 1.9]]]]
        p = R.tcp_pruefung(k, punkte, beschleunigungen, v_max=0.2,
                           a_max=2.0, roh=roh)
        self.assertAlmostEqual(p["v_max_m_s"], 0.25, places=9)
        self.assertEqual(p["v_bei_s"], 0.1)
        # 0.2001 liegt innerhalb der Toleranz, 0.25 nicht.
        self.assertEqual(p["v_darueber"], 1)
        self.assertFalse(p["v_eingehalten"])
        self.assertAlmostEqual(p["v_faktor_max"], 1.25, places=9)
        self.assertAlmostEqual(p["a_max_m_s2"], 3.0, places=5)
        self.assertFalse(p["a_eingehalten"])
        self.assertAlmostEqual(p["a_roh_max_m_s2"], 1.9, places=5)
        self.assertTrue(p["a_roh_eingehalten"])
        hinweise = R.tcp_hinweise(p)
        self.assertEqual(len(hinweise), 2)
        self.assertIn("Tip-Geschwindigkeit bis 0.2500", hinweise[0])
        self.assertIn("kubische Interpolation", hinweise[1])
        # Ohne Vorgabe nur gemessen, nichts bewertet.
        p = R.tcp_pruefung(k, punkte, beschleunigungen)
        self.assertNotIn("v_eingehalten", p)
        self.assertEqual(R.tcp_hinweise(p), [])


class Springt(Kinematik):
    """IK springt ab der zweiten Pose auf j3 um pi (anderer Ast).

    j3 wirkt hier nur bis auf pi auf die Pose -- der Sprung trifft die Pose
    also, wie ein echter zweiter Ast.
    """

    gelenke = GELENKE

    def __init__(self, rang=None):
        super().__init__()
        self._rang = rang

    def fk(self, lagen):
        q = [float(w) for w in lagen]
        q[2] = math.remainder(q[2], math.pi)
        return super().fk(q)

    def ik(self, pose):
        q = super().ik(pose)
        if self.ik_aufrufe > 1:
            q[2] += 3.14159
        return q

    def rang(self, lagen):
        if self._rang is None:
            raise RuntimeError("keine Jacobi-Matrix")
        return self._rang


class Sprung(unittest.TestCase):

    POSEN = [[0.0, 0.0, 0.0, 0, 0, 0, 1], [0.01, 0.0, 0.0, 0, 0, 0, 1]]

    def meldung(self, kinematik):
        with self.assertRaises(R.LinFehler) as fall:
            R.ik_kette(kinematik, self.POSEN, [0.0, 0.0, 0.0], "Start -> P2",
                       "grob")
        return str(fall.exception)

    def test_singulaer(self):
        text = self.meldung(Springt(rang=(5, 6)))
        self.assertIn("Achssprung 3.1416 rad zwischen Kurvenpose 0 und 1",
                      text)
        # Nur die springende Achse, nicht j1 (0.01).
        self.assertIn("an j3 +3.1416.", text)
        self.assertNotIn("j1", text)
        self.assertIn("singulaer (Jacobi-Rang 5 von 6)", text)
        self.assertIn("mit PTP herausfahren", text)

    def test_nicht_singulaer(self):
        text = self.meldung(Springt(rang=(6, 6)))
        self.assertIn("nicht singulaer (Jacobi-Rang 6 von 6)", text)

    def test_ohne_rang(self):
        text = self.meldung(Springt())
        self.assertTrue(text.endswith("Naehe zu Singularitaet oder "
                                      "Gelenkgrenze."), text)


class Ungenau(Kinematik):
    """IK meldet Erfolg, liegt aber ab der zweiten Pose daneben (wie KDL
    nahe einer Singularitaet)."""

    def __init__(self, daneben_m, daneben_rad=0.0, werte=None):
        super().__init__()
        self.daneben_m = daneben_m
        self.daneben_rad = daneben_rad
        self.werte = werte

    def fk(self, lagen):
        pose = super().fk(lagen)
        if self.ik_aufrufe > 1:
            pose[0] += self.daneben_m
            halb = self.daneben_rad / 2.0
            pose[3:] = [math.sin(halb), 0.0, 0.0, math.cos(halb)]
        return pose

    def singulaerwerte(self, lagen):
        if self.werte is None:
            raise RuntimeError("keine Jacobi-Matrix")
        return self.werte


class Posenfehler(unittest.TestCase):

    POSEN = [[0.0, 0.0, 0.0, 0, 0, 0, 1], [0.01, 0.0, 0.0, 0, 0, 0, 1],
             [0.02, 0.0, 0.0, 0, 0, 0, 1]]

    def kette(self, kinematik, **kwargs):
        return R.ik_kette(kinematik, self.POSEN, [0.0, 0.0, 0.0],
                          "P1 -> P3", "dicht", **kwargs)

    def test_pose_fehler(self):
        self.assertEqual(R.pose_fehler([1, 2, 3, 0, 0, 0, 1],
                                       [1, 2, 3, 0, 0, 0, -1]), (0.0, 0.0))
        lage, winkel = R.pose_fehler(
            [0.003, 0.004, 0, 0, 0, math.sin(0.05), math.cos(0.05)],
            [0, 0, 0, 0, 0, 0, 1])
        self.assertAlmostEqual(lage, 0.005)
        self.assertAlmostEqual(winkel, 0.1)

    def test_innerhalb_gemeldet(self):
        _, _, fehler = self.kette(Ungenau(0.0004, 0.002))
        self.assertAlmostEqual(fehler[0], 0.0004)
        self.assertAlmostEqual(fehler[1], 0.002)

    def test_lage_daneben(self):
        with self.assertRaises(R.LinFehler) as fall:
            self.kette(Ungenau(0.035, werte=[3.87, 0.9, 0.0225]))
        text = str(fall.exception)
        self.assertIn("P1 -> P3 (dicht): MoveIt-IK meldet Erfolg an "
                      "Kurvenpose 1 von 3", text)
        self.assertIn("35.000 mm / 0.00000 rad daneben (erlaubt 1 mm / "
                      "0.01 rad)", text)
        self.assertIn("Kleinster Singulaerwert der Jacobi-Matrix dort "
                      "0.0225", text)
        self.assertIn("mit PTP fahren", text)

    def test_winkel_daneben_und_toleranz(self):
        with self.assertRaisesRegex(R.LinFehler, "0.05000 rad daneben"):
            self.kette(Ungenau(0.0, 0.05))
        # Mit groesserer Toleranz geht es durch.
        _, _, fehler = self.kette(Ungenau(0.0, 0.05), toleranz_rad=0.1)
        self.assertAlmostEqual(fehler[1], 0.05)

    def test_ohne_singulaerwerte(self):
        with self.assertRaisesRegex(R.LinFehler, "Naehe zu Singularitaet"):
            self.kette(Ungenau(0.01))

    def ziel_meldung(self, kinematik, **aenderung):
        ordner = tempfile.mkdtemp()
        wrapper = os.path.join(ordner, "wrapper")
        with open(wrapper, "w") as datei:
            datei.write(ERSATZ_WRAPPER)
        os.chmod(wrapper, stat.S_IRWXU)
        daten = dict(auftrag(abschnitte=[[0, 3]], fahrarten=["LIN"]),
                     wrapper=wrapper, **aenderung)
        with self.assertRaisesRegex(R.LinFehler, "neben dem Zielwegpunkt") \
                as fall:
            R.rechne(daten, ordner, kinematik=kinematik)
        return str(fall.exception)

    def test_ziel_verfehlt_ungenau(self):
        # Die IK liegt ab der zweiten Pose 0.9 mm in x und y daneben: unter
        # der IK-Toleranz von 2 mm, am Ziel aber 1.27e-3 rad (Norm).
        class Drift(Kinematik):
            def ik(self, pose):
                q = super().ik(pose)
                if self.ik_aufrufe > 1:
                    q[0] += 0.0009
                    q[1] += 0.0009
                return q
        text = self.ziel_meldung(Drift(), ik_toleranz_m=0.002)
        self.assertIn("Endpose liegt 1.273 mm", text)
        self.assertIn("kein anderer Ast, sondern eine ungenaue IK", text)

    def test_ziel_verfehlt_anderer_ast(self):
        # j3 wirkt nicht auf die Pose: die IK waehlt einen anderen Wert bei
        # gleicher Pose -- ein anderer Ast.
        class Ast(Kinematik):
            def fk(self, lagen):
                return super().fk([lagen[0], lagen[1], 0.0])

            def ik(self, pose):
                q = super().ik(pose)
                # Am Start (x = y = 0) derselbe Ast wie der Startwegpunkt.
                q[2] = 0.0 if abs(pose[0]) + abs(pose[1]) < 1e-12 else 0.01
                return q
        text = self.ziel_meldung(Ast())
        self.assertIn("Die Pose stimmt, die Achsen nicht: anderer IK-Ast",
                      text)

    def test_toleranz_aus_auftrag(self):
        daten = auftrag(abschnitte=[[0, 3]], fahrarten=["LIN"],
                        ik_toleranz_m=0.05)
        ordner = tempfile.mkdtemp()
        wrapper = os.path.join(ordner, "wrapper")
        with open(wrapper, "w") as datei:
            datei.write(ERSATZ_WRAPPER)
        os.chmod(wrapper, stat.S_IRWXU)
        antwort = R.rechne(dict(daten, wrapper=wrapper), ordner,
                           kinematik=Ungenau(0.02))
        self.assertAlmostEqual(antwort["lin"][0]["ik_posenfehler_max_m"],
                               0.02)
        self.assertEqual(antwort["lin_vorgaben"]["ik_toleranz_m"], 0.05)
        with self.assertRaisesRegex(R.LinFehler, "20.000 mm"):
            R.rechne(dict(daten, wrapper=wrapper, ik_toleranz_m=0.01),
                     ordner, kinematik=Ungenau(0.02))


class AnderesJ3(Kinematik):
    """j3 wirkt nicht auf die Pose. Abseits des Ursprungs waehlt die IK
    j3 = 0.01 -- gleiche Pose, anderer Ast als die Achsziele mit j3 = 0."""

    def fk(self, lagen):
        return super().fk([lagen[0], lagen[1], 0.0])

    def ik(self, pose):
        q = super().ik(pose)
        q[2] = 0.0 if abs(pose[0]) + abs(pose[1]) < 1e-12 else 0.01
        return q


class AstFolgen(unittest.TestCase):
    """Stand 5: mit ast_folgen ersetzt die Endstellung der LIN-Kette den
    Zielwegpunkt, und die folgenden Abschnitte beginnen dort."""

    def rechne(self, **aenderung):
        ordner = tempfile.mkdtemp()
        wrapper = os.path.join(ordner, "wrapper")
        with open(wrapper, "w") as datei:
            datei.write(ERSATZ_WRAPPER)
        os.chmod(wrapper, stat.S_IRWXU)
        daten = dict(auftrag(stand=5, abschnitte=[[0, 1], [1, 2], [2, 3]],
                             fahrarten=["LIN", "LIN", "PTP"]),
                     wrapper=wrapper, **aenderung)
        antwort = R.rechne(daten, ordner, kinematik=AnderesJ3())
        with open(os.path.join(antwort["arbeitsordner_wrapper"],
                               "toppra_auftrag.json")) as datei:
            toppra = json.load(datei)
        return antwort, toppra

    def test_ohne_bricht_ab(self):
        with self.assertRaisesRegex(R.LinFehler, "anderer IK-Ast"):
            self.rechne()

    def test_folgt_dem_ast(self):
        antwort, toppra = self.rechne(ast_folgen=True)
        pfade = toppra["teilpfade"]
        self.assertEqual(pfade[0][-1], [0.5, 0.0, 0.01])
        # Der zweite LIN-Abschnitt beginnt am erreichten Ast, nicht am
        # Achsziel P1, und endet wieder auf ihm.
        self.assertEqual(pfade[1][0], [0.5, 0.0, 0.01])
        self.assertEqual(pfade[1][-1], [0.5, 0.5, 0.01])
        # PTP faehrt von dort im Gelenkraum zum eigenen Achsziel.
        self.assertEqual(pfade[2], [[0.5, 0.5, 0.01], [0.5, 0.5, 0.5]])
        self.assertTrue(antwort["ast_folgen"])
        self.assertEqual([a["punkt"] for a in antwort["ast_gewechselt"]],
                         ["P1", "P2"])
        self.assertEqual(antwort["ast_gewechselt"][0]["achsziel"],
                         [0.5, 0.0, 0.0])
        self.assertAlmostEqual(antwort["ast_gewechselt"][0]["abweichung_rad"],
                               0.01)
        self.assertEqual(antwort["lin"][0]["ziel_abweichung_rad"], 0.0)
        self.assertTrue(any("anderen IK-Ast" in h
                            for h in antwort["hinweise"]))

    def test_pose_daneben_bricht_trotzdem_ab(self):
        class Drift(Kinematik):
            def ik(self, pose):
                q = super().ik(pose)
                if self.ik_aufrufe > 1:
                    q[0] += 0.0009
                    q[1] += 0.0009
                return q
        ordner = tempfile.mkdtemp()
        wrapper = os.path.join(ordner, "wrapper")
        with open(wrapper, "w") as datei:
            datei.write(ERSATZ_WRAPPER)
        os.chmod(wrapper, stat.S_IRWXU)
        daten = dict(auftrag(stand=5, abschnitte=[[0, 3]], fahrarten=["LIN"]),
                     wrapper=wrapper, ast_folgen=True, ik_toleranz_m=0.002)
        with self.assertRaisesRegex(R.LinFehler, "ungenaue IK"):
            R.rechne(daten, ordner, kinematik=Drift())


class Verbinden(unittest.TestCase):

    NAMEN = ["Start", "P1", "P2", "P3"]
    STUECKE = [[0, 1], [1, 2], [2, 3]]
    PFADE = [[[0.0], [0.5]], [[0.5004], [0.7], [1.0]], [[1.0], [2.0]]]
    TEILNAMEN = ["Start -> P1", "P1 -> P2", "P2 -> P3"]

    def test_ohne(self):
        pfade, namen, verbunden = R.verbinden(
            self.STUECKE, self.PFADE, self.TEILNAMEN, [], self.NAMEN)
        self.assertEqual(pfade, self.PFADE)
        self.assertEqual(namen, self.TEILNAMEN)
        self.assertEqual(verbunden, [])

    def test_einer(self):
        pfade, namen, verbunden = R.verbinden(
            self.STUECKE, self.PFADE, self.TEILNAMEN, [1], self.NAMEN)
        # Der erste Punkt des hinteren Abschnitts faellt weg.
        self.assertEqual(pfade, [[[0.0], [0.5], [0.7], [1.0]], [[1.0], [2.0]]])
        self.assertEqual(namen, ["Start -> P1 -> P2", "P2 -> P3"])
        self.assertEqual(len(verbunden), 1)
        self.assertEqual(verbunden[0]["punkt"], "P1")
        self.assertAlmostEqual(verbunden[0]["sprung_rad"], 0.0004)

    def test_alle(self):
        pfade, namen, verbunden = R.verbinden(
            self.STUECKE, self.PFADE, self.TEILNAMEN, [1, 2], self.NAMEN)
        self.assertEqual(pfade, [[[0.0], [0.5], [0.7], [1.0], [2.0]]])
        self.assertEqual(namen, ["Start -> P1 -> P2 -> P3"])
        self.assertEqual([v["punkt"] for v in verbunden], ["P1", "P2"])

    def test_ungueltig(self):
        for falsch in ([0], [3], [5]):
            with self.assertRaisesRegex(R.LinFehler, "keine innere"):
                R.verbinden(self.STUECKE, self.PFADE, self.TEILNAMEN, falsch,
                            self.NAMEN)

    def test_rechne_verschleift_lin_ptp(self):
        ordner = tempfile.mkdtemp()
        wrapper = os.path.join(ordner, "wrapper")
        with open(wrapper, "w") as datei:
            datei.write(ERSATZ_WRAPPER)
        os.chmod(wrapper, stat.S_IRWXU)
        daten = dict(auftrag(stand=4, verschleifen=[1]), wrapper=wrapper)
        antwort = R.rechne(daten, ordner, kinematik=Kinematik())
        with open(os.path.join(antwort["arbeitsordner_wrapper"],
                               "toppra_auftrag.json")) as datei:
            toppra = json.load(datei)
        # LIN Start -> P1 und PTP P1 -> P3 als ein Teilpfad, ohne Halt.
        self.assertEqual(toppra["teilpfade"], [daten["wegpunkte"]])
        self.assertEqual(toppra["namen"], ["Start -> P1 -> P3"])
        self.assertEqual(antwort["verschliffen"],
                         [{"punkt": "P1", "sprung_rad": 0.0}])
        # Die Abschnittsinfo bleibt je Fahrart.
        self.assertEqual([a["fahrart"] for a in antwort["lin"]],
                         ["LIN", "PTP"])


class ZustandWieMoveIt:
    """Wie RobotState::setVariablePositions(std::map): Eintraege nach Namen
    sortiert, jedes Leitgelenk zieht sofort seine Mimic-Gelenke nach, ein
    Mimic-Eintrag selbst ueberschreibt nur sich."""

    MIMIC = {"joint6": ("joint5", -1.0)}

    def __init__(self):
        self.werte = {}

    @property
    def joint_positions(self):
        return dict(self.werte)

    @joint_positions.setter
    def joint_positions(self, werte):
        for name in sorted(werte):
            self.werte[name] = float(werte[name])
            for mimic, (leit, faktor) in self.MIMIC.items():
                if leit == name:
                    self.werte[mimic] = faktor * self.werte[name]

    def update(self):
        pass


class Mimic(unittest.TestCase):
    """Stand 3: _setzen darf die Kopplung nicht mit dem Startwert
    ueberschreiben (joint6 kommt alphabetisch nach joint5)."""

    def kinematik(self):
        k = R.MoveItKinematik.__new__(R.MoveItKinematik)
        k.gelenke = ["joint1", "joint5", "joint7"]
        start = {"joint1": 0.0, "joint5": 0.0, "joint6": 0.0, "joint7": 0.0}
        # Wie in __init__ seit Stand 3.
        k.abgeleitet = {"joint6"}
        k.grundstellung = {n: w for n, w in start.items()
                           if n not in k.abgeleitet}
        k.zustand = ZustandWieMoveIt()
        return k

    def test_kopplung_bleibt(self):
        k = self.kinematik()
        k._setzen([0.1, 0.2648, -0.3])
        self.assertAlmostEqual(k.zustand.werte["joint6"], -0.2648)
        self.assertEqual(k._lagen(), [0.1, 0.2648, -0.3])

    def test_ohne_ausblenden_waere_es_falsch(self):
        # Gegenprobe: mit joint6 in der Grundstellung tritt der Fehler auf.
        k = self.kinematik()
        k.grundstellung["joint6"] = 0.0
        k._setzen([0.1, 0.2648, -0.3])
        self.assertEqual(k.zustand.werte["joint6"], 0.0)

    def test_stand(self):
        self.assertEqual(R.STAND, 6)


if __name__ == "__main__":
    unittest.main(verbosity=1)
