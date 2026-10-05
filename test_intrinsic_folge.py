#!/usr/bin/env python3
"""Tests fuer intrinsic_folge.py. Ohne ROS, ohne Tk, ohne Bildschirm.

    python3 test_intrinsic_folge.py

Braucht sequenz.py und totg_folge.py daneben. Die Achsgrenzen kommen aus
einem Stub, der Wrapper ist eine leere ausfuehrbare Datei -- geprueft wird
der Auftrag, nicht die Rechnung.
"""

import json
import math
import os
import stat
import tempfile
import unittest

import intrinsic_folge as I
import sequenz

GELENKE = ["j1", "j2", "j3"]
GRENZEN = {"min_position": [-3.0] * 3, "max_position": [3.0] * 3,
           "max_velocity": [2.0] * 3, "max_acceleration": [5.0] * 3}


def posten(name, ziel, radius=0.0, fahrart="LIN", v=0.5, a=0.5):
    return {"group": "arm", "planner_id": fahrart, "velocity_scaling": v,
            "acceleration_scaling": a, "blend_radius": radius, "name": name,
            "goal": dict(zip(GELENKE, ziel))}


def nutzlast(*eintraege, start=(0.0, 0.0, 0.0)):
    eintraege = [dict(e) for e in eintraege]
    if eintraege:
        eintraege[0]["start"] = dict(zip(GELENKE, start))
    return {"group": "arm", "items": eintraege}


class Aufrufe:
    """Stub fuer grenzen_lesen. Merkt sich, ob er gerufen wurde."""

    def __init__(self, hinweise=(), fehler=None):
        self.anzahl = 0
        self.hinweise = list(hinweise)
        self.fehler = fehler

    def __call__(self, params_datei, gelenke):
        self.anzahl += 1
        if self.fehler:
            raise self.fehler
        assert list(gelenke) == GELENKE
        return dict(GRENZEN), list(self.hinweise)


class Basis(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.ordner = tempfile.mkdtemp()
        cls.wrapper = os.path.join(cls.ordner, "wrapper")
        with open(cls.wrapper, "w") as datei:
            datei.write("#!/bin/sh\n")
        os.chmod(cls.wrapper, stat.S_IRWXU)
        cls.params = os.path.join(cls.ordner, "move_group.yaml")
        with open(cls.params, "w") as datei:
            datei.write("x: {}\n")

    def frage(self, last, rot=0.1, dt=0.01, tip="tool0", params=None,
              wrapper=None, grenzen=None):
        grenzen = grenzen or Aufrufe()
        auftrag, befunde = I.anfrage(
            last, GELENKE, self.params if params is None else params, tip,
            rot, dt, wrapper=self.wrapper if wrapper is None else wrapper,
            grenzen_lesen=grenzen)
        return auftrag, befunde, grenzen

    def stufen(self, befunde, stufe):
        return [b.text for b in befunde if b.stufe == stufe]


class Parameter(Basis):

    def test_gut(self):
        self.assertEqual(I.pruefe_parameter(0.1, 0.01), ())
        self.assertEqual(I.pruefe_parameter("0.1", "0.01"), ())

    def test_grenzen_rot(self):
        self.assertEqual(I.pruefe_parameter(I.RUNDUNG_ROT_MIN, 0.01), ())
        self.assertEqual(I.pruefe_parameter(I.RUNDUNG_ROT_MAX, 0.01), ())
        for falsch in (0.0, -0.1, I.RUNDUNG_ROT_MAX * 1.001, "x", None,
                       I.RUNDUNG_ROT_MIN * 0.5):
            befunde = I.pruefe_parameter(falsch, 0.01)
            self.assertEqual(len(befunde), 1, falsch)
            self.assertEqual(befunde[0].stufe, sequenz.FEHLER)
            self.assertIn("rundung_rot", befunde[0].text)

    def test_resample_dt(self):
        for falsch in (0.0, 2.0, "y"):
            befunde = I.pruefe_parameter(0.1, falsch)
            self.assertEqual(len(befunde), 1, falsch)
            self.assertIn("resample_dt", befunde[0].text)


class Anfrage(Basis):

    def test_verrundet_ein_abschnitt(self):
        last = nutzlast(posten("P1", (0.5, 0, 0), 0.02),
                        posten("P2", (0.5, 0.5, 0), 0.02),
                        posten("P3", (0.5, 0.5, 0.5), 0.0))
        auftrag, befunde, grenzen = self.frage(last, rot=0.2, dt=0.004)
        self.assertEqual(self.stufen(befunde, sequenz.FEHLER), [])
        self.assertEqual(self.stufen(befunde, sequenz.WARNUNG), [])
        self.assertEqual(grenzen.anzahl, 1)
        self.assertEqual(auftrag["stand"], I.AUFTRAG_STAND)
        self.assertEqual(auftrag["wrapper"], self.wrapper)
        self.assertEqual(auftrag["params_datei"], self.params)
        self.assertEqual(auftrag["gruppe"], "arm")
        self.assertEqual(auftrag["tip"], "tool0")
        self.assertEqual(auftrag["gelenke"], GELENKE)
        self.assertEqual(auftrag["namen"], ["Start", "P1", "P2", "P3"])
        self.assertEqual(auftrag["wegpunkte"],
                         [[0.0, 0.0, 0.0], [0.5, 0.0, 0.0], [0.5, 0.5, 0.0],
                          [0.5, 0.5, 0.5]])
        self.assertEqual(auftrag["abschnitte"], [[0, 3]])
        self.assertEqual(auftrag["umkehr"], [])
        self.assertEqual(auftrag["rundung_translation_m"], 0.02)
        self.assertEqual(auftrag["rundung_rotation_rad"], 0.2)
        self.assertEqual(auftrag["resample_dt"], 0.004)
        self.assertEqual(auftrag["velocity_scaling"], 0.5)
        self.assertEqual(auftrag["acceleration_scaling"], 0.5)
        self.assertEqual(auftrag["grenzen"], GRENZEN)
        self.assertEqual(auftrag["start"], {"j1": 0.0, "j2": 0.0, "j3": 0.0})
        self.assertTrue(any("Verrundet mit 0.02 m / 0.2 rad: P1, P2" in t
                            for t in self.stufen(befunde, sequenz.HINWEIS)))
        # Der Auftrag ist JSON und kommt unveraendert zurueck.
        self.assertEqual(json.loads(I.auftrag_text(auftrag)), auftrag)

    def test_radius_am_letzten_punkt_zaehlt_nicht(self):
        last = nutzlast(posten("P1", (0.5, 0, 0), 0.02),
                        posten("P2", (0.5, 0.5, 0), 0.07))
        auftrag, befunde, _ = self.frage(last)
        self.assertIsNotNone(auftrag, befunde)
        self.assertEqual(auftrag["rundung_translation_m"], 0.02)

    def test_verschiedene_radien_sind_fehler(self):
        last = nutzlast(posten("P1", (0.5, 0, 0), 0.02),
                        posten("P2", (0.5, 0.5, 0), 0.03),
                        posten("P3", (0.5, 0.5, 0.5), 0.0))
        auftrag, befunde, grenzen = self.frage(last)
        self.assertIsNone(auftrag)
        fehler = self.stufen(befunde, sequenz.FEHLER)
        self.assertEqual(len(fehler), 1)
        self.assertIn("0.02, 0.03", fehler[0])
        self.assertEqual(grenzen.anzahl, 0)

    def test_genauhalt_zerlegt(self):
        last = nutzlast(posten("P1", (0.5, 0, 0), 0.0),
                        posten("P2", (0.5, 0.5, 0), 0.02),
                        posten("P3", (0.5, 0.5, 0.5), 0.0))
        auftrag, befunde, _ = self.frage(last)
        self.assertEqual(auftrag["abschnitte"], [[0, 1], [1, 3]])
        self.assertEqual(auftrag["rundung_translation_m"], 0.02)
        hinweise = self.stufen(befunde, sequenz.HINWEIS)
        self.assertTrue(any(t.startswith("Genauhalt (Radius 0) an P1.")
                            for t in hinweise), hinweise)
        self.assertTrue(any(": P2." in t for t in hinweise), hinweise)

    def test_ohne_radius_minimum(self):
        # Minimum der Eckenverrundung im Intrinsic Motion Planner (1 mm).
        self.assertEqual(I.RUNDUNG_TRANS_OHNE_RADIUS, 0.001)
        last = nutzlast(posten("P1", (0.5, 0, 0)), posten("P2", (0.5, 0.5, 0)))
        auftrag, befunde, _ = self.frage(last)
        self.assertEqual(auftrag["rundung_translation_m"],
                         I.RUNDUNG_TRANS_OHNE_RADIUS)
        self.assertEqual(auftrag["abschnitte"], [[0, 1], [1, 2]])
        hinweise = self.stufen(befunde, sequenz.HINWEIS)
        self.assertTrue(any(t.startswith("Genauhalt (Radius 0) an P1.")
                            for t in hinweise), hinweise)
        self.assertTrue(any(t.startswith("Kein Radius") for t in hinweise))
        # Ein einzelner Punkt: kein Halt, kein Verrunden -- nur der Hinweis.
        auftrag, befunde, _ = self.frage(nutzlast(posten("P1", (0.5, 0, 0))))
        self.assertEqual(auftrag["abschnitte"], [[0, 1]])
        self.assertTrue(any(t.startswith("Kein Radius") for t in
                            self.stufen(befunde, sequenz.HINWEIS)))

    def test_nur_lin_traegt_fahrarten(self):
        last = nutzlast(posten("P1", (0.5, 0, 0), 0.02),
                        posten("P2", (0.5, 0.5, 0), 0.0))
        auftrag, _, _ = self.frage(last)
        self.assertEqual(auftrag["fahrarten"], ["LIN"])
        self.assertEqual(auftrag["ueberschleifen_rad"], I.UEBERSCHLEIFEN_RAD)
        self.assertEqual(I.UEBERSCHLEIFEN_RAD, 0.2)

    def test_circ_ist_fehler(self):
        last = nutzlast(posten("P1", (0.5, 0, 0), 0.02, fahrart="PTP"),
                        posten("P2", (0.5, 0.5, 0), 0.0, fahrart="CIRC"),
                        posten("P3", (0.5, 0.5, 0.5), 0.0, fahrart=""))
        auftrag, befunde, grenzen = self.frage(last)
        self.assertIsNone(auftrag)
        self.assertEqual(grenzen.anzahl, 0)
        fehler = [b for b in befunde if b.stufe == sequenz.FEHLER]
        self.assertEqual([b.stelle for b in fehler], [1, 2])
        self.assertIn("P2: Fahrart CIRC", fehler[0].text)
        self.assertIn("P3: Fahrart (keine)", fehler[1].text)
        # Keine Folgemeldungen (Wechsel, Verrundung) zu einer Liste, die so
        # nicht gerechnet wird.
        self.assertEqual(self.stufen(befunde, sequenz.HINWEIS), [])
        self.assertEqual(self.stufen(befunde, sequenz.WARNUNG), [])

    def test_wechsel_grenze_und_verschleifen(self):
        # Start -LIN-> P1 -LIN-> P2 -PTP-> P3 -PTP-> P4 -LIN-> P5
        # P2: Wechsel mit Radius 0 -> Halt. P4: Wechsel mit Radius -> ohne
        # Halt, ueberschliffen.
        last = nutzlast(posten("P1", (0.5, 0, 0), 0.02),
                        posten("P2", (0.5, 0.5, 0), 0.0),
                        posten("P3", (0.5, 0.5, 0.5), 0.05, fahrart="PTP"),
                        posten("P4", (1.0, 0.5, 0.5), 0.05, fahrart="PTP"),
                        posten("P5", (1.0, 1.0, 0.5), 0.0))
        auftrag, befunde, _ = self.frage(last)
        self.assertEqual(self.stufen(befunde, sequenz.FEHLER), [])
        self.assertEqual(self.stufen(befunde, sequenz.WARNUNG), [])
        # Jeder Abschnitt hat eine Fahrart -- der Wechsel bleibt Grenze.
        self.assertEqual(auftrag["abschnitte"], [[0, 2], [2, 4], [4, 5]])
        self.assertEqual(auftrag["fahrarten"], ["LIN", "PTP", "LIN"])
        self.assertEqual(auftrag["verschleifen"], [4])
        # Die PTP-Radien zaehlen nicht zu den LIN-Radien: kein Fehler,
        # Verrundung aus dem LIN-Radius.
        self.assertEqual(auftrag["rundung_translation_m"], 0.02)
        hinweise = self.stufen(befunde, sequenz.HINWEIS)
        self.assertTrue(any(t.startswith("Genauhalt (Radius 0) an P2.")
                            for t in hinweise), hinweise)
        self.assertTrue(any(t.startswith(
            "Wechsel der Fahrart ueberschliffen im Gelenkraum mit 0.2 rad: "
            "P4 (PTP -> LIN).") for t in hinweise), hinweise)
        self.assertTrue(any(t.startswith("Verrundet mit 0.02 m") and ": P1."
                            in t for t in hinweise), hinweise)
        self.assertTrue(any(t.startswith("PTP ueberschliffen im Gelenkraum "
                                         "mit 0.2 rad: P3.")
                            for t in hinweise), hinweise)
        self.assertEqual(auftrag["stand"], 4)

    def test_wechsel_mit_umkehr_haelt(self):
        # P1 hat Radius, der Wechsel LIN -> PTP kehrt aber um: Halt.
        last = nutzlast(posten("P1", (0.5, 0, 0), 0.02),
                        posten("P2", (0.1, 0, 0), 0.0, fahrart="PTP"))
        auftrag, befunde, _ = self.frage(last)
        self.assertEqual(auftrag["abschnitte"], [[0, 1], [1, 2]])
        self.assertEqual(auftrag["umkehr"], [1])
        self.assertEqual(auftrag["verschleifen"], [])
        self.assertTrue(any("P1: die Bahn kehrt" in t for t in
                            self.stufen(befunde, sequenz.WARNUNG)))

    def test_nur_ptp(self):
        last = nutzlast(posten("P1", (0.5, 0, 0), 0.03, fahrart="PTP"),
                        posten("P2", (0.5, 0.5, 0), 0.07, fahrart="PTP"),
                        posten("P3", (0.5, 0.5, 0.5), 0.0, fahrart="PTP"))
        auftrag, befunde, _ = self.frage(last)
        # Verschiedene Radien sind bei PTP kein Fehler: sie wirken nicht
        # als Laenge.
        self.assertEqual(self.stufen(befunde, sequenz.FEHLER), [])
        self.assertEqual(self.stufen(befunde, sequenz.WARNUNG), [])
        self.assertEqual(auftrag["abschnitte"], [[0, 3]])
        self.assertEqual(auftrag["fahrarten"], ["PTP"])
        self.assertEqual(auftrag["rundung_translation_m"],
                         I.RUNDUNG_TRANS_OHNE_RADIUS)
        hinweise = self.stufen(befunde, sequenz.HINWEIS)
        self.assertFalse(any(t.startswith("Kein Radius") for t in hinweise))
        self.assertTrue(any("0.2 rad: P1, P2." in t for t in hinweise),
                        hinweise)

    def test_ptp_genauhalt_und_umkehr(self):
        last = nutzlast(posten("P1", (0.5, 0, 0), 0.0, fahrart="PTP"),
                        posten("P2", (0.5, 0.5, 0), 0.02, fahrart="PTP"),
                        posten("P3", (0.5, 0.1, 0), 0.0, fahrart="PTP"))
        auftrag, befunde, _ = self.frage(last)
        self.assertEqual(auftrag["abschnitte"], [[0, 1], [1, 2], [2, 3]])
        self.assertEqual(auftrag["fahrarten"], ["PTP"] * 3)
        self.assertEqual(auftrag["umkehr"], [2])

    def test_lin_radius_nur_zwischen_lin(self):
        # 0.02 zwischen LIN-Zeilen, 0.04 am Wechsel: nur ein LIN-Radius.
        last = nutzlast(posten("P1", (0.5, 0, 0), 0.02),
                        posten("P2", (0.5, 0.5, 0), 0.04),
                        posten("P3", (0.5, 0.5, 0.5), 0.0, fahrart="PTP"))
        auftrag, befunde, _ = self.frage(last)
        self.assertIsNotNone(auftrag, befunde)
        self.assertEqual(auftrag["rundung_translation_m"], 0.02)
        self.assertEqual(auftrag["abschnitte"], [[0, 2], [2, 3]])
        self.assertEqual(auftrag["fahrarten"], ["LIN", "PTP"])
        self.assertEqual(auftrag["verschleifen"], [2])

    def test_override_einheitlich(self):
        last = nutzlast(posten("P1", (0.5, 0, 0), v=0.5),
                        posten("P2", (0.5, 0.5, 0), v=0.8))
        auftrag, befunde, _ = self.frage(last)
        self.assertIsNone(auftrag)
        self.assertTrue(any("Override" in t for t in
                            self.stufen(befunde, sequenz.FEHLER)))

    def test_umkehr(self):
        last = nutzlast(posten("P1", (0.5, 0, 0), 0.02),
                        posten("P2", (0.1, 0, 0), 0.0))
        auftrag, befunde, _ = self.frage(last)
        self.assertEqual(auftrag["abschnitte"], [[0, 1], [1, 2]])
        self.assertEqual(auftrag["umkehr"], [1])
        warnungen = self.stufen(befunde, sequenz.WARNUNG)
        self.assertEqual(len(warnungen), 1)
        self.assertIn("P1: die Bahn kehrt", warnungen[0])

    def test_fehlendes_vor_der_rechnung(self):
        last = nutzlast(posten("P1", (0.5, 0, 0)))
        for kwargs, wort in (({"params": ""}, "Parameterdatei"),
                             ({"tip": ""}, "Tip-Link"),
                             ({"wrapper": "/gibt/es/nicht"}, "Wrapper"),
                             ({"wrapper": self.params}, "Wrapper"),
                             ({"rot": 0.0}, "rundung_rot")):
            auftrag, befunde, grenzen = self.frage(last, **kwargs)
            self.assertIsNone(auftrag, kwargs)
            self.assertEqual(grenzen.anzahl, 0, kwargs)
            self.assertTrue(any(wort in t for t in
                                self.stufen(befunde, sequenz.FEHLER)), kwargs)

    def test_leer_und_ohne_ziel(self):
        auftrag, befunde, _ = self.frage(nutzlast())
        self.assertIsNone(auftrag)
        self.assertIn("Die Folge ist leer", self.stufen(befunde, sequenz.FEHLER))
        ohne = posten("P1", (0.5, 0, 0))
        del ohne["goal"]
        auftrag, befunde, _ = self.frage(nutzlast(ohne))
        self.assertIsNone(auftrag)
        luecke = posten("P1", (0.5, 0, 0))
        del luecke["goal"]["j2"]
        auftrag, befunde, _ = self.frage(nutzlast(luecke))
        self.assertIsNone(auftrag)
        self.assertTrue(any("ohne j2" in t for t in
                            self.stufen(befunde, sequenz.FEHLER)))

    def test_start_unvollstaendig(self):
        last = nutzlast(posten("P1", (0.5, 0, 0)))
        del last["items"][0]["start"]["j3"]
        auftrag, befunde, _ = self.frage(last)
        self.assertIsNone(auftrag)
        self.assertTrue(any("fuehrt j3 nicht" in t for t in
                            self.stufen(befunde, sequenz.FEHLER)))

    def test_grenzen(self):
        last = nutzlast(posten("P1", (0.5, 0, 0)))
        auftrag, befunde, _ = self.frage(
            last, grenzen=Aufrufe(hinweise=["j1: continuous"]))
        self.assertIn("j1: continuous", self.stufen(befunde, sequenz.HINWEIS))
        auftrag, befunde, _ = self.frage(
            last, grenzen=Aufrufe(fehler=ValueError("j2 ohne a")))
        self.assertIsNone(auftrag)
        self.assertIn("Achsgrenzen nicht lesbar: j2 ohne a",
                      self.stufen(befunde, sequenz.FEHLER))

    def test_wrapper_aus_umgebung(self):
        alt = os.environ.get("INTRINSIC_WRAPPER")
        try:
            os.environ["INTRINSIC_WRAPPER"] = "/x/y"
            self.assertEqual(I.wrapper_pfad(), "/x/y")
            del os.environ["INTRINSIC_WRAPPER"]
            self.assertEqual(I.wrapper_pfad(), I.WRAPPER_STANDARD)
        finally:
            if alt is not None:
                os.environ["INTRINSIC_WRAPPER"] = alt


def antwort(**aenderung):
    daten = {
        "ok": True, "stand": 2, "intrinsic_release": "20260922.0",
        "rechner": "intrinsic_lin_rechner", "gelenke": GELENKE,
        "punkte": [[0.0, [0, 0, 0], [0, 0, 0]],
                   [0.5, [0.25, 0, 0], [1, 0, 0]],
                   [1.0, [0.5, 0, 0], [0, 0, 0]]],
        "dauer_s": 1.0, "teilbahnen": [3], "eingang": 40,
        "rechenzeit_s": 0.02, "moveit_version": "2.12.3", "start_s": 3.5,
        "uebergang_max_rad": 1e-9, "ik_aufrufe": 144,
        "ik_rechenzeit_s": 0.4,
        "abschnitte": [{"toppra_iterationen": 2}, {"toppra_iterationen": 3}],
        "grenzpruefung": {
            "roh": {"v_faktor_max": 0.98, "a_faktor_max": 0.97,
                    "eingehalten": True},
            "abgetastet": {"v_faktor_max": 0.99, "a_faktor_max": 1.01,
                           "eingehalten": False}},
        "lin": [{"ik_sprung_max_dicht_rad": 0.01, "ziel_abweichung_rad": 1e-9,
                 "pfadpunkte": 12},
                {"ik_sprung_max_dicht_rad": 0.03, "ziel_abweichung_rad": 2e-9,
                 "pfadpunkte": 7}],
        "hinweise": ["P1 -> P2: h1"],
        "lin_vorgaben": {"rundung_translation_m": 0.02},
        "feste_werte_intrinsic": {"sicherheitsfaktor": 0.98},
    }
    daten.update(aenderung)
    return json.dumps(daten)


class Antwort(unittest.TestCase):

    def test_lesen(self):
        g = I.antwort_lesen(antwort(), GELENKE)
        self.assertEqual(g["code"], 1)
        self.assertNotIn("totg", g)
        self.assertEqual(len(g["points"]), 3)
        self.assertEqual(g["points"][1][1], {"j1": 0.25, "j2": 0.0, "j3": 0.0})
        self.assertEqual(g["teilbahnen"], [3])
        self.assertEqual(g["planning_time"], 0.02)
        i = g["intrinsic"]
        self.assertEqual(i["dauer_s"], 1.0)
        self.assertEqual(i["moveit_version"], "2.12.3")
        self.assertEqual(i["start_s"], 3.5)
        self.assertEqual(i["intrinsic_release"], "20260922.0")
        self.assertEqual(i["wrapper_stand"], 2)
        self.assertEqual(i["toppra_iterationen"], [2, 3])
        self.assertEqual(i["v_faktor_max"], 0.98)
        self.assertEqual(i["a_faktor_max"], 0.97)
        self.assertIs(i["grenzen_roh_eingehalten"], True)
        self.assertEqual(i["v_faktor_max_abgetastet"], 0.99)
        self.assertEqual(i["a_faktor_max_abgetastet"], 1.01)
        self.assertIs(i["grenzen_abgetastet_eingehalten"], False)
        self.assertEqual(i["ik_aufrufe"], 144)
        self.assertEqual(i["ik_sprung_max_rad"], 0.03)
        self.assertEqual(i["ziel_abweichung_max_rad"], 2e-9)
        self.assertEqual(i["pfadpunkte"], [12, 7])
        self.assertEqual(i["fahrarten"], ["LIN", "LIN"])
        self.assertEqual(i["hinweise"], ["P1 -> P2: h1"])
        self.assertEqual(i["uebergang_max_rad"], 1e-9)
        self.assertEqual(i["feste_werte"], {"sicherheitsfaktor": 0.98})

    def test_verschliffen(self):
        i = I.antwort_lesen(antwort(verschliffen=[
            {"punkt": "P3", "sprung_rad": 2e-4},
            {"punkt": "P5", "sprung_rad": 7e-4}]), GELENKE)["intrinsic"]
        self.assertEqual(i["verschliffen"], ["P3", "P5"])
        self.assertEqual(i["verschliffen_sprung_max_rad"], 7e-4)
        i = I.antwort_lesen(antwort(), GELENKE)["intrinsic"]
        self.assertEqual(i["verschliffen"], [])
        self.assertEqual(i["verschliffen_sprung_max_rad"], 0.0)

    def test_mit_ptp(self):
        i = I.antwort_lesen(antwort(lin=[
            {"fahrart": "PTP", "pfadpunkte": 3},
            {"fahrart": "LIN", "ik_sprung_max_dicht_rad": 0.02,
             "ik_posenfehler_max_m": 4e-4, "ik_posenfehler_max_rad": 2e-3,
             "ziel_abweichung_rad": 1e-9, "pfadpunkte": 9}]),
            GELENKE)["intrinsic"]
        self.assertEqual(i["ik_posenfehler_max_m"], 4e-4)
        self.assertEqual(i["ik_posenfehler_max_rad"], 2e-3)
        self.assertEqual(i["fahrarten"], ["PTP", "LIN"])
        self.assertEqual(i["pfadpunkte"], [3, 9])
        self.assertEqual(i["ik_sprung_max_rad"], 0.02)

    def test_ohne_lin(self):
        i = I.antwort_lesen(antwort(lin=[]), GELENKE)["intrinsic"]
        self.assertEqual(i["ik_sprung_max_rad"], 0.0)
        self.assertEqual(i["ziel_abweichung_max_rad"], 0.0)

    def test_abgelehnt(self):
        with self.assertRaisesRegex(ValueError, "Wrapper kaputt"):
            I.antwort_lesen(antwort(ok=False, fehler="Wrapper kaputt"),
                            GELENKE)
        with self.assertRaisesRegex(ValueError, "Achsen"):
            I.antwort_lesen(antwort(), ["j1", "j2"])
        with self.assertRaisesRegex(ValueError, "JSON"):
            I.antwort_lesen("{kaputt", GELENKE)
        with self.assertRaisesRegex(ValueError, "endliche Zeit"):
            I.antwort_lesen(antwort(punkte=[[math.inf, [0, 0, 0], [0, 0, 0]],
                                            [1, [0, 0, 0], [0, 0, 0]]]),
                            GELENKE)


class Csv(unittest.TestCase):

    def test_zusatzzeilen(self):
        herkunft = {"verfahren": "Intrinsic", "resample_dt": 0.004,
                    "rundung_translation_m": 0.02,
                    "toppra_iterationen": [2, 3],
                    "fahrarten": ["LIN", "PTP"],
                    "grenzen_roh_eingehalten": True,
                    "ruck_begrenzt": False,
                    "intrinsic_hinweise": ["a", "b"]}
        zeilen = I.zusatzzeilen(herkunft)
        self.assertEqual(zeilen[0], ["#verfahren", "Intrinsic"])
        self.assertIn(["#resample_dt", "0.004"], zeilen)
        self.assertIn(["#rundung_translation_m", "0.02"], zeilen)
        self.assertIn(["#toppra_iterationen", "2,3"], zeilen)
        self.assertIn(["#fahrarten", "LIN,PTP"], zeilen)
        self.assertIn(["#grenzen_roh_eingehalten", "True"], zeilen)
        self.assertIn(["#ruck_begrenzt", "False"], zeilen)
        self.assertEqual(zeilen[-2:], [["#intrinsic_hinweis", "a"],
                                       ["#intrinsic_hinweis", "b"]])
        schluessel = [z[0] for z in zeilen]
        self.assertEqual(len(schluessel), len(set(schluessel)) + 1)


if __name__ == "__main__":
    unittest.main(verbosity=1)
