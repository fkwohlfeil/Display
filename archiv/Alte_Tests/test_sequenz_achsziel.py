#!/usr/bin/env python3
"""Tests fuer die Achsziele (J-Zeilen) in sequenz.py, seit 3.5.

Laeuft ohne ROS, ohne Tkinter und ohne Bildschirm:

    python3 test_sequenz_achsziel.py

Das Modul unter Test kommt aus SEQUENZ_PFAD, sonst aus sequenz.py neben
dieser Datei. Es wird unter einem eindeutigen Namen geladen, damit
mutation_sequenz_achsziel.py keine Bytecode-Reste eines anderen Mutanten
erwischt.
"""

import hashlib
import importlib.util
import math
import os
import sys
import unittest
from pathlib import Path


def lade_sequenz():
    pfad = Path(os.environ.get("SEQUENZ_PFAD")
                or Path(__file__).resolve().parent / "sequenz.py").resolve()
    name = "sequenz_unter_test_" + hashlib.sha1(
        str(pfad).encode()).hexdigest()[:10]
    spec = importlib.util.spec_from_file_location(name, pfad)
    modul = importlib.util.module_from_spec(spec)
    sys.modules[name] = modul
    spec.loader.exec_module(modul)
    return modul


S = lade_sequenz()

# Sieben Drehachsen wie bmKinematic, dazu Varianten fuer Sonderfaelle.
GRENZE = math.radians(170.0)
GELENKE7 = tuple({"name": f"joint{i}", "drehend": True,
                  "unten": -GRENZE, "oben": GRENZE} for i in range(1, 8))


def achsziel(werte, **felder):
    return S.Punkt(xyz=None, abc=None, achsziel=werte, **felder)


def aufgeloest(punkt, xyz, gelenke=GELENKE7):
    achsen, befunde = S.loese_achsziel(punkt, gelenke)
    assert not befunde, befunde
    punkt.achsen = achsen
    punkt.ergaenze_lage(xyz, (0.0, 0.0, 0.0))
    return punkt


def texte(befunde, stufe=None):
    return [b.text for b in befunde if stufe is None or b.stufe == stufe]


class Lesen(unittest.TestCase):

    def test_j_zeile_traegt_achsziel_und_keine_lage(self):
        p = S.lies_zeile("J 0 0 0 0 0 0 0 tempo=50 name=Null")
        self.assertEqual(p.achsziel, (0.0,) * 7)
        self.assertIsNone(p.xyz)
        self.assertIsNone(p.abc)
        self.assertIsNone(p.achsen)
        self.assertEqual(p.tempo, 50.0)
        self.assertEqual(p.name, "Null")
        self.assertIsNone(p.fahrart)
        self.assertIsNone(p.radius)

    def test_kleines_j(self):
        p = S.lies_zeile("j 1 2.5 -3")
        self.assertEqual(p.achsziel, (1.0, 2.5, -3.0))

    def test_beschleunigung_erlaubt(self):
        p = S.lies_zeile("J 1 2 beschleunigung=30")
        self.assertEqual(p.beschleunigung, 30.0)

    def test_fahrart_und_radius_verboten_auch_passend(self):
        for zeile in ("J 0 0 fahrart=LIN", "J 0 0 fahrart=PTP",
                      "J 0 0 radius=0.05", "J 0 0 radius=0",
                      "J 0 0 name=x RADIUS=0"):
            with self.subTest(zeile=zeile):
                with self.assertRaises(ValueError) as ctx:
                    S.lies_zeile(zeile)
                self.assertIn("J-Zeile", str(ctx.exception))

    def test_j_ohne_werte(self):
        with self.assertRaises(ValueError) as ctx:
            S.lies_zeile("J name=leer")
        self.assertIn("ohne Werte", str(ctx.exception))
        with self.assertRaises(ValueError):
            S.lies_zeile("J")

    def test_j_mit_unsinn(self):
        with self.assertRaises(ValueError) as ctx:
            S.lies_zeile("J 0 x 0")
        self.assertIn("keine Zahl", str(ctx.exception))
        with self.assertRaises(ValueError) as ctx:
            S.lies_zeile("J 0 0 geschwindigkeit=3")
        self.assertIn("kein Feld dieses Namens", str(ctx.exception))
        with self.assertRaises(ValueError) as ctx:
            S.lies_zeile("J 0 0 tempo=1 tempo=2")
        self.assertIn("Feld doppelt", str(ctx.exception))

    def test_kartesische_zeile_unveraendert(self):
        p = S.lies_zeile("1 2 3 45 0 10 fahrart=lin tempo=80 radius=0.05 name=A")
        self.assertEqual(p.xyz, (1.0, 2.0, 3.0))
        self.assertEqual(p.abc, (45.0, 0.0, 10.0))
        self.assertIsNone(p.achsziel)
        self.assertEqual(p.fahrart, "LIN")
        self.assertEqual(p.radius, 0.05)
        with self.assertRaises(ValueError) as ctx:
            S.lies_zeile("1 2 3 4 5 6 7")
        self.assertEqual(str(ctx.exception),
                         "sechs Zahlen erwartet (X Y Z A B C), 7 gefunden")

    def test_liste_meldet_zeilennummer(self):
        punkte, befunde = S.lies_liste(
            "# Kopf\nJ 0 0 0 0 0 0 0\nJ 0 0 radius=0\n1 1 1 0 0 0\n")
        self.assertEqual(len(punkte), 2)
        self.assertEqual(len(befunde), 1)
        self.assertTrue(befunde[0].text.startswith("Zeile 3:"))

    def test_punkt_ohne_lage_nur_als_achsziel(self):
        with self.assertRaises(ValueError):
            S.Punkt(xyz=None)
        S.Punkt(xyz=None, achsziel=(0.0,))

    def test_lage_nachtragen_nur_bei_achsziel(self):
        with self.assertRaises(ValueError):
            S.Punkt(xyz=(1, 2, 3)).ergaenze_lage((0, 0, 0), (0, 0, 0))
        p = achsziel((0.0,))
        p.ergaenze_lage((1, 2, 3), (4, 5, 6))
        self.assertEqual(p.xyz, (1.0, 2.0, 3.0))
        self.assertEqual(p.abc, (4.0, 5.0, 6.0))


class Zuordnen(unittest.TestCase):

    def test_ohne_achsziel_nichts(self):
        self.assertEqual(S.loese_achsziel(S.Punkt(xyz=(0, 0, 0)), GELENKE7),
                         (None, ()))

    def test_nullstellung(self):
        achsen, befunde = S.loese_achsziel(achsziel((0.0,) * 7), GELENKE7)
        self.assertEqual(befunde, ())
        self.assertEqual(list(achsen), [f"joint{i}" for i in range(1, 8)])
        self.assertTrue(all(w == 0.0 for w in achsen.values()))

    def test_grad_in_bogenmass_in_reihenfolge(self):
        werte = (10.0, -20.0, 0.0, 45.0, 0.0, 30.0, 90.0)
        achsen, _ = S.loese_achsziel(achsziel(werte), GELENKE7)
        self.assertEqual(tuple(achsen.values()),
                         tuple(math.radians(w) for w in werte))

    def test_falsche_anzahl(self):
        for werte in ((0.0,) * 6, (0.0,) * 8):
            with self.subTest(n=len(werte)):
                achsen, befunde = S.loese_achsziel(achsziel(werte), GELENKE7,
                                                   index=2)
                self.assertIsNone(achsen)
                self.assertEqual(len(befunde), 1)
                self.assertEqual(befunde[0].stufe, S.FEHLER)
                self.assertEqual(befunde[0].stelle, 2)
                self.assertIn(f"{len(werte)} Achswert(e)", befunde[0].text)
                self.assertIn("7 bewegliche", befunde[0].text)
                self.assertTrue(befunde[0].text.startswith("P3:"))

    def test_grenze_wird_nicht_begrenzt(self):
        werte = [0.0] * 7
        werte[2] = 171.0
        werte[5] = -171.0
        achsen, befunde = S.loese_achsziel(
            achsziel(werte, name="Weit"), GELENKE7)
        self.assertIsNone(achsen)
        self.assertEqual(len(befunde), 2)
        self.assertIn("joint3 = 171.0 deg", befunde[0].text)
        self.assertIn("-170.000000 .. 170.000000 deg", befunde[0].text)
        self.assertIn("joint6 = -171.0 deg", befunde[1].text)
        self.assertTrue(befunde[0].text.startswith("Weit:"))

    def test_genau_auf_der_grenze_erlaubt(self):
        gelenke = ({"name": "a", "drehend": True,
                    "unten": math.radians(-90.0), "oben": math.radians(90.0)},)
        for wert in (90.0, -90.0):
            with self.subTest(wert=wert):
                achsen, befunde = S.loese_achsziel(achsziel((wert,)), gelenke)
                self.assertEqual(befunde, ())
                self.assertEqual(achsen["a"], math.radians(wert))

    def test_knapp_ueber_der_grenze_verboten(self):
        gelenke = ({"name": "a", "drehend": True,
                    "unten": None, "oben": math.radians(90.0)},)
        wert = math.degrees(math.nextafter(math.radians(90.0), 4.0))
        if math.radians(wert) <= math.radians(90.0):
            wert = 90.000001
        achsen, befunde = S.loese_achsziel(achsziel((wert,)), gelenke)
        self.assertIsNone(achsen)
        self.assertIn("offen .. 90.000000 deg", befunde[0].text)

    def test_schubachse_in_metern(self):
        gelenke = ({"name": "lift", "drehend": False,
                    "unten": 0.0, "oben": 0.5},)
        achsen, befunde = S.loese_achsziel(achsziel((0.25,)), gelenke)
        self.assertEqual(befunde, ())
        self.assertEqual(achsen["lift"], 0.25)
        achsen, befunde = S.loese_achsziel(achsziel((0.6,)), gelenke)
        self.assertIsNone(achsen)
        self.assertIn("0.6 m", befunde[0].text)
        self.assertIn("0.000000 .. 0.500000 m", befunde[0].text)

    def test_ohne_grenze_alles(self):
        gelenke = ({"name": "endlos", "drehend": True,
                    "unten": None, "oben": None},)
        achsen, befunde = S.loese_achsziel(achsziel((720.0,)), gelenke)
        self.assertEqual(befunde, ())
        self.assertEqual(achsen["endlos"], math.radians(720.0))

    def test_nicht_endlich(self):
        gelenke = ({"name": "endlos", "drehend": True,
                    "unten": None, "oben": None},)
        for wert in ("nan", "inf", "-inf"):
            with self.subTest(wert=wert):
                achsen, befunde = S.loese_achsziel(
                    achsziel((float(wert),)), gelenke)
                self.assertIsNone(achsen)
                self.assertIn("keine endliche Zahl", befunde[0].text)


class Aufloesen(unittest.TestCase):

    def folge(self):
        p1 = S.Punkt(xyz=(0, 0, 0), fahrart="LIN", radius=0.05, tempo=80)
        p2 = achsziel((0.0,) * 7, tempo=40)
        p3 = S.Punkt(xyz=(1, 0, 0))
        p4 = S.Punkt(xyz=(2, 0, 0), radius=0.0)
        return S.Folge([p1, p2, p3, p4])

    def test_achsziel_legt_fest(self):
        s = self.folge().aufloesen()
        self.assertEqual(s[1].fahrart, "PTP")
        self.assertEqual(s[1].radius, 0.0)
        self.assertEqual(s[1].herkunft["fahrart"], "achsziel")
        self.assertEqual(s[1].herkunft["radius"], "achsziel")

    def test_modalzustand_geht_ueber_achsziel_hinweg(self):
        s = self.folge().aufloesen()
        self.assertEqual(s[2].fahrart, "LIN")
        self.assertEqual(s[2].radius, 0.05)
        self.assertEqual(s[2].herkunft["fahrart"], "modal")
        self.assertEqual(s[2].herkunft["radius"], "modal")

    def test_tempo_bleibt_modal(self):
        s = self.folge().aufloesen()
        self.assertEqual(s[1].tempo, 40.0)
        self.assertEqual(s[1].herkunft["tempo"], "zeile")
        self.assertEqual(s[2].tempo, 40.0)
        self.assertEqual(s[2].herkunft["tempo"], "modal")

    def test_achsziel_ohne_tempo_erbt(self):
        f = S.Folge([S.Punkt(xyz=(0, 0, 0), tempo=70), achsziel((0.0,))])
        s = f.aufloesen()
        self.assertEqual(s[1].tempo, 70.0)
        self.assertEqual(s[1].herkunft["tempo"], "modal")
        self.assertEqual(s[1].herkunft["beschleunigung"], "modal")

    def test_achsziel_ignoriert_modalstart(self):
        f = S.Folge([achsziel((0.0,))], modal={"fahrart": "LIN",
                                                "radius": 0.1})
        s = f.aufloesen()
        self.assertEqual((s[0].fahrart, s[0].radius), ("PTP", 0.0))

    def test_endzustand(self):
        f = S.Folge([S.Punkt(xyz=(0, 0, 0), fahrart="LIN", radius=0.02),
                     achsziel((0.0,), tempo=33)])
        ende = f.endzustand()
        self.assertEqual(ende["fahrart"], "LIN")
        self.assertEqual(ende["radius"], 0.02)
        self.assertEqual(ende["tempo"], 33.0)

    def test_endzustand_wie_aufloesen_auch_mit_feld_auf_achsziel(self):
        # Programmatisch entstanden: das Feld ist ein Fehler fuer pruefe(),
        # darf aber auch die Anzeige des Endzustands nicht verschieben.
        f = S.Folge([S.Punkt(xyz=(0, 0, 0), fahrart="LIN", radius=0.02),
                     achsziel((0.0,), fahrart="CIRC", radius=0.3),
                     S.Punkt(xyz=(1, 0, 0))])
        ende = f.endzustand()
        letzter = f.aufloesen()[-1]
        self.assertEqual((ende["fahrart"], ende["radius"]), ("LIN", 0.02))
        self.assertEqual((letzter.fahrart, letzter.radius), ("LIN", 0.02))


class Pruefen(unittest.TestCase):

    def test_nicht_aufgeloest_ist_fehler(self):
        f = S.Folge([S.Punkt(xyz=(0, 0, 0)), achsziel((0.0,) * 7)])
        fehler = texte(f.pruefe(), S.FEHLER)
        self.assertTrue(any("nicht aufgeloest" in t for t in fehler), fehler)

    def test_nur_achsen_ohne_lage_ist_fehler(self):
        p = achsziel((0.0,) * 7)
        p.achsen, _ = S.loese_achsziel(p, GELENKE7)
        f = S.Folge([S.Punkt(xyz=(1, 0, 0)), p])
        self.assertTrue(any("nicht aufgeloest" in t
                            for t in texte(f.pruefe(), S.FEHLER)))

    def test_nur_lage_ohne_achsen_ist_fehler(self):
        p = achsziel((0.0,) * 7)
        p.ergaenze_lage((1, 0, 0), (0, 0, 0))
        f = S.Folge([S.Punkt(xyz=(0, 0, 0)), p])
        self.assertTrue(any("nicht aufgeloest" in t
                            for t in texte(f.pruefe(), S.FEHLER)))

    def test_aufgeloest_ist_sauber(self):
        p = aufgeloest(achsziel((0.0,) * 7), (1, 0, 0))
        f = S.Folge([S.Punkt(xyz=(0, 0, 0)), p])
        self.assertEqual(texte(f.pruefe(), S.FEHLER), [])

    def test_programmatisches_feld_auf_achsziel(self):
        for felder in ({"fahrart": "LIN"}, {"fahrart": "PTP"},
                       {"radius": 0.0}, {"radius": 0.1}):
            with self.subTest(felder=felder):
                p = aufgeloest(achsziel((0.0,) * 7, **felder), (1, 0, 0))
                f = S.Folge([S.Punkt(xyz=(0, 0, 0)), p])
                fehler = texte(f.pruefe(), S.FEHLER)
                feld = next(iter(felder))
                self.assertTrue(any(f"{feld}= steht auf einem Achsziel" in t
                                    for t in fehler), fehler)

    def test_kugel_des_vorgaengers_reicht_bis_zum_achsziel(self):
        p = aufgeloest(achsziel((0.0,) * 7, name="Null"), (0.03, 0, 0))
        f = S.Folge([S.Punkt(xyz=(0, 0, 0), radius=0.05, name="A"), p])
        fehler = texte(f.pruefe(), S.FEHLER)
        self.assertTrue(any("A -> Null" in t and "ueberlappen" in t
                            for t in fehler), fehler)

    def test_kugel_passt(self):
        p = aufgeloest(achsziel((0.0,) * 7), (0.3, 0, 0))
        f = S.Folge([S.Punkt(xyz=(0, 0, 0), radius=0.05), p])
        self.assertEqual(texte(f.pruefe(), S.FEHLER), [])

    def test_ohne_lage_kein_absturz_und_kein_spielraum(self):
        f = S.Folge([S.Punkt(xyz=(0, 0, 0)), achsziel((0.0,) * 7),
                     S.Punkt(xyz=(1, 0, 0))])
        f.pruefe()
        self.assertEqual(f.spielraum(), ())
        f.bericht()

    def test_spielraum_mit_lage(self):
        p = aufgeloest(achsziel((0.0,) * 7), (0.3, 0, 0))
        f = S.Folge([S.Punkt(xyz=(0, 0, 0), radius=0.05), p])
        (zeile,) = f.spielraum()
        self.assertAlmostEqual(zeile["frei"], 0.25)


class Nutzlast(unittest.TestCase):

    def test_achsziel_geht_als_achsziel_ptp_ohne_radius(self):
        p = aufgeloest(achsziel((10.0, 0, 0, 0, 0, 0, 0), tempo=50),
                       (0.3, 0, 0))
        f = S.Folge([S.Punkt(xyz=(0, 0, 0), fahrart="LIN", radius=0.05,
                             achsen={"joint1": 0.0}), p])
        last = f.nutzlast("arm", start={"joint1": 0.0}, toleranz=0.01)
        eintrag = last["items"][1]
        self.assertEqual(eintrag["planner_id"], "PTP")
        self.assertEqual(eintrag["blend_radius"], 0.0)
        self.assertEqual(eintrag["velocity_scaling"], 0.5)
        self.assertNotIn("pose", eintrag)
        self.assertEqual(eintrag["goal"]["joint1"], math.radians(10.0))
        self.assertEqual(last["items"][0]["planner_id"], "LIN")

    def test_nicht_aufgeloest_geht_nicht_hinaus(self):
        f = S.Folge([S.Punkt(xyz=(0, 0, 0)), achsziel((0.0,) * 7)])
        with self.assertRaises(ValueError):
            f.nutzlast("arm", start={}, toleranz=0.01)

    def test_zweite_sicherung_ohne_pruefe(self):
        p = achsziel((0.0,) * 7)
        p.ergaenze_lage((1, 0, 0), (0, 0, 0))
        f = S.Folge([S.Punkt(xyz=(0, 0, 0)), p])
        f.pruefe = lambda schritte=None: ()
        with self.assertRaises(ValueError) as ctx:
            f.nutzlast("arm", start={}, toleranz=0.01)
        self.assertIn("Achsziel ohne Achswerte", str(ctx.exception))


class Bericht(unittest.TestCase):

    def test_eckige_klammern_und_legende(self):
        p = aufgeloest(achsziel((0.0,) * 7, tempo=50, name="Null"),
                       (0.3, 0.2, 0.1))
        text = S.Folge([S.Punkt(xyz=(0, 0, 0)), p]).bericht()
        zeile = [z for z in text.splitlines() if "Null" in z][0]
        self.assertIn("[PTP]", zeile)
        self.assertIn("[0 m]", zeile)
        self.assertIn("50%", zeile)
        self.assertNotIn("(50%)", zeile)
        self.assertIn("   0.300    0.200    0.100", zeile)
        self.assertIn("eckigen Klammern", text)

    def test_ohne_achsziel_keine_neue_legende(self):
        text = S.Folge([S.Punkt(xyz=(0, 0, 0))]).bericht()
        self.assertNotIn("eckigen Klammern", text)
        self.assertNotIn("[", text)

    def test_ohne_lage_platzhalter(self):
        text = S.Folge([achsziel((0.0,), name="Offen")]).bericht()
        zeile = [z for z in text.splitlines() if "Offen" in z][0]
        self.assertIn("--", zeile)

    def test_repr(self):
        self.assertIn("J(", repr(achsziel((1.0, 2.0), name="x")))
        self.assertEqual(repr(S.Punkt(xyz=(1, 2, 3), name="y")),
                         "<Punkt y (1.0, 2.0, 3.0)>")


if __name__ == "__main__":
    unittest.main(verbosity=1)
