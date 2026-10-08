#!/usr/bin/env python3
"""Testfaelle zu sequenz.py. Ohne Bildschirm, ohne ROS, ohne move_group.

    python3 test_sequenz.py

Geprueft wird die Logikschicht allein. Was hier gruen ist, ist noch keine
gefahrene Bahn -- es ist die Zusicherung, dass die Zahlen, die hinausgehen,
die sind, die eingegeben wurden, und dass die Folgen, die MoveIt ablehnen
wuerde, vorher auffallen.
"""

import math
import unittest

import sequenz
from sequenz import Befund, Folge, Punkt


def p(x, y, z, **rest):
    """Kurzform fuer einen Punkt. Die Winkel spielen hier nirgends mit."""
    return Punkt(xyz=(x, y, z), abc=(0.0, 0.0, 0.0), **rest)


def texte(befunde, stufe=None):
    return [b.text for b in befunde if stufe is None or b.stufe == stufe]


class TestPunkt(unittest.TestCase):

    def test_zahlen_werden_zu_float(self):
        punkt = Punkt(xyz=(1, 2, 3), abc=(45, 0, 10))
        self.assertEqual(punkt.xyz, (1.0, 2.0, 3.0))
        self.assertIsInstance(punkt.xyz[0], float)
        self.assertEqual(punkt.abc, (45.0, 0.0, 10.0))

    def test_fahrart_wird_gross(self):
        self.assertEqual(p(0, 0, 0, fahrart="lin").fahrart, "LIN")

    def test_leere_felder_bleiben_none(self):
        punkt = p(0, 0, 0)
        self.assertEqual(punkt.gesetzt(), ())

    def test_gesetzte_felder_werden_gemeldet(self):
        punkt = p(0, 0, 0, tempo=50, radius=0.1)
        self.assertEqual(set(punkt.gesetzt()), {"tempo", "radius"})

    def test_achsen_werden_kopiert(self):
        achsen = {"a1": 0.5}
        punkt = p(0, 0, 0, achsen=achsen)
        achsen["a1"] = 9.9
        self.assertEqual(punkt.achsen["a1"], 0.5)


class TestAbstand(unittest.TestCase):

    def test_dreiviervier(self):
        self.assertAlmostEqual(sequenz.abstand((0, 0, 0), (3, 4, 0)), 5.0)

    def test_gleicher_ort(self):
        self.assertEqual(sequenz.abstand((1, 2, 3), (1, 2, 3)), 0.0)

    def test_z_zaehlt_mit(self):
        self.assertAlmostEqual(sequenz.abstand((0, 0, 0), (0, 0, 2)), 2.0)

    def test_alle_drei_achsen(self):
        self.assertAlmostEqual(sequenz.abstand((0, 0, 0), (1, 2, 2)), 3.0)

    def test_hoehenunterschied_traegt_den_radius(self):
        """Ein Punkt genau ueber dem anderen: ohne z waere der Abstand null."""
        folge = Folge([p(1, 1, 1.0, radius=0.4), p(1, 1, 1.5, radius=0.0)])
        self.assertEqual(folge.fehler(), ())
        self.assertAlmostEqual(folge.spielraum()[0]["abstand"], 0.5)

    def test_symmetrisch(self):
        a, b = (1.0, -2.0, 0.5), (-3.0, 4.25, 7.0)
        self.assertEqual(sequenz.abstand(a, b), sequenz.abstand(b, a))


class TestModalzustand(unittest.TestCase):

    def test_leere_zeile_erbt_alles(self):
        folge = Folge([p(0, 0, 0)], modal={"tempo": 42.0})
        schritt = folge.aufloesen()[0]
        self.assertEqual(schritt.tempo, 42.0)
        self.assertEqual(schritt.herkunft["tempo"], "modal")

    def test_zeile_schlaegt_modal(self):
        folge = Folge([p(0, 0, 0, tempo=80)], modal={"tempo": 42.0})
        schritt = folge.aufloesen()[0]
        self.assertEqual(schritt.tempo, 80.0)
        self.assertEqual(schritt.herkunft["tempo"], "zeile")

    def test_wert_wirkt_weiter(self):
        """Der Kern der Modalitaet: gesetzt bleibt gesetzt."""
        folge = Folge([p(0, 0, 0, tempo=80), p(1, 0, 0), p(2, 0, 0)])
        schritte = folge.aufloesen()
        self.assertEqual([s.tempo for s in schritte], [80.0, 80.0, 80.0])
        self.assertEqual([s.herkunft["tempo"] for s in schritte],
                         ["zeile", "modal", "modal"])

    def test_wert_wird_wieder_ueberschrieben(self):
        folge = Folge([p(0, 0, 0, tempo=80), p(1, 0, 0, tempo=20),
                       p(2, 0, 0)])
        self.assertEqual([s.tempo for s in folge.aufloesen()],
                         [80.0, 20.0, 20.0])

    def test_felder_wirken_unabhaengig(self):
        folge = Folge([p(0, 0, 0, tempo=80), p(1, 0, 0, fahrart="LIN")])
        zwei = folge.aufloesen()[1]
        self.assertEqual(zwei.tempo, 80.0)
        self.assertEqual(zwei.fahrart, "LIN")
        self.assertEqual(zwei.herkunft["tempo"], "modal")
        self.assertEqual(zwei.herkunft["fahrart"], "zeile")

    def test_aufloesen_aendert_die_folge_nicht(self):
        """Zweimal gerufen muss zweimal dasselbe herauskommen."""
        folge = Folge([p(0, 0, 0, tempo=80), p(1, 0, 0)])
        erste = [s.tempo for s in folge.aufloesen()]
        zweite = [s.tempo for s in folge.aufloesen()]
        self.assertEqual(erste, zweite)
        self.assertEqual(folge.modal["tempo"], sequenz.MODAL_START["tempo"])

    def test_endzustand(self):
        folge = Folge([p(0, 0, 0, tempo=80), p(1, 0, 0, radius=0.0)])
        self.assertEqual(folge.endzustand()["tempo"], 80.0)

    def test_geerbte_felder(self):
        folge = Folge([p(0, 0, 0, tempo=80)])
        self.assertEqual(set(folge.aufloesen()[0].geerbt()),
                         {"fahrart", "beschleunigung", "radius"})

    def test_unbekanntes_modales_feld(self):
        with self.assertRaises(KeyError):
            Folge([], modal={"vorschub": 10.0})


class TestPruefungFelder(unittest.TestCase):

    def test_gute_folge_ist_sauber(self):
        folge = Folge([p(0, 0, 0), p(1, 0, 0), p(2, 0, 0)])
        self.assertEqual(folge.fehler(), ())
        self.assertTrue(folge.tragfaehig())

    def test_leere_folge(self):
        befunde = Folge([]).pruefe()
        self.assertEqual(len(befunde), 1)
        self.assertEqual(befunde[0].stufe, sequenz.FEHLER)

    def test_ein_punkt_ist_nur_ein_hinweis(self):
        folge = Folge([p(0, 0, 0)])
        self.assertEqual(folge.fehler(), ())
        self.assertTrue(any(b.stufe == sequenz.HINWEIS
                            for b in folge.pruefe()))

    def test_unbekannte_fahrart(self):
        folge = Folge([p(0, 0, 0, fahrart="SLERP"), p(1, 0, 0)])
        self.assertTrue(any("gibt es nicht" in t
                            for t in texte(folge.fehler())))

    def test_tempo_null(self):
        folge = Folge([p(0, 0, 0, tempo=0), p(1, 0, 0)])
        self.assertTrue(any("tempo" in t for t in texte(folge.fehler())))

    def test_tempo_ueber_hundert(self):
        folge = Folge([p(0, 0, 0, tempo=120), p(1, 0, 0)])
        self.assertTrue(any("tempo" in t for t in texte(folge.fehler())))

    def test_tempo_genau_hundert_ist_erlaubt(self):
        folge = Folge([p(0, 0, 0, tempo=100), p(1, 0, 0)])
        self.assertEqual(folge.fehler(), ())

    def test_beschleunigung_negativ(self):
        folge = Folge([p(0, 0, 0, beschleunigung=-5), p(1, 0, 0)])
        self.assertTrue(any("beschleunigung" in t
                            for t in texte(folge.fehler())))

    def test_negativer_radius(self):
        folge = Folge([p(0, 0, 0, radius=-0.1), p(1, 0, 0)])
        self.assertTrue(any("negativ" in t for t in texte(folge.fehler())))

    def test_circ_ohne_hilfspunkt(self):
        folge = Folge([p(0, 0, 0, fahrart="CIRC"), p(1, 0, 0, fahrart="PTP")])
        self.assertTrue(any("Hilfspunkt" in t for t in texte(folge.fehler())))

    def test_circ_mit_hilfspunkt(self):
        folge = Folge([p(0, 0, 0, fahrart="CIRC", hilfspunkt=(0.5, 0.5, 0)),
                       p(1, 0, 0, fahrart="PTP")])
        self.assertEqual(folge.fehler(), ())

    def test_hilfspunkt_ohne_circ_ist_warnung(self):
        folge = Folge([p(0, 0, 0, hilfspunkt=(0.5, 0.5, 0)), p(1, 0, 0)])
        self.assertEqual(folge.fehler(), ())
        self.assertTrue(any(b.stufe == sequenz.WARNUNG
                            for b in folge.pruefe()))

    def test_alle_fehler_auf_einmal(self):
        """Nicht beim ersten aufhoeren."""
        folge = Folge([p(0, 0, 0, tempo=0, fahrart="X"), p(1, 0, 0, radius=-1)])
        self.assertGreaterEqual(len(folge.fehler()), 3)


class TestPruefungRadien(unittest.TestCase):

    def test_letzter_radius_muss_null_sein(self):
        folge = Folge([p(0, 0, 0), p(1, 0, 0, radius=0.1)])
        self.assertTrue(any("letzter Punkt" in t
                            for t in texte(folge.fehler())))

    def test_modal_geerbter_radius_am_ende_faellt_auch_auf(self):
        """Der haeufige Fall: Radius einmal gesetzt und nie zurueckgenommen."""
        folge = Folge([p(0, 0, 0, radius=0.05), p(1, 0, 0), p(2, 0, 0)])
        self.assertTrue(any("letzter Punkt" in t
                            for t in texte(folge.fehler())))

    def test_richtig_zurueckgenommen(self):
        folge = Folge([p(0, 0, 0, radius=0.05), p(1, 0, 0),
                       p(2, 0, 0, radius=0.0)])
        self.assertEqual(folge.fehler(), ())

    def test_kugeln_ueberlappen(self):
        folge = Folge([p(0, 0, 0, radius=0.6), p(1, 0, 0, radius=0.6),
                       p(2, 0, 0, radius=0.0)])
        fehler = texte(folge.fehler())
        self.assertTrue(any("ueberlappen" in t for t in fehler))

    def test_ueberlappung_nennt_das_paar(self):
        folge = Folge([p(0, 0, 0), p(1, 0, 0, radius=0.9),
                       p(2, 0, 0, radius=0.9), p(9, 0, 0, radius=0.0)])
        treffer = [b for b in folge.fehler() if "ueberlappen" in b.text]
        self.assertEqual(len(treffer), 1)
        self.assertEqual(treffer[0].paar, (1, 2))

    def test_genau_aufgebraucht_ist_zu_viel(self):
        """Summe gleich Abstand: die Kugeln beruehren sich, das reicht nicht."""
        folge = Folge([p(0, 0, 0, radius=0.5), p(1, 0, 0, radius=0.5),
                       p(2, 0, 0, radius=0.0)])
        self.assertTrue(any("ueberlappen" in t for t in texte(folge.fehler())))

    def test_knapp_darunter_geht(self):
        folge = Folge([p(0, 0, 0, radius=0.49), p(1, 0, 0, radius=0.5),
                       p(2, 0, 0, radius=0.0)])
        self.assertEqual(folge.fehler(), ())

    def test_gleicher_ort_ohne_radius_ist_warnung(self):
        folge = Folge([p(1, 1, 1), p(1, 1, 1), p(2, 0, 0)])
        self.assertEqual(folge.fehler(), ())
        self.assertTrue(any(b.stufe == sequenz.WARNUNG
                            for b in folge.pruefe()))

    def test_gleicher_ort_mit_radius_ist_fehler(self):
        folge = Folge([p(1, 1, 1, radius=0.1), p(1, 1, 1), p(2, 0, 0)])
        self.assertTrue(any("selben Ort" in t for t in texte(folge.fehler())))

    def test_spielraum(self):
        folge = Folge([p(0, 0, 0, radius=0.2), p(1, 0, 0, radius=0.0)])
        zeile = folge.spielraum()[0]
        self.assertAlmostEqual(zeile["abstand"], 1.0)
        self.assertAlmostEqual(zeile["belegt"], 0.2)
        self.assertAlmostEqual(zeile["frei"], 0.8)


class TestNutzlast(unittest.TestCase):

    def bahn(self):
        return Folge([p(1, 1, 1, fahrart="LIN", tempo=50, radius=0.05),
                      p(2, 1, 1.5),
                      p(0, 1, 1, radius=0.0)])

    def test_anzahl_posten(self):
        nutz = self.bahn().nutzlast("arm", {"a1": 0.0}, toleranz=0.001)
        self.assertEqual(len(nutz["items"]), 3)

    def test_nur_der_erste_traegt_start(self):
        """Pilz erlaubt den Startzustand nur im ersten Auftrag einer Gruppe."""
        nutz = self.bahn().nutzlast("arm", {"a1": 0.25}, toleranz=0.001)
        self.assertEqual(nutz["items"][0]["start"], {"a1": 0.25})
        self.assertNotIn("start", nutz["items"][1])
        self.assertNotIn("start", nutz["items"][2])

    def test_letzter_radius_ist_null(self):
        nutz = self.bahn().nutzlast("arm", {}, toleranz=0.001)
        self.assertEqual(nutz["items"][-1]["blend_radius"], 0.0)

    def test_radius_wandert_durch(self):
        nutz = self.bahn().nutzlast("arm", {}, toleranz=0.001)
        self.assertAlmostEqual(nutz["items"][0]["blend_radius"], 0.05)

    def test_modal_geerbte_werte_stehen_im_posten(self):
        nutz = self.bahn().nutzlast("arm", {}, toleranz=0.001)
        self.assertEqual(nutz["items"][1]["planner_id"], "LIN")
        self.assertAlmostEqual(nutz["items"][1]["velocity_scaling"], 0.5)

    def test_prozent_wird_zu_anteil(self):
        folge = Folge([p(0, 0, 0, tempo=25, beschleunigung=40), p(1, 0, 0)])
        posten = folge.nutzlast("arm", {}, toleranz=0.001)["items"][0]
        self.assertAlmostEqual(posten["velocity_scaling"], 0.25)
        self.assertAlmostEqual(posten["acceleration_scaling"], 0.40)

    def test_achsziel_wenn_vorhanden(self):
        folge = Folge([p(0, 0, 0, achsen={"a1": 0.5}), p(1, 0, 0)])
        posten = folge.nutzlast("arm", {}, toleranz=0.001)["items"][0]
        self.assertEqual(posten["goal"], {"a1": 0.5})
        self.assertNotIn("pose", posten)

    def test_lageziel_wenn_keine_achsen(self):
        folge = Folge([p(1, 2, 3), p(4, 0, 0)])
        posten = folge.nutzlast("arm", {}, toleranz=0.001, tcp="tool0")
        posten = posten["items"][0]
        self.assertEqual(posten["pose"]["xyz"], (1.0, 2.0, 3.0))
        self.assertEqual(posten["pose"]["tcp"], "tool0")
        self.assertNotIn("goal", posten)

    def test_hilfspunkt_wandert_mit(self):
        folge = Folge([p(0, 0, 0, fahrart="CIRC", hilfspunkt=(0.5, 0.5, 0)),
                       p(1, 0, 0, fahrart="PTP")])
        posten = folge.nutzlast("arm", {}, toleranz=0.001)["items"][0]
        self.assertEqual(posten["hilfspunkt"], (0.5, 0.5, 0.0))

    def test_frist_waechst_mit_der_laenge(self):
        kurz = Folge([p(0, 0, 0), p(1, 0, 0)])
        lang = Folge([p(0, 0, 0), p(1, 0, 0), p(2, 0, 0), p(3, 0, 0)])
        a = kurz.nutzlast("arm", {}, toleranz=0.001, planungszeit=2.0)
        b = lang.nutzlast("arm", {}, toleranz=0.001, planungszeit=2.0)
        self.assertGreater(b["timeout"], a["timeout"])

    def test_frist_enthaelt_zuschlag(self):
        folge = Folge([p(0, 0, 0), p(1, 0, 0)])
        nutz = folge.nutzlast("arm", {}, toleranz=0.001, planungszeit=1.0,
                              versuche=1)
        self.assertAlmostEqual(nutz["timeout"],
                               2 * 1.0 + sequenz.FRIST_ZUSCHLAG_S)

    def test_tag_wandert_durch(self):
        nutz = self.bahn().nutzlast("arm", {}, toleranz=0.001, tag=17)
        self.assertEqual(nutz["tag"], 17)

    def test_fehlerhafte_folge_geht_nicht_hinaus(self):
        folge = Folge([p(0, 0, 0), p(1, 0, 0, radius=0.5)])
        with self.assertRaises(ValueError):
            folge.nutzlast("arm", {}, toleranz=0.001)

    def test_nutzlast_traegt_nur_zahlen_und_text(self):
        """Keine Punkte, keine Schritte, nichts aus diesem Modul."""
        nutz = self.bahn().nutzlast("arm", {"a1": 0.0}, toleranz=0.001)
        erlaubt = (int, float, str, bool, type(None), tuple, list, dict)
        stapel = [nutz]
        while stapel:
            teil = stapel.pop()
            self.assertIsInstance(teil, erlaubt)
            if isinstance(teil, dict):
                stapel.extend(teil.keys())
                stapel.extend(teil.values())
            elif isinstance(teil, (list, tuple)):
                stapel.extend(teil)


class TestZeitreihe(unittest.TestCase):

    def test_saubere_reihe_bleibt(self):
        punkte = [(0.0, {}), (0.1, {}), (0.2, {})]
        behalten, verworfen = sequenz.bereinige_zeitreihe(punkte)
        self.assertEqual(len(behalten), 3)
        self.assertEqual(verworfen, ())

    def test_doppelte_zeit_faellt_weg(self):
        """Das Fehlerbild beim Ueberschleifen."""
        punkte = [(0.0, "a"), (0.1, "b"), (0.1, "c"), (0.2, "d")]
        behalten, verworfen = sequenz.bereinige_zeitreihe(punkte)
        self.assertEqual([e[1] for e in behalten], ["a", "b", "d"])
        self.assertEqual(verworfen, (2,))

    def test_rueckwaerts_faellt_weg(self):
        punkte = [(0.0, "a"), (0.3, "b"), (0.2, "c"), (0.4, "d")]
        behalten, _ = sequenz.bereinige_zeitreihe(punkte)
        self.assertEqual([e[1] for e in behalten], ["a", "b", "d"])

    def test_erster_bleibt_immer(self):
        punkte = [(5.0, "a"), (5.0, "b")]
        behalten, _ = sequenz.bereinige_zeitreihe(punkte)
        self.assertEqual([e[1] for e in behalten], ["a"])

    def test_mindestschritt(self):
        punkte = [(0.0, "a"), (0.0005, "b"), (0.01, "c")]
        behalten, _ = sequenz.bereinige_zeitreihe(punkte, mindestschritt=0.001)
        self.assertEqual([e[1] for e in behalten], ["a", "c"])

    def test_leere_bahn_ist_fehler(self):
        befunde = sequenz.pruefe_bahn([])
        self.assertEqual(befunde[0].stufe, sequenz.FEHLER)

    def test_doppelte_zeit_wird_gemeldet(self):
        befunde = sequenz.pruefe_bahn([(0.0, {}), (0.1, {}), (0.1, {})])
        self.assertTrue(any("wachsende Zeit" in b.text for b in befunde))

    def test_pruefe_bahn_aendert_nichts(self):
        punkte = [(0.0, {}), (0.1, {}), (0.1, {})]
        sequenz.pruefe_bahn(punkte)
        self.assertEqual(len(punkte), 3)

    def test_start_nicht_bei_null(self):
        befunde = sequenz.pruefe_bahn([(0.5, {}), (0.6, {})])
        self.assertTrue(any("nicht bei null" in b.text for b in befunde))


class TestLesen(unittest.TestCase):

    def test_sechs_zahlen(self):
        punkt = sequenz.lies_zeile("1 1 1 45 0 10")
        self.assertEqual(punkt.xyz, (1.0, 1.0, 1.0))
        self.assertEqual(punkt.abc, (45.0, 0.0, 10.0))
        self.assertEqual(punkt.gesetzt(), ())

    def test_benannte_felder(self):
        punkt = sequenz.lies_zeile(
            "2 1 1.5 30 10 25 fahrart=LIN tempo=80 radius=0.05 name=Anfahrt")
        self.assertEqual(punkt.fahrart, "LIN")
        self.assertEqual(punkt.tempo, 80.0)
        self.assertEqual(punkt.radius, 0.05)
        self.assertEqual(punkt.name, "Anfahrt")

    def test_zu_wenige_zahlen(self):
        with self.assertRaises(ValueError):
            sequenz.lies_zeile("1 1 1 45 0")

    def test_zu_viele_zahlen(self):
        with self.assertRaises(ValueError):
            sequenz.lies_zeile("1 1 1 45 0 10 7")

    def test_keine_zahl(self):
        with self.assertRaises(ValueError):
            sequenz.lies_zeile("1 1 x 45 0 10")

    def test_unbekanntes_feld(self):
        with self.assertRaises(ValueError):
            sequenz.lies_zeile("1 1 1 0 0 0 vorschub=100")

    def test_feld_doppelt(self):
        with self.assertRaises(ValueError):
            sequenz.lies_zeile("1 1 1 0 0 0 tempo=10 tempo=20")

    def test_liste_mit_kommentar(self):
        text = ("# Anfahrt\n"
                "1 1 1 45 0 10 radius=0.05\n"
                "\n"
                "2 1 1.5 30 10 25   # durchfahren\n"
                "0 1 1 1 20 30 radius=0\n")
        punkte, befunde = sequenz.lies_liste(text)
        self.assertEqual(len(punkte), 3)
        self.assertEqual(befunde, ())

    def test_liste_meldet_alle_zeilenfehler(self):
        text = "1 1 1 0 0\n2 2 2 0 0 0\n3 x 3 0 0 0\n"
        punkte, befunde = sequenz.lies_liste(text)
        self.assertEqual(len(punkte), 1)
        self.assertEqual(len(befunde), 2)
        self.assertEqual([b.stelle for b in befunde], [1, 3])


class TestDeineFolge(unittest.TestCase):
    """Die drei Punkte aus der Frage, einmal ganz durch."""

    TEXT = ("1 1 1   45  0 10  fahrart=LIN tempo=80 radius=0.05\n"
            "2 1 1.5 30 10 25\n"
            "0 1 1    1 20 30  radius=0\n")

    def test_liest_sich(self):
        punkte, befunde = sequenz.lies_liste(self.TEXT)
        self.assertEqual(befunde, ())
        self.assertEqual(len(punkte), 3)

    def test_ist_tragfaehig(self):
        punkte, _ = sequenz.lies_liste(self.TEXT)
        self.assertEqual(Folge(punkte).fehler(), ())

    def test_mittlerer_punkt_erbt_alles(self):
        punkte, _ = sequenz.lies_liste(self.TEXT)
        mitte = Folge(punkte).aufloesen()[1]
        self.assertEqual(mitte.fahrart, "LIN")
        self.assertEqual(mitte.tempo, 80.0)
        self.assertAlmostEqual(mitte.radius, 0.05)
        self.assertEqual(set(mitte.geerbt()), set(sequenz.MODALFELDER))

    def test_abstaende_tragen_die_radien(self):
        punkte, _ = sequenz.lies_liste(self.TEXT)
        for zeile in Folge(punkte).spielraum():
            self.assertGreater(zeile["frei"], 0.0)

    def test_bericht_zeigt_geerbtes_in_klammern(self):
        punkte, _ = sequenz.lies_liste(self.TEXT)
        zeilen = Folge(punkte).bericht().splitlines()
        self.assertIn("(LIN)", zeilen[1])
        self.assertNotIn("(LIN)", zeilen[0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
