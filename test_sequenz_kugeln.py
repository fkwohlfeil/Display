#!/usr/bin/env python3
"""Tests fuer pruefe(kugeln_pruefen=...) in sequenz.py. Ohne ROS, ohne Tk.

    python3 test_sequenz_kugeln.py
"""

import unittest

import sequenz

LISTE = """
0 0 0 0 0 0   fahrart=LIN radius=0 name=A
0.4 0 0 0 0 0 fahrart=LIN radius=0.3 name=B
0.4 0.2 0 0 0 0 fahrart=LIN radius=0.3 name=C
0.4 0.2 0.5 0 0 0 fahrart=LIN radius=0 name=D
"""


def befunde(kugeln_pruefen):
    punkte, lesefehler = sequenz.lies_liste(LISTE)
    assert not lesefehler, lesefehler
    folge = sequenz.Folge(punkte)
    return folge.pruefe(kugeln_pruefen=kugeln_pruefen)


class Kugeln(unittest.TestCase):

    def test_pilz_fehler(self):
        # Standard wie bisher: B -> C (0.2 m) mit 0.3 + 0.3 ist ein Fehler,
        # A -> B (0.4 m) mit 0 + 0.3 nicht.
        fehler = [b for b in befunde(True) if b.stufe == sequenz.FEHLER]
        self.assertEqual(len(fehler), 1)
        self.assertIn("B -> C: die Ueberschleifkugeln ueberlappen",
                      fehler[0].text)
        self.assertEqual(fehler[0].paar, (1, 2))
        # Der Standard ist die Pruefung.
        punkte, _ = sequenz.lies_liste(LISTE)
        self.assertEqual(sequenz.Folge(punkte).pruefe(), befunde(True))

    def test_intrinsic_hinweis(self):
        alle = befunde(False)
        self.assertEqual([b for b in alle if b.stufe == sequenz.FEHLER], [])
        hinweise = [b for b in alle if b.stufe == sequenz.HINWEIS
                    and "kein Fehler" in b.text]
        self.assertEqual(len(hinweise), 1)
        self.assertIn("B -> C: Radien 0.3000 + 0.3000 = 0.6000 m bei nur "
                      "0.2000 m Abstand", hinweise[0].text)
        self.assertEqual(hinweise[0].paar, (1, 2))

    def test_nutzlast_gleiche_regel(self):
        # Der Fehler von damals: pruefe(kugeln_pruefen=False) liess die
        # Folge durch, nutzlast() pruefte mit Kugeln und warf nach der IK.
        punkte, _ = sequenz.lies_liste(LISTE)
        # Seit 2.3 geht nur hinaus, was Achswerte traegt -- wie nach der
        # IK-Kette der Oberflaeche.
        for punkt in punkte:
            punkt.achsen = {"j1": 0.0}
        folge = sequenz.Folge(punkte)
        with self.assertRaisesRegex(ValueError, "ueberlappen"):
            folge.nutzlast(gruppe="arm", start={}, toleranz=0.01)
        last = folge.nutzlast(gruppe="arm", start={}, toleranz=0.01,
                              kugeln_pruefen=False)
        self.assertEqual([p["blend_radius"] for p in last["items"]],
                         [0.0, 0.3, 0.3, 0.0])

    def test_andere_pruefungen_bleiben(self):
        # Letzter Punkt mit Radius bleibt auch ohne Kugelpruefung ein Fehler.
        punkte, _ = sequenz.lies_liste(LISTE.replace(
            "fahrart=LIN radius=0 name=D", "fahrart=LIN radius=0.1 name=D"))
        fehler = sequenz.Folge(punkte).fehler(
            sequenz.Folge(punkte).pruefe(kugeln_pruefen=False))
        self.assertTrue(any("letzter Punkt traegt Radius" in b.text
                            for b in fehler), fehler)


if __name__ == "__main__":
    unittest.main(verbosity=1)
