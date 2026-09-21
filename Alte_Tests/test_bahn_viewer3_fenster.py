#!/usr/bin/env python3
"""Fensterpruefung zu bahn_viewer3.py. Braucht eine Anzeige.

Aufruf:
    xvfb-run -a ./test_bahn_viewer3_fenster.py alt.csv neu.csv

Geprueft wird das, was die Logikpruefung nicht sehen kann: dass die geladene
Bahn wirklich gezeichnet wird, dass die Fusszeile die Konvention nennt und
dass die Dreibeine der Frames aus der Konvention der Datei kommen.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bahn_viewer3 as bv                              # noqa: E402

FEHLER = []


def pruefe(bedingung, text):
    print(f"  {'ok    ' if bedingung else 'FEHLER'} {text}")
    if not bedingung:
        FEHLER.append(text)


def hauptteil(dateien):
    app = bv.BahnViewer(None)
    app.update()
    for pfad in dateien:
        name = os.path.basename(pfad)
        print(f"\n{name}")
        app._load(pfad)
        app.update()
        track = app.track

        pruefe(track is not None and len(track) > 0, "Punkte geladen")
        pruefe(track.has_model, "Modell erkannt")

        info = app.info_var.get()
        pruefe(f"Winkel {track.konvention}" in info,
               f"Fusszeile nennt die Konvention: {info.split('   ')[-1]}")
        pruefe("ohne Modell" not in info, "Fusszeile meldet kein fehlendes Modell")

        marke = app.marker_var.get()
        erwartet = track.winkel_beschriftung[0]
        pruefe(erwartet in marke,
               f"Markerzeile beschriftet die Winkel mit {erwartet}")
        pruefe("R " not in marke.split("m   ")[-1],
               "keine alte R/P/Y-Beschriftung mehr")

        # Zeichnen: die Modellstriche muessen tatsaechlich entstehen.
        app.show_model.set(True)
        app.model_triads.set(True)
        items = app._model_items(0, False)
        striche = [i for i in items if i[0] == "line"]
        pruefe(len(striche) >= len(track.links),
               f"{len(striche)} Striche fuer {len(track.links)} Verbindungen")

        # Ein Dreibein eines Frames muss ein orthonormales Achsenkreuz sein.
        spalten = bv.rotation_from_abc(track.frame_pose(0, 0)[1], track.achsen)
        laengen = [sum(v * v for v in s) ** 0.5 for s in spalten]
        pruefe(all(abs(l - 1.0) < 1e-9 for l in laengen),
               "Dreibein des ersten Frames ist normiert")

        app._redraw()
        app.update()
        pruefe(len(app.canvas.find_all()) > 0, "Zeichenflaeche ist nicht leer")

    app.destroy()


if __name__ == "__main__":
    dateien = sys.argv[1:]
    if not dateien:
        print("Aufruf: xvfb-run -a ./test_bahn_viewer3_fenster.py datei.csv ...")
        sys.exit(2)
    hauptteil(dateien)
    print()
    if FEHLER:
        print(f"{len(FEHLER)} Pruefung(en) fehlgeschlagen")
        sys.exit(1)
    print("alle Fensterpruefungen bestanden")
