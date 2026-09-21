#!/usr/bin/env python3
"""Pruefreihe fuer den Bahnrekorder in 2.25. Ohne Fenster, ohne ROS.

Geprueft werden zwei Methoden von UrdfViewer, beide ungebunden und mit einem
Platzhalter statt eines Fensters aufgerufen. Es entsteht kein Tk-Fenster,
also braucht die Reihe keinen Bildschirm und kein xvfb-run:

    python3 test_rekorder_2_25.py

Zwei Aussagen stehen zur Pruefung.

    _frame_snapshot liefert je Frame genau sechs Zahlen, drei Wege und drei
    Winkel, in der Reihenfolge der uebergebenen Namen. Bis 2.24 lieferte sie
    einen NameError, weil in der zweiten extend-Zeile der Name rpy stand.

    _record_tick bestellt sich in jedem Fall nach. Bis 2.24 stand das after
    am Ende des Rumpfes, und eine Ausnahme darin nahm den Takt mit.

Dazu die Sonden. Eine Pruefung, die eine wiedereingesetzte Fehlerstelle
nicht bemerkt, prueft nichts. Jede Sonde baut den alten Stand nach und
verlangt, dass die zugehoerige Pruefung faellt.
"""

import inspect
import io
import math
import sys
import types

import urdf_gui2_25 as gui


# ----------------------------------------------------------------------
# Platzhalter
# ----------------------------------------------------------------------

class KonventionStub:
    """Steht fuer konvention. Die Reihe prueft den Rekorder, nicht die Drehung.

    Feste Rueckgabe, damit ein Fehler in der Winkelrechnung die Aussage
    dieser Reihe nicht verwaschen kann. konvention.py hat eine eigene.
    """

    ABC = (0.11, 0.22, 0.33)

    @classmethod
    def to_xyz_abc(cls, matrix, name):
        return ((matrix[0][3], matrix[1][3], matrix[2][3]), cls.ABC)


class RecorderStub:
    def __init__(self, frames=(), active=True):
        self.konvention = "zyx"
        self.frame_names = list(frames)
        self.joint_names = ["a1"]
        self.active = active
        self.count = 0

    def sample(self, xyz, abc, values, frames):
        self.count += 1
        return True


class ViewerStub:
    """Genau so viel Fenster, wie die beiden Methoden anfassen."""

    def __init__(self, recorder=None):
        self.recorder = recorder if recorder is not None else RecorderStub()
        self.after_calls = []
        self.dropped = []
        self.logs = []
        self.raise_in_sample = None
        self.sampled = 0

    # von _record_tick benutzt
    def after(self, ms, func):
        self.after_calls.append((ms, func))

    def _record_tick(self):
        """Nur als Bestellgut. Der Takt reicht sich selbst an after weiter."""
        raise AssertionError("die Bestellung darf nicht selbst laufen")

    def _record_sample(self):
        if self.raise_in_sample is not None:
            raise self.raise_in_sample
        self.sampled += 1

    def _drop_record(self, reason):
        self.dropped.append(reason)

    def _log(self, text, level="info"):
        self.logs.append((level, text))


def matrix_bei(x, y, z):
    m = [[1.0, 0.0, 0.0, x],
         [0.0, 1.0, 0.0, y],
         [0.0, 0.0, 1.0, z],
         [0.0, 0.0, 0.0, 1.0]]
    return m


# ----------------------------------------------------------------------
# Pruefungen. Die zu pruefende Funktion wird hereingereicht, damit die
# Sonden dieselbe Pruefung gegen einen mutierten Stand fahren koennen.
# ----------------------------------------------------------------------

def pruefe_snapshot_laenge(snapshot):
    """Sechs Zahlen je Frame, in der Reihenfolge der Namen."""
    viewer = ViewerStub()
    namen = ["base", "link1", "tcp"]
    poses = {"base": matrix_bei(0.0, 0.0, 0.0),
             "link1": matrix_bei(0.1, 0.2, 0.3),
             "tcp": matrix_bei(0.4, 0.5, 0.6)}
    flach = snapshot(viewer, poses, namen)
    assert len(flach) == 6 * len(namen), \
        f"erwartet {6 * len(namen)} Zahlen, bekommen {len(flach)}"
    assert flach[6:9] == [0.1, 0.2, 0.3], \
        f"Weg des zweiten Frames falsch: {flach[6:9]}"
    assert tuple(flach[9:12]) == KonventionStub.ABC, \
        f"Winkel des zweiten Frames falsch: {flach[9:12]}"
    assert all(isinstance(v, float) for v in flach), "keine reinen Fliesskommazahlen"


def pruefe_snapshot_ohne_lage(snapshot):
    """Ein abgehaengter Frame kostet die Spalten nicht, er fuellt sie mit Null."""
    viewer = ViewerStub()
    namen = ["da", "weg"]
    poses = {"da": matrix_bei(1.0, 2.0, 3.0)}
    flach = snapshot(viewer, poses, namen)
    assert len(flach) == 12, f"erwartet 12 Zahlen, bekommen {len(flach)}"
    assert flach[6:] == [0.0] * 6, f"fehlender Frame nicht genullt: {flach[6:]}"


def pruefe_takt_nach_ausnahme(tick):
    """Der Takt ueberlebt eine Ausnahme in der Abtastung."""
    viewer = ViewerStub()
    viewer.raise_in_sample = ValueError("absichtlich")
    tick(viewer)
    assert viewer.after_calls, "nach einer Ausnahme wurde kein after bestellt"
    ms, func = viewer.after_calls[-1]
    assert ms == gui.RECORD_TICK_MS, f"falscher Takt bestellt: {ms}"
    assert viewer.dropped, "Puffer nach der Ausnahme nicht verworfen"


def pruefe_takt_im_normalfall(tick):
    """Ohne Ausnahme genau eine Abtastung und genau eine Bestellung."""
    viewer = ViewerStub()
    tick(viewer)
    assert viewer.sampled == 1, f"{viewer.sampled} Abtastungen statt einer"
    assert len(viewer.after_calls) == 1, \
        f"{len(viewer.after_calls)} Bestellungen statt einer"
    assert not viewer.dropped, f"ohne Fehler verworfen: {viewer.dropped}"


def pruefe_takt_ohne_log(tick):
    """Ein kaputtes Log darf den Takt auch nicht mitnehmen."""

    class OhneLog(ViewerStub):
        def _drop_record(self, reason):
            raise RuntimeError("auch das Verwerfen ist kaputt")

    viewer = OhneLog()
    viewer.raise_in_sample = ValueError("absichtlich")
    # Der letzte Ausweg im Takt ist traceback.print_exc. Der Ausdruck gehoert
    # zur Sache und nicht in die Ausgabe dieser Reihe, deshalb umgeleitet.
    still = io.StringIO()
    laut, sys.stderr = sys.stderr, still
    try:
        tick(viewer)
    finally:
        sys.stderr = laut
    assert viewer.after_calls, "bei kaputtem Verwerfen kein after bestellt"
    assert "auch das Verwerfen ist kaputt" in still.getvalue(), \
        "der letzte Ausweg hat nichts ausgegeben"


# ----------------------------------------------------------------------
# Sonden. Der alte Stand, wortgleich nachgebaut.
# ----------------------------------------------------------------------

def sonde_rpy(self, poses, names):
    """_frame_snapshot wie bis 2.24: der zweite extend nennt rpy."""
    flat = []
    for name in names:
        matrix = poses.get(name)
        if matrix is None:
            flat.extend((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            continue
        xyz, abc = gui.konvention.to_xyz_abc(matrix, self.recorder.konvention)
        flat.extend(xyz)
        flat.extend(rpy)                               # noqa: F821 -- Absicht
    return flat


def sonde_after_am_ende(self):
    """_record_tick wie bis 2.24: das after steht im Rumpf, nicht im finally."""
    self._record_sample()
    self.after(gui.RECORD_TICK_MS, self._record_tick)


# ----------------------------------------------------------------------
# Ablauf
# ----------------------------------------------------------------------

PRUEFUNGEN = [
    ("_frame_snapshot: sechs Zahlen je Frame",
     pruefe_snapshot_laenge, "_frame_snapshot"),
    ("_frame_snapshot: fehlende Lage wird genullt",
     pruefe_snapshot_ohne_lage, "_frame_snapshot"),
    ("_record_tick: Takt ueberlebt eine Ausnahme",
     pruefe_takt_nach_ausnahme, "_record_tick"),
    ("_record_tick: Normalfall genau einmal",
     pruefe_takt_im_normalfall, "_record_tick"),
    ("_record_tick: Takt ueberlebt ein kaputtes Verwerfen",
     pruefe_takt_ohne_log, "_record_tick"),
]

SONDEN = [
    ("rpy statt abc in _frame_snapshot",
     sonde_rpy, [pruefe_snapshot_laenge, pruefe_snapshot_ohne_lage]),
    ("after am Rumpfende statt im finally",
     sonde_after_am_ende, [pruefe_takt_nach_ausnahme, pruefe_takt_ohne_log]),
]


def pruefe_stand():
    """Sagt an, welche Datei geprueft wird, und dass sie 2.25 ist.

    Der Anlass ist ein Lauf, in dem die Reihe genau das meldete, was 2.25
    behebt: NameError auf rpy und ein Takt, der die Abtastung noch selbst
    enthaelt. Nicht die Reparatur war falsch, sondern die Datei daneben war
    noch aelter. Eine Pruefreihe, die nicht sagt, was sie prueft, verwechselt
    einen alten Stand mit einem Fehler im neuen.
    """
    print(f"geprueft wird {gui.__file__}")
    fehlt = [name for name in ("_record_sample", "_frame_snapshot", "_record_tick")
             if not hasattr(gui.UrdfViewer, name)]
    if fehlt:
        print(f"  ABBRUCH: dieser Stand kennt {', '.join(fehlt)} nicht -- "
              f"das ist 2.24 oder aelter, nicht 2.25")
        return False
    quelle = inspect.getsource(gui.UrdfViewer._frame_snapshot)
    if "extend(abc)" not in quelle:
        print("  ABBRUCH: _frame_snapshot schreibt nicht extend(abc) -- "
              "das ist 2.24 oder aelter, nicht 2.25")
        return False
    return True


def main():
    if not pruefe_stand():
        return 2

    gui.konvention = KonventionStub                    # nur in dieser Reihe

    echt = {"_frame_snapshot": gui.UrdfViewer._frame_snapshot,
            "_record_tick": gui.UrdfViewer._record_tick}

    fehler = 0

    print("Pruefungen")
    for name, pruefung, ziel in PRUEFUNGEN:
        try:
            pruefung(echt[ziel])
        except Exception as exc:
            print(f"  FEHLER  {name}: {exc}")
            fehler += 1
        else:
            print(f"  ok      {name}")

    print("Mutationssonden")
    for name, mutant, betroffen in SONDEN:
        gefallen = 0
        for pruefung in betroffen:
            try:
                pruefung(mutant)
            except Exception:
                gefallen += 1
        if gefallen:
            print(f"  ok      {name}: {gefallen} von {len(betroffen)} gefallen")
        else:
            print(f"  BLIND   {name}: keine Pruefung bemerkt die Mutation")
            fehler += 1

    print("bestanden" if not fehler else f"{fehler} Beanstandungen")
    return 1 if fehler else 0


if __name__ == "__main__":
    sys.exit(main())
