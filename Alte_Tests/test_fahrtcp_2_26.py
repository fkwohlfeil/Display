#!/usr/bin/env python3
"""Pruefreihe fuer die TCP-Zeile im Fahrfenster, 2.26. Ohne Fenster, ohne ROS.

    python3 test_fahrtcp_2_26.py

Vier Aussagen stehen zur Pruefung.

    _update_drive_tcp zeigt die Lage an, wenn es eine gibt, und sagt an,
    wenn es keine gibt. Drei Stufen, drei Stilnamen.

    Es wird nicht gerechnet, solange das Fahrfenster zu ist. Die Zeile
    haengt an _push_positions und damit am Fahrtakt mit 50 Hz -- eine
    Vorwaertskinematik je Takt fuer ein Fenster, das niemand offen hat,
    waere ein Preis ohne Gegenwert.

    Text und Faerbung stammen aus einer Rechnung. _pose_text bekommt die
    Lage durchgereicht, statt sie ein zweites Mal zu holen. Sonst stuenden
    Zahl und Farbe aus zwei Durchlaeufen nebeneinander.

    Die Zeile steht ausserhalb des rollenden Bereichs. Das ist die
    eigentliche Forderung -- immer sichtbar --, und sie ist eine Aussage
    ueber den Aufbau, nicht ueber einen Wert. Geprueft wird sie deshalb am
    Quelltext von _open_drive und nicht an einem Wert. Eine Pruefung am
    Quelltext ist schwaecher als eine am Bild; sie faellt aber genau dann,
    wenn jemand die Zeile in den Canvas schiebt, und darum geht es.
"""

import inspect
import io
import math
import sys

import urdf_gui2_26 as gui


# ----------------------------------------------------------------------
# Platzhalter
# ----------------------------------------------------------------------

class VarStub:
    def __init__(self):
        self.wert = ""

    def set(self, wert):
        self.wert = wert

    def get(self):
        return self.wert


class LabelStub:
    def __init__(self, exists=True):
        self.exists = exists
        self.style = None

    def winfo_exists(self):
        return self.exists

    def configure(self, **kwargs):
        self.style = kwargs.get("style", self.style)


class ViewerStub:
    """Genau so viel Fenster, wie _update_drive_tcp anfasst."""

    LAGE = ((0.4, -0.002, 0.731), (0.0, math.radians(90.0), 0.0))

    def __init__(self, tcp="tool0", pose=LAGE, label=None):
        self.tcp = tcp
        self.pose = pose
        self.drive_tcp_var = VarStub()
        self.drive_tcp_label = LabelStub() if label is None else label
        self.fk_aufrufe = 0

    def _tcp_link(self):
        return self.tcp

    def _tcp_pose(self):
        self.fk_aufrufe += 1
        return self.pose

    # die echten Methoden, ungebunden angehaengt
    _pose_text = gui.UrdfViewer._pose_text


# ----------------------------------------------------------------------
# Pruefungen
# ----------------------------------------------------------------------

def pruefe_zeile_zeigt_lage(update):
    """Normalfall: Name und Zahlen stehen da, Stil ist die ruhige Stufe."""
    viewer = ViewerStub()
    update(viewer)
    text = viewer.drive_tcp_var.get()
    assert text.startswith("tool0\n"), f"TCP-Name fehlt oder steht falsch: {text!r}"
    for achse in ("X", "Y", "Z", "A", "B", "C"):
        assert f"{achse} " in text, f"Spalte {achse} fehlt: {text!r}"
    assert "0.4000" in text, f"X-Wert nicht in der Zeile: {text!r}"
    assert gui.ANZEIGE.name in text, "die Drehkonvention steht nicht dabei"
    assert viewer.drive_tcp_label.style in ("DriveTcp.TLabel",
                                            "DriveTcpWarn.TLabel"), \
        f"unerwarteter Stil: {viewer.drive_tcp_label.style}"


def pruefe_ohne_tcp(update):
    """Kein Endeffektor: eine Ansage und die rote Stufe, keine Zahlen."""
    viewer = ViewerStub(tcp=None)
    update(viewer)
    text = viewer.drive_tcp_var.get()
    assert "Kein Endeffektor" in text, f"keine Ansage: {text!r}"
    assert viewer.drive_tcp_label.style == "DriveTcpBad.TLabel", \
        f"falsche Stufe: {viewer.drive_tcp_label.style}"
    assert viewer.fk_aufrufe == 0, "ohne TCP wurde trotzdem gerechnet"


def pruefe_ohne_lage(update):
    """TCP da, aber nicht an der Basis: ebenfalls die rote Stufe."""
    viewer = ViewerStub(pose=None)
    update(viewer)
    text = viewer.drive_tcp_var.get()
    assert "tool0" in text and "Basis" in text, f"keine Ansage: {text!r}"
    assert viewer.drive_tcp_label.style == "DriveTcpBad.TLabel", \
        f"falsche Stufe: {viewer.drive_tcp_label.style}"


def pruefe_geschlossenes_fenster(update):
    """Zu: nichts rechnen, nichts schreiben."""
    for label in (None, LabelStub(exists=False)):
        viewer = ViewerStub(label=label) if label is not None else ViewerStub()
        if label is None:
            viewer.drive_tcp_label = None
        update(viewer)
        assert viewer.fk_aufrufe == 0, \
            "bei geschlossenem Fahrfenster wurde die Kinematik gerechnet"
        assert viewer.drive_tcp_var.get() == "", \
            f"bei geschlossenem Fahrfenster geschrieben: {viewer.drive_tcp_var.get()!r}"


def pruefe_eine_rechnung(update):
    """Genau ein Durchlauf der Vorwaertskinematik je Fortschreibung."""
    viewer = ViewerStub()
    update(viewer)
    assert viewer.fk_aufrufe == 1, \
        f"{viewer.fk_aufrufe} Durchlaeufe statt einem -- Zahl und Farbe " \
        f"kaemen aus verschiedenen Rechnungen"


def pruefe_zeile_rollt_nicht(_unbenutzt=None):
    """Die Zeile haengt am Fenster, nicht am rollenden Inhalt.

    Am Quelltext geprueft: head wird auf window gegrided, das Label sitzt in
    head_inner, und der rollende Rahmen heisst inside. Steht das Label
    irgendwann in inside, faellt diese Zeile.
    """
    quelle = inspect.getsource(gui.UrdfViewer._open_drive)
    assert "head = self._card(window, row=0" in quelle, \
        "die TCP-Zeile liegt nicht als eigene Karte in Zeile 0 des Fensters"
    assert "self.drive_tcp_label = ttk.Label(\n            head_inner" in quelle, \
        "das Label sitzt nicht in head_inner"
    assert "canvas.create_window((0, 0), window=inside" in quelle, \
        "der rollende Rahmen heisst nicht mehr inside -- Pruefung veraltet"
    kopf = quelle.split("card = self._card(window, row=1")[0]
    assert "inside" not in kopf, \
        "die TCP-Zeile entsteht erst nach dem rollenden Rahmen"
    assert "window.rowconfigure(1, weight=1)" in quelle, \
        "das Gewicht liegt nicht auf dem rollenden Teil"


def pruefe_takt_ruft_an(_unbenutzt=None):
    """Jeder Achsstand geht ueber _push_positions und muss oben ankommen."""
    quelle = inspect.getsource(gui.UrdfViewer._push_positions)
    assert "_update_drive_tcp" in quelle, \
        "_push_positions zieht die TCP-Zeile nicht nach"
    quelle = inspect.getsource(gui.UrdfViewer._update_kinematics)
    assert "_update_drive_tcp" in quelle, \
        "_update_kinematics zieht die TCP-Zeile nicht nach"
    quelle = inspect.getsource(gui.UrdfViewer._motion_tick)
    assert "_push_positions" in quelle, \
        "der Fahrtakt geht nicht ueber _push_positions -- Pruefung veraltet"


# ----------------------------------------------------------------------
# Sonden
# ----------------------------------------------------------------------

def sonde_ohne_fensterpruefung(self):
    """Wie ohne die Pruefung auf ein offenes Fenster: rechnet immer."""
    tcp = self._tcp_link()
    pose = self._tcp_pose()
    label = self.drive_tcp_label
    if label is None or not label.winfo_exists():
        return
    if tcp is None or pose is None:
        self.drive_tcp_var.set("Kein Endeffektor gewaehlt")
        label.configure(style="DriveTcpBad.TLabel")
        return
    self.drive_tcp_var.set(f"{tcp}\n{self._pose_text(pad=True, pose=pose)}")
    label.configure(style="DriveTcp.TLabel")


def sonde_zweite_rechnung(self):
    """Wie ohne den Durchgriff: _pose_text holt die Lage selbst."""
    label = self.drive_tcp_label
    if label is None or not label.winfo_exists():
        return
    tcp = self._tcp_link()
    if tcp is None:
        self.drive_tcp_var.set("Kein Endeffektor gewaehlt")
        label.configure(style="DriveTcpBad.TLabel")
        return
    pose = self._tcp_pose()
    if pose is None:
        self.drive_tcp_var.set(f"TCP {tcp} haengt nicht an der Basis")
        label.configure(style="DriveTcpBad.TLabel")
        return
    self.drive_tcp_var.set(f"{tcp}\n{self._pose_text(pad=True)}")
    label.configure(style="DriveTcp.TLabel")


# ----------------------------------------------------------------------
# Ablauf
# ----------------------------------------------------------------------

PRUEFUNGEN = [
    ("Zeile zeigt die Lage", pruefe_zeile_zeigt_lage, True),
    ("ohne TCP eine Ansage", pruefe_ohne_tcp, True),
    ("ohne Lage eine Ansage", pruefe_ohne_lage, True),
    ("geschlossenes Fenster kostet nichts", pruefe_geschlossenes_fenster, True),
    ("genau eine Vorwaertskinematik", pruefe_eine_rechnung, True),
    ("Zeile rollt nicht weg", pruefe_zeile_rollt_nicht, False),
    ("Fahrtakt zieht die Zeile nach", pruefe_takt_ruft_an, False),
]

SONDEN = [
    ("Fensterpruefung entfernt",
     sonde_ohne_fensterpruefung, [pruefe_geschlossenes_fenster]),
    ("_pose_text holt die Lage selbst",
     sonde_zweite_rechnung, [pruefe_eine_rechnung]),
]


def pruefe_stand():
    print(f"geprueft wird {gui.__file__}")
    fehlt = [n for n in ("_update_drive_tcp", "_open_drive", "_push_positions")
             if not hasattr(gui.UrdfViewer, n)]
    if fehlt:
        print(f"  ABBRUCH: dieser Stand kennt {', '.join(fehlt)} nicht -- "
              f"das ist 2.25 oder aelter, nicht 2.26")
        return False
    if "pose" not in inspect.signature(gui.UrdfViewer._pose_text).parameters:
        print("  ABBRUCH: _pose_text nimmt keine fertige Lage entgegen -- "
              "das ist 2.25 oder aelter, nicht 2.26")
        return False
    return True


def main():
    if not pruefe_stand():
        return 2

    echt = gui.UrdfViewer._update_drive_tcp
    fehler = 0

    print("Pruefungen")
    for name, pruefung, braucht_funktion in PRUEFUNGEN:
        try:
            pruefung(echt) if braucht_funktion else pruefung()
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
