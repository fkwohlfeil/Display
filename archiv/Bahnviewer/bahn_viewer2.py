#!/usr/bin/env python3
"""Bahnansicht: eine CSV-Aufzeichnung des Bahnrekorders im 3D-Raum ansehen.

Das Werkzeug steht bewusst allein. Es kennt weder ROS noch die URDF, es liest
nur die CSV-Spalten. Damit laeuft es ohne gesourctes Environment, neben der
laufenden Oberflaeche und auch auf einem anderen Rechner.

Oberflaeche nach Skizze Bahnansicht 1:

    Kopf     schmale Toolbar mit Dateitasten, Ansichten und dem Dateinamen
    Mitte    der 3D-Raum, darueber schwebend die aufklappbaren Fenster
             (rechts oben Darstellung, rechts unten Ablauf), unten links
             die Umschalter fuer Schieben und Drehen
    Fuss     Kennwerte der Bahn und die Werte am Marker

Gezeichnet wird orthographisch mit eigener Projektion, also ohne OpenGL.
Unter WSL entfaellt damit der ganze Softwarerenderer.

Enthaelt die CSV die Lage aller Frames, laesst sich zusaetzlich das ganze
kinematische Modell einblenden: die Frames werden entlang der Kette verbunden
und bekommen je ein Dreibein. Punkte, Bahnlinie und Einfaerbung bleiben dem TCP
vorbehalten, damit der Blick nicht zerfaellt.

Aufruf:
    ./bahn_viewer2.py
    ./bahn_viewer2.py ~/Display/URDF_Speicher/bahn_20260812_101500.csv
"""

import argparse
import csv
import json
import math
import os
import sys
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk, filedialog, messagebox, colorchooser

try:                                                   # nur fuer den JPEG-Export
    from PIL import Image, ImageDraw, ImageFont
    HAVE_PIL = True
except ImportError:
    HAVE_PIL = False


# ---------- Farben, gleiche Palette wie die Bedienoberflaeche ----------
BG = "#eef1f5"
SURFACE = "#ffffff"
BORDER = "#d5dbe3"
TEXT = "#1f2933"
MUTED = "#6b7480"
FAINT = "#9aa3ad"
HEADING = "#eaeef3"
BUTTON = "#e3e8ef"
BUTTON_HOVER = "#d5dce6"
BUTTON_DOWN = "#c7cfdb"
ACCENT = "#2563c9"
OK_GREEN = "#1a7f37"
WARN = "#9a6700"
ALARM = "#d13438"

SCENE_BG = "#fbfcfd"                                   # Zeichenflaeche
GRID = "#e2e7ee"
GRID_MAIN = "#ccd3dc"
PATH = "#2563c9"
POINT = "#1f4f9e"
MARKER = "#d13438"
AXIS_X = "#c0392b"
AXIS_Y = "#1a7f37"
AXIS_Z = "#2563c9"

PAD = 10

# ---------- Zeichnen ----------
# Beim Ziehen mit der Maus wird eine schlanke Fassung gezeichnet: keine Punkte,
# keine Dreibeine, ausgeduennte Bahn. Beim Loslassen kommt das volle Bild.
FAST_MAX_POINTS = 1500
POINT_BUDGET = 4000                                    # mehr Ovale wird zaeh
SPEED_BUCKETS = 24                                     # Farbstufen der Einfaerbung

MODEL_LINK = "#516070"                                 # Glieder des Modells
MODEL_JOINT = "#33414f"                                # Frameursprung
GHOST = "#c3cad3"                                      # zurueckliegende Stellungen

VIEWS = {                                              # Azimut, Elevation in grad
    "Iso": (-55.0, 25.0),
    "XY (Draufsicht)": (-90.0, 89.9),
    "XZ (vorne)": (-90.0, 0.0),
    "YZ (seitlich)": (0.0, 0.0),
}

ROOM_COLORS = ["#2563c9", "#1a7f37", "#9a6700", "#8e44ad", "#c0392b", "#00838f"]


# ==========================================================================
# Kleine Helfer
# ==========================================================================

def clamp(value, low, high):
    return low if value < low else (high if value > high else value)


def hex_to_rgb(colour):
    colour = colour.lstrip("#")
    return tuple(int(colour[i:i + 2], 16) for i in (0, 2, 4))


def rgb_to_hex(rgb):
    return "#%02x%02x%02x" % tuple(int(clamp(v, 0, 255)) for v in rgb)


# Blau, Tuerkis, Gruen, Gelb, Rot. Bewusst nicht Grau nach Rot, damit auch
# kleine Unterschiede im mittleren Bereich noch auffallen.
_RAMP = [(0.00, (37, 99, 201)), (0.25, (0, 150, 170)), (0.50, (26, 140, 60)),
         (0.75, (220, 170, 20)), (1.00, (200, 50, 45))]


def ramp_colour(fraction):
    """Farbe der Skala fuer 0..1."""
    fraction = clamp(fraction, 0.0, 1.0)
    for (lo, c_lo), (hi, c_hi) in zip(_RAMP, _RAMP[1:]):
        if fraction <= hi:
            span = hi - lo
            k = 0.0 if span <= 0 else (fraction - lo) / span
            return rgb_to_hex([a + (b - a) * k for a, b in zip(c_lo, c_hi)])
    return rgb_to_hex(_RAMP[-1][1])


def nice_step(extent, target=8):
    """Rastermass 1, 2, 5 mal Zehnerpotenz, sodass etwa `target` Linien passen."""
    if extent <= 0:
        return 1.0
    raw = extent / float(target)
    exponent = math.floor(math.log10(raw))
    base = 10.0 ** exponent
    for factor in (1.0, 2.0, 5.0, 10.0):
        if raw <= factor * base:
            return factor * base
    return 10.0 * base


def rotation_from_rpy(rpy):
    """RPY (rad) -> die drei Spalten der Drehmatrix, wie in der URDF.

    Feste Achsen: R = Rz(yaw) @ Ry(pitch) @ Rx(roll). Die Spalten sind die
    Werkzeugachsen im Basissystem, genau das, was das Dreibein zeigen soll.
    """
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, sy * cp, -sp),
        (cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr),
        (cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr),
    )


# ==========================================================================
# Kamera: orthographische Projektion
# ==========================================================================

class Camera:
    """Blickrichtung ueber Azimut und Elevation, Zoom als Pixel je Meter."""

    def __init__(self):
        self.az = math.radians(-55.0)
        self.el = math.radians(25.0)
        self.scale = 400.0                             # px/m
        self.pan = [0.0, 0.0]                          # Verschiebung in px
        self.center = [0.0, 0.0, 0.0]                  # Weltpunkt in der Mitte
        self.width = 900
        self.height = 620

    def basis(self):
        """Rechtsvektor, Hochvektor und Blickrichtung als Einheitsvektoren."""
        ca, sa = math.cos(self.az), math.sin(self.az)
        ce, se = math.cos(self.el), math.sin(self.el)
        forward = (ce * ca, ce * sa, se)               # vom Ziel zur Kamera
        right = (-sa, ca, 0.0)
        up = (-se * ca, -se * sa, ce)
        return right, up, forward

    def project(self, point):
        right, up, _f = self.basis()
        return self._apply(point, right, up)

    def _apply(self, point, right, up):
        dx = point[0] - self.center[0]
        dy = point[1] - self.center[1]
        dz = point[2] - self.center[2]
        sx = (dx * right[0] + dy * right[1] + dz * right[2]) * self.scale
        sy = (dx * up[0] + dy * up[1] + dz * up[2]) * self.scale
        return (sx + self.width * 0.5 + self.pan[0],
                self.height * 0.5 - sy + self.pan[1])  # Bildschirm-y zeigt nach unten

    def project_many(self, points):
        """Viele Punkte auf einmal. Die Basis wird nur einmal gerechnet."""
        right, up, _f = self.basis()
        rx, ry, rz = right
        ux, uy, uz = up
        cx, cy, cz = self.center
        s = self.scale
        ox = self.width * 0.5 + self.pan[0]
        oy = self.height * 0.5 + self.pan[1]
        out = []
        for px, py, pz in points:
            dx, dy, dz = px - cx, py - cy, pz - cz
            out.append(((dx * rx + dy * ry + dz * rz) * s + ox,
                        oy - (dx * ux + dy * uy + dz * uz) * s))
        return out

    def depth(self, point):
        """Groesser heisst naeher an der Kamera. Fuer die Malerreihenfolge."""
        _r, _u, forward = self.basis()
        return sum((point[i] - self.center[i]) * forward[i] for i in range(3))

    def zoom_at(self, factor, sx, sy):
        """Zoomen, ohne dass der Punkt unter dem Zeiger wegwandert."""
        self.pan[0] += (sx - self.width * 0.5 - self.pan[0]) * (1.0 - factor)
        self.pan[1] += (sy - self.height * 0.5 - self.pan[1]) * (1.0 - factor)
        self.scale = clamp(self.scale * factor, 1.0, 500000.0)

    def orbit(self, d_az_px, d_el_px):
        self.az += math.radians(d_az_px * 0.4)
        self.el = clamp(self.el + math.radians(-d_el_px * 0.4),
                        math.radians(-89.9), math.radians(89.9))


# ==========================================================================
# Daten
# ==========================================================================

T_NAMES = ("t_s", "t", "time", "zeit", "zeit_s")
X_NAMES = ("x_m", "x")
Y_NAMES = ("y_m", "y")
Z_NAMES = ("z_m", "z")
R_NAMES = ("roll_deg", "roll", "r_deg")
P_NAMES = ("pitch_deg", "pitch", "p_deg")
W_NAMES = ("yaw_deg", "yaw", "y_deg", "gier_deg")


def parse_number(text):
    """Punkt oder Komma als Dezimaltrenner, Tausenderpunkte werden geschluckt."""
    text = (text or "").strip()
    if not text:
        return 0.0
    try:
        return float(text)
    except ValueError:
        return float(text.replace(".", "").replace(",", "."))


FRAME_SUFFIX = ("_x_m", "_y_m", "_z_m", "_roll_deg", "_pitch_deg", "_yaw_deg")


def find_frames(header, used):
    """Frameblocks im Kopf finden.

    Ein Block heisst <name>_x_m bis <name>_yaw_deg. Gesucht wird ueber die
    Endungen, nicht ueber feste Positionen; fehlt auch nur eine der sechs
    Spalten, wird der Block uebergangen statt halb gelesen.
    """
    lowered = [h.strip().lower() for h in header]
    candidates = []
    for index, name in enumerate(lowered):
        if index in used or not name.endswith("_x_m") or name == "x_m":
            continue
        candidates.append(name[:-len("_x_m")])

    frames = []
    for stem in candidates:
        columns = []
        for suffix in FRAME_SUFFIX:
            wanted = stem + suffix
            if wanted not in lowered:
                columns = []
                break
            columns.append(lowered.index(wanted))
        if columns:
            frames.append((header[columns[0]].strip()[:-len("_x_m")], columns))
    return frames


def find_column(header, names):
    lowered = [h.strip().lower() for h in header]
    for name in names:
        if name in lowered:
            return lowered.index(name)
    return None


class Track:
    """Die eingelesene Bahn. Rechnet Geschwindigkeit und Grenzen einmal aus.

    Neben der TCP-Bahn koennen die Lagen aller Frames enthalten sein. Sie
    liegen je Punkt als flache Liste, sechs Zahlen je Frame, in der Reihenfolge
    von `frame_names`.
    """

    def __init__(self, path, times, points, angles, joints, joint_names,
                 frame_names=(), frames=(), parents=None, robot_name=""):
        self.path = path
        self.times = times                             # s
        self.points = points                           # [(x, y, z)] in m
        self.angles = angles                           # [(r, p, y)] in rad
        self.joints = joints                           # [(werte...)]
        self.joint_names = joint_names
        self.frame_names = list(frame_names)
        self.frames = frames                           # [flach je Punkt] oder []
        self.parents = parents or {}                   # {kind: elter}
        self.robot_name = robot_name

        self.speeds = self._speeds()
        self.length = self._length()
        self.lo, self.hi = self._bounds()
        self.v_min = min(self.speeds) if self.speeds else 0.0
        self.v_max = max(self.speeds) if self.speeds else 0.0
        # Ein einzelner Sprung, etwa nach einem Klick auf Go, hat rechnerisch
        # eine riesige Geschwindigkeit und wuerde die Farbskala zusammen-
        # druecken. Eingefaerbt wird deshalb ueber die mittleren 90 Prozent,
        # Ausreisser laufen einfach in die Endfarbe.
        self.v_lo, self.v_hi = self._percentiles(0.05, 0.95)
        # Grenzen einschliesslich Modell, damit das Einpassen den ganzen
        # Roboter zeigt und nicht nur die Spitze.
        self.model_lo, self.model_hi = self._model_bounds()
        self.links = self._links()

    def _percentiles(self, low, high):
        if not self.speeds:
            return 0.0, 0.0
        ordered = sorted(self.speeds)
        last = len(ordered) - 1
        return ordered[int(low * last)], ordered[int(math.ceil(high * last))]

    # -- Ableitungen -------------------------------------------------------

    def _speeds(self):
        speeds = [0.0] * len(self.points)
        for i in range(1, len(self.points)):
            dt = self.times[i] - self.times[i - 1]
            step = math.dist(self.points[i], self.points[i - 1])
            speeds[i] = step / dt if dt > 1e-9 else 0.0
        if len(speeds) > 1:
            speeds[0] = speeds[1]                      # erster Punkt hat keinen Vorgaenger
        return speeds

    def _length(self):
        return sum(math.dist(a, b) for a, b in zip(self.points, self.points[1:]))

    def _bounds(self):
        if not self.points:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        lo = [min(p[i] for p in self.points) for i in range(3)]
        hi = [max(p[i] for p in self.points) for i in range(3)]
        return tuple(lo), tuple(hi)

    def _model_bounds(self):
        """Huellbox ueber alle Frames aller Punkte."""
        if not self.frames:
            return self.lo, self.hi
        lo = list(self.lo)
        hi = list(self.hi)
        for flat in self.frames:
            for base in range(0, len(flat), 6):
                for axis in range(3):
                    value = flat[base + axis]
                    lo[axis] = min(lo[axis], value)
                    hi[axis] = max(hi[axis], value)
        return tuple(lo), tuple(hi)

    def _links(self):
        """Verbindungen als Indexpaare in `frame_names`.

        Ohne Kettenangabe im Kopf wird die Reihenfolge der Spalten als Kette
        gelesen. Das trifft die uebliche serielle Anordnung und ist besser als
        gar keine Verbindung, wird aber in der Fusszeile kenntlich gemacht.
        """
        order = {name: i for i, name in enumerate(self.frame_names)}
        if self.parents:
            pairs = []
            for child, parent in self.parents.items():
                if child in order and parent in order:
                    pairs.append((order[parent], order[child]))
            if pairs:
                return sorted(pairs)
        return [(i, i + 1) for i in range(len(self.frame_names) - 1)]

    @property
    def has_model(self):
        return bool(self.frames) and len(self.frame_names) > 1

    @property
    def chain_known(self):
        return bool(self.parents)

    @property
    def duration(self):
        return self.times[-1] - self.times[0] if len(self.times) > 1 else 0.0

    def frame_pose(self, index, frame):
        """(xyz, rpy) eines Frames an einem Abtastpunkt."""
        base = frame * 6
        flat = self.frames[index]
        return (flat[base], flat[base + 1], flat[base + 2]), \
               (flat[base + 3], flat[base + 4], flat[base + 5])

    def frame_points(self, index):
        flat = self.frames[index]
        return [(flat[b], flat[b + 1], flat[b + 2])
                for b in range(0, len(flat), 6)]

    def __len__(self):
        return len(self.points)

    # -- Einlesen ----------------------------------------------------------

    @classmethod
    def load(cls, path):
        """CSV lesen. Spalten werden ueber den Kopf gesucht, nicht ueber die
        Position: so bleibt die Datei lesbar, wenn spaeter Spalten dazukommen."""
        with open(path, "r", newline="") as handle:
            sample = handle.readline()
            handle.seek(0)
            delimiter = max(";,\t", key=sample.count)
            rows = list(csv.reader(handle, delimiter=delimiter))

        rows = [r for r in rows if r and any(c.strip() for c in r)]
        # Kommentarzeilen vor dem Spaltenkopf tragen Roboternamen und Kette.
        meta = [r for r in rows if r[0].strip().startswith("#")]
        rows = [r for r in rows if not r[0].strip().startswith("#")]
        if len(rows) < 2:
            raise ValueError("Die Datei enthaelt keine Datenzeilen")

        robot_name = ""
        parents = {}
        for row in meta:
            tag = row[0].strip().lower()
            if tag == "#roboter" and len(row) > 1:
                robot_name = row[1].strip()
            elif tag == "#kette" and len(row) > 2:
                parents[row[1].strip()] = row[2].strip()

        header = rows[0]
        index = {
            "t": find_column(header, T_NAMES),
            "x": find_column(header, X_NAMES),
            "y": find_column(header, Y_NAMES),
            "z": find_column(header, Z_NAMES),
            "roll": find_column(header, R_NAMES),
            "pitch": find_column(header, P_NAMES),
            "yaw": find_column(header, W_NAMES),
        }
        missing = [k for k in ("x", "y", "z") if index[k] is None]
        if missing:
            raise ValueError("Spalten fehlen: " + ", ".join(missing)
                             + "\nGefunden: " + ", ".join(header))

        used = {v for v in index.values() if v is not None}
        frame_blocks = find_frames(header, used)
        for _name, columns in frame_blocks:
            used.update(columns)
        frame_names = [name for name, _c in frame_blocks]

        joint_cols = [i for i, name in enumerate(header)
                      if i not in used and name.strip()]
        joint_names = [header[i].strip() for i in joint_cols]

        times, points, angles, joints, frames = [], [], [], [], []
        for line, row in enumerate(rows[1:], start=2):
            try:
                point = tuple(parse_number(row[index[k]]) for k in ("x", "y", "z"))
                angle = tuple(math.radians(parse_number(row[index[k]]))
                              if index[k] is not None and index[k] < len(row) else 0.0
                              for k in ("roll", "pitch", "yaw"))
                stamp = (parse_number(row[index["t"]])
                         if index["t"] is not None and index["t"] < len(row)
                         else float(len(times)))
                values = tuple(parse_number(row[i]) if i < len(row) else 0.0
                               for i in joint_cols)
                flat = []
                for _name, columns in frame_blocks:
                    for position, column in enumerate(columns):
                        raw = parse_number(row[column]) if column < len(row) else 0.0
                        flat.append(math.radians(raw) if position >= 3 else raw)
            except (ValueError, IndexError) as exc:
                raise ValueError(f"Zeile {line} ist nicht lesbar: {exc}")
            times.append(stamp)
            points.append(point)
            angles.append(angle)
            joints.append(values)
            if flat:
                frames.append(tuple(flat))

        return cls(path, times, points, angles, joints, joint_names,
                   frame_names, frames, parents, robot_name)


class Room:
    """Quader als Arbeitsraum oder Sperrbereich. Achsparallel."""

    def __init__(self, name, lo, hi, colour=ROOM_COLORS[0], visible=True):
        self.name = name
        self.lo = [min(a, b) for a, b in zip(lo, hi)]
        self.hi = [max(a, b) for a, b in zip(lo, hi)]
        self.colour = colour
        self.visible = visible

    def corners(self):
        x0, y0, z0 = self.lo
        x1, y1, z1 = self.hi
        return [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]

    FACES = ((0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
             (2, 3, 7, 6), (1, 2, 6, 5), (0, 3, 7, 4))
    EDGES = ((0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7))

    def as_dict(self):
        return {"name": self.name, "lo": self.lo, "hi": self.hi,
                "colour": self.colour, "visible": self.visible}

    @classmethod
    def from_dict(cls, data):
        return cls(data.get("name", "Raum"), data["lo"], data["hi"],
                   data.get("colour", ROOM_COLORS[0]), data.get("visible", True))


# ==========================================================================
# Anzeigeliste und die zwei Ausgaben
# ==========================================================================
# Erst wird eine Liste einfacher Zeichenbefehle gebaut, dann gibt sie entweder
# das Canvas aus oder Pillow. So sieht das JPEG genauso aus wie das Fenster,
# nur in hoeherer Aufloesung, und der Zeichencode steht nur einmal da.

def render_canvas(canvas, items):
    canvas.delete("all")
    for item in items:
        kind = item[0]
        if kind == "line":
            _k, coords, colour, width, dash = item
            canvas.create_line(*coords, fill=colour, width=max(1, int(round(width))),
                               dash=dash, capstyle="round")
        elif kind == "poly":
            _k, coords, fill, outline, stipple = item
            canvas.create_polygon(*coords, fill=fill, outline=outline,
                                  stipple=stipple, width=1)
        elif kind == "oval":
            _k, x, y, radius, fill, outline = item
            canvas.create_oval(x - radius, y - radius, x + radius, y + radius,
                               fill=fill, outline=outline or "")
        elif kind == "text":
            _k, x, y, text, colour, anchor, size, bold = item
            canvas.create_text(x, y, text=text, fill=colour, anchor=anchor,
                               font=("TkDefaultFont", size,
                                     "bold" if bold else "normal"))


def render_image(items, width, height, background=SURFACE):
    """Dieselbe Liste als Pillow-Bild. Stipple wird zu echter Transparenz."""
    image = Image.new("RGB", (int(width), int(height)), hex_to_rgb(background))
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)
    soft = ImageDraw.Draw(overlay)

    def font_for(size, bold):
        for name in (("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"),
                     "LiberationSans-Regular.ttf"):
            try:
                return ImageFont.truetype(name, size)
            except OSError:
                continue
        return ImageFont.load_default()

    for item in items:
        kind = item[0]
        if kind == "line":
            _k, coords, colour, width, dash = item
            pairs = list(zip(coords[0::2], coords[1::2]))
            if len(pairs) > 1:
                draw.line(pairs, fill=colour, width=max(1, int(width)), joint="curve")
        elif kind == "poly":
            _k, coords, fill, outline, stipple = item
            pairs = list(zip(coords[0::2], coords[1::2]))
            if len(pairs) < 3:
                continue
            if stipple and fill:
                # Die Tk-Muster grob in Deckkraft uebersetzen, damit Bild und
                # Fenster gleich aussehen.
                alpha = {"gray12": 34, "gray25": 66, "gray50": 128}.get(stipple, 66)
                soft.polygon(pairs, fill=hex_to_rgb(fill) + (alpha,))
            elif fill:
                draw.polygon(pairs, fill=fill)
            if outline:
                draw.line(pairs + [pairs[0]], fill=outline, width=1)
        elif kind == "oval":
            _k, x, y, radius, fill, outline = item
            draw.ellipse([x - radius, y - radius, x + radius, y + radius],
                         fill=fill, outline=outline or None)
        elif kind == "text":
            _k, x, y, text, colour, anchor, size, bold = item
            pil_anchor = {"nw": "la", "ne": "ra", "sw": "ld", "se": "rd",
                          "w": "lm", "e": "rm", "center": "mm",
                          "n": "ma", "s": "md"}.get(anchor, "la")
            draw.text((x, y), text, fill=colour, font=font_for(size, bold),
                      anchor=pil_anchor)

    image = Image.alpha_composite(image.convert("RGBA"), overlay)
    return image.convert("RGB")


# ==========================================================================
# Aufklappbares Fenster, schwebt ueber der Zeichenflaeche
# ==========================================================================

class FoldPanel(tk.Frame):
    """Karte mit Kopfzeile und Klapptaste, wie in der Skizze."""

    def __init__(self, master, title, opened=True, width=232):
        super().__init__(master, background=SURFACE, bd=0, highlightthickness=1,
                         highlightbackground=BORDER, highlightcolor=BORDER)
        self.opened = opened
        head = tk.Frame(self, background=HEADING)
        head.pack(fill="x")
        tk.Label(head, text=title, background=HEADING, foreground=MUTED,
                 font=("TkDefaultFont", 9, "bold"), padx=10, pady=5).pack(side="left")
        self.toggle_button = tk.Button(
            head, text="\u2013", command=self.toggle, background=HEADING,
            foreground=MUTED, activebackground=BUTTON_HOVER, relief="flat", bd=0,
            highlightthickness=0, width=2, cursor="hand2",
            font=("TkDefaultFont", 10, "bold"))
        self.toggle_button.pack(side="right", padx=(0, 4))
        self.body = tk.Frame(self, background=SURFACE, padx=12, pady=10,
                             width=width)
        if opened:
            self.body.pack(fill="both", expand=True)

    def toggle(self):
        self.opened = not self.opened
        if self.opened:
            self.body.pack(fill="both", expand=True)
            self.toggle_button.configure(text="\u2013")
        else:
            self.body.pack_forget()
            self.toggle_button.configure(text="+")


# ==========================================================================
# Hauptfenster
# ==========================================================================

class BahnViewer(tk.Tk):

    def __init__(self, csv_path=None):
        super().__init__()
        self.title("Bahnansicht")
        self.geometry("1280x820")
        self.minsize(900, 600)
        self.configure(background=BG)

        self.track = None
        self.rooms = []
        self.cam = Camera()
        self.mode = "rotate"                           # oder "pan"
        self.dragging = False
        self.drag_from = None
        self.redraw_job = None
        self.playing = False
        self.play_from = None                          # (Uhrzeit, Bahnzeit)
        self.marker = 0
        # Vervielfacher fuer alles, was in Pixeln bemessen ist: Strichstaerken,
        # Schriftgroessen, Punktradien. Beim Export auf 2 gesetzt, damit das
        # Bild in doppelter Aufloesung genauso aussieht und nicht feiner wird.
        self._px = 1.0

        self.file_var = tk.StringVar(value="keine Datei geladen")
        self.info_var = tk.StringVar(value="")
        self.marker_var = tk.StringVar(value="")
        self.show_line = tk.BooleanVar(value=True)
        self.show_points = tk.BooleanVar(value=False)
        self.show_triads = tk.BooleanVar(value=False)
        self.show_grid = tk.BooleanVar(value=True)
        self.show_model = tk.BooleanVar(value=True)
        self.model_triads = tk.BooleanVar(value=True)
        self.show_ghosts = tk.BooleanVar(value=False)
        self.ghost_every = tk.IntVar(value=40)
        self.show_rooms = tk.BooleanVar(value=True)
        self.trail_only = tk.BooleanVar(value=False)
        self.time_mode = tk.BooleanVar(value=True)
        self.colour_mode = tk.StringVar(value="Geschwindigkeit")
        self.connect_mode = tk.StringVar(value="Direkt")
        self.line_width = tk.IntVar(value=2)
        self.point_size = tk.IntVar(value=3)
        self.triad_every = tk.IntVar(value=25)
        self.triad_size = tk.IntVar(value=30)          # in px
        self.tempo = tk.StringVar(value="1x")
        self.panels_shown = True

        self._setup_fonts()
        self._setup_style()
        self._build_toolbar()
        self._build_scene_area()
        self._build_panels()
        self._build_foot()
        self._bind_mouse()

        if csv_path:
            self._load(csv_path)
        else:
            self._redraw()
        self._play_tick()

    # ----------------------------------------------------------------------
    # Aussehen
    # ----------------------------------------------------------------------

    def _setup_fonts(self):
        available = set(tkfont.families())
        base = tkfont.nametofont("TkDefaultFont")
        for family in ("Ubuntu", "Noto Sans", "DejaVu Sans", "Segoe UI", "Cantarell"):
            if family in available:
                base.configure(family=family)
                break
        base.configure(size=10)
        tkfont.nametofont("TkTextFont").configure(**base.actual())
        mono = tkfont.nametofont("TkFixedFont")
        for family in ("Ubuntu Mono", "DejaVu Sans Mono", "Noto Sans Mono",
                       "Liberation Mono", "Consolas"):
            if family in available:
                mono.configure(family=family)
                break
        mono.configure(size=10)

    def _setup_style(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", background=BG, foreground=TEXT,
                        fieldbackground=SURFACE, bordercolor=BORDER,
                        focuscolor=ACCENT)
        style.configure("TFrame", background=BG)
        style.configure("TLabel", background=BG, foreground=TEXT)
        style.configure("Card.TFrame", background=SURFACE)
        style.configure("Card.TLabel", background=SURFACE, foreground=TEXT)
        style.configure("Muted.TLabel", background=SURFACE, foreground=MUTED,
                        font=("TkDefaultFont", 9))
        style.configure("Foot.TLabel", background=BG, foreground=MUTED,
                        font=("TkFixedFont", 10))
        style.configure("TButton", background=BUTTON, foreground=TEXT,
                        bordercolor=BORDER, lightcolor=BUTTON, darkcolor=BUTTON,
                        relief="flat", padding=(10, 5), focusthickness=0)
        style.map("TButton", background=[("pressed", BUTTON_DOWN),
                                         ("active", BUTTON_HOVER),
                                         ("disabled", BG)],
                  foreground=[("disabled", FAINT)])
        style.configure("TCheckbutton", background=SURFACE, foreground=TEXT,
                        focuscolor=SURFACE, font=("TkDefaultFont", 9))
        style.map("TCheckbutton", background=[("active", SURFACE)],
                  indicatorcolor=[("selected", ACCENT)])
        style.configure("TCombobox", fieldbackground=SURFACE, background=BUTTON,
                        arrowcolor=TEXT, bordercolor=BORDER, padding=3)
        style.configure("TScale", background=SURFACE, troughcolor=BUTTON,
                        bordercolor=BORDER, lightcolor=ACCENT, darkcolor=ACCENT)
        style.configure("Treeview", background=SURFACE, fieldbackground=SURFACE,
                        foreground=TEXT, rowheight=24, borderwidth=0)
        style.configure("Treeview.Heading", background=HEADING, foreground=MUTED,
                        relief="flat", font=("TkDefaultFont", 9, "bold"))
        style.configure("TSeparator", background=BORDER)

    # ----------------------------------------------------------------------
    # Aufbau
    # ----------------------------------------------------------------------

    def _build_toolbar(self):
        """Schmaler Streifen ueber die ganze Breite, rechts der Dateiname."""
        bar = tk.Frame(self, background=SURFACE, highlightthickness=1,
                       highlightbackground=BORDER)
        bar.pack(side="top", fill="x", padx=PAD, pady=(PAD, 0))
        inner = tk.Frame(bar, background=SURFACE, padx=8, pady=6)
        inner.pack(fill="x")

        ttk.Button(inner, text="CSV laden...", command=self._choose).pack(side="left")
        ttk.Button(inner, text="Neu laden", command=self._reload).pack(
            side="left", padx=(6, 0))
        ttk.Separator(inner, orient="vertical").pack(side="left", fill="y",
                                                     padx=10, pady=2)

        ttk.Label(inner, text="Ansicht", style="Muted.TLabel").pack(side="left")
        view = ttk.Combobox(inner, values=list(VIEWS), state="readonly", width=16)
        view.set("Iso")
        view.bind("<<ComboboxSelected>>", lambda e: self._set_view(view.get()))
        view.pack(side="left", padx=(6, 0))
        ttk.Button(inner, text="Einpassen", command=self._fit).pack(
            side="left", padx=(6, 0))

        ttk.Separator(inner, orient="vertical").pack(side="left", fill="y",
                                                     padx=10, pady=2)
        ttk.Button(inner, text="Raeume...", command=self._open_rooms).pack(side="left")
        ttk.Button(inner, text="Bild speichern...", command=self._export_image).pack(
            side="left", padx=(6, 0))
        ttk.Button(inner, text="Toolbox", command=self._toggle_panels).pack(
            side="left", padx=(6, 0))

        tk.Label(inner, textvariable=self.file_var, background=SURFACE,
                 foreground=MUTED, font=("TkFixedFont", 10)).pack(side="right")

    def _build_scene_area(self):
        holder = tk.Frame(self, background=SURFACE, highlightthickness=1,
                          highlightbackground=BORDER)
        holder.pack(side="top", fill="both", expand=True, padx=PAD, pady=PAD)
        self.holder = holder

        self.canvas = tk.Canvas(holder, background=SCENE_BG, highlightthickness=0,
                                bd=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", self._on_resize)

        # Schieben und Drehen unten links, wie in der Skizze.
        tools = tk.Frame(holder, background=SURFACE, highlightthickness=1,
                         highlightbackground=BORDER)
        tools.place(relx=0.0, rely=1.0, x=12, y=-12, anchor="sw")
        self.mode_buttons = {}
        for key, label in (("pan", "Schieben"), ("rotate", "Drehen")):
            button = tk.Button(tools, text=label, width=9, relief="flat", bd=0,
                               highlightthickness=0, cursor="hand2",
                               font=("TkDefaultFont", 9, "bold"),
                               command=lambda k=key: self._set_mode(k))
            button.pack(side="left", padx=2, pady=2)
            self.mode_buttons[key] = button
        ttk.Separator(tools, orient="vertical").pack(side="left", fill="y",
                                                     padx=4, pady=4)
        for label, factor in (("\u2212", 1 / 1.25), ("+", 1.25)):
            tk.Button(tools, text=label, width=3, relief="flat", bd=0,
                      background=BUTTON, activebackground=BUTTON_HOVER,
                      highlightthickness=0, cursor="hand2",
                      font=("TkDefaultFont", 10, "bold"),
                      command=lambda f=factor: self._zoom_center(f)).pack(
                side="left", padx=2, pady=2)
        self._set_mode("rotate")

    def _build_panels(self):
        """Zwei schwebende Fenster rechts, oben Darstellung, unten Ablauf."""
        self.panel_view = FoldPanel(self.holder, "DARSTELLUNG")
        self.panel_run = FoldPanel(self.holder, "ABLAUF")
        self._fill_view_panel(self.panel_view.body)
        self._fill_run_panel(self.panel_run.body)
        self._place_panels()

    def _place_panels(self):
        self.panel_view.place(relx=1.0, rely=0.0, x=-12, y=12, anchor="ne")
        self.panel_run.place(relx=1.0, rely=1.0, x=-12, y=-12, anchor="se")

    def _fill_view_panel(self, body):
        def check(text, variable):
            ttk.Checkbutton(body, text=text, variable=variable,
                            command=self._redraw).pack(anchor="w", pady=1)

        check("Bahnlinie", self.show_line)
        row = tk.Frame(body, background=SURFACE)
        row.pack(fill="x", pady=(0, 4))
        ttk.Label(row, text="Verfahren", style="Muted.TLabel").pack(side="left")
        # Nur "Direkt" ist hinterlegt. Der Platz fuer Spline oder Kreisbogen
        # ist damit schon vorgesehen.
        connect = ttk.Combobox(row, values=["Direkt"], textvariable=self.connect_mode,
                               state="readonly", width=10)
        connect.pack(side="right")

        self._slider_row(body, "Liniendicke", self.line_width, 1, 6)
        check("Punkte", self.show_points)
        self._slider_row(body, "Punktgroesse", self.point_size, 1, 10)
        check("Dreibein", self.show_triads)
        self._slider_row(body, "jeder n-te Punkt", self.triad_every, 1, 200)
        self._slider_row(body, "Dreibeinlaenge", self.triad_size, 8, 90)

        ttk.Separator(body, orient="horizontal").pack(fill="x", pady=8)
        # Das Modell haengt an den Frame-Spalten der CSV. Ohne sie bleiben die
        # Schalter stehen, aber wirkungslos; der Fuss sagt warum.
        check("Modell", self.show_model)
        check("Dreibein je Frame", self.model_triads)
        check("Modell als Schatten", self.show_ghosts)
        self._slider_row(body, "Schatten alle n", self.ghost_every, 5, 300)

        ttk.Separator(body, orient="horizontal").pack(fill="x", pady=8)
        ttk.Label(body, text="Einfaerbung", style="Muted.TLabel").pack(anchor="w")
        colour = ttk.Combobox(
            body, values=["einfarbig", "Geschwindigkeit", "Zeit", "Hoehe Z"],
            textvariable=self.colour_mode, state="readonly", width=18)
        colour.pack(fill="x", pady=(2, 6))
        colour.bind("<<ComboboxSelected>>", lambda e: self._redraw())

        check("Bodengitter", self.show_grid)
        check("Raeume", self.show_rooms)

    def _fill_run_panel(self, body):
        row = tk.Frame(body, background=SURFACE)
        row.pack(fill="x")
        self.play_button = tk.Button(
            row, text="Start", width=8, relief="flat", bd=0, cursor="hand2",
            background=OK_GREEN, foreground="white", activebackground="#256628",
            activeforeground="white", font=("TkDefaultFont", 10, "bold"),
            command=self._toggle_play)
        self.play_button.pack(side="left")
        ttk.Button(row, text="Anfang", command=lambda: self._set_marker(0)).pack(
            side="left", padx=(6, 0))

        row2 = tk.Frame(body, background=SURFACE)
        row2.pack(fill="x", pady=(8, 0))
        ttk.Label(row2, text="Tempo", style="Muted.TLabel").pack(side="left")
        tempo = ttk.Combobox(row2, values=["0.25x", "0.5x", "1x", "2x", "5x", "10x"],
                             textvariable=self.tempo, state="readonly", width=7)
        tempo.pack(side="right")

        ttk.Checkbutton(body, text="zeitlicher Ablauf", variable=self.time_mode
                        ).pack(anchor="w", pady=(6, 0))
        ttk.Checkbutton(body, text="nur bis Marker zeichnen",
                        variable=self.trail_only, command=self._redraw).pack(
            anchor="w")

        self.marker_scale = ttk.Scale(body, from_=0, to=0, orient="horizontal",
                                      command=self._on_scale)
        self.marker_scale.pack(fill="x", pady=(8, 2))
        self.step_var = tk.StringVar(value="Punkt 0 / 0")
        ttk.Label(body, textvariable=self.step_var, style="Muted.TLabel").pack(
            anchor="w")

    def _slider_row(self, parent, label, variable, low, high):
        row = tk.Frame(parent, background=SURFACE)
        row.pack(fill="x", pady=(0, 2))
        ttk.Label(row, text=label, style="Muted.TLabel").pack(side="left")
        value = tk.Label(row, text=str(variable.get()), background=SURFACE,
                         foreground=TEXT, font=("TkFixedFont", 9), width=4,
                         anchor="e")
        value.pack(side="right")

        def moved(raw):
            variable.set(int(round(float(raw))))
            value.configure(text=str(variable.get()))
            self._redraw()

        scale = ttk.Scale(parent, from_=low, to=high, orient="horizontal",
                          command=moved)
        scale.set(variable.get())
        scale.pack(fill="x", pady=(0, 6))

    def _build_foot(self):
        foot = tk.Frame(self, background=BG)
        foot.pack(side="bottom", fill="x", padx=PAD + 4, pady=(0, 8))
        ttk.Label(foot, textvariable=self.info_var, style="Foot.TLabel").pack(
            side="left")
        ttk.Label(foot, textvariable=self.marker_var, style="Foot.TLabel").pack(
            side="right")

    # ----------------------------------------------------------------------
    # Maus und Tasten
    # ----------------------------------------------------------------------

    def _bind_mouse(self):
        self.canvas.bind("<ButtonPress-1>", self._press)
        self.canvas.bind("<B1-Motion>", self._drag)
        self.canvas.bind("<ButtonRelease-1>", self._release)
        # Rechte und mittlere Taste schieben immer, unabhaengig vom Modus.
        for button in ("2", "3"):
            self.canvas.bind(f"<ButtonPress-{button}>", self._press_pan)
            self.canvas.bind(f"<B{button}-Motion>", self._drag_pan)
            self.canvas.bind(f"<ButtonRelease-{button}>", self._release)
        self.canvas.bind("<MouseWheel>", self._wheel)              # Windows, macOS
        self.canvas.bind("<Button-4>", self._wheel)                # X11
        self.canvas.bind("<Button-5>", self._wheel)
        self.bind("<Key>", self._key)

    def _set_mode(self, mode):
        self.mode = mode
        for key, button in self.mode_buttons.items():
            active = key == mode
            button.configure(background=ACCENT if active else BUTTON,
                             foreground="white" if active else TEXT,
                             activebackground=ACCENT if active else BUTTON_HOVER,
                             activeforeground="white" if active else TEXT)

    def _press(self, event):
        self.drag_from = (event.x, event.y)
        self.drag_pan = self.mode == "pan"
        self.dragging = True

    def _press_pan(self, event):
        self.drag_from = (event.x, event.y)
        self.drag_pan = True
        self.dragging = True

    def _drag(self, event):
        if self.drag_from is None:
            return
        dx = event.x - self.drag_from[0]
        dy = event.y - self.drag_from[1]
        self.drag_from = (event.x, event.y)
        if self.drag_pan:
            self.cam.pan[0] += dx
            self.cam.pan[1] += dy
        else:
            self.cam.orbit(dx, dy)
        self._redraw(fast=True)

    def _drag_pan(self, event):
        self.drag_pan = True
        self._drag(event)

    def _release(self, _event):
        self.dragging = False
        self.drag_from = None
        self._redraw()

    def _wheel(self, event):
        step = 1.15
        up = getattr(event, "num", 0) == 4 or getattr(event, "delta", 0) > 0
        self.cam.zoom_at(step if up else 1.0 / step, event.x, event.y)
        self._redraw(fast=True)
        self._schedule_full()

    def _zoom_center(self, factor):
        self.cam.zoom_at(factor, self.cam.width * 0.5, self.cam.height * 0.5)
        self._redraw()

    def _key(self, event):
        if event.keysym == "space":
            self._toggle_play()
        elif event.keysym in ("Left", "Right") and self.track:
            self._set_marker(self.marker + (1 if event.keysym == "Right" else -1))
        elif event.char in ("f", "F"):
            self._fit()

    def _on_resize(self, event):
        self.cam.width, self.cam.height = event.width, event.height
        self._redraw()

    def _schedule_full(self):
        """Nach dem Zoomen einmal sauber nachzeichnen, aber nicht bei jedem Rad."""
        if self.redraw_job is not None:
            self.after_cancel(self.redraw_job)
        self.redraw_job = self.after(120, self._redraw_full)

    def _redraw_full(self):
        self.redraw_job = None
        self._redraw()

    def _toggle_panels(self):
        self.panels_shown = not self.panels_shown
        if self.panels_shown:
            self._place_panels()
        else:
            self.panel_view.place_forget()
            self.panel_run.place_forget()

    # ----------------------------------------------------------------------
    # Datei
    # ----------------------------------------------------------------------

    def _choose(self):
        path = filedialog.askopenfilename(
            title="Bahn-CSV waehlen",
            filetypes=[("CSV", "*.csv"), ("Alle Dateien", "*.*")])
        if path:
            self._load(path)

    def _reload(self):
        if self.track:
            self._load(self.track.path)

    def _load(self, path):
        try:
            track = Track.load(path)
        except (OSError, ValueError) as exc:
            messagebox.showerror("Laden", f"{path}\n\n{exc}")
            return
        if not len(track):
            messagebox.showwarning("Laden", "Die Datei enthaelt keine Punkte.")
            return

        self.track = track
        self.marker = 0
        self.playing = False
        self.play_button.configure(text="Start", background=OK_GREEN)
        label = os.path.basename(path)
        if track.robot_name:
            label = f"{track.robot_name}   {label}"
        self.file_var.set(label)
        self.marker_scale.configure(to=max(0, len(track) - 1))
        self.marker_scale.set(0)
        self._fit()
        self._update_info()

    def _update_info(self):
        if not self.track:
            self.info_var.set("keine Bahn geladen")
            self.step_var.set("Punkt 0 / 0")
            self.marker_var.set("")
            return
        track = self.track
        text = (f"{len(track)} Punkte   {track.duration:.1f} s   "
                f"Weg {track.length * 1000.0:.1f} mm   "
                f"v {track.v_min * 1000.0:.0f} .. {track.v_max * 1000.0:.0f} mm/s"
                f"   (Farbe bis {track.v_hi * 1000.0:.0f})")
        if track.has_model:
            text += f"   {len(track.frame_names)} Frames"
            if not track.chain_known:
                # Ohne #kette im Kopf ist die Verbindung nur geraten.
                text += " (Kette aus Spaltenfolge)"
        else:
            text += "   ohne Modell"
        self.info_var.set(text)
        self.step_var.set(f"Punkt {self.marker + 1} / {len(track)}")

        point = track.points[self.marker]
        angle = track.angles[self.marker]
        text = (f"t {track.times[self.marker]:7.2f} s   "
                f"X {point[0]:7.4f} Y {point[1]:7.4f} Z {point[2]:7.4f} m   "
                f"R {math.degrees(angle[0]):7.2f} P {math.degrees(angle[1]):7.2f} "
                f"Y {math.degrees(angle[2]):7.2f} deg   "
                f"v {track.speeds[self.marker] * 1000.0:6.1f} mm/s")
        if track.joint_names:
            values = track.joints[self.marker]
            text += "   " + "  ".join(f"{n} {v:.2f}" for n, v
                                      in zip(track.joint_names, values))
        self.marker_var.set(text)

    # ----------------------------------------------------------------------
    # Ansicht
    # ----------------------------------------------------------------------

    def _set_view(self, name):
        az, el = VIEWS[name]
        self.cam.az, self.cam.el = math.radians(az), math.radians(el)
        self._redraw()

    def _fit(self):
        """Kamera auf Bahn und Raeume einpassen."""
        boxes = []
        if self.track:
            # Mit Modell die Huellbox ueber alle Frames, sonst steht der
            # Roboter halb ausserhalb des Bildes.
            if self.show_model.get() and self.track.has_model:
                boxes.append((self.track.model_lo, self.track.model_hi))
            else:
                boxes.append((self.track.lo, self.track.hi))
        boxes.extend((room.lo, room.hi) for room in self.rooms if room.visible)
        if not boxes:
            self.cam.center = [0.0, 0.0, 0.0]
            self.cam.scale = 300.0
            self.cam.pan = [0.0, 0.0]
            self._redraw()
            return

        lo = [min(b[0][i] for b in boxes) for i in range(3)]
        hi = [max(b[1][i] for b in boxes) for i in range(3)]
        self.cam.center = [(a + b) * 0.5 for a, b in zip(lo, hi)]
        extent = max(max(b - a for a, b in zip(lo, hi)), 1e-3)
        span = min(self.cam.width, self.cam.height) * 0.72
        self.cam.scale = clamp(span / extent, 1.0, 500000.0)
        self.cam.pan = [0.0, 0.0]
        self._redraw()

    # ----------------------------------------------------------------------
    # Ablauf
    # ----------------------------------------------------------------------

    def _toggle_play(self):
        if not self.track:
            return
        self.playing = not self.playing
        if self.playing:
            if self.marker >= len(self.track) - 1:
                self.marker = 0
            self.play_from = (time.monotonic(), self.track.times[self.marker])
            self.play_button.configure(text="Pause", background=WARN)
        else:
            self.play_button.configure(text="Start", background=OK_GREEN)

    def _tempo_factor(self):
        try:
            return float(self.tempo.get().rstrip("x"))
        except ValueError:
            return 1.0

    def _play_tick(self):
        """Eigener Takt fuer den Ablauf, etwa 30 Hz."""
        if self.playing and self.track and len(self.track) > 1:
            factor = self._tempo_factor()
            if self.time_mode.get():
                # Echte Zeit aus der Spalte t_s, mit Tempofaktor gestreckt.
                started, base = self.play_from
                target = base + (time.monotonic() - started) * factor
                index = self.marker
                while (index < len(self.track) - 1
                       and self.track.times[index + 1] <= target):
                    index += 1
            else:
                # Ohne Zeitbezug: die ganze Bahn in zehn Sekunden durchlaufen.
                per_tick = max(1, int(len(self.track) / (10.0 / factor) / 30.0))
                index = self.marker + per_tick
            if index >= len(self.track) - 1:
                index = len(self.track) - 1
                self.playing = False
                self.play_button.configure(text="Start", background=OK_GREEN)
            if index != self.marker:
                self._set_marker(index)
        self.after(33, self._play_tick)

    def _on_scale(self, raw):
        if not self.track:
            return
        index = int(round(float(raw)))
        if index != self.marker:
            self.playing = False
            self.play_button.configure(text="Start", background=OK_GREEN)
            self._set_marker(index, from_scale=True)

    def _set_marker(self, index, from_scale=False):
        if not self.track:
            return
        self.marker = int(clamp(index, 0, len(self.track) - 1))
        if not from_scale:
            self.marker_scale.set(self.marker)
        self._update_info()
        self._redraw(fast=self.playing)

    # ----------------------------------------------------------------------
    # Raeume
    # ----------------------------------------------------------------------

    def _open_rooms(self):
        RoomDialog(self)

    # ----------------------------------------------------------------------
    # Szene bauen
    # ----------------------------------------------------------------------

    def _redraw(self, fast=False):
        items = self._scene(fast=fast, width=self.cam.width, height=self.cam.height)
        render_canvas(self.canvas, items)

    def _scene(self, fast, width, height):
        """Die vollstaendige Anzeigeliste. Reihenfolge ist die Malerreihenfolge."""
        items = []
        if self.show_rooms.get():
            items += self._room_items(fast)
        if self.show_grid.get():
            items += self._grid_items()
        items += self._axis_items()

        if self.track:
            if not fast and self.show_ghosts.get() and self.track.has_model:
                items += self._ghost_items()
            items += self._path_items(fast)
            if not fast:
                if self.show_points.get():
                    items += self._point_items()
                if self.show_triads.get():
                    items += self._triad_items()
            if self.show_model.get() and self.track.has_model:
                items += self._model_items(self.marker, fast)
            items += self._marker_items()
            if not fast and self.colour_mode.get() != "einfarbig":
                items += self._legend_items(width, height)
        else:
            items.append(("text", width * 0.5, height * 0.5,
                          "CSV ueber die Toolbar laden", FAINT, "center",
                          int(12 * self._px), False))
        return items

    # -- Gitter und Achsen -------------------------------------------------

    def _grid_extent(self):
        """Bereich und Rastermass fuer das Bodengitter in der Ebene Z = 0."""
        if self.track:
            lo, hi = self.track.lo, self.track.hi
            extent = max(hi[0] - lo[0], hi[1] - lo[1], 0.05)
            cx = (lo[0] + hi[0]) * 0.5
            cy = (lo[1] + hi[1]) * 0.5
        else:
            extent, cx, cy = 1.0, 0.0, 0.0
        step = nice_step(extent * 1.6)
        half = math.ceil(extent * 0.9 / step) + 1
        x0 = math.floor(cx / step) * step
        y0 = math.floor(cy / step) * step
        return step, half, x0, y0

    def _grid_items(self):
        step, half, x0, y0 = self._grid_extent()
        items = []
        low_x, high_x = x0 - half * step, x0 + half * step
        low_y, high_y = y0 - half * step, y0 + half * step
        for i in range(-half, half + 1):
            gx = x0 + i * step
            gy = y0 + i * step
            for a, b, at_zero in (((gx, low_y, 0.0), (gx, high_y, 0.0),
                                   abs(gx) < step * 1e-6),
                                  ((low_x, gy, 0.0), (high_x, gy, 0.0),
                                   abs(gy) < step * 1e-6)):
                pa, pb = self.cam.project(a), self.cam.project(b)
                items.append(("line", [pa[0], pa[1], pb[0], pb[1]],
                              GRID_MAIN if at_zero else GRID, self._px, None))
        return items

    def _axis_items(self):
        """Achsenkreuz im Ursprung, Laenge gleich einem Rastermass."""
        step, _half, _x0, _y0 = self._grid_extent()
        origin = self.cam.project((0.0, 0.0, 0.0))
        items = []
        for vector, colour, label in (((step, 0, 0), AXIS_X, "X"),
                                      ((0, step, 0), AXIS_Y, "Y"),
                                      ((0, 0, step), AXIS_Z, "Z")):
            end = self.cam.project(vector)
            items.append(("line", [origin[0], origin[1], end[0], end[1]],
                          colour, 2 * self._px, None))
            items.append(("text", end[0] + 6 * self._px, end[1] - 6 * self._px,
                          label, colour, "w", int(10 * self._px), True))
        items.append(("text", origin[0] + 6 * self._px, origin[1] + 8 * self._px,
                      f"{step:g} m", FAINT, "nw", int(8 * self._px), False))
        return items

    # -- Bahn --------------------------------------------------------------

    def _visible_range(self):
        """Bis wohin gezeichnet wird. Mit `trail_only` nur bis zum Marker."""
        end = self.marker + 1 if self.trail_only.get() else len(self.track)
        return max(end, 2)

    def _fraction(self, index):
        """Wert 0..1 fuer die Einfaerbung des Punktes."""
        track = self.track
        mode = self.colour_mode.get()
        if mode == "Geschwindigkeit":
            span = track.v_hi - track.v_lo
            return 0.5 if span <= 1e-12 else (track.speeds[index] - track.v_lo) / span
        if mode == "Zeit":
            span = track.duration
            return 0.5 if span <= 1e-9 else (track.times[index] - track.times[0]) / span
        if mode == "Hoehe Z":
            span = track.hi[2] - track.lo[2]
            return 0.5 if span <= 1e-12 else (track.points[index][2] - track.lo[2]) / span
        return 0.0

    def _path_items(self, fast):
        """Bahnlinie. Bei Einfaerbung wird in Farbstufen zusammengefasst.

        Ein Canvas-Linienzug kann nur eine Farbe haben. Statt je Abschnitt ein
        eigenes Element zu bauen, werden benachbarte Abschnitte gleicher
        Farbstufe zu einem Zug verkettet. Bei ruhigem Verlauf bleiben so
        wenige Dutzend Elemente statt vieler Tausend.
        """
        if not self.show_line.get():
            return []
        end = self._visible_range()
        points = self.track.points[:end]
        step = 1
        if fast and len(points) > FAST_MAX_POINTS:
            step = len(points) // FAST_MAX_POINTS + 1
        indices = list(range(0, len(points), step))
        if indices[-1] != len(points) - 1:
            indices.append(len(points) - 1)
        if len(indices) < 2:
            return []

        screen = self.cam.project_many([points[i] for i in indices])
        width = max(1, self.line_width.get()) * self._px
        if self.colour_mode.get() == "einfarbig" or fast:
            flat = []
            for x, y in screen:
                flat += [x, y]
            return [("line", flat, PATH, width, None)]

        buckets = [min(SPEED_BUCKETS - 1,
                       int(self._fraction(i) * SPEED_BUCKETS)) for i in indices]
        items = []
        run_start = 0
        for position in range(1, len(indices)):
            last = position == len(indices) - 1
            if buckets[position] != buckets[run_start] or last:
                stop = position if not last else position + 1
                flat = []
                for x, y in screen[run_start:stop]:
                    flat += [x, y]
                if len(flat) >= 4:
                    colour = ramp_colour((buckets[run_start] + 0.5) / SPEED_BUCKETS)
                    items.append(("line", flat, colour, width, None))
                run_start = position - 1           # ein Punkt Ueberlappung, keine Luecke
        return items

    def _point_items(self):
        end = self._visible_range()
        count = end
        step = max(1, count // POINT_BUDGET + (1 if count > POINT_BUDGET else 0))
        indices = list(range(0, end, step))
        screen = self.cam.project_many([self.track.points[i] for i in indices])
        radius = max(1, self.point_size.get()) * self._px
        single = self.colour_mode.get() == "einfarbig"
        return [("oval", x, y, radius,
                 POINT if single else ramp_colour(self._fraction(i)), "")
                for (x, y), i in zip(screen, indices)]

    def _triad_items(self):
        """Dreibein an jedem n-ten Punkt. Laenge in Pixeln, also zoomfest."""
        end = self._visible_range()
        every = max(1, self.triad_every.get())
        length = self.triad_size.get() * self._px / self.cam.scale   # px -> m
        items = []
        for index in range(0, end, every):
            items += self._triad_at(index, length, self._px)
        return items

    def _triad_at(self, index, length, width):
        base = self.track.points[index]
        columns = rotation_from_rpy(self.track.angles[index])
        origin = self.cam.project(base)
        items = []
        for column, colour in zip(columns, (AXIS_X, AXIS_Y, AXIS_Z)):
            tip = (base[0] + column[0] * length,
                   base[1] + column[1] * length,
                   base[2] + column[2] * length)
            sx, sy = self.cam.project(tip)
            items.append(("line", [origin[0], origin[1], sx, sy], colour,
                          width, None))
        return items

    def _model_items(self, index, fast, colour=None, width=None):
        """Das kinematische Modell an einem Abtastpunkt.

        Gezeichnet werden die Verbindungen entlang der Kette und je Frame ein
        kleines Dreibein. Punkte, Bahnlinie und Einfaerbung bleiben bewusst dem
        TCP vorbehalten: hingen sie an jedem Frame, waere im Bild nicht mehr zu
        erkennen, welche Kurve die eigentliche Bahn ist.
        """
        track = self.track
        px = self._px
        positions = track.frame_points(index)
        screen = self.cam.project_many(positions)
        link_colour = colour or MODEL_LINK
        line_width = width if width is not None else 3 * px

        items = []
        for parent, child in track.links:
            if parent >= len(screen) or child >= len(screen):
                continue
            a, b = screen[parent], screen[child]
            items.append(("line", [a[0], a[1], b[0], b[1]], link_colour,
                          line_width, None))
        if colour is None:                             # Schatten bleiben schlicht
            for x, y in screen:
                items.append(("oval", x, y, 2.5 * px, MODEL_JOINT, ""))
            if self.model_triads.get() and not fast:
                length = self.triad_size.get() * 0.7 * px / self.cam.scale
                for frame in range(len(track.frame_names)):
                    items += self._frame_triad(index, frame, length, px)
        return items

    def _frame_triad(self, index, frame, length, width):
        """Dreibein eines beliebigen Frames, nicht nur des TCP."""
        base, rpy = self.track.frame_pose(index, frame)
        columns = rotation_from_rpy(rpy)
        origin = self.cam.project(base)
        items = []
        for column, colour in zip(columns, (AXIS_X, AXIS_Y, AXIS_Z)):
            tip = (base[0] + column[0] * length,
                   base[1] + column[1] * length,
                   base[2] + column[2] * length)
            sx, sy = self.cam.project(tip)
            items.append(("line", [origin[0], origin[1], sx, sy], colour,
                          width, None))
        return items

    def _ghost_items(self):
        """Zurueckliegende Stellungen blass im Hintergrund.

        Zeigt auf einen Blick, wie sich der Arm umgestellt hat, auch dort wo
        der TCP kaum eine Strecke zurueckgelegt hat.
        """
        end = self._visible_range()
        every = max(5, self.ghost_every.get())
        items = []
        for index in range(0, end, every):
            if index == self.marker:
                continue
            items += self._model_items(index, fast=True, colour=GHOST,
                                       width=1.5 * self._px)
        return items

    def _marker_items(self):
        """Aktueller Punkt, sein Dreibein und ein Lot auf die Bodenebene."""
        point = self.track.points[self.marker]
        sx, sy = self.cam.project(point)
        foot = self.cam.project((point[0], point[1], 0.0))
        items = [("line", [sx, sy, foot[0], foot[1]], FAINT, self._px, (2, 3)),
                 ("oval", foot[0], foot[1], 2 * self._px, FAINT, "")]
        items += self._triad_at(
            self.marker,
            self.triad_size.get() * 1.4 * self._px / self.cam.scale, 2 * self._px)
        items.append(("oval", sx, sy,
                      max(4, self.point_size.get() + 2) * self._px,
                      MARKER, "#ffffff"))
        return items

    def _legend_items(self, width, height):
        """Farbskala unten links, damit die Einfaerbung ablesbar bleibt."""
        mode = self.colour_mode.get()
        clipped = False
        if mode == "Geschwindigkeit":
            low, high, unit = self.track.v_lo * 1000.0, self.track.v_hi * 1000.0, "mm/s"
            clipped = self.track.v_max > self.track.v_hi * 1.05
        elif mode == "Zeit":
            low, high, unit = 0.0, self.track.duration, "s"
        else:
            low, high, unit = self.track.lo[2], self.track.hi[2], "m"

        px = self._px
        x0, y0 = 16 * px, height - 96 * px
        bar_width, bar_height = 130 * px, 12 * px
        title = f"{mode} [{unit}]"
        items = [("text", x0, y0 - 6 * px, title, MUTED, "sw", int(9 * px), True)]
        steps = 26
        for i in range(steps):
            x = x0 + bar_width * i / steps
            items.append(("poly",
                          [x, y0, x + bar_width / steps + 1, y0,
                           x + bar_width / steps + 1, y0 + bar_height,
                           x, y0 + bar_height],
                          ramp_colour(i / (steps - 1.0)), "", ""))
        items.append(("text", x0, y0 + bar_height + 3 * px, f"{low:.4g}", MUTED,
                      "nw", int(8 * px), False))
        # Bei gekappter Skala steht ein Zeichen davor, sonst waere der
        # Endwert als Hoechstwert misszuverstehen.
        items.append(("text", x0 + bar_width, y0 + bar_height + 3 * px,
                      (">" if clipped else "") + f"{high:.4g}", MUTED,
                      "ne", int(8 * px), False))
        return items

    # -- Raeume ------------------------------------------------------------

    def _room_items(self, fast):
        """Quader als offene Kiste: nur die abgewandten Waende werden getoent.

        Wuerden alle sechs Flaechen gefuellt, laege ueber der ganzen Szene ein
        Schleier und die Bahn im Inneren waere kaum noch zu erkennen. Gefuellt
        wird deshalb nur, was von der Kamera wegzeigt.
        """
        items = []
        for room in self.rooms:
            if not room.visible:
                continue
            corners = room.corners()
            screen = self.cam.project_many(corners)
            middle = [(a + b) * 0.5 for a, b in zip(room.lo, room.hi)]
            _r, _u, forward = self.cam.basis()

            if not fast:
                for face in room.FACES:
                    centre = [sum(corners[i][k] for i in face) / 4.0
                              for k in range(3)]
                    outward = [c - m for c, m in zip(centre, middle)]
                    if sum(o * f for o, f in zip(outward, forward)) >= 0.0:
                        continue                       # zeigt zur Kamera, offen lassen
                    flat = []
                    for i in face:
                        flat += [screen[i][0], screen[i][1]]
                    items.append(("poly", flat, room.colour, "", "gray12"))

            for a, b in room.EDGES:
                items.append(("line", [screen[a][0], screen[a][1],
                                       screen[b][0], screen[b][1]],
                              room.colour, self._px, None))

            # Beschriftung an die im Bild oberste Ecke, nicht in die Mitte:
            # dort liegt sonst die Bahn darunter.
            top = min(screen, key=lambda p: p[1])
            label_y = max(top[1] - 6 * self._px, 12 * self._px)   # nicht abschneiden
            items.append(("text", top[0], label_y, room.name,
                          room.colour, "s", int(9 * self._px), True))
        return items

    # ----------------------------------------------------------------------
    # Bild speichern
    # ----------------------------------------------------------------------

    def _export_image(self):
        if not HAVE_PIL:
            self._export_postscript()
            return
        path = filedialog.asksaveasfilename(
            title="Bild speichern",
            initialfile=self._image_name(".jpg"),
            defaultextension=".jpg",
            filetypes=[("JPEG", "*.jpg"), ("PNG", "*.png")])
        if not path:
            return

        # Doppelte Aufloesung: Massstab, Groesse und Verschiebung mitziehen,
        # dann steht exakt derselbe Ausschnitt im Bild.
        factor = 2
        saved = (self.cam.width, self.cam.height, self.cam.scale, list(self.cam.pan))
        try:
            self.cam.width *= factor
            self.cam.height *= factor
            self.cam.scale *= factor
            self.cam.pan = [v * factor for v in self.cam.pan]
            self._px = float(factor)
            items = self._scene(fast=False, width=self.cam.width,
                                height=self.cam.height)
            image = render_image(items, self.cam.width, self.cam.height, SURFACE)
        finally:
            self.cam.width, self.cam.height, self.cam.scale, self.cam.pan = saved
            self._px = 1.0

        try:
            if path.lower().endswith(".png"):
                image.save(path)
            else:
                image.save(path, "JPEG", quality=92, subsampling=0)
        except OSError as exc:
            messagebox.showerror("Bild", f"Schreiben fehlgeschlagen:\n{exc}")
            return
        messagebox.showinfo("Bild", f"Gespeichert:\n{path}")

    def _export_postscript(self):
        """Ohne Pillow bleibt der eingebaute PostScript-Weg des Canvas."""
        path = filedialog.asksaveasfilename(
            title="Bild speichern (PostScript)",
            initialfile=self._image_name(".eps"),
            defaultextension=".eps",
            filetypes=[("PostScript", "*.eps")])
        if not path:
            return
        self.canvas.postscript(file=path, colormode="color")
        messagebox.showinfo(
            "Bild", f"Als PostScript gespeichert:\n{path}\n\n"
            "Fuer JPEG direkt aus dem Werkzeug:  pip install pillow")

    def _image_name(self, suffix):
        stem = (os.path.splitext(os.path.basename(self.track.path))[0]
                if self.track else "bahn")
        return stem + suffix


# ==========================================================================
# Raeume verwalten
# ==========================================================================

class RoomDialog(tk.Toplevel):
    """Liste der Quader, Felder darunter. Speichern als JSON neben der CSV."""

    FIELDS = [("x0", "X min"), ("x1", "X max"), ("y0", "Y min"),
              ("y1", "Y max"), ("z0", "Z min"), ("z1", "Z max")]

    def __init__(self, app):
        super().__init__(app)
        self.app = app
        self.title("Raeume")
        self.configure(background=BG)
        self.transient(app)
        self.colour = ROOM_COLORS[0]
        self.entries = {}

        card = tk.Frame(self, background=SURFACE, highlightthickness=1,
                        highlightbackground=BORDER)
        card.pack(fill="both", expand=True, padx=PAD, pady=PAD)
        inner = tk.Frame(card, background=SURFACE, padx=16, pady=14)
        inner.pack(fill="both", expand=True)

        self.tree = ttk.Treeview(inner, columns=("bereich",), height=7,
                                 selectmode="browse")
        self.tree.heading("#0", text="Name", anchor="w")
        self.tree.heading("bereich", text="Bereich [m]", anchor="w")
        self.tree.column("#0", width=150)
        self.tree.column("bereich", width=330, anchor="w")
        self.tree.grid(row=0, column=0, columnspan=6, sticky="nsew", pady=(0, 12))
        self.tree.bind("<<TreeviewSelect>>", lambda e: self._fill_from_selection())
        self.tree.bind("<Double-1>", lambda e: self._toggle_visible())

        name_row = tk.Frame(inner, background=SURFACE)
        name_row.grid(row=1, column=0, columnspan=6, sticky="w", pady=(0, 8))
        tk.Label(name_row, text="Name", background=SURFACE, foreground=MUTED,
                 font=("TkDefaultFont", 9)).pack(side="left")
        self.name_entry = tk.Entry(name_row, width=22, relief="flat",
                                   background=SURFACE, highlightthickness=1,
                                   highlightbackground=BORDER, highlightcolor=ACCENT)
        self.name_entry.pack(side="left", padx=(8, 12))
        self.colour_button = tk.Button(name_row, text="Farbe", width=8,
                                       relief="flat", bd=0, cursor="hand2",
                                       command=self._pick_colour)
        self.colour_button.pack(side="left")
        self._show_colour()

        for index, (key, label) in enumerate(self.FIELDS):
            row, column = 2 + index // 3, index % 3
            holder = tk.Frame(inner, background=SURFACE)
            holder.grid(row=row, column=column, sticky="w", pady=3, padx=(0, 14))
            tk.Label(holder, text=label, background=SURFACE, foreground=MUTED,
                     width=6, anchor="w", font=("TkDefaultFont", 9)).pack(side="left")
            entry = tk.Entry(holder, width=9, justify="right", relief="flat",
                             background=SURFACE, highlightthickness=1,
                             highlightbackground=BORDER, highlightcolor=ACCENT,
                             font=("TkFixedFont", 10))
            entry.insert(0, "0.0")
            entry.pack(side="left")
            self.entries[key] = entry

        buttons = tk.Frame(inner, background=SURFACE)
        buttons.grid(row=4, column=0, columnspan=6, sticky="w", pady=(14, 0))
        for text, command in (("Hinzufuegen", self._add),
                              ("Aendern", self._update),
                              ("Loeschen", self._delete),
                              ("Sichtbar", self._toggle_visible),
                              ("Aus Bahn", self._from_track)):
            ttk.Button(buttons, text=text, command=command).pack(side="left",
                                                                 padx=(0, 6))

        files = tk.Frame(inner, background=SURFACE)
        files.grid(row=5, column=0, columnspan=6, sticky="w", pady=(10, 0))
        ttk.Button(files, text="Laden...", command=self._load).pack(side="left")
        ttk.Button(files, text="Speichern...", command=self._save).pack(
            side="left", padx=(6, 0))
        tk.Label(files, text="Doppelklick schaltet die Sichtbarkeit",
                 background=SURFACE, foreground=FAINT,
                 font=("TkDefaultFont", 9)).pack(side="left", padx=(16, 0))

        inner.rowconfigure(0, weight=1)
        inner.columnconfigure(5, weight=1)
        self._refresh()

    # -- Liste -------------------------------------------------------------

    def _refresh(self):
        self.tree.delete(*self.tree.get_children())
        for index, room in enumerate(self.app.rooms):
            span = "   ".join(f"{a:.3f} .. {b:.3f}"
                              for a, b in zip(room.lo, room.hi))
            mark = "" if room.visible else "   (aus)"
            self.tree.insert("", "end", iid=str(index), text=room.name + mark,
                             values=(span,))
        self.app._redraw()

    def _selected(self):
        selection = self.tree.selection()
        return int(selection[0]) if selection else None

    def _fill_from_selection(self):
        index = self._selected()
        if index is None:
            return
        room = self.app.rooms[index]
        self.name_entry.delete(0, "end")
        self.name_entry.insert(0, room.name)
        for key, value in zip(("x0", "y0", "z0"), room.lo):
            self.entries[key].delete(0, "end")
            self.entries[key].insert(0, f"{value:.4f}")
        for key, value in zip(("x1", "y1", "z1"), room.hi):
            self.entries[key].delete(0, "end")
            self.entries[key].insert(0, f"{value:.4f}")
        self.colour = room.colour
        self._show_colour()

    def _show_colour(self):
        self.colour_button.configure(background=self.colour, foreground="white",
                                     activebackground=self.colour)

    def _pick_colour(self):
        chosen = colorchooser.askcolor(color=self.colour, parent=self)
        if chosen and chosen[1]:
            self.colour = chosen[1]
            self._show_colour()

    # -- Bearbeiten --------------------------------------------------------

    def _read(self):
        try:
            values = {k: parse_number(e.get()) for k, e in self.entries.items()}
        except ValueError:
            messagebox.showerror("Raeume", "Eine Eingabe ist nicht lesbar.",
                                 parent=self)
            return None
        name = self.name_entry.get().strip() or f"Raum {len(self.app.rooms) + 1}"
        return Room(name,
                    (values["x0"], values["y0"], values["z0"]),
                    (values["x1"], values["y1"], values["z1"]),
                    self.colour)

    def _add(self):
        room = self._read()
        if room:
            self.app.rooms.append(room)
            self._refresh()

    def _update(self):
        index = self._selected()
        room = self._read()
        if index is not None and room:
            room.visible = self.app.rooms[index].visible
            self.app.rooms[index] = room
            self._refresh()

    def _delete(self):
        index = self._selected()
        if index is not None:
            del self.app.rooms[index]
            self._refresh()

    def _toggle_visible(self):
        index = self._selected()
        if index is not None:
            self.app.rooms[index].visible = not self.app.rooms[index].visible
            self._refresh()

    def _from_track(self):
        """Felder mit der Huellbox der geladenen Bahn vorbelegen."""
        if not self.app.track:
            messagebox.showinfo("Raeume", "Es ist keine Bahn geladen.", parent=self)
            return
        lo, hi = self.app.track.lo, self.app.track.hi
        for key, value in zip(("x0", "y0", "z0"), lo):
            self.entries[key].delete(0, "end")
            self.entries[key].insert(0, f"{value:.4f}")
        for key, value in zip(("x1", "y1", "z1"), hi):
            self.entries[key].delete(0, "end")
            self.entries[key].insert(0, f"{value:.4f}")

    # -- Dateien -----------------------------------------------------------

    def _save(self):
        path = filedialog.asksaveasfilename(
            title="Raeume speichern", defaultextension=".json",
            initialfile="raeume.json", parent=self,
            filetypes=[("JSON", "*.json")])
        if not path:
            return
        with open(path, "w") as handle:
            json.dump([r.as_dict() for r in self.app.rooms], handle, indent=2)

    def _load(self):
        path = filedialog.askopenfilename(title="Raeume laden", parent=self,
                                          filetypes=[("JSON", "*.json")])
        if not path:
            return
        try:
            with open(path) as handle:
                data = json.load(handle)
            self.app.rooms = [Room.from_dict(d) for d in data]
        except (OSError, ValueError, KeyError) as exc:
            messagebox.showerror("Raeume", f"Nicht lesbar:\n{exc}", parent=self)
            return
        self._refresh()


def main():
    parser = argparse.ArgumentParser(description="Bahn-CSV im 3D-Raum ansehen")
    parser.add_argument("csv", nargs="?", help="Aufzeichnung des Bahnrekorders")
    args, _ = parser.parse_known_args()

    app = BahnViewer(args.csv)
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
