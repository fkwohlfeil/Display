#!/usr/bin/env python3
"""Pruefungen zum kartesischen Joggen aus 2.17.

Gegen eine Attrappe von /compute_ik. Sie rechnet keine Kinematik -- geprueft
wird der Kreislauf: Sollpose fuehren, fragen, anwenden, nachladen, anhalten.
Was der Solver antwortet, ist dafuer gleichgueltig; dass genau ein Schritt je
Antwort nachgeladen wird und die gefuehrte Pose nicht driftet, ist es nicht.
"""

import math
import os
import sys
import tempfile

import urdf_gui2_17 as G

CHECKS = []


def check(name, condition, detail=""):
    CHECKS.append((name, bool(condition), detail))


def near(a, b, tol=1e-9):
    return abs(a - b) <= tol


URDF = """<robot name="jog">
  <link name="base_link"/><link name="arm"/><link name="flansch"/>
  <joint name="a1" type="revolute">
    <parent link="base_link"/><child link="arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="100" velocity="1"/>
  </joint>
  <joint name="a2" type="revolute">
    <parent link="arm"/><child link="flansch"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="100" velocity="1"/>
  </joint>
</robot>"""


class FakeRos:
    """Nur das, was die Oberflaeche vom ROS-Takt anfasst."""

    def __init__(self):
        self.moveit_ready = True
        self.running = True
        self.enabled = True
        self.last_publish = None
        self.cycle_ms = None
        self.requests = []
        self.answer = {"code": 1, "values": {"a1": 0.0, "a2": 0.0}}
        self.sent = []

    def request_ik(self, payload):
        self.requests.append(payload)

    def set_positions(self, positions):
        self.sent.append(dict(positions))

    def set_model(self, *_a, **_k):
        return "Modell gesetzt"

    def reset_moveit(self, *_a):
        pass

    def take_results(self):
        return []

    def thread_alive(self):
        return True

    def shutdown(self):
        pass


path = os.path.join(tempfile.mkdtemp(), "jog.urdf")
open(path, "w").write(URDF)

app = G.UrdfViewer(path)
app.ros = FakeRos()
app.moveit_stale = False
app.moveit_config.tip = "flansch"
app.update()
app._open_drive()
app.drive_space_var.set(G.SPACE_CART)
app._drive_space_changed()
app.update()


def answer(a1=0.0, a2=0.0):
    """Die Attrappe antwortet mit diesen Achswerten."""
    return {"code": 1, "values": {"a1": a1, "a2": a2}}


def last_pose():
    return app.ros.requests[-1]["position"] if app.ros.requests else None


# ==========================================================================
# 1..6  Voraussetzungen
# ==========================================================================

ok, why = app._jog_ready()
check("01 bereit, wenn alles steht", ok, why)

app.stopped = True
ok, why = app._jog_ready()
check("02 STOP verhindert das Joggen", not ok and "STOP" in why, why)
app.stopped = False

app.ros.moveit_ready = False
ok, why = app._jog_ready()
check("03 ohne move_group kein Joggen", not ok, why)
app.ros.moveit_ready = True

app.moveit_stale = True
ok, why = app._jog_ready()
check("04 veraltetes Modell verhindert das Joggen", not ok, why)
app.moveit_stale = False

app.state = G.STATE_PLANNING
ok, _ = app._jog_ready()
check("05 nicht waehrend eines anderen Auftrags", not ok)
app.state = G.STATE_READY

check("06 Tasten sind angelegt", len(app.jog_buttons) == 12,
      len(app.jog_buttons))


# ==========================================================================
# 7..14  Ein Klick
# ==========================================================================

start = app._tcp_pose()
app.ros.requests.clear()
app._jog_press("z", 1)

check("07 genau eine Anfrage", len(app.ros.requests) == 1,
      len(app.ros.requests))
check("08 Anfrage ist unterwegs", app.jog_pending)
check("09 Taste gilt als gedrueckt", app.jog_key == "z")

asked = last_pose()
check("10 Z um genau eine Stufe weiter",
      near(asked[2], start[0][2] + G.JOG_STEP_M), f"{asked[2]!r}")
check("11 X unveraendert", near(asked[0], start[0][0]))
check("12 Tip-Link mitgegeben",
      app.ros.requests[-1]["tip"] == "flansch")
check("13 Startwert mitgegeben",
      set(app.ros.requests[-1]["seed"]) == {"a1", "a2"})

app._on_jog_result(answer(a1=0.02, a2=-0.01))
check("14 Achsen gestellt",
      near(app.positions["a1"], 0.02) and near(app.positions["a2"], -0.01),
      app.positions)


# ==========================================================================
# 15..20  Kein Nachladen ohne Halten
# ==========================================================================

check("15 keine zweite Anfrage nach einem Klick",
      len(app.ros.requests) == 1, len(app.ros.requests))
check("16 Anfrage nicht mehr offen", not app.jog_pending)
check("17 Verstaerkung angezeigt", app.jog_var.get() != "", app.jog_var.get())
check("18 Verstaerkung in rad/m", "rad/m" in app.jog_var.get(),
      app.jog_var.get())

app._jog_release()
check("19 nach dem Loslassen keine Taste mehr", app.jog_key is None)
check("20 gefuehrte Pose verworfen", app.jog_pose is None)


# ==========================================================================
# 21..28  Halten laedt nach, und die Pose driftet nicht
# ==========================================================================

start = app._tcp_pose()
app.ros.requests.clear()
app._jog_press("x", 1)
app._on_jog_result(answer(a1=0.5, a2=0.5))        # Roboter landet woanders
check("21 erster Schritt", len(app.ros.requests) == 1)

app._jog_begin_hold()
check("22 Halten laeuft", app.jog_hold)
check("23 Halten laedt sofort nach", len(app.ros.requests) == 2,
      len(app.ros.requests))

app._on_jog_result(answer(a1=0.6, a2=0.6))
check("24 je Antwort genau eine neue Anfrage", len(app.ros.requests) == 3,
      len(app.ros.requests))

# Der entscheidende Punkt: die Sollpose wird gefuehrt. Haette der Kreislauf
# bei jedem Schritt den TCP zurueckgelesen, laege sie jetzt irgendwo -- die
# Attrappe hat die Achsen absichtlich weit gestellt.
asked = last_pose()
check("25 X waechst nur um Stufen, nicht mit dem Ist-Stand",
      asked[0] > start[0][0] + G.JOG_STEP_M - 1e-9,
      f"{asked[0]!r} gegen {start[0][0]!r}")
check("26 Y unveraendert trotz gedrehter Achsen",
      near(asked[1], start[0][1]), f"{asked[1]!r}")
check("27 Z unveraendert", near(asked[2], start[0][2]), f"{asked[2]!r}")

app._jog_release()
count = len(app.ros.requests)
app._on_jog_result(answer(a1=0.7, a2=0.7))
check("28 nach dem Loslassen wird nicht nachgeladen",
      len(app.ros.requests) == count, len(app.ros.requests))


# ==========================================================================
# 29..34  Drehen
# ==========================================================================

app.state = G.STATE_READY
start = app._tcp_pose()
app.ros.requests.clear()
app._jog_press("yaw", -1)
asked = app.ros.requests[-1]["orientation"]
check("29 Drehung wird als Quaternion gefragt", len(asked) == 4)

matrix = G.from_rpy_xyz((start[1][0], start[1][1],
                         start[1][2] - math.radians(G.JOG_STEP_DEG)),
                        start[0])
want = G.to_quaternion(matrix)
check("30 Yaw um genau eine Stufe zurueck",
      max(abs(a - b) for a, b in zip(asked, want)) < 1e-9,
      f"{asked} gegen {want}")

app._on_jog_result(answer(a1=-0.001, a2=0.0))
check("31 Verstaerkung in rad/rad", "rad/rad" in app.jog_var.get(),
      app.jog_var.get())
check("32 Einheit der Vorgabe genannt", "deg/s" in app.jog_var.get(),
      app.jog_var.get())
app._jog_release()

# Ein Klick nach unten und einer nach oben heben sich auf.
app.ros.requests.clear()
before = app._tcp_pose()[0][1]
app._jog_press("y", 1)
up = last_pose()[1]
app._on_jog_result(answer())
app._jog_release()
app._jog_press("y", -1)
down = last_pose()[1]
app._on_jog_result(answer())
app._jog_release()
check("33 plus und minus sind spiegelbildlich",
      near(up - before, before - down), f"{up!r} {down!r} {before!r}")
check("34 Stufe ist die vorgegebene", near(up - before, G.JOG_STEP_M),
      f"{up - before!r}")


# ==========================================================================
# 35..41  Abbruch
# ==========================================================================

app.ros.requests.clear()
app._jog_press("z", 1)
app._jog_begin_hold()
app._on_jog_result({"code": -31, "values": {}})
check("35 kein Treffer haelt an", app.jog_key is None)
check("36 Halten aus", not app.jog_hold)
# Genau eine. Halten laedt nicht nach, solange eine Anfrage offen ist --
# sonst waeren zwei gleichzeitig unterwegs, und bei einem langsamen Solver
# stauten sie sich. Nachgeladen wird ausschliesslich aus der Antwort heraus.
count = len(app.ros.requests)
check("37 nie zwei Anfragen gleichzeitig unterwegs", count == 1, count)
check("38 Anzeige nennt den Grund",
      "Singularitaet" in app.jog_var.get() or "Loesung" in app.jog_var.get(),
      app.jog_var.get())

app.state = G.STATE_READY
app._jog_press("z", 1)
app._jog_begin_hold()
app._brake("Probe")
check("39 Bremsen haelt das Joggen an", app.jog_key is None)

app.state = G.STATE_READY
app._jog_press("z", 1)
app._jog_stop("Probe")
check("40 _jog_stop raeumt die Frist ab", app.jog_hold_job is None)
check("41 danach wieder bereit", app._jog_ready()[0], app._jog_ready()[1])


# ==========================================================================
# 42..46  Verstaerkung
# ==========================================================================

app.jog_delta = 0.01
app.jog_key = "x"
app._jog_report({"a1": 0.0, "a2": 0.0}, {"a1": 0.1, "a2": 0.0}, ["a1", "a2"])
text = app.jog_var.get()
check("42 Faktor 10 wird als 10 angezeigt", " 10.0 rad/m" in text, text)
check("43 schnellste Achse benannt", "a1" in text, text)
# 10 rad/m * 0.02 m/s = 0.2 rad/s, Grenze laut URDF 1 rad/s -> 20 Prozent
check("44 Anteil an der Achsgrenze", "20 %" in text, text)

app._jog_report({"a1": 0.0, "a2": 0.0}, {"a1": 1.0, "a2": 0.0}, ["a1", "a2"])
text = app.jog_var.get()
check("45 ueber der Grenze wird benannt", "UEBER DER ACHSGRENZE" in text, text)
check("46 mit Faktor", "2.0-fach" in text, text)

# ==========================================================================
# 47..49  Invariante: Taste und Halten fallen immer gemeinsam
# ==========================================================================
#
# Eine Mutationsprobe hat gezeigt, dass die Bedingung "jog_key is not None
# and jog_hold" in _on_jog_result nicht von "jog_hold" allein zu
# unterscheiden ist. Das ist kein Testleck, sondern folgt aus dieser
# Invariante -- und damit sie es bleibt, steht sie hier.

app.state = G.STATE_READY
app._jog_press("z", 1)
app._jog_begin_hold()
app._jog_stop("Probe")
check("47 _jog_stop loescht beides zugleich",
      app.jog_key is None and not app.jog_hold,
      f"{app.jog_key!r} {app.jog_hold!r}")

app._jog_begin_hold()                                  # ohne gedrueckte Taste
check("48 Halten ohne Taste beginnt nicht", not app.jog_hold)

app._jog_press("z", 1)
app._jog_release()
check("49 Loslassen loescht ebenfalls beides",
      app.jog_key is None and not app.jog_hold)

app._jog_stop("")
try:
    app.destroy()
except Exception:                                      # noqa: BLE001
    pass


# ==========================================================================
failed = [c for c in CHECKS if not c[1]]
for name, ok, detail in CHECKS:
    if not ok:
        print(f"FEHLER  {name}" + (f"   [{detail}]" if detail else ""))
print(f"\n{len(CHECKS) - len(failed)} von {len(CHECKS)} Pruefungen bestanden")
sys.exit(1 if failed else 0)
