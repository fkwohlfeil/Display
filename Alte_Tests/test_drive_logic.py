#!/usr/bin/env python3
"""Prueft die Trennung von Stellen, Planen und Fahren aus 2.7.

Ohne Tk und ohne ROS. Geprueft wird die Ablauflogik an einer Attrappe, die
nur die Felder traegt, die die geprueften Methoden anfassen -- nicht das
Fenster. Die Frage lautet: kann der Roboter losfahren, ohne dass jemand
"Fahrt starten" gedrueckt hat? In 2.4 bis 2.6 lautete die Antwort ja, und
zwar bei jeder Eingabe in ein Sollwertfeld.

Aufruf:  python3 test_drive_logic.py
"""

import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

for name in ("tkinter", "tkinter.ttk", "tkinter.font",
             "tkinter.filedialog", "tkinter.messagebox"):
    sys.modules[name] = types.ModuleType(name)
tk = sys.modules["tkinter"]
tk.ttk = sys.modules["tkinter.ttk"]


class _Widget:
    def __init__(self, *args, **kwargs):
        pass

    def __getattr__(self, name):
        return lambda *a, **k: None


for name in ("Tk", "Toplevel", "Frame", "Canvas", "Text", "Menu", "StringVar",
             "BooleanVar", "DoubleVar", "IntVar", "PhotoImage", "Scrollbar",
             "Listbox", "Entry", "Button", "Label"):
    setattr(tk, name, type(name, (_Widget,), {}))

import urdf_gui2_7 as gui                              # noqa: E402

FAILED = []


def check(name, condition, detail=""):
    print(f"  {'ok  ' if condition else 'FEHL'}  {name}"
          f"{'  ' + detail if detail else ''}")
    if not condition:
        FAILED.append(name)


class FakeVar:
    def __init__(self, value=""):
        self.value = value

    def set(self, value):
        self.value = value

    def get(self):
        return self.value


class FakeEntry:
    """Nur so viel Entry, wie die geprueften Methoden anfassen."""

    def __init__(self, text=""):
        self.text = text
        self.background = None
        self.state = "normal"

    def get(self):
        return self.text

    def delete(self, *_args):
        self.text = ""

    def insert(self, _index, text):
        self.text = str(text)

    def configure(self, **kwargs):
        self.background = kwargs.get("background", self.background)
        self.state = kwargs.get("state", self.state)

    def winfo_exists(self):
        return True


def joint(name, lower=-math.pi, upper=math.pi):
    return {"name": name, "type": "revolute", "mimic": None,
            "has_limit": True, "lower": lower, "upper": upper,
            "parent": "a", "child": "b", "velocity": 1.0}


class Rig:
    """Eine Attrappe der GUI mit den echten Methoden aus 2.7."""

    def __init__(self):
        self.joints = [joint("joint1"), joint("joint2")]
        self.positions = {"joint1": 0.0, "joint2": 0.0}
        self.drive_entries = {"joint1": FakeEntry("0.00"),
                              "joint2": FakeEntry("0.00")}
        self.entries = {}
        self.drive_widgets = []
        self.drive_buttons = []
        self.start_buttons = []
        self.go_buttons = []
        self.sliders = {}
        self.cart_entries = {}
        self.motion_entries = {}
        self.model_buttons = []
        self.param_widgets = []
        self.undo_button = None
        self.history = []
        self.plan_hold = None
        self.plan_hold_start = None
        self.plan_hold_goal = None
        self.plan_started = None
        self.player = None
        self.play_started = None
        self.motion_job = None
        self.state = gui.STATE_READY
        self.stopped = False
        self.lock_reason = None
        self.moveit_stale = False
        self.moveit_config = gui.MoveItConfig()
        self.drive_plan_var = FakeVar("kein Plan")
        self.drive_status_var = FakeVar("")
        self.state_var = FakeVar(gui.STATE_READY)
        self.messages_log = []
        self.sent = []
        self.requested = []
        self.played = []
        self.ros = types.SimpleNamespace(
            moveit_ready=True,
            set_positions=lambda values: self.sent.append(dict(values)),
            request_plan=lambda payload: self.requested.append(payload))

    # -- Stellen, die der Test ersetzt --------------------------------------

    def _log(self, text, level="info"):
        self.messages_log.append((level, text))

    def _update_locks(self):
        pass

    def _drive_status(self):
        pass

    def _params_status(self):
        pass

    def _set_entry(self, *_args, **_kwargs):
        pass

    def _sync_sliders(self):
        pass

    def _sync_joint_entries(self):
        pass

    def _update_kinematics(self):
        pass

    def _start_playing(self, player):
        self.plan_hold = None
        self.plan_hold_start = None
        self.drive_plan_var.set("wird gefahren")
        self.played.append(player)
        self.state = gui.STATE_MOVING

    def _set_state(self, state, why=""):
        self.state = state
        self.state_var.set(state)


# Die echten Methoden aus 2.7 auf die Attrappe legen.
for name in ("_drive_plan", "_drive_start", "_drop_plan",
             "_read_joint_targets", "_push_positions", "_on_plan_result"):
    setattr(Rig, name, getattr(gui.UrdfViewer, name))


# ---- 1. Ein Ziel bewegt nichts ------------------------------------------

print("\n1. Ein Ziel im Fahrfenster bewegt nichts")
rig = Rig()
rig.drive_entries["joint1"].insert(0, "45.0")
check("Achsstand unveraendert", rig.positions["joint1"] == 0.0)
check("nichts an ROS gesendet", rig.sent == [])
check("kein Plan gehalten", rig.plan_hold is None)


# ---- 2. Planen faehrt nicht ---------------------------------------------

print("\n2. Planen stellt den Auftrag, faehrt aber nicht")
rig._drive_plan()
check("ein Auftrag gestellt", len(rig.requested) == 1)
check("Zustand ist PLANEN", rig.state == gui.STATE_PLANNING)
check("Achsstand unveraendert", rig.positions["joint1"] == 0.0)
check("Ziel korrekt uebersetzt",
      abs(rig.requested[0]["goal"]["joint1"] - math.radians(45.0)) < 1e-12,
      f"{rig.requested[0]['goal']['joint1']:.6f} rad")
check("Start ist der aktuelle Stand",
      rig.requested[0]["start"] == {"joint1": 0.0, "joint2": 0.0})


# ---- 3. Das Ergebnis wird gehalten --------------------------------------

print("\n3. Der Plan wird gehalten, nicht gefahren")
points = [(0.0, {"joint1": 0.0, "joint2": 0.0}, {"joint1": 0.0, "joint2": 0.0}),
          (1.0, {"joint1": math.radians(45.0), "joint2": 0.0},
           {"joint1": 0.0, "joint2": 0.0})]
rig._on_plan_result({"code": 1, "points": points, "planning_time": 0.01})
check("Plan gehalten", rig.plan_hold is not None)
check("Zustand wieder BEREIT", rig.state == gui.STATE_READY)
check("noch nichts gefahren", rig.played == [])
check("Anzeige meldet den Plan", "Plan bereit" in rig.drive_plan_var.get(),
      rig.drive_plan_var.get())


# ---- 4. Erst die Taste faehrt -------------------------------------------

print("\n4. Erst 'Fahrt starten' bewegt")
rig._drive_start()
check("die Bahn laeuft", len(rig.played) == 1)
check("Zustand ist FAHREN", rig.state == gui.STATE_MOVING)
check("Plan ist verbraucht", rig.plan_hold is None)


# ---- 5. Ohne Plan faehrt nichts -----------------------------------------

print("\n5. Ohne Plan faehrt nichts")
rig = Rig()
rig._drive_start()
check("keine Fahrt", rig.played == [])
check("Meldung erklaert warum",
      any("Erst 'Planen'" in text for _l, text in rig.messages_log))


# ---- 6. Ein gestellter Sollwert verwirft den Plan -----------------------

print("\n6. Ein gestellter Sollwert verwirft den Plan")
rig = Rig()
rig.drive_entries["joint1"].insert(0, "45.0")
rig._drive_plan()
rig._on_plan_result({"code": 1, "points": points, "planning_time": 0.01})
check("Plan liegt bereit", rig.plan_hold is not None)

rig.positions["joint1"] = math.radians(10.0)           # wie der Schieber
rig._push_positions()
check("Plan verworfen", rig.plan_hold is None)
check("Grund im Protokoll",
      any("Plan verworfen" in text for _l, text in rig.messages_log))
rig._drive_start()
check("es faehrt nichts", rig.played == [])


# ---- 7. Waehrend der Fahrt wird kein Plan verworfen ---------------------

print("\n7. Der eigene Takt verwirft nichts")
rig = Rig()
rig.state = gui.STATE_MOVING
before = len(rig.messages_log)
rig.positions["joint1"] = 0.1
rig._push_positions()                                  # wie _motion_tick
check("keine Meldung erzeugt", len(rig.messages_log) == before)
check("Werte gesendet", rig.sent and rig.sent[-1]["joint1"] == 0.1)


# ---- 8. Ziel gleich Stand ----------------------------------------------

print("\n8. Ziel gleich Stand wird abgefangen")
rig = Rig()
rig._drive_plan()                                      # Felder stehen auf 0.00
check("kein Auftrag gestellt", rig.requested == [])
check("Meldung erklaert warum",
      any("nichts zu planen" in text for _l, text in rig.messages_log))


# ---- 9. Sperren waehrend der Fahrt --------------------------------------

print("\n9. Sperren")
rig = Rig()
rig.lock_reason = gui.STATE_MOVING
busy = gui.UrdfViewer._busy(rig)
check("_busy erkennt die Fahrt", busy)
rig.lock_reason = None
check("_busy ist sonst falsch", not gui.UrdfViewer._busy(rig))


print()
if FAILED:
    print(f"{len(FAILED)} Pruefung(en) fehlgeschlagen: {', '.join(FAILED)}")
    sys.exit(1)
print("Alle Pruefungen bestanden.")
