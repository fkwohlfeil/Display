#!/usr/bin/env python3
"""Prueft, dass der ROS-Takt aus 2.11 einen schlechten Callback ueberlebt.

Ohne Tk und ohne ROS. Die Frage ist die aus dem Fehlerbild: eine Ausnahme in
irgendeinem Callback -- bleibt danach die TF-Ausgabe stehen? Bis 2.10 lautete
die Antwort ja, weil executor.spin() unmittelbar das Ziel des Threads war und
eine Ausnahme ihn verliess. Der Thread endete, alle Timer mit ihm, und nach
aussen sah man nur eine Zahl in der Fusszeile hochlaufen.

Nachgestellt wird mit einem Executor aus zwei Zeilen, der sich wie der echte
verhaelt: spin_once ruft die angemeldeten Callbacks, und einer davon wirft.

Aufruf:  python3 test_spin_guard.py
"""

import os
import sys
import threading
import time
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

sys.modules["rclpy"] = types.ModuleType("rclpy")

import urdf_gui2_11 as gui                             # noqa: E402

FAILED = []


def check(name, condition, detail=""):
    print(f"  {'ok  ' if condition else 'FEHL'}  {name}"
          f"{'  ' + detail if detail else ''}")
    if not condition:
        FAILED.append(name)


class FakeExecutor:
    """Ruft bei jedem Durchlauf die angemeldeten Callbacks, wie der echte."""

    def __init__(self, callbacks):
        self.callbacks = callbacks
        self.rounds = 0

    def spin(self):
        while True:
            self.spin_once(0.05)

    def spin_once(self, timeout_sec=0.0):
        self.rounds += 1
        for callback in self.callbacks:
            callback()
        time.sleep(0.005)


def build(bridge, callbacks):
    bridge._executor = FakeExecutor(callbacks)
    bridge._stop = threading.Event()
    bridge._spin_errors = 0
    bridge._reset_wanted = False


# ---- 1. Das alte Verhalten ----------------------------------------------

print("\n1. Der Fehler aus 2.10: spin() als Thread-Ziel")
ticks = []
state = {"throw": False}


def tf_callback():
    if state["throw"]:
        state["throw"] = False
        raise ValueError("Antwort nicht lesbar")
    ticks.append(time.monotonic())


executor = FakeExecutor([tf_callback])
thread = threading.Thread(target=executor.spin, daemon=True)
thread.start()
time.sleep(0.2)
before = len(ticks)
state["throw"] = True
time.sleep(0.3)

check("Takt lief zuvor", before > 5, f"{before} Durchlaeufe")
check("Thread ist nach der Ausnahme tot", not thread.is_alive())
check("keine weitere TF-Ausgabe", len(ticks) == before,
      f"{len(ticks) - before} nach dem Fehler")


# ---- 2. Das neue Verhalten ----------------------------------------------

print("\n2. Die Korrektur in 2.11: eigene Schleife mit Auffangen")
bridge = gui.RosBridge()
ticks = []
state = {"throw": False}
build(bridge, [lambda: bridge._guard("TF", tf_callback)])
thread = threading.Thread(target=bridge._spin, daemon=True)
thread.start()
time.sleep(0.2)
before = len(ticks)
state["throw"] = True
time.sleep(0.3)

check("Takt lief zuvor", before > 5, f"{before} Durchlaeufe")
check("Thread lebt weiter", thread.is_alive())
check("TF geht weiter", len(ticks) > before,
      f"{len(ticks) - before} nach dem Fehler")

results = list(bridge.results)
errors = [value for kind, value in results if kind == "ros_error"]
check("Fehler wurde gemeldet", len(errors) == 1, f"{len(errors)} Meldung(en)")
check("Meldung nennt den Callback", errors and errors[0].startswith("TF:"),
      errors[0][:60] if errors else "")
check("Meldung nennt die Ausnahme",
      errors and "ValueError" in errors[0])
bridge._stop.set()
time.sleep(0.1)
check("Schleife endet auf Wunsch", not thread.is_alive())


# ---- 3. Ein Dauerfehler laeuft nicht endlos ------------------------------

print("\n3. Ein Dauerfehler beendet den Takt geordnet")
bridge = gui.RosBridge()


def always_bad():
    raise RuntimeError("immer kaputt")


class BadExecutor(FakeExecutor):
    def spin_once(self, timeout_sec=0.0):
        self.rounds += 1
        raise RuntimeError("Wartefeld kaputt")


bridge._executor = BadExecutor([])
bridge._stop = threading.Event()
bridge._spin_errors = 0
bridge._reset_wanted = False
thread = threading.Thread(target=bridge._spin, daemon=True)
thread.start()
thread.join(timeout=5.0)

errors = [value for kind, value in bridge.results if kind == "ros_error"]
check("Takt hat aufgegeben", not thread.is_alive())
check("Grenze eingehalten", bridge._spin_errors == gui.SPIN_ERROR_LIMIT,
      f"{bridge._spin_errors} Fehler")
check("letzte Meldung erklaert das Ende",
      errors and "wird beendet" in errors[-1], errors[-1] if errors else "")


# ---- 4. Der Waechter auf die TF-Ausgabe ---------------------------------

print("\n4. Der Waechter meldet einmal, nicht dauernd")


class Rig:
    def __init__(self):
        self.tf_stalled = False
        self.messages_log = []
        self.ros = types.SimpleNamespace(thread_alive=lambda: False)

    def _log(self, text, level="info"):
        self.messages_log.append((level, text))


Rig._tf_watchdog = gui.UrdfViewer._tf_watchdog

rig = Rig()
rig._tf_watchdog(33.0)
check("im Normalfall still", rig.messages_log == [])

rig._tf_watchdog(gui.TF_STALL_MS + 500.0)
check("Stillstand wird gemeldet", len(rig.messages_log) == 1)
check("Meldung nennt den toten Takt",
      "beendet" in rig.messages_log[0][1], rig.messages_log[0][1][:70])

for _ in range(20):
    rig._tf_watchdog(gui.TF_STALL_MS + 500.0)
check("nur einmal, kein Rauschen", len(rig.messages_log) == 1,
      f"{len(rig.messages_log)} Meldungen")

rig._tf_watchdog(33.0)
check("Erholung wird gemeldet", len(rig.messages_log) == 2)
check("und sagt es deutlich", "laeuft wieder" in rig.messages_log[1][1])

rig = Rig()
rig.ros = types.SimpleNamespace(thread_alive=lambda: True)
rig._tf_watchdog(gui.TF_STALL_MS + 500.0)
check("lebender Takt wird anders benannt",
      "laeuft, aber nichts wird gesendet" in rig.messages_log[0][1],
      rig.messages_log[0][1][:80])


print()
if FAILED:
    print(f"{len(FAILED)} Pruefung(en) fehlgeschlagen: {', '.join(FAILED)}")
    sys.exit(1)
print("Alle Pruefungen bestanden.")
