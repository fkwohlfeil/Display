#!/usr/bin/env python3
"""Prueft den Kindprozess von urdf_gui2_5 ohne Tk und ohne ROS.

Der Kern ist der Nachweis, dass 2.4 tatsaechlich Waisen hinterlaesst und 2.5
nicht. Dafuer wird move_group durch ein Paar aus Wrapper und Kind ersetzt:
ein Shell-Skript, das ein zweites startet und darauf wartet. Genau so
verhaelt sich "ros2 run" gegenueber der eigentlichen Binary.

Aufruf:  python3 test_moveit_process.py
"""

import os
import signal
import subprocess
import sys
import tempfile
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# ---- Tk und ROS wegstellen, wie in test_logic.py ------------------------

stub = types.ModuleType("tkinter")
stub.Tk = type("Tk", (), {})
stub.StringVar = stub.BooleanVar = type("Var", (), {})
stub.Frame = stub.Canvas = stub.Text = stub.Menu = type("W", (), {})
stub.ttk = types.ModuleType("tkinter.ttk")
stub.font = types.ModuleType("tkinter.font")
stub.filedialog = types.ModuleType("tkinter.filedialog")
stub.messagebox = types.ModuleType("tkinter.messagebox")
for name, module in (("tkinter", stub), ("tkinter.ttk", stub.ttk),
                     ("tkinter.font", stub.font),
                     ("tkinter.filedialog", stub.filedialog),
                     ("tkinter.messagebox", stub.messagebox)):
    sys.modules[name] = module

import urdf_gui2_5 as gui                              # noqa: E402

FAILED = []


def check(name, condition, detail=""):
    print(f"  {'ok  ' if condition else 'FEHL'}  {name}{'  ' + detail if detail else ''}")
    if not condition:
        FAILED.append(name)


def alive(pid):
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def make_pair(directory):
    """Wrapper, der ein Kind startet und wartet -- wie "ros2 run"."""
    child = os.path.join(directory, "child.sh")
    wrapper = os.path.join(directory, "wrapper.sh")
    with open(child, "w") as handle:
        handle.write("#!/bin/sh\necho kind laeuft\nwhile true; do sleep 1; done\n")
    with open(wrapper, "w") as handle:
        handle.write(f"#!/bin/sh\nexec 2>&1\n{child} &\necho $! > "
                     f"{directory}/child.pid\nwait\n")
    os.chmod(child, 0o755)
    os.chmod(wrapper, 0o755)
    return wrapper


# ---- 1. Das alte Verhalten nachstellen ---------------------------------

print("\n1. Der Fehler aus 2.4: terminate() trifft nur den Wrapper")
with tempfile.TemporaryDirectory() as directory:
    wrapper = make_pair(directory)
    process = subprocess.Popen([wrapper], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, start_new_session=True)
    time.sleep(0.6)
    with open(os.path.join(directory, "child.pid")) as handle:
        child = int(handle.read().strip())

    process.terminate()                                # genau wie in 2.4
    process.wait(timeout=5.0)
    time.sleep(0.5)
    check("Wrapper ist beendet", process.poll() is not None)
    check("Kind lebt weiter -- die Waise", alive(child), f"PID {child}")
    if alive(child):
        os.kill(child, signal.SIGKILL)


# ---- 2. Das neue Verhalten ---------------------------------------------

print("\n2. Die Korrektur in 2.5: killpg trifft die ganze Gruppe")
with tempfile.TemporaryDirectory() as directory:
    wrapper = make_pair(directory)
    proc = gui.MoveItProcess()
    proc.process = subprocess.Popen([wrapper], stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL,
                                    start_new_session=True)
    proc.pgid = os.getpgid(proc.process.pid)
    proc.started_at = time.monotonic()
    time.sleep(0.6)
    with open(os.path.join(directory, "child.pid")) as handle:
        child = int(handle.read().strip())

    check("Wrapper und Kind laufen", proc.alive and alive(child))
    proc.stop()
    time.sleep(0.5)
    check("Wrapper ist beendet", proc.process is None)
    check("Kind ist mitgegangen", not alive(child), f"PID {child}")
    if alive(child):
        os.kill(child, signal.SIGKILL)


# ---- 3. Zustaende ------------------------------------------------------

print("\n3. Zustaende und Aufraeumen")
proc = gui.MoveItProcess()
check("Zustand ohne Prozess ist 'aus'", proc.state() == "aus")
check("alive ist falsch", not proc.alive)
check("uptime ist None", proc.uptime() is None)
check("log_tail ist leer", proc.log_tail() == [])
check("stop ohne Prozess wirft nicht", proc.stop() is None)

base = tempfile.gettempdir()
old = tempfile.mkdtemp(prefix=gui.WORKDIR_PREFIX)
os.utime(old, (time.time() - 7200, time.time() - 7200))
fresh = tempfile.mkdtemp(prefix=gui.WORKDIR_PREFIX)
removed = gui.MoveItProcess.sweep_workdirs(max_age_s=3600.0)
check("alter Ordner entfernt", not os.path.isdir(old), f"{removed} entfernt")
check("frischer Ordner bleibt", os.path.isdir(fresh))
gui.shutil.rmtree(fresh, ignore_errors=True)


# ---- 4. Knotenname und Suchpfad ----------------------------------------

print("\n4. Knotenname und Suchpfad")
proc = gui.MoveItProcess()
proc._serial = 3
name = f"move_group_gui{os.getpid()}_4"
proc._serial += 1
proc.node_name = f"move_group_gui{os.getpid()}_{proc._serial}"
check("Knotenname ist eindeutig je Start", proc.node_name == name, proc.node_name)
check("Knotenname beginnt mit move_group",
      proc.node_name.startswith("move_group"))
check("find_move_group ohne AMENT liefert None",
      (os.environ.pop("AMENT_PREFIX_PATH", None), gui.find_move_group())[1] is None)
check("move_group_pids liefert eine Liste",
      isinstance(gui.move_group_pids(), list))
check("strays kennt den eigenen Prozess nicht mit",
      gui.MoveItProcess().strays() == gui.move_group_pids())


# ---- 5. Parameterdatei -------------------------------------------------

print("\n5. Parameterdatei entsteht weiterhin")
config = gui.MoveItConfig()
text = config.params_yaml("<robot name='r'/>", "<robot name='r'/>")
check("robot_description steht drin", "robot_description:" in text)
check("Gruppe steht in der Kinematik", gui.MOVEIT_GROUP in text)
check("Signatur ist vergleichbar",
      config.startup_signature() == gui.MoveItConfig().startup_signature())


print()
if FAILED:
    print(f"{len(FAILED)} Pruefung(en) fehlgeschlagen: {', '.join(FAILED)}")
    sys.exit(1)
print("Alle Pruefungen bestanden.")
