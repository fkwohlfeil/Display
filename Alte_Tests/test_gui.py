#!/usr/bin/env python3
"""Oberflaeche unter Xvfb mit echtem Fenstermanager pruefen.

Ohne WM bleiben Zustandsfehler unsichtbar, die beim Bedienen sofort
auffallen -- deshalb laeuft der Test hinter openbox.
"""
import os
import sys
import types

# rclpy fehlt hier. RosBridge faengt den ImportError selbst ab und meldet
# "nur Anzeige und FK", genau wie auf einem Rechner ohne ROS.
sys.path.insert(0, "/home/claude/work")

import tkinter as tk
import importlib
gui = importlib.import_module("urdf_gui2_3")

URDF = """<?xml version="1.0"?>
<robot name="bm">
  <link name="base_link"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link>
  <link name="link1"/>
  <link name="link2"/>
  <link name="tool"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="2.5"/>
  </joint>
  <joint name="j2" type="prismatic">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="0.4" effort="80" velocity="0.3"/>
  </joint>
  <joint name="j_fix" type="fixed">
    <parent link="link2"/><child link="tool"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>
</robot>
"""

path = "/tmp/test_bm.urdf"
with open(path, "w", encoding="utf-8") as handle:
    handle.write(URDF)

failed = []


def check(label, condition, detail=""):
    if condition:
        print(f"  ok    {label}")
    else:
        print(f"  FEHL  {label}  {detail}")
        failed.append(label)


app = gui.UrdfViewer(path)
app.update()

print("Start")
check("Modell geladen", app.robot_name == "bm")
check("zwei bewegliche Gelenke", len(app.sliders) == 2, list(app.sliders))
check("Achsgrenzen angelegt", set(app.moveit_config.limits) == {"j1", "j2"},
      set(app.moveit_config.limits))
check("nicht gesperrt", app._busy() is False)
check("Fusszeile zeigt move_group", "move_group" in app.moveit_var.get(),
      app.moveit_var.get())

print("\nParameterfenster")
app._open_params()
app.update()
check("Fenster da", app.params_window is not None
      and app.params_window.winfo_exists())
check("Felder pro Auftrag vorhanden",
      all(key in app.param_entries for key, _l, _u, _d in gui.PLAN_FIELDS))
check("Bahngrenzen vorhanden",
      all(key in app.param_entries for key, _l, _u, _d in gui.CARTESIAN_FIELDS))
check("Achsgrenzen je Gelenk", set(app.limit_entries) == {"j1", "j2"},
      set(app.limit_entries))
check("v vorbelegt aus URDF",
      app.limit_entries["j1"]["velocity"].get() == "2.5",
      app.limit_entries["j1"]["velocity"].get())
check("Override vorbelegt",
      app.param_entries["velocity_scaling"].get() == "100")

print("\nUebernehmen")
entry = app.param_entries["planning_time"]
entry.delete(0, "end")
entry.insert(0, "7.5")
check("Uebernehmen meldet Erfolg", app._params_apply() is True)
check("Wert in der Konfiguration",
      app.moveit_config.plan["planning_time"] == 7.5,
      app.moveit_config.plan["planning_time"])

entry.delete(0, "end")
entry.insert(0, "abc")
check("unlesbar wird abgewiesen", app._params_apply() is False)
check("Feld ist rot markiert", entry.cget("background") == gui.BG_INVALID)
check("alter Wert steht noch",
      app.moveit_config.plan["planning_time"] == 7.5)
entry.delete(0, "end")
entry.insert(0, "5")
app._params_apply()

print("\nNull beim Override")
entry = app.param_entries["velocity_scaling"]
entry.delete(0, "end")
entry.insert(0, "0")
check("Null wird abgewiesen", app._params_apply() is False)
entry.delete(0, "end")
entry.insert(0, "150")
app._params_apply()
check("ueber 100 wird begrenzt",
      app.moveit_config.plan["velocity_scaling"] == 100.0,
      app.moveit_config.plan["velocity_scaling"])
entry.delete(0, "end")
entry.insert(0, "50")
app._params_apply()

print("\nVorzeichen")
entry = app.param_entries["max_trans_dec"]
entry.delete(0, "end")
entry.insert(0, "5")
check("positive Bahnverzoegerung abgewiesen", app._params_apply() is False)
entry.delete(0, "end")
entry.insert(0, "-5")
check("negative geht durch", app._params_apply() is True)

field = app.limit_entries["j1"]["deceleration"]
field.delete(0, "end")
field.insert(0, "3")
check("positive Achsverzoegerung abgewiesen", app._params_apply() is False)
field.delete(0, "end")
field.insert(0, "-5")
app._params_apply()

print("\nGrenze abschalten")
field = app.limit_entries["j2"]["jerk"]
field.delete(0, "end")
app._params_apply()
check("leeres Feld heisst None",
      app.moveit_config.limits["j2"]["jerk"] is None)
check("YAML sagt has_jerk_limits false",
      "has_jerk_limits: false" in app.moveit_config.limits_yaml(0))

print("\nTip-Link")
entry = app.param_entries["tip"]
entry.delete(0, "end")
entry.insert(0, "gibt_es_nicht")
check("unbekannter Link abgewiesen", app._params_apply() is False)
entry.delete(0, "end")
entry.insert(0, "link2")
check("bekannter Link geht", app._params_apply() is True)
entry.delete(0, "end")
app._params_apply()

print("\nSperre waehrend der Fahrt")
app.begin_motion("Fahrt")
app.update()
check("gesperrt", app._busy() is True)
check("Schieber aus",
      str(app.sliders["j1"].cget("state")) == "disabled",
      app.sliders["j1"].cget("state"))
check("Sollwertfeld aus",
      str(app.entries["j1"].cget("state")) == "disabled")
check("Bewegungsfeld aus",
      str(app.motion_entries["speed"].cget("state")) == "disabled")
check("Parameterfeld aus",
      str(app.param_entries["planning_time"].cget("state")) == "disabled")
check("Achsgrenzenfeld aus",
      str(app.limit_entries["j1"]["velocity"].cget("state")) == "disabled")
check("Laden und Bearbeiten aus",
      all(str(b.cget("state")) == "disabled" for b in app.model_buttons))
check("Uebernehmen wird abgewiesen", app._params_apply() is False)

before = app.urdf_xml
check("Modellaenderung wird abgewiesen",
      app._commit("Test", lambda root: None) is False)
check("Modell unveraendert", app.urdf_xml == before)

app.end_motion()
app.update()
check("wieder frei", app._busy() is False)
check("Schieber wieder an",
      str(app.sliders["j1"].cget("state")) == "normal")
check("Parameterfeld wieder an",
      str(app.param_entries["planning_time"].cget("state")) == "normal")
check("Uebernehmen geht wieder", app._params_apply() is True)

print("\nSTOP sperrt Sollwerte, nicht die Parameter")
app._toggle_stop()
app.update()
check("Schieber aus", str(app.sliders["j1"].cget("state")) == "disabled")
check("Parameterfeld bleibt an",
      str(app.param_entries["planning_time"].cget("state")) == "normal")
app._toggle_stop()
app.update()

print("\nModellaenderung und Veraltet-Marke")
app.moveit.process = types.SimpleNamespace(poll=lambda: None,
                                           terminate=lambda: None,
                                           wait=lambda timeout=0: None,
                                           kill=lambda: None)
app.moveit.started_at = gui.time.monotonic() - 30.0
app.moveit_stale = False
app._commit("Achse geaendert",
            lambda root: root.find("joint[@name='j1']").find("limit")
            .set("velocity", "1.0"))
app.update()
check("veraltet gesetzt", app.moveit_stale is True)
check("Text sagt veraltet", "veraltet" in app._moveit_text(),
      app._moveit_text())
app.moveit.process = None
app.moveit.started_at = None
app.moveit_stale = False

print("\nGelenk faellt weg")
app._commit("Gelenk entfernt",
            lambda root: gui.remove_limb(root, "j2", keep_pose=True))
app.update()
check("Achsgrenzen nachgezogen", set(app.moveit_config.limits) == {"j1"},
      set(app.moveit_config.limits))
check("Tabelle neu aufgebaut", set(app.limit_entries) == {"j1"},
      set(app.limit_entries))

print("\nBewegungsfelder wirken auf die Bahngrenzen")
app.motion_entries["speed"].delete(0, "end")
app.motion_entries["speed"].insert(0, "0.25")
app._apply_motion()
check("Speed in den Bahngrenzen",
      app.moveit_config.cartesian["max_trans_vel"] == 0.25,
      app.moveit_config.cartesian["max_trans_vel"])
check("Feld im Fenster nachgezogen",
      app.param_entries["max_trans_vel"].get() == "0.25",
      app.param_entries["max_trans_vel"].get())

print("\nSRDF aus dem laufenden Modell")
srdf = gui.build_srdf(app.robot_name, app.links, app.joints,
                      app.moveit_config.group, app.moveit_config.tip or None)
check("Gruppe passt", f'name="{app.moveit_config.group}"' in srdf)
params = app.moveit_config.params_yaml(app.urdf_xml, srdf)
check("Parameterdatei entsteht", len(params) > 500)

print("\nFenster schliessen")
app._close_params()
app.update()
check("Fenster weg", app.params_window is None)
check("keine Restfelder", app.param_entries == {})
app._open_params()
app.update()
check("laesst sich wieder oeffnen",
      app.params_window is not None and app.params_window.winfo_exists())

app.moveit.stop()
app.destroy()

print()
if failed:
    print(f"{len(failed)} Pruefung(en) fehlgeschlagen: {failed}")
    sys.exit(1)
print("alle Pruefungen bestanden")
