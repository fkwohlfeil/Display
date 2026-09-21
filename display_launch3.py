#!/usr/bin/env python3
"""Startet die URDF-GUI und RViz2 mit passender Konfiguration.

3 startet urdf_gui2_1.py. 2 zeigte noch auf urdf_gui1_17.py.

Seit urdf_gui2_1 besteht die GUI aus mehreren Dateien: einstellungen, stil,
matrix, anzeige, urdf_modell, kinematik, moveit_konfig, bahn, totg_prozess
und ros_bruecke, dazu wie bisher konvention, sequenz, totg_folge und
totg_rechner. Alle muessen im selben Ordner wie die GUI liegen. Python
findet sie dort, weil der Ordner des gestarteten Skripts vorn in sys.path
steht -- unabhaengig davon, aus welchem Verzeichnis gestartet wird. Passt
ein Modul nicht zur GUI, bricht sie mit der Liste der Staende ab; das
Launch faehrt dann ueber on_exit mit herunter.

Aufruf:
    ros2 launch display_launch3.py
    ros2 launch display_launch3.py urdf:=/pfad/zur/robot.urdf
    ros2 launch display_launch3.py rviz:=false          # nur die GUI
    ros2 launch display_launch3.py software_gl:=false   # native GPU statt Software

Das Skript liegt bewusst neben der GUI und wird nicht als Package installiert.
Alle Pfade sind deshalb relativ zu diesem File aufgeloest.
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            SetEnvironmentVariable, Shutdown, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))

GUI_SCRIPT = os.path.join(HERE, "urdf_gui2_1.py")
DEFAULT_URDF = os.path.join(HERE, "bmKinematic.urdf")
DEFAULT_RVIZ = os.path.join(HERE, "display2.rviz")


def generate_launch_description():
    urdf = LaunchConfiguration("urdf")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("rviz")
    software_gl = LaunchConfiguration("software_gl")

    return LaunchDescription([

        DeclareLaunchArgument(
            "urdf", default_value=DEFAULT_URDF,
            description="/home/fwohlfeil/ros2_ws/src/bmkinematic_describtion/urdf/bmkKinematic.urdf"),
        DeclareLaunchArgument(
            "rviz_config", default_value=DEFAULT_RVIZ,
            description="/home/fwohlfeil/Display/display2.rviz"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="RViz2 mitstarten"),
        DeclareLaunchArgument(
            "software_gl", default_value="true",
            description="Software-Rendering erzwingen, noetig unter WSL"),

        # Unter WSL scheitert RViz sonst haeufig am OpenGL-Kontext.
        # Auf nativer Hardware mit GPU besser software_gl:=false setzen.
        SetEnvironmentVariable(
            "LIBGL_ALWAYS_SOFTWARE", "1",
            condition=IfCondition(software_gl)),

        LogInfo(msg=["URDF: ", urdf]),

        # Die GUI ist die Quelle von /tf, /tf_static und robot_description.
        # Beendet der Bediener das Fenster, faehrt das ganze Launch herunter.
        ExecuteProcess(
            cmd=["python3", GUI_SCRIPT, urdf],
            name="urdf_gui",
            output="screen",
            emulate_tty=True,
            on_exit=[Shutdown(reason="GUI wurde geschlossen")]),

        # Kleiner Vorlauf, damit robot_description schon anliegt, wenn RViz
        # startet. Dank TRANSIENT_LOCAL waere auch die andere Reihenfolge
        # unkritisch, aber so bleibt die Konsole frei von Startmeldungen.
        TimerAction(period=1.5, actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(use_rviz)),
        ]),
    ])
