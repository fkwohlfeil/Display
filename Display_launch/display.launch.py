#!/usr/bin/env python3
"""
display.launch.py  --  startet alle vier Prozesse des Robot-Displays.

Ersetzt diese vier Terminals:
    ros2 run robot_state_publisher robot_state_publisher bmKinematic.urdf
    ros2 run joint_state_publisher_gui joint_state_publisher_gui
    rviz2 -d bmKinematic.rviz
    python3 robot_display_umsetzung_2.py

Aufruf (aus ~/Display):
    ros2 launch display.launch.py

Optionale Argumente:
    ros2 launch display.launch.py gui:=false          # ohne joint_state_publisher_gui
    ros2 launch display.launch.py rviz:=false         # ohne RViz
    ros2 launch display.launch.py display:=false      # ohne Tkinter-Display
    ros2 launch display.launch.py base_frame:=link_1  # Bezugsframe des Displays
    ros2 launch display.launch.py urdf:=/pfad/zur/anderen.urdf
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Verzeichnis dieser Launch-Datei -> alle Pfade sind davon abgeleitet,
# damit der Start unabhaengig vom aktuellen Arbeitsverzeichnis funktioniert.
HERE = os.path.dirname(os.path.realpath(__file__))


def launch_setup(context, *args, **kwargs):
    urdf_path = LaunchConfiguration("urdf").perform(context)
    rviz_path = LaunchConfiguration("rviz_config").perform(context)
    script_path = LaunchConfiguration("display_script").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context)

    # URDF einlesen und als Parameter uebergeben (der frueher uebliche
    # Dateiname als Kommandozeilenargument ist in ROS 2 deprecated).
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Das Tkinter-Programm liegt nicht in einem ROS-Package, wird also
    # direkt als Prozess gestartet.
    robot_display = ExecuteProcess(
        cmd=[
            sys.executable,
            script_path,
            "--ros-args",
            "-p", "start_state_publisher:=false",
            "-p", f"base_frame:={base_frame}",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("display")),
    )

    return [robot_state_publisher, joint_state_publisher_gui, rviz2, robot_display]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf",
            default_value=os.path.join(HERE, "bmKinematic.urdf"),
            description="Pfad zur URDF-Datei",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(HERE, "bmKinematic.rviz"),
            description="Pfad zur RViz-Konfiguration",
        ),
        DeclareLaunchArgument(
            "display_script",
            default_value=os.path.join(HERE, "robot_display_umsetzung_4.py"),
            description="Pfad zum Tkinter-Display",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="Bezugsframe fuer die Frame-Tabelle im Display",
        ),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("display", default_value="true"),

        OpaqueFunction(function=launch_setup),
    ])
