"""
botzilla_autonomy.launch.py

All-in-one autonomy launch. Choose a mode:

  Simulation mode (default):
    ros2 launch botzilla_bringup botzilla_autonomy.launch.py mode:=sim

  Hardware mode (real robot):
    ros2 launch botzilla_bringup botzilla_autonomy.launch.py mode:=hardware

  Perception only (YOLO, no robot):
    ros2 launch botzilla_bringup botzilla_autonomy.launch.py mode:=perception

This launch file includes simulation.launch.py or hardware.launch.py
depending on the chosen mode, then adds the YOLO perception node on top
(in hardware mode, since simulation.launch.py already starts it).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('botzilla_bringup')
    launch_dir = os.path.join(pkg_bringup, 'launch')

    # ------------------------------------------------------------------ #
    # Launch arguments
    # ------------------------------------------------------------------ #
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Launch mode: "sim" | "hardware" | "perception"',
        choices=['sim', 'hardware', 'perception'],
    )

    mode = LaunchConfiguration('mode')

    # ------------------------------------------------------------------ #
    # Conditions
    # ------------------------------------------------------------------ #
    is_sim = PythonExpression(["'", mode, "' == 'sim'"])
    is_hardware = PythonExpression(["'", mode, "' == 'hardware'"])
    is_perception = PythonExpression(["'", mode, "' == 'perception'"])

    # ------------------------------------------------------------------ #
    # Include simulation launch (sim mode only)
    # ------------------------------------------------------------------ #
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'simulation.launch.py')
        ),
        condition=IfCondition(is_sim),
    )

    # ------------------------------------------------------------------ #
    # Include hardware launch (hardware mode only)
    # ------------------------------------------------------------------ #
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'hardware.launch.py')
        ),
        condition=IfCondition(is_hardware),
    )

    # ------------------------------------------------------------------ #
    # YOLO node — started separately in hardware/perception mode.
    # In sim mode it is already started by simulation.launch.py.
    # ------------------------------------------------------------------ #
    yolo_hardware = Node(
        package='botzilla_perception',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        condition=IfCondition(is_hardware),
        parameters=[{'use_sim_time': False}],
    )

    yolo_perception_only = Node(
        package='botzilla_perception',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        condition=IfCondition(is_perception),
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        mode_arg,
        sim_launch,
        hardware_launch,
        yolo_hardware,
        yolo_perception_only,
    ])
