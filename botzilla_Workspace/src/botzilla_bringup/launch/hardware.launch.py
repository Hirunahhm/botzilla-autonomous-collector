"""
hardware.launch.py

Launches all nodes needed for running on the real BotZilla robot:
  1. kinect_bridge  — reads Kinect camera and publishes to ROS2 topics
  2. kobuki_base_node — listens to /cmd_vel and drives the Kobuki wheels

Usage:
  ros2 launch botzilla_bringup hardware.launch.py

Prerequisites:
  - Kobuki QBot connected via USB (check: ls /dev/ttyUSB*)
  - Xbox Kinect connected via USB
  - sudo usermod -a -G dialout $USER  (if permission errors)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_NORESET = os.path.join(os.path.expanduser('~'), 'Desktop/Bozilla-ws/final-project-botzilla/noreset.so')


def generate_launch_description():

    # ------------------------------------------------------------------ #
    # Launch arguments
    # ------------------------------------------------------------------ #
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the Kobuki base (e.g. /dev/ttyUSB0)',
    )

    # ------------------------------------------------------------------ #
    # 1. Kinect Bridge — publishes:
    #      /camera/rgb/image_raw   (sensor_msgs/Image)
    #      /camera/depth/image_raw (sensor_msgs/Image)
    # ------------------------------------------------------------------ #
    kinect_bridge = Node(
        package='botzilla_perception',
        executable='kinect_bridge',
        name='kinect_bridge',
        output='screen',
        parameters=[{'use_sim_time': False}],
        additional_env={'LD_PRELOAD': _NORESET},
    )

    # ------------------------------------------------------------------ #
    # 2. Kobuki Base Node — subscribes to /cmd_vel and drives motors
    # ------------------------------------------------------------------ #
    kobuki_base_node = Node(
        package='botzilla_control',
        executable='kobuki_base_node',
        name='kobuki_base_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        serial_port_arg,
        kinect_bridge,
        kobuki_base_node,
    ])
