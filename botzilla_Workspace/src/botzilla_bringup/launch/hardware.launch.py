"""
hardware.launch.py

Launches all nodes needed for running on the physical BotZilla robot:
  1. robot_state_publisher — publishes /tf tree from botzilla_qbot.urdf
  2. kinect_bridge         — reads physical Kinect camera & publishes RGB + Depth
  3. kobuki_base_node      — listens to /cmd_vel, drives wheels, publishes /odom

Usage:
  ros2 launch botzilla_bringup hardware.launch.py serial_port:=/dev/ttyUSB0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('botzilla_bringup')
    urdf_file = os.path.join(pkg_share, 'description', 'botzilla_qbot.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ------------------------------------------------------------------ #
    # Launch arguments
    # ------------------------------------------------------------------ #
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the Kobuki base (e.g. /dev/ttyUSB0)',
    )

    # ------------------------------------------------------------------ #
    # 1. Robot State Publisher (publishes /tf tree for hardware sensors)
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # ------------------------------------------------------------------ #
    # 2. Kinect Bridge — publishes:
    #      /camera/rgb/image_raw   (sensor_msgs/Image)
    #      /camera/depth/image_raw (sensor_msgs/Image)
    # ------------------------------------------------------------------ #
    kinect_bridge = Node(
        package='botzilla_perception',
        executable='kinect_bridge',
        name='kinect_bridge',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ------------------------------------------------------------------ #
    # 3. Kobuki Base Node — subscribes to /cmd_vel and drives motors
    # ------------------------------------------------------------------ #
    kobuki_base_node = Node(
        package='botzilla_control',
        executable='kobuki_base_node',
        name='kobuki_base_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'serial_port': LaunchConfiguration('serial_port')
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        robot_state_publisher,
        kinect_bridge,
        kobuki_base_node,
    ])
