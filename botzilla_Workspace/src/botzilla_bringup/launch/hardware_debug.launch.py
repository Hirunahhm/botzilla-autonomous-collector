"""
hardware_debug.launch.py

Debug variant of hardware.launch.py — identical node set, but:
  - kobuki_base_node and ekf_node run at ROS log level DEBUG (surfaces
    kobuki_base_node's per-cmd_vel debug line and any rclcpp-level detail).
  - ekf_node uses ekf_hardware_debug.yaml instead of ekf_hardware.yaml, which
    turns on robot_localization's own built-in verbose diagnostics
    (debug: true) — every filter cycle (prediction + each sensor correction,
    with the raw measurement, innovation, and resulting state) gets written
    to /tmp/claude-1000/hwtest/ekf_debug.txt.

Built for the "URDF flips between 0 and 180 degrees while stationary, no
goal given" investigation — pairs with rtabmap_debug.launch.py (--udebug)
to see both ends of the pipeline: what the EKF is actually producing each
cycle, and what RTAB-Map does with it.

Usage:
  mkdir -p /tmp/claude-1000/hwtest
  ros2 launch botzilla_bringup hardware_debug.launch.py
  # defaults point at /dev/serial/by-id/... (stable per-device symlinks, immune to
  # ttyUSB0/ttyUSB1 enumeration order flipping on reboot/replug) — override only if
  # this exact Kobuki/RPLIDAR unit pair changes:
  #   ros2 launch botzilla_bringup hardware_debug.launch.py serial_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
  # then, separately:
  ros2 launch botzilla_navigation rtabmap_debug.launch.py use_sim_time:=false depth_topic:=/camera/depth/image_meters
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_NORESET = os.path.join(
    os.path.expanduser('~'), 'Desktop/Projects/sem5/final-project-botzilla/noreset.so'
)


def generate_launch_description():
    pkg_share = get_package_share_directory('botzilla_bringup')
    urdf_file = os.path.join(pkg_share, 'description', 'botzilla_qbot.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    ekf_config_file = os.path.join(
        get_package_share_directory('botzilla_navigation'), 'config', 'ekf_hardware_debug.yaml'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=(
            '/dev/serial/by-id/'
            'usb-Yujin_Robot_iClebo_Kobuki_kobuki_AI02MTI8-if00-port0'
        ),
        description='Serial port for the Kobuki base',
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value=(
            '/dev/serial/by-id/'
            'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
            'fed4f56bdc6ff011b4f08f301045c30f-if00-port0'
        ),
        description='Serial port for the RPLIDAR C1',
    )

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

    kinect_bridge = Node(
        package='botzilla_perception',
        executable='kinect_bridge',
        name='kinect_bridge',
        output='screen',
        parameters=[{'use_sim_time': False}],
        additional_env={'LD_PRELOAD': _NORESET},
    )

    kobuki_base_node = Node(
        package='botzilla_control',
        executable='kobuki_base_node',
        name='kobuki_base_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'serial_port': LaunchConfiguration('serial_port')
        }],
        arguments=['--ros-args', '--log-level', 'kobuki_base_node:=debug'],
    )

    rplidar_node = Node(
        package='botzilla_perception',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'port': LaunchConfiguration('lidar_port'),
        }],
    )

    odom_covariance_relay = Node(
        package='botzilla_navigation',
        executable='odom_covariance_relay',
        name='odom_covariance_relay',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': False}],
        # No --log-level debug here: robot_localization's own debug_out_file (set in
        # ekf_hardware_debug.yaml) gives far more useful per-cycle filter detail than
        # rclcpp's generic DEBUG level, which is dominated by rcl/DDS internals.
    )

    return LaunchDescription([
        serial_port_arg,
        lidar_port_arg,
        robot_state_publisher,
        kinect_bridge,
        kobuki_base_node,
        rplidar_node,
        odom_covariance_relay,
        ekf_node,
    ])
