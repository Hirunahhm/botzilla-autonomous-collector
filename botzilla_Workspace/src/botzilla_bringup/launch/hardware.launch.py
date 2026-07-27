"""
hardware.launch.py

Launches the robot + sensor + localization stack needed to run RTAB-Map (or any
other SLAM/Nav2 layer) on the physical BotZilla robot. Mirrors the scope of
simulation.launch.py: robot description, sensors, and localization — no
SLAM/Nav2/perception/brain nodes here, those are launched separately on top
(see botzilla_navigation/launch/rtabmap.launch.py, used with use_sim_time:=false).

Nodes started:
  1. robot_state_publisher — publishes /tf tree from botzilla_qbot.urdf
  2. kinect_bridge         — reads physical Kinect camera & publishes RGB + Depth
                              (+ /camera/camera_info) — needs LD_PRELOAD=noreset.so,
                              confirmed required on Jetson (see PROJECT.md)
  3. kobuki_base_node      — listens to /cmd_vel, drives wheels, publishes
                              /odom + /imu (rate gyro only)
  4. rplidar_node          — RPLIDAR C1 pyserial driver, publishes /scan
                              (bypasses the upstream rplidar_ros package — see
                              botzilla_perception/rplidar_driver.py's docstring
                              for why)
  5. odom_covariance_relay — republishes /odom as /odom_cov with realistic
                              covariance (kobuki_base_node's raw /odom has zero
                              covariance, which robot_localization reads as
                              "infinitely certain" and never fuses)
  6. ekf_node              — robot_localization, fuses wheel velocity + IMU yaw
                              rate into /odometry/filtered (config/ekf_hardware.yaml
                              — see that file for why it differs from sim's ekf.yaml)

Usage:
  ros2 launch botzilla_bringup hardware.launch.py serial_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
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
        get_package_share_directory('botzilla_navigation'), 'config', 'ekf_hardware.yaml'
    )

    # ------------------------------------------------------------------ #
    # Launch arguments
    # ------------------------------------------------------------------ #
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the Kobuki base (e.g. /dev/ttyUSB0)',
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for the RPLIDAR C1 (e.g. /dev/ttyUSB1)',
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
    #      /camera/camera_info     (sensor_msgs/CameraInfo)
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
    # 3. Kobuki Base Node — subscribes to /cmd_vel, drives motors,
    #    publishes /odom (wheel encoders) and /imu (rate gyro)
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

    # ------------------------------------------------------------------ #
    # 4. RPLIDAR C1 — publishes /scan
    # ------------------------------------------------------------------ #
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

    # ------------------------------------------------------------------ #
    # 5. Odom covariance relay — see module docstring
    # ------------------------------------------------------------------ #
    odom_covariance_relay = Node(
        package='botzilla_navigation',
        executable='odom_covariance_relay',
        name='odom_covariance_relay',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ------------------------------------------------------------------ #
    # 6. EKF — fuses /odom_cov (wheel velocity) + /imu (yaw rate) into
    #    /odometry/filtered, publishes odom -> base_footprint TF
    # ------------------------------------------------------------------ #
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': False}],
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
