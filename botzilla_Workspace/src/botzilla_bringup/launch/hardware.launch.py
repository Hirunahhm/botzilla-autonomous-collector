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
  7. depth_image_proc      — converts /camera/depth/image_meters (real metric depth)
                              into a 3D point cloud on /camera/points, for detecting
                              low obstacles (chair/wheelchair bases) the LiDAR's 2D
                              scan plane misses entirely.
  8. pointcloud_to_laserscan — crushes /camera/points into a virtual 2D scan on
                              /scan_camera, height-filtered to 0.05m-0.30m (above
                              floor-noise level, up to the LiDAR's own 0.30m mount
                              height — see botzilla_navigation/config/nav2_params.yaml
                              for how this feeds the costmaps as a second
                              observation source alongside /scan).

Usage:
  ros2 launch botzilla_bringup hardware.launch.py
  # defaults point at /dev/serial/by-id/... (stable per-device symlinks, immune to
  # ttyUSB0/ttyUSB1 enumeration order flipping on reboot/replug) — override only if
  # this exact Kobuki/RPLIDAR unit pair changes:
  #   ros2 launch botzilla_bringup hardware.launch.py serial_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

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

    # ------------------------------------------------------------------ #
    # 7. depth_image_proc — /camera/depth/image_meters (real metric depth,
    #    NOT /camera/depth/image_raw, which is kinect_bridge's mono8 preview
    #    built for yolo_node) -> /camera/points (3D point cloud)
    # ------------------------------------------------------------------ #
    depth_to_pointcloud = ComposableNodeContainer(
        name='depth_image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                remappings=[
                    ('image_rect', '/camera/depth/image_meters'),
                    # image_transport::CameraSubscriber derives the info topic from the
                    # image topic's own namespace (here: /camera/depth/camera_info), NOT
                    # from a remap targeting the generic 'camera_info' name — confirmed
                    # live: with only the 'camera_info' remap, point_cloud_xyz_node kept
                    # subscribing to /camera/depth/camera_info (0 messages, since
                    # kinect_bridge only publishes /camera/camera_info) and silently
                    # produced zero synchronized pairs. Must remap the actual resolved
                    # topic name.
                    ('/camera/depth/camera_info', '/camera/camera_info'),
                    ('points', '/camera/points'),
                ],
                parameters=[{'use_sim_time': False}],
            )
        ],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # 8. pointcloud_to_laserscan — /camera/points -> /scan_camera, a virtual
    #    2D scan covering 0.05m-0.30m height (low obstacles like chair/
    #    wheelchair bases the LiDAR's scan plane misses; 0.30m matches the
    #    LiDAR's own mount height, so the two sensors cover ground-to-LiDAR
    #    with no gap). Fed into nav2's costmaps as a second observation
    #    source alongside /scan (see botzilla_navigation/config/nav2_params.yaml).
    # ------------------------------------------------------------------ #
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/camera/points'),
            ('scan', '/scan_camera'),
        ],
        parameters=[{
            'use_sim_time': False,
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.05,
            'max_height': 0.30,
            'angle_min': -0.5,
            'angle_max': 0.5,
            'range_min': 0.55,
            'range_max': 3.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
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
        depth_to_pointcloud,
        pointcloud_to_laserscan,
    ])
