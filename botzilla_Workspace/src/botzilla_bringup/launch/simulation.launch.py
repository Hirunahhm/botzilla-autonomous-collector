"""
simulation.launch.py

Launches the full BotZilla Gazebo simulation:
  1. Gazebo Harmonic with the BotZilla arena world
  2. Robot State Publisher (URDF → /tf tree)
  3. Spawn the robot into Gazebo at the origin
  4. ros_gz_bridge to bridge Gazebo camera → ROS2 topics
  5. YOLO perception node (subscribes to /camera/rgb/image_raw)

Usage:
  ros2 launch botzilla_bringup simulation.launch.py
  ros2 launch botzilla_bringup simulation.launch.py gz_headless:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('botzilla_bringup')
    nav_share = get_package_share_directory('botzilla_navigation')

    # ------------------------------------------------------------------ #
    # Paths
    # ------------------------------------------------------------------ #
    world_file = os.path.join(pkg_share, 'worlds', 'botzilla_arena.world')
    urdf_file = os.path.join(pkg_share, 'description', 'botzilla_qbot.urdf')
    ekf_config_file = os.path.join(nav_share, 'config', 'ekf.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ------------------------------------------------------------------ #
    # Launch arguments
    # ------------------------------------------------------------------ #
    gz_headless_arg = DeclareLaunchArgument(
        'gz_headless',
        default_value='false',
        description='Run Gazebo without a GUI (headless mode)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )

    gz_headless = LaunchConfiguration('gz_headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ------------------------------------------------------------------ #
    # 1. Gazebo Harmonic (gz sim)
    # ------------------------------------------------------------------ #
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                PythonExpression(["'-s -r ' if 'true' == '", gz_headless, "' else '-r '"]),
                world_file
            ],
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # 2. Robot State Publisher (publishes /tf from URDF)
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # ------------------------------------------------------------------ #
    # 3. Spawn robot into Gazebo at origin
    # ------------------------------------------------------------------ #
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'botzilla_qbot',
            '-string', robot_description,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
        ],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # 4. ros_gz_bridge — bridge Gazebo topics → ROS2
    # ------------------------------------------------------------------ #
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        arguments=[
            # RGB camera image
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # Depth image
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            # Depth point cloud — the rgbd_camera sensor publishes this natively.
            # Bridged so local_costmap's voxel_layer can mark obstacles the 2D
            # lidar plane misses entirely (desk edges, table legs, anything above
            # or below the single scan plane). See nav2_params.yaml voxel_layer.
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # Camera info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # cmd_vel (ROS2 → Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry (Gazebo → ROS2)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # 2D LiDAR Scan (Gazebo → ROS2)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # IMU Data (Gazebo → ROS2)
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Simulation clock — required by every node running with
            # use_sim_time:=true (robot_state_publisher, rtabmap, tf2
            # listeners); without this their ROS clock stays frozen at
            # zero and all sim-time TF lookups silently fail.
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            # Remap Gazebo camera topic → topic the YOLO node expects
            ('/camera/image', '/camera/rgb/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
        ],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # 5. Sensor frame bridges & EKF
    # ------------------------------------------------------------------ #
    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_bridge',
        arguments=[
            '--frame-id', 'laser_frame',
            '--child-frame-id', 'botzilla_qbot/base_footprint/gpu_lidar',
        ],
    )

    camera_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_bridge',
        arguments=[
            '--frame-id', 'camera_link',
            '--child-frame-id', 'botzilla_qbot/base_footprint/rgbd_camera',
        ],
    )

    imu_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_frame_bridge',
        arguments=[
            '--frame-id', 'imu_link',
            '--child-frame-id', 'botzilla_qbot/base_footprint/imu_sensor',
        ],
    )

    # Gazebo's DiffDrive publishes an all-zero covariance matrix, which robot_localization
    # reads as "infinitely certain" and cannot fuse (wheel velocities were silently dropped
    # entirely). This relay republishes /odom as /odom_cov with realistic variances, which
    # is what the EKF actually consumes.
    odom_covariance_relay = Node(
        package='botzilla_navigation',
        executable='odom_covariance_relay',
        name='odom_covariance_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------ #
    # 6. YOLO Perception Node
    # ------------------------------------------------------------------ #
    yolo_node = Node(
        package='botzilla_perception',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        gz_headless_arg,
        use_sim_time_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
        lidar_frame_bridge,
        camera_frame_bridge,
        imu_frame_bridge,
        odom_covariance_relay,
        ekf_node,
        yolo_node,
    ])
