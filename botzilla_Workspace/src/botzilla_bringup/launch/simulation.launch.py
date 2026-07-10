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

    # ------------------------------------------------------------------ #
    # Paths
    # ------------------------------------------------------------------ #
    world_file = os.path.join(pkg_share, 'worlds', 'botzilla_arena.world')
    urdf_file = os.path.join(pkg_share, 'description', 'botzilla_qbot.urdf')

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
    #
    #   Gazebo RGBD camera publishes to:
    #     /camera/image          (gz.msgs.Image)         → /camera/rgb/image_raw  (sensor_msgs/Image)
    #     /camera/depth_image    (gz.msgs.Image)         → /camera/depth/image_raw (sensor_msgs/Image)
    #     /camera/camera_info    (gz.msgs.CameraInfo)    → /camera/camera_info (sensor_msgs/CameraInfo)
    #
    #   cmd_vel bridge:
    #     /cmd_vel (geometry_msgs/Twist) → /cmd_vel (gz.msgs.Twist)   [bidirectional]
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
            # Camera info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # cmd_vel (ROS2 → Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry (Gazebo → ROS2)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # 2D LiDAR Scan (Gazebo → ROS2)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # TF from diff drive plugin
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            # Remap Gazebo camera topic → topic the YOLO node expects
            ('/camera/image', '/camera/rgb/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
        ],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # 5. YOLO Perception Node
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
        yolo_node,
    ])
