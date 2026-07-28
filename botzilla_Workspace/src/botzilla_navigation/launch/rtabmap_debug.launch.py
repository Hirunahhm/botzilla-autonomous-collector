"""
rtabmap_debug.launch.py

Debug variant of rtabmap.launch.py — identical mapping/registration
parameters, but with RTAB-Map's own verbose internal logging turned on
(--udebug) and log_to_rosout_level lowered so that detail also flows
through ROS logging, not just the raw process stdout.

Built for the turn-triggered ghost-map investigation: run this alongside
botzilla_control's rotation_test_node (a bounded, isolated rotation, no
Nav2 controller/planner involved) to get a clean, verbose log of exactly
what RTAB-Map's registration is doing frame-by-frame during a pure turn.

Usage:
  ros2 launch botzilla_navigation rtabmap_debug.launch.py use_sim_time:=false depth_topic:=/camera/depth/image_meters
  # in another terminal, once the stack above is up and stationary:
  ros2 run botzilla_control rotation_test_node
  # then inspect the newest ~/.ros/log/rtabmap_*.log for the [DEBUG] lines
  # spanning the rotation_test_node's start/stop timestamps.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )
    delete_db_on_start_arg = DeclareLaunchArgument(
        'delete_db_on_start',
        default_value='true',
        description='Start mapping from an empty database each launch (set false to keep building the same map across runs)',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth/image_raw',
        description=(
            "Depth image topic. Sim's ros_gz_bridge publishes proper metric depth "
            "directly on /camera/depth/image_raw (the default). On hardware, "
            "kinect_bridge's own /camera/depth/image_raw is a mono8 preview built for "
            "yolo_node, not real depth — pass /camera/depth/image_meters instead."
        ),
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    depth_topic = LaunchConfiguration('depth_topic')

    delete_db_arg = PythonExpression([
        "'--delete_db_on_start' if '", delete_db_on_start, "' == 'true' else ''"
    ])

    rtabmap_parameters = {
        'use_sim_time': use_sim_time,
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': True,
        'approx_sync': True,
        # log_to_rosout_level: 0=Debug, 1=Info, 2=Warn, 3=Error, 4=Fatal (RTAB-Map's own
        # scale, not rclpy's). Normal rtabmap.launch.py leaves this at the RTAB-Map
        # default (4, fatal-only over rosout) since verbose output is noisy for routine
        # runs — lowered here specifically to see registration detail during the turn.
        'log_to_rosout_level': 0,
        'qos_image': 2,
        'qos_camera_info': 2,
        'qos_scan': 2,
        'qos_odom': 1,
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'Grid/RangeMax': '10.0',
        'Grid/Sensor': '2',
        'Grid/RayTracing': 'true',
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '0.6',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.3',
        'RGBD/LinearUpdate': '0.2',
    }

    rtabmap_remappings = [
        ('rgb/image', '/camera/rgb/image_raw'),
        ('depth/image', depth_topic),
        ('rgb/camera_info', '/camera/camera_info'),
        ('odom', '/odometry/filtered'),
        ('scan', '/scan'),
    ]

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remappings,
        # --udebug: RTAB-Map's ULogger at DEBUG level — every registration attempt,
        # transform, and ICP/visual match detail gets printed, not just the one-line
        # per-second "rtabmap (N): Rate=... WM=..." summary the normal launch shows.
        arguments=[delete_db_arg, '--udebug'],
    )

    return LaunchDescription([
        use_sim_time_arg,
        delete_db_on_start_arg,
        depth_topic_arg,
        rtabmap_node,
    ])
