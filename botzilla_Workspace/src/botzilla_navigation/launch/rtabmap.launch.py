"""
rtabmap.launch.py

Milestone 1 (PHASE_1_IMPLEMENTATION_PLAN.md, Option A): brings up RTAB-Map
graph SLAM directly against the sim's RGB-D + LiDAR + wheel-odometry topics.

Deliberately skips rtabmap_odom's visual odometry node — the robot already
publishes wheel odometry (/odom, from the Gazebo diff-drive plugin), which is
cheaper to run on the Jetson than visual odometry. RTAB-Map fuses that wheel
odometry with RGB-D + laser scan data for mapping and loop closure.

Usage:
  ros2 launch botzilla_navigation rtabmap.launch.py
  ros2 launch botzilla_navigation rtabmap.launch.py delete_db_on_start:=false
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

    use_sim_time = LaunchConfiguration('use_sim_time')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')

    rtabmap_args = PythonExpression([
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
        'qos_image': 1,
        'qos_scan': 1,
        'qos_odom': 1,
        'Reg/Strategy': '1',       # ICP + Visual (scan-assisted registration)
        'Reg/Force3DoF': 'true',   # ground robot, no need for full 6DoF
        'Grid/RangeMax': '10.0',
        'Grid/Sensor': '2',        # 0=laser only, 1=depth only, 2=both laser and depth combined for 3D point cloud
        'RGBD/NeighborLinkRefining': 'true',
    }

    rtabmap_remappings = [
        ('rgb/image', '/camera/rgb/image_raw'),
        ('depth/image', '/camera/depth/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('odom', '/odom'),
        ('scan', '/scan'),
    ]

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remappings,
        arguments=[rtabmap_args],
    )

    return LaunchDescription([
        use_sim_time_arg,
        delete_db_on_start_arg,
        rtabmap_node,
    ])
