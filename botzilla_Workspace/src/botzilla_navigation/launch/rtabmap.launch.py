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
        # 1=Reliable, 2=BestEffort. Image/camera_info/scan must be BestEffort here to
        # match the hardware sensor nodes, which all publish with
        # rclpy.qos.qos_profile_sensor_data (BestEffort) — a Reliable subscriber cannot
        # receive from a BestEffort publisher at all (QoS incompatibility, not just a
        # warning), so this was silently dropping every rgb/depth/scan frame on hardware.
        # This was never an issue in sim, where ros_gz_bridge's default topic QoS is
        # Reliable. /odometry/filtered (odom) is unaffected — robot_localization
        # publishes with the default Reliable QoS, matching qos_odom=1 below.
        'qos_image': 2,
        'qos_camera_info': 2,
        'qos_scan': 2,
        'qos_odom': 1,
        'Reg/Strategy': '1',       # ICP + Visual (best obstacle avoidance + loop closure)
        'Reg/Force3DoF': 'true',   # ground robot: x, y, yaw only
        'Grid/RangeMax': '10.0',
        'Grid/Sensor': '2',        # both laser + depth for 3D point cloud
        # Ghost-map hardening (overlapping/smeared occupancy layers during arcs):
        'Grid/RayTracing': 'true',           # clear free cells along each beam so stale marks
                                             # from earlier poses don't persist as extra layers
        'Grid/NormalsSegmentation': 'false', # flat sim floor -> use the deterministic height
                                             # passthrough instead of normal-based ground
                                             # segmentation (cheaper, and avoids frame-to-frame
                                             # normal-estimation flicker that smears the grid
                                             # during motion). Makes the two height filters
                                             # below the authoritative ground/obstacle split.
        'Grid/MaxGroundHeight': '0.05',      # points below 5cm = ground (not obstacle), so the
                                             # depth grid stops double-painting laser cells
        'Grid/MaxObstacleHeight': '0.6',     # cap at arena wall height; ignore ceiling/tall noise
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.3',
        'RGBD/LinearUpdate': '0.2',
    }

    rtabmap_remappings = [
        ('rgb/image', '/camera/rgb/image_raw'),
        ('depth/image', '/camera/depth/image_raw'),
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
        arguments=[rtabmap_args],
    )

    return LaunchDescription([
        use_sim_time_arg,
        delete_db_on_start_arg,
        rtabmap_node,
    ])
