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
  ros2 launch botzilla_navigation rtabmap.launch.py use_sim_time:=false depth_topic:=/camera/depth/image_meters
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
        # Turn-triggered ghost-map fix: RTAB-Map only processes one keyframe/second
        # (Rate=1.00s fixed), so a turn at even a modest ~0.3 rad/s puts ~17 degrees
        # between consecutive keyframes. Confirmed on hardware via --udebug logging
        # (rtabmap_debug.launch.py) that this legitimately exceeds ICP's default
        # sanity bounds during a real, correct turn — not a registration failure:
        #   "libpointmatcher has failed: limit out of bounds: rot: 0.18/0.78 tr: 0.24/0.2"
        #   "Cannot compute transform (cor=15 corrRatio=0.062/0.100 maxLaserScans=243)"
        # -> "Odometry refining rejected", falling back to the raw unrefined odometry
        # link between those two nodes instead of a properly ICP-registered one, which
        # is what produced the wall duplication/ghosting after a turn. Widening these
        # bounds lets legitimate large-turn corrections through without disabling the
        # sanity check outright (it still rejects truly wild/divergent ICP results).
        'Icp/MaxTranslation': '0.5',        # was default 0.2 — observed correction 0.244
        'Icp/MaxRotation': '1.57',          # was default 0.78 (~45 deg) — now ~90 deg
        'Icp/CorrespondenceRatio': '0.05',  # was default 0.1 — observed ratio 0.062
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
        arguments=[rtabmap_args],
    )

    return LaunchDescription([
        use_sim_time_arg,
        delete_db_on_start_arg,
        depth_topic_arg,
        rtabmap_node,
    ])
