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
from launch_ros.parameter_descriptions import ParameterValue


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

    grid_sensor_arg = DeclareLaunchArgument(
        'grid_sensor',
        default_value='2',
        description=(
            'Which sensor builds the occupancy grid: 0=laser scan only, 1=depth camera '
            'only, 2=both. Exposed as an argument so the depth camera can be isolated '
            'from the grid without editing this file — the depth contribution is a '
            'prime suspect whenever obstacles appear to sweep around with the robot, '
            'since with Grid/NormalsSegmentation=false the ground/obstacle split is '
            'purely by height and a camera pose error paints the floor as a wall.'
        ),
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    depth_topic = LaunchConfiguration('depth_topic')
    grid_sensor = LaunchConfiguration('grid_sensor')

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
        # value_type=str is required: RTAB-Map's own parameters are all strings, but a
        # bare LaunchConfiguration substitution would be auto-typed as an integer here.
        'Grid/Sensor': ParameterValue(grid_sensor, value_type=str),
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
        # Ghosting during turns traced (by measurement, not guesswork) to sensor
        # timestamping, NOT to registration tuning: rplidar_node and kinect_bridge both
        # stamped messages at publish time rather than capture time, so every scan
        # claimed to be ~115ms newer than its own data. Consumers resolved TF at that
        # later time and transformed the scan by a pose the robot only reached
        # afterwards — during a 0.4 rad/s turn that is ~2.7 deg, ~24cm of wall
        # displacement at 5m, drawn as a second wall. Both nodes now stamp at capture.
        #
        # ICP bounds are deliberately left at their defaults. They were widened here
        # earlier in response to "Cannot compute transform (corrRatio=0.062/0.100)" —
        # but that poor correspondence was itself a symptom of the timestamp bug
        # misaligning consecutive scans. Loosening them treated the symptom and made
        # things worse: it let a spurious neighbor-link "correction" through with
        # falsely high confidence (odometry said ~0.0002m of motion, ICP claimed 0.163m
        # at variance=0.00016), which propagated straight into a bad map correction.
        # The defaults' rejection is the safe failure mode — it falls back to raw
        # odometry rather than corrupting the graph.
        #
        # Halving the inter-keyframe interval is kept: it genuinely reduces how far the
        # robot moves between processed frames, so the correct alignment stays the clear
        # ICP minimum rather than one of several plausible ones in a symmetric room.
        'Rtabmap/DetectionRate': '2',  # was default 1 (Hz)
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
        grid_sensor_arg,
        rtabmap_node,
    ])
