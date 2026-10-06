"""
collector.launch.py — the whole collector robot (the Raspberry Pi), under one namespace.

Starts, all in /<ns> with TF on /<ns>/tf:
  1. botzilla_bringup hardware.launch.py — Kobuki, Kinect, RPLIDAR, EKF, depth cloud
  2. AMCL, localising in the LEADER's /map (no SLAM on the collector)
  3. Nav2 (controller, planner, behaviours, bt_navigator) + velocity_smoother, with the
     leader's nav2_params.yaml rewritten by params_rewrite.py
  4. cube detection, per detector:=
       leader (default)  remote_detection_node: JPEG to the leader's GPU yolo_node,
                         boxes back, depth paired here (see remote_detection.py; the
                         Pi measured 1.2 fps running YOLO itself under full load)
       local             yolo_node on the Pi's CPU (collector alone, no leader)
       none              no detector (bring-up tests)
  5. collector_node — waits for tasks from the leader's fleet_manager_node

Everything reaches the leader over the shared DDS graph: the collector's processes use
the leader's discovery server (run_collector.sh sets ROS_DISCOVERY_SERVER).

Why a namespace and a separate TF topic: both robots publish the same topic names
(/scan, /cmd_vel, /odom...) and the same frame names (odom, base_link...). Under /bz2
with /bz2/tf they cannot see each other's, and the collector's frames need no prefix.
Its map frame is the leader's map frame by construction: AMCL publishes map->odom
against the leader's /map, starting from the measured start offset below.

Usage (normally via run_collector.sh):
  ros2 launch botzilla_fleet collector.launch.py start_x:=0.0 start_y:=-0.8 start_yaw:=0.0
"""
import glob
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from botzilla_fleet.params_rewrite import collector_nav2_params, namespaced
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap
import yaml

KOBUKI_GLOBS = ('/dev/serial/by-id/*Kobuki*', '/dev/serial/by-id/*Yujin*')
# The collector's RPLIDAR C1 sits behind a CH340 USB adapter (1a86:7523), not the
# CP2102N the leader's has; both are listed so either robot's LiDAR is found.
LIDAR_GLOBS = ('/dev/serial/by-id/*CP2102N*', '/dev/serial/by-id/*Silicon_Labs*',
               '/dev/serial/by-id/usb-1a86_USB_Serial*', '/dev/serial/by-id/*CH340*')


def _find_port(given, patterns, what):
    """Return the explicit port, else the first by-id match; never a guessed ttyUSBn.

    The ttyUSB numbering depends on plug order: on the collector Pi ttyUSB0 is the LiDAR
    and ttyUSB1 the Kobuki, the reverse of the old defaults, and a driver opened on the
    wrong device fails in confusing ways.
    """
    if given:
        return given
    for pattern in patterns:
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    present = sorted(glob.glob('/dev/serial/by-id/*'))
    raise RuntimeError(
        f'{what} not found by id (looked for {patterns}); serial devices present: '
        f'{present or "none"}. Plug it in, or pass its port explicitly.'
    )


def _write_yaml(data, prefix):
    fd, path = tempfile.mkstemp(prefix=prefix, suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
    return path


def _launch(context, *_args, **_kwargs):
    def arg(name):
        return context.launch_configurations[name]

    ns = arg('ns').strip('/')
    nav_share = get_package_share_directory('botzilla_navigation')
    fleet_share = get_package_share_directory('botzilla_fleet')
    bringup_share = get_package_share_directory('botzilla_bringup')

    with open(os.path.join(nav_share, 'config', 'nav2_params.yaml')) as f:
        nav2_base = yaml.safe_load(f)
    with open(os.path.join(fleet_share, 'config', 'amcl_collector.yaml')) as f:
        amcl = yaml.safe_load(f)
    amcl_p = amcl['amcl']['ros__parameters']
    amcl_p['initial_pose'] = {
        'x': float(arg('start_x')), 'y': float(arg('start_y')), 'z': 0.0,
        'yaw': float(arg('start_yaw')),
    }
    nav2_file = _write_yaml(namespaced(collector_nav2_params(nav2_base, ns, amcl), ns),
                            f'{ns}_nav2_')
    with open(os.path.join(nav_share, 'config', 'ekf_hardware.yaml')) as f:
        ekf_file = _write_yaml(namespaced(yaml.safe_load(f), ns), f'{ns}_ekf_')

    kobuki = _find_port(arg('serial_port'), KOBUKI_GLOBS, 'Kobuki')
    lidar = _find_port(arg('lidar_port'), LIDAR_GLOBS, 'RPLIDAR')

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'hardware.launch.py')),
        launch_arguments={
            'serial_port': kobuki,
            'lidar_port': lidar,
            'ekf_params_file': ekf_file,
            'noreset_path': arg('noreset_path'),
            # Depth aligned to RGB: unregistered, this Kinect's depth for a cube 0.9 m
            # away fell ~30 px beside the cube's RGB box (see kinect_bridge).
            'depth_registered': 'true',
            # The collector carries the grabber arms (the leader no longer does).
            'arms': 'true',
        }.items(),
    )

    default_bt = os.path.join(nav_share, 'behavior_trees', 'navigate_to_pose_backup_first.xml')
    velocity_publishers = {'controller_server', 'behavior_server'}
    nav_nodes = []
    for pkg, exe in (('nav2_controller', 'controller_server'),
                     ('nav2_planner', 'planner_server'),
                     ('nav2_behaviors', 'behavior_server'),
                     ('nav2_bt_navigator', 'bt_navigator')):
        params = [nav2_file]
        if exe == 'bt_navigator':
            params.append({'default_nav_to_pose_bt_xml': default_bt})
        nav_nodes.append(Node(
            package=pkg, executable=exe, name=exe, output='screen', parameters=params,
            remappings=[('cmd_vel', 'cmd_vel_nav')] if exe in velocity_publishers else [],
        ))
    nav_nodes += [
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[nav2_file]),
        Node(package='botzilla_control', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[{'use_sim_time': False}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{
                 'use_sim_time': False, 'autostart': True,
                 # The default 4 s is too short on the Pi: AMCL's activation waits for
                 # and converts the leader's map, and on the loaded Pi that exceeded
                 # 4 s, so the manager declared AMCL dead and aborted the whole Nav2
                 # bring-up (run_logs/collector-20261005-184906).
                 'bond_timeout': 20.0,
                 'node_names': ['amcl', 'controller_server', 'planner_server',
                                'behavior_server', 'bt_navigator'],
             }]),
    ]

    mission = [Node(
        package='botzilla_fleet', executable='collector_node', name='collector_node',
        output='screen', parameters=[{'use_sim_time': False, 'robot_name': ns}],
    )]
    detector = arg('detector')
    if detector not in ('leader', 'local', 'none'):
        raise RuntimeError(f"detector must be leader, local or none (got '{detector}')")
    if detector == 'leader':
        mission.insert(0, Node(
            package='botzilla_fleet', executable='remote_detection_node',
            name='remote_detection_node', output='screen',
            parameters=[{'use_sim_time': False}],
        ))
    elif detector == 'local':
        mission.insert(0, Node(
            package='botzilla_perception', executable='yolo_node', name='yolo_node',
            output='screen', parameters=[{'use_sim_time': False}],
            # Headless Pi: a cv2 window would abort Qt (see yolo_node's imshow guard).
            additional_env={'QT_QPA_PLATFORM': 'offscreen'},
        ))

    return [
        LogInfo(msg=f'[collector] ns=/{ns} kobuki={kobuki} lidar={lidar} '
                    f'start=({arg("start_x")}, {arg("start_y")}, {arg("start_yaw")}) '
                    f'detector={detector}'),
        LogInfo(msg=f'[collector] nav2 params: {nav2_file}'),
        GroupAction([
            PushRosNamespace(ns),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            hardware,
            *nav_nodes,
            *mission,
        ]),
    ]


def generate_launch_description():
    noreset_default = os.path.join(
        os.path.expanduser('~'), 'hiruna/botzilla-autonomous-collector/noreset.so')
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='bz2',
                              description='Namespace for every collector node and topic'),
        DeclareLaunchArgument(
            'start_x', default_value='0.0',
            description="Collector start pose in the LEADER's map frame (= the offset "
                        'from the leader start spot), metres'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0', description='radians'),
        DeclareLaunchArgument('serial_port', default_value='',
                              description='Kobuki port; empty = find it by id'),
        DeclareLaunchArgument('lidar_port', default_value='',
                              description='RPLIDAR port; empty = find it by id'),
        DeclareLaunchArgument('noreset_path', default_value=noreset_default,
                              description='Kinect LD_PRELOAD shim (needed on a Pi 5)'),
        DeclareLaunchArgument(
            'detector', default_value='leader',
            description="'leader' (YOLO on the leader's GPU), 'local' (YOLO on this CPU) "
                        "or 'none'"),
        OpaqueFunction(function=_launch),
    ])
