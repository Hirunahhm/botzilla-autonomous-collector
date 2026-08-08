"""
nav2.launch.py

Milestone 3 (PHASE_1_IMPLEMENTATION_PLAN.md, Option A): Nav2 goal-following on
top of the map RTAB-Map builds (rtabmap.launch.py must already be running and
publishing /map + map->odom TF — this file does not launch SLAM/localization).

A slim, hand-picked subset of nav2_bringup's navigation_launch.py — only the
nodes actually needed for a single NavigateToPose goal: controller_server,
planner_server, behavior_server, bt_navigator. Skips
route_server/collision_monitor/docking_server/smoother_server/waypoint_follower,
which navigation_launch.py always brings up regardless of whether they're
configured or needed, adding failure surface (e.g. route_server expects a
routing graph file we don't have) for no benefit at this milestone.

cmd_vel chain: controller_server/behavior_server publish directly on plain
'cmd_vel', which kobuki_base_node consumes. velocity_smoother used to sit in
between ('cmd_vel_nav' -> 'cmd_vel') but was removed: on hardware it silently
stopped republishing anything — controller_server kept publishing to
'cmd_vel_nav' while '/cmd_vel' had no publisher at all, with the node reporting
lifecycle state active and logging no error, so every navigation goal stalled.
The smoother is optional (DWB's own accel limits in nav2_params.yaml already
bound the output); if smoothing is wanted later, do it in a node we own and can
instrument rather than reintroducing a silent failure point.

Simulation parameters: passing use_sim_time:=true additionally layers
config/nav2_params_sim.yaml on top of the main params file (ROS 2 merges multiple
params files in order, last one winning). That overlay carries only the values that
must differ in Gazebo — see its own header — rather than a full duplicate config that
would drift out of sync with the hardware tuning. Applied automatically so it cannot
be forgotten: the values it corrects cause a stall that looks like a planner bug.

Usage (hardware is the default; pass use_sim_time:=true for Gazebo):
  ros2 launch botzilla_navigation nav2.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM_PARAMS_FILENAME = 'nav2_params_sim.yaml'


def _resolve_param_files(context):
    """Build the ordered params list, appending the sim overlay when running on sim time.

    Kept in an OpaqueFunction because the decision needs the *resolved* value of
    use_sim_time, which a LaunchConfiguration substitution cannot provide while the
    launch description is still being constructed.
    """
    pkg_share = get_package_share_directory('botzilla_navigation')
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in (
        'true', '1', 'yes'
    )
    files = [params_file]
    if use_sim_time:
        files.append(os.path.join(pkg_share, 'config', SIM_PARAMS_FILENAME))
    return files, use_sim_time


def _launch_nav2(context, *_args, **_kwargs):
    param_files, use_sim_time = _resolve_param_files(context)
    autostart = LaunchConfiguration('autostart')
    params = param_files + [{'use_sim_time': use_sim_time}]

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]
    servers = [
        ('nav2_controller', 'controller_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
    ]
    nodes = [
        Node(package=pkg, executable=exe, name=exe, output='screen', parameters=params)
        for pkg, exe in servers
    ]
    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
        }],
    ))
    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('botzilla_navigation')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description=(
            'Use simulation (Gazebo) clock. Defaults false: on hardware nothing publishes '
            '/clock, so a stray true freezes ROS time and every wall-timer-driven Nav2 node '
            'silently stops firing with no error logged.'
        ),
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the Nav2 parameters file',
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically bring the lifecycle nodes up to the active state',
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        OpaqueFunction(function=_launch_nav2),
    ])
