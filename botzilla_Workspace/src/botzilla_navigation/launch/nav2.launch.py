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

Usage (hardware is the default; pass use_sim_time:=true for Gazebo):
  ros2 launch botzilla_navigation nav2.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ])
