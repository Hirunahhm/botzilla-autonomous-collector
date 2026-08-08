"""
frontier_explorer.launch.py.

Milestone 4 (PHASE_1_IMPLEMENTATION_PLAN.md): launches frontier_explorer_node standalone.
Requires simulation.launch.py, rtabmap.launch.py, and nav2.launch.py to already be running
(this file only launches the exploration node itself).

Usage:
  ros2 launch botzilla_navigation frontier_explorer.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    frontier_explorer_node = Node(
        package='botzilla_navigation',
        executable='frontier_explorer_node',
        name='frontier_explorer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        frontier_explorer_node,
    ])
