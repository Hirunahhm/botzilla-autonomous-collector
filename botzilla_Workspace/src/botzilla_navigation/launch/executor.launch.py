"""Launch the Milestone 5 mission executor and the frontier explorer it drives.

executor.launch.py

The executor latches the robot's starting pose (map->base_link) as HOME, lets the
frontier explorer search the arena, interrupts it when yolo_node reports a cube,
collects the cube, and delivers it back to HOME via Nav2 before resuming.

This file starts only the two mission nodes. Bring these up first:

  # sim
  ros2 launch botzilla_bringup simulation.launch.py gz_headless:=true
  ros2 launch botzilla_navigation rtabmap.launch.py
  ros2 launch botzilla_navigation nav2.launch.py use_sim_time:=true
  ros2 launch botzilla_navigation executor.launch.py use_sim_time:=true

  # hardware
  ros2 launch botzilla_bringup hardware.launch.py
  ros2 launch botzilla_navigation rtabmap.launch.py use_sim_time:=false \
      depth_topic:=/camera/depth/image_meters
  ros2 launch botzilla_navigation nav2.launch.py
  ./docker/yolo/run_yolo_container.sh          # yolo_node, GPU container
  ros2 launch botzilla_navigation executor.launch.py

Important: place the robot where you want cubes delivered before starting this —
HOME is latched once, at startup, and never moves.

Monitor:
  ros2 topic echo /mission/status
  ros2 topic echo /detected_cube
  ros2 topic echo /exploration_enabled
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description=(
            'Use simulation (Gazebo) clock. Defaults false so a hardware run cannot '
            'silently freeze on a /clock that nobody publishes — pass true for sim.'
        ),
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Coverage-policy selector. 'fraction' is the shipped interleaved behaviour and the
    # default, so a plain `ros2 launch botzilla_navigation executor.launch.py` — and
    # run_full_mission.sh, which passes only use_sim_time — is unchanged. 'exhaustion'
    # is the explore-then-sweep baseline used for the "how much mapped floor did the
    # camera never inspect?" measurement. See frontier_explorer_node's sweep_trigger_mode.
    sweep_trigger_mode_arg = DeclareLaunchArgument(
        'sweep_trigger_mode',
        default_value='fraction',
        description="Coverage sweep policy: 'fraction' (interleaved) or 'exhaustion'.",
    )
    sweep_trigger_mode = LaunchConfiguration('sweep_trigger_mode')

    executor = Node(
        package='botzilla_navigation',
        executable='executor_node',
        name='executor_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    frontier_explorer = Node(
        package='botzilla_navigation',
        executable='frontier_explorer_node',
        name='frontier_explorer_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'sweep_trigger_mode': sweep_trigger_mode,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        sweep_trigger_mode_arg,
        LogInfo(msg='[executor] Mission: explore -> collect cube -> deliver to HOME'),
        LogInfo(msg='[executor] HOME is latched at startup from map->base_link.'),
        LogInfo(msg='[executor] Place the robot at the drop-off point before starting.'),
        executor,
        frontier_explorer,
    ])
