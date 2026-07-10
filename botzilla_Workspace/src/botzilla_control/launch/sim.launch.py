import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define absolute paths based on your workspace structure
    home = os.path.expanduser('~')
    base_proj_path = os.path.join(home, 'Desktop/Bozilla-ws/final-project-botzilla')
    world_path = os.path.join(base_proj_path, 'worlds', 'botzilla_arena.world')
    urdf_path = os.path.join(base_proj_path, 'description', 'botzilla_qbot.urdf')

    # 1. Start Gazebo Sim with your world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # 2. Spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', urdf_path, '-name', 'botzilla_qbot', '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # 3. Start the ROS-Gazebo Bridge for cmd_vel
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        output='screen'
    )

    # 4. Start your FSM Brain Node
    brain_node = Node(
        package='botzilla_control',
        executable='brain_node',
        output='screen'
    )

    # 5. Start Perception Simulator
    perception_sim = Node(
        package='botzilla_control',
        executable='perception_simulator',
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        spawn_entity,
        bridge,
        brain_node,
        perception_sim # Add it here
    ])