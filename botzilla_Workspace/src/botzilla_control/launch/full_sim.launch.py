import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
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

    # 3. Start the ROS-Gazebo Bridge (Odometry, Velocity, AND Cameras)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',             # RGB
            '/camera_depth@sensor_msgs/msg/Image@gz.msgs.Image',       # Depth
        ],
        remappings=[
            ('/camera', '/camera/rgb/image_raw'),
            ('/camera_depth', '/camera/depth/image_raw'),
        ],
        output='screen'
    )

    # 4. YOLO Perception Node
    yolo_node = Node(
        package='botzilla_perception',
        executable='yolo_node',
        name='yolo_node',
        output='screen'
    )

    # 5. AprilTag Perception Node
    apriltag_node = Node(
        package='botzilla_perception',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen'
    )

    # 6. FSM Brain Node
    brain_node = Node(
        package='botzilla_control',
        executable='brain_node',
        name='botzilla_brain',
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        spawn_entity,
        bridge,
        yolo_node,
        apriltag_node,
        brain_node
    ])
