import os
from launch import LaunchDescription
from launch_ros.actions import Node

_NORESET = os.path.join(os.path.expanduser('~'), 'Desktop/Bozilla-ws/final-project-botzilla/noreset.so')


def generate_launch_description():
    """
    Launch the full BotZilla autonomy stack on hardware:
      1. kobuki_base_node  - serial driver: cmd_vel -> Kobuki wheel commands
      2. kinect_bridge      - Kinect RGB+Depth -> /camera/rgb/image_raw, /camera/depth/image_raw
      3. yolo_node          - YOLO + depth fusion -> /detected_cube, /perception/yolo_image
      4. brain_node         - State machine -> cmd_vel (SEARCHING/TARGETING/APPROACHING/CAPTURING/DELIVERING/DETACHING)
    """

    kobuki_base = Node(
        package='botzilla_control',
        executable='kobuki_base_node',
        name='kobuki_base_node',
        output='screen',
    )

    kinect_bridge = Node(
        package='botzilla_perception',
        executable='kinect_bridge',
        name='kinect_bridge',
        output='screen',
        additional_env={'LD_PRELOAD': _NORESET},
    )

    yolo_node = Node(
        package='botzilla_perception',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
    )

    apriltag_node = Node(
        package='botzilla_perception',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
    )

    brain_node = Node(
        package='botzilla_control',
        executable='brain_node',
        name='botzilla_brain',
        output='screen',
    )

    return LaunchDescription([
        kobuki_base,
        kinect_bridge,
        yolo_node,
        apriltag_node,
        brain_node,
    ])
