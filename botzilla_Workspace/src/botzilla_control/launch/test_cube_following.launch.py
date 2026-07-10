import os
from launch import LaunchDescription
from launch_ros.actions import Node

_NORESET = os.path.join(os.path.expanduser('~'), 'Desktop/Bozilla-ws/final-project-botzilla/noreset.so')


def generate_launch_description():
    """
    TEST 6: Cube Following Test (Brain Node Part 1)
    =============================================
    Tests that the robot can search for a cube, align with it,
    and drive toward it until it is grabbed.
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

    yolo_node = Node(
        package='botzilla_perception',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
    )

    cube_collector = Node(
        package='botzilla_control',
        executable='cube_collector',
        name='cube_collector',
        output='screen',
    )

    return LaunchDescription([
        kobuki_base,
        kinect_bridge,
        yolo_node,
        cube_collector,
    ])
