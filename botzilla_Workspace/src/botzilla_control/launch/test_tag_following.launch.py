import os
from launch import LaunchDescription
from launch_ros.actions import Node

_NORESET = os.path.join(os.path.expanduser('~'), 'Desktop/Bozilla-ws/final-project-botzilla/noreset.so')


def generate_launch_description():
    kinect_bridge = Node(
        package='botzilla_perception',
        executable='kinect_bridge',
        name='kinect_bridge',
        output='screen',
        additional_env={'LD_PRELOAD': _NORESET},
    )

    apriltag_node = Node(
        package='botzilla_perception',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
    )

    tag_follower = Node(
        package='botzilla_control',
        executable='tag_follower_node',
        name='tag_follower',
        output='screen',
    )

    kobuki_base = Node(
        package='botzilla_control',
        executable='kobuki_base_node',
        name='kobuki_base',
        output='screen',
    )

    return LaunchDescription([
        kinect_bridge,
        apriltag_node,
        tag_follower,
        kobuki_base
    ])
