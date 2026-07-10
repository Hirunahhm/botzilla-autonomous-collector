import os
from launch import LaunchDescription
from launch_ros.actions import Node

_NORESET = os.path.join(os.path.expanduser('~'), 'Desktop/Bozilla-ws/final-project-botzilla/noreset.so')


def generate_launch_description():
    """
    TEST 6b: Cube Following Test — SSH / headless variant
    ======================================================
    Same as test_cube_following.launch.py but runs yolo_node with
    QT_QPA_PLATFORM=offscreen so cv2.imshow does not crash when there
    is no connected display (e.g. SSH from VS Code).

    Annotated detections are still published to /perception/yolo_image
    and can be viewed with rqt_image_view on a machine that has a display.
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
        additional_env={'QT_QPA_PLATFORM': 'offscreen'},
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
