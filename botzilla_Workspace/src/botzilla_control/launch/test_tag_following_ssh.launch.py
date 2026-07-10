"""
Tag Following Test — SSH / headless variant
============================================
Same as test_tag_following.launch.py but runs apriltag_node with
QT_QPA_PLATFORM=offscreen so cv2.imshow does not crash when there
is no connected display (e.g. SSH from VS Code).

How to test:
  1. Run this launch file:
       ros2 launch botzilla_control test_tag_following_ssh.launch.py

  2. In a second terminal, monitor tag detection and robot commands:
       ros2 topic echo /drop_off_visible
       ros2 topic echo /drop_off_pose
       ros2 topic echo /cmd_vel

  3. Point the Kinect at the AprilTag. The robot should:
       - Turn to centre the tag (x offset → 0)
       - Drive forward once aligned
       - Stop when close (depth → 0.0 = blind spot)

Debug commands (second terminal):
  ros2 topic hz /camera/rgb/image_raw        # should be ~30 Hz
  ros2 topic hz /drop_off_pose               # publishes when tag visible
  ros2 topic echo /drop_off_visible          # True / False
  ros2 topic echo /drop_off_pose             # x offset
  ros2 topic echo /cmd_vel                   # linear/angular velocity commands
"""
import os
from launch import LaunchDescription
from launch.actions import LogInfoilding the full autonomy pipeline for cube collection and AprilTag delivery. Run colcon build
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
        additional_env={'QT_QPA_PLATFORM': 'offscreen'},
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
        LogInfo(msg='[test_tag_following_ssh] Starting headless tag-following pipeline...'),
        LogInfo(msg='[test_tag_following_ssh] Nodes: kinect_bridge | apriltag_node | tag_follower | kobuki_base'),
        LogInfo(msg='[test_tag_following_ssh] Monitor: ros2 topic echo /drop_off_visible'),
        LogInfo(msg='[test_tag_following_ssh] Monitor: ros2 topic echo /cmd_vel'),
        kinect_bridge,
        apriltag_node,
        tag_follower,
        kobuki_base,
    ])
