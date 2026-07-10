"""
TEST 3b: AprilTag Drop-off Zone Detection Test — SSH / headless variant
=======================================================================
Same as test_apriltag.launch.py but runs apriltag_node with
QT_QPA_PLATFORM=offscreen so cv2.imshow does not crash when there
is no connected display (e.g. SSH from VS Code).

Annotated detections are still published to /perception/apriltag_image
and can be viewed with rqt_image_view on a machine that has a display.

How to test:
  1. Run this launch file:
       ros2 launch botzilla_control test_apriltag_ssh.launch.py

  2. In a second terminal, echo the tag pose:
       ros2 topic echo /drop_off_pose

  3. Point the Kinect directly at the tag from ~1.5m away.
     Expect output like:
         x: ~0.0    (tag is centred in frame = robot is aimed correctly)
         y: <pixel height value>
         z: 0.0

  4. Move the robot slightly LEFT of the tag. Confirm x becomes POSITIVE.

  5. Echo /drop_off_visible to confirm it publishes True when tag is visible:
       ros2 topic echo /drop_off_visible

Debug commands (second terminal):
  ros2 topic hz /camera/rgb/image_raw        # should be ~30 Hz
  ros2 topic hz /drop_off_pose               # publishes when tag visible
  ros2 topic echo /drop_off_visible          # True / False
  ros2 topic echo /drop_off_pose             # x offset + tag centre y
"""
import os
from launch import LaunchDescription
from launch.actions import LogInfo
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

    return LaunchDescription([
        LogInfo(msg='[test_apriltag_ssh] Starting Kinect bridge + AprilTag node (headless)...'),
        LogInfo(msg='[test_apriltag_ssh] Watch for: "AprilTag Node started" then point camera at tag.'),
        LogInfo(msg='[test_apriltag_ssh] Check detections: ros2 topic echo /drop_off_pose'),
        kinect_bridge,
        apriltag_node,
    ])
