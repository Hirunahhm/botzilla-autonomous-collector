"""
TEST 3: AprilTag Drop-off Zone Detection Test
=============================================
Tests that the apriltag_node correctly detects the drop-off zone marker
and reports an accurate horizontal offset on /drop_off_pose.

Preparation:
  - Print the AprilTag ID 0 (DICT_APRILTAG_36h11 family) at full size.
    You can generate it with:
      python3 -c "
      import cv2, cv2.aruco as aruco, numpy as np
      d = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
      img = aruco.generateImageMarker(d, 0, 400)
      cv2.imwrite('/tmp/apriltag_id0.png', img)
      print('Saved to /tmp/apriltag_id0.png')
      "
    Print it and tape it to the drop-off zone wall at camera height (~30cm).

How to test:
  1. Run this launch file:
       ros2 launch botzilla_control test_apriltag.launch.py

  2. In a second terminal, echo the tag pose:
       ros2 topic echo /drop_off_pose

  3. Point the Kinect directly at the tag from ~1.5m away.
     Expect output like:
         x: ~0.0    (tag is centred in frame = robot is aimed correctly)
         y: <pixel height value>
         z: 0.0

  4. Move the robot slightly LEFT of the tag. Confirm x becomes POSITIVE
     (meaning the tag is to the RIGHT = robot must turn right to align).

  5. Echo /drop_off_visible to confirm it publishes True when tag is visible:
       ros2 topic echo /drop_off_visible

  6. Optional: view the annotated image with tag bounding boxes:
       QT_QPA_PLATFORM=xcb ros2 run rqt_image_view rqt_image_view
       (select topic: /perception/apriltag_image)

What to watch for:
  - "AprilTag Node started. Tracking Tag ID=0" in the launch output.
  - /drop_off_visible changes from False to True when you show it the tag.
  - x on /drop_off_pose approaches 0 as you aim the robot at the centre of the tag.
"""
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

    return LaunchDescription([kinect_bridge, apriltag_node])
