"""
TEST 2: Vision Pipeline Test (Kinect + YOLO)
=============================================
Tests that the Kinect streams RGB and Depth, and that YOLO correctly
detects cubes and reports accurate coordinates over the /detected_cube topic.

How to test:
  1. Connect the Kinect USB. Run: sudo chmod 666 /dev/bus/usb/*/*

  2. Run this launch file:
       ros2 launch botzilla_control test_vision.launch.py

  3. In a second terminal, check the cube detections:
       ros2 topic echo /detected_cube

  4. Hold a cube at exactly 1 metre directly in front of the robot.
     Expect output like:
         x: 0.02    (close to 0 = centred)
         y: 0.0
         z: 1.0     (distance in metres)

  5. Move the cube left and right. Confirm 'x' changes sign correctly:
         x < 0  → cube is to the LEFT  of centre
         x > 0  → cube is to the RIGHT of centre

  6. Optional: to view the annotated image with bounding boxes:
       QT_QPA_PLATFORM=xcb ros2 run rqt_image_view rqt_image_view
       (select topic: /perception/yolo_image)

What to watch for:
  - kinect_bridge: "Decoupled 30FPS Kinect Bridge Started!" (not "Can't open device").
  - yolo_node: "YOLO Perception Node Initialized. Waiting for video stream..."
  - /detected_cube messages publishing at a consistent rate.
  - z value closely matches your tape-measured distance to the cube.
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

    yolo_node = Node(
        package='botzilla_perception',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
    )

    return LaunchDescription([kinect_bridge, yolo_node])
