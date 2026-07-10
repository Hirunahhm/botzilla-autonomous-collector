"""
Full Autonomy — SSH / headless variant
=======================================
Runs the complete BotZilla pipeline over SSH (no display attached):

  1. kobuki_base_node   — serial driver: /cmd_vel → Kobuki wheel commands
  2. kinect_bridge      — Kinect RGB+Depth → /camera/rgb/image_raw + /camera/depth/image_raw
  3. yolo_node          — YOLOv8 + depth → /detected_cube  (offscreen, no window)
  4. apriltag_node      — AprilTag 203/113 → /drop_off_visible + /drop_off_pose  (offscreen)
  5. brain_node         — Full FSM → /cmd_vel
                          SEARCHING → TARGETING → APPROACHING →
                          CAPTURING → DELIVERING → DETACHING → SEARCHING

Both vision nodes have QT_QPA_PLATFORM=offscreen so cv2.imshow never tries
to open an X11 window (which crashes over SSH).

How to launch:
  ros2 launch botzilla_control full_autonomy_ssh.launch.py

Monitor in a second terminal:
  ros2 topic echo /detected_cube          # cube position (x=norm, z=dist)
  ros2 topic echo /drop_off_visible       # True when AprilTag in view
  ros2 topic echo /drop_off_pose          # tag x-offset used for steering
  ros2 topic echo /cmd_vel               # velocity commands to Kobuki

Debug streams (annotated images — view with rqt_image_view on a display machine):
  /perception/yolo_image
  /perception/apriltag_image
"""
import os
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

_NORESET = os.path.join(
    os.path.expanduser('~'),
    'Desktop/Bozilla-ws/final-project-botzilla/noreset.so'
)


def generate_launch_description():
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

    apriltag_node = Node(
        package='botzilla_perception',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        additional_env={'QT_QPA_PLATFORM': 'offscreen'},
    )

    brain_node = Node(
        package='botzilla_control',
        executable='brain_node',
        name='botzilla_brain',
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg='[full_autonomy_ssh] ========================================'),
        LogInfo(msg='[full_autonomy_ssh] BotZilla Full Autonomy (SSH/headless)'),
        LogInfo(msg='[full_autonomy_ssh] FSM: SEARCHING → TARGETING → APPROACHING → CAPTURING → DELIVERING → DETACHING'),
        LogInfo(msg='[full_autonomy_ssh] Nodes: kobuki_base | kinect_bridge | yolo_node | apriltag_node | brain_node'),
        LogInfo(msg='[full_autonomy_ssh] Monitor: ros2 topic echo /detected_cube'),
        LogInfo(msg='[full_autonomy_ssh] Monitor: ros2 topic echo /drop_off_visible'),
        LogInfo(msg='[full_autonomy_ssh] ========================================'),
        kobuki_base,
        kinect_bridge,
        yolo_node,
        apriltag_node,
        brain_node,
    ])
