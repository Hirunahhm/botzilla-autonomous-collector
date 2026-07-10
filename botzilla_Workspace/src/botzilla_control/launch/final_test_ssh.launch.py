"""
Final Integration Test — SSH / headless
========================================
Runs the full mission pipeline using encoder-based odometry navigation:

  1. kobuki_base_node  — serial driver + /odom from wheel encoders
  2. kinect_bridge     — Kinect RGB+Depth (LD_PRELOAD=noreset.so for Pi 5)
  3. yolo_node         — YOLOv8 + depth → /detected_cube  (QT offscreen)
  4. apriltag_node     — AprilTag → /drop_off_visible + /drop_off_pose  (QT offscreen)
  5. final_test_node   — Full mission FSM (encoder-nav + cube collect + delivery)

Mission
-------
  INIT → search quadrants Q1→Q2→Q3→Q4 (star pattern, via arena centre)
       → collect cube when YOLO detects it
       → navigate to quadrant where AprilTag was seen (via centre)
       → deliver cube next to AprilTag
       → reverse to release → return to centre → DONE

Both orderings handled:
  - Tag seen first: remembered, cube collected, then routed to tag quadrant
  - Cube found first: collected, remaining quads searched (tag-only) until tag seen

Prerequisites
-------------
  Place robot at ARENA CENTRE (0,0), facing the 300 cm LENGTH WALL (+X direction).
  Kinect and Kobuki connected via USB.

How to launch:
  ros2 launch botzilla_control final_test_ssh.launch.py

Monitor (second terminal):
  ros2 topic echo /mission/status      # FSM state + pose + flags
  ros2 topic echo /odom                # encoder pose (x, y, theta)
  ros2 topic echo /detected_cube       # YOLO cube (x=norm, z=dist_m)
  ros2 topic echo /drop_off_visible    # True when AprilTag in view
  ros2 topic echo /cmd_vel             # velocity commands
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

    final_test = Node(
        package='botzilla_control',
        executable='final_test_node',
        name='final_test_node',
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg='[final_test] ═══════════════════════════════════════════════'),
        LogInfo(msg='[final_test] BotZilla Final Integration Test (SSH/headless)'),
        LogInfo(msg='[final_test] Mission: collect cube → deliver to AprilTag → home'),
        LogInfo(msg='[final_test] Arena: 300 cm × 280 cm | Start: CENTRE facing LENGTH WALL'),
        LogInfo(msg='[final_test] Monitor: ros2 topic echo /mission/status'),
        LogInfo(msg='[final_test] Monitor: ros2 topic echo /odom'),
        LogInfo(msg='[final_test] ═══════════════════════════════════════════════'),
        kobuki_base,
        kinect_bridge,
        yolo_node,
        apriltag_node,
        final_test,
    ])
