"""
Navigation Test — SSH / headless
==================================
Tests star-pattern navigation using Kobuki wheel encoder odometry.
No camera, no YOLO, no AprilTag — pure drive control only.

Nodes launched:
  1. kobuki_base_node  — serial driver + publishes /odom from encoders
  2. nav_test_node     — P-controller that navigates the star pattern

Arena setup:
  300 cm (length) × 280 cm (width)
  Place robot at the ARENA CENTRE, facing the 300 cm LENGTH WALL.
  The robot treats this starting pose as (x=0, y=0, theta=0).

Star pattern:
  centre → Q1(+0.75,+0.70) → search → centre
         → Q2(+0.75,-0.70) → search → centre
         → Q3(-0.75,+0.70) → search → centre
         → Q4(-0.75,-0.70) → search → centre → DONE

  (+X = forward/length, +Y = left/width, ROS convention)

How to run:
  ros2 launch botzilla_control test_navigation_ssh.launch.py

Monitor in a second terminal:
  ros2 topic echo /odom              # live pose (x, y, heading)
  ros2 topic echo /nav/status        # current state, target, heading error
  ros2 topic echo /cmd_vel           # velocity commands

Calibration — if navigation is inaccurate:
  1. First run test_encoders_ssh.launch.py to verify TICKS_PER_M.
  2. Tune TICKS_PER_M in kobuki_base_node.py if encoder distance is off.
  3. Tune KP_ANG / KP_LIN in nav_test_node.py if overshooting.
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

    nav_test = Node(
        package='botzilla_control',
        executable='nav_test_node',
        name='nav_test_node',
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg='[test_navigation] ══════════════════════════════════════════'),
        LogInfo(msg='[test_navigation] Kobuki Star Navigation Test (encoder odometry)'),
        LogInfo(msg='[test_navigation] Arena: 300 cm × 280 cm'),
        LogInfo(msg='[test_navigation] Place robot at ARENA CENTRE facing the LENGTH WALL.'),
        LogInfo(msg='[test_navigation] Monitor pose: ros2 topic echo /odom'),
        LogInfo(msg='[test_navigation] Monitor state: ros2 topic echo /nav/status'),
        LogInfo(msg='[test_navigation] ══════════════════════════════════════════'),
        kobuki_base,
        nav_test,
    ])
