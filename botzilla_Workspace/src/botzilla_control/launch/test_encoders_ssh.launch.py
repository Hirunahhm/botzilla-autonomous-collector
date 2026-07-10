"""
Encoder Diagnostic Test — SSH / headless
=========================================
Runs encoder_test_node ONLY (no kobuki_base_node — encoder_test_node
owns the serial port itself).

Test:  drive 0.8 m forward → pause → drive 0.8 m backward.
       Logs raw ticks, cumulative ticks, and estimated pose throughout.
       Compare estimated distance to actual to verify TICKS_PER_M.

IMPORTANT: do NOT run kobuki_base_node at the same time — both
           would try to open /dev/ttyUSB0 simultaneously.

How to run:
  ros2 launch botzilla_control test_encoders_ssh.launch.py

Monitor in a second terminal:
  ros2 topic echo /encoder/pose        # estimated x, y, theta
  ros2 topic echo /encoder/ticks       # [L_raw, R_raw, dL, dR, cumL, cumR]
  ros2 topic hz /encoder/pose          # should be ~10 Hz

What to check:
  - SETTLE phase: dL=0, dR=0 every tick (no spurious movement)
  - FORWARD phase: dL and dR both positive, similar magnitude
  - cumL and cumR should both reach ~9379 after 0.8 m
  - BACKWARD phase: dL and dR both negative
  - Final cumL and cumR should be close to 0
  - Final pose x≈0.0, y≈0.0, theta≈0.0

If measured ticks differ >5% from expected:
  Update TICKS_PER_M in encoder_test_node.py and kobuki_base_node.py.
  Formula:  TICKS_PER_M = measured_cum_ticks / actual_distance_m
"""
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    encoder_test = Node(
        package='botzilla_control',
        executable='encoder_test_node',
        name='encoder_test_node',
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg='[test_encoders] ═══════════════════════════════════════'),
        LogInfo(msg='[test_encoders] Kobuki Encoder Diagnostic Test'),
        LogInfo(msg='[test_encoders] DO NOT run kobuki_base_node simultaneously!'),
        LogInfo(msg='[test_encoders] Place robot on FLAT OPEN FLOOR with ~1m clear ahead.'),
        LogInfo(msg='[test_encoders] Monitor: ros2 topic echo /encoder/pose'),
        LogInfo(msg='[test_encoders] ═══════════════════════════════════════'),
        encoder_test,
    ])
