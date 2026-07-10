"""
TEST 1: Chassis / Kobuki Base Test
===================================
Tests that the Kobuki serial connection and motor control work correctly.

How to test:
  1. Run this launch file:
       ros2 launch botzilla_control test_chassis.launch.py

  2. In a second terminal, source the workspace and send movement commands:

     # Drive forward at 0.1 m/s for 2 seconds then stop:
     ros2 topic pub -t 20 /cmd_vel geometry_msgs/msg/Twist \
       "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

     # Rotate in place counter-clockwise:
     ros2 topic pub -t 20 /cmd_vel geometry_msgs/msg/Twist \
       "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

     # EMERGENCY STOP (send once to stop motors):
     ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

What to watch for:
  - "Kobuki hardware connected on /dev/ttyUSB* " in the launch output.
  - The robot physically moves when you send cmd_vel messages.
  - No PermissionError (run: sudo chmod 666 /dev/ttyUSB0 first if needed).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    kobuki_base = Node(
        package='botzilla_control',
        executable='kobuki_base_node',
        name='kobuki_base_node',
        output='screen',
    )

    return LaunchDescription([kobuki_base])
