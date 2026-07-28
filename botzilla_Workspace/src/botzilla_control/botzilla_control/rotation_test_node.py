"""
rotation_test_node.py

Standalone, self-bounding rotation-in-place test, decoupled from Nav2's
controller/planner/behavior-tree stack. Purpose: isolate whether RTAB-Map's
registration holds up during a pure turn, without any of Nav2's own timing
and control-loop behavior confounding the result (see the ghost-map
investigation this node was built for).

Publishes /cmd_vel at a fixed angular rate for a fixed duration, then
explicitly publishes zero velocity several times and shuts itself down.
Never runs unattended past `duration` seconds — same bounded-motor-command
rule as every other hardware test in this project.

Usage:
  ros2 run botzilla_control rotation_test_node
  ros2 run botzilla_control rotation_test_node --ros-args -p angular_speed:=0.2 -p duration:=12.0
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

ANGULAR_SPEED_RAD_S = 0.3   # modest, controlled turn rate
ROTATION_DURATION_S = 8.0   # bounded — 0.3 rad/s * 8s =~ 137 degrees
PUBLISH_RATE_HZ = 20.0
STOP_MESSAGE_COUNT = 10     # publish zero velocity this many times before shutting down,
                            # to make sure the stop is actually received


class RotationTestNode(Node):
    def __init__(self):
        super().__init__('rotation_test_node')
        self.declare_parameter('angular_speed', ANGULAR_SPEED_RAD_S)
        self.declare_parameter('duration', ROTATION_DURATION_S)
        self._angular_speed = self.get_parameter('angular_speed').value
        self._duration = self.get_parameter('duration').value

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._start_time = self.get_clock().now()
        self._stop_requested_logged = False
        self._stop_messages_sent = 0

        total_deg = math.degrees(self._angular_speed * self._duration)
        self.get_logger().info(
            f'Rotating at {self._angular_speed:.2f} rad/s for {self._duration:.1f}s '
            f'(~{total_deg:.0f} deg total), then stopping.'
        )

        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

    def _tick(self):
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9

        if elapsed < self._duration:
            msg = Twist()
            msg.angular.z = self._angular_speed
            self._pub.publish(msg)
            self.get_logger().info(
                f'rotating... {elapsed:.1f}/{self._duration:.1f}s',
                throttle_duration_sec=1.0,
            )
            return

        # Bounded duration reached — stop, unconditionally, from here on.
        self._pub.publish(Twist())
        self._stop_messages_sent += 1
        if not self._stop_requested_logged:
            self.get_logger().info('Rotation complete — stopping.')
            self._stop_requested_logged = True
        if self._stop_messages_sent >= STOP_MESSAGE_COUNT:
            self.get_logger().info('Stop confirmed sent, shutting down.')
            self._timer.cancel()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RotationTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — sending stop.')
        node._pub.publish(Twist())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
