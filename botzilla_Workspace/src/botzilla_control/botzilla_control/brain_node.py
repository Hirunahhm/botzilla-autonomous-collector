import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool

# --- TUNABLE PARAMETERS ---
# Distance at which the robot considers itself aligned enough to start approaching
ALIGNMENT_THRESHOLD = 0.08    # normalized units (~5% of frame width)

# Distance at which the depth sensor enters blind spot: cube is "in arms" range (meters)
CAPTURE_BLIND_SPOT_M = 0.55

# How far to drive (blind) after reaching the blind spot to firmly pocket the cube (seconds at CAPTURE_SPEED)
CAPTURE_TIME_S = 2.5
CAPTURE_SPEED = 0.12  # m/s

# How long to drive toward drop-off zone after detecting AprilTag (seconds)
# This is open-loop for simplicity without an AprilTag node yet
DELIVER_TIME_S = 4.0
DELIVER_SPEED = 0.12  # m/s

# How far to reverse after delivering (seconds)
DETACH_TIME_S = 2.5
DETACH_SPEED = -0.12  # m/s (negative = reverse)

# Proportional gain for alignment rotation
KP_ANGULAR = 0.8

# Max linear speed during approach
APPROACH_SPEED = 0.15  # m/s

# How long to not see a cube before reverting back to SEARCHING
CUBE_LOST_TIMEOUT_S = 1.5


class BotzillaBrain(Node):
    def __init__(self):
        super().__init__('botzilla_brain')

        self.state = "SEARCHING"
        self.target_cube = None    # geometry_msgs/Point: x=norm_offset, z=distance_m
        self.drop_off_visible = False

        # Timing helpers for open-loop phases
        self._phase_timer = None   # rclpy.time
        self._cube_last_seen = self.get_clock().now()

        self.drop_off_pose = None   # Point: x=normalized offset from apriltag_node

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(Point, 'detected_cube', self.cube_callback, 10)
        self.create_subscription(Bool, 'drop_off_visible', self.drop_off_callback, 10)
        self.create_subscription(Point, 'drop_off_pose', self.drop_off_pose_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info("BotZilla Brain initialized in SEARCHING state.")

    # ---- Subscribers ----

    def cube_callback(self, msg):
        """Receives the closest detected cube coordinates from yolo_node."""
        self.target_cube = msg
        self._cube_last_seen = self.get_clock().now()
        if self.state == "SEARCHING":
            self.get_logger().info(f"Cube detected at x={msg.x:.2f}, z={msg.z:.2f}m. Transitioning to TARGETING.")
            self.state = "TARGETING"

    def drop_off_callback(self, msg):
        """Receives whether the AprilTag drop-off marker is visible."""
        self.drop_off_visible = msg.data

    def drop_off_pose_callback(self, msg):
        """Receives normalized horizontal offset of the drop-off AprilTag."""
        self.drop_off_pose = msg

    # ---- State Machine ----

    def control_loop(self):
        msg = Twist()
        now = self.get_clock().now()

        # ── SEARCHING: rotate slowly until a cube is found ──
        if self.state == "SEARCHING":
            msg.angular.z = 0.4
            self.target_cube = None

        # ── TARGETING: proportional alignment on cube center ──
        elif self.state == "TARGETING":
            if self._cube_timed_out(now):
                self._transition("SEARCHING", "Cube lost. Returning to SEARCHING.")
            elif self.target_cube is not None:
                error_x = self.target_cube.x   # normalized: -1 (left) to +1 (right)
                if abs(error_x) > ALIGNMENT_THRESHOLD:
                    # Proportional turn: positive error (cube to right) → turn right (negative angular)
                    msg.angular.z = -KP_ANGULAR * error_x
                    self.get_logger().debug(f"Aligning: error_x={error_x:.3f}, angular.z={msg.angular.z:.3f}")
                else:
                    self._transition("APPROACHING", f"Aligned. Approaching cube at z={self.target_cube.z:.2f}m")

        # ── APPROACHING: drive toward cube while keeping aligned ──
        elif self.state == "APPROACHING":
            if self._cube_timed_out(now):
                self._transition("SEARCHING", "Cube lost during approach. Returning to SEARCHING.")
            elif self.target_cube is not None:
                # Maintain centering while driving forward
                error_x = self.target_cube.x
                msg.angular.z = -KP_ANGULAR * 0.5 * error_x

                if self.target_cube.z == 0.0:
                    # z == 0.0 means cube is in the Kinect blind spot: drive blind to pocket it
                    self.get_logger().info("Cube in blind-spot range. Switching to CAPTURING (blind push).")
                    self._phase_timer = now
                    self._transition("CAPTURING", "Starting timed blind capture drive.")
                else:
                    msg.linear.x = APPROACH_SPEED

        # ── CAPTURING: drive blindly for CAPTURE_TIME_S to pocket the cube ──
        elif self.state == "CAPTURING":
            elapsed = self._elapsed_since(now)
            if elapsed < CAPTURE_TIME_S:
                msg.linear.x = CAPTURE_SPEED
                self.get_logger().debug(f"CAPTURING: {elapsed:.1f}/{CAPTURE_TIME_S}s")
            else:
                self._transition("DELIVERING", "Cube captured! Looking for drop-off zone.")

        # ── DELIVERING: steer toward AprilTag drop-off marker ──
        elif self.state == "DELIVERING":
            elapsed = self._elapsed_since(now)
            if elapsed < DELIVER_TIME_S:
                msg.linear.x = DELIVER_SPEED
                if self.drop_off_visible and self.drop_off_pose is not None:
                    # Proportional correction: steer toward the tag center
                    error_x = self.drop_off_pose.x  # normalized: -1 left, +1 right
                    msg.angular.z = -KP_ANGULAR * 0.5 * error_x
                    self.get_logger().debug(
                        f"DELIVERING (guided): elapsed={elapsed:.1f}s, tag_offset={error_x:.2f}"
                    )
                else:
                    # Tag not visible yet: drive straight (blind)
                    self.get_logger().debug(
                        f"DELIVERING (blind): elapsed={elapsed:.1f}/{DELIVER_TIME_S}s"
                    )
            else:
                self._transition("DETACHING", "Arrived at drop-off zone. Reversing.")

        # ── DETACHING: reverse away to release the cube ──
        elif self.state == "DETACHING":
            elapsed = self._elapsed_since(now)
            if elapsed < DETACH_TIME_S:
                msg.linear.x = DETACH_SPEED
                self.get_logger().debug(f"DETACHING: {elapsed:.1f}/{DETACH_TIME_S}s")
            else:
                self._transition("SEARCHING", "Cube delivered. Returning to SEARCHING for next cube.")

        self.cmd_pub.publish(msg)

    # ---- Helpers ----

    def _transition(self, new_state, reason=""):
        self.get_logger().info(f"[{self.state}] -> [{new_state}] | {reason}")
        self.state = new_state
        self._phase_timer = self.get_clock().now()   # reset phase timer on every transition

    def _elapsed_since(self, now):
        if self._phase_timer is None:
            return 0.0
        return (now - self._phase_timer).nanoseconds * 1e-9

    def _cube_timed_out(self, now):
        elapsed = (now - self._cube_last_seen).nanoseconds * 1e-9
        return elapsed > CUBE_LOST_TIMEOUT_S


def main(args=None):
    rclpy.init(args=args)
    node = BotzillaBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors on shutdown
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()