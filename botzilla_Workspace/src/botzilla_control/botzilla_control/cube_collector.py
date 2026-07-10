import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import time

# --- TUNABLE PARAMETERS ---
ALIGNMENT_THRESHOLD = 0.03
CAPTURE_BLIND_SPOT_M = 0.55
CAPTURE_TIME_S = 2.5
CAPTURE_SPEED = 0.12
KP_ANGULAR = 1.2
APPROACH_SPEED = 0.15
CUBE_LOST_TIMEOUT_S = 3.0   # Pi 5 YOLO runs ~0.75 fps; 1.5s was shorter than inter-frame gap

class CubeCollector(Node):
    def __init__(self):
        super().__init__('cube_collector')

        self.state = "IDLE"  # Start by doing nothing
        self.target_cube = None
        self._phase_timer = None
        self._cube_last_seen = self.get_clock().now()
        self._cube_lost_time = None  # Track when the box actually disappears
        self._blind_spot_frames = 0  # Counter for robust blind-spot detection
        self._vision_ready = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Point, 'detected_cube', self.cube_callback, 10)
        
        # Monitor the debug image to know when the camera is actually sending data
        from sensor_msgs.msg import Image
        self.create_subscription(Image, '/perception/yolo_image', self.vision_heartbeat_cb, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Cube Collector Node Started. Waiting for Kinect initialization...")

    def vision_heartbeat_cb(self, msg):
        if not self._vision_ready:
            self.get_logger().info("Vision Pipeline detected! Starting search rotation.")
            self._vision_ready = True
            if self.state == "IDLE":
                self.state = "SEARCHING"

    def cube_callback(self, msg):
        self.target_cube = msg
        self._cube_last_seen = self.get_clock().now()
        if self.state == "SEARCHING":
            self.get_logger().info(f"Cube detected! Transitioning to TARGETING.")
            self.state = "TARGETING"
        elif self.state == "APPROACHING":
            # Count blind-spot confirmations on ACTUAL new YOLO frames, not 10 Hz
            # control-loop ticks. On Pi 5 YOLO runs ~0.75 fps so a single z=0.0
            # frame must not immediately trigger capture.
            if msg.z == 0.0:
                self._blind_spot_frames += 1
                if self._blind_spot_frames >= 2:
                    self._transition("CAPTURING", "Confirmed blind spot. Final push.")
            else:
                self._blind_spot_frames = 0

    def control_loop(self):
        msg = Twist()
        now = self.get_clock().now()

        if self.state == "IDLE":
            # Stay completely still
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        elif self.state == "SEARCHING":
            # Reduced rotation speed for better stability
            msg.angular.z = 0.25 

        elif self.state == "TARGETING":
            if self._cube_timed_out(now):
                self._transition("SEARCHING", "Cube lost.")
            elif self.target_cube is not None:
                error_x = self.target_cube.x
                if abs(error_x) > ALIGNMENT_THRESHOLD:
                    msg.angular.z = -KP_ANGULAR * error_x
                else:
                    self._transition("APPROACHING", "Aligned. Moving in.")

        elif self.state == "APPROACHING":
            if self._cube_timed_out(now):
                self._transition("SEARCHING", "Cube lost.")
            elif self.target_cube is not None:
                msg.angular.z = -KP_ANGULAR * 0.5 * self.target_cube.x
                msg.linear.x = APPROACH_SPEED
                # Blind-spot transition is handled in cube_callback (real YOLO frames only)

        elif self.state == "CAPTURING":
            # Pi 5 YOLO runs ~0.75 fps so inter-frame gap is ~1.3s.
            # CUBE_TIMEOUT_S must be longer than that or we'd immediately think
            # the cube is lost between every pair of YOLO frames.
            CUBE_TIMEOUT_S = 2.0
            EXTRA_PUSH_S = 0.3

            if (now - self._cube_last_seen).nanoseconds / 1e9 < CUBE_TIMEOUT_S:
                self._cube_lost_time = None
                msg.linear.x = CAPTURE_SPEED
                if self.target_cube is not None:
                    msg.angular.z = -KP_ANGULAR * 0.3 * self.target_cube.x
            else:
                if self._cube_lost_time is None:
                    self.get_logger().info("Cube lost from view. Performing final push...")
                    self._cube_lost_time = now

                elapsed_since_lost = (now - self._cube_lost_time).nanoseconds / 1e9
                if elapsed_since_lost < EXTRA_PUSH_S:
                    msg.linear.x = CAPTURE_SPEED
                else:
                    self._transition("DONE", "Capture complete! Final push finished.")

        elif self.state == "DONE":
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

    def _transition(self, new_state, reason=""):
        self.get_logger().info(f"[{self.state}] -> [{new_state}] | {reason}")
        self.state = new_state
        self._phase_timer = self.get_clock().now()

    def _elapsed_since(self, now):
        if self._phase_timer is None: return 0.0
        return (now - self._phase_timer).nanoseconds * 1e-9

    def _cube_timed_out(self, now):
        return (now - self._cube_last_seen).nanoseconds * 1e-9 > CUBE_LOST_TIMEOUT_S

def main(args=None):
    rclpy.init(args=args)
    node = CubeCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Check if node and publisher still exist before publishing stop command
        if 'node' in locals() and rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
            except:
                pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
