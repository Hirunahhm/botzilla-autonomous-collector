import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

from .KobukiDriver import Kobuki

# ── Odometry constants ──────────────────────────────────────────────────────
# Kobuki QBot: 52 encoder ticks × 34.02 gear ratio / (π × 0.0688 m wheel diameter)
# ≈ 11724 ticks per metre.  Tune TICKS_PER_M if measured distance doesn't match.
TICKS_PER_M  = 11724.41
WHEEL_BASE_M = 0.230    # 23 cm — must match cmd_vel_callback below


class KobukiBaseNode(Node):
    def __init__(self):
        super().__init__('kobuki_base_node')

        self.get_logger().info('Connecting to Kobuki hardware...')
        self.robot = Kobuki()
        self.robot.play_on_sound()

        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Odometry publisher
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._ox = 0.0
        self._oy = 0.0
        self._ot = 0.0          # heading (rad, CCW+)
        self._prev_L = None     # previous raw 16-bit left  tick
        self._prev_R = None     # previous raw 16-bit right tick
        self.create_timer(0.02, self._odom_update)   # 50 Hz

        self.get_logger().info(
            'Kobuki Base Node started. /cmd_vel → motors | encoders → /odom')

    # ── Odometry ────────────────────────────────────────────────────────────

    @staticmethod
    def _tick_diff(new, old):
        """Signed delta between two 16-bit unsigned encoder readings (handles rollover)."""
        d = (new - old) & 0xFFFF
        return d if d < 32768 else d - 65536

    def _odom_update(self):
        try:
            enc = self.robot.encoder_data()
        except Exception:
            return   # __basic_sensor not populated yet

        L = enc['Left_encoder']
        R = enc['Right_encoder']

        if self._prev_L is None:
            self._prev_L, self._prev_R = L, R
            return

        dl = self._tick_diff(L, self._prev_L) / TICKS_PER_M
        dr = self._tick_diff(R, self._prev_R) / TICKS_PER_M
        self._prev_L, self._prev_R = L, R

        d      = (dl + dr) / 2.0
        dtheta = (dr - dl) / WHEEL_BASE_M
        self._ox += d * math.cos(self._ot + dtheta / 2.0)
        self._oy += d * math.sin(self._ot + dtheta / 2.0)
        self._ot += dtheta

        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'
        msg.pose.pose.position.x    = self._ox
        msg.pose.pose.position.y    = self._oy
        msg.pose.pose.orientation.z = math.sin(self._ot / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._ot / 2.0)
        self._odom_pub.publish(msg)

    # ── Velocity command ─────────────────────────────────────────────────────

    def cmd_vel_callback(self, msg):
        """
        Translates ROS 2 Twist messages into Kobuki hardware commands.
        """
        linear_x = msg.linear.x   # Forward/Backward speed (m/s)
        angular_z = msg.angular.z # Turning speed (rad/s)
        
        wheel_base = 0.230 # 23 cm wheel separation for Kobuki
        
        # Calculate left and right wheel speeds in mm/s
        left_wheel_speed = (linear_x - (angular_z * wheel_base / 2.0)) * 1000.0
        right_wheel_speed = (linear_x + (angular_z * wheel_base / 2.0)) * 1000.0
        
        # Determine the 'rotate' flag based on your driver's logic
        # 1 means pure rotation, 0 means forward/arc 
        rotate_flag = 1 if linear_x == 0.0 and angular_z != 0.0 else 0
        
        # Send to hardware
        self.robot.move(int(left_wheel_speed), int(right_wheel_speed), rotate_flag)
        
        self.get_logger().debug(f'Moving -> L: {left_wheel_speed:.1f}, R: {right_wheel_speed:.1f}, Rot: {rotate_flag}')

def main(args=None):
    rclpy.init(args=args)
    node = KobukiBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Kobuki node...')
        # Stop motors safely (0 left, 0 right, 0 rotate flag)
        node.robot.move(0, 0, 0) 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()