"""
Odometry-based star navigation test.
No cube collection, no AprilTag — pure navigation only.

Arena  : 300 cm (length, robot +X) × 280 cm (width, robot ±Y)
Start  : robot at arena centre, facing the length wall (+X), heading = 0.

Pattern (star from centre each time):
    centre → Q1 (front-left,  +0.75, +0.70) → search → centre
           → Q2 (front-right, +0.75, -0.70) → search → centre
           → Q3 (back-left,   -0.75, +0.70) → search → centre
           → Q4 (back-right,  -0.75, -0.70) → search → centre → DONE

Requires:
    kobuki_base_node running (publishes /odom, subscribes to /cmd_vel).

Navigation controller (P-control):
    - Turn in place until heading error < ALIGN_THRESH
    - Drive + steer until within ARRIVED_DIST of target
    - Search = spin one full 360° CCW

Launch:
    ros2 launch botzilla_control test_navigation_ssh.launch.py

Monitor:
    ros2 topic echo /odom           # pose feedback
    ros2 topic echo /nav/status     # current state and target
    ros2 topic echo /cmd_vel        # velocity commands
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# ── Arena ───────────────────────────────────────────────────────────────────
ARENA_L = 3.00   # 300 cm
ARENA_W = 2.80   # 280 cm

_QX = ARENA_L / 4   # 0.75 m
_QY = ARENA_W / 4   # 0.70 m

# ── Controller gains & limits ───────────────────────────────────────────────
KP_ANG        = 2.0    # proportional gain for heading error
KP_LIN        = 0.40   # proportional gain for distance error
MAX_LIN       = 0.15   # m/s  cap on forward speed
MIN_LIN       = 0.07   # m/s  minimum forward speed (avoid stall)
MAX_ANG       = 0.60   # rad/s cap on rotation speed
ALIGN_THRESH  = 0.20   # rad  (~11°): turn-in-place if heading error > this
ARRIVED_DIST  = 0.12   # m    consider waypoint reached within this radius

# ── Search spin ─────────────────────────────────────────────────────────────
SEARCH_ANG    = 0.45   # rad/s spin speed during search
SEARCH_RADS   = 2 * math.pi       # full 360°
SEARCH_DUR    = SEARCH_RADS / SEARCH_ANG  # ≈ 14.0 s

# ── Waypoints ────────────────────────────────────────────────────────────────
# (x_m, y_m, label, do_search)
WAYPOINTS = [
    ( _QX,  _QY, 'Q1_front_left',  True),
    ( 0.00,  0.00, 'CENTRE',        False),
    ( _QX, -_QY, 'Q2_front_right', True),
    ( 0.00,  0.00, 'CENTRE',        False),
    (-_QX,  _QY, 'Q3_back_left',   True),
    ( 0.00,  0.00, 'CENTRE',        False),
    (-_QX, -_QY, 'Q4_back_right',  True),
    ( 0.00,  0.00, 'CENTRE_FINAL',  False),
]


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _norm_angle(a):
    """Wrap angle to (-π, π]."""
    while a >  math.pi: a -= 2 * math.pi
    while a <= -math.pi: a += 2 * math.pi
    return a


class NavTestNode(Node):
    def __init__(self):
        super().__init__('nav_test_node')

        self._pub_cmd    = self.create_publisher(Twist,  'cmd_vel',     10)
        self._pub_status = self.create_publisher(String, '/nav/status', 10)

        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)

        # Pose (from /odom)
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._odom_ready = False

        # State machine
        self._state        = 'INIT'
        self._wp_idx       = 0
        self._search_start = None

        self.create_timer(0.05, self._control)   # 20 Hz

        dist_to_quad = math.sqrt(_QX**2 + _QY**2)
        self.get_logger().info(
            f'NavTest | arena {ARENA_L*100:.0f}×{ARENA_W*100:.0f} cm | '
            f'quadrant centres at (±{_QX}m, ±{_QY}m) | '
            f'dist_to_quad={dist_to_quad:.3f}m'
        )
        self.get_logger().info(
            f'NavTest | {len(WAYPOINTS)} waypoints | search={SEARCH_DUR:.1f}s each'
        )
        self.get_logger().info(
            'NavTest: waiting for /odom ... place robot at ARENA CENTRE facing LENGTH WALL.'
        )

    # ── Odometry callback ─────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self._theta = 2.0 * math.atan2(qz, qw)
        if not self._odom_ready:
            self._odom_ready = True
            self.get_logger().info(
                f'NavTest: /odom received — pose ({self._x:.3f}, {self._y:.3f}) '
                f'θ={math.degrees(self._theta):.1f}°. Starting in 1s...'
            )

    # ── Control loop ─────────────────────────────────────────────────────────

    def _control(self):
        cmd = Twist()
        now = self.get_clock().now()

        if self._state == 'INIT':
            if self._odom_ready:
                self._state = 'GOTO'
                self._log_waypoint()

        elif self._state == 'GOTO':
            tx, ty, label, _ = WAYPOINTS[self._wp_idx]
            dx = tx - self._x
            dy = ty - self._y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < ARRIVED_DIST:
                self._on_arrive(now)
            else:
                bearing = math.atan2(dy, dx)
                herr    = _norm_angle(bearing - self._theta)

                if abs(herr) > ALIGN_THRESH:
                    # Turn in place first
                    cmd.angular.z = _clamp(KP_ANG * herr, -MAX_ANG, MAX_ANG)
                else:
                    # Drive and steer together
                    cmd.linear.x  = _clamp(KP_LIN * dist, MIN_LIN, MAX_LIN)
                    cmd.angular.z = _clamp(KP_ANG * herr, -MAX_ANG, MAX_ANG)

                self.get_logger().info(
                    f'GOTO {WAYPOINTS[self._wp_idx][2]:<16s} | '
                    f'dist={dist:.3f}m herr={math.degrees(herr):+.1f}° | '
                    f'lin={cmd.linear.x:.2f} ang={cmd.angular.z:+.2f}',
                    throttle_duration_sec=0.5
                )

        elif self._state == 'SEARCH':
            if self._search_start is None:
                self._search_start = now
                self.get_logger().info(
                    f'SEARCH at {WAYPOINTS[self._wp_idx][2]} | '
                    f'pos=({self._x:.3f},{self._y:.3f}) | {SEARCH_DUR:.1f}s spin...'
                )
            elapsed = (now - self._search_start).nanoseconds / 1e9
            if elapsed < SEARCH_DUR:
                cmd.angular.z = SEARCH_ANG
            else:
                self.get_logger().info('Search complete.')
                self._search_start = None
                self._advance_wp()

        elif self._state == 'DONE':
            self.get_logger().info(
                f'NavTest COMPLETE | final pose ({self._x:.3f},{self._y:.3f}) '
                f'θ={math.degrees(self._theta):.1f}°',
                throttle_duration_sec=5.0
            )

        self._pub_cmd.publish(cmd)
        status = String()
        status.data = (
            f'state={self._state} wp={self._wp_idx}/{len(WAYPOINTS)} '
            f'x={self._x:.3f} y={self._y:.3f} θ={math.degrees(self._theta):.1f}'
        )
        self._pub_status.publish(status)

    # ── State helpers ─────────────────────────────────────────────────────────

    def _on_arrive(self, now):
        tx, ty, label, do_search = WAYPOINTS[self._wp_idx]
        self.get_logger().info(
            f'ARRIVED at {label} | pos=({self._x:.3f},{self._y:.3f}) | '
            f'target=({tx},{ty})'
        )
        if do_search:
            self._state = 'SEARCH'
        else:
            self._advance_wp()

    def _advance_wp(self):
        self._wp_idx += 1
        if self._wp_idx >= len(WAYPOINTS):
            self._state = 'DONE'
            self.get_logger().info('All waypoints visited. DONE.')
        else:
            self._state = 'GOTO'
            self._log_waypoint()

    def _log_waypoint(self):
        tx, ty, label, do_search = WAYPOINTS[self._wp_idx]
        dx = tx - self._x
        dy = ty - self._y
        dist = math.sqrt(dx * dx + dy * dy)
        self.get_logger().info(
            f'Next waypoint [{self._wp_idx+1}/{len(WAYPOINTS)}]: '
            f'{label} → ({tx},{ty}) | dist={dist:.3f}m'
            + (' | will search' if do_search else '')
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub_cmd.publish(Twist())   # safety stop
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
