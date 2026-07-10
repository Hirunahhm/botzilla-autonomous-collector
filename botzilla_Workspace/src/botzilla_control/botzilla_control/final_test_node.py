"""
Final integration test — collect one cube and deliver it to the AprilTag drop-off zone.

Arena  : 300 cm (length, +X) × 280 cm (width, ±Y)
Start  : robot at arena centre (0, 0), facing the length wall (+X), heading = 0.

Mission
-------
1. Search quadrants Q1→Q2→Q3→Q4 (star pattern, via centre between each).
2. While searching, remember the quadrant where the AprilTag is first seen.
3. When cube found in any quadrant — collect it.
4. Navigate through the centre to the AprilTag quadrant; steer up to the tag; drop cube.
5. Reverse to release (arms open), return to centre.

Both orderings handled
----------------------
A) Tag seen first (Qi), cube found later (Qj):
   save tag_pos=Qi, collect cube, → centre → Qi → deliver.

B) Cube found first (Qi), tag found later (Qj):
   collect cube, continue searching remaining quads (tag-only mode),
   tag seen in Qj → save tag_pos=Qj, → centre → Qj → deliver.

C) Both in same quadrant: collect cube, deliver in same quadrant (still via centre).

Topics
------
  Subscribe: /odom, detected_cube, /perception/yolo_image,
             drop_off_visible, drop_off_pose
  Publish  : cmd_vel, /mission/status

Launch
------
  ros2 launch botzilla_control final_test_ssh.launch.py

Monitor
-------
  ros2 topic echo /mission/status
  ros2 topic echo /odom
  ros2 topic echo /cmd_vel
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

# ── Arena ────────────────────────────────────────────────────────────────────
_QX = 3.00 / 4   # 0.75 m (half of half-length)
_QY = 2.80 / 4   # 0.70 m (half of half-width)

# Star-pattern visit order: (x_m, y_m, label)
QUADRANTS = [
    ( _QX,  _QY, 'Q1_front_left'),
    ( _QX, -_QY, 'Q2_front_right'),
    (-_QX,  _QY, 'Q3_back_left'),
    (-_QX, -_QY, 'Q4_back_right'),
]

# ── Navigation (from nav_test_node) ──────────────────────────────────────────
KP_ANG_NAV   = 2.0
KP_LIN       = 0.40
MAX_LIN      = 0.15    # m/s
MIN_LIN      = 0.07    # m/s
MAX_ANG      = 0.60    # rad/s
ALIGN_THRESH = 0.20    # rad
ARRIVED_DIST = 0.12    # m

# ── Search spin ───────────────────────────────────────────────────────────────
SEARCH_SPEED    = 0.25                          # rad/s (slow for ~0.75fps YOLO on Pi 5)
SEARCH_DURATION = 2 * math.pi / SEARCH_SPEED + 2.5  # ≈ 25 s per full 360°

# ── Cube collection — identical to cube_collector.py (verified working) ──────
KP_ANG_CUBE         = 1.2    # pure P-gain for angular correction
MAX_ANG_CUBE        = 0.35   # rad/s — clamp to prevent overshoot at slow YOLO rate
CUBE_ALIGN_THRESH   = 0.03
APPROACH_SPEED      = 0.15   # m/s
CAPTURE_SPEED       = 0.12   # m/s
CUBE_LOST_TIMEOUT_S = 5.0    # s — YOLO gaps can be 2.8s+; 5s survives two missed frames
CUBE_TIMEOUT_S      = 2.0    # s — grace period in CAPTURING before "lost"
EXTRA_PUSH_S        = 0.1    # s — final blind push after cube leaves frame
BLIND_SPOT_FRAMES   = 2      # consecutive z==0.0 readings before CAPTURING

# ── AprilTag delivery (from tag_follower_node) ────────────────────────────────
DELIVERY_SEARCH_SPEED = 0.30   # rad/s while scanning for tag at drop-off quadrant
FOLLOW_SPEED          = 0.08   # m/s forward during delivery approach
KP_ANG_TAG            = 0.6
TAG_ALIGN_THRESH      = 0.10   # normalised units
STOP_HEIGHT           = 230    # pixel height of tag when "arrived" (~0.4-0.5 m)

# ── Detach (arms release cube) ────────────────────────────────────────────────
DETACH_SPEED = -0.10   # m/s (reverse)
DETACH_TIME  =  1.8    # s
TAG_GRACE_S  =  2.0    # s — keep driving forward when tag briefly drops out


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _norm(a: float) -> float:
    """Wrap angle to (-π, π]."""
    while a >  math.pi: a -= 2 * math.pi
    while a <= -math.pi: a += 2 * math.pi
    return a


class FinalTestNode(Node):

    def __init__(self):
        super().__init__('final_test_node')

        # Publishers
        self._pub_cmd    = self.create_publisher(Twist,  'cmd_vel',         10)
        self._pub_status = self.create_publisher(String, '/mission/status', 10)
        self._pub_apriltag_en = self.create_publisher(Bool, '/apriltag/enable', 10)

        # Subscribers
        self.create_subscription(Odometry, 'odom',              self._odom_cb,    10)
        self.create_subscription(Point,    'detected_cube',     self._cube_cb,    10)
        self.create_subscription(Image,    '/perception/yolo_image', self._vision_cb, 1)
        self.create_subscription(Bool,     'drop_off_visible',  self._tag_vis_cb, 10)
        self.create_subscription(Point,    'drop_off_pose',     self._tag_pose_cb,10)

        # ── Pose ─────────────────────────────────────────────────────────
        self._x          = 0.0
        self._y          = 0.0
        self._theta      = 0.0
        self._odom_ready  = False
        self._vision_ready = False

        # ── Mission-level state ───────────────────────────────────────────
        self._state       = 'INIT'
        self._cube_captured = False
        self._mission_phase = 'TAG_SEARCH'  # 'TAG_SEARCH' → 'CUBE_HUNT' → 'DELIVERY'
        self._tag_pos       = None   # (x, y) quadrant centre where AprilTag was first seen

        # Quadrant tracking
        self._q_idx     = 0          # index of quadrant currently being navigated to / searched
        self._q_visited : set = set()  # indices of quadrants fully searched (360° done)

        # Generic navigation target and routing
        self._nav_target          = None    # (x, y) for current NAV_* state
        self._next_after_center   = None    # state to enter after NAV_TO_CENTER arrives

        # ── SEARCH_QUAD sub-state ─────────────────────────────────────────
        self._search_elapsed = 0.0    # seconds accumulated in current SEARCH_QUAD

        # ── Cube collection sub-state ─────────────────────────────────────
        self._target_cube        = None
        self._cube_last_seen     = self.get_clock().now()
        self._cube_lost_time     = None
        self._blind_spot_frames  = 0

        # ── Delivery sub-state ────────────────────────────────────────────
        self._tag_visible          = False
        self._tag_pose             = None   # Point from apriltag_node
        self._tag_last_seen        = None   # timestamp of last tag detection
        self._delivery_search_elapsed = 0.0

        # ── Phase timer (DETACH) ──────────────────────────────────────────
        self._phase_start = None

        self.create_timer(0.10, self._control)   # 10 Hz — matches cube_collector rate

        self.get_logger().info(
            'FinalTest: arena 300×280 cm | quadrants: '
            + '  '.join(f'{q[2]}({q[0]:.2f},{q[1]:.2f})' for q in QUADRANTS)
        )
        self.get_logger().info(
            f'FinalTest: search_dur={SEARCH_DURATION:.1f}s per quad | '
            f'blind_frames={BLIND_SPOT_FRAMES} | stop_height={STOP_HEIGHT}px'
        )
        self.get_logger().info(
            'FinalTest: PHASED mission — 1) TAG_SEARCH  2) CUBE_HUNT  3) DELIVERY'
        )
        self.get_logger().info(
            'FinalTest: waiting for /odom and /perception/yolo_image...'
        )

    # ═══════════════════════════════════════════════════════════════════════════
    # Sensor callbacks
    # ═══════════════════════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self._theta = 2.0 * math.atan2(qz, qw)
        if not self._odom_ready:
            self._odom_ready = True
            self.get_logger().info(
                f'FinalTest: /odom ready — ({self._x:.3f},{self._y:.3f}) '
                f'θ={math.degrees(self._theta):.1f}°'
            )

    def _vision_cb(self, _msg: Image):
        if not self._vision_ready:
            self._vision_ready = True
            self.get_logger().info('FinalTest: YOLO vision pipeline ready.')

    def _cube_cb(self, msg: Point):
        """Only react to cube detections during CUBE_HUNT phase."""
        if self._cube_captured or self._mission_phase != 'CUBE_HUNT':
            return

        # Ignore any cube detection over 1.0m distance
        if msg.z > 1.0:
            return


        self._target_cube    = msg
        self._cube_last_seen = self.get_clock().now()

        if self._state == 'SEARCH_QUAD':
            self.get_logger().info(f'Cube detected! Transitioning to TARGETING.')
            self._transition('TARGETING')

        elif self._state == 'APPROACHING':
            if msg.z == 0.0:
                self._blind_spot_frames += 1
                if self._blind_spot_frames >= 2:
                    self._transition('CAPTURING', 'Confirmed blind spot. Final push.')
            else:
                self._blind_spot_frames = 0

    def _tag_vis_cb(self, msg: Bool):
        self._tag_visible = msg.data
        if self._tag_visible:
            self._tag_last_seen = self.get_clock().now()

        # During TAG_SEARCH: save tag position and switch to CUBE_HUNT
        if (self._tag_visible
                and self._mission_phase == 'TAG_SEARCH'
                and self._tag_pos is None):
            self._tag_pos = (QUADRANTS[self._q_idx][0], QUADRANTS[self._q_idx][1])
            self.get_logger().info(
                f'APRILTAG FOUND in {QUADRANTS[self._q_idx][2]} '
                f'— tag_pos={self._tag_pos}. Switching to CUBE_HUNT phase.'
            )
            self._start_cube_hunt_phase()

    def _tag_pose_cb(self, msg: Point):
        self._tag_pose = msg

    # ═══════════════════════════════════════════════════════════════════════════
    # Main control loop  (20 Hz)
    # ═══════════════════════════════════════════════════════════════════════════

    def _control(self):
        cmd = Twist()
        now = self.get_clock().now()

        # ── INIT ─────────────────────────────────────────────────────────
        if self._state == 'INIT':
            if self._odom_ready and self._vision_ready:
                self.get_logger().info(
                    f'Mission START (TAG_SEARCH) — heading to {QUADRANTS[self._q_idx][2]}'
                )
                self._pub_apriltag_en.publish(Bool(data=True))
                self._set_nav(QUADRANTS[self._q_idx][:2])
                self._transition('NAV_TO_QUAD')

        # ── NAV_TO_QUAD ───────────────────────────────────────────────────
        elif self._state == 'NAV_TO_QUAD':
            if self._nav_goto(cmd, *self._nav_target):
                label = QUADRANTS[self._q_idx][2]
                self.get_logger().info(
                    f'Arrived at {label} — starting '
                    f'{self._mission_phase} search'
                )
                self._search_elapsed = 0.0
                self._transition('SEARCH_QUAD')

        # ── SEARCH_QUAD ───────────────────────────────────────────────────
        elif self._state == 'SEARCH_QUAD':
            self._search_elapsed += 0.10   # 10 Hz timer
            cmd.angular.z = SEARCH_SPEED

            if self._search_elapsed >= SEARCH_DURATION:
                # Full 360° done in this quadrant
                self._q_visited.add(self._q_idx)
                self.get_logger().info(
                    f'Search done: {QUADRANTS[self._q_idx][2]} '
                    f'| phase={self._mission_phase} '
                    f'| cube_captured={self._cube_captured} '
                    f'| tag_known={self._tag_pos is not None}'
                )
                next_q = self._pick_next_quad()
                if next_q is None:
                    if self._mission_phase == 'TAG_SEARCH':
                        self.get_logger().error('FAILURE — AprilTag not found in any quadrant.')
                    else:
                        self.get_logger().error('FAILURE — cube not found in any quadrant.')
                    self._transition('DONE')
                else:
                    self._q_idx = next_q
                    self._next_after_center = 'NAV_TO_QUAD'
                    self._set_nav((0.0, 0.0))
                    self._transition('NAV_TO_CENTER')

        # ── TARGETING — copied from cube_collector ────────────────────────
        elif self._state == 'TARGETING':
            if self._cube_timed_out(now):
                self._search_elapsed = 0.0   # fresh 360° to re-acquire
                self._transition('SEARCH_QUAD', 'Cube lost.')
            elif self._target_cube is not None:
                error_x = self._target_cube.x
                if abs(error_x) > CUBE_ALIGN_THRESH:
                    cmd.angular.z = _clamp(-KP_ANG_CUBE * error_x,
                                           -MAX_ANG_CUBE, MAX_ANG_CUBE)
                else:
                    self._transition('APPROACHING', 'Aligned. Moving in.')

        # ── APPROACHING — copied from cube_collector ──────────────────────
        elif self._state == 'APPROACHING':
            if self._cube_timed_out(now):
                self._search_elapsed = 0.0   # fresh 360° to re-acquire
                self._transition('SEARCH_QUAD', 'Cube lost.')
            elif self._target_cube is not None:
                cmd.angular.z = -KP_ANG_CUBE * 0.5 * self._target_cube.x
                cmd.linear.x  = APPROACH_SPEED

        # ── CAPTURING — copied from cube_collector ────────────────────────
        elif self._state == 'CAPTURING':
            if (now - self._cube_last_seen).nanoseconds / 1e9 < CUBE_TIMEOUT_S:
                self._cube_lost_time = None
                cmd.linear.x = CAPTURE_SPEED
                if self._target_cube is not None:
                    cmd.angular.z = -KP_ANG_CUBE * 0.3 * self._target_cube.x
            else:
                if self._cube_lost_time is None:
                    self.get_logger().info('Cube lost from view. Performing final push...')
                    self._cube_lost_time = now
                elapsed_since_lost = (now - self._cube_lost_time).nanoseconds / 1e9
                if elapsed_since_lost < EXTRA_PUSH_S:
                    cmd.linear.x = CAPTURE_SPEED
                else:
                    # Cube secured — start delivery
                    self._cube_captured = True
                    self.get_logger().info(
                        f'Capture complete! CUBE CAPTURED in {QUADRANTS[self._q_idx][2]}. '
                        f'Starting DELIVERY to tag_pos={self._tag_pos}'
                    )
                    self._start_delivery_phase()

        # ── NAV_TO_CENTER ─────────────────────────────────────────────────
        elif self._state == 'NAV_TO_CENTER':
            if self._nav_goto(cmd, 0.0, 0.0):
                self.get_logger().info(
                    f'At centre — next: {self._next_after_center}'
                    + (f' ({QUADRANTS[self._q_idx][2]})' if self._next_after_center == 'NAV_TO_QUAD' else '')
                )
                if self._next_after_center == 'NAV_TO_QUAD':
                    self._set_nav(QUADRANTS[self._q_idx][:2])
                    self._transition('NAV_TO_QUAD')
                elif self._next_after_center == 'NAV_TO_TAG_QUAD':
                    self._set_nav(self._tag_pos)
                    self._transition('NAV_TO_TAG_QUAD')

        # ── NAV_TO_TAG_QUAD ───────────────────────────────────────────────
        elif self._state == 'NAV_TO_TAG_QUAD':
            if self._nav_goto(cmd, *self._nav_target):
                self.get_logger().info(
                    f'At tag quadrant {self._nav_target} — starting DELIVERY_SEARCH.'
                )
                self._delivery_search_elapsed = 0.0
                self._transition('DELIVERY_SEARCH')

        # ── DELIVERY_SEARCH ───────────────────────────────────────────────
        elif self._state == 'DELIVERY_SEARCH':
            self._delivery_search_elapsed += 0.10   # 10 Hz timer
            if self._tag_visible and self._tag_pose is not None:
                self.get_logger().info('AprilTag visible — DELIVERY_APPROACH.')
                self._transition('DELIVERY_APPROACH')
            elif self._delivery_search_elapsed >= SEARCH_DURATION:
                self.get_logger().error(
                    'DELIVERY_SEARCH: 360° done, tag not found — depositing at quadrant centre.'
                )
                self._phase_start = None
                self._transition('DETACH')
            else:
                cmd.angular.z = DELIVERY_SEARCH_SPEED

        # ── DELIVERY_APPROACH ─────────────────────────────────────────────
        elif self._state == 'DELIVERY_APPROACH':
            # Check if tag is truly lost (beyond grace period)
            tag_age = 0.0
            if self._tag_last_seen is not None:
                tag_age = (now - self._tag_last_seen).nanoseconds / 1e9

            tag_ok = self._tag_visible and self._tag_pose is not None
            in_grace = (not tag_ok) and tag_age < TAG_GRACE_S and self._tag_pose is not None

            if not tag_ok and not in_grace:
                self.get_logger().warn(f'Tag lost for {tag_age:.1f}s — back to DELIVERY_SEARCH.')
                self._delivery_search_elapsed = 0.0
                self._transition('DELIVERY_SEARCH')
            else:
                err_x  = self._tag_pose.x
                height = self._tag_pose.z   # pixel proxy for closeness
                if tag_ok and height > STOP_HEIGHT:
                    self.get_logger().info(
                        f'DELIVERED! tag_height={height:.0f}px — cube at drop-off.'
                    )
                    self._phase_start = None
                    self._transition('DETACH')
                elif abs(err_x) > TAG_ALIGN_THRESH:
                    cmd.angular.z = _clamp(-KP_ANG_TAG * err_x, -MAX_ANG, MAX_ANG)
                    if in_grace:
                        cmd.linear.x = FOLLOW_SPEED  # keep creeping forward
                else:
                    cmd.linear.x  = FOLLOW_SPEED
                    cmd.angular.z = _clamp(-KP_ANG_TAG * err_x, -MAX_ANG, MAX_ANG)
                self.get_logger().info(
                    f'DELIVERY_APPROACH | err_x={err_x:+.3f} h={height:.0f}px '
                    f'lin={cmd.linear.x:.2f} ang={cmd.angular.z:+.2f}'
                    f'{" (grace)" if in_grace else ""}',
                    throttle_duration_sec=0.5,
                )

        # ── DETACH ────────────────────────────────────────────────────────
        elif self._state == 'DETACH':
            if self._phase_start is None:
                self._phase_start = now
                self.get_logger().info('DETACH: reversing to release cube.')
            elapsed = (now - self._phase_start).nanoseconds / 1e9
            if elapsed < DETACH_TIME:
                cmd.linear.x = DETACH_SPEED
            else:
                self.get_logger().info('DETACH done — heading home.')
                self._phase_start = None
                self._transition('NAV_HOME')

        # ── NAV_HOME ──────────────────────────────────────────────────────
        elif self._state == 'NAV_HOME':
            if self._nav_goto(cmd, 0.0, 0.0):
                self.get_logger().info(
                    f'HOME. Mission COMPLETE | '
                    f'final pose ({self._x:.3f},{self._y:.3f}) '
                    f'θ={math.degrees(self._theta):.1f}°'
                )
                self._transition('DONE')

        # ── DONE ──────────────────────────────────────────────────────────
        elif self._state == 'DONE':
            self.get_logger().info(
                f'DONE | cube_captured={self._cube_captured} '
                f'tag_pos={self._tag_pos} '
                f'visited={self._q_visited}',
                throttle_duration_sec=5.0,
            )

        self._pub_cmd.publish(cmd)
        self._pub_status.publish(String(data=(
            f'phase={self._mission_phase} state={self._state} '
            f'q={QUADRANTS[self._q_idx][2] if self._q_idx < len(QUADRANTS) else "?"} '
            f'cube={self._cube_captured} tag={self._tag_pos is not None} '
            f'x={self._x:.3f} y={self._y:.3f} θ={math.degrees(self._theta):.1f}'
        )))

    # ═══════════════════════════════════════════════════════════════════════════
    # Helpers
    # ═══════════════════════════════════════════════════════════════════════════

    def _nav_goto(self, cmd: Twist, tx: float, ty: float) -> bool:
        """Fill cmd for P-controller navigation. Returns True when arrived."""
        dx = tx - self._x
        dy = ty - self._y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < ARRIVED_DIST:
            return True
        bearing = math.atan2(dy, dx)
        herr    = _norm(bearing - self._theta)
        if abs(herr) > ALIGN_THRESH:
            cmd.angular.z = _clamp(KP_ANG_NAV * herr, -MAX_ANG, MAX_ANG)
        else:
            cmd.linear.x  = _clamp(KP_LIN * dist, MIN_LIN, MAX_LIN)
            cmd.angular.z = _clamp(KP_ANG_NAV * herr, -MAX_ANG, MAX_ANG)
        self.get_logger().info(
            f'{self._state} | dist={dist:.3f}m herr={math.degrees(herr):+.1f}° '
            f'→({tx:.2f},{ty:.2f})',
            throttle_duration_sec=0.5,
        )
        return False

    def _set_nav(self, pos):
        self._nav_target = (float(pos[0]), float(pos[1]))

    def _transition(self, new_state: str, reason: str = ''):
        self.get_logger().info(f'[{self._state}] -> [{new_state}] | {reason}')
        self._state = new_state

    def _start_cube_hunt_phase(self):
        """Transition TAG_SEARCH → CUBE_HUNT: disable apriltag, reset search."""
        self._mission_phase = 'CUBE_HUNT'
        self._pub_apriltag_en.publish(Bool(data=False))
        self.get_logger().info(
            'Phase: CUBE_HUNT — apriltag DISABLED, YOLO at full speed.'
        )
        # Reset quadrant search to visit all quadrants for the cube
        self._q_idx = 0
        self._q_visited = set()
        self._set_nav((0.0, 0.0))
        self._next_after_center = 'NAV_TO_QUAD'
        self._transition('NAV_TO_CENTER')

    def _start_delivery_phase(self):
        """Transition CUBE_HUNT → DELIVERY: re-enable apriltag, navigate to tag."""
        self._mission_phase = 'DELIVERY'
        self._pub_apriltag_en.publish(Bool(data=True))
        self.get_logger().info('Phase: DELIVERY — apriltag RE-ENABLED.')
        self._set_nav((0.0, 0.0))
        self._next_after_center = 'NAV_TO_TAG_QUAD'
        self._transition('NAV_TO_CENTER')

    def _cube_timed_out(self, now) -> bool:
        return (now - self._cube_last_seen).nanoseconds * 1e-9 > CUBE_LOST_TIMEOUT_S

    def _pick_next_quad(self) -> int | None:
        """Next quadrant index not yet fully searched and not the current one."""
        for i in range(len(QUADRANTS)):
            if i not in self._q_visited and i != self._q_idx:
                return i
        return None


def main(args=None):
    rclpy.init(args=args)
    node = FinalTestNode()
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
