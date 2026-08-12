"""
executor_node.py — Milestone 5 unified mission executor.

Replaces brain_node.py's role. Explores an unknown arena, collects cubes, and
delivers each one back to the pose the robot started from.

Mission loop
------------
    STARTUP     latch HOME = map->base_link once TF is available
    EXPLORING   frontier_explorer_node drives; interrupt on a real cube detection
    TARGETING   rotate in place to centre the cube (P-control on detected_cube.x)
    APPROACHING drive in, holding centre, until the cube enters the depth blind spot
    CAPTURING   keep pushing while the cube is still seen, then a short blind push
    DELIVERING  NavigateToPose(HOME)
    DETACHING   reverse to release the cube
                -> back to EXPLORING for the next one

Why HOME comes from TF and not odom
-----------------------------------
final_test_node.py's NAV_HOME drives to odom (0, 0), which is free because odom
starts at zero by definition — but odom drifts, and this project has already lost
time to exactly that (see docs/ghost_map_investigation.md: wheel slip corrupting
the map until the EKF was made to fuse IMU yaw). After a long exploration run,
odom (0, 0) can be metres from the true start. Latching map->base_link instead
means SLAM loop closure keeps HOME honest, and Nav2 plans a real route back
rather than dead-reckoning.

Who owns cmd_vel
----------------
Only one controller may drive the base at a time. This node publishes cmd_vel
*only* in TARGETING / APPROACHING / CAPTURING / DETACHING. During EXPLORING and
DELIVERING, Nav2 is driving and this node stays silent; it hands exploration
on and off via the latched /exploration_enabled topic, and frontier_explorer_node
cancels its in-flight goal when disabled.

That statement used to be aspirational rather than true: botzilla_control's
velocity_smoother sits downstream of Nav2 (cmd_vel_nav -> cmd_vel) and publishes on
its own 20 Hz timer for as long as nav2.launch.py is alive, with no awareness of
this node's state at all — so it was a second, independent /cmd_vel publisher the
entire time TARGETING/APPROACHING/CAPTURING/DETACHING were also publishing. Usually
harmless-looking (it just decays to zero and idles there), but right after a
NavigateToPose goal succeeds it can still be mid-decay of real leftover velocity at
the exact moment DETACHING starts its own reverse ramp — hardware testing traced a
vibration right at the instant reversing began to precisely that collision, on a
build where DETACHING already had a settle delay on this node's own side (which
cannot fix a race whose other half doesn't respect it). This node now toggles
velocity_smoother's mute switch (/velocity_smoother_enabled, latched) on every
transition — enabled only for EXPLORING/DELIVERING, disabled everywhere this node
drives — so the ownership claim above is actually enforced, not just documented.

Usage
-----
    ros2 launch botzilla_navigation executor.launch.py
requires simulation.launch.py (or hardware.launch.py), rtabmap.launch.py,
nav2.launch.py and a source of /detected_cube (yolo_node) already running.
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
import tf2_ros
from tf2_ros import TransformException

# ── Cube collection ──────────────────────────────────────────────────────────
# Values carried over from final_test_node.py, which is the version proven on
# hardware. NOTE: they were tuned against YOLO at ~0.75 fps on a Pi 5. On the
# Jetson (GPU container) inference is far faster, so EXTRA_PUSH_S covers much
# less ground than it used to — re-measure before trusting it.
KP_ANGULAR = 1.2            # P-gain for centring the cube
MAX_ANGULAR = 0.35          # rad/s clamp; prevents overshoot between slow frames
ALIGNMENT_THRESHOLD = 0.03  # normalized; below this we consider the cube centred
APPROACH_SPEED = 0.15       # m/s while driving toward the cube
CAPTURE_SPEED = 0.12        # m/s during the final push
# Ignore detections beyond this, so one noisy frame can't send the robot chasing a
# cube across the arena. Raised 1.0 -> 1.5 after hardware measurement: a cube sitting
# a normal distance in front of the robot ranged at 1.09 m and was silently dropped by
# the old gate while YOLO was reporting it confidently every frame. 1.0 came from
# final_test_node's scripted small-arena search and is too tight for open exploration.
# 1.5 still rejects far-field noise, which in a measured frame topped out at 0.138
# confidence versus 0.797 for the real cube.
CUBE_MAX_RANGE_M = 1.5
# Once DETACHING releases a cube, it sits right in front of the robot — well within
# CUBE_MAX_RANGE_M — so EXPLORING immediately re-detects and re-collects the same
# cube. A per-cube identity check isn't available (cubes aren't distinguishable), and
# a time-based cooldown can expire before Nav2 actually starts moving the robot away
# from HOME. So proximity to HOME is the signal instead: HOME is the drop-off point,
# cubes are expected to accumulate there over a mission, and none of them should ever
# be re-collected regardless of how long ago they were dropped. Estimated from Nav2's
# xy_goal_tolerance (0.15m) + the DETACHING reverse distance (~0.5m) + margin —
# confirm against where the cube actually ends up on hardware and retune if it's
# clipping legitimate nearby cubes or not covering the dropped one.
HOME_CUBE_SUPPRESS_RADIUS_M = 1.0
# No detection for this long -> give up and resume exploring.
CUBE_LOST_TIMEOUT_S = 5.0
# In CAPTURING, how long without a detection counts as "the cube has genuinely left
# the frame" rather than just a dropped frame.
CAPTURE_GRACE_S = 2.0
# Blind push after the cube leaves frame, to seat it between the arms. The depth
# blind spot is ~0.55 m, so the cube stops being reported well before it is captured.
EXTRA_PUSH_S = 0.1
# Consecutive z == 0.0 readings before believing the blind-spot signal.
BLIND_SPOT_FRAMES = 2

# ── Detach ───────────────────────────────────────────────────────────────────
DETACH_SPEED = -0.10        # m/s, reverse
# 1.8s (~0.18m nominal) was measured on hardware to not reliably clear the grabber
# arms — the cube sometimes stayed pinned. Raised to give real clearance margin.
DETACH_TIME_S = 5.0
# nav2_params.yaml's general_goal_checker only checks xy/yaw tolerance, not velocity
# — NavigateToPose can report SUCCEEDED while the robot still has real residual
# motion. DETACHING's slew-rate ramp (see MAX_LINEAR_ACCEL) assumes it starts from
# rest, so handing off straight into the reverse ramp while Nav2's last motion
# hasn't fully decayed collides a "smooth" setpoint with genuine leftover velocity —
# hardware testing found this reproduces the same abrupt-transition vibration as the
# unfixed APPROACHING case, right at the instant reversing starts. Command an
# explicit stop for this long first, so the reverse ramp genuinely begins from rest.
DETACH_SETTLE_S = 0.5

# ── Command smoothing ────────────────────────────────────────────────────────
# Unlike Nav2's path (controller_server -> velocity_smoother -> cmd_vel), this node
# publishes its P-control output straight to cmd_vel with no shaping at all — most
# visibly, entering APPROACHING steps linear.x from 0 to APPROACH_SPEED in one 100ms
# tick. Hardware testing traced a "go a bit, stop a bit" vibration during APPROACHING
# to kobuki_base_node's encoder-tick-rejection bursts, which only ever showed up
# under commands with abrupt transitions (direction/speed changes) — never under
# steady-state driving or pure rotation. Ramping every published command limits how
# fast the setpoint can change per tick, so state-entry steps become gradual instead
# of instantaneous, without changing what any state ultimately asks for.
MAX_LINEAR_ACCEL = 0.4      # m/s^2 -> 0 to APPROACH_SPEED in ~0.4s
MAX_ANGULAR_ACCEL = 1.5     # rad/s^2 -> 0 to MAX_ANGULAR in ~0.25s

# ── Delivery ─────────────────────────────────────────────────────────────────
# Generous: the route home can span the whole arena and Nav2 may run recoveries.
DELIVERY_TIMEOUT_S = 300.0
HOME_TF_WAIT_S = 60.0       # how long to wait at STARTUP for map->base_link

# nav2_params.yaml's FollowPath.min_vel_x is deliberately -0.10 (not 0.0) so DWB can
# back out of a wedge during EXPLORING — that's load-bearing and must stay in place
# for exploration/sweeping. But the grabber has no lock: hardware testing found that
# any reverse motion while carrying a cube drops it, which was silently producing a
# "second" cube detection that was actually the first one falling out mid-delivery.
# So min_vel_x is pushed to 0.0 for the HOME goal only, and restored the moment
# DELIVERING ends (success, failure, or timeout) — every other state, including
# EXPLORING, keeps the wedge-escape behaviour untouched.
NAV2_MIN_VEL_X_DEFAULT = -0.10  # must match FollowPath.min_vel_x in nav2_params.yaml

CONTROL_PERIOD_S = 0.1      # 10 Hz
MAP_FRAME = 'map'
ROBOT_FRAME = 'base_link'


class State:
    STARTUP = 'STARTUP'
    EXPLORING = 'EXPLORING'
    TARGETING = 'TARGETING'
    APPROACHING = 'APPROACHING'
    CAPTURING = 'CAPTURING'
    DELIVERING = 'DELIVERING'
    DETACHING = 'DETACHING'


class ExecutorNode(Node):
    def __init__(self):
        super().__init__('executor_node')

        self._state = State.STARTUP
        self._home = None            # (x, y, yaw) in MAP_FRAME, latched at STARTUP
        self._target_cube = None     # geometry_msgs/Point: x=norm offset, z=metres
        self._cube_last_seen = None  # rclpy.time.Time
        self._cube_lost_time = None  # when the cube left frame during CAPTURING
        self._blind_spot_frames = 0
        self._phase_start = self.get_clock().now()
        self._startup_start = self.get_clock().now()
        self._cubes_delivered = 0
        self._last_cmd_linear = 0.0   # for slew-limiting, see MAX_LINEAR_ACCEL
        self._last_cmd_angular = 0.0

        self._nav_goal_handle = None
        self._nav_result = None      # None while in flight; GoalStatus once finished
        self._nav_sent_time = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Latched: frontier_explorer_node must see the current value even if it
        # starts after us, otherwise it would happily explore while we chase a cube.
        enable_qos = QoSProfile(depth=1)
        enable_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._explore_pub = self.create_publisher(
            Bool, '/exploration_enabled', enable_qos
        )
        # See "Who owns cmd_vel" above — mutes velocity_smoother whenever this node
        # is the one driving the base, so the two never race on the same topic.
        self._smoother_enable_pub = self.create_publisher(
            Bool, 'velocity_smoother_enabled', enable_qos
        )
        self._status_pub = self.create_publisher(String, '/mission/status', 10)

        self.create_subscription(Point, 'detected_cube', self._cube_cb, 10)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._controller_param_client = AsyncParameterClient(self, 'controller_server')

        # Hold exploration off until HOME is latched — otherwise the robot could
        # drive away before we ever record where it started, and HOME would be
        # wherever it happened to be when TF finally came up.
        self._publish_exploration_enabled(False)

        self.create_timer(CONTROL_PERIOD_S, self._control_loop)
        self.get_logger().info(
            'executor_node started — waiting for map->base_link to latch HOME.'
        )

    # ------------------------------------------------------------------ #
    # Subscriptions
    # ------------------------------------------------------------------ #

    def _cube_cb(self, msg: Point):
        """Cache the latest cube detection, and react to it where relevant."""
        # Only chase cubes while hunting. During DELIVERING/DETACHING the robot is
        # already carrying one, and a detection of the cube it is holding (or of the
        # next one) must not derail the delivery.
        if self._state in (State.STARTUP, State.DELIVERING, State.DETACHING):
            return
        # See HOME_CUBE_SUPPRESS_RADIUS_M — a cube dropped off at HOME must not be
        # immediately re-collected once EXPLORING resumes.
        if self._state == State.EXPLORING and self._near_home():
            return

        # z == 0.0 is the blind-spot sentinel from yolo_node, not a real distance, so
        # it must bypass the range gate — it is precisely the signal that the cube is
        # close enough to capture.
        if msg.z > CUBE_MAX_RANGE_M:
            return

        self._target_cube = msg
        self._cube_last_seen = self.get_clock().now()

        if self._state == State.EXPLORING:
            self.get_logger().info(
                f'Cube detected (x={msg.x:+.2f}, z={msg.z:.2f}m) — '
                f'suspending exploration to collect it.'
            )
            self._publish_exploration_enabled(False)
            self._transition(State.TARGETING)

        elif self._state == State.APPROACHING:
            # Debounced: a single spurious z==0.0 frame should not trigger the blind
            # push, which is irreversible in the sense that the cube is then unseeable.
            if msg.z == 0.0:
                self._blind_spot_frames += 1
                if self._blind_spot_frames >= BLIND_SPOT_FRAMES:
                    self._transition(State.CAPTURING, 'Blind spot confirmed.')
            else:
                self._blind_spot_frames = 0

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #

    def _control_loop(self):
        cmd = Twist()
        now = self.get_clock().now()

        if self._state == State.STARTUP:
            self._do_startup(now)

        elif self._state == State.EXPLORING:
            # frontier_explorer_node + Nav2 own the base here. Publish nothing.
            self.get_logger().info(
                f'EXPLORING — delivered {self._cubes_delivered} cube(s) so far.',
                throttle_duration_sec=15.0,
            )
            self._publish_status()
            return

        elif self._state == State.TARGETING:
            self._do_targeting(cmd, now)

        elif self._state == State.APPROACHING:
            self._do_approaching(cmd, now)

        elif self._state == State.CAPTURING:
            self._do_capturing(cmd, now)

        elif self._state == State.DELIVERING:
            # Nav2 owns the base here. Publish nothing.
            self._do_delivering(now)
            self._publish_status()
            return

        elif self._state == State.DETACHING:
            self._do_detaching(cmd, now)

        max_dv = MAX_LINEAR_ACCEL * CONTROL_PERIOD_S
        max_dw = MAX_ANGULAR_ACCEL * CONTROL_PERIOD_S
        cmd.linear.x = self._slew_limit(cmd.linear.x, self._last_cmd_linear, max_dv)
        cmd.angular.z = self._slew_limit(cmd.angular.z, self._last_cmd_angular, max_dw)
        self._last_cmd_linear = cmd.linear.x
        self._last_cmd_angular = cmd.angular.z

        self._cmd_pub.publish(cmd)
        self._publish_status()

    # ------------------------------------------------------------------ #
    # States
    # ------------------------------------------------------------------ #

    def _do_startup(self, now):
        """Latch HOME from TF, then release exploration."""
        pose = self._get_robot_pose()
        if pose is None:
            waited = (now - self._startup_start).nanoseconds / 1e9
            if waited > HOME_TF_WAIT_S:
                self.get_logger().error(
                    f'No {MAP_FRAME}->{ROBOT_FRAME} transform after {waited:.0f}s. '
                    f'Is SLAM (rtabmap.launch.py) running? Still waiting.',
                    throttle_duration_sec=15.0,
                )
            else:
                self.get_logger().info(
                    f'Waiting for {MAP_FRAME}->{ROBOT_FRAME} to latch HOME '
                    f'({waited:.0f}s)...',
                    throttle_duration_sec=5.0,
                )
            return

        self._home = pose
        self.get_logger().info(
            f'HOME latched at x={pose[0]:.3f} y={pose[1]:.3f} '
            f'yaw={math.degrees(pose[2]):.1f}deg in "{MAP_FRAME}". '
            f'Every collected cube will be delivered here.'
        )
        self._publish_exploration_enabled(True)
        self._transition(State.EXPLORING, 'HOME latched.')

    def _do_targeting(self, cmd, now):
        """Rotate in place until the cube is centred."""
        if self._cube_timed_out(now):
            self._resume_exploring('Cube lost while targeting.')
            return
        if self._target_cube is None:
            return

        error_x = self._target_cube.x
        if abs(error_x) > ALIGNMENT_THRESHOLD:
            cmd.angular.z = self._clamp_angular(-KP_ANGULAR * error_x)
        else:
            self._transition(
                State.APPROACHING, f'Centred (x={error_x:+.3f}). Driving in.'
            )

    def _do_approaching(self, cmd, now):
        """Drive toward the cube, holding it centred. Exit is via _cube_cb."""
        if self._cube_timed_out(now):
            self._resume_exploring('Cube lost while approaching.')
            return
        if self._target_cube is None:
            return
        cmd.angular.z = self._clamp_angular(-KP_ANGULAR * 0.5 * self._target_cube.x)
        cmd.linear.x = APPROACH_SPEED

    def _do_capturing(self, cmd, now):
        """Push until the cube leaves the frame, then a short blind push more.

        The depth blind spot (~0.55 m) means the cube stops being reported well
        before it is actually between the arms, so the last thing to trust is the
        moment detections stop — then drive EXTRA_PUSH_S beyond it.
        """
        since_seen = (now - self._cube_last_seen).nanoseconds / 1e9

        if since_seen < CAPTURE_GRACE_S:
            self._cube_lost_time = None
            cmd.linear.x = CAPTURE_SPEED
            if self._target_cube is not None:
                cmd.angular.z = self._clamp_angular(
                    -KP_ANGULAR * 0.3 * self._target_cube.x
                )
            return

        if self._cube_lost_time is None:
            self._cube_lost_time = now
            self.get_logger().info('Cube left the frame — final blind push.')

        if (now - self._cube_lost_time).nanoseconds / 1e9 < EXTRA_PUSH_S:
            cmd.linear.x = CAPTURE_SPEED
            return

        if self._home is None:
            # Should be impossible: STARTUP gates everything on HOME being latched.
            self.get_logger().error('Cube captured but HOME was never latched!')
            self._resume_exploring('No HOME to deliver to.')
            return

        self.get_logger().info('Cube captured — delivering to HOME.')
        self._transition(State.DELIVERING)
        self._send_home_goal()

    def _do_delivering(self, now):
        """Wait on the NavigateToPose(HOME) goal."""
        if self._nav_result is not None:
            status = self._nav_result
            self._nav_result = None
            self._nav_goal_handle = None
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._transition(State.DETACHING, 'Arrived HOME.')
            else:
                # Releasing here is deliberate. The robot is somewhere short of HOME,
                # but dropping the cube and carrying on beats wedging the whole
                # mission on one failed route — and the cube stays findable.
                self.get_logger().warn(
                    f'Delivery goal ended with status {status} instead of SUCCEEDED. '
                    f'Releasing the cube here and resuming exploration.'
                )
                self._transition(State.DETACHING, 'Delivery failed; releasing anyway.')
            return

        if self._nav_sent_time is None:
            return
        elapsed = (now - self._nav_sent_time).nanoseconds / 1e9
        if elapsed > DELIVERY_TIMEOUT_S:
            self.get_logger().warn(
                f'Delivery exceeded {DELIVERY_TIMEOUT_S:.0f}s; giving up on the route '
                f'and releasing the cube here.'
            )
            if self._nav_goal_handle is not None:
                self._nav_goal_handle.cancel_goal_async()
                self._nav_goal_handle = None
            self._nav_sent_time = None
            self._transition(State.DETACHING, 'Delivery timed out.')
        else:
            self.get_logger().info(
                f'DELIVERING to HOME ({elapsed:.0f}s elapsed)...',
                throttle_duration_sec=10.0,
            )

    def _do_detaching(self, cmd, now):
        """Settle any residual motion, then reverse to leave the cube behind."""
        elapsed = (now - self._phase_start).nanoseconds / 1e9
        if elapsed < DETACH_SETTLE_S:
            return  # cmd stays zero — see DETACH_SETTLE_S
        if elapsed - DETACH_SETTLE_S < DETACH_TIME_S:
            cmd.linear.x = DETACH_SPEED
            return
        self._cubes_delivered += 1
        self.get_logger().info(
            f'Cube released. Total delivered: {self._cubes_delivered}.'
        )
        self._resume_exploring('Delivery complete.')

    # ------------------------------------------------------------------ #
    # Nav2 delivery goal
    # ------------------------------------------------------------------ #

    def _send_home_goal(self):
        self._nav_result = None
        self._nav_goal_handle = None
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'navigate_to_pose action server unavailable; cannot deliver. '
                'Releasing the cube here.'
            )
            self._transition(State.DETACHING, 'No Nav2 server.')
            return

        self._set_nav2_reverse_allowed(False)

        x, y, yaw = self._home
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._nav_sent_time = self.get_clock().now()
        self.get_logger().info(f'Sending NavigateToPose to HOME ({x:.2f}, {y:.2f}).')
        self._nav_client.send_goal_async(goal).add_done_callback(self._goal_response_cb)

    def _set_nav2_reverse_allowed(self, allowed: bool):
        """Push FollowPath.min_vel_x to 0.0 (or restore it) — see NAV2_MIN_VEL_X_DEFAULT."""
        value = NAV2_MIN_VEL_X_DEFAULT if allowed else 0.0
        self._controller_param_client.set_parameters(
            [Parameter('FollowPath.min_vel_x', Parameter.Type.DOUBLE, value)]
        )

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the HOME goal.')
            self._nav_result = GoalStatus.STATUS_ABORTED
            return
        self._nav_goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        # Recorded rather than acted on directly: the state machine owns transitions,
        # and this fires on an executor thread.
        self._nav_result = future.result().status

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _get_robot_pose(self):
        """Return (x, y, yaw) of the robot in MAP_FRAME, or None if TF isn't ready."""
        try:
            tf = self._tf_buffer.lookup_transform(
                MAP_FRAME, ROBOT_FRAME, rclpy.time.Time()
            )
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (t.x, t.y, yaw)

    def _near_home(self):
        """True if the robot is currently within HOME_CUBE_SUPPRESS_RADIUS_M of HOME."""
        if self._home is None:
            return False
        pose = self._get_robot_pose()
        if pose is None:
            return False
        dx = pose[0] - self._home[0]
        dy = pose[1] - self._home[1]
        return (dx * dx + dy * dy) < HOME_CUBE_SUPPRESS_RADIUS_M ** 2

    def _publish_exploration_enabled(self, enabled: bool):
        self._explore_pub.publish(Bool(data=enabled))

    def _resume_exploring(self, reason=''):
        self._target_cube = None
        self._cube_lost_time = None
        self._blind_spot_frames = 0
        self._publish_exploration_enabled(True)
        self._transition(State.EXPLORING, reason)

    def _transition(self, new_state, reason=''):
        self.get_logger().info(f'[{self._state}] -> [{new_state}] | {reason}')
        self._state = new_state
        self._phase_start = self.get_clock().now()
        # Every state entry ramps from rest — see MAX_LINEAR_ACCEL comment. Simpler
        # and safer than trying to carry a velocity across a state boundary whose
        # target profile (gain, speed) is about to change anyway.
        self._last_cmd_linear = 0.0
        self._last_cmd_angular = 0.0
        if new_state in (State.TARGETING, State.APPROACHING):
            self._blind_spot_frames = 0
        if new_state == State.DETACHING:
            # Covers every DELIVERING exit (arrived, failed, timed out) uniformly,
            # and is a harmless no-op on paths that never lowered it in the first
            # place (e.g. no Nav2 server available).
            self._set_nav2_reverse_allowed(True)
        # See "Who owns cmd_vel" — only one of Nav2 (via velocity_smoother) or this
        # node's own control loop may publish cmd_vel at a time.
        self._smoother_enable_pub.publish(
            Bool(data=new_state in (State.EXPLORING, State.DELIVERING))
        )

    def _cube_timed_out(self, now):
        if self._cube_last_seen is None:
            return True
        return (now - self._cube_last_seen).nanoseconds / 1e9 > CUBE_LOST_TIMEOUT_S

    @staticmethod
    def _clamp_angular(value):
        return max(-MAX_ANGULAR, min(MAX_ANGULAR, value))

    @staticmethod
    def _slew_limit(target, last, max_delta):
        return max(last - max_delta, min(last + max_delta, target))

    def _publish_status(self):
        home = (
            f'({self._home[0]:.2f},{self._home[1]:.2f})' if self._home else 'unset'
        )
        cube = (
            f'x={self._target_cube.x:+.2f},z={self._target_cube.z:.2f}'
            if self._target_cube else 'none'
        )
        self._status_pub.publish(String(data=(
            f'state={self._state} home={home} cube={cube} '
            f'delivered={self._cubes_delivered}'
        )))


def main(args=None):
    rclpy.init(args=args)
    node = ExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cmd_pub.publish(Twist())  # stop the base on the way out
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
