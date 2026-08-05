"""
frontier_explorer_node.py.

Milestone 4 (PHASE_1_IMPLEMENTATION_PLAN.md): autonomous frontier exploration. Watches
RTAB-Map's /map, finds frontiers (free cells bordering unknown space) via
frontier_detection.py, and drives the robot to the nearest one through Nav2's
NavigateToPose action — no manual driving, no scripted goals.

Replanning policy: a new goal is only sent when the explorer is IDLE (no goal currently
active). An in-flight Nav2 goal is never preempted for a newly-found target. This is a
deliberate choice, not the literal "replan on every map update" a naive reading might
suggest: this project's own Nav2 debugging (see PHASE_1_IMPLEMENTATION_PLAN.md's "DWB
rotate-in-place stall" fix) found that repeatedly preempting/resending goals starves real
progress, especially under the arena's low Gazebo real-time factor.

Reachability pre-check: before committing to a full NavigateToPose attempt (which can run
for a long time — see "Stall watchdog" below), this node first asks Nav2's planner
directly via ComputePathToPose whether ANY path to the target exists at all. That call
typically resolves in well under a second. A target with NO_VALID_PATH (behind a wall the
planner already knows about, outside the map, etc.) is skipped and blacklisted immediately
instead of spending a full navigation attempt — and the stall/exponential-backoff cost —
finding out the same thing the slow way. This does not replace the stall watchdog: a path
existing at plan time doesn't guarantee the local controller can still execute it a moment
later (dynamic obstacles, costmap changes mid-transit), so a real NavigateToPose attempt
can still stall and still needs its own watchdog.

Stall watchdog: this node does NOT rely solely on Nav2's own recovery behaviors to give
up on a bad goal. A genuinely-unreachable target was observed live taking Nav2 ~300s to
internally exhaust its recovery cycles and report ABORTED — for that entire time this
node would otherwise sit doing nothing, which looks indistinguishable from "exploration
is stuck" from the outside. But a FLAT timeout is the wrong tool here: a legitimate goal
on the far side of a large arena can genuinely take a while, and canceling it early would
throw away real progress. So the watchdog tracks the NavigateToPose feedback's
distance_remaining and only intervenes when it stops decreasing (NO_PROGRESS_TIMEOUT_S of
no meaningful improvement) — a robot that is actually driving toward its goal is never
touched, however long that takes. GOAL_ABS_TIMEOUT_S is a separate, much longer backstop
for the degenerate case where feedback never arrives at all, so the watchdog isn't fully
dependent on the feedback channel working. Either trigger cancels the goal itself and
treats it like any other non-SUCCEEDED result (blacklist + return to IDLE, see
_result_cb).

Both action clients (compute_path_to_pose and navigate_to_pose) go through the same
unaccepted-goal gap: their done-callback is normally near-instant, but if the action
server dies or is unresponsive mid-handshake (observed live on this hardware — Nav2's
servers took 90+s to even become responsive under CPU load), that callback may never
fire at all, leaving no goal_handle to cancel and no feedback channel to watch. Both
_check_stall (NAVIGATING) and _check_path_stall (CHECKING_PATH) bound this case
directly against elapsed time since the request was sent, independent of ever getting a
handle. Giving up this way opens a narrow race: the real done-callback can still land
afterward, past the point this node already moved on to a different target. A per-attempt
epoch counter (_goal_epoch / _path_epoch) lets _goal_response_cb / _path_goal_response_cb
recognize a callback that belongs to an attempt this node already abandoned; if that
stale response turns out to have been accepted after all, it's canceled as an orphan
instead of silently overwriting newer state or leaving the robot executing a goal this
node no longer tracks or can blacklist.

Evaluation is driven by a periodic timer (EVAL_PERIOD_S), not directly by /map arrival.
/map only republishes when rtabmap's map actually changes, which itself only happens when
the robot moves — so reacting solely to new messages creates a deadlock if the very first
evaluation fails for a transient reason (e.g. TF not ready yet, a few hundred ms after
launch): no further /map messages ever arrive to retry with, since nothing moved to change
the map. The timer always re-evaluates against the most recently received map, so a
transient startup failure self-heals on the next tick instead of stalling exploration
permanently.

Failed-target cooldown: a target whose Nav2 goal doesn't SUCCEED (ABORTED/CANCELED/timed
out) is blacklisted for a cooldown period — observed live that a frontier just inside a
tight doorway can fail the exact same way every time (repeated "Failed to make progress"),
and since nothing about the map changes when a goal merely fails, the unfiltered nearest-
frontier selection would just re-target the identical unreachable spot forever.

The cooldown uses exponential backoff, not a flat duration: a single Nav2 attempt on a
genuinely bad target was observed live to take ~300s to abort on Nav2's own recovery
timeline (multiple internal recovery cycles before giving up) before this node's own
stall watchdog existed. BLACKLIST_COOLDOWN_BASE_S is sized against that ~300s figure
rather than the (much shorter, in the common case) no-progress timeout, since a target
can still fail via Nav2's own ABORTED before the watchdog ever gets a chance to fire, and
the cooldown has to survive either path. A flat cooldown shorter than that doesn't work —
with two persistently-bad frontiers A and B and a 90s flat cooldown, A fails and is banned
for 90s, B is tried and takes up to ~300s to also fail, and by then A's 90s ban has long
since expired — so the two just ping-pong forever, never giving the OTHER known-good
frontiers a turn (this exact failure mode was observed live before this fix). Repeated
failures at the same spot
(tracked persistently across cooldown expiries, not just within one active window) double
the cooldown each time, up to BLACKLIST_COOLDOWN_MAX_S — so a truly unreachable frontier
gets pushed out far enough to let the rest of the map be explored, while a one-off failure
doesn't get penalized as harshly.

Usage:
  ros2 launch botzilla_navigation frontier_explorer.launch.py
"""

import math

from action_msgs.msg import GoalStatus
from botzilla_navigation.frontier_detection import (
    distance,
    find_frontiers,
    grid_to_world,
    select_target,
)
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
import tf2_ros
from tf2_ros import TransformException

# Clusters smaller than this many cells are discarded as noise (isolated frontier pixels
# at map edges, sensor-noise artifacts) rather than sent to as navigation goals.
MIN_FRONTIER_CLUSTER_SIZE = 8

# Skip sending a goal this close to the robot — avoids degenerate short hops when the
# nearest frontier is only marginally outside the robot's own footprint/goal tolerance.
MIN_TARGET_DISTANCE_M = 0.3

# How often to re-evaluate (using the latest cached /map) whether a new goal should be
# sent. Independent of /map's own publish rate — see module docstring.
EVAL_PERIOD_S = 2.0

# Cancel the active goal if NavigateToPose's own distance_remaining feedback hasn't
# improved by at least PROGRESS_EPSILON_M in this long — see module docstring's "Stall
# watchdog" section. A goal making real progress, however slowly, is never touched.
NO_PROGRESS_TIMEOUT_S = 30.0
PROGRESS_EPSILON_M = 0.15

# Absolute backstop regardless of the progress signal, for the degenerate case where
# feedback never arrives at all. Set well above the ~300s worst-case Nav2-internal abort
# latency observed live, so it only ever fires if the progress watchdog itself is broken.
# Also used to bound a NavigateToPose goal that never gets accepted/rejected at all (no
# goal_handle ever arrives) — see module docstring.
GOAL_ABS_TIMEOUT_S = 420.0

# How long to wait for a compute_path_to_pose response before giving up on the
# reachability check directly. Reachability checks normally resolve in well under a
# second (see module docstring) — this is a generous backstop against the exchange
# itself hanging (the planner's action server unresponsive under CPU load), not a
# tolerance for legitimately slow planning, hence far shorter than GOAL_ABS_TIMEOUT_S.
PATH_CHECK_TIMEOUT_S = 15.0

# A target within this radius of a recently-failed target is treated as the same spot and
# excluded until its cooldown expires (see module docstring).
BLACKLIST_RADIUS_M = 0.5

# Exponential backoff: cooldown = min(BASE * 2^(failure_count - 1), MAX). BASE is set well
# above the ~300s single-attempt failure latency observed live (see module docstring).
BLACKLIST_COOLDOWN_BASE_S = 360.0
BLACKLIST_COOLDOWN_MAX_S = 3600.0

MAP_FRAME = 'map'
ROBOT_FRAME = 'base_link'


class State:
    IDLE = 'IDLE'
    CHECKING_PATH = 'CHECKING_PATH'
    NAVIGATING = 'NAVIGATING'
    EXPLORATION_COMPLETE = 'EXPLORATION_COMPLETE'


class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('frontier_explorer_node')

        self._state = State.IDLE
        self._goal_handle = None
        self._latest_map = None
        self._current_target = None
        self._pending_yaw = None  # yaw computed for the target currently under path-check
        self._path_check_start_time = None  # rclpy.time.Time the path-check was sent
        self._path_epoch = 0  # bumped per path-check attempt; guards stale callbacks
        self._goal_start_time = None  # rclpy.time.Time when the active goal was sent
        self._goal_epoch = 0  # bumped per NavigateToPose attempt; guards stale callbacks
        self._cancel_requested = False  # avoid re-issuing cancel_goal_async every tick
        self._last_progress_distance = None  # smallest distance_remaining seen so far
        self._last_progress_time = None  # rclpy.time.Time it was last improved
        self._blacklist = []  # list of (x, y, expiry_time: rclpy.time.Time) — active bans
        self._failure_history = []  # list of [x, y, count] — persists across cooldowns

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)

        complete_qos = QoSProfile(depth=1)
        complete_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._complete_pub = self.create_publisher(Bool, '/exploration_complete', complete_qos)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        self.create_timer(EVAL_PERIOD_S, self._evaluate)

        self.get_logger().info('frontier_explorer_node started, waiting for /map ...')

    # ------------------------------------------------------------------ #
    # /map callback — just caches the latest message (see module docstring
    # for why evaluation itself is timer-driven, not callback-driven).
    # ------------------------------------------------------------------ #

    def _map_cb(self, msg: OccupancyGrid):
        self._latest_map = msg

    # ------------------------------------------------------------------ #
    # Periodic evaluation — drives the whole state machine
    # ------------------------------------------------------------------ #

    def _evaluate(self):
        self.get_logger().info(
            f'[heartbeat] state={self._state} target={self._current_target} '
            f'blacklist_active={len(self._blacklist)} '
            f'blacklist_known={len(self._failure_history)}',
            throttle_duration_sec=5.0,
        )
        if self._state == State.NAVIGATING:
            # Never preempt an in-flight goal with a new target — see module
            # docstring — but do enforce the stall watchdog against it.
            self._check_stall()
            return
        if self._state == State.CHECKING_PATH:
            # Reachability pre-check in flight for the current target; wait for its
            # callback rather than picking a second candidate concurrently, but bound
            # how long we wait — see _check_path_stall.
            self._check_path_stall()
            return
        if self._state == State.EXPLORATION_COMPLETE:
            return
        if self._latest_map is None:
            self.get_logger().info('Still waiting for the first /map message.',
                                    throttle_duration_sec=5.0)
            return

        msg = self._latest_map

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return
        robot_x, robot_y = robot_pose

        clusters = find_frontiers(
            msg.data, msg.info.width, msg.info.height, MIN_FRONTIER_CLUSTER_SIZE
        )
        self.get_logger().info(
            f'/map is {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f}m/cell, '
            f'{len(clusters)} frontier cluster(s) found this tick.',
            throttle_duration_sec=5.0,
        )
        frontiers_world = [
            grid_to_world(r, c, msg.info.resolution,
                          msg.info.origin.position.x, msg.info.origin.position.y)
            for (r, c, _size) in clusters
        ]

        if not frontiers_world:
            self._state = State.EXPLORATION_COMPLETE
            self._complete_pub.publish(Bool(data=True))
            self.get_logger().info('No frontiers remain — exploration complete.')
            return

        candidates = self._filter_blacklisted(frontiers_world)
        if not candidates:
            self.get_logger().debug(
                'All known frontiers are on cooldown after recent failures, waiting.'
            )
            return

        target = select_target(candidates, robot_x, robot_y)

        if distance(robot_x, robot_y, target[0], target[1]) < MIN_TARGET_DISTANCE_M:
            self.get_logger().debug('Nearest frontier is too close, waiting for next map update.')
            return

        self.get_logger().info(
            f'{len(clusters)} frontier(s) found, targeting ({target[0]:.2f}, {target[1]:.2f})'
        )
        yaw = math.atan2(target[1] - robot_y, target[0] - robot_x)
        self._check_reachability(target[0], target[1], yaw)

    def _filter_blacklisted(self, frontiers_world):
        now = self.get_clock().now()
        self._blacklist = [b for b in self._blacklist if b[2] > now]

        def is_blacklisted(x, y):
            return any(
                distance(x, y, bx, by) < BLACKLIST_RADIUS_M
                for (bx, by, _exp) in self._blacklist
            )

        return [(x, y) for (x, y) in frontiers_world if not is_blacklisted(x, y)]

    def _check_stall(self):
        """Cancel the active goal if it stops making progress, or hits the absolute cap.

        See module docstring's "Stall watchdog" section — this exists so a single bad
        target can't leave exploration looking stuck for minutes while Nav2 works
        through its own recovery cycles on its own timeline. A goal whose
        distance_remaining keeps shrinking, however slowly, is never touched by the
        no-progress check — only GOAL_ABS_TIMEOUT_S could ever cancel it, and only if
        feedback stops arriving entirely.
        """
        if self._goal_start_time is None or self._cancel_requested:
            return
        now = self.get_clock().now()
        elapsed_total_s = (now - self._goal_start_time).nanoseconds / 1e9

        if self._goal_handle is None:
            # send_goal_async's done-callback hasn't landed yet. Normally near-
            # instant, but if the action server dies or is unresponsive mid-handshake
            # it may never land at all — there's no goal_handle to cancel in that
            # case, so the only way out is to give up on this attempt directly rather
            # than wait on a callback that may never fire. See module docstring.
            if elapsed_total_s < GOAL_ABS_TIMEOUT_S:
                return
            self.get_logger().warn(
                f'Goal to {self._current_target} was never accepted or rejected '
                f'within {GOAL_ABS_TIMEOUT_S:.0f}s; giving up on it directly.'
            )
            self._goal_epoch += 1  # invalidate a late-arriving response, see docstring
            if self._current_target is not None:
                x, y = self._current_target
                self._blacklist_target(x, y)
            self._current_target = None
            self._goal_start_time = None
            self._last_progress_distance = None
            self._last_progress_time = None
            self._state = State.IDLE
            return

        progress_since = self._last_progress_time or self._goal_start_time
        elapsed_no_progress_s = (now - progress_since).nanoseconds / 1e9

        stalled = elapsed_no_progress_s >= NO_PROGRESS_TIMEOUT_S
        timed_out = elapsed_total_s >= GOAL_ABS_TIMEOUT_S
        if not (stalled or timed_out):
            return
        reason = (
            f'no progress for {elapsed_no_progress_s:.0f}s' if stalled
            else f'absolute {GOAL_ABS_TIMEOUT_S:.0f}s cap reached with no feedback'
        )
        self.get_logger().warn(
            f'Goal to {self._current_target} stalled ({reason}); canceling instead of '
            f'waiting on Nav2 to give up.'
        )
        self._goal_handle.cancel_goal_async()
        self._cancel_requested = True

    def _check_path_stall(self):
        """Give up on a reachability check that never resolves at all.

        The compute_path_to_pose equivalent of _check_stall's unaccepted-goal branch
        — see module docstring. Reachability checks normally resolve in well under a
        second, so PATH_CHECK_TIMEOUT_S is a generous but much shorter backstop than
        GOAL_ABS_TIMEOUT_S.
        """
        if self._path_check_start_time is None:
            return
        elapsed_s = (self.get_clock().now() - self._path_check_start_time).nanoseconds / 1e9
        if elapsed_s < PATH_CHECK_TIMEOUT_S:
            return
        self.get_logger().warn(
            f'Reachability check for {self._current_target} did not resolve within '
            f'{PATH_CHECK_TIMEOUT_S:.0f}s; giving up on it directly.'
        )
        self._path_epoch += 1  # invalidate a late-arriving response, see docstring
        if self._current_target is not None:
            x, y = self._current_target
            self._blacklist_target(x, y)
        self._current_target = None
        self._pending_yaw = None
        self._path_check_start_time = None
        self._state = State.IDLE

    def _blacklist_target(self, x, y):
        for entry in self._failure_history:
            if distance(x, y, entry[0], entry[1]) < BLACKLIST_RADIUS_M:
                entry[2] += 1
                count = entry[2]
                break
        else:
            count = 1
            self._failure_history.append([x, y, count])

        cooldown = min(
            BLACKLIST_COOLDOWN_BASE_S * (2 ** (count - 1)), BLACKLIST_COOLDOWN_MAX_S
        )
        expiry = self.get_clock().now() + Duration(seconds=cooldown)
        self._blacklist.append((x, y, expiry))
        self.get_logger().info(
            f'Blacklisting ({x:.2f}, {y:.2f}) for {cooldown:.0f}s (failure #{count}).'
        )

    # ------------------------------------------------------------------ #
    # Robot pose via TF (map -> base_link)
    # ------------------------------------------------------------------ #

    def _get_robot_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                MAP_FRAME, ROBOT_FRAME, rclpy.time.Time(), Duration(seconds=0.5)
            )
        except TransformException as ex:
            msg = f'Could not get robot pose ({MAP_FRAME}->{ROBOT_FRAME}): {ex}'
            self.get_logger().warn(msg, throttle_duration_sec=5.0)
            return None
        return t.transform.translation.x, t.transform.translation.y

    # ------------------------------------------------------------------ #
    # Nav2 ComputePathToPose action client — reachability pre-check
    # ------------------------------------------------------------------ #

    def _check_reachability(self, x, y, yaw):
        if not self._path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('compute_path_to_pose action server not available yet.')
            return
        self.get_logger().info(f'Sending compute_path_to_pose check for ({x:.2f}, {y:.2f}).')

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = MAP_FRAME
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(yaw / 2.0)
        goal.use_start = False  # plan from the robot's current pose

        self._state = State.CHECKING_PATH
        self._current_target = (x, y)
        self._pending_yaw = yaw
        self._path_check_start_time = self.get_clock().now()
        self._path_epoch += 1
        epoch = self._path_epoch
        send_future = self._path_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future, epoch=epoch: self._path_goal_response_cb(future, epoch)
        )

    def _path_goal_response_cb(self, future, epoch):
        goal_handle = future.result()
        if epoch != self._path_epoch:
            # This attempt was already given up on by _check_path_stall and
            # superseded by a newer one — see module docstring. If it turns out to
            # have been accepted after all, cancel it rather than leave an orphan
            # planning request running.
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().warn(
                'compute_path_to_pose goal rejected; will retry from scratch next tick.'
            )
            self._current_target = None
            self._pending_yaw = None
            self._path_check_start_time = None
            self._state = State.IDLE
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, epoch=epoch: self._path_result_cb(future, epoch)
        )

    def _path_result_cb(self, future, epoch):
        if epoch != self._path_epoch:
            # Superseded by a newer attempt; _check_path_stall already gave up on
            # this one and moved on. See module docstring.
            return
        result = future.result()
        reachable = (
            result.status == GoalStatus.STATUS_SUCCEEDED
            and result.result.error_code == ComputePathToPose.Result.NONE
            and len(result.result.path.poses) > 0
        )
        x, y = self._current_target
        self._path_check_start_time = None
        if reachable:
            self.get_logger().info(
                f'Path to ({x:.2f}, {y:.2f}) confirmed '
                f'({len(result.result.path.poses)} waypoints); sending NavigateToPose.'
            )
        if not reachable:
            self.get_logger().info(
                f'Target ({x:.2f}, {y:.2f}) has no valid path '
                f'(error_code={result.result.error_code}); skipping without spending a '
                f'full Nav2 attempt on it.'
            )
            self._blacklist_target(x, y)
            self._current_target = None
            self._pending_yaw = None
            self._state = State.IDLE
            return

        yaw = self._pending_yaw
        self._pending_yaw = None
        self._send_goal(x, y, yaw)

    # ------------------------------------------------------------------ #
    # Nav2 NavigateToPose action client
    # ------------------------------------------------------------------ #

    def _send_goal(self, x, y, yaw):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('navigate_to_pose action server not available yet.')
            # Reset state — without this the node would be stuck in CHECKING_PATH
            # forever when called from _path_result_cb, since _evaluate() unconditionally
            # no-ops on that state and nothing else would ever pull it back out.
            self._current_target = None
            self._state = State.IDLE
            return
        self.get_logger().info(f'Sending NavigateToPose goal to ({x:.2f}, {y:.2f}).')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # Face the direction of approach (robot -> target) instead of a fixed heading,
        # so Nav2 doesn't have to finish with an unnecessary rotate-in-place right at
        # the frontier edge — the same maneuver that motivated the DWB rotate-in-place
        # fix elsewhere in this project (see PHASE_1_IMPLEMENTATION_PLAN.md).
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._state = State.NAVIGATING
        self._current_target = (x, y)
        self._goal_start_time = self.get_clock().now()
        self._cancel_requested = False
        self._last_progress_distance = None
        self._last_progress_time = None
        self._goal_epoch += 1
        epoch = self._goal_epoch
        send_future = self._nav_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(
            lambda future, epoch=epoch: self._goal_response_cb(future, epoch)
        )

    def _feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'distance_remaining={remaining:.2f}m', throttle_duration_sec=2.0)
        if (
            self._last_progress_distance is None
            or remaining < self._last_progress_distance - PROGRESS_EPSILON_M
        ):
            self._last_progress_distance = remaining
            self._last_progress_time = self.get_clock().now()

    def _goal_response_cb(self, future, epoch):
        goal_handle = future.result()
        if epoch != self._goal_epoch:
            # This attempt was already given up on by _check_stall (never accepted
            # within GOAL_ABS_TIMEOUT_S) and superseded by a newer one — see module
            # docstring. If it turns out to have been accepted after all, cancel the
            # orphan instead of leaving the robot navigating to a target this node no
            # longer tracks or can blacklist.
            if goal_handle.accepted:
                self.get_logger().warn(
                    'A stale Nav2 goal was accepted after this node gave up on it; '
                    'canceling the orphan.'
                )
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the frontier goal.')
            self._state = State.IDLE
            self._current_target = None
            self._goal_start_time = None
            self._last_progress_distance = None
            self._last_progress_time = None
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }.get(result.status, f'status={result.status}')
        self.get_logger().info(f'Nav2 goal finished: {status_name}')

        if result.status != GoalStatus.STATUS_SUCCEEDED and self._current_target is not None:
            x, y = self._current_target
            self._blacklist_target(x, y)

        self._current_target = None
        self._goal_handle = None
        self._goal_start_time = None
        self._cancel_requested = False
        self._last_progress_distance = None
        self._last_progress_time = None
        self._state = State.IDLE


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
