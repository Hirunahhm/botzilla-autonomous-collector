"""
collector_node.py — the collector robot's mission FSM: wait for a task, collect, deliver.

Runs on the second robot (the Raspberry Pi), under its namespace (/bz2). It does not
search. The leader explores, detects cubes and sends one task at a time on
fleet/task; this node drives to the cube, collects it with the leader-proven chain from
botzilla_navigation's executor_node, delivers it to HOME and reports the result on
fleet/status.

    STARTUP     latch HOME = map->base_link once AMCL is localised in the leader's map
    IDLE        wait for a CubeTask
    GOING       NavigateToPose to a standoff pose facing the cube (standoff.py)
    SEEKING     at the standoff: face the cube's estimate, then sweep slowly either side
    TARGETING   \
    APPROACHING  |  unchanged from executor_node (subclassed, not copied), including the
    CAPTURING    |  carrying behaviour tree, the delivery watchdog and the no-reverse
    DELIVERING   |  rule while carrying
    DETACHING   /
                -> IDLE, reporting COLLECTED (released at HOME) or FAILED (anything else)

Only detections that project within TASK_MATCH_RADIUS_M of the task's estimate start a
chase. Another cube on the way is the leader's to register and assign; chasing it would
leave the registry thinking the assigned cube is still out there.

Per PROJECT.md §6/§9 the leader never sends velocities over the network, only these
discrete goals; the collector plans and drives locally with its own Nav2 and LiDAR.
"""

from collections import deque
import math

from action_msgs.msg import GoalStatus
from botzilla_fleet.leader_state import MOVING, PARKED, STUCK
from botzilla_fleet.standoff import choose_standoff, fallback_standoff
from botzilla_interfaces.msg import CollectorStatus, CubeTask
from botzilla_navigation.executor_node import (
    ExecutorNode, HELD_CUBE_OFFSET_M, MAP_FRAME, State,
)
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

# A detection this close to the task's estimate is the assigned cube. The leader's
# estimate is a mean of projections made from up to 1 m away with a ~0.1 m range error
# and its own localisation error on top; 0.7 m is loose enough not to reject the real
# cube and tight enough not to grab a neighbour.
TASK_MATCH_RADIUS_M = 0.7
# GOING: generous; the route can cross the whole arena.
GOING_TIMEOUT_S = 180.0
# A failed route still ends the task in SEEKING if the robot got this close.
SEEK_ANYWAY_M = 1.3
# SEEKING. YOLO on the Pi's CPU runs at about 1 fps, so the robot stops before looking
# and turns slowly enough that every ~10 deg of heading gets at least one frame.
SEEK_FACE_TOL_RAD = 0.08
SEEK_SETTLE_S = 2.5
SEEK_SPEED = 0.18                    # rad/s
SEEK_SWEEP_RAD = math.radians(60)    # either side of the bearing to the estimate
SEEK_PAUSE_S = 1.5                   # at each end of the sweep
SEEK_TIMEOUT_S = 40.0
SEEK_KP = 1.2
# 4 Hz: the leader also draws this pose into its costmaps as an obstacle
# (fleet_manager_node COLLECTOR_SHAPE), so its age is the obstacle's lag.
STATUS_PERIOD_S = 0.25
# A route that fails this fast never left the collector: Nav2 rejected or aborted it at
# once (inactive, or still coming up). That is the collector's problem, not the cube's,
# so it is reported with a 'collector:' prefix the leader does not count against the
# cube, and the collector goes back to waiting for Nav2. In run_logs/20261005-184223
# Nav2 never came up and four real cubes were each "failed" twice in 0.1 s.
QUICK_FAIL_S = 2.0
# Published costmap scale: 99 = inscribed, 100 = lethal. Nav2 refuses goals there.
COSTMAP_GOAL_BLOCKED = 99
NAV_READY_POLL_S = 1.0


class Collector:
    IDLE = 'IDLE'
    GOING = 'GOING'
    SEEKING = 'SEEKING'
    YIELDING = 'YIELDING'


# Right of way: the LEADER has priority. In a small arena the two robots kept meeting and
# getting each other stuck (multi_robot_runs.md runs 7, 9, 12, 13). So when the leader
# comes within YIELD_TRIGGER_M (centre to centre) and the collector has no cube in hand,
# the collector pauses its task, moves YIELD_STEP_M away from the leader — Nav2's BackUp
# if the leader is ahead, DriveOnHeading if it is behind; both check the costmap for
# collisions — then holds still until the leader is YIELD_CLEAR_M away (or YIELD_MAX_S
# has passed) and resumes the same task from its standoff. Never while capturing,
# delivering or releasing: a cube in hand must not be dropped to make room.
# YIELD_COOLDOWN_S stops it bouncing back and forth if the leader lingers nearby.
# What the leader is doing comes from the fleet manager (fleet/leader_state, see
# leader_state.py): MOVING (driving or turning), STUCK (Nav2 recovering or not getting
# anywhere) or PARKED. A MOVING or STUCK leader is given way at YIELD_TRIGGER_M, a PARKED
# one only inside YIELD_CLOSE_M. The wait ends once the leader has been PARKED for
# YIELD_PARKED_S: the leader stops near cubes to inspect them, and waiting out a parked
# leader cost the fake-robot harness two thirds of its deliveries. Around a parked leader
# the collector simply plans past it (it is drawn as an obstacle in its costmap).
# A STUCK leader is never planned past. Guessing from position alone could not tell stuck
# from parked, so in run 16 (multi_robot_runs.md) the collector resumed "past" a leader
# that was stuck beside it, drove closer, and wedged both for the rest of the run. It now
# holds clear for up to YIELD_STUCK_MAX_S, giving the leader's recoveries room, then
# hands its task back to the leader ('collector:' detail, so the cube is not marked
# failed; the fleet manager re-offers it after its fault hold-off) rather than drive
# past. Once already outside YIELD_TRIGGER_M of the stuck leader it is out of its way, and
# a stuck leader never "passes", so it hands back after YIELD_STUCK_CLEAR_S instead: in
# the harness, waiting the full time at 1.1-1.3 m only cost 45 s per encounter.
# The cooldown does not apply inside YIELD_CLOSE_M to a MOVING or STUCK leader.
YIELD_CLOSE_M = 0.6
YIELD_PARKED_S = 4.0
YIELD_STUCK_MAX_S = 30.0
YIELD_STUCK_CLEAR_S = 5.0
LEADER_MOVING_M = 0.08             # moved this far in LEADER_MOVING_WINDOW_S = moving
LEADER_MOVING_WINDOW_S = 1.5
YIELD_TRIGGER_M = 0.9
YIELD_CLEAR_M = 1.3
YIELD_STEP_M = 0.3
YIELD_SPEED = 0.10                 # m/s
YIELD_MOVE_TIMEOUT_S = 12.0
YIELD_MAX_S = 25.0
YIELD_COOLDOWN_S = 10.0
LEADER_POSE_MAX_AGE_S = 1.0
YIELDABLE = (Collector.IDLE, Collector.GOING, Collector.SEEKING,
             State.TARGETING, State.APPROACHING)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class CollectorNode(ExecutorNode):
    def __init__(self):
        super().__init__('collector_node')
        self.declare_parameter('robot_name', self.get_namespace().strip('/') or 'collector')
        self._robot_name = self.get_parameter('robot_name').value

        self._task = None             # (task_id, x, y) in progress
        self._queued_task = None      # a task that arrived while busy
        self._task_delivered_before = 0
        self._last_task_id = 0
        self._last_result = CollectorStatus.RESULT_NONE
        self._last_detail = ''
        self._seek = None             # SEEKING sub-state, see _do_seeking
        self._release_xy = None       # cube left short of HOME on this task, if any
        self._last_release = None     # ... and reported for last_task_id

        self._map = None              # (data, (ox, oy, res, w, h))
        # IDLE (= ready for tasks) also requires Nav2 to be fully active, not just a pose:
        # AMCL publishes map->odom before the rest of the stack is up, and once the
        # lifecycle manager has aborted a bring-up it never will be.
        self._nav_ready = False
        self._nav_ready_future = None
        self._nav_ready_polled = None
        self._is_active_client = self.create_client(
            Trigger, 'lifecycle_manager_navigation/is_active')
        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        # The leader's map (absolute on purpose), for choosing a clear standoff pose.
        # The fleet manager's copy of the leader's map with this robot's own footprint and
        # HOME cleared (see fleet_manager_node COLLECTOR_MAP_CLEAR_M); the same map the
        # costmaps and AMCL use.
        self.create_subscription(OccupancyGrid, 'fleet/map', self._map_cb, latched)
        # This robot's live global costmap: the leader, cubes, inflation and its own
        # LiDAR, which the static map does not have — used to reject standoff poses
        # Nav2 would refuse (see _costmap_blocked).
        self._costmap = None
        self.create_subscription(
            OccupancyGrid, 'global_costmap/costmap', self._costmap_cb, latched)
        self.create_subscription(CubeTask, 'fleet/task', self._task_cb, latched)
        # Leader priority — see YIELD_TRIGGER_M.
        self._leader_pose = None          # (x, y)
        self._leader_pose_time = None
        self._leader_hist = deque()       # (time_s, x, y), last few seconds
        self._leader_state = None         # MOVING / PARKED / STUCK from the fleet manager
        self._leader_state_time = None
        self._yield = None                # in-progress yield, see _start_yield
        self._last_yield_end = None
        self.create_subscription(PoseStamped, 'fleet/leader_pose', self._leader_cb, 10)
        self.create_subscription(String, 'fleet/leader_state', self._leader_state_cb, 10)
        self._backup_client = ActionClient(self, BackUp, 'backup')
        self._drive_client = ActionClient(self, DriveOnHeading, 'drive_on_heading')
        self._fleet_status_pub = self.create_publisher(CollectorStatus, 'fleet/status', 10)
        self.create_timer(STATUS_PERIOD_S, self._publish_fleet_status)
        self.get_logger().info(
            f'collector_node ({self._robot_name}) started — waiting for AMCL to localise '
            f'in the leader map before latching HOME.'
        )

    # ------------------------------------------------------------------ #
    # Inputs
    # ------------------------------------------------------------------ #

    def _costmap_cb(self, msg):
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        self._costmap = (data, info.origin.position.x, info.origin.position.y,
                         info.resolution)

    def _costmap_blocked(self, x, y):
        """Return True if Nav2 would refuse a goal at (x, y) (inscribed or lethal)."""
        if self._costmap is None:
            return False
        data, ox, oy, res = self._costmap
        i, j = int((x - ox) / res), int((y - oy) / res)
        if not (0 <= j < data.shape[0] and 0 <= i < data.shape[1]):
            return False
        return data[j, i] >= COSTMAP_GOAL_BLOCKED

    def _map_cb(self, msg):
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        self._map = (data, (info.origin.position.x, info.origin.position.y,
                            info.resolution, info.width, info.height))

    def _task_cb(self, msg):
        if msg.task_id == 0:
            return
        if self._task is not None and msg.task_id == self._task[0]:
            return
        if msg.task_id == self._last_task_id:
            return
        task = (msg.task_id, msg.target.x, msg.target.y)
        if self._state == Collector.IDLE:
            self._start_task(task)
        else:
            self._queued_task = task

    def _leader_cb(self, msg):
        self._leader_pose = (msg.pose.position.x, msg.pose.position.y)
        self._leader_pose_time = self.get_clock().now()
        t = self._leader_pose_time.nanoseconds / 1e9
        self._leader_hist.append((t, *self._leader_pose))
        while self._leader_hist and t - self._leader_hist[0][0] > 3 * LEADER_MOVING_WINDOW_S:
            self._leader_hist.popleft()

    def _leader_state_cb(self, msg):
        self._leader_state = msg.data
        self._leader_state_time = self.get_clock().now()

    def _leader_status(self, now):
        """Return MOVING / PARKED / STUCK: the fleet manager's word, else guessed from motion."""
        if (self._leader_state_time is not None and self._leader_state in (MOVING, PARKED, STUCK)
                and (now - self._leader_state_time).nanoseconds / 1e9 <= LEADER_POSE_MAX_AGE_S):
            return self._leader_state
        # A fleet manager without leader_state (older build): position only.
        return MOVING if self._leader_moving(now) else PARKED

    def _leader_moving(self, now):
        """Return True if the leader moved LEADER_MOVING_M within the last window."""
        t = now.nanoseconds / 1e9
        recent = [(x, y) for ts, x, y in self._leader_hist if t - ts <= LEADER_MOVING_WINDOW_S]
        if len(recent) < 2:
            return False
        return math.hypot(recent[-1][0] - recent[0][0],
                          recent[-1][1] - recent[0][1]) > LEADER_MOVING_M

    def _cube_cb(self, msg):
        if self._state in (State.STARTUP, Collector.IDLE, Collector.YIELDING):
            return
        if self._state in (Collector.GOING, Collector.SEEKING):
            self._maybe_start_chase(msg)
            return
        super()._cube_cb(msg)

    def _maybe_start_chase(self, msg):
        if msg.z <= 0.0 or msg.z > self._cube_max_range_m or self._task is None:
            return
        spot = self._detection_position(msg)
        if spot is None:
            return
        off = math.hypot(spot[0] - self._task[1], spot[1] - self._task[2])
        if off > TASK_MATCH_RADIUS_M:
            self.get_logger().info(
                f'Cube seen at ({spot[0]:.2f}, {spot[1]:.2f}), {off:.2f} m from task '
                f'{self._task[0]}; not the assigned cube, ignoring.',
                throttle_duration_sec=5.0,
            )
            return
        if self._home is not None and math.hypot(
                spot[0] - self._home[0], spot[1] - self._home[1]
        ) < self._home_cube_suppress_radius_m:
            return
        self._cancel_nav_goal()
        self._target_cube = msg
        self._cube_last_seen = self.get_clock().now()
        self._update_cube_world_estimate(msg)
        self._transition(
            State.TARGETING,
            f'Task {self._task[0]} cube seen at {msg.z:.2f} m ({off:.2f} m from estimate).'
        )

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #

    def _control_loop(self):
        now = self.get_clock().now()
        if self._state == Collector.YIELDING:
            self._do_yielding(now)
            self._publish_status()
            return
        if self._state in YIELDABLE and self._should_yield(now):
            self._start_yield(now)
            self._publish_status()
            return
        if self._state == Collector.IDLE:
            if self._queued_task is not None:
                task, self._queued_task = self._queued_task, None
                self._start_task(task)
            else:
                self._cmd_pub.publish(Twist())   # nobody else drives while idle
            self._publish_status()
            return
        if self._state == Collector.GOING:
            self._do_going(now)
            self._publish_status()
            return
        if self._state == Collector.SEEKING:
            cmd = Twist()
            self._do_seeking(cmd, now)
            if self._state == Collector.SEEKING:
                self._drive(cmd)
            self._publish_status()
            return
        super()._control_loop()

    # ------------------------------------------------------------------ #
    # Leader priority (see YIELD_TRIGGER_M)
    # ------------------------------------------------------------------ #

    def _leader_distance(self, now):
        """(distance, bearing in this robot's frame) to the leader, or None if unknown."""
        if self._leader_pose is None or self._leader_pose_time is None:
            return None
        if (now - self._leader_pose_time).nanoseconds / 1e9 > LEADER_POSE_MAX_AGE_S:
            return None
        pose = self._get_robot_pose()
        if pose is None:
            return None
        dx, dy = self._leader_pose[0] - pose[0], self._leader_pose[1] - pose[1]
        return math.hypot(dx, dy), _wrap(math.atan2(dy, dx) - pose[2])

    def _should_yield(self, now):
        d = self._leader_distance(now)
        if d is None:
            return False
        active = self._leader_status(now) != PARKED
        cooling = self._last_yield_end is not None and (
            now - self._last_yield_end).nanoseconds / 1e9 < YIELD_COOLDOWN_S
        if cooling and not (active and d[0] < YIELD_CLOSE_M):
            return False
        return d[0] < YIELD_CLOSE_M or (d[0] < YIELD_TRIGGER_M and active)

    def _start_yield(self, now):
        dist, bearing = self._leader_distance(now)
        was = self._state
        if was == Collector.GOING:
            self._cancel_nav_goal()
        # Any chase in progress is dropped; the task resumes from its standoff.
        self._target_cube = None
        self._cube_world_estimate = None
        self._blind_spot_frames = 0
        ahead = abs(bearing) < math.pi / 2
        client, action, kind = ((self._backup_client, BackUp, 'backing up') if ahead
                                else (self._drive_client, DriveOnHeading, 'driving forward'))
        self._yield = {'start': now, 'phase': 'moving', 'resume': self._task is not None,
                       'parked_since': None}
        self.get_logger().info(
            f'Leader {dist:.2f} m away ({math.degrees(bearing):+.0f} deg, '
            f'{self._leader_status(now)}) during {was}: '
            f'yielding — {kind} {YIELD_STEP_M} m, then waiting for it to pass.')
        self._transition(Collector.YIELDING, f'Leader priority ({dist:.2f} m).')
        if not client.wait_for_server(timeout_sec=1.0):
            self._yield['phase'] = 'holding'
            return
        goal = action.Goal()
        goal.target.x = YIELD_STEP_M
        goal.speed = YIELD_SPEED
        goal.time_allowance = Duration(seconds=YIELD_MOVE_TIMEOUT_S).to_msg()
        yield_state = self._yield

        def on_done(future):
            if self._yield is yield_state:
                yield_state['phase'] = 'holding'

        def on_accept(future):
            handle = future.result()
            if not handle.accepted:
                on_done(None)
                return
            yield_state['handle'] = handle
            handle.get_result_async().add_done_callback(on_done)
        client.send_goal_async(goal).add_done_callback(on_accept)

    def _do_yielding(self, now):
        y = self._yield
        waited = (now - y['start']).nanoseconds / 1e9
        if y['phase'] == 'moving' and waited < YIELD_MOVE_TIMEOUT_S + 1.0:
            return   # the BackUp/DriveOnHeading action drives
        d = self._leader_distance(now)
        clear = d is None or d[0] > YIELD_CLEAR_M
        status = self._leader_status(now)
        if status != PARKED:
            y['parked_since'] = None
        elif y['parked_since'] is None:
            y['parked_since'] = now
        parked = (y['parked_since'] is not None
                  and (now - y['parked_since']).nanoseconds / 1e9 > YIELD_PARKED_S)
        stuck = not clear and status == STUCK
        limit = YIELD_MAX_S
        if stuck:
            limit = YIELD_STUCK_CLEAR_S if d[0] >= YIELD_TRIGGER_M else YIELD_STUCK_MAX_S
        if not clear and not parked and waited < limit:
            self.get_logger().info(
                f'Yielding: leader {d[0]:.2f} m away ({status}), waiting ({waited:.0f}s).',
                throttle_duration_sec=5.0)
            return
        if y.get('handle') is not None and y['phase'] == 'moving':
            y['handle'].cancel_goal_async()
        self._yield = None
        self._last_yield_end = now
        if stuck:
            # Never plan past a stuck leader (see YIELD_STUCK_MAX_S): give the task back.
            if y['resume'] and self._task is not None:
                self._finish_task(False, f'collector: gave way to a stuck leader '
                                         f'({d[0]:.2f} m) for {waited:.0f}s')
            else:
                self._transition(Collector.IDLE, 'Yield over (leader still stuck).')
            return
        reason = ('leader clear' if clear else 'leader parked; planning past it' if parked
                  else f'gave way for {YIELD_MAX_S:.0f}s')
        if y['resume'] and self._task is not None:
            self.get_logger().info(f'Yield over ({reason}); resuming task {self._task[0]}.')
            self._start_task(self._task)
            if self._state == Collector.YIELDING:
                # Could not restart yet (no pose): it is queued and starts from IDLE.
                self._transition(Collector.IDLE, 'Yield over; task queued.')
        else:
            self._transition(Collector.IDLE, f'Yield over ({reason}).')

    def _do_startup(self, now):
        if self._home is None:
            pose = self._get_robot_pose()
            if pose is None:
                self.get_logger().info(
                    f'Waiting for {MAP_FRAME}->base_link (AMCL in the leader map)...',
                    throttle_duration_sec=5.0,
                )
                return
            self._home = pose
            self.get_logger().info(
                f'HOME latched at x={pose[0]:.3f} y={pose[1]:.3f} '
                f'yaw={math.degrees(pose[2]):.1f}deg in the leader map.'
            )
        if self._map is None:
            # Not ready until the fleet manager's cleaned map has arrived: before it,
            # AMCL and the costmaps run without the leader's map and every plan fails
            # (the collector came up 38 s before the fleet manager on 2026-10-05).
            self.get_logger().info('Waiting for the fleet map (fleet/map)...',
                                   throttle_duration_sec=5.0)
            return
        if not self._poll_nav_ready(now):
            self.get_logger().info('Waiting for Nav2 to be fully active...',
                                   throttle_duration_sec=5.0)
            return
        self._transition(Collector.IDLE, 'HOME latched and Nav2 active; waiting for tasks.')

    def _poll_nav_ready(self, now):
        """Ask the lifecycle manager whether every Nav2 node is active (non-blocking)."""
        if self._nav_ready:
            return True
        fut = self._nav_ready_future
        if fut is not None and fut.done():
            result = fut.result()
            self._nav_ready = bool(result is not None and result.success)
            self._nav_ready_future = None
            return self._nav_ready
        if fut is None and self._is_active_client.service_is_ready() and (
                self._nav_ready_polled is None
                or (now - self._nav_ready_polled).nanoseconds / 1e9 > NAV_READY_POLL_S):
            self._nav_ready_polled = now
            self._nav_ready_future = self._is_active_client.call_async(Trigger.Request())
        return False

    # ------------------------------------------------------------------ #
    # Task states
    # ------------------------------------------------------------------ #

    def _start_task(self, task):
        task_id, cx, cy = task
        pose = self._get_robot_pose()
        if pose is None:
            self._queued_task = task
            return
        self._task = task
        self._task_delivered_before = self._cubes_delivered
        standoff = None
        if self._map is not None:
            standoff = choose_standoff(self._map[0], self._map[1], (cx, cy), pose[:2],
                                       blocked=self._costmap_blocked)
        how = 'clear standoff from the leader map'
        if standoff is None:
            standoff = fallback_standoff((cx, cy), pose[:2])
            how = 'no clear standoff found; straight-line fallback'
        sx, sy, syaw = standoff
        self.get_logger().info(
            f'Task {task_id}: cube at ({cx:.2f}, {cy:.2f}); going to ({sx:.2f}, {sy:.2f}) '
            f'facing {math.degrees(syaw):.0f}deg ({how}).'
        )
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self._finish_task(False, 'navigate_to_pose unavailable')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = sx
        goal.pose.pose.position.y = sy
        goal.pose.pose.orientation.z = math.sin(syaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(syaw / 2.0)
        self._send_nav_goal(goal)
        self._transition(Collector.GOING, f'Task {task_id} accepted.')

    def _do_going(self, now):
        if self._nav_result is not None:
            status, self._nav_result = self._nav_result, None
            self._nav_goal_handle = None
            took = ((now - self._nav_sent_time).nanoseconds / 1e9
                    if self._nav_sent_time is not None else 0.0)
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._enter_seeking('At the standoff.')
            elif self._nav_rejected:
                # Nav2 refused the goal itself: it is not up. Accepted-then-failed is a
                # real "no route" and is reported normally below — counting those as
                # collector faults returned an unreachable cube to the pool forever.
                self._finish_task(
                    False, f'collector: navigation not ready (goal rejected after '
                           f'{took:.1f}s)')
                self._nav_ready = False
                self._transition(State.STARTUP, 'Nav2 rejected the route; re-checking it.')
            elif self._dist_to_task() < SEEK_ANYWAY_M:
                self._enter_seeking(f'Route ended with status {status}, but close enough.')
            else:
                self._finish_task(False, f'route to the standoff failed (status {status})')
            return
        if self._nav_sent_time is None:
            return
        if (now - self._nav_sent_time).nanoseconds / 1e9 > GOING_TIMEOUT_S:
            self._cancel_nav_goal()
            self._finish_task(False, f'standoff not reached in {GOING_TIMEOUT_S:.0f}s')

    def _enter_seeking(self, reason):
        self._seek = {'phase': 'face', 'start': self.get_clock().now(),
                      'phase_start': self.get_clock().now(), 'leg': 0}
        self._transition(Collector.SEEKING, reason)

    def _do_seeking(self, cmd, now):
        """Face the estimate, wait for YOLO, then sweep +-SEEK_SWEEP_RAD and back.

        A detection of the assigned cube (via _cube_cb) moves on to TARGETING from any
        phase; reaching the end of the sweep or SEEK_TIMEOUT_S fails the task.
        """
        s = self._seek
        if (now - s['start']).nanoseconds / 1e9 > SEEK_TIMEOUT_S:
            self._finish_task(False, 'cube not seen from the standoff')
            return
        pose = self._get_robot_pose()
        if pose is None:
            return
        bearing = math.atan2(self._task[2] - pose[1], self._task[1] - pose[0])
        in_phase = (now - s['phase_start']).nanoseconds / 1e9
        targets = [bearing, bearing + SEEK_SWEEP_RAD, bearing - SEEK_SWEEP_RAD, bearing]

        def next_phase(phase):
            s['phase'] = phase
            s['phase_start'] = now

        if s['phase'] in ('face', 'turn'):
            err = _wrap(targets[s['leg']] - pose[2])
            if abs(err) > SEEK_FACE_TOL_RAD:
                cmd.angular.z = max(-SEEK_SPEED, min(SEEK_SPEED, SEEK_KP * err))
            else:
                next_phase('settle')
        elif s['phase'] == 'settle':
            wait = SEEK_SETTLE_S if s['leg'] == 0 else SEEK_PAUSE_S
            if in_phase >= wait:
                s['leg'] += 1
                if s['leg'] >= len(targets):
                    self._finish_task(False, 'cube not seen from the standoff')
                    return
                next_phase('turn')

    # ------------------------------------------------------------------ #
    # Results
    # ------------------------------------------------------------------ #

    def _resume_exploring(self, reason=''):
        """Every end of a chase or delivery in ExecutorNode lands here."""
        collected = self._cubes_delivered > self._task_delivered_before
        self._target_cube = None
        self._cube_world_estimate = None
        self._cube_lost_time = None
        self._blind_spot_frames = 0
        if collected:
            detail = ''
        elif self._state == State.DETACHING:
            # ExecutorNode passes 'Delivery complete.' for every release, including one
            # short of HOME; say what actually happened.
            detail = 'released short of HOME (delivery route failed)'
        else:
            detail = reason
        self._finish_task(collected, detail)

    def _finish_task(self, collected, detail):
        task_id = self._task[0] if self._task else 0
        self._last_task_id = task_id
        self._last_result = (CollectorStatus.RESULT_COLLECTED if collected
                             else CollectorStatus.RESULT_FAILED)
        self._last_detail = detail
        self._last_release = None if collected else self._release_xy
        self._release_xy = None
        self._task = None
        self._seek = None
        verdict = 'COLLECTED' if collected else f'FAILED ({detail})'
        self.get_logger().info(f'Task {task_id} {verdict}.')
        self._transition(Collector.IDLE, f'Task {task_id} done.')
        self._publish_fleet_status()

    # ------------------------------------------------------------------ #
    # Helpers / overrides
    # ------------------------------------------------------------------ #

    def _publish_exploration_enabled(self, enabled):
        pass   # no frontier explorer on the collector

    def _transition(self, new_state, reason=''):
        if new_state == State.DETACHING and not self._delivery_arrived:
            # Released short of HOME: remember where the cube is left (between the arms,
            # HELD_CUBE_OFFSET_M ahead), so the leader's registry can move it there.
            pose = self._get_robot_pose()
            if pose is not None:
                self._release_xy = (pose[0] + HELD_CUBE_OFFSET_M * math.cos(pose[2]),
                                    pose[1] + HELD_CUBE_OFFSET_M * math.sin(pose[2]))
        super()._transition(new_state, reason)
        if new_state in (Collector.GOING, Collector.YIELDING):
            # Nav2 drives in GOING (and its BackUp/DriveOnHeading while YIELDING, which
            # also go through the smoother); the base class only knows
            # EXPLORING/DELIVERING.
            self._smoother_enable_pub.publish(Bool(data=True))

    def _dist_to_task(self):
        pose = self._get_robot_pose()
        if pose is None or self._task is None:
            return float('inf')
        return math.hypot(self._task[1] - pose[0], self._task[2] - pose[1])

    def _publish_fleet_status(self):
        msg = CollectorStatus()
        msg.robot = self._robot_name
        msg.state = self._state
        msg.task_id = self._task[0] if self._task else 0
        msg.last_task_id = self._last_task_id
        msg.last_result = self._last_result
        msg.last_detail = self._last_detail
        pose = self._get_robot_pose()
        msg.localised = pose is not None
        if pose is not None:
            msg.x, msg.y, msg.yaw = pose
        msg.home_set = self._home is not None
        if self._home is not None:
            msg.home_x, msg.home_y = self._home[0], self._home[1]
        msg.delivered = self._cubes_delivered
        if self._last_release is not None:
            msg.last_released = True
            msg.release_x, msg.release_y = self._last_release
        self._fleet_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
