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

Costmap snapping: a raw frontier cell is, by definition, adjacent to unknown space, and
unknown space in a bounded arena is almost always adjacent to a wall just beyond sensor
range — so most raw frontier cells fall inside that wall's inflation halo: free (cost 0)
in the raw SLAM map, but LETHAL_OBSTACLE/INSCRIBED_INFLATED_OBSTACLE (cost 96-100) in the
actual /global_costmap/costmap the planner uses. Confirmed live: in a compact arena this
made ComputePathToPose reject the majority of raw frontier targets with NO_VALID_PATH,
leaving exploration looking "stuck" between the rare targets far enough from a wall to
succeed. Before every reachability check, the frontier point is snapped to the nearest
costmap cell below COSTMAP_SAFE_COST (see _snap_to_reachable/frontier_detection.
find_low_cost_point) and that snapped point — not the raw frontier — is what's sent to
Nav2. Blacklisting still keys off the raw frontier point (see _frontier_origin), so a
cluster whose every nearby cell is unreachable gets banned as a whole rather than
re-tried against a slightly different snapped point next tick.

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

Stuck recovery: costmap snapping (above) fixes goals that individually land in wall
inflation, but a robot wedged into a tight nook can end up with EVERY known frontier
genuinely unreachable — no route exists in the known map from the robot's current pose to
any of them, not because a goal cell is bad but because the immediate surroundings pinch
the corridor shut. Confirmed live: a flood-fill from the robot's position through every
sub-inscribed-cost cell in the global costmap did not reach either remaining frontier,
while the passage the robot could still take (behind it) was outside the frontiers'
direction entirely. Waiting out BLACKLIST_COOLDOWN_BASE_S changes nothing here since the
robot hasn't moved — the geometry that caused the rejection is still exactly the same.
When every known frontier stays blacklisted for STUCK_RECOVERY_TIMEOUT_S straight, this
node sends a BackUp goal (Nav2's own recovery action, already used internally by
bt_navigator) to reverse BACKUP_DISTANCE_M, then clears the blacklist entirely so every
frontier gets a fresh reachability check from the new pose — backing up is specifically
meant to change the geometry that made them fail, so there's no reason to keep them
banned. failure_history (the exponential-backoff counters) is deliberately NOT cleared,
so a frontier that's genuinely unreachable rather than just occluded by the robot's own
position still gets pushed out further on repeat offenses.

Self-clearance: the stuck-recovery trigger above reacts to "no known frontier is
reachable," which is a SYMPTOM. NavFn plans FROM the robot's current pose, so a robot
that has parked itself in collision cannot produce a valid path to anywhere, and every
frontier gets rejected in turn for a reason that has nothing to do with the frontiers.
_evaluate() therefore tests the robot's own pose first, via
frontier_detection.in_collision, and backs up immediately rather than discovering the
same thing the slow way through a sequence of doomed reachability checks.

That test is a single lookup of the robot's centre cell against the inscribed cutoff —
Nav2's own criterion for a circular footprint. It began life as a scan of every cell
within robot_radius for cost >= 99, which was wrong: cost 99 already means "a lethal
obstacle is within the inscribed radius of THIS cell," so re-scanning a robot_radius
neighbourhood for it demands robot_radius + inscribed_radius (0.40 m under this
project's tuning) of clearance where the robot actually needs 0.20 m. Confirmed live on
hardware: the robot sat in open floor, centre cell cost 0, zero lethal cells anywhere
inside its real footprint, and the check still declared it blocked — producing an endless
backup/spin loop in space it could drive through freely. The lesson generalises: the
inflation layer has already done the footprint reasoning, so re-applying the footprint on
top of an inflated costmap double-counts it.

Spin fallback: BackUp can itself fail this exact same way. Confirmed live: the goal
heading a frontier approach ends on points toward the frontier, not away from whatever
wall the approach passed close to, so the wall self-clearance detects can sit
beside/behind the robot's heading rather than in front of it — meaning a straight
reverse drives directly into the very obstacle it's trying to escape (Nav2's BackUp
correctly detects this itself and aborts with "Collision Ahead"). A rotation only needs
radial clearance around the robot, not linear clearance in one specific direction, so it
can succeed exactly where a straight reverse can't — matching what driving the robot
manually confirmed live (turning was possible from spots straight reversal wasn't).
_backup_result_cb chains into a Spin goal whenever BackUp itself doesn't SUCCEED, turning
RECOVERY_SPIN_ANGLE_RAD (always the same direction, see the constant's own comment for
why alternating direction was tried first and abandoned) before ending the recovery
cycle either way — if the robot is still footprint-blocked afterward, the next tick's
self-clearance check starts another backup/spin cycle, sweeping a new quadrant each time
rather than re-trying the same one.

Coverage sweep: frontier exploration alone only ever drives to the *edges* of known
space — once LiDAR has seen a room's walls, no frontier remains there even though the
Kinect's narrow (~57 deg) FOV never got a camera view of the room's interior, so cubes
away from the walls would never come within range of a camera-based detector. This node
switches self._mode from 'FRONTIER' to 'SWEEPING', builds a boustrophedon waypoint queue
via coverage_planning.generate_coverage_waypoints, and starts dispatching it
(_dispatch_next_sweep_waypoint) one point at a time. Deliberately reuses every safety
primitive already hardened above for frontier targets — costmap-snapping
(_snap_to_reachable), the reachability pre-check, the stall watchdog, and the
self-clearance/BackUp/Spin recovery chain — rather than duplicating any of it: a sweep
waypoint is, from Nav2's perspective, exactly the same kind of goal a frontier target is.
Sweep waypoints set self._frontier_origin = None, which every existing blacklist call
site already guards on — so a failed sweep waypoint is simply skipped (it was already
popped) rather than entered into the cooldown/backoff bookkeeping meant for frontier
clusters that get re-evaluated every tick; a one-pass queue has no "next tick" to
re-blacklist against. A waypoint that fails reachability is still marked swept in
self._swept_mask (mark_world_point_swept) before being skipped — the camera never
actually covered that spot, but leaving an un-drivable pocket permanently un-swept would
block mission completion forever over ground the robot genuinely cannot reach.

Interleaved, not sequential: the sweep is no longer a one-shot phase entered only once
exploration is exhausted. Every tick, self._swept_mask (swept_mask.py) tracks which
cells the camera's forward wedge (+/- half its FOV, out to its ~1m reliable detection
range) has actually passed over, and swept_mask.should_trigger_sweep fires SWEEPING as
soon as un-swept known-free area exceeds SWEEP_FRACTION of total known free area —
deliberately a fraction, not a fixed square-meterage, so a small first room gets
camera-checked almost immediately while a large already-mostly-swept arena isn't
re-triggered by trivial new patches. This means a sweep can begin while frontiers still
exist; exploration resumes afterward via the same mechanism, not by waiting for
exploration to finish first.

The sweep is still entered from a SECOND condition, kept from the original design: "no
reachable frontier candidate" — either frontiers_world is genuinely empty, or every
candidate is currently blacklisted. Ghost frontiers just outside the arena walls never
truly reach zero in a bounded arena, they just accumulate permanent blacklist entries
(confirmed live in sim: frontier counts plateau in exactly that state) — so without the
blacklist-escalation path (STUCK_RECOVERIES_BEFORE_SWEEP consecutive stuck-recoveries,
see _handle_all_frontiers_blacklisted), a run where the dynamic trigger never quite
crosses SWEEP_FRACTION (because the small residual un-swept area sits behind those same
ghost frontiers) would back up forever and never sweep the last few percent. This path
is kept as a secondary safety net specifically for that case, not removed now that the
dynamic trigger handles the common case.

Mission completion (_publish_coverage_complete, State.COVERAGE_COMPLETE) requires BOTH
"no reachable frontier candidate" AND "swept_mask.count_unswept_free reports 0" — never
just the swept count alone. COVERAGE_COMPLETE is a genuine dead end in this node
(_evaluate returns unconditionally once in it, and _enabled_cb's reset explicitly leaves
it alone across an executor pause/resume cycle), and the dynamic trigger is tuned to
fire on a SMALL known-free area on purpose — checking only "is what we've swept so far
100%" would let the very first, tiny sweep at mission start satisfy it while the rest of
the arena is still completely unexplored.

Returning from SWEEPING to FRONTIER — whether because the queue drained with un-swept
area still remaining (new floor appeared passively mid-sweep) or because a new pocket
was revealed (see below) — sets self._sweep_cooldown_until (SWEEP_RETRIGGER_COOLDOWN_S
out), during which the dynamic trigger is not re-checked. Without it, the same ratio
that just caused a return to FRONTIER would very likely still be true on the very next
tick, re-triggering SWEEPING before a single frontier goal ever gets a chance to
dispatch — the node would appear to explore but never actually move toward a frontier.

The mode is not one-way, and the abort condition for "new pocket revealed mid-sweep" is
count-based, not presence-based. find_frontiers runs every tick in BOTH modes, because
sweeping drives through the room interior — precisely where pockets that were occluded
from the walls come into view. Under the ORIGINAL one-shot design, sweeping only ever
began once frontiers_world was already empty, so any non-empty frontiers_world during a
sweep was unambiguously new. That assumption no longer holds once the dynamic trigger
can start a sweep WHILE frontiers still exist (the whole point of interleaving) — a
presence check would abort the sweep after its very first waypoint, every time. Instead,
self._sweep_start_frontier_count snapshots len(frontiers_world) when a sweep begins, and
the sweep is only aborted-and-replanned if the live count later exceeds that snapshot —
i.e. a cluster appeared that wasn't there when this sweep started. Resuming the old
queue instead of rebuilding would be cheaper but would leave the newly-discovered floor
uncovered, which is the exact failure this phase exists to prevent.

Snap/self-clearance agreement: COSTMAP_SAFE_COST (50) sits well below the inscribed
cutoff SELF_CLEARANCE_MAX_COST (99), so any cell _snap_to_reachable accepts is by
construction not a colliding pose. The snap and the arrival check therefore agree, and a
goal this node picks cannot be one the robot declares itself stuck at on arrival.

Usage:
  ros2 launch botzilla_navigation frontier_explorer.launch.py
"""

import math

from action_msgs.msg import GoalStatus
from botzilla_navigation.coverage_planning import generate_coverage_waypoints
from botzilla_navigation.frontier_detection import (
    distance,
    find_frontiers,
    find_low_cost_point,
    grid_to_world,
    in_collision,
    select_target,
    world_to_grid,
)
from botzilla_navigation.swept_mask import (
    count_unswept_free,
    create_swept_mask,
    mark_swept_cells,
    mark_world_point_swept,
    resize_swept_mask,
    should_trigger_sweep,
)
from nav2_msgs.action import BackUp, ComputePathToPose, NavigateToPose, Spin
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

# Stuck recovery — see module docstring. If every known frontier stays blacklisted (no
# candidate at all, as opposed to a candidate that's merely still being evaluated) for
# this long, the robot backs up rather than waiting out the cooldown for geometry that
# backing up itself is meant to fix.
STUCK_RECOVERY_TIMEOUT_S = 15.0

# How many back-to-back stuck-recoveries to attempt before concluding the remaining
# frontiers are genuinely unreachable rather than merely awkward, and handing over to the
# coverage sweep. Recovery changes the robot's pose, so if this many cycles have not made
# a single frontier reachable again, the obstruction is not the robot's position.
# Confirmed live in sim: frontier counts plateau with every survivor permanently
# blacklisted (they face unreachable space beyond the arena walls), which the
# "no frontiers at all" branch never observes — so without this bound the node backs up
# indefinitely and the sweep is unreachable in any bounded arena.
STUCK_RECOVERIES_BEFORE_SWEEP = 3
BACKUP_DISTANCE_M = 0.3
BACKUP_SPEED_MPS = 0.1
BACKUP_TIME_ALLOWANCE_S = 10.0

# Fallback when BackUp itself aborts (Nav2's own "Collision Ahead" — the local costmap
# sees an obstacle in the reverse direction too). Confirmed live: a wall detected via
# self-clearance can sit BESIDE/BEHIND the robot's current heading rather than in front
# of it, since the goal-approach heading points toward the frontier, not away from
# whatever wall the approach happened to pass close to — so straight backward travel
# drives directly into the same obstacle self-clearance just flagged. A rotation needs
# only radial clearance around the robot, not linear clearance in one specific
# direction, so it can succeed in exactly the cases a straight reverse can't.
#
# Always the SAME direction (left — matches empirically confirmed clearance in this
# arena), never alternated: target_yaw is RELATIVE to the robot's current heading, so
# alternating +/-90 each attempt ping-pongs between only TWO absolute headings forever
# (theta, theta+90, theta, theta+90, ...) — confirmed live, this left the robot cycling
# in place at a genuine corner without ever trying the other two quadrants. A fixed
# +90 every time instead sweeps theta, theta+90, theta+180, theta+270 in turn — a full
# rotation across four recovery cycles worst case — before any heading repeats.
RECOVERY_SPIN_ANGLE_RAD = math.pi / 2.0
SPIN_TIME_ALLOWANCE_S = 10.0

# Generous backstop in case a recovery goal's own done-callback never lands (server
# unresponsive) — mirrors the same unaccepted-goal gap _check_stall/_check_path_stall
# guard against elsewhere in this node. Comfortably above
# BACKUP_TIME_ALLOWANCE_S + SPIN_TIME_ALLOWANCE_S combined (the worst case: backup runs
# its full allowance, aborts, then spin also runs its full allowance).
RECOVERY_ABS_TIMEOUT_S = 45.0

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

# Raw frontier cells sit against unknown space, which almost always sits against a wall
# just beyond sensor range — so most raw frontier cells fall inside the wall's inflation
# halo (cost 0 in the raw SLAM map, but 96-100 in the actual global_costmap), which is
# why ComputePathToPose was observed live rejecting the large majority of raw frontier
# targets in this arena. Before navigating, the target is snapped to the nearest cell in
# /global_costmap/costmap whose cost is below COSTMAP_SAFE_COST — see
# frontier_detection.find_low_cost_point. Well under the 99 inscribed-inflated cutoff so
# the result is a point Nav2 can actually plan into, not one that merely scrapes under
# the lethal threshold.
COSTMAP_SAFE_COST = 50
COSTMAP_SEARCH_RADIUS_CELLS = 10

# Self-clearance check — see module docstring's "Self-clearance" section. This is the
# published-OccupancyGrid value of INSCRIBED_INFLATED_OBSTACLE: a cell reaches it exactly
# when a lethal obstacle lies within the robot's inscribed radius, so for the circular
# footprint nav2_params.yaml configures, "centre cell >= 99" IS the collision test the
# planner itself uses. Deliberately not COSTMAP_SAFE_COST's stricter 50 — this asks "am I
# actually in collision," not "is this a comfortable place to aim for."
SELF_CLEARANCE_MAX_COST = 99

# Coverage sweep — see module docstring's "Coverage sweep" section. Row spacing/min run
# length aren't yet tuned against the Kinect's FOV (swept_mask.CAMERA_HALF_FOV_RAD /
# CAMERA_MARK_RANGE_M) — that's a follow-up tuning pass once this interleaved design is
# validated live, not part of this change.
SWEEP_ROW_SPACING_M = 0.5
SWEEP_MIN_RUN_M = 0.3

# Sweep once un-swept known-free area exceeds this fraction of total known free area — a
# fraction, not a fixed square-meterage, so a small first room gets camera-checked almost
# immediately while a large mostly-swept arena isn't re-triggered by trivial new patches.
# See module docstring's "Interleaved, not sequential" section.
SWEEP_FRACTION = 0.15

# After returning from SWEEPING to FRONTIER, don't re-check the dynamic trigger for this
# long — otherwise the same un-swept ratio that just caused the return would very likely
# still be true on the next tick, re-triggering SWEEPING before a single frontier goal
# ever gets a chance to dispatch. 10 eval ticks: comfortably more than the 1-2 ticks a
# target-selection/snap/reachability-check cycle normally takes. See module docstring.
SWEEP_RETRIGGER_COOLDOWN_S = 20.0

MAP_FRAME = 'map'
ROBOT_FRAME = 'base_link'


class State:
    IDLE = 'IDLE'
    CHECKING_PATH = 'CHECKING_PATH'
    NAVIGATING = 'NAVIGATING'
    RECOVERING = 'RECOVERING'
    COVERAGE_COMPLETE = 'COVERAGE_COMPLETE'


class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('frontier_explorer_node')

        self._state = State.IDLE
        self._goal_handle = None
        # Set False by executor_node via /exploration_enabled when it takes the robot
        # over to chase a detected cube. Exploration must yield the base immediately —
        # both this node and the executor publish motion, and two controllers fighting
        # over cmd_vel is worse than either alone.
        self._enabled = True
        self._latest_map = None
        self._latest_costmap = None
        self._current_target = None
        # The original frontier-cluster coordinate a NavigateToPose attempt was picked
        # for — kept separate from _current_target (which becomes the cost-map-snapped
        # navigation point actually sent to Nav2, see COSTMAP_SAFE_COST) so blacklisting
        # bans the frontier CLUSTER, not a snapped point that can drift a few cells
        # between ticks as the costmap updates.
        self._frontier_origin = None
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

        # Stuck recovery — see module docstring. Set the first tick every known frontier
        # is blacklisted (no candidate left at all); cleared once a candidate exists again
        # or a recovery backup completes.
        self._stuck_since = None
        self._backup_goal_handle = None
        self._spin_goal_handle = None
        self._recovery_start_time = None  # rclpy.time.Time the recovery sequence started

        # Coverage sweep — see module docstring's "Coverage sweep" section.
        self._mode = 'FRONTIER'  # 'FRONTIER' | 'SWEEPING'
        self._sweep_queue = []
        self._exploration_complete_published = False
        # Back-to-back stuck-recoveries with no frontier becoming reachable again; once
        # this hits STUCK_RECOVERIES_BEFORE_SWEEP the node stops trying to explore and
        # sweeps instead. Reset whenever any frontier becomes actionable again.
        self._consecutive_recoveries = 0

        # Swept mask — tracks which map cells the camera has actually passed over. None
        # until the first /map arrives (needs known width/height to allocate). See
        # swept_mask.py and module docstring's "Interleaved, not sequential" section.
        self._swept_mask = None
        self._swept_mask_width = 0
        self._swept_mask_height = 0
        self._swept_mask_origin_x = 0.0
        self._swept_mask_origin_y = 0.0
        # Frontier-cluster count snapshotted when the current sweep began — see module
        # docstring's count-based (not presence-based) mid-sweep abort condition.
        self._sweep_start_frontier_count = 0
        # rclpy Time; while set and in the future, the dynamic sweep trigger is not
        # re-checked — see SWEEP_RETRIGGER_COOLDOWN_S.
        self._sweep_cooldown_until = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)

        # Ground truth for what the planner will actually accept — see
        # COSTMAP_SAFE_COST above for why the raw /map alone isn't enough.
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        costmap_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._costmap_cb, costmap_qos
        )

        # Latched so the executor's current enable/disable state is picked up even if
        # this node restarts mid-mission.
        enable_qos = QoSProfile(depth=1)
        enable_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, '/exploration_enabled', self._enabled_cb, enable_qos)

        complete_qos = QoSProfile(depth=1)
        complete_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._complete_pub = self.create_publisher(Bool, '/exploration_complete', complete_qos)
        self._coverage_complete_pub = self.create_publisher(
            Bool, '/coverage_complete', complete_qos
        )

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._backup_client = ActionClient(self, BackUp, 'backup')
        self._spin_client = ActionClient(self, Spin, 'spin')

        self.create_timer(EVAL_PERIOD_S, self._evaluate)

        self.get_logger().info('frontier_explorer_node started, waiting for /map ...')

    # ------------------------------------------------------------------ #
    # /map callback — just caches the latest message (see module docstring
    # for why evaluation itself is timer-driven, not callback-driven).
    # ------------------------------------------------------------------ #

    def _map_cb(self, msg: OccupancyGrid):
        self._latest_map = msg

        # Keep the swept mask in step with the map's dimensions/origin. RTAB-Map's /map
        # can grow on any edge as SLAM discovers more space, shifting its origin in x
        # and/or y — not just extending outward from a fixed corner — so a plain resize
        # isn't enough; existing True cells must be relocated too. See
        # swept_mask.resize_swept_mask's docstring for why that's done via a coordinate
        # round-trip rather than a hand-derived offset. Comparing against the cached
        # _swept_mask_* values (set below) rather than a stale copy of the previous
        # message avoids any ordering hazard from reading self._latest_map here.
        new_w, new_h = msg.info.width, msg.info.height
        new_ox, new_oy = msg.info.origin.position.x, msg.info.origin.position.y
        if self._swept_mask is None:
            self._swept_mask = create_swept_mask(new_w, new_h)
        elif (new_w, new_h, new_ox, new_oy) != (
            self._swept_mask_width, self._swept_mask_height,
            self._swept_mask_origin_x, self._swept_mask_origin_y,
        ):
            self._swept_mask = resize_swept_mask(
                self._swept_mask, self._swept_mask_width, self._swept_mask_height,
                msg.info.resolution, self._swept_mask_origin_x, self._swept_mask_origin_y,
                new_w, new_h, new_ox, new_oy,
            )
        self._swept_mask_width, self._swept_mask_height = new_w, new_h
        self._swept_mask_origin_x, self._swept_mask_origin_y = new_ox, new_oy

    def _costmap_cb(self, msg: OccupancyGrid):
        self._latest_costmap = msg

    # ------------------------------------------------------------------ #
    # /exploration_enabled — executor_node hands the robot back and forth
    # ------------------------------------------------------------------ #

    def _enabled_cb(self, msg: Bool):
        if msg.data == self._enabled:
            return
        self._enabled = msg.data
        if self._enabled:
            self.get_logger().info('Exploration ENABLED — resuming frontier search.')
            return

        self.get_logger().info('Exploration DISABLED — yielding the base to executor_node.')
        # Actively cancel rather than just going idle: an in-flight NavigateToPose,
        # BackUp, or Spin goal keeps Nav2 publishing cmd_vel, which would fight the
        # executor's own commands all the way through cube approach and capture.
        if self._goal_handle is not None and not self._cancel_requested:
            self._goal_handle.cancel_goal_async()
            self._cancel_requested = True
        if self._backup_goal_handle is not None:
            self._backup_goal_handle.cancel_goal_async()
            self._backup_goal_handle = None
        if self._spin_goal_handle is not None:
            self._spin_goal_handle.cancel_goal_async()
            self._spin_goal_handle = None
        # Invalidate any in-flight attempt so its late callback can't resurrect state
        # after we've handed control over.
        self._goal_epoch += 1
        self._path_epoch += 1
        self._current_target = None
        self._frontier_origin = None
        self._goal_start_time = None
        self._path_check_start_time = None
        self._last_progress_distance = None
        self._last_progress_time = None
        self._stuck_since = None
        self._recovery_start_time = None
        self._consecutive_recoveries = 0
        if self._state != State.COVERAGE_COMPLETE:
            self._state = State.IDLE

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
        if not self._enabled:
            # executor_node owns the base right now (chasing/capturing/delivering a
            # cube). Do nothing at all — not even a reachability check, since accepting
            # a goal here would put Nav2 back on cmd_vel behind the executor's back.
            self.get_logger().info(
                'Paused (exploration disabled by executor_node).',
                throttle_duration_sec=10.0,
            )
            return
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
        if self._state == State.RECOVERING:
            # A BackUp goal is in flight; wait for it to finish (or time out) rather
            # than picking a target concurrently — see _check_recovery_stall.
            self._check_recovery_stall()
            return
        if self._state == State.COVERAGE_COMPLETE:
            return
        if self._latest_map is None:
            self.get_logger().info(
                'Still waiting for the first /map message.', throttle_duration_sec=5.0
            )
            return
        if self._latest_costmap is None:
            self.get_logger().info(
                'Still waiting for the first /global_costmap/costmap message.',
                throttle_duration_sec=5.0,
            )
            return

        msg = self._latest_map

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return
        robot_x, robot_y, robot_yaw = robot_pose

        if not self._is_self_clear(robot_x, robot_y):
            self.get_logger().warn(
                'Own footprint overlaps high-cost space (parked too close to an '
                'obstacle) — backing up immediately rather than trying targets that '
                'are doomed from this start pose. See module docstring "Self-clearance".'
            )
            self._start_recovery_backup()
            return

        # Frontier detection runs every tick in BOTH modes, deliberately: sweeping drives
        # through the interior, which is exactly where previously-occluded pockets come
        # into view, so new frontiers can appear mid-sweep. Checking only while in
        # FRONTIER mode would strand them unexplored forever.
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

        # Swept-mask update — see module docstring's "Interleaved, not sequential"
        # section. Runs every tick in both modes, same reasoning as frontier detection
        # above: the robot's camera passes over new ground regardless of which mode is
        # driving it.
        if self._swept_mask is not None:
            mark_swept_cells(
                self._swept_mask, self._swept_mask_width, self._swept_mask_height,
                msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y,
                robot_x, robot_y, robot_yaw,
            )
        total_free, unswept_free = count_unswept_free(
            msg.data, self._swept_mask, msg.info.width, msg.info.height
        )
        self.get_logger().info(
            f'Swept coverage: {total_free - unswept_free}/{total_free} free cells swept '
            f'({100.0 * (1.0 - unswept_free / max(1, total_free)):.1f}%)',
            throttle_duration_sec=10.0,
        )

        if self._mode == 'SWEEPING':
            if len(frontiers_world) > self._sweep_start_frontier_count:
                # A genuinely new pocket became visible mid-sweep — count-based, not
                # presence-based, see module docstring for why. Map first, cover second:
                # an unmapped pocket may contain floor the current queue doesn't even
                # mention, so the queue is discarded rather than resumed, and rebuilt
                # against the larger map once frontiers are handled again.
                self.get_logger().info(
                    f'{len(frontiers_world)} frontier cluster(s) now known (was '
                    f'{self._sweep_start_frontier_count} when this sweep began) — a new '
                    f'pocket became visible mid-sweep; returning to frontier exploration, '
                    f'the sweep will be replanned afterwards.'
                )
                self._mode = 'FRONTIER'
                self._sweep_queue = []
                self._sweep_cooldown_until = (
                    self.get_clock().now() + Duration(seconds=SWEEP_RETRIGGER_COOLDOWN_S)
                )
                # Fall through to FRONTIER handling below, same tick.
            else:
                self._dispatch_next_sweep_waypoint(robot_x, robot_y, frontiers_world)
                return

        # --- FRONTIER mode from here (originally, or just switched above) ---

        if not frontiers_world:
            if unswept_free == 0:
                self._publish_coverage_complete('nothing left to explore or sweep')
            else:
                self._begin_coverage_sweep(
                    msg, robot_x, robot_y, frontiers_world, 'no frontiers remain',
                    exploration_exhausted=True,
                )
            return

        candidates = self._filter_blacklisted(frontiers_world)
        if not candidates:
            if unswept_free == 0:
                self._publish_coverage_complete(
                    'all known frontiers unreachable and coverage is complete'
                )
            else:
                self._handle_all_frontiers_blacklisted(msg, robot_x, robot_y, frontiers_world)
            return
        self._stuck_since = None
        self._consecutive_recoveries = 0

        cooldown_active = (
            self._sweep_cooldown_until is not None
            and self.get_clock().now() < self._sweep_cooldown_until
        )
        if not cooldown_active and should_trigger_sweep(total_free, unswept_free, SWEEP_FRACTION):
            self._begin_coverage_sweep(
                msg, robot_x, robot_y, frontiers_world,
                f'un-swept area ({unswept_free}/{total_free} free cells) exceeds '
                f'{SWEEP_FRACTION * 100:.0f}% of known free space',
            )
            return

        target = select_target(candidates, robot_x, robot_y)

        if distance(robot_x, robot_y, target[0], target[1]) < MIN_TARGET_DISTANCE_M:
            self.get_logger().debug('Nearest frontier is too close, waiting for next map update.')
            return

        nav_point = self._snap_to_reachable(target)
        if nav_point is None:
            self.get_logger().info(
                f'Frontier ({target[0]:.2f}, {target[1]:.2f}) has no low-cost cell within '
                f'{COSTMAP_SEARCH_RADIUS_CELLS} cells in the costmap (buried in wall '
                f'inflation); blacklisting without spending a path-check on it.'
            )
            self._blacklist_target(target[0], target[1])
            return

        nav_x, nav_y = nav_point
        # The MIN_TARGET_DISTANCE_M check above guards the RAW frontier, but snapping can
        # move the point we actually drive to by several cells — occasionally right onto
        # where the robot already stands. Nav2 then returns SUCCEEDED within ~70 ms
        # without moving, and because it SUCCEEDED nothing gets blacklisted, so the very
        # same frontier is selected again on the next tick, forever. Confirmed live in
        # sim: the same target was re-issued every ~5 s for minutes, racking up
        # "successes" while the map stopped growing entirely and the frontier count
        # froze — exploration livelocked while every counter said it was working.
        # Blacklisting is the right response: driving to this frontier cannot consume it
        # (the robot is already there and the cells stayed unknown), so it must step
        # aside and let the other frontiers have a turn.
        if distance(robot_x, robot_y, nav_x, nav_y) < MIN_TARGET_DISTANCE_M:
            self.get_logger().info(
                f'Frontier ({target[0]:.2f}, {target[1]:.2f}) snapped to '
                f'({nav_x:.2f}, {nav_y:.2f}), which is where the robot already stands — '
                f'navigating there would succeed instantly without exploring anything. '
                f'Blacklisting so other frontiers get a turn.'
            )
            self._blacklist_target(target[0], target[1])
            return

        self.get_logger().info(
            f'{len(clusters)} frontier(s) found, targeting ({target[0]:.2f}, {target[1]:.2f}) '
            f'-> snapped to reachable point ({nav_x:.2f}, {nav_y:.2f})'
        )
        yaw = math.atan2(nav_y - robot_y, nav_x - robot_x)
        self._frontier_origin = target
        self._check_reachability(nav_x, nav_y, yaw)

    def _snap_to_reachable(self, target_world):
        """Move a raw frontier point to the nearest costmap cell Nav2 can plan into.

        See COSTMAP_SAFE_COST module comment for why this is necessary. Returns a world
        (x, y) tuple, or None if no low-cost cell exists within COSTMAP_SEARCH_RADIUS_CELLS.

        COSTMAP_SAFE_COST (50) is comfortably below the inscribed cutoff, so a cell that
        passes this snap is by construction not a colliding pose either — the snap and
        _is_self_clear agree rather than pulling against each other.
        """
        cm = self._latest_costmap
        row, col = world_to_grid(
            target_world[0], target_world[1], cm.info.resolution,
            cm.info.origin.position.x, cm.info.origin.position.y
        )
        snapped = find_low_cost_point(
            cm.data, cm.info.width, cm.info.height, row, col,
            max_cost=COSTMAP_SAFE_COST, search_radius=COSTMAP_SEARCH_RADIUS_CELLS
        )
        if snapped is None:
            return None
        return grid_to_world(
            snapped[0], snapped[1], cm.info.resolution,
            cm.info.origin.position.x, cm.info.origin.position.y
        )

    def _is_self_clear(self, robot_x, robot_y):
        """Whether the robot's current pose is collision-free, by Nav2's own criterion.

        See module docstring's "Self-clearance" section, and
        frontier_detection.in_collision for why this is a single centre-cell lookup
        rather than a scan over the footprint radius (the earlier radius scan
        double-counted the inflation and reported open floor as blocked).
        """
        cm = self._latest_costmap
        row, col = world_to_grid(
            robot_x, robot_y, cm.info.resolution,
            cm.info.origin.position.x, cm.info.origin.position.y
        )
        return not in_collision(
            cm.data, cm.info.width, cm.info.height, row, col,
            inscribed_cost=SELF_CLEARANCE_MAX_COST
        )

    def _publish_coverage_complete(self, reason):
        """Declare the mission genuinely done.

        See module docstring's "Mission completion" paragraph for why this requires both
        no reachable frontier candidate AND full swept coverage, checked together at
        every call site, never swept-ratio alone. State.COVERAGE_COMPLETE is a dead end
        (_evaluate returns unconditionally once in it), so this must not be reachable
        prematurely.
        """
        self._coverage_complete_pub.publish(Bool(data=True))
        self._state = State.COVERAGE_COMPLETE
        self.get_logger().info(f'Mission complete — {reason}.')

    def _dispatch_next_sweep_waypoint(self, robot_x, robot_y, frontiers_world):
        """Pop and send the next coverage-sweep waypoint — see module docstring.

        Reuses the same snap/reachability pipeline as a frontier target
        (_snap_to_reachable, _check_reachability), but _frontier_origin is left None:
        every blacklist call site already guards on it being set, so a sweep waypoint
        that fails is simply not retried (it was already popped from the queue) rather
        than entered into the cooldown bookkeeping meant for frontier clusters that get
        re-evaluated every tick.
        """
        if not self._sweep_queue:
            candidates = self._filter_blacklisted(frontiers_world)
            total_free, unswept_free = count_unswept_free(
                self._latest_map.data, self._swept_mask,
                self._latest_map.info.width, self._latest_map.info.height,
            )
            if not candidates and unswept_free == 0:
                self._publish_coverage_complete(
                    'sweep queue exhausted, no frontiers left, fully swept'
                )
            else:
                self._mode = 'FRONTIER'
                self._sweep_cooldown_until = (
                    self.get_clock().now() + Duration(seconds=SWEEP_RETRIGGER_COOLDOWN_S)
                )
                self.get_logger().info(
                    f'Sweep queue exhausted ({unswept_free} un-swept free cell(s), '
                    f'{len(candidates)} frontier candidate(s) remain) — returning to '
                    f'frontier mode.'
                )
            return

        target = self._sweep_queue.pop(0)
        if distance(robot_x, robot_y, target[0], target[1]) < MIN_TARGET_DISTANCE_M:
            self.get_logger().debug('Sweep waypoint too close to the robot; skipping it.')
            return

        nav_point = self._snap_to_reachable(target)
        if nav_point is None:
            self.get_logger().info(
                f'Sweep waypoint ({target[0]:.2f}, {target[1]:.2f}) has no low-cost cell '
                f'nearby in the costmap; marking it swept-but-skipped so an un-drivable '
                f'pocket cannot block coverage completion forever.'
            )
            if self._swept_mask is not None:
                mark_world_point_swept(
                    self._swept_mask, self._swept_mask_width, self._swept_mask_height,
                    self._latest_map.info.resolution,
                    self._latest_map.info.origin.position.x,
                    self._latest_map.info.origin.position.y,
                    target[0], target[1],
                )
            return

        nav_x, nav_y = nav_point
        self.get_logger().info(
            f'Sweep waypoint ({target[0]:.2f}, {target[1]:.2f}) -> snapped to reachable '
            f'point ({nav_x:.2f}, {nav_y:.2f}); {len(self._sweep_queue)} remaining.'
        )
        yaw = math.atan2(nav_y - robot_y, nav_x - robot_x)
        self._frontier_origin = None
        self._check_reachability(nav_x, nav_y, yaw)

    def _filter_blacklisted(self, frontiers_world):
        now = self.get_clock().now()
        self._blacklist = [b for b in self._blacklist if b[2] > now]

        def is_blacklisted(x, y):
            return any(
                distance(x, y, bx, by) < BLACKLIST_RADIUS_M
                for (bx, by, _exp) in self._blacklist
            )

        return [(x, y) for (x, y) in frontiers_world if not is_blacklisted(x, y)]

    # ------------------------------------------------------------------ #
    # Stuck recovery — see module docstring
    # ------------------------------------------------------------------ #

    def _handle_all_frontiers_blacklisted(self, msg, robot_x, robot_y, frontiers_world):
        now = self.get_clock().now()
        if self._stuck_since is None:
            self._stuck_since = now
        elapsed_s = (now - self._stuck_since).nanoseconds / 1e9
        if elapsed_s < STUCK_RECOVERY_TIMEOUT_S:
            self.get_logger().debug(
                'All known frontiers are on cooldown after recent failures, waiting '
                f'({elapsed_s:.0f}/{STUCK_RECOVERY_TIMEOUT_S:.0f}s before backing up).'
            )
            return

        # Recovery exists to change the geometry that made the frontiers unreachable. If
        # that many attempts have not made a single one reachable again, the geometry is
        # not the problem — what is left is genuinely unreachable (behind a wall, outside
        # the arena, on the far side of a gap the robot cannot cross), and backing up
        # forever accomplishes nothing. Exploration is as complete as it is ever going to
        # get, so hand over to the coverage sweep. Without this the sweep is effectively
        # dead code: in a bounded arena frontiers do not fall to zero, they end up
        # permanently blacklisted, which the "no frontiers remain" branch never sees.
        if self._consecutive_recoveries >= STUCK_RECOVERIES_BEFORE_SWEEP:
            self._begin_coverage_sweep(
                msg, robot_x, robot_y, frontiers_world,
                f'every remaining frontier stayed unreachable across '
                f'{self._consecutive_recoveries} recovery attempts',
                exploration_exhausted=True,
            )
            return

        self._consecutive_recoveries += 1
        self.get_logger().warn(
            f'Every known frontier has been unreachable for {STUCK_RECOVERY_TIMEOUT_S:.0f}s '
            f'straight — backing up instead of waiting out the cooldown '
            f'(recovery {self._consecutive_recoveries}/{STUCK_RECOVERIES_BEFORE_SWEEP} '
            f'before giving up on frontiers and sweeping).'
        )
        self._start_recovery_backup()

    def _begin_coverage_sweep(
        self, msg, robot_x, robot_y, frontiers_world, reason, exploration_exhausted=False,
    ):
        """Enter (or continue) the coverage sweep — see module docstring "Coverage sweep".

        Reached from three places: frontier detection returning nothing at all, every
        remaining frontier proving persistently unreachable, and the dynamic un-swept-area
        trigger firing while frontiers are still being actively explored. Only the first
        two mean frontier exploration is actually exhausted — the third is a normal
        interleaving pause, not the end of exploration, so exploration_exhausted must be
        passed True only by those first two call sites; it gates /exploration_complete,
        which would otherwise fire misleadingly early on the very first small sweep.
        """
        if exploration_exhausted and not self._exploration_complete_published:
            self._complete_pub.publish(Bool(data=True))
            self._exploration_complete_published = True
            self.get_logger().info(f'Exploration complete — {reason}.')
        if self._mode != 'SWEEPING':
            self._mode = 'SWEEPING'
            self._blacklist = []
            self._stuck_since = None
            self._consecutive_recoveries = 0
            self._sweep_start_frontier_count = len(frontiers_world)
            self._sweep_queue = generate_coverage_waypoints(
                msg.data, msg.info.width, msg.info.height, msg.info.resolution,
                msg.info.origin.position.x, msg.info.origin.position.y,
                row_spacing_m=SWEEP_ROW_SPACING_M, min_run_m=SWEEP_MIN_RUN_M,
                swept_mask=self._swept_mask,
            )
            self.get_logger().info(
                f'Switching to coverage sweep ({reason}): {len(self._sweep_queue)} '
                f'waypoint(s) queued over the known map.'
            )
        self._dispatch_next_sweep_waypoint(robot_x, robot_y, frontiers_world)

    def _start_recovery_backup(self):
        if not self._backup_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('backup action server not available yet; will retry next tick.')
            return
        self.get_logger().info(f'Backing up {BACKUP_DISTANCE_M:.2f}m.')
        goal = BackUp.Goal()
        goal.target.x = -BACKUP_DISTANCE_M
        goal.speed = BACKUP_SPEED_MPS
        goal.time_allowance = Duration(seconds=BACKUP_TIME_ALLOWANCE_S).to_msg()

        self._state = State.RECOVERING
        self._recovery_start_time = self.get_clock().now()
        send_future = self._backup_client.send_goal_async(goal)
        send_future.add_done_callback(self._backup_response_cb)

    def _backup_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('BackUp goal rejected; will retry stuck-recovery next tick.')
            self._finish_recovery()
            return
        self._backup_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._backup_result_cb)

    def _backup_result_cb(self, future):
        result = future.result()
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }.get(result.status, f'status={result.status}')
        self.get_logger().info(f'BackUp recovery finished: {status_name}')
        self._backup_goal_handle = None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._finish_recovery()
            return
        # BackUp itself failed — most likely Nav2's own "Collision Ahead" from the
        # local costmap. Confirmed live: this happens when the obstacle self-clearance
        # detected sits beside/behind the robot's current heading rather than in front
        # of it, so straight backward travel drives right into the same wall. A
        # rotation needs only radial clearance, not linear clearance in one specific
        # direction — see module docstring's "Self-clearance" section.
        self._start_recovery_spin()

    def _start_recovery_spin(self):
        if not self._spin_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                'spin action server not available yet; giving up on this recovery cycle.'
            )
            self._finish_recovery()
            return
        self.get_logger().info(
            f'BackUp could not clear the obstacle; spinning '
            f'{math.degrees(RECOVERY_SPIN_ANGLE_RAD):.0f} deg instead.'
        )
        goal = Spin.Goal()
        goal.target_yaw = RECOVERY_SPIN_ANGLE_RAD
        goal.time_allowance = Duration(seconds=SPIN_TIME_ALLOWANCE_S).to_msg()
        send_future = self._spin_client.send_goal_async(goal)
        send_future.add_done_callback(self._spin_response_cb)

    def _spin_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Spin goal rejected; ending this recovery cycle.')
            self._finish_recovery()
            return
        self._spin_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._spin_result_cb)

    def _spin_result_cb(self, future):
        result = future.result()
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }.get(result.status, f'status={result.status}')
        self.get_logger().info(f'Spin recovery finished: {status_name}')
        self._spin_goal_handle = None
        # Whether spin succeeded or not, the recovery cycle is over — if the robot is
        # still footprint-blocked, next tick's self-clearance check will catch it and
        # start another backup/spin cycle (see module docstring's "Self-clearance").
        self._finish_recovery()

    def _check_recovery_stall(self):
        """Give up waiting on a recovery goal (BackUp or Spin) that never resolves at all.

        Mirrors _check_stall/_check_path_stall's unaccepted-goal handling — see module
        docstring. RECOVERY_ABS_TIMEOUT_S is comfortably above the combined worst-case
        BackUp + Spin runtime, so this only fires if a done-callback never lands.
        """
        if self._recovery_start_time is None:
            return
        elapsed_s = (self.get_clock().now() - self._recovery_start_time).nanoseconds / 1e9
        if elapsed_s < RECOVERY_ABS_TIMEOUT_S:
            return
        self.get_logger().warn(
            f'Recovery did not resolve within {RECOVERY_ABS_TIMEOUT_S:.0f}s; '
            f'giving up on it directly.'
        )
        self._finish_recovery()

    def _finish_recovery(self):
        # Give every frontier a fresh chance from the new pose — the whole point of
        # backing up is that the geometry that made them unreachable may have changed.
        # failure_history is NOT cleared: a target that's genuinely unreachable (not
        # just occluded by the robot's prior position) still gets pushed out further on
        # repeat offenses — see module docstring.
        self._blacklist = []
        self._stuck_since = None
        self._backup_goal_handle = None
        self._spin_goal_handle = None
        self._recovery_start_time = None
        self._state = State.IDLE

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
            if self._frontier_origin is not None:
                x, y = self._frontier_origin
                self._blacklist_target(x, y)
            self._current_target = None
            self._frontier_origin = None
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
        if self._frontier_origin is not None:
            x, y = self._frontier_origin
            self._blacklist_target(x, y)
        self._current_target = None
        self._frontier_origin = None
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
        """Return (x, y, yaw) of the robot in MAP_FRAME, or None."""
        try:
            t = self._tf_buffer.lookup_transform(
                MAP_FRAME, ROBOT_FRAME, rclpy.time.Time(), Duration(seconds=0.5)
            )
        except TransformException as ex:
            msg = f'Could not get robot pose ({MAP_FRAME}->{ROBOT_FRAME}): {ex}'
            self.get_logger().warn(msg, throttle_duration_sec=5.0)
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

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
            self._frontier_origin = None
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
            if self._frontier_origin is not None:
                fx, fy = self._frontier_origin
                self._blacklist_target(fx, fy)
            self._current_target = None
            self._frontier_origin = None
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
            self._frontier_origin = None
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
            self._frontier_origin = None
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

        if result.status != GoalStatus.STATUS_SUCCEEDED and self._frontier_origin is not None:
            x, y = self._frontier_origin
            self._blacklist_target(x, y)

        self._current_target = None
        self._frontier_origin = None
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
