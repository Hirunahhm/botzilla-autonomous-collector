# Frontier exploration: fixing "gets stuck in places"

**Status:** implemented, unit-tested, not yet re-validated on hardware with a full
unattended run.

**Files:** `botzilla_navigation/frontier_detection.py`,
`botzilla_navigation/frontier_explorer_node.py`,
`botzilla_navigation/test/test_frontier_detection.py`.

## Symptom

Milestone 4's `frontier_explorer_node` (nearest-frontier exploration via Nav2's
`NavigateToPose`) would explore for a while and then stop making progress in certain
spots — exploration looked "stuck in places" even though the robot and Nav2 stack were
otherwise healthy.

Four separate contributing issues were found and fixed, roughly in order of impact.

## 1. Frontier target could land outside the frontier entirely (the main bug)

`find_frontiers()` clusters connected frontier cells (free cells bordering unknown space)
and picks a target point per cluster. It used to return the **raw mean** of all member
cells:

```python
mean_r = sum(p[0] for p in cells) / len(cells)
mean_c = sum(p[1] for p in cells) / len(cells)
clusters.append((mean_r, mean_c, len(cells)))
```

For a convex, straight-line cluster the mean is fine. For anything else — an L-shape, a
cluster wrapping a corner or a pillar, a ring around an obstacle — the mean can land
**outside the cluster**, sometimes on a cell that isn't even free space. Nav2 was handed
that point as a goal, failed to reach it (correctly — it often wasn't reachable at all),
and the node blacklisted that exact spot. Since the cluster's shape barely changes between
map updates, the same bad mean recomputed on the next tick, and exploration stalled on
that region.

Concretely, for a 5-cell L-shaped cluster tracing `(0,0)→(1,0)→(2,0)→(2,1)→(2,2)`, the
mean is `(1.4, 0.6)` — not a member of the cluster, and it lands almost exactly on cell
`(1,1)`, which in the test fixture is **unknown space**.

### Fix

Snap the target to the actual cluster member cell nearest the mean:

```python
target_r, target_c = min(
    cells, key=lambda p: (p[0] - mean_r) ** 2 + (p[1] - mean_c) ** 2
)
clusters.append((target_r, target_c, len(cells)))
```

This guarantees the returned point is always a real, free, in-cluster cell. It also
incidentally makes the failure-blacklist (see below) more effective, since the target for
a given physical frontier is now far more stable between map updates than a shifting
fractional mean was.

Regression test:
`test_frontier_detection.py::test_frontier_target_is_always_a_member_cell_for_l_shaped_cluster`
builds the exact L-shaped grid above and asserts the returned target is one of the 5
member cells.

## 2. No bound on how long a single bad goal could block exploration

The node's original policy was to never preempt an in-flight Nav2 goal, and just wait for
Nav2 to resolve it (`SUCCEEDED`/`ABORTED`/`CANCELED`). A genuinely unreachable target was
observed live taking Nav2 **~300s** to exhaust its internal recovery cycles before
reporting `ABORTED`. For that whole time the explorer does nothing — indistinguishable
from "stuck" from the outside, even before the blacklist mechanism gets a chance to route
around it.

### Fix (two iterations)

First pass: a flat 90s cap that cancels the goal itself instead of waiting on Nav2. On
reflection this was the wrong tool — a legitimate goal on the far side of a large arena
can genuinely take longer than 90s while still making real progress, and a flat cap would
cancel it.

Final version: track `NavigateToPose`'s `distance_remaining` feedback and only cancel when
it **stops improving**:

```python
NO_PROGRESS_TIMEOUT_S = 30.0   # cancel if no improvement in this long
PROGRESS_EPSILON_M = 0.15      # minimum improvement to count as "progress"
GOAL_ABS_TIMEOUT_S = 420.0     # backstop if feedback never arrives at all
```

A goal that is actually driving toward its target, however slowly, is never touched. Only
a goal that has genuinely stopped improving (stuck against an obstacle, oscillating, etc.)
or one that never produces feedback at all gets canceled. The cancellation is treated like
any other non-`SUCCEEDED` result — it flows into the existing blacklist path.

## 3. No fast way to detect an obviously unreachable target

Even with the watchdog, a bad target still cost up to 30s (or a full Nav2 attempt) to rule
out. Many bad targets are detectably bad **immediately** — behind a wall the global
costmap already knows about, outside the map bounds, etc.

### Fix

Before committing to a full `NavigateToPose` attempt, the node now calls Nav2's own
planner directly via the `compute_path_to_pose` action and checks whether a path exists at
all:

```python
reachable = (
    result.status == GoalStatus.STATUS_SUCCEEDED
    and result.result.error_code == ComputePathToPose.Result.NONE
    and len(result.result.path.poses) > 0
)
```

This typically resolves in well under a second. A `NO_VALID_PATH` (or similar) result skips
the target immediately and blacklists it, without ever spending a full navigation attempt.
This does **not** replace the stall watchdog — a path existing at plan time doesn't
guarantee the local controller can still execute it a moment later (dynamic obstacles,
costmap changes mid-transit) — so a real `NavigateToPose` attempt can still stall and still
needs its own detection.

New state: `CHECKING_PATH`, sitting between `IDLE` and `NAVIGATING` in the state machine.

## 4. Goal heading always faced a fixed direction

Every goal was sent with `orientation.w = 1.0` — a fixed heading regardless of the
robot's approach direction. This could force an unnecessary final rotate-in-place right at
the frontier edge, which is the same maneuver that motivated the project's own "DWB
rotate-in-place stall" fix elsewhere in the Nav2 stack.

### Fix

Orient the goal toward the direction of travel instead:

```python
yaw = math.atan2(target_y - robot_y, target_x - robot_x)
goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
```

## State machine (after all four fixes)

```
IDLE ──(frontier found, not blacklisted)──> CHECKING_PATH
CHECKING_PATH ──(path exists)──> NAVIGATING
CHECKING_PATH ──(no valid path)──> IDLE   (target blacklisted)
NAVIGATING ──(SUCCEEDED)──> IDLE
NAVIGATING ──(ABORTED / CANCELED / watchdog cancel)──> IDLE   (target blacklisted)
IDLE ──(no frontiers remain)──> EXPLORATION_COMPLETE
```

`_evaluate()` runs on a periodic timer (`EVAL_PERIOD_S = 2.0`), not directly off `/map`
arrival — `/map` only republishes when the map actually changes, which only happens when
the robot moves, so a callback-driven design can deadlock on a transient startup failure.

## Round 2: code-review follow-up (deadlocks, watchdog gap, 8-connectivity)

A code review of the round-1 changes above found two real bugs and two minor issues, all
confirmed against the code and fixed.

### Bug: `_send_goal` could strand the node in `CHECKING_PATH` forever

`_path_result_cb` calls `_send_goal(x, y, yaw)` on a successful reachability check, while
`self._state` is still `CHECKING_PATH`. `_send_goal`'s own `wait_for_server` early-return
didn't reset state or `_current_target` — and since `_evaluate()` unconditionally no-ops
on `CHECKING_PATH`, nothing could ever pull the node back out. Not just theoretical: this
project's own hardware testing has directly observed Nav2's action servers take 90+
seconds to become responsive under CPU load (see the CPU-pressure investigation the same
session), which is exactly the condition that trips this. Fixed by resetting `state=IDLE`
and clearing `_current_target` in that branch.

### Bug: unaccepted goals evaded both stall timeouts

`_check_stall` required `self._goal_handle is not None` before doing anything — including
the `GOAL_ABS_TIMEOUT_S` backstop that the module docstring explicitly promises covers
"the degenerate case where feedback never arrives at all." But `_goal_handle` is only set
inside the action's done-callback; if that callback never fires (server dies or is
unresponsive mid-handshake), the guard blocked the backstop from ever firing, and
`NAVIGATING` became a genuine permanent state — contradicting the docstring's own claim.

Fixed by giving `_check_stall` a branch for `_goal_handle is None` that bounds the
"never accepted" case directly against `_goal_start_time` and `GOAL_ABS_TIMEOUT_S`,
independent of ever getting a handle. The same gap existed for the reachability
pre-check's `compute_path_to_pose` client too (unflagged by the review, found while
implementing the fix), covered by a new `_check_path_stall()` called from `_evaluate()`'s
`CHECKING_PATH` branch, bounded by a much shorter `PATH_CHECK_TIMEOUT_S = 15.0` — path
checks normally resolve in well under a second, so 15s is already a generous multiple,
unlike `GOAL_ABS_TIMEOUT_S` (420s) which has to tolerate an actual multi-minute drive.

**Giving up this way opens a race**: the real done-callback can still land later, after
the node has already moved on to a different target. A per-attempt epoch counter
(`_goal_epoch` / `_path_epoch`, bumped each time `_send_goal` / `_check_reachability`
starts a new attempt) lets `_goal_response_cb` / `_path_goal_response_cb` recognize a
callback belonging to an attempt already abandoned. If that stale response turns out to
have been accepted after all, it's canceled as an orphan instead of silently overwriting
newer state — or worse, leaving the robot physically executing a goal this node no longer
tracks or can blacklist.

### Minor: 4-connectivity only

`frontier_detection.py` used the same 4-neighbor tuple for both "does this free cell
border unknown space" and the clustering flood-fill. Two frontier cells touching only
diagonally landed in separate clusters, and if each was under `min_cluster_size` alone,
both got silently discarded even though combined they'd clear it. Fixed by switching both
loops to a shared `_NEIGHBORS_8` tuple. All 9 existing unit tests passed unchanged (none
of their grids had diagonal-only adjacency to begin with); two new regression tests cover
the diagonal-merge case directly:
`test_diagonal_frontier_cells_merge_via_8_connectivity` and
`test_frontier_classification_uses_8_connectivity_too`.

### Minor: inconsistent cleanup on goal rejection

`_path_goal_response_cb`'s rejection branch cleared `_current_target`;
`_goal_response_cb`'s didn't. Harmless in practice (overwritten before next read), fixed
for consistency.

## What was NOT changed

- **Target selection is still pure nearest-by-distance** (`select_target` in
  `frontier_detection.py`), not weighted by cluster size / information gain. This can still
  produce inefficient back-and-forth between small nearby patches instead of committing to
  a large unexplored region. Left alone deliberately — it's a separate concern from "stuck,"
  more of an efficiency tuning question, and changing it touches the core selection
  algorithm's tests.
- **Blacklist radius/matching** (`BLACKLIST_RADIUS_M = 0.5`, keyed on exact world
  coordinates) wasn't changed directly — fix #1 already makes target points far more
  stable between map updates, which was the main thing undermining it.

## Verification

```bash
cd botzilla_Workspace
python3 -m pytest src/botzilla_navigation/test/test_frontier_detection.py -v
colcon build --symlink-install --packages-select botzilla_navigation
colcon test --packages-select botzilla_navigation
colcon test-result --verbose
```

Unit tests are pure Python (no ROS imports), so they run without sourcing a ROS
environment — 11 passing after round 2 (9 + 2 new 8-connectivity regression tests).
`colcon test` shows 18 flake8 warnings + pep257 failures, all pre-existing and in files
untouched by this work (`odom_covariance_relay.py`, `nav2.launch.py`, `rtabmap.launch.py`,
`rtabmap_debug.launch.py`, `setup.py`).

Round 2 (the deadlock/watchdog/epoch fixes in `frontier_explorer_node.py`) has no pure
unit test coverage — the state machine's rclpy Node lifecycle, action clients, and
timer-driven `_evaluate()` aren't exercised by the ROS-independent test file, so these
fixes are verified by code review + build/lint only, not by a test that actually drives
the deadlock scenario. Worth adding actual node-level tests if this class of bug recurs.

**Not yet done:** a full unattended hardware/sim exploration run to confirm the "stuck in
places" symptom is actually gone in practice, not just fixed in isolated unit tests. That's
the next step.
