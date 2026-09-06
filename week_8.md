# Week 8 Progress Report: Drivetrain Root Cause, Custom Nav2 Plugins, and the Obstacle-Hugging Deadlock

This document covers Week 8 of the Semester 5 project (*Interleaved Exploration and Coverage
for Target Retrieval under Short-Range Perception*), all of it on the `research` branch.

Two results dominate the week:

1. **A one-line condition in the Kobuki driver was destroying up to 91% of every commanded
   turn.** It had been present for the entire project and was invisible to every layer above
   it — the controller commanded a turn, the driver reported sending it, and the robot
   physically barely moved. Found by bench measurement, not by reading code.
2. **The repeated "no progress for 30s" stalls are a *planning* problem, not a control
   problem.** The robot travels inside the costmap's inscribed band, and from there DWB's
   `ObstacleFootprint` critic legitimately vetoes every trajectory. Four separate attempts to
   fix this at the controller/recovery layer produced one clear regression and one
   improvement; the change that actually addressed the cause was a cost-aware global planner.

A recurring theme, and the more useful lesson: **several plausible fixes made things worse,
and the metric that looked like the goal (stall count) turned out to be a bad proxy.** Those
negative results are recorded here in full, because they cost real hardware time and should
not be repeated.

---

## 1. The Drivetrain Root Cause: `rotate_flag`

### Symptom

Across every run, the frontier explorer's watchdog fired repeatedly with
`Goal to (x, y) stalled (no progress for 3Xs)`. Five hypotheses were investigated and
rejected before the real cause was found (§7).

### Cause

`kobuki_base_node.py` chose the driver's pure-rotation mode only when linear velocity was
**exactly** `0.0`:

```python
rotate_flag = 1 if self._cmd_lin == 0.0 and angular_z != 0.0 else 0
```

`KobukiDriver.move()` scales its speed argument differently in the two branches:

| branch | `botspeed` | meaning |
|---|---|---|
| rotation (`rotate == 1`) | `abs(R - L) / 2` | tangential **wheel** speed |
| arc (`rotate == 0`) | `(L + R) / 2` | robot **centre** speed |

For a near-pivot these describe near-identical motion, but the arc form asks for it at a
fraction of the magnitude, and the base's own low-speed threshold then swallows it. At
`lin=0.0105, ang=0.400` the rotation branch asks for **46 mm/s** and the arc branch for
**10.5 mm/s** — a 4.4× reduction.

DWB essentially never emits a linear velocity of precisely `0.0` while turning, so **nearly
every mission turn took the degraded path.**

### Measurement

A bench A/B was run with the threshold exposed as a ROS parameter (`pivot_radius_m`), so both
arms ran back-to-back on the same hardware at the same battery voltage. Setting it to `1e-9`
reproduces the old `lin == 0.0` test exactly — a pure rotation has turn radius 0, everything
else is far above any epsilon. Yaw was read from the **gyro**, which is independent of the
wheel-speed model under test.

Achieved yaw as a fraction of commanded, both arms at 15.9 V:

| lin | ang | radius | old test | radius test | |
|---|---|---|---|---|---|
| 0.0000 | 0.400 | 0.000 | 1.01 | 1.04 | unchanged |
| 0.0105 | −0.400 | 0.026 | **0.16** | **1.03** | 6.7× |
| 0.0300 | 0.400 | 0.075 | **0.36** | **1.01** | 2.8× |
| 0.0600 | −0.400 | 0.150 | 0.79 | 0.78 | **control — true arc, unchanged** |
| 0.0105 | 0.800 | 0.013 | **0.09** | **1.01** | 11.4× |

The 0.150 m row is the control: it is a genuine arc under both tests and comes out identical,
proving the change is confined to the near-pivot regime and does not alter real arcs.

Note the loss **worsens as commanded yaw rises** (0.09 at 0.800 rad/s) — precisely DWB's
turn-hard regime, and why recovery behaviours and tight goal alignment suffered most.

### Fix

Test the turn **radius**, not "linear is exactly zero". A radius inside half the wheelbase
means the turn centre lies within the robot's own footprint — a pivot in all but name:

```python
turn_radius = abs(self._cmd_lin / self._cmd_ang) if self._cmd_ang != 0.0 else float('inf')
rotate_flag = 1 if angular_z != 0.0 and turn_radius < pivot_radius else 0
```

### Secondary finding

Reproducing the fault at **15.9 V** definitively killed the leading prior hypothesis, that
motor torque fell as the pack drained. The deadband sweep also refuted it directly:
`achieved = 1.030 × cmd − 0.003`.

### Also added

`/battery` (`sensor_msgs/BatteryState`) and `/wheel_pwm` (`std_msgs/Int16MultiArray`).
`Batteryvolt` and the PWM bytes were already parsed by `Kobuki.py` and never surfaced, which
left a real failure mode unobservable. PWM matters as much as voltage because together they
separate the two explanations: high PWM with little motion means the wheels are loaded or
stuck; low PWM means the driver never asked for enough.

*Commit: `56fd9de`*

---

## 2. `botzilla_straightline_planner` — First C++ Package in the Workspace

**Problem.** During the boustrophedon coverage sweep, NavFn curved off pre-validated straight
rows toward locally-cheaper cells, which read as the robot wandering rather than tracing the
lawnmower pattern.

**Solution.** A `nav2_core::GlobalPlanner` pluginlib plugin that interpolates a straight line
at `interpolation_resolution` and throws `NoValidPathCouldBeFound` on a lethal or unknown
cell. This is the workspace's first `ament_cmake` (C++) package alongside the four
`ament_python` ones.

**Two design decisions worth recording:**

* **Per-goal selection via `NavigateToPose.behavior_tree`, not the `planner_selector` topic.**
  The topic is global mutable state shared by every goal sender on the robot, so a sweep leg
  would silently leave `SweepStraight` selected for `executor_node`'s next `DELIVERING` goal
  and try to plan a straight line through the walls. The `behavior_tree` field travels with
  the goal and cannot leak.
* **`planner_id` must now be set explicitly on every goal.** `planner_server.getPlan()` only
  falls back to "the one loaded plugin" when `planners_.size() == 1 && planner_id.empty()`.
  Registering a second plugin therefore broke every goal that relied on the fallback — the
  robot went completely immobile. (During diagnosis I initially misread error code 201 as
  "no valid path"; **201 is `INVALID_PLANNER`, 208 is `NO_VALID_PATH`.**)

**Result:** straight-row conversion improved 61% → 91%.

*Commit: `99a5e3a`*

---

## 3. Waypoint Dedup — Two Guards on the Snapped Point

**Problem.** `_snap_to_reachable` can map two different queue points onto the *same* reachable
cell. Measured in run 18: the row waypoint `(3.96, 1.92)` and the transit immediately after it
`(4.06, 1.92)` **both snapped to `(4.04, 1.86)`**, so the second goal re-sent the robot to the
point it had just failed to reach, and stalled for another full 30 s.

Five consecutive waypoints along one row behaved this way.

**Two guards, both testing the *snapped* point** (the existing check tested the raw queue point
*before* snapping, which snapping can invalidate):

1. **Recently-failed dedup.** Remember snapped sweep points that failed; refuse to dispatch
   onto one again. `SWEEP_FAILURE_RADIUS_M = 0.30` — two Nav2 `xy_goal_tolerance`s (0.15), so
   closer than this is the same goal as far as the goal checker is concerned.
2. **Robot-proximity.** If the snapped point is where the robot already stands, mark it swept
   and skip. Observed snapping a point **0.56 m** onto the robot's own position.

**Deliberately *not* a blacklist.** Sweep waypoints have no blacklist bookkeeping by design —
a one-pass queue has no "next tick" to re-evaluate, and banning the raw point would make
coverage completion (which requires `unswept_free == 0`) permanently unreachable over one
undrivable pocket. The narrower guard is the correct shape.

**Validated against the real run-18 coordinates:**

| candidate | distance | suppressed |
|---|---|---|
| the exact duplicate | 0.00 m | yes |
| near-duplicate | 0.06 m | yes |
| next row waypoint | 0.35 m | no |
| waypoint after | 0.40 m | no |
| earlier waypoint | 0.61 m | no |

The 0.30 m radius sits cleanly between duplicates and genuine neighbours. In run 21 the guards
fired **10 times** (7 dedup + 3 already-there), saving roughly 300 s.

**Known gap.** Run 21 produced three consecutive stalls marching along one row at 0.4–0.55 m
spacing — *outside* the 0.30 m radius. Failing waypoints and legitimate neighbours sit at the
same spacing, so proximity alone cannot separate them. The correct fix is **row-level
abandonment** (N consecutive failures in a row → skip the remainder of that row), not a larger
radius.

*Commit: `4acea45`*

---

## 4. Recovery Ordering: BackUp Before Spin

**This robot cannot pivot in a tight spot, and that cannot be tuned away.** The footprint's
circumscribed radius is `sqrt(0.36² + 0.215²) = 0.419 m`, so an in-place turn needs **0.84 m
of clear diameter**. That is dominated by the 0.36 m arm reach, not by footprint padding —
removing lateral padding entirely only reaches 0.396 m.

Nav2's stock recovery `RoundRobin` is `ClearingActions → Spin → Wait → BackUp`, so it spends
two of three slots on manoeuvres the geometry forbids before reaching the one that works.
Measured at a deadlock: **Spin aborted with `Collision Ahead - Exiting Spin` 5 times**, while
rear clearance was 0.934 m and the footprint reversed 0.30 m scored cost 80 — comfortably
drivable.

Reordered to `ClearingActions → BackUp → Spin → Wait`. Backing up also *relieves* the geometry
rather than fighting it, so the subsequent Spin has a better chance.

**Critically, this had to be applied in two places.** `default_nav_to_pose_bt_xml` was never
set, so bt_navigator fell back to nav2's built-in tree: in run 18, **32 of 41 goals (78%) used
the DEFAULT tree**, and only 9 the custom sweep tree. Reordering only
`navigate_to_pose_sweep_straight.xml` would have reached 22% of goals. A new
`navigate_to_pose_backup_first.xml` is now wired as the default in `nav2.launch.py`.

**A near-miss worth recording:** the first version of these comments used ` -- ` as a dash.
**XML comments cannot contain `--`**, which broke *both* trees — including the sweep tree,
which was valid before the edit. An `xml.etree` parse check caught it; without that, the sweep
would have failed to load at runtime with an obscure BT error.

*Commit: `4acea45`*

---

## 5. Recovery-Aware Watchdog — A Negative Result, Then a Correction

This is the most instructive sequence of the week and is recorded in full.

### The legitimate complaint

A Nav2 recovery deliberately does not reduce `distance_remaining`, so the no-progress watchdog
reads a running recovery as a stall and cancels the goal — killing the very mechanism built to
break the deadlock it is reacting to. Run 18 attempted **47 recoveries** (23 spin, 19 backup,
5 wait) in a run where 19 stalls consumed **63% of the mission wall-clock**.

`NavigateToPose` feedback carries `number_of_recoveries`, so a recovery starting is directly
observable without subscribing to the behavior server.

### Attempt 1: count-based grace — **failed**

Three grace windows, one per recovery increment. Run 19 exposed the flaw immediately:
`number_of_recoveries` **also increments for the `ClearingActions` costmap-clear subtree**,
which completes instantly. Seven increments landed in ~9 s and burned every grace *before the
first real motion behaviour had started*. Counting increments was the wrong currency.

### Attempt 2: 90 s time budget — **a clear regression**

Time from the goal's first recovery, so fast clear-driven increments merely refresh the same
window. Correct in mechanism, badly wrong in magnitude:

| | run 18 (no grace) | run 20 (90 s grace) |
|---|---|---|
| stalls | 1.26/min | **0.23/min** |
| 20% coverage | 2.3 min | 6.5 min |
| 30% coverage | 5.4 min | **never reached** |
| peak | 48.6% @ 15.9 min | 27.7% @ 12.9 min |

**The stall count fell 5×, and coverage got ~2.5× worse.** Six goals consumed the whole budget
plus the watchdog — ~120 s each in a 12.9-minute run.

**The lesson: stall count was a bad proxy.** It fell largely because that time stopped being
*counted* as a stall, not because the robot got unstuck. The 30 s cancellation being "wasted"
was doing real work — abandoning a hopeless goal quickly and moving to a reachable one.

### Attempt 3: 20 s budget — **correct**

20 s is one full RoundRobin cycle with margin (clear ≈0 + BackUp 2.6 s + Spin ≈7 s + Wait
5 s ≈ 15 s), which is all this needs to do: stop a *single* behaviour being cancelled
mid-manoeuvre. Worst case per goal drops from 120 s to 50 s. Run 21 recovered 30%/40%/50%
coverage levels that run 20 never reached.

---

## 6. The Obstacle-Hugging Deadlock and the Cost-Aware Planner

### Diagnosis

At a live deadlock, with the robot stationary (2.7 cm of travel in 19 s while yaw swung 40°):

* **The robot was not physically boxed in.** Nearest lidar returns: front 0.607 m, left
  0.408 m, right 1.352 m, rear 0.934 m.
* **It was not a ghost-map failure.** 5 of 6 lethal cells within 1.2 m were corroborated by
  the live scan to within 2 cm.
* **The footprint sat on cost 253 (`INSCRIBED_INFLATED_OBSTACLE`) in 54 of 64 consecutive
  `costmap_raw` samples**, and on 254/255 in **zero**. The robot travels essentially always
  inside the inscribed band, with the real lethal cells just beyond.
* **`ObstacleFootprint` alone vetoed all 440 trajectories in 60 separate cycles.** Oscillation
  never appeared alone. So the deadlock is primarily *geometric*, not oscillation-driven.

Two hypotheses were formed and then refuted during this investigation, both recorded because
each looked convincing:

* *"Rotation is geometrically impossible here."* Computed using **odom-frame axes while the
  robot was 156° off-axis**. Re-done with proper TF, rotation was clear at every yaw.
* *"DWB must also be vetoing on 253."* Replicating DWB's 400-trajectory set against the same
  live costmap with a 254-only rule returned **400/400 valid** while DWB reported 0/440, which
  seemed to force that conclusion. **The Jazzy source refutes it:**
  `ObstacleFootprintCritic::pointCost` throws only on `LETHAL_OBSTACLE` (254) and
  `NO_INFORMATION` (255); 253 returns normally as a score.

### The actual cause

NavFn weights costmap cost only weakly and is well known for hugging obstacles and cutting
corners. DWB then faithfully follows that path into the inscribed band, and once there,
trajectories heading into the obstacle hit 254 and are vetoed while `Oscillation` bans the
reversal that would turn away. **It is a planning problem presenting as a control problem.**

### The change

Registered `SmacGrid` (`nav2_smac_planner::SmacPlanner2D`), which scores costmap cost
explicitly via `cost_travel_multiplier` (set to 3.0; library default is 2.0) and routes down
the middle of free space.

`tolerance: 0.1` and `allow_unknown: false` are matched to `GridBased` deliberately, so the
two arms differ **only in the search**, not in what counts as a legal goal — otherwise the
comparison would be confounded by the tolerance behaviour that previously let unreachable
goals "succeed" without moving.

Selection is a launch parameter (`default_planner_id`) with the **default unchanged at
`GridBased`**, plus `run_full_mission.sh --planner GridBased|SmacGrid`, so a bare run plans
exactly as before and the A/B needs no rebuild.

*Uncommitted at time of writing.*

---

## 7. Rejected Hypotheses

Every one of these was measured and rejected. They are listed so they are not re-tried.

| hypothesis | how it was refuted |
|---|---|
| NoMachine CPU starvation | Real (NX runs at realtime priority, drops DWB 20 Hz → 4.7 Hz) but stalls persisted with it killed |
| `sim_time` mistuned | A/B measured **worse**: DWB vetoes 30 → 98. Reverted to 1.7 |
| Lidar sees the collection arms | Refuted by per-sector scan analysis |
| Physical contact with an obstacle | Refuted by the user directly: the robot was not touching anything |
| Collision monitor would help | Measured **harmful**: stalls 0.82 → 1.41/min, throughput 3.1 → 1.6 m/min. Kept opt-in, off by default (commit `53be328`) |
| Battery droop causing weak turns | Fault reproduced at 15.9 V; deadband fit `1.030 × cmd − 0.003` |
| Ghost/stale costmap obstacles | 5 of 6 nearby lethal cells corroborated by live lidar within 2 cm |
| DWB vetoes on inscribed (253) | Jazzy source: throws only on 254/255 |

---

## 8. Results

All runs on hardware, same arena, `policy=fraction`. Minutes to reach each coverage level.

### The controlled A/B (same arena, same session)

| run | config | 20% | 30% | 40% | 50% | peak | stalls/min |
|---|---|---|---|---|---|---|---|
| 18 | NavFn, no recovery grace | **2.3** | **5.4** | **8.3** | — | 48.6% @15.9m | 1.26 |
| 20 | NavFn + 90 s grace | 6.5 | — | — | — | 27.7% @12.9m | 0.23 |
| 21 | SmacGrid + 20 s grace | 6.6 | 9.5 | 11.7 | **14.4** | **62.8% @21.0m** | 0.62 |

**Run 21's shape is the result that matters: slower to start, but it does not hit a wall.**
Run 18's early speed came from easy nearby frontiers; it then bogged down at ~48% in exactly
the deadlocks described in §6. Run 21 pays an early cost and keeps climbing, reaching 50% and
60% — levels run 18 never touched.

### Correction on "best on record"

During the session I described run 21 as the best run on record. **Across the full week that
is an overstatement**, and the wider table corrects it:

| run | duration | 50% | peak |
|---|---|---|---|
| `20260905-221352` | 49.3 min | 26.0 | **66.4%** |
| `20260906-161239` | 25.6 min | **14.3** | 60.2% |
| `20260906-235356` (run 21) | 21.0 min | 14.4 | 62.8% |

Run 21 is *among* the best and is the best per unit time, but `20260906-161239` reached 50%
marginally faster (14.3 vs 14.4 min) and an earlier run reached a higher absolute peak over
more than twice the duration. The run-18/20/21 comparison remains valid as a controlled A/B;
the cross-week claim was not.

### Attribution caveat

**Run 21 changes four things at once** (SmacGrid, 20 s grace, waypoint dedup, BackUp-first)
against run 18's zero. The bundle is clearly better; *which part earned it is not yet
established.* The missing experiment is **`--planner GridBased` on the current build**, which
would separate the planner from the recovery and dedup work. The `--planner` flag exists
precisely so this needs no code change.

---

## 9. Open Items

**Blocking the attribution claim**
* Run the `--planner GridBased` control on the current build.

**Known defects**
* **Row-level abandonment** (§3) — point dedup cannot catch a whole bad sweep row.
* **SmacGrid's early deficit** — 20% at 6.6 min vs 2.3. Planner startup logs
  `Inflation layer ... not set sufficiently for optimized non-circular collision checking`,
  so Smac fell back to its slow collision path, which it warns "will substantially impact
  run-time performance." Plausibly the cause and fixable.
* **rclpy shutdown race** — `action/client.py take_feedback` raises `TypeError` on Ctrl-C.
  Upstream, cosmetic. (The two shutdown errors that *were* ours — publishing on a dead context
  and double `rclpy.shutdown()` — are fixed.)
* **YOLO false positives cluster at frame edges.** The `z == 0.0` gate removed only the
  depth-less subset. Phantom detections went 5 → 0 in run 14, but the edge cluster remains.
* `rtabmap.launch.py` still needs reverting (`Reg/Strategy` back to `'1'`, drop
  `Vis/MinInliers: '15'`) — long outstanding.

**Not yet attempted**
* Swept-coverage costmap layer; coverage-aware DWB critic; sweep-specific goal checker.
* Rebalancing DWB scales — `ObstacleFootprint.scale: 0.8` against `PathAlign/PathDist: 32.0`
  means path-following outweighs stay-clear by ~40×.

**Repo state.** Commits `99a5e3a`, `53be328`, `56fd9de`, `4acea45` are on `research`. The
SmacGrid work and `RECOVERY_BUDGET_S = 20.0` are uncommitted across
`nav2_params.yaml`, `frontier_explorer_node.py`, `executor.launch.py`, `run_full_mission.sh`.
