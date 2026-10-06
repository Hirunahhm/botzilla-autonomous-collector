# Multi-robot (leader + collector) hardware runs

Branch `multi-robot-test`. Design: `docs/multi_robot.md`.

## Setup common to every run

| | Leader (BotZilla) | Collector (`/bz2`) |
|---|---|---|
| Computer | Jetson Orin Nano | Raspberry Pi 5, 4 GB |
| Role | explore and detect only (`--detect-only`), fleet manager | collect assigned cubes, deliver to its own HOME |
| Localisation | RTAB-Map SLAM | AMCL in the leader's map (`/bz2/fleet/map` from run 6 on) |
| Cube detection | YOLO on the Jetson GPU | JPEG frames to a second YOLO container on the Jetson, depth paired on the Pi |
| Start | taped spot, map origin | beside the leader on its left, facing the same way, 0.3 m gap between bodies |

- **Search:** `--strategy interleaved --inspection mixed`, 10 min from the leader's HOME latch (`tools/timed_run.sh fleetN 10 --fleet ...`).
- **Cubes:** placed in the lab by hand, no layout file.
- **Network:** both robots on the same phone hotspot, Fast DDS discovery server on the Jetson.
- **Collector start offset:** `--start 0.0 0.73 0.0` in runs 1–8. From run 9 it is `0.0 0.64 0.0`, because the leader lost its arms and is now a 0.34 m circle; the collector is 0.33 m wide.

Counts below come from the logs: the leader's `run_logs/<run>/fleet.log` and `nav2.log`, and the collector's `run_logs/collector-<time>/collector.log` on the Pi.

- **Delivered** = released at HOME.
- **Released short** = captured, but the drive home failed, so the cube was released on the spot. The collector reverses as part of every release.
- **Footprint hits** = DWB rejecting all trajectories because the footprint touches a lethal cell.

## Results

| # | Leader run | Commit | Collector up | Tasks | Delivered | Released short | Times of deliveries (s after start) | Leader: plan fails / back-ups / spins refused / footprint hits |
|---|---|---|---|---|---|---|---|---|
| 1 | 20261005-184223 | 91e2397 | never (Nav2 aborted) | 8 | 0 | 0 | — | 14 / 2 / 4 / 33 |
| 2 | 20261005-212043 | fe7bb9a | yes, late (restarted) | 0 | 0 | 0 | — | 14 / 2 / 1 / 8 |
| 3 | 20261005-214048 | 4b098b6 | yes | 11 | 2 | 6 | 199, 323 | 29 / 4 / 6 / 42 |
| 4 | 20261005-221125 | 4e85742 | yes | 10 | 3 | 1 | 322, 392, 600 | 36 / 7 / 7 / 42 |
| 5 | 20261006-002941 | 1dc6975 (+ local edit) | yes | 6 | 2 | 0 | 315, 452 | 8 / 40 / 76 / 1081 |
| 6 | 20261006-005735 | 8ceb16a | after 6 min (script bug) | 5 | 3 | 0 | 437, 513, 577 | 14 / 103 / 202 / 626 |
| 7 | 20261006-181824 | e796a11 | yes | 5 | 0 | 4 | — | 20 / 69 / 138 / 706 |
| 8 | 20261006-183558 | b8cb5a1 | yes | 10 | 3 | 6 | 251, 346, 441 | 24 / 21 / 48 / 564 |
| 9 | 20261006-190324 | e72fa88 | yes | 7 | 2 | 2 | 306, 440 | 1 / 35 / 68 / 1018 |
| 10 | 20261006-224929 | 112a718 | yes | 6 | 2 | 4 | 76, 171 | 16 / 14 / 22 / 399 |

**Totals over runs 3–10, where the collector actually worked:** 60 tasks, 17 cubes delivered, 23 released short.

**Notes on the table:**
- **Run 7** was stopped early: both robots were deadlocked (see below).
- **Run 10** was stopped at about 8.5 min.
- **Run 1:** the collector never came up, so the tasks were rejected instantly.
- **Run 5** is the second attempt. The first, `20261006-002604`, aborted at start-up when the leader's Nav2 bond timed out.

Collector-side counters, per collector log:

| # | Collector log | Captures | At HOME | Short | Plan fails | Back-ups | Spins refused | Planner ack time-outs | HOME re-sends |
|---|---|---|---|---|---|---|---|---|---|
| 1 | collector-20261005-184906 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 2 | collector-20261005-212446 | 0 | 0 | 0 | 6 | 1 | 0 | 0 | 0 |
| 3 | collector-20261005-214049 | 8 | 2 | 6 | 53 | 11 | 2 | 16 | — |
| 4 | collector-20261005-221125 | 4 | 3 | 1 | 50 | 8 | 0 | 5 | — |
| 5 | collector-20261006-002942 | 2 | 2 | 0 | 57 | 9 | 4 | 0 | 0 |
| 6 | collector-20261006-010415 | 3 | 3 | 0 | 10 | 1 | 0 | 3 | 1 |
| 7 | collector-20261006-181825 | 4 | 0 | 4 | 34 | 4 | 14 | 0 | 5 |
| 8 | collector-20261006-183558 | 9 | 3 | 6 | 2 | 5 | 16 | 12 | 10 |
| 9 | collector-20261006-190325 | 4 | 2 | 2 | 0 | 4 | 7 | 1 | 0 |
| 10 | collector-20261006-224930 | 6 | 2 | 4 | 2 | 1 | 2 | 2 | 0 |

The HOME re-send retry did not exist before run 5.

## What each run found, and what changed after it

### Run 1
- **Problem:** the collector's Nav2 never became active. On the busy Pi, AMCL took longer than the lifecycle manager's 4 s bond timeout, so the whole bring-up was aborted.
- **Second problem:** the collector still reported IDLE. The leader assigned all 4 cubes, and each route was rejected in 0.1 s, so all 4 were marked failed.
- **Fixes:**
  - collector `bond_timeout` raised to 20 s;
  - the collector reports IDLE only once Nav2 is fully active;
  - the leader no longer counts a collector-side fault against a cube.

### Run 2
- **Problem:** the collector's `bt_navigator` waited only 1 s for the planner's action server while loading its tree. On the Pi that wasn't enough, so Nav2 aborted again.
- **Fix:** `wait_for_service_timeout` raised to 10 s.
- **Also:** the leader drove into cubes it had already detected. **Fix:** known cubes are published as obstacles for the leader's costmaps.
- **Also:** the readiness check in `run_collector.sh` listened for only 0.5 s. **Fix:** 5 s.

### Run 3: first cubes delivered
- **Result:** the collect chain worked end to end on hardware: the collector camera via the Jetson GPU, then TARGETING, APPROACHING, CAPTURING.
- **Problem:** every delivery planned from the collector's start position was refused. **Cause:** the leader's LiDAR had mapped the parked collector itself as an obstacle in `/map`. So the collector started inside a lethal blob, and its HOME was that same blob.
- **Fix:** `/bz2/fleet/map`, the leader's map with a 0.5 m disc around the collector's HOME and current pose forced free.
- **Problem:** the leader's cleanup stalled. The zero-`/cmd_vel` publish failed, and the leader kept moving until killed by hand. **Cause:** `kobuki_base_node` zeroed the motors only on Ctrl-C.
- **Fixes:**
  - the base driver zeroes on every exit;
  - new `tools/robot_stop.sh`: stop everything that commands velocity, let the base watchdog trip, then stop the base driver;
  - Docker stops get time limits;
  - the cleanup ignores repeated signals.

### Run 4
- **Result:** 3 delivered.
- **Problem:** many routes to the standoff failed.
- **Fixes:**
  - only a goal that Nav2 *rejects* counts as a collector fault;
  - clearer failure detail ("released short of HOME" instead of "Delivery complete.");
  - **footprint marks:** the robots' footprint marks (point clouds in the obstacle layers) lingered after the robots moved. They were replaced by a costmap layer repainted in full on every message: `botzilla_coverage_layer` in a new lethal mode.
- **Problem:** a delivery failed 30 ms after capture. `bt_navigator`'s 20 ms acknowledgement timeout ran out before the Pi's planner answered.
- **Fixes:**
  - the timeout raised to 1 s;
  - a HOME goal that fails in under 2 s is re-sent up to 3 times before the cube is released.

### Run 5
- **Problem:** the first attempt aborted at start-up, on two separate bring-up timeouts:
  - the leader's `bt_navigator` bond took just over 4 s;
  - the collector's planner hung waiting for a map, because it started before the fleet manager.
- **Fixes:**
  - leader `bond_timeout` raised to 20 s;
  - `run_collector.sh` waits for `/bz2/fleet/map`;
  - the collector stays in STARTUP until its map arrives.
- **Problem:** the standoff chosen was inside the parked leader.
- **Fix:** the standoff chooser also rejects candidates that are lethal in the collector's live costmap.
- **Problem:** in the local costmaps, the other robot was drawn offset by the map→odom correction. The layer ignored the grid's frame.
- **Fixes:**
  - the C++ layer transforms through TF;
  - robot marks shrunk to the real footprint with no margin;
  - cubes moved to their own layer after inflation, with a soft halo, because inflated cubes blocked corridors.

### Run 6
- **Problem:** the collector waited 6 min for a map that was already published. **Cause:** under `pipefail`, `grep -q` exited at the first match, `ros2 topic list` then died of SIGPIPE, and the test failed.
- **Fix:** don't use `grep -q` there.
- **Result:** once the collector was up, it delivered 3 cubes in under 4 min.

### Run 7: robots deadlocked
- **Problem:** the robots met 0.36 m apart. Each was inside the other's footprint, inflated by 0.45 m, so neither could plan. When checked mid-run, the leader had 16 failed plans, 29 back-ups and 32 spins; the table has the totals for the whole run. Every delivery was released short.
- **Fix:** the other robot is drawn exactly, as lethal cells with a fading 0.5 m soft halo, **after** inflation. Cubes get the same treatment with a 0.35 m halo. Both go in one grid per robot.

### Run 8
- **Result:** no planning failures on either robot.
- **Problem:** 6 deliveries released short. Causes: the Pi planner failing to acknowledge in time (12 time-outs in the collector log, even at 1 s), and DWB finding no valid trajectory right after a capture.

### Run 9: new hardware
- **Change:** the leader is now a bare circular Kobuki and the collector carries the arms.
  - leader Nav2 `robot_radius: 0.22`;
  - the collector gets back the padded arm polygon via `params_rewrite`;
  - the fleet manager draws the leader as a 0.17 m circle and the collector as a 0.48 × 0.33 m rectangle.
- **Problem:** the leader was stuck for the last ~4 min, with about 210 footprint hits per minute. In RViz it sat on a cube mark (pink) while in reality it was only beside the cube. A cube *estimate* had ended up under the robot, and a lethal cell inside a footprint makes Nav2 refuse every move.
- **Fix:** each robot's own Nav2 footprint is cleared in its own obstacle grid.

### Run 10
- **Result:** 2 cubes delivered in the first 3 minutes. The leader got through a tight spot by itself.
- **Problem:** two cubes about 0.5 m apart, at (0.10, 2.35) and (−0.44, 2.54), trapped the collector. Each capture ended with no valid trajectory or a refused spin while carrying, and the cube was released back into the same cluster. This repeated four times.
- **Remaining causes:**
  - the carrying tree allows only a spin;
  - the Pi planner's acknowledgement time-outs;
  - registry positions that go stale once a cube is released short.

## Open issues after run 10

- **Delivery while carrying in tight spots.** The carrying behaviour tree's only recovery is a spin. When the spin is refused, the cube is released. A short forward-and-turn manoeuvre, or a brief wait for the costmap to clear, might save some.
- **The Pi planner is too slow to acknowledge goals.** Proposed: a 5 s acknowledgement timeout, and republishing `/bz2/fleet/map` every 5 s instead of about every 1 s, to cut the Pi's costmap load.
- **Cubes released short.** The collector should report where it released a cube, so the registry entry moves there instead of keeping the pre-capture estimate.
- **Low obstacles seen by the camera are only in the local costmap,** so the planner keeps routing into cubes that the controller then refuses. Proposed: add `scan_camera` to the global costmap.
- **Base drivers need forcing to exit at shutdown,** on both robots, every run. This is safe, because the watchdog has already stopped the motors by then.
- **The URDF still has arm links on the leader,** which only affects the robot model in RViz.

## Fixes made after run 10 (one per open issue)

1. **Delivery while carrying in tight spots.**
   - The carrying behaviour tree now waits 2 s after clearing the costmaps, before its spin.
   - A HOME goal that genuinely ran and then failed is tried again up to 2 more times, each after holding still for 3 s (`DELIVERY_STUCK_RETRIES`, `DELIVERY_STUCK_WAIT_S` in `executor_node.py`). Only then is the cube released.
2. **The Pi planner is too slow to acknowledge goals.**
   - `bt_navigator` `default_server_timeout` raised to 5000 ms on the collector.
   - `/bz2/fleet/map` is republished at most every 5 s (`COLLECTOR_MAP_PERIOD_S`), and immediately once the collector's HOME is first known.
3. **Cubes released short.** `CollectorStatus` carries `last_released`, `release_x` and `release_y`. The fleet manager moves that cube's registry entry to the release point (`CubeRegistry.relocate`, logged as a `relocated` FLEET event).
4. **Camera-seen cubes in the global costmap.** `scan_camera` is now a source in the global costmap's obstacle layer too, with the same settings as in the local costmap. Note that this also applies to single-robot runs on this branch.
5. **Base drivers needing forcing at shutdown.** **Cause:** `KobukiDriver`'s serial reader was a non-daemon thread looping forever, so the process could never exit after `main()` had zeroed the motors. **Fix:** it is now a daemon thread.
6. **Arm links on the leader.** `hardware.launch.py` has a new `arms` argument, default `false`, which strips the four `arm_*` links and joints from the published model. `collector.launch.py` passes `arms:=true`.
