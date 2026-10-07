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
| 11 | 20261006-232154 | 6be6770 | no (aborted at start-up) | 0 | 0 | 0 | — | 0 / 0 / 0 / 8 |
| 12 | 20261007-004945 | de39f8d | yes | 12 | 0 (3 effectively) | 5 | — | 2 / 39 / 79 / 1144 |
| 13 | 20261007-010955 | 98b056f | yes | 9 | 2 | 1 | 135, 310 | 0 / 51 / 103 / 1607 |
| 14 | 20261007-013717 | dfa4e60 | yes | 5 | 2 | 2 | 271, 499 | 16 / 33 / 55 / 839 |
| 15 | 20261007-101042 | 7996c0e | yes | 2 | 0 | 1 | — | 0 / 24 / 48 / 720 |
| 16 | 20261007-102125 | b21b2e1 | yes | 2 | 0 | 2 | — | 10 / 34 / 66 / 980 |
| 17 | 20261007-173130 | 65ec1c0 | yes | 5 | 2 | 0 | 121, 366 | 0 / 19 / 29 / 441 |
| 18 | 20261007-183105 | 11bc3fb | yes | 1 | 0 | 0 | — | 9 / 1 / 0 / 2 |
| 19 | 20261007-183841 | 11bc3fb | yes | 1 | 0 | 0 | — | 0 / 0 / 0 / 9 |
| 20 | 20261007-184342 | 11bc3fb | yes | 6 | 4 | 0 | 100, 246, 333, 481 | 3 / 0 / 0 / 7 |
| 21 | 20261007-200547 | 11bc3fb | yes | 8 | 2 | 1 | 417, 575 | 6 / 11 / 22 / 357 |
| 23 | 20261007-210202 | d0d6356 | yes | 8 | 2 | 1 | 369, 495 | 8 / 5 / 9 / 139 |
| 24 | 20261007-211701 | d0d6356 | yes | 4 | 0 | 1 | — | 0 / 2 / 3 / 26 |
| 25 | 20261007-212525 | d0d6356 | yes | 5 | 0 | 1 | — | 2 / 2 / 0 / 5 |
| 26 | 20261007-213304 | d0d6356 | yes | 5 | 2 | 2 | 112, 311 | 15 / 2 / 0 / 2 |
| 27 | 20261007-221643 | d0d6356 | yes | 7 | 3 | 0 | 138, 271, 440 | 5 / 6 / 8 / 130 |

**Totals over runs 3–10, where the collector actually worked:** 60 tasks, 17 cubes delivered, 23 released short.
**Runs 12–16:** 30 tasks, 4 delivered (7 counting run 12's three releases at HOME), 11 released short. Every run from 13 on lost most of its time to the two robots blocking each other.

**Notes on the table:**
- **Run 7** was stopped early: both robots were deadlocked (see below).
- **Run 10** was stopped at about 8.5 min.
- **Run 1:** the collector never came up, so the tasks were rejected instantly.
- **Run 5** is the second attempt. The first, `20261006-002604`, aborted at start-up when the leader's Nav2 bond timed out.
- **Runs 11–16:** delivery times are measured from the leader's HOME latch, so they may run a few seconds later than the earlier rows (run 10 recomputed this way gives 84, 180).
- **Run 15** was killed at 6.6 min and **run 16** at 6.4 min, both with the robots stuck together.
- **Runs 23 and 27** ran the full 10 min; **runs 24, 25 and 26** were stopped at 6.8, 5.2 and 5.8 min. Run 22 was cancelled before any data (see below), so it has no row.
- **Runs 18 and 19** were stopped after 2.3 and 1.6 min because the collector looked like it was moving too fast (see below); neither got as far as a delivery. **Run 20** and **run 21** ran the full 10 min. Run 21's one release short came as the run ended, mid-delivery, so the leader logged no result for it; its 8 tasks include 3 handed back to a stuck leader.
- **Run 17** ran the full 10 min. From it on, the collector starts with a **1 m gap** to the leader's left (`--start 0.0 1.34 0.0`, centres 1.34 m apart) instead of 0.3 m, so its HOME is no longer next to the leader's start. 2 of its 5 tasks were handed back to the leader as "gave way to a stuck leader" (collector faults, not cube failures).

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
| 11 | collector-20261006-232155 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 12 | collector-20261007-004946 | 5 | 0 | 5 | 3 | 0 | 9 | 1 | 6 |
| 13 | collector-20261007-010955 | 3 | 2 | 1 | 0 | 19 | 37 | 0 | 2 |
| 14 | collector-20261007-013717 | 5 | 2 | 2 | 4 | 2 | 0 | 0 | 0 |
| 15 | collector-20261007-101042 | 2 | 0 | 1 | 4 | 16 | 37 | 4 | 0 |
| 16 | collector-20261007-102126 | 2 | 0 | 2 | 0 | 13 | 15 | 0 | 2 |
| 17 | collector-20261007-173254 | 2 | 2 | 0 | 0 | 8 | 14 | 0 | 0 |
| 18 | collector-20261007-183232 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 19 | collector-20261007-183945 | 0 | 0 | 0 | 0 | 1 | 1 | 0 | 0 |
| 20 | collector-20261007-184445 | 4 | 4 | 0 | 0 | 1 | 1 | 0 | 0 |
| 21 | collector-20261007-200721 | 3 | 2 | 1 | 1 | 1 | 4 | 0 | 2 |
| 23 | collector-20261007-210304 | 3 | 2 | 1 | 0 | 8 | 9 | 1 | 1 |
| 24 | collector-20261007-211808 | 2 | 0 | 1 | 0 | 1 | 2 | 3 | 4 |
| 25 | collector-20261007-212623 | 1 | 0 | 1 | 0 | 0 | 0 | 0 | 0 |
| 26 | collector-20261007-213403 | 4 | 2 | 2 | 1 | 0 | 3 | 1 | 4 |
| 27 | collector-20261007-221753 | 3 | 3 | 0 | 2 | 4 | 4 | 0 | 0 |

The HOME re-send retry did not exist before run 5. From run 11, "HOME re-sends" counts the stuck-delivery retries ("holding still 3s and trying again"). Yields (leader right of way, from run 14): 2 in run 14, 14 in run 15, 13 in run 16, 11 in run 17, 2 in run 19, 2 in run 20, 12 in run 21, 6 in run 23, 5 in run 24, 1 in run 25, 0 in run 26 and 7 in run 27 (of which clear-outs for a STUCK leader: 1 in run 19, 2 in run 20, 9 in run 21, 2 in run 23, 4 in run 24, 1 in run 25, 4 in run 27). Collector logs before run 18 have no clear-outs: they did not exist yet.

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

## Runs 11–16

### Run 11
- **Problem:** aborted at start-up. The collector started before the fleet manager was publishing `/bz2/fleet/map`: the readiness check only looked for the topic in the graph, and the laptop's RViz, subscribed to it, had already made it appear.
- **Fix:** `run_collector.sh` waits for an actual map message (`ros2 topic echo --once`, transient local). The fleet manager also exits cleanly on an external shutdown.

### Run 12: deliveries miscounted
- **Result:** 0 delivered by the count, but 3 of the 5 "released short" cubes were left 0.05–0.17 m from the collector's HOME: delivered in all but name. The last few centimetres failed because the leader kept hanging around that HOME, among the cubes already dropped there.
- **Worse:** each of those was relocated *onto* HOME and stayed pending, so it became an obstacle on HOME and the collector was later sent to collect cubes from its own drop zone.
- **Fixes:**
  - a delivery that fails within 0.5 m of HOME counts as delivered (`DELIVERY_CLOSE_ENOUGH_M`);
  - a cube released inside the HOME zone is marked collected, not relocated (`collected_in_home_zone` event);
  - a lethal "drop zone" disc around the collector's HOME in the leader's grid, to keep the leader away.

### Run 13
- **Result:** 2 delivered.
- **Problem:** the robots kept meeting: 19 back-ups and 37 refused spins on the collector, 1607 footprint hits on the leader.
- **Fix:** leader right of way. An unladen collector backs off 0.3 m when the leader comes within 0.9 m (if moving) or 0.6 m, and waits for it to pass; the leader is drawn with 0.10 m of padding and a longer halo in the collector's grid.

### Run 14
- **Result:** 2 delivered; the collector yielded twice.
- **Problem:** the leader moved in short bursts and got stuck inside the soft halos around the collector and around cubes.
- **Fix:** no halos in the leader's grid at all, only lethal bodies; drop-zone radius 0.45 → 0.25 m.

### Run 15
- **Problem:** both robots pinned near the start. The collector's HOME is only 0.64 m from the leader's start, so the drop-zone disc and the collector together trapped the leader whenever it came back there. Killed at 6.6 min.
- **Fix:** drop zone disabled (`DROP_ZONE_ENABLED = False`); its job is covered by the yield and the close-enough rule.

### Run 16: the two robots wedged together
- **Result:** 0 delivered; killed at 6.4 min.
- **What happened** (leader poses from `metrics.jsonl`, collector log on the Pi):
  1. 10:23:36: the leader confirmed a cube at (2.35, −0.01) while standing 0.4 m from it, turning to inspect it. It was assigned 1 s later.
  2. The collector drove in and captured it right beside the leader. Spinning in place did not count as "moving", and the leader was more than 0.6 m away, so the collector did not yield. From 10:23:51 the leader was in Nav2 recovery almost continuously (about 220 footprint hits a minute).
  3. Carrying, the collector could not get past the leader: no progress towards HOME for 45 s, then released short.
  4. 10:24:47: it yielded to the leader at 0.72 m. Seven seconds later it decided the leader was "parked" (it was barely moving because it was stuck, not parked), resumed, drove in for the cube again and captured it. Every delivery attempt then had its spin refused.
  5. From 10:25:20 the robots were 0.38 m apart, centre to centre: the leader's centre 0.32 m to the side of the collector's, so the bodies were touching or within about a centimetre. The collector tried to yield 13 more times; every back-up was refused at once ("Collision Ahead"), and each yield ended as "leader parked".
- **Lesson:** shrinking footprints would not have helped (the bodies were really that close). The failure is in how the robots interact: the leader was given no time to leave a cube before it was assigned, and the collector could not tell a stuck leader from a parked one.

## Fixes made after run 16

1. **No assignment while the leader is beside the cube.** The fleet manager holds a confirmed cube back while the leader is within 1 m of it (`LEADER_CUBE_CLEAR_M`, `deferred` event) and offers it once the leader has moved on. If the leader stays PARKED there for 30 s (`LEADER_DEFER_MAX_S`), the cube is offered anyway. The leader only ever confirms cubes within 1 m, so otherwise a leader that stops for good would hold them for ever. Never while it is turning or stuck.
2. **The leader says what it is doing.** `botzilla_fleet/leader_state.py` classifies the leader from its pose and its Nav2 `number_of_recoveries`. The fleet manager publishes the state on `/bz2/fleet/leader_state` and logs each change (`leader_state` event):
   - **MOVING:** driving, or turning (inspection spins count);
   - **STUCK:** Nav2 recovered recently and the leader has not got 0.25 m away since, or it has held a goal for 8 s without moving;
   - **PARKED:** neither.
3. **The collector never plans past a stuck leader.**
   - It yields at 0.9 m to a MOVING or STUCK leader, and only inside 0.6 m to a PARKED one.
   - The "leader parked; planning past it" exit only fires for a leader that really is PARKED.
   - With a STUCK leader it holds clear for up to 30 s, giving the leader's recoveries room, then gives the task back as a collector fault (the cube is not marked failed). If it is already beyond 0.9 m it gives the task back after 5 s, since a stuck leader will not pass.
   - The 10 s cooldown no longer applies inside 0.6 m unless the leader is PARKED.
   - Without a `leader_state` message (an older fleet manager) it falls back to the position-only guess.
4. **Tested** with unit tests (`test_leader_state.py`, `test_cube_registry.py`) and four `fake_fleet.py` scenarios: the default slow turn (3/3 delivered), an orbiting leader (3/3), `LEADER_MODE=inspect` (the cube is held back until the spinning leader leaves, then delivered) and `LEADER_MODE=stuck` (the collector gives way, never plans past the stuck leader, and delivers both reachable cubes on the second attempt).

### Run 17: 1 m gap, fixes after run 16
- **Result:** 2 delivered, **0 released short** (both captures reached HOME), and the robots never wedged together. Cubes were held back near the leader 3 times (16 s, 84 s and 59 s). Only 3 cubes were confirmed in 10 min, because the leader was stuck for about half the run, in two spells:
  1. **150–310 s, beside the idle collector.** The leader was at about (−0.6, 1.8), 0.75 m from the collector parked IDLE at its HOME (0, 1.34), with open floor around it. It started moving again at 313 s, 4 s after the collector drove off on a task. Backing off 0.3 m at a time did not get the collector out of the way.
  2. **From about 451 s to the end, next to cube 3** (223 and 183 footprint hits a minute; the collector was 1.2 m+ away, then idle). The leader's centre was 0.25 m from the cube's estimate: its 0.22 m footprint and the 0.05 m half-width of the cube mark overlap by 2 cm, so every move was refused. Same family as run 9.
- **Also:** the leader's state flipped STUCK → PARKED in the gap between the explorer cancelling a stuck goal and sending the next one, so the collector "planned past" twice seconds after the leader had been STUCK (201 s, 241 s); it re-yielded within about 2 s each time. RTAB-Map again mapped the parked collector into `/map` at its HOME.

## Fixes made after run 17

Each is in `botzilla_fleet`; the pure logic is unit-tested (62 tests) and every rule has a `fake_fleet.py` scenario.

1. **STUCK is sticky** (`leader_state.py`). Once STUCK, the leader stays STUCK until it has moved 0.25 m from where it got stuck, or 15 s pass with nothing stuck about it (`STUCK_RELEASE_S`).
2. **No cube marks touching the leader** (`LEADER_STATIC_CLEAR_M`). In the leader's own grid, cube marks (and the drop zone) are not drawn within 0.35 m of its centre. The collector's mark is never relaxed like this: it moves, and the leader must not creep into it.
3. **Clear-out instead of a 0.3 m step** (`collector_node.py`, `standoff.choose_clear_spot`). When the leader is STUCK within 0.9 m, an unladen collector drives (Nav2) to a clear spot at least 1.4 m from it, chosen on rings around itself. A yield that started as a plain step is upgraded the moment the leader turns STUCK. If the leader is still stuck once the collector is clear, it hands the task back rather than resume the same route past it. A STUCK leader also gets a wider soft halo (1.1 m) in the collector's grid, so its planner keeps out of the leader's recovery room when there is space.
4. **Escape when touching** (`right_of_way.EscapeLatch`). While the collector's footprint overlaps the leader's mark, the leader's body is not drawn in the collector's grid (its halo still is), so Nav2 can move the collector off. It is drawn again once they are 0.10 m clear. Only the collector is released; the leader keeps the collector's mark. Logged as `escape` / `escape_over`.
5. **The leader gives way to a carrying collector** (`right_of_way.GiveWay`). While the collector is CAPTURING, DELIVERING or DETACHING within 1.2 m, the fleet manager pauses the leader's exploration on `/exploration_enabled` (the same switch the executor uses), and resumes it once the collector is 1.6 m away or no longer carrying, after 30 s at most, with a 15 s cooldown. Logged as `leader_gives_way` / `leader_resumes`. The fleet manager re-enables exploration if it shuts down while it has the leader paused.
6. **The leader's route is shared** (`right_of_way.path_ahead`). The next 1.5 m of the path the leader is following (DWB's `/received_global_plan`, not `/plan`, which also carries the explorer's candidate-frontier queries) is drawn in the collector's grid as soft cost, never lethal, while the leader is MOVING. The collector's planner then routes around where the leader is going, not just where it is.
7. **Drop zone back on**, now that the collector's HOME is 1.34 m from the leader's start; never drawn within 0.35 m of the leader, so it cannot trap it as in run 15.
8. **Each robot filtered out of the other's scans** (`fleet_scan_filter_node.py`, `body_filter.py`). In a two-robot run the LiDAR and the camera's virtual scan publish `scan_raw` / `scan_camera_raw` (new `scan_topic` / `scan_camera_topic` arguments in `hardware.launch.py`), and the filter republishes `scan` / `scan_camera` with returns on the other robot's body (+0.10 m) set to NaN, timestamps untouched. RTAB-Map, Nav2 and AMCL are unchanged. On the leader it is started by `run_full_mission.sh --fleet` (log `scan_filter.log`), on the collector by `collector.launch.py` (`scan_filter:=false` turns it off). If the other robot's pose is more than 1 s old, scans pass through unfiltered. The fleet layer is then the only place each robot sees the other: exact, current and uninflated, and the leader no longer maps the collector. **Not yet checked on hardware**: in RViz, the other robot should vanish from `/scan` and the costmaps' obstacle layers and appear only in the fleet layer; the filter logs its removal counts every 30 s. With the filter on, the collision monitor (off by default) would not see the other robot either.
9. **Harness.** `fake_fleet.py` now: accepts cancels like Nav2 (a cancelled route used to keep driving, which hid chase failures); keeps a just-captured cube visible as a blind-spot detection for 1 s; routes its fake Nav2 around the leader (standing in for the planner and halos); obeys `/exploration_enabled`; publishes the leader's route. New `LEADER_MODE=stuckhome` (run 17's first stall: the leader stuck 0.4 m beside the collector's HOME for 60 s). Results, 200 s each: stuckhome 3/3, orbit 3/3, turn 3/3, inspect 1/1, stuck 1 delivered + 1 held back beside the permanently stuck leader (by design).

## Runs 18–20 (commit 11bc3fb: the fixes after run 17)

Same setup as run 17: 1 m gap, `--start 0.0 1.34 0.0`. From run 18 the Pi resolved the Jetson's `.local` name unreliably, so the collector was started with `--leader 10.156.103.192`.

### Run 18 (stopped at 2.3 min)
- **Stopped** because the collector seemed to move very fast. It had taken task 1 and was in TARGETING (turning to centre the cube) for 37 s without centring: the detections jumped across the frame (x = −0.84, +0.53, +0.95), possibly two cubes in view.
- **Scan filter:** the leader removed 254 collector returns from `/scan` in its first full 30 s window. The collector's first window, during start-up, counted only 87 scans (about 3 Hz) passed through unfiltered (no pose yet), which raised the question below.

### Run 19 (stopped at 1.6 min)
- **Stopped** for the same reason, while the collector was yielding to the leader (0.89 m) on its first task.
- **Measured on the Pi for the first time** (a load sampler every 5 s and a scan-rate counter):
  - **The collector's LiDAR is not slow:** 10.1 Hz on `scan_raw` and 10.2 Hz on `scan` after the filter, once running. The 3–4 Hz in runs 18 and 19 was the C1 spinning up.
  - **The Pi is heavily loaded:** load 0.04 idle, 5.3 one minute after the collector started (4 cores), 23% CPU idle, 60 °C, never throttled, about 2.4 GB free. Top users: `kinect_bridge` 45%, the Kobuki driver 36%, `collector_node` 36%, Nav2's controller 27%, the LiDAR driver 18%, the depth pipeline 18%. The scan filter was not among them.

### Run 20: 4 delivered, the leader barely stuck
- **Result:** 6 tasks, 4 delivered (at 100, 246, 333 and 481 s), 0 released short; the last two tasks (548 s and 558 s) failed as "Cube lost while approaching" (not yet investigated). 5 cubes confirmed in 10 min (run 17: 3).
- **The leader was hardly stuck:** 7 footprint hits, 0 back-ups and 0 refused spins all run (run 17: 441 / 19 / 29; run 16: 980 / 34 / 66). No task was handed back to a stuck leader (run 17: 2).
- **The coordination rules all fired:** the leader paused 4 times for a carrying collector (`leader_gives_way`), the collector cleared out twice for a STUCK leader, there was 1 escape from overlapping, 1 cube was held back near the leader, and the collector never planned past a stuck leader.
- **Scan filter** worked on both robots (for example, 450 collector returns removed from the leader's `/scan` in one 30 s window).
- **Pi load** (120 samples): median **6.3**, peak 8.4 on 4 cores; at most 61.5 °C and never throttled.
- **Collector speed:** a probe logged, every second, the largest commanded speed (`/bz2/cmd_vel`) and wheel-odometry speed. Commands never exceeded **0.20 m/s and 0.40 rad/s**, so nothing upstream (Nav2, the collector's own chase, the yield moves) asked for speed. The odometry maxima reached 0.74 m/s and 1.42 rad/s, but that twist is the driver's raw per-packet `d/dt`, which its own comments describe as spiky, and this run looked normal; so those maxima are noise, not evidence of overshoot.
- One run; runs vary a lot (compare runs 10 and 16), so this needs repeating before it counts as the new normal.

### Run 21: the leader chased the collector
- **Setup:** same as run 20 (commit 11bc3fb), after a Pi reboot; NoMachine killed on the Jetson first.
- **Result:** 8 tasks, 2 delivered (417 s, 575 s), 1 released short as the run ended, 3 handed back as "gave way to a stuck leader" (at 0.38, 0.29 and 0.42 m), 2 other failures ("route to the standoff failed", "cube not seen from the standoff"). Leader footprint hits 357, mostly 20:09–20:11 (about 130 a minute) and 20:14 (62).
- **The chase.** From about 20:09 to 20:11 the leader followed the collector across the arena: each clear-out put the collector on the far side of the leader's *position*, which was ahead of it on its own route, so the leader drove on, caught up, and got stuck again: leader (0.82, 1.24) → (−0.15, 1.61) → (−0.57, 0.51) → (−1.38, −1.09) while the collector cleared out to (−0.25, 1.46) → (−0.74, 0.60) → (−0.97, −0.50) → (−1.86, −1.85), into a corner; 9 clear-outs in all.
- **The escape stayed on while the collector drove.** The escape latch (leader not drawn while overlapping) stayed on after the collector had switched back to GOING; with the scan filter also removing the leader from its scans, the collector had no sign of the leader at all, and drove back to 0.32 m from it (20:10:31).
- **"Leader parked; planning past it" fired 4 times** right after STUCK spells (the sticky STUCK releases after 15 s with no recoveries, and the leader's explorer can sit that long between goals while still boxed in), and each time the collector drove straight back towards it.
- **Speed (new probe, from odometry position change):** the collector's real speed never exceeded what it was commanded: at most 0.23 m/s (commanded 0.20) and 0.52 rad/s (commanded 0.40); 0 seconds over 0.25 m/s or 0.6 rad/s. No "too fast" this run.
- **Pi load:** median 6.5, peak 9.4 on 4 cores; at most 64.2 °C, never throttled.

## Fixes made after run 21

1. **Clear-out keeps off the leader's route** (`standoff.choose_clear_spot`, `fleet_manager_node` `ROUTE_FOR_COLLECTOR_M`). The fleet manager sends the next 4 m of the leader's route (DWB's `received_global_plan`, in every state, STUCK included: that is where it will go once free) to the collector on `/bz2/fleet/leader_route`. Clear-out spots must be at least 0.7 m off it; if none qualifies, it keeps off only the next ~1.5 m of route, and only then ignores the route. The straight line to the spot may not come closer to the leader than the collector already is (or 0.45 m).
2. **Escape only while YIELDING** (`right_of_way.EscapeLatch`). The leader's body is left out of the collector's grid only while the collector is moving off for it; in every other state it is drawn, so a GOING collector always sees it.
3. **No "planning past" after a clear-out** (`collector_node`). Once the collector has cleared out for a STUCK leader, that yield ends only with the leader clear (over 1.3 m), or by handing the task back ("gave way to a STUCK/PARKED leader"), never by "leader parked".
4. **Tested:** 67 unit tests; harness scenario `LEADER_MODE=chase` (the leader drives a route through the collector's HOME and is blocked while the collector is within 0.55 m). Against the previous code (11bc3fb): leader blocked 18 s, 3 clear-outs, each further along its route. With the fixes: blocked about 1 s, 2 clear-outs, and the leader completes its route. The other five scenarios are unchanged: stuckhome 3/3, orbit 3/3, turn 3/3, inspect 1/1, stuck 1 delivered + 1 held back.

### Run 22 (cancelled)
- Commit e471747 (the fixes after run 21). The Pi's first build had not installed the new `botzilla_fleet` (its installed `standoff.py` was still the old one), so it was rebuilt before the run. Started at 20:49:44 and cancelled about a minute after the collector came up, to deal with the Pi's load first. No data worth recording.

## Fixes made after run 22: the Kinect's load on the Pi

- **`kinect_bridge` `fps` parameter** (`hardware.launch.py` `camera_fps`, default 30; `collector.launch.py` passes 15). The Kinect always streams 30 fps, and every frame used to be copied and converted in the `freenect` callbacks (RGB 0.9 MB, depth to 16-bit and float metres, 1.8 MB) whether or not it would be published. Surplus frames are now dropped in the callbacks, before any conversion. The leader keeps 30 (every frame, as before). The collector's consumers use about 8 fps (remote YOLO, `RELAY_HZ`) and 5 Hz (the local costmap, via the depth cloud), so 15 leaves a margin. Remote detection still pairs every RGB frame with depth: the nearest kept depth frame is at most about 33 ms away (50 ms with jitter), inside its 60 ms tolerance.
- **Measured on the Pi** (`kinect_bridge` alone, registered depth as on the collector, with stand-in subscribers for its three streams, 20 s CPU sample):

| fps | `kinect_bridge` CPU | consumers' CPU | together |
|---|---|---|---|
| 30 (before) | 42.8% | 45.0% | ~88% |
| 15 (collector) | 31.8% | 21.0% | ~53% |
| 10 | 27.8% | 16.0% | ~44% |

  About a third of a core freed at 15 fps. The bridge does not go much below ~28%: libfreenect still receives every frame from USB (the Kinect v1 cannot stream RGB slower at 640×480).
- **Also fixed:** `fps:=15` typed as an integer on the command line used to crash the node (declared as a float); it now accepts either.
- **Still to check in a run:** the Pi's overall load (median 6.3–6.5 in runs 20–21) and that TARGETING / APPROACHING behave the same with 15 fps camera input.

## Runs 23–27 (commit d0d6356: the Kinect at 15 fps on the collector, plus the fixes after run 21)

Same setup as runs 20–22 (1 m gap, `--start 0.0 1.34 0.0`, collector started with `--leader 10.156.103.192`). Every collector log confirms "Kinect Bridge Started at 15 fps". The Pi load sampler and a speed probe ran on the Pi in every run; from run 24 the probe also logs the gyro's turn rate (`/bz2/imu`) next to the wheel encoders' every second.

| Run | Length | Delivered | Pi load median / max (4 cores) | Peak temp | Over-speed seconds | Gyro/wheel disagreements |
|---|---|---|---|---|---|---|
| 20 (30 fps, for comparison) | 10 min | 4 | 6.3 / 8.4 | 61.5 °C | — | — |
| 21 (30 fps) | 10 min | 2 | 6.5 / 9.4 | 64.2 °C | 0 | — |
| 23 | 10 min | 2 | **4.2** / 7.3 | 60.4 °C | 28 | not logged |
| 24 | 6.8 min | 0 | **3.6** / 7.0 | 60.4 °C | 0 | 0 |
| 25 | 5.2 min | 0 | **4.2** / 8.5 | 60.4 °C | 0 | 0 |
| 26 | 5.8 min | 2 | **3.2** / 6.7 | 60.4 °C | 0 | 0 |
| 27 | 10 min | 3 | **4.6** / 8.0 | 60.9 °C | 0 | 0 |

- **Pi load:** median 3.2–4.6 with the Kinect at 15 fps, against 6.3–6.5 at 30 fps (runs 20–21). Never throttled. Still above 4 cores at peaks, so the other proposals (decimated depth cloud, Kobuki driver priority, lighter Nav2 controller) remain open.
- **Run 27: 3 delivered, 0 released short**, all 3 captures reaching HOME; 2 tasks handed back to a stuck leader, 1 "cube lost while targeting". The leader had one stuck spell (22:22–22:23, about 80 then 50 footprint hits a minute; 130 in the run). 3 cubes held back near the leader, 4 clear-outs, 3 escapes, the leader paused twice for a carrying collector.

### Run 23: the collector turned much faster than commanded
The speed probe caught what had looked "very fast" in runs 18 and 19. Commands were always normal; the base itself turned too fast:

| When | Commanded | Actual (wheel encoders) | Motor PWM |
|---|---|---|---|
| 21:04:21, backing 0.3 m off the leader (BackUp, 7 s) | 0.10 m/s straight, **0 rad/s** | pivoting at **0.5–0.6 rad/s** (about 200° in all), 0.06 m/s back | 53 → 75 |
| 21:05:22, approaching a cube (15 s) | 0.15 m/s, **0.35 rad/s** | **0.7–1.05 rad/s** (about two full circles) | 70–84 |
| then, delivering | Nav2's **0.40 rad/s** | **0.8–0.9 rad/s** | up to 84 |

Normal PWM is about 25. Checked and ruled out: the gyro's start-up calibration (−0.18 °/s, normal, and similar to run 21's −0.20 °/s), and its in-run bias tracking (it only adapts when the wheels are still *and* the command is zero). A blocked wheel fits the back-off spin (one wheel stopped and the other at ~0.14 m/s gives exactly 0.6 rad/s and 0.07 m/s), but not 15 s of steady over-turning while driving forward. Unresolved: the probe did not yet log the gyro. It does from run 24, and in runs 24–27 (about 24 minutes of collector driving) there was **no over-speed and no gyro/wheel disagreement**.

## Still open after run 27

- **Run 23's over-turning** has not recurred, so its cause is unknown (a wrong gyro reading in the base driver's speed loop, or a mechanical one). Proposed protection either way: the base driver stops the motors and logs a warning if the gyro and encoder turn rates disagree by more than ~0.3 rad/s, or it spins while commanded not to, for about half a second. It is the driver both robots use, so `docs/velocity_control_fix.md` should be read first.
- **Reversing is blind** on the collector (the camera faces forward; the LiDAR at 24 cm misses low objects such as cubes), and the yield's 0.3 m BackUp reverses. A forward or sideways clear-out would avoid that.
- **Pi load** peaks still exceed 4 cores (see above).
- **"Cube lost while targeting / approaching"** is now the most common collector failure (runs 20, 23–25, 27).
- **A route that has to pass close to a stuck leader** (a narrow arena) will keep handing the task back; the collector delivers nothing until the leader recovers. Safe, but slow.
- **The leader's control loop** misses its 20 Hz rate about as often as in earlier runs (median about 16 Hz when it does), with two YOLO containers on the Jetson.
