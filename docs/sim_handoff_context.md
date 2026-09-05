# Simulation handoff — context for continuing this work on the laptop

**Written:** 2026-09-05, from the Jetson.
**Audience:** whoever (human or agent) picks up the Gazebo simulation work on a different machine.
**Why this exists:** the Jetson Orin Nano cannot run this simulation fast enough to produce
publishable numbers (measured real-time factor **0.245**). The sim campaign moves to the
laptop. This file is everything learned so far, so none of it has to be rediscovered.

Read `CLAUDE.md` in the repo root first for the general architecture. This file covers only
what is specific to simulation and to the research measurements.

---

## 1. What the research is

**Working title:** *Interleaved Exploration and Coverage for Target Retrieval under
Short-Range Perception on a Physical Ground Robot.*

**Question:** when a robot's detection range (Kinect, ~57° / ~1 m) is much shorter than its
mapping range (360° lidar, ~6 m), how should it split effort between exploring unknown space
and sweeping already-mapped space?

**Core claim:** frontier exploration terminates when the map is complete, not when the space
has been *inspected*. The robot can finish mapping a room having never pointed its camera at
most of the floor.

The contribution is an **empirical characterisation plus a policy comparison on hardware**,
not a new algorithm class. Say that plainly in the paper.

### The go/no-go gate — NOT YET ACHIEVED

The headline measurement, and a genuine stop-and-rethink gate:

> Run plain frontier exploration until the map is complete, with the camera-coverage mask
> recording but **not** influencing behaviour. Report: *of the free floor area mapped, only
> X% was ever within camera detection range.*

If X comes back high (say >70%), exploration paths already cover the floor incidentally,
there is no gap worth closing, and the premise fails. **Learn that before building more
policy arms.**

Two critical properties of this measurement:

1. **It needs no cubes in the arena at all.** X is pure exploration geometry. So the fact
   that YOLO barely fires on Gazebo's untextured boxes does not affect it.
2. **X must be read at the moment frontier exploration completes, not at run end.** In
   `exhaustion` mode the coverage sweep starts immediately after exhaustion and begins
   filling in the very gap being measured. The marker is the executor log's one-shot
   `Exploration complete — <reason>.` line.

**Status: no valid X yet.** Three attempts on the Jetson all failed to reach
`Exploration complete` inside their time budget. See §5.

---

## 2. Where the implementation stands

Phases from the agreed plan. Everything is additive and default-off: **`run_full_mission.sh`
with no arguments must behave exactly as it did before any of this work.**

| Phase | What | Status |
|---|---|---|
| 0 | Parameterise policy knobs | ✅ done, committed `dc97d37` |
| 1 | Passive metrics logger | ✅ done, committed `247eb89` |
| 1.5 | `run_sim_mission.sh` sim bringup | ✅ done, **uncommitted** |
| — | **🚦 gate — X% measurement** | ❌ **not achieved** |
| 2 | Sensor characterisation (hardware) | not started |
| 3 | Sim frustum detector | not started |
| 4 | Arm C (fixed-interval sweep) | not started |
| 5 | Experiment harness | not started |
| 6 | `run_full_mission.sh` integration | not started |

### Policy arms

Selected by one ROS parameter, so every arm runs an otherwise identical stack:

- **A — `sweep_trigger_mode:=exhaustion`** — classic explore-then-sweep baseline. Implemented.
  This is what the gate runs under.
- **D — `sweep_trigger_mode:=fraction`** (default) — interleaved; sweeps once un-swept known
  free area exceeds `sweep_fraction` (0.15). This is the shipped behaviour.
- **C — fixed-interval** — not implemented (phase 4).
- **B — utility-weighted frontier** — **deliberately dropped.** It changes *which* frontier,
  not *whether* to sweep; orthogonal to the question and costs ~30 runs.

### Parameters added in phase 0

`frontier_explorer_node`: `sweep_trigger_mode`, `sweep_fraction` (0.15),
`sweep_retrigger_cooldown_s` (20.0), `camera_half_fov_rad` (0.497), `camera_mark_range_m` (1.0).

`executor_node`: `cube_max_range_m` (1.0), `home_cube_suppress_radius_m` (1.0).

All defaults equal the previous hardcoded constants, so unparameterised launches are unchanged.

### A note on `cube_max_range_m`

Changed **1.5 → 1.0** on the user's explicit instruction, so it agrees with
`swept_mask.CAMERA_MARK_RANGE_M`. These are the same physical quantity — how far the camera
can actually find a cube — and a coverage paper cannot have "inspected" and "collectible"
meaning different distances.

**Watch for a regression:** 1.5 existed because a hardware measurement caught a real cube at
**1.09 m** being silently dropped by the old 1.0 gate. If TARGETING success falls, suspect
this first — look for `/detected_cube` messages with `1.0 < z < 1.5` that the executor
ignores. Neither value is measured; phase 2's detection-probability-vs-range curve should
set both. Both are now ROS parameters, so candidates can be tested with a launch argument.

---

## 3. How to run

### Simulation

`run_sim_mission.sh` (repo root, **uncommitted**) composes what nothing else does:
`simulation.launch.py` + `rtabmap.launch.py` + `nav2.launch.py` + `executor.launch.py`, all
at `use_sim_time:=true`, plus the metrics node. Staged bringup with readiness checks, logs to
`run_logs/sim-<timestamp>/`.

```bash
./run_sim_mission.sh --policy exhaustion --metrics --duration 900 --headless   # the gate
./run_sim_mission.sh --rviz --display :1                                       # with GUIs
```

| flag | meaning |
|---|---|
| `--policy fraction\|exhaustion` | coverage policy (arm D / arm A) |
| `--metrics` | record `run_logs/<ts>/metrics.jsonl` |
| `--layout <file>` | ground-truth cube positions (implies `--metrics`) |
| `--duration <s>` | auto-stop; omit for open-ended |
| `--headless` | no Gazebo GUI |
| `--rviz` / `--rviz-config <f>` | start RViz last (defaults to `~/botzilla_nav_debug.rviz`) |
| `--display :N` | X display for GUIs; needed when launching over ssh |
| `--no-mission` / `--build` / `--yes` | stack only / build first / clear leftovers |

**RViz in sim must have `use_sim_time:=true`** or it rejects every transform as too old and
renders an empty scene. The script does this; if you start RViz by hand, pass it.

### Hardware

`run_full_mission.sh` — unchanged by any of this work, and must stay that way. It also starts
a Fast DDS discovery server so RViz can attach from a laptop.

---

## 4. The instrument: `mission_metrics_node`

`botzilla_navigation/mission_metrics_node.py`. Writes one JSON object per line to
`metrics.jsonl`. Record types: `run_start`, `pose` (10 Hz + cumulative distance), `state`,
`coverage`, `cube_detected`, `cube_inspected` (first frustum entry per ground-truth cube),
`run_end`.

**It is subscribe-only by construction** — no publishers, no action clients, no TF
broadcasts. `test/test_mission_metrics.py` AST-parses the source and fails if any appear.
That is what makes it safe to leave running during a real hardware mission without
qualifying the results. **Keep it that way.**

`cube_inspected` vs `cube_detected` is the load-bearing distinction: the first is what the
exploration policy controls, the second is what YOLO managed to do with it. Comparing them
quantifies detector loss separately from policy quality. **Make inspection the primary
metric** — collection is gated behind YOLO gaps and TARGETING aborts (~50% on hardware) whose
failure rate would otherwise bury any policy difference.

`swept_mask.is_in_frustum()` is the single definition of "the camera could have seen this
spot", used by *both* `mark_swept_cells` (coverage map) and the metrics node (inspection
times), with a test asserting cell-by-cell agreement. Do not reimplement it.

Layout file format:

```yaml
cubes:
  - {id: 1, x: 2.0, y: 0.5}
  - {id: 2, x: -1.0, y: 1.5}
```

### ⚠️ KNOWN BUG — fix this first on the laptop

`mission_metrics_node` stamps every record with `time.monotonic()`, i.e. **wall clock**. At
RTF 1.0 that is fine. At any other RTF it silently corrupts every time-based metric —
mission completion time, time-to-first-inspection, distance per cube. **Switch it to the ROS
clock** (`self.get_clock().now()`) so it records sim time when `use_sim_time` is set. This
must be done before any run whose numbers go in the paper.

---

## 5. Why the sim moved off the Jetson

Measured on the Jetson with the full stack plus GUIs running:

```
sim time advanced : 4.90 s
wall time elapsed : 20.00 s
REAL-TIME FACTOR  : 0.245
load average      : 16.2 on 6 cores
```

**Every run was ~4× shorter than it appeared.** The 900-second "15 minute" gate attempts were
about 3.7 minutes of simulated robot time, which is very likely why exploration never reached
`Exploration complete` and why coverage plateaued.

Top consumers: two `ruby` processes (gz server + GUI) at ~67% and ~58% of a core, then
`frontier_explorer` 50%, `controller_server` 42%, `executor` 33%.

Running `--headless` frees the GUI's share and should get RTF to roughly 0.35–0.4. Still not
good enough for measurement. **On the laptop, measure RTF first** (see §8) and do not plan a
run count until you know it.

The Jetson remains the right place for *hardware* runs and for interactive debugging — two
real bugs were found there today precisely because the GUIs were visible.

---

## 6. Bugs found and fixed (all uncommitted as of writing)

### 6.1 velocity_smoother reversal deadband — froze the robot in open floor

**Symptom:** robot stops dead in the middle of clear floor and appears to oscillate left and
right. No obstacle anywhere near it.

**Root cause:** `velocity_smoother.py` collapses an opposing command smaller than
`REVERSAL_DEADBAND_THETA` (0.05 rad/s) straight to zero as "dithering". That threshold was
sized against the **physical** Kobuki's measured mechanical deadband of **0.121 rad/s** — on
hardware, discarding such a command is free because the wheels would not have turned anyway.

**Simulation has no mechanical deadband.** Gazebo delivers 0.021 rad/s exactly. DWB emits
alternating ±0.021 rad/s heading corrections, every one is collapsed to zero, the robot never
turns, the heading error never shrinks, and DWB emits the same correction forever. Deadlock.

Caught live: `target=(0.000,-0.021) -> output=(0.000,0.000)`, with 2.37 m of clear floor
ahead, **zero** `ObstacleFootprint` vetoes (0/9828 forward trajectories), and cost 0 across
the whole footprint.

**Fix:** `reversal_deadband_x` / `reversal_deadband_theta` are now ROS parameters defaulting
to the hardware constants (0.025 / 0.05). `nav2.launch.py` passes
`SIM_REVERSAL_DEADBAND_X = 0.002` / `SIM_REVERSAL_DEADBAND_THETA = 0.005` **only** when
`use_sim_time` is true. Hardware behaviour is unchanged by construction.

Verify at startup: `Reversal deadbands: x=0.002 m/s theta=0.005 rad/s`.

**This reduced but did not eliminate stalls** — 6 × `Failed to make progress` still appeared
in the first ~7 minutes after the fix. There is at least one more cause.

### 6.2 URDF modelled the robot 55 mm shorter than reality

The Nav2 footprint is **correct and measured** — `nav2_params.yaml`'s `local_costmap` comment
records `0.48 m from the front of the grabber arms to the back of the chassis, 0.33 m across`
→ `front x=+0.310, rear x=-0.170, half-width 0.165`, plus a documented `+0.05 m padding on
all four sides` (2026-08-09, after the arms clipped a wall). Measured + padding = exactly the
polygon in use: `[[0.36, 0.215], [0.36, -0.215], [-0.22, -0.215], [-0.22, 0.215]]`.

The **URDF** was wrong: arms at `x=0.18` with length 0.15 reached only **0.255 m**, i.e. the
simulated robot was 55 mm shorter than the real one. Gazebo showed clearance the hardware
would not have had, so any sim result about squeezing past obstacles would not transfer.

**Fix:** arm joints moved `x=0.18 → 0.235`, so the tip lands at 0.310 m — the measured value.
Rear (−0.17) and width (±0.17 vs 0.165) already matched. `nav2_params.yaml` untouched.

### 6.3 Depth-as-laserscan stream was effectively dead (cubes invisible)

**Symptom:** robot drives into cubes. The 2D lidar sits at `laser_frame` z=0.24 and sweeps
clean over a 0.1 m cube, so low obstacles are supposed to come from the depth camera via
`pointcloud_to_laserscan` → `/scan_camera` (same config in sim and hardware).

**Measured:** `/camera/points` at **0.1 Hz** — one cloud every 12 seconds — and `/scan_camera`
with it. At 0.2 m/s the robot covered **2 m between obstacle updates**.

**Cause:** the URDF's `rgbd_camera` was `640×480 @ 30 Hz` = 307,200 points/cloud (~5–10 MB),
roughly 300 MB/s through `ros_gz_bridge` if it kept up. The Jetson cannot.

**Fix:** `320×240 @ 15 Hz` — 8× less traffic. Ample to see a 0.1 m cube at 0.5–3.0 m, and
YOLO letterboxes its input so cube *detection* is unaffected. Sim only.

**Result: 0.1 → 0.42 Hz wall, i.e. ~1.7 Hz sim-time against 15 Hz configured.** Better, still
short. The bridge remains a bottleneck *on top of* the RTF problem. **Re-measure on the
laptop** — this may resolve itself entirely there, and if so consider restoring 640×480 for
fidelity with the real Kinect.

### 6.4 NoMachine starves Nav2 (environmental, not code)

NoMachine's virtual session runs at **realtime priority** and starves Nav2's control loop.
With it running: 48 × `No valid trajectories out of 440!`. With it killed: **0**. Load average
dropped 10.75 → 1.04.

**Kill it before any measurement run** (needs sudo; processes are owned by `root`/`nx`):

```bash
sudo pkill -f nxnode.bin; sudo pkill -f nxexec; sudo pkill -f nxserver.bin
```

---

## 7. Open problems

| # | Problem | Notes |
|---|---|---|
| 1 | **`mission_metrics_node` uses wall clock** | §4. Fix before any paper numbers. |
| 2 | **Stalls not eliminated** | 6 × `Failed to make progress` post-fix. At least one more cause. |
| 3 | **`run_sim_mission.sh` teardown fails** | Observed **all 22 processes surviving** a clean SIGINT — Gazebo, RTAB-Map, full Nav2 stack. A leftover `gz sim server` holds the world and breaks the next spawn. Had to kill by explicit PID. **Real bug in that script.** |
| 4 | **RTAB-Map accepts zero loop closures** | The `map` frame is drift-accumulating odometry. The inspected-area mask is only as good as the pose. Quantify drift or fix, before publishing coverage numbers. |
| 5 | **Docstrings claim `LimitedAccelGenerator`** | Both `nav2.launch.py` and `velocity_smoother.py` say it is selected. Live parameter is `dwb_plugins::StandardTrajectoryGenerator` (`nav2_params.yaml:152` deliberately leaves it unset). Docs describe a change not in effect. |
| 6 | **Gazebo YOLO** | `simulation.launch.py` starts `yolo_node` with no `confidence` parameter → default 0.8 → **detects nothing in sim** (`yolo_node.py:74` says so explicitly). Sim needs ~0.25. On the Jetson it also dies outright: `ModuleNotFoundError: No module named 'ultralytics'` (hardware runs YOLO in a GPU container). Does not block the gate. |
| 7 | Kobuki serial framing | Hardware task. Sync on real `0xAA 0x55`, verify XOR checksum. ~1.3 desyncs/min, all caught by the plausibility filter. |
| 8 | TARGETING aborts ~33–50% | Hardware. YOLO detection gaps (measured 6+ s with zero `/detected_cube`). |

---

## 8. Traps and disproved hypotheses

**Do not re-chase these.** Each was measured and killed.

| Hypothesis | How it died |
|---|---|
| `vx = 0` is never sampled by DWB (asymmetric range `[-0.10, 0.20]`, 20 samples) | `OneDVelocityIterator` **explicitly injects zero** (`return_zero_` logic, `one_d_velocity_iterator.hpp:61`) |
| Local plan truncated by slow `map→odom`, making `RotateToGoal` fire early | The `No valid trajectories` failures vanished entirely once NoMachine was killed. Resource starvation, not config. |
| `distance_remaining` is a frozen constant, so the stall watchdog always fires | It changes — 12 distinct values across 88 samples, range 1.491–1.662 |
| Phantom footprint nose causes the stalls | Real (55 mm, now fixed) but **not** the stall cause: 0 vetoes during an actual stall |
| `/scan_camera` paints false obstacles | It was publishing **0/58 finite returns** — contributing nothing |
| Height filter (0.05–0.30 m) excludes the cubes | **37,231** cloud points fall inside the band |
| Velocity floors (`min_speed_xy` / `min_speed_theta`) regressed | Both live at 0.0 |
| Velocity smoother is at fault (first check) | Sampled during a *healthy* moment where `output == target`. **This one was actually right** — re-checked during a stall and caught it zeroing commands. Sample during the failure, not around it. |

**Method note:** the last row is the important lesson. Several of these were checked while
the robot was behaving and passed. The veto rate, the smoother output, and the costmap all
look completely different during a stall than 30 seconds either side of it. **Sample during
the failure.**

### Measuring real-time factor

Do this first on any new machine. Subscribe to `/clock`, compare its advance to wall time:

```python
sim = last_clock - first_clock
wall = time.time() - w0
rtf = sim / wall
```

Anything below ~0.8 makes wall-clock timing metrics untrustworthy and makes the run-count
planning fantasy.

---

## 9. What to do first on the laptop

1. **Install and build**: ROS 2 Jazzy + Gazebo Harmonic (8.11 on the Jetson), then
   `colcon build --symlink-install` in `botzilla_Workspace/`. Needs `ultralytics` in the
   Python environment if you want YOLO in sim.
2. **Measure RTF** (§8). Everything else depends on it.
3. **Fix `mission_metrics_node`'s clock** (§4) — one-line change, invalidates results if skipped.
4. **Fix `run_sim_mission.sh` teardown** (§7 #3) — otherwise leftovers silently corrupt the next run.
5. **Run the gate**:
   ```bash
   ./run_sim_mission.sh --policy exhaustion --metrics --duration 1800 --headless
   ```
   Read `swept_fraction` at the `Exploration complete` marker, **not** at `run_end`.
   Run it at least twice — a single Gazebo run can get an unlucky exploration order.
6. **If X is high, stop and rethink.** That is what the gate is for.

---

## 10. File map for this work

| Path | Role |
|---|---|
| `run_sim_mission.sh` | sim bringup (**uncommitted**) |
| `run_full_mission.sh` | hardware bringup (**uncommitted**) — do not change behaviour |
| `botzilla_navigation/mission_metrics_node.py` | the instrument |
| `botzilla_navigation/swept_mask.py` | `is_in_frustum`, coverage mask |
| `botzilla_navigation/frontier_explorer_node.py` | policy; `sweep_trigger_mode` |
| `botzilla_navigation/executor_node.py` | mission FSM |
| `botzilla_navigation/config/nav2_params.yaml` | **footprint is measured and correct** |
| `botzilla_navigation/config/nav2_params_sim.yaml` | sim overlay (currently a documented no-op) |
| `botzilla_navigation/launch/nav2.launch.py` | `SIM_REVERSAL_DEADBAND_*` |
| `botzilla_control/velocity_smoother.py` | reversal deadband parameters |
| `botzilla_bringup/description/botzilla_qbot.urdf` | robot model + Gazebo sensors |
| `docs/ghost_map_sensor_latency.md` | prior investigation — read before touching sensor timestamping |
| `docs/ghost_map_investigation.md` | prior investigation — mirrored scan |

### Uncommitted changes at handoff

```
 M .gitignore
 M botzilla_Workspace/src/botzilla_bringup/description/botzilla_qbot.urdf
 M botzilla_Workspace/src/botzilla_control/botzilla_control/velocity_smoother.py
 M botzilla_Workspace/src/botzilla_navigation/launch/nav2.launch.py
?? run_full_mission.sh
?? run_sim_mission.sh
```

Commit these before or during the move, or the laptop will not have the fixes.
