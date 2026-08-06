# Week 5 Progress Report: Milestone 5 Executor, GPU Perception, and End-to-End Hardware Validation

This document summarizes Week 5 of the Semester 5 Autonomous Multi-Cube Search & Collection
Robot project. The headline result is the **first complete end-to-end mission on real
hardware**: the robot detected a cube, aligned, captured it, navigated back to its recorded
starting position using Nav2, released the cube, and resumed exploring — fully autonomously.

Week 5 also involved substantial debugging of the navigation and perception stacks. Several
of the defects found were *silent* — the system reported itself healthy while doing nothing —
and those are documented in detail below, because the diagnostic method mattered more than
the fixes.

---

## 1. Milestone 5 Accomplishments: Unified Autonomy Executor (`executor_node.py`)

Week 3 closed with Milestone 5 as the only remaining Phase 1 objective. It is now implemented
as `botzilla_navigation/executor_node.py`, replacing the role of the legacy `brain_node.py`.

### A. Mission State Machine

```
STARTUP      latch HOME = map->base_link once TF is available
EXPLORING    frontier_explorer_node drives; interrupt on a real cube detection
TARGETING    rotate in place to centre the cube (P-control on detected_cube.x)
APPROACHING  drive in, holding centre, until the cube enters the depth blind spot
CAPTURING    keep pushing while the cube is still seen, then a short blind push
DELIVERING   NavigateToPose(HOME)
DETACHING    reverse to release the cube
             -> back to EXPLORING for the next one
```

### B. Delivery to the Recorded Start Position (Design Change)

The original Phase 1 plan delivered cubes to an **AprilTag-marked drop-off zone**. This was
revised: the robot now **records its own starting pose at launch** and delivers every cube
back to that point. This removes the AprilTag entirely from the delivery path, along with
`apriltag_node`'s failure modes (tag not visible, tag mirrored on a phone screen, etc.).

The critical design decision was **which coordinate frame HOME lives in**:

* The legacy `final_test_node.py` navigated home to **odom `(0, 0)`**, which is free because
  odometry starts at zero by definition — but wheel odometry drifts. After a long exploration
  run, odom `(0, 0)` can be metres away from the true start. This project has already lost
  time to precisely this failure class (the Week 3 "ghost map" investigation, resolved by
  making the EKF fuse IMU yaw).
* `executor_node` instead latches the **`map` → `base_link`** transform at startup. Because
  the `map` frame is maintained by RTAB-Map's pose graph, SLAM loop closure continuously
  corrects HOME, and Nav2 plans a genuine route back rather than dead-reckoning.

Startup deliberately **blocks exploration until HOME is latched**, so the robot cannot drive
away before recording where it began.

### C. Arbitration: Preventing Two Controllers Fighting for `/cmd_vel`

Both the frontier explorer (via Nav2) and the executor (via direct P-control) can command the
base. Two controllers publishing to `/cmd_vel` simultaneously is worse than either alone, so
explicit arbitration was added:

* A latched `/exploration_enabled` (`std_msgs/Bool`) topic hands control back and forth.
* `frontier_explorer_node` gained a subscription to it. On disable it **actively cancels its
  in-flight `NavigateToPose` goal** (rather than merely going idle, which would leave Nav2
  still publishing velocities) and bumps its epoch counters so late action callbacks cannot
  resurrect stale state.
* The executor publishes `cmd_vel` **only** in `TARGETING` / `APPROACHING` / `CAPTURING` /
  `DETACHING`. During `EXPLORING` and `DELIVERING`, Nav2 owns the base and the executor stays
  completely silent.

Verified in the hardware log: `frontier_explorer_node: Paused (exploration disabled by
executor_node)` appears for the entire duration of cube collection and delivery.

### D. Capture Logic (Depth Blind-Spot Handling)

The Kinect cannot measure closer than ~$0.55\text{m}$, so a cube stops being reported well
before it is physically between the arms. The capture routine was carried over from
`final_test_node.py` — the version already proven on hardware — rather than `brain_node.py`'s
cruder fixed-duration blind drive:

| | `brain_node.py` | Adopted (`final_test_node.py`) |
|---|---|---|
| Blind-spot trigger | 1 frame of `z == 0.0` | **2 consecutive** frames (debounced) |
| During capture | Fully blind, fixed $2.5\text{s}$ | Keeps steering on the cube while visible |
| Final push | Fixed duration regardless | Measured from **cube actually lost**, then a short push |
| Range gate | none | ignores implausibly distant detections |

---

## 2. GPU-Accelerated Perception in a Container (`docker/yolo/`)

`yolo_node` could not run on the Jetson at all — neither `torch` nor `ultralytics` was
installed, and the node died at import with `ModuleNotFoundError: No module named
'ultralytics'`. Installing a CUDA-enabled PyTorch directly on JetPack 7 / L4T r39 is awkward,
so perception was containerised instead.

### A. Image Design

Built on `nvcr.io/nvidia/pytorch:25.08-py3` — the NGC container already validated during
Jetson setup — which provides arm64 CUDA PyTorch on Ubuntu 24.04, exactly the platform ROS 2
Jazzy targets. ROS 2 Jazzy (`ros-base` + `cv_bridge`) and Ultralytics are layered on top.

Ultralytics is installed with **`--no-deps`**, which is load-bearing: a plain
`pip install ultralytics` pulls a generic PyPI `torch` wheel with no Jetson CUDA support and
would silently replace NVIDIA's build, dropping inference to CPU.

Verified in-container: `torch 2.8.0a0`, `cuda available: True`, `ultralytics 8.4.115`.

### B. The `libucs` Shadowing Defect

`import torch` failed inside the built image with:

```
ImportError: /opt/hpcx/ucc/lib/libucc.so.1: undefined symbol: ucs_config_doc_nop
```

Root cause: an apt dependency chain (`gdal` → `hdf5` → `libopenmpi3t64` → `libucx0`) installed
Ubuntu's `libucx0`, whose `/usr/lib/aarch64-linux-gnu/libucs.so.0` **shadowed** the HPC-X copy
shipped in the base image. HPC-X's `libucc.so.1` could then not resolve its symbol, and since
PyTorch links UCC for distributed support, the import died — with an error naming a library
nobody had asked for. Diagnosed with `apt-cache rdepends --installed libucx0`. Resolved by
forcing HPC-X's own libraries to the front of `LD_LIBRARY_PATH` in the container entrypoint.

---

## 3. Silent Failures Found and Fixed

Three defects this week shared a dangerous property: **the system reported itself healthy
while transporting no data or producing wrong data.** None raised an error.

### A. `velocity_smoother` Silently Dropping All Commands

`controller_server` published to `/cmd_vel_nav` continuously, `velocity_smoother` reported
lifecycle state `active [3]`, logged nothing, and **`/cmd_vel` had zero publishers**. The robot
could not move under Nav2 at all. Ruled out process death, lifecycle state, topic wiring, and
QoS before concluding the node itself was at fault.

**Fix:** removed `velocity_smoother` from the pipeline entirely. It is optional — DWB's own
`max_vel_*` / `acc_lim_*` limits in `nav2_params.yaml` are *identical* to what the smoother was
configured to enforce, so nothing was lost. `controller_server` now publishes straight to
`/cmd_vel`.

**Result:** `/cmd_vel` went from 0 messages to 1590+, and the robot physically moved 5.58 m.

### B. DDS Transport Not Crossing the Container Boundary

With `--network host`, DDS **discovery worked perfectly** — the container could enumerate every
host node and topic, and the host counted the container as a subscriber. But **zero data
crossed**. `yolo_node` sat at "Waiting for video stream..." indefinitely while `kinect_bridge`
published at $17.5\text{Hz}$.

FastDDS had concluded the host participants were on the same machine (shared network
namespace) and negotiated its **shared-memory transport**, whose segments do not cross the
container boundary.

The diagnostic that cracked it was subscribing to `/odom` (a tiny message) alongside the
images: **both were 0**. That ruled out message size and `/dev/shm` sizing, isolating transport
negotiation as the cause. Bind-mounting `/dev/shm` did *not* fix it; forcing UDP-only via a
FastDDS XML profile did.

| Test | Result |
|---|---|
| Before fix | `odom=0  scan=0  image=0` |
| After UDP-only profile | `odom=384  scan=77  image=224` |

### C. Depth Encoding Mismatch Between Simulation and Hardware

`/camera/depth/image_raw` carries **different formats in sim and on hardware**:

* **Hardware**: `kinect_bridge` rescales the Kinect's native 11-bit disparity to `mono8`.
* **Simulation**: the Gazebo bridge passes depth through as `32FC1` **already in metres**.

`get_depth_at()` unconditionally applied the Kinect inverse-scaling formula. Applied to metric
float data this returns a *plausible-looking but wrong* distance — a silent corruption that
would have made the FSM misjudge every approach. `yolo_node` now branches on the message
encoding (`mono8` / `32FC1` / `16UC1`). Both branches have since been confirmed live.

---

## 4. Navigation Tuning (`nav2_params.yaml`)

### A. Footprint Geometry and the Point-Robot Planner

`NavfnPlanner` is a **point-robot** planner with no footprint awareness. With the polygon
footprint, the costmap derived two different radii:

$$r_{\text{inscribed}} = 0.157\text{m} \qquad r_{\text{circumscribed}} = 0.275\text{m}$$

NavFn only hard-refuses gaps narrower than $2 r_{\text{inscribed}} = 0.314\text{m}$, while the
robot actually sweeps up to $0.550\text{m}$. **Every gap in that band was a path the planner
promised and the controller could not execute.**

Replacing the polygon with `robot_radius: 0.275` makes inscribed $=$ circumscribed, so the
costmap blocks within $0.275\text{m}$ of any obstacle and NavFn cannot route through a gap
under $0.55\text{m}$. Physical measurement confirmed the real robot with arms attached is
~$40\text{cm}$ front-to-back, comfortably inside the $0.43\text{m}$ modelled footprint, so
$0.275\text{m}$ is correct.

### B. Planner Tolerance — Fake "Successes"

With `tolerance: 0.5`, an unreachable goal did **not** fail. NavFn silently substituted
"nearest reachable cell within $0.5\text{m}$", which was frequently where the robot already
stood. `follow_path` then completed in ~$0.1\text{s}$ without moving and reported `SUCCEEDED`.

Measured live: the robot sat pinned at $(-2.571, -0.021)$ for 8 seconds while **four
consecutive goals to a target $0.527\text{m}$ away all "succeeded"** in 0.07–0.18 s each. The
frontier was never cleared, so the explorer re-selected it indefinitely.

This is worth recording as a methodological lesson: a raw `SUCCEEDED` count is not evidence.
**Goal duration must be sanity-checked against distance** — a $0.5\text{m}$ traverse cannot
complete in $0.08\text{s}$. An earlier "95% success rate" reported during this week's testing
was later retracted for exactly this reason.

Reduced to `tolerance: 0.1` so unreachable targets fail honestly and the existing blacklist
can route around them.

### C. Other Changes

* **`allow_unknown: false`** — NavFn was treating unknown cells as traversable and producing
  confident paths through never-sensed territory. During exploration every frontier target is
  roughly half-surrounded by unknown space (measured: 6/12/11 unknown cells in a 5×5 window
  around three live targets), so the planner routinely routed through the unexplored side.
* **`local_costmap` 3×3 m → 5×5 m** — 52% of DWB's rejected trajectories were
  `BaseObstacle/Trajectory Goes Off Grid`, i.e. discarded for leaving the map rather than for
  hitting anything. After the change: **zero** off-grid rejections.
* **`use_sim_time` default `true` → `false`** in `nav2.launch.py` — on hardware nothing
  publishes `/clock`, so a stray `true` freezes ROS time and every wall-timer-driven Nav2 node
  silently stops firing.

### D. Environmental Finding: Remote Desktop Starves the Control Loop

While debugging navigation, DWB's control loop was found running at **4.7 Hz against a 20 Hz
target**. The cause was not the robot software: a connected NoMachine session spawns its
virtual desktop at **realtime priority**, which preempts `controller_server`. With NoMachine
and RViz running, load average reached **18.34** on 6 cores; killing the session brought it to
**~2.1** with the loop holding 20 Hz and zero rate misses.

Navigation timing results are therefore only valid with the remote desktop disconnected — RViz
should be run on a separate machine over `ROS_DOMAIN_ID`.

---

## 5. End-to-End Hardware Validation

Full mission executed autonomously on the physical robot:

```
HOME latched at x=0.109 y=-0.442 yaw=91.0deg in "map"
[STARTUP]     -> [EXPLORING]
Cube detected (x=+0.07, z=1.09m) — suspending exploration to collect it
[EXPLORING]   -> [TARGETING]
[TARGETING]   -> [APPROACHING] | Centred (x=+0.028). Driving in.
[APPROACHING] -> [CAPTURING]   | Blind spot confirmed.
Cube left the frame — final blind push.
Cube captured — delivering to HOME.
[CAPTURING]   -> [DELIVERING]
Sending NavigateToPose to HOME (0.11, -0.44)
[DELIVERING]  -> [DETACHING]   | Arrived HOME.
Cube released. Total delivered: 1
[DETACHING]   -> [EXPLORING]   | Delivery complete.
```

Nav2 reached HOME and reported `SUCCEEDED` after working through several `Failed to make
progress` recovery cycles, completing the return leg in ~75 s.

### Perception Calibration from Hardware Measurement

Two thresholds were found to be empirically wrong and corrected against real data:

* **Detection confidence $0.8 \rightarrow 0.5$.** A genuine cube scored **0.797** — failing the
  threshold by $0.003$. At $0.5$ there is a wide margin: the next-highest (spurious) box in the
  same frame scored only $0.138$.
* **`CUBE_MAX_RANGE_M` $1.0 \rightarrow 1.5$.** The cube ranged at **$1.09\text{m}$** and was
  being silently discarded by the range gate while YOLO reported it confidently every frame.
  The $1.0\text{m}$ value was inherited from `final_test_node`'s scripted small-arena search
  and is too tight for open exploration.

The model is also markedly weaker on **simulated** cubes (peak confidence 0.10–0.25 on
Gazebo's flat-shaded boxes, versus 0.797 on a real cube) because `best.pt` was trained on
photographs. Confidence is therefore exposed as a ROS parameter rather than hardcoded.

---

## 6. Known Limitations and Next Steps

### A. Frontier Exploration Stalls (Open)

Autonomous exploration does not yet complete an arena. The cause is now precisely understood
and is **geometric, not a planner bug**:

A frontier cell is *by definition* a free cell adjacent to unknown space, which almost always
places it against a wall or an unexplored boundary. With `robot_radius: 0.275`, the costmap
marks everything within $0.275\text{m}$ of an obstacle as lethal, so the frontier cell itself
is nearly always unreachable — the robot's **centre** cannot legally occupy it. Every target
returns `error_code=208` (`NO_VALID_PATH`) until the blacklist is exhausted.

**Planned fix:** target a *reachable vantage point near* the frontier rather than the boundary
cell itself. Driving close enough to **see into** the unknown is what reveals new map;
physically occupying the boundary cell is unnecessary. This is a contained change to
`frontier_detection.py`, testable with the existing pure-function unit test suite (currently
11 passing tests, no ROS dependencies).

### B. Smaller Outstanding Items

* Bake `confidence: 0.5` in as the `yolo_node` default — it is currently only a runtime
  override, so a default container launch still uses the too-tight $0.8$.
* Wire a `depth_image_proc` node on hardware so the local costmap's depth observation source
  (`/camera/points`) exists. Currently the costmap silently falls back to lidar-only, meaning
  obstacles outside the single horizontal scan plane — desk edges, table overhangs — remain
  invisible to obstacle avoidance.
* Re-tune the capture push duration. The current value was calibrated against YOLO at
  ~$0.75\text{fps}$ on a Raspberry Pi 5; GPU inference on the Jetson is far faster, so the
  post-detection push now covers considerably less ground than when it was set.

### C. Phase 1 Status

| Milestone | Status |
|---|---|
| M1 — RTAB-Map SLAM | Complete (Week 3) |
| M3 — Nav2 goal following | Complete (Week 3), retuned Week 5 |
| M4 — Frontier exploration | Implemented; **reachability fix outstanding** |
| M5 — Unified executor | **Complete and validated on hardware** |

The Phase 1 success criterion — explore, detect, collect, deliver, resume — has been
demonstrated end to end on the physical robot for a single cube. Closing the frontier
reachability gap is what remains for a fully unattended multi-cube run.
