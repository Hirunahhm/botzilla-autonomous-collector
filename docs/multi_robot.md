# Two-robot setup: leader searches, collector collects

PROJECT.md §9, as built on branch `multi-robot-test`.

| | Leader (BotZilla) | Collector |
|---|---|---|
| Computer | Jetson Orin Nano (`hirunahhm.local`) | Raspberry Pi 5, 4 GB (`groot.local`) |
| Namespace | none (unchanged single-robot stack) | `/bz2`, TF on `/bz2/tf` |
| Localisation | RTAB-Map SLAM, builds `/map` | AMCL in the leader's live `/map`, own RPLIDAR |
| Job | explore and detect (`--detect-only`), allocate cubes | drive to a cube, collect it, deliver it to its HOME |
| Started by | `./run_full_mission.sh --fleet [research options]` | `./run_collector.sh --start X Y YAW` |

## How it works

```
leader                                             collector (/bz2)
──────                                             ────────────────
yolo → /detected_cube ─┐
                       ▼
            fleet_manager_node ── /bz2/fleet/task (CubeTask, latched) ──► collector_node
            (cube_registry.py)  ◄── /bz2/fleet/status (CollectorStatus) ── │
                       │                                                   ├─ Nav2 + AMCL
rtabmap → /map ────────┼──────────────────────────────────────────────────►┘ (/map only)
                       └─ /fleet/cubes (RViz markers)
```

- **Discrete goals only, never velocities** (PROJECT.md §6). The collector plans and drives itself. If the network drops, it finishes or fails its current task on its own. If its status stops for 10 s, the leader puts the task back in the pool.
- **Registry (`botzilla_fleet/cube_registry.py`).**
  - A cube becomes a task only after 2 sightings within 0.4 m of each other.
  - Detections are ignored within 1 m of the collector's HOME, so delivered cubes don't come back as tasks.
  - Detections are also ignored within 0.8 m of the collector, which may be pushing that cube.
  - The next task is the cube with the fewest failures, then the nearest one.
  - A cube that fails twice is parked for 180 s.
- **Task ids are per assignment.** A re-assigned cube gets a new id. Reusing the old id let the previous FAILED report close the new attempt immediately; the harness caught this.
- **Collector FSM (`collector_node.py`).** It subclasses `executor_node`, so the collect and deliver chain is the same code on both robots: carrying BT, no reversing with a cube, delivery watchdog, per-spot limit.

  ```
  STARTUP → IDLE → GOING → SEEKING → TARGETING → APPROACHING → CAPTURING → DELIVERING → DETACHING → IDLE
  ```

  - GOING: Nav2 to a standoff pose 0.75 m from the cube, facing it, chosen clear in the leader's map (`standoff.py`).
  - SEEKING: face the cube's estimate, wait 2.5 s (YOLO runs at about 1 fps on the Pi's CPU), then sweep ±60°.
  - Only a detection within 0.7 m of the task's estimate starts a chase.
  - The collector reports COLLECTED only for a release at HOME. Anything else is FAILED, with a reason.
- **Leader right of way.** The fleet manager publishes the leader's pose on `/bz2/fleet/leader_pose` and what it is doing on `/bz2/fleet/leader_state` (MOVING, STUCK or PARKED, from `leader_state.py`: its motion plus its Nav2 recovery count).
  - An unladen collector yields (YIELDING state) by backing off 0.3 m when a MOVING or STUCK leader is within 0.9 m, or any leader within 0.6 m.
  - It resumes once the leader is 1.3 m away or has been PARKED for 4 s. It never plans past a STUCK leader: after holding clear it gives the task back instead.
  - The fleet manager doesn't assign a cube while the leader is within 1 m of it, unless the leader has been PARKED there for 30 s.
  - Why, and the runs behind each number: `multi_robot_runs.md`, runs 13–16.
- **Why namespace + `/bz2/tf`.** Both robots use the same topic and frame names. Every shared node now uses relative topic names, so without a namespace (the leader) they resolve exactly as before.
- **Nav2 params.** `params_rewrite.py` derives the collector's params from the leader's `nav2_params.yaml`, so the tuning can't drift. It:
  - nests the params under the namespace;
  - makes topics relative, except the static layer's `/map`;
  - drops the SweepStraight planner and the coverage layer;
  - adds `config/amcl_collector.yaml`.

## Start pose

RTAB-Map puts the map origin at the leader's start pose. The collector's `--start X Y YAW` is its start pose in that frame, measured from the leader's start:
- X is forward along the leader's start heading;
- Y is to the leader's left;
- YAW is in radians, counter-clockwise.

Tape both spots and measure once. AMCL corrects a few decimetres, not a wrong guess. Check the `/bz2/particle_cloud` in RViz before the first task.

The collector delivers to **its own** start pose (its HOME). Put it beside the drop-off zone.

## Running

```bash
# Jetson (leader) — any research options work as usual; --fleet implies --detect-only
./run_full_mission.sh --fleet --strategy interleaved --inspection mixed

# Pi (collector) — after the leader prints STACK IS UP
cd ~/hiruna/botzilla-autonomous-collector
./run_collector.sh --start 0.0 -0.8 0.0          # --leader <host|ip> if not hirunahhm.local
```

Logs:
- leader: `run_logs/<ts>/fleet.log`, one `FLEET {json}` line per event (new_cube, confirmed, assign, result, …);
- Pi: `run_logs/collector-<ts>/collector.log`.

RViz on the laptop: add MarkerArray `/fleet/cubes` (grey unconfirmed, yellow pending, blue assigned, green collected, red failed). For the collector's own data, set the TF topic to `/bz2/tf`.

## First-time Pi setup

```bash
cd ~/hiruna/botzilla-autonomous-collector && bash tools/setup_collector_pi.sh
```

This installs:
- ROS 2 Jazzy ros-base, Nav2/AMCL, robot_localization, depth_image_proc and pointcloud_to_laserscan;
- libfreenect, with its Python wrapper built from source as on the Jetson;
- CPU ultralytics.

It then builds `--packages-up-to botzilla_fleet`. Log out and back in afterwards for the dialout/plugdev groups.

## Testing without hardware

```bash
tools/harness/fleet_harness.sh 200            # 3 cubes: all delivered to HOME
PHANTOM=1 tools/harness/fleet_harness.sh 200  # + a leader-only false detection: 2 tries, parked
```

`fake_fleet.py` fakes both robots: the leader's camera, and the collector's Nav2, base and camera. It uses the real FOV, blind spot and range gate. It tests decision logic only. Unit tests: `cd botzilla_Workspace/src/botzilla_fleet && python3 -m pytest test`.

## Known limits / open

- The collector shows up in the leader's map while it's in view. RTAB-Map clears it as it moves, and AMCL tolerates it.
- On a loop closure, RTAB-Map can shift `/map`. AMCL follows the new map, but a task's target is not re-projected.
- Pi load is unmeasured: Nav2, AMCL, YOLO on the CPU and the Kinect point cloud on 4 GB. Measure it on the first run before tuning anything.
- No live re-allocation. One collector gets one task at a time, and the leader never collects.
