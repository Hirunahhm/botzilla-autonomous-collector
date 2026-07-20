# Phase 1 Implementation Plan — Option A (RTAB-Map + Nav2)

This is the concrete, milestone-level execution plan for Option A of `PHASE_1_PLAN.md` (RTAB-Map visual-laser SLAM + Nav2), based on the actual current state of the repo (`botzilla_Workspace/`) and the actual packages installed on the Jetson Orin Nano Super — not the aspirational package layout described in `PHASE_1_PLAN.md`.

## Findings that shape this plan

- **Nothing heavy to install**: `rtabmap_ros`, `navigation2` (bringup/costmap_2d/amcl/controller/planner), `slam_toolbox`, and `robot_localization` are all already installed and resolvable via `ros2 pkg list` on the Jetson. `ros-jazzy-pointcloud-to-laserscan` is missing but is likely **not needed** — the robot's `gpu_lidar` already publishes a real 360° `/scan`, and Nav2's `VoxelLayer` consumes `PointCloud2` directly for 3D obstacles. Skip installing it unless a later step proves it's actually required.
- **Greenfield**: a repo-wide grep found zero references to `nav2`, `slam_toolbox`, `rtabmap`, `costmap`, or `NavigateToPose` anywhere in `src/`. This is a clean build, not a modification of existing SLAM code.
- **Camera frame gap**: `camera_link`'s `rgbd_camera` sensor in `botzilla_qbot.urdf` has no `optical_frame_id`/`gz_frame_id` override, so it publishes using URDF convention (X-forward) instead of the ROS optical convention (Z-forward) that RTAB-Map expects for image projection. This must be fixed before RTAB-Map will localize correctly against the depth data.
- **Reusable code**: `brain_node.py`'s `TARGETING` → `APPROACHING` → `CAPTURING` P-control / blind-push logic already works and should be reused almost verbatim inside the new executor. Only `SEARCHING` (currently a blind rotate) and `DELIVERING` (currently an open-loop timed drive) get replaced with Nav2 goals.

Sensor facts confirmed from the URDF:
- `camera_joint`: parent `base_link`, child `camera_link`, origin `xyz="0.15 0 0.07" rpy="0 0 0"`.
- `laser_joint`: parent `base_link`, child `laser_frame`, origin `xyz="0.0 0.0 0.12" rpy="0 0 0"`.
- `rgbd_camera` sensor: `horizontal_fov 1.047`, `640x480`, `near 0.1 / far 10.0`, `update_rate 30`, base `<topic>camera</topic>` (Gazebo auto-suffixes `camera/image`, `camera/depth_image`, `camera/points`, `camera/camera_info`).
- `gpu_lidar` sensor: full 360°, `update_rate 15`, range `0.15–10.0m`, base `<topic>scan</topic>`.
- Diff-drive plugin: `wheel_separation=0.23`, `wheel_radius=0.035`, publishes `/odom` at 50Hz, `frame_id=odom`, `child_frame_id=base_footprint`.

---

## Milestone 0 — Sensor correctness (prerequisite) — ✅ DONE

1. ~~Add a `camera_link_optical` frame~~ Added `camera_link_optical` as a zero-offset child link of `camera_link` (`camera_optical_joint`, `rpy="-1.570796 0 -1.570796"`), and set `<optical_frame_id>camera_link_optical</optical_frame_id>` inside the `rgbd_camera` sensor's `<camera>` block in `botzilla_qbot.urdf`. Confirmed via `sdformat14`'s `camera.sdf` spec that this element controls the `frame_id` used in the camera/depth/camera_info message headers.
2. Rebuilt `botzilla_bringup`, relaunched headless, and verified:
   - `/camera/camera_info`, `/camera/rgb/image_raw`, `/camera/depth/image_raw` all report `frame_id: camera_link_optical`.
   - Depth encoding is `32FC1` as expected.
   - `/tf_static` shows the full chain `base_footprint → base_link → camera_link → camera_link_optical` (plus the pre-existing `laser_frame`/`camera_link` → Gazebo-scoped-name bridges).
   - `tf2_echo base_link camera_link_optical` resolves to translation `(0.150, 0.000, 0.070)` and RPY `(-90°, 0°, -90°)` — exactly matching the URDF joint definition.
3. **Validation result**: pass. `yolo_node` still crashes on launch (`ModuleNotFoundError: ultralytics`) — pre-existing, unrelated to this change, not yet fixed.

## Milestone 1 — New `botzilla_navigation` package + RTAB-Map bring-up — ✅ DONE

1. Created `botzilla_navigation` (ament_python), matching existing package conventions. Added `launch/rtabmap.launch.py`: launches `rtabmap_slam`'s `rtabmap` node directly against wheel `/odom` (no `rtabmap_odom` visual odometry — cheaper on the Jetson), subscribing to `/camera/rgb/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info`, `/odom`, `/scan`, with `frame_id:=base_link`, `approx_sync:=true`, ICP-based `Reg/Strategy=1` registration against the laser scan.
2. **Found and fixed two real infrastructure bugs in `botzilla_bringup/launch/simulation.launch.py`** while getting this to actually produce a map (not RTAB-Map config issues — the sim's ROS bridge itself was incomplete):
   - **Missing `/clock` bridge**: nothing bridged Gazebo's sim clock to ROS, so every `use_sim_time:=true` node (`robot_state_publisher`, `rtabmap`, TF listeners) had a frozen clock, silently breaking all sim-time TF lookups. Fixed by adding `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock` to the bridge.
   - **Wrong TF source topic**: the bridge subscribed to Gazebo's plain `/tf` topic, but the `DiffDrive` plugin actually publishes `odom→base_footprint` on the *scoped* per-model topic `/model/botzilla_qbot/tf`. The plain topic was empty, so `odom` and `base_link` were permanently disconnected TF trees. Fixed by bridging the scoped topic and remapping it to `/tf`.
3. **Also found (not a code bug, an environment gotcha)**: `pkill -f` unreliably fails to kill `ros2 launch`-spawned process trees in this environment, even with exact-match patterns and an apparently-clean `ps aux` check immediately after. This caused hours of misdiagnosis chasing phantom TF/sync bugs that were actually multiple leftover sim instances silently fighting over the same `/tf`/`/clock`/sensor topics. Saved as a durable memory (`feedback_process_cleanup`) — future cleanup must gather PIDs via `pgrep -fa` and kill+verify each one individually with `kill -0`.
4. **Validation result — pass**, verified on a genuinely single clean instance:
   - `botzilla_qbot` confirmed present via `gz model --list`.
   - `odom → base_link` and full `map → base_link` TF chains resolve correctly (`tf2_echo` showed a real, non-identity pose matching an intentional in-place rotation).
   - `/info` and `/map` both populate with real data after a small nudge (`ref_id: 20`, occupancy grid 151×161 cells @ 5cm resolution ≈ 7.5m×8m, matching the arena's ~7×7m floor).
   - `tegrastats` baseline while RTAB-Map + Gazebo were both running: all 6 CPU cores 51–99% busy, CPU/GPU temp ~54.5°C (well below Jetson throttle range), ~6.2W total power draw, ~4GB/7.5GB RAM. No thermal concern for Option A on this hardware.
   - Not yet done: extended manual driving with `teleop_keyboard_node` + Foxglove to confirm loop-closure detection over a longer traverse — left for an interactive session since it needs a human driving and watching in real time.

## Milestone 2 — Option B fallback validated in parallel

1. Write `slam_toolbox.launch.py` (async mode) + a Nav2 `nav2_costmap_2d` config with `VoxelLayer` subscribing to `/scan` and `/camera/points` (bridge `camera/points` through `ros_gz_bridge` if not already — the current bridge only forwards `image`/`depth_image`/`camera_info`).
2. Run both configs back-to-back over the same drive path; compare CPU/thermal and map quality.
3. **Validation**: this is a comparison, not pass/fail — record numbers so the fallback switch (`use_rtabmap:=false use_slam_toolbox:=true`) is a data-backed decision if Jetson thermals become a problem later.

## Milestone 3 — Nav2 goal-following on top of the chosen map

1. Add `nav2_bringup`'s `navigation_launch.py` (controller/planner/bt_navigator/costmaps) pointed at rtabmap's `/map`.
2. Manually test with `ros2 action send_goal /navigate_to_pose ...` or RViz2/Foxglove's goal-pose tool — confirm the robot drives there and avoids the obstacles/furniture already in the arena.
3. **Validation**: goal-reached result returned; local costmap in Foxglove shows obstacles marked around furniture.

## Milestone 4 — Frontier exploration node

1. `frontier_explorer_node.py`: pure-function frontier detection (scan `OccupancyGrid` for free/unknown boundary cells, cluster, pick nearest reachable centroid) + a thin `rclpy` wrapper that feeds centroids to Nav2's `NavigateToPose` action client, replanning on each map update.
2. **Unit test**: the detection function takes a plain 2D array, not ROS messages — `test/test_frontier_detection.py` with small synthetic grids (5×5, known frontier cells), asserting the output coordinates. This is real, fast, ROS-independent unit testing, beyond the linter-only tests `botzilla_control/test/` currently has.
3. **Integration validation**: launch the full stack in sim, let it explore untouched — confirm `/map` completes without manual driving.

## Milestone 5 — Unified executor (replaces `brain_node.py`'s role)

1. New `executor_node.py` in `botzilla_navigation`: state machine mirroring `brain_node.py`'s `_transition`/timer/constants-at-top style, but:
   - `SEARCHING` → delegate to the frontier explorer, interrupted the moment `/detected_cube` reports a real detection (cancel the active Nav2 goal).
   - `TARGETING` / `APPROACHING` / `CAPTURING` → copied near-verbatim from `brain_node.py` — this logic already works.
   - `DELIVERING` → replace the fixed-4-second open-loop drive with a `NavigateToPose` goal to the drop-off origin, then fine-align using existing `/drop_off_pose` from `apriltag_node` on final approach.
   - `DETACHING` → unchanged reverse-and-release.
   - After delivery, resume frontier exploration for the next cube (multi-cube loop from `PHASE_1_PLAN.md`).
2. **End-to-end validation**: full sim run — robot explores → finds a cube spawned in the world → navigates, aligns, captures → Nav2-navigates back to drop-off → delivers → resumes exploring. This is the actual Phase 1 success criterion.

---

## Open decision points

- Whether `ros-jazzy-pointcloud-to-laserscan` ends up needed at all (currently expected: no).
- Go/no-go between Option A and Option B, decided empirically in Milestone 2 using CPU/thermal data, not guesswork.
