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

## Milestone 0 — Sensor correctness (prerequisite)

1. Add a `camera_link_optical` frame in `botzilla_qbot.urdf`: a zero-offset child link of `camera_link` rotated `-pi/2 0 -pi/2` (standard ROS optical convention), and point the `rgbd_camera` sensor's frame at it (`optical_frame_id` / gz-sensor equivalent tag).
2. Rebuild, relaunch the sim, and verify with `gz topic echo -t /camera/points -n 1` and `ros2 topic echo /camera/depth/image_raw --no-arr` that `frame_id` and `encoding` (`32FC1` expected) are correct.
3. **Validation**: URDF edit + topic inspection only — pass/fail is "frame_id correct, encoding correct."

## Milestone 1 — New `botzilla_navigation` package + RTAB-Map bring-up

1. `ros2 pkg create botzilla_navigation --build-type ament_python` alongside the existing three packages (`botzilla_control`, `botzilla_perception`, `botzilla_bringup`), matching their conventions. This replaces `PHASE_1_PLAN.md`'s `botzilla_slam`/`botzilla_executor` split — one package for all nav-related launch/config/nodes matches how this repo is actually organized.
2. Write `rtabmap.launch.py`: launch `rtabmap_slam`'s `rtabmap` node directly (skip `rtabmap_odom`'s visual odometry — the robot already has wheel `/odom` from the diff-drive plugin, cheaper on the Jetson) subscribing to `rgb/image:=/camera/rgb/image_raw`, `depth/image:=/camera/depth/image_raw`, `rgb/camera_info:=/camera/camera_info`, `odom_topic:=/odom`, `scan_topic:=/scan`, `frame_id:=base_link`, `approx_sync:=true`.
3. Drive around manually with the existing `teleop_keyboard_node` while watching `/map` in Foxglove and rtabmap's terminal log for `"Loop closure detected"` messages.
4. **Validation**: `/map` populates, `tf2_echo map base_link` resolves with no gaps, loop closures fire on revisiting an area. Run `tegrastats` alongside to log CPU/thermal — this is literally the fallback trigger condition from `PHASE_1_PLAN.md` §2, so capture a baseline number here.

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
