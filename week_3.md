# Week 3 Progress Report: Milestone 1 Completion & Multi-Sensor Graph SLAM

This document summarizes the engineering achievements, remote system configurations, and Milestone 1 verification completed during Week 3 of the Semester 5 Autonomous Multi-Cube Search & Collection Robot project.

---

## 1. Milestone 1 Accomplishments: RTAB-Map Multi-Sensor Graph SLAM (`Option A`)

We successfully brought up and verified the primary **Option A** SLAM architecture (`rtabmap_ros` / `rtabmap_slam`) inside the upgraded multi-room Gazebo simulation (`botzilla_arena.world`). The system fuses four distinct sensor streams in real time:

### A. Four-Stream Sensor Fusion Pipeline (`rtabmap.launch.py`)
Created and tuned `botzilla_navigation/launch/rtabmap.launch.py` to process and synchronize our robot's sensor suite:
1. **2D LiDAR (`/scan` $\rightarrow$ `Reg/Strategy: '1'`)**:
   * Uses Iterative Closest Point (ICP) laser registration to construct the primary 2D structural occupancy grid (`/map` at $5\text{cm}$ resolution, $150 \times 159$ cells).
2. **Kinect RGB Camera (`/camera/rgb/image_raw` & `/camera_info`)**:
   * Extracts visual feature points (ORB/SURF keypoints) across camera frames. When the robot revisits a previously explored hallway or doorway, RTAB-Map triggers a **Visual Loop Closure**, eliminating accumulated odometry and LiDAR drift.
3. **Kinect Depth Sensor (`/camera/depth/image_raw`)**:
   * Synchronized with the RGB stream (`approx_sync:=True`). We configured `Grid/Sensor: '2'` and `Grid/FromDepth: 'true'` so RTAB-Map projects dense 3D RGB-D voxels directly into `/cloud_map` (`PointCloud2`) while refining visual constraints (`RGBD/NeighborLinkRefining: 'true'`).
4. **Wheel Odometry (`/odom`)**:
   * Fused at $50\text{Hz}$ as the baseline motion prior (`odom_frame_id: 'odom'`), providing smooth frame-to-frame tracking while laser matching and visual loop closures correct long-term drift.

```yaml
# Active RTAB-Map Launch Configuration (`rtabmap_parameters`)
'use_sim_time': true
'frame_id': 'base_link'
'odom_frame_id': 'odom'
'map_frame_id': 'map'
'subscribe_depth': true
'subscribe_rgb': true
'subscribe_scan': true
'approx_sync': true
'Reg/Strategy': '1'          # ICP + Visual (scan-assisted registration)
'Reg/Force3DoF': 'true'      # Ground robot constraint (3 Degrees of Freedom)
'Grid/RangeMax': '10.0'      # Maximum laser/depth mapping distance in meters
'Grid/Sensor': '2'           # 0=laser only, 1=depth only, 2=both laser and depth combined
'Grid/FromDepth': 'true'     # Generate 3D occupancy voxels from RGB-D depth sensor
'RGBD/NeighborLinkRefining': 'true'
```

### B. EKF Odometry Stabilization & Map Ghosting Resolution (`ekf.yaml` & `botzilla_qbot.urdf`)
During extended curved navigation and arc maneuvers, we observed severe **Map Ghosting** (also known in SLAM literature as **Map Duplication / Pose Graph Forking**), where walls and obstacles appeared duplicated in overlapping, rotated layers. Through empirical investigation and multi-sensor debugging, we identified and resolved two distinct underlying causes:

1. **Spurious Lateral Velocity ($V_y$) Drift**:
   * **Issue**: Differential drive odometry (`/odom`) kinematically constrains lateral velocity to zero ($v_y = 0$). However, `ekf.yaml` initially left $v_y$ unmeasured (`odom0_config: false` for $v_y$). During curved arc motions, process noise allowed $v_y$ to drift into a non-zero state, making the robot estimate slide sideways and corrupting scan registration.
   * **Fix**: Constrained $v_y$ directly (`odom0_config: true` for $v_y$), forcing the Extended Kalman Filter (`robot_localization`) to enforce zero lateral drift at all times.

2. **Gazebo IMU Phase Lag & Zero-Covariance Lock**:
   * **Issue**: We integrated a simulated IMU sensor (`gz-sim-imu-system`) on `imu_link`. Testing revealed two critical flaws: first, Gazebo's default IMU published an all-zero covariance matrix (`0.0`), which `robot_localization` interpreted as absolute certainty (`zero uncertainty`). Second, the simulated IMU angular rate (`angular_velocity.z`) lagged several seconds behind true wheel velocity during step changes in rotation. Because the EKF treated the zero-covariance IMU as ground truth, the laggy orientation estimate pulled the pose graph away from true heading during turns, causing RTAB-Map to register laser scans at conflicting angles.
   * **Fix**:
     * **Realistic Sensor Noise Model**: Added Gaussian noise parameters (`stddev: 0.0003 rad/s` for gyroscope, `0.017 m/s²` for accelerometer) to `botzilla_qbot.urdf` matching MEMS MPU-6050 specifications, ensuring proper covariance weighting.
     * **TF & Architecture Clean-Up**: Disabled the `DiffDrive` plugin's internal TF broadcast (`<publish_odom_tf>false</publish_odom_tf>`) and remapped RTAB-Map's odometry input to `/odometry/filtered` across `simulation.launch.py` and `rtabmap.launch.py`.
     * **Sim vs. Real Hardware Blending**: Since Gazebo differential-drive odometry is kinematically ideal with zero simulated wheel slip, we enabled wheel-odom yaw fusion (`true` in `odom0_config`) and temporarily bypassed IMU yaw fusion (`false` in `imu0_config`) to eliminate the Gazebo sensor lag artifact. The full IMU + Wheel EKF fusion pipeline remains ready for physical hardware deployment, where real-world wheel slip makes IMU blending essential.

---

## 2. Remote Headless Infrastructure & Display Engineering

To operate the NVIDIA Jetson Orin Nano Super over SSH with full hardware GPU acceleration without requiring a physical HDMI monitor, we engineered a robust remote display and environment workflow using NoMachine:

### A. Headless Virtual Desktop Configuration (`:1001`)
* Solved zero-resolution (`0x0`) and `qt.qpa.xcb: could not connect to display` fatal crashes occurring when Xorg boots without physical HDMI EDID timings.
* Configured NoMachine to launch a clean virtual X11 desktop session (`DISPLAY=:1001`) on demand (`nxserver --restart` with GDM stopped or using `xserver-xorg-video-dummy`).
* Resolved X11 `No protocol specified` authorization rejections when launching ROS 2 GUI applications over external SSH sessions by properly pointing `$XAUTHORITY` to NoMachine's virtual session cookie:
  ```bash
  export DISPLAY=:1001
  export XAUTHORITY=/home/hirunahhm/.Xauthority
  ```

### B. RViz2 3D Visualization & QoS Alignment
* **Durability QoS Matching**: Diagnosed and resolved the `No map received` / `Topic /map: QoS incompatibility` warning in RViz2 by aligning RViz2's Map display Durability policy to **`Transient Local`** (latched) and Reliability to **`Reliable`**, matching RTAB-Map's occupancy grid publisher.
* **3D Point Cloud Rendering**: Configured RViz2 `PointCloud2` (`/cloud_map`) with `Style: Flat Squares / Spheres`, `Size (m): 0.05`, and `Color Transformer: RGB8`, allowing live visualization of dense colored 3D walls, doorways, and collectible red target cubes floating above the 2D floor grid.

---

## 3. Architecture Rationale: 2D Floor Map vs. 3D Voxel Layer

During Milestone 1 testing, we verified why target cubes (`5cm` tall, below the $12\text{cm}$ LiDAR plane) should **not** be baked into the static 2D room map (`/map`), and confirmed our two-layer navigation architecture:

| Mapping Layer | Topic / System | Data Source | Architectural Role |
|---|---|---|---|
| **Global 2D Structural Map** | `/map` (`OccupancyGrid`) | 2D LiDAR (`/scan`) + odometry | **Fast Room-to-Room Pathfinding**: Used by Nav2 global planner (`SmacPlanner` / `A*`) to compute paths across rooms in $<1\text{ms}$ without getting bogged down by dynamic obstacles or millions of 3D triangles. |
| **Local 3D Voxel / Target Layer** | `/cloud_map` & Nav2 `VoxelLayer` | 3D Kinect Depth (`/camera/depth/image_raw`) | **Real-Time Obstacle & Target Handling**: Nav2's local Voxel Layer registers objects below the LiDAR plane (such as the $6\text{cm}$ low barrier and movable target cubes) to prevent collisions during driving while keeping the global static wall graph clean. |

---

## 4. Milestone 3 Accomplishments: Autonomous Navigation & Path Following (`nav2.launch.py`)

We successfully accomplished **Milestone 3**, integrating ROS 2 Navigation (`Nav2`) directly on top of RTAB-Map's live occupancy grid (`/map`) and EKF odometry (`/odometry/filtered`). This gives BotZilla robust autonomous point-to-point pathfinding across the multi-room arena:

### A. Tailored Navigation Architecture (`botzilla_navigation/launch/nav2.launch.py`)
Rather than using `nav2_bringup`'s generic `navigation_launch.py`—which unconditionally boots unneeded servers like `route_server`, `docking_server`, and `waypoint_follower` that consume CPU and expect static maps—we developed a custom, lightweight launch file. It boots only the exact lifecycle nodes required for live SLAM navigation:
* `controller_server` (`DWBLocalPlanner`)
* `planner_server` (`GridBased` / `NavfnPlanner`)
* `behavior_server` (spin, backup, drive_on_heading behaviors)
* `bt_navigator` (`NavigateToPose` action server)
* `velocity_smoother` & `lifecycle_manager_navigation`

### B. Controller & Asymmetric Arm Footprint (`nav2_params.yaml`)
We adapted `/opt/ros/jazzy/share/rtabmap_demos/params/turtlebot3_rgbd_nav2_params.yaml` specifically for our custom robot physical structure and computational limits on the Jetson Orin Nano Super:
1. **Asymmetric Footprint Polygon**: Built a custom exact collision footprint covering the circular chassis ($r=0.17\text{m}$) plus the protruding twin front capture arms (extending forward to $x=0.26\text{m}$):
   $$\text{Footprint Polygon: } [[-0.17, -0.17], [-0.17, 0.17], [0.17, 0.17], [0.26, 0.10], [0.26, -0.10], [0.17, -0.17]]$$
   This guarantees Nav2 never clips doorframes or obstacles with the front collection arms during turns.
2. **DWB Trajectory Optimization**: Configured `DWBLocalPlanner` with reduced velocity sampling (`vx_samples: 20`, `vtheta_samples: 20`) to keep CPU utilization low while maintaining smooth trajectory evaluation (`sim_time: 1.7s`).
3. **Velocity Limits & Smoothing**: Set linear velocity bounds ($max\_vel\_x=0.2\text{m/s}$) and angular limits ($max\_vel\_theta=0.4\text{rad/s}$, $acc\_lim\_theta=1.5\text{rad/s}^2$) to match the speeds proven across `brain_node.py` and `teleop_keyboard_node`. Verified the command chain where `controller_server` / `behavior_server` output to `/cmd_vel_nav` ($20\text{Hz}$), and `velocity_smoother` relays clean acceleration-capped commands directly to `/cmd_vel` ($50\text{Hz}$).

### C. Empirical Goal Verification
We validated the complete autonomous navigation loop:
* **Costmap Generation**: Verified live costmap layers (`/global_costmap/costmap` in frame `map`, `/local_costmap/costmap` in frame `odom` with voxel and inflation layers active).
* **Autonomous Transit**: Sent `NavigateToPose` goals across open rooms and narrow corridors. Confirmed the `bt_navigator` action server successfully plans around dynamic obstacles, executing smooth paths and terminating within the configured tolerances ($0.15\text{m}$ position, $0.25\text{rad}$ yaw).

---

## 5. Next Steps (Milestones 4 & 5 Roadmap)

With **Milestone 1 (Multi-Sensor SLAM)**, **Odometry EKF Stabilization**, and **Milestone 3 (Nav2 Autonomous Navigation)** fully accomplished and verified, our next objectives focus on autonomy execution:

1. **Milestone 4 — Frontier Exploration Node (`frontier_explorer_node.py`)**:
   * Implement automated frontier detection (scanning `/map` for free/unknown boundary cells, clustering centroids, and picking the nearest reachable target).
   * Wrap the detector in an `rclpy` node that continuously dispatches `NavigateToPose` action goals to explore the arena autonomously without human teleoperation.
2. **Milestone 5 — Unified Autonomy Executor (`executor_node.py`)**:
   * Integrate our existing vision (`yolo_node` / `/detected_cube`) and precision alignment (`apriltag_node` / `/drop_off_pose`) into a unified state machine:
     $$\text{Autonomous Frontier Search} \longrightarrow \text{Cube Detection & Preemption} \longrightarrow \text{Target Alignment & Capture} \longrightarrow \text{Nav2 Transit to Drop-Off} \longrightarrow \text{Release & Resume Search}$$
   * Conduct the full Phase 1 multi-cube search, collection, and delivery verification run in simulation.
