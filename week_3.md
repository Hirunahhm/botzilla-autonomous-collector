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

## 4. Next Steps (Milestone 2 Roadmap)
With Milestone 1 (RTAB-Map Graph SLAM & Remote Visualization) fully verified, our focus shifts to **Milestone 2**:
1. **Nav2 Costmap & Footprint Tuning**: Configure `nav2_costmap_2d` parameters with our custom robot footprint (`botzilla_qbot` with twin passive capture arms) and connect the `/map` topic to the navigation stack.
2. **Target Coordinator Integration**: Connect `yolo_node` (`/detected_cube` 3D target coordinates) to the state machine coordinator (`botzilla_coordinator`) to command autonomous approach trajectories.
3. **End-to-End Simulation Run**: Execute the complete Phase 1 autonomous verification loop:
   $$\text{Explore Arena} \longrightarrow \text{Detect Red Cube} \longrightarrow \text{Navigate & Align} \longrightarrow \text{Capture Cube} \longrightarrow \text{Deliver to Drop-Off Zone}$$
