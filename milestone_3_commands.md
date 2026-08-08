# 🚀 Milestone 3 Execution & Verification Commands Guide

This guide lists the exact terminal commands to launch, operate, and verify **Milestone 3** (RTAB-Map Multi-Sensor SLAM + EKF Odometry Fusion + Nav2 Goal Following + Asymmetric Footprint) on the BotZilla workspace.

---

## 🖥️ 0. Remote Display / NoMachine Prep (If running via SSH)
If you are connected via SSH and viewing through a NoMachine virtual window (`DISPLAY=:1001`), export your X11 variables in any terminal where you plan to open a GUI tool (`rviz2` or `simulation.launch.py` with GUI):

```bash
export DISPLAY=:1001
export XAUTHORITY=$HOME/.Xauthority
```

---

## 🏃 1. Launching the Milestone 3 Stack (Open 4 Terminals)

Always source `/opt/ros/jazzy/setup.bash` and `install/setup.bash` inside the workspace directory (`~/Desktop/Projects/sem5/final-project-botzilla/botzilla_Workspace`) in every terminal.

### Terminal 1: Build & Launch Gazebo Simulation
```bash
cd ~/Desktop/Projects/sem5/final-project-botzilla/botzilla_Workspace
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install   # Only required if you modified C++ / Python / URDF files
source install/setup.bash

# Launch simulation with GUI (or pass gz_headless:=true for headless mode)
ros2 launch botzilla_bringup simulation.launch.py gz_headless:=false
```
*(Wait ~5–10 seconds for Gazebo to finish spawning `botzilla_qbot` and starting the `/clock` bridge).*

### Terminal 2: Launch RTAB-Map Multi-Sensor SLAM
Consumes camera RGB-D, 2D LiDAR (`/scan`), and EKF filtered odometry (`/odometry/filtered`) to build `/map` and `/cloud_map`:
```bash
cd ~/Desktop/Projects/sem5/final-project-botzilla/botzilla_Workspace
source install/setup.bash
ros2 launch botzilla_navigation rtabmap.launch.py
```

### Terminal 3: Launch Custom Nav2 Stack
Runs our lightweight custom launch with `controller_server` (`DWBLocalPlanner` with asymmetric arm footprint), `planner_server`, `behavior_server`, `bt_navigator`, and `velocity_smoother`:
```bash
cd ~/Desktop/Projects/sem5/final-project-botzilla/botzilla_Workspace
source install/setup.bash
ros2 launch botzilla_navigation nav2.launch.py
```

### Terminal 4: Launch RViz2 Visualization & Goal Setting
```bash
# Ensure X11 export is applied if running over SSH / NoMachine
export DISPLAY=:1001
export XAUTHORITY=$HOME/.Xauthority

cd ~/Desktop/Projects/sem5/final-project-botzilla/botzilla_Workspace
source install/setup.bash
ros2 run rviz2 rviz2
```

#### Recommended RViz2 Displays to Add:
1. **Global Map**: Add `Map` $\rightarrow$ Topic: `/map` $\rightarrow$ Durability Policy: **`Transient Local`**, Reliability: **`Reliable`**.
2. **Global Costmap**: Add `Map` $\rightarrow$ Topic: `/global_costmap/costmap` (Frame: `map`, shows static walls + inflation buffer).
3. **Local Costmap**: Add `Map` $\rightarrow$ Topic: `/local_costmap/costmap` (Frame: `odom`, shows live rolling window & obstacle voxels).
4. **Robot Footprint**: Add `Polygon` $\rightarrow$ Topic: `/local_costmap/published_footprint` (shows our custom asymmetric arm footprint: `[-0.17, -0.17] ... [0.26, 0.10]`).
5. **Filtered Odometry**: Add `Odometry` $\rightarrow$ Topic: `/odometry/filtered`.
6. **TF Trees & Robot Model**: Add `RobotModel` and `TF`.

---

## 🧪 2. Verification & Diagnostic Commands (Terminal 5)

Use these commands to verify that the navigation stack, controllers, and costmaps are running correctly without needing to eyeball the GUI.

### A. Verify Nav2 Lifecycle Manager Status
Check if all 5 Nav2 nodes are actively managed and running:
```bash
ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger
```
*(Expected output: `success=True, message='Managed nodes are active'`)*

### B. Verify Costmaps & Odometry Streams
Verify both costmaps and EKF odometry are publishing live data:
```bash
ros2 topic hz /odometry/filtered
ros2 topic hz /global_costmap/costmap
ros2 topic hz /local_costmap/costmap
```
*(Expected output: `/odometry/filtered` at ~50Hz, costmaps publishing continuously)*

### C. Verify Velocity Smoothing Chain (`cmd_vel_nav` $\rightarrow$ `cmd_vel`)
Check that `controller_server` outputs to `/cmd_vel_nav` and `velocity_smoother` relays acceleration-limited commands to `/cmd_vel`:
```bash
ros2 topic info /cmd_vel_nav
ros2 topic info /cmd_vel
```

---

## 🎯 3. Testing Autonomous Goal Following (`NavigateToPose`)

You can command autonomous point-to-point transit in two ways:

### Method A: Interactive RViz2 (`2D Goal Pose` button)
1. Click the **`2D Goal Pose`** button in the top toolbar of RViz2.
2. Click and drag anywhere in open space on the `/map` display to set the target position and desired facing orientation.
3. Watch the green global path appear and DWB execute the trajectory while avoiding obstacles.

### Method B: CLI Action Command (Automated Test Goal)
Send a test goal coordinate directly from the terminal (e.g., target `(0.0, -1.5)` in open space):
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: -1.5, z: 0.0}, orientation: {w: 1.0}}}"
```
* Monitor progress in Terminal 3 (`nav2.launch.py` logs showing `[bt_navigator] Goal succeeded`).
* To cancel/preempt an active goal via CLI, press `Ctrl+C` on the `ros2 action send_goal` command and send a new goal, or use:
  ```bash
  ros2 action send_goal /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
  ```
