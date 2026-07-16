# BotZilla Simulation & Foxglove Guide

This guide details the step-by-step instructions to build, run, and visualize the BotZilla simulation in Gazebo Harmonic, connect to Foxglove Studio, and manually control the robot.

---

## 🛠️ 1. Build the Workspace
Always build the workspace after making modifications to the code, world, URDF, or launch files.

```bash
# Navigate to the workspace
cd ~/Desktop/Projects/sem5/final-project-botzilla/botzilla_Workspace

# Source the main ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# Build packages
colcon build --symlink-install

# Source the local workspace setup
source install/setup.bash
```

---

## 🚀 2. Launch the Simulation

You can run the simulation in either **headless** mode (viewing everything through Foxglove Studio) or **GUI** mode (displaying the Gazebo Harmonic window).

### Option A: Headless Mode (Recommended for SSH / Foxglove)
Ideal when running the simulation remotely over SSH without forwarding heavy 3D rendering.
```bash
ros2 launch botzilla_bringup simulation.launch.py gz_headless:=true
```

### Option B: GUI Mode (Jetson Desktop / Remote Desktop)
Use this if you are logged into the Jetson physical desktop, or connecting via VNC/NoMachine.
```bash
# Configure the local display server environment variables (check display number with `who` or `w` if needed)
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority

# Launch the simulation with GUI enabled
ros2 launch botzilla_bringup simulation.launch.py gz_headless:=false
```

---

## 📡 3. Launch Foxglove Bridge (Separate Terminal)
To stream the simulation topics (camera, LiDAR, and TF frames) into Foxglove Studio, start the Foxglove WebSocket server.

1. Open a new terminal session.
2. Source ROS 2 and run the bridge:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
   ```
3. Open Foxglove Studio (desktop app or [app.foxglove.dev](https://app.foxglove.dev)) and connect to the data source:
   * **Connection type**: `Foxglove WebSocket` (not Rosbridge)
   * **URL**: `ws://localhost:8765` (if using VS Code port forwarding) or `ws://<JETSON_IP_ADDRESS>:8765`

---

## 🎮 4. Drive the Robot Manually (Separate Terminal)

If you do not have `teleop_twist_keyboard` installed, you can command the robot to move by publishing directly to the `/cmd_vel` topic:

* **Drive Forward (0.2 m/s)**:
  ```bash
  source /opt/ros/jazzy/setup.bash
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```
* **Turn in place (0.5 rad/s)**:
  ```bash
  source /opt/ros/jazzy/setup.bash
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
  ```
* Press `Ctrl + C` in the publishing terminal to stop sending the command (the robot will coast to a stop).

---

## ⚠️ Known Gaps & Solutions

### A. YOLO Node Crashes (`ModuleNotFoundError: No module named 'ultralytics'`)
The YOLOv8 detection node will crash if `ultralytics` is missing from the system Python environment.
* **Fix**: Install it for the system user:
  ```bash
  pip3 install --user ultralytics
  ```

### B. Installing Interactive Keyboard Teleop
If you want to drive the robot interactively using your keyboard arrow keys or WASD:
* **Fix**: Install the teleop package:
  ```bash
  sudo apt-get update
  sudo apt-get install ros-jazzy-teleop-twist-keyboard
  ```
* **Run**:
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
