# Autonomous Multi-Cube Search & Collection Robot

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?style=for-the-badge&logo=ros&logoColor=white)
![JetPack](https://img.shields.io/badge/JetPack-7.2-76B900?style=for-the-badge&logo=nvidia&logoColor=white)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Nano-ff6b35?style=for-the-badge)
![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

**Embedded Software Systems Project | Semester 5 | Integrated Computer Engineering | University of Moratuwa**

*An autonomous mobile robot system that explores an unknown indoor environment, detects and physically collects an unknown number of cubes, and deposits each at a defined drop-off point — without any predefined map, arena dimensions, or human intervention.*

---

## 📖 Overview

Most autonomous collection robots assume a known, pre-mapped environment. This project removes that assumption. Starting with no map and no knowledge of how many target objects exist, the system functions in two phases:

1. **Phase 1 (Weeks 1–5)**: Single-robot system. Exploring an unknown arena using frontier exploration, building the map, detecting target cubes, avoiding obstacles, and carrying out the collect-and-deposit loop.
2. **Phase 2 (Weeks 6–11)**: Multi-robot leader/executor system. A Jetson-based search robot localizes target cubes and coordinates task allocation, while a Raspberry Pi 5-based executor robot receives targets, avoids dynamic obstacles, and retrieves the cubes.

---

## 🏗️ System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    SEARCH ROBOT (Leader)                     │
│      Jetson Orin Nano + Kobuki QBot 2 + Kinect + LiDAR       │
│  Frontier Exploration → SLAM Mapping → Target Cube Detection  │
│             Sends cube map-coordinates to Executor            │
└───────────────────────────┬──────────────────────────────────┘
                             │  (discrete goals, not raw velocities)
                             ▼  ROS 2 DDS Middleware
┌──────────────────────────────────────────────────────────────┐
│                    EXECUTOR ROBOT (Member)                   │
│         Raspberry Pi 5 + Kobuki QBot 2 + Kinect + LiDAR      │
│  Nav2 Path Planning → Local 3D Obstacle Avoidance (Voxel)    │
│            Navigates to cube → Collects → Drops off           │
└────────────────────────────────────────────────┘
```

---

## ⚙️ Hardware Stack

* **Mobile Bases**: $2 \times$ Kobuki QBot 2 chassis with differential drive.
* **Vision & Depth**: Kinect RGB-D cameras (YOLOv8 cube detection & 3D Voxel obstacle avoidance).
* **Laser Scanners**: 2D LiDARs for map construction.
* **Compute**:
  * **Search Robot**: Jetson Orin Nano Super running JetPack 7.2 (ROS 2 Jazzy).
  * **Executor Robot**: Raspberry Pi 5 (ROS 2 Jazzy).
* **Capture Hardware**: Custom-designed passive funneling/pushing arms.

---

## 📅 Phased Timeline (Weeks 1–11)

### 🗓️ Phase 1: Single Robot Autonomous Stack

#### **Week 1 — Jetson Setup & Hardware Bring-Up**
* **Goals**: Configure Jetson Orin Nano Super with JetPack 7.2, install ROS 2 Jazzy and Gazebo Harmonic, and verify all physical sensors independently.
* **Tasks**:
  - [ ] Flash JetPack 7.2 on the Jetson Orin Nano Super.
  - [ ] Install ROS 2 Jazzy, Gazebo Harmonic, and dependencies (`slam_toolbox`, `navigation2`, `libfreenect-dev`, `ultralytics`).
  - [ ] Connect and verify Kobuki Base (confirm `/cmd_vel` control and `/odom` telemetry).
  - [ ] Connect and verify Kinect Camera (confirm `/camera/rgb/image_raw` and `/camera/depth/image_raw`).
  - [ ] Connect and verify 2D LiDAR (confirm `/scan`).
  - [ ] Set up Workspace and compile drivers using `colcon build`.

#### **Week 2 — Simulation Environment Creation & Architecture Decisions**
* **Goals**: Construct the digital twin simulation world and select the primary SLAM/perception architecture.
* **Tasks**:
  - [ ] Build a multi-room indoor Gazebo SDF world featuring partitions, doors, scattered cubes ($N=4$), and dynamic obstacles.
  - [ ] Compare Option A (RTAB-Map Visual-Laser graph fusion) and Option B (SLAM Toolbox 2D mapping + Nav2 3D Voxel Layer).
  - [ ] Define coordinate frames ($map \rightarrow odom \rightarrow base\_link \rightarrow sensor\_frames$).

#### **Week 3 — Validation with Simulation**
* **Goals**: Run, tune, and validate the complete navigation, SLAM, and search stack in Gazebo.
* **Tasks**:
  - [ ] Tune costmap inflation and footprint parameters for the passive pushing arms.
  - [ ] Verify frontier exploration successfully navigates through doorways to uncover unmapped rooms.
  - [ ] Validate 3D Voxel Layer obstacle avoidance using low floor barriers.
  - [ ] Test the N-cube state machine loop from exploration to delivery.

#### **Week 4 — Physical Hardware Deployment**
* **Goals**: Deploy the compiled simulation stack onto the physical Jetson Orin Nano Super and Kobuki QBot 2 base.
* **Tasks**:
  - [ ] Run `hardware.launch.py` to bring up the physical Kinect, LiDAR, and base.
  - [ ] Perform manual mapping runs and test basic autonomous point-to-point navigation.
  - [ ] Calibrate the YOLOv8 Nano model against real-world lighting conditions in the testing room.

#### **Week 5 — Hardware Perfecting & Debugging**
* **Goals**: Refine real-world collection runs, optimize processing loads, and finalize Phase 1 stability.
* **Tasks**:
  - [ ] Export YOLOv8 to a TensorRT engine (`best.engine`) to run GPU-accelerated inference.
  - [ ] Calibrate gripper/capture alignment speeds and blind-spot thresholds.
  - [ ] Optimize ROS 2 DDS settings for reliable performance under full SLAM + YOLO load.

---

### 🗓️ Phase 2: Multi-Robot Task Allocation

#### **Week 6 — Start of Phase 2 (Multi-Robot Architecture)**
* **Goals**: Design the network, message definition, and multi-robot coordinate alignment system.
* **Tasks**:
  - [ ] Establish ROS 2 multi-machine DDS communication between the Jetson and Raspberry Pi 5.
  - [ ] Define the custom task assignment message interface (Searcher sending cube coordinates to Executor).

#### **Week 7 — Executor Bring-up**
* **Goals**: Set up the Raspberry Pi 5 executor robot with ROS 2 Jazzy and configure its local navigation stack.
* **Tasks**:
  - [ ] Deploy Kobuki drivers and Kinect/LiDAR sensors on the Pi 5.
  - [ ] Run local path planning (Nav2) on the Executor using maps received from the Leader.

#### **Week 8 — Coordinate Alignment & Map Sharing**
* **Goals**: Align coordinate frames between the two robots using a known starting pose method.
* **Tasks**:
  - [ ] Verify the Executor receives the shared occupancy grid `/map` from the Leader.
  - [ ] Align transforms so that target coordinates sent by the Leader are accurate in the Executor's frame.

#### **Week 9 — Task Allocation Logic**
* **Goals**: Implement the decision-making brain on the Search robot to dispatch targets.
* **Tasks**:
  - [ ] Write the coordinator node: when a cube is found, check if Executor is idle, and send target coordinate.
  - [ ] Handle task queues (queuing detected cubes if the Executor is currently delivering).

#### **Week 10 — Dynamic Obstacle Avoidance Tuning**
* **Goals**: Perfect inter-robot collision avoidance.
* **Tasks**:
  - [ ] Tune Nav2 parameters on the Executor to treat the Search robot as a moving obstacle and steer around it.
  - [ ] Calibrate the local safety reflex (stop immediately if the other robot is too close).

#### **Week 11 — Integrated Evaluation & Final Debugging**
* **Goals**: Conduct full multi-room, multi-cube collection runs and compile benchmarks.
* **Tasks**:
  - [ ] Run full system tests with all 4 cubes placed in unknown rooms.
  - [ ] Measure average exploration time, collection efficiency, and network latency.

---

## 💻 Installation & Usage

### Prerequisites
* NVIDIA JetPack 7.2 on Jetson (Host)
* ROS 2 Jazzy
* Gazebo Harmonic

### Workspace Setup
```bash
mkdir -p ~/botzilla_ws/src
cd ~/botzilla_ws/src
# Clone repository
git clone https://github.com/IntellisenseLab/final-project-botzilla.git .
cd ~/botzilla_ws
colcon build --symlink-install
source install/setup.bash
```

### Running Simulation
```bash
ros2 launch botzilla_bringup simulation.launch.py
```

### Running Hardware
```bash
ros2 launch botzilla_bringup hardware.launch.py
```

---

## 🏛️ Affiliation
**Department of Computer Science and Engineering**  
University of Moratuwa  
Embedded Software Systems Project | 2026
