# 🤖 BotZilla — Autonomous Object Detection, Collection & Placement Robot

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?style=for-the-badge&logo=ros&logoColor=white)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Nano-ff6b35?style=for-the-badge)
![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/Raspberry_Pi_4-4GB-C51A4A?style=for-the-badge&logo=raspberrypi&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

**CS3340 — Robotics and Automation | University of Moratuwa**

*An autonomous mobile robot that detects, collects, and places cubes using YOLO-based vision and RGB-D sensing on the Kobuki QBot platform.*

</div>

---

## 📖 Overview

In this final project we are building robot(Kobuki-Qbot) that operates in a controlled indoor environment. Starting from a charging station, the robot searches for cubes, collects them using a maipulator arm, deposits them in a designated drop-off zone, and returns to dock — all without human intervention.

The project is motivated by real-world applications such as:
- 📦 Automated sorting in logistics warehouses
- ☢️ Debris clearance in hazardous environments
- 🏫 Educational robotics demonstrations

The system is built on three core robotics pillars: **Perception**, **Planning**, and **Control**, integrated via the ROS 2 framework.

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        BotZilla System                       │
│                                                              │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐   │
│  │  PERCEPTION  │───▶│   PLANNING   │───▶│   CONTROL    │   │
│  │              │    │              │    │              │   │
│  │ Xbox Kinect  │    │ State Machine│    │  Trajectory  │   │
│  │ RGB-D Camera │    │   (SMACH)    │    │  Tracking    │   │
│  │              │    │              │    │              │   │
│  │ YOLOv8 Nano  │    │ Grid Search  │    │  Gripper     │   │
│  │ PCL Fusion   │    │ Path Planning│    │  Control     │   │
│  └──────────────┘    └──────────────┘    └──────────────┘   │
│                                                              │
│                   ROS 2 (Jazzy) Middleware                   │
└─────────────────────────────────────────────────────────────┘
```

### State Machine Flow

```
[CHARGING DOCK] ──▶ [SEARCH] ──▶ [APPROACH] ──▶ [PICK]
                        ▲                           │
                        │                           ▼
                   [RETURN]  ◀──  [PLACE]  ◀── [NAVIGATE TO ZONE]
```

---

## ⚙️ Hardware Components

| Component | Details |
|-----------|---------|
| **Mobile Base** | Kobuki QBot (differential-drive) |
| **Vision System** | Xbox Kinect RGB-D Camera |
| **Compute** | Raspberry Pi 4 (4GB) |
| **Manipulator** | Simple gripper arm attached to base |
| **Power** | Kobuki battery with charging dock + voltage monitoring sensor |

---

## 💻 Software Stack

| Layer | Technology |
|-------|-----------|
| **Framework** | ROS 2 Jazzy |
| **Object Detection** | YOLOv8 Nano |
| **Depth Processing** | Point Cloud Library (PCL) |
| **Task Sequencing** | SMACH State Machine |
| **Simulation** | Gazebo |
| **Localisation Markers** | AprilTags |

---

## 🚀 Getting Started

### Prerequisites

- Ubuntu 22.04 (or compatible)
- ROS 2 Jazzy installed
- Python 3.10+
- Gazebo (for simulation) 

### Installation

```bash
# Clone the repository
git clone https://github.com/<your-org>/botzilla.git
cd botzilla

# Install Python dependencies
pip install -r requirements.txt

# Build the ROS 2 workspace
colcon build --symlink-install
source install/setup.bash
```

### Running in Simulation (Gazebo)

```bash
# Launch the Gazebo simulation environment
ros2 launch botzilla simulation.launch.py

# In a new terminal, start the main autonomy stack
ros2 launch botzilla botzilla_autonomy.launch.py
```

### Running on Hardware

```bash
# Ensure Kobuki and Kinect are connected, then:
ros2 launch botzilla hardware.launch.py

# Start the autonomy stack
ros2 launch botzilla botzilla_autonomy.launch.py
```

---

## 📁 Repository Structure

```
botzilla/
├── test/                   # test files for kuboki moving 
├── src/
│   ├── perception/          # YOLOv8 detection + PCL depth fusion nodes
│   ├── planning/            # SMACH state machine + path planning
│   ├── control/             # Motor control + gripper scripts
│   └── docking/             # Charging station alignment logic
├── models/
│   └── yolov8_cube.pt       # Trained YOLOv8 Nano model weights
├── config/
│   ├── kinect_calibration/  # Kinect RGB-D calibration files
│   └── apriltag_config/     # AprilTag configuration for drop-off zone
├── launch/
│   ├── simulation.launch.py
│   ├── hardware.launch.py
│   └── botzilla_autonomy.launch.py
├── datasets/                # Cube image datasets (see references)
├── docs/                    # Reports, diagrams, proposal
├── tests/                   # Unit and integration tests
├── requirements.txt
└── README.md
```

---

## 🎯 Expected Outcomes

- ✅ **≥ 80% accuracy** in cube detection and localization under varying lighting conditions
- ✅ Successful collection and placement of **all cubes** in the drop-off zone (0.5m × 0.5m)
- ✅ Safe return to the **charging dock** after task completion
- ✅ No fault tolerance issues with the **gripper arm** during manipulation

---

## 📅 Project Timeline

| Week | Task | Description |
|------|------|-------------|
| Week 08 | **Hardware Setup** | Kobuki QBot configuration and ROS 2 interface stabilisation |
| Week 09 | **Perception Setup** | Kinect RGB-D calibration and dataset collection |
| Week 10–12 | **Core Implementation** | Object detection, navigation, model training, iterative testing |
| Week 13 | **Final Report** | Navigation enhancement, model accuracy improvement, report finalisation |
| Week 14 | **Demo & Viva** | Live demonstration and Viva Voce |

---

## 👥 Team BotZilla

| Name | Index | Responsibilities |
|------|-------|-----------------|
| **Mudaliarachchi N.S** | 230415H | RGB-D depth sensing, dataset collection, model training |
| **H.H. Malavipathirana** | 230389E | QBot command configuration, key-command programming, object detection & navigation |
| **K.N.B. Abeysundara** | 230010L | Raspberry Pi setup, YOLOv8 pipeline, ROS 2 integration, final report |

---

## 📚 References

- [Blob Tracking using Kobuki QBot2](https://www.youtube.com/watch?v=L_A1ThF3q6Y)
- [Raspberry Pi 4 — Setup & Getting Started](https://www.youtube.com/watch?v=2RHuDKq7ONQ)
- [YOLO on Raspberry Pi](https://youtu.be/z70ZrSZNi-8?si=31wIvSw6LGXEnhvU)
- [Kobuki QBot Documentation](https://kobuki.readthedocs.io/en)
- [Cube Dataset — Roboflow](https://universe.roboflow.com/zeynabezzati/dataset-yzhqr)

---

## 🏛️ Affiliation

**Department of Computer Science and Engineering**  
University of Moratuwa  
CS3340 — Robotics and Automation | March 2026

---