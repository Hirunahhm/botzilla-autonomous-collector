# Autonomous Multi-Cube Search & Collection Robot

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?style=for-the-badge&logo=ros&logoColor=white)
![JetPack](https://img.shields.io/badge/JetPack-7.2-76B900?style=for-the-badge&logo=nvidia&logoColor=white)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Nano-ff6b35?style=for-the-badge)
![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

**Embedded Software Systems Project | Semester 5 | Integrated Computer Engineering | University of Moratuwa**

*An autonomous mobile robot that explores an unmapped indoor arena, detects and physically collects an unknown number of cubes, and deposits each at a defined drop-off point — without any predefined map, arena dimensions, or human intervention.*

---

## 📖 Overview

Most autonomous collection robots assume a known, pre-mapped environment. This project removes that assumption. Starting with no map and no knowledge of how many target objects exist, the robot:

1. **Explores** an unknown arena autonomously using frontier-based exploration
2. **Detects** cubes using YOLOv8 running on-device via TensorRT
3. **Avoids** static obstacles using 2D LiDAR-based SLAM and Nav2
4. **Collects** each detected cube using an onboard gripper
5. **Delivers** it to a fixed drop-off point
6. **Repeats** until no further cubes can be found, then stops

A planned Phase 2 extends this to a **two-robot leader/executor system** — a Jetson-based leader robot handles perception, mapping, and task planning, while a Raspberry Pi-based executor robot receives high-level goals and physically retrieves cubes.

For the full project specification, phase breakdown, architecture decisions, and scope boundaries, see [`PROJECT.md`](./PROJECT.md).

---

## 🏗️ System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                        PLANNER (Leader)                       │
│  Frontier Exploration → Cube Detection → Task State Machine   │
│                    Issues high-level goals                    │
└───────────────────────────┬────────────────────────────────────┘
                             │  (goals, not raw velocity)
┌───────────────────────────▼────────────────────────────────────┐
│                        EXECUTOR                                 │
│         Nav2 (local planning, obstacle avoidance)               │
│              Executes goal → reports status                     │
└───────────────────────────┬────────────────────────────────────┘
                             │
                    ROS 2 Jazyy Middleware
```

**Phase 1:** Jetson robot plays both planner and executor roles.
**Phase 2:** Jetson remains the planner; a Raspberry Pi robot becomes a second executor.

---

## ⚙️ Hardware

| Component | Role |
|---|---|
| Kobuki QBot (×2 in Phase 2) | Mobile base |
| Microsoft Kinect v1 (RGB-D) | Cube detection + depth sensing |
| 2D LiDAR | SLAM + static/dynamic obstacle avoidance |
| Jetson Orin Nano Super | Compute — Leader robot |
| Raspberry Pi 4/5 | Compute — Executor robot (Phase 2 only) |
| Gripper arm | Cube manipulation |

## 💻 Software Stack

| Layer | Technology |
|---|---|
| OS / Framework | JetPack 7.2, ROS 2 Jazzy |
| SLAM | slam_toolbox |
| Navigation | Nav2 |
| Object Detection | YOLOv8 Nano → TensorRT |
| Kobuki Driver | Custom pyserial (adapted from BotZilla) |
| Kinect Bridge | freenect (adapted from BotZilla) |

> **Note:** Isaac ROS is intentionally not used — see `PROJECT.md` §3 for reasoning.

---

## 🗺️ Project Phases

| Phase | Scope | Target Duration |
|---|---|---|
| **Phase 1** | Single robot: explore unknown arena, static obstacle avoidance, detect + collect N cubes, deposit, terminate | Weeks 1–5 |
| **Phase 2** | Second robot (leader/executor), dynamic obstacle avoidance | Weeks 6–7 (stretch, gated on Phase 1 stability) |

Full week-by-week breakdown: [`PROJECT.md`](./PROJECT.md)

---

## ✅ Week 1 — Jetson Setup & Hardware Bring-Up

**Goal:** a fully configured Jetson running ROS 2 Jazyy, with the Kobuki, Kinect, and 2D LiDAR each independently verified before any project-specific code is written.

### 1.1 Flash JetPack 7.2

- [ ] Download **NVIDIA SDK Manager** on a host Ubuntu machine (or use the JetPack 7.2 SD card/NVMe image method, depending on your Jetson Orin Nano Super carrier board).
- [ ] Put the Jetson into Force Recovery Mode (per NVIDIA's official Orin Nano Super instructions) and connect via USB-C to the host machine.
- [ ] Flash **JetPack 7.2** (Ubuntu 24.04-based) using SDK Manager, selecting the Jetson Linux + JetPack components (skip the Deep Learning SDKs you don't need yet — CUDA, cuDNN, and TensorRT are required; Isaac ROS related components are **not** required for this project).
- [ ] Complete first-boot Ubuntu setup (user account, WiFi, locale set to `en_US.UTF-8`).
- [ ] Confirm JetPack version:
  ```bash
  sudo apt-cache show nvidia-jetpack
  ```

### 1.2 Install ROS 2 Jazzy

```bash
# Locale
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable the ROS 2 apt repository
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo apt update

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS 2 Jazzy + tools
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  git build-essential cmake pkg-config

# rosdep
sudo rosdep init || true
rosdep update

# Source on every shell
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

- [ ] Verify install:
  ```bash
  ros2 doctor
  ```

### 1.3 Install Navigation & Perception Dependencies

```bash
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-tools \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  libfreenect-dev

pip install --break-system-packages ultralytics pyserial numpy opencv-python
```

- [ ] Verify TensorRT/CUDA are present (should ship with JetPack):
  ```bash
  python3 -c "import tensorrt; print(tensorrt.__version__)"
  nvcc --version
  ```

### 1.4 Set Up the Workspace

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clone BotZilla as reference/base for Kobuki + Kinect bring-up
git clone https://github.com/IntellisenseLab/final-project-botzilla.git botzilla_reference
```

- [ ] Review `botzilla_reference` and copy over (do not just build in place) the following into new packages in `~/robot_ws/src`, per `PROJECT.md` §5:
  - `KobukiDriver.py` + `kobuki_base_node.py`
  - `kinect_bridge.py` (test **without** the `LD_PRELOAD=noreset.so` workaround first — that fixes a Raspberry Pi 5-specific USB bug that likely doesn't apply to the Jetson)
- [ ] Strip out cube-quadrant search logic, AprilTag docking, and the fixed single-cube SMACH sequence — these are not reused (see `PROJECT.md` §5).

### 1.5 Bring Up Each Sensor Independently

- [ ] **Kobuki:** connect via USB, confirm serial port (`ls /dev/ttyUSB*`), add user to `dialout` group if needed:
  ```bash
  sudo usermod -a -G dialout $USER
  ```
  Launch the base node, confirm `/odom` publishes and the robot responds to `/cmd_vel` via teleop.

- [ ] **Kinect:** connect via USB, launch `kinect_bridge`, confirm `/camera/rgb/image_raw` and `/camera/depth/image_raw` publish and look correct in `rqt_image_view` or RViz.

- [ ] **2D LiDAR:** connect (check serial/USB adapter), launch its driver, confirm `/scan` publishes and looks correct in RViz.

### ✅ Week 1 Deliverable

All three sensors (Kobuki, Kinect, LiDAR) individually verified and publishing correct data on a Jetson running JetPack 7.2 + ROS 2 Jazzy. Robot is drivable via keyboard teleop. No SLAM, navigation, or detection yet — that begins Week 2.

---

## 📚 References

- [BotZilla — prior project (Kobuki + Kinect bring-up reference)](https://github.com/IntellisenseLab/final-project-botzilla)
- [Kobuki QBot Documentation](https://kobuki.readthedocs.io/en)
- Full project specification: [`PROJECT.md`](./PROJECT.md)

---

## 🏛️ Affiliation

**Department of Computer Science and Engineering**
University of Moratuwa
Embedded Software Systems Project | 2026
