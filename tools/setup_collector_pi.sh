#!/usr/bin/env bash
#
# setup_collector_pi.sh — one-time setup of the collector robot's Raspberry Pi 5
# (Ubuntu 24.04, arm64). Run ON THE PI, from the repo root:
#
#     bash tools/setup_collector_pi.sh
#
# It asks for the sudo password once. Safe to re-run: every step checks first.
#
# Installs ROS 2 Jazzy (ros-base, not desktop: the Pi runs headless and RViz stays on the
# laptop), the packages the collector stack launches (Nav2 + AMCL, robot_localization,
# depth_image_proc, pointcloud_to_laserscan, robot_state_publisher), libfreenect's Python
# bindings for the Kinect, and ultralytics (CPU) for yolo_node. Then builds the
# collector's packages.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
step() { echo; echo "==> $*"; }

[ "$(dpkg --print-architecture)" = arm64 ] || echo "!! not arm64 — this script targets the Pi"
. /etc/os-release
[ "${VERSION_CODENAME:-}" = noble ] || { echo "Ubuntu 24.04 (noble) required for Jazzy"; exit 1; }

step "ROS 2 apt source"
if [ ! -f /etc/apt/sources.list.d/ros2.sources ] && [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo apt-get update
    sudo apt-get install -y software-properties-common curl
    sudo add-apt-repository -y universe
    ver=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
          | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    curl -fL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.noble_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
else
    echo "   already present"
fi

step "ROS 2 Jazzy + collector dependencies"
sudo apt-get update
sudo apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-amcl \
    ros-jazzy-robot-localization \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-depth-image-proc \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros \
    ros-jazzy-visualization-msgs \
    python3-colcon-common-extensions \
    python3-serial python3-numpy python3-scipy python3-yaml python3-opencv \
    freenect libfreenect-dev \
    cython3 python3-dev python3-setuptools git \
    avahi-daemon

step "libfreenect Python wrapper"
# Ubuntu ships libfreenect but not its Python bindings; build them against the system
# library, exactly as on the Jetson (/usr/local/.../freenect-0.0.0-py3.12...egg).
if ! python3 -c 'import freenect' 2>/dev/null; then
    rm -rf /tmp/libfreenect
    git clone --depth 1 --branch v0.5.3 https://github.com/OpenKinect/libfreenect /tmp/libfreenect \
        || git clone --depth 1 https://github.com/OpenKinect/libfreenect /tmp/libfreenect
    ( cd /tmp/libfreenect/wrappers/python && sudo python3 setup.py install )
else
    echo "   already installed"
fi

step "serial + USB permissions"
sudo usermod -aG dialout,plugdev,video "$USER"
# libfreenect's udev rule, so the Kinect needs no root.
if [ ! -f /etc/udev/rules.d/51-kinect.rules ]; then
    sudo tee /etc/udev/rules.d/51-kinect.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
EOF
    sudo udevadm control --reload-rules && sudo udevadm trigger
fi

step "ultralytics (CPU) for yolo_node"
if ! python3 -c 'import ultralytics' 2>/dev/null; then
    # Ubuntu 24.04 marks the system Python as externally managed; yolo_node runs under
    # the system interpreter (ros2 run), so it has to go there. CPU-only torch keeps
    # the download small and avoids CUDA wheels the Pi cannot use.
    pip3 install --break-system-packages --index-url https://download.pytorch.org/whl/cpu torch torchvision
    pip3 install --break-system-packages ultralytics
fi

step "build the collector's packages"
cd "$REPO_ROOT/botzilla_Workspace"
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
colcon build --symlink-install --packages-up-to botzilla_fleet

step "checks"
python3 -c 'import freenect; print("   freenect ok")' || echo "!! python freenect missing"
python3 -c 'import ultralytics; print("   ultralytics", ultralytics.__version__)' || true
[ -f "$REPO_ROOT/runs/best-fit/best.pt" ] && echo "   YOLO model found" \
    || echo "!! runs/best-fit/best.pt missing (set BOTZILLA_YOLO_MODEL or copy it)"
[ -f "$REPO_ROOT/noreset.so" ] && echo "   noreset.so found" || echo "!! noreset.so missing"
echo
echo "Done. Log out and back in once so the new groups (dialout, plugdev) apply."
