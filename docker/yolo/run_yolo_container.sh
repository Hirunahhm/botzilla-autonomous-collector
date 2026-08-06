#!/usr/bin/env bash
# Run yolo_node inside the GPU container, talking to the ROS graph on the host.
#
# Usage:
#   ./run_yolo_container.sh                 # run yolo_node
#   ./run_yolo_container.sh bash            # shell inside the container, for poking around
#   ./run_yolo_container.sh ros2 topic list # any other command
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO_ROOT}/botzilla_Workspace"
IMAGE="${IMAGE:-botzilla/yolo:jazzy}"

if [ ! -d "${WS}/install" ]; then
    echo "ERROR: ${WS}/install not found. Build the workspace on the host first:" >&2
    echo "  cd ${WS} && colcon build --symlink-install" >&2
    exit 1
fi

# --network host + --ipc host: the ROS graph lives on the host (Gazebo, Nav2,
# rtabmap). Without both, DDS discovery either fails outright or falls back to
# a slow path — shared-memory transport needs the IPC namespace shared.
#
# The workspace is mounted rather than baked in so editing yolo_node.py only
# needs a host-side colcon build, not an image rebuild. --symlink-install means
# install/ points back at src/, so src must be mounted too — mounting the whole
# repo root covers that plus runs/best-fit/best.pt, which yolo_node resolves at
# import time via resolve_model_path().
# Only request a TTY when there actually is one. Launching this from nohup, a
# systemd unit, or a ros2 launch file otherwise dies immediately with
# "cannot attach stdin to a TTY-enabled container because stdin is not a terminal".
TTY_FLAGS=()
if [ -t 0 ]; then
    TTY_FLAGS=(-it)
fi

# The ROS graph lives on the host (kinect_bridge, Nav2, rtabmap). --network host
# makes DDS *discovery* work, but discovery working is deceptive: with it alone,
# `ros2 topic list` / `ros2 node list` inside the container show the entire host
# graph and the host reports the container as a subscriber, yet ZERO data crosses.
# FastDDS treats host participants as same-machine and negotiates its
# shared-memory transport, whose segments are not shared across the container
# boundary. Measured on hardware: 17.5 Hz on the host, 0 messages in-container —
# and /odom (tiny) was equally 0, which ruled out message size or /dev/shm sizing.
# FASTRTPS_DEFAULT_PROFILES_FILE below forces UDP-only and is what actually fixes
# it; see fastdds_udp_only.xml. The /dev/shm bind is kept as belt-and-braces for
# any other component that still wants SHM.
FASTDDS_PROFILE="$(dirname "${BASH_SOURCE[0]}")/fastdds_udp_only.xml"

exec docker run --rm "${TTY_FLAGS[@]}" \
    --runtime nvidia \
    --network host \
    --ipc host \
    -v /dev/shm:/dev/shm \
    -v "${FASTDDS_PROFILE}:/fastdds_udp_only.xml:ro" \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp_only.xml \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
    -v "${REPO_ROOT}:${REPO_ROOT}" \
    -v "${WS}:/ws" \
    -w "${REPO_ROOT}" \
    "${IMAGE}" \
    "$@"
