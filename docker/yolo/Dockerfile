# YOLO cube detection in a GPU-accelerated container on the Jetson.
#
# Why a container at all: the Jetson host has neither torch nor ultralytics, so
# yolo_node dies at import ("ModuleNotFoundError: No module named 'ultralytics'").
# Installing a CUDA-enabled torch directly on the host for JetPack 7 / L4T r39 is
# fiddly; NVIDIA already ships a working one in this image.
#
# Base: nvcr.io/nvidia/pytorch:25.08-py3
#   arm64 / aarch64, Ubuntu 24.04, Python 3.12, torch 2.8.0a0 with cuda.is_available()
#   == True on this board (verified before writing this file). Ubuntu 24.04 is also
#   exactly what ROS 2 Jazzy targets, so ROS installs from the normal apt repo.
FROM nvcr.io/nvidia/pytorch:25.08-py3

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# ---------------------------------------------------------------------------
# ROS 2 Jazzy (ros-base + cv_bridge). No desktop/rviz — this container only
# needs to subscribe to images and publish a Point.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release software-properties-common \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base \
        ros-jazzy-cv-bridge \
        ros-jazzy-sensor-msgs \
        ros-jazzy-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# ros-jazzy-vision-opencv is deliberately NOT installed: it is a metapackage
# (cv_bridge + image_geometry + opencv_tests) and only cv_bridge is needed here.
# Dropping it trims the image.
#
# IMPORTANT — the libucs conflict is NOT fixed by that omission. Ubuntu's libucx0
# still gets installed (pulled by libopenmpi3t64, verified with
# `apt-cache rdepends --installed libucx0` in the built image). It ships
# /usr/lib/aarch64-linux-gnu/libucs.so.0, which shadows the HPC-X copy in this base
# image; HPC-X's libucc.so.1 then cannot resolve ucs_config_doc_nop, and since torch
# links UCC for distributed support, `import torch` dies outright with:
#     ImportError: /opt/hpcx/ucc/lib/libucc.so.1: undefined symbol: ucs_config_doc_nop
#
# The actual fix is the LD_LIBRARY_PATH ordering in entrypoint.sh, which puts HPC-X's
# own ucx libs first. That means ANY command bypassing the entrypoint
# (e.g. `docker run --entrypoint python3 ...`) will still hit this error — run things
# through the entrypoint, or export LD_LIBRARY_PATH yourself.

# ---------------------------------------------------------------------------
# Ultralytics.
#
# --no-deps is deliberate and load-bearing: a plain `pip install ultralytics`
# pulls a generic PyPI torch wheel that has no Jetson CUDA support and would
# silently replace NVIDIA's build, dropping inference to CPU. So install the
# dependency set by hand, minus torch/torchvision/opencv which the base image
# and ROS already provide.
# ---------------------------------------------------------------------------
RUN pip install --no-cache-dir --no-deps ultralytics \
    && pip install --no-cache-dir \
        "pyyaml>=5.3.1" \
        "requests>=2.23.0" \
        "scipy>=1.4.1" \
        "tqdm>=4.64.0" \
        "psutil" \
        "py-cpuinfo" \
        "pandas>=1.1.4" \
        "ultralytics-thop>=2.0.0"

# Ultralytics otherwise tries to write settings/fonts into a read-only or
# root-owned HOME and warns on every start.
ENV YOLO_CONFIG_DIR=/tmp/ultralytics
ENV MPLCONFIGDIR=/tmp/matplotlib

# Headless: no display inside the container, and cv2.imshow would abort.
ENV QT_QPA_PLATFORM=offscreen

# DDS needs to reach the host stack; run_yolo_container.sh passes --network host.
ENV ROS_DOMAIN_ID=0

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "botzilla_perception", "yolo_node"]
