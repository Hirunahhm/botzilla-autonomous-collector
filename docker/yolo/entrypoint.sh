#!/usr/bin/env bash
# Container entrypoint: source ROS, source the bind-mounted workspace if it has
# been built, then exec whatever command was passed.
set -e

# Belt-and-braces guard for the libucs shadowing described in the Dockerfile: make
# HPC-X's own ucx libraries win the linker search regardless of what apt may pull in
# later. Without this, if anything ever reintroduces Ubuntu's libucx0, `import torch`
# dies with "libucc.so.1: undefined symbol: ucs_config_doc_nop" — a failure whose
# message points nowhere near the actual cause.
export LD_LIBRARY_PATH="/opt/hpcx/ucx/lib:${LD_LIBRARY_PATH}"

source /opt/ros/${ROS_DISTRO}/setup.bash

# run_yolo_container.sh bind-mounts the colcon workspace at /ws. It is mounted
# rather than built into the image so edits to yolo_node.py take effect on the
# next container start with no rebuild.
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
else
    echo "WARNING: /ws/install/setup.bash not found — was the workspace built on the host?" >&2
fi

exec "$@"
