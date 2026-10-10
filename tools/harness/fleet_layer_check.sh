#!/usr/bin/env bash
# fleet_layer_check.sh — check the fleet obstacle layer on a real Nav2 costmap.
#
#   tools/harness/fleet_layer_check.sh
#
# Starts the leader's controller_server (its local costmap, nav2_params.yaml unchanged
# except the observation sources) on its own ROS domain with static TF, a fake open-floor
# LiDAR, and map->odom offset by (0.5 m, 0.3 m, 0.3 rad) like a real localisation
# correction. Publishes another robot's footprint 0.15 m beside this robot and one cube
# through botzilla_fleet's shapes_grid, then prints the costs that matter:
#   - this robot's own centre must stay below 99 (inscribed), or it could not plan out
#     — the side-by-side deadlock of 2026-10-06;
#   - the profile from it towards the other robot, and around the cube.
# Needs no robot hardware. Everything started is stopped by process group.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE/../../botzilla_Workspace"
set +u; source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=${DOMAIN:-91}; unset ROS_DISCOVERY_SERVER
LOG=${OUT:-/tmp}/fleet_layer_check.log
P=src/botzilla_navigation/config/nav2_params.yaml
setsid ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom --x 0.5 --y 0.3 --yaw 0.3 > /dev/null 2>&1 & G1=$!
setsid ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link > /dev/null 2>&1 & G2=$!
setsid ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id laser_frame --z 0.24 > /dev/null 2>&1 & G3=$!
setsid ros2 run nav2_controller controller_server --ros-args --params-file $P \
    -p local_costmap.local_costmap.voxel_layer.observation_sources:="scan" > "$LOG" 2>&1 & G4=$!
sleep 6
timeout 30 ros2 lifecycle set /controller_server configure --no-daemon > /dev/null 2>&1
timeout 30 ros2 lifecycle set /controller_server activate --no-daemon > /dev/null 2>&1
timeout 60 python3 "$HERE/fleet_layer_check.py"
for g in $G1 $G2 $G3 $G4; do kill -INT -- -$g 2>/dev/null; done; sleep 2
for g in $G1 $G2 $G3 $G4; do kill -KILL -- -$g 2>/dev/null; done
