#!/usr/bin/env bash
# fleet_harness.sh — run fleet_manager_node + collector_node against fake_fleet.py.
#
#   tools/harness/fleet_harness.sh SECONDS
#
# Decision logic only — see fake_fleet.py. Logs go to $OUT (default
# /tmp/botzilla_harness/fleet). Own ROS_DOMAIN_ID (DOMAIN, default 88); everything is
# stopped by process group, never by name.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
OUT="${OUT:-/tmp/botzilla_harness/fleet}"
mkdir -p "$OUT"
SECS=${1:-240}
cd "$REPO/botzilla_Workspace"
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=${DOMAIN:-88}
unset ROS_DISCOVERY_SERVER
setsid python3 "$HERE/fake_fleet.py" > "$OUT/fake.log" 2>&1 & FP=$!
sleep 2
setsid ros2 run botzilla_fleet fleet_manager_node > "$OUT/manager.log" 2>&1 & MP=$!
setsid ros2 run botzilla_fleet collector_node --ros-args -r __ns:=/bz2 \
    -r /tf:=/bz2/tf -r /tf_static:=/bz2/tf_static -p robot_name:=bz2 \
    > "$OUT/collector.log" 2>&1 & CP=$!
sleep "$SECS"
kill -INT -- -$MP -$CP 2>/dev/null; sleep 3
kill -INT -- -$FP 2>/dev/null; sleep 1
kill -KILL -- -$MP -$CP -$FP 2>/dev/null
echo "finished -> $OUT"
grep -h "FLEET" "$OUT/manager.log" | sed 's/.*FLEET //'
grep -h "truth:" "$OUT/fake.log" | tail -1
