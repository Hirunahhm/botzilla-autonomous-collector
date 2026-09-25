#!/usr/bin/env bash
# harness.sh — run frontier_explorer_node + mission_metrics_node against fake_nav.py.
#
#   tools/harness/harness.sh NAME SECONDS [explorer --ros-args params...]
#   DOMAIN=82 tools/harness/harness.sh heats 280 -p search_strategy:=heats
#
# Tests decision logic only — see fake_nav.py. Logs and NAME.jsonl go to $OUT
# (default /tmp/botzilla_harness). Each run uses its own ROS_DOMAIN_ID (DOMAIN, default
# 87), so several can run in parallel. Everything started is stopped by process group,
# never by name: `pkill -f` on node names also matches whatever shell typed them.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
OUT="${OUT:-/tmp/botzilla_harness}"
mkdir -p "$OUT"
NAME=$1; SECS=$2; shift 2
cd "$REPO/botzilla_Workspace"
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=${DOMAIN:-87}
setsid python3 "$HERE/fake_nav.py" > "$OUT/fake_$NAME.log" 2>&1 & FP=$!
sleep 2
setsid ros2 run botzilla_navigation frontier_explorer_node --ros-args "$@" \
    > "$OUT/explorer_$NAME.log" 2>&1 & EP=$!
rm -f "$OUT/$NAME.jsonl"
setsid ros2 run botzilla_navigation mission_metrics_node --ros-args \
    -p output_path:="$OUT/$NAME.jsonl" -p arena_floor_m2:=18.0 \
    > "$OUT/metrics_$NAME.log" 2>&1 & MP=$!
sleep "$SECS"
kill -INT -- -$EP -$MP 2>/dev/null; sleep 3
kill -INT -- -$FP 2>/dev/null; sleep 1
kill -KILL -- -$EP -$MP -$FP 2>/dev/null
echo "finished $NAME -> $OUT"
