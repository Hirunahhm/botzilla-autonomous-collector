#!/usr/bin/env bash
#
# run_collector.sh — bring up the COLLECTOR robot (the Raspberry Pi) for a two-robot run.
#
# Start the leader first, on the Jetson:
#     ./run_full_mission.sh --fleet [research options...]
# then, on the Pi:
#     ./run_collector.sh --start X Y YAW
#
# The collector joins the leader's ROS graph through the leader's Fast DDS discovery
# server, localises in the leader's live /map with AMCL, and waits for tasks from the
# leader's fleet_manager_node. Everything it runs lives under /bz2 (see
# botzilla_fleet/launch/collector.launch.py).
#
# Options:
#   --leader HOST       leader's hostname or IP (default hirunahhm.local). A hostname
#                       survives the hotspot handing out new IPs; an IP does not.
#   --start X Y YAW     collector start pose in the LEADER's map frame: metres and
#                       radians from the leader's taped start spot (RTAB-Map's origin).
#                       Measure it. AMCL corrects decimetres, not a wrong guess.
#   --ns NAME           namespace (default bz2; must match the leader's --fleet NAME)
#   --kobuki PORT       Kobuki serial port (default: found by id)
#   --lidar PORT        LiDAR serial port (default: found by id)
#   --local-yolo        run YOLO on this Pi's CPU instead of the leader's GPU (default
#                       sends JPEG frames to the leader; the Pi alone manages ~1 fps)
#   --no-yolo           no cube detection at all (bring-up tests)
#   --build             colcon build the packages the collector needs first
#
# Ctrl+C once: stops the robot and tears everything down.
#
# Everything started is stopped by process group, never by name: a name pattern also
# matches whatever shell typed it.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tools/robot_stop.sh
source "$REPO_ROOT/tools/robot_stop.sh"
WS="${REPO_ROOT}/botzilla_Workspace"
ROS_SETUP=/opt/ros/jazzy/setup.bash
DISCOVERY_PORT=11811
LOG_DIR="${REPO_ROOT}/run_logs/collector-$(date +%Y%m%d-%H%M%S)"

LEADER="hirunahhm.local"
NS="bz2"
START_X=""; START_Y=""; START_YAW=""
KOBUKI=""; LIDAR=""
DETECTOR=leader
DO_BUILD=0

usage() { awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0"; }

while [ $# -gt 0 ]; do
    case "$1" in
        --leader)  LEADER="${2:-}"; shift ;;
        --ns)      NS="${2:-}"; shift ;;
        --start)   START_X="${2:-}"; START_Y="${3:-}"; START_YAW="${4:-}"; shift 3 ;;
        --kobuki)  KOBUKI="${2:-}"; shift ;;
        --lidar)   LIDAR="${2:-}"; shift ;;
        --no-yolo) DETECTOR=none ;;
        --local-yolo) DETECTOR=local ;;
        --build)   DO_BUILD=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
    esac
    shift
done

if [ -t 1 ]; then C_OK=$'\e[32m'; C_ERR=$'\e[31m'; C_INF=$'\e[36m'; C_WARN=$'\e[33m'; C_0=$'\e[0m'
else C_OK=; C_ERR=; C_INF=; C_WARN=; C_0=; fi
step() { echo "${C_INF}==>${C_0} $*"; }
ok()   { echo "${C_OK}  ok${C_0} $*"; }
warn() { echo "${C_WARN}  !!${C_0} $*"; }
die()  { echo "${C_ERR}ERROR:${C_0} $*" >&2; exit 1; }

num='^-?[0-9]+(\.[0-9]+)?$'
[ -n "$START_X" ] || die "--start X Y YAW is required: the collector's start pose in the leader's map"
for v in "$START_X" "$START_Y" "$START_YAW"; do
    [[ "$v" =~ $num ]] || die "--start values must be numbers (got: '$START_X' '$START_Y' '$START_YAW')"
done

source_ros() {
    set +u
    # shellcheck disable=SC1090,SC1091
    source "$ROS_SETUP"
    [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
    set -u
}

PGIDS=()
CLEANED=0
cleanup() {
    [ "$CLEANED" = 1 ] && return
    CLEANED=1
    # Ignore further INT/TERM until done. Otherwise a second signal (timed_run.sh's
    # timer and an operator's stop landing together, 2026-10-05) re-enters the trap,
    # which sees CLEANED=1 and exits on the spot, abandoning this cleanup halfway and
    # leaving the stack running.
    trap '' INT TERM
    echo
    step "shutting down"
    # The robot first, before anything that could stall: see tools/robot_stop.sh.
    ok "$(stop_robot_motion)"
    for pg in "${PGIDS[@]}"; do kill -INT -- "-$pg" 2>/dev/null; done
    for _ in $(seq 1 10); do
        alive=0
        for pg in "${PGIDS[@]}"; do kill -0 -- "-$pg" 2>/dev/null && alive=1; done
        [ "$alive" = 0 ] && break
        sleep 1
    done
    for pg in "${PGIDS[@]}"; do kill -TERM -- "-$pg" 2>/dev/null; done
    sleep 2
    for pg in "${PGIDS[@]}"; do kill -KILL -- "-$pg" 2>/dev/null; done
    ok "stopped"
    echo "logs: $LOG_DIR"
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

wait_for_log() {   # <file> <regex> <timeout_s> <what>
    local f=$1 re=$2 t=$3 what=$4 i=0
    while [ "$i" -lt "$t" ]; do
        if grep -qE "$re" "$f" 2>/dev/null; then printf '\r%72s\r' ''; ok "$what"; return 0; fi
        if grep -qE "Traceback|process has died|SerialException" "$f" 2>/dev/null; then
            printf '\r%72s\r' ''; grep -E -B2 -A12 "Traceback|process has died|SerialException" "$f" | head -40
            die "$what failed (see $f)"
        fi
        sleep 1; i=$((i+1))
        printf '\r  waiting for %s... %ds ' "$what" "$i"
    done
    printf '\r%72s\r' ''; tail -n 25 "$f"
    die "timed out after ${t}s waiting for $what (see $f)"
}

# ── preflight ────────────────────────────────────────────────────────────────
mkdir -p "$LOG_DIR"
step "preflight"
[ -f "$ROS_SETUP" ] || die "ROS 2 Jazzy not found at $ROS_SETUP"

LEADER_ADDR=$(getent ahostsv4 "$LEADER" 2>/dev/null | awk 'NR==1 {print $1}')
[ -n "$LEADER_ADDR" ] || die "cannot resolve the leader '$LEADER' (is it on the same hotspot? try --leader <ip>)"
ping -c 1 -W 2 "$LEADER_ADDR" >/dev/null 2>&1 || warn "leader $LEADER ($LEADER_ADDR) does not answer ping"
ok "leader $LEADER = $LEADER_ADDR (resolved now; re-run if the hotspot changes it)"

for dev in "$KOBUKI" "$LIDAR"; do
    [ -z "$dev" ] && continue
    [ -e "$dev" ] || die "$dev missing"
    [ -r "$dev" ] && [ -w "$dev" ] || die "no read/write on $dev; add yourself to 'dialout'"
done
ls /dev/serial/by-id/ 2>/dev/null | sed 's/^/    serial: /'

if [ "$DO_BUILD" = 1 ]; then
    step "building the collector's packages"
    ( cd "$WS" && source_ros && colcon build --symlink-install --packages-up-to botzilla_fleet ) \
        > "$LOG_DIR/build.log" 2>&1 || { tail -n 30 "$LOG_DIR/build.log"; die "build failed"; }
    ok "build finished"
fi
[ -d "$WS/install/botzilla_fleet" ] || die "botzilla_fleet is not built — run with --build"

export ROS_DISCOVERY_SERVER="$LEADER_ADDR:$DISCOVERY_PORT"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export BOTZILLA_YOLO_MODEL="$REPO_ROOT/runs/best-fit/best.pt"
# Ultralytics writes its settings under ~/.config; keep it out of anything root-owned.
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-$HOME/.cache/ultralytics}"
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE

# Waits for the fleet manager's map for this robot, not the leader's /map: the collector
# localises and plans on /$NS/fleet/map (the leader's map with its own floor cleared).
# Started before the fleet manager exists, its AMCL has no map, the planner's costmap
# waits forever for map->base_link and Nav2 aborts (2026-10-06 00:26). The fleet manager
# starts last on the leader, after HOME is latched, so this also waits for that.
step "waiting for /$NS/fleet/map from the leader's fleet manager (./run_full_mission.sh --fleet)"
for i in $(seq 1 60); do
    # Wait for an actual MESSAGE, not just the topic name. The name also appears when
    # only a subscriber exists — the laptop's RViz with botzilla_collector.rviz open
    # subscribes to it — and on 2026-10-06 that let the collector start before the
    # fleet manager existed: AMCL had no map, missed its lifecycle bond and Nav2
    # aborted. The map is latched, so one message is there as soon as it is published.
    if ( source_ros
         timeout 15 ros2 topic echo --once --qos-durability transient_local \
             --qos-reliability reliable --field header.frame_id \
             "/$NS/fleet/map" nav_msgs/msg/OccupancyGrid 2>/dev/null ) \
            | grep -x "map" > /dev/null; then
        # Not grep -q: it exits at the first match and, under pipefail, the SIGPIPE
        # the writer then gets fails the whole test (2026-10-06).
        printf '\r%72s\r' ''; ok "leader's fleet manager is publishing /$NS/fleet/map"; break
    fi
    [ "$i" = 60 ] && die "no /$NS/fleet/map from the leader after ~10 min of tries"
    printf '\r  no /%s/fleet/map yet (try %d)... ' "$NS" "$i"
    sleep 5
done

cat > "$LOG_DIR/run_config.json" <<EOF
{
  "started": "$(date -Is)",
  "role": "collector",
  "ns": "$NS",
  "leader": "$LEADER",
  "leader_addr": "$LEADER_ADDR",
  "start": [$START_X, $START_Y, $START_YAW],
  "detector": "$DETECTOR",
  "git_commit": "$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)",
  "host": "$(hostname)"
}
EOF

# ── launch ───────────────────────────────────────────────────────────────────
step "collector stack (/$NS): hardware, AMCL, Nav2, detector=$DETECTOR, collector_node"
echo "  ${C_WARN}the robot moves as soon as the leader assigns a cube${C_0}"
ARGS=("ns:=$NS" "start_x:=$START_X" "start_y:=$START_Y" "start_yaw:=$START_YAW"
      "detector:=$DETECTOR")
[ -n "$KOBUKI" ] && ARGS+=("serial_port:=$KOBUKI")
[ -n "$LIDAR" ] && ARGS+=("lidar_port:=$LIDAR")
( source_ros; exec setsid ros2 launch botzilla_fleet collector.launch.py "${ARGS[@]}" ) \
    > "$LOG_DIR/collector.log" 2>&1 &
PGIDS+=("$!")
sleep 1
wait_for_log "$LOG_DIR/collector.log" "Gyro bias calibrated" 90 "Kobuki base + gyro calibrated"
wait_for_log "$LOG_DIR/collector.log" "Managed nodes are active" 120 "AMCL + Nav2 active"
wait_for_log "$LOG_DIR/collector.log" "HOME latched" 120 "localised in the leader map, HOME latched"

cat <<EOF

${C_OK}============ COLLECTOR /$NS IS UP ============${C_0}
  Waiting for tasks from the leader's fleet_manager_node.

  Watch:
    tail -f $LOG_DIR/collector.log | grep -E 'Task|\] -> \['
    ros2 topic echo /$NS/fleet/status        # (with ROS_DISCOVERY_SERVER set as above)

  View the collector in RViz — on your laptop, in a SECOND terminal (the leader's RViz
  keeps running in the first), with the same exports as for the leader:

    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DISCOVERY_SERVER="${LEADER}:${DISCOVERY_PORT}"
    export ROS_SUPER_CLIENT=true
    unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE
    ros2 daemon stop

    # one-time (repeat if the config changes):
    scp hirunahhm@${LEADER}:~/Desktop/Projects/sem5/final-project-botzilla/tools/rviz/botzilla_collector.rviz ~/
    rviz2 -d ~/botzilla_collector.rviz --ros-args -r /tf:=/$NS/tf -r /tf_static:=/$NS/tf_static

  (the /tf remaps are required: the collector's TF is on /$NS/tf. See ros_dds.md.)

  Logs: $LOG_DIR
  ${C_WARN}Ctrl+C to stop the collector.${C_0}

EOF

while true; do
    for pg in "${PGIDS[@]}"; do
        if ! kill -0 -- "-$pg" 2>/dev/null; then
            warn "the collector launch exited — shutting down"
            exit 1
        fi
    done
    sleep 5
done
