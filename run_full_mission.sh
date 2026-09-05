#!/usr/bin/env bash
#
# run_full_mission.sh — bring up the complete BotZilla autonomy stack on hardware.
#
#   hardware (Kobuki + Kinect + RPLIDAR + EKF)
#     -> RTAB-Map SLAM
#       -> Nav2
#         -> YOLO cube detection (GPU container)
#           -> executor + frontier explorer  (explore -> collect cube -> deliver HOME)
#
# Also starts a Fast DDS Discovery Server bound on all interfaces, so RViz on a
# laptop on the same network can see the graph. The exact command to run there is
# printed once everything is up.
#
# Usage:
#   ./run_full_mission.sh              # full stack, exactly as before
#   ./run_full_mission.sh --no-mission # stack only, robot stays still (map/teleop by hand)
#   ./run_full_mission.sh --build      # colcon build first
#   ./run_full_mission.sh --yes        # clear leftover processes without asking
#
# Research options (all default OFF — a bare run is unchanged):
#   --policy exhaustion|fraction   exploration/coverage policy arm for this run.
#                                  'fraction' (default) is the shipped interleaved
#                                  behaviour; 'exhaustion' is the explore-then-sweep
#                                  baseline the go/no-go gate is measured under.
#   --metrics                      start mission_metrics_node, writing
#                                  <logdir>/metrics.jsonl. It is subscribe-only by
#                                  construction, so it cannot perturb the mission.
#   --layout FILE.yaml             ground-truth cube positions in the map frame;
#                                  implies --metrics. Without it per-cube inspection
#                                  times cannot be recorded, which is the primary
#                                  research metric. Format:
#                                      cubes:
#                                        - {id: 1, x: 2.0, y: 0.5}
#                                  Coordinates are relative to where the robot STARTS
#                                  (RTAB-Map puts the map origin at the start pose), so
#                                  every run of a campaign must begin from the same
#                                  taped spot or the layout means nothing.
#
# Measure the gate (plain frontier exploration, coverage recorded but not acting):
#   ./run_full_mission.sh --policy exhaustion --layout layouts/arena_a.yaml
#
# Ctrl+C once: stops the robot and tears everything down cleanly.
#
# NOTE: this deliberately does NOT chmod the serial ports with a hardcoded sudo
# password the way the older run_botzilla.sh did. /dev/ttyUSB* are group 'dialout'
# and this user is in that group, so no privilege escalation is needed. If the
# preflight reports a permissions problem, fix the group membership once:
#     sudo usermod -aG dialout $USER     # then log out and back in
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${REPO_ROOT}/botzilla_Workspace"
ROS_SETUP=/opt/ros/jazzy/setup.bash

DISCOVERY_PORT=11811
LOG_DIR="${REPO_ROOT}/run_logs/$(date +%Y%m%d-%H%M%S)"

RUN_MISSION=1
DO_BUILD=0
FORCE_CLEAN=0
RUN_METRICS=0
POLICY=""          # empty => don't pass it; executor.launch.py keeps its own default
LAYOUT=""

# Printed by --help: the contiguous comment block at the top of this file. Derived
# rather than a hardcoded line range, which silently goes stale whenever the header
# is edited (it already had).
usage() { awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0"; }

# --policy and --layout take a value, so this cannot be a `for arg in "$@"` loop.
# Both `--policy fraction` and `--policy=fraction` are accepted.
while [ $# -gt 0 ]; do
    case "$1" in
        --no-mission)  RUN_MISSION=0 ;;
        --build)       DO_BUILD=1 ;;
        -y|--yes)      FORCE_CLEAN=1 ;;
        --metrics)     RUN_METRICS=1 ;;
        --policy)      POLICY="${2:-}"; shift ;;
        --policy=*)    POLICY="${1#*=}" ;;
        --layout)      LAYOUT="${2:-}"; shift ;;
        --layout=*)    LAYOUT="${1#*=}" ;;
        -h|--help)     usage; exit 0 ;;
        *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
    esac
    shift
done

# Validated here, loudly, rather than at the launch file. A bad policy name would
# otherwise be accepted silently by rclpy and fall through to the default arm, and a
# whole run would be recorded under the wrong label — the worst possible failure for
# a campaign, because nothing about the resulting data looks wrong.
if [ -n "$POLICY" ] && [ "$POLICY" != "exhaustion" ] && [ "$POLICY" != "fraction" ]; then
    echo "ERROR: --policy must be 'exhaustion' or 'fraction' (got: '$POLICY')" >&2; exit 2
fi

# A layout is only meaningful if something is recording, so it implies --metrics.
if [ -n "$LAYOUT" ]; then
    RUN_METRICS=1
    # mission_metrics_node deliberately treats an unreadable layout as a warning and
    # keeps running, so that it can never take down a mission. That is right for the
    # node and wrong for a measurement run: a mistyped path yields a complete-looking
    # metrics.jsonl with zero ground-truth cubes and no inspection times at all. Fail
    # before the robot moves instead.
    [ -f "$LAYOUT" ] || { echo "ERROR: --layout file not found: $LAYOUT" >&2; exit 2; }
    [ -r "$LAYOUT" ] || { echo "ERROR: --layout file not readable: $LAYOUT" >&2; exit 2; }
    LAYOUT="$(cd "$(dirname "$LAYOUT")" && pwd)/$(basename "$LAYOUT")"
fi

# ── output helpers ───────────────────────────────────────────────────────────
if [ -t 1 ]; then C_OK=$'\e[32m'; C_ERR=$'\e[31m'; C_INF=$'\e[36m'; C_WARN=$'\e[33m'; C_0=$'\e[0m'
else C_OK=; C_ERR=; C_INF=; C_WARN=; C_0=; fi
step() { echo "${C_INF}==>${C_0} $*"; }
ok()   { echo "${C_OK}  ok${C_0} $*"; }
warn() { echo "${C_WARN}  !!${C_0} $*"; }
die()  { echo "${C_ERR}ERROR:${C_0} $*" >&2; exit 1; }

PIDS=()          # every host process we start, in start order
CLEANED=0

# ROS 2's setup.bash reads unbound variables (AMENT_TRACE_SETUP_FILES and friends),
# so it aborts instantly under `set -u`. Every caller below is a subshell, so
# toggling -u here cannot leak back into the script's own checks.
source_ros() {
    set +u
    # shellcheck disable=SC1090,SC1091
    source "$ROS_SETUP"
    [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
    set -u
}

# ── teardown ─────────────────────────────────────────────────────────────────
cleanup() {
    [ "$CLEANED" = 1 ] && return
    CLEANED=1
    echo
    step "shutting down"

    # Stop the base first, while the graph is still alive to carry the message.
    # Reported honestly: if the graph is already gone this cannot get through, and
    # claiming otherwise would hide a robot that is still driving.
    if ( source_ros
         export ROS_DISCOVERY_SERVER="127.0.0.1:${DISCOVERY_PORT}"
         export ROS_SUPER_CLIENT=True
         timeout 8 ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}" ) >/dev/null 2>&1
    then ok "sent zero /cmd_vel"
    else warn "could not publish zero /cmd_vel (graph already down?)"
    fi

    # Reverse start order: consumers before producers.
    for (( i=${#PIDS[@]}-1 ; i>=0 ; i-- )); do
        kill -INT "${PIDS[i]}" 2>/dev/null
    done

    if [ -n "${YOLO_CID:-}" ]; then
        docker stop "$YOLO_CID" >/dev/null 2>&1 && ok "stopped YOLO container"
    fi

    # ros2 launch forwards SIGINT to its children, but they need a moment.
    for _ in $(seq 1 10); do
        pgrep -f "$LAUNCH_PATTERN" >/dev/null 2>&1 || break
        sleep 1
    done

    # Escalate INT -> TERM -> KILL rather than jumping straight to KILL. `fastdds
    # discovery` in particular IGNORES SIGINT outright (verified on this machine), so
    # without a TERM stage it would always need killing, and a KILL denies every node
    # the chance to shut its hardware down tidily.
    local survivors
    survivors=$(pgrep -f "$LAUNCH_PATTERN" 2>/dev/null | grep -v "^$$\$" || true)
    if [ -n "$survivors" ]; then
        echo "$survivors" | while read -r p; do kill -TERM "$p" 2>/dev/null; done
        for _ in $(seq 1 5); do
            pgrep -f "$LAUNCH_PATTERN" >/dev/null 2>&1 || break
            sleep 1
        done
    fi
    survivors=$(pgrep -f "$LAUNCH_PATTERN" 2>/dev/null | grep -v "^$$\$" || true)
    if [ -n "$survivors" ]; then
        warn "forcing (SIGKILL): $(echo "$survivors" | tr '\n' ' ')"
        echo "$survivors" | while read -r p; do kill -9 "$p" 2>/dev/null; done
        sleep 1
    fi

    ( source_ros; ros2 daemon stop ) >/dev/null 2>&1
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

    if pgrep -f "$LAUNCH_PATTERN" >/dev/null 2>&1; then
        warn "some processes survived — check: pgrep -fa '$LAUNCH_PATTERN'"
    else
        ok "all processes stopped"
    fi
    if lsof /dev/ttyUSB0 /dev/ttyUSB1 >/dev/null 2>&1; then
        warn "a serial port is still held"
    else
        ok "serial ports free"
    fi
    echo "logs: $LOG_DIR"
}
# Everything this script starts, for pgrep-based checks and forced cleanup.
LAUNCH_PATTERN="fast-discovery-server|fastdds discovery|ros2 launch botzilla|kobuki_base_node|kinect_bridge|rplidar_node|ekf_node|odom_covariance_relay|pointcloud_to_laserscan|robot_state_publisher|component_container|rtabmap|controller_server|planner_server|behavior_server|bt_navigator|velocity_smoother|lifecycle_manager|executor_node|frontier_explorer_node|mission_metrics_node"
# Ctrl+C must exit outright. With a bare `trap cleanup INT` the shell resumes the
# interrupted `sleep` afterwards, the watch loop then notices the processes cleanup
# just stopped, and reports them as an unexpected crash — alarming and untrue.
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

# ── wait helpers (readiness, not guesswork) ──────────────────────────────────
# Fixed sleeps are why bringup is flaky: RTAB-Map needs the camera streaming
# before it publishes a map, and Nav2 needs that map before it will activate.
clear_line() { printf '\r%*s\r' 72 ''; }
wait_for_log() {   # <file> <regex> <timeout_s> <what>
    local f=$1 re=$2 t=$3 what=$4 i=0
    while [ "$i" -lt "$t" ]; do
        if grep -qE "$re" "$f" 2>/dev/null; then clear_line; ok "$what"; return 0; fi
        # Surface a crash immediately rather than burning the whole timeout. A dead
        # serial reader is the important one: kobuki_base_node stays alive and looks
        # healthy while publishing nothing at all.
        if grep -qE "Traceback|process has died|SerialException" "$f" 2>/dev/null; then
            clear_line; tail -n 25 "$f"
            die "$what failed to start (see $f)"
        fi
        sleep 1; i=$((i+1))
        printf '\r  waiting for %s... %ds ' "$what" "$i"
    done
    clear_line; tail -n 25 "$f"
    die "timed out after ${t}s waiting for $what (see $f)"
}

start_bg() {   # <logfile> <cmd...>
    local log=$1; shift
    ( source_ros
      export ROS_DISCOVERY_SERVER="127.0.0.1:${DISCOVERY_PORT}"
      export ROS_SUPER_CLIENT=True
      exec "$@" ) > "$log" 2>&1 &
    PIDS+=("$!")
}

# ── step numbering ───────────────────────────────────────────────────────────
# The metrics node adds a stage, so the denominator is computed rather than written
# into six separate strings that would disagree with each other the moment one moved.
TOTAL_STEPS=6
[ "$RUN_METRICS" = 1 ] && TOTAL_STEPS=7
STEP_N=0
next_step() { STEP_N=$((STEP_N + 1)); step "${STEP_N}/${TOTAL_STEPS}  $*"; }

# ── preflight ────────────────────────────────────────────────────────────────
mkdir -p "$LOG_DIR"
step "preflight"

[ -f "$ROS_SETUP" ] || die "ROS 2 Jazzy not found at $ROS_SETUP"

# A leftover node from a previous run holding /dev/ttyUSB is the single most
# common cause of a broken bringup: the new kobuki_base_node starts, its serial
# reader thread dies with "device reports readiness to read but returned no data",
# and the node then sits there alive but publishing nothing.
stale=$(pgrep -f "$LAUNCH_PATTERN" 2>/dev/null | grep -v "^$$\$" || true)
if [ -n "$stale" ]; then
    echo "  processes from a previous run are still alive:"
    pgrep -af "$LAUNCH_PATTERN" | grep -v "^$$ " | sed 's/^/    /'
    if [ "$FORCE_CLEAN" = 1 ]; then
        echo "  --yes given, clearing them"
    elif [ -t 0 ]; then
        read -rp "  kill them and continue? [y/N] " a
        [[ "$a" =~ ^[Yy]$ ]] || die "aborted; stop them first"
    else
        # No terminal to ask at (nohup, CI, a wrapper script). Killing processes
        # without consent is not something to do by default, so require the flag.
        die "stale processes present and no terminal to confirm at. Re-run with --yes to clear them automatically."
    fi
    echo "$stale" | while read -r p; do kill -INT "$p" 2>/dev/null; done
    sleep 4
    echo "$stale" | while read -r p; do kill -9 "$p" 2>/dev/null; done
    sleep 1
    ok "cleared"
fi
if docker ps -q --filter ancestor=botzilla/yolo:jazzy | grep -q .; then
    docker ps -q --filter ancestor=botzilla/yolo:jazzy | xargs -r docker stop >/dev/null
    ok "stopped a leftover YOLO container"
fi

for dev in /dev/ttyUSB0 /dev/ttyUSB1; do
    [ -e "$dev" ] || die "$dev missing — is the Kobuki/LIDAR plugged in and powered?"
    [ -r "$dev" ] && [ -w "$dev" ] || die "no read/write on $dev. Add yourself to 'dialout':
    sudo usermod -aG dialout $USER    (then log out and back in)"
done
ok "serial ports present and writable"

command -v docker >/dev/null || die "docker not found (needed for YOLO)"
docker image inspect botzilla/yolo:jazzy >/dev/null 2>&1 \
    || die "docker image botzilla/yolo:jazzy not found — build it first"
ok "YOLO image present"

if [ "$DO_BUILD" = 1 ]; then
    step "building workspace"
    ( cd "$WS" && source_ros && colcon build --symlink-install ) \
        > "$LOG_DIR/build.log" 2>&1 || { tail -n 30 "$LOG_DIR/build.log"; die "build failed"; }
    ok "build finished"
fi
[ -d "$WS/install" ] || die "$WS/install not found — run with --build first"

# NoMachine's virtual session runs at realtime priority and has been measured
# dropping Nav2's control loop from 20Hz to 4.7Hz. Warn, don't kill: it needs
# sudo and it is the user's remote desktop.
if pgrep -f "nxnode.bin" >/dev/null 2>&1; then
    warn "NoMachine session is running — it starves Nav2's control loop."
    warn "for a clean run:  sudo pkill -f nxnode.bin; sudo pkill -u gdm gnome-shell"
fi

# A run directory that does not say which arm produced it is unusable evidence once
# there are forty of them. Written before anything starts, so it exists even if the
# run is aborted halfway.
{
    echo "{"
    echo "  \"started\": \"$(date -Is)\","
    echo "  \"policy\": \"${POLICY:-fraction (launch default)}\","
    echo "  \"metrics\": $([ "$RUN_METRICS" = 1 ] && echo true || echo false),"
    echo "  \"layout\": \"${LAYOUT:-}\","
    echo "  \"mission\": $([ "$RUN_MISSION" = 1 ] && echo true || echo false),"
    echo "  \"git_commit\": \"$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)\","
    echo "  \"git_dirty\": $(git -C "$REPO_ROOT" diff --quiet 2>/dev/null && echo false || echo true),"
    echo "  \"host\": \"$(hostname)\""
    echo "}"
} > "$LOG_DIR/run_config.json"

if [ -n "$POLICY" ] || [ "$RUN_METRICS" = 1 ]; then
    step "research run"
    ok "policy   : ${POLICY:-fraction (launch default)}"
    if [ "$RUN_METRICS" = 1 ]; then
        ok "metrics  : $LOG_DIR/metrics.jsonl"
        if [ -n "$LAYOUT" ]; then
            ok "layout   : $LAYOUT ($(grep -c -- '- *{' "$LAYOUT" 2>/dev/null || echo '?') cube(s))"
        else
            warn "no --layout: per-cube inspection times will NOT be recorded."
            warn "coverage and distance still are. Pass --layout for the primary metric."
        fi
    fi
fi

LAN_IP=$(ip -4 route get 1.1.1.1 2>/dev/null | grep -oP 'src \K\S+' | head -1)
[ -n "$LAN_IP" ] || LAN_IP=$(hostname -I | awk '{print $1}')

# Hostname alternative for the laptop, so RViz survives a DHCP reassignment. Only
# advertised if avahi is actually running, since without it the name will not resolve.
HOST_MDNS="$(hostname).local"
if ! systemctl is-active --quiet avahi-daemon 2>/dev/null; then
    HOST_MDNS="$(hostname).local   # WARNING: avahi-daemon is not running, this will not resolve"
fi

RVIZ_CFG="$HOME/botzilla_nav_debug.rviz"
[ -f "$RVIZ_CFG" ] || warn "$RVIZ_CFG missing — the scp line printed below will fail (RViz still works without it)"

# A DHCP lease shorter than a demo is worth knowing about before the demo, not during.
LEASE=$(ip -4 -o addr show scope global 2>/dev/null | grep -oP 'valid_lft \K[0-9]+' | sort -n | head -1)
if [ -n "$LEASE" ] && [ "$LEASE" -lt 3600 ]; then
    warn "DHCP lease has ${LEASE}s left — this Jetson's IP may change mid-run."
    warn "prefer the hostname form printed below, or ask for a DHCP reservation."
fi

# ── 1. discovery server ──────────────────────────────────────────────────────
next_step "Fast DDS discovery server (port $DISCOVERY_PORT, all interfaces)"
( source_ros
  exec fastdds discovery --server-id 0 -p "$DISCOVERY_PORT" ) > "$LOG_DIR/discovery.log" 2>&1 &
PIDS+=("$!")
for i in $(seq 1 15); do
    ss -lun 2>/dev/null | grep -q ":$DISCOVERY_PORT" && break
    sleep 1
done
ss -lun 2>/dev/null | grep -q ":$DISCOVERY_PORT" || die "discovery server did not bind"
clear_line; ok "listening on 0.0.0.0:$DISCOVERY_PORT"

# ── 2. hardware ──────────────────────────────────────────────────────────────
next_step "hardware (Kobuki, Kinect, RPLIDAR, EKF)"
start_bg "$LOG_DIR/hardware.log" ros2 launch botzilla_bringup hardware.launch.py
wait_for_log "$LOG_DIR/hardware.log" "Gyro bias calibrated" 60 "Kobuki base + gyro calibrated"

# ── 3. SLAM ──────────────────────────────────────────────────────────────────
next_step "RTAB-Map SLAM"
start_bg "$LOG_DIR/rtabmap.log" \
    ros2 launch botzilla_navigation rtabmap.launch.py \
        use_sim_time:=false depth_topic:=/camera/depth/image_meters
wait_for_log "$LOG_DIR/rtabmap.log" "rtabmap \([0-9]+\)" 90 "RTAB-Map processing frames"

# ── 4. Nav2 ──────────────────────────────────────────────────────────────────
next_step "Nav2"
start_bg "$LOG_DIR/nav2.log" ros2 launch botzilla_navigation nav2.launch.py
wait_for_log "$LOG_DIR/nav2.log" "Managed nodes are active" 90 "Nav2 lifecycle active"

# ── 5. YOLO ──────────────────────────────────────────────────────────────────
next_step "YOLO cube detection (GPU container)"
( cd "$REPO_ROOT"
  export ROS_DISCOVERY_SERVER="127.0.0.1:${DISCOVERY_PORT}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  exec ./docker/yolo/run_yolo_container.sh ) > "$LOG_DIR/yolo.log" 2>&1 &
PIDS+=("$!")
wait_for_log "$LOG_DIR/yolo.log" "YOLO Perception Node Initialized|Loaded model" 120 "YOLO model loaded"
YOLO_CID=$(docker ps -q --filter ancestor=botzilla/yolo:jazzy | head -1)

# ── 6. metrics (optional) ────────────────────────────────────────────────────
# Started BEFORE the executor so the mission is recorded from its first state
# transition. ROS handles the late-joining publishers fine; starting it after the
# executor would lose the opening STARTUP -> EXPLORING edge.
if [ "$RUN_METRICS" = 1 ]; then
    next_step "mission metrics recorder"
    METRICS_ARGS=(--ros-args -p use_sim_time:=false -p "output_path:=$LOG_DIR/metrics.jsonl")
    [ -n "$LAYOUT" ] && METRICS_ARGS+=(-p "cube_layout:=$LAYOUT")
    start_bg "$LOG_DIR/metrics.log" \
        ros2 run botzilla_navigation mission_metrics_node "${METRICS_ARGS[@]}"
    wait_for_log "$LOG_DIR/metrics.log" "Metrics -> " 30 "metrics recorder writing"
    # The node logs this warning itself and keeps going. Surfaced here too because in
    # the scrollback of a full bringup it is easy to miss, and it is the difference
    # between a usable run and a wasted one.
    if grep -q "No cube layout given" "$LOG_DIR/metrics.log" 2>/dev/null; then
        warn "metrics recorder has no ground truth — inspection times will be missing"
    fi
fi

# ── 7. mission ───────────────────────────────────────────────────────────────
if [ "$RUN_MISSION" = 1 ]; then
    next_step "mission executor + frontier explorer"
    echo "  ${C_WARN}the robot will start moving once HOME is latched${C_0}"
    # Only appended when asked for, so an unflagged run launches the identical
    # command line it always did.
    EXEC_ARGS=(use_sim_time:=false)
    [ -n "$POLICY" ] && EXEC_ARGS+=("sweep_trigger_mode:=$POLICY")
    start_bg "$LOG_DIR/executor.log" \
        ros2 launch botzilla_navigation executor.launch.py "${EXEC_ARGS[@]}"
    wait_for_log "$LOG_DIR/executor.log" "HOME latched" 90 "HOME latched — mission running"
    clear_line
else
    next_step "mission executor SKIPPED (--no-mission)"
fi

# ── ready ────────────────────────────────────────────────────────────────────
cat <<EOF

${C_OK}================ STACK IS UP ================${C_0}

  View the map in RViz — run this ${C_INF}on your laptop${C_0} (same network):

    source /opt/ros/jazzy/setup.bash

    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DISCOVERY_SERVER="${LAN_IP}:${DISCOVERY_PORT}"
    export ROS_SUPER_CLIENT=true

    unset ROS_LOCALHOST_ONLY
    unset ROS_AUTOMATIC_DISCOVERY_RANGE
    ros2 daemon stop          # it caches discovery settings; stale ones look like a dead robot
    ros2 topic list           # sanity check: should list /map /scan /odom ...

    scp ${USER}@${LAN_IP}:~/botzilla_nav_debug.rviz ~/     # one-time, config does not change
    rviz2 -d ~/botzilla_nav_debug.rviz

  ${C_INF}If this Jetson's IP changes${C_0} (it is on DHCP), the address above goes stale and
  RViz silently stops updating. Either re-run this script to print the new one,
  or swap in the hostname, which follows the IP automatically:

    export ROS_DISCOVERY_SERVER="${HOST_MDNS}:${DISCOVERY_PORT}"

  (needs mDNS on the laptop — standard on Ubuntu; some guest/corporate WiFi
  blocks the multicast it relies on, in which case use the IP above.)
  Your laptop's own IP changing does not matter — it is the client here.

  Without the saved config, set Fixed Frame to ${C_INF}map${C_0} and Add ->
    Map (/map) · LaserScan (/scan) · TF · RobotModel
    Path (/plan) · Odometry (/odometry/filtered)

  Watch from here:
    tail -f $LOG_DIR/executor.log
    ros2 topic echo /mission/status
$( [ "$RUN_METRICS" = 1 ] && cat <<METRICS

  ${C_INF}Recording${C_0} to $LOG_DIR/metrics.jsonl
    policy: ${POLICY:-fraction (launch default)}${LAYOUT:+  layout: $(basename "$LAYOUT")}
    The final coverage number is written on shutdown, so let Ctrl+C finish.
    Gate result:  tail -1 $LOG_DIR/metrics.jsonl | python3 -m json.tool
METRICS
)

  Logs: $LOG_DIR
  ${C_WARN}Ctrl+C to stop the robot and shut everything down.${C_0}

EOF

# Hold until Ctrl+C; if any launch dies, say which and tear down.
while true; do
    for pid in "${PIDS[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            warn "a component exited (pid $pid) — shutting down"
            exit 1
        fi
    done
    sleep 5
done
