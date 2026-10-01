#!/usr/bin/env bash
#
# run_sim_mission.sh — bring up the complete BotZilla autonomy stack in Gazebo.
#
#   Gazebo + bridge + EKF + YOLO   (botzilla_bringup simulation.launch.py)
#     -> RTAB-Map SLAM
#       -> Nav2
#         -> executor + frontier explorer  (+ mission_metrics_node)
#
# The simulation counterpart of run_full_mission.sh, and deliberately shaped like it:
# same staged bringup with readiness checks rather than fixed sleeps, same INT->TERM->KILL
# teardown, same run_logs/ layout. Two things it does NOT need, because there is no
# hardware to fight over: a Fast DDS discovery server (RViz runs on this machine — just
# `rviz2` in another terminal) and any serial-port or dialout handling.
#
# Usage:
#   ./run_sim_mission.sh                          # full mission, GUI Gazebo
#   ./run_sim_mission.sh --policy exhaustion      # explore-then-sweep baseline
#   ./run_sim_mission.sh --metrics                # record run_logs/<ts>/metrics.jsonl
#   ./run_sim_mission.sh --layout layouts/a.yaml  # ground-truth cubes (implies --metrics)
#   ./run_sim_mission.sh --duration 600           # stop automatically after 600s
#   ./run_sim_mission.sh --headless               # no Gazebo GUI (faster, for batch runs)
#   ./run_sim_mission.sh --no-mission             # stack only; drive by hand
#   ./run_sim_mission.sh --build                  # colcon build first
#   ./run_sim_mission.sh --yes                    # clear leftover processes without asking
#
# THE COVERAGE-GAP MEASUREMENT (the go/no-go gate for the research):
#
#   ./run_sim_mission.sh --policy exhaustion --metrics --duration 900 --headless
#
# That runs plain frontier exploration with the camera-coverage mask recording but not
# influencing behaviour, which is exactly the "how much of the mapped floor was never
# camera-inspected?" measurement. It needs no cubes in the arena at all — the number is
# pure exploration geometry, so the fact that YOLO barely fires on Gazebo's untextured
# boxes does not affect it. Read swept_fraction from the run_end record in metrics.jsonl.
#
# Ctrl+C once: stops the robot and tears everything down cleanly.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${REPO_ROOT}/botzilla_Workspace"
ROS_SETUP=/opt/ros/jazzy/setup.bash
LOG_DIR="${REPO_ROOT}/run_logs/sim-$(date +%Y%m%d-%H%M%S)"

RUN_MISSION=1
DO_BUILD=0
FORCE_CLEAN=0
HEADLESS=false
WITH_METRICS=0
CUBE_LAYOUT=""
POLICY="fraction"
DURATION=0
WITH_RVIZ=0
RVIZ_CONFIG="${HOME}/botzilla_nav_debug.rviz"
# Which X display the GUIs open on. Defaults to whatever this shell has, which is empty
# over ssh — hence --display, so a run started from an ssh session can still put Gazebo
# and RViz on the machine's own monitor. Find it with `who` (the ":N" column).
GUI_DISPLAY="${DISPLAY:-}"

while [ $# -gt 0 ]; do
    case "$1" in
        --no-mission)  RUN_MISSION=0 ;;
        --build)       DO_BUILD=1 ;;
        -y|--yes)      FORCE_CLEAN=1 ;;
        --headless)    HEADLESS=true ;;
        --metrics)     WITH_METRICS=1 ;;
        --layout)      CUBE_LAYOUT="${2:?--layout needs a file}"; WITH_METRICS=1; shift ;;
        --policy)      POLICY="${2:?--policy needs fraction|exhaustion}"; shift ;;
        --duration)    DURATION="${2:?--duration needs seconds}"; shift ;;
        --rviz)        WITH_RVIZ=1 ;;
        --rviz-config) RVIZ_CONFIG="${2:?--rviz-config needs a file}"; WITH_RVIZ=1; shift ;;
        --display)     GUI_DISPLAY="${2:?--display needs e.g. :1}"; shift ;;
        -h|--help)     sed -n '2,40p' "$0"; exit 0 ;;
        *)             echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

case "$POLICY" in
    fraction|exhaustion) ;;
    *) echo "ERROR: --policy must be 'fraction' or 'exhaustion', got '$POLICY'" >&2; exit 2 ;;
esac

# ── output helpers ───────────────────────────────────────────────────────────
if [ -t 1 ]; then C_OK=$'\e[32m'; C_ERR=$'\e[31m'; C_INF=$'\e[36m'; C_WARN=$'\e[33m'; C_0=$'\e[0m'
else C_OK=; C_ERR=; C_INF=; C_WARN=; C_0=; fi
step() { echo "${C_INF}==>${C_0} $*"; }
ok()   { echo "${C_OK}  ok${C_0} $*"; }
warn() { echo "${C_WARN}  !!${C_0} $*"; }
die()  { echo "${C_ERR}ERROR:${C_0} $*" >&2; exit 1; }

PIDS=()
CLEANED=0

# ROS 2's setup.bash reads unbound variables (AMENT_TRACE_SETUP_FILES and friends), so it
# aborts instantly under `set -u`. Every caller is a subshell, so toggling -u here cannot
# leak back into the script's own checks.
source_ros() {
    set +u
    # shellcheck disable=SC1090,SC1091
    source "$ROS_SETUP"
    [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
    set -u
}

# Everything this script starts, for pgrep-based checks and forced cleanup. Gazebo's own
# server/GUI processes are included: `ros2 launch` does not always take them down, and a
# surviving gz server holds the world and silently breaks the next run's spawn.
LAUNCH_PATTERN="ros2 launch botzilla|gz sim|ruby.*gz sim|gzserver|gzclient|parameter_bridge|robot_state_publisher|ekf_node|odom_covariance_relay|pointcloud_to_laserscan|yolo_node|rtabmap|controller_server|planner_server|behavior_server|bt_navigator|velocity_smoother|lifecycle_manager|executor_node|frontier_explorer_node|mission_metrics_node|rviz2"

cleanup() {
    [ "$CLEANED" = 1 ] && return
    CLEANED=1
    echo
    step "shutting down"

    # Stop the robot first, while the graph is still alive to carry the message. Reported
    # honestly: if the graph is already gone this cannot get through, and claiming
    # otherwise would hide a robot still driving in the sim.
    if ( source_ros; timeout 8 ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}" ) >/dev/null 2>&1
    then ok "sent zero /cmd_vel"
    else warn "could not publish zero /cmd_vel (graph already down?)"
    fi

    # Reverse start order: consumers before producers.
    for (( i=${#PIDS[@]}-1 ; i>=0 ; i-- )); do
        kill -INT "${PIDS[i]}" 2>/dev/null || true
    done

    for _ in $(seq 1 10); do
        pgrep -f "$LAUNCH_PATTERN" >/dev/null 2>&1 || break
        sleep 1
    done

    # Escalate INT -> TERM -> KILL rather than jumping straight to KILL: a KILL denies
    # every node the chance to shut down tidily, and RTAB-Map in particular wants to close
    # its database. Gazebo is the usual survivor here.
    local survivors
    survivors=$(pgrep -f "$LAUNCH_PATTERN" 2>/dev/null | grep -v "^$$\$" || true)
    if [ -n "$survivors" ]; then
        echo "$survivors" | while read -r p; do kill -TERM "$p" 2>/dev/null || true; done
        for _ in $(seq 1 5); do
            pgrep -f "$LAUNCH_PATTERN" >/dev/null 2>&1 || break
            sleep 1
        done
    fi
    survivors=$(pgrep -f "$LAUNCH_PATTERN" 2>/dev/null | grep -v "^$$\$" || true)
    if [ -n "$survivors" ]; then
        warn "forcing (SIGKILL): $(echo "$survivors" | tr '\n' ' ')"
        echo "$survivors" | while read -r p; do kill -9 "$p" 2>/dev/null || true; done
        sleep 1
    fi

    ( source_ros; ros2 daemon stop ) >/dev/null 2>&1 || true
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

    if pgrep -f "$LAUNCH_PATTERN" >/dev/null 2>&1; then
        warn "some processes survived — check: pgrep -fa '$LAUNCH_PATTERN'"
    else
        ok "all processes stopped"
    fi

    if [ "$WITH_METRICS" = 1 ] && [ -s "$LOG_DIR/metrics.jsonl" ]; then
        echo
        step "metrics summary"
        python3 - "$LOG_DIR/metrics.jsonl" <<'PY' || warn "could not summarise metrics"
import json, sys
end = None
counts = {}
for line in open(sys.argv[1]):
    try: r = json.loads(line)
    except ValueError: continue
    counts[r['type']] = counts.get(r['type'], 0) + 1
    if r['type'] == 'run_end':
        end = r
print('  records      :', counts)
if end:
    frac = end.get('swept_fraction')
    print(f"  distance     : {end.get('distance_m')} m")
    print(f"  free cells   : {end.get('swept_free')}/{end.get('total_free')}")
    print(f"  CAMERA-SWEPT : {frac if frac is None else f'{frac*100:.1f}%'}"
          "   <- the coverage-gap number")
    print(f"  cubes        : {end.get('cubes_inspected')}/{end.get('cubes_in_layout')} inspected,"
          f" {end.get('delivered')} delivered")
else:
    print('  (no run_end record — metrics node did not shut down cleanly)')
PY
    fi
    echo "logs: $LOG_DIR"
}
# Ctrl+C must exit outright. With a bare `trap cleanup INT` the shell resumes the
# interrupted sleep afterwards and the watch loop reports the processes cleanup just
# stopped as an unexpected crash — alarming and untrue.
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

# ── wait helpers (readiness, not guesswork) ──────────────────────────────────
# Fixed sleeps are why bringup is flaky: RTAB-Map needs the camera streaming before it
# publishes a map, and Nav2 needs that map before it will activate.
clear_line() { printf '\r%*s\r' 72 ''; }
wait_for_log() {   # <file> <regex> <timeout_s> <what>
    local f=$1 re=$2 t=$3 what=$4 i=0
    while [ "$i" -lt "$t" ]; do
        if grep -qE "$re" "$f" 2>/dev/null; then clear_line; ok "$what"; return 0; fi
        if grep -qE "Traceback|process has died" "$f" 2>/dev/null; then
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
      # Gazebo's GUI and RViz need a display. An ssh shell has none, so without this the
      # GUI silently fails to open while the rest of the stack comes up fine — which
      # looks like "Gazebo didn't start" rather than "no display".
      if [ -n "$GUI_DISPLAY" ]; then
          export DISPLAY="$GUI_DISPLAY"
          [ -n "${GUI_XAUTHORITY:-}" ] && export XAUTHORITY="$GUI_XAUTHORITY"
      fi
      exec "$@" ) > "$log" 2>&1 &
    PIDS+=("$!")
}

# ── preflight ────────────────────────────────────────────────────────────────
mkdir -p "$LOG_DIR"

step "preflight"

# A leftover gz server or node from a previous run is the single most common cause of a
# broken sim bringup: the spawn silently fails against the old world, or two EKFs publish
# the same TF and the map tears.
STALE=$(pgrep -f "$LAUNCH_PATTERN" 2>/dev/null | grep -v "^$$\$" || true)
if [ -n "$STALE" ]; then
    warn "leftover processes from a previous run:"
    pgrep -af "$LAUNCH_PATTERN" 2>/dev/null | grep -v "^$$ " | sed 's/^/     /'
    if [ "$FORCE_CLEAN" = 1 ]; then
        echo "  --yes given, clearing them"
    elif [ -t 0 ]; then
        read -r -p "  kill them and continue? [y/N] " reply
        [ "$reply" = "y" ] || [ "$reply" = "Y" ] || die "aborted"
    else
        die "stale processes present and no terminal to confirm at. Re-run with --yes."
    fi
    echo "$STALE" | while read -r p; do kill -TERM "$p" 2>/dev/null || true; done
    sleep 2
    echo "$STALE" | while read -r p; do kill -9 "$p" 2>/dev/null || true; done
    ok "cleared"
else
    ok "no leftover processes"
fi

if [ -n "$CUBE_LAYOUT" ]; then
    [ -f "$CUBE_LAYOUT" ] || die "layout file not found: $CUBE_LAYOUT"
    CUBE_LAYOUT="$(cd "$(dirname "$CUBE_LAYOUT")" && pwd)/$(basename "$CUBE_LAYOUT")"
    ok "cube layout: $CUBE_LAYOUT"
fi

if [ "$DO_BUILD" = 1 ]; then
    step "building"
    ( cd "$WS" && source_ros && colcon build --symlink-install ) || die "build failed"
    ok "built"
fi
[ -d "$WS/install" ] || die "$WS/install not found — run with --build first"

command -v gz >/dev/null 2>&1 || warn "'gz' not on PATH — Gazebo Harmonic may not be installed"

# Resolve the display once, here, rather than letting each GUI fail on its own later.
if [ "$HEADLESS" = false ] || [ "$WITH_RVIZ" = 1 ]; then
    if [ -z "$GUI_DISPLAY" ]; then
        die "GUIs requested but no DISPLAY. Pass --display :N (find N with \`who\`), or --headless."
    fi
    for xa in "${XAUTHORITY:-}" "/run/user/$(id -u)/gdm/Xauthority" "$HOME/.Xauthority"; do
        [ -n "$xa" ] && [ -f "$xa" ] || continue
        if DISPLAY="$GUI_DISPLAY" XAUTHORITY="$xa" timeout 5 xdpyinfo >/dev/null 2>&1; then
            GUI_XAUTHORITY="$xa"; break
        fi
    done
    [ -n "${GUI_XAUTHORITY:-}" ] || die "cannot open display $GUI_DISPLAY (no usable Xauthority)"
    GUI_RES=$(DISPLAY="$GUI_DISPLAY" XAUTHORITY="$GUI_XAUTHORITY" xdpyinfo 2>/dev/null \
              | awk '/dimensions:/{print $2; exit}')
    ok "display $GUI_DISPLAY (${GUI_RES:-unknown})"
    # Gazebo's GUI plus RViz on a 640x480 desktop is unusable, and a 640x480 desktop on an
    # external monitor almost always means EDID was not read (mode list has one entry,
    # physical size reads 0mm) — replugging the cable usually fixes it.
    case "${GUI_RES:-}" in
        640x480|800x600)
            warn "display is only ${GUI_RES} — Gazebo + RViz will not fit."
            warn "  Usually EDID was not read. Replug the monitor, or:"
            warn "    DISPLAY=$GUI_DISPLAY xrandr --output DP-0 --auto"
            ;;
    esac
fi

if [ "$WITH_RVIZ" = 1 ] && [ ! -f "$RVIZ_CONFIG" ]; then
    warn "RViz config $RVIZ_CONFIG not found — RViz will open empty"
    RVIZ_CONFIG=""
fi

ok "policy=$POLICY headless=$HEADLESS metrics=$WITH_METRICS rviz=$WITH_RVIZ"

# ── 1. Gazebo + bridge + EKF + YOLO ──────────────────────────────────────────
step "1/5 Gazebo, bridge, EKF, YOLO"
start_bg "$LOG_DIR/simulation.log" \
    ros2 launch botzilla_bringup simulation.launch.py \
        use_sim_time:=true gz_headless:="$HEADLESS"
# The EKF is the last thing in this group to come up and the one everything downstream
# depends on for odom->base_link, so it is the honest readiness signal for the group.
wait_for_log "$LOG_DIR/simulation.log" \
    "ekf_node|Initializing filter|robot_localization" 120 "Gazebo + EKF up"

# simulation.launch.py starts yolo_node natively. On a machine without `ultralytics` in
# the system Python it dies immediately — on hardware run_full_mission.sh sidesteps this
# by running YOLO in a GPU container instead. That is NOT fatal here and must not abort
# the run: the coverage-gap measurement is pure exploration geometry and needs no cube
# detection at all. But it has to be SAID, because a silently-absent detector would
# otherwise look like a mission that simply never found any cubes.
sleep 3
if grep -qE "yolo_node.*process has died" "$LOG_DIR/simulation.log" 2>/dev/null; then
    if grep -q "No module named 'ultralytics'" "$LOG_DIR/simulation.log" 2>/dev/null; then
        warn "yolo_node died: 'ultralytics' is not installed in this Python."
    else
        warn "yolo_node died — see $LOG_DIR/simulation.log"
    fi
    warn "  no cube detection this run. Fine for --policy exhaustion coverage runs;"
    warn "  a cube-collection run needs it (pip install ultralytics, or the sim"
    warn "  frustum detector once that exists)."
    YOLO_UP=0
else
    ok "yolo_node running"
    YOLO_UP=1
fi

# ── 2. RTAB-Map SLAM ─────────────────────────────────────────────────────────
# depth_topic and use_sim_time both already default to the sim values, but they are
# passed explicitly so this file records what the stack is actually running on.
step "2/5 RTAB-Map SLAM"
start_bg "$LOG_DIR/rtabmap.log" \
    ros2 launch botzilla_navigation rtabmap.launch.py \
        use_sim_time:=true depth_topic:=/camera/depth/image_raw
wait_for_log "$LOG_DIR/rtabmap.log" "rtabmap \([0-9]+\)" 120 "RTAB-Map processing frames"

# ── 3. Nav2 ──────────────────────────────────────────────────────────────────
# use_sim_time:=true also layers config/nav2_params_sim.yaml over the hardware params —
# see nav2.launch.py's docstring. Not optional: without it the robot stalls in a way that
# reads as a planner bug.
step "3/5 Nav2"
start_bg "$LOG_DIR/nav2.log" \
    ros2 launch botzilla_navigation nav2.launch.py use_sim_time:=true
wait_for_log "$LOG_DIR/nav2.log" "Managed nodes are active" 120 "Nav2 lifecycle active"

# ── 4. Metrics recorder (passive) ────────────────────────────────────────────
# Started BEFORE the mission so the first frontier goal is already inside the recording.
# This node is subscribe-only by construction (see its module docstring and
# test_mission_metrics.py), so it cannot influence the run it is measuring.
if [ "$WITH_METRICS" = 1 ]; then
    step "4/5 metrics recorder"
    METRICS_ARGS=(--ros-args -p use_sim_time:=true -p output_path:="$LOG_DIR/metrics.jsonl")
    [ -n "$CUBE_LAYOUT" ] && METRICS_ARGS+=(-p cube_layout:="$CUBE_LAYOUT")
    start_bg "$LOG_DIR/metrics.log" \
        ros2 run botzilla_navigation mission_metrics_node "${METRICS_ARGS[@]}"
    wait_for_log "$LOG_DIR/metrics.log" "Metrics ->" 30 "metrics recording"
else
    step "4/5 metrics recorder — skipped (pass --metrics)"
fi

# ── 5. Mission ───────────────────────────────────────────────────────────────
if [ "$RUN_MISSION" = 1 ]; then
    if [ "${YOLO_UP:-1}" = 0 ] && [ "$POLICY" != "exhaustion" ]; then
        warn "running a collection mission with no detector — the robot will explore"
        warn "and sweep, but will never find a cube. Intended?"
    fi
    step "5/5 executor + frontier explorer (policy=$POLICY)"
    start_bg "$LOG_DIR/executor.log" \
        ros2 launch botzilla_navigation executor.launch.py \
            use_sim_time:=true sweep_trigger_mode:="$POLICY"
    wait_for_log "$LOG_DIR/executor.log" "HOME latched" 120 "HOME latched — mission running"
else
    step "5/5 mission — skipped (--no-mission); drive with teleop or RViz goals"
fi

# ── RViz ─────────────────────────────────────────────────────────────────────
# Last, so it attaches to a graph that already has /map, TF and the Nav2 topics — RViz
# started before them shows fixed-frame errors and needs a manual reset.
if [ "$WITH_RVIZ" = 1 ]; then
    step "RViz"
    RVIZ_ARGS=()
    [ -n "$RVIZ_CONFIG" ] && RVIZ_ARGS+=(-d "$RVIZ_CONFIG")
    # use_sim_time is not optional here: everything is stamped with /clock, and RViz on
    # wall-clock rejects every transform as too old and renders an empty scene.
    start_bg "$LOG_DIR/rviz.log" \
        ros2 run rviz2 rviz2 "${RVIZ_ARGS[@]}" --ros-args -p use_sim_time:=true
    ok "RViz starting on $GUI_DISPLAY"
fi

echo
ok "stack is up"
echo
echo "  RViz:        rviz2      (add /map, /swept_coverage_map, /scan, TF)"
echo "  coverage:    ros2 topic echo /swept_coverage_map --once | head"
echo "  mission:     ros2 topic echo /mission/status"
[ "$WITH_METRICS" = 1 ] && echo "  metrics:     tail -f $LOG_DIR/metrics.jsonl"
echo
if [ "$DURATION" -gt 0 ]; then
    echo "  running for ${DURATION}s, then stopping automatically (Ctrl+C to stop sooner)"
else
    echo "  Ctrl+C to stop."
fi
echo

# ── watch ────────────────────────────────────────────────────────────────────
# Report a crashed stage rather than sitting there looking healthy while half the graph
# is gone — the failure mode that wastes a whole run.
ELAPSED=0
while true; do
    sleep 5
    ELAPSED=$((ELAPSED + 5))
    for pid in "${PIDS[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            warn "a stage exited unexpectedly (pid $pid) — see $LOG_DIR"
            exit 1
        fi
    done
    if [ "$DURATION" -gt 0 ] && [ "$ELAPSED" -ge "$DURATION" ]; then
        echo
        ok "reached --duration ${DURATION}s"
        exit 0
    fi
done
