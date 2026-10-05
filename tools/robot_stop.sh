# shellcheck shell=bash
#
# robot_stop.sh — sourced by run_full_mission.sh and run_collector.sh. Stops the base
# FIRST on shutdown, without depending on DDS.
#
# The old cleanup's first step was publishing a zero /cmd_vel through the discovery
# server. That is the step that failed on 2026-10-05 ("graph already down?"), and it
# could never have worked anyway while Nav2 was alive: the controller and the velocity
# smoother republish at 20 Hz and override it. The robot kept moving while the rest of
# the cleanup stalled, until its processes were killed by hand.
#
# Now, in order:
#   1. stop every process that commands velocity (by exact process name, so nothing
#      that merely mentions a node name on its command line is touched);
#   2. with no commander left, kobuki_base_node's own watchdog (CMD_VEL_TIMEOUT_S,
#      0.5 s) halts the motors;
#   3. stop the base driver, which zeroes the motors again on exit (every exit path,
#      see kobuki_base_node.main).
#
# Process names are /proc comm values: the executable's (or Python script's) basename,
# cut to 15 characters.

MOTION_COMMANDERS=(controller_serv behavior_server velocity_smooth collision_monit
                   bt_navigator executor_node frontier_explor collector_node)
BASE_DRIVER=kobuki_base_nod

_pids_by_name() {   # one pgrep for all names: pgrep -x takes an extended regex
    local IFS='|'
    pgrep -x "$*"
}

_wait_gone() {   # <seconds> <names...>
    local deadline=$(( $(date +%s%N) + ${1%.*} * 1000000000 )); shift
    while [ "$(date +%s%N)" -lt "$deadline" ]; do
        [ -z "$(_pids_by_name "$@")" ] && return 0
        sleep 0.05
    done
    [ -z "$(_pids_by_name "$@")" ]
}

# Prints one line saying what happened; always returns 0.
stop_robot_motion() {
    local pids
    pids=$(_pids_by_name "${MOTION_COMMANDERS[@]}")
    if [ -n "$pids" ]; then
        # shellcheck disable=SC2086
        kill -INT $pids 2>/dev/null
        if ! _wait_gone 2 "${MOTION_COMMANDERS[@]}"; then
            pids=$(_pids_by_name "${MOTION_COMMANDERS[@]}")
            # shellcheck disable=SC2086
            [ -n "$pids" ] && kill -KILL $pids 2>/dev/null
        fi
    fi
    # Nothing commands velocity any more: give the base watchdog time to trip.
    sleep 0.7
    pids=$(_pids_by_name "$BASE_DRIVER")
    if [ -z "$pids" ]; then
        echo "base driver not running"
        return 0
    fi
    # shellcheck disable=SC2086
    kill -INT $pids 2>/dev/null
    if _wait_gone 4 "$BASE_DRIVER"; then
        echo "robot stopped: commanders gone, base driver exited and zeroed the motors"
        return 0
    fi
    pids=$(_pids_by_name "$BASE_DRIVER")
    # shellcheck disable=SC2086
    kill -TERM $pids 2>/dev/null
    _wait_gone 2 "$BASE_DRIVER" || {
        pids=$(_pids_by_name "$BASE_DRIVER")
        # shellcheck disable=SC2086
        [ -n "$pids" ] && kill -KILL $pids 2>/dev/null
    }
    echo "robot stopped by the base watchdog; the base driver had to be forced to exit"
}

# pgrep -f <pattern>, minus this script and any shell/ssh/grep whose command line merely
# mentions the pattern — killing those once took down an operator's monitoring shells.
pattern_pids() {
    local p c
    for p in $(pgrep -f "$1" 2>/dev/null); do
        [ "$p" = "$$" ] && continue
        c=$(ps -o comm= -p "$p" 2>/dev/null)
        case "$c" in
            ''|bash|sh|dash|zsh|ssh|sshd|sudo|timeout|grep|pgrep|tail|less|watch) continue ;;
        esac
        echo "$p"
    done
}
