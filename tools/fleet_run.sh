#!/usr/bin/env bash
# fleet_run.sh — one timed two-robot run, both robots started from the Jetson.
#
#   tools/fleet_run.sh LABEL MINUTES [run_full_mission.sh args...]
#   tools/fleet_run.sh fleet28 10 --strategy interleaved --inspection mixed
#
# 1. Starts the leader with tools/timed_run.sh LABEL MINUTES --fleet ARGS (the timer
#    runs MINUTES from the leader's HOME latch, then stops the leader).
# 2. Once the leader prints STACK IS UP, starts ./run_collector.sh on the Pi over ssh.
# 3. When the leader's timer ends, stops the collector (SIGTERM, its own clean
#    teardown) and copies its collector.log into the leader's run directory.
#
# Ctrl+C stops both robots.
#
# Settings (environment variables):
#   PI_SSH           ssh target for the Pi (default: pi, from ~/.ssh/config)
#   PI_HOST          the Pi's address (default 10.156.103.37; mDNS is unreliable on the
#                    hotspot, so the IP is used and groot.local only as the host key alias)
#   PI_REPO          repo on the Pi (default ~/hiruna/botzilla-autonomous-collector)
#   LEADER_IP        this Jetson's address for the collector (default: first of hostname -I)
#   COLLECTOR_START  collector start pose X Y YAW in the leader's map
#                    (default "0.0 1.34 0.0": 1 m gap, on the leader's left)
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[ $# -ge 2 ] || { sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
LABEL=$1; MINUTES=$2; shift 2

PI_SSH=${PI_SSH:-pi}
PI_HOST=${PI_HOST:-10.156.103.37}
PI_REPO=${PI_REPO:-'~/hiruna/botzilla-autonomous-collector'}
LEADER_IP=${LEADER_IP:-$(hostname -I | awk '{print $1}')}
COLLECTOR_START=${COLLECTOR_START:-"0.0 1.34 0.0"}
SSH=(ssh -o ConnectTimeout=8 -o HostName="$PI_HOST" -o HostKeyAlias=groot.local "$PI_SSH")
OUT=/tmp/fleet_run_$LABEL.out          # timed_run.sh's own lines
MISSION_OUT=/tmp/timed_run_$LABEL.out  # run_full_mission.sh output (timed_run writes it)
PI_OUT=/tmp/collector_$LABEL.out       # run_collector.sh output, on the Pi
PI_PID=/tmp/collector_$LABEL.pid

say() { echo "[fleet_run $LABEL] $*"; }
die() { say "ERROR: $*"; exit 1; }

# ── preflight ────────────────────────────────────────────────────────────────
pgrep -x nxnode.bin >/dev/null && die "NoMachine is running (it starves Nav2): sudo pkill -x nxnode.bin"
timeout 15 "${SSH[@]}" true 2>/dev/null || die "cannot ssh to the Pi ($PI_SSH at $PI_HOST)"
# "[b]ash": the remote shell's own command line contains this pattern, and would match a
# plain "bash ./run_collector.sh"; the regex [b] matches "bash" but not the text "[b]ash".
if timeout 15 "${SSH[@]}" 'pgrep -f "[b]ash ./run_collector.sh" >/dev/null'; then
    die "a collector is already running on the Pi; stop it first"
fi
pgrep -f "[b]ash ./run_full_mission.sh" >/dev/null && die "a leader stack is already running here"
say "leader $LEADER_IP, collector start ($COLLECTOR_START), $MINUTES min"

# ── stop both robots on Ctrl+C / exit ─────────────────────────────────────────
TPID=""; MPID=""; COLLECTOR_UP=0; LOG=""
# The collector's state as the Pi reports it: "alive", "dead", or "" when ssh itself
# failed (Wi-Fi blip). Only a definite "dead" may be acted on; an ssh timeout once made
# this script abort a run whose collector was fine.
collector_state() {
    timeout 15 "${SSH[@]}" "P=\$(cat $PI_PID 2>/dev/null); \
        [ -n \"\$P\" ] && kill -0 \$P 2>/dev/null && echo alive || echo dead" 2>/dev/null
}
stop_collector() {
    [ "$COLLECTOR_UP" = 1 ] || return 0
    COLLECTOR_UP=0
    say "stopping the collector"
    # SIGTERM, not SIGINT: run_collector.sh was started in the background by a
    # non-interactive shell, which starts it with SIGINT ignored, and a script cannot
    # trap a signal ignored at entry (2026-10-08). Its TERM trap does the same clean
    # teardown as Ctrl+C.
    for _ in $(seq 1 30); do
        timeout 15 "${SSH[@]}" "P=\$(cat $PI_PID 2>/dev/null); [ -n \"\$P\" ] && kill -TERM \$P" \
            2>/dev/null
        sleep 3
        [ "$(collector_state)" = dead ] && { say "collector stopped"; return 0; }
    done
    say "WARNING: the collector may still be running after 90 s; check the Pi"
}
stop_all() {
    trap '' INT TERM
    say "${1:-interrupted}: stopping both robots"
    [ -n "$TPID" ] && kill -TERM "$TPID" 2>/dev/null
    [ -n "$MPID" ] && kill -TERM -"$MPID" 2>/dev/null    # run_full_mission's process group
    stop_collector
    exit 130
}
trap 'stop_all interrupted' INT TERM

# ── 1. leader ────────────────────────────────────────────────────────────────
rm -f "$MISSION_OUT"
"$REPO/tools/timed_run.sh" "$LABEL" "$MINUTES" --fleet "$@" > "$OUT" 2>&1 &
TPID=$!
for _ in $(seq 1 120); do
    sleep 2
    MPID=$(sed -n 's/.* pid \([0-9]*\)$/\1/p' "$OUT" | head -1)
    grep -q "STACK IS UP" "$MISSION_OUT" 2>/dev/null && break
    kill -0 "$TPID" 2>/dev/null || { cat "$OUT"; die "the leader stopped during start-up"; }
done
grep -q "STACK IS UP" "$MISSION_OUT" 2>/dev/null || stop_all "leader not up after 240 s"
LOG=$(sed -n 's/.*log dir \(run_logs\/[^ ]*\).*/\1/p' "$OUT" | head -1)
say "leader up: $LOG (mission pid $MPID)"

# ── 2. collector ─────────────────────────────────────────────────────────────
# setsid makes run_collector.sh a session leader with its own pid, so $! is the pid to
# signal later; every stream is redirected so ssh returns at once. `cd` must not be
# chained with &&: the & would then background the whole list in a subshell that holds
# the ssh channel open, and $! would be that subshell's pid.
read -r SX SY SYAW <<< "$COLLECTOR_START"
timeout 20 "${SSH[@]}" "cd $PI_REPO || exit 1; setsid nohup ./run_collector.sh --leader $LEADER_IP \
    --start $SX $SY $SYAW > $PI_OUT 2>&1 < /dev/null & echo \$! > $PI_PID; disown" \
    || stop_all "could not start the collector"
COLLECTOR_UP=1
PI_LOG=""
for _ in $(seq 1 60); do
    sleep 3
    PI_LOG=$(timeout 10 "${SSH[@]}" "sed -n 's/^ *Logs: //p' $PI_OUT" 2>/dev/null | head -1)
    [ -n "$PI_LOG" ] && break
    if [ "$(collector_state)" = dead ]; then
        timeout 10 "${SSH[@]}" "tail -20 $PI_OUT"
        stop_all "the collector exited during start-up"
    fi
done
[ -n "$PI_LOG" ] || stop_all "the collector was not up after 180 s"
say "collector up: $PI_LOG"

# ── 3. wait for the leader's timer, then stop the collector ──────────────────
wait "$TPID"
cat "$OUT"
stop_collector
if [ -n "$LOG" ] && timeout 60 scp -q -o HostName="$PI_HOST" -o HostKeyAlias=groot.local \
        "$PI_SSH:$PI_LOG/collector.log" "$REPO/$LOG/collector.log"; then
    echo "$PI_LOG" > "$REPO/$LOG/collector_run_dir.txt"
    say "collector.log copied to $LOG"
else
    say "WARNING: could not copy the collector log ($PI_LOG)"
fi
say "DONE leader $LOG, collector $PI_LOG"
