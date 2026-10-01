#!/usr/bin/env bash
# timed_run.sh — one hardware run with a fixed timer.
#
#   tools/timed_run.sh LABEL MINUTES [--collect] [run_full_mission.sh args...]
#   tools/timed_run.sh heats 5 --strategy heats
#   tools/timed_run.sh heats_full 15 --collect --strategy heats
#
# Starts run_full_mission.sh (--yes --metrics always added, and --detect-only unless
# --collect is given: with --collect the robot chases, collects and delivers cubes to
# HOME — the full mission cycle), waits for
# the executor to latch HOME, runs MINUTES from that moment, then stops the stack with
# SIGTERM. SIGINT would not work: a script started in the background ignores it, which
# once left the robot exploring three minutes past its timer. The run directory is found
# by modification time — run_logs also holds sim-* directories that sort after the
# timestamped ones by name.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LABEL=$1; MINUTES=$2; shift 2
MODE=(--detect-only)
if [ "${1:-}" = "--collect" ]; then MODE=(); shift; fi
cd "$REPO"
before=$(ls -1t run_logs | head -1)
setsid ./run_full_mission.sh --yes "${MODE[@]}" --metrics "$@" \
    > "/tmp/timed_run_$LABEL.out" 2>&1 &
MPID=$!
for _ in $(seq 1 60); do
    sleep 2
    [ "$(ls -1t run_logs | head -1)" != "$before" ] && break
done
LOG=run_logs/$(ls -1t run_logs | head -1)
echo "[$LABEL] log dir $LOG pid $MPID"
latched=0
for _ in $(seq 1 150); do
    sleep 2
    if grep -q "HOME latched" "$LOG/executor.log" 2>/dev/null; then latched=1; break; fi
    kill -0 $MPID 2>/dev/null || break
done
if [ $latched = 0 ]; then
    echo "[$LABEL] HOME never latched — stopping"
    kill -TERM $MPID; sleep 30; exit 1
fi
echo "[$LABEL] HOME latched at $(date +%T); running $MINUTES min"
sleep $((MINUTES * 60))
echo "[$LABEL] timer done at $(date +%T); stopping"
kill -TERM $MPID
for _ in $(seq 1 60); do sleep 2; kill -0 $MPID 2>/dev/null || break; done
kill -0 $MPID 2>/dev/null && echo "[$LABEL] still alive after 120 s" \
    || echo "[$LABEL] stopped cleanly"
echo "[$LABEL] DONE $LOG"
