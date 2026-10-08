# Multi-robot commands

Everything is typed on the **Jetson**, in the repo root
(`~/Desktop/Projects/sem5/final-project-botzilla`). `tools/fleet_run.sh` starts the
collector on the Pi over ssh, so nothing has to be typed on the Pi.

## 1. Before each run

```bash
sudo pkill -x nxnode.bin          # NoMachine starves Nav2; it restarts after every reboot
pgrep -a nxnode.bin               # must print nothing
```

- Leader on its taped start mark; collector on its mark, 1 m to the leader's left, facing the
  same way.
- Cubes back in their places.
- Both robots switched on; the Pi reachable: `ssh pi true`

## 2. One timed two-robot run

```bash
tools/fleet_run.sh M1 10 --strategy interleaved --inspection mixed
```

- `M1` is the label, `10` the minutes (counted from the leader's HOME latch).
- It starts the leader, waits for `STACK IS UP`, starts the collector on the Pi, and at the end
  stops both and copies the collector's log into the leader's run folder.
- Output: `run_logs/<time>/` on the Jetson, with `collector.log` inside it.

## 3. Stopping a run early

Press **Ctrl+C** in the terminal running `fleet_run.sh`. It stops the leader, then the
collector, and waits until both are down.

If that terminal is gone:

```bash
# leader (Jetson): its process group
kill -TERM -$(pgrep -f "[b]ash ./run_full_mission.sh")

# collector (Pi): SIGTERM, not SIGINT. It was started in the background, so it ignores SIGINT.
ssh pi 'kill -TERM $(pgrep -f "[b]ash ./run_collector.sh")'
```

Check that nothing is left:

```bash
pgrep -af "[r]un_full_mission|[r]os2 launch"; docker ps
ssh pi 'pgrep -af "[r]un_collector|[r]os2 launch"'
```

## 4. Scoring a run

```bash
python3 tools/single_vs_multi.py run_logs/<time> --budget-min 10
```

Prints the cubes delivered and their times, tasks, failed tasks, chases, floor seen at 5 and
10 min, AUC, leader distance, stalls and how long the leader was blocked.

## 5. Manual start (two terminals)

If `fleet_run.sh` cannot be used:

```bash
# Jetson
tools/timed_run.sh M1 10 --fleet --strategy interleaved --inspection mixed

# Pi, after the Jetson prints STACK IS UP
ssh pi
cd ~/hiruna/botzilla-autonomous-collector
./run_collector.sh --leader 10.156.103.192 --start 0.0 1.34 0.0
```

Stop the collector with Ctrl+C in its own terminal once the leader's timer ends.

## 6. Settings and troubleshooting

`fleet_run.sh` takes these as environment variables:

| Variable | Default | Meaning |
|---|---|---|
| `PI_HOST` | `10.156.103.37` | the Pi's IP (mDNS is unreliable on the hotspot) |
| `LEADER_IP` | first address of `hostname -I` | the Jetson's IP, passed to the collector |
| `COLLECTOR_START` | `"0.0 1.34 0.0"` | collector start X Y YAW in the leader's map |
| `PI_REPO` | `~/hiruna/botzilla-autonomous-collector` | repo on the Pi |

Example: `COLLECTOR_START="0.0 1.0 0.0" tools/fleet_run.sh M2 10 --strategy interleaved --inspection mixed`

| Message | Fix |
|---|---|
| `NoMachine is running` | `sudo pkill -x nxnode.bin` (or `sudo kill <pid>` from `pgrep -a nxnode.bin`) |
| `cannot ssh to the Pi` | Pi on, on the same hotspot; check its IP and set `PI_HOST` |
| `a collector is already running on the Pi` | stop it (section 3), then rerun |
| `a leader stack is already running here` | stop it (section 3), then rerun |
| Pi's IP or the Jetson's IP changed | set `PI_HOST` / `LEADER_IP` |
