#!/usr/bin/env python3
"""single_vs_multi.py — score the single-robot vs two-robot runs the same way.

    python3 tools/single_vs_multi.py run_logs/<leader-run> [...] [--budget-min 10]

One row per run (single_vs_multi_runs.md §2). For a two-robot run the collector's log
must be copied into the leader's run directory as collector.log (the Pi keeps it under
run_logs/collector-<time>/).

- Deliveries: two-robot runs from the fleet manager's "result: collected" events (the
  collector reports COLLECTED only for a release at HOME); single-robot runs from the
  executor's delivered count, as tools/analyze_runs.py does. Times are seconds from the
  searching robot's start of exploring (its HOME latch).
- Floor seen: the searching robot's swept_m2 at 5 and 10 min, and its AUC against the
  fixed 59.2 m² basis of full_cycle_runs.md.
- Leader blocked: seconds with at least one DWB "Hits Obstacle" rejection, as % of the
  budget.
"""
import argparse
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_runs import load_run, analyze  # noqa: E402

FLOOR_M2 = 59.2
STAMP = re.compile(r'\[(\d{10}\.\d+)\]')


def stamps(path, pattern):
    if not os.path.isfile(path):
        return []
    out = []
    with open(path, errors='replace') as fh:
        for line in fh:
            if pattern in line:
                m = STAMP.search(line)
                if m:
                    out.append((float(m.group(1)), line))
    return out


def score(run_dir, budget_s):
    config, records = load_run(run_dir)
    start = next(r for r in records if r.get('type') == 'run_start')
    t0 = next((r['t'] for r in records
               if r.get('type') == 'state' and r.get('state') == 'EXPLORING'), None)
    if t0 is None:
        t0 = next(r['t'] for r in records if r.get('type') == 'pose')
    wall0 = start['wall_clock_start'] + t0
    wall_end = wall0 + budget_s

    # Floor seen by the searching robot's camera.
    cov = [(r['t'] - t0, r['swept_m2']) for r in records
           if r.get('type') == 'coverage' and t0 <= r['t'] <= t0 + budget_s]

    def seen_at(minutes):
        vals = [v for (ts, v) in cov if ts <= minutes * 60.0]
        return round(vals[-1], 1) if vals else 0.0
    total, prev_t, prev_v = 0.0, 0.0, 0.0
    for ts, v in cov:
        total += (ts - prev_t) * prev_v
        prev_t, prev_v = ts, v
    total += max(0.0, budget_s - prev_t) * prev_v
    auc = 100.0 * total / budget_s / FLOOR_M2

    base = analyze(config, records, budget_s)
    blocked = {int(t) for t, _ in stamps(os.path.join(run_dir, 'nav2.log'), 'Hits Obstacle')
               if wall0 <= t <= wall_end}

    row = {
        'run': os.path.basename(run_dir.rstrip('/')),
        'setup': 'M' if config.get('fleet') else 'S',
        'seen_5': seen_at(5), 'seen_10': seen_at(10), 'auc_pct': round(auc, 1),
        'leader_distance_m': base['distance_m'], 'stalls': base['stalls'],
        'leader_blocked_pct': round(100.0 * len(blocked) / budget_s, 1),
    }

    if config.get('fleet'):
        events = []
        with open(os.path.join(run_dir, 'fleet.log'), errors='replace') as fh:
            for line in fh:
                if 'FLEET ' in line:
                    events.append(json.loads(line.split('FLEET ', 1)[1]))
        events = [e for e in events if wall0 <= e['t'] <= wall_end]
        assigns = {e['task']: e for e in events if e['event'] == 'assign'}
        results = [e for e in events if e['event'] == 'result']
        row['delivery_times_s'] = [round(e['t'] - wall0) for e in results
                                   if e['result'] == 'collected']
        row['cubes_delivered'] = sorted({e['id'] for e in results if e['result'] == 'collected'})
        row['tasks'] = len(assigns)
        row['failed_tasks'] = [f"{round(e['t'] - wall0)} s: {e.get('detail', '')}"
                               for e in results if e['result'] != 'collected']
        row['released_short'] = sum(1 for e in results if 'release' in e.get('detail', ''))
        clog = os.path.join(run_dir, 'collector.log')
        trans = [(t, l) for t, l in stamps(clog, '] -> [') if wall0 <= t <= wall_end]
        chases = sum(1 for _, l in trans if '-> [TARGETING]' in l)
        lost = sum(1 for _, l in trans
                   if re.search(r'\[(TARGETING|APPROACHING)\] -> \[(?!APPROACHING|CAPTURING)', l))
        row['chases'], row['chases_lost'] = chases, lost
        row['goal_timeouts'] = len([1 for t, _ in stamps(clog, 'Failed to send goal response')
                                    if wall0 <= t <= wall_end])
    else:
        row['delivery_times_s'] = [round(t) for t in base['delivery_times_s']]
        row['chases'] = base['chases']
        states = [r for r in records if r.get('type') == 'state' and 'state' in r
                  and t0 <= r['t'] <= t0 + budget_s]
        row['chases_lost'] = sum(1 for a, b in zip(states, states[1:])
                                 if a['state'] in ('TARGETING', 'APPROACHING')
                                 and b['state'] not in ('APPROACHING', 'CAPTURING'))
        row['released_short'] = max((r.get('released_short', 0) for r in states), default=0)
    row['delivered'] = len(row['delivery_times_s'])
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--budget-min', type=float, default=10.0)
    args = ap.parse_args()
    for run in args.runs:
        print(json.dumps(score(run, args.budget_min * 60.0), indent=1))


if __name__ == '__main__':
    main()
