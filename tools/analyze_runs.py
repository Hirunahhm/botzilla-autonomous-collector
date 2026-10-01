#!/usr/bin/env python3
"""
analyze_runs.py — turn run_logs/<run>/metrics.jsonl into the research plan's numbers.

For every run directory holding a run_config.json and a metrics.jsonl, computes the
metrics of research_discussion.md §6.3 / the Research Plan "Metrics" section, all
measured inside a fixed time budget (default 15 min) from the moment the mission starts
(the first EXPLORING state, else the first pose):

  primary     cubes inspected (%) and cubes detected (%) within the budget, and the
              time to each cube
  supporting  camera coverage at 5/10/15 min and its area under the curve (the mean
              coverage over the budget), distance, turning, stalls, false detections

Coverage uses the fixed floor-area denominator when the run recorded one
(swept_fraction_of_floor), else the known-free share, and says which.

Arms are labelled from run_config.json: the strategy (and inspection mode for
'region'), or for 'sweep' the policy — e.g. region/mixed, heats, camera_greedy,
sweep/exhaustion (arm B), sweep/area (arm C).

Full-cycle runs (collection on, `timed_run.sh --collect`) also report the cubes
DELIVERED to HOME within the budget, the time of each delivery, and how many chases were
started (entries into TARGETING).

Offline ground truth (--layout FILE): a layout measured AFTER the runs can still be
applied, because every run records the robot pose at 10 Hz, every detection with its
map position, and the final map (map_final.npz). With --layout, inspected and detected
times are recomputed from those records with the same geometry the metrics node uses
(swept_mask.is_in_frustum + has_line_of_sight on the final map; detections matched with
cube_detections.match_detection). Valid only if HOME and the cubes were in the same
places in every run — the map frame is the start pose.

Usage:
  tools/analyze_runs.py run_logs                       # every run under run_logs/
  tools/analyze_runs.py run_logs --all --layout layouts/lab.yaml   # offline ground truth
  tools/analyze_runs.py run_logs --since 20261001      # only runs from a date on
  tools/analyze_runs.py run_logs --csv runs.csv        # also write one row per run
  tools/analyze_runs.py run_logs --reference region/mixed   # rank tests vs this arm

Counted runs only by default: detect-only, a layout, and a clean git tree. Pass
--all to include the rest (they are flagged).
"""

import argparse
import csv
import json
import math
import os
import statistics
import sys

try:
    from scipy import stats
except ImportError:   # the summary still works; only the p-values need scipy
    stats = None

CHECKPOINTS_MIN = (5, 10, 15)

# The pure geometry helpers live in the workspace package; import them from source so
# this script runs without sourcing ROS.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                                'botzilla_Workspace', 'src', 'botzilla_navigation'))


def load_layout(path):
    """[(id, x, y), ...] from a layout YAML (same format mission_metrics_node reads)."""
    import yaml
    with open(path) as fh:
        doc = yaml.safe_load(fh) or {}
    return [(e.get('id', i + 1), float(e['x']), float(e['y']))
            for i, e in enumerate(doc.get('cubes') or [])]


def offline_ground_truth(run_dir, records, layout, t0, t_end):
    """Inspected / detected times per cube, recomputed from the logs — see docstring."""
    from botzilla_navigation.cube_detections import match_detection
    from botzilla_navigation.swept_mask import has_line_of_sight, is_in_frustum
    import numpy as np
    start = next((r for r in records if r.get('type') == 'run_start'), {})
    half_fov = start.get('camera_half_fov_rad', 0.497)
    max_r = start.get('camera_mark_range_m', 1.0)
    min_r = start.get('camera_min_range_m', 0.48)
    grid = None
    map_path = os.path.join(run_dir, 'map_final.npz')
    if os.path.isfile(map_path):
        z = np.load(map_path)
        m = z['map']
        grid = (m.ravel().tolist(), m.shape[1], m.shape[0], float(z['resolution']),
                float(z['origin_x']), float(z['origin_y']))
    inspected, detected = {}, {}
    for r in records:
        if r.get('type') != 'pose' or not t0 <= r['t'] <= t_end:
            continue
        for cid, cx, cy in layout:
            if cid in inspected:
                continue
            if not is_in_frustum(r['x'], r['y'], r['yaw'], cx, cy, half_fov, max_r, min_r):
                continue
            if grid is not None and not has_line_of_sight(*grid, r['x'], r['y'], cx, cy):
                continue
            inspected[cid] = r['t'] - t0
    for r in records:
        if (r.get('type') != 'cube_detected' or not t0 <= r['t'] <= t_end
                or not r.get('in_range') or r.get('map_x') is None):
            continue
        cid, _ = match_detection(r['map_x'], r['map_y'], layout)
        if cid is not None and cid not in detected:
            detected[cid] = r['t'] - t0
    return inspected, detected


def load_run(path):
    """Return (config, records) for a run directory, or None if it is not a run."""
    cfg_path = os.path.join(path, 'run_config.json')
    met_path = os.path.join(path, 'metrics.jsonl')
    if not (os.path.isfile(cfg_path) and os.path.isfile(met_path)):
        return None
    with open(cfg_path) as fh:
        try:
            config = json.load(fh)
        except ValueError:
            return None
    records = []
    with open(met_path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except ValueError:
                pass   # a run killed mid-write leaves one partial last line
    return config, records


def arm_label(config):
    strategy = config.get('strategy') or 'sweep'
    if strategy in ('region', 'interleaved'):
        return f"{strategy}/{config.get('inspection') or 'mixed'}"
    if strategy == 'sweep':
        policy = (config.get('policy') or 'fraction').split()[0]
        return f'sweep/{policy}'
    return strategy


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def analyze(config, records, budget_s, layout=None, run_dir=None):
    """Metrics for one run — see module docstring."""
    start = next((r for r in records if r.get('type') == 'run_start'), {})
    cube_count = len(layout) if layout else (start.get('cube_count') or 0)
    t0 = next((r['t'] for r in records
               if r.get('type') == 'state' and r.get('state') == 'EXPLORING'), None)
    if t0 is None:
        t0 = next((r['t'] for r in records if r.get('type') == 'pose'), 0.0)
    t_end = t0 + budget_s
    last_t = max((r['t'] for r in records), default=t0)

    def within(r):
        return t0 <= r['t'] <= t_end

    if layout:
        inspected, detected = offline_ground_truth(run_dir, records, layout, t0, t_end)
    else:
        inspected = {r['cube_id']: r['t'] - t0 for r in records
                     if r.get('type') == 'cube_inspected' and within(r)}
        detected = {r['cube_id']: r['t'] - t0 for r in records
                    if r.get('type') == 'cube_first_detected' and within(r)}

    # Full-cycle runs: deliveries to HOME (delivered count increments) and chases started.
    deliveries, chases, prev_delivered, prev_state = [], 0, 0, None
    for r in records:
        if r.get('type') != 'state' or 'state' not in r:
            continue
        if within(r):
            if r['state'] == 'TARGETING' and prev_state != 'TARGETING':
                chases += 1
            if r.get('delivered', 0) > prev_delivered:
                deliveries.append(round(r['t'] - t0, 1))
        prev_delivered = max(prev_delivered, r.get('delivered', 0))
        prev_state = r['state']
    false_det = sum(1 for r in records if r.get('type') == 'cube_detected' and within(r)
                    and r.get('in_range') and r.get('cube_id') is None and cube_count)

    coverage = [r for r in records if r.get('type') == 'coverage' and within(r)]
    use_floor = any(r.get('swept_fraction_of_floor') is not None for r in coverage)
    key = 'swept_fraction_of_floor' if use_floor else 'swept_fraction'
    series = [(r['t'] - t0, r.get(key) or 0.0) for r in coverage]

    def coverage_at(minutes):
        t = minutes * 60.0
        if t > last_t - t0 + 30.0:
            return None   # the run ended well before this checkpoint (30 s slack for
            # a run stopped by hand a few seconds short of the budget)
        vals = [v for (ts, v) in series if ts <= t]
        return vals[-1] if vals else 0.0

    # Area under the coverage curve over the budget, as mean coverage (step function).
    auc = None
    if series:
        horizon = min(budget_s, last_t - t0)
        total, prev_t, prev_v = 0.0, 0.0, 0.0
        for ts, v in series:
            total += (ts - prev_t) * prev_v
            prev_t, prev_v = ts, v
        total += max(0.0, horizon - prev_t) * prev_v
        auc = total / budget_s

    poses = [r for r in records if r.get('type') == 'pose' and within(r)]
    distance = (poses[-1]['distance_m'] - poses[0]['distance_m']) if len(poses) > 1 else 0.0
    turning = sum(abs(wrap(b['yaw'] - a['yaw'])) for a, b in zip(poses, poses[1:]))

    events = [r['event'] for r in records if r.get('type') == 'explorer_event' and within(r)]
    stalls = sum(1 for e in events if e.get('event') == 'stall')
    looks = sum(1 for e in events if e.get('event') == 'look_result' and e.get('ok'))
    battery = next((r['voltage'] for r in records if r.get('type') == 'battery'), None)

    return {
        'run': config.get('_dir'),
        'arm': arm_label(config),
        'layout': os.path.basename(config.get('layout') or '') or None,
        'counted': bool(config.get('detect_only')) and bool(config.get('layout'))
        and not config.get('git_dirty', True),
        'git_commit': config.get('git_commit'),
        'duration_min': round((min(last_t, t_end) - t0) / 60.0, 2),
        'cubes': cube_count,
        'inspected_pct': round(100.0 * len(inspected) / cube_count, 1) if cube_count else None,
        'detected_pct': round(100.0 * len(detected) / cube_count, 1) if cube_count else None,
        'inspect_times_s': {str(k): round(v, 1) for k, v in sorted(inspected.items())},
        'detect_times_s': {str(k): round(v, 1) for k, v in sorted(detected.items())},
        'coverage_basis': 'floor' if use_floor else 'known_free',
        **{f'coverage_{m}min': (round(100 * coverage_at(m), 1)
                                if coverage_at(m) is not None else None)
           for m in CHECKPOINTS_MIN},
        'coverage_auc_pct': round(100 * auc, 1) if auc is not None else None,
        'distance_m': round(distance, 1),
        'turning_deg': round(math.degrees(turning)),
        'stalls': stalls,
        'looks': looks,
        'false_detections': false_det,
        'collect': not config.get('detect_only', False),
        'delivered': len(deliveries),
        'delivery_times_s': deliveries,
        'chases': chases,
        'battery_start_v': battery,
    }


def mean_std(values):
    values = [v for v in values if v is not None]
    if not values:
        return '—'
    if len(values) == 1:
        return f'{values[0]:.1f}'
    return f'{statistics.mean(values):.1f} ± {statistics.stdev(values):.1f}'


SUMMARY_COLUMNS = (
    ('inspected_pct', 'inspected %'), ('detected_pct', 'detected %'),
    ('coverage_5min', 'cov 5m'), ('coverage_10min', 'cov 10m'),
    ('coverage_15min', 'cov 15m'), ('coverage_auc_pct', 'cov AUC'),
    ('distance_m', 'dist m'), ('turning_deg', 'turn deg'), ('stalls', 'stalls'),
    ('false_detections', 'false det'), ('delivered', 'delivered'), ('chases', 'chases'),
)


def print_summary(rows, reference):
    arms = sorted({r['arm'] for r in rows})
    print(f'\n## Per arm (mean ± std), {len(rows)} run(s)\n')
    print('| arm | n | ' + ' | '.join(h for _, h in SUMMARY_COLUMNS) + ' |')
    print('|---|---|' + '---|' * len(SUMMARY_COLUMNS))
    for arm in arms:
        sub = [r for r in rows if r['arm'] == arm]
        cells = [mean_std([r[k] for r in sub]) for k, _ in SUMMARY_COLUMNS]
        print(f'| {arm} | {len(sub)} | ' + ' | '.join(cells) + ' |')

    layouts = sorted({r['layout'] for r in rows if r['layout']})
    if layouts:
        print('\n## Cubes inspected % by layout (paired comparison)\n')
        print('| arm | ' + ' | '.join(layouts) + ' |')
        print('|---|' + '---|' * len(layouts))
        for arm in arms:
            cells = [mean_std([r['inspected_pct'] for r in rows
                               if r['arm'] == arm and r['layout'] == lay]) for lay in layouts]
            print(f'| {arm} | ' + ' | '.join(cells) + ' |')

    if reference and reference in arms and stats is not None:
        print(f'\n## Rank tests against {reference} (Mann-Whitney U, two-sided)\n')
        print('Small samples: report the effect sizes above first; p-values are secondary.\n')
        print('| arm | metric | p |')
        print('|---|---|---|')
        ref = [r for r in rows if r['arm'] == reference]
        for arm in arms:
            if arm == reference:
                continue
            sub = [r for r in rows if r['arm'] == arm]
            for key in ('inspected_pct', 'coverage_auc_pct'):
                a = [r[key] for r in ref if r[key] is not None]
                b = [r[key] for r in sub if r[key] is not None]
                if len(a) < 2 or len(b) < 2:
                    continue
                p = stats.mannwhitneyu(a, b, alternative='two-sided').pvalue
                print(f'| {arm} | {key} | {p:.3f} |')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('roots', nargs='+', help='run_logs directories (or single runs)')
    ap.add_argument('--budget-min', type=float, default=15.0)
    ap.add_argument('--since', default='', help='only run dirs named >= this (YYYYMMDD...)')
    ap.add_argument('--all', action='store_true', help='include uncounted runs')
    ap.add_argument('--csv', help='write one row per run to this file')
    ap.add_argument('--reference', default='region/mixed',
                    help='arm the rank tests compare against')
    ap.add_argument('--layout', help='cube layout YAML measured after the runs (offline '
                                     'inspected/detected times; see module docstring)')
    args = ap.parse_args(argv)

    run_dirs = []
    for root in args.roots:
        if os.path.isfile(os.path.join(root, 'metrics.jsonl')):
            run_dirs.append(root)
            continue
        for name in sorted(os.listdir(root)):
            if name >= args.since:
                run_dirs.append(os.path.join(root, name))

    rows, skipped = [], []
    for d in run_dirs:
        loaded = load_run(d)
        if loaded is None:
            continue
        config, records = loaded
        config['_dir'] = os.path.basename(os.path.normpath(d))
        row = analyze(config, records, args.budget_min * 60.0,
                      layout=load_layout(args.layout) if args.layout else None, run_dir=d)
        if row['counted'] or args.all:
            rows.append(row)
        else:
            skipped.append(row['run'])

    if skipped:
        print(f'Skipped {len(skipped)} uncounted run(s) (not detect-only, no layout, or '
              f'dirty tree); --all includes them: {", ".join(skipped)}', file=sys.stderr)
    if not rows:
        print('No runs to analyse.', file=sys.stderr)
        return 1

    print('## Runs\n')
    print('| run | arm | layout | min | inspected % | detected % | cov 15m | AUC | '
          'stalls | delivered | counted |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    for r in rows:
        print(f"| {r['run']} | {r['arm']} | {r['layout'] or '—'} | {r['duration_min']} | "
              f"{r['inspected_pct']} | {r['detected_pct']} | {r['coverage_15min']} | "
              f"{r['coverage_auc_pct']} | {r['stalls']} | "
              f"{r['delivered'] if r['collect'] else '—'} | "
              f"{'yes' if r['counted'] else 'no'} |")
    print_summary(rows, args.reference)

    if args.csv:
        with open(args.csv, 'w', newline='') as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            for r in rows:
                writer.writerow({k: json.dumps(v) if isinstance(v, dict) else v
                                 for k, v in r.items()})
        print(f'\nWrote {args.csv}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
