#!/usr/bin/env python3
"""
make_poster_figures.py — the charts, lab map and diagrams for the A0 exhibition poster.

    python3 tools/make_poster_figures.py            # -> poster/figures/*.png

Each figure is sized to its slot on the Canva poster (3179 x 4494 canvas units for
A0, so 1 unit prints as 0.75 pt) and rendered at 3x for print. Every label is at least
32 units (24 pt printed): the first version converted units to points with a stray
0.72 factor and printed its figure text at 9-17 pt.

Numbers: Phase 1 from full_cycle_runs.md / extended_abstract_v3.tex (one 15-min
hardware run per configuration); Phase 2 from the logs listed in RUNS below (the same
runs as multi_robot_runs.md); the lab map from run 27 (run_logs/20261007-221643).

Colours are the poster's: teal = Phase 1, orange = Phase 2, grey = context. Checked for
colour-vision deficiency (OKLab, Machado 2009): every pair >= 13.5 under protan /
deutan / tritan simulation, >= 23 for normal vision.
"""
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Wedge, Circle  # noqa: E402
import numpy as np  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / 'poster' / 'figures'
LOGS = REPO / 'run_logs'
RUN27 = LOGS / '20261007-221643'

TEAL, ORANGE, GREY = '#1F6F7A', '#C2621B', '#A7B4BF'
INK, INK2, GRID, NAVY = '#1E2A36', '#5B6B7A', '#E3E8ED', '#042034'
SCALE = 3            # image px per Canva canvas unit
DPI = 300
PT = SCALE * 72 / DPI   # figure points per canvas unit (prints as 0.75 pt on A0)

# Two-robot test runs (multi_robot_runs.md): run number -> leader log dir. Run 22 was
# cancelled before any data.
RUNS = {1: '20261005-184223', 2: '20261005-212043', 3: '20261005-214048',
        4: '20261005-221125', 5: '20261006-002941', 6: '20261006-005735',
        7: '20261006-181824', 8: '20261006-183558', 9: '20261006-190324',
        10: '20261006-224929', 11: '20261006-232154', 12: '20261007-004945',
        13: '20261007-010955', 14: '20261007-013717', 15: '20261007-101042',
        16: '20261007-102125', 17: '20261007-173130', 18: '20261007-183105',
        19: '20261007-183841', 20: '20261007-184342', 21: '20261007-200547',
        23: '20261007-210202', 24: '20261007-211701', 25: '20261007-212525',
        26: '20261007-213304', 27: '20261007-221643'}

plt.rcParams.update({
    'font.family': 'Lato', 'text.color': INK, 'axes.labelcolor': INK2,
    'xtick.color': INK2, 'ytick.color': INK2, 'axes.edgecolor': GRID,
    'svg.fonttype': 'none',
})


def fig_for(w_units, h_units):
    return plt.figure(figsize=(w_units * SCALE / DPI, h_units * SCALE / DPI), dpi=DPI)


def fs(units):
    """Font size in figure points for a size in Canva canvas units."""
    return units * PT


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / name, dpi=DPI, facecolor='white')
    plt.close(fig)
    print('wrote', OUT / name)


def leader_blocked_share(run_dir):
    """Share of a run (HOME latch to end) in which the leader's controller had no safe
    move: seconds with at least one DWB "ObstacleFootprint/Trajectory Hits Obstacle"
    rejection (it logs up to ~14 a second while blocked, so raw counts overstate)."""
    home = None
    for line in open(run_dir / 'executor.log', errors='replace'):
        if 'HOME latched' in line:
            home = float(re.search(r'\[(\d{10}\.\d+)\]', line).group(1))
            break
    secs, last = set(), None
    for line in open(run_dir / 'nav2.log', errors='replace'):
        m = re.search(r'\[(\d{10}\.\d+)\]', line)
        if not m:
            continue
        t = float(m.group(1))
        last = t
        if 'Hits Obstacle' in line or 'ObstacleFootprint' in line:
            if t >= home:
                secs.add(int(t))
    return len(secs) / (last - home)


# ── Phase 1: coverage AUC by inspection method (slot 1448 x 340) ─────────────────────────
def coverage_chart():
    rows = [  # (label, AUC %, ours?)  region search order, runs F3/F11/F10/F9, and F7
        ('Turn-range viewpoints', 36.4, True),
        ('Spin grid', 31.5, False),
        ('One look per stop', 29.6, False),
        ('Lawnmower rows', 23.0, False),
        ('HEATS-style', 21.1, False),
    ]
    fig = fig_for(1448, 340)
    ax = fig.add_axes([0.335, 0.02, 0.58, 0.80])
    y = np.arange(len(rows))[::-1]
    for yi, (label, v, ours) in zip(y, rows):
        ax.barh(yi, v, height=0.68, color=TEAL if ours else GREY, edgecolor='white',
                linewidth=2)
        ax.text(v + 0.6, yi, f'{v:.1f}%', va='center', fontsize=fs(38),
                fontweight='bold' if ours else 'normal', color=INK)
    ax.set_yticks(y, [r[0] for r in rows], fontsize=fs(38))
    for t, (_, _, ours) in zip(ax.get_yticklabels(), rows):
        t.set_color(INK if ours else INK2)
        t.set_fontweight('bold' if ours else 'normal')
    ax.set_xlim(0, 41)
    ax.set_xticks([])
    ax.tick_params(length=0)
    for sp in ax.spines.values():
        sp.set_visible(False)
    fig.text(0.01, 0.88, 'Floor coverage by inspection method (AUC)', fontsize=fs(44),
             fontweight='bold', color=NAVY)
    save(fig, 'phase1_coverage_by_method.png')


# ── Phase 2: share of each run the leader was blocked (slot 1448 x 300) ─────────────────
def leader_blocked_chart():
    share = {r: 100 * leader_blocked_share(LOGS / d) for r, d in RUNS.items()}
    fig = fig_for(1448, 300)
    ax = fig.add_axes([0.075, 0.18, 0.915, 0.60])
    ax.axvspan(16.5, 27.6, color='#FBEBDD', zorder=0)
    for r, v in share.items():
        ax.bar(r, v, width=0.72, color=ORANGE if r >= 17 else GREY, edgecolor='white',
               linewidth=1.5, zorder=2)
    ax.set_xlim(0.4, 27.6)
    ax.set_ylim(0, 40)
    ax.set_xticks([1, 5, 10, 15, 20, 25], ['1', '5', '10', '15', '20', '25'],
                  fontsize=fs(34))
    ax.set_yticks([0, 10, 20, 30, 40], ['0', '10', '20', '30', '40%'], fontsize=fs(34))
    ax.tick_params(length=0)
    ax.grid(axis='y', color=GRID, linewidth=1.2, zorder=1)
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.text(17.0, 38.5, 'coordination rules', fontsize=fs(36), fontweight='bold',
            color=ORANGE, va='top')
    fig.text(0.075, 0.875, 'Time the leader was blocked, per test run',
             fontsize=fs(42), fontweight='bold', color=NAVY)
    save(fig, 'phase2_leader_blocked_per_run.png')
    return share


# ── Lab map from run 27 (slot 900 x 360) ────────────────────────────────────────────────
def lab_map():
    z = np.load(RUN27 / 'map_final.npz')
    m, s = z['map'], z['swept']
    ox, oy, res = float(z['origin_x']), float(z['origin_y']), float(z['resolution'])
    img = np.ones(m.shape + (3,))
    img[m < 0] = matplotlib.colors.to_rgb('#EEF1F4')
    img[(s > 0) & (m == 0)] = matplotlib.colors.to_rgb('#CDE4E7')   # camera has seen it
    img[m >= 65] = matplotlib.colors.to_rgb(INK)

    xs, ys = [], []
    for line in open(RUN27 / 'metrics.jsonl'):
        r = json.loads(line)
        if r.get('type') == 'pose':
            xs.append(r['x'])
            ys.append(r['y'])
    cubes, delivered, chome = {}, set(), None
    for line in open(RUN27 / 'fleet.log', errors='replace'):
        if 'FLEET ' not in line:
            continue
        e = json.loads(line.split('FLEET ', 1)[1])
        if e['event'] == 'confirmed':
            cubes[e['id']] = (e['x'], e['y'])
        elif e['event'] == 'collector_home':
            chome = (e['x'], e['y'])
        elif e['event'] == 'result' and e['result'] == 'collected':
            delivered.add(e['id'])

    # Crop to the walls (a stray long scan ray would stretch the frame) and rotate 90 deg
    # clockwise so the map is landscape: a map point (x, y) is drawn at (y, -x).
    walls = np.argwhere(m >= 65)
    (j0, i0), (j1, i1) = np.percentile(walls, 0.5, axis=0), np.percentile(walls, 99.5, axis=0)
    pad = 8
    j0, i0 = max(int(j0) - pad, 0), max(int(i0) - pad, 0)
    j1, i1 = min(int(j1) + pad, m.shape[0] - 1), min(int(i1) + pad, m.shape[1] - 1)
    crop = img[j0:j1 + 1, i0:i1 + 1]
    x0, x1 = ox + i0 * res, ox + (i1 + 1) * res
    y0, y1 = oy + j0 * res, oy + (j1 + 1) * res

    def rot(x, y):
        return y, -x
    extent = [y0, y1, -x1, -x0]

    fig = fig_for(900, 360)
    fig.text(0.012, 0.93, 'Lab map, run 27', fontsize=fs(40), fontweight='bold',
             color=NAVY, va='top')
    ax = fig.add_axes([0.0, 0.0, 0.50, 0.80])
    ax.imshow(np.rot90(crop, k=1), origin='lower', extent=extent, interpolation='nearest')
    px, py = rot(np.array(xs), np.array(ys))
    ax.plot(px, py, color=NAVY, linewidth=1.6, alpha=0.75)
    for cid, (cx, cy) in cubes.items():
        got = cid in delivered
        ax.plot(*rot(cx, cy), 's', markersize=11,
                markerfacecolor=ORANGE if got else 'white',
                markeredgecolor=ORANGE, markeredgewidth=2.5)
    ax.plot(*rot(0, 0), '*', markersize=22, color=TEAL, markeredgecolor='white',
            markeredgewidth=1.2)
    if chome:
        ax.plot(*rot(*chome), 'D', markersize=12, color=ORANGE, markeredgecolor='white',
                markeredgewidth=1.2)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect('equal')
    ax.axis('off')
    lx = fig.add_axes([0.52, 0.0, 0.48, 1.0])
    lx.axis('off')
    lx.set_xlim(0, 1)
    lx.set_ylim(0, 1)
    items = [
        ('patch', '#CDE4E7', 'camera-seen floor'),
        ('line', NAVY, 'leader path'),
        ('*', TEAL, 'leader start'),
        ('D', ORANGE, 'collector HOME'),
        ('sfill', ORANGE, 'cube delivered'),
        ('shollow', ORANGE, 'cube not delivered'),
    ]
    for k, (kind, col, text) in enumerate(items):
        yy = 0.90 - k * 0.158
        if kind == 'patch':
            lx.add_patch(FancyBboxPatch((0.02, yy - 0.045), 0.08, 0.09,
                                        boxstyle='round,pad=0,rounding_size=0.01',
                                        color=col))
        elif kind == 'line':
            lx.plot([0.02, 0.10], [yy, yy], color=col, linewidth=2.5)
        elif kind.startswith('s'):
            lx.plot(0.06, yy, 's', markersize=11,
                    markerfacecolor=col if kind == 'sfill' else 'white',
                    markeredgecolor=col, markeredgewidth=2.5)
        else:
            lx.plot(0.06, yy, kind, markersize=18 if kind == '*' else 11, color=col)
        lx.text(0.14, yy, text, va='center', fontsize=fs(32), color=INK)
    save(fig, 'lab_map_run27.png')


# ── Phase 1 diagram: one look vs viewpoint with turn range vs rows (1448 x 250) ────────
def phase1_diagram():
    fig = fig_for(1448, 250)
    titles = ['One look: about 0.5 m²', 'Turn-range viewpoint', 'Lawnmower rows']
    for k in range(3):
        ax = fig.add_axes([0.01 + k * 0.335, 0.0, 0.31, 0.74])
        ax.set_xlim(-1.4, 1.4)
        ax.set_ylim(-0.45, 1.25)
        ax.set_aspect('equal')
        ax.axis('off')
        fig.text(0.01 + k * 0.335 + 0.155, 0.84, titles[k], ha='center',
                 fontsize=fs(40), fontweight='bold', color=NAVY)
        if k == 0:
            ax.add_patch(Wedge((-0.35, 0), 1.0, 90 - 28.5, 90 + 28.5, width=0.5,
                               color=TEAL, alpha=0.85))
            ax.add_patch(Circle((-0.35, 0), 0.17, color=NAVY))
            ax.text(0.30, 0.62, '0.5–1.0 m\n57° cone', fontsize=fs(34), color=INK2,
                    va='center')
        elif k == 1:
            for a in range(-60, 61, 30):
                ax.add_patch(Wedge((0, 0), 1.0, 90 + a - 28.5, 90 + a + 28.5, width=0.5,
                                   color=TEAL, alpha=0.30))
            ax.add_patch(Wedge((0, 0), 1.0, 90 - 28.5, 90 + 28.5, width=0.5,
                               color=TEAL, alpha=0.85))
            ax.add_patch(Circle((0, 0), 0.17, color=NAVY))
            ax.add_patch(FancyArrowPatch((0.32, 0.12), (-0.32, 0.12),
                                         connectionstyle='arc3,rad=0.9',
                                         arrowstyle='-|>,head_length=8,head_width=5',
                                         color=NAVY, linewidth=2.5))
        else:
            xs = [-1.2, 1.2, 1.2, -1.2, -1.2, 1.2]
            ys = [-0.25, -0.25, 0.35, 0.35, 0.95, 0.95]
            ax.plot(xs, ys, color=GREY, linewidth=5, solid_capstyle='round')
            for yy in (-0.25, 0.35, 0.95):
                ax.add_patch(FancyBboxPatch((-1.2, yy - 0.1), 2.4, 0.2,
                                            boxstyle='round,pad=0,rounding_size=0.03',
                                            color=TEAL, alpha=0.30))
            ax.add_patch(Circle((1.2, 0.35), 0.13, color=NAVY))
    save(fig, 'phase1_inspection_diagram.png')


# ── System architecture: both robots and the link between them (slot 2936 x 400) ──────
def architecture_diagram():
    W, H = 2936, 400
    fig = fig_for(W, H)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(-24, W + 24)   # Canva zooms placed images ~1%; keep the borders clear of it
    ax.set_ylim(-3, H + 3)
    ax.axis('off')

    def container(x, w, colour, title):
        ax.add_patch(FancyBboxPatch((x, 4), w, H - 8, boxstyle='round,pad=0,rounding_size=24',
                                    facecolor='white', edgecolor=colour, linewidth=4))
        ax.add_patch(FancyBboxPatch((x, H - 72), w, 68,
                                    boxstyle='round,pad=0,rounding_size=24',
                                    facecolor=colour, edgecolor=colour, linewidth=4))
        ax.text(x + w / 2, H - 38, title, ha='center', va='center', fontsize=fs(40),
                fontweight='bold', color='white')

    def block(cx, cy, text, colour, w):
        ax.add_patch(FancyBboxPatch((cx - w / 2, cy - 56), w, 112,
                                    boxstyle='round,pad=0,rounding_size=14',
                                    facecolor='#F3F6F8', edgecolor=colour, linewidth=2.5))
        ax.text(cx, cy, text, ha='center', va='center', fontsize=fs(30), color=INK,
                linespacing=1.1)

    def arrow(x0, y0, x1, y1, colour=NAVY):
        ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1),
                                     arrowstyle='-|>,head_length=10,head_width=6',
                                     color=colour, linewidth=2.5, shrinkA=0, shrinkB=0))

    top, bot = 245, 88
    # Leader: five columns 210 wide.
    container(4, 1240, TEAL, 'Leader · Jetson Orin Nano')
    lw = 210
    lx = [129 + i * 247.5 for i in range(5)]
    for x, t in zip(lx, ['RPLIDAR\nC1', 'RTAB-Map\nSLAM', 'Search\nplanner', 'Nav2',
                         'Kobuki\nbase']):
        block(x, bot, t, TEAL, lw)
    block(lx[0], top, 'Kinect\nRGB-D', TEAL, lw)
    block(lx[1], top, 'YOLOv8\n(GPU)', TEAL, lw)
    block(lx[4], top, 'Fleet\nmanager', TEAL, lw)
    for i in range(4):
        arrow(lx[i] + lw / 2, bot, lx[i + 1] - lw / 2, bot)
    arrow(lx[0] + lw / 2, top, lx[1] - lw / 2, top)
    arrow(lx[1] + lw / 2, top, lx[4] - lw / 2, top)
    ax.text((lx[1] + lx[4]) / 2, top + 10, 'cube detections', ha='center', va='bottom',
            fontsize=fs(30), color=INK2)

    # Collector: four columns 240 wide.
    container(1692, 1240, ORANGE, 'Collector · Raspberry Pi 5')
    cw = 240
    cx = [1832 + i * 320 for i in range(4)]
    for x, t in zip(cx, ['RPLIDAR\nC1', "AMCL in the\nleader's map", 'Nav2', 'Kobuki\nbase']):
        block(x, bot, t, ORANGE, cw)
    block(cx[0], top, 'Kinect\nRGB-D', ORANGE, cw)
    block(cx[2], top, 'Collector\nFSM', ORANGE, cw)
    block(cx[3], top, 'Arms', ORANGE, cw)
    for i in range(3):
        arrow(cx[i] + cw / 2, bot, cx[i + 1] - cw / 2, bot)
    arrow(cx[2], top - 56, cx[2], bot + 56)
    arrow(cx[2] + cw / 2, top, cx[3] - cw / 2, top)

    # The link: discrete goals and status only, never velocities.
    ax.text(1468, 372, 'Wi-Fi · ROS 2', ha='center', va='center', fontsize=fs(34),
            fontweight='bold', color=INK2)
    for y, d, label in [(282, +1, 'map + cube task'), (207, -1, 'status + pose'),
                        (132, -1, 'camera frames'), (57, +1, 'detections')]:
        x0, x1 = (1252, 1684) if d > 0 else (1684, 1252)
        arrow(x0, y, x1, y)
        ax.text(1468, y + 8, label, ha='center', va='bottom', fontsize=fs(30), color=INK)
    save(fig, 'system_architecture.png')


# ── Related work (slot 2936 x 330) ──────────────────────────────────────────────────────
def literature_table():
    W, H = 2936, 330
    cols = [('Work', 640), ('Platform', 470), ('Camera aimed by', 400),
            ('Tracks what the camera saw', 760), ('Chooses how to inspect', 666)]
    rows = [  # extended_abstract_v3.tex, Related Work
        ('Frontier exploration (Yamauchi 1997)', 'Ground robot', '—',
         'No: stops once the LiDAR has mapped', 'No'),
        ('Coverage path planning (Galceran 2013)', 'Any, known map', '—',
         'Tool width only', 'No: fixed sweep'),
        ('Star-Searcher (Luo 2024)', 'Drone', 'Yaw', 'Yes', 'No'),
        ('HEATS (Zhang 2025)', 'Mobile manipulator', 'Arm', 'Yes, region by region', 'No'),
        ('Gao et al. (2024)', 'Ground, 3D LiDAR', '—', 'No: LiDAR object proposals', 'No'),
        ('BotZilla (ours)', 'Ground, fixed camera', 'Whole body', 'Yes: camera-footprint map',
         'Yes: rows or viewpoints'),
    ]
    fig = fig_for(W, H)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(-16, W + 16)
    ax.set_ylim(H + 2, -2)
    ax.axis('off')
    rh = H / (len(rows) + 1)
    ax.add_patch(FancyBboxPatch((0, 0), W, rh, boxstyle='round,pad=0,rounding_size=12',
                                facecolor=NAVY, edgecolor='none'))
    for k in range(len(rows)):
        y = (k + 1) * rh
        ours = k == len(rows) - 1
        if ours or k % 2:
            ax.add_patch(FancyBboxPatch((0, y), W, rh, boxstyle='round,pad=0,rounding_size=8',
                                        facecolor='#D7EBEE' if ours else '#F3F6F8',
                                        edgecolor='none'))
    x = 0
    for c, (title, w) in enumerate(cols):
        ax.text(x + 20, rh / 2, title, va='center', fontsize=fs(30), fontweight='bold',
                color='white')
        for k, row in enumerate(rows):
            ours = k == len(rows) - 1
            ax.text(x + 20, (k + 1.5) * rh, row[c], va='center', fontsize=fs(30),
                    fontweight='bold' if ours else 'normal', color=TEAL if ours else INK)
        x += w
    save(fig, 'literature_table.png')


# ── Phase 2 deadlock: before / after (slot 700 x 330) ───────────────────────────────────
def deadlock_diagram():
    fig = fig_for(700, 330)
    titles = [('Before: wedged', NAVY), ('After: steps aside', NAVY)]
    for k in range(2):
        ax = fig.add_axes([0.01 + k * 0.5, 0.0, 0.48, 0.80])
        ax.set_xlim(0, 2.0)
        ax.set_ylim(0, 1.6)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.add_patch(FancyBboxPatch((0.02, 0.02), 1.96, 1.56,
                                    boxstyle='round,pad=0,rounding_size=0.08',
                                    facecolor='#F6F8FA', edgecolor='none'))
        fig.text(0.01 + k * 0.5 + 0.24, 0.88, titles[k][0], ha='center',
                 fontsize=fs(36), fontweight='bold', color=titles[k][1])
        # the leader's route ahead (dashed) and the leader
        ax.plot([0.15, 1.85], [0.55, 0.55], color=TEAL, linewidth=3, linestyle=(0, (4, 3)))
        ax.add_patch(Circle((0.55, 0.55), 0.22, facecolor=TEAL, edgecolor='white',
                            linewidth=2))
        ax.add_patch(FancyArrowPatch((0.80, 0.55), (1.05, 0.55),
                                     arrowstyle='-|>,head_length=7,head_width=5',
                                     color=TEAL, linewidth=2.5))
        if k == 0:     # collector parked on the route, touching the leader
            cx, cy = 1.06, 0.62
            ax.add_patch(FancyBboxPatch((cx - 0.25, cy - 0.21), 0.50, 0.42,
                                        boxstyle='round,pad=0,rounding_size=0.06',
                                        facecolor=ORANGE, edgecolor='white', linewidth=2,
                                        zorder=3))
            for dy in (-0.21, 0.15):
                ax.add_patch(FancyBboxPatch((cx - 0.53, cy + dy), 0.30, 0.06,
                                            boxstyle='round,pad=0,rounding_size=0.01',
                                            facecolor=ORANGE, edgecolor='none', zorder=3))
            ax.text(1.0, 1.25, 'no safe move\nfor either', ha='center', va='center',
                    fontsize=fs(30), color=INK)
        else:          # collector has cleared out, off the leader's route
            cx, cy = 1.28, 1.15
            ax.add_patch(FancyBboxPatch((cx - 0.25, cy - 0.21), 0.50, 0.42,
                                        boxstyle='round,pad=0,rounding_size=0.06',
                                        facecolor=ORANGE, edgecolor='white', linewidth=2,
                                        zorder=3))
            for dy in (-0.21, 0.15):
                ax.add_patch(FancyBboxPatch((cx + 0.23, cy + dy), 0.30, 0.06,
                                            boxstyle='round,pad=0,rounding_size=0.01',
                                            facecolor=ORANGE, edgecolor='none', zorder=3))
            ax.add_patch(FancyArrowPatch((0.92, 0.72), (1.08, 0.92),
                                         arrowstyle='-|>,head_length=7,head_width=5',
                                         color=ORANGE, linewidth=2.5,
                                         linestyle=(0, (3, 2))))
            ax.text(0.42, 1.15, 'route clear', ha='center', va='center',
                    fontsize=fs(30), color=INK)
    save(fig, 'phase2_deadlock_before_after.png')


# ── "How it works" step icons (placeholders 555 x 215 each) ─────────────────────────────
STEP_BG = '#EEF3F6'


def _step_canvas():
    fig = fig_for(555, 215)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 2.58)
    ax.set_ylim(0, 1)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.add_patch(FancyBboxPatch((0.012, 0.012), 2.556, 0.976,
                                boxstyle='round,pad=0,rounding_size=0.09',
                                facecolor=STEP_BG, edgecolor='none'))
    return fig, ax


def _leader(ax, x, y, r=0.13):
    ax.add_patch(Circle((x, y), r, facecolor=TEAL, edgecolor='white', linewidth=2))


def _collector(ax, x, y, s=1.0, cube=False):
    """Top view, facing +x: a rounded body and two grabber arms."""
    w, h = 0.30 * s, 0.24 * s
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                                boxstyle=f'round,pad=0,rounding_size={0.05 * s}',
                                facecolor=ORANGE, edgecolor='white', linewidth=2))
    for dy in (-h / 2, h / 2 - 0.035 * s):
        ax.add_patch(FancyBboxPatch((x + w / 2 - 0.02 * s, y + dy), 0.17 * s, 0.035 * s,
                                    boxstyle='round,pad=0,rounding_size=0.01',
                                    facecolor=ORANGE, edgecolor='none'))
    if cube:
        c = 0.10 * s
        ax.add_patch(FancyBboxPatch((x + w / 2 + 0.03 * s, y - c / 2), c, c,
                                    boxstyle='round,pad=0,rounding_size=0.01',
                                    facecolor=NAVY, edgecolor='none'))


def step_icons():
    # 1 Map: the real SLAM map of run 27, wall pixels and seen floor only.
    fig, ax = _step_canvas()
    z = np.load(RUN27 / 'map_final.npz')
    m, s = z['map'], z['swept']
    walls = np.argwhere(m >= 65)
    (j0, i0), (j1, i1) = (np.percentile(walls, 0.5, axis=0).astype(int) - 4,
                          np.percentile(walls, 99.5, axis=0).astype(int) + 4)
    crop_m, crop_s = m[j0:j1, i0:i1], s[j0:j1, i0:i1]
    rgba = np.zeros(crop_m.shape + (4,))
    rgba[(crop_s > 0) & (crop_m == 0)] = matplotlib.colors.to_rgba('#CDE4E7')
    wall = crop_m >= 65
    thick = wall.copy()                 # walls 1 px thicker so they read at icon size
    thick[1:, :] |= wall[:-1, :]
    thick[:-1, :] |= wall[1:, :]
    thick[:, 1:] |= wall[:, :-1]
    thick[:, :-1] |= wall[:, 1:]
    rgba[thick] = matplotlib.colors.to_rgba(INK)
    rgba = np.rot90(rgba, k=1)
    hh = 0.86
    ww = hh * rgba.shape[1] / rgba.shape[0]
    ax.imshow(rgba, origin='lower', extent=[1.29 - ww / 2, 1.29 + ww / 2, 0.07, 0.07 + hh],
              interpolation='nearest', zorder=2)
    save(fig, 'step1_map.png')

    # 2 Search: stop and turn through a range.
    fig, ax = _step_canvas()
    cx, cy = 1.29, 0.16
    for a in range(-60, 61, 30):
        ax.add_patch(Wedge((cx, cy), 0.74, 90 + a - 28.5, 90 + a + 28.5, width=0.37,
                           color=TEAL, alpha=0.28))
    ax.add_patch(Wedge((cx, cy), 0.74, 90 - 28.5, 90 + 28.5, width=0.37, color=TEAL,
                       alpha=0.85))
    _leader(ax, cx, cy, 0.12)
    ax.add_patch(FancyArrowPatch((cx + 0.24, cy + 0.06), (cx - 0.24, cy + 0.06),
                                 connectionstyle='arc3,rad=0.9',
                                 arrowstyle='-|>,head_length=7,head_width=4.5',
                                 color=NAVY, linewidth=2.2))
    save(fig, 'step2_search.png')

    # 3 Detect: a camera frame with a cube and YOLO's box.
    fig, ax = _step_canvas()
    ax.add_patch(FancyBboxPatch((0.66, 0.10), 1.26, 0.80,
                                boxstyle='round,pad=0,rounding_size=0.04',
                                facecolor='white', edgecolor=INK2, linewidth=3))
    ax.plot([0.66, 1.92], [0.33, 0.33], color=GRID, linewidth=3)      # floor line
    ax.add_patch(FancyBboxPatch((1.17, 0.27), 0.24, 0.24,
                                boxstyle='round,pad=0,rounding_size=0.015',
                                facecolor=ORANGE, edgecolor='none'))
    ax.add_patch(FancyBboxPatch((1.11, 0.21), 0.36, 0.36,
                                boxstyle='round,pad=0,rounding_size=0.01',
                                facecolor='none', edgecolor=TEAL, linewidth=3.5,
                                linestyle=(0, (4, 2))))
    ax.text(1.11, 0.62, 'cube 0.91', fontsize=fs(36), fontweight='bold', color=TEAL,
            va='bottom')
    save(fig, 'step3_detect.png')

    # 4 Assign: the leader hands the cube's task to the collector.
    fig, ax = _step_canvas()
    _leader(ax, 0.55, 0.5, 0.17)
    _collector(ax, 1.95, 0.5, s=1.1)
    ax.add_patch(FancyArrowPatch((0.80, 0.5), (1.68, 0.5),
                                 arrowstyle='-|>,head_length=10,head_width=6',
                                 color=NAVY, linewidth=3))
    ax.add_patch(FancyBboxPatch((1.10, 0.62), 0.30, 0.22,
                                boxstyle='round,pad=0,rounding_size=0.03',
                                facecolor='white', edgecolor=NAVY, linewidth=2.5))
    ax.add_patch(FancyBboxPatch((1.21, 0.69), 0.08, 0.08,
                                boxstyle='round,pad=0,rounding_size=0.01',
                                facecolor=NAVY, edgecolor='none'))
    ax.text(1.25, 0.32, 'task', ha='center', fontsize=fs(36), color=INK2)
    save(fig, 'step4_assign.png')

    # 5 Collect: the collector carries the cube HOME.
    fig, ax = _step_canvas()
    _collector(ax, 0.62, 0.45, s=1.15, cube=True)
    ax.add_patch(FancyArrowPatch((1.05, 0.45), (1.70, 0.45),
                                 arrowstyle='-|>,head_length=10,head_width=6',
                                 color=NAVY, linewidth=3, linestyle=(0, (5, 3))))
    hx, hy = 2.02, 0.30          # a house: HOME
    ax.add_patch(plt.Polygon([[hx - 0.24, hy + 0.30], [hx, hy + 0.52],
                              [hx + 0.24, hy + 0.30]], closed=True, color=NAVY))
    ax.add_patch(FancyBboxPatch((hx - 0.19, hy), 0.38, 0.31,
                                boxstyle='round,pad=0,rounding_size=0.015',
                                facecolor=NAVY, edgecolor='none'))
    ax.add_patch(FancyBboxPatch((hx - 0.05, hy), 0.10, 0.15,
                                boxstyle='round,pad=0,rounding_size=0.01',
                                facecolor=STEP_BG, edgecolor='none'))
    ax.text(hx, hy - 0.09, 'HOME', ha='center', va='center', fontsize=fs(32),
            fontweight='bold', color=NAVY)
    save(fig, 'step5_collect.png')


if __name__ == '__main__':
    step_icons()
    coverage_chart()
    share = leader_blocked_chart()
    before = sorted(share[r] for r in (12, 13, 14, 15, 16))
    after = sorted(share[r] for r in (20, 21, 23, 27))
    print('leader blocked, median %%: runs 12-16 %.1f -> full runs 20-27 %.1f'
          % (before[2], (after[1] + after[2]) / 2))
    lab_map()
    phase1_diagram()
    architecture_diagram()
    literature_table()
    deadlock_diagram()
