#!/usr/bin/env python3
"""
make_poster_figures.py — the charts, lab map and diagrams for the A0 exhibition poster.

    python3 tools/make_poster_figures.py            # -> poster/figures/*.png

Each figure is sized to its placeholder on the Canva poster (3179 x 4494 canvas units)
and rendered at 3x for print. Numbers come from full_cycle_runs.md /
extended_abstract_v3.tex (Phase 1) and multi_robot_runs.md (Phase 2); the lab map from
run 27's logs (run_logs/20261007-221643).

Colours are the poster's: teal = Phase 1, orange = Phase 2, grey = context. Checked for
colour-vision deficiency (OKLab, Machado 2009): every pair >= 13.5 under protan /
deutan / tritan simulation, >= 23 for normal vision.
"""
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Wedge, Circle  # noqa: E402
import numpy as np  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / 'poster' / 'figures'
RUN27 = REPO / 'run_logs' / '20261007-221643'

TEAL, ORANGE, GREY = '#1F6F7A', '#C2621B', '#A7B4BF'
INK, INK2, GRID, NAVY = '#1E2A36', '#5B6B7A', '#E3E8ED', '#042034'
SCALE = 3            # image px per Canva canvas unit
DPI = 300
PT = 0.72 * SCALE * 72 / DPI   # points per Canva canvas unit (so 42 units = poster body text)

plt.rcParams.update({
    'font.family': 'Lato', 'text.color': INK, 'axes.labelcolor': INK2,
    'xtick.color': INK2, 'ytick.color': INK2, 'axes.edgecolor': GRID,
    'svg.fonttype': 'none',
})


def fig_for(w_units, h_units):
    return plt.figure(figsize=(w_units * SCALE / DPI, h_units * SCALE / DPI), dpi=DPI)


def fs(units):
    """Font size in points for a size in Canva canvas units."""
    return units * PT


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / name, dpi=DPI, facecolor='white')
    plt.close(fig)
    print('wrote', OUT / name)


# ── Phase 1: coverage AUC by inspection method (placeholder 1448 x 400) ─────────────────
def coverage_chart():
    rows = [  # (label, AUC %, ours?)  region order, 15-min full runs F3/F11/F10/F9, F7
        ('Viewpoints + turn range (ours)', 36.4, True),
        ('Spin grid', 31.5, False),
        ('One look per stop', 29.6, False),
        ('Lawnmower rows', 23.0, False),
        ('HEATS-style baseline', 21.1, False),
    ]
    fig = fig_for(1448, 400)
    ax = fig.add_axes([0.30, 0.10, 0.66, 0.70])
    y = np.arange(len(rows))[::-1]
    for yi, (label, v, ours) in zip(y, rows):
        ax.barh(yi, v, height=0.62, color=TEAL if ours else GREY, edgecolor='white',
                linewidth=2)
        ax.text(v + 0.6, yi, f'{v:.1f}%', va='center', fontsize=fs(34),
                fontweight='bold' if ours else 'normal', color=INK)
    ax.set_yticks(y, [r[0] for r in rows], fontsize=fs(32))
    for t, (_, _, ours) in zip(ax.get_yticklabels(), rows):
        t.set_color(INK if ours else INK2)
        t.set_fontweight('bold' if ours else 'normal')
    ax.set_xlim(0, 40)
    ax.set_xticks([0, 10, 20, 30, 40], ['0', '10', '20', '30', '40%'], fontsize=fs(26))
    ax.tick_params(length=0)
    ax.grid(axis='x', color=GRID, linewidth=1.5)
    ax.set_axisbelow(True)
    for sp in ax.spines.values():
        sp.set_visible(False)
    fig.text(0.02, 0.90, 'Floor coverage by inspection method', fontsize=fs(38),
             fontweight='bold', color=NAVY)
    fig.text(0.02, 0.83, 'area under the floor-seen curve, 15-min runs, higher is better',
             fontsize=fs(26), color=INK2)
    save(fig, 'phase1_coverage_by_method.png')


# ── Phase 2: leader stuck per run (placeholder 1448 x 270) ──────────────────────────────
def leader_stuck_chart():
    hits = {1: 33, 2: 8, 3: 42, 4: 42, 5: 1081, 6: 626, 7: 706, 8: 564, 9: 1018, 10: 399,
            11: 8, 12: 1144, 13: 1607, 14: 839, 15: 720, 16: 980, 17: 441, 18: 2, 19: 9,
            20: 7, 21: 357, 23: 139, 24: 26, 25: 5, 26: 2, 27: 130}
    runs = list(hits)
    fig = fig_for(1448, 270)
    ax = fig.add_axes([0.075, 0.19, 0.91, 0.60])
    x = np.arange(len(runs))
    first_coord = runs.index(17)
    ax.axvspan(first_coord - 0.5, len(runs) - 0.5, color='#FBEBDD', zorder=0)
    ax.bar(x, [hits[r] for r in runs], width=0.72,
           color=[ORANGE if r >= 17 else GREY for r in runs], edgecolor='white',
           linewidth=1.5, zorder=2)
    ax.set_xticks(x, [str(r) for r in runs], fontsize=fs(22))
    ax.set_xlim(-0.6, len(runs) - 0.4)
    ax.set_ylim(0, 1750)
    ax.set_yticks([0, 500, 1000, 1500], ['0', '500', '1000', '1500'], fontsize=fs(22))
    ax.tick_params(length=0)
    ax.grid(axis='y', color=GRID, linewidth=1.2, zorder=1)
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.text(first_coord - 0.3, 1660, 'coordination rules added', fontsize=fs(26),
            fontweight='bold', color=ORANGE, va='top')
    fig.text(0.075, 0.90, 'Leader stuck per test run', fontsize=fs(34),
             fontweight='bold', color=NAVY)
    fig.text(0.36, 0.90, '(Nav2 footprint-collision events; runs 15–16, 18–19, 24–26 '
             'stopped early)', fontsize=fs(22), color=INK2)
    save(fig, 'phase2_leader_stuck_per_run.png')


# ── Lab map from run 27 (placeholder 760 x 350) ─────────────────────────────────────────
def lab_map():
    z = np.load(RUN27 / 'map_final.npz')
    m, s = z['map'], z['swept']
    ox, oy, res = float(z['origin_x']), float(z['origin_y']), float(z['resolution'])
    img = np.ones(m.shape + (3,))
    unknown = m < 0
    img[unknown] = matplotlib.colors.to_rgb('#EEF1F4')
    img[(m == 0)] = (1, 1, 1)
    img[(s > 0) & (m == 0)] = matplotlib.colors.to_rgb('#CDE4E7')   # camera has seen it
    img[m >= 65] = matplotlib.colors.to_rgb(INK)

    start = None
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

    walls = np.argwhere(m >= 65)
    (j0, i0), (j1, i1) = np.percentile(walls, 0.5, axis=0), np.percentile(walls, 99.5, axis=0)
    pad = 8
    j0, i0 = max(int(j0) - pad, 0), max(int(i0) - pad, 0)
    j1, i1 = min(int(j1) + pad, m.shape[0] - 1), min(int(i1) + pad, m.shape[1] - 1)
    crop = img[j0:j1 + 1, i0:i1 + 1]
    x0, x1 = ox + i0 * res, ox + (i1 + 1) * res
    y0, y1 = oy + j0 * res, oy + (j1 + 1) * res

    # Rotate 90 deg clockwise: a map point (x, y) is drawn at (y, -x). Rows of `crop` run
    # along y (origin lower); np.rot90(k=1) on the lower-origin image gives exactly that.
    def rot(x, y):
        return y, -x
    rimg = np.rot90(crop, k=1)
    extent = [y0, y1, -x1, -x0]

    fig = fig_for(760, 350)
    ax = fig.add_axes([0.0, 0.0, 0.62, 1.0])
    ax.imshow(rimg, origin='lower', extent=extent, interpolation='nearest')
    px, py = rot(np.array(xs), np.array(ys))
    ax.plot(px, py, color=NAVY, linewidth=1.6, alpha=0.75, label='leader path')
    for cid, (cx, cy) in cubes.items():
        got = cid in delivered
        ax.plot(*rot(cx, cy), 's', markersize=13,
                markerfacecolor=ORANGE if got else 'white',
                markeredgecolor=ORANGE, markeredgewidth=3)
    ax.plot(*rot(0, 0), '*', markersize=26, color=TEAL, markeredgecolor='white',
            markeredgewidth=1.5)
    if chome:
        ax.plot(*rot(*chome), 'D', markersize=15, color=ORANGE, markeredgecolor='white',
                markeredgewidth=1.5)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect('equal')
    ax.axis('off')
    # Legend on the right, text in ink with the marks beside it.
    lx = fig.add_axes([0.63, 0.05, 0.36, 0.9])
    lx.axis('off')
    lx.set_xlim(0, 1)
    lx.set_ylim(0, 1)
    items = [
        ('patch', '#CDE4E7', 'floor the camera has seen'),
        ('line', NAVY, 'leader path'),
        ('*', TEAL, 'leader start'),
        ('D', ORANGE, 'collector HOME'),
        ('sfill', ORANGE, 'cube delivered'),
        ('shollow', ORANGE, 'cube found, not delivered'),
    ]
    for k, (kind, col, text) in enumerate(items):
        yy = 0.92 - k * 0.165
        if kind == 'patch':
            lx.add_patch(FancyBboxPatch((0.02, yy - 0.04), 0.09, 0.08,
                                        boxstyle='round,pad=0,rounding_size=0.01',
                                        color=col))
        elif kind == 'line':
            lx.plot([0.02, 0.11], [yy, yy], color=col, linewidth=2.5)
        elif kind.startswith('s'):
            lx.plot(0.065, yy, 's', markersize=12,
                    markerfacecolor=col if kind == 'sfill' else 'white',
                    markeredgecolor=col, markeredgewidth=3)
        else:
            lx.plot(0.065, yy, kind, markersize=20 if kind == '*' else 13, color=col)
        lx.text(0.16, yy, text, va='center', fontsize=fs(26), color=INK)
    save(fig, 'lab_map_run27.png')


# ── Phase 1 diagram: one look vs viewpoint with turn range vs rows (1448 x 300) ────────
def phase1_diagram():
    fig = fig_for(1448, 300)
    titles = ['One look covers about 0.5 m²', 'Viewpoint: stop and turn through a range',
              'Lawnmower rows: drive past everything']
    for k in range(3):
        ax = fig.add_axes([0.01 + k * 0.335, 0.02, 0.31, 0.78])
        ax.set_xlim(-1.4, 1.4)
        ax.set_ylim(-0.45, 1.25)
        ax.set_aspect('equal')
        ax.axis('off')
        fig.text(0.01 + k * 0.335 + 0.155, 0.88, titles[k], ha='center',
                 fontsize=fs(30), fontweight='bold', color=NAVY)
        if k == 0:
            ax.add_patch(Wedge((0, 0), 1.0, 90 - 28.5, 90 + 28.5, width=0.5,
                               color=TEAL, alpha=0.85))
            ax.add_patch(Circle((0, 0), 0.17, color=NAVY))
            ax.text(0.62, 0.55, '0.5–1.0 m\n57°', fontsize=fs(26), color=INK2,
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


# ── Phase 2 system diagram (1448 x 300) ─────────────────────────────────────────────────
def phase2_diagram():
    fig = fig_for(1448, 300)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1448)
    ax.set_ylim(0, 300)
    ax.axis('off')

    def box(x, w, colour, title, lines):
        ax.add_patch(FancyBboxPatch((x, 22), w, 256, boxstyle='round,pad=0,rounding_size=22',
                                    facecolor='white', edgecolor=colour, linewidth=4))
        ax.add_patch(FancyBboxPatch((x, 210), w, 68, boxstyle='round,pad=0,rounding_size=22',
                                    facecolor=colour, edgecolor=colour, linewidth=4))
        ax.text(x + w / 2, 244, title, ha='center', va='center', fontsize=fs(34),
                fontweight='bold', color='white')
        for k, ln in enumerate(lines):
            ax.text(x + 24, 178 - k * 38, ln, va='center', fontsize=fs(27), color=INK)

    box(16, 470, TEAL, 'Leader · Jetson Orin Nano',
        ['maps the room (RTAB-Map SLAM)', 'searches (Phase 1 method)',
         'YOLO for both robots (GPU)', 'assigns cubes, sets right of way'])
    box(962, 470, ORANGE, 'Collector · Raspberry Pi 5',
        ['localises in the leader\'s map', 'drives to the cube, grabs it',
         'delivers it HOME', 'steps aside for the leader'])

    arrows = [  # (y, direction, label)
        (205, +1, 'cube task'),
        (150, -1, 'status + position'),
        (95, -1, 'camera frames (JPEG)'),
        (40, +1, 'detections back'),
    ]
    for y, d, label in arrows:
        x0, x1 = (500, 948) if d > 0 else (948, 500)
        ax.add_patch(FancyArrowPatch((x0, y), (x1, y),
                                     arrowstyle='-|>,head_length=14,head_width=8',
                                     color=NAVY, linewidth=3))
        ax.text(724, y + 13, label, ha='center', va='bottom', fontsize=fs(25), color=INK)
    ax.text(724, 268, 'Wi-Fi · ROS 2', ha='center', va='center', fontsize=fs(28),
            fontweight='bold', color=INK2)
    save(fig, 'phase2_system_diagram.png')


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
    ax.text(1.11, 0.62, 'cube 0.91', fontsize=fs(30), fontweight='bold', color=TEAL,
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
    ax.text(1.25, 0.32, 'task', ha='center', fontsize=fs(28), color=INK2)
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
    ax.text(hx, hy - 0.09, 'HOME', ha='center', va='center', fontsize=fs(26),
            fontweight='bold', color=NAVY)
    save(fig, 'step5_collect.png')


if __name__ == '__main__':
    step_icons()
    coverage_chart()
    leader_stuck_chart()
    lab_map()
    phase1_diagram()
    phase2_diagram()
