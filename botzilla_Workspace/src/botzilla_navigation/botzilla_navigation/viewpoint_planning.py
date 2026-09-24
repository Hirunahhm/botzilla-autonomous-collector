"""
viewpoint_planning.py.

Pure, ROS-independent viewpoint planning with a turn range — the inspection primitive of
research_discussion.md §7 and §8.3, shared by the proposed method, its ablations, arm E
(camera-range exploration) and arm D (HEATS-style). Unit-tested in
test/test_viewpoint_planning.py.

A viewpoint is a position plus a look: a start heading and a turn span. Span 0 is one
look; a span of 360 deg minus the FOV is a full spin. The camera sees floor in an
annulus sector (CAMERA_MIN_RANGE_M to CAMERA_MARK_RANGE_M, +/- half FOV) and only along
an unobstructed line — exactly swept_mask's model, so what the planner expects to see is
what the swept mask will record.

How a candidate is scored. From the candidate cell, every target cell in the annulus
(unseen floor, sampled every `stride` cells) is tested for line of sight and binned by
bearing into a 36-bin histogram. A look whose heading sweeps from h to h + span covers
the bins in [h - FOV/2, h + span + FOV/2], so for each span the best start heading is a
circular window sum over the histogram. Each (start, span) is then scored as seen area
per second:

    score = gain / (travel + overhead + turn_to_start + span / turn_rate)

The choice between one look and a partial or full spin falls out of the same score —
there is no separate rule (§8.3). Turning is priced honestly because it is the robot's
weak point: at 0.4 rad/s a full spin costs ~16 s.

Line of sight is vectorised. The rays from a cell to its annulus targets are the same
relative offsets for every candidate, so they are computed once per (resolution, range)
in a RayTable; testing a candidate is one gather of occupied flags along those rays.
"""

import itertools
import math

from botzilla_navigation.swept_mask import (
    CAMERA_HALF_FOV_RAD,
    CAMERA_MARK_RANGE_M,
    CAMERA_MIN_RANGE_M,
    LOS_TARGET_CLEARANCE_M,
)
import numpy as np

BIN_COUNT = 36
# Motion model for travel time. Effective, not peak: max_vel_x is 0.2 m/s and
# max_rotational_vel 0.4 rad/s (nav2_params.yaml), but acceleration limits and the
# controller's heading corrections mean the robot averages well under both.
LINEAR_SPEED_MPS = 0.15
TURN_RATE_RPS = 0.35
# Fixed cost of every stop: goal dispatch, the reachability check, deceleration and the
# goal checker settling. Measured goals in run 22 took 3-5 s more than distance/speed.
STOP_OVERHEAD_S = 4.0
# Spans a look may use. 0 = one look; the last one plus the FOV is a full circle.
DEFAULT_SPANS_RAD = tuple(math.radians(d) for d in (0, 60, 120, 180, 240)) + (
    2.0 * math.pi - 2.0 * CAMERA_HALF_FOV_RAD,
)
FULL_SPIN_RAD = 2.0 * math.pi - 2.0 * CAMERA_HALF_FOV_RAD
# Target sampling stride in cells: every 2nd cell each way (0.1 m at 5 cm cells). Gains
# are scaled back up by stride^2, so they stay in m^2.
TARGET_STRIDE_CELLS = 2
# Candidate positions are sampled on this lattice.
CANDIDATE_SPACING_M = 0.25
# A candidate whose best look reveals less than this is not worth a stop.
MIN_GAIN_M2 = 0.1


def wrap_angle(a):
    """Wrap an angle (or array of angles) to (-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


class RayTable:
    """Relative rays from a cell to every target in an annulus — see module docstring."""

    def __init__(
        self, resolution, min_range_m=CAMERA_MIN_RANGE_M, max_range_m=CAMERA_MARK_RANGE_M,
        stride=TARGET_STRIDE_CELLS, bin_count=BIN_COUNT,
        target_clearance_m=LOS_TARGET_CLEARANCE_M,
    ):
        self.resolution = resolution
        self.stride = stride
        self.bin_count = bin_count
        self.cell_area = resolution * resolution * stride * stride
        reach = int(math.ceil(max_range_m / resolution))
        self.pad = reach + 1
        drs, dcs, bins, rays = [], [], [], []
        for dr in range(-reach, reach + 1, stride):
            for dc in range(-reach, reach + 1, stride):
                dist = math.hypot(dr, dc) * resolution
                if dist < min_range_m or dist > max_range_m or (dr == 0 and dc == 0):
                    continue
                bearing = math.atan2(dr, dc) % (2.0 * math.pi)
                drs.append(dr)
                dcs.append(dc)
                bins.append(int(bearing / (2.0 * math.pi) * bin_count) % bin_count)
                rays.append(self._ray(dr, dc, dist, resolution, target_clearance_m))
        self.n = len(drs)
        self.dr = np.array(drs, dtype=np.int64)
        self.dc = np.array(dcs, dtype=np.int64)
        self.bins = np.array(bins, dtype=np.int64)
        longest = max((len(r) for r in rays), default=1) or 1
        # Padded with (0, 0): the candidate's own cell, which is free by construction,
        # so padding never blocks.
        self.ray_r = np.zeros((self.n, longest), dtype=np.int64)
        self.ray_c = np.zeros((self.n, longest), dtype=np.int64)
        for k, ray in enumerate(rays):
            for j, (rr, cc) in enumerate(ray):
                self.ray_r[k, j] = rr
                self.ray_c[k, j] = cc

    @staticmethod
    def _ray(dr, dc, dist, resolution, clearance_m):
        """Cells strictly between the centre cell and the target, half-cell steps."""
        cells = []
        steps = int(math.ceil(dist / (resolution * 0.5)))
        stop = dist - clearance_m
        for k in range(1, steps + 1):
            travelled = min(k * resolution * 0.5, dist)
            if travelled > stop:
                break
            t = travelled / dist
            cell = (int(math.floor(dr * t + 0.5)), int(math.floor(dc * t + 0.5)))
            if cell != (0, 0) and (not cells or cells[-1] != cell):
                cells.append(cell)
        return cells

    def window_cells(self, fov_rad):
        """Target samples one look can see at most, for normalising gains."""
        return self.n * min(1.0, fov_rad / (2.0 * math.pi))


class PaddedGrids:
    """Target and occupied masks padded so ray gathers never index out of bounds."""

    def __init__(self, targets, occupied, pad):
        self.pad = pad
        self.targets = np.pad(targets, pad, constant_values=False)
        self.occupied = np.pad(occupied, pad, constant_values=False)


def visible_histogram(padded, table, row, col):
    """Bearing histogram (target samples per bin) visible from cell (row, col)."""
    r0 = row + padded.pad
    c0 = col + padded.pad
    hit = padded.targets[r0 + table.dr, c0 + table.dc]
    if not hit.any():
        return np.zeros(table.bin_count, dtype=np.int64)
    idx = np.flatnonzero(hit)
    blocked = padded.occupied[r0 + table.ray_r[idx], c0 + table.ray_c[idx]].any(axis=1)
    return np.bincount(table.bins[idx[~blocked]], minlength=table.bin_count)


def visible_count(padded, table, row, col):
    """Visible target samples from (row, col) in every direction (for 360 deg sensors)."""
    return int(visible_histogram(padded, table, row, col).sum())


def window_gains(hist, fov_rad, span_rad):
    """Per start bin s, target samples covered by a look sweeping span_rad from s.

    Returns (gains[bin_count], start_headings[bin_count]) where start_headings[s] is the
    robot heading at the start of that look.

    The window counts only WHOLE bins the look is sure to cover (rounded down), centred
    in the swept arc. Rounding up was tried first and over-promised: a 57 deg FOV became
    six 10 deg bins (60 deg), cells in the extra 1.5 deg each side were always "about to
    be seen" and never were, and the planner chose the same look forever (caught by the
    fake-Nav2 harness). Under-promising costs at most a few cells per look.
    """
    nb = len(hist)
    bin_w = 2.0 * math.pi / nb
    covered = fov_rad + span_rad
    width = min(nb, max(1, int(math.floor(covered / bin_w + 1e-9))))
    slack = max(0.0, covered - width * bin_w)
    ext = np.concatenate([hist, hist])
    cs = np.concatenate([[0], np.cumsum(ext)])
    starts = np.arange(nb)
    gains = cs[starts + width] - cs[starts]
    # Arc swept = [h - fov/2, h + span + fov/2]; place it so the slack is split evenly
    # either side of the counted bins [s, s + width).
    headings = wrap_angle(starts * bin_w - slack / 2.0 + fov_rad / 2.0)
    return gains, headings


class Viewpoint:
    """A planned look: where to stand, which way to face first, and how far to turn."""

    __slots__ = ('x', 'y', 'row', 'col', 'heading', 'span', 'gain_m2', 'time_s', 'score',
                 'gain_explore_m2')

    def __init__(self, x, y, row, col, heading, span, gain_m2, time_s, score,
                 gain_explore_m2=0.0):
        self.x, self.y, self.row, self.col = x, y, row, col
        self.heading, self.span = heading, span
        self.gain_m2, self.time_s, self.score = gain_m2, time_s, score
        self.gain_explore_m2 = gain_explore_m2

    def __repr__(self):
        return (f'Viewpoint(({self.x:.2f}, {self.y:.2f}) heading '
                f'{math.degrees(self.heading):.0f}deg span {math.degrees(self.span):.0f}deg '
                f'gain {self.gain_m2:.2f}m2 time {self.time_s:.1f}s)')


def travel_time(robot, x, y, heading, span, lin_speed=LINEAR_SPEED_MPS,
                turn_rate=TURN_RATE_RPS, overhead_s=STOP_OVERHEAD_S, min_move_m=0.3):
    """Seconds to get from robot=(x, y, yaw) to (x, y), face `heading`, and turn `span`.

    Travel is max(distance / speed, turning / turn rate) — HEATS's travel cost — plus a
    fixed per-stop overhead, the arrival turn to the start heading, and the look itself.
    heading may be an array (vectorised over start headings).
    """
    rx, ry, ryaw = robot
    dist = math.hypot(x - rx, y - ry)
    if dist < min_move_m:
        # A look from where the robot already stands still pays the full overhead: it is
        # a separate decision, Spin goal and settle. Charging less makes a string of
        # one-looks from the same spot look cheaper than the one spin they add up to.
        turn = np.abs(wrap_angle(heading - ryaw))
        return turn / turn_rate + span / turn_rate + overhead_s
    direction = math.atan2(y - ry, x - rx)
    leave = abs(float(wrap_angle(direction - ryaw)))
    move = max(dist / lin_speed, leave / turn_rate)
    arrive = np.abs(wrap_angle(heading - direction))
    return move + overhead_s + arrive / turn_rate + span / turn_rate


def candidate_cells(grid, allowed, cost_ok, spacing_m=CANDIDATE_SPACING_M,
                    avoid=(), lattice_origin=(0.0, 0.0)):
    """Candidate (row, col) cells: free, allowed, cost-OK, on a world-anchored lattice.

    avoid: [(x, y, radius_m), ...] — recently failed viewpoints to stay away from.
    """
    step = max(1, int(round(spacing_m / grid.resolution)))
    ix, iy = grid.world_keys()
    ox = int(round(lattice_origin[0] / grid.resolution))
    oy = int(round(lattice_origin[1] / grid.resolution))
    on_lattice = ((ix - ox) % step == 0) & ((iy - oy) % step == 0)
    ok = on_lattice & grid.free & allowed & cost_ok
    rows, cols = np.nonzero(ok)
    out = []
    for r, c in zip(rows.tolist(), cols.tolist()):
        x, y = grid.world_of(r, c)
        if any(math.hypot(x - ax, y - ay) < ar for ax, ay, ar in avoid):
            continue
        out.append((r, c))
    return out


def costmap_ok_mask(grid, costmap, max_cost):
    """Map-aligned mask of cells whose costmap value (published 0-100) is known and <= max.

    costmap is a Grid built from /global_costmap/costmap; the two grids differ in size
    and origin, so each map cell centre is looked up by world coordinate. Cells outside
    the costmap are not OK.
    """
    rows = np.arange(grid.height)
    cols = np.arange(grid.width)
    xs = grid.origin_x + (cols + 0.5) * grid.resolution
    ys = grid.origin_y + (rows + 0.5) * grid.resolution
    cc = np.floor((xs - costmap.origin_x) / costmap.resolution).astype(np.int64)
    cr = np.floor((ys - costmap.origin_y) / costmap.resolution).astype(np.int64)
    col_ok = (cc >= 0) & (cc < costmap.width)
    row_ok = (cr >= 0) & (cr < costmap.height)
    cc_c = np.clip(cc, 0, costmap.width - 1)
    cr_c = np.clip(cr, 0, costmap.height - 1)
    values = costmap.values[cr_c[:, None], cc_c[None, :]]
    inside = row_ok[:, None] & col_ok[None, :]
    return inside & (values >= 0) & (values <= max_cost)


def plan_looks(grid, padded, table, candidates, robot, spans=DEFAULT_SPANS_RAD,
               fov_rad=2.0 * CAMERA_HALF_FOV_RAD, overhead_s=STOP_OVERHEAD_S,
               min_gain_m2=MIN_GAIN_M2, top_k=1):
    """Best viewpoints by seen area per second — see module docstring.

    Returns up to top_k Viewpoints, best first; [] if none reveals min_gain_m2.
    """
    results = []
    for row, col in candidates:
        hist = visible_histogram(padded, table, row, col)
        if hist.sum() * table.cell_area < min_gain_m2:
            continue
        x, y = grid.world_of(row, col)
        best = None
        for span in spans:
            gains, headings = window_gains(hist, fov_rad, span)
            times = travel_time(robot, x, y, headings, span, overhead_s=overhead_s)
            scores = gains * table.cell_area / times
            s = int(np.argmax(scores))
            if best is None or scores[s] > best[0]:
                best = (float(scores[s]), float(gains[s] * table.cell_area),
                        float(headings[s]), span, float(times[s]))
        score, gain, heading, span, t = best
        if gain < min_gain_m2:
            continue
        results.append(Viewpoint(x, y, row, col, heading, span, gain, t, score))
    results.sort(key=lambda v: v.score, reverse=True)
    return results[:top_k]


def plan_heats(grid, padded_inspect, table_inspect, padded_explore, table_explore,
               candidates, robot, w_explore=0.2, w_inspect=0.8,
               fov_rad=2.0 * CAMERA_HALF_FOV_RAD, top_k=5, min_gain_m2=MIN_GAIN_M2):
    """HEATS-style blended viewpoints: one look per viewpoint, heading chosen.

    utility = (w_e * g_e + w_i * g_i) / cost, with both gains normalised to [0, 1] by
    the most one view can see (g_i: target samples in one camera window; g_e: unknown
    samples in the exploration disc) and cost = max(distance / speed, turn / rate) —
    research_discussion.md §6.5. No per-stop overhead: HEATS does not charge one.
    """
    max_i = max(1.0, table_inspect.window_cells(fov_rad))
    max_e = max(1.0, float(table_explore.n))
    results = []
    for row, col in candidates:
        hist = visible_histogram(padded_inspect, table_inspect, row, col)
        explore = visible_count(padded_explore, table_explore, row, col)
        if hist.sum() == 0 and explore == 0:
            continue
        x, y = grid.world_of(row, col)
        gains, headings = window_gains(hist, fov_rad, 0.0)
        times = travel_time(robot, x, y, headings, 0.0, overhead_s=0.0)
        g_i = gains / max_i
        g_e = explore / max_e
        # Floored at 1 s: HEATS's cost has no per-stop term, so a look from where the
        # robot stands would cost ~0 and score without bound.
        utility = (w_explore * g_e + w_inspect * g_i) / np.maximum(times, 1.0)
        s = int(np.argmax(utility))
        gain_i = float(gains[s] * table_inspect.cell_area)
        gain_e = float(explore * table_explore.cell_area)
        if gain_i < min_gain_m2 and gain_e < min_gain_m2 * 2.0:
            continue
        results.append(Viewpoint(x, y, row, col, float(headings[s]), 0.0, gain_i,
                                 float(times[s]), float(utility[s]), gain_e))
    results.sort(key=lambda v: v.score, reverse=True)
    return results[:top_k]


def order_open_tour(start, points):
    """Visit order (indices into points) minimising path length from start, open-ended.

    Exact for up to 7 points, nearest-neighbour plus 2-opt beyond.
    """
    n = len(points)
    if n <= 1:
        return list(range(n))

    def d(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def length(order):
        total = d(start, points[order[0]])
        for a, b in zip(order, order[1:]):
            total += d(points[a], points[b])
        return total

    if n <= 7:
        return list(min(itertools.permutations(range(n)), key=length))
    remaining = set(range(n))
    order = []
    cur = start
    while remaining:
        nxt = min(remaining, key=lambda i: d(cur, points[i]))
        order.append(nxt)
        remaining.discard(nxt)
        cur = points[nxt]
    improved = True
    while improved:
        improved = False
        for i in range(n - 1):
            for j in range(i + 1, n):
                cand = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                if length(cand) < length(order) - 1e-9:
                    order = cand
                    improved = True
    return order
