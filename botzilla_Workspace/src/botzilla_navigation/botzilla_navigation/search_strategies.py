"""
search_strategies.py.

Pure, ROS-independent decision logic for the search strategies compared in the research
plan (research_discussion.md §8.3-8.5). Each strategy is a class with one method,
next_action(snapshot) -> Action, that looks at the map, the swept (camera-seen) mask
and the frontiers and says what the robot should do next. frontier_explorer_node
executes the action with its existing, hardware-hardened machinery (goal snapping,
reachability pre-check, stall watchdog, recovery) and tells the strategy how it went via
report(). Keeping the decisions here means they can be unit-tested without ROS
(test/test_search_strategies.py) and every arm shares the same execution code — only the
decisions differ between arms, which is what the comparison needs.

Actions:
  frontier  drive to a frontier (x, y)                  — mapping
  rows      drive boustrophedon waypoints over a patch  — inspection by rows
  look      drive to a viewpoint, face `heading`, turn `span` — inspection by looking
  wait      nothing actionable right now (only blacklisted frontiers left)
  done      search finished

Strategies:

  RegionSearch — the proposed method (§8.3) and its ablations. One region at a time:
    explore it until no INSIDE frontier remains (exits stay locked), freeze its cells,
    inspect them until ~90% is camera-seen, then move to the nearest unfinished region
    (rooms before corridors) or, when none is known, through the nearest exit.
    inspection_mode picks the inspection primitive:
      mixed      one row pass over large open patches, then viewpoints with a turn range
      viewpoints viewpoints with a turn range only             (ablation 1)
      rows       one row pass over every patch only            (ablation 2)
      one_look   row pass, then viewpoints without turning     (ablation 3)
      spin_grid  full spins on a ~1 m lattice                  (optional arm)

  HeatsSearch — arm D, HEATS-style (§6.5), full two-stage version. Global stage:
    unfinished regions visited in the order of an open TSP tour over their centroids.
    Local stage: inside the region, viewpoints (position + heading, one look) scored by
    the blended utility (0.2 exploration + 0.8 inspection) / travel cost; the top five
    are ordered by an open TSP tour and the first is executed. A region is done when no
    viewpoint in it has any gain left. Called "HEATS-style", never "HEATS".

  CameraGreedySearch — arm E, Star-Searcher's FUEL-3m control: plain greedy
    exploration where "explored" means camera-seen. Viewpoints anywhere on the map,
    scored by camera-unseen cells (unknown or un-inspected floor) per second; LiDAR
    frontiers only when no viewpoint has gain. No regions.
"""

import math

from botzilla_navigation.coverage_planning import generate_coverage_waypoints
from botzilla_navigation.region_segmentation import (
    classify_frontier,
    keys_from_mask,
    mask_from_keys,
    segment_regions,
)
from botzilla_navigation.swept_mask import CAMERA_HALF_FOV_RAD, CAMERA_MIN_RANGE_M
from botzilla_navigation.viewpoint_planning import (
    candidate_cells,
    costmap_ok_mask,
    DEFAULT_SPANS_RAD,
    FULL_SPIN_RAD,
    MIN_GAIN_M2,
    order_open_tour,
    PaddedGrids,
    plan_heats,
    plan_looks,
    RayTable,
)
import numpy as np
from scipy import ndimage

INSPECTION_MODES = ('mixed', 'viewpoints', 'rows', 'one_look', 'spin_grid')
# A region is inspected once this share of its floor is camera-seen. Below 100% so an
# unreachable strip under furniture cannot hold the robot in a room forever (§8.3).
REGION_DONE_FRACTION = 0.9
# Unseen patches at least this large (and a row pass is allowed) get rows; smaller ones
# get viewpoints. ~2 m^2 is about four looks' worth of floor (§8.3).
ROW_PATCH_MIN_M2 = 2.0
# Viewpoints must sit on cells the robot can turn in place on: published cost 50 is
# about the circumscribed radius (see frontier_explorer_node COSTMAP_SAFE_COST).
LOOK_MAX_COST = 50
SPIN_GRID_SPACING_M = 1.0
# A failed viewpoint keeps new viewpoints this far away for this long.
FAILED_LOOK_RADIUS_M = 0.4
FAILED_LOOK_MEMORY_S = 120.0
# A look just completed is not repeated (same spot, heading and span) for this long.
# Backstop for any mismatch between what the planner predicts and what the swept mask
# records: without it, a look promising a few cells the mask never marks is chosen
# again and again (seen in the fake-Nav2 harness before the window fix).
REPEAT_LOOK_MEMORY_S = 60.0
# HEATS-style exploration gain: unknown cells visible within this radius (the LiDAR's
# useful mapping range indoors is far more, but the gain has to stay local to the
# viewpoint for the blend to mean anything).
HEATS_EXPLORE_RADIUS_M = 2.0
# HEATS-style exploration gain counts only unknown cells within this distance of a live
# frontier, and a frontier stops counting after this many looks aimed near it. Counting
# every unknown cell was tried first: unknown patches the LiDAR never resolves (behind
# furniture, map speckle) kept a steady 0.2 m^2 of "gain" alive, and the arm kept
# returning to look at them with zero inspection gain (caught by the fake-Nav2 harness).
HEATS_FRONTIER_BAND_M = 0.3
HEATS_MAX_LOOKS_PER_FRONTIER = 2
HEATS_FRONTIER_NEAR_M = 2.0
# See frontier_explorer_node SWEEP_MIN_UNSWEPT_M.
ROW_MIN_UNSWEPT_M = CAMERA_MIN_RANGE_M + 0.1


class Snapshot:
    """What a strategy sees each decision. Built by frontier_explorer_node."""

    def __init__(self, grid, costmap, seen, robot, frontiers, now, frontiers_given_up=False):
        self.grid = grid                  # region_segmentation.Grid of /map
        self.costmap = costmap            # region_segmentation.Grid of the global costmap
        self.seen = seen                  # (h, w) bool, the PLANNING swept mask
        self.robot = robot                # (x, y, yaw) in map
        # [{'x', 'y', 'cells': [(row, col), ...], 'blacklisted': bool}, ...]
        self.frontiers = frontiers
        self.now = now                    # seconds, monotonic
        # Set by the node after repeated stuck recoveries: blacklisted frontiers will not
        # come back in time to matter, so stop waiting on them.
        self.frontiers_given_up = frontiers_given_up


class Action:
    """What the robot should do next — see module docstring."""

    def __init__(self, kind, reason='', target=None, waypoints=None, viewpoint=None):
        self.kind = kind
        self.reason = reason
        self.target = target
        self.waypoints = waypoints
        self.viewpoint = viewpoint

    def __repr__(self):
        return f'Action({self.kind}: {self.reason})'


class _Base:
    """Shared plumbing: ray tables, failed-look memory, exit and finish handling."""

    def __init__(self, half_fov_rad=CAMERA_HALF_FOV_RAD, min_range_m=None, max_range_m=None):
        self.fov_rad = 2.0 * half_fov_rad
        self._min_range_m = min_range_m
        self._max_range_m = max_range_m
        self._tables = {}
        self._failed = []  # (x, y, expiry)
        self._done_looks = []  # (x, y, heading, span, expiry)
        self.events = []   # human-readable decisions, drained by the node for logging

    def _table(self, resolution, explore=False):
        key = (round(resolution, 4), explore)
        if key not in self._tables:
            if explore:
                self._tables[key] = RayTable(resolution, 0.0, HEATS_EXPLORE_RADIUS_M,
                                             stride=3)
            else:
                kwargs = {}
                if self._min_range_m is not None:
                    kwargs['min_range_m'] = self._min_range_m
                if self._max_range_m is not None:
                    kwargs['max_range_m'] = self._max_range_m
                self._tables[key] = RayTable(resolution, **kwargs)
        return self._tables[key]

    def _avoid(self, now):
        self._failed = [f for f in self._failed if f[2] > now]
        return [(x, y, FAILED_LOOK_RADIUS_M) for x, y, _ in self._failed]

    def report(self, action, succeeded, now):
        """Outcome of the last action: failed spots are avoided, done looks not repeated."""
        if action is None or action.kind != 'look' or action.viewpoint is None:
            return
        vp = action.viewpoint
        if succeeded:
            self._done_looks.append((vp.x, vp.y, vp.heading, vp.span,
                                     now + REPEAT_LOOK_MEMORY_S))
        else:
            self._failed.append((vp.x, vp.y, now + FAILED_LOOK_MEMORY_S))

    def _pick(self, viewpoints, now):
        """Best viewpoint that is not a repeat of a look just done — see REPEAT_LOOK_*."""
        self._done_looks = [d for d in self._done_looks if d[4] > now]
        for vp in viewpoints:
            repeat = any(
                math.hypot(vp.x - x, vp.y - y) < 0.1
                and abs(math.atan2(math.sin(vp.heading - h), math.cos(vp.heading - h))) < 0.35
                and abs(vp.span - sp) < 0.1
                for x, y, h, sp, _ in self._done_looks
            )
            if not repeat:
                return vp
        return None

    @staticmethod
    def _nearest_frontier(snap, frontiers=None):
        pool = snap.frontiers if frontiers is None else frontiers
        live = [f for f in pool if not f['blacklisted']]
        if not live:
            return None
        rx, ry = snap.robot[0], snap.robot[1]
        return min(live, key=lambda f: math.hypot(f['x'] - rx, f['y'] - ry))

    def _exit_or_finish(self, snap, why):
        """No region left to work on: go through the nearest exit, or finish."""
        f = self._nearest_frontier(snap)
        if f is not None:
            return Action('frontier', f'{why}; heading through the nearest exit',
                          target=(f['x'], f['y']))
        if any(f['blacklisted'] for f in snap.frontiers) and not snap.frontiers_given_up:
            return Action('wait', f'{why}; only blacklisted frontiers remain')
        return Action('done', f'{why}; no frontiers left')

    def _look_setup(self, snap, targets):
        table = self._table(snap.grid.resolution)
        padded = PaddedGrids(targets, snap.grid.occupied, table.pad)
        cost_ok = (costmap_ok_mask(snap.grid, snap.costmap, LOOK_MAX_COST)
                   if snap.costmap is not None else np.ones_like(targets))
        return table, padded, cost_ok


def _region_seed_label(seg, grid, seed, radius_m=0.5):
    """Label of the region holding world point seed (or the nearest one within radius)."""
    row, col = grid.cell_of(*seed)
    if grid.in_bounds(row, col) and seg.labels[row, col]:
        return int(seg.labels[row, col])
    r = max(1, int(round(radius_m / grid.resolution)))
    r0, r1 = max(0, row - r), min(grid.height, row + r + 1)
    c0, c1 = max(0, col - r), min(grid.width, col + r + 1)
    if r0 >= r1 or c0 >= c1:
        return None
    window = seg.labels[r0:r1, c0:c1]
    rows, cols = np.nonzero(window)
    if rows.size == 0:
        return None
    d = (rows + r0 - row) ** 2 + (cols + c0 - col) ** 2
    k = int(np.argmin(d))
    return int(window[rows[k], cols[k]])


def _nearest_cell(grid, mask, x, y):
    """World (x, y) of the True cell of mask nearest (x, y), and its distance."""
    rows, cols = np.nonzero(mask)
    if rows.size == 0:
        return None, math.inf
    xs = grid.origin_x + (cols + 0.5) * grid.resolution
    ys = grid.origin_y + (rows + 0.5) * grid.resolution
    d = np.hypot(xs - x, ys - y)
    k = int(np.argmin(d))
    return (float(xs[k]), float(ys[k])), float(d[k])


class RegionSearch(_Base):
    """The proposed method and its ablations — see module docstring."""

    def __init__(self, inspection_mode='mixed', done_fraction=REGION_DONE_FRACTION,
                 row_spacing_m=0.86, row_patch_min_m2=ROW_PATCH_MIN_M2, **kwargs):
        super().__init__(**kwargs)
        if inspection_mode not in INSPECTION_MODES:
            raise ValueError(f'unknown inspection_mode {inspection_mode!r}')
        self.inspection_mode = inspection_mode
        self.done_fraction = done_fraction
        self.row_spacing_m = row_spacing_m
        self.row_patch_min_m2 = row_patch_min_m2
        self.phase = 'select'         # select | explore | inspect
        self.seed = None              # world point identifying the active region
        self.active_keys = None       # frozen cells of the active region (inspect)
        self.done_keys = set()
        self.rows_done = False
        self.spin_queue = None        # remaining spin-grid points (spin_grid mode)
        self.regions_done = 0

    # -- region bookkeeping ---------------------------------------------------------

    def _select(self, snap, seg):
        rx, ry = snap.robot[0], snap.robot[1]
        best = None
        for region_id in seg.ids:
            point, dist = _nearest_cell(snap.grid, seg.cells(region_id), rx, ry)
            if point is None:
                continue
            key = (seg.is_corridor(region_id), dist)  # rooms before corridors
            if best is None or key < best[0]:
                best = (key, region_id, point)
        if best is None:
            return None
        _, region_id, point = best
        self.seed = point
        self.phase = 'explore'
        self.active_keys = None
        self.rows_done = False
        self.spin_queue = None
        area = seg.cells(region_id).sum() * snap.grid.resolution ** 2
        kind = 'corridor' if seg.is_corridor(region_id) else 'room'
        self.events.append(f'region selected: {kind} of {area:.1f} m^2 near '
                           f'({point[0]:.2f}, {point[1]:.2f})')
        return region_id

    def _finish_region(self, why):
        if self.active_keys:
            self.done_keys |= self.active_keys
        self.regions_done += 1
        self.events.append(f'region done ({why}); {self.regions_done} done so far')
        self.phase = 'select'
        self.seed = None
        self.active_keys = None

    # -- main entry -----------------------------------------------------------------

    def next_action(self, snap):
        for _ in range(4):  # a finished region hands straight on to the next one
            action = self._step(snap)
            if action is not None:
                return action
        return Action('wait', 'no decision after several region hand-offs')

    def _step(self, snap):
        grid = snap.grid
        done_mask = mask_from_keys(grid, self.done_keys)
        seg = segment_regions(grid, exclude=done_mask)

        region_id = None
        if self.phase in ('explore', 'inspect') and self.seed is not None:
            region_id = _region_seed_label(seg, grid, self.seed)
            if region_id is None and self.phase == 'explore':
                self.events.append('active region vanished from the segmentation; '
                                   're-selecting')
                self.phase = 'select'
        if self.phase == 'select':
            region_id = self._select(snap, seg)
            if region_id is None:
                return self._exit_or_finish(snap, 'no unfinished region known')

        if self.phase == 'explore':
            inside = [f for f in snap.frontiers
                      if classify_frontier(seg, f['cells'], region_id) == 'inside']
            f = self._nearest_frontier(snap, inside)
            if f is not None:
                return Action('frontier', f'exploring the active region '
                              f'({len(inside)} inside frontier(s))', target=(f['x'], f['y']))
            self.active_keys = keys_from_mask(grid, seg.cells(region_id))
            self.phase = 'inspect'
            self.events.append(f'region explored; inspecting '
                               f'{len(self.active_keys) * grid.resolution ** 2:.1f} m^2')

        return self._inspect(snap)

    # -- inspection -----------------------------------------------------------------

    def _inspect(self, snap):
        grid = snap.grid
        region = mask_from_keys(grid, self.active_keys) & grid.free
        total = int(region.sum())
        if total == 0:
            self._finish_region('region has no known floor left')
            return None
        seen_frac = float((region & snap.seen).sum()) / total
        if seen_frac >= self.done_fraction:
            self._finish_region(f'{seen_frac:.0%} of its floor seen')
            return None
        targets = region & ~snap.seen
        mode = self.inspection_mode

        if mode in ('mixed', 'rows', 'one_look') and not self.rows_done:
            self.rows_done = True
            min_patch = 0.0 if mode == 'rows' else self.row_patch_min_m2
            waypoints = self._row_waypoints(snap, targets, min_patch)
            if waypoints:
                return Action('rows', f'row pass over un-seen patches >= {min_patch} m^2 '
                              f'({len(waypoints) // 2} run(s)); {seen_frac:.0%} seen',
                              waypoints=waypoints)
        if mode == 'rows':
            self._finish_region(f'row pass complete, {seen_frac:.0%} seen')
            return None
        if mode == 'spin_grid':
            return self._spin_grid(snap, region, targets, seen_frac)

        spans = (0.0,) if mode == 'one_look' else DEFAULT_SPANS_RAD
        table, padded, cost_ok = self._look_setup(snap, targets)
        candidates = candidate_cells(grid, region, cost_ok, avoid=self._avoid(snap.now))
        best = plan_looks(grid, padded, table, candidates, snap.robot, spans=spans,
                          fov_rad=self.fov_rad, top_k=5)
        vp = self._pick(best, snap.now)
        if vp is None:
            self._finish_region(f'no viewpoint reveals >= {MIN_GAIN_M2} m^2, '
                                f'{seen_frac:.0%} seen')
            return None
        return Action('look', f'{vp!r}; region {seen_frac:.0%} seen', viewpoint=vp)

    def _row_waypoints(self, snap, targets, min_patch_m2):
        grid = snap.grid
        cell_area = grid.resolution ** 2
        labels, n = ndimage.label(targets, structure=np.ones((3, 3), dtype=bool))
        if n == 0:
            return []
        sizes = ndimage.sum_labels(targets, labels, index=np.arange(1, n + 1)) * cell_area
        keep = np.flatnonzero(sizes >= max(min_patch_m2, 1e-9)) + 1
        if keep.size == 0:
            return []
        patch = np.isin(labels, keep)
        # Rows are generated over the patch only: everything else reads as a wall to the
        # row generator, so runs start and end at the patch edges.
        masked = np.where(patch, grid.values, 100).astype(int)
        return generate_coverage_waypoints(
            masked.ravel().tolist(), grid.width, grid.height, grid.resolution,
            grid.origin_x, grid.origin_y, row_spacing_m=self.row_spacing_m,
            swept_mask=snap.seen.ravel().tolist(), min_unswept_m=ROW_MIN_UNSWEPT_M,
        )

    def _spin_grid(self, snap, region, targets, seen_frac):
        grid = snap.grid
        table, padded, cost_ok = self._look_setup(snap, targets)
        if self.spin_queue is None:
            lattice = candidate_cells(grid, region, cost_ok, spacing_m=SPIN_GRID_SPACING_M)
            self.spin_queue = [grid.world_of(r, c) for r, c in lattice]
        rx, ry = snap.robot[0], snap.robot[1]
        while self.spin_queue:
            self.spin_queue.sort(key=lambda p: math.hypot(p[0] - rx, p[1] - ry))
            x, y = self.spin_queue.pop(0)
            row, col = grid.cell_of(x, y)
            if not grid.in_bounds(row, col):
                continue
            best = plan_looks(grid, padded, table, [(row, col)], snap.robot,
                              spans=(FULL_SPIN_RAD,), fov_rad=self.fov_rad)
            if best:
                return Action('look', f'spin grid {best[0]!r}; {len(self.spin_queue)} '
                              f'point(s) left; region {seen_frac:.0%} seen',
                              viewpoint=best[0])
        self._finish_region(f'spin grid exhausted, {seen_frac:.0%} seen')
        return None


class HeatsSearch(_Base):
    """Arm D, HEATS-style two-stage search — see module docstring."""

    def __init__(self, w_explore=0.2, w_inspect=0.8, **kwargs):
        super().__init__(**kwargs)
        self.w_explore = w_explore
        self.w_inspect = w_inspect
        self.seed = None
        self.done_keys = set()
        self.regions_done = 0
        self._frontier_looks = []   # [x, y, count] — see HEATS_MAX_LOOKS_PER_FRONTIER

    def _frontier_record(self, x, y):
        for rec in self._frontier_looks:
            if math.hypot(rec[0] - x, rec[1] - y) < 0.5:
                return rec
        return None

    def _explore_targets(self, snap):
        """Unknown cells near live frontiers not yet looked at too often."""
        grid = snap.grid
        seeds = np.zeros_like(grid.unknown)
        for f in snap.frontiers:
            if f['blacklisted'] or not f['cells']:
                continue
            rec = self._frontier_record(f['x'], f['y'])
            if rec is not None and rec[2] >= HEATS_MAX_LOOKS_PER_FRONTIER:
                continue
            rows, cols = zip(*f['cells'])
            seeds[list(rows), list(cols)] = True
        band = max(1, int(round(HEATS_FRONTIER_BAND_M / grid.resolution)))
        near = ndimage.binary_dilation(seeds, iterations=band)
        return near & grid.unknown

    def report(self, action, succeeded, now):
        super().report(action, succeeded, now)
        if action is None or action.kind != 'look' or action.viewpoint is None:
            return
        vp = action.viewpoint
        if vp.gain_explore_m2 <= 0.0:
            return
        for f in getattr(self, '_last_frontiers', []):
            if math.hypot(f['x'] - vp.x, f['y'] - vp.y) > HEATS_FRONTIER_NEAR_M:
                continue
            rec = self._frontier_record(f['x'], f['y'])
            if rec is None:
                self._frontier_looks.append([f['x'], f['y'], 1])
            else:
                rec[2] += 1

    def next_action(self, snap):
        self._last_frontiers = [f for f in snap.frontiers if not f['blacklisted']]
        for _ in range(4):
            action = self._step(snap)
            if action is not None:
                return action
        return Action('wait', 'no decision after several region hand-offs')

    def _select(self, snap, seg):
        """Global stage: open TSP tour over unfinished region centroids; take the first."""
        grid = snap.grid
        centroids, ids = [], []
        for region_id in seg.ids:
            rows, cols = np.nonzero(seg.cells(region_id))
            if rows.size == 0:
                continue
            ids.append(region_id)
            centroids.append(grid.world_of(float(rows.mean()), float(cols.mean())))
        if not ids:
            return None
        order = order_open_tour(snap.robot[:2], centroids)
        region_id = ids[order[0]]
        self.seed, _ = _nearest_cell(grid, seg.cells(region_id), *centroids[order[0]])
        self.events.append(f'HEATS tour over {len(ids)} region(s); next region near '
                           f'({self.seed[0]:.2f}, {self.seed[1]:.2f})')
        return region_id

    def _step(self, snap):
        grid = snap.grid
        seg = segment_regions(grid, exclude=mask_from_keys(grid, self.done_keys))
        region_id = None
        if self.seed is not None:
            region_id = _region_seed_label(seg, grid, self.seed)
        if region_id is None:
            region_id = self._select(snap, seg)
            if region_id is None:
                return self._exit_or_finish(snap, 'no unfinished region known')

        region = seg.cells(region_id)
        targets = region & ~snap.seen
        table_i, padded_i, cost_ok = self._look_setup(snap, targets)
        table_e = self._table(grid.resolution, explore=True)
        padded_e = PaddedGrids(self._explore_targets(snap), grid.occupied, table_e.pad)
        candidates = candidate_cells(grid, region, cost_ok, avoid=self._avoid(snap.now))
        top = plan_heats(grid, padded_i, table_i, padded_e, table_e, candidates,
                         snap.robot, self.w_explore, self.w_inspect, fov_rad=self.fov_rad,
                         top_k=8)
        first = self._pick(top, snap.now)
        top = [v for v in top if v is first or self._pick([v], snap.now) is v][:5]
        if first is None or not top:
            self.done_keys |= keys_from_mask(grid, region)
            self.regions_done += 1
            self.events.append(f'HEATS region done (no viewpoint with gain); '
                               f'{self.regions_done} done so far')
            self.seed = None
            return None
        # Local stage: route through the best few, execute the first of the tour.
        order = order_open_tour(snap.robot[:2], [(v.x, v.y) for v in top])
        vp = top[order[0]]
        return Action('look', f'HEATS {vp!r} explore gain {vp.gain_explore_m2:.2f}m2 '
                      f'(1st of a {len(top)}-viewpoint tour)', viewpoint=vp)


class CameraGreedySearch(_Base):
    """Arm E, camera-range greedy exploration — see module docstring."""

    def next_action(self, snap):
        grid = snap.grid
        # Camera-unseen = not swept, whether known floor or still unknown.
        targets = (grid.free | grid.unknown) & ~snap.seen
        table, padded, cost_ok = self._look_setup(snap, targets)
        candidates = candidate_cells(grid, np.ones_like(targets), cost_ok,
                                     avoid=self._avoid(snap.now))
        best = plan_looks(grid, padded, table, candidates, snap.robot, spans=(0.0,),
                          fov_rad=self.fov_rad, top_k=5)
        vp = self._pick(best, snap.now)
        if vp is not None:
            return Action('look', f'greedy {vp!r}', viewpoint=vp)
        return self._exit_or_finish(snap, 'no viewpoint with camera gain')


def make_strategy(name, inspection_mode='mixed', **kwargs):
    """Build the strategy for frontier_explorer_node's search_strategy parameter."""
    if name == 'region':
        return RegionSearch(inspection_mode=inspection_mode, **kwargs)
    if name == 'heats':
        kwargs.pop('row_spacing_m', None)
        return HeatsSearch(**kwargs)
    if name == 'camera_greedy':
        kwargs.pop('row_spacing_m', None)
        return CameraGreedySearch(**kwargs)
    raise ValueError(f'unknown search strategy {name!r}')
