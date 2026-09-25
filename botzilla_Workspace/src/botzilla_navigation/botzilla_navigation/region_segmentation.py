"""
region_segmentation.py.

Pure, ROS-independent room segmentation of the 2D occupancy map, for the region-by-region
search strategies (research_discussion.md §8.2-8.3). Unlike the older helper modules this
one works on numpy arrays: segmentation runs a distance transform and several labelling
passes over the whole map every few seconds, which plain Python lists cannot do at
250x250 cells in time. Unit-tested in test/test_region_segmentation.py.

How rooms are found. A doorway is a gap narrower than door_width_m between occupied
cells. Every known-free cell gets its clearance (distance to the nearest OCCUPIED cell;
unknown space does not count as a wall, so a half-mapped room is not cut in two by its
own unknown edge). Cells with clearance >= door_width_m / 2 form room CORES: a doorway
narrower than door_width_m has no core cell in it, so the cores of two rooms joined by
a door are separate components. Each core is then grown back over all free space it can
reach (geodesic growth, one cell per pass), which hands every doorway and wall-side cell
to the room it opens off. Free space no core reaches (a corridor narrower than the door
width) becomes its own region flagged as a corridor, visited last (Gao et al.).

Open space without doorways is split by its SHAPE (split_by_shape). An L-, T- or U-shaped
space has no narrow passage, so the doorway test sees one room; but searching it "region
by region" should mean one arm, then the other. A region is L/T/U shaped when its
rectangularity (area / wall-aligned bounding-box area, holes such as furniture filled
first) is below NONRECT_THRESHOLD. For such a region every straight cut parallel to the
walls is tried
(the wall direction comes from the occupied cells, dominant_angle), and the cut that
leaves the two most rectangular parts is taken if it improves area-weighted
rectangularity by SPLIT_MIN_GAIN and leaves no part under SPLIT_MIN_PART_M2 or narrower
than SPLIT_MIN_EXTENT_M. For an L that is the cut
extending one arm's inner wall across the junction: two rectangles. Parts are split
again recursively (a U becomes three). A convex region is only cut when it exceeds
max_region_area_m2, and then in half across its long axis.

The cuts depend only on the map's shape, never on where the robot started. The first
version tiled oversized regions into 4 m squares on a grid anchored at the start pose.
On hardware (run_logs/20260925-154415, an L-shaped lab of ~67 m^2) that put the tile
corners exactly on the start point, cut the lab into arbitrary squares, and silently
shrank the robot's active region the moment the map passed the size cap.

Frozen boundaries are the caller's job (search_strategies.py): once a region is done its
cells are passed back in as `exclude`, so later segmentations never re-label them.
"""

import math

from botzilla_navigation.frontier_detection import OCCUPIED_THRESHOLD
import numpy as np
from scipy import ndimage

# Gaps narrower than this separate two regions. 1.0 m is wider than a normal interior
# door (~0.8 m) and than the gap between two boxes used as a partition, but narrower
# than any stretch of open floor worth calling a room.
DOOR_WIDTH_M = 1.0
# Cores smaller than this are noise (the space between two chair legs, a widening in
# a corridor) and are not seeds of their own region.
MIN_CORE_AREA_M2 = 0.5
# Regions smaller than this after growth are not worth a separate visit.
MIN_REGION_AREA_M2 = 0.5
# Shape split — see module docstring. A convex region is only halved above this area
# (a large living room): a normal room stays ONE region, which is the point of searching
# by room (§8.2). A 24 m^2 cap was tried first and cut four 6x6 m rooms into sixteen
# pieces — the many-small-units search the plan decided against.
MAX_REGION_AREA_M2 = 40.0
# Regions smaller than this are never split: a small non-convex room is still one look
# or two, and splitting it only adds stops.
SPLIT_MIN_AREA_M2 = 8.0
# No part of a cut may be smaller than this (about four camera looks).
SPLIT_MIN_PART_M2 = 3.0
# Below this rectangularity (cells / wall-aligned bounding box) a region is L/T/U
# shaped. A furnished rectangular room scores ~0.95 once its furniture holes are
# filled; an L with equal arms scores 0.75. The convex hull was tried first and is the
# wrong measure: an L's hull cuts across the inside corner, so a small L still scored
# 0.86 "convex" and was never split, and the cuts it preferred missed the corner.
NONRECT_THRESHOLD = 0.85
# A cut must raise area-weighted rectangularity by at least this much.
SPLIT_MIN_GAIN = 0.1
# No part of a cut may be narrower than this: a thin strip is not a room.
SPLIT_MIN_EXTENT_M = 1.2
CUT_STEP_M = 0.1
MAX_SPLIT_DEPTH = 4

_EIGHT = np.ones((3, 3), dtype=bool)


class Grid:
    """Numpy view of an OccupancyGrid's fields; world <-> cell helpers."""

    def __init__(self, data, width, height, resolution, origin_x, origin_y):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.values = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.free = (self.values >= 0) & (self.values < OCCUPIED_THRESHOLD)
        self.occupied = self.values >= OCCUPIED_THRESHOLD
        self.unknown = self.values < 0

    def cell_of(self, x, y):
        """Return (row, col) of world point (x, y); may be out of bounds."""
        return (int((y - self.origin_y) // self.resolution),
                int((x - self.origin_x) // self.resolution))

    def in_bounds(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    def world_of(self, row, col):
        """Return world (x, y) of the centre of cell (row, col)."""
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def world_keys(self):
        """Integer (ix, iy) world-cell keys for every cell, as two (h, w) arrays.

        Stable across RTAB-Map origin shifts (which move the origin by whole cells), so a
        set of keys identifies the same floor however the grid is later re-framed.
        """
        ix0 = int(round(self.origin_x / self.resolution))
        iy0 = int(round(self.origin_y / self.resolution))
        cols = np.arange(self.width) + ix0
        rows = np.arange(self.height) + iy0
        return np.broadcast_to(cols, (self.height, self.width)), \
            np.broadcast_to(rows[:, None], (self.height, self.width))


def pack_keys(ix, iy):
    """Pack world-cell keys into one int64 each, for set membership tests."""
    return (np.asarray(ix, dtype=np.int64) << 32) + (np.asarray(iy, dtype=np.int64) & 0xFFFFFFFF)


def mask_from_keys(grid, keys):
    """Boolean (h, w) mask of the grid cells whose world key is in `keys` (packed ints)."""
    if keys is None or len(keys) == 0:
        return np.zeros((grid.height, grid.width), dtype=bool)
    ix, iy = grid.world_keys()
    packed = pack_keys(ix, iy)
    return np.isin(packed, np.fromiter(keys, dtype=np.int64, count=len(keys)))


def keys_from_mask(grid, mask):
    """Packed world keys (a Python set) of the True cells of mask."""
    ix, iy = grid.world_keys()
    return set(pack_keys(ix[mask], iy[mask]).tolist())


def clearance_m(grid):
    """Distance in metres from every cell to the nearest occupied cell (unknown is free)."""
    if not grid.occupied.any():
        return np.full((grid.height, grid.width), np.inf)
    return ndimage.distance_transform_edt(~grid.occupied) * grid.resolution


def dominant_angle(grid):
    """Wall direction of the map, in [0, pi/2): the peak of edge orientations mod 90 deg.

    Walls in a building meet at right angles, so one angle serves both wall directions.
    The map frame follows the robot's start heading, not the walls, so cuts must not be
    assumed to run along x and y.
    """
    occ = grid.occupied.astype(float)
    gx = ndimage.sobel(occ, axis=1)
    gy = ndimage.sobel(occ, axis=0)
    mag = np.hypot(gx, gy)
    m = mag > 0
    if np.count_nonzero(m) < 20:
        return 0.0
    ang = np.mod(np.arctan2(gy[m], gx[m]), np.pi / 2.0)
    hist, edges = np.histogram(ang, bins=90, range=(0.0, np.pi / 2.0), weights=mag[m])
    hist = ndimage.uniform_filter1d(hist, 5, mode='wrap')
    k = int(np.argmax(hist))
    return float((edges[k] + edges[k + 1]) / 2.0)


def _rectangularity(u, v, n_cells):
    """Cells / area of their wall-aligned bounding box, extents trimmed at 1%/99%.

    Trimming keeps a few stray cells (map speckle, a leak under a door) from inflating
    the box. 1.0 for a rectangle; an L with equal arms is 0.75.
    """
    if n_cells == 0:
        return 1.0
    u_lo, u_hi = np.percentile(u, (1, 99))
    v_lo, v_hi = np.percentile(v, (1, 99))
    box = (u_hi - u_lo + 1.0) * (v_hi - v_lo + 1.0)
    return min(1.0, n_cells / max(1.0, box))


class _Shape:
    """A region's cells in wall-aligned coordinates, for scoring straight cuts."""

    def __init__(self, mask, angle):
        filled = ndimage.binary_fill_holes(mask)
        rows, cols = np.nonzero(filled)
        ca, sa = math.cos(angle), math.sin(angle)
        self.u = cols * ca + rows * sa
        self.v = -cols * sa + rows * ca
        self.n = rows.size
        # Scoring every candidate cut on every cell is slow on a 60 m^2 room (24k
        # cells); a 1-in-4 sample gives the same extents to within a cell.
        # Each sampled cell stands for `weight` cells when compared with box areas.
        k = max(1, self.n // 6000)
        self.su, self.sv = self.u[::k], self.v[::k]
        self.weight = self.n / self.su.size
        self.rect = _rectangularity(self.su, self.sv, self.n)

    def cut_score(self, axis, t):
        """(area-weighted rectangularity, cells below, cells above, min part extent)."""
        c, o = (self.su, self.sv) if axis == 0 else (self.sv, self.su)
        below = c < t
        n_a = int(np.count_nonzero(below))
        n_b = c.size - n_a
        if n_a < 2 or n_b < 2:
            return 0.0, n_a, n_b, 0.0
        r_a = _rectangularity(c[below], o[below], n_a * self.weight)
        r_b = _rectangularity(c[~below], o[~below], n_b * self.weight)
        extent = min(np.ptp(c[below]), np.ptp(o[below]), np.ptp(c[~below]), np.ptp(o[~below]))
        return (n_a * r_a + n_b * r_b) / c.size, n_a, n_b, extent


def _split_region(mask, angle, cell_area, max_area_m2, depth=0):
    """Recursively split one region's mask by shape — see module docstring."""
    area = np.count_nonzero(mask) * cell_area
    if depth >= MAX_SPLIT_DEPTH or area < SPLIT_MIN_AREA_M2:
        return [mask]
    shape = _Shape(mask, angle)
    cell = math.sqrt(cell_area)
    frac_min = SPLIT_MIN_PART_M2 / max(area, 1e-9)
    step = max(1.0, CUT_STEP_M / cell)
    min_extent = SPLIT_MIN_EXTENT_M / cell
    cut = None
    if shape.rect < NONRECT_THRESHOLD:
        best = None
        for axis, coord in ((0, shape.u), (1, shape.v)):
            for t in np.arange(coord.min() + step, coord.max(), step):
                score, n_a, n_b, extent = shape.cut_score(axis, t)
                total = n_a + n_b
                if min(n_a, n_b) < frac_min * total or extent < min_extent:
                    continue
                if best is None or score > best[0]:
                    best = (score, axis, t)
        if best is not None and best[0] >= shape.rect + SPLIT_MIN_GAIN:
            cut = best[1:]
    if cut is None and area > max_area_m2:
        # Rectangular but oversized: halve it across its longer extent.
        axis = 0 if np.ptp(shape.u) >= np.ptp(shape.v) else 1
        cut = (axis, float(np.median(shape.u if axis == 0 else shape.v)))
    if cut is None:
        return [mask]
    axis, t = cut
    rows, cols = np.nonzero(mask)
    ca, sa = math.cos(angle), math.sin(angle)
    coord = (cols * ca + rows * sa) if axis == 0 else (-cols * sa + rows * ca)
    below = np.zeros_like(mask)
    below[rows[coord < t], cols[coord < t]] = True
    min_cells = SPLIT_MIN_PART_M2 / cell_area
    parts = []
    for half in (mask & below, mask & ~below):
        pieces, n = ndimage.label(half, structure=_EIGHT)
        for k in range(1, n + 1):
            piece = pieces == k
            if np.count_nonzero(piece) >= min_cells:
                parts.extend(_split_region(piece, angle, cell_area, max_area_m2, depth + 1))
            # Smaller pieces are left unlabelled here and handed to a neighbouring part
            # by segment_regions' final growth pass.
    return parts or [mask]


def _grow(labels, allowed):
    """Grow labels geodesically into allowed cells until nothing changes."""
    labels = labels.copy()
    while True:
        frontier = allowed & (labels == 0)
        if not frontier.any():
            break
        grown = ndimage.grey_dilation(labels, footprint=_EIGHT)
        take = frontier & (grown > 0)
        if not take.any():
            break
        labels[take] = grown[take]
    return labels


class Segmentation:
    """Result of segment_regions: labels plus per-region facts."""

    def __init__(self, labels, corridor_ids, clearance):
        self.labels = labels            # (h, w) int32, 0 = not in any region
        self.corridor_ids = corridor_ids
        self.clearance = clearance      # (h, w) metres, see clearance_m
        ids = np.unique(labels)
        self.ids = [int(i) for i in ids if i != 0]

    def cells(self, region_id):
        return self.labels == region_id

    def is_corridor(self, region_id):
        return region_id in self.corridor_ids


def segment_regions(
    grid, door_width_m=DOOR_WIDTH_M, min_core_area_m2=MIN_CORE_AREA_M2,
    min_region_area_m2=MIN_REGION_AREA_M2, max_region_area_m2=MAX_REGION_AREA_M2,
    exclude=None, split_shapes=True,
):
    """Label the known-free cells of grid by region — see module docstring.

    exclude: optional boolean (h, w) mask of cells that must not be in any region (the
    cells of regions already done). Returns a Segmentation.
    """
    cell_area = grid.resolution * grid.resolution
    free = grid.free.copy()
    if exclude is not None:
        free &= ~exclude
    clearance = clearance_m(grid)

    core = free & (clearance >= door_width_m / 2.0)
    core_labels, n_cores = ndimage.label(core, structure=_EIGHT)
    if n_cores:
        sizes = ndimage.sum_labels(core, core_labels, index=np.arange(1, n_cores + 1))
        small = np.flatnonzero(sizes * cell_area < min_core_area_m2) + 1
        if small.size:
            core_labels[np.isin(core_labels, small)] = 0
    labels = _grow(core_labels.astype(np.int32), free)

    # Free space no core reached: narrow corridors (or narrow leftovers of done regions).
    next_id = int(labels.max()) + 1
    corridor_ids = set()
    rest_labels, n_rest = ndimage.label(free & (labels == 0), structure=_EIGHT)
    for k in range(1, n_rest + 1):
        part = rest_labels == k
        if part.sum() * cell_area < min_region_area_m2:
            continue
        labels[part] = next_id
        corridor_ids.add(next_id)
        next_id += 1

    # Open space: split non-convex (L/T/U) and oversized regions by shape.
    if split_shapes:
        angle = dominant_angle(grid)
        in_regions = labels > 0
        for region_id in [int(i) for i in np.unique(labels) if i != 0]:
            if region_id in corridor_ids:
                continue
            part = labels == region_id
            pieces = _split_region(part, angle, cell_area, max_region_area_m2)
            if len(pieces) == 1 and pieces[0] is part:
                continue
            labels[part] = 0
            for k, piece in enumerate(pieces):
                if k == 0:
                    labels[piece] = region_id
                else:
                    labels[piece] = next_id
                    next_id += 1
        # Slivers too small to be parts go to the part they touch.
        labels = _grow(labels, in_regions)

    # Drop regions that ended up too small.
    for region_id in [int(i) for i in np.unique(labels) if i != 0]:
        part = labels == region_id
        if part.sum() * cell_area < min_region_area_m2:
            labels[part] = 0
            corridor_ids.discard(region_id)

    return Segmentation(labels, corridor_ids, clearance)


def classify_frontier(seg, cells, active_id, door_width_m=DOOR_WIDTH_M):
    """Return 'inside' or 'exit' for a frontier cluster given as [(row, col), ...].

    Inside = most of its cells belong to the active region AND it is not narrow. A
    frontier is narrow when its median clearance is under half the door width: unknown
    space seen through a doorway (walls either side of the gap). Everything else —
    frontiers of other regions, or of no region — is an exit, locked until the active
    region is done (§8.3).
    """
    if not cells:
        return 'exit'
    rows = np.fromiter((c[0] for c in cells), dtype=np.int64, count=len(cells))
    cols = np.fromiter((c[1] for c in cells), dtype=np.int64, count=len(cells))
    labels = seg.labels[rows, cols]
    if active_id is None or np.count_nonzero(labels == active_id) * 2 < len(cells):
        return 'exit'
    if float(np.median(seg.clearance[rows, cols])) < door_width_m / 2.0:
        return 'exit'
    return 'inside'
