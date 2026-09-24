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

Open space. A region larger than max_region_area_m2 is cut into tile_m x tile_m tiles on
a grid anchored at the map frame's origin (the robot's start pose), so tile boundaries
do not move as the map grows — the "bounded virtual regions" fallback of §8.3.

Frozen boundaries are the caller's job (search_strategies.py): once a region is done its
cells are passed back in as `exclude`, so later segmentations never re-label them.
"""

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
# Open-space fallback — see module docstring. Only regions bigger than a large living
# room (40 m^2) are tiled: a normal room stays ONE region, which is the point of
# searching by room (§8.2). A 24 m^2 cap was tried first and cut four 6x6 m rooms into
# sixteen tiles — the many-small-units search the plan decided against.
TILE_M = 4.0
MAX_REGION_AREA_M2 = 40.0

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
    min_region_area_m2=MIN_REGION_AREA_M2, tile_m=TILE_M,
    max_region_area_m2=MAX_REGION_AREA_M2, exclude=None,
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

    # Open-space fallback: tile oversized regions on a fixed world grid.
    if tile_m > 0:
        ix, iy = grid.world_keys()
        tile_cells = max(1, int(round(tile_m / grid.resolution)))
        tile_x = np.floor_divide(ix, tile_cells)
        tile_y = np.floor_divide(iy, tile_cells)
        for region_id in [int(i) for i in np.unique(labels) if i != 0]:
            part = labels == region_id
            if part.sum() * cell_area <= max_region_area_m2:
                continue
            tiles = set(zip(tile_x[part].tolist(), tile_y[part].tolist()))
            first = True
            for tx, ty in sorted(tiles):
                sub = part & (tile_x == tx) & (tile_y == ty)
                # A tile can hold several disconnected pieces of the region; each piece
                # is its own region so that "the region" is always one driveable area.
                pieces, n_pieces = ndimage.label(sub, structure=_EIGHT)
                for p in range(1, n_pieces + 1):
                    piece = pieces == p
                    if first:
                        labels[piece] = region_id
                        first = False
                    else:
                        labels[piece] = next_id
                        if region_id in corridor_ids:
                            corridor_ids.add(next_id)
                        next_id += 1

    # Drop regions that ended up too small (tile slivers).
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
