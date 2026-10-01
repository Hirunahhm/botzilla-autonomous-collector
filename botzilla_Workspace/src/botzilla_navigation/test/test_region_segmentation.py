"""Tests for region_segmentation: rooms split at doorways, tiles, frontier classes."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from botzilla_navigation.region_segmentation import (  # noqa: E402,I100
    classify_frontier,
    Grid,
    keys_from_mask,
    mask_from_keys,
    segment_regions,
)

RES = 0.05


def two_rooms(door_m=0.6, room_m=3.0):
    """Two room_m x room_m rooms side by side, joined by a door_m doorway. Walls = 100."""
    n = int(round(room_m / RES))
    w = 2 * n + 3
    h = n + 2
    a = np.zeros((h, w), dtype=int)
    a[0, :] = a[-1, :] = 100
    a[:, 0] = a[:, -1] = 100
    wall_col = n + 1
    a[:, wall_col] = 100
    door = int(round(door_m / RES))
    mid = h // 2
    a[mid - door // 2: mid - door // 2 + door, wall_col] = 0
    return a, wall_col


def grid_of(a, ox=0.0, oy=0.0):
    return Grid(a.ravel().tolist(), a.shape[1], a.shape[0], RES, ox, oy)


def test_doorway_splits_two_rooms():
    a, wall_col = two_rooms()
    seg = segment_regions(grid_of(a))
    assert len(seg.ids) == 2
    left = seg.labels[20, 10]
    right = seg.labels[20, wall_col + 10]
    assert left and right and left != right
    assert not seg.corridor_ids


def test_wide_opening_is_one_region():
    a, _ = two_rooms(door_m=2.4)
    seg = segment_regions(grid_of(a))
    assert len(seg.ids) == 1


def enclose(free):
    """Occupancy grid of `free` floor with a one-cell wall around it, unknown outside."""
    from scipy import ndimage
    a = np.full(free.shape, -1, dtype=int)
    a[free] = 0
    a[ndimage.binary_dilation(free) & ~free] = 100
    return a


def areas(seg):
    return sorted((round(float(seg.cells(i).sum()) * RES * RES, 1) for i in seg.ids),
                  reverse=True)


def test_l_shaped_room_splits_into_its_two_arms():
    free = np.zeros((170, 170), dtype=bool)
    free[5:85, 5:165] = True     # 4 m x 8 m arm
    free[85:165, 5:85] = True    # 4 m x 4 m arm
    assert areas(segment_regions(grid_of(enclose(free)))) == [32.0, 16.0]


def test_small_l_still_splits_but_a_small_room_does_not():
    free = np.zeros((130, 130), dtype=bool)
    free[5:65, 5:125] = True
    free[65:125, 5:65] = True
    assert areas(segment_regions(grid_of(enclose(free)))) == [18.0, 9.0]
    square = np.zeros((130, 130), dtype=bool)
    square[5:125, 5:125] = True
    assert areas(segment_regions(grid_of(enclose(square)))) == [36.0]


def test_rectangular_room_with_furniture_is_not_split_by_shape():
    free = np.zeros((130, 130), dtype=bool)
    free[5:125, 5:125] = True
    a = enclose(free)
    for r, c in ((30, 30), (30, 80), (80, 50), (95, 95)):   # tables well inside
        a[r:r + 8, c:c + 12] = 100
    seg = segment_regions(grid_of(a))
    assert len(seg.ids) == 1


def test_large_open_space_is_halved_not_tiled_from_the_start_pose():
    free = np.pad(np.ones((200, 200), dtype=bool), 5)    # 10 x 10 m, no walls inside
    seg = segment_regions(grid_of(enclose(free), ox=-5.25, oy=-5.25))
    assert areas(seg) == [25.0, 25.0, 25.0, 25.0]
    # The cuts follow the room, not the map origin: moving the origin (the robot's
    # start) must not move them relative to the room.
    seg2 = segment_regions(grid_of(enclose(free), ox=-2.0, oy=-7.0))
    assert np.array_equal(seg.labels > 0, seg2.labels > 0)
    assert len(seg2.ids) == 4


def test_excluded_cells_get_no_region():
    a, wall_col = two_rooms()
    g = grid_of(a)
    first = segment_regions(g)
    left_id = first.labels[20, 10]
    done = first.cells(left_id)
    seg = segment_regions(g, exclude=done)
    assert seg.labels[20, 10] == 0
    assert len(seg.ids) == 1


def test_world_keys_survive_an_origin_shift():
    a, _ = two_rooms()
    g1 = grid_of(a)
    mask = np.zeros_like(g1.free)
    mask[10, 12] = True
    keys = keys_from_mask(g1, mask)
    # Same map with 4 extra columns of unknown on the left: origin moves 4 cells left.
    b = np.hstack([np.full((a.shape[0], 4), -1), a])
    g2 = grid_of(b, ox=-4 * RES)
    m2 = mask_from_keys(g2, keys)
    assert m2.sum() == 1 and m2[10, 16]


def test_doorway_frontier_is_an_exit_and_open_frontier_is_inside():
    a, wall_col = two_rooms()
    a[:, wall_col + 1:] = -1          # right room not mapped yet
    a[:, wall_col] = np.where(a[:, wall_col] == 0, 0, 100)
    g = grid_of(a)
    seg = segment_regions(g)
    left = seg.labels[20, 10]
    door_cells = [(r, wall_col) for r in range(a.shape[0]) if a[r, wall_col] == 0]
    assert classify_frontier(seg, door_cells, left) == 'exit'
    # An open edge of the left room: knock out its top-left wall so unknown is beyond.
    b = a.copy()
    b[1:10, 1:20] = -1
    g2 = grid_of(b)
    seg2 = segment_regions(g2)
    left2 = seg2.labels[30, 10]
    edge = [(10, c) for c in range(2, 19)]
    assert classify_frontier(seg2, edge, left2) == 'inside'
