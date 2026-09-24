"""Tests for search_strategies: the decision sequences of each research arm."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from botzilla_navigation.region_segmentation import Grid  # noqa: E402,I100
from botzilla_navigation.search_strategies import (  # noqa: E402
    make_strategy,
    Snapshot,
)
from test_region_segmentation import RES, two_rooms  # noqa: E402


def snap(a, seen=None, robot=(0.8, 0.8, 0.0), frontiers=(), given_up=False):
    g = Grid(a.ravel().tolist(), a.shape[1], a.shape[0], RES, 0.0, 0.0)
    if seen is None:
        seen = np.zeros_like(g.free)
    return Snapshot(g, None, seen, robot, list(frontiers), now=0.0,
                    frontiers_given_up=given_up)


def room_mask(a, wall_col, left=True):
    m = np.zeros(a.shape, dtype=bool)
    if left:
        m[:, :wall_col] = True
    else:
        m[:, wall_col + 1:] = True
    return m


def test_region_search_rows_first_then_next_room_then_done():
    a, wall_col = two_rooms()
    s = make_strategy('region', inspection_mode='mixed')
    act = s.next_action(snap(a))
    assert act.kind == 'rows' and act.waypoints
    # Every row waypoint is in the left room: the right room is locked until done.
    assert all(x < wall_col * RES for x, _ in act.waypoints)
    seen = room_mask(a, wall_col, left=True)
    act = s.next_action(snap(a, seen))
    assert act.kind == 'rows'
    assert all(x > wall_col * RES for x, _ in act.waypoints)
    act = s.next_action(snap(a, np.ones(a.shape, dtype=bool)))
    assert act.kind == 'done'
    assert s.regions_done == 2


def test_region_search_explores_inside_frontiers_before_inspecting():
    a, wall_col = two_rooms()
    a[1:10, 1:20] = -1      # an unmapped corner of the left room
    cells = [(10, c) for c in range(2, 19)]
    frontier = {'x': 0.5, 'y': 0.55, 'cells': cells, 'blacklisted': False}
    s = make_strategy('region', inspection_mode='mixed')
    act = s.next_action(snap(a, frontiers=[frontier]))
    assert act.kind == 'frontier' and act.target == (0.5, 0.55)


def test_region_search_does_not_leave_through_an_exit_early():
    a, wall_col = two_rooms()
    a[:, wall_col + 1:] = -1
    door = [(r, wall_col) for r in range(a.shape[0]) if a[r, wall_col] == 0]
    exit_frontier = {'x': wall_col * RES, 'y': 1.5, 'cells': door, 'blacklisted': False}
    s = make_strategy('region', inspection_mode='viewpoints')
    act = s.next_action(snap(a, frontiers=[exit_frontier]))
    assert act.kind == 'look'
    # Once the room is seen, the exit is the next move.
    act = s.next_action(snap(a, room_mask(a, wall_col), frontiers=[exit_frontier]))
    assert act.kind == 'frontier'


def test_one_look_mode_never_turns():
    a, wall_col = two_rooms()
    s = make_strategy('region', inspection_mode='one_look')
    s.rows_done = True
    first = s.next_action(snap(a))
    if first.kind == 'rows':   # the row pass comes first; skip it
        first = s.next_action(snap(a))
    assert first.kind == 'look' and first.viewpoint.span == 0.0


def test_spin_grid_uses_full_spins():
    a, wall_col = two_rooms()
    s = make_strategy('region', inspection_mode='spin_grid')
    act = s.next_action(snap(a))
    assert act.kind == 'look' and act.viewpoint.span > 5.0


def test_heats_search_looks_inside_one_region_first():
    a, wall_col = two_rooms()
    s = make_strategy('heats')
    act = s.next_action(snap(a))
    assert act.kind == 'look' and act.viewpoint.span == 0.0
    first_side = act.viewpoint.x < wall_col * RES
    for _ in range(3):
        act = s.next_action(snap(a))
        assert (act.viewpoint.x < wall_col * RES) == first_side


def test_camera_greedy_finishes_when_everything_is_seen():
    a, wall_col = two_rooms()
    s = make_strategy('camera_greedy')
    assert s.next_action(snap(a)).kind == 'look'
    assert s.next_action(snap(a, np.ones(a.shape, dtype=bool))).kind == 'done'


def test_blacklisted_frontiers_mean_wait_until_given_up():
    a, wall_col = two_rooms()
    f = {'x': 1.0, 'y': 1.0, 'cells': [], 'blacklisted': True}
    s = make_strategy('camera_greedy')
    everything = np.ones(a.shape, dtype=bool)
    assert s.next_action(snap(a, everything, frontiers=[f])).kind == 'wait'
    assert s.next_action(snap(a, everything, frontiers=[f], given_up=True)).kind == 'done'
