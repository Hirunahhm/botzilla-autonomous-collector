"""Tests for viewpoint_planning: visibility, turn-range choice, costs, tours."""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from botzilla_navigation.region_segmentation import Grid  # noqa: E402,I100
from botzilla_navigation.viewpoint_planning import (  # noqa: E402
    FULL_SPIN_RAD,
    order_open_tour,
    PaddedGrids,
    plan_looks,
    RayTable,
    visible_histogram,
    window_gains,
)

RES = 0.05


def open_grid(n=80):
    a = np.zeros((n, n), dtype=int)
    return Grid(a.ravel().tolist(), n, n, RES, 0.0, 0.0), a


def test_blind_ring_and_range_limit_the_targets():
    table = RayTable(RES, 0.48, 1.0, stride=1)
    d = np.hypot(table.dr, table.dc) * RES
    assert d.min() >= 0.48 and d.max() <= 1.0


def test_wall_blocks_what_is_behind_it():
    g, a = open_grid()
    targets = np.ones_like(g.free)
    occ = np.zeros_like(g.free)
    table = RayTable(RES, 0.48, 1.0, stride=1)
    open_hist = visible_histogram(PaddedGrids(targets, occ, table.pad), table, 40, 40)
    occ[:, 50] = True   # wall 0.5 m east of the robot, full height
    walled = visible_histogram(PaddedGrids(targets, occ, table.pad), table, 40, 40)
    east = [0, 1, 35]   # bins around bearing 0
    assert all(walled[b] < open_hist[b] for b in east)
    assert walled[18] == open_hist[18]   # due west unaffected


def test_window_gains_cover_the_span():
    hist = np.zeros(36, dtype=int)
    hist[9] = 10    # everything is at bearing ~95 deg
    gains, headings = window_gains(hist, math.radians(57), 0.0)
    s = int(np.argmax(gains))
    assert gains[s] == 10
    assert abs(math.degrees(headings[s]) - 95) < 35
    full, _ = window_gains(hist, math.radians(57), FULL_SPIN_RAD)
    assert (full == 10).all()


def _plan(targets, robot, spans):
    g, a = open_grid()
    table = RayTable(RES, 0.48, 1.0)
    padded = PaddedGrids(targets, np.zeros_like(targets), table.pad)
    return plan_looks(g, padded, table, [(40, 40)], robot, spans=spans)


def test_unseen_floor_all_around_chooses_a_spin():
    g, _ = open_grid()
    targets = np.ones_like(g.free)
    vp = _plan(targets, (2.02, 2.02, 0.0), spans=(0.0, math.pi, FULL_SPIN_RAD))[0]
    assert vp.span > 0


def test_unseen_floor_in_one_direction_chooses_one_look():
    g, _ = open_grid()
    targets = np.zeros_like(g.free)
    targets[36:45, 52:59] = True     # ~0.16 m^2 patch 0.6-0.95 m east
    vp = _plan(targets, (2.02, 2.02, 0.0), spans=(0.0, math.pi, FULL_SPIN_RAD))[0]
    assert vp.span == 0.0
    assert abs(vp.heading) < math.radians(40)


def test_nothing_to_see_returns_no_viewpoint():
    g, _ = open_grid()
    assert _plan(np.zeros_like(g.free), (2.0, 2.0, 0.0), spans=(0.0,)) == []


def test_open_tour_is_optimal_for_a_line():
    pts = [(3.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert order_open_tour((0.0, 0.0), pts) == [1, 2, 0]
    many = [(float(i), 0.0) for i in range(10, 0, -1)]
    assert order_open_tour((0.0, 0.0), many) == list(range(9, -1, -1))
