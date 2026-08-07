"""
test_frontier_detection.py.

Pure pytest for botzilla_navigation.frontier_detection — no ROS imports, runnable directly
with `python3 -m pytest` without sourcing a ROS environment.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from botzilla_navigation.frontier_detection import (  # noqa: E402,I100
    distance,
    find_frontiers,
    find_low_cost_point,
    grid_to_world,
    select_target,
    world_to_grid,
)

UNK = -1   # unknown
FREE = 0   # free
OCC = 100  # occupied


def grid(rows, width):
    """Flatten a list-of-rows into the flat row-major format find_frontiers expects."""
    flat = [v for row in rows for v in row]
    height = len(rows)
    assert all(len(row) == width for row in rows)
    return flat, width, height


def test_fully_known_grid_has_no_frontiers():
    rows = [
        [FREE, FREE, FREE],
        [FREE, OCC, FREE],
        [FREE, FREE, FREE],
    ]
    data, w, h = grid(rows, 3)
    assert find_frontiers(data, w, h, min_cluster_size=1) == []


def test_single_frontier_cluster():
    rows = [
        [UNK, UNK, UNK, UNK],
        [UNK, UNK, UNK, UNK],
        [FREE, FREE, FREE, FREE],
        [FREE, FREE, FREE, FREE],
    ]
    data, w, h = grid(rows, 4)
    clusters = find_frontiers(data, w, h, min_cluster_size=1)
    # Every free cell in row 2 borders an unknown cell in row 1 -> one connected cluster.
    assert len(clusters) == 1
    target_row, target_col, size = clusters[0]
    assert size == 4
    # Mean of the cluster is (2.0, 1.5); the nearest actual member cell to that mean
    # is (2, 1) (tied with (2, 2), first one wins) — never a non-member/fractional point.
    assert target_row == 2
    assert target_col == 1


def test_frontier_target_is_always_a_member_cell_for_l_shaped_cluster():
    """Regression test: an L-shaped cluster's raw mean lands outside the cluster.

    Before this fix, find_frontiers() returned the mean of all member cells as the
    target. Here the 5 frontier cells trace an L down column 0 then across row 2:
    (0,0),(1,0),(2,0),(2,1),(2,2). Their mean is (1.4, 0.6) — not a member of the
    cluster at all, and it lands almost exactly on cell (1,1), which is UNKNOWN space
    (not even free). Nav2 would be handed that as a goal every time this cluster shape
    recurred, and exploration would stall on that spot. The target must always be one
    of the cluster's own (free, in-cluster) member cells.
    """
    rows = [
        [FREE, UNK, OCC, OCC],
        [FREE, UNK, OCC, OCC],
        [FREE, FREE, FREE, UNK],
        [UNK, OCC, OCC, OCC],
    ]
    data, w, h = grid(rows, 4)
    clusters = find_frontiers(data, w, h, min_cluster_size=1)
    assert len(clusters) == 1
    target_row, target_col, size = clusters[0]
    member_cells = {(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)}
    assert size == 5
    assert (target_row, target_col) in member_cells


def test_two_disconnected_clusters_not_merged():
    rows = [
        [UNK, FREE, FREE, OCC, OCC, FREE, FREE, UNK],
    ]
    data, w, h = grid(rows, 8)
    clusters = find_frontiers(data, w, h, min_cluster_size=1)
    assert len(clusters) == 2
    sizes = sorted(c[2] for c in clusters)
    assert sizes == [1, 1]  # only the cell directly touching UNK on each side is a frontier


def test_small_cluster_filtered_by_min_size():
    rows = [
        [UNK, FREE, OCC, OCC],
        [OCC, OCC, OCC, OCC],
    ]
    data, w, h = grid(rows, 4)
    # The single free cell at (0,1) borders unknown at (0,0): a real but tiny cluster.
    assert find_frontiers(data, w, h, min_cluster_size=1) != []
    assert find_frontiers(data, w, h, min_cluster_size=2) == []


def test_diagonal_frontier_cells_merge_via_8_connectivity():
    """Regression test: two frontier cells touching only diagonally must merge.

    Before this fix, find_frontiers() only connected frontier cells 4-connectively, so
    two genuinely-adjacent (diagonal) frontier cells near a corner would each form
    their own 1-cell cluster and both get silently discarded as noise once
    min_cluster_size > 1, even though combined they'd clear it.
    """
    rows = [
        [FREE, UNK],
        [UNK, FREE],
    ]
    data, w, h = grid(rows, 2)
    clusters = find_frontiers(data, w, h, min_cluster_size=2)
    assert len(clusters) == 1
    _row, _col, size = clusters[0]
    assert size == 2


def test_frontier_classification_uses_8_connectivity_too():
    """A free cell touching unknown space only diagonally must count as a frontier.

    Matches the same 8-connectivity used for clustering — see
    test_diagonal_frontier_cells_merge_via_8_connectivity.
    """
    rows = [
        [UNK, OCC],
        [OCC, FREE],
    ]
    data, w, h = grid(rows, 2)
    clusters = find_frontiers(data, w, h, min_cluster_size=1)
    assert len(clusters) == 1
    assert clusters[0][:2] == (1, 1)


def test_grid_to_world_and_back():
    resolution = 0.05
    origin_x, origin_y = -2.0, -3.0
    x, y = grid_to_world(
        row=10, col=4, resolution=resolution, origin_x=origin_x, origin_y=origin_y
    )
    assert x == origin_x + 4.5 * resolution
    assert y == origin_y + 10.5 * resolution
    row, col = world_to_grid(x, y, resolution=resolution, origin_x=origin_x, origin_y=origin_y)
    assert row == 10
    assert col == 4


def test_select_target_picks_nearest():
    frontiers = [(5.0, 5.0), (1.0, 1.0), (-3.0, 0.0)]
    target = select_target(frontiers, robot_x=0.0, robot_y=0.0)
    assert target == (1.0, 1.0)


def test_select_target_empty_returns_none():
    assert select_target([], robot_x=0.0, robot_y=0.0) is None


def test_distance():
    assert distance(0.0, 0.0, 3.0, 4.0) == 5.0


def test_find_low_cost_point_returns_start_if_already_safe():
    rows = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
    ]
    data, w, h = grid(rows, 3)
    assert find_low_cost_point(data, w, h, row=1, col=1, max_cost=50) == (1, 1)


def test_find_low_cost_point_snaps_away_from_wall_inflation():
    """Regression test for the live "stuck exploration" bug.

    A frontier cell sitting in a wall's inflation halo (cost 99, just under lethal 100)
    is free in the raw SLAM map but unreachable in the costmap Nav2 actually plans
    against. find_low_cost_point must walk outward to the nearest cell clear of that
    halo rather than hand back the frontier cell itself.
    """
    rows = [
        [100, 99, 99, 10, 0],
        [100, 99, 99, 10, 0],
        [100, 99, 99, 10, 0],
    ]
    data, w, h = grid(rows, 5)
    # Frontier cell at (1, 1): buried in inflation (cost 99).
    result = find_low_cost_point(data, w, h, row=1, col=1, max_cost=50, search_radius=10)
    assert result is not None
    r, c = result
    assert data[r * w + c] < 50


def test_find_low_cost_point_returns_none_when_nothing_nearby():
    rows = [[99] * 5 for _ in range(5)]
    data, w, h = grid(rows, 5)
    assert find_low_cost_point(data, w, h, row=2, col=2, max_cost=50, search_radius=2) is None


def test_find_low_cost_point_out_of_bounds_returns_none():
    rows = [[0, 0], [0, 0]]
    data, w, h = grid(rows, 2)
    assert find_low_cost_point(data, w, h, row=5, col=5, max_cost=50) is None


def test_find_low_cost_point_ignores_unknown_cells():
    rows = [
        [99, 99, -1],
        [99, 99, -1],
        [99, 99, -1],
    ]
    data, w, h = grid(rows, 3)
    # Everything reachable is either inflated (99) or unknown (-1); no cell qualifies.
    assert find_low_cost_point(data, w, h, row=1, col=1, max_cost=50, search_radius=5) is None
