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
    centroid_row, centroid_col, size = clusters[0]
    assert size == 4
    assert centroid_row == 2.0
    assert centroid_col == 1.5  # mean of 0,1,2,3


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
