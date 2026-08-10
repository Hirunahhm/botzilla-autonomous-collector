"""
test_coverage_planning.py.

Pure pytest for botzilla_navigation.coverage_planning — no ROS imports, runnable directly
with `python3 -m pytest` without sourcing a ROS environment.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from botzilla_navigation.coverage_planning import generate_coverage_waypoints  # noqa: E402,I100

UNK = -1   # unknown
FREE = 0   # free
OCC = 100  # occupied


def grid(rows, width):
    """Flatten a list-of-rows into the flat row-major format the function expects."""
    flat = [v for row in rows for v in row]
    height = len(rows)
    assert all(len(row) == width for row in rows)
    return flat, width, height


def test_empty_map_has_no_waypoints():
    rows = [[UNK] * 5 for _ in range(4)]
    data, w, h = grid(rows, 5)
    waypoints = generate_coverage_waypoints(
        data, w, h, resolution=1.0, origin_x=0.0, origin_y=0.0,
        row_spacing_m=1.0, min_run_m=1.0,
    )
    assert waypoints == []


def test_single_open_rectangle_snake_order():
    rows = [[FREE] * 5 for _ in range(4)]
    data, w, h = grid(rows, 5)
    waypoints = generate_coverage_waypoints(
        data, w, h, resolution=1.0, origin_x=0.0, origin_y=0.0,
        row_spacing_m=1.0, min_run_m=1.0,
    )
    # Every row is a single 5-cell run -> 2 waypoints/row, alternating direction, and each
    # row's entry column matches the previous row's exit column (continuous snake).
    assert waypoints == [
        (0.5, 0.5), (4.5, 0.5),   # row 0: left -> right
        (4.5, 1.5), (0.5, 1.5),   # row 1: right -> left
        (0.5, 2.5), (4.5, 2.5),   # row 2: left -> right
        (4.5, 3.5), (0.5, 3.5),   # row 3: right -> left
    ]


def test_row_split_by_obstacle_produces_two_runs():
    rows = [[FREE, FREE, FREE, OCC, OCC, FREE, FREE, FREE]]
    data, w, h = grid(rows, 8)
    waypoints = generate_coverage_waypoints(
        data, w, h, resolution=1.0, origin_x=0.0, origin_y=0.0,
        row_spacing_m=1.0, min_run_m=3.0,
    )
    assert waypoints == [
        (0.5, 0.5), (2.5, 0.5),   # left run: cols 0-2
        (5.5, 0.5), (7.5, 0.5),   # right run: cols 5-7
    ]


def test_run_shorter_than_min_run_is_dropped():
    rows = [[FREE, FREE, UNK, UNK, UNK]]
    data, w, h = grid(rows, 5)
    waypoints = generate_coverage_waypoints(
        data, w, h, resolution=1.0, origin_x=0.0, origin_y=0.0,
        row_spacing_m=1.0, min_run_m=3.0,
    )
    # The only free run is 2 cells long, under the 3-cell minimum.
    assert waypoints == []


def test_row_spacing_smaller_than_one_cell_clamps_to_every_row():
    rows = [[FREE, FREE, FREE] for _ in range(3)]
    data, w, h = grid(rows, 3)
    waypoints = generate_coverage_waypoints(
        data, w, h, resolution=0.05, origin_x=0.0, origin_y=0.0,
        row_spacing_m=0.01, min_run_m=0.05,
    )
    # row_spacing_m (0.01) is under one cell (0.05); must clamp to 1 cell, not loop forever
    # or skip rows, so all 3 rows are visited (2 waypoints each).
    visited_rows = {round(y / 0.05 - 0.5) for (_x, y) in waypoints}
    assert visited_rows == {0, 1, 2}
    assert len(waypoints) == 6


def test_skips_fully_swept_runs():
    rows = [[FREE] * 5]
    data, w, h = grid(rows, 5)
    swept_mask = [True] * 5
    waypoints = generate_coverage_waypoints(
        data, w, h, resolution=1.0, origin_x=0.0, origin_y=0.0,
        row_spacing_m=1.0, min_run_m=1.0, swept_mask=swept_mask,
    )
    assert waypoints == []


def test_keeps_partially_swept_runs():
    rows = [[FREE] * 5]
    data, w, h = grid(rows, 5)
    # Every cell swept except one — the run as a whole is still un-covered ground.
    swept_mask = [True, True, False, True, True]
    waypoints = generate_coverage_waypoints(
        data, w, h, resolution=1.0, origin_x=0.0, origin_y=0.0,
        row_spacing_m=1.0, min_run_m=1.0, swept_mask=swept_mask,
    )
    assert waypoints == [(0.5, 0.5), (4.5, 0.5)]
