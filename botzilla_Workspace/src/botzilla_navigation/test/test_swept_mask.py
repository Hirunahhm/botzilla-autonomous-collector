"""
test_swept_mask.py.

Pure pytest for botzilla_navigation.swept_mask — no ROS imports, runnable directly with
`python3 -m pytest` without sourcing a ROS environment.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from botzilla_navigation.swept_mask import (  # noqa: E402,I100
    build_coverage_grid_data,
    count_unswept_free,
    create_swept_mask,
    mark_swept_cells,
    mark_world_point_swept,
    resize_swept_mask,
    should_trigger_sweep,
)


def test_create_swept_mask_all_false():
    mask = create_swept_mask(5, 4)
    assert len(mask) == 20
    assert all(cell is False for cell in mask)


def test_mark_swept_cells_marks_cell_dead_ahead_within_range():
    width, height, resolution = 21, 21, 1.0
    mask = create_swept_mask(width, height)
    # Robot at the center of cell (row=10, col=10), facing +x (yaw=0).
    robot_x, robot_y = 10.5, 10.5
    marked = mark_swept_cells(
        mask, width, height, resolution, 0.0, 0.0, robot_x, robot_y, 0.0
    )
    assert marked > 0
    # Cell (row=10, col=11) is 1.0m straight ahead — within range and dead-center of FOV.
    assert mask[10 * width + 11] is True


def test_mark_swept_cells_respects_range_cap():
    width, height, resolution = 21, 21, 1.0
    mask = create_swept_mask(width, height)
    robot_x, robot_y = 10.5, 10.5
    mark_swept_cells(mask, width, height, resolution, 0.0, 0.0, robot_x, robot_y, 0.0)
    # Cell (row=10, col=12) is 2.0m straight ahead — beyond CAMERA_MARK_RANGE_M (1.0m).
    assert mask[10 * width + 12] is False


def test_mark_swept_cells_respects_fov_cap():
    width, height, resolution = 21, 21, 1.0
    mask = create_swept_mask(width, height)
    robot_x, robot_y = 10.5, 10.5
    mark_swept_cells(mask, width, height, resolution, 0.0, 0.0, robot_x, robot_y, 0.0)
    # Cell (row=11, col=10) is 1.0m directly to the side (90 deg off heading) — within
    # range but well outside +/- CAMERA_HALF_FOV_RAD (~28.5 deg).
    assert mask[11 * width + 10] is False


def test_mark_swept_cells_returns_newly_marked_count():
    width, height, resolution = 21, 21, 1.0
    mask = create_swept_mask(width, height)
    marked = mark_swept_cells(mask, width, height, resolution, 0.0, 0.0, 10.5, 10.5, 0.0)
    assert marked == sum(1 for cell in mask if cell)
    # A second call over the same pose marks nothing new — every qualifying cell is
    # already True.
    marked_again = mark_swept_cells(mask, width, height, resolution, 0.0, 0.0, 10.5, 10.5, 0.0)
    assert marked_again == 0


def test_mark_swept_cells_heading_rotates_the_wedge():
    # Same geometry as the "dead ahead" test, but facing +y (yaw=pi/2) instead of +x —
    # the cell that was dead ahead before should now be a 90 deg side cell (unmarked),
    # and the cell above the robot should be the one marked instead.
    width, height, resolution = 21, 21, 1.0
    mask = create_swept_mask(width, height)
    mark_swept_cells(mask, width, height, resolution, 0.0, 0.0, 10.5, 10.5, math.pi / 2.0)
    assert mask[10 * width + 11] is False  # +x cell, now to the side
    assert mask[11 * width + 10] is True   # +y cell, now dead ahead


def test_mark_world_point_swept_marks_exactly_one_cell():
    width, height, resolution = 5, 5, 1.0
    mask = create_swept_mask(width, height)
    mark_world_point_swept(mask, width, height, resolution, 0.0, 0.0, 2.5, 3.5)
    assert mask[3 * width + 2] is True
    assert sum(1 for cell in mask if cell) == 1


def test_mark_world_point_swept_out_of_bounds_is_a_no_op():
    width, height, resolution = 5, 5, 1.0
    mask = create_swept_mask(width, height)
    mark_world_point_swept(mask, width, height, resolution, 0.0, 0.0, 500.0, 500.0)
    assert sum(1 for cell in mask if cell) == 0


def test_resize_swept_mask_relocates_cell_across_asymmetric_origin_shift():
    old_width, old_height, resolution = 3, 3, 1.0
    old_origin_x, old_origin_y = 0.0, 0.0
    old_mask = create_swept_mask(old_width, old_height)
    old_mask[1 * old_width + 1] = True  # world center (1.5, 1.5)

    new_width, new_height = 5, 5
    new_origin_x, new_origin_y = -1.0, -2.0  # shifted by different amounts per axis

    new_mask = resize_swept_mask(
        old_mask, old_width, old_height, resolution, old_origin_x, old_origin_y,
        new_width, new_height, new_origin_x, new_origin_y,
    )
    # world (1.5, 1.5) in the new grid: col = int((1.5 - -1.0)/1.0) = 2,
    # row = int((1.5 - -2.0)/1.0) = 3
    assert new_mask[3 * new_width + 2] is True
    assert sum(1 for cell in new_mask if cell) == 1


def test_resize_swept_mask_no_change_is_identity():
    width, height, resolution = 4, 4, 1.0
    mask = create_swept_mask(width, height)
    mask[2 * width + 1] = True
    resized = resize_swept_mask(
        mask, width, height, resolution, 0.0, 0.0, width, height, 0.0, 0.0
    )
    assert resized == mask


def test_count_unswept_free():
    # width=3, height=2. Row 0: free(0), occupied(100), unknown(-1).
    # Row 1: free(50), free(0), occupied(100).
    data = [0, 100, -1, 50, 0, 100]
    mask = [True, False, False, False, True, False]
    total_free, unswept_free = count_unswept_free(data, mask, 3, 2)
    assert total_free == 3       # indices 0, 3, 4
    assert unswept_free == 1     # index 3 only


def test_should_trigger_sweep_threshold():
    assert should_trigger_sweep(100, 20, 0.15) is True
    assert should_trigger_sweep(100, 15, 0.15) is False  # exactly at threshold, not over
    assert should_trigger_sweep(100, 10, 0.15) is False
    assert should_trigger_sweep(0, 0, 0.15) is False


def test_build_coverage_grid_data():
    # Same layout as test_count_unswept_free: width=3, height=2.
    # Row 0: free(0), occupied(100), unknown(-1). Row 1: free(50), free(0), occupied(100).
    data = [0, 100, -1, 50, 0, 100]
    mask = [True, False, False, False, True, False]
    grid = build_coverage_grid_data(data, mask, 3, 2)
    assert grid == [
        0, -1, -1,    # swept-free -> 0, occupied -> -1, unknown -> -1
        100, 0, -1,   # un-swept-free -> 100, swept-free -> 0, occupied -> -1
    ]
