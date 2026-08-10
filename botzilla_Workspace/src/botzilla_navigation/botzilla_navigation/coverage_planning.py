"""
coverage_planning.py.

Pure, ROS-independent boustrophedon (lawnmower) coverage-sweep planning for Milestone 5b.
Deliberately has no rclpy/message imports so it can be unit-tested with plain Python lists
(see test/test_coverage_planning.py) and called directly from frontier_explorer_node.py
with an OccupancyGrid's raw fields — mirrors frontier_detection.py's own style.

Coverage-sweep waypoints only look at the known map (which rows have free space, and how
long each free run is) — they say nothing about whether a point is safe or reachable right
now. That is deliberately left to the caller: frontier_explorer_node.py runs every waypoint
through the exact same costmap-snap + reachability-check pipeline it already uses for
frontier targets (see that file's module docstring "Coverage sweep" section), so this
module doesn't duplicate any of that hardened, live-tested safety logic.
"""

from botzilla_navigation.frontier_detection import grid_to_world, OCCUPIED_THRESHOLD


def _is_free(value):
    return 0 <= value < OCCUPIED_THRESHOLD


def _find_free_runs(row_values, min_run_cells):
    """Contiguous runs of free cells in one map row, as (start_col, end_col) inclusive."""
    runs = []
    start = None
    for col, value in enumerate(row_values):
        if _is_free(value):
            if start is None:
                start = col
            continue
        if start is not None and col - start >= min_run_cells:
            runs.append((start, col - 1))
        start = None
    if start is not None and len(row_values) - start >= min_run_cells:
        runs.append((start, len(row_values) - 1))
    return runs


def generate_coverage_waypoints(
    data, width, height, resolution, origin_x, origin_y, row_spacing_m, min_run_m=0.3,
    swept_mask=None,
):
    """Boustrophedon sweep of every known free-space row, as world (x, y) waypoints.

    Steps through the map every row_spacing_m (clamped to at least one cell, so a spacing
    smaller than the map resolution still terminates and visits every row rather than
    looping forever on a zero cell-step), and for each sampled row finds contiguous
    free-cell runs at least min_run_m long. Each qualifying run contributes its two
    endpoints (not its midpoint) as waypoints, so driving consecutive waypoints traces the
    run rather than just visiting its centre. Rows alternate left-to-right / right-to-left
    (a snake), so the returned list is already in a sane drive order — the caller does not
    need to re-sort it.

    swept_mask, if given (see swept_mask.py), is a flat row-major boolean grid matching
    this map's dimensions. A run is dropped entirely if every cell in it is already
    marked swept — the camera has already covered that stretch, so driving through it
    again would waste time better spent on newly-mapped space. swept_mask=None preserves
    the original behavior (every qualifying run included) unchanged.

    Returns [] if no row has a qualifying run (e.g. nothing free has been mapped yet).
    """
    row_spacing_cells = max(1, round(row_spacing_m / resolution))
    min_run_cells = max(1, round(min_run_m / resolution))

    waypoints = []
    for pass_index, row in enumerate(range(0, height, row_spacing_cells)):
        row_values = data[row * width:(row + 1) * width]
        runs = _find_free_runs(row_values, min_run_cells)
        if swept_mask is not None:
            row_base = row * width
            runs = [
                (start_col, end_col) for (start_col, end_col) in runs
                if not all(
                    swept_mask[row_base + col] for col in range(start_col, end_col + 1)
                )
            ]
        left_to_right = pass_index % 2 == 0
        if not left_to_right:
            runs = list(reversed(runs))
        for start_col, end_col in runs:
            entry_col, exit_col = (
                (start_col, end_col) if left_to_right else (end_col, start_col)
            )
            waypoints.append(grid_to_world(row, entry_col, resolution, origin_x, origin_y))
            waypoints.append(grid_to_world(row, exit_col, resolution, origin_x, origin_y))

    return waypoints
