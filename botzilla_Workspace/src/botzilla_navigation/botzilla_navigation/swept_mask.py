"""
swept_mask.py.

Pure, ROS-independent tracking of which map cells the Kinect's camera has actually swept
past — as opposed to merely mapped by LiDAR. Mirrors frontier_detection.py's and
coverage_planning.py's style exactly: no rclpy/message imports, flat row-major
OccupancyGrid-shaped input, unit-tested directly (see test/test_swept_mask.py). Reuses
grid_to_world/world_to_grid/OCCUPIED_THRESHOLD from frontier_detection.py rather than
redefining the same coordinate math a third time.

Used by frontier_explorer_node.py (see that file's module docstring, "Coverage sweep"
section) to decide WHEN to interrupt frontier exploration for a coverage sweep (once
un-swept known-free area exceeds a fraction of total known free area) and WHICH cells a
sweep still needs to visit (coverage_planning.generate_coverage_waypoints skips runs that
are already fully swept).

Camera geometry: the Kinect is forward-facing, not 360 degrees, and YOLO's cube-detection
confidence drops off sharply past ~1m (hardware measurements: 0.797 confidence at ~1.1m
vs 0.138 for false positives further out). So a cell only counts as "swept" if it fell
within a forward wedge (+/- half the horizontal FOV) AND within that 1m range at some
point the robot passed it — marking further out would overstate what the detector could
actually have seen there.
"""

import math

from botzilla_navigation.frontier_detection import (
    grid_to_world,
    OCCUPIED_THRESHOLD,
    world_to_grid,
)

CAMERA_HALF_FOV_RAD = 0.497  # ~28.5 deg, half of the Kinect's ~57 deg horizontal FOV
CAMERA_MARK_RANGE_M = 1.0    # effective YOLO cube-detection range — see module docstring


def create_swept_mask(width, height):
    """Return a flat row-major boolean grid, same dimensions as an OccupancyGrid, all False."""
    return [False] * (width * height)


def mark_swept_cells(
    mask, width, height, resolution, origin_x, origin_y, robot_x, robot_y, robot_yaw,
):
    """Mark cells within the camera's forward wedge of the robot's current pose as swept.

    Scans a CAMERA_MARK_RANGE_M bounding box of grid cells around the robot (in cells,
    not world units, so the scan cost is independent of map size) and marks each cell
    True iff it is within CAMERA_MARK_RANGE_M of the robot AND within +/-
    CAMERA_HALF_FOV_RAD of the robot's heading. Mutates mask in place. Returns the count
    of cells newly marked this call (for logging).
    """
    range_cells = max(1, math.ceil(CAMERA_MARK_RANGE_M / resolution))
    center_row, center_col = world_to_grid(robot_x, robot_y, resolution, origin_x, origin_y)

    newly_marked = 0
    for dr in range(-range_cells, range_cells + 1):
        row = center_row + dr
        if not (0 <= row < height):
            continue
        row_base = row * width
        for dc in range(-range_cells, range_cells + 1):
            col = center_col + dc
            if not (0 <= col < width):
                continue
            i = row_base + col
            if mask[i]:
                continue
            cell_x, cell_y = grid_to_world(row, col, resolution, origin_x, origin_y)
            dx, dy = cell_x - robot_x, cell_y - robot_y
            if math.hypot(dx, dy) > CAMERA_MARK_RANGE_M:
                continue
            relative_angle = math.atan2(dy, dx) - robot_yaw
            relative_angle = math.atan2(math.sin(relative_angle), math.cos(relative_angle))
            if abs(relative_angle) > CAMERA_HALF_FOV_RAD:
                continue
            mask[i] = True
            newly_marked += 1

    return newly_marked


def mark_world_point_swept(mask, width, height, resolution, origin_x, origin_y, x, y):
    """Mark the single cell containing world point (x, y) as swept.

    Used when a coverage-sweep waypoint turns out unreachable (nook behind inflation, gap
    narrower than the footprint): the camera never actually saw it, but leaving it
    permanently un-swept would block coverage completion forever over one un-drivable
    pocket — see frontier_explorer_node.py's _dispatch_next_sweep_waypoint.
    """
    row, col = world_to_grid(x, y, resolution, origin_x, origin_y)
    if 0 <= row < height and 0 <= col < width:
        mask[row * width + col] = True


def resize_swept_mask(
    old_mask, old_width, old_height, resolution, old_origin_x, old_origin_y,
    new_width, new_height, new_origin_x, new_origin_y,
):
    """Re-map a swept mask onto a new grid whose dimensions and/or origin have changed.

    RTAB-Map's /map can grow on any edge as SLAM discovers more space, shifting its
    origin in x and/or y — not just extending outward from a fixed corner. Each True
    cell in the old mask is round-tripped through world coordinates (old grid -> world
    via grid_to_world, world -> new grid via world_to_grid) rather than having its
    row/col offset hand-derived, since a hand-rolled offset with the wrong sign would
    silently corrupt the mask with no crash to signal it.
    """
    new_mask = create_swept_mask(new_width, new_height)
    for row in range(old_height):
        row_base = row * old_width
        for col in range(old_width):
            if not old_mask[row_base + col]:
                continue
            world_x, world_y = grid_to_world(row, col, resolution, old_origin_x, old_origin_y)
            new_row, new_col = world_to_grid(
                world_x, world_y, resolution, new_origin_x, new_origin_y
            )
            if 0 <= new_row < new_height and 0 <= new_col < new_width:
                new_mask[new_row * new_width + new_col] = True
    return new_mask


def count_unswept_free(data, mask, width, height):
    """Return (total_free_cells, unswept_free_cells) over the map's known-free cells."""
    total_free = 0
    unswept_free = 0
    for i in range(width * height):
        if 0 <= data[i] < OCCUPIED_THRESHOLD:
            total_free += 1
            if not mask[i]:
                unswept_free += 1
    return total_free, unswept_free


def should_trigger_sweep(total_free, unswept_free, sweep_fraction):
    """Whether un-swept free area exceeds sweep_fraction of total known free area."""
    return total_free > 0 and (unswept_free / total_free) > sweep_fraction
