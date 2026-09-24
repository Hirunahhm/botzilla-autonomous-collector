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

Two further limits, both of which made the old mask overstate coverage (see
research_discussion.md §1.3):

  * Near-field blind ring. The camera sits 0.19 m up with a ~43 deg vertical FOV, so
    with a level camera the floor only enters the image ~0.48 m ahead. Floor closer
    than CAMERA_MIN_RANGE_M is never seen from that pose — the wedge is really an
    annulus sector, and a spin in place covers a ring, not a disc.
  * Occlusion. A cell only counts if the straight line from the robot to it crosses no
    occupied map cell (has_line_of_sight). Floor behind a box or a chair leg inside the
    1 m cone is not inspected. Star-Searcher and HEATS both require "not occluded".
    Unknown cells do not block: they are mostly floor the LiDAR has not reached yet,
    and blocking on them would make the mask lag the map for no benefit.
"""

import math

from botzilla_navigation.frontier_detection import (
    grid_to_world,
    OCCUPIED_THRESHOLD,
    world_to_grid,
)

# Defaults only. frontier_explorer_node declares both as ROS parameters (defaulting to
# exactly these values) and passes them in, so an experiment can vary the camera envelope
# without editing this file — and so the sensor-characterization pass can set them from
# measurement rather than from the single-frame estimate in the module docstring. Kept as
# module constants because they are also the honest default for any direct caller
# (test/test_swept_mask.py included).
CAMERA_HALF_FOV_RAD = 0.497  # ~28.5 deg, half of the Kinect's ~57 deg horizontal FOV
# Effective YOLO cube-detection range — see module docstring. Must stay equal to
# executor_node.CUBE_MAX_RANGE_M, which gates which detections the mission actually acts
# on; that constant's comment carries the reasoning and the regression to watch for.
CAMERA_MARK_RANGE_M = 1.0
# Closest floor the camera can see — see module docstring. 0.19 m mount height over
# tan(21.5 deg), half the Kinect's ~43 deg vertical FOV, assuming zero tilt. Re-derive it
# once the tilt is measured: a camera pitched down a few degrees pulls this in a lot.
CAMERA_MIN_RANGE_M = 0.48
# has_line_of_sight ignores occupied cells this close to the target. The target cell
# itself (a cube, or a free cell right against a wall) must not occlude itself, and a
# half-cell of map noise on the far side of a free cell should not hide it either.
LOS_TARGET_CLEARANCE_M = 0.1


def is_in_frustum(
    robot_x, robot_y, robot_yaw, point_x, point_y,
    half_fov_rad=CAMERA_HALF_FOV_RAD, mark_range_m=CAMERA_MARK_RANGE_M,
    min_range_m=CAMERA_MIN_RANGE_M,
):
    """Whether world point (point_x, point_y) is inside the camera's detection cone.

    The single definition of "the camera could have seen this spot": between min_range_m
    and mark_range_m from the robot AND within +/- half_fov_rad of its heading.
    mark_swept_cells uses it per grid cell to build the coverage map, and
    mission_metrics_node uses it per ground-truth cube to timestamp first inspection.
    Those two must agree exactly — a coverage figure and a per-cube inspection time
    computed from different geometry would not be comparable — so the test lives here
    once rather than being written twice. Occlusion is a separate test
    (has_line_of_sight) because it needs the map, which not every caller has.
    """
    dx, dy = point_x - robot_x, point_y - robot_y
    dist = math.hypot(dx, dy)
    if dist > mark_range_m or dist < min_range_m:
        return False
    relative_angle = math.atan2(dy, dx) - robot_yaw
    relative_angle = math.atan2(math.sin(relative_angle), math.cos(relative_angle))
    return abs(relative_angle) <= half_fov_rad


def has_line_of_sight(
    data, width, height, resolution, origin_x, origin_y, x0, y0, x1, y1,
    target_clearance_m=LOS_TARGET_CLEARANCE_M,
):
    """Whether the segment (x0, y0) -> (x1, y1) crosses no occupied cell of the map.

    Samples the segment every half cell, skipping the start cell (the robot) and the
    last target_clearance_m before the target (see LOS_TARGET_CLEARANCE_M). Occupied
    means value >= OCCUPIED_THRESHOLD, the same cut every other module uses; unknown
    (-1) does not block — see the module docstring. Samples outside the grid do not
    block either: there is nothing known there to block with.
    """
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    if length <= 0.0:
        return True
    start_row, start_col = world_to_grid(x0, y0, resolution, origin_x, origin_y)
    step = resolution * 0.5
    samples = int(math.ceil(length / step))
    stop = length - target_clearance_m
    for k in range(1, samples + 1):
        travelled = min(k * step, length)
        if travelled > stop:
            break
        t = travelled / length
        row, col = world_to_grid(x0 + dx * t, y0 + dy * t, resolution, origin_x, origin_y)
        if (row, col) == (start_row, start_col):
            continue
        if not (0 <= row < height and 0 <= col < width):
            continue
        if data[row * width + col] >= OCCUPIED_THRESHOLD:
            return False
    return True


def create_swept_mask(width, height):
    """Return a flat row-major boolean grid, same dimensions as an OccupancyGrid, all False."""
    return [False] * (width * height)


def mark_swept_cells(
    mask, width, height, resolution, origin_x, origin_y, robot_x, robot_y, robot_yaw,
    half_fov_rad=CAMERA_HALF_FOV_RAD, mark_range_m=CAMERA_MARK_RANGE_M,
    min_range_m=CAMERA_MIN_RANGE_M, occupancy=None,
):
    """Mark cells within the camera's forward wedge of the robot's current pose as swept.

    Scans a mark_range_m bounding box of grid cells around the robot (in cells, not world
    units, so the scan cost is independent of map size) and marks each cell True iff it is
    inside is_in_frustum's annulus sector AND, when occupancy (the map's flat data, same
    dimensions as mask) is given, visible along an unobstructed line (has_line_of_sight).
    Mutates mask in place. Returns the count of cells newly marked this call (for logging).

    The line-of-sight test runs only on cells that pass the cheap geometric test and are
    not already marked, so it costs a few thousand samples per call at 5 cm cells.

    The camera parameters default to the module constants so direct callers and the unit
    tests need not pass them; frontier_explorer_node passes its ROS parameters.
    occupancy=None skips the occlusion test (the previous behaviour).
    """
    range_cells = max(1, math.ceil(mark_range_m / resolution))
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
            if not is_in_frustum(
                robot_x, robot_y, robot_yaw, cell_x, cell_y,
                half_fov_rad, mark_range_m, min_range_m,
            ):
                continue
            if occupancy is not None and not has_line_of_sight(
                occupancy, width, height, resolution, origin_x, origin_y,
                robot_x, robot_y, cell_x, cell_y,
            ):
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


def build_coverage_grid_data(data, mask, width, height):
    """Build OccupancyGrid.data values for visualizing swept coverage in RViz.

    -1 (unknown) for any cell that isn't free in the base map, so walls and unknown
    space pass through untouched rather than being overdrawn. 0 (free) for free cells
    the camera has swept — renders white in RViz's default Map color scheme. 100
    (occupied) for free cells still un-swept — renders black, making un-covered floor
    visually obvious layered over the base /map.
    """
    out = [-1] * (width * height)
    for i in range(width * height):
        if 0 <= data[i] < OCCUPIED_THRESHOLD:
            out[i] = 0 if mask[i] else 100
    return out


def _summed_area(values, width, height):
    """Summed-area table with a zero border: S[(r+1)*(w+1)+(c+1)] = sum of rows<=r, cols<=c."""
    stride = width + 1
    table = [0] * (stride * (height + 1))
    for row in range(height):
        running = 0
        src = row * width
        dst = (row + 1) * stride
        above = row * stride
        for col in range(width):
            running += values[src + col]
            table[dst + col + 1] = table[above + col + 1] + running
    return table


def build_coverage_cost_data(data, mask, width, height, resolution, radius_m, max_cost):
    """Build OccupancyGrid.data for the coverage cost layer (botzilla_coverage_layer).

    Each known-free cell gets round(max_cost * f), where f is the fraction of known-free
    cells within a (2r+1)-cell square window around it that the camera has already
    swept. Everything else (walls, unknown) is 0 — never -1, because the layer reads
    this as a cost and "unknown" must not become a penalty or a blocker there.

    Smoothed rather than per-cell on purpose. Driving along a line sweeps a band
    ~2*radius wide, so a single un-swept cell next to swept ones is not worth routing
    through: the planner would thread thin un-swept slivers that add almost nothing.
    Averaging over the swath half-width makes the cost track "how much new floor would
    the camera see from here", which is what the planner should trade distance for.
    Walls are excluded from the denominator so the cost next to a wall is not diluted
    toward zero just because half the window is wall.

    max_cost <= 0 returns all zeros without doing the work — the layer then has no
    effect, which is the control arm of the experiment.
    """
    size = width * height
    if max_cost <= 0:
        return [0] * size
    free = [1 if 0 <= data[i] < OCCUPIED_THRESHOLD else 0 for i in range(size)]
    swept = [1 if free[i] and mask[i] else 0 for i in range(size)]
    free_sat = _summed_area(free, width, height)
    swept_sat = _summed_area(swept, width, height)
    stride = width + 1
    r = max(0, int(round(radius_m / resolution)))

    out = [0] * size
    for row in range(height):
        r0 = max(0, row - r)
        r1 = min(height, row + r + 1)
        top = r0 * stride
        bottom = r1 * stride
        base = row * width
        for col in range(width):
            i = base + col
            if not free[i]:
                continue
            c0 = max(0, col - r)
            c1 = min(width, col + r + 1)
            n_free = (free_sat[bottom + c1] - free_sat[top + c1]
                      - free_sat[bottom + c0] + free_sat[top + c0])
            if n_free <= 0:
                continue
            n_swept = (swept_sat[bottom + c1] - swept_sat[top + c1]
                       - swept_sat[bottom + c0] + swept_sat[top + c0])
            out[i] = int(round(max_cost * n_swept / n_free))
    return out


def unmark_discs(mask, width, height, resolution, origin_x, origin_y, centers, radius_m):
    """Return a COPY of mask with every cell within radius_m of any center set to False.

    Used to build the PLANNING view of the swept mask: floor where a cube was seen but
    not collected is treated as un-inspected so the sweep and the coverage cost both
    steer the robot back to it. The original mask is left untouched because it is also
    the coverage METRIC — the camera really did cover that floor, and un-marking it
    would under-report inspection for whichever run happened to abort more chases.
    """
    out = list(mask)
    if not centers:
        return out
    r_cells = max(0, math.ceil(radius_m / resolution))
    r_sq = radius_m * radius_m
    for cx, cy in centers:
        center_row, center_col = world_to_grid(cx, cy, resolution, origin_x, origin_y)
        for row in range(center_row - r_cells, center_row + r_cells + 1):
            if not (0 <= row < height):
                continue
            for col in range(center_col - r_cells, center_col + r_cells + 1):
                if not (0 <= col < width):
                    continue
                x, y = grid_to_world(row, col, resolution, origin_x, origin_y)
                if (x - cx) ** 2 + (y - cy) ** 2 <= r_sq:
                    out[row * width + col] = False
    return out
