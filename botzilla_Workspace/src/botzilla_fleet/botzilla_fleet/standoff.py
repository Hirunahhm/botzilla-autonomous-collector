"""
standoff.py — where the collector should stop to look at an assigned cube.

ROS-free; tested in test/test_standoff.py.

The leader's cube estimate is a point on the floor, and Nav2 cannot drive the robot onto
it (the cube is an obstacle in the leader's map often enough, and the camera cannot see
anything closer than ~0.5 m anyway — executor_node's depth blind spot). So the collector
drives to a standoff pose STANDOFF_M from the cube, facing it, where the cube sits in
the middle of the camera's detection band (0.48-1.0 m), and hands over to the existing
TARGETING -> APPROACHING -> CAPTURING chain.

Candidate poses are on circles around the cube; each must have CLEARANCE_M of known-free
floor around it and an unobstructed line to the cube. The one nearest the robot wins,
which approaches from the robot's side and keeps the detour short.
"""
import math

import numpy as np

STANDOFF_DISTANCES_M = (0.75, 0.9, 0.6)
CLEARANCE_M = 0.25
N_ANGLES = 24
FREE_MAX = 50          # occupancy values 0..FREE_MAX are free; -1 is unknown


def _cell(grid_info, x, y):
    ox, oy, res, w, h = grid_info
    i = int(math.floor((x - ox) / res))
    j = int(math.floor((y - oy) / res))
    if 0 <= i < w and 0 <= j < h:
        return j, i
    return None


def _clear(data, grid_info, x, y, radius):
    ox, oy, res, w, h = grid_info
    c = _cell(grid_info, x, y)
    if c is None:
        return False
    r = int(math.ceil(radius / res))
    j, i = c
    if j - r < 0 or i - r < 0 or j + r >= h or i + r >= w:
        return False
    patch = data[j - r:j + r + 1, i - r:i + r + 1]
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    disk = (xx * xx + yy * yy) <= r * r
    vals = patch[disk]
    return bool(np.all((vals >= 0) & (vals <= FREE_MAX)))


def _line_open(data, grid_info, x0, y0, x1, y1, stop_short_m):
    res = grid_info[2]
    d = math.hypot(x1 - x0, y1 - y0)
    n = max(1, int((d - stop_short_m) / (res / 2)))
    for k in range(n + 1):
        f = k * (d - stop_short_m) / d / n if d > 0 else 0.0
        c = _cell(grid_info, x0 + f * (x1 - x0), y0 + f * (y1 - y0))
        if c is None or data[c] > FREE_MAX:
            return False
    return True


def choose_standoff(data, grid_info, cube_xy, robot_xy,
                    distances=STANDOFF_DISTANCES_M, clearance_m=CLEARANCE_M,
                    n_angles=N_ANGLES, blocked=None):
    """Return (x, y, yaw) facing the cube, or None if no candidate is clear.

    data: 2-D int array (rows = y) of occupancy values, as in nav_msgs/OccupancyGrid.
    grid_info: (origin_x, origin_y, resolution, width, height).
    blocked: optional (x, y) -> bool for what the static map cannot know — the
    collector passes its live costmap, which holds the other robot (on 2026-10-06 the
    chosen standoff was inside the parked leader and Nav2 refused it twice).
    """
    cx, cy = cube_xy
    rx, ry = robot_xy
    for dist in distances:
        best = None
        for k in range(n_angles):
            a = 2 * math.pi * k / n_angles
            x, y = cx + dist * math.cos(a), cy + dist * math.sin(a)
            if not _clear(data, grid_info, x, y, clearance_m):
                continue
            if blocked is not None and blocked(x, y):
                continue
            # The cube's own cell may be marked occupied; stop the check short of it.
            if not _line_open(data, grid_info, x, y, cx, cy, stop_short_m=0.15):
                continue
            cost = math.hypot(x - rx, y - ry)
            if best is None or cost < best[0]:
                best = (cost, x, y, math.atan2(cy - y, cx - x))
        if best is not None:
            return best[1:]
    return None


def fallback_standoff(cube_xy, robot_xy, dist=STANDOFF_DISTANCES_M[0]):
    """No map: stop dist short of the cube on the straight line from the robot."""
    cx, cy = cube_xy
    rx, ry = robot_xy
    d = math.hypot(cx - rx, cy - ry)
    yaw = math.atan2(cy - ry, cx - rx)
    if d <= dist:
        return rx, ry, yaw
    return cx - dist * math.cos(yaw), cy - dist * math.sin(yaw), yaw
