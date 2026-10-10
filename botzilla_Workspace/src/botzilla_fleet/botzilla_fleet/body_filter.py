"""
body_filter.py — remove the OTHER robot from a laser scan, given where it is.

ROS-free; tested in test/test_body_filter.py; used by fleet_scan_filter_node.

Each robot's LiDAR (and the depth camera's virtual scan) sees the other robot, and every
consumer then treats it as an anonymous obstacle: the costmaps mark it and inflate it by
0.45 m on top of the exact mark the fleet layer already draws, the marks linger after it
moves (camera clearing is not solved), and on the leader RTAB-Map maps it into /map —
run 17's map has the parked collector at its HOME, a blob the leader's global costmap
then planned around for the whole run. The fleet layer knows exactly where the other
robot is, so its returns are dropped here and the fleet layer becomes the one place it
appears: exact, current, uninflated. This is the standard way multi-robot systems handle
a known body (laser_filters' polygon/footprint filters do the same for a fixed frame).

Returns inside the other robot's shape grown by a margin are set to NaN, which every
consumer treats as "no return" (laser_filters does the same). The scan's timestamp is
untouched (docs/ghost_map_sensor_latency.md).
"""
import math

import numpy as np

NAN = float('nan')


def returns_xy(ranges, angle_min, angle_increment, laser_pose):
    """Map-frame (x, y) of every return, as two arrays (NaN/inf rays give non-finite)."""
    r = np.asarray(ranges, dtype=float)
    lx, ly, lyaw = laser_pose
    a = lyaw + angle_min + angle_increment * np.arange(len(r))
    return lx + r * np.cos(a), ly + r * np.sin(a)


def inside(xs, ys, body_pose, shape, margin):
    """Boolean mask: points inside shape at body_pose, grown by margin.

    shape: a circle radius, or a rectangle ((x0, x1), (y0, y1)) in the body's frame.
    """
    bx, by, byaw = body_pose
    dx, dy = xs - bx, ys - by
    if isinstance(shape, (int, float)):
        return np.hypot(dx, dy) <= shape + margin
    (x0, x1), (y0, y1) = shape
    c, s = math.cos(byaw), math.sin(byaw)
    u, v = c * dx + s * dy, -s * dx + c * dy
    return ((u >= x0 - margin) & (u <= x1 + margin)
            & (v >= y0 - margin) & (v <= y1 + margin))


def filter_ranges(ranges, angle_min, angle_increment, laser_pose, body_pose, shape,
                  margin):
    """Return (new ranges list, number removed): returns on the body set to NaN."""
    r = np.asarray(ranges, dtype=float)
    xs, ys = returns_xy(r, angle_min, angle_increment, laser_pose)
    hit = np.isfinite(r) & inside(xs, ys, body_pose, shape, margin)
    if not hit.any():
        return list(ranges), 0
    out = r.copy()
    out[hit] = NAN
    return out.tolist(), int(hit.sum())
