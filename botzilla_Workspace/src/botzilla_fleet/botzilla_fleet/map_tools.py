"""map_tools.py — small ROS-free occupancy-grid helpers for botzilla_fleet."""
import math

import numpy as np


def clear_discs(grid, origin_xy, resolution, spots, radius_m):
    """Set every cell within radius_m of each (x, y) spot to free (0), in place.

    grid: 2-D array (rows = y) as in nav_msgs/OccupancyGrid; spots off the grid are
    skipped, and discs at its edge are clipped.
    """
    ox, oy = origin_xy
    r = int(math.ceil(radius_m / resolution))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    disk = (xx * xx + yy * yy) * resolution * resolution <= radius_m ** 2
    h, w = grid.shape
    for x, y in spots:
        ci, cj = int(math.floor((x - ox) / resolution)), int(math.floor((y - oy) / resolution))
        j0, j1 = max(cj - r, 0), min(cj + r + 1, h)
        i0, i1 = max(ci - r, 0), min(ci + r + 1, w)
        if j0 >= j1 or i0 >= i1:
            continue
        sub = disk[j0 - (cj - r):j1 - (cj - r), i0 - (ci - r):i1 - (ci - r)]
        grid[j0:j1, i0:i1][sub] = 0
    return grid


def rect_points(x_range, y_range, step):
    """Points filling an axis-aligned rectangle (robot frame) at the given spacing."""
    xs = np.arange(x_range[0], x_range[1] + 1e-9, step)
    ys = np.arange(y_range[0], y_range[1] + 1e-9, step)
    gx, gy = np.meshgrid(xs, ys)
    return np.stack([gx.ravel(), gy.ravel()], axis=1)


def place(points, pose):
    """Robot-frame points moved to pose (x, y, yaw) in the map."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * points[:, 0] - s * points[:, 1],
                     y + s * points[:, 0] + c * points[:, 1]], axis=1)


def points_to_grid(points, resolution):
    """Rasterise map-frame points into the smallest grid covering them.

    Returns (data, origin_x, origin_y, width, height) with data an int8 array (rows = y)
    holding 100 on every cell a point falls in and 0 elsewhere, origin snapped to a
    multiple of resolution; or None when there are no points.
    """
    if points is None or len(points) == 0:
        return None
    pts = np.asarray(points, dtype=float)
    ox = math.floor(pts[:, 0].min() / resolution) * resolution
    oy = math.floor(pts[:, 1].min() / resolution) * resolution
    ix = np.floor((pts[:, 0] - ox) / resolution + 1e-9).astype(int)
    iy = np.floor((pts[:, 1] - oy) / resolution + 1e-9).astype(int)
    w, h = int(ix.max()) + 1, int(iy.max()) + 1
    data = np.zeros((h, w), dtype=np.int8)
    data[iy, ix] = 100
    return data, ox, oy, w, h
