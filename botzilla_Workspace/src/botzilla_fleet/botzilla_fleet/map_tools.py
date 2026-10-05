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
