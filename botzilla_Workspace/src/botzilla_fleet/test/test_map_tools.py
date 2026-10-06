from botzilla_fleet.map_tools import clear_discs
import numpy as np


def test_clears_disc_and_nothing_else():
    g = np.full((40, 40), 100, np.int8)
    clear_discs(g, (0.0, 0.0), 0.05, [(1.0, 1.0)], 0.5)
    assert g[20, 20] == 0 and g[20, 29] == 0          # centre, 0.45 m away
    assert g[20, 31] == 100 and g[29, 29] == 100      # 0.55 m, 0.64 m away
    assert (g == 0).sum() < np.pi * 11 ** 2


def test_edge_and_off_grid_spots():
    g = np.full((20, 20), -1, np.int8)
    clear_discs(g, (0.0, 0.0), 0.05, [(0.0, 0.0), (50.0, 50.0)], 0.2)
    assert g[0, 0] == 0 and g[19, 19] == -1


def test_rotated_footprint_has_no_holes():
    import math
    from botzilla_fleet.map_tools import place, points_to_grid, rect_points
    fp = rect_points((-0.27, 0.41), (-0.265, 0.265), 0.025)
    data, ox, oy, w, h = points_to_grid(place(fp, (1.0, 2.0, math.radians(30))), 0.05)
    # Every cell whose centre is well inside the rotated rectangle must be marked.
    c, s = math.cos(math.radians(-30)), math.sin(math.radians(-30))
    for j in range(h):
        for i in range(w):
            x, y = ox + (i + 0.5) * 0.05 - 1.0, oy + (j + 0.5) * 0.05 - 2.0
            u, v = c * x - s * y, s * x + c * y
            if -0.22 < u < 0.36 and -0.215 < v < 0.215:
                assert data[j, i] == 100, (i, j)


def test_no_points_is_none():
    from botzilla_fleet.map_tools import points_to_grid
    assert points_to_grid([], 0.05) is None


def test_shapes_grid_exact_footprint_and_halo():
    import math
    from botzilla_fleet.map_tools import shapes_grid
    fp = ((-0.22, 0.36), (-0.215, 0.215))
    data, ox, oy, w, h = shapes_grid([((1.0, 1.0, 0.0), fp, 0.5, 45)], 0.05)

    def at(x, y):
        i, j = int((x - ox) / 0.05), int((y - oy) / 0.05)
        return data[j, i] if 0 <= i < w and 0 <= j < h else 0
    # Lethal cells are exactly the footprint: count matches its area within one cell row.
    lethal = (data == 100).sum() * 0.05 * 0.05
    assert abs(lethal - 0.58 * 0.43) < 0.06
    assert at(1.0, 1.0) == 100 and at(1.33, 1.19) == 100       # inside, near a corner
    assert at(1.0, 1.24) < 100 and at(1.0, 1.24) > 30          # just outside: strong halo
    assert at(1.0, 1.75) == 0                                   # beyond the 0.5 m halo
    # Rotation: 90 deg turns the long axis to y.
    data2, ox2, oy2, w2, h2 = shapes_grid([((0.0, 0.0, math.pi / 2), fp, 0.0, 0)], 0.05)
    assert (data2 == 100).any(axis=1).sum() > (data2 == 100).any(axis=0).sum()


def test_shapes_grid_empty():
    from botzilla_fleet.map_tools import shapes_grid
    assert shapes_grid([], 0.05) is None
