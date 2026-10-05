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
