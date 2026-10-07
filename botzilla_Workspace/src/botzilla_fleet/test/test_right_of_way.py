import math

from botzilla_fleet.right_of_way import (
    EscapeLatch, footprint_distance, GIVE_WAY_COOLDOWN_S, GIVE_WAY_MAX_S, GiveWay,
    path_ahead,
)

RECT = ((-0.22, 0.36), (-0.215, 0.215))   # the collector's Nav2 footprint


def test_path_ahead_starts_at_robot_and_stops_at_length():
    path = [(0.05 * k, 0.0) for k in range(100)]          # 5 m straight line
    pts = path_ahead(path, (1.0, 0.1), length_m=1.5, step_m=0.15)
    assert abs(pts[0][0] - 1.0) < 1e-9
    assert pts[-1][0] <= 2.5 + 1e-9 and pts[-1][0] >= 2.35
    gaps = [b[0] - a[0] for a, b in zip(pts, pts[1:])]
    assert all(0.14 < g < 0.21 for g in gaps)


def test_path_ahead_ignores_stale_or_empty_plans():
    assert path_ahead([], (0, 0)) == []
    assert path_ahead([(5.0, 5.0), (5.1, 5.0)], (0.0, 0.0)) == []


def test_footprint_distance_rotated():
    pose = (1.0, 1.0, math.pi / 2)                        # facing +y: arms point up
    assert footprint_distance((1.0, 1.3), pose, RECT) == 0.0   # inside the arms
    assert abs(footprint_distance((1.0, 1.5), pose, RECT) - 0.14) < 1e-9


def test_escape_latch_hysteresis():
    latch = EscapeLatch()
    pose = (0.0, 0.0, 0.0)
    assert latch.update((0.0, 0.60), pose, RECT, 0.27) is True     # 0.385 m clear
    assert latch.update((0.0, 0.40), pose, RECT, 0.27) is False    # overlapping
    assert latch.update((0.0, 0.52), pose, RECT, 0.27) is False    # 0.305: not yet clear
    assert latch.update((0.0, 0.60), pose, RECT, 0.27) is True     # 0.385 >= 0.37


def test_give_way_pauses_for_a_carrying_collector_only():
    gw = GiveWay()
    assert gw.update(0.0, 'GOING', 0.8) is None
    assert gw.update(1.0, 'DELIVERING', 0.8) == 'pause'
    assert gw.update(2.0, 'DELIVERING', 1.4) is None              # inside the clear band
    assert gw.update(3.0, 'DELIVERING', 1.7) == 'resume'
    assert gw.update(4.0, 'DELIVERING', 0.8) is None              # cooldown
    assert gw.update(4.0 + GIVE_WAY_COOLDOWN_S, 'DELIVERING', 0.8) == 'pause'


def test_give_way_resumes_when_done_or_too_long():
    gw = GiveWay()
    gw.update(0.0, 'DELIVERING', 0.5)
    assert gw.update(1.0, 'IDLE', 0.5) == 'resume'
    gw = GiveWay()
    gw.update(0.0, 'DELIVERING', 0.5)
    assert gw.update(GIVE_WAY_MAX_S + 0.1, 'DELIVERING', 0.5) == 'resume'
    gw = GiveWay()
    gw.update(0.0, 'CAPTURING', 0.5)
    assert gw.update(1.0, 'DELIVERING', None) == 'resume'         # lost the collector
