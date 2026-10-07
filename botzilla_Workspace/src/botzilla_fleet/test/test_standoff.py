import math

from botzilla_fleet.standoff import choose_standoff, fallback_standoff
import numpy as np

RES = 0.05


def room(w_m=4.0, h_m=4.0):
    w, h = int(w_m / RES), int(h_m / RES)
    a = np.zeros((h, w), dtype=np.int16)
    a[0, :] = a[-1, :] = a[:, 0] = a[:, -1] = 100
    return a, (0.0, 0.0, RES, w, h)


def test_faces_cube_from_robot_side():
    data, info = room()
    sx, sy, yaw = choose_standoff(data, info, (2.0, 2.0), (0.5, 2.0))
    assert abs(math.hypot(sx - 2.0, sy - 2.0) - 0.75) < 1e-6
    assert sx < 2.0                                    # robot's side
    assert abs(math.atan2(2.0 - sy, 2.0 - sx) - yaw) < 1e-6


def test_avoids_wall_and_unknown():
    data, info = room()
    data[:, :30] = -1                                  # left 1.5 m unknown
    sx, sy, _ = choose_standoff(data, info, (2.0, 2.0), (0.5, 2.0))
    assert sx >= 1.5 + 0.25 - 1e-6


def test_blocked_line_of_sight_rejected():
    data, info = room()
    data[30:50, 33] = 100                              # wall segment at x=1.65
    sx, sy, _ = choose_standoff(data, info, (2.0, 2.0), (0.5, 2.0))
    # A straight-left standoff would look through the wall; it must go round.
    assert not (sx < 1.65 and 1.5 < sy < 2.5)


def test_none_when_boxed_in():
    data, info = room()
    data[:, :] = 100
    assert choose_standoff(data, info, (2.0, 2.0), (0.5, 2.0)) is None


def test_fallback_on_line():
    x, y, yaw = fallback_standoff((3.0, 0.0), (0.0, 0.0))
    assert abs(x - 2.25) < 1e-9 and abs(y) < 1e-9 and abs(yaw) < 1e-9
    assert fallback_standoff((0.5, 0.0), (0.0, 0.0))[:2] == (0.0, 0.0)


def test_blocked_callback_moves_the_standoff():
    data, info = room()
    free = choose_standoff(data, info, (2.0, 2.0), (0.5, 2.0))
    # Pretend the other robot stands on the preferred spot.
    moved = choose_standoff(data, info, (2.0, 2.0), (0.5, 2.0),
                            blocked=lambda x, y: math.hypot(x - free[0], y - free[1]) < 0.3)
    assert moved is not None
    assert math.hypot(moved[0] - free[0], moved[1] - free[1]) >= 0.3


def test_clear_spot_is_away_from_the_other_robot():
    from botzilla_fleet.standoff import choose_clear_spot
    data, info = room()
    x, y, yaw = choose_clear_spot(data, info, (2.0, 2.0), (2.6, 2.0), min_from_other=1.4)
    assert math.hypot(x - 2.6, y - 2.0) >= 1.4
    assert x < 2.0                                   # on the far side from the other robot
    assert abs(math.atan2(math.sin(yaw - math.pi), math.cos(yaw - math.pi))) < 0.6


def test_clear_spot_respects_walls_and_blocked():
    from botzilla_fleet.standoff import choose_clear_spot
    data, info = room()
    spot = choose_clear_spot(data, info, (0.5, 0.5), (1.1, 0.5), min_from_other=1.4,
                             blocked=lambda x, y: y > 1.0)
    assert spot is None or (spot[1] <= 1.0 and math.hypot(spot[0] - 1.1, spot[1] - 0.5) >= 1.4)


def test_clear_spot_keeps_off_the_leaders_route():
    # run 21: the leader at (2.0, 2.0) heading west (-x) through the collector at (1.5, 2.0);
    # "far side of the leader" was straight ahead of it on its route
    from botzilla_fleet.standoff import choose_clear_spot
    data, info = room()
    route = [(2.0 - 0.15 * k, 2.0) for k in range(17)]          # 2.5 m west
    x, y, _ = choose_clear_spot(data, info, (1.5, 2.0), (2.0, 2.0), min_from_other=1.4,
                                avoid_points=route)
    assert min(math.hypot(x - px, y - py) for px, py in route) >= 0.7
    assert math.hypot(x - 2.0, y - 2.0) >= 1.4


def test_clear_spot_never_passes_through_the_leader():
    from botzilla_fleet.standoff import choose_clear_spot
    data, info = room()
    x, y, _ = choose_clear_spot(data, info, (2.0, 2.0), (2.3, 2.0), min_from_other=1.4)
    assert x < 2.3                                       # not across the leader at (2.3, 2)


def test_clear_spot_ignores_route_rather_than_staying_put():
    from botzilla_fleet.standoff import choose_clear_spot
    data, info = room()
    everywhere = [(0.1 * i, 0.1 * j) for i in range(41) for j in range(41)]
    assert choose_clear_spot(data, info, (2.0, 2.0), (2.6, 2.0), min_from_other=1.4,
                             avoid_points=everywhere) is not None


def test_clear_spot_keeps_off_the_near_route_when_the_whole_route_is_impossible():
    from botzilla_fleet.standoff import choose_clear_spot
    data, info = room()
    near = [(2.0 - 0.15 * k, 2.0) for k in range(10)]           # next 1.5 m: west
    far = [(0.1 * i, 0.1 * j) for i in range(41) for j in range(41)]   # then everywhere
    x, y, _ = choose_clear_spot(data, info, (1.5, 2.0), (2.0, 2.0), min_from_other=1.4,
                                avoid_points=near + far)
    assert min(math.hypot(x - px, y - py) for px, py in near) >= 0.7
