import math

from botzilla_fleet.body_filter import filter_ranges

COLLECTOR = ((-0.17, 0.31), (-0.165, 0.165))


def _scan(n=360, r=3.0):
    return [r] * n, -math.pi, 2 * math.pi / n


def test_circle_body_removed_wall_kept():
    ranges, a0, inc = _scan()
    # a body 1 m straight ahead of the laser: rays near 0 rad hit it at ~0.83 m
    for k in range(len(ranges)):
        a = a0 + inc * k
        if abs(a) < math.radians(9):
            ranges[k] = 0.83
    out, n = filter_ranges(ranges, a0, inc, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.17, 0.10)
    assert n > 0
    assert all(math.isnan(out[k]) for k in range(len(out))
               if abs(a0 + inc * k) < math.radians(9))
    assert all(v == 3.0 for k, v in enumerate(out) if abs(a0 + inc * k) > math.radians(20))


def test_rotated_rectangle_and_laser_pose():
    # laser at (2, 2) facing +y; the collector 0.8 m ahead of it, turned 90 deg
    ranges, a0, inc = _scan()
    k_ahead = len(ranges) // 2                      # angle 0 in the laser frame
    ranges[k_ahead] = 0.8 - 0.165                    # its side, as the LiDAR sees it
    out, n = filter_ranges(ranges, a0, inc, (2.0, 2.0, math.pi / 2),
                           (2.0, 2.8, math.pi / 2), COLLECTOR, 0.10)
    assert n == 1 and math.isnan(out[k_ahead])


def test_body_elsewhere_or_no_returns_untouched():
    ranges, a0, inc = _scan()
    ranges[5] = float('inf')
    out, n = filter_ranges(ranges, a0, inc, (0.0, 0.0, 0.0), (10.0, 10.0, 0.0), 0.17, 0.1)
    assert n == 0 and out[5] == float('inf') and out[6] == 3.0
