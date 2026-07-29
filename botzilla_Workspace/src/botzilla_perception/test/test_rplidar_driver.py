"""
test_rplidar_driver.py.

Pure pytest for botzilla_perception.rplidar_driver's packet parsing and scan-binning
logic — no ROS imports, no hardware, runnable directly with `python3 -m pytest`
without sourcing a ROS environment.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from botzilla_perception.rplidar_driver import (  # noqa: E402,I100
    bin_scan_points,
    parse_health_payload,
    parse_info_payload,
    parse_scan_point,
    ScanPoint,
)

# quality=15, start_flag=1, inv_start_flag=0, check_bit=1, angle=90.0deg, distance=1000.0mm
VALID_PACKET = bytes([61, 1, 45, 160, 15])
# Same as above but check_bit=0 -> invalid.
INVALID_CHECKBIT_PACKET = bytes([61, 0, 45, 160, 15])


def test_parse_scan_point_valid():
    point = parse_scan_point(VALID_PACKET)
    assert point.valid is True
    assert point.new_revolution is True
    assert point.quality == 15
    assert point.angle_deg == 90.0
    assert point.distance_mm == 1000.0


def test_parse_scan_point_invalid_checkbit():
    point = parse_scan_point(INVALID_CHECKBIT_PACKET)
    assert point.valid is False


def test_parse_scan_point_wrong_length_raises():
    try:
        parse_scan_point(bytes([1, 2, 3, 4]))
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_parse_info_payload():
    data = bytes([85, 2, 1, 10]) + bytes(range(16))
    info = parse_info_payload(data)
    assert info['model'] == 85
    assert info['firmware_minor'] == 2
    assert info['firmware_major'] == 1
    assert info['hardware'] == 10
    assert info['serial_number'] == bytes(range(16))[::-1].hex()


def test_parse_info_payload_wrong_length_raises():
    try:
        parse_info_payload(bytes(10))
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_parse_health_payload():
    health = parse_health_payload(bytes([0, 5, 1]))
    assert health['status_code'] == 0
    assert health['status_text'] == 'Good'
    assert health['error_code'] == 5 | (1 << 8)


def test_parse_health_payload_wrong_length_raises():
    try:
        parse_health_payload(bytes([0, 0]))
        assert False, 'expected ValueError'
    except ValueError:
        pass


def _point(angle_deg, distance_mm, valid=True):
    return ScanPoint(
        valid=valid, new_revolution=False, quality=10,
        angle_deg=angle_deg, distance_mm=distance_mm,
    )


def test_bin_scan_points_places_points_in_correct_bins():
    # The RPLIDAR's angle_deg increases CLOCKWISE; a LaserScan bin index increases
    # COUNTER-CLOCKWISE (REP-103), so the angle is negated when binning.
    # 4 bins => 90 deg each. RPLIDAR 10 deg CW  == bearing -10 deg == 350 deg CCW -> bin 3.
    #                        RPLIDAR 100 deg CW == bearing -100 deg == 260 deg CCW -> bin 2.
    points = [_point(10.0, 1000.0), _point(100.0, 2000.0)]
    ranges, intensities = bin_scan_points(
        points, num_samples=4, range_min_m=0.05, range_max_m=25.0,
    )
    assert ranges == [float('inf'), float('inf'), 2.0, 1.0]
    assert intensities[3] == 10.0
    assert intensities[2] == 10.0


def test_bin_scan_points_is_not_mirrored():
    """Regression test: a point to the robot's RIGHT must land in the lower half.

    Binning the RPLIDAR's clockwise angle directly published a mirror image of the
    room. That is invisible while stationary, but during a turn the mirrored pattern
    rotates WITH the robot instead of against it, so walls sweep at twice the turn
    rate and the occupancy grid fills with rotated duplicate walls.
    """
    n = 360  # 1 deg per bin, so bin index == bearing in degrees CCW
    # RPLIDAR 90 deg (clockwise) is physically to the robot's RIGHT.
    # In ROS that is bearing -90 deg == 270 deg CCW.
    ranges, _ = bin_scan_points(
        [_point(90.0, 1000.0)], num_samples=n, range_min_m=0.05, range_max_m=25.0,
    )
    assert ranges[270] == 1.0, 'point to the right must bin at 270 deg CCW'
    assert ranges[90] == float('inf'), 'binning at +90 deg would mean a mirrored scan'


def test_bin_scan_points_ignores_invalid_points():
    points = [_point(10.0, 1000.0, valid=False)]
    ranges, _ = bin_scan_points(points, num_samples=4, range_min_m=0.05, range_max_m=25.0)
    assert ranges == [float('inf')] * 4


def test_bin_scan_points_ignores_zero_distance_no_return():
    points = [_point(10.0, 0.0)]
    ranges, _ = bin_scan_points(points, num_samples=4, range_min_m=0.05, range_max_m=25.0)
    assert ranges == [float('inf')] * 4


def test_bin_scan_points_ignores_out_of_range():
    points = [_point(10.0, 50.0)]  # 0.05m, exactly at range_min boundary is kept below
    ranges, _ = bin_scan_points(points, num_samples=4, range_min_m=0.5, range_max_m=25.0)
    assert ranges == [float('inf')] * 4


def test_bin_scan_points_closest_point_wins_within_a_bin():
    # Both 10 and 20 deg CW fall in the same 90 deg bin (bin 3, see binning test above).
    points = [_point(10.0, 1500.0), _point(20.0, 1200.0)]
    ranges, _ = bin_scan_points(points, num_samples=4, range_min_m=0.05, range_max_m=25.0)
    assert ranges[3] == 1.2


def test_bin_scan_points_empty_input_returns_all_inf():
    ranges, intensities = bin_scan_points(
        [], num_samples=4, range_min_m=0.05, range_max_m=25.0,
    )
    assert ranges == [float('inf')] * 4
    assert intensities == [0.0] * 4
