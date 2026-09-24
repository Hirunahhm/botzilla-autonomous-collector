"""Tests for cube_detections: detection projection and matching to layout cubes."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from botzilla_navigation.cube_detections import (  # noqa: E402,I100
    match_detection,
    project_detection,
)
from botzilla_navigation.swept_mask import CAMERA_HALF_FOV_RAD  # noqa: E402


def test_centred_detection_projects_straight_ahead():
    x, y = project_detection(1.0, 2.0, 0.0, 0.0, 0.8)
    assert math.isclose(x, 1.8) and math.isclose(y, 2.0)


def test_positive_x_norm_is_to_the_right():
    # Facing +y: right of the robot is +x.
    x, y = project_detection(0.0, 0.0, math.pi / 2.0, 1.0, 1.0)
    assert math.isclose(y, 1.0, abs_tol=1e-9)
    assert math.isclose(x, math.tan(CAMERA_HALF_FOV_RAD), abs_tol=1e-9)


def test_match_picks_the_nearest_cube_within_radius():
    cubes = [(1, 0.0, 0.0), (2, 1.0, 0.0)]
    cube_id, dist = match_detection(0.8, 0.1, cubes)
    assert cube_id == 2
    assert math.isclose(dist, math.hypot(0.2, 0.1))


def test_no_cube_within_radius_is_a_false_positive():
    cube_id, dist = match_detection(3.0, 3.0, [(1, 0.0, 0.0)], radius_m=0.5)
    assert cube_id is None
    assert dist > 0.5


def test_empty_layout_matches_nothing():
    assert match_detection(0.0, 0.0, []) == (None, None)
