"""Tests for mission_metrics_node's geometry and its passive-by-construction property."""

import ast
import math
import os

from botzilla_navigation.swept_mask import (
    CAMERA_HALF_FOV_RAD,
    CAMERA_MARK_RANGE_M,
    create_swept_mask,
    is_in_frustum,
    mark_swept_cells,
)

NODE_SOURCE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'botzilla_navigation',
    'mission_metrics_node.py',
)

# Anything that lets a node act on the world rather than merely observe it. See
# mission_metrics_node's module docstring: the whole value of running it during a real
# hardware mission is that it provably cannot have influenced that mission.
FORBIDDEN_CALLS = (
    'create_publisher',
    'create_client',
    'create_service',
    'ActionClient',
    'StaticTransformBroadcaster',
    'TransformBroadcaster',
)


def test_metrics_node_creates_no_publishers_or_clients():
    """mission_metrics_node must be subscribe-only — it may observe, never act.

    Enforced against the source rather than a live node so the guarantee holds without a
    ROS graph, and so a future edit that adds a publisher fails here loudly instead of
    silently turning every recorded run into a run that may have been influenced.
    """
    with open(NODE_SOURCE) as fh:
        tree = ast.parse(fh.read())

    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = func.attr if isinstance(func, ast.Attribute) else getattr(func, 'id', None)
        if name in FORBIDDEN_CALLS:
            found.append(f'{name} at line {node.lineno}')

    assert not found, (
        'mission_metrics_node must not be able to act on the robot, but found: '
        + ', '.join(found)
    )


def test_frustum_accepts_straight_ahead_within_range():
    assert is_in_frustum(0.0, 0.0, 0.0, 0.9, 0.0)


def test_frustum_rejects_beyond_range():
    """A point dead ahead but past the detection range is not inspected."""
    assert not is_in_frustum(0.0, 0.0, 0.0, CAMERA_MARK_RANGE_M + 0.01, 0.0)


def test_frustum_rejects_behind_the_robot():
    """The Kinect is forward-facing — this is the whole premise of the coverage problem.

    A 360-degree detector would make un-swept area meaningless, so a point directly
    behind the robot must never count as inspected however close it is.
    """
    assert not is_in_frustum(0.0, 0.0, 0.0, -0.5, 0.0)


def test_frustum_edge_is_the_half_fov():
    """Just inside the cone edge is inspected; just outside is not."""
    r = 0.5
    inside = CAMERA_HALF_FOV_RAD - 0.01
    outside = CAMERA_HALF_FOV_RAD + 0.01
    assert is_in_frustum(0.0, 0.0, 0.0, r * math.cos(inside), r * math.sin(inside))
    assert not is_in_frustum(0.0, 0.0, 0.0, r * math.cos(outside), r * math.sin(outside))


def test_frustum_follows_robot_heading():
    """The cone rotates with the robot, so a point is inspected only from a heading."""
    assert not is_in_frustum(0.0, 0.0, 0.0, 0.0, 0.9)
    assert is_in_frustum(0.0, 0.0, math.pi / 2.0, 0.0, 0.9)


def test_metric_and_coverage_map_agree_on_the_same_point():
    """The paper's two numbers must come from one geometry, not two look-alike copies.

    A cube's inspection time (is_in_frustum) and the coverage map (mark_swept_cells) are
    reported side by side, so if they ever disagreed about the same spot the comparison
    would be meaningless. This walks a grid and asserts cell-by-cell agreement.
    """
    width = height = 21
    resolution = 0.1
    origin_x = origin_y = -1.0
    robot_x, robot_y, robot_yaw = 0.0, 0.0, 0.3

    mask = create_swept_mask(width, height)
    mark_swept_cells(
        mask, width, height, resolution, origin_x, origin_y, robot_x, robot_y, robot_yaw,
    )

    for row in range(height):
        for col in range(width):
            cell_x = origin_x + (col + 0.5) * resolution
            cell_y = origin_y + (row + 0.5) * resolution
            expected = is_in_frustum(robot_x, robot_y, robot_yaw, cell_x, cell_y)
            assert mask[row * width + col] == expected, (
                f'disagreement at cell ({row}, {col}) -> world ({cell_x:.2f}, {cell_y:.2f})'
            )
