"""
cube_detections.py.

Pure, ROS-independent geometry for turning a /detected_cube message into a map
position and deciding which ground-truth cube (if any) it was. Same style as
swept_mask.py: no rclpy/message imports, unit-tested directly (see
test/test_cube_detections.py).

Two callers, deliberately sharing one definition:

  * executor_node projects the target of a chase it abandons, for /cube_abandoned.
  * mission_metrics_node projects every ranged detection and matches it against the
    layout, which is what turns "YOLO fired" into "cube 3 was detected at t=212 s" or
    "false positive". That is the DETECTED half of the experiment's metric pair (the
    INSPECTED half is swept_mask.is_in_frustum).
"""

import math

from botzilla_navigation.swept_mask import CAMERA_HALF_FOV_RAD

# A detection within this distance of a layout cube counts as that cube. The ranged
# detections are <=1 m (executor/explorer range gate), where the depth-plus-bearing
# projection has been a few tens of cm off at worst; cubes in a layout are expected to
# be further apart than this. research_discussion.md §6.7.
MATCH_RADIUS_M = 0.5


def project_detection(
    robot_x, robot_y, robot_yaw, x_norm, range_m, half_fov_rad=CAMERA_HALF_FOV_RAD,
):
    """Map-frame (x, y) of a detection seen from the given robot pose.

    yolo_node's x is the cube centre's offset from the image centre, normalised to +/-1
    at the image edges (positive = right). Under a pinhole model the image edge is the
    edge of the horizontal FOV, so the cube sits -x * tan(half_fov) * z to the robot's
    left at forward distance z. The camera's few-cm offset from base_link is ignored;
    it is well inside MATCH_RADIUS_M.
    """
    forward = range_m
    left = -x_norm * math.tan(half_fov_rad) * range_m
    return (
        robot_x + forward * math.cos(robot_yaw) - left * math.sin(robot_yaw),
        robot_y + forward * math.sin(robot_yaw) + left * math.cos(robot_yaw),
    )


def match_detection(x, y, cubes, radius_m=MATCH_RADIUS_M):
    """Return (cube_id, distance_m) of the nearest cube within radius_m, else (None, d).

    cubes is [(id, x, y), ...] as mission_metrics_node loads it. d is the distance to
    the nearest cube (None when there are no cubes), recorded either way so the match
    radius can be re-chosen offline without re-running anything.
    """
    best_id, best_d = None, None
    for cube_id, cx, cy in cubes:
        d = math.hypot(x - cx, y - cy)
        if best_d is None or d < best_d:
            best_id, best_d = cube_id, d
    if best_d is None or best_d > radius_m:
        return None, best_d
    return best_id, best_d
