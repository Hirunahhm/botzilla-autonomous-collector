"""
cube_depth.py — depth at a detected cube's pixel, in metres. ROS-free.

Shared by yolo_node (detection and depth on one machine) and botzilla_fleet's
remote_detection_node (boxes from the leader's GPU, depth on the collector), so the two
cannot drift apart. Moved here verbatim from yolo_node.get_depth_at.

camera/depth/image_raw carries a different format on hardware than in sim, so the
conversion has to branch on the message encoding:

* mono8   — hardware. kinect_bridge rescales the Kinect's native 11-bit disparity
            (0-2047, 2047 = no data) down to 0-255 to publish it as a standard mono8
            Image. Undo that, then apply the Kinect disparity->metres formula.
* 32FC1   — simulation. simulation.launch.py's ros_gz_bridge maps Gazebo's depth
            camera straight through, and Gazebo already emits metres. Also what
            camera/depth/image_meters carries on hardware.
* 16UC1   — millimetres, the common depth convention if a driver is ever swapped in
            that publishes it.

Getting this wrong is silent, not loud: the mono8 branch applied to metric data
returns a plausible-looking number that is simply wrong, which would make the FSM
misjudge every approach distance.
"""
import numpy as np

# Kinect minimum sensing range (objects closer become 0 or invalid). Also the
# blind-spot cutoff kinect_bridge agrees on — see CLAUDE.md "Depth handling".
KINECT_MIN_RANGE_M = 0.55


def depth_at(depth_img, cx, cy, encoding='mono8'):
    """Median depth (m) of a 5x5 patch around (cx, cy), or None if invalid."""
    h, w = depth_img.shape[:2]
    cx = max(2, min(int(cx), w - 3))
    cy = max(2, min(int(cy), h - 3))

    patch = depth_img[cy - 2:cy + 3, cx - 2:cx + 3].flatten().astype(np.float32)
    # Gazebo writes inf/NaN for "no return"; the Kinect path writes 0.
    valid = patch[np.isfinite(patch) & (patch > 0)]
    if len(valid) == 0:
        return None

    raw_val = float(np.median(valid))
    if encoding == '32FC1':
        distance_m = raw_val
    elif encoding == '16UC1':
        distance_m = raw_val / 1000.0
    else:
        # mono8 (hardware Kinect via kinect_bridge): reverse the 0-255 rescale back to
        # the native 11-bit value, then disparity -> metres.
        raw_11bit = (raw_val / 255.0) * 2047.0
        if raw_11bit >= 2040:  # Kinect reports 2047 for no-data pixels
            return None
        distance_m = 1.0 / (raw_11bit * -0.0030711016 + 3.3309495161)

    # Guard against nonsense from any branch (negative/infinite values near the
    # disparity formula's asymptote, or a bad sim frame).
    if not np.isfinite(distance_m) or distance_m <= 0.0 or distance_m > 20.0:
        return None
    return distance_m


def closest_cube(centres, depth_img, encoding, img_width):
    """Pick the cube to report from box centres [(cx, cy), ...] in pixels.

    Returns (norm_x, z) for detected_cube, or None if there are no boxes. z is metres,
    or 0.0 for "closer than KINECT_MIN_RANGE_M or no depth" (the blind-spot signal the
    executor relies on). Same choice yolo_node always made: the nearest cube, where a
    blind-spot cube (z = 0) counts as nearest of all — it is the one being captured.
    """
    centre_x = img_width / 2.0
    best, best_dist = None, float('inf')
    for cx, cy in centres:
        norm_x = (cx - centre_x) / centre_x
        dist = depth_at(depth_img, cx, cy, encoding) if depth_img is not None else None
        if dist is None or dist < KINECT_MIN_RANGE_M:
            dist = 0.0
        if dist < best_dist or (dist == 0.0 and best is None):
            best_dist = dist
            best = (norm_x, dist)
    return best
