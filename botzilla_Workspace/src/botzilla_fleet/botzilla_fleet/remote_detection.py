"""
remote_detection.py — the collector half of detecting cubes on the leader's GPU. ROS-free.

The collector's Pi cannot run YOLO alongside Nav2, AMCL and the Kinect (measured: 1.2
fps, load average 8 on 4 cores), so its camera is detected on the leader: the Pi sends
JPEG frames, the leader's yolo_node (boxes mode) sends back pixel boxes stamped with the
frame's own stamp, and the Pi pairs them with the depth frame taken at that moment.
Depth never crosses the network.

DepthBuffer keeps the last ~2 s of depth frames so a box that comes back after the
round trip is ranged against the depth the camera saw at the same instant, not the
newest one: while the robot turns at 0.35 rad/s the image moves ~20 px per 100 ms.
"""
from collections import deque

# Pairing tolerance between a box's stamp and a depth frame's. kinect_bridge publishes
# RGB and depth from separate freenect callbacks at ~20 Hz each, so the nearest depth
# frame is normally within half a period (25 ms).
MATCH_TOLERANCE_S = 0.06
# Boxes older than this when they arrive are dropped: the robot has moved on, and a
# stale offset would steer TARGETING the wrong way.
MAX_BOX_AGE_S = 0.6
BUFFER_S = 2.0


class DepthBuffer:
    def __init__(self, keep_s=BUFFER_S):
        self.keep_s = keep_s
        self._frames = deque()      # (stamp_s, image, encoding), oldest first

    def add(self, stamp_s, image, encoding):
        self._frames.append((stamp_s, image, encoding))
        while self._frames and stamp_s - self._frames[0][0] > self.keep_s:
            self._frames.popleft()

    def nearest(self, stamp_s, tolerance_s=MATCH_TOLERANCE_S):
        """(image, encoding, offset_s) of the frame closest in time, or None."""
        best = None
        for t, img, enc in self._frames:
            d = abs(t - stamp_s)
            if best is None or d < best[2]:
                best = (img, enc, d)
        if best is None or best[2] > tolerance_s:
            return None
        return best

    def __len__(self):
        return len(self._frames)


class RelayRate:
    """Pass at most one frame per period, measured on the frames' own stamps."""

    def __init__(self, hz):
        self.period = 1.0 / hz if hz > 0 else 0.0
        self._last = None

    def due(self, stamp_s):
        if self._last is None or stamp_s - self._last >= self.period - 1e-3:
            self._last = stamp_s
            return True
        return False
