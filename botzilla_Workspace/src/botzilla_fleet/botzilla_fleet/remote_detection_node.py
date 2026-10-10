"""
remote_detection_node.py — the collector's cube detector, with YOLO on the leader's GPU.

Runs on the collector (the Pi) under its namespace, in place of yolo_node:

  camera/rgb/image_raw ──JPEG, RELAY_HZ──► camera/rgb/compressed ──► leader yolo_node
                                                                       (mode boxes)
  yolo/boxes (pixel centres, source stamp) ◄─────────────────────────────┘
        + camera/depth/image_raw at that stamp (DepthBuffer)
        ──► detected_cube  (geometry_msgs/Point, exactly yolo_node's x/z convention)

so collector_node, and the executor states it inherits, cannot tell the difference.
See remote_detection.py for why and how the pairing works.

Frames are only sent while the collector can use a detection (ACTIVE_STATES, from its
own fleet/status): an idle or delivering collector costs the network and the leader's
GPU nothing. Set always_on:=true to stream regardless (bring-up tests).

If the network drops, boxes stop arriving and so do detections. That is safe by
construction: GOING still reaches its standoff, SEEKING times out, a chase hits the
executor's CUBE_LOST_TIMEOUT_S, and delivery never needed the camera.
"""
import statistics
import time

from botzilla_fleet.remote_detection import (
    DepthBuffer, MAX_BOX_AGE_S, RelayRate,
)
from botzilla_interfaces.msg import CollectorStatus, CubeBoxes
from botzilla_perception.cube_depth import closest_cube
import cv2
import cv_bridge
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

RELAY_HZ = 8.0
JPEG_QUALITY = 80
ACTIVE_STATES = {'GOING', 'SEEKING', 'TARGETING', 'APPROACHING', 'CAPTURING'}
STATS_PERIOD_S = 10.0


def _stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class RemoteDetectionNode(Node):
    def __init__(self):
        super().__init__('remote_detection_node')
        self.declare_parameter('relay_hz', RELAY_HZ)
        self.declare_parameter('jpeg_quality', JPEG_QUALITY)
        self.declare_parameter('always_on', False)
        self._rate = RelayRate(float(self.get_parameter('relay_hz').value))
        self._quality = int(self.get_parameter('jpeg_quality').value)
        self._always_on = bool(self.get_parameter('always_on').value)

        self._bridge = cv_bridge.CvBridge()
        self._depth = DepthBuffer()
        self._state = None
        self._stats = {'sent': 0, 'boxes': 0, 'detections': 0, 'stale': 0,
                       'no_depth': 0, 'bytes': 0}
        self._rtt = []
        self._inference = []

        self._jpeg_pub = self.create_publisher(
            CompressedImage, 'camera/rgb/compressed', qos_profile_sensor_data)
        self._cube_pub = self.create_publisher(Point, 'detected_cube', 10)
        self.create_subscription(
            Image, 'camera/rgb/image_raw', self._rgb_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, 'camera/depth/image_raw', self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(CubeBoxes, 'yolo/boxes', self._boxes_cb, 10)
        self.create_subscription(CollectorStatus, 'fleet/status', self._status_cb, 10)
        self.create_timer(STATS_PERIOD_S, self._log_stats)
        self.get_logger().info(
            f'remote detection: JPEG q{self._quality} at <= {1.0 / self._rate.period:.0f} Hz '
            f'to the leader while {"always" if self._always_on else sorted(ACTIVE_STATES)}'
        )

    def _active(self):
        return self._always_on or self._state in ACTIVE_STATES

    def _status_cb(self, msg):
        self._state = msg.state

    def _depth_cb(self, msg):
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self._depth.add(_stamp_s(msg.header.stamp), img, msg.encoding)

    def _rgb_cb(self, msg):
        if not self._active() or not self._rate.due(_stamp_s(msg.header.stamp)):
            return
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self._jpeg_pub.publish(out)
        self._stats['sent'] += 1
        self._stats['bytes'] += len(out.data)

    def _boxes_cb(self, msg):
        self._stats['boxes'] += 1
        t_img = _stamp_s(msg.header.stamp)
        age = time.time() - t_img
        self._rtt.append(age)
        self._inference.append(msg.inference_s)
        if not msg.cx:
            return
        if age > MAX_BOX_AGE_S:
            self._stats['stale'] += 1
            return
        match = self._depth.nearest(t_img)
        if match is None:
            # Dropped, not reported as z = 0: to the executor z = 0 means "in the blind
            # spot, capture it", and a missing depth frame is no evidence of that.
            self._stats['no_depth'] += 1
            return
        depth_img, encoding, _ = match
        best = closest_cube(list(zip(msg.cx, msg.cy)), depth_img, encoding, msg.width)
        if best is None:
            return
        self._cube_pub.publish(Point(x=best[0], y=0.0, z=best[1]))
        self._stats['detections'] += 1
        self.get_logger().info(
            f'[PUBLISH] detected_cube: x={best[0]:.2f}, z={best[1]:.2f}m '
            f'(round trip {age * 1000:.0f} ms)', throttle_duration_sec=1.0)

    def _log_stats(self):
        s = self._stats
        if s['sent'] == 0 and s['boxes'] == 0:
            return
        rtt = (f'{statistics.median(self._rtt) * 1000:.0f}/{max(self._rtt) * 1000:.0f} ms'
               if self._rtt else '-')
        inf = (f'{statistics.median(self._inference) * 1000:.0f} ms'
               if self._inference else '-')
        self.get_logger().info(
            f'last {STATS_PERIOD_S:.0f}s: sent {s["sent"]} frames '
            f'({s["bytes"] / STATS_PERIOD_S / 1024:.0f} KiB/s), got {s["boxes"]} box msgs, '
            f'{s["detections"]} detections, {s["stale"]} stale, {s["no_depth"]} without '
            f'depth | round trip median/max {rtt}, leader inference {inf} '
            f'| state {self._state}'
        )
        self._stats = dict.fromkeys(s, 0)
        self._rtt, self._inference = [], []


def main(args=None):
    rclpy.init(args=args)
    node = RemoteDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
