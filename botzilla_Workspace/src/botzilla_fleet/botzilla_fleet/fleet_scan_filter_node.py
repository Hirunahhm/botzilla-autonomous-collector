"""
fleet_scan_filter_node.py — drop the other robot's returns from this robot's scans.

Sits between the sensors and everything that reads them (body_filter.py says why):

    rplidar_node            -> scan_raw         -> [this node] -> scan         (RTAB-Map,
    pointcloud_to_laserscan -> scan_camera_raw  -> [this node] -> scan_camera   Nav2, AMCL)

so RTAB-Map, Nav2 and AMCL need no changes; the launch files only rename the sensors'
outputs (hardware.launch.py scan_topic / scan_camera_topic) when this node runs.

  other:=collector   on the leader: the collector's pose from /<ns>/fleet/status
  other:=leader      on the collector: the leader's pose from fleet/leader_pose

If the other robot's pose is older than max_age_s, or the scan's frame cannot be placed
in the map, the scan goes through UNFILTERED: a lost link then falls back to seeing the
other robot as an ordinary obstacle, which is the safe failure.
"""
import math

from botzilla_fleet.body_filter import filter_ranges
from botzilla_fleet.fleet_manager_node import COLLECTOR_SHAPE, LEADER_BODY_RADIUS_M
from botzilla_interfaces.msg import CollectorStatus
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_ros import TransformException

MAP_FRAME = 'map'
SCANS = (('scan_raw', 'scan'), ('scan_camera_raw', 'scan_camera'))
STATS_PERIOD_S = 30.0


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FleetScanFilter(Node):
    """Republish scans with the other robot's body removed."""

    def __init__(self):
        super().__init__('fleet_scan_filter')
        self.declare_parameter('other', 'collector')
        self.declare_parameter('collector_ns', 'bz2')
        self.declare_parameter('margin_m', 0.10)
        self.declare_parameter('max_age_s', 1.0)
        other = self.get_parameter('other').value
        ns = self.get_parameter('collector_ns').value.strip('/')
        self._margin = float(self.get_parameter('margin_m').value)
        self._max_age = float(self.get_parameter('max_age_s').value)
        self._body = None            # (x, y, yaw) of the other robot in the map
        self._body_time = None
        self._removed = {out: 0 for _, out in SCANS}
        self._passed = {out: 0 for _, out in SCANS}

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        if other == 'collector':
            self._shape = COLLECTOR_SHAPE
            self.create_subscription(CollectorStatus, f'/{ns}/fleet/status',
                                     self._status_cb, 10)
        elif other == 'leader':
            self._shape = LEADER_BODY_RADIUS_M
            self.create_subscription(PoseStamped, 'fleet/leader_pose', self._leader_cb, 10)
        else:
            raise ValueError(f"other must be 'collector' or 'leader' (got '{other}')")
        for src, out in SCANS:
            pub = self.create_publisher(LaserScan, out, qos_profile_sensor_data)
            self.create_subscription(
                LaserScan, src, lambda m, p=pub, o=out: self._scan_cb(m, p, o),
                qos_profile_sensor_data)
        self.create_timer(STATS_PERIOD_S, self._stats)
        self.get_logger().info(
            f'fleet_scan_filter: removing the {other} from {[s for s, _ in SCANS]} '
            f'(margin {self._margin} m, pose max age {self._max_age} s)')

    def _status_cb(self, msg):
        if msg.localised:
            self._body = (msg.x, msg.y, msg.yaw)
            self._body_time = self.get_clock().now()

    def _leader_cb(self, msg):
        p = msg.pose
        self._body = (p.position.x, p.position.y, _yaw(p.orientation))
        self._body_time = self.get_clock().now()

    def _laser_pose(self, frame):
        try:
            tf = self._tf_buffer.lookup_transform(MAP_FRAME, frame, Time())
        except TransformException:
            return None
        t = tf.transform
        return t.translation.x, t.translation.y, _yaw(t.rotation)

    def _scan_cb(self, msg, pub, out):
        body, laser = self._body, None
        fresh = (body is not None and self._body_time is not None
                 and (self.get_clock().now() - self._body_time).nanoseconds / 1e9
                 <= self._max_age)
        if fresh:
            laser = self._laser_pose(msg.header.frame_id)
        if laser is not None:
            ranges, n = filter_ranges(msg.ranges, msg.angle_min, msg.angle_increment,
                                      laser, body, self._shape, self._margin)
            if n:
                msg.ranges = ranges
                self._removed[out] += n
        else:
            self._passed[out] += 1
        pub.publish(msg)

    def _stats(self):
        self.get_logger().info(
            f'returns removed {self._removed}, scans passed unfiltered (no pose/TF) '
            f'{self._passed} in the last {STATS_PERIOD_S:.0f}s')
        self._removed = dict.fromkeys(self._removed, 0)
        self._passed = dict.fromkeys(self._passed, 0)


def main(args=None):
    rclpy.init(args=args)
    node = FleetScanFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
