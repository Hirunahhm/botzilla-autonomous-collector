"""
rplidar_node.py.

ROS2 wrapper around rplidar_driver.RPLidar, publishing sensor_msgs/LaserScan on /scan.
See rplidar_driver.py's module docstring for why this bypasses the upstream
ros-jazzy-rplidar-ros package.

A background thread owns the serial connection and blocks on RPLidar.iter_scans(),
accumulating points for the revolution currently in progress. Each time a new
revolution starts, the just-completed revolution's points are angle-binned into a
fixed-size LaserScan and handed off (under a lock) to a periodic timer on the node's
own thread, which does the actual publish() call — the same producer-thread /
timer-consumer split used by kinect_bridge.py, so a slow or stalled ROS executor never
blocks the serial read loop.

If the serial connection drops or the driver raises, the thread reconnects after
RECONNECT_DELAY_S rather than exiting, so the node recovers from a transient
disconnect (e.g. a USB re-enumeration) without needing to be relaunched.

Usage:
  ros2 run botzilla_perception rplidar_node
  ros2 run botzilla_perception rplidar_node --ros-args -p port:=/dev/ttyUSB1
"""
import math
import threading
import time

from botzilla_perception.rplidar_driver import (
    bin_scan_points,
    DEFAULT_BAUD,
    DEFAULT_PORT,
    RPLidar,
    RPLidarError,
)
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# Angular resolution of the published scan. The C1's legacy-mode point rate (~500
# points/revolution, observed live) comfortably covers this many bins without most
# of them going empty.
NUM_SAMPLES = 400

RANGE_MIN_M = 0.05
RANGE_MAX_M = 25.0

# How long to wait before retrying after a connect failure or a dropped connection.
RECONNECT_DELAY_S = 3.0

# Consecutive all-empty revolutions tolerated before forcing a reconnect.
#
# Attaching to a device that is still mid-stream (e.g. after the node was killed
# without a clean stop_scan) leaves the serial reader byte-misaligned. Points then
# parse as invalid, bin_scan_points filters every one of them, and the node happily
# publishes a 400-bin all-inf LaserScan forever — no exception, so the reconnect path
# in _run() never fires. Downstream this is silent and severe: RTAB-Map keeps mapping
# from depth alone with nothing to constrain scan registration, which draws the same
# room outline at several poses (a rotational "fan" of ghost walls). Measured on this
# hardware: desynced node published 0/400 finite bins; after a clean reconnect, 364/400.
# A genuinely blank scan (lidar staring into open space beyond range) is possible but
# will not persist for seconds on end, so a streak is a reliable desync signal.
EMPTY_SCAN_LIMIT = 20   # ~2s at the C1's ~10 rev/s

# Matches the <link name="laser_frame"> in botzilla_qbot.urdf (also the frame the
# sim's gpu_lidar publishes under), so /scan is usable via the same TF frame on both
# hardware and sim.
DEFAULT_FRAME_ID = 'laser_frame'


class RPLidarNode(Node):

    def __init__(self):
        super().__init__('rplidar_node')

        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUD)
        self.declare_parameter('frame_id', DEFAULT_FRAME_ID)

        self._port = self.get_parameter('port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._frame_id = self.get_parameter('frame_id').value

        self._scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)

        self._lock = threading.Lock()
        self._latest = None  # (ranges, intensities, scan_time_s) or None once consumed

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        self.create_timer(0.05, self._publish_latest)
        self.get_logger().info(
            f'rplidar_node started on {self._port} @ {self._baudrate} baud'
        )

    # ------------------------------------------------------------------ #
    # Serial thread — connect, stream scans, reconnect on failure
    # ------------------------------------------------------------------ #

    def _run(self):
        while rclpy.ok():
            lidar = RPLidar(port=self._port, baudrate=self._baudrate)
            try:
                lidar.connect()
                health = lidar.get_health()
                if health['status_code'] == 2:
                    self.get_logger().error(f'RPLidar health ERROR: {health}')
                    time.sleep(RECONNECT_DELAY_S)
                    continue
                lidar.start_motor()
                self._stream_scans(lidar)
            except Exception as ex:
                self.get_logger().warn(
                    f'RPLidar connection lost ({ex}); reconnecting in '
                    f'{RECONNECT_DELAY_S:.0f}s'
                )
            finally:
                lidar.disconnect()
            time.sleep(RECONNECT_DELAY_S)

    def _stream_scans(self, lidar):
        points = []
        empty_streak = 0
        revolution_start = time.monotonic()
        # ROS time of the first ray of the revolution currently being accumulated.
        # Captured here, in the serial thread, NOT at publish time — see
        # _publish_latest() for why that distinction matters.
        revolution_start_ros = self.get_clock().now()

        for point in lidar.iter_scans():
            if not rclpy.ok():
                lidar.stop_scan()
                return

            if point.new_revolution and points:
                now = time.monotonic()
                now_ros = self.get_clock().now()
                scan_time = now - revolution_start
                # Midpoint of the revolution, not its first ray — see _publish_latest().
                stamp = revolution_start_ros + Duration(seconds=scan_time / 2.0)
                revolution_start = now
                revolution_start_ros = now_ros

                ranges, intensities = bin_scan_points(
                    points, NUM_SAMPLES, RANGE_MIN_M, RANGE_MAX_M
                )

                if any(math.isfinite(r) for r in ranges):
                    empty_streak = 0
                else:
                    empty_streak += 1
                    if empty_streak >= EMPTY_SCAN_LIMIT:
                        # Raise so _run()'s handler disconnects and reconnects; connect()
                        # sends CMD_STOP and flushes the input buffer, which resyncs.
                        raise RPLidarError(
                            f'{empty_streak} consecutive empty scans '
                            f'({len(points)} pts/rev parsed, all filtered) — serial '
                            f'stream is desynchronised, forcing reconnect'
                        )

                with self._lock:
                    self._latest = (ranges, intensities, scan_time, stamp)
                points = []

            points.append(point)

    # ------------------------------------------------------------------ #
    # Publish timer — runs on the node's own thread/executor
    # ------------------------------------------------------------------ #

    def _publish_latest(self):
        with self._lock:
            latest = self._latest
            self._latest = None
        if latest is None:
            return
        ranges, intensities, scan_time, stamp = latest

        msg = LaserScan()
        # Stamp with when the data was CAPTURED, not with "now". Stamping at publish time
        # put the scan ~100-150ms in the future relative to its own data — a full
        # revolution (~100ms) elapses before the serial thread hands the scan off, plus up
        # to 50ms waiting for this timer. Consumers then resolved TF at that later time, so
        # during a turn every scan got rotated by however far the robot travelled in the
        # interim: at Nav2's max_vel_theta of 0.4 rad/s that is ~2.9 deg, i.e. ~26cm of
        # wall displacement at 5m — several map cells, drawn as a second copy of the wall.
        #
        # The capture instant used is the revolution's MIDPOINT, which is a deliberate
        # departure from the strict sensor_msgs/LaserScan convention of stamping the first
        # ray. The rays of one revolution are spread over scan_time (~100ms measured), and
        # every consumer in this stack — RTAB-Map's occupancy grid and Nav2's obstacle
        # layer — collapses them to the single instant in header.stamp rather than
        # deskewing with time_increment. Against a non-deskewing consumer the first-ray
        # convention is not neutral: it biases the whole scan late by half a revolution,
        # every scan, in the same direction. Hardware measurement (rotate in place, then
        # search for the time offset best aligning each scan to a stationary reference)
        # showed exactly such a systematic offset. Stamping the midpoint removes the bias
        # and halves the worst-case skew to +/- scan_time/2.
        #
        # time_increment below is still reported honestly, so this only helps: a consumer
        # that DID deskew would need to account for stamp being the midpoint. None does
        # here (rtabmap_util's lidar_deskewing node takes PointCloud2, not LaserScan).
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = 2.0 * math.pi / NUM_SAMPLES
        msg.time_increment = scan_time / NUM_SAMPLES if NUM_SAMPLES else 0.0
        msg.scan_time = scan_time
        msg.range_min = RANGE_MIN_M
        msg.range_max = RANGE_MAX_M
        msg.ranges = ranges
        msg.intensities = intensities
        self._scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RPLidarNode()
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
