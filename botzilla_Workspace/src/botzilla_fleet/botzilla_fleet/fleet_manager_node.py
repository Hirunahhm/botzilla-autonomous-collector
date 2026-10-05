"""
fleet_manager_node.py — the leader's task allocator: turn detections into collector tasks.

Runs on the leader (the Jetson), un-namespaced, next to its normal mission stack with
detect_only set, so the leader only searches and never chases. It

  1. projects every ranged /detected_cube into the map from the leader's own pose and
     merges it into a CubeRegistry (cube_registry.py: confirmation, HOME and
     collector exclusion zones, failure limits);
  2. whenever the collector reports IDLE, publishes the nearest confirmed cube to it as
     a CubeTask on /<ns>/fleet/task (latched, re-sent until the collector takes it);
  3. records the collector's COLLECTED / FAILED reports back into the registry.

Discrete goals only, never velocities (PROJECT.md §6): if the network drops the
collector finishes or fails its current task on its own, and a collector that goes
silent for STATUS_TIMEOUT_S has its task returned to the pool.

Also publishes /fleet/cube_obstacles (PointCloud2) so the leader's own Nav2 stops driving
into cubes it has already seen — see CUBE_OBSTACLE_Z — and /fleet/cubes (MarkerArray)
for RViz, and logs a one-line summary every SUMMARY_PERIOD_S, plus one JSON line per
event to the run log for analysis.
"""
import json
import math

from botzilla_fleet.cube_registry import CubeRegistry
from botzilla_interfaces.msg import CollectorStatus, CubeTask
from botzilla_navigation.cube_detections import project_detection
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import tf2_ros
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

CUBE_MAX_RANGE_M = 1.0              # = executor_node.CUBE_MAX_RANGE_M
COLLECTOR_HOME_EXCLUSION_M = 1.0    # = executor_node.HOME_CUBE_SUPPRESS_RADIUS_M
# Around the collector itself: the cube it is pushing sits ~0.3-0.6 m ahead of base_link.
COLLECTOR_BODY_EXCLUSION_M = 0.8
STATUS_TIMEOUT_S = 10.0
# After the collector reports its own failure ('collector:' detail, e.g. Nav2 not up),
# wait this long before assigning again; the cube goes back without a failure mark.
COLLECTOR_FAULT_HOLDOFF_S = 15.0
ALLOCATE_PERIOD_S = 1.0
SUMMARY_PERIOD_S = 30.0
MAP_FRAME = 'map'
ROBOT_FRAME = 'base_link'

# Known cubes as obstacles for the leader's costmaps (nav2_params.yaml source
# 'fleet_cubes'). Needed because the leader drives into cubes it has detected: the
# RPLIDAR scans at 0.24 m (URDF laser_joint), above a cube, so it never marks one and
# its rays clear the cells under them; the depth camera's low scan only sees from
# 0.55 m out; and in detect-only mode the executor deliberately ignores detections.
# Points are placed AT the LiDAR's height on purpose: re-published at CUBE_OBSTACLE_HZ
# a cube stays marked (each costmap update marks after it clears), and once a cube is
# no longer published (collected) the LiDAR's own clearing rays erase it.
CUBE_OBSTACLE_Z = 0.24
CUBE_OBSTACLE_HALF_M = 0.05      # 3 x 3 points 5 cm apart: a ~0.15 m square per cube
CUBE_OBSTACLE_HZ = 5.0

# Each robot's footprint as an obstacle for the OTHER robot's costmaps (nav2_params.yaml
# source 'fleet_robots'; the collector's copy reads /<ns>/fleet/robot_obstacles via
# params_rewrite). Needed because neither robot's LiDAR sees the other's body: the
# Kobuki is 0.09 m tall and the LiDARs scan at 0.24 m, where only the slim LiDAR
# housing and camera mount are, so returns come and go as the robots close in — the
# leader drove into the collector on 2026-10-05 after seeing it from further away.
# Same height trick as the cubes: re-marked every publish, cleared by the LiDAR once the
# robot has moved on. The footprint is nav2_params.yaml's (both robots share the URDF),
# with this margin added, and points 5 cm apart so no costmap cell inside is missed.
ROBOT_FOOTPRINT = ((-0.22, 0.36), (-0.215, 0.215))   # (x min/max, y min/max), base_link
ROBOT_OBSTACLE_MARGIN_M = 0.05
ROBOT_OBSTACLE_STEP_M = 0.05
# A pose older than this is not drawn: a stale footprint would block empty floor.
ROBOT_POSE_MAX_AGE_S = 1.5

COLOURS = {
    'unconfirmed': (0.6, 0.6, 0.6),
    'pending': (1.0, 0.8, 0.0),
    'assigned': (0.0, 0.6, 1.0),
    'collected': (0.0, 0.9, 0.0),
    'failed': (0.9, 0.0, 0.0),
}


class FleetManagerNode(Node):
    def __init__(self):
        super().__init__('fleet_manager_node')
        self.declare_parameter('collector_ns', 'bz2')
        self.declare_parameter('cube_max_range_m', CUBE_MAX_RANGE_M)
        self.declare_parameter('home_exclusion_m', COLLECTOR_HOME_EXCLUSION_M)
        self.declare_parameter('leader_home_exclusion_m', 0.0)
        self.declare_parameter('confirm_sightings', 2)
        self._ns = self.get_parameter('collector_ns').value.strip('/')
        self._max_range = self.get_parameter('cube_max_range_m').value
        self._home_excl = self.get_parameter('home_exclusion_m').value
        self._leader_home_excl = self.get_parameter('leader_home_exclusion_m').value
        self._registry = CubeRegistry(
            confirm_sightings=int(self.get_parameter('confirm_sightings').value)
        )

        self._leader_home = None
        self._status = None          # latest CollectorStatus
        self._status_time = None
        # Task ids are per ASSIGNMENT, not per cube: a cube that failed once is assigned
        # again later, and reusing its id would let the collector's report of the first
        # attempt (still in its status as last_task_id) close the second one at once.
        self._task_seq = 0
        self._holdoff_until = 0.0
        self._current = None         # (task_id, cube_id) assigned and not yet reported
        self._assigned_time = None
        self._home_zone_applied = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        self._task_pub = self.create_publisher(CubeTask, f'/{self._ns}/fleet/task', latched)
        self._marker_pub = self.create_publisher(MarkerArray, '/fleet/cubes', 10)
        self._obstacle_pub = self.create_publisher(PointCloud2, '/fleet/cube_obstacles', 10)
        self._collector_body_pub = self.create_publisher(
            PointCloud2, '/fleet/robot_obstacles', 10)            # for the leader
        self._leader_body_pub = self.create_publisher(
            PointCloud2, f'/{self._ns}/fleet/robot_obstacles', 10)  # for the collector
        m = ROBOT_OBSTACLE_MARGIN_M
        (x0, x1), (y0, y1) = ROBOT_FOOTPRINT
        xs = np.arange(x0 - m, x1 + m + 1e-9, ROBOT_OBSTACLE_STEP_M)
        ys = np.arange(y0 - m, y1 + m + 1e-9, ROBOT_OBSTACLE_STEP_M)
        self._footprint_pts = np.array([(x, y) for x in xs for y in ys])
        self.create_timer(1.0 / CUBE_OBSTACLE_HZ, self._publish_obstacles)
        self.create_subscription(Point, '/detected_cube', self._cube_cb, 10)
        self.create_subscription(
            CollectorStatus, f'/{self._ns}/fleet/status', self._status_cb, 10
        )
        self.create_timer(ALLOCATE_PERIOD_S, self._allocate)
        self.create_timer(SUMMARY_PERIOD_S, self._summary)
        self.get_logger().info(
            f'fleet_manager_node: leader searches, collector /{self._ns} collects. '
            f'Tasks on /{self._ns}/fleet/task.'
        )

    # ------------------------------------------------------------------ #

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _event(self, kind, **fields):
        fields.update(event=kind, t=round(self._now_s(), 2))
        self.get_logger().info('FLEET ' + json.dumps(fields))

    def _leader_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def _exclusions(self):
        zones = []
        if self._leader_home is not None and self._leader_home_excl > 0:
            zones.append((*self._leader_home, self._leader_home_excl))
        s = self._status
        if s is not None and s.home_set:
            zones.append((s.home_x, s.home_y, self._home_excl))
        if s is not None and s.localised and self._status_fresh():
            zones.append((s.x, s.y, COLLECTOR_BODY_EXCLUSION_M))
        return zones

    def _status_fresh(self):
        return (self._status_time is not None
                and self._now_s() - self._status_time < STATUS_TIMEOUT_S)

    # ------------------------------------------------------------------ #

    def _cube_cb(self, msg):
        if msg.z <= 0.0 or msg.z > self._max_range:
            return
        pose = self._leader_pose()
        if pose is None:
            return
        if self._leader_home is None:
            self._leader_home = pose[:2]
        x, y = project_detection(pose[0], pose[1], pose[2], msg.x, msg.z)
        before = len(self._registry.cubes)
        cube = self._registry.observe(x, y, self._now_s(), self._exclusions())
        if cube is None:
            return
        if len(self._registry.cubes) > before:
            self._event('new_cube', id=cube.id, x=round(x, 2), y=round(y, 2))
        elif cube.sightings == self._registry.confirm_sightings:
            self._event('confirmed', id=cube.id, x=round(cube.x, 2), y=round(cube.y, 2))

    def _status_cb(self, msg):
        first = self._status is None
        self._status = msg
        self._status_time = self._now_s()
        if first:
            self._event('collector_up', robot=msg.robot, state=msg.state)
        # A restarted manager must not reuse ids the collector has already seen.
        self._task_seq = max(self._task_seq, msg.task_id, msg.last_task_id)
        if msg.home_set and not self._home_zone_applied:
            self._home_zone_applied = True
            gone = self._registry.drop_inside([(msg.home_x, msg.home_y, self._home_excl)])
            self._event('collector_home', x=round(msg.home_x, 2), y=round(msg.home_y, 2),
                        dropped=gone)
        if (self._current is not None and msg.last_task_id == self._current[0]
                and msg.task_id != self._current[0]
                and msg.last_result != CollectorStatus.RESULT_NONE):
            collected = msg.last_result == CollectorStatus.RESULT_COLLECTED
            task_id, cube_id = self._current
            if not collected and msg.last_detail.startswith('collector:'):
                self._registry.unassign(cube_id)
                self._holdoff_until = self._now_s() + COLLECTOR_FAULT_HOLDOFF_S
                self._event('collector_fault', id=cube_id, task=task_id,
                            detail=msg.last_detail)
                self._current = None
                return
            cube = self._registry.report(cube_id, collected, self._now_s(),
                                         msg.last_detail)
            self._event('result', id=cube_id, task=task_id,
                        result='collected' if collected else 'failed',
                        detail=msg.last_detail, delivered=msg.delivered,
                        status=cube.status if cube else None,
                        took_s=round(self._now_s() - self._assigned_time, 1))
            self._current = None

    def _allocate(self):
        self._publish_markers()
        s = self._status
        if s is None:
            return
        if not self._status_fresh():
            if self._current is not None:
                self._event('collector_silent', id=self._current[1], task=self._current[0])
                self._registry.unassign(self._current[1])
                self._current = None
            return
        if self._current is not None or s.state != 'IDLE' or not s.localised:
            return
        if self._now_s() < self._holdoff_until:
            return
        cube = self._registry.next_task((s.x, s.y), self._now_s())
        if cube is None:
            return
        self._registry.assign(cube.id)
        self._task_seq += 1
        self._current = (self._task_seq, cube.id)
        self._assigned_time = self._now_s()
        task = CubeTask(task_id=self._task_seq, cube_id=cube.id, sightings=cube.sightings)
        task.target.x, task.target.y = cube.x, cube.y
        self._task_pub.publish(task)
        self._event('assign', id=cube.id, task=self._task_seq,
                    x=round(cube.x, 2), y=round(cube.y, 2), sightings=cube.sightings,
                    dist=round(math.hypot(cube.x - s.x, cube.y - s.y), 2))

    def _summary(self):
        c = self._registry.counts()
        s = self._status
        coll = (f'{s.robot} {s.state} delivered={s.delivered}' if s is not None
                else 'no collector status yet')
        self.get_logger().info(
            f'cubes: {c["unconfirmed"]} unconfirmed, {c["pending"]} pending, '
            f'{c["assigned"]} assigned, {c["collected"]} collected, {c["failed"]} failed '
            f'| {coll}'
        )

    def _publish_obstacles(self):
        """Every confirmed, not-yet-collected cube as a small square of points."""
        offs = np.arange(-1, 2) * CUBE_OBSTACLE_HALF_M
        pts = []
        for cube in self._registry.cubes.values():
            if cube.status == 'collected' or not self._registry.confirmed(cube):
                continue
            for dx in offs:
                for dy in offs:
                    pts.append((cube.x + dx, cube.y + dy, CUBE_OBSTACLE_Z))
        # Published even when empty, so the costmap's buffer holds "no cubes" rather
        # than the last non-empty cloud.
        header = Header(frame_id=MAP_FRAME, stamp=self.get_clock().now().to_msg())
        self._obstacle_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))

        # Robot bodies, each for the other robot (see ROBOT_FOOTPRINT).
        s = self._status
        collector = None
        if (s is not None and s.localised and self._status_time is not None
                and self._now_s() - self._status_time < ROBOT_POSE_MAX_AGE_S):
            collector = (s.x, s.y, s.yaw)
        self._collector_body_pub.publish(
            point_cloud2.create_cloud_xyz32(header, self._body_points(collector)))
        self._leader_body_pub.publish(
            point_cloud2.create_cloud_xyz32(header, self._body_points(self._leader_pose())))

    def _body_points(self, pose):
        """Footprint points of a robot at pose (x, y, yaw) in the map, or [] if None."""
        if pose is None:
            return []
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        fp = self._footprint_pts
        mx = x + c * fp[:, 0] - s * fp[:, 1]
        my = y + s * fp[:, 0] + c * fp[:, 1]
        return [(float(a), float(b), CUBE_OBSTACLE_Z) for a, b in zip(mx, my)]

    def _publish_markers(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for cube in self._registry.cubes.values():
            key = cube.status
            if key == 'pending' and not self._registry.confirmed(cube):
                key = 'unconfirmed'
            m = Marker()
            m.header.frame_id = MAP_FRAME
            m.header.stamp = stamp
            m.ns = 'fleet_cubes'
            m.id = cube.id
            m.type = Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = cube.x, cube.y, 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r, m.color.g, m.color.b = COLOURS[key]
            m.color.a = 0.9
            arr.markers.append(m)
        if arr.markers:
            self._marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
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
