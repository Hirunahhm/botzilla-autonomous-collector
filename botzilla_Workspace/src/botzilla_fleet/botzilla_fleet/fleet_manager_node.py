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

Also publishes an obstacle grid per robot (the other robot and the known cubes, see
OBSTACLE_GRID_HZ), the collector's cleaned map, and /fleet/cubes (MarkerArray)
for RViz, and logs a one-line summary every SUMMARY_PERIOD_S, plus one JSON line per
event to the run log for analysis.
"""
import json
import math

from botzilla_fleet.cube_registry import CubeRegistry
from botzilla_fleet.map_tools import (
    clear_discs, clear_shape, shapes_grid,
)
from botzilla_interfaces.msg import CollectorStatus, CubeTask
from botzilla_navigation.cube_detections import project_detection
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
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

# Obstacles for each robot's costmaps, as one small OccupancyGrid per robot drawn by the
# 'fleet_layer' (botzilla_coverage_layer in lethal mode, nav2_params.yaml, placed AFTER
# the inflation layer). It repaints its whole previous extent on every message, so a
# robot that moved, or a cube that was collected, is gone at the next update:
#   /fleet/obstacle_grid        for the leader:    the collector + known cubes
#   /<ns>/fleet/obstacle_grid   for the collector: the leader + known cubes other than
#                               the one it is collecting
# Why each is needed:
# - cubes: the leader drove into cubes it had detected. The RPLIDAR scans at 0.24 m (URDF
#   laser_joint), above a cube, the depth camera's low scan only sees from 0.55 m out, and
#   in detect-only mode the executor deliberately ignores detections.
# - robots: neither LiDAR sees the other robot's 0.09 m Kobuki body; at 0.24 m there is
#   only a slim LiDAR housing and camera mount, so returns come and go as they close in —
#   the leader drove into the collector on 2026-10-05.
# Each shape is drawn exactly (a cell is lethal when its centre lies inside the shape, no
# margin) with a soft halo around it, and NOT inflated: lethal is enough to stop either
# robot's footprint (DWB) from ever overlapping it, and the halo makes the planners keep
# their distance when there is room. Inflating the other robot's footprint by 0.45 m
# deadlocked both robots on 2026-10-06: once they were 0.36 m apart each stood inside the
# other's inflated zone, so neither could plan out (16 failed plans, 29 back-ups, 32
# spins on the leader; every delivery released short). Inflated cubes likewise turned a
# cube in a corridor into a wall.
OBSTACLE_GRID_HZ = 5.0
OBSTACLE_GRID_RES = 0.05
CUBE_OBSTACLE_HALF_M = 0.05        # a 0.10 m square per cube
# Halos fade linearly from the value at the shape's edge to a quarter of it at the radius
# (halo 45 -> costmap cost ~113). Never lethal or inscribed: a narrow pass stays possible.
# Cube: 0.35 m, enough that the planner's centre line keeps the ~0.22 m half-width of the
# robot off it when there is room.
CUBE_HALO_RADIUS_M = 0.35
CUBE_HALO_VALUE = 40
# Robots, as measured and unpadded (the 0.05 m padding in their Nav2 footprints is each
# robot's own safety margin, not part of the other robot's body):
# - leader: a bare circular Kobuki since 2026-10-06 (arms moved to the collector),
#   body radius 0.17 m (botzilla_qbot.urdf base_link cylinder);
# - collector: Kobuki with the grabber arms, 0.48 m from arm tips to the back of the
#   chassis and 0.33 m across, base_link at the body centre (nav2_params.yaml's measured
#   derivation): x -0.17..+0.31, y +-0.165.
# 0.5 m halo: the robots give each other room without ever being trapped by it.
# The leader is drawn with LEADER_PADDING_M around its 0.17 m body and a longer halo:
# the leader has right of way (collector_node YIELD_TRIGGER_M), so the collector's
# planner should keep well clear of it in the first place.
LEADER_PADDING_M = 0.10
LEADER_SHAPE = 0.17 + LEADER_PADDING_M               # circle radius, m
LEADER_HALO_RADIUS_M = 0.7
COLLECTOR_SHAPE = ((-0.17, 0.31), (-0.165, 0.165))   # (x min/max, y min/max), base_link
# Each robot's OWN Nav2 footprint (padded): nav2_params.yaml robot_radius for the leader,
# params_rewrite.COLLECTOR_FOOTPRINT for the collector. Nothing in a robot's own grid is
# drawn inside it. A robot cannot be standing on a cube, so a cube mark there is a wrong
# estimate — and a lethal cell inside a robot's footprint makes Nav2 refuse every move,
# even moving away. On 2026-10-06 a cube estimate ended up under the leader (RViz showed
# it on a pink cube it was in reality only next to) and it stayed stuck for 4 minutes.
# The mark stays in the other robot's grid, and comes back in this one once it moves off.
LEADER_NAV_FOOTPRINT = 0.22
# The collector's drop zone is kept out of the LEADER's way: a lethal disc of this radius
# (the collector's footprint reaches 0.35 m from base_link) around the collector's HOME,
# with a soft halo, in the leader's grid only. In run 12 of multi_robot_runs.md the
# leader spent the whole second half within 0.2-0.35 m of that HOME, among the cubes
# already dropped there: the collector's deliveries then failed in the last few
# centimetres, and the leader itself was hemmed in (~220 footprint hits a minute). The
# leader starts 0.64 m from it, outside. If ever caught inside, its own footprint is
# still cleared (LEADER_NAV_FOOTPRINT) so it can drive out.
# 0.25, not 0.45 (run 14, 2026-10-07): the leader starts 0.64 m from the collector's
# HOME, and a 0.45 m lethal disc came within ~0.2 m of its own body at the start; its
# first goals kept failing against it (241 footprint hits in one minute).
DROP_ZONE_RADIUS_M = 0.25

# NO soft halos in the LEADER's grid — bodies only (the collector, the cube cores, the
# drop zone), lethal. The leader has right of way (collector_node YIELD_TRIGGER_M), so it
# only needs to avoid what is actually there; DWB's footprint check still stops it
# touching any of it. With halos the leader kept getting stuck inside them, near the
# collector and near cubes, moving in short bursts (run 14). The collector's grid keeps
# its halos: it is the one meant to keep its distance.
COLLECTOR_NAV_FOOTPRINT = ((-0.22, 0.36), (-0.215, 0.215))
ROBOT_HALO_RADIUS_M = 0.5
ROBOT_HALO_VALUE = 45
# A pose older than this is not drawn: a stale footprint would block empty floor.
ROBOT_POSE_MAX_AGE_S = 1.5

# The collector's map: the leader's /map republished on /<ns>/fleet/map with a disc of
# this radius forced free around the collector's HOME and its current pose. The leader's
# LiDAR maps the parked collector itself as an obstacle; on the raw /map the collector
# started inside a lethal blob (every plan from its start failed, 11 back-up recoveries)
# and its HOME was that same blob, so every delivery failed (2026-10-05). The robot is
# physically there, so that floor cannot be an obstacle for it. 0.5 m covers the
# footprint's farthest corner (0.42 m from base_link) with margin.
COLLECTOR_MAP_CLEAR_M = 0.5
# Republish it at most this often. RTAB-Map updates /map about once a second, and every
# copy makes the collector's global costmap redo its static layer and re-inflate the
# whole map — heavy on the Pi, whose planner then answered too slowly (12 acknowledgement
# time-outs in run 8 of multi_robot_runs.md). The floor plan changes far slower than that.
COLLECTOR_MAP_PERIOD_S = 5.0

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
        self._collector_map_sent = None   # see COLLECTOR_MAP_PERIOD_S

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        self._task_pub = self.create_publisher(CubeTask, f'/{self._ns}/fleet/task', latched)
        self._marker_pub = self.create_publisher(MarkerArray, '/fleet/cubes', 10)
        # Latched (transient local, reliable): the layer subscribes that way, and a
        # volatile publisher is QoS-incompatible with it — DDS then delivers nothing,
        # with only a warning on the publisher's side.
        self._leader_grid_pub = self.create_publisher(
            OccupancyGrid, '/fleet/obstacle_grid', latched)                 # for the leader
        self._collector_grid_pub = self.create_publisher(
            OccupancyGrid, f'/{self._ns}/fleet/obstacle_grid', latched)     # for the collector

        self._leader_pose_pub = self.create_publisher(
            PoseStamped, f'/{self._ns}/fleet/leader_pose', 10)
        self._collector_map_pub = self.create_publisher(
            OccupancyGrid, f'/{self._ns}/fleet/map', latched)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, latched)
        self.create_timer(1.0 / OBSTACLE_GRID_HZ, self._publish_obstacles)
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
            self._collector_map_sent = None   # republish now, with its HOME cleared
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
            if not collected and msg.last_released:
                in_home_zone = msg.home_set and math.hypot(
                    msg.release_x - msg.home_x, msg.release_y - msg.home_y) < self._home_excl
                if in_home_zone:
                    # Left in the drop zone: that is a delivery, whatever the collector
                    # called it. Relocating it there made it an obstacle ON HOME, which
                    # blocked the next deliveries, and kept it pending, so the collector
                    # was sent to look for cubes at its own drop zone (run 12).
                    self._registry.mark_collected(cube_id)
                    self._event('collected_in_home_zone', id=cube_id,
                                x=round(msg.release_x, 2), y=round(msg.release_y, 2))
                else:
                    # The cube is no longer where it was estimated: the collector carried
                    # it and left it here. Without this the registry kept the old
                    # estimate, the obstacle mark stayed on empty floor and the next
                    # attempt went to the wrong spot (multi_robot_runs.md, run 10).
                    self._registry.relocate(cube_id, msg.release_x, msg.release_y)
                    self._event('relocated', id=cube_id, x=round(msg.release_x, 2),
                                y=round(msg.release_y, 2))
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
        """Publish each robot's obstacle grid (see OBSTACLE_GRID_HZ)."""
        assigned = self._current[1] if self._current is not None else None
        h = CUBE_OBSTACLE_HALF_M
        cubes, cubes_but_target = [], []
        for cube in self._registry.cubes.values():
            if cube.status == 'collected' or not self._registry.confirmed(cube):
                continue
            core = ((cube.x, cube.y, 0.0), ((-h, h), (-h, h)))
            cubes.append(core + (0.0, 0))                                  # leader
            if cube.id != assigned:
                cubes_but_target.append(core + (CUBE_HALO_RADIUS_M, CUBE_HALO_VALUE))
        s = self._status
        collector = None
        if (s is not None and s.localised and self._status_time is not None
                and self._now_s() - self._status_time < ROBOT_POSE_MAX_AGE_S):
            collector = (s.x, s.y, s.yaw)
        leader = self._leader_pose()

        def robot(pose, shape, halo=ROBOT_HALO_RADIUS_M):
            return ([(pose, shape, halo, ROBOT_HALO_VALUE)]
                    if pose is not None else [])
        # Empty grids are published too: the layer then repaints, and so clears, wherever
        # the previous grid was.
        drop_zone = []
        if s is not None and s.home_set:
            drop_zone = [((s.home_x, s.home_y, 0.0), DROP_ZONE_RADIUS_M, 0.0, 0)]
        res = OBSTACLE_GRID_RES
        self._leader_grid_pub.publish(self._to_msg(clear_shape(
            shapes_grid(robot(collector, COLLECTOR_SHAPE, halo=0.0) + cubes + drop_zone, res),
            res, leader, LEADER_NAV_FOOTPRINT)))
        # The leader's pose for the collector's right-of-way rule (collector_node
        # YIELD_TRIGGER_M); the collector has no other view of the leader.
        if leader is not None:
            msg = PoseStamped()
            msg.header.frame_id = MAP_FRAME
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y = leader[0], leader[1]
            msg.pose.orientation.z = math.sin(leader[2] / 2.0)
            msg.pose.orientation.w = math.cos(leader[2] / 2.0)
            self._leader_pose_pub.publish(msg)
        self._collector_grid_pub.publish(self._to_msg(clear_shape(
            shapes_grid(robot(leader, LEADER_SHAPE, LEADER_HALO_RADIUS_M) + cubes_but_target,
                        res),
            res, collector, COLLECTOR_NAV_FOOTPRINT)))

    def _to_msg(self, raster):
        """Wrap a (data, ox, oy, w, h) raster, or None, as an OccupancyGrid in the map."""
        msg = OccupancyGrid()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = OBSTACLE_GRID_RES
        msg.info.origin.orientation.w = 1.0
        if raster is not None:
            data, ox, oy, w, h = raster
            msg.info.origin.position.x, msg.info.origin.position.y = ox, oy
            msg.info.width, msg.info.height = w, h
            msg.data = data.ravel().tolist()
        return msg

    def _map_cb(self, msg):
        """Republish the leader's map for the collector, its own floor cleared."""
        now = self._now_s()
        if (self._collector_map_sent is not None
                and now - self._collector_map_sent < COLLECTOR_MAP_PERIOD_S):
            return
        self._collector_map_sent = now
        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        s = self._status
        spots = []
        if s is not None and s.home_set:
            spots.append((s.home_x, s.home_y))
        if (s is not None and s.localised and self._status_time is not None
                and self._now_s() - self._status_time < ROBOT_POSE_MAX_AGE_S):
            spots.append((s.x, s.y))
        clear_discs(grid, (msg.info.origin.position.x, msg.info.origin.position.y),
                    msg.info.resolution, spots, COLLECTOR_MAP_CLEAR_M)
        out.data = grid.flatten().tolist()
        self._collector_map_pub.publish(out)

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
