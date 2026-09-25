"""
mission_metrics_node.py.

Passive instrumentation for the coverage/exploration experiment. Records what a mission
did — where the robot went, when it changed state, how much of the mapped floor the
camera had actually inspected, and when each ground-truth cube first entered the camera
frustum — as one JSON object per line in a .jsonl file, for offline analysis.

SAFETY PROPERTY, LOAD-BEARING: this node is subscribe-only. It creates NO publishers, NO
action clients, and NO service clients, and it never writes to TF. It cannot alter robot
behaviour by construction, not merely by intent — which is what makes it safe to leave
running during a real hardware mission without qualifying the results. Keep it that way:
if a future change needs this node to publish anything, that belongs in a separate node,
because the moment this one can act, every run it observed becomes a run it may have
influenced.

Why it exists: the experiment's primary metric must be INSPECTION (did the camera's
detection cone ever cover this cube's position, and when), not COLLECTION (did the FSM
successfully pick it up). Collection is gated behind YOLO detection gaps and TARGETING
aborts whose failure rate is large and only loosely related to the exploration policy
being compared — measuring policies by cubes delivered would bury a policy difference
under detector noise. Both are recorded here, separately, so the policy comparison can
use the first and report the second as the practical consequence.

Ground truth: cube positions come from a layout file (see --cube-layout / the
`cube_layout` parameter), in the SAME frame as the robot pose (map). Without one the node
still records pose/state/coverage; it just cannot report per-cube inspection times.

  floor_area_m2: 18.5      # optional: measured arena floor, the coverage denominator
  cubes:
    - {id: 1, x: 2.0, y: 0.5}
    - {id: 2, x: -1.0, y: 1.5}

Coverage denominator: "swept / known free cells" rewards a policy for mapping LESS — the
same floor seen over a smaller map is a bigger percentage. With floor_area_m2 (layout
key, or the arena_floor_m2 parameter, which wins) every coverage record also carries
swept_m2 / floor_area_m2, a denominator fixed for the whole campaign. That is the
figure to compare arms on (research_discussion.md §1.3).

Inspection uses the same geometry as the coverage map: swept_mask.is_in_frustum (range
annulus 0.48-1.0 m, +/-28.5 deg) AND, once /map is available, an unobstructed line of
sight (swept_mask.has_line_of_sight). A cube behind a box inside the cone is not
inspected.

Detection matching: every ranged detection within detect_max_range_m is projected into
the map (cube_detections.project_detection) and matched to the nearest layout cube within
cube_detections.MATCH_RADIUS_M. A match is that cube detected; no match is a false
positive. Paired with cube_inspected this separates the strategy (did the camera point
at it) from the detector (did YOLO fire).

Record types written (each line is a complete JSON object with `t`, seconds since the
node started, and `type`):

  run_start        resolved configuration, once at startup
  pose             x/y/yaw in the map frame plus cumulative distance travelled
  state            mission state and delivered-count, on every change
  coverage         total known-free cells vs. camera-swept cells
  cube_detected    a /detected_cube message the perception stack actually produced,
                   with its map position and matched cube_id (None = false positive)
  cube_first_detected  the FIRST detection matched to each ground-truth cube
  cube_inspected   the FIRST time a ground-truth cube entered the camera frustum
  battery          /battery voltage, at the first message and every battery_period_s
                   (the protocol starts every counted run above a fixed voltage)
  explorer_event   one /explorer/events record from frontier_explorer_node (goal
                   results, stalls, strategy decisions, looks), passed through as-is
  run_end          summary, on clean shutdown

Usage:
  ros2 run botzilla_navigation mission_metrics_node --ros-args
      -p output_path:=run_logs/20260101-120000/metrics.jsonl
      -p cube_layout:=layouts/arena_a.yaml
"""

import json
import math
import os
import time

from botzilla_navigation.cube_detections import (
    match_detection,
    MATCH_RADIUS_M,
    project_detection,
)
from botzilla_navigation.swept_mask import (
    CAMERA_MIN_RANGE_M,
    has_line_of_sight,
    is_in_frustum,
)
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException
import yaml

MAP_FRAME = 'map'
ROBOT_FRAME = 'base_link'

# Defaults mirror swept_mask's camera envelope. They are declared as parameters here
# rather than imported so an analysis re-run can recompute inspection against a different
# envelope (e.g. once the detection-probability-vs-range curve is measured) without the
# recorded data having baked one in — the raw pose stream is kept for exactly that reason.
DEFAULT_HALF_FOV_RAD = 0.497
DEFAULT_MARK_RANGE_M = 1.0
DEFAULT_MIN_RANGE_M = CAMERA_MIN_RANGE_M
# Detections further than this are recorded but never matched: the executor does not
# act on them either (executor_node.cube_max_range_m), and the bearing-plus-depth
# projection error grows with range.
DEFAULT_DETECT_MAX_RANGE_M = 1.0

# /swept_coverage_map encodes free cells as 0 (camera-swept) or 100 (not yet swept), and
# everything else as -1 — see swept_mask.build_coverage_grid_data.
SWEPT_FREE = 0
UNSWEPT_FREE = 100


class MissionMetricsNode(Node):
    def __init__(self):
        super().__init__('mission_metrics_node')

        self.declare_parameter('output_path', '')
        self.declare_parameter('cube_layout', '')
        self.declare_parameter('pose_sample_hz', 10.0)
        self.declare_parameter('coverage_sample_period_s', 2.0)
        self.declare_parameter('camera_half_fov_rad', DEFAULT_HALF_FOV_RAD)
        self.declare_parameter('camera_mark_range_m', DEFAULT_MARK_RANGE_M)
        self.declare_parameter('camera_min_range_m', DEFAULT_MIN_RANGE_M)
        self.declare_parameter('inspection_occlusion', True)
        self.declare_parameter('detect_max_range_m', DEFAULT_DETECT_MAX_RANGE_M)
        self.declare_parameter('match_radius_m', MATCH_RADIUS_M)
        # 0 = take floor_area_m2 from the layout file, if it has one.
        self.declare_parameter('arena_floor_m2', 0.0)
        self.declare_parameter('battery_period_s', 30.0)
        self._battery_period_s = self.get_parameter('battery_period_s').value
        self._battery_first = None
        self._battery_last = None
        self._battery_logged_at = None
        self._stalls = 0
        self._events = 0

        self._half_fov_rad = self.get_parameter('camera_half_fov_rad').value
        self._mark_range_m = self.get_parameter('camera_mark_range_m').value
        self._min_range_m = self.get_parameter('camera_min_range_m').value
        self._inspection_occlusion = bool(self.get_parameter('inspection_occlusion').value)
        self._detect_max_range_m = self.get_parameter('detect_max_range_m').value
        self._match_radius_m = self.get_parameter('match_radius_m').value
        pose_hz = max(0.1, self.get_parameter('pose_sample_hz').value)
        coverage_period = max(0.1, self.get_parameter('coverage_sample_period_s').value)

        self._t0 = time.monotonic()
        self._path = self._resolve_output_path(self.get_parameter('output_path').value)
        # Line-buffered: a run ends with Ctrl+C, so anything still sitting in a block
        # buffer at that moment is lost. Every record is flushed as it is written.
        self._fh = open(self._path, 'a', buffering=1)

        self._floor_area_m2 = None
        self._cubes = self._load_layout(self.get_parameter('cube_layout').value)
        if self.get_parameter('arena_floor_m2').value > 0.0:
            self._floor_area_m2 = float(self.get_parameter('arena_floor_m2').value)
        self._inspected = {}          # cube id -> elapsed seconds first inspected
        self._detected = {}           # cube id -> elapsed seconds first detected
        self._detections_matched = 0
        self._detections_false = 0    # ranged, in range, no cube within match radius
        self._detections_unranged = 0  # z == 0: blind spot or depth failure, no position
        self._false_spots = []        # distinct false-positive positions, [x, y]
        self._latest_map = None
        self._latest_swept_map = None
        self._last_pose = None        # (x, y, yaw)
        self._distance_m = 0.0
        self._state = None
        self._delivered = 0
        self._status_parse_failed = False
        self._latest_coverage = None  # (total_free, swept_free, cell_area_m2)

        # ── Subscriptions only. See the module docstring's safety property. ──
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(
            OccupancyGrid, '/swept_coverage_map', self._coverage_cb, map_qos
        )
        # Only for the inspection line-of-sight test — see the module docstring.
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        self.create_subscription(BatteryState, '/battery', self._battery_cb, 10)
        self.create_subscription(String, '/explorer/events', self._explorer_event_cb, 50)
        self.create_subscription(String, '/mission/status', self._status_cb, 10)
        self.create_subscription(
            Point, '/detected_cube', self._cube_cb, qos_profile_sensor_data
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_timer(1.0 / pose_hz, self._sample_pose)
        self.create_timer(coverage_period, self._sample_coverage)

        self._write('run_start', {
            'output_path': self._path,
            'cube_layout': self.get_parameter('cube_layout').value or None,
            'cube_count': len(self._cubes),
            'camera_half_fov_rad': self._half_fov_rad,
            'camera_mark_range_m': self._mark_range_m,
            'camera_min_range_m': self._min_range_m,
            'inspection_occlusion': self._inspection_occlusion,
            'detect_max_range_m': self._detect_max_range_m,
            'match_radius_m': self._match_radius_m,
            'floor_area_m2': self._floor_area_m2,
            'pose_sample_hz': pose_hz,
            'wall_clock_start': time.time(),
        })
        self.get_logger().info(
            f'Metrics -> {self._path} '
            f'({len(self._cubes)} ground-truth cube(s), '
            f'frustum +/-{math.degrees(self._half_fov_rad):.1f}deg '
            f'@{self._min_range_m}-{self._mark_range_m}m, '
            f'floor area {self._floor_area_m2 or "unset"} m^2)'
        )
        if self._floor_area_m2 is None:
            self.get_logger().warn(
                'No arena floor area — coverage is recorded only as a share of known '
                'free cells, which favours arms that map less. Add floor_area_m2 to the '
                'layout or pass -p arena_floor_m2:=<m^2>.'
            )
        if not self._cubes:
            self.get_logger().warn(
                'No cube layout given — pose/state/coverage will be recorded, but '
                'per-cube inspection times cannot be. Pass -p cube_layout:=<file.yaml>.'
            )

    # ------------------------------------------------------------------ #
    # Setup helpers
    # ------------------------------------------------------------------ #

    def _resolve_output_path(self, configured):
        """Return an absolute .jsonl path, creating its parent directory if needed."""
        if configured:
            path = os.path.abspath(os.path.expanduser(configured))
        else:
            stamp = time.strftime('%Y%m%d-%H%M%S')
            path = os.path.abspath(f'mission_metrics_{stamp}.jsonl')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        return path

    def _load_layout(self, configured):
        """Return [(id, x, y), ...] of ground-truth cube positions in MAP_FRAME.

        A missing or malformed layout is a warning, never a crash: this node must not be
        able to take down a mission that is otherwise running fine.
        """
        if not configured:
            return []
        path = os.path.abspath(os.path.expanduser(configured))
        try:
            with open(path) as fh:
                doc = yaml.safe_load(fh) or {}
            if doc.get('floor_area_m2') is not None:
                self._floor_area_m2 = float(doc['floor_area_m2'])
            cubes = []
            for i, entry in enumerate(doc.get('cubes') or []):
                cubes.append((entry.get('id', i + 1), float(entry['x']), float(entry['y'])))
            return cubes
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().error(
                f'Could not read cube layout {path}: {exc}. Continuing without ground '
                f'truth — per-cube inspection times will be missing from this run.'
            )
            return []

    # ------------------------------------------------------------------ #
    # Writing
    # ------------------------------------------------------------------ #

    def _write(self, record_type, payload):
        record = {'t': round(time.monotonic() - self._t0, 3), 'type': record_type}
        record.update(payload)
        self._fh.write(json.dumps(record) + '\n')

    # ------------------------------------------------------------------ #
    # Subscriptions
    # ------------------------------------------------------------------ #

    def _status_cb(self, msg: String):
        """Record mission state changes, parsed from executor_node's /mission/status.

        The executor publishes a human-readable string; this parses the two fields it
        needs out of it rather than asking the executor for a structured message,
        deliberately, so that adding metrics requires no change at all to the node that
        drives the robot. A format change here degrades to one warning and raw text, not
        to a broken mission.
        """
        state = None
        delivered = self._delivered
        for token in msg.data.split():
            key, _, value = token.partition('=')
            if key == 'state':
                state = value
            elif key == 'delivered':
                try:
                    delivered = int(value)
                except ValueError:
                    pass
        if state is None:
            if not self._status_parse_failed:
                self._status_parse_failed = True
                self.get_logger().warn(
                    f'Could not parse state from /mission/status: {msg.data!r} — '
                    f'recording raw text instead.'
                )
            self._write('state', {'raw': msg.data})
            return
        if state != self._state or delivered != self._delivered:
            self._state = state
            self._delivered = delivered
            self._write('state', {'state': state, 'delivered': delivered})

    def _cube_cb(self, msg: Point):
        """Record a detection the perception stack actually produced.

        This is the COLLECTION-side signal (what the detector saw), as opposed to the
        geometric cube_inspected record (what the camera was pointed at). Comparing the
        two is what quantifies detector loss independently of the exploration policy.

        A ranged detection is projected into the map from the current pose and matched
        against the layout; see the module docstring's "Detection matching".
        """
        record = {
            'x_norm': round(msg.x, 4),
            'z_m': round(msg.z, 4),
            'state': self._state,
            'map_x': None,
            'map_y': None,
            'cube_id': None,
            'match_dist_m': None,
            'in_range': 0.0 < msg.z <= self._detect_max_range_m,
        }
        pose = self._get_robot_pose()
        if msg.z <= 0.0:
            self._detections_unranged += 1
        elif pose is not None:
            mx, my = project_detection(
                pose[0], pose[1], pose[2], msg.x, msg.z, self._half_fov_rad
            )
            record['map_x'], record['map_y'] = round(mx, 4), round(my, 4)
            cube_id, dist = match_detection(mx, my, self._cubes, self._match_radius_m)
            record['match_dist_m'] = round(dist, 4) if dist is not None else None
            if record['in_range']:
                if cube_id is not None:
                    record['cube_id'] = cube_id
                    self._detections_matched += 1
                    self._record_first_detection(cube_id, msg.z, dist)
                elif self._cubes:
                    self._detections_false += 1
                    self._note_false_spot(mx, my)
        self._write('cube_detected', record)

    def _record_first_detection(self, cube_id, range_m, match_dist):
        if cube_id in self._detected:
            return
        elapsed = round(time.monotonic() - self._t0, 3)
        self._detected[cube_id] = elapsed
        self._write('cube_first_detected', {
            'cube_id': cube_id,
            'range_m': round(range_m, 4),
            'match_dist_m': round(match_dist, 4),
            'distance_travelled_m': round(self._distance_m, 4),
            'state': self._state,
        })
        self.get_logger().info(
            f'Cube {cube_id} first detected at t={elapsed:.1f}s '
            f'({range_m:.2f}m, matched within {match_dist:.2f}m)'
        )

    def _note_false_spot(self, x, y):
        for spot in self._false_spots:
            if math.hypot(x - spot[0], y - spot[1]) < self._match_radius_m:
                return
        self._false_spots.append([round(x, 3), round(y, 3)])

    def _map_cb(self, msg: OccupancyGrid):
        self._latest_map = msg

    def _battery_cb(self, msg: BatteryState):
        now = time.monotonic()
        self._battery_last = round(msg.voltage, 2)
        if self._battery_first is None:
            self._battery_first = self._battery_last
        if (self._battery_logged_at is None
                or now - self._battery_logged_at >= self._battery_period_s):
            self._battery_logged_at = now
            self._write('battery', {'voltage': self._battery_last})

    def _explorer_event_cb(self, msg: String):
        try:
            event = json.loads(msg.data)
        except ValueError:
            event = {'raw': msg.data}
        self._events += 1
        if event.get('event') == 'stall':
            self._stalls += 1
        self._write('explorer_event', {'event': event})

    def _coverage_cb(self, msg: OccupancyGrid):
        """Cache swept/total counts from /swept_coverage_map — see SWEPT_FREE."""
        self._latest_swept_map = msg
        swept = 0
        total = 0
        for value in msg.data:
            if value == SWEPT_FREE:
                swept += 1
                total += 1
            elif value == UNSWEPT_FREE:
                total += 1
        self._latest_coverage = (total, swept, msg.info.resolution ** 2)

    # ------------------------------------------------------------------ #
    # Timers
    # ------------------------------------------------------------------ #

    def _sample_pose(self):
        pose = self._get_robot_pose()
        if pose is None:
            return
        x, y, yaw = pose
        if self._last_pose is not None:
            self._distance_m += math.hypot(x - self._last_pose[0], y - self._last_pose[1])
        self._last_pose = pose

        self._write('pose', {
            'x': round(x, 4),
            'y': round(y, 4),
            'yaw': round(yaw, 4),
            'distance_m': round(self._distance_m, 4),
            'state': self._state,
        })
        self._check_inspection(x, y, yaw)

    def _sample_coverage(self):
        if self._latest_coverage is None:
            return
        self._write('coverage', {
            **self._coverage_fields(),
            'distance_m': round(self._distance_m, 4),
        })

    def _coverage_fields(self):
        """Coverage against both denominators — see the module docstring."""
        total, swept, cell_area = self._latest_coverage or (0, 0, 0.0)
        swept_m2 = swept * cell_area
        return {
            'total_free': total,
            'swept_free': swept,
            'swept_fraction': round(swept / total, 4) if total else None,
            'swept_m2': round(swept_m2, 3),
            'known_free_m2': round(total * cell_area, 3),
            'floor_area_m2': self._floor_area_m2,
            'swept_fraction_of_floor': (
                round(swept_m2 / self._floor_area_m2, 4) if self._floor_area_m2 else None
            ),
        }

    # ------------------------------------------------------------------ #
    # Inspection test — the paper's primary metric
    # ------------------------------------------------------------------ #

    def _check_inspection(self, robot_x, robot_y, robot_yaw):
        """Record the first moment each ground-truth cube enters the camera frustum.

        Uses swept_mask.is_in_frustum — the same function mark_swept_cells uses to build
        the coverage map — so an inspection time and a coverage figure from the same run
        are computed from identical geometry. Recorded once per cube: this is "when could
        the robot first have seen it", which is what the exploration policy actually
        controls, independent of whether the detector fired.
        """
        for cube_id, cube_x, cube_y in self._cubes:
            if cube_id in self._inspected:
                continue
            if not is_in_frustum(
                robot_x, robot_y, robot_yaw, cube_x, cube_y,
                self._half_fov_rad, self._mark_range_m, self._min_range_m,
            ):
                continue
            grid = self._latest_map
            if self._inspection_occlusion and grid is not None and not has_line_of_sight(
                grid.data, grid.info.width, grid.info.height, grid.info.resolution,
                grid.info.origin.position.x, grid.info.origin.position.y,
                robot_x, robot_y, cube_x, cube_y,
            ):
                continue
            dx, dy = cube_x - robot_x, cube_y - robot_y
            range_m = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx) - robot_yaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            elapsed = round(time.monotonic() - self._t0, 3)
            self._inspected[cube_id] = elapsed
            self._write('cube_inspected', {
                'cube_id': cube_id,
                'cube_x': cube_x,
                'cube_y': cube_y,
                'range_m': round(range_m, 4),
                'bearing_rad': round(bearing, 4),
                'distance_travelled_m': round(self._distance_m, 4),
                'state': self._state,
            })
            self.get_logger().info(
                f'Cube {cube_id} first inspected at t={elapsed:.1f}s '
                f'({range_m:.2f}m, {math.degrees(bearing):+.1f}deg)'
            )

    def _get_robot_pose(self):
        """Return (x, y, yaw) of the robot in MAP_FRAME, or None if TF isn't ready."""
        try:
            tf = self._tf_buffer.lookup_transform(
                MAP_FRAME, ROBOT_FRAME, rclpy.time.Time()
            )
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (t.x, t.y, yaw)

    # ------------------------------------------------------------------ #
    # Shutdown
    # ------------------------------------------------------------------ #

    def close(self):
        """Write the run summary and close the file. Safe to call more than once."""
        if self._fh.closed:
            return
        self._write('run_end', {
            'delivered': self._delivered,
            'final_state': self._state,
            'distance_m': round(self._distance_m, 4),
            **self._coverage_fields(),
            'cubes_in_layout': len(self._cubes),
            'cubes_inspected': len(self._inspected),
            'inspection_times_s': self._inspected,
            'cubes_detected': len(self._detected),
            'detection_times_s': self._detected,
            'detections_matched': self._detections_matched,
            'detections_false': self._detections_false,
            'detections_unranged': self._detections_unranged,
            'false_positive_spots': self._false_spots,
            'battery_start_v': self._battery_first,
            'battery_end_v': self._battery_last,
            'stalls': self._stalls,
            'explorer_events': self._events,
        })
        self._fh.close()
        self._save_final_maps()

    def _save_final_maps(self):
        """Save the last /map and /swept_coverage_map next to the metrics file.

        map_final.npz holds the grids (int8, row-major, -1 unknown) with their origin
        and resolution, for figures and for re-checking segmentation offline — RTAB-Map
        does not keep the grid when the stack is stopped. Never raises: losing the maps
        must not lose the run_end record written just before.
        """
        if self._latest_map is None:
            return
        try:
            import numpy as np   # only needed here; keeps the node's import cheap
            info = self._latest_map.info
            arrays = {
                'map': np.asarray(self._latest_map.data, dtype=np.int8).reshape(
                    info.height, info.width),
                'origin_x': info.origin.position.x,
                'origin_y': info.origin.position.y,
                'resolution': info.resolution,
            }
            sw = self._latest_swept_map
            if sw is not None and sw.info.width == info.width and \
                    sw.info.height == info.height:
                arrays['swept'] = np.asarray(sw.data, dtype=np.int8).reshape(
                    info.height, info.width)
            np.savez_compressed(
                os.path.join(os.path.dirname(self._path), 'map_final.npz'), **arrays)
        except Exception as exc:  # noqa: B902 — see docstring
            self.get_logger().warn(f'Could not save the final map: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionMetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A run normally ends with Ctrl+C, so the summary record has to be written on the
        # interrupt path, not only on a graceful exit.
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
