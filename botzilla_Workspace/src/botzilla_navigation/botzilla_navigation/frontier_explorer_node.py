"""
frontier_explorer_node.py.

Milestone 4 (PHASE_1_IMPLEMENTATION_PLAN.md): autonomous frontier exploration. Watches
RTAB-Map's /map, finds frontiers (free cells bordering unknown space) via
frontier_detection.py, and drives the robot to the nearest one through Nav2's
NavigateToPose action — no manual driving, no scripted goals.

Replanning policy: a new goal is only sent when the explorer is IDLE (no goal currently
active). An in-flight Nav2 goal is never preempted for a newly-found target. This is a
deliberate choice, not the literal "replan on every map update" a naive reading might
suggest: this project's own Nav2 debugging (see PHASE_1_IMPLEMENTATION_PLAN.md's "DWB
rotate-in-place stall" fix) found that repeatedly preempting/resending goals starves real
progress, especially under the arena's low Gazebo real-time factor. Nav2's own recovery
behaviors already handle a stuck/failed goal; this node's only job is deciding whether one
is active.

Evaluation is driven by a periodic timer (EVAL_PERIOD_S), not directly by /map arrival.
/map only republishes when rtabmap's map actually changes, which itself only happens when
the robot moves — so reacting solely to new messages creates a deadlock if the very first
evaluation fails for a transient reason (e.g. TF not ready yet, a few hundred ms after
launch): no further /map messages ever arrive to retry with, since nothing moved to change
the map. The timer always re-evaluates against the most recently received map, so a
transient startup failure self-heals on the next tick instead of stalling exploration
permanently.

Usage:
  ros2 launch botzilla_navigation frontier_explorer.launch.py
"""

from action_msgs.msg import GoalStatus
from botzilla_navigation.frontier_detection import (
    distance,
    find_frontiers,
    grid_to_world,
    select_target,
)
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
import tf2_ros
from tf2_ros import TransformException

# Clusters smaller than this many cells are discarded as noise (isolated frontier pixels
# at map edges, sensor-noise artifacts) rather than sent to as navigation goals.
MIN_FRONTIER_CLUSTER_SIZE = 8

# Skip sending a goal this close to the robot — avoids degenerate short hops when the
# nearest frontier is only marginally outside the robot's own footprint/goal tolerance.
MIN_TARGET_DISTANCE_M = 0.3

# How often to re-evaluate (using the latest cached /map) whether a new goal should be
# sent. Independent of /map's own publish rate — see module docstring.
EVAL_PERIOD_S = 2.0

MAP_FRAME = 'map'
ROBOT_FRAME = 'base_link'


class State:
    IDLE = 'IDLE'
    NAVIGATING = 'NAVIGATING'
    EXPLORATION_COMPLETE = 'EXPLORATION_COMPLETE'


class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('frontier_explorer_node')

        self._state = State.IDLE
        self._goal_handle = None
        self._latest_map = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)

        complete_qos = QoSProfile(depth=1)
        complete_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._complete_pub = self.create_publisher(Bool, '/exploration_complete', complete_qos)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_timer(EVAL_PERIOD_S, self._evaluate)

        self.get_logger().info('frontier_explorer_node started, waiting for /map ...')

    # ------------------------------------------------------------------ #
    # /map callback — just caches the latest message (see module docstring
    # for why evaluation itself is timer-driven, not callback-driven).
    # ------------------------------------------------------------------ #

    def _map_cb(self, msg: OccupancyGrid):
        self._latest_map = msg

    # ------------------------------------------------------------------ #
    # Periodic evaluation — drives the whole state machine
    # ------------------------------------------------------------------ #

    def _evaluate(self):
        if self._state != State.IDLE:
            # Never preempt an in-flight goal, and nothing to do once exploration
            # is complete — see module docstring.
            return
        if self._latest_map is None:
            return

        msg = self._latest_map

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return
        robot_x, robot_y = robot_pose

        clusters = find_frontiers(
            msg.data, msg.info.width, msg.info.height, MIN_FRONTIER_CLUSTER_SIZE
        )
        frontiers_world = [
            grid_to_world(r, c, msg.info.resolution,
                          msg.info.origin.position.x, msg.info.origin.position.y)
            for (r, c, _size) in clusters
        ]

        target = select_target(frontiers_world, robot_x, robot_y)

        if target is None:
            self._state = State.EXPLORATION_COMPLETE
            self._complete_pub.publish(Bool(data=True))
            self.get_logger().info('No frontiers remain — exploration complete.')
            return

        if distance(robot_x, robot_y, target[0], target[1]) < MIN_TARGET_DISTANCE_M:
            self.get_logger().debug('Nearest frontier is too close, waiting for next map update.')
            return

        self.get_logger().info(
            f'{len(clusters)} frontier(s) found, targeting ({target[0]:.2f}, {target[1]:.2f})'
        )
        self._send_goal(target[0], target[1])

    # ------------------------------------------------------------------ #
    # Robot pose via TF (map -> base_link)
    # ------------------------------------------------------------------ #

    def _get_robot_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                MAP_FRAME, ROBOT_FRAME, rclpy.time.Time(), Duration(seconds=0.5)
            )
        except TransformException as ex:
            msg = f'Could not get robot pose ({MAP_FRAME}->{ROBOT_FRAME}): {ex}'
            self.get_logger().warn(msg, throttle_duration_sec=5.0)
            return None
        return t.transform.translation.x, t.transform.translation.y

    # ------------------------------------------------------------------ #
    # Nav2 NavigateToPose action client
    # ------------------------------------------------------------------ #

    def _send_goal(self, x, y):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('navigate_to_pose action server not available yet.')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self._state = State.NAVIGATING
        send_future = self._nav_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'distance_remaining={remaining:.2f}m', throttle_duration_sec=2.0)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the frontier goal.')
            self._state = State.IDLE
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }.get(result.status, f'status={result.status}')
        self.get_logger().info(f'Nav2 goal finished: {status_name}')
        self._goal_handle = None
        self._state = State.IDLE


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
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
