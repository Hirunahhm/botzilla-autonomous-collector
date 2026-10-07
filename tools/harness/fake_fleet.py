#!/usr/bin/env python3
"""
fake_fleet.py — kinematic stand-in for BOTH robots, for testing the fleet DECISION logic.

NOT a simulator and never a source of results (same spirit as fake_nav.py). It stands in
for everything around fleet_manager_node and collector_node:

  leader (un-namespaced)   turns slowly in place at LEADER_POSE, publishing map->base_link
                           on /tf and a /detected_cube for each cube in its camera cone
                           (LEADER_MODE picks other behaviours, see below)
  collector (/bz2)         serves /bz2/navigate_to_pose (drives straight at 0.5 m/s),
                           integrates /bz2/cmd_vel, publishes map->base_link on /bz2/tf
                           and a /bz2/detected_cube for cubes in its cone (2 Hz, like
                           YOLO on the Pi); a cube right in front of the arms is carried
                           until the robot reverses
  map                      a 5 x 4 m empty room on /map, latched

Camera model = the real one: +-28.5 deg, 0.55 m depth blind spot (z = 0 inside it),
1.0 m gate. Run through fleet_harness.sh; it prints where every cube ended up.
"""
import math
import os
import threading
import time

from geometry_msgs.msg import Point, TransformStamped, Twist
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

RES = 0.05
W_M, H_M = 5.0, 4.0
HALF_FOV = math.radians(28.5)
BLIND_M = 0.55
RANGE_M = 1.0
LEADER_POSE = (2.5, 2.0)
LEADER_W = 0.15                     # rad/s
# LEADER_ORBIT=1: the leader drives a slow 0.9 m circle around LEADER_POSE instead of
# turning in place, so it keeps crossing the collector's path (tests the right-of-way
# yield in collector_node).
LEADER_ORBIT = os.environ.get('LEADER_ORBIT') == '1'
ORBIT_R, ORBIT_W = 0.9, 0.12
# LEADER_MODE=inspect: the leader spins (0.6 rad/s) at INSPECT_POSE, 0.7 m from cube
# (1.9, 1.5), for INSPECT_S, then drives to LEADER_AWAY and turns slowly there — run 16's
# opening. That cube must be held back ('deferred') while the leader is beside it and
# assigned once it has gone; the spin must read as MOVING, not PARKED.
# LEADER_MODE=stuck: the leader turns at LEADER_POSE for STUCK_AFTER_S (so it sees the
# cubes), then drives to STUCK_POSE beside the collector's route and stays stuck there,
# with Nav2 feedback reporting a recovery every second. The collector must give it room
# and never plan past it ('leader parked; planning past it' must not appear).
LEADER_MODE = os.environ.get('LEADER_MODE', 'turn')
INSPECT_POSE = (1.9, 2.2)
INSPECT_S = 40.0
LEADER_AWAY = (4.0, 1.0)
STUCK_AFTER_S = 30.0
STUCK_POSE = (1.3, 1.6)
COLLECTOR_START = (0.6, 0.6, 0.0)
CUBES = [(3.2, 2.3), (1.9, 1.5), (2.6, 2.95)]
# PHANTOM=1: a false detection only the leader's camera produces (the collector never
# sees it), to exercise the failure path: FAILED twice, then parked.
PHANTOMS = [(3.3, 1.5)] if os.environ.get('PHANTOM') == '1' else []


def quat(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


class FakeFleet(Node):
    def __init__(self):
        super().__init__('fake_fleet')
        self.lock = threading.Lock()
        self.lyaw = 0.0
        self.x, self.y, self.yaw = COLLECTOR_START
        self.v = self.w = 0.0
        self.cubes = [list(c) for c in CUBES]
        self.carried = None
        self.navigating = False

        q = QoSProfile(depth=1)
        q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        q.reliability = QoSReliabilityPolicy.RELIABLE
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', q)
        self.leader_tf = self.create_publisher(TFMessage, '/tf', 50)
        self.coll_tf = self.create_publisher(TFMessage, '/bz2/tf', 50)
        # The leader's own Nav2 feedback, read by fleet_manager_node (leader_state.py).
        self.leader_fb = self.create_publisher(
            NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback', 10)
        self.t0 = time.monotonic()
        self.lxy = INSPECT_POSE if LEADER_MODE == 'inspect' else LEADER_POSE
        self.recoveries = 0
        self.leader_det = self.create_publisher(Point, '/detected_cube', 10)
        self.coll_det = self.create_publisher(Point, '/bz2/detected_cube', 10)
        self.truth_pub = self.create_publisher(String, '/fake_fleet/truth', 10)
        self.create_subscription(Twist, '/bz2/cmd_vel', self.cmd_cb, 10)
        ActionServer(self, NavigateToPose, '/bz2/navigate_to_pose', self.nav)
        ActionServer(self, BackUp, '/bz2/backup', lambda gh: self.straight(gh, BackUp, -1))
        ActionServer(self, DriveOnHeading, '/bz2/drive_on_heading',
                     lambda gh: self.straight(gh, DriveOnHeading, +1))
        self.yields = 0
        self.orbit = 0.0
        # collector_node only reports IDLE once Nav2's lifecycle manager says active.
        self.create_service(Trigger, '/bz2/lifecycle_manager_navigation/is_active',
                            lambda req, res: setattr(res, 'success', True) or res)
        self.create_timer(0.05, self.step)
        self.create_timer(0.5, self.detect)
        self.create_timer(2.0, self.pub_map)
        self.create_timer(5.0, self.pub_truth)
        self.create_timer(0.2, self.pub_leader_feedback)
        self.pub_map()

    # -- world ---------------------------------------------------------------------

    def pub_map(self):
        w, h = int(W_M / RES), int(H_M / RES)
        a = np.zeros((h, w), dtype=np.int8)
        a[0, :] = a[-1, :] = a[:, 0] = a[:, -1] = 100
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = RES
        m.info.width, m.info.height = w, h
        m.info.origin.orientation.w = 1.0
        m.data = a.flatten().tolist()
        self.map_pub.publish(m)

    def pub_truth(self):
        with self.lock:
            s = ' '.join(f'({c[0]:.2f},{c[1]:.2f})' for c in self.cubes)
            carried = self.carried
        self.truth_pub.publish(String(data=f'cubes {s} carried={carried}'))
        self.get_logger().info(f'truth: cubes {s} carried={carried}')

    def cmd_cb(self, msg):
        with self.lock:
            if not self.navigating:
                self.v, self.w = msg.linear.x, msg.angular.z

    def tf(self, pub, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = quat(yaw)
        pub.publish(TFMessage(transforms=[t]))

    def leader_goal(self, el):
        """(target xy or None, turn rate) for the scripted LEADER_MODE at elapsed el."""
        if LEADER_MODE == 'inspect':
            return (None, 0.6) if el < INSPECT_S else (LEADER_AWAY, LEADER_W)
        if LEADER_MODE == 'stuck':
            return (None, LEADER_W) if el < STUCK_AFTER_S else (STUCK_POSE, 0.0)
        return None, LEADER_W

    def pub_leader_feedback(self):
        el = time.monotonic() - self.t0
        if LEADER_MODE != 'stuck' or el < STUCK_AFTER_S:
            return
        with self.lock:
            arrived = math.hypot(self.lxy[0] - STUCK_POSE[0], self.lxy[1] - STUCK_POSE[1]) < 0.02
        if arrived:
            self.recoveries = int(el - STUCK_AFTER_S)        # one recovery a second
        fb = NavigateToPose.Impl.FeedbackMessage()
        fb.feedback.number_of_recoveries = self.recoveries
        fb.feedback.distance_remaining = 1.0
        self.leader_fb.publish(fb)

    def step(self):
        dt = 0.05
        target, w = self.leader_goal(time.monotonic() - self.t0)
        with self.lock:
            if target is not None:
                dx, dy = target[0] - self.lxy[0], target[1] - self.lxy[1]
                d = math.hypot(dx, dy)
                if d > 0.01:
                    step = min(d, 0.2 * dt)
                    self.lxy = (self.lxy[0] + step * dx / d, self.lxy[1] + step * dy / d)
                    self.lyaw = math.atan2(dy, dx)
                else:
                    self.lxy = target
            self.lyaw = (self.lyaw + w * dt) % (2 * math.pi)
            if LEADER_ORBIT:
                self.orbit = (self.orbit + ORBIT_W * dt) % (2 * math.pi)
            self.yaw += self.w * dt
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
            if self.carried is not None and self.v < -0.01:
                self.carried = None                     # reversing releases the cube
            # Nav2 plans around cubes (the depth camera's low virtual scan marks them),
            # so only the collector's own drive-in can pick one up.
            for i, c in enumerate(self.cubes):
                fx, lx = self.local(c)
                if (self.carried is None and not self.navigating and self.v > 0.01
                        and 0.1 < fx < 0.42 and abs(lx) < 0.12):
                    self.carried = i
            if self.carried is not None:
                c = self.cubes[self.carried]
                c[0] = self.x + 0.3 * math.cos(self.yaw)
                c[1] = self.y + 0.3 * math.sin(self.yaw)
            pose, lyaw = (self.x, self.y, self.yaw), self.lyaw
        self.tf(self.leader_tf, *self.leader_xy(), lyaw)
        self.tf(self.coll_tf, *pose)

    def local(self, c, origin=None):
        ox, oy, oyaw = origin or (self.x, self.y, self.yaw)
        dx, dy = c[0] - ox, c[1] - oy
        return (dx * math.cos(oyaw) + dy * math.sin(oyaw),
                -dx * math.sin(oyaw) + dy * math.cos(oyaw))

    def seen(self, origin, skip=None, cubes=None):
        out = []
        for i, c in enumerate(self.cubes if cubes is None else cubes):
            if i == skip:
                continue
            fx, lx = self.local(c, origin)
            d = math.hypot(fx, lx)
            b = math.atan2(lx, fx)
            if fx <= 0 or abs(b) > HALF_FOV or d > RANGE_M:
                continue
            out.append(Point(x=-math.tan(b) / math.tan(HALF_FOV),
                             z=fx if fx >= BLIND_M else 0.0))
        return out

    def detect(self):
        with self.lock:
            lead = self.seen((*self.leader_xy(), self.lyaw), skip=self.carried)
            lead += self.seen((*self.leader_xy(), self.lyaw), cubes=PHANTOMS)
            coll = self.seen((self.x, self.y, self.yaw), skip=self.carried)
        for p in lead:
            self.leader_det.publish(p)
        for p in coll:
            self.coll_det.publish(p)

    # -- collector Nav2 --------------------------------------------------------------

    def leader_xy(self):
        if not LEADER_ORBIT:
            return self.lxy
        return (LEADER_POSE[0] + ORBIT_R * math.cos(self.orbit),
                LEADER_POSE[1] + ORBIT_R * math.sin(self.orbit))

    def straight(self, gh, action, sign):
        """BackUp / DriveOnHeading: move target.x metres along the heading."""
        dist = abs(gh.request.target.x)
        speed = max(abs(gh.request.speed), 0.05)
        with self.lock:
            self.navigating = True
            self.v = self.w = 0.0
            self.yields += 1
        moved = 0.0
        while moved < dist and rclpy.ok():
            step = min(speed * 0.05, dist - moved)
            with self.lock:
                self.x += sign * step * math.cos(self.yaw)
                self.y += sign * step * math.sin(self.yaw)
            moved += step
            time.sleep(0.05)
        with self.lock:
            self.navigating = False
        self.get_logger().info(f'yield move #{self.yields}: {"back" if sign < 0 else "forward"} {dist:.2f} m')
        gh.succeed()
        return action.Result()

    def nav(self, gh):
        g = gh.request.pose.pose
        gx, gy = g.position.x, g.position.y
        gyaw = 2 * math.atan2(g.orientation.z, g.orientation.w)
        with self.lock:
            self.navigating = True
            self.v = self.w = 0.0
        try:
            while rclpy.ok():
                if gh.is_cancel_requested:
                    gh.canceled()
                    return NavigateToPose.Result()
                with self.lock:
                    dx, dy = gx - self.x, gy - self.y
                    d = math.hypot(dx, dy)
                    if d < 0.05:
                        self.yaw = gyaw
                        break
                    self.yaw = math.atan2(dy, dx)
                    step = min(d, 0.5 * 0.05)
                    self.x += step * math.cos(self.yaw)
                    self.y += step * math.sin(self.yaw)
                time.sleep(0.05)
        finally:
            with self.lock:
                self.navigating = False
        gh.succeed()
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = FakeFleet()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
