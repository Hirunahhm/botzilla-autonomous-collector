#!/usr/bin/env python3
"""
fake_nav.py — kinematic stand-in for Nav2 + SLAM, for testing the explorer's DECISION logic.

NOT a simulator and never a source of research results: the fake robot teleports along
its goals at 1 m/s, turns at 0.4 rad/s, never stalls, never slips, and its LiDAR is a
360-ray cast on a fixed two-room map (two ~3 x 3 m rooms joined by a 0.8 m doorway,
with one box). What it is good for: running every search strategy end to end —
decide, drive, look, mark the swept mask, finish a region, move on, finish — in a few
minutes, which catches livelocks and wiring bugs unit tests miss (the planner's
repeated look, the sweep that never ended, the explorer that never left its start).

Serves /map, /global_costmap/costmap, map->base_link TF, and the ComputePathToPose,
NavigateToPose, Spin and BackUp actions. Run through harness.sh.
"""
import math
import threading
import time

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import BackUp, ComputePathToPose, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from scipy import ndimage
from tf2_ros import TransformBroadcaster

RES = 0.05


def world():
    n = 60
    w, h = 2 * n + 3, n + 2
    a = np.zeros((h, w), int)
    a[0, :] = a[-1, :] = a[:, 0] = a[:, -1] = 100
    a[:, n + 1] = 100
    a[h // 2 - 8:h // 2 + 8, n + 1] = 0      # 0.8 m doorway
    a[10:14, 20:24] = 100                     # a box in the left room
    return a


class FakeNav(Node):
    def __init__(self):
        super().__init__('fake_nav')
        self.truth = world()
        self.known = np.full_like(self.truth, -1)
        self.x, self.y, self.yaw = 0.8, 0.8, 0.0
        q = QoSProfile(depth=1)
        q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        q.reliability = QoSReliabilityPolicy.RELIABLE
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', q)
        self.cm_pub = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', q)
        self.tf = TransformBroadcaster(self)
        self.lock = threading.Lock()
        ActionServer(self, ComputePathToPose, 'compute_path_to_pose', self.plan)
        ActionServer(self, NavigateToPose, 'navigate_to_pose', self.nav)
        ActionServer(self, Spin, 'spin', self.spin)
        ActionServer(self, BackUp, 'backup', self.backup)
        self.create_timer(0.05, self.pub_tf)
        self.create_timer(1.0, self.pub_map)

    def lidar(self):
        h, w = self.truth.shape
        r0, c0 = int(self.y / RES), int(self.x / RES)
        for k in range(360):
            a = math.radians(k)
            for st in range(1, 60):
                r = int(round(r0 + math.sin(a) * st))
                c = int(round(c0 + math.cos(a) * st))
                if not (0 <= r < h and 0 <= c < w):
                    break
                self.known[r, c] = self.truth[r, c]
                if self.truth[r, c] >= 65:
                    break

    def grid(self, data):
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = RES
        g.info.width, g.info.height = data.shape[1], data.shape[0]
        g.data = data.ravel().astype(int).tolist()
        return g

    def pub_map(self):
        with self.lock:
            self.lidar()
            self.map_pub.publish(self.grid(self.known))
            occ = self.known >= 65
            d = ndimage.distance_transform_edt(~occ) * RES
            cm = np.where(self.known < 0, -1, 0)
            cm = np.where(d < 0.2, 99, cm)
            cm = np.where(occ, 100, cm)
            self.cm_pub.publish(self.grid(cm))

    def pub_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z = math.sin(self.yaw / 2)
        t.transform.rotation.w = math.cos(self.yaw / 2)
        self.tf.sendTransform(t)

    def plan(self, gh):
        gh.succeed()
        r = ComputePathToPose.Result()
        r.path.poses = [PoseStamped()]
        return r

    def nav(self, gh):
        p = gh.request.pose.pose
        tx, ty = p.position.x, p.position.y
        tyaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
        steps = max(1, int(math.hypot(tx - self.x, ty - self.y) / 0.05))
        sx, sy = self.x, self.y
        if steps > 1:
            self.yaw = math.atan2(ty - sy, tx - sx)
        for k in range(1, steps + 1):
            if gh.is_cancel_requested:
                gh.canceled()
                return NavigateToPose.Result()
            self.x = sx + (tx - sx) * k / steps
            self.y = sy + (ty - sy) * k / steps
            time.sleep(0.05)
        self.yaw = tyaw
        gh.succeed()
        return NavigateToPose.Result()

    def spin(self, gh):
        ang = gh.request.target_yaw
        for _ in range(max(1, int(abs(ang) / 0.04))):
            if gh.is_cancel_requested:
                gh.canceled()
                return Spin.Result()
            self.yaw += math.copysign(0.04, ang)
            time.sleep(0.1)
        gh.succeed()
        return Spin.Result()

    def backup(self, gh):
        d = gh.request.target.x   # negative = reverse
        for _ in range(max(1, int(abs(d) / 0.05))):
            self.x += math.cos(self.yaw) * math.copysign(0.05, d)
            self.y += math.sin(self.yaw) * math.copysign(0.05, d)
            time.sleep(0.05)
        gh.succeed()
        return BackUp.Result()


def main():
    rclpy.init()
    ex = MultiThreadedExecutor(4)
    ex.add_node(FakeNav())
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
