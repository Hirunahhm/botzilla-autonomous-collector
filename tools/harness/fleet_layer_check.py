#!/usr/bin/env python3
"""fleet_layer_check.py — probe for fleet_layer_check.sh (see there)."""
import math, time, numpy as np, rclpy, sys
sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__file__), '..', '..', 'botzilla_Workspace', 'src', 'botzilla_fleet'))
from botzilla_fleet.map_tools import shapes_grid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
q = QoSProfile(depth=1); q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL; q.reliability = QoSReliabilityPolicy.RELIABLE
rclpy.init(); n = Node('deadlock_probe')
opub = n.create_publisher(OccupancyGrid, '/fleet/obstacle_grid', q)
spub = n.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
st = {}
n.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: st.update(cm=m), 10)
FP = ((-0.22, 0.36), (-0.215, 0.215))
TX, TY, TYAW = 0.5, 0.3, 0.3                          # map->odom; base_link = odom origin
ME = (TX, TY)                                         # this robot's centre, in the map
OTHER = (ME[0], ME[1] + 0.215 + 0.15 + 0.215, TYAW)   # side by side, 0.15 m gap
CUBE = (ME[0] + 1.2, ME[1] - 0.8)
def tick():
    r = shapes_grid([(OTHER, FP, 0.5, 45), ((CUBE[0], CUBE[1], 0.0), ((-0.05, 0.05), (-0.05, 0.05)), 0.35, 40)], 0.05)
    data, ox, oy, w, h = r; g = OccupancyGrid(); g.header.frame_id = 'map'; g.header.stamp = n.get_clock().now().to_msg()
    g.info.resolution = 0.05; g.info.width, g.info.height = w, h; g.info.origin.position.x, g.info.origin.position.y = ox, oy
    g.info.origin.orientation.w = 1.0; g.data = data.ravel().tolist(); opub.publish(g)
    s = LaserScan(); s.header.frame_id = 'laser_frame'; s.header.stamp = n.get_clock().now().to_msg()
    s.angle_min, s.angle_max, s.angle_increment = -math.pi, math.pi, math.radians(1)
    s.range_min, s.range_max = 0.15, 12.0; s.ranges = [2.4] * 361; spub.publish(s)
n.create_timer(0.2, tick)
def cost(x, y):
    dx, dy = x - TX, y - TY; c, s_ = math.cos(TYAW), math.sin(TYAW); x, y = c * dx + s_ * dy, -s_ * dx + c * dy
    m = st['cm']; g = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
    return int(g[int((y - m.info.origin.position.y) / m.info.resolution), int((x - m.info.origin.position.x) / m.info.resolution)])
end = time.time() + 8
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
print(f'own centre with the other robot 0.15 m away: cost {cost(*ME)}   (must be < 99 to plan out)')
print('cost from own centre towards the other robot: ' + ' '.join(f'{cost(ME[0], ME[1] + d):3d}' for d in (0, .1, .2, .3, .4, .5, .6, .7, .8)))
print('cube profile 0..0.5 m:                        ' + ' '.join(f'{cost(CUBE[0] + d, CUBE[1]):3d}' for d in (0, .1, .2, .3, .4, .5)))
