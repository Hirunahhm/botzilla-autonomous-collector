import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
import math

class PerceptionSimulator(Node):
    def __init__(self):
        super().__init__('perception_simulator')
        self.cube_pos = (1.5, 1.5) # Updated from (1.0, 0.0)
        self.publisher_ = self.create_publisher(Point, 'detected_cube', 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info("Perception Simulator Started. Tracking Cube at (1.5, 1.5)")

    def odom_callback(self, msg):
        # Current robot position
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        
        # Get Yaw (heading) from Quaternions
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Calculate absolute angle to cube
        abs_angle_to_cube = math.atan2(self.cube_pos[1] - ry, self.cube_pos[0] - rx)
        
        # Calculate RELATIVE angle (how much the robot needs to turn)
        relative_angle = abs_angle_to_cube - yaw
        
        # Normalize angle to [-pi, pi]
        relative_angle = (relative_angle + math.pi) % (2 * math.pi) - math.pi
        
        dist = math.sqrt((self.cube_pos[0] - rx)**2 + (self.cube_pos[1] - ry)**2)
        
        out = Point()
        out.x = relative_angle # This will now approach 0 as the robot turns toward the cube
        out.y = 0.0
        out.z = dist
        self.publisher_.publish(out)

def main():
    rclpy.init()
    node = PerceptionSimulator()
    rclpy.spin(node)
    rclpy.shutdown()