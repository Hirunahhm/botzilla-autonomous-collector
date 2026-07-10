import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool

# --- TUNABLE PARAMETERS ---
SEARCH_ROTATION_SPEED = 0.3
FOLLOW_LINEAR_SPEED = 0.08
KP_ANGULAR = 0.6
ALIGNMENT_THRESHOLD = 0.1  # normalized units
STOP_HEIGHT = 300      # pixel height when we should stop (approx 0.4-0.5m)

class TagFollower(Node):
    def __init__(self):
        super().__init__('tag_follower')
        
        self.state = "SEARCHING"
        self.tag_visible = False
        self.tag_pose = None
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.create_subscription(Bool, 'drop_off_visible', self.visible_callback, 10)
        self.create_subscription(Point, 'drop_off_pose', self.pose_callback, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Tag Follower Test Node Started. Searching for Tag...")

    def visible_callback(self, msg):
        self.tag_visible = msg.data

    def pose_callback(self, msg):
        self.tag_pose = msg

    def control_loop(self):
        msg = Twist()
        
        if not self.tag_visible:
            self.state = "SEARCHING"
            msg.angular.z = SEARCH_ROTATION_SPEED
            msg.linear.x = 0.0
            self.get_logger().info("SEARCHING: Rotating to find Tag...", throttle_duration_sec=2.0)
        else:
            if self.tag_pose is not None:
                error_x = self.tag_pose.x  # -1.0 to 1.0
                tag_height = self.tag_pose.z
                
                if tag_height > STOP_HEIGHT:
                    self.state = "ARRIVED"
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.get_logger().info(f"ARRIVED: Tag is close (height={tag_height:.1f}). Stopping.", throttle_duration_sec=2.0)
                else:
                    self.state = "FOLLOWING"
                    # Steer toward center
                    msg.angular.z = -KP_ANGULAR * error_x
                    
                    # Only move forward if somewhat aligned
                    if abs(error_x) < ALIGNMENT_THRESHOLD * 2:
                        msg.linear.x = FOLLOW_LINEAR_SPEED
                        self.get_logger().info(f"FOLLOWING: x={error_x:.2f}, height={tag_height:.1f}", throttle_duration_sec=2.0)
                    else:
                        msg.linear.x = 0.0
                        self.get_logger().info(f"ALIGNING: Centering on Tag (offset={error_x:.2f})", throttle_duration_sec=2.0)
        
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TagFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors BEFORE destroying node
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
