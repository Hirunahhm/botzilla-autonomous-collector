import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import cv_bridge
import cv2
import numpy as np

# --- TUNABLE PARAMETERS ---
KP_ANGULAR = 0.8
APPROACH_SPEED = 0.12
SEARCH_ROTATION_SPEED = 0.4
STOP_DISTANCE_M = 0.65  # Stop when red object is closer than this
KINECT_MIN_RANGE_M = 0.55

class RedColorFollower(Node):
    def __init__(self):
        super().__init__('red_color_follower')
        
        self.bridge = cv_bridge.CvBridge()
        self.latest_depth = None
        
        # Subscribe to Kinect streams
        self.create_subscription(Image, '/camera/rgb/image_raw', self.image_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, qos_profile_sensor_data)
        
        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Debug publisher for processed image
        self.debug_pub = self.create_publisher(Image, '/perception/red_detection', 10)
        
        self.get_logger().info("Red Color Follower Node Initialized. Searching for red objects...")

    def depth_callback(self, msg):
        try:
            # mono8 encoding from kinect_bridge (scaled 0-255 from 11-bit)
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth decode error: {e}')

    def get_depth_at(self, cx, cy):
        if self.latest_depth is None:
            return None
            
        h, w = self.latest_depth.shape[:2]
        cx = max(2, min(cx, w - 3))
        cy = max(2, min(cy, h - 3))

        # Sample a 5x5 patch and take the median
        patch = self.latest_depth[cy - 2:cy + 3, cx - 2:cx + 3].flatten().astype(np.float32)
        valid = patch[patch > 0]
        if len(valid) == 0:
            return None

        # Reverse scaling: uint8 0-255 -> 11-bit 0-2047
        raw_val = np.median(valid)
        raw_11bit = (raw_val / 255.0) * 2047.0
        
        if raw_11bit >= 2040:
            return None
            
        # Kinect disparity -> depth formula (meters)
        distance_m = 1.0 / (raw_11bit * -0.0030711016 + 3.3309495161)
        return distance_m

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = cv_image.shape[:2]
            center_x = w / 2.0
            
            # Convert to HSV for color detection
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define Red ranges (two masks needed due to HSV wrap-around)
            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 120, 70])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            
            # Morphological cleanup
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            target_found = False
            msg_twist = Twist()
            
            if contours:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 500: # Min area threshold
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        target_found = True
                        
                        # Normalized offset: -1 (left) to 1 (right)
                        norm_x = (cx - center_x) / center_x
                        
                        # Get distance
                        dist_m = self.get_depth_at(cx, cy)
                        
                        # Control logic
                        if dist_m is not None and dist_m < STOP_DISTANCE_M:
                            self.get_logger().info(f"Target reached! (dist={dist_m:.2f}m). Stopping.", throttle_duration_sec=2.0)
                            msg_twist.linear.x = 0.0
                            msg_twist.angular.z = 0.0
                        else:
                            # Align and move
                            msg_twist.angular.z = -KP_ANGULAR * norm_x
                            
                            # Only move forward if somewhat aligned
                            if abs(norm_x) < 0.2:
                                msg_twist.linear.x = APPROACH_SPEED
                            
                            dist_str = f"{dist_m:.2f}m" if dist_m else "Unknown"
                            self.get_logger().info(f"Heading towards red: offset={norm_x:.2f}, dist={dist_str}", throttle_duration_sec=1.0)
                        
                        # Visual feedback
                        cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
                        if dist_m:
                            cv2.putText(cv_image, f"{dist_m:.2f}m", (cx, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if not target_found:
                self.get_logger().info("Searching for red color...", throttle_duration_sec=2.0)
                msg_twist.angular.z = SEARCH_ROTATION_SPEED
                msg_twist.linear.x = 0.0
            
            # Publish command
            self.cmd_pub.publish(msg_twist)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RedColorFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on shutdown
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
