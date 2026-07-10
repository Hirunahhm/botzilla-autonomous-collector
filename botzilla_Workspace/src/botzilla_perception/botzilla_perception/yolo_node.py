import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
import cv_bridge
import cv2
import numpy as np
import os
from ultralytics import YOLO

# Use a path relative to the user home or a specific workspace location
import os
home = os.path.expanduser('~')
file_path = os.path.join(home, "Desktop/Bozilla-ws/final-project-botzilla/runs/best-fit/best.pt")

# Kinect minimum sensing range (objects closer become 0 or invalid)
KINECT_MIN_RANGE_M = 0.55

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.frame_count = 0

        self.bridge = cv_bridge.CvBridge()
        self.latest_depth = None  # Raw float32 depth frame (480x640), values in mm

        # Subscribe to the Kinect RGB stream
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Subscribe to the Kinect Depth stream
        self.subscription_depth = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            qos_profile_sensor_data
        )

        # Publish annotated image for debugging in rqt_image_view
        self.publisher_annotated = self.create_publisher(Image, '/perception/yolo_image', 10)

        # Publish cube position to brain_node: x=normalized horizontal, z=distance in meters
        self.cube_pub = self.create_publisher(Point, 'detected_cube', 10)

        # Load YOLO model
        self.model = YOLO(file_path)

        self.get_logger().info('YOLO Perception Node Initialized. Waiting for video stream...')

    def depth_callback(self, msg):
        """Store the latest depth frame for use in image_callback."""
        try:
            # mono8 encoding from kinect_bridge: depth scaled to 0-255 (lossy)
            # Use passthrough to keep raw bytes for index-based access
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth = depth_image
            # Log depth frame stats every 30 frames to confirm valid data is arriving
            if self.frame_count % 30 == 0:
                valid_px = int(np.count_nonzero(depth_image > 0))
                pct = valid_px * 100 // depth_image.size
                max_val = int(depth_image.max())
                self.get_logger().info(
                    f'Depth frame: {valid_px} valid px ({pct}%), max={max_val}'
                )
        except Exception as e:
            self.get_logger().error(f'Depth decode error: {e}')

    def get_depth_at(self, cx, cy, depth_img):
        """
        Sample the depth around a bounding box center in a small patch to avoid noise.
        Returns distance in meters, or None if invalid.
        """
        h, w = depth_img.shape[:2]
        # Clamp coordinates to image bounds
        cx = max(2, min(cx, w - 3))
        cy = max(2, min(cy, h - 3))

        # Sample a 5x5 patch and take the median non-zero value
        patch = depth_img[cy - 2:cy + 3, cx - 2:cx + 3].flatten().astype(np.float32)
        valid = patch[patch > 0]
        if len(valid) == 0:
            return None

        # Reverse the kinect_bridge scaling: uint8 0-255 was scaled from 11-bit 0-2047.
        # raw_11bit = (value / 255.0) * 2047.0
        # Convert raw 11-bit Kinect value to meters using the standard disparity formula:
        # depth_m = 1.0 / (raw_11bit * -0.0030711016 + 3.3309495161)
        raw_val = np.median(valid)
        raw_11bit = (raw_val / 255.0) * 2047.0
        if raw_11bit >= 2040:  # Kinect reports 2047 for no-data pixels
            return None
        # Standard Kinect disparity → depth formula
        distance_m = 1.0 / (raw_11bit * -0.0030711016 + 3.3309495161)
        return distance_m

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img_h, img_w = cv_image.shape[:2]
            img_center_x = img_w / 2.0

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Processing frame #{self.frame_count}. Latest depth: {"Received" if self.latest_depth is not None else "None"}')

            # Run YOLO inference – lower confidence for better detection rate
            results = self.model.predict(source=cv_image, conf = 0.8, verbose=False, iou=0.5)

            annotated_image = results[0].plot()

            best_cube = None  # (distance_m, normalized_x)
            best_dist = float('inf')

            boxes = results[0].boxes
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    # Normalized horizontal offset: -1.0 (far left) to +1.0 (far right)
                    norm_x = (cx - img_center_x) / img_center_x

                    # Get depth at bounding box center
                    dist_m = None
                    if self.latest_depth is not None:
                        dist_m = self.get_depth_at(cx, cy, self.latest_depth)

                    # If depth is invalid/too close, mark the cube as "captured" range
                    if dist_m is None or dist_m < KINECT_MIN_RANGE_M:
                        dist_m = 0.0  # Signal to brain: cube is in blind spot (already captured vicinity)

                    # Pick the closest cube
                    if dist_m < best_dist or (dist_m == 0.0 and best_cube is None):
                        best_dist = dist_m
                        best_cube = (dist_m, norm_x)

                    # Draw depth overlay on annotated image
                    label = f"{dist_m:.2f}m" if dist_m > 0 else "< 0.55m"
                    cv2.putText(annotated_image, label, (cx - 20, int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # Publish the best (closest) detected cube
            if best_cube is not None:
                cube_msg = Point()
                cube_msg.x = best_cube[1]    # normalized horizontal offset
                cube_msg.y = 0.0             # unused
                cube_msg.z = best_cube[0]    # distance in meters (0.0 = blind-spot/captured range)
                self.cube_pub.publish(cube_msg)
                self.get_logger().info(f'[PUBLISH] detected_cube: x={cube_msg.x:.2f}, z={cube_msg.z:.2f}m', throttle_duration_sec=1.0)

            # Publish annotated image for debugging
            ros_annotated = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            self.publisher_annotated.publish(ros_annotated)

            cv2.imshow("YOLO Debug View", annotated_image)
            cv2.waitKey(1)

        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()