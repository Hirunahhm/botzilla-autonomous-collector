import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from botzilla_interfaces.msg import CubeBoxes
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
import cv_bridge
import cv2
import numpy as np
import os
import time
from botzilla_perception.cube_depth import depth_at, KINECT_MIN_RANGE_M
from ultralytics import YOLO

# Dynamically locate best.pt model path
from ament_index_python.packages import get_package_share_directory
import os

def resolve_model_path():
    # 0. Explicit override, for a machine whose checkout lives somewhere else (the
    #    collector robot's Pi).
    override = os.environ.get('BOTZILLA_YOLO_MODEL')
    if override and os.path.exists(override):
        return override
    # 1. The repo this node was built from, wherever it is checked out: with
    #    --symlink-install the real path of this file is
    #    <repo>/botzilla_Workspace/src/botzilla_perception/botzilla_perception/.
    #    Then the working directory and its parent (launched from the workspace).
    here = os.path.dirname(os.path.realpath(__file__))
    candidates = [
        os.path.join(here, '..', '..', '..', '..', 'runs/best-fit/best.pt'),
        os.path.join(os.getcwd(), "runs/best-fit/best.pt"),
        os.path.join(os.getcwd(), '..', 'runs/best-fit/best.pt'),
        os.path.join(os.path.expanduser('~'), "Desktop/Projects/sem5/final-project-botzilla/runs/best-fit/best.pt"),
        os.path.join(os.path.expanduser('~'), "Desktop/Bozilla-ws/final-project-botzilla/runs/best-fit/best.pt"),
    ]
    for p in candidates:
        if os.path.exists(p):
            return p
    return candidates[1]

file_path = resolve_model_path()

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.frame_count = 0

        self.bridge = cv_bridge.CvBridge()
        self.latest_depth = None  # Raw depth frame; units depend on the encoding below
        # Set from each depth message. Drives the unit conversion in get_depth_at():
        # 'mono8' on hardware (kinect_bridge's rescaled 11-bit), '32FC1' in sim.
        self.latest_depth_encoding = None

        # mode 'full' (default): RGB + depth in, detected_cube out — the leader's own
        # camera, exactly as before. mode 'boxes': compressed RGB in, pixel boxes out —
        # a second instance under the collector's namespace, so the collector's camera
        # is detected on this GPU instead of the Pi's CPU (botzilla_fleet
        # remote_detection_node does the depth half on the Pi).
        self.declare_parameter('mode', 'full')
        self.mode = self.get_parameter('mode').value
        if self.mode == 'boxes':
            self.create_subscription(
                CompressedImage, 'camera/rgb/compressed', self.boxes_callback,
                qos_profile_sensor_data,
            )
            self.boxes_pub = self.create_publisher(CubeBoxes, 'yolo/boxes', 10)
        else:
            # Subscribe to the Kinect RGB stream
            self.subscription = self.create_subscription(
                Image,
                'camera/rgb/image_raw',
                self.image_callback,
                qos_profile_sensor_data
            )

            # Subscribe to the Kinect Depth stream
            self.subscription_depth = self.create_subscription(
                Image,
                'camera/depth/image_raw',
                self.depth_callback,
                qos_profile_sensor_data
            )

            # Publish annotated image for debugging in rqt_image_view
            self.publisher_annotated = self.create_publisher(
                Image, 'perception/yolo_image', 10)

            # Publish cube position to brain_node: x=normalized horizontal, z=distance in meters
            self.cube_pub = self.create_publisher(Point, 'detected_cube', 10)

        # Load YOLO model
        # Confidence threshold, exposed as a parameter because the right value differs
        # sharply between domains. best.pt was trained on photographs of real cubes, so
        # on hardware it fires confidently and 0.8 rejects false positives well. Gazebo's
        # cubes are flat-shaded untextured boxes with no photographic texture, and the
        # same model tops out around 0.25 on them — measured on a captured sim frame,
        # where the 0.25 detection's box centre (319, 344) matched an independent
        # red-pixel centroid (320, 345) almost exactly, so it is a true positive, just a
        # low-confidence one. Left at 0.8 the node detects nothing at all in sim.
        # Override with:  ros2 run ... --ros-args -p confidence:=0.25
        self.declare_parameter('confidence', 0.8)
        self.confidence = self.get_parameter('confidence').value

        self.model = YOLO(file_path)
        self.get_logger().info(
            f'Loaded model {file_path} (classes={self.model.names}) '
            f'confidence>={self.confidence} mode={self.mode}'
        )

        self.get_logger().info('YOLO Perception Node Initialized. Waiting for video stream...')

    def depth_callback(self, msg):
        """Store the latest depth frame for use in image_callback."""
        try:
            # Use passthrough to keep raw bytes for index-based access. The encoding
            # is kept alongside the frame because the SAME topic carries two very
            # different formats depending on where we're running — see get_depth_at().
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth = depth_image
            self.latest_depth_encoding = msg.encoding
            # Log depth frame stats every 30 frames to confirm valid data is arriving
            if self.frame_count % 30 == 0:
                valid_px = int(np.count_nonzero(depth_image > 0))
                pct = valid_px * 100 // depth_image.size
                max_val = float(np.nanmax(depth_image))
                self.get_logger().info(
                    f'Depth frame [{msg.encoding}]: {valid_px} valid px ({pct}%), '
                    f'max={max_val:.3f}'
                )
        except Exception as e:
            self.get_logger().error(f'Depth decode error: {e}')

    def get_depth_at(self, cx, cy, depth_img):
        """Depth (m) at a box centre, or None — see cube_depth.depth_at."""
        return depth_at(depth_img, cx, cy, getattr(self, 'latest_depth_encoding', 'mono8'))

    def boxes_callback(self, msg):
        """Boxes mode: JPEG in, pixel boxes out; no depth, no annotated image.

        Runs on the leader's GPU for the collector's camera (botzilla_fleet
        remote_detection_node). The output keeps the source image's stamp, so the
        collector can pair the boxes with the depth frame taken at the same moment.
        """
        try:
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn('Undecodable compressed frame', throttle_duration_sec=5.0)
                return
            t0 = time.monotonic()
            results = self.model.predict(
                source=frame, conf=self.confidence, verbose=False, iou=0.5
            )
            out = CubeBoxes()
            out.header = msg.header
            out.height, out.width = frame.shape[:2]
            out.inference_s = time.monotonic() - t0
            boxes = results[0].boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    out.cx.append((x1 + x2) / 2.0)
                    out.cy.append((y1 + y2) / 2.0)
                    out.confidence.append(float(box.conf[0]))
            self.boxes_pub.publish(out)
            self.frame_count += 1
            if self.frame_count % 50 == 0:
                self.get_logger().info(
                    f'boxes mode: {self.frame_count} frames, last inference '
                    f'{out.inference_s * 1000:.0f} ms, {len(out.cx)} box(es)'
                )
        except Exception as e:
            self.get_logger().error(f'Failed to process compressed frame: {e}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img_h, img_w = cv_image.shape[:2]
            img_center_x = img_w / 2.0

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Processing frame #{self.frame_count}. Latest depth: {"Received" if self.latest_depth is not None else "None"}')

            # Run YOLO inference – lower confidence for better detection rate
            results = self.model.predict(
                source=cv_image, conf=self.confidence, verbose=False, iou=0.5
            )

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

            # Only with a display: headless (the Jetson's container, the collector Pi over
            # SSH) imshow aborts the process unless Qt is forced offscreen, and even then
            # it renders a window nobody sees. /perception/yolo_image carries the same view.
            if os.environ.get('DISPLAY') and os.environ.get('QT_QPA_PLATFORM') != 'offscreen':
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