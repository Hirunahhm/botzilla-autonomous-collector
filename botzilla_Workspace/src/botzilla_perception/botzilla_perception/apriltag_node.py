import faulthandler
faulthandler.enable()   # print C-level stack trace on SIGSEGV

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
import cv_bridge
import cv2
import cv2.aruco as aruco
import numpy as np

# --- CONFIGURATION ---
# AprilTag dictionary: DICT_APRILTAG_36h11 is the standard default
APRILTAG_DICT = aruco.DICT_APRILTAG_36h11

# Only track these specific tag IDs as the drop-off zone markers
TARGET_TAG_IDS = [203, 113]

_FAMILY_NAMES = {
    aruco.DICT_APRILTAG_36h11: 'APRILTAG_36h11',
    aruco.DICT_APRILTAG_25h9:  'APRILTAG_25h9',
    aruco.DICT_APRILTAG_16h5:  'APRILTAG_16h5',
    aruco.DICT_6X6_250:        'ARUCO_6X6_250',
}

# Normalized horizontal offset threshold: below this = tag is "centered enough"
CENTER_THRESHOLD = 0.07


class AprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_node')

        print('[INIT] cv2 version:', cv2.__version__, flush=True)
        self.bridge = cv_bridge.CvBridge()
        print('[INIT] CvBridge OK', flush=True)

        # ArUco detector setup - Search multiple common AprilTag families
        self.families = [
            aruco.DICT_APRILTAG_36h11,
            aruco.DICT_APRILTAG_25h9,
            aruco.DICT_APRILTAG_16h5,
            aruco.DICT_6X6_250
        ]
        print('[INIT] families list OK', flush=True)
        self.dicts = [aruco.getPredefinedDictionary(f) for f in self.families]
        print('[INIT] getPredefinedDictionary OK', flush=True)
        # DetectorParameters_create() is the correct API for OpenCV 4.6 (ARM64).
        # DetectorParameters() constructor exists but attribute assignment segfaults on 4.6.
        # The four custom values below are all OpenCV defaults, so using create() with no
        # modifications is equivalent.
        self.aruco_params = aruco.DetectorParameters_create()
        print('[INIT] DetectorParameters_create() OK', flush=True)

        # Subscribe to Kinect RGB stream
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Publish whether the target drop-off tag is visible (simple bool for state machine)
        self.drop_off_visible_pub = self.create_publisher(Bool, 'drop_off_visible', 10)

        # Publish angular correction for brain_node to steer toward the tag
        # x = normalized horizontal offset of tag (-1.0 left, +1.0 right, 0.0 = centered)
        # z = 0.0 (distance not needed here, brain drives at a fixed speed)
        self.drop_off_pos_pub = self.create_publisher(Point, 'drop_off_pose', 10)

        # Debug annotated image for rqt
        self.debug_pub = self.create_publisher(Image, '/perception/apriltag_image', 10)

        # Enable/disable control — allows FSM to pause tag processing to free CPU
        self._enabled = True
        self.create_subscription(Bool, '/apriltag/enable', self._enable_cb, 10)

        self.get_logger().info(
            f'AprilTag Node started. Tracking Tag ID={TARGET_TAG_IDS} '
            f'(family: DICT_APRILTAG_36h11).'
        )

    def _enable_cb(self, msg: Bool):
        if self._enabled != msg.data:
            self._enabled = msg.data
            self.get_logger().info(
                f'AprilTag processing {"ENABLED" if self._enabled else "DISABLED"}'
            )

    def image_callback(self, msg):
        if not self._enabled:
            # Publish "not visible" so FSM doesn't use stale data
            vis = Bool()
            vis.data = False
            self.drop_off_visible_pub.publish(vis)
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            img_h, img_w = cv_image.shape[:2]
            img_center_x = img_w / 2.0

            # Try detection on normal AND mirrored image
            variations = [("Normal", gray), ("Mirrored", cv2.flip(gray, 1))]
            found = False

            for label, img_to_check in variations:
                for i, aruco_dict in enumerate(self.dicts):
                    corners, ids, _ = aruco.detectMarkers(img_to_check, aruco_dict, parameters=self.aruco_params)
                
                    if ids is not None:
                        detected_ids = ids.flatten().tolist()
                        family_name = _FAMILY_NAMES.get(self.families[i], str(self.families[i]))
                        self.get_logger().info(
                            f'[{label}] Tags detected: {detected_ids} (family: {family_name})',
                            throttle_duration_sec=1.0
                        )

                        aruco.drawDetectedMarkers(cv_image, corners, ids)
                        for idx, tag_id in enumerate(detected_ids):
                            if tag_id in TARGET_TAG_IDS:
                                found = True
                                tag_corners = corners[idx][0]
                                cx = int(np.mean(tag_corners[:, 0]))
                                cy = int(np.mean(tag_corners[:, 1]))

                                # If mirrored, we must un-mirror the cx coordinate
                                if label == "Mirrored":
                                    cx = img_w - cx
                                    self.get_logger().warn("MIRROR EFFECT DETECTED! Un-flipping coordinates.")

                                # Estimate tag height in pixels (average of left and right sides)
                                tag_h_l = np.linalg.norm(tag_corners[0] - tag_corners[3])
                                tag_h_r = np.linalg.norm(tag_corners[1] - tag_corners[2])
                                tag_height = (tag_h_l + tag_h_r) / 2.0

                                # Normalized horizontal offset: -1.0 (left) to +1.0 (right)
                                norm_x = (cx - img_center_x) / img_center_x
                                pos_msg = Point()
                                pos_msg.x = norm_x
                                pos_msg.y = float(cy)
                                pos_msg.z = float(tag_height)
                                self.drop_off_pos_pub.publish(pos_msg)
                                self.get_logger().info(
                                    f'[TARGET] Tag ID={tag_id} detected | x={norm_x:.3f} '
                                    f'({"LEFT" if norm_x < -CENTER_THRESHOLD else "RIGHT" if norm_x > CENTER_THRESHOLD else "CENTRED"})',
                                    throttle_duration_sec=0.5
                                )

                                # Draw debug overlay
                                cv2.circle(cv_image, (cx, cy), 8, (0, 0, 255), -1)
                                label_text = f"ID:{tag_id}  x={norm_x:.2f}"
                                cv2.putText(cv_image, label_text, (cx - 40, cy - 14),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                                break
                    if found: break
                if found: break

            if not found:
                self.get_logger().info(
                    f'[NOT DETECTED] No target AprilTag (IDs {TARGET_TAG_IDS}) in frame.',
                    throttle_duration_sec=2.0
                )

            visible_msg = Bool()
            visible_msg.data = found
            self.drop_off_visible_pub.publish(visible_msg)

            # Publish the debug view
            ros_debug = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.debug_pub.publish(ros_debug)

            cv2.imshow('AprilTag Debug View', cv_image)
            cv2.waitKey(1)

        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'AprilTag detection error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()
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
