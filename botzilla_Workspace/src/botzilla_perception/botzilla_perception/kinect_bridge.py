import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import qos_profile_sensor_data
import freenect
import numpy as np
import threading

# Pi 5 / RP1 USB controller fix: launch this node with
#   LD_PRELOAD=/path/to/noreset.so
# so that libusb_reset_device() is a no-op.  Without it, every freenect
# connection causes a USB device reset, the Pi 5 assigns a new bus address,
# and libfreenect cannot reopen the device at the old address (ENODEV).

# Kinect v1's well-known factory-default RGB intrinsics (uncalibrated for this
# specific unit, but close enough for RTAB-Map's RGB-D registration — these are
# the same defaults used across the ROS/OpenNI ecosystem for 640x480 Kinect v1).
KINECT_FX = 525.0
KINECT_FY = 525.0
KINECT_CX = 319.5
KINECT_CY = 239.5

# Standard Kinect v1 disparity -> depth(meters) formula, matching yolo_node.py's
# get_depth_at() exactly (see that function's comment for the same constants) — that
# function reverses kinect_bridge's separate mono8-rescaled depth stream to recover
# this formula's input; here we apply it directly to the raw 11-bit disparity value,
# no rescale/reverse round-trip needed, since this publisher is built straight from it.
KINECT_DISPARITY_A = -0.0030711016
KINECT_DISPARITY_B = 3.3309495161
KINECT_NO_DATA_THRESHOLD = 2040  # Kinect reports 2047 for pixels with no valid depth

# Must match botzilla_qbot.urdf's <link name="camera_link_optical"/> exactly — the
# URDF defines only one optical frame (RGB and depth treated as co-located, no
# separate depth-camera offset joint), and nothing under this name existed until
# RTAB-Map's TF lookups exposed it: 'camera_color_optical_frame'/
# 'camera_depth_optical_frame' (the previous values here) don't exist anywhere in
# the URDF, so any TF-based consumer would silently fail every lookup.
CAMERA_OPTICAL_FRAME = 'camera_link_optical'


def _build_camera_info():
    info = CameraInfo()
    info.height = 480
    info.width = 640
    info.distortion_model = 'plumb_bob'
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    info.k = [KINECT_FX, 0.0, KINECT_CX, 0.0, KINECT_FY, KINECT_CY, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [KINECT_FX, 0.0, KINECT_CX, 0.0, 0.0, KINECT_FY, KINECT_CY, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


def _disparity_to_meters(data):
    """Convert a raw 11-bit Kinect disparity array into metric depth (32FC1, meters).

    Invalid/no-data pixels (including non-finite results from the formula's pole) are
    set to 0.0 — the standard ROS depth-image convention for "no return", matching
    what depth_image_proc/RTAB-Map expect.
    """
    raw = data.astype(np.float32)
    with np.errstate(divide='ignore', invalid='ignore'):
        depth_m = 1.0 / (raw * KINECT_DISPARITY_A + KINECT_DISPARITY_B)
    invalid = (raw >= KINECT_NO_DATA_THRESHOLD) | ~np.isfinite(depth_m) | (depth_m < 0)
    depth_m[invalid] = 0.0
    return depth_m


class KinectBridge(Node):
    def __init__(self):
        super().__init__('kinect_bridge')

        self.publisher_rgb = self.create_publisher(Image, '/camera/rgb/image_raw', qos_profile_sensor_data)
        # mono8, 0-255 rescaled from the raw 11-bit disparity — kept exactly as-is for
        # yolo_node.py's get_depth_at(), which reverses this specific scaling.
        self.publisher_depth = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
        # 32FC1, real metric depth — for RTAB-Map/SLAM consumers, which need actual
        # depth values, not a compact rescaled preview image.
        self.publisher_depth_meters = self.create_publisher(
            Image, '/camera/depth/image_meters', qos_profile_sensor_data
        )
        self.publisher_camera_info = self.create_publisher(
            CameraInfo, '/camera/camera_info', qos_profile_sensor_data
        )
        self._camera_info_msg = _build_camera_info()

        self.latest_rgb = None
        self.latest_depth = None
        self.latest_depth_meters = None
        # Capture time of the frame currently held in latest_*. Recorded in the freenect
        # callback (when the frame actually arrives), NOT in publish_frames() — stamping
        # at publish time puts the frame up to a full timer period (33ms) in the future
        # relative to its own data, so TF-consuming nodes transform it by a pose the
        # robot only reached afterwards. During a turn that smears the cloud in the map,
        # the same way the lidar's publish-time stamping did (see rplidar_node.py).
        self.latest_rgb_stamp = None
        self.latest_depth_stamp = None
        self.new_rgb_available = False
        self.new_depth_available = False
        self._frames_received = 0

        self.kinect_thread = threading.Thread(target=self.run_camera_loop, daemon=True)
        self.kinect_thread.start()

        self.timer = self.create_timer(1.0 / 30.0, self.publish_frames)
        self.get_logger().info('Decoupled 30FPS Kinect Bridge Started!')

    # --- CAMERA THREAD (Producer) ---

    def video_cb(self, dev, data, timestamp):
        self.latest_rgb = data.tobytes()
        self.latest_rgb_stamp = self.get_clock().now()
        self.new_rgb_available = True
        self._frames_received += 1

    def depth_cb(self, dev, data, timestamp):
        # data is uint16 with 11-bit depth values (0-2047). 2047 = no data.
        # Frames arriving during USB stream re-sync (Stream 70 "Invalid magic") are
        # nearly all-zero and would overwrite the last good frame, causing z=0.00m
        # forever. Drop any frame where fewer than 5% of pixels carry valid depth.
        valid_px = int(np.count_nonzero((data > 0) & (data < 2040)))
        if valid_px < 15000:  # 15k / 307200 ≈ 5%
            return
        scaled = (data.astype(np.float32) / 2047.0 * 255.0).astype(np.uint8)
        self.latest_depth = scaled.tobytes()
        self.latest_depth_meters = _disparity_to_meters(data).tobytes()
        self.latest_depth_stamp = self.get_clock().now()
        self.new_depth_available = True

    def run_camera_loop(self):
        import time
        MAX_RETRIES = 10
        for attempt in range(MAX_RETRIES):
            try:
                self._frames_received = 0
                self.get_logger().info(f'Kinect: starting runloop (attempt {attempt + 1}/{MAX_RETRIES})…')
                freenect.runloop(video=self.video_cb, depth=self.depth_cb)
                if self._frames_received > 0:
                    # Runloop ended after real streaming — reconnect
                    self.get_logger().warn('Kinect runloop exited. Reconnecting in 2 s…')
                    time.sleep(2.0)
                else:
                    # 0 frames: device not accessible (missing LD_PRELOAD? unplugged?)
                    self.get_logger().error(
                        'runloop exited with 0 frames. '
                        'Check that LD_PRELOAD=/path/noreset.so is set and Kinect is plugged in. '
                        f'Retrying in 3 s… ({attempt + 1}/{MAX_RETRIES})'
                    )
                    time.sleep(3.0)
            except Exception as e:
                self.get_logger().error(f'Camera thread error (attempt {attempt + 1}): {e}')
                if attempt < MAX_RETRIES - 1:
                    self.get_logger().warn('Retrying Kinect connection in 2 s…')
                    time.sleep(2.0)
                else:
                    self.get_logger().fatal('Kinect: max retries reached. Is the device plugged in?')

    # --- ROS THREAD (Consumer) ---

    def publish_frames(self):
        if self.new_rgb_available:
            if self.latest_rgb_stamp is None:
                return
            stamp = self.latest_rgb_stamp.to_msg()

            # camera_info is gated on its own subscriber count, independent of
            # whether the image topic itself currently has one — a consumer
            # (e.g. RTAB-Map's synced RGB-D subscriber) may only be watching
            # camera_info at the instant this runs.
            if self.publisher_camera_info.get_subscription_count() > 0:
                self._camera_info_msg.header.stamp = stamp
                self._camera_info_msg.header.frame_id = CAMERA_OPTICAL_FRAME
                self.publisher_camera_info.publish(self._camera_info_msg)

            if self.publisher_rgb.get_subscription_count() > 0:
                msg = Image()
                msg.header.stamp = stamp
                msg.header.frame_id = CAMERA_OPTICAL_FRAME
                msg.height, msg.width, msg.step = 480, 640, 640 * 3
                msg.encoding = 'rgb8'
                msg.data = self.latest_rgb
                self.publisher_rgb.publish(msg)

            self.new_rgb_available = False

        if self.new_depth_available:
            if self.latest_depth_stamp is None:
                return
            stamp = self.latest_depth_stamp.to_msg()

            if self.publisher_depth.get_subscription_count() > 0:
                msg = Image()
                msg.header.stamp = stamp
                msg.header.frame_id = CAMERA_OPTICAL_FRAME
                msg.height, msg.width, msg.step = 480, 640, 640
                msg.encoding = 'mono8'
                msg.data = self.latest_depth
                self.publisher_depth.publish(msg)

            if self.publisher_depth_meters.get_subscription_count() > 0:
                msg = Image()
                msg.header.stamp = stamp
                msg.header.frame_id = CAMERA_OPTICAL_FRAME
                msg.height, msg.width, msg.step = 480, 640, 640 * 4
                msg.encoding = '32FC1'
                msg.data = self.latest_depth_meters
                self.publisher_depth_meters.publish(msg)

            self.new_depth_available = False


def main(args=None):
    rclpy.init(args=args)
    node = KinectBridge()
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
