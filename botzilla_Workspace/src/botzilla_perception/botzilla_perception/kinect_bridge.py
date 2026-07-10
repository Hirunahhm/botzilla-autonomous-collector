import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
import freenect
import numpy as np
import threading

# Pi 5 / RP1 USB controller fix: launch this node with
#   LD_PRELOAD=/path/to/noreset.so
# so that libusb_reset_device() is a no-op.  Without it, every freenect
# connection causes a USB device reset, the Pi 5 assigns a new bus address,
# and libfreenect cannot reopen the device at the old address (ENODEV).

class KinectBridge(Node):
    def __init__(self):
        super().__init__('kinect_bridge')

        self.publisher_rgb = self.create_publisher(Image, '/camera/rgb/image_raw', qos_profile_sensor_data)
        self.publisher_depth = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)

        self.latest_rgb = None
        self.latest_depth = None
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
        if self.new_rgb_available and self.publisher_rgb.get_subscription_count() > 0:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_optical_frame'
            msg.height, msg.width, msg.step = 480, 640, 640 * 3
            msg.encoding = 'rgb8'
            msg.data = self.latest_rgb
            self.publisher_rgb.publish(msg)
            self.new_rgb_available = False

        if self.new_depth_available and self.publisher_depth.get_subscription_count() > 0:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_depth_optical_frame'
            msg.height, msg.width, msg.step = 480, 640, 640
            msg.encoding = 'mono8'
            msg.data = self.latest_depth
            self.publisher_depth.publish(msg)
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
