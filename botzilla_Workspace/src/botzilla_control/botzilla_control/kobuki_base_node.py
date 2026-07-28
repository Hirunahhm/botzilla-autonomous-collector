import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math

from .KobukiDriver import Kobuki

# ── Odometry constants ──────────────────────────────────────────────────────
# Kobuki QBot: 52 encoder ticks × 34.02 gear ratio / (π × 0.0688 m wheel diameter)
# ≈ 11724 ticks per metre.  Tune TICKS_PER_M if measured distance doesn't match.
TICKS_PER_M  = 11724.41
WHEEL_BASE_M = 0.230    # 23 cm — must match cmd_vel_callback below

# ── IMU ──────────────────────────────────────────────────────────────────────
# The onboard gyro is rate-only (no absolute-heading register), unlike Gazebo's
# simulated IMU — so orientation is left unset (orientation_covariance[0] = -1,
# the sensor_msgs/Imu convention for "no orientation estimate available").
# Only angular_velocity.z (yaw rate) is populated; that's what the hardware EKF
# config (ekf_hardware.yaml) fuses, letting the filter integrate heading itself
# rather than trusting a fabricated absolute orientation.
YAW_RATE_VARIANCE = 0.02   # rad/s, moderate confidence — tune against measured noise

# Stationary gyro bias calibration: the raw z-axis rate has a small offset even
# at rest. With no absolute-yaw source to anchor it, ekf_hardware.yaml's
# yaw-rate-only fusion integrates that offset without bound — the filter's yaw
# state (and the map frame RTAB-Map registers against) slowly spins in place
# even when the robot never moves, which also shows up as smeared/ghosted map
# layers. Average the first GYRO_CALIB_SAMPLES readings at startup (robot must
# be stationary while the node launches) to get an initial bias estimate.
GYRO_CALIB_SAMPLES = 100   # 50 Hz updates -> 2 s of calibration

# The offset isn't a fixed constant — it's bias *instability*, common on cheap
# MEMS gyros: it wanders slowly even at rest (confirmed on this unit: ~0.01
# rad/s of residual drift remained after the one-shot startup calibration
# above). So keep tracking it: whenever the robot is confirmed stationary,
# slowly adapt the bias estimate toward the current raw reading (EMA). Frozen
# (not updated) while the robot is actually moving, since a real yaw rate
# would otherwise get absorbed into the "bias" and get subtracted back out.
#
# "Confirmed stationary" requires BOTH wheel encoders showing no motion AND the
# last /cmd_vel commanding zero. Encoders alone are not enough: an in-place
# rotation via skid-steering causes bursty, stick-slip wheel motion, so
# individual 20ms ticks can read as near-zero encoder motion even mid-turn —
# without the cmd_vel gate, those false-stationary ticks feed the real turn
# rate into the bias estimate and silently subtract a chunk of it back out,
# which is what caused /odometry/filtered to badly under-report an actual
# ~138 deg commanded turn (measured ~9 deg) during hardware testing.
GYRO_BIAS_EMA_ALPHA = 0.005          # slow adaptation — time-averages over ~tens of seconds
STATIONARY_D_THRESHOLD_M = 0.0005    # per 20ms odom tick (~2.5cm/s) — below encoder noise floor at rest
STATIONARY_DTHETA_THRESHOLD_RAD = 0.001   # per 20ms odom tick


class KobukiBaseNode(Node):
    def __init__(self):
        super().__init__('kobuki_base_node')

        self.get_logger().info('Connecting to Kobuki hardware...')
        self.robot = Kobuki()
        self.robot.play_on_sound()

        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Odometry publisher
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._ox = 0.0
        self._oy = 0.0
        self._ot = 0.0          # heading (rad, CCW+)
        self._prev_L = None     # previous raw 16-bit left  tick
        self._prev_R = None     # previous raw 16-bit right tick
        self._prev_odom_time = None   # rclpy.time.Time of the previous tick, for real dt
        self._stationary = True   # updated each odom tick; read by _imu_update for bias tracking
        self._cmd_vel_zero = True   # updated in cmd_vel_callback; see GYRO_BIAS_EMA_ALPHA comment
        self.create_timer(0.02, self._odom_update)   # 50 Hz

        # IMU publisher
        self._imu_pub = self.create_publisher(Imu, 'imu', 10)
        self._gyro_bias_dps = 0.0
        self._gyro_calib_readings = []
        self._gyro_calibrated = False
        self.create_timer(0.02, self._imu_update)     # 50 Hz

        self.get_logger().info(
            'Kobuki Base Node started. /cmd_vel → motors | encoders → /odom, gyro → /imu')

    # ── Odometry ────────────────────────────────────────────────────────────

    @staticmethod
    def _tick_diff(new, old):
        """Signed delta between two 16-bit unsigned encoder readings (handles rollover)."""
        d = (new - old) & 0xFFFF
        return d if d < 32768 else d - 65536

    def _odom_update(self):
        try:
            enc = self.robot.encoder_data()
        except Exception:
            return   # __basic_sensor not populated yet

        L = enc['Left_encoder']
        R = enc['Right_encoder']
        now = self.get_clock().now()

        if self._prev_L is None:
            self._prev_L, self._prev_R = L, R
            self._prev_odom_time = now
            return

        dl = self._tick_diff(L, self._prev_L) / TICKS_PER_M
        dr = self._tick_diff(R, self._prev_R) / TICKS_PER_M
        self._prev_L, self._prev_R = L, R

        dt = (now - self._prev_odom_time).nanoseconds / 1e9
        self._prev_odom_time = now
        if dt <= 0.0:
            return   # clock didn't advance (or went backwards) — skip this tick

        d      = (dl + dr) / 2.0
        dtheta = (dr - dl) / WHEEL_BASE_M
        self._stationary = abs(d) < STATIONARY_D_THRESHOLD_M and abs(dtheta) < STATIONARY_DTHETA_THRESHOLD_RAD
        self._ox += d * math.cos(self._ot + dtheta / 2.0)
        self._oy += d * math.sin(self._ot + dtheta / 2.0)
        self._ot += dtheta

        msg = Odometry()
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'
        msg.pose.pose.position.x    = self._ox
        msg.pose.pose.position.y    = self._oy
        msg.pose.pose.orientation.z = math.sin(self._ot / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._ot / 2.0)
        # robot_localization's ekf_hardware.yaml fuses ONLY these velocities from this
        # source (never the pose above — wheel odometry pose drifts under slip). Without
        # them the EKF fuses "measured velocity = 0" on every update regardless of real
        # motion, which is why /odometry/filtered was previously observed pinned at the
        # origin even during confirmed real motion.
        msg.twist.twist.linear.x  = d / dt
        msg.twist.twist.angular.z = dtheta / dt
        self._odom_pub.publish(msg)

    # ── IMU (rate gyro only — see YAW_RATE_VARIANCE comment above) ───────────

    def _imu_update(self):
        try:
            gyro = self.robot.gyro_velocity_data()
            z_samples = gyro['angular velocity of z: ']
            if not z_samples:
                return
            yaw_rate_dps = z_samples[-1]
        except Exception:
            return   # __gyro not populated yet

        if not self._gyro_calibrated:
            self._gyro_calib_readings.append(yaw_rate_dps)
            if len(self._gyro_calib_readings) >= GYRO_CALIB_SAMPLES:
                self._gyro_bias_dps = sum(self._gyro_calib_readings) / len(self._gyro_calib_readings)
                self._gyro_calibrated = True
                self.get_logger().info(
                    f'Gyro bias calibrated: {self._gyro_bias_dps:.4f} deg/s (robot must have been stationary)')
            return   # don't publish uncorrected samples during calibration

        if self._stationary and self._cmd_vel_zero:
            self._gyro_bias_dps += GYRO_BIAS_EMA_ALPHA * (yaw_rate_dps - self._gyro_bias_dps)

        yaw_rate_dps -= self._gyro_bias_dps

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # No absolute orientation available from this sensor: element 0 of the
        # covariance is the sensor_msgs/Imu sentinel for "whole field invalid" —
        # unlike a per-axis covariance, -1 here must NOT be set on
        # angular_velocity_covariance below, since z *is* valid data.
        msg.orientation_covariance[0] = -1.0
        # No accelerometer data available from this driver.
        msg.linear_acceleration_covariance[0] = -1.0

        msg.angular_velocity.z = math.radians(yaw_rate_dps)
        msg.angular_velocity_covariance[0] = 1e6   # x: not fused, large variance (not -1 —
        msg.angular_velocity_covariance[4] = 1e6   # y: that would invalidate z too)
        msg.angular_velocity_covariance[8] = YAW_RATE_VARIANCE

        self._imu_pub.publish(msg)

    # ── Velocity command ─────────────────────────────────────────────────────

    def cmd_vel_callback(self, msg):
        """
        Translates ROS 2 Twist messages into Kobuki hardware commands.
        """
        linear_x = msg.linear.x   # Forward/Backward speed (m/s)
        angular_z = msg.angular.z # Turning speed (rad/s)
        self._cmd_vel_zero = (linear_x == 0.0 and angular_z == 0.0)

        wheel_base = 0.230 # 23 cm wheel separation for Kobuki
        
        # Calculate left and right wheel speeds in mm/s
        left_wheel_speed = (linear_x - (angular_z * wheel_base / 2.0)) * 1000.0
        right_wheel_speed = (linear_x + (angular_z * wheel_base / 2.0)) * 1000.0
        
        # Determine the 'rotate' flag based on your driver's logic
        # 1 means pure rotation, 0 means forward/arc 
        rotate_flag = 1 if linear_x == 0.0 and angular_z != 0.0 else 0
        
        # Send to hardware
        self.robot.move(int(left_wheel_speed), int(right_wheel_speed), rotate_flag)
        
        self.get_logger().debug(f'Moving -> L: {left_wheel_speed:.1f}, R: {right_wheel_speed:.1f}, Rot: {rotate_flag}')

def main(args=None):
    rclpy.init(args=args)
    node = KobukiBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Kobuki node...')
        # Stop motors safely (0 left, 0 right, 0 rotate flag)
        node.robot.move(0, 0, 0) 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()