"""
Standalone encoder diagnostic for the Kobuki QBot.

Run WITHOUT kobuki_base_node — this node opens the serial port itself.

Test sequence:
  1. Settle 2 s  (robot stationary — verify ticks stay at 0)
  2. Drive FORWARD  0.8 m at 0.10 m/s  (~8 s)
  3. Pause 1.5 s
  4. Drive BACKWARD 0.8 m at 0.10 m/s  (~8 s)
  5. Stop and print summary

Logs every tick:
  raw L/R ticks | ΔL/ΔR per interval | cumulative Δ | estimated pose

Expected ticks for 0.8 m (one way):
  0.8 × 11724 ≈ 9379 ticks per wheel

If your measured ticks differ significantly, update TICKS_PER_M in both
encoder_test_node.py and kobuki_base_node.py.

Launch:
  ros2 launch botzilla_control test_encoders_ssh.launch.py
Monitor:
  ros2 topic echo /encoder/pose   (estimated x/y/theta)
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32MultiArray

from .KobukiDriver import Kobuki

# ── Tunable constants ───────────────────────────────────────────────────────
TICKS_PER_M   = 11724.41   # ~11.7 ticks/mm — adjust after calibration
WHEEL_BASE_M  = 0.230      # 23 cm wheel separation

TEST_DIST_M   = 0.80       # how far to drive each direction
TEST_SPEED_MM = 100        # drive speed in mm/s  (= 0.10 m/s)
LOG_RATE_HZ   = 10         # how often to log/update (Hz)


class EncoderTestNode(Node):
    def __init__(self):
        super().__init__('encoder_test_node')

        self.get_logger().info('EncoderTest: connecting to Kobuki...')
        self.robot = Kobuki()
        self.robot.play_on_sound()
        self.get_logger().info('EncoderTest: connected.')

        # Odometry state
        self._prev_L: int | None = None
        self._prev_R: int | None = None
        self._cum_L  = 0      # total signed tick delta from start
        self._cum_R  = 0
        self._ox     = 0.0
        self._oy     = 0.0
        self._ot     = 0.0    # heading (rad)

        # Test state
        self._phase       = 'SETTLE'
        self._phase_start = None

        drive_t = TEST_DIST_M / (TEST_SPEED_MM / 1000.0)

        # Publishers — useful for rqt_plot
        self._pose_pub  = self.create_publisher(Pose2D, '/encoder/pose',  10)
        self._tick_pub  = self.create_publisher(Int32MultiArray, '/encoder/ticks', 10)

        self.get_logger().info(
            f'EncoderTest | dist={TEST_DIST_M}m | speed={TEST_SPEED_MM}mm/s '
            f'| ~{drive_t:.1f}s each way'
        )
        self.get_logger().info(
            f'EncoderTest | expected ticks per direction: '
            f'{int(TEST_DIST_M * TICKS_PER_M)} '
            f'(TICKS_PER_M={TICKS_PER_M})'
        )
        self.get_logger().info('EncoderTest: settling 2s — ticks should stay ~0...')

        self.create_timer(1.0 / LOG_RATE_HZ, self._tick)

    # ── Helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _diff(new: int, old: int) -> int:
        d = (new - old) & 0xFFFF
        return d if d < 32768 else d - 65536

    def _read_enc(self):
        try:
            e = self.robot.encoder_data()
            return e['Left_encoder'], e['Right_encoder']
        except Exception:
            return None, None

    def _update_odom(self, L: int, R: int):
        if self._prev_L is None:
            self._prev_L, self._prev_R = L, R
            return 0, 0
        dl_t = self._diff(L, self._prev_L)
        dr_t = self._diff(R, self._prev_R)
        self._prev_L, self._prev_R = L, R
        self._cum_L += dl_t
        self._cum_R += dr_t

        dl = dl_t / TICKS_PER_M
        dr = dr_t / TICKS_PER_M
        d  = (dl + dr) / 2.0
        dt = (dr - dl) / WHEEL_BASE_M
        self._ox += d * math.cos(self._ot + dt / 2.0)
        self._oy += d * math.sin(self._ot + dt / 2.0)
        self._ot += dt
        return dl_t, dr_t

    def _transition(self, new_phase: str):
        self.get_logger().info(
            f'[PHASE] {self._phase} → {new_phase} | '
            f'cumL={self._cum_L} cumR={self._cum_R} | '
            f'estL={self._cum_L/TICKS_PER_M:.3f}m estR={self._cum_R/TICKS_PER_M:.3f}m'
        )
        self._phase       = new_phase
        self._phase_start = None

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _tick(self):
        now = self.get_clock().now()
        L, R = self._read_enc()
        if L is None:
            return

        dl_t, dr_t = self._update_odom(L, R)

        # Publish pose + raw ticks
        pose_msg = Pose2D(x=self._ox, y=self._oy, theta=self._ot)
        self._pose_pub.publish(pose_msg)
        ticks_msg = Int32MultiArray()
        ticks_msg.data = [L, R, dl_t, dr_t, self._cum_L, self._cum_R]
        self._tick_pub.publish(ticks_msg)

        if self._phase_start is None:
            self._phase_start = now

        elapsed = (now - self._phase_start).nanoseconds / 1e9
        drive_t = TEST_DIST_M / (TEST_SPEED_MM / 1000.0)

        # ── Log ──
        if self._phase != 'SETTLE':
            self.get_logger().info(
                f'[{self._phase:10s}] '
                f'L={L:5d} R={R:5d} | '
                f'dL={dl_t:+4d} dR={dr_t:+4d} | '
                f'cumL={self._cum_L:+6d} cumR={self._cum_R:+6d} | '
                f'x={self._ox:.3f}m y={self._oy:.3f}m θ={math.degrees(self._ot):.1f}°'
            )

        # ── State machine ──
        if self._phase == 'SETTLE':
            if dl_t != 0 or dr_t != 0:
                self.get_logger().warn(
                    f'SETTLE: unexpected tick delta dL={dl_t} dR={dr_t} '
                    '— check robot is stationary and encoders are reading.'
                )
            if elapsed > 2.0:
                self.get_logger().info('Settle OK. Starting FORWARD drive...')
                self._transition('FORWARD')

        elif self._phase == 'FORWARD':
            self.robot.move(TEST_SPEED_MM, TEST_SPEED_MM, 0)
            if elapsed >= drive_t:
                self.robot.move(0, 0, 0)
                self._transition('PAUSE')

        elif self._phase == 'PAUSE':
            if elapsed > 1.5:
                self.get_logger().info('Starting BACKWARD drive...')
                self._transition('BACKWARD')

        elif self._phase == 'BACKWARD':
            self.robot.move(-TEST_SPEED_MM, -TEST_SPEED_MM, 0)
            if elapsed >= drive_t:
                self.robot.move(0, 0, 0)
                self._transition('DONE')

        elif self._phase == 'DONE':
            est_L = self._cum_L / TICKS_PER_M
            est_R = self._cum_R / TICKS_PER_M
            self.get_logger().info(
                '══ ENCODER TEST SUMMARY ══════════════════════════════',
                throttle_duration_sec=5.0
            )
            self.get_logger().info(
                f'  cumulative ticks  : L={self._cum_L:+}  R={self._cum_R:+}',
                throttle_duration_sec=5.0
            )
            self.get_logger().info(
                f'  estimated travel  : L={est_L:.4f}m  R={est_R:.4f}m',
                throttle_duration_sec=5.0
            )
            self.get_logger().info(
                f'  final pose        : x={self._ox:.4f}m  y={self._oy:.4f}m  '
                f'θ={math.degrees(self._ot):.2f}°',
                throttle_duration_sec=5.0
            )
            self.get_logger().info(
                f'  (ideal: x≈0.0, y≈0.0, θ≈0° after fwd+back)',
                throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = EncoderTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.robot.move(0, 0, 0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
