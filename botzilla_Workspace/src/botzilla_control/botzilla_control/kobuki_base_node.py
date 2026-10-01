import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Int16MultiArray
import math
import time

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

# ── Closed-loop velocity control ─────────────────────────────────────────────
# This base has a large, purely mechanical deadband: measured on this unit (odom over a
# 3 s constant command with nothing else driving cmd_vel), a commanded yaw rate <= 0.12
# rad/s produces NO motion at all (0-2% of commanded), 0.15 rad/s only ~10%, and 0.40
# rad/s only ~0.22 rad/s actual (56%). Linear below ~0.04 m/s barely creeps.
#
# That deadband used to be worked around upstream, by flooring DWB's output
# (min_speed_theta 0.2 in nav2_params.yaml) so it never asked for anything the base
# would ignore. The floor made the robot move, but it also made the set of reachable
# commands effectively {0, +/-0.2 ... +/-0.4} — the robot could not rotate gently at
# all, so every heading correction was a step input. That is what the visible jerking
# was, and it is also why the robot oscillated instead of converging (24 angular sign
# flips measured in 80 s).
#
# The fix belongs here, not in the planner: close the loop on the sensors this node
# already owns, so a commanded velocity is *delivered* and the planner is free to ask
# for fine corrections. Yaw uses the bias-calibrated rate gyro (the same 50 Hz signal
# already published on /imu); forward speed uses the encoder-derived linear velocity
# computed in _odom_update.
#
# Structure: cmd_vel_callback now only stores the setpoint. A 50 Hz control timer does
# the actual hardware write, so correction continues between (and after) cmd_vel
# messages instead of only on arrival.
CONTROL_PERIOD_S = 0.02      # 50 Hz — matches the odom/IMU feedback rate

# ── Why feedforward inversion rather than pure PI ────────────────────────────
# A deadband is a STATIC nonlinearity, so the right primary correction is a static inverse
# applied feedforward — not integral action. Measured here with PI only (gyro step response
# to a commanded 0.20 rad/s, 0.2 s bins): steady state reached 0.180 rad/s (90% — accurate),
# but rise time was ~1.8 s and the signal rippled over a 0.316 rad/s spread, briefly even
# reversing sign. DWB re-decides at 20 Hz, so a base needing ~2 s to reach a commanded rate
# cannot track it at all; that lag is itself a source of Nav2-level oscillation, and the
# ripple is the integrator chasing stick-slip.
#
# So: invert the measured deadband curve directly (instant, no lag, no ripple), and keep
# only a small integral term to trim residual error. The base's response above the deadband
# is close to linear, so the model is
#     actual ~= gain * (|command| - deadband)     for |command| > deadband
# and the inverse applied to a setpoint s is
#     |output| = deadband + |s| / gain
#
# Exposed as ROS parameters, not constants: these are per-unit physical measurements
# (they will differ on another Kobuki, and drift as the gearbox wears), and tuning them
# by rebuilding is slow. Override at launch, e.g.
#   ros2 run botzilla_control kobuki_base_node --ros-args -p yaw_ff_gain:=0.62
# Yaw values are MEASURED on this unit, not guessed. Open-loop sweep with all correction
# disabled (ff/kp/ki all 0), 5 s hold per point, first 2 s of transient discarded:
#
#     cmd  0.10  0.15  0.20  0.25  0.30  0.40   rad/s
#     act  0.001 0.023 0.076 0.120 0.152 0.251  rad/s
#
# Least-squares fit: actual = 0.891 * (cmd - 0.121), R^2 = 0.997 — the deadband model is
# essentially exact for this base, and the slope above the deadband is near unity.
YAW_FF_DEADBAND_DEFAULT = 0.121  # rad/s — below this, commanded yaw produces no motion
YAW_FF_GAIN_DEFAULT = 0.891     # measured slope above the deadband

# Linear axis, measured the same way (short forward bursts with reverse returns, to stay
# inside a ~1.5 m corridor; 3.6 s per point, first 1.2 s of transient discarded):
#
#     cmd  0.05  0.08  0.10  0.13  0.16  0.20   m/s
#     act  0.033 0.056 0.082 0.101 0.126 0.157  m/s
#
# Least-squares fit: actual = 0.830 * (cmd - 0.009), R^2 = 0.995.
#
# Note how much SMALLER the linear deadband is than the yaw one (0.009 m/s vs 0.121
# rad/s). That is physically sensible: driving forward turns both wheels the same way,
# whereas rotating in place has to break stiction on both wheels in OPPOSITE directions.
# It also corrected a bad guess — these were previously estimated at deadband 0.04 /
# gain 0.90, and that 0.04 offset (4.4x too large) made the feedforward over-command by
# ~38% at low speed, producing a surge-then-settle that was felt as residual jerk.
LIN_FF_DEADBAND_DEFAULT = 0.009  # m/s, measured
LIN_FF_GAIN_DEFAULT = 0.830      # measured slope

# Trim gains only — the feedforward does the heavy lifting, and these correct just the
# residual the static model misses. Kept small because the feedback is genuinely noisy:
# measured open loop at a delivered ~0.17 rad/s, the gyro reports stdev 0.084 rad/s (half
# the mean) with a 0.349 rad/s spread that briefly reverses sign. That is real stick-slip
# in the drivetrain at low speed, not sensor error and not controller-induced — the
# closed-loop spread at the same delivered rate was 0.298, i.e. slightly BETTER than open
# loop. No gain schedule can remove it, so don't try: large gains here would only inject
# that noise straight into the motor command.
#
# ki lowered 0.40 -> 0.15 and the clamp tightened (see YAW_RATE_I_CLAMP) after observing
# the integrator slowly wind past target: with ki 0.40 the delivered rate crept from 0.15
# at 0.6 s up to 0.24 by 4.8 s against a 0.20 setpoint — a 20% overshoot arriving so slowly
# that it reads as drift rather than oscillation.
YAW_RATE_KP_DEFAULT = 0.20
YAW_RATE_KI_DEFAULT = 0.15
LIN_VEL_KP_DEFAULT = 0.20
LIN_VEL_KI_DEFAULT = 0.15

# Anti-windup: cap the integrator's authority. Tightened from 0.35 once the feedforward
# took over the deadband: the integrator no longer has to cover the whole shortfall, only
# the few percent the static model misses, so a large clamp buys nothing and actively hurt
# — it was what let the slow 20% overshoot described above accumulate. Small enough, too,
# that a robot with blocked wheels cannot wind up and then lurch when it frees.
YAW_RATE_I_CLAMP = 0.10      # rad/s
LIN_VEL_I_CLAMP = 0.05       # m/s

# Feedback below these magnitudes is treated as noise rather than real motion, so the
# integrator doesn't chase sensor jitter while genuinely commanded to hold still.
YAW_RATE_DEADZONE = 0.02     # rad/s
LIN_VEL_DEADZONE = 0.01      # m/s

# ── Feedback filtering ───────────────────────────────────────────────────────
# The encoder-derived velocity (d/dt in _odom_update) is badly behaved as a control
# signal, because we poll the Kobuki at 50 Hz but its encoder registers refresh more
# slowly. Many polls therefore return an UNCHANGED tick count -> d = 0 -> velocity 0,
# and then one poll catches the whole accumulated movement over a very short dt and
# d/dt explodes.
#
# Measured while driving at a commanded 0.05 m/s: median 0.000 m/s but mean 0.036 m/s,
# peaks to 0.35 m/s, and /odom inter-arrival median 19.9 ms with a minimum of 0.64 ms
# (16 of 259 intervals under 5 ms). The signal is bimodal — zeros punctuated by spikes —
# rather than noisy about the truth. Its MEAN is right, so it is unbiased and a
# low-pass filter recovers the real value cleanly.
#
# Unfiltered, those spikes land directly in the P term and the integrator, which is a
# source of translational jerk that no amount of deadband compensation can remove.
#
# Filtering is applied ONLY to the controller's copy of the feedback. The values
# published on /odom are deliberately left raw: robot_localization has its own noise
# model and covariances tuned against that raw signal (ekf_hardware.yaml), and quietly
# changing the statistics of a topic other nodes consume would be a much wider change
# than this fix warrants.
#
# alpha 0.25 at 50 Hz is a ~0.12 s time constant: long enough to bridge the runs of
# zeros, short enough that the added phase lag stays well inside what the feedforward
# (which needs no feedback at all) already covers.
MEAS_FILTER_ALPHA = 0.25

# Guard against the small-dt division itself. Below this the sample is discarded rather
# than filtered, because a 0.64 ms interval carrying a whole 20 ms of accumulated ticks
# is not a measurement of anything — it is an artifact of the polling mismatch.
MIN_ODOM_DT_S = 0.005

# Sanity bound on how stale a serial packet may be before _capture_stamp stops trusting
# the age and falls back to "now". Generous next to the ~20ms packet period and the ~20ms
# blocking read, so it only trips on a genuinely wedged reader or a monotonic-clock
# surprise, never in normal operation.
MAX_CAPTURE_AGE_S = 0.5

# Guard against a corrupted encoder tick. _tick_diff correctly unwraps a genuine 16-bit
# rollover, but has no way to tell a real reading from a garbage one (serial noise
# desyncing the Kobuki's binary protocol is the leading suspect) — it will happily
# report ~18 m/s from a single bad sample, five orders of magnitude past anything the
# robot can actually do (max_vel_x is 0.2). Measured live: one such spike reached
# meas=18.114 and drove the PI controller to command L=-3541/R=-3562 mm/s against a
# normal range of ~100-300 — a real, physical "sudden speed then drop" at the wheels,
# invisible to anything watching /cmd_vel because it originates inside this control
# loop, after the smoother. The same bad sample was also being integrated permanently
# into _ox/_oy/_ot (dead-reckoning never self-corrects) and published on /odom, which
# is exactly the mechanism behind RTAB-Map occasionally registering an offset/ghosted
# duplicate of an already-mapped room — its pose input just jumped 0.36 m in one 20 ms
# tick. Sized well above any real transient (5x max_vel_x) so it only catches readings
# that are unambiguously impossible, not aggressive-but-real motion.
MAX_PLAUSIBLE_SPEED_MPS = 1.0

# Safety watchdog. The old passthrough had none: the last command was latched into the
# motors forever, so if whatever was publishing cmd_vel died mid-drive the robot kept
# going. Stop if no cmd_vel arrives within this window.
CMD_VEL_TIMEOUT_S = 0.5


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

        # ── Drivetrain health ────────────────────────────────────────────────
        # The base already reports both of these on every basic-sensor packet and
        # nothing was surfacing them, which left a real failure mode invisible: the
        # yaw feed-forward below models this unit's mechanical deadband as
        # actual = YAW_FF_GAIN * (cmd - YAW_FF_DEADBAND), fitted at R^2 = 0.997, but
        # that fit is only valid for the battery state it was measured at. Motor
        # torque falls as the pack drains, which raises effective stiction, so the
        # curve silently stops describing the hardware and commanded rotation is
        # under-delivered. Measured live on 2026-09-06: a commanded 0.400 rad/s came
        # out of the base at 0.092 rad/s with the whole software chain verified
        # faithful end to end, and there was no way to correlate it against voltage.
        #
        # PWM matters as much as voltage here, because together they separate the two
        # explanations: high PWM with little motion means the wheels are loaded or
        # stuck, while low PWM means the driver never asked for enough in the first
        # place. 5 Hz is plenty for both — they are trend signals, not control inputs.
        self._battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self._pwm_pub = self.create_publisher(Int16MultiArray, 'wheel_pwm', 10)
        self.create_timer(0.2, self._diagnostics_update)   # 5 Hz

        # ── Closed-loop velocity control (see CONTROL_PERIOD_S comment block) ──
        # Declared as parameters so the deadband curve can be re-measured and tuned without
        # a rebuild — set feedforward gains to 0 and kp/ki to 0 to characterise open loop.
        self.declare_parameter('yaw_ff_deadband', YAW_FF_DEADBAND_DEFAULT)
        self.declare_parameter('yaw_ff_gain', YAW_FF_GAIN_DEFAULT)
        self.declare_parameter('lin_ff_deadband', LIN_FF_DEADBAND_DEFAULT)
        self.declare_parameter('lin_ff_gain', LIN_FF_GAIN_DEFAULT)
        self.declare_parameter('yaw_kp', YAW_RATE_KP_DEFAULT)
        self.declare_parameter('yaw_ki', YAW_RATE_KI_DEFAULT)
        self.declare_parameter('lin_kp', LIN_VEL_KP_DEFAULT)
        self.declare_parameter('lin_ki', LIN_VEL_KI_DEFAULT)

        # Turn radius below which the driver pivots instead of arcing — see the
        # rotate_flag block in _control_update for why the distinction matters.
        # Exposed as a parameter so the threshold can be A/B'd against the old
        # behaviour on the same battery state without a rebuild: setting it to a
        # near-zero epsilon reproduces the original `linear == 0.0` test exactly,
        # because a pure-rotation command has a turn radius of 0 and everything
        # else has a radius comfortably above any epsilon.
        self.declare_parameter('pivot_radius_m', WHEEL_BASE_M / 2.0)

        self._cmd_lin = 0.0          # setpoint from cmd_vel
        self._cmd_ang = 0.0
        self._cmd_vel_stamp = None   # for the watchdog
        self._meas_lin = 0.0         # encoder-derived, set in _odom_update
        self._meas_ang = 0.0         # gyro-derived, set in _imu_update
        self._i_lin = 0.0            # integrator accumulators
        self._i_ang = 0.0
        self._watchdog_tripped = False
        self.create_timer(CONTROL_PERIOD_S, self._control_update)   # 50 Hz

        self.get_logger().info(
            'Kobuki Base Node started. /cmd_vel → closed-loop (gyro + encoder) → motors '
            '| encoders → /odom, gyro → /imu')

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
        # Capture time, not poll time — and here it sets dt as well as the stamp, so it
        # decides what velocity this node reports. The encoder delta spans the interval
        # between the PACKETS the two readings came from; dividing it by the interval
        # between TIMER FIRINGS is only the same number while the two rates agree. When
        # they do not, this timer polls the same packet twice: the first poll sees delta=0
        # and the next sees two packets' worth of ticks, still divided by one timer period
        # — i.e. an apparent doubling. That is exactly the signature that trips
        # MAX_PLAUSIBLE_SPEED_MPS below, so the "Rejecting implausible encoder tick"
        # warnings were partly an artifact of this mismatch rather than corrupted reads.
        # Timing off capture makes it self-correcting: re-reading one packet yields dt<=0
        # and is skipped outright instead of inventing a velocity.
        now = self._capture_stamp(self.robot.packet_monotonic)

        if self._prev_L is None:
            self._prev_L, self._prev_R = L, R
            self._prev_odom_time = now
            return

        dl = self._tick_diff(L, self._prev_L) / TICKS_PER_M
        dr = self._tick_diff(R, self._prev_R) / TICKS_PER_M

        dt = (now - self._prev_odom_time).nanoseconds / 1e9
        if dt <= 0.0:
            self._prev_L, self._prev_R = L, R
            self._prev_odom_time = now
            return   # clock didn't advance (or went backwards) — skip this tick

        d      = (dl + dr) / 2.0
        dtheta = (dr - dl) / WHEEL_BASE_M

        if abs(d / dt) > MAX_PLAUSIBLE_SPEED_MPS:
            # Deliberately do NOT advance _prev_L/_prev_R/_prev_odom_time — see
            # MAX_PLAUSIBLE_SPEED_MPS. The next good reading then diffs against the last
            # known-good baseline, correctly folding in whatever real motion happened
            # during the dropped tick instead of losing it or baking in the glitch.
            self.get_logger().warn(
                f'Rejecting implausible encoder tick: {d / dt:.2f} m/s over '
                f'{dt * 1000:.1f} ms (L={L} R={R} prev_L={self._prev_L} '
                f'prev_R={self._prev_R}) — corrupted read, not real motion.')
            return

        self._prev_L, self._prev_R = L, R
        self._prev_odom_time = now
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

        # Feedback for the linear half of _control_update. Encoder-derived rather than
        # gyro (the gyro measures yaw only), and taken from the same averaged wheel delta
        # the odometry uses so the two can never disagree about how fast we're going.
        #
        # Filtered, unlike the raw value published above — see MEAS_FILTER_ALPHA. A
        # too-short dt means the encoder register simply had not refreshed, so d/dt is an
        # artifact rather than a measurement; drop it instead of feeding it to the filter.
        if dt >= MIN_ODOM_DT_S:
            self._meas_lin += MEAS_FILTER_ALPHA * ((d / dt) - self._meas_lin)

    def _capture_stamp(self, capture_monotonic):
        """ROS time at which `capture_monotonic` (a time.monotonic() value) happened.

        The driver records packet arrival on the monotonic clock because it is plain
        Python with no node handle; ROS time lives here. Rather than mix the two, measure
        the data's AGE on the monotonic clock and subtract that from ROS "now", which is
        correct regardless of any offset between the clocks.

        Falls back to now() if the driver has not recorded an arrival yet, or if the age
        comes out negative or implausibly large — a bad stamp is worse than a slightly
        stale one, since TF lookups at a wrong time silently return a wrong pose.
        """
        now = self.get_clock().now()
        if capture_monotonic is None:
            return now
        age = time.monotonic() - capture_monotonic
        if not (0.0 <= age < MAX_CAPTURE_AGE_S):
            return now
        return now - Duration(seconds=age)

    # ── IMU (rate gyro only — see YAW_RATE_VARIANCE comment above) ───────────

    def _imu_update(self):
        try:
            gyro = self.robot.gyro_velocity_data()
            z_samples = gyro['angular velocity of z: ']
            if not z_samples:
                return
            # MEAN of the packet's samples, not z_samples[-1]. The Kobuki reports the gyro
            # faster than the 50 Hz feedback packet rate, so each packet carries several
            # z-axis samples; taking only the last threw the rest away. What the EKF does
            # with this value is integrate it over the interval to get heading
            # (ekf_hardware.yaml fuses the yaw RATE and nothing else), and the integral of
            # the interval is exactly the mean — a trailing point-sample is both noisier
            # and, whenever the rate is changing across the packet, biased.
            yaw_rate_dps = sum(z_samples) / len(z_samples)
            capture_monotonic = self.robot.packet_monotonic
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
        # Stamp when this gyro sample was CAPTURED, not when this timer got round to
        # publishing it. This 50 Hz timer runs independently of the serial reader, so
        # "now" overstates the data's freshness by however long the packet has been
        # sitting there — and the EKF has no absolute yaw reference to correct against,
        # so a rate attributed to the wrong instant integrates straight into a heading
        # error that never washes out. Same defect, and same fix, as rplidar_node and
        # kinect_bridge (see docs/ghost_map_investigation.md); the IMU was simply missed
        # at the time, which mattered more than either, this being the only heading source.
        msg.header.stamp = self._capture_stamp(capture_monotonic).to_msg()
        msg.header.frame_id = 'imu_link'

        # No absolute orientation available from this sensor: element 0 of the
        # covariance is the sensor_msgs/Imu sentinel for "whole field invalid" —
        # unlike a per-axis covariance, -1 here must NOT be set on
        # angular_velocity_covariance below, since z *is* valid data.
        msg.orientation_covariance[0] = -1.0
        # No accelerometer data available from this driver.
        msg.linear_acceleration_covariance[0] = -1.0

        msg.angular_velocity.z = math.radians(yaw_rate_dps)
        # Feedback for the yaw half of _control_update — the bias-corrected rate, i.e. the
        # same value published here, so the controller and the EKF agree on the truth.
        # Filtered with the same time constant as the linear axis for consistency. The
        # gyro does not suffer the polling artifact the encoders do (it is a genuine
        # continuous measurement), but it does carry real stick-slip content: measured
        # stdev 0.084 rad/s against a 0.17 rad/s mean. Smoothing that keeps the drivetrain's
        # own judder out of the motor command instead of amplifying it round the loop.
        self._meas_ang += MEAS_FILTER_ALPHA * (math.radians(yaw_rate_dps) - self._meas_ang)
        msg.angular_velocity_covariance[0] = 1e6   # x: not fused, large variance (not -1 —
        msg.angular_velocity_covariance[4] = 1e6   # y: that would invalidate z too)
        msg.angular_velocity_covariance[8] = YAW_RATE_VARIANCE

        self._imu_pub.publish(msg)

    # ── Velocity command ─────────────────────────────────────────────────────

    def _diagnostics_update(self):
        """Publish battery voltage and per-wheel PWM — see the publisher setup comment."""
        try:
            sensor = self.robot.basic_sensor_data()
        except Exception:
            return   # __basic_sensor not populated yet, same guard as _odom_update

        raw_v = sensor.get('Batteryvolt')
        if raw_v is not None:
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            # Kobuki reports battery in 0.1 V units in a single byte.
            msg.voltage = float(raw_v) * 0.1
            # 4S Li-ion: ~16.5 V charged, ~13.2 V is the documented low-battery point.
            # Reported as a rough fraction only, for trend watching, not for gating.
            msg.percentage = max(0.0, min(1.0, (msg.voltage - 13.2) / (16.5 - 13.2)))
            msg.present = True
            msg.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_CHARGING
                if str(sensor.get('Charger', '')).endswith('CHARGING')
                else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            )
            self._battery_pub.publish(msg)

        left, right = sensor.get('LeftPWM'), sensor.get('RightPWM')
        if left is not None and right is not None:
            # Signed 8-bit: the base reports these as raw bytes.
            pwm = Int16MultiArray()
            pwm.data = [
                left - 256 if left > 127 else left,
                right - 256 if right > 127 else right,
            ]
            self._pwm_pub.publish(pwm)

    def cmd_vel_callback(self, msg):
        """Store the velocity setpoint; _control_update does the hardware write."""
        self._cmd_lin = msg.linear.x    # Forward/Backward speed (m/s)
        self._cmd_ang = msg.angular.z   # Turning speed (rad/s)
        self._cmd_vel_zero = (self._cmd_lin == 0.0 and self._cmd_ang == 0.0)
        self._cmd_vel_stamp = self.get_clock().now()

        if self._watchdog_tripped:
            self.get_logger().info('cmd_vel resumed; releasing watchdog stop.')
            self._watchdog_tripped = False

        # A commanded stop must take effect immediately and completely — don't wait for
        # the next control tick, and drop the integrators so a stop can never be
        # partially cancelled by accumulated correction from the motion that preceded it.
        if self._cmd_vel_zero:
            self._i_lin = 0.0
            self._i_ang = 0.0
            self._write_wheels(0.0, 0.0)

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    @staticmethod
    def _feedforward(setpoint, deadband, gain):
        """Invert the measured deadband curve: what to command to actually get `setpoint`.

        The base's response above its deadband is roughly actual = gain * (|cmd| - deadband),
        so getting `setpoint` out requires asking for deadband + |setpoint| / gain. Returns
        0 for a 0 setpoint — the deadband offset must never be added to a commanded stop,
        or the robot would creep whenever told to hold still.
        """
        if setpoint == 0.0 or gain <= 0.0:
            return setpoint
        sign = 1.0 if setpoint > 0.0 else -1.0
        return sign * (deadband + abs(setpoint) / gain)

    def _control_update(self):
        """Closed-loop velocity control at 50 Hz — see CONTROL_PERIOD_S comment block."""
        # Watchdog: stop if the commander went away mid-drive.
        if self._cmd_vel_stamp is None:
            return   # nothing has ever commanded us; leave the motors alone
        age = (self.get_clock().now() - self._cmd_vel_stamp).nanoseconds / 1e9
        if age > CMD_VEL_TIMEOUT_S:
            if not self._watchdog_tripped:
                self.get_logger().warn(
                    f'No /cmd_vel for {age:.2f}s (> {CMD_VEL_TIMEOUT_S}s) — stopping motors. '
                    'The previous behaviour latched the last command indefinitely.')
                self._watchdog_tripped = True
                self._cmd_lin = 0.0
                self._cmd_ang = 0.0
                self._i_lin = 0.0
                self._i_ang = 0.0
                self._write_wheels(0.0, 0.0)
            return

        # Holding still is open-loop on purpose: with zero setpoint there is no shortfall
        # to correct, and running the integrator here would make it fight sensor noise and
        # creep the robot instead of keeping it parked.
        if self._cmd_vel_zero:
            self._write_wheels(0.0, 0.0)
            return

        meas_lin = self._meas_lin if abs(self._meas_lin) > LIN_VEL_DEADZONE else 0.0
        meas_ang = self._meas_ang if abs(self._meas_ang) > YAW_RATE_DEADZONE else 0.0

        err_lin = self._cmd_lin - meas_lin
        err_ang = self._cmd_ang - meas_ang

        lin_ki = self.get_parameter('lin_ki').value
        yaw_ki = self.get_parameter('yaw_ki').value

        # Integrate only the axis we are actually being asked to move, so e.g. a pure
        # rotation doesn't wind up the linear integrator against its own zero setpoint.
        if self._cmd_lin != 0.0:
            self._i_lin = self._clamp(
                self._i_lin + err_lin * lin_ki * CONTROL_PERIOD_S, LIN_VEL_I_CLAMP)
        else:
            self._i_lin = 0.0
        if self._cmd_ang != 0.0:
            self._i_ang = self._clamp(
                self._i_ang + err_ang * yaw_ki * CONTROL_PERIOD_S, YAW_RATE_I_CLAMP)
        else:
            self._i_ang = 0.0

        # Static deadband inversion does the bulk of the work (instant, no lag); the small
        # PI terms only trim what the model gets wrong. See the _feedforward docstring.
        ff_lin = self._feedforward(
            self._cmd_lin,
            self.get_parameter('lin_ff_deadband').value,
            self.get_parameter('lin_ff_gain').value)
        ff_ang = self._feedforward(
            self._cmd_ang,
            self.get_parameter('yaw_ff_deadband').value,
            self.get_parameter('yaw_ff_gain').value)

        out_lin = ff_lin + self.get_parameter('lin_kp').value * err_lin + self._i_lin
        out_ang = ff_ang + self.get_parameter('yaw_kp').value * err_ang + self._i_ang

        self._write_wheels(out_lin, out_ang)

    def _write_wheels(self, linear_x, angular_z):
        """Differential-drive mixing + hardware write (mm/s, as the driver expects)."""
        left_wheel_speed = (linear_x - (angular_z * WHEEL_BASE_M / 2.0)) * 1000.0
        right_wheel_speed = (linear_x + (angular_z * WHEEL_BASE_M / 2.0)) * 1000.0

        # 1 means pure rotation, 0 means forward/arc. Keyed off the *commanded* linear
        # velocity, not the post-correction output: out_lin can pick up a small nonzero
        # value from the controller during an in-place turn, and that must not silently
        # switch the driver out of rotation mode.
        #
        # The test is the turn RADIUS, not "linear is exactly zero". This used to read
        # `self._cmd_lin == 0.0`, which meant any linear component at all — however
        # tiny — dropped the base into arc mode, and the two branches of Kobuki.move()
        # scale their speed argument completely differently:
        #   rotation: botspeed = |R - L| / 2      the TANGENTIAL WHEEL speed  (~66 mm/s)
        #   arc:      botspeed = (L + R) / 2      the ROBOT CENTRE speed      (~20 mm/s)
        # For a near-pivot those describe near-identical motion, but the arc form asks
        # for a fraction of the magnitude, and the base's own low-speed threshold then
        # swallows it. At lin 0.0105 / ang 0.400 the rotation branch asks for 46 mm/s
        # and the arc branch for 10.5 mm/s — same intended motion, 4.4x less command.
        #
        # A/B measured on the bench 2026-09-06, both arms back to back at 15.9 V, yaw
        # read from the gyro (independent of the wheel model under test), achieved
        # rad/s as a fraction of commanded:
        #     lin      ang     radius   old test   radius test
        #   0.0000    0.400     0.000     1.01        1.04
        #   0.0105   -0.400     0.026     0.16        1.03
        #   0.0300    0.400     0.075     0.36        1.01
        #   0.0600   -0.400     0.150     0.79        0.78   <- true arc, unchanged
        #   0.0105    0.800     0.013     0.09        1.01
        # The 0.150 m row is the control: it is a genuine arc under both tests, and it
        # comes out identical, so the change is confined to the near-pivot regime.
        # Note the loss worsens as commanded yaw rises (0.09 at 0.800 rad/s) — that is
        # precisely DWB's turn-hard regime.
        #
        # Nav2 emits exactly that combination constantly — DWB rarely outputs a linear
        # velocity of precisely 0.0 while turning — so in a mission almost every turn was
        # taking the degraded path. That is what produced the repeated "no progress for
        # 31s" stalls: the robot was commanded to turn, reported as turning by the
        # controller, and physically barely moved. Reproducing at 15.9 V also rules out
        # the battery-droop explanation that was chased first.
        #
        # Radius below half the wheelbase means the turn centre lies inside the robot's
        # own footprint, i.e. it is a pivot in all but name, so the linear term being
        # discarded by the rotation branch is negligible — and far cheaper than losing
        # most of the commanded yaw.
        turn_radius = (
            abs(self._cmd_lin / self._cmd_ang) if self._cmd_ang != 0.0 else float('inf')
        )
        pivot_radius = self.get_parameter('pivot_radius_m').value
        rotate_flag = 1 if angular_z != 0.0 and turn_radius < pivot_radius else 0

        self.robot.move(int(left_wheel_speed), int(right_wheel_speed), rotate_flag)
        self.get_logger().debug(
            f'cmd=({self._cmd_lin:.3f},{self._cmd_ang:.3f}) '
            f'meas=({self._meas_lin:.3f},{self._meas_ang:.3f}) '
            f'out=({linear_x:.3f},{angular_z:.3f}) '
            f'L={left_wheel_speed:.1f} R={right_wheel_speed:.1f} Rot={rotate_flag}')

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
        # rclpy may already have shut the context down on Ctrl-C; calling shutdown()
        # again raises RCLError and turns a clean stop into a -9/exit-1 in the logs.
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()