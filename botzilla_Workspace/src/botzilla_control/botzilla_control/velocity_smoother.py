"""
velocity_smoother.py.

Acceleration-limits the Nav2 velocity stream before it reaches the base:
subscribes 'cmd_vel_nav', publishes 'cmd_vel' at a fixed rate.

Why this exists at all
----------------------
Nothing in the chain used to enforce output continuity. DWB's default trajectory
generator (dwb_plugins::StandardTrajectoryGenerator) samples the *entire* configured
velocity range every control cycle regardless of what the robot is currently doing —
acc_lim_x / acc_lim_theta only shape the simulated trajectory, they do not bound the
emitted command. kobuki_base_node's cmd_vel handler was then a stateless passthrough
straight to the motors. So cmd_vel could legally jump 0 -> +0.4 -> -0.4 rad/s on
consecutive 20 Hz cycles, and did: 24 angular sign flips measured in 80 s of driving.
That is the jerking.

Two changes address that at the source: nav2_params.yaml now selects
dwb_plugins::LimitedAccelGenerator (so DWB only samples velocities reachable within one
control period), and kobuki_base_node closes the loop on gyro + encoders so the velocity
floors that used to force step inputs could be dropped to zero.

This node covers what neither of those can: **behavior_server**. BackUp and Spin publish
their own velocity commands directly, bypassing DWB and its trajectory generator
entirely, and those transitions are exactly where the robot is already close to
something. Recovery behaviours were also the most visibly violent part of the motion.
Smoothing here catches every publisher on the topic, not just the controller.

Why not nav2_velocity_smoother
------------------------------
It was in this chain before and was removed (see nav2.launch.py's header): on hardware it
silently stopped republishing — controller_server kept publishing to 'cmd_vel_nav' while
'/cmd_vel' had no publisher at all, the node reporting lifecycle state active and logging
nothing, so every goal stalled with no diagnosable cause. The failure was invisible,
which is the real problem. This node is deliberately built so the same failure cannot be
silent:

  * It publishes on a timer, unconditionally, whenever it has ever received input — so
    '/cmd_vel' always has a live publisher while this node is alive.
  * An input watchdog decays the output to zero (respecting decel limits) instead of
    latching the last command, and logs the transition once.
  * A periodic diagnostic logs input/output rates, so a stalled stream is visible in the
    log rather than inferred from a robot that mysteriously stopped.

Safety
------
A commanded stop is NOT rate-limited on the way down to zero when it comes from an empty
input stream — see DECEL_TO_STOP_IMMEDIATELY. Ramping a stop would mean overshooting into
whatever prompted it.
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

# Output rate. Matches nav2_params.yaml's controller_frequency (20 Hz) so the smoother
# neither starves the base nor invents intermediate setpoints the controller never asked
# for. Republishing faster than the input arrives is fine and intended: it is what keeps
# a live publisher on /cmd_vel and lets the ramp progress between input messages.
PUBLISH_RATE_HZ = 20.0

# Acceleration limits, per second. Kept in step with nav2_params.yaml's acc_lim_* so DWB's
# internal LimitedAccelGenerator and this node agree about what the robot can do — if this
# node were tighter it would silently invalidate DWB's trajectory predictions, making the
# controller believe it is following a path it cannot actually execute.
MAX_ACCEL_X = 0.4         # m/s^2
MAX_ACCEL_THETA = 1.5     # rad/s^2

# Deceleration is allowed to be more aggressive than acceleration: slowing down is the
# safe direction, and being sluggish about it is what causes overshoot into obstacles.
MAX_DECEL_X = 0.8         # m/s^2
MAX_DECEL_THETA = 2.5     # rad/s^2

# If no input arrives within this window, ramp to zero. Slightly longer than
# kobuki_base_node's own CMD_VEL_TIMEOUT_S (0.5 s) would be wrong — this must act first so
# the stop is smooth rather than the base's abrupt failsafe cut.
INPUT_TIMEOUT_S = 0.3

# When the input stream dies, go to zero in one step rather than ramping. The ramp exists
# to make *commanded* motion smooth; a vanished commander is a fault, and coasting to a
# stop over ~0.25 s while nothing is steering is worse than stopping hard.
DECEL_TO_STOP_IMMEDIATELY = True

DIAGNOSTIC_PERIOD_S = 10.0

# --- Reversal hysteresis -----------------------------------------------------------
# Acceleration limiting alone does not stop the robot looking jerky, because it bounds how
# FAST the command changes but not how OFTEN it changes direction. Measured on hardware
# over 164 s of exploring, per stage of the chain:
#
#                        max step/cycle          sign flips
#   /cmd_vel_nav (DWB)   0.300 m/s, 0.800 rad/s   x=92  theta=143
#   /cmd_vel (smoothed)  0.040 m/s, 0.125 rad/s   x=65  theta=109
#
# The smoother was doing its job on magnitude (7.5x and 6.4x reduction, exactly its decel
# limits) and yet two thirds of the reversals still reached the base — it faithfully ramped
# through every one of them. A ramped reversal is still a reversal: the robot visibly stops
# and goes the other way ~40 times a minute.
#
# The reversals originate in DWB: dwb_plugins::StandardTrajectoryGenerator re-samples the
# entire velocity range every cycle, so when the score landscape is nearly flat (a noisy
# costmap, a path that shifts slightly) the argmax can jump from +max to -max between
# consecutive cycles. Fixing it there would mean LimitedAccelGenerator, which was A/B'd on
# this robot and nearly stopped it translating (0.12 m in 89 s vs 20.2 m) — see the long
# note in nav2_params.yaml. So it is filtered here instead.
#
# Rule: a command that opposes the current direction of travel must persist for
# REVERSAL_CONFIRM_CYCLES before it is honoured. While unconfirmed, the target is treated
# as zero, so the robot DECELERATES rather than latching — which is where it was heading on
# the way to reversing anyway. Nothing is lost if the flip was real (it costs
# REVERSAL_CONFIRM_CYCLES / PUBLISH_RATE_HZ = 0.15 s of extra braking, during which the
# robot is already slowing), and a spurious one-cycle flip becomes a slight slow-down
# instead of a direction change.
#
# This deliberately does NOT delay stopping, and does not delay the reverse-escape
# behaviour that min_vel_x=-0.10 exists for: a genuine escape command persists for many
# cycles and confirms immediately.
REVERSAL_CONFIRM_CYCLES = 3

# A reversal smaller than this is dithering around zero, not a real change of intent, and
# is collapsed straight to zero without ever being confirmed. Sized below the base's
# measured deadbands (linear 0.009 m/s, yaw 0.121 rad/s — see kobuki_base_node) so that
# commands the hardware could actually execute are never silently discarded.
REVERSAL_DEADBAND_X = 0.01        # m/s
REVERSAL_DEADBAND_THETA = 0.05    # rad/s


class VelocitySmoother(Node):
    """Acceleration-limits 'cmd_vel_nav' onto 'cmd_vel' — see the module docstring."""

    def __init__(self):
        """Wire up the input subscription, output publisher, and the two timers."""
        super().__init__('velocity_smoother')

        self._target = Twist()      # latest command from Nav2
        self._output = Twist()      # what we last published (the ramp's current state)
        self._last_input_time = None
        self._input_count = 0
        self._output_count = 0
        self._timed_out = False
        # Consecutive cycles the input has opposed our direction of travel, per axis.
        self._flip_cycles = {'x': 0, 'theta': 0}
        self._suppressed_flips = 0

        self._sub = self.create_subscription(Twist, 'cmd_vel_nav', self._input_cb, 10)
        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self._period = 1.0 / PUBLISH_RATE_HZ
        self.create_timer(self._period, self._tick)
        self.create_timer(DIAGNOSTIC_PERIOD_S, self._diagnostics)

        self.get_logger().info(
            f'velocity_smoother started: cmd_vel_nav -> cmd_vel at {PUBLISH_RATE_HZ:.0f} Hz, '
            f'accel limits x={MAX_ACCEL_X} theta={MAX_ACCEL_THETA}, '
            f'decel x={MAX_DECEL_X} theta={MAX_DECEL_THETA}')

    def _input_cb(self, msg):
        self._target = msg
        self._last_input_time = self.get_clock().now()
        self._input_count += 1
        if self._timed_out:
            self.get_logger().info('cmd_vel_nav resumed after timeout.')
            self._timed_out = False

    @staticmethod
    def _ramp(current, target, max_accel, max_decel, dt):
        """Step `current` toward `target`, respecting whichever limit applies.

        Accelerating and decelerating carry different limits, and the one that applies
        depends on whether THIS step grows or shrinks the magnitude — which is not the same
        question as whether the target's magnitude is larger.

        The distinction matters on a sign reversal. Going 0.4 -> -0.4, the robot must first
        slow to zero (deceleration) and only then build up in the new direction
        (acceleration). Deciding from abs(target) > abs(current) gets that wrong: it calls
        the whole reversal "speeding up" and applies the gentler accel limit to the slowing
        phase, making a commanded reversal take longer to bite than a commanded stop —
        backwards, since a reversal is usually the more urgent of the two.

        So compare the direction of the step against the direction of travel: a delta with
        the same sign as `current` moves away from zero (accelerating), and one with the
        opposite sign moves toward zero (decelerating).
        """
        delta = target - current
        if delta == 0.0:
            return target
        if current == 0.0:
            speeding_up = True          # starting from rest, any motion is acceleration
        else:
            speeding_up = (delta * current) > 0.0
        limit = (max_accel if speeding_up else max_decel) * dt
        if abs(delta) <= limit:
            return target
        return current + (limit if delta > 0 else -limit)

    def _confirm_reversal(self, axis, current, target, deadband):
        """Hold off a direction reversal until it persists — see REVERSAL_CONFIRM_CYCLES.

        Returns the target to actually ramp toward. Zero means "keep decelerating while we
        decide", which is the same direction the ramp would travel anyway on its way to
        reversing, so waiting costs nothing but a little extra braking.
        """
        # Not a reversal: same sign, or either side is already at rest.
        if current == 0.0 or target == 0.0 or (target * current) > 0.0:
            self._flip_cycles[axis] = 0
            return target

        # Opposing, but too small to be a real change of intent — treat it as a stop.
        if abs(target) < deadband:
            self._flip_cycles[axis] = 0
            return 0.0

        self._flip_cycles[axis] += 1
        if self._flip_cycles[axis] >= REVERSAL_CONFIRM_CYCLES:
            self._flip_cycles[axis] = 0
            return target

        self._suppressed_flips += 1
        return 0.0

    def _tick(self):
        if self._last_input_time is None:
            return   # never commanded — stay silent rather than publish zeros at boot

        age = (self.get_clock().now() - self._last_input_time).nanoseconds / 1e9
        if age > INPUT_TIMEOUT_S:
            if not self._timed_out:
                self.get_logger().warn(
                    f'No cmd_vel_nav for {age:.2f}s (> {INPUT_TIMEOUT_S}s) — commanding stop.')
                self._timed_out = True
            self._target = Twist()   # zero
            if DECEL_TO_STOP_IMMEDIATELY:
                self._output = Twist()
                self._publish()
                return

        goal_x = self._confirm_reversal(
            'x', self._output.linear.x, self._target.linear.x, REVERSAL_DEADBAND_X)
        goal_th = self._confirm_reversal(
            'theta', self._output.angular.z, self._target.angular.z, REVERSAL_DEADBAND_THETA)

        self._output.linear.x = self._ramp(
            self._output.linear.x, goal_x, MAX_ACCEL_X, MAX_DECEL_X, self._period)
        self._output.angular.z = self._ramp(
            self._output.angular.z, goal_th, MAX_ACCEL_THETA, MAX_DECEL_THETA, self._period)
        self._publish()

    def _publish(self):
        self._pub.publish(self._output)
        self._output_count += 1

    def _diagnostics(self):
        """Make a stalled stream visible in the log — the old smoother's failure was silent."""
        in_hz = self._input_count / DIAGNOSTIC_PERIOD_S
        out_hz = self._output_count / DIAGNOSTIC_PERIOD_S
        self._input_count = 0
        self._output_count = 0
        if in_hz == 0.0 and out_hz == 0.0:
            return   # genuinely idle (no goal active) — not worth a line every 10 s
        self.get_logger().info(
            f'[smoother] in={in_hz:.1f}Hz out={out_hz:.1f}Hz '
            f'target=({self._target.linear.x:.3f},{self._target.angular.z:.3f}) '
            f'output=({self._output.linear.x:.3f},{self._output.angular.z:.3f}) '
            f'flips_suppressed={self._suppressed_flips}')
        self._suppressed_flips = 0


def main(args=None):
    """Spin the smoother, leaving the base commanded to zero on shutdown."""
    rclpy.init(args=args)
    node = VelocitySmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the base stopped rather than latched at whatever we last sent.
        try:
            node._pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
