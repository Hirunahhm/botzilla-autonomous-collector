"""
odom_covariance_relay.py

Republishes Gazebo's /odom with realistic covariances as /odom_cov.

Why this exists: the gz-sim DiffDrive plugin publishes odometry with an ALL-ZERO
covariance matrix. `robot_localization` reads a zero variance as "this measurement is
infinitely certain," which is both physically wrong and numerically degenerate — with
zero twist covariance the EKF silently fails to incorporate the wheel velocities at all
(measured: /odom reported vx=0.2000 m/s while /odometry/filtered's vx stayed pinned at
0.0000 and its position never left the origin).

That matters because the EKF is deliberately configured (see config/ekf.yaml) to take
only VELOCITIES from the wheels and heading from the IMU — wheel absolute pose is
untrustworthy under slip. So wheel velocity fusion has to actually work.

The variances below are ordinary diff-drive wheel-odometry values: reasonably confident
in forward speed, very confident that lateral speed is zero (a diff-drive robot cannot
translate sideways), and deliberately LOOSE on yaw rate so the IMU — which measures true
rotation and is immune to wheel slip — dominates heading.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# Diagonal variances for [x, y, z, roll, pitch, yaw]
POSE_VARIANCE = [0.05, 0.05, 1e6, 1e6, 1e6, 0.10]
# Diagonal variances for [vx, vy, vz, vroll, vpitch, vyaw]
# vy is tiny because differential drive is kinematically incapable of lateral motion.
# vyaw is large on purpose so the IMU wins the heading estimate.
TWIST_VARIANCE = [0.02, 1e-6, 1e6, 1e6, 1e6, 0.50]


class OdomCovarianceRelay(Node):
    def __init__(self):
        super().__init__('odom_covariance_relay')
        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('output_topic', '/odom_cov')
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.sub = self.create_subscription(Odometry, in_topic, self.cb, 10)
        self.get_logger().info(f'relaying {in_topic} -> {out_topic} with non-zero covariance')

    def cb(self, msg):
        # 6x6 row-major: index i*6+i walks the diagonal.
        pose_cov = list(msg.pose.covariance)
        twist_cov = list(msg.twist.covariance)
        for i in range(6):
            pose_cov[i * 6 + i] = POSE_VARIANCE[i]
            twist_cov[i * 6 + i] = TWIST_VARIANCE[i]
        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCovarianceRelay()
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
