import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

INSTRUCTIONS = """
BotZilla Keyboard Teleop
-------------------------
   w
 a s d

w/x : forward / backward
a/d : rotate left / right (in place)
s   : stop
+/- : increase / decrease speed
q   : quit
"""

MOVE_BINDINGS = {
    'w': (1, 0),
    'x': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
    's': (0, 0),
}

SPEED_STEP = 0.05


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed = 0.2
        self.angular_speed = 0.6

    def publish(self, linear_dir, angular_dir):
        msg = Twist()
        msg.linear.x = linear_dir * self.linear_speed
        msg.angular.z = angular_dir * self.angular_speed
        self.pub.publish(msg)

    def stop(self):
        self.pub.publish(Twist())


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = TeleopKeyboard()
    print(INSTRUCTIONS)
    print(f'Speed: linear={node.linear_speed:.2f} m/s, angular={node.angular_speed:.2f} rad/s')

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.3)
            if key in ('q', '\x03'):
                break
            elif key == '+':
                node.linear_speed += SPEED_STEP
                node.angular_speed += SPEED_STEP * 3
                print(f'Speed: linear={node.linear_speed:.2f} m/s, angular={node.angular_speed:.2f} rad/s')
            elif key == '-':
                node.linear_speed = max(0.0, node.linear_speed - SPEED_STEP)
                node.angular_speed = max(0.0, node.angular_speed - SPEED_STEP * 3)
                print(f'Speed: linear={node.linear_speed:.2f} m/s, angular={node.angular_speed:.2f} rad/s')
            elif key in MOVE_BINDINGS:
                linear_dir, angular_dir = MOVE_BINDINGS[key]
                node.publish(linear_dir, angular_dir)
            elif key == '':
                # Stop the robot when the key is released (no key event received within timeout)
                node.stop()
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
