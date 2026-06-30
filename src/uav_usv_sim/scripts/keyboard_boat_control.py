#!/usr/bin/env python3

import sys
import termios
import tty

from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node as GzTransportNode
import rclpy
from rclpy.node import Node


HELP_TEXT = """
Keyboard boat control

  W/S : increase/decrease forward speed
  A/D : turn left/right
  X   : reverse direction
  Space : stop
  Q or Ctrl-C : quit
"""


class KeyboardBoatControl(Node):

    def __init__(self):
        super().__init__('keyboard_boat_control')

        if not sys.stdin.isatty():
            raise RuntimeError('keyboard_boat_control must be run in a terminal')

        self.declare_parameter('topic', '/model/simple_boat/cmd_vel')
        self.declare_parameter('speed_step', 0.2)
        self.declare_parameter('turn_step', 0.15)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('max_turn', 1.2)

        self.topic = self.get_parameter('topic').value
        self.speed_step = self.get_parameter('speed_step').value
        self.turn_step = self.get_parameter('turn_step').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn = self.get_parameter('max_turn').value
        self.gz_node = GzTransportNode()
        self.publisher = self.gz_node.advertise(self.topic, Twist)

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.old_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info(HELP_TEXT)
        self.get_logger().info(f'Publishing Gazebo Twist commands on {self.topic}')

    def destroy_node(self):
        self.publish_cmd(0.0, 0.0)
        if hasattr(self, 'old_terminal_settings'):
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.old_terminal_settings,
            )
        super().destroy_node()

    def run(self):
        self.publish_cmd(self.linear_x, self.angular_z)
        while rclpy.ok():
            self.handle_key(sys.stdin.read(1).lower())

    def handle_key(self, key):
        old_linear_x = self.linear_x
        old_angular_z = self.angular_z

        if key == 'w':
            self.linear_x += self.speed_step
        elif key == 's':
            self.linear_x -= self.speed_step
        elif key == 'a':
            self.angular_z += self.turn_step
        elif key == 'd':
            self.angular_z -= self.turn_step
        elif key == 'x':
            self.linear_x = -self.linear_x
        elif key == ' ':
            self.linear_x = 0.0
            self.angular_z = 0.0
        elif key == 'q' or key == '\x03':
            raise KeyboardInterrupt

        self.linear_x = self.clamp(self.linear_x, -self.max_speed, self.max_speed)
        self.angular_z = self.clamp(self.angular_z, -self.max_turn, self.max_turn)
        changed = (
            self.linear_x != old_linear_x or self.angular_z != old_angular_z
        )

        if changed:
            self.publish_cmd(self.linear_x, self.angular_z)
            self.get_logger().info(
                f'linear_x={self.linear_x:.2f}, angular_z={self.angular_z:.2f}'
            )

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        if not self.publisher.publish(msg):
            self.get_logger().warn('Failed to publish boat command')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardBoatControl()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
