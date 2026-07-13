#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios


class WASDTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(
            Twist,
            '/teleop_cmd_vel',
            10
        )
        self.speed = 1.0
        self.turn = 0.5
        self.linear_x = 0.0
        self.angular_z = 0.0
        print("WASD Teleop — w/s=forward/back, a/d=left/right, space=stop, q=quit")

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def run(self):
        while True:
            key = self.get_key()
            if key == 'w':
                self.linear_x = self.speed
            elif key == 's':
                self.linear_x = -self.speed
            elif key == 'a':
                self.angular_z = self.turn
            elif key == 'd':
                self.angular_z = -self.turn
            elif key == ' ':
                self.linear_x = 0.0
                self.angular_z = 0.0
            elif key == 'q':
                break

            cmd = Twist()
            cmd.linear.x = self.linear_x
            cmd.angular.z = self.angular_z
            self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WASDTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
