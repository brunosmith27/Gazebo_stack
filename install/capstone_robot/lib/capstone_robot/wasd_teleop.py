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
        print("WASD Teleop — w/s=forward/back (also straightens steering), a/d=left/right, space=stop, q=quit")

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
                self.angular_z = 0.0  # driving straight cancels any active turn
            elif key == 's':
                self.linear_x = -self.speed
                self.angular_z = 0.0
            elif key == 'a':
                self.angular_z = self.turn
            elif key == 'd':
                self.angular_z = -self.turn
            elif key == ' ':
                self.linear_x = 0.0
                self.angular_z = 0.0
            elif key == 'q':
                break
            else:
                # Key not bound to any action -- still worth printing, since
                # seeing *something* here confirms this terminal has focus and
                # is receiving your keypresses at all.
                print(f'(ignored key: {key!r})')
                continue

            cmd = Twist()
            cmd.linear.x = self.linear_x
            cmd.angular.z = self.angular_z
            self.pub.publish(cmd)
            # Visual confirmation of what was actually sent -- raw terminal
            # mode suppresses normal key echo, so without this there's no way
            # to tell "key didn't reach this process" apart from "key was
            # received but the Ackermann-steered car can't turn without also
            # rolling forward" (angular_z alone does nothing while linear_x=0).
            print(f'key={key!r} -> linear_x={self.linear_x:+.2f} angular_z={self.angular_z:+.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = WASDTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
