#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

# Instructions
msg = """
Control Your Robot!
---------------------------
Moving around:
   w    
a    d
   s    

w/s: increase/decrease linear velocity (forward/backward)
a/d: increase/decrease angular velocity (left/right)

space key: stop
CTRL-C to quit
"""

# Velocity increments and limits
LINEAR_INCREMENT = 0.1  # Step for linear velocity
ANGULAR_INCREMENT = 0.1  # Step for angular velocity
MAX_LINEAR = 1.5  # Maximum linear velocity
MAX_ANGULAR = 1.5  # Maximum angular velocity

class RobotTeleopNode(Node):
    def __init__(self):
        super().__init__('robot_teleop_node')
        self.publisher = self.create_publisher(Twist, '/ackermann_steering_controller/reference_unstamped', 10)
        self.settings = termios.tcgetattr(sys.stdin)

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def run(self):
        print(msg)
        twist = Twist()
        try:
            while True:
                key = self.get_key()

                # Adjust linear/angular velocity
                if key == 'w':  # Increase forward speed
                    self.linear_velocity += LINEAR_INCREMENT
                elif key == 's':  # Decrease forward speed
                    self.linear_velocity -= LINEAR_INCREMENT
                elif key == 'a':  # Turn left
                    self.angular_velocity += ANGULAR_INCREMENT
                elif key == 'd':  # Turn right
                    self.angular_velocity -= ANGULAR_INCREMENT
                elif key == ' ':  # Stop all motion
                    self.linear_velocity = 0.0
                    self.angular_velocity = 0.0
                elif key == '\x03':  # CTRL+C to quit
                    break

                # Clamp velocities within limits
                self.linear_velocity = self.clamp(self.linear_velocity, -MAX_LINEAR, MAX_LINEAR)
                self.angular_velocity = self.clamp(self.angular_velocity, -MAX_ANGULAR, MAX_ANGULAR)

                # Set the current velocities
                twist.linear.x = self.linear_velocity
                twist.angular.z = self.angular_velocity

                # Publish the Twist message
                self.publisher.publish(twist)

                # Display current velocity values
                print(f"Linear Velocity: {self.linear_velocity:.2f}, Angular Velocity: {self.angular_velocity:.2f}")

        except Exception as e:
            print(e)
        finally:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
