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
            '/ackermann_steering_controller/reference_unstamped',
            10
        )
        self.speed = 1.0
        self.turn = 0.5
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
            cmd = Twist()
            if key == 'w':
                cmd.linear.x = self.speed
            elif key == 's':
                cmd.linear.x = -self.speed
            elif key == 'a':
                cmd.angular.z = self.turn
            elif key == 'd':
                cmd.angular.z = -self.turn
            elif key == ' ':
                pass  # zero cmd = stop
            elif key == 'q':
                break
            self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WASDTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
