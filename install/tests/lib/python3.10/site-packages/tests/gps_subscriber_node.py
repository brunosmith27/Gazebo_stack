import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped

class GPSSubscriber(Node):
    def __init__(self):
        super().__init__('gps_subscriber')
        self.fix_sub = self.create_subscription(
            NavSatFix, '/gps_plugin/fix', self.gps_fix_callback, 10)
        self.vel_sub = self.create_subscription(
            Vector3Stamped, '/gps_plugin/vel', self.gps_vel_callback, 10)

    def gps_fix_callback(self, msg):
        self.get_logger().info(
            f"GPS Fix: Lat {msg.latitude}, Lon {msg.longitude}, Alt {msg.altitude}")

    def gps_vel_callback(self, msg):
        self.get_logger().info(
            f"GPS Velocity: x={msg.vector.x}, y={msg.vector.y}, z={msg.vector.z}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
