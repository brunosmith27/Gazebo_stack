import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math

class ImuToHeadingNode(Node):
    def __init__(self):
        super().__init__('imu_to_heading_node')
        self.subscription = self.create_subscription(
            Imu,
            '/camera/imu/imu_plugin/out',
            self.imu_callback,
            10)

        self.publisher = self.create_publisher(Float32, 'heading', 10)

        self.latest_imu_msg = None

        # Timer set to 5 Hz
        self.timer = self.create_timer(1.0 / 5.0, self.publish_heading)

        self.get_logger().info("IMU to Heading node started (publishing at 5 Hz in degrees).")

    def imu_callback(self, msg: Imu):
        self.latest_imu_msg = msg

    def publish_heading(self):
        if self.latest_imu_msg is None:
            return

        q = self.latest_imu_msg.orientation

        # Quaternion to Yaw (in radians)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)

        # Convert to degrees
        yaw_deg = math.degrees(yaw_rad)

        # Normalize to 0–360°
        if yaw_deg < 0:
            yaw_deg += 360.0

        heading_msg = Float32()
        heading_msg.data = yaw_deg
        self.publisher.publish(heading_msg)

        self.get_logger().info(f"Published heading: {yaw_deg:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = ImuToHeadingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
