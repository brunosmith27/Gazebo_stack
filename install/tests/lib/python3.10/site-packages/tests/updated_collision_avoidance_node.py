import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np

class DodgingNode(Node):
    def __init__(self):
        super().__init__('dodging_node')

        self.subscription_depth = self.create_subscription(
            Image,
            '/camera/depth/depth_camera/depth/image_raw',  # ✅ Use your Gazebo topic here
            self.depth_callback,
            10
        )

        self.publisher_factor = self.create_publisher(
            Float32,
            '/obstacle_speed_factor',
            10
        )

        self.bridge = CvBridge()
        self.depth_frame = None
        self.get_logger().info("DodgingNode initialized (no Arduino)")

    def depth_callback(self, msg):
        try:
            # Gazebo usually uses 32FC1 (meters)
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
            return
        self.process_depth_image()

    def process_depth_image(self):
        if self.depth_frame is None:
            return

        height, width = self.depth_frame.shape
        region = self.depth_frame[:int(height * 3/4), :]
        valid_depths = region[(region > 0.2) & np.isfinite(region) & (region < 10.0)]

        if valid_depths.size == 0:
            factor = 1.0
        else:
            min_dist = float(np.min(valid_depths))

            if min_dist < 1.5:
                factor = 0.0
            elif 1.5 <= min_dist <= 2.0:
                factor = (min_dist - 1.5) / 0.5
            else:
                factor = 1.0

            self.get_logger().info(f"Min depth: {min_dist:.2f} m → Speed factor: {factor:.2f}")

        self.publisher_factor.publish(Float32(data=factor))

def main(args=None):
    rclpy.init(args=args)
    node = DodgingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
