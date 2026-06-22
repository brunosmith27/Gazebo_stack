import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/depth_camera/depth/image_raw',
            self.depth_callback,
            10
        )
        # Thresholds in meters
        self.CLEAR = 2.0
        self.NEARBY = 1.0
        self.CRITICAL = 0.5
        self.get_logger().info("Obstacle detector running...")

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h, w = depth_image.shape
        center = depth_image[h//3:2*h//3, w//3:2*w//3]
        valid = center[np.isfinite(center) & (center > 0)]
        if len(valid) == 0:
            return
        min_dist = float(np.min(valid))
        self.get_logger().info(f'Closest object: {min_dist:.2f}m — State: {self.classify(min_dist)}')

    def classify(self, distance):
        if distance > self.CLEAR:
            return "CLEAR"
        elif distance > self.NEARBY:
            return "NEARBY"
        elif distance > self.CRITICAL:
            return "CLOSE"
        else:
            return "CRITICAL"

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
