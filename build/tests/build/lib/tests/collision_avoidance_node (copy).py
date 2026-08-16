# === dodging_node.py ===
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np

class DodgingNode(Node):
    def __init__(self):
        super().__init__('dodging_node')

        self.subscription_depth = self.create_subscription(Image, '/camera/depth/depth_camera/depth/image_raw', self.depth_callback, 10)
        self.subscription_right_depth = self.create_subscription(Image, '/camera_right/depth/depth_camera_right/depth/image_raw', self.right_depth_callback, 10)
        self.subscription_left_depth = self.create_subscription(Image, '/camera_left/depth/depth_camera_left/depth/image_raw', self.left_depth_callback, 10)

        self.publisher_factor = self.create_publisher(Float32, '/obstacle_speed_factor', 10)
        self.publisher_steering_left = self.create_publisher(Float32, '/steering_correction_left', 10)
        self.publisher_steering_right = self.create_publisher(Float32, '/steering_correction_right', 10)
        self.publisher_steering_center = self.create_publisher(Float32, '/steering_correction_center', 10)

        self.bridge = CvBridge()
        self.depth_frame = None
        self.left_depth_frame = None
        self.right_depth_frame = None

        self.get_logger().info("DodgingNode initialized")

        self.min_depth = 0.3
        self.max_depth = 3.0
        self.max_steering_angle = 0.75  # radians
        self.min_speed_factor = 0.3

    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
            return

        self.process_depth_image()

    def left_depth_callback(self, msg):
        try:
            self.left_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f"Left depth conversion failed: {e}")

    def right_depth_callback(self, msg):
        try:
            self.right_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f"Right depth conversion failed: {e}")

    def process_depth_image(self):
        if self.depth_frame is None:
            return

        height, width = self.depth_frame.shape
        region = self.depth_frame[:int(height * 3 / 4), :]
        valid_mask = (region > self.min_depth) & (region < self.max_depth) & np.isfinite(region)

        factor = 1.0
        steering_center = 0.0

        if np.count_nonzero(valid_mask) > 0:
            y_idxs, x_idxs = np.where(valid_mask)
            depths = region[y_idxs, x_idxs]
            min_dist = float(np.min(depths))

            mean_x = np.mean(x_idxs)
            center_x = width / 2
            offset = center_x - mean_x
            offset_norm = offset / center_x
            steering_center = offset_norm * self.max_steering_angle
            steering_center = float(np.clip(steering_center, -self.max_steering_angle, self.max_steering_angle))
        else:
            min_dist = self.max_depth

        # Speed control based on front obstacle
        min_d = 0.3
        acc_min_d = 0.7
        max_d = 1.6
        if min_dist < acc_min_d:
            factor = self.min_speed_factor
        elif min_dist <= max_d:
            distance_factor = (min_dist - min_d) / (max_d - min_d)
            factor = max(self.min_speed_factor, distance_factor)
        else:
            factor = 1.0

        self.publisher_factor.publish(Float32(data=factor))
        self.publisher_steering_center.publish(Float32(data=steering_center))

        # Side steering
        left_angle = self.calculate_side_steering(self.left_depth_frame, is_left=True)
        right_angle = self.calculate_side_steering(self.right_depth_frame, is_left=False)

        if left_angle > 0:
            self.publisher_steering_left.publish(Float32(data=left_angle))
        if right_angle > 0:
            self.publisher_steering_right.publish(Float32(data=right_angle))

    def calculate_side_steering(self, frame, is_left):
        if frame is None:
            return 0.0

        try:
            height, width = frame.shape
            region = frame[int(height * 0.25):int(height * 0.75), int(width * 0.25):int(width * 0.75)]
            valid = (region > self.min_depth) & (region < self.max_depth) & np.isfinite(region)
            if np.count_nonzero(valid) < 50:
                return 0.0
            depths = region[valid]
            sorted_depths = np.sort(depths.flatten())
            avg_depth = float(np.mean(sorted_depths[:200]))
            if avg_depth < 1.2:
                strength = (1.2 - avg_depth) / 1.2
                angle = strength * self.max_steering_angle
                return angle if is_left else angle
            else:
                return 0.0
        except Exception as e:
            self.get_logger().warn(f"Failed to process {'left' if is_left else 'right'} side: {e}")
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = DodgingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
