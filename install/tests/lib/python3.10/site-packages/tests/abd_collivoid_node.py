# === gazebo_dodging_node.py ===
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64
from cv_bridge import CvBridge
import numpy as np
import math
import csv

class GazeboDodgingNode(Node):
    def __init__(self):
        super().__init__('gazebo_dodging_node')

        self.subscription_depth = self.create_subscription(Image, '/camera/depth/depth_camera/depth/image_raw', self.depth_callback, 10)
        self.subscription_right_depth = self.create_subscription(Image, '/camera_right/depth/depth_camera_right/depth/image_raw', self.right_depth_callback, 10)
        self.subscription_left_depth = self.create_subscription(Image, '/camera_left/depth/depth_camera_left/depth/image_raw', self.left_depth_callback, 10)

        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_factor = self.create_publisher(Float32, '/obstacle_speed_factor', 10)
        self.publisher_steering_left = self.create_publisher(Float32, '/steering_correction_left', 10)
        self.publisher_steering_right = self.create_publisher(Float32, '/steering_correction_right', 10)
        self.publisher_steering_center = self.create_publisher(Float32, '/steering_correction_center', 10)

        self.bridge = CvBridge()
        self.depth_frame = None
        self.left_depth_frame = None
        self.right_depth_frame = None

        self.min_depth = 0.3
        self.max_depth = 3.0
        self.max_steering_angle = 0.75
        self.min_speed_factor = 0.0
        self.emergency_distance = 0.5

    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Front depth conversion error: {e}")
            return
        self.process_depth_image()

    def left_depth_callback(self, msg):
        try:
            self.left_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f"Left depth conversion error: {e}")

    def right_depth_callback(self, msg):
        try:
            self.right_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f"Right depth conversion error: {e}")

    def process_depth_image(self):
        front_valid = False
        steering_center = 0.0
        speed_factor = 1.0

        if self.depth_frame is not None:
            height, width = self.depth_frame.shape
            h_start = int(height * 0.15)
            h_end = int(height * 0.85)
            w_start = int(width * 0.15)
            w_end = int(width * 0.85)
            region = self.depth_frame[h_start:h_end, w_start:w_end]
            valid_mask = (region > self.min_depth) & (region < self.max_depth) & np.isfinite(region)

            if np.count_nonzero(valid_mask) > 50:
                y_idxs, x_idxs = np.where(valid_mask)
                depths = region[y_idxs, x_idxs]
                min_dist = float(np.min(depths))

                if min_dist < self.emergency_distance:
                    self.get_logger().warn("Emergency stop: Obstacle < 0.5m")
                    stop = Twist()
                    stop.linear.x = 0.0
                    stop.angular.z = 0.0
                    self.publisher_cmd.publish(stop)
                    return

                mean_x = np.mean(x_idxs)
                center_x = region.shape[1] / 2
                offset = center_x - mean_x
                offset_norm = offset / center_x

                strength = (self.max_depth - min_dist) / self.max_depth
                steering_center = offset_norm * self.max_steering_angle * strength
                steering_center = float(np.clip(steering_center, -self.max_steering_angle, self.max_steering_angle))

                if min_dist < 0.7:
                    speed_factor = self.min_speed_factor
                elif min_dist < 1.6:
                    speed_factor = max(self.min_speed_factor, (min_dist - 0.3) / (1.6 - 0.3))
                else:
                    speed_factor = 1.0

                front_valid = True

        self.publisher_factor.publish(Float32(data=speed_factor))

        if front_valid:
            self.publisher_steering_center.publish(Float32(data=steering_center))
            return

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
            avg_depth = float(np.mean(np.sort(depths.flatten())[:200]))
            if avg_depth < 1.0:
                strength = (1.0 - avg_depth) / 1.0
                return strength * self.max_steering_angle
            return 0.0
        except Exception as e:
            self.get_logger().warn(f"Failed to process {'left' if is_left else 'right'} side: {e}")
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = GazeboDodgingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
