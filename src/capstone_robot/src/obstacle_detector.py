#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
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
        self.cmd_sub = self.create_subscription(
            Twist,
            '/teleop_cmd_vel',
            self.cmd_callback,
            10
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            '/ackermann_steering_controller/reference_unstamped',
            10
        )

        # Thresholds in meters
        self.CLEAR = 2.0
        self.NEARBY = 1.0
        self.CRITICAL = 0.5

        # Speed scale factors per state (applies to center-zone state only)
        self.SCALE = {
            "CLEAR": 1.0,
            "NEARBY": 0.5,
            "CLOSE": 0.2,
            "CRITICAL": 0.0,
        }

        # Auto-steer bias magnitude when dodging around a center obstacle
        self.AVOID_TURN = 0.4

        self.zone_state = {"left": "CLEAR", "center": "CLEAR", "right": "CLEAR"}
        self.zone_dist = {"left": float('inf'), "center": float('inf'), "right": float('inf')}
        self.last_cmd = Twist()

        self.get_logger().info("Obstacle detector running (zone-based recognition + speed control active)...")

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h, w = depth_image.shape

        # Vertical crop stays the same (avoid floor/ceiling noise)
        row_slice = slice(h // 3, 2 * h // 3)

        # Split width into left / center / right thirds
        zones = {
            "left": depth_image[row_slice, 0:w // 3],
            "center": depth_image[row_slice, w // 3:2 * w // 3],
            "right": depth_image[row_slice, 2 * w // 3:w],
        }

        for name, region in zones.items():
            valid = region[np.isfinite(region) & (region > 0)]
            if len(valid) == 0:
                continue
            dist = float(np.min(valid))
            self.zone_dist[name] = dist
            self.zone_state[name] = self.classify(dist)

        self.get_logger().info(
            f'L: {self.zone_dist["left"]:.2f}m ({self.zone_state["left"]}) | '
            f'C: {self.zone_dist["center"]:.2f}m ({self.zone_state["center"]}) | '
            f'R: {self.zone_dist["right"]:.2f}m ({self.zone_state["right"]})'
        )

        self.publish_scaled_cmd()

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.publish_scaled_cmd()

    def zone_rank(self, state):
        order = {"CLEAR": 3, "NEARBY": 2, "CLOSE": 1, "CRITICAL": 0}
        return order[state]

    def publish_scaled_cmd(self):
        center_state = self.zone_state["center"]
        scale = self.SCALE[center_state]

        out = Twist()
        out.linear.x = self.last_cmd.linear.x * scale

        # Default: respect the driver's own steering input
        out.angular.z = self.last_cmd.angular.z

        # Auto-avoid: only kicks in if center is blocked AND driver isn't already steering
        if center_state in ("CLOSE", "CRITICAL") and abs(self.last_cmd.angular.z) < 0.01:
            left_rank = self.zone_rank(self.zone_state["left"])
            right_rank = self.zone_rank(self.zone_state["right"])

            if left_rank > right_rank:
                out.angular.z = self.AVOID_TURN   # steer toward open left
            elif right_rank > left_rank:
                out.angular.z = -self.AVOID_TURN  # steer toward open right
            # if both sides equally blocked/open, no bias — just slow/stop as before

        self.cmd_pub.publish(out)

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
