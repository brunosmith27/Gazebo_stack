#!/usr/bin/env python3
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image


class FusedObstacleAvoider(Node):
    def __init__(self):
        super().__init__('fused_obstacle_avoider')
        self.bridge = CvBridge()

        model_dir = os.path.join(
            get_package_share_directory('capstone_robot'), 'models'
        )
        self.net = cv2.dnn.readNetFromDarknet(
            os.path.join(model_dir, 'yolov4-tiny.cfg'),
            os.path.join(model_dir, 'yolov4-tiny.weights'),
        )
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.output_layers = self.net.getUnconnectedOutLayersNames()
        with open(os.path.join(model_dir, 'coco.names'), 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.person_class_id = self.classes.index('person')
        self.CONF_THRESHOLD = 0.5
        self.NMS_THRESHOLD = 0.4
        self.STOP_DISTANCE = 1.5  # meters: hard stop if a person is closer than this

        # Obstacle-avoidance thresholds in meters (mirrors obstacle_detector.py)
        self.CLEAR = 2.0
        self.NEARBY = 1.0
        self.CRITICAL = 0.5
        self.SCALE = {'CLEAR': 1.0, 'NEARBY': 0.5, 'CLOSE': 0.2, 'CRITICAL': 0.0}
        self.AVOID_TURN = 0.4

        self.zone_state = {'left': 'CLEAR', 'center': 'CLEAR', 'right': 'CLEAR'}
        self.zone_dist = {'left': float('inf'), 'center': float('inf'), 'right': float('inf')}
        self.last_cmd = Twist()
        self.person_distance = None

        self.cmd_sub = self.create_subscription(
            Twist, '/teleop_cmd_vel', self.cmd_callback, 10
        )
        self.cmd_pub = self.create_publisher(
            Twist, '/ackermann_steering_controller/reference_unstamped', 10
        )

        # Three real, physically separate depth cameras -- each updates its own
        # zone independently and immediately, rather than waiting on the others.
        self.create_subscription(
            Image, '/camera_left/depth/depth_camera_left/depth/image_raw',
            lambda msg: self.depth_zone_callback('left', msg), 10
        )
        self.create_subscription(
            Image, '/camera/depth/depth_camera/depth/image_raw',
            lambda msg: self.depth_zone_callback('center', msg), 10
        )
        self.create_subscription(
            Image, '/camera_right/depth/depth_camera_right/depth/image_raw',
            lambda msg: self.depth_zone_callback('right', msg), 10
        )

        # RGB + center depth, synced, purely for YOLO person detection.
        rgb_sub = Subscriber(self, Image, '/camera/color/color_camera/image_raw')
        depth_sub = Subscriber(self, Image, '/camera/depth/depth_camera/depth/image_raw')
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.person_detect_callback)

        self.get_logger().info(
            f'Fused obstacle avoider running (3-camera zones + YOLO person stop @ '
            f'{self.STOP_DISTANCE}m)...'
        )

    def classify(self, distance):
        if distance > self.CLEAR:
            return 'CLEAR'
        elif distance > self.NEARBY:
            return 'NEARBY'
        elif distance > self.CRITICAL:
            return 'CLOSE'
        else:
            return 'CRITICAL'

    def depth_zone_callback(self, zone, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h, w = depth_image.shape
        region = depth_image[h // 3:2 * h // 3, :]
        valid = region[np.isfinite(region) & (region > 0)]
        if len(valid) == 0:
            return
        dist = float(np.min(valid))
        self.zone_dist[zone] = dist
        self.zone_state[zone] = self.classify(dist)
        self.publish_cmd()

    def person_detect_callback(self, rgb_msg, depth_msg):
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        h, w = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)

        boxes, confidences = [], []
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = int(np.argmax(scores))
                confidence = float(scores[class_id])
                if class_id != self.person_class_id or confidence < self.CONF_THRESHOLD:
                    continue
                cx, cy, bw, bh = (detection[0:4] * np.array([w, h, w, h])).astype(int)
                x, y = int(cx - bw / 2), int(cy - bh / 2)
                boxes.append([x, y, int(bw), int(bh)])
                confidences.append(confidence)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.CONF_THRESHOLD, self.NMS_THRESHOLD)

        distances = []
        for i in np.array(indices).flatten() if len(indices) else []:
            x, y, bw, bh = boxes[i]
            x0, y0 = max(x, 0), max(y, 0)
            x1, y1 = min(x + bw, w), min(y + bh, h)
            region = depth[y0:y1, x0:x1]
            valid = region[np.isfinite(region) & (region > 0)]
            if len(valid):
                distances.append(float(np.median(valid)))

        self.person_distance = min(distances) if distances else None
        if self.person_distance is not None:
            self.get_logger().info(
                f'person at {self.person_distance:.2f}m', throttle_duration_sec=1.0
            )
        self.publish_cmd()

    def zone_rank(self, state):
        order = {'CLEAR': 3, 'NEARBY': 2, 'CLOSE': 1, 'CRITICAL': 0}
        return order[state]

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.publish_cmd()

    def publish_cmd(self):
        # Safety override: a person closer than STOP_DISTANCE always wins -- full
        # stop, no avoidance steering attempted around a person.
        if self.person_distance is not None and self.person_distance < self.STOP_DISTANCE:
            out = Twist()
            out.angular.z = self.last_cmd.angular.z
            out.linear.x = 0.0
            self.get_logger().warn(
                f'STOPPING: person at {self.person_distance:.2f}m (< {self.STOP_DISTANCE}m)',
                throttle_duration_sec=1.0,
            )
            self.cmd_pub.publish(out)
            return

        # Otherwise, fuse the three real depth cameras: scale forward speed by
        # how blocked the center camera is, and auto-steer toward whichever
        # side (left/right camera) is more open if the center is blocked and
        # the driver isn't already steering.
        center_state = self.zone_state['center']
        scale = self.SCALE[center_state]

        out = Twist()
        out.linear.x = self.last_cmd.linear.x * scale
        out.angular.z = self.last_cmd.angular.z

        if center_state in ('NEARBY', 'CLOSE', 'CRITICAL') and abs(self.last_cmd.angular.z) < 0.01:
            left_rank = self.zone_rank(self.zone_state['left'])
            right_rank = self.zone_rank(self.zone_state['right'])
            if left_rank > right_rank:
                out.angular.z = self.AVOID_TURN
            elif right_rank > left_rank:
                out.angular.z = -self.AVOID_TURN
            elif left_rank >= self.zone_rank('NEARBY'):
                # tied but there's real room on both sides -- just pick one
                # (right) rather than idling straight at the obstacle
                out.angular.z = -self.AVOID_TURN

        self.get_logger().info(
            f"L: {self.zone_dist['left']:.2f}m ({self.zone_state['left']}) | "
            f"C: {self.zone_dist['center']:.2f}m ({self.zone_state['center']}) | "
            f"R: {self.zone_dist['right']:.2f}m ({self.zone_state['right']})",
            throttle_duration_sec=1.0,
        )
        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FusedObstacleAvoider()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
