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

# Must match data_logger.py's sentinel for "no reading".
NO_READING = 10.0


class LearnedMLP:
    """Loads the weights + normalization stats saved by train_avoidance_model.py
    and runs the same forward pass, in plain NumPy."""

    def __init__(self, npz_path):
        d = np.load(npz_path, allow_pickle=True)
        self.W1, self.b1 = d['W1'], d['b1']
        self.W2, self.b2 = d['W2'], d['b2']
        self.W3, self.b3 = d['W3'], d['b3']
        self.feature_mean, self.feature_std = d['feature_mean'], d['feature_std']
        self.label_mean, self.label_std = d['label_mean'], d['label_std']
        self.feature_cols = list(d['feature_cols'])

    def predict(self, features):
        x = np.array([features], dtype=np.float64)
        xn = (x - self.feature_mean) / self.feature_std
        h1 = np.maximum(xn @ self.W1 + self.b1, 0)
        h2 = np.maximum(h1 @ self.W2 + self.b2, 0)
        out = h2 @ self.W3 + self.b3
        out = out * self.label_std + self.label_mean
        return float(out[0][0]), float(out[0][1])


class LearnedAvoider(Node):
    def __init__(self):
        super().__init__('learned_avoider')
        self.bridge = CvBridge()

        model_dir = os.path.join(
            get_package_share_directory('capstone_robot'), 'models'
        )
        self.mlp = LearnedMLP(os.path.join(model_dir, 'avoidance_mlp.npz'))
        self.get_logger().info(f'Loaded learned model, features={self.mlp.feature_cols}')

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

        # Hard safety floor -- the learned model's person-slowdown behavior is
        # soft/approximate (see training sanity checks), so a person within
        # this range always forces a stop regardless of what the model predicts.
        self.STOP_DISTANCE = 1.5

        # Output clamps matching the physical/logged ranges (see train_avoidance_model.py
        # data: linear_x in [-1,1], angular_z in [-0.5,0.5]) -- a safety net against
        # the model extrapolating out of range on unfamiliar inputs.
        self.LINEAR_CLAMP = (-1.0, 1.0)
        self.ANGULAR_CLAMP = (-0.5, 0.5)

        self.zone_dist = {'left': NO_READING, 'center': NO_READING, 'right': NO_READING}
        self.person_distance = None
        self.last_teleop = Twist()

        self.cmd_sub = self.create_subscription(
            Twist, '/teleop_cmd_vel', self.cmd_callback, 10
        )
        self.cmd_pub = self.create_publisher(
            Twist, '/ackermann_steering_controller/reference_unstamped', 10
        )

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

        rgb_sub = Subscriber(self, Image, '/camera/color/color_camera/image_raw')
        depth_sub = Subscriber(self, Image, '/camera/depth/depth_camera/depth/image_raw')
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.person_detect_callback)

        self.get_logger().info(
            f'Learned avoider running (safety stop @ {self.STOP_DISTANCE}m)...'
        )

    def depth_zone_callback(self, zone, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h, w = depth_image.shape
        region = depth_image[h // 3:2 * h // 3, :]
        valid = region[np.isfinite(region) & (region > 0)]
        self.zone_dist[zone] = float(np.min(valid)) if len(valid) else NO_READING
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

    def cmd_callback(self, msg):
        self.last_teleop = msg
        self.publish_cmd()

    def publish_cmd(self):
        # Hard safety floor: a close person always blocks forward motion,
        # regardless of what the learned model predicts. Reverse is still
        # allowed -- backing away from something too close is the safe
        # response, and blocking it entirely would deadlock an Ackermann-
        # steered car (it can't turn in place to escape without rolling
        # forward, which would otherwise be the only other way out).
        if self.person_distance is not None and self.person_distance < self.STOP_DISTANCE:
            out = Twist()
            out.angular.z = self.last_teleop.angular.z
            out.linear.x = min(self.last_teleop.linear.x, 0.0)
            self.get_logger().warn(
                f'STOPPING (forward blocked): person at {self.person_distance:.2f}m '
                f'(< {self.STOP_DISTANCE}m)',
                throttle_duration_sec=1.0,
            )
            self.cmd_pub.publish(out)
            return

        features = [
            self.zone_dist['left'], self.zone_dist['center'], self.zone_dist['right'],
            self.person_distance if self.person_distance is not None else NO_READING,
            1.0 if self.person_distance is not None else 0.0,
            self.last_teleop.linear.x, self.last_teleop.angular.z,
        ]
        linear_x, angular_z = self.mlp.predict(features)
        linear_x = float(np.clip(linear_x, *self.LINEAR_CLAMP))
        angular_z = float(np.clip(angular_z, *self.ANGULAR_CLAMP))

        out = Twist()
        out.linear.x = linear_x
        out.angular.z = angular_z

        self.get_logger().info(
            f"L: {self.zone_dist['left']:.2f}m | C: {self.zone_dist['center']:.2f}m | "
            f"R: {self.zone_dist['right']:.2f}m -> linear={linear_x:.2f} angular={angular_z:.2f}",
            throttle_duration_sec=1.0,
        )
        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LearnedAvoider()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
