#!/usr/bin/env python3
import argparse
import csv
import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image

# Sentinel distance (meters) written when a zone/person has no valid reading --
# matches the depth sensor's own far-clip convention elsewhere in this package.
NO_READING = 10.0

CSV_HEADER = [
    'timestamp',
    'left_dist', 'center_dist', 'right_dist',
    'person_dist', 'person_present',
    'teleop_linear_x', 'teleop_angular_z',
    'cmd_linear_x', 'cmd_angular_z',
    'controller_source',
]

COMMAND_TOPIC = '/ackermann_steering_controller/reference_unstamped'


class DataLogger(Node):
    def __init__(self, output_path):
        super().__init__('data_logger')
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

        self.zone_dist = {'left': NO_READING, 'center': NO_READING, 'right': NO_READING}
        self.person_distance = None
        self.last_teleop = Twist()

        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        is_new = not os.path.exists(output_path)
        self.csv_file = open(output_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if is_new:
            self.csv_writer.writerow(CSV_HEADER)
            self.csv_file.flush()
        self.row_count = 0

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

        self.create_subscription(Twist, '/teleop_cmd_vel', self.teleop_callback, 10)
        # Log one row each time the active controller publishes a decision --
        # that's the actual (state -> action) pair we want to imitate.
        self.create_subscription(Twist, COMMAND_TOPIC, self.cmd_callback, 10)

        self.get_logger().info(f'Data logger running, writing to {output_path}')

    def classify_zone_distance(self, depth_image):
        h, w = depth_image.shape
        region = depth_image[h // 3:2 * h // 3, :]
        valid = region[np.isfinite(region) & (region > 0)]
        return float(np.min(valid)) if len(valid) else NO_READING

    def depth_zone_callback(self, zone, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.zone_dist[zone] = self.classify_zone_distance(depth_image)

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

    def teleop_callback(self, msg):
        self.last_teleop = msg

    def resolve_controller_source(self):
        infos = self.get_publishers_info_by_topic(COMMAND_TOPIC)
        names = {i.node_name for i in infos}
        if len(names) == 1:
            return next(iter(names))
        if len(names) == 0:
            return 'unknown'
        # More than one node publishing the command topic at once shouldn't
        # happen (every avoider node's docstring says run only one at a time),
        # but record it plainly rather than silently picking one.
        return 'multiple:' + '+'.join(sorted(names))

    def cmd_callback(self, msg):
        row = [
            time.time(),
            self.zone_dist['left'], self.zone_dist['center'], self.zone_dist['right'],
            self.person_distance if self.person_distance is not None else NO_READING,
            1 if self.person_distance is not None else 0,
            self.last_teleop.linear.x, self.last_teleop.angular.z,
            msg.linear.x, msg.angular.z,
            self.resolve_controller_source(),
        ]
        self.csv_writer.writerow(row)
        self.csv_file.flush()
        self.row_count += 1
        if self.row_count % 20 == 0:
            self.get_logger().info(f'{self.row_count} rows logged', throttle_duration_sec=2.0)

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='Log (sensor state -> driving command) pairs for imitation learning.')
    parser.add_argument(
        '--output', default=None,
        help='CSV path to append to (default: ~/capstone_avoidance_data/log_<timestamp>.csv)'
    )
    parsed, ros_args = parser.parse_known_args()

    output_path = parsed.output
    if output_path is None:
        output_path = os.path.expanduser(
            f'~/capstone_avoidance_data/log_{int(time.time())}.csv'
        )

    rclpy.init(args=ros_args)
    node = DataLogger(output_path)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
