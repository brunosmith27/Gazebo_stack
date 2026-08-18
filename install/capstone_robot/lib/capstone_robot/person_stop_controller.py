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


class PersonStopController(Node):
    def __init__(self):
        super().__init__('person_stop_controller')
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
        self.STOP_DISTANCE = 1.5  # meters: stop if a person is closer than this

        self.last_cmd = Twist()
        self.person_distance = None  # None means no person currently in view

        self.cmd_sub = self.create_subscription(
            Twist, '/teleop_cmd_vel', self.cmd_callback, 10
        )
        self.cmd_pub = self.create_publisher(
            Twist, '/ackermann_steering_controller/reference_unstamped', 10
        )

        rgb_sub = Subscriber(self, Image, '/camera/color/color_camera/image_raw')
        depth_sub = Subscriber(self, Image, '/camera/depth/depth_camera/depth/image_raw')
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_callback)

        self.get_logger().info(
            f'Person stop controller running (stop distance = {self.STOP_DISTANCE}m)...'
        )

    def image_callback(self, rgb_msg, depth_msg):
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
        self.last_cmd = msg
        self.publish_cmd()

    def publish_cmd(self):
        out = Twist()
        out.angular.z = self.last_cmd.angular.z

        if self.person_distance is not None and self.person_distance < self.STOP_DISTANCE:
            # Block forward motion but allow reverse -- an unconditional stop
            # can deadlock an Ackermann-steered car, since it can't turn in
            # place to escape without rolling forward.
            out.linear.x = min(self.last_cmd.linear.x, 0.0)
            self.get_logger().warn(
                f'STOPPING (forward blocked): person at {self.person_distance:.2f}m '
                f'(< {self.STOP_DISTANCE}m)',
                throttle_duration_sec=1.0,
            )
        else:
            out.linear.x = self.last_cmd.linear.x

        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PersonStopController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
