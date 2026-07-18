#!/usr/bin/env python3
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image


class ObjectDistanceViewer(Node):
    def __init__(self):
        super().__init__('object_distance_viewer')
        self.bridge = CvBridge()

        model_dir = os.path.join(
            get_package_share_directory('capstone_robot'), 'models'
        )
        weights = os.path.join(model_dir, 'yolov4-tiny.weights')
        cfg = os.path.join(model_dir, 'yolov4-tiny.cfg')
        names = os.path.join(model_dir, 'coco.names')

        with open(names, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]

        self.net = cv2.dnn.readNetFromDarknet(cfg, weights)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.output_layers = self.net.getUnconnectedOutLayersNames()

        self.CONF_THRESHOLD = 0.5
        self.NMS_THRESHOLD = 0.4

        rgb_sub = Subscriber(self, Image, '/camera/color/color_camera/image_raw')
        depth_sub = Subscriber(self, Image, '/camera/depth/depth_camera/depth/image_raw')
        self.sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.image_callback)

        self.get_logger().info('Object distance viewer running (YOLOv4-tiny)...')

    def image_callback(self, rgb_msg, depth_msg):
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        h, w = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)

        boxes, confidences, class_ids = [], [], []
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = int(np.argmax(scores))
                confidence = float(scores[class_id])
                if confidence < self.CONF_THRESHOLD:
                    continue
                cx, cy, bw, bh = (detection[0:4] * np.array([w, h, w, h])).astype(int)
                x, y = int(cx - bw / 2), int(cy - bh / 2)
                boxes.append([x, y, int(bw), int(bh)])
                confidences.append(confidence)
                class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.CONF_THRESHOLD, self.NMS_THRESHOLD)
        detections = []

        for i in np.array(indices).flatten() if len(indices) else []:
            x, y, bw, bh = boxes[i]
            x0, y0 = max(x, 0), max(y, 0)
            x1, y1 = min(x + bw, w), min(y + bh, h)
            region = depth[y0:y1, x0:x1]
            valid = region[np.isfinite(region) & (region > 0)]
            dist = float(np.median(valid)) if len(valid) else float('nan')

            label = self.classes[class_ids[i]]
            dist_text = f'{dist:.2f}m' if np.isfinite(dist) else 'unknown'
            detections.append(f'{label} {dist_text}')

            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2.putText(
                frame, f'{label} {dist_text}', (x0, max(y0 - 8, 0)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
            )

        summary = ', '.join(detections) if detections else 'no objects detected'
        self.get_logger().info(f'frame received: {summary}', throttle_duration_sec=1.0)

        cv2.imshow('Object Distance Viewer', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDistanceViewer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
