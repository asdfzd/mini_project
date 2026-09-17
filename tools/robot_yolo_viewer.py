#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO
from rokey_pjt.model_paths import get_default_model_path, resolve_model_path


class RobotYoloViewer(Node):
    def __init__(self):
        super().__init__('robot_yolo_viewer')

        self.declare_parameter('model_path', get_default_model_path())
        self.declare_parameter('target_class_id', 1)
        self.model_path = resolve_model_path(str(self.get_parameter('model_path').value))
        self.target_class_id = int(self.get_parameter('target_class_id').value)

        self.model = YOLO(self.model_path)

        self.rgb_topic = '/robot1/oakd/rgb/image_raw/compressed'

        self.sub = self.create_subscription(
            CompressedImage,
            self.rgb_topic,
            self.rgb_callback,
            10
        )

        self.get_logger().info(f'[YOLO VIEWER] model={self.model_path}')
        self.get_logger().info(f'[YOLO VIEWER] target_class_id={self.target_class_id}')
        self.get_logger().info(f'[YOLO VIEWER] topic={self.rgb_topic}')

    def rgb_callback(self, msg):
        frame = self.decode_rgb(msg)

        if frame is None:
            self.get_logger().warn('[YOLO VIEWER] RGB decode 실패', throttle_duration_sec=1.0)
            return

        results = self.model(frame, verbose=False)
        boxes = results[0].boxes

        detected = []

        for b in boxes:
            cls_id = int(b.cls[0])
            conf = float(b.conf[0])
            x1, y1, x2, y2 = map(int, b.xyxy[0].cpu().numpy())

            detected.append((cls_id, round(conf, 2)))

            if cls_id == self.target_class_id:
                color = (0, 255, 0)
                label = f'CAR id={cls_id} conf={conf:.2f}'
            else:
                color = (0, 0, 255)
                label = f'id={cls_id} conf={conf:.2f}'

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame,
                label,
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2
            )

        if detected:
            self.get_logger().info(
                f'[YOLO VIEWER] detected={detected}',
                throttle_duration_sec=1.0
            )

        cv2.imshow('Robot OAK-D YOLO Viewer', frame)
        cv2.waitKey(1)

    def decode_rgb(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            return frame
        except Exception:
            return None

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = RobotYoloViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
