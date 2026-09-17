#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from ultralytics import YOLO
import cv2

from rokey_pjt.model_paths import get_default_model_path, resolve_model_path


class WebcamTrigger(Node):
    def __init__(self):
        super().__init__('webcam_trigger')

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Bool, '/robot1/start_mission', qos)

        self.declare_parameter('model_path', get_default_model_path())
        self.declare_parameter('target_class_id', 1)

        self.model_path = resolve_model_path(str(self.get_parameter('model_path').value))
        self.car_class_id = int(self.get_parameter('target_class_id').value)
        self.model = YOLO(self.model_path)

        self.cap = cv2.VideoCapture(2)

        self.triggered = False
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info(f'YOLO model: {self.model_path}')
        self.get_logger().info('External webcam device index: 2')

    def timer_callback(self):
        if self.triggered:
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        results = self.model(frame, verbose=False)
        boxes = results[0].boxes
        car_detected = any(int(b.cls[0]) == self.car_class_id for b in boxes)

        if car_detected:
            msg = Bool()
            msg.data = True

            for _ in range(10):
                self.pub.publish(msg)

            self.get_logger().info('웹캠에서 차량 검출! 미션 시작 트리거 전송.')
            self.triggered = True


def main():
    rclpy.init()
    node = WebcamTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
