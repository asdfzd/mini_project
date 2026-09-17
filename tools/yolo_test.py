#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
import cv2
from rokey_pjt.model_paths import get_default_model_path, resolve_model_path


class YoloTestNode(Node):
    def __init__(self):
        super().__init__('yolo_test_node')
        self.bridge = CvBridge()
        self.declare_parameter('model_path', get_default_model_path())
        model_path = resolve_model_path(str(self.get_parameter('model_path').value))
        self.model = YOLO(model_path)

        ns = self.get_namespace()

        # RGB, CompressedDepth 동기화 구독
        self.rgb_sub = Subscriber(self, CompressedImage, f'{ns}/oakd/rgb/image_raw/compressed')
        self.depth_sub = Subscriber(self, CompressedImage, f'{ns}/oakd/stereo/image_raw/compressedDepth')

        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.30
        )
        self.ts.registerCallback(self.callback)

        self.pub = self.create_publisher(Image, f'{ns}/yolo_detections', 1)

        self.get_logger().info('YOLO + CompressedDepth (time-synced) test node started.')
        self.get_logger().info(f'Subscribing RGB: {ns}/oakd/rgb/image_raw/compressed')
        self.get_logger().info(f'Subscribing Depth: {ns}/oakd/stereo/image_raw/compressedDepth')

    def decode_compressed_depth(self, msg):
        """
        compressed_depth_image_transport 포맷 디코딩
        구조: [12바이트 헤더(float32 depthQuantA, depthQuantB 등)] + [PNG 압축 데이터]
        """
        try:
            # 앞 12바이트는 압축 설정 헤더 (depthParam 등), 그 이후가 실제 PNG 데이터
            depth_header_size = 12
            raw_data = msg.data[depth_header_size:]
            np_arr = np.frombuffer(raw_data, np.uint8)
            depth_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)  # 16UC1 PNG 디코딩
            return depth_img
        except Exception as e:
            self.get_logger().warn(f'Depth decode failed: {e}')
            return None

    def callback(self, rgb_msg, depth_msg):
        rgb_t = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        depth_t = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9
        dt = abs(rgb_t - depth_t)

        # RGB 디코딩
        np_arr = np.frombuffer(rgb_msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        # Depth 디코딩 (compressedDepth 전용)
        depth = self.decode_compressed_depth(depth_msg)
        if depth is None:
            return

        results = self.model(frame, verbose=False)
        boxes = results[0].boxes

        annotated = frame.copy()

        for box in boxes:
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

            # 해상도가 RGB와 다를 수 있으니 범위 체크
            if cy >= depth.shape[0] or cx >= depth.shape[1]:
                continue

            patch = depth[max(0, cy - 3):cy + 3, max(0, cx - 3):cx + 3]
            valid = patch[patch > 0]
            if len(valid) > 0:
                distance_m = float(np.median(valid)) / 1000.0
                dist_text = f'{distance_m:.2f}m'
            else:
                dist_text = 'N/A'

            self.get_logger().info(
                f'검출: {cls_name} (conf={conf:.2f}), 거리={dist_text}, sync_dt={dt*1000:.1f}ms'
            )

            cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            label = f'{cls_name} {conf:.2f} {dist_text}'
            cv2.putText(annotated, label, (int(x1), max(0, int(y1) - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = YoloTestNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
