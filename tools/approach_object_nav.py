#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from tf2_ros import Buffer, TransformListener

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from ultralytics import YOLO
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rokey_pjt.model_paths import get_default_model_path, resolve_model_path

import numpy as np
import cv2
import math
import time


class ApproachObjectNav(Node):
    def __init__(self):
        super().__init__('approach_object_nav')

        self.declare_parameter('model_path', get_default_model_path())
        model_path = resolve_model_path(str(self.get_parameter('model_path').value))
        self.model = YOLO(model_path)
        self.target_class_id = 1  # {0: 'box', 1: 'car'}

        ns = self.get_namespace()
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.depth_topic = f'{ns}/oakd/stereo/image_raw/compressedDepth'

        # 카메라 intrinsics (704x704 기준, 앞서 확인한 값 — camera_info로 갱신도 가능)
        self.fx, self.fy = 564.6512451171875, 564.6512451171875
        self.ppx, self.ppy = 353.1598815917969, 355.361572265625

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Nav2 (TurtleBot4Navigator, 강사님 자료 스타일) ----
        self.navigator = TurtleBot4Navigator()
        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        # ---- 상태 관리 ----
        self.approach_distance = 0.20   # 목표 접근 거리(m) — 20cm
        self.block_goal_updates = False  # 접근 완료 후 더 이상 갱신 안 함 (3_2_d 패턴)
        self.close_hit_count = 0
        self.goal_in_progress = False
        self.last_goal_time = 0.0
        self.goal_cooldown = 5.0
        self.min_goal_move = 0.15
        self.last_goal_xy = None

        # ---- 카메라 구독 (RGB + compressedDepth, 시간 동기화) ----
        self.rgb_sub = Subscriber(self, CompressedImage, self.rgb_topic)
        self.depth_sub = Subscriber(self, CompressedImage, self.depth_topic)
        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.15
        )
        self.ts.registerCallback(self.synced_callback)

        self.get_logger().info('Approach Object Nav node started. Waiting for detections...')

    # -------------------------------------------------
    def decode_compressed_depth(self, msg):
        """ compressed_depth_image_transport 포맷: 12바이트 헤더 + PNG """
        try:
            depth_header_size = 12
            raw_data = msg.data[depth_header_size:]
            np_arr = np.frombuffer(raw_data, np.uint8)
            depth_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            return depth_img
        except Exception as e:
            self.get_logger().warn(f'Depth decode failed: {e}')
            return None

    # -------------------------------------------------
    def synced_callback(self, rgb_msg, depth_msg):
        if self.goal_in_progress or self.block_goal_updates:
            return

        now = time.time()
        if now - self.last_goal_time < self.goal_cooldown:
            return

        # RGB 디코딩
        np_arr = np.frombuffer(rgb_msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        # Depth 디코딩
        depth = self.decode_compressed_depth(depth_msg)
        if depth is None:
            return

        camera_frame = depth_msg.header.frame_id  # oakd_rgb_camera_optical_frame

        # YOLO 검출
        results = self.model(frame, verbose=False)
        boxes = results[0].boxes
        target_boxes = [b for b in boxes if int(b.cls[0]) == self.target_class_id]
        if not target_boxes:
            return

        best = max(target_boxes, key=lambda b: float(b.conf[0]))
        x1, y1, x2, y2 = best.xyxy[0].cpu().numpy()
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

        if cy >= depth.shape[0] or cx >= depth.shape[1]:
            return

        patch = depth[max(0, cy - 3):cy + 3, max(0, cx - 3):cx + 3]
        valid = patch[patch > 0]
        if len(valid) == 0:
            return
        z = float(np.median(valid)) / 1000.0

        if not (0.2 < z < 8.0):
            return

        # ---- 픽셀 -> 카메라 3D 좌표 (역투영) ----
        X = (cx - self.ppx) * z / self.fx
        Y = (cy - self.ppy) * z / self.fy
        Z = z

        pt_camera = PointStamped()
        pt_camera.header.stamp = Time().to_msg()
        pt_camera.header.frame_id = camera_frame
        pt_camera.point.x = X
        pt_camera.point.y = Y
        pt_camera.point.z = Z

        # ---- TF: camera -> map (강사님 자료처럼 'map' 접두어 없이) ----
        try:
            pt_map = self.tf_buffer.transform(pt_camera, 'map', timeout=Duration(seconds=1.0))
            self.get_logger().info(
                f"차량 위치 (map): ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}), 거리={z:.2f}m"
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        self.try_send_goal(pt_map.point.x, pt_map.point.y)

    # -------------------------------------------------
    def try_send_goal(self, obj_x, obj_y):
        if self.last_goal_xy is not None:
            moved = math.hypot(obj_x - self.last_goal_xy[0], obj_y - self.last_goal_xy[1])
            if moved < self.min_goal_move:
                return

        # 로봇 현재 위치 (map 기준) — base_link TF 조회
        try:
            robot_tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            rx = robot_tf.transform.translation.x
            ry = robot_tf.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'Robot pose TF (base_link) failed: {e}')
            return

        dx, dy = obj_x - rx, obj_y - ry
        dist = math.hypot(dx, dy)

        if dist <= self.approach_distance:
            self.get_logger().info(f'이미 목표 거리 이내입니다 (dist={dist:.2f}m). 접근 완료.')
            self.block_goal_updates = True
            return

        # 목표 지점 앞 20cm 지점 계산
        ratio = (dist - self.approach_distance) / dist
        goal_x = rx + dx * ratio
        goal_y = ry + dy * ratio
        yaw = math.atan2(dy, dx)  # 객체를 바라보는 방향

        self.send_goal(goal_x, goal_y, yaw)
        self.last_goal_xy = (obj_x, obj_y)
        self.last_goal_time = time.time()

    # -------------------------------------------------
    def send_goal(self, x, y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        self.goal_in_progress = True
        self.get_logger().info(f'차량 근처(20cm)로 이동 시작: ({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.1f}°')

        # TurtleBot4Navigator.goToPose (강사님 자료 방식, blocking)
        self.navigator.goToPose(goal_pose)

        self.get_logger().info('이동 완료.')
        self.goal_in_progress = False


def main():
    rclpy.init()
    node = ApproachObjectNav()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
