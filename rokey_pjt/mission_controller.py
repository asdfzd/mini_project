#!/usr/bin/env python3

import math
import time
from enum import Enum
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time

from std_msgs.msg import Bool
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from tf2_ros import Buffer, TransformException, TransformListener

try:
    # PointStamped TF 변환 등록용. 환경에 따라 직접 참조하지 않아도 import가 필요할 수 있다.
    import tf2_geometry_msgs  # noqa: F401
except Exception:
    tf2_geometry_msgs = None

from message_filters import ApproximateTimeSynchronizer, Subscriber

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Navigator,
    TurtleBot4Directions,
)

from ultralytics import YOLO

from rokey_pjt.model_paths import get_default_model_path, resolve_model_path


class State(Enum):
    IDLE = 0
    NAVIGATING = 1
    SCANNING = 2
    CENTERING = 3
    APPROACHING = 4
    NAV_TO_OBJECT = 5
    TRACKING = 6
    DONE = 7


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # =========================
        # 기본 설정
        # =========================
        self.namespace = self.get_namespace()
        self.ns_clean = self.namespace.strip('/')

        if self.namespace == '/':
            self.namespace = '/robot1'
            self.ns_clean = 'robot1'

        self.state = State.IDLE

        # 기존에 실제로 도착까지 성공했던 좌표 유지
        self.SCAN_POINT = [-1.9688931703567505, 0.21015524864196777]
        self.SCAN_DIRECTION = TurtleBot4Directions.NORTH

        # YOLO 모델 경로
        self.declare_parameter(
            'model_path',
            get_default_model_path()
        )
        self.declare_parameter('target_class_id', 1)
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('stop_distance', 0.75)
        self.declare_parameter('min_goal_interval', 2.0)
        self.declare_parameter('replan_distance_threshold', 0.25)

        self.model_path = resolve_model_path(str(self.get_parameter('model_path').value))
        self.target_class_id = int(self.get_parameter('target_class_id').value)
        self.camera_frame_override = str(self.get_parameter('camera_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.nav_stop_distance = float(self.get_parameter('stop_distance').value)
        self.nav_goal_min_interval = float(self.get_parameter('min_goal_interval').value)
        self.replan_distance_threshold = float(self.get_parameter('replan_distance_threshold').value)

        if not Path(self.model_path).exists():
            self.get_logger().error(f'[모델 없음] YOLO 모델 파일을 찾을 수 없습니다: {self.model_path}')
            raise FileNotFoundError(self.model_path)

        self.get_logger().info(f'[YOLO] model_path={self.model_path}')
        self.get_logger().info(f'[YOLO] target_class_id={self.target_class_id}')

        self.model = YOLO(self.model_path)

        # =========================
        # 카메라 / Depth / CameraInfo
        # =========================
        self.rgb_topic = f'/{self.ns_clean}/oakd/rgb/image_raw/compressed'
        self.depth_topic = f'/{self.ns_clean}/oakd/stereo/image_raw/compressedDepth'
        self.camera_info_topic = f'/{self.ns_clean}/oakd/stereo/camera_info'
        self.cmd_vel_topic = f'/{self.ns_clean}/cmd_vel'

        # 강사님 hint 반영: RGB와 Depth를 시간 동기화해서 같은 callback에서 처리
        self.rgb_sub = Subscriber(self, CompressedImage, self.rgb_topic)
        self.depth_sub = Subscriber(self, CompressedImage, self.depth_topic)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=30,
            slop=0.50,
            allow_headerless=True,
        )
        self.sync.registerCallback(self.camera_callback)

        # CameraInfo로 fx, fy, ppx, ppy를 받는다.
        self.fx = None
        self.fy = None
        self.ppx = None
        self.ppy = None
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        # =========================
        # TF
        # =========================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # base frame 후보. 실제 TF tree에 맞는 것을 자동으로 찾는다.
        self.base_frame_candidates = [
            f'{self.ns_clean}/base_link',
            f'{self.ns_clean}/base_footprint',
            'base_link',
            'base_footprint',
        ]

        # =========================
        # Nav2
        # =========================
        self.navigator = TurtleBot4Navigator()
        self.nav_goal_active = False
        self.last_nav_goal_time = 0.0
        self.last_object_map_xy = None
        self.last_goal_xy = None
        self.nav_to_object_timer = self.create_timer(0.2, self.nav_to_object_timer_callback)

        # =========================
        # 스캔 / 정렬 / 접근 파라미터
        # =========================
        self.scan_angular_speed = 0.15
        self.scan_direction = 1
        self.scan_start_time = time.time()
        self.scan_full_turn_sec = (2.0 * math.pi) / self.scan_angular_speed
        self.scan_turn_count = 0

        self.center_kp = 0.4
        self.center_tolerance = 0.08
        self.center_max_angular = 0.35

        # Nav2 접근 이후 가까운 거리 tracking에서 사용하는 기준
        self.approach_target_distance = 0.75
        self.approach_stop_distance = 0.68
        self.approach_kp_linear = 0.35
        self.approach_max_linear = 0.12
        self.approach_max_angular = 0.15

        self.stationary_distance_min = 0.70
        self.stationary_distance_max = 0.80
        self.stationary_confirm_frames = 5
        self.stationary_count = 0

        # Depth 튐 방지 기준
        self.max_valid_target_depth = 2.0

        self.tracking_lost_count = 0
        self.tracking_max_lost = 30

        # =========================
        # YOLO car 판정 조건
        # =========================
        self.target_conf_threshold = 0.50
        self.target_confirm_frames = 5
        self.target_confirm_count = 0

        self.last_yolo_log_time = 0.0
        self.last_tf_warn_time = 0.0

        # 스캔 회전은 카메라 콜백과 분리해서 timer로 계속 수행
        self.scan_timer = self.create_timer(0.1, self.scan_timer_callback)

        # 웹캠 트리거 대기
        self.mission_started = False
        start_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.start_sub = self.create_subscription(
            Bool,
            '/robot1/start_mission',
            self.start_mission_callback,
            start_qos
        )

        self.get_logger().info('[준비 완료] mission_controller 시작됨.')
        self.get_logger().info('[구조] 웹캠 트리거를 기다립니다.')
        self.get_logger().info(f'[토픽] RGB={self.rgb_topic}')
        self.get_logger().info(f'[토픽] Depth={self.depth_topic}')
        self.get_logger().info(f'[토픽] CameraInfo={self.camera_info_topic}')
        self.get_logger().info(f'[토픽] cmd_vel={self.cmd_vel_topic}')
        self.get_logger().info('[구조] RGB/Depth 동기화 + TF2 map 좌표 변환 + Nav2 접근 + 근거리 cmd_vel tracking')
        self.get_logger().info(
            f'[Nav2 갱신 조건] min_goal_interval={self.nav_goal_min_interval:.1f}s, '
            f'replan_distance_threshold={self.replan_distance_threshold:.2f}m'
        )

    # =========================================================
    # 웹캠 트리거 수신
    # =========================================================
    def start_mission_callback(self, msg):
        self.get_logger().info(f'[트리거 콜백 진입] msg={msg.data}')

        if self.mission_started:
            self.get_logger().info(
                '[트리거 무시] 이미 미션이 시작되었습니다.',
                throttle_duration_sec=2.0
            )
            return

        if not msg.data:
            self.get_logger().info('[트리거 무시] msg.data=False')
            return

        self.mission_started = True
        self.get_logger().info('[트리거 수신] 웹캠에서 car 검출. 미션을 시작합니다.')
        self.start_mission()

    def start_mission(self):
        try:
            if not self.navigator.getDockedStatus():
                self.get_logger().info('[초기 상태] 이미 dock 해제 상태입니다.')
            else:
                self.get_logger().info('[Undock] 도킹 상태입니다. undock을 수행합니다.')

            initial_pose = self.navigator.getPoseStamped(
                [0.0, 0.0],
                TurtleBot4Directions.NORTH
            )
            self.navigator.setInitialPose(initial_pose)

            self.get_logger().info('[Nav2] Nav2 활성화 대기 중...')
            self.navigator.waitUntilNav2Active()
            self.get_logger().info('[Nav2] Nav2 활성화 완료.')

            if self.navigator.getDockedStatus():
                self.get_logger().info('[Undock] undock 실행 중...')
                self.navigator.undock()
                self.get_logger().info('[Undock] undock 완료.')
            else:
                self.get_logger().info('[Undock] 이미 undock 상태라 생략합니다.')

            self.navigate_to_scan_point()

        except Exception as e:
            self.get_logger().error(f'[시작 실패] {e}')

    # =========================================================
    # 지정 좌표 이동
    # =========================================================
    def navigate_to_scan_point(self):
        self.state = State.NAVIGATING

        self.get_logger().info(
            f'[이동 시작] 스캔 지점으로 이동: x={self.SCAN_POINT[0]:.3f}, y={self.SCAN_POINT[1]:.3f}'
        )

        goal_pose = self.navigator.getPoseStamped(
            self.SCAN_POINT,
            self.SCAN_DIRECTION
        )

        self.navigator.startToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                try:
                    eta = feedback.estimated_time_remaining.sec
                    self.get_logger().info(
                        f'[이동 중] ETA={eta}s',
                        throttle_duration_sec=2.0
                    )
                except Exception:
                    pass
            time.sleep(0.2)

        result = self.navigator.getResult()
        self.get_logger().info(f'[이동 결과] result={result}')
        self.get_logger().info('[도착] 지정 좌표 도착 완료. 스캔 모드로 전환합니다.')

        self.start_scanning()

    # =========================================================
    # SCANNING
    # =========================================================
    def start_scanning(self):
        self.cancel_nav_task_if_needed()
        self.stop_robot()
        self.state = State.SCANNING
        self.scan_direction = 1
        self.scan_turn_count = 0
        self.tracking_lost_count = 0
        self.target_confirm_count = 0
        self.stationary_count = 0
        self.last_object_map_xy = None
        self.last_goal_xy = None
        self.scan_start_time = time.time()
        self.scan_full_turn_sec = (2.0 * math.pi) / self.scan_angular_speed

        self.get_logger().info('[상태전환] SCANNING 시작: 한 방향으로 천천히 360도 회전하면서 RGB/Depth 동기화 YOLO 탐지')

    def scan_timer_callback(self):
        if self.state != State.SCANNING:
            return

        now = time.time()
        elapsed = now - self.scan_start_time

        if elapsed >= self.scan_full_turn_sec:
            self.scan_turn_count += 1
            self.scan_start_time = now
            elapsed = 0.0
            self.get_logger().warn(
                f'[스캔] 360도 회전 완료 count={self.scan_turn_count}. 같은 방향으로 다시 스캔합니다.'
            )

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.scan_direction * self.scan_angular_speed
        self.publish_direct_cmd(twist)

        self.get_logger().info(
            f'[스캔 회전 명령] angular.z={twist.angular.z:.2f}, '
            f'elapsed={elapsed:.1f}/{self.scan_full_turn_sec:.1f}s',
            throttle_duration_sec=1.0
        )

    # =========================================================
    # CameraInfo
    # =========================================================
    def camera_info_callback(self, msg):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.ppx = float(msg.k[2])
        self.ppy = float(msg.k[5])

    def has_camera_info(self):
        return None not in (self.fx, self.fy, self.ppx, self.ppy)

    # =========================================================
    # 동기화 Camera 콜백: YOLO + Depth + TF/Nav2 판단
    # =========================================================
    def camera_callback(self, rgb_msg, depth_msg):
        if self.state not in (
            State.SCANNING,
            State.CENTERING,
            State.APPROACHING,
            State.NAV_TO_OBJECT,
            State.TRACKING,
        ):
            return

        frame = self.decode_rgb(rgb_msg)
        if frame is None:
            self.get_logger().warn('[RGB] decode 실패', throttle_duration_sec=1.0)
            return

        depth = self.decode_compressed_depth(depth_msg)
        if depth is None:
            self.get_logger().warn('[Depth] decode 실패', throttle_duration_sec=1.0)
            return

        h, w = frame.shape[:2]
        target, detected = self.detect_target(frame)

        now = time.time()
        if now - self.last_yolo_log_time > 1.0:
            self.last_yolo_log_time = now
            self.get_logger().info(
                f'[YOLO 결과] state={self.state.name}, target={self.target_class_id}, detected={detected}'
            )

        if target is None:
            if self.state == State.SCANNING:
                self.target_confirm_count = 0
            self.handle_target_lost()
            return

        self.tracking_lost_count = 0

        x1, y1, x2, y2, conf, cls_id = target
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        offset_x = (cx - (w / 2)) / (w / 2)

        if self.state == State.SCANNING:
            self.target_confirm_count += 1
            self.get_logger().info(
                f'[car 후보] cls={cls_id}, conf={conf:.2f}, '
                f'confirm={self.target_confirm_count}/{self.target_confirm_frames}, '
                f'cx={cx}, offset={offset_x:.2f}'
            )

            if self.target_confirm_count < self.target_confirm_frames:
                return

            self.stop_robot()
            self.state = State.CENTERING
            self.get_logger().info(
                f'[car 확정] cls={cls_id}, conf={conf:.2f}, '
                f'{self.target_confirm_frames}프레임 연속 검출 완료, cx={cx}, offset={offset_x:.2f}'
            )
            self.get_logger().info('[상태전환] SCANNING -> CENTERING')
            return

        if self.state == State.CENTERING:
            self.center_to_target(offset_x)
            return

        if self.state == State.APPROACHING:
            self.approach_to_target_with_nav2(rgb_msg, depth, cx, cy, offset_x)
            return

        if self.state == State.NAV_TO_OBJECT:
            self.update_nav_goal_if_needed(rgb_msg, depth, cx, cy)
            return

        if self.state == State.TRACKING:
            self.monitor_target(depth, cx, cy, offset_x)
            return

    def detect_target(self, frame):
        t0 = time.time()
        results = self.model(frame, verbose=False)
        yolo_ms = (time.time() - t0) * 1000.0

        boxes = results[0].boxes

        detected = []
        target_candidates = []

        for b in boxes:
            cls_id = int(b.cls[0])
            conf = float(b.conf[0])
            x1, y1, x2, y2 = b.xyxy[0].cpu().numpy()

            detected.append((cls_id, round(conf, 2)))

            if cls_id == self.target_class_id and conf >= self.target_conf_threshold:
                target_candidates.append((x1, y1, x2, y2, conf, cls_id))

        self.get_logger().info(
            f'[YOLO 추론시간] {yolo_ms:.0f}ms',
            throttle_duration_sec=1.0
        )

        if not target_candidates:
            return None, detected

        best = max(target_candidates, key=lambda x: x[4])
        return best, detected

    # =========================================================
    # CENTERING
    # =========================================================
    def center_to_target(self, offset_x):
        self.get_logger().info(
            f'[정렬] offset_x={offset_x:.3f}',
            throttle_duration_sec=0.5
        )

        if abs(offset_x) <= self.center_tolerance:
            self.stop_robot()
            self.state = State.APPROACHING
            self.get_logger().info('[상태전환] CENTERING -> APPROACHING')
            self.get_logger().info('[접근] 중앙 정렬 완료. 객체 map 좌표 계산 후 Nav2 goal 접근을 시작합니다.')
            return

        twist = Twist()
        twist.linear.x = 0.0
        angular = -self.center_kp * offset_x
        angular = max(min(angular, self.center_max_angular), -self.center_max_angular)
        twist.angular.z = angular
        self.publish_direct_cmd(twist)

        self.get_logger().info(
            f'[정렬 명령] angular.z={twist.angular.z:.2f}',
            throttle_duration_sec=0.5
        )

    # =========================================================
    # APPROACHING: 강사님 방식 반영
    #   YOLO 중심 + Depth -> camera 3D -> map 좌표 -> Nav2 goal
    # =========================================================
    def approach_to_target_with_nav2(self, rgb_msg, depth, cx, cy, offset_x):
        depth_m = self.get_depth_at(depth, cx, cy)

        if depth_m is None:
            self.stop_robot()
            self.get_logger().warn('[Nav2 접근] Depth 값을 얻지 못했습니다. 정지 후 다음 프레임 대기.', throttle_duration_sec=1.0)
            return

        if depth_m > self.max_valid_target_depth:
            self.stop_robot()
            self.get_logger().warn(
                f'[Nav2 접근] Depth 튐 감지: dist={depth_m:.2f}m. goal 생성하지 않고 대기.',
                throttle_duration_sec=1.0
            )
            return

        if depth_m <= self.approach_stop_distance:
            self.stop_robot()
            self.state = State.TRACKING
            self.get_logger().info(
                f'[상태전환] APPROACHING -> TRACKING, distance={depth_m:.2f}m'
            )
            return

        # goal을 너무 자주 보내지 않도록 제한
        now = time.time()
        if now - self.last_nav_goal_time < self.nav_goal_min_interval:
            return

        if not self.has_camera_info():
            self.stop_robot()
            self.get_logger().warn(
                '[Nav2 접근] CameraInfo 미수신. fx/fy/ppx/ppy가 없어 map 좌표를 계산할 수 없습니다.',
                throttle_duration_sec=1.0
            )
            return

        camera_frame = self.resolve_camera_frame(rgb_msg)
        if not camera_frame:
            self.stop_robot()
            self.get_logger().warn('[Nav2 접근] camera_frame을 알 수 없습니다.', throttle_duration_sec=1.0)
            return

        object_map = self.pixel_depth_to_map_point(cx, cy, depth_m, camera_frame)
        if object_map is None:
            self.stop_robot()
            return

        robot_map = self.get_robot_map_xy()
        if robot_map is None:
            self.stop_robot()
            return

        goal_pose = self.make_goal_in_front_of_object(robot_map, object_map)
        if goal_pose is None:
            self.stop_robot()
            self.state = State.TRACKING
            self.get_logger().info('[상태전환] APPROACHING -> TRACKING: 이미 목표 거리 근처입니다.')
            return

        self.start_nav_to_object(goal_pose, object_map, depth_m)

    def pixel_depth_to_map_point(self, cx, cy, z, camera_frame):
        # optical frame 기준: x=오른쪽, y=아래, z=전방
        x = (float(cx) - self.ppx) * z / self.fx
        y = (float(cy) - self.ppy) * z / self.fy

        pt_camera = PointStamped()
        pt_camera.header.stamp = self.get_clock().now().to_msg()
        pt_camera.header.frame_id = camera_frame
        pt_camera.point.x = x
        pt_camera.point.y = y
        pt_camera.point.z = z

        try:
            pt_map = self.tf_buffer.transform(
                pt_camera,
                self.map_frame,
                timeout=Duration(seconds=0.5)
            )
            self.get_logger().info(
                f'[TF 변환] camera({x:.2f},{y:.2f},{z:.2f}) -> '
                f'{self.map_frame}({pt_map.point.x:.2f},{pt_map.point.y:.2f},{pt_map.point.z:.2f})',
                throttle_duration_sec=1.0
            )
            return (pt_map.point.x, pt_map.point.y)
        except TransformException as e:
            self.get_logger().warn(
                f'[TF 변환 실패] {camera_frame} -> {self.map_frame}: {e}',
                throttle_duration_sec=1.0
            )
            return None
        except Exception as e:
            self.get_logger().warn(
                f'[TF 변환 예외] {camera_frame} -> {self.map_frame}: {e}',
                throttle_duration_sec=1.0
            )
            return None

    def get_robot_map_xy(self):
        for base_frame in self.base_frame_candidates:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    base_frame,
                    Time(),
                    timeout=Duration(seconds=0.2)
                )
                return (tf.transform.translation.x, tf.transform.translation.y)
            except Exception:
                continue

        self.get_logger().warn(
            f'[TF 실패] {self.map_frame} 기준 로봇 base frame을 찾지 못했습니다. 후보={self.base_frame_candidates}',
            throttle_duration_sec=1.0
        )
        return None

    def make_goal_in_front_of_object(self, robot_xy, object_xy):
        rx, ry = robot_xy
        ox, oy = object_xy
        dx = ox - rx
        dy = oy - ry
        dist = math.hypot(dx, dy)

        if dist <= self.nav_stop_distance + 0.05:
            return None

        goal_x = ox - self.nav_stop_distance * dx / dist
        goal_y = oy - self.nav_stop_distance * dy / dist

        yaw = math.atan2(oy - goal_y, ox - goal_x)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.map_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'[Nav2 goal 생성] robot=({rx:.2f},{ry:.2f}), object=({ox:.2f},{oy:.2f}), '
            f'goal=({goal_x:.2f},{goal_y:.2f}), stop_distance={self.nav_stop_distance:.2f}'
        )
        return goal_pose

    def start_nav_to_object(self, goal_pose, object_xy, depth_m):
        self.cancel_nav_task_if_needed()
        self.stop_robot()

        self.state = State.NAV_TO_OBJECT
        self.nav_goal_active = True
        self.last_nav_goal_time = time.time()
        self.last_object_map_xy = object_xy
        self.last_goal_xy = (goal_pose.pose.position.x, goal_pose.pose.position.y)

        self.get_logger().info(
            f'[상태전환] APPROACHING -> NAV_TO_OBJECT: '
            f'object=({object_xy[0]:.2f},{object_xy[1]:.2f}), depth={depth_m:.2f}m. '
            f'Nav2가 장애물 회피 기반으로 접근합니다. '
            f'goal 갱신은 {self.nav_goal_min_interval:.1f}s 이상 경과 + 객체 위치 {self.replan_distance_threshold:.2f}m 이상 변화 시 수행합니다.'
        )

        self.navigator.startToPose(goal_pose)

    def update_nav_goal_if_needed(self, rgb_msg, depth, cx, cy):
        """NAV_TO_OBJECT 중 객체 위치가 충분히 바뀐 경우에만 Nav2 goal을 갱신한다.

        객체 위치는 계속 계산하지만, goal은 너무 자주 바꾸지 않는다.
        조건:
          1) 마지막 goal 전송 후 nav_goal_min_interval초 이상 경과
          2) map 기준 객체 위치가 replan_distance_threshold m 이상 변화
        """
        if not self.nav_goal_active:
            return

        now = time.time()
        if now - self.last_nav_goal_time < self.nav_goal_min_interval:
            self.get_logger().info(
                f'[Nav2 goal 유지] 마지막 goal 이후 {now - self.last_nav_goal_time:.1f}s. '
                f'{self.nav_goal_min_interval:.1f}s 전까지는 갱신하지 않습니다.',
                throttle_duration_sec=1.0
            )
            return

        depth_m = self.get_depth_at(depth, cx, cy)
        if depth_m is None:
            self.get_logger().warn('[Nav2 goal 유지] Depth 없음. 기존 goal 유지.', throttle_duration_sec=1.0)
            return

        if depth_m > self.max_valid_target_depth:
            self.get_logger().warn(
                f'[Nav2 goal 유지] Depth 튐 감지: dist={depth_m:.2f}m. 기존 goal 유지.',
                throttle_duration_sec=1.0
            )
            return

        if not self.has_camera_info():
            self.get_logger().warn('[Nav2 goal 유지] CameraInfo 미수신. 기존 goal 유지.', throttle_duration_sec=1.0)
            return

        camera_frame = self.resolve_camera_frame(rgb_msg)
        if not camera_frame:
            self.get_logger().warn('[Nav2 goal 유지] camera_frame 미확인. 기존 goal 유지.', throttle_duration_sec=1.0)
            return

        object_map = self.pixel_depth_to_map_point(cx, cy, depth_m, camera_frame)
        if object_map is None:
            return

        if self.last_object_map_xy is not None:
            moved = math.hypot(
                object_map[0] - self.last_object_map_xy[0],
                object_map[1] - self.last_object_map_xy[1]
            )
            if moved < self.replan_distance_threshold:
                self.get_logger().info(
                    f'[Nav2 goal 유지] 객체 이동량={moved:.2f}m < {self.replan_distance_threshold:.2f}m. 기존 goal 유지.',
                    throttle_duration_sec=1.0
                )
                return
        else:
            moved = 0.0

        robot_map = self.get_robot_map_xy()
        if robot_map is None:
            return

        goal_pose = self.make_goal_in_front_of_object(robot_map, object_map)
        if goal_pose is None:
            self.cancel_nav_task_if_needed()
            self.stop_robot()
            self.state = State.TRACKING
            self.get_logger().info('[상태전환] NAV_TO_OBJECT -> TRACKING: 객체가 목표 거리 근처입니다.')
            return

        self.get_logger().info(
            f'[Nav2 goal 갱신] 객체 위치 변화량={moved:.2f}m >= {self.replan_distance_threshold:.2f}m, '
            f'마지막 goal 이후 {now - self.last_nav_goal_time:.1f}s >= {self.nav_goal_min_interval:.1f}s. 새 goal 전송.'
        )

        try:
            if hasattr(self.navigator, 'cancelTask'):
                self.navigator.cancelTask()
                self.get_logger().info('[Nav2] 이동 중 goal cancel 후 새 goal 전송')
        except Exception as e:
            self.get_logger().warn(f'[Nav2] goal 갱신 중 cancel 실패: {e}', throttle_duration_sec=1.0)

        self.navigator.startToPose(goal_pose)
        self.nav_goal_active = True
        self.last_nav_goal_time = time.time()
        self.last_object_map_xy = object_map
        self.last_goal_xy = (goal_pose.pose.position.x, goal_pose.pose.position.y)

    def nav_to_object_timer_callback(self):
        if self.state != State.NAV_TO_OBJECT or not self.nav_goal_active:
            return

        try:
            if not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    try:
                        eta = feedback.estimated_time_remaining.sec
                        self.get_logger().info(
                            f'[Nav2 객체 접근 중] ETA={eta}s',
                            throttle_duration_sec=2.0
                        )
                    except Exception:
                        pass
                return

            result = self.navigator.getResult()
            self.nav_goal_active = False
            self.stop_robot()
            self.state = State.TRACKING
            self.get_logger().info(f'[Nav2 객체 접근 결과] result={result}')
            self.get_logger().info('[상태전환] NAV_TO_OBJECT -> TRACKING: 근거리 재이동 여부를 관찰합니다.')
        except Exception as e:
            self.get_logger().warn(f'[Nav2 객체 접근 확인 실패] {e}', throttle_duration_sec=1.0)
            self.nav_goal_active = False
            self.stop_robot()
            self.start_scanning()

    # =========================================================
    # TRACKING / MONITORING: 근거리에서는 직접 cmd_vel만 사용
    # =========================================================
    def monitor_target(self, depth, cx, cy, offset_x):
        depth_m = self.get_depth_at(depth, cx, cy)

        if depth_m is None:
            self.stop_robot()
            self.get_logger().warn(
                '[TRACKING] 객체는 보이지만 Depth 없음. 정지 유지.',
                throttle_duration_sec=1.0
            )
            return

        if depth_m > self.max_valid_target_depth:
            self.stop_robot()
            self.get_logger().warn(
                f'[TRACKING] Depth 튐 감지: dist={depth_m:.2f}m. 전진하지 않고 대기.',
                throttle_duration_sec=1.0
            )
            return

        desired_distance = 0.75
        min_distance = 0.65
        max_distance = 1.10

        if abs(offset_x) < 0.12:
            angular = 0.0
        else:
            angular = -self.center_kp * offset_x
            angular = max(
                min(angular, self.approach_max_angular),
                -self.approach_max_angular
            )

        distance_error = depth_m - desired_distance

        twist = Twist()

        if depth_m <= min_distance:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publish_direct_cmd(twist)
            self.get_logger().info(
                f'[TRACKING] 너무 가까움. 정지 유지: dist={depth_m:.2f}m',
                throttle_duration_sec=1.0
            )
            return

        if distance_error > 0.05:
            linear = self.approach_kp_linear * distance_error
            linear = max(min(linear, self.approach_max_linear), 0.0)
        else:
            linear = 0.0

        if depth_m >= max_distance:
            linear = self.approach_max_linear

        twist.linear.x = linear
        twist.angular.z = angular
        self.publish_direct_cmd(twist)

        self.get_logger().info(
            f'[TRACKING 추격] dist={depth_m:.2f}m, offset={offset_x:.2f}, '
            f'linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}',
            throttle_duration_sec=0.5
        )

    def handle_target_lost(self):
        if self.state == State.SCANNING:
            return

        if self.state in (State.CENTERING, State.APPROACHING, State.NAV_TO_OBJECT, State.TRACKING):
            self.tracking_lost_count += 1

            self.get_logger().warn(
                f'[객체 미검출] state={self.state.name}, lost_count={self.tracking_lost_count}',
                throttle_duration_sec=1.0
            )

            if self.tracking_lost_count >= self.tracking_max_lost:
                self.stop_robot()
                self.get_logger().warn('[객체 상실] 일정 시간 객체를 찾지 못해 SCANNING으로 복귀합니다.')
                self.start_scanning()

    # =========================================================
    # Depth / Decode
    # =========================================================
    def get_depth_at(self, depth, cx, cy):
        if depth is None:
            return None

        if cy < 0 or cy >= depth.shape[0] or cx < 0 or cx >= depth.shape[1]:
            return None

        y1 = max(0, cy - 4)
        y2 = min(depth.shape[0], cy + 5)
        x1 = max(0, cx - 4)
        x2 = min(depth.shape[1], cx + 5)

        patch = depth[y1:y2, x1:x2]
        valid = patch[patch > 0]

        if len(valid) == 0:
            return None

        depth_value = float(np.median(valid))

        if depth_value > 20.0:
            depth_m = depth_value / 1000.0
        else:
            depth_m = depth_value

        if not (0.05 < depth_m < 10.0):
            return None

        return depth_m

    def decode_compressed_depth(self, msg):
        try:
            raw_data = msg.data[12:]
            np_arr = np.frombuffer(raw_data, np.uint8)
            depth_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            return depth_img
        except Exception as e:
            self.get_logger().warn(f'[Depth] decode 예외: {e}', throttle_duration_sec=1.0)
            return None

    def decode_rgb(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            return frame
        except Exception as e:
            self.get_logger().warn(f'[RGB] decode 예외: {e}', throttle_duration_sec=1.0)
            return None

    def resolve_camera_frame(self, rgb_msg):
        if self.camera_frame_override:
            return self.camera_frame_override.lstrip('/')

        frame_id = getattr(rgb_msg.header, 'frame_id', '')
        if frame_id:
            return frame_id.lstrip('/')

        # frame_id가 비어 있으면 흔히 쓰는 후보를 사용한다.
        return f'{self.ns_clean}/oakd_rgb_camera_optical_frame'

    # =========================================================
    # 공통 / 제어 충돌 방지
    # =========================================================
    def publish_direct_cmd(self, twist):
        """직접 cmd_vel은 SCANNING/CENTERING/TRACKING에서만 허용한다.

        NAV_TO_OBJECT 상태에서는 Nav2가 cmd_vel을 발행하므로,
        직접 cmd_vel을 동시에 보내지 않는다.
        """
        if self.state == State.NAV_TO_OBJECT:
            self.get_logger().warn(
                '[cmd_vel 차단] NAV_TO_OBJECT 중에는 직접 cmd_vel을 발행하지 않습니다.',
                throttle_duration_sec=2.0
            )
            return

        try:
            if rclpy.ok():
                self.cmd_pub.publish(twist)
        except Exception:
            pass

    def stop_robot(self):
        try:
            if rclpy.ok():
                twist = Twist()
                self.cmd_pub.publish(twist)
        except Exception:
            pass

    def cancel_nav_task_if_needed(self):
        if not self.nav_goal_active:
            return
        try:
            if hasattr(self.navigator, 'cancelTask'):
                self.navigator.cancelTask()
                self.get_logger().info('[Nav2] 기존 goal cancel')
        except Exception as e:
            self.get_logger().warn(f'[Nav2] cancel 실패: {e}', throttle_duration_sec=1.0)
        finally:
            self.nav_goal_active = False

    def destroy_node(self):
        self.cancel_nav_task_if_needed()
        self.stop_robot()
        super().destroy_node()


def main():
    rclpy.init()

    node = MissionController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_nav_task_if_needed()
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
