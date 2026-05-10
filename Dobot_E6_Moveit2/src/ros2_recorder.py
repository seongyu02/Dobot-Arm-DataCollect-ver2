"""
ros2_recorder.py — ROS2 기반 동기화 레코더 (Dobot E6 데이터 수집용)

robot_server.py 가 import 하는 헬퍼 모듈.
HIK + ZED 두 카메라를 ApproximateTimeSynchronizer 로 동기화하여
정확한 타임스탬프 기준으로 프레임을 저장한다.

필수 패키지 (Ubuntu 22.04 / ROS2 Humble):
    sudo apt install ros-humble-ros-base ros-humble-cv-bridge \
                     python3-rclpy python3-sensor-msgs
    source /opt/ros/humble/setup.bash   # .bashrc 에 추가 권장

동작 방식:
    - HIK / ZED 프레임을 ROS2 Image 토픽으로 퍼블리시
    - 로봇 상태는 별도 스레드에서 ~50Hz 폴링 후 캐시
    - ApproximateTimeSynchronizer 가 두 카메라 타임스탬프를 35 ms 허용 오차로 맞춰 콜백 호출
    - 콜백에서 이미지 저장 큐에 투입 → 저장 전용 스레드가 disk write 처리
    - ZED 미연결 시 sync 콜백이 발동되지 않아 robot_server 기존 레코딩 경로가 fallback 으로 동작
"""

import threading
import queue
import os
import time
import numpy as np
import cv2

# rclpy 가 없으면 available() == False 로 graceful 하게 비활성화
_ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import Float32MultiArray
    import message_filters
    from cv_bridge import CvBridge
    _ROS2_AVAILABLE = True
except ImportError:
    # 클래스 정의와 타입 힌트가 실패하지 않도록 더미 선언
    class Node:           # type: ignore[no-redef]
        pass
    class Image:          # type: ignore[no-redef]
        pass
    class JointState:     # type: ignore[no-redef]
        pass
    class Float32MultiArray:  # type: ignore[no-redef]
        pass


# ─────────────────────────────────────────────────────────────────────────────
# 내부 ROS2 노드
# ─────────────────────────────────────────────────────────────────────────────

class _DobotRecorderNode(Node):
    """퍼블리셔 + ApproximateTimeSynchronizer 기반 동기화 저장 노드."""

    # HIK 저장 파라미터 (robot_server._record_tick 과 동일)
    _HIK_RESIZE  = (320, 240)
    _HIK_CROP    = (16, 240, 55, 279)   # y0, y1, x0, x1  →  224 × 224

    # ZED 저장 파라미터
    _ZED_CROP    = (120, 480, 150, 510)  # y0, y1, x0, x1  →  360 × 360 → 224 × 224
    _ZED_OUT     = (224, 224)

    def __init__(self):
        super().__init__('dobot_recorder')
        self._bridge = CvBridge()
        self._lock   = threading.Lock()

        # ── Publishers ─────────────────────────────────────────────────────
        self._pub_hik    = self.create_publisher(Image,             '/dobot/hik/image_raw', 5)
        self._pub_zed    = self.create_publisher(Image,             '/dobot/zed/image_raw', 5)
        self._pub_joints = self.create_publisher(JointState,        '/dobot/joint_states',  5)
        self._pub_tcp    = self.create_publisher(Float32MultiArray, '/dobot/tcp_pose',       5)

        # ── 로봇 상태 캐시 (publish_robot 에서 갱신) ───────────────────────
        self._joints  : list = []
        self._tcp     : list = []
        self._gripper : int  = 0
        self._mode    : int  = 0

        # ── 레코딩 상태 ────────────────────────────────────────────────────
        self._recording   : bool = False
        self._save_dir    : str  = None
        self._frames      : list = []
        self._frame_count : int  = 0

        # ── 이미지 저장 큐 (spin 스레드 블록 방지) ─────────────────────────
        self._save_q : queue.Queue = queue.Queue(maxsize=60)
        threading.Thread(target=self._save_worker, daemon=True).start()

        # ── Subscribers + Synchronizer (HIK + ZED) ─────────────────────────
        sub_hik = message_filters.Subscriber(self, Image, '/dobot/hik/image_raw')
        sub_zed = message_filters.Subscriber(self, Image, '/dobot/zed/image_raw')
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [sub_hik, sub_zed],
            queue_size=15,
            slop=0.035,     # 35 ms 허용 오차 (20 Hz 기준 프레임 주기 50 ms 의 70%)
        )
        self._sync.registerCallback(self._on_sync)

    # ── 퍼블리시 ──────────────────────────────────────────────────────────────

    def publish_hik(self, bgr: np.ndarray) -> None:
        """HIK 카메라 프레임을 ROS2 토픽으로 퍼블리시 (캡처 직후 호출)."""
        msg = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub_hik.publish(msg)

    def publish_zed(self, bgr: np.ndarray) -> None:
        """ZED 카메라 프레임을 ROS2 토픽으로 퍼블리시 (캡처 직후 호출)."""
        msg = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub_zed.publish(msg)

    def publish_robot(self, joints, tcp_pose, gripper_on: int, robot_mode: int) -> None:
        """로봇 상태를 ROS2 토픽으로 퍼블리시 + 내부 캐시 갱신."""
        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name     = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        js.position = [float(v) for v in joints]
        self._pub_joints.publish(js)

        tp = Float32MultiArray()
        tp.data = [float(v) for v in tcp_pose]
        self._pub_tcp.publish(tp)

        with self._lock:
            self._joints  = list(joints)
            self._tcp     = list(tcp_pose)
            self._gripper = int(gripper_on)
            self._mode    = int(robot_mode)

    # ── 레코딩 제어 ───────────────────────────────────────────────────────────

    def start_recording(self, save_dir: str) -> None:
        with self._lock:
            self._save_dir    = save_dir
            self._frames      = []
            self._frame_count = 0
            self._recording   = True
        self.get_logger().info(f'[ros2_recorder] recording started → {save_dir}')

    def stop_recording(self) -> list:
        """레코딩을 중단하고 수집된 프레임 목록을 반환."""
        with self._lock:
            self._recording = False
            data = list(self._frames)
        self.get_logger().info(f'[ros2_recorder] stopped — {len(data)} synced frames')
        return data

    def frame_count(self) -> int:
        return self._frame_count

    # ── 동기화 콜백 ───────────────────────────────────────────────────────────

    def _on_sync(self, hik_msg: Image, zed_msg: Image) -> None:
        """HIK + ZED 타임스탬프가 35 ms 이내로 맞춰질 때 호출."""
        if not self._recording:
            return

        with self._lock:
            if not self._recording:
                return
            fc       = self._frame_count
            joints   = list(self._joints)
            tcp      = list(self._tcp)
            gripper  = self._gripper
            mode     = self._mode
            save_dir = self._save_dir
            self._frame_count += 1

        # 타임스탬프: HIK 캡처 시각 기준 (ROS clock → float)
        ts    = hik_msg.header.stamp.sec + hik_msg.header.stamp.nanosec * 1e-9
        fname = f'frame_{fc:06d}.jpg'

        record = {
            'frame_id':        fc,
            'timestamp':       ts,
            'image_path_hik':  f'hik/{fname}',
            'image_path_zed':  f'zed/{fname}',
            'joint_angles':    joints,
            'tcp_pose':        tcp,
            'gripper_tooldo1': gripper,
            'gripper_tooldo2': 0,
            'robot_mode':      mode,
        }

        try:
            hik_raw = self._bridge.imgmsg_to_cv2(hik_msg, desired_encoding='bgr8')
            zed_raw = self._bridge.imgmsg_to_cv2(zed_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'[ros2_recorder] cv_bridge error: {e}')
            return

        with self._lock:
            self._frames.append(record)

        # disk write 는 별도 스레드에서 처리 (spin 스레드 블록 방지)
        try:
            self._save_q.put_nowait((save_dir, fname, hik_raw, zed_raw))
        except queue.Full:
            self.get_logger().warn('[ros2_recorder] save queue full — frame dropped')

    def _save_worker(self) -> None:
        """이미지 저장 전용 스레드 (데몬)."""
        while True:
            save_dir, fname, hik_raw, zed_raw = self._save_q.get()
            try:
                y0, y1, x0, x1 = self._HIK_CROP
                hik_320  = cv2.resize(hik_raw, self._HIK_RESIZE)
                hik_save = hik_320[y0:y1, x0:x1]
                cv2.imwrite(os.path.join(save_dir, 'images', 'hik', fname), hik_save)

                y0, y1, x0, x1 = self._ZED_CROP
                zed_crop = zed_raw[y0:y1, x0:x1]
                zed_save = cv2.resize(zed_crop, self._ZED_OUT)
                cv2.imwrite(os.path.join(save_dir, 'images', 'zed', fname), zed_save)
            except Exception as e:
                print(f'[ros2_recorder] save_worker error: {e}')


# ─────────────────────────────────────────────────────────────────────────────
# 모듈 공개 인터페이스 (robot_server.py 가 호출)
# ─────────────────────────────────────────────────────────────────────────────

_node        : _DobotRecorderNode = None
_spin_thread : threading.Thread   = None


def available() -> bool:
    """rclpy 설치 여부 반환."""
    return _ROS2_AVAILABLE


def start() -> bool:
    """rclpy 초기화 + 노드 시작. robot_server startup 에서 한 번 호출.
    성공 시 True, 실패(미설치 포함) 시 False 반환."""
    global _node, _spin_thread
    if not _ROS2_AVAILABLE:
        print('[ros2_recorder] rclpy 없음 — ROS2 레코딩 비활성화 (fallback 모드)')
        return False
    if _node is not None:
        return True
    try:
        rclpy.init()
        _node = _DobotRecorderNode()
        _spin_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
        _spin_thread.start()
        print('[ros2_recorder] ROS2 노드 시작 완료')
        return True
    except Exception as e:
        print(f'[ros2_recorder] 초기화 실패: {e}')
        return False


def publish_hik(bgr: np.ndarray) -> None:
    """HIK 프레임 퍼블리시. _hik_grab_loop 에서 호출."""
    if _node:
        _node.publish_hik(bgr)


def publish_zed(bgr: np.ndarray) -> None:
    """ZED 프레임 퍼블리시. _zed_grab_loop 에서 호출."""
    if _node:
        _node.publish_zed(bgr)


def publish_robot(joints, tcp_pose, gripper_on: int, robot_mode: int) -> None:
    """로봇 상태 퍼블리시 + 캐시 갱신. _robot_pub_loop 에서 호출."""
    if _node:
        _node.publish_robot(joints, tcp_pose, gripper_on, robot_mode)


def start_recording(save_dir: str) -> None:
    """레코딩 시작. _start_recording 에서 호출."""
    if _node:
        _node.start_recording(save_dir)


def stop_recording() -> list:
    """레코딩 중단 후 동기화된 프레임 목록 반환. _stop_and_save 에서 호출."""
    return _node.stop_recording() if _node else []


def frame_count() -> int:
    """현재 레코딩 중인 동기화 프레임 수 반환."""
    return _node.frame_count() if _node else 0


def shutdown() -> None:
    global _node
    if _node:
        _node.destroy_node()
        rclpy.shutdown()
        _node = None
