#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pick and Place GUI (New) - Pose control + Camera On/Off + Pick-Place Step
Base: pick_place_gui_moveit.py
Add: Camera streaming (Start/Stop only), One-cycle Pick-Place Step (→ 픽 초기 복귀까지).

[데이터 수집 전용 — 학습 코드 없음]
- RoboData-Forge: 이 스크립트가 생성하는 vla_dataset/ 데이터가 로봇 학습용 정제 데이터로 사용됨.
- Action-Collector VLA: Vision-Action 중심 20Hz 데이터 수집 (이미지 + 관절/TCP/그리퍼).
- Dobot-Behavior-Sync: 로봇 피드백과 카메라 프레임을 frame_id 기준으로 동기화하여 저장.
- Imitate-Flow: 모방 학습용 에피소드(초기→픽→플레이스→초기) 흐름 관리, 성공 에피소드만 저장.
"""

import sys
import os
import time
import random
import csv
import shutil
from datetime import datetime

# Windows 콘솔 UTF-8
if sys.platform == 'win32':
    try:
        import io
        if not isinstance(sys.stdout, io.TextIOWrapper) or (hasattr(sys.stdout, 'encoding') and sys.stdout.encoding.lower() != 'utf-8'):
            if hasattr(sys.stdout, 'buffer') and not getattr(sys.stdout.buffer, 'closed', True):
                sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
            if hasattr(sys.stderr, 'buffer') and not getattr(sys.stderr.buffer, 'closed', True):
                sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')
    except Exception:
        pass

import numpy as np
import cv2
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QGridLayout, QLabel, QLineEdit, QPushButton, QTextEdit,
    QDoubleSpinBox, QMessageBox, QCheckBox
)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QImage, QPixmap

# Robot control
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)  # Dobot_E6_Moveit2
workspace_root = os.path.dirname(parent_dir)  # TCP-IP-Python-V4
sys.path.insert(0, current_dir)
sys.path.insert(0, workspace_root)

from dobot_e6_controller import DobotE6Controller
from suction_gripper import SuctionGripper

# Camera (optional)
CAMERA_AVAILABLE = False
try:
    from camera_viewer import HikRobotCamera
    CAMERA_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Camera module not available ({e})")

if CAMERA_AVAILABLE:
    class CameraThread(QThread):
        frame_ready = pyqtSignal(np.ndarray)

        def __init__(self, camera):
            super().__init__()
            self.camera = camera
            self.running = False

        def run(self):
            self.running = True
            while self.running:
                ret, frame = self.camera.get_frame()
                if ret and frame is not None:
                    self.frame_ready.emit(frame)
                time.sleep(0.033)

        def stop(self):
            self.running = False


# 좌표 정의 (1~9번)
POS_1 = (139.37, -435.31, 100.84, 176.68, -3.05, -14.66)
POS_2 = (145.59, -414.15, 100.97, -178.28, -3.94, -14.14)
POS_3 = (217.75, -405.65, 100.05, 172.08, -0.15, -3.77)   # 3번
POS_4 = (220.21, -368.72, 100.23, 179.75, 2.03, -1.86)   # 4번 (시작 위치)
POS_5 = (221.63, -318.39, 100.10, -177.54, 5.35, 1.69)
POS_6 = (94.10, -311.54, 100.91, 175.14, -1.08, -19.63)
POS_7 = (84.97, -437.89, 100.85, -179.78, -2.11, -22.01)
POS_8 = (-27.02, -438.80, 100.61, -173.78, -12.63, -15.91)
POS_9 = (-15.38, -321.49, 100.67, -179.15, 0.60, 3.41)

# A섹션: 1~7번 좌표로 정의된 파여있는 사각형 (다각형)
# 순서: 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 1 (닫힌 다각형)
A_SECTION_POINTS = [
    (POS_1[0], POS_1[1]),  # 1번
    (POS_2[0], POS_2[1]),  # 2번
    (POS_3[0], POS_3[1]),  # 3번
    (POS_4[0], POS_4[1]),  # 4번
    (POS_5[0], POS_5[1]),  # 5번
    (POS_6[0], POS_6[1]),  # 6번
    (POS_7[0], POS_7[1]),  # 7번
]

# B섹션: 6~9번 좌표로 정의된 사각형
# 순서: 6 -> 7 -> 8 -> 9 -> 6 (사각형)
B_SECTION_POINTS = [
    (POS_6[0], POS_6[1]),  # 6번
    (POS_7[0], POS_7[1]),  # 7번
    (POS_8[0], POS_8[1]),  # 8번
    (POS_9[0], POS_9[1]),  # 9번
]

# A섹션 회전값 (시작 위치 4번 좌표 회전값 사용)
A_SECTION_RX, A_SECTION_RY, A_SECTION_RZ = 179.75, 2.03, -1.86  # 4번 좌표 회전값 사용
# B섹션 회전값 (평균 또는 대표값 사용)
B_SECTION_RX, B_SECTION_RY, B_SECTION_RZ = -179.15, 0.60, 3.41  # 9번 좌표 회전값 사용

RELEASE_Z = 101.0   # 그리퍼 해제 높이 (Z=101까지 내려오면 해제)
RELEASE_Z_TOLERANCE_MM = 1.5   # Z 도달 판정 허용 오차 (100.5~101.5)
PLACE_WAIT_AT_101_S = 2.0   # Place 시 Z=101 도달 후 release 전 대기 시간(초)
Z_MOVE_MIN, Z_MOVE_MAX = 120.0, 200.0   # 이동 중 Z 랜덤 범위
Z_AFTER_RELEASE = 200.0   # Release 후 Z 높이

INIT_X, INIT_Y, INIT_Z = 89.3715, -102.9836, 611.7122   # INIT = 대기(홈) 위치
INIT_RX, INIT_RY, INIT_RZ = 90.1244, 3.6761, 5.7400

# 픽/플레이스 시 석션 유지 시간 (초)
PICK_HOLD_TIME_LO, PICK_HOLD_TIME_HI = 0.85, 1.2
PLACE_HOLD_TIME_LO, PLACE_HOLD_TIME_HI = 0.5, 0.85

# 랜덤 구간 (캘리브레이션 안전 범위)
CALIB_X_MIN, CALIB_X_MAX = -185.0, 96.0   # mm
CALIB_Y_MIN, CALIB_Y_MAX = -466.0, -411.0
CALIB_Z_MIN, CALIB_Z_MAX = 130.0, 240.0
RANDOM_Z_MIN_MAIN_LO, RANDOM_Z_MIN_MAIN_HI = 150.0, 170.0
RANDOM_WAYPOINT_COUNT_MIN, RANDOM_WAYPOINT_COUNT_MAX = 3, 10
INIT_TO_PICK_RANDOM_MIN, INIT_TO_PICK_RANDOM_MAX = 1, 5
PLACE_LIFT_RANDOM_MIN, PLACE_LIFT_RANDOM_MAX = 1, 5
PICK_TO_INIT_RANDOM_MIN, PICK_TO_INIT_RANDOM_MAX = 1, 5
MAX_RADIUS = 400.0  # mm - IK solver 안정 범위
MIN_DIST_FROM_LINE_LO, MIN_DIST_FROM_LINE_HI = 35.0, 70.0
WAYPOINT_NOISE_MM = 4.0

# 진공 스위치 DI (E6 팔 끝단 ToolDI 1/2)
VACUUM_DI_ENABLED = True
VACUUM_DI_INDEX = 1   # ToolDI(1) — 진공 스위치(압력/진공 감지) 연결
MAX_PICK_DESCENT_ATTEMPTS = 3
LEVEL2_XY_DELTA_MM = 3.0
LEVEL2_MAX_TRIES = 1
LEVEL2_DWELL_S = 0.25
LEVEL2_Z_SEARCH_HI_LO, LEVEL2_Z_SEARCH_HI_HI = 150.0, 160.0

# 진공 스위치 DI (E6 팔 끝단 ToolDI 1/2). 센서 없으면 False로 두면 Level 2 미적용
# E6 DI는 PNP 타입 → PNP 출력 센서(3-wire PNP) 사용 권장
VACUUM_DI_ENABLED = True
VACUUM_DI_INDEX = 1   # ToolDI(1) — 진공 스위치(압력/진공 감지) 연결
# 픽 시도: 절대 좌표(PICK_Z)까지 내려가는 횟수 3회 이하 (블럭 애무 방지)
MAX_PICK_DESCENT_ATTEMPTS = 3   # 1차 + Level2 1점 + Level1 1회 = 3회
LEVEL2_XY_DELTA_MM = 3.0   # 미세 XY 탐색 ±mm
LEVEL2_MAX_TRIES = 1   # Level 2 최대 XY 시도 1점만 → 총 픽 시도 3회 이하
LEVEL2_DWELL_S = 0.25   # Level 2 각 시도 시 suction dwell (초)
LEVEL2_Z_SEARCH_HI_LO, LEVEL2_Z_SEARCH_HI_HI = 150.0, 160.0   # 탐색 높이 mm (Z 다이브 없음, 한 번만 내려서 grip)

# Recovery 모드: 보정본 데이터셋 수집용
RECOVERY_FAILURE_OFFSET_MM_LO = 15.0   # 실패 XY 오프셋 최소 (mm) — 너무 작으면 그리퍼가 물체를 밀어냄
RECOVERY_FAILURE_OFFSET_MM_HI = 35.0   # 실패 XY 오프셋 최대 (mm)
RECOVERY_APPROACH_Z_LO, RECOVERY_APPROACH_Z_HI = 120.0, 150.0   # 접근 시 Z 구간 (여기서 멈춤)
RECOVERY_FAIL_DIVE_Z_LO, RECOVERY_FAIL_DIVE_Z_HI = 110.0, 120.0 # 실패 시 내려가는 Z (101까지 안 내려감)
RECOVERY_LIFT_Z_LO, RECOVERY_LIFT_Z_HI = 120.0, 150.0   # 실패 후 올라가는 Z 랜덤 (200 대신)

# 20Hz 기록
RECORD_INTERVAL_MS = 50  # 20Hz = 50ms
VLA_DATASET_BASE = os.path.join(workspace_root, "vla_dataset")
# 저장 전 검증: 마지막 프레임이 초기 자세(INIT)에 있어야만 저장 (초기 복귀 안 한 에피소드 제외)
INIT_POSE_TOLERANCE_MM = 20.0   # x,y,z 허용 오차 mm
INIT_POSE_TOLERANCE_DEG = 8.0   # rx,ry,rz 허용 오차 도


def point_in_polygon(px, py, polygon):
    """점 (px, py)가 다각형 polygon 내부에 있는지 판단 (Ray casting algorithm)."""
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside

def generate_random_point_in_section(section_points, max_attempts=100):
    """섹션 내부에 랜덤 좌표 생성."""
    # 섹션의 경계 박스 계산
    xs = [p[0] for p in section_points]
    ys = [p[1] for p in section_points]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    
    for _ in range(max_attempts):
        rx = random.uniform(x_min, x_max)
        ry = random.uniform(y_min, y_max)
        if point_in_polygon(rx, ry, section_points):
            return (rx, ry)
    # 실패 시 섹션의 중심점 반환
    cx = sum(xs) / len(xs)
    cy = sum(ys) / len(ys)
    return (cx, cy)


class PickPlaceStepWorker(QThread):
    """한 스텝: A섹션 Pick → B섹션 Place 또는 B섹션 Pick → A섹션 Place."""
    finished = pyqtSignal(bool)
    log_signal = pyqtSignal(str)
    step_started = pyqtSignal()  # (legacy)
    recording_begin_at_initial = pyqtSignal()  # 초기 자세 1초 유지 후 발신 → 이때 20Hz 기록 시작
    episode_vacuum_durations = pyqtSignal(float, float)  # (pick_hold, place_hold) s

    def __init__(self, robot, gripper, pick_section="A", pick_x=None, pick_y=None, pick_rx=None, pick_ry=None, pick_rz=None, recovery_mode=None):
        super().__init__()
        self.robot = robot
        self.gripper = gripper
        self._stop_requested = False
        self.pick_section = pick_section  # "A" or "B"
        self.pick_x = pick_x  # None이면 섹션의 고정 좌표 사용 (A섹션은 4번, B섹션은 랜덤)
        self.pick_y = pick_y
        self.pick_rx = pick_rx
        self.pick_ry = pick_ry
        self.pick_rz = pick_rz
        self.recovery_mode = recovery_mode  # None, "re1" (1번 실패), "re2" (2번 실패)
        # Place 위치 저장 (다음 스텝에서 사용)
        self.place_x = None
        self.place_y = None

    def request_stop(self):
        self._stop_requested = True

    def _log(self, msg):
        self.log_signal.emit(msg)

    def _move(self, x, y, z, velocity=25.0, rx=None, ry=None, rz=None):
        if self._stop_requested:
            return False
        # rx, ry, rz가 None이면 기본값 사용 (A섹션 회전값)
        if rx is None:
            rx = A_SECTION_RX
        if ry is None:
            ry = A_SECTION_RY
        if rz is None:
            rz = A_SECTION_RZ
        ok = self.robot.move_j(x, y, z, rx, ry, rz, coordinate_mode=0, velocity=velocity, use_waypoint=False)
        if ok:
            self.robot.wait_for_motion_complete()
        return ok

    def _get_current_z(self):
        """현재 Z 좌표 반환."""
        try:
            pose = self.robot.get_current_pose_from_feedback()
            if pose and len(pose) >= 3:
                return float(pose[2])
        except Exception:
            pass
        return None

    def _move_xy_only(self, x, y, velocity=25.0, rx=None, ry=None, rz=None):
        """XY만 이동 (현재 Z 유지)."""
        current_z = self._get_current_z()
        if current_z is None:
            # Z를 가져올 수 없으면 기본값 사용
            current_z = Z_MOVE_MIN
        return self._move(x, y, current_z, velocity=velocity, rx=rx, ry=ry, rz=rz)

    def _wait_until_z_reached(self, target_z, tolerance_mm=1.5, timeout_s=15.0):
        """피드백으로 현재 Z가 target_z 근처(±tolerance_mm)에 도달할 때까지 대기."""
        start = time.time()
        while time.time() - start < timeout_s:
            if self._stop_requested:
                return False
            pose = self.robot.get_current_pose_from_feedback()
            if pose is not None and len(pose) >= 3:
                z = float(pose[2])
                if abs(z - target_z) <= tolerance_mm:
                    self._log(f"Z 도달 확인: {z:.2f} mm (목표 {target_z}±{tolerance_mm})")
                    return True
            time.sleep(0.12)
        self._log(f"Z 도달 대기 타임아웃 (목표 Z={target_z})")
        return False

    def _release_at_safe_z(self, x, y, rx=None, ry=None, rz=None, wait_time=0.1):
        """Z=101(RELEASE_Z)까지 내려온 뒤 그리퍼 해제."""
        if self.gripper:
            rx = rx if rx is not None else A_SECTION_RX
            ry = ry if ry is not None else A_SECTION_RY
            rz = rz if rz is not None else A_SECTION_RZ
            self._move(x, y, RELEASE_Z, velocity=18.0, rx=rx, ry=ry, rz=rz)   # Z=101로 내리고
            self.gripper.release(wait_time=wait_time)

    def _safe_return_home(self):
        """실패/중단 시 초기 자세(홈)로 복귀 → 재시도 전 준비."""
        if not self.robot or not self.robot.connected:
            return
        try:
            if self.gripper:
                try:
                    self.gripper.release(wait_time=0.1)
                except Exception:
                    pass
            self._log("실패 → 초기 자세로 복귀 중...")
            self._move(INIT_X, INIT_Y, INIT_Z, rx=INIT_RX, ry=INIT_RY, rz=INIT_RZ)
        except Exception as e:
            self._log(f"초기 복귀 실패: {e}")

    def _fail_and_go_home(self):
        """중간 실패 시 초기 자세로 복귀 후 한 스텝 성공으로 처리 (GUI에는 항상 성공)."""
        self._safe_return_home()
        self.finished.emit(True)

    def _dist_point_to_line(self, px, py, x1, y1, x2, y2):
        """점 (px,py)에서 선분 (x1,y1)-(x2,y2)까지의 거리 (mm)."""
        dx, dy = x2 - x1, y2 - y1
        length = np.sqrt(dx * dx + dy * dy)
        if length < 1e-6:
            return np.sqrt((px - x1) ** 2 + (py - y1) ** 2)
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (length * length)))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return np.sqrt((px - proj_x) ** 2 + (py - proj_y) ** 2)

    def _sample_waypoint_avoiding_line(self, from_x, from_y, to_x, to_y, z_min=None, z_max=None, min_dist=None):
        """직선 경로에서 최소 min_dist 이상 떨어진 waypoint 샘플. 마지막에 ±WAYPOINT_NOISE_MM 노이즈."""
        z_min = z_min if z_min is not None else random.uniform(RANDOM_Z_MIN_MAIN_LO, RANDOM_Z_MIN_MAIN_HI)
        z_max = z_max if z_max is not None else CALIB_Z_MAX
        min_dist = min_dist if min_dist is not None else random.uniform(MIN_DIST_FROM_LINE_LO, MIN_DIST_FROM_LINE_HI)
        for _ in range(35):
            rx_pt = random.uniform(CALIB_X_MIN, CALIB_X_MAX)
            ry_pt = random.uniform(CALIB_Y_MIN, CALIB_Y_MAX)
            rz_pt = random.uniform(z_min, z_max)
            radius = np.sqrt(rx_pt ** 2 + ry_pt ** 2)
            if radius >= MAX_RADIUS:
                continue
            dist = self._dist_point_to_line(rx_pt, ry_pt, from_x, from_y, to_x, to_y)
            if dist >= min_dist:
                n = WAYPOINT_NOISE_MM
                rx_pt = np.clip(rx_pt + random.uniform(-n, n), CALIB_X_MIN, CALIB_X_MAX)
                ry_pt = np.clip(ry_pt + random.uniform(-n, n), CALIB_Y_MIN, CALIB_Y_MAX)
                rz_pt = np.clip(rz_pt + random.uniform(-n, n), z_min, z_max)
                return (rx_pt, ry_pt, rz_pt)
        return None

    def run(self):
        try:
            if not self.robot or not self.robot.connected or not self.gripper:
                self._log("Robot/Gripper not connected")
                self.finished.emit(False)
                return

            # Pick 위치 결정
            if self.pick_section == "A":
                # A섹션: pick_x, pick_y가 None이면 4번 좌표 사용 (시작 위치)
                if self.pick_x is None or self.pick_y is None:
                    pick_x, pick_y, pick_z = POS_4[0], POS_4[1], POS_4[2]
                    pick_rx, pick_ry, pick_rz = POS_4[3], POS_4[4], POS_4[5]
                else:
                    pick_x, pick_y = self.pick_x, self.pick_y
                    pick_z = RELEASE_Z
                    pick_rx = self.pick_rx if self.pick_rx is not None else A_SECTION_RX
                    pick_ry = self.pick_ry if self.pick_ry is not None else A_SECTION_RY
                    pick_rz = self.pick_rz if self.pick_rz is not None else A_SECTION_RZ
                place_section = "B"
                place_section_points = B_SECTION_POINTS
                place_rx, place_ry, place_rz = B_SECTION_RX, B_SECTION_RY, B_SECTION_RZ
            else:  # "B"
                # B섹션: pick_x, pick_y가 None이면 랜덤 좌표 생성 (또는 이전에 Place한 위치 사용)
                if self.pick_x is None or self.pick_y is None:
                    pick_x, pick_y = generate_random_point_in_section(B_SECTION_POINTS)
                else:
                    pick_x, pick_y = self.pick_x, self.pick_y
                pick_z = RELEASE_Z
                pick_rx = self.pick_rx if self.pick_rx is not None else B_SECTION_RX
                pick_ry = self.pick_ry if self.pick_ry is not None else B_SECTION_RY
                pick_rz = self.pick_rz if self.pick_rz is not None else B_SECTION_RZ
                place_section = "A"
                place_section_points = A_SECTION_POINTS
                place_rx, place_ry, place_rz = A_SECTION_RX, A_SECTION_RY, A_SECTION_RZ

            # Place 위치: 반대 섹션 내 랜덤 좌표 생성
            place_x, place_y = generate_random_point_in_section(place_section_points)
            place_z = RELEASE_Z
            # Place 위치 저장 (다음 스텝에서 사용)
            self.place_x = place_x
            self.place_y = place_y

            # 1) 초기 위치
            self._log("1) Initial position...")
            if not self._move(INIT_X, INIT_Y, INIT_Z, rx=INIT_RX, ry=INIT_RY, rz=INIT_RZ):
                self._log("Failed: initial position")
                self._fail_and_go_home()
                return

            # 1.2) 초기 자세에서 1초 대기 → 20Hz 기록 시작
            self._log("1.2) Holding at initial pose 1s...")
            for _ in range(10):
                if self._stop_requested:
                    self._fail_and_go_home()
                    return
                time.sleep(0.1)
            self.recording_begin_at_initial.emit()

            # 2) Pick 위치로 이동 (Recovery 모드면 보정본 동작, 아니면 기존 방식)
            if self.recovery_mode == "re1":
                # Re1: 실패 XY로 다이렉트 이동 → Z 120~150에서 정지 → 실패 다이브(110~120) → 올림(120~150) → 정확한 XY로 이동 → Z=101 픽
                # 랜덤 실패 XY 오프셋 (너무 작으면 그리퍼가 물체 밀어냄 → 최소 15mm 이상)
                offset_dx = random.uniform(-RECOVERY_FAILURE_OFFSET_MM_HI, RECOVERY_FAILURE_OFFSET_MM_HI)
                offset_dy = random.uniform(-RECOVERY_FAILURE_OFFSET_MM_HI, RECOVERY_FAILURE_OFFSET_MM_HI)
                offset_norm = np.sqrt(offset_dx**2 + offset_dy**2)
                if offset_norm < RECOVERY_FAILURE_OFFSET_MM_LO:
                    angle = random.uniform(0, 2 * np.pi)
                    offset_dx = RECOVERY_FAILURE_OFFSET_MM_LO * np.cos(angle)
                    offset_dy = RECOVERY_FAILURE_OFFSET_MM_LO * np.sin(angle)
                fail_x = pick_x + offset_dx
                fail_y = pick_y + offset_dy
                
                # 실패 XY로 다이렉트 이동 → Z 120~150에서 정지
                z_approach = random.uniform(RECOVERY_APPROACH_Z_LO, RECOVERY_APPROACH_Z_HI)
                self._log(f"2) Recovery Re1: Moving to failure XY ({fail_x:.1f}, {fail_y:.1f}) at Z={z_approach:.1f}mm (120-150 구간에서 멈춤)...")
                if not self._move(fail_x, fail_y, z_approach, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._log("Failed: move to failure position")
                    self._fail_and_go_home()
                    return
                # 실패 다이브: Z 110~120까지만 내림 (101까지 안 내려감 → 물체 안 밀림)
                z_fail_dive = random.uniform(RECOVERY_FAIL_DIVE_Z_LO, RECOVERY_FAIL_DIVE_Z_HI)
                self._log(f"2.1) Re1: Descending to Z={z_fail_dive:.1f} (110-120, failure attempt)...")
                if not self._move(fail_x, fail_y, z_fail_dive, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._log("Failed: descend at failure position")
                    self._fail_and_go_home()
                    return
                # 실패 후 복귀: Z 120~150으로 올림
                z_lift = random.uniform(RECOVERY_LIFT_Z_LO, RECOVERY_LIFT_Z_HI)
                self._log(f"2.2) Re1: Lifting to Z={z_lift:.1f} (120-150)...")
                if not self._move(fail_x, fail_y, z_lift, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._log("Failed: lift after failure")
                    self._fail_and_go_home()
                    return
                
                # 정확한 pick XY로 이동 (Z는 120-150 유지)
                self._log(f"2.3) Re1: Moving to correct pick ({pick_x:.1f}, {pick_y:.1f})...")
                if not self._move_xy_only(pick_x, pick_y, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._log("Failed: move to correct pick position")
                    self._fail_and_go_home()
                    return
                # Z=101로 내려서 픽
                self._log("2.4) Re1: Descending to Z=101 (success)...")
                if not self._move(pick_x, pick_y, RELEASE_Z, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._log("Failed: descend to Z=101 for pick")
                    self._fail_and_go_home()
                    return
                
            elif self.recovery_mode == "re2":
                # Re2: 1차 실패 XY로 다이렉트 이동 → Z 120~150 정지 → 실패 다이브 → 올림 → 2차 실패 XY로 이동 → 실패 다이브 → 올림 → 정확한 XY로 이동 → Z=101 픽
                # 1차 실패
                offset_dx1 = random.uniform(-RECOVERY_FAILURE_OFFSET_MM_HI, RECOVERY_FAILURE_OFFSET_MM_HI)
                offset_dy1 = random.uniform(-RECOVERY_FAILURE_OFFSET_MM_HI, RECOVERY_FAILURE_OFFSET_MM_HI)
                offset_norm1 = np.sqrt(offset_dx1**2 + offset_dy1**2)
                if offset_norm1 < RECOVERY_FAILURE_OFFSET_MM_LO:
                    angle = random.uniform(0, 2 * np.pi)
                    offset_dx1 = RECOVERY_FAILURE_OFFSET_MM_LO * np.cos(angle)
                    offset_dy1 = RECOVERY_FAILURE_OFFSET_MM_LO * np.sin(angle)
                fail_x1 = pick_x + offset_dx1
                fail_y1 = pick_y + offset_dy1
                # 1차 실패 XY로 다이렉트 이동 → Z 120~150에서 정지
                z_approach1 = random.uniform(RECOVERY_APPROACH_Z_LO, RECOVERY_APPROACH_Z_HI)
                self._log(f"2) Recovery Re2: Moving to 1st failure XY ({fail_x1:.1f}, {fail_y1:.1f}) at Z={z_approach1:.1f}mm (120-150 구간에서 멈춤)...")
                if not self._move(fail_x1, fail_y1, z_approach1, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._log("Failed: move to 1st failure position")
                    self._fail_and_go_home()
                    return
                # 1차 실패 다이브: Z 110~120까지 내림
                z_fail1 = random.uniform(RECOVERY_FAIL_DIVE_Z_LO, RECOVERY_FAIL_DIVE_Z_HI)
                self._log(f"2.1) Re2: Descending to Z={z_fail1:.1f} (110-120, 1st failure)...")
                if not self._move(fail_x1, fail_y1, z_fail1, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._fail_and_go_home()
                    return
                # 1차 실패 후 복귀: Z 120~150으로 올림
                z_lift1 = random.uniform(RECOVERY_LIFT_Z_LO, RECOVERY_LIFT_Z_HI)
                self._log(f"2.2) Re2: Lifting to Z={z_lift1:.1f} (120-150)...")
                if not self._move(fail_x1, fail_y1, z_lift1, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._fail_and_go_home()
                    return
                
                # 2차 실패
                offset_dx2 = random.uniform(-RECOVERY_FAILURE_OFFSET_MM_HI, RECOVERY_FAILURE_OFFSET_MM_HI)
                offset_dy2 = random.uniform(-RECOVERY_FAILURE_OFFSET_MM_HI, RECOVERY_FAILURE_OFFSET_MM_HI)
                offset_norm2 = np.sqrt(offset_dx2**2 + offset_dy2**2)
                if offset_norm2 < RECOVERY_FAILURE_OFFSET_MM_LO:
                    angle = random.uniform(0, 2 * np.pi)
                    offset_dx2 = RECOVERY_FAILURE_OFFSET_MM_LO * np.cos(angle)
                    offset_dy2 = RECOVERY_FAILURE_OFFSET_MM_LO * np.sin(angle)
                fail_x2 = pick_x + offset_dx2
                fail_y2 = pick_y + offset_dy2
                # 2차 실패 XY로 이동 → Z 120~150에서 정지
                z_approach2 = random.uniform(RECOVERY_APPROACH_Z_LO, RECOVERY_APPROACH_Z_HI)
                self._log(f"2.3) Re2: Moving to 2nd failure XY ({fail_x2:.1f}, {fail_y2:.1f}) at Z={z_approach2:.1f}mm (120-150 구간에서 멈춤)...")
                if not self._move(fail_x2, fail_y2, z_approach2, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._log("Failed: move to 2nd failure position")
                    self._fail_and_go_home()
                    return
                # 2차 실패 다이브: Z 110~120까지 내림
                z_fail2 = random.uniform(RECOVERY_FAIL_DIVE_Z_LO, RECOVERY_FAIL_DIVE_Z_HI)
                self._log(f"2.4) Re2: Descending to Z={z_fail2:.1f} (110-120, 2nd failure)...")
                if not self._move(fail_x2, fail_y2, z_fail2, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._fail_and_go_home()
                    return
                # 2차 실패 후 복귀: Z 120~150으로 올림
                z_lift2 = random.uniform(RECOVERY_LIFT_Z_LO, RECOVERY_LIFT_Z_HI)
                self._log(f"2.5) Re2: Lifting to Z={z_lift2:.1f} (120-150)...")
                if not self._move(fail_x2, fail_y2, z_lift2, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._fail_and_go_home()
                    return
                
                # 정확한 pick XY로 이동 (Z는 120-150 유지)
                self._log(f"2.6) Re2: Moving to correct pick ({pick_x:.1f}, {pick_y:.1f})...")
                if not self._move_xy_only(pick_x, pick_y, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._fail_and_go_home()
                    return
                # Z=101로 내려서 픽
                self._log("2.7) Re2: Descending to Z=101 (success)...")
                if not self._move(pick_x, pick_y, RELEASE_Z, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._log("Failed: descend to Z=101 for pick")
                    self._fail_and_go_home()
                    return
                
            else:
                # 기존 방식: Pick 위치로 바로 이동
                z_move = random.uniform(Z_MOVE_MIN, Z_MOVE_MAX)
                self._log(f"2) Moving to {self.pick_section} section Pick position (Z={z_move:.1f}mm)...")
                if not self._move(pick_x, pick_y, z_move, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._log("Failed: move to pick position")
                    self._fail_and_go_home()
                    return

                # 3) Z=101로 내려가서 Pick (Gripper ON)
                self._log("3) Descending to Z=101 for Pick...")
                if not self._move(pick_x, pick_y, RELEASE_Z, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                    self._log("Failed: descend to Z=101 for pick")
                    self._fail_and_go_home()
                    return

            # Gripper ON
            pick_hold = random.uniform(PICK_HOLD_TIME_LO, PICK_HOLD_TIME_HI)
            self._log(f"3) Gripper ON at Pick (Z=101), holding {pick_hold:.2f}s...")
            self.gripper.grip(wait_time=pick_hold)
            time.sleep(0.3)  # 추가 유지 시간
            if self._stop_requested:
                self._release_at_safe_z(pick_x, pick_y, rx=pick_rx, ry=pick_ry, rz=pick_rz)
                self._fail_and_go_home()
                return

            # 4) Z를 120~200 사이 랜덤값으로 올림 (GRIPPER ON 상태로 객체를 들고)
            z_move2 = random.uniform(Z_MOVE_MIN, Z_MOVE_MAX)
            self._log(f"4) Lifting to Z={z_move2:.1f}mm (random 120~200) with object...")
            if not self._move(pick_x, pick_y, z_move2, velocity=18.0, rx=pick_rx, ry=pick_ry, rz=pick_rz):
                self._log("Failed: lift to random Z")
                self._release_at_safe_z(pick_x, pick_y, rx=pick_rx, ry=pick_ry, rz=pick_rz)
                self._fail_and_go_home()
                return

            # 5) Place 위치로 이동 (X,Y만 변경, Z는 현재 높이 유지)
            self._log(f"5) Moving to {place_section} section Place position (X,Y change, Z height maintained)...")
            if not self._move(place_x, place_y, z_move2, velocity=18.0, rx=place_rx, ry=place_ry, rz=place_rz):
                self._log("Failed: move to place position")
                self._release_at_safe_z(place_x, place_y, rx=place_rx, ry=place_ry, rz=place_rz)
                self._fail_and_go_home()
                return

            # 6) Z=101로 내려가서 Place & Release (반드시 101 도달 확인 후 2초 대기)
            self._log("6) Descending to Z=101 for Place...")
            if not self._move(place_x, place_y, RELEASE_Z, velocity=18.0, rx=place_rx, ry=place_ry, rz=place_rz):
                self._log("Failed: descend to Z=101 for place → 현재 위치에서 그리퍼 해제 시도")
                if self.gripper:
                    self.gripper.release(wait_time=0.2)
                self._fail_and_go_home()
                return

            # 피드백으로 Z=101 근처(100.5~101.5) 도달할 때까지 대기
            if not self._wait_until_z_reached(RELEASE_Z, tolerance_mm=RELEASE_Z_TOLERANCE_MM):
                self._log("Z=101 도달 확인 실패 → 그리퍼 해제 후 복귀")
                if self.gripper:
                    self.gripper.release(wait_time=0.2)
                self._fail_and_go_home()
                return
            if self._stop_requested:
                self._fail_and_go_home()
                return

            # Z=101 도달 확인 후 2초 대기 후 그리퍼 해제
            self._log(f"6) Z=101 도달됨. {PLACE_WAIT_AT_101_S}초 대기 후 Gripper OFF...")
            for _ in range(int(PLACE_WAIT_AT_101_S * 10)):
                if self._stop_requested:
                    if self.gripper:
                        self.gripper.release(wait_time=0.1)
                    self._fail_and_go_home()
                    return
                time.sleep(0.1)

            place_hold = random.uniform(PLACE_HOLD_TIME_LO, PLACE_HOLD_TIME_HI)
            self._log(f"6) Gripper OFF at Place (Z=101, 대기 완료), holding {place_hold:.2f}s...")
            if self.gripper:
                self.gripper.release(wait_time=place_hold)
            if self._stop_requested:
                self._fail_and_go_home()
                return

            # 7) Z=200까지 올림
            self._log("7) Lifting to Z=200...")
            if not self._move(place_x, place_y, Z_AFTER_RELEASE, velocity=18.0, rx=place_rx, ry=place_ry, rz=place_rz):
                self._log("Failed: lift to Z=200")
                self._fail_and_go_home()
                return

            # 8) 초기 위치로 복귀
            self._log("8) Returning to initial position...")
            if not self._move(INIT_X, INIT_Y, INIT_Z, rx=INIT_RX, ry=INIT_RY, rz=INIT_RZ):
                self._log("Failed: return to initial")
                self._fail_and_go_home()
                return

            self._log(f"Step complete ({self.pick_section} section Pick → {place_section} section Place).")
            self.episode_vacuum_durations.emit(pick_hold, place_hold)
            # Place 위치를 반환 (다음 스텝에서 사용)
            self.finished.emit(True)
        except Exception as e:
            self._log(f"Error: {e}")
            try:
                if self.pick_section == "A":
                    self._release_at_safe_z(POS_4[0], POS_4[1], rx=POS_4[3], ry=POS_4[4], rz=POS_4[5])
                else:
                    self._release_at_safe_z(place_x if 'place_x' in locals() else 0, place_y if 'place_y' in locals() else 0, rx=B_SECTION_RX, ry=B_SECTION_RY, rz=B_SECTION_RZ)
            except Exception:
                pass
            self._fail_and_go_home()


class PickPlaceGUINew(QMainWindow):
    """Pose GUI + Camera On/Off + Pick-Place Step (based on pick_place_gui_moveit)."""

    def __init__(self):
        super().__init__()
        self.robot = None
        self.gripper = None
        self.camera = None
        self.camera_thread = None
        self.camera_active = False
        self.step_worker = None
        self.auto_collect_target = 0   # 0=수동, N=N개 수집 중
        self.auto_collect_done = 0
        self.auto_collect_recovery_mode = None  # None, "re1", "re2" (Recovery N개 수집 시 사용)
        # 위치 기억: 마지막으로 Place한 위치 (다음 스텝의 Pick 위치)
        self.last_place_x = None
        self.last_place_y = None
        self.current_pick_section = "A"  # 현재 Pick할 섹션 ("A" or "B")
        # 20Hz 기록
        self.recorded_data = []
        self.recording = False
        self.record_save_dir = None
        self.record_frame_count = 0
        self.record_timer = QTimer(self)
        self.record_timer.timeout.connect(self._on_record_tick)
        # Vacuum 명령 기반 로그 (명령 ON 유지 시간, s)
        self.episode_vacuum_pick_hold = self.episode_vacuum_place_hold = 0.0
        self.init_ui()
        if CAMERA_AVAILABLE:
            self._init_camera()

    def _init_camera(self):
        try:
            calib_path = os.path.join(workspace_root, "hikrobot_calibration_20260126_143821.npz")
            if not os.path.exists(calib_path):
                calib_path = None
            self.camera = HikRobotCamera(calibration_file=calib_path)
            if self.camera.init_camera():
                self.log("Camera initialized")
            else:
                self.camera = None
                self.log("Camera init failed")
        except Exception as e:
            self.log(f"Camera init error: {e}")
            self.camera = None

    def init_ui(self):
        self.setWindowTitle("Dobot E6 Pick & Place (Recovery) - Pose + Camera")
        self.setGeometry(100, 100, 1200, 620)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)

        # Left: Camera
        left_layout = QVBoxLayout()
        self.camera_label = QLabel("Camera (Start to view)")
        self.camera_label.setMinimumSize(640, 480)
        self.camera_label.setMaximumSize(640, 480)
        self.camera_label.setStyleSheet("border: 2px solid gray; background-color: black; color: white;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.camera_label)
        cam_btn_layout = QHBoxLayout()
        self.start_camera_btn = QPushButton("Start Camera")
        self.start_camera_btn.clicked.connect(self.start_camera)
        self.stop_camera_btn = QPushButton("Stop Camera")
        self.stop_camera_btn.clicked.connect(self.stop_camera)
        self.stop_camera_btn.setEnabled(False)
        cam_btn_layout.addWidget(self.start_camera_btn)
        cam_btn_layout.addWidget(self.stop_camera_btn)
        left_layout.addLayout(cam_btn_layout)
        main_layout.addLayout(left_layout)

        # Right: Connection, Pose, Actions, Pick-Place Step, Status, Log
        right_layout = QVBoxLayout()
        right_layout.addWidget(self._create_connection_panel())
        right_layout.addWidget(self._create_pose_panel())
        right_layout.addWidget(self._create_action_panel())
        right_layout.addWidget(self._create_pick_place_step_panel())
        right_layout.addWidget(self._create_status_panel())

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(140)
        right_layout.addWidget(QLabel("Log:"))
        right_layout.addWidget(self.log_text)
        main_layout.addLayout(right_layout)

        self.log("GUI initialized (Pose + Camera + Pick-Place Step)")

    def _create_connection_panel(self):
        group = QGroupBox("Robot Connection")
        layout = QHBoxLayout()
        layout.addWidget(QLabel("IP:"))
        self.ip_input = QLineEdit("192.168.5.1")
        layout.addWidget(self.ip_input)
        layout.addWidget(QLabel("Port:"))
        self.port_input = QLineEdit("29999")
        self.port_input.setMaximumWidth(70)
        layout.addWidget(self.port_input)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_robot)
        layout.addWidget(self.connect_btn)
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_robot)
        self.disconnect_btn.setEnabled(False)
        layout.addWidget(self.disconnect_btn)
        group.setLayout(layout)
        return group

    def _create_pose_panel(self):
        group = QGroupBox("Target Pose (mm, degrees)")
        layout = QGridLayout()
        self.x_input = QDoubleSpinBox()
        self.x_input.setRange(-1000, 1000)
        self.x_input.setValue(300)
        self.x_input.setSuffix(" mm")
        self.y_input = QDoubleSpinBox()
        self.y_input.setRange(-1000, 1000)
        self.y_input.setValue(0)
        self.y_input.setSuffix(" mm")
        self.z_input = QDoubleSpinBox()
        self.z_input.setRange(0, 800)
        self.z_input.setValue(400)
        self.z_input.setSuffix(" mm")
        self.rx_input = QDoubleSpinBox()
        self.rx_input.setRange(-180, 180)
        self.rx_input.setValue(180)
        self.rx_input.setSuffix(" °")
        self.ry_input = QDoubleSpinBox()
        self.ry_input.setRange(-180, 180)
        self.ry_input.setValue(0)
        self.ry_input.setSuffix(" °")
        self.rz_input = QDoubleSpinBox()
        self.rz_input.setRange(-180, 180)
        self.rz_input.setValue(0)
        self.rz_input.setSuffix(" °")
        layout.addWidget(QLabel("X:"), 0, 0)
        layout.addWidget(self.x_input, 0, 1)
        layout.addWidget(QLabel("Y:"), 0, 2)
        layout.addWidget(self.y_input, 0, 3)
        layout.addWidget(QLabel("Z:"), 0, 4)
        layout.addWidget(self.z_input, 0, 5)
        layout.addWidget(QLabel("RX:"), 1, 0)
        layout.addWidget(self.rx_input, 1, 1)
        layout.addWidget(QLabel("RY:"), 1, 2)
        layout.addWidget(self.ry_input, 1, 3)
        layout.addWidget(QLabel("RZ:"), 1, 4)
        layout.addWidget(self.rz_input, 1, 5)
        self.move_pose_btn = QPushButton("Move to Pose (MoveJ)")
        self.move_pose_btn.clicked.connect(self.move_to_pose)
        self.move_pose_btn.setEnabled(False)
        layout.addWidget(self.move_pose_btn, 2, 0, 1, 6)
        group.setLayout(layout)
        return group

    def _create_action_panel(self):
        group = QGroupBox("Actions")
        layout = QHBoxLayout()
        self.home_btn = QPushButton("Home")
        self.home_btn.clicked.connect(self.go_home)
        self.home_btn.setEnabled(False)
        self.grip_btn = QPushButton("Grip")
        self.grip_btn.clicked.connect(self.grip)
        self.grip_btn.setEnabled(False)
        self.release_btn = QPushButton("Release")
        self.release_btn.clicked.connect(self.release)
        self.release_btn.setEnabled(False)
        self.estop_btn = QPushButton("E-STOP")
        self.estop_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        self.estop_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(self.home_btn)
        layout.addWidget(self.grip_btn)
        layout.addWidget(self.release_btn)
        layout.addWidget(self.estop_btn)
        group.setLayout(layout)
        return group

    def _create_pick_place_step_panel(self):
        group = QGroupBox("Pick-Place Step (Pick → [랜덤4~8개 zigzag] → Place → ... → Pick 복귀)")
        layout = QVBoxLayout()
        # 1개 수동 / 20Hz 체크
        row1 = QHBoxLayout()
        self.pick_place_step_btn = QPushButton("Run Pick-Place Step (1개)")
        self.pick_place_step_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.pick_place_step_btn.clicked.connect(self.run_pick_place_step)
        self.pick_place_step_btn.setEnabled(False)
        row1.addWidget(self.pick_place_step_btn)
        self.record_20hz_cb = QCheckBox("20Hz 기록 (vla_dataset/1,2,3...)")
        self.record_20hz_cb.setToolTip("체크 시 초기자세 도달 후 20Hz 기록 시작. frame_000000 = 초기자세. 카메라 자동시작.")
        row1.addWidget(self.record_20hz_cb)
        layout.addLayout(row1)
        # 자동 수집: 1, 5, 10, 50번 (랜덤 궤적)
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("자동 수집:"))
        for n in (1, 5, 10, 50):
            btn = QPushButton(f"{n}번")
            btn.setStyleSheet("background-color: #2196F3; color: white;")
            btn.clicked.connect(lambda checked, count=n: self.run_auto_collect(count))
            btn.setEnabled(False)
            setattr(self, f"auto_collect_{n}_btn", btn)
            row2.addWidget(btn)
        self.stop_auto_btn = QPushButton("Stop")
        self.stop_auto_btn.setStyleSheet("background-color: #f44336; color: white;")
        self.stop_auto_btn.clicked.connect(self.stop_auto_collect)
        self.stop_auto_btn.setEnabled(False)
        row2.addWidget(self.stop_auto_btn)
        layout.addLayout(row2)
        # 자동 수집 시 한 에피소드 성공 후 [다음] 버튼으로 다음 스텝 진행
        row_result = QHBoxLayout()
        row_result.addWidget(QLabel("다음 에피소드:"))
        self.episode_next_btn = QPushButton("다음")
        self.episode_next_btn.setStyleSheet("background-color: #9C27B0; color: white; font-weight: bold;")
        self.episode_next_btn.clicked.connect(self._on_next_episode_clicked)
        self.episode_next_btn.setVisible(False)
        row_result.addWidget(self.episode_next_btn)
        layout.addLayout(row_result)
        # Recovery 모드: 보정본 데이터셋 수집용 — Re1/Re2 각각 1, 5, 10, 25, 50개
        RECOVERY_COUNTS = (1, 5, 10, 25, 50)
        row_re1 = QHBoxLayout()
        row_re1.addWidget(QLabel("Recovery Re1:"))
        for n in RECOVERY_COUNTS:
            btn = QPushButton(f"{n}번")
            btn.setStyleSheet("background-color: #FF9800; color: white;")
            btn.clicked.connect(lambda checked, count=n: self.run_recovery_auto_collect(count, "re1"))
            btn.setEnabled(False)
            setattr(self, f"recovery_re1_{n}_btn", btn)
            row_re1.addWidget(btn)
        layout.addLayout(row_re1)
        row_re2 = QHBoxLayout()
        row_re2.addWidget(QLabel("Recovery Re2:"))
        for n in RECOVERY_COUNTS:
            btn = QPushButton(f"{n}번")
            btn.setStyleSheet("background-color: #FF5722; color: white;")
            btn.clicked.connect(lambda checked, count=n: self.run_recovery_auto_collect(count, "re2"))
            btn.setEnabled(False)
            setattr(self, f"recovery_re2_{n}_btn", btn)
            row_re2.addWidget(btn)
        layout.addLayout(row_re2)
        group.setLayout(layout)
        return group

    def _create_status_panel(self):
        group = QGroupBox("Status")
        layout = QGridLayout()
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: gray; font-weight: bold;")
        self.pose_label = QLabel("Position: N/A")
        self.pose_label.setWordWrap(True)
        layout.addWidget(QLabel("Connection:"), 0, 0)
        layout.addWidget(self.status_label, 0, 1)
        layout.addWidget(self.pose_label, 1, 0, 1, 2)
        group.setLayout(layout)
        return group

    def log(self, message):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {message}")
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def connect_robot(self):
        ip = self.ip_input.text().strip()
        try:
            port = int(self.port_input.text().strip())
        except ValueError:
            port = 29999
        self.log(f"Connecting to {ip}:{port}...")
        try:
            self.robot = DobotE6Controller(ip=ip, dashboard_port=port)
            if self.robot.connect():
                self.gripper = SuctionGripper(self.robot, do_index=1)
                self.status_label.setText("Connected")
                self.status_label.setStyleSheet("color: green; font-weight: bold;")
                self.connect_btn.setEnabled(False)
                self.disconnect_btn.setEnabled(True)
                self.move_pose_btn.setEnabled(True)
                self.home_btn.setEnabled(True)
                self.grip_btn.setEnabled(True)
                self.release_btn.setEnabled(True)
                self.pick_place_step_btn.setEnabled(True)
                for n in (1, 5, 10, 50):
                    getattr(self, f"auto_collect_{n}_btn").setEnabled(True)
                for n in (1, 5, 10, 25, 50):
                    getattr(self, f"recovery_re1_{n}_btn").setEnabled(True)
                    getattr(self, f"recovery_re2_{n}_btn").setEnabled(True)
                self.log("Robot connected")
                self.update_status()
            else:
                self.log("Connection failed")
                QMessageBox.warning(self, "Error", "Failed to connect to robot")
        except Exception as e:
            self.log(f"Error: {e}")
            QMessageBox.critical(self, "Error", str(e))

    def disconnect_robot(self):
        if self.robot:
            self.robot.disconnect()
            self.robot = None
            self.gripper = None
        self.status_label.setText("Disconnected")
        self.status_label.setStyleSheet("color: gray; font-weight: bold;")
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.move_pose_btn.setEnabled(False)
        self.home_btn.setEnabled(False)
        self.grip_btn.setEnabled(False)
        self.release_btn.setEnabled(False)
        self.pick_place_step_btn.setEnabled(False)
        for n in (1, 5, 10, 50):
            getattr(self, f"auto_collect_{n}_btn").setEnabled(False)
        for n in (1, 5, 10, 25, 50):
            getattr(self, f"recovery_re1_{n}_btn").setEnabled(False)
            getattr(self, f"recovery_re2_{n}_btn").setEnabled(False)
        self.stop_auto_btn.setEnabled(False)
        self.pose_label.setText("Position: N/A")
        self.log("Robot disconnected")

    def move_to_pose(self):
        if not self.robot or not self.robot.connected:
            return
        x = self.x_input.value()
        y = self.y_input.value()
        z = self.z_input.value()
        rx = self.rx_input.value()
        ry = self.ry_input.value()
        rz = self.rz_input.value()
        self.log(f"MoveJ to ({x:.1f}, {y:.1f}, {z:.1f}) mm")
        try:
            ok = self.robot.move_j(x, y, z, rx, ry, rz, coordinate_mode=0, use_waypoint=False)
            if ok:
                self.robot.wait_for_motion_complete()
                self.log("Motion complete")
                self.update_status()
            else:
                self.log("MoveJ failed (IK/collision?)")
                QMessageBox.warning(self, "Move Failed", "MoveJ failed. Check pose and robot state.")
        except Exception as e:
            self.log(f"Error: {e}")
            QMessageBox.critical(self, "Error", str(e))

    def go_home(self):
        if not self.robot or not self.robot.connected:
            return
        self.log("Going home...")
        try:
            ok = self.robot.move_j(300, 0, 400, 180, 0, 0, coordinate_mode=0, use_waypoint=False)
            if ok:
                self.robot.wait_for_motion_complete()
                self.log("Home reached")
                self.update_status()
            else:
                self.log("Home move failed")
        except Exception as e:
            self.log(f"Error: {e}")

    def grip(self):
        if self.gripper:
            self.gripper.grip()
            self.log("Gripper ON")

    def release(self):
        if self.gripper:
            self.gripper.release()
            self.log("Gripper OFF")

    def run_pick_place_step(self):
        """단일 스텝 실행 (항상 A섹션 4번 좌표에서 시작)."""
        if not self.robot or not self.robot.connected or not self.gripper:
            self.log("Connect robot first")
            return
        if self.step_worker and self.step_worker.isRunning():
            self.log("Step already running")
            return
        # 버튼을 누를 때마다 4번 좌표에서 시작
        self.current_pick_section = "A"
        self.last_place_x = None
        self.last_place_y = None
        self._begin_step_ui()
        do_record = self.record_20hz_cb.isChecked()
        if do_record:
            self.record_20hz_cb.setEnabled(False)
            self._ensure_camera_for_recording()
        self.step_worker = PickPlaceStepWorker(
            self.robot, self.gripper,
            pick_section=self.current_pick_section,
            pick_x=None, pick_y=None  # None이면 섹션의 고정 좌표 사용 (A섹션은 4번)
        )
        self.step_worker.log_signal.connect(self.log)
        self.step_worker.finished.connect(self.on_pick_place_step_finished)
        self.step_worker.episode_vacuum_durations.connect(self._on_episode_vacuum_durations)
        if do_record:
            self.step_worker.recording_begin_at_initial.connect(self._start_20hz_recording)
        self.step_worker.start()

    def run_recovery_mode(self, mode):
        """Recovery 모드 1회 실행 (re1 또는 re2)."""
        if not self.robot or not self.robot.connected or not self.gripper:
            self.log("Connect robot first")
            return
        if self.step_worker and self.step_worker.isRunning():
            self.log("Step already running")
            return
        self.current_pick_section = "A"
        self.last_place_x = None
        self.last_place_y = None
        self._begin_step_ui()
        do_record = self.record_20hz_cb.isChecked()
        if do_record:
            self.record_20hz_cb.setEnabled(False)
            self._ensure_camera_for_recording()
        self.log(f"Recovery {mode.upper()} 1회 시작")
        self.step_worker = PickPlaceStepWorker(
            self.robot, self.gripper,
            pick_section=self.current_pick_section,
            pick_x=None, pick_y=None,
            recovery_mode=mode
        )
        self.step_worker.log_signal.connect(self.log)
        self.step_worker.finished.connect(self.on_pick_place_step_finished)
        self.step_worker.episode_vacuum_durations.connect(self._on_episode_vacuum_durations)
        if do_record:
            self.step_worker.recording_begin_at_initial.connect(self._start_20hz_recording)
        self.step_worker.start()

    def run_recovery_auto_collect(self, n, mode):
        """Recovery 모드 N개 자동 수집 (re1 또는 re2)."""
        if not self.robot or not self.robot.connected or not self.gripper:
            self.log("Connect robot first")
            return
        if self.step_worker and self.step_worker.isRunning():
            self.log("Step already running")
            return
        self.auto_collect_target = n
        self.auto_collect_done = 0
        self.auto_collect_recovery_mode = mode
        self.current_pick_section = "A"
        self.last_place_x = None
        self.last_place_y = None
        self._begin_step_ui()
        self.record_20hz_cb.setChecked(True)
        self.record_20hz_cb.setEnabled(False)
        self.log(f"Recovery {mode.upper()} 자동 수집 시작: {n}개 (A섹션 4번 좌표에서 시작)")
        self._ensure_camera_for_recording()
        self.step_worker = PickPlaceStepWorker(
            self.robot, self.gripper,
            pick_section=self.current_pick_section,
            pick_x=None, pick_y=None,
            recovery_mode=mode
        )
        self.step_worker.log_signal.connect(self.log)
        self.step_worker.finished.connect(self.on_pick_place_step_finished)
        self.step_worker.episode_vacuum_durations.connect(self._on_episode_vacuum_durations)
        self.step_worker.recording_begin_at_initial.connect(self._start_20hz_recording)
        self.step_worker.start()

    def run_auto_collect(self, n):
        """N개 자동 수집 (20Hz 기록 자동 적용). 버튼을 누를 때마다 4번 좌표에서 시작."""
        if not self.robot or not self.robot.connected or not self.gripper:
            self.log("Connect robot first")
            return
        if self.step_worker and self.step_worker.isRunning():
            self.log("Step already running")
            return
        # 버튼을 누를 때마다 4번 좌표에서 시작
        self.auto_collect_target = n
        self.auto_collect_done = 0
        self.current_pick_section = "A"
        self.last_place_x = None
        self.last_place_y = None
        self._begin_step_ui()
        self.record_20hz_cb.setChecked(True)
        self.record_20hz_cb.setEnabled(False)
        self.log(f"자동 수집 시작: {n}개 (A섹션 4번 좌표에서 시작)")
        self._ensure_camera_for_recording()
        self.step_worker = PickPlaceStepWorker(
            self.robot, self.gripper,
            pick_section=self.current_pick_section,
            pick_x=None, pick_y=None,  # None이면 섹션의 고정 좌표 사용 (A섹션은 4번)
            recovery_mode=None  # 일반 모드
        )
        self.step_worker.log_signal.connect(self.log)
        self.step_worker.finished.connect(self.on_pick_place_step_finished)
        self.step_worker.episode_vacuum_durations.connect(self._on_episode_vacuum_durations)
        self.step_worker.recording_begin_at_initial.connect(self._start_20hz_recording)
        self.step_worker.start()

    def stop_auto_collect(self):
        """자동 수집 중단"""
        self.auto_collect_target = 0
        if self.step_worker and self.step_worker.isRunning():
            self.step_worker.request_stop()
        self.log("자동 수집 중단 요청")

    def _begin_step_ui(self):
        """스텝 시작 시 UI 비활성화"""
        self.pick_place_step_btn.setEnabled(False)
        self.move_pose_btn.setEnabled(False)
        self.home_btn.setEnabled(False)
        self.grip_btn.setEnabled(False)
        self.release_btn.setEnabled(False)
        for n in (1, 5, 10, 50):
            getattr(self, f"auto_collect_{n}_btn").setEnabled(False)
        for n in (1, 5, 10, 25, 50):
            getattr(self, f"recovery_re1_{n}_btn").setEnabled(False)
            getattr(self, f"recovery_re2_{n}_btn").setEnabled(False)
        self.stop_auto_btn.setEnabled(True)

    def _end_step_ui(self, auto_collect_active=False):
        """스텝 종료 시 UI 복원"""
        self.episode_next_btn.setVisible(False)
        if not auto_collect_active:
            self.pick_place_step_btn.setEnabled(True)
            self.move_pose_btn.setEnabled(True)
            self.home_btn.setEnabled(True)
            self.grip_btn.setEnabled(True)
            self.release_btn.setEnabled(True)
            for n in (1, 5, 10, 50):
                getattr(self, f"auto_collect_{n}_btn").setEnabled(True)
            for n in (1, 5, 10, 25, 50):
                getattr(self, f"recovery_re1_{n}_btn").setEnabled(True)
                getattr(self, f"recovery_re2_{n}_btn").setEnabled(True)
        self.stop_auto_btn.setEnabled(False)
        self.record_20hz_cb.setEnabled(True)

    def on_pick_place_step_finished(self, success):
        """한 스텝 끝남 → 무조건 성공 처리: 저장, 카운트, 위치 기억, [다음] 또는 UI 복원."""
        # Place 위치 기억 (다음 스텝의 Pick 위치로 사용)
        if self.step_worker and hasattr(self.step_worker, 'place_x') and self.step_worker.place_x is not None:
            self.last_place_x = self.step_worker.place_x
            self.last_place_y = self.step_worker.place_y
            # 섹션 교대: A섹션에서 Pick했으면 B섹션으로 Place했으므로, 다음은 B섹션에서 Pick
            # B섹션에서 Pick했으면 A섹션으로 Place했으므로, 다음은 A섹션에서 Pick
            self.current_pick_section = "B" if self.current_pick_section == "A" else "A"
            self.log(f"Place 위치 기억: ({self.last_place_x:.2f}, {self.last_place_y:.2f}), 다음 Pick 섹션: {self.current_pick_section}")
        
        do_record = self.record_20hz_cb.isChecked()
        if do_record:
            try:
                self._stop_20hz_recording_and_save(True)
            except Exception as e:
                self.log(f"20Hz 저장 중 오류: {e}")
                self.recording = False
                self.record_timer.stop()
                self.recorded_data.clear()
                self.record_save_dir = None
        if self.auto_collect_target > 0:
            self.auto_collect_done += 1
            self.log(f"자동 수집: {self.auto_collect_done}/{self.auto_collect_target} 완료 (초기 자세 복귀 → 저장됨)")
            if self.auto_collect_done >= self.auto_collect_target:
                self._end_auto_collect()
                return
            # 다음 스텝 자동 진행 (다음 버튼 없이)
            self.log(f"다음 스텝({self.auto_collect_done + 1}/{self.auto_collect_target}) 자동 시작...")
            QTimer.singleShot(300, self._run_next_auto_step)
        else:
            self._end_step_ui(auto_collect_active=False)
            self.update_status()
            self.log("Pick-Place step 완료.")

    def _on_next_episode_clicked(self):
        """다음 에피소드 시작 (자동 수집 시)."""
        self.episode_next_btn.setVisible(False)
        self._run_next_auto_step()

    def _end_auto_collect(self):
        self.auto_collect_target = 0
        self.auto_collect_done = 0
        self.auto_collect_recovery_mode = None
        self.episode_next_btn.setVisible(False)
        self._end_step_ui(auto_collect_active=False)
        self.update_status()
        self.log("자동 수집 완료.")

    def _run_next_auto_step(self):
        """다음 자동 수집 스텝 실행 (위치 기억 및 섹션 교대)"""
        if self.auto_collect_target <= 0 or self.auto_collect_done >= self.auto_collect_target:
            return
        if not self.robot or not self.robot.connected or not self.gripper:
            self._end_auto_collect()
            return
        
        # Pick 위치 결정: last_place_x, last_place_y가 있으면 그 위치에서 Pick, 없으면 섹션의 고정 좌표 사용
        pick_x = self.last_place_x if self.last_place_x is not None else None
        pick_y = self.last_place_y if self.last_place_y is not None else None
        
        recovery_mode = getattr(self, 'auto_collect_recovery_mode', None)
        self.step_worker = PickPlaceStepWorker(
            self.robot, self.gripper,
            pick_section=self.current_pick_section,
            pick_x=pick_x,
            pick_y=pick_y,
            recovery_mode=recovery_mode
        )
        self.step_worker.log_signal.connect(self.log)
        self.step_worker.finished.connect(self.on_pick_place_step_finished)
        self.step_worker.episode_vacuum_durations.connect(self._on_episode_vacuum_durations)
        self.step_worker.recording_begin_at_initial.connect(self._start_20hz_recording)
        self.step_worker.start()

    def emergency_stop(self):
        if self.robot:
            self.robot.disable_robot()
            self.log("E-STOP")
        if self.gripper:
            try:
                self.gripper.release()
            except Exception:
                pass
        QMessageBox.warning(self, "E-STOP", "Robot disabled. Re-enable from teach pendant if needed.")

    def update_status(self):
        if not self.robot or not self.robot.connected:
            return
        try:
            pose = self.robot.get_current_pose_from_feedback()
            if pose and len(pose) >= 6:
                self.pose_label.setText(
                    f"X={pose[0]:.1f} Y={pose[1]:.1f} Z={pose[2]:.1f} mm  |  "
                    f"RX={pose[3]:.1f} RY={pose[4]:.1f} RZ={pose[5]:.1f} °"
                )
        except Exception:
            pass

    def _get_next_folder_number(self):
        """vla_dataset 안에서 다음 폴더 번호(1, 2, 3, ...) 찾기."""
        if not os.path.exists(VLA_DATASET_BASE):
            return 1
        existing = [d for d in os.listdir(VLA_DATASET_BASE) if os.path.isdir(os.path.join(VLA_DATASET_BASE, d)) and d.isdigit()]
        if not existing:
            return 1
        max_num = max([int(d) for d in existing])
        return max_num + 1

    def _ensure_camera_for_recording(self):
        """20Hz 기록 전 카메라 준비: 꺼져 있으면 자동 시작 시도."""
        if not CAMERA_AVAILABLE:
            return
        if self.camera is None:
            self._init_camera()
        if self.camera and self.camera.initialized and not self.camera_active:
            try:
                self.camera_thread = CameraThread(self.camera)
                self.camera_thread.frame_ready.connect(self._update_camera_frame)
                self.camera_thread.start()
                self.camera_active = True
                self.start_camera_btn.setEnabled(False)
                self.stop_camera_btn.setEnabled(True)
                self.log("Camera auto-started for recording")
            except Exception as e:
                self.log(f"Camera auto-start failed: {e}")

    def _on_episode_vacuum_durations(self, pick_hold, place_hold):
        """Vacuum 명령 기반: worker가 emit한 grip/release 유지 시간(s) 저장 → metadata.txt에 기록."""
        self.episode_vacuum_pick_hold = float(pick_hold)
        self.episode_vacuum_place_hold = float(place_hold)

    def _start_20hz_recording(self):
        """20Hz 기록 시작: vla_dataset/(다음 번호) 생성, 타이머 시작. (초기 자세 도달 후 호출됨)"""
        folder_num = self._get_next_folder_number()
        self.record_save_dir = os.path.join(VLA_DATASET_BASE, str(folder_num))
        print(f"[DEBUG] Starting 20Hz recording: save_dir={self.record_save_dir} (folder #{folder_num})")
        os.makedirs(self.record_save_dir, exist_ok=True)
        os.makedirs(os.path.join(self.record_save_dir, "images"), exist_ok=True)
        self.recorded_data = []
        self.record_frame_count = 0
        self.recording = True
        self.record_timer.start(RECORD_INTERVAL_MS)
        print(f"[DEBUG] Recording flag set to True, timer started at {RECORD_INTERVAL_MS}ms")
        self.log(f"20Hz recording started (초기자세) → {self.record_save_dir}")

    def _on_record_tick(self):
        """20Hz: 로봇 피드백 + 카메라 프레임 저장."""
        if not self.recording or not self.robot or not self.robot.connected or not self.record_save_dir:
            return
        try:
            feed = self.robot.feed.feedBackData()
            if feed is None or len(feed) == 0:
                return
            joints = feed['QActual'][0].tolist()
            tcp_pose = feed['ToolVectorActual'][0].tolist()
            robot_mode = int(feed['RobotMode'][0]) if 'RobotMode' in feed.dtype.names else 0
            gripper_on = 1 if (self.gripper and self.gripper.is_gripping) else 0
            frame_filename = f"frame_{self.record_frame_count:06d}.jpg"
            frame_path = os.path.join(self.record_save_dir, "images", frame_filename)
            if self.camera and self.camera.initialized:
                ret, frame = self.camera.get_frame()
                if ret and frame is not None:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    cv2.imwrite(frame_path, frame_bgr)
                else:
                    cv2.imwrite(frame_path, np.zeros((480, 640, 3), dtype=np.uint8))
            else:
                cv2.imwrite(frame_path, np.zeros((480, 640, 3), dtype=np.uint8))
            record = {
                'frame_id': self.record_frame_count,
                'timestamp': time.time(),
                'image_path': frame_filename,
                'joint_angles': joints,
                'tcp_pose': tcp_pose,
                'gripper_tooldo1': gripper_on,
                'gripper_tooldo2': 0,
                'robot_mode': robot_mode
            }
            self.recorded_data.append(record)
            self.record_frame_count += 1
        except Exception as e:
            print(f"[20Hz Record Tick Error] {e}")
            import traceback
            traceback.print_exc()

    def _stop_20hz_recording_and_save(self, step_success):
        """20Hz 기록 중지. 성공한 에피소드만 저장(실패 시 저장 안 함)."""
        print(f"[DEBUG] _stop_20hz_recording_and_save called: success={step_success}, recorded_data={len(self.recorded_data)}")
        self.recording = False
        self.record_timer.stop()
        if not self.record_save_dir or len(self.recorded_data) == 0:
            self.log(f"20Hz: no data to save. (save_dir={self.record_save_dir}, data_count={len(self.recorded_data)})")
            print(f"[DEBUG] No data: save_dir={self.record_save_dir}, recorded_data count={len(self.recorded_data)}")
            return
        try:
            print(f"[DEBUG] Saving {len(self.recorded_data)} records to {self.record_save_dir}...")
            csv_path = os.path.join(self.record_save_dir, "robot_data.csv")
            print(f"[DEBUG] Writing CSV: {csv_path}")
            with open(csv_path, 'w', newline='') as f:
                f.write("frame_id,timestamp,image_path,")
                f.write("j1,j2,j3,j4,j5,j6,")
                f.write("x,y,z,rx,ry,rz,")
                f.write("gripper_tooldo1,gripper_tooldo2,robot_mode\n")
                for r in self.recorded_data:
                    f.write(f"{r['frame_id']},{r['timestamp']},{r['image_path']},")
                    f.write(','.join(map(str, r['joint_angles'])) + ',')
                    f.write(','.join(map(str, r['tcp_pose'])) + ',')
                    f.write(f"{r['gripper_tooldo1']},{r['gripper_tooldo2']},{r['robot_mode']}\n")
            print(f"[DEBUG] CSV written: {csv_path}")
            npy_path = os.path.join(self.record_save_dir, "dataset.npy")
            np.save(npy_path, self.recorded_data)
            print(f"[DEBUG] NPY written: {npy_path}")
            meta_path = os.path.join(self.record_save_dir, "metadata.txt")
            with open(meta_path, 'w') as f:
                folder_num = os.path.basename(self.record_save_dir)
                f.write("VLA Dataset - Pick-Place Step 20Hz\n")
                f.write("=" * 50 + "\n\n")
                f.write(f"Folder: {folder_num}\n")
                f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Total Frames: {len(self.recorded_data)}\n")
                f.write(f"Record Rate: 20Hz\n")
                f.write(f"Step Success: {step_success}\n")
                f.write(f"Success: {step_success}\n")
                # Vacuum 명령 기반 (ON 유지 시간, s) — 센서 없을 때 분석용
                f.write(f"VacuumCommandPickDuration_s: {self.episode_vacuum_pick_hold:.3f}\n")
                f.write(f"VacuumCommandPlaceDuration_s: {self.episode_vacuum_place_hold:.3f}\n")
            print(f"[DEBUG] Metadata written: {meta_path}")
            self.log(f"20Hz saved: {len(self.recorded_data)} frames → {self.record_save_dir}")
            print(f"[DEBUG] Save complete: {len(self.recorded_data)} frames")
        except Exception as e:
            self.log(f"20Hz save error: {e}")

    def start_camera(self):
        if not CAMERA_AVAILABLE:
            self.log("Camera module not available")
            QMessageBox.warning(self, "Camera", "Camera module not available.")
            return
        if self.camera is None:
            self._init_camera()
        if self.camera is None or not self.camera.initialized:
            self.log("Camera not initialized")
            QMessageBox.warning(self, "Camera", "Camera init failed.")
            return
        if self.camera_active:
            return
        try:
            self.camera_thread = CameraThread(self.camera)
            self.camera_thread.frame_ready.connect(self._update_camera_frame)
            self.camera_thread.start()
            self.camera_active = True
            self.start_camera_btn.setEnabled(False)
            self.stop_camera_btn.setEnabled(True)
            self.log("Camera started")
        except Exception as e:
            self.log(f"Camera start error: {e}")

    def stop_camera(self):
        if self.camera_thread:
            self.camera_thread.stop()
            self.camera_thread.wait()
            self.camera_thread = None
        self.camera_active = False
        self.start_camera_btn.setEnabled(True)
        self.stop_camera_btn.setEnabled(False)
        self.camera_label.setText("Camera (Start to view)")
        self.log("Camera stopped")

    def _update_camera_frame(self, frame):
        try:
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            scaled = pixmap.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.camera_label.setPixmap(scaled)
        except Exception as e:
            print(f"Frame update error: {e}")

    def closeEvent(self, event):
        if self.camera_active:
            self.stop_camera()
        if self.camera:
            try:
                self.camera.cleanup()
            except Exception:
                pass
        if self.robot and self.robot.connected:
            reply = QMessageBox.question(
                self, "Exit",
                "Robot is connected. Disconnect and exit?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.disconnect_robot()
                event.accept()
            else:
                event.ignore()
        else:
            event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gui = PickPlaceGUINew()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
