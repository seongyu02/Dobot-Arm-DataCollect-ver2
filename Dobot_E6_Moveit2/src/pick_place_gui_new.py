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

SIDE_CAMERA_AVAILABLE = True


class SideUsbCamera:
    """Generic USB camera for OBS_IMAGE_2 using OpenCV VideoCapture."""

    def __init__(self, camera_id=1, width=640, height=480):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.cap = None
        self.initialized = False

    def init_camera(self) -> bool:
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.initialized = bool(self.cap and self.cap.isOpened())
            return self.initialized
        except Exception:
            self.initialized = False
            return False

    def get_frame(self):
        if not self.initialized or self.cap is None:
            return False, None
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return False, None
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return True, rgb

    def cleanup(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        self.initialized = False

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

if SIDE_CAMERA_AVAILABLE:
    class SideCameraThread(QThread):
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

# SmolVLA 데이터 수집 권장 정책: 5개 시작 위치 × 각 10개 성공 trajectory
SMOLVLA_SUCCESS_TARGET_PER_POSE = 10
SMOLVLA_STRICT_TOTAL_TARGET = 50
SMOLVLA_START_POSE_SPECS = [
    {"id": "pose_A_2", "section": "A", "xy": (POS_2[0], POS_2[1])},
    {"id": "pose_A_3", "section": "A", "xy": (POS_3[0], POS_3[1])},
    {"id": "pose_A_4", "section": "A", "xy": (POS_4[0], POS_4[1])},
    {"id": "pose_B_8", "section": "B", "xy": (POS_8[0], POS_8[1])},
    {"id": "pose_B_9", "section": "B", "xy": (POS_9[0], POS_9[1])},
]
SMOLVLA_START_POSE_MAP = {spec["id"]: spec for spec in SMOLVLA_START_POSE_SPECS}
SMOLVLA_START_POSE_ORDER = [spec["id"] for spec in SMOLVLA_START_POSE_SPECS]

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

# Phase 1: 섹션 다각형을 centroid 기준 alpha=0.6 선형 축소 (docs/PHASE1_COLLECTION_SPEC.md)
A_CENTER_SMALL_POINTS = [
    (143.5293, -408.8776),
    (147.2613, -396.1816),
    (190.5573, -391.0816),
    (192.0333, -368.9236),
    (192.8853, -338.7256),
    (116.3673, -334.6156),
    (110.8893, -410.4256),
]
B_CENTER_SMALL_POINTS = [
    (70.3955, -338.0031),
    (64.9175, -413.8131),
    (-2.2765, -414.3591),
    (4.7075, -343.9731),
]


def _polygon_centroid_xy(polygon):
    """폐곡선 꼭짓점들의 산술 평균 (Phase1 고정 place/pick 보조점)."""
    xs = [pt[0] for pt in polygon]
    ys = [pt[1] for pt in polygon]
    n = len(polygon)
    return (sum(xs) / n, sum(ys) / n)


# Phase1 Define: source/target XY 고정 (INIT와 별도). A/B는 각각 center_small 다각형의 중심 한 점.
PHASE1_FIXED_A_CENTER_SMALL_XY = _polygon_centroid_xy(A_CENTER_SMALL_POINTS)
PHASE1_FIXED_B_CENTER_SMALL_XY = _polygon_centroid_xy(B_CENTER_SMALL_POINTS)
PHASE1_FIXED_PICK_A_XY = PHASE1_FIXED_A_CENTER_SMALL_XY
PHASE1_FIXED_PLACE_B_XY = PHASE1_FIXED_B_CENTER_SMALL_XY
PHASE1_FIXED_PICK_B_XY = PHASE1_FIXED_B_CENTER_SMALL_XY
PHASE1_FIXED_PLACE_A_XY = PHASE1_FIXED_A_CENTER_SMALL_XY


# Image–state row 규약 (한 row 안 처리 순서; _on_record_tick 와 동일 의미로 문서화)
SYNC_POLICY_STRING = "feedback_first_camera_second_row_timestamp_end"

# B섹션 Place 시 카메라에 잘 보이는 XY 영역 (로봇 하단에 가리지 않음)
# 5개 참조 포즈 기준 다각형, 순서: 2 -> 1 -> 3 -> 4 -> 5 (닫힌 다각형)
B_SECTION_PLACE_VISIBLE_POINTS = [
    (-58.01, -413.24),   # 참조 1
    (2.75, -385.76),     # 참조 2
    (133.13, -339.62),   # 참조 3
    (228.26, -250.57),   # 참조 4 (좌측에서 잘 보임)
    (261.52, -122.80),   # 참조 5 (좌측에서 잘 보임)
]

# A섹션 회전값 (시작 위치 4번 좌표 회전값 사용)
A_SECTION_RX, A_SECTION_RY, A_SECTION_RZ = 179.75, 2.03, -1.86  # 4번 좌표 회전값 사용
# B섹션 회전값 (평균 또는 대표값 사용)
B_SECTION_RX, B_SECTION_RY, B_SECTION_RZ = -179.15, 0.60, 3.41  # 9번 좌표 회전값 사용
# B섹션 7-8 구간 접근 시 사용할 RPY (해당 구간으로 접근 시 충돌 방지용)
B_SECTION_78_APPROACH_RX = -174.6
B_SECTION_78_APPROACH_RY = 1.22
B_SECTION_78_APPROACH_RZ = 2.75
B_SECTION_78_SEGMENT_RADIUS_MM = 60.0   # place 목표가 7-8 변에서 이 거리(mm) 안이면 7-8 접근 RPY 적용

# Pick/Place 시 Y에 따른 RX 보정 파라미터
DYN_RX_Y_NEAR = -460.0   # 뒤쪽(카메라 쪽) Y
DYN_RX_Y_FAR = -260.0    # 앞쪽 Y
DYN_RX_NEAR = -150.0     # 뒤쪽에서의 RX
DYN_RX_FAR = -175.0      # 앞쪽에서의 RX (더 눕힘)

# 빨간 블록 가림 방지용 탐색 파라미터 (카메라 시야 확보용, X- 방향으로만 탐색)
RED_SEARCH_X_LIMIT_MIN = -71.94   # 사용자가 지정한 LIMIT 포즈의 X (이보다 더 X-로는 이동 금지)
RED_SEARCH_X_STEP = -50.0         # 한 번에 이동할 X- 스텝(mm)
RED_SEARCH_MAX_STEPS = 4          # 최대 시도 횟수
RED_DETECT_MIN_AREA = 500         # 빨간 블록 감지용 최소 픽셀 수 (튜닝 가능)

RELEASE_Z = 122.7   # 그리퍼 해제/픽 높이 (물체 높이 변경 반영)
RELEASE_Z_TOLERANCE_MM = 1.5   # Z 도달 판정 허용 오차 (대략 100.2~103.2)
PLACE_WAIT_AT_101_S = 2.0   # Place 시 Z 도달 후 release 전 대기 시간(초)
Z_MOVE_MIN, Z_MOVE_MAX = 150.0, 300.0   # Pick/Place 후 궤적용 Z 랜덤 범위 (최소 150~최대 300)
Z_AFTER_RELEASE = 200.0   # Release 후 Z 높이

# Pick/Place 공통: Z 130에서 고정 후 101까지 하강 (사용자 지정)
DESCENT_MID_Z = 130.0   # 130mm에서 잠시 고정 후 101로 (Pick·Place 동일)
# 130→101 구간 RPY (Teach pendant 기준, Pick·Place 동일)
DESCENT_RX = 176.4624
DESCENT_RY = -1.7726
DESCENT_RZ = 8.1319
DESCENT_VELOCITY = 14.0   # 130→101 하강 속도 (8보다 상향)
# 130→101 하강 시 Rx 오프셋 (그립 정렬 보정)
DESCENT_RX_OFFSET_DEG = 1.0

# 높은 Z 이동용 공통 RPY (꺾임 방지 — INIT↔픽, 픽↔플레이스, 플레이스↔INIT 에서 사용)
TRAVEL_RX = DESCENT_RX
TRAVEL_RY = DESCENT_RY
TRAVEL_RZ = DESCENT_RZ

INIT_X, INIT_Y, INIT_Z = 89.3715, -102.9836, 611.7122   # INIT = 대기(홈) 위치
INIT_RX, INIT_RY, INIT_RZ = 90.1244, 3.6761, 5.7400

# 130→101 하강 시 (x,y)별 RPY 보정 테이블 (descent_calib_result.py 또는 descent_calibration.DESCENT_CALIB_TABLE)
_DESCENT_CALIB_TABLE = None

def _load_descent_calib_table():
    global _DESCENT_CALIB_TABLE
    if _DESCENT_CALIB_TABLE is not None:
        return _DESCENT_CALIB_TABLE
    try:
        import descent_calib_result as _m
        _DESCENT_CALIB_TABLE = getattr(_m, "DESCENT_CALIB_TABLE", [])
    except Exception:
        try:
            import descent_calibration as _m
            _DESCENT_CALIB_TABLE = getattr(_m, "DESCENT_CALIB_TABLE", [])
        except Exception:
            _DESCENT_CALIB_TABLE = []
    return _DESCENT_CALIB_TABLE

def get_descent_rpy(x, y):
    """(x, y)에서 130→101 하강 시 사용할 (rx, ry, rz). 테이블 없으면 DESCENT_RX/RY/RZ 반환."""
    table = _load_descent_calib_table()
    if not table:
        rx = float(DESCENT_RX) + float(DESCENT_RX_OFFSET_DEG)
        rx = max(-180.0, min(180.0, rx))
        return (rx, DESCENT_RY, DESCENT_RZ)
    best = None
    best_d2 = 1e30
    for row in table:
        if len(row) < 5:
            continue
        x0, y0, rx, ry, rz = row[0], row[1], row[2], row[3], row[4]
        d2 = (x - x0) ** 2 + (y - y0) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best = (rx, ry, rz)
    if best is None:
        rx = float(DESCENT_RX) + float(DESCENT_RX_OFFSET_DEG)
        rx = max(-180.0, min(180.0, rx))
        return (rx, DESCENT_RY, DESCENT_RZ)
    rx, ry, rz = best
    rx = float(rx) + float(DESCENT_RX_OFFSET_DEG)
    rx = max(-180.0, min(180.0, rx))
    return (rx, ry, rz)

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

# 실측 기록 레이트 (QTimer 50ms + 카메라 오버헤드 → 실제 ~15Hz)
RECORD_INTERVAL_MS = 50
RECORD_RATE_HZ = 15

# 방향 분류 (PRIMITIVE_SEGMENTATION_SPEC §3 기준)
_MIDDLE_DIAMOND = [(83.41, -339.22), (112.31, -378.54), (83.41, -417.86), (54.51, -378.54)]

def _in_middle_diamond(x, y):
    return point_in_polygon(x, y, _MIDDLE_DIAMOND)

def _transport_direction(place_x, place_y):
    """place 좌표 기준 transport primitive 반환."""
    if _in_middle_diamond(place_x, place_y):
        return "move_to_middle"
    return "move_left" if place_x >= 85.0 else "move_right"
VLA_DATASET_BASE = os.path.join(workspace_root, "vla_dataset_smolvla")
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
    episode_meta_ready = pyqtSignal(object)   # 에피소드 semantic 메타 dict

    def __init__(self, robot, gripper, pick_section="A",
                 pick_x=None, pick_y=None, pick_rx=None, pick_ry=None, pick_rz=None,
                 camera=None, fallback_initial_pose=None, phase1_episode_direction=None,
                 pick_only=False,
                 phase1_pick_xy_origin=None,
                 mode="pick_place",
                 reset_place_x=None, reset_place_y=None, drop_pose_id=None):
        super().__init__()
        self.robot = robot
        self.gripper = gripper
        self.camera = camera  # HikRobotCamera (또는 None)
        self._stop_requested = False
        # None | "A_to_B" | "B_to_A" — Phase 1 전용: center_small 샘플, last_place 비활성 GUI와 함께 사용
        self.phase1_episode_direction = phase1_episode_direction
        # Phase1 B_to_A: previous_episode_place = 직전 AB place 좌표를 pick source로 (메타에 명시)
        self.phase1_pick_xy_origin = phase1_pick_xy_origin
        self.phase1_place_xy_origin = None  # Phase1: place target 메타용
        self.mode = str(mode)
        if bool(pick_only) and self.mode == "pick_place":
            self.mode = "pick_only"
        self.pick_only = (self.mode == "pick_only")
        self.reset_place_x = reset_place_x
        self.reset_place_y = reset_place_y
        self.drop_pose_id = drop_pose_id
        self.pick_section = pick_section  # "A" or "B"
        self.pick_x = pick_x  # None이면 섹션의 고정 좌표 사용 (A섹션은 4번, B섹션은 랜덤)
        self.pick_y = pick_y
        self.pick_rx = pick_rx
        self.pick_ry = pick_ry
        self.pick_rz = pick_rz
        # 첫 MovJ(INIT) 실패 시 시도할 대체 포즈 (x,y,z,rx,ry,rz) 또는 None
        self.fallback_initial_pose = fallback_initial_pose
        # Place 위치 저장 (다음 스텝에서 사용)
        self.place_x = None
        self.place_y = None
        self.place_section = None
        self._events = []   # [(event_name, timestamp), ...]

    def _run_reset_place_mode(self):
        """비기록 reset step: 현재 잡고 있는 물체를 지정 drop pose에 놓고 INIT 복귀."""
        target_x = float(self.reset_place_x) if self.reset_place_x is not None else float(POS_4[0])
        target_y = float(self.reset_place_y) if self.reset_place_y is not None else float(POS_4[1])
        self._events = []
        self._log(f"[reset_place] target=({target_x:.2f}, {target_y:.2f}), drop_pose_id={self.drop_pose_id}")

        # 1) INIT 정렬 (안전 시작)
        if not self._move(INIT_X, INIT_Y, INIT_Z, rx=INIT_RX, ry=INIT_RY, rz=INIT_RZ):
            self._log("[reset_place] Failed: move to INIT")
            self._fail_and_go_home()
            return
        if self._stop_requested:
            self._fail_and_go_home()
            return

        # 2) 목표 XY 상공으로 이동
        z_move = random.uniform(Z_MOVE_MIN, Z_MOVE_MAX)
        if not self._move(target_x, target_y, z_move, velocity=24.0, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ):
            self._log("[reset_place] Failed: move to target XY")
            self._fail_and_go_home()
            return

        # 3) 하강 후 release
        if not self._move(target_x, target_y, DESCENT_MID_Z, velocity=18.0, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ):
            self._log("[reset_place] Failed: descend to mid Z")
            self._fail_and_go_home()
            return

        drop_drx, drop_dry, drop_drz = get_descent_rpy(target_x, target_y)
        if not self._move(target_x, target_y, RELEASE_Z, velocity=DESCENT_VELOCITY, rx=drop_drx, ry=drop_dry, rz=drop_drz):
            self._log("[reset_place] Failed: descend to release Z")
            self._fail_and_go_home()
            return

        if not self._wait_until_z_reached(RELEASE_Z, tolerance_mm=RELEASE_Z_TOLERANCE_MM, timeout_s=8.0):
            self._log("[reset_place] Release Z confirmation failed")
            self._fail_and_go_home()
            return

        place_hold = random.uniform(PLACE_HOLD_TIME_LO, PLACE_HOLD_TIME_HI)
        self._events.append(("gripper_off", time.time()))
        if self.gripper:
            self.gripper.release(wait_time=place_hold)

        # 4) 상승 후 INIT 복귀
        if not self._move(target_x, target_y, Z_AFTER_RELEASE, velocity=28.0, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ):
            self._log("[reset_place] Failed: lift after release")
            self._fail_and_go_home()
            return
        if not self._move(INIT_X, INIT_Y, INIT_Z, rx=INIT_RX, ry=INIT_RY, rz=INIT_RZ):
            self._log("[reset_place] Failed: return to INIT")
            self._fail_and_go_home()
            return

        self.episode_vacuum_durations.emit(0.0, place_hold)
        self.episode_meta_ready.emit({
            "task_name": "reset_place",
            "reset_only": True,
            "drop_pose_id": self.drop_pose_id,
            "drop_x": round(target_x, 3),
            "drop_y": round(target_y, 3),
            "events": list(self._events),
            "success": True,
        })
        self.finished.emit(True)

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
        """중간 실패 시 초기 자세로 복귀. stop_requested면 False 반환 → 저장 안 함."""
        self.place_x = None
        self.place_y = None
        self._safe_return_home()
        # 수동 STOP으로 중단된 경우 저장하지 않음
        self.finished.emit(not self._stop_requested)

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

    def _compute_dynamic_rx(self, y, base_rx):
        """
        Y가 + 방향(덜 음수)으로 갈수록 RX를 더 음수(-)로 눕혀 관절각을 안정화.
        Y가 DYN_RX_Y_NEAR~DYN_RX_Y_FAR 범위 밖이면 클램핑.
        """
        y_clamped = max(DYN_RX_Y_NEAR, min(DYN_RX_Y_FAR, y))
        if DYN_RX_Y_FAR == DYN_RX_Y_NEAR:
            t = 0.0
        else:
            t = (y_clamped - DYN_RX_Y_NEAR) / (DYN_RX_Y_FAR - DYN_RX_Y_NEAR)
        target_rx = DYN_RX_NEAR + t * (DYN_RX_FAR - DYN_RX_NEAR)
        return target_rx

    def _is_red_block_visible(self):
        """카메라 프레임에서 빨간 블록이 보이는지 OpenCV로 판단."""
        if not self.camera or not getattr(self.camera, "initialized", False):
            return True  # 카메라 없으면 추가 탐색 없이 통과
        try:
            ret, frame = self.camera.get_frame()
            if not ret or frame is None:
                return True
            # frame: RGB 가정 → HSV로 변환
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            # 빨간색은 HSV에서 두 구간 (0~10, 160~180)
            lower_red1 = np.array([0, 120, 70], dtype=np.uint8)
            upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
            lower_red2 = np.array([160, 120, 70], dtype=np.uint8)
            upper_red2 = np.array([180, 255, 255], dtype=np.uint8)
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            area = int(cv2.countNonZero(mask))
            return area >= RED_DETECT_MIN_AREA
        except Exception as e:
            self._log(f"Red block visibility check failed: {e}")
            return True

    def _phase1_visible_init_gate(self):
        """Phase1: INIT 직후 녹화 전 — 프레임이 너무 어둡지 않고 빨간 블록 마스크가 충분한지."""
        if not self.camera or not getattr(self.camera, "initialized", False):
            return True
        try:
            ret, frame = self.camera.get_frame()
            if not ret or frame is None:
                return False
            gray = float(np.mean(cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)))
            if gray < 8.0:
                self._log("Phase1 visible_init_gate: frame too dark")
                return False
            if not self._is_red_block_visible():
                self._log("Phase1 visible_init_gate: red blob below threshold")
                return False
            return True
        except Exception as e:
            self._log(f"Phase1 visible_init_gate error: {e}")
            return False

    def _search_x_for_visibility(self, current_z, rx, ry, rz):
        """
        현재 포즈에서 X- 방향으로 50mm씩 최대 RED_SEARCH_MAX_STEPS번 이동하며
        빨간 블록이 보일 때까지 탐색. X가 RED_SEARCH_X_LIMIT_MIN보다 작아지면 중단.

        피드백 캐시 버그 방지: cur_x를 루프 내에서 재조회하지 않고
        이동 후 next_x로 직접 갱신한다.
        """
        if not self.camera or not getattr(self.camera, "initialized", False):
            return
        # 현재 위치는 루프 시작 전 한 번만 읽는다
        pose = self.robot.get_current_pose_from_feedback()
        if not pose or len(pose) < 2:
            return
        cur_x = float(pose[0])
        cur_y = float(pose[1])
        for _ in range(int(RED_SEARCH_MAX_STEPS)):
            if self._stop_requested:
                return
            if self._is_red_block_visible():
                self._log("Red block visible — no further X- search needed.")
                return
            if cur_x <= RED_SEARCH_X_LIMIT_MIN:
                self._log(f"X limit reached (X={cur_x:.1f} <= {RED_SEARCH_X_LIMIT_MIN}) — stop X- search.")
                return
            next_x = max(RED_SEARCH_X_LIMIT_MIN, cur_x + RED_SEARCH_X_STEP)
            self._log(f"Red block not visible → moving X- search step to X={next_x:.1f}mm")
            if not self._move(next_x, cur_y, current_z, velocity=60.0, rx=rx, ry=ry, rz=rz):
                return
            cur_x = next_x  # 이동 완료 후 로컬 변수로 갱신 (피드백 재조회 없음)

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

            if self.mode == "reset_place":
                self._run_reset_place_mode()
                return

            # Pick 위치 결정
            if self.phase1_episode_direction == "A_to_B":
                self.pick_section = "A"
                pick_x, pick_y = PHASE1_FIXED_PICK_A_XY
                self.phase1_pick_xy_origin = "fixed_anchor_A_center_small_centroid"
                pick_z = RELEASE_Z
                pick_rx, pick_ry, pick_rz = A_SECTION_RX, A_SECTION_RY, A_SECTION_RZ
                place_section = "B"
                place_section_points = B_SECTION_POINTS
                place_rx, place_ry, place_rz = B_SECTION_RX, B_SECTION_RY, B_SECTION_RZ
                place_x, place_y = PHASE1_FIXED_PLACE_B_XY
                self.phase1_place_xy_origin = "fixed_anchor_B_center_small_centroid"
                place_z = RELEASE_Z
            elif self.phase1_episode_direction == "B_to_A":
                self.pick_section = "B"
                if (
                    self.phase1_pick_xy_origin == "previous_episode_place"
                    and self.pick_x is not None
                    and self.pick_y is not None
                ):
                    px, py = float(self.pick_x), float(self.pick_y)
                    if point_in_polygon(px, py, B_CENTER_SMALL_POINTS):
                        pick_x, pick_y = px, py
                        self._log(
                            f"Phase1 B_to_A: pick from previous_episode_place ({pick_x:.2f}, {pick_y:.2f})"
                        )
                    else:
                        self._log(
                            "Phase1 B_to_A: previous place outside B_center_small → fixed B center anchor"
                        )
                        pick_x, pick_y = PHASE1_FIXED_PICK_B_XY
                        self.phase1_pick_xy_origin = "fixed_anchor_B_center_small_centroid"
                else:
                    pick_x, pick_y = PHASE1_FIXED_PICK_B_XY
                    self.phase1_pick_xy_origin = "fixed_anchor_B_center_small_centroid"
                pick_z = RELEASE_Z
                pick_rx, pick_ry, pick_rz = B_SECTION_RX, B_SECTION_RY, B_SECTION_RZ
                place_section = "A"
                place_section_points = A_SECTION_POINTS
                place_rx, place_ry, place_rz = A_SECTION_RX, A_SECTION_RY, A_SECTION_RZ
                place_x, place_y = PHASE1_FIXED_PLACE_A_XY
                self.phase1_place_xy_origin = "fixed_anchor_A_center_small_centroid"
                place_z = RELEASE_Z
            elif self.pick_section == "A":
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
                # Place 위치: 반대 섹션 내 랜덤 좌표 생성
                place_x, place_y = generate_random_point_in_section(B_SECTION_POINTS)
                place_z = RELEASE_Z
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
                place_x, place_y = generate_random_point_in_section(place_section_points)
                place_z = RELEASE_Z
            # Place 위치 Y에 따라 RX 동적 보정 (특수 7-8 구간 RPY 적용 전)
            place_rx = self._compute_dynamic_rx(place_y, place_rx)
            # B섹션 7-8 구간 근처면 해당 구간 접근용 RPY 사용 (이미지 기준 접근 자세)
            if place_section == "B":
                d78 = self._dist_point_to_line(place_x, place_y, POS_7[0], POS_7[1], POS_8[0], POS_8[1])
                if d78 <= B_SECTION_78_SEGMENT_RADIUS_MM:
                    place_rx, place_ry, place_rz = B_SECTION_78_APPROACH_RX, B_SECTION_78_APPROACH_RY, B_SECTION_78_APPROACH_RZ
                    self._log("Place 목표가 7-8 구간 근처 → 7-8 접근 자세(RPY) 적용")
            # Place 위치 저장 (다음 스텝에서 사용)
            self.place_x = place_x
            self.place_y = place_y
            self.place_section = place_section
            self.episode_pick_x = pick_x
            self.episode_pick_y = pick_y
            self._events = []

            # 1) 초기 위치
            self._log("1) Initial position...")
            if not self._move(INIT_X, INIT_Y, INIT_Z, rx=INIT_RX, ry=INIT_RY, rz=INIT_RZ):
                if self.fallback_initial_pose and len(self.fallback_initial_pose) >= 6:
                    self._log("Initial move failed → trying fallback pose...")
                    fx, fy, fz, frx, fry, frz = self.fallback_initial_pose[:6]
                    if not self._move(fx, fy, fz, rx=frx, ry=fry, rz=frz):
                        self._log("Failed: initial position (and fallback)")
                        self._fail_and_go_home()
                        return
                else:
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
            if self.phase1_episode_direction:
                if not self._phase1_visible_init_gate():
                    self._log("Phase1: visible_init_gate failed — recording not started")
                    self.finished.emit(False)
                    return
            self.recording_begin_at_initial.emit()

            # 2) Pick 위치로 이동 — 공통 RPY로 꺾임 방지
            z_move = random.uniform(Z_MOVE_MIN, Z_MOVE_MAX)
            self._log(f"2) Moving to {self.pick_section} section Pick position (Z={z_move:.1f}mm, travel RPY)...")
            if not self._move(pick_x, pick_y, z_move, velocity=24.0, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ):
                self._log("Failed: move to pick position")
                self._fail_and_go_home()
                return
            # 2.5) 빨간 블록이 팔에 가려져 카메라에 보이지 않으면 X- 방향으로 탐색 (A섹션에서만 적용)
            if self.pick_section == "A":
                self._log("2.5) Checking red block visibility before Pick (X- search)...")
                self._search_x_for_visibility(z_move, TRAVEL_RX, TRAVEL_RY, TRAVEL_RZ)

            # 3) Pick 하강: 130까지 공통 RPY, 130→101.7 은 (x,y)별 보정 RPY
            self._log(f"3) Descending to Z={DESCENT_MID_Z:.1f} for Pick (travel RPY)...")
            if not self._move(pick_x, pick_y, DESCENT_MID_Z, velocity=18.0, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ):
                self._log(f"Failed: descend to Z={DESCENT_MID_Z:.1f} for pick")
                self._fail_and_go_home()
                return

            pick_drx, pick_dry, pick_drz = get_descent_rpy(pick_x, pick_y)
            self._log(
                f"3.1) Descending to Z={RELEASE_Z:.1f} for Pick (descent RPY: "
                f"RX={pick_drx:.2f} RY={pick_dry:.2f} RZ={pick_drz:.2f})..."
            )
            if not self._move(pick_x, pick_y, RELEASE_Z, velocity=DESCENT_VELOCITY, rx=pick_drx, ry=pick_dry, rz=pick_drz):
                self._log(f"Failed: descend to Z={RELEASE_Z:.1f} for pick")
                self._fail_and_go_home()
                return

            # Z=101.7 도달 확인 후 그립 (끝까지 내려온 뒤에만 집기)
            if not self._wait_until_z_reached(RELEASE_Z, tolerance_mm=RELEASE_Z_TOLERANCE_MM):
                self._log("Pick: Z=101 도달 확인 실패 → 그립하지 않고 복귀")
                self._fail_and_go_home()
                return

            pick_hold = random.uniform(PICK_HOLD_TIME_LO, PICK_HOLD_TIME_HI)
            self._log(f"3) Gripper ON at Pick (Z=101), holding {pick_hold:.2f}s...")
            self._events.append(("gripper_on", time.time()))
            self.gripper.grip(wait_time=pick_hold)
            time.sleep(0.3)  # 추가 유지 시간
            if self._stop_requested:
                self._release_at_safe_z(pick_x, pick_y, rx=pick_drx, ry=pick_dry, rz=pick_drz)
                self._fail_and_go_home()
                return

            # 4) Pick 후: Z=130 → 랜덤 Z 까지 공통 RPY로 올림 (꺾임 방지)
            self._log(f"4) Lifting to Z={DESCENT_MID_Z:.1f}mm with object (travel RPY)...")
            if not self._move(pick_x, pick_y, DESCENT_MID_Z, velocity=18.0, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ):
                self._log("Failed: lift to Z=130 after pick")
                self._release_at_safe_z(pick_x, pick_y, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ)
                self._fail_and_go_home()
                return
            z_move2 = random.uniform(Z_MOVE_MIN, Z_MOVE_MAX)
            self._log(f"4.1) Lifting to Z={z_move2:.1f}mm (travel RPY)...")
            if not self._move(pick_x, pick_y, z_move2, velocity=24.0, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ):
                self._log("Failed: lift to random Z")
                self._release_at_safe_z(pick_x, pick_y, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ)
                self._fail_and_go_home()
                return

            # Pick-only 모드: Place 생략, 집기 성공 안정화에 집중
            if self.mode == "pick_only":
                self.place_x = None
                self.place_y = None
                self.place_section = None
                self._log("Pick-only mode: skipping Place and returning to initial position...")
                if not self._move(pick_x, pick_y, Z_AFTER_RELEASE, velocity=28.0, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ):
                    self._log("Failed: lift to return height in pick-only mode")
                    self._fail_and_go_home()
                    return
                if not self._move(INIT_X, INIT_Y, INIT_Z, rx=INIT_RX, ry=INIT_RY, rz=INIT_RZ):
                    self._log("Failed: return to initial in pick-only mode")
                    self._fail_and_go_home()
                    return
                self._log(f"Step complete ({self.pick_section} section Pick only).")
                self.episode_vacuum_durations.emit(pick_hold, 0.0)
                self.episode_meta_ready.emit({
                    "task_name": "pick_only",
                    "object_label": "red_block",
                    "pick_section": self.pick_section,
                    "pick_x": round(pick_x, 3),
                    "pick_y": round(pick_y, 3),
                    "place_section": None,
                    "place_x": None,
                    "place_y": None,
                    "transport_direction": "pick_only",
                    "pick_only": True,
                    "success": True,
                    "events": list(self._events),
                })
                self.finished.emit(True)
                return

            # 5) Place 위치로 이동 — 공통 RPY로 XY만 이동 (꺾임 방지)
            self._log(f"5) Moving to {place_section} section Place position (travel RPY)...")
            if not self._move(place_x, place_y, z_move2, velocity=24.0, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ):
                self._log("Failed: move to place position")
                self._release_at_safe_z(place_x, place_y, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ)
                self._fail_and_go_home()
                return

            # 6) Place 하강: 130까지 공통 RPY, 130→101.7 은 (x,y)별 보정 RPY
            self._log(f"6) Descending to Z={DESCENT_MID_Z:.1f} for Place (travel RPY)...")
            if not self._move(place_x, place_y, DESCENT_MID_Z, velocity=18.0, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ):
                self._log(f"Failed: descend to Z={DESCENT_MID_Z:.1f} for place → 현재 위치에서 그리퍼 해제 시도")
                if self.gripper:
                    self.gripper.release(wait_time=0.2)
                self._fail_and_go_home()
                return

            place_drx, place_dry, place_drz = get_descent_rpy(place_x, place_y)
            self._log(
                f"6.1) Descending to Z={RELEASE_Z:.1f} for Place (descent RPY: "
                f"RX={place_drx:.2f} RY={place_dry:.2f} RZ={place_drz:.2f})..."
            )
            if not self._move(place_x, place_y, RELEASE_Z, velocity=DESCENT_VELOCITY, rx=place_drx, ry=place_dry, rz=place_drz):
                self._log(f"Failed: descend to Z={RELEASE_Z:.1f} for place → 현재 위치에서 그리퍼 해제 후 복귀")
                if self.gripper:
                    self.gripper.release(wait_time=0.2)
                self._fail_and_go_home()
                return

            # 101.7 도달할 때까지 그리퍼 OFF 하지 않음 — 도달 확인 후에만 해제
            # 피드백 갱신 대기 후 좁은 허용치로 1차 확인, 실패 시 ±8mm 넓은 허용치로 재확인
            time.sleep(0.2)
            if not self._wait_until_z_reached(RELEASE_Z, tolerance_mm=RELEASE_Z_TOLERANCE_MM, timeout_s=8.0):
                pose_now = self.robot.get_current_pose_from_feedback()
                z_now = float(pose_now[2]) if pose_now and len(pose_now) >= 3 else None
                if z_now is not None and abs(z_now - RELEASE_Z) <= 8.0:
                    self._log(f"Place Z: 1차 확인 실패, 현재 Z={z_now:.2f}mm (±8mm 이내) → 진행")
                else:
                    self._log(f"Place: Z=101 도달 확인 실패 (Z={z_now}) → 그리퍼 해제 후 복귀")
                    if self.gripper:
                        self.gripper.release(wait_time=0.2)
                    self._fail_and_go_home()
                    return
            if self._stop_requested:
                self._fail_and_go_home()
                return

            # Z=101.7 도달 확인 후에만 그리퍼 해제
            self._log(f"6.2) Z=101 도달 확인됨. {PLACE_WAIT_AT_101_S}초 대기 후 Gripper OFF...")
            for _ in range(int(PLACE_WAIT_AT_101_S * 10)):
                if self._stop_requested:
                    if self.gripper:
                        self.gripper.release(wait_time=0.1)
                    self._fail_and_go_home()
                    return
                time.sleep(0.1)

            place_hold = random.uniform(PLACE_HOLD_TIME_LO, PLACE_HOLD_TIME_HI)
            self._log(f"6.2) Gripper OFF at Place (Z=101), holding {place_hold:.2f}s...")
            self._events.append(("gripper_off", time.time()))
            if self.gripper:
                self.gripper.release(wait_time=place_hold)
            if self._stop_requested:
                self._fail_and_go_home()
                return

            # 7) Place 후: Z=130 → 랜덤 Z → Z=200 까지 공통 RPY로 올림 (꺾임 방지)
            self._log(f"7) Lifting to Z={DESCENT_MID_Z:.1f} (travel RPY)...")
            if not self._move(place_x, place_y, DESCENT_MID_Z, velocity=18.0, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ):
                self._log("Failed: lift to Z=130 after place")
                self._fail_and_go_home()
                return
            z_after_place = random.uniform(Z_MOVE_MIN, Z_MOVE_MAX)
            self._log(f"7.1) Lifting to Z={z_after_place:.1f} (travel RPY)...")
            if not self._move(place_x, place_y, z_after_place, velocity=24.0, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ):
                self._log("Failed: lift to random Z after place")
                self._fail_and_go_home()
                return
            self._log("7.2) Lifting to Z=200 (travel RPY)...")
            if not self._move(place_x, place_y, Z_AFTER_RELEASE, velocity=30.0, rx=TRAVEL_RX, ry=TRAVEL_RY, rz=TRAVEL_RZ):
                self._log("Failed: lift to Z=200")
                self._fail_and_go_home()
                return

            # 8) 복귀 전 현재 위치에서 INIT RPY로 맞춘 뒤 초기 위치로 복귀 (꺾임 방지)
            self._log("8) Aligning to INIT RPY at current XY...")
            if not self._move(place_x, place_y, Z_AFTER_RELEASE, velocity=20.0, rx=INIT_RX, ry=INIT_RY, rz=INIT_RZ):
                self._log("Failed: align to INIT RPY")
                self._fail_and_go_home()
                return
            self._log("8.1) Returning to initial position...")
            if not self._move(INIT_X, INIT_Y, INIT_Z, rx=INIT_RX, ry=INIT_RY, rz=INIT_RZ):
                self._log("Failed: return to initial")
                self._fail_and_go_home()
                return

            self._log(f"Step complete ({self.pick_section} section Pick → {place_section} section Place).")
            self.episode_vacuum_durations.emit(pick_hold, place_hold)
            self.episode_meta_ready.emit({
                "task_name": "pick_and_place",
                "object_label": "red_block",
                "pick_section": self.pick_section,
                "pick_x": round(pick_x, 3),
                "pick_y": round(pick_y, 3),
                "place_section": place_section,
                "place_x": round(place_x, 3),
                "place_y": round(place_y, 3),
                "transport_direction": _transport_direction(place_x, place_y),
                "success": True,
                "events": list(self._events),
            })
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
        self.side_camera = None
        self.side_camera_thread = None
        self.side_camera_active = False
        self.step_worker = None
        self.auto_collect_target = 0   # 0=수동, N=N개 수집 중
        self.auto_collect_done = 0
        self.auto_collect_mode = "manual"  # manual | legacy_count | smolvla_5x10_strict
        self.auto_collect_attempt_total = 0
        # Stop(중단) 버튼으로 에피소드 중단 시: 진행 중이던 "현재 폴더"는 저장하지 않기 위한 플래그
        self.abort_current_episode = False
        # STOP 후 재시도를 위한 일시정지 상태 (카운트·위치 유지)
        self._auto_paused = False
        # 위치 기억: 마지막으로 Place한 위치 (다음 스텝의 Pick 위치)
        self.last_place_x = None
        self.last_place_y = None
        self.current_pick_section = "A"  # 현재 Pick할 섹션 ("A" or "B")
        self.current_start_pose_id = None
        self._smolvla_rr_index = 0
        self.pose_success_counts = {pose_id: 0 for pose_id in SMOLVLA_START_POSE_ORDER}
        self.pose_attempt_counts = {pose_id: 0 for pose_id in SMOLVLA_START_POSE_ORDER}
        self.side_camera_id = 1
        # 기록
        self.vla_dataset_base = VLA_DATASET_BASE
        try:
            os.makedirs(self.vla_dataset_base, exist_ok=True)
        except Exception:
            pass
        self.recorded_data = []
        self.recording = False
        self.record_save_dir = None
        self.record_frame_count = 0
        self._pending_episode_meta = {}
        self.record_timer = QTimer(self)
        self.record_timer.timeout.connect(self._on_record_tick)
        # Vacuum 명령 기반 로그 (명령 ON 유지 시간, s)
        self.episode_vacuum_pick_hold = self.episode_vacuum_place_hold = 0.0
        self.init_ui()
        self.log(f"Dataset root ready: {self.vla_dataset_base}")
        if CAMERA_AVAILABLE:
            self._init_camera()
        if SIDE_CAMERA_AVAILABLE:
            self._init_side_camera()

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

    def _get_side_camera_id_from_ui(self):
        if not hasattr(self, "side_camera_id_input"):
            return self.side_camera_id
        try:
            camera_id = int(self.side_camera_id_input.text().strip())
        except Exception:
            camera_id = self.side_camera_id
        return max(0, camera_id)

    def _init_side_camera(self):
        try:
            camera_id = self._get_side_camera_id_from_ui()
            self.side_camera_id = camera_id
            self.side_camera = SideUsbCamera(camera_id=camera_id)
            if self.side_camera.init_camera():
                self.log(f"OBS_IMAGE_2 initialized (USB camera_id={camera_id})")
            else:
                self.side_camera = None
                self.log("OBS_IMAGE_2 init failed")
        except Exception as e:
            self.log(f"OBS_IMAGE_2 init error: {e}")
            self.side_camera = None

    def _apply_side_camera_id(self):
        camera_id = self._get_side_camera_id_from_ui()
        self.side_camera_id = camera_id
        if self.side_camera_active:
            self.stop_side_camera()
        if self.side_camera:
            try:
                self.side_camera.cleanup()
            except Exception:
                pass
            self.side_camera = None
        self._init_side_camera()
        self.log(f"OBS_IMAGE_2 camera_id set to {camera_id}")

    def init_ui(self):
        self.setWindowTitle("Dobot E6 Pick & Place (New) - SmolVLA Dual Camera Collector")
        self.setGeometry(100, 100, 1500, 760)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)

        # Left: Dual camera
        left_layout = QVBoxLayout()
        cam_view_layout = QHBoxLayout()

        self.camera_label = QLabel("OBS_IMAGE_1 (Start to view)")
        self.camera_label.setMinimumSize(520, 380)
        self.camera_label.setMaximumSize(520, 380)
        self.camera_label.setStyleSheet("border: 2px solid gray; background-color: black; color: white;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        cam_view_layout.addWidget(self.camera_label)

        self.side_camera_label = QLabel("OBS_IMAGE_2 (Start to view)")
        self.side_camera_label.setMinimumSize(520, 380)
        self.side_camera_label.setMaximumSize(520, 380)
        self.side_camera_label.setStyleSheet("border: 2px solid gray; background-color: black; color: white;")
        self.side_camera_label.setAlignment(Qt.AlignCenter)
        cam_view_layout.addWidget(self.side_camera_label)
        left_layout.addLayout(cam_view_layout)

        cam_btn_layout = QHBoxLayout()
        self.start_camera_btn = QPushButton("Start OBS_IMAGE_1")
        self.start_camera_btn.clicked.connect(self.start_camera)
        self.stop_camera_btn = QPushButton("Stop OBS_IMAGE_1")
        self.stop_camera_btn.clicked.connect(self.stop_camera)
        self.stop_camera_btn.setEnabled(False)
        cam_btn_layout.addWidget(self.start_camera_btn)
        cam_btn_layout.addWidget(self.stop_camera_btn)

        self.start_side_btn = QPushButton("Start OBS_IMAGE_2")
        self.start_side_btn.clicked.connect(self.start_side_camera)
        self.stop_side_btn = QPushButton("Stop OBS_IMAGE_2")
        self.stop_side_btn.clicked.connect(self.stop_side_camera)
        self.stop_side_btn.setEnabled(False)
        cam_btn_layout.addWidget(self.start_side_btn)
        cam_btn_layout.addWidget(self.stop_side_btn)
        cam_btn_layout.addWidget(QLabel("OBS_IMAGE_2 camera_id:"))
        self.side_camera_id_input = QLineEdit("1")
        self.side_camera_id_input.setMaximumWidth(50)
        cam_btn_layout.addWidget(self.side_camera_id_input)
        self.apply_side_camera_id_btn = QPushButton("Apply")
        self.apply_side_camera_id_btn.clicked.connect(self._apply_side_camera_id)
        cam_btn_layout.addWidget(self.apply_side_camera_id_btn)
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

        self.log("GUI initialized (Pose + OBS_IMAGE_1/2 + Pick-Place Step)")

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
        self.record_20hz_cb = QCheckBox("20Hz 기록 (vla_dataset_smolvla/1,2,3...)")
        self.record_20hz_cb.setToolTip("체크 시 초기자세 도달 후 20Hz 기록 시작. OBS_IMAGE_1/OBS_IMAGE_2를 동시에 저장합니다.")
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
        self.resume_auto_btn = QPushButton("Resume (같은위치 재시도)")
        self.resume_auto_btn.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold;")
        self.resume_auto_btn.clicked.connect(self.resume_auto_collect)
        self.resume_auto_btn.setVisible(False)
        row2.addWidget(self.resume_auto_btn)
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
        self.smolvla_progress_label = QLabel("")
        self.smolvla_progress_label.setWordWrap(True)
        self.smolvla_progress_label.setStyleSheet("color: #0d47a1;")
        layout.addWidget(self.smolvla_progress_label)
        self._update_smolvla_progress_ui()
        group.setLayout(layout)
        return group

    def _update_smolvla_progress_ui(self):
        total_success = sum(self.pose_success_counts.values())
        total_attempt = sum(self.pose_attempt_counts.values())
        lines = [
            f"SmolVLA 5x10 progress: success {total_success}/{SMOLVLA_STRICT_TOTAL_TARGET}, attempts {total_attempt}",
        ]
        for pose_id in SMOLVLA_START_POSE_ORDER:
            s = self.pose_success_counts.get(pose_id, 0)
            a = self.pose_attempt_counts.get(pose_id, 0)
            lines.append(f"- {pose_id}: {s}/{SMOLVLA_SUCCESS_TARGET_PER_POSE} success (attempts {a})")
        self.smolvla_progress_label.setText("\n".join(lines))

    def _reset_smolvla_progress(self):
        self.pose_success_counts = {pose_id: 0 for pose_id in SMOLVLA_START_POSE_ORDER}
        self.pose_attempt_counts = {pose_id: 0 for pose_id in SMOLVLA_START_POSE_ORDER}
        self.auto_collect_done = 0
        self.auto_collect_attempt_total = 0
        self.current_start_pose_id = None
        self._smolvla_rr_index = 0
        self._update_smolvla_progress_ui()

    def _choose_next_start_pose(self):
        candidates = [
            pose_id
            for pose_id in SMOLVLA_START_POSE_ORDER
            if self.pose_success_counts.get(pose_id, 0) < SMOLVLA_SUCCESS_TARGET_PER_POSE
        ]
        if not candidates:
            return None
        n = len(SMOLVLA_START_POSE_ORDER)
        for offset in range(n):
            idx = (self._smolvla_rr_index + offset) % n
            pose_id = SMOLVLA_START_POSE_ORDER[idx]
            if pose_id in candidates:
                self._smolvla_rr_index = (idx + 1) % n
                return pose_id
        return candidates[0]

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
        self.auto_collect_mode = "manual"
        self.current_start_pose_id = "manual_default"
        do_record = self.record_20hz_cb.isChecked()
        if do_record:
            self.record_20hz_cb.setEnabled(False)
            self._ensure_camera_for_recording()
        self.step_worker = PickPlaceStepWorker(
            self.robot, self.gripper,
            pick_section=self.current_pick_section,
            pick_x=None, pick_y=None,
            camera=self.camera
        )
        self.step_worker.log_signal.connect(self.log)
        self.step_worker.finished.connect(self.on_pick_place_step_finished)
        self.step_worker.episode_vacuum_durations.connect(self._on_episode_vacuum_durations)
        self.step_worker.episode_meta_ready.connect(self._on_episode_meta_ready)
        if do_record:
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
        self.auto_collect_target = n
        self.auto_collect_done = 0
        self.auto_collect_attempt_total = 0
        self.current_start_pose_id = None
        self.current_pick_section = "A"
        self.last_place_x = None
        self.last_place_y = None
        self._begin_step_ui()
        self.record_20hz_cb.setChecked(True)
        self.record_20hz_cb.setEnabled(False)
        if n >= SMOLVLA_STRICT_TOTAL_TARGET:
            self.auto_collect_mode = "smolvla_5x10_strict"
            self.auto_collect_target = SMOLVLA_STRICT_TOTAL_TARGET
            self._reset_smolvla_progress()
            self.log("자동 수집 시작: SmolVLA strict policy (5 start poses x 10 success each)")
        else:
            self.auto_collect_mode = "legacy_count"
            self.log(f"자동 수집 시작: {n}개 (legacy count)")
        self._ensure_camera_for_recording()
        self._run_next_auto_step()

    def stop_auto_collect(self):
        """자동 수집 일시 중단 (현재 에피소드 저장 안 함, 카운트·위치 유지).
        블록 위치 조정 후 Resume 버튼으로 같은 pick 좌표에서 재시도 가능.
        완전 종료는 Resume 없이 새로 Auto 버튼 누르면 됨."""
        self.abort_current_episode = True
        self._auto_paused = True  # 카운트·위치 유지 (target은 건드리지 않음)
        if self.step_worker and self.step_worker.isRunning():
            self.step_worker.request_stop()
        self.log("자동 수집 일시 중단 — 블록 위치 조정 후 [Resume] 버튼을 누르세요.")

    def resume_auto_collect(self):
        """STOP 후 블록 위치 조정 완료 → 같은 pick 좌표에서 에피소드 재시도."""
        if not self.robot or not self.robot.connected or not self.gripper:
            self.log("Connect robot first")
            return
        if self.step_worker and self.step_worker.isRunning():
            self.log("Still running")
            return
        self._auto_paused = False
        self.resume_auto_btn.setVisible(False)
        if self.auto_collect_mode == "smolvla_5x10_strict":
            remaining = SMOLVLA_STRICT_TOTAL_TARGET - sum(self.pose_success_counts.values())
        else:
            remaining = self.auto_collect_target - self.auto_collect_done
        self.log(f"Resume: 남은 {remaining}개 수집 재개 (pick 위치 유지: "
                 f"x={self.last_place_x}, y={self.last_place_y})")
        self._begin_step_ui()
        self._ensure_camera_for_recording()
        self._run_next_auto_step()

    def _begin_step_ui(self):
        """스텝 시작 시 UI 비활성화"""
        self.pick_place_step_btn.setEnabled(False)
        self.move_pose_btn.setEnabled(False)
        self.home_btn.setEnabled(False)
        self.grip_btn.setEnabled(False)
        self.release_btn.setEnabled(False)
        for n in (1, 5, 10, 50):
            getattr(self, f"auto_collect_{n}_btn").setEnabled(False)
        self.stop_auto_btn.setEnabled(True)
        self.resume_auto_btn.setVisible(False)

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
        self.stop_auto_btn.setEnabled(False)
        self.record_20hz_cb.setEnabled(True)

    def on_pick_place_step_finished(self, success):
        """한 스텝 종료 후 저장/카운트 처리."""
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
                # Stop(중단) 버튼으로 종료된 경우: 해당 폴더는 저장하지 않음(자동 삭제)
                save_ok = bool(success) and (not self.abort_current_episode)
                if self.abort_current_episode:
                    self.log("중단됨: 현재 에피소드 폴더는 저장하지 않습니다.")
                self._stop_20hz_recording_and_save(save_ok)
            except Exception as e:
                self.log(f"20Hz 저장 중 오류: {e}")
                self.recording = False
                self.record_timer.stop()
                self.recorded_data.clear()
                self.record_save_dir = None
        # 이번 스텝 종료 시 중단 플래그는 리셋 (다음 수동/자동 스텝에 영향 없게)
        was_aborted = self.abort_current_episode
        self.abort_current_episode = False

        if self.auto_collect_target > 0:
            # STOP으로 중단된 경우: 카운트 증가 없이 Resume 버튼 표시
            if was_aborted and self._auto_paused:
                self._end_step_ui(auto_collect_active=False)
                self.resume_auto_btn.setVisible(True)
                if self.auto_collect_mode == "smolvla_5x10_strict":
                    remaining = SMOLVLA_STRICT_TOTAL_TARGET - sum(self.pose_success_counts.values())
                else:
                    remaining = self.auto_collect_target - self.auto_collect_done
                self.log(f"일시 중단됨: {self.auto_collect_done}/{self.auto_collect_target} 완료. "
                         f"블록 조정 후 [Resume] 누르세요. (남은 {remaining}개)")
                return
            self.auto_collect_attempt_total += 1
            if self.current_start_pose_id in self.pose_attempt_counts:
                self.pose_attempt_counts[self.current_start_pose_id] += 1
            if success:
                self.auto_collect_done += 1
                if self.auto_collect_mode == "smolvla_5x10_strict" and self.current_start_pose_id in self.pose_success_counts:
                    self.pose_success_counts[self.current_start_pose_id] += 1
            else:
                self.log("이번 에피소드 실패로 판정되어 success 카운트는 증가하지 않습니다.")
            self._update_smolvla_progress_ui()
            if self.auto_collect_mode == "smolvla_5x10_strict":
                total_success = sum(self.pose_success_counts.values())
                self.log(f"자동 수집(strict): success {total_success}/{SMOLVLA_STRICT_TOTAL_TARGET}")
                done_target = total_success >= SMOLVLA_STRICT_TOTAL_TARGET
            else:
                self.log(f"자동 수집(legacy): success {self.auto_collect_done}/{self.auto_collect_target}")
                done_target = self.auto_collect_done >= self.auto_collect_target
            if done_target:
                self._end_auto_collect()
                return
            self.log("다음 스텝 자동 시작...")
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
        self._auto_paused = False
        self.auto_collect_mode = "manual"
        self.current_start_pose_id = None
        self.episode_next_btn.setVisible(False)
        self.resume_auto_btn.setVisible(False)
        self._end_step_ui(auto_collect_active=False)
        self.update_status()
        self.log("자동 수집 완료.")

    def _run_next_auto_step(self):
        """다음 자동 수집 스텝 실행 (위치 기억 및 섹션 교대)"""
        if self.auto_collect_target <= 0:
            return
        if self.auto_collect_mode != "smolvla_5x10_strict" and self.auto_collect_done >= self.auto_collect_target:
            self._end_auto_collect()
            return
        if self.auto_collect_mode == "smolvla_5x10_strict" and sum(self.pose_success_counts.values()) >= SMOLVLA_STRICT_TOTAL_TARGET:
            self._end_auto_collect()
            return
        if not self.robot or not self.robot.connected or not self.gripper:
            self._end_auto_collect()
            return

        pick_x = self.last_place_x if self.last_place_x is not None else None
        pick_y = self.last_place_y if self.last_place_y is not None else None
        pick_section = self.current_pick_section
        if self.auto_collect_mode == "smolvla_5x10_strict":
            next_pose_id = self._choose_next_start_pose()
            if next_pose_id is None:
                self._end_auto_collect()
                return
            pose_spec = SMOLVLA_START_POSE_MAP[next_pose_id]
            pick_section = pose_spec["section"]
            pick_x, pick_y = pose_spec["xy"]
            self.current_start_pose_id = next_pose_id
            self.log(f"다음 시작 위치: {next_pose_id} ({pick_section}, x={pick_x:.2f}, y={pick_y:.2f})")
        else:
            self.current_start_pose_id = "legacy"

        self.step_worker = PickPlaceStepWorker(
            self.robot, self.gripper,
            pick_section=pick_section,
            pick_x=pick_x,
            pick_y=pick_y,
            camera=self.camera
        )
        self.step_worker.log_signal.connect(self.log)
        self.step_worker.finished.connect(self.on_pick_place_step_finished)
        self.step_worker.episode_vacuum_durations.connect(self._on_episode_vacuum_durations)
        self.step_worker.episode_meta_ready.connect(self._on_episode_meta_ready)
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
        """vla_dataset_base 안에서 다음 폴더 번호(1, 2, 3, ...) 찾기."""
        if not os.path.exists(self.vla_dataset_base):
            return 1
        existing = [d for d in os.listdir(self.vla_dataset_base) if os.path.isdir(os.path.join(self.vla_dataset_base, d)) and d.isdigit()]
        if not existing:
            return 1
        max_num = max([int(d) for d in existing])
        return max_num + 1

    def _ensure_camera_for_recording(self):
        """20Hz 기록 전 2카메라 준비: OBS_IMAGE_1/OBS_IMAGE_2 자동 시작 시도."""
        if CAMERA_AVAILABLE:
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
                    self.log("OBS_IMAGE_1 auto-started for recording")
                except Exception as e:
                    self.log(f"OBS_IMAGE_1 auto-start failed: {e}")

        if SIDE_CAMERA_AVAILABLE:
            if self.side_camera is None:
                self._init_side_camera()
            if self.side_camera and self.side_camera.initialized and not self.side_camera_active:
                try:
                    self.side_camera_thread = SideCameraThread(self.side_camera)
                    self.side_camera_thread.frame_ready.connect(self._update_side_camera_frame)
                    self.side_camera_thread.start()
                    self.side_camera_active = True
                    self.start_side_btn.setEnabled(False)
                    self.stop_side_btn.setEnabled(True)
                    self.log("OBS_IMAGE_2 auto-started for recording")
                except Exception as e:
                    self.log(f"OBS_IMAGE_2 auto-start failed: {e}")

    def _on_episode_vacuum_durations(self, pick_hold, place_hold):
        self.episode_vacuum_pick_hold = float(pick_hold)
        self.episode_vacuum_place_hold = float(place_hold)

    def _on_episode_meta_ready(self, meta):
        self._pending_episode_meta = meta

    def _start_20hz_recording(self):
        """20Hz 기록 시작: vla_dataset/(다음 번호) 생성, 타이머 시작. (초기 자세 도달 후 호출됨)"""
        folder_num = self._get_next_folder_number()
        self.record_save_dir = os.path.join(self.vla_dataset_base, str(folder_num))
        print(f"[DEBUG] Starting 20Hz recording: save_dir={self.record_save_dir} (folder #{folder_num})")
        os.makedirs(self.record_save_dir, exist_ok=True)
        os.makedirs(os.path.join(self.record_save_dir, "images", "OBS_IMAGE_1"), exist_ok=True)
        os.makedirs(os.path.join(self.record_save_dir, "images", "OBS_IMAGE_2"), exist_ok=True)
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

            obs1_path = os.path.join(self.record_save_dir, "images", "OBS_IMAGE_1", frame_filename)
            obs2_path = os.path.join(self.record_save_dir, "images", "OBS_IMAGE_2", frame_filename)

            hik_frame_bgr = None
            if self.camera and self.camera.initialized:
                ret_hik, frame_hik = self.camera.get_frame()
                if ret_hik and frame_hik is not None:
                    hik_frame_bgr = cv2.cvtColor(frame_hik, cv2.COLOR_RGB2BGR)
            if hik_frame_bgr is None:
                hik_save = np.zeros((224, 224, 3), dtype=np.uint8)
            else:
                hik_320 = cv2.resize(hik_frame_bgr, (320, 240))
                hik_save = hik_320[16:240, 55:279]
            cv2.imwrite(obs1_path, hik_save)

            side_frame_bgr = None
            if self.side_camera and self.side_camera.initialized:
                ret_side, frame_side = self.side_camera.get_frame()
                if ret_side and frame_side is not None:
                    side_frame_bgr = cv2.cvtColor(frame_side, cv2.COLOR_RGB2BGR)
            if side_frame_bgr is None:
                side_save = np.zeros((224, 224, 3), dtype=np.uint8)
            else:
                side_crop = side_frame_bgr[120:480, 150:510]
                side_save = cv2.resize(side_crop, (224, 224))
            cv2.imwrite(obs2_path, side_save)

            record = {
                'frame_id': self.record_frame_count,
                'timestamp': time.time(),
                'image_path_OBS_IMAGE_1': f"OBS_IMAGE_1/{frame_filename}",
                'image_path_OBS_IMAGE_2': f"OBS_IMAGE_2/{frame_filename}",
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
        if not step_success:
            self.log("Step failed → episode not saved (only successful episodes go into dataset).")
            if self.record_save_dir and os.path.isdir(self.record_save_dir):
                try:
                    shutil.rmtree(self.record_save_dir)
                    self.log(f"Removed partial folder: {self.record_save_dir}")
                except Exception as e:
                    self.log(f"Could not remove partial folder: {e}")
            self.recorded_data.clear()
            self.record_save_dir = None
            return
        if not self.record_save_dir or len(self.recorded_data) == 0:
            self.log(f"20Hz: no data to save. (save_dir={self.record_save_dir}, data_count={len(self.recorded_data)})")
            print(f"[DEBUG] No data: save_dir={self.record_save_dir}, recorded_data count={len(self.recorded_data)}")
            return

        # 초기 자세 복귀 검증:
        # - 기본 vla_dataset: 마지막 프레임이 INIT 근처가 아니면 저장 안 함.
        # - vla_dataset_random_pose: 충돌 등 중간 이슈가 있어도 프로세스가 끝까지
        #   진행되었다면(== step_success=True) 에피소드를 남기기 위해 이 검증을 건너뛴다.
        skip_init_check = False
        try:
            base_name = os.path.basename(self.vla_dataset_base)
            if base_name == "vla_dataset_random_pose":
                skip_init_check = True
        except Exception:
            pass

        if not skip_init_check:
            last = self.recorded_data[-1]
            tcp = last.get("tcp_pose")
            if not tcp or len(tcp) < 6:
                self.log("20Hz: last frame has no valid tcp_pose → episode not saved (no final pose).")
                if self.record_save_dir and os.path.isdir(self.record_save_dir):
                    try:
                        shutil.rmtree(self.record_save_dir)
                        self.log(f"Removed partial folder: {self.record_save_dir}")
                    except Exception as e:
                        self.log(f"Could not remove partial folder: {e}")
                self.recorded_data.clear()
                self.record_save_dir = None
                return
            dx = abs(float(tcp[0]) - INIT_X)
            dy = abs(float(tcp[1]) - INIT_Y)
            dz = abs(float(tcp[2]) - INIT_Z)
            drx = abs(float(tcp[3]) - INIT_RX)
            dry = abs(float(tcp[4]) - INIT_RY)
            drz = abs(float(tcp[5]) - INIT_RZ)
            if (
                dx > INIT_POSE_TOLERANCE_MM
                or dy > INIT_POSE_TOLERANCE_MM
                or dz > INIT_POSE_TOLERANCE_MM
                or drx > INIT_POSE_TOLERANCE_DEG
                or dry > INIT_POSE_TOLERANCE_DEG
                or drz > INIT_POSE_TOLERANCE_DEG
            ):
                self.log(
                    f"20Hz: last frame not at initial (Δx={dx:.1f} Δy={dy:.1f} Δz={dz:.1f} mm) → episode not saved."
                )
                if self.record_save_dir and os.path.isdir(self.record_save_dir):
                    try:
                        shutil.rmtree(self.record_save_dir)
                        self.log(f"Removed partial folder: {self.record_save_dir}")
                    except Exception as e:
                        self.log(f"Could not remove partial folder: {e}")
                self.recorded_data.clear()
                self.record_save_dir = None
                return
        try:
            print(f"[DEBUG] Saving {len(self.recorded_data)} records to {self.record_save_dir}...")
            csv_path = os.path.join(self.record_save_dir, "robot_data.csv")
            print(f"[DEBUG] Writing CSV: {csv_path}")
            with open(csv_path, 'w', newline='') as f:
                f.write("frame_id,timestamp,image_path_OBS_IMAGE_1,image_path_OBS_IMAGE_2,")
                f.write("j1,j2,j3,j4,j5,j6,")
                f.write("x,y,z,rx,ry,rz,")
                f.write("gripper_tooldo1,gripper_tooldo2,robot_mode\n")
                for r in self.recorded_data:
                    f.write(f"{r['frame_id']},{r['timestamp']},{r['image_path_OBS_IMAGE_1']},{r['image_path_OBS_IMAGE_2']},")
                    f.write(','.join(map(str, r['joint_angles'])) + ',')
                    f.write(','.join(map(str, r['tcp_pose'])) + ',')
                    f.write(f"{r['gripper_tooldo1']},{r['gripper_tooldo2']},{r['robot_mode']}\n")
            print(f"[DEBUG] CSV written: {csv_path}")
            npy_path = os.path.join(self.record_save_dir, "dataset.npy")
            np.save(npy_path, self.recorded_data)

            # episode_meta.json
            import json as _json
            folder_num = os.path.basename(self.record_save_dir)
            n_frames = len(self.recorded_data)
            # 실측 fps: 타임스탬프 기반 계산 (QTimer 오버헤드 반영)
            if n_frames >= 2:
                actual_fps = round((n_frames - 1) / (self.recorded_data[-1]['timestamp'] - self.recorded_data[0]['timestamp']), 3)
            else:
                actual_fps = RECORD_RATE_HZ
            meta = dict(self._pending_episode_meta)
            if self.auto_collect_mode == "smolvla_5x10_strict":
                pose_id = self.current_start_pose_id
                current_pose_success = self.pose_success_counts.get(pose_id, 0)
                if bool(step_success):
                    current_pose_success += 1
                global_success_after = sum(self.pose_success_counts.values()) + (1 if bool(step_success) else 0)
            else:
                pose_id = self.current_start_pose_id
                current_pose_success = None
                global_success_after = self.auto_collect_done + (1 if bool(step_success) else 0)
            meta.update({
                "folder": folder_num,
                "date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "total_frames": n_frames,
                "record_rate_hz": actual_fps,
                "cameras": "OBS_IMAGE_1+OBS_IMAGE_2",
                "camera_mapping": {
                    "OBS_IMAGE_1": "top",
                    "OBS_IMAGE_2": "side"
                },
                "camera_source": {
                    "OBS_IMAGE_1": "camera_1",
                    "OBS_IMAGE_2": "camera_2"
                },
                "sync_policy": SYNC_POLICY_STRING,
                "success": bool(step_success),
                "collection_policy": self.auto_collect_mode,
                "start_pose_id": pose_id,
                "global_success_count_after_episode": global_success_after,
                "per_pose_success_target": SMOLVLA_SUCCESS_TARGET_PER_POSE if self.auto_collect_mode == "smolvla_5x10_strict" else None,
                "start_pose_success_count_after_episode": current_pose_success,
                "vacuum_pick_duration_s": round(self.episode_vacuum_pick_hold, 3),
                "vacuum_place_duration_s": round(self.episode_vacuum_place_hold, 3),
            })
            meta.pop("events", None)  # events는 별도 CSV로 저장
            meta_json_path = os.path.join(self.record_save_dir, "episode_meta.json")
            with open(meta_json_path, 'w', encoding='utf-8') as f:
                _json.dump(meta, f, ensure_ascii=False, indent=2)

            # episode_events.csv (gripper_on/off → 가장 가까운 frame_id 매핑)
            events_raw = self._pending_episode_meta.get("events", [])
            if events_raw and self.recorded_data:
                ts_list = [(r['frame_id'], r['timestamp']) for r in self.recorded_data]
                events_path = os.path.join(self.record_save_dir, "episode_events.csv")
                with open(events_path, 'w', newline='', encoding='utf-8') as f:
                    f.write("event,frame_id,timestamp\n")
                    for ev_name, ev_ts in events_raw:
                        closest_fid = min(ts_list, key=lambda t: abs(t[1] - ev_ts))[0]
                        f.write(f"{ev_name},{closest_fid},{ev_ts:.6f}\n")

            # metadata.txt (호환성 유지)
            meta_path = os.path.join(self.record_save_dir, "metadata.txt")
            with open(meta_path, 'w') as f:
                f.write("VLA Dataset - Pick-Place Step\n")
                f.write("=" * 50 + "\n\n")
                f.write(f"Folder: {folder_num}\n")
                f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Total Frames: {n_frames}\n")
                f.write(f"Record Rate: {actual_fps}Hz\n")
                f.write("Cameras: OBS_IMAGE_1(top) + OBS_IMAGE_2(side)\n")
                f.write(f"Step Success: {step_success}\n")
                f.write(f"VacuumCommandPickDuration_s: {self.episode_vacuum_pick_hold:.3f}\n")
                f.write(f"VacuumCommandPlaceDuration_s: {self.episode_vacuum_place_hold:.3f}\n")
            self._pending_episode_meta = {}
            self.log(f"Saved: {n_frames} frames → {self.record_save_dir}")
        except Exception as e:
            self.log(f"20Hz save error: {e}")

    def start_camera(self):
        if not CAMERA_AVAILABLE:
            self.log("OBS_IMAGE_1 module not available")
            QMessageBox.warning(self, "Camera", "OBS_IMAGE_1 module not available.")
            return
        if self.camera is None:
            self._init_camera()
        if self.camera is None or not self.camera.initialized:
            self.log("OBS_IMAGE_1 not initialized")
            QMessageBox.warning(self, "Camera", "OBS_IMAGE_1 init failed.")
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
            self.log("OBS_IMAGE_1 started")
        except Exception as e:
            self.log(f"OBS_IMAGE_1 start error: {e}")

    def stop_camera(self):
        if self.camera_thread:
            self.camera_thread.stop()
            self.camera_thread.wait()
            self.camera_thread = None
        self.camera_active = False
        self.start_camera_btn.setEnabled(True)
        self.stop_camera_btn.setEnabled(False)
        self.camera_label.setText("OBS_IMAGE_1 (Start to view)")
        self.log("OBS_IMAGE_1 stopped")

    def start_side_camera(self):
        if not SIDE_CAMERA_AVAILABLE:
            self.log("OBS_IMAGE_2 module not available")
            QMessageBox.warning(self, "Camera", "OBS_IMAGE_2 module not available.")
            return
        if self.side_camera is None:
            self._init_side_camera()
        if self.side_camera is None or not self.side_camera.initialized:
            self.log("OBS_IMAGE_2 not initialized")
            QMessageBox.warning(self, "Camera", "OBS_IMAGE_2 init failed.")
            return
        if self.side_camera_active:
            return
        try:
            self.side_camera_thread = SideCameraThread(self.side_camera)
            self.side_camera_thread.frame_ready.connect(self._update_side_camera_frame)
            self.side_camera_thread.start()
            self.side_camera_active = True
            self.start_side_btn.setEnabled(False)
            self.stop_side_btn.setEnabled(True)
            self.log("OBS_IMAGE_2 started")
        except Exception as e:
            self.log(f"OBS_IMAGE_2 start error: {e}")

    def stop_side_camera(self):
        if self.side_camera_thread:
            self.side_camera_thread.stop()
            self.side_camera_thread.wait()
            self.side_camera_thread = None
        self.side_camera_active = False
        self.start_side_btn.setEnabled(True)
        self.stop_side_btn.setEnabled(False)
        self.side_camera_label.setText("OBS_IMAGE_2 (Start to view)")
        self.log("OBS_IMAGE_2 stopped")

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

    def _update_side_camera_frame(self, frame):
        try:
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            scaled = pixmap.scaled(self.side_camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.side_camera_label.setPixmap(scaled)
        except Exception as e:
            print(f"OBS_IMAGE_2 frame update error: {e}")

    def closeEvent(self, event):
        if self.camera_active:
            self.stop_camera()
        if self.side_camera_active:
            self.stop_side_camera()
        if self.camera:
            try:
                self.camera.cleanup()
            except Exception:
                pass
        if self.side_camera:
            try:
                self.side_camera.cleanup()
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
