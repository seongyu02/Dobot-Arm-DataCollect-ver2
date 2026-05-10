#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FastAPI web server for Dobot E6 Pick-Place data collection.
Dual camera: OBS_IMAGE_1(HIKRobot) + OBS_IMAGE_2(USB camera)
Dashboard: J1-J6, TCP pose, Robot Mode live display

Usage:
    cd /home/billye6/Dobot-Arm-DataCollect/Dobot_E6_Moveit2/src
    python3 robot_server.py

Open: http://<jetson-ip>:8000
"""

import sys
import os
import time
import threading
import asyncio
import shutil
from datetime import datetime
from typing import Dict, List, Optional, Set

# ═══════════════════════════════════════════════════════════════════════════
# PyQt5 Mock — PickPlaceStepWorker(QThread) → threading.Thread 교체
# (pick_place_gui_new import 전 반드시 먼저 선언)
# ═══════════════════════════════════════════════════════════════════════════
import types as _types

class _BoundSignal:
    def __init__(self):
        self._cbs = []
    def connect(self, cb):
        self._cbs.append(cb)
    def emit(self, *args):
        for cb in self._cbs:
            try:
                cb(*args)
            except Exception:
                pass
    def disconnect(self, cb=None):
        self._cbs = [] if cb is None else [c for c in self._cbs if c != cb]

class _SignalDescriptor:
    def __init__(self, *_):
        self._attr = None
    def __set_name__(self, owner, name):
        self._attr = f'_sig_{name}'
    def __get__(self, obj, objtype=None):
        if obj is None:
            return self
        attr = self._attr or '_sig_unknown'
        if not hasattr(obj, attr):
            object.__setattr__(obj, attr, _BoundSignal())
        return object.__getattribute__(obj, attr)

def _pyqtSignal(*a, **kw):
    return _SignalDescriptor()

class _QThread(threading.Thread):
    def __init__(self, parent=None):
        super().__init__(daemon=True)
    def start(self):
        super().start()
    def isRunning(self):
        return self.is_alive()
    def wait(self, msecs=None):
        self.join(timeout=(msecs / 1000.0) if msecs else None)

class _MockQt:
    def __init__(self, *a, **kw): pass
    def __call__(self, *a, **kw): return _MockQt()
    def __getattr__(self, name): return _MockQt()

_qt5     = _types.ModuleType('PyQt5')
_qw      = _types.ModuleType('PyQt5.QtWidgets')
_qc      = _types.ModuleType('PyQt5.QtCore')
_qg      = _types.ModuleType('PyQt5.QtGui')

for _n in ['QApplication','QMainWindow','QWidget','QVBoxLayout','QHBoxLayout',
           'QGroupBox','QGridLayout','QLabel','QLineEdit','QPushButton',
           'QTextEdit','QDoubleSpinBox','QMessageBox','QCheckBox']:
    setattr(_qw, _n, _MockQt)
_qc.QThread    = _QThread
_qc.pyqtSignal = _pyqtSignal
_qc.QTimer     = _MockQt
_qc.Qt         = _MockQt()
for _n in ['QFont', 'QImage', 'QPixmap']:
    setattr(_qg, _n, _MockQt)

sys.modules['PyQt5']             = _qt5
sys.modules['PyQt5.QtWidgets']   = _qw
sys.modules['PyQt5.QtCore']      = _qc
sys.modules['PyQt5.QtGui']       = _qg

# ═══════════════════════════════════════════════════════════════════════════
# 모듈 import
# ═══════════════════════════════════════════════════════════════════════════
import numpy as np
import cv2

_current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _current_dir)

if not os.environ.get('MVCAM_COMMON_RUNENV'):
    os.environ['MVCAM_COMMON_RUNENV'] = '/opt/MVS/lib'

import pick_place_gui_new as base
from pick_place_gui_random_pose import (
    RandomPosePickPlaceStepWorker,
    generate_random_initial_pose,
    INIT_SAFE_RX, INIT_SAFE_RY, INIT_SAFE_RZ,
)

# 데이터 저장 경로 — 외장 드라이브 마운트 확인 필요
DATA_SAVE_DIR   = "/media/billye6/새 볼륨/Dobot/2CAM"
DATA_DRIVE_ROOT = "/media/billye6/새 볼륨"   # 마운트 여부 판단 기준
from dobot_e6_controller import DobotE6Controller
from suction_gripper import SuctionGripper

_hik_available = False
try:
    from camera_viewer import HikRobotCamera
    _hik_available = True
except Exception as e:
    print(f"[Server] HIK camera unavailable: {e}")

# ═══════════════════════════════════════════════════════════════════════════
# OBS_IMAGE_2 USB 카메라 래퍼 (OpenCV VideoCapture)
# ═══════════════════════════════════════════════════════════════════════════
_obs2_available = True


class SideUsbCamera:
    """OBS_IMAGE_2: generic USB camera wrapper."""

    def __init__(self, camera_id: int = 1, width: int = 640, height: int = 480):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.cap = None
        self.initialized = False

    def init_camera(self) -> bool:
        self.cap = cv2.VideoCapture(self.camera_id)
        if self.cap is None or not self.cap.isOpened():
            print(f"[OBS_IMAGE_2] Open failed: camera_id={self.camera_id}")
            return False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.initialized = True
        print(f"[OBS_IMAGE_2] Camera initialized (camera_id={self.camera_id})")
        return True

    def get_frame(self):
        """(ok, RGB ndarray) 반환."""
        if not self.initialized:
            return False, None
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return False, None
        bgr = cv2.resize(frame, (640, 480))
        rgb  = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return True, rgb

    def cleanup(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        if self.initialized:
            self.initialized = False

# ═══════════════════════════════════════════════════════════════════════════
# FastAPI
# ═══════════════════════════════════════════════════════════════════════════
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse

# ═══════════════════════════════════════════════════════════════════════════
# ROS2 레코더 (rclpy 미설치 시 graceful 비활성화)
# ═══════════════════════════════════════════════════════════════════════════
import ros2_recorder as _ros2
_ros2_ok: bool = False          # start() 성공 후 True 로 설정

# ═══════════════════════════════════════════════════════════════════════════
# 서버 상태
# ═══════════════════════════════════════════════════════════════════════════
_state = {
    "robot":          None,
    "gripper":        None,
    "camera_hik":     None,
    "camera_zed":     None,
    "worker":         None,
    "recording":      False,
    "recorded_data":  [],
    "record_save_dir":None,
    "record_frame_count": 0,
    "vacuum_pick":    0.0,
    "vacuum_place":   0.0,
    "episode_meta":   {},
    "auto_target":    0,
    "auto_done":      0,
    "auto_mode":      "manual",  # manual | legacy_count | smolvla_5x10_strict
    "auto_attempt_total": 0,
    "pick_section":   "A",
    "last_place_x":   None,
    "last_place_y":   None,
    "current_start_pose_id": None,
    "pose_success_counts": {},
    "pose_attempt_counts": {},
    "pose_rr_index": 0,
    "side_camera_id": 1,
    "pick_only_mode": True,
    "auto_reset_enabled": True,
    "reset_mode": "strict_balanced",  # off | strict_balanced
    "reset_running": False,
    "reset_target_pose_id": None,
    "next_drop_pose_id": None,
    "drop_cycle_index": 0,
    "drop_success_counts": {},
    "pick_saved_count": 0,
    "auto_pending_complete": False,
    "active_step_kind": "pick",  # pick | reset
    "record_enabled_for_step": True,
}

_state_lock  = threading.Lock()
_ws_clients: Set[WebSocket] = set()
_log_queue: asyncio.Queue   = None
_main_loop: asyncio.AbstractEventLoop = None

FIXED_INIT = (89.3715, -378.5400, 250.0000, -179.5275, -2.4369, 2.3663)

SMOLVLA_SUCCESS_TARGET_PER_POSE = 10
SMOLVLA_STRICT_TOTAL_TARGET = 50
SMOLVLA_START_POSE_SPECS = [
    {"id": "pose_A_2", "section": "A", "xy": (base.POS_2[0], base.POS_2[1])},
    {"id": "pose_A_3", "section": "A", "xy": (base.POS_3[0], base.POS_3[1])},
    {"id": "pose_A_4", "section": "A", "xy": (base.POS_4[0], base.POS_4[1])},
    {"id": "pose_B_8", "section": "B", "xy": (base.POS_8[0], base.POS_8[1])},
    {"id": "pose_B_9", "section": "B", "xy": (base.POS_9[0], base.POS_9[1])},
]
SMOLVLA_START_POSE_MAP = {spec["id"]: spec for spec in SMOLVLA_START_POSE_SPECS}
SMOLVLA_START_POSE_ORDER = [spec["id"] for spec in SMOLVLA_START_POSE_SPECS]


def _new_pose_counter_map() -> Dict[str, int]:
    return {pose_id: 0 for pose_id in SMOLVLA_START_POSE_ORDER}


def _sum_pose_success() -> int:
    return sum(int(v) for v in _state.get("pose_success_counts", {}).values())


def _choose_next_start_pose_id() -> Optional[str]:
    candidates = [
        pose_id for pose_id in SMOLVLA_START_POSE_ORDER
        if _state["pose_success_counts"].get(pose_id, 0) < SMOLVLA_SUCCESS_TARGET_PER_POSE
    ]
    if not candidates:
        return None
    n = len(SMOLVLA_START_POSE_ORDER)
    start_idx = int(_state.get("pose_rr_index", 0)) % n
    for offset in range(n):
        idx = (start_idx + offset) % n
        pose_id = SMOLVLA_START_POSE_ORDER[idx]
        if pose_id in candidates:
            _state["pose_rr_index"] = (idx + 1) % n
            return pose_id
    return candidates[0]


_state["pose_success_counts"] = _new_pose_counter_map()
_state["pose_attempt_counts"] = _new_pose_counter_map()
_state["drop_success_counts"] = _new_pose_counter_map()


def _choose_next_drop_pose_id_strict_balanced() -> str:
    """drop 카운트 균형 + 라운드로빈으로 다음 drop pose 선택."""
    counts = _state.get("drop_success_counts", {})
    if not counts:
        _state["drop_success_counts"] = _new_pose_counter_map()
        counts = _state["drop_success_counts"]
    min_count = min(int(v) for v in counts.values())
    candidates = [pose_id for pose_id in SMOLVLA_START_POSE_ORDER if int(counts.get(pose_id, 0)) == min_count]
    n = len(SMOLVLA_START_POSE_ORDER)
    start_idx = int(_state.get("drop_cycle_index", 0)) % n
    for offset in range(n):
        idx = (start_idx + offset) % n
        pose_id = SMOLVLA_START_POSE_ORDER[idx]
        if pose_id in candidates:
            _state["drop_cycle_index"] = (idx + 1) % n
            return pose_id
    fallback = candidates[0] if candidates else SMOLVLA_START_POSE_ORDER[0]
    _state["drop_cycle_index"] = (SMOLVLA_START_POSE_ORDER.index(fallback) + 1) % n
    return fallback


def _choose_next_drop_pose_id() -> Optional[str]:
    mode = _state.get("reset_mode", "strict_balanced")
    if mode == "off" or not _state.get("auto_reset_enabled", True):
        return None
    return _choose_next_drop_pose_id_strict_balanced()

ROBOT_MODE_LABELS = {
    1:"INIT", 2:"BRAKE_OPEN", 4:"DISABLED", 5:"ENABLE",
    6:"BACKDRIVE", 7:"RUNNING", 8:"RECORDING", 9:"ERROR",
    10:"PAUSE", 11:"JOG"
}

# ─── 프레임 버퍼 (MJPEG + 레코딩 공용) ────────────────────────────────────
_buf_hik_jpg: Optional[bytes]       = None   # MJPEG용 JPEG 바이트
_buf_zed_jpg: Optional[bytes]       = None
_buf_hik_np:  Optional[np.ndarray]  = None   # 레코딩용 BGR numpy
_buf_zed_np:  Optional[np.ndarray]  = None
_buf_lock     = threading.Lock()

_cam_hik_thread: Optional[threading.Thread] = None
_cam_zed_thread: Optional[threading.Thread] = None
_cam_hik_running = False
_cam_zed_running = False

_robot_pub_running = False   # _robot_pub_loop 제어 플래그

# ═══════════════════════════════════════════════════════════════════════════
# 헬퍼
# ═══════════════════════════════════════════════════════════════════════════

def _log(msg: str):
    ts = datetime.now().strftime("%H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line)
    if _main_loop and _log_queue:
        try:
            _main_loop.call_soon_threadsafe(_log_queue.put_nowait, line)
        except Exception:
            pass

_SAFE_INIT_FALLBACK_XYZ = (89.3715, -378.5400, 250.0)  # 실측 검증된 안전 대기 위치

def _set_random_init_pose(robot):
    rx, ry, rz = INIT_SAFE_RX, INIT_SAFE_RY, INIT_SAFE_RZ
    cx, cy, cz = _SAFE_INIT_FALLBACK_XYZ
    ok = False
    for _ in range(30):
        tx, ty, tz, *_ = generate_random_initial_pose()
        if robot and robot.connected:
            ok, _ = robot.check_ik_solution(tx, ty, tz, rx, ry, rz)
        else:
            ok = True
        if ok:
            cx, cy, cz = tx, ty, tz
            break
    base.INIT_X = cx;  base.INIT_Y = cy;  base.INIT_Z = cz
    base.INIT_RX = rx; base.INIT_RY = ry; base.INIT_RZ = rz
    _log(f"[RandomPose] INIT X={cx:.1f} Y={cy:.1f} Z={cz:.1f} (IK={ok})")

def _get_next_folder(base_dir: str) -> int:
    if not os.path.exists(base_dir):
        return 1
    nums = [int(d) for d in os.listdir(base_dir)
            if os.path.isdir(os.path.join(base_dir, d)) and d.isdigit()]
    return max(nums, default=0) + 1

# ═══════════════════════════════════════════════════════════════════════════
# 카메라 그랩 루프 (MJPEG 버퍼 + 레코딩 numpy 버퍼 동시 갱신)
# ═══════════════════════════════════════════════════════════════════════════

def _hik_grab_loop():
    global _buf_hik_jpg, _buf_hik_np, _cam_hik_running
    cam = _state["camera_hik"]
    while _cam_hik_running and cam and cam.initialized:
        ret, frame = cam.get_frame()   # RGB
        if ret and frame is not None:
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            _, enc = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with _buf_lock:
                _buf_hik_jpg = enc.tobytes()
                _buf_hik_np  = bgr
            if _ros2_ok:
                _ros2.publish_hik(bgr)   # 캡처 직후 타임스탬프로 퍼블리시
        else:
            time.sleep(0.02)

def _zed_grab_loop():
    global _buf_zed_jpg, _buf_zed_np, _cam_zed_running
    cam = _state["camera_zed"]
    while _cam_zed_running and cam and cam.initialized:
        ret, frame = cam.get_frame()   # RGB
        if ret and frame is not None:
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            _, enc = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with _buf_lock:
                _buf_zed_jpg = enc.tobytes()
                _buf_zed_np  = bgr
            if _ros2_ok:
                _ros2.publish_zed(bgr)   # 캡처 직후 타임스탬프로 퍼블리시
        else:
            time.sleep(0.02)

def _robot_pub_loop():
    """로봇 상태를 ~50Hz 로 ROS2 에 퍼블리시. startup 에서 데몬 스레드로 시작."""
    global _robot_pub_running
    while _robot_pub_running:
        if _ros2_ok:
            robot   = _state["robot"]
            gripper = _state["gripper"]
            if robot and robot.connected:
                try:
                    feed = robot.feed.feedBackData()
                    if feed is not None and len(feed) > 0:
                        joints     = feed['QActual'][0].tolist()
                        tcp_pose   = feed['ToolVectorActual'][0].tolist()
                        robot_mode = int(feed['RobotMode'][0]) if 'RobotMode' in feed.dtype.names else 0
                        gripper_on = 1 if (gripper and gripper.is_gripping) else 0
                        _ros2.publish_robot(joints, tcp_pose, gripper_on, robot_mode)
                except Exception:
                    pass
        time.sleep(0.02)   # ~50 Hz


def _mjpeg_gen(buf_getter):
    """공통 MJPEG 제너레이터."""
    placeholder = None
    while True:
        with _buf_lock:
            frame = buf_getter()
        if frame:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        else:
            if placeholder is None:
                blank = np.full((240, 320, 3), 60, dtype=np.uint8)
                cv2.putText(blank, "No Camera", (55, 125),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (180, 180, 180), 2)
                _, enc = cv2.imencode('.jpg', blank)
                placeholder = enc.tobytes()
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + placeholder + b"\r\n"
        time.sleep(0.04)

# ═══════════════════════════════════════════════════════════════════════════
# 20Hz 레코딩 (threading 기반, QTimer 대체)
# ═══════════════════════════════════════════════════════════════════════════

def _start_recording():
    if _state["recording"]:
        return

    # ── 외장 드라이브 마운트 확인 ──────────────────────────────────────────
    if not os.path.isdir(DATA_DRIVE_ROOT):
        _log(f"[ERROR] External drive not mounted: {DATA_DRIVE_ROOT}")
        _log("[ERROR] Data collection aborted — please connect the drive and retry")
        # 진행 중인 worker 도 중단
        w = _state.get("worker")
        if w:
            w._stop_requested = True
        _state.update(auto_target=0, auto_done=0, auto_mode="manual")
        return
    # ────────────────────────────────────────────────────────────────────────

    n = _get_next_folder(DATA_SAVE_DIR)
    save_dir = os.path.join(DATA_SAVE_DIR, str(n))
    has_obs2 = bool(_state["camera_zed"] and _state["camera_zed"].initialized)
    try:
        os.makedirs(os.path.join(save_dir, "images", "OBS_IMAGE_1"), exist_ok=True)
        if has_obs2:
            os.makedirs(os.path.join(save_dir, "images", "OBS_IMAGE_2"), exist_ok=True)
    except OSError as e:
        _log(f"[ERROR] Cannot create save directory: {e}")
        _log("[ERROR] Data collection aborted — check drive permissions")
        w = _state.get("worker")
        if w:
            w._stop_requested = True
        _state.update(auto_target=0, auto_done=0, auto_mode="manual")
        return

    _state.update(recording=True, recorded_data=[], record_save_dir=save_dir,
                  record_frame_count=0)
    mode_str = "ROS2+sync" if (_ros2_ok and has_obs2) else "legacy"
    _log(f"Recording started → {save_dir} (OBS_IMAGE_2={'ON' if has_obs2 else 'OFF'}, mode={mode_str})")
    if _ros2_ok:
        _ros2.start_recording(save_dir)
    threading.Thread(target=_record_loop, daemon=True).start()

def _record_loop():
    while _state["recording"]:
        # ros2 + ZED 동시 활성 시: sync callback 이 저장 처리 → legacy tick 건너뜀
        has_obs2_now = bool(_state["camera_zed"] and _state["camera_zed"].initialized)
        if not (_ros2_ok and has_obs2_now):
            _record_tick()
        time.sleep(0.05)

def _record_tick():
    robot    = _state["robot"]
    gripper  = _state["gripper"]
    save_dir = _state["record_save_dir"]
    if not robot or not robot.connected or not save_dir:
        return
    try:
        feed = robot.feed.feedBackData()
        if feed is None or len(feed) == 0:
            return
        joints     = feed['QActual'][0].tolist()
        tcp_pose   = feed['ToolVectorActual'][0].tolist()
        robot_mode = int(feed['RobotMode'][0]) if 'RobotMode' in feed.dtype.names else 0
        gripper_on = 1 if (gripper and gripper.is_gripping) else 0
        fc = _state["record_frame_count"]
        fname = f"frame_{fc:06d}.jpg"

        # HIK 이미지 저장
        with _buf_lock:
            hik_np = _buf_hik_np.copy() if _buf_hik_np is not None else None
            zed_np = _buf_zed_np.copy() if _buf_zed_np is not None else None

        # HIK: 320×240 리사이즈 후 (x=60, y=16) 기준 224×224 크롭
        hik_path = os.path.join(save_dir, "images", "OBS_IMAGE_1", fname)
        if hik_np is not None:
            hik_320  = cv2.resize(hik_np, (320, 240))
            hik_save = hik_320[16:240, 55:279]        # y:16~240, x:55~279 → 224×224
        else:
            hik_save = np.zeros((224, 224, 3), dtype=np.uint8)
        cv2.imwrite(hik_path, hik_save)

        # ZED: 320×240 리사이즈
        has_obs2 = bool(_state["camera_zed"] and _state["camera_zed"].initialized)
        if has_obs2:
            zed_path = os.path.join(save_dir, "images", "OBS_IMAGE_2", fname)
            if zed_np is not None:
                zed_crop = zed_np[120:480, 150:510]         # (150,120) 시작 360×360 크롭
                zed_save = cv2.resize(zed_crop, (224, 224))
            else:
                zed_save = np.zeros((224, 224, 3), dtype=np.uint8)
            cv2.imwrite(zed_path, zed_save)

        record = {
            'frame_id':       fc,
            'timestamp':      time.time(),
            'image_path_OBS_IMAGE_1': f"OBS_IMAGE_1/{fname}",
            'image_path_OBS_IMAGE_2': f"OBS_IMAGE_2/{fname}" if has_obs2 else "",
            'joint_angles':   joints,
            'tcp_pose':       tcp_pose,
            'gripper_tooldo1':gripper_on,
            'gripper_tooldo2':0,
            'robot_mode':     robot_mode,
        }
        _state["recorded_data"].append(record)
        _state["record_frame_count"] += 1
    except Exception as e:
        print(f"[record_tick] {e}")

def _stop_and_save(success: bool):
    _state["recording"] = False
    time.sleep(0.07)
    save_dir = _state["record_save_dir"]

    # ros2 sync 데이터 우선 사용, 없으면 legacy fallback
    if _ros2_ok:
        ros2_data = _ros2.stop_recording()
        data = ros2_data if ros2_data else _state["recorded_data"]
    else:
        data = _state["recorded_data"]

    if not success:
        _log("Episode failed → not saved")
        if save_dir and os.path.isdir(save_dir):
            shutil.rmtree(save_dir, ignore_errors=True)
        _state.update(recorded_data=[], record_save_dir=None)
        return

    if not save_dir or not data:
        _log(f"No data recorded (dir={save_dir}, n={len(data) if data else 0})")
        return
    try:
        has_obs2 = bool(data[0].get('image_path_OBS_IMAGE_2'))
        # CSV
        with open(os.path.join(save_dir, "robot_data.csv"), 'w', newline='') as f:
            f.write("frame_id,timestamp,image_path_OBS_IMAGE_1")
            if has_obs2:
                f.write(",image_path_OBS_IMAGE_2")
            f.write(",j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz"
                    ",gripper_tooldo1,gripper_tooldo2,robot_mode\n")
            for r in data:
                f.write(f"{r['frame_id']},{r['timestamp']},{r['image_path_OBS_IMAGE_1']}")
                if has_obs2:
                    f.write(f",{r['image_path_OBS_IMAGE_2']}")
                f.write(',' + ','.join(map(str, r['joint_angles'])))
                f.write(',' + ','.join(map(str, r['tcp_pose'])))
                f.write(f",{r['gripper_tooldo1']},{r['gripper_tooldo2']},{r['robot_mode']}\n")
        # NPY
        np.save(os.path.join(save_dir, "dataset.npy"), data)
        # episode_meta.json
        import json as _json
        folder_num = os.path.basename(save_dir)
        n_frames = len(data)
        if n_frames >= 2:
            actual_fps = round((n_frames - 1) / (data[-1]['timestamp'] - data[0]['timestamp']), 3)
        else:
            actual_fps = 15.0
        ep_meta = dict(_state.get("episode_meta") or {})
        if _state.get("auto_mode") == "smolvla_5x10_strict":
            pose_id = _state.get("current_start_pose_id")
            current_pose_success = _state["pose_success_counts"].get(pose_id, 0) + (1 if bool(success) else 0)
            global_success_after = _sum_pose_success() + (1 if bool(success) else 0)
        else:
            pose_id = _state.get("current_start_pose_id")
            current_pose_success = None
            global_success_after = int(_state.get("auto_done", 0)) + (1 if bool(success) else 0)
        ep_meta.update({
            "folder": folder_num,
            "date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "total_frames": n_frames,
            "record_rate_hz": actual_fps,
            "cameras": "OBS_IMAGE_1+OBS_IMAGE_2" if has_obs2 else "OBS_IMAGE_1",
            "camera_mapping": {
                "OBS_IMAGE_1": "top",
                "OBS_IMAGE_2": "side",
            },
            "camera_source": {
                "OBS_IMAGE_1": "camera_1",
                "OBS_IMAGE_2": f"usb_camera_{int(_state.get('side_camera_id', 1))}",
            },
            "collection_policy": _state.get("auto_mode", "manual"),
            "start_pose_id": pose_id,
            "global_success_count_after_episode": global_success_after,
            "per_pose_success_target": SMOLVLA_SUCCESS_TARGET_PER_POSE if _state.get("auto_mode") == "smolvla_5x10_strict" else None,
            "start_pose_success_count_after_episode": current_pose_success,
            "success": bool(success),
            "vacuum_pick_duration_s": round(_state['vacuum_pick'], 3),
            "vacuum_place_duration_s": round(_state['vacuum_place'], 3),
        })
        events_raw = ep_meta.pop("events", [])
        with open(os.path.join(save_dir, "episode_meta.json"), 'w', encoding='utf-8') as f:
            _json.dump(ep_meta, f, ensure_ascii=False, indent=2)
        # episode_events.csv
        if events_raw and data:
            ts_list = [(r['frame_id'], r['timestamp']) for r in data]
            with open(os.path.join(save_dir, "episode_events.csv"), 'w', newline='') as f:
                f.write("event,frame_id,timestamp\n")
                for ev_name, ev_ts in events_raw:
                    closest_fid = min(ts_list, key=lambda t: abs(t[1] - ev_ts))[0]
                    f.write(f"{ev_name},{closest_fid},{ev_ts:.6f}\n")
        # metadata.txt (호환성 유지)
        with open(os.path.join(save_dir, "metadata.txt"), 'w') as f:
            f.write("VLA Dataset - Pick-Place Step\n" + "="*50 + "\n\n")
            f.write(f"Folder: {folder_num}\n")
            f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Total Frames: {len(data)}\n")
            f.write(f"Record Rate: {actual_fps}Hz\n")
            f.write(f"Cameras: OBS_IMAGE_1(top){' + OBS_IMAGE_2(side)' if has_obs2 else ''}\n")
            f.write(f"Step Success: {success}\n")
            f.write(f"VacuumCommandPickDuration_s: {_state['vacuum_pick']:.3f}\n")
            f.write(f"VacuumCommandPlaceDuration_s: {_state['vacuum_place']:.3f}\n")
        _state["episode_meta"] = {}
        _log(f"Saved {len(data)} frames → {save_dir}")
    except Exception as e:
        _log(f"Save error: {e}")
    finally:
        _state.update(recorded_data=[], record_save_dir=None)

# ═══════════════════════════════════════════════════════════════════════════
# Worker 콜백
# ═══════════════════════════════════════════════════════════════════════════

def _on_log(msg):    _log(msg)
def _on_vacuum(ph, pl): _state["vacuum_pick"] = ph; _state["vacuum_place"] = pl
def _on_rec_begin():
    if _state.get("record_enabled_for_step", True):
        _start_recording()
    else:
        _log("[record] skipped for reset step (non-recorded)")
def _on_episode_meta(meta): _state["episode_meta"] = meta

def _finalize_auto_collect(msg: str):
    _state.update(
        auto_target=0,
        auto_done=0,
        auto_mode="manual",
        auto_pending_complete=False,
        reset_running=False,
        reset_target_pose_id=None,
        next_drop_pose_id=None,
        active_step_kind="pick",
        record_enabled_for_step=True,
    )
    _log(msg)


def _run_reset_step():
    robot   = _state["robot"]
    gripper = _state["gripper"]
    if not robot or not robot.connected or not gripper:
        _finalize_auto_collect("⚠ reset step 시작 실패 (robot/gripper not connected)")
        return
    if _state["worker"] and _state["worker"].isRunning():
        _log("Worker already running (reset pending)")
        return

    drop_pose_id = _choose_next_drop_pose_id()
    if not drop_pose_id:
        _log("Reset mode off: skipping reset step")
        if _state.get("auto_pending_complete"):
            _finalize_auto_collect("Auto collect complete")
        else:
            threading.Timer(0.3, _run_pick_step).start()
        return
    spec = SMOLVLA_START_POSE_MAP[drop_pose_id]
    drop_x, drop_y = spec["xy"]
    _state["reset_running"] = True
    _state["reset_target_pose_id"] = drop_pose_id
    _state["next_drop_pose_id"] = drop_pose_id
    _state["active_step_kind"] = "reset"
    _state["record_enabled_for_step"] = False
    _log(f"[Reset] target={drop_pose_id} ({drop_x:.1f}, {drop_y:.1f})")

    cam_hik = _state["camera_hik"] if (_state["camera_hik"] and
                                        _state["camera_hik"].initialized) else None
    worker = RandomPosePickPlaceStepWorker(
        robot, gripper,
        pick_section=spec["section"],
        pick_x=drop_x,
        pick_y=drop_y,
        camera=cam_hik,
        fallback_initial_pose=FIXED_INIT,
        mode="reset_place",
        reset_place_x=drop_x,
        reset_place_y=drop_y,
        drop_pose_id=drop_pose_id,
    )
    worker.log_signal.connect(_on_log)
    worker.finished.connect(_on_finished)
    worker.episode_vacuum_durations.connect(_on_vacuum)
    worker.episode_meta_ready.connect(_on_episode_meta)
    worker.recording_begin_at_initial.connect(_on_rec_begin)
    _state["worker"] = worker
    worker.start()


def _run_pick_step():
    robot   = _state["robot"]
    gripper = _state["gripper"]
    if not robot or not robot.connected or not gripper:
        _log("Robot not connected")
        return
    if _state["worker"] and _state["worker"].isRunning():
        _log("Worker already running")
        return

    _set_random_init_pose(robot)
    cam_hik = _state["camera_hik"] if (_state["camera_hik"] and
                                        _state["camera_hik"].initialized) else None
    pick_section = _state["pick_section"]
    pick_x = _state["last_place_x"]
    pick_y = _state["last_place_y"]

    # SmolVLA strict 모드: 5개 시작 위치 순환 + pose별 10 성공 목표
    if _state.get("auto_mode") == "smolvla_5x10_strict":
        next_pose_id = _choose_next_start_pose_id()
        if next_pose_id is None:
            _finalize_auto_collect("Auto collect complete (all pose targets reached)")
            return
        spec = SMOLVLA_START_POSE_MAP[next_pose_id]
        pick_section = spec["section"]
        pick_x, pick_y = spec["xy"]
        _state["current_start_pose_id"] = next_pose_id
        _log(f"[StartPose] {next_pose_id} ({pick_section}, x={pick_x:.1f}, y={pick_y:.1f})")
    elif _state.get("auto_target", 0) > 0:
        _state["current_start_pose_id"] = "legacy"
    else:
        _state["current_start_pose_id"] = "manual_default"
    _state["active_step_kind"] = "pick"
    _state["record_enabled_for_step"] = True

    worker = RandomPosePickPlaceStepWorker(
        robot, gripper,
        pick_section = pick_section,
        pick_x       = pick_x,
        pick_y       = pick_y,
        camera       = cam_hik,
        fallback_initial_pose = FIXED_INIT,
        pick_only    = bool(_state.get("pick_only_mode", True)),
    )
    worker.log_signal.connect(_on_log)
    worker.finished.connect(_on_finished)
    worker.episode_vacuum_durations.connect(_on_vacuum)
    worker.episode_meta_ready.connect(_on_episode_meta)
    worker.recording_begin_at_initial.connect(_on_rec_begin)
    _state["worker"] = worker
    worker.start()


def _on_finished(success: bool):
    step_kind = _state.get("active_step_kind", "pick")
    if step_kind == "pick":
        _stop_and_save(success)
        if success:
            _state["pick_saved_count"] = int(_state.get("pick_saved_count", 0)) + 1
    else:
        _state["reset_running"] = False
        if success and _state.get("reset_target_pose_id") in _state.get("drop_success_counts", {}):
            rid = _state["reset_target_pose_id"]
            _state["drop_success_counts"][rid] += 1

    w = _state["worker"]
    if w and hasattr(w, 'place_x') and w.place_x is not None:
        _state["last_place_x"] = w.place_x
        _state["last_place_y"] = w.place_y
        _state["pick_section"] = "B" if _state["pick_section"] == "A" else "A"
        _log(f"Place ({w.place_x:.1f},{w.place_y:.1f}) / next: {_state['pick_section']}")

    if _state["auto_target"] <= 0:
        _log("Reset step complete" if step_kind == "reset" else "Step complete")
        return

    # reset step finished in auto mode
    if step_kind == "reset":
        _state["active_step_kind"] = "pick"
        _state["record_enabled_for_step"] = True
        if not success:
            _finalize_auto_collect("⚠ reset step 실패로 자동 수집 중단")
            return
        if _state.get("auto_pending_complete"):
            _finalize_auto_collect("Auto collect complete (reset finished)")
            return
        threading.Timer(0.3, _run_pick_step).start()
        return

    # pick step finished in auto mode
    if not success:
        if _state.get("auto_mode") == "smolvla_5x10_strict":
            _state["auto_attempt_total"] += 1
            pose_id = _state.get("current_start_pose_id")
            if pose_id in _state["pose_attempt_counts"]:
                _state["pose_attempt_counts"][pose_id] += 1
            _log("⚠ pick 실패 — success 카운트 증가 없음 (strict 모드 계속 진행)")
            threading.Timer(0.3, _run_pick_step).start()
            return
        _finalize_auto_collect("⚠ 자동 수집 중단됨 (pick 실패/STOP)")
        return

    _state["auto_attempt_total"] += 1
    pose_id = _state.get("current_start_pose_id")
    if pose_id in _state["pose_attempt_counts"]:
        _state["pose_attempt_counts"][pose_id] += 1
    _state["auto_done"] += 1

    completed_now = False
    if _state.get("auto_mode") == "smolvla_5x10_strict":
        if pose_id in _state["pose_success_counts"]:
            _state["pose_success_counts"][pose_id] += 1
        total_success = _sum_pose_success()
        _log(f"Auto(strict) success {total_success}/{SMOLVLA_STRICT_TOTAL_TARGET}")
        completed_now = total_success >= SMOLVLA_STRICT_TOTAL_TARGET
    else:
        _log(f"Auto(legacy) success {_state['auto_done']}/{_state['auto_target']}")
        completed_now = _state["auto_done"] >= _state["auto_target"]

    _state["auto_pending_complete"] = bool(completed_now)
    should_reset = bool(_state.get("pick_only_mode", True)) and bool(_state.get("auto_reset_enabled", True)) and _state.get("reset_mode", "strict_balanced") != "off"
    if should_reset:
        threading.Timer(0.3, _run_reset_step).start()
        return
    if completed_now:
        _finalize_auto_collect("Auto collect complete")
    else:
        threading.Timer(0.3, _run_pick_step).start()


def _run_step():
    # 기존 호출부 호환: pick step 엔트리로 위임
    _run_pick_step()

# ═══════════════════════════════════════════════════════════════════════════
# FastAPI
# ═══════════════════════════════════════════════════════════════════════════
app = FastAPI(title="Dobot E6 Server")

@app.on_event("startup")
async def _startup():
    global _log_queue, _main_loop, _ros2_ok, _robot_pub_running
    _log_queue = asyncio.Queue()
    _main_loop = asyncio.get_event_loop()
    asyncio.create_task(_broadcast())

    # ROS2 레코더 초기화 (rclpy 미설치 시 False 반환 → fallback 모드)
    _ros2_ok = _ros2.start()
    if _ros2_ok:
        _robot_pub_running = True
        threading.Thread(target=_robot_pub_loop, daemon=True).start()
        _log("ROS2 recorder ready (sync mode)")
    else:
        _log("ROS2 unavailable — legacy recording mode")

    _log("Server ready")

async def _broadcast():
    while True:
        msg = await _log_queue.get()
        dead = set()
        for ws in list(_ws_clients):
            try:
                await ws.send_text(msg)
            except Exception:
                dead.add(ws)
        _ws_clients.difference_update(dead)

# ─── 연결 ────────────────────────────────────────────────────────────────

@app.post("/connect")
def connect(ip: str = "192.168.5.1"):
    if _state["robot"] and _state["robot"].connected:
        return {"ok": True, "msg": "Already connected"}
    try:
        robot = DobotE6Controller(ip=ip)
        if not robot.connect():
            return JSONResponse({"ok": False, "msg": "Connect failed — robot unreachable"}, status_code=500)
        _state["robot"]   = robot
        _state["gripper"] = SuctionGripper(robot, do_index=1)
        _log(f"Robot connected @ {ip}")
        return {"ok": True, "msg": f"Connected @ {ip}"}
    except Exception as e:
        _log(f"Connect error: {e}")
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=500)

@app.post("/enable")
def enable_robot():
    robot = _state["robot"]
    if not robot or not robot.connected:
        return JSONResponse({"ok": False, "msg": "Not connected"}, status_code=400)
    try:
        robot.dashboard.EnableRobot()
        _log("Robot enabled")
        return {"ok": True, "msg": "Robot enabled"}
    except Exception as e:
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=500)

@app.post("/disable")
def disable_robot():
    robot = _state["robot"]
    if not robot or not robot.connected:
        return JSONResponse({"ok": False, "msg": "Not connected"}, status_code=400)
    try:
        robot.dashboard.DisableRobot()
        _log("Robot disabled")
        return {"ok": True, "msg": "Robot disabled"}
    except Exception as e:
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=500)

@app.post("/clear-alarm")
def clear_alarm():
    robot = _state["robot"]
    if not robot or not robot.connected:
        return JSONResponse({"ok": False, "msg": "Not connected"}, status_code=400)
    try:
        result = robot.dashboard.ClearError()
        _log(f"ClearError → {result}")
        return {"ok": True, "msg": f"Alarm cleared ({result})"}
    except Exception as e:
        _log(f"ClearError failed: {e}")
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=500)

@app.post("/resume")
def resume_robot():
    robot = _state["robot"]
    if not robot or not robot.connected:
        return JSONResponse({"ok": False, "msg": "Not connected"}, status_code=400)
    try:
        robot.resume_robot()
        robot.clear_error()
        robot.enable_robot(sleep_after=0.1)
        _log("Resume → ClearError + EnableRobot 완료")
        return {"ok": True}
    except Exception as e:
        _log(f"Resume failed: {e}")
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=500)

@app.post("/disconnect")
def disconnect():
    if _state["robot"]:
        try:
            _state["robot"].disconnect()
        except Exception:
            pass
        _state["robot"] = _state["gripper"] = None
        _log("Robot disconnected")
    return {"ok": True}

@app.get("/status")
def status():
    robot     = _state["robot"]
    connected = bool(robot and robot.connected)
    pose = joints = None
    robot_mode = 0
    if connected:
        try:
            feed = robot.feed.feedBackData()
            if feed is not None and len(feed) > 0:
                joints     = [round(float(v), 3) for v in feed['QActual'][0]]
                pose       = [round(float(v), 3) for v in feed['ToolVectorActual'][0]]
                robot_mode = int(feed['RobotMode'][0]) if 'RobotMode' in feed.dtype.names else 0
        except Exception:
            pass
    return {
        "connected":      connected,
        "pose":           pose,
        "joints":         joints,
        "robot_mode":     robot_mode,
        "robot_mode_str": ROBOT_MODE_LABELS.get(robot_mode, str(robot_mode)),
        "cam_hik":        bool(_state["camera_hik"] and _state["camera_hik"].initialized),
        "cam_zed":        bool(_state["camera_zed"] and _state["camera_zed"].initialized),  # legacy key
        "cam_obs2":       bool(_state["camera_zed"] and _state["camera_zed"].initialized),
        "recording":      _state["recording"],
        "frames":         _state["record_frame_count"],
        "auto_target":    _state["auto_target"],
        "auto_done":      _state["auto_done"],
        "auto_mode":      _state.get("auto_mode", "manual"),
        "auto_attempt_total": _state.get("auto_attempt_total", 0),
        "current_start_pose_id": _state.get("current_start_pose_id"),
        "pose_success_counts": _state.get("pose_success_counts", {}),
        "pose_attempt_counts": _state.get("pose_attempt_counts", {}),
        "smolvla_total_success": _sum_pose_success(),
        "drop_success_counts": _state.get("drop_success_counts", {}),
        "reset_mode": _state.get("reset_mode", "strict_balanced"),
        "reset_running": bool(_state.get("reset_running", False)),
        "reset_target_pose_id": _state.get("reset_target_pose_id"),
        "next_drop_pose_id": _state.get("next_drop_pose_id"),
        "pick_saved_count": int(_state.get("pick_saved_count", 0)),
        "auto_reset_enabled": bool(_state.get("auto_reset_enabled", True)),
        "side_camera_id": int(_state.get("side_camera_id", 1)),
        "pick_only_mode": bool(_state.get("pick_only_mode", True)),
        "worker_running": bool(_state["worker"] and _state["worker"].isRunning()),
    }

# ─── 로봇 제어 ────────────────────────────────────────────────────────────

@app.post("/home")
def go_home():
    robot = _state["robot"]
    if not robot or not robot.connected:
        return JSONResponse({"ok": False, "msg": "Not connected"}, status_code=400)
    def _do():
        ok = robot.move_j(300, 0, 400, 180, 0, 0, coordinate_mode=0, use_waypoint=False)
        if ok: robot.wait_for_motion_complete()
    threading.Thread(target=_do, daemon=True).start()
    return {"ok": True}

@app.post("/move")
def move(x: float, y: float, z: float,
         rx: float = 180.0, ry: float = 0.0, rz: float = 0.0,
         velocity: float = 30.0):
    robot = _state["robot"]
    if not robot or not robot.connected:
        return JSONResponse({"ok": False, "msg": "Not connected"}, status_code=400)
    def _do():
        ok = robot.move_j(x, y, z, rx, ry, rz, coordinate_mode=0,
                          velocity=velocity, use_waypoint=False)
        if ok: robot.wait_for_motion_complete()
    threading.Thread(target=_do, daemon=True).start()
    return {"ok": True}

_jog_lock = threading.Lock()
_jog_axis_active: list = [None]   # [0] = currently jogging axis or None
_jog_stop_time: list  = [0.0]     # [0] = last stop timestamp

@app.post("/jog/start")
def jog_start(axis: str, speed: int = 20):
    robot = _state["robot"]
    if not robot or not robot.connected:
        return JSONResponse({"ok": False, "msg": "Not connected"}, status_code=400)
    speed = max(1, min(100, speed))
    def _do():
        with _jog_lock:
            # cooldown: wait until 200ms after last stop
            elapsed = time.time() - _jog_stop_time[0]
            if elapsed < 0.20:
                time.sleep(0.20 - elapsed)
            try:
                robot.dashboard.EnableRobot()
            except Exception:
                pass
            try:
                robot.dashboard.SpeedFactor(speed)
            except Exception:
                pass
            for attempt in range(2):
                try:
                    if axis.startswith('J'):
                        result = robot.dashboard.MoveJog(axis)
                    else:
                        result = robot.dashboard.MoveJog(axis, coordtype=1, user=0, tool=0)
                    result_str = str(result).strip() if result else ""
                    first = result_str.split(',')[0].strip() if result_str else ""
                    if first and first != "0":
                        if attempt == 0:
                            time.sleep(0.15)
                            continue  # retry once
                        _log(f"[jog] {axis} error: {result_str}")
                    else:
                        _jog_axis_active[0] = axis
                        _log(f"[jog] {axis} start (speed={speed}%)")
                    break
                except Exception as e:
                    _log(f"[jog] {axis} exception: {e}")
                    break
    threading.Thread(target=_do, daemon=True).start()
    return {"ok": True}

@app.post("/jog/stop")
def jog_stop():
    robot = _state["robot"]
    if robot and robot.connected:
        def _stop():
            with _jog_lock:
                try:
                    robot.dashboard.MoveJog("")
                except Exception as e:
                    _log(f"[jog] stop error: {e}")
                try:
                    robot.dashboard.SpeedFactor(100)
                except Exception:
                    pass
                was = _jog_axis_active[0]
                _jog_axis_active[0] = None
                _jog_stop_time[0] = time.time()
                if was:
                    _log(f"[jog] {was} stopped")
        threading.Thread(target=_stop, daemon=True).start()
    return {"ok": True}

@app.get("/pose")
def get_pose():
    """현재 TCP 좌표 반환 (조그 후 위치 확인용)."""
    robot = _state["robot"]
    if not robot or not robot.connected:
        return JSONResponse({"ok": False, "msg": "Not connected"}, status_code=400)
    try:
        pose = robot.get_current_pose_from_feedback()
        if pose and len(pose) >= 6:
            return {"ok": True, "x": round(pose[0],3), "y": round(pose[1],3), "z": round(pose[2],3),
                    "rx": round(pose[3],3), "ry": round(pose[4],3), "rz": round(pose[5],3)}
        return JSONResponse({"ok": False, "msg": "No pose data"}, status_code=503)
    except Exception as e:
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=500)

@app.post("/gripper/grip")
def grip():
    g = _state["gripper"]
    if not g: return JSONResponse({"ok": False, "msg": "No gripper"}, status_code=400)
    threading.Thread(target=g.grip, daemon=True).start()
    return {"ok": True}

@app.post("/gripper/release")
def release():
    g = _state["gripper"]
    if not g: return JSONResponse({"ok": False, "msg": "No gripper"}, status_code=400)
    threading.Thread(target=g.release, daemon=True).start()
    return {"ok": True}

@app.post("/estop")
def estop():
    w = _state["worker"]
    if w: w._stop_requested = True
    if _state["gripper"]:
        try: _state["gripper"].emergency_release()
        except Exception: pass
    if _state["robot"]: _state["robot"].disable_robot()
    _log("E-STOP triggered")
    return {"ok": True}

# ─── Pick-Place ───────────────────────────────────────────────────────────

@app.post("/pick-place/step")
def step(pick_only: bool = True):
    if _state["worker"] and _state["worker"].isRunning():
        return JSONResponse({"ok": False, "msg": "Already running"}, status_code=400)
    if not _state["robot"] or not _state["robot"].connected:
        return JSONResponse({"ok": False, "msg": "Not connected"}, status_code=400)
    _state["auto_target"] = 0
    _state["auto_mode"] = "manual"
    _state["current_start_pose_id"] = "manual_default"
    _state["pick_only_mode"] = bool(pick_only)
    _state["active_step_kind"] = "pick"
    _state["record_enabled_for_step"] = True
    threading.Thread(target=_run_step, daemon=True).start()
    return {"ok": True}

@app.post("/pick-place/auto")
def auto_collect(n: int = 10, strict: bool = False, pick_only: bool = True, auto_reset: bool = True):
    if _state["worker"] and _state["worker"].isRunning():
        return JSONResponse({"ok": False, "msg": "Already running"}, status_code=400)
    if not _state["robot"] or not _state["robot"].connected:
        return JSONResponse({"ok": False, "msg": "Not connected"}, status_code=400)
    if strict:
        _state.update(
            auto_target=SMOLVLA_STRICT_TOTAL_TARGET,
            auto_done=0,
            auto_mode="smolvla_5x10_strict",
            auto_attempt_total=0,
            current_start_pose_id=None,
            pose_success_counts=_new_pose_counter_map(),
            pose_attempt_counts=_new_pose_counter_map(),
            drop_success_counts=_new_pose_counter_map(),
            pose_rr_index=0,
            drop_cycle_index=0,
        )
        _log("Auto collect started: SmolVLA strict (5 poses x 10 success)")
    else:
        _state.update(
            auto_target=n,
            auto_done=0,
            auto_mode="legacy_count",
            auto_attempt_total=0,
            current_start_pose_id="legacy",
        )
        _log(f"Auto collect started: {n} episodes (legacy)")
    _state["pick_only_mode"] = bool(pick_only)
    _state["auto_reset_enabled"] = bool(auto_reset)
    _state["reset_mode"] = "strict_balanced" if bool(auto_reset) else "off"
    _state["pick_saved_count"] = 0
    _state["auto_pending_complete"] = False
    _state["next_drop_pose_id"] = None
    _state["reset_target_pose_id"] = None
    _state["reset_running"] = False
    _state["active_step_kind"] = "pick"
    _state["record_enabled_for_step"] = True
    threading.Thread(target=_run_step, daemon=True).start()
    return {"ok": True}

@app.post("/pick-place/stop")
def stop_collect():
    if _state["worker"]: _state["worker"]._stop_requested = True
    _state["auto_target"] = 0
    _state["auto_mode"] = "manual"
    _state["auto_pending_complete"] = False
    _state["reset_running"] = False
    _state["active_step_kind"] = "pick"
    _state["record_enabled_for_step"] = True
    _log("Stop requested")
    return {"ok": True}

# ─── 카메라 ──────────────────────────────────────────────────────────────

@app.post("/camera/hik/start")
def hik_start():
    global _cam_hik_thread, _cam_hik_running
    if not _hik_available:
        return JSONResponse({"ok": False, "msg": "HIK SDK not available"}, status_code=400)
    if _state["camera_hik"] and _state["camera_hik"].initialized:
        return {"ok": True, "msg": "Already running"}
    try:
        cam = HikRobotCamera()
        if not cam.init_camera():
            return JSONResponse({"ok": False, "msg": "Exterior Cam 1 (HIK) init failed — check USB connection"}, status_code=500)
        _state["camera_hik"] = cam
        _cam_hik_running = True
        _cam_hik_thread  = threading.Thread(target=_hik_grab_loop, daemon=True)
        _cam_hik_thread.start()
        _log("Exterior Cam 1 (HIKRobot) started")
        return {"ok": True}
    except Exception as e:
        _log(f"HIK start error: {e}")
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=500)

@app.post("/camera/hik/stop")
def hik_stop():
    global _cam_hik_running
    _cam_hik_running = False
    if _state["camera_hik"]:
        try: _state["camera_hik"].cleanup()
        except Exception: pass
        _state["camera_hik"] = None
    _log("Exterior Cam 1 (HIKRobot) stopped")
    return {"ok": True}

@app.post("/camera/zed/start")
def zed_start(camera_id: int = 1):
    global _cam_zed_thread, _cam_zed_running
    if not _obs2_available:
        return JSONResponse({"ok": False, "msg": "OBS_IMAGE_2 module not available"}, status_code=400)
    if _state["camera_zed"] and _state["camera_zed"].initialized:
        return {"ok": True, "msg": "Already running"}
    try:
        cam = SideUsbCamera(camera_id=int(camera_id))
        if not cam.init_camera():
            return JSONResponse({"ok": False, "msg": f"OBS_IMAGE_2 init failed (camera_id={camera_id})"}, status_code=500)
        _state["camera_zed"] = cam
        _state["side_camera_id"] = int(camera_id)
        _cam_zed_running = True
        _cam_zed_thread  = threading.Thread(target=_zed_grab_loop, daemon=True)
        _cam_zed_thread.start()
        _log(f"OBS_IMAGE_2 started (USB camera_id={camera_id})")
        return {"ok": True}
    except Exception as e:
        _log(f"OBS_IMAGE_2 start error: {e}")
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=500)

@app.post("/camera/zed/stop")
def zed_stop():
    global _cam_zed_running
    _cam_zed_running = False
    if _state["camera_zed"]:
        try: _state["camera_zed"].cleanup()
        except Exception: pass
        _state["camera_zed"] = None
    _log("OBS_IMAGE_2 stopped")
    return {"ok": True}

@app.get("/camera/hik/stream")
async def hik_stream():
    return StreamingResponse(
        _mjpeg_gen(lambda: _buf_hik_jpg),
        media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/camera/zed/stream")
async def zed_stream():
    return StreamingResponse(
        _mjpeg_gen(lambda: _buf_zed_jpg),
        media_type="multipart/x-mixed-replace; boundary=frame")

# ─── WebSocket ────────────────────────────────────────────────────────────

@app.websocket("/ws/logs")
async def ws_logs(ws: WebSocket):
    await ws.accept()
    _ws_clients.add(ws)
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        _ws_clients.discard(ws)

# ─── Web UI ───────────────────────────────────────────────────────────────

@app.get("/", response_class=HTMLResponse)
async def index():
    return HTMLResponse(_HTML)

_HTML = r"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Dobot E6 Server</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',sans-serif;background:#12121f;color:#dde;font-size:13px;min-width:900px}
h1{text-align:center;padding:9px;background:#0e0e1d;color:#00c8e8;font-size:1.05rem;letter-spacing:1px;border-bottom:1px solid #1e2a3a}

/* 3-column layout */
.layout{display:grid;grid-template-columns:300px 1fr 340px;gap:8px;padding:8px;align-items:start}
.col{display:flex;flex-direction:column;gap:8px;min-width:0}

/* card */
.card{background:#1a1a30;border-radius:7px;padding:11px;overflow:hidden}
h3{color:#00c8e8;font-size:.7rem;text-transform:uppercase;letter-spacing:.6px;margin-bottom:9px;border-bottom:1px solid #1e2a3a;padding-bottom:5px}

/* row, inputs, buttons */
.row{display:flex;gap:5px;margin-bottom:6px;align-items:center;flex-wrap:wrap}
input[type=text],input[type=number]{background:#0d1a2e;color:#dde;border:1px solid #2a4a6a;
  padding:4px 7px;border-radius:4px;flex:1;min-width:0;font-size:.8rem}
button{background:#0d1a2e;color:#aac8e0;border:1px solid #2a4a6a;padding:5px 10px;
  border-radius:4px;cursor:pointer;font-size:.78rem;white-space:nowrap;transition:background .12s}
button:hover{background:#00c8e8;color:#0d1a2e;border-color:#00c8e8}
button:active{filter:brightness(1.3)}
.btn-g{border-color:#2dc653;color:#2dc653}.btn-g:hover{background:#2dc653;color:#0d1a2e}
.btn-r{border-color:#e63946;color:#e63946}.btn-r:hover{background:#e63946;color:#fff}
.btn-y{border-color:#f4a261;color:#f4a261}.btn-y:hover{background:#f4a261;color:#0d1a2e}

/* status bars */
.sbar{background:#0d1a2e;padding:5px 9px;border-radius:4px;font-size:.72rem;margin-bottom:5px}
.dot{display:inline-block;width:7px;height:7px;border-radius:50%;margin-right:4px;vertical-align:middle}
.on{background:#2dc653}.off{background:#e63946}.rec{background:#e63946;animation:blink 1s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.3}}

/* dashboard grid */
.dash-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:4px}
.dash-cell{background:#0d1a2e;border-radius:4px;padding:5px 4px;text-align:center}
.dash-label{font-size:.58rem;color:#557;text-transform:uppercase}
.dash-val{font-size:.88rem;font-weight:700;color:#00c8e8;font-family:monospace}

/* cameras */
.cam-row{display:grid;grid-template-columns:1fr 1fr;gap:4px}
.cam-box{background:#0d1a2e;border-radius:5px;overflow:hidden}
.cam-label{font-size:.63rem;color:#446;padding:3px 6px;background:#0a0f1e}
img.stream{width:100%;height:480px;object-fit:contain;display:block;background:#000}

/* log */
#log{background:#060612;font-family:monospace;font-size:.67rem;height:140px;overflow-y:auto;
  padding:6px;border-radius:4px;color:#6fdf8f;word-break:break-all}

/* separator */
.sep{border-top:1px solid #1e2a3a;margin:8px 0}
.sub{font-size:.63rem;color:#557;margin-bottom:5px;margin-top:2px}

/* ── JOG ── */
/* D-pad: 3×3 grid */
.dpad{display:grid;grid-template-columns:repeat(3,52px);grid-template-rows:repeat(3,40px);gap:4px}
.dpad .jb{font-size:.9rem;font-weight:700;padding:0;display:flex;align-items:center;justify-content:center;
  border-radius:5px;user-select:none;-webkit-user-select:none;touch-action:none}
.dpad .jb.center{background:#1e2a3a;color:#446;font-size:.6rem;cursor:default;border:1px solid #1e2a3a}
.dpad .jb.center:hover{background:#1e2a3a;color:#446;border-color:#1e2a3a}

/* Z col */
.zcol{display:flex;flex-direction:column;gap:4px;margin-left:8px}
.zcol .jb{width:48px;height:40px;font-size:.85rem;font-weight:700;display:flex;align-items:center;
  justify-content:center;border-radius:5px;user-select:none;-webkit-user-select:none;touch-action:none}

/* rotation row */
.rot-row{display:grid;grid-template-columns:repeat(6,1fr);gap:4px}
.rot-row .jb{padding:5px 2px;font-size:.72rem;text-align:center;font-weight:600;
  border-radius:4px;user-select:none;-webkit-user-select:none;touch-action:none}

/* joint row */
.joint-table{display:grid;grid-template-columns:repeat(6,1fr);gap:4px}
.joint-table .jb{padding:6px 2px;font-size:.72rem;text-align:center;
  border-radius:4px;user-select:none;-webkit-user-select:none;touch-action:none}

/* active jog highlight */
.jb.jogging{background:#00c8e8 !important;color:#0d1a2e !important;border-color:#00c8e8 !important}

/* speed slider */
input[type=range]{width:100%;accent-color:#00c8e8}

/* pose capture */
#pose-display{font-size:.68rem;color:#9cf;margin-top:4px;font-family:monospace;word-break:break-all;min-height:16px}
</style>
</head>
<body>
<h1>⬡ Dobot E6 — Robot Control &amp; Data Collection</h1>
<div class="layout">

<!-- ══════════════ LEFT COL ══════════════ -->
<div class="col">

  <!-- Connection -->
  <div class="card">
    <h3>Connection</h3>
    <div class="row">
      <input id="ip" type="text" value="192.168.5.1" style="max-width:115px">
      <button class="btn-g" onclick="api('POST','/connect',{ip:$('ip').value})">Connect</button>
      <button onclick="api('POST','/disconnect')">Disconnect</button>
    </div>
    <div id="conn-bar" class="sbar"><span class="dot off"></span>Disconnected</div>
    <div id="mode-bar" class="sbar" style="margin-bottom:7px">Mode: —</div>
    <div class="row" style="margin-bottom:0;gap:4px">
      <button class="btn-g" onclick="api('POST','/enable')">Enable</button>
      <button onclick="api('POST','/disable')">Disable</button>
      <button class="btn-y" onclick="clearAlarm()">Clear Alarm</button>
      <button class="btn-y" onclick="api('POST','/resume').then(()=>addLog('▶ Resume sent'))">Resume</button>
      <button onclick="api('POST','/home')">Home</button>
    </div>
  </div>

  <!-- Dashboard -->
  <div class="card">
    <h3>Robot Dashboard</h3>
    <div class="sub">TCP Pose (mm / deg)</div>
    <div class="dash-grid">
      <div class="dash-cell"><div class="dash-label">X</div><div class="dash-val" id="dX">—</div></div>
      <div class="dash-cell"><div class="dash-label">Y</div><div class="dash-val" id="dY">—</div></div>
      <div class="dash-cell"><div class="dash-label">Z</div><div class="dash-val" id="dZ">—</div></div>
      <div class="dash-cell"><div class="dash-label">RX</div><div class="dash-val" id="dRX">—</div></div>
      <div class="dash-cell"><div class="dash-label">RY</div><div class="dash-val" id="dRY">—</div></div>
      <div class="dash-cell"><div class="dash-label">RZ</div><div class="dash-val" id="dRZ">—</div></div>
    </div>
    <div class="sub" style="margin-top:8px">Joint Angles (deg)</div>
    <div class="dash-grid">
      <div class="dash-cell"><div class="dash-label">J1</div><div class="dash-val" id="dJ1">—</div></div>
      <div class="dash-cell"><div class="dash-label">J2</div><div class="dash-val" id="dJ2">—</div></div>
      <div class="dash-cell"><div class="dash-label">J3</div><div class="dash-val" id="dJ3">—</div></div>
      <div class="dash-cell"><div class="dash-label">J4</div><div class="dash-val" id="dJ4">—</div></div>
      <div class="dash-cell"><div class="dash-label">J5</div><div class="dash-val" id="dJ5">—</div></div>
      <div class="dash-cell"><div class="dash-label">J6</div><div class="dash-val" id="dJ6">—</div></div>
    </div>
  </div>

  <!-- Data Collection -->
  <div class="card">
    <h3>Data Collection</h3>
    <div id="auto-bar" class="sbar">Ready</div>
    <div id="pose-progress" class="sbar" style="white-space:pre-line;font-size:.66rem;color:#9cc;min-height:76px">SmolVLA 5x10 progress: -</div>
    <div id="reset-progress" class="sbar" style="white-space:pre-line;font-size:.66rem;color:#9cc;min-height:48px">Reset progress: -</div>
    <div class="row">
      <button class="btn-g" onclick="stepRun()">▶ Step</button>
      <input id="auto-n" type="number" value="50" min="1" style="max-width:55px">
      <button class="btn-g" onclick="autoCollect()">▶ Auto (n)</button>
      <label style="font-size:.68rem;color:#9cc"><input id="strict-mode" type="checkbox" checked> strict 5x10</label>
      <label style="font-size:.68rem;color:#9cc"><input id="pick-only" type="checkbox" checked> pick-only</label>
      <label style="font-size:.68rem;color:#9cc"><input id="auto-reset" type="checkbox" checked> auto-reset</label>
      <button class="btn-y" onclick="api('POST','/pick-place/stop')">■ Stop</button>
    </div>
    <button class="btn-r" style="width:100%;padding:8px;font-size:.82rem;font-weight:700" onclick="doEstop()">⚠ E-STOP</button>
  </div>

</div><!-- end left col -->

<!-- ══════════════ CENTER COL ══════════════ -->
<div class="col">

  <!-- Camera controls -->
  <div class="card">
    <h3>Exterior Cameras</h3>
    <div class="row" style="margin-bottom:4px">
      <span style="font-size:.72rem;color:#00c8e8;min-width:105px">Cam 1 — HIKRobot</span>
      <button class="btn-g" onclick="api('POST','/camera/hik/start')">Start</button>
      <button onclick="api('POST','/camera/hik/stop')">Stop</button>
      <span id="hik-stat" style="font-size:.72rem;color:#668;margin-left:6px">OFF</span>
    </div>
    <div class="row" style="margin-bottom:0">
      <span style="font-size:.72rem;color:#00c8e8;min-width:105px">Cam 2 — OBS_IMAGE_2</span>
      <input id="obs2-id" type="number" value="1" min="0" style="max-width:55px">
      <button class="btn-g" onclick="api('POST','/camera/zed/start',{camera_id:$('obs2-id').value})">Start</button>
      <button onclick="api('POST','/camera/zed/stop')">Stop</button>
      <span id="zed-stat" style="font-size:.72rem;color:#668;margin-left:6px">OFF</span>
    </div>
  </div>

  <!-- Camera streams -->
  <div class="card" style="padding:8px">
    <div class="cam-row">
      <div class="cam-box">
        <div class="cam-label">Cam 1 — HIKRobot</div>
        <img class="stream" src="/camera/hik/stream" alt="HIK">
      </div>
      <div class="cam-box">
        <div class="cam-label">Cam 2 — OBS_IMAGE_2 (USB)</div>
        <img class="stream" src="/camera/zed/stream" alt="ZED">
      </div>
    </div>
  </div>

  <!-- Log -->
  <div class="card">
    <h3>Log</h3>
    <div id="log"></div>
  </div>

</div><!-- end center col -->

<!-- ══════════════ RIGHT COL: JOG ══════════════ -->
<div class="col">
  <div class="card">
    <h3>Jog Control</h3>

    <!-- Gripper -->
    <div class="row" style="margin-bottom:7px">
      <button class="btn-g" style="flex:1;padding:7px" onclick="api('POST','/gripper/grip')">Grip ON [Q]</button>
      <button style="flex:1;padding:7px" onclick="api('POST','/gripper/release')">Grip OFF [W]</button>
    </div>

    <!-- Speed -->
    <div style="margin-bottom:8px">
      <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:3px">
        <span style="font-size:.73rem;color:#aac8e0;font-weight:600">Jog Speed</span>
        <span style="font-size:.82rem;font-weight:700;color:#00c8e8"><span id="speed-val">5</span>%</span>
      </div>
      <input type="range" id="jog-speed" min="1" max="50" value="5"
             oninput="$('speed-val').textContent=this.value">
    </div>

    <!-- Capture pose -->
    <div style="margin-bottom:8px">
      <button onclick="capturePose()" style="width:100%;padding:6px;font-size:.75rem;background:#263445;border-color:#3a5a7a;color:#9cf">
        📍 현재 좌표 캡처
      </button>
      <div id="pose-display"></div>
    </div>

    <div class="sep"></div>

    <!-- TCP XY D-pad + Z -->
    <div class="sub">TCP XY / Z &nbsp;·&nbsp; 키보드: ←→↑↓ / Z=Z+ X=Z-</div>
    <div style="display:flex;align-items:center;margin-bottom:8px">
      <!-- D-pad 3×3 -->
      <div class="dpad">
        <div></div>
        <button class="jb btn-g" id="jb-Y+" data-axis="Y+">↑<br><small style="font-size:.55rem">Y+</small></button>
        <div></div>
        <button class="jb btn-g" id="jb-X+" data-axis="X+">←<br><small style="font-size:.55rem">X+</small></button>
        <div class="jb center">XY</div>
        <button class="jb btn-g" id="jb-X-" data-axis="X-">→<br><small style="font-size:.55rem">X-</small></button>
        <div></div>
        <button class="jb btn-g" id="jb-Y-" data-axis="Y-">↓<br><small style="font-size:.55rem">Y-</small></button>
        <div></div>
      </div>
      <!-- Z col -->
      <div class="zcol">
        <button class="jb btn-g" id="jb-Z+" data-axis="Z+">Z+<br><small style="font-size:.55rem">▲</small></button>
        <button class="jb btn-g" id="jb-Z-" data-axis="Z-">Z-<br><small style="font-size:.55rem">▼</small></button>
      </div>
    </div>

    <!-- Rotation -->
    <div class="sub">Rotation (Rx / Ry / Rz)</div>
    <div class="rot-row" style="margin-bottom:8px">
      <button class="jb" id="jb-Rx+" data-axis="Rx+">Rx+</button>
      <button class="jb" id="jb-Rx-" data-axis="Rx-">Rx-</button>
      <button class="jb" id="jb-Ry+" data-axis="Ry+">Ry+</button>
      <button class="jb" id="jb-Ry-" data-axis="Ry-">Ry-</button>
      <button class="jb" id="jb-Rz+" data-axis="Rz+">Rz+</button>
      <button class="jb" id="jb-Rz-" data-axis="Rz-">Rz-</button>
    </div>

    <div class="sep"></div>

    <!-- Joint jog -->
    <div class="sub">Joint Jog</div>
    <div class="joint-table">
      <button class="jb" id="jb-J1+" data-axis="J1+">J1+</button>
      <button class="jb" id="jb-J1-" data-axis="J1-">J1-</button>
      <button class="jb" id="jb-J2+" data-axis="J2+">J2+</button>
      <button class="jb" id="jb-J2-" data-axis="J2-">J2-</button>
      <button class="jb" id="jb-J3+" data-axis="J3+">J3+</button>
      <button class="jb" id="jb-J3-" data-axis="J3-">J3-</button>
      <button class="jb" id="jb-J4+" data-axis="J4+">J4+</button>
      <button class="jb" id="jb-J4-" data-axis="J4-">J4-</button>
      <button class="jb" id="jb-J5+" data-axis="J5+">J5+</button>
      <button class="jb" id="jb-J5-" data-axis="J5-">J5-</button>
      <button class="jb" id="jb-J6+" data-axis="J6+">J6+</button>
      <button class="jb" id="jb-J6-" data-axis="J6-">J6-</button>
    </div>

  </div>
</div><!-- end right col -->

</div><!-- end layout -->

<script>
const $ = id => document.getElementById(id);
const api = async (m, p, q={}) => {
  const url = p + (Object.keys(q).length ? '?' + new URLSearchParams(q) : '');
  try {
    const res = await fetch(url, {method:m});
    const data = await res.json();
    if (data && data.msg) addLog((data.ok ? '✓ ' : '✗ ') + data.msg);
    if (!res.ok && !data.msg) addLog(`✗ ${m} ${p} → HTTP ${res.status}`);
    return data;
  } catch(e) { addLog('✗ Network: ' + e); }
};
const addLog = msg => {
  const b=$('log'); b.innerHTML += msg+'<br>'; b.scrollTop=b.scrollHeight;
};

// WebSocket
const ws = new WebSocket(`ws://${location.host}/ws/logs`);
ws.onmessage = e => addLog(e.data);
ws.onopen = () => addLog('[WS] connected');
setInterval(() => { if(ws.readyState===1) ws.send('ping'); }, 10000);

// Status polling
setInterval(async () => {
  const s = await api('GET','/status');
  if(!s) return;
  $('conn-bar').innerHTML = `<span class="dot ${s.connected?'on':'off'}"></span>`
    + (s.connected ? 'Connected' : 'Disconnected')
    + (s.recording ? ' &nbsp;<span class="dot rec"></span><b> REC</b>' : '');

  // 로봇 MODE 표시 — ERROR(9)면 빨간 경고
  const isError = s.robot_mode === 9;
  $('mode-bar').textContent = (isError ? '⚠ ROBOT ERROR — ' : 'Mode: ')
    + (s.robot_mode_str||'—')
    + (s.recording ? ` | REC ${s.frames??''}f` : '');
  $('mode-bar').style.background = isError ? '#3a0a0a' : '#0d1a2e';
  $('mode-bar').style.color      = isError ? '#ff4444' : '#dde';
  $('mode-bar').style.fontWeight = isError ? '700' : 'normal';

  // ERROR 상태면 자동 수집 서버 측 중단 알림
  if (isError && s.auto_target > 0) {
    addLog('⚠ [ERROR] 로봇 충돌/알람 감지 — 자동 수집 중단됨. Clear Alarm 후 재시작하세요.');
    api('POST','/pick-place/stop');
  }
  if(s.joints) ['J1','J2','J3','J4','J5','J6'].forEach((k,i)=>{
    const el=$('d'+k); if(el) el.textContent=s.joints[i]?.toFixed(1)??'—';
  });
  if(s.pose) ['X','Y','Z','RX','RY','RZ'].forEach((k,i)=>{
    const el=$('d'+k); if(el) el.textContent=s.pose[i]?.toFixed(1)??'—';
  });
  $('hik-stat').textContent = s.cam_hik ? 'ON ●' : 'OFF';
  $('hik-stat').style.color  = s.cam_hik ? '#2dc653' : '#668';
  $('zed-stat').textContent  = s.cam_obs2 ? `ON ● (id=${s.side_camera_id})` : 'OFF';
  $('zed-stat').style.color  = s.cam_obs2 ? '#2dc653' : '#668';
  if(s.auto_target > 0)
    $('auto-bar').textContent = `Auto(${s.auto_mode||'manual'}): success ${s.auto_done}/${s.auto_target}, attempts ${s.auto_attempt_total||0}, reset=${s.reset_running?'RUN':'WAIT'}`;
  else if(s.worker_running)
    $('auto-bar').textContent = 'Running…';
  else
    $('auto-bar').textContent = `Ready (pick-only=${s.pick_only_mode ? 'ON' : 'OFF'}, auto-reset=${s.auto_reset_enabled ? 'ON' : 'OFF'})`;
  if (s.pose_success_counts) {
    const order = ['pose_A_2','pose_A_3','pose_A_4','pose_B_8','pose_B_9'];
    let lines = [`SmolVLA 5x10: total ${s.smolvla_total_success||0}/50`];
    for (const k of order) {
      const sv = (s.pose_success_counts && s.pose_success_counts[k]) || 0;
      const av = (s.pose_attempt_counts && s.pose_attempt_counts[k]) || 0;
      lines.push(`${k}: ${sv}/10 success (attempt ${av})`);
    }
    $('pose-progress').textContent = lines.join('\n');
  }
  if (s.drop_success_counts) {
    const order = ['pose_A_2','pose_A_3','pose_A_4','pose_B_8','pose_B_9'];
    let lines = [`Reset: next=${s.next_drop_pose_id||'-'}, active=${s.reset_target_pose_id||'-'}, pick_saved=${s.pick_saved_count||0}`];
    for (const k of order) {
      const dv = (s.drop_success_counts && s.drop_success_counts[k]) || 0;
      lines.push(`${k}: drop ${dv}`);
    }
    $('reset-progress').textContent = lines.join('\n');
  }
}, 800);

// ── Jog logic ──────────────────────────────
const getSpeed = () => parseInt($('jog-speed').value) || 5;
let _jogAxis = null;  // currently held axis (prevents duplicate stop calls)

const jogStart = axis => {
  if (_jogAxis === axis) return;  // already jogging this axis
  _jogAxis = axis;
  const el = document.getElementById('jb-' + axis);
  if (el) el.classList.add('jogging');
  api('POST','/jog/start',{axis, speed:getSpeed()});
};
const jogStop = () => {
  if (!_jogAxis) return;  // nothing to stop
  const el = document.getElementById('jb-' + _jogAxis);
  if (el) el.classList.remove('jogging');
  _jogAxis = null;
  api('POST','/jog/stop');
};

// Attach events to all .jb buttons with data-axis
document.querySelectorAll('.jb[data-axis]').forEach(btn => {
  const ax = btn.dataset.axis;
  btn.addEventListener('mousedown',  e => { e.preventDefault(); jogStart(ax); });
  btn.addEventListener('touchstart', e => { e.preventDefault(); jogStart(ax); });
  btn.addEventListener('mouseup',    () => jogStop());
  btn.addEventListener('touchend',   () => jogStop());
  btn.addEventListener('mouseleave', () => { if(_jogAxis===ax) jogStop(); });
});

// Keyboard jog
const KEY_MAP = {
  'ArrowLeft':'X+', 'ArrowRight':'X-',
  'ArrowUp':'Y+',   'ArrowDown':'Y-',
  'z':'Z+', 'Z':'Z+',
  'x':'Z-', 'X':'Z-',
};
document.addEventListener('keydown', e => {
  if (e.target.tagName==='INPUT'||e.target.tagName==='TEXTAREA') return;
  if (e.repeat) return;
  const k = e.key;
  if (k==='q'||k==='Q') { api('POST','/gripper/grip'); return; }
  if (k==='w'||k==='W') { api('POST','/gripper/release'); return; }
  const ax = KEY_MAP[k];
  if (ax) { jogStart(ax); e.preventDefault(); }
});
document.addEventListener('keyup', e => {
  const ax = KEY_MAP[e.key];
  if (ax && _jogAxis===ax) { jogStop(); e.preventDefault(); }
});
window.addEventListener('blur', () => jogStop());

// Capture pose
const capturePose = async () => {
  const d = await api('GET','/pose');
  if(d && d.ok)
    $('pose-display').textContent = `X:${d.x}  Y:${d.y}  Z:${d.z} | Rx:${d.rx}  Ry:${d.ry}  Rz:${d.rz}`;
  else
    $('pose-display').textContent = 'fetch failed';
};

// Misc handlers
const stepRun = () => api('POST','/pick-place/step',{
  pick_only:$('pick-only').checked ? 1 : 0
});
const autoCollect = () => api('POST','/pick-place/auto',{
  n:$('auto-n').value,
  strict:$('strict-mode').checked ? 1 : 0,
  pick_only:$('pick-only').checked ? 1 : 0,
  auto_reset:$('auto-reset').checked ? 1 : 0
});
const doEstop = () => { if(confirm('E-STOP?')) api('POST','/estop'); };
const clearAlarm = async () => {
  const d = await api('POST','/clear-alarm');
  if(d&&d.ok) addLog('✓ Alarm cleared');
};
</script>
</body>
</html>"""

# ═══════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import uvicorn, socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80)); ip = s.getsockname()[0]; s.close()
    except Exception:
        ip = "localhost"
    print(f"\n  Dobot E6 Robot Server")
    print(f"  Open: http://{ip}:8000\n")
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")
