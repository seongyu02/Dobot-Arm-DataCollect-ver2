#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Merged Pick-and-Place GUI
- Combines pick_place_gui.py (camera-based selection) + pick_place_gui_moveit.py (pose control)
- Adds waypoint creation and sequence execution
"""

import sys
import os
import json

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

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

# Path setup for imports (camera_viewer needs MvImport from workspace root)
_merge_dir = os.path.dirname(os.path.abspath(__file__))
_src_dir = os.path.join(_merge_dir, 'src')
_workspace_root = os.path.dirname(_merge_dir)
sys.path.insert(0, _src_dir)
sys.path.insert(0, _workspace_root)

# Camera availability check (camera_viewer imports MvImport from workspace root)
CAMERA_AVAILABLE = False
try:
    from camera_viewer import HikRobotCamera
    CAMERA_AVAILABLE = True
except ImportError as e:
    CAMERA_AVAILABLE = False
    print(f"Warning: Camera module not available ({e})")

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QGridLayout, QLabel, QLineEdit, QPushButton, QTextEdit,
    QDoubleSpinBox, QMessageBox, QComboBox, QCheckBox, QInputDialog,
    QListWidget, QListWidgetItem, QFileDialog, QTabWidget,
    QDialog, QFormLayout, QDialogButtonBox
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap
import numpy as np
import cv2
import time
from typing import Optional, Tuple, List, Dict
from datetime import datetime

# Import robot control modules (paths already set above)
current_dir = _merge_dir
src_dir = _src_dir
parent_dir = _workspace_root

from dobot_e6_controller import DobotE6Controller
from suction_gripper import SuctionGripper
from pick_place_logic import PickAndPlace

if CAMERA_AVAILABLE:
    from camera_viewer import HikRobotCamera


if CAMERA_AVAILABLE:
    class CameraThread(QThread):
        """Camera streaming thread"""
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
                time.sleep(0.033)  # ~30 FPS
        
        def stop(self):
            self.running = False


class WaypointSequenceWorker(QThread):
    """Worker thread for waypoint sequence execution. Supports joint/pose waypoints and optional per-waypoint randomness."""
    finished = pyqtSignal(bool)
    log_signal = pyqtSignal(str)
    waypoint_reached = pyqtSignal(int)  # waypoint index
    
    def __init__(self, robot, waypoints, use_gripper_list=None, waypoint_randomness=0.0):
        super().__init__()
        self.robot = robot
        self.waypoints = waypoints  # List of dict: pose, joints, use_gripper, name
        self.use_gripper_list = use_gripper_list or []
        self.waypoint_randomness = float(waypoint_randomness)  # ±° for joints, ±mm/° for pose
    
    def _get_joints_from_feedback(self):
        """Get joint angles from feedback"""
        try:
            if not self.robot or not self.robot.feed:
                return None
            data = self.robot.feed.feedBackData()
            if data is not None and len(data) > 0:
                if hex(data['TestValue'][0]) == '0x123456789abcdef':
                    joints = data['QActual'][0].tolist()
                    joints_deg = [np.rad2deg(j) for j in joints]
                    return joints_deg
        except Exception:
            pass
        return None
    
    def run(self):
        try:
            for i, wp in enumerate(self.waypoints):
                if isinstance(wp, dict):
                    pose = wp.get('pose', [])
                    use_gripper = wp.get('use_gripper', False)
                    name = wp.get('name', f'Waypoint {i+1}')
                else:
                    pose = wp if isinstance(wp, list) else []
                    use_gripper = self.use_gripper_list[i] if i < len(self.use_gripper_list) else False
                    name = f'Waypoint {i+1}'
                
                if len(pose) < 6:
                    self.log_signal.emit(f"✗ Invalid waypoint {i+1}: {pose}")
                    self.finished.emit(False)
                    return
                
                use_joint = wp.get('joints', False) if isinstance(wp, dict) else False
                r = self.waypoint_randomness
                # 에피소드 실행 시 웨이포인트마다 난수 부여 → 동일 에피소드라도 매번 다른 궤적
                if r > 0:
                    if use_joint:
                        # 관절: 각 J1~J6에 ±r (°) 난수
                        pose = [float(p) + np.random.uniform(-r, r) for p in pose[:6]]
                    else:
                        # 직교: x,y,z ±r (mm), rx,ry,rz ±r (°)
                        pose = [
                            pose[0] + np.random.uniform(-r, r),
                            pose[1] + np.random.uniform(-r, r),
                            pose[2] + np.random.uniform(-r, r),
                            pose[3] + np.random.uniform(-r, r),
                            pose[4] + np.random.uniform(-r, r),
                            pose[5] + np.random.uniform(-r, r),
                        ]
                
                x, y, z, rx, ry, rz = pose[:6]
                if use_joint:
                    self.log_signal.emit(f"Moving to {name}: J1={x:.1f}° ... J6={rz:.1f}°" + (f" (난수 ±{r})" if r > 0 else ""))
                    success = self.robot.move_j(x, y, z, rx, ry, rz, coordinate_mode=1, use_waypoint=False)
                else:
                    self.log_signal.emit(f"Moving to {name}: ({x:.1f}, {y:.1f}, {z:.1f}) mm" + (f" (난수 ±{r})" if r > 0 else ""))
                    success = self.robot.move_j(x, y, z, rx, ry, rz, coordinate_mode=0, use_waypoint=False)
                if not success:
                    self.log_signal.emit(f"✗ Failed to reach {name}")
                    self.finished.emit(False)
                    return
                
                self.robot.wait_for_motion_complete()
                time.sleep(0.05)  # 연속 명령 간 짧은 간격
                self.waypoint_reached.emit(i)
                
                # Log status at waypoint (joints + EE position)
                try:
                    current_pose = self.robot.get_current_pose_from_feedback()
                    joints = self._get_joints_from_feedback()
                    if current_pose and joints:
                        radius = np.sqrt(current_pose[0]**2 + current_pose[1]**2)
                        self.log_signal.emit(
                            f"  ✓ Reached {name}:\n"
                            f"    Joints: J1={joints[0]:.1f}° J2={joints[1]:.1f}° J3={joints[2]:.1f}° "
                            f"J4={joints[3]:.1f}° J5={joints[4]:.1f}° J6={joints[5]:.1f}°\n"
                            f"    EE: ({current_pose[0]:.1f}, {current_pose[1]:.1f}, {current_pose[2]:.1f}) mm, "
                            f"Radius: {radius:.1f} mm"
                        )
                except Exception:
                    pass
                
                if use_gripper:
                    self.log_signal.emit(f"  → Gripper action at {name}")
                    # Gripper control would be handled by GUI
            
            self.log_signal.emit("✅ Waypoint sequence complete")
            self.finished.emit(True)
        except Exception as e:
            self.log_signal.emit(f"Error: {e}")
            self.finished.emit(False)


class MoveToPoseWorker(QThread):
    """포즈 한 번 이동 (카메라 클릭→이동 테스트 등)."""
    finished = pyqtSignal(bool)
    
    def __init__(self, robot, x, y, z, rx=180, ry=0, rz=0):
        super().__init__()
        self.robot = robot
        self.x, self.y, self.z = x, y, z
        self.rx, self.ry, self.rz = rx, ry, rz
    
    def run(self):
        ok = False
        try:
            if self.robot:
                ok = self.robot.move_j(self.x, self.y, self.z, self.rx, self.ry, self.rz,
                                       coordinate_mode=0, use_waypoint=False)
        except Exception:
            pass
        self.finished.emit(ok)


class JointMoveWorker(QThread):
    """Worker for joint-space MovJ. GUI 블로킹 없이 관절 이동 실행."""
    finished = pyqtSignal(bool)
    log_signal = pyqtSignal(str)
    
    def __init__(self, robot, j1, j2, j3, j4, j5, j6):
        super().__init__()
        self.robot = robot
        self.joints = (j1, j2, j3, j4, j5, j6)
    
    def run(self):
        ok = False
        try:
            if not self.robot or not self.robot.connected:
                self.log_signal.emit("로봇 미연결")
                self.finished.emit(False)
                return
            j1, j2, j3, j4, j5, j6 = self.joints
            ok = self.robot.move_j(j1, j2, j3, j4, j5, j6, coordinate_mode=1, use_waypoint=False)
            if not ok:
                msg = getattr(self.robot, 'last_move_response', None) or "MovJ 거부됨"
                self.log_signal.emit(f"로봇 응답: {msg}")
                self.finished.emit(False)
                return
            self.robot.wait_for_motion_complete()
            self.finished.emit(True)
        except Exception as e:
            self.log_signal.emit(f"관절 이동 오류: {e}")
            self.finished.emit(False)


class KeyboardMoveWorker(QThread):
    """Worker for keyboard move. 상대 이동(RelMovJUser) 먼저 시도 → IK 꺾인 자세에서도 옆/Z 이동 가능."""
    finished = pyqtSignal(bool)
    
    def __init__(self, robot, dx, dy, dz, x, y, z, rx, ry, rz, try_lift_fallback=False, pose=None, step_mm=5.0):
        super().__init__()
        self.robot = robot
        self.dx, self.dy, self.dz = dx, dy, dz
        self.x, self.y, self.z = x, y, z
        self.rx, self.ry, self.rz = rx, ry, rz
        self.try_lift_fallback = try_lift_fallback
        self.pose = pose
        self.step_mm = step_mm
    
    def run(self):
        ok = False
        try:
            # 시작 위치 (이동 전)
            start_pose = self.robot.get_current_pose_from_feedback() if self.robot else None
            # 1) 상대 이동 먼저 시도 — IK 절대 좌표 대신 오프셋만 보냄
            ok = self.robot.move_j_relative(self.dx, self.dy, self.dz, 0, 0, 0, velocity=30)
            if not ok:
                ok = self.robot.move_j(self.x, self.y, self.z, self.rx, self.ry, self.rz,
                                       coordinate_mode=0, use_waypoint=False)
            if not ok and self.try_lift_fallback and self.pose is not None and len(self.pose) >= 6:
                x, y, z = self.pose[0], self.pose[1], self.pose[2]
                rx, ry, rz = self.pose[3], self.pose[4], self.pose[5]
                for scale in (0.95, 0.90, 0.85):
                    nx, ny, nz = x * scale, y * scale, z + self.step_mm
                    if self.robot.move_j(nx, ny, nz, rx, ry, rz, coordinate_mode=0, use_waypoint=False):
                        ok = True
                        break
            # 명령은 수락됐지만 실제로 안 움직이는 경우: 0.5초 후 위치 비교
            if ok and start_pose and len(start_pose) >= 3:
                time.sleep(0.5)
                now = self.robot.get_current_pose_from_feedback() if self.robot else None
                if now and len(now) >= 3:
                    dist = ((now[0] - start_pose[0])**2 + (now[1] - start_pose[1])**2 + (now[2] - start_pose[2])**2) ** 0.5
                    if dist < 2.0:
                        ok = False
                        self.robot.last_move_response = "로봇 미동작 (명령 수락됐으나 실행 안 됨 — TCP/조인트 모드 또는 티치펜던트 확인)"
        except Exception:
            pass
        self.finished.emit(ok)


class PickPlaceWorker(QThread):
    """Worker thread for pick and place operations"""
    finished = pyqtSignal(bool)
    log_signal = pyqtSignal(str)
    
    def __init__(self, pnp, pick_name, place_name, use_gripper=True, go_direct_to_place=False):
        super().__init__()
        self.pnp = pnp
        self.pick_name = pick_name
        self.place_name = place_name
        self.use_gripper = use_gripper
        self.go_direct_to_place = go_direct_to_place
    
    def run(self):
        try:
            result = self.pnp.execute_pick_and_place(
                self.pick_name, self.place_name,
                use_gripper=self.use_gripper,
                go_direct_to_place=self.go_direct_to_place
            )
            self.finished.emit(result)
        except Exception as e:
            self.log_signal.emit(f"Error: {e}")
            self.finished.emit(False)


class MergedPickPlaceGUI(QMainWindow):
    """Merged GUI: Camera-based pick/place + Pose control + Waypoints"""
    
    def __init__(self):
        super().__init__()
        self.robot = None
        self.gripper = None
        self.pnp = None
        self.config = None
        
        # Camera
        self.camera = None
        self.camera_thread = None
        self.camera_active = False
        
        # Camera calibration
        self.camera_matrix = None
        self.dist_coeffs = None
        self.transform_matrix = None
        self.translation_offset = None
        
        # Selected position
        self.selected_pixel_pos = None
        self.selected_robot_pos = None
        self.selection_mode = None  # 'pick', 'place', or None
        
        # Pick/Place locations
        self.pick_locations = {}
        self.place_locations = {}
        
        # Waypoints
        self.waypoints = []  # List of dict: {'name': str, 'pose': [x,y,z,rx,ry,rz], 'use_gripper': bool}
        self.waypoint_worker = None
        
        # Status logging timer
        self.status_log_timer = QTimer()
        self.status_log_timer.timeout.connect(self.log_robot_status)
        self.status_log_interval = 3000  # 3 seconds
        self.status_log_enabled = False
        
        # Episode recording
        self.episode_recording = False
        self.episode_data = []  # list of {t, pose, joints, gripper}
        self.episode_timer = QTimer()
        self.episode_timer.timeout.connect(self._episode_record_sample)
        self.episode_sample_interval = 100  # ms
        
        # Keyboard control step (mm)
        self.keyboard_step_mm = 5.0
        self.keyboard_move_busy = False
        self.keyboard_cooldown_timer = QTimer()
        self.keyboard_cooldown_timer.setSingleShot(True)
        self.keyboard_cooldown_timer.timeout.connect(self._keyboard_cooldown_done)
        self.keyboard_move_worker = None
        self._click_move_worker = None
        
        try:
            self.init_ui()
            self.load_config()
            self.load_camera_calibration()
            self.load_transform_matrix()
            if CAMERA_AVAILABLE:
                self.init_camera()
        except Exception as e:
            print(f"Error initializing GUI: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def init_ui(self):
        self.setWindowTitle("Dobot E6 Pick & Place - Merged GUI (Camera + Pose + Waypoints)")
        self.setGeometry(100, 100, 1600, 950)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Left: Camera view
        left_layout = QVBoxLayout()
        self.camera_label = QLabel("Camera not initialized")
        self.camera_label.setMinimumSize(640, 480)
        self.camera_label.setMaximumSize(640, 480)
        self.camera_label.setStyleSheet("border: 2px solid gray; background-color: black; color: white;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.mousePressEvent = self.on_camera_click
        left_layout.addWidget(self.camera_label)
        
        camera_control = QHBoxLayout()
        self.start_camera_btn = QPushButton("Start Camera")
        self.start_camera_btn.clicked.connect(self.start_camera)
        self.stop_camera_btn = QPushButton("Stop Camera")
        self.stop_camera_btn.clicked.connect(self.stop_camera)
        self.stop_camera_btn.setEnabled(False)
        camera_control.addWidget(self.start_camera_btn)
        camera_control.addWidget(self.stop_camera_btn)
        left_layout.addLayout(camera_control)
        
        pos_select = QHBoxLayout()
        self.select_pick_pos_btn = QPushButton("Select Pick Position")
        self.select_pick_pos_btn.clicked.connect(lambda: self.set_selection_mode('pick'))
        self.select_place_pos_btn = QPushButton("Select Place Position")
        self.select_place_pos_btn.clicked.connect(lambda: self.set_selection_mode('place'))
        pos_select.addWidget(self.select_pick_pos_btn)
        pos_select.addWidget(self.select_place_pos_btn)
        left_layout.addLayout(pos_select)
        
        # 카메라 클릭 → 즉시 이동 테스트 (좌표 찍으면 바로 이동)
        click_move_layout = QHBoxLayout()
        self.click_to_move_test_cb = QCheckBox("클릭 시 즉시 이동 테스트")
        self.click_to_move_test_cb.setToolTip("체크 후 카메라 화면 클릭 시 해당 좌표로 바로 이동. 로봇 연결 + 변환 매트릭스 필요.")
        click_move_layout.addWidget(self.click_to_move_test_cb)
        click_move_layout.addWidget(QLabel("Z (mm):"))
        self.click_test_z_spinbox = QDoubleSpinBox()
        self.click_test_z_spinbox.setRange(0, 600)
        self.click_test_z_spinbox.setValue(150)
        self.click_test_z_spinbox.setSuffix(" mm")
        self.click_test_z_spinbox.setMaximumWidth(90)
        click_move_layout.addWidget(self.click_test_z_spinbox)
        left_layout.addLayout(click_move_layout)
        
        main_layout.addLayout(left_layout)
        
        # Right: Control panels
        right_layout = QVBoxLayout()
        
        title = QLabel("Dobot E6 Controller")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(title)
        
        # Connection
        right_layout.addWidget(self._create_connection_panel())
        
        # Tabs for different modes
        tabs = QTabWidget()
        
        # Tab 1: Pick & Place
        pick_place_tab = QWidget()
        pick_place_layout = QVBoxLayout(pick_place_tab)
        pick_place_layout.addWidget(self._create_pick_place_panel())
        pick_place_layout.addWidget(self._create_pick_place_options_panel())
        tabs.addTab(pick_place_tab, "📦 Pick & Place")
        
        # Tab 2: Pose Control
        pose_tab = QWidget()
        pose_layout = QVBoxLayout(pose_tab)
        pose_layout.addWidget(self._create_pose_panel())
        tabs.addTab(pose_tab, "🎯 Pose Control")
        
        # Tab 3: Waypoints
        waypoint_tab = QWidget()
        waypoint_layout = QVBoxLayout(waypoint_tab)
        waypoint_layout.addWidget(self._create_waypoint_panel())
        tabs.addTab(waypoint_tab, "📍 Waypoints")
        
        right_layout.addWidget(tabs)
        
        # Actions
        right_layout.addWidget(self._create_action_panel())
        
        # Episode recording & Home pose
        right_layout.addWidget(self._create_episode_home_panel())
        
        # Keyboard control hint
        self.keyboard_hint_label = QLabel("⌨️ 키보드: ↑↓ Y ± | ←→ X ± | PgUp/PgDn 또는 [ ] Z ± | G/R 그리퍼 | H 홈")
        self.keyboard_hint_label.setStyleSheet("color: gray; font-size: 10px;")
        right_layout.addWidget(self.keyboard_hint_label)
        
        # Status
        right_layout.addWidget(self._create_status_panel())
        
        # Log
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(180)
        right_layout.addWidget(QLabel("Log:"))
        right_layout.addWidget(self.log_text)
        
        main_layout.addLayout(right_layout)
        self.setFocusPolicy(Qt.StrongFocus)
        self.log("GUI initialized")
    
    def _create_connection_panel(self):
        group = QGroupBox("🔌 Robot Connection")
        layout = QGridLayout()
        layout.addWidget(QLabel("IP:"), 0, 0)
        self.ip_input = QLineEdit("192.168.5.1")
        layout.addWidget(self.ip_input, 0, 1)
        layout.addWidget(QLabel("Port:"), 0, 2)
        self.port_input = QLineEdit("29999")
        self.port_input.setMaximumWidth(70)
        layout.addWidget(self.port_input, 0, 3)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_robot)
        layout.addWidget(self.connect_btn, 0, 4)
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_robot)
        self.disconnect_btn.setEnabled(False)
        layout.addWidget(self.disconnect_btn, 0, 5)
        self.reenable_robot_btn = QPushButton("🔄 로봇 재활성화")
        self.reenable_robot_btn.setToolTip("명령이 안 들어갈 때 티치펜던트 대신 한 번 눌러 보세요 (EnableRobot)")
        self.reenable_robot_btn.clicked.connect(self.reenable_robot)
        self.reenable_robot_btn.setEnabled(False)
        layout.addWidget(self.reenable_robot_btn, 1, 0, 1, 2)
        self.connection_status = QLabel("⚪ Disconnected")
        self.connection_status.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.connection_status, 1, 2, 1, 4)
        group.setLayout(layout)
        return group
    
    def _create_pick_place_panel(self):
        group = QGroupBox("📦 Pick & Place Locations")
        layout = QHBoxLayout()
        
        pick_layout = QVBoxLayout()
        pick_layout.addWidget(QLabel("Pick Object:"))
        self.object_combo = QComboBox()
        pick_layout.addWidget(self.object_combo)
        layout.addLayout(pick_layout)
        
        place_layout = QVBoxLayout()
        place_layout.addWidget(QLabel("Place Destination:"))
        self.destination_combo = QComboBox()
        place_layout.addWidget(self.destination_combo)
        layout.addLayout(place_layout)
        
        group.setLayout(layout)
        return group
    
    def _create_pick_place_options_panel(self):
        group = QGroupBox("🔧 Options")
        layout = QVBoxLayout()
        self.use_gripper_at_pick_cb = QCheckBox("Gripper ON when picking")
        self.use_gripper_at_pick_cb.setChecked(True)
        layout.addWidget(self.use_gripper_at_pick_cb)
        self.go_direct_to_place_cb = QCheckBox("Go directly to place (skip home)")
        self.go_direct_to_place_cb.setChecked(True)
        layout.addWidget(self.go_direct_to_place_cb)
        group.setLayout(layout)
        return group
    
    def _create_pose_panel(self):
        group = QGroupBox("🎯 관절 각도 제어 (J1~J6, °)")
        layout = QGridLayout()
        
        self.j1_input = QDoubleSpinBox()
        self.j1_input.setRange(-360, 360)
        self.j1_input.setValue(0)
        self.j1_input.setSuffix(" °")
        self.j2_input = QDoubleSpinBox()
        self.j2_input.setRange(-360, 360)
        self.j2_input.setValue(0)
        self.j2_input.setSuffix(" °")
        self.j3_input = QDoubleSpinBox()
        self.j3_input.setRange(-360, 360)
        self.j3_input.setValue(0)
        self.j3_input.setSuffix(" °")
        self.j4_input = QDoubleSpinBox()
        self.j4_input.setRange(-360, 360)
        self.j4_input.setValue(0)
        self.j4_input.setSuffix(" °")
        self.j5_input = QDoubleSpinBox()
        self.j5_input.setRange(-360, 360)
        self.j5_input.setValue(0)
        self.j5_input.setSuffix(" °")
        self.j6_input = QDoubleSpinBox()
        self.j6_input.setRange(-360, 360)
        self.j6_input.setValue(0)
        self.j6_input.setSuffix(" °")
        
        layout.addWidget(QLabel("J1:"), 0, 0)
        layout.addWidget(self.j1_input, 0, 1)
        layout.addWidget(QLabel("J2:"), 0, 2)
        layout.addWidget(self.j2_input, 0, 3)
        layout.addWidget(QLabel("J3:"), 0, 4)
        layout.addWidget(self.j3_input, 0, 5)
        layout.addWidget(QLabel("J4:"), 1, 0)
        layout.addWidget(self.j4_input, 1, 1)
        layout.addWidget(QLabel("J5:"), 1, 2)
        layout.addWidget(self.j5_input, 1, 3)
        layout.addWidget(QLabel("J6:"), 1, 4)
        layout.addWidget(self.j6_input, 1, 5)
        
        self.get_joints_btn = QPushButton("현재 관절 읽기")
        self.get_joints_btn.clicked.connect(self.fill_joints_from_feedback)
        self.get_joints_btn.setEnabled(False)
        self.move_pose_btn = QPushButton("Move to Joint (관절 이동)")
        self.move_pose_btn.clicked.connect(self.move_to_pose)
        self.move_pose_btn.setEnabled(False)
        layout.addWidget(self.get_joints_btn, 2, 0, 1, 3)
        layout.addWidget(self.move_pose_btn, 2, 3, 1, 3)
        
        group.setLayout(layout)
        return group
    
    def _create_waypoint_panel(self):
        group = QGroupBox("📍 Waypoint Sequence")
        layout = QVBoxLayout()
        
        # Waypoint list
        self.waypoint_list = QListWidget()
        self.waypoint_list.setMaximumHeight(200)
        layout.addWidget(QLabel("Waypoints:"))
        layout.addWidget(self.waypoint_list)
        
        # Buttons
        btn_layout = QHBoxLayout()
        self.add_current_waypoint_btn = QPushButton("Add Current Position")
        self.add_current_waypoint_btn.clicked.connect(self.add_current_as_waypoint)
        self.add_current_waypoint_btn.setEnabled(False)
        btn_layout.addWidget(self.add_current_waypoint_btn)
        
        self.add_pose_waypoint_btn = QPushButton("Add from Joint")
        self.add_pose_waypoint_btn.clicked.connect(self.add_pose_as_waypoint)
        self.add_pose_waypoint_btn.setEnabled(False)
        btn_layout.addWidget(self.add_pose_waypoint_btn)
        
        self.edit_waypoint_btn = QPushButton("Edit Selected")
        self.edit_waypoint_btn.clicked.connect(self.edit_selected_waypoint)
        self.edit_waypoint_btn.setEnabled(False)
        btn_layout.addWidget(self.edit_waypoint_btn)
        
        self.delete_waypoint_btn = QPushButton("Delete Selected")
        self.delete_waypoint_btn.clicked.connect(self.delete_selected_waypoint)
        btn_layout.addWidget(self.delete_waypoint_btn)
        
        layout.addLayout(btn_layout)
        
        # Sequence control (진행 중에도 저장·추가·편집 가능)
        seq_layout = QHBoxLayout()
        self.run_sequence_btn = QPushButton("▶ Run Sequence")
        self.run_sequence_btn.clicked.connect(self.run_waypoint_sequence)
        self.run_sequence_btn.setEnabled(False)
        seq_layout.addWidget(self.run_sequence_btn)
        
        seq_layout.addWidget(QLabel("에피소드 난수 ±:"))
        self.waypoint_randomness_spin = QDoubleSpinBox()
        self.waypoint_randomness_spin.setRange(0.0, 10.0)
        self.waypoint_randomness_spin.setValue(0.0)
        self.waypoint_randomness_spin.setSingleStep(0.5)
        self.waypoint_randomness_spin.setSuffix(" °/mm")
        self.waypoint_randomness_spin.setToolTip("실행 시 각 웨이포인트에 ±이 값만큼 난수 부여 (관절 °, 직교 mm/°)")
        seq_layout.addWidget(self.waypoint_randomness_spin)
        
        self.clear_waypoints_btn = QPushButton("Clear All")
        self.clear_waypoints_btn.clicked.connect(self.clear_waypoints)
        seq_layout.addWidget(self.clear_waypoints_btn)
        
        layout.addLayout(seq_layout)
        layout.addWidget(QLabel("※ 진행 중에도 Waypoint 저장·추가·편집 가능"))
        
        # Save/Load
        file_layout = QHBoxLayout()
        self.save_waypoints_btn = QPushButton("Save Waypoints")
        self.save_waypoints_btn.clicked.connect(self.save_waypoints)
        file_layout.addWidget(self.save_waypoints_btn)
        
        self.load_waypoints_btn = QPushButton("Load Waypoints")
        self.load_waypoints_btn.clicked.connect(self.load_waypoints)
        file_layout.addWidget(self.load_waypoints_btn)
        
        layout.addLayout(file_layout)
        
        # Enable/disable edit button based on selection
        self.waypoint_list.itemSelectionChanged.connect(
            lambda: self.edit_waypoint_btn.setEnabled(len(self.waypoint_list.selectedItems()) > 0)
        )
        
        group.setLayout(layout)
        return group
    
    def _create_action_panel(self):
        group = QGroupBox("⚙️ Actions")
        layout = QHBoxLayout()
        
        self.home_btn = QPushButton("🏠 Home")
        self.home_btn.clicked.connect(self.go_home)
        self.home_btn.setEnabled(False)
        layout.addWidget(self.home_btn)
        
        self.pick_btn = QPushButton("📦 Pick Only")
        self.pick_btn.clicked.connect(self.pick_only)
        self.pick_btn.setEnabled(False)
        layout.addWidget(self.pick_btn)
        
        self.place_btn = QPushButton("📍 Place Only")
        self.place_btn.clicked.connect(self.place_only)
        self.place_btn.setEnabled(False)
        layout.addWidget(self.place_btn)
        
        self.execute_btn = QPushButton("▶️ Execute Pick-and-Place")
        self.execute_btn.clicked.connect(self.execute_pick_and_place)
        self.execute_btn.setEnabled(False)
        self.execute_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        layout.addWidget(self.execute_btn)
        
        self.estop_btn = QPushButton("🛑 E-STOP")
        self.estop_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        self.estop_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(self.estop_btn)
        
        group.setLayout(layout)
        return group
    
    def _create_episode_home_panel(self):
        """Episode recording + Home pose specification"""
        group = QGroupBox("📼 에피소드 기록 & 홈포즈")
        layout = QVBoxLayout()
        
        # Episode recording
        ep_layout = QHBoxLayout()
        ep_layout.addWidget(QLabel("에피소드:"))
        self.start_episode_btn = QPushButton("▶ 기록 시작")
        self.start_episode_btn.clicked.connect(self.start_episode_recording)
        self.start_episode_btn.setEnabled(False)
        self.stop_episode_btn = QPushButton("⏹ 기록 중지")
        self.stop_episode_btn.clicked.connect(self.stop_episode_recording)
        self.stop_episode_btn.setEnabled(False)
        self.save_episode_btn = QPushButton("💾 저장")
        self.save_episode_btn.clicked.connect(self.save_episode)
        self.save_episode_btn.setEnabled(False)
        ep_layout.addWidget(self.start_episode_btn)
        ep_layout.addWidget(self.stop_episode_btn)
        ep_layout.addWidget(self.save_episode_btn)
        layout.addLayout(ep_layout)
        self.episode_status_label = QLabel("기록 중: 꺼짐")
        self.episode_status_label.setStyleSheet("color: gray;")
        layout.addWidget(self.episode_status_label)
        
        # Home pose
        home_layout = QHBoxLayout()
        home_layout.addWidget(QLabel("홈포즈:"))
        self.set_home_btn = QPushButton("📍 현재 위치를 홈으로 지정")
        self.set_home_btn.clicked.connect(self.set_current_as_home)
        self.set_home_btn.setEnabled(False)
        home_layout.addWidget(self.set_home_btn)
        layout.addLayout(home_layout)
        
        # 로봇 일직선 (조인트 0 = 세워진 자세)
        self.straight_up_btn = QPushButton("⬆️ 로봇 일직선 (세우기)")
        self.straight_up_btn.setToolTip("조인트 0°로 이동 — 팔을 위로 세운 자세")
        self.straight_up_btn.clicked.connect(self.go_straight_up)
        self.straight_up_btn.setEnabled(False)
        layout.addWidget(self.straight_up_btn)
        
        group.setLayout(layout)
        return group
    
    def _create_status_panel(self):
        group = QGroupBox("📊 Status")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("EE Position:"), 0, 0)
        self.position_label = QLabel("Unknown")
        layout.addWidget(self.position_label, 0, 1)
        
        layout.addWidget(QLabel("Joints:"), 1, 0)
        self.joints_label = QLabel("Unknown")
        self.joints_label.setWordWrap(True)
        layout.addWidget(self.joints_label, 1, 1)
        
        layout.addWidget(QLabel("Gripper:"), 2, 0)
        self.gripper_label = QLabel("Released")
        layout.addWidget(self.gripper_label, 2, 1)
        
        layout.addWidget(QLabel("Operation:"), 3, 0)
        self.operation_label = QLabel("Idle")
        layout.addWidget(self.operation_label, 3, 1)
        
        # Status logging toggle
        status_log_layout = QHBoxLayout()
        self.status_log_cb = QCheckBox("Auto-log status (every 3s)")
        self.status_log_cb.setChecked(False)
        self.status_log_cb.stateChanged.connect(self.toggle_status_logging)
        status_log_layout.addWidget(self.status_log_cb)
        layout.addLayout(status_log_layout, 4, 0, 1, 2)
        
        group.setLayout(layout)
        return group
    
    # === Waypoint Methods ===
    
    def _ask_gripper_action(self):
        """그리퍼 0/1 선택 — QMessageBox.question이 Windows에서 튕길 수 있어 QDialog 사용"""
        d = QDialog(self)
        d.setWindowTitle("Gripper Action")
        layout = QVBoxLayout(d)
        layout.addWidget(QLabel("Activate gripper at this waypoint?"))
        btn_layout = QHBoxLayout()
        yes_btn = QPushButton("Yes (1)")
        no_btn = QPushButton("No (0)")
        result = [False]
        def on_yes():
            result[0] = True
            d.accept()
        def on_no():
            result[0] = False
            d.accept()
        yes_btn.clicked.connect(on_yes)
        no_btn.clicked.connect(on_no)
        btn_layout.addWidget(yes_btn)
        btn_layout.addWidget(no_btn)
        layout.addLayout(btn_layout)
        d.exec_()
        return result[0]
    
    def add_current_as_waypoint(self):
        """Add current robot joint angles as waypoint."""
        if not self.robot or not self.robot.connected:
            QMessageBox.warning(self, "Not Connected", "Connect robot first")
            return
        
        joints = self.get_current_joints_from_feedback()
        if not joints or len(joints) < 6:
            QMessageBox.warning(self, "Error", "Could not get current joint angles")
            return
        
        name, ok = QInputDialog.getText(self, "Waypoint Name", "Enter waypoint name:")
        if not ok or not name:
            return
        
        use_gripper = self._ask_gripper_action()
        
        self.waypoints.append({
            'name': name,
            'pose': [float(x) for x in joints[:6]],
            'joints': True,
            'use_gripper': use_gripper
        })
        self._update_waypoint_list()
        self.log(f"Added waypoint: {name} at joints (J1={joints[0]:.1f}° ... J6={joints[5]:.1f}°)")
    
    def add_pose_as_waypoint(self):
        """Add current panel joint angles (J1~J6) as waypoint."""
        name, ok = QInputDialog.getText(self, "Waypoint Name", "Enter waypoint name:")
        if not ok or not name:
            return
        
        use_gripper = self._ask_gripper_action()
        
        joints = [
            self.j1_input.value(),
            self.j2_input.value(),
            self.j3_input.value(),
            self.j4_input.value(),
            self.j5_input.value(),
            self.j6_input.value()
        ]
        
        self.waypoints.append({
            'name': name,
            'pose': [float(x) for x in joints],
            'joints': True,
            'use_gripper': use_gripper
        })
        self._update_waypoint_list()
        self.log(f"Added waypoint: {name} from joint input")
    
    def edit_selected_waypoint(self):
        items = self.waypoint_list.selectedItems()
        if not items:
            return
        
        idx = self.waypoint_list.row(items[0])
        if idx < 0 or idx >= len(self.waypoints):
            return
        
        wp = self.waypoints[idx]
        is_joint = wp.get('joints', False)
        p = wp['pose']
        
        dialog = QDialog(self)
        dialog.setWindowTitle("Edit Waypoint")
        layout = QFormLayout(dialog)
        
        name_edit = QLineEdit(wp['name'])
        layout.addRow("Name:", name_edit)
        
        if is_joint:
            j1_edit = QDoubleSpinBox()
            j1_edit.setRange(-360, 360)
            j1_edit.setValue(p[0])
            layout.addRow("J1 (°):", j1_edit)
            j2_edit = QDoubleSpinBox()
            j2_edit.setRange(-360, 360)
            j2_edit.setValue(p[1])
            layout.addRow("J2 (°):", j2_edit)
            j3_edit = QDoubleSpinBox()
            j3_edit.setRange(-360, 360)
            j3_edit.setValue(p[2])
            layout.addRow("J3 (°):", j3_edit)
            j4_edit = QDoubleSpinBox()
            j4_edit.setRange(-360, 360)
            j4_edit.setValue(p[3])
            layout.addRow("J4 (°):", j4_edit)
            j5_edit = QDoubleSpinBox()
            j5_edit.setRange(-360, 360)
            j5_edit.setValue(p[4])
            layout.addRow("J5 (°):", j5_edit)
            j6_edit = QDoubleSpinBox()
            j6_edit.setRange(-360, 360)
            j6_edit.setValue(p[5])
            layout.addRow("J6 (°):", j6_edit)
        else:
            x_edit = QDoubleSpinBox()
            x_edit.setRange(-1000, 1000)
            x_edit.setValue(p[0])
            layout.addRow("X (mm):", x_edit)
            y_edit = QDoubleSpinBox()
            y_edit.setRange(-1000, 1000)
            y_edit.setValue(p[1])
            layout.addRow("Y (mm):", y_edit)
            z_edit = QDoubleSpinBox()
            z_edit.setRange(0, 800)
            z_edit.setValue(p[2])
            layout.addRow("Z (mm):", z_edit)
            rx_edit = QDoubleSpinBox()
            rx_edit.setRange(-180, 180)
            rx_edit.setValue(p[3])
            layout.addRow("RX (°):", rx_edit)
            ry_edit = QDoubleSpinBox()
            ry_edit.setRange(-180, 180)
            ry_edit.setValue(p[4])
            layout.addRow("RY (°):", ry_edit)
            rz_edit = QDoubleSpinBox()
            rz_edit.setRange(-180, 180)
            rz_edit.setValue(p[5])
            layout.addRow("RZ (°):", rz_edit)
        
        gripper_cb = QCheckBox()
        gripper_cb.setChecked(wp.get('use_gripper', False))
        layout.addRow("Use Gripper:", gripper_cb)
        
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)
        
        if dialog.exec_() == QDialog.Accepted:
            if is_joint:
                new_pose = [j1_edit.value(), j2_edit.value(), j3_edit.value(),
                            j4_edit.value(), j5_edit.value(), j6_edit.value()]
                self.waypoints[idx] = {
                    'name': name_edit.text(),
                    'pose': new_pose,
                    'joints': True,
                    'use_gripper': gripper_cb.isChecked()
                }
            else:
                new_pose = [x_edit.value(), y_edit.value(), z_edit.value(),
                            rx_edit.value(), ry_edit.value(), rz_edit.value()]
                self.waypoints[idx] = {
                    'name': name_edit.text(),
                    'pose': new_pose,
                    'use_gripper': gripper_cb.isChecked()
                }
            self._update_waypoint_list()
            self.log(f"Edited waypoint: {name_edit.text()}")
    
    def delete_selected_waypoint(self):
        items = self.waypoint_list.selectedItems()
        if not items:
            return
        
        idx = self.waypoint_list.row(items[0])
        if 0 <= idx < len(self.waypoints):
            name = self.waypoints[idx]['name']
            del self.waypoints[idx]
            self._update_waypoint_list()
            self.log(f"Deleted waypoint: {name}")
    
    def clear_waypoints(self):
        if len(self.waypoints) == 0:
            return
        
        reply = QMessageBox.question(
            self, "Clear Waypoints", f"Delete all {len(self.waypoints)} waypoints?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.waypoints.clear()
            self._update_waypoint_list()
            self.log("Cleared all waypoints")
    
    def _update_waypoint_list(self):
        self.waypoint_list.clear()
        for i, wp in enumerate(self.waypoints):
            gripper_str = " [Grip]" if wp.get('use_gripper', False) else ""
            p = wp['pose']
            if wp.get('joints', False):
                item_text = f"{i+1}. {wp['name']}: J1={p[0]:.1f}° J2={p[1]:.1f}° ... J6={p[5]:.1f}°{gripper_str}"
            else:
                item_text = f"{i+1}. {wp['name']}: ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm{gripper_str}"
            self.waypoint_list.addItem(item_text)
        
        self.run_sequence_btn.setEnabled(bool(len(self.waypoints) > 0 and self.robot and self.robot.connected))
    
    def run_waypoint_sequence(self):
        if not self.robot or not self.robot.connected:
            QMessageBox.warning(self, "Not Connected", "Connect robot first")
            return
        
        if len(self.waypoints) == 0:
            QMessageBox.warning(self, "No Waypoints", "Add waypoints first")
            return
        
        r = self.waypoint_randomness_spin.value()
        self.log(f"Starting waypoint sequence ({len(self.waypoints)} waypoints)" + (f", 난수 ±{r}" if r > 0 else "") + "...")
        self.operation_label.setText("Running Sequence")
        self.run_sequence_btn.setEnabled(False)
        # 진행 중에도 Save/Load/Add/Edit/Delete 는 비활성화하지 않음 → 저장·추가 가능
        
        self.waypoint_worker = WaypointSequenceWorker(
            self.robot, self.waypoints, waypoint_randomness=r
        )
        self.waypoint_worker.log_signal.connect(self.log)
        self.waypoint_worker.waypoint_reached.connect(self.on_waypoint_reached)
        self.waypoint_worker.finished.connect(self.on_waypoint_sequence_finished)
        self.waypoint_worker.start()
    
    def on_waypoint_reached(self, idx):
        if 0 <= idx < len(self.waypoints):
            wp = self.waypoints[idx]
            if wp['use_gripper'] and self.gripper:
                self.gripper.grip()
                self.log(f"  → Gripper activated at {wp['name']}")
    
    def on_waypoint_sequence_finished(self, success):
        self.operation_label.setText("Idle")
        self.run_sequence_btn.setEnabled(True)
        if success:
            self.log("✅ Waypoint sequence completed")
        else:
            self.log("❌ Waypoint sequence failed")
    
    def _waypoints_to_json_serializable(self, wps):
        """numpy float64 등이 있으면 json.dump에서 튕김 → Python 기본 타입으로 변환"""
        out = []
        for wp in wps:
            p = wp.get('pose', [0]*6)
            pose_safe = []
            for i, x in enumerate(p[:6]):
                try:
                    pose_safe.append(float(x))
                except (TypeError, ValueError):
                    pose_safe.append(0.0)
            out.append({
                'name': str(wp.get('name', '')),
                'pose': pose_safe,
                'use_gripper': bool(wp.get('use_gripper', False)),
                **({'joints': True} if wp.get('joints') else {})
            })
        return out
    
    def save_waypoints(self):
        if len(self.waypoints) == 0:
            QMessageBox.information(self, "No Waypoints", "No waypoints to save")
            return
        
        # Windows 네이티브 파일 대화상자가 튕기는 경우가 있음 → Qt 대화상자 사용
        options = QFileDialog.Options()
        if sys.platform == 'win32':
            options |= QFileDialog.DontUseNativeDialog
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Waypoints", "", "JSON Files (*.json)", options=options
        )
        if not filename or not filename.strip():
            return
        filename = filename.strip()
        if not filename.endswith('.json'):
            filename += '.json'
        try:
            wps = list(self.waypoints)  # 스레드 안전: 복사본 사용
            data = self._waypoints_to_json_serializable(wps)
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            self.log(f"Saved {len(wps)} waypoints to {os.path.basename(filename)}")
            QMessageBox.information(self, "Success", f"Saved {len(wps)} waypoints")
        except Exception as e:
            import traceback
            self.log(f"Failed to save: {e}\n{traceback.format_exc()}")
            QMessageBox.critical(self, "Error", f"Failed to save:\n{e}")
    
    def load_waypoints(self):
        options = QFileDialog.Options()
        if sys.platform == 'win32':
            options |= QFileDialog.DontUseNativeDialog
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Waypoints", "", "JSON Files (*.json)", options=options
        )
        if filename:
            try:
                with open(filename, 'r', encoding='utf-8') as f:
                    loaded = json.load(f)
                if isinstance(loaded, list):
                    self.waypoints = []
                    # 에피소드 형식(많은 샘플)이면 샘플링: 50개 초과 시 5간격으로 줄임
                    is_episode = (len(loaded) > 0 and isinstance(loaded[0], dict) and 
                                  isinstance(loaded[0].get('joints'), (list, tuple)))
                    step = 1
                    if is_episode and len(loaded) > 50:
                        step = max(1, len(loaded) // 40)
                        self.log(f"에피소드 샘플링: {len(loaded)} → {len(range(0, len(loaded), step))} waypoints (간격 {step})")
                    indices = range(0, len(loaded), step)
                    for ii, i in enumerate(indices):
                        w = loaded[i]
                        wp = w if isinstance(w, dict) else {}
                        jlist = wp.get('joints')
                        if isinstance(jlist, (list, tuple)) and len(jlist) >= 6:
                            p = (list(jlist) + [0.0] * 6)[:6]
                            use_joint = True
                            use_grip = (wp.get('gripper') == 'gripping')
                            name = str(wp.get('name', f'Pt{ii+1}'))
                        else:
                            p = wp.get('pose', []) or []
                            p = (list(p) + [0.0] * 6)[:6]
                            use_joint = bool(wp.get('joints') if isinstance(wp.get('joints'), bool) else False)
                            use_grip = bool(wp.get('use_gripper', False))
                            name = str(wp.get('name', ''))
                        self.waypoints.append({
                            'name': name or f'WP{ii+1}',
                            'pose': [float(x) for x in p],
                            'use_gripper': use_grip,
                            **({'joints': True} if use_joint else {})
                        })
                    self._update_waypoint_list()
                    self.log(f"Loaded {len(self.waypoints)} waypoints from {os.path.basename(filename)}")
                    QMessageBox.information(self, "Success", f"Loaded {len(self.waypoints)} waypoints")
                else:
                    QMessageBox.warning(self, "Invalid Format", "Waypoint file must contain a list")
            except Exception as e:
                self.log(f"Failed to load: {e}")
                QMessageBox.critical(self, "Error", f"Failed to load:\n{e}")
    
    # === Camera Methods (from pick_place_gui.py) ===
    
    def init_camera(self):
        if not CAMERA_AVAILABLE:
            return
        try:
            parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
            calib_file = os.path.join(parent_dir, "hikrobot_calibration_20260126_143821.npz")
            calibration_file = calib_file if os.path.exists(calib_file) else None
            self.camera = HikRobotCamera(calibration_file=calibration_file)
            if self.camera.init_camera():
                self.log("Camera initialized")
            else:
                self.camera = None
        except Exception as e:
            self.log(f"Camera init error: {e}")
            self.camera = None
    
    def start_camera(self):
        if not CAMERA_AVAILABLE or not self.camera:
            QMessageBox.warning(self, "Camera Not Available", "Camera module not available")
            return
        
        if self.camera_active:
            return
        
        try:
            self.camera_thread = CameraThread(self.camera)
            self.camera_thread.frame_ready.connect(self.update_camera_frame)
            self.camera_thread.start()
            self.camera_active = True
            self.start_camera_btn.setEnabled(False)
            self.stop_camera_btn.setEnabled(True)
            self.log("Camera streaming started")
        except Exception as e:
            self.log(f"Failed to start camera: {e}")
    
    def stop_camera(self):
        if self.camera_thread:
            self.camera_thread.stop()
            self.camera_thread.wait()
            self.camera_thread = None
        self.camera_active = False
        self.start_camera_btn.setEnabled(True)
        self.stop_camera_btn.setEnabled(False)
        self.camera_label.setText("Camera stopped")
        self.log("Camera streaming stopped")
    
    def update_camera_frame(self, frame):
        try:
            display_frame = frame.copy()
            if self.selected_pixel_pos:
                x, y = self.selected_pixel_pos
                cv2.circle(display_frame, (x, y), 10, (255, 0, 0), 2)
                cv2.line(display_frame, (x - 15, y), (x + 15, y), (255, 0, 0), 2)
                cv2.line(display_frame, (x, y - 15), (x, y + 15), (255, 0, 0), 2)
            
            height, width, channel = display_frame.shape
            bytes_per_line = 3 * width
            q_image = QImage(display_frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            scaled_pixmap = pixmap.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.camera_label.setPixmap(scaled_pixmap)
        except Exception as e:
            print(f"Frame update error: {e}")
    
    def on_camera_click(self, event):
        if not self.camera_active:
            return
        
        x = event.x()
        y = event.y()
        pixmap = self.camera_label.pixmap()
        if not pixmap:
            return
        
        label_size = self.camera_label.size()
        pixmap_size = pixmap.size()
        scale_x = 640.0 / pixmap_size.width() if pixmap_size.width() > 0 else 1.0
        scale_y = 480.0 / pixmap_size.height() if pixmap_size.height() > 0 else 1.0
        offset_x = (label_size.width() - pixmap_size.width()) / 2
        offset_y = (label_size.height() - pixmap_size.height()) / 2
        
        pix_x = int((x - offset_x) * scale_x)
        pix_y = int((y - offset_y) * scale_y)
        pix_x = max(0, min(639, pix_x))
        pix_y = max(0, min(479, pix_y))
        
        self.selected_pixel_pos = (pix_x, pix_y)
        robot_pos = self.pixel_to_robot_3d(pix_x, pix_y)
        
        # 클릭 시 즉시 이동 테스트: 좌표 찍으면 바로 해당 위치로 이동
        if self.click_to_move_test_cb.isChecked():
            if not self.robot or not self.robot.connected:
                self.log("이동 테스트: 로봇을 먼저 연결하세요.")
                QMessageBox.warning(self, "이동 테스트", "로봇을 먼저 연결한 뒤 클릭하세요.")
                return
            if not robot_pos:
                self.log("이동 테스트: 카메라-로봇 변환 매트릭스가 없습니다. camera_robot_transform.json 을 불러오세요.")
                QMessageBox.warning(self, "이동 테스트", "변환 매트릭스가 없습니다.\n프로젝트 루트의 camera_robot_transform.json 이 필요합니다.")
                return
            rx_robot = float(robot_pos[0])
            ry_robot = float(robot_pos[1])
            z_test = self.click_test_z_spinbox.value()
            self.log(f"이동 테스트: 픽셀 ({pix_x}, {pix_y}) → 로봇 ({rx_robot:.1f}, {ry_robot:.1f}, {z_test:.1f}) mm 로 이동 중...")
            self._click_move_worker = MoveToPoseWorker(self.robot, rx_robot, ry_robot, z_test, 180, 0, 0)
            self._click_move_worker.finished.connect(self._on_click_move_finished)
            self._click_move_worker.start()
            return
        
        if not robot_pos:
            return
        if not self.selection_mode:
            return
        
        self.selected_robot_pos = robot_pos
        self.log(f"Selected: pixel ({pix_x}, {pix_y}) → robot ({robot_pos[0]:.1f}, {robot_pos[1]:.1f}, {robot_pos[2]:.1f})")
        
        z_height, ok = QInputDialog.getDouble(
            self, "Z Height", f"Adjust Z height:\n({robot_pos[0]:.1f}, {robot_pos[1]:.1f}, {robot_pos[2]:.1f})",
            value=robot_pos[2], min=0.0, max=600.0, decimals=1
        )
        
        if ok:
            self.selected_robot_pos = (robot_pos[0], robot_pos[1], z_height)
            name, ok = QInputDialog.getText(self, "Save Position", f"Name for {self.selection_mode} position:")
            if ok and name:
                if self.selection_mode == 'pick':
                    self._save_pick_position(name)
                elif self.selection_mode == 'place':
                    self._save_place_position(name)
    
    def _on_click_move_finished(self, success):
        if getattr(self, '_click_move_worker', None):
            try:
                self._click_move_worker.finished.disconnect(self._on_click_move_finished)
            except Exception:
                pass
            self._click_move_worker = None
        if success:
            self.log("이동 테스트 완료")
        else:
            self.log("이동 테스트 실패 (목표 위치/관절 한계 확인)")
            if self.robot and getattr(self.robot, 'last_move_response', ''):
                self.log(f"   로봇 응답: {self.robot.last_move_response[:120]}")
        self.update_status()
    
    def set_selection_mode(self, mode):
        self.selection_mode = mode
        if mode == 'pick':
            self.select_pick_pos_btn.setStyleSheet("background-color: #4CAF50; color: white;")
            self.select_place_pos_btn.setStyleSheet("")
        elif mode == 'place':
            self.select_place_pos_btn.setStyleSheet("background-color: #4CAF50; color: white;")
            self.select_pick_pos_btn.setStyleSheet("")
    
    def _save_pick_position(self, name):
        if not self.selected_robot_pos:
            return
        if 'positions' not in self.config:
            self.config['positions'] = {}
        if 'pick_locations' not in self.config['positions']:
            self.config['positions']['pick_locations'] = {}
        
        self.config['positions']['pick_locations'][name] = {
            'x': float(self.selected_robot_pos[0]),
            'y': float(self.selected_robot_pos[1]),
            'z': float(self.selected_robot_pos[2]),
            'rx': 180, 'ry': 0, 'rz': 0
        }
        self.object_combo.addItem(name)
        self.pick_locations[name] = self.config['positions']['pick_locations'][name]
        self._save_config()
        self.log(f"Saved pick position: {name}")
        self.selection_mode = None
        self.select_pick_pos_btn.setStyleSheet("")
    
    def _save_place_position(self, name):
        if not self.selected_robot_pos:
            return
        if 'positions' not in self.config:
            self.config['positions'] = {}
        if 'place_locations' not in self.config['positions']:
            self.config['positions']['place_locations'] = {}
        
        self.config['positions']['place_locations'][name] = {
            'x': float(self.selected_robot_pos[0]),
            'y': float(self.selected_robot_pos[1]),
            'z': float(self.selected_robot_pos[2]),
            'rx': 180, 'ry': 0, 'rz': 0
        }
        self.destination_combo.addItem(name)
        self.place_locations[name] = self.config['positions']['place_locations'][name]
        self._save_config()
        self.log(f"Saved place position: {name}")
        self.selection_mode = None
        self.select_place_pos_btn.setStyleSheet("")
    
    def pixel_to_robot_3d(self, pixel_x, pixel_y, z_height=None):
        if self.transform_matrix is None:
            return None
        try:
            pixel_homogeneous = np.array([[pixel_x], [pixel_y], [1.0]])
            robot_2d = self.transform_matrix @ pixel_homogeneous
            robot_2d_normalized = robot_2d / robot_2d[2, 0]
            robot_x = float(robot_2d_normalized[0, 0])
            robot_y = float(robot_2d_normalized[1, 0])
            
            if z_height is None:
                z_height = self.translation_offset if self.translation_offset else 100.0
            
            return (robot_x, robot_y, float(z_height))
        except Exception as e:
            self.log(f"Transform error: {e}")
            return None
    
    # === Robot Control Methods ===
    
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
                
                config_dir = os.path.join(os.path.dirname(__file__), 'config')
                json_path = os.path.join(config_dir, 'robot_config.json')
                yaml_path = os.path.join(config_dir, 'robot_config.yaml')
                config_path = json_path if os.path.exists(json_path) else yaml_path
                
                self.pnp = PickAndPlace(self.robot, self.gripper, config_path)
                self._sync_pnp_locations()
                
                self.connection_status.setText("🟢 Connected")
                self.connection_status.setStyleSheet("color: green; font-weight: bold;")
                self.connect_btn.setEnabled(False)
                self.disconnect_btn.setEnabled(True)
                self.reenable_robot_btn.setEnabled(True)
                self.get_joints_btn.setEnabled(True)
                self.move_pose_btn.setEnabled(True)
                self.home_btn.setEnabled(True)
                self.pick_btn.setEnabled(True)
                self.place_btn.setEnabled(True)
                self.execute_btn.setEnabled(True)
                self.add_current_waypoint_btn.setEnabled(True)
                self.add_pose_waypoint_btn.setEnabled(True)
                self.run_sequence_btn.setEnabled(len(self.waypoints) > 0)
                self.start_episode_btn.setEnabled(True)
                self.set_home_btn.setEnabled(True)
                self.straight_up_btn.setEnabled(True)
                self.log("Robot connected")
                self.update_status()
                # Log initial status
                self.log_robot_status()
                # Start auto logging if enabled
                if self.status_log_cb.isChecked():
                    self.status_log_timer.start(self.status_log_interval)
            else:
                self.log("Connection failed")
                QMessageBox.warning(self, "Error", "Failed to connect")
        except Exception as e:
            self.log(f"Error: {e}")
            QMessageBox.critical(self, "Error", str(e))
    
    def reenable_robot(self):
        """로봇 재활성화 (EnableRobot) — 명령이 안 들어갈 때 사용"""
        if not self.robot or not self.robot.connected:
            QMessageBox.warning(self, "연결 없음", "로봇을 먼저 연결하세요.")
            return
        try:
            self.robot.enable_robot(sleep_after=0.5)
            self.log("로봇 재활성화 완료 (EnableRobot). 이동 명령을 다시 시도하세요.")
            QMessageBox.information(self, "재활성화", "로봇 재활성화 완료.\n이동/키보드 명령을 다시 시도하세요.")
        except Exception as e:
            self.log(f"재활성화 실패: {e}")
            QMessageBox.critical(self, "오류", str(e))
    
    def disconnect_robot(self):
        if self.robot:
            self.robot.disconnect()
            self.robot = None
            self.gripper = None
            self.pnp = None
        
        self.connection_status.setText("⚪ Disconnected")
        self.connection_status.setStyleSheet("")
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.get_joints_btn.setEnabled(False)
        self.move_pose_btn.setEnabled(False)
        self.home_btn.setEnabled(False)
        self.pick_btn.setEnabled(False)
        self.place_btn.setEnabled(False)
        self.execute_btn.setEnabled(False)
        self.add_current_waypoint_btn.setEnabled(False)
        self.add_pose_waypoint_btn.setEnabled(False)
        self.run_sequence_btn.setEnabled(False)
        self.start_episode_btn.setEnabled(False)
        self.stop_episode_btn.setEnabled(False)
        self.save_episode_btn.setEnabled(False)
        self.set_home_btn.setEnabled(False)
        self.straight_up_btn.setEnabled(False)
        if self.episode_recording:
            self.stop_episode_recording()
        self.status_log_timer.stop()
        self.position_label.setText("Unknown")
        self.joints_label.setText("Unknown")
        self.log("Robot disconnected")
    
    def move_to_pose(self):
        """Move to joint angles (J1~J6) set in the panel — worker thread로 실행해 GUI 멈춤 방지."""
        if not self.robot or not self.robot.connected:
            QMessageBox.warning(self, "Not Connected", "로봇을 먼저 연결하세요.")
            return
        
        j1 = self.j1_input.value()
        j2 = self.j2_input.value()
        j3 = self.j3_input.value()
        j4 = self.j4_input.value()
        j5 = self.j5_input.value()
        j6 = self.j6_input.value()
        
        self.log(f"MoveJ (Joint) to J1={j1:.1f}° J2={j2:.1f}° J3={j3:.1f}° J4={j4:.1f}° J5={j5:.1f}° J6={j6:.1f}°")
        self.move_pose_btn.setEnabled(False)
        
        self._joint_move_worker = JointMoveWorker(self.robot, j1, j2, j3, j4, j5, j6)
        self._joint_move_worker.log_signal.connect(self.log)
        self._joint_move_worker.finished.connect(self._on_joint_move_finished)
        self._joint_move_worker.start()
    
    def _on_joint_move_finished(self, ok):
        self.move_pose_btn.setEnabled(True)
        if ok:
            self.log("Motion complete")
            self.update_status()
            self.log_robot_status()
        else:
            self.log("MoveJ (Joint) failed — 로그의 '로봇 응답' 확인")
            QMessageBox.warning(
                self, "Move Failed",
                "관절 이동이 거부되었거나 실행되지 않았습니다.\n\n"
                "로그 창에서 '로봇 응답' 메시지를 확인하세요.\n"
                "· 'Control Mode Is Not Tcp' → 티치펜던트에서 TCP 모드로 전환\n"
                "· 로봇 재활성화 버튼 후 다시 시도"
            )
    
    def go_home(self):
        if not self.pnp:
            return
        self.log("Going home...")
        if self.pnp.go_home():
            self.log("Home reached")
            self.update_status()
            # Log status after movement
            self.log_robot_status()
        else:
            self.log("Home failed")
    
    def go_straight_up(self):
        """로봇을 일직선(세운) 자세로 이동 — 조인트 0° (J1~J6 모두 0), 팔이 위로 선 자세"""
        if not self.robot or not self.robot.connected:
            QMessageBox.warning(self, "Not Connected", "로봇을 먼저 연결하세요")
            return
        self.log("로봇 일직선(세우기) 이동 중... (Joint 0°)")
        self.straight_up_btn.setEnabled(False)
        try:
            # coordinate_mode=1: 조인트 공간, (0,0,0,0,0,0) = 팔을 위로 세운 자세
            ok = self.robot.move_j(0, 0, 0, 0, 0, 0, coordinate_mode=1, use_waypoint=False)
            if ok:
                self.robot.wait_for_motion_complete()
                self.log("로봇 일직선 도달")
                # 조인트 이동 후 포즈/TCP 명령이 안 들어가는 경우 대비: 현재 위치로 포즈 명령 1회 → TCP 모드 복구
                pose = self.robot.get_current_pose_from_feedback()
                if pose and len(pose) >= 6:
                    self.robot.move_j(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], coordinate_mode=0, use_waypoint=False)
                    self.log("TCP(포즈) 모드 복구 명령 전송")
                self.update_status()
                self.log_robot_status()
            else:
                self.log("일직선 이동 실패 (로봇 모드/한계 확인)")
                QMessageBox.warning(self, "이동 실패", "일직선 이동이 거부되었습니다.\n로봇이 TCP 모드인지, 관절 한계 내인지 확인하세요.")
        except Exception as e:
            self.log(f"오류: {e}")
            QMessageBox.critical(self, "오류", str(e))
        finally:
            self.straight_up_btn.setEnabled(True)
    
    def set_current_as_home(self):
        """Set current EE pose as home position and save to config"""
        if not self.robot or not self.robot.connected:
            QMessageBox.warning(self, "Not Connected", "Connect robot first")
            return
        pose = self.robot.get_current_pose_from_feedback()
        if not pose or len(pose) < 6:
            QMessageBox.warning(self, "Error", "Could not get current position")
            return
        if 'positions' not in self.config:
            self.config['positions'] = {}
        self.config['positions']['home'] = {
            'x': float(pose[0]), 'y': float(pose[1]), 'z': float(pose[2]),
            'rx': float(pose[3]), 'ry': float(pose[4]), 'rz': float(pose[5])
        }
        self._save_config()
        if self.pnp:
            self.pnp.home_pos = self.config['positions']['home']
        self.log(f"Home set to ({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}) mm")
        QMessageBox.information(self, "Home Set", "Current position saved as home.")
    
    def start_episode_recording(self):
        if not self.robot or not self.robot.connected:
            QMessageBox.warning(self, "Not Connected", "Connect robot first")
            return
        self.episode_recording = True
        self.episode_data = []
        self.episode_timer.start(self.episode_sample_interval)
        self.start_episode_btn.setEnabled(False)
        self.stop_episode_btn.setEnabled(True)
        self.save_episode_btn.setEnabled(False)
        self.episode_status_label.setText("기록 중: 켜짐")
        self.episode_status_label.setStyleSheet("color: green; font-weight: bold;")
        self.log("Episode recording started")
    
    def stop_episode_recording(self):
        self.episode_recording = False
        self.episode_timer.stop()
        self.start_episode_btn.setEnabled(bool(self.robot and self.robot.connected))
        self.stop_episode_btn.setEnabled(False)
        self.save_episode_btn.setEnabled(len(self.episode_data) > 0)
        self.episode_status_label.setText("기록 중: 꺼짐")
        self.episode_status_label.setStyleSheet("color: gray;")
        self.log(f"Episode recording stopped ({len(self.episode_data)} samples)")
    
    def _episode_record_sample(self):
        if not self.robot or not self.robot.connected or not self.episode_recording:
            return
        pose = self.robot.get_current_pose_from_feedback()
        joints = self.get_current_joints_from_feedback()
        gripper = "gripping" if (self.gripper and getattr(self.gripper, 'is_gripping', False)) else "released"
        t = time.time()
        self.episode_data.append({
            't': t,
            'pose': list(pose) if pose and len(pose) >= 6 else [0]*6,
            'joints': list(joints) if joints and len(joints) >= 6 else [0]*6,
            'gripper': gripper
        })
    
    def save_episode(self):
        if len(self.episode_data) == 0:
            QMessageBox.information(self, "No Data", "No episode data to save. Record first.")
            return
        config_dir = os.path.join(os.path.dirname(__file__), 'config')
        episodes_dir = os.path.join(config_dir, 'episodes')
        os.makedirs(episodes_dir, exist_ok=True)
        default_name = f"episode_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        options = QFileDialog.Options()
        if sys.platform == 'win32':
            options |= QFileDialog.DontUseNativeDialog
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Episode", os.path.join(episodes_dir, default_name), "JSON (*.json)", options=options
        )
        if not filename:
            return
        try:
            # Convert for JSON (numpy types -> float)
            out = []
            for s in self.episode_data:
                out.append({
                    't': float(s['t']),
                    'pose': [float(x) for x in s['pose']],
                    'joints': [float(x) for x in s['joints']],
                    'gripper': s['gripper']
                })
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(out, f, indent=2, ensure_ascii=False)
            self.log(f"Saved episode ({len(out)} samples) to {os.path.basename(filename)}")
            QMessageBox.information(self, "Saved", f"Episode saved: {len(out)} samples")
        except Exception as e:
            self.log(f"Failed to save episode: {e}")
            QMessageBox.critical(self, "Error", str(e))
    
    def pick_only(self):
        if not self.pnp:
            return
        object_name = self.object_combo.currentText()
        if not object_name:
            QMessageBox.warning(self, "No Selection", "Select pick location")
            return
        
        use_gripper = self.use_gripper_at_pick_cb.isChecked()
        self.log(f"Picking {object_name}... (Gripper: {'ON' if use_gripper else 'OFF'})")
        if self.pnp.pick_object(object_name, use_gripper=use_gripper):
            self.log(f"Picked {object_name}")
            self.gripper_label.setText("Gripping")
        else:
            self.log("Pick failed")
    
    def place_only(self):
        if not self.pnp:
            return
        location_name = self.destination_combo.currentText()
        if not location_name:
            QMessageBox.warning(self, "No Selection", "Select place location")
            return
        
        self.log(f"Placing at {location_name}...")
        if self.pnp.place_object(location_name):
            self.log(f"Placed at {location_name}")
            self.gripper_label.setText("Released")
            self.update_status()
            self.log_robot_status()
        else:
            self.log("Place failed")
    
    def execute_pick_and_place(self):
        if not self.pnp:
            return
        
        self._sync_pnp_locations()
        object_name = self.object_combo.currentText()
        location_name = self.destination_combo.currentText()
        
        if not object_name or not location_name:
            QMessageBox.warning(self, "No Selection", "Select pick and place locations")
            return
        
        use_gripper = self.use_gripper_at_pick_cb.isChecked()
        go_direct = self.go_direct_to_place_cb.isChecked()
        
        self.log(f"Executing: {object_name} → {location_name}")
        self.operation_label.setText("Executing")
        self.execute_btn.setEnabled(False)
        
        self.worker = PickPlaceWorker(self.pnp, object_name, location_name, use_gripper, go_direct)
        self.worker.finished.connect(self.on_pick_place_finished)
        self.worker.log_signal.connect(self.log)
        self.worker.start()
    
    def on_pick_place_finished(self, success):
        self.operation_label.setText("Complete" if success else "Failed")
        self.execute_btn.setEnabled(True)
        if success:
            self.log("✅ Pick-and-place complete")
        else:
            self.log("❌ Pick-and-place failed")
    
    def emergency_stop(self):
        if self.robot:
            self.robot.disable_robot()
        if self.gripper:
            self.gripper.emergency_release()
        self.log("🛑 E-STOP")
        QMessageBox.warning(self, "E-STOP", "Robot disabled")
    
    def get_current_joints_from_feedback(self):
        """Get current joint angles from feedback data"""
        if not self.robot or not self.robot.feed:
            return None
        try:
            data = self.robot.feed.feedBackData()
            if data is not None and len(data) > 0:
                if hex(data['TestValue'][0]) == '0x123456789abcdef':
                    joints = data['QActual'][0].tolist()  # [j1, j2, j3, j4, j5, j6] in radians
                    # Convert to degrees
                    joints_deg = [np.rad2deg(j) for j in joints]
                    return joints_deg
        except Exception as e:
            pass
        return None
    
    def fill_joints_from_feedback(self):
        """Fill J1~J6 panel from current robot joint angles."""
        if not self.robot or not self.robot.connected:
            self.log("로봇이 연결되지 않았습니다.")
            return
        if not self.robot.feed:
            self.log("피드백 연결 없음 — 관절 값을 읽을 수 없습니다.")
            return
        joints = self.get_current_joints_from_feedback()
        if not joints or len(joints) < 6:
            self.log("관절 값을 읽지 못했습니다. (피드백 데이터 확인)")
            return
        self.j1_input.setValue(joints[0])
        self.j2_input.setValue(joints[1])
        self.j3_input.setValue(joints[2])
        self.j4_input.setValue(joints[3])
        self.j5_input.setValue(joints[4])
        self.j6_input.setValue(joints[5])
        self.log(f"Filled joints: J1={joints[0]:.1f}° ... J6={joints[5]:.1f}°")
    
    def update_status(self):
        """Update status display with EE position and joint angles"""
        if not self.robot or not self.robot.connected:
            return
        try:
            # Get EE pose
            pose = self.robot.get_current_pose_from_feedback()
            if pose and len(pose) >= 6:
                self.position_label.setText(
                    f"X={pose[0]:.1f} Y={pose[1]:.1f} Z={pose[2]:.1f} mm | "
                    f"RX={pose[3]:.1f} RY={pose[4]:.1f} RZ={pose[5]:.1f} °"
                )
            
            # Get joint angles
            joints = self.get_current_joints_from_feedback()
            if joints and len(joints) >= 6:
                self.joints_label.setText(
                    f"J1={joints[0]:.1f}° J2={joints[1]:.1f}° J3={joints[2]:.1f}° | "
                    f"J4={joints[3]:.1f}° J5={joints[4]:.1f}° J6={joints[5]:.1f}°"
                )
        except Exception as e:
            pass
    
    def log_robot_status(self):
        """Log current robot status (joints + EE position)"""
        if not self.robot or not self.robot.connected:
            return
        
        try:
            pose = self.robot.get_current_pose_from_feedback()
            joints = self.get_current_joints_from_feedback()
            
            if pose and len(pose) >= 6 and joints and len(joints) >= 6:
                # Calculate radius for EE
                radius = np.sqrt(pose[0]**2 + pose[1]**2)
                
                status_msg = (
                    f"📊 Robot Status:\n"
                    f"   Joints: J1={joints[0]:.2f}° J2={joints[1]:.2f}° J3={joints[2]:.2f}° "
                    f"J4={joints[3]:.2f}° J5={joints[4]:.2f}° J6={joints[5]:.2f}°\n"
                    f"   EE Position: X={pose[0]:.2f} Y={pose[1]:.2f} Z={pose[2]:.2f} mm\n"
                    f"   EE Orientation: RX={pose[3]:.2f}° RY={pose[4]:.2f}° RZ={pose[5]:.2f}°\n"
                    f"   Radius: {radius:.2f} mm"
                )
                self.log(status_msg)
        except Exception as e:
            pass
    
    def toggle_status_logging(self, state):
        """Toggle automatic status logging"""
        if state == Qt.Checked:
            if self.robot and self.robot.connected:
                self.status_log_timer.start(self.status_log_interval)
                self.log("✅ Auto status logging enabled (every 3s)")
            else:
                self.status_log_cb.setChecked(False)
                QMessageBox.warning(self, "Not Connected", "Connect robot first to enable status logging")
        else:
            self.status_log_timer.stop()
            self.log("⏸️ Auto status logging disabled")
    
    def log(self, message):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {message}")
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())
    
    # === Config Methods ===
    
    def load_config(self):
        try:
            config_dir = os.path.join(os.path.dirname(__file__), 'config')
            json_path = os.path.join(config_dir, 'robot_config.json')
            yaml_path = os.path.join(config_dir, 'robot_config.yaml')
            
            if os.path.exists(json_path):
                with open(json_path, 'r', encoding='utf-8') as f:
                    self.config = json.load(f)
            elif os.path.exists(yaml_path) and HAS_YAML:
                with open(yaml_path, 'r', encoding='utf-8') as f:
                    self.config = yaml.safe_load(f)
            else:
                self.config = {'positions': {'pick_locations': {}, 'place_locations': {}}}
            
            if 'positions' in self.config:
                self.pick_locations = self.config['positions'].get('pick_locations', {})
                self.place_locations = self.config['positions'].get('place_locations', {})
                for name in sorted(self.pick_locations.keys()):
                    self.object_combo.addItem(name)
                for name in sorted(self.place_locations.keys()):
                    self.destination_combo.addItem(name)
            
            self.log("Config loaded")
        except Exception as e:
            self.log(f"Config load error: {e}")
            self.config = {'positions': {'pick_locations': {}, 'place_locations': {}}}
    
    def _save_config(self):
        try:
            config_dir = os.path.join(os.path.dirname(__file__), 'config')
            os.makedirs(config_dir, exist_ok=True)
            json_path = os.path.join(config_dir, 'robot_config.json')
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(self.config, f, indent=2, ensure_ascii=False)
        except Exception as e:
            self.log(f"Config save error: {e}")
    
    def _sync_pnp_locations(self):
        if self.pnp and 'positions' in self.config:
            self.pnp.pick_locations = self.config['positions'].get('pick_locations', {})
            self.pnp.place_locations = self.config['positions'].get('place_locations', {})
            if 'home' in self.config['positions']:
                self.pnp.home_pos = self.config['positions']['home']
    
    def load_camera_calibration(self):
        try:
            parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
            calib_file = os.path.join(parent_dir, "hikrobot_calibration_20260126_143821.npz")
            if os.path.exists(calib_file):
                calib_data = np.load(calib_file)
                self.camera_matrix = calib_data['camera_matrix']
                self.dist_coeffs = calib_data['dist_coeffs']
                self.log("Camera calibration loaded")
        except Exception as e:
            self.log(f"Calibration load error: {e}")
    
    def load_transform_matrix(self):
        try:
            # merge.py 기준 2단계 상위 = 프로젝트 루트 (TCP-IP-Python-V4)
            parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            transform_file = os.path.join(parent_dir, "camera_robot_transform.json")
            if os.path.exists(transform_file):
                with open(transform_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    self.transform_matrix = np.array(data['transform_matrix'])
                    self.translation_offset = data.get('translation_offset', 0)
                    self.log("Transform matrix loaded")
        except Exception as e:
            self.log(f"Transform load error: {e}")
    
    def _keyboard_cooldown_done(self):
        self.keyboard_move_busy = False
    
    def _keyboard_lift_fallback(self, pose, step_mm):
        """Z만 올리기 실패 시: 반경을 줄인 뒤 Z를 올려서 재시도 (팔이 뻗은 낮은 자세에서 사용)"""
        x, y, z, rx, ry, rz = pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]
        radius = np.sqrt(x*x + y*y)
        if radius < 1:
            return False
        # 반경을 95% → 90% → 85%로 줄이면서 Z를 올린 목표 시도
        for scale in (0.95, 0.90, 0.85):
            nx = x * scale
            ny = y * scale
            nz = z + step_mm
            if self.robot.move_j(nx, ny, nz, rx, ry, rz, coordinate_mode=0, use_waypoint=False):
                self.log(f"⌨ Z 올리기: 반경 줄임({scale:.0%}) 후 ({nx:.1f}, {ny:.1f}, {nz:.1f})")
                return True
        return False
    
    def keyPressEvent(self, event):
        """Keyboard control: Arrow X/Y, PgUp/PgDn Z, G gripper, H home"""
        if not self.robot or not self.robot.connected:
            super().keyPressEvent(event)
            return
        # 키 반복(누르고 있음) 무시 → 한 번 누를 때마다 한 번만 이동
        if event.isAutoRepeat():
            event.accept()
            return
        key = event.key()
        step = self.keyboard_step_mm
        dx, dy, dz = 0.0, 0.0, 0.0
        if key == Qt.Key_Left:
            dx = -step
        elif key == Qt.Key_Right:
            dx = step
        elif key == Qt.Key_Up:
            dy = step
        elif key == Qt.Key_Down:
            dy = -step
        elif key in (Qt.Key_PageUp, Qt.Key_BracketRight):  # ] 또는 PgUp = Z+
            dz = step
        elif key in (Qt.Key_PageDown, Qt.Key_BracketLeft):  # [ 또는 PgDn = Z-
            dz = -step
        elif key == Qt.Key_G:
            if self.gripper:
                self.gripper.grip()
                self.gripper_label.setText("Gripping")
                self.log("Gripper ON (G)")
            super().keyPressEvent(event)
            return
        elif key == Qt.Key_R:
            if self.gripper:
                self.gripper.release()
                self.gripper_label.setText("Released")
                self.log("Gripper OFF (R)")
            super().keyPressEvent(event)
            return
        elif key == Qt.Key_H:
            self.go_home()
            super().keyPressEvent(event)
            return
        else:
            super().keyPressEvent(event)
            return
        if dx == 0 and dy == 0 and dz == 0:
            super().keyPressEvent(event)
            return
        # 이전 키보드 이동이 아직 처리 중이면 무시 (버벅임 방지)
        if self.keyboard_move_busy:
            event.accept()
            return
        pose = self.robot.get_current_pose_from_feedback()
        if not pose or len(pose) < 6:
            super().keyPressEvent(event)
            return
        x, y, z = pose[0] + dx, pose[1] + dy, pose[2] + dz
        rx, ry, rz = pose[3], pose[4], pose[5]
        self.keyboard_move_busy = True
        # 워커 스레드에서 move_j 실행 → GUI 먹통 방지 (move_j 내부 대기 때문에)
        self.keyboard_move_worker = KeyboardMoveWorker(
            self.robot, dx, dy, dz, x, y, z, rx, ry, rz,
            try_lift_fallback=(dz > 0), pose=pose if dz > 0 else None, step_mm=step
        )
        self.keyboard_move_worker.finished.connect(self._on_keyboard_move_finished)
        self.keyboard_move_worker.start()
        event.accept()
    
    def _on_keyboard_move_finished(self, ok):
        """키보드 이동 워커 완료 — 로그/상태 갱신, 잠금 해제"""
        if self.keyboard_move_worker:
            try:
                self.keyboard_move_worker.finished.disconnect(self._on_keyboard_move_finished)
            except Exception:
                pass
            self.keyboard_move_worker = None
        if ok:
            self.log("⌨ Move OK")
            self.update_status()
            self.keyboard_cooldown_timer.start(400)
        else:
            self.log("⌨ Move failed (로봇이 거부함 — 목표 위치/관절 한계 확인)")
            if self.robot and getattr(self.robot, 'last_move_response', ''):
                self.log(f"   로봇 응답: {self.robot.last_move_response[:120]}")
            self.keyboard_move_busy = False
        self.update_status()
    
    def closeEvent(self, event):
        if self.camera_active:
            self.stop_camera()
        if self.camera:
            self.camera.cleanup()
        if self.robot and self.robot.connected:
            reply = QMessageBox.question(
                self, "Exit", "Disconnect and exit?",
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
    gui = MergedPickPlaceGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
