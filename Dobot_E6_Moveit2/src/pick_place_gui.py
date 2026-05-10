#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PyQt5 GUI for Dobot E6 Pick and Place
"""

import sys
import os
import json

# Windows 콘솔 UTF-8 인코딩 설정 (안전한 방법)
if sys.platform == 'win32':
    try:
        import io
        # stdout이 이미 래핑되어 있지 않은 경우에만 설정
        if not isinstance(sys.stdout, io.TextIOWrapper) or (hasattr(sys.stdout, 'encoding') and sys.stdout.encoding.lower() != 'utf-8'):
            if hasattr(sys.stdout, 'buffer'):
                sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
            if hasattr(sys.stderr, 'buffer'):
                sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')
    except:
        pass  # 실패해도 계속 진행
try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

# Check if camera module is available
CAMERA_AVAILABLE = False
try:
    # Try importing camera_viewer from current directory first
    from camera_viewer import HikRobotCamera
    CAMERA_AVAILABLE = True
except ImportError:
    try:
        # Try importing from src directory
        import sys
        current_dir = os.path.dirname(os.path.abspath(__file__))
        sys.path.insert(0, current_dir)
        from camera_viewer import HikRobotCamera
        CAMERA_AVAILABLE = True
    except ImportError as e:
        CAMERA_AVAILABLE = False
        print(f"Warning: Camera module not available: {e}")
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QLineEdit,
                             QComboBox, QTextEdit, QGroupBox, QGridLayout, QMessageBox, QInputDialog, QCheckBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap, QPainter, QPen
import numpy as np
import cv2
import time
from typing import Optional, Tuple

# Add parent directories to path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, current_dir)  # For local imports
sys.path.insert(0, parent_dir)    # For dobot_api.py

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
        """Execute pick and place in thread"""
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


class PickPlaceGUI(QMainWindow):
    """Main GUI for Pick and Place application"""
    
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
        
        # Camera calibration and transform
        self.camera_matrix = None
        self.dist_coeffs = None
        self.transform_matrix = None
        self.translation_offset = None
        
        # Selected position from camera
        self.selected_pixel_pos = None  # (x, y) pixel coordinates
        self.selected_robot_pos = None  # (x, y, z) robot coordinates
        
        self.pick_locations = {}
        self.place_locations = {}
        
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
        """Initialize user interface"""
        self.setWindowTitle("Dobot E6 Pick-and-Place Control")
        self.setGeometry(100, 100, 1400, 900)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout (horizontal split: camera left, controls right)
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Left side: Camera view
        left_layout = QVBoxLayout()
        
        # Camera label
        self.camera_label = QLabel("Camera not initialized")
        self.camera_label.setMinimumSize(640, 480)
        self.camera_label.setMaximumSize(640, 480)
        self.camera_label.setStyleSheet("border: 2px solid gray; background-color: black; color: white;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.mousePressEvent = self.on_camera_click
        left_layout.addWidget(self.camera_label)
        
        # Camera controls
        camera_control_layout = QHBoxLayout()
        self.start_camera_btn = QPushButton("Start Camera")
        self.start_camera_btn.clicked.connect(self.start_camera)
        self.stop_camera_btn = QPushButton("Stop Camera")
        self.stop_camera_btn.clicked.connect(self.stop_camera)
        self.stop_camera_btn.setEnabled(False)
        camera_control_layout.addWidget(self.start_camera_btn)
        camera_control_layout.addWidget(self.stop_camera_btn)
        left_layout.addLayout(camera_control_layout)
        
        # Camera-based position selection
        camera_pos_layout = QHBoxLayout()
        self.select_pick_pos_btn = QPushButton("Select Pick Position")
        self.select_pick_pos_btn.clicked.connect(lambda: self.set_selection_mode('pick'))
        self.select_place_pos_btn = QPushButton("Select Place Position")
        self.select_place_pos_btn.clicked.connect(lambda: self.set_selection_mode('place'))
        camera_pos_layout.addWidget(self.select_pick_pos_btn)
        camera_pos_layout.addWidget(self.select_place_pos_btn)
        left_layout.addLayout(camera_pos_layout)
        
        # Calibration accuracy check button
        self.check_calibration_btn = QPushButton("Check Calibration Accuracy")
        self.check_calibration_btn.clicked.connect(self.check_calibration_accuracy)
        left_layout.addWidget(self.check_calibration_btn)
        
        self.selection_mode = None  # 'pick' or 'place'
        
        main_layout.addLayout(left_layout)
        
        # Right side: Control panels
        right_layout = QVBoxLayout()
        
        # Title
        title = QLabel("Dobot E6 Pick-and-Place Controller")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(title)
        
        # Connection panel
        right_layout.addWidget(self.create_connection_panel())
        
        # Control panels in horizontal layout
        control_layout = QHBoxLayout()
        control_layout.addWidget(self.create_object_panel())
        control_layout.addWidget(self.create_destination_panel())
        right_layout.addLayout(control_layout)
        
        # Pick/Place options (gripper, direct to place)
        right_layout.addWidget(self.create_pick_place_options_panel())
        
        # Action buttons
        right_layout.addWidget(self.create_action_panel())
        
        # Status panel
        right_layout.addWidget(self.create_status_panel())
        
        # Log window
        right_layout.addWidget(self.create_log_panel())
        
        main_layout.addLayout(right_layout)
        
        self.log("Application started")
    
    def create_connection_panel(self):
        """Create connection control panel"""
        group = QGroupBox("🔌 Robot Connection")
        layout = QGridLayout()
        
        # IP address
        layout.addWidget(QLabel("IP Address:"), 0, 0)
        self.ip_input = QLineEdit("192.168.5.1")
        layout.addWidget(self.ip_input, 0, 1)
        
        # Port
        layout.addWidget(QLabel("Dashboard Port:"), 0, 2)
        self.port_input = QLineEdit("29999")
        layout.addWidget(self.port_input, 0, 3)
        
        # Connect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_robot)
        layout.addWidget(self.connect_btn, 0, 4)
        
        # Disconnect button
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_robot)
        self.disconnect_btn.setEnabled(False)
        layout.addWidget(self.disconnect_btn, 0, 5)
        
        # Status indicator
        self.connection_status = QLabel("⚪ Disconnected")
        self.connection_status.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.connection_status, 1, 0, 1, 6)
        
        group.setLayout(layout)
        return group
    
    def create_object_panel(self):
        """Create object selection panel"""
        group = QGroupBox("📦 Pick Object")
        layout = QVBoxLayout()
        
        layout.addWidget(QLabel("Select object to pick:"))
        self.object_combo = QComboBox()
        layout.addWidget(self.object_combo)
        
        # Add custom position button
        add_pick_btn = QPushButton("Add Custom Position")
        add_pick_btn.clicked.connect(lambda: self.log("Custom pick position not yet implemented"))
        layout.addWidget(add_pick_btn)
        
        layout.addStretch()
        group.setLayout(layout)
        return group
    
    def create_destination_panel(self):
        """Create destination selection panel"""
        group = QGroupBox("📍 Place Destination")
        layout = QVBoxLayout()
        
        layout.addWidget(QLabel("Select destination:"))
        self.destination_combo = QComboBox()
        layout.addWidget(self.destination_combo)
        
        # Add custom position button
        add_place_btn = QPushButton("Add Custom Position")
        add_place_btn.clicked.connect(lambda: self.log("Custom place position not yet implemented"))
        layout.addWidget(add_place_btn)
        
        layout.addStretch()
        group.setLayout(layout)
        return group
    
    def create_pick_place_options_panel(self):
        """Create options for pick-and-place: gripper at pick, go direct to place"""
        group = QGroupBox("🔧 Pick-and-Place Options")
        layout = QVBoxLayout()
        
        self.use_gripper_at_pick_cb = QCheckBox("Gripper ON when picking (suction at pick)")
        self.use_gripper_at_pick_cb.setChecked(True)
        self.use_gripper_at_pick_cb.setToolTip("If unchecked: move to pick position and lift only, no suction (for testing or non-grip moves)")
        layout.addWidget(self.use_gripper_at_pick_cb)
        
        self.go_direct_to_place_cb = QCheckBox("Go directly to place (skip home between pick and place)")
        self.go_direct_to_place_cb.setChecked(True)
        self.go_direct_to_place_cb.setToolTip("If checked: after pick, move straight to place position. If unchecked: return home first then go to place.")
        layout.addWidget(self.go_direct_to_place_cb)
        
        group.setLayout(layout)
        return group
    
    def create_action_panel(self):
        """Create action button panel"""
        group = QGroupBox("⚙️ Actions")
        layout = QHBoxLayout()
        
        # Home button
        self.home_btn = QPushButton("🏠 Go Home")
        self.home_btn.clicked.connect(self.go_home)
        self.home_btn.setEnabled(False)
        layout.addWidget(self.home_btn)
        
        # Set current as home button
        self.set_home_btn = QPushButton("📍 Set Current as Home")
        self.set_home_btn.clicked.connect(self.set_current_as_home)
        self.set_home_btn.setEnabled(False)
        self.set_home_btn.setToolTip("Save current robot position as home position")
        layout.addWidget(self.set_home_btn)
        
        # Pick button
        self.pick_btn = QPushButton("📦 Pick Only")
        self.pick_btn.clicked.connect(self.pick_only)
        self.pick_btn.setEnabled(False)
        layout.addWidget(self.pick_btn)
        
        # Place button
        self.place_btn = QPushButton("📍 Place Only")
        self.place_btn.clicked.connect(self.place_only)
        self.place_btn.setEnabled(False)
        layout.addWidget(self.place_btn)
        
        # Execute button
        self.execute_btn = QPushButton("▶️ Execute Pick-and-Place")
        self.execute_btn.clicked.connect(self.execute_pick_and_place)
        self.execute_btn.setEnabled(False)
        self.execute_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        layout.addWidget(self.execute_btn)
        
        # Emergency stop
        self.estop_btn = QPushButton("🛑 EMERGENCY STOP")
        self.estop_btn.clicked.connect(self.emergency_stop)
        self.estop_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 10px;")
        layout.addWidget(self.estop_btn)
        
        group.setLayout(layout)
        return group
    
    def create_status_panel(self):
        """Create status display panel"""
        group = QGroupBox("📊 Status")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Current Position:"), 0, 0)
        self.position_label = QLabel("Unknown")
        layout.addWidget(self.position_label, 0, 1)
        
        layout.addWidget(QLabel("Gripper Status:"), 1, 0)
        self.gripper_label = QLabel("Released")
        layout.addWidget(self.gripper_label, 1, 1)
        
        layout.addWidget(QLabel("Operation Status:"), 2, 0)
        self.operation_label = QLabel("Idle")
        layout.addWidget(self.operation_label, 2, 1)
        
        group.setLayout(layout)
        return group
    
    def create_log_panel(self):
        """Create log display panel"""
        group = QGroupBox("📝 Log")
        layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        layout.addWidget(self.log_text)
        
        # Clear button
        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(lambda: self.log_text.clear())
        layout.addWidget(clear_btn)
        
        group.setLayout(layout)
        return group
    
    def load_config(self):
        """Load robot configuration"""
        try:
            # Try JSON first, then YAML
            # Get absolute path to config directory
            current_file = os.path.abspath(__file__)
            config_dir = os.path.join(os.path.dirname(current_file), '..', 'config')
            config_dir = os.path.normpath(config_dir)
            json_path = os.path.join(config_dir, 'robot_config.json')
            yaml_path = os.path.join(config_dir, 'robot_config.yaml')
            
            print(f"Looking for config in: {config_dir}")
            print(f"JSON path: {json_path} (exists: {os.path.exists(json_path)})")
            print(f"YAML path: {yaml_path} (exists: {os.path.exists(yaml_path)})")
            
            if os.path.exists(json_path):
                print("Loading JSON config...")
                with open(json_path, 'r', encoding='utf-8') as f:
                    self.config = json.load(f)
            elif os.path.exists(yaml_path):
                if HAS_YAML:
                    print("Loading YAML config...")
                    with open(yaml_path, 'r', encoding='utf-8') as f:
                        self.config = yaml.safe_load(f)
                else:
                    raise ImportError("PyYAML is required for YAML config files.\nInstall with: pip install PyYAML\nOr use JSON config file: robot_config.json")
            else:
                raise FileNotFoundError(f"Config file not found. Expected:\n{json_path}\nor\n{yaml_path}")
            
            # Load pick locations
            if 'positions' in self.config and 'pick_locations' in self.config['positions']:
                self.pick_locations = self.config['positions']['pick_locations']
                if self.pick_locations:
                    for name in sorted(self.pick_locations.keys()):
                        self.object_combo.addItem(name)
                    self.log(f"Loaded {len(self.pick_locations)} pick locations")
                else:
                    self.log("⚠️  Warning: No pick locations found in config")
            else:
                self.log("⚠️  Warning: Pick locations not found in config")
            
            # Load place locations
            if 'positions' in self.config and 'place_locations' in self.config['positions']:
                self.place_locations = self.config['positions']['place_locations']
                if self.place_locations:
                    for name in sorted(self.place_locations.keys()):
                        self.destination_combo.addItem(name)
                    self.log(f"Loaded {len(self.place_locations)} place locations")
                else:
                    self.log("⚠️  Warning: No place locations found in config")
            else:
                self.log("⚠️  Warning: Place locations not found in config")
            
            # Validate that we have locations
            if not self.pick_locations:
                self.log("⚠️  ERROR: No pick locations available. Please add locations to config file.")
            if not self.place_locations:
                self.log("⚠️  ERROR: No place locations available. Please add locations to config file.")
            
            self.log("Configuration loaded successfully")
            print("Configuration loaded successfully")
            
        except Exception as e:
            error_msg = f"Error loading config: {e}"
            print(error_msg)
            import traceback
            traceback.print_exc()
            self.log(error_msg)
            QMessageBox.warning(self, "Config Error", f"Failed to load configuration:\n{e}")
    
    def log(self, message):
        """Add message to log"""
        self.log_text.append(message)
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )
    
    def connect_robot(self):
        """Connect to robot"""
        ip = self.ip_input.text()
        port = int(self.port_input.text())
        
        self.log(f"Connecting to robot at {ip}:{port}...")
        
        try:
            self.robot = DobotE6Controller(ip=ip, dashboard_port=port)
            
            if self.robot.connect():
                self.gripper = SuctionGripper(self.robot, do_index=self.config['gripper']['do_index'])
                
                # Try JSON first, then YAML
                config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
                json_path = os.path.join(config_dir, 'robot_config.json')
                yaml_path = os.path.join(config_dir, 'robot_config.yaml')
                
                if os.path.exists(json_path):
                    config_path = json_path
                elif os.path.exists(yaml_path):
                    config_path = yaml_path
                else:
                    config_path = json_path  # Default to JSON
                
                self.pnp = PickAndPlace(self.robot, self.gripper, config_path)
                
                self.connection_status.setText("🟢 Connected")
                self.connection_status.setStyleSheet("color: green; font-weight: bold;")
                self.log("✓ Robot connected successfully")
                
                # Enable buttons
                self.connect_btn.setEnabled(False)
                self.disconnect_btn.setEnabled(True)
                self.home_btn.setEnabled(True)
                self.set_home_btn.setEnabled(True)
                self.pick_btn.setEnabled(True)
                self.place_btn.setEnabled(True)
                self.execute_btn.setEnabled(True)
            else:
                self.log("✗ Connection failed")
                QMessageBox.critical(self, "Connection Error", "Failed to connect to robot")
                
        except Exception as e:
            self.log(f"✗ Connection error: {e}")
            QMessageBox.critical(self, "Connection Error", f"Error:\n{e}")
    
    def disconnect_robot(self):
        """Disconnect from robot"""
        if self.robot:
            self.robot.disconnect()
            self.robot = None
            self.gripper = None
            self.pnp = None
            
        self.connection_status.setText("⚪ Disconnected")
        self.connection_status.setStyleSheet("")
        self.log("Robot disconnected")
        
        # Disable buttons
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.home_btn.setEnabled(False)
        self.set_home_btn.setEnabled(False)
        self.pick_btn.setEnabled(False)
        self.place_btn.setEnabled(False)
        self.execute_btn.setEnabled(False)
    
    def go_home(self):
        """Move robot to home position"""
        if not self.pnp:
            self.log("Robot not connected. Please connect robot first.")
            return
            
        self.log("Moving to home...")
        self.operation_label.setText("Going Home")
        
        # Get current position first
        if self.robot and self.robot.connected:
            current_pose = self.robot.get_current_pose_from_feedback()
            if current_pose:
                self.log(f"Current position: ({current_pose[0]:.1f}, {current_pose[1]:.1f}, {current_pose[2]:.1f})")
        
        if self.pnp.go_home():
            self.log("✓ Home position reached")
            self.operation_label.setText("Idle")
        else:
            self.log("✗ Failed to go home")
            self.log("⚠️  Try moving robot manually to a safe position")
            self.operation_label.setText("Error")
            QMessageBox.warning(self, "Home Failed", 
                              "Failed to move to home position.\n"
                              "The configured home position may be unreachable.\n\n"
                              "Please:\n"
                              "1. Move robot manually to a safe position\n"
                              "2. Use 'Set Current as Home' button to update home position")
    
    def set_current_as_home(self):
        """Set current robot position as home position"""
        if not self.robot or not self.robot.connected:
            QMessageBox.warning(self, "Robot Not Connected", "Please connect robot first.")
            return
        
        # Get current position
        current_pose = self.robot.get_current_pose_from_feedback()
        if not current_pose:
            QMessageBox.warning(self, "Failed", "Could not get current robot position.")
            return
        
        x, y, z, rx, ry, rz = current_pose
        
        # Confirm with user
        reply = QMessageBox.question(
            self,
            "Set Home Position",
            f"Set current position as home?\n\n"
            f"X: {x:.1f} mm\n"
            f"Y: {y:.1f} mm\n"
            f"Z: {z:.1f} mm\n"
            f"RX: {rx:.1f}°\n"
            f"RY: {ry:.1f}°\n"
            f"RZ: {rz:.1f}°",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # Update config
            self.config['positions']['home'] = {
                'x': float(x),
                'y': float(y),
                'z': float(z),
                'rx': float(rx),
                'ry': float(ry),
                'rz': float(rz)
            }
            
            # Save to file
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                'config',
                'robot_config.json'
            )
            
            try:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(self.config, f, indent=2, ensure_ascii=False)
                
                # Update pick_place_logic
                if self.pnp:
                    self.pnp.home_pos = self.config['positions']['home']
                
                self.log(f"✓ Home position updated: ({x:.1f}, {y:.1f}, {z:.1f}, {rx:.1f}, {ry:.1f}, {rz:.1f})")
                QMessageBox.information(self, "Success", "Home position has been updated!")
            except Exception as e:
                self.log(f"✗ Failed to save home position: {e}")
                QMessageBox.critical(self, "Error", f"Failed to save home position:\n{e}")
    
    def pick_only(self):
        """Pick object only"""
        if not self.pnp:
            self.log("Robot not connected. Please connect robot first.")
            return
            
        object_name = self.object_combo.currentText()
        if not object_name or object_name == "":
            self.log("⚠️  Please select a pick location from the dropdown")
            QMessageBox.warning(self, "No Selection", "Please select a pick location from the dropdown menu.")
            return
        
        use_gripper = self.use_gripper_at_pick_cb.isChecked()
        self.log(f"Picking {object_name}... (Gripper: {'ON' if use_gripper else 'OFF'})")
        self.operation_label.setText("Picking")
        
        if self.pnp.pick_object(object_name, use_gripper=use_gripper):
            self.log(f"✓ Picked {object_name}")
            self.gripper_label.setText("Gripping")
            self.operation_label.setText("Idle")
        else:
            self.log(f"✗ Pick failed")
            self.operation_label.setText("Error")
    
    def place_only(self):
        """Place object only"""
        if not self.pnp:
            self.log("Robot not connected. Please connect robot first.")
            return
            
        location_name = self.destination_combo.currentText()
        if not location_name or location_name == "":
            self.log("⚠️  Please select a place location from the dropdown")
            QMessageBox.warning(self, "No Selection", "Please select a place location from the dropdown menu.")
            return
            
        self.log(f"Placing at {location_name}...")
        self.operation_label.setText("Placing")
        
        if self.pnp.place_object(location_name):
            self.log(f"✓ Placed at {location_name}")
            self.gripper_label.setText("Released")
            self.operation_label.setText("Idle")
        else:
            self.log(f"✗ Place failed")
            self.operation_label.setText("Error")
    
    def execute_pick_and_place(self):
        """Execute full pick and place sequence"""
        if not self.pnp:
            self.log("Robot not connected. Please connect robot first.")
            return
        
        # Use current config so newly saved "pick block" / "place block" are known to pnp
        self._sync_pnp_locations()
            
        object_name = self.object_combo.currentText()
        location_name = self.destination_combo.currentText()
        
        # Ensure selected names exist in pnp (after sync they should)
        if self.pnp and (object_name not in self.pnp.pick_locations or location_name not in self.pnp.place_locations):
            missing = []
            if object_name not in self.pnp.pick_locations:
                missing.append(f"pick '{object_name}'")
            if location_name not in self.pnp.place_locations:
                missing.append(f"place '{location_name}'")
            self.log(f"⚠️  Unknown location: {', '.join(missing)}. Available pick: {list(self.pnp.pick_locations.keys())}, place: {list(self.pnp.place_locations.keys())}")
            QMessageBox.warning(self, "Unknown Location", 
                f"Selected position not found.\n\n"
                f"Pick '{object_name}' in list: {object_name in self.pnp.pick_locations if self.pnp else False}\n"
                f"Place '{location_name}' in list: {location_name in self.pnp.place_locations if self.pnp else False}\n\n"
                f"Pick locations: {list(self.pnp.pick_locations.keys()) if self.pnp else []}\n"
                f"Place locations: {list(self.pnp.place_locations.keys()) if self.pnp else []}")
            return
        
        if not object_name or object_name == "":
            self.log("⚠️  Please select a pick location from the dropdown")
            QMessageBox.warning(self, "No Selection", "Please select a pick location from the dropdown menu.")
            return
            
        if not location_name or location_name == "":
            self.log("⚠️  Please select a place location from the dropdown")
            QMessageBox.warning(self, "No Selection", "Please select a place location from the dropdown menu.")
            return
        
        use_gripper = self.use_gripper_at_pick_cb.isChecked()
        go_direct = self.go_direct_to_place_cb.isChecked()
        self.log(f"🤖 Executing: {object_name} → {location_name} (Gripper: {'ON' if use_gripper else 'OFF'}, Direct to place: {'Yes' if go_direct else 'No'})")
        self.operation_label.setText("Executing")
        
        # Disable buttons during execution
        self.execute_btn.setEnabled(False)
        self.pick_btn.setEnabled(False)
        self.place_btn.setEnabled(False)
        self.home_btn.setEnabled(False)
        self.set_home_btn.setEnabled(False)
        
        # Run in thread
        self.worker = PickPlaceWorker(self.pnp, object_name, location_name, use_gripper=use_gripper, go_direct_to_place=go_direct)
        self.worker.finished.connect(self.on_pick_place_finished)
        self.worker.log_signal.connect(self.log)
        self.worker.start()
    
    def on_pick_place_finished(self, success):
        """Handle pick and place completion"""
        if success:
            self.log("✅ Pick-and-place COMPLETE")
            self.operation_label.setText("Complete")
        else:
            self.log("❌ Pick-and-place FAILED")
            self.operation_label.setText("Failed")
        
        # Re-enable buttons
        self.execute_btn.setEnabled(True)
        self.pick_btn.setEnabled(True)
        self.place_btn.setEnabled(True)
        self.home_btn.setEnabled(True)
        self.set_home_btn.setEnabled(True)
    
    def emergency_stop(self):
        """Emergency stop"""
        self.log("🛑 EMERGENCY STOP")
        
        if self.gripper:
            self.gripper.emergency_release()
        
        if self.robot:
            self.robot.disable_robot()
        
        self.operation_label.setText("STOPPED")
        QMessageBox.warning(self, "Emergency Stop", "Robot stopped!\nPlease check robot state before continuing.")
    
    def load_camera_calibration(self):
        """Load camera calibration file"""
        try:
            parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
            calibration_file = os.path.join(parent_dir, "hikrobot_calibration_20260126_143821.npz")
            
            if os.path.exists(calibration_file):
                calib_data = np.load(calibration_file)
                self.camera_matrix = calib_data['camera_matrix']
                self.dist_coeffs = calib_data['dist_coeffs']
                self.log(f"Camera calibration loaded: {os.path.basename(calibration_file)}")
                print(f"Camera calibration loaded: {calibration_file}")
            else:
                self.log("Camera calibration file not found")
                print(f"Warning: Camera calibration file not found: {calibration_file}")
        except Exception as e:
            self.log(f"Failed to load camera calibration: {e}")
            print(f"Error loading camera calibration: {e}")
    
    def load_transform_matrix(self):
        """Load camera-robot transform matrix"""
        try:
            parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
            transform_file = os.path.join(parent_dir, "camera_robot_transform.json")
            
            if os.path.exists(transform_file):
                with open(transform_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    self.transform_matrix = np.array(data['transform_matrix'])
                    self.translation_offset = data.get('translation_offset', 0)
                    self.log(f"Transform matrix loaded: {os.path.basename(transform_file)}")
                    print(f"Transform matrix loaded: {transform_file}")
                    print(f"  Translation offset (Z): {self.translation_offset:.2f} mm")
                    
                    # Z 좌표 범위 확인
                    calibration_points = data.get('calibration_points', [])
                    if len(calibration_points) > 0:
                        z_values = [p[4] for p in calibration_points]
                        z_min = min(z_values)
                        z_max = max(z_values)
                        z_range = z_max - z_min
                        print(f"  Z range: {z_min:.2f} ~ {z_max:.2f} mm (range: {z_range:.2f} mm)")
                        if z_range < 10:
                            print(f"  ⚠️  Warning: Z height variation is small ({z_range:.2f}mm)")
                            print(f"     Consider collecting points at different Z heights for better accuracy")
                    
                    # Validate calibration accuracy
                    num_points = data.get('num_points', 0)
                    self.log(f"  Calibration points: {num_points}")
                    
                    if num_points < 6:
                        self.log(f"  ⚠️  Warning: Only {num_points} calibration points")
                        self.log("     More points (6-12) improve accuracy")
                    
                    # Check calibration point distribution
                    if len(calibration_points) >= 4:
                        x_coords = [p[2] for p in calibration_points]
                        y_coords = [p[3] for p in calibration_points]
                        x_range = max(x_coords) - min(x_coords)
                        y_range = max(y_coords) - min(y_coords)
                        
                        if x_range < 50 or y_range < 50:
                            self.log(f"  ⚠️  Warning: Calibration points are clustered")
                            self.log(f"     X range: {x_range:.1f}mm, Y range: {y_range:.1f}mm")
                            self.log("     Spread points across workspace for better accuracy")
            else:
                self.log("Transform matrix file not found")
                print(f"Warning: Transform matrix file not found: {transform_file}")
        except Exception as e:
            self.log(f"Failed to load transform matrix: {e}")
            print(f"Error loading transform matrix: {e}")
    
    def init_camera(self):
        """Initialize camera"""
        if not CAMERA_AVAILABLE:
            self.log("⚠️ Camera module not available - check MvImport SDK installation")
            return
            
        try:
            # Use already loaded calibration
            calibration_file = None
            parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
            calib_file_path = os.path.join(parent_dir, "hikrobot_calibration_20260126_143821.npz")
            
            if os.path.exists(calib_file_path):
                calibration_file = calib_file_path
                self.log(f"Found calibration file: {calib_file_path}")
            else:
                self.log(f"⚠️ Calibration file not found: {calib_file_path}")
            
            self.camera = HikRobotCamera(calibration_file=calibration_file)
            if self.camera.init_camera():
                self.log("✓ Camera initialized successfully")
            else:
                self.log("✗ Camera initialization failed")
                self.log("  Possible causes:")
                self.log("    1. Camera not connected")
                self.log("    2. Camera driver not installed")
                self.log("    3. Another application using the camera")
                self.camera = None
        except Exception as e:
            self.log(f"✗ Camera initialization error: {e}")
            self.log("  Check camera connection and MvImport SDK installation")
            import traceback
            traceback.print_exc()
            self.camera = None
    
    def start_camera(self):
        """Start camera streaming"""
        if not CAMERA_AVAILABLE:
            self.log("⚠️ Camera module not available")
            QMessageBox.warning(self, "Camera Not Available", 
                              "Camera module is not available.\n\n"
                              "Please check:\n"
                              "1. MvImport SDK is installed\n"
                              "2. Camera is connected\n"
                              "3. Camera drivers are installed")
            return
        
        if self.camera is None:
            self.log("⚠️ Camera not initialized. Trying to initialize...")
            self.init_camera()
            if self.camera is None:
                QMessageBox.warning(self, "Camera Initialization Failed", 
                                  "Failed to initialize camera.\n\n"
                                  "Please check:\n"
                                  "1. Camera is connected\n"
                                  "2. No other application is using the camera\n"
                                  "3. Camera drivers are installed")
                return
        
        if not CAMERA_AVAILABLE:
            self.log("Camera module not available")
            return
            
        if not self.camera or not self.camera.initialized:
            self.log("Camera not initialized")
            return
            
        if self.camera_active:
            return
            
        try:
            if CAMERA_AVAILABLE:
                self.camera_thread = CameraThread(self.camera)
                self.camera_thread.frame_ready.connect(self.update_camera_frame)
                self.camera_thread.start()
                self.camera_active = True
                self.start_camera_btn.setEnabled(False)
                self.stop_camera_btn.setEnabled(True)
                self.log("Camera streaming started")
        except Exception as e:
            self.log(f"Failed to start camera: {e}")
            import traceback
            traceback.print_exc()
    
    def stop_camera(self):
        """Stop camera streaming"""
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
        """Update camera frame display"""
        try:
            # Draw selection marker on frame if in selection mode
            display_frame = frame.copy()
            if self.selection_mode and self.selected_pixel_pos:
                x, y = self.selected_pixel_pos
                # Draw crosshair
                cv2.circle(display_frame, (x, y), 10, (255, 0, 0), 2)
                cv2.line(display_frame, (x - 15, y), (x + 15, y), (255, 0, 0), 2)
                cv2.line(display_frame, (x, y - 15), (x, y + 15), (255, 0, 0), 2)
            
            # Convert numpy array to QImage
            height, width, channel = display_frame.shape
            bytes_per_line = 3 * width
            q_image = QImage(display_frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            
            # Scale to fit label while maintaining aspect ratio
            scaled_pixmap = pixmap.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.camera_label.setPixmap(scaled_pixmap)
        except Exception as e:
            print(f"Frame update error: {e}")
    
    def pixel_to_robot_3d(self, pixel_x: int, pixel_y: int, z_height: float = None) -> Optional[Tuple[float, float, float]]:
        """
        Convert pixel coordinates to robot 3D coordinates
        
        Args:
            pixel_x, pixel_y: Pixel coordinates (0-639, 0-479)
            z_height: Z height in mm (if None, uses translation_offset or current robot Z)
            
        Returns:
            (robot_x, robot_y, robot_z) or None if failed
        """
        # Use transform matrix if available (more accurate)
        if self.transform_matrix is not None:
            try:
                # Homogeneous coordinates
                pixel_homogeneous = np.array([[pixel_x], [pixel_y], [1.0]])
                
                # Transform to robot 2D coordinates
                robot_2d = self.transform_matrix @ pixel_homogeneous
                # Normalize homogeneous coordinates
                robot_2d_normalized = robot_2d / robot_2d[2, 0]
                # Extract scalar values properly (avoid deprecation warning)
                robot_x = float(robot_2d_normalized[0, 0])
                robot_y = float(robot_2d_normalized[1, 0])
                
                # Apply calibration accuracy warning if needed
                accuracy = self.validate_calibration_accuracy()
                if accuracy and accuracy['avg_error'] > 10.0:
                    # Log warning but don't block conversion
                    if not hasattr(self, '_accuracy_warned'):
                        self.log(f"⚠️  Calibration accuracy warning: avg error = {accuracy['avg_error']:.1f}mm")
                        self.log("   Consider recalibrating with more points (see CALIBRATION_GUIDE.md)")
                        self._accuracy_warned = True
                
                # Z coordinate - use calibration offset if available
                if z_height is None:
                    if self.translation_offset is not None:
                        z_height = self.translation_offset  # Use calibrated Z offset
                    elif self.robot and self.robot.connected:
                        pose = self.robot.get_current_pose_from_feedback()
                        if pose:
                            z_height = pose[2]
                        else:
                            z_height = 100.0  # Default Z
                    else:
                        z_height = 100.0  # Default Z
                
                robot_z = float(z_height)
                
                return (robot_x, robot_y, robot_z)
            except Exception as e:
                self.log(f"Transform error: {e}")
                return None
        
        # Fallback: Use current robot position if transform matrix not available
        if self.robot and self.robot.connected:
            pose = self.robot.get_current_pose_from_feedback()
            if pose:
                self.log("⚠️  Transform matrix not available. Using current robot position.")
                return tuple(pose[:3])
        
        self.log("⚠️  Cannot convert pixel to robot coordinates. Transform matrix not loaded.")
        return None
    
    def validate_calibration_accuracy(self) -> dict:
        """
        Validate calibration accuracy using calibration points
        
        Returns:
            dict with accuracy metrics or None if validation fails
        """
        if self.transform_matrix is None:
            return None
        
        try:
            # Load calibration points from transform file
            parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
            transform_file = os.path.join(parent_dir, "camera_robot_transform.json")
            
            if not os.path.exists(transform_file):
                return None
            
            with open(transform_file, 'r', encoding='utf-8') as f:
                transform_data = json.load(f)
            
            calibration_points = transform_data.get('calibration_points', [])
            if len(calibration_points) < 4:
                return None
            
            errors = []
            for point in calibration_points:
                px, py, rx_actual, ry_actual, rz_actual = point
                
                # Predict robot position using transform matrix
                pixel_homogeneous = np.array([[px], [py], [1.0]])
                robot_2d = self.transform_matrix @ pixel_homogeneous
                robot_2d_normalized = robot_2d / robot_2d[2, 0]
                rx_pred = float(robot_2d_normalized[0, 0])
                ry_pred = float(robot_2d_normalized[1, 0])
                
                # Calculate error
                error = np.sqrt((rx_pred - rx_actual)**2 + (ry_pred - ry_actual)**2)
                errors.append(error)
            
            avg_error = np.mean(errors)
            max_error = np.max(errors)
            min_error = np.min(errors)
            
            return {
                'avg_error': avg_error,
                'max_error': max_error,
                'min_error': min_error,
                'num_points': len(calibration_points),
                'errors': errors
            }
        except Exception as e:
            self.log(f"Calibration validation error: {e}")
            return None
    
    def check_calibration_accuracy(self):
        """Check and display calibration accuracy"""
        accuracy = self.validate_calibration_accuracy()
        
        if accuracy is None:
            QMessageBox.warning(self, "Calibration Check Failed", 
                              "Could not validate calibration accuracy.\n"
                              "Make sure transform matrix is loaded.")
            return
        
        # Create detailed message
        msg = f"Calibration Accuracy Report\n\n"
        msg += f"Number of calibration points: {accuracy['num_points']}\n\n"
        msg += f"Average error: {accuracy['avg_error']:.2f} mm\n"
        msg += f"Maximum error: {accuracy['max_error']:.2f} mm\n"
        msg += f"Minimum error: {accuracy['min_error']:.2f} mm\n\n"
        
        if accuracy['avg_error'] < 5.0:
            msg += "✅ Accuracy: Excellent (< 5mm)\n"
            msg += "   Calibration is very accurate."
        elif accuracy['avg_error'] < 10.0:
            msg += "⚠️  Accuracy: Good (5-10mm)\n"
            msg += "   Calibration is acceptable but could be improved."
        elif accuracy['avg_error'] < 20.0:
            msg += "⚠️  Accuracy: Fair (10-20mm)\n"
            msg += "   Consider recalibrating with more points."
        else:
            msg += "❌ Accuracy: Poor (> 20mm)\n"
            msg += "   Recalibration strongly recommended."
        
        msg += "\n\nRecommendations:\n"
        if accuracy['num_points'] < 6:
            msg += f"- Add more calibration points (currently {accuracy['num_points']}, recommend 6-12)\n"
        if accuracy['avg_error'] > 10.0:
            msg += "- Spread calibration points across entire workspace\n"
            msg += "- Ensure points are on a flat surface\n"
            msg += "- Re-run camera_robot_calibration.py\n"
        
        QMessageBox.information(self, "Calibration Accuracy", msg)
        self.log(f"Calibration accuracy check: avg={accuracy['avg_error']:.2f}mm, "
                f"max={accuracy['max_error']:.2f}mm, points={accuracy['num_points']}")
    
    def on_camera_click(self, event):
        """Handle camera image click"""
        if not self.camera_active:
            self.log("Camera is not active. Start camera first.")
            return
            
        if not self.selection_mode:
            self.log("Please select 'Select Pick Position' or 'Select Place Position' first")
            return
            
        x = event.x()
        y = event.y()
        
        # Get label size and pixmap size to calculate actual pixel coordinates
        label_size = self.camera_label.size()
        pixmap = self.camera_label.pixmap()
        if not pixmap:
            self.log("No camera image available")
            return
            
        pixmap_size = pixmap.size()
        
        # Calculate scale factor (pixmap might be scaled to fit label)
        scale_x = 640.0 / pixmap_size.width() if pixmap_size.width() > 0 else 1.0
        scale_y = 480.0 / pixmap_size.height() if pixmap_size.height() > 0 else 1.0
        
        # Calculate offset (centered pixmap)
        offset_x = (label_size.width() - pixmap_size.width()) / 2
        offset_y = (label_size.height() - pixmap_size.height()) / 2
        
        # Calculate actual pixel coordinates in 640x480 image
        pix_x = int((x - offset_x) * scale_x)
        pix_y = int((y - offset_y) * scale_y)
        
        # Clamp to image bounds
        pix_x = max(0, min(639, pix_x))
        pix_y = max(0, min(479, pix_y))
        
        self.selected_pixel_pos = (pix_x, pix_y)
        self.log(f"Selected pixel position: ({pix_x}, {pix_y})")
        
        # Convert pixel to robot coordinates
        robot_pos = self.pixel_to_robot_3d(pix_x, pix_y)
        
        if robot_pos:
            self.selected_robot_pos = robot_pos
            self.log(f"Converted to robot coordinates: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}, {robot_pos[2]:.2f}) mm")
            
            # Validate coordinates are within workspace (Z 높이 고려, 상한선 완화)
            radius = np.sqrt(robot_pos[0]**2 + robot_pos[1]**2)
            
            # Z 높이에 따른 최대 반경 계산 (캘리브레이션 데이터 기반 완화)
            z_height = robot_pos[2]
            if z_height < 100:
                max_radius = 500.0  # 캘리브레이션 데이터: Z=74-79mm에서 반경 450mm 도달 가능
            elif z_height < 300:
                max_radius = 500.0 + (z_height - 100) * (550.0 - 500.0) / (300.0 - 100.0)
            elif z_height < 500:
                max_radius = 550.0 - (z_height - 300) * (550.0 - 500.0) / (500.0 - 300.0)
            else:
                max_radius = 450.0
            
            if radius > max_radius:
                self.log(f"⚠️  Warning: Position may be unreachable")
                self.log(f"   Radius: {radius:.1f} mm (estimated max for Z={z_height:.1f}mm: {max_radius:.1f} mm)")
                self.log(f"   Note: Calibration data shows similar positions (Z=74-79mm, radius 350-450mm) are reachable")
                reply = QMessageBox.question(self, "Position Warning", 
                                            f"Converted position:\n\n"
                                            f"X: {robot_pos[0]:.1f} mm\n"
                                            f"Y: {robot_pos[1]:.1f} mm\n"
                                            f"Z: {z_height:.1f} mm\n"
                                            f"Radius: {radius:.1f} mm\n"
                                            f"Estimated max for Z={z_height:.1f}mm: {max_radius:.1f} mm\n\n"
                                            f"⚠️  This position exceeds estimated workspace.\n"
                                            f"However, calibration data shows similar positions\n"
                                            f"(Z=74-79mm, radius 350-450mm) were reachable.\n\n"
                                            f"Continue and attempt movement?",
                                            QMessageBox.Yes | QMessageBox.No)
                if reply == QMessageBox.No:
                    return
            
            # Ask user for Z height adjustment (optional)
            radius = np.sqrt(robot_pos[0]**2 + robot_pos[1]**2)
            
            # Z 높이 기본값: translation_offset 또는 현재 변환된 Z
            default_z = robot_pos[2]
            if self.translation_offset is not None and abs(default_z - self.translation_offset) < 50:
                # translation_offset과 비슷하면 그것을 기본값으로 사용
                default_z = self.translation_offset
            
            z_height, ok = QInputDialog.getDouble(
                self, 
                "Z Height Adjustment", 
                f"Robot coordinates:\n"
                f"X: {robot_pos[0]:.2f} mm\n"
                f"Y: {robot_pos[1]:.2f} mm\n"
                f"Z: {robot_pos[2]:.2f} mm (from calibration offset: {self.translation_offset:.2f} mm)\n"
                f"Radius: {radius:.1f} mm\n\n"
                f"Adjust Z height if needed:",
                value=default_z,  # Use calibration offset as default
                min=0.0,
                max=600.0,
                decimals=1
            )
            
            if ok:
                # Update Z coordinate
                self.selected_robot_pos = (robot_pos[0], robot_pos[1], z_height)
                self.log(f"Final robot position: ({self.selected_robot_pos[0]:.2f}, {self.selected_robot_pos[1]:.2f}, {self.selected_robot_pos[2]:.2f}) mm")
                
                # Ask user to test movement before saving
                reply = QMessageBox.question(self, "Test Movement?", 
                                            f"Save position:\n"
                                            f"({self.selected_robot_pos[0]:.1f}, {self.selected_robot_pos[1]:.1f}, {self.selected_robot_pos[2]:.1f}) mm\n\n"
                                            f"Test movement to this position first?",
                                            QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
                
                if reply == QMessageBox.Yes:
                    # Test movement
                    self.test_position_movement(self.selected_robot_pos)
                    # Ask again if user wants to save
                    save_reply = QMessageBox.question(self, "Save Position?", 
                                                     "Save this position?",
                                                     QMessageBox.Yes | QMessageBox.No)
                    if save_reply == QMessageBox.Yes:
                        if self.selection_mode == 'pick':
                            self.save_pick_position()
                        elif self.selection_mode == 'place':
                            self.save_place_position()
                elif reply == QMessageBox.No:
                    # Save without testing
                    if self.selection_mode == 'pick':
                        self.save_pick_position()
                    elif self.selection_mode == 'place':
                        self.save_place_position()
                # Cancel: do nothing
        else:
            self.log("⚠️  Failed to convert pixel to robot coordinates")
            QMessageBox.warning(self, "Conversion Failed", 
                              "Failed to convert pixel coordinates to robot coordinates.\n"
                              "Make sure:\n"
                              "1. Transform matrix is loaded (camera_robot_transform.json)\n"
                              "2. Robot is connected")
    
    def test_position_movement(self, target_pos):
        """Test movement to target position with improved IK handling"""
        if not self.robot or not self.robot.connected:
            QMessageBox.warning(self, "Robot Not Connected", "Please connect robot first.")
            return
        
        self.log(f"Testing movement to ({target_pos[0]:.1f}, {target_pos[1]:.1f}, {target_pos[2]:.1f})...")
        
        # Get current position for reference
        current_pose = self.robot.get_current_pose_from_feedback()
        if current_pose:
            self.log(f"Current position: ({current_pose[0]:.1f}, {current_pose[1]:.1f}, {current_pose[2]:.1f})")
        
        # Pre-check IK solution and workspace limits
        radius = np.sqrt(target_pos[0]**2 + target_pos[1]**2)
        self.log(f"Target radius: {radius:.1f} mm")
        self.log(f"Target Z: {target_pos[2]:.1f} mm")
        
        # Check if position is likely unreachable BEFORE attempting movement
        max_radius = self.robot._get_max_radius_for_z(target_pos[2]) if hasattr(self.robot, '_get_max_radius_for_z') else None
        
        # Strong warning for difficult positions
        if target_pos[2] < 150 and radius > 400:
            warning_msg = (
                f"⚠️  CRITICAL WARNING: This position is very difficult to reach!\n\n"
                f"Target: ({target_pos[0]:.1f}, {target_pos[1]:.1f}, {target_pos[2]:.1f}) mm\n"
                f"Radius: {radius:.1f} mm\n"
                f"Z height: {target_pos[2]:.1f} mm\n\n"
                f"Problem: Low Z ({target_pos[2]:.1f}mm) + Large radius ({radius:.1f}mm) combination\n"
                f"This combination often causes IK solution failures and collisions.\n\n"
                f"Recommendations:\n"
                f"1. Increase Z height to 150-200mm\n"
                f"2. Move robot to a different starting position first\n"
                f"3. Try a position with smaller radius\n\n"
                f"Continue anyway? (This may cause collision)"
            )
            
            reply = QMessageBox.warning(self, "Position Warning - High Collision Risk", 
                                       warning_msg,
                                       QMessageBox.Yes | QMessageBox.No,
                                       QMessageBox.No)  # Default to No for safety
            
            if reply == QMessageBox.No:
                self.log("❌ Movement cancelled by user due to safety concerns")
                return
        
        if max_radius and radius > max_radius:
            warning_msg = (
                f"⚠️  WARNING: Position may be outside workspace!\n\n"
                f"Target: ({target_pos[0]:.1f}, {target_pos[1]:.1f}, {target_pos[2]:.1f}) mm\n"
                f"Radius: {radius:.1f} mm (max for Z={target_pos[2]:.1f}mm: {max_radius:.1f} mm)\n\n"
                f"This position exceeds estimated workspace limits.\n"
                f"Movement may fail or cause collision.\n\n"
                f"Continue anyway?"
            )
            
            reply = QMessageBox.warning(self, "Workspace Warning", 
                                       warning_msg,
                                       QMessageBox.Yes | QMessageBox.No,
                                       QMessageBox.No)
            
            if reply == QMessageBox.No:
                self.log("❌ Movement cancelled by user")
                return
        
        # Try direct movement first, then alternative orientation if needed
        strategies = [
            {
                "name": "Direct movement",
                "pos": target_pos,
                "orient": (180, 0, 0)
            },
            {
                "name": "Alternative orientation (0,0,0)",
                "pos": target_pos,
                "orient": (0, 0, 0)
            },
        ]
        
        for strategy in strategies:
            self.log(f"Trying strategy: {strategy['name']}...")
            test_pos = strategy['pos']
            rx, ry, rz = strategy['orient']
            
            # Clear any previous errors before attempting movement
            self.robot.clear_error()
            
            # Log the attempt
            self.log(f"  Attempting: X={test_pos[0]:.1f}, Y={test_pos[1]:.1f}, Z={test_pos[2]:.1f}, Rx={rx}, Ry={ry}, Rz={rz}")
            
            # Adjust velocity based on position difficulty
            # Lower Z + larger radius = slower speed for safety
            test_radius = np.sqrt(test_pos[0]**2 + test_pos[1]**2)
            if test_pos[2] < 150 and test_radius > 400:
                # Low Z + large radius: use very slow speed to prevent collision
                test_velocity = 15.0  # Very slow
                self.log(f"  Using slow speed (15%) for safety: Low Z ({test_pos[2]:.1f}mm) + Large radius ({test_radius:.1f}mm)")
            elif test_pos[2] < 200 or test_radius > 450:
                # Medium difficulty: use moderate speed
                test_velocity = 20.0  # Slow
                self.log(f"  Using moderate speed (20%) for safety")
            else:
                # Normal position: use standard speed
                test_velocity = 25.0  # Reduced from 30.0
    
            success = self.robot.move_j(
                test_pos[0], test_pos[1], test_pos[2],
                rx, ry, rz,
                coordinate_mode=0,
                velocity=test_velocity,
                use_waypoint=False
            )
            
            if not success:
                # Log failure reason (error code should be printed by move_j)
                self.log(f"  ✗ Strategy '{strategy['name']}' failed")
                continue
            
            if success:
                self.robot.wait_for_motion_complete()
                
                if test_pos[2] != target_pos[2]:
                    # Reached intermediate position, try to lower Z
                    self.log(f"✓ Reached intermediate position (Z={test_pos[2]:.1f}mm), attempting to lower Z...")
                    final_success = self.robot.move_j(
                        target_pos[0], target_pos[1], target_pos[2],
                        rx, ry, rz,
                        coordinate_mode=0,
                        velocity=20.0,  # Very slow for precision
                        use_waypoint=False
                    )
                    
                    if final_success:
                        self.robot.wait_for_motion_complete()
                        self.log("✅ Test movement successful!")
                        QMessageBox.information(self, "Test Success", 
                                              f"Movement successful using strategy: {strategy['name']}")
                        return
                    else:
                        self.log(f"⚠️  Could not lower to target Z={target_pos[2]:.1f}mm")
                        QMessageBox.warning(self, "Partial Success", 
                                          f"Reached intermediate position:\n"
                                          f"X: {target_pos[0]:.1f} mm\n"
                                          f"Y: {target_pos[1]:.1f} mm\n"
                                          f"Z: {test_pos[2]:.1f} mm (target: {target_pos[2]:.1f} mm)\n\n"
                                          f"Strategy used: {strategy['name']}\n\n"
                                          f"Could not lower to target Z.\n"
                                          f"Try adjusting Z height or move manually.")
                        return
                else:
                    self.log("✅ Test movement successful!")
                    QMessageBox.information(self, "Test Success", 
                                          f"Movement successful using strategy: {strategy['name']}")
                    return
        
        # All strategies failed
        self.log("❌ All movement strategies failed")
        
        # Check robot error state
        error_info = ""
        try:
            # Try to get error information from robot
            error_state = self.robot.dashboard.GetErrorID()
            if error_state and error_state != "" and error_state != "0":
                error_info = f"\nRobot error code: {error_state}\n"
        except:
            pass
        
        # Calculate workspace info
        max_radius = self.robot._get_max_radius_for_z(target_pos[2]) if hasattr(self.robot, '_get_max_radius_for_z') else None
        
        workspace_info = ""
        if max_radius:
            if radius > max_radius:
                workspace_info = f"\n⚠️  Warning: Radius ({radius:.1f}mm) exceeds estimated max ({max_radius:.1f}mm) for Z={target_pos[2]:.1f}mm\n"
        
        QMessageBox.warning(self, "Test Failed - IK Solution Not Found", 
                          f"All movement strategies failed.\n\n"
                          f"Target position:\n"
                          f"X: {target_pos[0]:.1f} mm\n"
                          f"Y: {target_pos[1]:.1f} mm\n"
                          f"Z: {target_pos[2]:.1f} mm\n"
                          f"Radius: {radius:.1f} mm{workspace_info}"
                          f"{error_info}"
                          f"\nError: Inverse Kinematics (IK) solution not found\n"
                          f"This position/orientation combination is not reachable.\n\n"
                          f"Possible reasons:\n"
                          f"- Low Z ({target_pos[2]:.1f}mm) + Large radius ({radius:.1f}mm) combination\n"
                          f"- Current robot position makes this target unreachable\n"
                          f"- Orientation angles (Rx=180, Ry=0, Rz=0) cause singularity\n\n"
                          f"Solutions:\n"
                          f"1. Manually move robot to a different starting position\n"
                          f"2. Adjust Z height manually (try 150-200mm)\n"
                          f"3. Try different orientation angles\n"
                          f"4. Check if calibration point was actually reachable\n"
                          f"5. Use joint mode for manual control")
    
    def set_selection_mode(self, mode):
        """Set selection mode (pick or place)"""
        self.selection_mode = mode
        if mode == 'pick':
            self.log("Click on camera image to select pick position")
            self.select_pick_pos_btn.setStyleSheet("background-color: #4CAF50; color: white;")
            self.select_place_pos_btn.setStyleSheet("")
        elif mode == 'place':
            self.log("Click on camera image to select place position")
            self.select_place_pos_btn.setStyleSheet("background-color: #4CAF50; color: white;")
            self.select_pick_pos_btn.setStyleSheet("")
    
    def save_pick_position(self):
        """Save selected pick position"""
        if not self.selected_robot_pos:
            self.log("No robot position selected")
            return
            
        name, ok = QInputDialog.getText(self, "Save Pick Position", 
                                        f"Enter name for this pick position:\n"
                                        f"Position: ({self.selected_robot_pos[0]:.1f}, {self.selected_robot_pos[1]:.1f}, {self.selected_robot_pos[2]:.1f}) mm")
        if ok and name:
            # Update config
            if 'positions' not in self.config:
                self.config['positions'] = {}
            if 'pick_locations' not in self.config['positions']:
                self.config['positions']['pick_locations'] = {}
                
            self.config['positions']['pick_locations'][name] = {
                'x': float(self.selected_robot_pos[0]),
                'y': float(self.selected_robot_pos[1]),
                'z': float(self.selected_robot_pos[2]),
                'rx': 180,
                'ry': 0,
                'rz': 0
            }
            
            # Update combo box
            self.object_combo.addItem(name)
            self.pick_locations[name] = self.config['positions']['pick_locations'][name]
            
            # Save config file
            self.save_config_file()
            # Sync PickAndPlace so it knows the new position (otherwise Execute uses stale list)
            self._sync_pnp_locations()
            
            self.log(f"✅ Saved pick position: {name} at ({self.selected_robot_pos[0]:.1f}, {self.selected_robot_pos[1]:.1f}, {self.selected_robot_pos[2]:.1f})")
            self.selection_mode = None
            self.select_pick_pos_btn.setStyleSheet("")
            self.selected_pixel_pos = None
            self.selected_robot_pos = None
    
    def save_place_position(self):
        """Save selected place position"""
        if not self.selected_robot_pos:
            self.log("No robot position selected")
            return
            
        name, ok = QInputDialog.getText(self, "Save Place Position", 
                                       f"Enter name for this place position:\n"
                                       f"Position: ({self.selected_robot_pos[0]:.1f}, {self.selected_robot_pos[1]:.1f}, {self.selected_robot_pos[2]:.1f}) mm")
        if ok and name:
            # Update config
            if 'positions' not in self.config:
                self.config['positions'] = {}
            if 'place_locations' not in self.config['positions']:
                self.config['positions']['place_locations'] = {}
                
            self.config['positions']['place_locations'][name] = {
                'x': float(self.selected_robot_pos[0]),
                'y': float(self.selected_robot_pos[1]),
                'z': float(self.selected_robot_pos[2]),
                'rx': 180,
                'ry': 0,
                'rz': 0
            }
            
            # Update combo box
            self.destination_combo.addItem(name)
            self.place_locations[name] = self.config['positions']['place_locations'][name]
            
            # Save config file
            self.save_config_file()
            # Sync PickAndPlace so it knows the new position
            self._sync_pnp_locations()
            
            self.log(f"✅ Saved place position: {name} at ({self.selected_robot_pos[0]:.1f}, {self.selected_robot_pos[1]:.1f}, {self.selected_robot_pos[2]:.1f})")
            self.selection_mode = None
            self.select_place_pos_btn.setStyleSheet("")
            self.selected_pixel_pos = None
            self.selected_robot_pos = None
    
    def _sync_pnp_locations(self):
        """Sync PickAndPlace pick/place locations from current config (so Execute uses latest)."""
        if not self.pnp:
            return
        if 'positions' in self.config:
            self.pnp.pick_locations = self.config['positions'].get('pick_locations', {})
            self.pnp.place_locations = self.config['positions'].get('place_locations', {})
    
    def save_config_file(self):
        """Save config to file"""
        try:
            config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
            config_dir = os.path.normpath(config_dir)
            json_path = os.path.join(config_dir, 'robot_config.json')
            
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(self.config, f, indent=2, ensure_ascii=False)
            
            self.log(f"Config saved to {os.path.basename(json_path)}")
        except Exception as e:
            self.log(f"Failed to save config: {e}")
            QMessageBox.warning(self, "Save Error", f"Failed to save config file:\n{e}")
    
    def closeEvent(self, event):
        """Handle window close"""
        # Stop camera
        if self.camera_active:
            self.stop_camera()
        
        # Cleanup camera
        if self.camera:
            self.camera.cleanup()
        
        if self.robot and self.robot.connected:
            reply = QMessageBox.question(self, 'Exit',
                "Robot is still connected. Disconnect and exit?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
            if reply == QMessageBox.Yes:
                self.disconnect_robot()
                event.accept()
            else:
                event.ignore()
        else:
            event.accept()


def main():
    """Main entry point"""
    try:
        print("Starting GUI application...")
        app = QApplication(sys.argv)
        
        # Set application style
        app.setStyle('Fusion')
        
        print("Creating GUI window...")
        # Create and show GUI
        gui = PickPlaceGUI()
        gui.show()
        
        print("GUI window created successfully!")
        print("If you don't see the window, check your taskbar.")
        
        sys.exit(app.exec_())
        
    except Exception as e:
        print(f"Error starting GUI: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
