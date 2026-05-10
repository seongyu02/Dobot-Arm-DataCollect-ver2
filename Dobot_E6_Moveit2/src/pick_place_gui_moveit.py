#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pick and Place GUI (Windows) - Pose-based control
Target pose (X,Y,Z,Rx,Ry,Rz) → MoveJ via Dobot E6 TCP/IP (robot IK).
No ROS 2 / MoveIt dependency on Windows.
"""

import sys
import os

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
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QGridLayout, QLabel, QLineEdit, QPushButton, QTextEdit,
    QDoubleSpinBox, QMessageBox
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

# Robot control (same as pick_place_gui.py)
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, current_dir)
sys.path.insert(0, parent_dir)

from dobot_e6_controller import DobotE6Controller
from suction_gripper import SuctionGripper


class PickPlaceGUI(QMainWindow):
    """Pose-based Pick & Place GUI for Windows (no ROS/MoveIt)."""

    def __init__(self):
        super().__init__()
        self.robot = None
        self.gripper = None
        self.config = None
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Dobot E6 Pick & Place (Pose) - Windows")
        self.setGeometry(100, 100, 800, 620)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)

        # Connection
        main_layout.addWidget(self._create_connection_panel())
        # Target pose (mm, deg)
        main_layout.addWidget(self._create_pose_panel())
        # Actions
        main_layout.addWidget(self._create_action_panel())
        # Status
        main_layout.addWidget(self._create_status_panel())

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(160)
        main_layout.addWidget(QLabel("Log:"))
        main_layout.addWidget(self.log_text)

        self.log("GUI initialized (Windows, no ROS)")

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
        group = QGroupBox("Target Pose (mm, degrees) - Robot Cartesian")
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
        from datetime import datetime
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
        self.log(f"MoveJ to ({x:.1f}, {y:.1f}, {z:.1f}) mm, ({rx:.1f}, {ry:.1f}, {rz:.1f}) °")
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

    def emergency_stop(self):
        if self.robot:
            self.robot.disable_robot()
            self.log("E-STOP")
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

    def closeEvent(self, event):
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
    gui = PickPlaceGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
