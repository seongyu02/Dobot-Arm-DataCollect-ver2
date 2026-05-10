#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simple test script for GUI"""

import sys
import os

# Add paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

print("Testing imports...")

try:
    from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
    print("✓ PyQt5 imported successfully")
except Exception as e:
    print(f"✗ PyQt5 import failed: {e}")
    sys.exit(1)

try:
    from dobot_e6_controller import DobotE6Controller
    print("✓ dobot_e6_controller imported successfully")
except Exception as e:
    print(f"✗ dobot_e6_controller import failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\nCreating simple GUI window...")

try:
    app = QApplication(sys.argv)
    
    window = QMainWindow()
    window.setWindowTitle("Test GUI")
    window.setGeometry(100, 100, 400, 300)
    
    label = QLabel("GUI Test Window\nIf you see this, GUI is working!", window)
    label.setGeometry(50, 50, 300, 100)
    label.setStyleSheet("font-size: 16px;")
    
    window.show()
    
    print("✓ GUI window created!")
    print("Window should be visible now.")
    
    sys.exit(app.exec_())
    
except Exception as e:
    print(f"✗ GUI creation failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
