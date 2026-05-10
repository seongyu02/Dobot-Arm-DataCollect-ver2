#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Debug script for GUI"""

import sys
import os
import traceback
import io

# Windows 콘솔 UTF-8 인코딩 설정
if sys.platform == 'win32':
    try:
        if not isinstance(sys.stdout, io.TextIOWrapper) or (hasattr(sys.stdout, 'encoding') and sys.stdout.encoding.lower() != 'utf-8'):
            if hasattr(sys.stdout, 'buffer') and not sys.stdout.buffer.closed:
                sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
            if hasattr(sys.stderr, 'buffer') and not sys.stderr.buffer.closed:
                sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')
    except:
        pass

print("=" * 60)
print("GUI Debug Script")
print("=" * 60)

# Add paths
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(current_dir, 'src')
parent_dir = os.path.dirname(current_dir)

sys.path.insert(0, src_dir)
sys.path.insert(0, parent_dir)

print(f"Current dir: {current_dir}")
print(f"Src dir: {src_dir}")
print(f"Parent dir: {parent_dir}")
print()

# Test imports
print("Testing imports...")
try:
    from PyQt5.QtWidgets import QApplication
    print("[OK] PyQt5.QtWidgets imported")
except Exception as e:
    print(f"[FAIL] PyQt5.QtWidgets failed: {e}")
    traceback.print_exc()
    sys.exit(1)

try:
    from dobot_e6_controller import DobotE6Controller
    print("[OK] dobot_e6_controller imported")
except Exception as e:
    print(f"[FAIL] dobot_e6_controller failed: {e}")
    traceback.print_exc()
    sys.exit(1)

try:
    from suction_gripper import SuctionGripper
    print("[OK] suction_gripper imported")
except Exception as e:
    print(f"[FAIL] suction_gripper failed: {e}")
    traceback.print_exc()
    sys.exit(1)

try:
    from pick_place_logic import PickAndPlace
    print("[OK] pick_place_logic imported")
except Exception as e:
    print(f"[FAIL] pick_place_logic failed: {e}")
    traceback.print_exc()
    sys.exit(1)

# Test config loading
print("\nTesting config loading...")
config_dir = os.path.join(current_dir, 'config')
json_path = os.path.join(config_dir, 'robot_config.json')
yaml_path = os.path.join(config_dir, 'robot_config.yaml')

print(f"Config dir: {config_dir}")
print(f"JSON exists: {os.path.exists(json_path)}")
print(f"YAML exists: {os.path.exists(yaml_path)}")

if os.path.exists(json_path):
    try:
        import json
        with open(json_path, 'r', encoding='utf-8') as f:
            config = json.load(f)
        print("[OK] JSON config loaded successfully")
    except Exception as e:
        print(f"[FAIL] JSON config failed: {e}")
        traceback.print_exc()

# Test GUI creation
print("\nTesting GUI creation...")
try:
    app = QApplication(sys.argv)
    print("[OK] QApplication created")
    
    from pick_place_gui import PickPlaceGUI
    print("[OK] PickPlaceGUI imported")
    
    gui = PickPlaceGUI()
    print("[OK] PickPlaceGUI instance created")
    
    gui.show()
    print("[OK] GUI window shown")
    print("\n" + "=" * 60)
    print("GUI should be visible now!")
    print("=" * 60)
    
    sys.exit(app.exec_())
    
except Exception as e:
    print(f"[FAIL] GUI creation failed: {e}")
    traceback.print_exc()
    sys.exit(1)
