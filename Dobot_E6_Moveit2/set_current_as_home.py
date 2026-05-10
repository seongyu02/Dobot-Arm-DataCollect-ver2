#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Set Current Robot Position as Home Position
간단한 스크립트로 현재 로봇 위치를 Home position으로 설정
"""

import sys
import os
import json

# Windows 콘솔 UTF-8 인코딩 설정
if sys.platform == 'win32':
    try:
        import io
        if not isinstance(sys.stdout, io.TextIOWrapper) or (hasattr(sys.stdout, 'encoding') and sys.stdout.encoding.lower() != 'utf-8'):
            if hasattr(sys.stdout, 'buffer'):
                sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
            if hasattr(sys.stderr, 'buffer'):
                sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')
    except:
        pass

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Dobot_E6_Moveit2.src.dobot_e6_controller import DobotE6Controller

def main():
    """Set current robot position as home"""
    print("=" * 60)
    print("Set Current Position as Home")
    print("=" * 60)
    
    # Get robot IP from user or use default
    ip = input("Enter robot IP address (default: 192.168.5.1): ").strip()
    if not ip:
        ip = "192.168.5.1"
    
    # Connect to robot
    print(f"\nConnecting to robot at {ip}...")
    robot = DobotE6Controller(ip=ip)
    
    if not robot.connect():
        print("✗ Failed to connect to robot")
        return
    
    print("✓ Robot connected\n")
    
    # Get current position
    print("Getting current robot position...")
    current_pose = robot.get_current_pose_from_feedback()
    
    if not current_pose:
        print("✗ Failed to get current position")
        robot.disconnect()
        return
    
    x, y, z, rx, ry, rz = current_pose
    
    print("\nCurrent Robot Position:")
    print(f"  X:  {x:.2f} mm")
    print(f"  Y:  {y:.2f} mm")
    print(f"  Z:  {z:.2f} mm")
    print(f"  RX: {rx:.2f}°")
    print(f"  RY: {ry:.2f}°")
    print(f"  RZ: {rz:.2f}°")
    
    # Confirm
    print("\n" + "=" * 60)
    confirm = input("Set this position as home? (yes/no): ").strip().lower()
    
    if confirm not in ['yes', 'y']:
        print("Cancelled.")
        robot.disconnect()
        return
    
    # Load config file
    config_dir = os.path.join(os.path.dirname(__file__), 'config')
    config_file = os.path.join(config_dir, 'robot_config.json')
    
    if not os.path.exists(config_file):
        print(f"✗ Config file not found: {config_file}")
        robot.disconnect()
        return
    
    # Read config
    with open(config_file, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    # Update home position
    config['positions']['home'] = {
        'x': float(x),
        'y': float(y),
        'z': float(z),
        'rx': float(rx),
        'ry': float(ry),
        'rz': float(rz)
    }
    
    # Save config
    with open(config_file, 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2, ensure_ascii=False)
    
    print("\n✓ Home position updated successfully!")
    print(f"  Config saved to: {config_file}")
    
    # Also update YAML if exists
    yaml_file = os.path.join(config_dir, 'robot_config.yaml')
    if os.path.exists(yaml_file):
        try:
            import yaml
            config_yaml = {
                'robot': config['robot'],
                'gripper': config['gripper'],
                'positions': {
                    'home': config['positions']['home'],
                    'pick_approach_height': config['positions']['pick_approach_height'],
                    'place_approach_height': config['positions']['place_approach_height'],
                    'pick_locations': config['positions']['pick_locations'],
                    'place_locations': config['positions']['place_locations']
                },
                'movement': config['movement'],
                'safety': config['safety']
            }
            with open(yaml_file, 'w', encoding='utf-8') as f:
                yaml.dump(config_yaml, f, default_flow_style=False, allow_unicode=True)
            print(f"  YAML config also updated: {yaml_file}")
        except ImportError:
            print("  (YAML update skipped - PyYAML not installed)")
    
    robot.disconnect()
    print("\nDone!")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
