#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pick and Place Logic
Implements pick-and-place sequence with suction gripper
"""

import sys
import os
import time
import numpy as np
from typing import Dict, Tuple

# Windows 콘솔 UTF-8 인코딩 설정 (안전한 방법)
if sys.platform == 'win32':
    try:
        import io
        # stdout이 이미 래핑되어 있지 않은 경우에만 설정
        if not isinstance(sys.stdout, io.TextIOWrapper) or (hasattr(sys.stdout, 'encoding') and sys.stdout.encoding.lower() != 'utf-8'):
            if hasattr(sys.stdout, 'buffer') and not sys.stdout.buffer.closed:
                sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
            if hasattr(sys.stderr, 'buffer') and not sys.stderr.buffer.closed:
                sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')
    except:
        pass  # 실패해도 계속 진행

import json

# Add parent directories to path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, current_dir)  # For local imports
sys.path.insert(0, parent_dir)    # For dobot_api.py

from dobot_e6_controller import DobotE6Controller
from suction_gripper import SuctionGripper


class PickAndPlace:
    """Pick and place operation manager"""
    
    def __init__(self, robot: DobotE6Controller, gripper: SuctionGripper, config_file: str):
        """
        Initialize pick and place manager
        
        Args:
            robot: DobotE6Controller instance
            gripper: SuctionGripper instance
            config_file: Path to robot_config.yaml
        """
        self.robot = robot
        self.gripper = gripper
        
        # Load configuration (support both YAML and JSON)
        with open(config_file, 'r', encoding='utf-8') as f:
            if config_file.endswith('.yaml') or config_file.endswith('.yml'):
                try:
                    import yaml
                    self.config = yaml.safe_load(f)
                except ImportError:
                    # Fallback: try JSON version
                    json_file = config_file.replace('.yaml', '.json').replace('.yml', '.json')
                    if os.path.exists(json_file):
                        with open(json_file, 'r', encoding='utf-8') as jf:
                            self.config = json.load(jf)
                    else:
                        raise ImportError("PyYAML is required for YAML config files. Install with: pip install PyYAML\nOr use JSON config file: robot_config.json")
            else:
                self.config = json.load(f)
        
        self.pick_locations = self.config['positions'].get('pick_locations', {})
        self.place_locations = self.config['positions'].get('place_locations', {})
        self.home_pos = self.config['positions'].get('home', {
            'x': 300, 'y': 0, 'z': 300, 'rx': 180, 'ry': 0, 'rz': 0
        })
        
        # Validate locations
        if not self.pick_locations:
            print("⚠️  Warning: No pick locations found in config")
        if not self.place_locations:
            print("⚠️  Warning: No place locations found in config")
        
        self.pick_approach_height = self.config['positions']['pick_approach_height']
        self.place_approach_height = self.config['positions']['place_approach_height']
        
    def go_home(self) -> bool:
        """
        Move robot to home position with collision avoidance
        
        Returns:
            True if successful
        """
        print("\n🏠 Moving to home position...")
        home = self.home_pos
        
        # Clear any existing errors first
        self.robot.clear_error()
        
        # Get current position
        current_pose = self.robot.get_current_pose_from_feedback()
        if current_pose:
            print(f"Current position: ({current_pose[0]:.1f}, {current_pose[1]:.1f}, {current_pose[2]:.1f})")
        
        # Try pose mode first
        success = self.robot.move_j(home['x'], home['y'], home['z'], 
                                    home['rx'], home['ry'], home['rz'], 
                                    coordinate_mode=0,
                                    use_waypoint=False)
        
        # If pose mode fails, try moving to safe position first
        if not success:
            print("⚠️  Direct home movement failed, moving to safe position first...")
            if self.robot.move_to_safe_position():
                self.robot.wait_for_motion_complete()
                # Try home again from safe position
                print("Retrying home position from safe position...")
                success = self.robot.move_j(home['x'], home['y'], home['z'], 
                                           home['rx'], home['ry'], home['rz'], 
                                           coordinate_mode=0,
                                           use_waypoint=False)
        
        # If still fails, try joint mode (all joints at 0)
        if not success:
            print("⚠️  Pose mode failed, trying joint mode (all joints at 0)...")
            success = self.robot.move_j(0, 0, 0, 0, 0, 0, coordinate_mode=1, use_waypoint=False)
        
        if success:
            self.robot.wait_for_motion_complete()
            print("✓ Home position reached")
        else:
            print("✗ Failed to move to home position")
            print("  Try manually moving robot to a safe position")
            print("  Or use 'Set Current as Home' button in GUI to update home position")
        
        return success
    
    def pick_object(self, object_name: str, use_gripper: bool = True) -> bool:
        """
        Pick object from specified location
        
        Args:
            object_name: Name of object location in config
            use_gripper: If True, activate suction after descending (default). If False, only move to pick position and lift (no grip).
            
        Returns:
            True if successful
        """
        if object_name not in self.pick_locations:
            print(f"✗ Unknown object location: {object_name}")
            print(f"  Available locations: {list(self.pick_locations.keys())}")
            print(f"  Tip: Use full name like 'object_1' instead of '1'")
            return False
        
        pos = self.pick_locations[object_name]
        
        print(f"\n📦 Picking object: {object_name}")
        
        # 1. Move to approach position (above object)
        print("  1. Approaching...")
        approach_z = pos['z'] + self.pick_approach_height
        
        # Check if position is reachable (Z 높이 고려, 상한선 완화)
        radius = np.sqrt(pos['x']**2 + pos['y']**2)
        z_height = approach_z
        
        # Z 높이에 따른 최대 반경 계산 (캘리브레이션 데이터 기반 완화)
        if z_height < 100:
            max_radius = 500.0  # 캘리브레이션 데이터: Z=74-79mm에서 반경 450mm 도달 가능
        elif z_height < 300:
            max_radius = 500.0 + (z_height - 100) * (550.0 - 500.0) / (300.0 - 100.0)
        elif z_height < 500:
            max_radius = 550.0 - (z_height - 300) * (550.0 - 500.0) / (500.0 - 300.0)
        else:
            max_radius = 450.0
        
        if radius > max_radius:
            print(f"  ⚠️  Warning: Position may be unreachable")
            print(f"  Radius: {radius:.1f} mm (max for Z={z_height:.1f}mm: {max_radius:.1f} mm)")
            print(f"  This position may be outside robot workspace for this Z height")
        
        # Clear errors before movement
        self.robot.clear_error()
        
        # Use slower speed for safety, especially for low Z positions
        pick_velocity = 20.0  # Reduced from 30.0 for safety
        
        success = self.robot.move_j(pos['x'], pos['y'], approach_z, 
                                    pos['rx'], pos['ry'], pos['rz'], 
                                    coordinate_mode=0,
                                    velocity=pick_velocity,
                                    use_waypoint=False)  # Use waypoint if needed
        
        if not success:
            print(f"  ✗ Failed to move to approach position")
            print(f"  Tip: Position ({pos['x']:.1f}, {pos['y']:.1f}, {approach_z:.1f}) may be unreachable")
            print(f"  Possible reasons:")
            print(f"    - Position outside robot workspace")
            print(f"    - IK solution not found")
            print(f"    - Robot collision detected")
            print(f"  Try adjusting the position or moving robot manually first")
            return False
        
        self.robot.wait_for_motion_complete()
        
        # 2. Move down to object (linear movement for precision)
        print("  2. Descending...")
        success = self.robot.move_l(pos['x'], pos['y'], pos['z'], 
                                    pos['rx'], pos['ry'], pos['rz'], 
                                    coordinate_mode=0,
                                    velocity=20.0)  # Slow for precision
        if not success:
            print(f"  ✗ Failed to move to pick position")
            print(f"  Tip: Try adjusting Z height or position")
            return False
        self.robot.wait_for_motion_complete()
        
        # 3. Activate suction (optional)
        if use_gripper:
            print("  3. Gripping...")
            if not self.gripper.grip():
                return False
        else:
            print("  3. Skipping gripper (use_gripper=False)")
        
        # 4. Move up
        print("  4. Lifting...")
        success = self.robot.move_l(pos['x'], pos['y'], approach_z, 
                                    pos['rx'], pos['ry'], pos['rz'],
                                    coordinate_mode=0,
                                    velocity=30.0)
        if not success:
            print(f"  ✗ Failed to lift")
            self.gripper.emergency_release()
            return False
        self.robot.wait_for_motion_complete()
        
        print("✓ Pick complete")
        return True
    
    def place_object(self, location_name: str) -> bool:
        """
        Place object at specified location
        
        Args:
            location_name: Name of place location in config
            
        Returns:
            True if successful
        """
        if location_name not in self.place_locations:
            print(f"✗ Unknown place location: {location_name}")
            print(f"  Available locations: {list(self.place_locations.keys())}")
            print(f"  Tip: Use full name like 'destination_1' instead of '1'")
            return False
        
        pos = self.place_locations[location_name]
        
        print(f"\n📍 Placing at: {location_name}")
        
        # 1. Move to approach position (above destination)
        print("  1. Approaching destination...")
        approach_z = pos['z'] + self.place_approach_height
        
        # Check if position is reachable (Z 높이 고려, 상한선 완화)
        radius = np.sqrt(pos['x']**2 + pos['y']**2)
        z_height = approach_z
        
        # Z 높이에 따른 최대 반경 계산 (캘리브레이션 데이터 기반 완화)
        if z_height < 100:
            max_radius = 500.0  # 캘리브레이션 데이터: Z=74-79mm에서 반경 450mm 도달 가능
        elif z_height < 300:
            max_radius = 500.0 + (z_height - 100) * (550.0 - 500.0) / (300.0 - 100.0)
        elif z_height < 500:
            max_radius = 550.0 - (z_height - 300) * (550.0 - 500.0) / (500.0 - 300.0)
        else:
            max_radius = 450.0
        
        if radius > max_radius:
            print(f"  ⚠️  Warning: Position may be unreachable")
            print(f"  Radius: {radius:.1f} mm (max for Z={z_height:.1f}mm: {max_radius:.1f} mm)")
            print(f"  This position may be outside robot workspace for this Z height")
        
        # Clear errors before movement
        self.robot.clear_error()
        
        # Use slower speed for safety, especially for low Z positions
        place_velocity = 20.0  # Reduced from 30.0 for safety
        
        success = self.robot.move_j(pos['x'], pos['y'], approach_z, 
                                    pos['rx'], pos['ry'], pos['rz'], 
                                    coordinate_mode=0,
                                    velocity=place_velocity,
                                    use_waypoint=False)  # Use waypoint if needed
        
        if not success:
            print(f"  ✗ Failed to move to approach position")
            print(f"  Tip: Position ({pos['x']:.1f}, {pos['y']:.1f}, {approach_z:.1f}) may be unreachable")
            print(f"  Possible reasons:")
            print(f"    - Position outside robot workspace")
            print(f"    - IK solution not found")
            print(f"    - Robot collision detected")
            return False
        self.robot.wait_for_motion_complete()
        
        # 2. Move down to place position (linear movement for precision)
        print("  2. Descending...")
        success = self.robot.move_l(pos['x'], pos['y'], pos['z'], 
                                    pos['rx'], pos['ry'], pos['rz'], 
                                    coordinate_mode=0,
                                    velocity=20.0)  # Slow for precision
        if not success:
            print(f"  ✗ Failed to move to place position")
            print(f"  Tip: Try adjusting Z height or position")
            return False
        self.robot.wait_for_motion_complete()
        
        # 3. Release suction
        print("  3. Releasing...")
        if not self.gripper.release():
            return False
        
        # 4. Move up
        print("  4. Retracting...")
        success = self.robot.move_l(pos['x'], pos['y'], approach_z, 
                                    pos['rx'], pos['ry'], pos['rz'],
                                    coordinate_mode=0,
                                    velocity=30.0)
        if not success:
            print(f"  ✗ Failed to retract")
            return False
        self.robot.wait_for_motion_complete()
        
        print("✓ Place complete")
        return True
    
    def execute_pick_and_place(self, object_name: str, location_name: str,
                               use_gripper: bool = True,
                               go_direct_to_place: bool = False) -> bool:
        """
        Execute complete pick and place sequence
        
        Args:
            object_name: Name of object to pick
            location_name: Name of location to place
            use_gripper: If True, turn suction ON at pick (default). If False, only move to pick position and up (no grip).
            go_direct_to_place: If True, after pick do not return home — go straight to place position. If False, go home between pick and place.
            
        Returns:
            True if successful
        """
        print("\n" + "="*60)
        print(f"🤖 PICK AND PLACE: {object_name} → {location_name}")
        print(f"   Gripper at pick: {'ON' if use_gripper else 'OFF'}")
        print(f"   After pick: {'Go directly to place' if go_direct_to_place else 'Go home then place'}")
        print("="*60)
        
        try:
            # Go to home first
            if not self.go_home():
                print("✗ Failed to go home")
                return False
            
            # Pick object (with or without gripper)
            if not self.pick_object(object_name, use_gripper=use_gripper):
                print("✗ Pick failed")
                self.gripper.emergency_release()
                self.go_home()
                return False
            
            # Between pick and place: go home or go directly to place
            if not go_direct_to_place:
                if not self.go_home():
                    print("✗ Failed to go home after pick")
                    self.gripper.emergency_release()
                    return False
            
            # Place object
            if not self.place_object(location_name):
                print("✗ Place failed")
                self.gripper.emergency_release()
                self.go_home()
                return False
            
            # Return home
            if not self.go_home():
                print("✗ Failed to return home")
                return False
            
            print("\n" + "="*60)
            print("✅ PICK AND PLACE COMPLETE")
            print("="*60 + "\n")
            return True
            
        except Exception as e:
            print(f"\n✗ Error during pick and place: {e}")
            self.gripper.emergency_release()
            self.go_home()
            return False


# Example usage
if __name__ == "__main__":
    import os
    
    # Get config path (relative to script location) - try JSON first, then YAML
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.join(script_dir, '..', 'config')
    json_path = os.path.normpath(os.path.join(config_dir, 'robot_config.json'))
    yaml_path = os.path.normpath(os.path.join(config_dir, 'robot_config.yaml'))
    
    if os.path.exists(json_path):
        config_path = json_path
    elif os.path.exists(yaml_path):
        config_path = yaml_path
    else:
        config_path = json_path  # Default to JSON
    
    # Create robot controller
    robot = DobotE6Controller(ip="192.168.5.1")
    
    try:
        if robot.connect():
            # Create gripper
            gripper = SuctionGripper(robot, do_index=1)
            
            # Create pick and place manager
            pnp = PickAndPlace(robot, gripper, config_path)
            
            # Execute pick and place
            pnp.execute_pick_and_place("object_1", "destination_1")
            
        else:
            print("Failed to connect to robot")
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        if 'gripper' in locals():
            gripper.emergency_release()
    finally:
        robot.disconnect()
