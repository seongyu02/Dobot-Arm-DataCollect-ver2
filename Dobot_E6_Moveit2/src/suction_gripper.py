#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Suction Gripper Controller
Controls suction gripper via Digital Output
"""

import sys
import os
import time
from typing import Optional

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

# Add parent directories to path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, current_dir)  # For local imports
sys.path.insert(0, parent_dir)    # For dobot_api.py

from dobot_e6_controller import DobotE6Controller


class SuctionGripper:
    """Suction gripper controller"""
    
    def __init__(self, robot: DobotE6Controller, do_index: int = 1):
        """
        Initialize suction gripper
        
        Args:
            robot: DobotE6Controller instance
            do_index: Digital output index for suction control
        """
        self.robot = robot
        self.do_index = do_index
        self.is_gripping = False
        
    def grip(self, wait_time: float = 0.5) -> bool:
        """
        Activate suction (grip)
        
        Args:
            wait_time: Time to wait for suction to stabilize (seconds)
            
        Returns:
            True if successful
        """
        success = self.robot.set_digital_output(self.do_index, True)
        
        if success:
            self.is_gripping = True
            print(f"🔵 Suction ON (waiting {wait_time}s)")
            time.sleep(wait_time)
            return True
        
        print("✗ Failed to activate suction")
        return False
    
    def release(self, wait_time: float = 0.3) -> bool:
        """
        Deactivate suction (release)
        
        Args:
            wait_time: Time to wait for release (seconds)
            
        Returns:
            True if successful
        """
        success = self.robot.set_digital_output(self.do_index, False)
        
        if success:
            self.is_gripping = False
            print(f"⚪ Suction OFF (waiting {wait_time}s)")
            time.sleep(wait_time)
            return True
        
        print("✗ Failed to deactivate suction")
        return False
    
    def check_grip(self) -> bool:
        """
        Check if object is gripped
        
        Note: This requires pressure sensor or vacuum sensor connected to DI
        For basic implementation, we assume grip is successful if suction is ON
        
        Returns:
            True if object is gripped
        """
        # TODO: Implement actual sensor reading if available
        # For now, return the suction state
        return self.is_gripping
    
    def emergency_release(self):
        """Emergency release - immediate suction off"""
        self.robot.set_digital_output(self.do_index, False)
        self.is_gripping = False
        print("⚠️ EMERGENCY RELEASE")


# Example usage
if __name__ == "__main__":
    from dobot_e6_controller import DobotE6Controller
    
    # Create robot controller
    robot = DobotE6Controller(ip="192.168.5.1")
    
    try:
        if robot.connect():
            # Create suction gripper
            gripper = SuctionGripper(robot, do_index=1)
            
            print("\n=== Testing suction gripper ===\n")
            
            # Test grip
            gripper.grip()
            time.sleep(2)
            
            # Test release
            gripper.release()
            
            print("\n=== Test complete ===\n")
        else:
            print("Failed to connect to robot")
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        if 'gripper' in locals():
            gripper.emergency_release()
    finally:
        robot.disconnect()
