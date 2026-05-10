#!/usr/bin/env python3
"""
Dobot E6 MoveIt2 Control Script
================================

This script provides an interface to control the Dobot E6 robot arm using MoveIt2.
It demonstrates various control methods including:
- Joint space control
- Cartesian space control (pose goals)
- Predefined pose movements
- Current state queries

Author: Generated for Dobot E6 robot arm
Date: 2026-01-29
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from moveit2 import MoveIt2Interface
from geometry_msgs.msg import PoseStamped, Pose
from math import pi
import time


class DobotE6Controller(Node):
    """Controller class for Dobot E6 robot arm using MoveIt2"""
    
    def __init__(self):
        super().__init__('dobot_e6_controller')
        
        # Initialize MoveIt2 interface
        self.get_logger().info('Initializing MoveIt2 interface for Dobot E6...')
        
        # Planning group name (should match the group defined in MoveIt Setup Assistant)
        self.planning_group = "arm"  # 또는 "manipulator" - MoveIt 설정에 따라 변경
        
        # Initialize MoveIt2 (간단한 버전)
        try:
            from moveit.planning import MoveItPy
            self.moveit = MoveItPy(node=self)
            self.arm = self.moveit.get_planning_component(self.planning_group)
            self.get_logger().info(f'Successfully initialized MoveIt2 for group: {self.planning_group}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MoveIt2: {e}')
            self.get_logger().warn('Make sure MoveIt2 configuration is properly set up')
            
        # Joint names for the E6 robot
        self.joint_names = [
            'joint1',
            'joint2', 
            'joint3',
            'joint4',
            'joint5',
            'joint6'
        ]
        
        self.get_logger().info('Dobot E6 Controller initialized successfully')
        
    def get_current_joint_values(self):
        """Get current joint positions"""
        try:
            joint_state = self.arm.get_current_state()
            current_joints = joint_state.get_joint_group_positions(self.planning_group)
            
            self.get_logger().info('Current joint values:')
            for i, (name, value) in enumerate(zip(self.joint_names, current_joints)):
                self.get_logger().info(f'  {name}: {value:.4f} rad ({value*180/pi:.2f} deg)')
            
            return current_joints
        except Exception as e:
            self.get_logger().error(f'Failed to get current joint values: {e}')
            return None
    
    def get_current_pose(self):
        """Get current end-effector pose"""
        try:
            current_pose = self.arm.get_current_pose()
            
            self.get_logger().info('Current end-effector pose:')
            self.get_logger().info(f'  Position: x={current_pose.position.x:.4f}, '
                                 f'y={current_pose.position.y:.4f}, '
                                 f'z={current_pose.position.z:.4f}')
            self.get_logger().info(f'  Orientation: x={current_pose.orientation.x:.4f}, '
                                 f'y={current_pose.orientation.y:.4f}, '
                                 f'z={current_pose.orientation.z:.4f}, '
                                 f'w={current_pose.orientation.w:.4f}')
            
            return current_pose
        except Exception as e:
            self.get_logger().error(f'Failed to get current pose: {e}')
            return None
    
    def move_to_joint_values(self, joint_values, execute=True):
        """
        Move to specified joint values
        
        Args:
            joint_values: List of 6 joint angles in radians
            execute: If True, execute the motion. If False, only plan.
        """
        if len(joint_values) != 6:
            self.get_logger().error('Joint values must contain exactly 6 values')
            return False
        
        try:
            self.get_logger().info('Planning motion to joint values...')
            for i, (name, value) in enumerate(zip(self.joint_names, joint_values)):
                self.get_logger().info(f'  {name}: {value:.4f} rad ({value*180/pi:.2f} deg)')
            
            # Set joint value target
            self.arm.set_goal_state(configuration_name=None,
                                   joint_state={name: value for name, value in 
                                              zip(self.joint_names, joint_values)})
            
            # Plan
            plan_result = self.arm.plan()
            
            if plan_result:
                self.get_logger().info('Planning successful!')
                
                if execute:
                    self.get_logger().info('Executing motion...')
                    self.arm.execute()
                    self.get_logger().info('Motion completed!')
                    return True
                else:
                    self.get_logger().info('Planning only - not executing')
                    return True
            else:
                self.get_logger().error('Planning failed!')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Failed to move to joint values: {e}')
            return False
    
    def move_to_pose(self, position, orientation=None, execute=True):
        """
        Move to specified end-effector pose
        
        Args:
            position: [x, y, z] in meters
            orientation: [x, y, z, w] quaternion, or None for current orientation
            execute: If True, execute the motion. If False, only plan.
        """
        try:
            target_pose = Pose()
            target_pose.position.x = position[0]
            target_pose.position.y = position[1]
            target_pose.position.z = position[2]
            
            if orientation is None:
                # Keep current orientation
                current_pose = self.get_current_pose()
                if current_pose:
                    target_pose.orientation = current_pose.orientation
                else:
                    # Default orientation
                    target_pose.orientation.w = 1.0
            else:
                target_pose.orientation.x = orientation[0]
                target_pose.orientation.y = orientation[1]
                target_pose.orientation.z = orientation[2]
                target_pose.orientation.w = orientation[3]
            
            self.get_logger().info('Planning motion to pose...')
            self.get_logger().info(f'  Position: x={position[0]:.4f}, '
                                 f'y={position[1]:.4f}, z={position[2]:.4f}')
            
            # Set pose target
            self.arm.set_goal_state(pose_stamped_msg=PoseStamped(pose=target_pose),
                                   pose_link="Link6")  # End-effector link name
            
            # Plan
            plan_result = self.arm.plan()
            
            if plan_result:
                self.get_logger().info('Planning successful!')
                
                if execute:
                    self.get_logger().info('Executing motion...')
                    self.arm.execute()
                    self.get_logger().info('Motion completed!')
                    return True
                else:
                    self.get_logger().info('Planning only - not executing')
                    return True
            else:
                self.get_logger().error('Planning failed!')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Failed to move to pose: {e}')
            return False
    
    def move_home(self):
        """Move to home position (all joints at 0)"""
        self.get_logger().info('Moving to home position...')
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return self.move_to_joint_values(home_joints)
    
    def demo_sequence(self):
        """Execute a demonstration movement sequence"""
        self.get_logger().info('='*60)
        self.get_logger().info('Starting demonstration sequence')
        self.get_logger().info('='*60)
        
        # 1. Get current state
        self.get_logger().info('\n--- Step 1: Current State ---')
        self.get_current_joint_values()
        self.get_current_pose()
        time.sleep(1)
        
        # 2. Move to home position
        self.get_logger().info('\n--- Step 2: Moving to Home Position ---')
        self.move_home()
        time.sleep(2)
        
        # 3. Move to example pose 1
        self.get_logger().info('\n--- Step 3: Moving to Pose 1 ---')
        pose1_joints = [pi/4, -pi/6, pi/3, 0.0, pi/4, 0.0]
        self.move_to_joint_values(pose1_joints)
        time.sleep(2)
        
        # 4. Move to example pose 2
        self.get_logger().info('\n--- Step 4: Moving to Pose 2 ---')
        pose2_joints = [-pi/4, pi/6, -pi/3, pi/6, -pi/4, pi/2]
        self.move_to_joint_values(pose2_joints)
        time.sleep(2)
        
        # 5. Return to home
        self.get_logger().info('\n--- Step 5: Returning to Home ---')
        self.move_home()
        
        self.get_logger().info('='*60)
        self.get_logger().info('Demonstration sequence completed!')
        self.get_logger().info('='*60)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    controller = DobotE6Controller()
    
    print("\n" + "="*60)
    print("Dobot E6 MoveIt2 Controller")
    print("="*60)
    print("\nAvailable commands:")
    print("  1 - Get current joint values")
    print("  2 - Get current pose")
    print("  3 - Move to home position")
    print("  4 - Move to custom joint values")
    print("  5 - Move to custom pose")
    print("  6 - Run demonstration sequence")
    print("  q - Quit")
    print("="*60 + "\n")
    
    try:
        while rclpy.ok():
            choice = input("Enter command: ").strip().lower()
            
            if choice == 'q':
                break
            elif choice == '1':
                controller.get_current_joint_values()
            elif choice == '2':
                controller.get_current_pose()
            elif choice == '3':
                controller.move_home()
            elif choice == '4':
                print("\nEnter 6 joint values in radians (space-separated):")
                try:
                    values = input().strip().split()
                    joint_values = [float(v) for v in values]
                    controller.move_to_joint_values(joint_values)
                except ValueError:
                    print("Invalid input. Please enter 6 numbers.")
            elif choice == '5':
                print("\nEnter target position (x y z in meters):")
                try:
                    values = input().strip().split()
                    position = [float(v) for v in values]
                    if len(position) == 3:
                        controller.move_to_pose(position)
                    else:
                        print("Invalid input. Please enter 3 numbers.")
                except ValueError:
                    print("Invalid input. Please enter 3 numbers.")
            elif choice == '6':
                controller.demo_sequence()
            else:
                print("Invalid command. Please try again.")
            
            print()  # Empty line for readability
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
