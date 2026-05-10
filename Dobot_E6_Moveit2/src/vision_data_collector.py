#!/usr/bin/env python3
"""
Vision Learning Data Collector
Automatically collect images with labels for vision-based learning
"""

import os
import json
import time
import cv2
import numpy as np
from datetime import datetime
from typing import Dict, List, Optional, Tuple
import yaml

# Import robot modules
from dobot_e6_controller import DobotE6Controller
from suction_gripper import SuctionGripper
from pick_place_logic import PickAndPlace


class CameraInterface:
    """Camera interface for image capture"""
    
    def __init__(self, camera_id: int = 0, width: int = 640, height: int = 480):
        """
        Initialize camera
        
        Args:
            camera_id: Camera device ID
            width: Image width
            height: Image height
        """
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.cap = None
        
    def open(self) -> bool:
        """Open camera"""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            
            if not self.cap.isOpened():
                print("✗ Failed to open camera")
                return False
            
            print(f"✓ Camera opened: {self.width}x{self.height}")
            return True
            
        except Exception as e:
            print(f"✗ Camera error: {e}")
            return False
    
    def capture(self) -> Optional[np.ndarray]:
        """
        Capture single frame
        
        Returns:
            RGB image or None if failed
        """
        if not self.cap or not self.cap.isOpened():
            return None
        
        ret, frame = self.cap.read()
        if ret:
            # Convert BGR to RGB
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return None
    
    def close(self):
        """Close camera"""
        if self.cap:
            self.cap.release()
            print("Camera closed")


class VisionDataCollector:
    """Automated vision learning data collection"""
    
    def __init__(self, robot: DobotE6Controller, gripper: SuctionGripper, 
                 camera: CameraInterface, output_dir: str):
        """
        Initialize data collector
        
        Args:
            robot: Robot controller
            gripper: Gripper controller
            camera: Camera interface
            output_dir: Output directory for dataset
        """
        self.robot = robot
        self.gripper = gripper
        self.camera = camera
        self.output_dir = output_dir
        
        # Create directories
        self.images_dir = os.path.join(output_dir, 'images')
        self.labels_dir = os.path.join(output_dir, 'labels')
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.labels_dir, exist_ok=True)
        
        self.dataset = []
        self.sample_count = 0
        
    def capture_sample(self, object_name: str, object_class: str, 
                      object_pos: Dict, robot_pose: List[float]) -> bool:
        """
        Capture single training sample
        
        Args:
            object_name: Object identifier
            object_class: Object class label
            object_pos: Object position {'x', 'y', 'z', 'rx', 'ry', 'rz'}
            robot_pose: Current robot pose [x, y, z, rx, ry, rz]
            
        Returns:
            True if successful
        """
        # Capture image
        image = self.camera.capture()
        if image is None:
            print("✗ Failed to capture image")
            return False
        
        # Generate filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        sample_id = f"{timestamp}_{self.sample_count:04d}"
        image_filename = f"{sample_id}.jpg"
        label_filename = f"{sample_id}.json"
        
        # Save image
        image_path = os.path.join(self.images_dir, image_filename)
        cv2.imwrite(image_path, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        
        # Create label
        label = {
            'sample_id': sample_id,
            'timestamp': timestamp,
            'object': {
                'name': object_name,
                'class': object_class,
                'position': object_pos
            },
            'robot': {
                'pose': {
                    'x': robot_pose[0],
                    'y': robot_pose[1],
                    'z': robot_pose[2],
                    'rx': robot_pose[3],
                    'ry': robot_pose[4],
                    'rz': robot_pose[5]
                }
            },
            'camera': {
                'width': self.camera.width,
                'height': self.camera.height
            },
            'image_file': image_filename
        }
        
        # Save label
        label_path = os.path.join(self.labels_dir, label_filename)
        with open(label_path, 'w') as f:
            json.dump(label, f, indent=2)
        
        # Add to dataset
        self.dataset.append(label)
        self.sample_count += 1
        
        print(f"✓ Sample {self.sample_count} captured: {object_name} ({object_class})")
        return True
    
    def collect_pick_sequence(self, object_name: str, object_class: str,
                             object_pos: Dict, num_views: int = 5) -> bool:
        """
        Collect multiple views of pick sequence
        
        Args:
            object_name: Object identifier
            object_class: Object class
            object_pos: Object position
            num_views: Number of viewpoints to capture
            
        Returns:
            True if successful
        """
        print(f"\n📸 Collecting data for: {object_name}")
        
        # Approach position
        approach_z = object_pos['z'] + 150
        
        # Capture from different angles around object
        angles = np.linspace(0, 2*np.pi, num_views, endpoint=False)
        radius = 50  # mm from object center
        
        for i, angle in enumerate(angles):
            # Calculate viewpoint
            view_x = object_pos['x'] + radius * np.cos(angle)
            view_y = object_pos['y'] + radius * np.sin(angle)
            
            # Move to viewpoint
            self.robot.move_j(view_x, view_y, approach_z, 
                            object_pos['rx'], object_pos['ry'], object_pos['rz'])
            self.robot.wait_for_motion_complete()
            
            # Wait for stabilization
            time.sleep(0.5)
            
            # Get robot pose
            pose = self.robot.get_pose()
            if pose is None:
                pose = [view_x, view_y, approach_z, 
                       object_pos['rx'], object_pos['ry'], object_pos['rz']]
            
            # Capture sample
            self.capture_sample(object_name, object_class, object_pos, pose)
        
        print(f"✓ Collected {num_views} views of {object_name}")
        return True
    
    def automated_collection(self, config_file: str, num_samples_per_object: int = 10):
        """
        Fully automated data collection
        
        Args:
            config_file: Path to robot_config.yaml
            num_samples_per_object: Samples to collect per object
        """
        print("\n" + "="*60)
        print("🤖 AUTOMATED VISION DATA COLLECTION")
        print("="*60 + "\n")
        
        # Load configuration
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        
        pick_locations = config['positions']['pick_locations']
        
        # Collect data for each object
        for obj_name, obj_pos in pick_locations.items():
            print(f"\n📦 Object: {obj_name}")
            
            # Determine object class (you can customize this)
            object_class = obj_name.split('_')[0]  # e.g., "object" from "object_1"
            
            # Collect multiple samples
            for sample in range(num_samples_per_object):
                print(f"  Sample {sample+1}/{num_samples_per_object}")
                self.collect_pick_sequence(obj_name, object_class, obj_pos, num_views=5)
            
            # Return to home between objects
            self.robot.move_j(config['positions']['home']['x'],
                            config['positions']['home']['y'],
                            config['positions']['home']['z'],
                            config['positions']['home']['rx'],
                            config['positions']['home']['ry'],
                            config['positions']['home']['rz'])
            self.robot.wait_for_motion_complete()
        
        # Save dataset metadata
        self.save_metadata()
        
        print("\n" + "="*60)
        print(f"✅ DATA COLLECTION COMPLETE")
        print(f"   Total samples: {self.sample_count}")
        print(f"   Images: {self.images_dir}")
        print(f"   Labels: {self.labels_dir}")
        print("="*60 + "\n")
    
    def save_metadata(self):
        """Save dataset metadata"""
        metadata = {
            'dataset_name': os.path.basename(self.output_dir),
            'creation_date': datetime.now().isoformat(),
            'total_samples': self.sample_count,
            'num_objects': len(set(s['object']['name'] for s in self.dataset)),
            'classes': list(set(s['object']['class'] for s in self.dataset)),
            'image_size': {
                'width': self.camera.width,
                'height': self.camera.height
            },
            'samples': self.dataset
        }
        
        metadata_path = os.path.join(self.output_dir, 'dataset_metadata.json')
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        print(f"✓ Metadata saved: {metadata_path}")


def main():
    """Main entry point"""
    import sys
    
    # Configuration
    robot_ip = "192.168.1.6"
    camera_id = 0
    output_dir = "/home/sunbi/Dobot_E6_Moveit2/datasets/vision_data"
    config_file = "/home/sunbi/Dobot_E6_Moveit2/config/robot_config.yaml"
    
    # Number of samples
    samples_per_object = 10
    
    print("Vision Learning Data Collector")
    print("="*60)
    
    # Initialize robot
    robot = DobotE6Controller(ip=robot_ip)
    if not robot.connect():
        print("✗ Failed to connect to robot")
        return
    
    # Initialize gripper
    gripper = SuctionGripper(robot, do_index=1)
    
    # Initialize camera
    camera = CameraInterface(camera_id=camera_id)
    if not camera.open():
        print("✗ Failed to open camera")
        robot.disconnect()
        return
    
    try:
        # Create data collector
        collector = VisionDataCollector(robot, gripper, camera, output_dir)
        
        # Run automated collection
        collector.automated_collection(config_file, samples_per_object)
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        camera.close()
        robot.disconnect()
        print("Shutdown complete")


if __name__ == '__main__':
    main()
