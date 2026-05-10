#!/usr/bin/env python3
"""
Reinforcement Learning Data Collector
Automatically collect state-action-reward trajectories for RL training
"""

import os
import pickle
import json
import time
import numpy as np
from datetime import datetime
from typing import Dict, List, Tuple, Optional
import yaml
from collections import deque

# Import robot modules
from dobot_e6_controller import DobotE6Controller
from suction_gripper import SuctionGripper


class RobotEnvironment:
    """Gym-like environment for robot pick-and-place"""
    
    def __init__(self, robot: DobotE6Controller, gripper: SuctionGripper, config: Dict):
        """
        Initialize environment
        
        Args:
            robot: Robot controller
            gripper: Gripper controller
            config: Configuration dictionary
        """
        self.robot = robot
        self.gripper = gripper
        self.config = config
        
        # Environment parameters
        self.pick_locations = config['positions']['pick_locations']
        self.place_locations = config['positions']['place_locations']
        self.home_pos = config['positions']['home']
        
        # State tracking
        self.current_object = None
        self.current_destination = None
        self.has_object = False
        self.episode_step = 0
        self.max_steps = 100
        
    def reset(self) -> np.ndarray:
        """
        Reset environment to initial state
        
        Returns:
            Initial state observation
        """
        # Go to home position
        self.robot.move_j(self.home_pos['x'], self.home_pos['y'], self.home_pos['z'],
                         self.home_pos['rx'], self.home_pos['ry'], self.home_pos['rz'])
        self.robot.wait_for_motion_complete()
        
        # Release gripper
        self.gripper.release()
        
        # Select random object and destination
        obj_names = list(self.pick_locations.keys())
        dest_names = list(self.place_locations.keys())
        
        self.current_object = np.random.choice(obj_names)
        self.current_destination = np.random.choice(dest_names)
        self.has_object = False
        self.episode_step = 0
        
        print(f"🔄 Reset: {self.current_object} → {self.current_destination}")
        
        return self.get_state()
    
    def get_state(self) -> np.ndarray:
        """
        Get current state
        
        Returns:
            State vector [robot_pose(6), object_pos(3), goal_pos(3), has_object(1)]
        """
        # Get robot pose
        pose = self.robot.get_pose()
        if pose is None:
            pose = [0, 0, 0, 0, 0, 0]
        
        # Get object position
        obj_pos = self.pick_locations[self.current_object]
        object_xyz = [obj_pos['x'], obj_pos['y'], obj_pos['z']]
        
        # Get goal position
        goal_pos = self.place_locations[self.current_destination]
        goal_xyz = [goal_pos['x'], goal_pos['y'], goal_pos['z']]
        
        # Combine into state vector
        state = np.array(pose[:6] + object_xyz + goal_xyz + [float(self.has_object)])
        
        return state
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
        """
        Execute action and return transition
        
        Args:
            action: Action vector [dx, dy, dz, drx, dry, drz, gripper_action]
                    - dx, dy, dz: relative movement in mm
                    - drx, dry, drz: relative rotation in degrees
                    - gripper_action: 0=no change, 1=grip, -1=release
        
        Returns:
            (next_state, reward, done, info)
        """
        self.episode_step += 1
        
        # Get current pose
        current_pose = self.robot.get_pose()
        if current_pose is None:
            current_pose = [0, 0, 0, 0, 0, 0]
        
        # Apply action (relative movement)
        new_pose = [
            current_pose[0] + action[0],
            current_pose[1] + action[1],
            current_pose[2] + action[2],
            current_pose[3] + action[3],
            current_pose[4] + action[4],
            current_pose[5] + action[5]
        ]
        
        # Execute movement
        self.robot.move_l(new_pose[0], new_pose[1], new_pose[2],
                         new_pose[3], new_pose[4], new_pose[5])
        self.robot.wait_for_motion_complete(timeout=10.0)
        
        # Execute gripper action
        if action[6] > 0.5:  # Grip
            self.gripper.grip()
            self.has_object = True
        elif action[6] < -0.5:  # Release
            self.gripper.release()
            self.has_object = False
        
        # Get next state
        next_state = self.get_state()
        
        # Calculate reward
        reward, done, info = self.calculate_reward(next_state)
        
        # Check max steps
        if self.episode_step >= self.max_steps:
            done = True
            info['timeout'] = True
        
        return next_state, reward, done, info
    
    def calculate_reward(self, state: np.ndarray) -> Tuple[float, bool, Dict]:
        """
        Calculate reward based on current state
        
        Args:
            state: Current state vector
            
        Returns:
            (reward, done, info)
        """
        robot_pos = state[:3]
        object_pos = state[6:9]
        goal_pos = state[9:12]
        has_object = state[12]
        
        reward = 0.0
        done = False
        info = {}
        
        if not has_object:
            # Before picking: reward for getting close to object
            dist_to_object = np.linalg.norm(robot_pos - object_pos)
            reward = -0.01 * dist_to_object
            
            # Bonus for being very close
            if dist_to_object < 20:  # within 20mm
                reward += 1.0
                
        else:
            # After picking: reward for getting close to goal
            dist_to_goal = np.linalg.norm(robot_pos - goal_pos)
            reward = -0.01 * dist_to_goal
            
            # Bonus for being very close
            if dist_to_goal < 20:  # within 20mm
                reward += 1.0
            
            # Success: at goal with object
            if dist_to_goal < 30:
                reward += 10.0
                done = True
                info['success'] = True
                print("✅ Success!")
        
        # Penalty for collision or out of bounds
        if robot_pos[2] < 0 or robot_pos[2] > 600:
            reward -= 5.0
            done = True
            info['collision'] = True
        
        return reward, done, info


class RLDataCollector:
    """Reinforcement learning data collector"""
    
    def __init__(self, env: RobotEnvironment, output_dir: str):
        """
        Initialize RL data collector
        
        Args:
            env: Robot environment
            output_dir: Output directory for dataset
        """
        self.env = env
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        self.episodes = []
        self.current_episode = []
        self.episode_count = 0
        
    def collect_random_episode(self, max_steps: int = 50) -> Dict:
        """
        Collect one episode with random actions
        
        Args:
            max_steps: Maximum steps per episode
            
        Returns:
            Episode data dictionary
        """
        print(f"\n📊 Episode {self.episode_count + 1}")
        
        # Reset environment
        state = self.env.reset()
        
        episode_data = {
            'episode_id': self.episode_count,
            'timestamp': datetime.now().isoformat(),
            'transitions': [],
            'total_reward': 0.0,
            'success': False,
            'steps': 0
        }
        
        done = False
        step = 0
        
        while not done and step < max_steps:
            # Random action
            action = np.random.randn(7) * 10  # Random movements
            action[6] = np.random.choice([-1, 0, 1])  # Random gripper
            
            # Execute action
            next_state, reward, done, info = self.env.step(action)
            
            # Record transition
            transition = {
                'step': step,
                'state': state.tolist(),
                'action': action.tolist(),
                'reward': float(reward),
                'next_state': next_state.tolist(),
                'done': done,
                'info': info
            }
            
            episode_data['transitions'].append(transition)
            episode_data['total_reward'] += reward
            
            # Update state
            state = next_state
            step += 1
            
            print(f"  Step {step}: reward={reward:.2f}, done={done}")
        
        episode_data['steps'] = step
        episode_data['success'] = info.get('success', False)
        
        self.episodes.append(episode_data)
        self.episode_count += 1
        
        print(f"  📈 Total reward: {episode_data['total_reward']:.2f}")
        print(f"  {'✅ Success' if episode_data['success'] else '❌ Failure'}")
        
        return episode_data
    
    def automated_collection(self, num_episodes: int = 100):
        """
        Automatically collect multiple episodes
        
        Args:
            num_episodes: Number of episodes to collect
        """
        print("\n" + "="*60)
        print("🤖 AUTOMATED RL DATA COLLECTION")
        print("="*60 + "\n")
        
        successes = 0
        
        for ep in range(num_episodes):
            try:
                episode_data = self.collect_random_episode()
                
                if episode_data['success']:
                    successes += 1
                
                # Save periodically
                if (ep + 1) % 10 == 0:
                    self.save_dataset()
                    success_rate = successes / (ep + 1) * 100
                    print(f"\n📊 Progress: {ep+1}/{num_episodes} episodes")
                    print(f"   Success rate: {success_rate:.1f}%\n")
                
            except KeyboardInterrupt:
                print("\n\nCollection interrupted by user")
                break
            except Exception as e:
                print(f"\n✗ Episode {ep+1} failed: {e}")
                continue
        
        # Final save
        self.save_dataset()
        
        print("\n" + "="*60)
        print("✅ RL DATA COLLECTION COMPLETE")
        print(f"   Total episodes: {self.episode_count}")
        print(f"   Success rate: {successes/self.episode_count*100:.1f}%")
        print(f"   Dataset: {self.output_dir}")
        print("="*60 + "\n")
    
    def save_dataset(self):
        """Save dataset to disk"""
        # Save as pickle (for numpy arrays)
        pickle_path = os.path.join(self.output_dir, 'rl_dataset.pkl')
        with open(pickle_path, 'wb') as f:
            pickle.dump(self.episodes, f)
        
        # Save as JSON (for human readability)
        json_path = os.path.join(self.output_dir, 'rl_dataset.json')
        with open(json_path, 'w') as f:
            json.dump(self.episodes, f, indent=2)
        
        # Save metadata
        metadata = {
            'dataset_name': os.path.basename(self.output_dir),
            'creation_date': datetime.now().isoformat(),
            'total_episodes': self.episode_count,
            'successful_episodes': sum(1 for ep in self.episodes if ep['success']),
            'total_transitions': sum(len(ep['transitions']) for ep in self.episodes),
            'state_dim': 13,  # 6 robot + 3 object + 3 goal + 1 gripper
            'action_dim': 7   # 6 delta pose + 1 gripper
        }
        
        metadata_path = os.path.join(self.output_dir, 'metadata.json')
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        print(f"✓ Dataset saved: {self.episode_count} episodes")


def main():
    """Main entry point"""
    # Configuration
    robot_ip = "192.168.1.6"
    output_dir = "/home/sunbi/Dobot_E6_Moveit2/datasets/rl_data"
    config_file = "/home/sunbi/Dobot_E6_Moveit2/config/robot_config.yaml"
    
    # Collection parameters
    num_episodes = 100
    
    print("Reinforcement Learning Data Collector")
    print("="*60)
    
    # Load configuration
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # Initialize robot
    robot = DobotE6Controller(ip=robot_ip)
    if not robot.connect():
        print("✗ Failed to connect to robot")
        return
    
    # Initialize gripper
    gripper = SuctionGripper(robot, do_index=1)
    
    try:
        # Create environment
        env = RobotEnvironment(robot, gripper, config)
        
        # Create data collector
        collector = RLDataCollector(env, output_dir)
        
        # Run automated collection
        collector.automated_collection(num_episodes)
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        gripper.release()
        robot.disconnect()
        print("Shutdown complete")


if __name__ == '__main__':
    main()
