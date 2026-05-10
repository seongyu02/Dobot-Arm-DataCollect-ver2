from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the directory of this package
    pkg_dir = os.path.dirname(os.path.realpath(__file__))
    pkg_dir = os.path.dirname(pkg_dir)  # Go up to package root
    
    # Paths to configuration files
    urdf_file = os.path.join(pkg_dir, 'urdf', 'me6_robot_fast.urdf')
    srdf_file = os.path.join(pkg_dir, 'config', 'me6_robot.srdf')
    kinematics_yaml = os.path.join(pkg_dir, 'config', 'kinematics.yaml')
    ompl_planning_yaml = os.path.join(pkg_dir, 'config', 'ompl_planning.yaml')
    joint_limits_yaml = os.path.join(pkg_dir, 'config', 'joint_limits.yaml')
    
    # Read URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Read SRDF file
    with open(srdf_file, 'r') as file:
        robot_description_semantic = file.read()
    
    # Robot description parameters
    robot_description_params = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
    }
    
    # Planning parameters
    planning_params = {
        'robot_description_planning': {
            'joint_limits': joint_limits_yaml,
        }
    }
    
    # MoveIt parameters
    moveit_params = {
        **robot_description_params,
        'robot_description_kinematics': kinematics_yaml,
        'planning_pipelines': ['ompl'],
        'ompl': ompl_planning_yaml,
    }
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_params],
    )
    
    # Joint State Publisher (for demo without real robot)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )
    
    # MoveIt Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[moveit_params],
    )
    
    # RViz with MoveIt
    rviz_config = os.path.join(pkg_dir, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[moveit_params],
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        move_group_node,
        rviz_node,
    ])
