#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package
    pkg_share = get_package_share_directory('robot_control_py')
    
    # Path to the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    
    # Read the URDF file content
    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()
    
    # Controller configuration file (NO HARDWARE INTERFACE)
    controllers_file = os.path.join(pkg_share, 'config', 'tb6612fng_controllers_no_hw.yaml')
    
    # Controller manager
    controller_manager_node = Node(
        name='controller_manager', # Explicitly name the node
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            controllers_file
        ],
    )
    
    # Return the LaunchDescription
    return LaunchDescription([
        controller_manager_node,
    ])