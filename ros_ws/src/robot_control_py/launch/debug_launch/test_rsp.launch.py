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
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )
    
    # Return the LaunchDescription
    return LaunchDescription([
        robot_state_publisher_node,
    ])