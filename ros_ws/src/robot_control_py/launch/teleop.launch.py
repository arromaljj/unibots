#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('robot_control_py')
    
    # Create the launch configuration variables
    use_mock = LaunchConfiguration('use_mock')
    
    # Declare the launch arguments
    declare_use_mock_cmd = DeclareLaunchArgument(
        'use_mock',
        default_value='true',
        description='Flag to control whether to launch with mock hardware or real hardware')
    
    # Define the teleop node
    teleop_node = Node(
        package='robot_control_py',
        executable='teleop_keyboard.py',
        name='teleop_keyboard',
        output='screen',
    )
    
    # Define the robot launch files with conditions
    mock_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_dir, 'launch', 'tb6612fng_mock.launch.py')]),
        condition=IfCondition(use_mock)
    )
    
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_dir, 'launch', 'tb6612fng_control.launch.py')]),
        condition=UnlessCondition(use_mock)
    )
    
    # Create and return the launch description
    return LaunchDescription([
        declare_use_mock_cmd,
        mock_robot_launch,
        real_robot_launch,
        teleop_node,
    ]) 