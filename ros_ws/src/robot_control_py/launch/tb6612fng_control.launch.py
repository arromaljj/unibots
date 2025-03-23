#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the path to the package
    pkg_share = get_package_share_directory('robot_control_py')
    
    # Log the package directory for debugging
    log_pkg_path = LogInfo(msg=f'robot_control_py package path: {pkg_share}')
    
    # Path to the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    log_urdf_path = LogInfo(msg=f'URDF file path: {urdf_file}')
    
    # Read the URDF file content
    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()
    
    # Controller configuration file
    controllers_file = os.path.join(pkg_share, 'config', 'tb6612fng_controllers.yaml')
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'publish_frequency': 30.0
        }]
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            controllers_file
        ],
    )
    
    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=['joint_state_broadcaster', 
                   '--controller-manager', 
                   '/controller_manager'],
    )
    
    # Differential drive controller
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='diff_drive_controller_spawner',
        output='screen',
        arguments=['diff_drive_controller', 
                   '--controller-manager', 
                   '/controller_manager'],
    )
    
    # Define an event handler to ensure joint_state_broadcaster is launched first
    controller_sequence = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_drive_controller],
        )
    )
    
    # Return the LaunchDescription with controllers last to ensure robot_description is available
    return LaunchDescription([
        log_pkg_path,
        log_urdf_path,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        controller_sequence
    ]) 