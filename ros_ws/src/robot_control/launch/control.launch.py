#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Get the path to the package
    pkg_share = get_package_share_directory('robot_control')
    
    # Path to the URDF file 
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    
    # Generate URDF content
    robot_description_content = Command(
        ['xacro ', urdf_file]
    )
    
    # Path to the controller configuration
    diffdrive_config_file = os.path.join(pkg_share, 'config', 'diffdrive.yaml')
    
    # Robot state publisher (provides robot description and TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )
    
    # Initialize the controller manager
    # This loads our hardware interface and manages the controllers
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            diffdrive_config_file
        ],
        output='screen',
    )

    # Initialize the joint_state_broadcaster
    # This broadcasts the joint states even in open loop mode
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Initialize the diff_drive_controller
    # This converts cmd_vel into wheel commands
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Make the diff_drive_controller start after joint_state_broadcaster
    # This ensures proper initialization order
    diff_drive_delayed_spawn = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[diff_drive_controller_spawner],
        )
    )

    # Create and return launch description
    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_delayed_spawn
    ])
