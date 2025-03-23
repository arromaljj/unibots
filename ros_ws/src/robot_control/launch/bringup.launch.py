#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Get the path to the package
    pkg_share = get_package_share_directory('robot_control')
    
    # Path to the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    
    # Get URDF via xacro
    robot_description_content = Command([
        'xacro ', urdf_file
    ])
    
    # Controller configuration file
    # controllers_file = os.path.join(pkg_share, 'config', 'tb6612fng_controllers.yaml')
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )
    
    # Controller manager
    # controller_manager_node = Node(
    #     name='controller_manager', # Explicitly name the node
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     output='screen',
    #     parameters=[
    #         {'robot_description': robot_description_content},
    #         controllers_file
    #     ],
    # )

    # Static transform publisher for IMU
    imu_static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='bno055_to_imu_link',
        arguments=['0', '0', '0', '0', '0', '0', 'imu_link', 'bno055'],
    )


    ekf_localisation = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf.yaml')
        ]
    )
    
    # Odometry publisher node
    # odom_publisher_node = Node(
    #     package='robot_control',
    #     executable='odom_publisher',
    #     output='screen',
    # )

    # Return the LaunchDescription
    return LaunchDescription([
        robot_state_publisher_node,
        # controller_manager_node,
        imu_static_transform_node,
        ekf_localisation,
    ])
