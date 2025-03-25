#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the path to the package
    pkg_share = get_package_share_directory('robot_control')
    
    # Include the control launch file
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'control.launch.py')
        ])
    )

    # Static transform publisher for IMU
    imu_static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='bno055_to_imu_link',
        arguments=['0', '0', '0', '0', '0', '0', 'imu_link', 'bno055'],
    )

    # # EKF localization
    # ekf_localisation = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[
    #         os.path.join(pkg_share, 'config', 'ekf.yaml')
    #     ]
    # )

    # Return the LaunchDescription
    return LaunchDescription([
        control_launch,  # Include the control launch
        imu_static_transform_node,
        # ekf_localisation,
    ])
