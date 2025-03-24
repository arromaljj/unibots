from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_calibration',
            executable='imu_drift_calculator.py',
            name='imu_drift_calculator',
            output='screen'
        ),
        Node(
            package='imu_calibration',
            executable='cumulative_drift_calculator.py',
            name='cumulative_drift_calculator',
            output='screen'
        )
    ]) 