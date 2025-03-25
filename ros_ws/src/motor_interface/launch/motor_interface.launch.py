from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to robot_control package
    robot_control_pkg = get_package_share_directory('robot_control')
    
    # Include the robot_control's bringup launch file
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(robot_control_pkg, 'launch', 'bringup.launch.py')
        ])
    )

    return LaunchDescription([
        bringup_launch
    ]) 