.PHONY: build foxglox imu

build:
	colcon build

foxglove:
	ros2 launch foxglove_bridge foxglove_bridge_launch.xml

imu:
	ros2 launch bno055 bno055.launch.py

teleop:
	ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true

