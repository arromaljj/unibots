.PHONY: build foxglox imu

build:
	colcon build

foxglove:
	ros2 launch foxglove_bridge foxglove_bridge_launch.xml

imu:
	ros2 launch bno055 bno055.launch.py
