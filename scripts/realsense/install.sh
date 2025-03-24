#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

echo "============================================"
echo "RealSense ROS2 Installation Script Starting"
echo "============================================"
echo ""

##############################################
# Step 1: Install Dependencies
##############################################
echo "Step 1: Installing required dependencies..."
sudo apt-get update
sudo apt-get install -y libssl-dev libusb-1.0-0-dev libudev-dev pkg-config git wget cmake build-essential libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
echo "Dependencies installed successfully."
echo ""

##############################################
# Step 2: Clone and Build librealsense SDK
##############################################
echo "Step 2: Setting up librealsense SDK (v2.54.1)..."
cd ~
if [ -d "librealsense" ]; then
    echo "Directory 'librealsense' exists. Ensuring correct version..."
    cd librealsense
    git fetch
    git checkout v2.54.1
else
    git clone https://github.com/IntelRealSense/librealsense.git
    cd librealsense
    git checkout v2.54.1
fi

# Patch the version.h file to include cstdint
echo "Patching version.h file..."
sed -i '1i#include <cstdint>' third-party/rsutils/include/rsutils/version.h

echo "Setting up udev rules for RealSense camera..."
sudo ./scripts/setup_udev_rules.sh

echo "Creating build directory..."
mkdir -p build && cd build
echo "Configuring the librealsense SDK..."
cmake -DBUILD_GRAPHICAL_EXAMPLES=false \
      -DBUILD_PYTHON_BINDINGS=true \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 \
      -DFORCE_RSUSB_BACKEND=true \
      -DCMAKE_BUILD_TYPE=Release ../

echo "Building librealsense (this may take a while)..."
make -j4

echo "Installing librealsense SDK..."
sudo make install
echo "librealsense SDK installed successfully."
echo ""

##############################################
# Step 3: Set Up ROS2 Workspace for RealSense ROS Wrapper
##############################################
echo "Step 3: Setting up ROS2 workspace for RealSense ROS Wrapper..."
cd ~
mkdir -p ros2_ws/src
cd ros2_ws/src

if [ -d "realsense-ros" ]; then
    echo "Directory 'realsense-ros' exists. Updating repository..."
    cd realsense-ros
    git pull
    cd ..
else
    git clone https://github.com/IntelRealSense/realsense-ros.git
fi

echo "Building the ROS2 workspace with colcon (this may take a few minutes)..."
cd ~/ros2_ws
colcon build

echo "Sourcing the workspace..."
# Automatically source the workspace in new shell sessions:
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/ros2_ws/install/setup.bash
echo "ROS2 workspace built and sourced successfully."
echo ""

##############################################
# Post-installation Instructions
##############################################
echo "============================================"
echo "Installation Completed Successfully!"
echo "============================================"
echo ""
echo "Post-installation instructions:"
echo "1. Connect your RealSense camera to a USB3 port (for best performance)."
echo "2. If you run into device permission issues, re-run the udev rules script from ~/librealsense/scripts/setup_udev_rules.sh."
echo "3. To launch the RealSense ROS2 node, you can use (adjust parameters as needed):"
echo "   ros2 launch realsense2_camera rs_launch.py depth_module.profile:=424x240x6 rgb_camera.profile:=424x240x6 pointcloud.enable:=true initial_reset:=true"
echo "4. For visualization, run RViz2:"
echo "   ros2 run rviz2 rviz2"
echo "   and add topics like /camera/color/image_raw or /camera/depth/color/points."
echo "5. Check the official RealSense ROS2 documentation and GitHub issues for troubleshooting if needed."
echo ""
echo "Script execution completed."
