#!/bin/bash

# Sudo password
PASSWORD="123"

# Change permissions of ttyUSB0
echo "$PASSWORD" | sudo -S chmod 666 /dev/ttyUSB0

# Build workspace
colcon build

# Source ROS environments
source /opt/ros/jazzy/setup.bash
source install/setup.sh

# Launch the node
ros2 launch botzilla_control final_test_ssh.launch.py