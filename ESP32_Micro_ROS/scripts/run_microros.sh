#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source ~/Documents/Github/ORA/ESP32_Micro_ROS/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200