# setup.sh
#!/usr/bin/env bash
set -e

# Installing ROS 2 and the micro-ROS build system

# Source the ROS 2 installation
source /opt/ros/jazzy/setup.bash

# Put microros_ws next to this script (repo/scripts/ -> repo/microros_ws)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$REPO_DIR/microros_ws"

# Create a workspace and download the micro-ROS tools
# Current workspace is 
# Project:
#	- ESP program
#	- microros_ws
#	- scripts
#		- setup_microros_ws.sh

mkdir -p "$WS_DIR/src"
cd "$WS_DIR"

# Clone once
if [ ! -d "src/micro_ros_setup" ]; then
  git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
fi

# Update dependencies using rosdep
sudo apt update
sudo apt install -y python3-pip python3-rosdep

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash


# Creating a new firmware workspace

# Create firmware step
ros2 run micro_ros_setup create_firmware_ws.sh host


# Building the firmware

# Build step
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash


# Creating the micro-ROS agent

# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

# Add micro-ROS environment to bashrc (optional)
# Optional: add to bashrc (no duplicates)
grep -Fqx "source /opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
grep -Fqx "source $WS_DIR/install/local_setup.bash" ~/.bashrc || echo "source $WS_DIR/install/local_setup.bash" >> ~/.bashrc

echo "DONE. Workspace: $WS_DIR"
echo "To run agent:"
echo "  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200"