#!/usr/bin/env bash
set -e

# ---- Ensure ROS 2 Jazzy exists ----
if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
  echo "ROS 2 Jazzy is not installed."
  exit 1
fi

# ---- Source ROS 2 ----
source /opt/ros/jazzy/setup.bash

# ---- Determine workspace location (repo-relative) ----
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$REPO_DIR/microros_ws"

# ---- Create workspace ----
mkdir -p "$WS_DIR/src"
cd "$WS_DIR"

# ---- Clone micro-ROS setup tools (once) ----
if [ ! -d "src/micro_ros_setup" ]; then
  git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
fi

# ---- Install dependencies ----
sudo apt update
sudo apt install -y python3-pip python3-rosdep

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -y

# ---- Build micro-ROS tools ----
colcon build
source install/local_setup.bash

# ---- Create firmware workspace ----
ros2 run micro_ros_setup create_firmware_ws.sh host

# ---- Build firmware tools ----
ros2 run micro_ros_setup build_firmware.sh

# ---- Create micro-ROS agent workspace ----
ros2 run micro_ros_setup create_agent_ws.sh

# ---- Build agent ----
ros2 run micro_ros_setup build_agent.sh

# ---- Add environment to bashrc (no duplicates) ----
grep -Fqx "source /opt/ros/jazzy/setup.bash" ~/.bashrc || \
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

grep -Fqx "source $WS_DIR/install/local_setup.bash" ~/.bashrc || \
echo "source $WS_DIR/install/local_setup.bash" >> ~/.bashrc

echo ""
echo "Setup complete."
echo "Workspace: $WS_DIR"
echo ""
echo "Run: source ~/.bashrc"
echo "Then start the agent with:"
echo "  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200"