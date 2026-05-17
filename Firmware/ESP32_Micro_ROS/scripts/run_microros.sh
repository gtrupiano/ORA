#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")/microros_ws"

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/local_setup.bash"

DEV="${1:-/dev/ttyUSB0}"
BAUD="${2:-115200}"

while true; do
    echo "Starting micro-ROS agent on $DEV at $BAUD baud..."

    ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEV" -b "$BAUD"

    echo "micro-ROS agent exited. Restarting in 2 seconds..."
    sleep 2
done