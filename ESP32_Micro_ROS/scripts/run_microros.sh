#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")/microros_ws"

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/local_setup.bash"

ros2 run micro_ros_agent micro_ros_agent serial --dev "${1:-/dev/ttyUSB0}" -b "${2:-115200}"
