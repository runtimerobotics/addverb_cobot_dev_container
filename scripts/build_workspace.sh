#!/usr/bin/env bash
# Rebuild the cobot_ros2 workspace inside the container
set -euo pipefail

source /opt/ros/humble/setup.bash
cd ~/cobot_ros2_ws
colcon build --symlink-install
source install/setup.bash
echo "Workspace built successfully."
