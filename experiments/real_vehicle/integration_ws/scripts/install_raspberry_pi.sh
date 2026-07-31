#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-foxy}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "ROS 2 ${ROS_DISTRO} is not installed under /opt/ros." >&2
    exit 1
fi

sudo apt-get update
sudo apt-get install -y \
    "ros-${ROS_DISTRO}-ackermann-msgs" \
    "ros-${ROS_DISTRO}-diagnostic-msgs" \
    "ros-${ROS_DISTRO}-tf2-ros" \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-scipy \
    python3-yaml

source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${WORKSPACE_DIR}"
colcon build --symlink-install

echo "Raspberry Pi workspace built successfully."
echo "Hardware PWM remains disabled until pins and backend are confirmed."
