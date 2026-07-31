#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros_env.sh"

exec ros2 launch real_vehicle_integration real_lidar.launch.py \
    lidar_udp_allowed_host:="${PI_LIDAR_HOST:-}" \
    start_rviz:="${START_RVIZ:-false}" \
    start_localization:="${START_LOCALIZATION:-false}" \
    lidar_x_m:="${LIDAR_X_M:-0.25}" \
    lidar_y_m:="${LIDAR_Y_M:-0.0}" \
    lidar_z_m:="${LIDAR_Z_M:-0.11}" \
    lidar_yaw_rad:="${LIDAR_YAW_RAD:-0.0}" \
    "$@"
