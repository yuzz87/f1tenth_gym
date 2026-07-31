#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros_env.sh"

PI_SENSOR_HOST="${PI_SENSOR_HOST:-}"
if [[ -z "${PI_SENSOR_HOST}" ]]; then
    echo "PI_SENSOR_HOSTに現在のRaspberry Pi IPを指定してください。" >&2
    echo "例: PI_SENSOR_HOST=192.168.11.2 $0" >&2
    exit 2
fi

exec ros2 launch real_vehicle_integration real_mapping.launch.py \
    lidar_udp_allowed_host:="${PI_SENSOR_HOST}" \
    encoder_udp_allowed_host:="${PI_SENSOR_HOST}" \
    start_rviz:="${START_RVIZ:-true}" \
    start_direction_check:="${START_DIRECTION_CHECK:-false}" \
    "$@"
