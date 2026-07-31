#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros_env.sh"

PI_SENSOR_HOST="${PI_SENSOR_HOST:-}"
if [[ -z "${PI_SENSOR_HOST}" ]]; then
    echo "PI_SENSOR_HOSTに現在のRaspberry Pi IPを指定してください。" >&2
    exit 2
fi

MAP_YAML="${REAL_MAP_YAML:-${F1TENTH_GYM_ROOT}/experiments/real_vehicle/maps/small_test_area_03/small_test_area_03.yaml}"
if [[ ! -f "${MAP_YAML}" ]]; then
    echo "保存地図YAMLが見つかりません: ${MAP_YAML}" >&2
    exit 2
fi

exec ros2 launch real_vehicle_integration real_localization.launch.py \
    map_yaml:="${MAP_YAML}" \
    lidar_udp_allowed_host:="${PI_SENSOR_HOST}" \
    encoder_udp_allowed_host:="${PI_SENSOR_HOST}" \
    start_rviz:="${START_RVIZ:-true}" \
    start_direction_check:="${START_DIRECTION_CHECK:-false}" \
    "$@"
