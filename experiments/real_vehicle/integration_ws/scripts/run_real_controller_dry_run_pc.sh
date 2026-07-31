#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros_env.sh"

PI_SENSOR_HOST="${PI_SENSOR_HOST:-}"
if [[ -z "${PI_SENSOR_HOST}" ]]; then
    echo "PI_SENSOR_HOSTに現在のRaspberry Pi IPまたはraspberrypi.localを指定してください。" >&2
    exit 2
fi

MAP_YAML="${REAL_MAP_YAML:-${F1TENTH_GYM_ROOT}/experiments/real_vehicle/maps/small_test_area_03/small_test_area_03.yaml}"
if [[ ! -f "${MAP_YAML}" ]]; then
    echo "保存地図YAMLが見つかりません: ${MAP_YAML}" >&2
    exit 2
fi

FORBIDDEN_PROCESS_PATTERN='[u]dp_actuator_bridge|real_vehicle_integration/nodes/[a]ctuator_node|[m]ock_vehicle_node'
if pgrep -af "${FORBIDDEN_PROCESS_PATTERN}" >/tmp/real_vehicle_forbidden_processes.txt; then
    echo "ドライランを開始できません。ハードウェア経路またはmock車両プロセスが動作中です。" >&2
    cat /tmp/real_vehicle_forbidden_processes.txt >&2
    exit 3
fi

FORBIDDEN_ROS_NODES="$(
    ROS2CLI_NO_DAEMON=1 ros2 node list 2>/dev/null \
        | grep -E '^/(actuator|actuator_node|mock_vehicle|real_vehicle_actuator|udp_actuator_bridge)$' \
        || true
)"
if [[ -n "${FORBIDDEN_ROS_NODES}" ]]; then
    echo "ドライランを開始できません。禁止ROS nodeが動作中です。" >&2
    printf '%s\n' "${FORBIDDEN_ROS_NODES}" >&2
    exit 3
fi

INCOMPATIBLE_STACK_PATTERN='real_[l]ocalization.launch.py|real_[m]apping.launch.py|[u]dp_lidar_bridge|[e]ncoder_odom_node|[r]eal_map_localization_node'
if pgrep -af "${INCOMPATIBLE_STACK_PATTERN}" >/tmp/real_vehicle_existing_sensor_stack.txt; then
    echo "既存のmapping/localizationプロセスが動作中です。" >&2
    echo "UDPポートとtopicの二重起動を防ぐため、既存launchをCtrl+Cで終了してください。" >&2
    cat /tmp/real_vehicle_existing_sensor_stack.txt >&2
    exit 4
fi

echo "Phase 6 hardware isolation:"
echo "  actuator bridge: not launched"
echo "  actuator node:   not launched"
echo "  safe topic:      /dry_run/control_safe"
echo "  Pi PWM packets:  not sent"

exec ros2 launch real_vehicle_integration \
    real_controller_dry_run.launch.py \
    map_yaml:="${MAP_YAML}" \
    lidar_udp_allowed_host:="${PI_SENSOR_HOST}" \
    encoder_udp_allowed_host:="${PI_SENSOR_HOST}" \
    controller_type:="${CONTROLLER_TYPE:-mppi}" \
    reference_type:="${REFERENCE_TYPE:-straight}" \
    reference_speed_mps:="${REFERENCE_SPEED_MPS:-0.05}" \
    pose_timeout_s:="${POSE_TIMEOUT_S:-0.35}" \
    scan_timeout_s:="${SCAN_TIMEOUT_S:-0.50}" \
    fault_mode:="${FAULT_MODE:-none}" \
    start_rviz:="${START_RVIZ:-true}" \
    "$@"
