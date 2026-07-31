#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros_env.sh"

CONFIG_FILE="${ACTUATOR_CONFIG_FILE:-${SCRIPT_DIR}/../src/real_vehicle_integration/config/connection.yaml}"
exec ros2 run real_vehicle_integration actuator_node --ros-args \
    --params-file "${CONFIG_FILE}" \
    -p hardware_output_enabled:=false \
    -p pwm_backend:=mock
