#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros_env.sh"

exec ros2 run real_vehicle_integration mapping_readiness_check \
    --ros-args -p timeout_s:="${TOPIC_TIMEOUT_S:-10.0}"
