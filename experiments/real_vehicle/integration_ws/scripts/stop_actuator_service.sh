#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros_env.sh"
MAIN_PID="${1:-}"

timeout 2 ros2 topic pub -1 /vehicle/emergency_stop std_msgs/msg/Bool \
    "{data: true}" >/dev/null 2>&1 || true
if [[ -n "${MAIN_PID}" ]] && kill -0 "${MAIN_PID}" 2>/dev/null; then
    kill -INT "${MAIN_PID}"
fi
