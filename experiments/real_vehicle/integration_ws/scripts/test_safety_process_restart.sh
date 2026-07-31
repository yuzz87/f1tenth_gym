#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/ros_env.sh"
LAUNCH_PID=""
RESTART_PID=""
SAFETY_PID=""

cleanup() {
    [[ -n "${SAFETY_PID}" ]] && kill -CONT "${SAFETY_PID}" 2>/dev/null || true
    [[ -n "${RESTART_PID}" ]] && kill -TERM "${RESTART_PID}" 2>/dev/null || true
    if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
    fi
    sleep 1
}
trap cleanup EXIT INT TERM

setsid ros2 launch real_vehicle_integration simulation.launch.py \
    auto_arm:=true auto_start:=true controller_type:=mppi \
    >"${WORKSPACE_DIR}/log/safety_restart_launch.log" 2>&1 &
LAUNCH_PID=$!
sleep 4
SAFETY_PID="$(pgrep -f '/real_vehicle_integration/safety_node.*__node:=safety' | head -1)"
if [[ -z "${SAFETY_PID}" ]]; then
    echo "Safety process was not found." >&2
    exit 1
fi
kill -STOP "${SAFETY_PID}"
ros2 run real_vehicle_integration diagnostic_check --ros-args \
    -p topic:=/vehicle/actuator_status \
    -p expected_message:=command_timeout \
    -p duration_s:=3.0

kill -KILL "${SAFETY_PID}"
SAFETY_PID=""
CONFIG="${WORKSPACE_DIR}/src/real_vehicle_integration/config/connection.yaml"
ros2 run real_vehicle_integration safety_node --ros-args \
    --params-file "${CONFIG}" -r __node:=safety_restarted \
    -p auto_arm:=false -p auto_start:=false &
RESTART_PID=$!
sleep 2
ros2 run real_vehicle_integration diagnostic_check --ros-args \
    -p topic:=/vehicle/safety_status \
    -p expected_message:=DISARMED \
    -p duration_s:=3.0

echo "Safety stop, Actuator watchdog, and disarmed restart were confirmed."
