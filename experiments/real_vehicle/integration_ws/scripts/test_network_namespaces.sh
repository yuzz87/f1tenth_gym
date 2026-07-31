#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
RUN_USER="${SUDO_USER:-${USER}}"
DOMAIN_ID="${ROS_DOMAIN_ID:-57}"
PC_PID=""
PI_PID=""

cleanup() {
    for namespace in rv_pc rv_pi; do
        sudo ip netns pids "${namespace}" 2>/dev/null | xargs -r sudo kill -TERM || true
    done
    wait "${PC_PID}" 2>/dev/null || true
    wait "${PI_PID}" 2>/dev/null || true
    "${SCRIPT_DIR}/setup_network_namespaces.sh" cleanup
}
trap cleanup EXIT INT TERM

"${SCRIPT_DIR}/setup_network_namespaces.sh" setup
COMMON_ENV="ROS_DOMAIN_ID=${DOMAIN_ID} ROS_LOCALHOST_ONLY=0 F1TENTH_GYM_ROOT=${WORKSPACE_DIR}/../../.."

sudo ip netns exec rv_pi sudo -u "${RUN_USER}" env ${COMMON_ENV} \
    bash -lc "cd '${WORKSPACE_DIR}' && scripts/run_raspberry_pi.sh auto_arm:=true auto_start:=true" &
PI_PID=$!
sudo ip netns exec rv_pc sudo -u "${RUN_USER}" env ${COMMON_ENV} \
    bash -lc "cd '${WORKSPACE_DIR}' && scripts/run_pc.sh controller_type:=mppi" &
PC_PID=$!
sleep 5
sudo ip netns exec rv_pc sudo -u "${RUN_USER}" env ${COMMON_ENV} \
    bash -lc "cd '${WORKSPACE_DIR}' && source scripts/ros_env.sh && \
    ros2 run real_vehicle_integration smoke_check --ros-args -p duration_s:=5.0"

echo "Network namespace DDS test passed."
