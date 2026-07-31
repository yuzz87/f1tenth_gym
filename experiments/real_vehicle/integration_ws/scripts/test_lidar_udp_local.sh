#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros_env.sh"

LOG_FILE="$(mktemp)"
LAUNCH_PID=""
cleanup() {
    if [[ -n "${LAUNCH_PID}" ]]; then
        kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
        for _ in $(seq 1 20); do
            if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done
        if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
            kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
        fi
        wait "${LAUNCH_PID}" 2>/dev/null || true
    fi
    rm -f "${LOG_FILE}"
}
trap cleanup EXIT

setsid ros2 launch real_vehicle_integration real_lidar.launch.py \
    start_direction_check:=true start_rviz:=false >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!
sleep 2

ros2 run real_vehicle_integration lidar_udp_test_sender \
    127.0.0.1 --scans 8 --rate-hz 20 --sector front --reverse-chunks
sleep 1

if ! grep -q 'front=1.000 m' "${LOG_FILE}"; then
    echo "LiDAR direction output was not observed."
    cat "${LOG_FILE}"
    exit 1
fi

echo "Local LiDAR UDP -> LaserScan -> direction check: OK"
