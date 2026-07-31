#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PC_LIDAR_HOST="${PC_LIDAR_HOST:-127.0.0.1}"
LIDAR_DEVICE="${LIDAR_DEVICE:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
LIDAR_LOG="${LIDAR_LOG:-${SCRIPT_DIR}/lidar_scans.csv}"
LIDAR_ANGLE_INVERTED="${LIDAR_ANGLE_INVERTED:-true}"

ANGLE_ARGS=()
if [[ "${LIDAR_ANGLE_INVERTED}" == "true" ]]; then
    ANGLE_ARGS+=(--angle-inverted)
fi

exec "${SCRIPT_DIR}/build/real_vehicle_lidar_agent" \
    --device "${LIDAR_DEVICE}" \
    --host "${PC_LIDAR_HOST}" \
    --port "${PC_LIDAR_PORT:-5010}" \
    --beams 360 \
    --beams-per-packet 120 \
    --range-min 0.15 \
    --range-max 12.0 \
    --angle-offset-rad "${LIDAR_ANGLE_OFFSET_RAD:-0.0}" \
    "${ANGLE_ARGS[@]}" \
    --csv "${LIDAR_LOG}" \
    "$@"
