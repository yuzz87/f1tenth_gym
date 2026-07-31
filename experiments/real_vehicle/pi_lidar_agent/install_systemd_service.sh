#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="real-vehicle-lidar-agent.service"
PC_LIDAR_HOST="${PC_LIDAR_HOST:?Set PC_LIDAR_HOST to the current PC hostname or IP}"
TEMP_FILE="$(mktemp)"
trap 'rm -f "${TEMP_FILE}"' EXIT

sed \
    -e "s|@USER@|${USER}|g" \
    -e "s|@INSTALL_DIR@|${SCRIPT_DIR}|g" \
    -e "s|@PC_LIDAR_HOST@|${PC_LIDAR_HOST}|g" \
    "${SCRIPT_DIR}/systemd/${SERVICE_NAME}.in" > "${TEMP_FILE}"

sudo install -m 0644 "${TEMP_FILE}" "/etc/systemd/system/${SERVICE_NAME}"
sudo systemctl daemon-reload
echo "Installed ${SERVICE_NAME}."
echo "Manual start: sudo systemctl start ${SERVICE_NAME}"
echo "Enable at boot only after manual LiDAR validation."
