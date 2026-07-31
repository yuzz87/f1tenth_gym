#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMPLATE="${WORKSPACE_DIR}/systemd/real-vehicle-actuator.service.in"
SERVICE_NAME="real-vehicle-actuator.service"
TEMP_FILE="$(mktemp --suffix=.service)"
trap 'rm -f "${TEMP_FILE}"' EXIT

sed \
    -e "s|@USER@|${SUDO_USER:-${USER}}|g" \
    -e "s|@WORKSPACE@|${WORKSPACE_DIR}|g" \
    -e "s|@ROS_DOMAIN_ID@|${ROS_DOMAIN_ID:-42}|g" \
    "${TEMPLATE}" >"${TEMP_FILE}"

systemd-analyze verify "${TEMP_FILE}"
sudo install -m 0644 "${TEMP_FILE}" "/etc/systemd/system/${SERVICE_NAME}"
sudo systemctl daemon-reload

if [[ "${1:-}" == "--enable" ]]; then
    sudo systemctl enable "${SERVICE_NAME}"
    echo "${SERVICE_NAME} enabled. Hardware output is still forced off."
else
    echo "Installed but not enabled. Run with --enable after reviewing the service."
fi
