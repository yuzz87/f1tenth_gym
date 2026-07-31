#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE="${SCRIPT_DIR}/systemd/real-vehicle-pi-agent.service.in"
SERVICE_NAME="real-vehicle-pi-agent.service"
RUN_USER="${SUDO_USER:-${USER}}"
TEMP_FILE="$(mktemp --suffix=.service)"
trap 'rm -f "${TEMP_FILE}"' EXIT

sed \
    -e "s|@USER@|${RUN_USER}|g" \
    -e "s|@INSTALL_DIR@|${SCRIPT_DIR}|g" \
    "${TEMPLATE}" >"${TEMP_FILE}"

systemd-analyze verify "${TEMP_FILE}"
sudo install -m 0644 "${TEMP_FILE}" "/etc/systemd/system/${SERVICE_NAME}"
sudo systemctl daemon-reload

echo "Installed ${SERVICE_NAME} without enabling it."
echo "The service starts in dry-run mode and never requests GPIO/PWM output."
