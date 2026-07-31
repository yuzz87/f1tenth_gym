#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="real-vehicle-actuator.service"
sudo systemctl start "${SERVICE_NAME}"
sleep 2
systemctl is-active --quiet "${SERVICE_NAME}"
sudo systemctl stop "${SERVICE_NAME}"
systemctl is-active --quiet "${SERVICE_NAME}" && exit 1 || true

if ! sudo journalctl -u "${SERVICE_NAME}" -n 100 --no-pager | \
    grep -Eq "node_shutdown|emergency_stop"; then
    echo "Service stopped, but neutral shutdown evidence was not found in journal." >&2
    exit 1
fi
echo "Actuator service startup and neutral shutdown were confirmed."
