#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="$(cd "${SCRIPT_DIR}/../../pi_agent" && pwd)"
PI_TARGET="${PI_TARGET:-adrccar@raspberrypi.local}"
PI_INSTALL_DIR="${PI_INSTALL_DIR:-/home/adrccar/real_vehicle_pi_agent}"

printf 'Deploying standalone Pi agent to %s:%s\n' "${PI_TARGET}" "${PI_INSTALL_DIR}"
tar \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    -C "${SOURCE_DIR}" -cf - . | \
    ssh "${PI_TARGET}" "mkdir -p '${PI_INSTALL_DIR}' && tar -C '${PI_INSTALL_DIR}' -xf -"

ssh "${PI_TARGET}" \
    "chmod +x '${PI_INSTALL_DIR}/agent.py' '${PI_INSTALL_DIR}/run_agent.sh' && \
     python3 '${PI_INSTALL_DIR}/agent.py' --help >/dev/null"

echo "Deployment and Python import check completed."
echo "Dry-run start: ssh ${PI_TARGET} ${PI_INSTALL_DIR}/run_agent.sh"
