#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="$(cd "${SCRIPT_DIR}/../../pi_encoder_agent" && pwd)"
PI_TARGET="${PI_TARGET:-adrccar@raspberrypi.local}"
PI_INSTALL_DIR="${PI_ENCODER_INSTALL_DIR:-/home/adrccar/real_vehicle_pi_encoder_agent}"

printf 'Deploying encoder agent to %s:%s\n' "${PI_TARGET}" "${PI_INSTALL_DIR}"
tar \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    -C "${SOURCE_DIR}" -cf - . | \
    ssh "${PI_TARGET}" "mkdir -p '${PI_INSTALL_DIR}' && tar -C '${PI_INSTALL_DIR}' -xf -"

ssh "${PI_TARGET}" \
    "chmod +x '${PI_INSTALL_DIR}/agent.py' '${PI_INSTALL_DIR}/run_encoder_agent.sh' && \
     python3 '${PI_INSTALL_DIR}/agent.py' --help >/dev/null"

echo "Deployment and Python import check completed."
echo "Synthetic start:"
echo "  ssh ${PI_TARGET} PC_ENCODER_HOST=PC_IP ${PI_INSTALL_DIR}/run_encoder_agent.sh --synthetic --max-packets 20"

