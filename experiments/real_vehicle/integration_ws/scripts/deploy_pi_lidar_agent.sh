#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="$(cd "${SCRIPT_DIR}/../../pi_lidar_agent" && pwd)"
PI_TARGET="${PI_TARGET:-adrccar@raspberrypi.local}"
PI_INSTALL_DIR="${PI_LIDAR_INSTALL_DIR:-/home/adrccar/real_vehicle_pi_lidar_agent}"
RPLIDAR_SDK_ROOT="${RPLIDAR_SDK_ROOT:-/home/adrccar/rplidar_sdk}"

printf 'Deploying LiDAR agent to %s:%s\n' "${PI_TARGET}" "${PI_INSTALL_DIR}"
tar \
    --exclude='build' \
    --exclude='*.csv' \
    -C "${SOURCE_DIR}" -cf - . | \
    ssh "${PI_TARGET}" "mkdir -p '${PI_INSTALL_DIR}' && tar -C '${PI_INSTALL_DIR}' -xf -"

ssh "${PI_TARGET}" \
    "chmod +x '${PI_INSTALL_DIR}/run_lidar_agent.sh' \
      '${PI_INSTALL_DIR}/install_systemd_service.sh' && \
     make -C '${PI_INSTALL_DIR}' -j2 RPLIDAR_SDK_ROOT='${RPLIDAR_SDK_ROOT}'"

echo "Deployment and Pi build completed."
echo "Synthetic start:"
echo "  ssh ${PI_TARGET} ${PI_INSTALL_DIR}/build/real_vehicle_lidar_agent --synthetic --host PC_IP --max-scans 20"
