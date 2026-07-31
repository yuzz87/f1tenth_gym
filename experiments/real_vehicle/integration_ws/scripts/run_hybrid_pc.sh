#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros_env.sh"

PI_UDP_HOST="${PI_UDP_HOST:-raspberrypi.local}"

exec ros2 launch real_vehicle_integration hybrid_pc.launch.py \
    pi_udp_host:="${PI_UDP_HOST}" "$@"
