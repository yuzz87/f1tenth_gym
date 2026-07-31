#!/usr/bin/env bash

# Source this file from every PC/Pi terminal before launching the integration nodes.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPOSITORY_ROOT="$(cd "${WORKSPACE_DIR}/../../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-foxy}"

RESTORE_NOUNSET=false
if [[ "$-" == *u* ]]; then
    RESTORE_NOUNSET=true
    set +u
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -n "${ACKERMANN_MSGS_PREFIX:-}" ]]; then
    export COLCON_CURRENT_PREFIX="${ACKERMANN_MSGS_PREFIX}"
    source "${ACKERMANN_MSGS_PREFIX}/share/ackermann_msgs/local_setup.bash"
    unset COLCON_CURRENT_PREFIX
fi

if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
fi

export F1TENTH_GYM_ROOT="${F1TENTH_GYM_ROOT:-${REPOSITORY_ROOT}}"
if [[ -d "${REPOSITORY_ROOT}/gym_env/lib/python3.8/site-packages" ]]; then
    export PYTHONPATH="${REPOSITORY_ROOT}/gym_env/lib/python3.8/site-packages:${REPOSITORY_ROOT}:${PYTHONPATH:-}"
else
    export PYTHONPATH="${REPOSITORY_ROOT}:${PYTHONPATH:-}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

if [[ "${RESTORE_NOUNSET}" == "true" ]]; then
    set -u
fi
