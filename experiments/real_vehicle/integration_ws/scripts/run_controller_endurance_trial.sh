#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export DURATION_S="${DURATION_S:-300}"
export RECORD_ROSBAG="${RECORD_ROSBAG:-false}"
exec "${SCRIPT_DIR}/run_controller_dry_run_trial.sh" \
    --fault-mode none \
    "$@"
