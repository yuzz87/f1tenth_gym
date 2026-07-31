#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODE="${FAULT_MODE:-}"
ARGS=("$@")
for ((INDEX = 0; INDEX < ${#ARGS[@]}; INDEX += 1)); do
    case "${ARGS[INDEX]}" in
        --fault-mode)
            if ((INDEX + 1 < ${#ARGS[@]})); then
                MODE="${ARGS[INDEX + 1]}"
            fi
            ;;
        --fault-mode=*)
            MODE="${ARGS[INDEX]#--fault-mode=}"
            ;;
    esac
done

case "${MODE}" in
    scan_timeout|pose_timeout|control_timeout|emergency_stop)
        ;;
    *)
        echo "FAULT_MODEまたは--fault-modeに次のいずれかを指定してください。" >&2
        echo "scan_timeout, pose_timeout, control_timeout, emergency_stop" >&2
        exit 2
        ;;
esac

export DURATION_S="${DURATION_S:-12}"
exec "${SCRIPT_DIR}/run_controller_dry_run_trial.sh" \
    --fault-mode "${MODE}" \
    --fault-trigger-after-s "${FAULT_TRIGGER_AFTER_S:-5}" \
    "$@"
