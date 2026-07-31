#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros_env.sh"

ARGS=("$@")
TRIAL_NAME=""
CONTROLLER="${CONTROLLER_TYPE:-mppi}"
FAULT="${FAULT_MODE:-none}"
OUTPUT_BASE="${F1TENTH_GYM_ROOT}/experiments/real_vehicle/results/"
OUTPUT_BASE+="phase6_controller_dry_run/trials"

for ((INDEX = 0; INDEX < ${#ARGS[@]}; INDEX += 1)); do
    case "${ARGS[INDEX]}" in
        --trial-name)
            if ((INDEX + 1 < ${#ARGS[@]})); then
                TRIAL_NAME="${ARGS[INDEX + 1]}"
            fi
            ;;
        --trial-name=*)
            TRIAL_NAME="${ARGS[INDEX]#--trial-name=}"
            ;;
        --controller-type)
            if ((INDEX + 1 < ${#ARGS[@]})); then
                CONTROLLER="${ARGS[INDEX + 1]}"
            fi
            ;;
        --controller-type=*)
            CONTROLLER="${ARGS[INDEX]#--controller-type=}"
            ;;
        --fault-mode)
            if ((INDEX + 1 < ${#ARGS[@]})); then
                FAULT="${ARGS[INDEX + 1]}"
            fi
            ;;
        --fault-mode=*)
            FAULT="${ARGS[INDEX]#--fault-mode=}"
            ;;
        --output-dir)
            if ((INDEX + 1 < ${#ARGS[@]})); then
                OUTPUT_BASE="${ARGS[INDEX + 1]}"
            fi
            ;;
        --output-dir=*)
            OUTPUT_BASE="${ARGS[INDEX]#--output-dir=}"
            ;;
        -h|--help)
            exec ros2 run real_vehicle_integration \
                controller_dry_run_trial --help
            ;;
    esac
done

if [[ -z "${TRIAL_NAME}" ]]; then
    TRIAL_NAME="${CONTROLLER}_${FAULT}_$(date +%Y%m%d_%H%M%S)"
    ARGS+=(--trial-name "${TRIAL_NAME}")
fi
if [[ " ${ARGS[*]} " != *" --controller-type "* ]] \
    && [[ " ${ARGS[*]} " != *" --controller-type="* ]]; then
    ARGS+=(--controller-type "${CONTROLLER}")
fi
if [[ " ${ARGS[*]} " != *" --fault-mode "* ]] \
    && [[ " ${ARGS[*]} " != *" --fault-mode="* ]]; then
    ARGS+=(--fault-mode "${FAULT}")
fi

RESULT_DIR="${OUTPUT_BASE}/${TRIAL_NAME}"
if [[ -d "${RESULT_DIR}" ]] \
    && [[ -n "$(find "${RESULT_DIR}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    echo "試験を開始できません: 結果保存先が空ではありません。" >&2
    echo "  ${RESULT_DIR}" >&2
    echo "--trial-nameに未使用の名前を指定してください。" >&2
    exit 2
fi

RECORD_ROSBAG="${RECORD_ROSBAG:-true}"
BAG_PID=""
BAG_DIR=""
BAG_LOG=""

stop_bag() {
    if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
        kill -INT "${BAG_PID}" 2>/dev/null || true
        wait "${BAG_PID}" 2>/dev/null || true
    fi
    BAG_PID=""
}
trap stop_bag EXIT INT TERM

case "${RECORD_ROSBAG,,}" in
    true|1|yes|on)
        BAG_ROOT="${F1TENTH_GYM_ROOT}/experiments/real_vehicle/results/"
        BAG_ROOT+="phase6_controller_dry_run/rosbags"
        BAG_DIR="${BAG_ROOT}/${TRIAL_NAME}"
        BAG_LOG="${BAG_ROOT}/${TRIAL_NAME}.log"
        if [[ -e "${BAG_DIR}" ]]; then
            echo "試験を開始できません: rosbag保存先が既に存在します。" >&2
            echo "  ${BAG_DIR}" >&2
            exit 2
        fi
        mkdir -p "${BAG_ROOT}"
        ros2 bag record \
            -o "${BAG_DIR}" \
            /scan \
            /odom \
            /localization/pose \
            /localization/status \
            /lidar/transport_status \
            /encoder/transport_status \
            /dry_run/scan \
            /dry_run/pose \
            /dry_run/control_raw \
            /dry_run/control_request \
            /dry_run/control_safe \
            /dry_run/controller_status \
            /dry_run/safety_status \
            /dry_run/hardware_guard_status \
            /dry_run/fault_injector_status \
            /dry_run/emergency_stop \
            >"${BAG_LOG}" 2>&1 &
        BAG_PID=$!
        sleep 1
        if ! kill -0 "${BAG_PID}" 2>/dev/null; then
            echo "rosbag記録を開始できませんでした: ${BAG_LOG}" >&2
            wait "${BAG_PID}" || true
            exit 2
        fi
        echo "ROS bag recording: ${BAG_DIR}"
        ;;
    false|0|no|off)
        echo "ROS bag recording: disabled"
        ;;
    *)
        echo "RECORD_ROSBAG must be true or false: ${RECORD_ROSBAG}" >&2
        exit 2
        ;;
esac

set +e
ros2 run real_vehicle_integration controller_dry_run_trial \
    --duration-s "${DURATION_S:-60}" \
    "${ARGS[@]}"
TRIAL_STATUS=$?
set -e

stop_bag
trap - EXIT INT TERM
if [[ -n "${BAG_DIR}" ]]; then
    if [[ -d "${RESULT_DIR}" ]]; then
        printf '%s\n' "${BAG_DIR}" >"${RESULT_DIR}/rosbag_path.txt"
    fi
    echo "ROS bag saved: ${BAG_DIR}"
fi
exit "${TRIAL_STATUS}"
