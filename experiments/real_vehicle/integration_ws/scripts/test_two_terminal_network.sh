#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/ros_env.sh"

MODE="${1:-all}"
CONTROLLER_TYPE="${CONTROLLER_TYPE:-mppi}"
LOG_DIR="${WORKSPACE_DIR}/log/two_terminal_test"
mkdir -p "${LOG_DIR}"

PI_PID=""
PC_PID=""

stop_graph() {
    for pid in "${PC_PID}" "${PI_PID}"; do
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            kill -TERM -- "-${pid}" 2>/dev/null || true
        fi
    done
    sleep 2
    for pid in "${PC_PID}" "${PI_PID}"; do
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            kill -KILL -- "-${pid}" 2>/dev/null || true
        fi
    done
    wait "${PC_PID}" 2>/dev/null || true
    wait "${PI_PID}" 2>/dev/null || true
    PC_PID=""
    PI_PID=""
    sleep 1
}
trap stop_graph EXIT INT TERM

run_profile() {
    local profile="$1"
    local pc_faults=()
    local pi_faults=()
    local expect_dropout="false"
    local expect_replay="false"
    if [[ "${profile}" == "fault" ]]; then
        pc_faults=(scan_delay_s:=0.35 scan_dropout_probability:=0.25)
        pi_faults=(control_delay_s:=0.04 control_dropout_probability:=0.35)
        expect_dropout="true"
    elif [[ "${profile}" == "burst" ]]; then
        pc_faults=(scan_burst_start_probability:=0.20 scan_burst_length_messages:=3)
        pi_faults=(control_burst_start_probability:=0.10 control_burst_length_messages:=5)
        expect_dropout="true"
    elif [[ "${profile}" == "outage" ]]; then
        pi_faults=(control_outage_after_s:=2.0 control_outage_duration_s:=1.0)
        expect_dropout="true"
    elif [[ "${profile}" == "jitter" ]]; then
        pc_faults=(scan_delay_s:=0.02 scan_delay_jitter_s:=0.02)
        pi_faults=(control_delay_s:=0.01 control_delay_jitter_s:=0.01)
    elif [[ "${profile}" == "replay" ]]; then
        pi_faults=(control_duplicate_probability:=0.20 control_stale_replay_probability:=0.10)
        expect_replay="true"
    fi

    setsid "${SCRIPT_DIR}/run_raspberry_pi.sh" \
        auto_arm:=true auto_start:=true "${pi_faults[@]}" \
        >"${LOG_DIR}/${profile}_pi.log" 2>&1 &
    PI_PID=$!
    setsid "${SCRIPT_DIR}/run_pc.sh" \
        controller_type:="${CONTROLLER_TYPE}" "${pc_faults[@]}" \
        >"${LOG_DIR}/${profile}_pc.log" 2>&1 &
    PC_PID=$!
    sleep 3

    ros2 run real_vehicle_integration smoke_check --ros-args \
        -p duration_s:=4.0
    ros2 run real_vehicle_integration network_fault_check --ros-args \
        -p duration_s:=3.0 -p expect_dropout:="${expect_dropout}" \
        -p expect_replay:="${expect_replay}"
    stop_graph
}

case "${MODE}" in
    nominal|fault|burst|outage|jitter|replay) run_profile "${MODE}" ;;
    all)
        run_profile nominal
        run_profile fault
        ;;
    extended)
        run_profile nominal
        run_profile jitter
        run_profile burst
        run_profile outage
        run_profile replay
        ;;
    *)
        echo "usage: $0 [nominal|fault|burst|outage|jitter|replay|all|extended]" >&2
        exit 2
        ;;
esac
