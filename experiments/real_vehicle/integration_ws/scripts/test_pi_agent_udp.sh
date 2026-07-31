#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
PORT="${PI_AGENT_TEST_PORT:-15005}"
LOG_DIR="${SCRIPT_DIR}/../log"
AGENT_PID=""

cleanup() {
    if [[ -n "${AGENT_PID}" ]] && kill -0 "${AGENT_PID}" 2>/dev/null; then
        kill -TERM "${AGENT_PID}" 2>/dev/null || true
        wait "${AGENT_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

mkdir -p "${LOG_DIR}"
cd "${REPOSITORY_ROOT}"
PYTHONPATH=. python3 -m experiments.real_vehicle.pi_agent.agent \
    --host 127.0.0.1 --port "${PORT}" --arm-output --allow-motion \
    >"${LOG_DIR}/pi_agent_udp_test.log" 2>&1 &
AGENT_PID=$!
sleep 0.3

PYTHONPATH=. python3 -m experiments.real_vehicle.pi_agent.send_test_command \
    127.0.0.1 --port "${PORT}" --count 5
sleep 0.2
PYTHONPATH=. python3 -m experiments.real_vehicle.pi_agent.send_test_command \
    127.0.0.1 --port "${PORT}" --count 5 --motion-candidate
sleep 0.2

kill -TERM "${AGENT_PID}"
wait "${AGENT_PID}"
AGENT_PID=""

grep -q "watchdog: command_timeout -> neutral" "${LOG_DIR}/pi_agent_udp_test.log"
grep -q "mode=dry-run" "${LOG_DIR}/pi_agent_udp_test.log"
grep -q "shutdown: agent_shutdown -> neutral" "${LOG_DIR}/pi_agent_udp_test.log"
echo "UDP neutral, motion-candidate, status reply, and watchdog dry-run test passed."
