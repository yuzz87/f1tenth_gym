#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_PATH="${1:-${WORKSPACE_DIR}/log/step1_environment_${TIMESTAMP}.txt}"

mkdir -p "$(dirname "${OUTPUT_PATH}")"
exec > >(tee "${OUTPUT_PATH}") 2>&1

section() {
    printf '\n===== %s =====\n' "$1"
}

run_optional() {
    printf '$'
    printf ' %q' "$@"
    printf '\n'
    "$@" || printf '[not available or command failed: exit %s]\n' "$?"
}

run_shell_optional() {
    printf '$ %s\n' "$1"
    bash -c "$1" || printf '[not available or command failed: exit %s]\n' "$?"
}

printf '%s\n' "Real Vehicle Step 1 environment report"
printf '%s\n' "This script is read-only: it does not start pigpiod or access GPIO/PWM."
printf 'generated_at: %s\n' "$(date --iso-8601=seconds)"
printf 'output_path: %s\n' "${OUTPUT_PATH}"

section "Raspberry Pi and OS"
run_shell_optional "if [[ -r /proc/device-tree/model ]]; then tr -d '\\0' </proc/device-tree/model; echo; else echo 'model file not found'; fi"
run_optional uname -a
run_optional cat /etc/os-release
run_optional getconf LONG_BIT

section "User and permissions"
run_optional whoami
run_optional id
run_optional groups

section "Python"
run_optional command -v python3
run_optional python3 --version
run_shell_optional "python3 -c 'import sys; print(sys.executable); print(sys.version)'"

section "ROS 2"
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-<unset>}"
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-<unset>}"
printf 'ROS_LOCALHOST_ONLY=%s\n' "${ROS_LOCALHOST_ONLY:-<unset>}"
run_optional command -v ros2
run_shell_optional "find /opt/ros -mindepth 1 -maxdepth 1 -type d -printf '%f\\n' 2>/dev/null | sort"
run_shell_optional "dpkg-query -W -f='\${Package} \${Version}\\n' 'ros-*-ros-base' 'ros-*-desktop' 2>/dev/null | sort"

section "pigpio"
run_optional command -v pigpiod
run_optional pigpiod -v
run_optional pgrep -a pigpiod
run_shell_optional "dpkg-query -W -f='\${Package} \${Version}\\n' pigpio python3-pigpio 2>/dev/null"
run_shell_optional "python3 -c \"import pigpio; print('module:', pigpio.__file__); print('VERSION:', getattr(pigpio, 'VERSION', 'unknown'))\""

section "Wi-Fi and network"
run_optional hostname
run_optional hostname -I
run_optional ip -br address
run_optional ip route
if command -v iwgetid >/dev/null 2>&1; then
    run_optional iwgetid -r
fi
if command -v nmcli >/dev/null 2>&1; then
    run_optional nmcli -t -f GENERAL.DEVICE,GENERAL.TYPE,GENERAL.STATE,GENERAL.CONNECTION device show
fi

section "Time synchronization"
run_optional date --iso-8601=ns
run_optional timedatectl status
if command -v chronyc >/dev/null 2>&1; then
    run_optional chronyc tracking
fi

section "USB and serial devices"
run_optional lsusb
run_shell_optional "find /dev -maxdepth 1 -type c \( -name 'ttyUSB*' -o -name 'ttyACM*' \) -printf '%p\\n' 2>/dev/null | sort"
run_shell_optional "find -L /dev/serial/by-id -maxdepth 1 -type c -printf '%p -> %l\\n' 2>/dev/null | sort"

section "Potential legacy control processes"
run_shell_optional "pgrep -af 'run_ellipse|key_jog|udp_pwm|rccar|pigpiod' || true"

section "Result"
printf '%s\n' "Environment collection completed."
printf '%s\n' "No GPIO/PWM output was requested."
printf 'report: %s\n' "${OUTPUT_PATH}"

