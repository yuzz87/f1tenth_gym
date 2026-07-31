#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-setup}"
DELAY_MS="${DELAY_MS:-25}"
LOSS_PERCENT="${LOSS_PERCENT:-1}"
PC_NS="rv_pc"
PI_NS="rv_pi"

cleanup() {
    sudo ip netns delete "${PC_NS}" 2>/dev/null || true
    sudo ip netns delete "${PI_NS}" 2>/dev/null || true
}

if [[ "${ACTION}" == "cleanup" ]]; then
    cleanup
    exit 0
fi
if [[ "${ACTION}" != "setup" ]]; then
    echo "usage: $0 [setup|cleanup]" >&2
    exit 2
fi

cleanup
sudo ip netns add "${PC_NS}"
sudo ip netns add "${PI_NS}"
sudo ip link add rv_pc_veth type veth peer name rv_pi_veth
sudo ip link set rv_pc_veth netns "${PC_NS}"
sudo ip link set rv_pi_veth netns "${PI_NS}"
sudo ip -n "${PC_NS}" addr add 10.201.0.1/30 dev rv_pc_veth
sudo ip -n "${PI_NS}" addr add 10.201.0.2/30 dev rv_pi_veth
sudo ip -n "${PC_NS}" link set lo up
sudo ip -n "${PI_NS}" link set lo up
sudo ip -n "${PC_NS}" link set rv_pc_veth up
sudo ip -n "${PI_NS}" link set rv_pi_veth up
sudo ip -n "${PC_NS}" route add 224.0.0.0/4 dev rv_pc_veth
sudo ip -n "${PI_NS}" route add 224.0.0.0/4 dev rv_pi_veth
sudo ip netns exec "${PC_NS}" tc qdisc add dev rv_pc_veth root netem \
    delay "${DELAY_MS}ms" loss "${LOSS_PERCENT}%"
sudo ip netns exec "${PI_NS}" tc qdisc add dev rv_pi_veth root netem \
    delay "${DELAY_MS}ms" loss "${LOSS_PERCENT}%"

echo "Created ${PC_NS} (10.201.0.1) and ${PI_NS} (10.201.0.2)."
echo "Each direction: delay=${DELAY_MS} ms, loss=${LOSS_PERCENT} %."
