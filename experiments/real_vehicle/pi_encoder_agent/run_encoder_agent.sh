#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python3 "${SCRIPT_DIR}/agent.py" \
  --host "${PC_ENCODER_HOST:-127.0.0.1}" \
  --port "${PC_ENCODER_PORT:-5011}" \
  --gpio-a "${ENCODER_GPIO_A:-22}" \
  --gpio-b "${ENCODER_GPIO_B:-27}" \
  "$@"

