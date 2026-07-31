#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python3 "${SCRIPT_DIR}/receiver.py" \
  --host "${ENCODER_BIND_HOST:-0.0.0.0}" \
  --port "${ENCODER_UDP_PORT:-5011}" \
  "$@"

