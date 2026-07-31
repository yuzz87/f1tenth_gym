#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Dry-run is intentional. Hardware output requires explicit command-line flags.
exec python3 "${SCRIPT_DIR}/agent.py" "$@"
