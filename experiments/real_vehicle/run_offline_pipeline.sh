#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${repo_root}"

exec gym_env/bin/python -m \
  experiments.real_vehicle.evaluation.offline_pipeline "$@"
