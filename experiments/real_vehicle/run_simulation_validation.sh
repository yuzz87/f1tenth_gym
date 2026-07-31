#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RESULTS="${ROOT}/experiments/real_vehicle/results/simulation_validation"
mkdir -p "${RESULTS}"

cd "${ROOT}"
PYTHONPATH=. gym_env/bin/python -m unittest discover \
    -s experiments/real_vehicle/tests -p 'test_*.py'
PYTHONPATH=. gym_env/bin/python -m experiments.real_vehicle.evaluation.mpc_benchmark \
    --iterations 50 --output "${RESULTS}/mpc_benchmark.json"
PYTHONPATH=. gym_env/bin/python -m experiments.real_vehicle.evaluation.simulation_campaign \
    --smoke --output-dir "${RESULTS}/campaign_smoke"
PYTHONPATH=. gym_env/bin/python -m experiments.real_vehicle.evaluation.simulation_campaign \
    --seeds 123,456,789 --controllers mpc,mppi --scenarios straight \
    --pose-sources ground_truth --plant-profiles ideal,nominal,degraded \
    --lidar-profiles ideal --network-profiles normal --max-steps 250 \
    --output-dir "${RESULTS}/multiseed_plant_smoke"

echo "Simulation validation completed: ${RESULTS}"
