#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ROOT="${RUN_ROOT:-${ROOT_DIR}/results/reviewer6_comment3_3_kp6_split_geometry_map_formal100_jobs12}"
YAML_PATH="${ROOT_DIR}/src/coni_mpc/parameters/num_sim_non_one_point.yaml"
SEEDS="$(seq -s, 1 100)"

source /opt/ros/noetic/setup.bash
source "${ROOT_DIR}/devel/setup.bash"

python3 - "${YAML_PATH}" <<'PY'
import sys
import yaml

path = sys.argv[1]
with open(path, encoding="utf-8") as handle:
    config = yaml.safe_load(handle)
checks = {
    "failure_distance_margin": float(config["failure_distance_margin"]),
    "cbf.slack_max": float(config["cbf"]["slack_max"]),
    "cbf.R_slack": float(config["cbf"]["R_slack"]),
}
expected = {
    "failure_distance_margin": 0.0,
    "cbf.slack_max": 2.0,
    "cbf.R_slack": 500.0,
}
if checks != expected:
    raise SystemExit(f"Configuration mismatch: expected {expected}, got {checks}")
print(f"Verified formal configuration: {checks}")
PY

mkdir -p "${RUN_ROOT}"
exec > >(tee -a "${RUN_ROOT}/formal100.log") 2>&1

echo "Formal paired experiment: K_P=6 (V=W=3), Map, seeds=1..100"
echo "Controllers: noninertial_frozen, noninertial_stage"
echo "Stage geometry mode: split_predicted; Frozen remains current-curvature"
echo "Safety: A2_soft_cbf, slack_max=2, R_slack=500, m_fail=0"
echo "Parallel jobs: ${JOBS:-12}"

python3 "${ROOT_DIR}/scripts/2/run_vw_three_controller_20seed.py" \
  --run-root "${RUN_ROOT}" \
  --yaml "${YAML_PATH}" \
  --jobs "${JOBS:-12}" \
  --car-trajectory-mode figure_eight \
  --obs 0 \
  --vary-noise-seed \
  --seeds "${SEEDS}" \
  --v-values 3.0 \
  --w-values 3.0 \
  --scan-mode diagonal \
  --controllers noninertial_frozen,noninertial_stage \
  --active-uavs 1,2,3 \
  --safety-variant A2_soft_cbf \
  --obstacle-sensing-mode map \
  --cbf-slack-max 2.0 \
  --r-slack 500 \
  --use-split-predicted-stage-geometry \
  --kappa 1.0 \
  --no-aggregate

python3 "${ROOT_DIR}/scripts/2/aggregate_vw_three_controller_metrics.py" "${RUN_ROOT}"
