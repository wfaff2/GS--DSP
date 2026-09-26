#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ROOT="${RUN_ROOT:-${ROOT_DIR}/results/reviewer6_comment3_3_kp6_stage_geometry_consistency_pilot20/map_soft2_r500}"
YAML_PATH="${ROOT_DIR}/src/coni_mpc/parameters/num_sim_non_one_point.yaml"

source /opt/ros/noetic/setup.bash
source "${ROOT_DIR}/devel/setup.bash"

python3 "${ROOT_DIR}/scripts/2/run_vw_three_controller_20seed.py" \
  --run-root "${RUN_ROOT}" \
  --yaml "${YAML_PATH}" \
  --jobs "${JOBS:-12}" \
  --car-trajectory-mode figure_eight \
  --obs 0 \
  --vary-noise-seed \
  --seeds 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20 \
  --v-values 3.0 \
  --w-values 3.0 \
  --scan-mode diagonal \
  --controllers noninertial_frozen,noninertial_stage \
  --active-uavs 1,2,3 \
  --safety-variant A2_soft_cbf \
  --obstacle-sensing-mode map \
  --cbf-slack-max 2.0 \
  --r-slack 500 \
  --use-predicted-stage-geometry \
  --kappa 1.0 \
  --no-aggregate

python3 "${ROOT_DIR}/scripts/2/aggregate_vw_three_controller_metrics.py" "${RUN_ROOT}"
