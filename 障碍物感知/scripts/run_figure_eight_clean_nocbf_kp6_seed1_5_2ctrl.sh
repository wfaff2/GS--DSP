#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER_DIR="${ROOT_DIR}/scripts/2"
EXP_NAME="figure_eight_clean_nocbf_kp6_seed1_5_2ctrl"
RUN_ROOT="${ROOT_DIR}/results/${EXP_NAME}"
LOG_FILE="${RUN_ROOT}/${EXP_NAME}.log"

mkdir -p "${RUN_ROOT}"
exec > >(tee -a "${LOG_FILE}") 2>&1

echo "[${EXP_NAME}] start: $(date '+%F %T')"
source /opt/ros/noetic/setup.bash
source "${ROOT_DIR}/devel/setup.bash"

python3 "${RUNNER_DIR}/run_vw_three_controller_20seed.py" \
  --run-root "${RUN_ROOT}" \
  --yaml "${ROOT_DIR}/src/coni_mpc/parameters/num_sim_non_one_point.yaml" \
  --jobs 12 \
  --car-trajectory-mode figure_eight \
  --disable-figure-eight-obstacles \
  --obs 0 \
  --vary-noise-seed \
  --seed-start 1 \
  --seed-end 5 \
  --v-values 3.0 \
  --w-values 3.0 \
  --scan-mode diagonal \
  --controllers noninertial_frozen,noninertial_stage \
  --active-uavs 1,2,3 \
  --safety-variant A0_no_cbf \
  --kappa 1.0 \
  --no-aggregate

python3 "${RUNNER_DIR}/aggregate_vw_three_controller_metrics.py" "${RUN_ROOT}"

echo "[${EXP_NAME}] done: $(date '+%F %T')"
