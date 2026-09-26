#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="${ROOT_DIR}/scripts/2/run_vw_three_controller_20seed.py"
AGGREGATOR="${ROOT_DIR}/scripts/2/aggregate_vw_three_controller_metrics.py"
YAML_PATH="${ROOT_DIR}/src/coni_mpc/parameters/num_sim_non_one_point.yaml"

# Targeted paired diagnostic set from the completed Nominal K_P=6 batch:
# seed 1: both; 2,3,4,9,10: Stage-only; 27: neither; 100: Frozen-only.
SEEDS="${SEEDS:-1,2,3,4,9,10,27,100}"
JOBS="${JOBS:-12}"
EXP_ROOT="${EXP_ROOT:-${ROOT_DIR}/results/reviewer6_comment3_3_kp6_paired_ablation_8seed}"

source /opt/ros/noetic/setup.bash
source "${ROOT_DIR}/devel/setup.bash"

run_condition() {
  local label="$1"
  local mode="$2"
  shift 2
  local run_root="${EXP_ROOT}/${label}"

  python3 "${RUNNER}" \
    --run-root "${run_root}" \
    --yaml "${YAML_PATH}" \
    --jobs "${JOBS}" \
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
    --obstacle-sensing-mode "${mode}" \
    --kappa 1.0 \
    --no-aggregate \
    "$@"

  python3 "${AGGREGATOR}" "${run_root}"
}

run_condition map_current map

run_condition online_100hz_zero online_nominal \
  --sensing-update-rate-hz 100 \
  --sensing-position-std-m 0 \
  --sensing-delay-sec 0 \
  --sensing-dropout-probability 0

run_condition online_10hz_zero online_nominal \
  --sensing-update-rate-hz 10 \
  --sensing-position-std-m 0 \
  --sensing-delay-sec 0 \
  --sensing-dropout-probability 0

run_condition online_10hz_noise001 online_nominal \
  --sensing-update-rate-hz 10 \
  --sensing-position-std-m 0.01 \
  --sensing-delay-sec 0 \
  --sensing-dropout-probability 0

run_condition online_10hz_delay010 online_nominal \
  --sensing-update-rate-hz 10 \
  --sensing-position-std-m 0 \
  --sensing-delay-sec 0.10 \
  --sensing-dropout-probability 0

echo "K_P=6 paired sensing ablation complete: ${EXP_ROOT}"
