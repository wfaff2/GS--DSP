#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="${ROOT_DIR}/scripts/2/run_vw_three_controller_20seed.py"
AGGREGATOR="${ROOT_DIR}/scripts/2/aggregate_vw_three_controller_metrics.py"
YAML_PATH="${ROOT_DIR}/src/coni_mpc/parameters/num_sim_non_one_point.yaml"

# Same mechanism-localization seeds as the safety-layer ablation.
SEEDS="${SEEDS:-2,3,4,5,8,12,14,15,9,51,1,16}"
JOBS="${JOBS:-12}"
EXP_ROOT="${EXP_ROOT:-${ROOT_DIR}/results/reviewer6_comment3_3_kp6_stage_geometry_consistency_12seed}"

source /opt/ros/noetic/setup.bash
source "${ROOT_DIR}/devel/setup.bash"

run_variant() {
  local name="$1"
  local safety_variant="$2"
  local slack_max="$3"
  local r_slack="$4"
  local run_root="${EXP_ROOT}/${name}"

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
    --controllers noninertial_stage \
    --active-uavs 1,2,3 \
    --safety-variant "${safety_variant}" \
    --obstacle-sensing-mode map \
    --cbf-slack-max "${slack_max}" \
    --r-slack "${r_slack}" \
    --use-predicted-stage-geometry \
    --kappa 1.0 \
    --no-aggregate

  python3 "${AGGREGATOR}" "${run_root}"
}

mkdir -p "${EXP_ROOT}"
exec > >(tee -a "${EXP_ROOT}/stage_geometry_consistency_ablation.log") 2>&1

run_variant predicted_soft2_r500 A2_soft_cbf 2.0 500
run_variant predicted_hard0 A1_hard_cbf 0.0 500

echo "K_P=6 Stage geometry-consistency ablation complete: ${EXP_ROOT}"
