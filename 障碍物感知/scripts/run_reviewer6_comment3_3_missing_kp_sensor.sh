#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="${ROOT_DIR}/scripts/2/run_vw_three_controller_20seed.py"
AGGREGATOR="${ROOT_DIR}/scripts/2/aggregate_vw_three_controller_metrics.py"
YAML_PATH="${ROOT_DIR}/src/coni_mpc/parameters/num_sim_non_one_point.yaml"

# K_P maps to V=W as 1->0.5, 2->1.0, 3->1.5, 4->2.0, 6->3.0.
# K_P=4 already has a validated 100-seed online batch, so this script runs
# only the missing K_P={1,2,3,6} cases.
SEED_START="${SEED_START:-1}"
SEED_END="${SEED_END:-100}"
JOBS="${JOBS:-12}"
EXP_ROOT="${EXP_ROOT:-${ROOT_DIR}/results/reviewer6_comment3_3_kp1236_sensor_rplidar_a2m4_formal100_jobs12}"

source /opt/ros/noetic/setup.bash
source "${ROOT_DIR}/devel/setup.bash"

for sensing_mode in online_nominal online_degraded; do
  run_root="${EXP_ROOT}/${sensing_mode}"
  python3 "${RUNNER}" \
    --run-root "${run_root}" \
    --yaml "${YAML_PATH}" \
    --jobs "${JOBS}" \
    --car-trajectory-mode figure_eight \
    --obs 0 \
    --vary-noise-seed \
    --seed-start "${SEED_START}" \
    --seed-end "${SEED_END}" \
    --v-values 0.5,1.0,1.5,3.0 \
    --w-values 0.5,1.0,1.5,3.0 \
    --scan-mode diagonal \
    --controllers noninertial_frozen,noninertial_stage \
    --active-uavs 1,2,3 \
    --safety-variant A2_soft_cbf \
    --obstacle-sensing-mode "${sensing_mode}" \
    --kappa 1.0 \
    --no-aggregate

  python3 "${AGGREGATOR}" "${run_root}"
done

echo "Reviewer 6 Comment 3.3 missing-K_P experiment complete: ${EXP_ROOT}"
