#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="${ROOT_DIR}/scripts/2/run_vw_three_controller_20seed.py"
AGGREGATOR="${ROOT_DIR}/scripts/2/aggregate_vw_three_controller_metrics.py"
YAML_PATH="${ROOT_DIR}/src/coni_mpc/parameters/num_sim_non_one_point.yaml"

# Formal Reviewer 6 Comment 3.3 experiment: match the historical Map baseline
# with the same 100 case seeds.  The three-seed pilot has already passed.
SEED_START="${SEED_START:-1}"
SEED_END="${SEED_END:-100}"
JOBS="${JOBS:-12}"
# Keep the formal results separate from all pilot and diagnostic runs.
EXP_ROOT="${EXP_ROOT:-${ROOT_DIR}/results/reviewer6_comment3_3_kp4_sensor_rplidar_a2m4_formal100_jobs12}"

source /opt/ros/noetic/setup.bash
source "${ROOT_DIR}/devel/setup.bash"

# In this project K_P=4 is the operating point V=W=2.0.  The two controllers
# use the same obstacle layout, case seed, sensor-noise stream, and dropout
# stream within every sensing condition.  The Map baseline reuses seeds 1--100
# from the historical formal batch, so only the two online modes are run here.
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
    --v-values 2.0 \
    --w-values 2.0 \
    --scan-mode diagonal \
    --controllers noninertial_frozen,noninertial_stage \
    --active-uavs 1,2,3 \
    --safety-variant A2_soft_cbf \
    --obstacle-sensing-mode "${sensing_mode}" \
    --kappa 1.0 \
    --no-aggregate

  python3 "${AGGREGATOR}" "${run_root}"
done

echo "Reviewer 6 Comment 3.3 experiment complete: ${EXP_ROOT}"
