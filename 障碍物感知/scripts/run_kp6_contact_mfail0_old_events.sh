#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="${ROOT_DIR}/scripts/2/run_vw_three_controller_20seed.py"
AGGREGATOR="${ROOT_DIR}/scripts/2/aggregate_vw_three_controller_metrics.py"
ANALYZER="${ROOT_DIR}/scripts/analyze_kp6_contact_mfail0_rerun.py"
YAML_PATH="${ROOT_DIR}/src/coni_mpc/parameters/num_sim_non_one_point.yaml"
OLD_MAP_ROOT="${OLD_MAP_ROOT:-/home/jjm/桌面/论文修改/code/仿真/惯性 VS 非惯性/results/figure_eight_fixedobs_cbf_kp6_seed1_100_2ctrl}"
OLD_ONLINE_ROOT="${OLD_ONLINE_ROOT:-${ROOT_DIR}/results/reviewer6_comment3_3_kp1236_sensor_rplidar_a2m4_formal100_jobs12}"
EXP_ROOT="${EXP_ROOT:-${ROOT_DIR}/results/reviewer6_comment3_3_kp6_contact_mfail0_old_events_jobs12}"
JOBS="${JOBS:-12}"

source /opt/ros/noetic/setup.bash
source "${ROOT_DIR}/devel/setup.bash"

python3 - "${YAML_PATH}" <<'PY'
import sys
import yaml

path = sys.argv[1]
with open(path, encoding="utf-8") as handle:
    config = yaml.safe_load(handle)
value = float(config.get("failure_distance_margin", float("nan")))
if value != 0.0:
    raise SystemExit(
        f"Refusing to run: failure_distance_margin must be 0.0, got {value!r}"
    )
print(f"Verified failure_distance_margin={value:.1f}: {path}")
PY

# Create a source-auditable list of exactly the old m_fail=0.1 event-positive
# cases.  Old event-negative cases cannot become physical contacts after the
# threshold is reduced, so their reconstructed m_fail=0 result is zero.
python3 "${ANALYZER}" prepare \
  --old-map-root "${OLD_MAP_ROOT}" \
  --old-online-root "${OLD_ONLINE_ROOT}" \
  --rerun-root "${EXP_ROOT}"

run_subset() {
  local condition="$1"
  local sensing_mode="$2"
  local controller="$3"
  local seeds
  seeds="$(python3 "${ANALYZER}" seeds \
    --rerun-root "${EXP_ROOT}" \
    --condition "${condition}" \
    --controller "${controller}")"

  python3 "${RUNNER}" \
    --run-root "${EXP_ROOT}/${condition}" \
    --yaml "${YAML_PATH}" \
    --jobs "${JOBS}" \
    --car-trajectory-mode figure_eight \
    --obs 0 \
    --vary-noise-seed \
    --seeds "${seeds}" \
    --v-values 3.0 \
    --w-values 3.0 \
    --scan-mode diagonal \
    --controllers "${controller}" \
    --active-uavs 1,2,3 \
    --safety-variant A2_soft_cbf \
    --obstacle-sensing-mode "${sensing_mode}" \
    --kappa 1.0 \
    --no-aggregate
}

for condition_and_mode in \
  "map map" \
  "online_nominal online_nominal" \
  "online_degraded online_degraded"; do
  read -r condition sensing_mode <<<"${condition_and_mode}"
  run_subset "${condition}" "${sensing_mode}" noninertial_frozen
  run_subset "${condition}" "${sensing_mode}" noninertial_stage
  python3 "${AGGREGATOR}" "${EXP_ROOT}/${condition}"
done

python3 "${ANALYZER}" analyze \
  --old-map-root "${OLD_MAP_ROOT}" \
  --old-online-root "${OLD_ONLINE_ROOT}" \
  --rerun-root "${EXP_ROOT}"

echo "K_P=6 m_fail=0 old-event rerun complete: ${EXP_ROOT}"
