#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="${ROOT_DIR}/scripts/2/run_vw_three_controller_20seed.py"
AGGREGATOR="${ROOT_DIR}/scripts/2/aggregate_vw_three_controller_metrics.py"
YAML_PATH="${ROOT_DIR}/src/coni_mpc/parameters/num_sim_non_one_point.yaml"
EXP_ROOT="${EXP_ROOT:-${ROOT_DIR}/results/reviewer6_comment3_3_allkp_split_geometry_sensor_formal100_jobs12}"
JOBS="${JOBS:-12}"

source /opt/ros/noetic/setup.bash
source "${ROOT_DIR}/devel/setup.bash"

python3 - "${YAML_PATH}" <<'PY'
import sys
import yaml

path = sys.argv[1]
with open(path, encoding="utf-8") as handle:
    cfg = yaml.safe_load(handle)
sensing = cfg["obstacle_sensing"]
actual = {
    "sensor_model": sensing["sensor_model"],
    "range_min_m": float(sensing["range_min_m"]),
    "range_max_m": float(sensing["range_max_m"]),
    "horizontal_fov_deg": float(sensing["horizontal_fov_deg"]),
    "angular_resolution_deg": float(sensing["angular_resolution_deg"]),
    "minimum_hit_rays": int(sensing["minimum_hit_rays"]),
    "update_rate_hz": float(sensing["update_rate_hz"]),
    "nominal": sensing["nominal"],
    "degraded": sensing["degraded"],
    "failure_distance_margin": float(cfg["failure_distance_margin"]),
    "slack_max": float(cfg["cbf"]["slack_max"]),
    "R_slack": float(cfg["cbf"]["R_slack"]),
}
expected = {
    "sensor_model": "rplidar_a2m4",
    "range_min_m": 0.15,
    "range_max_m": 6.0,
    "horizontal_fov_deg": 360.0,
    "angular_resolution_deg": 0.9,
    "minimum_hit_rays": 3,
    "update_rate_hz": 10.0,
    "nominal": {
        "position_std_m": 0.01,
        "delay_sec": 0.10,
        "dropout_probability": 0.0,
        "hold_sec": 0.30,
    },
    "degraded": {
        "position_std_m": 0.05,
        "delay_sec": 0.30,
        "dropout_probability": 0.10,
        "hold_sec": 0.50,
    },
    "failure_distance_margin": 0.0,
    "slack_max": 2.0,
    "R_slack": 500.0,
}
if actual != expected:
    raise SystemExit(f"Formal configuration mismatch:\nexpected={expected}\nactual={actual}")
print(f"Verified formal sensor/safety configuration: {actual}")
PY

mkdir -p "${EXP_ROOT}"

run_mode() {
  local sensing_mode="$1"
  local run_root="${EXP_ROOT}/${sensing_mode}"
  mkdir -p "${run_root}"
  echo "Starting ${sensing_mode}: all K_P, 100 paired seeds, jobs=${JOBS}" \
    | tee -a "${run_root}/formal100.log"

  python3 "${RUNNER}" \
    --run-root "${run_root}" \
    --yaml "${YAML_PATH}" \
    --jobs "${JOBS}" \
    --car-trajectory-mode figure_eight \
    --obs 0 \
    --vary-noise-seed \
    --seed-start 1 \
    --seed-end 100 \
    --v-values 0.5,1.0,1.5,2.0,3.0 \
    --w-values 0.5,1.0,1.5,2.0,3.0 \
    --scan-mode diagonal \
    --controllers noninertial_frozen,noninertial_stage \
    --active-uavs 1,2,3 \
    --safety-variant A2_soft_cbf \
    --obstacle-sensing-mode "${sensing_mode}" \
    --cbf-slack-max 2.0 \
    --r-slack 500 \
    --use-split-predicted-stage-geometry \
    --kappa 1.0 \
    --no-aggregate \
    2>&1 | tee -a "${run_root}/formal100.log"

  python3 "${AGGREGATOR}" "${run_root}" \
    2>&1 | tee -a "${run_root}/formal100.log"
}

run_mode online_nominal
run_mode online_degraded

echo "All-K_P corrected Nominal/Degraded formal100 complete: ${EXP_ROOT}"
