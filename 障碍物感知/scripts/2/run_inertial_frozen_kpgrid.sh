#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
LAUNCH_FILE="${WORKSPACE}/src/coni_mpc/launch/num_sim_non_one_point.launch"

R="${R:-1.0}"
V="${V:-3.0}"
W="${W:-0.5}"
OBSTACLE_COUNT="${OBSTACLE_COUNT:-100}"
SEEDS="${SEEDS:-1-20}"
KPS="${KPS:-1.0,1.5,2.0,2.5,3.0}"
SAFETY_VARIANT="${SAFETY_VARIANT:-A2_soft_cbf}"
ACTIVE_UAVS="${ACTIVE_UAVS:-1,2,3}"
EXCLUDE_UAV0_FROM_SIMULATION="${EXCLUDE_UAV0_FROM_SIMULATION:-true}"
EXCLUDE_UAV0_FROM_ALL_METRICS="${EXCLUDE_UAV0_FROM_ALL_METRICS:-true}"
CAR_TRAJECTORY_MODE="${CAR_TRAJECTORY_MODE:-obstacle_aware}"
UGV_LONGITUDINAL_ACCEL="${UGV_LONGITUDINAL_ACCEL:-2.0}"
UGV_LONGITUDINAL_DECEL="${UGV_LONGITUDINAL_DECEL:-2.5}"
TIMEOUT_SEC="${TIMEOUT_SEC:-900}"
JOBS="${JOBS:-4}"

RUN_STAMP="$(date +%Y%m%d-%H%M%S)"
RUN_ROOT="${RUN_ROOT:-${WORKSPACE}/results/codex_true_inertial_frozen_with_cbf_kpgrid_20seed_${RUN_STAMP}}"
RUN_ROOT="$(realpath -m "${RUN_ROOT}")"
SUMMARY_CSV="${RUN_ROOT}/inertial_frozen_run_summary.csv"

declare -a SEED_LIST=()
declare -a KP_LIST=()

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "ERROR: missing file: ${path}" >&2
    exit 2
  fi
}

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "ERROR: missing command: ${cmd}" >&2
    exit 2
  fi
}

parse_seed_spec() {
  local raw="${1//,/ }"
  local token start end step value
  for token in ${raw}; do
    if [[ "${token}" =~ ^[0-9]+$ ]]; then
      SEED_LIST+=("${token}")
      continue
    fi
    if [[ "${token}" =~ ^([0-9]+)-([0-9]+)$ ]]; then
      start="${BASH_REMATCH[1]}"
      end="${BASH_REMATCH[2]}"
      if ((start <= end)); then
        step=1
      else
        step=-1
      fi
      value="${start}"
      while true; do
        SEED_LIST+=("${value}")
        if ((value == end)); then
          break
        fi
        value=$((value + step))
      done
      continue
    fi
    echo "ERROR: unsupported seed token: ${token}" >&2
    exit 2
  done
}

parse_kp_spec() {
  local raw="${1//,/ }"
  local token
  for token in ${raw}; do
    [[ -n "${token}" ]] || continue
    KP_LIST+=("${token}")
  done
}

kp_dir_name() {
  local kp="$1"
  echo "kp_${kp/./p}"
}

find_free_port() {
  python3 - <<'PY'
import socket
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind(("127.0.0.1", 0))
    print(s.getsockname()[1])
PY
}

metrics_has_all_row() {
  local metrics_csv="$1"
  local run_tag="$2"
  python3 - "${metrics_csv}" "${run_tag}" <<'PY'
import csv
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
run_tag = sys.argv[2]
if not path.exists():
    raise SystemExit(1)
with path.open("r", encoding="utf-8", newline="") as f:
    for row in csv.DictReader(f):
        if row.get("run_tag") == run_tag and row.get("scope") == "all":
            raise SystemExit(0)
raise SystemExit(1)
PY
}

append_summary_row() {
  local kp="$1"
  local seed="$2"
  local run_tag="$3"
  local status="$4"
  local metrics_csv="$5"
  local log_path="$6"
  python3 - "${SUMMARY_CSV}" "${kp}" "${seed}" "${run_tag}" "${status}" "${metrics_csv}" "${log_path}" <<'PY'
import csv
import pathlib
import sys

out = pathlib.Path(sys.argv[1])
kp, seed, run_tag, status, metrics_csv, log_path = sys.argv[2:]
header = [
    "kp", "seed", "run_tag", "status", "metrics_found", "frame_mode_effective",
    "ugv_rollout_mode", "tracking_rms", "collision", "fail_rate", "metrics_csv", "log_path",
]
row_out = {
    "kp": kp,
    "seed": seed,
    "run_tag": run_tag,
    "status": status,
    "metrics_found": "0",
    "frame_mode_effective": "",
    "ugv_rollout_mode": "",
    "tracking_rms": "nan",
    "collision": "nan",
    "fail_rate": "nan",
    "metrics_csv": str(pathlib.Path(metrics_csv).resolve()),
    "log_path": str(pathlib.Path(log_path).resolve()),
}
path = pathlib.Path(metrics_csv)
if path.exists():
    with path.open("r", encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            if row.get("run_tag") == run_tag and row.get("scope") == "all":
                row_out.update({
                    "metrics_found": "1",
                    "frame_mode_effective": row.get("frame_mode_effective", ""),
                    "ugv_rollout_mode": row.get("ugv_rollout_mode", ""),
                    "tracking_rms": row.get("tracking_rms", "nan"),
                    "collision": row.get("collision", "nan"),
                    "fail_rate": row.get("fail_rate", "nan"),
                })
                break

out.parent.mkdir(parents=True, exist_ok=True)
need_header = not out.exists()
with out.open("a", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=header)
    if need_header:
        writer.writeheader()
    writer.writerow(row_out)
PY
}

run_one_case() {
  local kp="$1"
  local seed="$2"
  local kp_dir seed_dir metrics_csv log_path roscore_log run_tag port ros_home ros_log_dir status
  kp_dir="$(kp_dir_name "${kp}")"
  seed_dir="${RUN_ROOT}/${kp_dir}/seed_${seed}"
  metrics_csv="${seed_dir}/inertial_metrics.csv"
  log_path="${seed_dir}/inertial.log"
  roscore_log="${seed_dir}/inertial_roscore.log"
  run_tag="${RUN_STAMP}_seed${seed}_inertial_frozen"
  port="$(find_free_port)"
  ros_home="${seed_dir}/inertial_ros_home"
  ros_log_dir="${seed_dir}/inertial_ros_logs"
  mkdir -p "${seed_dir}" "${ros_home}" "${ros_log_dir}"

  status="0"
  set +e
  timeout "${TIMEOUT_SEC}" bash -lc "
    set -euo pipefail
    cd '${WORKSPACE}'
    source '${WORKSPACE}/devel/setup.bash'
    export ROS_PACKAGE_PATH='${WORKSPACE}/src:'\"\${ROS_PACKAGE_PATH:-}\"
    export CMAKE_PREFIX_PATH='${WORKSPACE}/devel:'\"\${CMAKE_PREFIX_PATH:-}\"
    export ROS_MASTER_URI='http://127.0.0.1:${port}'
    export ROS_IP='127.0.0.1'
    export ROS_HOSTNAME='127.0.0.1'
    export ROS_HOME='${ros_home}'
    export ROS_LOG_DIR='${ros_log_dir}'
    roscore -p '${port}' >'${roscore_log}' 2>&1 &
    ROSCORE_PID=\$!
    cleanup() {
      rosnode kill -a >/dev/null 2>&1 || true
      kill \"\${ROSCORE_PID}\" >/dev/null 2>&1 || true
      wait \"\${ROSCORE_PID}\" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM
    until rosparam list >/dev/null 2>&1; do sleep 0.2; done
    echo '[INFO][惯性-frozen] r=${R} v=${V} w=${W} obs=${OBSTACLE_COUNT} seed=${seed} kappa=${kp} safety_variant=${SAFETY_VARIANT} ugv_rollout_mode=frozen active_uavs=${ACTIVE_UAVS} exclude_uav0_from_simulation=${EXCLUDE_UAV0_FROM_SIMULATION}'
    roslaunch '${LAUNCH_FILE}' \\
      obstacle_count:='${OBSTACLE_COUNT}' \\
      seed:='${seed}' \\
      r:='${R}' \\
      v:='${V}' \\
      w:='${W}' \\
      metrics_csv:='${metrics_csv}' \\
      run_tag:='${run_tag}' \\
      safety_variant:='${SAFETY_VARIANT}' \\
      kappa:='${kp}' \\
      ugv_rollout_mode:='frozen' \\
      car_trajectory_mode:='${CAR_TRAJECTORY_MODE}' \\
      ugv_longitudinal_accel:='${UGV_LONGITUDINAL_ACCEL}' \\
      ugv_longitudinal_decel:='${UGV_LONGITUDINAL_DECEL}' \\
      active_uavs_csv:='${ACTIVE_UAVS}' \\
      exclude_uav0_from_simulation:='${EXCLUDE_UAV0_FROM_SIMULATION}' \\
      exclude_uav0_from_all_metrics:='${EXCLUDE_UAV0_FROM_ALL_METRICS}' \\
      frame_mode:='inertial' \\
      use_inertial_frame:='true' \\
      use_noninertial_frame:='false' \\
      use_rviz:='false'
  " >"${log_path}" 2>&1
  status="$?"
  set -e

  if [[ "${status}" == "0" ]] && metrics_has_all_row "${metrics_csv}" "${run_tag}"; then
    append_summary_row "${kp}" "${seed}" "${run_tag}" "ok" "${metrics_csv}" "${log_path}"
    echo "[OK] kp=${kp} seed=${seed} -> ${metrics_csv}"
    return 0
  fi

  append_summary_row "${kp}" "${seed}" "${run_tag}" "failed_${status}" "${metrics_csv}" "${log_path}"
  echo "[FAIL] kp=${kp} seed=${seed} status=${status}; see ${log_path}" >&2
  return 1
}

require_file "${LAUNCH_FILE}"
require_cmd timeout
require_cmd python3
require_cmd bash

parse_seed_spec "${SEEDS}"
parse_kp_spec "${KPS}"
if ((${#SEED_LIST[@]} == 0 || ${#KP_LIST[@]} == 0)); then
  echo "ERROR: empty SEEDS or KPS" >&2
  exit 2
fi

mkdir -p "${RUN_ROOT}"
echo "[INFO] run root: ${RUN_ROOT}"
echo "[INFO] seeds: ${SEED_LIST[*]}"
echo "[INFO] kps: ${KP_LIST[*]}"
echo "[INFO] jobs: ${JOBS}"

export -f kp_dir_name find_free_port metrics_has_all_row append_summary_row run_one_case
export WORKSPACE LAUNCH_FILE RUN_ROOT RUN_STAMP SUMMARY_CSV
export R V W OBSTACLE_COUNT SAFETY_VARIANT ACTIVE_UAVS EXCLUDE_UAV0_FROM_SIMULATION
export EXCLUDE_UAV0_FROM_ALL_METRICS CAR_TRAJECTORY_MODE UGV_LONGITUDINAL_ACCEL UGV_LONGITUDINAL_DECEL TIMEOUT_SEC

printf '%s\n' "${KP_LIST[@]}" | while read -r kp; do
  for seed in "${SEED_LIST[@]}"; do
    printf '%s,%s\n' "${kp}" "${seed}"
  done
done | xargs -r -P "${JOBS}" -n 1 bash -lc '
  IFS=, read -r kp seed <<< "$1"
  run_one_case "$kp" "$seed"
' _

echo "[INFO] summary: ${SUMMARY_CSV}"
