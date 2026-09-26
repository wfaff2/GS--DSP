#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"

TRACK_OFFSET="${TRACK_OFFSET:-1.0}"
V="${V:-2.0}"
W="${W:-0.5}"
OBSTACLE_COUNT="${OBSTACLE_COUNT:-100}"
CAR_TRAJECTORY_MODE="${CAR_TRAJECTORY_MODE:-circle}"
KP_LIST="${KP_LIST:-${KAPPAS:-1.0,1.3,1.5}}"
USE_RVIZ="${USE_RVIZ:-true}"
RVIZ_VIEW="${RVIZ_VIEW:-world}"
NO_OBSTACLE_AVOIDANCE="${NO_OBSTACLE_AVOIDANCE:-true}"
HIDE_OBSTACLES_IN_RVIZ="${HIDE_OBSTACLES_IN_RVIZ:-true}"
SEED="${SEED:-3}"
START_Z="${START_Z:-2.0}"
ACTIVE_UAVS="${ACTIVE_UAVS:-1,2,3}"
TIMEOUT_SEC="${TIMEOUT_SEC:-900}"
YAML_REL="${YAML_REL:-src/coni_mpc/parameters/num_sim_non_one_point.yaml}"
RVIZ_DIR="${WORKSPACE}/src/coni_mpc/rviz"

RUN_STAMP="$(date +%Y%m%d-%H%M%S)"
RUN_ROOT="${RUN_ROOT:-${WORKSPACE}/results/circle_kappa_tracking_rms_${RUN_STAMP}}"
LOGS_ROOT="${RUN_ROOT}/logs"
PER_RUN_CSV="${RUN_ROOT}/per_run.csv"
SUMMARY_CSV="${RUN_ROOT}/tracking_rms_summary.csv"

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "ERROR: missing command: ${cmd}" >&2
    exit 2
  fi
}

trim() {
  local text="$1"
  text="${text#"${text%%[![:space:]]*}"}"
  text="${text%"${text##*[![:space:]]}"}"
  printf '%s' "${text}"
}

find_free_port() {
  python3 - <<'PY'
import socket
s = socket.socket()
s.bind(("127.0.0.1", 0))
print(s.getsockname()[1])
s.close()
PY
}

window_ids() {
  xprop -root _NET_CLIENT_LIST_STACKING 2>/dev/null \
    | sed 's/^.*# //' \
    | tr ',' ' '
}

window_id_by_pid() {
  local target_pid="$1"
  local wid win_pid
  for wid in $(window_ids); do
    win_pid="$(xprop -id "${wid}" _NET_WM_PID 2>/dev/null | awk '{print $3}')"
    if [[ "${win_pid}" == "${target_pid}" ]]; then
      echo "${wid}"
      return 0
    fi
  done
  return 1
}

rename_window() {
  local wid="$1"
  local ascii_title="$2"
  local utf8_title="$3"
  xprop -id "${wid}" -f WM_NAME 8s -set WM_NAME "${ascii_title}" >/dev/null 2>&1 || true
  xprop -id "${wid}" -f _NET_WM_NAME 8u -set _NET_WM_NAME "${utf8_title}" >/dev/null 2>&1 || true
  xprop -id "${wid}" -f _NET_WM_VISIBLE_NAME 8u -set _NET_WM_VISIBLE_NAME "${utf8_title}" >/dev/null 2>&1 || true
}

tag_window_by_pid() {
  local pid="$1"
  local ascii_title="$2"
  local utf8_title="$3"
  command -v xprop >/dev/null 2>&1 || return 0
  local deadline=$((SECONDS + 12))
  while ((SECONDS < deadline)); do
    local wid
    wid="$(window_id_by_pid "${pid}" || true)"
    if [[ -n "${wid}" ]]; then
      rename_window "${wid}" "${ascii_title}" "${utf8_title}"
      return 0
    fi
    sleep 0.5
  done
  return 0
}

prepare_rviz_config() {
  local src="$1"
  local dst="$2"
  python3 - "${src}" "${dst}" <<'PY'
import pathlib
import sys

src = pathlib.Path(sys.argv[1])
dst = pathlib.Path(sys.argv[2])
lines = src.read_text(encoding="utf-8").splitlines(True)
out = []
block = []

def flush():
    global block
    if not block:
        return
    text = "".join(block)
    if "Marker Topic: /coni_mpc/static_obstacles" in text or "Marker Topic: /coni_mpc/dynamic_obstacles" in text:
        text = text.replace("      Enabled: true\n", "      Enabled: false\n")
        text = text.replace("      Value: true\n", "      Value: false\n")
    out.append(text)
    block = []

for line in lines:
    if line.startswith("    - ") or line.startswith("Visualization Manager:"):
        flush()
    block.append(line)

flush()
dst.write_text("".join(out), encoding="utf-8")
PY
}

wait_for_rosparam() {
  local timeout_sec="$1"
  shift
  local deadline=$((SECONDS + timeout_sec))
  while ((SECONDS < deadline)); do
    if env "$@" rosparam list >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

append_per_run_row() {
  local frame_mode="$1"
  local kp="$2"
  local tracking_rms="$3"
  local fail_rate="$4"
  local effective_mode="$5"
  local run_tag="$6"
  local metrics_csv="$7"
  local return_code="$8"
  printf '%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "${frame_mode}" "${kp}" "${tracking_rms}" "${fail_rate}" \
    "${effective_mode}" "${run_tag}" "${metrics_csv}" "${return_code}" >> "${PER_RUN_CSV}"
}

run_case() {
  local mode="$1"
  local kp="$2"
  local run_index="$3"
  local run_tag="${RUN_STAMP}_$(printf '%06d' "${run_index}")_${mode}_astar_kp${kp}_s${SEED}"
  local run_dir="${LOGS_ROOT}/${run_tag}"
  local metrics_csv="${run_dir}/metrics.csv"
  local setup_log="${run_dir}/setup.log"
  local roscore_log="${run_dir}/roscore.log"
  local rosrun_log="${run_dir}/rosrun.log"
  local ros_home="${run_dir}/ros_home"
  local ros_log_dir="${run_dir}/ros_logs"
  local rviz_world_log="${run_dir}/rviz_world.log"
  local rviz_non_log="${run_dir}/rviz_non_inertial.log"
  local rviz_world_cfg="${run_dir}/world.hidden_obstacles.rviz"
  local rviz_non_cfg="${run_dir}/non_inertial.hidden_obstacles.rviz"
  local rviz_world_pid=""
  local rviz_non_pid=""
  local port
  port="$(find_free_port)"

  mkdir -p "${run_dir}" "${ros_home}" "${ros_log_dir}"

  local -a ros_env=(
    "ROS_MASTER_URI=http://127.0.0.1:${port}"
    "ROS_IP=127.0.0.1"
    "ROS_HOSTNAME=127.0.0.1"
    "ROS_HOME=${ros_home}"
    "ROS_LOG_DIR=${ros_log_dir}"
  )

  env "${ros_env[@]}" roscore -p "${port}" >"${roscore_log}" 2>&1 &
  local roscore_pid=$!
  local rc=1

  cleanup_case() {
    trap - RETURN
    env "${ros_env[@]}" rosnode kill -a >/dev/null 2>&1 || true
    [[ -n "${rviz_world_pid:-}" ]] && kill "${rviz_world_pid}" >/dev/null 2>&1 || true
    [[ -n "${rviz_non_pid:-}" ]] && kill "${rviz_non_pid}" >/dev/null 2>&1 || true
    [[ -n "${roscore_pid:-}" ]] && kill "${roscore_pid}" >/dev/null 2>&1 || true
    [[ -n "${rviz_world_pid:-}" ]] && wait "${rviz_world_pid}" 2>/dev/null || true
    [[ -n "${rviz_non_pid:-}" ]] && wait "${rviz_non_pid}" 2>/dev/null || true
    [[ -n "${roscore_pid:-}" ]] && wait "${roscore_pid}" 2>/dev/null || true
  }
  trap cleanup_case RETURN

  if ! wait_for_rosparam 15 "${ros_env[@]}"; then
    echo "ERROR: roscore did not become ready for ${run_tag}" >&2
    append_per_run_row "${mode}" "${kp}" "nan" "nan" "" "${run_tag}" "${metrics_csv}" "1"
    return 1
  fi

  {
    env "${ros_env[@]}" rosparam load "${WORKSPACE}/${YAML_REL}" /num_sim_non_one_point_node
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/frame_mode "${mode}"
    if [[ "${mode}" == "inertial" ]]; then
      env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/use_inertial_frame true
      env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/use_noninertial_frame false
    else
      env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/use_inertial_frame false
      env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/use_noninertial_frame true
    fi
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/safety_variant A2_soft_cbf
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/metrics/debug false
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/active_uavs "[${ACTIVE_UAVS}]"
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/random_obstacles/count "${OBSTACLE_COUNT}"
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/random_obstacles/seed "${SEED}"
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/noise_seed "${SEED}"
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/kappa "${kp}"
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/metrics_csv "${metrics_csv}"
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/run_tag "${run_tag}"
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/use_dynamic_obs false
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/warm_start true
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/dynamic_obs/start_z "${START_Z}"
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/car_trajectory_mode "${CAR_TRAJECTORY_MODE}"
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/obstacles_for_car_traj_only "${NO_OBSTACLE_AVOIDANCE}"
    env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/publish_obstacle_markers false
    if [[ "${NO_OBSTACLE_AVOIDANCE}" == "true" ]]; then
      env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/cbf/enabled false
      env "${ros_env[@]}" rosparam set /num_sim_non_one_point_node/cbf/use_in_sim false
    fi
  } >"${setup_log}" 2>&1

  local ascii_mode_label utf8_mode_label
  if [[ "${mode}" == "inertial" ]]; then
    ascii_mode_label="INERTIAL"
    utf8_mode_label="INERTIAL / 惯性"
  else
    ascii_mode_label="NONINERTIAL"
    utf8_mode_label="NONINERTIAL / 非惯性"
  fi

  if [[ "${USE_RVIZ}" == "true" ]]; then
    local world_rviz_source="${RVIZ_DIR}/world.rviz"
    local non_rviz_source="${RVIZ_DIR}/non_inertial.rviz"
    if [[ "${HIDE_OBSTACLES_IN_RVIZ}" == "true" ]]; then
      prepare_rviz_config "${RVIZ_DIR}/world.rviz" "${rviz_world_cfg}"
      prepare_rviz_config "${RVIZ_DIR}/non_inertial.rviz" "${rviz_non_cfg}"
      world_rviz_source="${rviz_world_cfg}"
      non_rviz_source="${rviz_non_cfg}"
    fi

    env "${ros_env[@]}" rviz -d "${world_rviz_source}" >"${rviz_world_log}" 2>&1 &
    rviz_world_pid=$!
    tag_window_by_pid \
      "${rviz_world_pid}" \
      "[${ascii_mode_label}][kp=${kp}] world.rviz" \
      "[${utf8_mode_label}][kp=${kp}] world.rviz" &
    if [[ "${RVIZ_VIEW}" == "both" ]]; then
      env "${ros_env[@]}" rviz -d "${non_rviz_source}" >"${rviz_non_log}" 2>&1 &
      rviz_non_pid=$!
      tag_window_by_pid \
        "${rviz_non_pid}" \
        "[${ascii_mode_label}][kp=${kp}] non_inertial.rviz" \
        "[${utf8_mode_label}][kp=${kp}] non_inertial.rviz" &
    fi
  fi

  echo "[INFO] mode=${mode} kp=${kp} track_offset=${TRACK_OFFSET} v=${V} w=${W} obs=${OBSTACLE_COUNT} seed=${SEED} car_trajectory_mode=${CAR_TRAJECTORY_MODE} avoidance_disabled=${NO_OBSTACLE_AVOIDANCE} rviz=${USE_RVIZ}/${RVIZ_VIEW} hide_obstacles=${HIDE_OBSTACLES_IN_RVIZ} run_tag=${run_tag}"

  set +e
  timeout --signal=INT --kill-after=10 "${TIMEOUT_SEC}" \
    env "${ros_env[@]}" rosrun coni_mpc num_sim_non_one_point_node \
      -r "${TRACK_OFFSET}" -v "${V}" -w "${W}" >"${rosrun_log}" 2>&1
  rc=$?
  set -e

  local parsed
  parsed="$(python3 - "${metrics_csv}" "${run_tag}" <<'PY'
import csv
import pathlib
import sys

metrics_csv = pathlib.Path(sys.argv[1])
run_tag = sys.argv[2]
if not metrics_csv.exists():
    print("nan,nan,,missing")
    raise SystemExit(0)

with metrics_csv.open("r", encoding="utf-8", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        if row.get("run_tag") == run_tag and row.get("scope") == "all":
            print(
                f"{row.get('tracking_rms', 'nan')},"
                f"{row.get('fail_rate', 'nan')},"
                f"{row.get('frame_mode_effective', '')},ok"
            )
            raise SystemExit(0)

print("nan,nan,,missing")
PY
)"

  local tracking_rms fail_rate effective_mode parse_status
  IFS=',' read -r tracking_rms fail_rate effective_mode parse_status <<< "${parsed}"
  append_per_run_row "${mode}" "${kp}" "${tracking_rms}" "${fail_rate}" "${effective_mode}" "${run_tag}" "${metrics_csv}" "${rc}"

  if [[ "${parse_status}" != "ok" ]]; then
    echo "WARN: metrics not found for ${run_tag}. Check ${rosrun_log}" >&2
  fi
  if [[ "${rc}" -ne 0 ]]; then
    echo "WARN: rosrun exited with rc=${rc} for ${run_tag}. Check ${rosrun_log}" >&2
  fi
}

require_cmd python3
require_cmd timeout
require_cmd roscore
require_cmd rosparam
require_cmd rosnode
require_cmd rosrun
if [[ "${USE_RVIZ}" == "true" ]]; then
  require_cmd rviz
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "ERROR: USE_RVIZ=true but DISPLAY is empty." >&2
    exit 2
  fi
fi

mkdir -p "${LOGS_ROOT}"
printf 'frame_mode,kp,tracking_rms,fail_rate,frame_mode_effective,run_tag,metrics_csv,return_code\n' > "${PER_RUN_CSV}"

cd "${WORKSPACE}"
source "${WORKSPACE}/devel/setup.bash"
rospack find coni_mpc >/dev/null

echo "[INFO] workspace: ${WORKSPACE}"
echo "[INFO] run_root: ${RUN_ROOT}"
echo "[INFO] circle settings: track_offset=${TRACK_OFFSET} v=${V} w=${W} obstacle_count=${OBSTACLE_COUNT} seed=${SEED} kp_list=${KP_LIST} car_trajectory_mode=${CAR_TRAJECTORY_MODE} no_obstacle_avoidance=${NO_OBSTACLE_AVOIDANCE} use_rviz=${USE_RVIZ} rviz_view=${RVIZ_VIEW} hide_obstacles=${HIDE_OBSTACLES_IN_RVIZ}"

IFS=',' read -r -a raw_kps <<< "${KP_LIST}"
run_index=0
for raw_kp in "${raw_kps[@]}"; do
  kp="$(trim "${raw_kp}")"
  [[ -z "${kp}" ]] && continue
  for mode in noninertial inertial; do
    run_index=$((run_index + 1))
    run_case "${mode}" "${kp}" "${run_index}"
  done
done

python3 - "${PER_RUN_CSV}" "${SUMMARY_CSV}" <<'PY'
import csv
import math
import pathlib
import sys

per_run_csv = pathlib.Path(sys.argv[1])
summary_csv = pathlib.Path(sys.argv[2])

rows = list(csv.DictReader(per_run_csv.open("r", encoding="utf-8", newline="")))

def safe_float(text: str) -> float:
    try:
        return float(text)
    except Exception:
        return math.nan

grouped = {}
for row in rows:
    kp = row["kp"]
    mode = row["frame_mode"]
    grouped.setdefault(kp, {}).setdefault(mode, []).append(safe_float(row["tracking_rms"]))

header = [
    "kp",
    "noninertial_tracking_rms",
    "inertial_tracking_rms",
    "pct_noninertial_better_tracking_rms",
]
out_rows = []
for kp in sorted(grouped.keys(), key=lambda x: float(x)):
    non_vals = [v for v in grouped[kp].get("noninertial", []) if math.isfinite(v)]
    in_vals = [v for v in grouped[kp].get("inertial", []) if math.isfinite(v)]
    non_mean = sum(non_vals) / len(non_vals) if non_vals else math.nan
    in_mean = sum(in_vals) / len(in_vals) if in_vals else math.nan
    pct = (
        (in_mean - non_mean) / in_mean * 100.0
        if math.isfinite(non_mean) and math.isfinite(in_mean) and abs(in_mean) > 1e-12
        else math.nan
    )
    out_rows.append({
        "kp": kp,
        "noninertial_tracking_rms": "nan" if not math.isfinite(non_mean) else f"{non_mean:.6f}",
        "inertial_tracking_rms": "nan" if not math.isfinite(in_mean) else f"{in_mean:.6f}",
        "pct_noninertial_better_tracking_rms": "nan" if not math.isfinite(pct) else f"{pct:.6f}",
    })

with summary_csv.open("w", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=header)
    writer.writeheader()
    writer.writerows(out_rows)

for row in out_rows:
    print(
        f"kp={row['kp']} "
        f"noninertial_tracking_rms={row['noninertial_tracking_rms']} "
        f"inertial_tracking_rms={row['inertial_tracking_rms']} "
        f"pct_noninertial_better_tracking_rms={row['pct_noninertial_better_tracking_rms']}%"
    )
PY

echo
echo "[INFO] per-run csv: ${PER_RUN_CSV}"
echo "[INFO] summary csv: ${SUMMARY_CSV}"
echo
column -s, -t < "${SUMMARY_CSV}"
