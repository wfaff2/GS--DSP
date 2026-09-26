#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"

TRACK_OFFSET="${TRACK_OFFSET:-1.0}"
V="${V:-2.0}"
W="${W:-0.25}"
OBSTACLE_COUNT="${OBSTACLE_COUNT:-100}"
SEED="${SEED:-3}"
KP="${KP:-${KAPPA:-1.0}}"
USE_RVIZ="${USE_RVIZ:-true}"
RVIZ_VIEW="${RVIZ_VIEW:-world}"
NO_OBSTACLE_AVOIDANCE="${NO_OBSTACLE_AVOIDANCE:-true}"
START_Z="${START_Z:-2.0}"
ACTIVE_UAVS="${ACTIVE_UAVS:-1,2,3}"
METRIC_SCOPE="${METRIC_SCOPE:-all}"
TIMEOUT_SEC="${TIMEOUT_SEC:-900}"               
YAML_REL="${YAML_REL:-src/coni_mpc/parameters/num_sim_non_one_point.yaml}"
RVIZ_DIR="${WORKSPACE}/src/coni_mpc/rviz"

NON_PORT="${NON_PORT:-11331}"
IN_PORT="${IN_PORT:-11332}"

RUN_STAMP="$(date +%Y%m%d-%H%M%S)"
RUN_ROOT="${RUN_ROOT:-${WORKSPACE}/results/astar_compare_kp_rviz_${RUN_STAMP}}"
SUMMARY_CSV="${RUN_ROOT}/tracking_rms_summary.csv"
PER_MODE_CSV="${RUN_ROOT}/per_mode.csv"

declare -a SESSION_PGIDS=()
NON_SESSION_PID=""
IN_SESSION_PID=""
NON_SESSION_LOG=""
IN_SESSION_LOG=""
NON_ROSRUN_LOG=""
IN_ROSRUN_LOG=""
NON_METRICS_CSV=""
IN_METRICS_CSV=""
NON_RUN_TAG=""
IN_RUN_TAG=""
LAST_CASE_WAIT_RC=0

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "ERROR: missing command: ${cmd}" >&2
    exit 2
  fi
}

kill_by_cmd_substr() {
  local signal_name="$1"
  local needle="$2"
  local pid args
  while read -r pid args; do
    [[ -z "${pid}" ]] && continue
    [[ "${args}" == *"${needle}"* ]] || continue
    kill "-${signal_name}" "${pid}" >/dev/null 2>&1 || true
  done < <(ps -eo pid=,args=)
}

reap_session_jobs() {
  local pid
  for pid in "${SESSION_PGIDS[@]:-}"; do
    [[ -n "${pid}" ]] || continue
    wait "${pid}" 2>/dev/null || true
  done
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

tag_rviz_windows_for_port() {
  local port="$1"
  local ascii_label="$2"
  local utf8_label="$3"
  local uav_label="$4"
  local kp_label="$5"
  local idle_rounds=0
  while true; do
    local found_this_round=0
    while read -r pid args; do
      [[ -z "${pid}" ]] && continue
      if ! ps eww -p "${pid}" 2>/dev/null | grep -q "ROS_MASTER_URI=http://127.0.0.1:${port}"; then
        continue
      fi
      local wid
      wid="$(window_id_by_pid "${pid}" || true)"
      [[ -z "${wid}" ]] && continue
      if [[ "${args}" == *"/src/coni_mpc/rviz/world.rviz"* ]]; then
        rename_window \
          "${wid}" \
          "[${ascii_label}][${uav_label}][kp=${kp_label}] world.rviz" \
          "[${utf8_label}][${uav_label}][kp=${kp_label}] world.rviz"
      else
        rename_window \
          "${wid}" \
          "[${ascii_label}][${uav_label}][kp=${kp_label}] non_inertial.rviz" \
          "[${utf8_label}][${uav_label}][kp=${kp_label}] non_inertial.rviz"
      fi
      found_this_round=1
    done < <(ps -eo pid=,args= | grep -E '(^| )rviz -d .*(world|non_inertial)\.rviz' | grep -v grep || true)

    if ((found_this_round == 1)); then
      idle_rounds=0
    else
      idle_rounds=$((idle_rounds + 1))
      if ((idle_rounds >= 10)); then
        return 0
      fi
    fi
    sleep 1
  done
}

wait_for_log_pattern() {
  local log_path="$1"
  local pattern="$2"
  local timeout_sec="$3"
  local deadline=$((SECONDS + timeout_sec))
  while ((SECONDS < deadline)); do
    if [[ -f "${log_path}" ]] && grep -q "${pattern}" "${log_path}"; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

extract_metric() {
  local metrics_csv="$1"
  local run_tag="$2"
  local scope="$3"
  local field="$4"
  python3 - "${metrics_csv}" "${run_tag}" "${scope}" "${field}" <<'PY'
import csv
import pathlib
import sys

metrics_csv = pathlib.Path(sys.argv[1])
run_tag = sys.argv[2]
scope = sys.argv[3]
field = sys.argv[4]
if not metrics_csv.exists():
    print("nan")
    raise SystemExit(0)

with metrics_csv.open("r", encoding="utf-8", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        if row.get("run_tag") == run_tag and row.get("scope") == scope:
            print(row.get(field, "nan"))
            raise SystemExit(0)

print("nan")
PY
}

print_metrics_summary() {
  local metrics_csv="$1"
  local run_tag="$2"
  local scope="$3"
  local label="$4"

  if [[ ! -f "${metrics_csv}" ]]; then
    echo "[WARN] ${label} metrics not found: ${metrics_csv}"
    return 0
  fi

  python3 - "${metrics_csv}" "${run_tag}" "${scope}" "${label}" <<'PY'
import csv
import pathlib
import sys

metrics_csv = pathlib.Path(sys.argv[1])
run_tag = sys.argv[2]
scope = sys.argv[3]
label = sys.argv[4]

with metrics_csv.open("r", encoding="utf-8", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        if row.get("run_tag") == run_tag and row.get("scope") == scope:
            print(
                f"[INFO] {label} result:"
                f" scope={scope}"
                f" tracking_rms={row.get('tracking_rms', 'nan')}"
                f" tracking_rms_high={row.get('tracking_rms_high', 'nan')}"
                f" fail_rate={row.get('fail_rate', 'nan')}"
                f" collision={row.get('collision', 'nan')}"
                f" min_h={row.get('min_h', 'nan')}"
                f" max_slack={row.get('max_slack', 'nan')}"
            )
            raise SystemExit(0)

print(f"[WARN] {label} metrics row not found in {metrics_csv} for scope={scope}")
PY
}

remove_session_pgid() {
  local target="$1"
  local remaining=()
  local pgid
  for pgid in "${SESSION_PGIDS[@]}"; do
    if [[ "${pgid}" != "${target}" ]]; then
      remaining+=("${pgid}")
    fi
  done
  SESSION_PGIDS=("${remaining[@]}")
}

cleanup() {
  trap - EXIT INT TERM
  if ((${#SESSION_PGIDS[@]} > 0)); then
    echo
    echo "[INFO] Stopping compare sessions..."
    for port in "${NON_PORT}" "${IN_PORT}"; do
      env \
        ROS_MASTER_URI="http://127.0.0.1:${port}" \
        ROS_IP="127.0.0.1" \
        ROS_HOSTNAME="127.0.0.1" \
        rosnode kill -a >/dev/null 2>&1 || true
    done
    for pgid in "${SESSION_PGIDS[@]}"; do
      kill -TERM -- "-${pgid}" >/dev/null 2>&1 || true
    done
    sleep 1
    kill_by_cmd_substr TERM "${RUN_ROOT}"
    sleep 1
    for pgid in "${SESSION_PGIDS[@]}"; do
      kill -KILL -- "-${pgid}" >/dev/null 2>&1 || true
    done
    kill_by_cmd_substr KILL "${RUN_ROOT}"
    reap_session_jobs
  fi
}

launch_case() {
  local mode="$1"
  local port="$2"
  local label="$3"
  local prefix="$4"
  local run_tag="${RUN_STAMP}_${prefix}_astar_kp${KP}_s${SEED}"
  local run_dir="${RUN_ROOT}/${prefix}"
  local session_log="${run_dir}/session.log"
  local roscore_log="${run_dir}/roscore.log"
  local rosrun_log="${run_dir}/rosrun.log"
  local setup_log="${run_dir}/setup.log"
  local rviz_world_log="${run_dir}/rviz_world.log"
  local rviz_non_log="${run_dir}/rviz_non_inertial.log"
  local ros_home="${run_dir}/ros_home"
  local ros_log_dir="${run_dir}/ros_logs"
  local metrics_csv="${run_dir}/metrics.csv"

  mkdir -p "${run_dir}" "${ros_home}" "${ros_log_dir}"

  env \
    CASE_MODE="${mode}" \
    CASE_LABEL="${label}" \
    CASE_WORKSPACE="${WORKSPACE}" \
    CASE_PORT="${port}" \
    CASE_TRACK_OFFSET="${TRACK_OFFSET}" \
    CASE_V="${V}" \
    CASE_W="${W}" \
    CASE_OBS="${OBSTACLE_COUNT}" \
    CASE_SEED="${SEED}" \
    CASE_KP="${KP}" \
    CASE_START_Z="${START_Z}" \
    CASE_ACTIVE_UAVS="${ACTIVE_UAVS}" \
    CASE_TIMEOUT_SEC="${TIMEOUT_SEC}" \
    CASE_YAML_REL="${YAML_REL}" \
    CASE_USE_RVIZ="${USE_RVIZ}" \
    CASE_RVIZ_VIEW="${RVIZ_VIEW}" \
    CASE_NO_AVOID="${NO_OBSTACLE_AVOIDANCE}" \
    CASE_RVIZ_DIR="${RVIZ_DIR}" \
    CASE_RUN_TAG="${run_tag}" \
    CASE_METRICS_CSV="${metrics_csv}" \
    CASE_ROSCORE_LOG="${roscore_log}" \
    CASE_ROSRUN_LOG="${rosrun_log}" \
    CASE_SETUP_LOG="${setup_log}" \
    CASE_RVIZ_WORLD_LOG="${rviz_world_log}" \
    CASE_RVIZ_NON_LOG="${rviz_non_log}" \
    CASE_ROS_HOME="${ros_home}" \
    CASE_ROS_LOG_DIR="${ros_log_dir}" \
    setsid bash -lc '
      set -euo pipefail
      cd "${CASE_WORKSPACE}"
      source "${CASE_WORKSPACE}/devel/setup.bash"

      export ROS_MASTER_URI="http://127.0.0.1:${CASE_PORT}"
      export ROS_IP="127.0.0.1"
      export ROS_HOSTNAME="127.0.0.1"
      export ROS_HOME="${CASE_ROS_HOME}"
      export ROS_LOG_DIR="${CASE_ROS_LOG_DIR}"

      roscore -p "${CASE_PORT}" >"${CASE_ROSCORE_LOG}" 2>&1 &
      ROSCORE_PID=$!
      RVIZ_WORLD_PID=""
      RVIZ_NON_PID=""

      cleanup_inner() {
        if rosparam list >/dev/null 2>&1; then
          rosnode kill -a >/dev/null 2>&1 || true
        fi
        if [[ -n "${RVIZ_WORLD_PID}" ]]; then
          kill "${RVIZ_WORLD_PID}" >/dev/null 2>&1 || true
          wait "${RVIZ_WORLD_PID}" 2>/dev/null || true
        fi
        if [[ -n "${RVIZ_NON_PID}" ]]; then
          kill "${RVIZ_NON_PID}" >/dev/null 2>&1 || true
          wait "${RVIZ_NON_PID}" 2>/dev/null || true
        fi
        kill "${ROSCORE_PID}" >/dev/null 2>&1 || true
        wait "${ROSCORE_PID}" 2>/dev/null || true
      }
      trap cleanup_inner EXIT INT TERM

      until rosparam list >/dev/null 2>&1; do
        sleep 0.2
      done

      {
        rosparam load "${CASE_WORKSPACE}/${CASE_YAML_REL}" /num_sim_non_one_point_node
        rosparam set /num_sim_non_one_point_node/frame_mode "${CASE_MODE}"
        if [[ "${CASE_MODE}" == "inertial" ]]; then
          rosparam set /num_sim_non_one_point_node/use_inertial_frame true
          rosparam set /num_sim_non_one_point_node/use_noninertial_frame false
        else
          rosparam set /num_sim_non_one_point_node/use_inertial_frame false
          rosparam set /num_sim_non_one_point_node/use_noninertial_frame true
        fi
        rosparam set /num_sim_non_one_point_node/safety_variant A2_soft_cbf
        rosparam set /num_sim_non_one_point_node/metrics/debug false
        rosparam set /num_sim_non_one_point_node/active_uavs "[${CASE_ACTIVE_UAVS}]"
        rosparam set /num_sim_non_one_point_node/random_obstacles/count "${CASE_OBS}"
        rosparam set /num_sim_non_one_point_node/random_obstacles/seed "${CASE_SEED}"
        rosparam set /num_sim_non_one_point_node/noise_seed "${CASE_SEED}"
        rosparam set /num_sim_non_one_point_node/kappa "${CASE_KP}"
        rosparam set /num_sim_non_one_point_node/metrics_csv "${CASE_METRICS_CSV}"
        rosparam set /num_sim_non_one_point_node/run_tag "${CASE_RUN_TAG}"
        rosparam set /num_sim_non_one_point_node/use_dynamic_obs false
        rosparam set /num_sim_non_one_point_node/warm_start true
        rosparam set /num_sim_non_one_point_node/dynamic_obs/start_z "${CASE_START_Z}"
        rosparam set /num_sim_non_one_point_node/car_trajectory_mode obstacle_aware
        rosparam set /num_sim_non_one_point_node/obstacles_for_car_traj_only "${CASE_NO_AVOID}"
        rosparam set /num_sim_non_one_point_node/publish_obstacle_markers false
        if [[ "${CASE_NO_AVOID}" == "true" ]]; then
          rosparam set /num_sim_non_one_point_node/cbf/enabled false
          rosparam set /num_sim_non_one_point_node/cbf/use_in_sim false
        fi
      } >"${CASE_SETUP_LOG}" 2>&1

      if [[ "${CASE_USE_RVIZ}" == "true" ]]; then
        rviz -d "${CASE_RVIZ_DIR}/world.rviz" >"${CASE_RVIZ_WORLD_LOG}" 2>&1 &
        RVIZ_WORLD_PID=$!
        if [[ "${CASE_RVIZ_VIEW}" == "both" ]]; then
          rviz -d "${CASE_RVIZ_DIR}/non_inertial.rviz" >"${CASE_RVIZ_NON_LOG}" 2>&1 &
          RVIZ_NON_PID=$!
        fi
      fi

      echo "[INFO][${CASE_LABEL}] kp=${CASE_KP} active_uavs=[${CASE_ACTIVE_UAVS}] r=${CASE_TRACK_OFFSET} v=${CASE_V} w=${CASE_W} obs=${CASE_OBS} seed=${CASE_SEED}"

      timeout --signal=INT --kill-after=10 "${CASE_TIMEOUT_SEC}" \
        rosrun coni_mpc num_sim_non_one_point_node \
          -r "${CASE_TRACK_OFFSET}" \
          -v "${CASE_V}" \
          -w "${CASE_W}" >"${CASE_ROSRUN_LOG}" 2>&1
    ' >"${session_log}" 2>&1 &

  local session_pid=$!
  SESSION_PGIDS+=("${session_pid}")

  if [[ "${prefix}" == "noninertial" ]]; then
    NON_SESSION_PID="${session_pid}"
    NON_SESSION_LOG="${session_log}"
    NON_ROSRUN_LOG="${rosrun_log}"
    NON_METRICS_CSV="${metrics_csv}"
    NON_RUN_TAG="${run_tag}"
  else
    IN_SESSION_PID="${session_pid}"
    IN_SESSION_LOG="${session_log}"
    IN_ROSRUN_LOG="${rosrun_log}"
    IN_METRICS_CSV="${metrics_csv}"
    IN_RUN_TAG="${run_tag}"
  fi
}

run_case_and_wait() {
  local mode="$1"
  local port="$2"
  local label="$3"
  local prefix="$4"
  local expected_frame="$5"
  local session_pid=""
  local rosrun_log=""
  local metrics_csv=""
  local run_tag=""
  local rviz_ascii_label=""
  local rviz_utf8_label=""

  echo
  echo "[INFO] Starting ${label} case..."
  launch_case "${mode}" "${port}" "${label}" "${prefix}"

  if [[ "${prefix}" == "noninertial" ]]; then
    session_pid="${NON_SESSION_PID}"
    rosrun_log="${NON_ROSRUN_LOG}"
    metrics_csv="${NON_METRICS_CSV}"
    run_tag="${NON_RUN_TAG}"
    rviz_ascii_label="NONINERTIAL"
    rviz_utf8_label="NONINERTIAL / 非惯性"
  else
    session_pid="${IN_SESSION_PID}"
    rosrun_log="${IN_ROSRUN_LOG}"
    metrics_csv="${IN_METRICS_CSV}"
    run_tag="${IN_RUN_TAG}"
    rviz_ascii_label="INERTIAL"
    rviz_utf8_label="INERTIAL / 惯性"
  fi

  if ! wait_for_log_pattern "${rosrun_log}" "frame_mode_effective=${expected_frame}" 60; then
    echo "ERROR: ${prefix} session did not report frame_mode_effective=${expected_frame}" >&2
    echo "Check log: ${rosrun_log}" >&2
    exit 1
  fi

  if [[ "${USE_RVIZ}" == "true" ]] && command -v xprop >/dev/null 2>&1; then
    tag_rviz_windows_for_port "${port}" "${rviz_ascii_label}" "${rviz_utf8_label}" "${ACTIVE_UAVS_TITLE}" "${KP}" &
  fi

  echo "[INFO] ${label} log: ${rosrun_log}"
  echo "[INFO] ${label} metrics: ${metrics_csv}"

  set +e
  wait "${session_pid}"
  LAST_CASE_WAIT_RC=$?
  set -e

  remove_session_pgid "${session_pid}"

  if [[ "${LAST_CASE_WAIT_RC}" -ne 0 ]]; then
    echo "WARN: ${prefix} session exited with rc=${LAST_CASE_WAIT_RC}. Check ${rosrun_log}" >&2
  fi

  print_metrics_summary "${metrics_csv}" "${run_tag}" "${METRIC_SCOPE}" "${label}"
}

require_cmd setsid
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

ACTIVE_UAVS_TITLE=""
IFS=',' read -r -a active_parts <<< "${ACTIVE_UAVS}"
for part in "${active_parts[@]}"; do
  part="${part//[[:space:]]/}"
  [[ -z "${part}" ]] && continue
  if [[ -n "${ACTIVE_UAVS_TITLE}" ]]; then
    ACTIVE_UAVS_TITLE+="+"
  fi
  ACTIVE_UAVS_TITLE+="UAV${part}"
done
if [[ -z "${ACTIVE_UAVS_TITLE}" ]]; then
  ACTIVE_UAVS_TITLE="UAV2"
fi

mkdir -p "${RUN_ROOT}"
trap cleanup EXIT INT TERM

echo "[INFO] workspace: ${WORKSPACE}"
echo "[INFO] run_root: ${RUN_ROOT}"
echo "[INFO] settings: kp=${KP} active_uavs=${ACTIVE_UAVS_TITLE} metric_scope=${METRIC_SCOPE} r=${TRACK_OFFSET} v=${V} w=${W} obs=${OBSTACLE_COUNT} seed=${SEED} use_rviz=${USE_RVIZ} rviz_view=${RVIZ_VIEW}"

run_case_and_wait noninertial "${NON_PORT}" "非惯性" "noninertial" "noninertial"
NON_WAIT_RC="${LAST_CASE_WAIT_RC}"
run_case_and_wait inertial "${IN_PORT}" "惯性" "inertial" "inertial"
IN_WAIT_RC="${LAST_CASE_WAIT_RC}"

NON_TRACKING_RMS="$(extract_metric "${NON_METRICS_CSV}" "${NON_RUN_TAG}" "${METRIC_SCOPE}" tracking_rms)"
IN_TRACKING_RMS="$(extract_metric "${IN_METRICS_CSV}" "${IN_RUN_TAG}" "${METRIC_SCOPE}" tracking_rms)"
NON_TRACKING_RMS_HIGH="$(extract_metric "${NON_METRICS_CSV}" "${NON_RUN_TAG}" "${METRIC_SCOPE}" tracking_rms_high)"
IN_TRACKING_RMS_HIGH="$(extract_metric "${IN_METRICS_CSV}" "${IN_RUN_TAG}" "${METRIC_SCOPE}" tracking_rms_high)"
NON_FAIL_RATE="$(extract_metric "${NON_METRICS_CSV}" "${NON_RUN_TAG}" "${METRIC_SCOPE}" fail_rate)"
IN_FAIL_RATE="$(extract_metric "${IN_METRICS_CSV}" "${IN_RUN_TAG}" "${METRIC_SCOPE}" fail_rate)"

python3 - "${PER_MODE_CSV}" "${SUMMARY_CSV}" "${KP}" "${ACTIVE_UAVS_TITLE}" \
  "${METRIC_SCOPE}" "${W}" \
  "${NON_TRACKING_RMS}" "${IN_TRACKING_RMS}" \
  "${NON_TRACKING_RMS_HIGH}" "${IN_TRACKING_RMS_HIGH}" \
  "${NON_FAIL_RATE}" "${IN_FAIL_RATE}" <<'PY'
import csv
import math
import pathlib
import sys

per_mode_csv = pathlib.Path(sys.argv[1])
summary_csv = pathlib.Path(sys.argv[2])
kp = sys.argv[3]
active_uavs = sys.argv[4]
metric_scope = sys.argv[5]
requested_w = sys.argv[6]
non_tracking = sys.argv[7]
in_tracking = sys.argv[8]
non_tracking_high = sys.argv[9]
in_tracking_high = sys.argv[10]
non_fail = sys.argv[11]
in_fail = sys.argv[12]

def to_float(text: str) -> float:
    try:
        return float(text)
    except Exception:
        return math.nan

def advantage_pct(non_value: float, in_value: float) -> float:
    if not math.isfinite(non_value) or not math.isfinite(in_value):
        return math.nan
    if abs(in_value) <= 1e-12:
        return math.nan
    return (in_value - non_value) / in_value * 100.0

def delta_value(non_value: float, in_value: float) -> float:
    if not math.isfinite(non_value) or not math.isfinite(in_value):
        return math.nan
    return non_value - in_value

non_val = to_float(non_tracking)
in_val = to_float(in_tracking)
non_high_val = to_float(non_tracking_high)
in_high_val = to_float(in_tracking_high)
tracking_adv_pct = advantage_pct(non_val, in_val)
tracking_high_adv_pct = advantage_pct(non_high_val, in_high_val)
tracking_delta = delta_value(non_val, in_val)
tracking_high_delta = delta_value(non_high_val, in_high_val)

with per_mode_csv.open("w", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(
        f,
        fieldnames=[
            "frame_mode",
            "kp",
            "requested_w",
            "active_uavs",
            "metric_scope",
            "tracking_rms",
            "tracking_rms_high",
            "fail_rate",
        ],
    )
    writer.writeheader()
    writer.writerow({
        "frame_mode": "noninertial",
        "kp": kp,
        "requested_w": requested_w,
        "active_uavs": active_uavs,
        "metric_scope": metric_scope,
        "tracking_rms": non_tracking,
        "tracking_rms_high": non_tracking_high,
        "fail_rate": non_fail,
    })
    writer.writerow({
        "frame_mode": "inertial",
        "kp": kp,
        "requested_w": requested_w,
        "active_uavs": active_uavs,
        "metric_scope": metric_scope,
        "tracking_rms": in_tracking,
        "tracking_rms_high": in_tracking_high,
        "fail_rate": in_fail,
    })

with summary_csv.open("w", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(
        f,
        fieldnames=[
            "kp",
            "requested_w",
            "active_uavs",
            "metric_scope",
            "noninertial_tracking_rms",
            "inertial_tracking_rms",
            "delta_tracking_rms_non_minus_inertial",
            "noninertial_tracking_rms_high",
            "inertial_tracking_rms_high",
            "delta_tracking_rms_high_non_minus_inertial",
            "pct_noninertial_better_tracking_rms",
            "pct_noninertial_better_tracking_rms_high",
        ],
    )
    writer.writeheader()
    writer.writerow({
        "kp": kp,
        "requested_w": requested_w,
        "active_uavs": active_uavs,
        "metric_scope": metric_scope,
        "noninertial_tracking_rms": non_tracking,
        "inertial_tracking_rms": in_tracking,
        "delta_tracking_rms_non_minus_inertial":
            "nan" if not math.isfinite(tracking_delta) else f"{tracking_delta:.6f}",
        "noninertial_tracking_rms_high": non_tracking_high,
        "inertial_tracking_rms_high": in_tracking_high,
        "delta_tracking_rms_high_non_minus_inertial":
            "nan" if not math.isfinite(tracking_high_delta) else f"{tracking_high_delta:.6f}",
        "pct_noninertial_better_tracking_rms":
            "nan" if not math.isfinite(tracking_adv_pct) else f"{tracking_adv_pct:.6f}",
        "pct_noninertial_better_tracking_rms_high":
            "nan" if not math.isfinite(tracking_high_adv_pct) else f"{tracking_high_adv_pct:.6f}",
    })

print(
    f"kp={kp} requested_w={requested_w} active_uavs={active_uavs} metric_scope={metric_scope} "
    f"noninertial_tracking_rms={non_tracking} "
    f"inertial_tracking_rms={in_tracking} "
    f"delta_tracking_rms_non_minus_inertial="
    f"{'nan' if not math.isfinite(tracking_delta) else f'{tracking_delta:.6f}'} "
    f"noninertial_tracking_rms_high={non_tracking_high} "
    f"inertial_tracking_rms_high={in_tracking_high} "
    f"delta_tracking_rms_high_non_minus_inertial="
    f"{'nan' if not math.isfinite(tracking_high_delta) else f'{tracking_high_delta:.6f}'} "
    f"pct_noninertial_better_tracking_rms="
    f"{'nan' if not math.isfinite(tracking_adv_pct) else f'{tracking_adv_pct:.6f}%'} "
    f"pct_noninertial_better_tracking_rms_high="
    f"{'nan' if not math.isfinite(tracking_high_adv_pct) else f'{tracking_high_adv_pct:.6f}%'}"
)
PY

echo
echo "[INFO] per-mode csv: ${PER_MODE_CSV}"
echo "[INFO] summary csv: ${SUMMARY_CSV}"
echo
column -s, -t < "${SUMMARY_CSV}"
