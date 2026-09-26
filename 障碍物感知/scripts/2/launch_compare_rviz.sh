#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
LAUNCH_FILE="${WORKSPACE}/src/coni_mpc/launch/num_sim_non_one_point.launch"

# User-editable settings.
# Examples:
#   V=0.8 W=0.3 ./scripts/launch_compare_rviz.sh
#   SEEDS=1,2,3 USE_RVIZ=false ./scripts/launch_compare_rviz.sh
#   SEED_START=1 SEED_END=10 USE_RVIZ=false ./scripts/launch_compare_rviz.sh
#   UGV_ROLLOUT_MODE=stage USE_RVIZ=false ./scripts/launch_compare_rviz.sh
#   UGV_ROLLOUT_MODE=frozen USE_RVIZ=false ./scripts/launch_compare_rviz.sh
R="${R:-1.0}"
V="${V:-3.0}"
W="${W:-0.5}"
OBSTACLE_COUNT="${OBSTACLE_COUNT:-100}"
SEED="${SEED:-3}"
SEEDS="${SEEDS:-}"
SEED_START="${SEED_START:-}"
SEED_END="${SEED_END:-}"
KAPPA="${KAPPA:-1.0}"
UGV_ROLLOUT_MODE="${UGV_ROLLOUT_MODE:-frozen}"
SAFETY_VARIANT="${SAFETY_VARIANT:-A2_soft_cbf}"
CBF_ENABLED="${CBF_ENABLED:-}"
CBF_USE_IN_SIM="${CBF_USE_IN_SIM:-}"
CBF_SLACK_MAX="${CBF_SLACK_MAX:-}"
CBF_R_SLACK="${CBF_R_SLACK:-}"
CAR_TRAJECTORY_MODE="${CAR_TRAJECTORY_MODE:-obstacle_aware}"
UGV_LONGITUDINAL_ACCEL="${UGV_LONGITUDINAL_ACCEL:-2.0}"
UGV_LONGITUDINAL_DECEL="${UGV_LONGITUDINAL_DECEL:-2.5}"
OBSTACLES_FOR_CAR_TRAJ_ONLY="${OBSTACLES_FOR_CAR_TRAJ_ONLY:-}"
ACTIVE_UAVS="${ACTIVE_UAVS:-}"
USE_RVIZ="${USE_RVIZ:-true}"
METRICS_WAIT_SEC="${METRICS_WAIT_SEC:-900}"
FRAME_LOG_WAIT_SEC="${FRAME_LOG_WAIT_SEC:-40}"
RVIZ_APPEAR_WAIT_SEC="${RVIZ_APPEAR_WAIT_SEC:-8}"
SESSION_STOP_GRACE_SEC="${SESSION_STOP_GRACE_SEC:-0.5}"
PLOT_COMPARE_CURVES="${PLOT_COMPARE_CURVES:-true}"
PLOT_UGV_HORIZON_CURVES="${PLOT_UGV_HORIZON_CURVES:-true}"
PLOT_DISTANCE_FIELD="${PLOT_DISTANCE_FIELD:-solver_planar_surface_distance}"
PLOT_UAV_IDX="${PLOT_UAV_IDX:-0}"
ROBUSTNESS_ACCEL_THR="${ROBUSTNESS_ACCEL_THR:-0.8}"
EXCLUDE_UAV0_FROM_SIMULATION="${EXCLUDE_UAV0_FROM_SIMULATION:-true}"
EXCLUDE_UAV0_FROM_ALL_METRICS="${EXCLUDE_UAV0_FROM_ALL_METRICS:-true}"

NON_PORT="${NON_PORT:-11331}"
IN_PORT="${IN_PORT:-11332}"

RUN_STAMP="$(date +%Y%m%d-%H%M%S)"
RUN_ROOT="${RUN_ROOT:-${WORKSPACE}/results/live_compare_rviz_${RUN_STAMP}}"
RUN_ROOT="$(realpath -m "${RUN_ROOT}")"

declare -a SESSION_PGIDS=()
declare -a TAGGER_PIDS=()
declare -a SEED_LIST=()
LAST_SESSION_LOG=""
LAST_SESSION_METRICS_CSV=""
LAST_SESSION_RUN_TAG=""
LAST_SESSION_PID=""
LAST_SESSION_ROS_LOG_DIR=""
BATCH_SEED_METRICS_CSV=""
BATCH_COLLISION_RATE_CSV=""
BATCH_AVERAGE_METRICS_CSV=""
BATCH_SEED_WITNESS_PNG=""
PLOT_STEP_SCRIPT="${WORKSPACE}/scripts/plot_step_obstacle_tracking_curves.py"
PLOT_UGV_HORIZON_SCRIPT="${WORKSPACE}/scripts/plot_ugv_horizon_prediction_curves.py"
PLOT_SEED_STAGEWISE_ALIGNMENT_SCRIPT="${WORKSPACE}/scripts/plot_seed_stagewise_alignment.py"
PLOT_SEED_WITNESS_SCRIPT="${WORKSPACE}/scripts/plot_seed_witness_distances.py"
PLOT_STAGEWISE_ROBUSTNESS_SCRIPT="${WORKSPACE}/scripts/plot_stagewise_robustness.py"

opposite_rollout_mode() {
  case "$1" in
    stage)
      echo "frozen"
      ;;
    frozen)
      echo "stage"
      ;;
    *)
      echo "ERROR: unsupported UGV_ROLLOUT_MODE for stage-vs-frozen plotting: $1" >&2
      exit 2
      ;;
  esac
}

should_auto_close_sessions_after_metrics() {
  [[ "${USE_RVIZ}" == "true" ]] && ((${#SEED_LIST[@]} > 1))
}

pid_is_alive() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1
}

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "ERROR: missing command: ${cmd}" >&2
    exit 2
  fi
}

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "ERROR: missing file: ${path}" >&2
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

append_seed_token() {
  local token="$1"
  if [[ ! "${token}" =~ ^-?[0-9]+$ ]]; then
    echo "ERROR: invalid seed token: ${token}" >&2
    exit 2
  fi
  SEED_LIST+=("${token}")
}

parse_seed_spec() {
  local raw="$1"
  local token start end step value
  raw="${raw//,/ }"
  for token in ${raw}; do
    if [[ "${token}" =~ ^-?[0-9]+$ ]]; then
      append_seed_token "${token}"
      continue
    fi
    if [[ "${token}" =~ ^(-?[0-9]+)-(-?[0-9]+)$ ]]; then
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
    echo "ERROR: unsupported seed spec: ${token}" >&2
    exit 2
  done
}

build_seed_list() {
  SEED_LIST=()
  if [[ -n "${SEEDS}" ]]; then
    parse_seed_spec "${SEEDS}"
  elif [[ -n "${SEED_START}" || -n "${SEED_END}" ]]; then
    local start="${SEED_START:-${SEED}}"
    local end="${SEED_END:-${start}}"
    parse_seed_spec "${start}-${end}"
  else
    SEED_LIST=("${SEED}")
  fi

  if ((${#SEED_LIST[@]} == 0)); then
    echo "ERROR: no seeds resolved." >&2
    exit 2
  fi
}

seed_list_csv() {
  local IFS=","
  echo "${SEED_LIST[*]}"
}

derive_step_metrics_csv_path() {
  local metrics_csv="$1"
  if [[ -z "${metrics_csv}" ]]; then
    echo ""
    return 0
  fi
  if [[ "${metrics_csv}" == *.csv ]]; then
    echo "${metrics_csv%.csv}_steps.csv"
  else
    echo "${metrics_csv}_steps.csv"
  fi
}

derive_ugv_horizon_summary_csv_path() {
  local metrics_csv="$1"
  if [[ -z "${metrics_csv}" ]]; then
    echo ""
    return 0
  fi
  if [[ "${metrics_csv}" == *.csv ]]; then
    echo "${metrics_csv%.csv}_ugv_horizon_summary.csv"
  else
    echo "${metrics_csv}_ugv_horizon_summary.csv"
  fi
}

plot_seed_compare_curves() {
  local seed="$1"
  local non_step_csv="$2"
  local non_run_tag="$3"
  local in_step_csv="$4"
  local in_run_tag="$5"
  local run_dir="$6"

  if [[ "${PLOT_COMPARE_CURVES}" != "true" ]]; then
    return 0
  fi
  if [[ ! -f "${non_step_csv}" ]]; then
    echo "[WARN] seed=${seed} noninertial step metrics not found: ${non_step_csv}"
    return 0
  fi
  if [[ ! -f "${in_step_csv}" ]]; then
    echo "[WARN] seed=${seed} inertial step metrics not found: ${in_step_csv}"
    return 0
  fi

  local out_png="${run_dir}/seed_${seed}_uav${PLOT_UAV_IDX}_${PLOT_DISTANCE_FIELD}_compare.png"
  if python3 "${PLOT_STEP_SCRIPT}" \
      --csv "${non_step_csv}" \
      --run-tag "${non_run_tag}" \
      --label "noninertial" \
      --compare-csv "${in_step_csv}" \
      --compare-run-tag "${in_run_tag}" \
      --compare-label "inertial" \
      --uav "${PLOT_UAV_IDX}" \
      --distance-field "${PLOT_DISTANCE_FIELD}" \
      --title "seed=${seed} uav=${PLOT_UAV_IDX} ${PLOT_DISTANCE_FIELD} vs tracking_error" \
      --out "${out_png}"; then
    echo "[INFO] seed=${seed} compare plot: ${out_png}"
  else
    echo "[WARN] seed=${seed} compare plot generation failed" >&2
  fi
}

plot_seed_ugv_horizon_curves() {
  local seed="$1"
  local stage_summary_csv="$2"
  local stage_run_tag="$3"
  local frozen_summary_csv="$4"
  local frozen_run_tag="$5"
  local run_dir="$6"

  if [[ "${PLOT_UGV_HORIZON_CURVES}" != "true" ]]; then
    return 0
  fi
  if [[ ! -f "${stage_summary_csv}" ]]; then
    echo "[WARN] seed=${seed} stage noninertial UGV horizon summary not found: ${stage_summary_csv}"
    return 0
  fi
  if [[ ! -f "${frozen_summary_csv}" ]]; then
    echo "[WARN] seed=${seed} frozen noninertial UGV horizon summary not found: ${frozen_summary_csv}"
    return 0
  fi

  local out_png="${run_dir}/seed_${seed}_noninertial_stage_vs_frozen_ugv_horizon_rms.png"
  if python3 "${PLOT_UGV_HORIZON_SCRIPT}" \
      --csv "${stage_summary_csv}" \
      --run-tag "${stage_run_tag}" \
      --label "stage" \
      --compare-csv "${frozen_summary_csv}" \
      --compare-run-tag "${frozen_run_tag}" \
      --compare-label "frozen" \
      --metric "horizon_rms_xy" \
      --title "seed=${seed} noninertial ugv horizon rms: stage vs frozen" \
      --out "${out_png}"; then
    echo "[INFO] seed=${seed} noninertial stage-vs-frozen UGV horizon plot: ${out_png}"
  else
    echo "[WARN] seed=${seed} noninertial stage-vs-frozen UGV horizon plot generation failed" >&2
  fi

  local out_three_axis_png="${run_dir}/seed_${seed}_noninertial_stage_vs_frozen_three_axis.png"
  if python3 "${PLOT_SEED_STAGEWISE_ALIGNMENT_SCRIPT}" \
      --stage-step-csv "${run_dir}/noninertial_metrics_steps.csv" \
      --frozen-step-csv "${run_dir}/noninertial_frozen_metrics_steps.csv" \
      --stage-horizon-csv "${stage_summary_csv}" \
      --frozen-horizon-csv "${frozen_summary_csv}" \
      --accel-thr "${ROBUSTNESS_ACCEL_THR}" \
      --title "seed=${seed} noninertial stage vs frozen" \
      --out "${out_three_axis_png}"; then
    echo "[INFO] seed=${seed} noninertial stage-vs-frozen three-axis plot: ${out_three_axis_png}"
  else
    echo "[WARN] seed=${seed} noninertial stage-vs-frozen three-axis plot generation failed" >&2
  fi
}

cleanup_sessions() {
  cleanup_taggers
  if ((${#SESSION_PGIDS[@]} == 0)); then
    return 0
  fi

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
    kill -INT -- "-${pgid}" >/dev/null 2>&1 || true
  done

  sleep "${SESSION_STOP_GRACE_SEC}"
  for pgid in "${SESSION_PGIDS[@]}"; do
    if pid_is_alive "${pgid}"; then
      kill -TERM -- "-${pgid}" >/dev/null 2>&1 || true
    fi
  done
  kill_by_cmd_substr TERM "${RUN_ROOT}"

  sleep "${SESSION_STOP_GRACE_SEC}"
  for pgid in "${SESSION_PGIDS[@]}"; do
    if pid_is_alive "${pgid}"; then
      kill -KILL -- "-${pgid}" >/dev/null 2>&1 || true
    fi
  done
  kill_by_cmd_substr KILL "${RUN_ROOT}"

  # Reap background session jobs so bash does not emit its own
  # "Killed/Terminated" notification for intentionally stopped sessions.
  reap_session_jobs

  SESSION_PGIDS=()
  LAST_SESSION_LOG=""
  LAST_SESSION_METRICS_CSV=""
  LAST_SESSION_RUN_TAG=""
  LAST_SESSION_PID=""
  LAST_SESSION_ROS_LOG_DIR=""
}

cleanup() {
  trap - EXIT INT TERM
  cleanup_sessions
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

metrics_has_row() {
  local metrics_csv="$1"
  local run_tag="$2"
  if [[ ! -f "${metrics_csv}" ]]; then
    return 1
  fi
  python3 - "${metrics_csv}" "${run_tag}" <<'PY'
import csv
import pathlib
import sys

metrics_csv = pathlib.Path(sys.argv[1])
run_tag = sys.argv[2]

with metrics_csv.open("r", encoding="utf-8", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        if row.get("run_tag") == run_tag and row.get("scope") == "all":
            raise SystemExit(0)

raise SystemExit(1)
PY
}

wait_for_metrics_row() {
  local metrics_csv="$1"
  local run_tag="$2"
  local timeout_sec="$3"
  local watched_pid="${4:-}"
  local deadline=$((SECONDS + timeout_sec))
  while ((SECONDS < deadline)); do
    if metrics_has_row "${metrics_csv}" "${run_tag}"; then
      return 0
    fi
    if [[ -n "${watched_pid}" ]] && ! kill -0 "${watched_pid}" >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
  metrics_has_row "${metrics_csv}" "${run_tag}"
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

cleanup_taggers() {
  local pid
  for pid in "${TAGGER_PIDS[@]:-}"; do
    [[ -z "${pid}" ]] && continue
    kill -TERM "${pid}" >/dev/null 2>&1 || true
    kill -KILL "${pid}" >/dev/null 2>&1 || true
    wait "${pid}" 2>/dev/null || true
  done
  TAGGER_PIDS=()
}

find_rviz_pid_for_log_dir() {
  local log_dir="$1"
  local pid args
  while read -r pid args; do
    [[ -z "${pid}" ]] && continue
    [[ "${args}" == *"${log_dir}"* ]] || continue
    echo "${pid}"
    return 0
  done < <(ps -eo pid=,args= | grep -E '/opt/ros/.*/rviz -d .*/src/coni_mpc/rviz/(non_inertial|world)\.rviz' | grep -v grep || true)
  return 1
}

wait_for_rviz_window_for_log_dir() {
  local log_dir="$1"
  local timeout_sec="$2"
  local deadline=$((SECONDS + timeout_sec))
  local pid wid
  while ((SECONDS < deadline)); do
    pid="$(find_rviz_pid_for_log_dir "${log_dir}" || true)"
    if [[ -n "${pid}" ]]; then
      wid="$(window_id_by_pid "${pid}" || true)"
      if [[ -n "${wid}" ]]; then
        echo "${wid}"
        return 0
      fi
    fi
    sleep 0.2
  done
  return 1
}

tag_rviz_window_for_log_dir() {
  local log_dir="$1"
  local ascii_title="$2"
  local utf8_title="$3"
  local timeout_sec="${4:-${RVIZ_APPEAR_WAIT_SEC}}"
  local wid
  wid="$(wait_for_rviz_window_for_log_dir "${log_dir}" "${timeout_sec}" || true)"
  [[ -n "${wid}" ]] || return 1
  rename_window "${wid}" "${ascii_title}" "${utf8_title}"
  return 0
}

launch_case() {
  local mode="$1"
  local port="$2"
  local label="$3"
  local prefix="$4"
  local seed="$5"
  local run_dir="$6"
  local rollout_mode="${7:-${UGV_ROLLOUT_MODE}}"
  local case_use_rviz="${8:-${USE_RVIZ}}"
  local case_safety_variant="${SAFETY_VARIANT}"
  local case_cbf_enabled="${CBF_ENABLED}"
  local case_cbf_use_in_sim="${CBF_USE_IN_SIM}"
  local case_cbf_slack_max="${CBF_SLACK_MAX}"
  local case_cbf_r_slack="${CBF_R_SLACK}"
  local case_obstacles_for_car_traj_only="${OBSTACLES_FOR_CAR_TRAJ_ONLY}"
  case "${case_safety_variant}" in
    A0_no_cbf)
      [[ -n "${case_cbf_enabled}" ]] || case_cbf_enabled="false"
      [[ -n "${case_cbf_use_in_sim}" ]] || case_cbf_use_in_sim="false"
      [[ -n "${case_cbf_slack_max}" ]] || case_cbf_slack_max="0.0"
      ;;
    A1_hard_cbf)
      [[ -n "${case_cbf_enabled}" ]] || case_cbf_enabled="true"
      [[ -n "${case_cbf_use_in_sim}" ]] || case_cbf_use_in_sim="true"
      [[ -n "${case_cbf_slack_max}" ]] || case_cbf_slack_max="0.0"
      ;;
  esac
  local rviz_config="${WORKSPACE}/src/coni_mpc/rviz/world.rviz"
  local rviz_node_name="rviz_world"
  if [[ "${mode}" != "inertial" ]]; then
    rviz_config="${WORKSPACE}/src/coni_mpc/rviz/non_inertial.rviz"
    rviz_node_name="rviz_non"
  fi
  local session_log="${run_dir}/${prefix}.log"
  local metrics_csv="${run_dir}/${prefix}_metrics.csv"
  local run_tag="${RUN_STAMP}_seed${seed}_${prefix}_${rollout_mode}"
  local roscore_log="${run_dir}/${prefix}_roscore.log"
  local ros_home="${run_dir}/${prefix}_ros_home"
  local ros_log_dir="${run_dir}/${prefix}_ros_logs"

  mkdir -p "${run_dir}" "${ros_home}" "${ros_log_dir}"

  env \
    CASE_MODE="${mode}" \
    CASE_PORT="${port}" \
    CASE_LABEL="${label}" \
    CASE_WORKSPACE="${WORKSPACE}" \
    CASE_LAUNCH_FILE="${LAUNCH_FILE}" \
    CASE_R="${R}" \
    CASE_V="${V}" \
    CASE_W="${W}" \
    CASE_METRICS_CSV="${metrics_csv}" \
    CASE_RUN_TAG="${run_tag}" \
    CASE_SAFETY_VARIANT="${case_safety_variant}" \
    CASE_CBF_ENABLED="${case_cbf_enabled}" \
    CASE_CBF_USE_IN_SIM="${case_cbf_use_in_sim}" \
    CASE_CBF_SLACK_MAX="${case_cbf_slack_max}" \
    CASE_CBF_R_SLACK="${case_cbf_r_slack}" \
    CASE_OBSTACLES_FOR_CAR_TRAJ_ONLY="${case_obstacles_for_car_traj_only}" \
    CASE_RVIZ_CONFIG="${rviz_config}" \
    CASE_RVIZ_NODE_NAME="${rviz_node_name}" \
    CASE_OBS="${OBSTACLE_COUNT}" \
    CASE_SEED="${seed}" \
    CASE_KAPPA="${KAPPA}" \
    CASE_UGV_ROLLOUT_MODE="${rollout_mode}" \
    CASE_CAR_TRAJECTORY_MODE="${CAR_TRAJECTORY_MODE}" \
    CASE_UGV_LONGITUDINAL_ACCEL="${UGV_LONGITUDINAL_ACCEL}" \
    CASE_UGV_LONGITUDINAL_DECEL="${UGV_LONGITUDINAL_DECEL}" \
    CASE_ACTIVE_UAVS="${ACTIVE_UAVS}" \
    CASE_EXCLUDE_UAV0_FROM_SIMULATION="${EXCLUDE_UAV0_FROM_SIMULATION}" \
    CASE_EXCLUDE_UAV0_FROM_ALL_METRICS="${EXCLUDE_UAV0_FROM_ALL_METRICS}" \
    CASE_USE_RVIZ="${case_use_rviz}" \
    CASE_ROS_HOME="${ros_home}" \
    CASE_ROS_LOG_DIR="${ros_log_dir}" \
    CASE_ROSCORE_LOG="${roscore_log}" \
    setsid bash -lc '
      set -euo pipefail
      cd "${CASE_WORKSPACE}"
      source "${CASE_WORKSPACE}/devel/setup.bash"

      export ROS_PACKAGE_PATH="${CASE_WORKSPACE}/src:${ROS_PACKAGE_PATH:-}"
      export CMAKE_PREFIX_PATH="${CASE_WORKSPACE}/devel:${CMAKE_PREFIX_PATH:-}"
      export ROS_MASTER_URI="http://127.0.0.1:${CASE_PORT}"
      export ROS_IP="127.0.0.1"
      export ROS_HOSTNAME="127.0.0.1"
      export ROS_HOME="${CASE_ROS_HOME}"
      export ROS_LOG_DIR="${CASE_ROS_LOG_DIR}"

      roscore -p "${CASE_PORT}" >"${CASE_ROSCORE_LOG}" 2>&1 &
      ROSCORE_PID=$!
      ROSLAUNCH_PID=""
      RVIZ_PID=""

      cleanup_inner() {
        if rosparam list >/dev/null 2>&1; then
          rosnode kill -a >/dev/null 2>&1 || true
        fi
        if [[ -n "${RVIZ_PID}" ]]; then
          kill -INT "${RVIZ_PID}" >/dev/null 2>&1 || true
          sleep 1
          kill -TERM "${RVIZ_PID}" >/dev/null 2>&1 || true
          wait "${RVIZ_PID}" 2>/dev/null || true
        fi
        if [[ -n "${ROSLAUNCH_PID}" ]]; then
          kill -INT "${ROSLAUNCH_PID}" >/dev/null 2>&1 || true
          sleep 1
          kill -TERM "${ROSLAUNCH_PID}" >/dev/null 2>&1 || true
          wait "${ROSLAUNCH_PID}" 2>/dev/null || true
        fi
        kill "${ROSCORE_PID}" >/dev/null 2>&1 || true
        wait "${ROSCORE_PID}" 2>/dev/null || true
      }
      trap cleanup_inner EXIT INT TERM

      until rosparam list >/dev/null 2>&1; do
        sleep 0.2
      done

      if [[ "${CASE_USE_RVIZ}" == "true" ]]; then
        rviz -d "${CASE_RVIZ_CONFIG}" __name:="${CASE_RVIZ_NODE_NAME}" &
        RVIZ_PID=$!
        sleep 1
      fi

      echo "[INFO][${CASE_LABEL}] ROS_MASTER_URI=${ROS_MASTER_URI}"
      echo "[INFO][${CASE_LABEL}] r=${CASE_R} v=${CASE_V} w=${CASE_W} obs=${CASE_OBS} seed=${CASE_SEED} kappa=${CASE_KAPPA} safety_variant=${CASE_SAFETY_VARIANT:-yaml_default} cbf_enabled=${CASE_CBF_ENABLED:-yaml_default} cbf_use_in_sim=${CASE_CBF_USE_IN_SIM:-yaml_default} ugv_rollout_mode=${CASE_UGV_ROLLOUT_MODE} car_trajectory_mode=${CASE_CAR_TRAJECTORY_MODE:-yaml_default} ugv_longitudinal_accel=${CASE_UGV_LONGITUDINAL_ACCEL:-yaml_default} ugv_longitudinal_decel=${CASE_UGV_LONGITUDINAL_DECEL:-yaml_default} obstacles_for_car_traj_only=${CASE_OBSTACLES_FOR_CAR_TRAJ_ONLY:-yaml_default} active_uavs=${CASE_ACTIVE_UAVS:-default} exclude_uav0_from_simulation=${CASE_EXCLUDE_UAV0_FROM_SIMULATION}"

      roslaunch "${CASE_LAUNCH_FILE}" \
        obstacle_count:="${CASE_OBS}" \
        seed:="${CASE_SEED}" \
        r:="${CASE_R}" \
        v:="${CASE_V}" \
        w:="${CASE_W}" \
        metrics_csv:="${CASE_METRICS_CSV}" \
        run_tag:="${CASE_RUN_TAG}" \
        safety_variant:="${CASE_SAFETY_VARIANT}" \
        cbf_enabled:="${CASE_CBF_ENABLED}" \
        cbf_use_in_sim:="${CASE_CBF_USE_IN_SIM}" \
        cbf_slack_max:="${CASE_CBF_SLACK_MAX}" \
        cbf_r_slack:="${CASE_CBF_R_SLACK}" \
        kappa:="${CASE_KAPPA}" \
        ugv_rollout_mode:="${CASE_UGV_ROLLOUT_MODE}" \
        car_trajectory_mode:="${CASE_CAR_TRAJECTORY_MODE}" \
        ugv_longitudinal_accel:="${CASE_UGV_LONGITUDINAL_ACCEL}" \
        ugv_longitudinal_decel:="${CASE_UGV_LONGITUDINAL_DECEL}" \
        obstacles_for_car_traj_only:="${CASE_OBSTACLES_FOR_CAR_TRAJ_ONLY}" \
        active_uavs_csv:="${CASE_ACTIVE_UAVS}" \
        exclude_uav0_from_simulation:="${CASE_EXCLUDE_UAV0_FROM_SIMULATION}" \
        exclude_uav0_from_all_metrics:="${CASE_EXCLUDE_UAV0_FROM_ALL_METRICS}" \
        frame_mode:="${CASE_MODE}" \
        use_rviz:="false" &
      ROSLAUNCH_PID=$!
      wait "${ROSLAUNCH_PID}"
      if [[ -n "${RVIZ_PID}" ]]; then
        kill -INT "${RVIZ_PID}" >/dev/null 2>&1 || true
        sleep 0.2
        kill -TERM "${RVIZ_PID}" >/dev/null 2>&1 || true
        wait "${RVIZ_PID}" 2>/dev/null || true
      fi
    ' >"${session_log}" 2>&1 &

  local session_pid=$!
  SESSION_PGIDS+=("${session_pid}")
  LAST_SESSION_LOG="${session_log}"
  LAST_SESSION_METRICS_CSV="${metrics_csv}"
  LAST_SESSION_RUN_TAG="${run_tag}"
  LAST_SESSION_PID="${session_pid}"
  LAST_SESSION_ROS_LOG_DIR="${ros_log_dir}"
}

print_metrics_summary() {
  local metrics_csv="$1"
  local run_tag="$2"
  local label="$3"

  if [[ ! -f "${metrics_csv}" ]]; then
    echo "[WARN] ${label} metrics not found: ${metrics_csv}"
    return 0
  fi

  python3 - "${metrics_csv}" "${run_tag}" "${label}" <<'PY'
import csv
import pathlib
import sys

metrics_csv = pathlib.Path(sys.argv[1])
run_tag = sys.argv[2]
label = sys.argv[3]

with metrics_csv.open("r", encoding="utf-8", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        if row.get("run_tag") == run_tag and row.get("scope") == "all":
            solver_tracking_rms_avoid = row.get(
                "solver_tracking_rms_avoid",
                row.get("tracking_rms_cbf_active", "nan"),
            )
            solver_tracking_rms_avoid_samples = row.get(
                "solver_tracking_rms_avoid_samples", "0"
            )
            solver_min_h = row.get("solver_min_h", row.get("min_h", "nan"))
            solver_active_mean_h = row.get(
                "solver_active_mean_h", row.get("solver_mean_h", "nan")
            )
            solver_min_planar_clearance = row.get(
                "solver_min_planar_clearance", "nan"
            )
            witness_h = row.get("solver_min_h_witness_h", "nan")
            witness_planar_clearance = row.get(
                "solver_min_h_witness_planar_clearance", "nan"
            )
            witness_e_track = row.get(
                "solver_min_h_witness_tracking_error", "nan"
            )
            witness_delta = row.get("solver_min_h_witness_delta", "nan")
            witness_planar_distance = row.get(
                "solver_min_h_witness_planar_distance", "nan"
            )
            witness_solver_active_obstacle_key = row.get(
                "solver_min_h_witness_active_obstacle_key", "none"
            )
            print(
                f"[INFO] {label} metrics:"
                f" collision={row.get('collision', 'nan')}"
                f" solver_tracking_rms_avoid={solver_tracking_rms_avoid}"
                f" solver_tracking_rms_avoid_samples={solver_tracking_rms_avoid_samples}"
                f" solver_min_h={solver_min_h}"
                f" solver_active_mean_h={solver_active_mean_h}"
                f" solver_min_planar_clearance={solver_min_planar_clearance}"
                f" witness_h={witness_h}"
                f" witness_planar_clearance={witness_planar_clearance}"
                f" witness_e_track={witness_e_track}"
                f" witness_delta={witness_delta}"
                f" witness_planar_distance={witness_planar_distance}"
                f" witness_solver_active_obstacle_key={witness_solver_active_obstacle_key}"
                f" fail_rate={row.get('fail_rate', 'nan')}"
            )
            raise SystemExit(0)

print(f"[WARN] {label} metrics row not found in {metrics_csv}")
PY
}

wait_for_pid_status() {
  local pid="$1"
  local status=0
  if [[ -z "${pid}" ]]; then
    return 0
  fi
  set +e
  # When the script intentionally stops a background compare session, bash
  # would otherwise print "Killed"/"Terminated" for the waited job.
  wait "${pid}" 2>/dev/null
  status=$?
  set -e
  return "${status}"
}

record_seed_metrics() {
  local out_csv="$1"
  local metrics_csv="$2"
  local run_tag="$3"
  local seed="$4"
  local frame_mode="$5"
  local session_status="$6"

  python3 - "${out_csv}" "${metrics_csv}" "${run_tag}" "${seed}" "${frame_mode}" "${session_status}" <<'PY'
import csv
import pathlib
import sys

out_csv = pathlib.Path(sys.argv[1])
metrics_csv = pathlib.Path(sys.argv[2])
run_tag = sys.argv[3]
seed = sys.argv[4]
frame_mode = sys.argv[5]
session_status = sys.argv[6]

header = [
    "seed",
    "frame_mode",
    "run_tag",
    "session_status",
    "metrics_found",
    "collision",
    "solver_tracking_rms_avoid",
    "solver_tracking_rms_avoid_samples",
    "solver_min_h",
    "solver_active_mean_h",
    "solver_min_planar_clearance",
    "solver_min_h_witness_h",
    "solver_min_h_witness_planar_clearance",
    "solver_min_h_witness_tracking_error",
    "solver_min_h_witness_delta",
    "solver_min_h_witness_planar_distance",
    "solver_min_h_witness_active_obstacle_key",
    "min_h",
    "mean_h",
    "min_cbf",
    "fail_rate",
    "metrics_csv",
]

row_out = {
    "seed": seed,
    "frame_mode": frame_mode,
    "run_tag": run_tag,
    "session_status": session_status,
    "metrics_found": "0",
    "collision": "nan",
    "solver_tracking_rms_avoid": "nan",
    "solver_tracking_rms_avoid_samples": "0",
    "solver_min_h": "nan",
    "solver_active_mean_h": "nan",
    "solver_min_planar_clearance": "nan",
    "solver_min_h_witness_h": "nan",
    "solver_min_h_witness_planar_clearance": "nan",
    "solver_min_h_witness_tracking_error": "nan",
    "solver_min_h_witness_delta": "nan",
    "solver_min_h_witness_planar_distance": "nan",
    "solver_min_h_witness_active_obstacle_key": "none",
    "min_h": "nan",
    "mean_h": "nan",
    "min_cbf": "nan",
    "fail_rate": "nan",
    "metrics_csv": str(metrics_csv.resolve()),
}

if metrics_csv.is_file():
    with metrics_csv.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("run_tag") == run_tag and row.get("scope") == "all":
                row_out["metrics_found"] = "1"
                row_out["collision"] = row.get("collision", "nan")
                row_out["solver_tracking_rms_avoid"] = row.get(
                    "solver_tracking_rms_avoid",
                    row.get("tracking_rms_cbf_active", "nan"),
                )
                row_out["solver_tracking_rms_avoid_samples"] = row.get(
                    "solver_tracking_rms_avoid_samples", "0"
                )
                row_out["solver_min_h"] = row.get(
                    "solver_min_h", row.get("min_h", "nan")
                )
                row_out["solver_active_mean_h"] = row.get(
                    "solver_active_mean_h", row.get("solver_mean_h", "nan")
                )
                row_out["solver_min_planar_clearance"] = row.get(
                    "solver_min_planar_clearance", "nan"
                )
                row_out["solver_min_h_witness_h"] = row.get(
                    "solver_min_h_witness_h", "nan"
                )
                row_out["solver_min_h_witness_planar_clearance"] = row.get(
                    "solver_min_h_witness_planar_clearance", "nan"
                )
                row_out["solver_min_h_witness_tracking_error"] = row.get(
                    "solver_min_h_witness_tracking_error", "nan"
                )
                row_out["solver_min_h_witness_delta"] = row.get(
                    "solver_min_h_witness_delta", "nan"
                )
                row_out["solver_min_h_witness_planar_distance"] = row.get(
                    "solver_min_h_witness_planar_distance", "nan"
                )
                row_out["solver_min_h_witness_active_obstacle_key"] = row.get(
                    "solver_min_h_witness_active_obstacle_key", "none"
                )
                row_out["min_h"] = row.get("min_h", "nan")
                row_out["mean_h"] = row.get("mean_h", "nan")
                row_out["min_cbf"] = row.get("min_cbf", "nan")
                row_out["fail_rate"] = row.get("fail_rate", "nan")
                break

out_csv.parent.mkdir(parents=True, exist_ok=True)
need_header = not out_csv.exists()
with out_csv.open("a", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=header)
    if need_header:
      writer.writeheader()
    writer.writerow(row_out)

print(
    f"[INFO] recorded seed={seed} frame={frame_mode}"
    f" collision={row_out['collision']}"
    f" solver_tracking_rms_avoid={row_out['solver_tracking_rms_avoid']}"
    f" solver_tracking_rms_avoid_samples={row_out['solver_tracking_rms_avoid_samples']}"
    f" solver_min_h={row_out['solver_min_h']}"
    f" solver_active_mean_h={row_out['solver_active_mean_h']}"
    f" solver_min_planar_clearance={row_out['solver_min_planar_clearance']}"
    f" witness_h={row_out['solver_min_h_witness_h']}"
    f" witness_planar_clearance={row_out['solver_min_h_witness_planar_clearance']}"
    f" witness_e_track={row_out['solver_min_h_witness_tracking_error']}"
    f" witness_delta={row_out['solver_min_h_witness_delta']}"
    f" witness_planar_distance={row_out['solver_min_h_witness_planar_distance']}"
    f" witness_solver_active_obstacle_key={row_out['solver_min_h_witness_active_obstacle_key']}"
    f" fail_rate={row_out['fail_rate']}"
    f" metrics_found={row_out['metrics_found']}"
    f" status={session_status}"
)
PY
}

summarize_average_metrics() {
  local per_seed_csv="$1"
  local out_csv="$2"

  python3 - "${per_seed_csv}" "${out_csv}" <<'PY'
import csv
import math
import pathlib
import sys

per_seed_csv = pathlib.Path(sys.argv[1])
out_csv = pathlib.Path(sys.argv[2])

header = [
    "frame_mode",
    "total_runs",
    "metrics_found_runs",
    "avg_collision",
    "avg_solver_tracking_rms_avoid",
    "avg_solver_tracking_rms_avoid_samples",
    "avg_solver_min_h",
    "avg_solver_active_mean_h",
    "avg_solver_min_planar_clearance",
    "avg_min_h",
    "avg_mean_h",
    "avg_min_cbf",
    "avg_fail_rate",
]

def parse_float(value):
    try:
        result = float(value)
    except Exception:
        return math.nan
    return result if math.isfinite(result) else math.nan

def mean_of(entries, key):
    values = [parse_float(row.get(key, "nan")) for row in entries]
    values = [v for v in values if math.isfinite(v)]
    if not values:
        return math.nan
    return sum(values) / len(values)

def fmt(value):
    return "nan" if not math.isfinite(value) else f"{value:.6f}"

if not per_seed_csv.is_file():
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
    print(f"[WARN] per-seed metrics summary not found: {per_seed_csv}")
    raise SystemExit(0)

with per_seed_csv.open("r", encoding="utf-8", newline="") as f:
    rows = list(csv.DictReader(f))

groups = {}
for row in rows:
    groups.setdefault(row.get("frame_mode", ""), []).append(row)

ordered_modes = [mode for mode in ("noninertial", "inertial") if mode in groups]
for mode in sorted(groups):
    if mode and mode not in ordered_modes:
        ordered_modes.append(mode)

summary_rows = []
for mode in ordered_modes:
    entries = groups.get(mode, [])
    total_runs = len(entries)
    metrics_found_runs = sum(1 for row in entries if row.get("metrics_found") == "1")
    summary_rows.append(
        {
            "frame_mode": mode,
            "total_runs": str(total_runs),
            "metrics_found_runs": str(metrics_found_runs),
            "avg_collision": fmt(mean_of(entries, "collision")),
            "avg_solver_tracking_rms_avoid": fmt(
                mean_of(entries, "solver_tracking_rms_avoid")
            ),
            "avg_solver_tracking_rms_avoid_samples": fmt(
                mean_of(entries, "solver_tracking_rms_avoid_samples")
            ),
            "avg_solver_min_h": fmt(mean_of(entries, "solver_min_h")),
            "avg_solver_active_mean_h": fmt(mean_of(entries, "solver_active_mean_h")),
            "avg_solver_min_planar_clearance": fmt(
                mean_of(entries, "solver_min_planar_clearance")
            ),
            "avg_min_h": fmt(mean_of(entries, "min_h")),
            "avg_mean_h": fmt(mean_of(entries, "mean_h")),
            "avg_min_cbf": fmt(mean_of(entries, "min_cbf")),
            "avg_fail_rate": fmt(mean_of(entries, "fail_rate")),
        }
    )

if rows:
    summary_rows.append(
        {
            "frame_mode": "all_modes",
            "total_runs": str(len(rows)),
            "metrics_found_runs": str(sum(1 for row in rows if row.get("metrics_found") == "1")),
            "avg_collision": fmt(mean_of(rows, "collision")),
            "avg_solver_tracking_rms_avoid": fmt(
                mean_of(rows, "solver_tracking_rms_avoid")
            ),
            "avg_solver_tracking_rms_avoid_samples": fmt(
                mean_of(rows, "solver_tracking_rms_avoid_samples")
            ),
            "avg_solver_min_h": fmt(mean_of(rows, "solver_min_h")),
            "avg_solver_active_mean_h": fmt(mean_of(rows, "solver_active_mean_h")),
            "avg_solver_min_planar_clearance": fmt(
                mean_of(rows, "solver_min_planar_clearance")
            ),
            "avg_min_h": fmt(mean_of(rows, "min_h")),
            "avg_mean_h": fmt(mean_of(rows, "mean_h")),
            "avg_min_cbf": fmt(mean_of(rows, "min_cbf")),
            "avg_fail_rate": fmt(mean_of(rows, "fail_rate")),
        }
    )

out_csv.parent.mkdir(parents=True, exist_ok=True)
with out_csv.open("w", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=header)
    writer.writeheader()
    writer.writerows(summary_rows)

for row in summary_rows:
    print(
        f"[INFO] average summary {row['frame_mode']}:"
        f" avg_collision={row['avg_collision']}"
        f" avg_solver_tracking_rms_avoid={row['avg_solver_tracking_rms_avoid']}"
        f" avg_solver_tracking_rms_avoid_samples={row['avg_solver_tracking_rms_avoid_samples']}"
        f" avg_solver_min_h={row['avg_solver_min_h']}"
        f" avg_solver_active_mean_h={row['avg_solver_active_mean_h']}"
        f" avg_fail_rate={row['avg_fail_rate']}"
    )
PY
}

summarize_batch_results() {
  local per_seed_csv="$1"
  local out_csv="$2"

  python3 - "${per_seed_csv}" "${out_csv}" <<'PY'
import csv
import math
import pathlib
import sys

per_seed_csv = pathlib.Path(sys.argv[1])
out_csv = pathlib.Path(sys.argv[2])

header = [
    "frame_mode",
    "total_runs",
    "metrics_found_runs",
    "collision_count",
    "collision_rate",
]

if not per_seed_csv.is_file():
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
    print(f"[WARN] per-seed metrics summary not found: {per_seed_csv}")
    raise SystemExit(0)

with per_seed_csv.open("r", encoding="utf-8", newline="") as f:
    rows = list(csv.DictReader(f))

groups = {}
for row in rows:
    groups.setdefault(row.get("frame_mode", ""), []).append(row)

ordered_modes = [mode for mode in ("noninertial", "inertial") if mode in groups]
for mode in sorted(groups):
    if mode and mode not in ordered_modes:
        ordered_modes.append(mode)

summary_rows = []
for mode in ordered_modes:
    entries = groups.get(mode, [])
    total_runs = len(entries)
    metrics_found_runs = sum(1 for row in entries if row.get("metrics_found") == "1")
    collision_count = 0
    for row in entries:
        try:
            collision = float(row.get("collision", "nan"))
        except Exception:
            collision = math.nan
        if math.isfinite(collision) and int(collision) != 0:
            collision_count += 1
    collision_rate = (
        collision_count / total_runs if total_runs > 0 else math.nan
    )
    summary_rows.append(
        {
            "frame_mode": mode,
            "total_runs": str(total_runs),
            "metrics_found_runs": str(metrics_found_runs),
            "collision_count": str(collision_count),
            "collision_rate": "nan"
            if not math.isfinite(collision_rate)
            else f"{collision_rate:.6f}",
        }
    )

if rows:
    total_runs = len(rows)
    metrics_found_runs = sum(1 for row in rows if row.get("metrics_found") == "1")
    collision_count = 0
    for row in rows:
        try:
            collision = float(row.get("collision", "nan"))
        except Exception:
            collision = math.nan
        if math.isfinite(collision) and int(collision) != 0:
            collision_count += 1
    collision_rate = collision_count / total_runs
    summary_rows.append(
        {
            "frame_mode": "all_modes",
            "total_runs": str(total_runs),
            "metrics_found_runs": str(metrics_found_runs),
            "collision_count": str(collision_count),
            "collision_rate": f"{collision_rate:.6f}",
        }
    )

out_csv.parent.mkdir(parents=True, exist_ok=True)
with out_csv.open("w", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=header)
    writer.writeheader()
    writer.writerows(summary_rows)

for row in summary_rows:
    print(
        f"[INFO] collision summary {row['frame_mode']}:"
        f" collision_count={row['collision_count']}"
        f" total_runs={row['total_runs']}"
        f" collision_rate={row['collision_rate']}"
        f" metrics_found_runs={row['metrics_found_runs']}"
    )
PY
}

run_seed_compare() {
  local seed="$1"
  local run_dir="${RUN_ROOT}/seed_${seed}"
  local seed_failed=0
  local sessions_closed_by_script=0

  echo
  echo "[INFO] ===== seed=${seed} ====="

  launch_case noninertial "${NON_PORT}" "非惯性" "noninertial" "${seed}" "${run_dir}" "${UGV_ROLLOUT_MODE}"
  local non_log="${LAST_SESSION_LOG}"
  local non_metrics_csv="${LAST_SESSION_METRICS_CSV}"
  local non_run_tag="${LAST_SESSION_RUN_TAG}"
  local non_pid="${LAST_SESSION_PID}"
  local non_ros_log_dir="${LAST_SESSION_ROS_LOG_DIR}"
  local non_step_metrics_csv
  non_step_metrics_csv="$(derive_step_metrics_csv_path "${non_metrics_csv}")"
  local non_ugv_horizon_summary_csv
  non_ugv_horizon_summary_csv="$(derive_ugv_horizon_summary_csv_path "${non_metrics_csv}")"

  launch_case inertial "${IN_PORT}" "惯性" "inertial" "${seed}" "${run_dir}"
  local in_log="${LAST_SESSION_LOG}"
  local in_metrics_csv="${LAST_SESSION_METRICS_CSV}"
  local in_run_tag="${LAST_SESSION_RUN_TAG}"
  local in_pid="${LAST_SESSION_PID}"
  local in_ros_log_dir="${LAST_SESSION_ROS_LOG_DIR}"
  local in_step_metrics_csv
  in_step_metrics_csv="$(derive_step_metrics_csv_path "${in_metrics_csv}")"

  if ! wait_for_log_pattern "${non_log}" 'frame_mode_effective=noninertial' "${FRAME_LOG_WAIT_SEC}"; then
    echo "ERROR: seed=${seed} noninertial session did not report frame_mode_effective=noninertial" >&2
    echo "Check log: ${non_log}" >&2
    seed_failed=1
  fi

  if ! wait_for_log_pattern "${in_log}" 'frame_mode_effective=inertial' "${FRAME_LOG_WAIT_SEC}"; then
    echo "ERROR: seed=${seed} inertial session did not report frame_mode_effective=inertial" >&2
    echo "Check log: ${in_log}" >&2
    seed_failed=1
  fi

  if [[ "${USE_RVIZ}" == "true" ]] && command -v xprop >/dev/null 2>&1; then
    tag_rviz_window_for_log_dir "${non_ros_log_dir}" "[NONINERTIAL][seed=${seed}] relative view" "[非惯性][seed=${seed}] 相对视图" "${RVIZ_APPEAR_WAIT_SEC}" &
    TAGGER_PIDS+=("$!")
    tag_rviz_window_for_log_dir "${in_ros_log_dir}" "[INERTIAL][seed=${seed}] world view" "[惯性][seed=${seed}] 世界视图" "${RVIZ_APPEAR_WAIT_SEC}" &
    TAGGER_PIDS+=("$!")
  fi

  echo "[INFO] seed=${seed} noninertial log: ${non_log}"
  echo "[INFO] seed=${seed} noninertial metrics: ${non_metrics_csv}"
  echo "[INFO] seed=${seed} inertial log: ${in_log}"
  echo "[INFO] seed=${seed} inertial metrics: ${in_metrics_csv}"

  if wait_for_metrics_row "${non_metrics_csv}" "${non_run_tag}" "${METRICS_WAIT_SEC}" "${non_pid}"; then
    print_metrics_summary "${non_metrics_csv}" "${non_run_tag}" "非惯性 seed=${seed}"
  else
    echo "[WARN] seed=${seed} noninertial metrics row was not written within ${METRICS_WAIT_SEC}s: ${non_metrics_csv}"
    seed_failed=1
  fi

  if wait_for_metrics_row "${in_metrics_csv}" "${in_run_tag}" "${METRICS_WAIT_SEC}" "${in_pid}"; then
    print_metrics_summary "${in_metrics_csv}" "${in_run_tag}" "惯性 seed=${seed}"
  else
    echo "[WARN] seed=${seed} inertial metrics row was not written within ${METRICS_WAIT_SEC}s: ${in_metrics_csv}"
    seed_failed=1
  fi

  if should_auto_close_sessions_after_metrics; then
    echo "[INFO] seed=${seed} metrics collected; closing RViz sessions and continuing."
    cleanup_sessions
    sessions_closed_by_script=1
  fi

  local non_status="0"
  local in_status="0"
  if wait_for_pid_status "${non_pid}"; then
    non_status="0"
  else
    non_status=$?
    if ((sessions_closed_by_script)); then
      echo "[INFO] seed=${seed} noninertial session stopped by script after metrics (status ${non_status})."
      non_status="closed_after_metrics"
    else
      echo "[WARN] seed=${seed} noninertial session exited with status ${non_status}" >&2
      seed_failed=1
    fi
  fi

  if wait_for_pid_status "${in_pid}"; then
    in_status="0"
  else
    in_status=$?
    if ((sessions_closed_by_script)); then
      echo "[INFO] seed=${seed} inertial session stopped by script after metrics (status ${in_status})."
      in_status="closed_after_metrics"
    else
      echo "[WARN] seed=${seed} inertial session exited with status ${in_status}" >&2
      seed_failed=1
    fi
  fi

  record_seed_metrics \
    "${BATCH_SEED_METRICS_CSV}" \
    "${non_metrics_csv}" \
    "${non_run_tag}" \
    "${seed}" \
    "noninertial" \
    "${non_status}"
  record_seed_metrics \
    "${BATCH_SEED_METRICS_CSV}" \
    "${in_metrics_csv}" \
    "${in_run_tag}" \
    "${seed}" \
    "inertial" \
    "${in_status}"

  plot_seed_compare_curves \
    "${seed}" \
    "${non_step_metrics_csv}" \
    "${non_run_tag}" \
    "${in_step_metrics_csv}" \
    "${in_run_tag}" \
    "${run_dir}"

  if ((sessions_closed_by_script == 0)); then
    cleanup_sessions
  fi

  if [[ "${PLOT_UGV_HORIZON_CURVES}" == "true" ]]; then
    local aux_rollout_mode
    aux_rollout_mode="$(opposite_rollout_mode "${UGV_ROLLOUT_MODE}")"
    local aux_prefix="noninertial_${aux_rollout_mode}"

    launch_case \
      noninertial \
      "${NON_PORT}" \
      "非惯性-${aux_rollout_mode}" \
      "${aux_prefix}" \
      "${seed}" \
      "${run_dir}" \
      "${aux_rollout_mode}" \
      "false"
    local aux_log="${LAST_SESSION_LOG}"
    local aux_metrics_csv="${LAST_SESSION_METRICS_CSV}"
    local aux_run_tag="${LAST_SESSION_RUN_TAG}"
    local aux_pid="${LAST_SESSION_PID}"
    local aux_ugv_horizon_summary_csv
    aux_ugv_horizon_summary_csv="$(derive_ugv_horizon_summary_csv_path "${aux_metrics_csv}")"

    if ! wait_for_log_pattern "${aux_log}" 'frame_mode_effective=noninertial' "${FRAME_LOG_WAIT_SEC}"; then
      echo "ERROR: seed=${seed} ${aux_rollout_mode} noninertial session did not report frame_mode_effective=noninertial" >&2
      echo "Check log: ${aux_log}" >&2
      seed_failed=1
    fi

    echo "[INFO] seed=${seed} ${aux_rollout_mode} noninertial log: ${aux_log}"
    echo "[INFO] seed=${seed} ${aux_rollout_mode} noninertial metrics: ${aux_metrics_csv}"

    if wait_for_metrics_row "${aux_metrics_csv}" "${aux_run_tag}" "${METRICS_WAIT_SEC}" "${aux_pid}"; then
      print_metrics_summary "${aux_metrics_csv}" "${aux_run_tag}" "非惯性 ${aux_rollout_mode} seed=${seed}"
    else
      echo "[WARN] seed=${seed} ${aux_rollout_mode} noninertial metrics row was not written within ${METRICS_WAIT_SEC}s: ${aux_metrics_csv}"
      seed_failed=1
    fi

    local aux_status="0"
    if wait_for_pid_status "${aux_pid}"; then
      aux_status="0"
    else
      aux_status=$?
      echo "[WARN] seed=${seed} ${aux_rollout_mode} noninertial session exited with status ${aux_status}" >&2
      seed_failed=1
    fi
    cleanup_sessions

    local stage_summary_csv
    local stage_run_tag
    local frozen_summary_csv
    local frozen_run_tag
    if [[ "${UGV_ROLLOUT_MODE}" == "stage" ]]; then
      stage_summary_csv="${non_ugv_horizon_summary_csv}"
      stage_run_tag="${non_run_tag}"
      frozen_summary_csv="${aux_ugv_horizon_summary_csv}"
      frozen_run_tag="${aux_run_tag}"
    else
      stage_summary_csv="${aux_ugv_horizon_summary_csv}"
      stage_run_tag="${aux_run_tag}"
      frozen_summary_csv="${non_ugv_horizon_summary_csv}"
      frozen_run_tag="${non_run_tag}"
    fi

    plot_seed_ugv_horizon_curves \
      "${seed}" \
      "${stage_summary_csv}" \
      "${stage_run_tag}" \
      "${frozen_summary_csv}" \
      "${frozen_run_tag}" \
      "${run_dir}"
  fi
  return "${seed_failed}"
}

require_cmd setsid
require_cmd roscore
require_cmd rosparam
require_cmd roslaunch
require_cmd rosnode
require_cmd python3
require_file "${LAUNCH_FILE}"
if [[ "${PLOT_COMPARE_CURVES}" == "true" ]]; then
  require_file "${PLOT_STEP_SCRIPT}"
fi
if [[ "${PLOT_UGV_HORIZON_CURVES}" == "true" ]]; then
  require_file "${PLOT_UGV_HORIZON_SCRIPT}"
  require_file "${PLOT_SEED_STAGEWISE_ALIGNMENT_SCRIPT}"
fi
require_file "${PLOT_STAGEWISE_ROBUSTNESS_SCRIPT}"
require_file "${PLOT_SEED_WITNESS_SCRIPT}"

if [[ "${USE_RVIZ}" == "true" ]]; then
  require_cmd rviz
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "ERROR: USE_RVIZ=true but DISPLAY is empty." >&2
    exit 2
  fi
fi

build_seed_list
mkdir -p "${RUN_ROOT}"
BATCH_SEED_METRICS_CSV="${RUN_ROOT}/seed_metrics_summary.csv"
BATCH_COLLISION_RATE_CSV="${RUN_ROOT}/collision_rate_summary.csv"
BATCH_AVERAGE_METRICS_CSV="${RUN_ROOT}/average_metrics_summary.csv"
BATCH_SEED_WITNESS_PNG="${RUN_ROOT}/seed_metrics_summary_both_collision_vs_witness.png"
COMBINED_ROBUSTNESS_CSV="${RUN_ROOT}/combined_robustness_data.csv"
THREE_AXIS_PLOT_PNG="${RUN_ROOT}/three_axis_alignment_robustness.png"
trap cleanup EXIT INT TERM

echo "[INFO] workspace: ${WORKSPACE}"
echo "[INFO] run root: ${RUN_ROOT}"
echo "[INFO] settings: r=${R} v=${V} w=${W} obs=${OBSTACLE_COUNT} seeds=$(seed_list_csv) kappa=${KAPPA} safety_variant=${SAFETY_VARIANT} cbf_enabled=${CBF_ENABLED:-auto} cbf_use_in_sim=${CBF_USE_IN_SIM:-auto} ugv_rollout_mode=${UGV_ROLLOUT_MODE} car_trajectory_mode=${CAR_TRAJECTORY_MODE:-yaml_default} ugv_longitudinal_accel=${UGV_LONGITUDINAL_ACCEL} ugv_longitudinal_decel=${UGV_LONGITUDINAL_DECEL} obstacles_for_car_traj_only=${OBSTACLES_FOR_CAR_TRAJ_ONLY:-yaml_default} active_uavs=${ACTIVE_UAVS:-default} use_rviz=${USE_RVIZ}"
echo "[INFO] simulation excludes UAV0 by default: ${EXCLUDE_UAV0_FROM_SIMULATION}"
echo "[INFO] plotting: enabled=${PLOT_COMPARE_CURVES} uav=${PLOT_UAV_IDX} distance_field=${PLOT_DISTANCE_FIELD}"
echo "[INFO] aggregate metrics exclude UAV0: ${EXCLUDE_UAV0_FROM_ALL_METRICS}"
echo "[INFO] UGV horizon plotting: enabled=${PLOT_UGV_HORIZON_CURVES}"
if ((${#SEED_LIST[@]} > 1)) && [[ "${USE_RVIZ}" == "true" ]]; then
  echo "[INFO] multi-seed mode with RViz enabled will open/close RViz for each seed."
fi

batch_failed=0
for seed in "${SEED_LIST[@]}"; do
  if ! run_seed_compare "${seed}"; then
    batch_failed=1
  fi
done

summarize_batch_results "${BATCH_SEED_METRICS_CSV}" "${BATCH_COLLISION_RATE_CSV}"
summarize_average_metrics "${BATCH_SEED_METRICS_CSV}" "${BATCH_AVERAGE_METRICS_CSV}"
python3 "${PLOT_SEED_WITNESS_SCRIPT}" \
  "${RUN_ROOT}" \
  --frame-mode both \
  --out "${BATCH_SEED_WITNESS_PNG}"

echo "[INFO] Aggregating data for Stage-wise vs Frozen Robustness Analysis..."
python3 "${SCRIPT_DIR}/aggregate_robustness_data.py" "${RUN_ROOT}" || \
  echo "[WARN] Failed to aggregate robustness data."

if [[ -f "${COMBINED_ROBUSTNESS_CSV}" ]]; then
  echo "[INFO] Plotting Three-Axis Alignment Plot..."
  python3 "${PLOT_STAGEWISE_ROBUSTNESS_SCRIPT}" \
    --csv "${COMBINED_ROBUSTNESS_CSV}" \
    --out-plot "${THREE_AXIS_PLOT_PNG}" \
    --accel-thr "${ROBUSTNESS_ACCEL_THR}" || echo "[WARN] Failed to plot robustness figures."
fi

echo
echo "[INFO] per-seed metrics summary: ${BATCH_SEED_METRICS_CSV}"
echo "[INFO] collision-rate summary: ${BATCH_COLLISION_RATE_CSV}"
echo "[INFO] average-metrics summary: ${BATCH_AVERAGE_METRICS_CSV}"
echo "[INFO] seed-witness plot: ${BATCH_SEED_WITNESS_PNG}"
echo "[INFO] three-axis plot: ${THREE_AXIS_PLOT_PNG}"

exit "${batch_failed}"
