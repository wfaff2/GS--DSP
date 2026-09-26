#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
LAUNCH_COMPARE_SCRIPT="${SCRIPT_DIR}/launch_compare_rviz.sh"
PLOT_ROLLOUT_COMPARE_SCRIPT="${SCRIPT_DIR}/plot_rollout_tracking_error_compare.py"
PLOT_UGV_HORIZON_COMPARE_SCRIPT="${SCRIPT_DIR}/plot_rollout_ugv_horizon_error_compare.py"

# Examples:
#   USE_RVIZ=false SEED_START=1 SEED_END=50 ./scripts/launch_rollout_compare.sh
#   USE_RVIZ=false SEED_START=3 SEED_END=3 FRAME_MODES=inertial ./scripts/launch_rollout_compare.sh
#   USE_RVIZ=false SEEDS=1,3,5 COMPARE_UAV_IDX=all ./scripts/launch_rollout_compare.sh
R="${R:-1.0}"
V="${V:-2.0}"
W="${W:-0.5}"
OBSTACLE_COUNT="${OBSTACLE_COUNT:-80}"
SEED="${SEED:-3}"
SEEDS="${SEEDS:-}"
SEED_START="${SEED_START:-}"
SEED_END="${SEED_END:-}"
KAPPA="${KAPPA:-1.0}"
ACTIVE_UAVS="${ACTIVE_UAVS:-}"
EXCLUDE_UAV0_FROM_SIMULATION="${EXCLUDE_UAV0_FROM_SIMULATION:-true}"
USE_RVIZ="${USE_RVIZ:-false}"
METRICS_WAIT_SEC="${METRICS_WAIT_SEC:-900}"
FRAME_LOG_WAIT_SEC="${FRAME_LOG_WAIT_SEC:-40}"
RVIZ_APPEAR_WAIT_SEC="${RVIZ_APPEAR_WAIT_SEC:-8}"
SESSION_STOP_GRACE_SEC="${SESSION_STOP_GRACE_SEC:-0.5}"
PLOT_COMPARE_CURVES="${PLOT_COMPARE_CURVES:-true}"
PLOT_DISTANCE_FIELD="${PLOT_DISTANCE_FIELD:-solver_planar_clearance}"
PLOT_UAV_IDX="${PLOT_UAV_IDX:-0}"
PLOT_TRACKING_ERROR_COMPARE="${PLOT_TRACKING_ERROR_COMPARE:-true}"
PLOT_UGV_HORIZON_COMPARE="${PLOT_UGV_HORIZON_COMPARE:-false}"

COMPARE_UAV_IDX="${COMPARE_UAV_IDX:-all}"
FRAME_MODES="${FRAME_MODES:-inertial,noninertial}"
MASTER_STAMP="$(date +%Y%m%d-%H%M%S)"
COMPARE_ROOT="${COMPARE_ROOT:-${WORKSPACE}/results/rollout_compare_${MASTER_STAMP}}"
STAGE_RUN_ROOT="${STAGE_RUN_ROOT:-${COMPARE_ROOT}/stage}"
FROZEN_RUN_ROOT="${FROZEN_RUN_ROOT:-${COMPARE_ROOT}/frozen}"
COMPARE_PLOT_ROOT="${COMPARE_PLOT_ROOT:-${COMPARE_ROOT}/comparisons}"

declare -a SEED_LIST=()
declare -a FRAME_MODE_LIST=()

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "ERROR: missing file: ${path}" >&2
    exit 2
  fi
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

parse_frame_modes() {
  local raw="${FRAME_MODES//,/ }"
  local token
  FRAME_MODE_LIST=()
  for token in ${raw}; do
    case "${token}" in
      inertial|in)
        FRAME_MODE_LIST+=("inertial")
        ;;
      noninertial|non)
        FRAME_MODE_LIST+=("noninertial")
        ;;
      *)
        echo "ERROR: unsupported FRAME_MODES token: ${token}" >&2
        exit 2
        ;;
    esac
  done
  if ((${#FRAME_MODE_LIST[@]} == 0)); then
    echo "ERROR: no frame modes resolved." >&2
    exit 2
  fi
}

seed_list_csv() {
  local IFS=","
  echo "${SEED_LIST[*]}"
}

frame_mode_list_csv() {
  local IFS=","
  echo "${FRAME_MODE_LIST[*]}"
}

run_rollout_batch() {
  local rollout_mode="$1"
  local run_root="$2"

  echo
  echo "[INFO] starting ${rollout_mode} batch -> ${run_root}"
  mkdir -p "${run_root}"

  env \
    R="${R}" \
    V="${V}" \
    W="${W}" \
    OBSTACLE_COUNT="${OBSTACLE_COUNT}" \
    SEED="${SEED}" \
    SEEDS="${SEEDS}" \
    SEED_START="${SEED_START}" \
    SEED_END="${SEED_END}" \
    KAPPA="${KAPPA}" \
    ACTIVE_UAVS="${ACTIVE_UAVS}" \
    EXCLUDE_UAV0_FROM_SIMULATION="${EXCLUDE_UAV0_FROM_SIMULATION}" \
    UGV_ROLLOUT_MODE="${rollout_mode}" \
    USE_RVIZ="${USE_RVIZ}" \
    METRICS_WAIT_SEC="${METRICS_WAIT_SEC}" \
    FRAME_LOG_WAIT_SEC="${FRAME_LOG_WAIT_SEC}" \
    RVIZ_APPEAR_WAIT_SEC="${RVIZ_APPEAR_WAIT_SEC}" \
    SESSION_STOP_GRACE_SEC="${SESSION_STOP_GRACE_SEC}" \
    PLOT_COMPARE_CURVES="${PLOT_COMPARE_CURVES}" \
    PLOT_DISTANCE_FIELD="${PLOT_DISTANCE_FIELD}" \
    PLOT_UAV_IDX="${PLOT_UAV_IDX}" \
    RUN_ROOT="${run_root}" \
    "${LAUNCH_COMPARE_SCRIPT}"
}

plot_all_tracking_error_compares() {
  local seed frame_mode out_png

  if [[ "${PLOT_TRACKING_ERROR_COMPARE}" != "true" ]]; then
    return
  fi

  mkdir -p "${COMPARE_PLOT_ROOT}"
  for seed in "${SEED_LIST[@]}"; do
    for frame_mode in "${FRAME_MODE_LIST[@]}"; do
      out_png="${COMPARE_PLOT_ROOT}/seed_${seed}_${frame_mode}_stage_vs_frozen_tracking_error_uav_${COMPARE_UAV_IDX}.png"
      echo "[INFO] plotting seed=${seed} frame_mode=${frame_mode} -> ${out_png}"
      python3 "${PLOT_ROLLOUT_COMPARE_SCRIPT}" \
        --stage-run "${STAGE_RUN_ROOT}" \
        --frozen-run "${FROZEN_RUN_ROOT}" \
        --seed "${seed}" \
        --frame-mode "${frame_mode}" \
        --uav "${COMPARE_UAV_IDX}" \
        --out "${out_png}"
    done
  done
}

plot_all_ugv_horizon_compares() {
  local seed frame_mode out_png

  if [[ "${PLOT_UGV_HORIZON_COMPARE}" != "true" ]]; then
    return
  fi

  mkdir -p "${COMPARE_PLOT_ROOT}"
  for seed in "${SEED_LIST[@]}"; do
    for frame_mode in "${FRAME_MODE_LIST[@]}"; do
      out_png="${COMPARE_PLOT_ROOT}/seed_${seed}_${frame_mode}_stage_vs_frozen_ugv_horizon_rms.png"
      echo "[INFO] plotting UGV horizon compare seed=${seed} frame_mode=${frame_mode} -> ${out_png}"
      python3 "${PLOT_UGV_HORIZON_COMPARE_SCRIPT}" \
        --stage-run "${STAGE_RUN_ROOT}" \
        --frozen-run "${FROZEN_RUN_ROOT}" \
        --seed "${seed}" \
        --frame-mode "${frame_mode}" \
        --out "${out_png}"
    done
  done
}

require_file "${LAUNCH_COMPARE_SCRIPT}"
require_file "${PLOT_ROLLOUT_COMPARE_SCRIPT}"
require_file "${PLOT_UGV_HORIZON_COMPARE_SCRIPT}"

build_seed_list
parse_frame_modes

mkdir -p "${COMPARE_ROOT}"

echo "[INFO] workspace: ${WORKSPACE}"
echo "[INFO] compare root: ${COMPARE_ROOT}"
echo "[INFO] stage run root: ${STAGE_RUN_ROOT}"
echo "[INFO] frozen run root: ${FROZEN_RUN_ROOT}"
echo "[INFO] comparison plots: ${COMPARE_PLOT_ROOT}"
echo "[INFO] settings: r=${R} v=${V} w=${W} obs=${OBSTACLE_COUNT} seeds=$(seed_list_csv) kappa=${KAPPA} active_uavs=${ACTIVE_UAVS:-default} use_rviz=${USE_RVIZ}"
echo "[INFO] simulation excludes UAV0 by default: ${EXCLUDE_UAV0_FROM_SIMULATION}"
echo "[INFO] compare settings: frame_modes=$(frame_mode_list_csv) compare_uav=${COMPARE_UAV_IDX}"

run_rollout_batch "stage" "${STAGE_RUN_ROOT}"
run_rollout_batch "frozen" "${FROZEN_RUN_ROOT}"
plot_all_tracking_error_compares
plot_all_ugv_horizon_compares

echo
echo "[INFO] one-click rollout comparison completed."
echo "[INFO] stage batch: ${STAGE_RUN_ROOT}"
echo "[INFO] frozen batch: ${FROZEN_RUN_ROOT}"
echo "[INFO] comparison plots: ${COMPARE_PLOT_ROOT}"
