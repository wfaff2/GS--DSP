#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${WORKSPACE}"
source "${WORKSPACE}/devel/setup.bash"

RUN_STAMP="$(date +%Y%m%d-%H%M%S)"

# Override any of these from the terminal, e.g.
#   V_VALUES=1,1.25,1.5,1.75,2 W_VALUES=1,1.25,1.5,1.75,2 bash scripts/run_fig8_seed1_vw5x5_three_controller.sh
RUN_ROOT="${RUN_ROOT:-${WORKSPACE}/results/fig8_fixed_geom_seed1_diag_3ctrl_${RUN_STAMP}}"
V_VALUES="${V_VALUES:-3.1,3.2,3.3,3.4,3.5}"
W_VALUES="${W_VALUES:-3.1,3.2,3.3,3.4,3.5}"
OBS="${OBS:-0}"
JOBS="${JOBS:-8}"
R="${R:-1.0}"
KAPPA="${KAPPA:-1.0}"
ACTIVE_UAVS="${ACTIVE_UAVS:-1,2,3}"
TIMEOUT_SEC="${TIMEOUT_SEC:-900}"
UGV_ACCEL="${UGV_ACCEL:-8.0}"
UGV_DECEL="${UGV_DECEL:-8.0}"
SAFETY_VARIANT="${SAFETY_VARIANT:-A2_soft_cbf}"
CBF_SLACK_MAX="${CBF_SLACK_MAX:-}"

# RViz compare mode opens the three controllers at the same time for one V/W.
# Example:
#   RVIZ_COMPARE=true RVIZ_V=1 RVIZ_W=2 bash scripts/run_fig8_seed1_vw5x5_three_controller.sh
RVIZ_COMPARE="${RVIZ_COMPARE:-false}"
RVIZ_V="${RVIZ_V:-1}"
RVIZ_W="${RVIZ_W:-2}"
RVIZ_VIEW="${RVIZ_VIEW:-world}"
RVIZ_PORT_BASE="${RVIZ_PORT_BASE:-11431}"

launch_rviz_case() {
  local controller="$1"
  local frame_mode="$2"
  local rollout_mode="$3"
  local use_inertial="$4"
  local use_noninertial="$5"
  local port="$6"
  local run_dir="${RUN_ROOT}/rviz_${controller}"
  local log_path="${run_dir}/${controller}.log"
  local ros_home="${run_dir}/ros_home"
  local ros_log_dir="${run_dir}/ros_logs"

  mkdir -p "${run_dir}" "${ros_home}" "${ros_log_dir}"

  echo "[INFO] Launching ${controller}: V=${RVIZ_V} W=${RVIZ_W} ROS_MASTER_URI=http://127.0.0.1:${port}" >&2
  env \
    CASE_WORKSPACE="${WORKSPACE}" \
    CASE_PORT="${port}" \
    CASE_CONTROLLER="${controller}" \
    CASE_FRAME_MODE="${frame_mode}" \
    CASE_ROLLOUT_MODE="${rollout_mode}" \
    CASE_USE_INERTIAL="${use_inertial}" \
    CASE_USE_NONINERTIAL="${use_noninertial}" \
    CASE_ROS_HOME="${ros_home}" \
    CASE_ROS_LOG_DIR="${ros_log_dir}" \
    CASE_V="${RVIZ_V}" \
    CASE_W="${RVIZ_W}" \
    CASE_SEED="1" \
    CASE_OBS="${OBS}" \
    CASE_R="${R}" \
    CASE_KAPPA="${KAPPA}" \
    CASE_ACTIVE_UAVS="${ACTIVE_UAVS}" \
    CASE_UGV_ACCEL="${UGV_ACCEL}" \
    CASE_UGV_DECEL="${UGV_DECEL}" \
    CASE_RVIZ_VIEW="${RVIZ_VIEW}" \
    setsid bash -lc '
      set -euo pipefail
      cd "${CASE_WORKSPACE}"
      source "${CASE_WORKSPACE}/devel/setup.bash"
      export ROS_MASTER_URI="http://127.0.0.1:${CASE_PORT}"
      export ROS_IP="127.0.0.1"
      export ROS_HOSTNAME="127.0.0.1"
      export ROS_HOME="${CASE_ROS_HOME}"
      export ROS_LOG_DIR="${CASE_ROS_LOG_DIR}"

      roslaunch coni_mpc num_sim_non_one_point.launch \
        v:="${CASE_V}" \
        w:="${CASE_W}" \
        seed:="${CASE_SEED}" \
        obstacle_count:="${CASE_OBS}" \
        r:="${CASE_R}" \
        kappa:="${CASE_KAPPA}" \
        active_uavs_csv:="${CASE_ACTIVE_UAVS}" \
        car_trajectory_mode:=figure_eight \
        ugv_rollout_mode:="${CASE_ROLLOUT_MODE}" \
        frame_mode:="${CASE_FRAME_MODE}" \
        use_inertial_frame:="${CASE_USE_INERTIAL}" \
        use_noninertial_frame:="${CASE_USE_NONINERTIAL}" \
        ugv_longitudinal_accel:="${CASE_UGV_ACCEL}" \
        ugv_longitudinal_decel:="${CASE_UGV_DECEL}" \
        use_rviz:=true \
        rviz_view:="${CASE_RVIZ_VIEW}"
    ' >"${log_path}" 2>&1 &

  echo "$!"
}

if [[ "${RVIZ_COMPARE}" == "true" ]]; then
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "ERROR: RVIZ_COMPARE=true but DISPLAY is empty." >&2
    exit 2
  fi
  echo "[INFO] RViz compare mode: opening three controllers simultaneously"
  echo "[INFO] run_root=${RUN_ROOT}"
  echo "[INFO] V=${RVIZ_V} W=${RVIZ_W} seed=1 obs=${OBS} rviz_view=${RVIZ_VIEW}"

  pids=()
  pids+=("$(launch_rviz_case inertial_frozen inertial frozen true false "${RVIZ_PORT_BASE}")")
  pids+=("$(launch_rviz_case noninertial_frozen noninertial frozen false true "$((RVIZ_PORT_BASE + 1))")")
  pids+=("$(launch_rviz_case noninertial_stage noninertial stage false true "$((RVIZ_PORT_BASE + 2))")")

  cleanup_rviz_compare() {
    local pid
    for pid in "${pids[@]:-}"; do
      if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
        kill -INT "-${pid}" >/dev/null 2>&1 || kill -INT "${pid}" >/dev/null 2>&1 || true
      fi
    done
  }
  trap cleanup_rviz_compare INT TERM

  echo "[INFO] Three RViz sessions are running. Close RViz windows or press Ctrl+C to stop all."
  wait
  exit 0
fi

echo "[INFO] Running fixed-geometry figure-8 seed1 diagonal three-controller experiment"
echo "[INFO] workspace=${WORKSPACE}"
echo "[INFO] run_root=${RUN_ROOT}"
echo "[INFO] V_VALUES=${V_VALUES}"
echo "[INFO] W_VALUES=${W_VALUES}"
echo "[INFO] OBS=${OBS} JOBS=${JOBS} ACTIVE_UAVS=${ACTIVE_UAVS}"
echo "[INFO] SAFETY_VARIANT=${SAFETY_VARIANT} CBF_SLACK_MAX=${CBF_SLACK_MAX:-auto}"

extra_args=()
if [[ -n "${CBF_SLACK_MAX}" ]]; then
  extra_args+=(--cbf-slack-max "${CBF_SLACK_MAX}")
fi

python3 scripts/run_vw_three_controller_20seed.py \
  --run-root "${RUN_ROOT}" \
  --seed-start 1 \
  --seed-end 1 \
  --v-values "${V_VALUES}" \
  --w-values "${W_VALUES}" \
  --scan-mode diagonal \
  --controllers inertial_frozen,noninertial_frozen,noninertial_stage \
  --car-trajectory-mode figure_eight \
  --safety-variant "${SAFETY_VARIANT}" \
  --obs "${OBS}" \
  --jobs "${JOBS}" \
  --active-uavs "${ACTIVE_UAVS}" \
  --r "${R}" \
  --kappa "${KAPPA}" \
  --timeout-sec "${TIMEOUT_SEC}" \
  --ugv-longitudinal-accel "${UGV_ACCEL}" \
  --ugv-longitudinal-decel "${UGV_DECEL}" \
  --clean \
  "${extra_args[@]}"

echo "[INFO] Done."
echo "[INFO] Summary: ${RUN_ROOT}/tables/summary_by_vw_controller.csv"
