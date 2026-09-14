#!/usr/bin/env bash
set -u

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "${workspace_root}/devel/setup.bash"
config_file="${workspace_root}/src/coni_mpc/parameters/lv_dot_box_collection.yaml"
generator="${workspace_root}/src/coni_mpc/scripts/generate_box_calibration_map.py"
coverage_check="${workspace_root}/src/coni_mpc/scripts/check_gt_coverage.py"
preflight_root="$(mktemp -d /tmp/lv_dot_box_preflight.XXXXXX)"
preflight_map="${preflight_root}/static_box_map.pcd"
python3 "${generator}" --config "${config_file}" --output "${preflight_map}"

run_static() {
  local output_root="$1" duration="$2" map_file="$3" raw_file="$4" log_file="$5"
  roslaunch coni_mpc marsim_lv_dot_uav1.launch \
    box_collection:=true box_config:="${config_file}" map_name:="${map_file}" \
    start_validator:=false start_collector:=true collection_scenario:=static_box \
    raw_filename:="${raw_file}" expected_gt_count:=1 static_box_gt:=true \
    deterministic_box_mode:=false dynamic_obstacles:=false motion:=parallel \
    duration_sec:="${duration}" warmup_sec:=3.0 collection_root:="${output_root}" \
    >"${log_file}" 2>&1
}

run_dynamic() {
  local output_root="$1" duration="$2" raw_file="$3" log_file="$4"
  roslaunch coni_mpc marsim_lv_dot_uav1.launch \
    box_collection:=true box_config:="${config_file}" \
    map_name:="${workspace_root}/src/coni_mpc/test_data/lv_dot_validation_bounds.pcd" \
    start_validator:=false start_collector:=true collection_scenario:=dynamic_box \
    raw_filename:="${raw_file}" expected_gt_count:=1 static_box_gt:=false \
    deterministic_box_mode:=true dynamic_obstacles:=true dynamic_obstacle_count:=1 \
    motion:=parallel duration_sec:="${duration}" warmup_sec:=3.0 \
    collection_root:="${output_root}" >"${log_file}" 2>&1
}

# GT continuity preflight. Formal result directory is not created unless both pass.
run_static "${preflight_root}/static" 3.0 "${preflight_map}" preflight_static.csv \
  "${preflight_root}/static.log" || exit 20
python3 "${coverage_check}" "${preflight_root}/static/experiment_config.json" static_box || exit 21
run_dynamic "${preflight_root}/dynamic" 3.0 preflight_dynamic.csv \
  "${preflight_root}/dynamic.log" || exit 22
python3 "${coverage_check}" "${preflight_root}/dynamic/experiment_config.json" dynamic_box || exit 23

timestamp="$(date +%Y%m%d_%H%M%S)"
result_root="${workspace_root}/results/lv_dot_box_collection_${timestamp}"
mkdir -p "${result_root}"
static_map="${result_root}/static_box_map.pcd"
python3 "${generator}" --config "${config_file}" --output "${static_map}"

static_command="roslaunch coni_mpc marsim_lv_dot_uav1.launch box_collection:=true map_name:=${static_map} start_validator:=false start_collector:=true collection_scenario:=static_box raw_filename:=moving_uav_static_box_raw.csv expected_gt_count:=1 static_box_gt:=true dynamic_obstacles:=false motion:=parallel duration_sec:=20.0 warmup_sec:=3.0 collection_root:=${result_root}"
dynamic_command="roslaunch coni_mpc marsim_lv_dot_uav1.launch box_collection:=true start_validator:=false start_collector:=true collection_scenario:=dynamic_box raw_filename:=moving_uav_dynamic_box_raw.csv expected_gt_count:=1 deterministic_box_mode:=true dynamic_obstacles:=true dynamic_obstacle_count:=1 motion:=parallel duration_sec:=20.0 warmup_sec:=3.0 collection_root:=${result_root}"
printf '[static_box] %s\n[dynamic_box] %s\n' "${static_command}" "${dynamic_command}" \
  >"${result_root}/launch_commands.txt"

run_static "${result_root}" 20.0 "${static_map}" moving_uav_static_box_raw.csv \
  "${result_root}/static_box_roslaunch.log"
static_exit=$?
run_dynamic "${result_root}" 20.0 moving_uav_dynamic_box_raw.csv \
  "${result_root}/dynamic_box_roslaunch.log"
dynamic_exit=$?

python3 "${workspace_root}/src/coni_mpc/scripts/finalize_lv_dot_box_collection.py" \
  "${result_root}" "${static_exit}" "${dynamic_exit}"
printf '%s\n' "${result_root}"
exit $(( static_exit != 0 || dynamic_exit != 0 ))
