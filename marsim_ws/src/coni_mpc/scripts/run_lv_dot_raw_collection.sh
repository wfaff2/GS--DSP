#!/usr/bin/env bash
set -u

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "${workspace_root}/devel/setup.bash"

timestamp="$(date +%Y%m%d_%H%M%S)"
result_root="${workspace_root}/results/lv_dot_raw_collection_${timestamp}"
mkdir -p "${result_root}"

static_display="roslaunch coni_mpc marsim_lv_dot_uav1.launch start_validator:=false start_collector:=true collection_scenario:=static motion:=stationary duration_sec:=20.0 warmup_sec:=3.0 dynamic_obstacles:=false collection_root:=${result_root}"
dynamic_map="${workspace_root}/src/coni_mpc/test_data/lv_dot_validation_bounds.pcd"
dynamic_display="roslaunch coni_mpc marsim_lv_dot_uav1.launch start_validator:=false start_collector:=true collection_scenario:=dynamic map_name:=${dynamic_map} motion:=stationary duration_sec:=22.0 warmup_sec:=3.0 dynamic_obstacles:=true dynamic_obstacle_count:=20 dynamic_obstacle_size:=0.8 dynamic_obstacle_velocity:=0.8 collection_root:=${result_root}"
printf '[static] %s\n[dynamic] %s\n' "${static_display}" "${dynamic_display}" \
  >"${result_root}/launch_commands.txt"

roslaunch coni_mpc marsim_lv_dot_uav1.launch \
  start_validator:=false start_collector:=true collection_scenario:=static \
  motion:=stationary duration_sec:=20.0 warmup_sec:=3.0 dynamic_obstacles:=false \
  collection_root:="${result_root}" \
  >"${result_root}/static_roslaunch.log" 2>&1
static_exit=$?

roslaunch coni_mpc marsim_lv_dot_uav1.launch \
  start_validator:=false start_collector:=true collection_scenario:=dynamic \
  map_name:="${dynamic_map}" motion:=stationary duration_sec:=22.0 warmup_sec:=3.0 \
  dynamic_obstacles:=true dynamic_obstacle_count:=20 \
  dynamic_obstacle_size:=0.8 dynamic_obstacle_velocity:=0.8 \
  collection_root:="${result_root}" \
  >"${result_root}/dynamic_roslaunch.log" 2>&1
dynamic_exit=$?

python3 "${workspace_root}/src/coni_mpc/scripts/finalize_lv_dot_raw_collection.py" \
  "${result_root}" "${static_exit}" "${dynamic_exit}"

printf '%s\n' "${result_root}"
exit $(( static_exit != 0 || dynamic_exit != 0 ))
