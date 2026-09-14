#!/usr/bin/env bash
set -u

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "${workspace_root}/devel/setup.bash"
timestamp="$(date +%Y%m%d_%H%M%S)"
result_root="${workspace_root}/results/visible_gt_geometry_${timestamp}"
mkdir -p "${result_root}"
config_file="${workspace_root}/src/coni_mpc/parameters/lv_dot_box_collection.yaml"
map_file="${result_root}/static_box_map.pcd"
python3 "${workspace_root}/src/coni_mpc/scripts/generate_box_calibration_map.py" \
  --config "${config_file}" --output "${map_file}"

command="roslaunch coni_mpc marsim_lv_dot_uav1.launch box_collection:=true box_config:=${config_file} map_name:=${map_file} motion:=parallel duration_sec:=20.0 warmup_sec:=3.0 dynamic_obstacles:=false static_box_gt:=true visible_gt_debug:=true start_validator:=false start_collector:=false start_visible_gt_collector:=true visible_gt_target_frames:=200 visible_gt_result_root:=${result_root}"
printf '%s\n' "${command}" >"${result_root}/launch_command.txt"

roslaunch coni_mpc marsim_lv_dot_uav1.launch \
  box_collection:=true box_config:="${config_file}" map_name:="${map_file}" \
  motion:=parallel duration_sec:=20.0 warmup_sec:=3.0 \
  dynamic_obstacles:=false static_box_gt:=true visible_gt_debug:=true \
  start_validator:=false start_collector:=false start_visible_gt_collector:=true \
  visible_gt_target_frames:=200 \
  visible_gt_result_root:="${result_root}" \
  >"${result_root}/roslaunch.log" 2>&1
exit_code=$?

python3 - "${result_root}" "${exit_code}" <<'PY'
import json
import os
import sys
root, exit_code = sys.argv[1], int(sys.argv[2])
path = os.path.join(root, "visible_gt_collection_config.json")
with open(path) as handle:
    config = json.load(handle)
config["roslaunch_exit_code"] = exit_code
config["launch_command_file"] = "launch_command.txt"
config["raw_file"] = "visible_gt_vs_lvdot_box_raw.csv"
temporary = path + ".tmp"
with open(temporary, "w") as handle:
    json.dump(config, handle, indent=2, sort_keys=True)
    handle.write("\n")
os.replace(temporary, path)
PY

printf '%s\n' "${result_root}"
exit "${exit_code}"
