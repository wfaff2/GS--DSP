#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 /path/to/marsim_ws" >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_root="$(cd "${script_dir}/.." && pwd)"
workspace_root="$(cd "${package_root}/../.." && pwd)"
result_root="${workspace_root}/results/depth_cbf_m25_mid360_min01"
marsim_workspace="$1"
scene_root="${package_root}/test_data/depth_cbf_m25"
official_map="${marsim_workspace}/src/MARSIM/map_generator/resource/small_forest01cutoff.pcd"

source /opt/ros/noetic/setup.bash
source "${marsim_workspace}/devel/setup.bash" --extend
source "${workspace_root}/devel/setup.bash" --extend

mkdir -p "${result_root}/logs" "${result_root}/frames"
printf 'run_id,scene,motion,min_raylength,map,status\n' > "${result_root}/run_manifest.csv"

run_one() {
  local scene="$1" motion="$2" min_ray="$3" map="$4" run_id="$5"
  local log="${result_root}/logs/${run_id}.log"
  echo "[M2.5] ${run_id}"
  set +e
  timeout 18s roslaunch coni_mpc depth_cbf_m25.launch \
    "scene:=${scene}" "motion:=${motion}" "run_id:=${run_id}" \
    "map_name:=${map}" "gt_pcd:=${map}" \
    "result_root:=${result_root}" "duration_sec:=5.0" "warmup_sec:=2.0" \
    "min_raylength:=${min_ray}" >"${log}" 2>&1
  local status=$?
  set -e
  if [[ ${status} -eq 0 ]] && [[ -s "${result_root}/frames/${run_id}.csv" ]]; then
    printf '%s,%s,%s,%s,%s,PASS\n' "${run_id}" "${scene}" "${motion}" \
      "${min_ray}" "${map}" >> "${result_root}/run_manifest.csv"
  else
    printf '%s,%s,%s,%s,%s,FAIL_%s\n' "${run_id}" "${scene}" "${motion}" \
      "${min_ray}" "${map}" "${status}" >> "${result_root}/run_manifest.csv"
    echo "Run failed: ${run_id}; see ${log}" >&2
    return 1
  fi
}

for scene in A1_plane A2_cylinder A3_corner A4_switch; do
  for motion in front parallel oblique switching; do
    run_one "${scene}" "${motion}" 0.1 "${scene_root}/${scene}.pcd" \
      "${scene}_${motion}"
  done
done
# With Mid-360 this yaw sweep checks omnidirectional azimuth coverage; it is
# no longer an Avia front-FOV boundary test.
run_one A2_cylinder fov 0.1 "${scene_root}/A2_cylinder.pcd" A2_cylinder_azimuth360
run_one A1_plane blind 0.1 "${scene_root}/A1_plane.pcd" A1_plane_blind
run_one official official 0.1 "${official_map}" official_small_forest

echo "M2.5 runs complete: ${result_root}"
