#!/usr/bin/env bash
# figure-8 纯跟踪批量实验 (无障碍物)
#
# 场景：UGV 走 figure-8 轨迹，无随机障碍物，CBF 不激活
# 目的：对比 stage vs frozen 在 omega 持续翻转下的运动学跟踪误差
#
# 用法: bash scripts/run_figure8_tracking_batch.sh [--jobs N] [extra args...]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ -f devel/setup.bash ]]; then
  # shellcheck disable=SC1091
  source devel/setup.bash
fi

exec python3 scripts/run_vw_three_controller_20seed.py \
  --jobs "${JOBS:-20}" \
  --car-trajectory-mode figure_eight \
  --obs 0 \
  --vary-noise-seed \
  --seed-start 1 \
  --seed-end 30 \
  --v-values 1.5,2.0,2.5,3.0 \
  --w-values 1.5,2.0,2.5,3.0 \
  --scan-mode diagonal \
  --controllers inertial_frozen,noninertial_frozen,noninertial_stage \
  --active-uavs 1,2,3 \
  "$@"
