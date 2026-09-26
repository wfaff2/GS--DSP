#!/usr/bin/env bash
# figure-8 + 障碍物 CBF 鲁棒性批量实验
#
# 场景：UGV 走 figure-8 轨迹，静态障碍物集中在圆弧区域内
# 目的：对比 noninertial_stage vs noninertial_frozen vs inertial_frozen 的 CBF 安全性
#
# 实验设计：跨随机环境的安全性（distributional claim）
#   每个 seed → 不同障碍物布局，但同一 seed 下三个控制器面对相同布局（公平对比）
#   --vary-noise-seed   : 障碍物 seed 和噪声 seed 均随 case.seed 变化
#   --seed-end 50       : 50 个独立随机场景取平均，排除 cherry-pick 嫌疑
#
# obs-half-range 说明：
#   V=2,W=1 → R=2m，图8跨度 ±2m(x) ±4m(y)；±3.5/±4.5 覆盖圆弧区域
#   V=3,W=1 → R=3m，图8跨度 ±3m(x) ±6m(y)；±4.5/±6.5 更合适
#   此处保守取 ±4.5/±5.0，障碍物落在圆弧内侧
#
# 用法: bash scripts/run_figure8_cbf_batch.sh [--jobs N] [extra args...]
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
  --sim-duration-sec 35 \
  --obs 8 \
  --obs-half-range 4.5 \
  --obs-y-half-range 5.0 \
  --vary-noise-seed \
  --seed-start 1 \
  --seed-end 50 \
  --v-values 2.0,2.5,3.0 \
  --w-values 2.0,2.5,3.0 \
  --scan-mode diagonal \
  --controllers inertial_frozen,noninertial_frozen,noninertial_stage \
  --active-uavs 1,2,3 \
  "$@"
