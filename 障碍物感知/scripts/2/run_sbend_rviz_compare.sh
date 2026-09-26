#!/bin/bash
# 5-seed S弯场景 NIMPCB vs NI-Frozen RViz 对比
# 用法: bash run_sbend_rviz_compare.sh [v] [w] [seeds]
# 示例: bash run_sbend_rviz_compare.sh 3.0 1.5 "1 2 3 4 5"

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS="$(dirname "$SCRIPT_DIR")"

source "$WS/devel/setup.bash"

V="${1:-3.0}"
W="${2:-1.5}"
SEEDS="${3:-1 2 3 4 5}"
OBS=0   # S弯：无随机障碍物，UGV走干净S形

CONTROLLERS=("noninertial_stage" "noninertial_frozen")
ROLLOUTS=("stage" "frozen")

echo "=========================================="
echo " S弯对比: v=$V  w=$W  obs=$OBS"
echo " seeds: $SEEDS"
echo "=========================================="

for SEED in $SEEDS; do
    for i in 0 1; do
        CTRL="${CONTROLLERS[$i]}"
        ROLLOUT="${ROLLOUTS[$i]}"

        echo ""
        echo "------------------------------------------"
        echo " Controller : $CTRL"
        echo " Seed       : $SEED"
        echo " v=$V  w=$W  obs=$OBS"
        echo " 关闭 RViz 窗口或按 Ctrl+C 进入下一组"
        echo "------------------------------------------"
        read -p " 按 Enter 开始..."

        roslaunch coni_mpc num_sim_non_one_point.launch \
            v:="$V" \
            w:="$W" \
            seed:="$SEED" \
            obstacle_count:="$OBS" \
            ugv_rollout_mode:="$ROLLOUT" \
            use_noninertial_frame:=true \
            use_inertial_frame:=false \
            use_rviz:=true \
            rviz_view:=world \
            || true
    done
done

echo ""
echo "=========================================="
echo " 全部完成"
echo "=========================================="
