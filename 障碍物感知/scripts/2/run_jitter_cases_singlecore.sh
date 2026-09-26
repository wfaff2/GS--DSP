#!/bin/bash
# 单核重跑 OS 调度抖动导致的不一致 cases
# V/W/seed 组合来自修复前后碰撞结果的差异分析

set -e
cd "$(dirname "$0")/.."
source devel/setup.bash

RUN_ROOT="results/jitter_singlecore_$(date +%Y%m%d-%H%M%S)"
SCRIPT="scripts/run_vw_three_controller_20seed.py"

echo "Run root: $RUN_ROOT"
echo "Running 14 (V,W,seed) groups × 3 controllers = 42 cases, --jobs 1"
echo ""

run_seed() {
    local V=$1 W=$2 SEED=$3
    echo ">>> V=$V W=$W seed=$SEED"
    python3 $SCRIPT \
        --v-values "$V" \
        --w-values "$W" \
        --seed-start $SEED --seed-end $SEED \
        --obs 100 \
        --jobs 1 \
        --no-aggregate \
        --run-root "$RUN_ROOT"
}

# Group A: V=2.5, W=2.5
run_seed 2.5 2.5 4
run_seed 2.5 2.5 17
run_seed 2.5 2.5 19

# Group B: V=2.5, W=3.0
run_seed 2.5 3.0 7
run_seed 2.5 3.0 9
run_seed 2.5 3.0 12
run_seed 2.5 3.0 16
run_seed 2.5 3.0 17
run_seed 2.5 3.0 19

# Group C: V=3.0, W=2.5
run_seed 3.0 2.5 7
run_seed 3.0 2.5 9
run_seed 3.0 2.5 10

# Group D: V=3.0, W=3.0
run_seed 3.0 3.0 4
run_seed 3.0 3.0 10

# Final aggregation
echo ""
echo "=== 最终聚合 ==="
python3 $SCRIPT \
    --v-values "2.5,3.0" \
    --w-values "2.5,3.0" \
    --seed-start 1 --seed-end 20 \
    --obs 100 \
    --jobs 1 \
    --dry-run \
    --run-root "$RUN_ROOT" || true

python3 scripts/aggregate_vw_three_controller_metrics.py "$RUN_ROOT" 2>/dev/null || \
python3 -c "
import subprocess, sys
r = subprocess.run(['python3','scripts/aggregate_suite.py','$RUN_ROOT'], capture_output=True, text=True)
print(r.stdout[-2000:] if r.stdout else r.stderr[-1000:])
" || echo "[aggregation skipped]"

echo ""
echo "Done. Results in: $RUN_ROOT"
