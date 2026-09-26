#!/bin/bash
# 扫描不同 obs 密度下三控制器碰撞率差异
# obs=30,50,70，每个20-seed，找最能体现stage优势的甜蜜点

set -e
cd "$(dirname "$0")/.."
source devel/setup.bash

SCRIPT="scripts/run_vw_three_controller_20seed.py"
RUN_ROOT="results/obs_scan_$(date +%Y%m%d-%H%M%S)"
JOBS=20

echo "Run root: $RUN_ROOT"
echo "Scanning obs=30,50,70 × V={2.5,3.0} × W={2.5,3.0} × 20 seeds × 3 controllers = 720 cases"
echo ""

for OBS in 30 50 70; do
    echo "============================================"
    echo ">>> obs=$OBS"
    echo "============================================"
    python3 $SCRIPT \
        --v-values "2.5,3.0" \
        --w-values "2.5,3.0" \
        --seed-start 1 --seed-end 20 \
        --obs $OBS \
        --jobs $JOBS \
        --run-root "$RUN_ROOT"
    echo ""
done

echo "=== 聚合完成，汇总各 obs 碰撞率 ==="
python3 - << PYEOF
import pandas as pd, pathlib, re

root = pathlib.Path("$RUN_ROOT")
rows = []
for p in sorted(root.glob("*/tables/per_seed_controller_metrics.csv")):
    df = pd.read_csv(p)
    obs_val = int(re.search(r'obs(\d+)', str(p)).group(1)) if re.search(r'obs(\d+)', str(p)) else -1
    # Get obs from data
    if 'obs' in df.columns:
        obs_val = df['obs'].iloc[0]
    df['obs_dir'] = str(p.parent.parent.name)
    rows.append(df)

if not rows:
    print("No data found in", root)
    exit()

df_all = pd.concat(rows, ignore_index=True)
grp = df_all.groupby(['obs','v','w','controller']).agg(
    total=('seed','count'), collisions=('collision','sum')
).reset_index()
grp['rate'] = (grp['collisions'] / grp['total'] * 100).round(1)

ctrl_label = {'inertial_frozen':'i-fz', 'noninertial_frozen':'ni-fz', 'noninertial_stage':'stage'}

print("\n碰撞率扫描结果 (%):")
print("="*70)
for obs in sorted(grp['obs'].unique()):
    print(f"\n--- obs={int(obs)} ---")
    sub = grp[grp['obs']==obs]
    pivot = sub.pivot_table(index=['v','w'], columns='controller', values='rate')
    pivot.columns = [ctrl_label.get(c,c) for c in pivot.columns]
    # Add difference columns
    if 'ni-fz' in pivot.columns and 'stage' in pivot.columns:
        pivot['stage-ni差'] = (pivot['ni-fz'] - pivot['stage']).round(1)
    if 'i-fz' in pivot.columns and 'stage' in pivot.columns:
        pivot['stage-ifz差'] = (pivot['i-fz'] - pivot['stage']).round(1)
    print(pivot.to_string())
PYEOF

echo ""
echo "Done. Results in: $RUN_ROOT"
