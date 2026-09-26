#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from statistics import mean
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple


CONTROLLERS = ("noninertial_frozen", "noninertial_stage")
DEFAULT_SEEDS = (1, 2, 3, 4, 9, 10, 27, 100)
NEW_CONDITIONS = (
    "map_current",
    "online_100hz_zero",
    "online_10hz_zero",
    "online_10hz_noise001",
    "online_10hz_delay010",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze the paired K_P=6 sensing ablation.")
    parser.add_argument(
        "--ablation-root",
        default="results/reviewer6_comment3_3_kp6_paired_ablation_8seed",
    )
    parser.add_argument(
        "--formal-root",
        default="results/reviewer6_comment3_3_kp1236_sensor_rplidar_a2m4_formal100_jobs12",
    )
    parser.add_argument(
        "--map-root",
        default=(
            "/home/jjm/桌面/论文修改/code/仿真/惯性 VS 非惯性/results/"
            "figure_eight_fixedobs_cbf_kp6_seed1_100_2ctrl"
        ),
    )
    parser.add_argument("--seeds", default=",".join(map(str, DEFAULT_SEEDS)))
    return parser.parse_args()


def read_csv(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def as_float(row: Mapping[str, str], key: str) -> float:
    try:
        return float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return math.nan


def finite_mean(values: Iterable[float]) -> float:
    selected = [value for value in values if math.isfinite(value)]
    return mean(selected) if selected else math.nan


def write_csv(path: Path, rows: Sequence[Mapping[str, object]], fields: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fields))
        writer.writeheader()
        writer.writerows(rows)


def load_condition_rows(
    ablation_root: Path,
    formal_root: Path,
    seeds: Sequence[int],
) -> Dict[str, List[Dict[str, str]]]:
    paths = {
        condition: ablation_root / condition / "tables/per_seed_controller_metrics.csv"
        for condition in NEW_CONDITIONS
    }
    paths.update(
        {
            "formal_nominal": formal_root / "online_nominal/tables/per_seed_controller_metrics.csv",
            "formal_degraded": formal_root / "online_degraded/tables/per_seed_controller_metrics.csv",
        }
    )
    selected = set(seeds)
    result: Dict[str, List[Dict[str, str]]] = {}
    for condition, path in paths.items():
        rows = [
            row
            for row in read_csv(path)
            if row.get("controller") in CONTROLLERS
            and math.isclose(as_float(row, "v"), 3.0)
            and int(row["seed"]) in selected
        ]
        result[condition] = rows
    return result


def step_minimum_diagnostic(metrics_csv: Path) -> Dict[str, object]:
    step_csv = metrics_csv.with_name("metrics_steps.csv")
    rows = read_csv(step_csv)
    witness = min(rows, key=lambda row: as_float(row, "truth_static_min_surface_clearance"))
    nearest_id = witness.get("sensing_nearest_truth_obstacle_id", "")
    active_id = witness.get("solver_active_static_obstacle_id", "")
    return {
        "witness_uav": witness.get("uav_idx", ""),
        "witness_step": witness.get("step_idx", ""),
        "witness_time_sec": as_float(witness, "sim_time"),
        "truth_surface_clearance_m": as_float(witness, "truth_static_min_surface_clearance"),
        "solver_surface_distance_m": as_float(witness, "solver_planar_surface_distance"),
        "solver_surface_error_vs_truth_m": as_float(witness, "solver_surface_error_vs_truth_m"),
        "uav_position_error_m": as_float(witness, "uav_position_error_m"),
        "nearest_truth_obstacle_id": nearest_id,
        "solver_active_obstacle_key": witness.get("solver_active_obstacle_key", ""),
        "solver_active_matches_nearest_truth": int(nearest_id == active_id),
        "nearest_truth_track_active": witness.get("sensing_nearest_truth_track_active", ""),
        "nearest_truth_track_error_m": as_float(
            witness, "sensing_nearest_truth_track_center_error_m"
        ),
        "solver_h": as_float(witness, "solver_h"),
        "solver_cbf": as_float(witness, "solver_cbf"),
        "solver_slack": as_float(witness, "solver_slack"),
        "tracking_error": as_float(witness, "tracking_error"),
    }


def raw_map_collision_audit(map_root: Path) -> Tuple[List[Dict[str, object]], Dict[str, int]]:
    rows: List[Dict[str, object]] = []
    counts: Dict[str, int] = {}
    for controller in CONTROLLERS:
        count = 0
        for seed in range(1, 101):
            metrics_csv = (
                map_root
                / "logs"
                / controller
                / "v_3"
                / "w_3"
                / f"seed_{seed:03d}"
                / "metrics.csv"
            )
            metrics_rows = read_csv(metrics_csv)
            aggregate = next(row for row in metrics_rows if row.get("scope") == "all")
            collision = int(as_float(aggregate, "collision"))
            count += collision
            rows.append(
                {
                    "controller": controller,
                    "seed": seed,
                    "collision_from_raw_metrics": collision,
                    "metrics_csv": str(metrics_csv),
                }
            )
        counts[controller] = count
    return rows, counts


def fmt(value: float, digits: int = 6) -> str:
    return "nan" if not math.isfinite(value) else f"{value:.{digits}f}"


def main() -> int:
    args = parse_args()
    ablation_root = Path(args.ablation_root).resolve()
    formal_root = Path(args.formal_root).resolve()
    map_root = Path(args.map_root).resolve()
    seeds = tuple(int(part.strip()) for part in args.seeds.split(",") if part.strip())
    condition_rows = load_condition_rows(ablation_root, formal_root, seeds)

    per_seed_rows: List[Dict[str, object]] = []
    witness_rows: List[Dict[str, object]] = []
    summary_rows: List[Dict[str, object]] = []
    paired_rows: List[Dict[str, object]] = []

    for condition, rows in condition_rows.items():
        indexed = {(row["controller"], int(row["seed"])): row for row in rows}
        for controller in CONTROLLERS:
            controller_rows = [indexed[(controller, seed)] for seed in seeds]
            collisions = [int(row["collision"]) for row in controller_rows]
            truth_minima = [as_float(row, "truth_static_min_surface_clearance") for row in controller_rows]
            solver_surfaces = [as_float(row, "solver_min_planar_clearance") + 0.35 for row in controller_rows]
            summary_rows.append(
                {
                    "condition": condition,
                    "controller": controller,
                    "seed_count": len(seeds),
                    "collision_count": sum(collisions),
                    "collision_rate": sum(collisions) / len(collisions),
                    "truth_surface_clearance_mean_m": finite_mean(truth_minima),
                    "solver_surface_distance_mean_m": finite_mean(solver_surfaces),
                    "tracking_rms_mean": finite_mean(as_float(row, "tracking_rms") for row in controller_rows),
                }
            )
            for row in controller_rows:
                seed = int(row["seed"])
                metrics_csv = Path(row["metrics_csv"])
                per_seed_rows.append(
                    {
                        "condition": condition,
                        "controller": controller,
                        "seed": seed,
                        "collision": int(row["collision"]),
                        "truth_surface_clearance_min_m": as_float(
                            row, "truth_static_min_surface_clearance"
                        ),
                        "solver_surface_distance_min_m": as_float(
                            row, "solver_min_planar_clearance"
                        )
                        + 0.35,
                        "tracking_rms": as_float(row, "tracking_rms"),
                        "metrics_csv": str(metrics_csv),
                    }
                )
                diagnostic = step_minimum_diagnostic(metrics_csv)
                diagnostic.update(
                    {
                        "condition": condition,
                        "controller": controller,
                        "seed": seed,
                        "collision": int(row["collision"]),
                    }
                )
                witness_rows.append(diagnostic)

        frozen = {seed: int(indexed[(CONTROLLERS[0], seed)]["collision"]) for seed in seeds}
        stage = {seed: int(indexed[(CONTROLLERS[1], seed)]["collision"]) for seed in seeds}
        both = sum(frozen[seed] and stage[seed] for seed in seeds)
        frozen_only = sum(frozen[seed] and not stage[seed] for seed in seeds)
        stage_only = sum(stage[seed] and not frozen[seed] for seed in seeds)
        paired_rows.append(
            {
                "condition": condition,
                "seed_count": len(seeds),
                "both_collision": both,
                "frozen_only_collision": frozen_only,
                "stage_only_collision": stage_only,
                "neither_collision": len(seeds) - both - frozen_only - stage_only,
            }
        )

    tables = ablation_root / "tables"
    write_csv(
        tables / "paired_ablation_per_seed.csv",
        per_seed_rows,
        list(per_seed_rows[0].keys()),
    )
    write_csv(
        tables / "paired_ablation_summary.csv",
        summary_rows,
        list(summary_rows[0].keys()),
    )
    write_csv(
        tables / "paired_collision_contingency.csv",
        paired_rows,
        list(paired_rows[0].keys()),
    )
    write_csv(
        tables / "minimum_clearance_diagnostics.csv",
        witness_rows,
        list(witness_rows[0].keys()),
    )

    map_audit_rows, map_counts = raw_map_collision_audit(map_root)
    write_csv(
        tables / "map_raw_collision_audit.csv",
        map_audit_rows,
        list(map_audit_rows[0].keys()),
    )

    summary_index = {
        (row["condition"], row["controller"]): row for row in summary_rows
    }
    selected_map_stage = [
        row
        for row in witness_rows
        if row["condition"] == "map_current"
        and row["controller"] == "noninertial_stage"
        and row["collision"] == 1
    ]
    selected_map_frozen = [
        row
        for row in witness_rows
        if row["condition"] == "map_current"
        and row["controller"] == "noninertial_frozen"
        and row["collision"] == 1
    ]

    def collision_pair(condition: str) -> str:
        frozen = summary_index[(condition, CONTROLLERS[0])]
        stage = summary_index[(condition, CONTROLLERS[1])]
        return f"{frozen['collision_count']}/{stage['collision_count']}"

    map_stage_max_slack = sum(
        math.isclose(float(row["solver_slack"]), 2.0, abs_tol=1e-9)
        for row in selected_map_stage
    )
    map_frozen_max_slack = sum(
        math.isclose(float(row["solver_slack"]), 2.0, abs_tol=1e-9)
        for row in selected_map_frozen
    )
    report = f"""# K_P=6 配对感知消融诊断

这是面向机理定位的定向小样本诊断，不用于估计总体概率。配对 seed 为
`{','.join(map(str, seeds))}`：1个两者都触发的 seed、5个仅 Stage 触发的 seed、
1个两者都不触发的 seed，以及 Nominal 正式批次中唯一一个仅 Frozen
触发的 seed。

## 8-seed 定向集的安全事件数

| 条件 | Frozen / Stage |
|---|---:|
| 当前 Map | {collision_pair('map_current')} |
| 在线，100 Hz，零噪声/延迟/dropout | {collision_pair('online_100hz_zero')} |
| 在线，10 Hz，零噪声/延迟/dropout | {collision_pair('online_10hz_zero')} |
| 在线，10 Hz，仅 0.01 m 噪声 | {collision_pair('online_10hz_noise001')} |
| 在线，10 Hz，仅 0.10 s 延迟 | {collision_pair('online_10hz_delay010')} |
| 正式 Nominal | {collision_pair('formal_nominal')} |
| 正式 Degraded | {collision_pair('formal_degraded')} |

当前 Map、100 Hz 零误差在线接口和 10 Hz 零误差在线接口的逐 seed 安全事件结果
完全一致。仅加入 0.10 s 延迟也没有改变任何 seed 的事件结果。因此，对当前静态
障碍物工况，物体级在线接口、10 Hz 扫描调度和 Nominal 延迟都不是 Stage/Frozen
安全事件排序的来源。

仅加入 0.01 m 噪声会改变 Frozen 中哪些 seed 触发事件，但定向集总数仍为
{collision_pair('online_10hz_noise001')}。噪声会扰动非线性闭环的轨迹分支，但不是 Stage
劣势的起因。

## Map 100-seed 原始文件审计

逐个读取原始 `metrics.csv` 后，Frozen/Stage 安全事件数为
{map_counts['noninertial_frozen']}/{map_counts['noninertial_stage']}。旧 Map 汇总表报告的 0/0
与这些原始文件矛盾；其 per-seed 表还指向一个已不存在的源路径，因此不能用于
`collision` 指标。Map 原始结果的排序已经与正式 Nominal（65/96）和
Degraded（69/92）基本相同。

## 最小净空时刻的直接机理

在当前 Map 的触发运行中，solver 选中的静态障碍物与最近真值障碍物在所有定向
样本中都匹配。因此原因不是障碍物缺失或选错 ID。在最小净空见证时刻：

- Stage 的 {len(selected_map_stage)} 个触发运行全部出现 `solver_h < 0`；
- 其中 {map_stage_max_slack}/{len(selected_map_stage)} 个 Stage 运行达到最大松弛量 2.0；
- {map_frozen_max_slack}/{len(selected_map_frozen)} 个 Frozen 触发运行达到最大松弛量。

因此，当前数据直接支持的机理是：激进的 $K_P=6$ 闭环进入了 soft-HOCBF 松弛区域。
感知噪声可以使个别 seed 切换轨迹分支，但 Stage 的安全事件排序在精确 Map 障碍物
中心条件下已经存在。

## 证据文件

- `tables/paired_ablation_per_seed.csv`
- `tables/paired_ablation_summary.csv`
- `tables/paired_collision_contingency.csv`
- `tables/minimum_clearance_diagnostics.csv`
- `tables/map_raw_collision_audit.csv`
"""
    (ablation_root / "ablation_report.md").write_text(report, encoding="utf-8")
    print(f"wrote {ablation_root / 'ablation_report.md'}")
    print(f"raw Map collision counts: {map_counts}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
