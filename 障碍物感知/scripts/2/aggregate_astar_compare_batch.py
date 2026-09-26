#!/usr/bin/env python3
"""
Aggregate a batch of launch_astar_compare_kp_rviz.sh runs.

Expected layout:
  <batch_root>/kp{kp}_seed{seed}/tracking_rms_summary.csv

For each kp, this script computes:
  - mean non/inertial tracking RMS values across seeds
  - delta from those means
  - percentage improvement from those means

This avoids mixing two different aggregation rules such as:
  mean(non - in)  vs  mean((in - non) / in)
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
from collections import defaultdict


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Aggregate paired astar compare runs by kp."
    )
    parser.add_argument(
        "batch_root",
        help="Directory containing kp*_seed*/tracking_rms_summary.csv",
    )
    parser.add_argument(
        "--out",
        default=None,
        help="Output CSV path. Default: <batch_root>/averages.csv",
    )
    return parser.parse_args()


def safe_float(text: object) -> float:
    try:
        return float(text)
    except Exception:
        return math.nan


def mean(values: list[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return math.nan
    return sum(finite) / len(finite)


def fmt(value: float) -> str:
    return "nan" if not math.isfinite(value) else f"{value:.6f}"


def pct_from_means(non_mean: float, in_mean: float) -> float:
    if not math.isfinite(non_mean) or not math.isfinite(in_mean):
        return math.nan
    if abs(in_mean) <= 1e-12:
        return math.nan
    return (in_mean - non_mean) / in_mean * 100.0


def main() -> int:
    args = parse_args()
    batch_root = pathlib.Path(args.batch_root).resolve()
    out_path = (
        pathlib.Path(args.out).resolve()
        if args.out
        else batch_root / "averages.csv"
    )

    grouped: dict[str, dict[str, list[float]]] = defaultdict(
        lambda: defaultdict(list)
    )

    for summary_csv in sorted(batch_root.glob("kp*_seed*/tracking_rms_summary.csv")):
        with summary_csv.open("r", encoding="utf-8", newline="") as f:
            row = next(csv.DictReader(f))
        kp = str(row.get("kp", "")).strip()
        if not kp:
            continue
        grouped[kp]["noninertial_tracking_rms"].append(
            safe_float(row.get("noninertial_tracking_rms"))
        )
        grouped[kp]["inertial_tracking_rms"].append(
            safe_float(row.get("inertial_tracking_rms"))
        )
        grouped[kp]["noninertial_tracking_rms_high"].append(
            safe_float(row.get("noninertial_tracking_rms_high"))
        )
        grouped[kp]["inertial_tracking_rms_high"].append(
            safe_float(row.get("inertial_tracking_rms_high"))
        )

    fieldnames = [
        "kp",
        "samples",
        "noninertial_tracking_rms_mean",
        "inertial_tracking_rms_mean",
        "delta_tracking_rms_non_minus_inertial",
        "pct_noninertial_better_tracking_rms",
        "noninertial_tracking_rms_high_mean",
        "inertial_tracking_rms_high_mean",
        "delta_tracking_rms_high_non_minus_inertial",
        "pct_noninertial_better_tracking_rms_high",
    ]

    rows: list[dict[str, str]] = []
    for kp in sorted(grouped.keys(), key=lambda x: float(x)):
        non_mean = mean(grouped[kp]["noninertial_tracking_rms"])
        in_mean = mean(grouped[kp]["inertial_tracking_rms"])
        non_high_mean = mean(grouped[kp]["noninertial_tracking_rms_high"])
        in_high_mean = mean(grouped[kp]["inertial_tracking_rms_high"])
        delta = (
            non_mean - in_mean
            if math.isfinite(non_mean) and math.isfinite(in_mean)
            else math.nan
        )
        delta_high = (
            non_high_mean - in_high_mean
            if math.isfinite(non_high_mean) and math.isfinite(in_high_mean)
            else math.nan
        )
        rows.append(
            {
                "kp": kp,
                "samples": str(
                    len(
                        [
                            v
                            for v in grouped[kp]["noninertial_tracking_rms"]
                            if math.isfinite(v)
                        ]
                    )
                ),
                "noninertial_tracking_rms_mean": fmt(non_mean),
                "inertial_tracking_rms_mean": fmt(in_mean),
                "delta_tracking_rms_non_minus_inertial": fmt(delta),
                "pct_noninertial_better_tracking_rms": fmt(
                    pct_from_means(non_mean, in_mean)
                ),
                "noninertial_tracking_rms_high_mean": fmt(non_high_mean),
                "inertial_tracking_rms_high_mean": fmt(in_high_mean),
                "delta_tracking_rms_high_non_minus_inertial": fmt(delta_high),
                "pct_noninertial_better_tracking_rms_high": fmt(
                    pct_from_means(non_high_mean, in_high_mean)
                ),
            }
        )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    for row in rows:
        print(
            f"kp={row['kp']} "
            f"delta_tracking_rms_non_minus_inertial={row['delta_tracking_rms_non_minus_inertial']} "
            f"pct_noninertial_better_tracking_rms={row['pct_noninertial_better_tracking_rms']}% "
            f"delta_tracking_rms_high_non_minus_inertial={row['delta_tracking_rms_high_non_minus_inertial']} "
            f"pct_noninertial_better_tracking_rms_high={row['pct_noninertial_better_tracking_rms_high']}%"
        )

    print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
