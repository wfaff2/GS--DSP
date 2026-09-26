#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import pandas as pd

BASE_NUMERIC_FIELDS = [
    "tracking_rms",
    "tracking_rms_track_only",
    "tracking_rms_cbf_active",
    "solver_tracking_rms_avoid",
    "slack_sum_effective",
    "min_h",
    "track_only_ratio",
    "cbf_active_ratio",
    "collision",
]


def infer_kp(path: Path) -> float | None:
    for part in path.parts:
        match = re.fullmatch(r"kp_(\d+)p(\d+)", part)
        if match:
            return float(f"{match.group(1)}.{match.group(2)}")
    return None


def infer_seed(path: Path) -> int | None:
    for part in path.parts:
        match = re.fullmatch(r"seed_(\d+)", part)
        if match:
            return int(match.group(1))
    return None


def safe_float(value: object) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return math.nan
    return out if math.isfinite(out) else math.nan


def load_seed_filter(path: Path | None) -> dict[float, set[int]]:
    if path is None:
        return {}
    df = pd.read_csv(path)
    if not {"kp", "seed"}.issubset(df.columns):
        raise RuntimeError(f"Seed filter CSV must contain kp, seed columns: {path}")
    keep: dict[float, set[int]] = {}
    for kp, dkp in df.groupby("kp"):
        keep[float(kp)] = set(int(v) for v in dkp["seed"].tolist())
    return keep


def load_scope_all(metrics_path: Path, surface_distance_radius_sum: float | None) -> dict[str, float]:
    df = pd.read_csv(metrics_path)
    if "scope" not in df.columns:
        raise RuntimeError(f"Missing scope column in {metrics_path}")
    scope_all = df[df["scope"] == "all"]
    if scope_all.empty:
        raise RuntimeError(f"No scope=all row in {metrics_path}")
    row = scope_all.iloc[0]
    values = {field: safe_float(row.get(field)) for field in BASE_NUMERIC_FIELDS}
    solver_slack_sum = safe_float(row.get("solver_slack_sum"))
    raw_slack_sum = safe_float(row.get("slack_sum"))
    values["slack_sum_effective"] = (
        solver_slack_sum if math.isfinite(solver_slack_sum) else raw_slack_sum
    )
    if surface_distance_radius_sum is not None:
        witness_planar_distance = safe_float(row.get("solver_min_h_witness_planar_distance"))
        values["surface_distance"] = (
            witness_planar_distance - surface_distance_radius_sum
            if math.isfinite(witness_planar_distance)
            else math.nan
        )
    values["frame_mode_effective"] = str(row.get("frame_mode_effective", ""))
    values["ugv_rollout_mode"] = str(row.get("ugv_rollout_mode", ""))
    return values


def canonical_case(frame_mode_effective: str, ugv_rollout_mode: str) -> str | None:
    if frame_mode_effective == "inertial":
        return "inertial_frozen"
    if frame_mode_effective == "noninertial" and ugv_rollout_mode == "frozen":
        return "noninertial_frozen"
    if frame_mode_effective == "noninertial" and ugv_rollout_mode == "stage":
        return "noninertial_stage"
    return None


def discover_metrics_files(root: Path) -> list[Path]:
    files: list[Path] = []
    for seed_dir in sorted(path for path in root.rglob("seed_*") if path.is_dir()):
        files.extend(
            sorted(
                path
                for path in seed_dir.glob("*_metrics.csv")
                if path.name.endswith("_metrics.csv")
            )
        )
    return files


def collect_rows(
    roots: list[Path],
    subset: str,
    seed_filter: dict[float, set[int]],
    surface_distance_radius_sum: float | None,
) -> list[dict]:
    rows: list[dict] = []
    seen_metrics: set[Path] = set()
    for root in roots:
        for metrics_path in discover_metrics_files(root):
            metrics_path = metrics_path.resolve()
            if metrics_path in seen_metrics:
                continue
            seen_metrics.add(metrics_path)
            kp = infer_kp(metrics_path)
            seed = infer_seed(metrics_path)
            if kp is None or seed is None:
                continue
            if seed_filter and seed not in seed_filter.get(kp, set()):
                continue
            values = load_scope_all(metrics_path, surface_distance_radius_sum)
            case = canonical_case(
                str(values.pop("frame_mode_effective", "")),
                str(values.pop("ugv_rollout_mode", "")),
            )
            if case is None:
                continue
            rows.append(
                {
                    "subset": subset,
                    "kp": kp,
                    "case": case,
                    "seed": seed,
                    "metrics_csv": str(metrics_path),
                    **values,
                }
            )
    return rows


def build_summary(seed_df: pd.DataFrame) -> pd.DataFrame:
    agg_map: dict[str, list[str]] = {"seed": ["count"]}
    numeric_fields = [
        field for field in [*BASE_NUMERIC_FIELDS, "surface_distance"] if field in seed_df.columns
    ]
    for field in numeric_fields:
        agg_map[field] = ["mean", "std"]
    summary = (
        seed_df.groupby(["subset", "kp", "case"], dropna=False)
        .agg(agg_map)
        .reset_index()
        .sort_values(["kp", "case"])
    )
    flat_columns: list[str] = []
    for col in summary.columns:
        if isinstance(col, tuple):
            left, right = col
            if left == "seed" and right == "count":
                flat_columns.append("num_seeds")
            elif right:
                flat_columns.append(f"{left}_{right}")
            else:
                flat_columns.append(left)
        else:
            flat_columns.append(str(col))
    summary.columns = flat_columns
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Aggregate a kp-grid of inertial/noninertial/noninertial-stage metrics."
    )
    parser.add_argument(
        "run_roots",
        nargs="+",
        type=Path,
        help="One or more run roots containing seed_* metrics under kp_* directories.",
    )
    parser.add_argument("--subset", default="run", help="Subset label written into the CSV.")
    parser.add_argument(
        "--out-prefix",
        default="kp_grid_three_case",
        help="Prefix for the generated CSV files.",
    )
    parser.add_argument(
        "--seed-filter-csv",
        type=Path,
        default=None,
        help="Optional CSV with kp,seed rows to restrict which seeds are kept.",
    )
    parser.add_argument(
        "--surface-distance-radius-sum",
        type=float,
        default=None,
        help=(
            "If set, also compute `surface_distance = solver_min_h_witness_planar_distance - radius_sum` "
            "and include it in the seed and summary CSVs."
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_roots = [path.resolve() for path in args.run_roots]
    seed_filter = load_seed_filter(args.seed_filter_csv.resolve() if args.seed_filter_csv else None)
    rows = collect_rows(
        run_roots,
        args.subset,
        seed_filter,
        args.surface_distance_radius_sum,
    )
    if not rows:
        raise SystemExit(f"No seed metrics found under {run_roots}")

    seed_df = pd.DataFrame(rows).sort_values(["kp", "case", "seed"])
    summary_df = build_summary(seed_df)

    out_root = run_roots[0]
    seed_path = out_root / f"{args.out_prefix}_seed_metrics.csv"
    summary_path = out_root / f"{args.out_prefix}_summary.csv"
    seed_df.to_csv(seed_path, index=False)
    summary_df.to_csv(summary_path, index=False)

    print(f"[INFO] wrote {seed_path}")
    print(f"[INFO] wrote {summary_path}")
    print(summary_df.to_string(index=False))


if __name__ == "__main__":
    main()
