#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, List

import numpy as np
import pandas as pd


SUMMARY_COLUMNS = [
    "tracking_rms",
    "tracking_p95",
    "prediction_horizon_rms_xy_mean",
    "prediction_horizon_rms_xy_p95",
    "eq17_boundary_drift_rms_mean",
    "eq17_boundary_drift_rms_p95",
    "eq18_frot_mismatch_rms_mean",
    "eq18_frot_mismatch_rms_p95",
    "solver_min_h",
    "solver_slack_sum",
    "core_time_mean_ms",
    "core_time_p95_ms",
    "core_time_worst_uav_p95_ms",
    "core_time_max_ms",
    "core_time_over_20ms_rate",
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Merge per-seed VW experiment tables into one consolidated summary."
    )
    p.add_argument("--run-roots", nargs="+", required=True)
    p.add_argument("--out-dir", required=True)
    p.add_argument("--controllers", default="")
    p.add_argument("--v-values", default="")
    p.add_argument("--w-values", default="")
    return p.parse_args()


def parse_float_list(text: str) -> List[float]:
    if not text.strip():
        return []
    return [float(x.strip()) for x in text.split(",") if x.strip()]


def parse_text_list(text: str) -> List[str]:
    if not text.strip():
        return []
    return [x.strip() for x in text.split(",") if x.strip()]


def load_csvs(paths: Iterable[Path]) -> pd.DataFrame:
    frames = []
    for path in paths:
        if path.is_file():
            frames.append(pd.read_csv(path))
    if not frames:
        return pd.DataFrame()
    return pd.concat(frames, ignore_index=True)


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    controllers = set(parse_text_list(args.controllers))
    v_values = set(parse_float_list(args.v_values))
    w_values = set(parse_float_list(args.w_values))

    run_roots = [Path(p) for p in args.run_roots]
    per_seed_paths = [r / "tables" / "per_seed_controller_metrics.csv" for r in run_roots]
    core_time_paths = [r / "tables" / "core_time_by_seed_uav.csv" for r in run_roots]

    per_seed = load_csvs(per_seed_paths)
    core_time = load_csvs(core_time_paths)

    if per_seed.empty:
        raise SystemExit("no per_seed_controller_metrics.csv found")

    # Keep the newest duplicate if the same controller/v/w/seed appears more than once.
    if "metrics_csv" in per_seed.columns:
        per_seed = per_seed.sort_values("metrics_csv")
    key_cols = ["controller", "v", "w", "seed"]
    per_seed = per_seed.drop_duplicates(subset=key_cols, keep="last").copy()

    if controllers:
        per_seed = per_seed[per_seed["controller"].isin(controllers)].copy()
        if not core_time.empty:
            core_time = core_time[core_time["controller"].isin(controllers)].copy()
    if v_values:
        per_seed = per_seed[per_seed["v"].isin(v_values)].copy()
        if not core_time.empty:
            core_time = core_time[core_time["v"].isin(v_values)].copy()
    if w_values:
        per_seed = per_seed[per_seed["w"].isin(w_values)].copy()
        if not core_time.empty:
            core_time = core_time[core_time["w"].isin(w_values)].copy()

    per_seed.to_csv(out_dir / "per_seed_controller_metrics.csv", index=False)
    if not core_time.empty:
        core_time = core_time.drop_duplicates(
            subset=["controller", "v", "w", "seed", "uav_idx"], keep="last"
        ).copy()
        core_time.to_csv(out_dir / "core_time_by_seed_uav.csv", index=False)

    grouped = []
    for (v, w, controller), g in per_seed.groupby(["v", "w", "controller"], sort=True):
        row = {
            "v": float(v),
            "w": float(w),
            "controller": str(controller),
            "seed_count": int(g["seed"].nunique()),
            "row_count": int(len(g)),
            "ok_count": int((g["status"].astype(str) == "ok").sum()) if "status" in g.columns else int(len(g)),
            "collision_count": int(pd.to_numeric(g.get("collision"), errors="coerce").fillna(0).sum()),
            "collision_rate": float(pd.to_numeric(g.get("collision"), errors="coerce").mean()),
        }
        for col in SUMMARY_COLUMNS:
            series = pd.to_numeric(g.get(col), errors="coerce")
            finite = series[np.isfinite(series)]
            if col == "solver_min_h":
                row["solver_min_h_min"] = float(finite.min()) if not finite.empty else np.nan
                row["solver_min_h_mean"] = float(finite.mean()) if not finite.empty else np.nan
            elif col == "solver_slack_sum":
                row["solver_slack_sum_mean"] = float(finite.mean()) if not finite.empty else np.nan
                row["solver_slack_sum_total"] = float(finite.sum()) if not finite.empty else np.nan
            elif col == "core_time_worst_uav_p95_ms":
                row["core_time_worst_uav_p95_ms_max"] = float(finite.max()) if not finite.empty else np.nan
            elif col == "core_time_max_ms":
                row["core_time_max_ms_max"] = float(finite.max()) if not finite.empty else np.nan
            else:
                out_name = {
                    "tracking_rms": "tracking_rms_mean",
                    "tracking_p95": "tracking_p95_mean",
                    "prediction_horizon_rms_xy_mean": "prediction_horizon_rms_xy_mean",
                    "prediction_horizon_rms_xy_p95": "prediction_horizon_rms_xy_p95",
                    "eq17_boundary_drift_rms_mean": "eq17_boundary_drift_rms_mean",
                    "eq17_boundary_drift_rms_p95": "eq17_boundary_drift_rms_p95",
                    "eq18_frot_mismatch_rms_mean": "eq18_frot_mismatch_rms_mean",
                    "eq18_frot_mismatch_rms_p95": "eq18_frot_mismatch_rms_p95",
                    "core_time_mean_ms": "core_time_mean_ms_mean",
                    "core_time_p95_ms": "core_time_p95_ms_mean",
                    "core_time_over_20ms_rate": "core_time_over_20ms_rate_mean",
                }[col]
                row[out_name] = float(finite.mean()) if not finite.empty else np.nan
                if col == "tracking_rms":
                    row["tracking_rms_std"] = float(finite.std(ddof=1)) if len(finite) > 1 else np.nan
                    row["tracking_rms_p95"] = float(finite.quantile(0.95)) if not finite.empty else np.nan
        grouped.append(row)

    summary = pd.DataFrame(grouped).sort_values(["v", "w", "controller"]).reset_index(drop=True)
    summary.to_csv(out_dir / "summary_by_vw_controller.csv", index=False)
    print(out_dir / "summary_by_vw_controller.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
