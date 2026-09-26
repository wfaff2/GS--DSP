#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

DECIMALS = 6


def format_scalar(value: float | int | None) -> str:
    if value is None or not np.isfinite(value):
        return "N/A"
    return f"{float(value):.{DECIMALS}f}"


def format_rate(count: int, total: int) -> str:
    if total <= 0:
        return "N/A"
    return f"{count}/{total}"


def format_mean_std(series: pd.Series) -> str:
    values = pd.to_numeric(series, errors="coerce").dropna()
    if values.empty:
        return "N/A"
    if len(values) == 1:
        return f"{values.iloc[0]:.{DECIMALS}f} ± {0.0:.{DECIMALS}f}"
    std = float(values.std(ddof=1))
    if not np.isfinite(std):
        std = 0.0
    return f"{values.mean():.{DECIMALS}f} ± {std:.{DECIMALS}f}"


def load_scope_all_rows(run_root: Path, filename: str, method: str) -> pd.DataFrame:
    rows: list[dict] = []
    for seed_dir in sorted(path for path in run_root.glob("seed_*") if path.is_dir()):
        metrics_path = seed_dir / filename
        if not metrics_path.exists():
            continue
        metrics_df = pd.read_csv(metrics_path)
        scope_all = metrics_df[metrics_df["scope"] == "all"]
        if scope_all.empty:
            continue
        row = scope_all.iloc[0].to_dict()
        row["seed_key"] = seed_dir.name
        row["method"] = method
        rows.append(row)
    return pd.DataFrame(rows)


def build_no_cbf_table(run_root: Path) -> tuple[list[dict], str]:
    combined_path = run_root / "combined_robustness_data.csv"
    combined_df = pd.read_csv(combined_path)
    combined_df = combined_df[combined_df["method"].isin(["stage", "frozen"])].copy()
    combined_df["seed"] = pd.to_numeric(combined_df["seed"], errors="coerce")
    combined_df["tracking_error"] = pd.to_numeric(combined_df["tracking_error"], errors="coerce")
    combined_df["horizon_rms_xy"] = pd.to_numeric(combined_df["horizon_rms_xy"], errors="coerce")

    per_seed_prediction = (
        combined_df.groupby(["seed", "method"], as_index=False)
        .agg(
            mean_horizon_rmse=("horizon_rms_xy", "mean"),
            max_deviation=("tracking_error", "max"),
        )
    )

    tracking_rows = pd.concat(
        [
            load_scope_all_rows(run_root, "noninertial_metrics.csv", "stage"),
            load_scope_all_rows(run_root, "noninertial_frozen_metrics.csv", "frozen"),
        ],
        ignore_index=True,
    )
    tracking_rows["seed"] = tracking_rows["seed_key"].str.extract(r"seed_(\d+)").astype(float)
    tracking_rows["tracking_rms"] = pd.to_numeric(tracking_rows["tracking_rms"], errors="coerce")

    per_seed = per_seed_prediction.merge(
        tracking_rows[["seed", "method", "tracking_rms"]],
        on=["seed", "method"],
        how="left",
    )

    wide = per_seed.pivot(index="seed", columns="method")
    stage_wins = (
        (wide["mean_horizon_rmse"]["stage"] < wide["mean_horizon_rmse"]["frozen"])
        & (wide["tracking_rms"]["stage"] < wide["tracking_rms"]["frozen"])
        & (wide["max_deviation"]["stage"] < wide["max_deviation"]["frozen"])
    ).sum()
    total = len(wide)
    frozen_wins = total - int(stage_wins)

    rows: list[dict] = []
    for method in ("frozen", "stage"):
        df_method = per_seed[per_seed["method"] == method]
        rows.append(
            {
                "label": "Frozen" if method == "frozen" else "Stage-wise",
                "mean_horizon_rmse": format_mean_std(df_method["mean_horizon_rmse"]),
                "tracking_rms": format_mean_std(df_method["tracking_rms"]),
                "max_deviation": format_mean_std(df_method["max_deviation"]),
                "win_rate": format_rate(frozen_wins if method == "frozen" else int(stage_wins), total),
            }
        )

    note = (
        "Win Rate = fraction of seeds where the method beats the other on "
        "Mean Horizon RMSE, Tracking RMS, and Max Deviation simultaneously."
    )
    return rows, note


def build_with_cbf_table(run_root: Path) -> tuple[list[dict], str]:
    per_seed = pd.concat(
        [
            load_scope_all_rows(run_root, "noninertial_metrics.csv", "stage"),
            load_scope_all_rows(run_root, "noninertial_frozen_metrics.csv", "frozen"),
        ],
        ignore_index=True,
    )
    per_seed["tracking_rms"] = pd.to_numeric(per_seed["tracking_rms"], errors="coerce")
    per_seed["min_h"] = pd.to_numeric(per_seed["min_h"], errors="coerce")
    per_seed["collision"] = pd.to_numeric(per_seed["collision"], errors="coerce").fillna(0.0)
    per_seed["slack_sum_effective"] = pd.to_numeric(
        per_seed["solver_slack_sum"].where(per_seed["solver_slack_sum"].notna(), per_seed["slack_sum"]),
        errors="coerce",
    )

    stage = per_seed[per_seed["method"] == "stage"][["seed_key", "tracking_rms", "min_h", "collision"]]
    frozen = per_seed[per_seed["method"] == "frozen"][["seed_key", "tracking_rms", "min_h", "collision"]]
    merged = stage.merge(frozen, on="seed_key", suffixes=("_stage", "_frozen"))
    stage_wins = (
        (merged["tracking_rms_stage"] < merged["tracking_rms_frozen"])
        & (merged["collision_stage"] <= merged["collision_frozen"])
    ).sum()
    total = len(merged)
    frozen_wins = total - int(stage_wins)

    rows: list[dict] = []
    for method in ("frozen", "stage"):
        df_method = per_seed[per_seed["method"] == method]
        collision_count = int((df_method["collision"] > 0).sum())
        rows.append(
            {
                "label": "Frozen + CBF" if method == "frozen" else "Stage-wise + CBF",
                "tracking_rms": format_mean_std(df_method["tracking_rms"]),
                "min_h": format_mean_std(df_method["min_h"]),
                "slack_sum": format_mean_std(df_method["slack_sum_effective"]),
                "collision_rate": format_rate(collision_count, len(df_method)),
                "win_rate": format_rate(frozen_wins if method == "frozen" else int(stage_wins), total),
            }
        )

    note = (
        "Min h uses each seed's scope=all minimum h, then reports mean ± std across seeds. "
        "Slack Sum uses each seed's scope=all solver_slack_sum (fallback: slack_sum), then reports mean ± std across seeds. "
        "Win Rate = fraction of seeds where the method has lower Tracking RMS without higher collision count."
    )
    return rows, note


def render_no_cbf_section(title: str, rows: list[dict], note: str) -> str:
    lines = [f"# {title}", ""]
    for row in rows:
        lines.extend(
            [
                f"{row['label']}:",
                f"Mean Horizon RMSE = {row['mean_horizon_rmse']}",
                f"Tracking RMS = {row['tracking_rms']}",
                f"Max Deviation = {row['max_deviation']}",
                f"Win Rate = {row['win_rate']}",
                "",
            ]
        )
    lines.extend([note, ""])
    return "\n".join(lines)


def render_with_cbf_section(title: str, rows: list[dict], note: str) -> str:
    lines = [f"# {title}", ""]
    for row in rows:
        lines.extend(
            [
                f"{row['label']}:",
                f"Tracking RMS = {row['tracking_rms']}",
                f"Min h = {row['min_h']}",
                f"Slack Sum = {row['slack_sum']}",
                f"Collision Rate = {row['collision_rate']}",
                f"Win Rate = {row['win_rate']}",
                "",
            ]
        )
    lines.extend([note, ""])
    return "\n".join(lines)


def write_table(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Build no-CBF and with-CBF summary tables.")
    parser.add_argument("--nocbf-root", required=True, help="Run root for the no-CBF result set.")
    parser.add_argument("--with-cbf-root", required=True, help="Run root for the with-CBF result set.")
    parser.add_argument("--out", required=True, help="Combined markdown output path.")
    args = parser.parse_args()

    nocbf_root = Path(args.nocbf_root)
    with_cbf_root = Path(args.with_cbf_root)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    no_cbf_df, no_cbf_note = build_no_cbf_table(nocbf_root)
    with_cbf_df, with_cbf_note = build_with_cbf_table(with_cbf_root)

    no_cbf_text = render_no_cbf_section(
        "Table 1: no-CBF Prediction + Tracking Summary",
        no_cbf_df,
        no_cbf_note,
    )
    with_cbf_text = render_with_cbf_section(
        "Table 2: with-CBF Summary",
        with_cbf_df,
        with_cbf_note,
    )
    combined_text = "\n".join(
        [
            "# Stage-wise Summary Tables",
            "",
            "## Table 1: no-CBF Prediction + Tracking Summary",
            "",
            *no_cbf_text.splitlines()[2:],
            "",
            "## Table 2: with-CBF Summary",
            "",
            *with_cbf_text.splitlines()[2:],
            "",
        ]
    )
    out_path.write_text(combined_text, encoding="utf-8")

    write_table(
        nocbf_root / "table_1_nocbf_prediction_tracking_summary.md",
        no_cbf_text,
    )
    write_table(
        with_cbf_root / "table_2_with_cbf_summary.md",
        with_cbf_text,
    )
    print(f"[INFO] combined summary tables written to: {out_path}")
    print(f"[INFO] no-CBF table written to: {nocbf_root / 'table_1_nocbf_prediction_tracking_summary.md'}")
    print(f"[INFO] with-CBF table written to: {with_cbf_root / 'table_2_with_cbf_summary.md'}")


if __name__ == "__main__":
    main()
