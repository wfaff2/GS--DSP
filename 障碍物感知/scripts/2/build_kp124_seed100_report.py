#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


KP_TO_V = {1: 0.5, 2: 1.0, 4: 2.0}
V_TO_KP = {v: k for k, v in KP_TO_V.items()}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Build requested 100-seed KPI report.")
    p.add_argument("--circle-run", required=True)
    p.add_argument("--figure8-clean-run", required=True)
    p.add_argument("--with-cbf-merged-summary", required=True)
    p.add_argument("--with-cbf-run-roots", nargs="+", required=True)
    p.add_argument("--failure-threshold-x", type=float, default=0.215)
    p.add_argument("--out-dir", required=True)
    return p.parse_args()


def load_summary(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["v"] = pd.to_numeric(df["v"], errors="coerce")
    df["w"] = pd.to_numeric(df["w"], errors="coerce")
    return df


def filter_kp_rows(df: pd.DataFrame) -> pd.DataFrame:
    keep_v = set(KP_TO_V.values())
    out = df[df["v"].isin(keep_v) & df["w"].isin(keep_v)].copy()
    out["KP"] = out["v"].map(V_TO_KP)
    return out


def load_complete_trials(run_root: Path) -> pd.DataFrame:
    per_seed_path = run_root / "tables" / "per_seed_controller_metrics.csv"
    if not per_seed_path.is_file():
        raise FileNotFoundError(f"Missing per-seed table: {per_seed_path}")
    trials = pd.read_csv(per_seed_path)
    trials["v"] = pd.to_numeric(trials["v"], errors="coerce")
    trials["w"] = pd.to_numeric(trials["w"], errors="coerce")
    bad = trials[~trials["status"].isin(["ok", "skipped"])].copy()
    if not bad.empty:
        preview = ", ".join(
            f"{row.controller} v={row.v} w={row.w} seed={int(row.seed)} status={row.status}"
            for row in bad.head(8).itertuples()
        )
        raise RuntimeError(
            "Incomplete trial set detected; repair or rerun these cases before building the report: "
            f"{preview}"
        )
    return trials


def dedupe_concat(csv_paths: Iterable[Path], key_cols: List[str]) -> pd.DataFrame:
    frames = []
    for path in csv_paths:
        if path.is_file():
            frame = pd.read_csv(path)
            frame["__src"] = str(path)
            frames.append(frame)
    if not frames:
        return pd.DataFrame()
    df = pd.concat(frames, ignore_index=True)
    df = df.sort_values("__src").drop_duplicates(subset=key_cols, keep="last").copy()
    df.drop(columns=["__src"], inplace=True)
    return df


def build_no_cbf_tables(circle_summary: pd.DataFrame, fig8_summary: pd.DataFrame) -> pd.DataFrame:
    rows: List[Dict[str, object]] = []
    for scenario, df in [("circle", circle_summary), ("figure8_clean", fig8_summary)]:
        filtered = filter_kp_rows(df)
        for _, row in filtered.iterrows():
            rows.append(
                {
                    "scenario": scenario,
                    "KP": int(row["KP"]),
                    "V": float(row["v"]),
                    "W": float(row["w"]),
                    "controller": str(row["controller"]),
                    "seed_count": int(row["seed_count"]),
                    "tracking_rms_mean": float(row["tracking_rms_mean"]),
                    "prediction_rms_mean": float(row["prediction_horizon_rms_xy_mean"]),
                }
            )
    return pd.DataFrame(rows).sort_values(["scenario", "KP", "controller"]).reset_index(drop=True)


def collect_step_metrics(run_roots: List[Path]) -> pd.DataFrame:
    rows: List[Dict[str, object]] = []
    for run_root in run_roots:
        trials = load_complete_trials(run_root)
        missing_step_files: List[str] = []
        for trial in trials.itertuples():
            controller = str(trial.controller)
            v = float(trial.v)
            w = float(trial.w)
            seed = int(trial.seed)
            metrics_path = Path(str(trial.metrics_csv))
            step_path = metrics_path.with_name("metrics_steps.csv")
            if not step_path.is_file():
                missing_step_files.append(
                    f"{run_root.name}: {controller} v={v} w={w} seed={seed}"
                )
                continue
            min_surface = math.nan
            slack_num = 0
            with step_path.open(newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    val = row.get("solver_planar_surface_distance", "")
                    if val not in ("", "nan", "NaN"):
                        x = float(val)
                        if not math.isfinite(min_surface) or x < min_surface:
                            min_surface = x
                    sval = row.get("solver_slack", "")
                    if sval not in ("", "nan", "NaN") and float(sval) > 1e-9:
                        slack_num += 1
            rows.append(
                {
                    "controller": controller,
                    "v": v,
                    "w": w,
                    "seed": seed,
                    "min_surface_distance": min_surface,
                    "slack_num": slack_num,
                    "__src": str(step_path),
                }
            )
        if missing_step_files:
            raise RuntimeError(
                "Missing metrics_steps.csv for completed trials: "
                + "; ".join(missing_step_files[:8])
            )
    if not rows:
        return pd.DataFrame()
    df = pd.DataFrame(rows)
    df = df.sort_values("__src").drop_duplicates(
        subset=["controller", "v", "w", "seed"], keep="last"
    )
    df.drop(columns=["__src"], inplace=True)
    return df


def build_with_cbf_advantage_table(
    merged_summary: pd.DataFrame,
    step_df: pd.DataFrame,
    failure_threshold_x: float,
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    summary = filter_kp_rows(merged_summary)
    summary = summary[summary["controller"].isin(["inertial_frozen", "noninertial_frozen", "noninertial_stage"])].copy()

    raw_rows: List[Dict[str, object]] = []
    adv_rows: List[Dict[str, object]] = []

    step_df = step_df[step_df["v"].isin(KP_TO_V.values()) & step_df["w"].isin(KP_TO_V.values())].copy()
    step_df["KP"] = step_df["v"].map(V_TO_KP)

    for kp, v in KP_TO_V.items():
        kp_summary = summary[(summary["v"] == v) & (summary["w"] == v)]
        kp_steps = step_df[(step_df["v"] == v) & (step_df["w"] == v)]
        raw_by_ctrl: Dict[str, Dict[str, float]] = {}
        for controller in ["inertial_frozen", "noninertial_frozen", "noninertial_stage"]:
            srow = kp_summary[kp_summary["controller"] == controller]
            if srow.empty:
                continue
            srow = srow.iloc[0]
            prow = kp_steps[kp_steps["controller"] == controller]
            if prow.empty:
                fail_rate = math.nan
                slack_num_mean = math.nan
            else:
                fail_rate = float((prow["min_surface_distance"] < failure_threshold_x).mean())
                slack_num_mean = float(pd.to_numeric(prow["slack_num"], errors="coerce").mean())
            raw = {
                "KP": kp,
                "V": v,
                "controller": controller,
                "seed_count": int(srow["seed_count"]),
                "failure_rate_x": fail_rate,
                "solver_min_h_mean": float(srow["solver_min_h_mean"]),
                "solver_slack_sum_mean": float(srow["solver_slack_sum_mean"]),
                "slack_num_mean": slack_num_mean,
            }
            raw_rows.append(raw)
            raw_by_ctrl[controller] = raw

        if "noninertial_frozen" in raw_by_ctrl and "noninertial_stage" in raw_by_ctrl:
            fr = raw_by_ctrl["noninertial_frozen"]
            st = raw_by_ctrl["noninertial_stage"]
            slack_num_adv_pct = (
                (fr["slack_num_mean"] - st["slack_num_mean"]) / fr["slack_num_mean"] * 100.0
                if fr["slack_num_mean"] and math.isfinite(fr["slack_num_mean"])
                else math.nan
            )
            adv_rows.append(
                {
                    "KP": kp,
                    "V": v,
                    "failure_threshold_x": failure_threshold_x,
                    "frozen_failure_rate": fr["failure_rate_x"],
                    "stage_failure_rate": st["failure_rate_x"],
                    "failure_rate_gap": fr["failure_rate_x"] - st["failure_rate_x"],
                    "frozen_solver_min_h_mean": fr["solver_min_h_mean"],
                    "stage_solver_min_h_mean": st["solver_min_h_mean"],
                    "min_h_delta": st["solver_min_h_mean"] - fr["solver_min_h_mean"],
                    "frozen_slack_num_mean": fr["slack_num_mean"],
                    "stage_slack_num_mean": st["slack_num_mean"],
                    "slack_num_adv_pct": slack_num_adv_pct,
                }
            )

    return (
        pd.DataFrame(raw_rows).sort_values(["KP", "controller"]).reset_index(drop=True),
        pd.DataFrame(adv_rows).sort_values(["KP"]).reset_index(drop=True),
    )


def build_stage_p95_table(
    circle_summary: pd.DataFrame,
    fig8_summary: pd.DataFrame,
    with_cbf_summary: pd.DataFrame,
) -> pd.DataFrame:
    rows: List[Dict[str, object]] = []
    for scenario, df in [("circle_no_cbf", circle_summary), ("figure8_clean_no_cbf", fig8_summary), ("figure8_fixedobs_with_cbf", with_cbf_summary)]:
        filtered = filter_kp_rows(df)
        filtered = filtered[filtered["controller"] == "noninertial_stage"].copy()
        for _, row in filtered.iterrows():
            rows.append(
                {
                    "scenario": scenario,
                    "KP": int(row["KP"]),
                    "V": float(row["v"]),
                    "W": float(row["w"]),
                    "stage_core_time_p95_ms_mean": float(row["core_time_p95_ms_mean"]),
                }
            )
    return pd.DataFrame(rows).sort_values(["scenario", "KP"]).reset_index(drop=True)


def write_markdown(
    no_cbf_df: pd.DataFrame,
    with_cbf_raw: pd.DataFrame,
    with_cbf_adv: pd.DataFrame,
    stage_p95_df: pd.DataFrame,
    out_dir: Path,
    failure_threshold_x: float,
) -> None:
    md_path = out_dir / "kp124_seed100_report.md"
    lines: List[str] = []
    lines.append("# KP124 100-seed report")
    lines.append("")
    lines.append("## No-CBF mean metrics")
    lines.append("")
    lines.append("| Scenario | KP | Controller | Mean tracking RMS | Mean prediction RMS |")
    lines.append("|---|---:|---|---:|---:|")
    for _, row in no_cbf_df.iterrows():
        lines.append(
            f"| {row['scenario']} | {int(row['KP'])} | {row['controller']} | "
            f"{float(row['tracking_rms_mean']):.6f} | {float(row['prediction_rms_mean']):.6f} |"
        )
    lines.append("")
    lines.append(f"## With-CBF advantage table (failure threshold x = {failure_threshold_x:.3f} m)")
    lines.append("")
    lines.append("| KP | Frozen failure rate | Stage failure rate | Failure-rate gap | Min-h delta | Frozen slack_num | Stage slack_num | Slack_num advantage |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|")
    for _, row in with_cbf_adv.iterrows():
        lines.append(
            f"| {int(row['KP'])} | {float(row['frozen_failure_rate'])*100:.1f}% | "
            f"{float(row['stage_failure_rate'])*100:.1f}% | {float(row['failure_rate_gap'])*100:.1f}% | "
            f"{float(row['min_h_delta']):+.6f} | {float(row['frozen_slack_num_mean']):.2f} | "
            f"{float(row['stage_slack_num_mean']):.2f} | {float(row['slack_num_adv_pct']):.2f}% |"
        )
    lines.append("")
    lines.append("## Stage P95 solve time")
    lines.append("")
    lines.append("| Scenario | KP | Stage P95 solve time [ms] |")
    lines.append("|---|---:|---:|")
    for _, row in stage_p95_df.iterrows():
        lines.append(
            f"| {row['scenario']} | {int(row['KP'])} | {float(row['stage_core_time_p95_ms_mean']):.6f} |"
        )
    md_path.write_text("\n".join(lines), encoding="utf-8")


def build_plot(no_cbf_df: pd.DataFrame, with_cbf_adv: pd.DataFrame, stage_p95_df: pd.DataFrame, out_dir: Path) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6), constrained_layout=True)

    color_map = {
        "inertial_frozen": "#4C72B0",
        "noninertial_frozen": "#55A868",
        "noninertial_stage": "#C44E52",
    }
    marker_map = {"circle": "o", "figure8_clean": "s"}

    ax = axes[0]
    for scenario, label in [("circle", "Circle"), ("figure8_clean", "Figure-8 clean")]:
        sub = no_cbf_df[no_cbf_df["scenario"] == scenario]
        for controller in ["inertial_frozen", "noninertial_frozen", "noninertial_stage"]:
            ss = sub[sub["controller"] == controller]
            ax.plot(
                ss["KP"], ss["tracking_rms_mean"],
                marker=marker_map[scenario], linewidth=2.0,
                color=color_map[controller], alpha=0.85,
                label=f"{label} / {controller}",
            )
    ax.set_title("No-CBF Mean Tracking RMS")
    ax.set_xlabel("KP")
    ax.set_ylabel("Tracking RMS [m]")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.legend(frameon=False, fontsize=8)

    ax = axes[1]
    ax.plot(with_cbf_adv["KP"], with_cbf_adv["failure_rate_gap"] * 100.0, marker="o", linewidth=2.2, color="#C44E52", label="Failure-rate gap")
    ax.plot(with_cbf_adv["KP"], with_cbf_adv["slack_num_adv_pct"], marker="s", linewidth=2.2, color="#64B5CD", label="Slack_num advantage")
    ax.axhline(0.0, color="0.6", linewidth=1.0)
    ax.set_title("With-CBF Stage vs Frozen")
    ax.set_xlabel("KP")
    ax.set_ylabel("Advantage [%]")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.legend(frameon=False)

    ax = axes[2]
    for scenario in ["circle_no_cbf", "figure8_clean_no_cbf", "figure8_fixedobs_with_cbf"]:
        sub = stage_p95_df[stage_p95_df["scenario"] == scenario]
        ax.plot(sub["KP"], sub["stage_core_time_p95_ms_mean"], marker="o", linewidth=2.2, label=scenario)
    ax.set_title("Stage Controller P95 Solve Time")
    ax.set_xlabel("KP")
    ax.set_ylabel("P95 solve time [ms]")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.legend(frameon=False, fontsize=8)

    fig.savefig(out_dir / "kp124_seed100_report.png", dpi=220, bbox_inches="tight")
    fig.savefig(out_dir / "kp124_seed100_report.pdf", bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    circle_summary = load_summary(Path(args.circle_run) / "tables" / "summary_by_vw_controller.csv")
    fig8_summary = load_summary(Path(args.figure8_clean_run) / "tables" / "summary_by_vw_controller.csv")
    with_cbf_summary = load_summary(Path(args.with_cbf_merged_summary))
    with_cbf_roots = [Path(p) for p in args.with_cbf_run_roots]

    no_cbf_df = build_no_cbf_tables(circle_summary, fig8_summary)
    step_df = collect_step_metrics(with_cbf_roots)
    with_cbf_raw, with_cbf_adv = build_with_cbf_advantage_table(
        with_cbf_summary, step_df, args.failure_threshold_x
    )
    stage_p95_df = build_stage_p95_table(circle_summary, fig8_summary, with_cbf_summary)

    no_cbf_df.to_csv(out_dir / "kp124_seed100_no_cbf_mean_metrics.csv", index=False)
    with_cbf_raw.to_csv(out_dir / "kp124_seed100_with_cbf_raw.csv", index=False)
    with_cbf_adv.to_csv(out_dir / "kp124_seed100_with_cbf_advantage.csv", index=False)
    stage_p95_df.to_csv(out_dir / "kp124_seed100_stage_p95_solve_time.csv", index=False)
    write_markdown(no_cbf_df, with_cbf_raw, with_cbf_adv, stage_p95_df, out_dir, args.failure_threshold_x)
    build_plot(no_cbf_df, with_cbf_adv, stage_p95_df, out_dir)
    print(out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
