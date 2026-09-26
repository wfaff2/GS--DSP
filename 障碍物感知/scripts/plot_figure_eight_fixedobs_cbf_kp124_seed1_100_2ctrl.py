#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


BASE_V = 0.5
CONTROLLERS = [
    ("noninertial_frozen", "NI-Frozen", "#1f77b4"),
    ("noninertial_stage", "NI-Stage", "#d62728"),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot fixed-obstacle figure-eight CBF KP=1,2,4 seed1-100 two-controller results."
    )
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--out-prefix", type=Path, required=True)
    parser.add_argument("--failure-threshold-x", type=float, default=0.215)
    return parser.parse_args()


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
            "Incomplete trial set detected; repair or rerun these cases before plotting: "
            f"{preview}"
        )
    return trials


def collect_step_metrics(run_root: Path) -> pd.DataFrame:
    rows = []
    trials = load_complete_trials(run_root)
    missing_step_files = []
    for trial in trials.itertuples():
        controller = str(trial.controller)
        v = float(trial.v)
        w = float(trial.w)
        seed = int(trial.seed)
        metrics_path = Path(str(trial.metrics_csv))
        step_path = metrics_path.with_name("metrics_steps.csv")
        if not step_path.is_file():
            missing_step_files.append(
                f"{controller} v={v} w={w} seed={seed} missing {step_path}"
            )
            continue
        min_surface = math.nan
        slack_num = 0
        with step_path.open(newline="") as f:
            reader = pd.read_csv(f)
        if "solver_planar_surface_distance" in reader.columns:
            values = pd.to_numeric(reader["solver_planar_surface_distance"], errors="coerce")
            values = values[values.notna()]
            if not values.empty:
                min_surface = float(values.min())
        if "solver_slack" in reader.columns:
            sval = pd.to_numeric(reader["solver_slack"], errors="coerce").fillna(0.0)
            slack_num = int((sval > 1e-9).sum())
        rows.append(
            {
                "controller": controller,
                "v": v,
                "w": w,
                "seed": seed,
                "min_surface_distance": min_surface,
                "slack_num": slack_num,
            }
        )
    if missing_step_files:
        raise RuntimeError(
            "Missing metrics_steps.csv for completed trials: "
            + "; ".join(missing_step_files[:8])
        )
    return pd.DataFrame(rows)


def main() -> None:
    args = parse_args()
    summary_csv = args.run_root / "tables" / "summary_by_vw_controller.csv"
    summary = pd.read_csv(summary_csv)
    summary["v"] = pd.to_numeric(summary["v"], errors="coerce")
    summary["w"] = pd.to_numeric(summary["w"], errors="coerce")
    summary = summary[summary["controller"].isin([name for name, _, _ in CONTROLLERS])].copy()
    summary = summary[(summary["v"] - summary["w"]).abs() < 1e-9].copy()
    summary["kp"] = (summary["v"] / BASE_V).round().astype(int)
    kp_values = sorted(summary["kp"].unique().tolist())

    step_df = collect_step_metrics(args.run_root)
    step_df = step_df[step_df["controller"].isin([name for name, _, _ in CONTROLLERS])].copy()
    step_df = step_df[(step_df["v"] - step_df["w"]).abs() < 1e-9].copy()
    step_df["kp"] = (step_df["v"] / BASE_V).round().astype(int)

    grouped_steps = (
        step_df.groupby(["kp", "controller"], as_index=False)
        .agg(
            failure_rate_x=("min_surface_distance", lambda s: float((pd.to_numeric(s, errors="coerce") < args.failure_threshold_x).mean())),
            slack_num_mean=("slack_num", lambda s: float(pd.to_numeric(s, errors="coerce").mean())),
            min_surface_distance_mean=("min_surface_distance", lambda s: float(pd.to_numeric(s, errors="coerce").mean())),
        )
    )

    export = summary.merge(grouped_steps, on=["kp", "controller"], how="left")
    export_cols = [
        "kp",
        "controller",
        "seed_count",
        "tracking_rms_mean",
        "tracking_rms_p95",
        "solver_min_h_mean",
        "solver_slack_sum_mean",
        "failure_rate_x",
        "slack_num_mean",
        "min_surface_distance_mean",
        "core_time_p95_ms_mean",
        "core_time_worst_uav_p95_ms_max",
    ]

    out_prefix = args.out_prefix
    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    export[export_cols].sort_values(["kp", "controller"]).to_csv(
        out_prefix.with_suffix(".csv"), index=False
    )

    fig, axes = plt.subplots(1, 3, figsize=(13.0, 4.2), constrained_layout=True)

    for controller, label, color in CONTROLLERS:
        sub = export[export["controller"] == controller].sort_values("kp")
        axes[0].plot(
            sub["kp"],
            sub["failure_rate_x"] * 100.0,
            marker="o",
            linewidth=2.0,
            markersize=6.0,
            color=color,
            label=label,
        )
        axes[1].plot(
            sub["kp"],
            sub["tracking_rms_mean"],
            marker="o",
            linewidth=2.0,
            markersize=6.0,
            color=color,
            label=label,
        )
        axes[2].plot(
            sub["kp"],
            sub["solver_slack_sum_mean"],
            marker="o",
            linewidth=2.0,
            markersize=6.0,
            color=color,
            label=label,
        )

    axes[0].set_title(f"Failure Rate (x={args.failure_threshold_x:.3f} m)")
    axes[1].set_title("Tracking RMS Under CBF")
    axes[2].set_title("Mean Slack Sum")
    ylabels = ["Failure rate [%]", "Tracking RMS", "Slack sum"]

    for ax, ylabel in zip(axes, ylabels):
        ax.set_xlabel("KP")
        ax.set_ylabel(ylabel)
        ax.set_xticks(kp_values)
        ax.grid(True, alpha=0.25)

    axes[0].legend(frameon=False, loc="best")
    kp_label = ",".join(str(v) for v in kp_values)
    fig.suptitle(
        f"Figure-Eight Fixed Obstacles + CBF | KP={kp_label} | 100 Seeds | Two Non-Inertial Controllers"
    )
    fig.savefig(out_prefix.with_suffix(".png"), dpi=220, bbox_inches="tight")
    fig.savefig(out_prefix.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
