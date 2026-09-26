#!/usr/bin/env python3
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import pandas as pd


ROOT = Path(__file__).resolve().parent.parent
OUT_PREFIX = ROOT / "figures" / "paper_fig3_cbf_main"
CONTROLLERS = {
    "noninertial_frozen": ("Frozen", "#d95f02"),
    "noninertial_stage": ("Stage", "#1b9e77"),
}
KP_ORDER = [1, 2, 4]
KP_LABELS = ["Low", "Medium", "High"]
FAILURE_THRESHOLD = 0.215
SAFETY_MARGIN = 0.35


def load_trials() -> pd.DataFrame:
    df = pd.read_csv(
        ROOT
        / "results/figure_eight_fixedobs_cbf_kp124_seed1_100_2ctrl/tables/per_seed_controller_metrics.csv"
    )
    bad = df[~df["status"].isin(["ok", "skipped"])].copy()
    if not bad.empty:
        preview = ", ".join(
            f"{row.controller} kp={int(round(float(row.v) * 2))} seed={int(row.seed)} status={row.status}"
            for row in bad.head(8).itertuples()
        )
        raise RuntimeError(
            "Incomplete trials detected in the fixed-obstacle CBF dataset; "
            f"repair or rerun before plotting: {preview}"
        )
    df = df[df["status"].isin(["ok", "skipped"])].copy()
    df["kp"] = (df["v"] * 2).round().astype(int)
    df["min_surface_distance"] = df["solver_min_planar_clearance"] + SAFETY_MARGIN
    df["failure_x"] = (df["min_surface_distance"] < FAILURE_THRESHOLD).astype(float)
    return df


def mean_ci(series: pd.Series) -> Tuple[float, float, int]:
    series = series.dropna()
    n = len(series)
    mean = float(series.mean())
    ci95 = float(1.96 * series.std(ddof=1) / (n ** 0.5)) if n > 1 else 0.0
    return mean, ci95, n


def summarize(df: pd.DataFrame, col: str) -> pd.DataFrame:
    rows = []
    for ctrl in CONTROLLERS:
        sub = df[df["controller"] == ctrl]
        for kp in KP_ORDER:
            mean, ci95, n = mean_ci(sub[sub["kp"] == kp][col])
            rows.append(
                {
                    "controller": ctrl,
                    "kp": kp,
                    "mean": mean,
                    "ci95": ci95,
                    "n": n,
                }
            )
    return pd.DataFrame(rows)


def style_axis(ax, title: str, ylabel: str) -> None:
    ax.set_title(title, fontsize=11, pad=5)
    ax.set_xlabel("Operating condition", fontsize=10)
    ax.set_ylabel(ylabel, fontsize=10)
    ax.set_xticks(KP_ORDER, KP_LABELS)
    ax.tick_params(labelsize=9)
    ax.grid(True, axis="y", alpha=0.22, linewidth=0.7)
    ax.grid(False, axis="x")
    for spine in ax.spines.values():
        spine.set_linewidth(0.9)


def plot_panel(ax, summary: pd.DataFrame, panel_name: str) -> List[dict]:
    rows = []
    for ctrl, (label, color) in CONTROLLERS.items():
        sub = summary[summary["controller"] == ctrl].sort_values("kp")
        ax.errorbar(
            sub["kp"],
            sub["mean"],
            yerr=sub["ci95"],
            color=color,
            label=label,
            marker="o",
            markersize=5.5,
            linewidth=2.0,
            elinewidth=1.0,
            capsize=3,
            capthick=1.0,
        )
        for _, r in sub.iterrows():
            rows.append(
                {
                    "figure": "fig3",
                    "panel": panel_name,
                    "kp": int(r["kp"]),
                    "controller": ctrl,
                    "mean": float(r["mean"]),
                    "ci95": float(r["ci95"]),
                    "seed_count": int(r["n"]),
                }
            )
    return rows


def main() -> None:
    plt.rcParams.update(
        {
            "font.size": 9,
            "axes.titlesize": 11,
            "axes.labelsize": 10,
            "legend.fontsize": 9,
        }
    )

    df = load_trials()
    failure = summarize(df, "failure_x")
    tracking = summarize(df, "tracking_rms")
    slack = summarize(df, "solver_slack_sum")

    fig, axes = plt.subplots(1, 3, figsize=(10.8, 2.95), constrained_layout=True)
    rows = []
    rows.extend(plot_panel(axes[0], failure, "safety_violation_rate"))
    rows.extend(plot_panel(axes[1], tracking, "tracking_rms"))
    rows.extend(plot_panel(axes[2], slack, "slack_sum"))

    style_axis(axes[0], "(a) Safety-violation rate", "Violation rate")
    style_axis(axes[1], "(b) Tracking RMS", "Tracking RMS [m]")
    style_axis(axes[2], "(c) Slack sum", "Slack sum")
    axes[0].set_ylim(0.0, 1.08)
    axes[0].legend(frameon=False, loc="upper left", ncol=2)

    OUT_PREFIX.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(OUT_PREFIX.with_suffix(".csv"), index=False)
    fig.savefig(OUT_PREFIX.with_suffix(".png"), dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PREFIX.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
