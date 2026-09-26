#!/usr/bin/env python3
from pathlib import Path
from typing import List

import matplotlib.pyplot as plt
import pandas as pd


ROOT = Path(__file__).resolve().parent.parent
OUT_PREFIX = ROOT / "figures" / "paper_fig2_pure_tracking"
CONTROLLERS = {
    "noninertial_frozen": ("Frozen", "#d95f02"),
    "noninertial_stage": ("Stage", "#1b9e77"),
}
KP_ORDER = [1, 2, 4]
KP_LABELS = ["Low", "Medium", "High"]


def load_trials(rel_path: str) -> pd.DataFrame:
    df = pd.read_csv(ROOT / rel_path)
    df["kp"] = (df["v"] * 2).round().astype(int)
    bad = df[~df["status"].isin(["ok", "skipped"])].copy()
    if not bad.empty:
        preview = ", ".join(
            f"{row.controller} kp={int(row.kp)} seed={int(row.seed)} status={row.status}"
            for row in bad.head(8).itertuples()
        )
        raise RuntimeError(
            f"Incomplete trials detected in {rel_path}; repair or rerun before plotting: {preview}"
        )
    return df[df["status"].isin(["ok", "skipped"])].copy()


def summarize(df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for ctrl in CONTROLLERS:
        sub = df[df["controller"] == ctrl]
        for kp in KP_ORDER:
            grp = sub[sub["kp"] == kp]["tracking_rms"].dropna()
            n = len(grp)
            mean = float(grp.mean())
            ci95 = float(1.96 * grp.std(ddof=1) / (n ** 0.5)) if n > 1 else 0.0
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


def style_axis(ax, title: str) -> None:
    ax.set_title(title, fontsize=11, pad=5)
    ax.set_xlabel("Operating condition", fontsize=10)
    ax.set_ylabel("Tracking RMS [m]", fontsize=10)
    ax.set_xticks(KP_ORDER, KP_LABELS)
    ax.tick_params(labelsize=9)
    ax.grid(True, axis="y", alpha=0.22, linewidth=0.7)
    ax.grid(False, axis="x")
    for spine in ax.spines.values():
        spine.set_linewidth(0.9)


def plot_panel(ax, summary: pd.DataFrame) -> List[dict]:
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
                    "figure": "fig2",
                    "kp": int(r["kp"]),
                    "controller": ctrl,
                    "mean_tracking_rms": float(r["mean"]),
                    "ci95_tracking_rms": float(r["ci95"]),
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

    circle = summarize(
        load_trials(
            "results/circle_nocbf_kp124_seed1_100_2ctrl/tables/per_seed_controller_metrics.csv"
        )
    )
    fig8 = summarize(
        load_trials(
            "results/figure_eight_clean_nocbf_kp124_seed1_100_2ctrl/tables/per_seed_controller_metrics.csv"
        )
    )

    fig, axes = plt.subplots(1, 2, figsize=(7.2, 2.9), constrained_layout=True)
    rows = []
    rows.extend(plot_panel(axes[0], circle))
    rows.extend(plot_panel(axes[1], fig8))
    style_axis(axes[0], "(a) Circle")
    style_axis(axes[1], "(b) Figure-eight")
    axes[0].legend(frameon=False, loc="upper left", ncol=2)

    OUT_PREFIX.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(OUT_PREFIX.with_suffix(".csv"), index=False)
    fig.savefig(OUT_PREFIX.with_suffix(".png"), dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PREFIX.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
