#!/usr/bin/env python3
from __future__ import annotations

import argparse
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
        description="Plot clean figure-eight no-CBF KP=1,2,4 seed1-100 two-controller summary."
    )
    parser.add_argument("--summary-csv", type=Path, required=True)
    parser.add_argument("--out-prefix", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    df = pd.read_csv(args.summary_csv)

    df = df[df["controller"].isin([name for name, _, _ in CONTROLLERS])].copy()
    df = df[(df["v"] - df["w"]).abs() < 1e-9].copy()
    df["kp"] = (df["v"] / BASE_V).round().astype(int)
    df = df.sort_values(["kp", "controller"]).reset_index(drop=True)
    kp_values = sorted(df["kp"].unique().tolist())

    out_prefix = args.out_prefix
    out_prefix.parent.mkdir(parents=True, exist_ok=True)

    export_cols = [
        "kp",
        "controller",
        "seed_count",
        "tracking_rms_mean",
        "tracking_rms_p95",
        "prediction_horizon_rms_xy_mean",
        "prediction_horizon_rms_xy_p95",
        "core_time_p95_ms_mean",
        "core_time_worst_uav_p95_ms_max",
    ]
    df[export_cols].to_csv(out_prefix.with_suffix(".csv"), index=False)

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.2), constrained_layout=True)

    for controller, label, color in CONTROLLERS:
        sub = df[df["controller"] == controller].sort_values("kp")
        axes[0].plot(
            sub["kp"],
            sub["tracking_rms_mean"],
            marker="o",
            linewidth=2.0,
            markersize=6.0,
            color=color,
            label=label,
        )
        axes[1].plot(
            sub["kp"],
            sub["prediction_horizon_rms_xy_mean"],
            marker="o",
            linewidth=2.0,
            markersize=6.0,
            color=color,
            label=label,
        )

    axes[0].set_title("Mean Tracking RMS")
    axes[1].set_title("Mean Prediction RMS")
    for ax in axes:
        ax.set_xlabel("KP")
        ax.set_xticks(kp_values)
        ax.grid(True, alpha=0.25)
    axes[0].set_ylabel("RMS")
    axes[1].set_ylabel("RMS")
    axes[0].legend(frameon=False, loc="best")

    kp_label = ",".join(str(v) for v in kp_values)
    fig.suptitle(
        f"Clean Figure-Eight No-CBF | KP={kp_label} | 100 Seeds | Two Non-Inertial Controllers"
    )
    fig.savefig(out_prefix.with_suffix(".png"), dpi=220, bbox_inches="tight")
    fig.savefig(out_prefix.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
