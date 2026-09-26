#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Patch
from matplotlib.ticker import PercentFormatter

BLUE = "#0072B2"
RED = "#C23B22"
GRAY = "#7A7A7A"
LIGHT_RED = "#F2C9C1"
LIGHT_BLUE = "#CFE6F3"
GRID = "#D6DADF"
MUTED = "#5B6570"
SHADE = "#E8EDF2"


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description=(
            "Plot time-composition and duration-distribution summaries for "
            "episodes where Stage has prediction disadvantage and tracking disadvantage."
        )
    )
    parser.add_argument(
        "--stage-root",
        type=Path,
        default=repo_root / "results" / "codex_stage_omega_with_cbf_kpgrid_20seed_20260403",
        help="Root directory containing the stage rollout kp sweep.",
    )
    parser.add_argument(
        "--frozen-root",
        type=Path,
        default=repo_root / "results" / "codex_frozen_backfill_for_stageomega_with_cbf_kpgrid_20seed_20260403",
        help="Root directory containing the frozen rollout kp sweep.",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=2.5,
        help="k_p slice used for the figure.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=repo_root / "results" / "new" / "ieee_publication_figures",
        help="Directory for output CSV and figures.",
    )
    return parser.parse_args()


def kp_dir_name(kp: float) -> str:
    return f"kp_{str(kp).replace('.', 'p')}"


def load_combined(root: Path, kp: float) -> pd.DataFrame:
    csv_path = root / kp_dir_name(kp) / "combined_robustness_data.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"Missing combined robustness CSV: {csv_path}")
    df = pd.read_csv(csv_path)
    if "frame_mode_effective" in df.columns:
        df = df[df["frame_mode_effective"] == "noninertial"].copy()
    keep = ["seed", "sim_time", "tracking_error", "horizon_rms_xy"]
    df = df[keep].copy()
    for column in keep:
        df[column] = pd.to_numeric(df[column], errors="coerce")
    df["seed"] = df["seed"].astype(int)
    return df.sort_values(["seed", "sim_time"]).reset_index(drop=True)


def merge_stage_frozen(stage_root: Path, frozen_root: Path, kp: float) -> pd.DataFrame:
    stage = load_combined(stage_root, kp).rename(
        columns={
            "tracking_error": "tracking_error_stage",
            "horizon_rms_xy": "horizon_rms_xy_stage",
        }
    )
    frozen = load_combined(frozen_root, kp).rename(
        columns={
            "tracking_error": "tracking_error_frozen",
            "horizon_rms_xy": "horizon_rms_xy_frozen",
        }
    )
    merged = frozen.merge(stage, on=["seed", "sim_time"], how="inner")
    merged = merged.sort_values(["seed", "sim_time"]).reset_index(drop=True)
    merged["pred_disadv"] = merged["horizon_rms_xy_frozen"] < merged["horizon_rms_xy_stage"]
    merged["track_disadv"] = merged["tracking_error_frozen"] < merged["tracking_error_stage"]
    merged["both_disadv"] = merged["pred_disadv"] & merged["track_disadv"]
    return merged


def nominal_dt(time_values: np.ndarray) -> float:
    if time_values.size <= 1:
        return 0.0
    diffs = np.diff(time_values)
    positive = diffs[diffs > 0.0]
    return float(np.median(positive)) if positive.size else 0.0


def build_episode_metrics(merged: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, float | int]] = []
    for seed, sub in merged.groupby("seed", sort=True):
        sub = sub.sort_values("sim_time").reset_index(drop=True)
        time_values = sub["sim_time"].to_numpy(dtype=float)
        mask = sub["both_disadv"].to_numpy(dtype=bool)
        dt_nom = nominal_dt(time_values)
        start_idx: int | None = None

        def flush(end_idx: int) -> None:
            assert start_idx is not None
            episode = sub.iloc[start_idx : end_idx + 1].copy()
            stage_track = episode["tracking_error_stage"].to_numpy(dtype=float)
            frozen_track = episode["tracking_error_frozen"].to_numpy(dtype=float)
            stage_pred = episode["horizon_rms_xy_stage"].to_numpy(dtype=float)
            frozen_pred = episode["horizon_rms_xy_frozen"].to_numpy(dtype=float)
            rows.append(
                {
                    "seed": int(seed),
                    "start_time_s": float(episode["sim_time"].iloc[0]),
                    "end_time_s": float(episode["sim_time"].iloc[-1]),
                    "duration_s": float((episode["sim_time"].iloc[-1] - episode["sim_time"].iloc[0]) + dt_nom),
                    "num_samples": int(len(episode)),
                    "stage_tracking_mean_m": float(np.mean(stage_track)),
                    "stage_tracking_rms_m": float(np.sqrt(np.mean(np.square(stage_track)))),
                    "stage_tracking_max_m": float(np.max(stage_track)),
                    "frozen_tracking_mean_m": float(np.mean(frozen_track)),
                    "frozen_tracking_rms_m": float(np.sqrt(np.mean(np.square(frozen_track)))),
                    "frozen_tracking_max_m": float(np.max(frozen_track)),
                    "track_gap_stage_minus_frozen_m": float(np.mean(stage_track - frozen_track)),
                    "pred_gap_stage_minus_frozen_m": float(np.mean(stage_pred - frozen_pred)),
                }
            )

        for idx in range(len(sub)):
            if not mask[idx]:
                if start_idx is not None:
                    flush(idx - 1)
                    start_idx = None
                continue
            contiguous = (
                idx > 0
                and mask[idx - 1]
                and (time_values[idx] - time_values[idx - 1]) <= 1.5 * dt_nom + 1e-12
            )
            if start_idx is None:
                start_idx = idx
            elif not contiguous:
                flush(idx - 1)
                start_idx = idx
        if start_idx is not None:
            flush(len(sub) - 1)

    if not rows:
        raise RuntimeError("No prediction-disadvantage + tracking-disadvantage episodes found.")
    return pd.DataFrame(rows).sort_values(["seed", "start_time_s"]).reset_index(drop=True)


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": ["Helvetica", "Arial", "DejaVu Sans"],
            "mathtext.fontset": "dejavusans",
            "font.size": 7.4,
            "axes.labelsize": 7.8,
            "axes.titlesize": 8.2,
            "xtick.labelsize": 6.8,
            "ytick.labelsize": 6.8,
            "legend.fontsize": 6.6,
            "axes.linewidth": 0.75,
            "lines.linewidth": 1.55,
            "lines.markersize": 4.2,
            "figure.dpi": 170,
            "savefig.dpi": 320,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "axes.unicode_minus": False,
        }
    )


def stylize_axes(ax, grid_axis: str = "y", hide_right: bool = True) -> None:
    ax.spines["top"].set_visible(False)
    if hide_right:
        ax.spines["right"].set_visible(False)
    ax.spines["left"].set_linewidth(0.75)
    ax.spines["bottom"].set_linewidth(0.75)
    ax.tick_params(direction="out", length=2.4, width=0.75, pad=2)
    ax.grid(axis=grid_axis, color=GRID, linewidth=0.55, alpha=0.55)
    ax.set_axisbelow(True)


def panel_title(ax, label: str, title: str) -> None:
    ax.text(
        0.0,
        1.015,
        f"({label}) {title}",
        transform=ax.transAxes,
        ha="left",
        va="bottom",
        fontsize=8.0,
        fontweight="semibold",
    )


def draw_segment_label(ax, x_left: float, width: float, y_center: float, edge_color: str) -> None:
    label = f"{width:.2f}%"
    x_center = x_left + 0.5 * width
    if width >= 8.0:
        ax.text(
            x_center,
            y_center,
            label,
            ha="center",
            va="center",
            color=edge_color,
            fontsize=6.9 if width >= 12.0 else 6.1,
            fontweight="semibold",
        )
        return
    ax.text(
        x_center,
        y_center + 0.33,
        label,
        ha="center",
        va="bottom",
        color=edge_color,
        fontsize=6.7,
        fontweight="semibold",
        clip_on=False,
    )


def compute_summary_metrics(merged: pd.DataFrame, episodes: pd.DataFrame, kp: float) -> dict[str, float]:
    pred_disadv_mask = merged["pred_disadv"].to_numpy(dtype=bool)
    track_disadv_mask = merged["track_disadv"].to_numpy(dtype=bool)
    both_disadv_mask = merged["both_disadv"].to_numpy(dtype=bool)
    durations = episodes["duration_s"].to_numpy(dtype=float)

    pred_disadv_pct = 100.0 * float(np.mean(pred_disadv_mask))
    joint_disadvantage_pct = 100.0 * float(np.mean(both_disadv_mask))
    pred_disadv_but_track_better_pct = 100.0 * float(np.mean(pred_disadv_mask & ~track_disadv_mask))

    if np.any(pred_disadv_mask):
        tracking_adv_within_pred_disadv_pct = 100.0 * float(
            np.mean(
                merged.loc[pred_disadv_mask, "tracking_error_stage"].to_numpy(dtype=float)
                < merged.loc[pred_disadv_mask, "tracking_error_frozen"].to_numpy(dtype=float)
            )
        )
    else:
        tracking_adv_within_pred_disadv_pct = np.nan

    return {
        "kp": kp,
        "num_samples": float(len(merged)),
        "num_episodes": float(len(episodes)),
        "prediction_disadvantage_pct": pred_disadv_pct,
        "prediction_advantage_or_equal_pct": 100.0 - pred_disadv_pct,
        "prediction_disadvantage_but_tracking_better_pct": pred_disadv_but_track_better_pct,
        "joint_disadvantage_pct": joint_disadvantage_pct,
        "tracking_advantage_within_prediction_disadvantage_pct": tracking_adv_within_pred_disadv_pct,
        "duration_mean_s": float(np.mean(durations)),
        "duration_std_s": float(np.std(durations, ddof=1)),
        "duration_median_s": float(np.median(durations)),
        "duration_max_s": float(np.max(durations)),
        "duration_leq_0p10_pct": 100.0 * float(np.mean(durations <= 0.10)),
        "duration_leq_0p20_pct": 100.0 * float(np.mean(durations <= 0.20)),
    }


def plot_joint_disadvantage_summary(
    merged: pd.DataFrame,
    episodes: pd.DataFrame,
    out_dir: Path,
    kp: float,
) -> None:
    configure_style()
    summary = compute_summary_metrics(merged, episodes, kp)
    fig, (ax_left, ax_right) = plt.subplots(
        1,
        2,
        figsize=(7.6, 3.0),
        gridspec_kw={"width_ratios": [1.02, 1.18]},
    )

    better_or_equal_pred = summary["prediction_advantage_or_equal_pct"]
    worse_pred_better_track = summary["prediction_disadvantage_but_tracking_better_pct"]
    joint_disadvantage = summary["joint_disadvantage_pct"]
    left_stage = 0.0
    left_protected = better_or_equal_pred
    left_joint = better_or_equal_pred + worse_pred_better_track
    y_bar = 0.0
    bar_height = 0.38

    ax_left.barh(
        y_bar,
        better_or_equal_pred,
        height=bar_height,
        left=left_stage,
        color=LIGHT_BLUE,
        edgecolor=BLUE,
        linewidth=0.9,
    )
    ax_left.barh(
        y_bar,
        worse_pred_better_track,
        height=bar_height,
        left=left_protected,
        color=SHADE,
        edgecolor=GRAY,
        linewidth=0.9,
    )
    ax_left.barh(
        y_bar,
        joint_disadvantage,
        height=bar_height,
        left=left_joint,
        color=LIGHT_RED,
        edgecolor=RED,
        linewidth=0.9,
    )

    draw_segment_label(ax_left, left_stage, better_or_equal_pred, y_bar, BLUE)
    draw_segment_label(ax_left, left_protected, worse_pred_better_track, y_bar, GRAY)
    draw_segment_label(ax_left, left_joint, joint_disadvantage, y_bar, RED)

    panel_title(ax_left, "a", "Time Composition of Local Disadvantage")
    stylize_axes(ax_left, grid_axis="x")
    ax_left.spines["left"].set_visible(False)
    ax_left.set_xlim(0.0, 100.0)
    ax_left.set_ylim(-0.58, 0.58)
    ax_left.set_yticks([])
    ax_left.xaxis.set_major_formatter(PercentFormatter(xmax=100.0, decimals=0))
    ax_left.set_xlabel("Fraction of simulated time")

    durations = episodes["duration_s"].to_numpy(dtype=float)
    max_duration = float(np.max(durations))
    duration_limit = max(1.15, max_duration * 1.02)
    durations_sorted = np.sort(durations)
    cdf = 100.0 * np.arange(1, len(durations_sorted) + 1) / len(durations_sorted)
    ax_right.step(
        durations_sorted,
        cdf,
        where="post",
        color=BLUE,
        linewidth=1.8,
    )
    median_value = summary["duration_median_s"]
    mean_value = summary["duration_mean_s"]
    ax_right.axvline(
        median_value,
        color=BLUE,
        linestyle="-",
        linewidth=1.6,
    )
    ax_right.axvline(
        mean_value,
        color=RED,
        linestyle="--",
        linewidth=1.6,
    )

    panel_title(ax_right, "b", f"Episode Duration CDF (N={len(episodes)})")
    stylize_axes(ax_right, grid_axis="y")
    ax_right.set_xlim(0.0, duration_limit)
    ax_right.set_xlabel("Joint-disadvantage episode duration [s]")
    ax_right.set_ylabel("Cumulative fraction of episodes [%]")
    ax_right.set_ylim(0.0, 100.0)
    ax_right.yaxis.set_major_formatter(PercentFormatter(xmax=100.0, decimals=0))
    legend_lines = [
        plt.Line2D([0], [0], color=BLUE, linestyle="-", linewidth=1.8),
        plt.Line2D([0], [0], color=BLUE, linestyle="-", linewidth=1.6),
        plt.Line2D([0], [0], color=RED, linestyle="--", linewidth=1.6),
    ]
    legend_labels = [
        "Empirical CDF",
        f"Median: {median_value:.4f} s",
        f"Mean: {mean_value:.4f} s",
    ]
    ax_right.legend(
        legend_lines,
        legend_labels,
        loc="lower right",
        bbox_to_anchor=(0.985, 0.03),
        frameon=False,
        handlelength=2.3,
        borderpad=0.2,
        labelspacing=0.25,
    )

    out_dir.mkdir(parents=True, exist_ok=True)
    stem = "fig_c2_joint_disadvantage_summary_cdf"
    png_path = out_dir / f"{stem}.png"
    pdf_path = out_dir / f"{stem}.pdf"
    summary_csv_path = out_dir / f"{stem}_stats.csv"
    pd.DataFrame([summary]).to_csv(summary_csv_path, index=False)
    fig.legend(
        [
            Patch(facecolor=LIGHT_BLUE, edgecolor=BLUE, linewidth=0.9),
            Patch(facecolor=SHADE, edgecolor=GRAY, linewidth=0.9),
            Patch(facecolor=LIGHT_RED, edgecolor=RED, linewidth=0.9),
        ],
        [
            "Better/equal RMSE",
            "Worse RMSE, better tracking",
            "Worse in both",
        ],
        loc="upper center",
        bbox_to_anchor=(0.5, 1.02),
        ncol=3,
        frameon=False,
        columnspacing=1.2,
        handlelength=1.6,
        handletextpad=0.5,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.90), w_pad=1.4)
    fig.savefig(png_path, bbox_inches="tight")
    fig.savefig(pdf_path, bbox_inches="tight")
    plt.close(fig)
    print(f"[INFO] wrote {png_path}")
    print(f"[INFO] wrote {pdf_path}")
    print(f"[INFO] wrote {summary_csv_path}")


def main() -> None:
    args = parse_args()
    merged = merge_stage_frozen(args.stage_root, args.frozen_root, args.kp)
    episodes = build_episode_metrics(merged)
    out_csv = args.out_dir / "fig3_pred_disadv_track_disadv_episode_metrics.csv"
    args.out_dir.mkdir(parents=True, exist_ok=True)
    episodes.to_csv(out_csv, index=False)
    print(f"[INFO] wrote {out_csv}")
    plot_joint_disadvantage_summary(merged, episodes, args.out_dir, args.kp)


if __name__ == "__main__":
    main()
