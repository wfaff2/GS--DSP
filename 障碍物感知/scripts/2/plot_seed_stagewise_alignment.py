#!/usr/bin/env python3
"""Plot a per-seed three-axis stage-vs-frozen alignment figure."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd
import seaborn as sns


def place_panel_legend(ax, handles, labels, ncol=2, y=1.08) -> None:
    if not handles:
        return
    ax.legend(
        handles,
        labels,
        loc="lower center",
        bbox_to_anchor=(0.5, y),
        ncol=ncol,
        framealpha=0.92,
        fontsize=9,
        columnspacing=1.2,
        handlelength=2.2,
        borderaxespad=0.2,
    )


def compute_norm(df: pd.DataFrame, preferred_cols: tuple[str, str, str], fallback_cols: tuple[str, str, str]) -> pd.Series:
    if all(col in df.columns for col in preferred_cols):
        x_col, y_col, z_col = preferred_cols
    elif all(col in df.columns for col in fallback_cols):
        x_col, y_col, z_col = fallback_cols
    else:
        return pd.Series(np.nan, index=df.index, dtype=float)
    return np.sqrt(df[x_col] ** 2 + df[y_col] ** 2 + df[z_col] ** 2)


def load_step_summary(csv_path: Path, method: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    if "frame_mode_effective" in df.columns:
        df = df[df["frame_mode_effective"] == "noninertial"].copy()

    df["car_a_norm"] = compute_norm(
        df,
        ("car_a_world_x", "car_a_world_y", "car_a_world_z"),
        ("a_car_non_x", "a_car_non_y", "a_car_non_z"),
    )
    df["car_omega_norm"] = compute_norm(
        df,
        ("car_omega_world_x", "car_omega_world_y", "car_omega_world_z"),
        ("omega_non_x", "omega_non_y", "omega_non_z"),
    )
    if "solver_planar_surface_distance" in df.columns:
        df["obstacle_surface_distance"] = df["solver_planar_surface_distance"]
    elif "solver_planar_clearance" in df.columns:
        df["obstacle_surface_distance"] = df["solver_planar_clearance"]
    else:
        df["obstacle_surface_distance"] = np.nan

    grouped = (
        df.groupby("sim_time", as_index=False)
        .agg(
            tracking_error=("tracking_error", "max"),
            obstacle_surface_distance=("obstacle_surface_distance", "min"),
            car_a_norm=("car_a_norm", "mean"),
            car_omega_norm=("car_omega_norm", "mean"),
        )
        .sort_values("sim_time")
        .reset_index(drop=True)
    )
    grouped["method"] = method
    return grouped


def load_horizon_summary(csv_path: Path, method: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    if "frame_mode_effective" in df.columns:
        df = df[df["frame_mode_effective"] == "noninertial"].copy()
    grouped = (
        df.groupby("sim_time", as_index=False)
        .agg(
            horizon_rms_xy=("horizon_rms_xy", "mean"),
            horizon_max_xy=("horizon_max_xy", "mean"),
            horizon_final_xy=("horizon_final_xy", "mean"),
        )
        .sort_values("sim_time")
        .reset_index(drop=True)
    )
    grouped["method"] = method
    return grouped


def smooth_columns(df: pd.DataFrame, columns: list[str], window_size: int) -> pd.DataFrame:
    smoothed = df.sort_values(["method", "sim_time"]).copy()
    for column in columns:
        smoothed[f"{column}_smooth"] = smoothed.groupby("method")[column].transform(
            lambda values: values.rolling(window_size, min_periods=1, center=True).mean()
        )
    return smoothed


def plot_highlight_regions(ax_list, sim_time: np.ndarray, accel_values: np.ndarray, accel_thr: float) -> None:
    if sim_time.size < 2:
        return
    mask = accel_values > accel_thr
    if not np.any(mask):
        return
    step = np.median(np.diff(sim_time))
    start_idx = None
    for idx, active in enumerate(mask):
        if active and start_idx is None:
            start_idx = idx
        elif not active and start_idx is not None:
            start_t = sim_time[start_idx]
            end_t = sim_time[idx - 1] + 0.5 * step
            for ax in ax_list:
                ax.axvspan(start_t, end_t, color="red", alpha=0.08)
            start_idx = None
    if start_idx is not None:
        start_t = sim_time[start_idx]
        end_t = sim_time[-1] + 0.5 * step
        for ax in ax_list:
            ax.axvspan(start_t, end_t, color="red", alpha=0.08)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Per-seed stage-vs-frozen three-axis alignment plot")
    parser.add_argument("--stage-step-csv", required=True)
    parser.add_argument("--frozen-step-csv", required=True)
    parser.add_argument("--stage-horizon-csv", required=True)
    parser.add_argument("--frozen-horizon-csv", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--title", default="")
    parser.add_argument("--accel-thr", type=float, default=0.8)
    parser.add_argument("--window-size", type=int, default=15)
    return parser.parse_args()


def resolve_existing_csv(csv_path: Path) -> Path:
    if csv_path.is_file():
        return csv_path
    if csv_path.name == "noninertial_frozen_metrics_steps.csv":
        fallback = csv_path.with_name("noninertial_metrics_steps.csv")
        if fallback.is_file():
            return fallback
    raise FileNotFoundError(f"Missing CSV: {csv_path}")


def main() -> int:
    args = parse_args()
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    stage_steps = load_step_summary(resolve_existing_csv(Path(args.stage_step_csv)), "stage")
    frozen_steps = load_step_summary(resolve_existing_csv(Path(args.frozen_step_csv)), "frozen")
    stage_horizon = load_horizon_summary(Path(args.stage_horizon_csv), "stage")
    frozen_horizon = load_horizon_summary(Path(args.frozen_horizon_csv), "frozen")

    merged = pd.concat(
        [
            pd.merge(stage_steps, stage_horizon, on=["sim_time", "method"], how="inner"),
            pd.merge(frozen_steps, frozen_horizon, on=["sim_time", "method"], how="inner"),
        ],
        ignore_index=True,
        sort=False,
    ).sort_values(["method", "sim_time"])
    if merged.empty:
        raise SystemExit("No rows available after merging step and horizon CSVs.")

    merged = smooth_columns(
        merged,
        [
            "tracking_error",
            "obstacle_surface_distance",
            "car_a_norm",
            "car_omega_norm",
            "horizon_rms_xy",
        ],
        args.window_size,
    )

    ugv_ref = (
        merged.groupby("sim_time", as_index=False)
        .agg(
            car_a_norm=("car_a_norm_smooth", "max"),
            car_omega_norm=("car_omega_norm_smooth", "max"),
        )
        .sort_values("sim_time")
        .reset_index(drop=True)
    )

    sns.set_theme(style="whitegrid", context="paper", font_scale=1.15)
    plt.rcParams.update(
        {
            "font.family": "DejaVu Serif",
            "axes.linewidth": 1.1,
            "lines.linewidth": 1.6,
        }
    )

    palette = {"stage": "#1f77b4", "frozen": "#d62728"}
    fig, (ax1, ax2, ax3) = plt.subplots(
        3,
        1,
        figsize=(10.5, 12.2),
        sharex=True,
        gridspec_kw={"height_ratios": [1, 1, 1.15]},
    )

    # Top panel: tracking error only
    top_handles: list[plt.Artist] = []
    top_labels: list[str] = []
    for method in ("frozen", "stage"):
        df_m = merged[merged["method"] == method].sort_values("sim_time")
        if df_m.empty:
            continue
        track_handle = ax1.plot(
            df_m["sim_time"].to_numpy(),
            df_m["tracking_error_smooth"].to_numpy(),
            color=palette[method],
            label=f"{method} tracking",
        )[0]
        top_handles.append(track_handle)
        top_labels.append(f"{method} tracking")
    ax1.set_title("Top: Worst-UAV Tracking Error", pad=10)
    ax1.set_ylabel("Tracking Error [m]")
    ax1.set_ylim(bottom=0.0)
    place_panel_legend(ax1, top_handles, top_labels, ncol=2, y=1.10)

    # Middle panel: UGV dynamics
    if not ugv_ref.empty:
        accel_handle = ax2.plot(
            ugv_ref["sim_time"].to_numpy(),
            ugv_ref["car_a_norm"].to_numpy(),
            color="purple",
            label=r"$|\mathbf{a}_{car,world}|$",
        )[0]
        ax2_twin = ax2.twinx()
        omega_handle = ax2_twin.plot(
            ugv_ref["sim_time"].to_numpy(),
            ugv_ref["car_omega_norm"].to_numpy(),
            color="orange",
            label=r"$|\omega_{car,world}|$",
        )[0]
        ax2.legend([accel_handle, omega_handle], [accel_handle.get_label(), omega_handle.get_label()], loc="upper left")
        ax2_twin.set_ylabel(r"$|\omega_{car,world}|$ [rad/s]")
        ax2_twin.set_ylim(bottom=0.0)
        ax2_twin.grid(False)
        plot_highlight_regions([ax1, ax2, ax3], ugv_ref["sim_time"].to_numpy(), ugv_ref["car_a_norm"].to_numpy(), args.accel_thr)
        ax2.legend_.remove()
        place_panel_legend(
            ax2,
            [accel_handle, omega_handle],
            [accel_handle.get_label(), omega_handle.get_label()],
            ncol=2,
            y=1.10,
        )
    ax2.set_title("Middle: UGV Maneuver Dynamics", pad=10)
    ax2.set_ylabel(r"$|\mathbf{a}_{car,world}|$ [m/s$^2$]")
    ax2.set_ylim(bottom=0.0)

    # Bottom panel: horizon RMS
    bottom_handles: list[plt.Artist] = []
    for method in ("frozen", "stage"):
        df_m = merged[merged["method"] == method].sort_values("sim_time")
        if df_m.empty:
            continue
        handle = ax3.plot(
            df_m["sim_time"].to_numpy(),
            df_m["horizon_rms_xy_smooth"].to_numpy(),
            color=palette[method],
            label=method,
        )[0]
        bottom_handles.append(handle)
    ax3.set_title("Bottom: Per-Seed Horizon RMS Error", pad=10)
    ax3.set_ylabel(r"Horizon $e_{rms}$ [m]")
    ax3.set_xlabel("Simulation Time [s]")
    ax3.set_ylim(bottom=0.0)
    place_panel_legend(ax3, bottom_handles, [h.get_label() for h in bottom_handles], ncol=2, y=1.10)

    title = args.title.strip()
    if title:
        fig.suptitle(title, y=0.995, fontsize=14)
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.970), h_pad=2.6)
    else:
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.980), h_pad=2.6)

    out_path = Path(args.out).expanduser().resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=220, bbox_inches="tight")
    plt.close(fig)
    print(f"saved seed three-axis plot: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
