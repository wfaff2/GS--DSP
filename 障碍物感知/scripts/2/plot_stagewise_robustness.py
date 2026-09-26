#!/usr/bin/env python3
"""
CoNi-MPC Performance Analysis Script
Generates a Three-Axis Alignment Plot and a Statistical Robustness Table
comparing Stage-wise (Ours) vs. Frozen (Baseline) non-inertial MPC rollouts.
"""

import argparse
import pathlib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import signal


def place_panel_legend(ax, handles, labels, ncol=2, y=1.08):
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


def compute_phase_lag(time, sig_ref, sig_response, max_lag_sec=2.0):
    """Compute phase lag between a reference signal and a response."""
    time = np.asarray(time, dtype=float)
    sig_ref = np.asarray(sig_ref, dtype=float)
    sig_response = np.asarray(sig_response, dtype=float)
    if time.size < 3:
        return np.nan

    order = np.argsort(time)
    time = time[order]
    sig_ref = sig_ref[order]
    sig_response = sig_response[order]
    unique_time, unique_idx = np.unique(time, return_index=True)
    if unique_time.size < 3:
        return np.nan

    sig_ref = sig_ref[unique_idx]
    sig_response = sig_response[unique_idx]
    dt = np.median(np.diff(unique_time))
    if not np.isfinite(dt) or dt <= 0.0:
        return np.nan

    uniform_time = np.arange(unique_time[0], unique_time[-1] + 0.5 * dt, dt)
    ref_uniform = np.interp(uniform_time, unique_time, sig_ref)
    resp_uniform = np.interp(uniform_time, unique_time, sig_response)
    ref_centered = ref_uniform - np.mean(ref_uniform)
    resp_centered = resp_uniform - np.mean(resp_uniform)
    if np.std(ref_centered) < 1e-9 or np.std(resp_centered) < 1e-9:
        return np.nan

    correlation = signal.correlate(resp_centered, ref_centered, mode="full")
    lags = signal.correlation_lags(len(resp_centered), len(ref_centered), mode="full")
    if max_lag_sec is not None and np.isfinite(max_lag_sec) and max_lag_sec > 0.0:
        max_lag_steps = int(round(max_lag_sec / dt))
        keep_mask = np.abs(lags) <= max_lag_steps
        correlation = correlation[keep_mask]
        lags = lags[keep_mask]
        if lags.size == 0:
            return np.nan
    lag_idx = lags[np.argmax(correlation)]
    return lag_idx * dt

def identify_high_g_regions(df, accel_threshold=0.8):
    """Identify contiguous regions where UGV acceleration is high."""
    high_g = df[df['car_a_norm'] > accel_threshold]
    regions = []
    if high_g.empty:
        return regions
    
    times = high_g['sim_time'].values
    step = np.median(np.diff(df['sim_time']))
    
    start_t = times[0]
    prev_t = times[0]
    for t in times[1:]:
        if t - prev_t > step * 5:  # gap detected
            regions.append((start_t, prev_t))
            start_t = t
        prev_t = t
    regions.append((start_t, prev_t))
    return regions


def build_time_region_mask(time_values, regions):
    """Map a list of [start, end] regions back to a boolean mask over time."""
    time_values = np.asarray(time_values, dtype=float)
    if time_values.size == 0 or not regions:
        return np.zeros(time_values.shape, dtype=bool)
    mask = np.zeros(time_values.shape, dtype=bool)
    for start_t, end_t in regions:
        mask |= (time_values >= start_t) & (time_values <= end_t)
    return mask

def format_mean_std(data):
    """Format mean and std gracefully handling single-sample NaN std."""
    valid_data = [x for x in data if not np.isnan(x)]
    if not valid_data:
        return "N/A"
    if len(valid_data) == 1:
        return f"{valid_data[0]:.4f} ± 0.0000"
    return f"{np.mean(valid_data):.4f} ± {np.std(valid_data, ddof=1):.4f}"


def format_scalar(value):
    """Format a single scalar while handling NaN/inf gracefully."""
    if value is None or not np.isfinite(value):
        return "N/A"
    return f"{value:.4f}"


def format_win_rate(wins, total):
    """Format a seed-level win rate."""
    if total <= 0:
        return "N/A"
    return f"{wins}/{total} ({100.0 * wins / total:.1f}%)"


def format_seed_label(seed_value):
    """Render numeric seed ids as seed_N for readability."""
    try:
        seed_float = float(seed_value)
    except (TypeError, ValueError):
        return str(seed_value)
    if np.isfinite(seed_float) and float(seed_float).is_integer():
        return f"seed_{int(seed_float)}"
    return str(seed_value)


def smooth_by_seed(df, columns, window_size):
    smoothed = df.sort_values(by=["seed", "method", "sim_time"]).copy()
    for column in columns:
        smooth_column = f"{column}_smooth"
        smoothed[smooth_column] = smoothed.groupby(["seed", "method"])[column].transform(
            lambda values: values.rolling(window_size, min_periods=1, center=True).mean()
        )
    return smoothed


def build_plot_summary(df, obstacle_distance_column):
    summary = (
        df.groupby(["method", "sim_time"], as_index=False)
        .agg(
            tracking_error_mean=("tracking_error_smooth", "mean"),
            tracking_error_std=("tracking_error_smooth", "std"),
            horizon_rms_xy_mean=("horizon_rms_xy_smooth", "mean"),
            horizon_rms_xy_std=("horizon_rms_xy_smooth", "std"),
            obstacle_surface_distance_mean=(
                f"{obstacle_distance_column}_smooth",
                "mean",
            ),
            car_a_norm_mean=("car_a_norm_smooth", "mean"),
            car_omega_norm_mean=("car_omega_norm_smooth", "mean"),
        )
        .sort_values(["method", "sim_time"])
        .reset_index(drop=True)
    )
    for column in ("tracking_error_std", "horizon_rms_xy_std"):
        summary[column] = summary[column].fillna(0.0)
    return summary


def plot_mean_with_band(ax, summary, method, value_key, std_key, color, label, clamp_lower=None):
    method_df = summary[summary["method"] == method].sort_values("sim_time")
    if method_df.empty:
        return None
    times = method_df["sim_time"].to_numpy()
    mean_values = method_df[value_key].to_numpy()
    std_values = method_df[std_key].to_numpy()
    line = ax.plot(times, mean_values, color=color, label=label)[0]
    lower = mean_values - std_values
    upper = mean_values + std_values
    if clamp_lower is not None:
        lower = np.maximum(lower, clamp_lower)
    ax.fill_between(times, lower, upper, color=color, alpha=0.14, linewidth=0.0)
    return line


def normalize_seed_key(seed_value):
    """Normalize heterogeneous seed representations to a stable join key."""
    try:
        seed_float = float(seed_value)
    except (TypeError, ValueError):
        return str(seed_value)
    if np.isfinite(seed_float) and float(seed_float).is_integer():
        return int(seed_float)
    return seed_float


def load_tracking_rms_records(run_root):
    """Load run-level tracking RMS from per-seed metrics CSVs."""
    records = []
    metrics_files = {
        "stage": "noninertial_metrics.csv",
        "frozen": "noninertial_frozen_metrics.csv",
    }
    for seed_dir in sorted(run_root.glob("seed_*")):
        seed_token = seed_dir.name.split("_", 1)[-1]
        seed_key = normalize_seed_key(seed_token)
        for method, filename in metrics_files.items():
            metrics_path = seed_dir / filename
            if not metrics_path.exists():
                continue
            metrics_df = pd.read_csv(metrics_path)
            scope_all = metrics_df[metrics_df["scope"] == "all"].copy()
            if scope_all.empty:
                continue
            tracking_rms = pd.to_numeric(scope_all["tracking_rms"], errors="coerce").iloc[0]
            records.append(
                {
                    "seed_key": seed_key,
                    "method": method,
                    "tracking_rms": tracking_rms,
                }
            )
    return pd.DataFrame(records)


def build_c2_summary(df, run_root):
    """Build the C2 summary table tying prediction and tracking results together."""
    per_seed = (
        df.groupby(["seed_key", "method"], as_index=False)
        .agg(
            mean_horizon_rmse=("horizon_rms_xy", "mean"),
            max_deviation=("tracking_error", "max"),
            tracking_rms_fallback=("tracking_error", lambda s: np.sqrt(np.mean(np.square(s)))),
        )
    )

    tracking_rms_records = load_tracking_rms_records(run_root)
    if not tracking_rms_records.empty:
        per_seed = per_seed.merge(
            tracking_rms_records,
            on=["seed_key", "method"],
            how="left",
        )
    else:
        per_seed["tracking_rms"] = np.nan

    per_seed["tracking_rms"] = per_seed["tracking_rms"].fillna(per_seed["tracking_rms_fallback"])
    per_seed = per_seed.drop(columns=["tracking_rms_fallback"])

    methods = [method for method in ("frozen", "stage") if method in set(per_seed["method"])]
    label_map = {"frozen": "Frozen", "stage": "Stage"}
    metric_columns = ["mean_horizon_rmse", "tracking_rms", "max_deviation"]

    seed_metric_map = {}
    for row in per_seed.itertuples(index=False):
        seed_metric_map[(row.seed_key, row.method)] = {
            "mean_horizon_rmse": row.mean_horizon_rmse,
            "tracking_rms": row.tracking_rms,
            "max_deviation": row.max_deviation,
        }

    comparable_seeds = sorted(
        set(per_seed.loc[per_seed["method"] == "frozen", "seed_key"])
        & set(per_seed.loc[per_seed["method"] == "stage", "seed_key"])
    )
    eps = 1e-12

    summary_rows = []
    for method in methods:
        method_rows = per_seed[per_seed["method"] == method]
        other_method = "stage" if method == "frozen" else "frozen"
        wins = 0
        for seed_key in comparable_seeds:
            current = seed_metric_map.get((seed_key, method))
            other = seed_metric_map.get((seed_key, other_method))
            if current is None or other is None:
                continue
            if all(
                np.isfinite(current[metric])
                and np.isfinite(other[metric])
                and current[metric] < other[metric] - eps
                for metric in metric_columns
            ):
                wins += 1

        summary_rows.append(
            {
                "Method": label_map.get(method, method.title()),
                "Mean Horizon RMSE [m]": format_scalar(method_rows["mean_horizon_rmse"].mean()),
                "Tracking RMS [m]": format_scalar(method_rows["tracking_rms"].mean()),
                "Max Deviation [m]": format_scalar(method_rows["max_deviation"].mean()),
                "Win Rate": format_win_rate(wins, len(comparable_seeds)),
            }
        )

    summary_df = pd.DataFrame(summary_rows)
    per_seed_export = per_seed.copy()
    per_seed_export["Seed"] = per_seed_export["seed_key"].map(format_seed_label)
    per_seed_export["Method"] = per_seed_export["method"].map(label_map).fillna(per_seed_export["method"])
    per_seed_export = per_seed_export[
        ["Seed", "Method", "mean_horizon_rmse", "tracking_rms", "max_deviation"]
    ].rename(
        columns={
            "mean_horizon_rmse": "Mean Horizon RMSE [m]",
            "tracking_rms": "Tracking RMS [m]",
            "max_deviation": "Max Deviation [m]",
        }
    )
    return summary_df, per_seed_export, len(comparable_seeds)

def main():
    parser = argparse.ArgumentParser(description="Stage-wise vs Frozen Robustness Analysis")
    parser.add_argument("--csv", required=True, help="Merged CSV containing all seeds data (sim_time, method, seed, horizon_rms_xy, car_a_norm, car_omega_norm, tracking_error, obstacle_surface_distance, etc.)")
    parser.add_argument("--out-plot", default="results/three_axis_alignment.png", help="Output path for the plot")
    parser.add_argument("--accel-thr", type=float, default=0.8, help="Threshold for High-G maneuvers")
    parser.add_argument("--phase-max-lag-sec", type=float, default=2.0, help="Maximum absolute lag to consider for phase-lag estimation.")
    args = parser.parse_args()

    csv_path = pathlib.Path(args.csv)
    if not csv_path.exists():
        raise FileNotFoundError(f"Data file {csv_path} not found. Please ensure you have exported and merged your ROS bag data into this CSV format.")

    df = pd.read_csv(csv_path)
    
    if "frame_mode_effective" in df.columns:
        df = df[df["frame_mode_effective"] == "noninertial"].copy()
    df = df[df["method"].isin(["stage", "frozen"])].copy()
    if df.empty:
        raise SystemExit("No noninertial stage/frozen rows found in robustness CSV.")
    df["seed_key"] = df["seed"].map(normalize_seed_key)

    obstacle_distance_column = (
        "obstacle_surface_distance"
        if "obstacle_surface_distance" in df.columns
        else "min_h"
    )

    window_size = 15
    df = smooth_by_seed(
        df,
        [
            "tracking_error",
            "horizon_rms_xy",
            obstacle_distance_column,
            "car_a_norm",
            "car_omega_norm",
        ],
        window_size,
    )
    plot_summary = build_plot_summary(df, obstacle_distance_column)

    # Set up Seaborn style
    sns.set_theme(style="whitegrid", context="paper", font_scale=1.2)
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Helvetica", "Arial", "DejaVu Sans"],
        "mathtext.fontset": "dejavusans",
        "axes.linewidth": 1.2,
        "lines.linewidth": 1.5
    })

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12.4), sharex=True, gridspec_kw={'height_ratios': [1, 1, 1.2]})
    
    # Define color palette
    palette = {"frozen": "#d62728", "stage": "#1f77b4"}  # Red for Frozen, Blue for Stage-wise
    
    # ---------------------------------------------------------
    # TOP PLOT: Tracking Error only
    # ---------------------------------------------------------
    top_handles = []
    legend_method_labels = {"frozen": "Frozen", "stage": "Stage"}
    for method in ("frozen", "stage"):
        handle = plot_mean_with_band(
            ax1,
            plot_summary,
            method,
            "tracking_error_mean",
            "tracking_error_std",
            palette[method],
            legend_method_labels[method],
            clamp_lower=0.0,
        )
        if handle is not None:
            top_handles.append(handle)
    place_panel_legend(ax1, top_handles, [h.get_label() for h in top_handles], ncol=2, y=1.10)
    ax1.set_title("Top: Worst-UAV Tracking Error", pad=10)
    ax1.set_ylabel("Tracking Error $e_{track}$ [m]\n(SMA Smoothed)")
    ax1.set_ylim(bottom=0.0)

    # ---------------------------------------------------------
    # MIDDLE PLOT: UGV Maneuver Dynamics
    # ---------------------------------------------------------
    df_ugv = plot_summary[plot_summary["method"] == "stage"].sort_values("sim_time")
    if not df_ugv.empty:
        ax2.plot(
            df_ugv["sim_time"].to_numpy(),
            df_ugv["car_a_norm_mean"].to_numpy(),
            color="purple",
            label=r"Lin Accel $|\mathbf{a}_{car,world}|$",
        )
    ax2_twin = ax2.twinx()
    if not df_ugv.empty:
        ax2_twin.plot(
            df_ugv["sim_time"].to_numpy(),
            df_ugv["car_omega_norm_mean"].to_numpy(),
            color="orange",
            label=r"World Ang Vel $|\omega_{car,world}|$",
        )
    
    ax2.set_title("Middle: UGV Maneuver Dynamics", pad=10)
    ax2.set_ylabel(r"$|\mathbf{a}_{car,world}|$ [m/s$^2$]", color="purple")
    ax2_twin.set_ylabel(r"$|\omega_{car,world}|$ [rad/s]", color="orange")
    ax2_twin.grid(False)
    ax2.set_ylim(bottom=0.0)
    ax2_twin.set_ylim(bottom=0.0)
    
    # Combine legends for ax2
    lines_1, labels_1 = ax2.get_legend_handles_labels()
    lines_2, labels_2 = ax2_twin.get_legend_handles_labels()
    place_panel_legend(ax2, lines_1 + lines_2, labels_1 + labels_2, ncol=2, y=1.10)

    # ---------------------------------------------------------
    # BOTTOM PLOT: Horizon RMS Error (e_rms)
    # ---------------------------------------------------------
    bottom_handles = []
    for method in ("frozen", "stage"):
        handle = plot_mean_with_band(
            ax3,
            plot_summary,
            method,
            "horizon_rms_xy_mean",
            "horizon_rms_xy_std",
            palette[method],
            legend_method_labels[method],
            clamp_lower=0.0,
        )
        if handle is not None:
            bottom_handles.append(handle)
    ax3.set_title("Bottom: Across-Seed Mean ± Std of Horizon RMS Error", pad=10)
    ax3.set_ylabel("Horizon $e_{rms}$ [m]\n(seed-mean with ±1 std band)")
    ax3.set_xlabel("Simulation Time [s]")
    ax3.set_ylim(bottom=0.0)
    place_panel_legend(ax3, bottom_handles, [h.get_label() for h in bottom_handles], ncol=2, y=1.10)
    
    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.980), h_pad=2.6)
    out_path = pathlib.Path(args.out_plot)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    pdf_path = out_path.with_suffix(".pdf")
    if pdf_path != out_path:
        plt.savefig(pdf_path, bbox_inches='tight')
        print(f"\n[INFO] Three-Axis Alignment Plot saved to: {out_path}")
        print(f"[INFO] Three-Axis Alignment PDF saved to: {pdf_path}")
    else:
        print(f"\n[INFO] Three-Axis Alignment Plot saved to: {out_path}")

    # ---------------------------------------------------------
    # STATISTICAL ROBUSTNESS TABLE
    # ---------------------------------------------------------
    table_str = "\n" + "="*60 + "\n"
    table_str += " STATISTICAL ROBUSTNESS ANALYSIS (Across Seeds)\n"
    table_str += "="*60 + "\n\n"
    table_str += (
        "Across Seeds means: first compute each statistic inside each seed, "
        "then report mean ± std across seed_1..seed_N.\n\n"
    )

    run_root = out_path.parent
    c2_summary_df, c2_per_seed_df, comparable_seed_count = build_c2_summary(df, run_root)
    if not c2_summary_df.empty:
        table_str += "## C2 Summary\n\n"
        table_str += (
            "Lower is better for all numeric columns. "
            "Win Rate = fraction of seeds where the method beats the other on "
            "Mean Horizon RMSE, Tracking RMS, and Max Deviation simultaneously.\n\n"
        )
        table_str += c2_summary_df.to_markdown(index=False, disable_numparse=True) + "\n\n"
        table_str += (
            f"Comparable seeds used for Win Rate: {comparable_seed_count}\n\n"
        )

    methods = [method for method in ("frozen", "stage") if method in set(df["method"])]
    stats_dict = {}

    for method in methods:
        df_m = df[df['method'] == method]
        
        # Aggregate by seed first
        seed_means = df_m.groupby('seed')['horizon_rms_xy'].mean()
        seed_maxs = df_m.groupby('seed')['horizon_rms_xy'].max()
        seed_overshoots = df_m.groupby('seed')['tracking_error'].max()
        
        # Calculate Phase Lag per seed (Lag of tracking error behind UGV Accel)
        phase_lags = []
        for s in df_m['seed'].unique():
            df_seed = df_m[df_m['seed'] == s].sort_values("sim_time")
            lag = compute_phase_lag(
                df_seed['sim_time'].values,
                df_seed['car_a_norm_smooth'].values,
                df_seed['tracking_error_smooth'].values,
                max_lag_sec=args.phase_max_lag_sec,
            )
            phase_lags.append(lag)
            
        stats_dict[method] = {
            "Mean e_rms [m]": format_mean_std(seed_means.values),
            "Max e_rms [m]": format_mean_std(seed_maxs.values),
            "Overshoot (e_track) [m]": format_mean_std(seed_overshoots.values),
            "Phase Lag [s]": format_mean_std(phase_lags)
        }
        
    stats_df = pd.DataFrame(stats_dict).T
    table_str += stats_df.to_markdown() + "\n\n"

    per_seed_rows = []
    method_labels = {"frozen": "Frozen", "stage": "Stage"}
    for seed in sorted(df["seed"].dropna().unique()):
        row = {"Seed": format_seed_label(seed)}
        for method in methods:
            df_seed_method = df[(df["seed"] == seed) & (df["method"] == method)]
            mean_e_rms = (
                df_seed_method["horizon_rms_xy"].mean()
                if not df_seed_method.empty
                else np.nan
            )
            max_e_rms = (
                df_seed_method["horizon_rms_xy"].max()
                if not df_seed_method.empty
                else np.nan
            )
            label_prefix = method_labels.get(method, method.title())
            row[f"{label_prefix} Mean e_rms [m]"] = format_scalar(mean_e_rms)
            row[f"{label_prefix} Max e_rms [m]"] = format_scalar(max_e_rms)
        per_seed_rows.append(row)

    if per_seed_rows:
        per_seed_df = pd.DataFrame(per_seed_rows)
        table_str += "--- Per-Seed e_rms ---\n"
        table_str += per_seed_df.to_markdown(index=False, disable_numparse=True) + "\n\n"
    
    table_str += "--- Safety & Robustness Metrics ---\n"
    observed_max_accel = df_ugv["car_a_norm_mean"].max() if not df_ugv.empty else np.nan
    if np.isfinite(observed_max_accel):
        table_str += (
            f"Observed max |a_car,world|: {observed_max_accel:.4f} m/s^2\n"
        )

    print(table_str)

    out_table_path = out_path.with_name("robustness_table.md")
    out_table_path.write_text(table_str, encoding="utf-8")
    print(f"\n[INFO] Statistical Robustness Table saved to file: {out_table_path}")

    if not c2_summary_df.empty:
        c2_table_path = out_path.with_name("c2_summary_table.md")
        c2_table_lines = [
            "# C2 Summary",
            "",
            "Lower is better for all numeric columns.",
            "Win Rate = fraction of seeds where the method beats the other on "
            "Mean Horizon RMSE, Tracking RMS, and Max Deviation simultaneously.",
            "",
            c2_summary_df.to_markdown(index=False, disable_numparse=True),
            "",
            f"Comparable seeds: {comparable_seed_count}",
            "",
        ]
        c2_table_path.write_text("\n".join(c2_table_lines), encoding="utf-8")
        c2_csv_path = out_path.with_name("c2_per_seed_metrics.csv")
        c2_per_seed_df.to_csv(c2_csv_path, index=False)
        print(f"[INFO] C2 summary table saved to: {c2_table_path}")
        print(f"[INFO] C2 per-seed metrics saved to: {c2_csv_path}")

if __name__ == "__main__":
    main()
