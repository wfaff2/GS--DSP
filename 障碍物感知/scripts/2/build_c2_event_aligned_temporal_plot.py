#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_step_summary(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    for col in ("sim_time", "tracking_error", "a_car_non_x"):
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return (
        df.groupby("sim_time", as_index=False)
        .agg(
            tracking_error=("tracking_error", "mean"),
            a_car_non_x=("a_car_non_x", "mean"),
        )
        .sort_values("sim_time")
        .reset_index(drop=True)
    )


def load_horizon_summary(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    for col in ("sim_time", "horizon_rms_xy"):
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df[["sim_time", "horizon_rms_xy"]].sort_values("sim_time").reset_index(drop=True)


def compute_jerk(step_df: pd.DataFrame) -> pd.DataFrame:
    out = step_df.copy()
    dt = out["sim_time"].diff()
    da = out["a_car_non_x"].diff()
    jerk = da / dt
    jerk[~np.isfinite(jerk)] = np.nan
    out["jerk_x"] = jerk
    return out


def jerk_sign(value: float, threshold: float) -> int:
    if not np.isfinite(value) or abs(value) < threshold:
        return 0
    return 1 if value > 0.0 else -1


def interpolate_series(time: np.ndarray, values: np.ndarray, query: np.ndarray) -> np.ndarray:
    valid = np.isfinite(time) & np.isfinite(values)
    if valid.sum() < 2:
        return np.full(query.shape, np.nan, dtype=float)
    x = time[valid]
    y = values[valid]
    out = np.interp(query, x, y)
    out[(query < x[0]) | (query > x[-1])] = np.nan
    return out


def finite_quantile(arr: np.ndarray, q: float) -> np.ndarray:
    with np.errstate(all="ignore"):
        return np.nanquantile(arr, q, axis=0)


def collect_event_stacks(
    run_root: Path,
    jerk_threshold: float,
    window_radius: float,
    pre_window: float,
    post_window: float,
    grid_dt: float,
    min_reversal_time: float,
) -> tuple[np.ndarray, dict[str, np.ndarray], pd.DataFrame]:
    rel_t = np.arange(-pre_window, post_window + 1e-9, grid_dt, dtype=float)

    stage_pred_stack: list[np.ndarray] = []
    frozen_pred_stack: list[np.ndarray] = []
    stage_track_stack: list[np.ndarray] = []
    frozen_track_stack: list[np.ndarray] = []
    rows: list[dict[str, float | int]] = []

    for seed_dir in sorted(p for p in run_root.glob("seed_*") if p.is_dir()):
        try:
            seed = int(seed_dir.name.split("_", 1)[1])
        except Exception:
            continue

        stage_step = compute_jerk(load_step_summary(seed_dir / "noninertial_metrics_steps.csv"))
        frozen_step = load_step_summary(seed_dir / "noninertial_frozen_metrics_steps.csv")
        stage_horizon = load_horizon_summary(seed_dir / "noninertial_metrics_ugv_horizon_summary.csv")
        frozen_horizon = load_horizon_summary(seed_dir / "noninertial_frozen_metrics_ugv_horizon_summary.csv")

        prev_sign = 0
        last_window_end = -np.inf
        event_idx = 0
        for row in stage_step.itertuples(index=False):
            sim_time = float(row.sim_time)
            if sim_time < min_reversal_time:
                continue
            current_sign = jerk_sign(float(row.jerk_x), jerk_threshold)
            if current_sign == 0:
                continue
            if prev_sign != 0 and current_sign != prev_sign:
                window_start = sim_time - window_radius
                window_end = sim_time + window_radius
                if window_start < last_window_end:
                    prev_sign = current_sign
                    continue
                last_window_end = window_end
                event_idx += 1
                query = sim_time + rel_t
                stage_pred_stack.append(
                    interpolate_series(
                        stage_horizon["sim_time"].to_numpy(dtype=float),
                        stage_horizon["horizon_rms_xy"].to_numpy(dtype=float),
                        query,
                    )
                )
                frozen_pred_stack.append(
                    interpolate_series(
                        frozen_horizon["sim_time"].to_numpy(dtype=float),
                        frozen_horizon["horizon_rms_xy"].to_numpy(dtype=float),
                        query,
                    )
                )
                stage_track_stack.append(
                    interpolate_series(
                        stage_step["sim_time"].to_numpy(dtype=float),
                        stage_step["tracking_error"].to_numpy(dtype=float),
                        query,
                    )
                )
                frozen_track_stack.append(
                    interpolate_series(
                        frozen_step["sim_time"].to_numpy(dtype=float),
                        frozen_step["tracking_error"].to_numpy(dtype=float),
                        query,
                    )
                )
                rows.append(
                    {
                        "seed": seed,
                        "event_idx": event_idx,
                        "reversal_time": sim_time,
                        "window_start": window_start,
                        "window_end": window_end,
                        "jerk_x": float(row.jerk_x),
                    }
                )
            prev_sign = current_sign

    if not stage_pred_stack:
        raise SystemExit(f"No jerk sign-reversal events found under {run_root}")

    stacks = {
        "stage_pred": np.vstack(stage_pred_stack),
        "frozen_pred": np.vstack(frozen_pred_stack),
        "stage_track": np.vstack(stage_track_stack),
        "frozen_track": np.vstack(frozen_track_stack),
    }
    events_df = pd.DataFrame(rows).sort_values(["seed", "reversal_time", "event_idx"]).reset_index(drop=True)
    return rel_t, stacks, events_df


def plot_bands(rel_t: np.ndarray, stacks: dict[str, np.ndarray], out_path: Path) -> None:
    colors = {"stage": "#1f77b4", "frozen": "#d62728"}
    labels = {"stage": "Stage", "frozen": "Frozen"}
    fig, axes = plt.subplots(2, 1, figsize=(9.8, 7.6), sharex=True, constrained_layout=True)

    plot_specs = [
        ("Horizon RMSE [m]", "pred"),
        ("Tracking Error [m]", "track"),
    ]
    for idx, (ax, (ylabel, suffix)) in enumerate(zip(axes, plot_specs)):
        for method in ("stage", "frozen"):
            arr = stacks[f"{method}_{suffix}"]
            median = finite_quantile(arr, 0.5)
            q10 = finite_quantile(arr, 0.1)
            q90 = finite_quantile(arr, 0.9)
            ax.fill_between(rel_t, q10, q90, color=colors[method], alpha=0.18)
            ax.plot(rel_t, median, color=colors[method], linewidth=2.2, label=labels[method])
        ax.axvline(0.0, color="black", linestyle="--", linewidth=1.1)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25, linestyle="--")
        if idx == 0:
            ax.legend(framealpha=0.92, ncol=2, loc="upper right")

    axes[1].set_xlabel("Relative Time to Jerk Sign Reversal [s]")
    fig.savefig(out_path, dpi=220)
    plt.close(fig)


def write_summary(
    out_path: Path,
    rel_t: np.ndarray,
    stacks: dict[str, np.ndarray],
    events_df: pd.DataFrame,
    fig_name: str,
    jerk_threshold: float,
    window_radius: float,
    pre_window: float,
    post_window: float,
    grid_dt: float,
) -> None:
    def fmt(value: float) -> str:
        return "nan" if not math.isfinite(value) else f"{value:.6f}"

    stage_pred_t0 = float(finite_quantile(stacks["stage_pred"], 0.5)[np.argmin(np.abs(rel_t))])
    frozen_pred_t0 = float(finite_quantile(stacks["frozen_pred"], 0.5)[np.argmin(np.abs(rel_t))])
    stage_track_t0 = float(finite_quantile(stacks["stage_track"], 0.5)[np.argmin(np.abs(rel_t))])
    frozen_track_t0 = float(finite_quantile(stacks["frozen_track"], 0.5)[np.argmin(np.abs(rel_t))])

    lines = [
        "# C2 Event-Aligned Temporal Plot",
        "",
        "- Comparison: `non-inertial stage-wise` vs `non-inertial frozen`",
        "- Event definition: non-overlapping jerk sign-reversal windows on `a_car_non_x`",
        f"- Jerk threshold: `{jerk_threshold}`",
        f"- Non-overlap window radius: `{window_radius:.3f} s`",
        f"- Time window: `[-{pre_window:.3f}, +{post_window:.3f}] s`",
        f"- Grid dt: `{grid_dt:.3f} s`",
        f"- Total jerk-reversal events: `{len(events_df)}`",
        "",
        "## Median at t = 0",
        "",
        f"- Stage-wise horizon RMSE median: `{fmt(stage_pred_t0)} m`",
        f"- Frozen horizon RMSE median: `{fmt(frozen_pred_t0)} m`",
        f"- Stage-wise tracking error median: `{fmt(stage_track_t0)} m`",
        f"- Frozen tracking error median: `{fmt(frozen_track_t0)} m`",
        "",
        "## Figure",
        "",
        f"![c2_event_aligned_temporal]({fig_name})",
        "",
    ]
    out_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build C2 event-aligned temporal plot.")
    parser.add_argument("run_root", type=Path, help="C2 batch result directory")
    parser.add_argument("--out-dir", type=Path, default=None, help="Output directory")
    parser.add_argument("--jerk-threshold", type=float, default=0.5)
    parser.add_argument("--window-radius", type=float, default=0.75)
    parser.add_argument("--pre-window", type=float, default=1.0)
    parser.add_argument("--post-window", type=float, default=2.0)
    parser.add_argument("--grid-dt", type=float, default=0.02)
    parser.add_argument("--min-reversal-time", type=float, default=0.0)
    args = parser.parse_args()

    run_root = args.run_root.expanduser().resolve()
    out_dir = (args.out_dir or run_root).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    rel_t, stacks, events_df = collect_event_stacks(
        run_root=run_root,
        jerk_threshold=args.jerk_threshold,
        window_radius=args.window_radius,
        pre_window=args.pre_window,
        post_window=args.post_window,
        grid_dt=args.grid_dt,
        min_reversal_time=args.min_reversal_time,
    )

    fig_path = out_dir / "c2_event_aligned_temporal_plot.png"
    csv_path = out_dir / "c2_event_aligned_events.csv"
    md_path = out_dir / "c2_event_aligned_temporal_plot.md"

    plot_bands(rel_t, stacks, fig_path)
    events_df.to_csv(csv_path, index=False)
    write_summary(
        md_path,
        rel_t,
        stacks,
        events_df,
        fig_path.name,
        args.jerk_threshold,
        args.window_radius,
        args.pre_window,
        args.post_window,
        args.grid_dt,
    )

    print(f"[INFO] c2 event csv: {csv_path}")
    print(f"[INFO] c2 event plot: {fig_path}")
    print(f"[INFO] c2 event summary: {md_path}")
    print(f"[INFO] total events: {len(events_df)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
