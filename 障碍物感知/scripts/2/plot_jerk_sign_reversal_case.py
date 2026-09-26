#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import re

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


@dataclass
class Candidate:
    kp: float
    seed: int
    seed_dir: Path
    reversal_time: float
    window_start: float
    window_end: float
    jerk_value: float
    horizon_gain: float
    tracking_gain: float
    score: float


def load_step_summary(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    for col in ("sim_time", "tracking_error", "a_car_non_x"):
        df[col] = pd.to_numeric(df[col], errors="coerce")
    grouped = (
        df.groupby("sim_time", as_index=False)
        .agg(
            tracking_error=("tracking_error", "mean"),
            a_car_non_x=("a_car_non_x", "mean"),
        )
        .sort_values("sim_time")
    )
    return grouped


def load_horizon_summary(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    for col in ("sim_time", "horizon_rms_xy"):
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df[["sim_time", "horizon_rms_xy"]].sort_values("sim_time")


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


def infer_kp(path: Path) -> float | None:
    for part in path.parts:
        match = re.fullmatch(r"kp_(\d+)p(\d+)", part)
        if match:
            return float(f"{match.group(1)}.{match.group(2)}")
    return None


def load_seed_filter(path: Path | None) -> dict[float, set[int]]:
    if path is None:
        return {}
    df = pd.read_csv(path)
    if not {"kp", "seed"}.issubset(df.columns):
        raise RuntimeError(f"Seed filter CSV must contain kp, seed columns: {path}")
    keep: dict[float, set[int]] = {}
    for kp, dkp in df.groupby("kp"):
        keep[float(kp)] = set(int(v) for v in dkp["seed"].tolist())
    return keep


def build_candidates(
    run_root: Path,
    jerk_threshold: float,
    window_radius: float,
    min_reversal_time: float,
    seed_filter: dict[float, set[int]],
) -> tuple[list[Candidate], dict[tuple[float, int], tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, pd.DataFrame]]]:
    candidates: list[Candidate] = []
    cache: dict[tuple[float, int], tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, pd.DataFrame]] = {}
    for seed_dir in sorted(p for p in run_root.rglob("seed_*") if p.is_dir()):
        try:
            seed = int(seed_dir.name.split("_", 1)[1])
        except Exception:
            continue
        kp = infer_kp(seed_dir)
        if kp is None:
            continue
        if seed_filter and seed not in seed_filter.get(kp, set()):
            continue
        stage_step_path = seed_dir / "noninertial_stage_metrics_steps.csv"
        frozen_step_path = seed_dir / "noninertial_metrics_steps.csv"
        stage_horizon_path = seed_dir / "noninertial_stage_metrics_ugv_horizon_summary.csv"
        frozen_horizon_path = seed_dir / "noninertial_metrics_ugv_horizon_summary.csv"
        if not all(p.exists() for p in (
            stage_step_path,
            frozen_step_path,
            stage_horizon_path,
            frozen_horizon_path,
        )):
            continue

        stage_step = compute_jerk(load_step_summary(stage_step_path))
        frozen_step = load_step_summary(frozen_step_path)
        stage_horizon = load_horizon_summary(stage_horizon_path)
        frozen_horizon = load_horizon_summary(frozen_horizon_path)
        cache[(kp, seed)] = (stage_step, frozen_step, stage_horizon, frozen_horizon)

        merged_step = stage_step.merge(
            frozen_step[["sim_time", "tracking_error"]],
            on="sim_time",
            how="inner",
            suffixes=("_stage", "_frozen"),
        )
        merged_horizon = stage_horizon.merge(
            frozen_horizon,
            on="sim_time",
            how="inner",
            suffixes=("_stage", "_frozen"),
        )
        if merged_step.empty or merged_horizon.empty:
            continue
        step_time = merged_step["sim_time"].to_numpy()
        step_gain = (
            merged_step["tracking_error_frozen"] - merged_step["tracking_error_stage"]
        ).to_numpy()
        horizon_time = merged_horizon["sim_time"].to_numpy()
        horizon_gain_series = (
            merged_horizon["horizon_rms_xy_frozen"] - merged_horizon["horizon_rms_xy_stage"]
        ).to_numpy()

        prev_sign = 0
        for row in stage_step.itertuples(index=False):
            if float(row.sim_time) < min_reversal_time:
                continue
            current_sign = jerk_sign(row.jerk_x, jerk_threshold)
            if current_sign == 0:
                continue
            if prev_sign != 0 and current_sign != prev_sign:
                window_start = float(row.sim_time) - window_radius
                window_end = float(row.sim_time) + window_radius

                step_mask = (step_time >= window_start) & (step_time <= window_end)
                horizon_mask = (horizon_time >= window_start) & (horizon_time <= window_end)
                if not np.any(step_mask) or not np.any(horizon_mask):
                    prev_sign = current_sign
                    continue
                horizon_gain = float(np.nanmean(horizon_gain_series[horizon_mask]))
                tracking_gain = float(np.nanmean(step_gain[step_mask]))
                if not np.isfinite(horizon_gain) or not np.isfinite(tracking_gain):
                    prev_sign = current_sign
                    continue
                score = horizon_gain + tracking_gain + 0.05 * abs(float(row.jerk_x))
                candidates.append(
                    Candidate(
                        kp=kp,
                        seed=seed,
                        seed_dir=seed_dir,
                        reversal_time=float(row.sim_time),
                        window_start=window_start,
                        window_end=window_end,
                        jerk_value=float(row.jerk_x),
                        horizon_gain=horizon_gain,
                        tracking_gain=tracking_gain,
                        score=score,
                    )
                )
            prev_sign = current_sign
    return candidates, cache


def select_candidate(candidates: list[Candidate]) -> Candidate:
    preferred = [c for c in candidates if c.horizon_gain > 0.0 and c.tracking_gain > 0.0]
    pool = preferred if preferred else candidates
    if not pool:
        raise SystemExit("No jerk sign reversal candidate found.")
    return max(pool, key=lambda c: c.score)


def plot_case(
    candidate: Candidate,
    cache: dict[tuple[float, int], tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, pd.DataFrame]],
    out_path: Path,
) -> None:
    stage_step, frozen_step, stage_horizon, frozen_horizon = cache[(candidate.kp, candidate.seed)]
    win_mask_stage = (
        (stage_step["sim_time"] >= candidate.window_start)
        & (stage_step["sim_time"] <= candidate.window_end)
    )
    win_mask_frozen = (
        (frozen_step["sim_time"] >= candidate.window_start)
        & (frozen_step["sim_time"] <= candidate.window_end)
    )
    win_mask_stage_h = (
        (stage_horizon["sim_time"] >= candidate.window_start)
        & (stage_horizon["sim_time"] <= candidate.window_end)
    )
    win_mask_frozen_h = (
        (frozen_horizon["sim_time"] >= candidate.window_start)
        & (frozen_horizon["sim_time"] <= candidate.window_end)
    )

    stage_step_local = stage_step[win_mask_stage].copy()
    frozen_step_local = frozen_step[win_mask_frozen].copy()
    stage_h_local = stage_horizon[win_mask_stage_h].copy()
    frozen_h_local = frozen_horizon[win_mask_frozen_h].copy()

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True, constrained_layout=True)

    ax = axes[0]
    ax.plot(stage_step_local["sim_time"], stage_step_local["a_car_non_x"], label="a_long", color="#1f77b4", linewidth=2.0)
    ax.plot(stage_step_local["sim_time"], stage_step_local["jerk_x"], label="jerk_x", color="#ff7f0e", linewidth=1.8)
    ax.axvline(candidate.reversal_time, color="#d62728", linestyle="--", linewidth=1.5)
    ax.axvspan(candidate.window_start, candidate.window_end, color="#d62728", alpha=0.08)
    ax.set_ylabel("Accel / Jerk")
    ax.set_title(
        f"kp={candidate.kp:.1f}, seed={candidate.seed}, jerk sign reversal at t={candidate.reversal_time:.2f}s"
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    ax = axes[1]
    ax.plot(stage_h_local["sim_time"], stage_h_local["horizon_rms_xy"], label="stage horizon RMSE", color="#1f77b4", linewidth=2.0)
    ax.plot(frozen_h_local["sim_time"], frozen_h_local["horizon_rms_xy"], label="frozen horizon RMSE", color="#d62728", linewidth=2.0)
    ax.axvline(candidate.reversal_time, color="#444444", linestyle="--", linewidth=1.2)
    ax.axvspan(candidate.window_start, candidate.window_end, color="#d62728", alpha=0.08)
    ax.set_ylabel("Horizon RMSE [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    ax = axes[2]
    ax.plot(stage_step_local["sim_time"], stage_step_local["tracking_error"], label="stage tracking error", color="#1f77b4", linewidth=2.0)
    ax.plot(frozen_step_local["sim_time"], frozen_step_local["tracking_error"], label="frozen tracking error", color="#d62728", linewidth=2.0)
    ax.axvline(candidate.reversal_time, color="#444444", linestyle="--", linewidth=1.2)
    ax.axvspan(candidate.window_start, candidate.window_end, color="#d62728", alpha=0.08)
    ax.set_xlabel("Simulation Time [s]")
    ax.set_ylabel("Tracking Error [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=200)
    plt.close(fig)


def write_summary(candidate: Candidate, out_path: Path) -> None:
    summary_path = out_path.with_suffix(".md")
    lines = [
        "# Jerk Sign Reversal Representative Case",
        "",
        f"Kappa = {candidate.kp:.1f}",
        f"Seed = {candidate.seed}",
        f"Time Window = [{candidate.window_start:.2f}, {candidate.window_end:.2f}] s",
        f"Reversal Time = {candidate.reversal_time:.2f} s",
        f"Jerk at Reversal = {candidate.jerk_value:.4f}",
        f"Local Horizon RMSE Gain (frozen - stage) = {candidate.horizon_gain:.4f} m",
        f"Local Tracking Error Gain (frozen - stage) = {candidate.tracking_gain:.4f} m",
        f"Score = {candidate.score:.4f}",
        "",
    ]
    summary_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Select and plot a representative jerk sign reversal case.")
    parser.add_argument("--run-root", required=True, help="Batch result directory containing seed_* subdirectories.")
    parser.add_argument("--out", required=True, help="Output PNG path.")
    parser.add_argument("--jerk-threshold", type=float, default=0.5)
    parser.add_argument("--window-radius", type=float, default=0.75)
    parser.add_argument("--min-reversal-time", type=float, default=0.0)
    parser.add_argument(
        "--seed-filter-csv",
        default="",
        help="Optional CSV containing kp,seed rows to restrict the candidate pool.",
    )
    args = parser.parse_args()

    run_root = Path(args.run_root).expanduser().resolve()
    out_path = Path(args.out).expanduser().resolve()
    seed_filter_path = Path(args.seed_filter_csv).expanduser().resolve() if args.seed_filter_csv else None
    seed_filter = load_seed_filter(seed_filter_path)
    candidates, cache = build_candidates(
        run_root=run_root,
        jerk_threshold=args.jerk_threshold,
        window_radius=args.window_radius,
        min_reversal_time=args.min_reversal_time,
        seed_filter=seed_filter,
    )
    candidate = select_candidate(candidates)
    plot_case(candidate, cache, out_path)
    write_summary(candidate, out_path)
    print(
        f"[INFO] jerk sign reversal case saved to: {out_path} "
        f"(kp={candidate.kp:.1f}, seed={candidate.seed}, t={candidate.reversal_time:.3f}s, "
        f"h_gain={candidate.horizon_gain:.4f}, track_gain={candidate.tracking_gain:.4f})"
    )


if __name__ == "__main__":
    main()
