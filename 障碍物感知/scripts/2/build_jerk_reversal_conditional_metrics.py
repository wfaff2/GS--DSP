#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

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
    )


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


def safe_mean(df: pd.DataFrame, column: str, mask: pd.Series) -> float:
    values = df.loc[mask, column].to_numpy(dtype=float)
    if values.size == 0:
        return float("nan")
    return float(np.nanmean(values))


def compute_run_metrics(
    run_root: Path,
    jerk_threshold: float,
    window_radius: float,
    min_reversal_time: float,
) -> tuple[pd.DataFrame, dict[str, float]]:
    rows: list[dict[str, float | int | str | bool]] = []

    for seed_dir in sorted(p for p in run_root.glob("seed_*") if p.is_dir()):
        try:
            seed = int(seed_dir.name.split("_", 1)[1])
        except Exception:
            continue

        stage_step = compute_jerk(load_step_summary(seed_dir / "noninertial_metrics_steps.csv"))
        frozen_step = load_step_summary(seed_dir / "noninertial_frozen_metrics_steps.csv")
        stage_horizon = load_horizon_summary(seed_dir / "noninertial_metrics_ugv_horizon_summary.csv")
        frozen_horizon = load_horizon_summary(seed_dir / "noninertial_frozen_metrics_ugv_horizon_summary.csv")

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

        prev_sign = 0
        last_window_end = -np.inf
        seed_event_idx = 0
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
                seed_event_idx += 1

                step_mask = (
                    (merged_step["sim_time"] >= window_start)
                    & (merged_step["sim_time"] <= window_end)
                )
                horizon_mask = (
                    (merged_horizon["sim_time"] >= window_start)
                    & (merged_horizon["sim_time"] <= window_end)
                )
                if not bool(step_mask.any()) or not bool(horizon_mask.any()):
                    prev_sign = current_sign
                    continue

                stage_pred = safe_mean(merged_horizon, "horizon_rms_xy_stage", horizon_mask)
                frozen_pred = safe_mean(merged_horizon, "horizon_rms_xy_frozen", horizon_mask)
                stage_track = safe_mean(merged_step, "tracking_error_stage", step_mask)
                frozen_track = safe_mean(merged_step, "tracking_error_frozen", step_mask)
                pred_penalty = stage_pred - frozen_pred
                track_gain = frozen_track - stage_track
                eligible = bool(stage_pred > frozen_pred)
                win = bool(eligible and stage_track < frozen_track)

                rows.append(
                    {
                        "seed": seed,
                        "event_idx": seed_event_idx,
                        "reversal_time": sim_time,
                        "window_start": window_start,
                        "window_end": window_end,
                        "jerk_x": float(row.jerk_x),
                        "stage_horizon_rmse": stage_pred,
                        "frozen_horizon_rmse": frozen_pred,
                        "stage_tracking_error": stage_track,
                        "frozen_tracking_error": frozen_track,
                        "pred_penalty_stage_minus_frozen": pred_penalty,
                        "tracking_gain_frozen_minus_stage": track_gain,
                        "eligible_pred_worse": eligible,
                        "conditional_win": win,
                    }
                )
            prev_sign = current_sign

    events_df = pd.DataFrame(rows).sort_values(
        ["seed", "reversal_time", "event_idx"], ignore_index=True
    )
    if events_df.empty:
        return events_df, {
            "total_windows": 0,
            "eligible_windows": 0,
            "winning_windows": 0,
            "conditional_win_rate": float("nan"),
            "seeds_with_eligible_windows": 0,
            "seeds_with_any_win": 0,
            "seeds_all_eligible_windows_win": 0,
            "mean_pred_penalty_eligible": float("nan"),
            "std_pred_penalty_eligible": float("nan"),
            "mean_tracking_gain_eligible": float("nan"),
            "std_tracking_gain_eligible": float("nan"),
        }

    eligible_df = events_df[events_df["eligible_pred_worse"]].copy()
    seed_stats = (
        eligible_df.groupby("seed", as_index=False)
        .agg(
            eligible_windows=("eligible_pred_worse", "sum"),
            winning_windows=("conditional_win", "sum"),
        )
    )
    seeds_with_eligible = int((seed_stats["eligible_windows"] > 0).sum())
    seeds_with_any_win = int((seed_stats["winning_windows"] > 0).sum())
    seeds_all_eligible_windows_win = int(
        (
            (seed_stats["eligible_windows"] > 0)
            & (seed_stats["eligible_windows"] == seed_stats["winning_windows"])
        ).sum()
    )

    summary = {
        "total_windows": int(len(events_df)),
        "eligible_windows": int(len(eligible_df)),
        "winning_windows": int(eligible_df["conditional_win"].sum()),
        "conditional_win_rate": float(eligible_df["conditional_win"].mean())
        if not eligible_df.empty
        else float("nan"),
        "seeds_with_eligible_windows": seeds_with_eligible,
        "seeds_with_any_win": seeds_with_any_win,
        "seeds_all_eligible_windows_win": seeds_all_eligible_windows_win,
        "mean_pred_penalty_eligible": float(
            eligible_df["pred_penalty_stage_minus_frozen"].mean()
        )
        if not eligible_df.empty
        else float("nan"),
        "std_pred_penalty_eligible": float(
            eligible_df["pred_penalty_stage_minus_frozen"].std(ddof=0)
        )
        if not eligible_df.empty
        else float("nan"),
        "mean_tracking_gain_eligible": float(
            eligible_df["tracking_gain_frozen_minus_stage"].mean()
        )
        if not eligible_df.empty
        else float("nan"),
        "std_tracking_gain_eligible": float(
            eligible_df["tracking_gain_frozen_minus_stage"].std(ddof=0)
        )
        if not eligible_df.empty
        else float("nan"),
    }
    return events_df, summary


def fmt(value: float) -> str:
    if value != value:
        return "nan"
    return f"{value:.4f}"


def write_markdown(
    out_path: Path,
    label: str,
    run_root: Path,
    jerk_threshold: float,
    window_radius: float,
    min_reversal_time: float,
    events_df: pd.DataFrame,
    summary: dict[str, float],
) -> None:
    lines = [
        f"# Conditional Win Rate on Jerk-Reversal Segments ({label})",
        "",
        "定义：仅在 `jerk` 符号反转的非重叠时间窗口内统计；若该窗口满足 "
        "`Stage-wise` 的局部 `Mean Horizon RMSE` 高于 `Frozen`，但局部 "
        "`Tracking Error` 低于 `Frozen`，则记为一次 `conditional win`。",
        "",
        f"- Run Root: `{run_root}`",
        f"- Jerk Threshold: `{jerk_threshold}`",
        f"- Window Radius: `{window_radius} s`",
        f"- Min Reversal Time: `{min_reversal_time} s`",
        "",
        f"- Total Jerk-Reversal Windows: `{int(summary['total_windows'])}`",
        f"- Eligible Windows (`stage pred > frozen pred`): `{int(summary['eligible_windows'])}`",
        f"- Winning Windows (`stage track < frozen track`): `{int(summary['winning_windows'])}`",
        f"- Conditional Win Rate: `{int(summary['winning_windows'])}/{int(summary['eligible_windows'])}` ({fmt(summary['conditional_win_rate'] * 100.0)}%)",
        f"- Seeds with Eligible Windows: `{int(summary['seeds_with_eligible_windows'])}/30`",
        f"- Seeds with Any Conditional Win: `{int(summary['seeds_with_any_win'])}/30`",
        f"- Seeds Winning All Eligible Windows: `{int(summary['seeds_all_eligible_windows_win'])}/{int(summary['seeds_with_eligible_windows'])}`",
        f"- Mean Prediction Penalty on Eligible Windows (`stage - frozen`): `{fmt(summary['mean_pred_penalty_eligible'])} ± {fmt(summary['std_pred_penalty_eligible'])} m`",
        f"- Mean Tracking Gain on Eligible Windows (`frozen - stage`): `{fmt(summary['mean_tracking_gain_eligible'])} ± {fmt(summary['std_tracking_gain_eligible'])} m`",
        "",
    ]

    if not events_df.empty:
        preview = events_df[events_df["eligible_pred_worse"]].head(10).copy()
        if not preview.empty:
            lines.extend(
                [
                    "## Eligible Window Preview",
                    "",
                    "| Seed | t_rev [s] | Stage Horizon RMSE | Frozen Horizon RMSE | Stage Tracking | Frozen Tracking | Pred Penalty | Track Gain | Win |",
                    "|:--|:--|:--|:--|:--|:--|:--|:--|:--|",
                ]
            )
            for row in preview.itertuples(index=False):
                lines.append(
                    f"| {int(row.seed)} | {fmt(float(row.reversal_time))} | "
                    f"{fmt(float(row.stage_horizon_rmse))} | {fmt(float(row.frozen_horizon_rmse))} | "
                    f"{fmt(float(row.stage_tracking_error))} | {fmt(float(row.frozen_tracking_error))} | "
                    f"{fmt(float(row.pred_penalty_stage_minus_frozen))} | {fmt(float(row.tracking_gain_frozen_minus_stage))} | "
                    f"{'yes' if bool(row.conditional_win) else 'no'} |"
                )
            lines.append("")

    out_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compute conditional win rate on jerk-reversal windows."
    )
    parser.add_argument("--run-root", required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--out-md", required=True)
    parser.add_argument("--out-csv", required=True)
    parser.add_argument("--jerk-threshold", type=float, default=0.5)
    parser.add_argument("--window-radius", type=float, default=0.75)
    parser.add_argument("--min-reversal-time", type=float, default=0.0)
    args = parser.parse_args()

    run_root = Path(args.run_root).expanduser().resolve()
    out_md = Path(args.out_md).expanduser().resolve()
    out_csv = Path(args.out_csv).expanduser().resolve()

    events_df, summary = compute_run_metrics(
        run_root=run_root,
        jerk_threshold=args.jerk_threshold,
        window_radius=args.window_radius,
        min_reversal_time=args.min_reversal_time,
    )

    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    write_markdown(
        out_path=out_md,
        label=args.label,
        run_root=run_root,
        jerk_threshold=args.jerk_threshold,
        window_radius=args.window_radius,
        min_reversal_time=args.min_reversal_time,
        events_df=events_df,
        summary=summary,
    )
    events_df.to_csv(out_csv, index=False)


if __name__ == "__main__":
    main()
