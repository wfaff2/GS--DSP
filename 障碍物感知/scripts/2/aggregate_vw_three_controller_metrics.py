#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd


STEP_REQUIRED_EQ18 = (
    "uav_idx",
    "step_idx",
    "sim_time",
    "rot_load_position_non_x",
    "rot_load_position_non_y",
    "rot_load_position_non_z",
    "rot_load_velocity_non_x",
    "rot_load_velocity_non_y",
    "rot_load_velocity_non_z",
    "rot_load_omega_non_x",
    "rot_load_omega_non_y",
    "rot_load_omega_non_z",
)

HORIZON_REQUIRED_EQ18 = (
    "step_idx",
    "horizon_idx",
    "pred_omega_non_x",
    "pred_omega_non_y",
    "pred_omega_non_z",
)


def safe_float(value: object) -> float:
    try:
        return float(value)  # type: ignore[arg-type]
    except Exception:
        return math.nan


def safe_int(value: object, default: int = 0) -> int:
    try:
        return int(float(value))  # type: ignore[arg-type]
    except Exception:
        return default


def finite_series(values: Iterable[object]) -> pd.Series:
    series = pd.to_numeric(pd.Series(list(values)), errors="coerce")
    return series[np.isfinite(series)]


def mean_value(values: Iterable[object]) -> float:
    series = finite_series(values)
    return float(series.mean()) if not series.empty else math.nan


def std_value(values: Iterable[object]) -> float:
    series = finite_series(values)
    return float(series.std(ddof=1)) if len(series) > 1 else math.nan


def p95_value(values: Iterable[object]) -> float:
    series = finite_series(values)
    return float(series.quantile(0.95)) if not series.empty else math.nan


def min_value(values: Iterable[object]) -> float:
    series = finite_series(values)
    return float(series.min()) if not series.empty else math.nan


def max_value(values: Iterable[object]) -> float:
    series = finite_series(values)
    return float(series.max()) if not series.empty else math.nan


def sum_value(values: Iterable[object]) -> float:
    series = finite_series(values)
    return float(series.sum()) if not series.empty else math.nan


def q_label_to_float(part: str, prefix: str) -> float:
    text = part
    if text.startswith(prefix):
        text = text[len(prefix) :]
    return float(text.replace("p", "."))


def resolve_path(run_root: Path, text: str) -> Path:
    path = Path(text)
    if path.is_absolute():
        return path
    return run_root / path


def controller_from_modes(frame_mode: str, rollout: str) -> str:
    if frame_mode == "inertial":
        return "inertial_frozen"
    if rollout == "stage":
        return "noninertial_stage"
    return "noninertial_frozen"


def scope_all_row(metrics_path: Path) -> Optional[pd.Series]:
    if not metrics_path.is_file():
        return None
    try:
        df = pd.read_csv(metrics_path)
    except Exception:
        return None
    if df.empty:
        return None
    if "scope" in df.columns:
        df = df[df["scope"].astype(str) == "all"]
    if df.empty:
        return None
    return df.iloc[-1]


def metric_value(row: Optional[pd.Series], key: str, fallback: str = "") -> float:
    if row is None:
        return math.nan
    if key in row.index and str(row[key]).strip() != "":
        return safe_float(row[key])
    if fallback and fallback in row.index and str(row[fallback]).strip() != "":
        return safe_float(row[fallback])
    return math.nan


def metric_text(row: Optional[pd.Series], key: str, default: str = "") -> str:
    if row is None or key not in row.index:
        return default
    value = row[key]
    if pd.isna(value):
        return default
    return str(value)


def compute_frot(
    omega: np.ndarray,
    velocity: np.ndarray,
    position: np.ndarray,
) -> np.ndarray:
    return 2.0 * np.cross(omega, velocity) + np.cross(
        omega, np.cross(omega, position)
    )


def read_csv_existing_columns(path: Path, columns: Sequence[str]) -> pd.DataFrame:
    try:
        header = pd.read_csv(path, nrows=0)
    except (pd.errors.EmptyDataError, Exception):
        return pd.DataFrame()
    usecols = [col for col in columns if col in header.columns]
    if not usecols:
        return pd.DataFrame()
    return pd.read_csv(path, usecols=usecols)


def compute_step_stats(
    step_path: Path,
    base: Dict[str, object],
) -> Tuple[Dict[str, object], List[Dict[str, object]]]:
    stats: Dict[str, object] = {
        "step_rows": 0,
        "tracking_error_step_rms": math.nan,
        "tracking_error_step_p95": math.nan,
        "core_time_mean_ms": math.nan,
        "core_time_p95_ms": math.nan,
        "core_time_max_ms": math.nan,
        "core_time_over_20ms_count": math.nan,
        "core_time_over_20ms_rate": math.nan,
        "core_time_worst_uav_p95_ms": math.nan,
        "sensing_scan_events_total": math.nan,
        "sensing_scan_trigger_rate_per_control_row": math.nan,
        "sensing_cast_rays_per_scan_mean": math.nan,
        "sensing_visible_total": math.nan,
        "sensing_generated_total": math.nan,
        "sensing_dropped_total": math.nan,
        "sensing_delivered_total": math.nan,
        "sensing_created_tracks_total": math.nan,
        "sensing_deleted_tracks_total": math.nan,
        "sensing_dropout_observed_rate": math.nan,
        "sensing_active_tracks_mean": math.nan,
        "sensing_active_tracks_nonzero_rate": math.nan,
        "sensing_low_clearance_samples": math.nan,
        "sensing_low_clearance_episodes": math.nan,
        "sensing_low_clearance_rate": math.nan,
        "sensing_low_clearance_no_track_samples": math.nan,
        "sensing_low_clearance_no_track_episodes": math.nan,
        "sensing_low_clearance_no_track_given_low_rate": math.nan,
        "sensing_low_clearance_current_dropout_samples": math.nan,
        "sensing_low_clearance_current_dropout_episodes": math.nan,
        "sensing_low_clearance_current_dropout_given_low_rate": math.nan,
        "sensing_low_clearance_pending_no_track_samples": math.nan,
        "sensing_low_clearance_pending_no_track_episodes": math.nan,
        "sensing_low_clearance_pending_no_track_given_low_rate": math.nan,
        "sensing_low_clearance_stale_track_samples": math.nan,
        "sensing_low_clearance_stale_track_episodes": math.nan,
        "sensing_low_clearance_stale_track_given_low_rate": math.nan,
    }
    uav_rows: List[Dict[str, object]] = []
    if not step_path.is_file():
        return stats, uav_rows

    columns = [
        "uav_idx",
        "step_idx",
        "tracking_error",
        "core_time_ms",
        "feedback_time_ms",
        "preparation_time_ms",
        "sensing_scan_triggered",
        "sensing_scan_index",
        "sensing_cast_rays",
        "sensing_visible_obstacles",
        "sensing_generated_measurements",
        "sensing_dropped_measurements",
        "sensing_delivered_measurements",
        "sensing_created_tracks",
        "sensing_deleted_tracks",
        "sensing_active_tracks",
        "sensing_low_clearance_sample",
        "sensing_low_clearance_no_track_sample",
        "sensing_low_clearance_current_dropout_sample",
        "sensing_low_clearance_pending_no_track_sample",
        "sensing_low_clearance_stale_track_sample",
    ]
    df = read_csv_existing_columns(step_path, columns)
    if df.empty:
        return stats, uav_rows
    stats["step_rows"] = int(len(df))

    if "sensing_scan_triggered" in df.columns:
        scan_triggered = pd.to_numeric(
            df["sensing_scan_triggered"], errors="coerce"
        ).fillna(0.0)
        scan_events = int((scan_triggered > 0.0).sum())
        stats["sensing_scan_events_total"] = scan_events
        stats["sensing_scan_trigger_rate_per_control_row"] = (
            float(scan_events) / float(len(df)) if len(df) else math.nan
        )
        if "sensing_cast_rays" in df.columns and scan_events > 0:
            cast_rays = pd.to_numeric(
                df["sensing_cast_rays"], errors="coerce"
            ).fillna(0.0)
            stats["sensing_cast_rays_per_scan_mean"] = float(
                cast_rays.sum() / float(scan_events)
            )

    sensing_columns = {
        "sensing_visible_total": "sensing_visible_obstacles",
        "sensing_generated_total": "sensing_generated_measurements",
        "sensing_dropped_total": "sensing_dropped_measurements",
        "sensing_delivered_total": "sensing_delivered_measurements",
        "sensing_created_tracks_total": "sensing_created_tracks",
        "sensing_deleted_tracks_total": "sensing_deleted_tracks",
    }
    for output_name, column_name in sensing_columns.items():
        if column_name in df.columns:
            values = pd.to_numeric(df[column_name], errors="coerce").fillna(0.0)
            stats[output_name] = int(values.sum())
    if "sensing_visible_obstacles" in df.columns:
        visible_total = float(stats["sensing_visible_total"])
        dropped_total = float(stats["sensing_dropped_total"])
        if visible_total > 0.0 and math.isfinite(dropped_total):
            stats["sensing_dropout_observed_rate"] = dropped_total / visible_total
    if "sensing_active_tracks" in df.columns:
        active_tracks = pd.to_numeric(
            df["sensing_active_tracks"], errors="coerce"
        ).dropna()
        if not active_tracks.empty:
            stats["sensing_active_tracks_mean"] = float(active_tracks.mean())
            stats["sensing_active_tracks_nonzero_rate"] = float(
                (active_tracks > 0.0).mean()
            )

    def count_episodes(column_name: str) -> int:
        if column_name not in df.columns:
            return 0
        episode_count = 0
        if "uav_idx" in df.columns:
            groups = df.groupby("uav_idx", sort=False, dropna=False)
        else:
            groups = [(0, df)]
        for _, group in groups:
            if "step_idx" in group.columns:
                group = group.sort_values("step_idx")
            flags = (
                pd.to_numeric(group[column_name], errors="coerce")
                .fillna(0.0)
                .gt(0.0)
            )
            episode_count += int((flags & ~flags.shift(fill_value=False)).sum())
        return episode_count

    sensing_event_columns = {
        "sensing_low_clearance": "sensing_low_clearance_sample",
        "sensing_low_clearance_no_track":
            "sensing_low_clearance_no_track_sample",
        "sensing_low_clearance_current_dropout":
            "sensing_low_clearance_current_dropout_sample",
        "sensing_low_clearance_pending_no_track":
            "sensing_low_clearance_pending_no_track_sample",
        "sensing_low_clearance_stale_track":
            "sensing_low_clearance_stale_track_sample",
    }
    sensing_event_counts: Dict[str, int] = {}
    for prefix, column_name in sensing_event_columns.items():
        if column_name not in df.columns:
            continue
        flags = pd.to_numeric(df[column_name], errors="coerce").fillna(0.0)
        count = int((flags > 0.0).sum())
        sensing_event_counts[prefix] = count
        stats[f"{prefix}_samples"] = count
        stats[f"{prefix}_episodes"] = count_episodes(column_name)

    low_count = sensing_event_counts.get("sensing_low_clearance")
    if low_count is not None:
        stats["sensing_low_clearance_rate"] = (
            float(low_count) / float(len(df)) if len(df) else math.nan
        )
        for prefix in (
            "sensing_low_clearance_no_track",
            "sensing_low_clearance_current_dropout",
            "sensing_low_clearance_pending_no_track",
            "sensing_low_clearance_stale_track",
        ):
            count = sensing_event_counts.get(prefix)
            if count is not None:
                stats[f"{prefix}_given_low_rate"] = (
                    float(count) / float(low_count)
                    if low_count > 0
                    else 0.0
                )

    if "tracking_error" in df.columns:
        tracking = pd.to_numeric(df["tracking_error"], errors="coerce").dropna()
        tracking = tracking[np.isfinite(tracking)]
        if not tracking.empty:
            stats["tracking_error_step_rms"] = float(np.sqrt(np.mean(tracking * tracking)))
            stats["tracking_error_step_p95"] = float(tracking.quantile(0.95))

    if "core_time_ms" in df.columns:
        core = pd.to_numeric(df["core_time_ms"], errors="coerce")
    elif {"feedback_time_ms", "preparation_time_ms"}.issubset(df.columns):
        core = pd.to_numeric(df["feedback_time_ms"], errors="coerce") + pd.to_numeric(
            df["preparation_time_ms"], errors="coerce"
        )
    else:
        return stats, uav_rows

    core = core[np.isfinite(core)]
    if core.empty:
        return stats, uav_rows
    over20 = core > 20.0
    stats.update(
        {
            "core_time_mean_ms": float(core.mean()),
            "core_time_p95_ms": float(core.quantile(0.95)),
            "core_time_max_ms": float(core.max()),
            "core_time_over_20ms_count": int(over20.sum()),
            "core_time_over_20ms_rate": float(over20.mean()),
        }
    )

    if "uav_idx" in df.columns:
        df_uav = df.copy()
        df_uav["uav_idx"] = pd.to_numeric(df_uav["uav_idx"], errors="coerce")
        df_uav["core_time_ms_effective"] = (
            pd.to_numeric(df_uav["core_time_ms"], errors="coerce")
            if "core_time_ms" in df_uav.columns
            else pd.to_numeric(df_uav["feedback_time_ms"], errors="coerce")
            + pd.to_numeric(df_uav["preparation_time_ms"], errors="coerce")
        )
        uav_p95_values: List[float] = []
        for uav_idx, group in df_uav.dropna(subset=["uav_idx"]).groupby("uav_idx"):
            core_uav = pd.to_numeric(
                group["core_time_ms_effective"], errors="coerce"
            ).dropna()
            core_uav = core_uav[np.isfinite(core_uav)]
            if core_uav.empty:
                continue
            uav_p95 = float(core_uav.quantile(0.95))
            uav_p95_values.append(uav_p95)
            uav_rows.append(
                {
                    **base,
                    "uav_idx": int(uav_idx),
                    "command_count": int(len(core_uav)),
                    "core_time_mean_ms": float(core_uav.mean()),
                    "core_time_p95_ms": uav_p95,
                    "core_time_max_ms": float(core_uav.max()),
                    "core_time_over_20ms_count": int((core_uav > 20.0).sum()),
                    "core_time_over_20ms_rate": float((core_uav > 20.0).mean()),
                }
            )
        if uav_p95_values:
            stats["core_time_worst_uav_p95_ms"] = max(uav_p95_values)

    return stats, uav_rows


def compute_horizon_stats(
    summary_path: Path,
    solver_obstacle_drift_path: Optional[Path] = None,
) -> Dict[str, object]:
    stats: Dict[str, object] = {
        "horizon_rows": 0,
        "prediction_horizon_rms_xy_mean": math.nan,
        "prediction_horizon_rms_xy_p95": math.nan,
        "prediction_horizon_max_xy_mean": math.nan,
        "prediction_horizon_final_xy_mean": math.nan,
        "eq17_boundary_drift_rms_mean": math.nan,
        "eq17_boundary_drift_rms_p95": math.nan,
        "eq17_boundary_drift_max": math.nan,
    }
    columns = [
        "horizon_rms_xy",
        "horizon_max_xy",
        "horizon_final_xy",
        "boundary_drift_rms",
        "boundary_drift_max",
    ]
    if summary_path.is_file():
        df = read_csv_existing_columns(summary_path, columns)
        if not df.empty:
            stats["horizon_rows"] = int(len(df))
            for col in columns:
                if col in df.columns:
                    df[col] = pd.to_numeric(df[col], errors="coerce")

            if "horizon_rms_xy" in df.columns:
                values = df["horizon_rms_xy"].dropna()
                values = values[np.isfinite(values)]
                if not values.empty:
                    stats["prediction_horizon_rms_xy_mean"] = float(values.mean())
                    stats["prediction_horizon_rms_xy_p95"] = float(values.quantile(0.95))
            if "horizon_max_xy" in df.columns:
                values = df["horizon_max_xy"].dropna()
                values = values[np.isfinite(values)]
                if not values.empty:
                    stats["prediction_horizon_max_xy_mean"] = float(values.mean())
            if "horizon_final_xy" in df.columns:
                values = df["horizon_final_xy"].dropna()
                values = values[np.isfinite(values)]
                if not values.empty:
                    stats["prediction_horizon_final_xy_mean"] = float(values.mean())
            if "boundary_drift_rms" in df.columns:
                values = df["boundary_drift_rms"].dropna()
                values = values[np.isfinite(values)]
                if not values.empty:
                    stats["eq17_boundary_drift_rms_mean"] = float(values.mean())
                    stats["eq17_boundary_drift_rms_p95"] = float(values.quantile(0.95))
            if "boundary_drift_max" in df.columns:
                values = df["boundary_drift_max"].dropna()
                values = values[np.isfinite(values)]
                if not values.empty:
                    stats["eq17_boundary_drift_max"] = float(values.max())

    if solver_obstacle_drift_path is not None and solver_obstacle_drift_path.is_file():
        drift_cols = ["uav_idx", "step_idx", "diff_norm_xy"]
        drift_df = read_csv_existing_columns(solver_obstacle_drift_path, drift_cols)
        if all(col in drift_df.columns for col in drift_cols) and not drift_df.empty:
            for col in drift_cols:
                drift_df[col] = pd.to_numeric(drift_df[col], errors="coerce")
            drift_df = drift_df.dropna(subset=drift_cols)
            drift_df = drift_df[np.isfinite(drift_df["diff_norm_xy"])]
            if not drift_df.empty:
                grouped_rms = drift_df.groupby(["uav_idx", "step_idx"])["diff_norm_xy"].apply(
                    lambda s: math.sqrt(float(np.mean(np.square(s.to_numpy(dtype=float)))))
                )
                grouped_rms = grouped_rms.dropna()
                grouped_rms = grouped_rms[np.isfinite(grouped_rms)]
                if not grouped_rms.empty:
                    stats["eq17_boundary_drift_rms_mean"] = float(grouped_rms.mean())
                    stats["eq17_boundary_drift_rms_p95"] = float(grouped_rms.quantile(0.95))
                    stats["eq17_boundary_drift_max"] = float(drift_df["diff_norm_xy"].max())
    return stats


def compute_eq18_stats(
    metrics_row: Optional[pd.Series],
    step_path: Path,
    horizon_path: Path,
) -> Dict[str, object]:
    stats: Dict[str, object] = {
        "eq18_anchor_count": 0,
        "eq18_future_sample_count": 0,
        "eq18_frot_mismatch_rms_mean": math.nan,
        "eq18_frot_mismatch_rms_p95": math.nan,
        "eq18_frot_mismatch_max": math.nan,
        "eq18_true_frot_norm_mean": math.nan,
        "eq18_pred_frot_norm_mean": math.nan,
    }
    if metrics_row is None or not step_path.is_file() or not horizon_path.is_file():
        return stats

    prediction_dt = metric_value(metrics_row, "prediction_dt_sec")
    prediction_steps = safe_int(metrics_row.get("prediction_steps", 0), default=0)
    if not (math.isfinite(prediction_dt) and prediction_dt > 0.0 and prediction_steps > 1):
        return stats

    try:
        step_df = read_csv_existing_columns(step_path, STEP_REQUIRED_EQ18)
        horizon_df = read_csv_existing_columns(horizon_path, HORIZON_REQUIRED_EQ18)
    except Exception:
        return stats
    if any(col not in step_df.columns for col in STEP_REQUIRED_EQ18):
        return stats
    if any(col not in horizon_df.columns for col in HORIZON_REQUIRED_EQ18):
        return stats

    for col in STEP_REQUIRED_EQ18:
        step_df[col] = pd.to_numeric(step_df[col], errors="coerce")
    for col in HORIZON_REQUIRED_EQ18:
        horizon_df[col] = pd.to_numeric(horizon_df[col], errors="coerce")
    step_df = step_df.dropna(subset=["uav_idx", "step_idx", "sim_time"])
    horizon_df = horizon_df.dropna(subset=["step_idx", "horizon_idx"])
    if step_df.empty or horizon_df.empty:
        return stats

    horizon_df["step_idx"] = horizon_df["step_idx"].astype(int)
    horizon_df["horizon_idx"] = horizon_df["horizon_idx"].astype(int)
    horizon_by_stage = {
        int(stage): group[["step_idx", "pred_omega_non_x", "pred_omega_non_y", "pred_omega_non_z"]]
        .drop_duplicates(subset=["step_idx"], keep="last")
        .copy()
        for stage, group in horizon_df.groupby("horizon_idx")
    }

    rms_values: List[float] = []
    max_values: List[float] = []
    true_norm_values: List[float] = []
    pred_norm_values: List[float] = []
    future_sample_count = 0

    for _uav_idx, group in step_df.groupby("uav_idx"):
        group = group.sort_values("step_idx").reset_index(drop=True)
        if len(group) < 2:
            continue
        pos = group[
            [
                "rot_load_position_non_x",
                "rot_load_position_non_y",
                "rot_load_position_non_z",
            ]
        ].to_numpy(dtype=float)
        vel = group[
            [
                "rot_load_velocity_non_x",
                "rot_load_velocity_non_y",
                "rot_load_velocity_non_z",
            ]
        ].to_numpy(dtype=float)
        omega = group[
            [
                "rot_load_omega_non_x",
                "rot_load_omega_non_y",
                "rot_load_omega_non_z",
            ]
        ].to_numpy(dtype=float)
        sim_time = group["sim_time"].to_numpy(dtype=float)
        step_idx = group["step_idx"].to_numpy(dtype=int)
        dt_values = np.diff(sim_time)
        dt_values = dt_values[np.isfinite(dt_values) & (dt_values > 0.0)]
        if dt_values.size == 0:
            continue
        sim_dt = float(np.median(dt_values))
        if not (math.isfinite(sim_dt) and sim_dt > 0.0):
            continue
        stage_stride = int(round(prediction_dt / sim_dt))
        if stage_stride <= 0:
            continue

        index_by_step = {int(step): idx for idx, step in enumerate(step_idx)}
        true_frot_all = compute_frot(omega, vel, pos)
        mismatch_sq_sum = np.zeros(len(group), dtype=float)
        mismatch_sum = np.zeros(len(group), dtype=float)
        mismatch_max = np.zeros(len(group), dtype=float)
        true_norm_sum = np.zeros(len(group), dtype=float)
        pred_norm_sum = np.zeros(len(group), dtype=float)
        count = np.zeros(len(group), dtype=int)
        anchors = pd.DataFrame(
            {
                "anchor_step": step_idx.astype(int),
                "anchor_local_idx": np.arange(len(group), dtype=int),
            }
        )

        for stage_idx in range(1, prediction_steps):
            h_stage = horizon_by_stage.get(stage_idx)
            if h_stage is None or h_stage.empty:
                continue
            merged = anchors.merge(
                h_stage,
                left_on="anchor_step",
                right_on="step_idx",
                how="inner",
            )
            if merged.empty:
                continue
            future_steps = merged["anchor_step"].to_numpy(dtype=int) + stage_stride * stage_idx
            future_idx = np.array(
                [index_by_step.get(int(step), -1) for step in future_steps],
                dtype=int,
            )
            anchor_idx = merged["anchor_local_idx"].to_numpy(dtype=int)
            valid = future_idx >= 0
            if not np.any(valid):
                continue
            future_idx = future_idx[valid]
            anchor_idx = anchor_idx[valid]
            expected_time = sim_time[anchor_idx] + stage_idx * prediction_dt
            time_ok = np.abs(sim_time[future_idx] - expected_time) <= max(
                1e-6, 0.25 * prediction_dt
            )
            if not np.any(time_ok):
                continue
            future_idx = future_idx[time_ok]
            anchor_idx = anchor_idx[time_ok]
            pred_omega = merged.loc[
                valid,
                ["pred_omega_non_x", "pred_omega_non_y", "pred_omega_non_z"],
            ].to_numpy(dtype=float)[time_ok]

            pred_frot = compute_frot(pred_omega, vel[future_idx], pos[future_idx])
            true_frot = true_frot_all[future_idx]
            mismatch_norm = np.linalg.norm(pred_frot - true_frot, axis=1)
            true_norm = np.linalg.norm(true_frot, axis=1)
            pred_norm = np.linalg.norm(pred_frot, axis=1)

            np.add.at(mismatch_sq_sum, anchor_idx, mismatch_norm * mismatch_norm)
            np.add.at(mismatch_sum, anchor_idx, mismatch_norm)
            np.maximum.at(mismatch_max, anchor_idx, mismatch_norm)
            np.add.at(true_norm_sum, anchor_idx, true_norm)
            np.add.at(pred_norm_sum, anchor_idx, pred_norm)
            np.add.at(count, anchor_idx, 1)

        valid_anchor = count > 0
        if not np.any(valid_anchor):
            continue
        future_sample_count += int(count[valid_anchor].sum())
        rms_values.extend(np.sqrt(mismatch_sq_sum[valid_anchor] / count[valid_anchor]).tolist())
        max_values.extend(mismatch_max[valid_anchor].tolist())
        true_norm_values.extend((true_norm_sum[valid_anchor] / count[valid_anchor]).tolist())
        pred_norm_values.extend((pred_norm_sum[valid_anchor] / count[valid_anchor]).tolist())

    if not rms_values:
        return stats
    rms_series = finite_series(rms_values)
    max_series = finite_series(max_values)
    stats.update(
        {
            "eq18_anchor_count": int(len(rms_series)),
            "eq18_future_sample_count": int(future_sample_count),
            "eq18_frot_mismatch_rms_mean": float(rms_series.mean()),
            "eq18_frot_mismatch_rms_p95": float(rms_series.quantile(0.95)),
            "eq18_frot_mismatch_max": float(max_series.max()) if not max_series.empty else math.nan,
            "eq18_true_frot_norm_mean": mean_value(true_norm_values),
            "eq18_pred_frot_norm_mean": mean_value(pred_norm_values),
        }
    )
    return stats


def manifest_cases(run_root: Path) -> List[Dict[str, str]]:
    manifest = run_root / "run_manifest.csv"
    if not manifest.is_file():
        return []
    latest: Dict[Tuple[str, str, str, str, str], Dict[str, str]] = {}
    with manifest.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            key = (
                row.get("controller", ""),
                row.get("obstacle_sensing_mode", ""),
                row.get("v", ""),
                row.get("w", ""),
                row.get("seed", ""),
            )
            latest[key] = row
    return list(latest.values())


def discovered_cases(run_root: Path) -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []
    for metrics_path in sorted(run_root.rglob("metrics.csv")):
        row = scope_all_row(metrics_path)
        try:
            seed_part = metrics_path.parent.name
            w_part = metrics_path.parent.parent.name
            v_part = metrics_path.parent.parent.parent.name
            controller = metrics_path.parent.parent.parent.parent.name
            seed = int(seed_part.replace("seed_", ""))
            v = q_label_to_float(v_part, "v_")
            w = q_label_to_float(w_part, "w_")
        except Exception:
            if row is None:
                continue
            frame = metric_text(row, "frame_mode_effective")
            rollout = metric_text(row, "ugv_rollout_mode")
            controller = controller_from_modes(frame, rollout)
            seed = safe_int(row.get("seed", 0), default=0)
            v = metric_value(row, "requested_v")
            w = metric_value(row, "requested_w")
        rows.append(
            {
                "controller": controller,
                "obstacle_sensing_mode": metric_text(
                    row, "obstacle_sensing_mode", "map"
                ),
                "v": str(v),
                "w": str(w),
                "seed": str(seed),
                "obs": "",
                "status": "ok",
                "metrics_csv": str(metrics_path),
                "step_metrics_csv": str(metrics_path.with_name("metrics_steps.csv")),
                "ugv_horizon_summary_csv": str(
                    metrics_path.with_name("metrics_ugv_horizon_summary.csv")
                ),
                "ugv_horizon_steps_csv": str(
                    metrics_path.with_name("metrics_ugv_horizon_steps.csv")
                ),
            }
        )
    return rows


def process_case(
    run_root: Path,
    case: Dict[str, str],
) -> Tuple[Dict[str, object], List[Dict[str, object]]]:
    metrics_path = resolve_path(run_root, case.get("metrics_csv", ""))
    step_path = resolve_path(
        run_root,
        case.get("step_metrics_csv", str(metrics_path.with_name("metrics_steps.csv"))),
    )
    horizon_summary_path = resolve_path(
        run_root,
        case.get(
            "ugv_horizon_summary_csv",
            str(metrics_path.with_name("metrics_ugv_horizon_summary.csv")),
        ),
    )
    horizon_steps_path = resolve_path(
        run_root,
        case.get(
            "ugv_horizon_steps_csv",
            str(metrics_path.with_name("metrics_ugv_horizon_steps.csv")),
        ),
    )
    solver_obstacle_drift_path = resolve_path(
        run_root,
        case.get(
            "solver_obstacle_drift_csv",
            str(metrics_path.with_name("metrics_solver_obstacle_drift.csv")),
        ),
    )
    metrics_row = scope_all_row(metrics_path)
    controller = case.get("controller", "")
    frame_mode = metric_text(metrics_row, "frame_mode_effective", case.get("frame_mode", ""))
    rollout = metric_text(metrics_row, "ugv_rollout_mode", case.get("ugv_rollout_mode", ""))
    obstacle_sensing_mode = metric_text(
        metrics_row,
        "obstacle_sensing_mode",
        case.get("obstacle_sensing_mode", "map"),
    )
    if not controller and frame_mode and rollout:
        controller = controller_from_modes(frame_mode, rollout)

    base: Dict[str, object] = {
        "controller": controller,
        "frame_mode_effective": frame_mode,
        "ugv_rollout_mode": rollout,
        "stage_geometry_mode": case.get("stage_geometry_mode", "unrecorded"),
        "obstacle_sensing_mode": obstacle_sensing_mode,
        "v": metric_value(metrics_row, "requested_v")
        if metrics_row is not None
        else safe_float(case.get("v", "")),
        "w": metric_value(metrics_row, "requested_w")
        if metrics_row is not None
        else safe_float(case.get("w", "")),
        "seed": safe_int(metric_text(metrics_row, "seed", case.get("seed", "0"))),
        "obs": safe_int(metric_text(metrics_row, "obstacle_count", case.get("obs", "0"))),
        "status": case.get("status", "ok" if metrics_row is not None else "missing"),
        "run_tag": metric_text(metrics_row, "run_tag", case.get("run_tag", "")),
        "metrics_csv": str(metrics_path),
    }
    if metrics_row is None:
        row: Dict[str, object] = {
            **base,
            "tracking_rms": math.nan,
            "tracking_p95": math.nan,
            "collision": math.nan,
            "solver_min_h": math.nan,
            "solver_slack_sum": math.nan,
            "solver_max_slack": math.nan,
            "solver_fail_rate": math.nan,
            "dmin_lin": math.nan,
            "truth_static_min_surface_clearance": math.nan,
        }
        step_stats, uav_rows = compute_step_stats(step_path, base)
        row.update(step_stats)
        row.update(compute_horizon_stats(horizon_summary_path, solver_obstacle_drift_path))
        row.update(compute_eq18_stats(metrics_row, step_path, horizon_steps_path))
        return row, uav_rows

    row = {
        **base,
        "tracking_rms": metric_value(metrics_row, "tracking_rms"),
        "tracking_p95": metric_value(metrics_row, "tracking_p95"),
        "collision": safe_int(metrics_row.get("collision", 0), default=0),
        "min_h": metric_value(metrics_row, "min_h"),
        "solver_min_h": metric_value(metrics_row, "solver_min_h", "min_h"),
        "solver_min_planar_clearance": metric_value(
            metrics_row, "solver_min_planar_clearance"
        ),
        "solver_slack_sum": metric_value(metrics_row, "solver_slack_sum", "slack_sum"),
        "solver_max_slack": metric_value(metrics_row, "solver_max_slack", "max_slack"),
        "solver_fail_rate": metric_value(metrics_row, "solver_fail_rate", "fail_rate"),
        "dmin_lin": metric_value(metrics_row, "dmin_lin"),
        "truth_static_min_surface_clearance": metric_value(
            metrics_row, "truth_static_min_surface_clearance"
        ),
    }
    step_stats, uav_rows = compute_step_stats(step_path, base)
    row.update(step_stats)
    row.update(compute_horizon_stats(horizon_summary_path, solver_obstacle_drift_path))
    row.update(compute_eq18_stats(metrics_row, step_path, horizon_steps_path))
    return row, uav_rows


def build_summary(df: pd.DataFrame) -> pd.DataFrame:
    rows: List[Dict[str, object]] = []
    if df.empty:
        return pd.DataFrame()
    for (v, w, controller, stage_geometry_mode, obstacle_sensing_mode), group in df.groupby(
        ["v", "w", "controller", "stage_geometry_mode", "obstacle_sensing_mode"],
        dropna=False,
    ):
        collision = pd.to_numeric(group.get("collision"), errors="coerce")
        rows.append(
            {
                "v": v,
                "w": w,
                "controller": controller,
                "stage_geometry_mode": stage_geometry_mode,
                "obstacle_sensing_mode": obstacle_sensing_mode,
                "seed_count": int(group["seed"].nunique()),
                "row_count": int(len(group)),
                "ok_count": int(group["status"].isin(["ok", "skipped"]).sum()),
                "collision_count": int(collision.fillna(0).sum()),
                "collision_rate": float(collision.mean()) if len(collision.dropna()) else math.nan,
                "tracking_rms_mean": mean_value(group["tracking_rms"]),
                "tracking_rms_std": std_value(group["tracking_rms"]),
                "tracking_rms_p95": p95_value(group["tracking_rms"]),
                "tracking_p95_mean": mean_value(group["tracking_p95"]),
                "prediction_horizon_rms_xy_mean": mean_value(
                    group["prediction_horizon_rms_xy_mean"]
                ),
                "prediction_horizon_rms_xy_p95": p95_value(
                    group["prediction_horizon_rms_xy_mean"]
                ),
                "eq17_boundary_drift_rms_mean": mean_value(
                    group["eq17_boundary_drift_rms_mean"]
                ),
                "eq17_boundary_drift_rms_p95": p95_value(
                    group["eq17_boundary_drift_rms_mean"]
                ),
                "eq18_frot_mismatch_rms_mean": mean_value(
                    group["eq18_frot_mismatch_rms_mean"]
                ),
                "eq18_frot_mismatch_rms_p95": p95_value(
                    group["eq18_frot_mismatch_rms_mean"]
                ),
                "solver_min_h_min": min_value(group["solver_min_h"]),
                "solver_min_h_mean": mean_value(group["solver_min_h"]),
                "solver_slack_sum_mean": mean_value(group["solver_slack_sum"]),
                "solver_slack_sum_total": sum_value(group["solver_slack_sum"]),
                "truth_static_min_surface_clearance_min": min_value(
                    group["truth_static_min_surface_clearance"]
                ),
                "truth_static_min_surface_clearance_mean": mean_value(
                    group["truth_static_min_surface_clearance"]
                ),
                "sensing_dropout_observed_rate_mean": mean_value(
                    group["sensing_dropout_observed_rate"]
                ),
                "sensing_active_tracks_mean": mean_value(
                    group["sensing_active_tracks_mean"]
                ),
                "sensing_active_tracks_nonzero_rate_mean": mean_value(
                    group["sensing_active_tracks_nonzero_rate"]
                ),
                "sensing_low_clearance_samples_total": sum_value(
                    group["sensing_low_clearance_samples"]
                ),
                "sensing_low_clearance_episodes_total": sum_value(
                    group["sensing_low_clearance_episodes"]
                ),
                "sensing_low_clearance_rate_mean": mean_value(
                    group["sensing_low_clearance_rate"]
                ),
                "sensing_low_clearance_no_track_samples_total": sum_value(
                    group["sensing_low_clearance_no_track_samples"]
                ),
                "sensing_low_clearance_no_track_episodes_total": sum_value(
                    group["sensing_low_clearance_no_track_episodes"]
                ),
                "sensing_low_clearance_no_track_given_low_rate_mean": mean_value(
                    group["sensing_low_clearance_no_track_given_low_rate"]
                ),
                "sensing_low_clearance_current_dropout_samples_total": sum_value(
                    group["sensing_low_clearance_current_dropout_samples"]
                ),
                "sensing_low_clearance_current_dropout_episodes_total": sum_value(
                    group["sensing_low_clearance_current_dropout_episodes"]
                ),
                "sensing_low_clearance_current_dropout_given_low_rate_mean": mean_value(
                    group[
                        "sensing_low_clearance_current_dropout_given_low_rate"
                    ]
                ),
                "sensing_low_clearance_pending_no_track_samples_total": sum_value(
                    group["sensing_low_clearance_pending_no_track_samples"]
                ),
                "sensing_low_clearance_pending_no_track_episodes_total": sum_value(
                    group["sensing_low_clearance_pending_no_track_episodes"]
                ),
                "sensing_low_clearance_pending_no_track_given_low_rate_mean": mean_value(
                    group[
                        "sensing_low_clearance_pending_no_track_given_low_rate"
                    ]
                ),
                "sensing_low_clearance_stale_track_samples_total": sum_value(
                    group["sensing_low_clearance_stale_track_samples"]
                ),
                "sensing_low_clearance_stale_track_episodes_total": sum_value(
                    group["sensing_low_clearance_stale_track_episodes"]
                ),
                "sensing_low_clearance_stale_track_given_low_rate_mean": mean_value(
                    group["sensing_low_clearance_stale_track_given_low_rate"]
                ),
                "core_time_mean_ms_mean": mean_value(group["core_time_mean_ms"]),
                "core_time_p95_ms_mean": mean_value(group["core_time_p95_ms"]),
                "core_time_worst_uav_p95_ms_max": max_value(
                    group["core_time_worst_uav_p95_ms"]
                ),
                "core_time_max_ms_max": max_value(group["core_time_max_ms"]),
                "core_time_over_20ms_rate_mean": mean_value(
                    group["core_time_over_20ms_rate"]
                ),
            }
        )
    summary = pd.DataFrame(rows)
    return summary.sort_values(
        ["v", "w", "controller", "obstacle_sensing_mode"]
    ).reset_index(drop=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Aggregate V/W three-controller experiment outputs."
    )
    parser.add_argument("run_root", type=Path)
    parser.add_argument("--out-dir", type=Path, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_root = args.run_root.resolve()
    out_dir = args.out_dir.resolve() if args.out_dir else run_root / "tables"
    out_dir.mkdir(parents=True, exist_ok=True)

    cases = manifest_cases(run_root)
    if not cases:
        cases = discovered_cases(run_root)
    if not cases:
        raise SystemExit(f"No experiment cases found under {run_root}")

    per_seed_rows: List[Dict[str, object]] = []
    uav_rows: List[Dict[str, object]] = []
    for case in cases:
        row, case_uav_rows = process_case(run_root, case)
        per_seed_rows.append(row)
        uav_rows.extend(case_uav_rows)

    per_seed = pd.DataFrame(per_seed_rows).sort_values(
        ["v", "w", "controller", "obstacle_sensing_mode", "seed"],
        kind="stable",
    )
    summary = build_summary(per_seed)
    core_by_uav = pd.DataFrame(uav_rows)
    if not core_by_uav.empty:
        core_by_uav = core_by_uav.sort_values(
            ["v", "w", "controller", "obstacle_sensing_mode", "seed", "uav_idx"],
            kind="stable",
        )

    per_seed_path = out_dir / "per_seed_controller_metrics.csv"
    summary_path = out_dir / "summary_by_vw_controller.csv"
    core_uav_path = out_dir / "core_time_by_seed_uav.csv"
    per_seed.to_csv(per_seed_path, index=False)
    summary.to_csv(summary_path, index=False)
    core_by_uav.to_csv(core_uav_path, index=False)

    print(f"[INFO] wrote {per_seed_path}")
    print(f"[INFO] wrote {summary_path}")
    print(f"[INFO] wrote {core_uav_path}")
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
