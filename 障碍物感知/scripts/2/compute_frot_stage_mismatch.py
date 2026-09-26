#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import numpy as np
import pandas as pd

STEP_SUFFIX = "_metrics_steps.csv"
HORIZON_SUFFIX = "_metrics_ugv_horizon_steps.csv"
REQUIRED_STEP_COLUMNS = (
    "uav_idx",
    "step_idx",
    "sim_time",
    "frame_mode_effective",
    "ugv_rollout_mode",
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
REQUIRED_HORIZON_COLUMNS = (
    "step_idx",
    "horizon_idx",
    "pred_omega_non_x",
    "pred_omega_non_y",
    "pred_omega_non_z",
)


def infer_kp(path: Path) -> float | None:
    for part in path.parts:
        match = re.fullmatch(r"kp_(\d+)p(\d+)", part)
        if match:
            return float(f"{match.group(1)}.{match.group(2)}")
    return None


def infer_seed(path: Path) -> int | None:
    for part in path.parts:
        match = re.fullmatch(r"seed_(\d+)", part)
        if match:
            return int(match.group(1))
    return None


def compute_frot(omega: np.ndarray, velocity: np.ndarray, position: np.ndarray) -> np.ndarray:
    return 2.0 * np.cross(omega, velocity) + np.cross(omega, np.cross(omega, position))


def q95(series: pd.Series) -> float:
    return float(series.quantile(0.95))


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


def load_scope_all(metrics_path: Path) -> tuple[float, int]:
    df = pd.read_csv(metrics_path)
    if "scope" not in df.columns:
        raise RuntimeError(f"Missing scope column in {metrics_path}")
    scope_all = df[df["scope"] == "all"]
    if scope_all.empty:
        raise RuntimeError(f"No scope=all row in {metrics_path}")
    row = scope_all.iloc[0]
    prediction_dt = float(row["prediction_dt_sec"])
    prediction_steps = int(float(row["prediction_steps"]))
    if not (math.isfinite(prediction_dt) and prediction_dt > 0.0):
        raise RuntimeError(f"Invalid prediction_dt_sec in {metrics_path}: {prediction_dt}")
    if prediction_steps <= 1:
        raise RuntimeError(f"Invalid prediction_steps in {metrics_path}: {prediction_steps}")
    return prediction_dt, prediction_steps


def discover_stage_step_files(run_roots: list[Path]) -> list[Path]:
    candidates: list[Path] = []
    for root in run_roots:
        for path in sorted(root.rglob(f"*{STEP_SUFFIX}")):
            if path.name.endswith(HORIZON_SUFFIX):
                continue
            candidates.append(path)
    return candidates


def is_stage_step_file(step_path: Path) -> bool:
    df = pd.read_csv(step_path, nrows=1)
    missing = [col for col in ("frame_mode_effective", "ugv_rollout_mode") if col not in df.columns]
    if missing or df.empty:
        return False
    row = df.iloc[0]
    return (
        str(row.get("frame_mode_effective", "")) == "noninertial"
        and str(row.get("ugv_rollout_mode", "")) == "stage"
    )


def load_steps(step_path: Path) -> pd.DataFrame:
    df = pd.read_csv(step_path, usecols=list(REQUIRED_STEP_COLUMNS))
    missing = [col for col in REQUIRED_STEP_COLUMNS if col not in df.columns]
    if missing:
        raise RuntimeError(f"Missing required columns in {step_path}: {missing}")
    for col in REQUIRED_STEP_COLUMNS:
        if col not in {"frame_mode_effective", "ugv_rollout_mode"}:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df[
        (df["frame_mode_effective"] == "noninertial")
        & (df["ugv_rollout_mode"] == "stage")
    ].copy()
    return df.dropna(subset=["uav_idx", "step_idx", "sim_time"]).copy()


def load_horizon_steps(horizon_path: Path) -> pd.DataFrame:
    header_df = pd.read_csv(horizon_path, nrows=0)
    missing = [col for col in REQUIRED_HORIZON_COLUMNS if col not in header_df.columns]
    if missing:
        raise RuntimeError(
            "Missing required columns in "
            f"{horizon_path}: {missing}. "
            "Re-run noninertial stage with the updated logger first."
        )
    df = pd.read_csv(horizon_path, usecols=list(REQUIRED_HORIZON_COLUMNS))
    for col in REQUIRED_HORIZON_COLUMNS:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df.dropna(subset=["step_idx", "horizon_idx"]).copy()


def compute_seed_rows(step_path: Path) -> list[dict]:
    metrics_path = step_path.with_name(step_path.name.replace("_steps.csv", ".csv"))
    horizon_path = step_path.with_name(
        step_path.name.replace("_steps.csv", "_ugv_horizon_steps.csv")
    )
    if not metrics_path.is_file() or not horizon_path.is_file():
        return []

    prediction_dt, prediction_steps = load_scope_all(metrics_path)
    step_df = load_steps(step_path)
    horizon_df = load_horizon_steps(horizon_path)
    if step_df.empty or horizon_df.empty:
        return []

    kp = infer_kp(step_path)
    seed = infer_seed(step_path)
    if kp is None or seed is None:
        return []

    time_tol = max(1e-6, 0.25 * prediction_dt)
    rows: list[dict] = []

    horizon_df["step_idx"] = horizon_df["step_idx"].astype(int)
    horizon_df["horizon_idx"] = horizon_df["horizon_idx"].astype(int)
    omega_lookup = {
        (int(row.step_idx), int(row.horizon_idx)): np.array(
            [row.pred_omega_non_x, row.pred_omega_non_y, row.pred_omega_non_z],
            dtype=float,
        )
        for row in horizon_df.itertuples(index=False)
    }

    for uav_idx, df_uav in step_df.groupby("uav_idx"):
        df_uav = df_uav.sort_values("step_idx").reset_index(drop=True)
        pos = df_uav[
            ["rot_load_position_non_x", "rot_load_position_non_y", "rot_load_position_non_z"]
        ].to_numpy(dtype=float)
        vel = df_uav[
            ["rot_load_velocity_non_x", "rot_load_velocity_non_y", "rot_load_velocity_non_z"]
        ].to_numpy(dtype=float)
        omg = df_uav[
            ["rot_load_omega_non_x", "rot_load_omega_non_y", "rot_load_omega_non_z"]
        ].to_numpy(dtype=float)
        sim_time = df_uav["sim_time"].to_numpy(dtype=float)
        step_idx = df_uav["step_idx"].to_numpy(dtype=int)

        if sim_time.size < 2:
            continue
        sim_dt = float(np.median(np.diff(sim_time)))
        if not math.isfinite(sim_dt) or sim_dt <= 0.0:
            continue

        stage_stride = int(round(prediction_dt / sim_dt))
        if stage_stride <= 0:
            continue

        index_by_step = {int(s): i for i, s in enumerate(step_idx)}
        true_frot_all = compute_frot(omg, vel, pos)
        mismatch_sq_sum = np.zeros(sim_time.size, dtype=float)
        mismatch_sum = np.zeros(sim_time.size, dtype=float)
        mismatch_max = np.zeros(sim_time.size, dtype=float)
        true_norm_sum = np.zeros(sim_time.size, dtype=float)
        pred_norm_sum = np.zeros(sim_time.size, dtype=float)
        count = np.zeros(sim_time.size, dtype=int)

        for stage_idx in range(1, prediction_steps):
            offset = stage_stride * stage_idx
            for anchor_local_idx, anchor_step in enumerate(step_idx):
                pred_omega = omega_lookup.get((int(anchor_step), stage_idx))
                if pred_omega is None:
                    continue
                future_step = int(anchor_step) + offset
                future_local_idx = index_by_step.get(future_step)
                if future_local_idx is None:
                    continue
                expected_future_time = sim_time[anchor_local_idx] + stage_idx * prediction_dt
                if abs(sim_time[future_local_idx] - expected_future_time) > time_tol:
                    continue

                pred_frot = compute_frot(
                    pred_omega.reshape(1, 3),
                    vel[future_local_idx].reshape(1, 3),
                    pos[future_local_idx].reshape(1, 3),
                )[0]
                true_frot = true_frot_all[future_local_idx]
                mismatch_norm = float(np.linalg.norm(pred_frot - true_frot))
                true_norm = float(np.linalg.norm(true_frot))
                pred_norm = float(np.linalg.norm(pred_frot))

                mismatch_sq_sum[anchor_local_idx] += mismatch_norm * mismatch_norm
                mismatch_sum[anchor_local_idx] += mismatch_norm
                mismatch_max[anchor_local_idx] = max(
                    mismatch_max[anchor_local_idx], mismatch_norm
                )
                true_norm_sum[anchor_local_idx] += true_norm
                pred_norm_sum[anchor_local_idx] += pred_norm
                count[anchor_local_idx] += 1

        valid_anchor_idx = np.flatnonzero(count > 0)
        for anchor_local_idx in valid_anchor_idx:
            used_steps = int(count[anchor_local_idx])
            rows.append(
                {
                    "kp": kp,
                    "seed": seed,
                    "uav_idx": int(uav_idx),
                    "anchor_step_idx": int(step_idx[anchor_local_idx]),
                    "anchor_sim_time": float(sim_time[anchor_local_idx]),
                    "future_samples": used_steps,
                    "frot_stage_mismatch_rms": float(
                        np.sqrt(mismatch_sq_sum[anchor_local_idx] / used_steps)
                    ),
                    "frot_stage_mismatch_mean": float(
                        mismatch_sum[anchor_local_idx] / used_steps
                    ),
                    "frot_stage_mismatch_max": float(mismatch_max[anchor_local_idx]),
                    "true_frot_mean_norm": float(true_norm_sum[anchor_local_idx] / used_steps),
                    "pred_frot_mean_norm": float(pred_norm_sum[anchor_local_idx] / used_steps),
                }
            )
    return rows


def build_summary(df: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    per_seed = (
        df.groupby(["kp", "seed"], dropna=False)
        .agg(
            anchor_count=("frot_stage_mismatch_rms", "size"),
            mean_frot_stage_mismatch_rms=("frot_stage_mismatch_rms", "mean"),
            p95_frot_stage_mismatch_rms=("frot_stage_mismatch_rms", q95),
            mean_frot_stage_mismatch_max=("frot_stage_mismatch_max", "mean"),
            mean_true_frot_norm=("true_frot_mean_norm", "mean"),
            mean_pred_frot_norm=("pred_frot_mean_norm", "mean"),
        )
        .reset_index()
        .sort_values(["kp", "seed"])
    )

    summary = (
        df.groupby(["kp"], dropna=False)
        .agg(
            seed_count=("seed", "nunique"),
            anchor_count=("frot_stage_mismatch_rms", "size"),
            mean_frot_stage_mismatch_rms=("frot_stage_mismatch_rms", "mean"),
            p95_frot_stage_mismatch_rms=("frot_stage_mismatch_rms", q95),
            mean_frot_stage_mismatch_max=("frot_stage_mismatch_max", "mean"),
            mean_true_frot_norm=("true_frot_mean_norm", "mean"),
            mean_pred_frot_norm=("pred_frot_mean_norm", "mean"),
        )
        .reset_index()
        .sort_values(["kp"])
    )
    return summary, per_seed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compute stage-horizon f_rot mismatch for noninertial stage runs: "
            "true future f_rot vs the stage omega profile injected to MPC."
        )
    )
    parser.add_argument(
        "run_roots",
        nargs="+",
        type=Path,
        help="One or more run roots to scan recursively.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Output directory for summary CSVs. Defaults to the first run root.",
    )
    parser.add_argument(
        "--out-prefix",
        default="frot_stage_mismatch",
        help="Prefix for the generated CSV files.",
    )
    parser.add_argument(
        "--seed-filter-csv",
        type=Path,
        default=None,
        help="Optional CSV with kp,seed rows to restrict which seeds are kept.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_roots = [path.resolve() for path in args.run_roots]
    out_dir = args.out_dir.resolve() if args.out_dir else run_roots[0]
    out_dir.mkdir(parents=True, exist_ok=True)
    seed_filter = load_seed_filter(args.seed_filter_csv.resolve() if args.seed_filter_csv else None)

    rows: list[dict] = []
    skipped_missing_columns: list[str] = []
    for step_path in discover_stage_step_files(run_roots):
        try:
            if not is_stage_step_file(step_path):
                continue
            kp = infer_kp(step_path)
            seed = infer_seed(step_path)
            if kp is None or seed is None:
                continue
            if seed_filter and seed not in seed_filter.get(kp, set()):
                continue
            rows.extend(compute_seed_rows(step_path))
        except RuntimeError as exc:
            skipped_missing_columns.append(str(exc))

    if skipped_missing_columns:
        print("[WARN] skipped some files:")
        for line in skipped_missing_columns[:20]:
            print(f"  - {line}")
        if len(skipped_missing_columns) > 20:
            print(f"  ... {len(skipped_missing_columns) - 20} more")

    if not rows:
        raise SystemExit(
            "No stage mismatch rows were computed. "
            "Check that noninertial stage logs exist and horizon steps contain pred_omega_non_* columns."
        )

    df = pd.DataFrame(rows)
    summary, per_seed = build_summary(df)

    anchor_path = out_dir / f"{args.out_prefix}_anchor_rows.csv"
    per_seed_path = out_dir / f"{args.out_prefix}_by_seed.csv"
    summary_path = out_dir / f"{args.out_prefix}_summary_by_kp.csv"
    df.to_csv(anchor_path, index=False)
    per_seed.to_csv(per_seed_path, index=False)
    summary.to_csv(summary_path, index=False)

    print(f"[INFO] wrote {anchor_path}")
    print(f"[INFO] wrote {per_seed_path}")
    print(f"[INFO] wrote {summary_path}")
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
