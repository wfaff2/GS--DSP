#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import numpy as np
import pandas as pd

METRICS_NAME = "noninertial_metrics.csv"
STEP_NAME = "noninertial_metrics_steps.csv"
REQUIRED_STEP_COLUMNS = (
    "uav_idx",
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


def load_steps(step_path: Path) -> pd.DataFrame:
    df = pd.read_csv(step_path)
    missing = [col for col in REQUIRED_STEP_COLUMNS if col not in df.columns]
    if missing:
        raise RuntimeError(f"Missing required columns in {step_path}: {missing}")
    if "frame_mode_effective" in df.columns:
        df = df[df["frame_mode_effective"] == "noninertial"].copy()
    for col in REQUIRED_STEP_COLUMNS:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df.dropna(subset=["uav_idx", "sim_time"]).copy()


def compute_seed_rows(run_root: Path, seed_dir: Path) -> list[dict]:
    metrics_path = seed_dir / METRICS_NAME
    step_path = seed_dir / STEP_NAME
    if not metrics_path.is_file() or not step_path.is_file():
        return []

    prediction_dt, prediction_steps = load_scope_all(metrics_path)
    df = load_steps(step_path)
    if df.empty:
        return []

    seed = infer_seed(seed_dir)
    kp = infer_kp(seed_dir)
    time_tol = max(1e-6, 0.25 * prediction_dt)

    rows: list[dict] = []
    for uav_idx, df_uav in df.groupby("uav_idx"):
        df_uav = df_uav.sort_values("sim_time").reset_index(drop=True)
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

        if sim_time.size < 2:
            continue
        sim_dt = float(np.median(np.diff(sim_time)))
        if not math.isfinite(sim_dt) or sim_dt <= 0.0:
            continue

        stage_stride = int(round(prediction_dt / sim_dt))
        if stage_stride <= 0:
            continue

        max_offset = stage_stride * (prediction_steps - 1)
        n = sim_time.size
        if n <= max_offset:
            continue

        true_frot_all = compute_frot(omg, vel, pos)
        mismatch_sq_sum = np.zeros(n, dtype=float)
        mismatch_sum = np.zeros(n, dtype=float)
        mismatch_max = np.zeros(n, dtype=float)
        true_norm_sum = np.zeros(n, dtype=float)
        pred_norm_sum = np.zeros(n, dtype=float)
        count = np.zeros(n, dtype=int)

        for stage_idx in range(1, prediction_steps):
            offset = stage_idx * stage_stride
            valid = n - offset
            if valid <= 0:
                break

            anchor_idx = np.arange(valid, dtype=int)
            future_idx = anchor_idx + offset
            expected_future_time = sim_time[anchor_idx] + stage_idx * prediction_dt
            time_ok = np.abs(sim_time[future_idx] - expected_future_time) <= time_tol
            if not np.any(time_ok):
                continue

            anchor_idx = anchor_idx[time_ok]
            future_idx = future_idx[time_ok]

            pred_frot = compute_frot(omg[anchor_idx], vel[future_idx], pos[future_idx])
            true_frot = true_frot_all[future_idx]
            mismatch_norm = np.linalg.norm(pred_frot - true_frot, axis=1)
            true_norm = np.linalg.norm(true_frot, axis=1)
            pred_norm = np.linalg.norm(pred_frot, axis=1)

            mismatch_sq_sum[anchor_idx] += mismatch_norm * mismatch_norm
            mismatch_sum[anchor_idx] += mismatch_norm
            mismatch_max[anchor_idx] = np.maximum(mismatch_max[anchor_idx], mismatch_norm)
            true_norm_sum[anchor_idx] += true_norm
            pred_norm_sum[anchor_idx] += pred_norm
            count[anchor_idx] += 1

        valid_anchor_mask = count > 0
        if not np.any(valid_anchor_mask):
            continue

        valid_anchor_idx = np.flatnonzero(valid_anchor_mask)
        for anchor_idx in valid_anchor_idx:
            used_steps = int(count[anchor_idx])
            rows.append(
                {
                    "kp": kp,
                    "seed": seed,
                    "uav_idx": int(uav_idx),
                    "anchor_sim_time": float(sim_time[anchor_idx]),
                    "future_samples": used_steps,
                    "frot_mismatch_rms": float(np.sqrt(mismatch_sq_sum[anchor_idx] / used_steps)),
                    "frot_mismatch_mean": float(mismatch_sum[anchor_idx] / used_steps),
                    "frot_mismatch_max": float(mismatch_max[anchor_idx]),
                    "true_frot_mean_norm": float(true_norm_sum[anchor_idx] / used_steps),
                    "pred_frot_mean_norm": float(pred_norm_sum[anchor_idx] / used_steps),
                }
            )
    return rows


def collect_seed_dirs(run_roots: list[Path]) -> list[tuple[Path, Path]]:
    pairs: list[tuple[Path, Path]] = []
    for root in run_roots:
        for seed_dir in sorted(path for path in root.rglob("seed_*") if path.is_dir()):
            pairs.append((root, seed_dir))
    return pairs


def build_summary(df: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    per_seed = (
        df.groupby(["kp", "seed"], dropna=False)
        .agg(
            anchor_count=("frot_mismatch_rms", "size"),
            mean_frot_mismatch_rms=("frot_mismatch_rms", "mean"),
            p95_frot_mismatch_rms=("frot_mismatch_rms", q95),
            mean_frot_mismatch_max=("frot_mismatch_max", "mean"),
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
            anchor_count=("frot_mismatch_rms", "size"),
            mean_frot_mismatch_rms=("frot_mismatch_rms", "mean"),
            p95_frot_mismatch_rms=("frot_mismatch_rms", q95),
            mean_frot_mismatch_max=("frot_mismatch_max", "mean"),
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
            "Compute frozen-horizon f_rot mismatch: true future f_rot vs the "
            "constant-omega frozen f_rot implicitly injected to MPC."
        )
    )
    parser.add_argument(
        "run_roots",
        nargs="+",
        type=Path,
        help="One or more kp run roots to scan recursively.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Output directory for summary CSVs. Defaults to the first run root.",
    )
    parser.add_argument(
        "--seed-filter-csv",
        type=Path,
        default=None,
        help="Optional CSV with kp,seed rows to restrict which seeds are kept.",
    )
    parser.add_argument(
        "--anchor-path",
        type=Path,
        default=None,
        help="Optional explicit output path for the anchor-row CSV.",
    )
    parser.add_argument(
        "--per-seed-path",
        type=Path,
        default=None,
        help="Optional explicit output path for the per-seed summary CSV.",
    )
    parser.add_argument(
        "--summary-path",
        type=Path,
        default=None,
        help="Optional explicit output path for the kp summary CSV.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_roots = [path.resolve() for path in args.run_roots]
    out_dir = args.out_dir.resolve() if args.out_dir else run_roots[0]
    out_dir.mkdir(parents=True, exist_ok=True)
    seed_filter = load_seed_filter(args.seed_filter_csv.resolve() if args.seed_filter_csv else None)

    rows: list[dict] = []
    for run_root, seed_dir in collect_seed_dirs(run_roots):
        kp = infer_kp(seed_dir)
        seed = infer_seed(seed_dir)
        if seed_filter and (kp is None or seed is None or seed not in seed_filter.get(kp, set())):
            continue
        rows.extend(compute_seed_rows(run_root, seed_dir))

    if not rows:
        raise SystemExit("No frozen mismatch rows were computed. Check that noninertial frozen logs exist.")

    df = pd.DataFrame(rows)
    summary, per_seed = build_summary(df)

    anchor_path = (
        args.anchor_path.resolve()
        if args.anchor_path
        else out_dir / "frot_frozen_mismatch_anchor_rows.csv"
    )
    per_seed_path = (
        args.per_seed_path.resolve()
        if args.per_seed_path
        else out_dir / "frot_frozen_mismatch_by_seed.csv"
    )
    summary_path = (
        args.summary_path.resolve()
        if args.summary_path
        else out_dir / "frot_frozen_mismatch_summary_by_kp.csv"
    )
    anchor_path.parent.mkdir(parents=True, exist_ok=True)
    per_seed_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(anchor_path, index=False)
    per_seed.to_csv(per_seed_path, index=False)
    summary.to_csv(summary_path, index=False)

    print(f"[INFO] wrote {anchor_path}")
    print(f"[INFO] wrote {per_seed_path}")
    print(f"[INFO] wrote {summary_path}")
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
