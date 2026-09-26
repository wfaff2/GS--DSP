#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np
import pandas as pd

STEP_SUFFIX = "_metrics_steps.csv"
REQUIRED_COLUMNS = (
    "run_tag",
    "uav_idx",
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


def infer_kp(path: Path) -> float | None:
    for part in path.parts:
        match = re.fullmatch(r"kp_(\d+)p(\d+)", part)
        if match:
            return float(f"{match.group(1)}.{match.group(2)}")
    return None


def infer_seed(path: Path, run_tag: str) -> int | None:
    match = re.search(r"seed[_-]?(\d+)", run_tag)
    if match:
        return int(match.group(1))
    for part in path.parts:
        match = re.fullmatch(r"seed_(\d+)", part)
        if match:
            return int(match.group(1))
    return None


def canonical_method(frame_mode: str, rollout_mode: str) -> str:
    if frame_mode == "inertial":
        return "inertial_frozen"
    if frame_mode == "noninertial" and rollout_mode == "stage":
        return "noninertial_stage"
    if frame_mode == "noninertial" and rollout_mode == "frozen":
        return "noninertial_frozen"
    return f"{frame_mode}_{rollout_mode}".strip("_")


def discover_step_files(run_roots: list[Path]) -> list[Path]:
    files: list[Path] = []
    for root in run_roots:
        files.extend(
            path for path in sorted(root.rglob(f"*{STEP_SUFFIX}"))
            if not path.name.endswith("_ugv_horizon_steps.csv")
        )
    return files


def compute_vectors(df: pd.DataFrame) -> pd.DataFrame:
    omega = df[
        ["rot_load_omega_non_x", "rot_load_omega_non_y", "rot_load_omega_non_z"]
    ].to_numpy(dtype=float)
    position = df[
        ["rot_load_position_non_x", "rot_load_position_non_y", "rot_load_position_non_z"]
    ].to_numpy(dtype=float)
    velocity = df[
        ["rot_load_velocity_non_x", "rot_load_velocity_non_y", "rot_load_velocity_non_z"]
    ].to_numpy(dtype=float)

    coriolis = 2.0 * np.cross(omega, velocity)
    centrifugal = np.cross(omega, np.cross(omega, position))
    c_rot = coriolis + centrifugal

    df = df.copy()
    df["coriolis_norm"] = np.linalg.norm(coriolis, axis=1)
    df["centrifugal_norm"] = np.linalg.norm(centrifugal, axis=1)
    df["c_rot_norm"] = np.linalg.norm(c_rot, axis=1)
    return df


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


def build_summary(df: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    per_seed = (
        df.groupby(["method", "kp", "seed"], dropna=False)
        .agg(
            step_samples=("c_rot_norm", "size"),
            uav_count=("uav_idx", "nunique"),
            mean_c_rot_norm=("c_rot_norm", "mean"),
            p95_c_rot_norm=("c_rot_norm", q95),
            mean_coriolis_norm=("coriolis_norm", "mean"),
            p95_coriolis_norm=("coriolis_norm", q95),
            mean_centrifugal_norm=("centrifugal_norm", "mean"),
            p95_centrifugal_norm=("centrifugal_norm", q95),
        )
        .reset_index()
        .sort_values(["method", "kp", "seed"])
    )

    summary = (
        df.groupby(["method", "kp"], dropna=False)
        .agg(
            seed_count=("seed", "nunique"),
            step_samples=("c_rot_norm", "size"),
            uav_count=("uav_idx", "nunique"),
            mean_c_rot_norm=("c_rot_norm", "mean"),
            p95_c_rot_norm=("c_rot_norm", q95),
            mean_coriolis_norm=("coriolis_norm", "mean"),
            p95_coriolis_norm=("coriolis_norm", q95),
            mean_centrifugal_norm=("centrifugal_norm", "mean"),
            p95_centrifugal_norm=("centrifugal_norm", q95),
        )
        .reset_index()
        .sort_values(["method", "kp"])
    )
    return summary, per_seed


def load_data(step_files: list[Path]) -> pd.DataFrame:
    frames: list[pd.DataFrame] = []
    skipped: list[str] = []
    for path in step_files:
        df = pd.read_csv(path)
        missing = [col for col in REQUIRED_COLUMNS if col not in df.columns]
        if missing:
            skipped.append(f"{path}: missing {missing}")
            continue
        df = df.copy()
        df["source_file"] = str(path)
        df["kp"] = infer_kp(path)
        df["seed"] = df["run_tag"].map(lambda tag: infer_seed(path, str(tag)))
        df["method"] = [
            canonical_method(str(frame), str(rollout))
            for frame, rollout in zip(
                df["frame_mode_effective"], df["ugv_rollout_mode"]
            )
        ]
        frames.append(df)

    if not frames:
        joined = "\n".join(skipped[:20])
        raise RuntimeError(
            "No step CSV with objective rot-load columns was found.\n"
            "Expected columns: "
            f"{list(REQUIRED_COLUMNS)}\n"
            "This usually means the closed-loop logger has not been rebuilt/rerun yet.\n"
            f"Sample skipped entries:\n{joined}"
        )
    if skipped:
        print("[WARN] skipped files without objective rot-load columns:")
        for line in skipped[:20]:
            print(f"  - {line}")
        if len(skipped) > 20:
            print(f"  ... {len(skipped) - 20} more")

    return pd.concat(frames, ignore_index=True, sort=False)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compute objective closed-loop c_rot statistics from step CSV logs."
    )
    parser.add_argument(
        "run_roots",
        nargs="+",
        type=Path,
        help="One or more run root directories to scan recursively.",
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
        "--summary-path",
        type=Path,
        default=None,
        help="Optional explicit output path for the method-by-kp summary CSV.",
    )
    parser.add_argument(
        "--per-seed-path",
        type=Path,
        default=None,
        help="Optional explicit output path for the per-seed summary CSV.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_roots = [path.resolve() for path in args.run_roots]
    step_files = discover_step_files(run_roots)
    if not step_files:
        raise SystemExit(f"No {STEP_SUFFIX} files found under: {run_roots}")

    out_dir = args.out_dir.resolve() if args.out_dir else run_roots[0]
    out_dir.mkdir(parents=True, exist_ok=True)
    seed_filter = load_seed_filter(args.seed_filter_csv.resolve() if args.seed_filter_csv else None)

    df = load_data(step_files)
    if seed_filter:
        keep_mask = [
            (kp in seed_filter and seed in seed_filter[kp])
            for kp, seed in zip(df["kp"], df["seed"])
        ]
        df = df[pd.Series(keep_mask, index=df.index)].copy()
        if df.empty:
            raise SystemExit("No c_rot rows remain after applying the seed filter.")
    df = compute_vectors(df)
    summary, per_seed = build_summary(df)

    summary_path = (
        args.summary_path.resolve()
        if args.summary_path
        else out_dir / "crot_summary_by_method_kp.csv"
    )
    per_seed_path = (
        args.per_seed_path.resolve()
        if args.per_seed_path
        else out_dir / "crot_summary_by_seed.csv"
    )
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    per_seed_path.parent.mkdir(parents=True, exist_ok=True)
    summary.to_csv(summary_path, index=False)
    per_seed.to_csv(per_seed_path, index=False)

    print(f"[INFO] wrote {summary_path}")
    print(f"[INFO] wrote {per_seed_path}")
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
