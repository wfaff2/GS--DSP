#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
from pathlib import Path

import pandas as pd

SUMMARY_SUFFIX = "_metrics_ugv_horizon_summary.csv"
REQUIRED_COLUMNS = (
    "run_tag",
    "sim_time",
    "frame_mode_effective",
    "ugv_rollout_mode",
    "horizon_rms_xy",
    "horizon_max_xy",
    "horizon_final_xy",
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


def discover_summary_files(run_roots: list[Path]) -> list[Path]:
    files: list[Path] = []
    for root in run_roots:
        files.extend(sorted(root.rglob(f"*{SUMMARY_SUFFIX}")))
    return files


def q95(series: pd.Series) -> float:
    return float(series.quantile(0.95))


def load_data(summary_files: list[Path]) -> pd.DataFrame:
    frames: list[pd.DataFrame] = []
    skipped: list[str] = []
    for path in summary_files:
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
            "No valid UGV horizon summary CSV was found.\n"
            f"Expected columns: {list(REQUIRED_COLUMNS)}\n"
            f"Sample skipped entries:\n{joined}"
        )
    if skipped:
        print("[WARN] skipped malformed horizon summary files:")
        for line in skipped[:20]:
            print(f"  - {line}")
        if len(skipped) > 20:
            print(f"  ... {len(skipped) - 20} more")

    return pd.concat(frames, ignore_index=True, sort=False)


def build_summary(df: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    numeric_cols = ["horizon_rms_xy", "horizon_max_xy", "horizon_final_xy"]
    for col in numeric_cols:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    per_seed = (
        df.groupby(["method", "kp", "seed"], dropna=False)
        .agg(
            step_samples=("horizon_rms_xy", "size"),
            mean_horizon_rms_xy=("horizon_rms_xy", "mean"),
            p95_horizon_rms_xy=("horizon_rms_xy", q95),
            mean_horizon_max_xy=("horizon_max_xy", "mean"),
            p95_horizon_max_xy=("horizon_max_xy", q95),
            mean_horizon_final_xy=("horizon_final_xy", "mean"),
            p95_horizon_final_xy=("horizon_final_xy", q95),
        )
        .reset_index()
        .sort_values(["method", "kp", "seed"])
    )

    summary = (
        df.groupby(["method", "kp"], dropna=False)
        .agg(
            seed_count=("seed", "nunique"),
            step_samples=("horizon_rms_xy", "size"),
            mean_horizon_rms_xy=("horizon_rms_xy", "mean"),
            p95_horizon_rms_xy=("horizon_rms_xy", q95),
            mean_horizon_max_xy=("horizon_max_xy", "mean"),
            p95_horizon_max_xy=("horizon_max_xy", q95),
            mean_horizon_final_xy=("horizon_final_xy", "mean"),
            p95_horizon_final_xy=("horizon_final_xy", q95),
        )
        .reset_index()
        .sort_values(["method", "kp"])
    )
    return summary, per_seed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compute UGV horizon prediction-vs-actual error statistics."
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
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_roots = [path.resolve() for path in args.run_roots]
    summary_files = discover_summary_files(run_roots)
    if not summary_files:
        raise SystemExit(f"No {SUMMARY_SUFFIX} files found under: {run_roots}")

    out_dir = args.out_dir.resolve() if args.out_dir else run_roots[0]
    out_dir.mkdir(parents=True, exist_ok=True)

    df = load_data(summary_files)
    summary, per_seed = build_summary(df)

    summary_path = out_dir / "ugv_horizon_summary_by_method_kp.csv"
    per_seed_path = out_dir / "ugv_horizon_summary_by_seed.csv"
    summary.to_csv(summary_path, index=False)
    per_seed.to_csv(per_seed_path, index=False)

    print(f"[INFO] wrote {summary_path}")
    print(f"[INFO] wrote {per_seed_path}")
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
