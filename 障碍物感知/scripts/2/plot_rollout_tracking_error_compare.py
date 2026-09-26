#!/usr/bin/env python3
"""Plot stage-vs-frozen tracking error from two batch result directories."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare tracking error between `ugv_rollout_mode=stage` and "
            "`ugv_rollout_mode=frozen` using two live_compare_rviz result dirs."
        )
    )
    parser.add_argument(
        "--stage-run",
        required=True,
        help="Path to the batch result directory generated with UGV_ROLLOUT_MODE=stage.",
    )
    parser.add_argument(
        "--frozen-run",
        required=True,
        help="Path to the batch result directory generated with UGV_ROLLOUT_MODE=frozen.",
    )
    parser.add_argument(
        "--seed",
        required=True,
        type=int,
        help="Seed index to compare, e.g. 3.",
    )
    parser.add_argument(
        "--frame-mode",
        choices=["inertial", "noninertial", "in", "non"],
        default="inertial",
        help="Which controller frame mode to compare.",
    )
    parser.add_argument(
        "--uav",
        default="all",
        help=(
            "UAV index to plot, `all`/`followers` for the mean across UAV1+, "
            "or `all_including_uav0` for the mean across all UAVs."
        ),
    )
    parser.add_argument(
        "--title",
        default="",
        help="Optional figure title. Defaults to an auto-generated title.",
    )
    parser.add_argument(
        "--out",
        default="",
        help="Optional output PNG path. Defaults to results/<stage_run>/comparisons/...",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Also open an interactive plot window after saving.",
    )
    return parser.parse_args()


def resolve_frame_mode(frame_mode: str) -> str:
    if frame_mode == "in":
        return "inertial"
    if frame_mode == "non":
        return "noninertial"
    return frame_mode


def resolve_step_csv(run_dir: Path, seed: int, frame_mode: str) -> Path:
    stem = "inertial_metrics_steps.csv" if frame_mode == "inertial" else "noninertial_metrics_steps.csv"
    csv_path = run_dir / f"seed_{seed}" / stem
    if not csv_path.is_file():
        raise FileNotFoundError(f"Missing step CSV: {csv_path}")
    return csv_path


def default_output_path(stage_run: Path, seed: int, frame_mode: str, uav: str) -> Path:
    compare_dir = stage_run / "comparisons"
    compare_dir.mkdir(parents=True, exist_ok=True)
    return compare_dir / f"seed_{seed}_{frame_mode}_stage_vs_frozen_tracking_error_uav_{uav}.png"


def main() -> int:
    args = parse_args()
    frame_mode = resolve_frame_mode(args.frame_mode)

    stage_run = Path(args.stage_run).expanduser().resolve()
    frozen_run = Path(args.frozen_run).expanduser().resolve()

    if not stage_run.is_dir():
        raise NotADirectoryError(f"Stage run directory not found: {stage_run}")
    if not frozen_run.is_dir():
        raise NotADirectoryError(f"Frozen run directory not found: {frozen_run}")

    stage_csv = resolve_step_csv(stage_run, args.seed, frame_mode)
    frozen_csv = resolve_step_csv(frozen_run, args.seed, frame_mode)

    title = args.title or f"Seed {args.seed} {frame_mode}: stage vs frozen tracking error"
    out_path = Path(args.out).expanduser().resolve() if args.out else default_output_path(stage_run, args.seed, frame_mode, args.uav)

    plot_script = Path(__file__).resolve().with_name("plot_step_obstacle_tracking_curves.py")
    cmd = [
        sys.executable,
        str(plot_script),
        "--csv",
        str(stage_csv),
        "--compare-csv",
        str(frozen_csv),
        "--uav",
        str(args.uav),
        "--label",
        "stage",
        "--compare-label",
        "frozen",
        "--figure-mode",
        "tracking_only",
        "--title",
        title,
        "--out",
        str(out_path),
    ]
    if args.show:
        cmd.append("--show")

    subprocess.run(cmd, check=True)
    print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
