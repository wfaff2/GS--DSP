#!/usr/bin/env python3
"""Plot stage-vs-frozen UGV horizon prediction RMS error."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare UGV horizon prediction RMS error between "
            "`ugv_rollout_mode=stage` and `ugv_rollout_mode=frozen`."
        )
    )
    parser.add_argument("--stage-run", required=True)
    parser.add_argument("--frozen-run", required=True)
    parser.add_argument("--seed", required=True, type=int)
    parser.add_argument(
        "--frame-mode",
        choices=["inertial", "noninertial", "in", "non"],
        default="noninertial",
    )
    parser.add_argument("--title", default="")
    parser.add_argument("--out", default="")
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def resolve_frame_mode(frame_mode: str) -> str:
    if frame_mode == "in":
        return "inertial"
    if frame_mode == "non":
        return "noninertial"
    return frame_mode


def resolve_summary_csv(run_dir: Path, seed: int, frame_mode: str) -> Path:
    stem = (
        "inertial_metrics_ugv_horizon_summary.csv"
        if frame_mode == "inertial"
        else "noninertial_metrics_ugv_horizon_summary.csv"
    )
    csv_path = run_dir / f"seed_{seed}" / stem
    if not csv_path.is_file():
        raise FileNotFoundError(f"Missing UGV horizon summary CSV: {csv_path}")
    return csv_path


def default_output_path(stage_run: Path, seed: int, frame_mode: str) -> Path:
    compare_dir = stage_run.parent / "comparisons"
    compare_dir.mkdir(parents=True, exist_ok=True)
    return compare_dir / f"seed_{seed}_{frame_mode}_stage_vs_frozen_ugv_horizon_rms.png"


def _safe_float(text: str | None) -> float:
    if text is None:
        return math.nan
    stripped = str(text).strip()
    if not stripped:
        return math.nan
    try:
        return float(stripped)
    except ValueError:
        return math.nan


def load_series(csv_path: Path) -> tuple[list[float], list[float]]:
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    rows.sort(key=lambda row: _safe_float(row.get("sim_time")))
    times: list[float] = []
    rms: list[float] = []
    for row in rows:
        t = _safe_float(row.get("sim_time"))
        e = _safe_float(row.get("horizon_rms_xy"))
        if math.isfinite(t) and math.isfinite(e):
            times.append(t)
            rms.append(e)
    if not times:
        raise SystemExit(f"No finite sim_time/horizon_rms_xy rows in {csv_path}")
    return times, rms


def main() -> int:
    args = parse_args()
    frame_mode = resolve_frame_mode(args.frame_mode)

    stage_run = Path(args.stage_run).expanduser().resolve()
    frozen_run = Path(args.frozen_run).expanduser().resolve()
    if not stage_run.is_dir():
        raise NotADirectoryError(f"Stage run directory not found: {stage_run}")
    if not frozen_run.is_dir():
        raise NotADirectoryError(f"Frozen run directory not found: {frozen_run}")

    stage_csv = resolve_summary_csv(stage_run, args.seed, frame_mode)
    frozen_csv = resolve_summary_csv(frozen_run, args.seed, frame_mode)
    out_path = (
        Path(args.out).expanduser().resolve()
        if args.out
        else default_output_path(stage_run, args.seed, frame_mode)
    )

    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    stage_t, stage_e = load_series(stage_csv)
    frozen_t, frozen_e = load_series(frozen_csv)

    fig, ax = plt.subplots(figsize=(11.0, 4.8))
    ax.plot(stage_t, stage_e, linewidth=2.0, label="stage")
    ax.plot(frozen_t, frozen_e, linewidth=2.0, label="frozen")
    ax.set_xlabel("sim_time (s)")
    ax.set_ylabel("UGV horizon RMS xy error (m)")
    ax.set_title(
        args.title
        or f"Seed {args.seed} {frame_mode}: UGV predicted-vs-actual horizon RMS",
        pad=12,
    )
    ax.grid(True, alpha=0.3)
    ax.legend(
        loc="lower center",
        bbox_to_anchor=(0.5, 1.08),
        ncol=2,
        framealpha=0.92,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.975), h_pad=2.0)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180)
    print(f"saved compare plot: {out_path}")
    print(out_path)

    if args.show:
        plt.show()
    else:
        plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
