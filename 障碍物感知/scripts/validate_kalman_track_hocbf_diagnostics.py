#!/usr/bin/env python3
"""Validate Kalman-track-HOCBF diagnostic CSV structure and identities."""

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, List, Tuple


def load_csv(path: Path) -> Tuple[List[str], List[Dict[str, str]]]:
    with path.open(newline="") as stream:
        raw_rows = list(csv.reader(stream))
    if not raw_rows:
        raise ValueError(f"empty CSV: {path}")
    header = raw_rows[0]
    bad_widths = sorted({len(row) for row in raw_rows[1:] if len(row) != len(header)})
    if bad_widths:
        raise ValueError(
            f"row width mismatch in {path}: header={len(header)}, bad={bad_widths}"
        )
    return header, [dict(zip(header, row)) for row in raw_rows[1:]]


def require_columns(header: List[str], columns: List[str], path: Path) -> None:
    missing = sorted(set(columns) - set(header))
    if missing:
        raise ValueError(f"missing columns in {path}: {missing}")


def number(row: Dict[str, str], column: str) -> float:
    return float(row[column])


def validate_track_rows(rows: List[Dict[str, str]], tolerance: float) -> Dict[str, float]:
    if not rows:
        raise ValueError("sensing-track CSV contains no active-track rows")
    online_rows = [row for row in rows if row["tracker_online"] == "1"]
    updated_rows = [row for row in online_rows if row["updated_this_cycle"] == "1"]
    created_rows = [row for row in updated_rows if row["created_this_cycle"] == "1"]
    corrected_rows = [row for row in updated_rows if row["innovation_valid"] == "1"]
    if not online_rows or not updated_rows or not created_rows or not corrected_rows:
        raise ValueError(
            "online diagnostics must include active, updated, created, and corrected rows"
        )

    max_center_residual = 0.0
    max_surface_residual = 0.0
    min_covariance_trace = math.inf
    max_covariance_trace = -math.inf
    for row in online_rows:
        truth_x = number(row, "truth_x")
        truth_y = number(row, "truth_y")
        track_x = number(row, "track_x")
        track_y = number(row, "track_y")
        expected_center_error = math.hypot(track_x - truth_x, track_y - truth_y)
        max_center_residual = max(
            max_center_residual,
            abs(expected_center_error - number(row, "track_center_error_m")),
        )
        expected_surface_error = (
            number(row, "track_surface_clearance_m")
            - number(row, "truth_surface_clearance_m")
        )
        max_surface_residual = max(
            max_surface_residual,
            abs(expected_surface_error - number(row, "track_surface_error_m")),
        )
        covariance_trace = number(row, "covariance_trace")
        if not math.isfinite(covariance_trace) or covariance_trace <= 0.0:
            raise ValueError(f"invalid covariance trace: {covariance_trace}")
        min_covariance_trace = min(min_covariance_trace, covariance_trace)
        max_covariance_trace = max(max_covariance_trace, covariance_trace)

        measurement_age = int(row["measurement_age_frames"])
        delivery_age = int(row["cycles_since_delivery"])
        imposed_delay = int(row["imposed_delay_frames"])
        if measurement_age - delivery_age != imposed_delay:
            raise ValueError(
                "timestamp identity failed: measurement_age - delivery_age "
                f"!= imposed_delay for obstacle {row['obstacle_id']}"
            )
        if row["reinitialized_this_cycle"] == "1" and int(row["track_instance"]) <= 1:
            raise ValueError("reinitialized track must have track_instance > 1")

    for row in updated_rows:
        if int(row["cycles_since_delivery"]) != 0:
            raise ValueError("a delivered update must have zero cycles_since_delivery")
        if int(row["measurement_age_frames"]) != int(row["imposed_delay_frames"]):
            raise ValueError("updated-row measurement age must equal imposed delay")

    if max_center_residual > tolerance or max_surface_residual > tolerance:
        raise ValueError(
            "track diagnostic formula residual exceeds tolerance: "
            f"center={max_center_residual}, surface={max_surface_residual}"
        )
    return {
        "track_rows": float(len(rows)),
        "updated_rows": float(len(updated_rows)),
        "created_rows": float(len(created_rows)),
        "corrected_rows": float(len(corrected_rows)),
        "max_center_residual": max_center_residual,
        "max_surface_residual": max_surface_residual,
        "min_covariance_trace": min_covariance_trace,
        "max_covariance_trace": max_covariance_trace,
    }


def validate_step_rows(rows: List[Dict[str, str]], tolerance: float) -> Dict[str, float]:
    if not rows:
        raise ValueError("step CSV contains no rows")
    selected = [row for row in rows if row["solver_active_static_track_valid"] == "1"]
    if not selected:
        raise ValueError("step CSV contains no solver-active static-obstacle rows")

    max_truth_residual = 0.0
    max_track_residual = 0.0
    max_alias_residual = 0.0
    for row in selected:
        solver_surface = number(row, "solver_planar_surface_distance")
        truth_surface = number(row, "solver_active_static_truth_surface_clearance_m")
        track_surface = number(row, "solver_active_static_track_surface_clearance_m")
        max_truth_residual = max(
            max_truth_residual,
            abs(
                solver_surface
                - truth_surface
                - number(row, "solver_surface_error_vs_truth_m")
            ),
        )
        max_track_residual = max(
            max_track_residual,
            abs(
                solver_surface
                - track_surface
                - number(row, "solver_surface_error_vs_track_m")
            ),
        )
        max_alias_residual = max(
            max_alias_residual,
            abs(number(row, "active_obstacle_x") - number(row, "solver_active_obstacle_x")),
            abs(number(row, "active_obstacle_y") - number(row, "solver_active_obstacle_y")),
        )
    if max(max_truth_residual, max_track_residual, max_alias_residual) > tolerance:
        raise ValueError(
            "HOCBF diagnostic formula residual exceeds tolerance: "
            f"truth={max_truth_residual}, track={max_track_residual}, "
            f"alias={max_alias_residual}"
        )
    return {
        "step_rows": float(len(rows)),
        "selected_static_rows": float(len(selected)),
        "max_solver_truth_residual": max_truth_residual,
        "max_solver_track_residual": max_track_residual,
        "max_legacy_alias_residual": max_alias_residual,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--step-csv", type=Path, required=True)
    parser.add_argument("--track-csv", type=Path, required=True)
    parser.add_argument("--tolerance", type=float, default=3e-6)
    args = parser.parse_args()

    step_header, step_rows = load_csv(args.step_csv)
    track_header, track_rows = load_csv(args.track_csv)
    require_columns(
        step_header,
        [
            "solver_active_static_track_valid",
            "solver_planar_surface_distance",
            "solver_active_static_truth_surface_clearance_m",
            "solver_active_static_track_surface_clearance_m",
            "solver_surface_error_vs_truth_m",
            "solver_surface_error_vs_track_m",
            "active_obstacle_x",
            "active_obstacle_y",
            "solver_active_obstacle_x",
            "solver_active_obstacle_y",
        ],
        args.step_csv,
    )
    require_columns(
        track_header,
        [
            "tracker_online",
            "truth_x",
            "truth_y",
            "track_x",
            "track_y",
            "track_center_error_m",
            "covariance_trace",
            "innovation_valid",
            "measurement_age_frames",
            "cycles_since_delivery",
            "imposed_delay_frames",
            "track_instance",
            "created_this_cycle",
            "updated_this_cycle",
            "reinitialized_this_cycle",
            "truth_surface_clearance_m",
            "track_surface_clearance_m",
            "track_surface_error_m",
        ],
        args.track_csv,
    )

    track_summary = validate_track_rows(track_rows, args.tolerance)
    step_summary = validate_step_rows(step_rows, args.tolerance)
    for key, value in {**track_summary, **step_summary}.items():
        if key.endswith("rows"):
            print(f"{key}={int(value)}")
        else:
            print(f"{key}={value:.9g}")
    print("kalman_track_hocbf_diagnostics=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
