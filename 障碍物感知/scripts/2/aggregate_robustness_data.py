#!/usr/bin/env python3
from __future__ import annotations

import pathlib
import sys

import numpy as np
import pandas as pd

STEP_FILENAMES = (
    "noninertial_metrics_steps.csv",
    "noninertial_frozen_metrics_steps.csv",
)
HORIZON_FILENAMES = (
    "noninertial_metrics_ugv_horizon_summary.csv",
    "noninertial_frozen_metrics_ugv_horizon_summary.csv",
)
REQUIRED_STEP_COLUMNS = (
    "tracking_error",
    "sim_time",
    "run_tag",
    "ugv_rollout_mode",
)


def collect_seed_files(run_root: pathlib.Path, filenames: tuple[str, ...]) -> list[pathlib.Path]:
    files: list[pathlib.Path] = []
    for seed_dir in sorted(run_root.glob("seed_*")):
        if not seed_dir.is_dir():
            continue
        for filename in filenames:
            path = seed_dir / filename
            if path.is_file():
                files.append(path)
    return files


def load_noninertial_steps(step_files: list[pathlib.Path]) -> pd.DataFrame:
    df_steps = pd.concat([pd.read_csv(path) for path in step_files], ignore_index=True, sort=False)
    missing_cols = [col for col in REQUIRED_STEP_COLUMNS if col not in df_steps.columns]
    if missing_cols:
        print(f"\n[ERROR] robustness step CSV 缺少列: {missing_cols}")
        print(">>> 请确认你读取的是 *_metrics_steps.csv，并且 C++ 修改已经重新编译生效。\n")
        sys.exit(1)

    accel_cols_world = ("car_a_world_x", "car_a_world_y", "car_a_world_z")
    accel_cols_non = ("a_car_non_x", "a_car_non_y", "a_car_non_z")
    if all(col in df_steps.columns for col in accel_cols_world):
        ax_col, ay_col, az_col = accel_cols_world
    elif all(col in df_steps.columns for col in accel_cols_non):
        ax_col, ay_col, az_col = accel_cols_non
    else:
        print(
            "\n[ERROR] robustness step CSV 缺少加速度列: "
            f"need {accel_cols_world} or {accel_cols_non}\n"
        )
        sys.exit(1)

    if "frame_mode_effective" in df_steps.columns:
        df_steps = df_steps[df_steps["frame_mode_effective"] == "noninertial"].copy()

    df_steps["car_a_norm"] = np.sqrt(
        df_steps[ax_col] ** 2
        + df_steps[ay_col] ** 2
        + df_steps[az_col] ** 2
    )
    omega_cols_world = (
        "car_omega_world_x",
        "car_omega_world_y",
        "car_omega_world_z",
    )
    omega_cols_non = ("omega_non_x", "omega_non_y", "omega_non_z")
    if all(col in df_steps.columns for col in omega_cols_world):
        wx_col, wy_col, wz_col = omega_cols_world
    elif all(col in df_steps.columns for col in omega_cols_non):
        wx_col, wy_col, wz_col = omega_cols_non
    else:
        print(
            "\n[ERROR] robustness step CSV 缺少角速度列: "
            f"need {omega_cols_world} or {omega_cols_non}\n"
        )
        sys.exit(1)

    df_steps["car_omega_norm"] = np.sqrt(
        df_steps[wx_col] ** 2
        + df_steps[wy_col] ** 2
        + df_steps[wz_col] ** 2
    )
    df_steps["seed"] = df_steps["run_tag"].str.extract(r"seed(\d+)").astype(float)
    df_steps["method"] = df_steps["ugv_rollout_mode"]
    if "solver_planar_surface_distance" in df_steps.columns:
        df_steps["obstacle_surface_distance"] = df_steps[
            "solver_planar_surface_distance"
        ]
    elif "solver_planar_clearance" in df_steps.columns:
        df_steps["obstacle_surface_distance"] = df_steps[
            "solver_planar_clearance"
        ]
    else:
        df_steps["obstacle_surface_distance"] = np.nan
    df_steps["sim_time"] = df_steps["sim_time"].round(3)

    grouped = (
        df_steps.groupby(["seed", "method", "sim_time"], as_index=False)
        .agg(
            tracking_error=("tracking_error", "max"),
            car_a_norm=("car_a_norm", "mean"),
            car_omega_norm=("car_omega_norm", "mean"),
            obstacle_surface_distance=("obstacle_surface_distance", "min"),
        )
    )
    ugv_profile = (
        grouped.groupby(["seed", "sim_time"], as_index=False)
        .agg(
            car_a_norm_ref=("car_a_norm", "max"),
            car_omega_norm_ref=("car_omega_norm", "max"),
        )
    )
    grouped = grouped.merge(ugv_profile, on=["seed", "sim_time"], how="left")
    grouped["car_a_norm"] = grouped["car_a_norm_ref"].fillna(grouped["car_a_norm"])
    grouped["car_omega_norm"] = grouped["car_omega_norm_ref"].fillna(grouped["car_omega_norm"])
    return grouped.drop(columns=["car_a_norm_ref", "car_omega_norm_ref"])

def load_noninertial_horizon(horizon_files: list[pathlib.Path]) -> pd.DataFrame:
    df_hor = pd.concat([pd.read_csv(path) for path in horizon_files], ignore_index=True, sort=False)
    required_cols = ("run_tag", "ugv_rollout_mode", "sim_time", "horizon_rms_xy")
    missing_cols = [col for col in required_cols if col not in df_hor.columns]
    if missing_cols:
        print(f"\n[ERROR] robustness horizon CSV 缺少列: {missing_cols}\n")
        sys.exit(1)

    if "frame_mode_effective" in df_hor.columns:
        df_hor = df_hor[df_hor["frame_mode_effective"] == "noninertial"].copy()

    df_hor["seed"] = df_hor["run_tag"].str.extract(r"seed(\d+)").astype(float)
    df_hor["method"] = df_hor["ugv_rollout_mode"]
    df_hor["sim_time"] = df_hor["sim_time"].round(3)

    return (
        df_hor.groupby(["seed", "method", "sim_time"], as_index=False)
        .agg(horizon_rms_xy=("horizon_rms_xy", "mean"))
    )


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python3 aggregate_robustness_data.py <run_root_dir>")
        sys.exit(1)

    run_root = pathlib.Path(sys.argv[1])
    out_csv = run_root / "combined_robustness_data.csv"

    step_files = collect_seed_files(run_root, STEP_FILENAMES)
    horizon_files = collect_seed_files(run_root, HORIZON_FILENAMES)

    if not step_files or not horizon_files:
        print(
            f"[WARN] Missing noninertial stage/frozen robustness source CSV files in {run_root}"
        )
        sys.exit(0)

    try:
        df_steps = load_noninertial_steps(step_files)
        df_hor = load_noninertial_horizon(horizon_files)
        df_merged = pd.merge(
            df_steps,
            df_hor,
            on=["seed", "method", "sim_time"],
            how="inner",
        ).sort_values(["seed", "method", "sim_time"])
        df_merged.insert(0, "frame_mode_effective", "noninertial")
        df_merged.to_csv(out_csv, index=False)
        print(f"\n[INFO] robustness data aggregated successfully: {out_csv}")
        print(
            "[INFO] robustness methods rows:",
            df_merged["method"].value_counts().to_dict(),
        )
    except Exception as exc:
        print(f"[ERROR] 聚合失败: {exc}")
        sys.exit(1)


if __name__ == "__main__":
    main()
