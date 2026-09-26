#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
import rosbag


ROOT = Path(__file__).resolve().parent.parent
OUT_CSV = ROOT / "results" / "hardware_lap_protocol_summary.csv"
STATIC_OBSTACLE_TOPIC = "/coni_mpc/static_obstacles"
UAV_RADIUS_M = 0.15
OBSTACLE_RADIUS_M = 0.40
OBSTACLE_SURFACE_CORRECTION_M = 0.10


@dataclass(frozen=True)
class HardwareCase:
    group: str
    case_id: str
    lap_count: int
    csv_path: Path
    t_start: float
    t_stop: float
    bag_path: Path | None = None


CASES = [
    HardwareCase(
        group="obstacle_free",
        case_id="2026-05-25-21-04-53",
        lap_count=2,
        csv_path=Path("/home/jjm/桌面/数据/3A1G8/2026-05-25-21-04-53/paper_log.csv"),
        t_start=1779714312.9094663,
        t_stop=1779714399.2892010,
    ),
    HardwareCase(
        group="obstacle_free",
        case_id="2026-05-26-13-16-37",
        lap_count=3,
        csv_path=Path("/home/jjm/桌面/数据/3A1G8/2026-05-26-13-16-37/paper_log.csv"),
        t_start=1779772615.7059681,
        t_stop=1779772737.4244568,
    ),
    HardwareCase(
        group="fixed_obstacle",
        case_id="2026-05-26-13-47-50",
        lap_count=1,
        csv_path=Path("/home/jjm/桌面/数据/3A1G8cbf/2026-05-26-13-47-50/paper_log.csv"),
        bag_path=Path("/home/jjm/桌面/数据/3A1G8cbf/2026-05-26-13-47-50/2026-05-26-13-47-50.bag"),
        t_start=1779774498.0588787,
        t_stop=1779774539.2828615,
    ),
    HardwareCase(
        group="fixed_obstacle",
        case_id="2026-05-26-16-33-40",
        lap_count=1,
        csv_path=Path("/home/jjm/桌面/数据/3A1G8cbf/2026-05-26-16-33-40/paper_log.csv"),
        bag_path=Path("/home/jjm/桌面/数据/3A1G8cbf/2026-05-26-16-33-40/2026-05-26-16-33-40.bag"),
        t_start=1779784431.8337872,
        t_stop=1779784502.7723467,
    ),
    HardwareCase(
        group="fixed_obstacle",
        case_id="2026-05-26-20-42-08",
        lap_count=2,
        csv_path=Path("/home/jjm/桌面/数据/3A1G8cbf/2026-05-26-20-42-08/paper_log.csv"),
        bag_path=Path("/home/jjm/桌面/数据/3A1G8cbf/2026-05-26-20-42-08/2026-05-26-20-42-08.bag"),
        t_start=1779799330.9874773,
        t_stop=1779799414.1899083,
    ),
]


def load_window(case: HardwareCase) -> pd.DataFrame:
    df = pd.read_csv(case.csv_path)
    df = df[(df["t"] >= case.t_start) & (df["t"] <= case.t_stop)].copy()
    numeric_cols = [
        "pN_x",
        "pN_y",
        "pN_z",
        "prefN_x",
        "prefN_y",
        "prefN_z",
        "pW_x",
        "pW_y",
        "pW_z",
        "solve_ms",
    ]
    for col in numeric_cols:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df.sort_values(["t", "uav_id"]).reset_index(drop=True)


def load_static_obstacle_centers(bag_path: Path) -> np.ndarray:
    with rosbag.Bag(str(bag_path)) as bag:
        for _, msg, _ in bag.read_messages(topics=[STATIC_OBSTACLE_TOPIC]):
            return np.array(
                [[marker.pose.position.x, marker.pose.position.y] for marker in msg.markers],
                dtype=float,
            )
    raise RuntimeError(f"Topic {STATIC_OBSTACLE_TOPIC} not found in {bag_path}")


def compute_team_rms(lap_df: pd.DataFrame) -> float:
    err_sq = (
        (lap_df["pN_x"] - lap_df["prefN_x"]) ** 2
        + (lap_df["pN_y"] - lap_df["prefN_y"]) ** 2
        + (lap_df["pN_z"] - lap_df["prefN_z"]) ** 2
    )
    return math.sqrt(float(err_sq.mean()))


def compute_max_uav_rms(lap_df: pd.DataFrame) -> float:
    return max(compute_team_rms(uav_df) for _, uav_df in lap_df.groupby("uav_id"))


def compute_min_inter_uav_surface(lap_df: pd.DataFrame) -> float:
    min_surface = math.inf
    for _, time_df in lap_df.groupby("t"):
        positions = time_df[["pW_x", "pW_y", "pW_z"]].dropna().to_numpy(dtype=float)
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                surface = np.linalg.norm(positions[i] - positions[j]) - 2.0 * UAV_RADIUS_M
                min_surface = min(min_surface, float(surface))
    return min_surface


def compute_lap_rows() -> pd.DataFrame:
    rows: list[dict[str, float | int | str]] = []
    obstacle_cache: dict[Path, np.ndarray] = {}

    for case in CASES:
        window_df = load_window(case)
        lap_duration = (case.t_stop - case.t_start) / case.lap_count
        centers = None
        if case.bag_path is not None:
            centers = obstacle_cache.setdefault(
                case.bag_path, load_static_obstacle_centers(case.bag_path)
            )

        for lap_idx in range(case.lap_count):
            lap_start = case.t_start + lap_idx * lap_duration
            lap_stop = case.t_start + (lap_idx + 1) * lap_duration
            if lap_idx == case.lap_count - 1:
                lap_stop = case.t_stop
            lap_df = window_df[(window_df["t"] >= lap_start) & (window_df["t"] <= lap_stop)].copy()

            row: dict[str, float | int | str] = {
                "group": case.group,
                "case_id": case.case_id,
                "lap_index": lap_idx + 1,
                "lap_start": lap_start,
                "lap_stop": lap_stop,
                "team_tracking_rms_m": compute_team_rms(lap_df),
                "max_uav_tracking_rms_m": compute_max_uav_rms(lap_df),
                "solve_mean_ms": float(lap_df["solve_ms"].mean()),
                "solve_p95_ms": float(lap_df["solve_ms"].quantile(0.95)),
                "solve_peak_ms": float(lap_df["solve_ms"].max()),
            }

            if centers is not None:
                positions = lap_df[["pW_x", "pW_y"]].to_numpy(dtype=float)
                center_distance = np.sqrt(
                    ((positions[:, None, :] - centers[None, :, :]) ** 2).sum(axis=2)
                )
                row["min_obstacle_surface_m"] = float(
                    np.nanmin(
                        center_distance
                        - (UAV_RADIUS_M + OBSTACLE_RADIUS_M)
                        + OBSTACLE_SURFACE_CORRECTION_M
                    )
                )
                row["min_inter_uav_surface_m"] = compute_min_inter_uav_surface(lap_df)

            rows.append(row)

    return pd.DataFrame(rows)


def print_group_summary(lap_df: pd.DataFrame) -> None:
    for group_name, group_df in lap_df.groupby("group"):
        print(f"\n[{group_name}]")
        for col in [
            "team_tracking_rms_m",
            "max_uav_tracking_rms_m",
            "solve_mean_ms",
            "solve_p95_ms",
            "solve_peak_ms",
        ]:
            series = group_df[col]
            print(
                f"{col}: mean={series.mean():.6f}, std={series.std(ddof=1):.6f}, "
                f"min={series.min():.6f}, max={series.max():.6f}"
            )
        if group_name == "fixed_obstacle":
            for col in ["min_obstacle_surface_m", "min_inter_uav_surface_m"]:
                series = group_df[col]
                print(
                    f"{col}: mean={series.mean():.6f}, std={series.std(ddof=1):.6f}, "
                    f"min={series.min():.6f}, max={series.max():.6f}"
                )


def main() -> None:
    lap_df = compute_lap_rows()
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    lap_df.to_csv(OUT_CSV, index=False)
    print(f"Wrote {OUT_CSV}")
    print(lap_df.round(6).to_string(index=False))
    print_group_summary(lap_df)


if __name__ == "__main__":
    main()
