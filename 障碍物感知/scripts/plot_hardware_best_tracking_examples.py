#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag


ROOT = Path(__file__).resolve().parent.parent
OUT_PREFIX = ROOT / "figures" / "hardware_figure8_best_tracking_examples"

CASE_GROUPS = [
    {
        "group_name": "Obstacle-Free Figure-Eight",
        "panel_prefix": "Obstacle-Free",
        "cases": [
            {
                "case_id": "2026-05-25-21-04-53",
                "csv_path": Path("/home/jjm/桌面/数据/3A1G8/2026-05-25-21-04-53/paper_log.csv"),
                "t_start": 1779714312.9094663,
                "t_stop": 1779714399.2892010,
                "lap1_duration": 42.640,
            },
            {
                "case_id": "2026-05-26-13-16-37",
                "csv_path": Path("/home/jjm/桌面/数据/3A1G8/2026-05-26-13-16-37/paper_log.csv"),
                "t_start": 1779772615.7059681,
                "t_stop": 1779772737.4244568,
                "lap1_duration": 42.563,
            },
        ],
    },
    {
        "group_name": "Fixed-Obstacle Figure-Eight",
        "panel_prefix": "With Obstacle",
        "cases": [
            {
                "case_id": "2026-05-26-13-47-50",
                "csv_path": Path("/home/jjm/桌面/数据/3A1G8cbf/2026-05-26-13-47-50/paper_log.csv"),
                "bag_path": Path("/home/jjm/桌面/数据/3A1G8cbf/2026-05-26-13-47-50/2026-05-26-13-47-50.bag"),
                "t_start": 1779774498.0588787,
                "t_stop": 1779774539.2828615,
                "lap1_duration": 41.224,
            },
            {
                "case_id": "2026-05-26-16-33-40",
                "csv_path": Path("/home/jjm/桌面/数据/3A1G8cbf/2026-05-26-16-33-40/paper_log.csv"),
                "bag_path": Path("/home/jjm/桌面/数据/3A1G8cbf/2026-05-26-16-33-40/2026-05-26-16-33-40.bag"),
                "t_start": 1779784431.8337872,
                "t_stop": 1779784502.7723467,
                "lap1_duration": 42.642,
            },
            {
                "case_id": "2026-05-26-20-42-08",
                "csv_path": Path("/home/jjm/桌面/数据/3A1G8cbf/2026-05-26-20-42-08/paper_log.csv"),
                "bag_path": Path("/home/jjm/桌面/数据/3A1G8cbf/2026-05-26-20-42-08/2026-05-26-20-42-08.bag"),
                "t_start": 1779799330.9874773,
                "t_stop": 1779799414.1899083,
                "lap1_duration": 42.676,
            },
        ],
    },
]

AXIS_COLORS = {
    "x": "#4c78a8",
    "y": "#f58518",
    "z": "#54a24b",
}
DISTANCE_COLOR = "#2cb7c9"
POSITION_YLIM = (-2.0, 2.0)
ERROR_YLIM = (-2.0, 2.0)
SIM_UAV_RADIUS = 0.15
SIM_OBSTACLE_RADIUS = 0.40
SAFE_MARGIN_DELTA = 0.10
SURFACE_DISTANCE_OFFSET = SIM_UAV_RADIUS + SIM_OBSTACLE_RADIUS - SAFE_MARGIN_DELTA
STATIC_OBSTACLE_TOPIC = "/coni_mpc/static_obstacles"
_OBSTACLE_CACHE: dict[str, np.ndarray] = {}


def load_window_df(case: dict) -> pd.DataFrame:
    df = pd.read_csv(case["csv_path"])
    df = df[(df["t"] >= case["t_start"]) & (df["t"] <= case["t_stop"])].copy()
    return df.sort_values(["t", "uav_id"]).reset_index(drop=True)


def add_tracking_columns(df: pd.DataFrame) -> pd.DataFrame:
    out = df.copy()
    out["err_x"] = pd.to_numeric(out["pN_x"], errors="coerce") - pd.to_numeric(
        out["prefN_x"], errors="coerce"
    )
    out["err_y"] = pd.to_numeric(out["pN_y"], errors="coerce") - pd.to_numeric(
        out["prefN_y"], errors="coerce"
    )
    out["err_z"] = pd.to_numeric(out["pN_z"], errors="coerce") - pd.to_numeric(
        out["prefN_z"], errors="coerce"
    )
    out["err_3d"] = np.sqrt(out["err_x"] ** 2 + out["err_y"] ** 2 + out["err_z"] ** 2)
    return out


def select_best_case(group_spec: dict) -> tuple[dict, pd.DataFrame, float]:
    best_case = None
    best_df = None
    best_mean = None
    for case in group_spec["cases"]:
        df = add_tracking_columns(load_window_df(case))
        team_mean = float(df["err_3d"].mean())
        if best_mean is None or team_mean < best_mean:
            best_case = case
            best_df = df
            best_mean = team_mean
    if best_case is None or best_df is None or best_mean is None:
        raise RuntimeError(f"No valid case found for {group_spec['group_name']}")
    return best_case, best_df, best_mean


def select_best_uav(case_df: pd.DataFrame) -> tuple[int, pd.DataFrame, float]:
    best_uav = None
    best_uav_df = None
    best_mean = None
    for uav_id, uav_df in case_df.groupby("uav_id"):
        uav_mean = float(uav_df["err_3d"].mean())
        if best_mean is None or uav_mean < best_mean:
            best_uav = int(uav_id)
            best_uav_df = uav_df.sort_values("t").reset_index(drop=True)
            best_mean = uav_mean
    if best_uav is None or best_uav_df is None or best_mean is None:
        raise RuntimeError("No valid UAV series found")
    return best_uav, best_uav_df, best_mean


def load_static_obstacle_centers(case: dict) -> np.ndarray:
    bag_path = case.get("bag_path")
    if bag_path is None:
        raise RuntimeError(f"No bag_path configured for case {case['case_id']}")
    cache_key = str(bag_path)
    if cache_key in _OBSTACLE_CACHE:
        return _OBSTACLE_CACHE[cache_key]

    with rosbag.Bag(str(bag_path)) as bag:
        for _, msg, _ in bag.read_messages(topics=[STATIC_OBSTACLE_TOPIC]):
            centers = np.array(
                [[marker.pose.position.x, marker.pose.position.y] for marker in msg.markers],
                dtype=float,
            )
            if centers.size == 0:
                raise RuntimeError(f"No static obstacles found in {bag_path}")
            _OBSTACLE_CACHE[cache_key] = centers
            return centers
    raise RuntimeError(f"Topic {STATIC_OBSTACLE_TOPIC} not found in {bag_path}")


def add_adjusted_surface_distance(case: dict, uav_df: pd.DataFrame) -> pd.DataFrame:
    out = uav_df.copy()
    if case.get("bag_path") is None:
        out["surface_distance_adjusted"] = np.nan
        return out

    centers = load_static_obstacle_centers(case)
    positions = out[["pW_x", "pW_y"]].to_numpy(dtype=float)
    center_distances = np.sqrt(((positions[:, None, :] - centers[None, :, :]) ** 2).sum(axis=2))
    out["surface_distance_adjusted"] = center_distances.min(axis=1) - SURFACE_DISTANCE_OFFSET
    return out


def panel_title(prefix: str, suffix: str) -> str:
    return f"{prefix}: {suffix}"


def export_plot_data(rows: list[pd.DataFrame]) -> None:
    out = pd.concat(rows, ignore_index=True)
    OUT_PREFIX.parent.mkdir(parents=True, exist_ok=True)
    out.to_csv(OUT_PREFIX.with_suffix(".csv"), index=False)


def auto_upper_limit(values: pd.Series, minimum: float = 1.0, step: float = 0.5) -> float:
    finite = pd.to_numeric(values, errors="coerce").replace([np.inf, -np.inf], np.nan).dropna()
    if finite.empty:
        return minimum
    upper = float(finite.max())
    upper = max(minimum, upper)
    return step * np.ceil(upper / step)


def ordered_left_panel_legend(ax: plt.Axes) -> None:
    handles, labels = ax.get_legend_handles_labels()
    label_to_handle = {label: handle for handle, label in zip(handles, labels)}
    order = ["p_x", "p_y", "p_z", "p_x_ref", "p_y_ref", "p_z_ref"]
    ordered_handles = [label_to_handle[label] for label in order if label in label_to_handle]
    ordered_labels = [label for label in order if label in label_to_handle]
    ax.legend(ordered_handles, ordered_labels, loc="lower left", ncol=2, framealpha=0.92)


def main() -> None:
    plt.rcParams.update(
        {
            "font.size": 10,
            "axes.titlesize": 13,
            "axes.labelsize": 11,
            "legend.fontsize": 9,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )

    fig, axes = plt.subplots(2, 2, figsize=(12.4, 7.4), constrained_layout=True)
    export_rows: list[pd.DataFrame] = []
    selection_rows = []

    for row_idx, group_spec in enumerate(CASE_GROUPS):
        case, case_df, team_mean = select_best_case(group_spec)
        best_uav, uav_df, uav_mean = select_best_uav(case_df)
        duration = min(float(case["t_stop"] - case["t_start"]), float(case["lap1_duration"]))

        plot_df = add_adjusted_surface_distance(case, uav_df)
        plot_df["time_s"] = plot_df["t"] - case["t_start"]
        plot_df = plot_df[plot_df["time_s"] <= duration].copy()
        plot_df["group_name"] = group_spec["group_name"]
        plot_df["case_id"] = case["case_id"]
        plot_df["selected_uav_id"] = best_uav
        plot_df["team_mean_err_3d"] = team_mean
        plot_df["uav_mean_err_3d"] = uav_mean
        plot_df["phase"] = np.where(plot_df["time_s"] <= case["lap1_duration"], "I", "II")
        export_rows.append(
            plot_df[
                [
                    "group_name",
                    "case_id",
                    "selected_uav_id",
                    "time_s",
                    "phase",
                    "pN_x",
                    "pN_y",
                    "pN_z",
                    "prefN_x",
                    "prefN_y",
                    "prefN_z",
                    "err_x",
                    "err_y",
                    "err_z",
                    "err_3d",
                    "dmin",
                    "surface_distance_adjusted",
                    "team_mean_err_3d",
                    "uav_mean_err_3d",
                ]
            ]
        )
        selection_rows.append(
            {
                "group_name": group_spec["group_name"],
                "selected_case_id": case["case_id"],
                "team_mean_err_3d": team_mean,
                "selected_uav_id": best_uav,
                "selected_uav_mean_err_3d": uav_mean,
                "duration_s": duration,
                "lap1_duration_s": float(case["lap1_duration"]),
            }
        )

        ax_left = axes[row_idx, 0]
        ax_right = axes[row_idx, 1]

        for axis_name in ("x", "y", "z"):
            color = AXIS_COLORS[axis_name]
            ax_left.plot(
                plot_df["time_s"],
                plot_df[f"pN_{axis_name}"],
                color=color,
                lw=2.0,
                label=f"p_{axis_name}",
            )
            ax_left.plot(
                plot_df["time_s"],
                plot_df[f"prefN_{axis_name}"],
                color=color,
                lw=1.5,
                ls="--",
                alpha=0.7,
                label=f"p_{axis_name}_ref",
            )
            if group_spec["panel_prefix"] != "With Obstacle":
                ax_right.plot(
                    plot_df["time_s"],
                    plot_df[f"err_{axis_name}"],
                    color=color,
                    lw=1.9,
                    label=f"p_{axis_name}_error",
                )

        ax_left.set_title(panel_title(group_spec["panel_prefix"], "Estimation & Reference of P"))
        ax_left.set_ylabel(r"$p_N$ [m]")
        ax_left.set_ylim(*POSITION_YLIM)
        ax_left.grid(True, alpha=0.22)
        ordered_left_panel_legend(ax_left)
        if group_spec["panel_prefix"] == "With Obstacle":
            distance_df = plot_df[["time_s", "surface_distance_adjusted"]].dropna().copy()
            ax_right.plot(
                distance_df["time_s"],
                distance_df["surface_distance_adjusted"],
                color=DISTANCE_COLOR,
                lw=2.2,
                label="Distance",
            )
            ax_right.set_title(panel_title(group_spec["panel_prefix"], "Distance to Obstacle"))
            ax_right.set_ylabel("Distance [m]")
            ax_right.set_ylim(0.0, auto_upper_limit(distance_df["surface_distance_adjusted"]))
            ax_right.grid(True, alpha=0.22)
            ax_right.legend(loc="lower left", framealpha=0.92)
        else:
            ax_right.set_title(panel_title(group_spec["panel_prefix"], "Tracking Error"))
            ax_right.axhline(0.0, color="0.35", lw=0.8, ls=":")
            ax_right.set_ylabel("Error [m]")
            ax_right.set_ylim(*ERROR_YLIM)
            ax_right.grid(True, alpha=0.22)
            ax_right.legend(loc="lower left", framealpha=0.92)

        for ax in (ax_left, ax_right):
            ax.set_xlim(0.0, duration)
            ax.set_xlabel("Time [s]")

    export_plot_data(export_rows)
    pd.DataFrame(selection_rows).to_csv(OUT_PREFIX.with_name(OUT_PREFIX.name + "_selection.csv"), index=False)
    OUT_PREFIX.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_PREFIX.with_suffix(".png"), dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PREFIX.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
