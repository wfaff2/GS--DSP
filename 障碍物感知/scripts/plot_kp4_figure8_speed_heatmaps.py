#!/usr/bin/env python3
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.collections import LineCollection


ROOT = Path(__file__).resolve().parent.parent
RUN_ROOT = (
    ROOT
    / "results"
    / "figure_eight_clean_nocbf_kp124_seed1_100_2ctrl"
    / "logs"
    / "noninertial_stage"
    / "v_2"
    / "w_2"
    / "seed_068"
)
OUT_PREFIX = ROOT / "figures" / "kp4_figure8_speed_heatmaps"


def build_segments(x: np.ndarray, y: np.ndarray):
    pts = np.column_stack([x, y]).reshape(-1, 1, 2)
    return np.concatenate([pts[:-1], pts[1:]], axis=1)


def add_colored_path(ax, x, y, values, cmap, linewidth=3.0, vmin=None, vmax=None):
    segs = build_segments(x, y)
    val = 0.5 * (values[:-1] + values[1:])
    lc = LineCollection(segs, cmap=cmap, linewidth=linewidth)
    lc.set_array(val)
    if vmin is not None or vmax is not None:
        lc.set_clim(vmin=vmin, vmax=vmax)
    ax.add_collection(lc)
    ax.autoscale()
    ax.set_aspect("equal")
    ax.axis("off")
    return lc


def main():
    ugv = pd.read_csv(
        RUN_ROOT / "metrics_ugv_horizon_steps.csv",
        usecols=[
            "step_idx",
            "sim_time",
            "horizon_idx",
            "actual_position_x",
            "actual_position_y",
            "actual_velocity_x",
            "actual_velocity_y",
        ],
    )
    ugv = ugv[ugv["horizon_idx"] == 0].sort_values("step_idx").reset_index(drop=True)

    step = pd.read_csv(
        RUN_ROOT / "metrics_steps.csv",
        usecols=["uav_idx", "step_idx", "car_omega_world_z"],
    )
    step = step[step["uav_idx"] == 1].sort_values("step_idx").reset_index(drop=True)

    n = min(len(ugv), len(step))
    ugv = ugv.iloc[:n].copy()
    step = step.iloc[:n].copy()

    # One lap only: the logged run contains two laps.
    half = max(2, n // 2)
    ugv = ugv.iloc[:half].reset_index(drop=True)
    step = step.iloc[:half].reset_index(drop=True)

    x = ugv["actual_position_x"].to_numpy(float)
    y = ugv["actual_position_y"].to_numpy(float)
    v = np.hypot(
        ugv["actual_velocity_x"].to_numpy(float),
        ugv["actual_velocity_y"].to_numpy(float),
    )
    w = np.abs(step["car_omega_world_z"].to_numpy(float))

    fig, axes = plt.subplots(1, 2, figsize=(9.0, 4.2), constrained_layout=True)
    lc_v = add_colored_path(
        axes[0],
        x,
        y,
        v,
        cmap="viridis",
        linewidth=3.4,
        vmin=float(v.min()),
        vmax=float(v.max()),
    )
    lc_w = add_colored_path(
        axes[1],
        x,
        y,
        w,
        cmap="magma",
        linewidth=3.4,
        vmin=float(w.min()),
        vmax=float(w.max()),
    )

    cbar_v = fig.colorbar(lc_v, ax=axes[0], shrink=0.86, pad=0.02)
    cbar_v.set_label("Linear speed [m/s]")
    cbar_w = fig.colorbar(lc_w, ax=axes[1], shrink=0.86, pad=0.02)
    cbar_w.set_label("Angular speed [rad/s]")

    OUT_PREFIX.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_PREFIX.with_suffix(".png"), dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PREFIX.with_suffix(".pdf"), bbox_inches="tight")

    pd.DataFrame(
        {
            "x": x,
            "y": y,
            "linear_speed": v,
            "angular_speed": w,
        }
    ).to_csv(OUT_PREFIX.with_suffix(".csv"), index=False)
    plt.close(fig)


if __name__ == "__main__":
    main()
