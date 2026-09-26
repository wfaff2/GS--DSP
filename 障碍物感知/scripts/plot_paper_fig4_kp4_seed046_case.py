#!/usr/bin/env python3
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


ROOT = Path(__file__).resolve().parent.parent
OUT_PREFIX = ROOT / "figures" / "paper_fig4_kp4_seed046_case"
RUN_ROOT = ROOT / "results" / "figure_eight_fixedobs_cbf_kp124_seed1_100_2ctrl"
SEED = 46
WINDOW_HALF_WIDTH = 1.4
FAILURE_THRESHOLD = 0.215
COLORS = {
    "Frozen": "#d95f02",
    "Stage": "#1b9e77",
    "Obstacle": "#4c78a8",
    "ObstacleEdge": "#2f5b94",
}


def load_steps(controller: str) -> pd.DataFrame:
    return pd.read_csv(
        RUN_ROOT / "logs" / controller / "v_2" / "w_2" / f"seed_{SEED:03d}" / "metrics_steps.csv"
    )


def team_series(df: pd.DataFrame) -> pd.DataFrame:
    return (
        df.groupby("sim_time")
        .agg(
            min_surface=("solver_planar_surface_distance", "min"),
            slack_sum=("solver_slack", "sum"),
        )
        .reset_index()
    )


def main() -> None:
    plt.rcParams.update(
        {
            "font.size": 9,
            "axes.titlesize": 11,
            "axes.labelsize": 10,
            "legend.fontsize": 9,
        }
    )

    frozen = load_steps("noninertial_frozen")
    stage = load_steps("noninertial_stage")

    crit_stats = frozen.groupby("uav_idx")["solver_planar_surface_distance"].min().sort_values()
    critical_uav = int(crit_stats.index[0])

    f_team = team_series(frozen)
    s_team = team_series(stage)
    t_crit = float(f_team.loc[f_team["min_surface"].idxmin(), "sim_time"])
    t0 = t_crit - WINDOW_HALF_WIDTH
    t1 = t_crit + WINDOW_HALF_WIDTH

    f_u = frozen[(frozen["uav_idx"] == critical_uav) & (frozen["sim_time"].between(t0, t1))].copy()
    s_u = stage[(stage["uav_idx"] == critical_uav) & (stage["sim_time"].between(t0, t1))].copy()
    f_team_w = f_team[f_team["sim_time"].between(t0, t1)].copy()
    s_team_w = s_team[s_team["sim_time"].between(t0, t1)].copy()

    ref_xy = (
        frozen.loc[
            frozen["uav_idx"] == critical_uav,
            ["rot_load_position_non_x", "rot_load_position_non_y"],
        ]
        .iloc[0]
        .to_numpy(float)
    )

    active = f_u[f_u["active_obstacle_x"].notna() & f_u["active_obstacle_y"].notna()].copy()
    sampled = active.iloc[::6].copy() if not active.empty else active
    if not active.empty:
        closest = active["solver_planar_surface_distance"].idxmin()
        ox = float(active.loc[closest, "active_obstacle_x"])
        oy = float(active.loc[closest, "active_obstacle_y"])
        rr = float(active.loc[closest, "active_obstacle_radius"])
    else:
        ox = oy = rr = 0.0

    rows = []
    fig, axes = plt.subplots(1, 3, figsize=(10.6, 3.05), constrained_layout=True)

    ax = axes[0]
    if not active.empty:
        circ = plt.Circle((ox, oy), rr, facecolor="#c7e9c0", edgecolor="#238b45", alpha=0.75, lw=1.1)
        ax.add_patch(circ)
        ax.scatter(
            sampled["active_obstacle_x"],
            sampled["active_obstacle_y"],
            color=COLORS["Obstacle"],
            s=10,
            alpha=0.35,
            label="Obstacle samples",
        )
    ax.scatter([ref_xy[0]], [ref_xy[1]], color="black", s=28, marker="x", label="Desired position", zorder=5)
    ax.plot(f_u["position_x"], f_u["position_y"], color=COLORS["Frozen"], lw=2.0, label="Frozen")
    ax.plot(s_u["position_x"], s_u["position_y"], color=COLORS["Stage"], lw=2.0, label="Stage")
    ax.scatter(f_u["position_x"].iloc[0], f_u["position_y"].iloc[0], color=COLORS["Frozen"], s=18, zorder=6)
    ax.scatter(s_u["position_x"].iloc[0], s_u["position_y"].iloc[0], color=COLORS["Stage"], s=18, zorder=6)
    ax.axvline(ref_xy[0], color="black", ls=":", lw=0.9, alpha=0.45)
    ax.axhline(ref_xy[1], color="black", ls=":", lw=0.9, alpha=0.45)
    ax.set_title("(a) Critical interval in non-inertial frame", pad=5)
    ax.set_xlabel(r"$x_N$ [m]")
    ax.set_ylabel(r"$y_N$ [m]")
    ax.set_aspect("equal")
    ax.set_xlim(-0.95, 1.55)
    ax.set_ylim(-0.15, 1.95)
    ax.grid(True, alpha=0.20)
    ax.legend(frameon=False, loc="lower left", ncol=1, fontsize=8.4)

    ax = axes[1]
    ax.plot(f_team_w["sim_time"], f_team_w["min_surface"], color=COLORS["Frozen"], lw=2.0, label="Frozen")
    ax.plot(s_team_w["sim_time"], s_team_w["min_surface"], color=COLORS["Stage"], lw=2.0, label="Stage")
    ax.axhline(FAILURE_THRESHOLD, color="crimson", ls="--", lw=1.2, label="Violation threshold")
    ax.set_title("(b) Minimum surface distance", pad=5)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Distance [m]")
    ax.grid(True, alpha=0.20)
    ax.legend(frameon=False, loc="upper right")

    ax = axes[2]
    ax.plot(f_team_w["sim_time"], f_team_w["slack_sum"], color=COLORS["Frozen"], lw=2.0, label="Frozen")
    ax.plot(s_team_w["sim_time"], s_team_w["slack_sum"], color=COLORS["Stage"], lw=2.0, label="Stage")
    ax.set_title("(c) Slack sum over the same interval", pad=5)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Slack sum")
    ax.grid(True, alpha=0.20)
    ax.legend(frameon=False, loc="upper right")

    for t, d, s in zip(f_team_w["sim_time"], f_team_w["min_surface"], f_team_w["slack_sum"]):
        rows.append(
            {
                "panel": "critical_window_team_metrics",
                "controller": "noninertial_frozen",
                "time": float(t),
                "min_surface": float(d),
                "slack_sum": float(s),
            }
        )
    for t, d, s in zip(s_team_w["sim_time"], s_team_w["min_surface"], s_team_w["slack_sum"]):
        rows.append(
            {
                "panel": "critical_window_team_metrics",
                "controller": "noninertial_stage",
                "time": float(t),
                "min_surface": float(d),
                "slack_sum": float(s),
            }
        )
    for ctrl_name, ctrl_df in [("noninertial_frozen", f_u), ("noninertial_stage", s_u)]:
        for _, r in ctrl_df.iterrows():
            rows.append(
                {
                    "panel": "critical_window_path",
                    "controller": ctrl_name,
                    "time": float(r["sim_time"]),
                    "x": float(r["position_x"]),
                    "y": float(r["position_y"]),
                }
            )

    OUT_PREFIX.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(OUT_PREFIX.with_suffix(".csv"), index=False)
    fig.savefig(OUT_PREFIX.with_suffix(".png"), dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PREFIX.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
