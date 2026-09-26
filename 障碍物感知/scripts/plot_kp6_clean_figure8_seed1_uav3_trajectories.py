#!/usr/bin/env python3
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parent.parent
RUN_ROOT = ROOT / "results" / "figure_eight_clean_nocbf_kp6_seed1_5_2ctrl" / "logs"
SEED = 1
UAV_IDX = 3
SMOOTH_WINDOW = 15
V = 3.0
W = 3.0
TRAJ_LAPS = 2.0
START_PHASE = 0.5 * np.pi
REF_V = 3.0
REF_W = 3.0
K_SPEED = 0.9704032544031163
K_YAW = 3.171202014425998
UPSAMPLE = 6

OUT_WORLD = ROOT / "figures" / "kp6_clean_figure8_seed1_uav3_world_one_lap"
OUT_REL = ROOT / "figures" / "kp6_clean_figure8_seed1_uav3_relative_one_lap"

COLORS = {"theory": "#111111", "frozen": "#1f77b4", "stage": "#d62728"}


def figure8_amplitude(v_speed: float, w_yaw_rate: float) -> float:
    phase_rate = abs(w_yaw_rate) / K_YAW
    return abs(v_speed) / (phase_rate * K_SPEED)


def eval_phase_profile(
    t_query: float,
    tracking_duration: float,
    total_phase: float,
    phase_rate: float,
    ramp_duration: float,
):
    t = max(0.0, min(t_query, tracking_duration))

    def smooth_step(u: float) -> float:
        return u * u * (3.0 - 2.0 * u)

    def smooth_step_deriv(u: float) -> float:
        return 6.0 * u * (1.0 - u)

    if ramp_duration <= 1e-9 or tracking_duration <= 2.0 * ramp_duration:
        phase_offset = min(total_phase, phase_rate * t)
        phase_dot = phase_rate if t < tracking_duration else 0.0
        return phase_offset, phase_dot, 0.0

    decel_start = tracking_duration - ramp_duration
    if t < ramp_duration:
        u = t / ramp_duration
        integral = u * u * u - 0.5 * u * u * u * u
        phase_offset = phase_rate * ramp_duration * integral
        phase_dot = phase_rate * smooth_step(u)
        phase_ddot = phase_rate * smooth_step_deriv(u) / ramp_duration
    elif t < decel_start:
        phase_offset = phase_rate * (0.5 * ramp_duration + (t - ramp_duration))
        phase_dot = phase_rate
        phase_ddot = 0.0
    else:
        u = (t - decel_start) / ramp_duration
        integral = u - u * u * u + 0.5 * u * u * u * u
        phase_offset = phase_rate * (
            tracking_duration - 1.5 * ramp_duration + ramp_duration * integral
        )
        phase_dot = phase_rate * (1.0 - smooth_step(u))
        phase_ddot = -phase_rate * smooth_step_deriv(u) / ramp_duration

    phase_offset = max(0.0, min(total_phase, phase_offset))
    if t_query >= tracking_duration:
        phase_offset = total_phase
        phase_dot = 0.0
        phase_ddot = 0.0
    return phase_offset, phase_dot, phase_ddot


def one_lap_mask(times: np.ndarray) -> np.ndarray:
    amplitude = figure8_amplitude(REF_V, REF_W)
    cruise_phase_rate = min(abs(V) / (amplitude * K_SPEED), abs(W) / K_YAW)
    total_phase = max(1.0, TRAJ_LAPS) * 2.0 * np.pi
    cruise_phase_duration = total_phase / cruise_phase_rate
    ramp_duration = min(2.0, 0.25 * cruise_phase_duration)
    tracking_duration = cruise_phase_duration + ramp_duration
    one_lap_phase = 2.0 * np.pi
    keep = []
    for t in times:
        phase_offset, _, _ = eval_phase_profile(
            float(t), tracking_duration, total_phase, cruise_phase_rate, ramp_duration
        )
        keep.append(phase_offset <= one_lap_phase + 1e-9)
    return np.asarray(keep, dtype=bool)


def smooth_xy(df: pd.DataFrame, xcol: str, ycol: str, window: int) -> pd.DataFrame:
    out = df.copy()
    out[xcol] = out[xcol].rolling(window=window, center=True, min_periods=1).mean()
    out[ycol] = out[ycol].rolling(window=window, center=True, min_periods=1).mean()
    return out


def densify_xy(df: pd.DataFrame, xcol: str, ycol: str, factor: int) -> pd.DataFrame:
    if factor <= 1 or len(df) < 2:
        return df.copy()
    out = df.copy()
    t = np.arange(len(out), dtype=float)
    tf = np.linspace(t[0], t[-1], len(out) * factor)
    out2 = pd.DataFrame()
    if "sim_time" in out.columns:
        out2["sim_time"] = np.interp(tf, t, out["sim_time"].to_numpy(float))
    out2[xcol] = np.interp(tf, t, out[xcol].to_numpy(float))
    out2[ycol] = np.interp(tf, t, out[ycol].to_numpy(float))
    return out2


def load_ctrl(ctrl: str) -> pd.DataFrame:
    p = RUN_ROOT / ctrl / "v_3" / "w_3" / f"seed_{SEED:03d}" / "metrics_steps.csv"
    df = pd.read_csv(p)
    return df[df["uav_idx"] == UAV_IDX].copy().sort_values("step_idx").reset_index(drop=True)


def load_ugv(ctrl: str) -> pd.DataFrame:
    p = RUN_ROOT / ctrl / "v_3" / "w_3" / f"seed_{SEED:03d}" / "metrics_ugv_horizon_steps.csv"
    df = pd.read_csv(
        p,
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
    return df[df["horizon_idx"] == 0].copy().sort_values("step_idx").reset_index(drop=True)


def to_world(ugv: pd.DataFrame, rel_df: pd.DataFrame, xcol: str, ycol: str) -> pd.DataFrame:
    out = rel_df.copy().reset_index(drop=True)
    ugv = ugv.iloc[: len(out)].reset_index(drop=True)
    vx = ugv["actual_velocity_x"].to_numpy(float)
    vy = ugv["actual_velocity_y"].to_numpy(float)
    yaw = np.arctan2(vy, vx)
    bad = (np.abs(vx) + np.abs(vy)) < 1e-9
    if bad.any():
        for i in range(1, len(yaw)):
            if bad[i]:
                yaw[i] = yaw[i - 1]
        for i in range(len(yaw) - 2, -1, -1):
            if bad[i]:
                yaw[i] = yaw[i + 1]
    c = np.cos(yaw)
    s = np.sin(yaw)
    xr = out[xcol].to_numpy(float)
    yr = out[ycol].to_numpy(float)
    out["world_x"] = ugv["actual_position_x"].to_numpy(float) + xr * c - yr * s
    out["world_y"] = ugv["actual_position_y"].to_numpy(float) + xr * s + yr * c
    return out


def save_plain_plot(df_theory, df_frozen, df_stage, xcol, ycol, out_prefix):
    fig, ax = plt.subplots(figsize=(6.2, 4.9), constrained_layout=True)
    ax.plot(
        df_theory[xcol],
        df_theory[ycol],
        color=COLORS["theory"],
        lw=2.2,
        solid_capstyle="round",
        solid_joinstyle="round",
    )
    ax.plot(
        df_frozen[xcol],
        df_frozen[ycol],
        color=COLORS["frozen"],
        lw=2.0,
        solid_capstyle="round",
        solid_joinstyle="round",
        antialiased=True,
    )
    ax.plot(
        df_stage[xcol],
        df_stage[ycol],
        color=COLORS["stage"],
        lw=2.0,
        solid_capstyle="round",
        solid_joinstyle="round",
        antialiased=True,
    )
    ax.set_aspect("equal")
    ax.axis("off")
    fig.savefig(out_prefix.with_suffix(".png"), dpi=300, bbox_inches="tight")
    fig.savefig(out_prefix.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)


def main():
    frozen = load_ctrl("noninertial_frozen")
    stage = load_ctrl("noninertial_stage")
    ugv = load_ugv("noninertial_stage")

    n = min(len(frozen), len(stage), len(ugv))
    frozen = frozen.iloc[:n].reset_index(drop=True)
    stage = stage.iloc[:n].reset_index(drop=True)
    ugv = ugv.iloc[:n].reset_index(drop=True)
    keep = one_lap_mask(frozen["sim_time"].to_numpy(float))
    frozen = frozen.loc[keep].reset_index(drop=True)
    stage = stage.loc[keep].reset_index(drop=True)
    ugv = ugv.loc[keep].reset_index(drop=True)

    theory_rel = pd.DataFrame(
        {
            "step_idx": frozen["step_idx"],
            "sim_time": frozen["sim_time"],
            "x": frozen["rot_load_position_non_x"].to_numpy(float),
            "y": frozen["rot_load_position_non_y"].to_numpy(float),
        }
    )
    frozen_rel = smooth_xy(
        pd.DataFrame({"step_idx": frozen["step_idx"], "sim_time": frozen["sim_time"], "x": frozen["position_x"], "y": frozen["position_y"]}),
        "x",
        "y",
        SMOOTH_WINDOW,
    )
    stage_rel = smooth_xy(
        pd.DataFrame({"step_idx": stage["step_idx"], "sim_time": stage["sim_time"], "x": stage["position_x"], "y": stage["position_y"]}),
        "x",
        "y",
        SMOOTH_WINDOW,
    )

    theory_world = to_world(ugv, theory_rel.rename(columns={"x": "rel_x", "y": "rel_y"}), "rel_x", "rel_y")
    frozen_world = smooth_xy(to_world(ugv, frozen, "position_x", "position_y"), "world_x", "world_y", SMOOTH_WINDOW)
    stage_world = smooth_xy(to_world(ugv, stage, "position_x", "position_y"), "world_x", "world_y", SMOOTH_WINDOW)
    theory_world = densify_xy(theory_world, "world_x", "world_y", UPSAMPLE)
    frozen_world = densify_xy(frozen_world, "world_x", "world_y", UPSAMPLE)
    stage_world = densify_xy(stage_world, "world_x", "world_y", UPSAMPLE)
    theory_rel = densify_xy(theory_rel, "x", "y", UPSAMPLE)
    frozen_rel = densify_xy(frozen_rel, "x", "y", UPSAMPLE)
    stage_rel = densify_xy(stage_rel, "x", "y", UPSAMPLE)

    OUT_WORLD.parent.mkdir(parents=True, exist_ok=True)
    save_plain_plot(theory_world, frozen_world, stage_world, "world_x", "world_y", OUT_WORLD)
    save_plain_plot(theory_rel, frozen_rel, stage_rel, "x", "y", OUT_REL)

    pd.concat(
        [
            theory_world.assign(series="theory_world")[["sim_time", "world_x", "world_y", "series"]],
            frozen_world.assign(series="frozen_world")[["sim_time", "world_x", "world_y", "series"]],
            stage_world.assign(series="stage_world")[["sim_time", "world_x", "world_y", "series"]],
            theory_rel.assign(series="theory_rel")[["sim_time", "x", "y", "series"]],
            frozen_rel.assign(series="frozen_rel")[["sim_time", "x", "y", "series"]],
            stage_rel.assign(series="stage_rel")[["sim_time", "x", "y", "series"]],
        ],
        ignore_index=True,
    ).to_csv(ROOT / "figures" / "kp6_clean_figure8_seed1_uav3_trajectories.csv", index=False)


if __name__ == "__main__":
    main()
