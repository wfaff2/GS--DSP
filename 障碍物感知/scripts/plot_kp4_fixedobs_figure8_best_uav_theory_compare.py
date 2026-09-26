#!/usr/bin/env python3
from pathlib import Path

import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parent.parent
RUN_ROOT = ROOT / "results" / "figure_eight_fixedobs_cbf_kp124_seed1_100_2ctrl" / "logs"
OUT_PREFIX = ROOT / "figures" / "kp4_fixedobs_figure8_best_uav_theory_compare_one_lap"

SEED = 75
UAV_IDX = 1
V = 2.0
W = 2.0
TRAJ_LAPS = 2.0
START_PHASE = 0.5 * math.pi
REF_V = 3.0
REF_W = 3.0
K_SPEED = 0.9704032544031163
K_YAW = 3.171202014425998
SMOOTH_WINDOW = 15

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


def one_lap_mask(times):
    amplitude = figure8_amplitude(REF_V, REF_W)
    cruise_phase_rate = min(abs(V) / (amplitude * K_SPEED), abs(W) / K_YAW)
    total_phase = max(1.0, TRAJ_LAPS) * 2.0 * math.pi
    cruise_phase_duration = total_phase / cruise_phase_rate
    ramp_duration = min(2.0, 0.25 * cruise_phase_duration)
    tracking_duration = cruise_phase_duration + ramp_duration
    one_lap_phase = 2.0 * math.pi
    keep = []
    for t in times:
        phase_offset, _, _ = eval_phase_profile(
            float(t), tracking_duration, total_phase, cruise_phase_rate, ramp_duration
        )
        keep.append(phase_offset <= one_lap_phase + 1e-9)
    return np.asarray(keep, dtype=bool)


def load_controller(ctrl: str) -> pd.DataFrame:
    path = RUN_ROOT / ctrl / "v_2" / "w_2" / f"seed_{SEED:03d}" / "metrics_steps.csv"
    df = pd.read_csv(path)
    return df[df["uav_idx"] == UAV_IDX].copy().sort_values("step_idx").reset_index(drop=True)


def load_ugv(ctrl: str) -> pd.DataFrame:
    path = RUN_ROOT / ctrl / "v_2" / "w_2" / f"seed_{SEED:03d}" / "metrics_ugv_horizon_steps.csv"
    df = pd.read_csv(
        path,
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


def smooth_world_xy(df: pd.DataFrame, window: int) -> pd.DataFrame:
    out = df.copy()
    out["world_x"] = out["world_x"].rolling(window=window, center=True, min_periods=1).mean()
    out["world_y"] = out["world_y"].rolling(window=window, center=True, min_periods=1).mean()
    return out


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
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    xr = out[xcol].to_numpy(float)
    yr = out[ycol].to_numpy(float)
    out["world_x"] = ugv["actual_position_x"].to_numpy(float) + xr * cos_yaw - yr * sin_yaw
    out["world_y"] = ugv["actual_position_y"].to_numpy(float) + xr * sin_yaw + yr * cos_yaw
    return out


def main():
    frozen = load_controller("noninertial_frozen")
    stage = load_controller("noninertial_stage")
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
            "position_x": frozen["rot_load_position_non_x"].to_numpy(float),
            "position_y": frozen["rot_load_position_non_y"].to_numpy(float),
        }
    )

    theory_w = to_world(ugv, theory_rel, "position_x", "position_y")
    frozen_w = smooth_world_xy(to_world(ugv, frozen, "position_x", "position_y"), SMOOTH_WINDOW)
    stage_w = smooth_world_xy(to_world(ugv, stage, "position_x", "position_y"), SMOOTH_WINDOW)

    fig, ax = plt.subplots(figsize=(6.2, 4.9), constrained_layout=True)
    ax.plot(theory_w["world_x"], theory_w["world_y"], color=COLORS["theory"], lw=2.2, ls="-.")
    ax.plot(frozen_w["world_x"], frozen_w["world_y"], color=COLORS["frozen"], lw=2.0)
    ax.plot(stage_w["world_x"], stage_w["world_y"], color=COLORS["stage"], lw=2.0)
    ax.set_aspect("equal")
    ax.axis("off")

    rows = []
    for label, df in [("theory", theory_w), ("frozen", frozen_w), ("stage", stage_w)]:
        tmp = df[[c for c in ["step_idx", "sim_time", "world_x", "world_y"] if c in df.columns]].copy()
        tmp["series"] = label
        rows.append(tmp)

    OUT_PREFIX.parent.mkdir(parents=True, exist_ok=True)
    pd.concat(rows, ignore_index=True).to_csv(OUT_PREFIX.with_suffix(".csv"), index=False)
    fig.savefig(OUT_PREFIX.with_suffix(".png"), dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PREFIX.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
