#!/usr/bin/env python3
from pathlib import Path

import math
import matplotlib.pyplot as plt
import pandas as pd


ROOT = Path(__file__).resolve().parent.parent
RUN_ROOT = ROOT / "results" / "figure_eight_clean_nocbf_kp124_seed1_100_2ctrl" / "logs"
OUT_PREFIX = ROOT / "figures" / "kp4_clean_figure8_best_uav_world"

SEED = 68
UAV_IDX = 3
V = 2.0
W = 2.0
TRAJ_LAPS = 2.0
START_PHASE = 0.5 * math.pi
REF_V = 3.0
REF_W = 3.0
K_SPEED = 0.9704032544031163
K_YAW = 3.171202014425998
COLORS = {"frozen": "#d95f02", "stage": "#1b9e77", "desired": "#1f1f1f", "ugv": "#4c78a8"}


def figure8_amplitude(v_speed: float, w_yaw_rate: float) -> float:
    phase_rate = abs(w_yaw_rate) / K_YAW
    return abs(v_speed) / (phase_rate * K_SPEED)


def eval_phase_profile(t_query: float, tracking_duration: float, total_phase: float, phase_rate: float, ramp_duration: float):
    t = max(0.0, min(t_query, tracking_duration))
    phase_offset = 0.0
    phase_dot = 0.0
    phase_ddot = 0.0

    def smooth_step(u: float) -> float:
        return u * u * (3.0 - 2.0 * u)

    def smooth_step_deriv(u: float) -> float:
        return 6.0 * u * (1.0 - u)

    if ramp_duration <= 1e-9 or tracking_duration <= 2.0 * ramp_duration:
        phase_offset = min(total_phase, phase_rate * t)
        phase_dot = phase_rate if t < tracking_duration else 0.0
        return phase_offset, phase_dot, phase_ddot

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
    else:
        u = (t - decel_start) / ramp_duration
        integral = u - u * u * u + 0.5 * u * u * u * u
        phase_offset = phase_rate * (tracking_duration - 1.5 * ramp_duration + ramp_duration * integral)
        phase_dot = phase_rate * (1.0 - smooth_step(u))
        phase_ddot = -phase_rate * smooth_step_deriv(u) / ramp_duration

    phase_offset = max(0.0, min(total_phase, phase_offset))
    if t_query >= tracking_duration:
        phase_offset = total_phase
        phase_dot = 0.0
        phase_ddot = 0.0
    return phase_offset, phase_dot, phase_ddot


def eval_figure8_reference(times):
    amplitude = figure8_amplitude(REF_V, REF_W)
    cruise_phase_rate = min(abs(V) / (amplitude * K_SPEED), abs(W) / K_YAW)
    total_phase = max(1.0, TRAJ_LAPS) * 2.0 * math.pi
    cruise_phase_duration = total_phase / cruise_phase_rate
    ramp_duration = min(2.0, 0.25 * cruise_phase_duration)
    tracking_duration = cruise_phase_duration + ramp_duration

    xs, ys, yaws = [], [], []
    for t in times:
        phase_offset, phase_dot, phase_ddot = eval_phase_profile(
            float(t), tracking_duration, total_phase, cruise_phase_rate, ramp_duration
        )
        phase = math.fmod(START_PHASE + phase_offset, 2.0 * math.pi)
        if phase < 0.0:
            phase += 2.0 * math.pi
        sin_phase = math.sin(phase)
        cos_phase = math.cos(phase)
        sin_2phase = math.sin(2.0 * phase)
        cos_2phase = math.cos(2.0 * phase)
        x = amplitude * sin_phase
        y = 0.5 * amplitude * sin_2phase
        dpos_dphase_x = amplitude * cos_phase
        dpos_dphase_y = amplitude * cos_2phase
        d2pos_dphase2_x = -amplitude * sin_phase
        d2pos_dphase2_y = -2.0 * amplitude * sin_2phase
        vx = dpos_dphase_x * phase_dot
        vy = dpos_dphase_y * phase_dot
        ax = d2pos_dphase2_x * phase_dot * phase_dot + dpos_dphase_x * phase_ddot
        ay = d2pos_dphase2_y * phase_dot * phase_dot + dpos_dphase_y * phase_ddot
        speed_sq = vx * vx + vy * vy
        tangent_norm = math.hypot(dpos_dphase_x, dpos_dphase_y)
        if speed_sq > 1e-12:
            yaw = math.atan2(vy, vx)
        elif tangent_norm > 1e-12:
            yaw = math.atan2(dpos_dphase_y, dpos_dphase_x)
        else:
            yaw = 0.0
        xs.append(x)
        ys.append(y)
        yaws.append(yaw)
    return pd.DataFrame({"sim_time": times, "car_x": xs, "car_y": ys, "car_yaw": yaws})


def load_controller(ctrl: str) -> pd.DataFrame:
    p = RUN_ROOT / ctrl / "v_2" / "w_2" / f"seed_{SEED:03d}" / "metrics_steps.csv"
    df = pd.read_csv(p)
    return df[df["uav_idx"] == UAV_IDX].copy().reset_index(drop=True)


def to_world(car_df: pd.DataFrame, rel_df: pd.DataFrame, xcol: str, ycol: str) -> pd.DataFrame:
    out = rel_df.copy()
    car = car_df.iloc[: len(out)].reset_index(drop=True)
    xr = out[xcol].to_numpy(float)
    yr = out[ycol].to_numpy(float)
    yaw = car["car_yaw"].to_numpy(float)
    out["world_x"] = car["car_x"].to_numpy(float) + xr * pd.Series(yaw).apply(math.cos).to_numpy() - yr * pd.Series(yaw).apply(math.sin).to_numpy()
    out["world_y"] = car["car_y"].to_numpy(float) + xr * pd.Series(yaw).apply(math.sin).to_numpy() + yr * pd.Series(yaw).apply(math.cos).to_numpy()
    return out


def main() -> None:
    plt.rcParams.update(
        {
            "font.size": 10,
            "axes.titlesize": 12,
            "axes.labelsize": 11,
            "legend.fontsize": 9,
        }
    )

    frozen = load_controller("noninertial_frozen")
    stage = load_controller("noninertial_stage")
    times = frozen["sim_time"].to_numpy(float)
    car = eval_figure8_reference(times)

    desired = pd.DataFrame(
        {
            "sim_time": times,
            "position_x": frozen["rot_load_position_non_x"].to_numpy(float),
            "position_y": frozen["rot_load_position_non_y"].to_numpy(float),
        }
    )

    frozen_w = to_world(car, frozen, "position_x", "position_y")
    stage_w = to_world(car, stage, "position_x", "position_y")
    desired_w = to_world(car, desired, "position_x", "position_y")

    fig, ax = plt.subplots(figsize=(6.6, 4.8), constrained_layout=True)
    ax.plot(car["car_x"], car["car_y"], color=COLORS["ugv"], lw=1.5, ls="--", label="UGV reference path")
    ax.plot(desired_w["world_x"], desired_w["world_y"], color=COLORS["desired"], lw=2.0, ls="-.", label="Desired UAV path")
    ax.plot(frozen_w["world_x"], frozen_w["world_y"], color=COLORS["frozen"], lw=2.0, label="Frozen actual")
    ax.plot(stage_w["world_x"], stage_w["world_y"], color=COLORS["stage"], lw=2.0, label="Stage actual")

    for df, color in [(desired_w, COLORS["desired"]), (frozen_w, COLORS["frozen"]), (stage_w, COLORS["stage"])]:
        ax.scatter(df["world_x"].iloc[0], df["world_y"].iloc[0], s=18, color=color, zorder=5)

    ax.set_aspect("equal")
    ax.grid(True, alpha=0.22)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("KP=4 clean figure-eight: best single-UAV world-frame trajectory")
    ax.legend(frameon=False, loc="upper left")

    fr_rms = float((frozen["tracking_error"].pow(2).mean()) ** 0.5)
    st_rms = float((stage["tracking_error"].pow(2).mean()) ** 0.5)
    adv = (fr_rms - st_rms) / fr_rms * 100.0
    ax.text(
        0.02,
        0.02,
        f"seed={SEED}, UAV#{UAV_IDX}\\nFrozen RMS={fr_rms:.4f} m\\nStage RMS={st_rms:.4f} m\\nAdvantage={adv:.2f}%",
        transform=ax.transAxes,
        ha="left",
        va="bottom",
        fontsize=9,
        bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="0.75", alpha=0.95),
    )

    rows = []
    for label, df in [("ugv", car), ("desired", desired_w), ("frozen", frozen_w), ("stage", stage_w)]:
        cols = [c for c in ["sim_time", "car_x", "car_y", "world_x", "world_y"] if c in df.columns]
        tmp = df[cols].copy()
        tmp["series"] = label
        rows.append(tmp)

    OUT_PREFIX.parent.mkdir(parents=True, exist_ok=True)
    pd.concat(rows, ignore_index=True).to_csv(OUT_PREFIX.with_suffix(".csv"), index=False)
    fig.savefig(OUT_PREFIX.with_suffix(".png"), dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PREFIX.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
