#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List

import genpy
import rosbag
import yaml
from geometry_msgs.msg import Twist


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_YAML = ROOT_DIR / "src" / "coni_mpc" / "parameters" / "num_sim_non_one_point.yaml"
DEFAULT_BAG_DIR = ROOT_DIR / "bags"

K_FIGURE_EIGHT_GERONO_MEAN_SPEED_FACTOR = 0.9704032544031163
K_FIGURE_EIGHT_GERONO_MAX_YAW_RATE_FACTOR = 3.171202014425998


@dataclass
class FigureEightGeometryConfig:
    fixed_geometry: bool = True
    reference_v: float = 3.0
    reference_w: float = 3.0
    amplitude: float = 0.0
    start_phase: float = 0.5 * math.pi


@dataclass
class FigureEightReference:
    x: float
    y: float
    yaw: float
    linear_speed: float
    angular_speed: float


@dataclass
class FigureEightTiming:
    amplitude: float
    lap_count: float
    phase_rate: float
    ramp_duration: float
    tracking_duration: float
    stop_hold_duration: float
    total_duration: float


@dataclass
class ProfileExtrema:
    max_linear_speed: float
    max_angular_speed: float
    sample_count: int


def load_yaml_config(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def load_geometry_config(path: Path) -> FigureEightGeometryConfig:
    raw = load_yaml_config(path).get("figure_eight_geometry", {})
    if not isinstance(raw, dict):
        raw = {}
    cfg = FigureEightGeometryConfig()
    cfg.fixed_geometry = bool(raw.get("fixed", cfg.fixed_geometry))
    cfg.reference_v = max(1e-6, abs(float(raw.get("reference_v", cfg.reference_v))))
    cfg.reference_w = max(1e-6, abs(float(raw.get("reference_w", cfg.reference_w))))
    cfg.amplitude = max(0.0, float(raw.get("amplitude", cfg.amplitude)))
    cfg.start_phase = float(raw.get("start_phase", cfg.start_phase))
    return cfg


def load_float_from_yaml(path: Path, key: str, default: float) -> float:
    raw = load_yaml_config(path).get(key, default)
    try:
        return float(raw)
    except (TypeError, ValueError):
        return float(default)


def figure_eight_amplitude_from_vw(v_speed: float, w_yaw_rate: float) -> float:
    spd = abs(v_speed)
    omg = abs(w_yaw_rate)
    if spd <= 1e-6 or omg <= 1e-6:
        return 0.0
    phase_rate = omg / K_FIGURE_EIGHT_GERONO_MAX_YAW_RATE_FACTOR
    return spd / (phase_rate * K_FIGURE_EIGHT_GERONO_MEAN_SPEED_FACTOR)


def resolve_figure_eight_amplitude(
    v_speed: float,
    w_yaw_rate: float,
    geometry_cfg: FigureEightGeometryConfig,
) -> float:
    if geometry_cfg.fixed_geometry:
        if geometry_cfg.amplitude > 1e-6:
            return geometry_cfg.amplitude
        return figure_eight_amplitude_from_vw(
            geometry_cfg.reference_v,
            geometry_cfg.reference_w,
        )
    return figure_eight_amplitude_from_vw(v_speed, w_yaw_rate)


def smooth_step(u: float) -> float:
    return u * u * (3.0 - 2.0 * u)


def smooth_step_derivative(u: float) -> float:
    return 6.0 * u * (1.0 - u)


def build_figure_eight_timing(
    desired_speed: float,
    yaw_rate_limit: float,
    trajectory_laps: float,
    duration_hint: float,
    geometry_cfg: FigureEightGeometryConfig,
) -> FigureEightTiming:
    speed = abs(desired_speed)
    omega_limit = abs(yaw_rate_limit)
    if speed <= 1e-6 or omega_limit <= 1e-6:
        raise ValueError("v and w must both be positive in magnitude.")

    amplitude = resolve_figure_eight_amplitude(speed, omega_limit, geometry_cfg)
    if amplitude <= 1e-6:
        raise ValueError("Resolved figure-eight amplitude is zero.")

    cruise_phase_rate = min(
        speed / (amplitude * K_FIGURE_EIGHT_GERONO_MEAN_SPEED_FACTOR),
        omega_limit / K_FIGURE_EIGHT_GERONO_MAX_YAW_RATE_FACTOR,
    )
    if cruise_phase_rate <= 1e-9:
        raise ValueError("Cruise phase rate is zero.")

    lap_count = max(1.0, trajectory_laps)
    total_phase = lap_count * 2.0 * math.pi
    cruise_phase_duration = total_phase / cruise_phase_rate
    ramp_duration = min(2.0, 0.25 * cruise_phase_duration)
    tracking_duration = cruise_phase_duration + ramp_duration
    phase_rate = cruise_phase_rate

    if duration_hint > 1e-6:
        tracking_duration = duration_hint
        ramp_duration = min(2.0, 0.25 * tracking_duration)
        if tracking_duration <= ramp_duration + 1e-6:
            ramp_duration = 0.25 * tracking_duration
        phase_rate = total_phase / max(1e-6, tracking_duration - ramp_duration)

    stop_hold_duration = 3.0
    total_duration = tracking_duration + stop_hold_duration
    return FigureEightTiming(
        amplitude=amplitude,
        lap_count=lap_count,
        phase_rate=phase_rate,
        ramp_duration=ramp_duration,
        tracking_duration=tracking_duration,
        stop_hold_duration=stop_hold_duration,
        total_duration=total_duration,
    )


def eval_phase_profile(t_query: float, timing: FigureEightTiming) -> tuple[float, float, float]:
    t = max(0.0, min(t_query, timing.tracking_duration))
    phase_offset = 0.0
    phase_dot = 0.0
    phase_ddot = 0.0
    total_phase = timing.lap_count * 2.0 * math.pi

    if timing.ramp_duration <= 1e-9 or timing.tracking_duration <= 2.0 * timing.ramp_duration:
        phase_offset = min(total_phase, timing.phase_rate * t)
        phase_dot = timing.phase_rate if t < timing.tracking_duration else 0.0
        return phase_offset, phase_dot, phase_ddot

    decel_start = timing.tracking_duration - timing.ramp_duration
    if t < timing.ramp_duration:
        u = t / timing.ramp_duration
        integral = u * u * u - 0.5 * u * u * u * u
        phase_offset = timing.phase_rate * timing.ramp_duration * integral
        phase_dot = timing.phase_rate * smooth_step(u)
        phase_ddot = timing.phase_rate * smooth_step_derivative(u) / timing.ramp_duration
    elif t < decel_start:
        phase_offset = timing.phase_rate * (
            0.5 * timing.ramp_duration + (t - timing.ramp_duration)
        )
        phase_dot = timing.phase_rate
        phase_ddot = 0.0
    else:
        u = (t - decel_start) / timing.ramp_duration
        integral = u - u * u * u + 0.5 * u * u * u * u
        phase_offset = timing.phase_rate * (
            timing.tracking_duration
            - 1.5 * timing.ramp_duration
            + timing.ramp_duration * integral
        )
        phase_dot = timing.phase_rate * (1.0 - smooth_step(u))
        phase_ddot = -timing.phase_rate * smooth_step_derivative(u) / timing.ramp_duration

    phase_offset = max(0.0, min(total_phase, phase_offset))
    if t_query >= timing.tracking_duration:
        phase_offset = total_phase
        phase_dot = 0.0
        phase_ddot = 0.0
    return phase_offset, phase_dot, phase_ddot


def eval_figure_eight_reference(
    t_query: float,
    timing: FigureEightTiming,
    geometry_cfg: FigureEightGeometryConfig,
) -> FigureEightReference:
    phase_offset, phase_dot, phase_ddot = eval_phase_profile(t_query, timing)
    phase = math.fmod(geometry_cfg.start_phase + phase_offset, 2.0 * math.pi)
    if phase < 0.0:
        phase += 2.0 * math.pi

    sin_phase = math.sin(phase)
    cos_phase = math.cos(phase)
    sin_2phase = math.sin(2.0 * phase)
    cos_2phase = math.cos(2.0 * phase)

    x = timing.amplitude * sin_phase
    y = 0.5 * timing.amplitude * sin_2phase
    dpos_dphase_x = timing.amplitude * cos_phase
    dpos_dphase_y = timing.amplitude * cos_2phase
    d2pos_dphase2_x = -timing.amplitude * sin_phase
    d2pos_dphase2_y = -2.0 * timing.amplitude * sin_2phase

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

    angular_speed = 0.0
    if speed_sq > 1e-9:
        angular_speed = (vx * ay - vy * ax) / speed_sq

    return FigureEightReference(
        x=x,
        y=y,
        yaw=yaw,
        linear_speed=math.sqrt(max(0.0, speed_sq)),
        angular_speed=angular_speed,
    )


def scan_profile_extrema(
    timing: FigureEightTiming,
    geometry_cfg: FigureEightGeometryConfig,
    sample_hz: float = 2000.0,
) -> ProfileExtrema:
    max_linear = 0.0
    max_angular = 0.0
    count = 0
    for t_query in iterate_sample_times(timing.total_duration, sample_hz):
        ref = eval_figure_eight_reference(t_query, timing, geometry_cfg)
        max_linear = max(max_linear, abs(ref.linear_speed))
        max_angular = max(max_angular, abs(ref.angular_speed))
        count += 1
    return ProfileExtrema(
        max_linear_speed=max_linear,
        max_angular_speed=max_angular,
        sample_count=count,
    )


def scale_timing_for_real_vehicle(
    timing: FigureEightTiming,
    geometry_cfg: FigureEightGeometryConfig,
    max_linear_x: float,
    max_angular_z: float,
) -> tuple[FigureEightTiming, ProfileExtrema, float]:
    extrema = scan_profile_extrema(timing, geometry_cfg)
    stretch = 1.0
    if max_linear_x > 1e-9:
        stretch = max(stretch, extrema.max_linear_speed / max_linear_x)
    if max_angular_z > 1e-9:
        stretch = max(stretch, extrema.max_angular_speed / max_angular_z)

    if stretch <= 1.0 + 1e-9:
        return timing, extrema, 1.0

    scaled = FigureEightTiming(
        amplitude=timing.amplitude,
        lap_count=timing.lap_count,
        phase_rate=timing.phase_rate / stretch,
        ramp_duration=timing.ramp_duration * stretch,
        tracking_duration=timing.tracking_duration * stretch,
        stop_hold_duration=timing.stop_hold_duration,
        total_duration=timing.tracking_duration * stretch + timing.stop_hold_duration,
    )
    return scaled, extrema, stretch


def iterate_sample_times(duration: float, rate_hz: float) -> Iterable[float]:
    sample_count = int(math.floor(duration * rate_hz + 1e-9)) + 1
    dt = 1.0 / rate_hz
    for idx in range(sample_count):
        yield idx * dt
    final_time = sample_count * dt
    if duration - (final_time - dt) > 1e-9:
        yield duration


def default_output_name(v_value: float, w_value: float, laps: float, rate_hz: float) -> str:
    def sanitize(value: float) -> str:
        return str(value).replace(".", "p")

    return (
        "figure8_cmdvel_"
        f"v{sanitize(v_value)}_"
        f"w{sanitize(w_value)}_"
        f"laps{sanitize(laps)}_"
        f"{sanitize(rate_hz)}hz.bag"
    )


def write_csv(csv_path: Path, rows: List[Dict[str, float]]) -> None:
    fieldnames = [
        "time_from_start",
        "linear_x",
        "angular_z",
        "world_x",
        "world_y",
        "world_yaw",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Export the repository's figure-eight UGV command profile to a ROS1 rosbag.",
    )
    parser.add_argument("--yaml", type=Path, default=DEFAULT_YAML, help="Path to num_sim_non_one_point.yaml")
    parser.add_argument("--out", type=Path, default=None, help="Output .bag path")
    parser.add_argument("--csv-out", type=Path, default=None, help="Optional CSV sidecar path")
    parser.add_argument("--topic", default="/cmd_vel", help="Output Twist topic")
    parser.add_argument("--v", type=float, default=1.5, help="Requested UGV linear speed limit [m/s]")
    parser.add_argument("--w", type=float, default=1.5, help="Requested UGV yaw-rate limit [rad/s]")
    parser.add_argument("--rate", type=float, default=100.0, help="Bag publish rate [Hz]")
    parser.add_argument("--trajectory-laps", type=float, default=None, help="Number of figure-eight laps")
    parser.add_argument("--sim-duration-sec", type=float, default=None, help="Optional override for tracking duration")
    parser.add_argument("--fixed-geometry", dest="fixed_geometry", action="store_true", help="Use YAML fixed figure-eight geometry")
    parser.add_argument("--dynamic-geometry", dest="fixed_geometry", action="store_false", help="Scale geometry directly from v/w")
    parser.set_defaults(fixed_geometry=None)
    parser.add_argument("--reference-v", type=float, default=None, help="Reference V used by fixed geometry")
    parser.add_argument("--reference-w", type=float, default=None, help="Reference W used by fixed geometry")
    parser.add_argument("--amplitude", type=float, default=None, help="Explicit Gerono amplitude [m]")
    parser.add_argument("--start-phase", type=float, default=None, help="Initial Gerono phase [rad]")
    parser.add_argument("--start-time-secs", type=float, default=None, help="Bag timestamp for t=0. Default: current wall time.")
    parser.add_argument("--limit-linear-x", type=float, default=None, help="Real-car absolute cap for Twist.linear.x")
    parser.add_argument("--limit-angular-z", type=float, default=None, help="Real-car absolute cap for Twist.angular.z")
    parser.add_argument("--fit-to-limits", action="store_true", help="Uniformly slow the profile until both limits are satisfied")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.rate <= 1e-9:
        raise SystemExit("--rate must be positive.")
    if abs(args.v) <= 1e-9 or abs(args.w) <= 1e-9:
        raise SystemExit("--v and --w must both be non-zero.")

    geometry_cfg = load_geometry_config(args.yaml)
    if args.fixed_geometry is not None:
        geometry_cfg.fixed_geometry = args.fixed_geometry
    if args.reference_v is not None:
        geometry_cfg.reference_v = max(1e-6, abs(args.reference_v))
    if args.reference_w is not None:
        geometry_cfg.reference_w = max(1e-6, abs(args.reference_w))
    if args.amplitude is not None:
        geometry_cfg.amplitude = max(0.0, args.amplitude)
    if args.start_phase is not None:
        geometry_cfg.start_phase = args.start_phase

    trajectory_laps = (
        args.trajectory_laps
        if args.trajectory_laps is not None
        else load_float_from_yaml(args.yaml, "trajectory_laps", 2.0)
    )
    sim_duration_sec = (
        args.sim_duration_sec
        if args.sim_duration_sec is not None
        else load_float_from_yaml(args.yaml, "sim_duration_sec", -1.0)
    )
    duration_hint = sim_duration_sec if sim_duration_sec > 1e-6 else -1.0

    timing = build_figure_eight_timing(
        desired_speed=args.v,
        yaw_rate_limit=args.w,
        trajectory_laps=trajectory_laps,
        duration_hint=duration_hint,
        geometry_cfg=geometry_cfg,
    )

    base_extrema = scan_profile_extrema(timing, geometry_cfg)
    stretch_factor = 1.0
    if args.fit_to_limits:
        if args.limit_linear_x is None and args.limit_angular_z is None:
            raise SystemExit("--fit-to-limits requires --limit-linear-x and/or --limit-angular-z.")
        timing, _, stretch_factor = scale_timing_for_real_vehicle(
            timing,
            geometry_cfg,
            max_linear_x=(float("inf") if args.limit_linear_x is None else abs(args.limit_linear_x)),
            max_angular_z=(float("inf") if args.limit_angular_z is None else abs(args.limit_angular_z)),
        )

    out_path = args.out
    if out_path is None:
        DEFAULT_BAG_DIR.mkdir(parents=True, exist_ok=True)
        out_path = DEFAULT_BAG_DIR / default_output_name(
            args.v,
            args.w,
            trajectory_laps,
            args.rate,
        )
    out_path = out_path.resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    csv_path = args.csv_out.resolve() if args.csv_out else out_path.with_suffix(".csv")
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    start_time_secs = args.start_time_secs if args.start_time_secs is not None else time.time()
    rows: List[Dict[str, float]] = []
    max_linear = 0.0
    max_angular = 0.0
    message_count = 0

    with rosbag.Bag(str(out_path), "w") as bag:
        for t_from_start in iterate_sample_times(timing.total_duration, args.rate):
            ref = eval_figure_eight_reference(t_from_start, timing, geometry_cfg)
            msg = Twist()
            msg.linear.x = ref.linear_speed
            msg.angular.z = ref.angular_speed

            stamp = genpy.Time.from_sec(start_time_secs + t_from_start)
            bag.write(args.topic, msg, t=stamp)
            rows.append(
                {
                    "time_from_start": t_from_start,
                    "linear_x": ref.linear_speed,
                    "angular_z": ref.angular_speed,
                    "world_x": ref.x,
                    "world_y": ref.y,
                    "world_yaw": ref.yaw,
                }
            )
            max_linear = max(max_linear, abs(ref.linear_speed))
            max_angular = max(max_angular, abs(ref.angular_speed))
            message_count += 1

    write_csv(csv_path, rows)

    print(f"bag: {out_path}")
    print(f"csv: {csv_path}")
    print(f"topic: {args.topic}")
    print(f"messages: {message_count}")
    print(f"base_max_linear_x_mps: {base_extrema.max_linear_speed:.6f}")
    print(f"base_max_abs_angular_z_radps: {base_extrema.max_angular_speed:.6f}")
    print(f"time_stretch_factor: {stretch_factor:.6f}")
    print(f"duration_sec: {timing.total_duration:.6f}")
    print(f"tracking_duration_sec: {timing.tracking_duration:.6f}")
    print(f"stop_hold_duration_sec: {timing.stop_hold_duration:.6f}")
    print(f"amplitude_m: {timing.amplitude:.6f}")
    print(f"max_linear_x_mps: {max_linear:.6f}")
    print(f"max_abs_angular_z_radps: {max_angular:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
