#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import pathlib
from dataclasses import dataclass

import matplotlib

if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.lines import Line2D


@dataclass(frozen=True)
class SimulationConfig:
    dt: float = 0.01
    duration_sec: float = 18.0
    horizon_sec: float = 2.0
    snapshot_interval_sec: float = 2.0
    gamma: float = 0.99
    sigma_v: float = 0.05
    sigma_omega: float = 0.05
    seeds: tuple[int, ...] = (1, 2, 3, 4, 5, 6, 7, 8, 9)
    q_linear: float = 8.0
    q_angular: float = 12.0
    initial_heading_rad: float = 0.35
    initial_accel_var: float = 4.0
    initial_alpha_var: float = 9.0

    @property
    def horizon_steps(self) -> int:
        return int(round(self.horizon_sec / self.dt))

    @property
    def snapshot_stride(self) -> int:
        return int(round(self.snapshot_interval_sec / self.dt))


@dataclass(frozen=True)
class GroundTruthTrajectory:
    times: np.ndarray
    x: np.ndarray
    y: np.ndarray
    theta: np.ndarray
    v: np.ndarray
    omega: np.ndarray
    a: np.ndarray
    alpha: np.ndarray

    @property
    def positions(self) -> np.ndarray:
        return np.column_stack((self.x, self.y))


@dataclass(frozen=True)
class SeedResult:
    seed: int
    measured_v: np.ndarray
    measured_omega: np.ndarray
    estimated_v: np.ndarray
    estimated_omega: np.ndarray
    estimated_a: np.ndarray
    estimated_alpha: np.ndarray
    predictions: list[np.ndarray]


class ConstantAccelerationKalmanFilter:
    """1D KF for [velocity, acceleration] with a white-jerk process model."""

    def __init__(
        self,
        dt: float,
        process_noise_density: float,
        measurement_std: float,
        initial_accel_var: float,
    ) -> None:
        self.dt = float(dt)
        self.A = np.array([[1.0, self.dt], [0.0, 1.0]], dtype=float)
        self.H = np.array([[1.0, 0.0]], dtype=float)
        # Q comes from a white-jerk CA model: larger q tracks sharper maneuvers,
        # smaller q rejects more noise but makes the acceleration estimate less responsive.
        self.Q = process_noise_density * np.array(
            [
                [self.dt**3 / 3.0, self.dt**2 / 2.0],
                [self.dt**2 / 2.0, self.dt],
            ],
            dtype=float,
        )
        # R is fixed directly from the sensor noise variance.
        self.R = np.array([[measurement_std**2]], dtype=float)
        self.I = np.eye(2, dtype=float)
        self.initial_accel_var = float(initial_accel_var)
        self.x = np.zeros((2, 1), dtype=float)
        self.P = np.eye(2, dtype=float)
        self.initialized = False

    def initialize(self, measured_velocity: float) -> None:
        self.x[:, 0] = np.array([measured_velocity, 0.0], dtype=float)
        self.P = np.diag([self.R[0, 0], self.initial_accel_var]).astype(float)
        self.initialized = True

    def step(self, measurement: float) -> np.ndarray:
        if not self.initialized:
            self.initialize(measurement)

        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

        innovation = np.array([[measurement]], dtype=float) - self.H @ self.x
        innovation_cov = self.H @ self.P @ self.H.T + self.R
        kalman_gain = (self.P @ self.H.T) / innovation_cov[0, 0]

        self.x = self.x + kalman_gain @ innovation
        joseph_factor = self.I - kalman_gain @ self.H
        self.P = (
            joseph_factor @ self.P @ joseph_factor.T
            + kalman_gain @ self.R @ kalman_gain.T
        )
        return self.x[:, 0].copy()


def _repo_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1]


def _default_output_path() -> pathlib.Path:
    return _repo_root() / "results" / "ugv_short_horizon_prediction_grid.png"


def _configure_matplotlib() -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Serif",
            "font.size": 11,
            "axes.labelsize": 11,
            "axes.titlesize": 12,
            "legend.fontsize": 10,
            "xtick.labelsize": 10,
            "ytick.labelsize": 10,
            "axes.linewidth": 0.9,
            "grid.linewidth": 0.55,
        }
    )


def _integrate_kinematics(
    v: np.ndarray,
    omega: np.ndarray,
    a: np.ndarray,
    alpha: np.ndarray,
    dt: float,
    initial_heading_rad: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    num_steps = v.shape[0]
    x = np.zeros(num_steps, dtype=float)
    y = np.zeros(num_steps, dtype=float)
    theta = np.zeros(num_steps, dtype=float)
    theta[0] = float(initial_heading_rad)

    for k in range(num_steps - 1):
        travel = v[k] * dt + 0.5 * a[k] * dt**2
        theta_mid = theta[k] + 0.5 * omega[k] * dt + 0.125 * alpha[k] * dt**2
        x[k + 1] = x[k] + travel * np.cos(theta_mid)
        y[k + 1] = y[k] + travel * np.sin(theta_mid)
        theta[k + 1] = theta[k] + omega[k] * dt + 0.5 * alpha[k] * dt**2

    return x, y, theta


def _generate_ground_truth(config: SimulationConfig) -> GroundTruthTrajectory:
    times = np.arange(0.0, config.duration_sec + 0.5 * config.dt, config.dt)

    v_true = (
        1.15
        + 0.26 * np.sin(0.58 * times - 0.25)
        + 0.10 * np.sin(1.85 * times + 0.55)
        + 0.04 * np.cos(0.16 * times)
    )
    v_true = np.clip(v_true, 0.35, None)

    omega_true = (
        0.44 * np.sin(0.46 * times + 0.10)
        + 0.19 * np.sin(1.30 * times - 0.55)
        + 0.07 * np.cos(2.55 * times + 0.15)
    )

    a_true = np.gradient(v_true, config.dt, edge_order=2)
    alpha_true = np.gradient(omega_true, config.dt, edge_order=2)
    x, y, theta = _integrate_kinematics(
        v_true,
        omega_true,
        a_true,
        alpha_true,
        config.dt,
        config.initial_heading_rad,
    )

    return GroundTruthTrajectory(
        times=times,
        x=x,
        y=y,
        theta=theta,
        v=v_true,
        omega=omega_true,
        a=a_true,
        alpha=alpha_true,
    )


def _inject_measurement_noise(
    ground_truth: GroundTruthTrajectory,
    sigma_v: float,
    sigma_omega: float,
    seed: int,
) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(seed)
    measured_v = ground_truth.v + rng.normal(
        loc=0.0,
        scale=sigma_v,
        size=ground_truth.v.shape,
    )
    measured_omega = ground_truth.omega + rng.normal(
        loc=0.0,
        scale=sigma_omega,
        size=ground_truth.omega.shape,
    )
    return measured_v, measured_omega


def _prediction_snapshot_indices(num_steps: int, config: SimulationConfig) -> np.ndarray:
    max_start = num_steps - config.horizon_steps - 1
    if max_start < 0:
        raise ValueError("Trajectory is shorter than the requested prediction horizon.")
    return np.arange(0, max_start + 1, config.snapshot_stride, dtype=int)


def _predict_trajectory(
    x0: float,
    y0: float,
    theta0: float,
    v0: float,
    omega0: float,
    a0: float,
    alpha0: float,
    config: SimulationConfig,
) -> np.ndarray:
    prediction = np.zeros((config.horizon_steps + 1, 2), dtype=float)
    prediction[0] = np.array([x0, y0], dtype=float)

    x = float(x0)
    y = float(y0)
    theta = float(theta0)
    v = float(v0)
    omega = float(omega0)

    for j in range(config.horizon_steps):
        a_j = (config.gamma**j) * a0
        alpha_j = (config.gamma**j) * alpha0
        travel = v * config.dt + 0.5 * a_j * config.dt**2
        theta_mid = theta + 0.5 * omega * config.dt + 0.125 * alpha_j * config.dt**2

        x += travel * np.cos(theta_mid)
        y += travel * np.sin(theta_mid)
        theta += omega * config.dt + 0.5 * alpha_j * config.dt**2
        v += a_j * config.dt
        omega += alpha_j * config.dt
        prediction[j + 1] = np.array([x, y], dtype=float)

    return prediction


def _run_seed_simulation(
    ground_truth: GroundTruthTrajectory,
    snapshot_indices: np.ndarray,
    config: SimulationConfig,
    seed: int,
) -> SeedResult:
    measured_v, measured_omega = _inject_measurement_noise(
        ground_truth=ground_truth,
        sigma_v=config.sigma_v,
        sigma_omega=config.sigma_omega,
        seed=seed,
    )

    linear_kf = ConstantAccelerationKalmanFilter(
        dt=config.dt,
        process_noise_density=config.q_linear,
        measurement_std=config.sigma_v,
        initial_accel_var=config.initial_accel_var,
    )
    angular_kf = ConstantAccelerationKalmanFilter(
        dt=config.dt,
        process_noise_density=config.q_angular,
        measurement_std=config.sigma_omega,
        initial_accel_var=config.initial_alpha_var,
    )

    estimated_v = np.zeros_like(measured_v)
    estimated_omega = np.zeros_like(measured_omega)
    estimated_a = np.zeros_like(measured_v)
    estimated_alpha = np.zeros_like(measured_omega)
    predictions: list[np.ndarray] = []
    snapshot_set = set(snapshot_indices.tolist())

    for k in range(ground_truth.times.shape[0]):
        linear_state = linear_kf.step(measured_v[k])
        angular_state = angular_kf.step(measured_omega[k])

        estimated_v[k], estimated_a[k] = linear_state
        estimated_omega[k], estimated_alpha[k] = angular_state

        if k in snapshot_set:
            predictions.append(
                _predict_trajectory(
                    x0=ground_truth.x[k],
                    y0=ground_truth.y[k],
                    theta0=ground_truth.theta[k],
                    v0=estimated_v[k],
                    omega0=estimated_omega[k],
                    a0=estimated_a[k],
                    alpha0=estimated_alpha[k],
                    config=config,
                )
            )

    return SeedResult(
        seed=seed,
        measured_v=measured_v,
        measured_omega=measured_omega,
        estimated_v=estimated_v,
        estimated_omega=estimated_omega,
        estimated_a=estimated_a,
        estimated_alpha=estimated_alpha,
        predictions=predictions,
    )


def _plot_seed_grid(
    ground_truth: GroundTruthTrajectory,
    seed_results: list[SeedResult],
    snapshot_indices: np.ndarray,
    output_path: pathlib.Path,
    config: SimulationConfig,
    show: bool,
) -> None:
    _configure_matplotlib()

    snapshot_times = ground_truth.times[snapshot_indices]
    norm = Normalize(vmin=float(snapshot_times[0]), vmax=float(snapshot_times[-1]))
    cmap = plt.get_cmap("viridis")

    fig, axes = plt.subplots(
        3,
        3,
        figsize=(14.5, 11.5),
        sharex=True,
        sharey=True,
        constrained_layout=True,
    )

    for axis, seed_result in zip(axes.flat, seed_results):
        axis.plot(
            ground_truth.x,
            ground_truth.y,
            color="black",
            linewidth=2.0,
            label="Ground truth",
            zorder=1,
        )

        for launch_time, prediction in zip(snapshot_times, seed_result.predictions):
            color = cmap(norm(float(launch_time)))
            axis.plot(
                prediction[:, 0],
                prediction[:, 1],
                linestyle="--",
                color=color,
                linewidth=1.35,
                alpha=0.96,
                zorder=2,
            )
            axis.scatter(
                prediction[0, 0],
                prediction[0, 1],
                s=14.0,
                color=color,
                edgecolors="none",
                zorder=3,
            )

        axis.set_title(f"Seed {seed_result.seed}")
        axis.grid(True, linestyle=":", alpha=0.55)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")

    fig.suptitle(
        "UGV Short-Horizon Prediction Validation with KF-Based Acceleration Estimation\n"
        rf"$dt={config.dt:.2f}\,\mathrm{{s}},\ T={config.horizon_sec:.1f}\,\mathrm{{s}},\ "
        rf"\gamma={config.gamma:.2f},\ \sigma_v={config.sigma_v:.2f}\,\mathrm{{m/s}},\ "
        rf"\sigma_\omega={config.sigma_omega:.2f}\,\mathrm{{rad/s}}$",
        fontsize=14,
    )

    handles = [
        Line2D([0], [0], color="black", linewidth=2.0, label="Ground truth"),
        Line2D(
            [0],
            [0],
            color=cmap(norm(float(snapshot_times[min(1, len(snapshot_times) - 1)]))),
            linewidth=1.35,
            linestyle="--",
            label="2.0 s predictions",
        ),
    ]
    fig.legend(
        handles=handles,
        loc="upper right",
        ncol=1,
        frameon=True,
        framealpha=0.95,
        bbox_to_anchor=(0.85, 1.015),
    )

    colorbar = fig.colorbar(
        ScalarMappable(norm=norm, cmap=cmap),
        ax=axes,
        shrink=0.92,
        pad=0.02,
    )
    colorbar.set_label("Prediction launch time [s]")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220, bbox_inches="tight")

    if show:
        plt.show()
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Standalone 100 Hz UGV short-horizon motion prediction validation "
            "using KF-based acceleration estimation across 9 random seeds."
        )
    )
    parser.add_argument(
        "--out",
        type=str,
        default=str(_default_output_path()),
        help="Output PNG path.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the figure interactively after saving it.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = SimulationConfig()
    ground_truth = _generate_ground_truth(config)
    snapshot_indices = _prediction_snapshot_indices(ground_truth.times.shape[0], config)

    seed_results = [
        _run_seed_simulation(
            ground_truth=ground_truth,
            snapshot_indices=snapshot_indices,
            config=config,
            seed=seed,
        )
        for seed in config.seeds
    ]

    output_path = pathlib.Path(args.out).expanduser().resolve()
    _plot_seed_grid(
        ground_truth=ground_truth,
        seed_results=seed_results,
        snapshot_indices=snapshot_indices,
        output_path=output_path,
        config=config,
        show=bool(args.show),
    )

    print(f"saved figure: {output_path}")
    print(
        "KF tuning: "
        f"R_v={config.sigma_v**2:.5f}, R_omega={config.sigma_omega**2:.5f}, "
        f"q_linear={config.q_linear:.2f}, q_angular={config.q_angular:.2f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
