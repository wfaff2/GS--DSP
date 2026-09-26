#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.ticker import PercentFormatter
from matplotlib.transforms import blended_transform_factory

KP_ORDER = [1.0, 1.5, 2.0, 2.5, 3.0]

BLUE = "#0072B2"
RED = "#C23B22"
GRAY = "#7A7A7A"
LIGHT_RED = "#F2C9C1"
LIGHT_BLUE = "#CFE6F3"
REGION_GREEN = "#D7EBD8"
REGION_RED = "#F2D8D5"
GRID = "#D6DADF"
MUTED = "#5B6570"
DEADLINE_RED = "#D9472B"
SHADE = "#E8EDF2"

CASE_DISPLAY = {
    "inertial_frozen": "Inertial Frozen",
    "noninertial_frozen": "Non-inertial Frozen",
    "noninertial_stage": "Non-inertial Stage",
}

STYLE = {
    "Inertial Frozen": {"color": GRAY, "linestyle": ":", "marker": "o"},
    "Inertial": {"color": GRAY, "linestyle": ":", "marker": "^"},
    "Non-inertial Frozen": {"color": RED, "linestyle": (0, (4.5, 2.2)), "marker": "o"},
    "Non-inertial Stage": {"color": BLUE, "linestyle": "-", "marker": "s"},
    "Frozen": {"color": RED, "linestyle": (0, (4.5, 2.2)), "marker": "o"},
    "Stage": {"color": BLUE, "linestyle": "-", "marker": "s"},
    "True": {"color": BLUE, "linestyle": "-", "marker": None},
}

CASE_KP = 2.5
CASE_SEED = 14
CASE_WINDOW = (10.97, 12.47)
DIFFICULT_SEGMENT = (11.20, 12.20)
REVERSAL_TIME = 11.72
ALIGN_KP = 2.5
ALIGN_SMOOTH_WINDOW = 15
ALIGN_ACCEL_THR = 0.8
FROZEN_BACKFILL_WITH_CBF_ROOT = "codex_frozen_backfill_for_stageomega_with_cbf_kpgrid_20seed_20260403"
DEADLINE_MS = 20.0
MAX_VIOLIN_POINTS = 20000


@dataclass
class Fig3Case:
    omega_truth: pd.DataFrame
    frot_series: pd.DataFrame
    tracking_series: pd.DataFrame


@dataclass
class AlignmentData:
    plot_summary: pd.DataFrame
    stage_ugv_summary: pd.DataFrame
    per_seed_summary: pd.DataFrame
    seed_count: int
    stage_gt_frozen_points: int
    comparable_points: int
    stage_gt_frozen_not_worse_points: int


@dataclass
class AlignmentSeedStats:
    per_seed_summary: pd.DataFrame
    seed_count: int
    comparable_points: int
    stage_better_ratio: float
    stage_worse_ratio: float
    not_worse_when_pred_worse_ratio: float


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Generate the five publication figures used in the paper draft."
    )
    parser.add_argument(
        "--run-root",
        type=Path,
        default=repo_root / "results" / "codex_stage_omega_with_cbf_kpgrid_20seed_20260403",
        help="Unified with-CBF result root.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=repo_root / "results" / "new" / "ieee_publication_figures",
        help="Directory used to save the figures.",
    )
    return parser.parse_args()


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": ["Helvetica", "Arial", "DejaVu Sans"],
            "mathtext.fontset": "dejavusans",
            "font.size": 7.4,
            "axes.labelsize": 7.8,
            "axes.titlesize": 8.2,
            "xtick.labelsize": 6.8,
            "ytick.labelsize": 6.8,
            "legend.fontsize": 6.6,
            "axes.linewidth": 0.75,
            "lines.linewidth": 1.55,
            "lines.markersize": 4.2,
            "figure.dpi": 170,
            "savefig.dpi": 320,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "axes.unicode_minus": False,
        }
    )


def stylize_axes(ax, grid_axis: str = "y", hide_right: bool = True) -> None:
    ax.spines["top"].set_visible(False)
    if hide_right:
        ax.spines["right"].set_visible(False)
    ax.spines["left"].set_linewidth(0.75)
    ax.spines["bottom"].set_linewidth(0.75)
    ax.tick_params(direction="out", length=2.4, width=0.75, pad=2)
    ax.grid(axis=grid_axis, color=GRID, linewidth=0.55, alpha=0.55)
    ax.set_axisbelow(True)


def panel_title(ax, label: str, title: str) -> None:
    ax.text(
        0.0,
        1.015,
        f"({label}) {title}",
        transform=ax.transAxes,
        ha="left",
        va="bottom",
        fontsize=8.0,
        fontweight="semibold",
    )


def save_figure(fig, out_dir: Path, stem: str) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    pdf_path = out_dir / f"{stem}.pdf"
    png_path = out_dir / f"{stem}.png"
    fig.savefig(pdf_path, bbox_inches="tight")
    fig.savefig(png_path, bbox_inches="tight")
    plt.close(fig)
    print(f"[INFO] wrote {pdf_path}")
    print(f"[INFO] wrote {png_path}")


def kp_dir_name(kp: float) -> str:
    return f"kp_{str(kp).replace('.', 'p')}"


def infer_kp(path: Path) -> float | None:
    for part in path.parts:
        match = re.fullmatch(r"kp_(\d+)p(\d+)", part)
        if match:
            return float(f"{match.group(1)}.{match.group(2)}")
    return None


def infer_seed(path: Path) -> int | None:
    for part in path.parts:
        match = re.fullmatch(r"seed_(\d+)", part)
        if match:
            return int(match.group(1))
    return None


def load_csv(run_root: Path, name: str) -> pd.DataFrame:
    return pd.read_csv(run_root / name)


def load_fig1_data(run_root: Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    target_cases = ["inertial_frozen", "noninertial_stage", "noninertial_frozen"]
    display_names = {
        "inertial_frozen": "Inertial",
        "noninertial_stage": "Stage",
        "noninertial_frozen": "Frozen",
    }
    seed_metrics = (
        load_csv(run_root, "kp_grid_three_case_with_cbf_1to20_plus_kp3_seed_metrics.csv")
        .loc[:, ["case", "kp", "seed", "tracking_rms", "collision"]]
        .assign(
            kp=lambda df: pd.to_numeric(df["kp"], errors="coerce"),
            seed=lambda df: pd.to_numeric(df["seed"], errors="coerce"),
            tracking_rms=lambda df: pd.to_numeric(df["tracking_rms"], errors="coerce"),
            collision=lambda df: pd.to_numeric(df["collision"], errors="coerce"),
        )
        .loc[lambda df: df["case"].isin(target_cases) & df["kp"].isin(KP_ORDER)]
        .reset_index(drop=True)
    )

    collision = (
        seed_metrics.groupby(["case", "kp"], as_index=False)
        .agg(
            num_seeds=("seed", "count"),
            collision_count=("collision", "sum"),
        )
        .assign(
            collision_rate=lambda df: df["collision_count"] / df["num_seeds"],
            method=lambda df: df["case"].map(display_names),
        )
        .sort_values(["method", "kp"])
        .reset_index(drop=True)
    )

    common_seed_map = load_common_seed_map(run_root)
    common_rows: list[pd.DataFrame] = []
    for kp, seeds in common_seed_map.items():
        if not seeds:
            continue
        common_rows.append(
            seed_metrics[
                (seed_metrics["kp"] == float(kp))
                & (seed_metrics["seed"].isin(sorted(seeds)))
            ]
        )
    common_seed_metrics = pd.concat(common_rows, ignore_index=True) if common_rows else seed_metrics.iloc[0:0].copy()

    # Track only on the common no-collision subset so the three curves remain directly comparable
    # and do not inherit survivor bias from method-specific collisions at high k_p.
    tracking = (
        common_seed_metrics.loc[common_seed_metrics["collision"] == 0.0]
        .groupby(["case", "kp"], as_index=False)
        .agg(
            num_seeds=("seed", "count"),
            mean_rms=("tracking_rms", "mean"),
            std_rms=("tracking_rms", "std"),
            var_rms=("tracking_rms", "var"),
        )
        .assign(
            std_rms=lambda df: df["std_rms"].fillna(0.0),
            var_rms=lambda df: df["var_rms"].fillna(0.0),
            method=lambda df: df["case"].map(display_names),
        )
        .sort_values(["method", "kp"])
        .reset_index(drop=True)
    )
    return collision, tracking


def load_crot_mean_gain_data(run_root: Path) -> pd.DataFrame:
    target_cases = ["inertial_frozen", "noninertial_stage", "noninertial_frozen"]
    display_names = {
        "inertial_frozen": "Inertial",
        "noninertial_stage": "Stage",
        "noninertial_frozen": "Frozen",
    }
    df = (
        load_csv(run_root, "crot_summary_by_method_kp_common_nocollision.csv")
        .rename(columns={"method": "case"})
        .loc[:, ["case", "kp", "seed_count", "mean_c_rot_norm"]]
        .assign(
            kp=lambda x: pd.to_numeric(x["kp"], errors="coerce"),
            seed_count=lambda x: pd.to_numeric(x["seed_count"], errors="coerce"),
            mean_c_rot_norm=lambda x: pd.to_numeric(x["mean_c_rot_norm"], errors="coerce"),
        )
        .loc[lambda x: x["case"].isin(target_cases) & x["kp"].isin(KP_ORDER)]
    )
    base = (
        df[df["kp"] == 1.0][["case", "mean_c_rot_norm"]]
        .rename(columns={"mean_c_rot_norm": "mean_base"})
    )
    return (
        df.merge(base, on="case", how="left")
        .assign(
            method=lambda x: x["case"].map(display_names),
            mean_gain_vs_kp1=lambda x: x["mean_c_rot_norm"] / x["mean_base"],
        )
        .sort_values(["method", "kp"])
        .reset_index(drop=True)
    )


def load_fig2_data(run_root: Path) -> tuple[list[float], list[float], list[float], list[float]]:
    frozen = load_csv(run_root, "frot_frozen_mismatch_anchor_rows_common_nocollision.csv")
    stage = load_csv(run_root, "frot_stage_mismatch_common_nocollision_anchor_rows.csv")
    frozen_p99 = (
        frozen.groupby("kp")["frot_mismatch_rms"].quantile(0.99).reindex(KP_ORDER)
    )
    stage_p99 = (
        stage.groupby("kp")["frot_stage_mismatch_rms"].quantile(0.99).reindex(KP_ORDER)
    )
    frozen_mean = frozen.groupby("kp")["frot_mismatch_rms"].mean().reindex(KP_ORDER)
    stage_mean = stage.groupby("kp")["frot_stage_mismatch_rms"].mean().reindex(KP_ORDER)
    return (
        [float(frozen_mean.loc[kp]) for kp in KP_ORDER],
        [float(stage_mean.loc[kp]) for kp in KP_ORDER],
        [float(frozen_p99.loc[kp]) for kp in KP_ORDER],
        [float(stage_p99.loc[kp]) for kp in KP_ORDER],
    )


def cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.stack(
        [
            a[:, 1] * b[:, 2] - a[:, 2] * b[:, 1],
            a[:, 2] * b[:, 0] - a[:, 0] * b[:, 2],
            a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0],
        ],
        axis=1,
    )


def compute_frot(omega: np.ndarray, velocity: np.ndarray, position: np.ndarray) -> np.ndarray:
    return 2.0 * cross(omega, velocity) + cross(omega, cross(omega, position))


def load_truth_step(seed_dir: Path, frozen: bool) -> pd.DataFrame:
    name = "noninertial_metrics_steps.csv" if frozen else "noninertial_stage_metrics_steps.csv"
    df = pd.read_csv(seed_dir / name)
    cols = [
        "sim_time",
        "step_idx",
        "uav_idx",
        "tracking_error",
        "rot_load_position_non_x",
        "rot_load_position_non_y",
        "rot_load_position_non_z",
        "rot_load_velocity_non_x",
        "rot_load_velocity_non_y",
        "rot_load_velocity_non_z",
        "rot_load_omega_non_x",
        "rot_load_omega_non_y",
        "rot_load_omega_non_z",
    ]
    use_cols = [col for col in cols if col in df.columns]
    df = df[use_cols].copy()
    for col in use_cols:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df.sort_values(["uav_idx", "step_idx"]).copy()


def load_pred_omega_lookup(path: Path) -> dict[tuple[int, int], np.ndarray]:
    df = pd.read_csv(
        path,
        usecols=[
            "step_idx",
            "horizon_idx",
            "pred_omega_non_x",
            "pred_omega_non_y",
            "pred_omega_non_z",
        ],
    )
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return {
        (int(row.step_idx), int(row.horizon_idx)): np.array(
            [row.pred_omega_non_x, row.pred_omega_non_y, row.pred_omega_non_z],
            dtype=float,
        )
        for row in df.itertuples(index=False)
    }


def load_prediction_stride(metrics_path: Path, step_df: pd.DataFrame) -> tuple[float, int]:
    metrics = pd.read_csv(metrics_path)
    scope = metrics[metrics["scope"] == "all"].iloc[0]
    prediction_dt = float(scope["prediction_dt_sec"])
    sim_dt = float(
        np.median(
            np.diff(
                step_df.groupby("step_idx", as_index=False)["sim_time"]
                .mean()
                .sort_values("step_idx")["sim_time"]
                .to_numpy()
            )
        )
    )
    stride = int(round(prediction_dt / sim_dt))
    return prediction_dt, stride


def load_fig3_case(run_root: Path) -> Fig3Case:
    seed_dir = run_root / f"kp_{str(CASE_KP).replace('.', 'p')}" / f"seed_{CASE_SEED}"
    truth_step = load_truth_step(seed_dir, frozen=True)
    stage_step = load_truth_step(seed_dir, frozen=False)

    prediction_dt, stride = load_prediction_stride(seed_dir / "noninertial_metrics.csv", truth_step)
    frozen_lookup = load_pred_omega_lookup(seed_dir / "noninertial_metrics_ugv_horizon_steps.csv")
    stage_lookup = load_pred_omega_lookup(seed_dir / "noninertial_stage_metrics_ugv_horizon_steps.csv")

    omega_truth = (
        truth_step.groupby("sim_time", as_index=False)["rot_load_omega_non_z"]
        .mean()
        .rename(columns={"rot_load_omega_non_z": "true_omega_z"})
    )
    omega_truth = omega_truth[
        (omega_truth["sim_time"] >= CASE_WINDOW[0]) & (omega_truth["sim_time"] <= CASE_WINDOW[1])
    ].copy()

    tracking_series = (
        truth_step.groupby("sim_time", as_index=False)["tracking_error"]
        .mean()
        .rename(columns={"tracking_error": "frozen_tracking_error"})
        .merge(
            stage_step.groupby("sim_time", as_index=False)["tracking_error"]
            .mean()
            .rename(columns={"tracking_error": "stage_tracking_error"}),
            on="sim_time",
            how="inner",
        )
    )
    tracking_series = tracking_series[
        (tracking_series["sim_time"] >= CASE_WINDOW[0])
        & (tracking_series["sim_time"] <= CASE_WINDOW[1])
    ].copy()

    records: list[dict[str, float]] = []
    for uav_idx, df_uav in truth_step.groupby("uav_idx"):
        df_uav = df_uav.sort_values("step_idx").reset_index(drop=True)
        step_idx = df_uav["step_idx"].astype(int).to_numpy()
        sim_time = df_uav["sim_time"].to_numpy(dtype=float)
        pos = df_uav[
            ["rot_load_position_non_x", "rot_load_position_non_y", "rot_load_position_non_z"]
        ].to_numpy(dtype=float)
        vel = df_uav[
            ["rot_load_velocity_non_x", "rot_load_velocity_non_y", "rot_load_velocity_non_z"]
        ].to_numpy(dtype=float)
        omg = df_uav[
            ["rot_load_omega_non_x", "rot_load_omega_non_y", "rot_load_omega_non_z"]
        ].to_numpy(dtype=float)
        true_frot = compute_frot(omg, vel, pos)
        future_index_by_step = {int(step): idx for idx, step in enumerate(step_idx)}

        for anchor_idx, anchor_step in enumerate(step_idx):
            for horizon_idx in range(1, 21):
                future_step = int(anchor_step) + stride * horizon_idx
                future_local_idx = future_index_by_step.get(future_step)
                if future_local_idx is None:
                    continue
                target_time = float(sim_time[future_local_idx])
                if not (CASE_WINDOW[0] <= target_time <= CASE_WINDOW[1]):
                    continue

                frozen_pred_omega = frozen_lookup.get((int(anchor_step), horizon_idx))
                stage_pred_omega = stage_lookup.get((int(anchor_step), horizon_idx))
                if frozen_pred_omega is None or stage_pred_omega is None:
                    continue

                future_vel = vel[future_local_idx].reshape(1, 3)
                future_pos = pos[future_local_idx].reshape(1, 3)
                frozen_pred = compute_frot(frozen_pred_omega.reshape(1, 3), future_vel, future_pos)[0]
                stage_pred = compute_frot(stage_pred_omega.reshape(1, 3), future_vel, future_pos)[0]

                records.append(
                    {
                        "target_time": round(target_time, 2),
                        "uav_idx": float(uav_idx),
                        "true_frot": float(np.linalg.norm(true_frot[future_local_idx])),
                        "frozen_pred": float(np.linalg.norm(frozen_pred)),
                        "stage_pred": float(np.linalg.norm(stage_pred)),
                    }
                )

    if not records:
        raise RuntimeError("Failed to assemble figure-3 local case series.")

    frot_series = (
        pd.DataFrame(records)
        .groupby("target_time", as_index=False)[["true_frot", "frozen_pred", "stage_pred"]]
        .mean()
        .rename(columns={"target_time": "sim_time"})
        .sort_values("sim_time")
    )

    return Fig3Case(
        omega_truth=omega_truth,
        frot_series=frot_series,
        tracking_series=tracking_series,
    )


def iter_noninertial_step_paths(run_root: Path) -> list[Path]:
    paths: list[Path] = []
    for path in sorted(run_root.rglob("seed_*/*_metrics_steps.csv")):
        if path.name.endswith("_ugv_horizon_steps.csv"):
            continue
        if path.name in {"noninertial_metrics_steps.csv", "noninertial_stage_metrics_steps.csv"}:
            paths.append(path)
    return paths


def load_common_seed_map(run_root: Path) -> dict[float, set[int]]:
    df = load_csv(run_root, "common_no_collision_seeds_by_kp.csv")
    out: dict[float, set[int]] = {}
    for kp, sub in df.groupby("kp"):
        out[float(kp)] = set(sub["seed"].astype(int))
    return out


def build_c2_alignment_legacy_source(run_root: Path, staging_dir: Path, kp: float = ALIGN_KP) -> Path:
    kp_dir = kp_dir_name(kp)
    stage_csv = run_root / kp_dir / "combined_robustness_data.csv"
    frozen_root = run_root.parent / FROZEN_BACKFILL_WITH_CBF_ROOT
    frozen_csv = frozen_root / kp_dir / "combined_robustness_data.csv"
    if not stage_csv.exists():
        raise FileNotFoundError(f"Stage robustness CSV not found: {stage_csv}")
    if not frozen_csv.exists():
        raise FileNotFoundError(f"Frozen robustness CSV not found: {frozen_csv}")

    stage_df = pd.read_csv(stage_csv)
    frozen_df = pd.read_csv(frozen_csv)
    stage_seed_ids = set(pd.to_numeric(stage_df["seed"], errors="coerce").dropna().astype(int))
    frozen_seed_ids = set(pd.to_numeric(frozen_df["seed"], errors="coerce").dropna().astype(int))
    seed_ids = sorted(stage_seed_ids & frozen_seed_ids)
    if not seed_ids:
        raise RuntimeError(f"No overlapping stage/frozen seeds found for k_p={kp:.1f}")

    # Restore the original Fig. 3 data scope: aggregate the full stage/frozen
    # time series over all available seeds, including early-stop collision runs.
    stage_df = stage_df[stage_df["seed"].astype(int).isin(seed_ids)].copy()
    frozen_df = frozen_df[frozen_df["seed"].astype(int).isin(seed_ids)].copy()
    merged = pd.concat([frozen_df, stage_df], ignore_index=True)
    merged_path = staging_dir / f"fig_c2_alignment_robustness_{kp_dir}_20seed_merged.csv"
    merged.to_csv(merged_path, index=False)

    def link_or_copy(src: Path, dst: Path) -> None:
        if dst.exists() or dst.is_symlink():
            dst.unlink()
        try:
            dst.symlink_to(src.resolve())
        except OSError:
            shutil.copy2(src, dst)

    for seed in seed_ids:
        seed_dir = staging_dir / f"seed_{seed}"
        seed_dir.mkdir(parents=True, exist_ok=True)
        link_or_copy(
            run_root / kp_dir / f"seed_{seed}" / "noninertial_stage_metrics.csv",
            seed_dir / "noninertial_metrics.csv",
        )
        link_or_copy(
            frozen_root / kp_dir / f"seed_{seed}" / "noninertial_metrics.csv",
            seed_dir / "noninertial_frozen_metrics.csv",
        )
    return merged_path


def load_alignment_step_summary(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    keep = [
        "sim_time",
        "tracking_error",
        "car_a_world_x",
        "car_a_world_y",
        "car_a_world_z",
        "car_omega_world_x",
        "car_omega_world_y",
        "car_omega_world_z",
    ]
    use_cols = [col for col in keep if col in df.columns]
    df = df[use_cols].copy()
    for col in use_cols:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df["car_a_norm"] = np.sqrt(
        df.get("car_a_world_x", 0.0) ** 2
        + df.get("car_a_world_y", 0.0) ** 2
        + df.get("car_a_world_z", 0.0) ** 2
    )
    df["car_omega_norm"] = np.sqrt(
        df.get("car_omega_world_x", 0.0) ** 2
        + df.get("car_omega_world_y", 0.0) ** 2
        + df.get("car_omega_world_z", 0.0) ** 2
    )
    return (
        df.groupby("sim_time", as_index=False)
        .agg(
            tracking_error=("tracking_error", "mean"),
            car_a_norm=("car_a_norm", "mean"),
            car_omega_norm=("car_omega_norm", "mean"),
        )
        .sort_values("sim_time")
        .reset_index(drop=True)
    )


def load_alignment_horizon_summary(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, usecols=["sim_time", "horizon_rms_xy"])
    for col in ("sim_time", "horizon_rms_xy"):
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df.sort_values("sim_time").reset_index(drop=True)


def smooth_by_seed_method(df: pd.DataFrame, columns: list[str], window: int) -> pd.DataFrame:
    out = df.sort_values(["seed", "method", "sim_time"]).copy()
    for column in columns:
        out[f"{column}_smooth"] = out.groupby(["seed", "method"])[column].transform(
            lambda values: values.rolling(window, min_periods=1, center=True).mean()
        )
    return out


def summarize_alignment_series(df: pd.DataFrame) -> pd.DataFrame:
    summary = (
        df.groupby(["method", "sim_time"], as_index=False)
        .agg(
            tracking_error_mean=("tracking_error_smooth", "mean"),
            tracking_error_std=("tracking_error_smooth", "std"),
            horizon_rms_xy_mean=("horizon_rms_xy_smooth", "mean"),
            horizon_rms_xy_std=("horizon_rms_xy_smooth", "std"),
            car_a_norm_mean=("car_a_norm_smooth", "mean"),
            car_omega_norm_mean=("car_omega_norm_smooth", "mean"),
        )
        .sort_values(["method", "sim_time"])
        .reset_index(drop=True)
    )
    for column in ("tracking_error_std", "horizon_rms_xy_std"):
        summary[column] = summary[column].fillna(0.0)
    return summary


def identify_high_g_regions_from_summary(
    sim_time: np.ndarray,
    accel_norm: np.ndarray,
    threshold: float,
) -> list[tuple[float, float]]:
    if sim_time.size == 0:
        return []
    active = accel_norm > threshold
    if not np.any(active):
        return []
    regions: list[tuple[float, float]] = []
    start: float | None = None
    prev_t = float(sim_time[0])
    for t, is_active in zip(sim_time, active):
        t = float(t)
        if is_active and start is None:
            start = t
        if not is_active and start is not None:
            regions.append((start, prev_t))
            start = None
        prev_t = t
    if start is not None:
        regions.append((start, prev_t))
    return regions


def plot_mean_band(
    ax,
    summary: pd.DataFrame,
    method: str,
    mean_key: str,
    std_key: str,
    label: str,
    alpha: float,
    zorder: int,
) -> Line2D | None:
    method_df = summary[summary["method"] == method].sort_values("sim_time")
    if method_df.empty:
        return None
    style = STYLE[method]
    time = method_df["sim_time"].to_numpy(dtype=float)
    mean = method_df[mean_key].to_numpy(dtype=float)
    std = method_df[std_key].to_numpy(dtype=float)
    lower = np.maximum(mean - std, 0.0)
    upper = mean + std
    ax.fill_between(time, lower, upper, color=style["color"], alpha=alpha, linewidth=0.0, zorder=zorder - 1)
    line = ax.plot(
        time,
        mean,
        color=style["color"],
        linestyle=style["linestyle"],
        linewidth=2.0 if method != "Inertial" else 1.9,
        label=label,
        zorder=zorder,
    )[0]
    return line


def plot_paired_seed_metric(
    ax,
    per_seed_summary: pd.DataFrame,
    metric: str,
    ylabel: str,
    label: str,
    title: str,
) -> None:
    order = ["Frozen", "Stage"]
    positions = {"Frozen": 0.0, "Stage": 1.0}
    paired = (
        per_seed_summary.pivot(index="seed", columns="Method", values=metric)
        .dropna(subset=order)
        .sort_index()
    )
    rng = np.random.default_rng(0)
    jitter = {
        method: rng.uniform(-0.06, 0.06, size=len(paired))
        for method in order
    }
    for idx, (_, row) in enumerate(paired.iterrows()):
        ax.plot(
            [positions["Frozen"] + jitter["Frozen"][idx], positions["Stage"] + jitter["Stage"][idx]],
            [float(row["Frozen"]), float(row["Stage"])],
            color="#C7CED6",
            linewidth=0.9,
            alpha=0.55,
            zorder=1,
        )

    for method in order:
        style = STYLE[method]
        values = paired[method].to_numpy(dtype=float)
        x = np.full(len(values), positions[method]) + jitter[method]
        ax.scatter(
            x,
            values,
            s=22,
            color=style["color"],
            alpha=0.28,
            edgecolors="none",
            zorder=2,
        )
        mean = float(np.mean(values))
        std = float(np.std(values, ddof=1)) if len(values) > 1 else 0.0
        ax.errorbar(
            positions[method],
            mean,
            yerr=std,
            fmt=style["marker"],
            color=style["color"],
            markerfacecolor="white",
            markeredgewidth=1.3,
            markersize=7.2,
            elinewidth=1.7,
            capsize=4.0,
            linewidth=0.0,
            zorder=4,
        )
        ax.text(
            positions[method],
            mean + std + 0.03 * max(1.0, ax.get_ylim()[1] - ax.get_ylim()[0]),
            f"{mean:.3f} ± {std:.3f}",
            ha="center",
            va="bottom",
            fontsize=8.6,
            color=style["color"],
        )

    panel_title(ax, label, title)
    stylize_axes(ax, "y")
    ax.set_ylabel(ylabel)
    ax.set_xticks([positions[method] for method in order])
    ax.set_xticklabels(order)
    ax.set_xlim(-0.45, 1.45)


def load_alignment_data(run_root: Path, kp: float = ALIGN_KP) -> AlignmentData:
    common_seed_map = load_common_seed_map(run_root)
    seeds = sorted(common_seed_map.get(kp, set()))
    if not seeds:
        raise RuntimeError(f"No common no-collision seeds found for k_p={kp:.1f}.")
    kp_dir = run_root / f"kp_{str(kp).replace('.', 'p')}"
    method_files = {
        "Inertial": ("inertial_metrics_steps.csv", "inertial_metrics_ugv_horizon_summary.csv"),
        "Frozen": ("noninertial_metrics_steps.csv", "noninertial_metrics_ugv_horizon_summary.csv"),
        "Stage": ("noninertial_stage_metrics_steps.csv", "noninertial_stage_metrics_ugv_horizon_summary.csv"),
    }
    rows: list[pd.DataFrame] = []
    per_seed_rows: list[dict[str, float | int | str]] = []
    for method, (step_name, horizon_name) in method_files.items():
        for seed in seeds:
            seed_dir = kp_dir / f"seed_{seed}"
            step = load_alignment_step_summary(seed_dir / step_name)
            horizon = load_alignment_horizon_summary(seed_dir / horizon_name)
            merged = step.merge(horizon, on="sim_time", how="inner").sort_values("sim_time").copy()
            merged["seed"] = int(seed)
            merged["method"] = method
            rows.append(merged)
            per_seed_rows.append(
                {
                    "Method": method,
                    "seed": int(seed),
                    "mean_horizon_rmse": float(merged["horizon_rms_xy"].mean()),
                    "tracking_rms": float(np.sqrt(np.mean(np.square(merged["tracking_error"])))),
                    "max_tracking_error": float(merged["tracking_error"].max()),
                }
            )
    combined = smooth_by_seed_method(
        pd.concat(rows, ignore_index=True),
        ["tracking_error", "horizon_rms_xy", "car_a_norm", "car_omega_norm"],
        window=ALIGN_SMOOTH_WINDOW,
    )
    summary = summarize_alignment_series(combined)
    stage_ugv = summary[summary["method"] == "Stage"].sort_values("sim_time").reset_index(drop=True)
    piv_h = (
        summary.pivot(index="sim_time", columns="method", values="horizon_rms_xy_mean")
        .dropna(subset=["Stage", "Frozen"])
        .sort_index()
    )
    piv_t = (
        summary.pivot(index="sim_time", columns="method", values="tracking_error_mean")
        .dropna(subset=["Stage", "Frozen"])
        .sort_index()
    )
    stage_gt_frozen = int((piv_h["Stage"] > piv_h["Frozen"]).sum())
    stage_gt_frozen_not_worse = int(
        ((piv_h["Stage"] > piv_h["Frozen"]) & (piv_t["Stage"] <= 1.05 * piv_t["Frozen"])).sum()
    )
    return AlignmentData(
        plot_summary=summary,
        stage_ugv_summary=stage_ugv,
        per_seed_summary=pd.DataFrame(per_seed_rows),
        seed_count=len(seeds),
        stage_gt_frozen_points=stage_gt_frozen,
        comparable_points=int(len(piv_h)),
        stage_gt_frozen_not_worse_points=stage_gt_frozen_not_worse,
    )


def load_alignment_seed_stats(run_root: Path, kp: float = ALIGN_KP) -> AlignmentSeedStats:
    kp_dir = run_root / f"kp_{str(kp).replace('.', 'p')}"
    method_files = {
        "Frozen": ("noninertial_metrics_steps.csv", "noninertial_metrics_ugv_horizon_summary.csv"),
        "Stage": ("noninertial_stage_metrics_steps.csv", "noninertial_stage_metrics_ugv_horizon_summary.csv"),
    }
    per_seed_rows: list[dict[str, float | int | str]] = []
    comparable_points = 0
    stage_better = 0
    stage_worse = 0
    not_worse_when_pred_worse = 0
    valid_seed_count = 0

    for seed_dir in sorted(p for p in kp_dir.glob("seed_*") if p.is_dir()):
        method_payload: dict[str, tuple[pd.DataFrame, pd.DataFrame]] = {}
        for method, (step_name, horizon_name) in method_files.items():
            step_path = seed_dir / step_name
            horizon_path = seed_dir / horizon_name
            if not step_path.exists() or not horizon_path.exists():
                method_payload = {}
                break
            step = load_alignment_step_summary(step_path)
            horizon = load_alignment_horizon_summary(horizon_path)
            method_payload[method] = (step, horizon)

            per_seed_rows.append(
                {
                    "Method": method,
                    "seed": int(seed_dir.name.split("_", 1)[1]),
                    "tracking_rms": float(np.sqrt(np.mean(np.square(step["tracking_error"])))),
                    "mean_horizon_rmse": float(horizon["horizon_rms_xy"].mean()),
                    "max_tracking_error": float(step["tracking_error"].max()),
                }
            )

        if set(method_payload.keys()) != {"Frozen", "Stage"}:
            continue

        valid_seed_count += 1
        stage_step, stage_horizon = method_payload["Stage"]
        frozen_step, frozen_horizon = method_payload["Frozen"]
        merged_h = stage_horizon.merge(
            frozen_horizon,
            on="sim_time",
            how="inner",
            suffixes=("_stage", "_frozen"),
        )
        merged_t = stage_step[["sim_time", "tracking_error"]].merge(
            frozen_step[["sim_time", "tracking_error"]],
            on="sim_time",
            how="inner",
            suffixes=("_stage", "_frozen"),
        )
        merged = merged_h.merge(merged_t, on="sim_time", how="inner")
        if merged.empty:
            continue
        comparable_points += int(len(merged))
        stage_better += int((merged["horizon_rms_xy_stage"] < merged["horizon_rms_xy_frozen"]).sum())
        stage_worse += int((merged["horizon_rms_xy_stage"] > merged["horizon_rms_xy_frozen"]).sum())
        pred_worse = merged["horizon_rms_xy_stage"] > merged["horizon_rms_xy_frozen"]
        not_worse_when_pred_worse += int(
            (pred_worse & (merged["tracking_error_stage"] <= 1.05 * merged["tracking_error_frozen"])).sum()
        )

    per_seed_summary = pd.DataFrame(per_seed_rows)
    if comparable_points == 0:
        raise RuntimeError(f"No comparable stage/frozen alignment samples found for k_p={kp:.1f}.")
    return AlignmentSeedStats(
        per_seed_summary=per_seed_summary,
        seed_count=valid_seed_count,
        comparable_points=comparable_points,
        stage_better_ratio=stage_better / comparable_points,
        stage_worse_ratio=stage_worse / comparable_points,
        not_worse_when_pred_worse_ratio=not_worse_when_pred_worse / max(stage_worse, 1),
    )


def sample_for_violin(values: np.ndarray, max_points: int = MAX_VIOLIN_POINTS) -> np.ndarray:
    values = values[np.isfinite(values)]
    if values.size <= max_points:
        return values
    tail_keep = min(400, max(100, values.size // 200))
    tail_keep = min(tail_keep, max_points - 1)
    order = np.argsort(values)
    tail = values[order[-tail_keep:]]
    body = values[order[:-tail_keep]]
    rng = np.random.default_rng(0)
    idx = rng.choice(body.size, size=max_points - tail_keep, replace=False)
    return np.concatenate([body[idx], tail])


def load_fig4_data(run_root: Path) -> tuple[dict[str, dict[float, np.ndarray]], dict[str, np.ndarray], dict[str, np.ndarray]]:
    common_seed_map = load_common_seed_map(run_root)
    prep_by_kp = {
        "Non-inertial Frozen": {kp: [] for kp in KP_ORDER},
        "Non-inertial Stage": {kp: [] for kp in KP_ORDER},
    }
    solve_kp3_avoid = {"Non-inertial Frozen": [], "Non-inertial Stage": []}
    total_kp3_avoid = {"Non-inertial Frozen": [], "Non-inertial Stage": []}

    for path in iter_noninertial_step_paths(run_root):
        kp = infer_kp(path)
        seed = infer_seed(path)
        if kp is None:
            continue
        if seed is None or seed not in common_seed_map.get(kp, set()):
            continue
        method = "Non-inertial Stage" if "stage" in path.name else "Non-inertial Frozen"
        df = pd.read_csv(
            path,
            usecols=["solver_cbf_active", "preparation_time_ms", "feedback_time_ms", "core_time_ms"],
        )
        for col in ("solver_cbf_active", "preparation_time_ms", "feedback_time_ms", "core_time_ms"):
            df[col] = pd.to_numeric(df[col], errors="coerce")

        prep = df["preparation_time_ms"].to_numpy(dtype=float)
        prep_by_kp[method][kp].append(prep[np.isfinite(prep)])

        if math.isclose(kp, 3.0):
            avoid_mask = df["solver_cbf_active"].fillna(0).to_numpy(dtype=float) > 0.0
            solve_vals = df.loc[avoid_mask, "feedback_time_ms"].to_numpy(dtype=float)
            total_vals = df.loc[avoid_mask, "core_time_ms"].to_numpy(dtype=float)
            solve_kp3_avoid[method].append(solve_vals[np.isfinite(solve_vals)])
            total_kp3_avoid[method].append(total_vals[np.isfinite(total_vals)])

    prep_arrays = {
        method: {
            kp: sample_for_violin(np.concatenate(parts) if parts else np.array([], dtype=float))
            for kp, parts in data.items()
        }
        for method, data in prep_by_kp.items()
    }
    solve_arrays = {
        method: sample_for_violin(np.concatenate(parts) if parts else np.array([], dtype=float))
        for method, parts in solve_kp3_avoid.items()
    }
    total_arrays = {
        method: sample_for_violin(np.concatenate(parts) if parts else np.array([], dtype=float))
        for method, parts in total_kp3_avoid.items()
    }
    return prep_arrays, solve_arrays, total_arrays


def load_fig5_data(
    run_root: Path,
) -> tuple[dict[str, list[float]], dict[str, list[float]], dict[str, list[float]]]:
    common = load_csv(run_root, "kp_grid_three_case_with_cbf_common_nocollision_summary.csv")
    case_order = [
        ("inertial_frozen", "Inertial"),
        ("noninertial_frozen", "Frozen"),
        ("noninertial_stage", "Stage"),
    ]
    sub = common[common["case"].isin([case for case, _ in case_order])].copy()
    slack = {
        label: [
            float(sub[(sub["case"] == case) & (sub["kp"] == kp)]["slack_sum_effective_mean"].iloc[0])
            for kp in KP_ORDER
        ]
        for case, label in case_order
    }
    solver = {
        label: [
            float(sub[(sub["case"] == case) & (sub["kp"] == kp)]["solver_tracking_rms_avoid_mean"].iloc[0])
            for kp in KP_ORDER
        ]
        for case, label in case_order
    }
    min_h = {
        label: [
            float(sub[(sub["case"] == case) & (sub["kp"] == kp)]["min_h_mean"].iloc[0])
            for kp in KP_ORDER
        ]
        for case, label in case_order
    }
    return slack, solver, min_h


def add_method_legend(
    fig,
    labels: list[str],
    anchor_y: float = 1.02,
    marker_only: bool = False,
    framed: bool = False,
) -> None:
    handles = []
    for label in labels:
        style = STYLE[label]
        handles.append(
            Line2D(
                [0],
                [0],
                color=style["color"],
                linestyle="None" if marker_only else style["linestyle"],
                marker=style["marker"],
                markerfacecolor="white",
                markeredgewidth=1.1,
                linewidth=0.0 if marker_only else 1.8,
                markersize=7.0 if marker_only else 6.0,
                label=label,
            )
        )
    fig.legend(
        handles=handles,
        labels=labels,
        loc="upper center",
        ncol=len(labels),
        frameon=framed,
        framealpha=0.94 if framed else None,
        fancybox=False,
        edgecolor="black" if framed else None,
        bbox_to_anchor=(0.5, anchor_y),
        handlelength=2.0,
        columnspacing=1.0,
        handletextpad=0.5,
    )


def plot_grouped_dot(
    ax,
    series: dict[str, list[float]],
    ylabel: str,
    label: str,
    title: str,
) -> dict[str, Line2D]:
    handles: dict[str, Line2D] = {}
    for method in series:
        style = STYLE[method]
        (line,) = ax.plot(
            KP_ORDER,
            series[method],
            color=style["color"],
            linestyle=style["linestyle"],
            marker=style["marker"],
            markerfacecolor="white",
            markeredgewidth=1.1,
            linewidth=1.8,
            label=method,
        )
        handles[method] = line
    panel_title(ax, label, title)
    stylize_axes(ax, "y")
    ax.set_xlabel("$k_p$")
    ax.set_ylabel(ylabel)
    return handles
    ax.set_xticks(KP_ORDER)


def plot_gap_dumbbell(
    ax,
    frozen_values: list[float],
    stage_values: list[float],
    xlabel: str,
    label: str,
    title: str,
) -> None:
    y = np.arange(len(KP_ORDER))
    for idx, kp in enumerate(KP_ORDER):
        x0 = stage_values[idx]
        x1 = frozen_values[idx]
        lo, hi = sorted((x0, x1))
        ax.fill_betweenx([idx - 0.085, idx + 0.085], lo, hi, color=SHADE, alpha=0.38, linewidth=0.0, zorder=1)
        ax.plot([lo, hi], [idx, idx], color="#A9B2BB", linewidth=1.25, solid_capstyle="round", zorder=2)
        ax.plot(
            x1,
            idx,
            linestyle="None",
            marker=STYLE["Frozen"]["marker"],
            color=STYLE["Frozen"]["color"],
            markerfacecolor="white",
            markeredgewidth=1.15,
            markersize=5.8,
            zorder=3,
        )
        ax.plot(
            x0,
            idx,
            linestyle="None",
            marker=STYLE["Stage"]["marker"],
            color=STYLE["Stage"]["color"],
            markerfacecolor="white",
            markeredgewidth=1.15,
            markersize=5.8,
            zorder=3,
        )
    ax.set_title(f"({label}) {title}", pad=10)
    stylize_axes(ax, "x")
    ax.set_xlabel(xlabel)
    ax.set_yticks(y)
    ax.set_yticklabels([f"$k_p$={kp:.1f}" for kp in KP_ORDER])
    ax.invert_yaxis()
    ax.margins(x=0.06)


def add_gap_subplot_note(ax) -> None:
    marker_x = 0.80
    text_x = 0.845
    y_top = 0.90
    y_bottom = 0.82
    ax.scatter(
        [marker_x],
        [y_top],
        transform=ax.transAxes,
        marker=STYLE["Stage"]["marker"],
        s=42,
        facecolors="white",
        edgecolors=STYLE["Stage"]["color"],
        linewidths=1.15,
        zorder=4,
        clip_on=False,
    )
    ax.scatter(
        [marker_x],
        [y_bottom],
        transform=ax.transAxes,
        marker=STYLE["Frozen"]["marker"],
        s=42,
        facecolors="white",
        edgecolors=STYLE["Frozen"]["color"],
        linewidths=1.15,
        zorder=4,
        clip_on=False,
    )
    ax.text(
        text_x,
        y_top,
        "Stage",
        transform=ax.transAxes,
        ha="left",
        va="center",
        fontsize=8.7,
        color=STYLE["Stage"]["color"],
        bbox={
            "boxstyle": "round,pad=0.18",
            "facecolor": "white",
            "edgecolor": "none",
            "alpha": 0.82,
        },
    )
    ax.text(
        text_x,
        y_bottom,
        "Frozen",
        transform=ax.transAxes,
        ha="left",
        va="center",
        fontsize=8.7,
        color=STYLE["Frozen"]["color"],
        bbox={
            "boxstyle": "round,pad=0.18",
            "facecolor": "white",
            "edgecolor": "none",
            "alpha": 0.82,
        },
    )


def add_difficult_segment(ax) -> None:
    ax.axvspan(DIFFICULT_SEGMENT[0], DIFFICULT_SEGMENT[1], color=SHADE, alpha=0.28, zorder=0)
    ax.axvline(REVERSAL_TIME, color=MUTED, linestyle=(0, (4, 3)), linewidth=0.95)


def add_violin(ax, data: list[np.ndarray], positions: list[float], color: str) -> None:
    parts = ax.violinplot(
        data,
        positions=positions,
        widths=0.28,
        showmeans=False,
        showmedians=True,
        showextrema=False,
    )
    for body in parts["bodies"]:
        body.set_facecolor(color)
        body.set_edgecolor(color)
        body.set_alpha(0.28)
        body.set_linewidth(0.9)
    parts["cmedians"].set_color(color)
    parts["cmedians"].set_linewidth(1.5)


def overlay_tail_points(ax, position: float, values: np.ndarray, color: str, threshold: float | None = None) -> None:
    values = values[np.isfinite(values)]
    if values.size == 0:
        return
    if threshold is None:
        threshold = float(np.quantile(values, 0.995))
    tail = values[values >= threshold]
    if tail.size == 0:
        tail = np.array([float(values.max())])
    rng = np.random.default_rng(0)
    jitter = rng.uniform(-0.035, 0.035, size=tail.size)
    ax.scatter(
        np.full(tail.size, position) + jitter,
        tail,
        s=9,
        color=color,
        alpha=0.28,
        edgecolors="none",
        zorder=3,
    )


def annotate_peak(ax, x: float, y: float, text: str, color: str, dx: float = 0.12, dy: float = 0.8) -> None:
    ax.scatter([x], [y], s=12, color=color, zorder=4)
    ax.annotate(
        text,
        xy=(x, y),
        xytext=(x + dx, y + dy),
        ha="left",
        va="bottom",
        fontsize=6.4,
        color=color,
        arrowprops={"arrowstyle": "-", "lw": 0.75, "color": color},
    )


def add_regime_shading(ax, with_labels: bool = False) -> None:
    ax.axvspan(1.0, 2.0, color=REGION_GREEN, alpha=0.34, zorder=0)
    ax.axvspan(2.5, 3.0, color=REGION_RED, alpha=0.34, zorder=0)
    if not with_labels:
        return
    label_transform = blended_transform_factory(ax.transData, ax.transAxes)
    ax.text(
        1.5,
        0.96,
        "Stable Region",
        transform=label_transform,
        ha="center",
        va="top",
        fontsize=6.5,
        color="#355E3B",
        fontweight="semibold",
    )
    ax.text(
        2.75,
        0.96,
        "Unstable Region",
        transform=label_transform,
        ha="center",
        va="top",
        fontsize=6.5,
        color="#8B2C25",
        fontweight="semibold",
    )


def make_fig1_macro_overview(run_root: Path, out_dir: Path) -> None:
    collision, tracking = load_fig1_data(run_root)
    plot_methods = ("Frozen", "Stage", "Inertial")
    legend_labels = {
        "Frozen": "Frozen",
        "Stage": "Stage",
        "Inertial": "Inertial",
    }
    legacy_style = {
        "font.family": "sans-serif",
        "font.sans-serif": ["Helvetica", "Arial", "DejaVu Sans"],
        "mathtext.fontset": "dejavusans",
        "font.size": 11.8,
        "axes.titlesize": 13.5,
        "axes.labelsize": 11.3,
        "xtick.labelsize": 10.2,
        "ytick.labelsize": 10.2,
        "legend.fontsize": 9.5,
        "axes.linewidth": 1.05,
        "lines.linewidth": 1.8,
        "lines.markersize": 5.4,
    }

    with plt.rc_context(legacy_style):
        fig, axes = plt.subplots(2, 1, figsize=(8.1, 6.4), sharex=True, constrained_layout=False)

        for ax in axes:
            ax.grid(True, alpha=0.24, linestyle="--", linewidth=0.85)
            ax.spines["top"].set_visible(False)
            ax.spines["right"].set_visible(False)

        ax = axes[0]
        line_handles = []
        line_labels = []
        collision_marker_offset = {"Frozen": -0.035, "Stage": 0.0}
        collision_marker_size = {"Frozen": 6.0, "Stage": 6.0, "Inertial": 8.0}
        for method in plot_methods:
            method_df = collision[collision["method"] == method]
            style = STYLE[method]
            x = method_df["kp"].to_numpy(dtype=float)
            y = method_df["collision_rate"].to_numpy(dtype=float)
            line_width = 2.2 if method != "Inertial" else 2.0
            line_zorder = 5 if method == "Inertial" else (4 if method == "Stage" else 3)
            ax.plot(
                x,
                y,
                color=style["color"],
                linestyle=style["linestyle"],
                linewidth=line_width,
                solid_capstyle="round",
                dash_capstyle="round",
                zorder=line_zorder - 1,
            )
            marker_y = y[1:]
            if method == "Inertial":
                marker_x = 0.5 * (x[1:] + x[:-1])
            else:
                marker_x = x[1:] + collision_marker_offset[method]
            ax.plot(
                marker_x,
                marker_y,
                linestyle="None",
                marker=style["marker"],
                color=style["color"],
                markerfacecolor="white",
                markeredgewidth=1.2,
                markersize=collision_marker_size[method],
                zorder=line_zorder,
            )
            handle = Line2D(
                [0],
                [0],
                color=style["color"],
                linestyle=style["linestyle"],
                marker=style["marker"],
                markerfacecolor="white",
                markeredgewidth=1.2,
                linewidth=line_width,
                markersize=collision_marker_size[method],
            )
            line_handles.append(handle)
            line_labels.append(legend_labels[method])
        ax.set_ylabel("Collision Rate [%]")
        ax.set_title("(a) Collision Rate Over All 20 Seeds", pad=10)
        ax.yaxis.set_major_formatter(PercentFormatter(1.0, decimals=0))
        ax.set_ylim(0.0, 0.34)
        ax.set_yticks([0.0, 0.1, 0.2, 0.3])
        ax.tick_params(axis="x", which="both", labelbottom=False, bottom=False)
        fig.legend(
            line_handles,
            line_labels,
            loc="upper center",
            bbox_to_anchor=(0.5, 0.995),
            framealpha=0.94,
            fancybox=False,
            edgecolor="black",
            ncol=3,
            columnspacing=1.0,
            handlelength=2.2,
        )
        ax.annotate(
            "30% (6/20)",
            xy=(3.0, 0.30),
            xytext=(2.72, 0.245),
            fontsize=10.6,
            color=RED,
            arrowprops={"arrowstyle": "->", "linewidth": 1.0, "color": RED},
        )

        ax = axes[1]
        band_alpha = {"Inertial": 0.12, "Stage": 0.16, "Frozen": 0.16}
        for method in plot_methods:
            method_df = tracking[tracking["method"] == method]
            style = STYLE[method]
            x = method_df["kp"].to_numpy(dtype=float)
            mean_rms = method_df["mean_rms"].to_numpy(dtype=float)
            std_rms = method_df["std_rms"].to_numpy(dtype=float)
            lower = np.maximum(mean_rms - std_rms, 0.0)
            upper = mean_rms + std_rms
            ax.fill_between(
                x,
                lower,
                upper,
                color=style["color"],
                alpha=band_alpha[method],
                linewidth=0.0,
                zorder=1,
            )
            ax.plot(
                x,
                mean_rms,
                color=style["color"],
                linestyle=style["linestyle"],
                marker=style["marker"],
                markerfacecolor="white",
                markeredgewidth=1.2,
                linewidth=2.2 if method != "Inertial" else 2.0,
                solid_capstyle="round",
                dash_capstyle="round",
                zorder=5 if method == "Inertial" else (4 if method == "Stage" else 3),
            )
        ax.set_ylabel("Tracking RMS [m]")
        ax.set_title("(b) Tracking RMS On Common No-Collision Seeds", pad=10)
        ax.set_xlabel(r"Angular Velocity Gain $k_p$")
        ax.set_xticks(KP_ORDER)
        rms_low = float(np.nanmin(tracking["mean_rms"] - tracking["std_rms"]))
        rms_high = float(np.nanmax(tracking["mean_rms"] + tracking["std_rms"]))
        ax.set_ylim(max(0.0, rms_low - 0.05), rms_high + 0.05)

        fig.subplots_adjust(top=0.89, bottom=0.10, left=0.11, right=0.985, hspace=0.32)
        save_figure(fig, out_dir, "fig_c1_macro_trend")


def make_fig_crot_mean_gain(run_root: Path, out_dir: Path) -> None:
    df = load_crot_mean_gain_data(run_root)
    fig, ax = plt.subplots(1, 1, figsize=(3.55, 2.45), constrained_layout=False)

    ax.axvspan(1.0, 2.0, color=REGION_GREEN, alpha=0.14, zorder=0)
    ax.axvspan(2.5, 3.0, color=REGION_RED, alpha=0.14, zorder=0)
    for method in ("Inertial", "Stage", "Frozen"):
        method_df = df[df["method"] == method]
        if method == "Inertial":
            style = {"color": GRAY, "linestyle": ":", "marker": "^"}
        else:
            style = STYLE[method]
        ax.plot(
            method_df["kp"],
            method_df["mean_gain_vs_kp1"],
            color=style["color"],
            linestyle=style["linestyle"],
            marker=style["marker"],
            markerfacecolor="white",
            markeredgewidth=1.15,
            linewidth=1.8,
            solid_capstyle="round",
            dash_capstyle="round",
            zorder=3,
        )

    stylize_axes(ax, "both")
    ax.axhline(1.0, color=MUTED, linewidth=0.9, linestyle=(0, (3, 3)), alpha=0.9, zorder=2)
    ax.set_xlabel("$k_p$")
    ax.set_ylabel("Normalized mean $\\Vert c_{rot} \\Vert$")
    ax.set_xticks(KP_ORDER)
    ax.set_ylim(0.95, 2.38)
    label_transform = blended_transform_factory(ax.transData, ax.transAxes)
    ax.text(
        1.50,
        0.965,
        "$k_p \\in [1.0, 2.0]$",
        transform=label_transform,
        ha="center",
        va="top",
        fontsize=5.9,
        color=MUTED,
    )
    ax.annotate(
        "2.28x",
        xy=(3.0, float(df[(df["method"] == "Frozen") & (df["kp"] == 3.0)]["mean_gain_vs_kp1"].iloc[0])),
        xytext=(2.83, 2.24),
        ha="left",
        va="center",
        fontsize=6.2,
        color=RED,
        arrowprops={"arrowstyle": "-", "lw": 0.75, "color": RED},
    )
    ax.annotate(
        "Stage 1.39x",
        xy=(3.0, float(df[(df["method"] == "Stage") & (df["kp"] == 3.0)]["mean_gain_vs_kp1"].iloc[0])),
        xytext=(2.57, 1.47),
        ha="left",
        va="center",
        fontsize=6.0,
        color=BLUE,
        arrowprops={"arrowstyle": "-", "lw": 0.75, "color": BLUE},
    )
    ax.annotate(
        "Inertial 1.38x",
        xy=(3.0, float(df[(df["method"] == "Inertial") & (df["kp"] == 3.0)]["mean_gain_vs_kp1"].iloc[0])),
        xytext=(2.53, 1.31),
        ha="left",
        va="center",
        fontsize=5.9,
        color=GRAY,
        arrowprops={"arrowstyle": "-", "lw": 0.7, "color": GRAY},
    )
    fig.legend(
        handles=[
            Line2D([0], [0], color=GRAY, linestyle=":", marker="^", markerfacecolor="white", markeredgewidth=1.1, linewidth=1.6, label="Inertial"),
            Line2D([0], [0], color=BLUE, linestyle="-", marker="s", markerfacecolor="white", markeredgewidth=1.1, linewidth=1.8, label="Stage"),
            Line2D([0], [0], color=RED, linestyle=(0, (4.5, 2.2)), marker="o", markerfacecolor="white", markeredgewidth=1.1, linewidth=1.8, label="Frozen"),
        ],
        labels=["Inertial", "Stage", "Frozen"],
        loc="upper center",
        ncol=3,
        frameon=False,
        bbox_to_anchor=(0.5, 0.995),
        columnspacing=1.0,
        handlelength=2.2,
    )
    fig.subplots_adjust(top=0.83, bottom=0.15, left=0.18, right=0.98)
    save_figure(fig, out_dir, "fig_crot_mean_gain")


def make_fig2_mismatch_gap(run_root: Path, out_dir: Path) -> None:
    frozen_mean, stage_mean, frozen_p95, stage_p95 = load_fig2_data(run_root)
    legacy_style = {
        "font.family": "sans-serif",
        "font.sans-serif": ["Helvetica", "Arial", "DejaVu Sans"],
        "mathtext.fontset": "dejavusans",
        "font.size": 11.8,
        "axes.titlesize": 13.5,
        "axes.labelsize": 11.3,
        "xtick.labelsize": 10.2,
        "ytick.labelsize": 10.2,
        "legend.fontsize": 9.5,
        "axes.linewidth": 1.05,
        "lines.linewidth": 1.8,
        "lines.markersize": 5.4,
    }
    with plt.rc_context(legacy_style):
        fig, axes = plt.subplots(1, 2, figsize=(8.1, 3.0), constrained_layout=False)
        plot_gap_dumbbell(
            axes[0],
            frozen_mean,
            stage_mean,
            "Mismatch RMS [m/s$^2$]",
            "a",
            "Mean $f_{rot}$ Mismatch",
        )
        plot_gap_dumbbell(
            axes[1],
            frozen_p95,
            stage_p95,
            "P99 Mismatch RMS [m/s$^2$]",
            "b",
            "P99 $f_{rot}$ Mismatch",
        )
        add_gap_subplot_note(axes[0])
        add_gap_subplot_note(axes[1])
        add_method_legend(fig, ["Stage", "Frozen"], anchor_y=1.03, marker_only=True, framed=True)
        fig.subplots_adjust(top=0.72, bottom=0.18, left=0.08, right=0.985, wspace=0.18)
        save_figure(fig, out_dir, "fig_c2_mismatch_mechanism")


def make_fig_c2_alignment_robustness(run_root: Path, out_dir: Path) -> None:
    repo_root = Path(__file__).resolve().parents[1]
    staging_dir = out_dir / "_fig_c2_alignment_legacy_source"
    staging_dir.mkdir(parents=True, exist_ok=True)
    merged_csv = build_c2_alignment_legacy_source(run_root, staging_dir, kp=ALIGN_KP)
    legacy_plot_path = staging_dir / "fig_c2_alignment_robustness.png"
    cmd = [
        "python3",
        str(repo_root / "scripts" / "plot_stagewise_robustness.py"),
        "--csv",
        str(merged_csv),
        "--out-plot",
        str(legacy_plot_path),
        "--accel-thr",
        str(ALIGN_ACCEL_THR),
    ]
    subprocess.run(cmd, check=True)

    out_dir.mkdir(parents=True, exist_ok=True)
    artifact_map = {
        legacy_plot_path.with_suffix(".png"): out_dir / "fig_c2_alignment_robustness.png",
        legacy_plot_path.with_suffix(".pdf"): out_dir / "fig_c2_alignment_robustness.pdf",
        staging_dir / "robustness_table.md": out_dir / "robustness_table.md",
        staging_dir / "c2_summary_table.md": out_dir / "c2_summary_table.md",
        staging_dir / "c2_per_seed_metrics.csv": out_dir / "c2_per_seed_metrics.csv",
    }
    for src, dst in artifact_map.items():
        if src.exists():
            shutil.copy2(src, dst)
            print(f"[INFO] wrote {dst}")


def make_fig3_micro_case(run_root: Path, out_dir: Path) -> None:
    case = load_fig3_case(run_root)
    fig, axes = plt.subplots(3, 1, figsize=(6.8, 4.9), sharex=True, constrained_layout=False)

    ax = axes[0]
    add_difficult_segment(ax)
    ax.plot(
        case.omega_truth["sim_time"],
        case.omega_truth["true_omega_z"],
        color=STYLE["True"]["color"],
        linestyle=STYLE["True"]["linestyle"],
        linewidth=1.6,
    )
    panel_title(ax, "a", "True $\\omega_g^N$")
    stylize_axes(ax, "y")
    ax.set_ylabel("$\\omega_{g,z}^N$ [rad/s]")
    ax.text(
        0.48,
        0.92,
        "Difficult segment",
        transform=ax.transAxes,
        ha="center",
        va="top",
        fontsize=6.3,
        color=MUTED,
    )
    span = ax.get_ylim()[1] - ax.get_ylim()[0]
    ax.annotate(
        "sign reversal",
        xy=(REVERSAL_TIME, ax.get_ylim()[1] - 0.12 * span),
        xytext=(REVERSAL_TIME + 0.10, ax.get_ylim()[1] - 0.04 * span),
        ha="left",
        va="top",
        fontsize=6.2,
        color=MUTED,
        arrowprops={"arrowstyle": "-", "lw": 0.7, "color": MUTED},
    )

    ax = axes[1]
    add_difficult_segment(ax)
    ax.plot(
        case.frot_series["sim_time"],
        case.frot_series["true_frot"],
        color="#1F4E79",
        linestyle="-",
        linewidth=1.55,
        label="True",
    )
    ax.plot(
        case.frot_series["sim_time"],
        case.frot_series["stage_pred"],
        color=BLUE,
        linestyle="--",
        linewidth=1.45,
        label="Stage predicted",
    )
    ax.plot(
        case.frot_series["sim_time"],
        case.frot_series["frozen_pred"],
        color=RED,
        linestyle="--",
        linewidth=1.45,
        label="Frozen predicted",
    )
    panel_title(ax, "b", "$f_{rot}$: true vs. predicted")
    stylize_axes(ax, "y")
    ax.set_ylabel("$f_{rot}$ [m/s$^2$]")

    ax = axes[2]
    add_difficult_segment(ax)
    ax.plot(
        case.tracking_series["sim_time"],
        case.tracking_series["stage_tracking_error"],
        color=BLUE,
        linestyle="-",
        linewidth=1.5,
        label="Stage",
    )
    ax.plot(
        case.tracking_series["sim_time"],
        case.tracking_series["frozen_tracking_error"],
        color=RED,
        linestyle="--",
        linewidth=1.5,
        label="Frozen",
    )
    panel_title(ax, "c", "Local tracking error")
    stylize_axes(ax, "y")
    ax.set_ylabel("Tracking error [m]")
    ax.set_xlabel("Time [s]")
    ax.set_xlim(*CASE_WINDOW)

    handles = [
        Line2D([0], [0], color="#1F4E79", linestyle="-", linewidth=1.55, label="True"),
        Line2D([0], [0], color=BLUE, linestyle="--", linewidth=1.45, label="Stage"),
        Line2D([0], [0], color=RED, linestyle="--", linewidth=1.45, label="Frozen"),
    ]
    fig.legend(
        handles=handles,
        labels=["True", "Stage", "Frozen"],
        loc="upper center",
        ncol=3,
        frameon=False,
        bbox_to_anchor=(0.5, 0.988),
        handlelength=2.0,
        columnspacing=1.2,
    )

    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.92), h_pad=0.45)
    save_figure(fig, out_dir, "fig_c3_jerk_reversal_case")


def make_fig4_computational_collapse(run_root: Path, out_dir: Path) -> None:
    prep_by_kp, solve_kp3, total_kp3 = load_fig4_data(run_root)
    fig, axes = plt.subplots(1, 3, figsize=(6.9, 2.75), constrained_layout=False, gridspec_kw={"width_ratios": [1.45, 0.95, 0.95]})

    ax = axes[0]
    base_positions = np.arange(len(KP_ORDER)) + 1
    offsets = {"Non-inertial Frozen": -0.16, "Non-inertial Stage": 0.16}
    for method in ("Non-inertial Frozen", "Non-inertial Stage"):
        positions = [pos + offsets[method] for pos in base_positions]
        data = [prep_by_kp[method][kp] for kp in KP_ORDER]
        add_violin(ax, data, positions, STYLE[method]["color"])
    panel_title(ax, "a", "Preparation time $t_{prep}$")
    stylize_axes(ax, "y")
    ax.set_ylabel("$t_{prep}$ [ms]")
    ax.set_xlabel("$k_p$")
    ax.set_xticks(base_positions)
    ax.set_xticklabels([f"{kp:.1f}" for kp in KP_ORDER])
    ax.text(
        0.02,
        0.96,
        "Common no-collision subset",
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=6.3,
        color=MUTED,
        style="italic",
    )

    ax = axes[1]
    positions = [1, 2]
    add_violin(ax, [solve_kp3["Non-inertial Frozen"]], [1], RED)
    add_violin(ax, [solve_kp3["Non-inertial Stage"]], [2], BLUE)
    overlay_tail_points(ax, 1, solve_kp3["Non-inertial Frozen"], RED, threshold=20.0)
    overlay_tail_points(ax, 2, solve_kp3["Non-inertial Stage"], BLUE, threshold=20.0)
    panel_title(ax, "b", "Avoid-phase $t_{solve}$")
    stylize_axes(ax, "y")
    ax.set_ylabel("$t_{solve}$ [ms]")
    ax.set_xticks(positions)
    ax.set_xticklabels(["Frozen", "Stage"])
    ax.set_ylim(0.0, 43.5)
    ax.axhline(DEADLINE_MS, color=DEADLINE_RED, linewidth=1.35, linestyle=(0, (5, 3)))
    ax.text(1.86, DEADLINE_MS + 0.6, "20 ms", color=DEADLINE_RED, fontsize=6.3, ha="right", va="bottom")
    solve_frozen_max = float(np.max(solve_kp3["Non-inertial Frozen"])) if solve_kp3["Non-inertial Frozen"].size else float("nan")
    annotate_peak(ax, 1.0, solve_frozen_max, f"{solve_frozen_max:.1f} ms", RED, dx=0.12, dy=0.7)
    ax.text(0.98, 0.96, "$k_p=3.0$", transform=ax.transAxes, ha="right", va="top", fontsize=7, color=MUTED)

    ax = axes[2]
    add_violin(ax, [total_kp3["Non-inertial Frozen"]], [1], RED)
    add_violin(ax, [total_kp3["Non-inertial Stage"]], [2], BLUE)
    overlay_tail_points(ax, 1, total_kp3["Non-inertial Frozen"], RED, threshold=20.0)
    overlay_tail_points(ax, 2, total_kp3["Non-inertial Stage"], BLUE, threshold=20.0)
    panel_title(ax, "c", "Avoid-phase $t_{total}$")
    stylize_axes(ax, "y")
    ax.set_ylabel("$t_{total}$ [ms]")
    ax.set_xticks(positions)
    ax.set_xticklabels(["Frozen", "Stage"])
    ax.set_ylim(0.0, 43.5)
    ax.axhline(DEADLINE_MS, color=DEADLINE_RED, linewidth=1.35, linestyle=(0, (5, 3)))
    total_frozen_max = float(np.max(total_kp3["Non-inertial Frozen"])) if total_kp3["Non-inertial Frozen"].size else float("nan")
    ax.text(1.86, DEADLINE_MS + 0.6, "20 ms", color=DEADLINE_RED, fontsize=6.3, ha="right", va="bottom")
    annotate_peak(ax, 1.0, total_frozen_max, f"{total_frozen_max:.1f} ms", RED, dx=0.12, dy=0.7)
    ax.text(0.98, 0.96, "$k_p=3.0$", transform=ax.transAxes, ha="right", va="top", fontsize=7, color=MUTED)

    handles = [
        Patch(facecolor=RED, edgecolor=RED, alpha=0.28, label="Frozen"),
        Patch(facecolor=BLUE, edgecolor=BLUE, alpha=0.28, label="Stage"),
    ]
    fig.legend(
        handles=handles,
        labels=["Frozen", "Stage"],
        loc="upper center",
        ncol=2,
        frameon=False,
        bbox_to_anchor=(0.5, 0.982),
        columnspacing=1.1,
    )

    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.90), w_pad=0.6)
    save_figure(fig, out_dir, "fig_c4_computational_collapse")


def make_fig5_constraint_feedback(run_root: Path, out_dir: Path) -> None:
    slack, solver, min_h = load_fig5_data(run_root)
    fig, axes = plt.subplots(1, 3, figsize=(10.6, 3.15), constrained_layout=False)

    legend_handles = plot_grouped_dot(
        axes[0],
        solver,
        "solver_tracking_rms_avoid [m]",
        "a",
        "solver_tracking_rms_avoid vs. $k_p$",
    )
    axes[0].text(
        0.58,
        0.92,
        "Frozen avoid cost\ninflates",
        transform=axes[0].transAxes,
        ha="left",
        va="top",
        fontsize=6.8,
        color=RED,
    )

    plot_grouped_dot(axes[1], min_h, "min_h", "b", "min_h vs. $k_p$")
    axes[1].text(
        0.54,
        0.09,
        "Less negative is better",
        transform=axes[1].transAxes,
        ha="left",
        va="bottom",
        fontsize=6.7,
        color=MUTED,
    )
    min_h_values = np.array([value for series in min_h.values() for value in series], dtype=float)
    if np.isfinite(min_h_values).any():
        axes[1].set_ylim(float(np.nanmin(min_h_values) - 0.006), float(np.nanmax(min_h_values) + 0.006))

    plot_grouped_dot(axes[2], slack, "slack_sum_effective", "c", "slack_sum_effective vs. $k_p$")
    axes[2].text(
        0.50,
        0.92,
        "Frozen slack gap\nopens up",
        transform=axes[2].transAxes,
        ha="left",
        va="top",
        fontsize=6.8,
        color=RED,
    )

    fig.legend(
        handles=[
            legend_handles["Inertial"],
            legend_handles["Stage"],
            legend_handles["Frozen"],
        ],
        labels=["Inertial", "Stage", "Frozen"],
        loc="upper center",
        ncol=3,
        frameon=False,
        bbox_to_anchor=(0.5, 0.995),
        columnspacing=1.1,
        handlelength=2.2,
    )

    fig.subplots_adjust(top=0.80, bottom=0.18, left=0.075, right=0.99, wspace=0.28)
    save_figure(fig, out_dir, "fig_c5_constraint_feedback")


def main() -> None:
    args = parse_args()
    configure_style()
    run_root = args.run_root.resolve()
    out_dir = args.out_dir.resolve()

    make_fig1_macro_overview(run_root, out_dir)
    make_fig2_mismatch_gap(run_root, out_dir)
    make_fig_c2_alignment_robustness(run_root, out_dir)
    make_fig5_constraint_feedback(run_root, out_dir)


if __name__ == "__main__":
    main()
