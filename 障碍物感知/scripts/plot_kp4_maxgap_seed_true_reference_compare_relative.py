#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

import math

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parent.parent
RUN_ROOT = ROOT / "results" / "figure_eight_clean_nocbf_kp124_seed1_100_2ctrl" / "logs"
OUT_PREFIX = ROOT / "figures" / "kp4_maxgap_seed_true_reference_compare_relative"
V_KEY = "v_2"
W_KEY = "w_2"
STAGE_CTRL = "noninertial_stage"
FROZEN_CTRL = "noninertial_frozen"
COLORS = {
    "reference": "#111111",
    "frozen": "#1f77b4",
    "stage": "#d62728",
}


def load_scope_all(metrics_csv: Path) -> pd.Series:
    df = pd.read_csv(metrics_csv)
    row = df[df["scope"] == "all"]
    if row.empty:
        raise RuntimeError(f"No scope=all row in {metrics_csv}")
    return row.iloc[0]


def select_max_gap_seed() -> tuple[int, pd.Series, pd.Series]:
    base = RUN_ROOT / FROZEN_CTRL / V_KEY / W_KEY
    best_seed = None
    best_gap = -math.inf
    best_frozen = None
    best_stage = None
    for seed_dir in sorted(base.glob("seed_*")):
        seed = int(seed_dir.name.split("_")[1])
        frozen_row = load_scope_all(seed_dir / "metrics.csv")
        stage_row = load_scope_all(
            RUN_ROOT / STAGE_CTRL / V_KEY / W_KEY / seed_dir.name / "metrics.csv"
        )
        frozen_rms = float(pd.to_numeric(frozen_row["tracking_rms"], errors="coerce"))
        stage_rms = float(pd.to_numeric(stage_row["tracking_rms"], errors="coerce"))
        gap = frozen_rms - stage_rms
        if math.isfinite(gap) and gap > best_gap:
            best_seed = seed
            best_gap = gap
            best_frozen = frozen_row
            best_stage = stage_row
    if best_seed is None or best_frozen is None or best_stage is None:
        raise RuntimeError("Failed to find a valid KP=4 seed.")
    return best_seed, best_frozen, best_stage


def load_steps(ctrl: str, seed: int) -> pd.DataFrame:
    path = RUN_ROOT / ctrl / V_KEY / W_KEY / f"seed_{seed:03d}" / "metrics_steps.csv"
    return pd.read_csv(path)


def select_max_gap_uav(seed: int) -> tuple[int, float, float]:
    frozen = load_steps(FROZEN_CTRL, seed)
    stage = load_steps(STAGE_CTRL, seed)
    best_uav = None
    best_gap = -math.inf
    best_frozen = math.nan
    best_stage = math.nan
    for uav_idx in sorted(int(v) for v in frozen["uav_idx"].dropna().unique()):
        f_uav = pd.to_numeric(
            frozen.loc[frozen["uav_idx"] == uav_idx, "tracking_error"], errors="coerce"
        ).dropna()
        s_uav = pd.to_numeric(
            stage.loc[stage["uav_idx"] == uav_idx, "tracking_error"], errors="coerce"
        ).dropna()
        if f_uav.empty or s_uav.empty:
            continue
        frozen_rms = float(np.sqrt(np.mean(np.square(f_uav.to_numpy(dtype=float)))))
        stage_rms = float(np.sqrt(np.mean(np.square(s_uav.to_numpy(dtype=float)))))
        gap = frozen_rms - stage_rms
        if math.isfinite(gap) and gap > best_gap:
            best_uav = uav_idx
            best_gap = gap
            best_frozen = frozen_rms
            best_stage = stage_rms
    if best_uav is None:
        raise RuntimeError(f"Failed to find a valid UAV for seed {seed}.")
    return best_uav, best_frozen, best_stage


def load_uav_steps(ctrl: str, seed: int, uav_idx: int) -> pd.DataFrame:
    df = load_steps(ctrl, seed)
    df = df[df["uav_idx"] == uav_idx].copy()
    return df.sort_values("step_idx").reset_index(drop=True)


def load_ugv(seed: int) -> pd.DataFrame:
    path = RUN_ROOT / STAGE_CTRL / V_KEY / W_KEY / f"seed_{seed:03d}" / "metrics_ugv_horizon_steps.csv"
    df = pd.read_csv(path)
    df = df[df["horizon_idx"] == 0].copy()
    return df.sort_values("step_idx").reset_index(drop=True)


def default_offset_for_uav(uav_idx: int, radius: float, z_ref: float) -> np.ndarray:
    canonical = {
        0: np.array([-radius, 0.0, z_ref], dtype=float),
        1: np.array([0.0, radius, z_ref], dtype=float),
        2: np.array([0.0, -radius, z_ref], dtype=float),
        3: np.array([radius, 0.0, z_ref], dtype=float),
    }
    if uav_idx in canonical:
        return canonical[uav_idx]
    raise RuntimeError(
        f"Unsupported uav_idx={uav_idx} for default offset reconstruction."
    )


def build_reference_relative(
    ugv: pd.DataFrame, seed_stage_row: pd.Series, uav_idx: int, sample_count: int
) -> pd.DataFrame:
    formation_radius = float(pd.to_numeric(seed_stage_row["requested_r"], errors="coerce"))
    start_z = float(pd.to_numeric(seed_stage_row["start_z"], errors="coerce"))
    car_start_z = float(pd.to_numeric(ugv.iloc[0]["actual_position_z"], errors="coerce"))
    rel_offset = default_offset_for_uav(uav_idx, formation_radius, start_z - car_start_z)
    return pd.DataFrame(
        {
            "sim_time": ugv["sim_time"].iloc[:sample_count].to_numpy(dtype=float),
            "rel_x": np.full(sample_count, rel_offset[0], dtype=float),
            "rel_y": np.full(sample_count, rel_offset[1], dtype=float),
            "rel_z": np.full(sample_count, rel_offset[2], dtype=float),
        }
    )


def build_actual_relative(ctrl_df: pd.DataFrame) -> pd.DataFrame:
    return pd.DataFrame(
        {
            "sim_time": ctrl_df["sim_time"].to_numpy(dtype=float),
            "rel_x": pd.to_numeric(ctrl_df["position_x"], errors="coerce").to_numpy(dtype=float),
            "rel_y": pd.to_numeric(ctrl_df["position_y"], errors="coerce").to_numpy(dtype=float),
            "rel_z": pd.to_numeric(ctrl_df["position_z"], errors="coerce").to_numpy(dtype=float),
            "tracking_error": pd.to_numeric(ctrl_df["tracking_error"], errors="coerce").to_numpy(dtype=float),
        }
    )


def to_tracking_error_frame(actual_r: pd.DataFrame, reference_r: pd.DataFrame) -> pd.DataFrame:
    out = actual_r.copy().reset_index(drop=True)
    ref = reference_r.iloc[: len(out)].reset_index(drop=True)
    out["err_x"] = (
        pd.to_numeric(out["rel_x"], errors="coerce").to_numpy(dtype=float)
        - pd.to_numeric(ref["rel_x"], errors="coerce").to_numpy(dtype=float)
    )
    out["err_y"] = (
        pd.to_numeric(out["rel_y"], errors="coerce").to_numpy(dtype=float)
        - pd.to_numeric(ref["rel_y"], errors="coerce").to_numpy(dtype=float)
    )
    out["err_z"] = (
        pd.to_numeric(out["rel_z"], errors="coerce").to_numpy(dtype=float)
        - pd.to_numeric(ref["rel_z"], errors="coerce").to_numpy(dtype=float)
    )
    return out


def main() -> None:
    plt.rcParams.update(
        {
            "font.size": 10,
        }
    )

    seed, frozen_seed_row, stage_seed_row = select_max_gap_seed()
    uav_idx, frozen_uav_rms, stage_uav_rms = select_max_gap_uav(seed)

    frozen = load_uav_steps(FROZEN_CTRL, seed, uav_idx)
    stage = load_uav_steps(STAGE_CTRL, seed, uav_idx)
    ugv = load_ugv(seed)

    sample_count = min(len(frozen), len(stage), len(ugv))
    frozen = frozen.iloc[:sample_count].reset_index(drop=True)
    stage = stage.iloc[:sample_count].reset_index(drop=True)
    ugv = ugv.iloc[:sample_count].reset_index(drop=True)

    reference_r = build_reference_relative(ugv, stage_seed_row, uav_idx, sample_count)
    frozen_r = build_actual_relative(frozen)
    stage_r = build_actual_relative(stage)
    frozen_e = to_tracking_error_frame(frozen_r, reference_r)
    stage_e = to_tracking_error_frame(stage_r, reference_r)
    fig, ax = plt.subplots(figsize=(5.6, 5.2), constrained_layout=True)
    ax.scatter(
        [0.0],
        [0.0],
        color=COLORS["reference"],
        marker="x",
        s=90,
        linewidths=2.2,
        zorder=6,
    )
    ax.plot(
        frozen_e["err_x"],
        frozen_e["err_y"],
        color=COLORS["frozen"],
        lw=1.8,
    )
    ax.plot(
        stage_e["err_x"],
        stage_e["err_y"],
        color=COLORS["stage"],
        lw=1.8,
    )
    ax.set_aspect("equal")
    ax.axhline(0.0, color="0.65", lw=0.8, zorder=1)
    ax.axvline(0.0, color="0.65", lw=0.8, zorder=1)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    rows = []
    for label, df in (
        ("reference", reference_r),
        ("frozen", frozen_e),
        ("stage", stage_e),
    ):
        tmp = df.copy()
        tmp["series"] = label
        rows.append(tmp)

    out_prefix = OUT_PREFIX.with_name(
        f"{OUT_PREFIX.name}_seed{seed:03d}_uav{uav_idx}"
    )
    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    pd.concat(rows, ignore_index=True).to_csv(out_prefix.with_suffix(".csv"), index=False)
    fig.savefig(out_prefix.with_suffix(".png"), dpi=300, bbox_inches="tight")
    fig.savefig(out_prefix.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)

    print(f"output_png={out_prefix.with_suffix('.png')}")
    print(f"output_pdf={out_prefix.with_suffix('.pdf')}")


if __name__ == "__main__":
    main()
