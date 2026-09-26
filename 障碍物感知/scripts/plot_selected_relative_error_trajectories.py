#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Rectangle


ROOT = Path(__file__).resolve().parent.parent
RESULTS_ROOT = ROOT / "results"
FIG_ROOT = ROOT / "figures"
BASE_V = 0.5
COLORS = {
    "reference": "#111111",
    "frozen": "#1f77b4",
    "stage": "#d62728",
}

SCENARIOS = [
    {
        "scenario": "circle",
        "name_fmt": "circle_kp{kp}_min_disadv",
        "run_dirs": {
            1: "circle_nocbf_kp124_seed1_100_2ctrl",
            2: "circle_nocbf_kp124_seed1_100_2ctrl",
            3: "circle_nocbf_kp3_seed1_100_2ctrl",
            4: "circle_nocbf_kp124_seed1_100_2ctrl",
            6: "circle_nocbf_kp6_seed1_100_2ctrl",
        },
    },
    {
        "scenario": "figure8_clean",
        "name_fmt": "figure8_clean_kp{kp}_max_adv",
        "run_dirs": {
            1: "figure_eight_clean_nocbf_kp124_seed1_100_2ctrl",
            2: "figure_eight_clean_nocbf_kp124_seed1_100_2ctrl",
            3: "figure_eight_clean_nocbf_kp3_seed1_100_2ctrl",
            4: "figure_eight_clean_nocbf_kp124_seed1_100_2ctrl",
            6: "figure_eight_clean_nocbf_kp6_seed1_100_2ctrl",
        },
    },
    {
        "scenario": "figure8_obs",
        "name_fmt": "figure8_obs_kp{kp}_max_adv",
        "run_dirs": {
            1: "figure_eight_fixedobs_cbf_kp124_seed1_100_2ctrl",
            2: "figure_eight_fixedobs_cbf_kp124_seed1_100_2ctrl",
            3: "figure_eight_fixedobs_cbf_kp3_seed1_100_2ctrl",
            4: "figure_eight_fixedobs_cbf_kp124_seed1_100_2ctrl",
            6: "figure_eight_fixedobs_cbf_kp6_seed1_100_2ctrl",
        },
    },
]
KP_VALUES = [1, 2, 3, 4, 6]
ZOOM_INSET_CASES = {
    "circle_kp1_min_disadv",
    "circle_kp2_min_disadv",
    "circle_kp3_min_disadv",
    "circle_kp4_min_disadv",
    "figure8_clean_kp1_max_adv",
    "figure8_clean_kp2_max_adv",
    "figure8_clean_kp3_max_adv",
    "figure8_clean_kp4_max_adv",
    "figure8_clean_kp6_max_adv",
    "figure8_obs_kp1_max_adv",
    "figure8_obs_kp2_max_adv",
    "figure8_obs_kp3_max_adv",
    "figure8_obs_kp4_max_adv",
}


def v_from_kp(kp: int) -> float:
    return BASE_V * float(kp)


def q_label(value: float) -> str:
    return f"{float(value):.2f}".rstrip("0").rstrip(".").replace(".", "p")


def metric_csv_path(run_root: Path, ctrl: str, v_key: str, w_key: str, seed: int) -> Path:
    return run_root / "logs" / ctrl / v_key / w_key / f"seed_{seed:03d}" / "metrics.csv"


def load_scope_all(metrics_csv: Path) -> pd.Series:
    df = pd.read_csv(metrics_csv)
    row = df[df["scope"] == "all"]
    if row.empty:
        raise RuntimeError(f"No scope=all row in {metrics_csv}")
    return row.iloc[0]


def load_scope_uavs(metrics_csv: Path) -> pd.DataFrame:
    df = pd.read_csv(metrics_csv)
    df = df[df["scope"].astype(str).str.startswith("uav")].copy()
    if df.empty:
        raise RuntimeError(f"No per-UAV rows in {metrics_csv}")
    df["tracking_rms"] = pd.to_numeric(df["tracking_rms"], errors="coerce")
    return df[["scope", "tracking_rms"]].dropna(subset=["tracking_rms"]).reset_index(drop=True)


def load_uav_steps(run_root: Path, ctrl: str, v_key: str, w_key: str, seed: int, uav_idx: int) -> pd.DataFrame:
    path = run_root / "logs" / ctrl / v_key / w_key / f"seed_{seed:03d}" / "metrics_steps.csv"
    df = pd.read_csv(path)
    df = df[df["uav_idx"] == uav_idx].copy()
    return df.sort_values("step_idx").reset_index(drop=True)


def load_ugv(run_root: Path, v_key: str, w_key: str, seed: int) -> pd.DataFrame:
    path = run_root / "logs" / "noninertial_stage" / v_key / w_key / f"seed_{seed:03d}" / "metrics_ugv_horizon_steps.csv"
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
    raise RuntimeError(f"Unsupported uav_idx={uav_idx} for default offset reconstruction.")


def build_reference_relative(ugv: pd.DataFrame, metrics_row: pd.Series, uav_idx: int, sample_count: int) -> pd.DataFrame:
    formation_radius = float(pd.to_numeric(metrics_row["requested_r"], errors="coerce"))
    start_z = float(pd.to_numeric(metrics_row["start_z"], errors="coerce"))
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


def select_case(scenario_spec: dict, kp: int) -> dict:
    result_dir = scenario_spec["run_dirs"][kp]
    run_root = RESULTS_ROOT / result_dir
    per_seed_path = run_root / "tables" / "per_seed_controller_metrics.csv"
    per_seed = pd.read_csv(per_seed_path)
    v = v_from_kp(kp)
    per_seed["v"] = pd.to_numeric(per_seed["v"], errors="coerce")
    per_seed["w"] = pd.to_numeric(per_seed["w"], errors="coerce")
    sub = per_seed[
        per_seed["controller"].isin(["noninertial_frozen", "noninertial_stage"])
        & np.isclose(per_seed["v"], v)
        & np.isclose(per_seed["w"], v)
    ].copy()
    bad = sub[~sub["status"].isin(["ok", "skipped"])].copy()
    if not bad.empty:
        preview = ", ".join(
            f"{row.controller} seed={int(row.seed)} status={row.status}"
            for row in bad.head(8).itertuples()
        )
        raise RuntimeError(
            f"Incomplete trials detected for scenario={scenario_spec['scenario']} kp={kp}; "
            f"repair or rerun before selecting representative trajectories: {preview}"
        )
    pivot = sub.pivot(index="seed", columns="controller", values="tracking_rms").dropna()
    if pivot.empty:
        raise RuntimeError(f"No paired seed rows for scenario={scenario_spec['scenario']} kp={kp}")
    pivot["team_gap"] = pivot["noninertial_frozen"] - pivot["noninertial_stage"]
    best_seed = int(pivot["team_gap"].idxmax())
    team_gap = float(pivot.loc[best_seed, "team_gap"])

    v_key = f"v_{q_label(v)}"
    w_key = f"w_{q_label(v)}"
    frozen_metrics = load_scope_uavs(metric_csv_path(run_root, "noninertial_frozen", v_key, w_key, best_seed))
    stage_metrics = load_scope_uavs(metric_csv_path(run_root, "noninertial_stage", v_key, w_key, best_seed))
    uav_pivot = (
        frozen_metrics.rename(columns={"tracking_rms": "frozen_tracking_rms"})
        .merge(stage_metrics.rename(columns={"tracking_rms": "stage_tracking_rms"}), on="scope", how="inner")
    )
    if uav_pivot.empty:
        raise RuntimeError(
            f"No paired UAV rows for scenario={scenario_spec['scenario']} kp={kp} seed={best_seed}"
        )
    uav_pivot["uav_gap"] = uav_pivot["frozen_tracking_rms"] - uav_pivot["stage_tracking_rms"]
    best_scope = str(uav_pivot.loc[uav_pivot["uav_gap"].idxmax(), "scope"])
    uav_idx = int(best_scope.replace("uav", ""))
    uav_gap = float(uav_pivot.loc[uav_pivot["uav_gap"].idxmax(), "uav_gap"])

    return {
        "name": scenario_spec["name_fmt"].format(kp=kp),
        "scenario": scenario_spec["scenario"],
        "result_dir": result_dir,
        "run_root": run_root,
        "kp": kp,
        "v_key": v_key,
        "w_key": w_key,
        "seed": best_seed,
        "uav_idx": uav_idx,
        "team_gap": team_gap,
        "uav_gap": uav_gap,
    }


def load_case_data(case: dict) -> dict:
    run_root = case["run_root"]
    seed = int(case["seed"])
    uav_idx = int(case["uav_idx"])
    v_key = case["v_key"]
    w_key = case["w_key"]

    frozen = load_uav_steps(run_root, "noninertial_frozen", v_key, w_key, seed, uav_idx)
    stage = load_uav_steps(run_root, "noninertial_stage", v_key, w_key, seed, uav_idx)
    ugv = load_ugv(run_root, v_key, w_key, seed)

    sample_count = min(len(frozen), len(stage), len(ugv))
    frozen = frozen.iloc[:sample_count].reset_index(drop=True)
    stage = stage.iloc[:sample_count].reset_index(drop=True)
    ugv = ugv.iloc[:sample_count].reset_index(drop=True)

    metrics_row = load_scope_all(
        run_root / "logs" / "noninertial_stage" / v_key / w_key / f"seed_{seed:03d}" / "metrics.csv"
    )
    reference_r = build_reference_relative(ugv, metrics_row, uav_idx, sample_count)
    frozen_e = to_tracking_error_frame(build_actual_relative(frozen), reference_r)
    stage_e = to_tracking_error_frame(build_actual_relative(stage), reference_r)

    return {
        **case,
        "reference_r": reference_r,
        "frozen_e": frozen_e,
        "stage_e": stage_e,
    }


def compute_scenario_limits(cases_data: list[dict]) -> dict[str, tuple[float, float, float, float]]:
    limits: dict[str, tuple[float, float, float, float]] = {}
    scenario_names = sorted({case_data["scenario"] for case_data in cases_data})
    for scenario in scenario_names:
        xs = [0.0]
        ys = [0.0]
        for case_data in cases_data:
            if case_data["scenario"] != scenario:
                continue
            xs.extend(pd.to_numeric(case_data["frozen_e"]["err_x"], errors="coerce").tolist())
            xs.extend(pd.to_numeric(case_data["stage_e"]["err_x"], errors="coerce").tolist())
            ys.extend(pd.to_numeric(case_data["frozen_e"]["err_y"], errors="coerce").tolist())
            ys.extend(pd.to_numeric(case_data["stage_e"]["err_y"], errors="coerce").tolist())

        x_abs = float(np.nanmax(np.abs(np.asarray(xs, dtype=float))))
        y_abs = float(np.nanmax(np.abs(np.asarray(ys, dtype=float))))
        span = max(x_abs, y_abs)
        margin = max(0.02, span * 0.08)
        bound = span + margin
        limits[scenario] = (-bound, bound, -bound, bound)
    return limits


def compute_zoom_limits(frozen_e: pd.DataFrame, stage_e: pd.DataFrame) -> tuple[float, float, float, float]:
    xs = np.concatenate(
        [
            pd.to_numeric(frozen_e["err_x"], errors="coerce").to_numpy(dtype=float),
            pd.to_numeric(stage_e["err_x"], errors="coerce").to_numpy(dtype=float),
            np.asarray([0.0], dtype=float),
        ]
    )
    ys = np.concatenate(
        [
            pd.to_numeric(frozen_e["err_y"], errors="coerce").to_numpy(dtype=float),
            pd.to_numeric(stage_e["err_y"], errors="coerce").to_numpy(dtype=float),
            np.asarray([0.0], dtype=float),
        ]
    )
    x_min = float(np.nanmin(xs))
    x_max = float(np.nanmax(xs))
    y_min = float(np.nanmin(ys))
    y_max = float(np.nanmax(ys))
    center_x = 0.5 * (x_min + x_max)
    center_y = 0.5 * (y_min + y_max)
    span = max(x_max - x_min, y_max - y_min, 0.02)
    half = 0.5 * span + max(0.01, 0.14 * span)
    return (
        center_x - half,
        center_x + half,
        center_y - half,
        center_y + half,
    )


def add_zoom_inset(
    ax: plt.Axes,
    frozen_e: pd.DataFrame,
    stage_e: pd.DataFrame,
    zoom_limits: tuple[float, float, float, float],
) -> None:
    x0, x1, y0, y1 = zoom_limits
    ax.add_patch(
        Rectangle(
            (x0, y0),
            x1 - x0,
            y1 - y0,
            fill=False,
            edgecolor="0.35",
            linewidth=1.0,
            linestyle="--",
            zorder=4,
        )
    )

    inset_ax = ax.inset_axes([0.57, 0.08, 0.35, 0.35])
    inset_ax.scatter(
        [0.0],
        [0.0],
        color=COLORS["reference"],
        marker="x",
        s=55,
        linewidths=1.8,
        zorder=6,
    )
    inset_ax.plot(frozen_e["err_x"], frozen_e["err_y"], color=COLORS["frozen"], lw=1.5)
    inset_ax.plot(stage_e["err_x"], stage_e["err_y"], color=COLORS["stage"], lw=1.5)
    inset_ax.axhline(0.0, color="0.75", lw=0.6, zorder=1)
    inset_ax.axvline(0.0, color="0.75", lw=0.6, zorder=1)
    inset_ax.set_xlim(x0, x1)
    inset_ax.set_ylim(y0, y1)
    inset_ax.set_aspect("equal")
    inset_ax.tick_params(labelsize=7, length=2.5, pad=1)
    for spine in inset_ax.spines.values():
        spine.set_linewidth(1.0)
        spine.set_edgecolor("0.25")
    ax.indicate_inset_zoom(inset_ax, edgecolor="0.35", alpha=0.9)


def plot_case(case_data: dict, axis_limits: tuple[float, float, float, float]) -> tuple[Path, Path]:
    reference_r = case_data["reference_r"]
    frozen_e = case_data["frozen_e"]
    stage_e = case_data["stage_e"]

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
    ax.plot(frozen_e["err_x"], frozen_e["err_y"], color=COLORS["frozen"], lw=1.8)
    ax.plot(stage_e["err_x"], stage_e["err_y"], color=COLORS["stage"], lw=1.8)
    ax.set_aspect("equal")
    ax.axhline(0.0, color="0.65", lw=0.8, zorder=1)
    ax.axvline(0.0, color="0.65", lw=0.8, zorder=1)
    ax.set_xlim(axis_limits[0], axis_limits[1])
    ax.set_ylim(axis_limits[2], axis_limits[3])
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    if case_data["name"] in ZOOM_INSET_CASES:
        add_zoom_inset(ax, frozen_e, stage_e, compute_zoom_limits(frozen_e, stage_e))

    out_prefix = FIG_ROOT / case_data["name"]
    out_csv = out_prefix.with_suffix(".csv")
    out_png = out_prefix.with_suffix(".png")
    out_pdf = out_prefix.with_suffix(".pdf")
    pd.concat(
        [
            reference_r.assign(series="reference"),
            frozen_e.assign(series="frozen"),
            stage_e.assign(series="stage"),
        ],
        ignore_index=True,
    ).to_csv(out_csv, index=False)
    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    fig.savefig(out_pdf, bbox_inches="tight")
    plt.close(fig)
    return out_png, out_pdf


def main() -> None:
    plt.rcParams.update({"font.size": 10})
    FIG_ROOT.mkdir(parents=True, exist_ok=True)
    selected_cases = [select_case(scenario_spec, kp) for scenario_spec in SCENARIOS for kp in KP_VALUES]
    cases_data = [load_case_data(case) for case in selected_cases]
    limits_by_scenario = compute_scenario_limits(cases_data)
    selection_rows = []
    for case_data in cases_data:
        out_png, out_pdf = plot_case(case_data, limits_by_scenario[case_data["scenario"]])
        print(f"{case_data['name']}_png={out_png}")
        print(f"{case_data['name']}_pdf={out_pdf}")
        selection_rows.append(
            {
                "name": case_data["name"],
                "scenario": case_data["scenario"],
                "kp": case_data["kp"],
                "result_dir": case_data["result_dir"],
                "seed": case_data["seed"],
                "uav_idx": case_data["uav_idx"],
                "team_gap": case_data["team_gap"],
                "uav_gap": case_data["uav_gap"],
                "axis_limit": limits_by_scenario[case_data["scenario"]][1],
            }
        )
    pd.DataFrame(selection_rows).to_csv(FIG_ROOT / "paper_fig1_selected_cases.csv", index=False)


if __name__ == "__main__":
    main()
