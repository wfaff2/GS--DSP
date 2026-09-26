#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


CONTROLLERS = ["inertial_frozen", "noninertial_frozen", "noninertial_stage"]
LABELS = {
    "inertial_frozen": "I-frozen",
    "noninertial_frozen": "NI-frozen",
    "noninertial_stage": "NiMPB",
}
METRIC_GROUPS = {
    "constraint": [
        ("solver_slack_sum_mean", "Solver slack sum mean", "lower"),
        ("min_h_min", "min h, worst seed", "higher"),
    ],
    "eq17_eq18": [
        ("eq17_boundary_drift_rms_mean", r"Eq.17 $d_{bd}$ RMS", "lower"),
        ("eq18_frot_mismatch_rms_mean", r"Eq.18 $e_{rot}$ RMS", "lower"),
    ],
    "core_time": [
        ("core_time_mean_ms_mean", "core time mean [ms]", "lower"),
        ("core_time_p95_ms_mean", "core time p95 [ms]", "lower"),
    ],
}


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    default_root = (
        repo_root
        / "results"
        / "vw_three_controller_obs100_v1to3_w1to2_20seed_solverdbd_20260519-0856"
    )
    parser = argparse.ArgumentParser(
        description="Plot requested V/W metrics for the three-controller experiment."
    )
    parser.add_argument(
        "run_root",
        nargs="?",
        type=Path,
        default=default_root,
        help="Run root containing tables/summary_by_vw_controller.csv.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Output directory. Defaults to <run_root>/figures.",
    )
    parser.add_argument(
        "--formats",
        default="png,svg,pdf",
        help="Comma-separated figure formats.",
    )
    parser.add_argument("--dpi", type=int, default=320)
    return parser.parse_args()


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": ["Helvetica", "Arial", "DejaVu Sans"],
            "mathtext.fontset": "dejavusans",
            "font.size": 7.2,
            "axes.labelsize": 7.8,
            "axes.titlesize": 8.0,
            "xtick.labelsize": 6.8,
            "ytick.labelsize": 6.8,
            "figure.dpi": 170,
            "savefig.dpi": 320,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "axes.unicode_minus": False,
        }
    )


def fmt_axis_value(value: float) -> str:
    if math.isclose(value, round(value), abs_tol=1e-9):
        return str(int(round(value)))
    return f"{value:g}"


def read_metric_table(run_root: Path) -> pd.DataFrame:
    summary_path = run_root / "tables" / "summary_by_vw_controller.csv"
    per_seed_path = run_root / "tables" / "per_seed_controller_metrics.csv"
    if not summary_path.exists():
        raise FileNotFoundError(f"Missing summary CSV: {summary_path}")
    if not per_seed_path.exists():
        raise FileNotFoundError(f"Missing per-seed CSV: {per_seed_path}")

    summary = pd.read_csv(summary_path)
    per_seed = pd.read_csv(per_seed_path)
    for df in (summary, per_seed):
        for column in ("v", "w"):
            df[column] = pd.to_numeric(df[column], errors="coerce")
    numeric_summary_cols = [
        "solver_slack_sum_mean",
        "solver_slack_sum_total",
        "eq17_boundary_drift_rms_mean",
        "eq18_frot_mismatch_rms_mean",
        "core_time_mean_ms_mean",
        "core_time_p95_ms_mean",
    ]
    for column in numeric_summary_cols:
        summary[column] = pd.to_numeric(summary[column], errors="coerce")
    for column in ("min_h", "solver_min_h", "solver_slack_sum"):
        per_seed[column] = pd.to_numeric(per_seed[column], errors="coerce")

    min_h = (
        per_seed.groupby(["v", "w", "controller"], as_index=False)
        .agg(
            min_h_mean=("min_h", "mean"),
            min_h_min=("min_h", "min"),
            solver_min_h_mean_from_seed=("solver_min_h", "mean"),
            solver_min_h_min_from_seed=("solver_min_h", "min"),
            solver_slack_sum_mean_from_seed=("solver_slack_sum", "mean"),
        )
    )
    merged = summary.merge(min_h, on=["v", "w", "controller"], how="left")
    merged["controller"] = pd.Categorical(
        merged["controller"], categories=CONTROLLERS, ordered=True
    )
    return merged.sort_values(["v", "w", "controller"]).reset_index(drop=True)


def metric_matrix(
    table: pd.DataFrame,
    *,
    controller: str,
    metric: str,
    v_values: list[float],
    w_values: list[float],
) -> np.ndarray:
    subset = table[table["controller"] == controller].set_index(["v", "w"])
    matrix = np.full((len(v_values), len(w_values)), np.nan)
    for i, v in enumerate(v_values):
        for j, w in enumerate(w_values):
            if (v, w) in subset.index:
                matrix[i, j] = float(subset.loc[(v, w), metric])
    return matrix


def format_cell(value: float, metric: str) -> str:
    if not np.isfinite(value):
        return "nan"
    if "slack" in metric:
        return f"{value:.0f}"
    if "core_time" in metric:
        return f"{value:.2f}"
    return f"{value:.3f}"


def finite_limits(matrices: list[np.ndarray], metric: str) -> tuple[float, float, str]:
    values = np.concatenate([m[np.isfinite(m)] for m in matrices if np.isfinite(m).any()])
    if values.size == 0:
        return 0.0, 1.0, "viridis"
    if metric in {"min_h_mean", "min_h_min"}:
        bound = max(abs(float(values.min())), abs(float(values.max())), 1e-6)
        return -bound, bound, "RdBu"
    vmin = float(values.min())
    vmax = float(values.max())
    if math.isclose(vmin, vmax, rel_tol=0.0, abs_tol=1e-12):
        pad = max(abs(vmin) * 0.05, 1e-6)
        return vmin - pad, vmax + pad, "viridis"
    return vmin, vmax, "viridis"


def annotate(ax: plt.Axes, matrix: np.ndarray, metric: str, vmin: float, vmax: float) -> None:
    span = max(vmax - vmin, 1e-12)
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            value = matrix[i, j]
            label = format_cell(value, metric)
            normalized = (value - vmin) / span if np.isfinite(value) else 0.0
            if metric in {"min_h_mean", "min_h_min"}:
                color = (
                    "white"
                    if np.isfinite(value)
                    and abs(value) > 0.45 * max(abs(vmin), abs(vmax))
                    else "#1F2933"
                )
            else:
                color = "white" if normalized < 0.55 else "#1F2933"
            ax.text(j, i, label, ha="center", va="center", fontsize=6.5, color=color)


def style_axes(ax: plt.Axes, v_values: list[float], w_values: list[float]) -> None:
    ax.set_xticks(range(len(w_values)))
    ax.set_xticklabels([fmt_axis_value(v) for v in w_values])
    ax.set_yticks(range(len(v_values)))
    ax.set_yticklabels([fmt_axis_value(v) for v in v_values])
    ax.set_xlabel("W [rad/s]")
    ax.set_ylabel("V [m/s]")
    ax.set_xticks(np.arange(-0.5, len(w_values), 1.0), minor=True)
    ax.set_yticks(np.arange(-0.5, len(v_values), 1.0), minor=True)
    ax.grid(which="minor", color="white", linewidth=1.0)
    ax.tick_params(which="minor", bottom=False, left=False)
    for spine in ax.spines.values():
        spine.set_color("#D6DADF")
        spine.set_linewidth(0.75)


def plot_group(
    table: pd.DataFrame,
    *,
    group_name: str,
    out_dir: Path,
    formats: list[str],
    dpi: int,
) -> None:
    configure_style()
    metrics = METRIC_GROUPS[group_name]
    v_values = sorted(table["v"].dropna().unique())
    w_values = sorted(table["w"].dropna().unique())
    fig, axes = plt.subplots(
        len(metrics),
        len(CONTROLLERS),
        figsize=(8.1, 5.1),
        constrained_layout=False,
    )
    if len(metrics) == 1:
        axes = np.asarray([axes])

    for row_idx, (metric, title, _) in enumerate(metrics):
        matrices = [
            metric_matrix(
                table,
                controller=controller,
                metric=metric,
                v_values=v_values,
                w_values=w_values,
            )
            for controller in CONTROLLERS
        ]
        vmin, vmax, cmap = finite_limits(matrices, metric)
        for col_idx, controller in enumerate(CONTROLLERS):
            ax = axes[row_idx, col_idx]
            image = ax.imshow(
                matrices[col_idx],
                origin="lower",
                cmap=cmap,
                vmin=vmin,
                vmax=vmax,
                aspect="auto",
            )
            annotate(ax, matrices[col_idx], metric, vmin, vmax)
            style_axes(ax, v_values, w_values)
            ax.set_title(f"{LABELS[controller]} | {title}", pad=4)
            if col_idx != 0:
                ax.set_ylabel("")
            cbar = fig.colorbar(image, ax=ax, fraction=0.046, pad=0.025)
            cbar.ax.tick_params(labelsize=6.0, length=2.0, width=0.6)
            cbar.outline.set_linewidth(0.6)

    fig.tight_layout(h_pad=1.1, w_pad=1.0)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"vw_{group_name}_heatmaps"
    for fmt in formats:
        path = out_dir / f"{stem}.{fmt}"
        fig.savefig(path, bbox_inches="tight", dpi=dpi if fmt.lower() == "png" else None)
        print(f"[INFO] wrote {path}")
    plt.close(fig)


def main() -> None:
    args = parse_args()
    run_root = args.run_root.resolve()
    out_dir = (args.out_dir or run_root / "figures").resolve()
    formats = [fmt.strip().lstrip(".") for fmt in args.formats.split(",") if fmt.strip()]
    if not formats:
        raise ValueError("No output formats requested.")

    table = read_metric_table(run_root)
    out_dir.mkdir(parents=True, exist_ok=True)
    requested_cols = [
        "v",
        "w",
        "controller",
        "seed_count",
        "collision_count",
        "collision_rate",
        "solver_slack_sum_mean",
        "solver_slack_sum_total",
        "min_h_mean",
        "min_h_min",
        "solver_min_h_mean_from_seed",
        "solver_min_h_min_from_seed",
        "eq17_boundary_drift_rms_mean",
        "eq18_frot_mismatch_rms_mean",
        "core_time_mean_ms_mean",
        "core_time_p95_ms_mean",
    ]
    csv_path = out_dir / "vw_requested_metrics_by_controller.csv"
    table[requested_cols].to_csv(csv_path, index=False)
    print(f"[INFO] wrote {csv_path}")

    for group_name in METRIC_GROUPS:
        group_cols = ["v", "w", "controller"] + [m[0] for m in METRIC_GROUPS[group_name]]
        group_csv = out_dir / f"vw_{group_name}_source.csv"
        table[group_cols].to_csv(group_csv, index=False)
        print(f"[INFO] wrote {group_csv}")
        plot_group(table, group_name=group_name, out_dir=out_dir, formats=formats, dpi=args.dpi)


if __name__ == "__main__":
    main()
