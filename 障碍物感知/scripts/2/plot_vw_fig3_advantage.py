#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


BLUE = "#0072B2"
RED = "#C23B22"
GRID = "#D6DADF"
TEXT = "#1F2933"
MUTED = "#5B6570"


MEAN_COLUMNS = {
    "d_bd": "eq17_boundary_drift_rms_mean",
    "e_rot": "eq18_frot_mismatch_rms_mean",
}
P95_COLUMNS = {
    "d_bd": "eq17_boundary_drift_rms_p95",
    "e_rot": "eq18_frot_mismatch_rms_p95",
}


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    default_root = (
        repo_root
        / "results"
        / "vw_three_controller_obs100_v1to3_w1to2_20seed_solverdbd_20260519-0856"
    )
    parser = argparse.ArgumentParser(
        description="Plot Fig. 3 style NiMPB-vs-NI-frozen V/W advantage heatmaps."
    )
    parser.add_argument(
        "run_root",
        nargs="?",
        type=Path,
        default=default_root,
        help="V/W experiment root containing tables/summary_by_vw_controller.csv.",
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
        help="Comma-separated output formats.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=320,
        help="DPI for raster outputs.",
    )
    return parser.parse_args()


def fmt_axis_value(value: float) -> str:
    if math.isclose(value, round(value), abs_tol=1e-9):
        return str(int(round(value)))
    return f"{value:g}"


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": ["Helvetica", "Arial", "DejaVu Sans"],
            "mathtext.fontset": "dejavusans",
            "font.size": 7.4,
            "axes.labelsize": 8.0,
            "axes.titlesize": 8.2,
            "xtick.labelsize": 7.0,
            "ytick.labelsize": 7.0,
            "axes.linewidth": 0.75,
            "figure.dpi": 170,
            "savefig.dpi": 320,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "axes.unicode_minus": False,
        }
    )


def read_summary(run_root: Path) -> pd.DataFrame:
    csv_path = run_root / "tables" / "summary_by_vw_controller.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"Missing summary CSV: {csv_path}")
    df = pd.read_csv(csv_path)
    required = {
        "v",
        "w",
        "controller",
        *MEAN_COLUMNS.values(),
        *P95_COLUMNS.values(),
    }
    missing = sorted(required - set(df.columns))
    if missing:
        raise ValueError(f"Summary CSV is missing required columns: {missing}")
    for column in ["v", "w", *MEAN_COLUMNS.values(), *P95_COLUMNS.values()]:
        df[column] = pd.to_numeric(df[column], errors="coerce")
    return df


def build_advantage_table(
    summary: pd.DataFrame,
    metric_columns: dict[str, str],
    *,
    suffix: str = "",
) -> pd.DataFrame:
    frozen = (
        summary[summary["controller"] == "noninertial_frozen"]
        .set_index(["v", "w"])
        .sort_index()
    )
    stage = (
        summary[summary["controller"] == "noninertial_stage"]
        .set_index(["v", "w"])
        .sort_index()
    )
    common = frozen.index.intersection(stage.index)
    if common.empty:
        raise ValueError("No shared V/W rows for noninertial_frozen and noninertial_stage.")

    rows: list[dict[str, float]] = []
    for v, w in common:
        d_col = metric_columns["d_bd"]
        e_col = metric_columns["e_rot"]
        d_frozen = float(frozen.loc[(v, w), d_col])
        d_stage = float(stage.loc[(v, w), d_col])
        e_frozen = float(frozen.loc[(v, w), e_col])
        e_stage = float(stage.loc[(v, w), e_col])
        rows.append(
            {
                "v": float(v),
                "w": float(w),
                f"adv_d_bd{suffix}": d_frozen - d_stage,
                f"adv_e_rot{suffix}": e_frozen - e_stage,
                f"d_bd{suffix}_frozen": d_frozen,
                f"d_bd{suffix}_nimpb": d_stage,
                f"e_rot{suffix}_frozen": e_frozen,
                f"e_rot{suffix}_nimpb": e_stage,
            }
        )
    return pd.DataFrame(rows).sort_values(["v", "w"]).reset_index(drop=True)


def matrix_from_table(table: pd.DataFrame, value_column: str) -> tuple[np.ndarray, list[float], list[float]]:
    v_values = sorted(table["v"].dropna().unique())
    w_values = sorted(table["w"].dropna().unique())
    matrix = np.full((len(v_values), len(w_values)), np.nan, dtype=float)
    index_v = {value: idx for idx, value in enumerate(v_values)}
    index_w = {value: idx for idx, value in enumerate(w_values)}
    for row in table.itertuples(index=False):
        matrix[index_v[row.v], index_w[row.w]] = float(getattr(row, value_column))
    return matrix, v_values, w_values


def symmetric_limit(values: Iterable[float]) -> float:
    finite = np.asarray([value for value in values if np.isfinite(value)], dtype=float)
    if finite.size == 0:
        return 1.0
    vmax = float(np.nanmax(np.abs(finite)))
    return max(vmax, 1e-6)


def annotate_heatmap(ax: plt.Axes, matrix: np.ndarray, vmax: float) -> None:
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            value = matrix[i, j]
            if not np.isfinite(value):
                label = "nan"
                color = MUTED
            else:
                label = f"{value:.3f}"
                color = "white" if abs(value) > 0.62 * vmax else TEXT
            ax.text(j, i, label, ha="center", va="center", fontsize=7.0, color=color)


def style_heatmap_axes(ax: plt.Axes, v_values: list[float], w_values: list[float]) -> None:
    ax.set_xticks(range(len(w_values)))
    ax.set_xticklabels([fmt_axis_value(value) for value in w_values])
    ax.set_yticks(range(len(v_values)))
    ax.set_yticklabels([fmt_axis_value(value) for value in v_values])
    ax.set_xlabel("W [rad/s]")
    ax.set_ylabel("V [m/s]")
    ax.set_xticks(np.arange(-0.5, len(w_values), 1.0), minor=True)
    ax.set_yticks(np.arange(-0.5, len(v_values), 1.0), minor=True)
    ax.grid(which="minor", color="white", linewidth=1.1)
    ax.tick_params(which="minor", bottom=False, left=False)
    for spine in ax.spines.values():
        spine.set_color(GRID)
        spine.set_linewidth(0.75)


def plot_advantage(
    table: pd.DataFrame,
    *,
    out_dir: Path,
    stem: str,
    suffix: str,
    title_suffix: str,
    formats: list[str],
    dpi: int,
) -> None:
    configure_style()
    d_key = f"adv_d_bd{suffix}"
    e_key = f"adv_e_rot{suffix}"
    d_matrix, v_values, w_values = matrix_from_table(table, d_key)
    e_matrix, _, _ = matrix_from_table(table, e_key)

    d_limit = symmetric_limit(d_matrix.ravel())
    e_limit = symmetric_limit(e_matrix.ravel())

    fig, axes = plt.subplots(1, 2, figsize=(6.35, 3.25), constrained_layout=False)
    panels = [
        (
            axes[0],
            d_matrix,
            d_limit,
            "Boundary drift advantage",
            "$\\Delta d_{bd}$ [m]",
        ),
        (
            axes[1],
            e_matrix,
            e_limit,
            "Rotational mismatch advantage",
            "$\\Delta e_{rot}$ [m/s$^2$]",
        ),
    ]
    for ax, matrix, limit, title, colorbar_label in panels:
        image = ax.imshow(matrix, origin="lower", cmap="RdBu", vmin=-limit, vmax=limit)
        annotate_heatmap(ax, matrix, limit)
        style_heatmap_axes(ax, v_values, w_values)
        ax.set_title(title, pad=5)
        cbar = fig.colorbar(image, ax=ax, fraction=0.046, pad=0.035)
        cbar.ax.tick_params(labelsize=6.4, length=2.0, width=0.6)
        cbar.outline.set_linewidth(0.6)
        cbar.set_label(colorbar_label, fontsize=7.0, labelpad=3)

    fig.suptitle(
        f"NiMPB advantage over NI-frozen ({title_suffix}); positive means lower error",
        y=0.992,
        fontsize=8.6,
        fontweight="semibold",
        color=TEXT,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.93), w_pad=1.2)

    out_dir.mkdir(parents=True, exist_ok=True)
    for fmt in formats:
        path = out_dir / f"{stem}.{fmt}"
        fig.savefig(path, bbox_inches="tight", dpi=dpi if fmt.lower() == "png" else None)
        print(f"[INFO] wrote {path}")
    plt.close(fig)


def main() -> None:
    args = parse_args()
    run_root = args.run_root.resolve()
    out_dir = (args.out_dir or (run_root / "figures")).resolve()
    formats = [fmt.strip().lstrip(".") for fmt in args.formats.split(",") if fmt.strip()]
    if not formats:
        raise ValueError("No output formats requested.")

    summary = read_summary(run_root)
    mean_table = build_advantage_table(summary, MEAN_COLUMNS)
    p95_table = build_advantage_table(summary, P95_COLUMNS, suffix="_p95")

    mean_csv = out_dir / "fig3_nimpb_vs_ni_frozen_advantage_vw.csv"
    p95_csv = out_dir / "fig3_nimpb_vs_ni_frozen_advantage_p95_vw.csv"
    out_dir.mkdir(parents=True, exist_ok=True)
    mean_table.to_csv(mean_csv, index=False)
    p95_table.to_csv(p95_csv, index=False)
    print(f"[INFO] wrote {mean_csv}")
    print(f"[INFO] wrote {p95_csv}")

    plot_advantage(
        mean_table,
        out_dir=out_dir,
        stem="fig3_nimpb_vs_ni_frozen_advantage_vw",
        suffix="",
        title_suffix="mean over seeds",
        formats=formats,
        dpi=args.dpi,
    )
    plot_advantage(
        p95_table,
        out_dir=out_dir,
        stem="fig3_nimpb_vs_ni_frozen_advantage_p95_vw",
        suffix="_p95",
        title_suffix="p95 over seeds",
        formats=formats,
        dpi=args.dpi,
    )


if __name__ == "__main__":
    main()
