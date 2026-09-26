#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import pandas as pd
from matplotlib import pyplot as plt

DISPLAY_NAME = {
    "inertial_frozen": "Inertial Frozen",
    "noninertial_frozen": "Non-inertial Frozen",
    "noninertial_stage": "Non-inertial Stage",
}


@dataclass
class TableSpec:
    stem: str
    title: str
    headers: list[str]
    rows: list[list[str]]
    group_breaks: list[int]
    width: float
    row_height: float = 0.34
    title_height: float = 0.36
    font_size: float = 7.1
    header_font_size: float = 7.1
    left_align_cols: tuple[int, ...] = (0,)
    header_scale: float = 1.35


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Generate IEEE-style table figures as PDF and PNG."
    )
    parser.add_argument(
        "--run-root",
        type=Path,
        default=repo_root / "results" / "codex_stage_omega_with_cbf_kpgrid_20seed_20260403",
        help="Unified with-CBF run root.",
    )
    parser.add_argument(
        "--pure-root",
        type=Path,
        default=repo_root / "results" / "codex_pure_tracking_obs0_no_cbf_kpgrid_20seed_20260402",
        help="Pure-tracking baseline root.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=repo_root / "results" / "new" / "ieee_publication_tables",
        help="Directory used to save table figures.",
    )
    return parser.parse_args()


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
            "font.size": 7.2,
            "axes.unicode_minus": False,
            "figure.dpi": 180,
            "savefig.dpi": 320,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def fmt_num(value: float, digits: int = 4) -> str:
    return f"{float(value):.{digits}f}"


def fmt_gain(value: float, digits: int = 2) -> str:
    return f"{float(value):.{digits}f}x"


def fmt_int(value: float | int) -> str:
    return f"{int(value):,}"


def fmt_pct(value: float, digits: int = 0) -> str:
    return f"{100.0 * float(value):.{digits}f}%"


def max_line_len(text: str) -> int:
    return max(len(part) for part in str(text).split("\n"))


def estimate_col_widths(headers: list[str], rows: list[list[str]]) -> list[float]:
    widths: list[float] = []
    for col_idx in range(len(headers)):
        column_cells = [headers[col_idx]] + [row[col_idx] for row in rows]
        max_chars = max(max_line_len(cell) for cell in column_cells)
        widths.append(max(0.65, 0.082 * max_chars + 0.26))
    total = sum(widths)
    return [width / total for width in widths]


def draw_table(spec: TableSpec, out_dir: Path) -> None:
    col_widths = estimate_col_widths(spec.headers, spec.rows)
    n_rows = len(spec.rows)
    fig_height = spec.title_height + spec.row_height * n_rows + spec.row_height * spec.header_scale + 0.26
    fig, ax = plt.subplots(figsize=(spec.width, fig_height))
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    left_margin = 0.02
    right_margin = 0.02
    usable_width = 1 - left_margin - right_margin
    x_edges = [left_margin]
    for width in col_widths:
        x_edges.append(x_edges[-1] + usable_width * width)

    table_top = 1 - spec.title_height / fig_height
    unit_h = (table_top - 0.06) / (n_rows + spec.header_scale)
    header_h = unit_h * spec.header_scale
    row_h = unit_h

    ax.text(
        0.5,
        0.975,
        spec.title,
        ha="center",
        va="top",
        fontsize=7.4,
        fontweight="bold",
    )

    ax.hlines(table_top, x_edges[0], x_edges[-1], linewidth=1.35, color="black")
    header_y = table_top - header_h / 2
    for idx, header in enumerate(spec.headers):
        x0, x1 = x_edges[idx], x_edges[idx + 1]
        x = x0 + 0.006 if idx in spec.left_align_cols else (x0 + x1) / 2
        ha = "left" if idx in spec.left_align_cols else "center"
        ax.text(
            x,
            header_y,
            header,
            ha=ha,
            va="center",
            fontsize=spec.header_font_size,
            fontweight="bold",
        )
    header_bottom = table_top - header_h
    ax.hlines(header_bottom, x_edges[0], x_edges[-1], linewidth=0.95, color="black")

    for row_idx, row in enumerate(spec.rows):
        y = header_bottom - row_h * (row_idx + 0.5)
        for col_idx, value in enumerate(row):
            x0, x1 = x_edges[col_idx], x_edges[col_idx + 1]
            x = x0 + 0.006 if col_idx in spec.left_align_cols else (x0 + x1) / 2
            ha = "left" if col_idx in spec.left_align_cols else "center"
            ax.text(
                x,
                y,
                value,
                ha=ha,
                va="center",
                fontsize=spec.font_size,
            )
        if row_idx in spec.group_breaks:
            ax.hlines(
                header_bottom - row_h * (row_idx + 1),
                x_edges[0],
                x_edges[-1],
                linewidth=0.75,
                color="black",
            )

    ax.hlines(header_bottom - row_h * n_rows, x_edges[0], x_edges[-1], linewidth=1.35, color="black")

    out_dir.mkdir(parents=True, exist_ok=True)
    pdf_path = out_dir / f"{spec.stem}.pdf"
    png_path = out_dir / f"{spec.stem}.png"
    fig.savefig(pdf_path, bbox_inches="tight", pad_inches=0.02)
    fig.savefig(png_path, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    print(f"[INFO] wrote {pdf_path}")
    print(f"[INFO] wrote {png_path}")


def build_main_table(run_root: Path) -> TableSpec:
    crot = pd.read_csv(run_root / "crot_summary_by_method_kp_common_nocollision.csv")
    metrics = pd.read_csv(run_root / "kp_grid_three_case_with_cbf_common_nocollision_summary.csv")
    merged = crot.merge(metrics, left_on=["method", "kp"], right_on=["case", "kp"], how="inner")
    rows: list[list[str]] = []
    group_breaks: list[int] = []
    row_idx = -1
    for method in ("inertial_frozen", "noninertial_frozen", "noninertial_stage"):
        block = merged[merged["method"] == method].sort_values("kp")
        for offset, rec in enumerate(block.itertuples(index=False)):
            row_idx += 1
            rows.append(
                [
                    DISPLAY_NAME[method] if offset == 0 else "",
                    f"{rec.kp:.1f}",
                    fmt_num(rec.mean_c_rot_norm, 4),
                    fmt_num(rec.p95_c_rot_norm, 4),
                    fmt_num(rec.tracking_rms_mean, 4),
                    fmt_num(rec.tracking_rms_track_only_mean, 4),
                    fmt_num(rec.surface_distance_mean, 4),
                ]
            )
        group_breaks.append(row_idx)
    group_breaks = group_breaks[:-1]
    return TableSpec(
        stem="table_main_summary",
        title="MAIN SUMMARY   MACRO PERFORMANCE ON THE COMMON NO-COLLISION SUBSET",
        headers=[
            "Method",
            "$k_p$",
            "Mean\n$\\Vert c_{rot} \\Vert$",
            "P95\n$\\Vert c_{rot} \\Vert$",
            "Tracking\nRMS",
            "Track-only\nRMS",
            "Nearest-surface\ndistance",
        ],
        rows=rows,
        group_breaks=group_breaks,
        width=7.1,
    )


def build_fig12_combined_table(run_root: Path) -> TableSpec:
    full = pd.read_csv(run_root / "kp_grid_three_case_with_cbf_1to20_plus_kp3_summary.csv")
    common = pd.read_csv(run_root / "kp_grid_three_case_with_cbf_common_nocollision_summary.csv")
    frozen_anchor = pd.read_csv(run_root / "frot_frozen_mismatch_anchor_rows_common_nocollision.csv")
    stage_anchor = pd.read_csv(run_root / "frot_stage_mismatch_common_nocollision_anchor_rows.csv")

    full_map = full.set_index(["kp", "case"])
    common_map = common.set_index(["kp", "case"])
    frozen_stats = frozen_anchor.groupby("kp").agg(
        mean_mismatch=("frot_mismatch_rms", "mean"),
        p99_mismatch=("frot_mismatch_rms", lambda s: s.quantile(0.99)),
    )
    stage_stats = stage_anchor.groupby("kp").agg(
        mean_mismatch=("frot_stage_mismatch_rms", "mean"),
        p99_mismatch=("frot_stage_mismatch_rms", lambda s: s.quantile(0.99)),
    )

    rows: list[list[str]] = []
    for kp in [1.0, 1.5, 2.0, 2.5, 3.0]:
        inertial = common_map.loc[(kp, "inertial_frozen")]
        frozen_common = common_map.loc[(kp, "noninertial_frozen")]
        stage_common = common_map.loc[(kp, "noninertial_stage")]
        inertial_full = full_map.loc[(kp, "inertial_frozen")]
        frozen_full = full_map.loc[(kp, "noninertial_frozen")]
        stage_full = full_map.loc[(kp, "noninertial_stage")]
        frozen_m = frozen_stats.loc[kp]
        stage_m = stage_stats.loc[kp]

        rows.append(
            [
                f"{kp:.1f}",
                fmt_int(inertial["num_seeds"]),
                " / ".join(
                    [
                        fmt_pct(inertial_full["collision_mean"]),
                        fmt_pct(frozen_full["collision_mean"]),
                        fmt_pct(stage_full["collision_mean"]),
                    ]
                ),
                " / ".join(
                    [
                        f"{fmt_num(inertial['tracking_rms_mean'], 4)} +/- {fmt_num(inertial['tracking_rms_std'], 4)}",
                        f"{fmt_num(frozen_common['tracking_rms_mean'], 4)} +/- {fmt_num(frozen_common['tracking_rms_std'], 4)}",
                        f"{fmt_num(stage_common['tracking_rms_mean'], 4)} +/- {fmt_num(stage_common['tracking_rms_std'], 4)}",
                    ]
                ),
                " / ".join(
                    [
                        fmt_num(frozen_m["mean_mismatch"], 4),
                        fmt_num(stage_m["mean_mismatch"], 4),
                    ]
                ),
                " / ".join(
                    [
                        fmt_num(frozen_m["p99_mismatch"], 4),
                        fmt_num(stage_m["p99_mismatch"], 4),
                    ]
                ),
            ]
        )

    return TableSpec(
        stem="table_fig12_combined",
        title="TABLE 1   MERGED SUMMARY OF ORIGINAL FIG. 1-2  (I/F/S = INERTIAL/FROZEN/STAGE, F/S = FROZEN/STAGE)",
        headers=[
            "$k_p$",
            "Common\nNo-Collision\nSeeds",
            "Collision Rate\n[I / F / S]",
            "Tracking RMS +/- 1 sigma\n[I / F / S] [m]",
            "Mean $f_{rot}$\nMismatch [F / S]\n[m/s$^2$]",
            "P99 $f_{rot}$\nMismatch [F / S]\n[m/s$^2$]",
        ],
        rows=rows,
        group_breaks=[],
        width=11.3,
        row_height=0.44,
        font_size=6.25,
        header_font_size=6.2,
        left_align_cols=(),
    )


def load_crot_compare_frame(run_root: Path) -> pd.DataFrame:
    df = pd.read_csv(run_root / "crot_summary_by_method_kp_common_nocollision.csv")
    method_order = ["noninertial_frozen", "noninertial_stage"]
    base = (
        df[df["kp"] == 1.0][["method", "mean_c_rot_norm", "p95_c_rot_norm"]]
        .rename(
            columns={
                "mean_c_rot_norm": "mean_base",
                "p95_c_rot_norm": "p95_base",
            }
        )
    )
    rel = (
        df.merge(base, on="method", how="left")
        .assign(
            mean_gain_vs_kp1=lambda x: x["mean_c_rot_norm"] / x["mean_base"],
            p95_gain_vs_kp1=lambda x: x["p95_c_rot_norm"] / x["p95_base"],
        )
    )
    return rel[rel["method"].isin(method_order)].copy()


def build_crot_trend_table(run_root: Path) -> TableSpec:
    df = pd.read_csv(run_root / "crot_summary_by_method_kp_common_nocollision.csv")
    method_order = ["inertial_frozen", "noninertial_frozen", "noninertial_stage"]
    base = (
        df[df["kp"] == 1.0][["method", "mean_c_rot_norm"]]
        .rename(columns={"mean_c_rot_norm": "mean_base"})
    )
    rel = (
        df.merge(base, on="method", how="left")
        .assign(mean_gain_vs_kp1=lambda x: x["mean_c_rot_norm"] / x["mean_base"])
    )
    metric_order = [
        "mean_c_rot_norm",
        "mean_gain_vs_kp1",
    ]
    wide = (
        rel[rel["method"].isin(method_order)]
        .pivot(index="kp", columns="method", values=metric_order)
        .sort_index()
        .reindex(columns=pd.MultiIndex.from_product([metric_order, method_order]))
    )

    rows: list[list[str]] = []
    for kp, rec in wide.iterrows():
        rows.append(
            [
                f"{kp:.1f}",
                f"{fmt_num(rec[('mean_c_rot_norm', 'inertial_frozen')], 4)} ({fmt_gain(rec[('mean_gain_vs_kp1', 'inertial_frozen')])})",
                f"{fmt_num(rec[('mean_c_rot_norm', 'noninertial_frozen')], 4)} ({fmt_gain(rec[('mean_gain_vs_kp1', 'noninertial_frozen')])})",
                f"{fmt_num(rec[('mean_c_rot_norm', 'noninertial_stage')], 4)} ({fmt_gain(rec[('mean_gain_vs_kp1', 'noninertial_stage')])})",
            ]
        )

    return TableSpec(
        stem="table_crot_trend",
        title="SUPPLEMENTARY TABLE   MEAN CLOSED-LOOP $c_{rot}$ BURDEN  (CELL = ABSOLUTE VALUE, GAIN VS. $k_p$=1.0)",
        headers=[
            "$k_p$",
            "Inertial\nMean\n$\\Vert c_{rot} \\Vert$ [m/s$^2$]",
            "Frozen\nMean\n$\\Vert c_{rot} \\Vert$ [m/s$^2$]",
            "Stage\nMean\n$\\Vert c_{rot} \\Vert$ [m/s$^2$]",
        ],
        rows=rows,
        group_breaks=[],
        width=7.6,
        row_height=0.38,
        font_size=6.65,
        header_font_size=6.55,
    )


def build_crot_p95_appendix_table(run_root: Path) -> TableSpec:
    rel = load_crot_compare_frame(run_root)
    method_order = ["noninertial_frozen", "noninertial_stage"]
    metric_order = [
        "seed_count",
        "p95_c_rot_norm",
        "p95_gain_vs_kp1",
    ]
    wide = (
        rel[rel["method"].isin(method_order)]
        .pivot(index="kp", columns="method", values=metric_order)
        .sort_index()
        .reindex(columns=pd.MultiIndex.from_product([metric_order, method_order]))
    )

    rows: list[list[str]] = []
    for kp, rec in wide.iterrows():
        common_seeds = int(rec[("seed_count", "noninertial_stage")])
        rows.append(
            [
                f"{kp:.1f}",
                fmt_int(common_seeds),
                f"{fmt_num(rec[('p95_c_rot_norm', 'noninertial_frozen')], 4)} ({fmt_gain(rec[('p95_gain_vs_kp1', 'noninertial_frozen')])})",
                f"{fmt_num(rec[('p95_c_rot_norm', 'noninertial_stage')], 4)} ({fmt_gain(rec[('p95_gain_vs_kp1', 'noninertial_stage')])})",
            ]
        )

    return TableSpec(
        stem="table_crot_p95_appendix",
        title="APPENDIX TABLE A2   FROZEN VS. STAGE P95 CLOSED-LOOP $c_{rot}$ BURDEN  (CELL = ABSOLUTE VALUE, GAIN VS. $k_p$=1.0)",
        headers=[
            "$k_p$",
            "Common\nseeds",
            "Frozen\nP95\n$\\Vert c_{rot} \\Vert$ [m/s$^2$]",
            "Stage\nP95\n$\\Vert c_{rot} \\Vert$ [m/s$^2$]",
        ],
        rows=rows,
        group_breaks=[],
        width=6.2,
        row_height=0.38,
        font_size=6.8,
        header_font_size=6.7,
    )


def build_frot_table(run_root: Path, frozen: bool) -> TableSpec:
    if frozen:
        df = pd.read_csv(run_root / "frot_frozen_mismatch_anchor_rows_common_nocollision.csv")
        grouped = df.groupby("kp", as_index=False).agg(
            mean_mismatch=("frot_mismatch_rms", "mean"),
            p99_mismatch=("frot_mismatch_rms", lambda s: s.quantile(0.99)),
            mean_true_frot_norm=("true_frot_mean_norm", "mean"),
            mean_pred_frot_norm=("pred_frot_mean_norm", "mean"),
        )
        pred_label = "Mean frozen\nnorm"
        stem = "table_frot_frozen_mismatch"
        title = "TABLE A   NON-INERTIAL FROZEN PREDICTIVE-DOMAIN $f_{rot}$ MISMATCH"
    else:
        df = pd.read_csv(run_root / "frot_stage_mismatch_common_nocollision_anchor_rows.csv")
        grouped = df.groupby("kp", as_index=False).agg(
            mean_mismatch=("frot_stage_mismatch_rms", "mean"),
            p99_mismatch=("frot_stage_mismatch_rms", lambda s: s.quantile(0.99)),
            mean_true_frot_norm=("true_frot_mean_norm", "mean"),
            mean_pred_frot_norm=("pred_frot_mean_norm", "mean"),
        )
        pred_label = "Mean stage\nnorm"
        stem = "table_frot_stage_mismatch"
        title = "TABLE B   NON-INERTIAL STAGE PREDICTIVE-DOMAIN $f_{rot}$ MISMATCH"
    rows = [
        [
            f"{rec.kp:.1f}",
            fmt_num(rec.mean_mismatch, 4),
            fmt_num(rec.p99_mismatch, 4),
            fmt_num(rec.mean_true_frot_norm, 4),
            fmt_num(rec.mean_pred_frot_norm, 4),
        ]
        for rec in grouped.sort_values("kp").itertuples(index=False)
    ]
    return TableSpec(
        stem=stem,
        title=title,
        headers=[
            "$k_p$",
            "Mean\nmismatch",
            "P99\nmismatch",
            "Mean true\nnorm",
            pred_label,
        ],
        rows=rows,
        group_breaks=[],
        width=5.4,
        row_height=0.36,
    )


def build_slack_table(run_root: Path) -> TableSpec:
    df = pd.read_csv(run_root / "kp_grid_three_case_with_cbf_common_nocollision_summary.csv")
    pivot = (
        df[df["case"].isin(["noninertial_frozen", "noninertial_stage"])]
        .pivot(index="case", columns="kp", values="slack_sum_effective_mean")
        .loc[["noninertial_frozen", "noninertial_stage"], [1.0, 1.5, 2.0, 2.5, 3.0]]
    )
    rows = []
    for case, row in pivot.iterrows():
        rows.append(
            [DISPLAY_NAME[case]]
            + [fmt_num(row[kp], 1) for kp in [1.0, 1.5, 2.0, 2.5, 3.0]]
        )
    return TableSpec(
        stem="table_slack_sum",
        title="TABLE 5   ACCUMULATED SLACK UNDER NON-INERTIAL WITH-CBF",
        headers=["Method", "$k_p$=1.0", "1.5", "2.0", "2.5", "3.0"],
        rows=rows,
        group_breaks=[],
        width=6.0,
        row_height=0.38,
    )


def build_slack_table_three_method(run_root: Path) -> TableSpec:
    df = pd.read_csv(run_root / "kp_grid_three_case_with_cbf_common_nocollision_summary.csv")
    pivot = (
        df[df["case"].isin(["inertial_frozen", "noninertial_frozen", "noninertial_stage"])]
        .pivot(index="case", columns="kp", values="slack_sum_effective_mean")
        .loc[
            ["inertial_frozen", "noninertial_frozen", "noninertial_stage"],
            [1.0, 1.5, 2.0, 2.5, 3.0],
        ]
    )
    rows = []
    for case, row in pivot.iterrows():
        rows.append(
            [DISPLAY_NAME[case]]
            + [fmt_num(row[kp], 1) for kp in [1.0, 1.5, 2.0, 2.5, 3.0]]
        )
    return TableSpec(
        stem="table_slack_sum_three_method",
        title="APPENDIX TABLE   ACCUMULATED SLACK ACROSS ALL THREE METHODS",
        headers=["Method", "$k_p$=1.0", "1.5", "2.0", "2.5", "3.0"],
        rows=rows,
        group_breaks=[],
        width=6.3,
        row_height=0.38,
    )


def build_timing_overall_table(run_root: Path) -> TableSpec:
    df = pd.read_csv(run_root / "noninertial_timing_overall.csv")
    df = df.set_index("method").loc[["noninertial_frozen", "noninertial_stage"]]
    rows = []
    for method, rec in df.iterrows():
        rows.append(
            [
                DISPLAY_NAME[method],
                fmt_int(rec["samples"]),
                fmt_num(rec["t_prep_mean_ms"], 3),
                fmt_num(rec["t_prep_p99_ms"], 3),
                fmt_num(rec["t_prep_max_ms"], 3),
                fmt_num(rec["t_solve_mean_ms"], 3),
                fmt_num(rec["t_solve_p99_ms"], 3),
                fmt_num(rec["t_solve_max_ms"], 3),
                fmt_num(rec["t_total_mean_ms"], 3),
                fmt_num(rec["t_total_p99_ms"], 3),
                fmt_num(rec["t_total_max_ms"], 3),
            ]
        )
    return TableSpec(
        stem="table_timing_overall",
        title="TABLE 6A   NON-INERTIAL WITH-CBF TIMING SUMMARY",
        headers=[
            "Method",
            "Samples",
            "Prep\nMean",
            "Prep\nP99",
            "Prep\nMax",
            "Solve\nMean",
            "Solve\nP99",
            "Solve\nMax",
            "Total\nMean",
            "Total\nP99",
            "Total\nMax",
        ],
        rows=rows,
        group_breaks=[],
        width=7.1,
        row_height=0.38,
        font_size=6.8,
        header_font_size=6.8,
    )


def build_timing_slice_table(run_root: Path) -> TableSpec:
    solve = pd.read_csv(run_root / "noninertial_tsolve_by_cbf_slice.csv")
    total = pd.read_csv(run_root / "noninertial_ttotal_by_cbf_slice.csv")
    df = solve.merge(total, on=["method", "cbf_slice", "samples"], how="inner")
    order = [("noninertial_frozen", "no-avoid"), ("noninertial_frozen", "avoid"),
             ("noninertial_stage", "no-avoid"), ("noninertial_stage", "avoid")]
    rows = []
    for method, slice_name in order:
        rec = df[(df["method"] == method) & (df["cbf_slice"] == slice_name)].iloc[0]
        rows.append(
            [
                DISPLAY_NAME[method],
                slice_name,
                fmt_int(rec["samples"]),
                fmt_num(rec["t_solve_mean_ms"], 3),
                fmt_num(rec["t_solve_p99_ms"], 3),
                fmt_num(rec["t_solve_max_ms"], 3),
                fmt_num(rec["t_total_mean_ms"], 3),
                fmt_num(rec["t_total_p99_ms"], 3),
                fmt_num(rec["t_total_max_ms"], 3),
            ]
        )
    return TableSpec(
        stem="table_timing_slices",
        title="TABLE 6B   TIMING SLICES BY CBF ACTIVITY",
        headers=[
            "Method",
            "Slice",
            "Samples",
            "Solve\nMean",
            "Solve\nP99",
            "Solve\nMax",
            "Total\nMean",
            "Total\nP99",
            "Total\nMax",
        ],
        rows=rows,
        group_breaks=[1],
        width=7.1,
        row_height=0.38,
        font_size=6.9,
        header_font_size=6.9,
        left_align_cols=(0, 1),
    )


def build_pure_tracking_table(pure_root: Path) -> TableSpec:
    df = pd.read_csv(pure_root / "kp_grid_three_case_pure_tracking_obs0_1to20_summary.csv")
    rows: list[list[str]] = []
    group_breaks: list[int] = []
    row_idx = -1
    for case in ("inertial_frozen", "noninertial_frozen", "noninertial_stage"):
        block = df[df["case"] == case].sort_values("kp")
        for offset, rec in enumerate(block.itertuples(index=False)):
            row_idx += 1
            rows.append(
                [
                    DISPLAY_NAME[case] if offset == 0 else "",
                    f"{rec.kp:.1f}",
                    fmt_num(rec.tracking_rms_mean, 4),
                    fmt_num(rec.tracking_rms_track_only_mean, 4),
                    fmt_num(rec.collision_mean, 2),
                ]
            )
        group_breaks.append(row_idx)
    group_breaks = group_breaks[:-1]
    return TableSpec(
        stem="table_pure_tracking_baseline",
        title="APPENDIX TABLE A1   PURE-TRACKING OBS0 BASELINE (20 SEEDS)",
        headers=["Method", "$k_p$", "Tracking\nRMS", "Track-only\nRMS", "Collision\nrate"],
        rows=rows,
        group_breaks=group_breaks,
        width=5.4,
        row_height=0.36,
    )


def build_specs(run_root: Path, pure_root: Path) -> Iterable[TableSpec]:
    yield build_fig12_combined_table(run_root)
    yield build_frot_table(run_root, frozen=True)
    yield build_frot_table(run_root, frozen=False)


def main() -> None:
    args = parse_args()
    configure_style()
    run_root = args.run_root.resolve()
    pure_root = args.pure_root.resolve()
    out_dir = args.out_dir.resolve()

    for spec in build_specs(run_root, pure_root):
        draw_table(spec, out_dir)


if __name__ == "__main__":
    main()
