#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import pathlib
from dataclasses import dataclass

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


@dataclass
class SeedMetric:
    seed: str
    method: str
    active_threshold: float
    active_rows: int
    finite_h_rows: int
    safety_boundary_violation_rows: int
    safety_boundary_violation_rate: float
    masked_max_tracking_error: float
    global_min_h: float
    full_run_tracking_rms: float


def _safe_float(value: str | None) -> float:
    if value is None:
        return math.nan
    text = str(value).strip()
    if not text:
        return math.nan
    try:
        return float(text)
    except ValueError:
        return math.nan


def _fmt(value: float) -> str:
    if not math.isfinite(value):
        return "nan"
    return f"{value:.6f}"


def load_seed_metric(seed_dir: pathlib.Path, prefix: str, threshold: float) -> SeedMetric | None:
    step_csv = seed_dir / f"{prefix}_metrics_steps.csv"
    metrics_csv = seed_dir / f"{prefix}_metrics.csv"
    if not step_csv.is_file():
        return None

    active_rows = 0
    finite_h_rows = 0
    safety_boundary_violation_rows = 0
    masked_max_tracking_error = -math.inf
    global_min_h = math.inf
    full_run_tracking_rms = math.nan

    if metrics_csv.is_file():
        with metrics_csv.open("r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                if str(row.get("scope", "")).strip() == "all":
                    full_run_tracking_rms = _safe_float(row.get("tracking_rms"))
                    break

    with step_csv.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            solver_h = _safe_float(row.get("solver_h"))
            tracking_error = _safe_float(row.get("tracking_error"))
            if math.isfinite(solver_h):
                finite_h_rows += 1
                global_min_h = min(global_min_h, solver_h)
                if solver_h < 0.0:
                    safety_boundary_violation_rows += 1
                if solver_h < threshold:
                    active_rows += 1
                    if math.isfinite(tracking_error):
                        masked_max_tracking_error = max(masked_max_tracking_error, tracking_error)

    if not math.isfinite(global_min_h):
        global_min_h = math.nan
    if masked_max_tracking_error == -math.inf:
        masked_max_tracking_error = math.nan
    safety_boundary_violation_rate = (
        float(safety_boundary_violation_rows) / float(finite_h_rows)
        if finite_h_rows > 0
        else math.nan
    )

    return SeedMetric(
        seed=seed_dir.name.replace("seed_", ""),
        method=prefix,
        active_threshold=threshold,
        active_rows=active_rows,
        finite_h_rows=finite_h_rows,
        safety_boundary_violation_rows=safety_boundary_violation_rows,
        safety_boundary_violation_rate=safety_boundary_violation_rate,
        masked_max_tracking_error=masked_max_tracking_error,
        global_min_h=global_min_h,
        full_run_tracking_rms=full_run_tracking_rms,
    )


def percentile(values: list[float], q: float) -> float:
    finite = np.asarray([v for v in values if math.isfinite(v)], dtype=float)
    if finite.size == 0:
        return math.nan
    return float(np.percentile(finite, q))


def plot_boxplots(inertial_vals: list[float],
                  noninertial_vals: list[float],
                  inertial_h: list[float],
                  noninertial_h: list[float],
                  out_path: pathlib.Path,
                  threshold: float) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 5.2), constrained_layout=True)

    colors = ["#d62728", "#1f77b4"]
    labels = ["Inertial Frozen", "Non-inertial Frozen"]

    left = axes[0]
    bp_left = left.boxplot(
        [inertial_vals, noninertial_vals],
        labels=labels,
        patch_artist=True,
        showfliers=True,
        widths=0.55,
    )
    for patch, color in zip(bp_left["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.45)
    left.set_ylabel("Masked Max Tracking Error [m]")
    left.set_title("Max Tracking Error on Active Segments")
    left.grid(axis="y", alpha=0.25, linestyle="--")

    right = axes[1]
    bp_right = right.boxplot(
        [inertial_h, noninertial_h],
        labels=labels,
        patch_artist=True,
        showfliers=True,
        widths=0.55,
    )
    for patch, color in zip(bp_right["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.45)
    right.axhline(0.0, color="black", linewidth=1.0, linestyle="--", alpha=0.8)
    right.set_ylabel("Global Min h(x) [CBF Safety Function]")
    right.set_title("Minimum CBF Safety Margin over Full Run")
    right.grid(axis="y", alpha=0.25, linestyle="--")
    right.text(
        0.02,
        0.98,
        "Dashed line: h = 0 (CBF safety boundary)",
        transform=right.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox=dict(boxstyle="round,pad=0.25", facecolor="white", alpha=0.85, linewidth=0.0),
    )

    fig.suptitle(f"C1 State-Masked Frozen Comparison (threshold: h < {threshold:.3f})")
    fig.savefig(out_path, dpi=220)
    plt.close(fig)


def write_csv(rows: list[SeedMetric], out_path: pathlib.Path) -> None:
    with out_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "seed",
                "method",
                "active_threshold",
                "active_rows",
                "finite_h_rows",
                "safety_boundary_violation_rows",
                "safety_boundary_violation_rate",
                "masked_max_tracking_error",
                "global_min_h",
                "full_run_tracking_rms",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    row.seed,
                    row.method,
                    f"{row.active_threshold:.6f}",
                    row.active_rows,
                    row.finite_h_rows,
                    row.safety_boundary_violation_rows,
                    _fmt(row.safety_boundary_violation_rate),
                    _fmt(row.masked_max_tracking_error),
                    _fmt(row.global_min_h),
                    _fmt(row.full_run_tracking_rms),
                ]
            )


def write_markdown(
    inertial_rows: list[SeedMetric],
    noninertial_rows: list[SeedMetric],
    out_path: pathlib.Path,
    boxplot_path: pathlib.Path,
    threshold: float,
) -> None:
    inertial_tracking = [row.masked_max_tracking_error for row in inertial_rows]
    noninertial_tracking = [row.masked_max_tracking_error for row in noninertial_rows]
    inertial_h = [row.global_min_h for row in inertial_rows]
    noninertial_h = [row.global_min_h for row in noninertial_rows]
    inertial_full_run_tracking = [row.full_run_tracking_rms for row in inertial_rows]
    noninertial_full_run_tracking = [row.full_run_tracking_rms for row in noninertial_rows]
    inertial_violation_rate = [row.safety_boundary_violation_rate for row in inertial_rows]
    noninertial_violation_rate = [row.safety_boundary_violation_rate for row in noninertial_rows]

    def line(label: str, values: list[float]) -> str:
        finite = [v for v in values if math.isfinite(v)]
        if not finite:
            return f"{label}: n=0"
        return (
            f"{label}: n={len(finite)}, "
            f"mean={np.mean(finite):.6f}, median={np.median(finite):.6f}, "
            f"p25={percentile(finite, 25):.6f}, p75={percentile(finite, 75):.6f}"
        )

    lines = [
        "# C1 State-Masked Frozen Comparison",
        "",
        f"- Active-segment trigger: `solver_h < {threshold:.6f}`",
        "- Left boxplot metric: per-seed masked max tracking error.",
        "- Right boxplot metric: per-seed global minimum CBF safety function value.",
        "- `h = 0` denotes the CBF safety boundary, not the physical collision boundary.",
        "",
        "## Masked Max Tracking Error",
        "",
        f"- {line('Inertial Frozen', inertial_tracking)}",
        f"- {line('Non-inertial Frozen', noninertial_tracking)}",
        "",
        "## Global Min h(x)",
        "",
        f"- {line('Inertial Frozen', inertial_h)}",
        f"- {line('Non-inertial Frozen', noninertial_h)}",
        "",
        "## Full-Run Tracking RMS",
        "",
        f"- {line('Inertial Frozen', inertial_full_run_tracking)}",
        f"- {line('Non-inertial Frozen', noninertial_full_run_tracking)}",
        "",
        "## Safety-Boundary Violation Rate",
        "",
        "- Definition: fraction of finite samples satisfying `solver_h < 0`.",
        f"- {line('Inertial Frozen', inertial_violation_rate)}",
        f"- {line('Non-inertial Frozen', noninertial_violation_rate)}",
        "",
        "## Figure",
        "",
        f"![c1_state_mask_boxplots]({boxplot_path.name})",
        "",
    ]
    out_path.write_text("\n".join(lines), encoding="utf-8")


def write_full_run_tracking_table(
    inertial_rows: list[SeedMetric],
    noninertial_rows: list[SeedMetric],
    out_path: pathlib.Path,
) -> None:
    def mean_std(values: list[float]) -> str:
        finite = np.asarray([v for v in values if math.isfinite(v)], dtype=float)
        if finite.size == 0:
            return "N/A"
        if finite.size == 1:
            return f"{finite[0]:.6f} ± {0.0:.6f}"
        return f"{float(finite.mean()):.6f} ± {float(finite.std(ddof=1)):.6f}"

    inertial_tracking = [row.full_run_tracking_rms for row in inertial_rows]
    noninertial_tracking = [row.full_run_tracking_rms for row in noninertial_rows]

    lines = [
        "# C1 Summary Table",
        "",
        "| Method | Full-Run Tracking RMS [m] |",
        "|:--|:--|",
        f"| Inertial Frozen | {mean_std(inertial_tracking)} |",
        f"| Non-inertial Frozen | {mean_std(noninertial_tracking)} |",
        "",
    ]
    out_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build C1 state-masked frozen comparison boxplots.")
    parser.add_argument("run_root", type=pathlib.Path, help="30-seed compare run root")
    parser.add_argument("--out-dir", type=pathlib.Path, default=None, help="Output directory")
    parser.add_argument("--threshold", type=float, default=0.0, help="Active mask threshold on solver_h")
    args = parser.parse_args()

    run_root = args.run_root.expanduser().resolve()
    out_dir = (args.out_dir or run_root).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    inertial_rows: list[SeedMetric] = []
    noninertial_rows: list[SeedMetric] = []

    for seed_dir in sorted(run_root.glob("seed_*")):
        if not seed_dir.is_dir():
            continue
        inertial_row = load_seed_metric(seed_dir, "inertial", args.threshold)
        noninertial_row = load_seed_metric(seed_dir, "noninertial", args.threshold)
        if inertial_row is None or noninertial_row is None:
            continue
        inertial_rows.append(inertial_row)
        noninertial_rows.append(noninertial_row)

    if not inertial_rows or not noninertial_rows:
        raise SystemExit(f"No valid seed rows found under {run_root}")

    all_rows = inertial_rows + noninertial_rows
    csv_path = out_dir / "c1_state_mask_metrics.csv"
    boxplot_path = out_dir / "c1_state_mask_boxplots.png"
    md_path = out_dir / "c1_state_mask_summary.md"
    table_path = out_dir / "c1_summary_table.md"

    write_csv(all_rows, csv_path)
    plot_boxplots(
        [row.masked_max_tracking_error for row in inertial_rows],
        [row.masked_max_tracking_error for row in noninertial_rows],
        [row.global_min_h for row in inertial_rows],
        [row.global_min_h for row in noninertial_rows],
        boxplot_path,
        args.threshold,
    )
    write_markdown(inertial_rows, noninertial_rows, md_path, boxplot_path, args.threshold)
    write_full_run_tracking_table(inertial_rows, noninertial_rows, table_path)

    print(f"[INFO] c1 metrics csv: {csv_path}")
    print(f"[INFO] c1 boxplots: {boxplot_path}")
    print(f"[INFO] c1 summary: {md_path}")
    print(f"[INFO] c1 table: {table_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
