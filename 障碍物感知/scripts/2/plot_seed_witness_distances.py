#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse

matplotlib.rcParams["font.family"] = ["Nimbus Roman", "Liberation Serif", "Liberation Sans"]
matplotlib.rcParams["font.size"] = 14
matplotlib.rcParams["mathtext.fontset"] = "stix"


def _safe_float(text: object) -> float:
    try:
        return float(text)
    except Exception:
        return math.nan


def _safe_int(text: object) -> int | None:
    try:
        return int(str(text).strip())
    except Exception:
        return None


def _sanitize_filename(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", text)


def _resolve_summary_csv(path_text: str) -> pathlib.Path:
    path = pathlib.Path(path_text).expanduser().resolve()
    if path.is_dir():
        path = path / "seed_metrics_summary.csv"
    if not path.is_file():
        raise SystemExit(f"seed summary CSV not found: {path}")
    return path


def _resolve_steps_csv(metrics_csv_text: str) -> pathlib.Path | None:
    metrics_csv = pathlib.Path(metrics_csv_text).expanduser().resolve()
    if not metrics_csv.is_file():
        return None
    if metrics_csv.name.endswith("_metrics.csv"):
        steps_name = metrics_csv.name.replace("_metrics.csv", "_metrics_steps.csv")
        steps_csv = metrics_csv.with_name(steps_name)
        if steps_csv.is_file():
            return steps_csv
    return None


def _load_seed_series(
    summary_csv: pathlib.Path, frame_mode: str, threshold: float
) -> tuple[list[int], list[float], list[float], list[int]]:
    seeds: list[int] = []
    masked_max_tracking_error: list[float] = []
    global_min_h: list[float] = []
    active_rows_per_seed: list[int] = []

    with summary_csv.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))

    filtered: list[tuple[int, dict[str, str]]] = []
    for row in rows:
        if row.get("frame_mode", "").strip() != frame_mode:
            continue
        if str(row.get("metrics_found", "")).strip() not in {"1", "true", "True"}:
            continue
        seed = _safe_int(row.get("seed"))
        if seed is None:
            continue
        filtered.append((seed, row))

    filtered.sort(key=lambda item: item[0])
    if not filtered:
        raise SystemExit(
            f"no valid rows found in {summary_csv} for frame_mode={frame_mode}"
        )

    for seed, row in filtered:
        steps_csv = _resolve_steps_csv(row.get("metrics_csv", ""))
        if steps_csv is None:
            continue

        active_rows = 0
        max_tracking = -math.inf
        min_h = math.inf

        with steps_csv.open("r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            for step_row in reader:
                solver_h = _safe_float(step_row.get("solver_h"))
                tracking_error = _safe_float(step_row.get("tracking_error"))
                if math.isfinite(solver_h):
                    min_h = min(min_h, solver_h)
                    if solver_h < threshold:
                        active_rows += 1
                        if math.isfinite(tracking_error):
                            max_tracking = max(max_tracking, tracking_error)

        seeds.append(seed)
        active_rows_per_seed.append(active_rows)
        masked_max_tracking_error.append(
            math.nan if max_tracking == -math.inf else max_tracking
        )
        global_min_h.append(math.nan if min_h == math.inf else min_h)

    return seeds, masked_max_tracking_error, global_min_h, active_rows_per_seed


def _default_output_path(summary_csv: pathlib.Path, frame_mode: str) -> pathlib.Path:
    stem = summary_csv.stem
    return summary_csv.with_name(
        f"{stem}_{_sanitize_filename(frame_mode)}_state_mask_scatter.png"
    )


def _add_confidence_ellipse(
    ax: plt.Axes,
    x_vals: list[float],
    y_vals: list[float],
    color: str,
) -> None:
    finite = np.asarray(
        [
            (x, y)
            for x, y in zip(x_vals, y_vals)
            if math.isfinite(x) and math.isfinite(y)
        ],
        dtype=float,
    )
    if finite.shape[0] < 2:
        return
    cov = np.cov(finite[:, 0], finite[:, 1])
    if cov.shape != (2, 2) or not np.all(np.isfinite(cov)):
        return
    eigvals, eigvecs = np.linalg.eigh(cov)
    if np.any(eigvals <= 0.0):
        return
    order = np.argsort(eigvals)[::-1]
    eigvals = eigvals[order]
    eigvecs = eigvecs[:, order]
    chi2_95 = 5.991464547107979
    width = 2.0 * math.sqrt(eigvals[0] * chi2_95)
    height = 2.0 * math.sqrt(eigvals[1] * chi2_95)
    angle = math.degrees(math.atan2(eigvecs[1, 0], eigvecs[0, 0]))
    center = finite.mean(axis=0)
    ellipse = Ellipse(
        xy=(center[0], center[1]),
        width=width,
        height=height,
        angle=angle,
        facecolor=color,
        edgecolor=color,
        alpha=0.16,
        linewidth=1.8,
        zorder=1.0,
        label="_nolegend_",
    )
    ax.add_patch(ellipse)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot seed-wise masked max tracking error and global min h(x) from "
            "seed_metrics_summary.csv."
        )
    )
    parser.add_argument(
        "summary",
        help="seed_metrics_summary.csv path or its parent run directory",
    )
    parser.add_argument(
        "--frame-mode",
        default="noninertial",
        choices=["noninertial", "inertial", "both"],
        help="Which frame_mode rows to plot",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.0,
        help="Active mask threshold applied to solver_h",
    )
    parser.add_argument(
        "--title",
        default="",
        help="Optional custom figure title",
    )
    parser.add_argument(
        "--out",
        default="",
        help="Optional output PNG path",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show the figure interactively in addition to saving it",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary_csv = _resolve_summary_csv(args.summary)

    modes = (
        ["noninertial", "inertial"]
        if args.frame_mode == "both"
        else [args.frame_mode]
    )
    series_by_mode: dict[str, tuple[list[int], list[float], list[float], list[int]]] = {}
    for mode in modes:
        series_by_mode[mode] = _load_seed_series(summary_csv, mode, args.threshold)

    out_path = (
        pathlib.Path(args.out).expanduser().resolve()
        if args.out
        else _default_output_path(summary_csv, args.frame_mode)
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(1, 1, figsize=(8.8, 6.6), constrained_layout=True)

    mode_colors = {
        "noninertial": "#1f77b4",
        "inertial": "#d62728",
    }

    summary_parts: list[str] = []
    for mode in modes:
        seeds, tracking_vals, min_h_vals, active_rows = series_by_mode[mode]
        summary_parts.append(
            f"{mode}:samples={len(tracking_vals)}"
            f"/finite_tracking={sum(math.isfinite(v) for v in tracking_vals)}"
            f"/finite_min_h={sum(math.isfinite(v) for v in min_h_vals)}"
            f"/active_rows_total={sum(active_rows)}"
        )
    labels = {
        "noninertial": "Non-inertial Frozen",
        "inertial": "Inertial Frozen",
    }

    for mode in modes:
        _, tracking_vals, min_h_vals, _ = series_by_mode[mode]
        color = mode_colors[mode]
        ax.scatter(
            min_h_vals,
            tracking_vals,
            s=56,
            color=color,
            alpha=0.88,
            edgecolors="white",
            linewidths=0.7,
            label=labels[mode],
            zorder=3.0,
        )
        _add_confidence_ellipse(
            ax,
            min_h_vals,
            tracking_vals,
            color,
        )

    ax.axvline(0.0, color="#c62828", linewidth=1.8, linestyle="--", alpha=0.95, zorder=2.0)
    ax.text(
        0.0,
        0.97,
        "CBF boundary",
        transform=ax.get_xaxis_transform(),
        rotation=90,
        color="#b71c1c",
        va="top",
        ha="left",
        fontsize=12,
        bbox=dict(boxstyle="round,pad=0.20", facecolor="white", edgecolor="#b71c1c", alpha=0.94),
    )
    ax.set_xlabel(r"$h_{\min}$")
    ax.set_ylabel(r"$e_{\max}\ \mathrm{[m]}$")
    ax.grid(True, alpha=0.22, linestyle="--")
    ax.legend(
        loc="best",
        framealpha=0.96,
        fancybox=False,
        edgecolor="black",
        fontsize=12,
    )
    if args.title:
        ax.set_title(args.title, fontsize=15)
    fig.savefig(out_path, dpi=220)

    print(f"saved plot: {out_path}")
    print(
        f"frame_mode={args.frame_mode} threshold={args.threshold:.6f} "
        + " ".join(summary_parts)
    )

    if args.show:
        plt.show()
    else:
        plt.close("all")
    return 0


if __name__ == "__main__":
    sys.exit(main())
