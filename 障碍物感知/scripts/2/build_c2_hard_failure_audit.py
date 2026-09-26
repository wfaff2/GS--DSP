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


def load_cycle_metrics(step_csv: Path) -> pd.DataFrame:
    df = pd.read_csv(step_csv)
    for col in ("step_idx", "sim_time", "core_time_ms", "solver_fail_flag"):
        df[col] = pd.to_numeric(df[col], errors="coerce")
    grouped = (
        df.groupby("step_idx", as_index=False)
        .agg(
            sim_time=("sim_time", "first"),
            core_time_ms=("core_time_ms", "max"),
            solver_fail_flag=("solver_fail_flag", "max"),
        )
        .sort_values("step_idx")
    )
    return grouped


def summarize_seed(step_csv: Path, threshold_ms: float) -> dict[str, float]:
    cycle_df = load_cycle_metrics(step_csv)
    if cycle_df.empty:
        return {
            "total_cycles": 0,
            "overlimit_cycles": 0,
            "fail_cycles": 0,
            "overlimit_ratio": math.nan,
            "fail_ratio": math.nan,
            "max_core_time_ms": math.nan,
        }
    overlimit = cycle_df["core_time_ms"] > threshold_ms
    fail = cycle_df["solver_fail_flag"] > 0.5
    total = int(len(cycle_df))
    return {
        "total_cycles": total,
        "overlimit_cycles": int(overlimit.sum()),
        "fail_cycles": int(fail.sum()),
        "overlimit_ratio": float(overlimit.mean()),
        "fail_ratio": float(fail.mean()),
        "max_core_time_ms": float(cycle_df["core_time_ms"].max()),
    }


def build_dataframe(run_root: Path, threshold_ms: float) -> pd.DataFrame:
    rows: list[dict[str, float | int | str]] = []
    for seed_dir in sorted(p for p in run_root.glob("seed_*") if p.is_dir()):
        try:
            seed = int(seed_dir.name.split("_", 1)[1])
        except Exception:
            continue
        files = {
            "stage": seed_dir / "noninertial_metrics_steps.csv",
            "frozen": seed_dir / "noninertial_frozen_metrics_steps.csv",
        }
        for method, path in files.items():
            if not path.is_file():
                continue
            stats = summarize_seed(path, threshold_ms)
            rows.append(
                {
                    "seed": seed,
                    "method": method,
                    "threshold_ms": threshold_ms,
                    **stats,
                }
            )
    return pd.DataFrame(rows).sort_values(["seed", "method"]).reset_index(drop=True)


def plot_bars(df: pd.DataFrame, out_path: Path, threshold_ms: float) -> None:
    methods = ["stage", "frozen"]
    labels = {"stage": "Stage-wise", "frozen": "Frozen"}
    colors = {"stage": "#1f77b4", "frozen": "#d62728"}
    seeds = sorted(df["seed"].unique())
    x = np.arange(len(seeds), dtype=float)
    width = 0.36

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True, constrained_layout=True)
    for ax, column, title, ylabel in [
        (axes[0], "overlimit_cycles", f"Over-limit Control Cycles (> {threshold_ms:.1f} ms)", "cycle count"),
        (axes[1], "fail_cycles", "Solver Failure Control Cycles", "cycle count"),
    ]:
        for idx, method in enumerate(methods):
            method_df = (
                df[df["method"] == method]
                .set_index("seed")
                .reindex(seeds)
            )
            offset = (-0.5 + idx) * width
            ax.bar(
                x + offset,
                method_df[column].fillna(0.0).to_numpy(),
                width=width,
                color=colors[method],
                alpha=0.78,
                label=labels[method],
            )
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.25, linestyle="--")
        ax.legend(framealpha=0.92)

    axes[1].set_xlabel("seed")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels([str(seed) for seed in seeds], rotation=0)
    fig.suptitle("C2 Hard-Failure Audit: Non-inertial Stage-wise vs Frozen", fontsize=15)
    fig.savefig(out_path, dpi=220)
    plt.close(fig)


def write_summary(df: pd.DataFrame, out_path: Path, figure_name: str, threshold_ms: float) -> None:
    def fmt(value: float) -> str:
        return "nan" if not math.isfinite(value) else f"{value:.6f}"

    lines = [
        "# C2 Hard-Failure Audit",
        "",
        "- Comparison: `non-inertial stage-wise` vs `non-inertial frozen`",
        f"- Red-line threshold: `core_time_ms > {threshold_ms:.3f}`",
        "- Control-cycle audit is aggregated by `step_idx` and uses the maximum `core_time_ms` across UAV solves within that cycle.",
        "- `core_time_ms = feedback_time_ms + preparation_time_ms`.",
        "",
        "## Aggregate",
        "",
    ]

    for method in ("frozen", "stage"):
        part = df[df["method"] == method]
        lines.extend(
            [
                f"### {'Frozen' if method == 'frozen' else 'Stage-wise'}",
                "",
                f"- Seeds: `{len(part)}`",
                f"- Total over-limit cycles: `{int(part['overlimit_cycles'].sum())}`",
                f"- Mean over-limit cycles per seed: `{fmt(part['overlimit_cycles'].mean())}`",
                f"- Total solver-fail cycles: `{int(part['fail_cycles'].sum())}`",
                f"- Mean solver-fail cycles per seed: `{fmt(part['fail_cycles'].mean())}`",
                f"- Mean max core time per seed: `{fmt(part['max_core_time_ms'].mean())} ms`",
                "",
            ]
        )

    lines.extend(
        [
            "## Figure",
            "",
            f"![c2_hard_failure_audit]({figure_name})",
            "",
        ]
    )
    out_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build C2 hard-failure audit plots.")
    parser.add_argument("run_root", type=Path, help="C2 batch result directory")
    parser.add_argument("--out-dir", type=Path, default=None, help="Output directory")
    parser.add_argument("--threshold-ms", type=float, default=5.0, help="Red-line threshold on core_time_ms")
    args = parser.parse_args()

    run_root = args.run_root.expanduser().resolve()
    out_dir = (args.out_dir or run_root).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    df = build_dataframe(run_root, args.threshold_ms)
    if df.empty:
        raise SystemExit(f"No valid C2 step files found under {run_root}")

    csv_path = out_dir / "c2_hard_failure_audit.csv"
    fig_path = out_dir / "c2_hard_failure_audit.png"
    md_path = out_dir / "c2_hard_failure_audit.md"

    df.to_csv(csv_path, index=False)
    plot_bars(df, fig_path, args.threshold_ms)
    write_summary(df, md_path, fig_path.name, args.threshold_ms)

    print(f"[INFO] c2 hard-failure csv: {csv_path}")
    print(f"[INFO] c2 hard-failure plot: {fig_path}")
    print(f"[INFO] c2 hard-failure summary: {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
