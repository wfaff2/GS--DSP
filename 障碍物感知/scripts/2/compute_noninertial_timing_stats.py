#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import pandas as pd

STEP_SUFFIX = "_metrics_steps.csv"
REQUIRED_COLUMNS = [
    "frame_mode_effective",
    "ugv_rollout_mode",
    "solver_cbf_active",
    "feedback_time_ms",
    "preparation_time_ms",
    "core_time_ms",
]


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


def canonical_method(frame_mode: str, rollout_mode: str) -> str | None:
    if frame_mode != "noninertial":
        return None
    if rollout_mode == "frozen":
        return "noninertial_frozen"
    if rollout_mode == "stage":
        return "noninertial_stage"
    return None


def load_seed_filter(path: Path | None) -> dict[float, set[int]]:
    if path is None:
        return {}
    df = pd.read_csv(path)
    if not {"kp", "seed"}.issubset(df.columns):
        raise RuntimeError(f"Seed filter CSV must contain kp, seed columns: {path}")
    keep: dict[float, set[int]] = {}
    for kp, dkp in df.groupby("kp"):
        keep[float(kp)] = set(int(seed) for seed in dkp["seed"].tolist())
    return keep


def discover_step_files(root: Path) -> list[Path]:
    files: list[Path] = []
    for path in sorted(root.rglob(f"seed_*/*{STEP_SUFFIX}")):
        if path.name.endswith("_ugv_horizon_steps.csv"):
            continue
        files.append(path)
    return files


def load_rows(root: Path, seed_filter: dict[float, set[int]]) -> pd.DataFrame:
    frames: list[pd.DataFrame] = []
    for path in discover_step_files(root):
        kp = infer_kp(path)
        seed = infer_seed(path)
        if kp is None or seed is None:
            continue
        if seed_filter and seed not in seed_filter.get(kp, set()):
            continue

        df = pd.read_csv(path, usecols=REQUIRED_COLUMNS)
        method = canonical_method(
            str(df["frame_mode_effective"].iloc[0]),
            str(df["ugv_rollout_mode"].iloc[0]),
        )
        if method is None:
            continue

        df = df.copy()
        df["method"] = method
        df["kp"] = kp
        df["seed"] = seed
        df["t_prep_ms"] = pd.to_numeric(df["preparation_time_ms"], errors="coerce")
        df["t_solve_ms"] = pd.to_numeric(df["feedback_time_ms"], errors="coerce")
        core_time = pd.to_numeric(df["core_time_ms"], errors="coerce")
        df["t_total_ms"] = core_time
        missing_total = ~df["t_total_ms"].map(math.isfinite)
        df.loc[missing_total, "t_total_ms"] = (
            df.loc[missing_total, "t_prep_ms"] + df.loc[missing_total, "t_solve_ms"]
        )
        df["solver_cbf_active"] = pd.to_numeric(df["solver_cbf_active"], errors="coerce").fillna(0)
        df["cbf_slice"] = df["solver_cbf_active"].map(
            lambda value: "avoid" if int(value) > 0 else "no-avoid"
        )
        frames.append(df)

    if not frames:
        raise RuntimeError(f"No noninertial step CSV rows found under {root}")

    merged = pd.concat(frames, ignore_index=True)
    return merged.dropna(subset=["t_prep_ms", "t_solve_ms", "t_total_ms"]).copy()


def q99(series: pd.Series) -> float:
    return float(series.quantile(0.99))


def build_overall_summary(df: pd.DataFrame) -> pd.DataFrame:
    return (
        df.groupby("method", dropna=False)
        .agg(
            samples=("t_prep_ms", "size"),
            t_prep_mean_ms=("t_prep_ms", "mean"),
            t_prep_p99_ms=("t_prep_ms", q99),
            t_prep_max_ms=("t_prep_ms", "max"),
            t_solve_mean_ms=("t_solve_ms", "mean"),
            t_solve_p99_ms=("t_solve_ms", q99),
            t_solve_max_ms=("t_solve_ms", "max"),
            t_total_mean_ms=("t_total_ms", "mean"),
            t_total_p99_ms=("t_total_ms", q99),
            t_total_max_ms=("t_total_ms", "max"),
        )
        .reset_index()
    )


def build_solve_slice_summary(df: pd.DataFrame) -> pd.DataFrame:
    return (
        df.groupby(["method", "cbf_slice"], dropna=False)
        .agg(
            samples=("t_solve_ms", "size"),
            t_solve_mean_ms=("t_solve_ms", "mean"),
            t_solve_p99_ms=("t_solve_ms", q99),
            t_solve_max_ms=("t_solve_ms", "max"),
        )
        .reset_index()
    )


def build_total_slice_summary(df: pd.DataFrame) -> pd.DataFrame:
    return (
        df.groupby(["method", "cbf_slice"], dropna=False)
        .agg(
            samples=("t_total_ms", "size"),
            t_total_mean_ms=("t_total_ms", "mean"),
            t_total_p99_ms=("t_total_ms", q99),
            t_total_max_ms=("t_total_ms", "max"),
        )
        .reset_index()
    )


def format_number(value: object, digits: int = 4) -> str:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "nan"
    if not math.isfinite(numeric):
        return "nan"
    return f"{numeric:.{digits}f}"


def markdown_table(df: pd.DataFrame, numeric_columns: set[str]) -> str:
    headers = list(df.columns)
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["--:" if col in numeric_columns else ":--" for col in headers]) + " |",
    ]
    for row in df.itertuples(index=False):
        values: list[str] = []
        for column, value in zip(headers, row):
            if column in numeric_columns:
                if column == "samples":
                    values.append(str(int(value)))
                else:
                    values.append(format_number(value))
            else:
                values.append(str(value))
        lines.append("| " + " | ".join(values) + " |")
    return "\n".join(lines)


def build_markdown(
    root: Path,
    seed_filter_path: Path | None,
    overall: pd.DataFrame,
    solve_slice: pd.DataFrame,
    total_slice: pd.DataFrame,
) -> str:
    filter_note = (
        f"- Seed filter: `{seed_filter_path}`"
        if seed_filter_path is not None
        else "- Seed filter: none (all discovered seeds)"
    )
    lines = [
        "# Noninertial With-CBF Timing Summary",
        "",
        f"- Run root: `{root}`",
        filter_note,
        "- Time mapping: `t_prep = preparation_time_ms`, `t_solve = feedback_time_ms`, `t_total = core_time_ms`.",
        "- `t_solve` slices use the per-solve `solver_cbf_active` flag: `no-avoid = 0`, `avoid = 1`.",
        "",
        "## Overall Timing by Method",
        "",
        markdown_table(
            overall,
            {
                "samples",
                "t_prep_mean_ms",
                "t_prep_p99_ms",
                "t_prep_max_ms",
                "t_solve_mean_ms",
                "t_solve_p99_ms",
                "t_solve_max_ms",
                "t_total_mean_ms",
                "t_total_p99_ms",
                "t_total_max_ms",
            },
        ),
        "",
        "## Solve Time by CBF Slice",
        "",
        markdown_table(
            solve_slice,
            {"samples", "t_solve_mean_ms", "t_solve_p99_ms", "t_solve_max_ms"},
        ),
        "",
        "## Total Time by CBF Slice",
        "",
        markdown_table(
            total_slice,
            {"samples", "t_total_mean_ms", "t_total_p99_ms", "t_total_max_ms"},
        ),
        "",
    ]
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compute noninertial with-CBF timing summaries and export Markdown tables."
    )
    parser.add_argument("run_root", type=Path, help="Root directory containing kp_*/seed_* step CSVs.")
    parser.add_argument(
        "--seed-filter-csv",
        type=Path,
        default=None,
        help="Optional kp,seed CSV used to keep only a comparable seed subset.",
    )
    parser.add_argument(
        "--out-md",
        type=Path,
        default=None,
        help="Optional Markdown output path. Defaults to <run_root>/noninertial_timing_summary.md.",
    )
    parser.add_argument(
        "--out-overall-csv",
        type=Path,
        default=None,
        help="Optional CSV path for the overall by-method summary.",
    )
    parser.add_argument(
        "--out-solve-slice-csv",
        type=Path,
        default=None,
        help="Optional CSV path for the t_solve by-cbf-slice summary.",
    )
    parser.add_argument(
        "--out-total-slice-csv",
        type=Path,
        default=None,
        help="Optional CSV path for the t_total by-cbf-slice summary.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_root = args.run_root.resolve()
    seed_filter_path = args.seed_filter_csv.resolve() if args.seed_filter_csv else None
    seed_filter = load_seed_filter(seed_filter_path)
    df = load_rows(run_root, seed_filter)
    overall = build_overall_summary(df)
    solve_slice = build_solve_slice_summary(df)
    total_slice = build_total_slice_summary(df)

    overall = overall.sort_values("method").reset_index(drop=True)
    solve_slice["cbf_slice"] = pd.Categorical(
        solve_slice["cbf_slice"], categories=["no-avoid", "avoid"], ordered=True
    )
    solve_slice = solve_slice.sort_values(["method", "cbf_slice"]).reset_index(drop=True)
    total_slice["cbf_slice"] = pd.Categorical(
        total_slice["cbf_slice"], categories=["no-avoid", "avoid"], ordered=True
    )
    total_slice = total_slice.sort_values(["method", "cbf_slice"]).reset_index(drop=True)

    out_md = (
        args.out_md.resolve()
        if args.out_md
        else run_root / "noninertial_timing_summary.md"
    )
    out_overall_csv = (
        args.out_overall_csv.resolve()
        if args.out_overall_csv
        else run_root / "noninertial_timing_overall.csv"
    )
    out_solve_slice_csv = (
        args.out_solve_slice_csv.resolve()
        if args.out_solve_slice_csv
        else run_root / "noninertial_tsolve_by_cbf_slice.csv"
    )
    out_total_slice_csv = (
        args.out_total_slice_csv.resolve()
        if args.out_total_slice_csv
        else run_root / "noninertial_ttotal_by_cbf_slice.csv"
    )

    markdown = build_markdown(run_root, seed_filter_path, overall, solve_slice, total_slice)
    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_overall_csv.parent.mkdir(parents=True, exist_ok=True)
    out_solve_slice_csv.parent.mkdir(parents=True, exist_ok=True)
    out_total_slice_csv.parent.mkdir(parents=True, exist_ok=True)
    out_md.write_text(markdown, encoding="utf-8")
    overall.to_csv(out_overall_csv, index=False)
    solve_slice.to_csv(out_solve_slice_csv, index=False)
    total_slice.to_csv(out_total_slice_csv, index=False)

    print(markdown)
    print(f"[INFO] wrote {out_md}")
    print(f"[INFO] wrote {out_overall_csv}")
    print(f"[INFO] wrote {out_solve_slice_csv}")
    print(f"[INFO] wrote {out_total_slice_csv}")


if __name__ == "__main__":
    main()
