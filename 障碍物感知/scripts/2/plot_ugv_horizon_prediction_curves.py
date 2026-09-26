#!/usr/bin/env python3
"""Plot UGV predicted-vs-actual horizon error curves from summary CSVs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib


def _safe_float(text: str | None) -> float:
    if text is None:
        return math.nan
    stripped = str(text).strip()
    if not stripped:
        return math.nan
    try:
        return float(stripped)
    except ValueError:
        return math.nan


def _resolve_run_tag(rows: list[dict[str, str]], requested_run_tag: str | None) -> str:
    run_tags = sorted({row.get("run_tag", "") for row in rows if row.get("run_tag", "")})
    if requested_run_tag:
        if requested_run_tag not in run_tags:
            raise SystemExit(
                f"run_tag {requested_run_tag!r} not found in CSV. "
                f"Available run_tags: {', '.join(run_tags[:10])}"
            )
        return requested_run_tag
    if len(run_tags) == 1:
        return run_tags[0]
    raise SystemExit(
        "CSV contains multiple run_tag values. Pass --run-tag explicitly. "
        f"Available run_tags: {', '.join(run_tags[:10])}"
    )


def _load_series(
    csv_path: Path,
    requested_run_tag: str | None,
    metric: str,
) -> tuple[str, list[float], list[float], int]:
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"CSV is empty: {csv_path}")

    run_tag = _resolve_run_tag(rows, requested_run_tag)
    selected = [row for row in rows if row.get("run_tag") == run_tag]
    selected.sort(
        key=lambda row: (
            _safe_float(row.get("sim_time")),
            _safe_float(row.get("step_idx")),
        )
    )

    times: list[float] = []
    values: list[float] = []
    for row in selected:
        sim_time = _safe_float(row.get("sim_time"))
        value = _safe_float(row.get(metric))
        if math.isfinite(sim_time) and math.isfinite(value):
            times.append(sim_time)
            values.append(value)

    if not times:
        raise SystemExit(
            f"No finite sim_time/{metric} rows found for run_tag={run_tag!r} in {csv_path}"
        )
    return run_tag, times, values, len(values)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot sim_time vs UGV predicted-vs-actual horizon error metric "
            "from summary CSVs exported by num_sim_non_one_point_node."
        )
    )
    parser.add_argument("--csv", required=True, help="Path to primary *_ugv_horizon_summary.csv")
    parser.add_argument("--run-tag", default="", help="Filter primary CSV by run_tag")
    parser.add_argument("--label", default="", help="Primary series label")
    parser.add_argument("--compare-csv", required=True, help="Path to secondary summary CSV")
    parser.add_argument("--compare-run-tag", default="", help="Filter secondary CSV by run_tag")
    parser.add_argument("--compare-label", default="", help="Secondary series label")
    parser.add_argument(
        "--metric",
        default="horizon_rms_xy",
        choices=["horizon_rms_xy", "horizon_max_xy", "horizon_final_xy"],
        help="Which UGV horizon error metric to plot.",
    )
    parser.add_argument("--title", default="", help="Optional figure title")
    parser.add_argument("--out", default="", help="Output PNG path")
    parser.add_argument("--show", action="store_true", help="Also open an interactive plot window")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    csv_path = Path(args.csv).expanduser().resolve()
    compare_csv_path = Path(args.compare_csv).expanduser().resolve()
    if not csv_path.is_file():
        raise FileNotFoundError(f"Primary CSV not found: {csv_path}")
    if not compare_csv_path.is_file():
        raise FileNotFoundError(f"Secondary CSV not found: {compare_csv_path}")

    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    run_tag, times, values, row_count = _load_series(
        csv_path,
        args.run_tag or None,
        args.metric,
    )
    compare_run_tag, compare_times, compare_values, compare_row_count = _load_series(
        compare_csv_path,
        args.compare_run_tag or None,
        args.metric,
    )

    label = args.label or "primary"
    compare_label = args.compare_label or "secondary"
    title = args.title or f"{args.metric} vs sim_time"
    out_path = (
        Path(args.out).expanduser().resolve()
        if args.out
        else csv_path.with_suffix(".png")
    )

    fig, ax = plt.subplots(figsize=(11.0, 4.8))
    ax.plot(times, values, linewidth=2.0, label=label)
    ax.plot(compare_times, compare_values, linewidth=2.0, label=compare_label)
    ax.set_xlabel("sim_time (s)")
    ax.set_ylabel(args.metric)
    ax.set_title(title, pad=12)
    ax.grid(True, alpha=0.3)
    ax.legend(
        loc="lower center",
        bbox_to_anchor=(0.5, 1.08),
        ncol=2,
        framealpha=0.92,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.975), h_pad=2.0)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180)
    if args.show:
        plt.show()
    else:
        plt.close(fig)

    print(f"saved compare plot: {out_path}")
    print(
        f"primary rows={row_count} run_tag={run_tag} "
        f"secondary rows={compare_row_count} run_tag={compare_run_tag}"
    )
    print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
