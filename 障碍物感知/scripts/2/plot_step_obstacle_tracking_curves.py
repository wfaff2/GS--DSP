#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
import math
import pathlib
import re
import sys

import matplotlib


def _place_figure_legend(fig, handles, labels, ncol=2, y=0.99):
    if not handles:
        return
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, y),
        ncol=ncol,
        frameon=True,
        columnspacing=1.2,
        handlelength=2.2,
        borderaxespad=0.2,
    )


def _place_axes_legend(ax, ncol=2, y=1.08):
    handles, labels = ax.get_legend_handles_labels()
    if not handles:
        return
    ax.legend(
        handles,
        labels,
        loc="lower center",
        bbox_to_anchor=(0.5, y),
        ncol=ncol,
        framealpha=0.92,
        columnspacing=1.2,
        handlelength=2.2,
        borderaxespad=0.2,
    )


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


def _safe_int(text: str | None) -> int | None:
    if text is None:
        return None
    stripped = str(text).strip()
    if not stripped:
        return None
    try:
        return int(stripped)
    except ValueError:
        return None


def _mean_finite(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan
    return sum(finite) / len(finite)


def _is_aggregate_selector(selector: str) -> bool:
    return selector in {
        "all",
        "followers",
        "all_no0",
        "all_except_0",
        "all_including_uav0",
        "all_with_0",
    }


def _include_uav_for_selector(selector: str, uav_idx: int) -> bool:
    if selector in {"all_including_uav0", "all_with_0"}:
        return True
    if selector in {"all", "followers", "all_no0", "all_except_0"}:
        return uav_idx != 0
    return False


def _sanitize_filename(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", text)


def _default_series_label(csv_path: pathlib.Path) -> str:
    stem = csv_path.stem
    for suffix in ("_metrics_steps", "_steps", "_metrics"):
        if stem.endswith(suffix):
            stem = stem[: -len(suffix)]
            break
    return stem or csv_path.stem


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


def _default_output_path(
    csv_path: pathlib.Path, run_tag: str, uav_label: str, distance_field: str
) -> pathlib.Path:
    stem = csv_path.stem
    return csv_path.with_name(
        f"{stem}_{_sanitize_filename(run_tag)}_uav{_sanitize_filename(uav_label)}_{distance_field}.png"
    )


def _default_tracking_output_path(
    csv_path: pathlib.Path, run_tag: str, uav_label: str
) -> pathlib.Path:
    stem = csv_path.stem
    return csv_path.with_name(
        f"{stem}_{_sanitize_filename(run_tag)}_uav{_sanitize_filename(uav_label)}_tracking_error.png"
    )


def _default_compare_output_path(
    csv_a: pathlib.Path,
    run_tag_a: str,
    csv_b: pathlib.Path,
    run_tag_b: str,
    uav_label: str,
    distance_field: str,
) -> pathlib.Path:
    stem_a = _sanitize_filename(csv_a.stem)
    stem_b = _sanitize_filename(csv_b.stem)
    tag_a = _sanitize_filename(run_tag_a)
    tag_b = _sanitize_filename(run_tag_b)
    return csv_a.with_name(
        f"{stem_a}_{tag_a}_vs_{stem_b}_{tag_b}_uav{_sanitize_filename(uav_label)}_{distance_field}_compare.png"
    )


def _default_tracking_compare_output_path(
    csv_a: pathlib.Path,
    run_tag_a: str,
    csv_b: pathlib.Path,
    run_tag_b: str,
    uav_label: str,
) -> pathlib.Path:
    stem_a = _sanitize_filename(csv_a.stem)
    stem_b = _sanitize_filename(csv_b.stem)
    tag_a = _sanitize_filename(run_tag_a)
    tag_b = _sanitize_filename(run_tag_b)
    return csv_a.with_name(
        f"{stem_a}_{tag_a}_vs_{stem_b}_{tag_b}_uav{_sanitize_filename(uav_label)}_tracking_error_compare.png"
    )


def _load_series(
    csv_path: pathlib.Path,
    requested_run_tag: str | None,
    uav_selector: str,
    distance_field: str,
) -> tuple[str, list[float], list[float], list[float], int]:
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"CSV is empty: {csv_path}")

    run_tag = _resolve_run_tag(rows, requested_run_tag)
    available_columns = set(rows[0].keys())
    effective_distance_field = distance_field
    if (
        distance_field == "solver_planar_surface_distance"
        and distance_field not in available_columns
        and "solver_planar_clearance" in available_columns
    ):
        effective_distance_field = "solver_planar_clearance"
    normalized_selector = (uav_selector or "0").strip().lower()
    if _is_aggregate_selector(normalized_selector):
        grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
        for row in rows:
            if row.get("run_tag") != run_tag:
                continue
            row_uav = _safe_int(row.get("uav_idx"))
            if row_uav is None:
                continue
            if not _include_uav_for_selector(normalized_selector, row_uav):
                continue
            sim_time_key = str(row.get("sim_time", "")).strip()
            if not sim_time_key:
                continue
            grouped[sim_time_key].append(row)
        if not grouped:
            raise SystemExit(
                f"No rows found for run_tag={run_tag!r} uav_idx={normalized_selector} in {csv_path}"
            )
        ordered_keys = sorted(grouped.keys(), key=lambda text: _safe_float(text))
        times: list[float] = []
        obstacle_metric: list[float] = []
        tracking_error: list[float] = []
        row_count = 0
        for key in ordered_keys:
            bucket = grouped[key]
            times.append(_safe_float(key))
            obstacle_metric.append(
                _mean_finite(
                    [_safe_float(row.get(effective_distance_field)) for row in bucket]
                )
            )
            tracking_error.append(
                _mean_finite([_safe_float(row.get("tracking_error")) for row in bucket])
            )
            row_count += len(bucket)
        return run_tag, times, obstacle_metric, tracking_error, row_count

    uav_idx = _safe_int(normalized_selector)
    if uav_idx is None:
        raise SystemExit(
            "Unsupported --uav value "
            f"{uav_selector!r}. Use an integer UAV index, "
            "'all'/'followers' (exclude UAV0), or 'all_including_uav0'."
        )
    selected: list[dict[str, str]] = []
    for row in rows:
        if row.get("run_tag") != run_tag:
            continue
        row_uav = _safe_int(row.get("uav_idx"))
        if row_uav != uav_idx:
            continue
        selected.append(row)

    if not selected:
        raise SystemExit(
            f"No rows found for run_tag={run_tag!r} uav_idx={uav_idx} in {csv_path}"
        )

    selected.sort(
        key=lambda row: (
            _safe_float(row.get("sim_time")),
            _safe_int(row.get("step_idx")) or -1,
        )
    )
    times = [_safe_float(row.get("sim_time")) for row in selected]
    obstacle_metric = [_safe_float(row.get(effective_distance_field)) for row in selected]
    tracking_error = [_safe_float(row.get("tracking_error")) for row in selected]
    return run_tag, times, obstacle_metric, tracking_error, len(selected)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot sim_time vs solver-active obstacle distance metric and tracking "
            "error from a per-step CSV exported by num_sim_non_one_point_node."
        )
    )
    parser.add_argument("--csv", required=True, help="Path to *_steps.csv")
    parser.add_argument("--run-tag", default="", help="Filter by run_tag")
    parser.add_argument(
        "--uav",
        default="0",
        help=(
            "UAV index to plot, 'all'/'followers' for the mean across UAV1+, "
            "or 'all_including_uav0' for the mean across all UAVs."
        ),
    )
    parser.add_argument("--label", default="", help="Primary series label")
    parser.add_argument(
        "--compare-csv",
        default="",
        help="Optional second *_steps.csv to overlay in a compare figure",
    )
    parser.add_argument(
        "--compare-run-tag",
        default="",
        help="Optional run_tag for the second CSV",
    )
    parser.add_argument(
        "--compare-uav",
        default="",
        help=(
            "UAV selector for the second CSV; default uses --uav. "
            "Supports the same values as --uav."
        ),
    )
    parser.add_argument("--compare-label", default="", help="Second series label")
    parser.add_argument(
        "--distance-field",
        default="solver_planar_surface_distance",
        choices=[
            "solver_planar_surface_distance",
            "solver_planar_clearance",
            "solver_planar_distance",
        ],
        help=(
            "Obstacle-distance curve to plot. "
            "`solver_planar_surface_distance` is center distance minus physical radii; "
            "`solver_planar_clearance` is solver-effective clearance; "
            "`solver_planar_distance` is center distance."
        ),
    )
    parser.add_argument(
        "--figure-mode",
        default="combined",
        choices=["combined", "tracking_only"],
        help=(
            "combined: obstacle metric + tracking error; "
            "tracking_only: only draw the tracking error curve(s)."
        ),
    )
    parser.add_argument("--title", default="", help="Optional figure title")
    parser.add_argument("--out", default="", help="Output PNG path")
    parser.add_argument(
        "--show",
        action="store_true",
        help="Also open an interactive plot window after saving.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    csv_path = pathlib.Path(args.csv).expanduser().resolve()
    if not csv_path.is_file():
        raise SystemExit(f"CSV not found: {csv_path}")

    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    run_tag, times, obstacle_metric, tracking_error, row_count = _load_series(
        csv_path, args.run_tag or None, args.uav, args.distance_field
    )

    output_path: pathlib.Path
    obstacle_label = (
        "solver active obstacle planar surface distance"
        if args.distance_field == "solver_planar_surface_distance"
        else (
            "solver active obstacle planar clearance"
            if args.distance_field == "solver_planar_clearance"
            else "solver active obstacle planar distance"
        )
    )
    primary_label = args.label or _default_series_label(csv_path)

    if args.compare_csv:
        compare_csv_path = pathlib.Path(args.compare_csv).expanduser().resolve()
        if not compare_csv_path.is_file():
            raise SystemExit(f"compare CSV not found: {compare_csv_path}")
        compare_uav = args.compare_uav.strip() if args.compare_uav.strip() else args.uav
        (
            compare_run_tag,
            compare_times,
            compare_obstacle_metric,
            compare_tracking_error,
            compare_row_count,
        ) = _load_series(
            compare_csv_path,
            args.compare_run_tag or None,
            compare_uav,
            args.distance_field,
        )
        compare_label = args.compare_label or _default_series_label(compare_csv_path)
        output_path = (
            pathlib.Path(args.out).expanduser().resolve()
            if args.out
            else (
                _default_tracking_compare_output_path(
                    csv_path,
                    run_tag,
                    compare_csv_path,
                    compare_run_tag,
                    args.uav,
                )
                if args.figure_mode == "tracking_only"
                else _default_compare_output_path(
                    csv_path,
                    run_tag,
                    compare_csv_path,
                    compare_run_tag,
                    args.uav,
                    args.distance_field,
                )
            )
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)

        if args.figure_mode == "tracking_only":
            fig, ax = plt.subplots(figsize=(11.5, 4.8))
            primary_handle = ax.plot(times, tracking_error, linewidth=1.9, label=primary_label)[0]
            compare_handle = ax.plot(
                compare_times,
                compare_tracking_error,
                linewidth=1.9,
                linestyle="--",
                label=compare_label,
            )[0]
            ax.set_xlabel("sim_time [s]")
            ax.set_ylabel("tracking error [m]")
            ax.grid(True, alpha=0.3)
        else:
            fig, axes = plt.subplots(2, 1, figsize=(11.5, 7.2), sharex=True)
            ax_top, ax_bottom = axes
            primary_handle = ax_top.plot(
                times, obstacle_metric, linewidth=1.9, label=primary_label
            )[0]
            compare_handle = ax_top.plot(
                compare_times,
                compare_obstacle_metric,
                linewidth=1.9,
                linestyle="--",
                label=compare_label,
            )[0]
            ax_top.set_ylabel(f"{obstacle_label} [m]")
            ax_top.grid(True, alpha=0.3)

            ax_bottom.plot(times, tracking_error, linewidth=1.9)
            ax_bottom.plot(
                compare_times,
                compare_tracking_error,
                linewidth=1.9,
                linestyle="--",
            )
            ax_bottom.set_xlabel("sim_time [s]")
            ax_bottom.set_ylabel("tracking error [m]")
            ax_bottom.grid(True, alpha=0.3)
            _place_figure_legend(
                fig,
                [primary_handle, compare_handle],
                [primary_label, compare_label],
                ncol=2,
                y=0.992,
            )
        fig.suptitle(
            args.title
            or (
                (
                    "tracking error comparison | "
                    if args.figure_mode == "tracking_only"
                    else f"{obstacle_label} and tracking error comparison | "
                )
                + f"{primary_label} (run_tag={run_tag}, uav={args.uav}) vs "
                + f"{compare_label} (run_tag={compare_run_tag}, uav={compare_uav})"
            ),
            y=0.935,
        )
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.88), h_pad=2.2)
        fig.savefig(output_path, dpi=180)

        print(f"saved compare plot: {output_path}")
        print(
            f"primary rows={row_count} run_tag={run_tag} uav={args.uav} "
            f"secondary rows={compare_row_count} run_tag={compare_run_tag} uav={compare_uav}"
        )
    else:
        output_path = (
            pathlib.Path(args.out).expanduser().resolve()
            if args.out
            else (
                _default_tracking_output_path(csv_path, run_tag, args.uav)
                if args.figure_mode == "tracking_only"
                else _default_output_path(
                    csv_path,
                    run_tag,
                    args.uav,
                    args.distance_field,
                )
            )
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)

        fig, ax = plt.subplots(figsize=(11, 5.5))
        if args.figure_mode == "tracking_only":
            ax.plot(times, tracking_error, linewidth=1.8, label="tracking error")
        else:
            ax.plot(times, obstacle_metric, linewidth=1.8, label=obstacle_label)
            ax.plot(times, tracking_error, linewidth=1.8, label="tracking error")
        ax.set_xlabel("sim_time [s]")
        ax.set_ylabel("error [m]")
        ax.grid(True, alpha=0.3)
        _place_axes_legend(ax, ncol=2, y=1.08)
        ax.set_title(
            args.title
            or (
                f"run_tag={run_tag} | uav={args.uav} | tracking_error"
                if args.figure_mode == "tracking_only"
                else f"run_tag={run_tag} | uav={args.uav} | {args.distance_field} vs tracking_error"
            ),
            pad=12,
        )
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.975), h_pad=2.0)
        fig.savefig(output_path, dpi=180)

        finite_obstacle = sum(1 for value in obstacle_metric if math.isfinite(value))
        finite_tracking = sum(1 for value in tracking_error if math.isfinite(value))
        print(f"saved plot: {output_path}")
        print(
            f"rows={row_count} run_tag={run_tag} uav={args.uav} "
            f"finite_{args.distance_field}={finite_obstacle} "
            f"finite_tracking_error={finite_tracking}"
        )

    if args.show:
        plt.show()
    else:
        plt.close("all")
    return 0


if __name__ == "__main__":
    sys.exit(main())
