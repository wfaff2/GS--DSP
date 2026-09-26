#!/usr/bin/env python3
"""
Aggregate results/raw_runs.csv into results/compare_grid.csv.
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import sys
from collections import defaultdict
from typing import Dict, Iterable, List, Sequence, Tuple


def _safe_float(v: object) -> float:
    try:
        return float(v)  # type: ignore[arg-type]
    except Exception:
        return math.nan


def _safe_int(v: object, default: int = 0) -> int:
    try:
        return int(float(v))  # type: ignore[arg-type]
    except Exception:
        return default


def _is_finite(v: float) -> bool:
    return math.isfinite(v)


def _fmt_float(v: float) -> str:
    return "nan" if not _is_finite(v) else f"{v:.6f}"


def _quantile(values: Sequence[float], q: float) -> float:
    if not values:
        return math.nan
    vals = sorted(values)
    if len(vals) == 1:
        return vals[0]
    q = max(0.0, min(1.0, q))
    idx = q * (len(vals) - 1)
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return vals[lo]
    frac = idx - lo
    return vals[lo] * (1.0 - frac) + vals[hi] * frac


def _wilson_95(k: int, n: int) -> Tuple[float, float, float]:
    if n <= 0:
        return math.nan, math.nan, math.nan
    z = 1.959963984540054
    p = float(k) / float(n)
    denom = 1.0 + (z * z) / n
    center = (p + (z * z) / (2.0 * n)) / denom
    half = (z / denom) * math.sqrt((p * (1.0 - p) + (z * z) / (4.0 * n)) / n)
    low = max(0.0, center - half)
    high = min(1.0, center + half)
    return p, low, high


def _triplet(values: Iterable[float]) -> Tuple[float, float, float]:
    vals = [v for v in values if _is_finite(v)]
    if not vals:
        return math.nan, math.nan, math.nan
    return _quantile(vals, 0.50), _quantile(vals, 0.10), _quantile(vals, 0.90)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Aggregate raw_runs.csv into compare_grid.csv."
    )
    parser.add_argument("--raw-csv", default="results/raw_runs.csv")
    parser.add_argument("--out", default="results/compare_grid.csv")
    parser.add_argument("--clean", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    workspace_root = pathlib.Path(__file__).resolve().parent.parent
    raw_csv = workspace_root / args.raw_csv
    out_csv = workspace_root / args.out

    if not raw_csv.exists():
        print(f"ERROR: input not found: {raw_csv}", file=sys.stderr)
        return 2

    if args.clean and out_csv.exists():
        out_csv.unlink()
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    groups: Dict[Tuple[str, str, int, str], List[Dict[str, str]]] = defaultdict(list)
    with raw_csv.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                key = (
                    str(row["frame_mode"]),
                    str(row["safety_variant"]),
                    int(row["obs"]),
                    f"{float(row['kappa']):.6f}",
                )
            except Exception:
                continue
            groups[key].append(row)

    out_header = [
        "frame_mode",
        "safety_variant",
        "obs",
        "kappa",
        "n_runs",
        "P_col",
        "P_col_ci_low",
        "P_col_ci_high",
        "P_margin",
        "P_margin_ci_low",
        "P_margin_ci_high",
        "P_fail",
        "P_fail_ci_low",
        "P_fail_ci_high",
        "tracking_rms_median",
        "tracking_rms_p10",
        "tracking_rms_p90",
        "tracking_p95_median",
        "tracking_p95_p10",
        "tracking_p95_p90",
        "dmin_lin_median",
        "dmin_lin_p10",
        "dmin_lin_p90",
        "min_h_median",
        "min_h_p10",
        "min_h_p90",
        "min_cbf_median",
        "min_cbf_p10",
        "min_cbf_p90",
        "solve_time_mean_ms_median",
        "solve_time_mean_ms_p10",
        "solve_time_mean_ms_p90",
        "solve_time_p95_ms_median",
        "solve_time_p95_ms_p10",
        "solve_time_p95_ms_p90",
        "solve_time_max_ms_median",
        "solve_time_max_ms_p10",
        "solve_time_max_ms_p90",
        "slack_sum_median",
        "slack_sum_p10",
        "slack_sum_p90",
        "slack_max_median",
        "slack_max_p10",
        "slack_max_p90",
        "n_col",
        "n_margin",
        "n_fail",
    ]

    frame_order = {"noninertial": 0, "inertial": 1}
    safety_order = {"A0_no_cbf": 0, "A1_hard_cbf": 1, "A2_soft_cbf": 2}
    ordered_keys = sorted(
        groups.keys(),
        key=lambda k: (
            int(k[2]),
            float(k[3]),
            frame_order.get(k[0], 99),
            safety_order.get(k[1], 99),
        ),
    )

    rows_out: List[Dict[str, str]] = []
    for key in ordered_keys:
        frame_mode, safety_variant, obs, kappa = key
        rows = groups[key]
        n = len(rows)
        if n == 0:
            continue

        col_k = sum(1 for r in rows if _safe_int(r.get("collision_flag")) != 0)
        margin_k = sum(1 for r in rows if _safe_int(r.get("margin_flag")) != 0)
        fail_k = sum(1 for r in rows if _safe_int(r.get("fail_flag")) != 0)

        p_col, p_col_lo, p_col_hi = _wilson_95(col_k, n)
        p_margin, p_margin_lo, p_margin_hi = _wilson_95(margin_k, n)
        p_fail, p_fail_lo, p_fail_hi = _wilson_95(fail_k, n)

        tr_med, tr_p10, tr_p90 = _triplet(_safe_float(r.get("tracking_rms")) for r in rows)
        tp95_med, tp95_p10, tp95_p90 = _triplet(
            _safe_float(r.get("tracking_p95")) for r in rows
        )
        dlin_med, dlin_p10, dlin_p90 = _triplet(_safe_float(r.get("dmin_lin")) for r in rows)
        min_h_med, min_h_p10, min_h_p90 = _triplet(_safe_float(r.get("min_h")) for r in rows)
        min_cbf_med, min_cbf_p10, min_cbf_p90 = _triplet(
            _safe_float(r.get("min_cbf")) for r in rows
        )

        stm_med, stm_p10, stm_p90 = _triplet(
            _safe_float(r.get("solve_time_mean_ms")) for r in rows
        )
        stp95_med, stp95_p10, stp95_p90 = _triplet(
            _safe_float(r.get("solve_time_p95_ms")) for r in rows
        )
        stmax_med, stmax_p10, stmax_p90 = _triplet(
            _safe_float(r.get("solve_time_max_ms")) for r in rows
        )

        slack_sum_med, slack_sum_p10, slack_sum_p90 = _triplet(
            _safe_float(r.get("slack_sum")) for r in rows
        )
        slack_max_med, slack_max_p10, slack_max_p90 = _triplet(
            _safe_float(r.get("slack_max")) for r in rows
        )

        rows_out.append(
            {
                "frame_mode": frame_mode,
                "safety_variant": safety_variant,
                "obs": str(obs),
                "kappa": kappa,
                "n_runs": str(n),
                "P_col": _fmt_float(p_col),
                "P_col_ci_low": _fmt_float(p_col_lo),
                "P_col_ci_high": _fmt_float(p_col_hi),
                "P_margin": _fmt_float(p_margin),
                "P_margin_ci_low": _fmt_float(p_margin_lo),
                "P_margin_ci_high": _fmt_float(p_margin_hi),
                "P_fail": _fmt_float(p_fail),
                "P_fail_ci_low": _fmt_float(p_fail_lo),
                "P_fail_ci_high": _fmt_float(p_fail_hi),
                "tracking_rms_median": _fmt_float(tr_med),
                "tracking_rms_p10": _fmt_float(tr_p10),
                "tracking_rms_p90": _fmt_float(tr_p90),
                "tracking_p95_median": _fmt_float(tp95_med),
                "tracking_p95_p10": _fmt_float(tp95_p10),
                "tracking_p95_p90": _fmt_float(tp95_p90),
                "dmin_lin_median": _fmt_float(dlin_med),
                "dmin_lin_p10": _fmt_float(dlin_p10),
                "dmin_lin_p90": _fmt_float(dlin_p90),
                "min_h_median": _fmt_float(min_h_med),
                "min_h_p10": _fmt_float(min_h_p10),
                "min_h_p90": _fmt_float(min_h_p90),
                "min_cbf_median": _fmt_float(min_cbf_med),
                "min_cbf_p10": _fmt_float(min_cbf_p10),
                "min_cbf_p90": _fmt_float(min_cbf_p90),
                "solve_time_mean_ms_median": _fmt_float(stm_med),
                "solve_time_mean_ms_p10": _fmt_float(stm_p10),
                "solve_time_mean_ms_p90": _fmt_float(stm_p90),
                "solve_time_p95_ms_median": _fmt_float(stp95_med),
                "solve_time_p95_ms_p10": _fmt_float(stp95_p10),
                "solve_time_p95_ms_p90": _fmt_float(stp95_p90),
                "solve_time_max_ms_median": _fmt_float(stmax_med),
                "solve_time_max_ms_p10": _fmt_float(stmax_p10),
                "solve_time_max_ms_p90": _fmt_float(stmax_p90),
                "slack_sum_median": _fmt_float(slack_sum_med),
                "slack_sum_p10": _fmt_float(slack_sum_p10),
                "slack_sum_p90": _fmt_float(slack_sum_p90),
                "slack_max_median": _fmt_float(slack_max_med),
                "slack_max_p10": _fmt_float(slack_max_p10),
                "slack_max_p90": _fmt_float(slack_max_p90),
                "n_col": str(col_k),
                "n_margin": str(margin_k),
                "n_fail": str(fail_k),
            }
        )

    with out_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=out_header)
        writer.writeheader()
        for row in rows_out:
            writer.writerow(row)

    print(
        f"Wrote {len(rows_out)} aggregated rows to {out_csv} "
        f"(from {sum(len(v) for v in groups.values())} runs)."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
