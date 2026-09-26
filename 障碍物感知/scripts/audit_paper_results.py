#!/usr/bin/env python3
from __future__ import annotations

import csv
import math
from collections import Counter
from pathlib import Path

import pandas as pd


ROOT = Path(__file__).resolve().parent.parent
RESULTS = ROOT / "results"

MAIN_RUNS = [
    RESULTS / "circle_nocbf_kp124_seed1_100_2ctrl",
    RESULTS / "circle_nocbf_kp3_seed1_100_2ctrl",
    RESULTS / "circle_nocbf_kp6_seed1_100_2ctrl",
    RESULTS / "figure_eight_clean_nocbf_kp124_seed1_100_2ctrl",
    RESULTS / "figure_eight_clean_nocbf_kp3_seed1_100_2ctrl",
    RESULTS / "figure_eight_clean_nocbf_kp6_seed1_100_2ctrl",
    RESULTS / "figure_eight_fixedobs_cbf_kp124_seed1_100_2ctrl",
    RESULTS / "figure_eight_fixedobs_cbf_kp3_seed1_100_2ctrl",
    RESULTS / "figure_eight_fixedobs_cbf_kp6_seed1_100_2ctrl",
]


def latest_manifest_cases(run_root: Path) -> tuple[list[dict[str, str]], int]:
    manifest_path = run_root / "run_manifest.csv"
    with manifest_path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    latest: dict[tuple[str, str, str, str], dict[str, str]] = {}
    counts = Counter(
        (r.get("controller", ""), r.get("v", ""), r.get("w", ""), r.get("seed", ""))
        for r in rows
    )
    for row in rows:
        key = (
            row.get("controller", ""),
            row.get("v", ""),
            row.get("w", ""),
            row.get("seed", ""),
        )
        latest[key] = row
    duplicate_key_count = sum(1 for c in counts.values() if c > 1)
    return list(latest.values()), duplicate_key_count


def norm_key(controller: str, v: object, w: object, seed: object) -> tuple[str, str, str, str]:
    return (
        str(controller),
        f"{float(v):.6f}",
        f"{float(w):.6f}",
        str(int(float(seed))),
    )


def check_summary_consistency(per_seed: pd.DataFrame, summary: pd.DataFrame) -> list[str]:
    issues: list[str] = []
    mean_checks = {
        "tracking_rms_mean": "tracking_rms",
        "tracking_p95_mean": "tracking_p95",
        "solver_min_h_mean": "solver_min_h",
        "solver_slack_sum_mean": "solver_slack_sum",
        "core_time_mean_ms_mean": "core_time_mean_ms",
        "core_time_p95_ms_mean": "core_time_p95_ms",
    }
    for srow in summary.to_dict("records"):
        sub = per_seed[
            (per_seed["controller"] == srow["controller"])
            & (per_seed["v"] == srow["v"])
            & (per_seed["w"] == srow["w"])
        ].copy()
        seed_count = int(sub["seed"].nunique())
        row_count = int(len(sub))
        ok_count = int(sub["status"].isin(["ok", "skipped"]).sum())
        collision = pd.to_numeric(sub["collision"], errors="coerce")
        collision_count = int(collision.fillna(0).sum())
        collision_rate = float(collision.mean()) if len(collision.dropna()) else math.nan

        if seed_count != int(srow["seed_count"]):
            issues.append(
                f"summary seed_count mismatch for {srow['controller']} v={srow['v']} w={srow['w']}: "
                f"{seed_count} != {srow['seed_count']}"
            )
        if row_count != int(srow["row_count"]):
            issues.append(
                f"summary row_count mismatch for {srow['controller']} v={srow['v']} w={srow['w']}: "
                f"{row_count} != {srow['row_count']}"
            )
        if ok_count != int(srow["ok_count"]):
            issues.append(
                f"summary ok_count mismatch for {srow['controller']} v={srow['v']} w={srow['w']}: "
                f"{ok_count} != {srow['ok_count']}"
            )
        if collision_count != int(srow["collision_count"]):
            issues.append(
                f"summary collision_count mismatch for {srow['controller']} v={srow['v']} w={srow['w']}: "
                f"{collision_count} != {srow['collision_count']}"
            )
        summary_rate = float(srow["collision_rate"])
        if abs(collision_rate - summary_rate) > 1e-12:
            issues.append(
                f"summary collision_rate mismatch for {srow['controller']} v={srow['v']} w={srow['w']}: "
                f"{collision_rate} != {summary_rate}"
            )
        for summary_field, per_seed_field in mean_checks.items():
            vals = pd.to_numeric(sub[per_seed_field], errors="coerce").dropna()
            recomputed = float(vals.mean()) if not vals.empty else math.nan
            target = float(srow[summary_field])
            if (math.isnan(recomputed) and math.isnan(target)) or abs(recomputed - target) <= 1e-9:
                continue
            issues.append(
                f"summary {summary_field} mismatch for {srow['controller']} v={srow['v']} w={srow['w']}: "
                f"{recomputed} != {target}"
            )
    return issues


def main() -> None:
    overall_issues: list[str] = []
    for run_root in MAIN_RUNS:
        manifest_rows, duplicate_keys = latest_manifest_cases(run_root)
        per_seed_path = run_root / "tables" / "per_seed_controller_metrics.csv"
        summary_path = run_root / "tables" / "summary_by_vw_controller.csv"
        per_seed = pd.read_csv(per_seed_path)
        summary = pd.read_csv(summary_path)

        manifest_keys = {
            norm_key(r.get("controller", ""), r.get("v", "nan"), r.get("w", "nan"), r.get("seed", "nan"))
            for r in manifest_rows
        }
        per_seed_keys = {
            norm_key(r.controller, r.v, r.w, r.seed)
            for r in per_seed.itertuples()
        }

        issues: list[str] = []
        if manifest_keys != per_seed_keys:
            issues.append(
                f"manifest/per_seed key mismatch: manifest={len(manifest_keys)} per_seed={len(per_seed_keys)}"
            )

        bad = per_seed[~per_seed["status"].isin(["ok", "skipped"])].copy()
        if not bad.empty:
            issues.extend(
                f"non-ok trial {row.controller} v={row.v} w={row.w} seed={int(row.seed)} status={row.status}"
                for row in bad.itertuples()
            )

        for row in per_seed[per_seed["status"].isin(["ok", "skipped"])].itertuples():
            metrics_path = Path(str(row.metrics_csv))
            step_path = metrics_path.with_name("metrics_steps.csv")
            if not metrics_path.is_file():
                issues.append(
                    f"missing metrics.csv for {row.controller} v={row.v} w={row.w} seed={int(row.seed)}"
                )
            if not step_path.is_file():
                issues.append(
                    f"missing metrics_steps.csv for {row.controller} v={row.v} w={row.w} seed={int(row.seed)}"
                )

        issues.extend(check_summary_consistency(per_seed, summary))

        print(f"\n## {run_root.name}")
        print(f"manifest_latest_cases={len(manifest_rows)} duplicate_case_keys={duplicate_keys}")
        print(f"per_seed_rows={len(per_seed)} summary_rows={len(summary)}")
        if issues:
            print("issues:")
            for item in issues:
                print(f"- {item}")
            overall_issues.extend(f"{run_root.name}: {item}" for item in issues)
        else:
            print("issues: none")

    selected_cases_path = ROOT / "figures" / "paper_fig1_selected_cases.csv"
    if selected_cases_path.is_file():
        selected = pd.read_csv(selected_cases_path)
        fig_issues: list[str] = []
        for row in selected.itertuples():
            per_seed = pd.read_csv(
                RESULTS / str(row.result_dir) / "tables" / "per_seed_controller_metrics.csv"
            )
            v = 0.5 * int(row.kp)
            sub = per_seed[
                (per_seed["seed"] == int(row.seed))
                & (per_seed["v"] == v)
                & (per_seed["w"] == v)
            ]
            bad = sub[~sub["status"].isin(["ok", "skipped"])]
            if not bad.empty:
                fig_issues.append(f"Fig.1 case {row.name} includes non-ok seed={int(row.seed)}")
        print("\n## paper_fig1_selected_cases")
        if fig_issues:
            print("issues:")
            for item in fig_issues:
                print(f"- {item}")
            overall_issues.extend(fig_issues)
        else:
            print("issues: none")

    if overall_issues:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
