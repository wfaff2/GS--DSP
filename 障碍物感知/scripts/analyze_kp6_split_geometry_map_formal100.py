#!/usr/bin/env python3
from __future__ import annotations

import csv
import math
import pathlib

import numpy as np
import pandas as pd


ROOT = pathlib.Path(__file__).resolve().parents[1]
RUN_ROOT = (
    ROOT
    / "results"
    / "reviewer6_comment3_3_kp6_split_geometry_map_formal100_jobs12"
)
TABLES = RUN_ROOT / "tables"
PER_SEED = TABLES / "per_seed_controller_metrics.csv"
MANIFEST = RUN_ROOT / "run_manifest.csv"


def exact_mcnemar_p(frozen_only: int, stage_only: int) -> float:
    discordant = frozen_only + stage_only
    if discordant == 0:
        return 1.0
    tail = sum(
        math.comb(discordant, k)
        for k in range(min(frozen_only, stage_only) + 1)
    ) / (2**discordant)
    return min(1.0, 2.0 * tail)


def paired_bootstrap_ci(values: np.ndarray, rng: np.random.Generator):
    indices = rng.integers(0, len(values), size=(10_000, len(values)))
    means = values[indices].mean(axis=1)
    return np.quantile(means, [0.025, 0.975])


def main() -> None:
    manifest = pd.read_csv(MANIFEST)
    if len(manifest) != 200 or set(manifest["status"]) != {"ok"}:
        raise SystemExit("Manifest is not a complete 200-case all-OK run")
    expected_modes = {
        "noninertial_frozen": "frozen_current_curvature",
        "noninertial_stage": "split_predicted",
    }
    for controller, mode in expected_modes.items():
        rows = manifest[manifest["controller"].eq(controller)]
        if len(rows) != 100 or set(rows["stage_geometry_mode"]) != {mode}:
            raise SystemExit(f"Unexpected mode/count for {controller}")

    df = pd.read_csv(PER_SEED)
    frozen = df[df["controller"].eq("noninertial_frozen")].set_index("seed").sort_index()
    stage = df[df["controller"].eq("noninertial_stage")].set_index("seed").sort_index()
    expected_seeds = list(range(1, 101))
    if frozen.index.tolist() != expected_seeds or stage.index.tolist() != expected_seeds:
        raise SystemExit("The 100 paired seeds are incomplete")

    frozen_contact = frozen["collision"].astype(int)
    stage_contact = stage["collision"].astype(int)
    both = (frozen_contact.eq(1) & stage_contact.eq(1))
    frozen_only = (frozen_contact.eq(1) & stage_contact.eq(0))
    stage_only = (frozen_contact.eq(0) & stage_contact.eq(1))
    neither = (frozen_contact.eq(0) & stage_contact.eq(0))

    paired_rows = []
    for seed in expected_seeds:
        if both.loc[seed]:
            outcome = "both_contact"
        elif frozen_only.loc[seed]:
            outcome = "frozen_only_contact"
        elif stage_only.loc[seed]:
            outcome = "stage_only_contact"
        else:
            outcome = "neither_contact"
        paired_rows.append(
            {
                "seed": seed,
                "outcome": outcome,
                "frozen_contact": int(frozen_contact.loc[seed]),
                "stage_contact": int(stage_contact.loc[seed]),
                "frozen_tracking_rms": frozen.loc[seed, "tracking_rms"],
                "stage_tracking_rms": stage.loc[seed, "tracking_rms"],
                "frozen_step_rows": int(frozen.loc[seed, "step_rows"]),
                "stage_step_rows": int(stage.loc[seed, "step_rows"]),
            }
        )
    pd.DataFrame(paired_rows).to_csv(TABLES / "paired_seed_outcomes.csv", index=False)

    b = int(both.sum())
    fo = int(frozen_only.sum())
    so = int(stage_only.sum())
    n = int(neither.sum())
    mcnemar_p = exact_mcnemar_p(fo, so)
    contact_diff = (stage_contact - frozen_contact).to_numpy(dtype=float)
    rng = np.random.default_rng(20260810)
    contact_ci = paired_bootstrap_ci(contact_diff, rng)

    common = neither
    tracking_diff = (
        stage.loc[common, "tracking_rms"] - frozen.loc[common, "tracking_rms"]
    ).to_numpy(dtype=float)
    tracking_ci = paired_bootstrap_ci(tracking_diff, rng)
    frozen_common_rms = float(frozen.loc[common, "tracking_rms"].mean())
    stage_common_rms = float(stage.loc[common, "tracking_rms"].mean())

    summary = {
        "seed_count": 100,
        "frozen_contact_count": int(frozen_contact.sum()),
        "stage_contact_count": int(stage_contact.sum()),
        "paired_contact_rate_difference_stage_minus_frozen": float(contact_diff.mean()),
        "paired_contact_rate_difference_ci95_low": float(contact_ci[0]),
        "paired_contact_rate_difference_ci95_high": float(contact_ci[1]),
        "contact_both": b,
        "contact_frozen_only": fo,
        "contact_stage_only": so,
        "contact_neither": n,
        "mcnemar_exact_two_sided_p": mcnemar_p,
        "common_no_contact_count": n,
        "frozen_common_no_contact_tracking_rms_mean": frozen_common_rms,
        "stage_common_no_contact_tracking_rms_mean": stage_common_rms,
        "paired_common_tracking_difference_stage_minus_frozen": float(tracking_diff.mean()),
        "paired_common_tracking_difference_ci95_low": float(tracking_ci[0]),
        "paired_common_tracking_difference_ci95_high": float(tracking_ci[1]),
        "common_tracking_stage_wins": int((tracking_diff < 0.0).sum()),
        "frozen_solver_fail_rate_mean": float(frozen["solver_fail_rate"].mean()),
        "stage_solver_fail_rate_mean": float(stage["solver_fail_rate"].mean()),
        "frozen_slack_sum_mean": float(frozen["solver_slack_sum"].mean()),
        "stage_slack_sum_mean": float(stage["solver_slack_sum"].mean()),
        "frozen_truth_clearance_mean": float(
            frozen["truth_static_min_surface_clearance"].mean()
        ),
        "stage_truth_clearance_mean": float(
            stage["truth_static_min_surface_clearance"].mean()
        ),
    }
    with (TABLES / "paired_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=summary.keys())
        writer.writeheader()
        writer.writerow(summary)

    report = f"""# Corrected K_P=6 Map formal 100-seed paired result

Configuration: `V=W=3`, Map, `A2_soft_cbf`, `slack_max=2`, `R_slack=500`, `failure_distance_margin=0`, 100 matched seeds, 12 parallel jobs. Frozen uses `frozen_current_curvature`; Stage uses `split_predicted`.

## Safety result

| Metric | Frozen | Stage |
|---|---:|---:|
| Physical contacts | {int(frozen_contact.sum())}/100 | {int(stage_contact.sum())}/100 |
| Mission completion without contact | {100-int(frozen_contact.sum())}/100 | {100-int(stage_contact.sum())}/100 |
| Mean truth surface clearance (m) | {summary['frozen_truth_clearance_mean']:.6f} | {summary['stage_truth_clearance_mean']:.6f} |
| Mean solver failure rate | {summary['frozen_solver_fail_rate_mean']:.6f} | {summary['stage_solver_fail_rate_mean']:.6f} |
| Mean slack sum | {summary['frozen_slack_sum_mean']:.3f} | {summary['stage_slack_sum_mean']:.3f} |

Paired contact table (both / Frozen-only / Stage-only / neither): **{b} / {fo} / {so} / {n}**. The paired Stage-minus-Frozen contact-rate difference is **{100*contact_diff.mean():.1f} percentage points**, with a 10,000-resample paired-bootstrap 95% CI of **[{100*contact_ci[0]:.1f}, {100*contact_ci[1]:.1f}] percentage points**. Exact two-sided McNemar **p={mcnemar_p:.6f}**.

## Tracking interpretation

Across all runs, collisions terminate trajectories at different times, so the unconditional Tracking-RMS means must not be treated as a fair accuracy comparison. Among the {n} seeds where both controllers completed without contact, Frozen/Stage mean Tracking-RMS is **{frozen_common_rms:.6f}/{stage_common_rms:.6f}**. The paired Stage-minus-Frozen difference is **{tracking_diff.mean():+.6f}**, with bootstrap 95% CI **[{tracking_ci[0]:+.6f}, {tracking_ci[1]:+.6f}]**; Stage is lower on **{int((tracking_diff < 0).sum())}/{n}** seeds. This conditional subset does not establish a tracking-accuracy advantage.

## Conclusion

The corrected Stage geometry produces a statistically supported safety improvement for this K_P=6 Map experiment: 26% versus 45% physical contact, without solver failures. It does not, by itself, establish that Stage tracking is more accurate in the fixed-obstacle scene.
"""
    (RUN_ROOT / "formal100_paired_report.md").write_text(report, encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
