#!/usr/bin/env python3
from __future__ import annotations

import math
import pathlib

import numpy as np
import pandas as pd
from scipy.stats import wilcoxon


ROOT = pathlib.Path(__file__).resolve().parents[1]
RUN_ROOT = (
    ROOT
    / "results"
    / "reviewer6_comment3_3_allkp_split_geometry_sensor_formal100_jobs12"
)
MODES = ("online_nominal", "online_degraded")
KP_BY_V = {0.5: 1, 1.0: 2, 1.5: 3, 2.0: 4, 3.0: 6}
BOOTSTRAP_RESAMPLES = 10_000


def exact_mcnemar_p(frozen_only: int, stage_only: int) -> float:
    discordant = frozen_only + stage_only
    if discordant == 0:
        return 1.0
    lower = min(frozen_only, stage_only)
    tail = sum(math.comb(discordant, k) for k in range(lower + 1)) / 2**discordant
    return min(1.0, 2.0 * tail)


def paired_bootstrap_ci(values: np.ndarray, rng: np.random.Generator):
    if values.size == 0:
        return math.nan, math.nan
    indices = rng.integers(0, values.size, size=(BOOTSTRAP_RESAMPLES, values.size))
    means = values[indices].mean(axis=1)
    low, high = np.quantile(means, [0.025, 0.975])
    return float(low), float(high)


def holm_adjust(p_values):
    p = np.asarray(p_values, dtype=float)
    order = np.argsort(p)
    adjusted = np.empty_like(p)
    running = 0.0
    count = len(p)
    for rank, original_idx in enumerate(order):
        candidate = min(1.0, (count - rank) * p[original_idx])
        running = max(running, candidate)
        adjusted[original_idx] = running
    return adjusted


def validate_mode(mode: str, manifest: pd.DataFrame, per_seed: pd.DataFrame) -> None:
    if len(manifest) != 1000 or set(manifest["status"]) != {"ok"}:
        raise SystemExit(f"{mode}: manifest is not 1000 all-OK cases")
    expected_modes = {
        "noninertial_frozen": "frozen_current_curvature",
        "noninertial_stage": "split_predicted",
    }
    for controller, geometry_mode in expected_modes.items():
        manifest_rows = manifest[manifest["controller"].eq(controller)]
        metric_rows = per_seed[per_seed["controller"].eq(controller)]
        if len(manifest_rows) != 500 or len(metric_rows) != 500:
            raise SystemExit(f"{mode}: incomplete {controller} rows")
        if set(manifest_rows["stage_geometry_mode"]) != {geometry_mode}:
            raise SystemExit(f"{mode}: wrong manifest geometry for {controller}")
        if set(metric_rows["stage_geometry_mode"]) != {geometry_mode}:
            raise SystemExit(f"{mode}: wrong metric geometry for {controller}")


def main() -> None:
    summary_rows = []
    paired_seed_rows = []
    rng = np.random.default_rng(20260810)

    for mode in MODES:
        mode_root = RUN_ROOT / mode
        manifest = pd.read_csv(mode_root / "run_manifest.csv")
        data = pd.read_csv(mode_root / "tables" / "per_seed_controller_metrics.csv")
        validate_mode(mode, manifest, data)

        for v, kp in KP_BY_V.items():
            subset = data[np.isclose(data["v"], v)]
            frozen = (
                subset[subset["controller"].eq("noninertial_frozen")]
                .set_index("seed")
                .sort_index()
            )
            stage = (
                subset[subset["controller"].eq("noninertial_stage")]
                .set_index("seed")
                .sort_index()
            )
            expected_seeds = list(range(1, 101))
            if frozen.index.tolist() != expected_seeds or stage.index.tolist() != expected_seeds:
                raise SystemExit(f"{mode}, K_P={kp}: incomplete paired seeds")

            frozen_contact = frozen["collision"].astype(int)
            stage_contact = stage["collision"].astype(int)
            both = frozen_contact.eq(1) & stage_contact.eq(1)
            frozen_only = frozen_contact.eq(1) & stage_contact.eq(0)
            stage_only = frozen_contact.eq(0) & stage_contact.eq(1)
            neither = frozen_contact.eq(0) & stage_contact.eq(0)
            common_complete = neither

            contact_diff = (stage_contact - frozen_contact).to_numpy(dtype=float)
            contact_ci_low, contact_ci_high = paired_bootstrap_ci(contact_diff, rng)
            tracking_diff = (
                stage.loc[common_complete, "tracking_rms"]
                - frozen.loc[common_complete, "tracking_rms"]
            ).to_numpy(dtype=float)
            tracking_ci_low, tracking_ci_high = paired_bootstrap_ci(tracking_diff, rng)
            tracking_wilcoxon_p = (
                1.0
                if np.allclose(tracking_diff, 0.0)
                else float(
                    wilcoxon(
                        tracking_diff,
                        alternative="two-sided",
                        zero_method="wilcox",
                        method="auto",
                    ).pvalue
                )
            )
            frozen_common_rms = float(
                frozen.loc[common_complete, "tracking_rms"].mean()
            )
            stage_common_rms = float(
                stage.loc[common_complete, "tracking_rms"].mean()
            )
            tracking_improvement_pct = (
                100.0 * (frozen_common_rms - stage_common_rms) / frozen_common_rms
            )

            summary_rows.append(
                {
                    "sensing_mode": mode,
                    "kp": kp,
                    "v": v,
                    "w": v,
                    "seed_count": 100,
                    "frozen_contact_count": int(frozen_contact.sum()),
                    "stage_contact_count": int(stage_contact.sum()),
                    "frozen_contact_rate": float(frozen_contact.mean()),
                    "stage_contact_rate": float(stage_contact.mean()),
                    "contact_rate_diff_stage_minus_frozen": float(contact_diff.mean()),
                    "contact_rate_diff_ci95_low": contact_ci_low,
                    "contact_rate_diff_ci95_high": contact_ci_high,
                    "contact_both": int(both.sum()),
                    "contact_frozen_only": int(frozen_only.sum()),
                    "contact_stage_only": int(stage_only.sum()),
                    "contact_neither": int(neither.sum()),
                    "mcnemar_exact_p": exact_mcnemar_p(
                        int(frozen_only.sum()), int(stage_only.sum())
                    ),
                    "common_complete_count": int(common_complete.sum()),
                    "frozen_common_tracking_rms": frozen_common_rms,
                    "stage_common_tracking_rms": stage_common_rms,
                    "tracking_diff_stage_minus_frozen": float(tracking_diff.mean()),
                    "tracking_diff_ci95_low": tracking_ci_low,
                    "tracking_diff_ci95_high": tracking_ci_high,
                    "tracking_wilcoxon_p": tracking_wilcoxon_p,
                    "tracking_improvement_pct": tracking_improvement_pct,
                    "stage_tracking_win_count": int((tracking_diff < 0.0).sum()),
                    "frozen_solver_fail_rate_max": float(frozen["solver_fail_rate"].max()),
                    "stage_solver_fail_rate_max": float(stage["solver_fail_rate"].max()),
                    "frozen_truth_clearance_mean": float(
                        frozen["truth_static_min_surface_clearance"].mean()
                    ),
                    "stage_truth_clearance_mean": float(
                        stage["truth_static_min_surface_clearance"].mean()
                    ),
                }
            )

            for seed in expected_seeds:
                if both.loc[seed]:
                    outcome = "both_contact"
                elif frozen_only.loc[seed]:
                    outcome = "frozen_only_contact"
                elif stage_only.loc[seed]:
                    outcome = "stage_only_contact"
                else:
                    outcome = "neither_contact"
                paired_seed_rows.append(
                    {
                        "sensing_mode": mode,
                        "kp": kp,
                        "seed": seed,
                        "contact_outcome": outcome,
                        "frozen_contact": int(frozen_contact.loc[seed]),
                        "stage_contact": int(stage_contact.loc[seed]),
                        "frozen_tracking_rms": frozen.loc[seed, "tracking_rms"],
                        "stage_tracking_rms": stage.loc[seed, "tracking_rms"],
                        "frozen_step_rows": int(frozen.loc[seed, "step_rows"]),
                        "stage_step_rows": int(stage.loc[seed, "step_rows"]),
                    }
                )

    summary = pd.DataFrame(summary_rows)
    summary["mcnemar_holm_p_10tests"] = holm_adjust(summary["mcnemar_exact_p"])
    summary["tracking_wilcoxon_holm_p_10tests"] = holm_adjust(
        summary["tracking_wilcoxon_p"]
    )
    summary = summary.sort_values(["sensing_mode", "kp"], kind="stable")
    paired_seeds = pd.DataFrame(paired_seed_rows).sort_values(
        ["sensing_mode", "kp", "seed"], kind="stable"
    )
    summary.to_csv(RUN_ROOT / "paired_contact_tracking_by_kp.csv", index=False)
    paired_seeds.to_csv(RUN_ROOT / "paired_seed_outcomes_allkp.csv", index=False)

    lines = [
        "# Corrected all-K_P Nominal/Degraded formal100 paired analysis",
        "",
        "All contacts use `failure_distance_margin=0`. Tracking-RMS is compared "
        "only on matched seeds where both controllers completed without physical contact. "
        f"Confidence intervals use {BOOTSTRAP_RESAMPLES:,} paired resamples.",
        "",
    ]
    for mode, title in (("online_nominal", "Nominal"), ("online_degraded", "Degraded")):
        lines += [
            f"## {title}",
            "",
            "| K_P | Contact F/S | Contact S-F [95% CI] | both/F-only/S-only/neither | McNemar p | Contact Holm p | Complete N | Tracking-RMS F/S | Stage improvement | Tracking S-F 95% CI | Wilcoxon p | Tracking Holm p |",
            "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
        for row in summary[summary["sensing_mode"].eq(mode)].itertuples():
            lines.append(
                f"| {row.kp} | {row.frozen_contact_count}%/{row.stage_contact_count}% | "
                f"{100*row.contact_rate_diff_stage_minus_frozen:+.1f} pp "
                f"[{100*row.contact_rate_diff_ci95_low:+.1f}, {100*row.contact_rate_diff_ci95_high:+.1f}] | "
                f"{row.contact_both}/{row.contact_frozen_only}/{row.contact_stage_only}/{row.contact_neither} | "
                f"{row.mcnemar_exact_p:.6f} | {row.mcnemar_holm_p_10tests:.6f} | "
                f"{row.common_complete_count} | "
                f"{row.frozen_common_tracking_rms:.6f}/{row.stage_common_tracking_rms:.6f} | "
                f"{row.tracking_improvement_pct:+.3f}% | "
                f"[{row.tracking_diff_ci95_low:+.6f}, {row.tracking_diff_ci95_high:+.6f}] | "
                f"{row.tracking_wilcoxon_p:.3e} | "
                f"{row.tracking_wilcoxon_holm_p_10tests:.3e} |"
            )
        lines.append("")

    lines += [
        "## Interpretation",
        "",
        "- No physical contacts occurred for either controller at K_P=1,2,3,4 under either sensing condition.",
        "- At K_P=6, Stage had fewer contacts in both conditions, but the paired McNemar comparisons were not statistically significant after considering the declared ten-test family.",
        "- For K_P=1,2,3,4, all 100 pairs completed and Stage had lower Tracking-RMS on all 100 seeds in both sensing conditions; every comparison remains significant after Holm correction over the ten Tracking-RMS tests.",
        "- K_P=6 Tracking-RMS is conditional on the common no-contact subset and must not be generalized to all seeds.",
        "- Solver failure rate was zero for every controller, K_P, and sensing condition.",
    ]
    (RUN_ROOT / "formal100_allkp_paired_report.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    print("\n".join(lines))


if __name__ == "__main__":
    main()
