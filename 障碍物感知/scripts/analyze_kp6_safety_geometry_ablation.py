#!/usr/bin/env python3
from __future__ import annotations

import csv
import pathlib
import statistics


ROOT = pathlib.Path(__file__).resolve().parents[1]
OUT = ROOT / "results" / "reviewer6_comment3_3_kp6_safety_geometry_ablation_report.md"


def load(relative: str):
    path = ROOT / relative
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def controller_summary(rows, controller: str):
    selected = [row for row in rows if row["controller"] == controller]
    return {
        "n": len(selected),
        "contacts": sum(int(float(row["collision"])) for row in selected),
        "rms": statistics.mean(float(row["tracking_rms"]) for row in selected),
        "clearance": statistics.mean(
            float(row["truth_static_min_surface_clearance"]) for row in selected
        ),
        "slack": statistics.mean(float(row["solver_slack_sum"]) for row in selected),
        "prediction": statistics.mean(
            float(row["prediction_horizon_rms_xy_mean"]) for row in selected
        ),
    }


def paired(rows):
    by_seed = {}
    for row in rows:
        by_seed.setdefault(int(row["seed"]), {})[row["controller"]] = row
    both = frozen_only = stage_only = neither = 0
    common_safe = []
    for seed, pair in sorted(by_seed.items()):
        if len(pair) != 2:
            continue
        frozen_contact = int(float(pair["noninertial_frozen"]["collision"]))
        stage_contact = int(float(pair["noninertial_stage"]["collision"]))
        if frozen_contact and stage_contact:
            both += 1
        elif frozen_contact:
            frozen_only += 1
        elif stage_contact:
            stage_only += 1
        else:
            neither += 1
            common_safe.append(seed)
    diffs = [
        float(by_seed[seed]["noninertial_stage"]["tracking_rms"])
        - float(by_seed[seed]["noninertial_frozen"]["tracking_rms"])
        for seed in common_safe
    ]
    frozen_rms = statistics.mean(
        float(by_seed[seed]["noninertial_frozen"]["tracking_rms"])
        for seed in common_safe
    )
    stage_rms = statistics.mean(
        float(by_seed[seed]["noninertial_stage"]["tracking_rms"])
        for seed in common_safe
    )
    return {
        "both": both,
        "frozen_only": frozen_only,
        "stage_only": stage_only,
        "neither": neither,
        "common_safe": common_safe,
        "frozen_rms": frozen_rms,
        "stage_rms": stage_rms,
        "mean_diff": statistics.mean(diffs),
        "stage_wins": sum(diff < 0.0 for diff in diffs),
    }


def fmt(summary):
    return (
        f'{summary["contacts"]}/{summary["n"]} | '
        f'{summary["rms"]:.6f} | {summary["clearance"]:.6f} | '
        f'{summary["slack"]:.3f}'
    )


def main():
    variants = [
        (
            "Legacy soft (2, 500)",
            "results/reviewer6_comment3_3_kp6_safety_layer_ablation_12seed/"
            "baseline_soft2_r500/tables/per_seed_controller_metrics.csv",
        ),
        (
            "Strong soft (0.5, 2000)",
            "results/reviewer6_comment3_3_kp6_safety_layer_ablation_12seed/"
            "strong_soft0p5_r2000/tables/per_seed_controller_metrics.csv",
        ),
        (
            "Hard CBF",
            "results/reviewer6_comment3_3_kp6_safety_layer_ablation_12seed/"
            "hard0/tables/per_seed_controller_metrics.csv",
        ),
    ]
    stage_only_variants = [
        (
            "Full predicted geometry + soft",
            "results/reviewer6_comment3_3_kp6_stage_geometry_consistency_12seed/"
            "predicted_soft2_r500/tables/per_seed_controller_metrics.csv",
        ),
        (
            "Full predicted geometry + hard",
            "results/reviewer6_comment3_3_kp6_stage_geometry_consistency_12seed/"
            "predicted_hard0/tables/per_seed_controller_metrics.csv",
        ),
    ]

    lines = [
        "# K_P=6 safety-layer and Stage-geometry ablation",
        "",
        "Contact uses `failure_distance_margin=0`, i.e. physical surface contact. "
        "The 12-seed set is targeted for mechanism localization and must not be "
        "reported as a population collision probability.",
        "",
        "## Targeted 12-seed localization",
        "",
        "| Variant | Controller | contacts/N | Tracking-RMS | mean truth clearance (m) | mean slack sum |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for name, path in variants:
        rows = load(path)
        for controller, label in [
            ("noninertial_frozen", "Frozen"),
            ("noninertial_stage", "Stage"),
        ]:
            lines.append(f"| {name} | {label} | {fmt(controller_summary(rows, controller))} |")
    for name, path in stage_only_variants:
        rows = load(path)
        lines.append(
            f"| {name} | Stage | {fmt(controller_summary(rows, 'noninertial_stage'))} |"
        )

    pilot_path = (
        "results/reviewer6_comment3_3_kp6_split_geometry_consistency_pilot20/"
        "map_soft2_r500/tables/per_seed_controller_metrics.csv"
    )
    pilot = load(pilot_path)
    frozen = controller_summary(pilot, "noninertial_frozen")
    stage = controller_summary(pilot, "noninertial_stage")
    pair = paired(pilot)
    improvement = 100.0 * (pair["frozen_rms"] - pair["stage_rms"]) / pair["frozen_rms"]
    prediction_improvement = 100.0 * (frozen["prediction"] - stage["prediction"]) / frozen["prediction"]

    lines += [
        "",
        "## Independent 20-seed Map pilot: split geometry + legacy soft-CBF",
        "",
        "| Controller | contacts/N | all-run Tracking-RMS | prediction-horizon RMS |",
        "|---|---:|---:|---:|",
        f'| Frozen | {frozen["contacts"]}/{frozen["n"]} | {frozen["rms"]:.6f} | {frozen["prediction"]:.6f} |',
        f'| Stage | {stage["contacts"]}/{stage["n"]} | {stage["rms"]:.6f} | {stage["prediction"]:.6f} |',
        "",
        f'- Paired contact table (both / Frozen-only / Stage-only / neither): '
        f'{pair["both"]} / {pair["frozen_only"]} / {pair["stage_only"]} / {pair["neither"]}.',
        f'- On the {len(pair["common_safe"])} seeds where both controllers completed without contact, '
        f'Frozen/Stage Tracking-RMS was {pair["frozen_rms"]:.6f}/{pair["stage_rms"]:.6f}; '
        f'Stage improved the paired mean by {improvement:.3f}% and won '
        f'{pair["stage_wins"]}/{len(pair["common_safe"])} seeds.',
        f'- Stage prediction-horizon RMS was {prediction_improvement:.3f}% lower.',
        "- All-run Tracking-RMS is not a valid standalone accuracy comparison here because "
        "contact terminates a run early and the two controllers have different contact sets.",
        "",
        "## Interpretation",
        "",
        "1. Tightening or eliminating slack reduces some contacts but does not remove the Stage disadvantage.",
        "2. Aligning the HOCBF obstacle transform with the Stage-wise frame profile reverses the contact trend in both the targeted set and the independent pilot.",
        "3. The recommended candidate is split predicted geometry with the original soft-CBF settings; the tracking reference remains unchanged while HOCBF uses the consistent Stage frame profile.",
        "4. This pilot localizes the mechanism. A formal claim still requires the prespecified 100 matched seeds and all perception conditions.",
        "",
        "## Regression evidence",
        "",
        "After adding the opt-in switches, the default geometry path was rerun on seeds 1--3. "
        "Collision, Tracking-RMS, Tracking-P95, solver minimum h, slack sum, truth clearance, "
        "and step count were exactly equal to the pre-change baseline for all six controller/seed cases.",
    ]
    OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(OUT)


if __name__ == "__main__":
    main()
