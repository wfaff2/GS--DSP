#!/usr/bin/env python3
"""Threshold-sensitivity analysis for Reviewer Comment 2.3.

This script re-labels the retained per-trial continuous surface-clearance
minima using thresholds tau = 0.000, 0.005, ..., 0.300 m.  It does not
change the controller or re-run any trajectory.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats


ROOT = Path(__file__).resolve().parent.parent
RESULTS_ROOT = ROOT / "results"
OUTPUT_ROOT = RESULTS_ROOT / "reviewer_comment_2_3_threshold_sensitivity"
FIGURE_PREFIX = ROOT / "figures" / "reviewer_comment_2_3_threshold_sensitivity"

CONTROLLERS = ("noninertial_frozen", "noninertial_stage")
CONTROLLER_LABELS = {
    "noninertial_frozen": "FF-MPSC",
    "noninertial_stage": "MF-MPSC",
}
CONTROLLER_COLORS = {
    "noninertial_frozen": "#D95F02",
    "noninertial_stage": "#1B9E77",
}
CONTROLLER_LINESTYLES = {
    "noninertial_frozen": "-",
    "noninertial_stage": "--",
}

KPS = (1, 2, 3, 4, 6)
PACKAGE_BY_KP = {
    1: "figure_eight_fixedobs_cbf_kp124_seed1_100_2ctrl",
    2: "figure_eight_fixedobs_cbf_kp124_seed1_100_2ctrl",
    3: "figure_eight_fixedobs_cbf_kp3_seed1_100_2ctrl",
    4: "figure_eight_fixedobs_cbf_kp124_seed1_100_2ctrl",
    6: "figure_eight_fixedobs_cbf_kp6_seed1_100_2ctrl",
}
V_BY_KP = {1: 0.5, 2: 1.0, 3: 1.5, 4: 2.0, 6: 3.0}

THRESHOLDS_M = np.round(np.arange(0.0, 0.300 + 0.0001, 0.005), 3)
PRIMARY_THRESHOLD_M = 0.215
SAFETY_MARGIN_M = 0.35
N_SEEDS = 100
N_BOOTSTRAP = 10_000

EXPECTED_PRIMARY_RATES = {
    1: (0.14, 0.13),
    2: (0.20, 0.17),
    3: (0.63, 0.52),
    4: (0.99, 0.86),
    6: (1.00, 1.00),
}


@dataclass(frozen=True)
class QualitySummary:
    rows: int
    duplicate_keys: int
    missing_distances: int
    status_counts: dict[str, int]
    min_distance_m: float
    max_distance_m: float
    max_summary_step_discrepancy_m: float


def folder_number(value: float) -> str:
    rounded = int(round(value))
    if np.isclose(value, rounded):
        return str(rounded)
    return f"{value:g}".replace(".", "p")


def holm_adjust(p_values: np.ndarray) -> np.ndarray:
    p_values = np.asarray(p_values, dtype=float)
    order = np.argsort(p_values)
    adjusted = np.empty(p_values.size, dtype=float)
    running_max = 0.0
    total = p_values.size
    for rank, original_index in enumerate(order):
        candidate = min(1.0, (total - rank) * p_values[original_index])
        running_max = max(running_max, candidate)
        adjusted[original_index] = running_max
    return adjusted


def bootstrap_mean_ci(differences: np.ndarray, seed: int) -> tuple[float, float]:
    rng = np.random.default_rng(seed)
    indices = rng.integers(
        0,
        differences.size,
        size=(N_BOOTSTRAP, differences.size),
    )
    means = differences[indices].mean(axis=1)
    low, high = np.quantile(means, (0.025, 0.975))
    return float(low), float(high)


def load_clearance_minima() -> tuple[pd.DataFrame, QualitySummary]:
    rows: list[dict[str, float | int | str]] = []
    for kp in KPS:
        package_root = RESULTS_ROOT / PACKAGE_BY_KP[kp]
        table_path = package_root / "tables" / "per_seed_controller_metrics.csv"
        frame = pd.read_csv(table_path)
        expected_v = V_BY_KP[kp]
        frame = frame[
            np.isclose(pd.to_numeric(frame["v"], errors="coerce"), expected_v)
            & np.isclose(pd.to_numeric(frame["w"], errors="coerce"), expected_v)
            & frame["controller"].isin(CONTROLLERS)
        ].copy()

        allowed_statuses = {"ok", "skipped"}
        observed_statuses = set(frame["status"].astype(str))
        if not observed_statuses.issubset(allowed_statuses):
            raise ValueError(
                f"K_P={kp}: unexpected run statuses {sorted(observed_statuses)}"
            )

        for controller in CONTROLLERS:
            controller_rows = frame[frame["controller"] == controller]
            seeds = set(pd.to_numeric(controller_rows["seed"]).astype(int))
            if seeds != set(range(1, N_SEEDS + 1)):
                raise ValueError(
                    f"K_P={kp}, {controller}: expected seeds 1--{N_SEEDS}, "
                    f"found {len(seeds)} unique seeds"
                )

        for row in frame.itertuples():
            seed = int(row.seed)
            controller = str(row.controller)
            step_path = (
                package_root
                / "logs"
                / controller
                / f"v_{folder_number(float(row.v))}"
                / f"w_{folder_number(float(row.w))}"
                / f"seed_{seed:03d}"
                / "metrics_steps.csv"
            )
            steps = pd.read_csv(
                step_path,
                usecols=["solver_planar_surface_distance"],
            )
            distances = pd.to_numeric(
                steps["solver_planar_surface_distance"],
                errors="coerce",
            ).dropna()
            if distances.empty:
                raise ValueError(f"No usable surface-clearance samples in {step_path}")
            step_min = float(distances.min())
            summary_min = float(row.solver_min_planar_clearance) + SAFETY_MARGIN_M
            rows.append(
                {
                    "kp": kp,
                    "controller": controller,
                    "controller_label": CONTROLLER_LABELS[controller],
                    "seed": seed,
                    "status": str(row.status),
                    "min_surface_clearance_m": step_min,
                    "summary_derived_clearance_m": summary_min,
                    "summary_step_discrepancy_m": step_min - summary_min,
                    "source_metrics_steps": str(step_path.relative_to(ROOT)),
                }
            )

    minima = pd.DataFrame(rows).sort_values(["kp", "controller", "seed"])
    key = ["kp", "controller", "seed"]
    duplicate_keys = int(minima.duplicated(key).sum())
    missing_distances = int(minima["min_surface_clearance_m"].isna().sum())
    expected_rows = len(KPS) * len(CONTROLLERS) * N_SEEDS
    if len(minima) != expected_rows or duplicate_keys or missing_distances:
        raise ValueError(
            "Invalid trial grain: "
            f"rows={len(minima)} (expected {expected_rows}), "
            f"duplicate_keys={duplicate_keys}, missing={missing_distances}"
        )

    counts = minima.groupby(["kp", "controller"])["seed"].nunique()
    if not bool((counts == N_SEEDS).all()):
        raise ValueError(f"Unexpected per-group seed counts:\n{counts}")

    quality = QualitySummary(
        rows=len(minima),
        duplicate_keys=duplicate_keys,
        missing_distances=missing_distances,
        status_counts={
            str(key): int(value)
            for key, value in minima["status"].value_counts().sort_index().items()
        },
        min_distance_m=float(minima["min_surface_clearance_m"].min()),
        max_distance_m=float(minima["min_surface_clearance_m"].max()),
        max_summary_step_discrepancy_m=float(
            minima["summary_step_discrepancy_m"].abs().max()
        ),
    )
    return minima, quality


def exact_mcnemar(
    frozen_violation: np.ndarray,
    stage_violation: np.ndarray,
) -> tuple[int, int, float]:
    frozen_only = int(np.sum(frozen_violation & ~stage_violation))
    stage_only = int(np.sum(~frozen_violation & stage_violation))
    discordant = frozen_only + stage_only
    p_value = (
        float(stats.binomtest(frozen_only, discordant, 0.5).pvalue)
        if discordant
        else 1.0
    )
    return frozen_only, stage_only, p_value


def build_sensitivity(minima: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, float | int]] = []
    for kp in KPS:
        pivot = minima[minima["kp"] == kp].pivot(
            index="seed",
            columns="controller",
            values="min_surface_clearance_m",
        )
        pivot = pivot[list(CONTROLLERS)].sort_index()
        if pivot.shape != (N_SEEDS, 2) or pivot.isna().any().any():
            raise ValueError(f"K_P={kp}: incomplete paired clearance matrix")
        frozen = pivot["noninertial_frozen"].to_numpy(dtype=float)
        stage = pivot["noninertial_stage"].to_numpy(dtype=float)
        for threshold_m in THRESHOLDS_M:
            frozen_violation = frozen < threshold_m
            stage_violation = stage < threshold_m
            frozen_only, stage_only, p_value = exact_mcnemar(
                frozen_violation,
                stage_violation,
            )
            frozen_rate = float(frozen_violation.mean())
            stage_rate = float(stage_violation.mean())
            reduction = frozen_rate - stage_rate
            rows.append(
                {
                    "kp": kp,
                    "threshold_m": float(threshold_m),
                    "n_paired": N_SEEDS,
                    "ff_mpsc_low_clearance_rate": frozen_rate,
                    "mf_mpsc_low_clearance_rate": stage_rate,
                    "absolute_rate_reduction_ff_minus_mf": reduction,
                    "relative_rate_reduction_vs_ff": (
                        reduction / frozen_rate if frozen_rate > 0.0 else np.nan
                    ),
                    "discordant_ff_only": frozen_only,
                    "discordant_mf_only": stage_only,
                    "mcnemar_exact_p": p_value,
                }
            )

    sensitivity = pd.DataFrame(rows)
    sensitivity["mcnemar_p_holm_across_kp_at_threshold"] = np.nan
    for _, index in sensitivity.groupby("threshold_m").groups.items():
        sensitivity.loc[index, "mcnemar_p_holm_across_kp_at_threshold"] = (
            holm_adjust(sensitivity.loc[index, "mcnemar_exact_p"].to_numpy())
        )

    primary = sensitivity[np.isclose(sensitivity["threshold_m"], PRIMARY_THRESHOLD_M)]
    if len(primary) != len(KPS):
        raise ValueError("Primary threshold is missing from the sensitivity grid")
    for row in primary.itertuples():
        expected_ff, expected_mf = EXPECTED_PRIMARY_RATES[int(row.kp)]
        if not (
            np.isclose(row.ff_mpsc_low_clearance_rate, expected_ff)
            and np.isclose(row.mf_mpsc_low_clearance_rate, expected_mf)
        ):
            raise ValueError(f"K_P={row.kp}: failed to reproduce Table I at 0.215 m")
    return sensitivity


def primary_positive_interval(group: pd.DataFrame) -> tuple[float, float] | None:
    group = group.sort_values("threshold_m").reset_index(drop=True)
    primary_matches = np.flatnonzero(
        np.isclose(group["threshold_m"].to_numpy(), PRIMARY_THRESHOLD_M)
    )
    if primary_matches.size != 1:
        raise ValueError("Primary threshold must occur exactly once per K_P")
    center = int(primary_matches[0])
    positive = group["absolute_rate_reduction_ff_minus_mf"].to_numpy() > 0.0
    if not positive[center]:
        return None
    left = center
    right = center
    while left > 0 and positive[left - 1]:
        left -= 1
    while right + 1 < len(group) and positive[right + 1]:
        right += 1
    return (
        float(group.loc[left, "threshold_m"]),
        float(group.loc[right, "threshold_m"]),
    )


def build_clearance_summary(
    minima: pd.DataFrame,
    sensitivity: pd.DataFrame,
) -> pd.DataFrame:
    rows: list[dict[str, float | int | str]] = []
    for kp in KPS:
        pivot = minima[minima["kp"] == kp].pivot(
            index="seed",
            columns="controller",
            values="min_surface_clearance_m",
        )[list(CONTROLLERS)].sort_index()
        frozen = pivot["noninertial_frozen"].to_numpy(dtype=float)
        stage = pivot["noninertial_stage"].to_numpy(dtype=float)
        differences = stage - frozen
        if np.allclose(differences, 0.0):
            statistic, p_value = 0.0, 1.0
        else:
            statistic, p_value = stats.wilcoxon(
                stage,
                frozen,
                zero_method="wilcox",
                alternative="two-sided",
            )
        # Match the deterministic seed convention used by the response-wide
        # paired-statistics audit for the fixed-obstacle clearance metric.
        ci_low, ci_high = bootstrap_mean_ci(
            differences,
            seed=20_000 + 100 * kp + 2,
        )

        group = sensitivity[sensitivity["kp"] == kp].copy()
        primary = group[np.isclose(group["threshold_m"], PRIMARY_THRESHOLD_M)].iloc[0]
        max_row = group.loc[
            group["absolute_rate_reduction_ff_minus_mf"].idxmax()
        ]
        interval = primary_positive_interval(group)
        rows.append(
            {
                "kp": kp,
                "n_paired": N_SEEDS,
                "ff_clearance_mean_m": float(frozen.mean()),
                "ff_clearance_sd_m": float(frozen.std(ddof=1)),
                "mf_clearance_mean_m": float(stage.mean()),
                "mf_clearance_sd_m": float(stage.std(ddof=1)),
                "paired_mean_difference_mf_minus_ff_m": float(differences.mean()),
                "paired_bootstrap_ci95_low_m": ci_low,
                "paired_bootstrap_ci95_high_m": ci_high,
                "wilcoxon_statistic": float(statistic),
                "wilcoxon_p": float(p_value),
                "primary_threshold_m": PRIMARY_THRESHOLD_M,
                "primary_ff_rate": float(primary["ff_mpsc_low_clearance_rate"]),
                "primary_mf_rate": float(primary["mf_mpsc_low_clearance_rate"]),
                "primary_absolute_reduction": float(
                    primary["absolute_rate_reduction_ff_minus_mf"]
                ),
                "primary_mcnemar_exact_p": float(primary["mcnemar_exact_p"]),
                "primary_mcnemar_p_holm_across_kp": float(
                    primary["mcnemar_p_holm_across_kp_at_threshold"]
                ),
                "largest_observed_absolute_reduction": float(
                    max_row["absolute_rate_reduction_ff_minus_mf"]
                ),
                "threshold_at_largest_reduction_m": float(max_row["threshold_m"]),
                "positive_interval_containing_0p215_m": (
                    "none"
                    if interval is None
                    else f"[{interval[0]:.3f}, {interval[1]:.3f}]"
                ),
            }
        )

    summary = pd.DataFrame(rows)
    summary["wilcoxon_p_holm_across_kp"] = holm_adjust(
        summary["wilcoxon_p"].to_numpy()
    )
    return summary


def plot_sensitivity(sensitivity: pd.DataFrame) -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 8.0,
            "axes.titlesize": 8.5,
            "axes.labelsize": 8.5,
            "legend.fontsize": 7.5,
            "xtick.labelsize": 7.3,
            "ytick.labelsize": 7.3,
            "axes.linewidth": 0.8,
            "lines.linewidth": 1.6,
        }
    )
    figure = plt.figure(figsize=(7.15, 4.25), constrained_layout=False)
    outer = figure.add_gridspec(
        2,
        1,
        height_ratios=(1.05, 1.0),
        left=0.08,
        right=0.99,
        bottom=0.12,
        top=0.88,
        hspace=0.48,
    )
    top = outer[0].subgridspec(1, len(KPS), wspace=0.20)
    top_axes = [figure.add_subplot(top[0, index]) for index in range(len(KPS))]
    difference_axis = figure.add_subplot(outer[1])

    for index, (axis, kp) in enumerate(zip(top_axes, KPS)):
        group = sensitivity[sensitivity["kp"] == kp].sort_values("threshold_m")
        for controller, rate_column in (
            ("noninertial_frozen", "ff_mpsc_low_clearance_rate"),
            ("noninertial_stage", "mf_mpsc_low_clearance_rate"),
        ):
            axis.plot(
                group["threshold_m"],
                group[rate_column],
                color=CONTROLLER_COLORS[controller],
                linestyle=CONTROLLER_LINESTYLES[controller],
                label=CONTROLLER_LABELS[controller],
            )
            primary = group[np.isclose(group["threshold_m"], PRIMARY_THRESHOLD_M)]
            axis.plot(
                primary["threshold_m"],
                primary[rate_column],
                marker="o",
                markersize=3.3,
                markerfacecolor="white" if controller == "noninertial_stage" else CONTROLLER_COLORS[controller],
                markeredgecolor=CONTROLLER_COLORS[controller],
                markeredgewidth=0.8,
                linestyle="none",
                zorder=4,
            )
        axis.axvline(PRIMARY_THRESHOLD_M, color="#4D4D4D", linestyle=":", linewidth=0.9)
        axis.set_title(rf"$K_P={kp}$", pad=2.5)
        axis.set_xlim(0.0, 0.300)
        axis.set_ylim(0.0, 1.02)
        axis.set_xticks((0.0, 0.1, 0.2, 0.3))
        axis.set_yticks((0.0, 0.5, 1.0))
        axis.grid(axis="y", color="#D9D9D9", linewidth=0.55)
        axis.grid(axis="x", visible=False)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)
        if index == 0:
            axis.set_ylabel("Low-clearance rate")
        else:
            axis.tick_params(labelleft=False)
        axis.set_xlabel(r"Threshold $\tau$ [m]")

    handles, labels = top_axes[0].get_legend_handles_labels()
    figure.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.985),
        ncol=2,
        frameon=False,
    )
    figure.text(0.02, 0.955, "(a)", fontsize=8.5, fontweight="bold")

    kp_colors = {
        1: "#332288",
        2: "#117733",
        3: "#44AA99",
        4: "#CC6677",
        6: "#AA4499",
    }
    kp_linestyles = {1: "-", 2: "--", 3: "-.", 4: ":", 6: (0, (5, 1, 1, 1))}
    for kp in KPS:
        group = sensitivity[sensitivity["kp"] == kp].sort_values("threshold_m")
        difference_axis.plot(
            group["threshold_m"],
            group["absolute_rate_reduction_ff_minus_mf"],
            label=rf"$K_P={kp}$",
            color=kp_colors[kp],
            linestyle=kp_linestyles[kp],
            linewidth=1.55,
        )
        primary = group[np.isclose(group["threshold_m"], PRIMARY_THRESHOLD_M)]
        difference_axis.plot(
            primary["threshold_m"],
            primary["absolute_rate_reduction_ff_minus_mf"],
            marker="o",
            markersize=3.2,
            color=kp_colors[kp],
            linestyle="none",
            zorder=4,
        )
    difference_axis.axhline(0.0, color="#333333", linewidth=0.8)
    difference_axis.axvline(
        PRIMARY_THRESHOLD_M,
        color="#4D4D4D",
        linestyle=":",
        linewidth=0.9,
    )
    difference_axis.set_xlim(0.0, 0.300)
    difference_axis.set_xlabel(r"Threshold $\tau$ [m]")
    difference_axis.set_ylabel(r"Rate reduction $P_{\rm FF}-P_{\rm MF}$")
    difference_axis.set_title(
        "(b) Absolute reduction in the thresholded low-clearance rate",
        loc="left",
        pad=3.5,
    )
    difference_axis.grid(axis="y", color="#D9D9D9", linewidth=0.55)
    difference_axis.grid(axis="x", visible=False)
    difference_axis.spines["top"].set_visible(False)
    difference_axis.spines["right"].set_visible(False)
    difference_axis.legend(
        frameon=False,
        ncol=5,
        loc="upper right",
        columnspacing=1.1,
        handlelength=2.5,
    )
    difference_axis.annotate(
        r"$\tau=0.215$ m",
        xy=(PRIMARY_THRESHOLD_M, 0.045),
        xycoords="data",
        xytext=(3, 0),
        textcoords="offset points",
        rotation=90,
        ha="left",
        va="bottom",
        fontsize=7.0,
        color="#4D4D4D",
    )

    FIGURE_PREFIX.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(FIGURE_PREFIX.with_suffix(".pdf"), bbox_inches="tight")
    figure.savefig(FIGURE_PREFIX.with_suffix(".png"), dpi=400, bbox_inches="tight")
    plt.close(figure)


def write_summary_markdown(
    quality: QualitySummary,
    summary: pd.DataFrame,
) -> None:
    lines = [
        "# Reviewer Comment 2.3: threshold-sensitivity analysis",
        "",
        "## Data and grain",
        "",
        f"- {quality.rows} controller-specific trials: five $K_P$ settings, two controllers, and 100 matched seeds per controller and setting.",
        f"- Composite-key duplicates: {quality.duplicate_keys}; missing per-trial clearance minima: {quality.missing_distances}.",
        "- Batch-runner status counts: "
        + ", ".join(f"`{key}`={value}" for key, value in quality.status_counts.items())
        + ". Here `skipped` denotes reuse of an already-complete metrics file, not an excluded trial.",
        f"- Observed per-trial minimum recorded surface-clearance range: {quality.min_distance_m:.6f}--{quality.max_distance_m:.6f} m.",
        f"- Maximum absolute discrepancy between the step-log minimum and `solver_min_planar_clearance + 0.35 m`: {quality.max_summary_step_discrepancy_m:.3e} m.",
        "",
        "## Method",
        "",
        "For every matched trial, the minimum recorded planar surface clearance was extracted from `metrics_steps.csv`. The offline threshold was swept from 0.000 to 0.300 m in 0.005 m increments, and the low-clearance rate was recomputed as $P(d_{min}<\\tau)$. Changing $\\tau$ changes only the reporting label; it does not change the simulated trajectory or the controller's 0.35 m internal safety margin.",
        "",
        "## Primary-threshold and continuous-clearance results",
        "",
        "| $K_P$ | FF/MF rate at 0.215 m | FF-MF reduction | Positive interval containing 0.215 m | Paired clearance difference MF-FF [m] (95% CI) |",
        "|---:|:---:|---:|:---:|:---:|",
    ]
    for row in summary.itertuples():
        lines.append(
            f"| {int(row.kp)} | {row.primary_ff_rate:.2f}/{row.primary_mf_rate:.2f} | "
            f"{row.primary_absolute_reduction:+.2f} | {row.positive_interval_containing_0p215_m} | "
            f"{row.paired_mean_difference_mf_minus_ff_m:+.4f} "
            f"([{row.paired_bootstrap_ci95_low_m:+.4f}, {row.paired_bootstrap_ci95_high_m:+.4f}]) |"
        )
    lines.extend(
        [
            "",
            "## Interpretation for the response",
            "",
            "- The 0.215 m result is exactly reproduced for all five operating points.",
            "- The sensitivity curves should be presented as descriptive robustness evidence; the thresholds are nested, so point-by-point significance labels across the full sweep would overstate independent evidence.",
            "- The binary rate is operating-point and threshold dependent. At the primary cutoff, the clearest reduction occurs at $K_P=4$, whereas both methods are saturated at $K_P=6$.",
            "- The continuous paired clearance analysis remains the appropriate complement when a binary cutoff saturates.",
            "",
            "## Reproducible artifacts",
            "",
            "- `clearance_minima_by_seed.csv`: one row per controller, $K_P$, and seed.",
            "- `threshold_sensitivity.csv`: 61 thresholds by five operating points.",
            "- `clearance_summary_by_kp.csv`: paired continuous-clearance and primary-threshold summary.",
            "- `figures/reviewer_comment_2_3_threshold_sensitivity.pdf` and `.png`: manuscript-ready plot.",
            "- Rebuild command: `MPLCONFIGDIR=/tmp/mpl-comment-2-3 python3 scripts/analyze_threshold_sensitivity_comment_2_3.py`.",
        ]
    )
    (OUTPUT_ROOT / "analysis_summary.md").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    minima, quality = load_clearance_minima()
    sensitivity = build_sensitivity(minima)
    summary = build_clearance_summary(minima, sensitivity)

    minima.to_csv(OUTPUT_ROOT / "clearance_minima_by_seed.csv", index=False)
    sensitivity.to_csv(OUTPUT_ROOT / "threshold_sensitivity.csv", index=False)
    summary.to_csv(OUTPUT_ROOT / "clearance_summary_by_kp.csv", index=False)
    plot_sensitivity(sensitivity)
    write_summary_markdown(quality, summary)

    print(
        "Validated "
        f"{quality.rows} trials with no duplicate keys or missing clearance minima."
    )
    print(
        "Surface-clearance range: "
        f"{quality.min_distance_m:.6f}--{quality.max_distance_m:.6f} m."
    )
    print("Reproduced all five Table I rates at tau=0.215 m.")
    print(f"Wrote analysis artifacts under {OUTPUT_ROOT.relative_to(ROOT)}")
    print(f"Wrote figure {FIGURE_PREFIX.relative_to(ROOT)}.[pdf|png]")


if __name__ == "__main__":
    main()
