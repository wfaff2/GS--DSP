#!/usr/bin/env python3
from __future__ import annotations

import re
from pathlib import Path

import pandas as pd


ROOT = Path(__file__).resolve().parent.parent
PAPER_MD = ROOT / "paper.md"
BASE_V = 0.5
KP_VALUES = [1, 2, 3, 4, 6]
FAILURE_THRESHOLD = 0.215


def load_summary(run_root: Path) -> pd.DataFrame:
    df = pd.read_csv(run_root / "tables" / "summary_by_vw_controller.csv")
    df = df[df["controller"].isin(["noninertial_frozen", "noninertial_stage"])].copy()
    df["v"] = pd.to_numeric(df["v"], errors="coerce")
    df["w"] = pd.to_numeric(df["w"], errors="coerce")
    df["kp"] = (df["v"] / BASE_V).round().astype(int)
    return df


def collect_step_min_surface(run_root: Path) -> pd.DataFrame:
    rows = []
    for step_path in run_root.glob("logs/*/v_*/w_*/seed_*/metrics_steps.csv"):
        controller = step_path.parts[-5]
        v = float(step_path.parts[-4].split("_", 1)[1].replace("p", "."))
        w = float(step_path.parts[-3].split("_", 1)[1].replace("p", "."))
        seed = int(step_path.parts[-2].split("_", 1)[1])
        df = pd.read_csv(step_path, usecols=["solver_planar_surface_distance"])
        values = pd.to_numeric(df["solver_planar_surface_distance"], errors="coerce").dropna()
        rows.append(
            {
                "controller": controller,
                "v": v,
                "w": w,
                "seed": seed,
                "kp": int(round(v / BASE_V)),
                "min_surface": float(values.min()),
            }
        )
    out = pd.DataFrame(rows)
    out = out[out["controller"].isin(["noninertial_frozen", "noninertial_stage"])].copy()
    return out


def merge_kp_roots(roots_by_kp: dict[int, Path]) -> pd.DataFrame:
    frames = []
    for kp, run_root in roots_by_kp.items():
        df = load_summary(run_root)
        frames.append(df[df["kp"] == kp].copy())
    return pd.concat(frames, ignore_index=True)


def merge_step_roots(roots_by_kp: dict[int, Path]) -> pd.DataFrame:
    frames = []
    for kp, run_root in roots_by_kp.items():
        df = collect_step_min_surface(run_root)
        frames.append(df[df["kp"] == kp].copy())
    return pd.concat(frames, ignore_index=True)


def rel_improvement(frozen: float, stage: float, higher_is_better: bool = False) -> float:
    if abs(frozen) < 1e-12:
        return 0.0
    if higher_is_better:
        return (stage - frozen) / frozen * 100.0
    return (frozen - stage) / frozen * 100.0


def fmt_cell(frozen: float, stage: float, higher_is_better: bool = False) -> str:
    adv = rel_improvement(frozen, stage, higher_is_better=higher_is_better)
    return f"`{frozen:.6f} / {stage:.6f} ({adv:+.3f}%)`"


def fmt_adv_only(frozen: float, stage: float, higher_is_better: bool = False) -> str:
    adv = rel_improvement(frozen, stage, higher_is_better=higher_is_better)
    return f"`{adv:+.3f}%`"


def fmt_pair_only(frozen: float, stage: float) -> str:
    return f"`{frozen:.6f} / {stage:.6f}`"


def build_table_lines() -> list[str]:
    circle_roots = {
        1: ROOT / "results/circle_nocbf_kp124_seed1_100_2ctrl",
        2: ROOT / "results/circle_nocbf_kp124_seed1_100_2ctrl",
        3: ROOT / "results/circle_nocbf_kp3_seed1_100_2ctrl",
        4: ROOT / "results/circle_nocbf_kp124_seed1_100_2ctrl",
        6: ROOT / "results/circle_nocbf_kp6_seed1_100_2ctrl",
    }
    clean_roots = {
        1: ROOT / "results/figure_eight_clean_nocbf_kp124_seed1_100_2ctrl",
        2: ROOT / "results/figure_eight_clean_nocbf_kp124_seed1_100_2ctrl",
        3: ROOT / "results/figure_eight_clean_nocbf_kp3_seed1_100_2ctrl",
        4: ROOT / "results/figure_eight_clean_nocbf_kp124_seed1_100_2ctrl",
        6: ROOT / "results/figure_eight_clean_nocbf_kp6_seed1_100_2ctrl",
    }
    obs_roots = {
        1: ROOT / "results/figure_eight_fixedobs_cbf_kp124_seed1_100_2ctrl",
        2: ROOT / "results/figure_eight_fixedobs_cbf_kp124_seed1_100_2ctrl",
        3: ROOT / "results/figure_eight_fixedobs_cbf_kp3_seed1_100_2ctrl",
        4: ROOT / "results/figure_eight_fixedobs_cbf_kp124_seed1_100_2ctrl",
        6: ROOT / "results/figure_eight_fixedobs_cbf_kp6_seed1_100_2ctrl",
    }

    circle = merge_kp_roots(circle_roots)
    clean = merge_kp_roots(clean_roots)
    obs_summary = merge_kp_roots(obs_roots)
    obs_steps = merge_step_roots(obs_roots)

    lines = [
        "**Table I. Summary of Main Simulation Results**",
        "",
        "| Scene | Metric | `KP=1` | `KP=2` | `KP=3` | `KP=4` | `KP=6` |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]

    def pick(summary: pd.DataFrame, kp: int, field: str) -> tuple[float, float]:
        sub = summary[summary["kp"] == kp]
        frozen = float(sub[sub["controller"] == "noninertial_frozen"].iloc[0][field])
        stage = float(sub[sub["controller"] == "noninertial_stage"].iloc[0][field])
        return frozen, stage

    circle_cells = [fmt_adv_only(*pick(circle, kp, "tracking_rms_mean")) for kp in KP_VALUES]
    lines.append("| Circle, no CBF | Tracking-RMS improvement | " + " | ".join(circle_cells) + " |")

    clean_cells = [fmt_adv_only(*pick(clean, kp, "tracking_rms_mean")) for kp in KP_VALUES]
    lines.append("| Figure-eight, no CBF | Tracking-RMS improvement | " + " | ".join(clean_cells) + " |")

    tracking_cells = [fmt_adv_only(*pick(obs_summary, kp, "tracking_rms_mean")) for kp in KP_VALUES]
    lines.append("| Figure-eight, with CBF | Tracking-RMS improvement | " + " | ".join(tracking_cells) + " |")

    slack_cells = [fmt_adv_only(*pick(obs_summary, kp, "solver_slack_sum_mean")) for kp in KP_VALUES]
    lines.append("| Figure-eight, with CBF | Slack-sum reduction | " + " | ".join(slack_cells) + " |")

    safety_cells = []
    for kp in KP_VALUES:
        sub = obs_steps[obs_steps["kp"] == kp]
        frz_vals = sub[sub["controller"] == "noninertial_frozen"]["min_surface"]
        stg_vals = sub[sub["controller"] == "noninertial_stage"]["min_surface"]
        frz_fail = float((frz_vals < FAILURE_THRESHOLD).mean())
        stg_fail = float((stg_vals < FAILURE_THRESHOLD).mean())
        safety_cells.append(fmt_pair_only(frz_fail, stg_fail))
    lines.append(
        f"| Figure-eight, with CBF | Safety-violation rate (<{FAILURE_THRESHOLD:.3f} m) | "
        + " | ".join(safety_cells)
        + " |"
    )
    lines.append("")
    lines.append(
        "Tracking RMS and slack sum are reported as relative improvement only; "
        "the corresponding absolute means and across-seed dispersions are given in Sec. V-B."
    )
    lines.append(
        f"Safety-violation rate is reported as `Frozen / Stage` and uses a `{FAILURE_THRESHOLD:.3f} m` minimum-surface-distance threshold; at `KP=6` the binary metric saturates at `1.0 / 1.0`."
    )
    return lines


def update_paper_md(table_lines: list[str]) -> None:
    text = PAPER_MD.read_text(encoding="utf-8")
    pattern = re.compile(
        r"\*\*Table I\. Summary of Main Simulation Results\*\*\n.*?(?=\n## D\. Simulation-Side Computational Overhead)",
        re.S,
    )
    replacement = "\n".join(table_lines)
    new_text, count = pattern.subn(replacement, text, count=1)
    if count != 1:
        raise RuntimeError("Failed to locate Table I block in paper.md")
    PAPER_MD.write_text(new_text, encoding="utf-8")


def main() -> None:
    lines = build_table_lines()
    update_paper_md(lines)
    print("\n".join(lines))


if __name__ == "__main__":
    main()
