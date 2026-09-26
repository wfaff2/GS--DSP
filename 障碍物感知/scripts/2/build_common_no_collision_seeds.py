#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

REQUIRED_CASES = [
    "inertial_frozen",
    "noninertial_frozen",
    "noninertial_stage",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract kp,seed pairs where all three methods are collision-free."
    )
    parser.add_argument(
        "seed_metrics_csv",
        type=Path,
        help="Seed-level three-case metrics CSV produced by aggregate_kp_three_case_metrics.py",
    )
    parser.add_argument(
        "--out",
        type=Path,
        required=True,
        help="Output CSV path for the common no-collision seed list.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    seed_metrics_csv = args.seed_metrics_csv.resolve()
    out_path = args.out.resolve()

    df = pd.read_csv(seed_metrics_csv)
    required_cols = {"kp", "seed", "case", "collision"}
    missing = required_cols - set(df.columns)
    if missing:
        raise SystemExit(f"Missing columns in {seed_metrics_csv}: {sorted(missing)}")

    df = df[df["case"].isin(REQUIRED_CASES)].copy()
    if df.empty:
        raise SystemExit(f"No required three-case rows found in {seed_metrics_csv}")

    wide = (
        df.pivot_table(
            index=["kp", "seed"],
            columns="case",
            values="collision",
            aggfunc="first",
        )
        .reset_index()
    )

    missing_cases = [case for case in REQUIRED_CASES if case not in wide.columns]
    if missing_cases:
        raise SystemExit(
            f"Some required cases are absent from {seed_metrics_csv}: {missing_cases}"
        )

    keep_mask = (wide[REQUIRED_CASES].fillna(1.0) <= 0.0).all(axis=1)
    keep_df = wide.loc[keep_mask, ["kp", "seed"]].sort_values(["kp", "seed"]).copy()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    keep_df.to_csv(out_path, index=False)
    print(f"[INFO] wrote {out_path}")
    if keep_df.empty:
        print("[WARN] no common no-collision seeds found.")
        return

    counts = keep_df.groupby("kp")["seed"].size().reset_index(name="count")
    print(counts.to_string(index=False))


if __name__ == "__main__":
    main()
