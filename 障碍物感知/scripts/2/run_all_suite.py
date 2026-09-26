#!/usr/bin/env python3
"""
One-shot entrypoint:
1) run_grid_suite.py
2) aggregate_suite.py
"""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run full suite (grid runs + aggregation) in one command."
    )
    parser.add_argument("--clean", action="store_true")
    parser.add_argument("--raw-csv", default="results/raw_runs.csv")
    parser.add_argument("--compare-csv", default="results/compare_grid.csv")
    parser.add_argument("--hash-csv", default="results/raw_runs_config_hashes.csv")
    parser.add_argument("--logs-root", default="results/logs/grid_suite")
    parser.add_argument(
        "--yaml",
        default="src/coni_mpc/parameters/num_sim_non_one_point.yaml",
    )
    parser.add_argument("--seed-start", type=int, default=1)
    parser.add_argument("--seed-end", type=int, default=100)
    parser.add_argument("--obs", default="20,40,60")
    parser.add_argument("--kappa", default="1.0,1.2,1.5")
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--timeout-sec", type=float, default=900.0)
    parser.add_argument("--continue-on-fail", action="store_true")
    parser.set_defaults(continue_on_fail=True)
    parser.add_argument("--rerun-failed", action="store_true")
    parser.add_argument("--fail-rate-threshold", type=float, default=0.0)
    parser.add_argument("--hard-slack-tol", type=float, default=1e-9)
    parser.add_argument("--active-uavs", default="0,1,2,3")
    parser.add_argument("--r", type=float, default=1.0)
    parser.add_argument("--v", type=float, default=0.5)
    parser.add_argument("--w", type=float, default=0.25)
    parser.add_argument("--start-z", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    scripts_dir = pathlib.Path(__file__).resolve().parent

    run_grid = scripts_dir / "run_grid_suite.py"
    aggregate = scripts_dir / "aggregate_suite.py"

    cmd_grid = [
        sys.executable,
        str(run_grid),
        "--raw-csv",
        args.raw_csv,
        "--hash-csv",
        args.hash_csv,
        "--logs-root",
        args.logs_root,
        "--yaml",
        args.yaml,
        "--seed-start",
        str(args.seed_start),
        "--seed-end",
        str(args.seed_end),
        "--obs",
        args.obs,
        "--kappa",
        args.kappa,
        "--jobs",
        str(args.jobs),
        "--timeout-sec",
        str(args.timeout_sec),
        "--fail-rate-threshold",
        str(args.fail_rate_threshold),
        "--hard-slack-tol",
        str(args.hard_slack_tol),
        "--active-uavs",
        args.active_uavs,
        "--r",
        str(args.r),
        "--v",
        str(args.v),
        "--w",
        str(args.w),
        "--start-z",
        str(args.start_z),
    ]
    if args.clean:
        cmd_grid.append("--clean")
    if args.continue_on_fail:
        cmd_grid.append("--continue-on-fail")
    if args.rerun_failed:
        cmd_grid.append("--rerun-failed")

    print("Running grid suite:", flush=True)
    print(" ".join(cmd_grid), flush=True)
    rc = subprocess.run(cmd_grid).returncode
    if rc != 0:
        return rc

    cmd_agg = [
        sys.executable,
        str(aggregate),
        "--raw-csv",
        args.raw_csv,
        "--out",
        args.compare_csv,
    ]
    if args.clean:
        cmd_agg.append("--clean")

    print("Running aggregation:", flush=True)
    print(" ".join(cmd_agg), flush=True)
    rc = subprocess.run(cmd_agg).returncode
    return rc


if __name__ == "__main__":
    sys.exit(main())
