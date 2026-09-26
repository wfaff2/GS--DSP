#!/usr/bin/env python3
"""
Batch-run roslaunch experiments and aggregate metrics into a single CSV.

Minimal usage:
  source devel/setup.bash
  python3 scripts/run_metrics_suite.py --out results/metrics_summary.csv
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class RunCase:
    batch: str
    obstacle_count: int
    kappa: float
    seed: Optional[int]
    use_dynamic_obs: bool
    warm_start: bool
    start_z: Optional[float]
    metrics_debug: bool


def _bool_str(v: bool) -> str:
    return "true" if v else "false"


def _parse_active_uavs(text: str) -> List[int]:
    parts = [p.strip() for p in text.split(",") if p.strip()]
    return [int(p) for p in parts]


def _run_cmd(cmd: List[str], timeout_sec: Optional[float]) -> int:
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    try:
        if timeout_sec is None or timeout_sec <= 0:
            return proc.wait()
        return proc.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        return 124
    except KeyboardInterrupt:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        raise


def _set_rosparam(param: str, value: str) -> int:
    return subprocess.run(["rosparam", "set", param, value]).returncode


def _roscore_is_running() -> bool:
    return (
        subprocess.run(
            ["rosparam", "list"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ).returncode
        == 0
    )


def _start_roscore_if_needed() -> Optional[subprocess.Popen]:
    if _roscore_is_running():
        return None
    proc = subprocess.Popen(
        ["roscore"],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    deadline = time.time() + 10.0
    while time.time() < deadline:
        if _roscore_is_running():
            return proc
        time.sleep(0.2)
    os.killpg(proc.pid, signal.SIGTERM)
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
    raise RuntimeError("roscore failed to start within 10 seconds")


def _build_cases(args: argparse.Namespace) -> List[RunCase]:
    cases: List[RunCase] = []

    include_stress = args.only_stress or not (args.only_dyn or args.only_warm)
    include_dyn = args.only_dyn or not (args.only_stress or args.only_warm)
    include_warm = args.only_warm or not (args.only_stress or args.only_dyn)

    if include_stress:
        for obstacle_count in (20, 40, 60):
            for kappa in (1, 1.2, 1.5):
                cases.append(
                    RunCase(
                        batch="stress",
                        obstacle_count=obstacle_count,
                        kappa=kappa,
                        seed=None if args.seed <= 0 else args.seed,
                        use_dynamic_obs=False,
                        warm_start=True,
                        start_z=None,
                        metrics_debug=args.metrics_debug,
                    )
                )

    if include_dyn:
        cases.append(
            RunCase(
                batch="dyn",
                obstacle_count=40,
                kappa=1.0,
                seed=None if args.seed <= 0 else args.seed,
                use_dynamic_obs=True,
                warm_start=True,
                start_z=args.start_z,
                metrics_debug=args.metrics_debug,
            )
        )

    if include_warm:
        warm_start_values = (True, False)
        if include_stress:
            # Avoid duplicating the stress baseline (warm_start=True) case.
            warm_start_values = (False,)
        for warm_start in warm_start_values:
                cases.append(
                    RunCase(
                        batch="warm",
                        obstacle_count=40,
                        kappa=1.0,
                        seed=None if args.seed <= 0 else args.seed,
                        use_dynamic_obs=False,
                        warm_start=warm_start,
                        start_z=None,
                        metrics_debug=args.metrics_debug,
                    )
                )

    return cases


def _format_run_tag(ts: str, case: RunCase, index: int) -> str:
    kappa_str = f"{case.kappa:.1f}"
    seed_str = "yaml" if case.seed is None else str(case.seed)
    return (
        f"{ts}_c{case.obstacle_count}_k{kappa_str}_s{seed_str}"
        f"_dyn{int(case.use_dynamic_obs)}_ws{int(case.warm_start)}_{index:02d}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run coni_mpc metrics sweeps via roslaunch and aggregate CSV output."
    )
    parser.add_argument(
        "--out",
        default="results/metrics_summary.csv",
        help="Output metrics CSV (same path passed to every run).",
    )
    parser.add_argument(
        "--keep-existing",
        action="store_true",
        help="Do not delete existing output CSV before running.",
    )
    parser.add_argument(
        "--continue-on-fail",
        action="store_true",
        help="Continue even if a run fails or times out.",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=None,
        help="Per-run timeout in seconds (default: no timeout).",
    )
    parser.add_argument("--only-stress", action="store_true", help="Run only stress sweep.")
    parser.add_argument("--only-dyn", action="store_true", help="Run only dynamic-obstacle case.")
    parser.add_argument("--only-warm", action="store_true", help="Run only warm-start ablation.")
    parser.add_argument(
        "--seed",
        type=int,
        default=-1,
        help="Seed for stress sweep. Use a positive value to override; <=0 uses YAML.",
    )
    parser.add_argument(
        "--start-z",
        type=float,
        default=2.7,
        help="dynamic_obs/start_z for dynamic-obstacle case (default: 2.7).",
    )
    parser.add_argument(
        "--metrics-debug",
        action="store_true",
        help="Set /num_sim_non_one_point_node/metrics/debug true for all runs.",
    )
    parser.add_argument(
        "--active-uavs",
        type=str,
        default=None,
        help="Comma-separated active UAV indices (e.g., 0 or 0,1,2,3). "
             "Default: [0] for stress runs; otherwise node defaults.",
    )
    parser.add_argument("--r", type=float, default=1.0, help="Initial offset r (default: 1.0).")
    parser.add_argument("--v", type=float, default=0.5, help="Car speed v (default: 0.5).")
    parser.add_argument("--w", type=float, default=0.25, help="Car yaw rate w (default: 0.25).")
    parser.add_argument(
        "--rviz",
        action="store_true",
        help="Launch RViz for each run (default: off for batch).",
    )

    args = parser.parse_args()
    only_flags = [args.only_stress, args.only_dyn, args.only_warm]
    if sum(1 for f in only_flags if f) > 1:
        print("ERROR: --only-stress/--only-dyn/--only-warm are mutually exclusive.", file=sys.stderr)
        return 2

    out_path = os.path.abspath(args.out)
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    if os.path.exists(out_path) and not args.keep_existing:
        os.remove(out_path)

    cases = _build_cases(args)
    if not cases:
        print("No runs selected. Check --only-* flags.", file=sys.stderr)
        return 2

    timestamp = time.strftime("%Y%m%d-%H%M%S")

    print(f"Runs: {len(cases)} | output: {out_path}")

    roscore_proc = None
    try:
        roscore_proc = _start_roscore_if_needed()

        for i, case in enumerate(cases, start=1):
            run_tag = _format_run_tag(timestamp, case, i)

            print(
                f"\n[START] {i}/{len(cases)} batch={case.batch} run_tag={run_tag}",
                flush=True,
            )

            rc = _set_rosparam(
                "/num_sim_non_one_point_node/metrics/debug",
                _bool_str(case.metrics_debug),
            )
            if rc != 0:
                print("rosparam set metrics/debug failed", file=sys.stderr)
                if not args.continue_on_fail:
                    return rc

            if case.use_dynamic_obs and case.start_z is not None:
                rc = _set_rosparam(
                    "/num_sim_non_one_point_node/dynamic_obs/start_z",
                    str(case.start_z),
                )
                if rc != 0:
                    print("rosparam set dynamic_obs/start_z failed", file=sys.stderr)
                    if not args.continue_on_fail:
                        return rc

            active_uavs_list = None
            if args.active_uavs is not None:
                active_uavs_list = _parse_active_uavs(args.active_uavs)
            else:
                active_uavs_list = [1, 2, 3]
            if active_uavs_list:
                active_uavs_value = "[" + ",".join(str(v) for v in active_uavs_list) + "]"
                rc = _set_rosparam(
                    "/num_sim_non_one_point_node/active_uavs",
                    active_uavs_value,
                )
                if rc != 0:
                    print("rosparam set active_uavs failed", file=sys.stderr)
                    if not args.continue_on_fail:
                        return rc

            cmd = [
                "roslaunch",
                "coni_mpc",
                "num_sim_non_one_point.launch",
                f"r:={args.r}",
                f"v:={args.v}",
                f"w:={args.w}",
                f"use_rviz:={_bool_str(args.rviz)}",
                f"obstacle_count:={case.obstacle_count}",
                *( [] if case.seed is None else [f"seed:={case.seed}"] ),
                f"kappa:={case.kappa}",
                f"use_dynamic_obs:={_bool_str(case.use_dynamic_obs)}",
                f"warm_start:={_bool_str(case.warm_start)}",
                f"metrics_csv:={out_path}",
                f"run_tag:={run_tag}",
            ]

            print(f"Command: {' '.join(cmd)}", flush=True)
            try:
                rc = _run_cmd(cmd, args.timeout_sec)
            except KeyboardInterrupt:
                print("\nInterrupted. Killed current roslaunch process group.")
                return 130

            print(f"[END] run_tag={run_tag} returncode={rc}", flush=True)

            if rc != 0 and not args.continue_on_fail:
                return rc
    finally:
        if roscore_proc is not None:
            os.killpg(roscore_proc.pid, signal.SIGTERM)
            try:
                roscore_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(roscore_proc.pid, signal.SIGKILL)

    return 0


if __name__ == "__main__":
    sys.exit(main())
