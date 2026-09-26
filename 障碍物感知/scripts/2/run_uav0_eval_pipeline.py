#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import importlib.util
import math
import pathlib
import shutil
import statistics
import sys
import time
from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
from types import SimpleNamespace

NA = "NA"
WORKSPACE = pathlib.Path(__file__).resolve().parent.parent
MOD_PATH = WORKSPACE / "scripts" / "run_grid_suite.py"

spec = importlib.util.spec_from_file_location("run_grid_suite_mod_uav0", MOD_PATH)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)

PAIR_KEY_FIELDS = [
    "seed_env",
    "seed_traj",
    "obstacle_count",
    "kappa",
    "requested_r",
    "requested_v",
    "requested_w",
    "start_z",
    "noise_seed",
    "use_dynamic_obs",
    "warm_start",
    "active_uavs_csv",
    "sim_control_dt_sec",
    "prediction_dt_sec",
    "prediction_steps",
    "turning_omega_thr",
    "safety_variant",
]

PAIR_CHECK_FIELDS = PAIR_KEY_FIELDS + [
    "max_abs_ugv_yaw_rate",
]

PER_RUN_HEADER = [
    "frame_mode",
    "frame_mode_effective",
    "run_tag",
    "metrics_path",
    "parse_ok",
    "safety_variant",
    "obstacle_count",
    "seed",
    "seed_env",
    "seed_traj",
    "traj_id",
    "valid_pair_key",
    "kappa",
    "requested_r",
    "requested_v",
    "requested_w",
    "start_z",
    "noise_seed",
    "use_dynamic_obs",
    "warm_start",
    "active_uavs_csv",
    "sim_control_dt_sec",
    "prediction_dt_sec",
    "prediction_steps",
    "max_abs_ugv_yaw_rate",
    "turning_omega_thr",
    "collision",
    "tracking_rms",
    "tracking_rms_high",
    "tracking_rms_all",
    "tracking_rms_track_only",
    "tracking_rms_cbf_active",
    "solver_tracking_rms_avoid",
    "solver_tracking_rms_avoid_samples",
    "c66_metric",
    "high_c_samples",
    "min_h_high_c",
    "slack_sum_high_c",
    "tracking_rms_high_c",
    "fail_rate_high_c",
    "fail_rate",
    "min_h",
    "solver_min_h",
    "solver_active_mean_h",
    "solver_min_planar_clearance",
    "slack_sum",
    "max_slack",
    "cbf_active_samples",
    "cbf_active_ratio",
    "track_only_samples",
    "track_only_ratio",
    "fail_rate_turn",
    "fail_rate_straight",
    "min_h_turn",
    "min_h_straight",
    "slack_sum_turn",
    "slack_sum_straight",
    "max_slack_turn",
    "max_slack_straight",
    "tracking_rms_all_turn",
    "tracking_rms_all_straight",
]

COMPARE_HEADER = [
    "valid_pair_key",
    "seed_env",
    "seed_traj",
    "obstacle_count",
    "kappa",
    "requested_r",
    "requested_v",
    "requested_w",
    "start_z",
    "noise_seed",
    "use_dynamic_obs",
    "warm_start",
    "active_uavs_csv",
    "sim_control_dt_sec",
    "prediction_dt_sec",
    "prediction_steps",
    "turning_omega_thr",
    "safety_variant",
    "traj_id_non",
    "traj_id_in",
    "traj_match",
    "settings_match",
    "valid_pair",
    "pair_status",
    "noninertial_run_tag",
    "noninertial_collision",
    "noninertial_tracking_rms",
    "noninertial_tracking_rms_high",
    "noninertial_tracking_rms_all",
    "noninertial_tracking_rms_track_only",
    "noninertial_tracking_rms_cbf_active",
    "noninertial_solver_tracking_rms_avoid",
    "noninertial_solver_tracking_rms_avoid_samples",
    "noninertial_c66_metric",
    "noninertial_high_c_samples",
    "noninertial_min_h_high_c",
    "noninertial_slack_sum_high_c",
    "noninertial_tracking_rms_high_c",
    "noninertial_fail_rate_high_c",
    "noninertial_fail_rate",
    "noninertial_min_h",
    "noninertial_solver_min_h",
    "noninertial_solver_active_mean_h",
    "noninertial_solver_min_planar_clearance",
    "noninertial_slack_sum",
    "noninertial_max_slack",
    "noninertial_fail_rate_turn",
    "noninertial_fail_rate_straight",
    "noninertial_min_h_turn",
    "noninertial_min_h_straight",
    "noninertial_slack_sum_turn",
    "noninertial_slack_sum_straight",
    "noninertial_max_slack_turn",
    "noninertial_max_slack_straight",
    "noninertial_tracking_rms_all_turn",
    "noninertial_tracking_rms_all_straight",
    "inertial_run_tag",
    "inertial_collision",
    "inertial_tracking_rms",
    "inertial_tracking_rms_high",
    "inertial_tracking_rms_all",
    "inertial_tracking_rms_track_only",
    "inertial_tracking_rms_cbf_active",
    "inertial_solver_tracking_rms_avoid",
    "inertial_solver_tracking_rms_avoid_samples",
    "inertial_c66_metric",
    "inertial_high_c_samples",
    "inertial_min_h_high_c",
    "inertial_slack_sum_high_c",
    "inertial_tracking_rms_high_c",
    "inertial_fail_rate_high_c",
    "inertial_fail_rate",
    "inertial_min_h",
    "inertial_solver_min_h",
    "inertial_solver_active_mean_h",
    "inertial_solver_min_planar_clearance",
    "inertial_slack_sum",
    "inertial_max_slack",
    "inertial_fail_rate_turn",
    "inertial_fail_rate_straight",
    "inertial_min_h_turn",
    "inertial_min_h_straight",
    "inertial_slack_sum_turn",
    "inertial_slack_sum_straight",
    "inertial_max_slack_turn",
    "inertial_max_slack_straight",
    "inertial_tracking_rms_all_turn",
    "inertial_tracking_rms_all_straight",
    "delta_fail_rate",
    "delta_min_h",
    "delta_slack_sum",
    "delta_max_slack",
    "delta_tracking_rms_all",
    "delta_fail_rate_high_c",
    "delta_min_h_high_c",
    "delta_slack_sum_high_c",
    "delta_tracking_rms_high",
    "delta_solver_tracking_rms_avoid_non_minus_inertial",
    "delta_solver_min_h_non_minus_inertial",
    "delta_solver_min_planar_clearance_non_minus_inertial",
]

SUMMARY_HEADER = [
    "frame_mode",
    "aggregation_scope",
    "n_runs_total",
    "n_runs_aggregated",
    "n_pairs_total",
    "n_pairs_valid",
    "n_pairs_invalid_or_unpaired",
    "collisions",
    "collision_rate",
    "mean_tracking_rms_all",
    "median_tracking_rms_all",
    "p95_tracking_rms_all",
    "mean_tracking_rms_track_only",
    "median_tracking_rms_track_only",
    "p95_tracking_rms_track_only",
    "mean_tracking_rms_cbf_active",
    "median_tracking_rms_cbf_active",
    "p95_tracking_rms_cbf_active",
    "mean_solver_tracking_rms_avoid",
    "median_solver_tracking_rms_avoid",
    "p95_solver_tracking_rms_avoid",
    "mean_tracking_rms_high",
    "median_tracking_rms_high",
    "p95_tracking_rms_high",
    "mean_fail_rate",
    "median_fail_rate",
    "p95_fail_rate",
    "mean_fail_rate_high_c",
    "median_fail_rate_high_c",
    "p95_fail_rate_high_c",
    "mean_min_h",
    "median_min_h",
    "p05_min_h",
    "mean_solver_min_h",
    "median_solver_min_h",
    "p05_solver_min_h",
    "mean_solver_active_mean_h",
    "median_solver_active_mean_h",
    "p05_solver_active_mean_h",
    "mean_solver_min_planar_clearance",
    "median_solver_min_planar_clearance",
    "p05_solver_min_planar_clearance",
    "mean_min_h_high_c",
    "median_min_h_high_c",
    "p05_min_h_high_c",
    "mean_slack_sum",
    "median_slack_sum",
    "p95_slack_sum",
    "mean_slack_sum_high_c",
    "median_slack_sum_high_c",
    "p95_slack_sum_high_c",
    "mean_max_slack",
    "median_max_slack",
    "p95_max_slack",
    "mean_fail_rate_turn",
    "mean_fail_rate_straight",
    "mean_min_h_turn",
    "mean_min_h_straight",
    "mean_slack_sum_turn",
    "mean_slack_sum_straight",
    "mean_max_slack_turn",
    "mean_max_slack_straight",
    "mean_tracking_rms_all_turn",
    "mean_tracking_rms_all_straight",
    "pct_noninertial_better_solver_tracking_rms_avoid",
]


def safe_float(value: object) -> float:
    try:
        text = str(value).strip()
        if not text or text.upper() == NA:
            return math.nan
        return float(text)
    except Exception:
        return math.nan


def safe_int(value: object, default: int = 0) -> int:
    try:
        text = str(value).strip()
        if not text or text.upper() == NA:
            return default
        return int(float(text))
    except Exception:
        return default


def fmt_float(value: float) -> str:
    return NA if not math.isfinite(value) else f"{value:.6f}"


def fmt_int(value: int) -> str:
    return str(int(value))


def derive_tracking_rms_high(existing_high: object = NA,
                             legacy_high_c: object = NA) -> str:
    high_value = safe_float(existing_high)
    if math.isfinite(high_value):
        return fmt_float(high_value)
    legacy_value = safe_float(legacy_high_c)
    if math.isfinite(legacy_value):
        return fmt_float(legacy_value)
    return NA


_LEGACY_METRIC_WARNINGS: set[str] = set()
_MISSING_CLEAN_METRIC_WARNINGS: set[str] = set()


def metric_with_fallback(metrics_row: dict[str, str], clean_key: str, legacy_key: str) -> str:
    clean_value = str(metrics_row.get(clean_key, "")).strip()
    if clean_value and clean_value.upper() != NA:
        return clean_value
    legacy_value = str(metrics_row.get(legacy_key, "")).strip()
    if legacy_value and legacy_value.upper() != NA:
        warning_key = f"{clean_key}->{legacy_key}"
        if warning_key not in _LEGACY_METRIC_WARNINGS:
            print(
                f"[WARN] metrics.csv is missing {clean_key}; falling back to legacy {legacy_key}",
                file=sys.stderr,
                flush=True,
            )
            _LEGACY_METRIC_WARNINGS.add(warning_key)
        return legacy_value
    return NA


def clean_metric_or_warn(metrics_row: dict[str, str], clean_key: str) -> str:
    clean_value = str(metrics_row.get(clean_key, "")).strip()
    if clean_value and clean_value.upper() != NA:
        return clean_value
    if clean_key not in _MISSING_CLEAN_METRIC_WARNINGS:
        print(
            f"[WARN] metrics.csv is missing clean field {clean_key}",
            file=sys.stderr,
            flush=True,
        )
        _MISSING_CLEAN_METRIC_WARNINGS.add(clean_key)
    return NA


def write_csv(path: pathlib.Path, header: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, NA) for k in header})


def parse_list_int(text: str) -> list[int]:
    return [int(part.strip()) for part in text.split(",") if part.strip()]


def parse_list_float(text: str) -> list[float]:
    return [float(part.strip()) for part in text.split(",") if part.strip()]


def normalize_key_value(field: str, value: object) -> str:
    if field in {
        "kappa",
        "requested_r",
        "requested_v",
        "requested_w",
        "start_z",
        "sim_control_dt_sec",
        "prediction_dt_sec",
        "turning_omega_thr",
        "max_abs_ugv_yaw_rate",
    }:
        return fmt_float(safe_float(value))
    if field in {
        "obstacle_count",
        "seed",
        "seed_env",
        "seed_traj",
        "noise_seed",
        "prediction_steps",
        "collision",
        "parse_ok",
        "cbf_active_samples",
        "track_only_samples",
    }:
        return fmt_int(safe_int(value))
    return str(value).strip() if str(value).strip() else NA


def build_valid_pair_key(row: dict[str, object]) -> str:
    parts = [f"{field}={normalize_key_value(field, row.get(field, NA))}" for field in PAIR_KEY_FIELDS]
    return "|".join(parts)


def read_scope_metrics(metrics_csv: pathlib.Path, scope: str, run_tag: str) -> dict[str, str] | None:
    if not metrics_csv.exists():
        return None
    matched: dict[str, str] | None = None
    with metrics_csv.open("r", encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            if row.get("run_tag") == run_tag and row.get("scope") == scope:
                matched = row
    return matched


def quantile(values: list[float], q: float) -> float:
    finite = sorted(v for v in values if math.isfinite(v))
    if not finite:
        return math.nan
    if len(finite) == 1:
        return finite[0]
    pos = max(0.0, min(1.0, q)) * float(len(finite) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return finite[lo]
    frac = pos - float(lo)
    return finite[lo] * (1.0 - frac) + finite[hi] * frac


def mean(values: list[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return math.nan if not finite else statistics.mean(finite)


def median(values: list[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return math.nan if not finite else statistics.median(finite)


def pair_status(non: dict[str, str] | None, inertial: dict[str, str] | None, traj_match: int, settings_match: int) -> str:
    if non is None:
        return "missing_noninertial"
    if inertial is None:
        return "missing_inertial"
    if traj_match != 1:
        return "traj_mismatch"
    if settings_match != 1:
        return "setting_mismatch"
    return "ok"


def compare_field_equal(non: dict[str, str], inertial: dict[str, str], field: str) -> bool:
    return normalize_key_value(field, non.get(field, NA)) == normalize_key_value(field, inertial.get(field, NA))


def maybe_delta(valid_pair: int, inertial_value: object, non_value: object, flip_non_minus_in: bool = False) -> str:
    if valid_pair != 1:
        return NA
    v_in = safe_float(inertial_value)
    v_non = safe_float(non_value)
    if not math.isfinite(v_in) or not math.isfinite(v_non):
        return NA
    delta = (v_non - v_in) if flip_non_minus_in else (v_in - v_non)
    return fmt_float(delta)


def write_readme(out_root: pathlib.Path) -> None:
    text = """# UAV0 Evaluation Pipeline

This directory contains a paired `UAV0-only` evaluation between `noninertial` and `inertial`.

Fairness rules:

- Runs are paired by `valid_pair_key`.
- `valid_pair = 1` only when the pair has identical controlled settings and `traj_id_non == traj_id_in`.
- `traj_id` is a SHA1 hash of the deterministically serialized UGV world-position sequence `{p_g^W(k)}` with fixed 6-decimal formatting.
- `seed_traj` is currently set equal to `seed_env` because the trajectory generator does not expose a separate seed.

Metric notes:

- `solver_*` fields are the clean solver-side metrics. They are evaluated only on the obstacle actually constrained by the MPC solver on that step.
- `diag_*` fields are diagnostics only. They come from nearest-obstacle bookkeeping and are not used for paper-facing summaries or deltas here.
- Legacy unprefixed barrier columns are treated as compatibility aliases; this pipeline reads `solver_*` first and warns before falling back.
- `tracking_rms_all` is the per-run RMS over all controller steps.
- `tracking_rms_high` uses the legacy `c66` high-curvature subset RMS; old CSVs fall back to `tracking_rms_high_c`.
- `tracking_rms_track_only` uses the clean solver-side predicate: steps with no active obstacle constrained by the solver.
- `solver_tracking_rms_avoid` is the tracking RMS restricted to solver-active barrier samples. Older CSVs may fall back to `tracking_rms_cbf_active` with a warning.
- `solver_tracking_rms_avoid_samples` is the number of valid samples contributing to `solver_tracking_rms_avoid`.
- `solver_min_h` is the minimum solver-side barrier value on the injected active obstacle.
- `solver_active_mean_h` is the mean solver-side barrier value on the injected active obstacle over solver-active samples.
- `solver_min_planar_clearance` is the minimum planar clearance under the same solver barrier geometry used by `solver_h`; it is not a full 3D Euclidean clearance.
- `tracking_rms_cbf_active` is kept as a legacy compatibility alias and is not the preferred paper-facing field.
- `fail_rate` and `solver_fail_rate` mean solver failure rate only; they are not safety-violation rates.
- `min_h`/`min_cbf` are solver-side active-obstacle margins in new CSVs. Nearest-obstacle diagnostics are exported separately as `diag_min_h_any` and `diag_min_cbf_any`.
- Turning vs straight uses `omega_thr = 0.5 * max_k |omega_g(k)|`.
- `metrics_summary.csv` is an aggregation of run-level metrics over valid pairs; it is not a recomputation over all timesteps across all runs.
"""
    (out_root / "README_metrics.md").write_text(text, encoding="utf-8")


def build_cases(seed_start: int, seed_end: int, obs_values: list[int], kappa_values: list[float]) -> list[mod.Case]:
    cases: list[mod.Case] = []
    idx = 0
    for obs in obs_values:
        for kappa in kappa_values:
            for seed in range(seed_start, seed_end + 1):
                for frame_mode in ("noninertial", "inertial"):
                    idx += 1
                    cases.append(
                        mod.Case(
                            run_index=idx,
                            frame_mode=frame_mode,
                            safety_variant="A2_soft_cbf",
                            obs=int(obs),
                            kappa=float(kappa),
                            seed=int(seed),
                        )
                    )
    return cases


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run UAV0-only paired evaluation and aggregate four CSV reports.")
    parser.add_argument("--out-root", default="")
    parser.add_argument("--seed-start", type=int, default=1)
    parser.add_argument("--seed-end", type=int, default=5)
    parser.add_argument("--obs", default="40")
    parser.add_argument("--kappa", default="1.0")
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--timeout-sec", type=float, default=900.0)
    parser.add_argument("--r", type=float, default=1.0)
    parser.add_argument("--v", type=float, default=0.5)
    parser.add_argument("--w", type=float, default=0.25)
    parser.add_argument("--start-z", type=float, default=2.0)
    parser.add_argument("--active-uavs", default="0")
    parser.add_argument("--yaml", default="src/coni_mpc/parameters/num_sim_non_one_point.yaml")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    obs_values = parse_list_int(args.obs)
    kappa_values = parse_list_float(args.kappa)
    active_uavs = parse_list_int(args.active_uavs)

    suite_timestamp = time.strftime("%Y%m%d-%H%M%S")
    if args.out_root:
        out_root = (WORKSPACE / args.out_root).resolve() if not pathlib.Path(args.out_root).is_absolute() else pathlib.Path(args.out_root)
    else:
        out_root = WORKSPACE / f"results/uav0_eval_pipeline_{suite_timestamp}"
    logs_root = out_root / "logs"
    out_root.mkdir(parents=True, exist_ok=True)
    logs_root.mkdir(parents=True, exist_ok=True)

    yaml_abs = (WORKSPACE / args.yaml).resolve() if not pathlib.Path(args.yaml).is_absolute() else pathlib.Path(args.yaml)
    soft_slack_max, soft_rho, yaml_hash_payload = mod._load_soft_defaults(yaml_abs)
    safety_cfg = mod._make_safety_map(soft_slack_max=soft_slack_max, soft_rho=soft_rho)["A2_soft_cbf"]
    mod._require_ros_setup()

    run_args = SimpleNamespace(
        r=args.r,
        v=args.v,
        w=args.w,
        start_z=args.start_z,
        active_uavs_list=active_uavs,
        yaml_abs=yaml_abs,
        timeout_sec=args.timeout_sec,
        fail_rate_threshold=1.0,
        hard_slack_tol=1e-9,
    )

    cases = build_cases(args.seed_start, args.seed_end, obs_values, kappa_values)
    total = len(cases)
    print(f"[INFO] total_cases={total} out_root={out_root}", flush=True)

    per_run_rows: list[dict[str, object]] = []
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as executor:
        inflight: dict[object, mod.Case] = {}
        case_iter = iter(cases)

        def submit_next() -> bool:
            try:
                case = next(case_iter)
            except StopIteration:
                return False
            fut = executor.submit(
                mod._run_case,
                case,
                args=run_args,
                safety_cfg=safety_cfg,
                suite_timestamp=suite_timestamp,
                logs_root=logs_root,
                yaml_hash_payload=yaml_hash_payload,
            )
            inflight[fut] = case
            return True

        for _ in range(max(1, args.jobs)):
            if not submit_next():
                break

        completed = 0
        while inflight:
            done, _ = wait(list(inflight.keys()), return_when=FIRST_COMPLETED)
            for fut in done:
                case = inflight.pop(fut)
                completed += 1
                run_tag = ""
                metrics_path = pathlib.Path()
                row: dict[str, object]
                try:
                    result = fut.result()
                    run_tag = str(result["run_tag"])
                    metrics_path = logs_root / run_tag / "metrics.csv"
                    metrics_row = read_scope_metrics(metrics_path, "uav0", run_tag)
                    if metrics_row is None:
                        row = {
                            "frame_mode": case.frame_mode,
                            "frame_mode_effective": NA,
                            "run_tag": run_tag,
                            "metrics_path": str(metrics_path.resolve()),
                            "parse_ok": "0",
                            "safety_variant": case.safety_variant,
                            "obstacle_count": str(case.obs),
                            "seed": str(case.seed),
                            "seed_env": str(case.seed),
                            "seed_traj": str(case.seed),
                            "traj_id": NA,
                            "kappa": fmt_float(case.kappa),
                            "requested_r": fmt_float(args.r),
                            "requested_v": fmt_float(args.v),
                            "requested_w": fmt_float(args.w),
                            "start_z": fmt_float(args.start_z),
                            "noise_seed": str(case.seed),
                            "use_dynamic_obs": "0",
                            "warm_start": "1",
                            "active_uavs_csv": ";".join(str(x) for x in active_uavs),
                            "sim_control_dt_sec": NA,
                            "prediction_dt_sec": NA,
                            "prediction_steps": NA,
                            "max_abs_ugv_yaw_rate": NA,
                            "turning_omega_thr": NA,
                            "collision": NA,
                            "tracking_rms": NA,
                            "tracking_rms_high": NA,
                            "tracking_rms_all": NA,
                            "tracking_rms_track_only": NA,
                            "tracking_rms_cbf_active": NA,
                            "solver_tracking_rms_avoid": NA,
                            "solver_tracking_rms_avoid_samples": NA,
                            "c66_metric": NA,
                            "high_c_samples": NA,
                            "min_h_high_c": NA,
                            "slack_sum_high_c": NA,
                            "tracking_rms_high_c": NA,
                            "fail_rate_high_c": NA,
                            "fail_rate": NA,
                            "min_h": NA,
                            "solver_min_h": NA,
                            "solver_active_mean_h": NA,
                            "solver_min_planar_clearance": NA,
                            "slack_sum": NA,
                            "max_slack": NA,
                            "cbf_active_samples": NA,
                            "cbf_active_ratio": NA,
                            "track_only_samples": NA,
                            "track_only_ratio": NA,
                            "fail_rate_turn": NA,
                            "fail_rate_straight": NA,
                            "min_h_turn": NA,
                            "min_h_straight": NA,
                            "slack_sum_turn": NA,
                            "slack_sum_straight": NA,
                            "max_slack_turn": NA,
                            "max_slack_straight": NA,
                            "tracking_rms_all_turn": NA,
                            "tracking_rms_all_straight": NA,
                        }
                    else:
                        row = {
                            "frame_mode": case.frame_mode,
                            "frame_mode_effective": metrics_row.get("frame_mode_effective", NA) or NA,
                            "run_tag": run_tag,
                            "metrics_path": str(metrics_path.resolve()),
                            "parse_ok": "1",
                            "safety_variant": metrics_row.get("safety_variant", case.safety_variant) or case.safety_variant,
                            "obstacle_count": normalize_key_value("obstacle_count", metrics_row.get("obstacle_count", case.obs)),
                            "seed": normalize_key_value("seed", metrics_row.get("seed", case.seed)),
                            "seed_env": normalize_key_value("seed_env", metrics_row.get("seed_env", case.seed)),
                            "seed_traj": normalize_key_value("seed_traj", metrics_row.get("seed_traj", metrics_row.get("seed_env", case.seed))),
                            "traj_id": metrics_row.get("traj_id", NA) or NA,
                            "kappa": normalize_key_value("kappa", metrics_row.get("kappa", case.kappa)),
                            "requested_r": normalize_key_value("requested_r", metrics_row.get("requested_r", args.r)),
                            "requested_v": normalize_key_value("requested_v", metrics_row.get("requested_v", args.v)),
                            "requested_w": normalize_key_value("requested_w", metrics_row.get("requested_w", args.w)),
                            "start_z": normalize_key_value("start_z", metrics_row.get("start_z", args.start_z)),
                            "noise_seed": normalize_key_value("noise_seed", metrics_row.get("noise_seed", case.seed)),
                            "use_dynamic_obs": normalize_key_value("use_dynamic_obs", metrics_row.get("use_dynamic_obs", 0)),
                            "warm_start": normalize_key_value("warm_start", metrics_row.get("warm_start", 1)),
                            "active_uavs_csv": metrics_row.get("active_uavs_csv", ";".join(str(x) for x in active_uavs)) or NA,
                            "sim_control_dt_sec": normalize_key_value("sim_control_dt_sec", metrics_row.get("sim_control_dt_sec", NA)),
                            "prediction_dt_sec": normalize_key_value("prediction_dt_sec", metrics_row.get("prediction_dt_sec", NA)),
                            "prediction_steps": normalize_key_value("prediction_steps", metrics_row.get("prediction_steps", NA)),
                            "max_abs_ugv_yaw_rate": normalize_key_value("max_abs_ugv_yaw_rate", metrics_row.get("max_abs_ugv_yaw_rate", NA)),
                            "turning_omega_thr": normalize_key_value("turning_omega_thr", metrics_row.get("turning_omega_thr", NA)),
                            "collision": normalize_key_value("collision", metrics_row.get("collision", NA)),
                            "tracking_rms": fmt_float(safe_float(metrics_row.get("tracking_rms"))),
                            "tracking_rms_high": derive_tracking_rms_high(
                                metrics_row.get("tracking_rms_high", NA),
                                metrics_row.get("tracking_rms_high_c", NA),
                            ),
                            "tracking_rms_all": fmt_float(safe_float(metrics_row.get("tracking_rms"))),
                            "tracking_rms_track_only": fmt_float(safe_float(metrics_row.get("tracking_rms_track_only"))),
                            "tracking_rms_cbf_active": fmt_float(safe_float(metric_with_fallback(
                                metrics_row, "solver_tracking_rms_avoid", "tracking_rms_cbf_active"
                            ))),
                            "solver_tracking_rms_avoid": fmt_float(safe_float(metric_with_fallback(
                                metrics_row, "solver_tracking_rms_avoid", "tracking_rms_cbf_active"
                            ))),
                            "solver_tracking_rms_avoid_samples": normalize_key_value(
                                "solver_tracking_rms_avoid_samples",
                                clean_metric_or_warn(
                                    metrics_row, "solver_tracking_rms_avoid_samples"
                                ),
                            ),
                            "c66_metric": fmt_float(safe_float(metrics_row.get("c66_metric"))),
                            "high_c_samples": normalize_key_value("cbf_active_samples", metrics_row.get("high_c_samples", NA)),
                            "min_h_high_c": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_min_h_high_c", "min_h_high_c"))),
                            "slack_sum_high_c": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_slack_sum_high_c", "slack_sum_high_c"))),
                            "tracking_rms_high_c": fmt_float(safe_float(metrics_row.get("tracking_rms_high_c"))),
                            "fail_rate_high_c": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_fail_rate_high_c", "fail_rate_high_c"))),
                            "fail_rate": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_fail_rate", "fail_rate"))),
                            "min_h": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_min_h", "min_h"))),
                            "solver_min_h": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_min_h", "min_h"))),
                            "solver_active_mean_h": fmt_float(safe_float(metric_with_fallback(
                                metrics_row, "solver_active_mean_h", "solver_mean_h"
                            ))),
                            "solver_min_planar_clearance": fmt_float(safe_float(clean_metric_or_warn(
                                metrics_row, "solver_min_planar_clearance"
                            ))),
                            "slack_sum": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_slack_sum", "slack_sum"))),
                            "max_slack": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_max_slack", "max_slack"))),
                            "cbf_active_samples": normalize_key_value("cbf_active_samples", metric_with_fallback(metrics_row, "solver_cbf_active_samples", "cbf_active_samples")),
                            "cbf_active_ratio": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_cbf_active_ratio", "cbf_active_ratio"))),
                            "track_only_samples": normalize_key_value("track_only_samples", metric_with_fallback(metrics_row, "solver_track_only_samples", "track_only_samples")),
                            "track_only_ratio": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_track_only_ratio", "track_only_ratio"))),
                            "fail_rate_turn": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_fail_rate_turn", "fail_rate_turn"))),
                            "fail_rate_straight": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_fail_rate_straight", "fail_rate_straight"))),
                            "min_h_turn": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_min_h_turn", "min_h_turn"))),
                            "min_h_straight": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_min_h_straight", "min_h_straight"))),
                            "slack_sum_turn": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_slack_sum_turn", "slack_sum_turn"))),
                            "slack_sum_straight": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_slack_sum_straight", "slack_sum_straight"))),
                            "max_slack_turn": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_max_slack_turn", "max_slack_turn"))),
                            "max_slack_straight": fmt_float(safe_float(metric_with_fallback(metrics_row, "solver_max_slack_straight", "max_slack_straight"))),
                            "tracking_rms_all_turn": fmt_float(safe_float(metrics_row.get("tracking_rms_all_turn"))),
                            "tracking_rms_all_straight": fmt_float(safe_float(metrics_row.get("tracking_rms_all_straight"))),
                        }
                    row["valid_pair_key"] = build_valid_pair_key(row)
                    print(
                        f"[OK] {completed}/{total} mode={case.frame_mode} seed={case.seed} "
                        f"obs={case.obs} kappa={case.kappa:.1f} traj={row.get('traj_id', NA)}",
                        flush=True,
                    )
                except Exception as exc:
                    row = {
                        "frame_mode": case.frame_mode,
                        "frame_mode_effective": NA,
                        "run_tag": run_tag,
                        "metrics_path": str(metrics_path) if metrics_path else NA,
                        "parse_ok": "0",
                        "safety_variant": case.safety_variant,
                        "obstacle_count": str(case.obs),
                        "seed": str(case.seed),
                        "seed_env": str(case.seed),
                        "seed_traj": str(case.seed),
                        "traj_id": NA,
                        "valid_pair_key": NA,
                        "kappa": fmt_float(case.kappa),
                        "requested_r": fmt_float(args.r),
                        "requested_v": fmt_float(args.v),
                        "requested_w": fmt_float(args.w),
                        "start_z": fmt_float(args.start_z),
                        "noise_seed": str(case.seed),
                        "use_dynamic_obs": "0",
                        "warm_start": "1",
                        "active_uavs_csv": ";".join(str(x) for x in active_uavs),
                        "sim_control_dt_sec": NA,
                        "prediction_dt_sec": NA,
                        "prediction_steps": NA,
                        "max_abs_ugv_yaw_rate": NA,
                        "turning_omega_thr": NA,
                        "collision": NA,
                        "tracking_rms": NA,
                        "tracking_rms_high": NA,
                        "tracking_rms_all": NA,
                        "tracking_rms_track_only": NA,
                        "tracking_rms_cbf_active": NA,
                        "solver_tracking_rms_avoid": NA,
                        "solver_tracking_rms_avoid_samples": NA,
                        "c66_metric": NA,
                        "high_c_samples": NA,
                        "min_h_high_c": NA,
                        "slack_sum_high_c": NA,
                        "tracking_rms_high_c": NA,
                        "fail_rate_high_c": NA,
                        "fail_rate": NA,
                        "min_h": NA,
                        "solver_min_h": NA,
                        "solver_active_mean_h": NA,
                        "solver_min_planar_clearance": NA,
                        "slack_sum": NA,
                        "max_slack": NA,
                        "cbf_active_samples": NA,
                        "cbf_active_ratio": NA,
                        "track_only_samples": NA,
                        "track_only_ratio": NA,
                        "fail_rate_turn": NA,
                        "fail_rate_straight": NA,
                        "min_h_turn": NA,
                        "min_h_straight": NA,
                        "slack_sum_turn": NA,
                        "slack_sum_straight": NA,
                        "max_slack_turn": NA,
                        "max_slack_straight": NA,
                        "tracking_rms_all_turn": NA,
                        "tracking_rms_all_straight": NA,
                    }
                    row["valid_pair_key"] = build_valid_pair_key(row)
                    print(f"[FAIL] {completed}/{total} mode={case.frame_mode} seed={case.seed} err={exc}", flush=True)
                per_run_rows.append(row)
                submit_next()

    per_run_rows = sorted(per_run_rows, key=lambda r: (
        normalize_key_value("seed_env", r.get("seed_env", NA)),
        normalize_key_value("obstacle_count", r.get("obstacle_count", NA)),
        normalize_key_value("kappa", r.get("kappa", NA)),
        str(r.get("frame_mode", "")),
    ))
    write_csv(out_root / "per_run_uav0.csv", PER_RUN_HEADER, per_run_rows)

    grouped: dict[str, dict[str, dict[str, str]]] = {}
    for row in per_run_rows:
        key = str(row.get("valid_pair_key", NA))
        grouped.setdefault(key, {})
        grouped[key][str(row.get("frame_mode", ""))] = {k: str(v) for k, v in row.items()}

    compare_rows: list[dict[str, object]] = []
    valid_compare_rows: list[dict[str, object]] = []
    for key in sorted(grouped):
        modes = grouped[key]
        non = modes.get("noninertial")
        inertial = modes.get("inertial")
        traj_match = int(non is not None and inertial is not None and non.get("traj_id", NA) != NA and non.get("traj_id") == inertial.get("traj_id"))
        settings_match = int(
            non is not None
            and inertial is not None
            and all(compare_field_equal(non, inertial, field) for field in PAIR_CHECK_FIELDS)
        )
        valid_pair = int(non is not None and inertial is not None and traj_match == 1 and settings_match == 1)
        row: dict[str, object] = {
            "valid_pair_key": key,
            "seed_env": non.get("seed_env") if non else (inertial.get("seed_env") if inertial else NA),
            "seed_traj": non.get("seed_traj") if non else (inertial.get("seed_traj") if inertial else NA),
            "obstacle_count": non.get("obstacle_count") if non else (inertial.get("obstacle_count") if inertial else NA),
            "kappa": non.get("kappa") if non else (inertial.get("kappa") if inertial else NA),
            "requested_r": non.get("requested_r") if non else (inertial.get("requested_r") if inertial else NA),
            "requested_v": non.get("requested_v") if non else (inertial.get("requested_v") if inertial else NA),
            "requested_w": non.get("requested_w") if non else (inertial.get("requested_w") if inertial else NA),
            "start_z": non.get("start_z") if non else (inertial.get("start_z") if inertial else NA),
            "noise_seed": non.get("noise_seed") if non else (inertial.get("noise_seed") if inertial else NA),
            "use_dynamic_obs": non.get("use_dynamic_obs") if non else (inertial.get("use_dynamic_obs") if inertial else NA),
            "warm_start": non.get("warm_start") if non else (inertial.get("warm_start") if inertial else NA),
            "active_uavs_csv": non.get("active_uavs_csv") if non else (inertial.get("active_uavs_csv") if inertial else NA),
            "sim_control_dt_sec": non.get("sim_control_dt_sec") if non else (inertial.get("sim_control_dt_sec") if inertial else NA),
            "prediction_dt_sec": non.get("prediction_dt_sec") if non else (inertial.get("prediction_dt_sec") if inertial else NA),
            "prediction_steps": non.get("prediction_steps") if non else (inertial.get("prediction_steps") if inertial else NA),
            "turning_omega_thr": non.get("turning_omega_thr") if non else (inertial.get("turning_omega_thr") if inertial else NA),
            "safety_variant": non.get("safety_variant") if non else (inertial.get("safety_variant") if inertial else NA),
            "traj_id_non": non.get("traj_id", NA) if non else NA,
            "traj_id_in": inertial.get("traj_id", NA) if inertial else NA,
            "traj_match": str(traj_match),
            "settings_match": str(settings_match),
            "valid_pair": str(valid_pair),
            "pair_status": pair_status(non, inertial, traj_match, settings_match),
            "noninertial_run_tag": non.get("run_tag", NA) if non else NA,
            "noninertial_collision": non.get("collision", NA) if non else NA,
            "noninertial_tracking_rms": non.get("tracking_rms", NA) if non else NA,
            "noninertial_tracking_rms_high": non.get("tracking_rms_high", NA) if non else NA,
            "noninertial_tracking_rms_all": non.get("tracking_rms_all", NA) if non else NA,
            "noninertial_tracking_rms_track_only": non.get("tracking_rms_track_only", NA) if non else NA,
            "noninertial_tracking_rms_cbf_active": non.get("tracking_rms_cbf_active", NA) if non else NA,
            "noninertial_solver_tracking_rms_avoid": non.get("solver_tracking_rms_avoid", NA) if non else NA,
            "noninertial_solver_tracking_rms_avoid_samples": non.get(
                "solver_tracking_rms_avoid_samples", NA
            ) if non else NA,
            "noninertial_c66_metric": non.get("c66_metric", NA) if non else NA,
            "noninertial_high_c_samples": non.get("high_c_samples", NA) if non else NA,
            "noninertial_min_h_high_c": non.get("min_h_high_c", NA) if non else NA,
            "noninertial_slack_sum_high_c": non.get("slack_sum_high_c", NA) if non else NA,
            "noninertial_tracking_rms_high_c": non.get("tracking_rms_high_c", NA) if non else NA,
            "noninertial_fail_rate_high_c": non.get("fail_rate_high_c", NA) if non else NA,
            "noninertial_fail_rate": non.get("fail_rate", NA) if non else NA,
            "noninertial_min_h": non.get("min_h", NA) if non else NA,
            "noninertial_solver_min_h": non.get("solver_min_h", NA) if non else NA,
            "noninertial_solver_active_mean_h": non.get("solver_active_mean_h", NA) if non else NA,
            "noninertial_solver_min_planar_clearance": non.get(
                "solver_min_planar_clearance", NA
            ) if non else NA,
            "noninertial_slack_sum": non.get("slack_sum", NA) if non else NA,
            "noninertial_max_slack": non.get("max_slack", NA) if non else NA,
            "noninertial_fail_rate_turn": non.get("fail_rate_turn", NA) if non else NA,
            "noninertial_fail_rate_straight": non.get("fail_rate_straight", NA) if non else NA,
            "noninertial_min_h_turn": non.get("min_h_turn", NA) if non else NA,
            "noninertial_min_h_straight": non.get("min_h_straight", NA) if non else NA,
            "noninertial_slack_sum_turn": non.get("slack_sum_turn", NA) if non else NA,
            "noninertial_slack_sum_straight": non.get("slack_sum_straight", NA) if non else NA,
            "noninertial_max_slack_turn": non.get("max_slack_turn", NA) if non else NA,
            "noninertial_max_slack_straight": non.get("max_slack_straight", NA) if non else NA,
            "noninertial_tracking_rms_all_turn": non.get("tracking_rms_all_turn", NA) if non else NA,
            "noninertial_tracking_rms_all_straight": non.get("tracking_rms_all_straight", NA) if non else NA,
            "inertial_run_tag": inertial.get("run_tag", NA) if inertial else NA,
            "inertial_collision": inertial.get("collision", NA) if inertial else NA,
            "inertial_tracking_rms": inertial.get("tracking_rms", NA) if inertial else NA,
            "inertial_tracking_rms_high": inertial.get("tracking_rms_high", NA) if inertial else NA,
            "inertial_tracking_rms_all": inertial.get("tracking_rms_all", NA) if inertial else NA,
            "inertial_tracking_rms_track_only": inertial.get("tracking_rms_track_only", NA) if inertial else NA,
            "inertial_tracking_rms_cbf_active": inertial.get("tracking_rms_cbf_active", NA) if inertial else NA,
            "inertial_solver_tracking_rms_avoid": inertial.get("solver_tracking_rms_avoid", NA) if inertial else NA,
            "inertial_solver_tracking_rms_avoid_samples": inertial.get(
                "solver_tracking_rms_avoid_samples", NA
            ) if inertial else NA,
            "inertial_c66_metric": inertial.get("c66_metric", NA) if inertial else NA,
            "inertial_high_c_samples": inertial.get("high_c_samples", NA) if inertial else NA,
            "inertial_min_h_high_c": inertial.get("min_h_high_c", NA) if inertial else NA,
            "inertial_slack_sum_high_c": inertial.get("slack_sum_high_c", NA) if inertial else NA,
            "inertial_tracking_rms_high_c": inertial.get("tracking_rms_high_c", NA) if inertial else NA,
            "inertial_fail_rate_high_c": inertial.get("fail_rate_high_c", NA) if inertial else NA,
            "inertial_fail_rate": inertial.get("fail_rate", NA) if inertial else NA,
            "inertial_min_h": inertial.get("min_h", NA) if inertial else NA,
            "inertial_solver_min_h": inertial.get("solver_min_h", NA) if inertial else NA,
            "inertial_solver_active_mean_h": inertial.get("solver_active_mean_h", NA) if inertial else NA,
            "inertial_solver_min_planar_clearance": inertial.get(
                "solver_min_planar_clearance", NA
            ) if inertial else NA,
            "inertial_slack_sum": inertial.get("slack_sum", NA) if inertial else NA,
            "inertial_max_slack": inertial.get("max_slack", NA) if inertial else NA,
            "inertial_fail_rate_turn": inertial.get("fail_rate_turn", NA) if inertial else NA,
            "inertial_fail_rate_straight": inertial.get("fail_rate_straight", NA) if inertial else NA,
            "inertial_min_h_turn": inertial.get("min_h_turn", NA) if inertial else NA,
            "inertial_min_h_straight": inertial.get("min_h_straight", NA) if inertial else NA,
            "inertial_slack_sum_turn": inertial.get("slack_sum_turn", NA) if inertial else NA,
            "inertial_slack_sum_straight": inertial.get("slack_sum_straight", NA) if inertial else NA,
            "inertial_max_slack_turn": inertial.get("max_slack_turn", NA) if inertial else NA,
            "inertial_max_slack_straight": inertial.get("max_slack_straight", NA) if inertial else NA,
            "inertial_tracking_rms_all_turn": inertial.get("tracking_rms_all_turn", NA) if inertial else NA,
            "inertial_tracking_rms_all_straight": inertial.get("tracking_rms_all_straight", NA) if inertial else NA,
        }
        row["delta_fail_rate"] = maybe_delta(valid_pair, row["inertial_fail_rate"], row["noninertial_fail_rate"])
        row["delta_min_h"] = maybe_delta(valid_pair, row["inertial_min_h"], row["noninertial_min_h"], flip_non_minus_in=True)
        row["delta_slack_sum"] = maybe_delta(valid_pair, row["inertial_slack_sum"], row["noninertial_slack_sum"])
        row["delta_max_slack"] = maybe_delta(valid_pair, row["inertial_max_slack"], row["noninertial_max_slack"])
        row["delta_tracking_rms_all"] = maybe_delta(valid_pair, row["inertial_tracking_rms_all"], row["noninertial_tracking_rms_all"])
        row["delta_fail_rate_high_c"] = maybe_delta(valid_pair, row["inertial_fail_rate_high_c"], row["noninertial_fail_rate_high_c"])
        row["delta_min_h_high_c"] = maybe_delta(valid_pair, row["inertial_min_h_high_c"], row["noninertial_min_h_high_c"], flip_non_minus_in=True)
        row["delta_slack_sum_high_c"] = maybe_delta(valid_pair, row["inertial_slack_sum_high_c"], row["noninertial_slack_sum_high_c"])
        row["delta_tracking_rms_high"] = maybe_delta(valid_pair, row["inertial_tracking_rms_high"], row["noninertial_tracking_rms_high"])
        row["delta_solver_tracking_rms_avoid_non_minus_inertial"] = maybe_delta(
            valid_pair,
            row["inertial_solver_tracking_rms_avoid"],
            row["noninertial_solver_tracking_rms_avoid"],
            flip_non_minus_in=True,
        )
        row["delta_solver_min_h_non_minus_inertial"] = maybe_delta(
            valid_pair,
            row["inertial_solver_min_h"],
            row["noninertial_solver_min_h"],
            flip_non_minus_in=True,
        )
        row["delta_solver_min_planar_clearance_non_minus_inertial"] = maybe_delta(
            valid_pair,
            row["inertial_solver_min_planar_clearance"],
            row["noninertial_solver_min_planar_clearance"],
            flip_non_minus_in=True,
        )
        compare_rows.append(row)
        if valid_pair == 1:
            valid_compare_rows.append(row)

    write_csv(out_root / "compare_per_seed_uav0.csv", COMPARE_HEADER, compare_rows)

    summary_rows: list[dict[str, object]] = []
    n_pairs_total = len(compare_rows)
    n_pairs_valid = sum(1 for row in compare_rows if safe_int(row.get("valid_pair")) == 1)
    solver_tracking_pair_values = [
        row for row in valid_compare_rows
        if math.isfinite(safe_float(row.get("noninertial_solver_tracking_rms_avoid")))
        and math.isfinite(safe_float(row.get("inertial_solver_tracking_rms_avoid")))
    ]
    pct_noninertial_better_solver_tracking_rms_avoid = (
        math.nan
        if not solver_tracking_pair_values
        else 100.0
        * (
            sum(
                1
                for row in solver_tracking_pair_values
                if safe_float(row.get("noninertial_solver_tracking_rms_avoid"))
                < safe_float(row.get("inertial_solver_tracking_rms_avoid"))
            )
            / float(len(solver_tracking_pair_values))
        )
    )
    for frame_mode in ("noninertial", "inertial"):
        prefix = f"{frame_mode}_"
        valid_runs = [row for row in valid_compare_rows]
        all_runs = [row for row in per_run_rows if str(row.get("frame_mode")) == frame_mode]

        def collect(name: str) -> list[float]:
            return [safe_float(row.get(prefix + name)) for row in valid_runs]

        collisions = [safe_int(row.get(prefix + "collision")) for row in valid_runs]
        summary_rows.append(
            {
                "frame_mode": frame_mode,
                "aggregation_scope": "valid_pair_runs",
                "n_runs_total": fmt_int(len(all_runs)),
                "n_runs_aggregated": fmt_int(len(valid_runs)),
                "n_pairs_total": fmt_int(n_pairs_total),
                "n_pairs_valid": fmt_int(n_pairs_valid),
                "n_pairs_invalid_or_unpaired": fmt_int(n_pairs_total - n_pairs_valid),
                "collisions": fmt_int(sum(collisions)),
                "collision_rate": fmt_float(mean([float(v) for v in collisions])),
                "mean_tracking_rms_all": fmt_float(mean(collect("tracking_rms_all"))),
                "median_tracking_rms_all": fmt_float(median(collect("tracking_rms_all"))),
                "p95_tracking_rms_all": fmt_float(quantile(collect("tracking_rms_all"), 0.95)),
                "mean_tracking_rms_track_only": fmt_float(mean(collect("tracking_rms_track_only"))),
                "median_tracking_rms_track_only": fmt_float(median(collect("tracking_rms_track_only"))),
                "p95_tracking_rms_track_only": fmt_float(quantile(collect("tracking_rms_track_only"), 0.95)),
                "mean_tracking_rms_cbf_active": fmt_float(mean(collect("tracking_rms_cbf_active"))),
                "median_tracking_rms_cbf_active": fmt_float(median(collect("tracking_rms_cbf_active"))),
                "p95_tracking_rms_cbf_active": fmt_float(quantile(collect("tracking_rms_cbf_active"), 0.95)),
                "mean_solver_tracking_rms_avoid": fmt_float(mean(collect("solver_tracking_rms_avoid"))),
                "median_solver_tracking_rms_avoid": fmt_float(median(collect("solver_tracking_rms_avoid"))),
                "p95_solver_tracking_rms_avoid": fmt_float(quantile(collect("solver_tracking_rms_avoid"), 0.95)),
                "mean_tracking_rms_high": fmt_float(mean(collect("tracking_rms_high"))),
                "median_tracking_rms_high": fmt_float(median(collect("tracking_rms_high"))),
                "p95_tracking_rms_high": fmt_float(quantile(collect("tracking_rms_high"), 0.95)),
                "mean_fail_rate": fmt_float(mean(collect("fail_rate"))),
                "median_fail_rate": fmt_float(median(collect("fail_rate"))),
                "p95_fail_rate": fmt_float(quantile(collect("fail_rate"), 0.95)),
                "mean_fail_rate_high_c": fmt_float(mean(collect("fail_rate_high_c"))),
                "median_fail_rate_high_c": fmt_float(median(collect("fail_rate_high_c"))),
                "p95_fail_rate_high_c": fmt_float(quantile(collect("fail_rate_high_c"), 0.95)),
                "mean_min_h": fmt_float(mean(collect("min_h"))),
                "median_min_h": fmt_float(median(collect("min_h"))),
                "p05_min_h": fmt_float(quantile(collect("min_h"), 0.05)),
                "mean_solver_min_h": fmt_float(mean(collect("solver_min_h"))),
                "median_solver_min_h": fmt_float(median(collect("solver_min_h"))),
                "p05_solver_min_h": fmt_float(quantile(collect("solver_min_h"), 0.05)),
                "mean_solver_active_mean_h": fmt_float(mean(collect("solver_active_mean_h"))),
                "median_solver_active_mean_h": fmt_float(median(collect("solver_active_mean_h"))),
                "p05_solver_active_mean_h": fmt_float(quantile(collect("solver_active_mean_h"), 0.05)),
                "mean_solver_min_planar_clearance": fmt_float(mean(collect("solver_min_planar_clearance"))),
                "median_solver_min_planar_clearance": fmt_float(median(collect("solver_min_planar_clearance"))),
                "p05_solver_min_planar_clearance": fmt_float(quantile(collect("solver_min_planar_clearance"), 0.05)),
                "mean_min_h_high_c": fmt_float(mean(collect("min_h_high_c"))),
                "median_min_h_high_c": fmt_float(median(collect("min_h_high_c"))),
                "p05_min_h_high_c": fmt_float(quantile(collect("min_h_high_c"), 0.05)),
                "mean_slack_sum": fmt_float(mean(collect("slack_sum"))),
                "median_slack_sum": fmt_float(median(collect("slack_sum"))),
                "p95_slack_sum": fmt_float(quantile(collect("slack_sum"), 0.95)),
                "mean_slack_sum_high_c": fmt_float(mean(collect("slack_sum_high_c"))),
                "median_slack_sum_high_c": fmt_float(median(collect("slack_sum_high_c"))),
                "p95_slack_sum_high_c": fmt_float(quantile(collect("slack_sum_high_c"), 0.95)),
                "mean_max_slack": fmt_float(mean(collect("max_slack"))),
                "median_max_slack": fmt_float(median(collect("max_slack"))),
                "p95_max_slack": fmt_float(quantile(collect("max_slack"), 0.95)),
                "mean_fail_rate_turn": fmt_float(mean(collect("fail_rate_turn"))),
                "mean_fail_rate_straight": fmt_float(mean(collect("fail_rate_straight"))),
                "mean_min_h_turn": fmt_float(mean(collect("min_h_turn"))),
                "mean_min_h_straight": fmt_float(mean(collect("min_h_straight"))),
                "mean_slack_sum_turn": fmt_float(mean(collect("slack_sum_turn"))),
                "mean_slack_sum_straight": fmt_float(mean(collect("slack_sum_straight"))),
                "mean_max_slack_turn": fmt_float(mean(collect("max_slack_turn"))),
                "mean_max_slack_straight": fmt_float(mean(collect("max_slack_straight"))),
                "mean_tracking_rms_all_turn": fmt_float(mean(collect("tracking_rms_all_turn"))),
                "mean_tracking_rms_all_straight": fmt_float(mean(collect("tracking_rms_all_straight"))),
                "pct_noninertial_better_solver_tracking_rms_avoid": fmt_float(
                    pct_noninertial_better_solver_tracking_rms_avoid
                ),
            }
        )

    write_csv(out_root / "summary_uav0.csv", SUMMARY_HEADER, summary_rows)
    shutil.copyfile(out_root / "summary_uav0.csv", out_root / "metrics_summary.csv")
    write_readme(out_root)
    print(f"[DONE] out_root={out_root}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
