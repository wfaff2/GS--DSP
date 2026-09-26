#!/usr/bin/env python3
"""
Run baseline + ablation grid and export per-run rows to results/raw_runs.csv.

Experiment set (A3 intentionally removed):
- Baseline: (inertial, A2_soft_cbf) vs (noninertial, A2_soft_cbf)
- Ablation: (noninertial, A0_no_cbf/A1_hard_cbf/A2_soft_cbf)
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import fcntl
import hashlib
import json
import math
import os
import pathlib
import re
import signal
import socket
import subprocess
import sys
import time
from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import yaml  # type: ignore
except Exception:  # pragma: no cover
    yaml = None


RAW_HEADER = [
    "frame_mode",
    "frame_mode_effective",
    "safety_variant",
    "obs",
    "kappa",
    "seed",
    "tracking_rms",
    "tracking_p95",
    "solver_tracking_rms_avoid",
    "solver_tracking_rms_avoid_samples",
    "collision_flag",
    "margin_flag",
    "min_h",
    "solver_min_h",
    "solver_active_mean_h",
    "solver_min_planar_clearance",
    "min_cbf",
    "dmin_lin",
    "slack_sum",
    "slack_max",
    "solve_time_mean_ms",
    "solve_time_p95_ms",
    "solve_time_max_ms",
    "fail_flag",
]

HASH_HEADER = ["obs", "kappa", "seed", "config_hash", "source_run_tag"]

_METRIC_FALLBACK_WARNINGS: set[str] = set()
_MISSING_CLEAN_METRIC_WARNINGS: set[str] = set()


class ConfigMismatchError(RuntimeError):
    pass


@dataclasses.dataclass(frozen=True)
class Case:
    run_index: int
    frame_mode: str
    safety_variant: str
    obs: int
    kappa: float
    seed: int


@dataclasses.dataclass(frozen=True)
class SafetyConfig:
    cbf_enabled: bool
    cbf_use_in_sim: bool
    slack_max: float
    rho_s: float


def _log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def _bool_str(v: bool) -> str:
    return "true" if v else "false"


def _safe_float(text: object) -> float:
    try:
        return float(text)  # type: ignore[arg-type]
    except Exception:
        return math.nan


def _safe_int(text: object, default: int = 0) -> int:
    try:
        return int(float(text))  # type: ignore[arg-type]
    except Exception:
        return default


def _float_cell(v: float) -> str:
    return "nan" if not math.isfinite(v) else f"{v:.6f}"


def _warn_once(cache: set[str], key: str, msg: str) -> None:
    if key in cache:
        return
    print(msg, file=sys.stderr, flush=True)
    cache.add(key)


def _metric_field(row: Dict[str, str], clean_key: str, legacy_key: str) -> object:
    value = row.get(clean_key)
    if value is not None and str(value).strip() != "":
        return value
    legacy_value = row.get(legacy_key)
    if legacy_value is not None and str(legacy_value).strip() != "":
        _warn_once(
            _METRIC_FALLBACK_WARNINGS,
            f"{clean_key}->{legacy_key}",
            f"[WARN] metrics.csv is missing {clean_key}; falling back to legacy {legacy_key}",
        )
        return legacy_value
    _warn_once(
        _MISSING_CLEAN_METRIC_WARNINGS,
        clean_key,
        f"[WARN] metrics.csv is missing clean field {clean_key}",
    )
    return None


def _clean_metric_field(row: Dict[str, str], clean_key: str) -> object:
    value = row.get(clean_key)
    if value is not None and str(value).strip() != "":
        return value
    _warn_once(
        _MISSING_CLEAN_METRIC_WARNINGS,
        clean_key,
        f"[WARN] metrics.csv is missing clean field {clean_key}",
    )
    return None


def _norm_kappa(v: float) -> str:
    return f"{float(v):.6f}"


def _normalize_frame_mode(text: str) -> str:
    mode = text.strip().lower().replace("-", "").replace("_", "")
    if mode == "inertial":
        return "inertial"
    if mode == "noninertial":
        return "noninertial"
    return ""


def _parse_int_list_csv(text: str) -> List[int]:
    out: List[int] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        out.append(int(part))
    if not out:
        raise ValueError("empty integer list")
    return out


def _parse_float_list_csv(text: str) -> List[float]:
    out: List[float] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        out.append(float(part))
    if not out:
        raise ValueError("empty float list")
    return out


def _atomic_append_csv(path: pathlib.Path, header: Sequence[str], row: Dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+", encoding="utf-8", newline="") as f:
        fcntl.flock(f.fileno(), fcntl.LOCK_EX)
        try:
            f.seek(0, os.SEEK_END)
            need_header = f.tell() == 0
            writer = csv.DictWriter(f, fieldnames=list(header))
            if need_header:
                writer.writeheader()
            writer.writerow(row)
            f.flush()
            os.fsync(f.fileno())
        finally:
            fcntl.flock(f.fileno(), fcntl.LOCK_UN)


def _remove_path(path: pathlib.Path) -> None:
    if path.is_file() or path.is_symlink():
        path.unlink()
        return
    if not path.exists():
        return
    for sub in sorted(path.rglob("*"), reverse=True):
        if sub.is_file() or sub.is_symlink():
            sub.unlink()
        elif sub.is_dir():
            sub.rmdir()
    path.rmdir()


def _read_existing_rows(raw_csv: pathlib.Path) -> Dict[Tuple[str, str, int, str, int], Dict[str, str]]:
    existing: Dict[Tuple[str, str, int, str, int], Dict[str, str]] = {}
    if not raw_csv.exists():
        return existing
    with raw_csv.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                key = (
                    str(row["frame_mode"]),
                    str(row["safety_variant"]),
                    int(row["obs"]),
                    _norm_kappa(float(row["kappa"])),
                    int(row["seed"]),
                )
            except Exception:
                continue
            existing[key] = row
    return existing


def _read_hash_map(path: pathlib.Path) -> Dict[Tuple[int, str, int], str]:
    out: Dict[Tuple[int, str, int], str] = {}
    if not path.exists():
        return out
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                key = (int(row["obs"]), _norm_kappa(float(row["kappa"])), int(row["seed"]))
                out[key] = str(row["config_hash"])
            except Exception:
                continue
    return out


def _serialize_yaml_hash_payload(v: object) -> object:
    if isinstance(v, dict):
        return {str(k): _serialize_yaml_hash_payload(v[k]) for k in sorted(v)}
    if isinstance(v, list):
        return [_serialize_yaml_hash_payload(x) for x in v]
    if isinstance(v, (int, float, bool, str)) or v is None:
        return v
    return str(v)


def _load_soft_defaults(yaml_path: pathlib.Path) -> Tuple[float, float, Dict[str, object]]:
    slack_default = 10.0
    rho_default = 100.0
    hash_payload: Dict[str, object] = {}

    if yaml is None or not yaml_path.exists():
        return slack_default, rho_default, hash_payload

    with yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        return slack_default, rho_default, hash_payload

    cbf = data.get("cbf", {})
    if isinstance(cbf, dict):
        slack_default = float(cbf.get("slack_max", cbf.get("s_max", slack_default)))
        rho_default = float(cbf.get("R_slack", cbf.get("rho_s", rho_default)))

    hash_payload = {
        "uav_radius": data.get("uav_radius"),
        "car_radius": data.get("car_radius"),
        "dynamic_obs": data.get("dynamic_obs"),
        "random_obstacles": data.get("random_obstacles"),
    }
    hash_payload = _serialize_yaml_hash_payload(hash_payload)  # type: ignore[assignment]
    return slack_default, rho_default, hash_payload


def _make_safety_map(soft_slack_max: float, soft_rho: float) -> Dict[str, SafetyConfig]:
    return {
        "A0_no_cbf": SafetyConfig(
            cbf_enabled=False,
            cbf_use_in_sim=False,
            slack_max=0.0,
            rho_s=1.0,
        ),
        "A1_hard_cbf": SafetyConfig(
            cbf_enabled=True,
            cbf_use_in_sim=True,
            slack_max=0.0,
            rho_s=soft_rho,
        ),
        "A2_soft_cbf": SafetyConfig(
            cbf_enabled=True,
            cbf_use_in_sim=True,
            slack_max=soft_slack_max,
            rho_s=soft_rho,
        ),
    }


def _build_cases(
    seed_start: int,
    seed_end: int,
    obs_values: Sequence[int],
    kappa_values: Sequence[float],
) -> List[Case]:
    combos = [
        ("noninertial", "A0_no_cbf"),
        ("noninertial", "A1_hard_cbf"),
        ("noninertial", "A2_soft_cbf"),
        ("inertial", "A2_soft_cbf"),
    ]
    out: List[Case] = []
    idx = 0
    for obs in obs_values:
        for kappa in kappa_values:
            for seed in range(seed_start, seed_end + 1):
                for frame_mode, safety_variant in combos:
                    idx += 1
                    out.append(
                        Case(
                            run_index=idx,
                            frame_mode=frame_mode,
                            safety_variant=safety_variant,
                            obs=int(obs),
                            kappa=float(kappa),
                            seed=int(seed),
                        )
                    )
    return out


def _compute_config_hash(
    case: Case,
    *,
    r: float,
    v: float,
    w: float,
    start_z: float,
    active_uavs: Sequence[int],
    yaml_payload: Dict[str, object],
) -> str:
    payload = {
        "obs": case.obs,
        "kappa": float(_norm_kappa(case.kappa)),
        "seed": case.seed,
        "noise_seed": case.seed,
        "r": r,
        "v": v,
        "w": w,
        "start_z": start_z,
        "use_dynamic_obs": False,
        "warm_start": True,
        "active_uavs": list(active_uavs),
        "yaml": yaml_payload,
        "controlled_factors": ["obstacles", "trajectory", "noise", "init"],
    }
    data = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def _find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        return int(s.getsockname()[1])


def _kill_process_group(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except Exception:
        return
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            pass


def _rosparam_list_ok(env: Dict[str, str]) -> bool:
    rc = subprocess.run(
        ["rosparam", "list"],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode
    return rc == 0


def _start_roscore(
    env: Dict[str, str],
    port: int,
    log_path: pathlib.Path,
    wait_sec: float = 12.0,
) -> Tuple[subprocess.Popen, object]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("wb")
    proc = subprocess.Popen(
        ["roscore", "-p", str(port)],
        env=env,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    deadline = time.time() + wait_sec
    while time.time() < deadline:
        if _rosparam_list_ok(env):
            return proc, log_file
        if proc.poll() is not None:
            break
        time.sleep(0.2)

    _kill_process_group(proc)
    log_file.close()
    raise RuntimeError(f"roscore failed to start on port {port}")


def _run_cmd_logged(
    cmd: Sequence[str],
    env: Dict[str, str],
    log_path: pathlib.Path,
    timeout_sec: Optional[float],
) -> Tuple[int, bool]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("wb") as log_file:
        proc = subprocess.Popen(
            list(cmd),
            env=env,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        try:
            if timeout_sec is None or timeout_sec <= 0:
                rc = proc.wait()
                return rc, False
            rc = proc.wait(timeout=timeout_sec)
            return rc, False
        except subprocess.TimeoutExpired:
            _kill_process_group(proc)
            return 124, True


def _run_quick_cmd(
    cmd: Sequence[str],
    env: Dict[str, str],
    log_handle: object,
) -> int:
    return subprocess.run(list(cmd), env=env, stdout=log_handle, stderr=log_handle).returncode


def _rosparam_set(
    env: Dict[str, str],
    key: str,
    value: object,
    log_handle: object,
) -> None:
    if isinstance(value, bool):
        text = _bool_str(value)
    elif isinstance(value, (list, tuple)):
        text = "[" + ",".join(str(x) for x in value) + "]"
    else:
        text = str(value)
    rc = _run_quick_cmd(["rosparam", "set", key, text], env=env, log_handle=log_handle)
    if rc != 0:
        raise RuntimeError(f"rosparam set failed: {key}={text}")


def _parse_metrics_all_row(metrics_csv: pathlib.Path, run_tag: str) -> Optional[Dict[str, str]]:
    if not metrics_csv.exists():
        return None
    matched: Optional[Dict[str, str]] = None
    with metrics_csv.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("run_tag") == run_tag and row.get("scope") == "all":
                matched = row
    return matched


def _parse_frame_mode_from_log(log_path: pathlib.Path) -> str:
    if not log_path.exists():
        return ""
    text = log_path.read_text(encoding="utf-8", errors="replace")
    matches = re.findall(r"frame_mode_effective=([A-Za-z_\-]+)", text)
    if not matches:
        return ""
    return _normalize_frame_mode(matches[-1])


def _make_empty_row(case: Case, fail_flag: int) -> Dict[str, str]:
    return {
        "frame_mode": case.frame_mode,
        "frame_mode_effective": "",
        "safety_variant": case.safety_variant,
        "obs": str(case.obs),
        "kappa": _norm_kappa(case.kappa),
        "seed": str(case.seed),
        "tracking_rms": "nan",
        "tracking_p95": "nan",
        "solver_tracking_rms_avoid": "nan",
        "solver_tracking_rms_avoid_samples": "0",
        "collision_flag": "0",
        "margin_flag": "0",
        "min_h": "nan",
        "solver_min_h": "nan",
        "solver_active_mean_h": "nan",
        "solver_min_planar_clearance": "nan",
        "min_cbf": "nan",
        "dmin_lin": "nan",
        "slack_sum": "nan",
        "slack_max": "nan",
        "solve_time_mean_ms": "nan",
        "solve_time_p95_ms": "nan",
        "solve_time_max_ms": "nan",
        "fail_flag": str(int(fail_flag)),
    }


def _case_key(case: Case) -> Tuple[str, str, int, str, int]:
    return (
        case.frame_mode,
        case.safety_variant,
        case.obs,
        _norm_kappa(case.kappa),
        case.seed,
    )


def _base_key(case: Case) -> Tuple[int, str, int]:
    return (case.obs, _norm_kappa(case.kappa), case.seed)


def _variant_short(variant: str) -> str:
    return variant.replace("_", "")


def _run_case(
    case: Case,
    *,
    args: argparse.Namespace,
    safety_cfg: SafetyConfig,
    suite_timestamp: str,
    logs_root: pathlib.Path,
    yaml_hash_payload: Dict[str, object],
) -> Dict[str, object]:
    run_tag = (
        f"{suite_timestamp}_{case.run_index:06d}_"
        f"{case.frame_mode}_{_variant_short(case.safety_variant)}_"
        f"o{case.obs}_k{case.kappa:.1f}_s{case.seed}"
    )
    run_dir = logs_root / run_tag
    run_dir.mkdir(parents=True, exist_ok=True)

    metrics_csv = run_dir / "metrics.csv"
    setup_log_path = run_dir / "setup.log"
    roscore_log_path = run_dir / "roscore.log"
    launch_log_path = run_dir / "rosrun.log"
    meta_path = run_dir / "meta.json"

    cfg_hash = _compute_config_hash(
        case,
        r=args.r,
        v=args.v,
        w=args.w,
        start_z=args.start_z,
        active_uavs=args.active_uavs_list,
        yaml_payload=yaml_hash_payload,
    )

    env = os.environ.copy()
    port = _find_free_port()
    env["ROS_MASTER_URI"] = f"http://127.0.0.1:{port}"
    env["ROS_IP"] = "127.0.0.1"
    env["ROS_HOSTNAME"] = "127.0.0.1"
    env["ROS_HOME"] = str(run_dir / "ros_home")
    env["ROS_LOG_DIR"] = str(run_dir / "ros_logs")
    pathlib.Path(env["ROS_HOME"]).mkdir(parents=True, exist_ok=True)
    pathlib.Path(env["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)

    rc = 1
    timed_out = False
    row_out = _make_empty_row(case, fail_flag=1)
    error_msg: Optional[str] = None
    roscore_proc: Optional[subprocess.Popen] = None
    roscore_log_file = None
    wall_start = time.time()

    try:
        roscore_proc, roscore_log_file = _start_roscore(env, port, roscore_log_path)

        with setup_log_path.open("w", encoding="utf-8") as setup_log:
            rc_load = _run_quick_cmd(
                [
                    "rosparam",
                    "load",
                    str(args.yaml_abs),
                    "/num_sim_non_one_point_node",
                ],
                env=env,
                log_handle=setup_log,
            )
            if rc_load != 0:
                raise RuntimeError(f"rosparam load failed: {args.yaml_abs}")

            frame_is_inertial = case.frame_mode == "inertial"
            params = [
                ("/num_sim_non_one_point_node/frame_mode", case.frame_mode),
                ("/num_sim_non_one_point_node/use_inertial_frame", frame_is_inertial),
                ("/num_sim_non_one_point_node/use_noninertial_frame", not frame_is_inertial),
                ("/num_sim_non_one_point_node/safety_variant", case.safety_variant),
                ("/num_sim_non_one_point_node/metrics/debug", False),
                ("/num_sim_non_one_point_node/active_uavs", args.active_uavs_list),
                ("/num_sim_non_one_point_node/random_obstacles/count", case.obs),
                ("/num_sim_non_one_point_node/random_obstacles/seed", case.seed),
                ("/num_sim_non_one_point_node/noise_seed", case.seed),
                ("/num_sim_non_one_point_node/kappa", case.kappa),
                ("/num_sim_non_one_point_node/metrics_csv", str(metrics_csv)),
                ("/num_sim_non_one_point_node/run_tag", run_tag),
                ("/num_sim_non_one_point_node/use_dynamic_obs", False),
                ("/num_sim_non_one_point_node/warm_start", True),
                ("/num_sim_non_one_point_node/dynamic_obs/start_z", args.start_z),
                ("/num_sim_non_one_point_node/cbf/enabled", safety_cfg.cbf_enabled),
                ("/num_sim_non_one_point_node/cbf/use_in_sim", safety_cfg.cbf_use_in_sim),
                ("/num_sim_non_one_point_node/cbf/slack_max", safety_cfg.slack_max),
                ("/num_sim_non_one_point_node/cbf/R_slack", safety_cfg.rho_s),
            ]
            for key, value in params:
                _rosparam_set(env, key, value, setup_log)

        cmd = [
            "rosrun",
            "coni_mpc",
            "num_sim_non_one_point_node",
            "-r",
            str(args.r),
            "-v",
            str(args.v),
            "-w",
            str(args.w),
        ]
        rc, timed_out = _run_cmd_logged(cmd, env, launch_log_path, args.timeout_sec)

        all_row = _parse_metrics_all_row(metrics_csv, run_tag)
        if all_row is None:
            row_out = _make_empty_row(case, fail_flag=1)
            row_out["fail_flag"] = "1"
        else:
            actual_obs = _safe_int(all_row.get("obstacle_count"), default=-1)
            actual_seed = _safe_int(all_row.get("seed"), default=-1)
            actual_kappa = _safe_float(all_row.get("kappa"))
            if actual_obs != case.obs or actual_seed != case.seed:
                raise ConfigMismatchError(
                    "metrics mismatch for obs/seed: "
                    f"expected ({case.obs},{case.seed}), got ({actual_obs},{actual_seed})"
                )
            if not math.isfinite(actual_kappa) or abs(actual_kappa - case.kappa) > 1e-6:
                raise ConfigMismatchError(
                    "metrics mismatch for kappa: "
                    f"expected {case.kappa}, got {all_row.get('kappa')}"
                )

            frame_mode_effective_csv = _normalize_frame_mode(
                str(all_row.get("frame_mode_effective", ""))
            )
            frame_mode_effective_log = _parse_frame_mode_from_log(launch_log_path)
            frame_mode_effective = frame_mode_effective_log or frame_mode_effective_csv
            if not frame_mode_effective:
                raise RuntimeError("cannot parse frame_mode_effective from node logs/metrics")
            if frame_mode_effective_csv and frame_mode_effective_log:
                if frame_mode_effective_csv != frame_mode_effective_log:
                    raise RuntimeError(
                        "frame_mode_effective mismatch between metrics.csv and rosrun.log: "
                        f"{frame_mode_effective_csv} vs {frame_mode_effective_log}"
                    )
            if frame_mode_effective != case.frame_mode:
                raise RuntimeError(
                    "frame_mode_effective mismatch with requested frame_mode: "
                    f"requested={case.frame_mode}, effective={frame_mode_effective}"
                )

            tracking_rms = _safe_float(all_row.get("tracking_rms"))
            tracking_p95 = _safe_float(all_row.get("tracking_p95"))
            solver_tracking_rms_avoid = _safe_float(
                _metric_field(all_row, "solver_tracking_rms_avoid", "tracking_rms_cbf_active")
            )
            solver_tracking_rms_avoid_samples = _safe_int(
                _clean_metric_field(all_row, "solver_tracking_rms_avoid_samples")
            )
            min_h = _safe_float(_metric_field(all_row, "solver_min_h", "min_h"))
            solver_min_h = _safe_float(_metric_field(all_row, "solver_min_h", "min_h"))
            solver_active_mean_h = _safe_float(
                _metric_field(all_row, "solver_active_mean_h", "solver_mean_h")
            )
            solver_min_planar_clearance = _safe_float(
                _clean_metric_field(all_row, "solver_min_planar_clearance")
            )
            min_cbf = _safe_float(_metric_field(all_row, "solver_min_cbf", "min_cbf"))
            dmin_lin = _safe_float(all_row.get("dmin_lin"))
            slack_sum = _safe_float(_metric_field(all_row, "solver_slack_sum", "slack_sum"))
            slack_max = _safe_float(_metric_field(all_row, "solver_max_slack", "max_slack"))
            solve_mean = _safe_float(all_row.get("solve_time_mean_ms"))
            solve_p95 = _safe_float(all_row.get("solve_time_p95_ms"))
            solve_max = _safe_float(all_row.get("solve_time_max_ms"))
            collision_flag = 1 if _safe_int(all_row.get("collision")) != 0 else 0
            margin_flag = 1 if math.isfinite(min_h) and min_h < 0.0 else 0
            fail_rate = _safe_float(_metric_field(all_row, "solver_fail_rate", "fail_rate"))

            fail_flag = 0
            if rc != 0 or timed_out:
                fail_flag = 1
            elif not math.isfinite(fail_rate):
                fail_flag = 1
            elif fail_rate > args.fail_rate_threshold:
                fail_flag = 1

            required_finite = [
                tracking_rms,
                tracking_p95,
                min_h,
                min_cbf,
                dmin_lin,
                slack_sum,
                slack_max,
                solve_mean,
                solve_p95,
                solve_max,
            ]
            if any((not math.isfinite(v)) for v in required_finite):
                fail_flag = 1

            if case.safety_variant in ("A0_no_cbf", "A1_hard_cbf"):
                if abs(slack_sum) > args.hard_slack_tol or abs(slack_max) > args.hard_slack_tol:
                    raise RuntimeError(
                        "hard/no-CBF variant reported nonzero slack: "
                        f"variant={case.safety_variant}, slack_sum={slack_sum}, slack_max={slack_max}"
                    )

            row_out = {
                "frame_mode": case.frame_mode,
                "frame_mode_effective": frame_mode_effective,
                "safety_variant": case.safety_variant,
                "obs": str(case.obs),
                "kappa": _norm_kappa(case.kappa),
                "seed": str(case.seed),
                "tracking_rms": _float_cell(tracking_rms),
                "tracking_p95": _float_cell(tracking_p95),
                "solver_tracking_rms_avoid": _float_cell(solver_tracking_rms_avoid),
                "solver_tracking_rms_avoid_samples": str(
                    int(solver_tracking_rms_avoid_samples)
                ),
                "collision_flag": str(collision_flag),
                "margin_flag": str(margin_flag),
                "min_h": _float_cell(min_h),
                "solver_min_h": _float_cell(solver_min_h),
                "solver_active_mean_h": _float_cell(solver_active_mean_h),
                "solver_min_planar_clearance": _float_cell(
                    solver_min_planar_clearance
                ),
                "min_cbf": _float_cell(min_cbf),
                "dmin_lin": _float_cell(dmin_lin),
                "slack_sum": _float_cell(slack_sum),
                "slack_max": _float_cell(slack_max),
                "solve_time_mean_ms": _float_cell(solve_mean),
                "solve_time_p95_ms": _float_cell(solve_p95),
                "solve_time_max_ms": _float_cell(solve_max),
                "fail_flag": str(fail_flag),
            }

    except ConfigMismatchError:
        raise
    except Exception as exc:
        error_msg = str(exc)
        row_out = _make_empty_row(case, fail_flag=1)
    finally:
        if roscore_proc is not None:
            _kill_process_group(roscore_proc)
        if roscore_log_file is not None:
            roscore_log_file.close()

    wall_time = time.time() - wall_start
    meta = {
        "run_tag": run_tag,
        "case": dataclasses.asdict(case),
        "safety_cfg": dataclasses.asdict(safety_cfg),
        "config_hash": cfg_hash,
        "return_code": rc,
        "timed_out": timed_out,
        "error": error_msg,
        "wall_time_sec": wall_time,
        "raw_row": row_out,
    }
    with meta_path.open("w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, sort_keys=True)

    return {
        "run_tag": run_tag,
        "case": case,
        "base_key": _base_key(case),
        "config_hash": cfg_hash,
        "row": row_out,
    }


def _require_ros_setup() -> None:
    rc = subprocess.run(
        ["rospack", "find", "coni_mpc"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode
    if rc != 0:
        raise RuntimeError("ROS environment is not ready. Run `source devel/setup.bash` first.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run baseline+ablation grid and export results/raw_runs.csv."
    )
    parser.add_argument("--raw-csv", default="results/raw_runs.csv")
    parser.add_argument("--hash-csv", default="results/raw_runs_config_hashes.csv")
    parser.add_argument("--logs-root", default="results/logs/grid_suite")
    parser.add_argument(
        "--yaml",
        default="src/coni_mpc/parameters/num_sim_non_one_point.yaml",
        help="YAML used to read A2 defaults (slack_max, R_slack).",
    )
    parser.add_argument("--seed-start", type=int, default=1)
    parser.add_argument("--seed-end", type=int, default=100)
    parser.add_argument("--obs", default="20,40,60")
    parser.add_argument("--kappa", default="1.0,1.2,1.5")
    parser.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    parser.add_argument("--timeout-sec", type=float, default=900.0)
    parser.add_argument("--clean", action="store_true")
    parser.add_argument("--continue-on-fail", action="store_true")
    parser.set_defaults(continue_on_fail=True)
    parser.add_argument("--rerun-failed", action="store_true")
    parser.add_argument(
        "--fail-rate-threshold",
        type=float,
        default=0.0,
        help="Mark run fail_flag=1 when solver_fail_rate > this threshold.",
    )
    parser.add_argument(
        "--hard-slack-tol",
        type=float,
        default=1e-9,
        help="Tolerance used to enforce slack_sum/slack_max == 0 for A0/A1.",
    )
    parser.add_argument("--active-uavs", default="0,1,2,3")
    parser.add_argument("--r", type=float, default=1.0)
    parser.add_argument("--v", type=float, default=0.5)
    parser.add_argument("--w", type=float, default=0.25)
    parser.add_argument("--start-z", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.seed_start <= 0 or args.seed_end < args.seed_start:
        print("ERROR: seed range must satisfy 1 <= seed_start <= seed_end", file=sys.stderr)
        return 2

    try:
        obs_values = _parse_int_list_csv(args.obs)
        kappa_values = _parse_float_list_csv(args.kappa)
        args.active_uavs_list = _parse_int_list_csv(args.active_uavs)
    except Exception as exc:
        print(f"ERROR: failed to parse list arguments: {exc}", file=sys.stderr)
        return 2

    workspace_root = pathlib.Path(__file__).resolve().parent.parent
    raw_csv = workspace_root / args.raw_csv
    hash_csv = workspace_root / args.hash_csv
    logs_root = workspace_root / args.logs_root
    yaml_path = workspace_root / args.yaml
    args.yaml_abs = yaml_path

    if args.clean:
        _log(f"Cleaning outputs: {raw_csv}, {hash_csv}, {logs_root}")
        _remove_path(raw_csv)
        _remove_path(hash_csv)
        _remove_path(logs_root)

    logs_root.mkdir(parents=True, exist_ok=True)
    raw_csv.parent.mkdir(parents=True, exist_ok=True)
    hash_csv.parent.mkdir(parents=True, exist_ok=True)

    try:
        _require_ros_setup()
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    soft_slack_max, soft_rho, yaml_hash_payload = _load_soft_defaults(yaml_path)
    safety_map = _make_safety_map(soft_slack_max=soft_slack_max, soft_rho=soft_rho)

    all_cases = _build_cases(
        seed_start=args.seed_start,
        seed_end=args.seed_end,
        obs_values=obs_values,
        kappa_values=kappa_values,
    )
    existing = _read_existing_rows(raw_csv)
    hash_map = _read_hash_map(hash_csv)

    pending: List[Case] = []
    skipped = 0
    for case in all_cases:
        key = _case_key(case)
        if key not in existing:
            pending.append(case)
            continue
        prior_fail = _safe_int(existing[key].get("fail_flag"), default=1) != 0
        if args.rerun_failed and prior_fail:
            pending.append(case)
        else:
            skipped += 1

    total = len(all_cases)
    _log(f"Total cases={total} | pending={len(pending)} | skipped={skipped} | jobs={args.jobs}")
    _log(
        "Safety defaults from YAML: "
        f"A2 slack_max={soft_slack_max:.6f}, R_slack={soft_rho:.6f}"
    )

    if not pending:
        _log("Nothing to run.")
        return 0

    suite_timestamp = time.strftime("%Y%m%d-%H%M%S")
    failures = 0
    fatal_error: Optional[Exception] = None

    executor = ThreadPoolExecutor(max_workers=max(1, args.jobs))
    inflight: Dict[object, Case] = {}
    case_iter = iter(pending)
    completed = 0

    def submit_next() -> bool:
        try:
            case = next(case_iter)
        except StopIteration:
            return False
        fut = executor.submit(
            _run_case,
            case,
            args=args,
            safety_cfg=safety_map[case.safety_variant],
            suite_timestamp=suite_timestamp,
            logs_root=logs_root,
            yaml_hash_payload=yaml_hash_payload,
        )
        inflight[fut] = case
        return True

    try:
        for _ in range(max(1, args.jobs)):
            if not submit_next():
                break

        while inflight:
            done, _ = wait(list(inflight.keys()), return_when=FIRST_COMPLETED)
            for fut in done:
                case = inflight.pop(fut)
                completed += 1
                try:
                    result = fut.result()
                except ConfigMismatchError as exc:
                    fatal_error = exc
                    _log(f"[FATAL] config mismatch on case={_case_key(case)}: {exc}")
                except Exception as exc:
                    failures += 1
                    _log(f"[FAIL] worker exception on case={_case_key(case)}: {exc}")
                    if not args.continue_on_fail:
                        fatal_error = exc
                else:
                    row = result["row"]
                    base_key = result["base_key"]
                    cfg_hash = result["config_hash"]
                    run_tag = result["run_tag"]

                    prev_hash = hash_map.get(base_key)
                    if prev_hash is None:
                        hash_map[base_key] = cfg_hash
                        _atomic_append_csv(
                            hash_csv,
                            HASH_HEADER,
                            {
                                "obs": str(base_key[0]),
                                "kappa": base_key[1],
                                "seed": str(base_key[2]),
                                "config_hash": str(cfg_hash),
                                "source_run_tag": str(run_tag),
                            },
                        )
                    elif prev_hash != cfg_hash:
                        fatal_error = ConfigMismatchError(
                            "config hash mismatch for base key "
                            f"{base_key}: {prev_hash} != {cfg_hash}"
                        )
                        _log(f"[FATAL] {fatal_error}")

                    _atomic_append_csv(raw_csv, RAW_HEADER, row)
                    run_failed = _safe_int(row.get("fail_flag"), default=1) != 0
                    if run_failed:
                        failures += 1
                        _log(
                            f"[{completed}/{len(pending)}] FAIL run_tag={run_tag} "
                            f"case={_case_key(case)}"
                        )
                        if not args.continue_on_fail:
                            fatal_error = RuntimeError(f"run failed: {run_tag}")
                    else:
                        _log(
                            f"[{completed}/{len(pending)}] OK   run_tag={run_tag} "
                            f"case={_case_key(case)}"
                        )

                if fatal_error is None:
                    submit_next()

            if fatal_error is not None:
                break
    finally:
        executor.shutdown(wait=True)

    if fatal_error is not None:
        print(f"ERROR: {fatal_error}", file=sys.stderr)
        return 3

    _log(f"Finished pending runs={len(pending)} | failures={failures} | raw_csv={raw_csv}")
    if failures > 0 and not args.continue_on_fail:
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
