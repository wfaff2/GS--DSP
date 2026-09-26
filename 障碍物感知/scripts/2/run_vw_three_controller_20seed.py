#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import os
import pathlib
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import yaml  # type: ignore
except Exception:  # pragma: no cover
    yaml = None


CONTROLLERS = {
    "inertial_frozen": ("inertial", "frozen", True, False),
    "noninertial_frozen": ("noninertial", "frozen", False, True),
    "noninertial_stage": ("noninertial", "stage", False, True),
}

# Selecting an ephemeral port and starting roscore must be atomic within this
# multi-threaded runner.  Without the lock, two workers can observe the same
# free port after the probe socket closes and then join the same ROS master.
ROSCORE_START_LOCK = threading.Lock()

MANIFEST_HEADER = [
    "run_index",
    "controller",
    "frame_mode",
    "ugv_rollout_mode",
    "stage_geometry_mode",
    "obstacle_sensing_mode",
    "v",
    "w",
    "seed",
    "noise_seed",
    "obs",
    "kappa",
    "r_slack",
    "active_uavs",
    "run_tag",
    "status",
    "return_code",
    "timed_out",
    "wall_time_sec",
    "metrics_csv",
    "step_metrics_csv",
    "ugv_horizon_summary_csv",
    "ugv_horizon_steps_csv",
    "solver_obstacle_drift_csv",
    "run_dir",
    "rosrun_log",
    "error",
]


@dataclasses.dataclass(frozen=True)
class Case:
    run_index: int
    controller: str
    frame_mode: str
    ugv_rollout_mode: str
    use_inertial_frame: bool
    use_noninertial_frame: bool
    v: float
    w: float
    seed: int
    obs: int


@dataclasses.dataclass(frozen=True)
class SafetyConfig:
    cbf_enabled: bool
    cbf_use_in_sim: bool
    slack_max: float
    r_slack: float


def _stage_geometry_mode(case: Case, args: argparse.Namespace) -> str:
    if case.ugv_rollout_mode != "stage":
        return "frozen_current_curvature"
    if args.use_predicted_stage_geometry:
        return "full_predicted"
    if args.use_split_predicted_stage_geometry:
        return "split_predicted"
    return "legacy_current_curvature"


def _log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def _bool_str(value: bool) -> str:
    return "true" if value else "false"


def _float_cell(value: float) -> str:
    return "nan" if not math.isfinite(value) else f"{value:.6f}"


def _safe_int(value: object, default: int = 0) -> int:
    try:
        return int(float(value))  # type: ignore[arg-type]
    except Exception:
        return default


def _parse_float_list_csv(text: str) -> List[float]:
    out: List[float] = []
    for part in text.split(","):
        part = part.strip()
        if part:
            out.append(float(part))
    if not out:
        raise ValueError("empty float list")
    return out


def _parse_int_list_csv(text: str) -> List[int]:
    out: List[int] = []
    for part in text.split(","):
        part = part.strip()
        if part:
            out.append(int(part))
    if not out:
        raise ValueError("empty integer list")
    return out


def _parse_controller_list(text: str) -> List[str]:
    controllers: List[str] = []
    for part in text.split(","):
        name = part.strip()
        if not name:
            continue
        if name not in CONTROLLERS:
            raise ValueError(
                f"unknown controller {name!r}; valid values: {','.join(CONTROLLERS)}"
            )
        controllers.append(name)
    if not controllers:
        raise ValueError("empty controller list")
    return controllers


def _label_float(value: float) -> str:
    return f"{float(value):.2f}".rstrip("0").rstrip(".").replace(".", "p")


def _remove_path(path: pathlib.Path) -> None:
    if path.is_file() or path.is_symlink():
        path.unlink()
        return
    if path.exists():
        shutil.rmtree(path)


def _find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        return int(sock.getsockname()[1])


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
    return (
        subprocess.run(
            ["rosparam", "list"],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ).returncode
        == 0
    )


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
                return proc.wait(), False
            return proc.wait(timeout=timeout_sec), False
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
        text = "[" + ",".join(str(item) for item in value) + "]"
    else:
        text = str(value)
    rc = _run_quick_cmd(["rosparam", "set", key, text], env=env, log_handle=log_handle)
    if rc != 0:
        raise RuntimeError(f"rosparam set failed: {key}={text}")


def _require_ros_setup() -> None:
    rc = subprocess.run(
        ["rospack", "find", "coni_mpc"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode
    if rc != 0:
        raise RuntimeError("ROS environment is not ready. Run `source devel/setup.bash` first.")


def _load_soft_cbf_defaults(yaml_path: pathlib.Path) -> SafetyConfig:
    slack_max = 10.0
    r_slack = 100.0
    if yaml is not None and yaml_path.exists():
        with yaml_path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if isinstance(data, dict):
            cbf = data.get("cbf", {})
            if isinstance(cbf, dict):
                slack_max = float(cbf.get("slack_max", cbf.get("s_max", slack_max)))
                r_slack = float(cbf.get("R_slack", cbf.get("rho_s", r_slack)))
    return SafetyConfig(
        cbf_enabled=True,
        cbf_use_in_sim=True,
        slack_max=slack_max,
        r_slack=r_slack,
    )


def _read_scope_all(metrics_csv: pathlib.Path) -> Optional[Dict[str, str]]:
    if not metrics_csv.is_file():
        return None
    with metrics_csv.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        matched: Optional[Dict[str, str]] = None
        for row in reader:
            if row.get("scope") == "all":
                matched = row
        return matched


def _case_completed(
    case: Case,
    metrics_csv: pathlib.Path,
    *,
    expected_kappa: float,
    expected_noise_seed: int,
    expected_obstacle_sensing_mode: str,
) -> bool:
    row = _read_scope_all(metrics_csv)
    if row is None:
        return False
    if _safe_int(row.get("seed"), default=-1) != case.seed:
        return False
    if _safe_int(row.get("obstacle_count"), default=-1) != case.obs:
        return False
    if _safe_int(row.get("noise_seed"), default=-1) != expected_noise_seed:
        return False
    if row.get("obstacle_sensing_mode") != expected_obstacle_sensing_mode:
        return False
    try:
        if abs(float(row.get("kappa", "nan")) - expected_kappa) > 1e-6:
            return False
        if abs(float(row.get("requested_v", "nan")) - case.v) > 1e-6:
            return False
        if abs(float(row.get("requested_w", "nan")) - case.w) > 1e-6:
            return False
    except Exception:
        return False
    return (
        row.get("frame_mode_effective") == case.frame_mode
        and row.get("ugv_rollout_mode") == case.ugv_rollout_mode
    )


def _build_cases(
    *,
    controllers: Sequence[str],
    v_values: Sequence[float],
    w_values: Sequence[float],
    scan_mode: str,
    seeds: Sequence[int],
    obs: int,
) -> List[Case]:
    out: List[Case] = []
    run_index = 0
    if scan_mode == "diagonal":
        vw_pairs = list(zip(v_values, w_values))
    else:
        vw_pairs = [(v, w) for v in v_values for w in w_values]

    for v, w in vw_pairs:
        for seed in seeds:
            for controller in controllers:
                frame_mode, rollout, use_i, use_non = CONTROLLERS[controller]
                run_index += 1
                out.append(
                    Case(
                        run_index=run_index,
                        controller=controller,
                        frame_mode=frame_mode,
                        ugv_rollout_mode=rollout,
                        use_inertial_frame=use_i,
                        use_noninertial_frame=use_non,
                        v=float(v),
                        w=float(w),
                        seed=int(seed),
                        obs=int(obs),
                    )
                )
    return out


def _case_paths(
    run_root: pathlib.Path,
    case: Case,
) -> Tuple[
    pathlib.Path,
    pathlib.Path,
    pathlib.Path,
    pathlib.Path,
    pathlib.Path,
    pathlib.Path,
]:
    run_dir = (
        run_root
        / "logs"
        / case.controller
        / f"v_{_label_float(case.v)}"
        / f"w_{_label_float(case.w)}"
        / f"seed_{case.seed:03d}"
    )
    metrics_csv = run_dir / "metrics.csv"
    step_metrics_csv = run_dir / "metrics_steps.csv"
    ugv_horizon_summary_csv = run_dir / "metrics_ugv_horizon_summary.csv"
    ugv_horizon_steps_csv = run_dir / "metrics_ugv_horizon_steps.csv"
    solver_obstacle_drift_csv = run_dir / "metrics_solver_obstacle_drift.csv"
    return (
        run_dir,
        metrics_csv,
        step_metrics_csv,
        ugv_horizon_summary_csv,
        ugv_horizon_steps_csv,
        solver_obstacle_drift_csv,
    )


def _run_case(
    case: Case,
    *,
    args: argparse.Namespace,
    run_root: pathlib.Path,
    yaml_path: pathlib.Path,
    safety_cfg: SafetyConfig,
    suite_timestamp: str,
) -> Dict[str, str]:
    (
        run_dir,
        metrics_csv,
        step_metrics_csv,
        ugv_horizon_summary_csv,
        ugv_horizon_steps_csv,
        solver_obstacle_drift_csv,
    ) = _case_paths(run_root, case)
    run_dir.mkdir(parents=True, exist_ok=True)
    run_tag = (
        f"vw3_{suite_timestamp}_{case.run_index:04d}_{case.controller}_"
        f"{_stage_geometry_mode(case, args)}_"
        f"{args.obstacle_sensing_mode}_v{_label_float(case.v)}_"
        f"w{_label_float(case.w)}_s{case.seed}"
    )
    setup_log_path = run_dir / "setup.log"
    roscore_log_path = run_dir / "roscore.log"
    rosrun_log_path = run_dir / "rosrun.log"
    meta_path = run_dir / "meta.json"

    if args.resume and not args.rerun and _case_completed(
        case,
        metrics_csv,
        expected_kappa=args.kappa,
        expected_noise_seed=(
            case.seed if args.vary_noise_seed else args.noise_seed
        ),
        expected_obstacle_sensing_mode=args.obstacle_sensing_mode,
    ):
        return {
            "run_index": str(case.run_index),
            "controller": case.controller,
            "frame_mode": case.frame_mode,
            "ugv_rollout_mode": case.ugv_rollout_mode,
            "stage_geometry_mode": _stage_geometry_mode(case, args),
            "obstacle_sensing_mode": args.obstacle_sensing_mode,
            "v": _float_cell(case.v),
            "w": _float_cell(case.w),
            "seed": str(case.seed),
            "noise_seed": str(
                case.seed if args.vary_noise_seed else args.noise_seed
            ),
            "obs": str(case.obs),
            "kappa": _float_cell(args.kappa),
            "r_slack": _float_cell(safety_cfg.r_slack),
            "active_uavs": ";".join(str(item) for item in args.active_uavs_list),
            "run_tag": run_tag,
            "status": "skipped",
            "return_code": "0",
            "timed_out": "0",
            "wall_time_sec": "0.000000",
            "metrics_csv": str(metrics_csv),
            "step_metrics_csv": str(step_metrics_csv),
            "ugv_horizon_summary_csv": str(ugv_horizon_summary_csv),
            "ugv_horizon_steps_csv": str(ugv_horizon_steps_csv),
            "solver_obstacle_drift_csv": str(solver_obstacle_drift_csv),
            "run_dir": str(run_dir),
            "rosrun_log": str(rosrun_log_path),
            "error": "",
        }

    for path in (
        metrics_csv,
        step_metrics_csv,
        ugv_horizon_summary_csv,
        ugv_horizon_steps_csv,
        solver_obstacle_drift_csv,
    ):
        if path.exists():
            path.unlink()

    env = os.environ.copy()
    env["ROS_IP"] = "127.0.0.1"
    env["ROS_HOSTNAME"] = "127.0.0.1"
    env["ROS_HOME"] = str(run_dir / "ros_home")
    env["ROS_LOG_DIR"] = str(run_dir / "ros_logs")
    pathlib.Path(env["ROS_HOME"]).mkdir(parents=True, exist_ok=True)
    pathlib.Path(env["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)

    rc = 1
    timed_out = False
    error_msg = ""
    status = "failed"
    roscore_proc: Optional[subprocess.Popen] = None
    roscore_log_file = None
    wall_start = time.time()

    try:
        with ROSCORE_START_LOCK:
            port = _find_free_port()
            env["ROS_MASTER_URI"] = f"http://127.0.0.1:{port}"
            roscore_proc, roscore_log_file = _start_roscore(
                env, port, roscore_log_path
            )
        with setup_log_path.open("w", encoding="utf-8") as setup_log:
            rc_load = _run_quick_cmd(
                [
                    "rosparam",
                    "load",
                    str(yaml_path),
                    "/num_sim_non_one_point_node",
                ],
                env=env,
                log_handle=setup_log,
            )
            if rc_load != 0:
                raise RuntimeError(f"rosparam load failed: {yaml_path}")

            params = [
                ("/num_sim_non_one_point_node/frame_mode", case.frame_mode),
                (
                    "/num_sim_non_one_point_node/use_inertial_frame",
                    case.use_inertial_frame,
                ),
                (
                    "/num_sim_non_one_point_node/use_noninertial_frame",
                    case.use_noninertial_frame,
                ),
                ("/num_sim_non_one_point_node/ugv_rollout_mode", case.ugv_rollout_mode),
                (
                    "/num_sim_non_one_point_node/non_inertial_predictor/use_predicted_stage_geometry",
                    args.use_predicted_stage_geometry,
                ),
                (
                    "/num_sim_non_one_point_node/non_inertial_predictor/use_split_predicted_stage_geometry",
                    args.use_split_predicted_stage_geometry,
                ),
                ("/num_sim_non_one_point_node/safety_variant", args.safety_variant),
                (
                    "/num_sim_non_one_point_node/obstacle_sensing/mode",
                    args.obstacle_sensing_mode,
                ),
                ("/num_sim_non_one_point_node/metrics/debug", False),
                ("/num_sim_non_one_point_node/active_uavs", args.active_uavs_list),
                ("/num_sim_non_one_point_node/exclude_uav0_from_simulation", True),
                ("/num_sim_non_one_point_node/exclude_uav0_from_all_metrics", True),
                ("/num_sim_non_one_point_node/random_obstacles/count", case.obs),
                ("/num_sim_non_one_point_node/random_obstacles/seed",
                 args.fixed_obs_seed if args.fixed_obs_seed > 0 else case.seed),
                ("/num_sim_non_one_point_node/random_obstacles/x_range",
                 [-args.obs_half_range, args.obs_half_range]),
                ("/num_sim_non_one_point_node/random_obstacles/y_range",
                 [-args.obs_y_half_range_eff, args.obs_y_half_range_eff]),
                ("/num_sim_non_one_point_node/noise_seed",
                 case.seed if args.vary_noise_seed else args.noise_seed),
                ("/num_sim_non_one_point_node/kappa", args.kappa),
                ("/num_sim_non_one_point_node/metrics_csv", str(metrics_csv)),
                (
                    "/num_sim_non_one_point_node/solver_obstacle_drift_csv",
                    str(solver_obstacle_drift_csv),
                ),
                ("/num_sim_non_one_point_node/run_tag", run_tag),
                ("/num_sim_non_one_point_node/use_dynamic_obs", False),
                ("/num_sim_non_one_point_node/warm_start", True),
                ("/num_sim_non_one_point_node/dynamic_obs/start_z", args.start_z),
                ("/num_sim_non_one_point_node/car_trajectory_mode", args.car_trajectory_mode),
                (
                    "/num_sim_non_one_point_node/figure_eight_obstacles/enabled",
                    not args.disable_figure_eight_obstacles,
                ),
                ("/num_sim_non_one_point_node/sim_duration_sec", args.sim_duration_sec),
                (
                    "/num_sim_non_one_point_node/ugv_path/longitudinal_accel_max",
                    args.ugv_longitudinal_accel,
                ),
                (
                    "/num_sim_non_one_point_node/ugv_path/longitudinal_decel_max",
                    args.ugv_longitudinal_decel,
                ),
                (
                    "/num_sim_non_one_point_node/ugv_path/longitudinal_jerk_max",
                    args.ugv_longitudinal_jerk,
                ),
                (
                    "/num_sim_non_one_point_node/ugv_path/yaw_accel_max",
                    args.ugv_yaw_accel,
                ),
                (
                    "/num_sim_non_one_point_node/ugv_path/yaw_jerk_max",
                    args.ugv_yaw_jerk,
                ),
                ("/num_sim_non_one_point_node/obstacles_for_car_traj_only", False),
                ("/num_sim_non_one_point_node/publish_obstacle_markers", False),
                ("/num_sim_non_one_point_node/cbf/enabled", safety_cfg.cbf_enabled),
                ("/num_sim_non_one_point_node/cbf/use_in_sim", safety_cfg.cbf_use_in_sim),
                ("/num_sim_non_one_point_node/cbf/slack_max", safety_cfg.slack_max),
                ("/num_sim_non_one_point_node/cbf/R_slack", safety_cfg.r_slack),
            ]
            sensing_branch = (
                "degraded"
                if args.obstacle_sensing_mode == "online_degraded"
                else "nominal"
            )
            sensing_overrides = [
                (
                    "/num_sim_non_one_point_node/obstacle_sensing/update_rate_hz",
                    args.sensing_update_rate_hz,
                ),
                (
                    f"/num_sim_non_one_point_node/obstacle_sensing/{sensing_branch}/position_std_m",
                    args.sensing_position_std_m,
                ),
                (
                    f"/num_sim_non_one_point_node/obstacle_sensing/{sensing_branch}/delay_sec",
                    args.sensing_delay_sec,
                ),
                (
                    f"/num_sim_non_one_point_node/obstacle_sensing/{sensing_branch}/dropout_probability",
                    args.sensing_dropout_probability,
                ),
            ]
            params.extend(
                (key, value)
                for key, value in sensing_overrides
                if math.isfinite(value)
            )
            for key, value in params:
                _rosparam_set(env, key, value, setup_log)

        if args.node_executable.is_file():
            cmd = [
                str(args.node_executable),
                "-r",
                str(args.r),
                "-v",
                str(case.v),
                "-w",
                str(case.w),
            ]
        else:
            cmd = [
                "rosrun",
                "coni_mpc",
                "num_sim_non_one_point_node",
                "-r",
                str(args.r),
                "-v",
                str(case.v),
                "-w",
                str(case.w),
            ]
        rc, timed_out = _run_cmd_logged(cmd, env, rosrun_log_path, args.timeout_sec)
        if timed_out:
            status = "timeout"
        elif rc != 0:
            status = "failed"
        elif _read_scope_all(metrics_csv) is None:
            status = "failed"
            error_msg = "metrics.csv has no scope=all row"
        else:
            status = "ok"
    except Exception as exc:
        error_msg = str(exc)
        status = "failed"
    finally:
        if roscore_proc is not None:
            _kill_process_group(roscore_proc)
        if roscore_log_file is not None:
            roscore_log_file.close()

    wall_time = time.time() - wall_start
    meta = {
        "case": dataclasses.asdict(case),
        "run_tag": run_tag,
        "status": status,
        "return_code": rc,
        "timed_out": timed_out,
        "wall_time_sec": wall_time,
        "noise_seed": case.seed if args.vary_noise_seed else args.noise_seed,
        "obstacle_sensing_mode": args.obstacle_sensing_mode,
        "stage_geometry_mode": _stage_geometry_mode(case, args),
        "r_slack": safety_cfg.r_slack,
        "error": error_msg,
    }
    with meta_path.open("w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, sort_keys=True)

    return {
        "run_index": str(case.run_index),
        "controller": case.controller,
        "frame_mode": case.frame_mode,
        "ugv_rollout_mode": case.ugv_rollout_mode,
        "stage_geometry_mode": _stage_geometry_mode(case, args),
        "obstacle_sensing_mode": args.obstacle_sensing_mode,
        "v": _float_cell(case.v),
        "w": _float_cell(case.w),
        "seed": str(case.seed),
        "noise_seed": str(case.seed if args.vary_noise_seed else args.noise_seed),
        "obs": str(case.obs),
        "kappa": _float_cell(args.kappa),
        "r_slack": _float_cell(safety_cfg.r_slack),
        "active_uavs": ";".join(str(item) for item in args.active_uavs_list),
        "run_tag": run_tag,
        "status": status,
        "return_code": str(rc),
        "timed_out": "1" if timed_out else "0",
        "wall_time_sec": _float_cell(wall_time),
        "metrics_csv": str(metrics_csv),
        "step_metrics_csv": str(step_metrics_csv),
        "ugv_horizon_summary_csv": str(ugv_horizon_summary_csv),
        "ugv_horizon_steps_csv": str(ugv_horizon_steps_csv),
        "solver_obstacle_drift_csv": str(solver_obstacle_drift_csv),
        "run_dir": str(run_dir),
        "rosrun_log": str(rosrun_log_path),
        "error": error_msg,
    }


def _append_manifest_row(path: pathlib.Path, row: Dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    need_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=MANIFEST_HEADER)
        if need_header:
            writer.writeheader()
        writer.writerow(row)


def _run_aggregator(run_root: pathlib.Path) -> int:
    script = pathlib.Path(__file__).resolve().with_name(
        "aggregate_vw_three_controller_metrics.py"
    )
    if not script.is_file():
        _log(f"Aggregator script not found: {script}")
        return 1
    return subprocess.run(
        [
            sys.executable,
            str(script),
            str(run_root),
            "--out-dir",
            str(run_root / "tables"),
        ]
    ).returncode


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run V/W grid experiments for inertial_frozen, noninertial_frozen, "
            "and noninertial_stage controllers."
        )
    )
    parser.add_argument("--run-root", default="")
    parser.add_argument(
        "--yaml",
        default="src/coni_mpc/parameters/num_sim_non_one_point.yaml",
    )
    parser.add_argument("--jobs", type=int, default=16)
    parser.add_argument("--obs", type=int, default=100)
    parser.add_argument("--seed-start", type=int, default=1)
    parser.add_argument("--seed-end", type=int, default=20)
    parser.add_argument(
        "--seeds",
        default="",
        help=(
            "Optional comma-separated explicit seed list. When provided, it "
            "takes precedence over --seed-start/--seed-end."
        ),
    )
    parser.add_argument("--v-values", default="1.0,1.5,2.0,2.5,3.0")
    parser.add_argument("--w-values", default="0.5,1.0,1.5,2.0")
    parser.add_argument(
        "--scan-mode",
        choices=("grid", "diagonal"),
        default="grid",
        help=(
            "Case enumeration mode: grid runs the full Cartesian product of "
            "V/W values; diagonal pairs the i-th V with the i-th W."
        ),
    )
    parser.add_argument(
        "--controllers",
        default="inertial_frozen,noninertial_frozen,noninertial_stage",
    )
    parser.add_argument("--active-uavs", default="1,2,3")
    parser.add_argument("--r", type=float, default=1.0)
    parser.add_argument("--kappa", type=float, default=1.0)
    parser.add_argument("--safety-variant", default="A2_soft_cbf")
    parser.add_argument(
        "--obstacle-sensing-mode",
        choices=("map", "online_nominal", "online_degraded"),
        default="map",
        help=(
            "Static-obstacle input mode. Use online_nominal or "
            "online_degraded for the Reviewer 6 Comment 3.3 experiment."
        ),
    )
    parser.add_argument(
        "--use-predicted-stage-geometry",
        action="store_true",
        default=False,
        help="For the Stage controller, build target geometry from the same "
             "stage-wise non-inertial profiles used by the controller.",
    )
    parser.add_argument(
        "--use-split-predicted-stage-geometry",
        action="store_true",
        default=False,
        help="Keep the current-curvature target reference but transform HOCBF "
             "obstacles with the stage-wise predicted frame profile.",
    )
    parser.add_argument(
        "--sensing-update-rate-hz",
        type=float,
        default=float("nan"),
        help="Override obstacle_sensing/update_rate_hz for an ablation run.",
    )
    parser.add_argument(
        "--sensing-position-std-m",
        type=float,
        default=float("nan"),
        help="Override the selected online mode's position noise standard deviation.",
    )
    parser.add_argument(
        "--sensing-delay-sec",
        type=float,
        default=float("nan"),
        help="Override the selected online mode's measurement delay.",
    )
    parser.add_argument(
        "--sensing-dropout-probability",
        type=float,
        default=float("nan"),
        help="Override the selected online mode's dropout probability.",
    )
    parser.add_argument(
        "--cbf-slack-max",
        type=float,
        default=float("nan"),
        help="Override cbf/slack_max. Use 0 with --safety-variant A1_hard_cbf.",
    )
    parser.add_argument(
        "--r-slack",
        type=float,
        default=float("nan"),
        help="Override cbf/R_slack from the YAML when finite and positive.",
    )
    parser.add_argument(
        "--noise-seed",
        type=int,
        default=1,
        help="Noise/random perturbation seed shared by every case (default: 1).",
    )
    parser.add_argument(
        "--vary-noise-seed",
        action="store_true",
        default=False,
        help="When set, each case uses its own seed as the noise seed instead of "
             "the fixed --noise-seed value. Combined with --fixed-obs-seed this "
             "gives fixed environment + varying noise = robustness study. Without "
             "--fixed-obs-seed both obstacle layout AND noise vary = generalization study.",
    )
    parser.add_argument(
        "--fixed-obs-seed",
        type=int,
        default=0,
        help="When > 0, ALL cases use this fixed obstacle seed (same environment). "
             "noise_seed still varies per case when --vary-noise-seed is set. "
             "Use for robustness study: one fixed obstacle layout + N noise realizations. "
             "Default 0 = obstacle seed varies with case.seed (different layout per seed).",
    )
    parser.add_argument(
        "--obs-half-range",
        type=float,
        default=10.0,
        help="Half-range for random obstacle placement in both x and y (default: 10.0). "
             "Obstacles are placed in [-val, val] x [-val, val]. Set to V/W+1.5 "
             "to concentrate obstacles inside the figure-8 loop area.",
    )
    parser.add_argument(
        "--obs-y-half-range",
        type=float,
        default=float("nan"),
        help="Override y half-range independently (default: same as --obs-half-range).",
    )
    parser.add_argument("--start-z", type=float, default=2.0)
    parser.add_argument("--timeout-sec", type=float, default=900.0)
    parser.add_argument("--car-trajectory-mode", default="obstacle_aware")
    parser.add_argument(
        "--disable-figure-eight-obstacles",
        action="store_true",
        default=False,
        help=(
            "Disable the fixed strategic obstacles that are otherwise appended "
            "for figure-eight trajectories."
        ),
    )
    parser.add_argument("--sim-duration-sec", type=float, default=-1.0,
                        dest="sim_duration_sec",
                        help="Override simulation duration in seconds (<0 = use default)")
    parser.add_argument("--ugv-longitudinal-accel", type=float, default=2.0)
    parser.add_argument("--ugv-longitudinal-decel", type=float, default=2.5)
    parser.add_argument("--ugv-longitudinal-jerk", type=float, default=8.0)
    parser.add_argument("--ugv-yaw-accel", type=float, default=1.0)
    parser.add_argument("--ugv-yaw-jerk", type=float, default=8.0)
    parser.add_argument("--clean", action="store_true")
    parser.add_argument("--resume", action="store_true", default=True)
    parser.add_argument("--no-resume", action="store_false", dest="resume")
    parser.add_argument("--rerun", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--aggregate", action="store_true", default=True)
    parser.add_argument("--no-aggregate", action="store_false", dest="aggregate")
    return parser.parse_args()


def _discover_workspace_root(script_path: pathlib.Path) -> pathlib.Path:
    resolved = script_path.resolve()
    candidates = [resolved.parent] + list(resolved.parents)
    for candidate in candidates:
        if (candidate / "src" / "coni_mpc").exists() and (candidate / "devel").exists():
            return candidate
    return resolved.parent.parent


def main() -> int:
    args = parse_args()
    workspace_root = _discover_workspace_root(pathlib.Path(__file__))
    try:
        args.v_list = _parse_float_list_csv(args.v_values)
        args.w_list = _parse_float_list_csv(args.w_values)
        args.controllers_list = _parse_controller_list(args.controllers)
        args.active_uavs_list = _parse_int_list_csv(args.active_uavs)
        args.seed_list = (
            _parse_int_list_csv(args.seeds)
            if args.seeds.strip()
            else list(range(args.seed_start, args.seed_end + 1))
        )
    except Exception as exc:
        print(f"ERROR: failed to parse arguments: {exc}", file=sys.stderr)
        return 2

    if args.seed_start <= 0 or args.seed_end < args.seed_start:
        print("ERROR: seed range must satisfy 1 <= seed_start <= seed_end", file=sys.stderr)
        return 2
    if any(seed <= 0 for seed in args.seed_list):
        print("ERROR: every seed must be positive", file=sys.stderr)
        return 2
    if len(set(args.seed_list)) != len(args.seed_list):
        print("ERROR: --seeds must not contain duplicates", file=sys.stderr)
        return 2
    if args.jobs <= 0:
        print("ERROR: --jobs must be positive", file=sys.stderr)
        return 2
    if args.use_predicted_stage_geometry and args.use_split_predicted_stage_geometry:
        print(
            "ERROR: full and split predicted Stage geometry modes are mutually exclusive",
            file=sys.stderr,
        )
        return 2
    sensing_override_checks = [
        ("--sensing-update-rate-hz", args.sensing_update_rate_hz, lambda x: x > 0.0),
        ("--sensing-position-std-m", args.sensing_position_std_m, lambda x: x >= 0.0),
        ("--sensing-delay-sec", args.sensing_delay_sec, lambda x: x >= 0.0),
        (
            "--sensing-dropout-probability",
            args.sensing_dropout_probability,
            lambda x: 0.0 <= x <= 1.0,
        ),
    ]
    for name, value, predicate in sensing_override_checks:
        if math.isfinite(value) and not predicate(value):
            print(f"ERROR: invalid value for {name}: {value}", file=sys.stderr)
            return 2
    if args.noise_seed <= 0:
        print("ERROR: --noise-seed must be positive", file=sys.stderr)
        return 2
    if args.scan_mode == "diagonal" and len(args.v_list) != len(args.w_list):
        print(
            "ERROR: --scan-mode diagonal requires --v-values and --w-values to "
            "have the same length",
            file=sys.stderr,
        )
        return 2
    args.obs_y_half_range_eff = (
        args.obs_y_half_range if not math.isnan(args.obs_y_half_range)
        else args.obs_half_range
    )

    run_root = (
        pathlib.Path(args.run_root)
        if args.run_root
        else workspace_root
        / "results"
        / f"vw_three_controller_obs{args.obs}_20seed_{time.strftime('%Y%m%d-%H%M%S')}"
    )
    if not run_root.is_absolute():
        run_root = workspace_root / run_root
    yaml_path = pathlib.Path(args.yaml)
    if not yaml_path.is_absolute():
        yaml_path = workspace_root / yaml_path
    args.node_executable = (
        workspace_root / "devel" / "lib" / "coni_mpc" / "num_sim_non_one_point_node"
    )

    cases = _build_cases(
        controllers=args.controllers_list,
        v_values=args.v_list,
        w_values=args.w_list,
        scan_mode=args.scan_mode,
        seeds=args.seed_list,
        obs=args.obs,
    )
    manifest_csv = run_root / "run_manifest.csv"
    _log(
        f"Total cases={len(cases)} | controllers={len(args.controllers_list)} "
        f"| scan_mode={args.scan_mode} | V={len(args.v_list)} | W={len(args.w_list)} "
        f"| seeds={len(args.seed_list)} ({','.join(map(str, args.seed_list))}) "
        f"| jobs={args.jobs}"
    )
    _log(f"Run root: {run_root}")
    if args.dry_run:
        return 0

    try:
        _require_ros_setup()
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if args.clean:
        _log(f"Cleaning run root: {run_root}")
        _remove_path(run_root)
    run_root.mkdir(parents=True, exist_ok=True)

    safety_cfg = _load_soft_cbf_defaults(yaml_path)
    if args.safety_variant == "A1_hard_cbf":
        safety_cfg = SafetyConfig(
            cbf_enabled=True,
            cbf_use_in_sim=True,
            slack_max=0.0,
            r_slack=safety_cfg.r_slack,
        )
    elif args.safety_variant == "A0_no_cbf":
        safety_cfg = SafetyConfig(
            cbf_enabled=False,
            cbf_use_in_sim=False,
            slack_max=0.0,
            r_slack=safety_cfg.r_slack,
        )
    if math.isfinite(args.cbf_slack_max):
        if args.cbf_slack_max < 0.0:
            print("ERROR: --cbf-slack-max must be non-negative when provided", file=sys.stderr)
            return 2
        safety_cfg = SafetyConfig(
            cbf_enabled=safety_cfg.cbf_enabled,
            cbf_use_in_sim=safety_cfg.cbf_use_in_sim,
            slack_max=float(args.cbf_slack_max),
            r_slack=safety_cfg.r_slack,
        )
    if math.isfinite(args.r_slack):
        if args.r_slack <= 0.0:
            print("ERROR: --r-slack must be positive when provided", file=sys.stderr)
            return 2
        safety_cfg = SafetyConfig(
            cbf_enabled=safety_cfg.cbf_enabled,
            cbf_use_in_sim=safety_cfg.cbf_use_in_sim,
            slack_max=safety_cfg.slack_max,
            r_slack=float(args.r_slack),
        )
    _log(
        f"safety_variant={args.safety_variant}: "
        f"slack_max={safety_cfg.slack_max:.6f}, R_slack={safety_cfg.r_slack:.6f}"
    )
    _log(f"obstacle_sensing_mode={args.obstacle_sensing_mode}")
    if args.fixed_obs_seed > 0:
        _log(f"obs_seed=FIXED({args.fixed_obs_seed}) — same obstacle layout for all cases")
    else:
        _log("obs_seed=VARY — obstacle layout differs per seed (generalization study)")
    if args.vary_noise_seed:
        _log("noise_seed=VARY (each case uses its own seed as noise seed)")
    else:
        _log(f"Fixed noise_seed={args.noise_seed}")

    suite_timestamp = time.strftime("%Y%m%d-%H%M%S")
    executor = ThreadPoolExecutor(max_workers=max(1, args.jobs))
    inflight: Dict[object, Case] = {}
    case_iter = iter(cases)
    completed = 0
    failures = 0

    def submit_next() -> bool:
        try:
            case = next(case_iter)
        except StopIteration:
            return False
        fut = executor.submit(
            _run_case,
            case,
            args=args,
            run_root=run_root,
            yaml_path=yaml_path,
            safety_cfg=safety_cfg,
            suite_timestamp=suite_timestamp,
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
                    row = fut.result()
                except Exception as exc:
                    row = {
                        "run_index": str(case.run_index),
                        "controller": case.controller,
                        "frame_mode": case.frame_mode,
                        "ugv_rollout_mode": case.ugv_rollout_mode,
                        "stage_geometry_mode": _stage_geometry_mode(case, args),
                        "obstacle_sensing_mode": args.obstacle_sensing_mode,
                        "v": _float_cell(case.v),
                        "w": _float_cell(case.w),
                        "seed": str(case.seed),
                        "noise_seed": str(
                            case.seed if args.vary_noise_seed else args.noise_seed
                        ),
                        "obs": str(case.obs),
                        "kappa": _float_cell(args.kappa),
                        "r_slack": _float_cell(safety_cfg.r_slack),
                        "active_uavs": ";".join(
                            str(item) for item in args.active_uavs_list
                        ),
                        "run_tag": "",
                        "status": "failed",
                        "return_code": "1",
                        "timed_out": "0",
                        "wall_time_sec": "nan",
                        "metrics_csv": "",
                        "step_metrics_csv": "",
                        "ugv_horizon_summary_csv": "",
                        "ugv_horizon_steps_csv": "",
                        "solver_obstacle_drift_csv": "",
                        "run_dir": "",
                        "rosrun_log": "",
                        "error": str(exc),
                    }
                _append_manifest_row(manifest_csv, row)
                if row["status"] not in {"ok", "skipped"}:
                    failures += 1
                _log(
                    f"[{completed}/{len(cases)}] {row['status'].upper():7s} "
                    f"{case.controller} V={case.v:g} W={case.w:g} seed={case.seed}"
                )
                submit_next()
    finally:
        executor.shutdown(wait=True)

    _log(f"Finished cases={len(cases)} | failures={failures} | manifest={manifest_csv}")
    if args.aggregate:
        agg_rc = _run_aggregator(run_root)
        if agg_rc != 0:
            _log(f"Aggregator exited with code {agg_rc}")
            return agg_rc
    return 0 if failures == 0 else 3


if __name__ == "__main__":
    sys.exit(main())
