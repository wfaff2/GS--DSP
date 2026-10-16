#!/usr/bin/env python3
"""Read-only validation of the UGV yaw-only angular-noise change.

This tool joins exact world-command provenance, simulator truth odometry,
recorded dynamic boxes, controller metrics, Field-HOCBF diagnostics and the
captured condensed QP.  It never treats sampled static PCD points as a
continuous collision-geometry ground truth.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from qp_numeric_audit import load_capture  # noqa: E402

NOD = 87
NSTAGE = 21
FIELD_LOCAL_ROW = 9
ROWS_PER_STAGE = 10
DEFAULT_REFERENCE_N = np.array([0.0, 1.5, 2.0])


def finite_stats(values: Iterable[float]) -> Dict[str, object]:
    a = np.asarray(list(values), dtype=float)
    a = a[np.isfinite(a)]
    if not a.size:
        return {"count": 0, "min": None, "mean": None, "p95": None, "max": None}
    return {"count": int(a.size), "min": float(a.min()), "mean": float(a.mean()),
            "p95": float(np.quantile(a, 0.95)), "max": float(a.max())}


def delta_stats(vectors: np.ndarray, threshold: float = 0.5) -> Dict[str, object]:
    norms = np.linalg.norm(np.diff(vectors, axis=0), axis=1)
    result = finite_stats(norms)
    result.update({"count_gt_0p5": int(np.sum(norms > threshold)),
                   "count_gt_1p0": int(np.sum(norms > 1.0))})
    return result


def columns(frame: pd.DataFrame, names: Sequence[str]) -> np.ndarray:
    missing = [name for name in names if name not in frame]
    if missing:
        raise ValueError(f"missing columns: {missing}")
    return frame[list(names)].to_numpy(dtype=float)


def rotations(prov: pd.DataFrame) -> np.ndarray:
    names = [f"R{i}{j}" for i in range(3) for j in range(3)]
    return columns(prov, names).reshape((-1, 3, 3))


def rotation_angles(R: np.ndarray) -> np.ndarray:
    # ZYX Euler angles.  The recorded rotations are far from pitch singularity.
    roll = np.arctan2(R[:, 2, 1], R[:, 2, 2])
    pitch = np.arcsin(np.clip(-R[:, 2, 0], -1.0, 1.0))
    yaw = np.arctan2(R[:, 1, 0], R[:, 0, 0])
    return np.column_stack((roll, pitch, yaw))


def box_sdf(points: np.ndarray, centers: np.ndarray, sizes: np.ndarray) -> np.ndarray:
    """Signed Euclidean distance to axis-aligned solid boxes."""
    q = np.abs(points - centers) - 0.5 * sizes
    return np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(
        np.max(q, axis=1), 0.0)


def load_bag_truth(path: Path) -> Tuple[pd.DataFrame, pd.DataFrame]:
    odom, dynamic = [], []
    topics = ["/coni_mpc/quad_odom0",
              "/quad0_pcl_render_node/dynamic_obstacle_states"]
    with rosbag.Bag(str(path)) as bag:
        for topic, msg, _ in bag.read_messages(topics=topics):
            stamp = msg.header.stamp.to_sec()
            if topic == topics[0]:
                p, v = msg.pose.pose.position, msg.twist.twist.linear
                odom.append((stamp, p.x, p.y, p.z, v.x, v.y, v.z))
            else:
                if msg.header.frame_id not in ("map", "world"):
                    raise ValueError(f"dynamic box frame is {msg.header.frame_id!r}")
                for obs in msg.obstacles:
                    p, v, s = obs.position, obs.velocity, obs.size
                    dynamic.append((stamp, int(obs.obstacle_id), str(obs.geometry_type),
                                    p.x, p.y, p.z, v.x, v.y, v.z,
                                    s.x, s.y, s.z, msg.header.frame_id))
    truth = pd.DataFrame(odom, columns=["stamp", "px", "py", "pz", "vx", "vy", "vz"])
    dyn = pd.DataFrame(dynamic, columns=["stamp", "id", "geometry_type", "x", "y", "z",
                                             "vx", "vy", "vz", "sx", "sy", "sz", "frame"])
    return truth, dyn


def align_dynamic_boxes(truth: pd.DataFrame, dyn: pd.DataFrame, radius: float,
                        max_extrapolation: float = 0.2) -> Tuple[pd.DataFrame, Dict[str, object]]:
    result = pd.DataFrame(index=truth.index)
    audit: Dict[str, object] = {"available": False, "reason": None, "obstacles": {}}
    if dyn.empty:
        audit["reason"] = "dynamic_obstacle_states topic has no obstacle rows"
        result["dynamic_true_clearance_m"] = np.nan
        return result, audit
    if not dyn.geometry_type.eq("box").all():
        raise ValueError("only recorded geometry_type=box is supported")
    all_clearance, all_zoh_clearance = [], []
    stamps_q = truth.stamp.to_numpy()
    points = truth[["px", "py", "pz"]].to_numpy()
    for obstacle_id, group in dyn.groupby("id", sort=True):
        group = group.sort_values("stamp").drop_duplicates("stamp", keep="last").reset_index(drop=True)
        stamps = group.stamp.to_numpy()
        idx = np.searchsorted(stamps, stamps_q, side="right") - 1
        covered = idx >= 0
        clipped = np.clip(idx, 0, len(group) - 1)
        selected = group.iloc[clipped]
        dt = stamps_q - selected.stamp.to_numpy()
        covered &= dt >= -1e-12
        covered &= dt <= max_extrapolation + 1e-12
        centers = selected[["x", "y", "z"]].to_numpy() + selected[["vx", "vy", "vz"]].to_numpy() * dt[:, None]
        held_centers = selected[["x", "y", "z"]].to_numpy()
        sizes = selected[["sx", "sy", "sz"]].to_numpy()
        clearance = box_sdf(points, centers, sizes) - radius
        zoh_clearance = box_sdf(points, held_centers, sizes) - radius
        clearance[~covered] = np.nan
        zoh_clearance[~covered] = np.nan
        result[f"dynamic_box_{obstacle_id}_clearance_m"] = clearance
        result[f"dynamic_box_{obstacle_id}_zoh_clearance_m"] = zoh_clearance
        result[f"dynamic_box_{obstacle_id}_source_age_sec"] = np.where(covered, dt, np.nan)
        all_clearance.append(clearance)
        all_zoh_clearance.append(zoh_clearance)
        velocities = group[["vx", "vy", "vz"]].to_numpy()
        box_sizes = group[["sx", "sy", "sz"]].to_numpy()
        audit["obstacles"][str(obstacle_id)] = {
            "messages": int(len(group)), "coverage_cycles": int(covered.sum()),
            "coverage_fraction": float(covered.mean()),
            "source_age_sec": finite_stats(np.where(covered, dt, np.nan)),
            "max_age_times_speed_m": float(np.nanmax(np.where(
                covered, dt * np.linalg.norm(selected[["vx", "vy", "vz"]].to_numpy(), axis=1), np.nan))),
            "velocity_span_max": float(np.ptp(velocities, axis=0).max()),
            "size_span_max": float(np.ptp(box_sizes, axis=0).max()),
            "constant_velocity_supported": bool(np.ptp(velocities, axis=0).max() <= 1e-12),
            "constant_size_supported": bool(np.ptp(box_sizes, axis=0).max() <= 5e-6),
        }
        if not audit["obstacles"][str(obstacle_id)]["constant_velocity_supported"]:
            raise ValueError(f"dynamic obstacle {obstacle_id} is not constant velocity")
    result["dynamic_true_clearance_m"] = np.nanmin(np.vstack(all_clearance), axis=0)
    result["dynamic_zoh_clearance_m"] = np.nanmin(np.vstack(all_zoh_clearance), axis=0)
    audit["available"] = True
    audit["definition"] = "axis-aligned box SDF at odometry ROS stamp minus UAV radius"
    audit["alignment"] = "latest past recorded box state propagated by its recorded constant velocity"
    audit["timestamp_limit_sec"] = max_extrapolation
    audit["caveat"] = "message header is sampled immediately before the renderer position update; interpolation is recorded-state truth approximation"
    return result, audit


def ordered_decomposition(prov: pd.DataFrame) -> Tuple[pd.DataFrame, float]:
    R = rotations(prov)
    u = columns(prov, [f"mpc_cmd_v{x}" for x in "xyz"])
    vg = columns(prov, [f"ugv_v{x}_w" for x in "xyz"])
    w = columns(prov, [f"ugv_omega_{x}_n" for x in "xyz"])
    p = columns(prov, [f"p_rel_{x}_n" for x in "xyz"])
    before = columns(prov, [f"world_before_{x}" for x in "xyz"])
    after = columns(prov, [f"world_after_{x}" for x in "xyz"])
    pub = columns(prov, [f"published_v{x}" for x in "xyz"])
    raw_w = columns(prov, [f"raw_ugv_omega_{x}" for x in "xyz"])
    linear_noise = vg - columns(prov, [f"raw_ugv_v{x}" for x in "xyz"])
    angular_noise = w - raw_w
    c_rp = np.einsum("nij,nj->ni", R, np.cross(
        np.column_stack((angular_noise[:, :2], np.zeros(len(prov)))), p))
    c_yaw = np.einsum("nij,nj->ni", R, np.cross(
        np.column_stack((np.zeros((len(prov), 2)), angular_noise[:, 2])), p))
    c_height = np.einsum("nij,nj->ni", R, np.column_stack((
        angular_noise[:, 1] * p[:, 2], -angular_noise[:, 0] * p[:, 2],
        np.zeros(len(prov)))))
    rows: List[Dict[str, object]] = []
    closure_max = 0.0
    term_names = ["ugv_velocity", "mpc_output", "omega_change", "position_change",
                  "rotation_change", "limit_change", "postprocess_change"]
    for i in range(1, len(prov)):
        terms = [
            vg[i] - vg[i-1],
            R[i-1] @ (u[i] - u[i-1]),
            R[i-1] @ np.cross(w[i] - w[i-1], p[i-1]),
            R[i-1] @ np.cross(w[i], p[i] - p[i-1]),
            (R[i] - R[i-1]) @ (u[i] + np.cross(w[i], p[i])),
            (after[i] - before[i]) - (after[i-1] - before[i-1]),
            (pub[i] - after[i]) - (pub[i-1] - after[i-1]),
        ]
        target = pub[i] - pub[i-1]
        closure = float(np.max(np.abs(np.sum(terms, axis=0) - target)))
        closure_max = max(closure_max, closure)
        row: Dict[str, object] = {
            "cycle_id": int(prov.cycle_id.iloc[i]), "sim_time": float(prov.sim_time.iloc[i]),
            "command_delta_norm": float(np.linalg.norm(target)),
            "mpc_delta_norm": float(np.linalg.norm(u[i] - u[i-1])),
            "cross_delta_norm": float(np.linalg.norm(columns(prov.iloc[i-1:i+1], [f"omega_cross_p_{x}_n" for x in "xyz"])[1] -
                                                        columns(prov.iloc[i-1:i+1], [f"omega_cross_p_{x}_n" for x in "xyz"])[0])),
            "closure_inf": closure,
            "linear_noise_delta_norm": float(np.linalg.norm(linear_noise[i] - linear_noise[i-1])),
            "c_rp_delta_norm": float(np.linalg.norm(c_rp[i] - c_rp[i-1])),
            "c_yaw_delta_norm": float(np.linalg.norm(c_yaw[i] - c_yaw[i-1])),
            "c_height_delta_norm": float(np.linalg.norm(c_height[i] - c_height[i-1])),
        }
        for name, term in zip(term_names, terms):
            row[f"decomp_{name}_norm"] = float(np.linalg.norm(term))
            for j, axis in enumerate("xyz"):
                row[f"decomp_{name}_{axis}"] = float(term[j])
        for name, value in (("c_rp", c_rp[i]), ("c_yaw", c_yaw[i]),
                            ("c_height", c_height[i])):
            row[f"{name}_norm"] = float(np.linalg.norm(value))
            for j, axis in enumerate("xyz"):
                row[f"{name}_{axis}"] = float(value[j])
        rows.append(row)
    return pd.DataFrame(rows), closure_max


def qp_audit(path: Path, prov: pd.DataFrame, steps: pd.DataFrame,
             node_params: Mapping[str, object]) -> Tuple[pd.DataFrame, pd.DataFrame, Dict[str, object]]:
    capture = load_capture(path)
    if len(capture) != len(prov):
        raise ValueError(f"QP records {len(capture)} != provenance cycles {len(prov)}")
    capture_indices = np.asarray(capture["index"], dtype=np.uint64)
    provenance_cycles = prov.cycle_id.to_numpy(dtype=np.uint64)
    if not np.array_equal(capture_indices, provenance_cycles):
        mismatch = int(np.flatnonzero(capture_indices != provenance_cycles)[0])
        raise ValueError(
            f"QP record index mismatch at row {mismatch}: "
            f"capture={capture_indices[mismatch]} provenance={provenance_cycles[mismatch]}")
    capture_status = np.asarray(capture["status"], dtype=np.int64)
    provenance_status = prov.qp_status.to_numpy(dtype=np.int64)
    if not np.array_equal(capture_status, provenance_status):
        mismatch = int(np.flatnonzero(capture_status != provenance_status)[0])
        raise ValueError(
            f"QP status mismatch at cycle {provenance_cycles[mismatch]}: "
            f"capture={capture_status[mismatch]} provenance={provenance_status[mismatch]}")
    rows, horizon_rows = [], []
    all_omega_xy, stage0_omega_err, x0_pos_err = [], [], []
    binding_count = violation_count = 0
    lower_slacks, failed_lower_slacks = [], []
    successful_records = 0
    returned_residuals, failed_returned_residuals = [], []
    stage0_active_count = 0
    alpha1 = float(node_params["cbf"]["alpha1"])
    alpha2 = float(node_params["cbf"]["alpha2"])
    for i, rec in enumerate(capture):
        od = np.asarray(rec["od"]).reshape(NSTAGE, NOD)
        omega = od[:, 68:71]
        all_omega_xy.extend(np.abs(omega[:, :2]).ravel())
        expected_omega = prov.loc[i, ["omega_profile0_x", "omega_profile0_y", "omega_profile0_z"]].to_numpy(float)
        stage0_omega_err.append(float(np.max(np.abs(omega[0] - expected_omega))))
        expected_p = prov.loc[i, ["estimate_px_n", "estimate_py_n", "estimate_pz_n"]].to_numpy(float)
        x0_pos_err.append(float(np.max(np.abs(np.asarray(rec["x0"])[:3] - expected_p))))
        qp_success = int(rec["status"]) == 0 and int(prov.solve_ok.iloc[i]) == 1
        stage0_active = bool(od[0, 84] > 0.5)
        stage0_active_count += int(stage0_active)
        returned_residual = np.nan
        if stage0_active:
            inv_tau = od[0, [85, 85, 86]]
            if np.any(inv_tau <= 0.0):
                raise ValueError(f"invalid captured inverse tau at cycle {i}")
            tau = 1.0 / inv_tau
            kv = (alpha1 + alpha2) * tau - 1.0
            kp = alpha1 * alpha2 * tau
            returned_u = prov.loc[i, ["mpc_cmd_vx", "mpc_cmd_vy", "mpc_cmd_vz"]].to_numpy(float)
            x0 = np.asarray(rec["x0"])
            returned_slack = float(steps.solver_slack.iloc[i])
            if not np.isfinite(returned_slack):
                returned_slack = float(prov.acado_slack0.iloc[i])
            returned_residual = float(od[0, 80:83] @ (
                returned_u + kv * x0[3:6] + kp * x0[:3]) + returned_slack - od[0, 83])
            (returned_residuals if qp_success else failed_returned_residuals).append(returned_residual)
        A = np.asarray(rec["A"]).reshape(200, 140)
        solution = np.asarray(rec["solution"])
        ax = A @ solution
        dual = np.asarray(rec["dual"])[140:]
        successful_records += int(qp_success)
        per_cycle_bind = per_cycle_viol = 0
        per_cycle_min_slack = math.inf
        for stage in range(20):
            row_idx = FIELD_LOCAL_ROW + ROWS_PER_STAGE * stage
            lower = float(np.asarray(rec["lbA"])[row_idx])
            if not np.isfinite(lower) or abs(lower) >= 1e20:
                continue
            slack = float(ax[row_idx] - lower)
            (lower_slacks if qp_success else failed_lower_slacks).append(slack)
            per_cycle_min_slack = min(per_cycle_min_slack, slack)
            if qp_success and abs(float(dual[row_idx])) > 1e-8:
                binding_count += 1; per_cycle_bind += 1
            if slack < -1e-8:
                per_cycle_viol += 1
                if qp_success:
                    violation_count += 1
            horizon_rows.append({"cycle_id": int(prov.cycle_id.iloc[i]), "horizon_idx": stage,
                                 "qp_capture_status": int(rec["status"]),
                                 "qp_solution_valid_for_binding": int(qp_success),
                                 "qp_field_dual": float(dual[row_idx]),
                                 "qp_field_binding": int(qp_success and abs(float(dual[row_idx])) > 1e-8),
                                 "qp_field_lower_slack": slack,
                                 "qp_field_lower_violation": max(0.0, -slack)})
        rows.append({"cycle_id": int(prov.cycle_id.iloc[i]),
                     "qp_capture_status": int(rec["status"]),
                     "qp_solution_valid_for_binding": int(qp_success),
                     "field_stage0_od_active": int(stage0_active),
                     "returned_stage0_affine_residual": returned_residual,
                     "qp_field_binding_stages": per_cycle_bind,
                     "qp_field_violating_stages": per_cycle_viol,
                     "qp_field_min_lower_slack": per_cycle_min_slack if math.isfinite(per_cycle_min_slack) else np.nan})
    return pd.DataFrame(rows), pd.DataFrame(horizon_rows), {
        "records": int(len(capture)), "successful_records_for_binding": successful_records,
        "failed_records_excluded_from_binding": int(len(capture) - successful_records),
        "all_stage_omega_xy_abs_max": float(max(all_omega_xy)),
        "stage0_omega_vs_provenance_max_abs_error": float(max(stage0_omega_err)),
        "x0_position_vs_provenance_estimate_max_abs_error": float(max(x0_pos_err)),
        "stage0_field_od_active_cycles": stage0_active_count,
        "field_rows_audited": int(len(lower_slacks)), "field_binding_dual_threshold": 1e-8,
        "field_binding_rows": binding_count, "field_lower_violation_rows": violation_count,
        "field_lower_slack": finite_stats(lower_slacks),
        "returned_stage0_affine_residual_successful_qp": finite_stats(returned_residuals),
        "returned_stage0_affine_residual_failed_fallback": finite_stats(failed_returned_residuals),
        "returned_stage0_affine_residual_definition": "captured affine out9 evaluated at returned MPC xyz, captured x0, and returned solver slack; not nonlinear h or post-clipped physical safety",
        "failed_qp_partial_primal_field_lower_slack_diagnostic_only": finite_stats(failed_lower_slacks),
        "failed_qp_limitation": "partial primal/dual arrays from failed solves are excluded from binding claims",
        "status_minus_2_meaning": "qpOASES getSimpleStatus maps -2 to infeasibility",
        "field_row_definition": "condensed constraint row 9 + 10*stage, stages 0..19",
    }


def load_runtime(path: Path) -> Tuple[Mapping[str, object], Mapping[str, object]]:
    params = yaml.safe_load(path.read_text())
    node = params.get("num_sim_non_one_point_node", {})
    return params, node


def dynamic_config_audit(dyn: pd.DataFrame, params: Mapping[str, object]) -> Dict[str, object]:
    cfg = params.get("box_validation", {}).get("dynamic", {})
    if not cfg or dyn.empty:
        return {"available": False, "reason": "box_validation.dynamic runtime parameters unavailable"}
    velocity = np.asarray(cfg.get("velocity", []), dtype=float)
    size = np.asarray(cfg.get("size", []), dtype=float)
    initial = np.asarray(cfg.get("initial", []), dtype=float)
    result = {"available": True, "note": "initial position is checked only; clearance propagation starts from recorded messages"}
    checks = {}
    for obstacle_id, group in dyn.groupby("id", sort=True):
        idx = int(obstacle_id)
        first = group.sort_values("stamp").iloc[0]
        checks[str(idx)] = {
            "first_recorded_position_vs_config_max_abs_error": float(np.max(np.abs(
                first[["x", "y", "z"]].to_numpy(float) - initial[idx]))) if idx < len(initial) else None,
            "recorded_velocity_vs_config_max_abs_error": float(np.max(np.abs(
                group[["vx", "vy", "vz"]].to_numpy(float) - velocity[idx]))) if idx < len(velocity) else None,
            "recorded_size_vs_config_max_abs_error": float(np.max(np.abs(
                group[["sx", "sy", "sz"]].to_numpy(float) - size[idx]))) if idx < len(size) else None,
        }
    result["obstacles"] = checks
    return result


def scheduler_log_audit(path: Path) -> Dict[str, object]:
    text = path.read_text(errors="replace")
    warning_count = len(re.findall(r"MPC scheduler overrun:", text))
    matches = re.findall(r"MPC scheduler summary: cycles=(\d+) mean_cycle_ms=([\d.eE+-]+) max_cycle_ms=([\d.eE+-]+) overruns=(\d+) max_overrun_ms=([\d.eE+-]+)", text)
    exact = None
    if matches:
        c, mean, maximum, over, max_over = matches[-1]
        exact = {"cycles": int(c), "mean_cycle_ms": float(mean), "max_cycle_ms": float(maximum),
                 "overruns": int(over), "max_overrun_ms": float(max_over)}
    return {"throttled_warning_count_lower_bound": warning_count,
            "exact_scheduler_summary": exact,
            "warning_count_is_not_exact_overrun_count": True}


def analyze_run(label: str, root: Path, output: Path) -> Tuple[Dict[str, object], pd.DataFrame, pd.DataFrame]:
    required = ["world_command_provenance.csv", "metrics.csv", "metrics_steps.csv",
                "metrics_field_hocbf_steps.csv", "runtime_params.yaml", "sensors.bag",
                "acado_qp_capture.bin", "roslaunch.log"]
    missing = [name for name in required if not (root / name).is_file()]
    if missing:
        raise FileNotFoundError(f"{label}: missing {missing}")
    prov = pd.read_csv(root / "world_command_provenance.csv", float_precision="round_trip")
    steps = pd.read_csv(root / "metrics_steps.csv", float_precision="round_trip")
    field = pd.read_csv(root / "metrics_field_hocbf_steps.csv", float_precision="round_trip")
    if len(prov) != len(steps) or not np.array_equal(prov.cycle_id.to_numpy(), steps.step_idx.to_numpy()):
        raise ValueError(f"{label}: provenance and metrics cycle mismatch")
    if prov.shape[1] != 96:
        raise ValueError(f"{label}: expected 96 provenance columns, got {prov.shape[1]}")
    expected_cycles = np.arange(len(prov))
    if not np.array_equal(prov.cycle_id.to_numpy(), expected_cycles):
        raise ValueError(f"{label}: cycle IDs are not contiguous from zero")
    if not np.array_equal(np.unique(prov.uav_idx.to_numpy(dtype=int)), np.array([0])):
        raise ValueError(f"{label}: fixed reference validation requires the single recorded UAV index 0")
    nominal_reference = columns(
        steps, ["nominal_position_x", "nominal_position_y", "nominal_position_z"])
    nominal_reference_error = np.max(np.abs(nominal_reference - DEFAULT_REFERENCE_N))
    if nominal_reference_error > 1e-6:
        raise ValueError(
            f"{label}: DEFAULT_REFERENCE_N does not match metrics nominal position; "
            f"max error={nominal_reference_error:.17g}")
    truth, dyn = load_bag_truth(root / "sensors.bag")
    if len(truth) != len(prov):
        raise ValueError(f"{label}: odometry {len(truth)} != cycles {len(prov)}")
    stamp_error = np.abs(truth.stamp.to_numpy() - prov.state_stamp_sec.to_numpy())
    position_error = np.max(np.abs(truth[["px", "py", "pz"]].to_numpy() -
                                       columns(prov, [f"quad_p{x}_w" for x in "xyz"])), axis=1)
    if np.max(stamp_error) != 0.0 or np.max(position_error) != 0.0:
        raise ValueError(f"{label}: odometry/provenance is not exact same-cycle data")
    all_params, node_params = load_runtime(root / "runtime_params.yaml")
    tau_xy, tau_z = float(node_params["tau_v_xy"]), float(node_params["tau_v_z"])
    radius = float(node_params["uav_radius"])
    dynamic_aligned, dynamic_audit = align_dynamic_boxes(truth, dyn, radius)
    R = rotations(prov)
    angles = rotation_angles(R)
    measured_w = columns(prov, [f"ugv_omega_{x}_n" for x in "xyz"])
    raw_w = columns(prov, [f"raw_ugv_omega_{x}" for x in "xyz"])
    measured_v = columns(prov, [f"ugv_v{x}_w" for x in "xyz"])
    raw_v = columns(prov, [f"raw_ugv_v{x}" for x in "xyz"])
    angular_noise = measured_w - raw_w
    linear_noise = measured_v - raw_v
    command = columns(prov, [f"published_v{x}" for x in "xyz"])
    mpc = columns(prov, [f"mpc_cmd_v{x}" for x in "xyz"])
    cross = columns(prov, [f"omega_cross_p_{x}_n" for x in "xyz"])
    actual_v = truth[["vx", "vy", "vz"]].to_numpy()
    tau = np.array([tau_xy, tau_xy, tau_z])
    model_accel = (command - actual_v) / tau
    sim_dt = float(pd.read_csv(root / "metrics.csv", float_precision="round_trip").iloc[0].sim_control_dt_sec)
    sampled_accel = np.vstack((np.full(3, np.nan), np.diff(actual_v, axis=0) / sim_dt))
    predicted_next_v = command[:-1] + (actual_v[:-1] - command[:-1]) * np.exp(-sim_dt / tau)
    plant_step_residual = actual_v[1:] - predicted_next_v
    ugv_position = columns(prov, [f"ugv_p{x}_w" for x in "xyz"])
    reference = ugv_position + np.einsum("nij,j->ni", R, DEFAULT_REFERENCE_N)
    true_position = truth[["px", "py", "pz"]].to_numpy()
    true_tracking = np.linalg.norm(true_position - reference, axis=1)
    raw_relative_n = columns(steps, [f"rot_load_position_non_{x}" for x in "xyz"])
    independent_true_tracking = np.linalg.norm(raw_relative_n - DEFAULT_REFERENCE_N, axis=1)
    tracking_crosscheck = np.abs(independent_true_tracking - true_tracking)
    decomp, closure = ordered_decomposition(prov)
    qp_cycle, qp_horizon, qp_summary = qp_audit(
        root / "acado_qp_capture.bin", prov, steps, node_params)
    stage0 = field[field.horizon_idx.eq(0)].sort_values("step_idx")
    if len(stage0) != len(prov):
        raise ValueError(f"{label}: Field-HOCBF stage0 rows {len(stage0)} != cycles {len(prov)}")
    timing_path = root / "world_command_provenance.csv.timing.csv"
    log_timing = pd.read_csv(timing_path, float_precision="round_trip") if timing_path.exists() else pd.DataFrame()
    feedback_ms = (prov.feedback_end_mono - prov.feedback_start_mono) * 1000.0
    mpc_ms = (prov.mpc_end_mono_sec - prov.mpc_start_mono_sec) * 1000.0
    publish_delay_ms = (prov.publish_mono_sec - prov.mpc_end_mono_sec) * 1000.0
    field_active = field.field_active.eq(1)
    percycle = pd.DataFrame({
        "label": label, "cycle_id": prov.cycle_id, "sim_time": prov.sim_time,
        "ros_stamp": truth.stamp, "gt_px": truth.px, "gt_py": truth.py, "gt_pz": truth.pz,
        "gt_vx": truth.vx, "gt_vy": truth.vy, "gt_vz": truth.vz,
        "reference_x": reference[:, 0], "reference_y": reference[:, 1], "reference_z": reference[:, 2],
        "true_tracking_error_m": true_tracking, "estimated_tracking_error_m": steps.tracking_error,
        "roll_rad": angles[:, 0], "pitch_rad": angles[:, 1], "yaw_rad": angles[:, 2],
        "raw_wx": raw_w[:, 0], "raw_wy": raw_w[:, 1], "raw_wz": raw_w[:, 2],
        "measured_wx": measured_w[:, 0], "measured_wy": measured_w[:, 1], "measured_wz": measured_w[:, 2],
        "yaw_noise": angular_noise[:, 2],
        "linear_noise_x": linear_noise[:, 0], "linear_noise_y": linear_noise[:, 1], "linear_noise_z": linear_noise[:, 2],
        "command_x": command[:, 0], "command_y": command[:, 1], "command_z": command[:, 2],
        "model_accel_x": model_accel[:, 0], "model_accel_y": model_accel[:, 1], "model_accel_z": model_accel[:, 2],
        "sampled_accel_x": sampled_accel[:, 0], "sampled_accel_y": sampled_accel[:, 1], "sampled_accel_z": sampled_accel[:, 2],
        "static_true_clearance_m": np.nan,
        "field_stage0_h": stage0.field_h.to_numpy(), "field_stage0_residual": stage0.field_residual.to_numpy(),
        "field_stage0_slack": stage0.field_slack.to_numpy(), "field_stage0_active_profile": stage0.field_active.to_numpy(),
        "solve_ok": prov.solve_ok, "qp_status": prov.qp_status,
        "fallback_used": prov.fallback_used,
        "mpc_duration_ms": mpc_ms, "feedback_duration_ms": feedback_ms,
        "publish_after_mpc_ms": publish_delay_ms,
    })
    percycle = pd.concat([percycle, dynamic_aligned.reset_index(drop=True)], axis=1)
    percycle = percycle.merge(qp_cycle, on="cycle_id", how="left", validate="one_to_one")
    percycle["command_delta_norm"] = np.r_[np.nan, np.linalg.norm(np.diff(command, axis=0), axis=1)]
    percycle["mpc_delta_norm"] = np.r_[np.nan, np.linalg.norm(np.diff(mpc, axis=0), axis=1)]
    percycle["cross_delta_norm"] = np.r_[np.nan, np.linalg.norm(np.diff(cross, axis=0), axis=1)]
    percycle = percycle.merge(decomp, on=["cycle_id", "sim_time"], how="left", suffixes=("", "_decomp"))
    percycle.to_csv(output / f"{label}_per_cycle.csv", index=False)
    decomp.nlargest(5, "command_delta_norm").to_csv(output / f"{label}_top5_command_events.csv", index=False)
    failure_cycles = prov.loc[prov.solve_ok.eq(0), "cycle_id"].astype(int).tolist()
    failure_window_ids = sorted({i for c in failure_cycles for i in range(max(0, c-2), min(len(prov), c+3))})
    percycle[percycle.cycle_id.isin(failure_window_ids)].to_csv(
        output / f"{label}_solver_failure_windows.csv", index=False)
    field_selected = field[["step_idx", "horizon_idx", "field_valid", "field_active", "field_h",
                            "field_Ax", "field_Ay", "field_Az", "field_b", "field_b_profile",
                            "field_residual", "field_slack"]].rename(columns={"step_idx": "cycle_id"})
    field_selected.insert(0, "label", label)
    field_selected.merge(qp_horizon, on=["cycle_id", "horizon_idx"], how="left",
                         validate="many_to_one").to_csv(output / f"{label}_field_horizon.csv", index=False)
    yaw_noise = angular_noise[:, 2]
    summary: Dict[str, object] = {
        "label": label, "run_dir": str(root), "cycles": int(len(prov)),
        "sim_time": {"first": float(prov.sim_time.iloc[0]), "last": float(prov.sim_time.iloc[-1]),
                     "step_stats": finite_stats(np.diff(prov.sim_time))},
        "exact_truth_alignment": {"max_stamp_error_sec": float(stamp_error.max()),
                                  "max_position_error_m": float(position_error.max())},
        "command_delta": delta_stats(command), "returned_mpc_delta": delta_stats(mpc),
        "omega_cross_p_delta": delta_stats(cross),
        "rotation_abs_max_rad": dict(zip(["roll", "pitch", "yaw"], np.max(np.abs(angles), axis=0).tolist())),
        "raw_omega_xy_abs_max": float(np.max(np.abs(raw_w[:, :2]))),
        "measured_omega_xy_abs_max": float(np.max(np.abs(measured_w[:, :2]))),
        "yaw_noise": {**finite_stats(yaw_noise), "std": float(np.std(yaw_noise)),
                      "rms": float(np.sqrt(np.mean(yaw_noise**2))),
                      "nonzero_count_exact": int(np.count_nonzero(yaw_noise))},
        "linear_velocity_noise": {axis: finite_stats(linear_noise[:, i]) for i, axis in enumerate("xyz")},
        "actual_horizontal_speed_max_mps": float(np.linalg.norm(actual_v[:, :2], axis=1).max()),
        "plant_model_instantaneous_accel_norm": finite_stats(np.linalg.norm(model_accel, axis=1)),
        "sampled_velocity_difference_accel_norm": finite_stats(np.linalg.norm(sampled_accel, axis=1)),
        "plant_first_order_next_velocity_residual_norm": finite_stats(np.linalg.norm(plant_step_residual, axis=1)),
        "acceleration_definitions": {
            "plant_model": "(command_k - measured_true_velocity_k) / tau at control interval start t_k+",
            "sampled": f"(velocity_k - velocity_(k-1)) / {sim_dt} s logical simulation step",
            "warning": "command delta is not physical acceleration"},
        "true_tracking_error_m": {**finite_stats(true_tracking),
                                  "rms": float(np.sqrt(np.mean(true_tracking**2)))},
        "true_tracking_independent_relative_state_crosscheck_max_abs_error_m": float(tracking_crosscheck.max()),
        "estimated_noisy_tracking_error_m": {**finite_stats(steps.tracking_error),
                                             "rms": float(np.sqrt(np.mean(steps.tracking_error**2)))},
        "tracking_reference_definition": "UGV world position + R_WN @ [0, 1.5, 2]",
        "dynamic_geometry": {**dynamic_audit,
                             "clearance_m": finite_stats(dynamic_aligned.dynamic_true_clearance_m),
                             "zoh_clearance_m": finite_stats(dynamic_aligned.dynamic_zoh_clearance_m),
                             "cv_vs_zoh_clearance_difference_m": finite_stats(
                                 np.abs(dynamic_aligned.dynamic_true_clearance_m - dynamic_aligned.dynamic_zoh_clearance_m)),
                             "runtime_config_check": dynamic_config_audit(dyn, all_params)},
        "static_true_clearance": {"available": False, "value": None,
            "reason": "archive contains only sampled static_scene.pcd; no solid geometry metadata/generator is archived"},
        "collision_flag": int(pd.read_csv(root / "metrics.csv").iloc[0].collision),
        "collision_flag_limitation": "recorded monitor flag only; unavailable static solid truth prevents full independent collision-clearance validation",
        "ordered_decomposition": {
            "definition": "dvg + Ra*du + Ra*(dw x pa) + Ra*(wb x dp) + (Rb-Ra)*(ub + wb x pb) + dlimit + dpostprocess",
            "max_closure_inf": closure,
            "term_norm_stats": {c: finite_stats(decomp[c]) for c in decomp if c.startswith("decomp_") and c.endswith("_norm")},
            "noise_terms": {
                "c_rp": "R @ ([noise_wx, noise_wy, 0] x p)",
                "c_yaw": "R @ ([0, 0, noise_wz] x p)",
                "c_height": "R @ [noise_wy*pz, -noise_wx*pz, 0]",
                "delta_norm_stats": {c: finite_stats(decomp[c]) for c in
                    ["c_rp_delta_norm", "c_yaw_delta_norm", "c_height_delta_norm"]}}},
        "field_hocbf": {
            "rows": int(len(field)), "stage0_rows": int(len(stage0)),
            "profile_active_rows": int(field_active.sum()),
            "profile_active_definition": "field_active means enabled diagnostic profile, not QP binding",
            "all_h": finite_stats(field.field_h), "all_raw_residual": finite_stats(field.field_residual),
            "all_slack": finite_stats(field.field_slack),
            "stage0_h": finite_stats(stage0.field_h), "stage0_raw_residual": finite_stats(stage0.field_residual),
            "stage0_slack": finite_stats(stage0.field_slack)},
        "qp_capture": qp_summary,
        "solver": {"solve_failures": int(prov.solve_ok.eq(0).sum()),
                   "solve_failure_cycles": prov.loc[prov.solve_ok.eq(0), "cycle_id"].astype(int).tolist(),
                   "fallback_cycles": int(prov.fallback_used.eq(1).sum()),
                   "nonzero_slack_cycles": int(np.count_nonzero(prov.acado_slack0.to_numpy())),
                   "feedback_ms": finite_stats(feedback_ms), "mpc_start_to_end_ms": finite_stats(mpc_ms),
                   "publish_after_mpc_ms": finite_stats(publish_delay_ms),
                   "metrics_total_pipeline_ms": finite_stats(steps.total_pipeline_time_ms),
                   "metrics_pipeline_over_20ms": int((steps.total_pipeline_time_ms > 20.0).sum())},
        "logging": {"provenance_collection_ms": finite_stats(prov.collection_ms),
                    "timing_records": int(len(log_timing)),
                    "serialize_ms": finite_stats(log_timing.serialize_ms) if "serialize_ms" in log_timing else finite_stats([])},
        "scheduler": scheduler_log_audit(root / "roslaunch.log"),
        "runtime": {"tau_v_xy": tau_xy, "tau_v_z": tau_z, "uav_radius": radius,
                    "estimation_noise": node_params.get("estimation_noise")},
    }
    return summary, percycle, prov


def exact_noise_comparison(labels: Sequence[str], provs: Mapping[str, pd.DataFrame]) -> Dict[str, object]:
    if "after" not in provs or "before" not in provs:
        return {"available": False, "reason": "requires labels after and before"}
    a, b = provs["after"], provs["before"]
    if len(a) != len(b) or not np.array_equal(a.cycle_id.to_numpy(), b.cycle_id.to_numpy()):
        return {"available": False, "reason": "cycle sequences differ"}
    def noise(frame: pd.DataFrame, measured: Sequence[str], raw: Sequence[str]) -> np.ndarray:
        return columns(frame, measured) - columns(frame, raw)
    angular_a = noise(a, [f"ugv_omega_{x}_n" for x in "xyz"], [f"raw_ugv_omega_{x}" for x in "xyz"])
    angular_b = noise(b, [f"ugv_omega_{x}_n" for x in "xyz"], [f"raw_ugv_omega_{x}" for x in "xyz"])
    linear_a = noise(a, [f"ugv_v{x}_w" for x in "xyz"], [f"raw_ugv_v{x}" for x in "xyz"])
    linear_b = noise(b, [f"ugv_v{x}_w" for x in "xyz"], [f"raw_ugv_v{x}" for x in "xyz"])
    return {"available": True, "cycles": int(len(a)),
            "target_roll_pitch_noise": {
                "after_exactly_zero": bool(np.count_nonzero(angular_a[:, :2]) == 0),
                "before_abs_max": float(np.max(np.abs(angular_b[:, :2]))),
                "after_before_exactly_equal": bool(np.array_equal(angular_a[:, :2], angular_b[:, :2]))},
            "non_target_yaw_noise": {
                "exactly_equal": bool(np.array_equal(angular_a[:, 2], angular_b[:, 2])),
                "max_abs_difference": float(np.max(np.abs(angular_a[:, 2] - angular_b[:, 2])))},
            "non_target_linear_xyz_noise": {
                "exactly_equal": bool(np.array_equal(linear_a, linear_b)),
                "max_abs_difference": float(np.max(np.abs(linear_a - linear_b)))}}


def runtime_comparison(summaries: Sequence[Mapping[str, object]]) -> Dict[str, object]:
    values = {str(s["label"]): s["runtime"] for s in summaries}
    canonical = [json.dumps(v, sort_keys=True, separators=(",", ":")) for v in values.values()]
    return {"selected_parameters": values, "all_selected_values_equal": len(set(canonical)) <= 1,
            "excluded": "metrics/output paths, ROS node URI and other run-specific metadata"}


def make_plots(output: Path, summaries: Sequence[Mapping[str, object]], frames: Mapping[str, pd.DataFrame]) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(14, 12), constrained_layout=True)
    for summary in summaries:
        label = str(summary["label"]); f = frames[label]
        axes[0, 0].plot(f.sim_time, f.command_delta_norm, label=label, lw=.7)
        axes[0, 1].plot(f.sim_time, f.cross_delta_norm, label=label, lw=.7)
        axes[1, 0].plot(f.sim_time, f.true_tracking_error_m, label=label, lw=.7)
        axes[1, 1].plot(f.gt_px, f.gt_py, label=label, lw=.8)
        axes[2, 0].plot(f.sim_time, f.measured_wx, label=f"{label}:wx", lw=.6)
        axes[2, 0].plot(f.sim_time, f.measured_wy, label=f"{label}:wy", lw=.6)
        axes[2, 0].plot(f.sim_time, f.measured_wz, label=f"{label}:wz", lw=.6)
        axes[2, 1].plot(f.sim_time, f.c_height_delta_norm, label=label, lw=.7)
    labels = [("command delta", "m/s"), ("omega cross p delta", "m/s"),
              ("true tracking error", "m"), ("ground-truth XY trajectory", "m"),
              ("measured angular velocity", "rad/s"), ("height-lever contribution delta", "m/s")]
    for ax, (title, unit) in zip(axes.flat, labels):
        ax.set_title(title); ax.set_ylabel(unit); ax.grid(alpha=.2); ax.legend(fontsize=7)
    axes[1, 1].set_xlabel("world x (m)"); axes[1, 1].set_ylabel("world y (m)")
    for ax in [axes[0, 0], axes[0, 1], axes[1, 0], axes[2, 0], axes[2, 1]]:
        ax.set_xlabel("simulation time (s)")
    fig.savefig(output / "yaw_only_validation.png", dpi=160)
    plt.close(fig)


def parse_run(value: str) -> Tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("--run must be label=path")
    label, path = value.split("=", 1)
    if not label or not path:
        raise argparse.ArgumentTypeError("--run must be label=path")
    return label, Path(path).resolve()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", type=parse_run, required=True,
                        help="repeatable label=run_directory")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    labels = [label for label, _ in args.run]
    if len(labels) != len(set(labels)):
        raise ValueError("run labels must be unique")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    summaries, frames, provs = [], {}, {}
    for label, root in args.run:
        summary, frame, prov = analyze_run(label, root, output)
        summaries.append(summary); frames[label] = frame; provs[label] = prov
    comparison = exact_noise_comparison(labels, provs)
    comparison_rows = []
    for s in summaries:
        comparison_rows.append({
            "label": s["label"], "cycles": s["cycles"],
            "command_delta_mean": s["command_delta"]["mean"],
            "command_delta_p95": s["command_delta"]["p95"],
            "command_delta_max": s["command_delta"]["max"],
            "command_delta_gt_0p5": s["command_delta"]["count_gt_0p5"],
            "command_delta_gt_1p0": s["command_delta"]["count_gt_1p0"],
            "cross_delta_max": s["omega_cross_p_delta"]["max"],
            "true_tracking_rms": s["true_tracking_error_m"]["rms"],
            "horizontal_speed_max": s["actual_horizontal_speed_max_mps"],
            "model_accel_max": s["plant_model_instantaneous_accel_norm"]["max"],
            "sampled_accel_max": s["sampled_velocity_difference_accel_norm"]["max"],
            "solver_failures": s["solver"]["solve_failures"],
            "fallback_cycles": s["solver"]["fallback_cycles"],
            "pipeline_over_20ms": s["solver"]["metrics_pipeline_over_20ms"],
            "scheduler_overruns_exact": (s["scheduler"]["exact_scheduler_summary"] or {}).get("overruns"),
        })
    pd.DataFrame(comparison_rows).to_csv(output / "run_comparison.csv", index=False)
    combined = pd.concat([frames[label] for label in labels], ignore_index=True)
    combined.to_csv(output / "all_runs_per_cycle.csv", index=False)
    result = {"schema": 1, "runs": summaries,
              "after_before_noise_comparison": comparison,
              "runtime_parameter_comparison": runtime_comparison(summaries),
              "limitations": [
                  "Static continuous solid geometry is unavailable; no static true-clearance claim is made.",
                  "Dynamic clearance is a recorded-state constant-velocity box interpolation with stated source ages.",
                  "The collision flag is reported as a monitor output, not as independent proof of no collision.",
                  "Field profile activity and condensed-QP binding are reported separately."]}
    (output / "summary.json").write_text(json.dumps(result, indent=2, allow_nan=False))
    make_plots(output, summaries, frames)
    print(json.dumps({"runs": labels, "output_dir": str(output),
                      "after_before_noise_comparison": comparison}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
