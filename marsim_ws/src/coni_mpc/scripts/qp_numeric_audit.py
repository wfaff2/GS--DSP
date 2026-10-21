#!/usr/bin/env python3
"""Independent qpOASES/ACADO QP audit for the fixed capture layout.

The script never calls the solver. It recomputes residuals from H, g, A,
bounds, primal and dual arrays recorded before/after QProblem_init.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
import struct
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np

NVAR, NCON = 140, 200
HEADER_BYTES = 32
ARRAY_FIELDS = [
    ("x", 147), ("u", 140), ("x0", 7), ("od", 1827), ("yref", 280),
    ("yN", 7), ("W", 3920), ("WN", 49), ("lbValues", 140),
    ("ubValues", 140), ("lbAValues", 200), ("ubAValues", 200),
    ("H", 19600), ("g", 140), ("A", 28000), ("lb", 140),
    ("ub", 140), ("lbA", 200), ("ubA", 200), ("solution", 140),
    ("dual", 340),
]


def record_dtype(ncon: int = NCON, nod_nodes: int = 1827) -> np.dtype:
    fields = [
        ("index", "<u8"), ("status", "<i4"), ("nwsr", "<i4"),
        ("objective", "<f8"), ("kkt", "<f8"),
    ]
    sizes = dict(ARRAY_FIELDS)
    sizes.update(od=nod_nodes, lbAValues=ncon, ubAValues=ncon,
                 A=ncon * NVAR, lbA=ncon, ubA=ncon, dual=NVAR + ncon)
    fields.extend((name, ("<f8", (sizes[name],))) for name, _ in ARRAY_FIELDS)
    return np.dtype(fields, align=False)


def load_capture(path: Path) -> np.ndarray:
    size = path.stat().st_size
    if size < HEADER_BYTES:
        raise ValueError("capture header is truncated")
    with path.open("rb") as stream:
        header = stream.read(HEADER_BYTES)
    magic, version, nv, nc, nx, nu, nod = struct.unpack("<Q6I", header)
    expected_magic = 0x3150514D4F434343
    if ((magic, version, nv, nx, nu) != (expected_magic, 1, NVAR, 147, 140)
            or (nc, nod) not in ((200, 1827), (240, 2079))):
        raise ValueError(f"unexpected capture header {magic:x} {version} {nv} {nc} {nx} {nu} {nod}")
    dtype = record_dtype(nc, nod)
    if (size - HEADER_BYTES) % dtype.itemsize:
        raise ValueError(f"capture size {size} is not aligned to {dtype.itemsize}")
    count = (size - HEADER_BYTES) // dtype.itemsize
    return np.memmap(path, mode="r", dtype=dtype, offset=HEADER_BYTES, shape=(count,))


def finite_bound(values: np.ndarray) -> np.ndarray:
    return np.isfinite(values) & (np.abs(values) < 1.0e20)


def qp_residuals(rec: np.void) -> Dict[str, float]:
    H = np.asarray(rec["H"]).reshape(NVAR, NVAR)
    g = np.asarray(rec["g"])
    A = np.asarray(rec["A"]).reshape(rec["lbA"].size, NVAR)
    lb, ub = np.asarray(rec["lb"]), np.asarray(rec["ub"])
    lbA, ubA = np.asarray(rec["lbA"]), np.asarray(rec["ubA"])
    x, y = np.asarray(rec["solution"]), np.asarray(rec["dual"])
    yv, yc = y[:NVAR], y[NVAR:]
    if not all(np.all(np.isfinite(rec[name])) for name, _ in ARRAY_FIELDS):
        raise ValueError(f"nonfinite record {rec['index']}")
    ax = A @ x
    station = H @ x + g - yv - A.T @ yc
    lower_v = finite_bound(lb)
    upper_v = finite_bound(ub)
    lower_c = finite_bound(lbA)
    upper_c = finite_bound(ubA)

    violations = []
    violations.extend((lb[lower_v] - x[lower_v]).tolist())
    violations.extend((x[upper_v] - ub[upper_v]).tolist())
    violations.extend((lbA[lower_c] - ax[lower_c]).tolist())
    violations.extend((ax[upper_c] - ubA[upper_c]).tolist())
    primal = max([0.0] + [float(v) for v in violations])

    comp = []
    for i in np.flatnonzero(lower_v & (yv > 1.0e-12)):
        comp.append((x[i] - lb[i]) * yv[i])
    for i in np.flatnonzero(upper_v & (yv < -1.0e-12)):
        comp.append((x[i] - ub[i]) * yv[i])
    for i in np.flatnonzero(lower_c & (yc > 1.0e-12)):
        comp.append((ax[i] - lbA[i]) * yc[i])
    for i in np.flatnonzero(upper_c & (yc < -1.0e-12)):
        comp.append((ax[i] - ubA[i]) * yc[i])
    comp_abs = max([0.0] + [abs(float(v)) for v in comp])

    sign = []
    for i, yi in enumerate(yv):
        if yi > 1.0e-12 and not lower_v[i]:
            sign.append(yi)
        if yi < -1.0e-12 and not upper_v[i]:
            sign.append(-yi)
    for i, yi in enumerate(yc):
        if yi > 1.0e-12 and not lower_c[i]:
            sign.append(yi)
        if yi < -1.0e-12 and not upper_c[i]:
            sign.append(-yi)
    dual_sign = max([0.0] + [float(v) for v in sign])

    station_inf = float(np.max(np.abs(station)))
    h_scale = max(1.0, float(np.max(np.abs(H @ x))), float(np.max(np.abs(g))),
                  float(np.max(np.abs(A.T @ yc))), float(np.max(np.abs(yv))))
    primal_relative = 0.0
    for z, lo, hi, lm, um in ((x, lb, ub, lower_v, upper_v),
                              (ax, lbA, ubA, lower_c, upper_c)):
        for signed, bound, mask in ((lo-z, lo, lm), (z-hi, hi, um)):
            scaled = np.maximum(0.0, signed[mask]) / np.maximum(
                1.0, np.maximum(np.abs(z[mask]), np.abs(bound[mask])))
            if scaled.size:
                primal_relative = max(primal_relative, float(np.max(scaled)))
    comp_scale = max(1.0, float(np.max(np.abs(x * yv))),
                     float(np.max(np.abs(ax * yc))))
    objective = float(0.5 * x @ H @ x + g @ x)
    # This is the exact generated ACADO getKKT expression, included to
    # identify the recorded scalar; it is not a QP KKT residual.
    generated = abs(float(g @ x))
    generated += float(np.sum(np.abs(lb * np.where(yv > 1e-12, yv, 0.0))))
    generated += float(np.sum(np.abs(ub * np.where(yv < -1e-12, yv, 0.0))))
    generated += float(np.sum(np.abs(lbA * np.where(yc > 1e-12, yc, 0.0))))
    generated += float(np.sum(np.abs(ubA * np.where(yc < -1e-12, yc, 0.0))))
    return {
        "index": int(rec["index"]),
        "status": int(rec["status"]),
        "nwsr": int(rec["nwsr"]),
        "objective_recorded": float(rec["objective"]),
        "objective_recomputed": objective,
        "objective_abs_error": abs(objective - float(rec["objective"])),
        "acado_kkt_recorded": float(rec["kkt"]),
        "acado_kkt_expression_recomputed": generated,
        "acado_kkt_abs_error": abs(generated - float(rec["kkt"])),
        "primal_inf": primal,
        "stationarity_inf": station_inf,
        "dual_sign_inf": dual_sign,
        "complementarity_inf": comp_abs,
        "primal_relative": primal_relative,
        "stationarity_relative": station_inf / h_scale,
        "dual_sign_relative": dual_sign / max(1.0, float(np.max(np.abs(y)))),
        "complementarity_relative": comp_abs / comp_scale,
        "max_abs_x": float(np.max(np.abs(x))),
        "max_abs_dual": float(np.max(np.abs(y))),
    }


def quantiles(rows: List[Dict[str, float]], key: str) -> Dict[str, float]:
    vals = np.asarray([row[key] for row in rows], dtype=float)
    return {"max": float(np.max(vals)), "p95": float(np.quantile(vals, 0.95)),
            "median": float(np.median(vals))}


def parse_rows(path: Path, steps: Iterable[int]) -> List[Dict[str, str]]:
    wanted = set(steps)
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    return [row for row in rows if not wanted or int(row["step"]) in wanted]


def row_ints(value: str) -> List[int]:
    if not value:
        return []
    return [int(v) for v in value.split(";") if v]


def boundary_audit(rows: List[Dict[str, str]], path_size: int = 10) -> Dict[str, object]:
    result = []
    for row in rows:
        changed = row_ints(row.get("changed_A_rows", ""))
        role_counts = {"acceleration": 0, "legacy_cbf": 0,
                       "safe_set": 0, "field_hocbf": 0, "world_velocity": 0, "other": 0}
        for idx in changed:
            within = idx % path_size
            role = ("acceleration" if within < 3 else
                    "legacy_cbf" if within in (3, 5, 7) else
                    "safe_set" if within in (4, 6, 8) else
                    "field_hocbf" if within == 9 else
                    "world_velocity" if within in (10, 11) else "other")
            role_counts[role] += 1
        result.append({
            "step": int(row["step"]), "variant": row["variant"],
            "dH": float(row.get("dH", "nan")),
            "dA": float(row.get("dA", "nan")),
            "dg": float(row.get("dg", "nan")),
            "dlb": float(row.get("dlb", "nan")),
            "dub": float(row.get("dub", "nan")),
            "dlbA": float(row.get("dlbA", "nan")),
            "dubA": float(row.get("dubA", "nan")),
            "changed_A_rows": changed,
            "changed_A_role_counts": role_counts,
            "changed_lbA_rows": row_ints(row.get("changed_lbA_rows", "")),
            "changed_ubA_rows": row_ints(row.get("changed_ubA_rows", "")),
            "valid": int(row["valid"]),
            "status": int(row["status"]),
            "nwsr": int(row["nwsr"]),
            "objective": float(row["objective"]),
            "kkt": float(row["kkt"]),
            "residual": float(row["residual"]),
            "delta_velocity_xyz_norm": float(np.linalg.norm([
                float(row["delta_u0_from_A"]), float(row["delta_u1_from_A"]),
                float(row["delta_u2_from_A"])])),
            "delta_yaw_rate": float(row["delta_u3_from_A"]),
            # Legacy four-component norm mixes m/s and rad/s; do not call it speed.
            "delta_u_norm": float(np.linalg.norm([
                float(row["delta_u0_from_A"]), float(row["delta_u1_from_A"]),
                float(row["delta_u2_from_A"]), float(row["delta_u3_from_A"])])),
        })
        result[-1]["changed_lbA_role_counts"] = dict(role_counts)
        result[-1]["changed_ubA_role_counts"] = dict(role_counts)
        for key, rows_key in (("changed_lbA_role_counts", "changed_lbA_rows"),
                              ("changed_ubA_role_counts", "changed_ubA_rows")):
            result[-1][key] = {name: 0 for name in role_counts}
            for idx in result[-1][rows_key]:
                within = idx % path_size
                role = ("acceleration" if within < 3 else
                        "legacy_cbf" if within in (3, 5, 7) else
                        "safe_set" if within in (4, 6, 8) else
                        "field_hocbf" if within == 9 else
                        "world_velocity" if within in (10, 11) else "other")
                result[-1][key][role] += 1
    return {"rows": result, "changed_row_encoding": f"condensed path row = stage*{path_size} + local row; local 9 is Field-HOCBF"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("capture", type=Path)
    ap.add_argument("--counterfactual", type=Path)
    ap.add_argument("--output-dir", type=Path, default=Path("."))
    ap.add_argument("--steps", nargs="*", type=int, default=[322, 330, 1314])
    ap.add_argument("--max-records", type=int, default=0)
    args = ap.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    records = load_capture(args.capture)
    if args.max_records:
        records = records[:args.max_records]
    rows = [qp_residuals(rec) for rec in records]
    summary = {
        "capture": str(args.capture), "records": len(rows),
        "max_residuals": {key: quantiles(rows, key) for key in (
            "primal_inf", "stationarity_inf", "dual_sign_inf", "complementarity_inf",
            "primal_relative", "stationarity_relative", "complementarity_relative")},
        "max_objective_abs_error": max(r["objective_abs_error"] for r in rows),
        "max_acado_kkt_expression_abs_error": max(r["acado_kkt_abs_error"] for r in rows),
        "max_recorded_acado_kkt": max(r["acado_kkt_recorded"] for r in rows),
        "exact_qp_kkt_definition": (
            "stationarity Hx+g-y_var-A^T y_con, primal bound violation, "
            "qpOASES sign-aware complementarity; dual signs follow qpOASES "
            "y>0 lower, y<0 upper; infinities ignored at |bound|>=1e20"),
        "acado_getKKT_definition": (
            "generated ACADO scalar = abs(g^T x) plus abs(bound*dual) terms "
            "selected by dual sign; it is not stationarity or a normalized QP KKT norm"),
    }
    with (args.output_dir / "qp_numeric_audit_all.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    event_rows = [r for r in rows if r["index"] in set(args.steps)]
    with (args.output_dir / "qp_numeric_audit_events.csv").open("w", newline="") as stream:
        fields = sorted(event_rows[0].keys()) if event_rows else []
        writer = csv.DictWriter(stream, fieldnames=fields)
        if fields:
            writer.writeheader(); writer.writerows(event_rows)
    if args.counterfactual:
        cf_rows = parse_rows(args.counterfactual, args.steps)
        summary["boundary_audit"] = boundary_audit(cf_rows, records[0]["lbA"].size // 20)
        with (args.output_dir / "qp_constraint_boundary_audit.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=[
                "step", "variant", "valid", "status", "nwsr", "objective", "kkt",
                "residual", "dH", "dA", "dg", "dlb", "dub", "dlbA", "dubA",
                "changed_A_rows", "changed_lbA_rows", "changed_ubA_rows",
                "delta_velocity_xyz_norm", "delta_yaw_rate", "delta_u_norm"])
            writer.writeheader()
            for row in summary["boundary_audit"]["rows"]:
                out = dict(row)
                for key in ("changed_A_rows", "changed_lbA_rows", "changed_ubA_rows"):
                    out[key] = ";".join(map(str, out[key]))
                out.pop("changed_A_role_counts", None)
                out.pop("changed_lbA_role_counts", None)
                out.pop("changed_ubA_role_counts", None)
                writer.writerow(out)
    with (args.output_dir / "qp_numeric_audit.json").open("w") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

