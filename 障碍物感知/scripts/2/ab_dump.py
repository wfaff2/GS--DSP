#!/usr/bin/env python3
import argparse
import csv
import math
import os
import pathlib
import sys


def _parse_kv_list(values, value_name):
    out = {}
    if not values:
        return out
    for item in values:
        if ":" not in item:
            raise ValueError(f"{value_name} must be in 'id:val' format, got '{item}'")
        key_str, val_str = item.split(":", 1)
        key = int(key_str.strip())
        out[key] = val_str.strip()
    return out


def _parse_offset_list(values):
    raw = _parse_kv_list(values, "offset")
    out = {}
    for uav_id, vec_str in raw.items():
        parts = [p.strip() for p in vec_str.split(",")]
        if len(parts) != 3:
            raise ValueError(f"offset for uav {uav_id} must be x,y,z")
        out[uav_id] = (float(parts[0]), float(parts[1]), float(parts[2]))
    return out


def _parse_quad_list(values):
    return _parse_kv_list(values, "quad-odom")


def _read_odom_csv(path):
    times = []
    pos = []
    quat = []
    idx = None
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            if line.startswith("%time"):
                header = line.strip().split(",")
                def col(name):
                    if name not in header:
                        raise RuntimeError(f"Missing column {name} in {path}")
                    return header.index(name)
                idx = {
                    "t": col("%time"),
                    "px": col("field.pose.pose.position.x"),
                    "py": col("field.pose.pose.position.y"),
                    "pz": col("field.pose.pose.position.z"),
                    "qx": col("field.pose.pose.orientation.x"),
                    "qy": col("field.pose.pose.orientation.y"),
                    "qz": col("field.pose.pose.orientation.z"),
                    "qw": col("field.pose.pose.orientation.w"),
                }
                continue
            if idx is None:
                continue
            if not (line[0].isdigit() or line[0] == "-"):
                continue
            parts = line.strip().split(",")
            if len(parts) <= max(idx.values()):
                continue
            try:
                t = int(parts[idx["t"]])
                px = float(parts[idx["px"]])
                py = float(parts[idx["py"]])
                pz = float(parts[idx["pz"]])
                qx = float(parts[idx["qx"]])
                qy = float(parts[idx["qy"]])
                qz = float(parts[idx["qz"]])
                qw = float(parts[idx["qw"]])
            except ValueError:
                continue
            times.append(t)
            pos.append((px, py, pz))
            quat.append((qw, qx, qy, qz))
    return times, pos, quat


def _quat_to_R(qw, qx, qy, qz):
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n <= 0.0:
        return (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return (
        (1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)),
        (2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)),
        (2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)),
    )


def _mat_vec(R, v):
    return (
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    )


def _mat_T(R):
    return (
        (R[0][0], R[1][0], R[2][0]),
        (R[0][1], R[1][1], R[2][1]),
        (R[0][2], R[1][2], R[2][2]),
    )


def _default_offset(uav_id, r, ref_h):
    if uav_id == 0:
        return (-r, 0.0, ref_h)
    if uav_id == 2:
        return (0.0, -r, ref_h)
    raise ValueError(f"No default offset for uav {uav_id}, please pass --offset {uav_id}:x,y,z")


def _write_ab_csv(out_path, frame_mode, dN, car_times, car_pos, car_quat, quad_csv):
    quad_times, quad_pos, _ = _read_odom_csv(quad_csv)
    if not quad_times:
        raise RuntimeError(f"No quad data in {quad_csv}")
    if not car_times:
        raise RuntimeError("No car odom data")

    car_idx = 0
    with open(out_path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_ns", "ax", "ay", "az", "bx", "by", "bz", "cx", "cy", "cz"])
        for t, p_uav in zip(quad_times, quad_pos):
            while car_idx + 1 < len(car_times) and car_times[car_idx + 1] <= t:
                car_idx += 1
            p_car = car_pos[car_idx]
            qw, qx, qy, qz = car_quat[car_idx]
            R_WN = _quat_to_R(qw, qx, qy, qz)
            if frame_mode == "inertial":
                p_ref = (
                    p_car[0] + R_WN[0][0] * dN[0] + R_WN[0][1] * dN[1] + R_WN[0][2] * dN[2],
                    p_car[1] + R_WN[1][0] * dN[0] + R_WN[1][1] * dN[1] + R_WN[1][2] * dN[2],
                    p_car[2] + R_WN[2][0] * dN[0] + R_WN[2][1] * dN[1] + R_WN[2][2] * dN[2],
                )
                ax, ay, az = p_uav
                bx, by, bz = p_ref
            else:
                R_NW = _mat_T(R_WN)
                rel = (
                    p_uav[0] - p_car[0],
                    p_uav[1] - p_car[1],
                    p_uav[2] - p_car[2],
                )
                ax, ay, az = _mat_vec(R_NW, rel)
                bx, by, bz = dN
            cx, cy, cz = ax - bx, ay - by, az - bz
            w.writerow([t, ax, ay, az, bx, by, bz, cx, cy, cz])


def parse_args():
    p = argparse.ArgumentParser(description="Dump A/B/C (A-B=C) from odom CSVs.")
    p.add_argument("--frame-mode", choices=["noninertial", "inertial"], required=True)
    p.add_argument("--car-odom", required=True, help="Path to car_odom.csv (rostopic echo -p).")
    p.add_argument(
        "--quad-odom",
        action="append",
        default=[],
        help="uav_id:path, e.g. 0:results/.../quad_odom0.csv (repeatable).",
    )
    p.add_argument("--out-dir", required=True, help="Output directory for AB_uav*.csv")
    p.add_argument("--r", type=float, default=1.0)
    p.add_argument("--start-z", type=float, default=2.0)
    p.add_argument(
        "--ref-height-coeff",
        type=float,
        default=0.5,
        help="reference_height_rel = coeff*(start_z - car_start_z) when --ref-height not set.",
    )
    p.add_argument("--ref-height", type=float, default=None, help="Override reference_height_rel directly.")
    p.add_argument(
        "--offset",
        action="append",
        default=[],
        help="uav_id:x,y,z to override default offset (repeatable).",
    )
    return p.parse_args()


def main():
    args = parse_args()
    quad_map = _parse_quad_list(args.quad_odom)
    if not quad_map:
        print("ERROR: at least one --quad-odom uav_id:path is required", file=sys.stderr)
        return 2

    offsets = _parse_offset_list(args.offset)
    car_times, car_pos, car_quat = _read_odom_csv(args.car_odom)
    if not car_times:
        print("ERROR: car_odom.csv has no data", file=sys.stderr)
        return 2
    car_start_z = car_pos[0][2]
    if args.ref_height is None:
        ref_h = args.ref_height_coeff * (args.start_z - car_start_z)
    else:
        ref_h = args.ref_height

    out_dir = pathlib.Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for uav_id_str, quad_csv in quad_map.items():
        uav_id = int(uav_id_str)
        quad_csv = os.fspath(quad_csv)
        if uav_id in offsets:
            dN = offsets[uav_id]
        else:
            dN = _default_offset(uav_id, args.r, ref_h)
        out_path = out_dir / f"AB_uav{uav_id}.csv"
        _write_ab_csv(out_path, args.frame_mode, dN, car_times, car_pos, car_quat, quad_csv)

    print(str(out_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
