#!/usr/bin/env python3
"""Generate deterministic analytic PCD maps used only by M2.5 validation."""

import argparse
import math
from pathlib import Path


def plane_x(x, ys, zs):
    return [(x, y, z, 1.0) for y in ys for z in zs]


def plane_y(y, xs, zs):
    return [(x, y, z, 1.0) for x in xs for z in zs]


def cylinder(cx, cy, radius, zs, angular_count=180):
    return [
        (cx + radius * math.cos(2.0 * math.pi * i / angular_count),
         cy + radius * math.sin(2.0 * math.pi * i / angular_count), z, 1.0)
        for i in range(angular_count) for z in zs
    ]


def grid(low, high, step):
    count = int(round((high - low) / step))
    return [low + i * step for i in range(count + 1)]


def write_pcd(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
        stream.write("VERSION 0.7\nFIELDS x y z intensity\n")
        stream.write("SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n")
        stream.write(f"WIDTH {len(points)}\nHEIGHT 1\n")
        stream.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        stream.write(f"POINTS {len(points)}\nDATA ascii\n")
        for point in points:
            stream.write("{:.6f} {:.6f} {:.6f} {:.1f}\n".format(*point))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    horizontal = grid(-4.0, 8.0, 0.08)
    vertical = grid(0.0, 4.0, 0.08)
    scenes = {
        "A1_plane.pcd": plane_x(5.0, horizontal, vertical),
        "A2_cylinder.pcd": cylinder(5.0, 0.0, 1.0, vertical),
        "A3_corner.pcd": (
            plane_x(5.0, grid(-2.0, 5.0, 0.08), vertical)
            + plane_y(5.0, grid(-2.0, 5.0, 0.08), vertical)
        ),
        "A4_switch.pcd": (
            cylinder(5.0, -1.5, 0.8, vertical)
            + cylinder(5.0, 1.5, 0.8, vertical)
        ),
    }
    for name, points in scenes.items():
        write_pcd(args.output / name, points)
        print(f"{name}: {len(points)} points")


if __name__ == "__main__":
    main()
