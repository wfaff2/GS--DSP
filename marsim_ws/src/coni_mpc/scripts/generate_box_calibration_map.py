#!/usr/bin/env python3
import argparse
from pathlib import Path

import yaml


def surface_points(center, size, resolution):
    counts = [max(1, int(round(axis / resolution))) for axis in size]
    axes = [[-axis / 2.0 + i * axis / count for i in range(count + 1)]
            for axis, count in zip(size, counts)]
    points = set()
    for x in axes[0]:
        for y in axes[1]:
            points.add((x, y, -size[2] / 2.0))
            points.add((x, y, size[2] / 2.0))
    for x in axes[0]:
        for z in axes[2]:
            points.add((x, -size[1] / 2.0, z))
            points.add((x, size[1] / 2.0, z))
    for y in axes[1]:
        for z in axes[2]:
            points.add((-size[0] / 2.0, y, z))
            points.add((size[0] / 2.0, y, z))
    return [(x + center[0], y + center[1], z + center[2], 1.0)
            for x, y, z in sorted(points)]


def bounds_points():
    points = []
    for x in (-6.0, 6.0):
        for y in (-6.0, 6.0):
            for z in (0.0, 3.0):
                for dx in (0.0, 0.1):
                    for dy in (0.0, 0.1):
                        points.append((x - dx if x > 0 else x + dx,
                                       y - dy if y > 0 else y + dy, z, 1.0))
    return points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    with open(args.config) as handle:
        config = yaml.safe_load(handle)["static_box"]
    points = bounds_points() + surface_points(config["center"], config["size"],
                                               float(config.get("surface_resolution", 0.05)))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n")
        stream.write("FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n")
        stream.write("COUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\n" % len(points))
        stream.write("VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n" % len(points))
        for point in points:
            stream.write("%.9f %.9f %.9f %.1f\n" % point)


if __name__ == "__main__":
    main()
