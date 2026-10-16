#!/usr/bin/env python3
"""Render the existing strategic figure-eight cylinders as a LiDAR PCD map.

Geometry mirrors generateFigureEightStrategicObstacles() and
resolveFigureEightAmplitude() in num_sim_non_one_point_node.cpp. Only the
physical map is generated; runtime obstacles still enter MPC through DSP.
"""

import argparse
import math
from pathlib import Path

import yaml

from generate_depth_cbf_m25_scenes import cylinder, grid, write_pcd


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=package / "parameters/num_sim_non_one_point.yaml")
    parser.add_argument("--output", type=Path,
                        default=package / "test_data/dsp_figure_eight/six_cylinders.pcd")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    geometry = config["figure_eight_geometry"]
    obstacles = config["figure_eight_obstacles"]
    if not geometry.get("fixed", True):
        parser.error("the static map requires figure_eight_geometry/fixed=true")
    amplitude = float(geometry.get("amplitude", 0.0))
    if amplitude <= 1e-6:
        speed = max(1e-6, abs(float(geometry.get("reference_v", 3.0))))
        yaw_rate = max(1e-6, abs(float(geometry.get("reference_w", 3.0))))
        amplitude = speed * 3.171202014425998 / (yaw_rate * 0.9704032544031163)
    radius = max(0.05, float(obstacles.get("radius", 0.40)))
    height = max(0.1, float(obstacles.get("height", 3.0)))
    x_turn = amplitude / math.sqrt(2.0)
    y_turn = max(0.05, float(obstacles.get("turn_y_ratio", 0.612))) * amplitude / 2.0
    y_crossing = max(0.05, float(obstacles.get("crossing_y_ratio", 1.10))) * amplitude / 2.0
    centers = [(sx * x_turn, sy * y_turn) for sx in (1, -1) for sy in (1, -1)]
    centers += [(0.0, y_crossing), (0.0, -y_crossing)]
    # Sample the cylinder sides at <= 5 cm; include both end caps.
    angular_count = max(16, math.ceil(2.0 * math.pi * radius / 0.05))
    zs = grid(0.0, height, height / math.ceil(height / 0.05))
    points = []
    for cx, cy in centers:
        points.extend(cylinder(cx, cy, radius, zs, angular_count))
        for radial_index in range(math.ceil(radius / 0.05)):
            cap_radius = radius * radial_index / math.ceil(radius / 0.05)
            points.extend(cylinder(cx, cy, cap_radius, [0.0, height], angular_count))
    write_pcd(args.output, points)
    print(f"amplitude={amplitude:.9f} m; radius={radius}; height={height}")
    print(f"centers={centers}")
    print(f"{args.output}: {len(points)} points")


if __name__ == "__main__":
    main()
