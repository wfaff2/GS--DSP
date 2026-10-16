#!/usr/bin/env python3
"""Independent C1 stage-3B moment/Hessian/risk audit.

This module deliberately does not import ROS or the coni_mpc C++ libraries.
It reimplements the cardinal cubic B-spline and the existing fifth-order
smootherstep collision kernel so that the moment identities and the proposed
Taylor correction can be checked independently of the controller stack.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np


Index = Tuple[int, int, int]


def cubic_weights(u: float) -> np.ndarray:
    """Weights for nodes floor(u)-1,...,floor(u)+2."""
    a = u - math.floor(u)
    return np.array(
        [
            (1.0 - a) ** 3 / 6.0,
            (3.0 * a**3 - 6.0 * a**2 + 4.0) / 6.0,
            (-3.0 * a**3 + 3.0 * a**2 + 3.0 * a + 1.0) / 6.0,
            a**3 / 6.0,
        ],
        dtype=float,
    )


def support(p: np.ndarray, delta: float) -> Iterable[Tuple[Index, float]]:
    """Return the 64 tensor-product weights on an infinite regular grid."""
    local = np.asarray(p, dtype=float) / float(delta)
    bases = [math.floor(float(v)) for v in local]
    one_d = [cubic_weights(float(v)) for v in local]
    for ix in range(4):
        for iy in range(4):
            for iz in range(4):
                weight = one_d[0][ix] * one_d[1][iy] * one_d[2][iz]
                if weight != 0.0:
                    yield (
                        (bases[0] + ix - 1, bases[1] + iy - 1, bases[2] + iz - 1),
                        float(weight),
                    )


def node(index: Index, delta: float) -> np.ndarray:
    return float(delta) * np.asarray(index, dtype=float)


def moments(p: Sequence[float], delta: float) -> Dict[str, float | np.ndarray]:
    """Evaluate partition, first-moment and squared-distance identities."""
    point = np.asarray(p, dtype=float)
    weights = list(support(point, delta))
    mass = sum(weight for _, weight in weights)
    first = sum((weight * node(index, delta) for index, weight in weights), np.zeros(3))
    second = sum(
        (weight * float(np.dot(node(index, delta) - point, node(index, delta) - point))
         for index, weight in weights),
        0.0,
    )
    return {
        "sum_N": float(mass),
        "first_error": first - point,
        "second_moment": float(second),
        "second_error": float(second - delta * delta),
        "node_count": len(weights),
    }


def truncated_moments(p: Sequence[float], delta: float, dimensions: int = 8) -> Dict[str, float | np.ndarray]:
    """The same moments after clipping a finite grid, to expose boundary loss."""
    point = np.asarray(p, dtype=float)
    weights = [(idx, w) for idx, w in support(point, delta)
               if all(0 <= axis < dimensions for axis in idx)]
    mass = sum(weight for _, weight in weights)
    first = sum((weight * node(index, delta) for index, weight in weights), np.zeros(3))
    second = sum(
        (weight * float(np.dot(node(index, delta) - point, node(index, delta) - point))
         for index, weight in weights),
        0.0,
    )
    return {
        "sum_N": float(mass),
        "first_error": first - point,
        "second_moment": float(second),
        "second_error": float(second - delta * delta),
        "node_count": len(weights),
    }


def smootherstep(x: float) -> float:
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    return x**3 * (6.0 * x**2 - 15.0 * x + 10.0)


def phi(distance: float, radius: float, width: float) -> float:
    if distance <= radius:
        return 1.0
    if distance >= radius + width:
        return 0.0
    return smootherstep((radius + width - distance) / width)


def hessian_norm(distance: float, radius: float, width: float) -> float:
    """Exact spectral norm of the radial Hessian away from the plateau."""
    if distance <= radius or distance >= radius + width:
        return 0.0
    x = (radius + width - distance) / width
    gp = 30.0 * x * x * (1.0 - x) * (1.0 - x)
    gpp = 60.0 * x * (1.0 - x) * (1.0 - 2.0 * x)
    radial = abs(gpp) / (width * width)
    tangential = gp / (width * distance)
    return max(radial, tangential)


def hessian_bound(radius: float, width: float) -> float:
    """Global L2 bound for ||nabla^2 phi(||q-p||)||_2."""
    if radius <= 0.0 or width <= 0.0:
        raise ValueError("radius and width must be positive")
    return max(10.0 / (math.sqrt(3.0) * width * width), 15.0 / (8.0 * radius * width))


@dataclass(frozen=True)
class Particle:
    position: np.ndarray
    weight: float


def projected_nodes(particles: Sequence[Particle], delta: float) -> Dict[Index, float]:
    values: Dict[Index, float] = {}
    for particle in particles:
        for index, weight in support(particle.position, delta):
            values[index] = values.get(index, 0.0) + particle.weight * weight
    return values


def node_second_moments(particles: Sequence[Particle], delta: float) -> Dict[Index, float]:
    values: Dict[Index, float] = {}
    for particle in particles:
        for index, weight in support(particle.position, delta):
            error = particle.position - node(index, delta)
            values[index] = values.get(index, 0.0) + particle.weight * weight * float(np.dot(error, error))
    return values


def smooth_risk(particles: Sequence[Particle], query: np.ndarray, radius: float, width: float) -> float:
    return sum(p.weight * phi(float(np.linalg.norm(query - p.position)), radius, width) for p in particles)


def grid_risk(nodes: Dict[Index, float], query: np.ndarray, delta: float, radius: float, width: float) -> float:
    return sum(mass * phi(float(np.linalg.norm(query - node(index, delta))), radius, width)
               for index, mass in nodes.items())


def candidate_minus(particles: Sequence[Particle], query: np.ndarray, delta: float,
                    radius: float, width: float, chi: float = 1.0) -> Tuple[float, float, float]:
    nodes = projected_nodes(particles, delta)
    moments_j = node_second_moments(particles, delta)
    base = grid_risk(nodes, query, delta, radius, width)
    correction = 0.5 * hessian_bound(radius, width) * chi * sum(moments_j.values())
    return base - correction, base, correction


def find_counterexample(delta: float = 1.0, radius: float = 1.0, width: float = 0.5) -> Dict[str, float | List[float]]:
    """Find a positive Taylor interpolation error for the requested minus sign."""
    # A single particle is enough; the first-moment cancellation is still exact
    # after summing its 64 tensor-product support nodes.
    particle = Particle(np.array([0.37, 0.23, -0.19]), 1.0)
    best = None
    for qx in np.linspace(-3.0, 3.0, 1201):
        query = np.array([qx, 0.23, -0.19])
        exact = smooth_risk([particle], query, radius, width)
        candidate, base, correction = candidate_minus([particle], query, delta, radius, width)
        gap = exact - candidate
        if best is None or gap > best["gap"]:
            best = {"query_x": float(qx), "exact": exact, "grid0": base,
                    "correction": correction, "candidate_minus": candidate,
                    "gap": gap}
    assert best is not None
    return best


def find_multi_particle_counterexample(delta: float = 1.0, radius: float = 1.0,
                                       width: float = 0.5) -> Dict[str, float]:
    """Repeat the sign check with four distinct weighted particles."""
    particles = [
        Particle(np.array([0.37, 0.23, -0.19]), 0.4),
        Particle(np.array([0.41, 0.19, -0.17]), 0.3),
        Particle(np.array([0.28, 0.26, -0.22]), 0.5),
        Particle(np.array([0.52, 0.16, -0.11]), 0.2),
    ]
    best = None
    for qx in np.linspace(-3.0, 3.0, 1201):
        query = np.array([qx, 0.23, -0.19])
        exact = smooth_risk(particles, query, radius, width)
        candidate, base, correction = candidate_minus(particles, query, delta, radius, width)
        gap = exact - candidate
        if best is None or gap > best["gap"]:
            best = {"query_x": float(qx), "exact": exact, "grid0": base,
                    "correction": correction, "candidate_minus": candidate,
                    "gap": gap}
    assert best is not None
    return best


def benchmark(particle_count: int, steps: int, delta: float, radius: float, width: float) -> float:
    rng = np.random.default_rng(20261009 + particle_count)
    particles = [Particle(rng.uniform(-8.0, 8.0, size=3), float(rng.uniform(0.05, 1.5)))
                 for _ in range(particle_count)]
    queries = [rng.uniform(-4.0, 4.0, size=3) for _ in range(steps)]
    start = time.perf_counter()
    sink = 0.0
    for query in queries:
        nodes = projected_nodes(particles, delta)
        moments_j = node_second_moments(particles, delta)
        sink += grid_risk(nodes, query, delta, radius, width)
        sink += 0.5 * hessian_bound(radius, width) * sum(moments_j.values())
    elapsed = time.perf_counter() - start
    # Keep the result live in optimized/interpreter runs.
    if not math.isfinite(sink):
        raise RuntimeError("benchmark became non-finite")
    return 1e3 * elapsed / steps


def report() -> Dict[str, object]:
    delta_values = [1.0, 0.5, 0.25]
    moment_rows = []
    for delta in delta_values:
        for p in ([0.37, 0.23, -0.19], [0.5, 0.5, 0.5], [2.01, 3.27, 4.49]):
            row = moments(np.asarray(p), delta)
            row["delta"] = delta
            row["p"] = list(p)
            moment_rows.append(row)
    boundary = truncated_moments([0.2, 0.3, 0.4], 1.0)
    radius = 1.0
    width = 0.5
    scan = np.linspace(radius, radius + width, 100001)[1:-1]
    scanned = max(hessian_norm(float(d), radius, width) for d in scan)
    analytic = hessian_bound(radius, width)
    return {
        "moments": moment_rows,
        "boundary_truncation": boundary,
        "hessian": {"radius": radius, "width": width, "analytic_L2": analytic,
                     "dense_scan_max": scanned, "scan_gap": analytic - scanned},
        "counterexample": find_counterexample(),
        "four_particle_counterexample": find_multi_particle_counterexample(),
        "benchmarks_ms_per_step": {str(n): benchmark(n, 20, 0.5, radius, width)
                                    for n in (256, 1024, 2048)},
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    args = parser.parse_args()
    result = report()
    if args.json:
        print(json.dumps(result, indent=2, default=lambda value: value.tolist()
                         if isinstance(value, np.ndarray) else value))
        return
    print(json.dumps(result, indent=2, default=lambda value: value.tolist()
                     if isinstance(value, np.ndarray) else value))


if __name__ == "__main__":
    main()
