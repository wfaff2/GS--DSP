#!/usr/bin/env python3
"""Independent stage-3D audit using an exact Euclidean p=4 superellipsoid.

The script reuses only the independent stage-3C reference module.  It never
imports ROS or the coni_mpc C++ libraries.  The p=4 set is

    S = {x : x_0^4+x_1^4+x_2^4 <= 3 a^4},  a=2*Delta.

Distances are computed from the KKT projection, not from a norm difference.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from stage3c_cube_support_experiment import (
    Particle,
    ball_distance,
    cube_distance,
    kernel,
    kernel_derivatives,
    node,
    original_risk,
    project,
    projected_risk,
    support,
)


def superellipsoid_value(x: np.ndarray, delta: float) -> float:
    a = 2.0 * delta
    return float(np.sum(np.asarray(x, dtype=float) ** 4) - 3.0 * a**4)


def coordinate_projection(lam: float, y: np.ndarray) -> np.ndarray:
    """Solve x + 4*lambda*x^3 = y exactly using the sinh cubic formula."""
    y = np.asarray(y, dtype=float)
    if lam <= 1e-14:
        return y.copy()
    root = math.sqrt(12.0 * lam)
    argument = 1.5 * root * y
    return (2.0 / root) * np.sinh(np.arcsinh(argument) / 3.0)


def super_projection(
    query_offset: Sequence[float],
    delta: float,
    iterations: int = 64,
) -> Tuple[float, np.ndarray, float, float]:
    """Return distance, projection, KKT lambda, and constraint residual."""
    y = np.asarray(query_offset, dtype=float)
    a = 2.0 * delta
    target = 3.0 * a**4
    if float(np.sum(y**4)) <= target:
        return 0.0, y.copy(), 0.0, float(np.sum(y**4) - target)

    lo = 0.0
    hi = 1.0
    while float(np.sum(coordinate_projection(hi, y) ** 4)) > target:
        hi *= 2.0
        if hi > 1e300:
            raise RuntimeError("failed to bracket p=4 projection multiplier")
    for _ in range(iterations):
        mid = 0.5 * (lo + hi)
        x = coordinate_projection(mid, y)
        if float(np.sum(x**4)) > target:
            lo = mid
        else:
            hi = mid
    lam = 0.5 * (lo + hi)
    x = coordinate_projection(lam, y)
    residual = float(np.sum(x**4) - target)
    return float(np.linalg.norm(y - x)), x, lam, residual


def super_distance_terms(
    query_offset: Sequence[float],
    delta: float,
    radius: float,
    width: float,
    iterations: int = 64,
) -> Tuple[float, np.ndarray, np.ndarray, Dict[str, float]]:
    """Return kappa(dist), its q-gradient/Hessian, and projection diagnostics."""
    y = np.asarray(query_offset, dtype=float)
    distance, x, lam, residual = super_projection(y, delta, iterations)
    value, first, second = kernel_derivatives(distance, radius, width)
    if distance <= radius or (first == 0.0 and second == 0.0):
        return value, np.zeros(3), np.zeros((3, 3)), {
            "distance": distance, "lambda": lam, "constraint_residual": residual,
            "projection_residual": float(np.linalg.norm(y - x - 4.0 * lam * x**3)),
        }

    # Outside the set, q=x+4*lambda*x^3.  Differentiate the KKT system.
    b = 4.0 * x**3
    A_diag = 1.0 + 12.0 * lam * x**2
    A_inv = 1.0 / A_diag
    denominator = float(np.sum((b**2) * A_inv))
    if denominator <= 0.0:
        raise RuntimeError("singular p=4 projection sensitivity")
    dx_dq = np.diag(A_inv) - np.outer(A_inv * b, A_inv * b) / denominator
    normal = b / np.linalg.norm(b)
    db_dq = np.diag(12.0 * x**2) @ dx_dq
    h_distance = (np.eye(3) - np.outer(normal, normal)) @ db_dq / np.linalg.norm(b)
    h_distance = 0.5 * (h_distance + h_distance.T)
    gradient = first * normal
    hessian = second * np.outer(normal, normal) + first * h_distance
    return value, gradient, hessian, {
        "distance": distance, "lambda": lam, "constraint_residual": residual,
        "projection_residual": float(np.linalg.norm(y - x - 4.0 * lam * x**3)),
    }


def super_risk(nodes: Dict[Tuple[int, int, int], float], query: np.ndarray,
               delta: float, radius: float, width: float) -> float:
    result = 0.0
    for index, mass in nodes.items():
        distance = super_projection(query - node(index, delta), delta)[0]
        result += mass * kernel(distance, radius, width)
    return result


def super_derivatives(nodes: Dict[Tuple[int, int, int], float], query: np.ndarray,
                      delta: float, radius: float, width: float) -> Tuple[float, np.ndarray, np.ndarray]:
    value = 0.0
    gradient = np.zeros(3)
    hessian = np.zeros((3, 3))
    for index, mass in nodes.items():
        terms = super_distance_terms(query - node(index, delta), delta, radius, width)
        value += mass * terms[0]
        gradient += mass * terms[1]
        hessian += mass * terms[2]
    return value, gradient, hessian


def super_distance_batch(offsets: np.ndarray, delta: float, iterations: int = 48) -> np.ndarray:
    """Vectorized value-only projection for the benchmark."""
    y = np.asarray(offsets, dtype=float)
    a = 2.0 * delta
    target = 3.0 * a**4
    inside = np.sum(y**4, axis=1) <= target
    result = np.zeros(y.shape[0])
    if np.all(inside):
        return result
    outside_y = y[~inside]
    lo = np.zeros(outside_y.shape[0])
    hi = np.ones(outside_y.shape[0])
    for _ in range(64):
        root_hi = np.sqrt(12.0 * hi)
        x_hi = (2.0 / root_hi[:, None]) * np.sinh(
            np.arcsinh(1.5 * root_hi[:, None] * outside_y) / 3.0
        )
        need = np.sum(x_hi**4, axis=1) > target
        if not np.any(need):
            break
        hi[need] *= 2.0
    for _ in range(iterations):
        mid = 0.5 * (lo + hi)
        root = np.sqrt(12.0 * mid)
        x = (2.0 / root[:, None]) * np.sinh(np.arcsinh(1.5 * root[:, None] * outside_y) / 3.0)
        need = np.sum(x**4, axis=1) > target
        lo[need] = mid[need]
        hi[~need] = mid[~need]
    lam = 0.5 * (lo + hi)
    root = np.sqrt(12.0 * lam)
    x = (2.0 / root[:, None]) * np.sinh(np.arcsinh(1.5 * root[:, None] * outside_y) / 3.0)
    result[~inside] = np.linalg.norm(outside_y - x, axis=1)
    return result


def containment_scan(delta: float = 1.0, seed: int = 20261011) -> Dict[str, float]:
    rng = np.random.default_rng(seed)
    a = 2.0 * delta
    rho = math.sqrt(3.0) * a
    cube_violation = 0.0
    ball_violation = 0.0
    for _ in range(10000):
        point = rng.uniform(-a, a, 3)
        cube_violation = max(cube_violation, superellipsoid_value(point, delta))
        point_s = rng.normal(0.0, 1.0, 3)
        scale = (rng.uniform(0.0, 1.0) * 3.0 * a**4 / np.sum(point_s**4)) ** 0.25
        point_s *= scale
        ball_violation = max(ball_violation, float(np.linalg.norm(point_s) - rho))
    return {"max_cube_F": cube_violation, "max_superball_minus_sphere": ball_violation,
            "a": a, "rho": rho}


def projection_scan(delta: float = 1.0, seed: int = 20261011) -> Dict[str, float]:
    rng = np.random.default_rng(seed)
    max_constraint = 0.0
    max_kkt = 0.0
    min_lambda = math.inf
    max_inside_violation = 0.0
    for _ in range(500):
        point = rng.uniform(-6.0, 6.0, 3)
        distance, x, lam, residual = super_projection(point, delta)
        if lam > 0.0:
            max_constraint = max(max_constraint, abs(residual))
            max_kkt = max(max_kkt, float(np.linalg.norm(point - x - 4.0 * lam * x**3)))
        else:
            max_inside_violation = max(max_inside_violation, max(residual, 0.0))
        if lam > 0.0:
            min_lambda = min(min_lambda, lam)
        if distance > 0.0:
            np.testing.assert_allclose(distance, np.linalg.norm(point - x), atol=2e-12)
    return {"max_constraint_residual": max_constraint, "max_kkt_residual": max_kkt,
            "max_inside_violation": max_inside_violation,
            "min_positive_lambda": min_lambda}


def convergence_scan(delta: float = 0.7, seed: int = 20261011) -> Dict[str, float]:
    """Check bisection convergence against a higher-iteration reference."""
    rng = np.random.default_rng(seed)
    points = rng.uniform(-6.0, 6.0, (100, 3))
    reference = np.array([super_projection(point, delta, iterations=96)[0] for point in points])
    result: Dict[str, float] = {}
    for iterations in (24, 32, 48, 64):
        values = np.array([super_projection(point, delta, iterations=iterations)[0] for point in points])
        result[f"max_distance_error_{iterations}"] = float(np.max(np.abs(values - reference)))
    return result


def inequality_scan(seed: int = 20261011, delta: float = 0.5) -> Dict[str, object]:
    rng = np.random.default_rng(seed)
    scenarios = {
        "single": [Particle(np.array([0.37, 0.23, -0.19]), 1.0)],
        "sparse": [Particle(rng.uniform(-4.0, 4.0, 3), float(rng.uniform(0.05, 1.0))) for _ in range(6)],
        "dense": [Particle(rng.normal(0.0, 0.7, 3), float(rng.uniform(0.05, 1.0))) for _ in range(40)],
    }
    result: Dict[str, object] = {}
    radius, width = 1.0, 0.5
    for name, particles in scenarios.items():
        nodes, _ = project(particles, delta)
        queries = [p.position.copy() for p in particles]
        queries += [p.position + rng.normal(0.0, 1.0, 3) for p in particles for _ in range(3)]
        low = []
        cube_low = []
        super_mid = []
        high = []
        residuals = []
        for query in queries:
            original = original_risk(particles, query, radius, width)
            cube = projected_risk(nodes, query, delta, radius, width, "cube")
            ball = projected_risk(nodes, query, delta, radius, width, "ball")
            sup = super_risk(nodes, query, delta, radius, width)
            low.append(sup - original)
            cube_low.append(cube - original)
            super_mid.append(sup - cube)
            high.append(ball - sup)
            for index in list(nodes)[: min(8, len(nodes))]:
                _, _, multiplier, residual = super_projection(query - node(index, delta), delta)
                if multiplier > 0.0:
                    residuals.append(residual)
        result[name] = {
            "particle_count": len(particles), "node_count": len(nodes),
            "min_super_minus_original": float(min(low)),
            "min_cube_minus_original": float(min(cube_low)),
            "min_super_minus_cube": float(min(super_mid)),
            "min_ball_minus_super": float(min(high)),
            "max_cube_minus_original": float(max(projected_risk(nodes, q, delta, radius, width, "cube") - original_risk(particles, q, radius, width) for q in queries)),
            "max_ball_minus_super": float(max(high)),
            "mean_super_minus_original": float(np.mean(low)),
            "mean_cube_minus_original": float(np.mean(cube_low)),
            "mean_super_minus_cube": float(np.mean(super_mid)),
            "mean_ball_minus_super": float(np.mean(high)),
            "max_sampled_projection_residual": float(max(abs(v) for v in residuals)),
        }
    return result


def parameter_scan(seed: int = 20261011) -> List[Dict[str, float]]:
    """Compare all three grid bounds over resolution and kernel-width choices."""
    rng = np.random.default_rng(seed)
    particles = [
        Particle(np.array([0.37, 0.23, -0.19]), 0.8),
        Particle(np.array([1.31, -0.44, 0.27]), 0.6),
        Particle(np.array([-1.18, 0.63, 0.52]), 0.4),
        *[Particle(rng.normal(0.0, 0.8, 3), float(rng.uniform(0.05, 0.8))) for _ in range(8)],
    ]
    queries = [p.position.copy() for p in particles[:3]]
    queries.extend(rng.uniform(-4.0, 4.0, 12))
    rows: List[Dict[str, float]] = []
    for delta in (0.25, 0.5, 1.0):
        nodes, _ = project(particles, delta)
        for width in (0.25, 0.5, 1.0):
            gaps_low = []
            gaps_cube_low = []
            gaps_mid = []
            gaps_high = []
            for query in queries:
                original = original_risk(particles, query, 1.0, width)
                cube = projected_risk(nodes, query, delta, 1.0, width, "cube")
                sup = super_risk(nodes, query, delta, 1.0, width)
                ball = projected_risk(nodes, query, delta, 1.0, width, "ball")
                gaps_low.append(sup - original)
                gaps_cube_low.append(cube - original)
                gaps_mid.append(sup - cube)
                gaps_high.append(ball - sup)
            rows.append({
                "delta": delta,
                "smoothing_width": width,
                "min_super_minus_original": float(min(gaps_low)),
                "min_cube_minus_original": float(min(gaps_cube_low)),
                "min_super_minus_cube": float(min(gaps_mid)),
                "min_ball_minus_super": float(min(gaps_high)),
                "max_ball_minus_super": float(max(gaps_high)),
            })
    return rows


def derivative_audit(delta: float = 1.0, radius: float = 0.5, width: float = 0.8) -> Dict[str, float]:
    center = np.zeros(3)
    query = np.array([2.8, 2.8, 0.0])
    value, gradient, hessian, diagnostic = super_distance_terms(query, delta, radius, width)
    h = 2e-5

    def value_at(point: np.ndarray) -> float:
        return super_distance_terms(point, delta, radius, width)[0]

    finite_gradient = np.zeros(3)
    finite_hessian = np.zeros((3, 3))
    for axis in range(3):
        offset = np.zeros(3)
        offset[axis] = h
        finite_gradient[axis] = (value_at(query + offset) - value_at(query - offset)) / (2.0 * h)
        finite_hessian[:, axis] = (
            super_distance_terms(query + offset, delta, radius, width)[1]
            - super_distance_terms(query - offset, delta, radius, width)[1]
        ) / (2.0 * h)
    return {
        "value": value,
        "gradient_error": float(np.linalg.norm(gradient - finite_gradient)),
        "hessian_error": float(np.linalg.norm(hessian - finite_hessian)),
        "constraint_residual": diagnostic["constraint_residual"],
        "projection_residual": diagnostic["projection_residual"],
    }


def boundary_counterexample(delta: float = 1.0) -> Dict[str, float]:
    particle = Particle(np.array([0.2, 0.3, 0.4]), 1.0)
    nodes, input_mass = project([particle], delta, dimensions=8, clip=True)
    query = particle.position
    original = original_risk([particle], query, 0.5, 0.5)
    sup = super_risk(nodes, query, delta, 0.5, 0.5)
    return {"input_mass": input_mass, "clipped_mass": sum(nodes.values()),
            "original": original, "super": sup, "super_minus_original": sup - original}


def benchmark(count: int, horizon: int = 21, delta: float = 0.5) -> Dict[str, float]:
    rng = np.random.default_rng(7000 + count)
    particles = [Particle(rng.uniform(-4.0, 4.0, 3), float(rng.uniform(0.05, 1.5))) for _ in range(count)]
    query = np.array([0.2, -0.3, 0.4])
    start = time.perf_counter()
    sink = 0.0
    max_nodes = 0
    for stage in range(horizon):
        shifted = [Particle(p.position + 0.03 * stage * p.position / max(np.linalg.norm(p.position), 1.0), p.weight)
                   for p in particles]
        nodes, _ = project(shifted, delta)
        max_nodes = max(max_nodes, len(nodes))
        # The benchmark deliberately uses the exact scalar projection.  It is
        # a reference-cost measurement, not a real-time implementation claim.
        sink += super_risk(nodes, query, delta, 1.0, 0.5)
    elapsed_ms = 1e3 * (time.perf_counter() - start)
    return {"particles": count, "horizon": horizon, "max_nodes": max_nodes,
            "total_ms": elapsed_ms, "per_stage_ms": elapsed_ms / horizon, "sink": sink}


def report() -> Dict[str, object]:
    return {
        "containment": containment_scan(),
        "projection": projection_scan(),
        "convergence": convergence_scan(),
        "inequalities": inequality_scan(),
        "parameter_scan": parameter_scan(),
        "derivatives": derivative_audit(),
        "boundary_counterexample": boundary_counterexample(),
        "benchmarks": [benchmark(count) for count in (16, 64, 256)],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    print(json.dumps(report(), indent=2, default=lambda value: value.tolist() if isinstance(value, np.ndarray) else value))


if __name__ == "__main__":
    main()
