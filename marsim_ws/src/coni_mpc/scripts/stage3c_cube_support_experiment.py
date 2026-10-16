#!/usr/bin/env python3
"""Independent stage-3C audit for a cubic B-spline support-domain bound.

The module does not import ROS or the coni_mpc C++ libraries.  It reproduces
the active cubic P2G support and fifth-order smootherstep kernel, then tests
the candidate

  R_cube(q) = sum_j M_j kappa(dist(q, c_j + [-2*Delta,2*Delta]^3)).

The candidate is intentionally kept out of the online C++ path.  This script
is a mathematical/reference experiment only.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


Index = Tuple[int, int, int]


@dataclass(frozen=True)
class Particle:
    position: np.ndarray
    weight: float


def cubic_weights(u: float) -> np.ndarray:
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
    """64 tensor-product nodes with a nonzero cubic cardinal weight."""
    local = np.asarray(p, dtype=float) / float(delta)
    bases = [math.floor(float(v)) for v in local]
    weights = [cubic_weights(float(v)) for v in local]
    for ix in range(4):
        for iy in range(4):
            for iz in range(4):
                value = weights[0][ix] * weights[1][iy] * weights[2][iz]
                if value != 0.0:
                    yield (bases[0] + ix - 1, bases[1] + iy - 1, bases[2] + iz - 1), float(value)


def node(index: Index, delta: float) -> np.ndarray:
    return float(delta) * np.asarray(index, dtype=float)


def project(
    particles: Sequence[Particle],
    delta: float,
    dimensions: Optional[int] = None,
    clip: bool = False,
) -> Tuple[Dict[Index, float], float]:
    """Project particle mass; clip=True intentionally models an invalid crop."""
    values: Dict[Index, float] = {}
    input_mass = sum(float(p.weight) for p in particles)
    for particle in particles:
        for index, weight in support(particle.position, delta):
            if dimensions is not None and not all(0 <= a < dimensions for a in index):
                if clip:
                    continue
                raise ValueError("positive-weight cubic support leaves finite grid")
            values[index] = values.get(index, 0.0) + particle.weight * weight
    return values, input_mass


def smootherstep(x: float) -> float:
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    return x**3 * (6.0 * x**2 - 15.0 * x + 10.0)


def kernel(distance: float, radius: float, width: float) -> float:
    if distance <= radius:
        return 1.0
    if distance >= radius + width:
        return 0.0
    return smootherstep((radius + width - distance) / width)


def kernel_derivatives(distance: float, radius: float, width: float) -> Tuple[float, float, float]:
    """Return kappa, kappa', kappa'' with respect to scalar distance."""
    if distance <= radius or distance >= radius + width:
        return (1.0 if distance <= radius else 0.0, 0.0, 0.0)
    x = (radius + width - distance) / width
    gp = 30.0 * x * x * (1.0 - x) * (1.0 - x)
    gpp = 60.0 * x * (1.0 - x) * (1.0 - 2.0 * x)
    return (smootherstep(x), -gp / width, gpp / (width * width))


def cube_distance(query: np.ndarray, center: np.ndarray, half_width: float) -> Tuple[float, np.ndarray, np.ndarray]:
    """Distance to an axis-aligned cube plus its local derivatives.

    Derivatives are the piecewise formulas away from active-set/sign
    boundaries.  They are still well-defined for the value at all points.
    """
    offset = np.asarray(query, dtype=float) - np.asarray(center, dtype=float)
    excess = np.maximum(np.abs(offset) - half_width, 0.0)
    distance = float(np.linalg.norm(excess))
    if distance == 0.0:
        return distance, np.zeros(3), np.zeros((3, 3))
    active = excess > 0.0
    sign = np.sign(offset)
    grad_distance = np.zeros(3)
    grad_distance[active] = sign[active] * excess[active] / distance
    h_distance = np.zeros((3, 3))
    h_distance[np.diag_indices(3)] = active.astype(float) / distance
    h_distance -= np.outer(grad_distance, grad_distance) / distance
    return distance, grad_distance, h_distance


def ball_distance(query: np.ndarray, center: np.ndarray, radius: float) -> float:
    return max(float(np.linalg.norm(np.asarray(query) - np.asarray(center))) - radius, 0.0)


def cube_terms(query: np.ndarray, center: np.ndarray, delta: float, radius: float, width: float) -> Tuple[float, np.ndarray, np.ndarray]:
    distance, grad_distance, h_distance = cube_distance(query, center, 2.0 * delta)
    value, first, second = kernel_derivatives(distance, radius, width)
    gradient = first * grad_distance
    hessian = second * np.outer(grad_distance, grad_distance) + first * h_distance
    return value, gradient, hessian


def ball_terms(query: np.ndarray, center: np.ndarray, delta: float, radius: float, width: float) -> Tuple[float, np.ndarray, np.ndarray]:
    rho = 2.0 * math.sqrt(3.0) * delta
    displacement = np.asarray(query, dtype=float) - np.asarray(center, dtype=float)
    norm = float(np.linalg.norm(displacement))
    distance = max(norm - rho, 0.0)
    value, first, second = kernel_derivatives(distance, radius, width)
    if norm <= rho or first == 0.0 and second == 0.0:
        return value, np.zeros(3), np.zeros((3, 3))
    unit = displacement / norm
    grad_distance = unit
    h_distance = (np.eye(3) - np.outer(unit, unit)) / norm
    return value, first * grad_distance, second * np.outer(grad_distance, grad_distance) + first * h_distance


def original_risk(particles: Sequence[Particle], query: np.ndarray, radius: float, width: float) -> float:
    return sum(p.weight * kernel(float(np.linalg.norm(query - p.position)), radius, width) for p in particles)


def hard_risk(particles: Sequence[Particle], query: np.ndarray, radius: float) -> float:
    return sum(p.weight for p in particles if float(np.linalg.norm(query - p.position)) <= radius)


def projected_risk(nodes: Dict[Index, float], query: np.ndarray, delta: float, radius: float, width: float, mode: str) -> float:
    result = 0.0
    for index, mass in nodes.items():
        center = node(index, delta)
        distance = cube_distance(query, center, 2.0 * delta)[0] if mode == "cube" else ball_distance(query, center, 2.0 * math.sqrt(3.0) * delta)
        result += mass * kernel(distance, radius, width)
    return result


def projected_derivatives(nodes: Dict[Index, float], query: np.ndarray, delta: float, radius: float, width: float, mode: str) -> Tuple[float, np.ndarray, np.ndarray]:
    value = 0.0
    gradient = np.zeros(3)
    hessian = np.zeros((3, 3))
    for index, mass in nodes.items():
        terms = cube_terms(query, node(index, delta), delta, radius, width) if mode == "cube" else ball_terms(query, node(index, delta), delta, radius, width)
        value += mass * terms[0]
        gradient += mass * terms[1]
        hessian += mass * terms[2]
    return value, gradient, hessian


def all_nodes_in_support(particles: Sequence[Particle], delta: float) -> int:
    return len(project(particles, delta)[0])


def inequality_scan(seed: int = 20261011, count: int = 80, delta: float = 0.5) -> Dict[str, object]:
    rng = np.random.default_rng(seed)
    scenarios = {
        "single": [Particle(np.array([0.37, 0.23, -0.19]), 1.0)],
        "sparse": [Particle(rng.uniform(-6.0, 6.0, 3), float(rng.uniform(0.05, 1.0))) for _ in range(5)],
        "dense": [Particle(rng.normal(0.0, 0.7, 3), float(rng.uniform(0.05, 1.0))) for _ in range(80)],
    }
    result: Dict[str, object] = {}
    radius, width = 1.0, 0.5
    for name, particles in scenarios.items():
        nodes, _ = project(particles, delta)
        min_cube_minus_original = math.inf
        min_ball_minus_cube = math.inf
        max_cube_minus_original = -math.inf
        max_ball_minus_cube = -math.inf
        max_cube_record = None
        max_ball_record = None
        sum_cube_minus_original = 0.0
        sum_ball_minus_cube = 0.0
        positive_queries = 0
        cube_active_counts = []
        ball_active_counts = []
        worst = None
        queries = [p.position.copy() for p in particles]
        for particle in particles[: min(16, len(particles))]:
            for _ in range(max(1, count // 16)):
                queries.append(particle.position + rng.normal(0.0, 1.0, 3))
        queries.extend(rng.uniform(-5.0, 5.0, 3) for _ in range(count))
        for query in queries:
            original = original_risk(particles, query, radius, width)
            cube = projected_risk(nodes, query, delta, radius, width, "cube")
            ball = projected_risk(nodes, query, delta, radius, width, "ball")
            cube_active_counts.append(sum(kernel(cube_distance(query, node(index, delta), 2.0 * delta)[0], radius, width) > 0.0
                                         for index in nodes))
            ball_active_counts.append(sum(kernel(ball_distance(query, node(index, delta), 2.0 * math.sqrt(3.0) * delta), radius, width) > 0.0
                                         for index in nodes))
            gap_low = cube - original
            gap_high = ball - cube
            if original > 1e-12 or cube > 1e-12:
                positive_queries += 1
            sum_cube_minus_original += gap_low
            sum_ball_minus_cube += gap_high
            if gap_low > max_cube_minus_original:
                max_cube_minus_original = gap_low
                max_cube_record = {"query": query.tolist(), "original": original,
                                   "cube": cube, "ball": ball}
            if gap_high > max_ball_minus_cube:
                max_ball_minus_cube = gap_high
                max_ball_record = {"query": query.tolist(), "original": original,
                                   "cube": cube, "ball": ball}
            if gap_low < min_cube_minus_original:
                min_cube_minus_original = gap_low
                worst = {"query": query.tolist(), "original": original, "cube": cube, "ball": ball}
            min_ball_minus_cube = min(min_ball_minus_cube, gap_high)
        result[name] = {"particle_count": len(particles), "node_count": len(nodes),
                        "min_cube_minus_original": min_cube_minus_original,
                        "min_ball_minus_cube": min_ball_minus_cube,
                        "max_cube_minus_original": max_cube_minus_original,
                        "max_ball_minus_cube": max_ball_minus_cube,
                        "mean_cube_minus_original": sum_cube_minus_original / len(queries),
                        "mean_ball_minus_cube": sum_ball_minus_cube / len(queries),
                        "positive_query_count": positive_queries,
                        "query_count": len(queries),
                        "mean_cube_active_nodes": float(np.mean(cube_active_counts)),
                        "max_cube_active_nodes": max(cube_active_counts),
                        "mean_ball_active_nodes": float(np.mean(ball_active_counts)),
                        "max_ball_active_nodes": max(ball_active_counts),
                        "max_cube_record": max_cube_record,
                        "max_ball_record": max_ball_record,
                        "worst": worst}
    return result


def find_transition_face_jump(delta: float = 1.0, radius: float = 0.5, width: float = 0.75) -> Dict[str, object]:
    """Cross a cube face while another coordinate remains in transition."""
    center = np.zeros(3)
    # Face is x=2*Delta; y=2*Delta+0.8 makes d=0.8, inside
    # (r,r+width) while x crosses the cube face.
    query = np.array([2.0 * delta, 2.0 * delta + 0.8, 0.0])
    epsilon = 1e-5
    left = cube_terms(query - np.array([epsilon, 0.0, 0.0]), center, delta, radius, width)
    right = cube_terms(query + np.array([epsilon, 0.0, 0.0]), center, delta, radius, width)
    return {
        "query": query.tolist(),
        "left_hessian_xx": float(left[2][0, 0]),
        "right_hessian_xx": float(right[2][0, 0]),
        "hessian_jump": float(right[2][0, 0] - left[2][0, 0]),
        "left_gradient": left[1].tolist(),
        "right_gradient": right[1].tolist(),
    }


def boundary_counterexample(delta: float = 1.0, radius: float = 0.5, width: float = 0.5) -> Dict[str, object]:
    particle = Particle(np.array([0.2, 0.3, 0.4]), 1.0)
    query = particle.position.copy()
    nodes, input_mass = project([particle], delta, dimensions=8, clip=True)
    cube = projected_risk(nodes, query, delta, radius, width, "cube")
    ball = projected_risk(nodes, query, delta, radius, width, "ball")
    original = original_risk([particle], query, radius, width)
    return {"input_mass": input_mass, "clipped_mass": sum(nodes.values()),
            "node_count": len(nodes), "original": original, "cube": cube, "ball": ball,
            "cube_minus_original": cube - original, "ball_minus_original": ball - original}


def parameter_scan() -> List[Dict[str, float]]:
    particles = [
        Particle(np.array([0.37, 0.23, -0.19]), 0.8),
        Particle(np.array([1.31, -0.44, 0.27]), 0.6),
        Particle(np.array([-1.18, 0.63, 0.52]), 0.4),
        Particle(np.array([0.12, 1.42, -0.66]), 0.9),
    ]
    queries = [p.position for p in particles]
    queries += [np.array([0.4, 0.4, 0.4]), np.array([2.1, 2.0, 1.7]), np.array([-1.0, 1.2, 0.0])]
    rows: List[Dict[str, float]] = []
    for delta in (1.0, 0.5, 0.25):
        nodes, _ = project(particles, delta)
        for width in (0.25, 0.5, 1.0):
            lower = []
            upper = []
            for query in queries:
                original = original_risk(particles, query, 0.75, width)
                cube = projected_risk(nodes, query, delta, 0.75, width, "cube")
                ball = projected_risk(nodes, query, delta, 0.75, width, "ball")
                lower.append(cube - original)
                upper.append(ball - cube)
            rows.append({"delta": delta, "smoothing_width": width,
                         "min_cube_minus_original": min(lower),
                         "max_cube_minus_original": max(lower),
                         "mean_cube_minus_original": float(np.mean(lower)),
                         "min_ball_minus_cube": min(upper),
                         "max_ball_minus_cube": max(upper),
                         "mean_ball_minus_cube": float(np.mean(upper)),
                         "node_count": float(len(nodes))})
    return rows


def benchmark(particle_count: int, horizon: int = 21, delta: float = 0.5) -> Dict[str, float]:
    rng = np.random.default_rng(1000 + particle_count)
    particles = [Particle(rng.uniform(-8.0, 8.0, 3), float(rng.uniform(0.05, 1.5))) for _ in range(particle_count)]
    start = time.perf_counter()
    cube_total = 0.0
    ball_total = 0.0
    node_count = 0
    for stage in range(horizon):
        shifted = [Particle(p.position + 0.05 * stage * rng.normal(0.0, 1.0, 3), p.weight) for p in particles]
        nodes, _ = project(shifted, delta)
        node_count = max(node_count, len(nodes))
        query = np.array([0.2, -0.3, 0.4])
        cube_total += projected_risk(nodes, query, delta, 1.0, 0.5, "cube")
        ball_total += projected_risk(nodes, query, delta, 1.0, 0.5, "ball")
    elapsed_ms = 1e3 * (time.perf_counter() - start)
    return {"particle_count": particle_count, "horizon": horizon, "max_nodes": node_count,
            "total_ms": elapsed_ms, "per_stage_ms": elapsed_ms / horizon,
            "sink": cube_total + ball_total}


def report() -> Dict[str, object]:
    return {
        "inequality_scan": inequality_scan(),
        "boundary_counterexample": boundary_counterexample(),
        "transition_face_jump": find_transition_face_jump(),
        "parameter_scan": parameter_scan(),
        "benchmarks": [benchmark(count) for count in (256, 1024, 2048)],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    print(json.dumps(report(), indent=2, default=lambda value: value.tolist() if isinstance(value, np.ndarray) else value))


if __name__ == "__main__":
    main()
