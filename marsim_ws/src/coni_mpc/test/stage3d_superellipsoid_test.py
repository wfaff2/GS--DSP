#!/usr/bin/env python3
"""Independent tests for the exact p=4 superellipsoid support candidate."""

import pathlib
import sys
import unittest

import numpy as np

SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from stage3d_superellipsoid_experiment import (  # noqa: E402
    Particle,
    ball_distance,
    boundary_counterexample,
    containment_scan,
    convergence_scan,
    derivative_audit,
    inequality_scan,
    node,
    original_risk,
    project,
    projected_risk,
    projection_scan,
    super_distance_batch,
    super_distance_terms,
    super_projection,
    super_risk,
)
from stage3c_cube_support_experiment import cube_distance  # noqa: E402


class Stage3DSuperellipsoidTest(unittest.TestCase):
    def test_geometric_containment(self):
        result = containment_scan()
        self.assertLessEqual(result["max_cube_F"], 1e-10)
        self.assertLessEqual(result["max_superball_minus_sphere"], 1e-10)

    def test_projection_kkt_residual_and_pointwise_distance_order(self):
        result = projection_scan()
        self.assertLess(result["max_constraint_residual"], 1e-10)
        self.assertLess(result["max_kkt_residual"], 1e-10)
        rng = np.random.default_rng(4)
        for _ in range(200):
            query = rng.uniform(-6.0, 6.0, 3)
            delta = 0.7
            d_super = super_projection(query, delta)[0]
            d_cube = cube_distance(query, np.zeros(3), 2.0 * delta)[0]
            d_ball = ball_distance(query, np.zeros(3), 2.0 * np.sqrt(3.0) * delta)
            self.assertGreaterEqual(d_super + 1e-10, d_ball)
            self.assertLessEqual(d_super, d_cube + 1e-10)

    def test_projection_bisection_converges(self):
        result = convergence_scan()
        self.assertLess(result["max_distance_error_48"], 1e-12)
        self.assertLess(result["max_distance_error_64"], 1e-14)

    def test_vectorized_projection_matches_scalar_projection(self):
        rng = np.random.default_rng(7)
        points = rng.uniform(-5.0, 5.0, (40, 3))
        batch = super_distance_batch(points, 0.6)
        scalar = np.array([super_projection(point, 0.6)[0] for point in points])
        np.testing.assert_allclose(batch, scalar, atol=2e-10, rtol=2e-10)

    def test_complete_support_risk_sandwich(self):
        result = inequality_scan(seed=20261011, delta=0.5)
        for scenario in result.values():
            self.assertGreaterEqual(scenario["min_super_minus_original"], -2e-10)
            self.assertGreaterEqual(scenario["min_cube_minus_original"], -2e-10)
            self.assertGreaterEqual(scenario["min_super_minus_cube"], -2e-10)
            self.assertGreaterEqual(scenario["min_ball_minus_super"], -2e-10)

    def test_risk_sandwich_at_transition_queries(self):
        particles = [
            Particle(np.array([0.37, 0.23, -0.19]), 0.8),
            Particle(np.array([1.31, -0.44, 0.27]), 0.6),
            Particle(np.array([-1.18, 0.63, 0.52]), 0.4),
        ]
        nodes, _ = project(particles, 0.5)
        for query in (np.array([2.8, 2.8, 0.0]), np.array([1.1, 0.7, -0.2]),
                      np.array([0.37, 0.23, -0.19])):
            original = original_risk(particles, query, 0.5, 0.8)
            super_value = super_risk(nodes, query, 0.5, 0.5, 0.8)
            ball_value = projected_risk(nodes, query, 0.5, 0.5, 0.8, "ball")
            self.assertLessEqual(original, super_value + 1e-10)
            self.assertLessEqual(super_value, ball_value + 1e-10)

    def test_derivatives_match_finite_difference(self):
        result = derivative_audit()
        self.assertLess(result["gradient_error"], 2e-6)
        self.assertLess(result["hessian_error"], 2e-5)
        self.assertLess(result["constraint_residual"], 1e-10)
        self.assertLess(result["projection_residual"], 1e-10)

    def test_kernel_platform_and_support_joins_have_zero_first_second_derivatives(self):
        delta, radius, width = 1.0, 0.5, 0.8
        boundary = 2.0 * delta * (3.0 ** 0.25)
        for distance in (radius, radius + width):
            query = np.array([boundary + distance, 0.0, 0.0])
            value, gradient, hessian, _ = super_distance_terms(query, delta, radius, width)
            if distance == radius:
                self.assertAlmostEqual(value, 1.0, places=9)
            else:
                self.assertAlmostEqual(value, 0.0, places=9)
            self.assertLess(np.linalg.norm(gradient), 1e-8)
            self.assertLess(np.linalg.norm(hessian), 1e-7)

    def test_boundary_clipping_remains_invalid(self):
        result = boundary_counterexample()
        self.assertLess(result["clipped_mass"], result["input_mass"])
        self.assertLess(result["super"], result["original"] - 1e-6)


if __name__ == "__main__":
    unittest.main()
