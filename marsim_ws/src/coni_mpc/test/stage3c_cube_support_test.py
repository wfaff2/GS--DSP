#!/usr/bin/env python3
"""Independent tests for the stage-3C cubic-support risk candidate."""

import pathlib
import sys
import unittest

import numpy as np

SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from stage3c_cube_support_experiment import (  # noqa: E402
    Particle,
    ball_distance,
    boundary_counterexample,
    cube_distance,
    cube_terms,
    find_transition_face_jump,
    inequality_scan,
    kernel,
    node,
    project,
    projected_risk,
    original_risk,
    support,
)


class Stage3CCubeSupportTest(unittest.TestCase):
    def test_cubic_support_is_contained_in_the_true_cube_and_moments_hold(self):
        delta = 0.7
        point = np.array([0.37, -0.23, 1.19])
        entries = list(support(point, delta))
        self.assertEqual(len(entries), 64)
        self.assertAlmostEqual(sum(weight for _, weight in entries), 1.0, places=14)
        first = sum((weight * node(index, delta) for index, weight in entries), np.zeros(3))
        np.testing.assert_allclose(first, point, atol=2e-14)
        second = sum(weight * np.dot(node(index, delta) - point, node(index, delta) - point)
                     for index, weight in entries)
        self.assertAlmostEqual(second, delta * delta, places=13)
        for index, _ in entries:
            error = node(index, delta) - point
            self.assertLessEqual(np.max(np.abs(error)), 2.0 * delta + 1e-12)

    def test_complete_support_has_the_two_sided_value_ordering(self):
        result = inequality_scan(seed=20261011, count=200, delta=0.5)
        for scenario in result.values():
            self.assertGreaterEqual(scenario["min_cube_minus_original"], -2e-12)
            self.assertGreaterEqual(scenario["min_ball_minus_cube"], -2e-12)

    def test_targeted_transition_queries_preserve_ordering(self):
        radius, width, delta = 0.5, 0.75, 1.0
        particles = [
            Particle(np.array([0.37, 0.23, -0.19]), 0.8),
            Particle(np.array([1.31, -0.44, 0.27]), 0.6),
            Particle(np.array([-1.18, 0.63, 0.52]), 0.4),
            Particle(np.array([0.12, 1.42, -0.66]), 0.9),
        ]
        nodes, _ = project(particles, delta)
        for query in (
            np.array([1.0, 0.23, -0.19]),
            np.array([2.2, 2.1, 0.4]),
            np.array([-0.4, 0.7, 0.2]),
            np.array([0.37, 0.23, -0.19]),
        ):
            original = original_risk(particles, query, radius, width)
            cube = projected_risk(nodes, query, delta, radius, width, "cube")
            ball = projected_risk(nodes, query, delta, radius, width, "ball")
            self.assertLessEqual(original, cube + 1e-12)
            self.assertLessEqual(cube, ball + 1e-12)

    def test_cube_and_ball_distances_have_the_expected_pointwise_order(self):
        rng = np.random.default_rng(3)
        for _ in range(1000):
            center = rng.uniform(-2.0, 2.0, 3)
            query = rng.uniform(-5.0, 5.0, 3)
            cube = cube_distance(query, center, 1.0)[0]
            ball = ball_distance(query, center, np.sqrt(3.0))
            self.assertGreaterEqual(cube + 1e-12, ball)

    def test_boundary_clipping_breaks_the_upper_bound(self):
        result = boundary_counterexample()
        self.assertLess(result["clipped_mass"], result["input_mass"])
        self.assertLess(result["cube"], result["original"] - 1e-6)
        self.assertLess(result["ball"], result["original"] - 1e-6)

    def test_local_node_crop_breaks_the_upper_bound(self):
        particles = [Particle(np.array([3.37, 3.23, 3.19]), 1.0)]
        nodes, _ = project(particles, 1.0)
        query = particles[0].position
        original = original_risk(particles, query, 0.5, 0.5)
        # Keep only a strict subset of positive-mass nodes, modeling a query
        # crop that did not preserve all P2G contributors.
        one_node = {next(iter(nodes)): next(iter(nodes.values()))}
        cropped = projected_risk(one_node, query, 1.0, 0.5, 0.5, "cube")
        self.assertLess(cropped, original)

    def test_derivatives_match_finite_difference_away_from_cube_switches(self):
        delta, radius, width = 1.0, 0.5, 0.9
        center = np.zeros(3)
        query = np.array([2.7, 2.4, 0.2])
        value, gradient, hessian = cube_terms(query, center, delta, radius, width)
        h = 1e-5

        def value_at(point):
            return cube_terms(point, center, delta, radius, width)[0]

        finite_gradient = np.zeros(3)
        finite_hessian = np.zeros((3, 3))
        for axis in range(3):
            offset = np.zeros(3)
            offset[axis] = h
            finite_gradient[axis] = (value_at(query + offset) - value_at(query - offset)) / (2.0 * h)
            finite_hessian[:, axis] = (
                cube_terms(query + offset, center, delta, radius, width)[1]
                - cube_terms(query - offset, center, delta, radius, width)[1]
            ) / (2.0 * h)
        self.assertAlmostEqual(value, value_at(query), places=14)
        np.testing.assert_allclose(gradient, finite_gradient, atol=2e-6)
        np.testing.assert_allclose(hessian, finite_hessian, atol=2e-5)

    def test_cube_kernel_has_a_hessian_jump_at_a_face_active_set_switch(self):
        result = find_transition_face_jump()
        self.assertGreater(abs(result["hessian_jump"]), 1.0)
        self.assertLess(np.linalg.norm(np.asarray(result["left_gradient"]) -
                                       np.asarray(result["right_gradient"])), 1e-3)


if __name__ == "__main__":
    unittest.main()
