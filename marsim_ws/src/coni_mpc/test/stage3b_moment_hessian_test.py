#!/usr/bin/env python3
"""Independent numerical tests for the C1 stage-3B audit."""

import pathlib
import sys
import unittest

import numpy as np

SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from stage3b_moment_hessian_experiment import (  # noqa: E402
    find_counterexample,
    find_multi_particle_counterexample,
    hessian_bound,
    hessian_norm,
    moments,
    truncated_moments,
)


class Stage3BMomentHessianTest(unittest.TestCase):
    def test_interior_partition_first_and_second_moments(self):
        for delta in (1.0, 0.5, 0.25, 0.13):
            for point in ([0.37, 0.23, -0.19], [0.5, 0.5, 0.5], [2.01, 3.27, 4.49]):
                result = moments(np.asarray(point), delta)
                self.assertAlmostEqual(result["sum_N"], 1.0, places=14)
                np.testing.assert_allclose(result["first_error"], np.zeros(3), atol=2e-14)
                self.assertAlmostEqual(result["second_moment"], delta * delta, places=13)

    def test_boundary_truncation_is_outside_the_moment_theorem(self):
        result = truncated_moments([0.2, 0.3, 0.4], 1.0)
        self.assertLess(result["sum_N"], 1.0)
        self.assertGreater(np.linalg.norm(result["first_error"]), 1e-3)
        self.assertNotAlmostEqual(result["second_moment"], 1.0, places=3)

    def test_hessian_closed_form_bound_covers_dense_scan(self):
        for radius, width in ((0.2, 0.1), (1.0, 0.5), (2.0, 1.7)):
            bound = hessian_bound(radius, width)
            distances = np.linspace(radius, radius + width, 10001)[1:-1]
            maximum = max(hessian_norm(float(distance), radius, width) for distance in distances)
            self.assertGreaterEqual(bound + 1e-10, maximum)
            self.assertGreater(bound, 0.0)

    def test_requested_minus_sign_is_not_an_upper_bound(self):
        result = find_counterexample()
        self.assertGreater(result["gap"], 1e-3)
        self.assertGreater(result["exact"], result["candidate_minus"])

    def test_requested_minus_sign_also_fails_for_four_particles(self):
        result = find_multi_particle_counterexample()
        self.assertGreater(result["gap"], 1e-3)
        self.assertGreater(result["exact"], result["candidate_minus"])


if __name__ == "__main__":
    unittest.main()
