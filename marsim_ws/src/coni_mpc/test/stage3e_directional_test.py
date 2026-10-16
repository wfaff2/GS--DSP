#!/usr/bin/env python3
"""Proof-facing, offline checks; numerical tests do not replace the report proof."""
import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]/"scripts"))
import stage3e_directional_experiment as E
from stage3c_cube_support_experiment import ball_terms, cube_terms
from stage3d_superellipsoid_experiment import super_distance_terms


class DirectionalAuditTest(unittest.TestCase):
    def test_exact_constant_and_support_envelope(self):
        self.assertAlmostEqual(E.ALPHA, .3480762113533159, places=15)
        scan = E.geometry_audit()
        for key in ("min_b_minus_cube_support", "min_sphere_radius_minus_b",
                    "min_E_minus_A_distance", "min_C_minus_E_distance"):
            self.assertGreaterEqual(scan[key], -2e-14)

    def test_axes_diagonals_and_sign_permutation_symmetry(self):
        _, bound = E.directional_distance(np.array([[1, 0, 0], [1, 1, 1]]), .5)
        np.testing.assert_allclose(bound, [1.5, np.sqrt(3)], atol=1e-15)
        y = np.array([2.6, .8, -.4])
        a = E.node_terms(y, .5, .48, .5)
        b = E.node_terms(-y[::-1], .5, .48, .5)
        np.testing.assert_allclose(a[0], b[0], atol=1e-14)
        np.testing.assert_allclose(a[1], -b[1][:, ::-1], atol=1e-14)
        np.testing.assert_allclose(a[2], b[2][:, ::-1, ::-1], atol=1e-14)

    def test_origin_and_positive_part_switch_hidden_by_plateau(self):
        for y in ([0, 0, 0], [1e-20, -1e-20, 0], [1.5, 0, 0], [1.5+1e-8, 0, 0]):
            k, g, h = E.node_terms(y, .5, .48, .5)
            np.testing.assert_array_equal(k, [1.])
            np.testing.assert_array_equal(g, np.zeros((1, 3)))
            np.testing.assert_array_equal(h, np.zeros((1, 3, 3)))

    def test_smootherstep_joins_from_both_sides(self):
        for u in (np.array([1., 0, 0]), np.ones(3)/np.sqrt(3), np.array([1., 1., 0])/np.sqrt(2)):
            bound = E.directional_distance(u, .5)[1][0]
            for d in (.48, .98):
                limit = E.node_terms((bound+d)*u, .5, .48, .5)
                self.assertLess(np.linalg.norm(limit[1]), 1e-12)
                self.assertLess(np.linalg.norm(limit[2]), 1e-11)
                norms = []
                for eps in (1e-4, 1e-6, 1e-8):
                    l = E.node_terms((bound+d-eps)*u, .5, .48, .5)
                    r = E.node_terms((bound+d+eps)*u, .5, .48, .5)
                    norms.append(np.linalg.norm(l[2]-r[2]))
                self.assertLess(norms[-1], norms[0]/1000)

    def test_gradient_hessian_finite_differences(self):
        result = E.derivative_audit()
        self.assertLess(result["max_gradient_error"], 2e-7)
        self.assertLess(result["max_hessian_error"], 2e-6)

    def test_comparison_evaluators_match_unchanged_references(self):
        rng = np.random.default_rng(42)
        y = rng.uniform(-4, 4, (40, 3))
        for mode, reference in (("A", ball_terms), ("C", cube_terms), ("D", super_distance_terms)):
            vals, gs, hs = E.node_terms(y, .5, .48, .5, mode)
            for k, q in enumerate(y):
                if mode == "D":
                    value, gradient, hessian, _ = reference(q, .5, .48, .5)
                else:
                    value, gradient, hessian = reference(q, np.zeros(3), .5, .48, .5)
                self.assertAlmostEqual(vals[k], value, places=10)
                np.testing.assert_allclose(gs[k], gradient, atol=1e-9)
                np.testing.assert_allclose(hs[k], hessian, atol=1e-8)

    def test_complete_support_sandwich_parameter_changes(self):
        p, qs = E.matched_scenarios()["sparse"]
        for delta in (.25, .5, 1.):
            for width in (.12, .5):
                result = E.gap_metrics(p, qs[::3], delta, .48, width)
                for key in ("smooth-hard", "C-smooth", "E-C", "A-E", "D-C", "A-D"):
                    self.assertGreaterEqual(result["gaps"][key]["min"], -1e-10)

    def test_p4_axis_counterexample_to_uniform_E_improvement(self):
        # a=1, dE=.70, dD=.883925..., both in (.48,.98).
        y = np.array([2.2, 0., 0.])
        dE, _ = E.directional_distance(y, .5)
        dD = E.super_projection(y, .5)[0]
        self.assertLess(dE[0], dD)
        self.assertGreater(E.node_terms(y, .5, .48, .5, "E")[0][0],
                           E.node_terms(y, .5, .48, .5, "D")[0][0])

    def test_temporal_and_mixed_derivatives_at_grid_cell_crossing(self):
        r = E.temporal_audit()
        for key in ("time_derivative_error", "time_second_derivative_error", "mixed_gradient_time_error"):
            self.assertLess(r[key], 2e-7)
        np.testing.assert_allclose(r["mass_derivatives"], [1.7, 0, 0], atol=2e-14)

    def test_boundary_and_query_cropping_fail_conservativeness(self):
        r = E.cropping_audit()
        self.assertGreater(r["lost_mass"], .1)
        self.assertLess(r["boundary_E"], r["smooth_at_particle"])
        self.assertLess(r["query_crop_E"], r["smooth_at_particle"])
        with self.assertRaises(ValueError):
            E.project([E.Particle(np.array([.2, .3, .4]), 1.)], 1., dimensions=8)

    def test_only_proved_exact_zero_pruning_equivalent(self):
        p, qs = E.matched_scenarios()["sparse"]
        nodes, _ = E.project(p, .5)
        c, m = E.arrays(nodes, .5)
        for q in qs:
            # sphere cutoff implies zero value and zero derivatives for all four bounds.
            keep = np.linalg.norm(q-c, axis=1) < np.sqrt(3)+.48+.5
            for mode in E.MODES:
                full = E.evaluate(c, m, q, .5, .48, .5, mode)
                pruned = E.evaluate(c[keep], m[keep], q, .5, .48, .5, mode)
                for f, pp in zip(full, pruned):
                    np.testing.assert_allclose(f, pp, atol=3e-12)

    def test_nonnegative_mass_required(self):
        with self.assertRaises(ValueError):
            E.evaluate(np.zeros((1, 3)), np.array([-1.]), np.zeros(3), .5, .48, .5)
        with self.assertRaises(ValueError):
            E.node_terms([0, 0, 0], .5, 0., .5)

    def test_component_only_outputs_match_full(self):
        y = np.array([[2.2, .3, -.4], [0., 0., 0.], [20., 10., 1.]])
        for mode in E.MODES:
            full = E.node_terms(y, .5, .48, .5, mode)
            for k, component in enumerate(("value", "gradient", "hessian")):
                isolated = E.node_terms(y, .5, .48, .5, mode, k, component)
                np.testing.assert_allclose(full[k], isolated[k], atol=1e-12)

    def test_live_singleton_boundary_geometry_is_separate_from_risk(self):
        rows = E.live_boundary_geometry()["rows"]
        for row in rows:
            self.assertGreaterEqual(row["grid_values"]["E"]+1e-12, row["particle_smooth"])
        hzero = [row for row in rows if abs(row["frozen_singleton_live_h_m"]) < 1e-8]
        self.assertEqual(len(hzero), 3)
        for row in hzero:
            self.assertAlmostEqual(row["particle_smooth"], 1.)


if __name__ == "__main__":
    unittest.main()
