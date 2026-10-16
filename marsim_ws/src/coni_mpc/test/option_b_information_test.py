#!/usr/bin/env python3
"""Standalone proof-facing and exact-arithmetic Option B checks."""
from fractions import Fraction as F
import pathlib
import sys
import unittest
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]/"scripts"))
import option_b_information_experiment as B
from stage3c_cube_support_experiment import project


class OptionBInformationTest(unittest.TestCase):
    def test_exact_p2g_partition_and_raw_global_moments(self):
        ps = [B.Particle(np.array([.37, .23, -.19]), .7),
              B.Particle(np.array([1.2, -.4, .6]), 1.1)]
        nodes = B.exact_project(ps, .5)
        total = sum(v[0] for v in nodes.values())
        self.assertEqual(total, sum(F(float(p.weight)) for p in ps))
        world_first = [F(0)]*3
        world_second = [[F(0)]*3 for _ in range(3)]
        for idx, (m, ell, Q) in nodes.items():
            c = [F(i, 2) for i in idx]
            for k in range(3):
                world_first[k] += ell[k]+m*c[k]
                for l in range(3):
                    world_second[k][l] += Q[k][l]+c[k]*ell[l]+c[l]*ell[k]+m*c[k]*c[l]
        for k in range(3):
            self.assertEqual(world_first[k], sum(F(float(p.weight))*F(float(p.position[k])) for p in ps))
            for l in range(3):
                self.assertEqual(world_second[k][l], sum(F(float(p.weight))*F(float(p.position[k]))*
                                                        F(float(p.position[l])) for p in ps))

    def test_mass_projection_matches_unchanged_reference(self):
        ps, _ = B.matched_scenarios()["dense"]
        old, _ = project(ps, .5)
        stats = B.project_moments(ps, .5)
        np.testing.assert_allclose(stats.mass, list(old.values()), atol=1e-14)

    def test_single_atom_moments_recover_original_risk(self):
        ps, qs = B.matched_scenarios()["single"]
        stats = B.project_moments(ps, .5)
        for q in qs:
            v = B.bounds(stats, q, .5)
            self.assertAlmostEqual(v["full"], B.original_risk(ps, q, 1., .5), places=12)

    def test_exact_zero_and_vanishing_mass(self):
        zero = [F(0), [F(0)]*3, [[F(0)]*3 for _ in range(3)]]
        r = B.exact_node_bounds(zero, [F(0)]*3, F(1), F(1, 2), F(1, 2))
        self.assertTrue(all(v == 0 for v in r.values()))
        bad = [F(0), [F(1), F(0), F(0)], zero[2]]
        with self.assertRaises(ValueError):
            B.exact_node_bounds(bad, [F(0)]*3, F(1), F(1, 2), F(1, 2))
        for m in (F(1, 10**30), F(1, 10**200)):
            stats = [m, [F(0)]*3, [[F(0)]*3 for _ in range(3)]]
            r = B.exact_node_bounds(stats, [F(1), F(0), F(0)], F(1), F(1, 2), F(1, 2))
            self.assertGreaterEqual(r["full_upper"], 0)
            self.assertLessEqual(r["full_upper"], m)

    def test_interval_square_roots_and_directed_dyadic_rounding(self):
        for x in (F(0), F(1, 3), F(2), F(4), F(1, 10**1000), F(10**100)):
            l, u = B.sqrt_bracket(x)
            self.assertLessEqual(l*l, x)
            self.assertGreaterEqual(u*u, x)
        for x in (F(1, 3), F(-1, 3), F(2, 7)):
            self.assertLessEqual(B.rational_down(x), x)
            self.assertGreaterEqual(B.rational_up(x), x)

    def test_exact_certificate_dominates_known_two_atom_measure(self):
        # Interior feasible measure, exact moments, and no particle argument to evaluator.
        moment = [F(1), [F(0)]*3, [[F(1, 4), F(0), F(0)], [F(0)]*3, [F(0)]*3]]
        for yx in (F(0), F(1, 2), F(1), F(3, 2)):
            y = [yx, F(0), F(0)]
            r = B.exact_node_bounds(moment, y, F(1), F(1, 5), F(1, 5))
            actual = (B.phi_fraction(abs(yx-F(1, 2)), F(1, 5), F(1, 5))+
                      B.phi_fraction(abs(yx+F(1, 2)), F(1, 5), F(1, 5)))/2
            self.assertGreaterEqual(r["full_upper"], actual)
            self.assertLessEqual(r["full_upper"], r["first_upper"])
            self.assertLessEqual(r["first_upper"], r["C_upper"])

    def test_paired_matched_sandwich_and_second_moment_information(self):
        cases = B.matched_scenarios()
        for name, (p, q) in cases.items():
            stats = B.project_moments(p, .5)
            gain = 0
            for query in q[::max(1, len(q)//8)]:
                row = B.bounds(stats, query, .5)
                original = B.original_risk(p, query, 1., .5)
                self.assertLessEqual(original, row["full"]+1e-10)
                self.assertLessEqual(row["full"], row["first"]+1e-10)
                self.assertLessEqual(row["first"], row["C"]+1e-10)
                gain += row["first"]-row["full"]
            self.assertGreater(gain, 1e-4)

    def test_psd_and_coordinate_bounds_are_not_sufficient(self):
        Q = np.eye(3)*1.5-np.ones((3, 3))*.5
        self.assertGreaterEqual(np.linalg.eigvalsh(Q).min(), -1e-14)
        np.testing.assert_array_equal(np.diag(Q), np.ones(3))
        self.assertEqual(Q.sum(), 0.)
        # All sign-corner sums have squared magnitude >=1, contradicting Q.sum=0.
        sums = [sum(2*np.array(i)-1)**2 for i in np.ndindex(2, 2, 2)]
        self.assertGreaterEqual(min(sums), 1)

    def test_true_moment_optimum_can_have_spatial_cusp(self):
        r = B.theoretical_examples()["cusp"]
        self.assertAlmostEqual(r["value"], .125)
        self.assertAlmostEqual(r["left_slope"], -.46875, places=5)
        self.assertAlmostEqual(r["right_slope"], .46875, places=5)

    def test_same_interior_moments_do_not_identify_risk(self):
        r = B.theoretical_examples()["same_interior_moments"]
        self.assertEqual(r["risk_A"], 0.)
        self.assertAlmostEqual(r["risk_B"], .609375)

    def test_finite_atom_lp_is_not_a_continuum_upper_bound(self):
        r = B.theoretical_examples()["same_interior_moments"]
        self.assertTrue(r["numerical_restricted_LP"]["success"])
        value = r["numerical_restricted_LP"]["restricted_objective"]
        self.assertAlmostEqual(value, .609375, places=10)
        self.assertLess(value, r["known_continuum_feasible_risk"])

    def test_off_grid_dirac_moments_make_restricted_lp_infeasible(self):
        r = B.theoretical_examples()["zero_variance_off_grid_LP"]
        self.assertFalse(r["success"])
        self.assertEqual(r["status"], 2)

    def test_bin_refinement_is_nested_safe_upper_sum(self):
        p, q = B.matched_scenarios()["dense"]
        stats = B.project_moments(p, .5)
        previous = float("inf")
        for bins in (8, 16, 32, 64):
            row = B.bounds(stats, q[0], .5, bins=bins)
            self.assertLessEqual(row["full"], previous+1e-10)
            previous = row["full"]

    def test_mass_scaling_and_no_nonzero_node_cropping(self):
        p, q = B.matched_scenarios()["sparse"]
        stats = B.project_moments(p, .5)
        scaled = B.Moments(stats.centers, 3*stats.mass, 3*stats.first, 3*stats.second)
        r, rs = B.bounds(stats, q[0], .5), B.bounds(scaled, q[0], .5)
        for k in ("C", "first", "full"):
            self.assertAlmostEqual(rs[k], 3*r[k], places=11)
        self.assertEqual(r["nodes"], len(stats.mass))

    def test_first_moments_are_information_beyond_all_grid_masses(self):
        r = B.same_grid_information_examples()["same_mass_different_first"]
        self.assertTrue(r["same_all_P2G_node_masses_exact"])
        self.assertFalse(r["same_all_node_first_moments_exact"])
        self.assertGreater(F(r["max_first_difference_exact"]), 0)
        self.assertAlmostEqual(r["risk_A"], .75)
        self.assertEqual(r["risk_B"], 0.)

    def test_second_moments_are_information_beyond_mass_and_first(self):
        r = B.same_grid_information_examples()["same_mass_and_first_different_second"]
        self.assertTrue(r["same_all_P2G_node_masses_exact"])
        self.assertTrue(r["same_all_node_first_moments_exact"])
        self.assertGreater(F(r["max_second_difference_exact"]), 0)
        self.assertEqual(r["risk_A"], 0.)
        self.assertAlmostEqual(r["risk_B"], .625)


if __name__ == "__main__":
    unittest.main()
