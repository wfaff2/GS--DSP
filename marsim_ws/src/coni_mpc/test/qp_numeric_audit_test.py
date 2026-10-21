#!/usr/bin/env python3
"""Checks independent KKT definitions and rejection of corrupted evidence."""
import sys
import unittest
import struct
import tempfile
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from qp_numeric_audit import record_dtype, qp_residuals, load_capture

class QpAuditTest(unittest.TestCase):
    def record(self):
        r = np.zeros(1, dtype=record_dtype())[0]
        r['H'] = np.eye(140).ravel()
        r['lb'] = -1e20; r['ub'] = 1e20
        r['lbA'] = -1e20; r['ubA'] = 1e20
        return r

    def test_lower_variable_and_upper_linear_dual_order(self):
        r = self.record()
        r['lb'][0] = 1; r['solution'][0] = 1; r['dual'][0] = 1
        r['A'][140 + 1] = 1; r['ubA'][1] = -2
        r['solution'][1] = -2; r['dual'][141] = -2
        r['objective'] = 2.5; r['kkt'] = 5
        m = qp_residuals(r)
        for k in ('primal_inf','stationarity_inf','dual_sign_inf','complementarity_inf'):
            self.assertEqual(m[k], 0)
        self.assertEqual(m['acado_kkt_expression_recomputed'], 5)

    def test_large_acado_scalar_can_have_zero_true_residual(self):
        r = self.record(); r['g'][0] = -10; r['solution'][0] = 10
        self.assertEqual(qp_residuals(r)['stationarity_inf'], 0)
        self.assertEqual(qp_residuals(r)['acado_kkt_expression_recomputed'], 100)

    def test_corrupt_solution_and_dual_detected(self):
        r = self.record(); r['lb'][0] = 1; r['solution'][0] = .5
        m = qp_residuals(r)
        self.assertEqual(m['primal_inf'], .5)
        self.assertGreater(m['primal_relative'], .1)  # remote sentinel cannot hide it
        r['solution'][0] = 1; r['dual'][0] = -1
        self.assertGreater(qp_residuals(r)['stationarity_inf'], 0)
        self.assertGreater(qp_residuals(r)['dual_sign_inf'], 0)

    def test_interior_multiplier_and_nonfinite_detected(self):
        r = self.record(); r['lb'][0] = 0; r['solution'][0] = 1; r['dual'][0] = 1
        self.assertEqual(qp_residuals(r)['complementarity_inf'], 1)
        r['g'][0] = np.nan
        with self.assertRaises(ValueError): qp_residuals(r)

    def test_old_and_world_velocity_capture_contracts(self):
        for nc, nod in ((200, 1827), (240, 2079)):
            dtype = record_dtype(nc, nod)
            records = np.zeros(1, dtype=dtype)
            records['H'][0] = np.eye(140).ravel()
            records['lb'] = -1e20; records['ub'] = 1e20
            records['lbA'] = -1e20; records['ubA'] = 1e20
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'capture.bin'
                path.write_bytes(struct.pack('<Q6I', 0x3150514D4F434343,
                    1, 140, nc, 147, 140, nod) + records.tobytes())
                loaded = load_capture(path)
                self.assertEqual(loaded[0]['lbA'].size, nc)
                self.assertEqual(loaded[0]['od'].size, nod)
                self.assertEqual(qp_residuals(loaded[0])['primal_inf'], 0.)

if __name__ == '__main__': unittest.main()
