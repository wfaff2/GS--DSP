#!/usr/bin/env python3
"""Cross-check one-UAV provenance against actual QP capture and an off/on gate."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from qp_numeric_audit import load_capture


def check_capture(provenance, capture):
    rows = list(csv.DictReader(provenance.open()))
    qp = load_capture(capture)
    assert len(rows) == len(qp) and len(rows) > 0
    # The legacy binary format has no UAV ID: refuse ambiguous multi-UAV joins.
    assert len({r['uav_idx'] for r in rows}) == 1, 'QP format has no UAV ID'
    for i, (r, q) in enumerate(zip(rows, qp)):
        assert int(r['cycle_id']) == int(q['index']) == i
        assert int(r['qp_status']) == int(q['status'])
        for j in range(4):
            assert float(r[f'qp_nominal{j}']) == q['u'][j]
            assert float(r[f'qp_increment{j}']) == q['solution'][j]
            assert abs(float(r[f'acado_u{j}']) - q['u'][j] - q['solution'][j]) < 1e-12
        for j, axis in enumerate('xyz'):
            assert float(r[f'estimate_p{axis}_n']) == q['x0'][j]
            assert float(r[f'omega_profile0_{axis}']) == q['od'][68+j]
    return {'cycles': len(rows), 'status_and_nominal_increment_expand_match': True,
            'od_omega_and_estimate_x0_match': True}


def check_gate(off, on):
    a, b = off/'acado_qp_capture.bin', on/'acado_qp_capture.bin'
    assert a.read_bytes() == b.read_bytes(), 'off/on full QP differs'
    assert not (off/'world_command_provenance.csv').exists()
    cols = ['applied_cmd_vx_world', 'applied_cmd_vy_world',
            'applied_cmd_vz_world', 'applied_cmd_yaw_rate_world']
    ra = list(csv.DictReader((off/'metrics_steps.csv').open()))
    rb = list(csv.DictReader((on/'metrics_steps.csv').open()))
    assert len(ra) == len(rb) > 0
    for x, y in zip(ra, rb):
        assert (x['uav_idx'], x['step_idx']) == (y['uav_idx'], y['step_idx'])
        assert all(x[k] == y[k] for k in cols)
    return {'cycles': len(ra), 'capture_byte_identical': True, 'off_file_absent': True,
            'command_columns_identical': cols,
            'scope': 'deterministic no-perception software gate test; not identity of two asynchronous live-DSP runs'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('provenance', type=Path)
    ap.add_argument('capture', type=Path)
    ap.add_argument('--gate-off', type=Path)
    ap.add_argument('--gate-on', type=Path)
    ap.add_argument('--output-dir', type=Path, required=True)
    args = ap.parse_args()
    assert bool(args.gate_off) == bool(args.gate_on), 'specify both gate archives'
    args.output_dir.mkdir(parents=True, exist_ok=True)
    result = check_capture(args.provenance, args.capture)
    (args.output_dir/'cross_capture_validation.json').write_text(json.dumps(result, indent=2))
    if args.gate_off:
        gate = check_gate(args.gate_off, args.gate_on)
        (args.output_dir/'gate_validation.json').write_text(json.dumps(gate, indent=2))
        result['gate'] = gate
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
