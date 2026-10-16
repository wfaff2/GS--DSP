#!/usr/bin/env python3
"""Validate actually captured cycles and reject corrupted command association."""
import argparse,csv,sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_world_command_provenance import vec,rotation,decompose


def validate(rows):
    groups={}
    for r in rows:groups.setdefault(int(r['uav_idx']),[]).append(r)
    for group in groups.values():
        assert [int(r['cycle_id']) for r in group]==list(range(len(group)))
        for r in group:
            assert int(r['qp_status'])==0 and int(r['solve_ok'])==1
            assert not int(r['fallback_used'])
            assert np.max(abs(vec(r,'comp')-vec(r,'u')-vec(r,'cross')))<1e-12
            assert np.max(abs(vec(r,'before')-vec(r,'vg')-rotation(r)@vec(r,'comp')))<1e-12
            assert np.max(abs(vec(r,'after')-vec(r,'published')))<1e-12
            assert all(abs(float(r[f'acado_u{i}'])-float(r[f'qp_nominal{i}'])-float(r[f'qp_increment{i}']))<1e-12 for i in range(4))
            assert all(float(r[f'quad_p{x}_w'])==float(r[f'solve_quad_p{x}_w']) for x in 'xyz')
            assert all(float(r[f'ugv_omega_{x}_n'])==float(r[f'omega_profile0_{x}']) for x in 'xyz')
            assert float(r['feedback_start_mono'])<=float(r['feedback_end_mono'])<=float(r['mpc_end_mono_sec'])<=float(r['publish_mono_sec'])
        for a,b in zip(group,group[1:]):
            assert np.max(abs(sum(decompose(a,b).values())-(vec(b,'published')-vec(a,'published'))))<1e-12
    return len(rows)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('csv',type=Path);a=p.parse_args()
    rows=list(csv.DictReader(a.csv.open()));print(f'validated_cycles={validate(rows)}')
    corrupted=[dict(r) for r in rows];corrupted[1]['published_vx']=str(float(corrupted[1]['published_vx'])+.01)
    try: validate(corrupted)
    except AssertionError: print('corrupted publisher value rejected')
    else: raise AssertionError('missed corruption')
