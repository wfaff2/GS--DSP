#!/usr/bin/env python3
"""Same-cycle validation, ordered exact command decomposition and event plots.

Decomposition terms are vectors in W, ordered as documented in decompose().
Their norms must NOT be summed as scalar causal contributions.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def vec(r, base):
    keys = {
        'u': ['mpc_cmd_v'+x for x in 'xyz'],
        'vg': ['ugv_v'+x+'_w' for x in 'xyz'],
        'w': ['ugv_omega_'+x+'_n' for x in 'xyz'],
        'p': ['p_rel_'+x+'_n' for x in 'xyz'],
        'cross': ['omega_cross_p_'+x+'_n' for x in 'xyz'],
        'comp': ['comp_rel_'+x+'_n' for x in 'xyz'],
        'rot': ['rot_rel_'+x+'_w' for x in 'xyz'],
        'before': ['world_before_'+x for x in 'xyz'],
        'after': ['world_after_'+x for x in 'xyz'],
        'published': ['published_v'+x for x in 'xyz'],
        'raw_w': ['raw_ugv_omega_'+x for x in 'xyz'],
        'raw_v': ['raw_ugv_v'+x for x in 'xyz'],
    }
    return np.array([float(r[k]) for k in keys[base]])


def rotation(r):
    return np.array([[float(r[f'R{i}{j}']) for j in range(3)] for i in range(3)])


def decompose(a, b):
    """Δvg+RaΔu+Ra(Δw×pa)+Ra(wb×Δp)+(Rb−Ra)(ub+wb×pb)+Δclip.

    Exactly telescopes to Δpublished on the non-inertial, successful MPC path.
    Position/orientation coupling is assigned by the stated order, not uniquely.
    """
    Ra, Rb = rotation(a), rotation(b)
    ua, ub, wa, wb, pa, pb = (vec(a,'u'),vec(b,'u'),vec(a,'w'),vec(b,'w'),
                             vec(a,'p'),vec(b,'p'))
    terms = {
        'ugv_velocity': vec(b,'vg')-vec(a,'vg'),
        'mpc_output': Ra @ (ub-ua),
        'omega_change': Ra @ np.cross(wb-wa,pa),
        'position_change': Ra @ np.cross(wb,pb-pa),
        'rotation_change': (Rb-Ra) @ (ub+np.cross(wb,pb)),
        'limit_change': (vec(b,'after')-vec(b,'before'))-(vec(a,'after')-vec(a,'before')),
        'postprocess_change': (vec(b,'published')-vec(b,'after'))-(vec(a,'published')-vec(a,'after')),
    }
    return terms


def stats(values):
    a = np.asarray(values)
    return {'mean': float(a.mean()), 'p95': float(np.quantile(a,.95)), 'max':float(a.max())}


def write_csv(path, rows):
    if not rows: return
    with path.open('w', newline='') as out:
        w = csv.DictWriter(out, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('provenance',type=Path)
    ap.add_argument('--output-dir',type=Path,required=True)
    ap.add_argument('--steps',nargs='*',type=int,default=[322,330,1314])
    ap.add_argument('--threshold',type=float,default=.5)
    args=ap.parse_args(); args.output_dir.mkdir(parents=True,exist_ok=True)
    rows=list(csv.DictReader(args.provenance.open()))
    if not rows: raise ValueError('empty provenance')
    summary={'rows':len(rows), 'decomposition_definition':decompose.__doc__, 'events':[]}
    groups={}
    for r in rows: groups.setdefault(int(r['uav_idx']),[]).append(r)
    deltas=[]; windows=[]; max_closure=0.0
    for qid, group in groups.items():
        assert [int(r['cycle_id']) for r in group] == list(range(len(group))), 'cycle gap'
        for a,b in zip(group,group[1:]):
            d={'uav_idx':qid,'cycle_id':int(b['cycle_id']),'sim_time':float(b['sim_time'])}
            for label in ('u','vg','w','cross','comp','rot','before','after','published'):
                v=vec(b,label)-vec(a,label)
                for i,x in enumerate('xyz'): d[f'delta_{label}_{x}']=float(v[i])
                d[f'delta_{label}_norm']=float(np.linalg.norm(v))
            terms=decompose(a,b); v=vec(b,'published')-vec(a,'published')
            closure=float(np.max(np.abs(sum(terms.values())-v)));max_closure=max(max_closure,closure)
            d['closure_inf']=closure
            for name,t in terms.items():
                for i,x in enumerate('xyz'):d[f'contribution_{name}_{x}']=float(t[i])
                d[f'contribution_{name}_norm']=float(np.linalg.norm(t))
            # Noise contribution is algebraic/conditional, not a rerun of the closed loop.
            angular_noise_delta=(vec(b,'w')-vec(b,'raw_w'))-(vec(a,'w')-vec(a,'raw_w'))
            angular_noise_term=rotation(a)@np.cross(angular_noise_delta,vec(a,'p'))
            d['angular_noise_term_norm']=float(np.linalg.norm(angular_noise_term))
            d['angular_noise_projection_fraction']=float(angular_noise_term@v/(v@v)) if v@v>0 else 0.0
            d['raw_omega_delta_norm']=float(np.linalg.norm(vec(b,'raw_w')-vec(a,'raw_w')))
            d['auto_event']=int(d['delta_published_norm']>args.threshold)
            d['stamp_offset_us']=(float(b['ugv_stamp_sec'])-float(b['state_stamp_sec']))*1e6
            deltas.append(d)
        auto=[d for d in deltas if d['uav_idx']==qid and d['auto_event']]
        targets=set(args.steps)|{d['cycle_id'] for d in sorted(auto,key=lambda d:d['delta_published_norm'],reverse=True)[:5]}
        for r in group:
            if any(abs(int(r['cycle_id'])-k)<=1 for k in targets): windows.append(r)
        summary['events'] += [d for d in deltas if d['uav_idx']==qid and d['cycle_id'] in targets]
    def err(f): return float(max(f(r) for r in rows))
    summary.update({
        'max_decomposition_closure_inf':max_closure,
        'auto_event_count':sum(d['auto_event'] for d in deltas),
        'max_compensation_identity_error':err(lambda r:np.max(np.abs(vec(r,'comp')-vec(r,'u')-vec(r,'cross')))),
        'max_world_identity_error':err(lambda r:np.max(np.abs(vec(r,'before')-vec(r,'vg')-rotation(r)@vec(r,'comp')))),
        'max_publish_error':err(lambda r:np.max(np.abs(vec(r,'published')-vec(r,'after')))),
        'max_expand_identity_error':err(lambda r:max(abs(float(r[f'acado_u{i}'])-float(r[f'qp_nominal{i}'])-float(r[f'qp_increment{i}'])) for i in range(4))),
        'max_solve_mapping_position_error':err(lambda r:max(abs(float(r[f'quad_p{x}_w'])-float(r[f'solve_quad_p{x}_w'])) for x in 'xyz')),
        'max_profile_mapping_omega_error':err(lambda r:max(abs(float(r[f'ugv_omega_{x}_n'])-float(r[f'omega_profile0_{x}'])) for x in 'xyz')),
        'stamp_offset_us':stats([abs(float(r['ugv_stamp_sec'])-float(r['state_stamp_sec']))*1e6 for r in rows]),
    })
    timing=list(csv.DictReader(Path(str(args.provenance)+'.timing.csv').open()))
    summary['collection_ms']=stats([float(r['collection_ms']) for r in timing])
    summary['serialization_ms']=stats([float(r['serialize_ms']) for r in timing])
    summary['node_logging_ms']=stats([float(r['collection_ms'])+float(r['serialize_ms']) for r in timing])
    write_csv(args.output_dir/'world_command_all_deltas.csv',deltas)
    write_csv(args.output_dir/'world_command_event_comparison.csv',summary['events'])
    write_csv(args.output_dir/'world_command_event_windows.csv',windows)
    # Keep complete JSON rows as an inspectable alternative to CSV.
    (args.output_dir/'world_command_provenance.json').write_text(json.dumps(rows,indent=2))
    (args.output_dir/'world_command_provenance_summary.json').write_text(json.dumps(summary,indent=2))
    fig,axes=plt.subplots(2,1,figsize=(11,7),constrained_layout=True)
    for label in ('u','cross','before','published'):
        axes[0].plot([d['sim_time'] for d in deltas],[d[f'delta_{label}_norm'] for d in deltas],label=label,linewidth=.8)
    axes[0].set(xlabel='simulation time (s)',ylabel='adjacent command change (m/s)');axes[0].legend();axes[0].grid(alpha=.2)
    focus=[d for d in deltas if abs(d['sim_time']-26.28)<=.16]
    for label in ('u','cross','before','published'):
        axes[1].plot([d['sim_time'] for d in focus],[d[f'delta_{label}_norm'] for d in focus],label=label,marker='.')
    axes[1].set(xlabel='simulation time (s)',ylabel='change near 26.28 s (m/s)');axes[1].legend();axes[1].grid(alpha=.2)
    fig.savefig(args.output_dir/'world_command_provenance.png',dpi=160);plt.close(fig)
    print(json.dumps({k:v for k,v in summary.items() if k!='events'},indent=2))

if __name__=='__main__': main()
