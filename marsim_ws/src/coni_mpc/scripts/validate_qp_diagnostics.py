#!/usr/bin/env python3
"""Archive-level QP assertions; per-element A-E deltas with no solver mutations."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from qp_numeric_audit import load_capture, qp_residuals


def check(r):
    m=qp_residuals(r)
    assert m['status']==0
    assert m['primal_inf']<1e-7
    assert m['stationarity_inf']<1e-7
    assert m['dual_sign_inf']<1e-7
    assert m['complementarity_inf']<1e-7
    H=np.asarray(r['H']).reshape(140,140)
    assert np.max(np.abs(H-H.T))<1e-9
    np.linalg.cholesky(H)  # independently verify positive definiteness
    return m


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('original',type=Path)
    ap.add_argument('counterfactual',type=Path)
    ap.add_argument('labels',type=Path)
    ap.add_argument('--output-dir',type=Path,required=True)
    args=ap.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    original=load_capture(args.original);cf=load_capture(args.counterfactual)
    labels=list(csv.DictReader(args.labels.open()))
    assert len(original)==1500 and len(cf)==len(labels)==15
    original_metrics=[check(r) for r in original]
    variants=[];differences=[]
    for i,(r,label) in enumerate(zip(cf,labels)):
        assert int(r['index'])==i
        m=check(r);m.update({'step':int(label['step']),'variant':label['variant']})
        assert abs(m['objective_recorded']-float(label['objective']))<1e-9
        base=cf[i-i%5];source=original[int(label['step'])]
        if label['variant']=='A_original':
            for field in ('H','g','A','lb','ub','lbA','ubA','solution','dual'):
                assert np.array_equal(r[field],source[field]), f'A differs from original {field}'
        for field in ('H','g','A','lb','ub','lbA','ubA'):
            d=np.asarray(r[field])-np.asarray(base[field]);changed=np.flatnonzero(abs(d)>1e-12)
            if label['variant'] in ('B_fixed_snapshot','C_fixed_slice','D_fixed_ids_current_state'):
                if field in ('H','g','lb','ub'):assert len(changed)==0,field
                if field=='A':assert all((int(j)//140)%10==9 for j in changed)
                if field in ('lbA','ubA'):assert all(int(j)%10==9 for j in changed)
            for j in changed:
                row,col=(int(j)//140,int(j)%140) if field in ('H','A') else (int(j),-1)
                differences.append({'step':label['step'],'variant':label['variant'],'array':field,
                    'row':row,'col':col,'A_value':float(base[field][j]),'variant_value':float(r[field][j]),
                    'delta':float(d[j]), 'path_stage':row//10 if field in ('A','lbA','ubA') else -1,
                    'local_path_row':row%10 if field in ('A','lbA','ubA') else -1})
        # 26.28 B-D unchanged primal/dual despite modified rows/bounds.
        if int(label['step'])==1314 and label['variant'] in ('B_fixed_snapshot','C_fixed_slice','D_fixed_ids_current_state'):
            assert np.array_equal(r['solution'],base['solution'])
            assert np.array_equal(r['dual'],base['dual'])
        variants.append(m)
    for name,rows in [('counterfactual_numeric_audit.csv',variants),('matrix_element_differences.csv',differences)]:
        with (args.output_dir/name).open('w',newline='') as out:
            writer=csv.DictWriter(out,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    summary={'original_count':len(original),'variant_count':len(cf),'changed_elements':len(differences),
             'all_H_cholesky_success':True, 'all_residuals_pass':True,
             'cf_max':{k:max(m[k] for m in variants) for k in ['primal_inf','stationarity_inf','dual_sign_inf','complementarity_inf']}}
    (args.output_dir/'validation.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
