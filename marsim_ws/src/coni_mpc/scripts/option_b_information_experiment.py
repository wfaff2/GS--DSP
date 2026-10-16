#!/usr/bin/env python3
"""Offline position-moment information audit, independent of ROS/runtime.

Float experiments evaluate analytically proved continuum upper bounds, not the
moment optimum. Selected paired queries additionally use exact Fraction moment
accumulation, outward rational square-root brackets, and exact polynomial/bin
weights. No risk evaluator accepts original particles.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from fractions import Fraction as F
import hashlib
import json
import math
import pathlib
import platform
import subprocess
import time

import numpy as np
from scipy.optimize import linprog

from stage3c_cube_support_experiment import Particle, support, node, project, original_risk, kernel
from stage3e_directional_experiment import matched_scenarios


def rational_up(x, bits=80):
    """Exact directed rounding to a dyadic rational, controlling denominator growth."""
    scale = 1 << bits
    return F(-((-x.numerator*scale)//x.denominator), scale)


def rational_down(x, bits=80):
    scale = 1 << bits
    return F((x.numerator*scale)//x.denominator, scale)


@dataclass
class Moments:
    centers: np.ndarray
    mass: np.ndarray
    first: np.ndarray
    second: np.ndarray


def project_moments(particles, delta):
    """Only preprocessing sees particles. Raw relative-position moments."""
    nodes = {}
    for p in particles:
        if p.weight < 0 or not np.all(np.isfinite(p.position)):
            raise ValueError("nonnegative finite particle inputs required")
        if p.weight == 0:
            continue
        for idx, N in support(p.position, delta):
            z = p.position-node(idx, delta)
            m = p.weight*N
            old = nodes.setdefault(idx, [0., np.zeros(3), np.zeros((3, 3))])
            old[0] += m
            old[1] += m*z
            old[2] += m*np.outer(z, z)
    if not nodes:
        return Moments(np.empty((0, 3)), np.empty(0), np.empty((0, 3)), np.empty((0, 3, 3)))
    return Moments(delta*np.array(list(nodes), float), np.array([v[0] for v in nodes.values()]),
                   np.array([v[1] for v in nodes.values()]), np.array([v[2] for v in nodes.values()]))


def phi_batch(d, radius, width):
    d = np.asarray(d)
    z = np.clip((radius+width-d)/width, 0, 1)
    return z**3*(6*z**2-15*z+10)


def bounds(stats, query, delta, radius=1., width=.5, bins=32):
    """Continuum analytic certificates, ordinary double reference evaluation.

    C: mass-only optimum; first: support/Markov layer certificate; full: minimum
    of first, covariance/Cantelli layer certificate, positive Taylor, and C.
    These are not reported as the exact moment-problem optimum.
    """
    if delta <= 0 or radius <= 0 or width <= 0 or bins < 1:
        raise ValueError("positive geometry/kernel/bins required")
    if np.any(stats.mass <= 0):
        raise ValueError("zero mass nodes must have zero moments and be handled before normalization")
    m = stats.mass
    mu = stats.first/m[:, None]
    cov = stats.second/m[:, None, None]-mu[:, :, None]*mu[:, None, :]
    cov = (cov+cov.transpose(0, 2, 1))/2
    y = np.asarray(query)-stats.centers
    diff = y-mu
    D = np.linalg.norm(diff, axis=1)
    v = np.divide(diff, D[:, None], out=np.zeros_like(diff), where=D[:, None] > 0)
    var = np.maximum(np.einsum("ni,nij,nj->n", v, cov, v), 0)
    trace = np.maximum(np.trace(cov, axis1=1, axis2=2), 0)
    cube_d = np.linalg.norm(np.maximum(np.abs(y)-2*delta, 0), axis=1)
    cube = phi_batch(cube_d, radius, width)
    s = np.linspace(radius, radius+width, bins+1)
    # Exact-in-real-arithmetic bin weights of the smootherstep layer density.
    weights = phi_batch(s[:-1], radius, width)-phi_batch(s[1:], radius, width)
    gap = D[:, None]-s[None, 1:]
    tail2 = np.ones_like(gap)
    good = gap > 0
    numerator = np.broadcast_to(var[:, None], gap.shape)
    tail2[good] = numerator[good]/(numerator[good]+gap[good]**2)
    L = -2*delta*np.sum(np.abs(v), axis=1)
    H = -L
    mean = np.einsum("ni,ni->n", v, mu)
    k = np.einsum("ni,ni->n", v, y)[:, None]-s[None, 1:]
    tail1 = np.ones_like(k)
    good = k > L[:, None]
    numer = np.broadcast_to((mean-L)[:, None], k.shape)
    denom = k-L[:, None]
    tail1[good] = np.minimum(1., np.maximum(0., numer[good]/denom[good]))
    tail1[k > H[:, None]] = 0.
    first = np.minimum(cube, tail1@weights)
    cantelli = tail2@weights
    # One-sided radial Hessian upper bound; tangential eigenvalues <=0.
    L2 = 10/(math.sqrt(3)*width**2)
    taylor = phi_batch(D, radius, width)+.5*L2*trace
    full = np.minimum.reduce([cube, first, cantelli, taylor])
    # Float clipping is diagnostic-only; exact Fraction path does not clip moments.
    return {"C": float(m@cube), "first": float(m@first), "full": float(m@full),
            "cantelli_only_capped": float(m@np.minimum(cube, cantelli)),
            "taylor_only_capped": float(m@np.minimum(cube, taylor)),
            "nodes": len(m), "cov_min_eigenvalue": float(np.linalg.eigvalsh(cov).min()) if len(m) else 0.,
            "per_node": {"C": cube, "first": first, "full": full}}


def sqrt_bracket(x, bits=80):
    """Exact integer square root gives outward dyadic brackets without libm."""
    if x < 0:
        raise ValueError("negative square-root argument")
    if x == 0:
        return F(0), F(0)
    scale = 1 << bits
    k = math.isqrt((x.numerator*scale*scale)//x.denominator)
    lo = F(k, scale)
    hi = lo if lo*lo == x else F(k+1, scale)
    return lo, hi


def phi_fraction(d, radius, width):
    if d <= radius:
        return F(1)
    if d >= radius+width:
        return F(0)
    z = (radius+width-d)/width
    return z**3*(6*z*z-15*z+10)


def exact_project(particles, delta):
    """Exact cubic weights for the exact binary-rational particle inputs."""
    delta = F(float(delta))
    result = {}
    for p in particles:
        pos = tuple(F(float(v)) for v in p.position)
        weight = F(float(p.weight))
        if weight < 0:
            raise ValueError("negative mass")
        if weight == 0:
            continue
        loc = [x/delta for x in pos]
        base = [x.numerator//x.denominator for x in loc]
        u = [loc[k]-base[k] for k in range(3)]
        basis = [[(1-x)**3/6, (3*x**3-6*x*x+4)/6,
                  (-3*x**3+3*x*x+3*x+1)/6, x**3/6] for x in u]
        for ijk in np.ndindex(4, 4, 4):
            m = weight*math.prod(basis[k][ijk[k]] for k in range(3))
            if not m:
                continue
            idx = tuple(base[k]+ijk[k]-1 for k in range(3))
            z = tuple(pos[k]-delta*idx[k] for k in range(3))
            old = result.setdefault(idx, [F(0), [F(0)]*3, [[F(0)]*3 for _ in range(3)]])
            old[0] += m
            for k in range(3):
                old[1][k] += m*z[k]
                for l in range(3):
                    old[2][k][l] += m*z[k]*z[l]
    return result


def exact_node_bounds(moment, y, a, radius, width, bins=32):
    """Genuine continuum upper certificate via Fraction arithmetic.

    For any rational v (not necessarily unit), event ||y-z||<=s implies
    v·z >= v·y-||v||s. An outward sqrt upper makes a weaker, valid event.
    """
    m, ell, Q = moment
    if m == 0:
        if any(ell) or any(x for row in Q for x in row):
            raise ValueError("infeasible zero-mass moments")
        return {k: F(0) for k in ("C_lower", "C_upper", "first_upper", "full_upper")}
    if m < 0:
        raise ValueError("negative mass")
    mu = [x/m for x in ell]
    cov = [[Q[k][l]/m-mu[k]*mu[l] for l in range(3)] for k in range(3)]
    trace = sum(cov[k][k] for k in range(3))
    if trace < 0:
        raise ValueError("infeasible covariance")
    dC2 = sum(max(abs(y[k])-a, F(0))**2 for k in range(3))
    dc_lo, dc_hi = sqrt_bracket(dC2)
    clo, chi = phi_fraction(dc_hi, radius, width), phi_fraction(dc_lo, radius, width)
    if chi == 0:
        return {k: F(0) for k in ("C_lower", "C_upper", "first_upper", "full_upper")}
    v = [y[k]-mu[k] for k in range(3)]
    D2 = sum(x*x for x in v)
    Dlo, Dhi = sqrt_bracket(D2)
    mean = sum(v[k]*mu[k] for k in range(3))
    vy = sum(v[k]*y[k] for k in range(3))
    L, H = -a*sum(abs(x) for x in v), a*sum(abs(x) for x in v)
    var = sum(v[k]*cov[k][l]*v[l] for k in range(3) for l in range(3))
    if var < 0:
        raise ValueError("infeasible projected variance")
    first = second = F(0)
    previous = F(1)
    for b in range(1, bins+1):
        s = radius+width*F(b, bins)
        current = phi_fraction(s, radius, width)
        layer = previous-current
        previous = current
        threshold = vy-Dhi*s
        if threshold > H:
            t1 = F(0)
        elif threshold <= L:
            t1 = F(1)
        else:
            t1 = min(F(1), (mean-L)/(threshold-L))
        gap = threshold-mean
        t2 = F(1) if gap <= 0 else var/(var+gap*gap)
        first = rational_up(first+layer*t1)
        second = rational_up(second+layer*t2)
    sqrt3lo, _ = sqrt_bracket(F(3))
    L2upper = F(10)/(sqrt3lo*width*width)
    taylor = phi_fraction(Dlo, radius, width)+L2upper*trace/2
    first = min(chi, first)
    full = min(chi, first, second, taylor)
    return {"C_lower": rational_down(m*clo), "C_upper": min(m, rational_up(m*chi)),
            "first_upper": min(m, rational_up(m*first)), "full_upper": min(m, rational_up(m*full))}


def exact_bounds(nodes, query, delta, radius=1., width=.5, bins=32):
    """Input is moments only. Totals and certified gaps returned as rationals."""
    delta, radius, width = F(float(delta)), F(float(radius)), F(float(width))
    q = [F(float(v)) for v in query]
    total = {k: F(0) for k in ("C_lower", "C_upper", "first_upper", "full_upper")}
    for idx, moment in nodes.items():
        y = [q[k]-delta*idx[k] for k in range(3)]
        row = exact_node_bounds(moment, y, 2*delta, radius, width, bins)
        for k in total:
            total[k] += row[k]
    return total


def feature(points):
    x = np.asarray(points)
    return np.column_stack([np.ones(len(x)), x, x[:, 0]**2, x[:, 1]**2, x[:, 2]**2,
                            x[:, 0]*x[:, 1], x[:, 0]*x[:, 2], x[:, 1]*x[:, 2]])


def restricted_lp(points, first, second, query, radius=.2, width=.2):
    """Fixed-atom LP, numerical restricted problem only; NOT an upper bound."""
    b = np.r_[1., first, np.diag(second), second[0, 1], second[0, 2], second[1, 2]]
    A = feature(points).T
    values = phi_batch(np.linalg.norm(points-query, axis=1), radius, width)
    sol = linprog(-values, A_eq=A, b_eq=b, bounds=(0, None), method="highs")
    return {"success": bool(sol.success), "status": int(sol.status),
            "restricted_objective": float(-sol.fun) if sol.success else None,
            "moment_residual": float(np.max(np.abs(A@sol.x-b))) if sol.success else None,
            "classification": "numerical restricted optimum; no continuum upper certification"}


def theoretical_examples():
    a = 1.
    corners = np.array(list(np.ndindex(2, 2, 2)), float)*2-1
    parity = np.prod(corners, axis=1)
    rc, width = .5, 1.
    def value(x):
        f = phi_batch(np.linalg.norm(corners-[x, a, a], axis=1), rc, width)
        return max(np.dot((1+parity)/8, f), np.dot((1-parity)/8, f))
    eps = 1e-6
    left, right = (value(0)-value(-eps))/eps, (value(eps)-value(0))/eps
    corr = np.eye(3)*1.5-np.ones((3, 3))*.5
    # Interior measures with equal mass/mean/covariance but different risks.
    ambiguity_A = kernel(.5, .2, .2)
    ambiguity_B = (1-.25/.8**2)*kernel(0, .2, .2)+(.25/.8**2)*kernel(.8, .2, .2)
    lp_points = np.array([[x, 0., 0.] for x in (-.8, -.5, 0., .5, .8)])
    lp = restricted_lp(lp_points, np.zeros(3), np.diag([.25, 0., 0.]), np.zeros(3))
    grid = np.array([[x, 0., 0.] for x in (-1., -.5, 0., .5, 1.)])
    mu = np.array([.37, 0., 0.])
    infeasible_grid = restricted_lp(grid, mu, np.outer(mu, mu), mu)
    return {"cusp": {"mass": 1., "first": [0., 0., 0.], "second": np.eye(3).tolist(),
            "radius": rc, "width": width, "query": [0., 1., 1.], "value": value(0),
            "left_slope": left, "right_slope": right, "expected_slope_magnitude": .46875,
            "closed_support_boundary_example": True},
            "infeasible_psd_example": {"second": corr.tolist(), "eigenvalues": np.linalg.eigvalsh(corr).tolist(),
                "required_E_sum_square_at_least": 1., "proposed_E_sum_square": float(corr.sum())},
            "same_interior_moments": {"mean": 0., "variance": .25, "risk_A": ambiguity_A, "risk_B": ambiguity_B,
                                      "numerical_restricted_LP": lp,
                                      "known_continuum_feasible_risk": .75,
                                      "continuum_witness_atoms": [[-1., .125], [0., .75], [1., .125]]},
            "zero_variance_off_grid_LP": infeasible_grid}


def same_grid_information_examples():
    """Exact finite differences inside one cell show extra stats are independent.

    Cubic P2G masses see degree<=3 in x; node first moments see degree<=4;
    second moments see degree<=5. Fixed y=z=.5 supplies the tensor factors.
    """
    output = {}
    examples = (("same_mass_different_first", [1, -4, 6, -4, 1], F(1, 8), F(3, 16), F(1, 2)),
                ("same_mass_and_first_different_second", [1, -5, 10, -10, 5, -1], F(1, 8), F(1, 8), F(1, 2)))
    for name, coeff, start, step, query_x in examples:
        norm = sum(c for c in coeff if c > 0)
        pa = [Particle(np.array([float(start+i*step), .5, .5]), c/norm) for i, c in enumerate(coeff) if c > 0]
        pb = [Particle(np.array([float(start+i*step), .5, .5]), -c/norm) for i, c in enumerate(coeff) if c < 0]
        na, nb = exact_project(pa, 1.), exact_project(pb, 1.)
        self_same_mass = set(na) == set(nb) and all(na[k][0] == nb[k][0] for k in na)
        same_first = all(na[k][1] == nb[k][1] for k in na)
        first_delta = max(abs(na[k][1][j]-nb[k][1][j]) for k in na for j in range(3))
        second_delta = max(abs(na[k][2][j][l]-nb[k][2][j][l]) for k in na for j in range(3) for l in range(3))
        q = np.array([float(query_x), .5, .5])
        output[name] = {"same_all_P2G_node_masses_exact": self_same_mass,
            "same_all_node_first_moments_exact": same_first,
            "max_first_difference_exact": rational_string(first_delta),
            "max_second_difference_exact": rational_string(second_delta),
            "risk_A": original_risk(pa, q, .05, .05), "risk_B": original_risk(pb, q, .05, .05),
            "kernel_radius": .05, "kernel_width": .05, "query": q.tolist(),
            "particles_A": [[*p.position.tolist(), p.weight] for p in pa],
            "particles_B": [[*p.position.tolist(), p.weight] for p in pb]}
    return output


def paired(cases, delta=.5, radius=1., width=.5, bins=32):
    output = {}
    for name, (particles, queries) in cases.items():
        stats = project_moments(particles, delta)
        rows = []
        mincov = math.inf
        for q in queries:
            row = bounds(stats, q, delta, radius, width, bins)
            mincov = min(mincov, row["cov_min_eigenvalue"])
            rows.append([original_risk(particles, q, radius, width), row["C"], row["first"], row["full"]])
        values = np.array(rows)
        excess = values[:, 1:]-values[:, :1]
        output[name] = {"particles": len(particles), "nodes": len(stats.mass), "queries": len(queries),
            "values_columns": ["smooth", "C", "first_upper_float", "full_upper_float"], "values": values.tolist(),
            "mean_excess": dict(zip(("C", "first", "full"), map(float, excess.mean(axis=0)))),
            "min_full_minus_original": float(excess[:, 2].min()),
            "mean_C_minus_first": float(np.mean(values[:, 1]-values[:, 2])),
            "mean_first_minus_full": float(np.mean(values[:, 2]-values[:, 3])),
            "max_C_minus_full": float(np.max(values[:, 1]-values[:, 3])),
            "min_raw_cov_eigenvalue_float": mincov,
            "raw_mass_total": float(stats.mass.sum())}
    return output


def rational_string(x):
    return f"{x.numerator}/{x.denominator}"


def exact_queries(cases, delta=.5, radius=1., width=.5, bins=32):
    rows = []
    for name, (p, qs) in cases.items():
        nodes = exact_project(p, delta)
        # Predetermined equally spaced query indices, never selected for improvement.
        indices = sorted(set(np.linspace(0, len(qs)-1, 4, dtype=int)))
        for i in indices:
            start = time.perf_counter()
            exact = exact_bounds(nodes, qs[i], delta, radius, width, bins)
            elapsed = 1e3*(time.perf_counter()-start)
            gap = exact["C_lower"]-exact["full_upper"]
            rows.append({"scenario": name, "query_index": int(i), "query": qs[i].tolist(),
                "values_float_display": {k: float(v) for k, v in exact.items()},
                "rigorous_C_minus_full_lower": rational_string(gap),
                "strict_tightening_certified": gap > 0,
                "exact_fraction_identity_sha256": hashlib.sha256(
                    json.dumps({k: rational_string(v) for k, v in exact.items()}, sort_keys=True).encode()).hexdigest(),
                "elapsed_ms": elapsed})
    return rows


def timing(cases):
    output = {}
    for name, (p, qs) in cases.items():
        def median_ms(fn):
            fn()
            values = []
            for _ in range(5):
                t = time.perf_counter_ns()
                fn()
                values.append((time.perf_counter_ns()-t)/1e6)
            return float(np.median(values))
        stats = project_moments(p, .5)
        output[name] = {"P2G_mass_and_moments_ms": median_ms(lambda: project_moments(p, .5)),
            "P2G_mass_only_ms": median_ms(lambda: project(p, .5)),
            "query_float_bounds_ms": median_ms(lambda: bounds(stats, qs[0], .5)),
            "node_count": len(stats.mass), "bins": 32, "repeats": 5}
    return output


def report():
    cases = matched_scenarios()
    paired_result = paired(cases)
    extra = {}
    p, q = cases["dense"]
    for width in (.12, .5, 1.):
        extra[str(width)] = paired({"dense": (p, q[::8])}, radius=.48, width=width)["dense"]
    sensitivity = {}
    for bins in (8, 16, 32, 64):
        sensitivity[str(bins)] = paired({"dense": (p, q[::16])}, bins=bins)["dense"]
    here = pathlib.Path(__file__).resolve()
    root = here.parents[4]
    sources = [here, here.with_name("stage3c_cube_support_experiment.py"),
               here.with_name("stage3e_directional_experiment.py"),
               here.parents[1]/"include/coni_mpc/dsp_particle_to_grid.h",
               here.parents[1]/"src/dsp_particle_to_grid.cpp",
               here.parents[1]/"include/coni_mpc/field_hocbf.h",
               here.parents[1]/"parameters/num_sim_non_one_point.yaml"]
    return {"metadata": {"audit_date": "2026-10-11", "seed": 20261011,
        "commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
        "git_dirty_state": subprocess.check_output(["git", "status", "--short"], cwd=root, text=True).splitlines(),
        "python": platform.python_version(), "numpy": np.__version__, "platform": platform.platform(),
        "source_sha256": {str(f.relative_to(root)): hashlib.sha256(f.read_bytes()).hexdigest() for f in sources},
        "matched_parameters": {"delta": .5, "radius": 1., "width": .5, "bins": 32},
        "float_results_machine_certified": False, "selected_Fraction_certificates": True,
        "rational_outward_rounding_bits": 80,
        "optimal_moment_problem_solved_in_general": False, "live_PHD_threshold": None},
        "inputs": {n: {"particles": [{"p": pp.position.tolist(), "weight": pp.weight} for pp in ps],
                       "queries": qs.tolist()} for n, (ps, qs) in cases.items()},
        "paired": paired_result, "radius_0.48_width_scan": extra,
        "bin_refinement": sensitivity, "exact_selected_query_certificates": exact_queries(cases),
        "counterexamples": theoretical_examples(), "same_grid_information_examples": same_grid_information_examples(),
        "timings": timing(cases)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    payload = json.dumps(report(), indent=2, allow_nan=False)
    if args.output:
        args.output.write_text(payload+"\n")
        print(f"Saved {args.output}")
    else:
        print(payload)


if __name__ == "__main__":
    main()
