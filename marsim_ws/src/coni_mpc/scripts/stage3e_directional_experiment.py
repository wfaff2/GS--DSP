#!/usr/bin/env python3
"""Offline Stage 3E audit. No ROS imports, runtime edits, or mass cropping.

All A/C/D/E evaluations share the same vectorized full-node summation. D uses
the unchanged Stage 3D batch KKT solver, with 64 bisections. Component-only
timings avoid forming unrequested kernel outputs; full timings return all three.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import pathlib
import platform
import subprocess
import time

import numpy as np

from stage3c_cube_support_experiment import Particle, project, original_risk
from stage3d_superellipsoid_experiment import super_distance_batch, super_projection

ALPHA = math.sqrt(3) / (2 * (1 + 1 / math.sqrt(3))**2)
MODES = ("A", "C", "D", "E")
SEED = 20261011


def projection_batch(y, delta):
    """Same KKT bisection as D, retaining sensitivities without a second solve."""
    target = 3*(2*delta)**4
    outside = np.sum(y**4, axis=1) > target
    x = y.copy()
    lam = np.zeros(len(y))
    d = np.zeros(len(y))
    if not np.any(outside):
        return d, x, lam
    v = y[outside]
    lo, hi = np.zeros(len(v)), np.ones(len(v))
    def coords(l):
        root = np.sqrt(12*l)
        return 2/root[:, None]*np.sinh(np.arcsinh(1.5*root[:, None]*v)/3)
    for _ in range(64):
        need = np.sum(coords(hi)**4, axis=1) > target
        if not np.any(need):
            break
        hi[need] *= 2
    else:
        raise RuntimeError("KKT bracketing failed")
    for _ in range(64):
        mid = (lo+hi)/2
        need = np.sum(coords(mid)**4, axis=1) > target
        lo[need], hi[~need] = mid[need], mid[~need]
    lam[outside] = (lo+hi)/2
    x[outside] = coords(lam[outside])
    d[outside] = np.linalg.norm(v-x[outside], axis=1)
    return d, x, lam


def directional_distance(y, delta):
    y = np.asarray(y, dtype=float).reshape(-1, 3)
    norm = np.linalg.norm(y, axis=1)
    unit = np.divide(y, norm[:, None], out=np.zeros_like(y), where=norm[:, None] > 0)
    spread = np.sum((unit**2 - 1/3)**2, axis=1)
    bound = 2*delta * (math.sqrt(3) - ALPHA*spread)
    return np.maximum(norm - bound, 0), bound


def node_terms(y, delta, radius, width, mode="E", order=2, only=None):
    """Per-node kernel derivatives. Differentiation only in the transition.

    No node is dropped. The C Hessian is a piecewise derivative and must not
    be read as a continuous global Hessian at cube active-set switches.
    """
    if delta <= 0 or radius <= 0 or width <= 0 or order not in (0, 1, 2):
        raise ValueError("positive geometry/kernel parameters and order 0/1/2 required")
    y = np.asarray(y, dtype=float).reshape(-1, 3)
    if not np.all(np.isfinite(y)):
        raise ValueError("finite offsets required")
    norm = np.linalg.norm(y, axis=1)
    a = 2*delta
    if mode == "E":
        distance = directional_distance(y, delta)[0]
    elif mode == "A":
        distance = np.maximum(norm - a*math.sqrt(3), 0)
    elif mode == "C":
        excess = np.maximum(np.abs(y) - a, 0)
        distance = np.linalg.norm(excess, axis=1)
    elif mode == "D":
        if order == 0:
            distance = super_distance_batch(y, delta, iterations=64)
        else:
            distance, px, plam = projection_batch(y, delta)
    else:
        raise ValueError("unknown mode")
    need_value = only is None or only == "value"
    need_grad = order >= 1 and (only is None or only == "gradient")
    value = np.zeros(len(y)) if need_value else None
    if need_value:
        value[distance <= radius] = 1
    active = (distance > radius) & (distance < radius + width)
    z = (radius + width - distance[active]) / width
    if need_value:
        value[active] = z**3 * (6*z**2 - 15*z + 10)
    if order == 0:
        return value, None, None
    grad = np.zeros_like(y) if need_grad else None
    hess = np.zeros((len(y), 3, 3)) if order == 2 else None
    if not np.any(active):
        return value, grad, hess
    v = y[active]
    r = norm[active]
    u = v / r[:, None]
    eye = np.eye(3)[None, :, :]
    outer = lambda p, q: p[:, :, None] * q[:, None, :]
    if mode in ("A", "E"):
        gd = u.copy()
        if order == 2:
            hd = (eye - outer(u, u)) / r[:, None, None]
        if mode == "E":
            # Ambient derivatives of S(y)=sum(y_k^4)/||y||^4-1/3.
            T = np.sum(v**4, axis=1)
            gS = 4*v**3/r[:, None]**4 - 4*T[:, None]*v/r[:, None]**6
            gd += a*ALPHA*gS
            if order == 2:
                diag = eye * v[:, :, None]**2
                hS = (12*diag/r[:, None, None]**4
                      - 16*(outer(v**3, v)+outer(v, v**3))/r[:, None, None]**6
                      - 4*T[:, None, None]*eye/r[:, None, None]**6
                      + 24*T[:, None, None]*outer(v, v)/r[:, None, None]**8)
                hd += a*ALPHA*hS
    elif mode == "C":
        ex = np.maximum(np.abs(v)-a, 0)
        d = distance[active]
        gd = np.sign(v)*ex/d[:, None]
        if order == 2:
            hd = (eye*(ex > 0)[:, :, None]-outer(gd, gd))/d[:, None, None]
    else:
        # Recover projection sensitivities only when derivatives are nonzero.
        x = px[active]
        lam = plam[active]
        b = 4*x**3
        bn = np.linalg.norm(b, axis=1)
        gd = b/bn[:, None]
        if order == 2:
            inv = 1/(1+12*lam[:, None]*x**2)
            ib = inv*b
            denom = np.sum(b*ib, axis=1)
            P = eye*inv[:, :, None] - outer(ib, ib)/denom[:, None, None]
            db = 12*x[:, :, None]**2*P
            hd = np.matmul(eye-outer(gd, gd), db)/bn[:, None, None]
            hd = (hd+hd.transpose(0, 2, 1))/2
    first = -30*z**2*(1-z)**2/width
    if need_grad:
        grad[active] = first[:, None]*gd
    if order == 2:
        second = 60*z*(1-z)*(1-2*z)/width**2
        hess[active] = second[:, None, None]*outer(gd, gd)+first[:, None, None]*hd
    return value, grad, hess


def arrays(nodes, delta):
    return delta*np.array(list(nodes), dtype=float), np.array(list(nodes.values()))


def evaluate(centers, mass, query, delta, radius, width, mode="E", order=2, only=None):
    if np.any(mass < 0) or not np.all(np.isfinite(mass)):
        raise ValueError("finite nonnegative masses required")
    k, g, h = node_terms(np.asarray(query)-centers, delta, radius, width, mode, order, only)
    return (None if k is None else float(mass@k), None if g is None else mass@g,
            None if h is None else np.einsum("n,nij->ij", mass, h))


def matched_scenarios():
    """Exact Stage 3D inequality_scan seed, generation order and query set."""
    rng = np.random.default_rng(SEED)
    cases = {
        "single": [Particle(np.array([.37, .23, -.19]), 1.)],
        "sparse": [Particle(rng.uniform(-4, 4, 3), float(rng.uniform(.05, 1))) for _ in range(6)],
        "dense": [Particle(rng.normal(0, .7, 3), float(rng.uniform(.05, 1))) for _ in range(40)],
    }
    result = {}
    for name, particles in cases.items():
        queries = [p.position.copy() for p in particles]
        queries += [p.position+rng.normal(0, 1, 3) for p in particles for _ in range(3)]
        result[name] = particles, np.array(queries)
    return result


def geometry_audit():
    rng = np.random.default_rng(SEED)
    u = rng.normal(size=(20000, 3))
    u /= np.linalg.norm(u, axis=1)[:, None]
    u = np.vstack([u, np.eye(3), np.ones(3)/math.sqrt(3), [1/math.sqrt(2), 1/math.sqrt(2), 0]])
    y = u*rng.uniform(0, 8, len(u))[:, None]
    de, b = directional_distance(y, .5)
    dc = np.linalg.norm(np.maximum(np.abs(y)-1, 0), axis=1)
    da = np.maximum(np.linalg.norm(y, axis=1)-math.sqrt(3), 0)
    return {"directions": len(u), "alpha": ALPHA,
            "min_b_minus_cube_support": float(np.min(b-np.sum(np.abs(u), axis=1))),
            "min_sphere_radius_minus_b": float(np.min(math.sqrt(3)-b)),
            "min_E_minus_A_distance": float(np.min(de-da)),
            "min_C_minus_E_distance": float(np.min(dc-de))}


def gap_metrics(particles, queries, delta, radius, width):
    nodes, _ = project(particles, delta)
    centers, mass = arrays(nodes, delta)
    rows = []
    for q in queries:
        smooth = original_risk(particles, q, radius, width)
        hard = sum(p.weight for p in particles if np.linalg.norm(q-p.position) <= radius)
        vals = [evaluate(centers, mass, q, delta, radius, width, mode, 0)[0] for mode in MODES]
        rows.append([hard, smooth, *vals])
    rows = np.array(rows)
    # columns hard, smooth, A, C, D, E
    gaps = {"smooth-hard": rows[:, 1]-rows[:, 0], "C-smooth": rows[:, 3]-rows[:, 1],
            "E-C": rows[:, 5]-rows[:, 3], "A-E": rows[:, 2]-rows[:, 5],
            "D-C": rows[:, 4]-rows[:, 3], "A-D": rows[:, 2]-rows[:, 4],
            "E-D": rows[:, 5]-rows[:, 4]}
    return {"nodes": len(nodes), "queries": len(queries),
            "gaps": {k: {"min": float(v.min()), "max": float(v.max()), "mean": float(v.mean())}
                     for k, v in gaps.items()},
            "mean_excess": {m: float(np.mean(rows[:, 2+i]-rows[:, 1])) for i, m in enumerate(MODES)},
            "values_columns": ["hard", "smooth", *MODES], "values": rows.tolist()}


def derivative_audit():
    rng = np.random.default_rng(SEED)
    directions = [np.array([1., 0, 0]), np.ones(3)/math.sqrt(3),
                  np.array([1., 1., 0])/math.sqrt(2)]
    directions += list(rng.normal(size=(30, 3)))
    gerr, herr = [], []
    joins = []
    for u in directions:
        u = u/np.linalg.norm(u)
        b = directional_distance(u[None], .5)[1][0]
        q = (b+.5+.8*.43)*u
        _, g, H = node_terms(q, .5, .5, .8)
        step = 2e-5
        fg, fh = np.zeros(3), np.zeros((3, 3))
        for k in range(3):
            e = np.eye(3)[k]*step
            plus = node_terms(q+e, .5, .5, .8)
            minus = node_terms(q-e, .5, .5, .8)
            fg[k] = (plus[0][0]-minus[0][0])/(2*step)
            fh[:, k] = (plus[1][0]-minus[1][0])/(2*step)
        gerr.append(np.linalg.norm(g[0]-fg))
        herr.append(np.linalg.norm(H[0]-fh))
        for d in (0., .5, 1.3):
            # Both sides of positive-part and both smootherstep boundaries.
            for eps in (1e-4, 1e-6):
                left = node_terms((b+d-eps)*u, .5, .5, .8)
                right = node_terms((b+d+eps)*u, .5, .5, .8)
                joins.append({"d": d, "eps": eps, "gradient_jump": float(np.linalg.norm(left[1]-right[1])),
                              "hessian_jump": float(np.linalg.norm(left[2]-right[2]))})
    return {"transition_points": len(directions), "max_gradient_error": float(max(gerr)),
            "max_hessian_error": float(max(herr)),
            "max_join_hessian_difference_eps_1e6": max(j["hessian_jump"] for j in joins if j["eps"] == 1e-6),
            "join_checks": joins}


def temporal_projection(positions, velocities, weights, delta, t):
    """Independent CV cubic P2G value/rate/acceleration, on infinite grid."""
    nodes = {}
    for p, v, w in zip(positions, velocities, weights):
        loc = (p+t*v)/delta
        base = np.floor(loc).astype(int)
        u = loc-base
        B = np.array([(1-u)**3/6, (3*u**3-6*u**2+4)/6,
                      (-3*u**3+3*u**2+3*u+1)/6, u**3/6])
        D = np.array([-.5*(1-u)**2, 1.5*u**2-2*u, -1.5*u**2+u+.5, .5*u**2])/delta
        H = np.array([1-u, 3*u-2, -3*u+1, u])/delta**2
        for index in np.ndindex(4, 4, 4):
            b = np.array([B[index[k], k] for k in range(3)])
            dd = np.array([D[index[k], k] for k in range(3)])
            hh = np.array([H[index[k], k] for k in range(3)])
            gradient = np.array([dd[k]*np.prod(np.delete(b, k)) for k in range(3)])
            Hess = np.diag([hh[k]*np.prod(np.delete(b, k)) for k in range(3)])
            for k in range(3):
                for l in range(k+1, 3):
                    Hess[k, l] = Hess[l, k] = dd[k]*dd[l]*b[3-k-l]
            terms = w*np.array([np.prod(b), gradient@v, v@Hess@v])
            key = tuple(base+np.array(index)-1)
            nodes[key] = nodes.get(key, np.zeros(3))+terms
    centers = delta*np.array(list(nodes), float)
    return centers, np.array(list(nodes.values()))


def temporal_audit():
    p = np.array([[.5, .23, -.19], [-.7, .15, .2]])
    v = np.array([[.4, -.1, .2], [.1, .3, -.2]])
    weights = np.array([1., .7])
    q = np.array([2.1, .7, .15])
    def terms(t):
        c, m = temporal_projection(p, v, weights, .5, t)
        k, g, H = node_terms(q-c, .5, .48, .5)
        return np.array([m[:, 0]@k, m[:, 1]@k, m[:, 2]@k]), m[:, 0]@g, m[:, 1]@g
    h = 1e-5
    z, grad, mixed = terms(0.)
    plus, gp, _ = terms(h)
    minus, gm, _ = terms(-h)
    # FD first derivatives rather than second value differences avoids cancellation.
    return {"time_derivative_error": float(abs(z[1]-(plus[0]-minus[0])/(2*h))),
            "time_second_derivative_error": float(abs(z[2]-(plus[1]-minus[1])/(2*h))),
            "mixed_gradient_time_error": float(np.linalg.norm(mixed-(gp-gm)/(2*h))),
            "at_cell_crossing_t": 0., "mass_derivatives": temporal_projection(p, v, weights, .5, 0.)[1].sum(axis=0).tolist()}


def cropping_audit():
    p = Particle(np.array([.2, .3, .4]), 1.)
    nodes, total = project([p], 1., dimensions=8, clip=True)
    c, m = arrays(nodes, 1.)
    clipped = evaluate(c, m, p.position, 1., .5, .5, order=0)[0]
    p2 = Particle(np.array([3.37, 3.23, 3.19]), 1.)
    full, _ = project([p2], 1.)
    nearest = min(full, key=lambda i: np.linalg.norm(np.array(i)-p2.position))
    cc, mm = arrays({nearest: full[nearest]}, 1.)
    local = evaluate(cc, mm, p2.position, 1., .5, .5, order=0)[0]
    return {"input_mass": total, "boundary_retained_mass": sum(nodes.values()),
            "lost_mass": total-sum(nodes.values()), "boundary_E": clipped,
            "smooth_at_particle": 1., "query_crop_E": local, "query_crop_nodes": 1,
            "full_node_count": len(full)}


def timed(fn, repeats=5):
    fn()  # explicit warmup
    samples = []
    for _ in range(repeats):
        start = time.perf_counter_ns()
        fn()
        samples.append((time.perf_counter_ns()-start)/1e6)
    return {"median_ms": float(np.median(samples)), "min_ms": min(samples), "repeats": repeats}


def costs(cases):
    result = {}
    for name, (p, qs) in cases.items():
        nodes, _ = project(p, .5)
        c, m = arrays(nodes, .5)
        result[name] = {"particles": len(p), "nodes": len(nodes),
                        "P2G": timed(lambda: project(p, .5)),
                        "queries_timed": 3, "evaluations": {}}
        # Same three queries, including a transition-sensitive external point.
        queries = [qs[0], np.array([2.5, 1., .2]), np.array([0., 0., 0.])]
        for mode in MODES:
            result[name]["evaluations"][mode] = {}
            for component, order, only in (("value", 0, "value"), ("gradient", 1, "gradient"),
                                            ("hessian", 2, "hessian"), ("full", 2, None)):
                fn = lambda: [evaluate(c, m, q, .5, .48, .5, mode, order, only) for q in queries]
                row = timed(fn)
                row["per_query_median_ms"] = row["median_ms"]/len(queries)
                result[name]["evaluations"][mode][component] = row
    return result


def live_boundary_geometry():
    """Singleton frozen voxel h=d_planar-.48: exact current barrier geometry.

    This illustrative geometry is not an online record or PHD/occupancy map.
    Any positive singleton occupancy cancels in the normalized live soft-min.
    """
    p = Particle(np.array([.37, .23, -.19]), 1.)
    rows = []
    for delta in (.25, .5, 1.):
        nodes, _ = project([p], delta)
        c, m = arrays(nodes, delta)
        for h in (-.1, 0., .05, .12, .25, .5, 1., 2.):
            q = p.position + np.array([.48+h, 0., 0.])
            actual_h = math.sqrt((.48+h)**2+1e-12)-.48
            vals, gradients = {}, {}
            for mode in MODES:
                k, g, _ = evaluate(c, m, q, delta, .48, .12, mode, order=1)
                vals[mode] = k
                gradients[mode] = float(np.linalg.norm(g))
            rows.append({"delta": delta, "frozen_singleton_live_h_m": actual_h,
                         "particle_smooth": original_risk([p], q, .48, .12),
                         "grid_values": vals, "grid_gradient_norms": gradients})
    return {"occupancy": "any positive singleton; normalized factor cancels",
            "mass": 1., "kernel_width_m": .12, "width_choice": "offline choice, not live field_sigma mapping",
            "rows": rows}


def level_sets(cases):
    """Sampled XY areas and rowwise crossing brackets, no monotonicity assumption.

    There is no live PHD-risk threshold. All tau values here are exploratory
    raw-mass units, distinct from live d_safe=.48 m and occupancy filter .15.
    """
    axis = np.linspace(-5., 5., 31)
    result, plotting = {}, {}
    for name, (p, _) in cases.items():
        nodes, _ = project(p, .5)
        c, m = arrays(nodes, .5)
        z = -.19 if name == "single" else 0.
        grid = np.array([[x, y, z] for y in axis for x in axis])
        vals = np.array([[original_risk(p, q, .48, .5),
                          *[evaluate(c, m, q, .5, .48, .5, mode, 0)[0] for mode in MODES]] for q in grid])
        scans = {}
        for tau in (.05, .15, .5, 1.):
            safe = vals[:, 0] <= tau
            masks = vals[:, 1:] > tau
            scans[str(tau)] = {"original_safe_samples": int(safe.sum()),
                "excluded_original_safe_samples": {mode: int(np.sum(safe&masks[:, k])) for k, mode in enumerate(MODES)},
                "unsafe_accepted_samples": {mode: int(np.sum((~safe)&(~masks[:, k]))) for k, mode in enumerate(MODES)},
                "sample_area_per_point": float((axis[1]-axis[0])**2),
                "E_vs_D_recovered_samples": int(np.sum(masks[:, 2]&(~masks[:, 3]))),
                "D_vs_E_recovered_samples": int(np.sum(masks[:, 3]&(~masks[:, 2])))}
        # All horizontal crossing brackets on y=0 at tau=.15, including multiple components.
        line = vals.reshape(31, 31, 5)[15]
        brackets = {}
        for k, mode in enumerate(("smooth", *MODES)):
            inside = line[:, k] > .15
            brackets[mode] = [[float(axis[i]), float(axis[i+1])] for i in range(30) if inside[i] != inside[i+1]]
        result[name] = {"z": z, "grid_shape": [31, 31], "extent": [-5., 5.],
                        "thresholds_raw_mass": scans, "tau_0.15_y0_crossing_brackets": brackets}
        plotting[name] = vals.reshape(31, 31, 5)
    return result, axis, plotting


def write_plot(path, axis, plotting):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(12, 4), constrained_layout=True)
    colors = ("black", "tab:blue", "tab:green", "tab:orange", "tab:red")
    for ax, (name, vals) in zip(axes, plotting.items()):
        for k, mode in enumerate(("smooth", *MODES)):
            if vals[:, :, k].min() < .15 < vals[:, :, k].max():
                ax.contour(axis, axis, vals[:, :, k], levels=[.15], colors=[colors[k]])
            ax.plot([], [], color=colors[k], label=mode)
        ax.set(title=name, xlabel="x [m]", ylabel="y [m]", aspect="equal")
    axes[0].legend(fontsize=8)
    fig.suptitle("Offline raw PHD mass level tau=0.15 (uncalibrated); Delta=0.5, r=0.48, width=0.5")
    fig.savefig(path)
    plt.close(fig)


def report():
    cases = matched_scenarios()
    inequalities = {n: gap_metrics(p, q, .5, 1., .5) for n, (p, q) in cases.items()}
    parameters = []
    p, q = cases["sparse"]
    for delta in (.25, .5, 1.):
        for width in (.12, .5, 1.):
            parameters.append({"delta": delta, "radius": .48, "width": width,
                               **gap_metrics(p, q, delta, .48, width)})
    levels, axis, plotting = level_sets(cases)
    here = pathlib.Path(__file__).resolve()
    root = here.parents[4]
    source_files = [here, here.with_name("stage3c_cube_support_experiment.py"),
                    here.with_name("stage3d_superellipsoid_experiment.py"),
                    here.parents[1]/"src/dsp_particle_to_grid.cpp",
                    here.parents[1]/"src/conservative_grid_risk.cpp",
                    here.parents[1]/"include/coni_mpc/field_hocbf.h",
                    here.parents[1]/"src/acado_mpc/mpc_wrapper.cpp",
                    here.parents[1]/"acado_model/quadrotor_mpc_codegen/acado_common.h"]
    config = here.parents[1]/"parameters/num_sim_non_one_point.yaml"
    result = {"metadata": {"audit_date": "2026-10-11", "seed": SEED, "commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
                "git_status_at_execution": subprocess.check_output(["git", "status", "--short"], cwd=root, text=True).splitlines(),
                "python": platform.python_version(), "numpy": np.__version__, "platform": platform.platform(),
                "source_sha256": {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in [*source_files, config]},
                "live_d_safe_m": .48, "live_risk_threshold": None,
                "complete_support": True, "node_cropping": False, "D_projection_iterations": 64},
        "inputs": {n: {"particles": [{"p": pp.position.tolist(), "weight": pp.weight} for pp in p],
                       "queries": q.tolist()} for n, (p, q) in cases.items()},
        "geometry": geometry_audit(), "matched_D_scenarios": inequalities,
        "parameter_scan": parameters, "spatial_derivatives": derivative_audit(),
        "temporal_derivatives": temporal_audit(), "cropping_failures": cropping_audit(),
        "level_sets": levels, "live_boundary_geometry": live_boundary_geometry(), "timings": costs(cases)}
    return result, axis, plotting


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument("--plot", type=pathlib.Path)
    args = parser.parse_args()
    result, axis, plotting = report()
    payload = json.dumps(result, indent=2, allow_nan=False)
    if args.output:
        args.output.write_text(payload+"\n")
        print(f"Saved {args.output}")
    else:
        print(payload)
    if args.plot:
        write_plot(args.plot, axis, plotting)
        print(f"Saved {args.plot}")


if __name__ == "__main__":
    main()
