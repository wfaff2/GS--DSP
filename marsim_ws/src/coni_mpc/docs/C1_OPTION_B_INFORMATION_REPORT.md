# Option B: independent audit of particle-position moment information

Audit date: 2026-10-11. Package: `/home/jjm/桌面/DSP/marsim_ws/src/coni_mpc`.
Code identity: `1e66bf5017cd93d07c2fa5a3c67b5cc2ab76f27f`, with pre-existing dirty work preserved.

**Decision: do not terminate Option B for lack of information value.** First and
second node-position moments contain demonstrably new information, and admit
provable continuum upper bounds substantially tighter than Stage 3C in the
matched examples. Continue it as an offline theoretical benchmark. Neither the
optimal moment envelope nor the reference certificates have a demonstrated
smooth online HOCBF interface. This audit does not authorize integration or
claim closed-loop safety improvement.

Deliverables: [reference experiment](../scripts/option_b_information_experiment.py),
[tests](../test/option_b_information_test.py), and
[full paired results and certificates](C1_OPTION_B_INFORMATION_RESULTS.json).
Only these standalone files and this report were added; `TASK_LOG.md` was appended.
No existing algorithm, YAML, ROS/DSP publication, controller, ACADO or online API
was changed; no Git commit or simulation was performed.

## 1. Sources and actual interfaces

The [3B](C1_STAGE3B_MOMENT_HESSIAN_REPORT.md),
[3C](C1_STAGE3C_CUBE_SUPPORT_REPORT.md),
[3D](C1_STAGE3D_SUPERELLIPSOID_REPORT.md), and
[3E](C1_STAGE3E_DIRECTIONAL_REPORT.md) mathematical reports, their standalone
reference experiments and tests were inspected. Earlier reads in this same
session were reused for unchanged sources. Stage 3A is represented in the
scoped files by [conservative_grid_risk.cpp](../src/conservative_grid_risk.cpp)
and [its tests](../test/conservative_grid_risk_test.cpp); no separate Stage 3A
report was found in the targeted report listing.

- 3A: spherical support, radius `2sqrt(3)Delta`, C² kernel composite.
- 3B: the negative Taylor correction is disproven. It is not reused here.
- 3C: exact cube-support upper risk; value-tight but not globally continuous Hessian.
- 3D: exact fourth-order superellipsoid projection, spatial C², relatively costly.
- 3E: analytical directional distance lower bound, spatial C² composite;
  matched dense mean excess improves 4.136% over 3A. This is offline evidence.

[P2G header](../include/coni_mpc/dsp_particle_to_grid.h) and
[implementation](../src/dsp_particle_to_grid.cpp) expose only node
`mass,mass_rate,mass_acceleration`. They do **not** expose position first/second
moments. Centers are fixed, each CV particle contributes to at most 64 nodes,
weights are nonnegative, and nonzero finite-grid support overflow is rejected.
Raw PHD weight is accumulated without occupancy conversion.

The live [Field-HOCBF](../include/coni_mpc/field_hocbf.h) consumes selected
kinematic occupancy points and uses

```text
d_m = sqrt(||(p-p_m)_xy||²+1e-12)
h = -sigma log(sum P_m exp(-d_m/sigma)/sum P_m) - d_safe.
```

[Current YAML](../parameters/num_sim_non_one_point.yaml): `sigma=.12 m`,
`d_safe=.48 m`, occupancy selection threshold `.15`.
There is no calibrated raw-PHD-mass threshold `tau`. The latter `.15` is not one.
The live field is planar and normalized; this benchmark is a 3D unnormalized
mass-weighted kernel sum. Multiplying all PHD weights scales the benchmark,
whereas multiplying all frozen occupancy weights leaves normalized soft-min
unchanged. This proves a basic semantic mismatch, even before dynamics.

`num_sim_mpc.cpp::buildFieldProfile` selects prediction slices, transforms and
prunes points. The wrapper sends frozen `Ax,Ay,Az,b,active` to the affine ACADO
row; generated dimensions remain `NOD=87, NPAC=10`. The current second-order
construction uses spatial Hessian information through along-trajectory
derivatives. ACADO does not differentiate this benchmark or require its third
derivatives. Spatial smoothness alone does not solve slice/pruning/snapshot
time discontinuities.

## 2. Contribution measure and moments

Let `a=2Delta`, `c_j=origin+Delta*j`, and let `N_ij(t)` be the existing cubic
tensor-product coefficient. Define the finite nonnegative contribution measure
in coordinates relative to node `j`:

```text
nu_j(t) = sum_i w_i N_ij(t) delta_{z_ij(t)}
z_ij(t) = p_i(t)-c_j
K = [-a,a]^3
m_j = integral 1 dnu_j
ell_j = integral z dnu_j                 (3 components)
Q_j = integral zz^T dnu_j                (6 independent components).
```

For every nonzero contribution `z_ij in K`. Endpoints of the cardinal spline
can have zero coefficient; using the closed box is a conservative closure.
These are **node-conditional particle-position moments**, not the familiar
global B-spline reproduction identities over grid centers.

When `m>0`, write `mu=ell/m`, `Sigma=Q/m-mu mu^T`. Normalization is only an
internal probability-measure device; it does not turn physical PHD mass into
occupancy probability. Store raw moments to avoid a singular normalized API.

The raw value statistics require 10 scalar entries: one mass, three first and
six second moments. Retaining the existing mass rate/acceleration alongside
them requires 12 entries. Full raw value/rate/acceleration jets for all moments
would require 30 entries; the current API has only three.

## 3. Continuum worst-case moment problem

Keep exactly the existing smootherstep kernel, for `r_c>0,w>0`:

```text
phi(d)=1                          d<=r_c
phi(d)=g((r_c+w-d)/w)              r_c<d<r_c+w
phi(d)=0                          d>=r_c+w
g(s)=6s^5-15s^4+10s^3.
```

For `y=q-c_j`, define

```text
V_j(y;m,ell,Q) = max_nu integral phi(||y-z||) dnu(z)
subject to nu>=0, support(nu) subset K,
           integral 1 dnu=m, integral z dnu=ell, integral zz^T dnu=Q.
```

This is the theoretical benchmark. It optimizes over **all** consistent
measures, not just the observed particle support or a query-dependent finite
grid. Risk evaluation needs only moments and node geometry.

**Existence.** For fixed feasible moments, measures of fixed mass on compact
`K` are weakly compact; the moment equalities are closed. The continuous bounded
kernel objective therefore attains its maximum. At `m=0`, feasibility forces
`ell=Q=0`, the only measure is zero, and `V=0`.

**Sandwich and hierarchy.** Let `V_1` omit second moments and `V_0` omit both
position moments. Adding constraints shrinks the feasible set, and

```text
V_full <= V_1 <= V_0 = m phi(dist(y,K)).
```

The equality for `V_0` is attained by placing all mass at a nearest point in
the box. It is exactly the Stage 3C node upper bound, not the Stage 3A sphere.
The actual contribution measure `nu_j` is feasible for `V_full`. Complete P2G
partition of unity gives

```text
R_smooth = sum_j integral phi(||q-c_j-z||) dnu_j
         <= sum_j V_full,j <= R_3C <= R_3A.
R_hard <= R_smooth.
```

Thus **weak dominance over 3C is rigorous**, and strict dominance is possible,
but not universal. If `Sigma=0`, nonnegativity forces `nu=m delta_mu`, so
`V=m phi(||y-mu||)`. A single-particle P2G cloud has this property at every
positive node, and recovers its exact smooth particle risk without particle
queries during evaluation. Plateau and zero-kernel queries can have equality
with 3C.

More generally, let `A_y=argmax_{z in K} phi(||y-z||)`. Equality with `V_0`
holds exactly when the prescribed moments admit a measure supported on `A_y`:
integrating the nonnegative deficit from the maximum must give zero.
Compact attainment means failure of that feasibility condition implies strict
tightening. This is an information-value criterion, independent of a solver.

Independent per-node optimization discards cross-node consistency: its
optimizers need not arise from one shared particle cloud and the original
B-spline weights. This relaxation preserves the upper bound but can leave
conservatism. It does not solve the globally coupled particle reconstruction.

## 4. Feasibility, ambiguity and genuinely new information

For `m>0`, exact feasibility is

```text
(ell/m,Q/m) in conv{(z,zz^T): z in K}.
```

Necessary conditions include `Sigma>=0`, `|mu_k|<=a`,
`Q_kk/m<=a²`, and the PSD moment block. They are not sufficient in 3D.
Counterexample: `m=1,ell=0,Q_kk=a²,Q_kl=-a²/2` for `k!=l`.
The matrix is PSD with eigenvalues `0,1.5a²,1.5a²`. Its diagonal forces every
coordinate to be `+/-a`, for which `(z_1+z_2+z_3)²>=a²`. Yet the proposed
expectation is `1^T Q 1=0`, a contradiction.

The experiment constructs its moments from particles, so feasibility has an
explicit witness. Its evaluator is not a general box-moment feasibility
validator. Clipping noisy covariance eigenvalues would change the constraints;
it is not a validity proof.

Even full moments do not identify risk. In a one-dimensional subspace strictly
inside `[-1,1]^3`, take mean zero and variance `.25`. Equal masses at `+/-.5`
give risk zero at the origin for `r_c=w=.2`. Mass `.609375` at zero and
`.1953125` at each of `+/-.8` has identical moments but risk `.609375`.
Both are inside the true open spline support and can be realized as a node's
contributions by choosing finite particle weights proportional to `1/N_ij`.

More strongly, **the entire grid's existing masses do not determine these
new statistics**. Exact same-P2G paired examples use particles inside one cell,
`Delta=1,y=z=.5`, where every node coefficient is a degree-three polynomial in
the particle's `x` coordinate:

- Five equally spaced dyadic positions `.125,.3125,.5,.6875,.875`, signed
  coefficients `[1,-4,6,-4,1]`. Normalize positive and negative sides by 8.
  Fourth differences annihilate every cubic, so all grid node masses coincide.
  Node first moments contain degree four and differ: maximum exact difference
  `14283/33554432`. At query `(.5,.5,.5)`, `r_c=w=.05`, the two particle risks
  are `.75` and `0`.
- Six positions `.125,.25,.375,.5,.625,.75`, coefficients
  `[1,-5,10,-10,5,-1]`, normalize each side by 16. Fifth differences annihilate
  degree four, so **all masses and all node first moments coincide**, while
  second moments differ by up to `2645/100663296`. At the same query/kernel,
  risks are `0` and `.625`.

Exact Fraction P2G verifies these equalities. The first and second statistics
each add independent information; they cannot be reconstructed in general
from the preceding statistics. These examples do not assert that moments
always determine the original cloud or its risk.

## 5. Dual formulation and why discretization is insufficient

Any quadratic majorant

```text
P(z)=alpha+beta·z+z^T C z >= phi(||y-z||) for EVERY z in K
```

gives the certified continuum upper bound
`V <= alpha m+beta·ell+tr(CQ)`. Integrating this pointwise inequality proves
weak duality without numerical optimization. The infimum over such majorants
is the compact moment problem's dual value. Closed concavity of the primal
value as a function of feasible moments, or separation of its hypograph,
gives equality of this infimum with the primal value; a finite optimal dual
polynomial need not exist on moment-cone boundaries.

An optimal measure exists with at most ten atoms. One justification is to
apply finite-dimensional convex-hull representation to moments plus objective
(at most eleven atoms), then remove a null direction among the ten moment
constraints. Optimality makes that perturbation objective-neutral; one weight
can be eliminated. This is an existence result, not a fixed-atom algorithm:
unknown atom positions make the search nonlinear and nonconvex.

A fixed grid/atom LP restricts the feasible measures. If it is exactly feasible,
its optimum is a **lower bound on V**, not an upper bound. Solver tolerances do
not certify even that lower bound unless feasibility errors are controlled.
The sampled dual only constrains sample points, not all of `K`.

The experiment gives two explicit warnings:

- Atoms `[-.8,-.5,0,.5,.8]`, mean zero, variance `.25`, and `r_c=w=.2` yield
  restricted optimum `.609375`. A continuum-feasible measure on `[-1,0,1]`
  gives `.75`, disproving its use as a continuum upper bound. The restricted
  optimum is also analytically checkable: `E x² <= .64(1-p_zero)`.
- A grid `[-1,-.5,0,.5,1]` cannot match a zero-variance Dirac at `.37`, despite
  the continuum problem being feasible. The LP correctly reports infeasible.

No general continuum optimum was numerically solved in this audit. The
certificates below bound it from above using moments alone.

## 6. Constructed continuum upper certificates

Three independent positive upper constructions are used, always capped by 3C.

**Positive Taylor bound about the conditional mean.** The radial Hessian has
radial eigenvalue `phi''` and nonpositive tangential eigenvalue `phi'/d`.
Since `max phi'' <= 10/(sqrt(3)w²)=L_+`,

```text
phi(||y-z||) <= phi(||y-mu||) + grad_z phi(||y-mu||)·(z-mu)
               + (L_+/2)||z-mu||²
V <= m[phi(||y-mu||)+(L_+/2)tr Sigma].
```

The linear term integrates to zero. This is a separately derived **plus**
upper bound, not Stage 3B's invalid negative correction.

**First-moment tail bound.** For any vector `v`, put
`L=-a||v||_1,H=a||v||_1,beta=v·mu`. The event `||y-z||<=s` implies
`v·z >= k=v·y-||v||s`. Because `v·z>=L`, Markov's inequality gives

```text
B_1(s)=0                            k>H
B_1(s)=1                            k<=L
B_1(s)=min(1,(beta-L)/(k-L))         L<k<=H.
```

At `k=H` it must not be forced to zero; boundary atoms are allowed.

**Second-moment Cantelli bound.** Set `v=y-mu` (unit normalization is optional),
`sigma_v²=v^T Sigma v`, and `g=k-beta`. When `g>0`,

```text
P(v·(z-mu)>=g) <= sigma_v²/(sigma_v²+g²);
B_2(s)=1 when g<=0.
```

This follows from Markov applied to `(X+t)²`, choosing `t=sigma_v²/g`.
Zero variance and positive gap give zero. The `v=0` case safely gives one.

**Certified layer integration.** The original kernel satisfies

```text
phi(d)=integral_[r_c,r_c+w] [-phi'(s)] 1_{d<=s} ds.
```

`B_1,B_2` are nondecreasing in `s`. For bin endpoints `s_b`, their right sums

```text
U_k = m sum_b [phi(s_(b-1))-phi(s_b)] B_k(s_b)
```

are therefore continuum upper bounds. This is not arbitrary quadrature or a
finite-support approximation. Nested bin refinement decreases the unrounded
upper sums; it converges to this chosen tail certificate, not necessarily `V`.

Per node the implementation takes

```text
U_first=min(U_3C,U_1)
U_full=min(U_3C,U_first,U_2,U_Taylor).
```

All candidates are valid for the full moment class, hence
`V_full <= U_full <= U_3C`. Minima can introduce nonsmooth switches.
The paired `U_first-U_full` isolates achieved gain from using second moments
beyond this first-moment certificate; it is not an exact comparison of `V_1`
and `V_full`.

## 7. Numerical certification and paired results

The float reference uses matched Stage 3E/3D particles and queries:
seed `20261011`, single/6 sparse/40 dense particles, same RNG consumption,
`Delta=.5,r_c=1,w=.5`, 32 bins. Every node is included. Original particle risk
is evaluated separately as ground truth; neither bound evaluator accepts
particles. JSON records complete particle/query inputs, commit, dirty state,
parameters and source hashes.

Ordinary double statistics can have tiny negative covariance roundoff. The
float path clips projected variance/trace at zero and records the minimum
eigenvalue. It is explicitly **not machine-certified**. The following whole
sample averages are numerical evidence for the analytically proved formulas:

| Scenario | Nodes/queries | mean(3C-original) | mean(first-original) | mean(full-original) | full excess reduction vs 3C |
|---|---:|---:|---:|---:|---:|
| Single | 64/4 | .619582 | .502854 | approximately 0 | 100% |
| Sparse | 348/24 | .756935 | .565226 | .056760 | 92.501% |
| Dense | 551/160 | 11.512652 | 8.743461 | 2.649132 | 76.989% |

Dense first-only excess reduction is 24.053%; adding second moments reduces
the achieved mean excess by another `6.094329` raw mass units.
Minimum full-minus-original is `-3.61e-16`, `-1.18e-16`, `.000834512` for the
three scenarios. Negative values are float roundoff, not asserted certificates.
No probability interpretation or population confidence interval is assigned.

**Genuine selected-query numerical certificates.** Four equally spaced query
indices per scenario were fixed in advance: single `0,1,2,3`, sparse `0,7,15,23`,
dense `0,53,106,159`. Exact binary-rational particle inputs are accumulated
with exact cubic Fraction arithmetic. Integer square roots bracket radical
distances with dyadic rationals. Kernel polynomials and bin weights are exact
rationals; tail/node upper sums round **up**, and cube lower sums round **down**
to 80 binary fractional bits to control denominator growth. Final mass caps
preserve `U<=m`, including vanishing masses. For nonunit `v`, an upper norm
bracket weakens the tail threshold in the conservative direction.

Each exact rational `C_lower-U_full_upper>0` proves continuum tightening,
including arithmetic error. Ten of twelve selected queries are strictly
tighter; the other two are legitimate plateau equality cases. All four dense
queries are strictly tighter:

| Dense query index | 3C lower (display) | full upper (display) | rigorous positive gap (display) |
|---:|---:|---:|---:|
| 0 | 20.17105765 | 7.43305120 | 12.73800645 |
| 53 | .20598376 | .00248404 | .20349972 |
| 106 | 4.22993064 | .36185985 | 3.86807079 |
| 159 | 16.34571383 | 5.47171472 | 10.87399911 |

Displayed decimals are rounded for readability. JSON contains exact rational
gap strings and reproducible certificate digests; those, not rounded displays,
establish strict positivity. These are conservative certificates, not exact
values of `V`. The 76.989% full-sample average was not independently recomputed
entirely in rational arithmetic.

Additional dense paired numerical scans use `r_c=.48`, 20 fixed queries:

| Width | mean(3C-original) | mean(first-original) | mean(full-original) |
|---:|---:|---:|---:|
| .12 | 9.071631 | 6.450955 | 1.826261 |
| .5 | 9.709695 | 7.289137 | 2.218231 |
| 1 | 10.231576 | 7.972002 | 2.301799 |

These widths are offline choices, not a mapping from live `field_sigma`.
On another fixed ten-query subset, 8/16/32/64 bins give full mean excess
`2.026686/1.948695/1.909867/1.890484`, against the same 3C excess `9.589819`.
Refinement demonstrates the chosen upper sum's residual discretization
conservatism; it does not solve the optimal moment problem.

## 8. Spatial derivatives: a rigorous regularity failure

For fixed feasible moments, each admissible measure's kernel integral is C²,
but their maximum need not be differentiable. Since the kernel gradient has
norm at most `15/(8w)`, `V` is Lipschitz in query with constant `m*15/(8w)`.
That continuity does not imply a gradient or Hessian at optimizer switches.

Actual-kernel counterexample on the closed feasible domain:
`m=1,ell=0,Q=a²I`. The diagonal moments force cube corners. All first and pair
sign correlations vanish, so the only remaining corner-weight parameter is

```text
p_s=(1+t s_1 s_2 s_3)/8, -1<=t<=1, s_k in {-1,+1}.
```

Choose `r_c<a<r_c+w<sqrt(5)a` and query `y=(x,a,a)` near `x=0`.
Only `(a,a,a)` and `(-a,a,a)` contribute. Maximizing over `t` yields

```text
V(x,a,a)=phi(a-|x|)/4.
```

At `a=1,r_c=.5,w=1`, `V(0,1,1)=.125` and the left/right derivatives are
`-.46875,+.46875`. Thus the continuum full-moment optimum is **not even C¹**.
The numerical one-sided slopes confirm the analytical result.

This example is on the closed-support feasibility boundary. Positive finite
cubic contributions at exact support endpoints have zero coefficient, so
these exact corner moments are not an actual finite P2G output. It disproves
automatic smoothness for the declared general moment problem; a theorem on
the practically realizable interior remains unresolved. Interior feasibility
alone is not a proved smoothness certificate.

With a unique stable optimizer, envelope differentiation can yield the
integral of kernel gradients. A second derivative additionally depends on
optimizer movement; merely freezing the maximizing measure's Hessian is not
generally correct. The reference bound's minima, tail clipping, directions at
`y=mu`, and bin switches also lack a demonstrated global C² guarantee. This
audit therefore provides no deployable gradient/Hessian API.

## 9. Temporal and mixed derivative requirements

Mass jets alone do not determine moment jets. Write `b_i=w_i N_ij,z_i=p_i-c_j`,
`v_i=z_i'`, `a_i=z_i''` on a fixed grid. Required raw formulas include

```text
m' = sum b_i';                  m''=sum b_i''
ell' = sum(b_i' z_i+b_i v_i)
ell'' = sum(b_i'' z_i+2b_i' v_i+b_i a_i)
Q' = sum[b_i' z_i z_i^T+b_i(v_i z_i^T+z_i v_i^T)]
Q'' = sum[b_i'' z_i z_i^T+2b_i'(v_i z_i^T+z_i v_i^T)
          +b_i(a_i z_i^T+z_i a_i^T+2v_i v_i^T)].
```

Fixed-weight CV particles and complete cubic support make these raw moments
C² across cell boundaries. Actual current P2G exports only the first line.
Snapshot replacement, birth/death/resampling, changed weights, map clipping
and non-inertial frame rotation require separate treatment.

Even complete jets do not repair a nonsmooth optimal envelope. Conditional on
`V(q,b)` being C² in raw moment vector `b=(m,ell,Q)` on a feasible neighborhood,

```text
R_t = sum V_b·b'
R_tt = sum[b'^T V_bb b'+V_b·b'']
grad_q R_t = sum V_qb b'.
```

Along a query trajectory one additionally needs
`R_tt+2 grad R_t·v+v^T H_R v+grad R·a`. Obtaining `V_bb,V_qb` requires stable
optimizer/dual sensitivity, not only stored statistics. Moving/rotating grid
geometry adds its own chain-rule terms. No such online regularity theorem or
jets interface was implemented.

At zero mass the risk is zero and `0<=V<=m`; raw moments remain meaningful,
while normalized mean/covariance may be undefined or numerically unstable.
Nonzero small masses cannot be silently discarded: add a worst-case bound
for any lost mass, or preserve it. Missing boundary contributions invalidate
the link to original risk even if retained moments are internally feasible.

## 10. Complexity and implementation limitations

Moment preprocessing is `O(64P)` with more arithmetic per contribution;
storage is `O(10J)` value scalars instead of mass-only `O(J)`. The closed-form
tail/Taylor reference is `O(JB)` with `B=32` bins and requires no particle
access during evaluation. It does not solve a query-specific LP at every node.

Reference timings, one warmup and five repeats, median milliseconds:

| Scenario | mass-only P2G | mass+position moments P2G | float certificate/query |
|---|---:|---:|---:|
| Single | .034509 | .370113 | .191279 |
| Sparse | .391308 | 2.380359 | .744198 |
| Dense | 1.177551 | 12.894355 | 1.306766 |

These Python/NumPy measurements separate preprocessing from query work;
they are not C++ runtime estimates. Exact selected-query certification took
about 43.6–1580.3 ms/query, depending on nonzero geometry and rational cost.
General atomic LPs have ten equalities but many grid weights; true continuum
dual verification requires global domination on the box. Unknown optimal
atom positions and optimizer sensitivities are substantially harder than
the inexpensive certificates demonstrated here.

Float scans and selected exact certificates answer different questions:
full-sample numerical benefit versus rigorously verified strict benefit at
preselected queries. A general optimal-value solver, uniform numerical error
budget for an online implementation, and derivative costs are not delivered.
No probability-risk threshold, calibrated controller gain or closed-loop
comparison is implied by the reductions.

## 11. Validation and research decision

Executed final commands:

```bash
python3 -m unittest -v marsim_ws/src/coni_mpc/test/option_b_information_test.py
python3 marsim_ws/src/coni_mpc/scripts/option_b_information_experiment.py \
  --output marsim_ws/src/coni_mpc/docs/C1_OPTION_B_INFORMATION_RESULTS.json
git diff --check
```

Initial 14 tests passed; adding the two exact same-grid information examples
gave **16/16 final tests passed**. Coverage: exact global raw moments and P2G
partition, old mass reference equality, Dirac recovery, zero/small mass,
outward radical/dyadic arithmetic, two-atom domination, matched sandwich and
second-moment gain, PSD infeasibility, spatial cusp, risk ambiguity, fixed-LP
lower-bound direction/infeasibility, bin refinement, mass scaling/full nodes,
and both exact same-grid information examples. No existing-stage tests or
ROS builds were rerun, since those algorithms were untouched.

Exploratory full Fraction totals originally suffered denominator growth and
were interrupted; directed dyadic upper/lower rounding fixed that cost while
preserving certification. A preliminary helper also hit Python 3.8's missing
`math.nextafter`; final radical bracketing uses exact integer square roots,
independent of libm. Outputs were regenerated after the new information
examples; all tables refer to final JSON. These development observations are
not failed mathematical gates.

Independent mathematical review and independent read-only code review
confirmed the sandwich, counterexamples, certificate directions, and exact
same-grid information constructions. The final artifact checks preserve the
existing work and source identities.

**Research decision and stopping point:**

1. Option B passes the information-value gate. Exact same-P2G examples prove
   new information, and sizeable rigorous selected-query gains show that
   full moments can beat 3C without original-particle evaluation.
2. Do not terminate Option B for negligible certified tightening. Conversely,
   do not interpret this as proof that every distribution/query improves or
   that the entire 77% numerical average is arithmetic-certified.
3. Continue only as a separately scoped offline benchmark. A useful next
   research question is the remaining gap between these cheap certificates
   and the continuum optimum, followed by smooth conservative surrogates if
   a control use case is justified. No additional solver engineering is
   needed to answer the present information-value question.
4. Stop online deployment work: the general optimum can be nonsmooth;
   practical-interior C², moment time jets/sensitivity, frame handling,
   boundary mass policies and PHD-threshold calibration remain unresolved.
   The current normalized soft-min barrier has different semantics.
5. Preserve 3A–3E as unchanged independent baselines. No closed-loop safety,
   solver feasibility, or runtime improvement has been established here.
