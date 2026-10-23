# Mathematical Design and Exact Codex Implementation Specification
## Robust MF-MPSC Under Causal Non-Inertial Prediction-Sequence Errors

**Baseline:** local checked-out `GS--DSP` branch `1016`, last verified commit `cdb5c55205878d5f7cedd6799c61f14dc31265b4` (2026-10-23). **Codex:** use the actual local workspace rather than downloading a repository. If the local source has since changed, retain the same mathematical model and adjust only path/API references.

**MANDATORY IMPLEMENTATION RULE:** The mathematical architecture, formulas, update rules, and fallback are fixed below. Do not delegate control-theory choices to Codex. Codex is to implement, test, and report. Where certified conditions are unavailable, label the module uncertified and keep the already proved fallback policy; do not invent approximate proof constants.

## 0. Scope, assumptions, and guarantee

1. Single UAV; UGV moving on a plane (yaw only). The **only predicted uncertain signal** is the original MF-MPSC causal *non-inertial* sequence
   \(\hat\Xi_{j|k}=(\hat a_{g,j|k}^{N},\hat\omega_{j|k}^{N},\hat\beta_{j|k}^{N})\).
   Here \(a_g^N=R^T\ddot G^W\), \(\omega^N=\omega e_z\), \(\beta^N=\dot\omega e_z\). Do not replace these with an unrelated speed/acceleration predictor.
2. Current UAV pose/velocity and current UGV origin pose/velocity/angular velocity are known accurately; the UAV first-order velocity model and command application are accurate. These are explicit idealizations, not additional error sources.
3. Three **predictor-error bounds** over continuous prediction time (piecewise-constant upper envelopes on each interval):
   \(\|a_g^N(t)-\hat a_g^N(t)\|\le d_a(t)\),
   \(|\omega(t)-\hat\omega(t)|\le d_\omega(t)\),
   \(|\beta(t)-\hat\beta(t)|\le d_\beta(t)\).
   Must hold between nodes, not only at the 100-ms stages. The angular-rate/acceleration errors must be physically jointly consistent: \(\dot\omega=\beta\); when the nominal staged \(\hat\omega,\hat\beta\) are not derivative-consistent, use the explicit nominal-consistency residual below.
4. The certified obstacle set is covered by high-confidence world-frame voxel balls (whole cube half diagonal \(r_v=\sqrt3\ell/2\)) and has a fixed, time-persistent certified representation for the first *rigorous static-obstacle guarantee*. This is a scope assumption about what perception supplies, not a new perception algorithm. For moving obstacles, a certified continuous-time swept-occupancy model and a persistent safe terminal region are additional prerequisites. The currently published 10-Hz DSP occupancy slices by themselves do not establish those premises.
5. Starting state satisfies the braking-recoverability condition \(B_F(P_0,W_0)\ge\eta\) below and the prescribed input/acceleration limits. The safety supervisor shall never claim a guarantee if this fails.
6. Theoretical results proven by this specification: (i) physically consistent worst-case non-inertial predictor-error propagation, (ii) conservative robust Field-HOCBF tightening for a static certified Field, (iii) *continuous-time collision safety and recursive feasibility of a certified safety supervisor*, conditional on 1–5. Not asserted: unconditional recursive feasibility of the original unconstrained ACADO tracking optimization, or unconditional safety with arbitrary time-varying/new obstacle sets.

## 1. Exact baseline mathematics and current implementation

Relative state \(x^N=(p^N,v^N)\) and relative command \(u^N\):

\[
\dot p^N=v^N,\qquad
\dot v^N=\Lambda(u^N-v^N)-a_g^N-2\omega^N\times v^N-\beta^N\times p^N-\omega^N\times(\omega^N\times p^N).
\tag{1}
\]

UGV world pose \(G^W,R=R_{WN}\), world velocity \(V_g^W=\dot G^W\), UAV world state

\[
P^W=G^W+Rp^N,\qquad W^W=V_g^W+R(v^N+\omega^N\times p^N).
\tag{2}
\]

The executed velocity command is

\[
q^W=V_g^W+R(u^N+\omega^N\times p^N).
\tag{3}
\]

For yaw-only rotations and \(\Lambda=\lambda I\) (the current simulation YAML has \(\tau_{xy}=\tau_z=0.2\) s, \(\lambda=5\) s\(^{-1}\)), consistent transformation gives the actual world-frame plant

\[
\dot P^W=W^W,\qquad\dot W^W=\lambda(q^W-W^W).
\tag{4}
\]

If separate axis time constants are later used, do not use (4) with a scalar \(\lambda\) unless the actual world-frame transfer is re-derived. This first implementation can assert equal time constants.

Current project: `marsim_ws/src/coni_mpc/src/coni_mpc/num_sim_mpc.cpp` `buildNonInertialDataProfile`, `buildFieldProfile`; `marsim_ws/src/coni_mpc/include/coni_mpc/field_hocbf.h`; `marsim_ws/src/coni_mpc/acado_model/quadrotor_model_thrustrates.cpp`; `marsim_ws/src/coni_mpc/src/acado_mpc/mpc_controller.cpp`; simulation parameters `marsim_ws/src/coni_mpc/parameters/num_sim_non_one_point.yaml` (50 Hz control, 20 MPC stages at 0.1 s, hard CBF slack zero).

## 2. Predictor-error signal and exact non-inertial residual

At fixed \((p,v)\) and using the same N-frame convention:

\[
 e_a=a_g^N-\hat a_g^N,\quad e_\omega=\omega^N-\hat\omega^N,\quad e_\beta=\beta^N-\hat\beta^N.
\tag{5}
\]

With \([z]_\times y=z\times y\), expansion of (1) is **exact**:

\[
\Delta f_{\rm NI}=-e_a+M_\omega e_\omega+[p]_\times e_\beta-e_\omega\times(e_\omega\times p),
\quad M_\omega=2[v]_\times+[\hat\omega\times p]_\times+[\hat\omega]_\times[p]_\times.
\tag{6}
\]

For any direction \(g\), a conservative, directional scalar certificate is

\[
-g^T\Delta f_{\rm NI}\le d_a\|g\|+d_\omega\|M_\omega^Tg\|+d_\beta\|[p]_\times^Tg\|+d_\omega^2\|p\|\|g\|.
\tag{7}
\]

**Do not insert (7) blindly into the physical Field-HOCBF.** For a world-static voxel \(C^W\), its moving-frame center \(c=R^T(C^W-G^W)\) obeys

\[
\dot c=-R^TV_g^W-\omega\times c,\quad
\ddot c=-a_g^N-2\omega\times\dot c-\beta\times c-\omega\times(\omega\times c).
\tag{8}
\]

The relative vector \(r=p-c\) then obeys

\[
\ddot r=\Lambda(u-v)-2\omega\times\dot r-\beta\times r-\omega\times(\omega\times r),
\tag{9}
\]

so the frame-origin acceleration cancels. The true world-frame Field and the actual world-frame command (3) are used below to avoid double-counting fictitious forces. Keep (6)–(7) for diagnostics and checking the model, not as a second independent disturbance in the final world-frame bound.

## 3. Reconstruct *consistent* UGV pose and velocity tubes from the THREE causal sequences

The instantaneous forecast is the pre-existing MF-MPSC tuple \((\hat a_g^N,\hat\omega,\hat\beta)\), **not** a newly fitted UGV state model. Initialize at current measured \(G_0,V_{g0},\theta_0,\omega_0\). For continuous time \(s\in[0,2]\) seconds define nominal trajectories by physically consistent integration:

\[
\dot{\hat\theta}=\hat\omega,\quad \dot{\hat R}=\hat R[\hat\omega e_z]_\times,\quad
\dot{\hat V}_g^W=\hat R\hat a_g^N,\quad \dot{\hat G}^W=\hat V_g^W.
\tag{10}
\]

Use **zero-order hold** of the original stage values on each 100-ms interval, and split into five 20-ms subintervals for the reachability computation. This interpolation policy is fixed for version 1. Integrate each constant-yaw-rate stage exactly. In the horizontal plane use complex notation \(a_c=\hat a_x+i\hat a_y\), \(V_c=V_x+iV_y\), \(G_c=G_x+iG_y\), and \(z=i\hat\omega s\):

\[
F_1(s)=s\frac{e^z-1}{z},\qquad F_2(s)=s^2\frac{e^z-1-z}{z^2},
\]
\[
\boxed{\theta^+=\theta+\hat\omega s,\quad V_c^+=V_c+e^{i\theta}a_cF_1(s),\quad G_c^+=G_c+V_cs+e^{i\theta}a_cF_2(s).}
\]

For \(\hat\omega=0\), use the limits \(F_1=s\), \(F_2=s^2/2\); for \(|z|<10^{-3}\), evaluate \(F_1=s\sum_{n=0}^{8}z^n/(n+1)!\) and \(F_2=s^2\sum_{n=0}^{8}z^n/(n+2)!\) to avoid cancellation (the omitted absolute terms at this threshold are far below double precision). For other values evaluate the displayed closed forms. Vertical motion: \(V_z^+=V_z+\hat a_zs\), \(G_z^+=G_z+V_zs+\hat a_zs^2/2\). Do not introduce a new UGV predictor.

Check physical consistency of \(\hat\omega,\hat\beta\). Let

\[
r_\omega(s)=\hat\omega(s)-\hat\omega(0)-\int_0^s\hat\beta(t)\,dt.
\tag{11}
\]

Since real \(\dot\omega=\beta\) and the initial yaw rate is known,

\[
e_\omega(s)=\int_0^s e_\beta(t)\,dt-r_\omega(s).
\tag{12}
\]

When both \(d_\omega(s)\) and \(d_\beta(s)\) are valid bounds, an effective rate bound is

\[
E_\omega(s)=\min\left\{d_\omega(s),\ \int_0^s d_\beta(t)dt+|r_\omega(s)|\right\}.
\tag{13}
\]

Use \(E_\theta(s)=\int_0^sE_\omega(t)dt\), \(E_R(s)=\min\{2,E_\theta(s)\}\) (spectral norm of yaw rotation difference). Other known model-consistency residuals between the pre-existing `car_trajectory_window_` and the integrated trajectory must be included in the corresponding nominal-center enclosure; never mix inconsistent nominal sequences and call the difference uncertainty-free.

True and nominal world UGV velocities satisfy

\[
V_g^W(s)-\hat V_g^W(s)=\int_0^s\left(R(t)a_g^N(t)-\hat R(t)\hat a_g^N(t)\right)dt.
\tag{14}
\]

Hence the computable comparison envelopes are

\[
\boxed{E_{V_g}(s)=\int_0^s\left(d_a(t)+E_R(t)\|\hat a_g^N(t)\|\right)dt,\qquad E_G(s)=\int_0^sE_{V_g}(t)dt.}
\tag{15}
\]

These bounds are **derived from the causal NI predictions** and directly specified predictor-error bounds; the four quantities \(E_\omega,E_R,E_{V_g},E_G\) are not four new independent disturbance hypotheses.

## 4. Exact world-frame command-error identity and scalar reachability tube

For yaw-only UGV with \(Jz=e_z\times z\) (3D cross-product matrix, vertical coordinate unchanged), the command mapping (3) can be written entirely in world variables:

\[
q=V_g+Ru+\omega J(P-G).
\tag{16}
\]

For a **fixed planned relative-frame command** \(u=\hat u\) while the true and nominal UAV positions differ by \(\Delta P=P-\hat P\):

\[
\boxed{\Delta q=\Delta V_g+(R-\hat R)u+(\omega-\hat\omega)J(\hat P-\hat G)+\omega J(\Delta P-\Delta G).}
\tag{17}
\]

Consequently, on any interval \(I_j=[t_j,t_{j+1}]\), choose certified uniform upper envelopes \(\bar E_{V_g,j},\bar E_{R,j},\bar E_{\omega,j},\bar E_{G,j}\), \(U_j\ge\|u\|\), \(D_j\ge\|\hat P-\hat G\|\), and

\[
\Omega_j=\max_{I_j}|\hat\omega|+\bar E_{\omega,j},\quad
C_j=\bar E_{V_g,j}+\bar E_{R,j}U_j+\bar E_{\omega,j}D_j+\Omega_j\bar E_{G,j}.
\tag{18}
\]

Then \(\|\Delta q\|\le C_j+\Omega_j\|\Delta P\|\). For \(E_P\ge\|\Delta P\|\), \(E_W\ge\|\Delta W\|\), the comparison dynamics of (4) are

\[
\boxed{\dot E_P=E_W,\qquad \dot E_W=\lambda\Omega_j E_P-\lambda E_W+\lambda C_j.}
\tag{19}
\]

Initialize \(E_P(0)=E_W(0)=0\) for a fresh prediction with exact current state. Propagate the **positive system** exactly over a subinterval of duration \(d\):

\[
\begin{bmatrix}E_P\\E_W\\1\end{bmatrix}_{t+d}=
\exp\left(
 d\begin{bmatrix}0&1&0\\\lambda\Omega_j&-\lambda&\lambda C_j\\0&0&0\end{bmatrix}\right)
\begin{bmatrix}E_P\\E_W\\1\end{bmatrix}_{t}.
\tag{20}
\]

The 3x3 matrix exponential is explicit and stable using Eigen's matrix-functions implementation. Use 20-ms subintervals (at least) to cover each 100-ms MPC stage. Because the comparison matrix is Metzler and \(C_j,\Omega_j\ge0\), this encloses all errors in the specified uncertainty envelopes.

At stage \(j\):

\[
\boxed{E_{q,j}=C_j+\Omega_j E_{P,j}.}
\tag{21}
\]

For per-stage robust bounds, take the maximum envelopes over the **entire stage**, not only its endpoint. If a future command is stored as a **fixed executed world-frame command**, set the future UGV-to-command coupling \(\Delta q=0\) for that held command: the physical plant does not suffer fictitious UGV prediction disturbances. This branch distinction is essential.

## 5. Exact current C2 Field and analytic rigorous derivative bounds

For certified static world-frame voxel centers \(C_i\), occupancy \(P_i\), voxel size \(\ell\), UAV radius \(r_u\), confidence-core threshold \(P_{\rm core}\), and current Field settings, define

\[
B=r_u+\frac{\sqrt3}{2}\ell+d_{\rm margin},\quad
s_i=\sqrt{\|P-C_i\|^2+\epsilon^2},\quad
\rho_i=s_i-\epsilon,
\]
\[
 a_i=Q(P_i/P_{\rm core})(1+\alpha P_i),\quad
Q(x)=\operatorname{smootherstep}(\operatorname{clip}(x,0,1)),
\]
\[
S(P)=S_0+\sum_i a_i c(\rho_i)e^{-(\rho_i-B)/\sigma},\qquad S_0=e^{-H_{\rm clear}/\sigma},\qquad
h_F(P)=-\sigma\log S(P).
\tag{22}
\]

Radial cutoff \(c(\rho)=1\) for \(\rho\le R-w\), \(c(\rho)=Q((R-\rho)/w)\) on \((R-w,R)\), zero for \(\rho\ge R\). Here \(R=2.0\) m and \(w=0.5\) m in the checked YAML. Maintain every voxel that can contribute anywhere in the uncertainty region; pruning using only the nominal position is **not** sufficient. For a center \(\hat P\) and error ball radius \(E_P\), include all voxels with center distance less than \(R+E_P+\epsilon\).

For each voxel define \(f_i(\rho)=a_ic(\rho)e^{-(\rho-B)/\sigma}\), and its exact radial derivatives:

\[
 f_i'=a_ie^{-(\rho-B)/\sigma}(c'-c/\sigma),
\]
\[
 f_i''=a_ie^{-(\rho-B)/\sigma}(c''-2c'/\sigma+c/\sigma^2),
\]
\[
 f_i'''=a_ie^{-(\rho-B)/\sigma}(c'''-3c''/\sigma+3c'/\sigma^2-c/\sigma^3).
\tag{23}
\]

In the taper interval, \(u=(R-\rho)/w\),

\[
c'=-30u^2(1-u)^2/w,\quad c''=60u(1-u)(1-2u)/w^2,\quad c'''=-60(1-6u+6u^2)/w^3.
\tag{24}
\]

Although \(c\) is globally C2 and its third derivative jumps across two boundaries, \(c''\) is Lipschitz, so interval bounds for the Hessian using a **piecewise** third-derivative supremum are valid. Safe absolute constants on taper are:

\[
|c|\le1,\quad |c'|\le1.875/w,\quad |c''|\le15/w^2,\quad |c'''|\le60/w^3.
\tag{25}
\]

### 5.1 Exact derivatives

Let \(n_i=(P-C_i)/s_i\) and \(D_i=I/s_i-(P-C_i)(P-C_i)^T/s_i^3\). Then

\[
\nabla S=\sum_if_i'n_i,\quad
\nabla^2S=\sum_i(f_i''n_in_i^T+f_i'D_i),
\]
\[
\boxed{g=\nabla h_F=-\sigma\nabla S/S,\qquad H=\nabla^2 h_F=-\sigma\left(\nabla^2S/S-\nabla S\nabla S^T/S^2\right).}
\tag{26}
\]

These must match the current `computeConstraint` mathematical expressions (when transformed consistently into the world frame).

### 5.2 Exact conservative box/ball derivative bounds

For a ball \(\mathcal B_j=B(\hat P_j,E_{P,j})\), each voxel center has \(d_i=\|\hat P_j-C_i\|\), \(r^-_i=\max(0,d_i-E_{P,j})\), \(r^+_i=d_i+E_{P,j}\),

\[
\rho_i^-=\sqrt{(r_i^-)^2+\epsilon^2}-\epsilon,\quad
\rho_i^+=\sqrt{(r_i^+)^2+\epsilon^2}-\epsilon,\quad
s_i^-=\sqrt{(r_i^-)^2+\epsilon^2}.
\tag{27}
\]

For each interval \([\rho_i^-,\rho_i^+]\), derive \(C_{0i}=\max c\le1\), \(C_{1i}=\max|c'|\), \(C_{2i}=\max|c''|\), \(C_{3i}=\max|c'''|\), using (24)–(25); take derivative constants zero if the interval does not intersect the taper. Set \(C_{0i}=0\) if the entire interval lies outside support. Define

\[
A_i^+=a_i e^{-(\rho_i^--B)/\sigma},\quad
\underline S=S_0+\sum_i a_ic(\rho_i^+)e^{-(\rho_i^+-B)/\sigma}>0.
\tag{28}
\]

Monotonicity of \(c(\rho)e^{-(\rho-B)/\sigma}\) makes \(\underline S\) valid. Define

\[
\begin{aligned}
D_{1i}&=A_i^+(C_{1i}+C_{0i}/\sigma),\\
D_{2i}&=A_i^+\left[C_{2i}+2C_{1i}/\sigma+C_{0i}/\sigma^2+(C_{1i}+C_{0i}/\sigma)/s_i^-\right],\\
D_{3i}&=A_i^+\Big[C_{3i}+3C_{2i}/\sigma+3C_{1i}/\sigma^2+C_{0i}/\sigma^3\\
&\hspace{12mm}+3(C_{2i}+2C_{1i}/\sigma+C_{0i}/\sigma^2)/s_i^-+6(C_{1i}+C_{0i}/\sigma)/(s_i^-)^2\Big].
\end{aligned}
\tag{29}
\]

The \(1/s_i^-\) and \(6/(s_i^-)^2\) terms are safe bounds for second/third derivatives of regularized Euclidean distance. Set \(S_r=\sum_iD_{ri}\), \(r=1,2,3\), and

\[
\boxed{G_j=\sigma S_1/\underline S,\quad
H_j=\sigma\left(S_2/\underline S+S_1^2/\underline S^2\right),\quad
T_j=\sigma\left(S_3/\underline S+3S_2S_1/\underline S^2+2S_1^3/\underline S^3\right).}
\tag{30}
\]

These satisfy over the ball: \(\|g\|\le G_j\), \(\|H\|\le H_j\), and \(\|H(P)-H(\hat P)\|\le T_j E_{P,j}\). If a bound is very conservative, subdivide the spatial ball into smaller certified boxes and maximize the resulting bounds. Never use unverified finite sampling as a global bound.

### 5.3 Useful rigorous global Lipschitz constant for the Field

Let \(R_{\rm on}=R-w\), and \(A_{\rm sum}=\sum_i a_i\). Because \(\|\nabla\rho_i\|\le1\), \(\sum_i f_i/S\le1\), and \(|c'|\le1.875/w\) only where \(\rho\ge R_{\rm on}\):

\[
\boxed{L_F=1+\sigma\frac{1.875}{w} A_{\rm sum}\exp\left(\frac{H_{\rm clear}+B-R_{\rm on}}{\sigma}\right)}
\tag{31}
\]

is a conservative global bound \(\|\nabla h_F\|\le L_F\) for a **fixed certified voxel snapshot**. Evaluate \(A_{\rm sum}\) over *all* terms that could contribute, not a nominal-position-pruned subset. This constant can be loose but, for the current taper and safety radius, does not require a global Hessian bound. Track its validity with the snapshot ID.

For any core voxel with \(P_i\ge P_{\rm core}\), \(a_i\ge1\), and in its possible collision region \(c(\rho_i)=1\), so \(h_F(P)\le \rho_i-B\); therefore \(h_F(P)\ge0\) implies geometrical separation from every such certified core voxel. The *assumption* that those core voxels cover actual obstacles is indispensable.

## 6. Exact robust Field-HOCBF margin from causal-sequence uncertainty

For a **static world-frame Field** (the first formally certified scope), actual UAV world dynamics (4) yield

\[
\dot h=g^TW,
\quad\ddot h=W^THW+\lambda g^T(q-W),
\]
\[
\boxed{\psi_2(P,W,q)=\lambda g(P)^T(q-W)+W^TH(P)W+(\gamma_1+\gamma_2)g(P)^TW+\gamma_1\gamma_2h(P).}
\tag{32}
\]

At a nominal stage \((\bar P,\bar W,\bar q)\), set \(s=\gamma_1+\gamma_2\), \(p=\gamma_1\gamma_2\), \(\bar g=\nabla h(\bar P)\). Compute \(E_P,E_W,E_q\) from (20)–(21), with \(G,H,T\) from (30). Use the **fully explicit, conservative** bound

\[
\boxed{\begin{aligned}
\rho_{g,j}={}&\lambda\left[\|\bar g\|(E_q+E_W)+H E_P(\|\bar q-\bar W\|+E_q+E_W)\right]\\
&+H(2\|\bar W\|E_W+E_W^2)+T E_P\|\bar W\|^2\\
&+s\left[\|\bar g\|E_W+H E_P(\|\bar W\|+E_W)\right]+pG E_P.
\end{aligned}}
\tag{33}
\]

All \(E_*\) and derivative constants apply to the whole interval of interest. (33) follows by adding and subtracting nominal terms in (32), applying mean-value bounds for \(h\) and \(g\), and the certified Hessian variation \(\|H(P)-H(\bar P)\|\le TE_P\). **No unaccounted Taylor remainder remains.** Thus

\[
\boxed{\psi_2(P,W,q)\ge\psi_2(\bar P,\bar W,\bar q)-\rho_{g,j}}
\tag{34}
\]

for every actual trajectory consistent with the causal NI error tubes and same planned relative command sequence. If error bounds vanish, all \(E\) and \(\rho_g\) are exactly zero.

**ACADO integration:** At the linearization state, `field_hocbf::computeConstraint` returns `A=lambda*g` and `b=-(lf2 + (gamma1+gamma2)*hdot + gamma1*gamma2*h)`, so its frozen row is `A*u - b >= 0`. Existing `profile_b = b + A.dot(k_v*vbar + k_p*pbar)` and ACADO checks `A.dot(u+k_v*v+k_p*p) - profile_b >= 0`. For the *UGV prediction uncertainty alone*, set

\[
\boxed{\texttt{profile\_b\_robust}=\texttt{profile\_b}+\rho_{g,j}}.
\tag{35}
\]

This is the correct sign. The ACADO row is an *affine surrogate*, not the exact nonlinear Field along all possible QP states: after every solve, compute (32) on the returned predicted world trajectory and compare with (33). The solver-stage tightening by (35) alone cannot be considered an unconditional certificate for the ACADO solver, unless the affine surrogate error is separately bounded over a solver trust region. **Overall collision safety is certified independently by the exact safety supervisor in Section 7**, which does not rely on affine surrogate correctness. Record exact postsolve margin `psi2_exact_nominal - rho_g`; reject the MPC plan as uncertified when negative rather than calling it a proof. Do **not** implement an unspecified affine-gap heuristic in this first version. The robust ACADO row is a performance-planning constraint. The **final executed world command must always pass the separate exact safety supervisor**, so the system-level safety proof does not rely on solver affine-model certification.

For a time-varying known C2 Field \(h(P,t)\), the correct generalization is

\[
\psi_2= W^THW+\lambda g^T(q-W)+2(\partial_t g)^TW+\partial_{tt}h+s(g^TW+\partial_t h)+ph.
\tag{36}
\]

Current discrete DSP stage changes do **not** provide the required continuously differentiable time model or certified derivative bounds. Do not claim (33) formally guarantees moving-obstacle safety without extending all these terms and the obstacle swept-volume coverage.

## 7. Fully explicit, *recursively feasible* safety supervisor — no LQR guesswork

The theoretical obstacle to using an LQR ellipsoid as the only backup is that **LQR stability does not certify collision avoidance**. Instead, construct a certified braking control-invariant set directly from the present plant and Field. This is the simplest exact terminal design consistent with (4).

Let \(\tau=1/\lambda=0.2\) s, \(F(P)=h_F(P)\), \(L=L_F\) from (31), and choose fixed certificate reserve \(\eta\ge0\) (e.g. zero for the mathematical theorem; positive reserve is a configurable engineering margin). Define

\[
\boxed{B_F(P,W)=h_F(P)-L\tau\|W\|_2.}
\tag{37}
\]

The **braking invariant / terminal set** is

\[
\boxed{\begin{aligned}
\mathcal K_F=\{(P,W):\ &B_F(P,W)\ge\eta,\\
&\|W_{xy}\|_2\le\min(v_{xy,\max},\tau a_{xy,\max}),\\
&|W_z|\le\min(v_{z,\max},\tau a_{z,\max})\}.\end{aligned}}
\tag{38}
\]

Terminal feedback is simply **world-frame braking**

\[
\boxed{q_f^W(P,W)=0,\qquad u_f^N=R^T(0-V_g^W)-\omega\times p^N.}
\tag{39}
\]

For the accurate first-order dynamics with world command \(q=0\):

\[
W(s)=e^{-s/\tau}W_0,\qquad
P(s)=P_0+\tau(1-e^{-s/\tau})W_0.
\tag{40}
\]

Field Lipschitz continuity gives

\[
\begin{aligned}
B_F(P(s),W(s))
&\ge h_F(P_0)-L\tau(1-e^{-s/\tau})\|W_0\|-L\tau e^{-s/\tau}\|W_0\|\\
&=B_F(P_0,W_0)\ge\eta.
\end{aligned}
\tag{41}
\]

World velocity and physical acceleration decay monotonically, and \(q=0\) lies within every command-speed limit. Thus \(\mathcal K_F\) is a **control-invariant safe set** and \(q_f^W=0\) remains admissible for all subsequent times, under the fixed persistent Field assumption.

This gives an explicit *terminal safe set and feedback*, without numerical terminal-LQR tuning. It is also directly implementable when MPC fails. A terminal LQR can optionally be designed **within** \(\mathcal K_F\) to improve performance, but is not required for the guarantee and must not replace (37)–(41) without a new invariance certificate.

### 7.1 Exact 20-ms transition under a constant world command

Let \(\Delta=0.02\) s, \(\alpha=e^{-\Delta/\tau}\), \(c=\tau(1-\alpha)\). For a candidate constant world command \(q\),

\[
\boxed{P^+=P+cW+(\Delta-c)q,\qquad W^+=\alpha W+(1-\alpha)q.}
\tag{42}
\]

A **fully explicit sufficient condition** for preserving \(\mathcal K_F\) at the next control update is

\[
\boxed{L\left(\|cW+(\Delta-c)q\|_2+\tau\|\alpha W+(1-\alpha)q\|_2\right)\le h_F(P)-\eta.}
\tag{43}
\]

An additional conservative **intersample safety condition** valid for every \(s\in[0,\Delta]\) is

\[
\boxed{L\left(c\|W\|_2+(\Delta-c)\|q\|_2\right)\le h_F(P).}
\tag{44}
\]

Proof: \(c(s),s-c(s)\) are nonnegative and monotone increasing for \(s\ge0\), and \(h(P(s))\ge h(P)-L\|P(s)-P\|\). (43) gives \(h(P^+)-L\tau\|W^+\|\ge\eta\); (44) guarantees safety between samples.

Enforce the existing physical limits **for the candidate world command**:

\[
\boxed{\|q_{xy}\|_2\le v_{xy,\max},\ |q_z|\le v_{z,\max},\quad
\|(q-W)_{xy}\|_2/\tau\le a_{xy,\max},\quad |q_z-W_z|/\tau\le a_{z,\max}.}
\tag{45}
\]

**Additionally, preserve the next-step braking admissibility bounds**:

\[
\boxed{\|W^+_{xy}\|_2\le\min(v_{xy,\max},\tau a_{xy,\max}),\qquad |W_z^+|\le\min(v_{z,\max},\tau a_{z,\max}),}\tag{45b}
\]

These next-step bounds are ESSENTIAL: enforcing present acceleration alone does not guarantee that the world braking command \(q=0\) will remain acceleration-feasible at the next control tick. At \(q=0\), they automatically hold because \(W^+=\alpha W\). The velocity/acceleration inequalities remain true through the entire 20-ms zero-order-held execution interval when starting from feasible current velocity: \(W(s)=\alpha_s W+(1-\alpha_s)q\) is a convex combination, and physical acceleration is \((q-W(s))/\tau=\alpha_s(q-W)/\tau\).

### 7.2 No additional optimization solver: convex ray search toward guaranteed brake command

Let the nominal MPC first-stage world command be \(q_{\rm MPC}\). Use the *explicit candidate family*

\[
\boxed{q(\mu)=\mu q_{\rm MPC},\qquad 0\le\mu\le1.}
\tag{46}
\]

Each constraint (43), (44), (45), and (45b) is convex in \(q\). Their intersection with the ray \(q(\mu)\) is an interval. At \(\mu=0\), (43) is satisfied with equality to the existing braking-certificate bound since \(c+\tau\alpha=\tau\); (44) follows from \(c\le\tau\), and (45) follows from \(\mathcal K_F\). Therefore **\(\mu=0\) is always feasible whenever the current state lies in \(\mathcal K_F\)**.

Implementation: evaluate (43)–(45) at \(\mu=1\); accept if true. Otherwise binary-search the largest feasible \(\mu\in[0,1]\) for 30 iterations and apply \(q=\mu q_{\rm MPC}\). For numerical robustness, after bisection reduce \(\mu\) by one ULP toward zero if needed and re-evaluate; on computational failure apply \(q=0\). No new QP/SOCP package required. World-frame yaw control can be kept from existing MPC subject to the original world-yaw physical limit; yaw is not in the spherical translational plant (4).

**Recursive-feasibility guarantee:** if \(x_k\in\mathcal K_F\), then the explicitly feasible candidate \(q=0\) always exists, the chosen \(q_k\) maintains \(x_{k+1}\in\mathcal K_F\), and the same holds by induction at every subsequent update. Combined with (44), \(h_F(P(t))\ge0\) **at every continuous time**, not merely at the 50-Hz nodes. This holds for **every admissible causal predictor error**, indeed independently of future errors, because the executed safe world command uses current measured UGV kinematics and the accurate world plant (4).

**Important limitation:** This proves recursive feasibility of the **certified safety-control supervisor**, not automatic feasibility of every future ACADO tracking optimization or certified 2-second robust *tracking* policy. The latter requires a distinct full-horizon robust feedback policy and scenario/tube tightening at every stage, plus terminal shift consistency. Do not relabel the safety-supervisor theorem as that stronger claim.

### 7.3 Terminal-safe reference region, if a spatial terminal reference is desired

For a fixed known certified-safe center \(P_s\), pick \(r_s\ge0\) satisfying \(h_F(P_s)-Lr_s\ge\eta\). An explicit compact invariant terminal subset is

\[
\boxed{\mathcal X_f(P_s)=\{(P,W):\ \|P-P_s\|+\tau\|W\|\le r_s,\quad W\text{ within the limits in (38)}\}.}
\tag{47}
\]

Under the terminal command \(q_f=0\), the sum \(\|P(s)-P_s\|+\tau\|W(s)\|\) never increases by the triangle inequality and (40). Choose \(P_s\) from an already known persistently safe region. This is optional for *terminal targeting*, since \(\mathcal K_F\) itself is already invariant.

## 8. Relationship with the existing ACADO MPC and shift fallback

1. Keep existing 2-second causal-stage non-inertial MPC and cost. Add the predictor-derived robust margin (33) to the existing Field online parameter as (35). Do not change optimizer dimensionality unless necessary; the runtime online data already has the Field affine row.
2. At every 20-ms control tick, convert proposed first-stage command to world coordinates **using CURRENT measured UGV pose, linear velocity and yaw rate**; then apply the explicit supervisor in (42)–(46), including (45b). If the solver fails, propose the time-shifted prior input as a **performance candidate** to the same supervisor, or choose \(q=0\) directly. Do **not** execute the cached command without the supervisor.
3. Maintain the original cached-plan time stamp and `j=floor(elapsed/0.1)` convention: a 20-ms controller tick must not consume one entire 100-ms prediction interval.
4. The world-frame safety supervisor is the final source of safety certification; it guarantees a safe continuation even if the ACADO optimizer returns infeasible, its affine approximation is imperfect, or it is not re-solved on a cycle.
5. Do not advertise that the ACADO solution has full robust recursive feasibility merely because this supervisor keeps the UAV safe. If the original tracking MPC is required to be recursively feasible too, this is a different, stronger implementation: a scenario-robust feedback policy and shifted tube over the full 2-second horizon with terminal membership in (47) would need to be optimized as part of ACADO. This specification deliberately gives the complete **safety-control** guarantee while retaining the nominal MF-MPSC performance optimizer.
6. To ensure the certified braking control is *actually executed*, use the currently measured UGV state in (3) inversion \(u^N=R^T(q^W-V_g^W)-\omega\times p^N\), and avoid any downstream command clipping or recomputation using different, predicted UGV values.

## 9. Snapshot validity and dynamic obstacles

- For the above strict proof, the field used for safety certification must represent a **persistent static world obstacle set** and remain a valid geometric upper cover. DSP updates are permitted as extra performance observations, but cannot invalidate the persistent certified field without a formal update condition.
- A map-switch safety rule can be added if the new field satisfies \(h_{F_{k+1}}(P)\ge h_{F_k}(P)-\Delta_h\) for all relevant \(P\), and the current braking reserve is large enough to absorb the verified switch: \(B_{F_k}(x_k)\ge\eta+\Delta_h+\Delta_L\tau\|W_k\|\), where \(\Delta_L\ge\max(0,L_{k+1}-L_k)\). This **single update** condition alone is not an infinite-horizon recursive guarantee; persistent cumulative validity or a common invariant safety set must also be established.
- Moving-obstacle extension requires a certified time-continuous swept occupancy cover, time derivatives in (36), and a terminal region that is guaranteed safe indefinitely or for an explicitly stated emergency-horizon strategy. Arbitrarily moving people can reach a previously safe hover point, so one cannot claim perpetual collision avoidance under unqualified future obstacle motion.
- The current confidence core `occupancy_full_confidence=0.15` is an engineering threshold, not a statistical guarantee of physical coverage. Scope the proof to voxels certified to cover the physical obstacles; keep that mapping assumption explicit.

## 10. Concrete C++ implementation map

1. **Existing** `marsim_ws/src/coni_mpc/src/coni_mpc/num_sim_mpc.cpp`:
   - Immediately after `buildNonInertialDataProfile` obtain the three staged causal predictor signals.
   - Build the uncertainty tube with the current stage-0 UGV pose/velocity and those signals. Compute \(E_\omega,E_\theta,E_R,E_{V_g},E_G\) and then \(E_P,E_W,E_q\), at 20-ms substeps.
   - In `buildFieldProfile`, calculate exact nominal world Field quantities, \(\rho_{g,j}\) from (33), and replace `profile_b` by `profile_b + rho_g` (for active Field rows only).
   - Include every voxel that may contribute throughout \(B(\bar P_j,E_{P,j})\). Keep current diagnostics.
   - Before publishing the actual control command, transform to world using **current measurements**, call the supervisor, convert the resulting world command back to the original relative interface.
2. **New** `include/coni_mpc/causal_ni_uncertainty.h`, `src/causal_ni_uncertainty.cpp`: structures for 21-stage causal predictions, three timewise error envelopes, consistency residual (11), UGV integration, and comparison tube (15),(18)–(21). The module does not estimate a new UGV predictor.
3. **New** `include/coni_mpc/field_derivative_cert.h`, `src/field_derivative_cert.cpp`: compute \(h,g,H,G,H,T,L_F\) from (22)–(31) and \(\rho_g\) from (33). Use complete world voxel slices as needed for certificates, not a nominal-only pruned subset.
4. **New** `include/coni_mpc/recursive_safety_supervisor.h`, `src/recursive_safety_supervisor.cpp`: implement (37)–(46), deterministic ray bisection, exact current world physical limits, and certified fallback `q=0`. Cache the fixed certified Field metadata and its validity.
5. **Existing** `marsim_ws/src/coni_mpc/src/acado_mpc/mpc_controller.cpp`: preserve its existing time-correct shifted input caching; hand proposed commands to supervisor instead of treating them as intrinsically safe. Last-stage indefinite hold is not a safety policy.
6. **Existing** ACADO model and regenerated solver: preserve current relative dynamics, costs, world physical constraints, and frozen affine Field row; the online `field_b` update does not need a new ACADO control dimension. Audit sign using (35).
7. **Existing** YAML: new `robust_ni.enabled`, `delta_a`, `delta_omega`, `delta_beta`, `safety_reserve_eta`, `require_certified_static_field`, `ray_bisection_iterations:30`, `debug_csv`, plus explicit status fields for certificate validity. Never silently enable strict guarantee when the initial state is outside (38).
8. Preserve baseline launch and ROS topic names. Add diagnostics: `rho_ni`, `E_P`, `E_W`, `E_q`, `h`, `h-L*tau*norm(W)`, chosen ray scale `mu`, `q_mpc_world`, `q_applied_world`, snapshot ID, `certified_safe` and `backup_used`, and physical clearance.

## 11. Required *focused* implementation checks + one simulation

These are basic correctness checks, not a paper-level experiment campaign:

- Zero predictor errors give \(E_* =0\) and \(\rho_g=0\).
- Coordinate-invariant prediction residual (6) verified algebraically for random small finite vectors, including the quadratic angular term.
- Consistent current world-command execution yields world plant (4), no artificial disturbance.
- Field analytic \(g,H\) coincide with original code; derivative upper bounds (30) upper-bound evaluated samples (sampling checks the implementation but is *not* the mathematical justification).
- If starting from a certified braking set, \(q=0\) passes (43)–(45b), for at least static-obstacle test states with nonzero velocity.
- Output \(q\) passes constraints (43)–(45b) and time-indexed cache selection does not advance by one 100-ms stage per 20-ms control tick.
- Build the ROS workspace, run **one** existing DSP + MF-MPSC closed-loop simulation. Produce a CSV with predictor vs actual causal NI terms, \(\rho_g,E_P,E_W,E_q\), Field HOCBF, geometric obstacle clearance, physical limits, solve/fallback status, braking certificate, and final world command.

## 12. Mathematical guarantee and precise proof

**Theorem A (causal-NI uncertainty enclosure).** Under the bounded continuous-time predictor error assumption, accurate current states, and consistent transformation, (15), (20), (21) bound the true UGV world motion, UAV world state, and future command deviation for the fixed planned relative command. Proof: integral inequalities (13)–(17), norm comparison inequality and positivity of the matrix in (19).

**Theorem B (robust static Field-HOCBF margin).** Under the fixed spatial C2 Field (22), including all relevant voxel terms, and uncertainty envelopes from Theorem A, (33) is an upper bound on the loss of the exact physical HOCBF (32). Proof: certified gradient, Hessian and piecewise third-derivative bounds (26)–(30), decomposition of each term in (32). If the exact nominal HOCBF \(\psi_{2,\rm nom}\ge\rho_g\), then the true continuous-time HOCBF is nonnegative at the evaluated model state/time. A discrete-stage-only check is **not** a whole-trajectory continuous-time proof.

**Theorem C (continuous obstacle safety + recursively feasible safety controller).** Under fixed persistent certified Field and valid initial \(\mathcal K_F\), the executed control chosen by (46), subject to (43)–(45), preserves \(\mathcal K_F\) at every 20-ms tick and \(h_F(P(t))\ge0\) for all real \(t\). Proof: candidate \(q=0\) is always admissible by (37)–(41); each accepted ray control satisfies (43)–(45); the entire intersample path is safe by (44); induction. Since the world-frame plant under an executed world command does not contain the predicted UGV non-inertial terms, the result holds for **all** admitted prediction errors in the causal sequences (and is structurally independent of their magnitude at the final execution layer).

**What the full implemented system guarantees:** certified collision avoidance and recursive feasibility of its *safety supervisor*, with nominal tracking MPC permitted to fail and safe braking as fallback, for the specified accurate plant and persistent certified static Field. The robust stage margin additionally improves forecast-aware safe planning. It does **not** establish unlimited-time moving-obstacle safety, true physical safety of low-confidence non-certified voxels, or robust recursive feasibility of the original unmodified ACADO performance NLP.

## 13. Codex work instruction

Implement **exactly** the algorithms and interfaces above in the current local repository. Do not substitute a new disturbance source, estimator, predictor, terminal-LQR approach, or generic tube-MPC architecture. First build the causal NI uncertainty module, then certified Field derivative and robust-margin module, then the invariant braking supervisor and integration. Wire the simulator to execute the certified world command. Build and run one closed-loop case; return changed files, actual formulas/constants, solver/build result, and CSV/log paths. Theoretical restrictions above are part of the delivered implementation design; report them factually and succinctly, without undertaking a broad research review or experimental matrix.
