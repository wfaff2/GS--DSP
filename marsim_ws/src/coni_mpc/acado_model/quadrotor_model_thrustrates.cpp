// Created by Baozhe on Oct 11, 2022

/*    coni_mpc
 *    CoNi-MPC: Cooperative Non-inertial Frame Based Model Predictive Control
 *    Copyright (C) 2023 Baozhe Zhang, 
 *    Fast Lab, Huzhou Institute of Zhejiang University
 *  
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

// Reference: rpg_mpc (the original LICENSE is listed below)

/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <cmath>
#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

// Standalone code generation for a parameter-free quadrotor model
// with thrust and rates input. 

int main( ){
  // Use Acado
  USING_NAMESPACE_ACADO

  /*
  Switch between code generation and analysis.

  If CODE_GEN is true the system is compiled into an optimizaiton problem
  for real-time iteration and all code to run it online is generated.
  Constraints and reference structure is used but the values will be set on
  runtinme.

  If CODE_GEN is false, the system is compiled into a standalone optimization
  and solved on execution. The reference and constraints must be set in here.
  */
  const bool CODE_GEN = true;

  // System variables

  /*
   * Note:
   *
   * 1. The first 10 variables are the so-called "relative" state variables, i.e., 
   * they are the relative position, velocity, and orientation of the quadrotor
   * w.r.t. the non-inertial frame (e.g., a frame of a moving car).
   * For convenience, the names of them are plain without additional sub(or supper)-scripts.
   */
  DifferentialState     p_x, p_y, p_z;                              // relative position
  DifferentialState     v_x, v_y, v_z;                              // relative velocity
  DifferentialState     yaw;                                        // relative yaw

  // Commanded velocities and yaw rate in the non-inertial frame plus one
  // slack per obstacle.
  Control               v_cmd_x, v_cmd_y, v_cmd_z, yaw_rate_cmd,
                        delta0, delta1, delta2;
  DifferentialEquation  f;
  Function              h, hN;

  // Each risk region is a stage-wise constant p=4 superellipsoid.  The
  // center velocity/acceleration are supplied separately so the barrier uses
  // relative kinematics, while axes and orientation remain fixed constants at
  // the current shooting node.
  OnlineData            obs0_x, obs0_y, obs0_z;
  OnlineData            obs0_a, obs0_b, obs0_c;
  OnlineData            obs0_q00, obs0_q01, obs0_q02;
  OnlineData            obs0_q10, obs0_q11, obs0_q12;
  OnlineData            obs0_q20, obs0_q21, obs0_q22;
  OnlineData            obs0_vx, obs0_vy, obs0_vz;
  OnlineData            obs0_ax, obs0_ay, obs0_az, obs0_active;
  OnlineData            obs1_x, obs1_y, obs1_z;
  OnlineData            obs1_a, obs1_b, obs1_c;
  OnlineData            obs1_q00, obs1_q01, obs1_q02;
  OnlineData            obs1_q10, obs1_q11, obs1_q12;
  OnlineData            obs1_q20, obs1_q21, obs1_q22;
  OnlineData            obs1_vx, obs1_vy, obs1_vz;
  OnlineData            obs1_ax, obs1_ay, obs1_az, obs1_active;
  OnlineData            obs2_x, obs2_y, obs2_z;
  OnlineData            obs2_a, obs2_b, obs2_c;
  OnlineData            obs2_q00, obs2_q01, obs2_q02;
  OnlineData            obs2_q10, obs2_q11, obs2_q12;
  OnlineData            obs2_q20, obs2_q21, obs2_q22;
  OnlineData            obs2_vx, obs2_vy, obs2_vz;
  OnlineData            obs2_ax, obs2_ay, obs2_az, obs2_active;
  OnlineData            cbf_alpha1, cbf_alpha2;
  // Non-inertial frame online data (expressed in non-inertial frame N):
  // omega_non: angular velocity of frame N wrt world.
  // beta_non: angular acceleration of frame N wrt world.
  // a_car_non: linear acceleration of frame origin in frame N.
  OnlineData            omega_non_x, omega_non_y, omega_non_z;
  OnlineData            beta_non_x, beta_non_y, beta_non_z;
  OnlineData            a_car_non_x, a_car_non_y, a_car_non_z;
  // Stage-wise planar reference and trust for discounted UGV tracking.
  OnlineData            ref_x, ref_y, trust_factor;
  // Frozen-nominal probabilistic field HOCBF affine row.
  OnlineData            field_Ax, field_Ay, field_Az, field_b, field_active;

  // Parameters with exemplary values. These are set/overwritten at runtime.
  const double t_start = 0.0;     // Initial time [s]
  const double t_end = 2.0;       // Time horizon [s]
  const double dt = 0.1;          // Discretization time [s]
  const int N = round(t_end/dt);  // Number of nodes
  const double v_xy_max = 3.0;    // Maximal commanded velocity in xy [m/s]
  const double v_z_max = 1.0;     // Maximal commanded velocity in z  [m/s]
  const double yaw_rate_max = 3.14; // Maximal commanded yaw rate [rad/s]
  const double a_max_xy = 6.0;  // Physical acceleration limit XY [m/s^2]
  const double a_max_z = 4.0;   // Physical acceleration limit Z [m/s^2]
  const double tau_v_xy = 0.2;  // Tracking constant XY [s]
  const double tau_v_z = 0.2;   // Tracking constant Z [s]

  // First-order velocity tracking dynamics.
  Expression acc_x = (v_cmd_x - v_x) / tau_v_xy;
  Expression acc_y = (v_cmd_y - v_y) / tau_v_xy;
  Expression acc_z = (v_cmd_z - v_z) / tau_v_z;

  // Non-inertial compensation terms:
  // a_ni = -2 * (omega x v) - (beta x p) - omega x (omega x p) - a_car_non
  // Coriolis term: -2 * omega x v
  Expression omega_cross_v_x =
      omega_non_y * v_z - omega_non_z * v_y;
  Expression omega_cross_v_y =
      omega_non_z * v_x - omega_non_x * v_z;
  Expression omega_cross_v_z =
      omega_non_x * v_y - omega_non_y * v_x;
  Expression coriolis_x = -2.0 * omega_cross_v_x;
  Expression coriolis_y = -2.0 * omega_cross_v_y;
  Expression coriolis_z = -2.0 * omega_cross_v_z;

  // Euler term: - beta x p
  Expression beta_cross_p_x =
      beta_non_y * p_z - beta_non_z * p_y;
  Expression beta_cross_p_y =
      beta_non_z * p_x - beta_non_x * p_z;
  Expression beta_cross_p_z =
      beta_non_x * p_y - beta_non_y * p_x;
  Expression euler_x = -beta_cross_p_x;
  Expression euler_y = -beta_cross_p_y;
  Expression euler_z = -beta_cross_p_z;

  // Centrifugal term: - omega x (omega x p)
  Expression omega_cross_p_x =
      omega_non_y * p_z - omega_non_z * p_y;
  Expression omega_cross_p_y =
      omega_non_z * p_x - omega_non_x * p_z;
  Expression omega_cross_p_z =
      omega_non_x * p_y - omega_non_y * p_x;
  Expression omega_cross_omega_cross_p_x =
      omega_non_y * omega_cross_p_z - omega_non_z * omega_cross_p_y;
  Expression omega_cross_omega_cross_p_y =
      omega_non_z * omega_cross_p_x - omega_non_x * omega_cross_p_z;
  Expression omega_cross_omega_cross_p_z =
      omega_non_x * omega_cross_p_y - omega_non_y * omega_cross_p_x;
  Expression centrifugal_x = -omega_cross_omega_cross_p_x;
  Expression centrifugal_y = -omega_cross_omega_cross_p_y;
  Expression centrifugal_z = -omega_cross_omega_cross_p_z;

  // Frame-origin acceleration term: -a_car_non
  Expression frame_acc_x = -a_car_non_x;
  Expression frame_acc_y = -a_car_non_y;
  Expression frame_acc_z = -a_car_non_z;

  Expression non_inertial_acc_x =
      coriolis_x + euler_x + centrifugal_x + frame_acc_x;
  Expression non_inertial_acc_y =
      coriolis_y + euler_y + centrifugal_y + frame_acc_y;
  Expression non_inertial_acc_z =
      coriolis_z + euler_z + centrifugal_z + frame_acc_z;

  // System Dynamics
  f << dot(p_x) ==  v_x;
  f << dot(p_y) ==  v_y;
  f << dot(p_z) ==  v_z;
  f << dot(v_x) ==  acc_x + non_inertial_acc_x;
  f << dot(v_y) ==  acc_y + non_inertial_acc_y;
  f << dot(v_z) ==  acc_z + non_inertial_acc_z;
  f << dot(yaw) == yaw_rate_cmd;

  // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
  //
  // For the planar position tracking terms, the reference is passed via
  // OnlineData instead of acadoVariables.y. We embed the confidence decay in h:
  //
  //   J_xy = \sum_{k=0}^{N-1} \lambda^k e_k^\top Q_{xy} e_k,
  //   e_k = [p_x(k)-ref_x(k), p_y(k)-ref_y(k)]^\top.
  //
  // ACADO LSQ evaluates h_k^\top W h_k, therefore the correct stage-wise factor
  // inside the residual is \sqrt{\lambda^k} rather than \lambda^k:
  //
  //   (\sqrt{\lambda^k} e_k)^\top Q_{xy} (\sqrt{\lambda^k} e_k)
  //   = \lambda^k e_k^\top Q_{xy} e_k.
  //
  // If trust_factor were set to \lambda^k directly, the effective discount
  // would become \lambda^{2k}, which is not the intended objective.
  //
  // Running cost vector consists of trust-weighted planar tracking, the
  // remaining state tracking channels, and the control inputs.
  h << trust_factor * (p_x - ref_x)
    << trust_factor * (p_y - ref_y)
    << p_z
    << v_x << v_y << v_z
    << yaw
    << v_cmd_x << v_cmd_y << v_cmd_z << yaw_rate_cmd
    << delta0 << delta1 << delta2;

  // End cost vector consists of the trust-weighted planar terminal tracking
  // and the remaining state channels (no inputs at the terminal knot).
  hN << trust_factor * (p_x - ref_x)
     << trust_factor * (p_y - ref_y)
     << p_z
    << v_x << v_y << v_z
    << yaw;

  // Running cost weight matrix
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0,0) = 200;   // p_x
  Q(1,1) = 200;   // p_y
  Q(2,2) = 200;   // p_z
  Q(3,3) = 30;   // v_x 
  Q(4,4) = 30;   // v_y
  Q(5,5) = 30;   // v_z
  Q(6,6) = 20;   // yaw
  Q(7,7) = 1;    // v_cmd_x
  Q(8,8) = 1;    // v_cmd_y
  Q(9,9) = 1;    // v_cmd_z
  Q(10,10) = 1;  // yaw_rate_cmd
  Q(11,11) = 1;  // slack delta0
  Q(12,12) = 1;  // slack delta1
  Q(13,13) = 1;  // slack delta2

  // End cost weight matrix
  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  QN(0,0) = 200;   // p_x
  QN(1,1) = 200;   // p_y
  QN(2,2) = 200;   // p_z
  QN(3,3) = 30;   // v_x 
  QN(4,4) = 30;   // v_y
  QN(5,5) = 30;   // v_z
  QN(6,6) = 20;   // yaw
  // TODO: Insert offline LQR calculated values here

  // Set a reference for the analysis (if CODE_GEN is false).
  // The planar x/y references are injected via OnlineData and therefore their
  // LSQ references stay at zero in r and rN.
  DVector r(h.getDim());    // Running cost reference
  r.setZero();
  r(2) = 5.0;
  r(3) = 0.0;
  r(4) = 0.0;
  r(5) = 0.0;
  r(6) = 0.0;

  DVector rN(hN.getDim());   // End cost reference
  rN.setZero();
  rN(0) = r(0);
  rN(1) = r(1);
  rN(2) = r(2);
  rN(3) = r(3);
  rN(4) = r(4);
  rN(5) = r(5);
  rN(6) = r(6);


  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ----------------------------------
  OCP ocp( t_start, t_end, N );
  if(!CODE_GEN)
  {
    // For analysis, set references.
    ocp.minimizeLSQ( Q, h, r );
    ocp.minimizeLSQEndTerm( QN, hN, rN );
  }else{
    // For code generation, references are set during run time.
    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();
    ocp.minimizeLSQ( Q_sparse, h);
    ocp.minimizeLSQEndTerm( QN_sparse, hN );
  }

  // Add system dynamics
  ocp.subjectTo( f );
  ocp.setNU(7);
  // Add constraints
  ocp.subjectTo(-v_xy_max <= v_cmd_x <= v_xy_max);
  ocp.subjectTo(-v_xy_max <= v_cmd_y <= v_xy_max);
  ocp.subjectTo(-v_z_max <= v_cmd_z <= v_z_max);
  ocp.subjectTo(-yaw_rate_max <= yaw_rate_cmd <= yaw_rate_max);
  
  // Physical acceleration constraints (Mixed State-Control)
  ocp.subjectTo(-a_max_xy <= acc_x + non_inertial_acc_x <= a_max_xy);
  ocp.subjectTo(-a_max_xy <= acc_y + non_inertial_acc_y <= a_max_xy);
  ocp.subjectTo(-a_max_z  <= acc_z + non_inertial_acc_z <= a_max_z);

  ocp.subjectTo(delta0 >= 0.0);
  ocp.subjectTo(delta1 >= 0.0);
  ocp.subjectTo(delta2 >= 0.0);

  auto addObstacleCbfConstraint =
      [&](const OnlineData& obs_x, const OnlineData& obs_y,
          const OnlineData& obs_z, const OnlineData& obs_a,
          const OnlineData& obs_b, const OnlineData& obs_c,
          const OnlineData& obs_q00, const OnlineData& obs_q01,
          const OnlineData& obs_q02, const OnlineData& obs_q10,
          const OnlineData& obs_q11, const OnlineData& obs_q12,
          const OnlineData& obs_q20, const OnlineData& obs_q21,
          const OnlineData& obs_q22, const OnlineData& obs_vx,
          const OnlineData& obs_vy, const OnlineData& obs_vz,
          const OnlineData& obs_ax, const OnlineData& obs_ay,
          const OnlineData& obs_az, const OnlineData& obs_active,
          const Control& slack) {
        Expression rel_px = p_x - obs_x;
        Expression rel_py = p_y - obs_y;
        Expression rel_pz = p_z - obs_z;
        Expression rel_vx = v_x - obs_vx;
        Expression rel_vy = v_y - obs_vy;
        Expression rel_vz = v_z - obs_vz;
        Expression rel_ax = acc_x + non_inertial_acc_x - obs_ax;
        Expression rel_ay = acc_y + non_inertial_acc_y - obs_ay;
        Expression rel_az = acc_z + non_inertial_acc_z - obs_az;

        // q = Q^T (p - c), where Q is supplied row-major as the local-to-
        // solver-frame rotation matrix.
        Expression xi = obs_q00 * rel_px + obs_q10 * rel_py + obs_q20 * rel_pz;
        Expression eta = obs_q01 * rel_px + obs_q11 * rel_py + obs_q21 * rel_pz;
        Expression zeta = obs_q02 * rel_px + obs_q12 * rel_py + obs_q22 * rel_pz;
        Expression xidot = obs_q00 * rel_vx + obs_q10 * rel_vy + obs_q20 * rel_vz;
        Expression etadot = obs_q01 * rel_vx + obs_q11 * rel_vy + obs_q21 * rel_vz;
        Expression zetadot = obs_q02 * rel_vx + obs_q12 * rel_vy + obs_q22 * rel_vz;
        
        // p=4 superellipsoid barrier.  Axes and Q are OnlineData constants at
        // this node, so only the UAV position/velocity/acceleration are
        // differentiated.
        Expression inv_a4 = 1.0 / pow(obs_a, 4);
        Expression inv_b4 = 1.0 / pow(obs_b, 4);
        Expression inv_c4 = 1.0 / pow(obs_c, 4);
        Expression h = pow(xi, 4) * inv_a4 +
                       pow(eta, 4) * inv_b4 +
                       pow(zeta, 4) * inv_c4 - 1.0;
        Expression hdot = 4.0 * pow(xi, 3) * xidot * inv_a4 +
                          4.0 * pow(eta, 3) * etadot * inv_b4 +
                          4.0 * pow(zeta, 3) * zetadot * inv_c4;
        Expression hddot =
            12.0 * pow(xi, 2) * pow(xidot, 2) * inv_a4 +
            12.0 * pow(eta, 2) * pow(etadot, 2) * inv_b4 +
            12.0 * pow(zeta, 2) * pow(zetadot, 2) * inv_c4 +
            4.0 * pow(xi, 3) *
                (obs_q00 * rel_ax + obs_q10 * rel_ay + obs_q20 * rel_az) * inv_a4 +
            4.0 * pow(eta, 3) *
                (obs_q01 * rel_ax + obs_q11 * rel_ay + obs_q21 * rel_az) * inv_b4 +
            4.0 * pow(zeta, 3) *
                (obs_q02 * rel_ax + obs_q12 * rel_ay + obs_q22 * rel_az) * inv_c4;

        Expression cbf2 = hddot + (cbf_alpha1 + cbf_alpha2) * hdot + (cbf_alpha1 * cbf_alpha2) * h;
        ocp.subjectTo(obs_active * (cbf2 + slack) >= 0.0);
        // HOCBF is a differential condition.  In sampled MPC, also enforce
        // membership in the safe set at every shooting node so a warm-started
        // trajectory cannot jump across the p=4 boundary between nodes.
        ocp.subjectTo(obs_active * h >= 0.0);
      };
  addObstacleCbfConstraint(
      obs0_x, obs0_y, obs0_z, obs0_a, obs0_b, obs0_c,
      obs0_q00, obs0_q01, obs0_q02, obs0_q10, obs0_q11, obs0_q12,
      obs0_q20, obs0_q21, obs0_q22, obs0_vx, obs0_vy, obs0_vz,
      obs0_ax, obs0_ay, obs0_az, obs0_active,
      delta0);
  addObstacleCbfConstraint(
      obs1_x, obs1_y, obs1_z, obs1_a, obs1_b, obs1_c,
      obs1_q00, obs1_q01, obs1_q02, obs1_q10, obs1_q11, obs1_q12,
      obs1_q20, obs1_q21, obs1_q22, obs1_vx, obs1_vy, obs1_vz,
      obs1_ax, obs1_ay, obs1_az, obs1_active,
      delta1);
  addObstacleCbfConstraint(
      obs2_x, obs2_y, obs2_z, obs2_a, obs2_b, obs2_c,
      obs2_q00, obs2_q01, obs2_q02, obs2_q10, obs2_q11, obs2_q12,
      obs2_q20, obs2_q21, obs2_q22, obs2_vx, obs2_vy, obs2_vz,
      obs2_ax, obs2_ay, obs2_az, obs2_active,
      delta2);

  // Coefficients are assembled outside ACADO at the current nominal
  // trajectory. Multiplication by field_active makes an empty/stale field a
  // no-op while preserving one fixed generated path-constraint row.
  ocp.subjectTo(field_active *
                (field_Ax * v_cmd_x + field_Ay * v_cmd_y +
                 field_Az * v_cmd_z + delta0 - field_b) >= 0.0);

  // Online data layout (ACADO_NOD):
  // Each region occupies 22 values: center(3), axes(3), Q row-major(9),
  // center velocity(3), center acceleration(3), active(1).
  // [66..67] alpha1/alpha2, [68..76] non-inertial data,
  // [77..79] ref_x/ref_y/trust_factor, [80..84] field Ax/Ay/Az/b/active.
  ocp.setNOD(85);


  if(!CODE_GEN)
  {
    // Set initial state
    ocp.subjectTo( AT_START, p_x ==  0.0 );
    ocp.subjectTo( AT_START, p_y ==  0.0 );
    ocp.subjectTo( AT_START, p_z ==  0.0 );
    ocp.subjectTo( AT_START, v_x ==  0.0 );
    ocp.subjectTo( AT_START, v_y ==  0.0 );
    ocp.subjectTo( AT_START, v_z ==  0.0 );
    ocp.subjectTo( AT_START, yaw ==  0.0 );

    // Setup some visualization
    GnuplotWindow window1( PLOT_AT_EACH_ITERATION );
    window1.addSubplot( p_x,"position x" );
    window1.addSubplot( p_y,"position y" );
    window1.addSubplot( p_z,"position z" );
    window1.addSubplot( v_x,"verlocity x" );
    window1.addSubplot( v_y,"verlocity y" );
    window1.addSubplot( v_z,"verlocity z" );
    window1.addSubplot( yaw,"yaw" );

    VariablesGrid states, parameters, controls;


    // Define an algorithm to solve it.
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( INTEGRATOR_TOLERANCE, 1e-3 );
    algorithm.set( KKT_TOLERANCE, 1e-5 );
    algorithm << window1;
    algorithm.solve();

    algorithm.getDifferentialStates(states);
    algorithm.getParameters(parameters);
    algorithm.getControls(controls);
    // states.print();
    // parameters.print();
    // controls.print();

  }else{
    // For code generation, we can set some properties.
    // The main reason for a setting is given as comment.
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
    mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
    mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING);     // more robust than FULL_CONDENSING_N2 for larger horizons
    mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
    // tau_v = 0.01 makes the velocity channel much stiffer than the original
    // export. Use a finer integration grid per shooting interval so the RTI
    // linearization stays numerically well-behaved on the longer horizon.
    mpc.set(NUM_INTEGRATOR_STEPS,   4 * N);
    mpc.set(USE_SINGLE_PRECISION,   NO);                  // improve QP robustness
    mpc.set(QP_SOLVER,              QP_QPOASES3);         // embedded qpOASES variant is more robust for the longer horizon
    mpc.set(HOTSTART_QP,            NO);                  // avoid qpOASES hotstart crash at larger condensed QPs
    mpc.set(LEVENBERG_MARQUARDT,    10.0);                // Regularization for larger condensed QPs
    mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs

    // Do not generate tests, makes or matlab-related interfaces.
    mpc.set( GENERATE_TEST_FILE,          NO);
    mpc.set( GENERATE_MAKE_FILE,          NO);
    mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
    mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

    // Finally, export everything.
    if(mpc.exportCode("quadrotor_mpc_codegen") != SUCCESSFUL_RETURN)
      exit( EXIT_FAILURE );
    mpc.printDimensionsQP( );
  }

  return EXIT_SUCCESS;
}
