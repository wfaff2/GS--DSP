// Created by Baozhe Zhang on Oct 11, 2022

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

#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <cstring>
#include <mutex>
#include <vector>
#include <limits>

namespace acado_mpc {

#include "acado_auxiliary_functions.h"
#include "acado_common.h"

std::recursive_mutex& acadoGlobalSolverMutex();

static constexpr int kSamples = ACADO_N;      // number of samples
static constexpr int kStateSize = ACADO_NX;   // number of states
static constexpr int kRefSize = ACADO_NY;     // number of reference states
static constexpr int kEndRefSize = ACADO_NYN; // number of end reference states
static constexpr int kInputSize = ACADO_NU;   // number of inputs
static constexpr int kCostSize = ACADO_NY - ACADO_NU; // number of state costs
static constexpr int kOdSize = ACADO_NOD;     // number of online data
static constexpr int kPathConstraintSize = ACADO_NPAC; // number of path constraints
// Must match the obstacle count exported by acado_model/quadrotor_model_thrustrates.cpp.
static constexpr int kMaxObstacles = 3;
// Must match the mixed acceleration constraints exported by
// acado_model/quadrotor_model_thrustrates.cpp.
static constexpr int kAccelerationConstraintSize = 3;
static constexpr int kCbfConstraintOffset = kAccelerationConstraintSize;
static constexpr int kCbfConstraintSize = kMaxObstacles;
static constexpr int kSafeSetConstraintOffset =
    kCbfConstraintOffset + kCbfConstraintSize;
static constexpr int kSafeSetConstraintSize = kMaxObstacles;
static constexpr int kFieldConstraintOffset =
    kSafeSetConstraintOffset + kSafeSetConstraintSize;
static constexpr int kFieldConstraintSize = 1;
// Must match the discretization exported by the ACADO model source.
static constexpr double kModelDt = 0.1;
static_assert(kPathConstraintSize == kAccelerationConstraintSize +
                  kCbfConstraintSize + kSafeSetConstraintSize +
                  kFieldConstraintSize,
              "MPC: Path-constraint size does not match the model layout.");
// Must match the model-side online-data layout exported by
// acado_model/quadrotor_model_thrustrates.cpp:
// Each obstacle occupies center(3), axes(3), rotation(9), velocity(3),
// acceleration(3), active(1) = 22 entries.  The generated model then stores
// alpha1/alpha2, non-inertial data, planar tracking data, and the one affine
// field-HOCBF row, followed by inverse velocity tracking time constants.
static constexpr int kOdObstacleStride = 22;
static constexpr int kOdObstacleOffset = 0;
static constexpr int kOdAlpha1Index =
    kOdObstacleOffset + kMaxObstacles * kOdObstacleStride;
static constexpr int kOdAlpha2Index = kOdAlpha1Index + 1;
static constexpr int kOdOmegaNonOffset = kOdAlpha2Index + 1;
static constexpr int kOdBetaNonOffset = kOdOmegaNonOffset + 3;
static constexpr int kOdCarAccOffset = kOdBetaNonOffset + 3;
static constexpr int kOdRefXIndex = kOdCarAccOffset + 3;
static constexpr int kOdRefYIndex = kOdRefXIndex + 1;
static constexpr int kOdTrustFactorIndex = kOdRefYIndex + 1;
static constexpr int kOdFieldOffset = kOdTrustFactorIndex + 1;
static constexpr int kOdInvTauVxyIndex = kOdFieldOffset + 5;
static constexpr int kOdInvTauVzIndex = kOdInvTauVxyIndex + 1;
static_assert(kOdTrustFactorIndex < kOdSize,
              "MPC: ACADO online data size does not match model layout.");
static_assert(kOdInvTauVzIndex < kOdSize,
              "MPC: ACADO online-data layout does not fit ACADO_NOD.");

/**
 * @brief Wrapper for the ACADO MPC implementation
 * 
 * @tparam T scalar type
 */
template <typename T>
class MpcWrapper
{
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using AcadoScalar = real_t;
  using Obstacle = Eigen::Matrix<T, 7, 1>;
  using ObstacleVector =
      std::vector<Obstacle, Eigen::aligned_allocator<Obstacle>>;
  using ObstacleProfile = Eigen::Matrix<T, 7, kSamples + 1>;
  using Vec3Profile = Eigen::Matrix<T, 3, kSamples + 1>;
  using ObstacleProfileVector =
      std::vector<ObstacleProfile, Eigen::aligned_allocator<ObstacleProfile>>;
  // Ax, Ay, Az, b, active, diagnostic slack weight.
  using FieldHocbfProfile = Eigen::Matrix<T, 6, kSamples + 1>;

  MpcWrapper();
  MpcWrapper(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R);

  bool setCosts(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
    const T state_cost_scaling = 0.0, const T input_cost_scaling = 0.0);
  bool setCosts(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
    const Eigen::Ref<const Eigen::Matrix<T, kEndRefSize, kEndRefSize>> WN,
    const T state_cost_scaling = 0.0, const T input_cost_scaling = 0.0);

  bool setLimits(T max_v_xy, T max_v_z, T max_yaw_rate,
    T a_max_xy, T a_max_z,
    T slack_max = std::numeric_limits<T>::infinity());
  bool setVelocityTrackingTimeConstants(T tau_v_xy, T tau_v_z);
  bool setObstacles(
    const ObstacleVector& obstacles,
    const ObstacleProfileVector& obstacle_profiles,
    T alpha1, T alpha2, bool enabled);
  bool setFieldHocbf(const FieldHocbfProfile& profile, bool enabled);
  bool setNonInertialData(
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& omega_non,
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& beta_non,
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& a_car_non);
  bool setNonInertialDataProfile(
    const Eigen::Ref<const Vec3Profile>& omega_non_profile,
    const Eigen::Ref<const Vec3Profile>& beta_non_profile,
    const Eigen::Ref<const Vec3Profile>& a_car_non_profile);
  // bool setCameraParameters(
  //   const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& p_B_C,
  //   Eigen::Quaternion<T>& q_B_C);
  // bool setPointOfInterest(
  //   const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& position);

  bool setReferencePose(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state);
  bool setTrajectory(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples+1>> states,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples+1>> inputs);
  bool setTrackingTrustTimeDampingLambda(T tracking_trust_time_damping_lambda);

  bool solve(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state);
  bool update(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
              bool do_preparation = true);
  bool prepare();

  // Cached objective after the last solve/update call (thread-safe to read).
  T getObjective() const { return static_cast<T>(cached_objective_); }

  void getState(const int node_index,
    Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> return_state);
  void getStates(
    Eigen::Ref<Eigen::Matrix<T, kStateSize, kSamples+1>> return_states);
  void getInput(const int node_index,
    Eigen::Ref<Eigen::Matrix<T, kInputSize, 1>> return_input);
  void getInputs(
    Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_input);
  Eigen::Matrix<T, kInputSize, 1> getPreviousInput() const {
    return previous_input_;
  }
  bool setPreviousInput(
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>>& input) {
    if (!input.allFinite()) return false;
    previous_input_ = input;
    return true;
  }
  T getTimestep() const { return dt_; }

 private:
  struct ScopedContext {
    explicit ScopedContext(MpcWrapper& wrapper)
        : wrapper_(wrapper),
          lock_(acadoGlobalSolverMutex()) {
      std::memcpy(&acadoVariables, &wrapper_.acado_variables_storage_,
                  sizeof(ACADOvariables));
      std::memcpy(&acadoWorkspace, &wrapper_.acado_workspace_storage_,
                  sizeof(ACADOworkspace));
    }

    ~ScopedContext() {
      std::memcpy(&wrapper_.acado_variables_storage_, &acadoVariables,
                  sizeof(ACADOvariables));
      std::memcpy(&wrapper_.acado_workspace_storage_, &acadoWorkspace,
                  sizeof(ACADOworkspace));
    }

   private:
    MpcWrapper& wrapper_;
    std::unique_lock<std::recursive_mutex> lock_;
  };

  // Helper that assumes this wrapper's ACADO context is already bound.
  bool updateLocked(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
                    bool do_preparation);
  void setTrajectoryReferenceDataLocked(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples + 1>> inputs);
  void setPlanarTrustReferenceOnlineDataLocked(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states);

  Eigen::Map<Eigen::Matrix<AcadoScalar, kRefSize, kSamples, Eigen::ColMajor>>
    acado_reference_states_{acadoVariables.y};

  Eigen::Map<Eigen::Matrix<AcadoScalar, kEndRefSize, 1, Eigen::ColMajor>>
    acado_reference_end_state_{acadoVariables.yN};

  Eigen::Map<Eigen::Matrix<AcadoScalar, kStateSize, 1, Eigen::ColMajor>>
    acado_initial_state_{acadoVariables.x0};

  Eigen::Map<Eigen::Matrix<AcadoScalar, kStateSize, kSamples+1, Eigen::ColMajor>>
    acado_states_{acadoVariables.x};

  Eigen::Map<Eigen::Matrix<AcadoScalar, kInputSize, kSamples, Eigen::ColMajor>>
    acado_inputs_{acadoVariables.u};

  Eigen::Map<Eigen::Matrix<AcadoScalar, kOdSize, kSamples+1, Eigen::ColMajor>>
    acado_online_data_{acadoVariables.od};

  Eigen::Map<Eigen::Matrix<AcadoScalar, kRefSize, kRefSize * kSamples>>
    acado_W_{acadoVariables.W};

  Eigen::Map<Eigen::Matrix<AcadoScalar, kEndRefSize, kEndRefSize>>
    acado_W_end_{acadoVariables.WN};

  Eigen::Map<Eigen::Matrix<AcadoScalar, kInputSize, kSamples, Eigen::ColMajor>>
    acado_lower_bounds_{acadoVariables.lbValues};

  Eigen::Map<Eigen::Matrix<AcadoScalar, kInputSize, kSamples, Eigen::ColMajor>>
    acado_upper_bounds_{acadoVariables.ubValues};
  
  Eigen::Map<Eigen::Matrix<AcadoScalar, kPathConstraintSize, kSamples>>
    acado_state_lower_bounds_{acadoVariables.lbAValues};

  Eigen::Map<Eigen::Matrix<AcadoScalar, kPathConstraintSize, kSamples>>
    acado_state_upper_bounds_{acadoVariables.ubAValues};

  ACADOvariables acado_variables_storage_{};
  ACADOworkspace acado_workspace_storage_{};

  // Per-instance cached outputs to avoid reading ACADO globals outside locks.
  Eigen::Matrix<AcadoScalar, kStateSize, kSamples+1> cached_states_;
  Eigen::Matrix<AcadoScalar, kInputSize, kSamples> cached_inputs_;
  AcadoScalar cached_objective_{std::numeric_limits<AcadoScalar>::quiet_NaN()};
  Eigen::Matrix<AcadoScalar, kStateSize, kSamples + 1> reference_state_guess_;
  Eigen::Matrix<AcadoScalar, kInputSize, kSamples> reference_input_guess_;

  FieldHocbfProfile field_profile_ = FieldHocbfProfile::Zero();
  bool field_enabled_{false};
  Eigen::Matrix<T, kInputSize, 1> previous_input_ =
      Eigen::Matrix<T, kInputSize, 1>::Zero();

  Eigen::Matrix<T, kRefSize, kRefSize> W_ = (Eigen::Matrix<T, kRefSize, 1>() <<
    200, 200, 200,
    30, 30, 30,
    20,
    1, 1, 1,
    1,
    100, 100, 100).finished().asDiagonal();

  Eigen::Matrix<T, kEndRefSize, kEndRefSize> WN_ =
    W_.block(0, 0, kEndRefSize, kEndRefSize);

  bool acado_is_prepared_{false};
  T tracking_trust_time_damping_lambda_{static_cast<T>(0.0)};
  const T dt_{static_cast<T>(kModelDt)};
  const Eigen::Matrix<real_t, kInputSize, 1> kHoverInput_ =
    (Eigen::Matrix<real_t, kInputSize, 1>() << 0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 0.0).finished();
};
} // namespace acado_mpc
