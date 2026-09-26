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

#include "acado_mpc/mpc_wrapper.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>

namespace acado_mpc {

extern "C" {
ACADOworkspace acadoWorkspace;
ACADOvariables acadoVariables;
}

std::recursive_mutex& acadoGlobalSolverMutex() {
  static std::recursive_mutex solver_mutex;
  return solver_mutex;
}

template <typename T>
MpcWrapper<T>::MpcWrapper() {
  std::memset(&acado_workspace_storage_, 0, sizeof(ACADOworkspace));
  std::memset(&acado_variables_storage_, 0, sizeof(ACADOvariables));

  ScopedContext context(*this);

  std::memset(&acadoWorkspace, 0, sizeof(ACADOworkspace));
  std::memset(&acadoVariables, 0, sizeof(ACADOvariables));

  acado_initializeSolver();

  const Eigen::Matrix<T, kStateSize, 1> hover_state =
      (Eigen::Matrix<T, kStateSize, 1>() << 0.0, 0.0, 1.0,
                                            0.0, 0.0, 0.0,
                                            0.0)
          .finished();
  const Eigen::Matrix<T, kStateSize, kSamples + 1> hover_states =
      hover_state.replicate(1, kSamples + 1);
  const Eigen::Matrix<T, kInputSize, kSamples + 1> hover_inputs =
      kHoverInput_.template cast<T>().replicate(1, kSamples + 1);

  acado_initial_state_ = hover_state.template cast<AcadoScalar>();
  setTrajectoryReferenceDataLocked(hover_states, hover_inputs);
  acado_states_ = reference_state_guess_;
  acado_inputs_ = reference_input_guess_;

  if (!(acado_W_.trace() > static_cast<AcadoScalar>(0.0))) {
    acado_W_ = W_.replicate(1, kSamples).template cast<AcadoScalar>();
    acado_W_end_ = WN_.template cast<AcadoScalar>();
  }

  acado_preparationStep();
  acado_is_prepared_ = true;

  cached_states_ = acado_states_;
  cached_inputs_ = acado_inputs_;
  cached_objective_ = std::numeric_limits<AcadoScalar>::quiet_NaN();
}

template <typename T>
MpcWrapper<T>::MpcWrapper(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R)
    : MpcWrapper() {
  setCosts(Q, R);
}

template <typename T>
bool MpcWrapper<T>::setCosts(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
    const T state_cost_scaling, const T input_cost_scaling) {
  return setCosts(Q, R, Q, state_cost_scaling, input_cost_scaling);
}

template <typename T>
bool MpcWrapper<T>::setCosts(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
    const Eigen::Ref<const Eigen::Matrix<T, kEndRefSize, kEndRefSize>> WN,
    const T state_cost_scaling, const T input_cost_scaling) {
  ScopedContext context(*this);

  if (state_cost_scaling < 0.0 || input_cost_scaling < 0.0) {
    ROS_ERROR("MPC: Cost scaling is wrong, must be non-negative!");
    return false;
  }

  W_.block(0, 0, kCostSize, kCostSize) = Q;
  W_.block(kCostSize, kCostSize, kInputSize, kInputSize) = R;
  WN_ = WN;

  AcadoScalar state_scale = static_cast<AcadoScalar>(1.0);
  AcadoScalar input_scale = static_cast<AcadoScalar>(1.0);
  for (int i = 0; i < kSamples; ++i) {
    state_scale = std::exp(-static_cast<AcadoScalar>(i) / static_cast<AcadoScalar>(kSamples) *
                           static_cast<AcadoScalar>(state_cost_scaling));
    input_scale = std::exp(-static_cast<AcadoScalar>(i) / static_cast<AcadoScalar>(kSamples) *
                           static_cast<AcadoScalar>(input_cost_scaling));
    acado_W_.block(0, i * kRefSize, kCostSize, kCostSize) =
        W_.block(0, 0, kCostSize, kCostSize).template cast<AcadoScalar>() * state_scale;
    acado_W_.block(kCostSize, i * kRefSize + kCostSize, kInputSize, kInputSize) =
        W_.block(kCostSize, kCostSize, kInputSize, kInputSize).template cast<AcadoScalar>() *
        input_scale;
  }
  acado_W_end_ = WN_.template cast<AcadoScalar>() * state_scale;
  return true;
}

template <typename T>
bool MpcWrapper<T>::setLimits(T max_v_xy, T max_v_z, T max_yaw_rate,
                              T slack_max) {
  ScopedContext context(*this);

  if (max_v_xy <= 0.0 || max_v_z <= 0.0 || max_yaw_rate <= 0.0) {
    ROS_ERROR("MPC: Maximal command bounds are not set properly, not changed.");
    return false;
  }

  Eigen::Matrix<T, kInputSize, 1> lower_bounds = Eigen::Matrix<T, kInputSize, 1>::Zero();
  Eigen::Matrix<T, kInputSize, 1> upper_bounds = Eigen::Matrix<T, kInputSize, 1>::Zero();
  Eigen::Matrix<T, kPathConstraintSize, 1> path_lower_bounds =
      Eigen::Matrix<T, kPathConstraintSize, 1>::Zero();
  Eigen::Matrix<T, kPathConstraintSize, 1> path_upper_bounds =
      Eigen::Matrix<T, kPathConstraintSize, 1>::Zero();

  T slack_upper = slack_max;
  if (slack_upper < static_cast<T>(0.0) ||
      !std::isfinite(static_cast<double>(slack_upper))) {
    slack_upper = static_cast<T>(std::numeric_limits<AcadoScalar>::max());
  }

  lower_bounds << -max_v_xy, -max_v_xy, -max_v_z, -max_yaw_rate,
                  static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0);
  upper_bounds << max_v_xy, max_v_xy, max_v_z, max_yaw_rate,
                  slack_upper, slack_upper, slack_upper;
  path_lower_bounds << static_cast<T>(-kModelMaxAccXy),
      static_cast<T>(-kModelMaxAccXy),
      static_cast<T>(-kModelMaxAccZ),
      static_cast<T>(0.0),
      static_cast<T>(0.0),
      static_cast<T>(0.0);
  path_upper_bounds << static_cast<T>(kModelMaxAccXy),
      static_cast<T>(kModelMaxAccXy),
      static_cast<T>(kModelMaxAccZ),
      static_cast<T>(1.0e12),
      static_cast<T>(1.0e12),
      static_cast<T>(1.0e12);

  acado_lower_bounds_ = lower_bounds.replicate(1, kSamples).template cast<AcadoScalar>();
  acado_upper_bounds_ = upper_bounds.replicate(1, kSamples).template cast<AcadoScalar>();
  acado_state_lower_bounds_ =
      path_lower_bounds.replicate(1, kSamples).template cast<AcadoScalar>();
  acado_state_upper_bounds_ =
      path_upper_bounds.replicate(1, kSamples).template cast<AcadoScalar>();
  return true;
}

template <typename T>
bool MpcWrapper<T>::setObstacles(const ObstacleVector& obstacles,
                                 const ObstacleProfileVector& obstacle_profiles,
                                 T alpha1, T alpha2, bool enabled) {
  ScopedContext context(*this);

  for (int row = kOdObstacleOffset; row <= kOdAlpha2Index; ++row) {
    acado_online_data_.row(row).setZero();
  }

  if (obstacles.size() > static_cast<std::size_t>(kMaxObstacles)) {
    ROS_WARN("MPC: Received %zu obstacles, only the closest %d will be used.",
             obstacles.size(), kMaxObstacles);
  }

  const int obstacle_count =
      std::min<int>(static_cast<int>(obstacles.size()), kMaxObstacles);
  for (int i = 0; i < kMaxObstacles; ++i) {
    const int row_offset = kOdObstacleOffset + i * kOdObstacleStride;
    if (enabled && i < obstacle_count) {
      const Obstacle& obs = obstacles.at(i);
      if (i < static_cast<int>(obstacle_profiles.size())) {
        const ObstacleProfile& profile = obstacle_profiles.at(i);
        for (int dim = 0; dim < 7; ++dim) {
          acado_online_data_.row(row_offset + dim) =
              profile.row(dim).template cast<AcadoScalar>();
        }
        acado_online_data_.row(row_offset + 7).setConstant(static_cast<AcadoScalar>(1.0));
      } else {
        for (int dim = 0; dim < 7; ++dim) {
          acado_online_data_.row(row_offset + dim)
              .setConstant(static_cast<AcadoScalar>(obs(dim)));
        }
        acado_online_data_.row(row_offset + 7).setConstant(static_cast<AcadoScalar>(1.0));
      }
    } else {
      for (int dim = 0; dim < kOdObstacleStride; ++dim) {
        acado_online_data_.row(row_offset + dim).setZero();
      }
    }
  }

  acado_online_data_.row(kOdAlpha1Index).setConstant(
      enabled ? static_cast<AcadoScalar>(alpha1) : static_cast<AcadoScalar>(0.0));
  acado_online_data_.row(kOdAlpha2Index).setConstant(
      enabled ? static_cast<AcadoScalar>(alpha2) : static_cast<AcadoScalar>(0.0));
  return true;
}

template <typename T>
bool MpcWrapper<T>::setNonInertialData(
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& omega_non,
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& beta_non,
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& a_car_non) {
  Vec3Profile omega_non_profile =
      omega_non.replicate(1, kSamples + 1).eval();
  Vec3Profile beta_non_profile =
      beta_non.replicate(1, kSamples + 1).eval();
  Vec3Profile a_car_non_profile =
      a_car_non.replicate(1, kSamples + 1).eval();
  return setNonInertialDataProfile(omega_non_profile,
                                   beta_non_profile,
                                   a_car_non_profile);
}

template <typename T>
bool MpcWrapper<T>::setNonInertialDataProfile(
    const Eigen::Ref<const Vec3Profile>& omega_non_profile,
    const Eigen::Ref<const Vec3Profile>& beta_non_profile,
    const Eigen::Ref<const Vec3Profile>& a_car_non_profile) {
  ScopedContext context(*this);

  for (int k = 0; k < kSamples + 1; ++k) {
    for (int axis = 0; axis < 3; ++axis) {
      acado_online_data_(kOdOmegaNonOffset + axis, k) =
          static_cast<AcadoScalar>(omega_non_profile(axis, k));
      acado_online_data_(kOdBetaNonOffset + axis, k) =
          static_cast<AcadoScalar>(beta_non_profile(axis, k));
      acado_online_data_(kOdCarAccOffset + axis, k) =
          static_cast<AcadoScalar>(a_car_non_profile(axis, k));
    }
  }
  return true;
}

template <typename T>
bool MpcWrapper<T>::setTrackingTrustTimeDampingLambda(
    T tracking_trust_time_damping_lambda) {
  if (!(tracking_trust_time_damping_lambda >= static_cast<T>(0.0)) ||
      !std::isfinite(static_cast<double>(tracking_trust_time_damping_lambda))) {
    ROS_ERROR("MPC: tracking_trust_time_damping_lambda must satisfy lambda >= 0.");
    return false;
  }
  tracking_trust_time_damping_lambda_ = tracking_trust_time_damping_lambda;
  return true;
}

template <typename T>
void MpcWrapper<T>::setPlanarTrustReferenceOnlineDataLocked(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states) {
  for (int k = 0; k < kSamples + 1; ++k) {
    acado_online_data_(kOdRefXIndex, k) =
        static_cast<AcadoScalar>(states(0, k));
    acado_online_data_(kOdRefYIndex, k) =
        static_cast<AcadoScalar>(states(1, k));
    const double horizon_time = static_cast<double>(k) * static_cast<double>(dt_);
    const double trust_factor = std::exp(
        -0.5 * static_cast<double>(tracking_trust_time_damping_lambda_) *
        horizon_time);
    acado_online_data_(kOdTrustFactorIndex, k) =
        static_cast<AcadoScalar>(trust_factor);
  }
}

template <typename T>
void MpcWrapper<T>::setTrajectoryReferenceDataLocked(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples + 1>> inputs) {
  reference_state_guess_ = states.template cast<AcadoScalar>();
  reference_input_guess_ =
      inputs.block(0, 0, kInputSize, kSamples).template cast<AcadoScalar>();

  acado_reference_states_.block(0, 0, kStateSize, kSamples) =
      states.block(0, 0, kStateSize, kSamples).template cast<AcadoScalar>();
  acado_reference_states_.block(kStateSize, 0, kInputSize, kSamples) =
      inputs.block(0, 0, kInputSize, kSamples).template cast<AcadoScalar>();
  // p_x and p_y references are encoded directly in h through OnlineData.
  acado_reference_states_.row(0).setZero();
  acado_reference_states_.row(1).setZero();

  acado_reference_end_state_.setZero();
  acado_reference_end_state_.segment(0, kStateSize) =
      states.col(kSamples).template cast<AcadoScalar>();
  acado_reference_end_state_(0) = static_cast<AcadoScalar>(0.0);
  acado_reference_end_state_(1) = static_cast<AcadoScalar>(0.0);

  setPlanarTrustReferenceOnlineDataLocked(states);
}

template <typename T>
bool MpcWrapper<T>::setReferencePose(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state) {
  ScopedContext context(*this);
  const Eigen::Matrix<T, kStateSize, kSamples + 1> states =
      state.replicate(1, kSamples + 1);
  const Eigen::Matrix<T, kInputSize, kSamples + 1> inputs =
      kHoverInput_.template cast<T>().replicate(1, kSamples + 1);
  setTrajectoryReferenceDataLocked(states, inputs);
  acado_states_ = reference_state_guess_;
  acado_inputs_ = reference_input_guess_;
  cached_states_ = acado_states_;
  cached_inputs_ = acado_inputs_;
  return true;
}

template <typename T>
bool MpcWrapper<T>::setTrajectory(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples + 1>> inputs) {
  ScopedContext context(*this);
  setTrajectoryReferenceDataLocked(states, inputs);
  return true;
}

template <typename T>
bool MpcWrapper<T>::solve(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state) {
  ScopedContext context(*this);

  // Cold-start from the stage reference profile and re-run preparation on this
  // freshly seeded trajectory before the first feedback step. Without this,
  // the first QP can still be built from a stale linearization left by the
  // previous preparation pass, which becomes fragile on longer horizons.
  acado_states_ = reference_state_guess_;
  acado_states_.col(0) = state.template cast<AcadoScalar>();
  acado_inputs_ = reference_input_guess_;
  acado_initial_state_ = state.template cast<AcadoScalar>();

  acado_preparationStep();
  acado_is_prepared_ = true;

  const bool ok = updateLocked(state, /*do_preparation=*/true);
  cached_states_ = acado_states_;
  cached_inputs_ = acado_inputs_;
  cached_objective_ = acado_getObjective();
  return ok;
}

template <typename T>
bool MpcWrapper<T>::update(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    bool do_preparation) {
  ScopedContext context(*this);

  const bool ok = updateLocked(state, do_preparation);
  cached_states_ = acado_states_;
  cached_inputs_ = acado_inputs_;
  cached_objective_ = acado_getObjective();
  return ok;
}

template <typename T>
bool MpcWrapper<T>::updateLocked(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    bool do_preparation) {
  if (!acado_is_prepared_) {
    ROS_WARN("MPC: Solver was triggered without preparation, abort!");
    return false;
  }

  acado_initial_state_ = state.template cast<AcadoScalar>();
  acado_feedbackStep();
  acado_is_prepared_ = false;

  if (do_preparation) {
    acado_preparationStep();
    acado_is_prepared_ = true;
  }
  return true;
}

template <typename T>
bool MpcWrapper<T>::prepare() {
  ScopedContext context(*this);

  acado_preparationStep();
  acado_is_prepared_ = true;
  cached_states_ = acado_states_;
  cached_inputs_ = acado_inputs_;
  return true;
}

template <typename T>
void MpcWrapper<T>::getState(
    const int node_index, Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> return_state) {
  return_state = cached_states_.col(node_index).cast<T>();
}

template <typename T>
void MpcWrapper<T>::getStates(
    Eigen::Ref<Eigen::Matrix<T, kStateSize, kSamples + 1>> return_states) {
  return_states = cached_states_.cast<T>();
}

template <typename T>
void MpcWrapper<T>::getInput(
    const int node_index, Eigen::Ref<Eigen::Matrix<T, kInputSize, 1>> return_input) {
  return_input = cached_inputs_.col(node_index).cast<T>();
}

template <typename T>
void MpcWrapper<T>::getInputs(
    Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_inputs) {
  return_inputs = cached_inputs_.cast<T>();
}

template class MpcWrapper<float>;
template class MpcWrapper<double>;

}  // namespace acado_mpc
