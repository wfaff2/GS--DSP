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


#include "acado_mpc/mpc_controller.h"

#include <ctime>
#include <cmath>

namespace acado_mpc {

namespace {
template <typename Derived>
bool containsNonFinite(const Eigen::MatrixBase<Derived>& matrix) {
  return !matrix.allFinite();
}

template <typename T>
T wrapAngle(T angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

template <typename T>
T unwrapAngleNear(T angle, T reference) {
  return reference + wrapAngle(angle - reference);
}

template <typename T>
T quaternionToYaw(const Eigen::Quaternion<T>& q_in) {
  Eigen::Quaternion<T> q = q_in.normalized();
  const T siny_cosp =
      static_cast<T>(2.0) * (q.w() * q.z() + q.x() * q.y());
  const T cosy_cosp =
      static_cast<T>(1.0) -
      static_cast<T>(2.0) * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

template <typename T>
T aggregateObstacleSlack(
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>>& input) {
  return input.template segment<3>(INPUT::kSlack0).sum();
}
}  // namespace

template<typename T>
MpcController<T>::MpcController(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const int id,
    const std::string& topic, const std::string& topic_world) :
    nh_(nh),
    pnh_(pnh),
    mpc_wrapper_(MpcWrapper<T>()),
    timing_feedback_(T(1e-3)),
    timing_preparation_(T(1e-3)),
    solve_from_scratch_(true),
    warm_start_(true),
    est_state_(Eigen::Matrix<T, kStateSize, 1>::Zero()),
    reference_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    reference_inputs_(Eigen::Matrix<T, kInputSize, kSamples + 1>::Zero()),
    predicted_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    predicted_inputs_(Eigen::Matrix<T, kInputSize, kSamples>::Zero()),
    latest_slack_(T(0)),
    last_feedback_solve_time_sec_(0.0),
    last_preparation_time_sec_(0.0),
    last_solve_ok_(true),
    nonfinite_recovery_count_(0u),
    obstacle_update_fail_count_(0u),
    last_valid_control_(),
    has_last_valid_control_(false) {
  const std::string predicted_topic =
      acado_mpc_common::resolveTopicName(topic + std::to_string(id));
  const std::string predicted_world_topic =
      acado_mpc_common::resolveTopicName(topic_world + std::to_string(id));
  const std::string predicted_world_marker_topic =
      acado_mpc_common::resolveTopicName(
          topic_world + "_marker" + std::to_string(id));
  pub_predicted_trajectory_ =
      nh_.advertise<nav_msgs::Path>(predicted_topic, 1);
  pub_predicted_trajectory_world_ = 
      nh_.advertise<nav_msgs::Path>(predicted_world_topic, 1);
  pub_predicted_trajectory_marker_world_ =
      nh_.advertise<visualization_msgs::Marker>(
          predicted_world_marker_topic, 1);


  if (!params_.loadParameters(pnh_)) {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
  setNewParams(params_);

  solve_from_scratch_ = true;
  preparationThread();
}

template<typename T>
MpcController<T>::~MpcController() = default;

template<typename T>
void MpcController<T>::setWarmStart(bool enable) {
  warm_start_ = enable;
  if (!warm_start_) {
    solve_from_scratch_ = true;
  }
}

template<typename T>
void MpcController<T>::requestSolveFromScratch(const std::string& reason) {
  solve_from_scratch_ = true;
  if (!reason.empty()) {
    ROS_INFO_STREAM_THROTTLE(1.0,
                             "[MpcController] forcing solve_from_scratch: "
                                 << reason);
  }
}

template<typename T>
void MpcController<T>::resetPredictionsToSafeState() {
  predicted_states_.colwise() = est_state_;
  predicted_inputs_.setZero();
}

template<typename T>
void MpcController<T>::buildLegacyCurrentTimeNonInertialData(
    const acado_mpc_common::QuadRelativeEstimate& state_estimate,
    Eigen::Matrix<T, 3, 1>& omega_non,
    Eigen::Matrix<T, 3, 1>& beta_non,
    Eigen::Matrix<T, 3, 1>& a_car_non) {
  omega_non = state_estimate.omega_non.template cast<T>();
  beta_non = state_estimate.beta_non.template cast<T>();
  a_car_non = state_estimate.a_imu.template cast<T>();
  // Legacy current-time path removes gravity just before pushing a_car_non
  // into the controller's non-inertial profile.
  a_car_non(2) -= static_cast<T>(9.81);
}


template<typename T>
acado_mpc_common::ControlCommand MpcController<T>::off() {
  return acado_mpc_common::ControlCommand();
}

template<typename T>
acado_mpc_common::ControlCommand MpcController<T>::run(
    const acado_mpc_common::QuadRelativeEstimate& state_estimate,
    const acado_mpc_common::RelativeTrajectory& reference_trajectory,
    const MpcParams<T>& params, 
    const Eigen::Quaterniond& W_q_non,
    const Eigen::Vector3d& W_p_non,
    const NonInertialProfile* omega_non_profile,
    const NonInertialProfile* beta_non_profile,
    const NonInertialProfile* a_car_non_profile) {

#define GREEN std::string("\033[1;42m")
#define RST std::string("\033[0m")

  auto makeFailureFallback = [this](const char* reason) {
    if (has_last_valid_control_) {
      ROS_WARN_STREAM_THROTTLE(
          0.5, "[MpcController] keeping previous valid control after "
                   << reason);
      return last_valid_control_;
    }
    ROS_WARN_STREAM_THROTTLE(
        0.5, "[MpcController] no previous valid control after " << reason
                                                                  << "; using zero command");
    return acado_mpc_common::ControlCommand();
  };

  ros::Time call_time = ros::Time::now();
  if (params.changed_) {
    params_ = params;
    setNewParams(params_);
  }
  // Convert everything into Eigen format.
  setStateEstimate(state_estimate);
  setReference(reference_trajectory);
  Eigen::Matrix<T, 3, 1> omega_non = Eigen::Matrix<T, 3, 1>::Zero();
  Eigen::Matrix<T, 3, 1> beta_non = Eigen::Matrix<T, 3, 1>::Zero();
  Eigen::Matrix<T, 3, 1> a_car_non = Eigen::Matrix<T, 3, 1>::Zero();
  buildLegacyCurrentTimeNonInertialData(
      state_estimate, omega_non, beta_non, a_car_non);
  bool debug_check_noninertial_profile_col0 = false;
  pnh_.param("debug/check_noninertial_profile_col0",
             debug_check_noninertial_profile_col0, false);
  if (debug_check_noninertial_profile_col0) {
    ROS_INFO_STREAM(
        "[NONINERTIAL_COL0_CHECK][LEGACY_PATH]"
        << " old_omega_non=" << omega_non.transpose()
        << " old_beta_non=" << beta_non.transpose()
        << " old_a_car_non=" << a_car_non.transpose()
        << " using_profile=" << std::boolalpha
        << (omega_non_profile != nullptr &&
            beta_non_profile != nullptr &&
            a_car_non_profile != nullptr));
  }
  if (omega_non_profile != nullptr &&
      beta_non_profile != nullptr &&
      a_car_non_profile != nullptr) {
    mpc_wrapper_.setNonInertialDataProfile(*omega_non_profile,
                                           *beta_non_profile,
                                           *a_car_non_profile);
  } else {
    mpc_wrapper_.setNonInertialData(omega_non, beta_non, a_car_non);
  }
  const bool field_active = params.cbf_use_field_hocbf_;
  const bool previous_field_enabled = last_field_enabled_;
  const bool obstacle_update_ok = mpc_wrapper_.setObstacles(
      params.cbf_obstacles_, params.cbf_obstacle_profiles_,
      params.cbf_alpha1_, params.cbf_alpha2_,
      params.cbf_enabled_ && !field_active);
  const bool field_update_ok = mpc_wrapper_.setFieldHocbf(
      params.cbf_field_hocbf_profile_, field_active);
  const bool field_mode_changed = field_active != last_field_enabled_;
  const bool field_changed = field_mode_changed ||
      !params.cbf_field_hocbf_profile_.isApprox(last_field_profile_, T(1e-12));
  if (field_changed) {
    // Honor the configured slack bound in every CBF mode; zero is hard.
    const T field_slack_limit = params.cbf_slack_max_;
    if (!mpc_wrapper_.setLimits(params.max_v_xy_, params.max_v_z_,
                                params.max_yaw_rate_, params.a_max_xy_,
                                params.a_max_z_, field_slack_limit)) {
      ROS_ERROR("MPC: Failed to update Field-HOCBF slack bounds.");
      ++obstacle_update_fail_count_;
      solve_from_scratch_ = true;
      last_solve_ok_ = false;
      preparationThread();
      return makeFailureFallback("Field-HOCBF slack-bound update failure");
    }
    if (field_mode_changed) {
      // A mode transition changes the path row and its slack bounds.  One
      // cold solve installs both atomically; subsequent coefficient updates
      // follow the normal prepared-for-next-cycle RTI pipeline.
      solve_from_scratch_ = true;
    }
    // Field coefficients are frozen at the latest nominal trajectory; the
    // preparation pass at the end of this cycle makes them available to the
    // next feedback step while retaining the current primal warm start.
    last_field_profile_ = params.cbf_field_hocbf_profile_;
    last_field_enabled_ = field_active;
  }
  if (!obstacle_update_ok || !field_update_ok) {
    ROS_ERROR("MPC: Failed to update CBF obstacles.");
    ++obstacle_update_fail_count_;
    solve_from_scratch_ = true;
    latest_slack_ = T(0);
    last_solve_ok_ = false;
    resetPredictionsToSafeState();
    ROS_ERROR("[MPC][RECOVERY] obstacle update failed -> force cold restart next cycle");
    preparationThread();
    return makeFailureFallback("CBF online-data update failure");
  }

  static const bool do_preparation_step(false);

  // Get the feedback from MPC.
  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);
  const clock_t solve_start = clock();
  const bool use_cold_restart = (!warm_start_ || solve_from_scratch_);
  bool solver_status = false;
  if (use_cold_restart) {
    if (warm_start_ && solve_from_scratch_) {
      ROS_INFO("Solving MPC with hover as initial guess.");
    }
    solver_status = mpc_wrapper_.solve(est_state_);
  } else {
    // Prepare the QP matrices using the current acadoVariables.x and the
    // freshly installed online data (Field-HOCBF and non-inertial profiles)
    // that were linearized around this exact same nominal trajectory.
    preparationThread();
    solver_status = mpc_wrapper_.update(est_state_, do_preparation_step);
  }
  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);
  const T objective = mpc_wrapper_.getObjective();
  const bool objective_nonfinite =
      !std::isfinite(static_cast<double>(objective));
  const bool states_nonfinite = containsNonFinite(predicted_states_);
  const bool inputs_nonfinite = containsNonFinite(predicted_inputs_);
  if (!solver_status || objective_nonfinite || states_nonfinite || inputs_nonfinite) {
    ++nonfinite_recovery_count_;
    solve_from_scratch_ = true;
    latest_slack_ = T(0);
    last_solve_ok_ = false;
    resetPredictionsToSafeState();
    ROS_ERROR("[MPC][RECOVERY] non-finite prediction/objective detected -> cold restart armed");
    ROS_ERROR_STREAM(
        "[MpcController] recovery diagnostics"
        << " | solver_status=" << std::boolalpha << solver_status
        << " | objective=" << objective
        << " | objective_nonfinite=" << objective_nonfinite
        << " | states_nonfinite=" << states_nonfinite
        << " | inputs_nonfinite=" << inputs_nonfinite
        << " | warm_start=" << warm_start_
        << " | requested_cold_restart=" << use_cold_restart
        << " | obstacle_update_fail_count=" << obstacle_update_fail_count_
        << " | nonfinite_recovery_count=" << nonfinite_recovery_count_);
    return makeFailureFallback("MPC solve failure or invalid prediction");
  }
  if (warm_start_) {
    solve_from_scratch_ = false;
  }
  latest_slack_ = aggregateObstacleSlack<T>(predicted_inputs_.col(0));
  if (field_active || previous_field_enabled) {
    const typename MpcWrapper<T>::FieldHocbfProfile& feedback_profile =
        params.cbf_field_hocbf_profile_;
    const bool feedback_field_enabled = field_active;
    const bool current_stage0_active =
        field_active &&
        params.cbf_field_hocbf_profile_(4, 0) > static_cast<T>(0.5) &&
        params.cbf_field_hocbf_profile_.col(0).allFinite();
    const bool feedback_stage0_active =
        feedback_field_enabled &&
        feedback_profile(4, 0) > static_cast<T>(0.5) &&
        feedback_profile.col(0).allFinite();
    Eigen::Matrix<T, 3, 1> current_A = Eigen::Matrix<T, 3, 1>::Zero();
    if (current_stage0_active) {
      current_A = params.cbf_field_hocbf_profile_.template block<3, 1>(0, 0);
    }
    const T current_b = current_stage0_active
        ? params.cbf_field_hocbf_profile_(3, 0) : static_cast<T>(0.0);
    Eigen::Matrix<T, 3, 1> feedback_A = Eigen::Matrix<T, 3, 1>::Zero();
    if (feedback_stage0_active) {
      feedback_A = feedback_profile.template block<3, 1>(0, 0);
    }
    const T feedback_b = feedback_stage0_active
        ? feedback_profile(3, 0) : static_cast<T>(0.0);
    const Eigen::Matrix<T, 3, 1> raw_u_xyz =
        predicted_inputs_.template block<3, 1>(0, 0);
    const T raw_slack0 = predicted_inputs_(INPUT::kSlack0, 0);
    const T current_residual = current_A.dot(raw_u_xyz) + raw_slack0 - current_b;
    const T feedback_residual =
        feedback_A.dot(raw_u_xyz) + raw_slack0 - feedback_b;
    ROS_DEBUG_STREAM_NAMED(
        "field_hocbf",
        "[FIELD_HOCBF_SOLVE] feedback_profile_source="
            << (use_cold_restart ? "cold_current_prepare"
                                 : "warm_synchronized_prepare")
            << " current_A=" << current_A.transpose()
            << " current_b=" << current_b
            << " current_active=" << std::boolalpha << current_stage0_active
            << " current_residual=" << current_residual
            << " feedback_A=" << feedback_A.transpose()
            << " feedback_b=" << feedback_b
            << " feedback_active=" << feedback_stage0_active
            << " feedback_residual=" << feedback_residual
            << " raw_u_xyz=" << raw_u_xyz.transpose()
            << " raw_slack0=" << raw_slack0);
  }
  ROS_INFO_STREAM_THROTTLE(
      1.0,
      "[MpcController] pred_x0=(" << predicted_states_(kPosX, 0) << ", "
                                   << predicted_states_(kPosY, 0) << ", "
                                   << predicted_states_(kPosZ, 0) << ", "
                                   << predicted_states_(kVelX, 0) << ", "
                                   << predicted_states_(kVelY, 0) << ", "
                                   << predicted_states_(kVelZ, 0) << ", "
                                   << predicted_states_(kYaw, 0)
                                   << ") pred_u0=("
                                   << predicted_inputs_(INPUT::kVelCmdX, 0) << ", "
                                   << predicted_inputs_(INPUT::kVelCmdY, 0) << ", "
                                   << predicted_inputs_(INPUT::kVelCmdZ, 0) << ", "
                                   << predicted_inputs_(INPUT::kYawRate, 0) << ", "
                                   << predicted_inputs_(INPUT::kSlack0, 0) << ", "
                                   << predicted_inputs_(INPUT::kSlack1, 0) << ", "
                                   << predicted_inputs_(INPUT::kSlack2, 0) << ")"
                                   << " slack_sum=" << latest_slack_);

  const clock_t solve_end = clock();
  const double solve_time_sec = double(solve_end - solve_start) / CLOCKS_PER_SEC;
  last_feedback_solve_time_sec_ = solve_time_sec;
  last_solve_ok_ = true;

  // Publish the predicted trajectory.
  publishPrediction(predicted_states_, predicted_inputs_, call_time, W_q_non,
                    W_p_non);

  // Timing
  timing_feedback_ = 0.9 * timing_feedback_ +
                     0.1 * solve_time_sec;
  if (params_.print_info_)
    ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
                      timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);

  // Return the input control command.
  const acado_mpc_common::ControlCommand command =
      updateControlCommand(predicted_states_.col(0),
                           predicted_inputs_.col(0),
                           call_time,
                           field_active);
  if (!command.velocity_cmd.allFinite() ||
      !std::isfinite(command.yaw_rate) || !std::isfinite(command.slack)) {
    ++nonfinite_recovery_count_;
    solve_from_scratch_ = true;
    last_solve_ok_ = false;
    ROS_ERROR("[MPC][RECOVERY] invalid control command; keeping previous valid control");
    preparationThread();
    return makeFailureFallback("invalid control command");
  }
  last_valid_control_ = command;
  has_last_valid_control_ = true;
  return command;
#undef GREEN
#undef RST
}

template<typename T>
bool MpcController<T>::setStateEstimate(
    const acado_mpc_common::QuadRelativeEstimate& state_estimate) {
  est_state_(kPosX) = state_estimate.position.x();
  est_state_(kPosY) = state_estimate.position.y();
  est_state_(kPosZ) = state_estimate.position.z();
  est_state_(kVelX) = state_estimate.velocity.x();
  est_state_(kVelY) = state_estimate.velocity.y();
  est_state_(kVelZ) = state_estimate.velocity.z();
  est_state_(kYaw) =
      quaternionToYaw<T>(state_estimate.orientation.template cast<T>());
  return true;
}

// TODO: this function also computes the reference inputs?
template<typename T>
bool MpcController<T>::setReference(
    const acado_mpc_common::RelativeTrajectory& reference_trajectory) {
  reference_states_.setZero();
  reference_inputs_.setZero();
  Eigen::Matrix<T, 1, kSamples + 1> yaw_reference =
      Eigen::Matrix<T, 1, kSamples + 1>::Zero();
  const T prediction_dt = static_cast<T>(mpc_wrapper_.getTimestep());

  if (reference_trajectory.points.size() == 1) {
    const T yaw_ref =
        unwrapAngleNear<T>(quaternionToYaw<T>(
                               reference_trajectory.points.front().orientation
                                   .template cast<T>()),
                           est_state_(kYaw));
    reference_states_ = (Eigen::Matrix<T, kStateSize, 1>()
        << reference_trajectory.points.front().position.template cast<T>(),
        reference_trajectory.points.front().velocity.template cast<T>(),
        yaw_ref
    ).finished().replicate(1, kSamples + 1);
    yaw_reference.setConstant(yaw_ref);
  } else {
    T previous_yaw = est_state_(kYaw);
    for (int i = 0; i < kSamples + 1; i++) {
      const T yaw_ref =
          unwrapAngleNear<T>(quaternionToYaw<T>(
                                 reference_trajectory.points.at(i).orientation
                                     .template cast<T>()),
                             previous_yaw);
      reference_states_.col(i) << reference_trajectory.points.at(i).position.template cast<T>(),
          reference_trajectory.points.at(i).velocity.template cast<T>(),
          yaw_ref;
      yaw_reference(i) = yaw_ref;
      previous_yaw = yaw_ref;
    }
  }

  // Keep state tracking and input feed-forward aligned with the reference
  // trajectory; only the non-inertial compensation terms may be frozen via the
  // dedicated debug switch.
  for (int i = 0; i < kSamples + 1; ++i) {
    reference_inputs_(INPUT::kVelCmdX, i) = reference_states_(kVelX, i);
    reference_inputs_(INPUT::kVelCmdY, i) = reference_states_(kVelY, i);
    reference_inputs_(INPUT::kVelCmdZ, i) = reference_states_(kVelZ, i);
    if (i < kSamples && prediction_dt > static_cast<T>(1e-9)) {
      reference_inputs_(INPUT::kYawRate, i) =
          (yaw_reference(i + 1) - yaw_reference(i)) / prediction_dt;
    } else if (i > 0) {
      reference_inputs_(INPUT::kYawRate, i) =
          reference_inputs_(INPUT::kYawRate, i - 1);
    } else {
      reference_inputs_(INPUT::kYawRate, i) = static_cast<T>(0.0);
    }
  }
  return true;
}

template<typename T>
acado_mpc_common::ControlCommand MpcController<T>::updateControlCommand(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
    ros::Time& time,
    bool /* field_active */) {
  Eigen::Matrix<T, kInputSize, 1> input_bounded = input.template cast<T>();

  // Bound inputs for sanity.
  input_bounded(INPUT::kVelCmdX) = std::max(-params_.max_v_xy_,
                                           std::min(params_.max_v_xy_, input_bounded(INPUT::kVelCmdX)));
  input_bounded(INPUT::kVelCmdY) = std::max(-params_.max_v_xy_,
                                           std::min(params_.max_v_xy_, input_bounded(INPUT::kVelCmdY)));
  input_bounded(INPUT::kVelCmdZ) = std::max(-params_.max_v_z_,
                                           std::min(params_.max_v_z_, input_bounded(INPUT::kVelCmdZ)));
  input_bounded(INPUT::kYawRate) = std::max(-params_.max_yaw_rate_,
                                           std::min(params_.max_yaw_rate_, input_bounded(INPUT::kYawRate)));
  for (int slack_idx = INPUT::kSlack0; slack_idx <= INPUT::kSlack2; ++slack_idx) {
    if (std::isfinite(static_cast<double>(params_.cbf_slack_max_))) {
      input_bounded(slack_idx) =
          std::max<T>(0.0, std::min(params_.cbf_slack_max_, input_bounded(slack_idx)));
    } else {
      input_bounded(slack_idx) = std::max<T>(0.0, input_bounded(slack_idx));
    }
  }

  acado_mpc_common::ControlCommand command;

  command.velocity_cmd.x() = input_bounded(INPUT::kVelCmdX);
  command.velocity_cmd.y() = input_bounded(INPUT::kVelCmdY);
  command.velocity_cmd.z() = input_bounded(INPUT::kVelCmdZ);
  command.yaw_rate = input_bounded(INPUT::kYawRate);
  command.slack = aggregateObstacleSlack<T>(input_bounded);
  return command;
}

template<typename T>
bool MpcController<T>::publishPrediction(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
    ros::Time& time, 
    const Eigen::Quaterniond& W_q_non,
    const Eigen::Vector3d& W_p_non) {
  nav_msgs::Path path_msg;
  nav_msgs::Path path_world_msg;
  path_msg.header.stamp = time;
  path_world_msg.header.stamp = time;
  path_msg.header.frame_id = acado_mpc_common::relativeFrameId();
  path_world_msg.header.frame_id = acado_mpc_common::worldFrameId();
  geometry_msgs::PoseStamped pose;
  geometry_msgs::PoseStamped pose_world;
  visualization_msgs::Marker marker_world;
  T dt = mpc_wrapper_.getTimestep();

  marker_world.header.stamp = time;
  marker_world.header.frame_id = acado_mpc_common::worldFrameId();
  marker_world.ns = "predicted_traj";
  marker_world.id = 0;
  marker_world.type = visualization_msgs::Marker::LINE_STRIP;
  marker_world.action = visualization_msgs::Marker::ADD;
  marker_world.pose.orientation.w = 1.0;
  marker_world.scale.x = 0.03;
  marker_world.color.a = 1.0;  // per-vertex colors below; keep alpha default

  for (int i = 0; i < kSamples; i++) {
    const Eigen::AngleAxis<T> yaw_rot(states(kYaw, i), Eigen::Matrix<T, 3, 1>::UnitZ());
    const Eigen::Quaternion<T> q_rel(yaw_rot);
    pose.header.stamp = time + ros::Duration(i * dt);
    pose.header.seq = i;
    pose.pose.position.x = states(kPosX, i);
    pose.pose.position.y = states(kPosY, i);
    pose.pose.position.z = states(kPosZ, i);
    pose.pose.orientation.w = q_rel.w();
    pose.pose.orientation.x = q_rel.x();
    pose.pose.orientation.y = q_rel.y();
    pose.pose.orientation.z = q_rel.z();
    path_msg.poses.push_back(pose);
    Eigen::Vector3d temp_position(states(kPosX, i), 
        states(kPosY, i), states(kPosZ, i));
    temp_position = W_p_non + W_q_non * temp_position;
    const Eigen::Quaterniond q_world = W_q_non * q_rel.template cast<double>();
    pose_world.header.stamp = time + ros::Duration(i * dt);
    pose_world.header.seq = i;
    pose_world.pose.position.x = temp_position.x();
    pose_world.pose.position.y = temp_position.y();
    pose_world.pose.position.z = temp_position.z();
    pose_world.pose.orientation.w = q_world.w();
    pose_world.pose.orientation.x = q_world.x();
    pose_world.pose.orientation.y = q_world.y();
    pose_world.pose.orientation.z = q_world.z();
    path_world_msg.poses.push_back(pose_world);

    geometry_msgs::Point p;
    p.x = temp_position.x();
    p.y = temp_position.y();
    p.z = temp_position.z();
    marker_world.points.push_back(p);
    std_msgs::ColorRGBA c;
    const double t = (kSamples <= 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(kSamples - 1);
    c.r = 0.0;
    c.g = 1.0 - t;  // start green
    c.b = t;        // end blue
    c.a = 1.0;
    marker_world.colors.push_back(c);
  }

  pub_predicted_trajectory_.publish(path_msg);
  pub_predicted_trajectory_world_.publish(path_world_msg);
  if (!marker_world.points.empty()) {
    pub_predicted_trajectory_marker_world_.publish(marker_world);
  }

  return true;
}

template<typename T>
void MpcController<T>::preparationThread() {
  const clock_t start = clock();

  if (!mpc_wrapper_.prepare()) {
    ROS_ERROR("[MpcController] QP preparation failed; next cycle will use the last valid command");
  }

  // Timing
  const clock_t end = clock();
  last_preparation_time_sec_ = double(end - start) / CLOCKS_PER_SEC;
  timing_preparation_ = 0.9 * timing_preparation_ +
                        0.1 * last_preparation_time_sec_;
}

template<typename T>
bool MpcController<T>::setNewParams(MpcParams<T>& params) {
  if (!mpc_wrapper_.setTrackingTrustTimeDampingLambda(
          params.tracking_trust_time_damping_lambda_)) {
    ROS_ERROR("MPC: Failed to configure tracking trust time damping.");
    return false;
  }
  if (!mpc_wrapper_.setCosts(params.Q_, params.R_, params.P_,
                             params.state_cost_exponential_,
                             params.input_cost_exponential_)) {
    ROS_ERROR("MPC: Failed to configure cost matrices.");
    return false;
  }
  if (!mpc_wrapper_.setLimits(
          params.max_v_xy_, params.max_v_z_, params.max_yaw_rate_,
          params.a_max_xy_, params.a_max_z_, params.cbf_slack_max_)) {
    ROS_ERROR("MPC: Failed to configure input/path limits.");
    return false;
  }
  if (!mpc_wrapper_.setVelocityTrackingTimeConstants(params.tau_v_xy_,
                                                       params.tau_v_z_)) {
    ROS_ERROR("MPC: Failed to configure velocity tracking time constants.");
    return false;
  }
  const bool field_active = params.cbf_use_field_hocbf_;
  const bool obstacle_update_ok = mpc_wrapper_.setObstacles(
      params.cbf_obstacles_, params.cbf_obstacle_profiles_,
      params.cbf_alpha1_, params.cbf_alpha2_,
      params.cbf_enabled_ && !field_active);
  if (!obstacle_update_ok) {
    ROS_ERROR("MPC: Failed to configure CBF obstacles.");
    return false;
  }
  if (!mpc_wrapper_.setFieldHocbf(params.cbf_field_hocbf_profile_, field_active)) {
    ROS_ERROR("MPC: Failed to configure field HOCBF.");
    return false;
  }
  // mpc_wrapper_.setCameraParameters(params.p_B_C_, params.q_B_C_);
  params.changed_ = false;
  return true;
}


template
class MpcController<float>;

template
class MpcController<double>;

} // namespace acado_mpc
