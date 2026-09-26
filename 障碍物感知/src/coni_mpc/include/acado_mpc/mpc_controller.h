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

#include "acado_mpc/mpc_common.h"
#include "acado_mpc/mpc_wrapper.h"
#include "acado_mpc/mpc_params.h"

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>


namespace acado_mpc {

enum STATE {
  kPosX = 0,
  kPosY,
  kPosZ,
  kVelX,
  kVelY,
  kVelZ,
  kYaw
};

enum INPUT {
  kVelCmdX = 0,
  kVelCmdY = 1,
  kVelCmdZ = 2,
  kYawRate = 3,
  kSlack0 = 4,
  kSlack1 = 5,
  kSlack2 = 6
};

/**
 * @brief MpcController class
 * 
 * @tparam T scalar type
 */
template<typename T>
class MpcController {
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using NonInertialProfile = Eigen::Matrix<T, 3, kSamples + 1>;

  static_assert(kStateSize == 7,
                "MpcController: Wrong model size. Number of states does not match.");
  static_assert(kInputSize == 7,
                "MpcController: Wrong model size. Number of inputs does not match.");

  MpcController(const ros::NodeHandle& nh,
                const ros::NodeHandle& pnh,
                int id,
                const std::string& topic = "acado_mpc/trajectory_predicted", 
                const std::string& topic_world = "acado_mpc/trajectory_predicted_world");

  MpcController() : MpcController(ros::NodeHandle(), ros::NodeHandle("~"),0) {}
  ~MpcController();

  acado_mpc_common::ControlCommand off();

  static void buildLegacyCurrentTimeNonInertialData(
      const acado_mpc_common::QuadRelativeEstimate& state_estimate,
      Eigen::Matrix<T, 3, 1>& omega_non,
      Eigen::Matrix<T, 3, 1>& beta_non,
      Eigen::Matrix<T, 3, 1>& a_car_non);

  acado_mpc_common::ControlCommand run(
      const acado_mpc_common::QuadRelativeEstimate& state_estimate,
      const acado_mpc_common::RelativeTrajectory& reference_trajectory,
      const MpcParams<T>& params, 
      const Eigen::Quaterniond& W_q_non = Eigen::Quaterniond::Identity(),
      const Eigen::Vector3d& W_p_non = Eigen::Vector3d::Zero(),
      const NonInertialProfile* omega_non_profile = nullptr,
      const NonInertialProfile* beta_non_profile = nullptr,
      const NonInertialProfile* a_car_non_profile = nullptr);

  T getLatestSlack() const { return latest_slack_; }
  double getLastSolveTimeMs() const { return last_feedback_solve_time_sec_ * 1000.0; }
  double getLastFeedbackTimeMs() const { return last_feedback_solve_time_sec_ * 1000.0; }
  double getLastPreparationTimeMs() const { return last_preparation_time_sec_ * 1000.0; }
  bool getLastSolveOk() const { return last_solve_ok_; }
  double getPredictionDt() const { return static_cast<double>(mpc_wrapper_.getTimestep()); }
  const Eigen::Matrix<T, kStateSize, kSamples + 1>& getPredictedStates() const {
    return predicted_states_;
  }
  void setWarmStart(bool enable);
  void requestSolveFromScratch(const std::string& reason = std::string());

private:
  bool setStateEstimate(
      const acado_mpc_common::QuadRelativeEstimate& state_estimate);

  bool setReference(const acado_mpc_common::RelativeTrajectory& reference_trajectory);

  acado_mpc_common::ControlCommand updateControlCommand(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
      ros::Time& time);

  bool publishPrediction(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
      ros::Time& time, 
      const Eigen::Quaterniond& W_q_non,
      const Eigen::Vector3d& W_p_non);

  void resetPredictionsToSafeState();

  void preparationThread();

  bool setNewParams(MpcParams<T>& params);

  // Handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // // Subscribers and publisher.
  ros::Publisher pub_predicted_trajectory_;
  ros::Publisher pub_predicted_trajectory_world_;
  ros::Publisher pub_predicted_trajectory_marker_world_;

  // Parameters
  MpcParams<T> params_;

  // MPC
  MpcWrapper<T> mpc_wrapper_;

  // Variables
  T timing_feedback_, timing_preparation_;
  bool solve_from_scratch_;
  bool warm_start_;
  Eigen::Matrix<T, kStateSize, 1> est_state_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> reference_states_;
  Eigen::Matrix<T, kInputSize, kSamples + 1> reference_inputs_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> predicted_states_;
  Eigen::Matrix<T, kInputSize, kSamples> predicted_inputs_;
  T latest_slack_;
  double last_feedback_solve_time_sec_;
  double last_preparation_time_sec_;
  bool last_solve_ok_;
  std::size_t nonfinite_recovery_count_;
  std::size_t obstacle_update_fail_count_;
};


} // namespace acado_mpc
