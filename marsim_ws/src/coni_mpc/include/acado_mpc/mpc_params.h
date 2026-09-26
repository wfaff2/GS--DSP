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

#include "acado_mpc/mpc_wrapper.h"
#include "acado_mpc/mpc_common.h"

#include <xmlrpcpp/XmlRpcValue.h>

#include <ros/ros.h>
#include <algorithm>
#include <limits>
#include <cmath>


namespace acado_mpc
{

template <typename T>
class MpcParams {
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using ObstacleVector = typename MpcWrapper<T>::ObstacleVector;
  using ObstacleProfileVector = typename MpcWrapper<T>::ObstacleProfileVector;
  using RiskRegionVector = typename MpcWrapper<T>::RiskRegionVector;
  using RiskRegionProfileVector = typename MpcWrapper<T>::RiskRegionProfileVector;
  using FieldHocbfProfile = typename MpcWrapper<T>::FieldHocbfProfile;
  
  MpcParams() :
    changed_(false),
    print_info_(false),
    state_cost_exponential_(0.0),
    input_cost_exponential_(0.0),
    tracking_trust_time_damping_lambda_(0.0),
    max_v_xy_(0.0),
    max_v_z_(0.0),
    max_yaw_rate_(0.0),
    cbf_enabled_(false),
    cbf_alpha1_(1.0),
    cbf_alpha2_(1.0),
    cbf_safety_margin_(0.0),
    cbf_slack_max_(std::numeric_limits<T>::infinity()),
    // p_B_C_(Eigen::Matrix<T, 3, 1>::Zero()),
    // q_B_C_(Eigen::Quaternion<T>(1.0, 0.0, 0.0, 0.0)),
    Q_(Eigen::Matrix<T, kCostSize, kCostSize>::Zero()),
    P_(Eigen::Matrix<T, kEndRefSize, kEndRefSize>::Zero()),
    R_(Eigen::Matrix<T, kInputSize, kInputSize>::Zero()),
    cbf_obstacles_(),
    cbf_obstacle_profiles_(),
    cbf_risk_regions_(),
    cbf_risk_region_profiles_(),
    cbf_use_risk_regions_(false),
    cbf_use_field_hocbf_(false),
    cbf_field_hocbf_profile_(FieldHocbfProfile::Zero())
  {
  }

  ~MpcParams()
  {
  }

  bool loadParameters(ros::NodeHandle& pnh)
  {
    #define GET_PARAM(name) \
    if (!acado_mpc_common::getParam(#name, name, pnh)) \
      return false

    #define GET_PARAM_(name) \
    if (!acado_mpc_common::getParam(#name, name ## _, pnh)) \
      return false

    // Read state costs.
    // T Q_pos_xy, Q_pos_z, Q_attitude, Q_velocity, Q_perception;
    T Q_p_xy, Q_p_z, Q_v, Q_yaw;
    GET_PARAM(Q_p_xy);
    GET_PARAM(Q_p_z);
    GET_PARAM(Q_v);
    acado_mpc_common::getParam("Q_yaw", Q_yaw, (T)20.0, pnh);
    // quadrotor_common::getParam("Q_perception", Q_perception, (T)0.0, pnh);

    // Check whether all state costs are positive.
    if(Q_p_xy       <= 0.0 ||
       Q_p_z        <= 0.0 ||
       Q_v          <= 0.0 ||
       Q_yaw        <  0.0)

      //  Q_perception < 0.0)      // Perception cost can be zero to deactivate.
    {
      ROS_ERROR("MPC: State cost Q has negative enries!");
      return false;
    }

    // Read input costs.
    T R_v_xy, R_v_z, R_yaw_rate;
    GET_PARAM(R_v_xy);
    GET_PARAM(R_v_z);
    acado_mpc_common::getParam("R_yaw_rate", R_yaw_rate, (T)10.0, pnh);
    T R_slack = (T)1.0;
    if (pnh.hasParam("cbf/R_slack")) {
      acado_mpc_common::getParam("cbf/R_slack", R_slack, (T)1.0, pnh);
    } else {
      acado_mpc_common::getParam("R_slack", R_slack, (T)1.0, pnh);
    }

    // Check whether all input costs are positive.
    if(R_v_xy     <= 0.0 ||
       R_v_z      <= 0.0 ||
       R_yaw_rate <= 0.0 ||
       R_slack    <= 0.0)
    {
      ROS_ERROR("MPC: Input cost R has negative enries!");
      return false;
    }

    // Set state and input cost matrices.
    Q_ = (Eigen::Matrix<T, kCostSize, 1>() <<
      Q_p_xy, Q_p_xy, Q_p_z,
      Q_v, Q_v, Q_v,
      Q_yaw).finished().asDiagonal();
    R_ = (Eigen::Matrix<T, kInputSize, 1>() <<
      R_v_xy, R_v_xy, R_v_z, R_yaw_rate,
      R_slack, R_slack, R_slack).finished().asDiagonal();

    // Read cost scaling values
    acado_mpc_common::getParam("state_cost_exponential",
      state_cost_exponential_, (T)0.0, pnh);
    acado_mpc_common::getParam("input_cost_exponential",
      input_cost_exponential_, (T)0.0, pnh);
    acado_mpc_common::getParam("tracking_trust_time_damping_lambda",
      tracking_trust_time_damping_lambda_, (T)0.0, pnh);
    if (!(tracking_trust_time_damping_lambda_ >= static_cast<T>(0.0)) ||
        !std::isfinite(static_cast<double>(tracking_trust_time_damping_lambda_))) {
      ROS_ERROR("MPC: tracking_trust_time_damping_lambda must satisfy lambda >= 0.");
      return false;
    }

    // Read command limits.
    GET_PARAM_(max_v_xy);
    GET_PARAM_(max_v_z);
    acado_mpc_common::getParam("max_yaw_rate", max_yaw_rate_, (T)1.5, pnh);

    // Check whether all input limits are positive.
    if(max_v_xy_ <= 0.0 ||
       max_v_z_  <= 0.0 ||
       max_yaw_rate_ <= 0.0)
    {
      ROS_ERROR("MPC: All limits must be positive non-zero values!");
      return false;
    }


    acado_mpc_common::getParam("print_info", print_info_, false, pnh);
    if(print_info_) ROS_INFO("MPC: Informative printing enabled.");

    P_ = Q_;
    if (print_info_) {
      ROS_INFO_STREAM("MPC: Using terminal cost P = Q, diag(P) = "
                      << P_.diagonal().transpose());
    }

    acado_mpc_common::getParam("cbf/enabled", cbf_enabled_, false, pnh);
    acado_mpc_common::getParam("cbf/use_field_hocbf", cbf_use_field_hocbf_,
                               false, pnh);
    const bool has_alpha1 = pnh.hasParam("cbf/alpha1");
    const bool has_alpha2 = pnh.hasParam("cbf/alpha2");
    if (has_alpha1) {
      acado_mpc_common::getParam("cbf/alpha1", cbf_alpha1_, (T)1.0, pnh);
    }
    if (has_alpha2) {
      acado_mpc_common::getParam("cbf/alpha2", cbf_alpha2_, (T)1.0, pnh);
    }
    if (!has_alpha1 && !has_alpha2) {
      const bool has_alpha_hdot = pnh.hasParam("cbf/alpha_hdot");
      const bool has_alpha_h = pnh.hasParam("cbf/alpha_h");
      const bool has_alpha = pnh.hasParam("cbf/alpha");
      if (has_alpha_hdot) {
        acado_mpc_common::getParam("cbf/alpha_hdot", cbf_alpha1_, (T)1.0, pnh);
      }
      if (has_alpha_h) {
        acado_mpc_common::getParam("cbf/alpha_h", cbf_alpha2_, (T)1.0, pnh);
      }
      if (!has_alpha_hdot && has_alpha_h) {
        cbf_alpha1_ = cbf_alpha2_;
      } else if (has_alpha_hdot && !has_alpha_h) {
        cbf_alpha2_ = cbf_alpha1_;
      } else if (has_alpha) {
        T legacy_alpha = static_cast<T>(1.0);
        acado_mpc_common::getParam("cbf/alpha", legacy_alpha, (T)1.0, pnh);
        cbf_alpha1_ = legacy_alpha;
        cbf_alpha2_ = legacy_alpha;
      }
    } else {
      if (!has_alpha1) {
        cbf_alpha1_ = cbf_alpha2_;
      }
      if (!has_alpha2) {
        cbf_alpha2_ = cbf_alpha1_;
      }
    }
    acado_mpc_common::getParam("cbf/safety_margin", cbf_safety_margin_, (T)0.0, pnh);
    acado_mpc_common::getParam("cbf/slack_max", cbf_slack_max_, std::numeric_limits<T>::infinity(), pnh);
    // Keep cbf_slack_max == 0 as hard-CBF (slack fixed to zero).
    // Only negative or non-finite values are treated as unbounded.
    if (cbf_slack_max_ < 0.0 || !std::isfinite(static_cast<double>(cbf_slack_max_))) {
      cbf_slack_max_ = std::numeric_limits<T>::infinity();
    }
    cbf_obstacles_.clear();
    cbf_obstacle_profiles_.clear();
    if (cbf_enabled_) {
      XmlRpc::XmlRpcValue obstacle_list;
      if (pnh.getParam("cbf/obstacles", obstacle_list)) {
        if (obstacle_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
          ROS_ERROR("MPC: Parameter cbf/obstacles must be a list.");
          return false;
        }
        auto valueToDouble = [](const XmlRpc::XmlRpcValue& value, double& output) -> bool {
          if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            output = static_cast<double>(value);
            return true;
          }
          if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
            output = static_cast<int>(value);
            return true;
          }
          return false;
        };
        for (int i = 0; i < obstacle_list.size(); ++i) {
          if (obstacle_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR("MPC: Each CBF obstacle must be a dictionary.");
            return false;
          }
          XmlRpc::XmlRpcValue& entry = obstacle_list[i];
          if (!entry.hasMember("position") || !entry.hasMember("radius")) {
            ROS_ERROR("MPC: CBF obstacle requires 'position' and 'radius'.");
            return false;
          }
          XmlRpc::XmlRpcValue& position = entry["position"];
          if (position.getType() != XmlRpc::XmlRpcValue::TypeArray || position.size() != 3) {
            ROS_ERROR("MPC: CBF obstacle 'position' must be a list of three numbers.");
            return false;
          }
          Eigen::Matrix<T, 7, 1> obstacle =
              Eigen::Matrix<T, 7, 1>::Zero();
          for (int j = 0; j < 3; ++j) {
            double component = 0.0;
            if (!valueToDouble(position[j], component)) {
              ROS_ERROR("MPC: CBF obstacle position entries must be numeric.");
              return false;
            }
            obstacle(j) = static_cast<T>(component);
          }
          double radius = 0.0;
          if (!valueToDouble(entry["radius"], radius)) {
            ROS_ERROR("MPC: CBF obstacle radius must be numeric.");
            return false;
          }
          if (radius <= 0.0) {
            ROS_ERROR("MPC: CBF obstacle radius must be positive.");
            return false;
          }
          obstacle(3) = static_cast<T>(radius + cbf_safety_margin_);
          if (entry.hasMember("velocity")) {
            XmlRpc::XmlRpcValue& velocity = entry["velocity"];
            if (velocity.getType() != XmlRpc::XmlRpcValue::TypeArray ||
                velocity.size() != 3) {
              ROS_ERROR("MPC: CBF obstacle 'velocity' must be a list of three numbers.");
              return false;
            }
            for (int j = 0; j < 3; ++j) {
              double component = 0.0;
              if (!valueToDouble(velocity[j], component)) {
                ROS_ERROR("MPC: CBF obstacle velocity entries must be numeric.");
                return false;
              }
              obstacle(4 + j) = static_cast<T>(component);
            }
          }
          cbf_obstacles_.push_back(obstacle);
        }
      } else {
        ROS_WARN_STREAM("[" << pnh.getNamespace()
            << "] CBF enabled but no obstacles configured.");
      }
    }

    changed_ = true;

    #undef GET_PARAM
    #undef GET_PARAM_OPT
    #undef GET_PARAM_
    #undef GET_PARAM_OPT_

    return true;
  }

 public:

  bool changed_;

  bool print_info_;

  T state_cost_exponential_;
  T input_cost_exponential_;
  T tracking_trust_time_damping_lambda_;

  T max_v_xy_;
  T max_v_z_;
  T max_yaw_rate_;

  bool cbf_enabled_;
  T cbf_alpha1_;
  T cbf_alpha2_;
  T cbf_safety_margin_;
  T cbf_slack_max_;

  // Eigen::Matrix<T, 3, 1> p_B_C_;
  // Eigen::Quaternion<T> q_B_C_;

  Eigen::Matrix<T, kCostSize, kCostSize> Q_;
  Eigen::Matrix<T, kEndRefSize, kEndRefSize> P_;
  Eigen::Matrix<T, kInputSize, kInputSize> R_;
  ObstacleVector cbf_obstacles_;
  ObstacleProfileVector cbf_obstacle_profiles_;
  RiskRegionVector cbf_risk_regions_;
  RiskRegionProfileVector cbf_risk_region_profiles_;
  bool cbf_use_risk_regions_;
  bool cbf_use_field_hocbf_;
  FieldHocbfProfile cbf_field_hocbf_profile_;
};



} // namespace acado_mpc
