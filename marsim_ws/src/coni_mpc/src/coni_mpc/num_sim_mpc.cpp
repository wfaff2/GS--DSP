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

#include "coni_mpc/num_sim_mpc.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <std_msgs/ColorRGBA.h>

namespace coni_mpc
{

std::mutex NumSimMpc::shared_state_mutex_;
std::vector<Eigen::Vector3d> NumSimMpc::shared_positions_;
std::vector<Eigen::Vector3d> NumSimMpc::shared_velocities_;
std::vector<bool> NumSimMpc::shared_position_valid_;
NumSimMpc::ObstacleVector NumSimMpc::shared_static_obstacles_;

namespace {
#ifndef CONI_MPC_EXTERNAL_FIELD_MICRO_ITERATIONS
// Keep source compatibility for standalone users of this translation unit:
// an OsqpEigen build enables the outer refinement unless its build system
// explicitly disables it.
#define CONI_MPC_EXTERNAL_FIELD_MICRO_ITERATIONS 1
#endif

std::string toLowerCopy(const std::string& text) {
  std::string out = text;
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

NumSimMpc::FrameMode resolveFrameMode(const ros::NodeHandle& pnh,
                                      std::string& frame_mode_effective) {
  bool use_inertial_frame = false;
  bool use_noninertial_frame = true;
  std::string frame_mode;
  pnh.param("use_inertial_frame", use_inertial_frame, false);
  pnh.param("use_noninertial_frame", use_noninertial_frame, true);
  pnh.param("frame_mode", frame_mode, std::string(""));
  const std::string mode = toLowerCopy(frame_mode);
  if (mode == "inertial") {
    frame_mode_effective = "inertial";
    return NumSimMpc::FrameMode::kInertial;
  }
  if (mode == "noninertial" || mode == "non-inertial" || mode == "non_inertial") {
    frame_mode_effective = "noninertial";
    return NumSimMpc::FrameMode::kNonInertial;
  }
  if (use_inertial_frame && !use_noninertial_frame) {
    frame_mode_effective = "inertial";
    return NumSimMpc::FrameMode::kInertial;
  }
  frame_mode_effective = "noninertial";
  return NumSimMpc::FrameMode::kNonInertial;
}

NumSimMpc::CarState_t carStateFromOdom(const nav_msgs::Odometry& car_odom) {
  NumSimMpc::CarState_t car_state = NumSimMpc::CarState_t::Zero();
  car_state(0) = car_odom.pose.pose.position.x;
  car_state(1) = car_odom.pose.pose.position.y;
  car_state(2) = car_odom.pose.pose.position.z;
  car_state(3) = car_odom.twist.twist.linear.x;
  car_state(4) = car_odom.twist.twist.linear.y;
  car_state(5) = car_odom.twist.twist.linear.z;
  car_state(6) = car_odom.pose.pose.orientation.w;
  car_state(7) = car_odom.pose.pose.orientation.x;
  car_state(8) = car_odom.pose.pose.orientation.y;
  car_state(9) = car_odom.pose.pose.orientation.z;
  car_state(13) = car_odom.twist.twist.angular.x;
  car_state(14) = car_odom.twist.twist.angular.y;
  car_state(15) = car_odom.twist.twist.angular.z;
  return car_state;
}

NumSimMpc::Obstacle obstacleInNonInertialFrame(
    const NumSimMpc::Obstacle& world_obstacle,
    const NumSimMpc::CarState_t& car_state,
    double planar_z) {
  Eigen::Quaterniond W_q_non(car_state(6), car_state(7), car_state(8), car_state(9));
  W_q_non.normalize();
  const Eigen::Vector3d car_position(car_state(0), car_state(1), car_state(2));
  const Eigen::Vector3d car_velocity(car_state(3), car_state(4), car_state(5));
  const Eigen::Vector3d omega_non(car_state(13), car_state(14), car_state(15));
  const Eigen::Vector3d world_position(world_obstacle(0), world_obstacle(1),
                                       world_obstacle(2));
  const Eigen::Vector3d world_velocity(world_obstacle(4), world_obstacle(5),
                                       world_obstacle(6));
  Eigen::Vector3d relative_position =
      W_q_non.inverse() * (world_position - car_position);
  const Eigen::Vector3d relative_velocity =
      -omega_non.cross(relative_position) +
      W_q_non.inverse() * (world_velocity - car_velocity);
  relative_position.z() = planar_z;

  NumSimMpc::Obstacle relative_obstacle = world_obstacle;
  relative_obstacle(0) = relative_position.x();
  relative_obstacle(1) = relative_position.y();
  relative_obstacle(2) = relative_position.z();
  relative_obstacle(4) = relative_velocity.x();
  relative_obstacle(5) = relative_velocity.y();
  relative_obstacle(6) = relative_velocity.z();
  return relative_obstacle;
}

std::string formatScalar(double value) {
  if (!std::isfinite(value)) {
    return "NA";
  }
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6) << value;
  return oss.str();
}

std::string formatVec3(const Eigen::Vector3d& value) {
  std::ostringstream oss;
  oss << "(" << formatScalar(value.x()) << ", " << formatScalar(value.y())
      << ", " << formatScalar(value.z()) << ")";
  return oss.str();
}

std_msgs::ColorRGBA makeColor(double r, double g, double b, double a) {
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std_msgs::ColorRGBA quadBodyColor(int quad_id, double alpha) {
  if (quad_id == 0) {
    return makeColor(0.95, 0.15, 0.15, alpha);
  }
  if (quad_id == 1) {
    return makeColor(0.12, 0.47, 0.90, alpha);
  }
  if (quad_id == 2) {
    return makeColor(1.00, 0.72, 0.10, alpha);
  }
  if (quad_id == 3) {
    return makeColor(0.12, 0.70, 0.28, alpha);
  }
  return makeColor(0.40, 0.40, 0.40, alpha);
}

std_msgs::ColorRGBA quadPathColor(int quad_id, double alpha) {
  // Keep each trajectory visually bound to its corresponding UAV body.
  return quadBodyColor(quad_id, alpha);
}

geometry_msgs::Pose composePose(const geometry_msgs::Pose& base_pose,
                                const Eigen::Vector3d& local_translation,
                                const Eigen::Quaterniond& local_rotation) {
  Eigen::Quaterniond base_q(base_pose.orientation.w, base_pose.orientation.x,
                            base_pose.orientation.y, base_pose.orientation.z);
  base_q.normalize();
  const Eigen::Vector3d base_p(base_pose.position.x, base_pose.position.y,
                               base_pose.position.z);
  const Eigen::Vector3d world_p = base_p + base_q * local_translation;
  const Eigen::Quaterniond world_q = base_q * local_rotation;

  geometry_msgs::Pose pose = base_pose;
  pose.position.x = world_p.x();
  pose.position.y = world_p.y();
  pose.position.z = world_p.z();
  pose.orientation.w = world_q.w();
  pose.orientation.x = world_q.x();
  pose.orientation.y = world_q.y();
  pose.orientation.z = world_q.z();
  return pose;
}

visualization_msgs::Marker makeMeshMarker(const std_msgs::Header& header,
                                          const geometry_msgs::Pose& base_pose,
                                          const std::string& mesh_resource,
                                          const Eigen::Vector3d& local_translation,
                                          const Eigen::Quaterniond& local_rotation,
                                          double uniform_scale,
                                          const std_msgs::ColorRGBA& color,
                                          int marker_id) {
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.header.frame_id = acado_mpc_common::worldFrameId();
  marker.ns = "car_body";
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = composePose(base_pose, local_translation, local_rotation);
  marker.mesh_resource = mesh_resource;
  marker.mesh_use_embedded_materials = false;
  marker.scale.x = uniform_scale;
  marker.scale.y = uniform_scale;
  marker.scale.z = uniform_scale;
  marker.color = color;
  marker.lifetime = ros::Duration(0.0);
  return marker;
}

std::string formatObstacle(const NumSimMpc::Obstacle& obstacle) {
  std::ostringstream oss;
  oss << "(" << formatScalar(obstacle(0)) << ", " << formatScalar(obstacle(1))
      << ", " << formatScalar(obstacle(2)) << ", r="
      << formatScalar(obstacle(3)) << ", v=("
      << formatScalar(obstacle(4)) << ", " << formatScalar(obstacle(5))
      << ", " << formatScalar(obstacle(6)) << "))";
  return oss.str();
}

}  // namespace

NumSimMpc::NumSimMpc(const ros::NodeHandle &nh, const ros::NodeHandle &pnh,int id) : 
    MpcBase(nh, pnh,id), 
    quad_id(id),
    nh_(nh), pnh_(pnh), 
    imu_(0.0, 0.0, 9.81),
    car_beta_non_(Eigen::Vector3d::Zero()),
    car_omega_non_stage_profile_(NonInertialProfile::Zero()),
    car_beta_non_stage_profile_(NonInertialProfile::Zero()),
    car_a_car_non_stage_profile_(NonInertialProfile::Zero()),
    has_explicit_noninertial_stage_profiles_(false),
    has_car_beta_non_stage_profile_(false),
    car_trajectory_window_(),
    uav_radius_(0.25),
    last_obstacles_(),
    cbf_use_in_sim_(true),
    frame_mode_(FrameMode::kNonInertial),
    frame_mode_effective_("noninertial"),
    safety_variant_("A2_soft_cbf"),
    noise_rng_(1u),
    noise_dist_(0.0, 1.0),
    measured_car_linear_velocity_(Eigen::Vector3d::Zero()),
    measured_car_angular_velocity_(Eigen::Vector3d::Zero()),
    estimation_position_noise_std_(0.025),
    estimation_relative_velocity_noise_std_(0.025),
    estimation_orientation_noise_std_(0.0435),
    estimation_imu_non_noise_std_(0.025),
    car_linear_velocity_noise_std_(0.025),
    car_angular_velocity_noise_std_(0.0435),
    metrics_debug_(false),
    debug_check_noninertial_profile_col0_(false),
    debug_force_zero_a_car_non_profile_(false),
    debug_noninertial_profile_sim_time_(std::numeric_limits<double>::quiet_NaN()),
    debug_traj_id_(),
    warm_start_(true),
    inter_uav_priority_enabled_(true),
    obstacle_hysteresis_hold_cycles_(10),
    obstacle_hysteresis_cycles_since_switch_(0),
    obstacle_hysteresis_min_risk_improvement_(0.2),
    metrics_min_h_(std::numeric_limits<double>::infinity()),
    metrics_min_cbf_(std::numeric_limits<double>::infinity()),
    metrics_max_slack_(0.0),
    metrics_sum_h_(0.0),
    metrics_sum_cbf_(0.0),
    metrics_tracking_sum_sq_(0.0),
    metrics_tracking_sum_sq_xyz_(Eigen::Vector3d::Zero()),
    metrics_tracking_errors_(),
    metrics_tracking_sum_sq_track_only_(0.0),
    metrics_tracking_sum_sq_track_only_xyz_(Eigen::Vector3d::Zero()),
    metrics_tracking_errors_track_only_(),
    metrics_tracking_sum_sq_cbf_active_(0.0),
    metrics_tracking_sum_sq_solver_cbf_active_(0.0),
    metrics_tracking_sum_sq_turn_(0.0),
    metrics_tracking_sum_sq_straight_(0.0),
    metrics_diag_min_h_any_(std::numeric_limits<double>::infinity()),
    metrics_diag_min_cbf_any_(std::numeric_limits<double>::infinity()),
    metrics_diag_sum_h_any_(0.0),
    metrics_diag_sum_cbf_any_(0.0),
    metrics_solver_metric_samples_(0),
    metrics_solver_cbf_active_tracking_samples_(0),
    metrics_solver_track_only_samples_(0),
    metrics_solver_min_h_(std::numeric_limits<double>::infinity()),
    metrics_solver_min_planar_clearance_(std::numeric_limits<double>::infinity()),
    metrics_solver_min_h_witness_h_(std::numeric_limits<double>::quiet_NaN()),
    metrics_solver_min_h_witness_planar_clearance_(
        std::numeric_limits<double>::quiet_NaN()),
    metrics_solver_min_h_witness_tracking_error_(
        std::numeric_limits<double>::quiet_NaN()),
    metrics_solver_min_h_witness_delta_(std::numeric_limits<double>::quiet_NaN()),
    metrics_solver_min_h_witness_planar_distance_(
        std::numeric_limits<double>::quiet_NaN()),
    metrics_solver_min_h_witness_active_obstacle_key_("none"),
    metrics_dmin_min_(std::numeric_limits<double>::infinity()),
    metrics_slack_sum_(0.0),
    metrics_slack_sum_turn_(0.0),
    metrics_slack_sum_straight_(0.0),
    metrics_max_slack_turn_(0.0),
    metrics_max_slack_straight_(0.0),
    metrics_min_h_turn_(std::numeric_limits<double>::infinity()),
    metrics_min_h_straight_(std::numeric_limits<double>::infinity()),
    metrics_solve_time_ms_(),
    metrics_solve_time_max_ms_(0.0),
    metrics_feedback_time_ms_(),
    metrics_feedback_time_max_ms_(0.0),
    metrics_preparation_time_ms_(),
    metrics_preparation_time_max_ms_(0.0),
    metrics_failures_(0),
    metrics_failures_turn_(0),
    metrics_failures_straight_(0),
    metrics_cbf_active_samples_(0),
    metrics_turn_samples_(0),
    metrics_straight_samples_(0),
    metrics_samples_(0),
    turning_omega_threshold_(0.0),
    last_active_obstacle_key_("none"),
    last_cbf_active_(false),
    last_selected_obstacle_keys_(),
    last_solver_obstacle_profile_snapshot_(),
    last_predicted_world_positions_(),
    has_last_predicted_world_positions_(false),
    prev_step_debug_()
{/* +std::to_string(quad_id) */


    
  /*   for (int i = 0; i < num_uavs; i++) 
    {
    std::string topic_name = std::string(QUAD_ODOM_TOPIC) + std::to_string(i);
    quad_odom_pub_.push_back(nh_.advertise<nav_msgs::Odometry>(topic_name,1));
    } */

quad_odom_pub_.resize(num_uavs);
quad_odom_pub_ [quad_id]= 
      nh_.advertise<nav_msgs::Odometry>(
          acado_mpc_common::resolveTopicName(
              std::string(QUAD_ODOM_TOPIC) + std::to_string(quad_id)),
          1); 
  car_odom_pub_ = 
      nh_.advertise<nav_msgs::Odometry>(
          acado_mpc_common::resolveTopicName(CAR_ODOM_TOPIC),1); 
  car_path_pub_ =
      nh_.advertise<nav_msgs::Path>(
          acado_mpc_common::resolveTopicName(CAR_PATH_TOPIC), 1);
  car_path_marker_pub_ =
      nh_.advertise<visualization_msgs::Marker>(
          acado_mpc_common::resolveTopicName("coni_mpc/car_path_marker"), 1);
  car_body_marker_pub_ =
      nh_.advertise<visualization_msgs::Marker>(
          acado_mpc_common::resolveTopicName("coni_mpc/car_body"), 1);
quad_path_pub_.resize(num_uavs);
quad_path_pub_[quad_id] =
      nh_.advertise<nav_msgs::Path>(
          acado_mpc_common::resolveTopicName(
              std::string(QUAD_PATH_TOPIC) + std::to_string(quad_id)),
          1);
quad_path_marker_pub_.resize(num_uavs);
quad_path_marker_pub_[quad_id] =
      nh_.advertise<visualization_msgs::Marker>(
          acado_mpc_common::resolveTopicName(
              "coni_mpc/quad_path_marker" + std::to_string(quad_id)),
          1);
 /*    for (int i = 0; i < num_uavs; i++) 
    {
    std::string REL_topic_name = std::string(RELATIVE_EST_TOPIC) + std::to_string(i);
    relative_est_pub_.push_back(nh_.advertise<nav_msgs::Odometry>(REL_topic_name,1));
    } */
relative_est_pub_.resize(num_uavs);
relative_est_pub_[quad_id] = 
      nh_.advertise<nav_msgs::Odometry>(
          acado_mpc_common::resolveTopicName(
              std::string(RELATIVE_EST_TOPIC) + std::to_string(quad_id)),
          1);
  quad_radius_marker_pub_ =
      nh_.advertise<visualization_msgs::Marker>(
          acado_mpc_common::resolveTopicName("coni_mpc/quad_radius"), 10);
  quad_path_.header.frame_id = acado_mpc_common::worldFrameId();
  car_path_.header.frame_id = acado_mpc_common::worldFrameId();
  quad_path_.poses.reserve(kMaxPathLength);

  pnh_.param("uav_radius", uav_radius_, 0.25);
  if (uav_radius_ < 0.0) {
    ROS_WARN("[%s] Parameter uav_radius should be non-negative. Using zero.",
             pnh_.getNamespace().c_str());
    uav_radius_ = 0.0;
  }
  pnh_.param("car_radius", car_radius_, 0.3);
  if (car_radius_ < 0.0) {
    ROS_WARN("[%s] Parameter car_radius should be non-negative. Using zero.",
             pnh_.getNamespace().c_str());
    car_radius_ = 0.0;
  }

  {
    std::lock_guard<std::mutex> lock(shared_state_mutex_);
    if (shared_positions_.size() != static_cast<size_t>(num_uavs) ||
        shared_velocities_.size() != static_cast<size_t>(num_uavs) ||
        shared_position_valid_.size() != static_cast<size_t>(num_uavs)) {
      shared_positions_.assign(num_uavs, Eigen::Vector3d::Zero());
      shared_velocities_.assign(num_uavs, Eigen::Vector3d::Zero());
      shared_position_valid_.assign(num_uavs, false);
    }
  }

  if (!mpc_params_.loadParameters(pnh_)) {
    ROS_ERROR("[%s] Failed to load MPC parameters.", pnh_.getNamespace().c_str());
  }
  frame_mode_ = resolveFrameMode(pnh_, frame_mode_effective_);
  pnh_.param("safety_variant", safety_variant_, std::string("A2_soft_cbf"));
  int noise_seed = 1;
  pnh_.param("noise_seed", noise_seed, 1);
  if (noise_seed <= 0) {
    noise_seed = 1;
  }
  const uint32_t noise_seed_effective =
      static_cast<uint32_t>(noise_seed + std::max(0, quad_id) * 10007);
  noise_rng_.seed(noise_seed_effective);
  pnh_.param("cbf/use_in_sim", cbf_use_in_sim_, mpc_params_.cbf_enabled_);
  pnh_.param("cbf/use_risk_regions", risk_regions_enabled_, false);
  pnh_.param("cbf/risk_regions_timeout", risk_regions_timeout_, 4.0);
  pnh_.param("cbf/risk_region_time_alignment",
             risk_region_time_alignment_, true);
  pnh_.param("cbf/use_field_hocbf", field_hocbf_enabled_, false);
  if (field_hocbf_enabled_) {
    std::string field_topic = "/my_map/future_occupancy_3d";
    pnh_.param("cbf/field_topic", field_topic, field_topic);
    field_occupancy_subscriber_ = nh_.subscribe(
        field_topic, 1, &NumSimMpc::fieldOccupancyCallback, this);
    ROS_INFO_STREAM("[MPC UAV " << quad_id << "] using field-HOCBF topic "
                    << field_topic << ", retaining the latest DSP snapshot");
    ROS_INFO_STREAM("[MPC UAV " << quad_id << "] field-HOCBF config: "
                    << "sigma=" << field_hocbf::kSigma
                    << " Vmax=" << field_hocbf::kVmax
                    << " d_safe=" << field_hocbf::kDSafe
                    << " gamma1=" << field_hocbf::kGamma1
                    << " gamma2=" << field_hocbf::kGamma2
                    << " prune_radius=" << field_hocbf::kPruneRadius
                    << " (=3sigma) seed=10 max=35 separation=0.15");
  }
  if (risk_regions_enabled_) {
    std::string risk_region_topic = "/coni_mpc/risk_regions";
    pnh_.param("cbf/risk_region_topic", risk_region_topic, risk_region_topic);
    risk_region_subscriber_ = nh_.subscribe(
        risk_region_topic, 1, &NumSimMpc::riskRegionCallback, this);
    ROS_INFO_STREAM("[MPC UAV " << quad_id << "] using risk-region topic "
                    << risk_region_topic
                    << ", time_alignment="
                    << (risk_region_time_alignment_ ? "on" : "off")
                    << ", timeout=" << risk_regions_timeout_);
  }
  pnh_.param("estimation_noise/position_std",
             estimation_position_noise_std_, estimation_position_noise_std_);
  pnh_.param("estimation_noise/relative_velocity_std",
             estimation_relative_velocity_noise_std_,
             estimation_relative_velocity_noise_std_);
  pnh_.param("estimation_noise/orientation_std",
             estimation_orientation_noise_std_,
             estimation_orientation_noise_std_);
  pnh_.param("estimation_noise/imu_non_std",
             estimation_imu_non_noise_std_, estimation_imu_non_noise_std_);
  pnh_.param("estimation_noise/car_linear_velocity_std",
             car_linear_velocity_noise_std_,
             car_linear_velocity_noise_std_);
  pnh_.param("estimation_noise/car_angular_velocity_std",
             car_angular_velocity_noise_std_,
             car_angular_velocity_noise_std_);
  pnh_.param("metrics/debug", metrics_debug_, false);
  pnh_.param("debug/check_noninertial_profile_col0",
             debug_check_noninertial_profile_col0_, false);
  pnh_.param("debug/force_zero_a_car_non_profile",
             debug_force_zero_a_car_non_profile_, false);
  pnh_.param("warm_start", warm_start_, true);
  pnh_.param("cbf/inter_uav_priority_enabled", inter_uav_priority_enabled_, true);
  pnh_.param("cbf/obstacle_hysteresis_hold_cycles",
             obstacle_hysteresis_hold_cycles_, 10);
  pnh_.param("cbf/obstacle_hysteresis_min_risk_improvement",
             obstacle_hysteresis_min_risk_improvement_, 0.2);
  obstacle_hysteresis_hold_cycles_ = std::max(0, obstacle_hysteresis_hold_cycles_);
  obstacle_hysteresis_min_risk_improvement_ =
      std::max(0.0, obstacle_hysteresis_min_risk_improvement_);
  mpc_controller_.setWarmStart(warm_start_);
  ROS_INFO_STREAM("[" << pnh_.getNamespace() << "] frame_mode_effective="
                  << frame_mode_effective_
                  << " safety_variant_effective=" << safety_variant_
                  << " cbf_slack_max=" << mpc_params_.cbf_slack_max_
                  << " inter_uav_priority_enabled="
                  << (inter_uav_priority_enabled_ ? "true" : "false")
                  << " obstacle_hysteresis_hold_cycles="
                  << obstacle_hysteresis_hold_cycles_
                  << " obstacle_hysteresis_min_risk_improvement="
                  << obstacle_hysteresis_min_risk_improvement_
                  << " noise_seed_effective=" << noise_seed_effective);
  resetMetrics();
}

void NumSimMpc::setQuadOdom(const nav_msgs::Odometry &quad_odom, int i)
{
  quad_odom_ = quad_odom;
  quad_id = i;
}

void NumSimMpc::setSharedQuadOdom(const nav_msgs::Odometry &quad_odom, int i)
{
  const Eigen::Vector3d position(
      quad_odom.pose.pose.position.x,
      quad_odom.pose.pose.position.y,
      quad_odom.pose.pose.position.z);
  const Eigen::Vector3d velocity(
      quad_odom.twist.twist.linear.x,
      quad_odom.twist.twist.linear.y,
      quad_odom.twist.twist.linear.z);

  std::lock_guard<std::mutex> lock(shared_state_mutex_);
  if (shared_positions_.size() != static_cast<size_t>(num_uavs) ||
      shared_velocities_.size() != static_cast<size_t>(num_uavs) ||
      shared_position_valid_.size() != static_cast<size_t>(num_uavs)) {
    shared_positions_.assign(num_uavs, Eigen::Vector3d::Zero());
    shared_velocities_.assign(num_uavs, Eigen::Vector3d::Zero());
    shared_position_valid_.assign(num_uavs, false);
  }
  if (i >= 0 && static_cast<size_t>(i) < shared_positions_.size()) {
    shared_positions_[static_cast<size_t>(i)] = position;
    shared_velocities_[static_cast<size_t>(i)] = velocity;
    shared_position_valid_[static_cast<size_t>(i)] = true;
  }
}

void NumSimMpc::setCarOdom(const nav_msgs::Odometry &car_odom)
{
  car_odom_ = car_odom;
  measured_car_linear_velocity_ =
      Eigen::Vector3d(car_odom.twist.twist.linear.x,
                      car_odom.twist.twist.linear.y,
                      car_odom.twist.twist.linear.z);
  measured_car_angular_velocity_ =
      Eigen::Vector3d(car_odom.twist.twist.angular.x,
                      car_odom.twist.twist.angular.y,
                      car_odom.twist.twist.angular.z);
}

void NumSimMpc::riskRegionCallback(
    const coni_mpc::RiskRegionArray::ConstPtr& message) {
  if (!message) {
    return;
  }
  std::lock_guard<std::mutex> lock(risk_region_mutex_);
  latest_risk_regions_ = *message;
  has_latest_risk_regions_ = true;
}

void NumSimMpc::fieldOccupancyCallback(
    const sensor_msgs::PointCloud2::ConstPtr& message) {
  if (!message) return;
  if (message->header.stamp.isZero()) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[MPC UAV " << quad_id
        << "] rejecting field cloud with zero timestamp");
    return;
  }
  std::string frame = message->header.frame_id;
  if (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
  if (frame != "map") {
    ROS_WARN_STREAM_THROTTLE(1.0, "[MPC UAV " << quad_id
        << "] rejecting field cloud in frame '" << message->header.frame_id
        << "' (expected map)");
    return;
  }
  // The DSP publisher provides x/y/z, vx/vy/vz, occupancy and stage_index.
  // prediction_time is accepted when present for backward compatibility, but
  // it is not required to construct the stage-indexed snapshot.
  const char* required[] = {"x", "y", "z", "vx", "vy", "vz",
                            "occupancy", "stage_index"};
  std::array<const sensor_msgs::PointField*, 8> fields{};
  for (std::size_t i = 0; i < fields.size(); ++i) {
    const auto it = std::find_if(message->fields.begin(), message->fields.end(),
        [&](const sensor_msgs::PointField& field) { return field.name == required[i]; });
    if (it == message->fields.end() || it->count != 1) {
      ROS_WARN_STREAM_THROTTLE(1.0, "[MPC UAV " << quad_id
          << "] field cloud missing scalar field " << required[i]);
      return;
    }
    fields[i] = &*it;
  }
  const sensor_msgs::PointField* prediction_time_field = nullptr;
  const auto prediction_time_it = std::find_if(
      message->fields.begin(), message->fields.end(),
      [](const sensor_msgs::PointField& field) {
        return field.name == "prediction_time" && field.count == 1;
      });
  if (prediction_time_it != message->fields.end()) {
    prediction_time_field = &*prediction_time_it;
  }
  auto read = [](const std::uint8_t* ptr, std::uint8_t datatype,
                 double& value) -> bool {
    switch (datatype) {
      case sensor_msgs::PointField::FLOAT32: { float v; std::memcpy(&v, ptr, 4); value = v; return true; }
      case sensor_msgs::PointField::FLOAT64: { double v; std::memcpy(&v, ptr, 8); value = v; return true; }
      case sensor_msgs::PointField::UINT8: { std::uint8_t v; std::memcpy(&v, ptr, 1); value = v; return true; }
      case sensor_msgs::PointField::UINT16: { std::uint16_t v; std::memcpy(&v, ptr, 2); value = v; return true; }
      case sensor_msgs::PointField::UINT32: { std::uint32_t v; std::memcpy(&v, ptr, 4); value = v; return true; }
      case sensor_msgs::PointField::INT8: { std::int8_t v; std::memcpy(&v, ptr, 1); value = v; return true; }
      case sensor_msgs::PointField::INT16: { std::int16_t v; std::memcpy(&v, ptr, 2); value = v; return true; }
      case sensor_msgs::PointField::INT32: { std::int32_t v; std::memcpy(&v, ptr, 4); value = v; return true; }
      default: return false;
    }
  };
  auto snapshot = std::make_shared<FieldSnapshot>();
  snapshot->stamp = message->header.stamp;
  snapshot->frame_id = frame;
  const std::size_t count = static_cast<std::size_t>(message->width) * message->height;
  snapshot->points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t base = i * message->point_step;
    if (base + message->point_step > message->data.size()) break;
    double values[8]{};
    bool valid = true;
    for (std::size_t f = 0; f < fields.size(); ++f) {
      if (fields[f]->offset >= message->point_step ||
          !read(message->data.data() + base + fields[f]->offset,
                fields[f]->datatype, values[f]) || !std::isfinite(values[f])) {
        valid = false;
        break;
      }
    }
    double prediction_time = static_cast<double>(values[7]) * 0.1;
    if (valid && prediction_time_field != nullptr) {
      if (prediction_time_field->offset >= message->point_step ||
          !read(message->data.data() + base + prediction_time_field->offset,
                prediction_time_field->datatype, prediction_time) ||
          !std::isfinite(prediction_time)) {
        valid = false;
      }
    }
    const double rounded_stage = std::round(values[7]);
    if (!valid || values[6] < 0.0 || values[6] > 1.0 ||
        values[7] < 0.0 || rounded_stage > acado_mpc::kSamples ||
        std::abs(values[7] - rounded_stage) > 1e-6) continue;
    field_hocbf::Point point;
    point.position_world = Eigen::Vector3d(values[0], values[1], values[2]);
    point.velocity_world = Eigen::Vector3d(values[3], values[4], values[5]);
    point.occupancy = values[6];
    point.prediction_time = prediction_time;
    point.stage_index = static_cast<std::uint32_t>(rounded_stage);
    snapshot->points.push_back(point);
    snapshot->last_occupied_stage =
        std::max(snapshot->last_occupied_stage, point.stage_index);
  }
  std::lock_guard<std::mutex> lock(field_snapshot_mutex_);
  field_snapshot_ = std::move(snapshot);
}

void NumSimMpc::getSharedPositions(std::vector<Eigen::Vector3d>& positions,
                                   std::vector<Eigen::Vector3d>& velocities,
                                   std::vector<bool>& valid)
{
  std::lock_guard<std::mutex> lock(shared_state_mutex_);
  positions = shared_positions_;
  velocities = shared_velocities_;
  valid = shared_position_valid_;
}

void NumSimMpc::getEstimatedWorldKinematics(
    Eigen::Vector3d& position_world,
    Eigen::Vector3d& velocity_world) const
{
  if (isInertialFrame()) {
    position_world = state_estimate_.position;
    velocity_world = state_estimate_.velocity;
    return;
  }

  Eigen::Quaterniond world_q_non = getCarOrientation();
  if (world_q_non.norm() > 1e-9) {
    world_q_non.normalize();
  } else {
    world_q_non = Eigen::Quaterniond::Identity();
  }
  const Eigen::Vector3d position_relative_non = state_estimate_.position;
  position_world = getCarPosition() + world_q_non * position_relative_non;
  velocity_world =
      getCarLinearVelocity() +
      world_q_non * (state_estimate_.velocity +
                     getCarAngularVelocity().cross(position_relative_non));
}

void NumSimMpc::setStaticObstacles(const ObstacleVector& obstacles)
{
  std::lock_guard<std::mutex> lock(shared_state_mutex_);
  shared_static_obstacles_ = obstacles;
}

void NumSimMpc::runReactiveBaselineStep()
{
  last_solver_obstacle_profile_snapshot_ = SolverObstacleProfileSnapshot();
  last_solver_obstacle_profile_snapshot_.uav_idx = quad_id;
  last_predicted_world_positions_.clear();
  has_last_predicted_world_positions_ = false;
  last_obstacles_.clear();
  last_selected_obstacle_keys_.clear();
  last_active_obstacle_key_ = "none";
  last_cbf_active_ = false;

  genRelativeEstimate();
  setEstimationNoise(estimation_position_noise_std_,
                     estimation_relative_velocity_noise_std_,
                     estimation_orientation_noise_std_,
                     estimation_imu_non_noise_std_);

  car_odom_pub_.publish(car_odom_);
  geometry_msgs::PoseStamped car_pose_stamped;
  car_pose_stamped.header = car_odom_.header;
  car_pose_stamped.pose = car_odom_.pose.pose;
  car_path_.header.stamp = car_pose_stamped.header.stamp;
  car_path_.poses.push_back(car_pose_stamped);
  car_path_pub_.publish(car_path_);

  const std::size_t qidx = static_cast<std::size_t>(std::max(0, quad_id));
  if (qidx < relative_est_pub_.size()) {
    relative_est_pub_[qidx].publish(relative_est_);
  }
  if (qidx < quad_odom_pub_.size()) {
    quad_odom_pub_[qidx].publish(quad_odom_);
  }
  geometry_msgs::PoseStamped quad_pose_stamped;
  quad_pose_stamped.header = quad_odom_.header;
  quad_pose_stamped.pose = quad_odom_.pose.pose;
  quad_path_.header.stamp = quad_pose_stamped.header.stamp;
  quad_path_.poses.push_back(quad_pose_stamped);
  if (qidx < quad_path_pub_.size()) {
    quad_path_pub_[qidx].publish(quad_path_);
  }

  mpc_params_.cbf_obstacles_.clear();
  mpc_params_.cbf_obstacle_profiles_.clear();
  mpc_params_.cbf_enabled_ = false;
  mpc_params_.changed_ = false;

  const double nan = std::numeric_limits<double>::quiet_NaN();
  metrics_samples_ += 1;
  const bool turning =
      (turning_omega_threshold_ > 0.0 &&
       std::abs(car_odom_.twist.twist.angular.z) >= turning_omega_threshold_);
  if (turning) {
    metrics_turn_samples_ += 1;
  } else {
    metrics_straight_samples_ += 1;
  }

  const double c_value = std::abs(car_odom_.twist.twist.angular.z);
  metrics_c_values_.push_back(c_value);
  const Eigen::Vector2d relative_position_xy(
      quad_odom_.pose.pose.position.x - car_odom_.pose.pose.position.x,
      quad_odom_.pose.pose.position.y - car_odom_.pose.pose.position.y);
  metrics_c66_coupled_values_.push_back(c_value * relative_position_xy.norm());

  metrics_step_min_h_.push_back(nan);
  metrics_step_slack_.push_back(0.0);
  metrics_step_tracking_error_.push_back(nan);
  metrics_step_fail_flags_.push_back(0);
  metrics_solver_track_only_samples_ += 1;

  if (!reference_window_.points.empty()) {
    const Eigen::Vector3d tracking_error =
        state_estimate_.position - reference_window_.points.front().position;
    const double e = tracking_error.norm();
    metrics_tracking_sum_sq_ += e * e;
    metrics_tracking_sum_sq_xyz_ += tracking_error.cwiseProduct(tracking_error);
    metrics_tracking_errors_.push_back(e);
    metrics_tracking_sum_sq_track_only_ += e * e;
    metrics_tracking_sum_sq_track_only_xyz_ +=
        tracking_error.cwiseProduct(tracking_error);
    metrics_tracking_errors_track_only_.push_back(e);
    if (turning) {
      metrics_tracking_sum_sq_turn_ += e * e;
    } else {
      metrics_tracking_sum_sq_straight_ += e * e;
    }
    metrics_step_tracking_error_.back() = e;
  }

  metrics_slack_sum_ += 0.0;
  metrics_max_slack_ = std::max(metrics_max_slack_, 0.0);
  if (turning) {
    metrics_slack_sum_turn_ += 0.0;
    metrics_max_slack_turn_ = std::max(metrics_max_slack_turn_, 0.0);
  } else {
    metrics_slack_sum_straight_ += 0.0;
    metrics_max_slack_straight_ = std::max(metrics_max_slack_straight_, 0.0);
  }
  metrics_solve_time_ms_.push_back(0.0);
  metrics_feedback_time_ms_.push_back(0.0);
  metrics_preparation_time_ms_.push_back(0.0);

  const Eigen::Vector3d quad_position_world(
      quad_odom_.pose.pose.position.x,
      quad_odom_.pose.pose.position.y,
      quad_odom_.pose.pose.position.z);
  double step_physical_surface_distance =
      std::numeric_limits<double>::infinity();
  ObstacleVector static_obstacles_copy;
  {
    std::lock_guard<std::mutex> lock(shared_state_mutex_);
    static_obstacles_copy = shared_static_obstacles_;
  }
  for (const auto& obstacle : static_obstacles_copy) {
    const double obstacle_radius = std::max(0.0, obstacle(3));
    const double surface_distance =
        (quad_position_world.head<2>() - obstacle.head<2>()).norm() -
        (std::max(0.0, uav_radius_) + obstacle_radius);
    step_physical_surface_distance =
        std::min(step_physical_surface_distance, surface_distance);
  }
  std::vector<Eigen::Vector3d> peer_positions;
  std::vector<Eigen::Vector3d> peer_velocities;
  std::vector<bool> peer_valid;
  getSharedPositions(peer_positions, peer_velocities, peer_valid);
  (void)peer_velocities;
  for (std::size_t idx = 0; idx < peer_positions.size(); ++idx) {
    if (idx == qidx || idx >= peer_valid.size() || !peer_valid[idx]) {
      continue;
    }
    const double surface_distance =
        (quad_position_world.head<2>() - peer_positions[idx].head<2>()).norm() -
        2.0 * std::max(0.0, uav_radius_);
    step_physical_surface_distance =
        std::min(step_physical_surface_distance, surface_distance);
  }
  if (std::isfinite(step_physical_surface_distance)) {
    metrics_dmin_min_ =
        std::min(metrics_dmin_min_, step_physical_surface_distance);
  }

  StepDebugSnapshot current_step_debug;
  current_step_debug.valid = true;
  current_step_debug.cbf_active = false;
  current_step_debug.solver_cbf_active = false;
  current_step_debug.solve_ok = true;
  current_step_debug.solver_fail_flag = false;
  current_step_debug.turning = turning;
  current_step_debug.active_obstacle_key = "none";
  current_step_debug.solver_active_obstacle_key = "none";
  current_step_debug.solver_track_only = true;
  current_step_debug.solver_h = nan;
  current_step_debug.solver_hdot = nan;
  current_step_debug.solver_cbf = nan;
  current_step_debug.solver_slack = 0.0;
  current_step_debug.solver_planar_distance = nan;
  current_step_debug.solver_planar_clearance = nan;
  current_step_debug.solver_planar_surface_distance = nan;
  current_step_debug.feedback_time_ms = 0.0;
  current_step_debug.preparation_time_ms = 0.0;
  current_step_debug.core_time_ms = 0.0;
  current_step_debug.tracking_error = metrics_step_tracking_error_.back();
  current_step_debug.diag_min_h_any = nan;
  current_step_debug.diag_min_cbf_any = nan;
  current_step_debug.min_h = nan;
  current_step_debug.min_cbf = nan;
  current_step_debug.slack = 0.0;
  current_step_debug.slack_raw = 0.0;
  current_step_debug.omega_non = state_estimate_.omega_non;
  current_step_debug.beta_non = state_estimate_.beta_non;
  current_step_debug.a_car_non =
      state_estimate_.a_imu - Eigen::Vector3d(0.0, 0.0, 9.81);
  if (!car_trajectory_window_.empty()) {
    current_step_debug.car_a_world = Eigen::Vector3d(
        car_trajectory_window_.front()(10),
        car_trajectory_window_.front()(11),
        car_trajectory_window_.front()(12));
    current_step_debug.car_omega_world = Eigen::Vector3d(
        car_trajectory_window_.front()(13),
        car_trajectory_window_.front()(14),
        car_trajectory_window_.front()(15));
  }
  current_step_debug.position = state_estimate_.position;
  current_step_debug.velocity = state_estimate_.velocity;

  Eigen::Quaterniond W_q_non(car_odom_.pose.pose.orientation.w,
                             car_odom_.pose.pose.orientation.x,
                             car_odom_.pose.pose.orientation.y,
                             car_odom_.pose.pose.orientation.z);
  if (W_q_non.norm() > 1e-9) {
    W_q_non.normalize();
  } else {
    W_q_non = Eigen::Quaterniond::Identity();
  }
  const Eigen::Vector3d car_position_world(
      car_odom_.pose.pose.position.x,
      car_odom_.pose.pose.position.y,
      car_odom_.pose.pose.position.z);
  const Eigen::Vector3d car_velocity_world(
      car_odom_.twist.twist.linear.x,
      car_odom_.twist.twist.linear.y,
      car_odom_.twist.twist.linear.z);
  const Eigen::Vector3d quad_velocity_world(
      quad_odom_.twist.twist.linear.x,
      quad_odom_.twist.twist.linear.y,
      quad_odom_.twist.twist.linear.z);
  const Eigen::Vector3d car_omega_non_raw(
      car_odom_.twist.twist.angular.x,
      car_odom_.twist.twist.angular.y,
      car_odom_.twist.twist.angular.z);
  const Eigen::Vector3d position_relative_non =
      W_q_non.inverse() * (quad_position_world - car_position_world);
  const Eigen::Vector3d velocity_relative_non =
      -car_omega_non_raw.cross(position_relative_non) +
      W_q_non.inverse() * (quad_velocity_world - car_velocity_world);
  current_step_debug.rot_load_position_non = position_relative_non;
  current_step_debug.rot_load_velocity_non = velocity_relative_non;
  current_step_debug.rot_load_omega_non = car_omega_non_raw;

  prev_step_debug_ = current_step_debug;
}

// helper function
inline static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &vec) {
  Eigen::Matrix3d result;
  result << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return result;
}

void NumSimMpc::genRelativeEstimate()
{
  Eigen::Quaterniond W_q_quad(quad_odom_.pose.pose.orientation.w, 
                              quad_odom_.pose.pose.orientation.x, 
                              quad_odom_.pose.pose.orientation.y, 
                              quad_odom_.pose.pose.orientation.z);
  Eigen::Quaterniond W_q_non(car_odom_.pose.pose.orientation.w, 
                            car_odom_.pose.pose.orientation.x, 
                            car_odom_.pose.pose.orientation.y, 
                            car_odom_.pose.pose.orientation.z);
  W_q_quad.normalize();
  W_q_non.normalize();
  const Eigen::Vector3d W_position_quad(
      quad_odom_.pose.pose.position.x,
      quad_odom_.pose.pose.position.y,
      quad_odom_.pose.pose.position.z);
  const Eigen::Vector3d W_velocity_quad(
      quad_odom_.twist.twist.linear.x,
      quad_odom_.twist.twist.linear.y,
      quad_odom_.twist.twist.linear.z);
  const Eigen::Vector3d car_angular_velocity_raw(
      car_odom_.twist.twist.angular.x,
      car_odom_.twist.twist.angular.y,
      car_odom_.twist.twist.angular.z);
  const Eigen::Vector3d car_linear_velocity_raw(
      car_odom_.twist.twist.linear.x,
      car_odom_.twist.twist.linear.y,
      car_odom_.twist.twist.linear.z);
  measured_car_linear_velocity_ =
      car_linear_velocity_raw +
      Eigen::Vector3d(car_linear_velocity_noise_std_ * sampleNormal(),
                      car_linear_velocity_noise_std_ * sampleNormal(),
                      car_linear_velocity_noise_std_ * sampleNormal());
  measured_car_angular_velocity_ =
      car_angular_velocity_raw +
      Eigen::Vector3d(car_angular_velocity_noise_std_ * sampleNormal(),
                      car_angular_velocity_noise_std_ * sampleNormal(),
                      car_angular_velocity_noise_std_ * sampleNormal());
  if (isInertialFrame()) {
    relative_est_.header.stamp = quad_odom_.header.stamp;
    state_estimate_.position = W_position_quad;
    state_estimate_.velocity = W_velocity_quad;
    state_estimate_.orientation = W_q_quad;
    // In inertial mode, frame compensation terms should be zero.
    // Keep gravity-only convention so a_car_non = a_imu - g = 0.
    state_estimate_.a_imu = Eigen::Vector3d(0.0, 0.0, 9.81);
    state_estimate_.omega_non = Eigen::Vector3d::Zero();
    state_estimate_.beta_non = Eigen::Vector3d::Zero();

    relative_est_.header.frame_id = acado_mpc_common::worldFrameId();
    relative_est_.pose.pose.position.x = state_estimate_.position.x();
    relative_est_.pose.pose.position.y = state_estimate_.position.y();
    relative_est_.pose.pose.position.z = state_estimate_.position.z();
    relative_est_.twist.twist.linear.x = state_estimate_.velocity.x();
    relative_est_.twist.twist.linear.y = state_estimate_.velocity.y();
    relative_est_.twist.twist.linear.z = state_estimate_.velocity.z();
    relative_est_.pose.pose.orientation.w = state_estimate_.orientation.w();
    relative_est_.pose.pose.orientation.x = state_estimate_.orientation.x();
    relative_est_.pose.pose.orientation.y = state_estimate_.orientation.y();
    relative_est_.pose.pose.orientation.z = state_estimate_.orientation.z();
    return;
  }

  Eigen::Vector3d W_position_relative(
      quad_odom_.pose.pose.position.x - 
          car_odom_.pose.pose.position.x, 
      quad_odom_.pose.pose.position.y - 
          car_odom_.pose.pose.position.y, 
      quad_odom_.pose.pose.position.z - 
          car_odom_.pose.pose.position.z);
  Eigen::Vector3d non_position_relative = 
      W_q_non.inverse() * W_position_relative;
  state_estimate_.position = non_position_relative;
  const Eigen::Vector3d& W_velocity_non = measured_car_linear_velocity_;
  Eigen::Vector3d w_velocity_relative_linear = 
      W_velocity_quad - W_velocity_non;
  Eigen::Vector3d non_velocity_relative = 
      -skewSymmetric(measured_car_angular_velocity_) * non_position_relative + 
      W_q_non.inverse() * w_velocity_relative_linear;
  state_estimate_.velocity = non_velocity_relative;
  state_estimate_.orientation = W_q_non.inverse() * W_q_quad;
  // Keep acceleration represented in non-inertial frame N.
  state_estimate_.a_imu = W_q_non.inverse() * imu_;
  state_estimate_.omega_non = measured_car_angular_velocity_;
  state_estimate_.beta_non = car_beta_non_;

  relative_est_.header.stamp = quad_odom_.header.stamp;
  relative_est_.header.frame_id = acado_mpc_common::relativeFrameId();
  relative_est_.pose.pose.position.x = state_estimate_.position.x();
  relative_est_.pose.pose.position.y = state_estimate_.position.y();
  relative_est_.pose.pose.position.z = state_estimate_.position.z();
  relative_est_.twist.twist.linear.x = state_estimate_.velocity.x();
  relative_est_.twist.twist.linear.y = state_estimate_.velocity.y();
  relative_est_.twist.twist.linear.z = state_estimate_.velocity.z();
  relative_est_.pose.pose.orientation.w = state_estimate_.orientation.w();
  relative_est_.pose.pose.orientation.x = state_estimate_.orientation.x();
  relative_est_.pose.pose.orientation.y = state_estimate_.orientation.y();
  relative_est_.pose.pose.orientation.z = state_estimate_.orientation.z();
}

void NumSimMpc::buildNonInertialDataProfile(
    NonInertialProfile& omega_non_profile,
    NonInertialProfile& beta_non_profile,
    NonInertialProfile& a_car_non_profile) const {
  omega_non_profile.setZero();
  beta_non_profile.setZero();
  a_car_non_profile.setZero();

  if (isInertialFrame()) {
    return;
  }

  if (has_explicit_noninertial_stage_profiles_) {
    omega_non_profile = car_omega_non_stage_profile_;
    beta_non_profile = car_beta_non_stage_profile_;
    if (debug_force_zero_a_car_non_profile_) {
      a_car_non_profile.setZero();
    } else {
      a_car_non_profile = car_a_car_non_stage_profile_;
    }
  } else {
    const double prediction_dt = mpc_controller_.getPredictionDt();
    const CarState_t fallback_state = carStateFromOdom(car_odom_);
    const bool have_window = !car_trajectory_window_.empty();
    auto car_state_at_stage = [&](std::size_t k) -> const CarState_t& {
      if (have_window && k < car_trajectory_window_.size()) {
        return car_trajectory_window_[k];
      }
      if (have_window) {
        return car_trajectory_window_.back();
      }
      return fallback_state;
    };

    for (int k = 0; k < acado_mpc::kSamples + 1; ++k) {
      const CarState_t& car_state =
          car_state_at_stage(static_cast<std::size_t>(k));
      Eigen::Quaterniond W_q_non(car_state(6), car_state(7), car_state(8),
                                 car_state(9));
      if (W_q_non.norm() > 1e-9) {
        W_q_non.normalize();
      } else {
        W_q_non = Eigen::Quaterniond::Identity();
      }
      const Eigen::Vector3d omega_world(car_state(13), car_state(14),
                                        car_state(15));
      const Eigen::Vector3d a_world(car_state(10), car_state(11), car_state(12));
      omega_non_profile.col(k) = W_q_non.inverse() * omega_world;
      if (debug_force_zero_a_car_non_profile_) {
        a_car_non_profile.col(k).setZero();
      } else {
        a_car_non_profile.col(k) = W_q_non.inverse() * a_world;
      }
    }

    if (has_car_beta_non_stage_profile_) {
      beta_non_profile = car_beta_non_stage_profile_;
    } else {
      beta_non_profile.col(0) = car_beta_non_;
      if (!(std::isfinite(prediction_dt) && prediction_dt > 1e-6)) {
        for (int k = 1; k < acado_mpc::kSamples + 1; ++k) {
          beta_non_profile.col(k) = beta_non_profile.col(0);
        }
      } else {
        for (int k = 1; k < acado_mpc::kSamples + 1; ++k) {
          // Keep the fallback profile causal: stage k only depends on the
          // current and previous omega samples.
          beta_non_profile.col(k) =
              (omega_non_profile.col(k) - omega_non_profile.col(k - 1)) /
              prediction_dt;
        }
      }
    }
  }

  // Keep stage 0 aligned with the current estimator-side non-inertial data,
  // even when an explicit stage profile is provided.
  Eigen::Vector3d legacy_omega_non = Eigen::Vector3d::Zero();
  Eigen::Vector3d legacy_beta_non = Eigen::Vector3d::Zero();
  Eigen::Vector3d legacy_a_car_non = Eigen::Vector3d::Zero();
  acado_mpc::MpcController<double>::buildLegacyCurrentTimeNonInertialData(
      state_estimate_, legacy_omega_non, legacy_beta_non, legacy_a_car_non);
  omega_non_profile.col(0) = legacy_omega_non;
  beta_non_profile.col(0) = legacy_beta_non;
  if (debug_force_zero_a_car_non_profile_) {
    a_car_non_profile.col(0).setZero();
  } else {
    a_car_non_profile.col(0) = legacy_a_car_non;
  }
}

double NumSimMpc::sampleNormal()
{
  return noise_dist_(noise_rng_);
}

void NumSimMpc::syncRelativeEstimateMsgFromStateEstimate() {
  relative_est_.pose.pose.position.x = state_estimate_.position.x();
  relative_est_.pose.pose.position.y = state_estimate_.position.y();
  relative_est_.pose.pose.position.z = state_estimate_.position.z();
  relative_est_.twist.twist.linear.x = state_estimate_.velocity.x();
  relative_est_.twist.twist.linear.y = state_estimate_.velocity.y();
  relative_est_.twist.twist.linear.z = state_estimate_.velocity.z();
  relative_est_.pose.pose.orientation.w = state_estimate_.orientation.w();
  relative_est_.pose.pose.orientation.x = state_estimate_.orientation.x();
  relative_est_.pose.pose.orientation.y = state_estimate_.orientation.y();
  relative_est_.pose.pose.orientation.z = state_estimate_.orientation.z();
}

void NumSimMpc::setEstimationNoise(double p_var, 
                                  double v_var, 
                                  double ori_var, 
                                  double imu_non_var)
{
  state_estimate_.position += 
      Eigen::Vector3d(p_var * sampleNormal(), 
                      p_var * sampleNormal(), 
                      p_var * sampleNormal());
  state_estimate_.velocity += 
      Eigen::Vector3d(v_var * sampleNormal(), 
                      v_var * sampleNormal(), 
                      v_var * sampleNormal());
  Eigen::AngleAxisd temp(state_estimate_.orientation);
  temp.angle() += ori_var * sampleNormal();
  state_estimate_.orientation = Eigen::Quaterniond(temp);
  state_estimate_.a_imu += 
      Eigen::Vector3d(imu_non_var * sampleNormal(), 
                      imu_non_var * sampleNormal(), 
                      imu_non_var * sampleNormal());
  syncRelativeEstimateMsgFromStateEstimate();
}


acado_mpc_common::ControlCommand NumSimMpc::run( ) 
{
  last_solver_obstacle_profile_snapshot_ = SolverObstacleProfileSnapshot();
  last_solver_obstacle_profile_snapshot_.uav_idx = quad_id;
  genRelativeEstimate();
  setEstimationNoise(estimation_position_noise_std_,
                     estimation_relative_velocity_noise_std_,
                     estimation_orientation_noise_std_,
                     estimation_imu_non_noise_std_);

  car_odom_pub_.publish(car_odom_);
  // Append car path (non-fading)
  geometry_msgs::PoseStamped car_pose_stamped;
  car_pose_stamped.header = car_odom_.header;
  car_pose_stamped.pose = car_odom_.pose.pose;
  car_path_.header.stamp = car_pose_stamped.header.stamp;
  car_path_.poses.push_back(car_pose_stamped);
  car_path_pub_.publish(car_path_);
  if (!car_path_.poses.empty()) {
    visualization_msgs::Marker car_path_marker;
    car_path_marker.header = car_odom_.header;
    car_path_marker.header.frame_id = acado_mpc_common::worldFrameId();
    car_path_marker.ns = "car_path_marker";
    car_path_marker.id = 0;
    car_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    car_path_marker.action = visualization_msgs::Marker::ADD;
    car_path_marker.pose.orientation.w = 1.0;
    car_path_marker.scale.x = 0.04;
    const size_t n_car = car_path_.poses.size();
    car_path_marker.points.reserve(n_car);
    car_path_marker.colors.reserve(n_car);
    for (size_t idx = 0; idx < n_car; ++idx) {
      const auto& pose = car_path_.poses[idx];
      geometry_msgs::Point p;
      p.x = pose.pose.position.x;
      p.y = pose.pose.position.y;
      p.z = pose.pose.position.z;
      car_path_marker.points.push_back(p);
      std_msgs::ColorRGBA c;
      // Car path: solid black
      c.r = 0.0;
      c.g = 0.0;
      c.b = 0.0;
      c.a = 1.0;
      car_path_marker.colors.push_back(c);
    }
    car_path_marker.lifetime = ros::Duration(0.0);
    car_path_marker_pub_.publish(car_path_marker);
  }
  constexpr double kJackalChassisLengthM = 0.420;
  constexpr double kJackalWheelbaseM = 0.262;
  constexpr double kJackalTrackM = 0.37559;
  constexpr double kJackalWheelVerticalOffsetM = 0.0345;
  constexpr double kJackalFootprintOffsetM = -0.0655;
  const double car_target_length = std::max(0.2, 2.8 * car_radius_);
  const double car_uniform_scale = car_target_length / kJackalChassisLengthM;
  const Eigen::Quaterniond identity_q = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond chassis_q(
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
  const Eigen::Quaterniond wheel_q(
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
  const Eigen::Quaterniond rear_fender_q(
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  const auto chassis_color = makeColor(0.18, 0.18, 0.20, 0.98);
  const auto fender_color = makeColor(0.84, 0.72, 0.12, 0.98);
  const auto wheel_color = makeColor(0.10, 0.10, 0.10, 0.98);
  const geometry_msgs::Pose& car_pose = car_odom_.pose.pose;
  const std_msgs::Header& car_header = car_odom_.header;
  const Eigen::Vector3d scaled_front_left_wheel(
      0.5 * kJackalWheelbaseM * car_uniform_scale,
      0.5 * kJackalTrackM * car_uniform_scale,
      kJackalWheelVerticalOffsetM * car_uniform_scale);
  const Eigen::Vector3d scaled_front_right_wheel(
      scaled_front_left_wheel.x(), -scaled_front_left_wheel.y(),
      scaled_front_left_wheel.z());
  const Eigen::Vector3d scaled_rear_left_wheel(
      -scaled_front_left_wheel.x(), scaled_front_left_wheel.y(),
      scaled_front_left_wheel.z());
  const Eigen::Vector3d scaled_rear_right_wheel(
      -scaled_front_left_wheel.x(), -scaled_front_left_wheel.y(),
      scaled_front_left_wheel.z());
  car_body_marker_pub_.publish(makeMeshMarker(
      car_header, car_pose, "package://coni_mpc/meshes/jackal-base.stl",
      Eigen::Vector3d(0.0, 0.0, kJackalFootprintOffsetM * car_uniform_scale),
      chassis_q, car_uniform_scale, chassis_color, 0));
  car_body_marker_pub_.publish(makeMeshMarker(
      car_header, car_pose, "package://coni_mpc/meshes/jackal-fender.stl",
      Eigen::Vector3d::Zero(), identity_q, car_uniform_scale, fender_color,
      1));
  car_body_marker_pub_.publish(makeMeshMarker(
      car_header, car_pose, "package://coni_mpc/meshes/jackal-fender.stl",
      Eigen::Vector3d::Zero(), rear_fender_q, car_uniform_scale, fender_color,
      2));
  car_body_marker_pub_.publish(makeMeshMarker(
      car_header, car_pose, "package://coni_mpc/meshes/jackal-wheel.stl",
      scaled_front_left_wheel, wheel_q, car_uniform_scale, wheel_color, 3));
  car_body_marker_pub_.publish(makeMeshMarker(
      car_header, car_pose, "package://coni_mpc/meshes/jackal-wheel.stl",
      scaled_front_right_wheel, wheel_q, car_uniform_scale, wheel_color, 4));
  car_body_marker_pub_.publish(makeMeshMarker(
      car_header, car_pose, "package://coni_mpc/meshes/jackal-wheel.stl",
      scaled_rear_left_wheel, wheel_q, car_uniform_scale, wheel_color, 5));
  car_body_marker_pub_.publish(makeMeshMarker(
      car_header, car_pose, "package://coni_mpc/meshes/jackal-wheel.stl",
      scaled_rear_right_wheel, wheel_q, car_uniform_scale, wheel_color, 6));

  relative_est_pub_[quad_id].publish(relative_est_);
  
/*   relative_est_pub_.publish(relative_est_);
   */
   quad_odom_pub_[quad_id].publish(quad_odom_);
  (void)quad_odom_;
  geometry_msgs::PoseStamped quad_pose_stamped;
  quad_pose_stamped.header = quad_odom_.header;
  quad_pose_stamped.pose = quad_odom_.pose.pose;
  quad_path_.header.stamp = quad_pose_stamped.header.stamp;
  quad_path_.poses.push_back(quad_pose_stamped);
  // 不再裁剪路径，保持完整轨迹
  quad_path_pub_[quad_id].publish(quad_path_);
  // Publish a colored line strip marker matching the corresponding UAV body.
  if (!quad_path_.poses.empty()) {
    visualization_msgs::Marker path_marker;
    path_marker.header = quad_odom_.header;
    path_marker.header.frame_id = acado_mpc_common::worldFrameId();
    path_marker.ns = "quad_path_marker";
    path_marker.id = quad_id;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.03;
    const size_t n = quad_path_.poses.size();
    path_marker.points.reserve(n);
    path_marker.colors.reserve(n);
    for (size_t idx = 0; idx < n; ++idx) {
      const auto& pose = quad_path_.poses[idx];
      geometry_msgs::Point p;
      p.x = pose.pose.position.x;
      p.y = pose.pose.position.y;
      p.z = pose.pose.position.z;
      path_marker.points.push_back(p);
      path_marker.colors.push_back(quadPathColor(quad_id, 1.0));
    }
    quad_path_marker_pub_[quad_id].publish(path_marker);
  }

  visualization_msgs::Marker radius_marker;
  radius_marker.header = quad_odom_.header;
  radius_marker.header.frame_id = acado_mpc_common::worldFrameId();
  radius_marker.ns = "uav_radius";
  radius_marker.id = quad_id;
  radius_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  radius_marker.action = visualization_msgs::Marker::ADD;
  radius_marker.pose = quad_odom_.pose.pose;
  radius_marker.mesh_resource =
      "package://coni_mpc/meshes/hummingbird.dae";
  radius_marker.mesh_use_embedded_materials = false;
  constexpr double kUavMeshSpanM = 0.46221;
  const double uav_target_span = std::max(0.56, 6.0 * uav_radius_);
  const double uav_uniform_scale = uav_target_span / kUavMeshSpanM;
  radius_marker.scale.x = uav_uniform_scale;
  radius_marker.scale.y = uav_uniform_scale;
  radius_marker.scale.z = uav_uniform_scale;
  radius_marker.color = quadBodyColor(quad_id, 0.88);
  radius_marker.lifetime = ros::Duration(0.0);
  quad_radius_marker_pub_.publish(radius_marker);

  ObstacleVector obstacles;
  {
    std::lock_guard<std::mutex> lock(shared_state_mutex_);
    obstacles = shared_static_obstacles_;
  }
  std::vector<std::string> obstacle_keys;
  obstacle_keys.reserve(obstacles.size() + static_cast<std::size_t>(num_uavs));
  for (std::size_t idx = 0; idx < obstacles.size(); ++idx) {
    obstacle_keys.push_back("static:" + std::to_string(idx));
  }
  const double static_margin = uav_radius_ + mpc_params_.cbf_safety_margin_;
  for (auto& obstacle : obstacles) {
    obstacle(3) = obstacle(3) + static_margin;
  }
  const double effective_radius = 2.0 * uav_radius_ + mpc_params_.cbf_safety_margin_;
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Vector3d> velocities;
  std::vector<bool> valid;
  getSharedPositions(positions, velocities, valid);
  const size_t vehicle_count = positions.size();
  for (size_t idx = 0; idx < vehicle_count; ++idx) {
    if (idx == static_cast<size_t>(quad_id)) {
      continue;
    }
    if (idx >= valid.size() || idx >= velocities.size() || !valid[idx]) {
      continue;
    }
    Obstacle obstacle = Obstacle::Zero();
    obstacle << positions[idx].x(),
                positions[idx].y(),
                positions[idx].z(),  // z will be ignored later (treated as infinite cylinder)
                effective_radius,
                velocities[idx].x(),
                velocities[idx].y(),
                velocities[idx].z();
    obstacles.push_back(obstacle);
    obstacle_keys.push_back("uav:" + std::to_string(idx));
  }
  // 移除周期性障碍高度日志

  std::vector<double> h_values;
  std::vector<double> hdot_values;
  std::vector<double> hddot_values;
  std::vector<double> cbf_values;
  std::vector<double> active_h_values;
  std::vector<double> active_hdot_values;
  std::vector<double> active_hddot_values;
  std::vector<double> active_cbf_values;
  constexpr std::size_t kInvalidSolverActiveIdx =
      std::numeric_limits<std::size_t>::max();
  std::size_t solver_active_idx = kInvalidSolverActiveIdx;
  std::string active_obstacle_key = "none";
  std::string solver_active_obstacle_key = "none";
  Obstacle active_obstacle_world = Obstacle::Zero();
  Obstacle active_obstacle_solver = Obstacle::Zero();
  bool has_active_obstacle_world = false;
  bool has_active_obstacle_solver = false;

  ObstacleVector mpc_obstacles;
  ObstacleVector metric_obstacles;
  ObstacleVector all_solver_obstacles;
  ObstacleVector world_mpc_obstacles;
  std::vector<std::string> selected_keys;
  acado_mpc::MpcWrapper<double>::ObstacleProfileVector mpc_obstacle_profiles;
  RiskRegionVector mpc_risk_regions;
  RiskRegionProfileVector mpc_risk_region_profiles;
  bool use_mpc_risk_regions = false;
  if (risk_regions_enabled_) {
    coni_mpc::RiskRegionArray risk_message;
    bool have_message = false;
    {
      std::lock_guard<std::mutex> lock(risk_region_mutex_);
      if (has_latest_risk_regions_) {
        risk_message = latest_risk_regions_;
        have_message = true;
      }
    }
    const double risk_age_sec =
        have_message && !risk_message.header.stamp.isZero()
            ? std::max(0.0, (ros::Time::now() - risk_message.header.stamp).toSec())
            : 0.0;
    const double dt = std::max(1.0e-3, mpc_controller_.getPredictionDt());
    const double prediction_horizon_sec =
        dt * static_cast<double>(acado_mpc::kSamples);
    // The message contains a finite [stamp, stamp+T_p] forecast.  Once its
    // stamp is older than T_p, even the final DSP stage is in the past; using
    // that stage as a current obstacle creates a time-shifted barrier and can
    // either force a deadlock or miss the real surface.  After this cutoff the
    // risk-region HOCBF is disabled until a newly stamped DSP result arrives.
    const bool fresh = have_message &&
        (risk_message.header.stamp.isZero() ||
         (risk_age_sec <= std::max(0.0, risk_regions_timeout_) &&
          risk_age_sec <= prediction_horizon_sec + 1.0e-6));
    if (risk_regions_enabled_) {
      const double risk_age =
          have_message && !risk_message.header.stamp.isZero()
              ? risk_age_sec
              : std::numeric_limits<double>::infinity();
      ROS_WARN_STREAM_THROTTLE_NAMED(
          0.2, "risk_region_freshness",
          "[MPC UAV " << quad_id << "] risk_region_age=" << risk_age
                       << " timeout=" << risk_regions_timeout_
                       << " fresh=" << (fresh ? "true" : "false")
                       << " have_message=" << (have_message ? "true" : "false"));
    }
    if (fresh) {
      std::vector<std::uint32_t> track_ids;
      for (const auto& message_region : risk_message.regions) {
        if (!message_region.valid ||
            std::find(track_ids.begin(), track_ids.end(), message_region.track_id) ==
                track_ids.end()) {
          if (message_region.valid) track_ids.push_back(message_region.track_id);
        }
      }
      // A risk message describes stages relative to its own sensor stamp.
      // If construction took tau seconds, stage zero is already tau seconds
      // in the past.  Align the message to the current MPC horizon instead
      // of reusing that old stage as "now".  Clamping at the final DSP stage
      // is a conservative hold of the last predicted occupied region.
      // A message is generated at its header stamp.  When alignment is
      // enabled, stage k in the current MPC horizon consumes the source DSP
      // stage that is closest to "now + k*dt".  Disabling this parameter keeps
      // the original stage-0-at-now behavior for an A/B comparison.
      const int risk_stage_shift = risk_region_time_alignment_
          ? std::max(0, static_cast<int>(std::llround(risk_age_sec / dt)))
          : 0;
      auto carStateAt = [&](std::size_t stage) {
        if (stage < car_trajectory_window_.size()) {
          return car_trajectory_window_[stage];
        }
        return carStateFromOdom(car_odom_);
      };

      // Rank risk regions against the UAV trajectory that the controller is
      // expected to follow.  Prefer the previous successful MPC prediction;
      // before the first solve, fall back to the current reference window and
      // finally to the current estimated position.  The resulting points are
      // expressed in the same (possibly non-inertial) solver frame as the
      // risk-region profiles below.
      std::vector<Eigen::Vector3d> uav_predicted_solver(
          static_cast<std::size_t>(acado_mpc::kSamples + 1),
          state_estimate_.position);
      if (has_last_predicted_world_positions_ &&
          last_predicted_world_positions_.size() == uav_predicted_solver.size()) {
        for (std::size_t stage = 0; stage < uav_predicted_solver.size(); ++stage) {
          const CarState_t car_state = carStateAt(stage);
          if (isInertialFrame()) {
            uav_predicted_solver[stage] = last_predicted_world_positions_[stage];
          } else {
            Eigen::Quaterniond world_q_non(car_state(6), car_state(7),
                                            car_state(8), car_state(9));
            world_q_non.normalize();
            const Eigen::Vector3d car_position(car_state(0), car_state(1),
                                               car_state(2));
            uav_predicted_solver[stage] =
                world_q_non.inverse() *
                (last_predicted_world_positions_[stage] - car_position);
          }
        }
      } else if (!reference_window_.points.empty()) {
        const std::size_t last_ref_idx = reference_window_.points.size() - 1;
        for (std::size_t stage = 0; stage < uav_predicted_solver.size(); ++stage) {
          uav_predicted_solver[stage] =
              reference_window_.points[std::min(stage, last_ref_idx)].position;
        }
      }

      struct RankedRiskRegion {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::uint32_t track_id = 0;
        RiskRegionProfile profile = RiskRegionProfile::Zero();
        double trajectory_margin = std::numeric_limits<double>::infinity();
      };
      std::vector<RankedRiskRegion,
                  Eigen::aligned_allocator<RankedRiskRegion>> ranked_regions;
      ranked_regions.reserve(track_ids.size());

      for (const std::uint32_t track_id : track_ids) {
        RiskRegionProfile profile = RiskRegionProfile::Zero();
        std::vector<Eigen::Vector3d> centers(acado_mpc::kSamples + 1,
                                             Eigen::Vector3d::Zero());
        std::vector<bool> active(acado_mpc::kSamples + 1, false);
        for (int stage = 0; stage <= acado_mpc::kSamples; ++stage) {
          const int source_stage = std::min(
              acado_mpc::kSamples, stage + risk_stage_shift);
          const coni_mpc::RiskRegion* selected = nullptr;
          for (const auto& message_region : risk_message.regions) {
            if (message_region.valid && message_region.track_id == track_id &&
                message_region.stage_index ==
                    static_cast<std::uint32_t>(source_stage)) {
              selected = &message_region;
              break;
            }
          }
          if (selected == nullptr) continue;
          const CarState_t car_state = carStateAt(static_cast<std::size_t>(stage));
          Eigen::Quaterniond world_q_non(car_state(6), car_state(7), car_state(8),
                                         car_state(9));
          world_q_non.normalize();
          const Eigen::Matrix3d R_nw =
              isInertialFrame() ? Eigen::Matrix3d::Identity()
                                : world_q_non.inverse().toRotationMatrix();
          const Eigen::Vector3d car_position(car_state(0), car_state(1), car_state(2));
          const Eigen::Vector3d world_center(selected->center.x,
                                              selected->center.y,
                                              selected->center.z);
          centers[stage] = isInertialFrame() ? world_center
                                             : R_nw * (world_center - car_position);
          Eigen::Matrix3d Q_world(
              Eigen::Quaterniond(selected->orientation.w,
                                 selected->orientation.x,
                                 selected->orientation.y,
                                 selected->orientation.z));
          const Eigen::Matrix3d Q_solver =
              isInertialFrame() ? Q_world : R_nw * Q_world;
          profile(0, stage) = centers[stage].x();
          profile(1, stage) = centers[stage].y();
          profile(2, stage) = centers[stage].z();
          profile(3, stage) = std::max(1.0e-3, static_cast<double>(selected->semi_axes.x));
          profile(4, stage) = std::max(1.0e-3, static_cast<double>(selected->semi_axes.y));
          profile(5, stage) = std::max(1.0e-3, static_cast<double>(selected->semi_axes.z));
          for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
              profile(6 + 3 * row + col, stage) = Q_solver(row, col);
            }
          }
          active[stage] = true;
          profile(21, stage) = 1.0;
        }
        for (int stage = 0; stage <= acado_mpc::kSamples; ++stage) {
          if (!active[stage]) continue;
          const int previous = std::max(0, stage - 1);
          const int next = std::min(acado_mpc::kSamples, stage + 1);
          profile(15, stage) =
              (centers[next].x() - centers[previous].x()) /
              (std::max(1, next - previous) * dt);
          profile(16, stage) =
              (centers[next].y() - centers[previous].y()) /
              (std::max(1, next - previous) * dt);
          profile(17, stage) =
              (centers[next].z() - centers[previous].z()) /
              (std::max(1, next - previous) * dt);
          if (stage > 0 && stage < acado_mpc::kSamples && active[previous] && active[next]) {
            profile(18, stage) = (centers[next].x() - 2.0 * centers[stage].x() +
                                  centers[previous].x()) / (dt * dt);
            profile(19, stage) = (centers[next].y() - 2.0 * centers[stage].y() +
                                  centers[previous].y()) / (dt * dt);
            profile(20, stage) = (centers[next].z() - 2.0 * centers[stage].z() +
                                  centers[previous].z()) / (dt * dt);
          }
        }
        if (active[0]) {
          // rho_j = min_k [g_j,k(p_uav,k) - 1].  Smaller values mean that
          // the predicted UAV trajectory is closer to, or already inside,
          // that risk region.  This is the ranking rule specified by the
          // risk-region design instead of sorting by arbitrary track ID.
          double trajectory_margin = std::numeric_limits<double>::infinity();
          for (int stage = 0; stage <= acado_mpc::kSamples; ++stage) {
            if (!active[stage]) continue;
            const Eigen::Vector3d center(profile(0, stage), profile(1, stage),
                                         profile(2, stage));
            const Eigen::Vector3d axes(
                std::max(1.0e-6, static_cast<double>(profile(3, stage))),
                std::max(1.0e-6, static_cast<double>(profile(4, stage))),
                std::max(1.0e-6, static_cast<double>(profile(5, stage))));
            Eigen::Matrix3d orientation;
            for (int row = 0; row < 3; ++row) {
              for (int col = 0; col < 3; ++col) {
                orientation(row, col) = profile(6 + 3 * row + col, stage);
              }
            }
            const Eigen::Vector3d local =
                orientation.transpose() * (uav_predicted_solver[stage] - center);
            const double g = std::pow(local.x() / axes.x(), 4.0) +
                             std::pow(local.y() / axes.y(), 4.0) +
                             std::pow(local.z() / axes.z(), 4.0);
            trajectory_margin = std::min(trajectory_margin, g - 1.0);
          }

          RankedRiskRegion ranked;
          ranked.track_id = track_id;
          ranked.profile = profile;
          ranked.trajectory_margin = trajectory_margin;
          ranked_regions.push_back(ranked);
        }
      }

      std::sort(ranked_regions.begin(), ranked_regions.end(),
                [](const RankedRiskRegion& lhs, const RankedRiskRegion& rhs) {
                  if (lhs.trajectory_margin != rhs.trajectory_margin) {
                    return lhs.trajectory_margin < rhs.trajectory_margin;
                  }
                  return lhs.track_id < rhs.track_id;
                });
      const std::size_t max_tracks = std::min<std::size_t>(
          ranked_regions.size(), static_cast<std::size_t>(acado_mpc::kMaxObstacles));
      for (std::size_t track_index = 0; track_index < max_tracks; ++track_index) {
        mpc_risk_regions.push_back(ranked_regions[track_index].profile.col(0));
        mpc_risk_region_profiles.push_back(ranked_regions[track_index].profile);
      }
      use_mpc_risk_regions = !mpc_risk_regions.empty();
      if (use_mpc_risk_regions) {
          ROS_INFO_STREAM_THROTTLE(1.0, "[MPC UAV " << quad_id
                                 << "] using " << mpc_risk_regions.size()
                                 << " DSP risk-region tracks ranked by predicted UAV trajectory"
                                 << " age=" << risk_age_sec
                                 << " time_alignment="
                                 << (risk_region_time_alignment_ ? "on" : "off")
                                 << " stage_shift=" << risk_stage_shift);
      }
    }
  }
  if (!obstacles.empty() && !use_mpc_risk_regions) {
    const Eigen::Vector3d uav_position_world(
        quad_odom_.pose.pose.position.x,
        quad_odom_.pose.pose.position.y,
        quad_odom_.pose.pose.position.z);
    const Eigen::Vector3d uav_position_solver = state_estimate_.position;
    const CarState_t current_car_state = carStateFromOdom(car_odom_);
    std::size_t nearest_any_idx = obstacles.size();
    double nearest_any_dist_sq = std::numeric_limits<double>::infinity();
    for (std::size_t obs_idx = 0; obs_idx < obstacles.size(); ++obs_idx) {
      const auto& obstacle = obstacles[obs_idx];
      const Eigen::Vector2d diff_xy =
          obstacle.head<2>() - uav_position_world.head<2>();
      const double dist_sq = diff_xy.squaredNorm();
      if (dist_sq < nearest_any_dist_sq) {
        nearest_any_dist_sq = dist_sq;
        nearest_any_idx = obs_idx;
      }
    }
    ObstacleVector world_metric_obstacles;
    double preview_margin = 0.6;
    pnh_.param("cbf/preview_margin", preview_margin, 0.6);
    preview_margin = std::max(0.0, preview_margin);
    int preview_steps = 3;
    pnh_.param("cbf/preview_steps", preview_steps, 3);
    preview_steps = std::max(0, preview_steps);

    const std::size_t horizon_steps =
        static_cast<std::size_t>(acado_mpc::kSamples + 1);
    const std::size_t preview_horizon_steps =
        std::min<std::size_t>(horizon_steps,
                              static_cast<std::size_t>(preview_steps + 1));
    const double prediction_dt = mpc_controller_.getPredictionDt();
    auto previewPositionAtSolverStep = [&](const Eigen::Vector3d& world_position,
                                           std::size_t k) {
      if (isInertialFrame()) {
        return world_position;
      }
      const CarState_t& car_state_k =
          (k < car_trajectory_window_.size()) ? car_trajectory_window_[k]
                                              : current_car_state;
      Eigen::Quaterniond W_q_non(
          car_state_k(6), car_state_k(7), car_state_k(8), car_state_k(9));
      W_q_non.normalize();
      const Eigen::Vector3d car_position(
          car_state_k(0), car_state_k(1), car_state_k(2));
      Eigen::Vector3d relative_position =
          W_q_non.inverse() * (world_position - car_position);
      relative_position.z() = uav_position_solver.z();
      return relative_position;
    };
    std::vector<Eigen::Vector3d> uav_preview_positions(
        horizon_steps, uav_position_solver);
    if (has_last_predicted_world_positions_ &&
        last_predicted_world_positions_.size() == horizon_steps) {
      for (std::size_t k = 0; k < horizon_steps; ++k) {
        uav_preview_positions[k] =
            previewPositionAtSolverStep(last_predicted_world_positions_[k], k);
      }
    } else if (!reference_window_.points.empty()) {
      // Before a valid MPC prediction is available, preview risk falls back to
      // the reference window.
      const std::size_t last_ref_idx = reference_window_.points.size() - 1;
      for (std::size_t k = 1; k < horizon_steps; ++k) {
        const std::size_t ref_idx = std::min(k, last_ref_idx);
        uav_preview_positions[k] = reference_window_.points[ref_idx].position;
      }
    }

    auto obstacleInWorldAtStep = [&](const Obstacle& world_obstacle,
                                     std::size_t k) {
      Obstacle obstacle_k = world_obstacle;
      // The obstacle source provides position and velocity at the current step.
      // Use a constant-velocity rollout across the MPC horizon.
      const double dt_k = prediction_dt * static_cast<double>(k);
      obstacle_k(0) += obstacle_k(4) * dt_k;
      obstacle_k(1) += obstacle_k(5) * dt_k;
      obstacle_k(2) += obstacle_k(6) * dt_k;
      return obstacle_k;
    };
    auto obstacleAtSolverStep = [&](const Obstacle& world_obstacle,
                                    std::size_t k) {
      const Obstacle world_obstacle_k = obstacleInWorldAtStep(world_obstacle, k);
      if (isInertialFrame()) {
        Obstacle solver_obstacle = world_obstacle_k;
        solver_obstacle(2) = uav_preview_positions[k].z();
        return solver_obstacle;
      }
      const CarState_t& car_state_k =
          (k < car_trajectory_window_.size()) ? car_trajectory_window_[k]
                                              : current_car_state;
      return obstacleInNonInertialFrame(
          world_obstacle_k, car_state_k, uav_preview_positions[k].z());
    };
    auto planarDistanceSq = [&](const Eigen::Vector3d& uav_position,
                                const Obstacle& obstacle_solver) {
      return (uav_position.head<2>() - obstacle_solver.head<2>()).squaredNorm();
    };
    auto planarMargin = [&](const Eigen::Vector3d& uav_position,
                            const Obstacle& obstacle_solver) {
      const double radius = obstacle_solver(3);
      return planarDistanceSq(uav_position, obstacle_solver) - radius * radius;
    };

    struct ObstacleRiskInfo {
      std::size_t index = std::numeric_limits<std::size_t>::max();
      std::string stage = "none";
      double risk = std::numeric_limits<double>::quiet_NaN();
      int preview_hit_step = -1;
    };

    int preview_candidates = 0;
    std::vector<ObstacleRiskInfo> candidate_obstacles;
    candidate_obstacles.reserve(obstacles.size());
    for (std::size_t obs_idx = 0; obs_idx < obstacles.size(); ++obs_idx) {
      ObstacleRiskInfo risk_info;
      risk_info.index = obs_idx;

      double obstacle_risk = std::numeric_limits<double>::infinity();
      int preview_hit_step = -1;
      for (std::size_t k = 1; k < preview_horizon_steps; ++k) {
        const auto obstacle_solver = obstacleAtSolverStep(obstacles[obs_idx], k);
        const double dist_sq =
            planarDistanceSq(uav_preview_positions[k], obstacle_solver);
        const double margin =
            planarMargin(uav_preview_positions[k], obstacle_solver);
        obstacle_risk = std::min(obstacle_risk, margin);
        const double preview_radius = obstacle_solver(3) + preview_margin;
        if (preview_hit_step < 0 && dist_sq <= preview_radius * preview_radius) {
          preview_hit_step = static_cast<int>(k);
        }
      }
      if (preview_hit_step < 0) {
        continue;
      }
      ++preview_candidates;
      risk_info.stage = "preview-risk";
      risk_info.risk = obstacle_risk;
      risk_info.preview_hit_step = preview_hit_step;
      candidate_obstacles.push_back(risk_info);
    }

    std::sort(candidate_obstacles.begin(), candidate_obstacles.end(),
              [](const ObstacleRiskInfo& lhs, const ObstacleRiskInfo& rhs) {
                if (lhs.risk != rhs.risk) {
                  return lhs.risk < rhs.risk;
                }
                if (lhs.preview_hit_step != rhs.preview_hit_step) {
                  if (lhs.preview_hit_step < 0) {
                    return false;
                  }
                  if (rhs.preview_hit_step < 0) {
                    return true;
                  }
                  return lhs.preview_hit_step < rhs.preview_hit_step;
                }
                return lhs.index < rhs.index;
              });

    const auto sameObstacleOrdering =
        [](const std::vector<std::string>& lhs,
           const std::vector<std::string>& rhs) {
          if (lhs.size() != rhs.size()) {
            return false;
          }
          for (std::size_t idx = 0; idx < lhs.size(); ++idx) {
            if (lhs[idx] != rhs[idx]) {
              return false;
            }
          }
          return true;
        };
    auto selectionKeys =
        [&](const std::vector<ObstacleRiskInfo>& selected) {
          std::vector<std::string> keys;
          keys.reserve(selected.size());
          for (const auto& selected_obstacle : selected) {
            keys.push_back(obstacle_keys[selected_obstacle.index]);
          }
          return keys;
        };
    auto appendUniqueByRisk =
        [&](std::vector<ObstacleRiskInfo>& output,
            const std::vector<ObstacleRiskInfo>& ranked_candidates) {
          for (const auto& candidate : ranked_candidates) {
            if (output.size() >=
                static_cast<std::size_t>(acado_mpc::kMaxObstacles)) {
              break;
            }
            bool already_selected = false;
            for (const auto& selected : output) {
              if (selected.index == candidate.index) {
                already_selected = true;
                break;
              }
            }
            if (already_selected) {
              continue;
            }
            output.push_back(candidate);
          }
        };
    std::vector<ObstacleRiskInfo> sticky_selected_obstacles;
    sticky_selected_obstacles.reserve(
        std::min<std::size_t>(last_selected_obstacle_keys_.size(),
                              static_cast<std::size_t>(acado_mpc::kMaxObstacles)));
    for (const auto& sticky_key : last_selected_obstacle_keys_) {
      for (const auto& candidate : candidate_obstacles) {
        if (obstacle_keys[candidate.index] == sticky_key) {
          sticky_selected_obstacles.push_back(candidate);
          if (sticky_selected_obstacles.size() >=
              static_cast<std::size_t>(acado_mpc::kMaxObstacles)) {
            break;
          }
          break;
        }
      }
      if (sticky_selected_obstacles.size() >=
          static_cast<std::size_t>(acado_mpc::kMaxObstacles)) {
        break;
      }
    }
    std::sort(sticky_selected_obstacles.begin(), sticky_selected_obstacles.end(),
              [](const ObstacleRiskInfo& lhs, const ObstacleRiskInfo& rhs) {
                if (lhs.risk != rhs.risk) {
                  return lhs.risk < rhs.risk;
                }
                if (lhs.preview_hit_step != rhs.preview_hit_step) {
                  if (lhs.preview_hit_step < 0) {
                    return false;
                  }
                  if (rhs.preview_hit_step < 0) {
                    return true;
                  }
                  return lhs.preview_hit_step < rhs.preview_hit_step;
                }
                return lhs.index < rhs.index;
              });
    std::vector<ObstacleRiskInfo> selected_obstacles;
    const std::size_t selected_count =
        std::min<std::size_t>(candidate_obstacles.size(),
                              static_cast<std::size_t>(acado_mpc::kMaxObstacles));
    const std::vector<std::string> fresh_selected_keys = [&]() {
      std::vector<std::string> keys;
      keys.reserve(selected_count);
      for (std::size_t idx = 0; idx < selected_count; ++idx) {
        keys.push_back(obstacle_keys[candidate_obstacles[idx].index]);
      }
      return keys;
    }();
    bool keep_sticky_selection = false;
    if (!sticky_selected_obstacles.empty()) {
      const bool within_hold_window =
          obstacle_hysteresis_cycles_since_switch_ <
          obstacle_hysteresis_hold_cycles_;
      if (within_hold_window) {
        keep_sticky_selection = true;
      } else {
        const std::vector<std::string> sticky_keys =
            selectionKeys(sticky_selected_obstacles);
        if (!sameObstacleOrdering(sticky_keys, fresh_selected_keys) &&
            !candidate_obstacles.empty()) {
          const double sticky_best_risk = sticky_selected_obstacles.front().risk;
          const double fresh_best_risk = candidate_obstacles.front().risk;
          keep_sticky_selection =
              (fresh_best_risk + obstacle_hysteresis_min_risk_improvement_ >=
               sticky_best_risk);
        }
      }
    }
    if (keep_sticky_selection) {
      selected_obstacles = sticky_selected_obstacles;
      appendUniqueByRisk(selected_obstacles, candidate_obstacles);
    } else {
      selected_obstacles.reserve(selected_count);
      for (std::size_t idx = 0; idx < selected_count; ++idx) {
        selected_obstacles.push_back(candidate_obstacles[idx]);
      }
    }
    if (selected_obstacles.size() >
        static_cast<std::size_t>(acado_mpc::kMaxObstacles)) {
      selected_obstacles.resize(static_cast<std::size_t>(acado_mpc::kMaxObstacles));
    }
    selected_keys = selectionKeys(selected_obstacles);
    if (sameObstacleOrdering(selected_keys, last_selected_obstacle_keys_)) {
      obstacle_hysteresis_cycles_since_switch_ += 1;
    } else {
      obstacle_hysteresis_cycles_since_switch_ = 0;
    }
    last_selected_obstacle_keys_ = selected_keys;
    for (const auto& selected : selected_obstacles) {
      world_mpc_obstacles.push_back(obstacles[selected.index]);
    }

    if (!selected_obstacles.empty()) {
      std::ostringstream active_key_stream;
      for (std::size_t idx = 0; idx < selected_obstacles.size(); ++idx) {
        if (idx > 0) {
          active_key_stream << "|";
        }
        active_key_stream << obstacle_keys[selected_obstacles[idx].index];
      }
      active_obstacle_key = active_key_stream.str();
    }
    // Diagnostic-only barrier stats stay on the nearest obstacle; solver-side
    // Metrics are evaluated separately on the selected obstacles passed to the
    // controller backend.
    if (nearest_any_idx < obstacles.size()) {
      world_metric_obstacles.push_back(obstacles[nearest_any_idx]);
    }

    auto enforcePlanarCbfGeometry = [&](ObstacleVector& obstacles_to_adjust) {
      for (auto& obstacle : obstacles_to_adjust) {
        // Keep solver CBF geometry consistent with planar (xy) collision checks.
        obstacle(2) = state_estimate_.position.z();
      }
    };
    all_solver_obstacles.reserve(obstacles.size());
    for (const auto& obstacle_world : obstacles) {
      all_solver_obstacles.push_back(obstacleAtSolverStep(obstacle_world, 0));
    }
    if (!all_solver_obstacles.empty()) {
      enforcePlanarCbfGeometry(all_solver_obstacles);
    }
    if (!world_mpc_obstacles.empty()) {
      mpc_obstacles.reserve(world_mpc_obstacles.size());
      mpc_obstacle_profiles.reserve(world_mpc_obstacles.size());
      for (const auto& world_obstacle : world_mpc_obstacles) {
        acado_mpc::MpcWrapper<double>::ObstacleProfile profile;
        for (int k = 0; k < acado_mpc::kSamples + 1; ++k) {
          profile.col(k) =
              obstacleAtSolverStep(world_obstacle, static_cast<std::size_t>(k));
        }
        mpc_obstacles.push_back(profile.col(0).eval());
        mpc_obstacle_profiles.push_back(profile);
      }
    }
    if (!world_metric_obstacles.empty()) {
      metric_obstacles.push_back(
          obstacleAtSolverStep(world_metric_obstacles.front(), 0));
    }
    if (!mpc_obstacles.empty()) {
      enforcePlanarCbfGeometry(mpc_obstacles);
    }
    if (!metric_obstacles.empty()) {
      enforcePlanarCbfGeometry(metric_obstacles);
    }

    const Eigen::Vector2d acc_xy =
        (state_estimate_.a_imu - Eigen::Vector3d(0.0, 0.0, 9.81)).head<2>();
    const Eigen::Vector2d vel_xy = state_estimate_.velocity.head<2>();
    const Eigen::Vector2d pos_xy = state_estimate_.position.head<2>();
    auto evaluateObstacleSet = [&](const ObstacleVector& obstacles_to_evaluate,
                                   std::vector<double>& h_out,
                                   std::vector<double>& hdot_out,
                                   std::vector<double>& hddot_out,
                                   std::vector<double>& cbf_out,
                                   bool update_dmin) {
      h_out.reserve(obstacles_to_evaluate.size());
      hdot_out.reserve(obstacles_to_evaluate.size());
      hddot_out.reserve(obstacles_to_evaluate.size());
      cbf_out.reserve(obstacles_to_evaluate.size());
      for (const auto& obstacle : obstacles_to_evaluate) {
        const Eigen::Vector2d obstacle_xy(obstacle(0), obstacle(1));
        const Eigen::Vector2d obstacle_vel_xy(obstacle(4), obstacle(5));
        const double radius = obstacle(3);
        const Eigen::Vector2d rel_pos_xy = pos_xy - obstacle_xy;
        const Eigen::Vector2d rel_vel_xy = vel_xy - obstacle_vel_xy;
        const double h = rel_pos_xy.squaredNorm() - radius * radius;
        const double hdot = 2.0 * rel_pos_xy.dot(rel_vel_xy);
        // Debug-only quantity: obstacle acceleration is not modeled here.
        const double hddot = 2.0 * rel_vel_xy.squaredNorm() +
                             2.0 * rel_pos_xy.dot(acc_xy);
        const double cbf_value =
            hddot +
            (mpc_params_.cbf_alpha1_ + mpc_params_.cbf_alpha2_) * hdot +
            (mpc_params_.cbf_alpha1_ * mpc_params_.cbf_alpha2_) * h;
        h_out.push_back(h);
        hdot_out.push_back(hdot);
        hddot_out.push_back(hddot);
        cbf_out.push_back(cbf_value);
        if (update_dmin) {
          const double d_clear = rel_pos_xy.norm() - radius;
          metrics_dmin_min_ = std::min(metrics_dmin_min_, d_clear);
        }
      }
    };

    if (!mpc_obstacles.empty()) {
      evaluateObstacleSet(mpc_obstacles,
                          active_h_values,
                          active_hdot_values,
                          active_hddot_values,
                          active_cbf_values,
                          false);
      if (active_h_values.size() == mpc_obstacles.size() &&
          active_h_values.size() == selected_keys.size() &&
          active_h_values.size() == world_mpc_obstacles.size()) {
        const auto min_h_it =
            std::min_element(active_h_values.begin(), active_h_values.end());
        solver_active_idx =
            static_cast<std::size_t>(std::distance(active_h_values.begin(),
                                                   min_h_it));
        solver_active_obstacle_key = selected_keys[solver_active_idx];
        active_obstacle_solver = mpc_obstacles[solver_active_idx];
        active_obstacle_world = world_mpc_obstacles[solver_active_idx];
        has_active_obstacle_solver = true;
        has_active_obstacle_world = true;
      } else {
        ROS_WARN_STREAM_THROTTLE(
            1.0,
            "[" << pnh_.getNamespace()
                << "] Unable to bind solver_active_obstacle_key to a unique "
                   "solver obstacle state; skipping clean obstacle-bound "
                   "metrics for this step.");
      }
    }
    if (!selected_obstacles.empty()) {
      const std::string selection_log_name =
          "cbf_obstacle_select_uav_" + std::to_string(quad_id);
      std::ostringstream obstacle_selection_log;
      obstacle_selection_log << "[UAV " << quad_id << "]"
                             << " [" << pnh_.getNamespace()
                             << "] cbf_obstacle_select"
                             << " keys=" << active_obstacle_key
                             << " selected=" << selected_obstacles.size()
                             << " solver_limit=" << acado_mpc::kMaxObstacles
                             << " preview_steps=" << preview_steps
                             << " source="
                             << (keep_sticky_selection ? "sticky" : "fresh")
                             << " hold_cycles_since_switch="
                             << obstacle_hysteresis_cycles_since_switch_
                             << " preview_candidates=" << preview_candidates;
      for (std::size_t idx = 0; idx < selected_obstacles.size(); ++idx) {
        const auto& selected = selected_obstacles[idx];
        obstacle_selection_log
            << " | [" << idx << "] key=" << obstacle_keys[selected.index]
            << " stage=" << selected.stage
            << " risk=" << formatScalar(selected.risk)
            << " preview_hit_k=" << selected.preview_hit_step;
      }
      ROS_INFO_STREAM_THROTTLE_NAMED(
          1.0, selection_log_name, obstacle_selection_log.str());
    }

    if (!metric_obstacles.empty()) {
      evaluateObstacleSet(metric_obstacles,
                          h_values,
                          hdot_values,
                          hddot_values,
                          cbf_values,
                          true);

      double step_mean_h = 0.0;
      double step_mean_cbf = 0.0;
      for (size_t obs_idx = 0; obs_idx < h_values.size(); ++obs_idx) {
        const double h_val = h_values[obs_idx];
        const double cbf_val = cbf_values[obs_idx];
        metrics_diag_min_h_any_ = std::min(metrics_diag_min_h_any_, h_val);
        metrics_diag_min_cbf_any_ =
            std::min(metrics_diag_min_cbf_any_, cbf_val);
        step_mean_h += h_val;
        step_mean_cbf += cbf_val;
      }
      if (!h_values.empty()) {
        step_mean_h /= static_cast<double>(h_values.size());
        step_mean_cbf /= static_cast<double>(h_values.size());
        metrics_diag_sum_h_any_ += step_mean_h;
        metrics_diag_sum_cbf_any_ += step_mean_cbf;
      }
    }
  }

  // Effective CBF switch in simulation: require both static config and sim toggle.
  bool cbf_enabled_param = mpc_params_.cbf_enabled_;
  pnh_.param("cbf/enabled", cbf_enabled_param, cbf_enabled_param);
  const bool enable_cbf = cbf_enabled_param && cbf_use_in_sim_;
  const bool cbf_active = enable_cbf &&
      (use_mpc_risk_regions ? !mpc_risk_regions.empty() : !mpc_obstacles.empty());
  const std::string effective_active_obstacle_key =
      cbf_active ? active_obstacle_key : "none";
  if (cbf_active && !mpc_obstacle_profiles.empty()) {
    last_solver_obstacle_profile_snapshot_.valid = true;
    last_solver_obstacle_profile_snapshot_.keys = selected_keys;
    last_solver_obstacle_profile_snapshot_.world_obstacles = world_mpc_obstacles;
    last_solver_obstacle_profile_snapshot_.solver_profiles = mpc_obstacle_profiles;
  }
  mpc_params_.cbf_obstacles_ = mpc_obstacles;
  mpc_params_.cbf_obstacle_profiles_ = mpc_obstacle_profiles;
  mpc_params_.cbf_risk_regions_ = mpc_risk_regions;
  mpc_params_.cbf_risk_region_profiles_ = mpc_risk_region_profiles;
  mpc_params_.cbf_use_risk_regions_ = use_mpc_risk_regions;
  mpc_params_.cbf_enabled_ = enable_cbf;
  mpc_params_.changed_ = false;
  last_obstacles_ = all_solver_obstacles;

  Eigen::Quaterniond world_q_model = Eigen::Quaterniond::Identity();
  Eigen::Vector3d world_p_model = Eigen::Vector3d::Zero();
  if (!isInertialFrame()) {
    world_q_model = getCarOrientation();
    world_q_model.normalize();
    world_p_model = getCarPosition();
  }
  if (!reference_window_.points.empty()) {
    const auto& ref0 = reference_window_.points.front();
    ROS_INFO_STREAM_THROTTLE(
        1.0,
        "[MPC UAV " << quad_id << "] est_p=(" << state_estimate_.position.x()
                     << ", " << state_estimate_.position.y() << ", "
                     << state_estimate_.position.z() << ") est_v=("
                     << state_estimate_.velocity.x() << ", "
                     << state_estimate_.velocity.y() << ", "
                     << state_estimate_.velocity.z() << ") ref_p=("
                     << ref0.position.x() << ", " << ref0.position.y() << ", "
                     << ref0.position.z() << ") ref_v=("
                     << ref0.velocity.x() << ", " << ref0.velocity.y() << ", "
                     << ref0.velocity.z() << ")");
  }
  NonInertialProfile omega_non_profile = NonInertialProfile::Zero();
  NonInertialProfile beta_non_profile = NonInertialProfile::Zero();
  NonInertialProfile a_car_non_profile = NonInertialProfile::Zero();
  buildNonInertialDataProfile(omega_non_profile,
                              beta_non_profile,
                              a_car_non_profile);
  std::shared_ptr<const FieldSnapshot> field_snapshot;
  {
    std::lock_guard<std::mutex> lock(field_snapshot_mutex_);
    field_snapshot = field_snapshot_;
  }
  const ros::Time field_now = ros::Time::now();
  const double field_dt = std::max(1.0e-3, mpc_controller_.getPredictionDt());
  const double field_age = field_snapshot && !field_snapshot->stamp.isZero()
      ? std::max(0.0, (field_now - field_snapshot->stamp).toSec()) : 0.0;
  const bool field_available = field_hocbf_enabled_ && field_snapshot &&
      !field_snapshot->stamp.isZero() &&
      !field_snapshot->points.empty();
  if (field_available && field_age > field_dt * acado_mpc::kSamples) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[MPC UAV " << quad_id
        << "] DSP prediction age=" << field_age
        << " s exceeds its horizon; extrapolating the last voxel stage");
  }
  using FieldNominalPrediction =
      Eigen::Matrix<double, acado_mpc::kStateSize, acado_mpc::kSamples + 1>;
  const auto buildFieldProfile =
      [&](const FieldNominalPrediction* nominal_prediction,
          bool& profile_active_out) {
        FieldHocbfProfile profile = FieldHocbfProfile::Zero();
        profile_active_out = false;
        if (!field_available) return profile;

        // If the last published DSP stage is empty, reuse its latest occupied
        // stage after the prediction horizon rather than losing all barriers.
        const int source_horizon = field_age > field_dt * acado_mpc::kSamples
            ? static_cast<int>(field_snapshot->last_occupied_stage)
            : acado_mpc::kSamples;
        const int source_shift = std::max(
            0, static_cast<int>(std::min(
                static_cast<double>(source_horizon),
                std::floor(field_age / field_dt + 0.5))));
        auto carStateAt = [&](std::size_t stage) {
          return stage < car_trajectory_window_.size()
              ? car_trajectory_window_[stage] : carStateFromOdom(car_odom_);
        };
        for (int stage = 0; stage <= acado_mpc::kSamples; ++stage) {
          const int source_stage = std::min(
              source_horizon, stage + source_shift);
          // Once the DSP horizon is exhausted, retain its final voxel slice
          // and project it to this MPC stage using the published velocity.
          const double extrapolation_time =
              field_age + static_cast<double>(stage - source_stage) * field_dt;
          const CarState_t car_state =
              carStateAt(static_cast<std::size_t>(stage));
          Eigen::Quaterniond world_q_non(car_state(6), car_state(7),
                                          car_state(8), car_state(9));
          if (world_q_non.norm() > 1e-9) world_q_non.normalize();
          else world_q_non = Eigen::Quaterniond::Identity();
          const Eigen::Matrix3d R_nw = world_q_non.inverse().toRotationMatrix();
          const Eigen::Vector3d car_position = car_state.segment<3>(0);
          const Eigen::Vector3d car_velocity = car_state.segment<3>(3);
          const Eigen::Vector3d omega_non = omega_non_profile.col(stage);
          const Eigen::Vector3d beta_non = beta_non_profile.col(stage);
          const Eigen::Vector3d a_car_non = a_car_non_profile.col(stage);

          Eigen::Vector3d nominal_position = state_estimate_.position;
          Eigen::Vector3d nominal_velocity = state_estimate_.velocity;
          if (nominal_prediction != nullptr &&
              nominal_prediction->allFinite()) {
            // The second micro-iteration uses the rollout produced by the
            // first QP in this same control cycle.
            nominal_position = nominal_prediction->block<3, 1>(
                acado_mpc::kPosX, stage);
            nominal_velocity = nominal_prediction->block<3, 1>(
                acado_mpc::kVelX, stage);
          } else if (has_last_predicted_world_positions_ &&
                     last_predicted_world_positions_.size() ==
                         static_cast<std::size_t>(acado_mpc::kSamples + 1)) {
            // First iteration: use the previous cycle's predicted horizon.
            const auto& last_prediction = mpc_controller_.getPredictedStates();
            nominal_position = last_prediction.block<3, 1>(
                acado_mpc::kPosX, stage);
            nominal_velocity = last_prediction.block<3, 1>(
                acado_mpc::kVelX, stage);
          } else {
            // Startup fallback remains stage-wise and does not reuse p0 at
            // every stage.
            nominal_position = state_estimate_.position +
                static_cast<double>(stage) * field_dt * state_estimate_.velocity;
            nominal_velocity = state_estimate_.velocity;
          }

          field_hocbf::KinematicPoints candidates;
          for (const auto& point : field_snapshot->points) {
            if (point.stage_index != static_cast<std::uint32_t>(source_stage)) {
              continue;
            }
            const Eigen::Vector3d world_position =
                point.position_world + point.velocity_world * extrapolation_time;
            field_hocbf::KinematicPoint transformed;
            transformed.occupancy = point.occupancy;
            if (isInertialFrame()) {
              transformed.position = world_position;
              transformed.velocity = point.velocity_world;
              transformed.acceleration.setZero();
            } else {
              transformed.position = R_nw * (world_position - car_position);
              transformed.velocity =
                  R_nw * (point.velocity_world - car_velocity) -
                  omega_non.cross(transformed.position);
              transformed.acceleration =
                  -a_car_non - 2.0 * omega_non.cross(transformed.velocity) -
                  beta_non.cross(transformed.position) -
                  omega_non.cross(omega_non.cross(transformed.position));
            }
            candidates.push_back(transformed);
          }
          const auto selected =
              field_hocbf::prune(candidates, nominal_position);
          Eigen::Vector3d nominal_a_non = Eigen::Vector3d::Zero();
          if (!isInertialFrame()) {
            nominal_a_non =
                -a_car_non - 2.0 * omega_non.cross(nominal_velocity) -
                beta_non.cross(nominal_position) -
                omega_non.cross(omega_non.cross(nominal_position));
          }
          const auto constraint = field_hocbf::computeConstraint(
              selected, nominal_position, nominal_velocity, nominal_a_non);
          if (!constraint.active) continue;
          profile.block<3, 1>(0, stage) = constraint.A;
          profile(3, stage) = constraint.b;
          profile(4, stage) = 1.0;
          profile(5, stage) =
              500.0 * constraint.max_occupancy * constraint.max_occupancy;
        }
        profile_active_out = (profile.row(4).array() > 0.5).any();
        return profile;
      };

  bool field_profile_active = false;
  FieldHocbfProfile field_profile = buildFieldProfile(
      nullptr, field_profile_active);
  mpc_params_.cbf_field_hocbf_profile_ = field_profile;
  // The latest nonempty field owns the safety path even when pruning leaves a
  // stage inactive. This prevents legacy risk regions from being mixed with
  // a valid field snapshot. A new empty snapshot relinquishes priority.
  mpc_params_.cbf_use_field_hocbf_ = enable_cbf && field_available;
  if (field_hocbf_enabled_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "[MPC UAV " << quad_id
        << "] field_hocbf available=" << (field_available ? "true" : "false")
        << " active=" << (field_profile_active ? "true" : "false")
        << " age=" << field_age);
  }
  if (debug_check_noninertial_profile_col0_) {
    Eigen::Vector3d legacy_omega_non = Eigen::Vector3d::Zero();
    Eigen::Vector3d legacy_beta_non = Eigen::Vector3d::Zero();
    Eigen::Vector3d legacy_a_car_non = Eigen::Vector3d::Zero();
    acado_mpc::MpcController<double>::buildLegacyCurrentTimeNonInertialData(
        state_estimate_, legacy_omega_non, legacy_beta_non, legacy_a_car_non);
    const Eigen::Vector3d profile_omega_col0 = omega_non_profile.col(0);
    const Eigen::Vector3d profile_beta_col0 = beta_non_profile.col(0);
    const Eigen::Vector3d profile_a_car_col0 = a_car_non_profile.col(0);
    const Eigen::Vector3d diff_omega = profile_omega_col0 - legacy_omega_non;
    const Eigen::Vector3d diff_beta = profile_beta_col0 - legacy_beta_non;
    const Eigen::Vector3d diff_a_car = profile_a_car_col0 - legacy_a_car_non;
    constexpr double kCol0Tol = 1e-9;
    // beta_non is a discrete derivative signal; allow a slightly looser
    // tolerance to avoid flagging pure sampling/finite-difference noise.
    constexpr double kBetaCol0Tol = 1e-6;
    const double norm_diff_omega = diff_omega.norm();
    const double norm_diff_beta = diff_beta.norm();
    const double norm_diff_a_car = diff_a_car.norm();
    const bool a_car_match =
        debug_force_zero_a_car_non_profile_ || norm_diff_a_car <= kCol0Tol;
    const bool pass =
        norm_diff_omega <= kCol0Tol &&
        norm_diff_beta <= kBetaCol0Tol &&
        a_car_match;
    std::ostringstream check_log;
    check_log << "[NONINERTIAL_COL0_CHECK]"
              << " sim_time=" << formatScalar(debug_noninertial_profile_sim_time_)
              << " frame_mode_effective=" << frame_mode_effective_
              << " traj_id="
              << (debug_traj_id_.empty() ? std::string("NA") : debug_traj_id_)
              << " old_omega_non=" << formatVec3(legacy_omega_non)
              << " profile_omega_col0=" << formatVec3(profile_omega_col0)
              << " diff_omega=" << formatVec3(diff_omega)
              << " norm(diff_omega)=" << formatScalar(norm_diff_omega)
              << " old_beta_non=" << formatVec3(legacy_beta_non)
              << " profile_beta_col0=" << formatVec3(profile_beta_col0)
              << " diff_beta=" << formatVec3(diff_beta)
              << " norm(diff_beta)=" << formatScalar(norm_diff_beta)
              << " old_a_car_non=" << formatVec3(legacy_a_car_non)
              << " profile_a_car_non_col0=" << formatVec3(profile_a_car_col0)
              << " diff_a_car=" << formatVec3(diff_a_car)
              << " norm(diff_a_car)=" << formatScalar(norm_diff_a_car)
              << " a_car_check="
              << (debug_force_zero_a_car_non_profile_
                      ? "skipped(force_zero_a_car_non_profile=true)"
                      : "enabled")
              << " result="
              << (pass
                      ? "[PASS] col0 matches legacy current-time non-inertial data within tolerance"
                      : "[FAIL] col0 mismatch exceeds tolerance");
    if (pass) {
      ROS_INFO_STREAM(check_log.str());
    } else {
      ROS_WARN_STREAM(check_log.str());
    }
  }
  bool debug_force_legacy_noninertial_constant_profile = false;
  pnh_.param("debug/force_legacy_noninertial_constant_profile",
             debug_force_legacy_noninertial_constant_profile, false);
  const NonInertialProfile* omega_profile_arg =
      debug_force_legacy_noninertial_constant_profile ? nullptr
                                                      : &omega_non_profile;
  const NonInertialProfile* beta_profile_arg =
      debug_force_legacy_noninertial_constant_profile ? nullptr
                                                      : &beta_non_profile;
  const NonInertialProfile* a_car_profile_arg =
      debug_force_legacy_noninertial_constant_profile ? nullptr
                                                      : &a_car_non_profile;
#if CONI_MPC_EXTERNAL_FIELD_MICRO_ITERATIONS
  // Save the applied-input anchor before the first solve.  The second
  // OsqpEigen refinement must use the same anchor as the first QP.
  const auto slew_anchor_before_cycle = mpc_controller_.getSlewRateAnchor();
  const auto micro_iteration_start = std::chrono::steady_clock::now();
#endif
  auto command = mpc_controller_.run(
      state_estimate_, reference_window_, mpc_params_, world_q_model,
      world_p_model, omega_profile_arg, beta_profile_arg,
      a_car_profile_arg);
#if CONI_MPC_EXTERNAL_FIELD_MICRO_ITERATIONS
  // The OsqpEigen field controller performs two outer solves and restores
  // the anchor before the refinement solve.
  if (field_hocbf_enabled_ && field_available &&
      mpc_controller_.getLastSolveOk()) {
    // Rebuild the field from the first QP's rollout, then solve the refined
    // QP in the same control cycle.  The point cloud, car profile, and time
    // alignment are captured above and therefore remain fixed across both
    // micro-iterations.
    const auto first_prediction = mpc_controller_.getPredictedStates();
    const auto first_inputs = mpc_controller_.getPredictedInputs();
    const double first_slack = mpc_controller_.getLatestSlack();
    bool refined_field_active = false;
    const FieldHocbfProfile refined_field_profile = buildFieldProfile(
        &first_prediction, refined_field_active);
    const double profile_delta =
        (refined_field_profile - field_profile).norm();
    mpc_params_.cbf_field_hocbf_profile_ = refined_field_profile;
    mpc_params_.cbf_use_field_hocbf_ = enable_cbf && field_available;
    mpc_params_.changed_ = false;
    if (!mpc_controller_.setSlewRateAnchor(slew_anchor_before_cycle)) {
      ROS_ERROR_STREAM("[FIELD_HOCBF_MICRO] failed to restore slew-rate anchor");
    }
    const auto refined_command = mpc_controller_.run(
        state_estimate_, reference_window_, mpc_params_, world_q_model,
        world_p_model, omega_profile_arg, beta_profile_arg,
        a_car_profile_arg);
    if (mpc_controller_.getLastSolveOk()) {
      command = refined_command;
    } else {
      // The first solve is a valid feasible fallback.  Restore its rollout
      // and slew anchor so a failed refinement cannot publish half-updated
      // solver state or poison the next cycle's rate penalty.
      mpc_controller_.restoreSuccessfulPrediction(
          first_prediction, first_inputs, first_slack);
      ROS_WARN_STREAM_THROTTLE(
          0.5, "[FIELD_HOCBF_MICRO] refinement solve failed; keeping first QP command");
    }
    const double micro_elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - micro_iteration_start).count();
    ROS_INFO_STREAM_THROTTLE(
        1.0, "[FIELD_HOCBF_MICRO] iterations=2 first_active="
                 << (field_profile_active ? "true" : "false")
                 << " refined_active="
                 << (refined_field_active ? "true" : "false")
                 << " profile_delta=" << profile_delta
                 << " total_time_ms=" << micro_elapsed_ms);
  }
#else
  // ACADO's preparationStep/feedbackStep and qpOASES solve are the complete
  // backend iteration.  Do not run the OsqpEigen-style outer profile
  // refinement on top of that baseline.
  if (field_hocbf_enabled_ && field_available) {
    ROS_INFO_STREAM_THROTTLE(
        1.0, "[FIELD_HOCBF_MICRO] backend=ACADO iterations=1 "
             "external profile refinement disabled");
  }
#endif
  // Risk-region CBFs do not use the legacy spherical-obstacle diagnostic
  // vectors below.  Report the actual stage-0 p=4 barrier separately so a
  // collision can be distinguished from a stale/incorrect profile or from a
  // solver fallback.  This is evaluated in the same solver frame and with
  // the same Q^T convention as the condensed controller model.
  if (use_mpc_risk_regions && !mpc_risk_region_profiles.empty()) {
    std::ostringstream risk_state_log;
    risk_state_log << "[MPC UAV " << quad_id
                   << "] risk_region_stage0_state";
    for (std::size_t region_idx = 0;
         region_idx < mpc_risk_region_profiles.size(); ++region_idx) {
      const auto& profile = mpc_risk_region_profiles[region_idx];
      if (profile(21, 0) <= 0.5) continue;
      const Eigen::Vector3d center(profile(0, 0), profile(1, 0), profile(2, 0));
      const Eigen::Vector3d axes(
          std::max(1.0e-6, static_cast<double>(profile(3, 0))),
          std::max(1.0e-6, static_cast<double>(profile(4, 0))),
          std::max(1.0e-6, static_cast<double>(profile(5, 0))));
      Eigen::Matrix3d orientation;
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
          orientation(row, col) = profile(6 + 3 * row + col, 0);
        }
      }
      const Eigen::Vector3d local =
          orientation.transpose() * (state_estimate_.position - center);
      const Eigen::Vector3d rel_velocity =
          state_estimate_.velocity -
          Eigen::Vector3d(profile(15, 0), profile(16, 0), profile(17, 0));
      const Eigen::Vector3d local_velocity = orientation.transpose() * rel_velocity;
      const double h = std::pow(local.x() / axes.x(), 4.0) +
                       std::pow(local.y() / axes.y(), 4.0) +
                       std::pow(local.z() / axes.z(), 4.0) - 1.0;
      const double hdot =
          4.0 * std::pow(local.x(), 3.0) * local_velocity.x() /
              std::pow(axes.x(), 4.0) +
          4.0 * std::pow(local.y(), 3.0) * local_velocity.y() /
              std::pow(axes.y(), 4.0) +
          4.0 * std::pow(local.z(), 3.0) * local_velocity.z() /
              std::pow(axes.z(), 4.0);
      risk_state_log << " | region[" << region_idx << "] active="
                     << profile(21, 0) << " h=" << h << " hdot=" << hdot
                     << " center=(" << center.x() << "," << center.y() << ","
                     << center.z() << ") axes=(" << axes.x() << "," << axes.y()
                     << "," << axes.z() << ")";
    }
    ROS_WARN_STREAM_THROTTLE_NAMED(0.5, "risk_region_stage0_state",
                                   risk_state_log.str());
  }
  ROS_INFO_STREAM_THROTTLE(
      1.0,
      "[MPC UAV " << quad_id << "] cmd_rel=(" << command.velocity_cmd.x() << ", "
                   << command.velocity_cmd.y() << ", " << command.velocity_cmd.z()
                   << ") yaw_rate_rel=" << command.yaw_rate
                   << " slack=" << command.slack);
  if (mpc_controller_.getLastSolveOk()) {
    const auto& predicted_states = mpc_controller_.getPredictedStates();
    const std::size_t horizon_steps =
        static_cast<std::size_t>(acado_mpc::kSamples + 1);

    // Validate the actual condensed OsqpEigen trajectory against the sampled
    // p=4 risk-region safe-set constraints.  This diagnostic distinguishes a
    // failed/infeasible solve from a profile, coordinate, or timing mismatch.
    if (use_mpc_risk_regions && !mpc_risk_region_profiles.empty()) {
      double min_predicted_h = std::numeric_limits<double>::infinity();
      std::size_t min_region = 0u;
      std::size_t min_stage = 0u;
      for (std::size_t region_idx = 0u;
           region_idx < mpc_risk_region_profiles.size(); ++region_idx) {
        const auto& profile = mpc_risk_region_profiles[region_idx];
        const std::size_t profile_steps =
            std::min(horizon_steps, static_cast<std::size_t>(profile.cols()));
        for (std::size_t k = 0u; k < profile_steps; ++k) {
          if (profile(21, static_cast<int>(k)) <= 0.5) continue;
          const Eigen::Vector3d center(
              profile(0, static_cast<int>(k)),
              profile(1, static_cast<int>(k)),
              profile(2, static_cast<int>(k)));
          const Eigen::Vector3d axes(
              std::max(1.0e-6, static_cast<double>(profile(3, static_cast<int>(k)))),
              std::max(1.0e-6, static_cast<double>(profile(4, static_cast<int>(k)))),
              std::max(1.0e-6, static_cast<double>(profile(5, static_cast<int>(k)))));
          Eigen::Matrix3d orientation;
          for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
              orientation(row, col) =
                  profile(6 + 3 * row + col, static_cast<int>(k));
            }
          }
          const Eigen::Vector3d position(
              predicted_states(acado_mpc::kPosX, static_cast<int>(k)),
              predicted_states(acado_mpc::kPosY, static_cast<int>(k)),
              predicted_states(acado_mpc::kPosZ, static_cast<int>(k)));
          const Eigen::Vector3d local = orientation.transpose() * (position - center);
          const double h = std::pow(local.x() / axes.x(), 4.0) +
                           std::pow(local.y() / axes.y(), 4.0) +
                           std::pow(local.z() / axes.z(), 4.0) - 1.0;
          if (h < min_predicted_h) {
            min_predicted_h = h;
            min_region = region_idx;
            min_stage = k;
          }
        }
      }
      if (std::isfinite(min_predicted_h)) {
        ROS_WARN_STREAM_THROTTLE_NAMED(
            0.2, "risk_predicted_min_h",
            "[MPC UAV " << quad_id << "] risk_predicted_min_h="
                         << min_predicted_h << " region=" << min_region
                         << " stage=" << min_stage);
      }
    }
    last_predicted_world_positions_.assign(horizon_steps, Eigen::Vector3d::Zero());
    const CarState_t current_car_state = carStateFromOdom(car_odom_);
    for (std::size_t k = 0; k < horizon_steps; ++k) {
      const Eigen::Vector3d relative_position(
          predicted_states(acado_mpc::kPosX, static_cast<int>(k)),
          predicted_states(acado_mpc::kPosY, static_cast<int>(k)),
          predicted_states(acado_mpc::kPosZ, static_cast<int>(k)));
      if (isInertialFrame()) {
        last_predicted_world_positions_[k] = relative_position;
        continue;
      }
      const CarState_t& car_state_k =
          (k < car_trajectory_window_.size()) ? car_trajectory_window_[k]
                                              : current_car_state;
      Eigen::Quaterniond W_q_non(
          car_state_k(6), car_state_k(7), car_state_k(8), car_state_k(9));
      W_q_non.normalize();
      const Eigen::Vector3d car_position(
          car_state_k(0), car_state_k(1), car_state_k(2));
      last_predicted_world_positions_[k] =
          car_position + W_q_non * relative_position;
    }
    has_last_predicted_world_positions_ = true;
  } else {
    last_predicted_world_positions_.clear();
    has_last_predicted_world_positions_ = false;
  }
  mpc_params_.changed_ = false;
  metrics_samples_ += 1;

  const bool solver_cbf_active = cbf_active && !active_h_values.empty() &&
                                 !active_hdot_values.empty() &&
                                 !active_cbf_values.empty();
  const bool has_bound_solver_active_obstacle =
      solver_cbf_active && has_active_obstacle_solver &&
      has_active_obstacle_world && solver_active_obstacle_key != "none" &&
      solver_active_idx != kInvalidSolverActiveIdx &&
      solver_active_idx < active_h_values.size() &&
      solver_active_idx < active_hdot_values.size() &&
      solver_active_idx < active_cbf_values.size();
  const bool solver_track_only = !solver_cbf_active;
  const bool turning =
      (turning_omega_threshold_ > 0.0 &&
       std::abs(car_odom_.twist.twist.angular.z) >= turning_omega_threshold_);
  if (turning) {
    metrics_turn_samples_ += 1;
  } else {
    metrics_straight_samples_ += 1;
  }
  if (solver_cbf_active && !has_bound_solver_active_obstacle) {
    ROS_WARN_STREAM_THROTTLE(
        1.0,
        "[" << pnh_.getNamespace()
            << "] solver_cbf_active=true but solver-side clean metrics cannot "
               "be bound to a consistent solver_active_idx; step_solver_h, "
               "step_solver_hdot, step_solver_cbf, and "
               "step_solver_planar_clearance are NaN for this step.");
  }
  // solver-side clean metrics are bound to the obstacle actually injected into
  // the solver in the current step.
  const double step_solver_h =
      has_bound_solver_active_obstacle
          ? active_h_values[solver_active_idx]
          : std::numeric_limits<double>::quiet_NaN();
  const double step_solver_hdot =
      has_bound_solver_active_obstacle
          ? active_hdot_values[solver_active_idx]
          : std::numeric_limits<double>::quiet_NaN();
  const double step_solver_cbf =
      has_bound_solver_active_obstacle
          ? active_cbf_values[solver_active_idx]
          : std::numeric_limits<double>::quiet_NaN();
  const double step_solver_planar_clearance =
      has_bound_solver_active_obstacle
          ? ((state_estimate_.position.head<2>() -
              active_obstacle_solver.head<2>()).norm() -
             active_obstacle_solver(3))
          : std::numeric_limits<double>::quiet_NaN();
  const double step_solver_planar_distance =
      has_bound_solver_active_obstacle
          ? (state_estimate_.position.head<2>() -
             active_obstacle_solver.head<2>()).norm()
          : std::numeric_limits<double>::quiet_NaN();
  const double step_solver_planar_surface_distance =
      has_bound_solver_active_obstacle
          ? (step_solver_planar_distance -
             std::max(active_obstacle_solver(3) -
                          mpc_params_.cbf_safety_margin_,
                      0.0))
          : std::numeric_limits<double>::quiet_NaN();
  const double step_diag_min_h_any =
      h_values.empty()
          ? std::numeric_limits<double>::quiet_NaN()
          : *std::min_element(h_values.begin(), h_values.end());
  const double step_diag_min_cbf_any =
      cbf_values.empty()
          ? std::numeric_limits<double>::quiet_NaN()
          : *std::min_element(cbf_values.begin(), cbf_values.end());
  const std::string effective_solver_active_obstacle_key =
      has_bound_solver_active_obstacle ? solver_active_obstacle_key : "none";
  if (std::isfinite(step_solver_h)) {
    metrics_min_h_ = std::min(metrics_min_h_, step_solver_h);
    metrics_sum_h_ += step_solver_h;
    metrics_solver_metric_samples_ += 1;
  }
  if (solver_cbf_active && std::isfinite(step_solver_planar_clearance)) {
    metrics_solver_min_planar_clearance_ =
        std::min(metrics_solver_min_planar_clearance_,
                 step_solver_planar_clearance);
  }
  if (std::isfinite(step_solver_cbf)) {
    metrics_min_cbf_ = std::min(metrics_min_cbf_, step_solver_cbf);
    metrics_sum_cbf_ += step_solver_cbf;
  }
  const double c_value = std::abs(car_odom_.twist.twist.angular.z);
  metrics_c_values_.push_back(c_value);
  const Eigen::Vector2d relative_position_xy(
      quad_odom_.pose.pose.position.x - car_odom_.pose.pose.position.x,
      quad_odom_.pose.pose.position.y - car_odom_.pose.pose.position.y);
  const double c66_coupled_value = c_value * relative_position_xy.norm();
  metrics_c66_coupled_values_.push_back(c66_coupled_value);
  metrics_step_min_h_.push_back(step_solver_h);
  metrics_step_slack_.push_back(std::numeric_limits<double>::quiet_NaN());
  metrics_step_tracking_error_.push_back(std::numeric_limits<double>::quiet_NaN());
  metrics_step_fail_flags_.push_back(0);
  if (solver_track_only) {
    metrics_solver_track_only_samples_ += 1;
  }
  if (!reference_window_.points.empty()) {
    const Eigen::Vector3d tracking_error =
        state_estimate_.position - reference_window_.points.front().position;
    const double e = tracking_error.norm();
    metrics_tracking_sum_sq_ += e * e;
    metrics_tracking_sum_sq_xyz_ += tracking_error.cwiseProduct(tracking_error);
    metrics_tracking_errors_.push_back(e);
    if (solver_track_only) {
      metrics_tracking_sum_sq_track_only_ += e * e;
      metrics_tracking_sum_sq_track_only_xyz_ +=
          tracking_error.cwiseProduct(tracking_error);
      metrics_tracking_errors_track_only_.push_back(e);
    }
    if (solver_cbf_active) {
      metrics_tracking_sum_sq_cbf_active_ += e * e;
      metrics_cbf_active_samples_ += 1;
      metrics_tracking_sum_sq_solver_cbf_active_ += e * e;
      metrics_solver_cbf_active_tracking_samples_ += 1;
    }
    if (turning) {
      metrics_tracking_sum_sq_turn_ += e * e;
    } else {
      metrics_tracking_sum_sq_straight_ += e * e;
    }
    metrics_step_tracking_error_.back() = e;
  } else if (solver_cbf_active) {
    ROS_WARN_STREAM_THROTTLE(
        1.0,
        "[" << pnh_.getNamespace()
            << "] solver_cbf_active=true but reference_window_ is empty; "
               "solver_tracking_rms_avoid is not accumulated for this step.");
  }

  const double latest_slack_raw =
      static_cast<double>(mpc_controller_.getLatestSlack());
  const double latest_slack =
      static_cast<double>(command.slack);
  metrics_step_slack_.back() = latest_slack;
  const bool update_solver_min_h_witness =
      solver_cbf_active && std::isfinite(step_solver_h) &&
      step_solver_h < metrics_solver_min_h_;
  if (solver_cbf_active && std::isfinite(step_solver_h)) {
    if (update_solver_min_h_witness) {
      metrics_solver_min_h_ = step_solver_h;
      metrics_solver_min_h_witness_h_ = step_solver_h;
      metrics_solver_min_h_witness_planar_clearance_ =
          step_solver_planar_clearance;
      metrics_solver_min_h_witness_tracking_error_ =
          metrics_step_tracking_error_.back();
      metrics_solver_min_h_witness_delta_ = latest_slack;
      metrics_solver_min_h_witness_planar_distance_ =
          step_solver_planar_distance;
      metrics_solver_min_h_witness_active_obstacle_key_ =
          effective_solver_active_obstacle_key;
    } else {
      metrics_solver_min_h_ = std::min(metrics_solver_min_h_, step_solver_h);
    }
  }
  metrics_slack_sum_ += latest_slack;
  metrics_max_slack_ = std::max(metrics_max_slack_, latest_slack);
  if (turning) {
    metrics_slack_sum_turn_ += latest_slack;
    metrics_max_slack_turn_ = std::max(metrics_max_slack_turn_, latest_slack);
    if (std::isfinite(step_solver_h)) {
      metrics_min_h_turn_ = std::min(metrics_min_h_turn_, step_solver_h);
    }
  } else {
    metrics_slack_sum_straight_ += latest_slack;
    metrics_max_slack_straight_ = std::max(metrics_max_slack_straight_, latest_slack);
    if (std::isfinite(step_solver_h)) {
      metrics_min_h_straight_ = std::min(metrics_min_h_straight_, step_solver_h);
    }
  }
  const bool debug_use_active = solver_cbf_active;
  const auto& debug_h_values = debug_use_active ? active_h_values : h_values;
  const auto& debug_hdot_values = debug_use_active ? active_hdot_values : hdot_values;
  const auto& debug_hddot_values =
      debug_use_active ? active_hddot_values : hddot_values;
  const auto& debug_cbf_values = debug_use_active ? active_cbf_values : cbf_values;
  if (metrics_debug_ && !debug_h_values.empty()) {
    for (size_t obs_idx = 0; obs_idx < debug_h_values.size(); ++obs_idx) {
      ROS_INFO_STREAM("[UAV " << quad_id << "] CBF metrics | h[" << obs_idx << "]: " << debug_h_values[obs_idx]
                      << " | hdot[" << obs_idx << "]: " << debug_hdot_values[obs_idx]
                      << " | hddot[" << obs_idx << "]: " << debug_hddot_values[obs_idx]
                      << " | cbf[" << obs_idx << "]: " << debug_cbf_values[obs_idx]
                      << " | slack: " << latest_slack
                      << " (raw: " << latest_slack_raw << ")"
                      << " | enabled: " << std::boolalpha << enable_cbf);
    }
  }

  const double feedback_time_ms = mpc_controller_.getLastFeedbackTimeMs();
  const double preparation_time_ms = mpc_controller_.getLastPreparationTimeMs();
  const double solve_time_ms = feedback_time_ms;
  metrics_solve_time_ms_.push_back(solve_time_ms);
  metrics_solve_time_max_ms_ = std::max(metrics_solve_time_max_ms_, solve_time_ms);
  metrics_feedback_time_ms_.push_back(feedback_time_ms);
  metrics_feedback_time_max_ms_ =
      std::max(metrics_feedback_time_max_ms_, feedback_time_ms);
  metrics_preparation_time_ms_.push_back(preparation_time_ms);
  metrics_preparation_time_max_ms_ =
      std::max(metrics_preparation_time_max_ms_, preparation_time_ms);
  StepDebugSnapshot current_step_debug;
  current_step_debug.valid = true;
  current_step_debug.cbf_active = cbf_active;
  current_step_debug.solver_cbf_active = solver_cbf_active;
  current_step_debug.solve_ok = mpc_controller_.getLastSolveOk();
  current_step_debug.solver_fail_flag = !current_step_debug.solve_ok;
  current_step_debug.turning = turning;
  current_step_debug.active_obstacle_key = effective_active_obstacle_key;
  current_step_debug.solver_active_obstacle_key =
      effective_solver_active_obstacle_key;
  current_step_debug.solver_track_only = solver_track_only;
  if (has_active_obstacle_world) {
    current_step_debug.active_obstacle_world = active_obstacle_world;
  }
  if (has_active_obstacle_solver) {
    current_step_debug.active_obstacle_solver = active_obstacle_solver;
  }
  current_step_debug.solver_h = step_solver_h;
  current_step_debug.solver_hdot = step_solver_hdot;
  current_step_debug.solver_cbf = step_solver_cbf;
  current_step_debug.solver_slack = latest_slack;
  current_step_debug.solver_planar_distance = step_solver_planar_distance;
  current_step_debug.solver_planar_clearance = step_solver_planar_clearance;
  current_step_debug.solver_planar_surface_distance =
      step_solver_planar_surface_distance;
  current_step_debug.feedback_time_ms = feedback_time_ms;
  current_step_debug.preparation_time_ms = preparation_time_ms;
  current_step_debug.core_time_ms = feedback_time_ms + preparation_time_ms;
  current_step_debug.tracking_error = metrics_step_tracking_error_.back();
  current_step_debug.diag_min_h_any = step_diag_min_h_any;
  current_step_debug.diag_min_cbf_any = step_diag_min_cbf_any;
  current_step_debug.min_h = step_solver_h;
  current_step_debug.min_cbf = step_solver_cbf;
  current_step_debug.slack = latest_slack;
  current_step_debug.slack_raw = latest_slack_raw;
  current_step_debug.omega_non = state_estimate_.omega_non;
  current_step_debug.beta_non = state_estimate_.beta_non;
  current_step_debug.a_car_non =
      state_estimate_.a_imu - Eigen::Vector3d(0.0, 0.0, 9.81);
  if (!car_trajectory_window_.empty()) {
    current_step_debug.car_a_world = Eigen::Vector3d(
        car_trajectory_window_.front()(10),
        car_trajectory_window_.front()(11),
        car_trajectory_window_.front()(12));
    current_step_debug.car_omega_world = Eigen::Vector3d(
        car_trajectory_window_.front()(13),
        car_trajectory_window_.front()(14),
        car_trajectory_window_.front()(15));
  }
  current_step_debug.position = state_estimate_.position;
  current_step_debug.velocity = state_estimate_.velocity;
  Eigen::Quaterniond W_q_non(car_odom_.pose.pose.orientation.w,
                             car_odom_.pose.pose.orientation.x,
                             car_odom_.pose.pose.orientation.y,
                             car_odom_.pose.pose.orientation.z);
  if (W_q_non.norm() > 1e-9) {
    W_q_non.normalize();
  } else {
    W_q_non = Eigen::Quaterniond::Identity();
  }
  const Eigen::Vector3d car_position_world(
      car_odom_.pose.pose.position.x,
      car_odom_.pose.pose.position.y,
      car_odom_.pose.pose.position.z);
  const Eigen::Vector3d quad_position_world(
      quad_odom_.pose.pose.position.x,
      quad_odom_.pose.pose.position.y,
      quad_odom_.pose.pose.position.z);
  const Eigen::Vector3d car_velocity_world(
      car_odom_.twist.twist.linear.x,
      car_odom_.twist.twist.linear.y,
      car_odom_.twist.twist.linear.z);
  const Eigen::Vector3d quad_velocity_world(
      quad_odom_.twist.twist.linear.x,
      quad_odom_.twist.twist.linear.y,
      quad_odom_.twist.twist.linear.z);
  const Eigen::Vector3d car_omega_non_raw(
      car_odom_.twist.twist.angular.x,
      car_odom_.twist.twist.angular.y,
      car_odom_.twist.twist.angular.z);
  const Eigen::Vector3d position_relative_non =
      W_q_non.inverse() * (quad_position_world - car_position_world);
  const Eigen::Vector3d velocity_relative_non =
      -car_omega_non_raw.cross(position_relative_non) +
      W_q_non.inverse() * (quad_velocity_world - car_velocity_world);
  current_step_debug.rot_load_position_non = position_relative_non;
  current_step_debug.rot_load_velocity_non = velocity_relative_non;
  current_step_debug.rot_load_omega_non = car_omega_non_raw;
  if (!current_step_debug.solve_ok) {
    metrics_failures_ += 1;
    if (turning) {
      metrics_failures_turn_ += 1;
    } else {
      metrics_failures_straight_ += 1;
    }
    logFailureContext(current_step_debug);
  }
  metrics_step_fail_flags_.back() = current_step_debug.solve_ok ? 0 : 1;
  prev_step_debug_ = current_step_debug;
  last_cbf_active_ = cbf_active;
  last_active_obstacle_key_ = effective_active_obstacle_key;
  return command;
}

namespace {
double computeMean(const std::vector<double>& values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double sum = 0.0;
  for (double v : values) {
    sum += v;
  }
  return sum / static_cast<double>(values.size());
}

double computePercentile(const std::vector<double>& values, double q) {
  std::vector<double> finite_values;
  finite_values.reserve(values.size());
  for (double value : values) {
    if (std::isfinite(value)) {
      finite_values.push_back(value);
    }
  }
  if (finite_values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(finite_values.begin(), finite_values.end());
  const double clamped_q = std::max(0.0, std::min(1.0, q));
  const double rank =
      clamped_q * static_cast<double>(finite_values.size() - 1);
  const size_t lower_idx = static_cast<size_t>(std::floor(rank));
  const size_t upper_idx = static_cast<size_t>(std::ceil(rank));
  if (lower_idx == upper_idx) {
    return finite_values[lower_idx];
  }
  const double alpha = rank - static_cast<double>(lower_idx);
  return finite_values[lower_idx] * (1.0 - alpha) +
         finite_values[upper_idx] * alpha;
}

double computeP95(const std::vector<double>& values) {
  return computePercentile(values, 0.95);
}
}  // namespace

void NumSimMpc::logFailureContext(
    const StepDebugSnapshot& current_step) const {
  ROS_ERROR_STREAM(
      "[UAV " << quad_id << "] MPC fail current-step"
      << " | frame=" << frame_mode_effective_
      << " | safety_variant=" << safety_variant_
      << " | cbf_active=" << std::boolalpha << current_step.cbf_active
      << " | active_key=" << current_step.active_obstacle_key
      << " | solver_active_key=" << current_step.solver_active_obstacle_key
      << " | min_h=" << formatScalar(current_step.min_h)
      << " | min_cbf=" << formatScalar(current_step.min_cbf)
      << " | slack=" << formatScalar(current_step.slack)
      << " | slack_raw=" << formatScalar(current_step.slack_raw)
      << " | omega_non=" << formatVec3(current_step.omega_non)
      << " | beta_non=" << formatVec3(current_step.beta_non)
      << " | a_car_non=" << formatVec3(current_step.a_car_non)
      << " | position=" << formatVec3(current_step.position)
      << " | velocity=" << formatVec3(current_step.velocity)
      << " | obs_world=" << formatObstacle(current_step.active_obstacle_world)
      << " | obs_solver=" << formatObstacle(current_step.active_obstacle_solver));
  if (!prev_step_debug_.valid) {
    ROS_ERROR_STREAM("[UAV " << quad_id
                             << "] MPC fail previous-step | unavailable");
    return;
  }
  ROS_ERROR_STREAM(
      "[UAV " << quad_id << "] MPC fail previous-step"
      << " | solve_ok=" << std::boolalpha << prev_step_debug_.solve_ok
      << " | turning=" << std::boolalpha << prev_step_debug_.turning
      << " | cbf_active=" << std::boolalpha << prev_step_debug_.cbf_active
      << " | active_key=" << prev_step_debug_.active_obstacle_key
      << " | solver_active_key="
      << prev_step_debug_.solver_active_obstacle_key
      << " | min_h=" << formatScalar(prev_step_debug_.min_h)
      << " | min_cbf=" << formatScalar(prev_step_debug_.min_cbf)
      << " | slack=" << formatScalar(prev_step_debug_.slack)
      << " | slack_raw=" << formatScalar(prev_step_debug_.slack_raw)
      << " | omega_non=" << formatVec3(prev_step_debug_.omega_non)
      << " | beta_non=" << formatVec3(prev_step_debug_.beta_non)
      << " | a_car_non=" << formatVec3(prev_step_debug_.a_car_non)
      << " | position=" << formatVec3(prev_step_debug_.position)
      << " | velocity=" << formatVec3(prev_step_debug_.velocity)
      << " | obs_world=" << formatObstacle(prev_step_debug_.active_obstacle_world)
      << " | obs_solver=" << formatObstacle(prev_step_debug_.active_obstacle_solver));
}

void NumSimMpc::resetMetrics() {
  metrics_min_h_ = std::numeric_limits<double>::infinity();
  metrics_min_cbf_ = std::numeric_limits<double>::infinity();
  metrics_max_slack_ = 0.0;
  metrics_sum_h_ = 0.0;
  metrics_sum_cbf_ = 0.0;
  metrics_tracking_sum_sq_ = 0.0;
  metrics_tracking_sum_sq_xyz_.setZero();
  metrics_tracking_errors_.clear();
  metrics_tracking_sum_sq_track_only_ = 0.0;
  metrics_tracking_sum_sq_track_only_xyz_.setZero();
  metrics_tracking_errors_track_only_.clear();
  metrics_tracking_sum_sq_cbf_active_ = 0.0;
  metrics_tracking_sum_sq_solver_cbf_active_ = 0.0;
  metrics_tracking_sum_sq_turn_ = 0.0;
  metrics_tracking_sum_sq_straight_ = 0.0;
  metrics_diag_min_h_any_ = std::numeric_limits<double>::infinity();
  metrics_diag_min_cbf_any_ = std::numeric_limits<double>::infinity();
  metrics_diag_sum_h_any_ = 0.0;
  metrics_diag_sum_cbf_any_ = 0.0;
  metrics_solver_metric_samples_ = 0;
  metrics_solver_cbf_active_tracking_samples_ = 0;
  metrics_solver_track_only_samples_ = 0;
  metrics_solver_min_h_ = std::numeric_limits<double>::infinity();
  metrics_solver_min_planar_clearance_ = std::numeric_limits<double>::infinity();
  metrics_solver_min_h_witness_h_ = std::numeric_limits<double>::quiet_NaN();
  metrics_solver_min_h_witness_planar_clearance_ =
      std::numeric_limits<double>::quiet_NaN();
  metrics_solver_min_h_witness_tracking_error_ =
      std::numeric_limits<double>::quiet_NaN();
  metrics_solver_min_h_witness_delta_ =
      std::numeric_limits<double>::quiet_NaN();
  metrics_solver_min_h_witness_planar_distance_ =
      std::numeric_limits<double>::quiet_NaN();
  metrics_solver_min_h_witness_active_obstacle_key_ = "none";
  metrics_c_values_.clear();
  metrics_c66_coupled_values_.clear();
  metrics_step_min_h_.clear();
  metrics_step_slack_.clear();
  metrics_step_tracking_error_.clear();
  metrics_step_fail_flags_.clear();
  metrics_dmin_min_ = std::numeric_limits<double>::infinity();
  metrics_slack_sum_ = 0.0;
  metrics_slack_sum_turn_ = 0.0;
  metrics_slack_sum_straight_ = 0.0;
  metrics_max_slack_turn_ = 0.0;
  metrics_max_slack_straight_ = 0.0;
  metrics_min_h_turn_ = std::numeric_limits<double>::infinity();
  metrics_min_h_straight_ = std::numeric_limits<double>::infinity();
  metrics_solve_time_ms_.clear();
  metrics_solve_time_max_ms_ = 0.0;
  metrics_feedback_time_ms_.clear();
  metrics_feedback_time_max_ms_ = 0.0;
  metrics_preparation_time_ms_.clear();
  metrics_preparation_time_max_ms_ = 0.0;
  metrics_failures_ = 0;
  metrics_failures_turn_ = 0;
  metrics_failures_straight_ = 0;
  metrics_cbf_active_samples_ = 0;
  metrics_turn_samples_ = 0;
  metrics_straight_samples_ = 0;
  metrics_samples_ = 0;
  last_active_obstacle_key_ = "none";
  last_cbf_active_ = false;
  last_selected_obstacle_keys_.clear();
  last_predicted_world_positions_.clear();
  has_last_predicted_world_positions_ = false;
  obstacle_hysteresis_cycles_since_switch_ = 0;
  prev_step_debug_ = StepDebugSnapshot();
}

NumSimMpc::MetricsSummary NumSimMpc::getMetrics() const {
  MetricsSummary summary;
  summary.samples = metrics_samples_;
  if (metrics_samples_ == 0) {
    summary.min_h = std::numeric_limits<double>::quiet_NaN();
    summary.min_cbf = std::numeric_limits<double>::quiet_NaN();
    summary.max_slack = std::numeric_limits<double>::quiet_NaN();
    summary.mean_h = std::numeric_limits<double>::quiet_NaN();
    summary.mean_cbf = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_x = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_y = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_z = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_p95 = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_track_only = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_track_only_x = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_track_only_y = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_track_only_z = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_p95_track_only = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_cbf_active = std::numeric_limits<double>::quiet_NaN();
    summary.solver_tracking_rms_avoid = std::numeric_limits<double>::quiet_NaN();
    summary.solver_tracking_rms_avoid_samples = 0;
    summary.fail_rate_turn = std::numeric_limits<double>::quiet_NaN();
    summary.fail_rate_straight = std::numeric_limits<double>::quiet_NaN();
    summary.min_h_turn = std::numeric_limits<double>::quiet_NaN();
    summary.min_h_straight = std::numeric_limits<double>::quiet_NaN();
    summary.slack_sum_turn = std::numeric_limits<double>::quiet_NaN();
    summary.slack_sum_straight = std::numeric_limits<double>::quiet_NaN();
    summary.max_slack_turn = std::numeric_limits<double>::quiet_NaN();
    summary.max_slack_straight = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_all_turn = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_all_straight = std::numeric_limits<double>::quiet_NaN();
    summary.c44_metric = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_high_c44 = std::numeric_limits<double>::quiet_NaN();
    summary.c66_metric = std::numeric_limits<double>::quiet_NaN();
    summary.c88_metric = std::numeric_limits<double>::quiet_NaN();
    summary.min_h_high_c = std::numeric_limits<double>::quiet_NaN();
    summary.slack_sum_high_c = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_high_c = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_high_c88 = std::numeric_limits<double>::quiet_NaN();
    summary.fail_rate_high_c = std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h = std::numeric_limits<double>::quiet_NaN();
    summary.solver_active_mean_h = std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_planar_clearance =
        std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h_witness_h = std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h_witness_planar_clearance =
        std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h_witness_tracking_error =
        std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h_witness_delta =
        std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h_witness_planar_distance =
        std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h_witness_active_obstacle_key = "none";
    summary.solver_min_cbf = std::numeric_limits<double>::quiet_NaN();
    summary.solver_mean_h = std::numeric_limits<double>::quiet_NaN();
    summary.solver_mean_cbf = std::numeric_limits<double>::quiet_NaN();
    summary.solver_slack_sum = std::numeric_limits<double>::quiet_NaN();
    summary.solver_max_slack = std::numeric_limits<double>::quiet_NaN();
    summary.solver_fail_rate = std::numeric_limits<double>::quiet_NaN();
    summary.solver_fail_rate_turn = std::numeric_limits<double>::quiet_NaN();
    summary.solver_fail_rate_straight = std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h_turn = std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h_straight = std::numeric_limits<double>::quiet_NaN();
    summary.solver_slack_sum_turn = std::numeric_limits<double>::quiet_NaN();
    summary.solver_slack_sum_straight = std::numeric_limits<double>::quiet_NaN();
    summary.solver_max_slack_turn = std::numeric_limits<double>::quiet_NaN();
    summary.solver_max_slack_straight = std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h_high_c = std::numeric_limits<double>::quiet_NaN();
    summary.solver_slack_sum_high_c = std::numeric_limits<double>::quiet_NaN();
    summary.solver_fail_rate_high_c = std::numeric_limits<double>::quiet_NaN();
    summary.diag_min_h_any = std::numeric_limits<double>::quiet_NaN();
    summary.diag_mean_h_any = std::numeric_limits<double>::quiet_NaN();
    summary.diag_min_cbf_any = std::numeric_limits<double>::quiet_NaN();
    summary.diag_mean_cbf_any = std::numeric_limits<double>::quiet_NaN();
    summary.dmin_min = std::numeric_limits<double>::quiet_NaN();
    summary.slack_sum = std::numeric_limits<double>::quiet_NaN();
    summary.solve_time_mean_ms = std::numeric_limits<double>::quiet_NaN();
    summary.solve_time_p95_ms = std::numeric_limits<double>::quiet_NaN();
    summary.solve_time_max_ms = std::numeric_limits<double>::quiet_NaN();
    summary.feedback_time_mean_ms = std::numeric_limits<double>::quiet_NaN();
    summary.feedback_time_p95_ms = std::numeric_limits<double>::quiet_NaN();
    summary.feedback_time_max_ms = std::numeric_limits<double>::quiet_NaN();
    summary.preparation_time_mean_ms = std::numeric_limits<double>::quiet_NaN();
    summary.preparation_time_p95_ms = std::numeric_limits<double>::quiet_NaN();
    summary.preparation_time_max_ms = std::numeric_limits<double>::quiet_NaN();
    summary.fail_rate = std::numeric_limits<double>::quiet_NaN();
    summary.solver_cbf_active_samples = 0;
    summary.solver_track_only_samples = 0;
    summary.cbf_active_samples = 0;
    summary.high_c44_samples = 0;
    summary.high_c_samples = 0;
    summary.high_c88_samples = 0;
    summary.track_only_samples = 0;
    summary.turn_samples = 0;
    summary.straight_samples = 0;
    return summary;
  }
  summary.min_h = std::isfinite(metrics_min_h_)
                      ? metrics_min_h_
                      : std::numeric_limits<double>::quiet_NaN();
  summary.min_cbf = std::isfinite(metrics_min_cbf_)
                        ? metrics_min_cbf_
                        : std::numeric_limits<double>::quiet_NaN();
  summary.max_slack = metrics_max_slack_;
  summary.mean_h = (metrics_solver_metric_samples_ > 0)
                       ? (metrics_sum_h_ /
                          static_cast<double>(metrics_solver_metric_samples_))
                       : std::numeric_limits<double>::quiet_NaN();
  summary.mean_cbf = (metrics_solver_metric_samples_ > 0)
                         ? (metrics_sum_cbf_ /
                            static_cast<double>(metrics_solver_metric_samples_))
                         : std::numeric_limits<double>::quiet_NaN();
  summary.tracking_rms =
      std::sqrt(metrics_tracking_sum_sq_ / static_cast<double>(metrics_samples_));
  summary.tracking_rms_x = std::sqrt(
      metrics_tracking_sum_sq_xyz_.x() / static_cast<double>(metrics_samples_));
  summary.tracking_rms_y = std::sqrt(
      metrics_tracking_sum_sq_xyz_.y() / static_cast<double>(metrics_samples_));
  summary.tracking_rms_z = std::sqrt(
      metrics_tracking_sum_sq_xyz_.z() / static_cast<double>(metrics_samples_));
  summary.tracking_p95 = computeP95(metrics_tracking_errors_);
  summary.solver_cbf_active_samples = metrics_solver_metric_samples_;
  summary.solver_tracking_rms_avoid_samples =
      metrics_solver_cbf_active_tracking_samples_;
  summary.cbf_active_samples = metrics_cbf_active_samples_;
  if (summary.cbf_active_samples == 0) {
    summary.tracking_rms_cbf_active = std::numeric_limits<double>::quiet_NaN();
  } else {
    summary.tracking_rms_cbf_active =
        std::sqrt(metrics_tracking_sum_sq_cbf_active_ /
                  static_cast<double>(summary.cbf_active_samples));
  }
  if (summary.solver_tracking_rms_avoid_samples == 0) {
    summary.solver_tracking_rms_avoid =
        std::numeric_limits<double>::quiet_NaN();
  } else {
    summary.solver_tracking_rms_avoid =
        std::sqrt(metrics_tracking_sum_sq_solver_cbf_active_ /
                  static_cast<double>(
                      summary.solver_tracking_rms_avoid_samples));
  }
  summary.solver_track_only_samples = metrics_solver_track_only_samples_;
  summary.track_only_samples = metrics_solver_track_only_samples_;
  const std::size_t track_only_tracking_samples =
      metrics_tracking_errors_track_only_.size();
  if (track_only_tracking_samples == 0) {
    summary.tracking_rms_track_only = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_track_only_x = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_track_only_y = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_rms_track_only_z = std::numeric_limits<double>::quiet_NaN();
    summary.tracking_p95_track_only = std::numeric_limits<double>::quiet_NaN();
  } else {
    summary.tracking_rms_track_only = std::sqrt(
        metrics_tracking_sum_sq_track_only_ /
        static_cast<double>(track_only_tracking_samples));
    summary.tracking_rms_track_only_x = std::sqrt(
        metrics_tracking_sum_sq_track_only_xyz_.x() /
        static_cast<double>(track_only_tracking_samples));
    summary.tracking_rms_track_only_y = std::sqrt(
        metrics_tracking_sum_sq_track_only_xyz_.y() /
        static_cast<double>(track_only_tracking_samples));
    summary.tracking_rms_track_only_z = std::sqrt(
        metrics_tracking_sum_sq_track_only_xyz_.z() /
        static_cast<double>(track_only_tracking_samples));
    summary.tracking_p95_track_only = computeP95(metrics_tracking_errors_track_only_);
  }
  summary.dmin_min = std::isfinite(metrics_dmin_min_)
                         ? metrics_dmin_min_
                         : std::numeric_limits<double>::quiet_NaN();
  summary.slack_sum = metrics_slack_sum_;
  summary.solver_min_h =
      (metrics_solver_metric_samples_ > 0 && std::isfinite(metrics_solver_min_h_))
          ? metrics_solver_min_h_
          : std::numeric_limits<double>::quiet_NaN();
  summary.solver_active_mean_h =
      (metrics_solver_metric_samples_ > 0)
          ? (metrics_sum_h_ / static_cast<double>(metrics_solver_metric_samples_))
          : std::numeric_limits<double>::quiet_NaN();
  summary.solver_min_planar_clearance =
      (metrics_solver_metric_samples_ > 0 &&
       std::isfinite(metrics_solver_min_planar_clearance_))
          ? metrics_solver_min_planar_clearance_
          : std::numeric_limits<double>::quiet_NaN();
  if (metrics_solver_metric_samples_ > 0 && std::isfinite(metrics_solver_min_h_)) {
    summary.solver_min_h_witness_h = metrics_solver_min_h_witness_h_;
    summary.solver_min_h_witness_planar_clearance =
        metrics_solver_min_h_witness_planar_clearance_;
    summary.solver_min_h_witness_tracking_error =
        metrics_solver_min_h_witness_tracking_error_;
    summary.solver_min_h_witness_delta = metrics_solver_min_h_witness_delta_;
    summary.solver_min_h_witness_planar_distance =
        metrics_solver_min_h_witness_planar_distance_;
    summary.solver_min_h_witness_active_obstacle_key =
        metrics_solver_min_h_witness_active_obstacle_key_;
  } else {
    summary.solver_min_h_witness_h = std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h_witness_planar_clearance =
        std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h_witness_tracking_error =
        std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h_witness_delta =
        std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h_witness_planar_distance =
        std::numeric_limits<double>::quiet_NaN();
    summary.solver_min_h_witness_active_obstacle_key = "none";
  }
  summary.solver_min_cbf = summary.min_cbf;
  summary.solver_mean_h = summary.mean_h;
  summary.solver_mean_cbf = summary.mean_cbf;
  summary.solver_slack_sum = metrics_slack_sum_;
  summary.solver_max_slack = metrics_max_slack_;
  summary.diag_min_h_any = std::isfinite(metrics_diag_min_h_any_)
                               ? metrics_diag_min_h_any_
                               : std::numeric_limits<double>::quiet_NaN();
  summary.diag_min_cbf_any = std::isfinite(metrics_diag_min_cbf_any_)
                                 ? metrics_diag_min_cbf_any_
                                 : std::numeric_limits<double>::quiet_NaN();
  summary.diag_mean_h_any = (metrics_samples_ > 0 &&
                             std::isfinite(metrics_diag_min_h_any_))
                                ? (metrics_diag_sum_h_any_ /
                                   static_cast<double>(metrics_samples_))
                                : std::numeric_limits<double>::quiet_NaN();
  summary.diag_mean_cbf_any = (metrics_samples_ > 0 &&
                               std::isfinite(metrics_diag_min_cbf_any_))
                                  ? (metrics_diag_sum_cbf_any_ /
                                     static_cast<double>(metrics_samples_))
                                  : std::numeric_limits<double>::quiet_NaN();
  summary.turn_samples = metrics_turn_samples_;
  summary.straight_samples = metrics_straight_samples_;
  summary.fail_rate_turn =
      (summary.turn_samples == 0)
          ? std::numeric_limits<double>::quiet_NaN()
          : static_cast<double>(metrics_failures_turn_) /
                static_cast<double>(summary.turn_samples);
  summary.fail_rate_straight =
      (summary.straight_samples == 0)
          ? std::numeric_limits<double>::quiet_NaN()
          : static_cast<double>(metrics_failures_straight_) /
                static_cast<double>(summary.straight_samples);
  summary.min_h_turn = std::isfinite(metrics_min_h_turn_)
                           ? metrics_min_h_turn_
                           : std::numeric_limits<double>::quiet_NaN();
  summary.min_h_straight = std::isfinite(metrics_min_h_straight_)
                               ? metrics_min_h_straight_
                               : std::numeric_limits<double>::quiet_NaN();
  summary.slack_sum_turn = (summary.turn_samples == 0)
                               ? std::numeric_limits<double>::quiet_NaN()
                               : metrics_slack_sum_turn_;
  summary.slack_sum_straight = (summary.straight_samples == 0)
                                   ? std::numeric_limits<double>::quiet_NaN()
                                   : metrics_slack_sum_straight_;
  summary.max_slack_turn = (summary.turn_samples == 0)
                               ? std::numeric_limits<double>::quiet_NaN()
                               : metrics_max_slack_turn_;
  summary.max_slack_straight = (summary.straight_samples == 0)
                                   ? std::numeric_limits<double>::quiet_NaN()
                                   : metrics_max_slack_straight_;
  summary.solver_fail_rate_turn = summary.fail_rate_turn;
  summary.solver_fail_rate_straight = summary.fail_rate_straight;
  summary.solver_min_h_turn = summary.min_h_turn;
  summary.solver_min_h_straight = summary.min_h_straight;
  summary.solver_slack_sum_turn = summary.slack_sum_turn;
  summary.solver_slack_sum_straight = summary.slack_sum_straight;
  summary.solver_max_slack_turn = summary.max_slack_turn;
  summary.solver_max_slack_straight = summary.max_slack_straight;
  summary.tracking_rms_all_turn =
      (summary.turn_samples == 0)
          ? std::numeric_limits<double>::quiet_NaN()
          : std::sqrt(metrics_tracking_sum_sq_turn_ /
                      static_cast<double>(summary.turn_samples));
  summary.tracking_rms_all_straight =
      (summary.straight_samples == 0)
          ? std::numeric_limits<double>::quiet_NaN()
          : std::sqrt(metrics_tracking_sum_sq_straight_ /
                      static_cast<double>(summary.straight_samples));
  const std::size_t metrics_step_count = std::min(
      metrics_step_min_h_.size(),
      std::min(metrics_step_slack_.size(),
               std::min(metrics_step_tracking_error_.size(),
                        metrics_step_fail_flags_.size())));
  const std::size_t legacy_c_step_count =
      std::min(metrics_c_values_.size(), metrics_step_count);
  const std::size_t high_c_step_count =
      std::min(metrics_c66_coupled_values_.size(), metrics_step_count);
  struct HighCTrackingSubset {
    double threshold_metric = std::numeric_limits<double>::quiet_NaN();
    std::size_t samples = 0;
    double tracking_rms = std::numeric_limits<double>::quiet_NaN();
  };
  auto computeHighCTrackingSubset = [&](double percentile) {
    HighCTrackingSubset subset;
    subset.threshold_metric = computePercentile(metrics_c_values_, percentile);
    if (!std::isfinite(subset.threshold_metric)) {
      return subset;
    }
    double tracking_sum_sq = 0.0;
    std::size_t tracking_samples = 0;
    for (std::size_t idx = 0; idx < legacy_c_step_count; ++idx) {
      const double c_value = metrics_c_values_[idx];
      if (!std::isfinite(c_value) || c_value < subset.threshold_metric) {
        continue;
      }
      subset.samples += 1;
      if (std::isfinite(metrics_step_tracking_error_[idx])) {
        const double e = metrics_step_tracking_error_[idx];
        tracking_sum_sq += e * e;
        tracking_samples += 1;
      }
    }
    if (tracking_samples > 0) {
      subset.tracking_rms = std::sqrt(
          tracking_sum_sq / static_cast<double>(tracking_samples));
    }
    return subset;
  };
  const HighCTrackingSubset high_c44 = computeHighCTrackingSubset(0.44);
  summary.c44_metric = high_c44.threshold_metric;
  summary.high_c44_samples = high_c44.samples;
  summary.tracking_rms_high_c44 = high_c44.tracking_rms;
  summary.c66_metric = computePercentile(metrics_c66_coupled_values_, 0.66);
  summary.high_c_samples = 0;
  double high_c_min_h = std::numeric_limits<double>::infinity();
  double high_c_slack_sum = 0.0;
  double high_c_tracking_sum_sq = 0.0;
  std::size_t high_c_tracking_samples = 0;
  std::size_t high_c_failures = 0;
  if (std::isfinite(summary.c66_metric)) {
    for (std::size_t idx = 0; idx < high_c_step_count; ++idx) {
      const double c_value = metrics_c66_coupled_values_[idx];
      if (!std::isfinite(c_value) || c_value < summary.c66_metric) {
        continue;
      }
      summary.high_c_samples += 1;
      if (std::isfinite(metrics_step_min_h_[idx])) {
        high_c_min_h = std::min(high_c_min_h, metrics_step_min_h_[idx]);
      }
      if (std::isfinite(metrics_step_slack_[idx])) {
        high_c_slack_sum += metrics_step_slack_[idx];
      }
      if (std::isfinite(metrics_step_tracking_error_[idx])) {
        const double e = metrics_step_tracking_error_[idx];
        high_c_tracking_sum_sq += e * e;
        high_c_tracking_samples += 1;
      }
      high_c_failures +=
          (metrics_step_fail_flags_[idx] != 0) ? static_cast<std::size_t>(1) : 0;
    }
  }
  summary.min_h_high_c =
      (summary.high_c_samples > 0 && std::isfinite(high_c_min_h))
          ? high_c_min_h
          : std::numeric_limits<double>::quiet_NaN();
  summary.slack_sum_high_c =
      (summary.high_c_samples > 0)
          ? high_c_slack_sum
          : std::numeric_limits<double>::quiet_NaN();
  summary.tracking_rms_high_c =
      (high_c_tracking_samples > 0)
          ? std::sqrt(high_c_tracking_sum_sq /
                      static_cast<double>(high_c_tracking_samples))
          : std::numeric_limits<double>::quiet_NaN();
  const HighCTrackingSubset high_c88 = computeHighCTrackingSubset(0.88);
  summary.c88_metric = high_c88.threshold_metric;
  summary.high_c88_samples = high_c88.samples;
  summary.tracking_rms_high_c88 = high_c88.tracking_rms;
  summary.fail_rate_high_c =
      (summary.high_c_samples > 0)
          ? static_cast<double>(high_c_failures) /
                static_cast<double>(summary.high_c_samples)
          : std::numeric_limits<double>::quiet_NaN();
  summary.solver_min_h_high_c = summary.min_h_high_c;
  summary.solver_slack_sum_high_c = summary.slack_sum_high_c;
  summary.solver_fail_rate_high_c = summary.fail_rate_high_c;
  summary.solve_time_mean_ms = computeMean(metrics_solve_time_ms_);
  summary.solve_time_p95_ms = computeP95(metrics_solve_time_ms_);
  summary.solve_time_max_ms =
      metrics_solve_time_ms_.empty()
          ? std::numeric_limits<double>::quiet_NaN()
          : metrics_solve_time_max_ms_;
  summary.feedback_time_mean_ms = computeMean(metrics_feedback_time_ms_);
  summary.feedback_time_p95_ms = computeP95(metrics_feedback_time_ms_);
  summary.feedback_time_max_ms =
      metrics_feedback_time_ms_.empty()
          ? std::numeric_limits<double>::quiet_NaN()
          : metrics_feedback_time_max_ms_;
  summary.preparation_time_mean_ms = computeMean(metrics_preparation_time_ms_);
  summary.preparation_time_p95_ms = computeP95(metrics_preparation_time_ms_);
  summary.preparation_time_max_ms =
      metrics_preparation_time_ms_.empty()
          ? std::numeric_limits<double>::quiet_NaN()
          : metrics_preparation_time_max_ms_;
  summary.fail_rate =
      static_cast<double>(metrics_failures_) / static_cast<double>(metrics_samples_);
  summary.solver_fail_rate = summary.fail_rate;
  return summary;
}


}; // namespace coni_mpc
