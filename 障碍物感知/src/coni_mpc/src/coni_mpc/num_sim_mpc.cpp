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
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <std_msgs/ColorRGBA.h>

namespace coni_mpc
{

std::mutex NumSimMpc::shared_state_mutex_;
std::vector<Eigen::Vector3d> NumSimMpc::shared_positions_;
std::vector<Eigen::Vector3d> NumSimMpc::shared_velocities_;
std::vector<bool> NumSimMpc::shared_position_valid_;
NumSimMpc::ObstacleVector NumSimMpc::shared_static_obstacles_;
std::vector<double> NumSimMpc::shared_static_obstacle_heights_m_;

namespace {
std::string toLowerCopy(const std::string& text) {
  std::string out = text;
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

double maxEigenvalueSymmetric2d(const Eigen::Matrix2d& matrix) {
  const double trace = matrix.trace();
  const double discriminant = std::max(
      0.0, (matrix(0, 0) - matrix(1, 1)) *
                   (matrix(0, 0) - matrix(1, 1)) +
               4.0 * matrix(0, 1) * matrix(1, 0));
  return 0.5 * (trace + std::sqrt(discriminant));
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
  if (quad_id == 0) {
    return makeColor(1.0, 0.0, 0.0, alpha);
  }
  if (quad_id == 1) {
    return makeColor(0.0, 0.65, 0.2, alpha);
  }
  if (quad_id == 2) {
    return makeColor(0.0, 0.0, 1.0, alpha);
  }
  if (quad_id == 3) {
    return makeColor(1.0, 0.55, 0.0, alpha);
  }
  return makeColor(0.30, 0.30, 0.30, alpha);
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
    obstacle_frame_trajectory_window_(),
    uav_radius_(0.25),
    last_obstacles_(),
    cbf_use_in_sim_(true),
    frame_mode_(FrameMode::kNonInertial),
    frame_mode_effective_("noninertial"),
    safety_variant_("A2_soft_cbf"),
    zero_slack_required_(false),
    obstacle_sensing_mode_("map"),
    obstacle_sensing_online_(false),
    sensing_range_min_m_(0.15),
    sensing_range_max_m_(6.0),
    sensing_horizontal_fov_rad_(2.0 * M_PI),
    sensing_angular_resolution_rad_(0.9 * M_PI / 180.0),
    sensing_minimum_hit_rays_(3),
    sensing_control_rate_hz_(100.0),
    sensing_update_rate_hz_(10.0),
    sensing_position_std_m_(0.01),
    sensing_dropout_probability_(0.0),
    sensing_delay_sec_(0.1),
    sensing_hold_sec_(0.3),
    sensing_kf_process_variance_(1e-6),
    sensing_safety_clearance_threshold_m_(0.215),
    obstacle_sensing_tracker_(),
    sensing_step_stats_(),
    sensing_safety_snapshot_(),
    sensing_track_diagnostics_(),
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
    metrics_truth_static_min_surface_clearance_(
        std::numeric_limits<double>::infinity()),
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
  zero_slack_required_ =
      (safety_variant_ == "A0_no_cbf" || safety_variant_ == "A1_hard_cbf");
  int noise_seed = 1;
  pnh_.param("noise_seed", noise_seed, 1);
  if (noise_seed <= 0) {
    noise_seed = 1;
  }
  const uint32_t noise_seed_effective =
      static_cast<uint32_t>(noise_seed + std::max(0, quad_id) * 10007);
  noise_rng_.seed(noise_seed_effective);
  pnh_.param("obstacle_sensing/mode", obstacle_sensing_mode_,
             obstacle_sensing_mode_);
  obstacle_sensing_mode_ = toLowerCopy(obstacle_sensing_mode_);
  if (obstacle_sensing_mode_ == "nominal") {
    obstacle_sensing_mode_ = "online_nominal";
  } else if (obstacle_sensing_mode_ == "degraded") {
    obstacle_sensing_mode_ = "online_degraded";
  }
  if (obstacle_sensing_mode_ != "map" &&
      obstacle_sensing_mode_ != "online_nominal" &&
      obstacle_sensing_mode_ != "online_degraded") {
    ROS_WARN_STREAM("[" << pnh_.getNamespace()
                    << "] Unknown obstacle_sensing/mode='"
                    << obstacle_sensing_mode_
                    << "'. Falling back to map.");
    obstacle_sensing_mode_ = "map";
  }
  obstacle_sensing_online_ = obstacle_sensing_mode_ != "map";
  double horizontal_fov_deg = 360.0;
  double angular_resolution_deg = 0.9;
  int minimum_hit_rays = 3;
  pnh_.param("obstacle_sensing/range_min_m", sensing_range_min_m_, 0.15);
  pnh_.param("obstacle_sensing/range_max_m", sensing_range_max_m_, 6.0);
  pnh_.param("obstacle_sensing/horizontal_fov_deg", horizontal_fov_deg,
             360.0);
  pnh_.param("obstacle_sensing/angular_resolution_deg",
             angular_resolution_deg, 0.9);
  pnh_.param("obstacle_sensing/minimum_hit_rays", minimum_hit_rays, 3);
  pnh_.param("obstacle_sensing/control_rate_hz", sensing_control_rate_hz_,
             100.0);
  pnh_.param("obstacle_sensing/update_rate_hz", sensing_update_rate_hz_,
             10.0);
  pnh_.param("obstacle_sensing/kalman_process_variance",
             sensing_kf_process_variance_, 1e-6);
  pnh_.param("obstacle_sensing/safety_event_clearance_threshold_m",
             sensing_safety_clearance_threshold_m_, 0.215);
  sensing_range_min_m_ = std::max(0.0, sensing_range_min_m_);
  sensing_range_max_m_ =
      std::max(sensing_range_min_m_, sensing_range_max_m_);
  horizontal_fov_deg = std::max(0.0, std::min(360.0, horizontal_fov_deg));
  angular_resolution_deg = std::max(1e-6, angular_resolution_deg);
  minimum_hit_rays = std::max(1, minimum_hit_rays);
  sensing_horizontal_fov_rad_ = horizontal_fov_deg * M_PI / 180.0;
  sensing_angular_resolution_rad_ =
      angular_resolution_deg * M_PI / 180.0;
  sensing_minimum_hit_rays_ = static_cast<std::size_t>(minimum_hit_rays);
  sensing_control_rate_hz_ = std::max(1e-6, sensing_control_rate_hz_);
  sensing_update_rate_hz_ = std::max(1e-6, sensing_update_rate_hz_);
  sensing_kf_process_variance_ =
      std::max(0.0, sensing_kf_process_variance_);
  sensing_safety_clearance_threshold_m_ =
      std::max(0.0, sensing_safety_clearance_threshold_m_);
  if (obstacle_sensing_mode_ == "online_degraded") {
    pnh_.param("obstacle_sensing/degraded/position_std_m",
               sensing_position_std_m_, 0.05);
    pnh_.param("obstacle_sensing/degraded/dropout_probability",
               sensing_dropout_probability_, 0.10);
    pnh_.param("obstacle_sensing/degraded/delay_sec", sensing_delay_sec_,
               0.3);
    pnh_.param("obstacle_sensing/degraded/hold_sec", sensing_hold_sec_, 0.5);
  } else {
    pnh_.param("obstacle_sensing/nominal/position_std_m",
               sensing_position_std_m_, 0.01);
    pnh_.param("obstacle_sensing/nominal/dropout_probability",
               sensing_dropout_probability_, 0.0);
    pnh_.param("obstacle_sensing/nominal/delay_sec", sensing_delay_sec_,
               0.1);
    pnh_.param("obstacle_sensing/nominal/hold_sec", sensing_hold_sec_, 0.3);
  }
  sensing_position_std_m_ = std::max(0.0, sensing_position_std_m_);
  sensing_dropout_probability_ =
      std::max(0.0, std::min(1.0, sensing_dropout_probability_));
  sensing_delay_sec_ = std::max(0.0, sensing_delay_sec_);
  sensing_hold_sec_ = std::max(0.0, sensing_hold_sec_);
  ObstacleSensingTracker::Config sensing_config;
  sensing_config.online = obstacle_sensing_online_;
  sensing_config.range_min_m = sensing_range_min_m_;
  sensing_config.range_max_m = sensing_range_max_m_;
  sensing_config.horizontal_fov_rad = sensing_horizontal_fov_rad_;
  sensing_config.angular_resolution_rad = sensing_angular_resolution_rad_;
  sensing_config.minimum_hit_rays = sensing_minimum_hit_rays_;
  sensing_config.control_rate_hz = sensing_control_rate_hz_;
  sensing_config.scan_rate_hz = sensing_update_rate_hz_;
  sensing_config.position_std_m = sensing_position_std_m_;
  sensing_config.dropout_probability = sensing_dropout_probability_;
  sensing_config.delay_sec = sensing_delay_sec_;
  sensing_config.hold_sec = sensing_hold_sec_;
  sensing_config.kalman_process_variance = sensing_kf_process_variance_;
  sensing_config.experiment_seed = static_cast<std::uint64_t>(noise_seed);
  sensing_config.uav_id = static_cast<std::uint32_t>(std::max(0, quad_id));
  obstacle_sensing_tracker_.configure(sensing_config);
  pnh_.param("cbf/use_in_sim", cbf_use_in_sim_, mpc_params_.cbf_enabled_);
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
                  << " zero_slack_required=" << (zero_slack_required_ ? "true" : "false")
                  << " inter_uav_priority_enabled="
                  << (inter_uav_priority_enabled_ ? "true" : "false")
                  << " obstacle_hysteresis_hold_cycles="
                  << obstacle_hysteresis_hold_cycles_
                  << " obstacle_hysteresis_min_risk_improvement="
                  << obstacle_hysteresis_min_risk_improvement_
                  << " noise_seed_effective=" << noise_seed_effective
                  << " obstacle_sensing_mode=" << obstacle_sensing_mode_
                  << " sensing_range_min_m=" << sensing_range_min_m_
                  << " sensing_range_max_m=" << sensing_range_max_m_
                  << " sensing_horizontal_fov_deg=" << horizontal_fov_deg
                  << " sensing_angular_resolution_deg="
                  << angular_resolution_deg
                  << " sensing_minimum_hit_rays="
                  << sensing_minimum_hit_rays_
                  << " sensing_control_rate_hz=" << sensing_control_rate_hz_
                  << " sensing_update_rate_hz=" << sensing_update_rate_hz_
                  << " sensing_position_std_m=" << sensing_position_std_m_
                  << " sensing_delay_sec=" << sensing_delay_sec_
                  << " sensing_delay_control_cycles="
                  << obstacle_sensing_tracker_.delayControlCycles()
                  << " sensing_dropout_probability="
                  << sensing_dropout_probability_
                  << " sensing_hold_sec=" << sensing_hold_sec_
                  << " sensing_hold_control_cycles="
                  << obstacle_sensing_tracker_.holdControlCycles()
                  << " sensing_scan_period_control_cycles="
                  << obstacle_sensing_tracker_.scanPeriodControlCycles()
                  << " sensing_kf_process_variance="
                  << sensing_kf_process_variance_
                  << " sensing_safety_clearance_threshold_m="
                  << sensing_safety_clearance_threshold_m_
                  << " sensing_experiment_seed=" << noise_seed
                  << " sensing_uav_id=" << std::max(0, quad_id));
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

void NumSimMpc::getSharedPositions(std::vector<Eigen::Vector3d>& positions,
                                   std::vector<Eigen::Vector3d>& velocities,
                                   std::vector<bool>& valid)
{
  std::lock_guard<std::mutex> lock(shared_state_mutex_);
  positions = shared_positions_;
  velocities = shared_velocities_;
  valid = shared_position_valid_;
}

void NumSimMpc::setStaticObstacles(
    const ObstacleVector& obstacles,
    const std::vector<double>& obstacle_heights_m)
{
  std::lock_guard<std::mutex> lock(shared_state_mutex_);
  shared_static_obstacles_ = obstacles;
  if (obstacle_heights_m.size() == obstacles.size()) {
    shared_static_obstacle_heights_m_ = obstacle_heights_m;
  } else if (obstacles.empty()) {
    shared_static_obstacle_heights_m_.clear();
  } else if (shared_static_obstacle_heights_m_.size() != obstacles.size()) {
    // Missing heights retain the legacy infinite-cylinder visibility model.
    shared_static_obstacle_heights_m_.clear();
  }
}

NumSimMpc::ObstacleVector NumSimMpc::buildAvailableStaticObstacles(
    const ObstacleVector& truth_obstacles,
    const std::vector<double>& obstacle_heights_m,
    std::vector<std::string>& obstacle_keys) {
  Eigen::Quaterniond world_q_body(
      quad_odom_.pose.pose.orientation.w,
      quad_odom_.pose.pose.orientation.x,
      quad_odom_.pose.pose.orientation.y,
      quad_odom_.pose.pose.orientation.z);
  if (world_q_body.norm() > 1e-9) {
    world_q_body.normalize();
  } else {
    world_q_body = Eigen::Quaterniond::Identity();
  }
  const double body_yaw = std::atan2(
      2.0 * (world_q_body.w() * world_q_body.z() +
             world_q_body.x() * world_q_body.y()),
      1.0 - 2.0 * (world_q_body.y() * world_q_body.y() +
                   world_q_body.z() * world_q_body.z()));
  const Eigen::Vector3d uav_position_world(
      quad_odom_.pose.pose.position.x,
      quad_odom_.pose.pose.position.y,
      quad_odom_.pose.pose.position.z);
  const std::uint64_t current_cycle = obstacle_sensing_tracker_.cycle();
  ObstacleSensingTracker::StepResult sensing_result =
      obstacle_sensing_tracker_.step(truth_obstacles, uav_position_world,
                                     body_yaw, obstacle_heights_m);
  sensing_step_stats_ = sensing_result.stats;

  sensing_track_diagnostics_.clear();
  sensing_track_diagnostics_.reserve(sensing_result.tracks.size());
  for (const auto& track : sensing_result.tracks) {
    if (track.obstacle_id >= truth_obstacles.size()) {
      continue;
    }
    const Obstacle& truth = truth_obstacles[track.obstacle_id];
    SensingTrackDiagnostic diagnostic;
    diagnostic.valid = true;
    diagnostic.tracker_online = obstacle_sensing_online_;
    diagnostic.obstacle_id = track.obstacle_id;
    diagnostic.obstacle_key = "static:" + std::to_string(track.obstacle_id);
    diagnostic.truth_position_world = truth.head<2>();
    diagnostic.track_position_world = track.position;
    diagnostic.truth_z = truth(2);
    diagnostic.track_z = track.z;
    diagnostic.truth_radius = truth(3);
    diagnostic.track_radius = track.radius;
    diagnostic.track_center_error_m =
        (track.position - truth.head<2>()).norm();
    diagnostic.truth_surface_clearance_m =
        (uav_position_world.head<2>() - truth.head<2>()).norm() -
        uav_radius_ - truth(3);
    diagnostic.track_surface_clearance_m =
        (uav_position_world.head<2>() - track.position).norm() -
        uav_radius_ - track.radius;
    diagnostic.track_surface_error_m =
        diagnostic.track_surface_clearance_m -
        diagnostic.truth_surface_clearance_m;

    if (obstacle_sensing_online_) {
      diagnostic.last_measurement_position_world =
          track.last_measurement_position;
      diagnostic.last_measurement_error_m =
          (track.last_measurement_position - truth.head<2>()).norm();
      diagnostic.covariance = track.covariance;
      diagnostic.covariance_trace = track.covariance.trace();
      diagnostic.covariance_max_eigenvalue =
          maxEigenvalueSymmetric2d(track.covariance);
      diagnostic.innovation_valid = track.last_innovation_valid;
      if (track.last_innovation_valid) {
        diagnostic.innovation = track.last_innovation;
        diagnostic.innovation_norm_m = track.last_innovation.norm();
        diagnostic.innovation_covariance_trace =
            track.last_innovation_covariance.trace();
        diagnostic.kalman_gain_trace = track.last_kalman_gain.trace();
      }
      diagnostic.generation_scan_index = static_cast<std::int64_t>(
          track.last_generation_scan_index);
      diagnostic.generation_cycle =
          static_cast<std::int64_t>(track.last_generation_cycle);
      diagnostic.delivery_cycle =
          static_cast<std::int64_t>(track.last_delivery_cycle);
      diagnostic.update_cycle =
          static_cast<std::int64_t>(track.last_update_cycle);
      diagnostic.measurement_age_frames = static_cast<std::int64_t>(
          current_cycle - track.last_generation_cycle);
      diagnostic.measurement_age_sec =
          static_cast<double>(diagnostic.measurement_age_frames) /
          sensing_control_rate_hz_;
      diagnostic.cycles_since_delivery = static_cast<std::int64_t>(
          current_cycle - track.last_delivery_cycle);
      diagnostic.sec_since_delivery =
          static_cast<double>(diagnostic.cycles_since_delivery) /
          sensing_control_rate_hz_;
      diagnostic.imposed_delay_frames = static_cast<std::int64_t>(
          track.last_delivery_cycle - track.last_generation_cycle);
      diagnostic.imposed_delay_sec =
          static_cast<double>(diagnostic.imposed_delay_frames) /
          sensing_control_rate_hz_;
      diagnostic.track_instance = track.track_instance;
      diagnostic.measurement_update_count = track.measurement_update_count;
      diagnostic.created_this_cycle = track.created_this_cycle;
      diagnostic.updated_this_cycle = track.updated_this_cycle;
      diagnostic.reinitialized_this_cycle = track.reinitialized_this_cycle;
    }
    sensing_track_diagnostics_.push_back(diagnostic);
  }

  sensing_safety_snapshot_ = SensingSafetySnapshot();
  sensing_safety_snapshot_.clearance_threshold_m =
      sensing_safety_clearance_threshold_m_;
  if (!truth_obstacles.empty()) {
    const Eigen::Vector2d uav_xy = uav_position_world.head<2>();
    double nearest_clearance = std::numeric_limits<double>::infinity();
    std::size_t nearest_id = 0;
    for (std::size_t obstacle_id = 0; obstacle_id < truth_obstacles.size();
         ++obstacle_id) {
      const Obstacle& truth = truth_obstacles[obstacle_id];
      const double clearance =
          (uav_xy - truth.head<2>()).norm() - uav_radius_ - truth(3);
      if (clearance < nearest_clearance) {
        nearest_clearance = clearance;
        nearest_id = obstacle_id;
      }
    }

    const auto contains_id = [nearest_id](const std::vector<std::size_t>& ids) {
      return std::find(ids.begin(), ids.end(), nearest_id) != ids.end();
    };
    const auto track_it = std::find_if(
        sensing_result.tracks.begin(), sensing_result.tracks.end(),
        [nearest_id](const ObstacleSensingTracker::TrackSnapshot& track) {
          return track.obstacle_id == nearest_id;
        });

    sensing_safety_snapshot_.nearest_truth_obstacle_id =
        static_cast<int>(nearest_id);
    sensing_safety_snapshot_.nearest_truth_clearance_m = nearest_clearance;
    sensing_safety_snapshot_.nearest_truth_in_sensor_fov =
        contains_id(sensing_result.visible_obstacle_ids);
    sensing_safety_snapshot_.nearest_truth_current_dropout =
        contains_id(sensing_result.dropped_obstacle_ids);
    sensing_safety_snapshot_.nearest_truth_pending_measurement =
        contains_id(sensing_result.pending_obstacle_ids);
    sensing_safety_snapshot_.nearest_truth_track_active =
        track_it != sensing_result.tracks.end();
    const auto diagnostic_it = std::find_if(
        sensing_track_diagnostics_.begin(), sensing_track_diagnostics_.end(),
        [nearest_id](const SensingTrackDiagnostic& diagnostic) {
          return diagnostic.obstacle_id == nearest_id;
        });
    if (diagnostic_it != sensing_track_diagnostics_.end() &&
        diagnostic_it->tracker_online) {
      const std::int64_t age_frames =
          diagnostic_it->measurement_age_frames;
      sensing_safety_snapshot_.nearest_truth_track_age_frames =
          age_frames;
      sensing_safety_snapshot_.nearest_truth_track_age_sec =
          diagnostic_it->measurement_age_sec;
      sensing_safety_snapshot_.nearest_truth_cycles_since_delivery =
          diagnostic_it->cycles_since_delivery;
      sensing_safety_snapshot_.nearest_truth_sec_since_delivery =
          diagnostic_it->sec_since_delivery;
      sensing_safety_snapshot_.nearest_truth_imposed_delay_frames =
          diagnostic_it->imposed_delay_frames;
      sensing_safety_snapshot_.nearest_truth_imposed_delay_sec =
          diagnostic_it->imposed_delay_sec;
      sensing_safety_snapshot_.nearest_truth_last_measurement_error_m =
          diagnostic_it->last_measurement_error_m;
      sensing_safety_snapshot_.nearest_truth_track_center_error_m =
          diagnostic_it->track_center_error_m;
      sensing_safety_snapshot_.nearest_truth_track_covariance_trace =
          diagnostic_it->covariance_trace;
      sensing_safety_snapshot_.nearest_truth_track_innovation_norm_m =
          diagnostic_it->innovation_norm_m;
      sensing_safety_snapshot_.nearest_truth_track_instance =
          static_cast<std::int64_t>(diagnostic_it->track_instance);
      sensing_safety_snapshot_.nearest_truth_track_reinitialized_this_cycle =
          diagnostic_it->reinitialized_this_cycle;
    }

    const bool low_clearance =
        nearest_clearance < sensing_safety_clearance_threshold_m_;
    const bool track_active =
        sensing_safety_snapshot_.nearest_truth_track_active;
    sensing_safety_snapshot_.low_clearance_sample = low_clearance;
    sensing_safety_snapshot_.low_clearance_no_track_sample =
        low_clearance && !track_active;
    sensing_safety_snapshot_.low_clearance_current_dropout_sample =
        low_clearance &&
        sensing_safety_snapshot_.nearest_truth_current_dropout;
    sensing_safety_snapshot_.low_clearance_pending_no_track_sample =
        low_clearance && !track_active &&
        sensing_safety_snapshot_.nearest_truth_pending_measurement;
    sensing_safety_snapshot_.low_clearance_stale_track_sample =
        low_clearance && track_active &&
        sensing_safety_snapshot_.nearest_truth_track_age_frames > 0;
  }
  obstacle_keys = std::move(sensing_result.obstacle_keys);
  ROS_INFO_STREAM_THROTTLE(
      1.0,
      "[SENSING UAV " << quad_id << "] mode=" << obstacle_sensing_mode_
                       << " cycle=" << current_cycle
                       << " scan_triggered="
                       << (sensing_step_stats_.scan_triggered ? "true" : "false")
                       << " scan_index=" << sensing_step_stats_.scan_index
                       << " cast_rays=" << sensing_step_stats_.cast_rays
                       << " truth=" << sensing_step_stats_.truth_obstacles
                       << " visible=" << sensing_step_stats_.visible_obstacles
                       << " generated="
                       << sensing_step_stats_.generated_measurements
                       << " dropped="
                       << sensing_step_stats_.dropped_measurements
                       << " delivered="
                       << sensing_step_stats_.delivered_measurements
                       << " created=" << sensing_step_stats_.created_tracks
                       << " reinitialized="
                       << sensing_step_stats_.reinitialized_tracks
                       << " corrected="
                       << sensing_step_stats_.kalman_corrections
                       << " deleted=" << sensing_step_stats_.deleted_tracks
                       << " active=" << sensing_step_stats_.active_tracks);
  return sensing_result.obstacles;
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
  const CarTrajectoryWindow& obstacle_frame_window =
      obstacle_frame_trajectory_window_.empty()
          ? car_trajectory_window_
          : obstacle_frame_trajectory_window_;
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
  // Publish a colored line strip marker for the quad path with gradient green->blue.
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
      // Path colors are intentionally permuted against body colors for the paper screenshots.
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

  ObstacleVector truth_static_obstacles;
  std::vector<double> truth_static_obstacle_heights_m;
  {
    std::lock_guard<std::mutex> lock(shared_state_mutex_);
    truth_static_obstacles = shared_static_obstacles_;
    truth_static_obstacle_heights_m = shared_static_obstacle_heights_m_;
  }
  std::vector<std::string> obstacle_keys;
  ObstacleVector obstacles = buildAvailableStaticObstacles(
      truth_static_obstacles, truth_static_obstacle_heights_m, obstacle_keys);
  obstacle_keys.reserve(obstacles.size() + static_cast<std::size_t>(num_uavs));

  double step_truth_static_min_surface_clearance =
      std::numeric_limits<double>::quiet_NaN();
  if (!truth_static_obstacles.empty()) {
    const Eigen::Vector2d truth_uav_xy(
        quad_odom_.pose.pose.position.x,
        quad_odom_.pose.pose.position.y);
    double min_clearance = std::numeric_limits<double>::infinity();
    for (const auto& truth_obstacle : truth_static_obstacles) {
      const double clearance =
          (truth_uav_xy - truth_obstacle.head<2>()).norm() -
          uav_radius_ - truth_obstacle(3);
      min_clearance = std::min(min_clearance, clearance);
    }
    step_truth_static_min_surface_clearance = min_clearance;
    metrics_truth_static_min_surface_clearance_ = std::min(
        metrics_truth_static_min_surface_clearance_, min_clearance);
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
  if (!obstacles.empty()) {
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
          (k < obstacle_frame_window.size()) ? obstacle_frame_window[k]
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
          (k < obstacle_frame_window.size()) ? obstacle_frame_window[k]
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
    // metrics are evaluated separately on the selected obstacles passed to ACADO.
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
  const bool cbf_active = enable_cbf && !mpc_obstacles.empty();
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
  auto command = mpc_controller_.run(
      state_estimate_, reference_window_, mpc_params_, world_q_model,
      world_p_model, omega_profile_arg, beta_profile_arg,
      a_car_profile_arg);
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
          (k < obstacle_frame_window.size()) ? obstacle_frame_window[k]
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
  if (zero_slack_required_ && std::abs(latest_slack) > 1e-9) {
    ROS_FATAL_STREAM("[" << pnh_.getNamespace() << "] safety_variant="
                     << safety_variant_ << " requires zero slack but got "
                     << latest_slack << ". Aborting.");
    std::abort();
  }
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
  for (auto& diagnostic : sensing_track_diagnostics_) {
    diagnostic.selected_for_hocbf =
        std::find(selected_keys.begin(), selected_keys.end(),
                  diagnostic.obstacle_key) != selected_keys.end();
    diagnostic.solver_active =
        effective_solver_active_obstacle_key == diagnostic.obstacle_key;
  }
  StepDebugSnapshot current_step_debug;
  current_step_debug.valid = true;
  current_step_debug.cbf_active = cbf_active;
  current_step_debug.solver_cbf_active = solver_cbf_active;
  current_step_debug.solve_ok = mpc_controller_.getLastSolveOk();
  current_step_debug.solver_fail_flag = !current_step_debug.solve_ok;
  current_step_debug.turning = turning;
  current_step_debug.obstacle_sensing_mode = obstacle_sensing_mode_;
  current_step_debug.sensing_scan_triggered =
      sensing_step_stats_.scan_triggered;
  current_step_debug.sensing_scan_index = sensing_step_stats_.scan_index;
  current_step_debug.sensing_cast_rays = sensing_step_stats_.cast_rays;
  current_step_debug.sensing_truth_obstacles =
      sensing_step_stats_.truth_obstacles;
  current_step_debug.sensing_visible_obstacles =
      sensing_step_stats_.visible_obstacles;
  current_step_debug.sensing_generated_measurements =
      sensing_step_stats_.generated_measurements;
  current_step_debug.sensing_dropped_measurements =
      sensing_step_stats_.dropped_measurements;
  current_step_debug.sensing_delivered_measurements =
      sensing_step_stats_.delivered_measurements;
  current_step_debug.sensing_created_tracks =
      sensing_step_stats_.created_tracks;
  current_step_debug.sensing_reinitialized_tracks =
      sensing_step_stats_.reinitialized_tracks;
  current_step_debug.sensing_kalman_corrections =
      sensing_step_stats_.kalman_corrections;
  current_step_debug.sensing_deleted_tracks =
      sensing_step_stats_.deleted_tracks;
  current_step_debug.sensing_active_tracks =
      sensing_step_stats_.active_tracks;
  current_step_debug.truth_static_min_surface_clearance =
      step_truth_static_min_surface_clearance;
  current_step_debug.sensing_safety = sensing_safety_snapshot_;
  current_step_debug.sensing_tracks = sensing_track_diagnostics_;
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
  const Eigen::Vector3d estimated_uav_position_world =
      isInertialFrame()
          ? state_estimate_.position
          : car_position_world + W_q_non * state_estimate_.position;
  current_step_debug.truth_uav_position_world = quad_position_world;
  current_step_debug.estimated_uav_position_world =
      estimated_uav_position_world;
  current_step_debug.uav_position_error_m =
      (estimated_uav_position_world - quad_position_world).norm();
  const auto solver_track_it = std::find_if(
      current_step_debug.sensing_tracks.begin(),
      current_step_debug.sensing_tracks.end(),
      [](const SensingTrackDiagnostic& diagnostic) {
        return diagnostic.solver_active;
      });
  if (solver_track_it != current_step_debug.sensing_tracks.end()) {
    current_step_debug.solver_active_static_track = *solver_track_it;
    if (std::isfinite(step_solver_planar_surface_distance)) {
      current_step_debug.solver_surface_error_vs_track_m =
          step_solver_planar_surface_distance -
          solver_track_it->track_surface_clearance_m;
      current_step_debug.solver_surface_error_vs_truth_m =
          step_solver_planar_surface_distance -
          solver_track_it->truth_surface_clearance_m;
    }
  }
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
  metrics_truth_static_min_surface_clearance_ =
      std::numeric_limits<double>::infinity();
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
  sensing_track_diagnostics_.clear();
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
    summary.truth_static_min_surface_clearance =
        std::numeric_limits<double>::quiet_NaN();
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
  summary.truth_static_min_surface_clearance =
      std::isfinite(metrics_truth_static_min_surface_clearance_)
          ? metrics_truth_static_min_surface_clearance_
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
