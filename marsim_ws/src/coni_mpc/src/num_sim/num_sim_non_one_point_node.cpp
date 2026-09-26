/*
 * @Author: Baozhe ZHANG 
 * @Date: 2023-05-22 12:46:41 
 * @Last Modified by: Baozhe ZHANG
 * @Last Modified time: 2023-05-22 13:50:20
 */

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
#include "num_sim/Simulator.h"
#include "num_sim/System.h"
#include "acado_mpc/mpc_controller.h"
#include "acado_mpc/mpc_common.h"
#include "coni_mpc/common.hpp"
#include "coni_mpc/non_inertial_predictor.hpp"
#include "coni_mpc/num_sim_mpc.h"
#include "num_sim/polytopic_vo_hrvo_filter.h"
#include <array>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cctype>
#include <chrono>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <cmath>
#include <random>
#include <queue>
#include <boost/uuid/detail/sha1.hpp>
#include <ros/ros.h>
#include <gflags/gflags.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


using namespace num_sim;
using namespace acado_mpc_common;

DEFINE_double(r, 0.0, "R value");
DEFINE_double(v, 0.0, "V value");
DEFINE_double(w, 0.01, "W value");

const double DEFAULT_START_Z = 2.0;
const double DURATION = 125;
int num_uavs=4;

struct StaticObstacle {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  double radius;
  double height;
};

// Collision monitor for the active MARSIM/LiDAR point-cloud scene.  The
// point cloud is expected to be expressed in the same world frame as the
// simulator (normally `map`).  This monitor is deliberately separate from
// the DSP occupancy prediction: prediction drives avoidance, while this
// point cloud is only a terminal physical-contact check.
class LidarCollisionMonitor {
 public:
  void configure(ros::NodeHandle& nh,
                 bool enabled,
                 const std::string& topic,
                 const std::string& expected_frame,
                 double timeout_sec,
                 double point_margin) {
    enabled_ = enabled && !topic.empty();
    topic_ = topic;
    expected_frame_ = expected_frame;
    timeout_sec_ = std::max(0.0, timeout_sec);
    point_margin_ = std::max(0.0, point_margin);
    if (!enabled_) {
      if (enabled && topic.empty()) {
        ROS_WARN("LiDAR collision monitor requested but cloud_topic is empty; disabled.");
      }
      return;
    }
    subscriber_ = nh.subscribe(topic_, 1, &LidarCollisionMonitor::callback, this);
    ROS_INFO_STREAM("LiDAR collision monitor enabled: topic=" << topic_
                    << " frame=" << expected_frame_
                    << " timeout=" << timeout_sec_ << " s"
                    << " point_margin=" << point_margin_ << " m");
  }

  bool enabled() const { return enabled_; }

  bool nearestPoint(const Eigen::Vector3d& position,
                    double& distance,
                    Eigen::Vector3d& point) const {
    if (!enabled_ || !has_cloud_ || !position.allFinite()) {
      return false;
    }
    if (timeout_sec_ > 0.0 &&
        (ros::WallTime::now() - last_receipt_wall_).toSec() > timeout_sec_) {
      return false;
    }
    pcl::PointXYZ query;
    query.x = static_cast<float>(position.x());
    query.y = static_cast<float>(position.y());
    query.z = static_cast<float>(position.z());
    std::vector<int> indices;
    std::vector<float> squared_distances;
    if (kdtree_.nearestKSearch(query, 1, indices, squared_distances) != 1 ||
        squared_distances.empty() || indices.empty() ||
        !std::isfinite(squared_distances.front()) ||
        squared_distances.front() < 0.0f) {
      return false;
    }
    const pcl::PointXYZ& nearest = cloud_->points[indices.front()];
    distance = std::sqrt(static_cast<double>(squared_distances.front()));
    point = Eigen::Vector3d(nearest.x, nearest.y, nearest.z);
    return point.allFinite() && std::isfinite(distance);
  }

  double pointMargin() const { return point_margin_; }

 private:
  void callback(const sensor_msgs::PointCloud2ConstPtr& message) {
    if (!message) {
      return;
    }
    const std::string frame = message->header.frame_id;
    if (!expected_frame_.empty() && frame != expected_frame_) {
      has_cloud_ = false;
      ROS_WARN_THROTTLE(2.0,
                        "LiDAR collision cloud frame '%s' does not match expected world frame '%s'; ignoring it.",
                        frame.c_str(), expected_frame_.c_str());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> converted;
    try {
      pcl::fromROSMsg(*message, converted);
    } catch (const std::exception& error) {
      has_cloud_ = false;
      ROS_WARN_THROTTLE(2.0, "Failed to decode LiDAR collision cloud: %s",
                        error.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
        new pcl::PointCloud<pcl::PointXYZ>());
    filtered->reserve(converted.points.size());
    for (const pcl::PointXYZ& candidate : converted.points) {
      if (std::isfinite(candidate.x) && std::isfinite(candidate.y) &&
          std::isfinite(candidate.z)) {
        filtered->push_back(candidate);
      }
    }
    filtered->width = static_cast<std::uint32_t>(filtered->points.size());
    filtered->height = 1;
    filtered->is_dense = true;
    cloud_ = filtered;
    if (cloud_->empty()) {
      has_cloud_ = false;
      return;
    }
    kdtree_.setInputCloud(cloud_);
    last_receipt_wall_ = ros::WallTime::now();
    has_cloud_ = true;
  }

  bool enabled_ = false;
  bool has_cloud_ = false;
  std::string topic_;
  std::string expected_frame_;
  double timeout_sec_ = 0.5;
  double point_margin_ = 0.0;
  ros::Subscriber subscriber_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{
      new pcl::PointCloud<pcl::PointXYZ>()};
  mutable pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  ros::WallTime last_receipt_wall_;
};

struct RandomObstacleConfig {
  int count;
  double x_min;
  double x_max;
  double y_min;
  double y_max;
  double h_min;
  double h_max;
  double radius;
  int seed;
};

struct UgvPathPlannerConfig {
  int a_star_connectivity;
  bool use_midpoint_guide;
  bool use_last_guide_as_goal;
  bool enable_shortcut_pruning;
  bool enable_endpoint_snapping;
  double longitudinal_accel_max;
  double longitudinal_decel_max;
  double longitudinal_jerk_max;
  double yaw_accel_max;
  double yaw_jerk_max;
  double speed_tracking_tau;
  double yaw_rate_tracking_tau;
  double along_track_feedback_gain;
  double heading_feedback_gain;
  double lateral_feedback_gain;
  std::vector<Eigen::Vector2d> guide_waypoints;
};

struct NonInertialPredictorConfig {
  int warmup_frames = 5;
  double ema_alpha_min = 0.05;
  double ema_alpha_max = 0.6;
  double var_threshold = 0.5;
  double gamma_a = 0.85;
  double gamma_alpha = 0.85;
  double speed_max = 0.0;
  double omega_z_max = 0.0;
  double a_max = 10.0;
  double alpha_max = 10.0;
  int exit_history_size = 8;
  std::string trend_mode = "decay_only";
  coni_mpc::AdaptiveTrendGainConfig long_trend;
  coni_mpc::AdaptiveTrendGainConfig angular_trend;
  bool debug_log = false;
  double turn_exit_omega_hold = 0.2;
  double turn_exit_omega_low = 0.05;
  double accel_exit_a_hold = 0.3;
  double accel_exit_a_low = 0.1;
  double cruise_speed_ratio = 0.92;
  double stop_speed_threshold = 0.1;
  coni_mpc::PredictorParams damped_rollout;
};

struct ExternalAvoidanceConfig {
  std::string mode = "none";
  double time_horizon = 2.0;
  std::size_t max_neighbors = 0u;
  double safety_margin = 0.0;
  std::size_t polygon_sides = 16u;
  double velocity_window = 0.5;
  double candidate_resolution = 0.05;
  double fallback_penalty_weight = 4.0;
  std::string preferred_velocity_mode = "slot_velocity_ff";
  double goal_gain = 1.0;
  double velocity_compensation_tau_sec = 0.2;

  bool enabled() const { return mode == "polytopic_hrvo"; }
};

static bool fileHasData(const std::string& path) {
  std::ifstream in(path.c_str(), std::ios::in);
  if (!in.good()) {
    return false;
  }
  return in.peek() != std::ifstream::traits_type::eof();
}

static std::string fmtDouble(double value) {
  if (!std::isfinite(value)) {
    return "nan";
  }
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss << std::setprecision(6) << value;
  return ss.str();
}

static Eigen::Vector2d clampPlanarVelocity(const Eigen::Vector2d& velocity,
                                           double max_speed) {
  if (max_speed <= 1.0e-9 || !velocity.allFinite()) {
    return velocity.allFinite() ? velocity : Eigen::Vector2d::Zero();
  }
  const double norm = velocity.norm();
  if (norm <= max_speed || norm <= 1.0e-12) {
    return velocity;
  }
  return velocity * (max_speed / norm);
}

static std::string joinIntVector(const std::vector<int>& values) {
  std::ostringstream ss;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      ss << ";";
    }
    ss << values[i];
  }
  return ss.str();
}

static std::vector<int> filterAggregateUavs(const std::vector<int>& active_uavs,
                                            bool exclude_uav0_from_all_metrics) {
  if (!exclude_uav0_from_all_metrics) {
    return active_uavs;
  }
  std::vector<int> filtered;
  filtered.reserve(active_uavs.size());
  for (int idx : active_uavs) {
    if (idx == 0) {
      continue;
    }
    filtered.push_back(idx);
  }
  return filtered;
}

static std::vector<int> buildDefaultActiveUavs(int vehicle_count,
                                               bool exclude_uav0_from_simulation) {
  std::vector<int> active_uavs;
  active_uavs.reserve(static_cast<size_t>(std::max(0, vehicle_count)));
  const int start_idx =
      (exclude_uav0_from_simulation && vehicle_count > 1) ? 1 : 0;
  for (int idx = start_idx; idx < vehicle_count; ++idx) {
    active_uavs.push_back(idx);
  }
  if (active_uavs.empty() && vehicle_count > 0) {
    active_uavs.push_back(0);
  }
  return active_uavs;
}

template <typename T>
static std::vector<T> selectActiveEntries(const std::vector<T>& values,
                                          const std::vector<int>& active_uavs) {
  std::vector<T> selected;
  selected.reserve(active_uavs.size());
  for (int idx : active_uavs) {
    if (idx < 0 || idx >= static_cast<int>(values.size())) {
      continue;
    }
    selected.push_back(values[static_cast<size_t>(idx)]);
  }
  return selected;
}

static std::string sha1Hex(const std::string& text) {
  boost::uuids::detail::sha1 sha1;
  sha1.process_bytes(text.data(), text.size());
  unsigned int digest[5] = {0u, 0u, 0u, 0u, 0u};
  sha1.get_digest(digest);
  std::ostringstream ss;
  ss << std::hex << std::setfill('0');
  for (unsigned int word : digest) {
    ss << std::setw(2) << ((word >> 24) & 0xffu)
       << std::setw(2) << ((word >> 16) & 0xffu)
       << std::setw(2) << ((word >> 8) & 0xffu)
       << std::setw(2) << (word & 0xffu);
  }
  return ss.str();
}

static std::string toLowerCopy(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

static std::string resolveFrameModeEffective(const ros::NodeHandle& pnh) {
  bool use_inertial_frame = false;
  bool use_noninertial_frame = true;
  std::string frame_mode;
  pnh.param("use_inertial_frame", use_inertial_frame, false);
  pnh.param("use_noninertial_frame", use_noninertial_frame, true);
  pnh.param("frame_mode", frame_mode, std::string(""));
  const std::string mode = toLowerCopy(frame_mode);
  if (mode == "inertial") {
    return "inertial";
  }
  if (mode == "noninertial" || mode == "non-inertial" || mode == "non_inertial") {
    return "noninertial";
  }
  if (use_inertial_frame && !use_noninertial_frame) {
    return "inertial";
  }
  return "noninertial";
}

static std::string resolveUgvRolloutMode(const ros::NodeHandle& pnh) {
  std::string ugv_rollout_mode;
  pnh.param("ugv_rollout_mode", ugv_rollout_mode, std::string("stage"));
  ugv_rollout_mode = toLowerCopy(ugv_rollout_mode);
  if (ugv_rollout_mode == "stage") {
    return "stage";
  }
  if (ugv_rollout_mode == "frozen" ||
      ugv_rollout_mode == "freeze" ||
      ugv_rollout_mode == "constant") {
    return "frozen";
  }
  ROS_WARN_STREAM("Unsupported ugv_rollout_mode=" << ugv_rollout_mode
                  << ", falling back to 'stage'.");
  return "stage";
}

static std::string controllerDisplayName(const std::string& frame_mode_effective,
                                         const std::string& ugv_rollout_mode) {
  if (ugv_rollout_mode == "stage") {
    return "stage";
  }
  if (frame_mode_effective == "inertial" && ugv_rollout_mode == "frozen") {
    return "i-frozen";
  }
  if (frame_mode_effective == "noninertial" && ugv_rollout_mode == "frozen") {
    return "ni-frozen";
  }
  return frame_mode_effective + "/" + ugv_rollout_mode;
}

static void publishRelativeFrameTf(const nav_msgs::Odometry& car_odom) {
  static tf2_ros::TransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = car_odom.header.stamp;
  tf_msg.header.frame_id = acado_mpc_common::worldFrameId();
  tf_msg.child_frame_id = acado_mpc_common::relativeFrameId();
  tf_msg.transform.translation.x = car_odom.pose.pose.position.x;
  tf_msg.transform.translation.y = car_odom.pose.pose.position.y;
  tf_msg.transform.translation.z = car_odom.pose.pose.position.z;
  tf_msg.transform.rotation = car_odom.pose.pose.orientation;
  tf_broadcaster.sendTransform(tf_msg);
}

static std::string uavTfFrameId(int uav_idx) {
  return "uav_" + std::to_string(uav_idx);
}

static void publishUavTf(const nav_msgs::Odometry& quad_odom, int uav_idx) {
  static tf2_ros::TransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = quad_odom.header.stamp;
  tf_msg.header.frame_id = acado_mpc_common::worldFrameId();
  tf_msg.child_frame_id = uavTfFrameId(uav_idx);
  tf_msg.transform.translation.x = quad_odom.pose.pose.position.x;
  tf_msg.transform.translation.y = quad_odom.pose.pose.position.y;
  tf_msg.transform.translation.z = quad_odom.pose.pose.position.z;
  tf_msg.transform.rotation = quad_odom.pose.pose.orientation;
  tf_broadcaster.sendTransform(tf_msg);
}

static double computeP95FromValues(const std::vector<double>& values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::vector<double> tmp(values.begin(), values.end());
  std::sort(tmp.begin(), tmp.end());
  const double rank = 0.95 * static_cast<double>(tmp.size() - 1);
  const size_t idx = static_cast<size_t>(std::ceil(rank));
  return tmp[idx];
}

static void writeMetricsHeader(std::ostream& out) {
  out << "run_tag,scope,frame_mode_effective,ugv_rollout_mode,safety_variant,obstacle_count,seed,seed_env,seed_traj,traj_id,kappa,"
         "requested_r,requested_v,requested_w,use_dynamic_obs,warm_start,start_z,noise_seed,active_uavs_csv,"
         "sim_control_dt_sec,prediction_dt_sec,prediction_steps,max_abs_ugv_yaw_rate,turning_omega_thr,"
         "metrics_debug,simulation_duration_sec,wall_time_sec,collision,cbf_samples,"
         "min_h,mean_h,min_cbf,mean_cbf,max_slack,tracking_rms,tracking_rms_x,tracking_rms_y,tracking_rms_z,tracking_p95,tracking_rms_high,"
         "tracking_rms_track_only,tracking_rms_track_only_x,tracking_rms_track_only_y,tracking_rms_track_only_z,"
         "tracking_p95_track_only,tracking_rms_cbf_active,cbf_active_samples,cbf_active_ratio,"
         "track_only_samples,track_only_ratio,tracking_p95_worst_uav,"
         "fail_rate_turn,fail_rate_straight,min_h_turn,min_h_straight,slack_sum_turn,slack_sum_straight,"
         "max_slack_turn,max_slack_straight,tracking_rms_all_turn,tracking_rms_all_straight,turn_samples,straight_samples,"
         "c44_metric,high_c44_samples,tracking_rms_high_c44,"
         "c66_metric,high_c_samples,min_h_high_c,slack_sum_high_c,tracking_rms_high_c,fail_rate_high_c,"
         "c88_metric,high_c88_samples,tracking_rms_high_c88,"
         "dmin_lin,slack_sum,solve_time_mean_ms,solve_time_p95_ms,solve_time_p95_worst_uav,"
         "solve_time_max_ms,feedback_time_mean_ms,feedback_time_p95_ms,feedback_time_max_ms,"
         "preparation_time_mean_ms,preparation_time_p95_ms,preparation_time_max_ms,fail_rate,"
         "solver_tracking_rms_avoid,solver_tracking_rms_avoid_samples,solver_min_h,solver_active_mean_h,solver_min_planar_clearance,"
         "solver_min_h_witness_h,solver_min_h_witness_planar_clearance,solver_min_h_witness_tracking_error,"
         "solver_min_h_witness_delta,solver_min_h_witness_planar_distance,solver_min_h_witness_active_obstacle_key,"
         "solver_mean_h,solver_min_cbf,solver_mean_cbf,solver_max_slack,solver_slack_sum,"
         "solver_fail_rate,solver_cbf_active_samples,solver_cbf_active_ratio,"
         "solver_track_only_samples,solver_track_only_ratio,"
         "solver_fail_rate_turn,solver_fail_rate_straight,solver_min_h_turn,solver_min_h_straight,"
         "solver_slack_sum_turn,solver_slack_sum_straight,solver_max_slack_turn,solver_max_slack_straight,"
         "solver_min_h_high_c,solver_slack_sum_high_c,solver_fail_rate_high_c,"
         "diag_min_h_any,diag_mean_h_any,diag_min_cbf_any,diag_mean_cbf_any\n";
}

static void writeMetricsRow(std::ostream& out,
                            const std::string& run_tag,
                            const std::string& scope,
                            const std::string& frame_mode_effective,
                            const std::string& ugv_rollout_mode,
                            const std::string& safety_variant,
                            int obstacle_count,
                            int seed,
                            int seed_env,
                            int seed_traj,
                            const std::string& traj_id,
                            double kappa,
                            double requested_r,
                            double requested_v,
                            double requested_w,
                            bool use_dynamic_obs,
                            bool warm_start,
                            double start_z,
                            int noise_seed,
                            const std::string& active_uavs_csv,
                            double sim_control_dt_sec,
                            double prediction_dt_sec,
                            int prediction_steps,
                            double max_abs_ugv_yaw_rate,
                            double turning_omega_thr,
                            bool metrics_debug,
                            double simulation_duration,
                            double wall_time,
                            bool collision,
                            const coni_mpc::NumSimMpc::MetricsSummary& summary,
                            double tracking_p95_worst_uav,
                            double solve_time_p95_worst_uav) {
  const double tracking_rms_high = summary.tracking_rms_high_c;
  const double solver_cbf_active_ratio =
      summary.samples > 0
          ? (static_cast<double>(summary.solver_cbf_active_samples) /
             static_cast<double>(summary.samples))
          : std::numeric_limits<double>::quiet_NaN();
  const double solver_track_only_ratio =
      summary.samples > 0
          ? (static_cast<double>(summary.solver_track_only_samples) /
             static_cast<double>(summary.samples))
          : std::numeric_limits<double>::quiet_NaN();
  out << run_tag << "," << scope << "," << frame_mode_effective << ","
      << ugv_rollout_mode << "," << safety_variant << ","
      << obstacle_count << "," << seed << "," << seed_env << "," << seed_traj << ","
      << traj_id << "," << fmtDouble(kappa) << ","
      << fmtDouble(requested_r) << "," << fmtDouble(requested_v) << "," << fmtDouble(requested_w) << ","
      << (use_dynamic_obs ? 1 : 0) << "," << (warm_start ? 1 : 0) << ","
      << fmtDouble(start_z) << "," << noise_seed << "," << active_uavs_csv << ","
      << fmtDouble(sim_control_dt_sec) << "," << fmtDouble(prediction_dt_sec) << ","
      << prediction_steps << "," << fmtDouble(max_abs_ugv_yaw_rate) << ","
      << fmtDouble(turning_omega_thr) << "," << (metrics_debug ? 1 : 0) << ","
      << fmtDouble(simulation_duration) << "," << fmtDouble(wall_time) << ","
      << (collision ? 1 : 0) << "," << summary.samples << ","
      << fmtDouble(summary.min_h) << "," << fmtDouble(summary.mean_h) << ","
      << fmtDouble(summary.min_cbf) << "," << fmtDouble(summary.mean_cbf) << ","
      << fmtDouble(summary.max_slack) << "," << fmtDouble(summary.tracking_rms) << ","
      << fmtDouble(summary.tracking_rms_x) << ","
      << fmtDouble(summary.tracking_rms_y) << ","
      << fmtDouble(summary.tracking_rms_z) << ","
      << fmtDouble(summary.tracking_p95) << ","
      << fmtDouble(tracking_rms_high) << ","
      << fmtDouble(summary.tracking_rms_track_only) << ","
      << fmtDouble(summary.tracking_rms_track_only_x) << ","
      << fmtDouble(summary.tracking_rms_track_only_y) << ","
      << fmtDouble(summary.tracking_rms_track_only_z) << ","
      << fmtDouble(summary.tracking_p95_track_only) << ","
      << fmtDouble(summary.tracking_rms_cbf_active) << ","
      << summary.cbf_active_samples << ","
      << fmtDouble(summary.samples > 0
                       ? (static_cast<double>(summary.cbf_active_samples) /
                          static_cast<double>(summary.samples))
                       : std::numeric_limits<double>::quiet_NaN())
      << ","
      << summary.track_only_samples << ","
      << fmtDouble(summary.samples > 0
                       ? (static_cast<double>(summary.track_only_samples) /
                          static_cast<double>(summary.samples))
                       : std::numeric_limits<double>::quiet_NaN())
      << "," << fmtDouble(tracking_p95_worst_uav) << ","
      << fmtDouble(summary.fail_rate_turn) << ","
      << fmtDouble(summary.fail_rate_straight) << ","
      << fmtDouble(summary.min_h_turn) << ","
      << fmtDouble(summary.min_h_straight) << ","
      << fmtDouble(summary.slack_sum_turn) << ","
      << fmtDouble(summary.slack_sum_straight) << ","
      << fmtDouble(summary.max_slack_turn) << ","
      << fmtDouble(summary.max_slack_straight) << ","
      << fmtDouble(summary.tracking_rms_all_turn) << ","
      << fmtDouble(summary.tracking_rms_all_straight) << ","
      << summary.turn_samples << "," << summary.straight_samples << ","
      << fmtDouble(summary.c44_metric) << ","
      << summary.high_c44_samples << ","
      << fmtDouble(summary.tracking_rms_high_c44) << ","
      << fmtDouble(summary.c66_metric) << ","
      << summary.high_c_samples << ","
      << fmtDouble(summary.min_h_high_c) << ","
      << fmtDouble(summary.slack_sum_high_c) << ","
      << fmtDouble(summary.tracking_rms_high_c) << ","
      << fmtDouble(summary.fail_rate_high_c) << ","
      << fmtDouble(summary.c88_metric) << ","
      << summary.high_c88_samples << ","
      << fmtDouble(summary.tracking_rms_high_c88) << ","
      << fmtDouble(summary.dmin_min) << ","
      << fmtDouble(summary.slack_sum) << "," << fmtDouble(summary.solve_time_mean_ms) << ","
      << fmtDouble(summary.solve_time_p95_ms) << "," << fmtDouble(solve_time_p95_worst_uav) << ","
      << fmtDouble(summary.solve_time_max_ms) << ","
      << fmtDouble(summary.feedback_time_mean_ms) << ","
      << fmtDouble(summary.feedback_time_p95_ms) << ","
      << fmtDouble(summary.feedback_time_max_ms) << ","
      << fmtDouble(summary.preparation_time_mean_ms) << ","
      << fmtDouble(summary.preparation_time_p95_ms) << ","
      << fmtDouble(summary.preparation_time_max_ms) << ","
      << fmtDouble(summary.fail_rate) << ","
      << fmtDouble(summary.solver_tracking_rms_avoid) << ","
      << summary.solver_tracking_rms_avoid_samples << ","
      << fmtDouble(summary.solver_min_h) << ","
      << fmtDouble(summary.solver_active_mean_h) << ","
      << fmtDouble(summary.solver_min_planar_clearance) << ","
      << fmtDouble(summary.solver_min_h_witness_h) << ","
      << fmtDouble(summary.solver_min_h_witness_planar_clearance) << ","
      << fmtDouble(summary.solver_min_h_witness_tracking_error) << ","
      << fmtDouble(summary.solver_min_h_witness_delta) << ","
      << fmtDouble(summary.solver_min_h_witness_planar_distance) << ","
      << summary.solver_min_h_witness_active_obstacle_key << ","
      << fmtDouble(summary.solver_mean_h) << ","
      << fmtDouble(summary.solver_min_cbf) << ","
      << fmtDouble(summary.solver_mean_cbf) << ","
      << fmtDouble(summary.solver_max_slack) << ","
      << fmtDouble(summary.solver_slack_sum) << ","
      << fmtDouble(summary.solver_fail_rate) << ","
      << summary.solver_cbf_active_samples << ","
      << fmtDouble(solver_cbf_active_ratio) << ","
      << summary.solver_track_only_samples << ","
      << fmtDouble(solver_track_only_ratio) << ","
      << fmtDouble(summary.solver_fail_rate_turn) << ","
      << fmtDouble(summary.solver_fail_rate_straight) << ","
      << fmtDouble(summary.solver_min_h_turn) << ","
      << fmtDouble(summary.solver_min_h_straight) << ","
      << fmtDouble(summary.solver_slack_sum_turn) << ","
      << fmtDouble(summary.solver_slack_sum_straight) << ","
      << fmtDouble(summary.solver_max_slack_turn) << ","
      << fmtDouble(summary.solver_max_slack_straight) << ","
      << fmtDouble(summary.solver_min_h_high_c) << ","
      << fmtDouble(summary.solver_slack_sum_high_c) << ","
      << fmtDouble(summary.solver_fail_rate_high_c) << ","
      << fmtDouble(summary.diag_min_h_any) << ","
      << fmtDouble(summary.diag_mean_h_any) << ","
      << fmtDouble(summary.diag_min_cbf_any) << ","
      << fmtDouble(summary.diag_mean_cbf_any) << "\n";
}

static std::string deriveStepMetricsCsvPath(const std::string& metrics_csv) {
  if (metrics_csv.empty()) {
    return "";
  }
  const std::string suffix = ".csv";
  if (metrics_csv.size() >= suffix.size() &&
      metrics_csv.compare(metrics_csv.size() - suffix.size(),
                          suffix.size(),
                          suffix) == 0) {
    return metrics_csv.substr(0, metrics_csv.size() - suffix.size()) +
           "_steps.csv";
  }
  return metrics_csv + "_steps.csv";
}

static std::string deriveUgvHorizonDetailCsvPath(
    const std::string& step_metrics_csv) {
  if (step_metrics_csv.empty()) {
    return "";
  }
  const std::string suffix = "_steps.csv";
  if (step_metrics_csv.size() >= suffix.size() &&
      step_metrics_csv.compare(step_metrics_csv.size() - suffix.size(),
                               suffix.size(),
                               suffix) == 0) {
    return step_metrics_csv.substr(0, step_metrics_csv.size() - suffix.size()) +
           "_ugv_horizon_steps.csv";
  }
  return step_metrics_csv + "_ugv_horizon_steps.csv";
}

static std::string deriveUgvHorizonSummaryCsvPath(
    const std::string& step_metrics_csv) {
  if (step_metrics_csv.empty()) {
    return "";
  }
  const std::string suffix = "_steps.csv";
  if (step_metrics_csv.size() >= suffix.size() &&
      step_metrics_csv.compare(step_metrics_csv.size() - suffix.size(),
                               suffix.size(),
                               suffix) == 0) {
    return step_metrics_csv.substr(0, step_metrics_csv.size() - suffix.size()) +
           "_ugv_horizon_summary.csv";
  }
  return step_metrics_csv + "_ugv_horizon_summary.csv";
}

static std::string deriveSolverObstacleDriftCsvPath(
    const std::string& step_metrics_csv) {
  if (step_metrics_csv.empty()) {
    return "";
  }
  const std::string suffix = "_steps.csv";
  if (step_metrics_csv.size() >= suffix.size() &&
      step_metrics_csv.compare(step_metrics_csv.size() - suffix.size(),
                               suffix.size(),
                               suffix) == 0) {
    return step_metrics_csv.substr(0, step_metrics_csv.size() - suffix.size()) +
           "_solver_obstacle_drift.csv";
  }
  return step_metrics_csv + "_solver_obstacle_drift.csv";
}

static std::string deriveExternalAvoidanceCsvPath(
    const std::string& step_metrics_csv) {
  if (step_metrics_csv.empty()) {
    return "";
  }
  const std::string suffix = "_steps.csv";
  if (step_metrics_csv.size() >= suffix.size() &&
      step_metrics_csv.compare(step_metrics_csv.size() - suffix.size(),
                               suffix.size(),
                               suffix) == 0) {
    return step_metrics_csv.substr(0, step_metrics_csv.size() - suffix.size()) +
           "_vo_hrvo_steps.csv";
  }
  return step_metrics_csv + "_vo_hrvo_steps.csv";
}

static void writeStepMetricsHeader(std::ostream& out) {
  out << "run_tag,uav_idx,step_idx,sim_time,frame_mode_effective,ugv_rollout_mode,safety_variant,"
         "solver_cbf_active,solver_track_only,solve_ok,solver_fail_flag,"
         "solver_active_obstacle_key,solver_h,solver_hdot,solver_cbf,solver_slack,"
         "solver_planar_distance,solver_planar_clearance,solver_planar_surface_distance,"
         "feedback_time_ms,preparation_time_ms,core_time_ms,"
         "external_filter_time_ms,total_pipeline_time_ms,tracking_error,"
         "position_x,position_y,position_z,"
         "velocity_x,velocity_y,velocity_z,"
         "applied_cmd_vx_world,applied_cmd_vy_world,applied_cmd_vz_world,"
         "applied_cmd_yaw_rate_world,truth_min_planar_surface_clearance,"
         "rot_load_position_non_x,rot_load_position_non_y,rot_load_position_non_z,"
         "rot_load_velocity_non_x,rot_load_velocity_non_y,rot_load_velocity_non_z,"
         "rot_load_omega_non_x,rot_load_omega_non_y,rot_load_omega_non_z,"
         "active_obstacle_x,active_obstacle_y,active_obstacle_z,active_obstacle_radius,"
         "a_car_non_x,a_car_non_y,a_car_non_z,"
         "car_a_world_x,car_a_world_y,car_a_world_z,"
         "car_omega_world_x,car_omega_world_y,car_omega_world_z,"
         "omega_non_x,omega_non_y,omega_non_z\n";
}

static void writeStepMetricsRow(
    std::ostream& out,
    const std::string& run_tag,
    int uav_idx,
    std::size_t step_idx,
    double sim_time,
    const std::string& frame_mode_effective,
    const std::string& ugv_rollout_mode,
    const std::string& safety_variant,
    const coni_mpc::NumSimMpc::StepDebugSnapshot& step_debug,
    const Eigen::Vector4d& applied_control_world,
    double truth_min_planar_surface_clearance,
    double external_filter_time_ms) {
  const bool has_solver_active_obstacle =
      step_debug.solver_cbf_active &&
      step_debug.solver_active_obstacle_key != "none" &&
      std::isfinite(step_debug.solver_planar_distance);
  const double obstacle_x =
      has_solver_active_obstacle
          ? step_debug.active_obstacle_solver(0)
          : std::numeric_limits<double>::quiet_NaN();
  const double obstacle_y =
      has_solver_active_obstacle
          ? step_debug.active_obstacle_solver(1)
          : std::numeric_limits<double>::quiet_NaN();
  const double obstacle_z =
      has_solver_active_obstacle
          ? step_debug.active_obstacle_solver(2)
          : std::numeric_limits<double>::quiet_NaN();
  const double obstacle_radius =
      has_solver_active_obstacle
          ? step_debug.active_obstacle_solver(3)
          : std::numeric_limits<double>::quiet_NaN();
  out << run_tag << ","
      << uav_idx << ","
      << step_idx << ","
      << fmtDouble(sim_time) << ","
      << frame_mode_effective << ","
      << ugv_rollout_mode << ","
      << safety_variant << ","
      << (step_debug.solver_cbf_active ? 1 : 0) << ","
      << (step_debug.solver_track_only ? 1 : 0) << ","
      << (step_debug.solve_ok ? 1 : 0) << ","
      << (step_debug.solver_fail_flag ? 1 : 0) << ","
      << step_debug.solver_active_obstacle_key << ","
      << fmtDouble(step_debug.solver_h) << ","
      << fmtDouble(step_debug.solver_hdot) << ","
      << fmtDouble(step_debug.solver_cbf) << ","
      << fmtDouble(step_debug.solver_slack) << ","
      << fmtDouble(step_debug.solver_planar_distance) << ","
      << fmtDouble(step_debug.solver_planar_clearance) << ","
      << fmtDouble(step_debug.solver_planar_surface_distance) << ","
      << fmtDouble(step_debug.feedback_time_ms) << ","
      << fmtDouble(step_debug.preparation_time_ms) << ","
      << fmtDouble(step_debug.core_time_ms) << ","
      << fmtDouble(external_filter_time_ms) << ","
      << fmtDouble(step_debug.core_time_ms + external_filter_time_ms) << ","
      << fmtDouble(step_debug.tracking_error) << ","
      << fmtDouble(step_debug.position.x()) << ","
      << fmtDouble(step_debug.position.y()) << ","
      << fmtDouble(step_debug.position.z()) << ","
      << fmtDouble(step_debug.velocity.x()) << ","
      << fmtDouble(step_debug.velocity.y()) << ","
      << fmtDouble(step_debug.velocity.z()) << ","
      << fmtDouble(applied_control_world.x()) << ","
      << fmtDouble(applied_control_world.y()) << ","
      << fmtDouble(applied_control_world.z()) << ","
      << fmtDouble(applied_control_world.w()) << ","
      << fmtDouble(truth_min_planar_surface_clearance) << ","
      << fmtDouble(step_debug.rot_load_position_non.x()) << ","
      << fmtDouble(step_debug.rot_load_position_non.y()) << ","
      << fmtDouble(step_debug.rot_load_position_non.z()) << ","
      << fmtDouble(step_debug.rot_load_velocity_non.x()) << ","
      << fmtDouble(step_debug.rot_load_velocity_non.y()) << ","
      << fmtDouble(step_debug.rot_load_velocity_non.z()) << ","
      << fmtDouble(step_debug.rot_load_omega_non.x()) << ","
      << fmtDouble(step_debug.rot_load_omega_non.y()) << ","
      << fmtDouble(step_debug.rot_load_omega_non.z()) << ","
      << fmtDouble(obstacle_x) << ","
      << fmtDouble(obstacle_y) << ","
      << fmtDouble(obstacle_z) << ","
      << fmtDouble(obstacle_radius) << ","
      << fmtDouble(step_debug.a_car_non.x()) << ","
      << fmtDouble(step_debug.a_car_non.y()) << ","
      << fmtDouble(step_debug.a_car_non.z()) << ","
      << fmtDouble(step_debug.car_a_world.x()) << ","
      << fmtDouble(step_debug.car_a_world.y()) << ","
      << fmtDouble(step_debug.car_a_world.z()) << ","
      << fmtDouble(step_debug.car_omega_world.x()) << ","
      << fmtDouble(step_debug.car_omega_world.y()) << ","
      << fmtDouble(step_debug.car_omega_world.z()) << ","
      << fmtDouble(step_debug.omega_non.x()) << ","
      << fmtDouble(step_debug.omega_non.y()) << ","
      << fmtDouble(step_debug.omega_non.z()) << "\n";
}

static void writeUgvHorizonDetailHeader(std::ostream& out) {
  out << "run_tag,step_idx,sim_time,frame_mode_effective,ugv_rollout_mode,horizon_idx,"
         "pred_position_x,pred_position_y,pred_position_z,"
         "actual_position_x,actual_position_y,actual_position_z,"
         "pred_velocity_x,pred_velocity_y,pred_velocity_z,"
         "actual_velocity_x,actual_velocity_y,actual_velocity_z,"
         "pred_omega_non_x,pred_omega_non_y,pred_omega_non_z,"
         "actual_omega_non_x,actual_omega_non_y,actual_omega_non_z,"
         "pos_error_xy,pos_error_xyz\n";
}

static void writeUgvHorizonSummaryHeader(std::ostream& out) {
  out << "run_tag,step_idx,sim_time,frame_mode_effective,ugv_rollout_mode,"
         "horizon_rms_xy,horizon_max_xy,horizon_final_xy,horizon_samples,"
         "boundary_drift_rms,boundary_drift_max,boundary_drift_samples\n";
}

RandomObstacleConfig loadRandomObstacleConfig(const ros::NodeHandle& pnh) {
  RandomObstacleConfig cfg;
  cfg.count = 30;
  cfg.x_min = -5.0;
  cfg.x_max = 5.0;
  cfg.y_min = -5.0;
  cfg.y_max = 5.0;
  cfg.h_min = 0.5;
  cfg.h_max = 3.0;
  cfg.radius = 0.15;
  cfg.seed = 1;

  pnh.param("random_obstacles/count", cfg.count, cfg.count);
  std::vector<double> x_range;
  if (pnh.getParam("random_obstacles/x_range", x_range) && x_range.size() == 2) {
    cfg.x_min = x_range[0];
    cfg.x_max = x_range[1];
  }
  std::vector<double> y_range;
  if (pnh.getParam("random_obstacles/y_range", y_range) && y_range.size() == 2) {
    cfg.y_min = y_range[0];
    cfg.y_max = y_range[1];
  }
  std::vector<double> h_range;
  if (pnh.getParam("random_obstacles/height_range", h_range) && h_range.size() == 2) {
    cfg.h_min = h_range[0];
    cfg.h_max = h_range[1];
  }
  pnh.param("random_obstacles/radius", cfg.radius, cfg.radius);
  pnh.param("random_obstacles/seed", cfg.seed, cfg.seed);

  if (cfg.x_min > cfg.x_max) std::swap(cfg.x_min, cfg.x_max);
  if (cfg.y_min > cfg.y_max) std::swap(cfg.y_min, cfg.y_max);
  if (cfg.h_min > cfg.h_max) std::swap(cfg.h_min, cfg.h_max);
  cfg.h_min = std::max(cfg.h_min, 0.05);
  cfg.h_max = std::max(cfg.h_max, cfg.h_min);
  cfg.radius = std::max(cfg.radius, 0.05);
  cfg.count = std::max(0, cfg.count);
  return cfg;
}

double xmlRpcNumberToDouble(const XmlRpc::XmlRpcValue& value, bool& ok) {
  ok = true;
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  ok = false;
  return 0.0;
}

std::vector<Eigen::Vector2d> loadGuideWaypoints(const ros::NodeHandle& pnh) {
  std::vector<Eigen::Vector2d> guide_waypoints;
  XmlRpc::XmlRpcValue raw_waypoints;
  if (!pnh.getParam("ugv_path/guide_waypoints", raw_waypoints)) {
    return guide_waypoints;
  }
  if (raw_waypoints.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_WARN("Parameter ugv_path/guide_waypoints should be a list of [x, y] pairs.");
    return guide_waypoints;
  }
  for (int i = 0; i < raw_waypoints.size(); ++i) {
    const XmlRpc::XmlRpcValue& point = raw_waypoints[i];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray || point.size() != 2) {
      ROS_WARN_STREAM("Skipping invalid guide waypoint at index " << i
                      << ": expected [x, y].");
      continue;
    }
    bool ok_x = false;
    bool ok_y = false;
    const double x = xmlRpcNumberToDouble(point[0], ok_x);
    const double y = xmlRpcNumberToDouble(point[1], ok_y);
    if (!ok_x || !ok_y) {
      ROS_WARN_STREAM("Skipping invalid numeric guide waypoint at index " << i << ".");
      continue;
    }
    guide_waypoints.push_back(Eigen::Vector2d(x, y));
  }
  return guide_waypoints;
}

UgvPathPlannerConfig loadUgvPathPlannerConfig(const ros::NodeHandle& pnh) {
  UgvPathPlannerConfig cfg;
  cfg.a_star_connectivity = 4;
  cfg.use_midpoint_guide = false;
  cfg.use_last_guide_as_goal = true;
  cfg.enable_shortcut_pruning = false;
  cfg.enable_endpoint_snapping = true;
  cfg.longitudinal_accel_max = 2.0;
  cfg.longitudinal_decel_max = 2.5;
  cfg.longitudinal_jerk_max = 8.0;
  cfg.yaw_accel_max = 1.0;
  cfg.yaw_jerk_max = 8.0;
  cfg.speed_tracking_tau = 0.25;
  cfg.yaw_rate_tracking_tau = 0.18;
  cfg.along_track_feedback_gain = 1.0;
  cfg.heading_feedback_gain = 2.2;
  cfg.lateral_feedback_gain = 0.8;
  pnh.param("ugv_path/a_star_connectivity",
            cfg.a_star_connectivity,
            cfg.a_star_connectivity);
  if (cfg.a_star_connectivity != 4 && cfg.a_star_connectivity != 8) {
    ROS_WARN_STREAM("Parameter ugv_path/a_star_connectivity=" << cfg.a_star_connectivity
                    << " is unsupported. Falling back to 4-connected A*.");
    cfg.a_star_connectivity = 4;
  }
  pnh.param("ugv_path/use_midpoint_guide",
            cfg.use_midpoint_guide,
            cfg.use_midpoint_guide);
  pnh.param("ugv_path/use_last_guide_as_goal",
            cfg.use_last_guide_as_goal,
            cfg.use_last_guide_as_goal);
  pnh.param("ugv_path/enable_shortcut_pruning",
            cfg.enable_shortcut_pruning,
            cfg.enable_shortcut_pruning);
  pnh.param("ugv_path/enable_endpoint_snapping",
            cfg.enable_endpoint_snapping,
            cfg.enable_endpoint_snapping);
  pnh.param("ugv_path/longitudinal_accel_max",
            cfg.longitudinal_accel_max,
            cfg.longitudinal_accel_max);
  pnh.param("ugv_path/longitudinal_decel_max",
            cfg.longitudinal_decel_max,
            cfg.longitudinal_decel_max);
  pnh.param("ugv_path/longitudinal_jerk_max",
            cfg.longitudinal_jerk_max,
            cfg.longitudinal_jerk_max);
  pnh.param("ugv_path/yaw_accel_max",
            cfg.yaw_accel_max,
            cfg.yaw_accel_max);
  pnh.param("ugv_path/yaw_jerk_max",
            cfg.yaw_jerk_max,
            cfg.yaw_jerk_max);
  pnh.param("ugv_path/speed_tracking_tau",
            cfg.speed_tracking_tau,
            cfg.speed_tracking_tau);
  pnh.param("ugv_path/yaw_rate_tracking_tau",
            cfg.yaw_rate_tracking_tau,
            cfg.yaw_rate_tracking_tau);
  pnh.param("ugv_path/along_track_feedback_gain",
            cfg.along_track_feedback_gain,
            cfg.along_track_feedback_gain);
  pnh.param("ugv_path/heading_feedback_gain",
            cfg.heading_feedback_gain,
            cfg.heading_feedback_gain);
  pnh.param("ugv_path/lateral_feedback_gain",
            cfg.lateral_feedback_gain,
            cfg.lateral_feedback_gain);
  if (!(cfg.longitudinal_accel_max > 0.0)) {
    ROS_WARN_STREAM("Parameter ugv_path/longitudinal_accel_max="
                    << cfg.longitudinal_accel_max
                    << " is invalid. Falling back to 2.0 m/s^2.");
    cfg.longitudinal_accel_max = 2.0;
  }
  if (!(cfg.longitudinal_decel_max > 0.0)) {
    ROS_WARN_STREAM("Parameter ugv_path/longitudinal_decel_max="
                    << cfg.longitudinal_decel_max
                    << " is invalid. Falling back to 2.5 m/s^2.");
    cfg.longitudinal_decel_max = 2.5;
  }
  if (!(cfg.longitudinal_jerk_max > 0.0)) {
    ROS_WARN_STREAM("Parameter ugv_path/longitudinal_jerk_max="
                    << cfg.longitudinal_jerk_max
                    << " is invalid. Falling back to 8.0 m/s^3.");
    cfg.longitudinal_jerk_max = 8.0;
  }
  if (!(cfg.yaw_accel_max > 0.0)) {
    ROS_WARN_STREAM("Parameter ugv_path/yaw_accel_max="
                    << cfg.yaw_accel_max
                    << " is invalid. Falling back to 1.0 rad/s^2.");
    cfg.yaw_accel_max = 1.0;
  }
  if (!(cfg.yaw_jerk_max > 0.0)) {
    ROS_WARN_STREAM("Parameter ugv_path/yaw_jerk_max="
                    << cfg.yaw_jerk_max
                    << " is invalid. Falling back to 8.0 rad/s^3.");
    cfg.yaw_jerk_max = 8.0;
  }
  if (!(cfg.speed_tracking_tau > 1e-3)) {
    ROS_WARN_STREAM("Parameter ugv_path/speed_tracking_tau="
                    << cfg.speed_tracking_tau
                    << " is invalid. Falling back to 0.25 s.");
    cfg.speed_tracking_tau = 0.25;
  }
  if (!(cfg.yaw_rate_tracking_tau > 1e-3)) {
    ROS_WARN_STREAM("Parameter ugv_path/yaw_rate_tracking_tau="
                    << cfg.yaw_rate_tracking_tau
                    << " is invalid. Falling back to 0.18 s.");
    cfg.yaw_rate_tracking_tau = 0.18;
  }
  cfg.guide_waypoints = loadGuideWaypoints(pnh);
  return cfg;
}

void scaleUgvPathPlannerDynamics(UgvPathPlannerConfig& cfg,
                                 double speed_limit,
                                 double yaw_rate_limit) {
  const double speed_scale = std::abs(speed_limit);
  const double yaw_scale = std::abs(yaw_rate_limit);
  if (!(speed_scale > 1e-9)) {
    ROS_WARN_STREAM("UGV speed scale from V=" << speed_limit
                    << " is non-positive; leaving longitudinal dynamics unscaled.");
  } else {
    cfg.longitudinal_accel_max *= speed_scale;
    cfg.longitudinal_decel_max *= speed_scale;
    cfg.longitudinal_jerk_max *= speed_scale;
  }
  if (!(yaw_scale > 1e-9)) {
    ROS_WARN_STREAM("UGV yaw scale from W=" << yaw_rate_limit
                    << " is non-positive; leaving yaw dynamics unscaled.");
  } else {
    cfg.yaw_accel_max *= yaw_scale;
    cfg.yaw_jerk_max *= yaw_scale;
  }
}

std::vector<Eigen::Vector3d> defaultUavReferenceOffsets(int vehicle_count,
                                                        double formation_radius,
                                                        double default_z) {
  std::vector<Eigen::Vector3d> offsets(
      static_cast<size_t>(std::max(0, vehicle_count)),
      Eigen::Vector3d(0.0, 0.0, default_z));
  if (offsets.empty()) {
    return offsets;
  }

  const std::vector<Eigen::Vector2d> canonical_offsets = {
      Eigen::Vector2d(-formation_radius, 0.0),
      Eigen::Vector2d(0.0, formation_radius),
      Eigen::Vector2d(0.0, -formation_radius),
      Eigen::Vector2d(formation_radius, 0.0),
  };
  const size_t direct_count = std::min(offsets.size(), canonical_offsets.size());
  for (size_t idx = 0; idx < direct_count; ++idx) {
    offsets[idx].x() = canonical_offsets[idx].x();
    offsets[idx].y() = canonical_offsets[idx].y();
  }

  for (size_t idx = canonical_offsets.size(); idx < offsets.size(); ++idx) {
    const double angle = 2.0 * M_PI *
                         static_cast<double>(idx - canonical_offsets.size()) /
                         static_cast<double>(std::max<std::size_t>(
                             1, offsets.size() - canonical_offsets.size()));
    offsets[idx].x() = formation_radius * std::cos(angle);
    offsets[idx].y() = formation_radius * std::sin(angle);
  }
  return offsets;
}

std::vector<Eigen::Vector3d> loadUavReferenceOffsets(const ros::NodeHandle& pnh,
                                                     int vehicle_count,
                                                     double formation_radius,
                                                     double default_z) {
  XmlRpc::XmlRpcValue raw_offsets;
  const bool has_custom_offsets = pnh.getParam("uav_reference_offsets", raw_offsets);

  double effective_formation_radius = formation_radius;
  if (!has_custom_offsets && vehicle_count > 1 &&
      std::abs(effective_formation_radius) < 1e-6) {
    effective_formation_radius = 1.0;
    ROS_WARN_STREAM("launch arg r is zero while num_uavs=" << vehicle_count
                    << ". Falling back to a 1.0 m default formation radius. "
                    << "Set r or uav_reference_offsets explicitly to override.");
  }

  std::vector<Eigen::Vector3d> offsets = defaultUavReferenceOffsets(
      vehicle_count, effective_formation_radius, default_z);
  if (!has_custom_offsets) {
    return offsets;
  }
  if (raw_offsets.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_WARN("Parameter uav_reference_offsets should be a list of [x, y] or [x, y, z].");
    return offsets;
  }

  const int limit = std::min<int>(vehicle_count, raw_offsets.size());
  for (int idx = 0; idx < limit; ++idx) {
    const XmlRpc::XmlRpcValue& entry = raw_offsets[idx];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        (entry.size() != 2 && entry.size() != 3)) {
      ROS_WARN_STREAM("Skipping invalid uav_reference_offsets[" << idx
                      << "]: expected [x, y] or [x, y, z].");
      continue;
    }
    bool ok_x = false;
    bool ok_y = false;
    const double x = xmlRpcNumberToDouble(entry[0], ok_x);
    const double y = xmlRpcNumberToDouble(entry[1], ok_y);
    bool ok_z = true;
    double z = default_z;
    if (entry.size() == 3) {
      z = xmlRpcNumberToDouble(entry[2], ok_z);
    }
    if (!ok_x || !ok_y || !ok_z) {
      ROS_WARN_STREAM("Skipping non-numeric uav_reference_offsets[" << idx << "].");
      continue;
    }
    offsets[static_cast<size_t>(idx)] = Eigen::Vector3d(x, y, z);
  }
  return offsets;
}

std::vector<StaticObstacle> generateRandomCylinders(const RandomObstacleConfig& cfg,
                                                    double uav_radius) {
  std::mt19937 gen(static_cast<uint32_t>(cfg.seed));
  std::uniform_real_distribution<double> dist_x(cfg.x_min, cfg.x_max);
  std::uniform_real_distribution<double> dist_y(cfg.y_min, cfg.y_max);
  std::uniform_real_distribution<double> dist_h(cfg.h_min, cfg.h_max);

  std::vector<StaticObstacle> obstacles;
  obstacles.reserve(static_cast<size_t>(cfg.count));
  (void)uav_radius;
  // Keep at least 1.0 m of free space between obstacle surfaces.
  const double min_center_dist = 2.0 * cfg.radius + 1.0;
  int attempts = 0;
  const int max_attempts = cfg.count * 20;
  while (static_cast<int>(obstacles.size()) < cfg.count && attempts < max_attempts) {
    attempts++;
    const double x = dist_x(gen);
    const double y = dist_y(gen);
    const double h = dist_h(gen);
    bool overlaps = false;
    for (const auto& o : obstacles) {
      if ((Eigen::Vector2d(o.position.x(), o.position.y()) - Eigen::Vector2d(x, y)).norm() < min_center_dist) {
        overlaps = true;
        break;
      }
    }
    if (overlaps) continue;
    StaticObstacle obs;
    obs.position = Eigen::Vector3d(x, y, 0.5 * h);
    obs.velocity = Eigen::Vector3d::Zero();
    obs.radius = cfg.radius;
    obs.height = h;
    obstacles.push_back(obs);
  }
  return obstacles;
}

// ──────────────────────────────────────────────────────────────────────────
// Figure-8 strategic obstacle generation
// Obstacles are placed at analytically derived positions that scale with the
// Gerono amplitude (A = v * K_yaw / (w * K_speed)) so that they always land
// on the inner side of the four highest-curvature zones regardless of v and w.
// ──────────────────────────────────────────────────────────────────────────

struct FigureEightObstacleConfig {
  bool enabled = false;
  double obstacle_radius = 0.40;
  double obstacle_height = 3.0;
  // Ratios relative to the Gerono high-curvature ordinate y_hc = A / 2.
  // Defaults give the V=2,W=2 obstacle set:
  //   (±2.31, ±1.00), (0, ±1.80).
  // The crossing obstacles are kept farther from the initial formation so the
  // controllers must avoid them instead of starting in collision.
  double turn_y_ratio = 0.612;
  double crossing_y_ratio = 1.10;
};

struct CircleObstacleConfig {
  bool enabled = false;
  int count = 6;
  double obstacle_radius = 0.40;
  double obstacle_height = 3.0;
  double start_angle_deg = 0.0;
  // Nominal physical surface clearance between the outer formation path and
  // each cylinder.  The default 0.25 m is contact-free but lies inside the
  // common 0.35 m controller safety margin, so avoidance must actually act.
  double nominal_surface_clearance = 0.25;
};

struct FigureEightGeometryConfig {
  bool fixed_geometry = true;
  double reference_v = 3.0;
  double reference_w = 3.0;
  double amplitude = 0.0;
  // Gerono phase at t=0. pi/2 starts at the right outer tip (A, 0).
  double start_phase = 0.5 * M_PI;
};

constexpr double kFigureEightGeronoMeanSpeedFactor = 0.9704032544031163;
constexpr double kFigureEightGeronoMaxYawRateFactor = 3.171202014425998;

static double figureEightAmplitudeFromVw(double v_speed, double w_yaw_rate) {
  const double spd = std::abs(v_speed);
  const double omg = std::abs(w_yaw_rate);
  if (spd <= 1e-6 || omg <= 1e-6) {
    return 0.0;
  }
  const double phase_rate = omg / kFigureEightGeronoMaxYawRateFactor;
  return spd / (phase_rate * kFigureEightGeronoMeanSpeedFactor);
}

static FigureEightGeometryConfig loadFigureEightGeometryConfig(
    const ros::NodeHandle& pnh) {
  FigureEightGeometryConfig cfg;
  pnh.param("figure_eight_geometry/fixed", cfg.fixed_geometry,
            cfg.fixed_geometry);
  pnh.param("figure_eight_geometry/reference_v", cfg.reference_v,
            cfg.reference_v);
  pnh.param("figure_eight_geometry/reference_w", cfg.reference_w,
            cfg.reference_w);
  pnh.param("figure_eight_geometry/amplitude", cfg.amplitude, cfg.amplitude);
  pnh.param("figure_eight_geometry/start_phase", cfg.start_phase,
            cfg.start_phase);
  cfg.reference_v = std::max(1e-6, std::abs(cfg.reference_v));
  cfg.reference_w = std::max(1e-6, std::abs(cfg.reference_w));
  cfg.amplitude = std::max(0.0, cfg.amplitude);
  return cfg;
}

static double resolveFigureEightAmplitude(
    double v_speed,
    double w_yaw_rate,
    const FigureEightGeometryConfig& geometry_cfg) {
  if (geometry_cfg.fixed_geometry) {
    if (geometry_cfg.amplitude > 1e-6) {
      return geometry_cfg.amplitude;
    }
    return figureEightAmplitudeFromVw(geometry_cfg.reference_v,
                                      geometry_cfg.reference_w);
  }
  return figureEightAmplitudeFromVw(v_speed, w_yaw_rate);
}

static FigureEightObstacleConfig loadFigureEightObstacleConfig(
    const ros::NodeHandle& pnh) {
  FigureEightObstacleConfig cfg;
  pnh.param("figure_eight_obstacles/enabled", cfg.enabled, cfg.enabled);
  pnh.param("figure_eight_obstacles/radius", cfg.obstacle_radius,
            cfg.obstacle_radius);
  pnh.param("figure_eight_obstacles/height", cfg.obstacle_height,
            cfg.obstacle_height);
  pnh.param("figure_eight_obstacles/turn_y_ratio", cfg.turn_y_ratio,
            cfg.turn_y_ratio);
  pnh.param("figure_eight_obstacles/crossing_y_ratio", cfg.crossing_y_ratio,
            cfg.crossing_y_ratio);
  cfg.obstacle_radius = std::max(0.05, cfg.obstacle_radius);
  cfg.obstacle_height = std::max(0.1, cfg.obstacle_height);
  cfg.turn_y_ratio = std::max(0.05, cfg.turn_y_ratio);
  cfg.crossing_y_ratio = std::max(0.05, cfg.crossing_y_ratio);
  return cfg;
}

static CircleObstacleConfig loadCircleObstacleConfig(
    const ros::NodeHandle& pnh) {
  CircleObstacleConfig cfg;
  pnh.param("circle_obstacles/enabled", cfg.enabled, cfg.enabled);
  pnh.param("circle_obstacles/count", cfg.count, cfg.count);
  pnh.param("circle_obstacles/radius", cfg.obstacle_radius,
            cfg.obstacle_radius);
  pnh.param("circle_obstacles/height", cfg.obstacle_height,
            cfg.obstacle_height);
  pnh.param("circle_obstacles/start_angle_deg", cfg.start_angle_deg,
            cfg.start_angle_deg);
  pnh.param("circle_obstacles/nominal_surface_clearance",
            cfg.nominal_surface_clearance,
            cfg.nominal_surface_clearance);
  cfg.count = std::max(1, cfg.count);
  cfg.obstacle_radius = std::max(0.05, cfg.obstacle_radius);
  cfg.obstacle_height = std::max(0.1, cfg.obstacle_height);
  cfg.nominal_surface_clearance =
      std::max(0.0, cfg.nominal_surface_clearance);
  return cfg;
}

// Place fixed cylinders at equal angular intervals around a world-frame ring.
// The ring is outside the UGV path and the outer nominal formation path:
//
//   R_ring = R_UGV + r_formation + r_UAV + r_obstacle + d_surface,nom.
//
// With count=6 this gives one cylinder every 60 degrees.  This construction is
// independent of the frozen/stage rollout and therefore does not target the
// high-curvature failure mechanism criticized in Comment 3.4.
static std::vector<StaticObstacle> generateCircleRingObstacles(
    double v_speed,
    double w_yaw_rate,
    double formation_radius,
    double uav_radius,
    const CircleObstacleConfig& cfg) {
  std::vector<StaticObstacle> obstacles;
  if (!cfg.enabled) return obstacles;

  const double speed = std::abs(v_speed);
  const double omega = std::abs(w_yaw_rate);
  if (speed <= 1e-6 || omega <= 1e-6) {
    ROS_WARN("Circle obstacles requested with zero circle speed/yaw rate; no ring obstacles generated.");
    return obstacles;
  }

  const double circle_radius = speed / omega;
  const double outer_formation_radius =
      circle_radius + std::abs(formation_radius);
  const double ring_radius =
      outer_formation_radius + std::max(0.0, uav_radius) +
      cfg.obstacle_radius + cfg.nominal_surface_clearance;
  const double start_angle = cfg.start_angle_deg * M_PI / 180.0;
  const double height_center = 0.5 * cfg.obstacle_height;
  obstacles.reserve(static_cast<std::size_t>(cfg.count));
  for (int index = 0; index < cfg.count; ++index) {
    const double angle =
        start_angle + 2.0 * M_PI * static_cast<double>(index) /
                          static_cast<double>(cfg.count);
    StaticObstacle obstacle;
    obstacle.position = Eigen::Vector3d(ring_radius * std::cos(angle),
                                        ring_radius * std::sin(angle),
                                        height_center);
    obstacle.velocity = Eigen::Vector3d::Zero();
    obstacle.radius = cfg.obstacle_radius;
    obstacle.height = cfg.obstacle_height;
    obstacles.push_back(obstacle);
  }

  ROS_INFO_STREAM(
      "Circle ring obstacles: circle_radius=" << circle_radius
      << " outer_formation_radius=" << outer_formation_radius
      << " ring_radius=" << ring_radius
      << " count=" << obstacles.size()
      << " angular_spacing_deg=" << (360.0 / cfg.count)
      << " start_angle_deg=" << cfg.start_angle_deg
      << " obstacle_radius=" << cfg.obstacle_radius
      << " nominal_surface_clearance="
      << cfg.nominal_surface_clearance);
  return obstacles;
}

// Compute 6 strategic obstacles for the Gerono figure-8.
//
// The Gerono curve: x = A sin(φ), y = 0.5 A sin(2φ).
// Maximum |omega_z| occurs at φ = 45°/135°/225°/315°, at positions
//   (±A/√2, ±A/2).  At these points the curvature centre lies inward
//   (toward lobe interior), so obstacles placed there force the UAV to
//   deflect while the non-inertial frame is rotating fastest — exactly
//   where the "frozen" UGV rollout prediction diverges most from reality.
//
// Two additional obstacles at the crossing point (0,0) test the high-speed
// straight phase where the frozen solver has maximum position-prediction lag.
static std::vector<StaticObstacle> generateFigureEightStrategicObstacles(
    double v_speed,
    double w_yaw_rate,
    double formation_radius,
    double uav_radius,
    double safety_margin,
    const FigureEightGeometryConfig& geometry_cfg,
    const FigureEightObstacleConfig& cfg) {
  std::vector<StaticObstacle> obstacles;
  if (!cfg.enabled) return obstacles;
  (void)formation_radius;
  (void)uav_radius;
  (void)safety_margin;

  const double amplitude =
      resolveFigureEightAmplitude(v_speed, w_yaw_rate, geometry_cfg);
  if (amplitude <= 1e-6) return obstacles;

  // High-curvature positions (phi = 45°): (A/√2, A/2)
  const double x_hc = amplitude / std::sqrt(2.0);
  const double y_hc = amplitude / 2.0;
  const double y_turn = cfg.turn_y_ratio * y_hc;
  const double y_crossing = cfg.crossing_y_ratio * y_hc;

  const double h2 = cfg.obstacle_height / 2.0;

  auto makeObs = [&](double x, double y) {
    StaticObstacle obs;
    obs.position = Eigen::Vector3d(x, y, h2);
    obs.velocity = Eigen::Vector3d::Zero();
    obs.radius   = cfg.obstacle_radius;
    obs.height   = cfg.obstacle_height;
    return obs;
  };

  // 4 turn-zone obstacles near the maximum-yaw-rate parts of both lobes.
  obstacles.push_back(makeObs( x_hc,  y_turn));
  obstacles.push_back(makeObs( x_hc, -y_turn));
  obstacles.push_back(makeObs(-x_hc,  y_turn));
  obstacles.push_back(makeObs(-x_hc, -y_turn));

  // 2 crossing-zone obstacles near the high-speed sign-change region.
  obstacles.push_back(makeObs(0.0,  y_crossing));
  obstacles.push_back(makeObs(0.0, -y_crossing));

  ROS_INFO_STREAM(
      "Figure-8 strategic obstacles: amplitude=" << amplitude
      << " m | x_hc=" << x_hc << " y_hc=" << y_hc
      << " | y_turn=" << y_turn
      << " | y_crossing=" << y_crossing
      << " | count=" << obstacles.size());
  return obstacles;
}

double wrapAngle(double angle) {
  const double pi = 3.14159265358979323846;
  const double two_pi = 2.0 * pi;
  while (angle > pi) angle -= two_pi;
  while (angle < -pi) angle += two_pi;
  return angle;
}

Eigen::Vector2d obstacleRepulsion(const Eigen::Vector2d& desired_xy,
                                  const std::vector<StaticObstacle>& obstacles,
                                  double clearance,
                                  double max_offset)
{
  Eigen::Vector2d correction = Eigen::Vector2d::Zero();
  for (const auto& obstacle : obstacles) {
    Eigen::Vector2d obs_xy(obstacle.position.x(), obstacle.position.y());
    Eigen::Vector2d diff = desired_xy - obs_xy;
    const double dist = diff.norm();
    const double keepout = obstacle.radius + clearance;
    if (dist < keepout && dist > 1e-4) {
      const double strength = (keepout - dist) / keepout;
      correction += strength * diff.normalized();
    }
  }
  const double norm = correction.norm();
  if (norm > 1e-6) {
    correction = correction / norm * std::min(max_offset, norm);
  }
  return correction;
}

// Forward declaration for smoothing helper used inside relaxPathWithObstacles.
void smoothPolyline(std::vector<Eigen::Vector2d>& pts, int iterations, double alpha);

void relaxPathWithObstacles(std::vector<Eigen::Vector2d>& pts,
                            const std::vector<StaticObstacle>& obstacles,
                            double clearance,
                            double max_offset,
                            int iterations,
                            double step_scale,
                            double smooth_alpha)
{
  if (pts.size() < 3 || iterations <= 0) return;
  const double clamped_step = std::max(0.0, std::min(step_scale, 1.0));
  for (int k = 0; k < iterations; ++k) {
    for (size_t i = 1; i + 1 < pts.size(); ++i) {
      Eigen::Vector2d correction = obstacleRepulsion(pts[i], obstacles, clearance, max_offset);
      pts[i] += clamped_step * correction;
    }
    smoothPolyline(pts, 1, smooth_alpha);
  }
}

void smoothPolyline(std::vector<Eigen::Vector2d>& pts, int iterations, double alpha)
{
  if (pts.size() < 3 || iterations <= 0 || alpha <= 1e-6) return;
  const size_t n = pts.size();
  for (int k = 0; k < iterations; ++k) {
    std::vector<Eigen::Vector2d> temp = pts;
    for (size_t i = 1; i + 1 < n; ++i) {
      const Eigen::Vector2d avg = 0.5 * (temp[i - 1] + temp[i + 1]);
      pts[i] = pts[i] + alpha * (avg - temp[i]);
    }
  }
}

std::vector<Eigen::Vector2d> resampleUniformArcLength(const std::vector<Eigen::Vector2d>& pts,
                                                      size_t sample_num)
{
  std::vector<Eigen::Vector2d> out;
  if (pts.size() < 2 || sample_num == 0) {
    return out;
  }
  std::vector<double> s;
  s.reserve(pts.size());
  s.push_back(0.0);
  for (size_t i = 1; i < pts.size(); ++i) {
    const double ds = (pts[i] - pts[i - 1]).norm();
    s.push_back(s.back() + ds);
  }
  const double total = s.back();
  if (total < 1e-6) {
    out.assign(sample_num, pts.front());
    return out;
  }
  out.reserve(sample_num);
  for (size_t k = 0; k < sample_num; ++k) {
    const double target = (static_cast<double>(k) / static_cast<double>(sample_num - 1)) * total;
    auto it = std::upper_bound(s.begin(), s.end(), target);
    size_t idx = std::min<size_t>(std::max<size_t>(1, it - s.begin()), s.size() - 1);
    const double s0 = s[idx - 1];
    const double s1 = s[idx];
    const double ratio = (s1 > s0) ? (target - s0) / (s1 - s0) : 0.0;
    out.push_back(pts[idx - 1] + ratio * (pts[idx] - pts[idx - 1]));
  }
  return out;
}

std::vector<Eigen::Vector2d> simplifyCollinearPolyline(
    const std::vector<Eigen::Vector2d>& pts) {
  std::vector<Eigen::Vector2d> out;
  out.reserve(pts.size());
  for (const auto& point : pts) {
    if (!out.empty() && (point - out.back()).norm() <= 1e-6) {
      continue;
    }
    out.push_back(point);
    while (out.size() >= 3) {
      const Eigen::Vector2d v0 = out[out.size() - 2] - out[out.size() - 3];
      const Eigen::Vector2d v1 = out[out.size() - 1] - out[out.size() - 2];
      if (v0.norm() <= 1e-6 || v1.norm() <= 1e-6) {
        out.erase(out.end() - 2);
        continue;
      }
      const double cross = v0.x() * v1.y() - v0.y() * v1.x();
      const double dot = v0.dot(v1);
      if (std::abs(cross) <= 1e-6 && dot > 0.0) {
        out.erase(out.end() - 2);
      } else {
        break;
      }
    }
  }
  return out;
}

std_msgs::ColorRGBA jetColor(double t) {
  t = std::min(1.0, std::max(0.0, t));
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  const double r = std::min(1.0, std::max(0.0, 1.5 - std::fabs(4.0 * t - 3.0)));
  const double g = std::min(1.0, std::max(0.0, 1.5 - std::fabs(4.0 * t - 2.0)));
  const double b = std::min(1.0, std::max(0.0, 1.5 - std::fabs(4.0 * t - 1.0)));
  color.r = r;
  color.g = g;
  color.b = b;
  return color;
}

std_msgs::ColorRGBA rgba(double r, double g, double b, double a) {
  std_msgs::ColorRGBA color;
  color.r = static_cast<float>(r);
  color.g = static_cast<float>(g);
  color.b = static_cast<float>(b);
  color.a = static_cast<float>(a);
  return color;
}

class NaturalCubicSpline {
 public:
  bool build(const std::vector<double>& x, const std::vector<double>& y) {
    const size_t n = x.size();
    if (n < 2 || y.size() != n) return false;
    x_ = x;
    a_ = y;
    b_.assign(n - 1, 0.0);
    c_.assign(n, 0.0);
    d_.assign(n - 1, 0.0);

    std::vector<double> h(n - 1, 0.0);
    for (size_t i = 0; i + 1 < n; ++i) {
      h[i] = x_[i + 1] - x_[i];
      if (h[i] <= 1e-12) return false;
    }

    std::vector<double> alpha(n - 1, 0.0);
    for (size_t i = 1; i + 1 < n; ++i) {
      alpha[i] = (3.0 / h[i]) * (a_[i + 1] - a_[i]) - (3.0 / h[i - 1]) * (a_[i] - a_[i - 1]);
    }

    std::vector<double> l(n, 0.0), mu(n, 0.0), z(n, 0.0);
    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;

    for (size_t i = 1; i + 1 < n; ++i) {
      l[i] = 2.0 * (x_[i + 1] - x_[i - 1]) - h[i - 1] * mu[i - 1];
      if (std::fabs(l[i]) <= 1e-12) return false;
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n - 1] = 1.0;
    z[n - 1] = 0.0;
    c_[n - 1] = 0.0;

    for (size_t j = n - 1; j-- > 0;) {
      c_[j] = z[j] - mu[j] * c_[j + 1];
      b_[j] = (a_[j + 1] - a_[j]) / h[j] - h[j] * (c_[j + 1] + 2.0 * c_[j]) / 3.0;
      d_[j] = (c_[j + 1] - c_[j]) / (3.0 * h[j]);
    }
    return true;
  }

  double eval(double x_query, double* d1_out = nullptr, double* d2_out = nullptr) const {
    if (x_.empty()) return 0.0;
    size_t i = findSegment(x_query);
    const double dx = x_query - x_[i];
    const double value = a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
    if (d1_out) {
      *d1_out = b_[i] + 2.0 * c_[i] * dx + 3.0 * d_[i] * dx * dx;
    }
    if (d2_out) {
      *d2_out = 2.0 * c_[i] + 6.0 * d_[i] * dx;
    }
    return value;
  }

 private:
  size_t findSegment(double x_query) const {
    if (x_query <= x_.front()) return 0;
    if (x_query >= x_.back()) return x_.size() - 2;
    auto it = std::upper_bound(x_.begin(), x_.end(), x_query);
    size_t idx = static_cast<size_t>(std::distance(x_.begin(), it));
    return std::max<size_t>(1, idx) - 1;
  }

  std::vector<double> x_;
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
};

class QuadrotorSimulator : public NumericalSimulator<QuadrotorSystem>
{
 public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Base_t = NumericalSimulator<QuadrotorSystem>;
  using CarState_t = Eigen::Matrix<double, 16, 1>; // p, v, q, a, w
  struct UgvHorizonCompareSnapshot {
    bool valid = false;
    double sim_time = std::numeric_limits<double>::quiet_NaN();
    double prediction_dt = std::numeric_limits<double>::quiet_NaN();
    double horizon_rms_xy = std::numeric_limits<double>::quiet_NaN();
    double horizon_max_xy = std::numeric_limits<double>::quiet_NaN();
    double horizon_final_xy = std::numeric_limits<double>::quiet_NaN();
    double boundary_drift_rms = std::numeric_limits<double>::quiet_NaN();
    double boundary_drift_max = std::numeric_limits<double>::quiet_NaN();
    std::size_t boundary_drift_samples = 0u;
    std::vector<CarState_t> predicted_window;
    std::vector<CarState_t> actual_window;
  };

  struct SolverObstacleDriftRecord {
    int uav_idx = -1;
    std::size_t obstacle_slot = 0u;
    std::string obstacle_key;
    std::size_t horizon_idx = 0u;
    double radius = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d solver_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d real_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d diff = Eigen::Vector3d::Zero();
    double diff_norm_xy = std::numeric_limits<double>::quiet_NaN();
    double diff_norm_xyz = std::numeric_limits<double>::quiet_NaN();
  };

  struct SolverObstacleDriftSnapshot {
    bool valid = false;
    double sim_time = std::numeric_limits<double>::quiet_NaN();
    double prediction_dt = std::numeric_limits<double>::quiet_NaN();
    double drift_rms_xy = std::numeric_limits<double>::quiet_NaN();
    double drift_max_xy = std::numeric_limits<double>::quiet_NaN();
    std::size_t drift_samples = 0u;
    std::vector<SolverObstacleDriftRecord> records;
  };

  struct ExternalAvoidanceSnapshot {
    bool valid = false;
    double sim_time = std::numeric_limits<double>::quiet_NaN();
    std::string status = "disabled";
    std::string self_state_source = "disabled";
    Eigen::Vector2d self_position_world = Eigen::Vector2d::Zero();
    Eigen::Vector2d preferred_velocity_world = Eigen::Vector2d::Zero();
    Eigen::Vector2d filtered_velocity_world = Eigen::Vector2d::Zero();
    Eigen::Vector2d self_velocity_world = Eigen::Vector2d::Zero();
    bool intervened = false;
    bool feasible = true;
    bool used_fallback = false;
    bool emergency_stop = false;
    std::size_t static_obstacle_count = 0u;
    std::size_t peer_count = 0u;
    std::size_t velocity_obstacle_count = 0u;
    std::size_t official_overlap_gate_count = 0u;
    std::size_t official_overlap_collision_count = 0u;
    std::size_t official_overlap_cone_changed_count = 0u;
    std::size_t official_overlap_output_changed_count = 0u;
    std::size_t candidate_count = 0u;
    std::size_t safe_candidate_count = 0u;
    double time_horizon = 0.0;
    std::size_t max_neighbors = 0u;
    double safety_margin = 0.0;
    std::size_t polygon_sides = 0u;
    double velocity_window = 0.0;
    double candidate_resolution = 0.0;
    double fallback_penalty_weight = 0.0;
    double static_neighbor_distance = 0.0;
    double peer_neighbor_distance = 0.0;
    double intervention_norm = 0.0;
    double fallback_min_ttc = std::numeric_limits<double>::infinity();
    double official_overlap_output_delta_norm = 0.0;
    double filter_time_ms = 0.0;
  };
 
  // constructor
  QuadrotorSimulator(double system_dt, 
      double control_dt, 
       State_t &x0, 
      std::shared_ptr<QuadrotorSystem> system_ptr, 
      std::shared_ptr<coni_mpc::NumSimMpc> control_system_ptr, 
      const std::vector<RelativeTrajectoryPoint> &trajectory, 
      const std::function<RelativeTrajectory(size_t, double, double, size_t, const std::vector<RelativeTrajectoryPoint>&)> &plan_strategy, 
      const std::vector<CarState_t> &car_trajectory, 
      const NonInertialPredictorConfig& non_inertial_predictor_cfg,
      const std::function<std::vector<CarState_t>(size_t, double, double, size_t, const std::vector<CarState_t>&)> &car_plan_strategy, 
      double trajectory_duration, 
      size_t trajectory_sample_num, 
      double car_trajectory_duration, 
      size_t car_trajectory_sample_num, 
      double uav_radius,
      double safety_margin,
      double failure_distance_margin,
      const std::vector<StaticObstacle>& obstacles,
      std::function<void(double)> dynamic_obstacle_callback,
      std::function<void(int, const Eigen::Vector3d&, const StaticObstacle&, double)> collision_callback,
      int quad_index,
      bool owns_dynamic_obstacle_updates,
      bool freeze_ugv_rollout,
      const ExternalAvoidanceConfig& external_avoidance_cfg,
      bool verbose = true) : 
      Base_t(system_dt, control_dt, x0, system_ptr, verbose), 
      control_system_ptr_(control_system_ptr), 
      trajectory_(trajectory), 
      plan_strategy_(plan_strategy), 
      car_trajectory_(car_trajectory), 
      car_plan_strategy_(car_plan_strategy), 
      non_inertial_predictor_(
          std::max(0, static_cast<int>(control_system_ptr_->WINDOW_NUM) - 1),
          control_system_ptr_->getPredictionDt(),
          non_inertial_predictor_cfg.warmup_frames,
          non_inertial_predictor_cfg.ema_alpha_min,
          non_inertial_predictor_cfg.ema_alpha_max,
          non_inertial_predictor_cfg.var_threshold,
          non_inertial_predictor_cfg.gamma_a,
          non_inertial_predictor_cfg.gamma_alpha,
          non_inertial_predictor_cfg.speed_max,
          non_inertial_predictor_cfg.omega_z_max,
          non_inertial_predictor_cfg.a_max,
          non_inertial_predictor_cfg.alpha_max,
          non_inertial_predictor_cfg.exit_history_size,
          coni_mpc::parseTrendPredictionMode(non_inertial_predictor_cfg.trend_mode),
          non_inertial_predictor_cfg.long_trend,
          non_inertial_predictor_cfg.angular_trend,
          non_inertial_predictor_cfg.debug_log,
          non_inertial_predictor_cfg.turn_exit_omega_hold,
          non_inertial_predictor_cfg.turn_exit_omega_low,
          non_inertial_predictor_cfg.accel_exit_a_hold,
          non_inertial_predictor_cfg.accel_exit_a_low,
          non_inertial_predictor_cfg.cruise_speed_ratio,
          non_inertial_predictor_cfg.stop_speed_threshold),
      predictor_last_sim_time_(std::numeric_limits<double>::quiet_NaN()),
      current_beta_non_initialized_(false),
	      current_beta_non_prev_sim_time_(std::numeric_limits<double>::quiet_NaN()),
	      current_beta_non_prev_omega_non_(Eigen::Vector3d::Zero()),
	      current_beta_non_last_value_(Eigen::Vector3d::Zero()),
	      freeze_ugv_rollout_(freeze_ugv_rollout),
	      external_avoidance_cfg_(external_avoidance_cfg),
	      damped_rollout_params_(non_inertial_predictor_cfg.damped_rollout),
	      damped_rollout_acc_initialized_(false),
	      damped_rollout_prev_acc_non_xy_(Eigen::Vector2d::Zero()),
	      predictor_speed_max_(std::max(0.0, non_inertial_predictor_cfg.speed_max)),
      predictor_omega_z_max_(
          std::max(0.0, std::abs(non_inertial_predictor_cfg.omega_z_max))),
      trajectory_duration_(trajectory_duration), 
      trajectory_sample_num_(trajectory_sample_num), 
      car_trajectory_duration_(car_trajectory_duration), 
      car_trajectory_sample_num_(car_trajectory_sample_num),
      uav_radius_(std::max(0.0, uav_radius)),
      safety_margin_(std::max(0.0, safety_margin)),
      failure_distance_margin_(std::max(0.0, failure_distance_margin)),
      obstacles_(obstacles),
      dynamic_obstacle_callback_(std::move(dynamic_obstacle_callback)),
      collision_callback_(std::move(collision_callback)),
      vehicle_index_(quad_index),
      owns_dynamic_obstacle_updates_(owns_dynamic_obstacle_updates),
      collision_detected_(false)
  {
  }   
  

  // FIXME: if this virtual destructor is not specified or
  // finish() is not called (i.e., the threads are joined because of the destructor)
  // then the following overriden functions are not properly dispatched
  virtual ~QuadrotorSimulator() { finish(); }

  void setObstacles(const std::vector<StaticObstacle>& obstacles) {
    std::lock_guard<std::mutex> lock(obstacles_mtx_);
    obstacles_ = obstacles;
  }

  const UgvHorizonCompareSnapshot& getLastUgvHorizonCompareSnapshot() const {
    return last_ugv_horizon_compare_;
  }

  const SolverObstacleDriftSnapshot& getLastSolverObstacleDriftSnapshot() const {
    return last_solver_obstacle_drift_;
  }

  const ExternalAvoidanceSnapshot& getLastExternalAvoidanceSnapshot() const {
    return last_external_avoidance_;
  }

  const Control_t& getLastAppliedControlWorld() const {
    return last_applied_control_world_;
  }

  Eigen::Vector3d getWorldPosition() {
    const State_t state = getStateSnapshot();
    return Eigen::Vector3d(state(0), state(1), state(2));
  }

  double getTruthMinPlanarSurfaceClearance() {
    const State_t state = getStateSnapshot();
    const Eigen::Vector2d position(state(0), state(1));
    double minimum = std::numeric_limits<double>::infinity();

    std::vector<Eigen::Vector3d> peer_positions;
    std::vector<Eigen::Vector3d> peer_velocities;
    std::vector<bool> peer_valid;
    control_system_ptr_->getSharedPositions(peer_positions,
                                            peer_velocities,
                                            peer_valid);
    for (std::size_t index = 0U; index < peer_positions.size(); ++index) {
      if (static_cast<int>(index) == vehicle_index_ ||
          index >= peer_valid.size() || !peer_valid[index]) {
        continue;
      }
      minimum = std::min(
          minimum,
          (peer_positions[index].head<2>() - position).norm() -
              2.0 * uav_radius_);
    }

    {
      std::lock_guard<std::mutex> lock(obstacles_mtx_);
      for (const auto& obstacle : obstacles_) {
        minimum = std::min(
            minimum,
            (obstacle.position.head<2>() - position).norm() -
                (uav_radius_ + obstacle.radius));
      }
    }
    return std::isfinite(minimum)
               ? minimum
               : std::numeric_limits<double>::quiet_NaN();
  }

  void captureCycleQuadOdomSnapshot(int i, const ros::Time& stamp) {
    cycle_quad_odom_snapshot_ =
        buildQuadOdomFromState(getStateSnapshot(), i, stamp);
    cycle_quad_odom_snapshot_valid_ = true;
  }

  void commitCycleQuadOdomSnapshot() {
    if (!cycle_quad_odom_snapshot_valid_) {
      return;
    }
    coni_mpc::NumSimMpc::setSharedQuadOdom(cycle_quad_odom_snapshot_,
                                           vehicle_index_);
    cycle_quad_odom_snapshot_valid_ = false;
  }

  static Eigen::Quaterniond normalizedCarQuaternion(const CarState_t& state) {
    Eigen::Quaterniond q(state(6), state(7), state(8), state(9));
    if (q.norm() > 1e-9) {
      q.normalize();
    } else {
      q = Eigen::Quaterniond::Identity();
    }
    return q;
  }

  static double planarYawFromQuaternion(const Eigen::Quaterniond& q) {
    const Eigen::Matrix3d rotation = q.toRotationMatrix();
    return std::atan2(rotation(1, 0), rotation(0, 0));
  }

  static Eigen::Quaterniond quaternionFromRotationVector(
      const Eigen::Vector3d& rotation_vector) {
    const double angle = rotation_vector.norm();
    if (angle <= 1e-12) {
      return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(angle, rotation_vector / angle));
  }

  nav_msgs::Odometry buildQuadOdomFromState(const State_t& state,
                                            int i,
                                            const ros::Time& stamp) const {
    nav_msgs::Odometry quad_odom;
    quad_odom.header.frame_id = acado_mpc_common::worldFrameId();
    quad_odom.header.stamp = stamp;
    quad_odom.child_frame_id = uavTfFrameId(i);
    quad_odom.pose.pose.position.x = state(0);
    quad_odom.pose.pose.position.y = state(1);
    quad_odom.pose.pose.position.z = state(2);
    quad_odom.twist.twist.linear.x = state(3);
    quad_odom.twist.twist.linear.y = state(4);
    quad_odom.twist.twist.linear.z = state(5);
    Eigen::Quaterniond quad_orientation(state(6), state(7), state(8), state(9));
    quad_orientation.normalize();
    quad_odom.pose.pose.orientation.w = quad_orientation.w();
    quad_odom.pose.pose.orientation.x = quad_orientation.x();
    quad_odom.pose.pose.orientation.y = quad_orientation.y();
    quad_odom.pose.pose.orientation.z = quad_orientation.z();
    quad_odom.twist.twist.angular.x = 0.0;
    quad_odom.twist.twist.angular.y = 0.0;
    quad_odom.twist.twist.angular.z = system_ptr_->getControlAction()(3);
    return quad_odom;
  }

  std::vector<CarState_t> buildFrozenCarStageWindow(
      const CarState_t& current_state) const {
    std::vector<CarState_t> result;
    const std::size_t horizon_steps = control_system_ptr_->WINDOW_NUM;
    if (horizon_steps == 0u) {
      return result;
    }

    const double prediction_dt = control_system_ptr_->getPredictionDt();
    if (!(std::isfinite(prediction_dt) && prediction_dt > 1e-6)) {
      result.assign(horizon_steps, current_state);
      return result;
    }

    result.reserve(horizon_steps);

    double x = current_state(0);
    double y = current_state(1);
    double z = current_state(2);
    const double vz = current_state(5);
    const double omega_x = current_state(13);
    const double omega_y = current_state(14);
    const double omega_z = current_state(15);

    const Eigen::Quaterniond q0 = normalizedCarQuaternion(current_state);
    double yaw = planarYawFromQuaternion(q0);
    const Eigen::Vector2d heading(std::cos(yaw), std::sin(yaw));
    const Eigen::Vector2d velocity_xy(current_state(3), current_state(4));
    const double speed = heading.dot(velocity_xy);

    for (std::size_t k = 0; k < horizon_steps; ++k) {
      CarState_t state = CarState_t::Zero();
      const double vx = speed * std::cos(yaw);
      const double vy = speed * std::sin(yaw);
      const Eigen::AngleAxisd yaw_rot(yaw, Eigen::Vector3d::UnitZ());
      const Eigen::Quaterniond q(yaw_rot);

      state(0) = x;
      state(1) = y;
      state(2) = z;
      state(3) = vx;
      state(4) = vy;
      state(5) = vz;
      state(6) = q.w();
      state(7) = q.x();
      state(8) = q.y();
      state(9) = q.z();
      // Both frozen controllers use the same constant-(speed, yaw-rate)
      // UGV rollout.  Their only difference is the MPC coordinate frame.
      const Eigen::Vector3d a_world =
          q * Eigen::Vector3d(0.0, speed * omega_z, 0.0);
      state(10) = a_world.x();
      state(11) = a_world.y();
      state(12) = a_world.z();
      state(13) = omega_x;
      state(14) = omega_y;
      state(15) = omega_z;
      result.push_back(state);

      if (std::abs(omega_z) > 1e-4) {
        // Constant-curvature integration using the frozen current yaw rate.
        x += (speed / omega_z) * (std::sin(yaw + omega_z * prediction_dt) - std::sin(yaw));
        y += (speed / omega_z) * (-std::cos(yaw + omega_z * prediction_dt) + std::cos(yaw));
        yaw += omega_z * prediction_dt;
      } else {
        // The zero-yaw-rate limit of the constant-curvature model.
        x += vx * prediction_dt;
        y += vy * prediction_dt;
      }
      z += vz * prediction_dt;
    }

    return result;
  }

  void buildFrozenNonInertialProfiles(
      const CarState_t& current_state,
      coni_mpc::NumSimMpc::NonInertialProfile& omega_profile,
      coni_mpc::NumSimMpc::NonInertialProfile& beta_profile,
      coni_mpc::NumSimMpc::NonInertialProfile& a_car_profile) const {
    omega_profile.setZero();
    beta_profile.setZero();
    a_car_profile.setZero();
    const Eigen::Quaterniond W_q_non = normalizedCarQuaternion(current_state);
    const Eigen::Vector3d omega_world(
        current_state(13), current_state(14), current_state(15));
    const Eigen::Vector3d omega_non = W_q_non.inverse() * omega_world;
    const double yaw = planarYawFromQuaternion(W_q_non);
    const Eigen::Vector2d heading(std::cos(yaw), std::sin(yaw));
    const Eigen::Vector2d velocity_xy(current_state(3), current_state(4));
    const double speed = heading.dot(velocity_xy);
    const double a_lat = speed * omega_non.z();
    for (int k = 0; k < omega_profile.cols(); ++k) {
      omega_profile.col(k) = omega_non;
      // beta stays zero (matching original CO-MPC behavior)
      a_car_profile.col(k) = Eigen::Vector3d(0.0, a_lat, 0.0);
    }
  }

  Eigen::Vector3d computeCurrentBetaNonCausal(const CarState_t& current_state,
                                              double sim_time) {
    const Eigen::Quaterniond W_q_non = normalizedCarQuaternion(current_state);
    const Eigen::Vector3d omega_world(
        current_state(13), current_state(14), current_state(15));
    const Eigen::Vector3d omega_non = W_q_non.inverse() * omega_world;

    Eigen::Vector3d beta_non = Eigen::Vector3d::Zero();
    if (current_beta_non_initialized_) {
      const bool valid_dt = std::isfinite(current_beta_non_prev_sim_time_) &&
                            sim_time > current_beta_non_prev_sim_time_ + 1e-6;
      if (valid_dt) {
        const double dt = sim_time - current_beta_non_prev_sim_time_;
        beta_non = (omega_non - current_beta_non_prev_omega_non_) / dt;
      } else {
        beta_non = current_beta_non_last_value_;
      }
    }

    current_beta_non_prev_omega_non_ = omega_non;
    current_beta_non_prev_sim_time_ = sim_time;
    current_beta_non_last_value_ = beta_non;
    current_beta_non_initialized_ = true;
    return beta_non;
  }

  void alignCurrentNonInertialProfileCol0(
      const CarState_t& current_state,
      const Eigen::Vector3d& current_beta_non,
      coni_mpc::NumSimMpc::NonInertialProfile& omega_profile,
      coni_mpc::NumSimMpc::NonInertialProfile& beta_profile,
      coni_mpc::NumSimMpc::NonInertialProfile& a_car_profile) const {
    if (omega_profile.cols() <= 0 || beta_profile.cols() <= 0 ||
        a_car_profile.cols() <= 0) {
      return;
    }

    const Eigen::Quaterniond W_q_non = normalizedCarQuaternion(current_state);
    const Eigen::Vector3d omega_world(
        current_state(13), current_state(14), current_state(15));
    const Eigen::Vector3d omega_non = W_q_non.inverse() * omega_world;

    omega_profile.col(0) = omega_non;
    beta_profile.col(0) = current_beta_non;
    // Stage-0 online data must remain aligned with the estimator-side
    // legacy current-time convention rather than pre-applying a predicted
    // stage acceleration on top of the already-fixed current state.
    a_car_profile.col(0).setZero();
  }

  std::vector<CarState_t> buildPredictedCarStageWindow(
      const CarState_t& current_state,
      const coni_mpc::NumSimMpc::NonInertialProfile& omega_non_profile,
      const coni_mpc::NumSimMpc::NonInertialProfile& beta_non_profile,
      const coni_mpc::NumSimMpc::NonInertialProfile& a_car_non_profile) const {
    std::vector<CarState_t> result;
    const std::size_t horizon_steps = control_system_ptr_->WINDOW_NUM;
    if (horizon_steps == 0u) {
      return result;
    }

    const double prediction_dt = control_system_ptr_->getPredictionDt();
    if (!(std::isfinite(prediction_dt) && prediction_dt > 1e-6)) {
      result.assign(horizon_steps, current_state);
      return result;
    }

    result.reserve(horizon_steps);

    const auto clampAbs = [](double value, double max_abs_value) {
      if (max_abs_value <= 1e-9) {
        return value;
      }
      return std::max(-max_abs_value, std::min(max_abs_value, value));
    };

    Eigen::Vector3d position(current_state(0), current_state(1), current_state(2));
    const double z_fixed = current_state(2);
    const double vz_fixed = current_state(5);
    double yaw = planarYawFromQuaternion(normalizedCarQuaternion(current_state));
    const Eigen::Vector2d heading(std::cos(yaw), std::sin(yaw));
    const Eigen::Vector2d velocity_xy(current_state(3), current_state(4));
    double speed_long = std::max(0.0, heading.dot(velocity_xy));
    if (predictor_speed_max_ > 1e-9) {
      speed_long = std::min(speed_long, predictor_speed_max_);
    }

    const auto sampleProfile = [](
                                   const coni_mpc::NumSimMpc::NonInertialProfile& profile,
                                   std::size_t idx) -> Eigen::Vector3d {
      if (profile.cols() <= 0) {
        return Eigen::Vector3d::Zero();
      }
      return profile.col(
          std::min<int>(static_cast<int>(idx), profile.cols() - 1));
    };
    for (std::size_t k = 0; k < horizon_steps; ++k) {
      const Eigen::Vector3d omega_non_raw = sampleProfile(omega_non_profile, k);
      const Eigen::Vector3d beta_non_raw = sampleProfile(beta_non_profile, k);
      const Eigen::Vector3d a_non_raw = sampleProfile(a_car_non_profile, k);
      const double omega_z = clampAbs(omega_non_raw.z(), predictor_omega_z_max_);
      const double beta_z = beta_non_raw.z();
      const double a_long = a_non_raw.x();
      const double a_lat = speed_long * omega_z;
      const Eigen::Vector3d omega_non(0.0, 0.0, omega_z);
      const Eigen::Vector3d beta_non(0.0, 0.0, beta_z);
      const Eigen::Vector3d a_non(a_long, a_lat, 0.0);
      const Eigen::Vector3d velocity_world(speed_long * std::cos(yaw),
                                           speed_long * std::sin(yaw),
                                           vz_fixed);
      const Eigen::Quaterniond W_q_non(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
      const Eigen::Vector3d a_world = W_q_non * a_non;
      const Eigen::Vector3d omega_world = W_q_non * omega_non;

      CarState_t state = CarState_t::Zero();
      state(0) = position.x();
      state(1) = position.y();
      state(2) = z_fixed;
      state(3) = velocity_world.x();
      state(4) = velocity_world.y();
      state(5) = vz_fixed;
      state(6) = W_q_non.w();
      state(7) = W_q_non.x();
      state(8) = W_q_non.y();
      state(9) = W_q_non.z();
      state(10) = a_world.x();
      state(11) = a_world.y();
      state(12) = a_world.z();
      state(13) = omega_world.x();
      state(14) = omega_world.y();
      state(15) = omega_world.z();
      result.push_back(state);

      if (k + 1u < horizon_steps) {
        const Eigen::Vector3d omega_non_next_raw =
            sampleProfile(omega_non_profile, k + 1u);
        const Eigen::Vector3d beta_non_next_raw =
            sampleProfile(beta_non_profile, k + 1u);
        const Eigen::Vector3d a_non_next_raw =
            sampleProfile(a_car_non_profile, k + 1u);
        const double omega_z_next =
            clampAbs(omega_non_next_raw.z(), predictor_omega_z_max_);
        const double beta_z_next = beta_non_next_raw.z();
        const double a_long_next = a_non_next_raw.x();
        const double beta_z_mid = 0.5 * (beta_z + beta_z_next);
        double omega_z_end = omega_z + beta_z_mid * prediction_dt;
        omega_z_end = clampAbs(0.5 * (omega_z_end + omega_z_next),
                               predictor_omega_z_max_);
        const double omega_z_mid = 0.5 * (omega_z + omega_z_end);
        const double delta_yaw = omega_z_mid * prediction_dt;
        const double a_long_mid = 0.5 * (a_long + a_long_next);
        double next_speed = std::max(0.0, speed_long + a_long_mid * prediction_dt);
        if (predictor_speed_max_ > 1e-9) {
          next_speed = std::min(next_speed, predictor_speed_max_);
        }
        const double avg_speed = 0.5 * (speed_long + next_speed);

        if (std::abs(omega_z_mid) > 1e-4) {
          position.x() +=
              (avg_speed / omega_z_mid) *
              (std::sin(yaw + delta_yaw) - std::sin(yaw));
          position.y() +=
              (avg_speed / omega_z_mid) *
              (-std::cos(yaw + delta_yaw) + std::cos(yaw));
        } else {
          position.x() += avg_speed * std::cos(yaw) * prediction_dt;
          position.y() += avg_speed * std::sin(yaw) * prediction_dt;
        }
        position.z() = z_fixed;
        yaw += delta_yaw;
        speed_long = next_speed;
      }
    }
    return result;
  }

  static coni_mpc::NumSimMpc::NonInertialProfile denseToStageProfile(
      const Eigen::MatrixXd& dense) {
    coni_mpc::NumSimMpc::NonInertialProfile profile =
        coni_mpc::NumSimMpc::NonInertialProfile::Zero();
    if (dense.rows() <= 0 || dense.cols() <= 0) {
      return profile;
    }

    const int copy_rows = std::min<int>(profile.rows(), dense.rows());
    const int copy_cols = std::min<int>(profile.cols(), dense.cols());
    profile.block(0, 0, copy_rows, copy_cols) =
        dense.block(0, 0, copy_rows, copy_cols);
    if (copy_cols > 0 && copy_cols < profile.cols()) {
      for (int col = copy_cols; col < profile.cols(); ++col) {
        profile.col(col) = profile.col(copy_cols - 1);
      }
    }
    return profile;
  }

  void buildDampedVaryingCurvatureProfiles(
      const CarState_t& current_state,
      const Eigen::Vector3d& /*current_beta_non*/,
      double dt_update,
      coni_mpc::NumSimMpc::NonInertialProfile& omega_profile,
      coni_mpc::NumSimMpc::NonInertialProfile& beta_profile,
      coni_mpc::NumSimMpc::NonInertialProfile& a_car_profile) {
    omega_profile.setZero();
    beta_profile.setZero();
    a_car_profile.setZero();

    const Eigen::Quaterniond W_q_non = normalizedCarQuaternion(current_state);
    const Eigen::Vector3d v_world(current_state(3), current_state(4), current_state(5));
    const Eigen::Vector3d omega_world(
        current_state(13), current_state(14), current_state(15));
    const Eigen::Vector3d omega_non = W_q_non.inverse() * omega_world;

    Eigen::MatrixXd omega_tmp, beta_tmp, a_tmp;
    non_inertial_predictor_.updateAndPredict(
        v_world, omega_non, W_q_non, dt_update,
        omega_tmp, beta_tmp, a_tmp);

    const int cols = std::min<int>(omega_profile.cols(),
                                   static_cast<int>(omega_tmp.cols()));
    if (cols > 0) {
      omega_profile.leftCols(cols) = omega_tmp.leftCols(cols);
      beta_profile.leftCols(cols) = beta_tmp.leftCols(cols);
      a_car_profile.leftCols(cols) = a_tmp.leftCols(cols);
    }
  }

  std::vector<CarState_t> sampleCarStageWindow(
      double sim_time,
      coni_mpc::NumSimMpc::NonInertialProfile& omega_stage_profile,
      coni_mpc::NumSimMpc::NonInertialProfile& beta_stage_profile,
      coni_mpc::NumSimMpc::NonInertialProfile& a_car_stage_profile) {
    const auto actual_window = sampleActualCarStageWindow(sim_time);
    if (actual_window.empty()) {
      omega_stage_profile.setZero();
      beta_stage_profile.setZero();
      a_car_stage_profile.setZero();
      return {};
    }

    const CarState_t& current_state = actual_window.front();
    const Eigen::Vector3d current_beta_non =
        computeCurrentBetaNonCausal(current_state, sim_time);
    if (freeze_ugv_rollout_) {
      buildFrozenNonInertialProfiles(current_state,
                                     omega_stage_profile,
                                     beta_stage_profile,
                                     a_car_stage_profile);
      alignCurrentNonInertialProfileCol0(current_state,
                                         Eigen::Vector3d::Zero(),
                                         omega_stage_profile,
                                         beta_stage_profile,
                                         a_car_stage_profile);
      return buildFrozenCarStageWindow(current_state);
    }

    const double dt_update =
        (std::isfinite(predictor_last_sim_time_) && sim_time > predictor_last_sim_time_)
            ? (sim_time - predictor_last_sim_time_)
            : control_dt_;

    buildDampedVaryingCurvatureProfiles(current_state,
                                        current_beta_non,
                                        dt_update,
                                        omega_stage_profile,
                                        beta_stage_profile,
                                        a_car_stage_profile);
    predictor_last_sim_time_ = sim_time;

    // Use frozen circular-arc for the MPC reference trajectory (car position).
    // EMA force profiles above are kept for fictitious-force compensation.
    // Rationale: EMA omega lags measured omega at k=0, causing a ~1.5 cm/step
    // world-y under-prediction error that accumulates into a ~12 cm car-x lag.
    // The frozen arc (current omega_z, exact at k=1) eliminates this bias.
    return buildFrozenCarStageWindow(current_state);
  }

  std::vector<CarState_t> sampleActualCarStageWindow(double sim_time) const {
    return car_plan_strategy_(control_system_ptr_->WINDOW_NUM, sim_time,
                              car_trajectory_duration_,
                              car_trajectory_sample_num_,
                              car_trajectory_);
  }

  UgvHorizonCompareSnapshot buildUgvHorizonCompareSnapshot(
      double sim_time,
      const std::vector<CarState_t>& injected_window,
      const coni_mpc::NumSimMpc::NonInertialProfile& beta_stage_profile) const {
    UgvHorizonCompareSnapshot snapshot;
    snapshot.sim_time = sim_time;
    snapshot.prediction_dt = control_system_ptr_->getPredictionDt();
    snapshot.actual_window = sampleActualCarStageWindow(sim_time);
    if (snapshot.actual_window.empty() || injected_window.empty()) {
      return snapshot;
    }

    const double dt = snapshot.prediction_dt;
    if (!(std::isfinite(dt) && dt > 1e-6)) {
      return snapshot;
    }

    const std::size_t horizon_steps =
        std::min(snapshot.actual_window.size(), injected_window.size());
    if (horizon_steps == 0u) {
      return snapshot;
    }

    snapshot.predicted_window.reserve(horizon_steps);

    for (std::size_t k = 0; k < horizon_steps; ++k) {
      // 直接采纳注入给 MPC 的精确预测状态，不再做重复错误积分
      snapshot.predicted_window.push_back(injected_window[k]);
    }

    const std::size_t sample_count =
        std::min(snapshot.predicted_window.size(), snapshot.actual_window.size());
    if (sample_count == 0u) {
      return snapshot;
    }

    double sum_sq_xy = 0.0;
    double max_xy = 0.0;
    double final_xy = std::numeric_limits<double>::quiet_NaN();
    double boundary_sum_sq = 0.0;
    double boundary_max = 0.0;
    std::size_t boundary_samples = 0u;
    std::vector<StaticObstacle> obstacles_copy;
    {
      std::lock_guard<std::mutex> lock(obstacles_mtx_);
      obstacles_copy = obstacles_;
    }
    for (std::size_t k = 0; k < sample_count; ++k) {
      const Eigen::Vector2d pred_xy(snapshot.predicted_window[k](0),
                                    snapshot.predicted_window[k](1));
      const Eigen::Vector2d actual_xy(snapshot.actual_window[k](0),
                                      snapshot.actual_window[k](1));
      const double err_xy = (pred_xy - actual_xy).norm();
      sum_sq_xy += err_xy * err_xy;
      max_xy = std::max(max_xy, err_xy);
      final_xy = err_xy;

      if (!obstacles_copy.empty() && k > 0u) {
        const CarState_t& pred = snapshot.predicted_window[k];
        const CarState_t& actual = snapshot.actual_window[k];
        const Eigen::Vector3d pred_position(pred(0), pred(1), pred(2));
        const Eigen::Vector3d actual_position(actual(0), actual(1), actual(2));
        const Eigen::Quaterniond pred_q = normalizedCarQuaternion(pred);
        const Eigen::Quaterniond actual_q = normalizedCarQuaternion(actual);
        for (const auto& obstacle : obstacles_copy) {
          const Eigen::Vector3d solver_boundary_center =
              pred_q.inverse() * (obstacle.position - pred_position);
          const Eigen::Vector3d physical_boundary_center =
              actual_q.inverse() * (obstacle.position - actual_position);
          const double drift =
              (solver_boundary_center - physical_boundary_center).norm();
          boundary_sum_sq += drift * drift;
          boundary_max = std::max(boundary_max, drift);
          ++boundary_samples;
        }
      }
    }

    snapshot.horizon_rms_xy =
        std::sqrt(sum_sq_xy / static_cast<double>(sample_count));
    snapshot.horizon_max_xy = max_xy;
    snapshot.horizon_final_xy = final_xy;
    if (boundary_samples > 0u) {
      snapshot.boundary_drift_rms =
          std::sqrt(boundary_sum_sq / static_cast<double>(boundary_samples));
      snapshot.boundary_drift_max = boundary_max;
      snapshot.boundary_drift_samples = boundary_samples;
    }
    snapshot.valid = true;
    return snapshot;
  }

  SolverObstacleDriftSnapshot buildSolverObstacleDriftSnapshot(double sim_time) const {
    SolverObstacleDriftSnapshot snapshot;
    snapshot.sim_time = sim_time;
    snapshot.prediction_dt = control_system_ptr_->getPredictionDt();

    const auto& solver_snapshot =
        control_system_ptr_->getLastSolverObstacleProfileSnapshot();
    if (!solver_snapshot.valid || solver_snapshot.solver_profiles.empty()) {
      return snapshot;
    }
    const auto actual_window = sampleActualCarStageWindow(sim_time);
    if (actual_window.empty()) {
      return snapshot;
    }
    const double dt = snapshot.prediction_dt;
    if (!(std::isfinite(dt) && dt > 1e-6)) {
      return snapshot;
    }

    const std::size_t obstacle_count = std::min(
        solver_snapshot.solver_profiles.size(),
        solver_snapshot.world_obstacles.size());
    if (obstacle_count == 0u) {
      return snapshot;
    }
    const std::size_t horizon_steps =
        std::min<std::size_t>(actual_window.size(),
                              static_cast<std::size_t>(acado_mpc::kSamples + 1));
    if (horizon_steps <= 1u) {
      return snapshot;
    }

    double sum_sq_xy = 0.0;
    double max_xy = 0.0;
    for (std::size_t obs_idx = 0; obs_idx < obstacle_count; ++obs_idx) {
      const auto& profile = solver_snapshot.solver_profiles[obs_idx];
      const auto& world_obstacle = solver_snapshot.world_obstacles[obs_idx];
      const std::string obstacle_key =
          (obs_idx < solver_snapshot.keys.size())
              ? solver_snapshot.keys[obs_idx]
              : ("slot:" + std::to_string(obs_idx));
      for (std::size_t k = 1u; k < horizon_steps; ++k) {
        const auto solver_obstacle = profile.col(static_cast<int>(k));
        const Eigen::Vector3d solver_position(
            solver_obstacle(0), solver_obstacle(1), solver_obstacle(2));

        Eigen::Vector3d world_position(
            world_obstacle(0) + world_obstacle(4) * dt * static_cast<double>(k),
            world_obstacle(1) + world_obstacle(5) * dt * static_cast<double>(k),
            world_obstacle(2) + world_obstacle(6) * dt * static_cast<double>(k));

        Eigen::Vector3d real_position = world_position;
        if (!control_system_ptr_->isInertialFrame()) {
          const CarState_t& actual = actual_window[k];
          const Eigen::Vector3d actual_car_position(
              actual(0), actual(1), actual(2));
          const Eigen::Quaterniond actual_q = normalizedCarQuaternion(actual);
          real_position = actual_q.inverse() * (world_position - actual_car_position);
        }
        // CBF obstacles are planar cylinders; the solver overwrites z with the
        // preview UAV height. Keep z aligned so d_bd measures target-frame
        // translation/rotation drift in the planar obstacle location.
        real_position.z() = solver_position.z();

        SolverObstacleDriftRecord record;
        record.uav_idx = solver_snapshot.uav_idx;
        record.obstacle_slot = obs_idx;
        record.obstacle_key = obstacle_key;
        record.horizon_idx = k;
        record.radius = solver_obstacle(3);
        record.solver_position = solver_position;
        record.real_position = real_position;
        record.diff = solver_position - real_position;
        record.diff_norm_xy = record.diff.head<2>().norm();
        record.diff_norm_xyz = record.diff.norm();
        sum_sq_xy += record.diff_norm_xy * record.diff_norm_xy;
        max_xy = std::max(max_xy, record.diff_norm_xy);
        ++snapshot.drift_samples;
        snapshot.records.push_back(record);
      }
    }

    if (snapshot.drift_samples > 0u) {
      snapshot.drift_rms_xy =
          std::sqrt(sum_sq_xy / static_cast<double>(snapshot.drift_samples));
      snapshot.drift_max_xy = max_xy;
      snapshot.valid = true;
    }
    return snapshot;
  }

  RelativeTrajectory buildMpcReferenceWindow(double sim_time) const {
    RelativeTrajectory x_ref = plan_strategy_(control_system_ptr_->WINDOW_NUM,
                                              sim_time,
                                              trajectory_duration_,
                                              trajectory_sample_num_,
                                              trajectory_);
    if (!control_system_ptr_->isInertialFrame() || x_ref.points.empty()) {
      return x_ref;
    }

    const auto& car_window = control_system_ptr_->getCarTrajectoryWindow();
    Eigen::Vector3d fallback_car_pos = control_system_ptr_->getCarPosition();
    Eigen::Quaterniond fallback_W_q_non = control_system_ptr_->getCarOrientation();
    if (fallback_W_q_non.norm() > 1e-9) {
      fallback_W_q_non.normalize();
    } else {
      fallback_W_q_non = Eigen::Quaterniond::Identity();
    }
    const Eigen::Vector3d fallback_v_car_world =
        control_system_ptr_->getCarLinearVelocity();
    const Eigen::Vector3d fallback_omega_non_world =
        control_system_ptr_->getCarAngularVelocity();

    for (size_t idx = 0; idx < x_ref.points.size(); ++idx) {
      auto& point = x_ref.points[idx];
      const Eigen::Vector3d p_ref_non = point.position;
      const Eigen::Vector3d v_ref_non = point.velocity;

      Eigen::Vector3d car_pos_k = fallback_car_pos;
      Eigen::Quaterniond W_q_non_k = fallback_W_q_non;
      Eigen::Vector3d v_car_world_k = fallback_v_car_world;
      Eigen::Vector3d omega_non_world_k = fallback_omega_non_world;
      if (!car_window.empty()) {
        const CarState_t& car_state_k =
            car_window[std::min(idx, car_window.size() - 1)];
        car_pos_k = Eigen::Vector3d(car_state_k(0), car_state_k(1), car_state_k(2));
        v_car_world_k = Eigen::Vector3d(
            car_state_k(3), car_state_k(4), car_state_k(5));
        omega_non_world_k = Eigen::Vector3d(
            car_state_k(13), car_state_k(14), car_state_k(15));
        W_q_non_k = Eigen::Quaterniond(
            car_state_k(6), car_state_k(7), car_state_k(8), car_state_k(9));
        if (W_q_non_k.norm() > 1e-9) {
          W_q_non_k.normalize();
        } else {
          W_q_non_k = Eigen::Quaterniond::Identity();
        }
      }

      const Eigen::Vector3d omega_non_k = W_q_non_k.inverse() * omega_non_world_k;
      point.position = car_pos_k + W_q_non_k * p_ref_non;
      point.velocity =
          v_car_world_k + W_q_non_k * (v_ref_non + omega_non_k.cross(p_ref_non));
      point.orientation = W_q_non_k * point.orientation;
      point.orientation.normalize();
    }
    return x_ref;
  }

  virtual void finishSystemIteration(double sim_time) override
  {
    (void)sim_time;
    checkCollision();
        // Keep the quaternion normalized to avoid drift during integration.
    std::lock_guard<std::mutex> lock(state_mtx_);
    Eigen::Vector4d q_vec = x_.segment<4>(6);
    const double norm = q_vec.norm();
    if (norm > 1e-6) {
      x_.segment<4>(6) = q_vec / norm;
    }
    // printf("[System Iteration]\t t: %.3f \tx: %.3f \ty: %.3f \tz: %.3f\n \tT: %.3f \tw_x: %.3f, \tw_y: %.3f \tw_z: %.3f\n", 
    //     sim_time, x_(0), x_(1), x_(2), 
    //     system_ptr_->getControlAction()(0), 
    //     system_ptr_->getControlAction()(1), 
    //     system_ptr_->getControlAction()(2), 
    //     system_ptr_->getControlAction()(3));
  }
  virtual void car_prepareControllerIteration(double sim_time) 
  {

   nav_msgs::Odometry car_odom;
    coni_mpc::NumSimMpc::NonInertialProfile omega_stage_profile =
        coni_mpc::NumSimMpc::NonInertialProfile::Zero();
    coni_mpc::NumSimMpc::NonInertialProfile beta_stage_profile =
        coni_mpc::NumSimMpc::NonInertialProfile::Zero();
    coni_mpc::NumSimMpc::NonInertialProfile a_car_stage_profile =
        coni_mpc::NumSimMpc::NonInertialProfile::Zero();
    auto car_window = sampleCarStageWindow(sim_time,
                                           omega_stage_profile,
                                           beta_stage_profile,
                                           a_car_stage_profile);
    if (car_window.empty()) {
      return;
    }
    control_system_ptr_->setCarTrajectoryWindow(car_window);
    const auto& car_point = car_window.front();
    car_odom.header.frame_id = acado_mpc_common::worldFrameId();
    car_odom.header.stamp = ros::Time::now();
    car_odom.pose.pose.position.x = car_point(0);
    car_odom.pose.pose.position.y = car_point(1);
    car_odom.pose.pose.position.z = car_point(2);
    car_odom.twist.twist.linear.x = car_point(3);
    car_odom.twist.twist.linear.y = car_point(4);
    car_odom.twist.twist.linear.z = car_point(5);
    car_odom.pose.pose.orientation.w = car_point(6);
    car_odom.pose.pose.orientation.x = car_point(7);
    car_odom.pose.pose.orientation.y = car_point(8);
    car_odom.pose.pose.orientation.z = car_point(9);
    const Eigen::Quaterniond W_q_non = normalizedCarQuaternion(car_point);
    // 修复 Bug：非惯性系动力学已在内部补偿 a_car，这里只应传递纯净的世界重力向量！
    control_system_ptr_->setImu(Eigen::Vector3d(0.0, 0.0, 9.81));
    car_odom.twist.twist.angular.x = car_point(13);
    car_odom.twist.twist.angular.y = car_point(14);
    car_odom.twist.twist.angular.z = car_point(15);
    publishRelativeFrameTf(car_odom);
    control_system_ptr_->setCarNonInertialStageProfiles(omega_stage_profile,
                                                        beta_stage_profile,
                                                        a_car_stage_profile);
    control_system_ptr_->setCarBetaNon(beta_stage_profile.col(0));
    control_system_ptr_->setCarOdom(car_odom);

  }
 
  virtual void prepareControllerIteration(double sim_time,int i) override
  { 
    if (owns_dynamic_obstacle_updates_ && dynamic_obstacle_callback_) {
      dynamic_obstacle_callback_(sim_time);
    }
    last_ugv_horizon_compare_ = UgvHorizonCompareSnapshot();
    nav_msgs::Odometry car_odom;
    coni_mpc::NumSimMpc::NonInertialProfile omega_stage_profile =
        coni_mpc::NumSimMpc::NonInertialProfile::Zero();
    coni_mpc::NumSimMpc::NonInertialProfile beta_stage_profile =
        coni_mpc::NumSimMpc::NonInertialProfile::Zero();
    coni_mpc::NumSimMpc::NonInertialProfile a_car_stage_profile =
        coni_mpc::NumSimMpc::NonInertialProfile::Zero();
    auto car_window = sampleCarStageWindow(sim_time,
                                           omega_stage_profile,
                                           beta_stage_profile,
                                           a_car_stage_profile);
    if (car_window.empty()) {
      return;
    }
    control_system_ptr_->setCarTrajectoryWindow(car_window);
    const auto& car_point = car_window.front();
    car_odom.header.frame_id = acado_mpc_common::worldFrameId();
    car_odom.header.stamp = ros::Time::now();
    car_odom.pose.pose.position.x = car_point(0);
    car_odom.pose.pose.position.y = car_point(1);
    car_odom.pose.pose.position.z = car_point(2);
    car_odom.twist.twist.linear.x = car_point(3);
    car_odom.twist.twist.linear.y = car_point(4);
    car_odom.twist.twist.linear.z = car_point(5);
    car_odom.pose.pose.orientation.w = car_point(6);
    car_odom.pose.pose.orientation.x = car_point(7);
    car_odom.pose.pose.orientation.y = car_point(8);
    car_odom.pose.pose.orientation.z = car_point(9);
    const Eigen::Quaterniond W_q_non = normalizedCarQuaternion(car_point);
    // 修复 Bug：防止双重反向补偿
    control_system_ptr_->setImu(Eigen::Vector3d(0.0, 0.0, 9.81));
    car_odom.twist.twist.angular.x = car_point(13);
    car_odom.twist.twist.angular.y = car_point(14);
    car_odom.twist.twist.angular.z = car_point(15);
    publishRelativeFrameTf(car_odom);
    last_ugv_horizon_compare_ =
        buildUgvHorizonCompareSnapshot(sim_time, car_window, beta_stage_profile);
    control_system_ptr_->setCarNonInertialStageProfiles(omega_stage_profile,
                                                        beta_stage_profile,
                                                        a_car_stage_profile);
    control_system_ptr_->setCarBetaNon(beta_stage_profile.col(0));
    control_system_ptr_->setCarOdom(car_odom);

    // printf("[Controller Iteration]\t t: %.3f \tx: %.3f \ty: %.3f \tz: %.3f\n", 
    //     sim_time, car_point(0), car_point(1), car_point(2)); 

   
    nav_msgs::Odometry quad_odom =
        cycle_quad_odom_snapshot_valid_
            ? cycle_quad_odom_snapshot_
            : buildQuadOdomFromState(getStateSnapshot(), i, ros::Time::now());
    publishUavTf(quad_odom, i);
    control_system_ptr_->setQuadOdom(quad_odom,i);

  }

  virtual void finishControllerIteration(double sim_time,int i) override
  {  
    last_solver_obstacle_drift_ = SolverObstacleDriftSnapshot();
    if (collision_detected_.load()) {
      return;
    }
    control_system_ptr_->setDebugNonInertialProfileSimTime(sim_time);
    auto x_ref = buildMpcReferenceWindow(sim_time);
    
    control_system_ptr_->setReferenceWindow(x_ref,&i);

    // Enforce a unified world-frame velocity envelope across frame modes.
    // This keeps the actuator command constraint in the same physical units.
    const double max_v_xy_world =
        std::max(0.0, control_system_ptr_->getMaxVelocityXY());
    const double max_v_z_world =
        std::max(0.0, control_system_ptr_->getMaxVelocityZ());
    const double max_yaw_rate_world =
        std::max(0.0, control_system_ptr_->getMaxYawRate());

    Control_t control;
    Eigen::Vector3d v_world_cmd = Eigen::Vector3d::Zero();
    double yaw_rate_world_cmd = 0.0;
    last_external_avoidance_ = ExternalAvoidanceSnapshot();
    if (external_avoidance_cfg_.enabled()) {
      control_system_ptr_->runReactiveBaselineStep();
      const auto filter_start = std::chrono::high_resolution_clock::now();
      PolytopicVoHrvoInput filter_input;
      Eigen::Vector3d estimated_position_world = Eigen::Vector3d::Zero();
      Eigen::Vector3d estimated_velocity_world = Eigen::Vector3d::Zero();
      control_system_ptr_->getEstimatedWorldKinematics(
          estimated_position_world, estimated_velocity_world);
      filter_input.position = estimated_position_world.head<2>();
      filter_input.current_velocity = estimated_velocity_world.head<2>();
      Eigen::Vector2d preferred_velocity = Eigen::Vector2d::Zero();
      Eigen::Vector2d accepted_velocity_compensation = Eigen::Vector2d::Zero();
      if (!x_ref.points.empty()) {
        Eigen::Quaterniond W_q_non = control_system_ptr_->getCarOrientation();
        if (W_q_non.norm() > 1.0e-9) {
          W_q_non.normalize();
        } else {
          W_q_non = Eigen::Quaterniond::Identity();
        }
        const Eigen::Vector3d slot_rel = x_ref.points.front().position;
        const Eigen::Vector3d slot_rel_velocity = x_ref.points.front().velocity;
        const Eigen::Vector3d slot_offset_world = W_q_non * slot_rel;
        const Eigen::Vector3d slot_goal_world =
            control_system_ptr_->getCarPosition() + slot_offset_world;
        Eigen::Vector3d slot_velocity_world = Eigen::Vector3d::Zero();
        Eigen::Vector3d slot_velocity_compensation_world =
            Eigen::Vector3d::Zero();
        if (external_avoidance_cfg_.preferred_velocity_mode ==
            "slot_velocity_ff") {
          Eigen::Vector3d car_acceleration_world = Eigen::Vector3d::Zero();
          Eigen::Vector3d car_angular_velocity_world =
              control_system_ptr_->getCarAngularVelocity();
          const auto& car_window = control_system_ptr_->getCarTrajectoryWindow();
          if (!car_window.empty()) {
            const CarState_t& car_state_0 = car_window.front();
            car_acceleration_world = Eigen::Vector3d(
                car_state_0(10), car_state_0(11), car_state_0(12));
            car_angular_velocity_world = Eigen::Vector3d(
                car_state_0(13), car_state_0(14), car_state_0(15));
          }
          const Eigen::Vector3d omega_non =
              W_q_non.inverse() * car_angular_velocity_world;
          slot_velocity_world =
              control_system_ptr_->getCarLinearVelocity() +
              W_q_non * (slot_rel_velocity + omega_non.cross(slot_rel));
          if (external_avoidance_cfg_.velocity_compensation_tau_sec > 0.0) {
            const Eigen::Vector3d slot_acceleration_world =
                car_acceleration_world +
                W_q_non * (2.0 * omega_non.cross(slot_rel_velocity) +
                           omega_non.cross(omega_non.cross(slot_rel)));
            slot_velocity_compensation_world =
                external_avoidance_cfg_.velocity_compensation_tau_sec *
                slot_acceleration_world;
          }
        }
        preferred_velocity =
            slot_velocity_world.head<2>() +
            external_avoidance_cfg_.goal_gain *
                (slot_goal_world.head<2>() - filter_input.position);
        accepted_velocity_compensation =
            slot_velocity_compensation_world.head<2>();
      }
      filter_input.preferred_velocity =
          clampPlanarVelocity(preferred_velocity, max_v_xy_world);
      filter_input.max_speed = max_v_xy_world;
      filter_input.velocity_window = external_avoidance_cfg_.velocity_window;
      filter_input.candidate_resolution =
          external_avoidance_cfg_.candidate_resolution;
      filter_input.fallback_penalty_weight =
          external_avoidance_cfg_.fallback_penalty_weight;

      // The formal Comment 3.4 baseline fixes safety_margin to zero and uses
      // the physical UAV/obstacle footprints.  The split is retained only so
      // legacy non-formal runs with an explicitly requested inflation remain
      // geometrically symmetric.
      const double self_polygon_inradius =
          uav_radius_ + 0.5 * external_avoidance_cfg_.safety_margin;
      const double polygon_half_angle =
          std::acos(-1.0) /
          static_cast<double>(external_avoidance_cfg_.polygon_sides);
      filter_input.self_enclosing_radius =
          self_polygon_inradius / std::cos(polygon_half_angle);
      filter_input.self_vertices = makeCircumscribedRegularPolygon(
          filter_input.position,
          self_polygon_inradius,
          external_avoidance_cfg_.polygon_sides);

      // Paper Equation (4): l=(v_R,max+v_O,max)*tau. Static obstacles have
      // v_O,max=0; peer UAVs share the same actuator speed bound.
      const double peer_neighbor_distance = polytopicVoNeighborDistance(
          max_v_xy_world,
          max_v_xy_world,
          external_avoidance_cfg_.time_horizon);
      const double static_neighbor_distance = polytopicVoNeighborDistance(
          max_v_xy_world,
          0.0,
          external_avoidance_cfg_.time_horizon);

      std::vector<Eigen::Vector3d> peer_positions;
      std::vector<Eigen::Vector3d> peer_velocities;
      std::vector<bool> peer_valid;
      control_system_ptr_->getSharedPositions(peer_positions,
                                              peer_velocities,
                                              peer_valid);
      struct PeerCandidate {
        double distance_sq = 0.0;
        std::size_t index = 0u;
      };
      std::vector<PeerCandidate> peer_candidates;
      for (std::size_t idx = 0; idx < peer_positions.size(); ++idx) {
        if (static_cast<int>(idx) == vehicle_index_ ||
            idx >= peer_valid.size() || !peer_valid[idx] ||
            idx >= peer_velocities.size()) {
          continue;
        }
        const double distance_sq =
            (peer_positions[idx].head<2>() - filter_input.position).squaredNorm();
        if (distance_sq <= peer_neighbor_distance * peer_neighbor_distance) {
          peer_candidates.push_back(PeerCandidate{distance_sq, idx});
        }
      }
      std::sort(peer_candidates.begin(), peer_candidates.end(),
                [](const PeerCandidate& lhs, const PeerCandidate& rhs) {
                  if (lhs.distance_sq != rhs.distance_sq) {
                    return lhs.distance_sq < rhs.distance_sq;
                  }
                  return lhs.index < rhs.index;
                });
      if (external_avoidance_cfg_.max_neighbors > 0u &&
          peer_candidates.size() > external_avoidance_cfg_.max_neighbors) {
        peer_candidates.resize(external_avoidance_cfg_.max_neighbors);
      }
      for (const auto& candidate : peer_candidates) {
        const std::size_t idx = candidate.index;
        PolytopicBody2d peer;
        peer.position = peer_positions[idx].head<2>();
        peer.velocity = peer_velocities[idx].head<2>();
        peer.enclosing_radius = filter_input.self_enclosing_radius;
        peer.vertices = makeCircumscribedRegularPolygon(
            peer.position,
            self_polygon_inradius,
            external_avoidance_cfg_.polygon_sides);
        filter_input.peers.push_back(peer);
      }

      std::vector<StaticObstacle> obstacles_copy;
      {
        std::lock_guard<std::mutex> lock(obstacles_mtx_);
        obstacles_copy = obstacles_;
      }
      for (const auto& obstacle : obstacles_copy) {
        if ((obstacle.position.head<2>() - filter_input.position).norm() >
            static_neighbor_distance) {
          continue;
        }
        PolytopicBody2d polygon_obstacle;
        polygon_obstacle.position = obstacle.position.head<2>();
        polygon_obstacle.velocity = Eigen::Vector2d::Zero();
        const double obstacle_polygon_inradius =
            obstacle.radius + 0.5 * external_avoidance_cfg_.safety_margin;
        polygon_obstacle.enclosing_radius =
            obstacle_polygon_inradius / std::cos(polygon_half_angle);
        polygon_obstacle.vertices = makeCircumscribedRegularPolygon(
            polygon_obstacle.position,
            obstacle_polygon_inradius,
            external_avoidance_cfg_.polygon_sides);
        filter_input.static_obstacles.push_back(polygon_obstacle);
      }

      const PolytopicVoHrvoResult filter_result =
          external_avoidance_filter_.filter(filter_input);
      Eigen::Vector2d filtered_velocity = filter_result.velocity;
      Eigen::Vector2d applied_velocity_compensation = Eigen::Vector2d::Zero();
      bool emergency_stop = false;
      if (!filter_result.hasCommand() || !filtered_velocity.allFinite()) {
        // Equation (8) already provides the published crowded-environment
        // fallback. Zero is reserved only for invalid filter inputs/results.
        filtered_velocity.setZero();
        emergency_stop = true;
      }
      if (!emergency_stop &&
          filter_result.status ==
              PolytopicVoHrvoStatus::kPreferredVelocitySafe) {
        // Keep the VO/HRVO cone test on the nominal velocity.  The extra term is
        // only an inverse response compensation for the simulated velocity lag.
        applied_velocity_compensation = accepted_velocity_compensation;
        filtered_velocity += applied_velocity_compensation;
      }
      const double filtered_norm = filtered_velocity.norm();
      if (max_v_xy_world > 1e-9 && filtered_norm > max_v_xy_world) {
        filtered_velocity *= max_v_xy_world / filtered_norm;
      }
      v_world_cmd.x() = filtered_velocity.x();
      v_world_cmd.y() = filtered_velocity.y();
      v_world_cmd.z() = 0.0;
      yaw_rate_world_cmd = 0.0;

      const auto filter_end = std::chrono::high_resolution_clock::now();
      last_external_avoidance_.valid = true;
      last_external_avoidance_.sim_time = sim_time;
      last_external_avoidance_.self_state_source = "mg_polyhrvo_noisy_estimate";
      last_external_avoidance_.self_position_world = filter_input.position;
      switch (filter_result.status) {
        case PolytopicVoHrvoStatus::kPreferredVelocitySafe:
          last_external_avoidance_.status = "preferred_safe";
          break;
        case PolytopicVoHrvoStatus::kAdjustedVelocity:
          last_external_avoidance_.status = "adjusted_safe";
          break;
        case PolytopicVoHrvoStatus::kPenalizedFallback:
          last_external_avoidance_.status = "penalized_fallback";
          break;
        case PolytopicVoHrvoStatus::kInvalidInput:
        default:
          last_external_avoidance_.status = "invalid_input";
          break;
      }
      const Eigen::Vector2d logged_preferred_velocity =
          clampPlanarVelocity(filter_input.preferred_velocity +
                                  applied_velocity_compensation,
                              max_v_xy_world);
      last_external_avoidance_.preferred_velocity_world =
          logged_preferred_velocity;
      last_external_avoidance_.filtered_velocity_world = filtered_velocity;
      last_external_avoidance_.self_velocity_world =
          filter_input.current_velocity;
      last_external_avoidance_.intervention_norm =
          (filtered_velocity - logged_preferred_velocity).norm();
      last_external_avoidance_.intervened =
          last_external_avoidance_.intervention_norm > 1e-9;
      last_external_avoidance_.feasible =
          filter_result.selectedVelocityIsSafe();
      last_external_avoidance_.used_fallback = filter_result.usedFallback();
      last_external_avoidance_.emergency_stop = emergency_stop;
      last_external_avoidance_.static_obstacle_count =
          filter_input.static_obstacles.size();
      last_external_avoidance_.peer_count = filter_input.peers.size();
      last_external_avoidance_.velocity_obstacle_count =
          filter_result.velocity_obstacle_count;
      last_external_avoidance_.official_overlap_gate_count =
          filter_result.official_overlap_gate_count;
      last_external_avoidance_.official_overlap_collision_count =
          filter_result.official_overlap_collision_count;
      last_external_avoidance_.official_overlap_cone_changed_count =
          filter_result.official_overlap_cone_changed_count;
      last_external_avoidance_.official_overlap_output_changed_count =
          filter_result.official_overlap_output_changed_count;
      last_external_avoidance_.candidate_count = filter_result.candidate_count;
      last_external_avoidance_.safe_candidate_count =
          filter_result.safe_candidate_count;
      last_external_avoidance_.fallback_min_ttc =
          filter_result.fallback_min_ttc;
      last_external_avoidance_.official_overlap_output_delta_norm =
          filter_result.official_overlap_output_delta_norm;
      last_external_avoidance_.time_horizon =
          external_avoidance_cfg_.time_horizon;
      last_external_avoidance_.max_neighbors =
          external_avoidance_cfg_.max_neighbors;
      last_external_avoidance_.safety_margin =
          external_avoidance_cfg_.safety_margin;
      last_external_avoidance_.polygon_sides =
          external_avoidance_cfg_.polygon_sides;
      last_external_avoidance_.velocity_window =
          external_avoidance_cfg_.velocity_window;
      last_external_avoidance_.candidate_resolution =
          external_avoidance_cfg_.candidate_resolution;
      last_external_avoidance_.fallback_penalty_weight =
          external_avoidance_cfg_.fallback_penalty_weight;
      last_external_avoidance_.static_neighbor_distance =
          static_neighbor_distance;
      last_external_avoidance_.peer_neighbor_distance =
          peer_neighbor_distance;
      last_external_avoidance_.filter_time_ms =
          std::chrono::duration<double, std::milli>(filter_end - filter_start)
              .count();
    } else {
      auto command = control_system_ptr_->run();
      last_solver_obstacle_drift_ = buildSolverObstacleDriftSnapshot(sim_time);
      const bool solve_ok = control_system_ptr_->getLastSolveOk();
      v_world_cmd = command.velocity_cmd;
      const Eigen::Quaterniond W_q_non =
          control_system_ptr_->getCarOrientation();
      const Eigen::Vector3d v_non =
          control_system_ptr_->getCarLinearVelocity();
      const Eigen::Vector3d omega_non =
          control_system_ptr_->getCarAngularVelocity();
      const Eigen::Vector3d car_pos = control_system_ptr_->getCarPosition();
      const State_t x_snapshot = getStateSnapshot();
      Eigen::Vector3d quad_pos_world(x_snapshot(0),
                                     x_snapshot(1),
                                     x_snapshot(2));
      Eigen::Vector3d p_rel = W_q_non.inverse() * (quad_pos_world - car_pos);
      if (!solve_ok && control_system_ptr_->isInertialFrame()) {
        // Keep consistent fallback behavior across modes on solver failure:
        // maintain current relative position to the moving non-inertial frame.
        v_world_cmd = v_non + W_q_non * omega_non.cross(p_rel);
      } else if (!control_system_ptr_->isInertialFrame()) {
        // Convert velocity command from non-inertial frame to world frame.
        Eigen::Vector3d v_rel_cmd(command.velocity_cmd.x(),
                                  command.velocity_cmd.y(),
                                  command.velocity_cmd.z());
        // v_world = v_non + R * (v_rel_cmd + omega_non x p_rel)
        v_world_cmd = v_non + W_q_non * (v_rel_cmd + omega_non.cross(p_rel));
      }
      Eigen::Vector2d v_world_xy(v_world_cmd.x(), v_world_cmd.y());
      const double v_world_xy_norm = v_world_xy.norm();
      if (max_v_xy_world > 1e-9 && v_world_xy_norm > max_v_xy_world) {
        v_world_xy *= (max_v_xy_world / v_world_xy_norm);
        v_world_cmd.x() = v_world_xy.x();
        v_world_cmd.y() = v_world_xy.y();
      }
      v_world_cmd.z() =
          std::max(-max_v_z_world, std::min(max_v_z_world, v_world_cmd.z()));
      yaw_rate_world_cmd = command.yaw_rate;
      if (!control_system_ptr_->isInertialFrame()) {
        yaw_rate_world_cmd += omega_non.z();
      }
      if (max_yaw_rate_world > 1e-9) {
        yaw_rate_world_cmd = std::max(
            -max_yaw_rate_world,
            std::min(max_yaw_rate_world, yaw_rate_world_cmd));
      }
    }
    control << v_world_cmd.x(), 
               v_world_cmd.y(), 
               v_world_cmd.z(), 
               yaw_rate_world_cmd;

    last_applied_control_world_ = control;
    system_ptr_->setControlAction(control);

  }

  
 private: 
  std::shared_ptr<coni_mpc::NumSimMpc> control_system_ptr_;

  // the trajectory in the non-inertial frame
  std::vector<RelativeTrajectoryPoint> trajectory_;
  // the plan strategy of the relative trajectory
  std::function<RelativeTrajectory(size_t, double, double, size_t, const std::vector<RelativeTrajectoryPoint>&)> plan_strategy_;
  // car trajectory
  std::vector<CarState_t> car_trajectory_;
  // the plan strategy for car 
  std::function<std::vector<CarState_t>(size_t, double, double, size_t, const std::vector<CarState_t>&)> car_plan_strategy_;
  coni_mpc::NonInertialPredictor non_inertial_predictor_;
  double predictor_last_sim_time_;
  bool current_beta_non_initialized_;
  double current_beta_non_prev_sim_time_;
  Eigen::Vector3d current_beta_non_prev_omega_non_;
  Eigen::Vector3d current_beta_non_last_value_;
  bool freeze_ugv_rollout_;
  ExternalAvoidanceConfig external_avoidance_cfg_;
  num_sim::PolytopicVoHrvoFilter external_avoidance_filter_;
  coni_mpc::PredictorParams damped_rollout_params_;
  bool damped_rollout_acc_initialized_;
  Eigen::Vector2d damped_rollout_prev_acc_non_xy_;
  double predictor_speed_max_;
  double predictor_omega_z_max_;

  double trajectory_duration_;
  size_t trajectory_sample_num_;

  double car_trajectory_duration_;
  size_t car_trajectory_sample_num_;

  void checkCollision()
  {
    if (collision_detected_.load()) {
      return;
    }
    const State_t x_snapshot = getStateSnapshot();
    Eigen::Vector3d quad_position(x_snapshot(0), x_snapshot(1), x_snapshot(2));

    // --- UAV-UAV collision check ---
    {
      std::vector<Eigen::Vector3d> peer_positions;
      std::vector<Eigen::Vector3d> peer_velocities;
      std::vector<bool> peer_valid;
      control_system_ptr_->getSharedPositions(peer_positions, peer_velocities, peer_valid);
      for (size_t idx = 0; idx < peer_positions.size(); ++idx) {
        if (static_cast<int>(idx) == vehicle_index_) continue;
        if (idx >= peer_valid.size() || !peer_valid[idx]) continue;
        const double dist = (quad_position.head<2>() - peer_positions[idx].head<2>()).norm();
        // Tangency counts as contact: the UAV footprint edge touching the
        // peer footprint is already a collision for the terminal detector.
        if (dist <= 2.0 * uav_radius_) {
          if (!collision_detected_.exchange(true)) {
            stop();
            if (collision_callback_) {
              StaticObstacle dummy;
              dummy.position = peer_positions[idx];
              dummy.radius   = uav_radius_;
              collision_callback_(vehicle_index_, quad_position, dummy, dist);
            }
          }
          return;
        }
      }
    }

    // --- UAV-static obstacle collision check ---
    std::vector<StaticObstacle> obstacles_copy;
    {
      std::lock_guard<std::mutex> lock(obstacles_mtx_);
      obstacles_copy = obstacles_;
    }
    for (const auto& obstacle : obstacles_copy) {
      const double distance = (quad_position.head<2>() - obstacle.position.head<2>()).norm();
      const double collision_threshold =
          uav_radius_ + obstacle.radius + failure_distance_margin_;
      // Tangency counts as contact: the UAV radius edge touching the obstacle
      // surface is considered a collision, not merely a near miss.
      if (distance <= collision_threshold) {
        if (!collision_detected_.exchange(true)) {
          stop();
          if (collision_callback_) {
            collision_callback_(vehicle_index_, quad_position, obstacle, distance);
          }
        }
        break;
      }
    }
  }

  double uav_radius_;
  double safety_margin_;
  double failure_distance_margin_;
  mutable std::mutex obstacles_mtx_;
  std::vector<StaticObstacle> obstacles_;
  std::function<void(double)> dynamic_obstacle_callback_;
  std::function<void(int, const Eigen::Vector3d&, const StaticObstacle&, double)> collision_callback_;
  int vehicle_index_;
  bool owns_dynamic_obstacle_updates_;
  std::atomic<bool> collision_detected_;
  UgvHorizonCompareSnapshot last_ugv_horizon_compare_;
  SolverObstacleDriftSnapshot last_solver_obstacle_drift_;
  ExternalAvoidanceSnapshot last_external_avoidance_;
  Control_t last_applied_control_world_ = Control_t::Zero();
  nav_msgs::Odometry cycle_quad_odom_snapshot_;
  bool cycle_quad_odom_snapshot_valid_ = false;
};

static void writeUgvHorizonDetailRows(
    std::ostream& out,
    const std::string& run_tag,
    std::size_t step_idx,
    const std::string& frame_mode_effective,
    const std::string& ugv_rollout_mode,
    const QuadrotorSimulator::UgvHorizonCompareSnapshot& snapshot) {
  const std::size_t sample_count =
      std::min(snapshot.predicted_window.size(), snapshot.actual_window.size());
  for (std::size_t k = 0; k < sample_count; ++k) {
    const auto& pred = snapshot.predicted_window[k];
    const auto& actual = snapshot.actual_window[k];
    const Eigen::Vector3d pred_pos(pred(0), pred(1), pred(2));
    const Eigen::Vector3d actual_pos(actual(0), actual(1), actual(2));
    const Eigen::Vector3d pred_vel(pred(3), pred(4), pred(5));
    const Eigen::Vector3d actual_vel(actual(3), actual(4), actual(5));
    Eigen::Quaterniond pred_q_non(pred(6), pred(7), pred(8), pred(9));
    if (pred_q_non.norm() > 1e-9) {
      pred_q_non.normalize();
    } else {
      pred_q_non = Eigen::Quaterniond::Identity();
    }
    Eigen::Quaterniond actual_q_non(actual(6), actual(7), actual(8), actual(9));
    if (actual_q_non.norm() > 1e-9) {
      actual_q_non.normalize();
    } else {
      actual_q_non = Eigen::Quaterniond::Identity();
    }
    const Eigen::Vector3d pred_omega_world(pred(13), pred(14), pred(15));
    const Eigen::Vector3d actual_omega_world(actual(13), actual(14), actual(15));
    const Eigen::Vector3d pred_omega_non =
        pred_q_non.inverse() * pred_omega_world;
    const Eigen::Vector3d actual_omega_non =
        actual_q_non.inverse() * actual_omega_world;
    const double pos_error_xy =
        (pred_pos.head<2>() - actual_pos.head<2>()).norm();
    const double pos_error_xyz = (pred_pos - actual_pos).norm();
    out << run_tag << ","
        << step_idx << ","
        << fmtDouble(snapshot.sim_time) << ","
        << frame_mode_effective << ","
        << ugv_rollout_mode << ","
        << k << ","
        << fmtDouble(pred_pos.x()) << ","
        << fmtDouble(pred_pos.y()) << ","
        << fmtDouble(pred_pos.z()) << ","
        << fmtDouble(actual_pos.x()) << ","
        << fmtDouble(actual_pos.y()) << ","
        << fmtDouble(actual_pos.z()) << ","
        << fmtDouble(pred_vel.x()) << ","
        << fmtDouble(pred_vel.y()) << ","
        << fmtDouble(pred_vel.z()) << ","
        << fmtDouble(actual_vel.x()) << ","
        << fmtDouble(actual_vel.y()) << ","
        << fmtDouble(actual_vel.z()) << ","
        << fmtDouble(pred_omega_non.x()) << ","
        << fmtDouble(pred_omega_non.y()) << ","
        << fmtDouble(pred_omega_non.z()) << ","
        << fmtDouble(actual_omega_non.x()) << ","
        << fmtDouble(actual_omega_non.y()) << ","
        << fmtDouble(actual_omega_non.z()) << ","
        << fmtDouble(pos_error_xy) << ","
        << fmtDouble(pos_error_xyz) << "\n";
  }
}

static void writeUgvHorizonSummaryRow(
    std::ostream& out,
    const std::string& run_tag,
    std::size_t step_idx,
    const std::string& frame_mode_effective,
    const std::string& ugv_rollout_mode,
    const QuadrotorSimulator::UgvHorizonCompareSnapshot& snapshot) {
  const std::size_t sample_count =
      std::min(snapshot.predicted_window.size(), snapshot.actual_window.size());
  out << run_tag << ","
      << step_idx << ","
      << fmtDouble(snapshot.sim_time) << ","
      << frame_mode_effective << ","
      << ugv_rollout_mode << ","
      << fmtDouble(snapshot.horizon_rms_xy) << ","
      << fmtDouble(snapshot.horizon_max_xy) << ","
      << fmtDouble(snapshot.horizon_final_xy) << ","
      << sample_count << ","
      << fmtDouble(snapshot.boundary_drift_rms) << ","
      << fmtDouble(snapshot.boundary_drift_max) << ","
      << snapshot.boundary_drift_samples << "\n";
}

static void writeSolverObstacleDriftHeader(std::ostream& out) {
  out << "run_tag,uav_idx,step_idx,sim_time,frame_mode_effective,ugv_rollout_mode,"
         "obstacle_slot,obstacle_key,horizon_idx,obstacle_radius,"
         "solver_position_x,solver_position_y,solver_position_z,"
         "real_position_x,real_position_y,real_position_z,"
         "diff_x,diff_y,diff_z,diff_norm_xy,diff_norm_xyz\n";
}

static void writeExternalAvoidanceHeader(std::ostream& out) {
  out << "run_tag,uav_idx,step_idx,sim_time,external_avoidance_mode,status,"
         "self_state_source,self_px_world,self_py_world,"
         "preferred_vx_world,preferred_vy_world,filtered_vx_world,filtered_vy_world,"
         "self_vx_world,self_vy_world,intervened,selected_velocity_safe,"
         "used_fallback,emergency_stop,"
         "intervention_norm,static_obstacle_count,peer_count,velocity_obstacle_count,"
         "official_overlap_gate_count,official_overlap_collision_count,"
         "official_overlap_cone_changed_count,official_overlap_output_changed_count,"
         "candidate_count,safe_candidate_count,fallback_min_ttc,time_horizon,"
         "max_neighbors,safety_margin,polygon_sides,velocity_window,"
         "candidate_resolution,fallback_penalty_weight,static_neighbor_distance,"
         "peer_neighbor_distance,official_overlap_output_delta_norm,filter_time_ms\n";
}

static void writeExternalAvoidanceRow(
    std::ostream& out,
    const std::string& run_tag,
    int uav_idx,
    std::size_t step_idx,
    const std::string& mode,
    const QuadrotorSimulator::ExternalAvoidanceSnapshot& snapshot) {
  out << run_tag << ","
      << uav_idx << ","
      << step_idx << ","
      << fmtDouble(snapshot.sim_time) << ","
      << mode << ","
      << snapshot.status << ","
      << snapshot.self_state_source << ","
      << fmtDouble(snapshot.self_position_world.x()) << ","
      << fmtDouble(snapshot.self_position_world.y()) << ","
      << fmtDouble(snapshot.preferred_velocity_world.x()) << ","
      << fmtDouble(snapshot.preferred_velocity_world.y()) << ","
      << fmtDouble(snapshot.filtered_velocity_world.x()) << ","
      << fmtDouble(snapshot.filtered_velocity_world.y()) << ","
      << fmtDouble(snapshot.self_velocity_world.x()) << ","
      << fmtDouble(snapshot.self_velocity_world.y()) << ","
      << (snapshot.intervened ? 1 : 0) << ","
      << (snapshot.feasible ? 1 : 0) << ","
      << (snapshot.used_fallback ? 1 : 0) << ","
      << (snapshot.emergency_stop ? 1 : 0) << ","
      << fmtDouble(snapshot.intervention_norm) << ","
      << snapshot.static_obstacle_count << ","
      << snapshot.peer_count << ","
      << snapshot.velocity_obstacle_count << ","
      << snapshot.official_overlap_gate_count << ","
      << snapshot.official_overlap_collision_count << ","
      << snapshot.official_overlap_cone_changed_count << ","
      << snapshot.official_overlap_output_changed_count << ","
      << snapshot.candidate_count << ","
      << snapshot.safe_candidate_count << ","
      << fmtDouble(snapshot.fallback_min_ttc) << ","
      << fmtDouble(snapshot.time_horizon) << ","
      << snapshot.max_neighbors << ","
      << fmtDouble(snapshot.safety_margin) << ","
      << snapshot.polygon_sides << ","
      << fmtDouble(snapshot.velocity_window) << ","
      << fmtDouble(snapshot.candidate_resolution) << ","
      << fmtDouble(snapshot.fallback_penalty_weight) << ","
      << fmtDouble(snapshot.static_neighbor_distance) << ","
      << fmtDouble(snapshot.peer_neighbor_distance) << ","
      << fmtDouble(snapshot.official_overlap_output_delta_norm) << ","
      << fmtDouble(snapshot.filter_time_ms) << "\n";
}

static void writeSolverObstacleDriftRows(
    std::ostream& out,
    const std::string& run_tag,
    std::size_t step_idx,
    const std::string& frame_mode_effective,
    const std::string& ugv_rollout_mode,
    const QuadrotorSimulator::SolverObstacleDriftSnapshot& snapshot) {
  for (const auto& record : snapshot.records) {
    out << run_tag << ","
        << record.uav_idx << ","
        << step_idx << ","
        << fmtDouble(snapshot.sim_time) << ","
        << frame_mode_effective << ","
        << ugv_rollout_mode << ","
        << record.obstacle_slot << ","
        << record.obstacle_key << ","
        << record.horizon_idx << ","
        << fmtDouble(record.radius) << ","
        << fmtDouble(record.solver_position.x()) << ","
        << fmtDouble(record.solver_position.y()) << ","
        << fmtDouble(record.solver_position.z()) << ","
        << fmtDouble(record.real_position.x()) << ","
        << fmtDouble(record.real_position.y()) << ","
        << fmtDouble(record.real_position.z()) << ","
        << fmtDouble(record.diff.x()) << ","
        << fmtDouble(record.diff.y()) << ","
        << fmtDouble(record.diff.z()) << ","
        << fmtDouble(record.diff_norm_xy) << ","
        << fmtDouble(record.diff_norm_xyz) << "\n";
  }
}

static std::string computeTrajectoryId(
    const std::vector<QuadrotorSimulator::CarState_t>& car_trajectory) {
  std::ostringstream serialized;
  for (const auto& point : car_trajectory) {
    serialized << fmtDouble(point(0)) << ","
               << fmtDouble(point(1)) << ","
               << fmtDouble(point(2)) << "\n";
  }
  return sha1Hex(serialized.str());
}

static double computeMaxAbsYawRate(
    const std::vector<QuadrotorSimulator::CarState_t>& car_trajectory) {
  double max_abs_yaw_rate = 0.0;
  for (const auto& point : car_trajectory) {
    max_abs_yaw_rate = std::max(max_abs_yaw_rate, std::abs(point(15)));
  }
  return max_abs_yaw_rate;
}

static double computeMaxPlanarSpeed(
    const std::vector<QuadrotorSimulator::CarState_t>& car_trajectory) {
  double max_planar_speed = 0.0;
  for (const auto& point : car_trajectory) {
    max_planar_speed =
        std::max(max_planar_speed, std::hypot(point(3), point(4)));
  }
  return max_planar_speed;
}

static QuadrotorSimulator::CarState_t sampleCarStateAtTime(
    const std::vector<QuadrotorSimulator::CarState_t>& trajectory,
    double trajectory_duration,
    double query_time) {
  QuadrotorSimulator::CarState_t result =
      QuadrotorSimulator::CarState_t::Zero();
  if (trajectory.empty()) {
    return result;
  }
  if (trajectory.size() == 1 || trajectory_duration <= 1e-6) {
    return trajectory.front();
  }

  const double clamped_time =
      std::max(0.0, std::min(query_time, trajectory_duration));
  const double scaled_idx =
      (clamped_time / trajectory_duration) *
      static_cast<double>(trajectory.size() - 1);
  std::size_t idx0 = static_cast<std::size_t>(std::floor(scaled_idx));
  idx0 = std::min(idx0, trajectory.size() - 1);
  const std::size_t idx1 = std::min(idx0 + 1, trajectory.size() - 1);
  const double blend =
      std::max(0.0, std::min(1.0, scaled_idx - static_cast<double>(idx0)));

  result = (1.0 - blend) * trajectory[idx0] + blend * trajectory[idx1];
  Eigen::Quaterniond q0(trajectory[idx0](6), trajectory[idx0](7),
                        trajectory[idx0](8), trajectory[idx0](9));
  Eigen::Quaterniond q1(trajectory[idx1](6), trajectory[idx1](7),
                        trajectory[idx1](8), trajectory[idx1](9));
  if (q0.norm() > 1e-9) {
    q0.normalize();
  } else {
    q0 = Eigen::Quaterniond::Identity();
  }
  if (q1.norm() > 1e-9) {
    q1.normalize();
  } else {
    q1 = Eigen::Quaterniond::Identity();
  }
  const Eigen::Quaterniond q_interp = q0.slerp(blend, q1).normalized();
  result(6) = q_interp.w();
  result(7) = q_interp.x();
  result(8) = q_interp.y();
  result(9) = q_interp.z();
  if (query_time <= trajectory_duration) {
    return result;
  }

  const auto& tail = trajectory.back();
  const double extra_time = query_time - trajectory_duration;
  const Eigen::Vector3d tail_position(tail(0), tail(1), tail(2));
  const Eigen::Vector3d tail_velocity(tail(3), tail(4), tail(5));
  const Eigen::Vector3d tail_acc(tail(10), tail(11), tail(12));
  const Eigen::Vector3d tail_omega(tail(13), tail(14), tail(15));
  Eigen::Quaterniond tail_q(tail(6), tail(7), tail(8), tail(9));
  if (tail_q.norm() > 1e-9) {
    tail_q.normalize();
  } else {
    tail_q = Eigen::Quaterniond::Identity();
  }

  // Keep the horizon frame continuous past the terminal knot instead of
  // collapsing it to an artificial zero-velocity/zero-yaw endpoint.
  const Eigen::Vector3d extrapolated_position =
      tail_position + tail_velocity * extra_time;
  const Eigen::AngleAxisd extra_yaw(tail_omega.z() * extra_time,
                                    Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond extrapolated_q =
      (tail_q * Eigen::Quaterniond(extra_yaw)).normalized();
  result(0) = extrapolated_position.x();
  result(1) = extrapolated_position.y();
  result(2) = extrapolated_position.z();
  result(3) = tail_velocity.x();
  result(4) = tail_velocity.y();
  result(5) = tail_velocity.z();
  result(6) = extrapolated_q.w();
  result(7) = extrapolated_q.x();
  result(8) = extrapolated_q.y();
  result(9) = extrapolated_q.z();
  result(10) = tail_acc.x();
  result(11) = tail_acc.y();
  result(12) = tail_acc.z();
  result(13) = tail_omega.x();
  result(14) = tail_omega.y();
  result(15) = tail_omega.z();
  return result;
}

std::vector<QuadrotorSimulator::CarState_t>
GenerateCarTrajectory(size_t sample_num,
                      double duration_hint,
                      double desired_speed,
                      double max_yaw_rate,
                      const RandomObstacleConfig& obstacle_cfg,
                      const UgvPathPlannerConfig& planner_cfg,
                      const std::vector<StaticObstacle>& obstacles,
                      double uav_radius,
                      double car_radius,
                      const std::vector<Eigen::Vector2d>& uav_start_offsets_xy,
                      double safety_margin,
                      double& trajectory_duration_out)
{
  auto clampValue = [](double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
  };
  auto moveTowards = [&](double current, double target, double max_delta) {
    if (target > current) {
      return std::min(current + max_delta, target);
    }
    return std::max(current - max_delta, target);
  };
  auto yawFromQuaternion = [](const Eigen::Quaterniond& q) {
    return std::atan2(
        2.0 * (q.w() * q.z() + q.x() * q.y()),
        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  };

  std::vector<QuadrotorSimulator::CarState_t> result;
  trajectory_duration_out = 0.0;
  if (sample_num < 2) {
    return result;
  }

  const double speed = std::max(0.15, std::abs(desired_speed));
  const double yaw_rate_limit = std::max(0.05, std::abs(max_yaw_rate));
  const double span_x = std::max(0.0, obstacle_cfg.x_max - obstacle_cfg.x_min);
  const double span_y = std::max(0.0, obstacle_cfg.y_max - obstacle_cfg.y_min);
  const double margin_abs = 0.5;  // Absolute margin in meters.
  const double margin_x = std::min(margin_abs, 0.5 * span_x);
  const double margin_y = std::min(margin_abs, 0.5 * span_y);
  const Eigen::Vector2d start_xy(obstacle_cfg.x_min + margin_x,
                                 obstacle_cfg.y_min + margin_y);
  const Eigen::Vector2d goal_xy(obstacle_cfg.x_max - margin_x,
                                obstacle_cfg.y_max - margin_y);
  (void)safety_margin;
  const double clearance_scale = 1.0;

  auto requiredClearance = [&](const StaticObstacle& obstacle) {
    // The generated path is the UGV center path.  Its keep-out must include
    // the UGV footprint as well as the UAV footprint used by the formation;
    // otherwise an obstacle can be avoided by the UAV center while the car
    // body still cuts through it.
    const double vehicle_radius = std::max(uav_radius, car_radius);
    return clearance_scale * (vehicle_radius + obstacle.radius);
  };

  auto pointIsFree = [&](const Eigen::Vector2d& point) {
    for (const auto& obstacle : obstacles) {
      const double keepout = requiredClearance(obstacle);
      const Eigen::Vector2d obs_xy(obstacle.position.x(), obstacle.position.y());
      const double dist_sq = (point - obs_xy).squaredNorm();
      if (dist_sq < keepout * keepout) {
        return false;
      }
    }
    return true;
  };

  auto segmentDistanceSq = [](const Eigen::Vector2d& p,
                              const Eigen::Vector2d& a,
                              const Eigen::Vector2d& b) {
    const Eigen::Vector2d ab = b - a;
    const double ab_len_sq = ab.squaredNorm();
    if (ab_len_sq < 1e-10) {
      return (p - a).squaredNorm();
    }
    double t = ab.dot(p - a) / ab_len_sq;
    t = std::max(0.0, std::min(1.0, t));
    const Eigen::Vector2d proj = a + t * ab;
    return (p - proj).squaredNorm();
  };

  auto segmentIsFree = [&](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    for (const auto& obstacle : obstacles) {
      const double keepout = requiredClearance(obstacle);
      const Eigen::Vector2d obs_xy(obstacle.position.x(), obstacle.position.y());
      if (segmentDistanceSq(obs_xy, a, b) < keepout * keepout) {
        return false;
      }
    }
    return true;
  };

  const double x_mid = 0.5 * (obstacle_cfg.x_min + obstacle_cfg.x_max);
  const double y_mid = 0.5 * (obstacle_cfg.y_min + obstacle_cfg.y_max);
  const Eigen::Vector2d mid_xy(x_mid, y_mid);
  auto clampToPlanningBounds = [&](const Eigen::Vector2d& point) {
    return Eigen::Vector2d(
        clampValue(point.x(), obstacle_cfg.x_min + margin_x, obstacle_cfg.x_max - margin_x),
        clampValue(point.y(), obstacle_cfg.y_min + margin_y, obstacle_cfg.y_max - margin_y));
  };
  auto pointClearance = [&](const Eigen::Vector2d& point) {
    double min_clearance = std::numeric_limits<double>::infinity();
    for (const auto& obstacle : obstacles) {
      const Eigen::Vector2d obs_xy(obstacle.position.x(), obstacle.position.y());
      const double clearance =
          (point - obs_xy).norm() - requiredClearance(obstacle);
      min_clearance = std::min(min_clearance, clearance);
    }
    if (!std::isfinite(min_clearance)) {
      return std::max(span_x, span_y);
    }
    return min_clearance;
  };
  auto generateSeededSWaypoints = [&]() {
    std::vector<Eigen::Vector2d> generated;
    const Eigen::Vector2d chord = goal_xy - start_xy;
    const double chord_norm = chord.norm();
    if (chord_norm <= 1e-6) {
      return generated;
    }
    const Eigen::Vector2d tangent = chord / chord_norm;
    const Eigen::Vector2d normal(-tangent.y(), tangent.x());
    const double map_span = std::min(span_x, span_y);
    const double max_lateral = std::max(2.0, 0.26 * map_span);
    const double mid_lateral = std::max(1.5, 0.20 * map_span);
    const double min_lateral = std::max(1.0, 0.14 * map_span);
    const double seed_bias =
        static_cast<double>((std::abs(obstacle_cfg.seed) % 5) - 2) * 0.015;
    const std::vector<double> fractions_1 = {
        clampValue(0.30 + seed_bias, 0.24, 0.38),
        clampValue(0.34 - 0.5 * seed_bias, 0.26, 0.40),
    };
    const std::vector<double> fractions_2 = {
        clampValue(0.70 - seed_bias, 0.62, 0.76),
        clampValue(0.66 + 0.5 * seed_bias, 0.60, 0.74),
    };
    const std::vector<double> amplitudes = {
        max_lateral,
        0.5 * (max_lateral + mid_lateral),
        mid_lateral,
        0.5 * (mid_lateral + min_lateral),
        min_lateral,
    };
    const double preferred_sign = (obstacle_cfg.seed % 2 == 0) ? 1.0 : -1.0;
    const std::array<double, 2> sign_order = {preferred_sign, -preferred_sign};
    double best_score = -std::numeric_limits<double>::infinity();
    for (double sign : sign_order) {
      for (double frac_1 : fractions_1) {
        for (double frac_2 : fractions_2) {
          if (frac_2 - frac_1 < 0.18) {
            continue;
          }
          const Eigen::Vector2d base_1 = start_xy + frac_1 * chord;
          const Eigen::Vector2d base_2 = start_xy + frac_2 * chord;
          for (double amplitude : amplitudes) {
            const Eigen::Vector2d waypoint_1 =
                clampToPlanningBounds(base_1 + sign * amplitude * normal);
            const Eigen::Vector2d waypoint_2 =
                clampToPlanningBounds(base_2 - sign * amplitude * normal);
            const double clearance_1 = pointClearance(waypoint_1);
            const double clearance_2 = pointClearance(waypoint_2);
            const double score =
                3.0 * amplitude + std::min(clearance_1, clearance_2) +
                0.2 * (clearance_1 + clearance_2);
            if (score <= best_score) {
              continue;
            }
            best_score = score;
            generated = {waypoint_1, waypoint_2};
          }
        }
      }
    }
    return generated;
  };
  std::vector<Eigen::Vector2d> route_waypoints = planner_cfg.guide_waypoints;
  if (route_waypoints.empty() && planner_cfg.use_midpoint_guide) {
    route_waypoints.push_back(mid_xy);
  }
  bool auto_generated_s_guides = false;
  if (route_waypoints.empty()) {
    route_waypoints = generateSeededSWaypoints();
    auto_generated_s_guides = !route_waypoints.empty();
    if (auto_generated_s_guides) {
      std::ostringstream oss;
      oss << "Auto-generated seed-aware S-bend guide waypoints:";
      for (size_t idx = 0; idx < route_waypoints.size(); ++idx) {
        const Eigen::Vector2d& p = route_waypoints[idx];
        oss << " [" << idx << "]=(" << p.x() << ", " << p.y() << ")";
      }
      ROS_INFO_STREAM(oss.str());
    }
  }
  const bool last_guide_is_goal =
      !auto_generated_s_guides &&
      planner_cfg.use_last_guide_as_goal && !route_waypoints.empty();
  const Eigen::Vector2d final_goal_xy =
      last_guide_is_goal ? route_waypoints.back() : goal_xy;
  std::vector<Eigen::Vector2d> start_offsets;
  start_offsets.reserve(uav_start_offsets_xy.size() + 1);
  start_offsets.push_back(Eigen::Vector2d(0.0, 0.0));
  for (const auto& offset : uav_start_offsets_xy) {
    bool duplicate = false;
    for (const auto& existing : start_offsets) {
      if ((existing - offset).norm() <= 1e-6) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      start_offsets.push_back(offset);
    }
  }
  double start_formation_radius = 0.0;
  for (const auto& offset : start_offsets) {
    start_formation_radius = std::max(start_formation_radius, offset.norm());
  }

  auto startPointIsFree = [&](const Eigen::Vector2d& point) {
    for (const auto& obstacle : obstacles) {
      const double keepout = requiredClearance(obstacle) + start_formation_radius;
      const Eigen::Vector2d obs_xy(obstacle.position.x(), obstacle.position.y());
      if ((point - obs_xy).squaredNorm() < keepout * keepout) {
        return false;
      }
    }
    return true;
  };

  auto startSegmentIsFree = [&](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    for (const auto& obstacle : obstacles) {
      const double keepout = requiredClearance(obstacle) + start_formation_radius;
      const Eigen::Vector2d obs_xy(obstacle.position.x(), obstacle.position.y());
      if (segmentDistanceSq(obs_xy, a, b) < keepout * keepout) {
        return false;
      }
    }
    return true;
  };

  double min_x = std::min(start_xy.x(), final_goal_xy.x());
  double max_x = std::max(start_xy.x(), final_goal_xy.x());
  double min_y = std::min(start_xy.y(), final_goal_xy.y());
  double max_y = std::max(start_xy.y(), final_goal_xy.y());
  for (const auto& waypoint : route_waypoints) {
    min_x = std::min(min_x, waypoint.x());
    max_x = std::max(max_x, waypoint.x());
    min_y = std::min(min_y, waypoint.y());
    max_y = std::max(max_y, waypoint.y());
  }
  double max_keepout = clearance_scale * uav_radius;
  double min_keepout = max_keepout;
  for (const auto& obstacle : obstacles) {
    min_x = std::min(min_x, obstacle.position.x());
    max_x = std::max(max_x, obstacle.position.x());
    min_y = std::min(min_y, obstacle.position.y());
    max_y = std::max(max_y, obstacle.position.y());
    const double keepout = requiredClearance(obstacle);
    max_keepout = std::max(max_keepout, keepout);
    min_keepout = std::min(min_keepout, keepout);
  }
  const double bound_margin = std::max(2.0 * max_keepout, 1.0);
  min_x -= bound_margin;
  max_x += bound_margin;
  min_y -= bound_margin;
  max_y += bound_margin;

  double grid_res = std::max(0.2, std::min(0.5, 0.33 * std::max(min_keepout, 0.2)));
  auto computeGridSize = [&](double res) {
    const int nx = static_cast<int>(std::ceil((max_x - min_x) / res)) + 1;
    const int ny = static_cast<int>(std::ceil((max_y - min_y) / res)) + 1;
    return std::pair<int, int>(nx, ny);
  };

  auto grid_size = computeGridSize(grid_res);
  const size_t max_nodes = 200000;
  if (static_cast<size_t>(grid_size.first) * static_cast<size_t>(grid_size.second) > max_nodes) {
    const double scale =
        std::sqrt(static_cast<double>(grid_size.first) * static_cast<double>(grid_size.second) /
                  static_cast<double>(max_nodes));
    grid_res = grid_res * scale;
    grid_size = computeGridSize(grid_res);
  }

  const int grid_nx = grid_size.first;
  const int grid_ny = grid_size.second;
  if (grid_nx <= 1 || grid_ny <= 1) {
    trajectory_duration_out = duration_hint > 1e-3 ? duration_hint : 1.0;
    return result;
  }

  auto clampIndex = [](int value, int max_value) {
    if (value < 0) return 0;
    if (value > max_value) return max_value;
    return value;
  };

  auto toIndex = [&](const Eigen::Vector2d& point) {
    const int ix = clampIndex(static_cast<int>(std::round((point.x() - min_x) / grid_res)), grid_nx - 1);
    const int iy = clampIndex(static_cast<int>(std::round((point.y() - min_y) / grid_res)), grid_ny - 1);
    return std::pair<int, int>(ix, iy);
  };

  std::vector<double> grid_x(grid_nx, 0.0);
  std::vector<double> grid_y(grid_ny, 0.0);
  for (int ix = 0; ix < grid_nx; ++ix) {
    grid_x[ix] = min_x + grid_res * static_cast<double>(ix);
  }
  for (int iy = 0; iy < grid_ny; ++iy) {
    grid_y[iy] = min_y + grid_res * static_cast<double>(iy);
  }

  auto toCoord = [&](int ix, int iy) {
    return Eigen::Vector2d(grid_x[ix], grid_y[iy]);
  };

  const size_t grid_count = static_cast<size_t>(grid_nx) * static_cast<size_t>(grid_ny);
  std::vector<uint8_t> is_free(grid_count, 1);
  std::vector<uint8_t> is_free_start(grid_count, 1);
  auto flatIndex = [&](int ix, int iy) {
    return static_cast<size_t>(iy) * static_cast<size_t>(grid_nx) + static_cast<size_t>(ix);
  };

  for (int iy = 0; iy < grid_ny; ++iy) {
    for (int ix = 0; ix < grid_nx; ++ix) {
      const Eigen::Vector2d p = toCoord(ix, iy);
      const size_t idx = flatIndex(ix, iy);
      const bool free = pointIsFree(p);
      is_free[idx] = free ? 1 : 0;
      if (!free) {
        is_free_start[idx] = 0;
        continue;
      }
      is_free_start[idx] = startPointIsFree(p) ? 1 : 0;
    }
  }

  auto nearestFreeCell = [&](int sx, int sy, const std::vector<uint8_t>& free_grid) {
    const size_t start_idx = flatIndex(sx, sy);
    if (free_grid[start_idx]) {
      return std::pair<int, int>(sx, sy);
    }
    std::queue<std::pair<int, int>> q;
    std::vector<uint8_t> visited(grid_count, 0);
    q.push({sx, sy});
    visited[start_idx] = 1;
    static const int dirs4[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    static const int dirs8[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                                    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    const int (*dirs)[2] = (planner_cfg.a_star_connectivity == 8) ? dirs8 : dirs4;
    const int dir_count = (planner_cfg.a_star_connectivity == 8) ? 8 : 4;
    while (!q.empty()) {
      const int cx = q.front().first;
      const int cy = q.front().second;
      q.pop();
      for (int dir_idx = 0; dir_idx < dir_count; ++dir_idx) {
        const int* d = dirs[dir_idx];
        const int nx = cx + d[0];
        const int ny = cy + d[1];
        if (nx < 0 || nx >= grid_nx || ny < 0 || ny >= grid_ny) {
          continue;
        }
        const size_t nidx = flatIndex(nx, ny);
        if (visited[nidx]) {
          continue;
        }
        if (free_grid[nidx]) {
          return std::pair<int, int>(nx, ny);
        }
        visited[nidx] = 1;
        q.push({nx, ny});
      }
    }
    return std::pair<int, int>(-1, -1);
  };

  const auto start_cell = toIndex(start_xy);
  const auto goal_cell = toIndex(final_goal_xy);
  auto safe_start_cell = nearestFreeCell(start_cell.first, start_cell.second, is_free_start);
  auto safe_goal_cell = nearestFreeCell(goal_cell.first, goal_cell.second, is_free);
  if (safe_start_cell.first < 0 || safe_goal_cell.first < 0) {
    ROS_ERROR("No collision-free grid cells found for trajectory endpoints.");
    return result;
  }

  if (safe_start_cell != start_cell) {
    ROS_WARN_STREAM("Start point is inside keepout; snapping to nearest free cell at ("
                    << grid_x[safe_start_cell.first] << ", " << grid_y[safe_start_cell.second] << ").");
  }
  if (safe_goal_cell != goal_cell) {
    ROS_WARN_STREAM("Goal point is inside keepout; snapping to nearest free cell at ("
                    << grid_x[safe_goal_cell.first] << ", " << grid_y[safe_goal_cell.second] << ").");
  }

  struct QueueNode {
    int idx;
    double f;
  };
  struct QueueCompare {
    bool operator()(const QueueNode& a, const QueueNode& b) const {
      return a.f > b.f;
    }
  };
  static const int neighbors4[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  static const int neighbors8[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                                       {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  const int (*neighbors)[2] = (planner_cfg.a_star_connectivity == 8) ? neighbors8 : neighbors4;
  const int neighbor_count = (planner_cfg.a_star_connectivity == 8) ? 8 : 4;

  auto buildPath = [&](const std::pair<int, int>& start_cell,
                       const std::pair<int, int>& goal_cell,
                       std::vector<Eigen::Vector2d>& out_path) {
    const size_t start_flat = flatIndex(start_cell.first, start_cell.second);
    const size_t goal_flat = flatIndex(goal_cell.first, goal_cell.second);
    std::vector<double> g_cost(grid_count, std::numeric_limits<double>::infinity());
    std::vector<int> parent(grid_count, -1);
    std::vector<uint8_t> closed(grid_count, 0);
    auto heuristic = [&](int ix, int iy) {
      const Eigen::Vector2d p = toCoord(ix, iy);
      return (p - toCoord(goal_cell.first, goal_cell.second)).norm();
    };
    std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open;
    g_cost[start_flat] = 0.0;
    open.push({static_cast<int>(start_flat),
               heuristic(start_cell.first, start_cell.second)});
    bool found = false;
    while (!open.empty()) {
      const QueueNode node = open.top();
      open.pop();
      const size_t current = static_cast<size_t>(node.idx);
      if (closed[current]) {
        continue;
      }
      closed[current] = 1;
      if (current == goal_flat) {
        found = true;
        break;
      }
      const int cx = static_cast<int>(current % static_cast<size_t>(grid_nx));
      const int cy = static_cast<int>(current / static_cast<size_t>(grid_nx));
      const Eigen::Vector2d cpos = toCoord(cx, cy);
      for (int dir_idx = 0; dir_idx < neighbor_count; ++dir_idx) {
        const int* d = neighbors[dir_idx];
        const int nx = cx + d[0];
        const int ny = cy + d[1];
        if (nx < 0 || nx >= grid_nx || ny < 0 || ny >= grid_ny) {
          continue;
        }
        const size_t nflat = flatIndex(nx, ny);
        if (!is_free[nflat] || closed[nflat]) {
          continue;
        }
        const Eigen::Vector2d npos = toCoord(nx, ny);
        if (!segmentIsFree(cpos, npos)) {
          continue;
        }
        const double step =
            (planner_cfg.a_star_connectivity == 8 && d[0] != 0 && d[1] != 0)
                ? grid_res * std::sqrt(2.0)
                : grid_res;
        const double tentative_g = g_cost[current] + step;
        if (tentative_g < g_cost[nflat]) {
          g_cost[nflat] = tentative_g;
          parent[nflat] = static_cast<int>(current);
          open.push({static_cast<int>(nflat), tentative_g + heuristic(nx, ny)});
        }
      }
    }
    if (!found) {
      return false;
    }
    out_path.clear();
    for (size_t idx = goal_flat; idx != start_flat; idx = static_cast<size_t>(parent[idx])) {
      const int ix = static_cast<int>(idx % static_cast<size_t>(grid_nx));
      const int iy = static_cast<int>(idx / static_cast<size_t>(grid_nx));
      out_path.push_back(toCoord(ix, iy));
    }
    out_path.push_back(toCoord(start_cell.first, start_cell.second));
    std::reverse(out_path.begin(), out_path.end());
    return true;
  };

  std::vector<Eigen::Vector2d> segment_targets = route_waypoints;
  if (!last_guide_is_goal) {
    segment_targets.push_back(goal_xy);
  }

  std::vector<Eigen::Vector2d> path;
  std::pair<int, int> current_cell = safe_start_cell;
  for (size_t segment_idx = 0; segment_idx < segment_targets.size(); ++segment_idx) {
    const Eigen::Vector2d& target_xy = segment_targets[segment_idx];
    const std::pair<int, int> target_cell = toIndex(target_xy);
    const std::pair<int, int> safe_target_cell =
        nearestFreeCell(target_cell.first, target_cell.second, is_free);
    if (safe_target_cell.first < 0) {
      ROS_ERROR_STREAM("No collision-free grid cell found for UGV guide segment "
                       << segment_idx << " target (" << target_xy.x() << ", "
                       << target_xy.y() << ").");
      return result;
    }

    std::vector<Eigen::Vector2d> segment_path;
    if (!buildPath(current_cell, safe_target_cell, segment_path)) {
      ROS_ERROR_STREAM("Segmented A* failed on segment " << segment_idx
                       << " toward target (" << target_xy.x() << ", "
                       << target_xy.y() << ").");
      return result;
    }
    std::vector<Eigen::Vector2d> segment_motion_path;
    segment_motion_path.reserve(segment_path.size());
    size_t anchor = 0;
    while (anchor < segment_path.size()) {
      size_t next = segment_path.size() - 1;
      while (next > anchor + 1) {
        if (segmentIsFree(segment_path[anchor], segment_path[next])) {
          break;
        }
        --next;
      }
      segment_motion_path.push_back(segment_path[anchor]);
      if (next == anchor) {
        break;
      }
      anchor = next;
    }
    if (segment_motion_path.empty() ||
        segment_motion_path.back() != segment_path.back()) {
      segment_motion_path.push_back(segment_path.back());
    }
    segment_motion_path = simplifyCollinearPolyline(segment_motion_path);
    if (planner_cfg.enable_endpoint_snapping &&
        pointIsFree(target_xy) &&
        (segment_motion_path.size() < 2 ||
         segmentIsFree(segment_motion_path[segment_motion_path.size() - 2], target_xy))) {
      segment_motion_path.back() = target_xy;
    }
    if (path.empty()) {
      path = segment_motion_path;
    } else if (!segment_motion_path.empty()) {
      path.insert(path.end(), segment_motion_path.begin() + 1, segment_motion_path.end());
    }
    current_cell = safe_target_cell;
  }

  if (path.empty()) {
    ROS_ERROR("Segmented A* produced an empty path.");
    return result;
  }

  if (planner_cfg.enable_endpoint_snapping &&
      startPointIsFree(start_xy) &&
      startSegmentIsFree(start_xy, path.front())) {
    path.front() = start_xy;
  }
  if (planner_cfg.enable_endpoint_snapping &&
      pointIsFree(final_goal_xy) &&
      segmentIsFree(path.back(), final_goal_xy)) {
    path.back() = final_goal_xy;
  }

  std::vector<Eigen::Vector2d> simplified;
  if (planner_cfg.enable_shortcut_pruning) {
    simplified.reserve(path.size());
    size_t anchor = 0;
    while (anchor < path.size()) {
      size_t next = path.size() - 1;
      while (next > anchor + 1) {
        if (segmentIsFree(path[anchor], path[next])) {
          break;
        }
        --next;
      }
      simplified.push_back(path[anchor]);
      if (next == anchor) {
        break;
      }
      anchor = next;
    }
    if (simplified.empty() || simplified.back() != path.back()) {
      simplified.push_back(path.back());
    }
  } else {
    simplified = path;
  }

  std::vector<Eigen::Vector2d> polyline = simplifyCollinearPolyline(simplified);
  if (polyline.size() < 2) {
    trajectory_duration_out = duration_hint > 1e-3 ? duration_hint : 1.0;
    return result;
  }

  struct MotionPhase {
    enum Type { Translate, Arc } type = Translate;
    Eigen::Vector2d start_pos = Eigen::Vector2d::Zero();
    Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    Eigen::Vector2d radial_start = Eigen::Vector2d::UnitX();
    double start_yaw = 0.0;
    double yaw_rate = 0.0;
    double speed = 0.0;
    double length = 0.0;
    double radius = 0.0;
    double duration = 0.0;
  };

  struct CornerArc {
    bool active = false;
    Eigen::Vector2d tangent_in = Eigen::Vector2d::Zero();
    Eigen::Vector2d tangent_out = Eigen::Vector2d::Zero();
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    double start_yaw = 0.0;
    double yaw_rate = 0.0;
    double speed = 0.0;
    double radius = 0.0;
    double angle = 0.0;
  };

  std::vector<Eigen::Vector2d> segment_dirs(polyline.size() - 1, Eigen::Vector2d::Zero());
  std::vector<double> segment_lengths(polyline.size() - 1, 0.0);
  for (size_t i = 0; i + 1 < polyline.size(); ++i) {
    const Eigen::Vector2d delta = polyline[i + 1] - polyline[i];
    const double length = delta.norm();
    if (length <= 1e-6) {
      continue;
    }
    segment_dirs[i] = delta / length;
    segment_lengths[i] = length;
  }

  std::vector<CornerArc> corner_arcs(polyline.size());
  const double min_turn_radius = speed / yaw_rate_limit;
  auto leftNormal = [](const Eigen::Vector2d& vec) {
    return Eigen::Vector2d(-vec.y(), vec.x());
  };
  for (size_t corner_idx = 1; corner_idx + 1 < polyline.size(); ++corner_idx) {
    const Eigen::Vector2d d_in = segment_dirs[corner_idx - 1];
    const Eigen::Vector2d d_out = segment_dirs[corner_idx];
    const double len_in = segment_lengths[corner_idx - 1];
    const double len_out = segment_lengths[corner_idx];
    if (len_in <= 1e-6 || len_out <= 1e-6) {
      continue;
    }
    const double cross = d_in.x() * d_out.y() - d_in.y() * d_out.x();
    const double dot = std::max(-1.0, std::min(1.0, d_in.dot(d_out)));
    const double angle = std::acos(dot);
    if (std::abs(cross) <= 1e-6 || angle <= 1e-3 || angle >= M_PI - 1e-3) {
      continue;
    }
    const double tan_half = std::tan(0.5 * angle);
    if (tan_half <= 1e-6) {
      continue;
    }
    const double offset_max = 0.49 * std::min(len_in, len_out);
    const double desired_offset = min_turn_radius * tan_half;
    const double offset = std::min(desired_offset, offset_max);
    if (offset <= 1e-6) {
      continue;
    }
    const double radius = offset / tan_half;
    const double arc_speed = std::min(speed, yaw_rate_limit * radius);
    if (radius <= 1e-6 || arc_speed <= 1e-6) {
      continue;
    }
    const double turn_sign = (cross >= 0.0) ? 1.0 : -1.0;
    CornerArc arc;
    arc.active = true;
    arc.tangent_in = polyline[corner_idx] - d_in * offset;
    arc.tangent_out = polyline[corner_idx] + d_out * offset;
    arc.radius = radius;
    arc.speed = arc_speed;
    arc.angle = angle;
    arc.start_yaw = std::atan2(d_in.y(), d_in.x());
    arc.yaw_rate = turn_sign * (arc_speed / radius);
    const Eigen::Vector2d turn_normal =
        (turn_sign > 0.0) ? leftNormal(d_in) : -leftNormal(d_in);
    arc.center = arc.tangent_in + turn_normal * radius;
    corner_arcs[corner_idx] = arc;
  }

  std::vector<MotionPhase> phases;
  phases.reserve(polyline.size() * 2);
  double total_duration = 0.0;
  for (size_t seg_idx = 0; seg_idx + 1 < polyline.size(); ++seg_idx) {
    const Eigen::Vector2d direction = segment_dirs[seg_idx];
    const double direction_norm = direction.norm();
    if (direction_norm <= 1e-6) {
      continue;
    }
    const Eigen::Vector2d seg_start =
        (seg_idx == 0 || !corner_arcs[seg_idx].active)
            ? polyline[seg_idx]
            : corner_arcs[seg_idx].tangent_out;
    const Eigen::Vector2d seg_end =
        (seg_idx + 1 >= polyline.size() - 1 || !corner_arcs[seg_idx + 1].active)
            ? polyline[seg_idx + 1]
            : corner_arcs[seg_idx + 1].tangent_in;
    const double segment_length = (seg_end - seg_start).norm();
    const double segment_yaw = std::atan2(direction.y(), direction.x());

    if (segment_length > 1e-6) {
      MotionPhase translate;
      translate.type = MotionPhase::Translate;
      translate.start_pos = seg_start;
      translate.direction = direction;
      translate.start_yaw = segment_yaw;
      translate.speed = speed;
      translate.length = segment_length;
      translate.duration = segment_length / speed;
      phases.push_back(translate);
      total_duration += translate.duration;
    }

    if (seg_idx + 1 < polyline.size() - 1 && corner_arcs[seg_idx + 1].active) {
      const CornerArc& arc = corner_arcs[seg_idx + 1];
      MotionPhase turn_arc;
      turn_arc.type = MotionPhase::Arc;
      turn_arc.start_pos = arc.tangent_in;
      turn_arc.center = arc.center;
      turn_arc.radial_start = arc.tangent_in - arc.center;
      turn_arc.start_yaw = arc.start_yaw;
      turn_arc.yaw_rate = arc.yaw_rate;
      turn_arc.speed = arc.speed;
      turn_arc.radius = arc.radius;
      turn_arc.duration = arc.angle / std::abs(arc.yaw_rate);
      phases.push_back(turn_arc);
      total_duration += turn_arc.duration;
    }
  }

  if (phases.empty() || total_duration <= 1e-6) {
    trajectory_duration_out = duration_hint > 1e-3 ? duration_hint : 1.0;
    return result;
  }

  // --- S-Curve / 动态速度规划生成 ---
  double total_length = 0.0;
  for (auto& phase : phases) {
    if (phase.type == MotionPhase::Translate) {
      // translate 阶段已分配 phase.length
    } else {
      phase.length = std::abs(phase.radius * phase.duration * phase.yaw_rate); // 弧长 = R * Theta
    }
    total_length += phase.length;
  }

  if (total_length <= 1e-3) {
    trajectory_duration_out = duration_hint > 1e-3 ? duration_hint : 1.0;
    return result;
  }

  // 在空间上进行等距采样 (步长: 0.05m)
  double ds = 0.05;
  int num_s = std::max(2, static_cast<int>(std::ceil(total_length / ds)) + 1);
  ds = total_length / (num_s - 1); // 精确微调
  
  // 辅助函数：根据沿路径长度 s 计算位置、姿态和曲率
  auto evalPhase = [&](double s, Eigen::Vector2d& pos, double& y, double& curv, double& limit_v) {
    double s_accum = 0.0;
    for (const auto& p : phases) {
      if (s <= s_accum + p.length + 1e-6) {
        double tau = (p.length > 1e-6) ? (s - s_accum) / p.length : 0.0;
        tau = std::max(0.0, std::min(1.0, tau));
        limit_v = p.speed;
        if (p.type == MotionPhase::Translate) {
          pos = p.start_pos + p.direction * (tau * p.length);
          y = p.start_yaw;
          curv = 0.0;
        } else {
          double angle_progress = tau * (p.duration * p.yaw_rate);
          double cos_a = std::cos(angle_progress);
          double sin_a = std::sin(angle_progress);
          Eigen::Vector2d radial(
              cos_a * p.radial_start.x() - sin_a * p.radial_start.y(),
              sin_a * p.radial_start.x() + cos_a * p.radial_start.y());
          pos = p.center + radial;
          y = wrapAngle(p.start_yaw + angle_progress);
          curv = p.yaw_rate / std::max(0.01, p.speed); // 曲率与转向半径相关
        }
        return;
      }
      s_accum += p.length;
    }
    // 后备处理: 卡在最后阶段
    const auto& p = phases.back();
    limit_v = p.speed;
    if (p.type == MotionPhase::Translate) {
      pos = p.start_pos + p.direction * p.length;
      y = p.start_yaw;
      curv = 0.0;
    } else {
      double angle_progress = p.duration * p.yaw_rate;
      double cos_a = std::cos(angle_progress);
      double sin_a = std::sin(angle_progress);
      Eigen::Vector2d radial(
          cos_a * p.radial_start.x() - sin_a * p.radial_start.y(),
          sin_a * p.radial_start.x() + cos_a * p.radial_start.y());
      pos = p.center + radial;
      y = wrapAngle(p.start_yaw + angle_progress);
      curv = p.yaw_rate / std::max(0.01, p.speed);
    }
  };

  const double a_acc = planner_cfg.longitudinal_accel_max;
  const double a_dec = planner_cfg.longitudinal_decel_max;
  double v_min = 0.05; // 最低维持速度，降低以减小终点急停时的加速度冲击

  std::vector<double> v_max_grid(num_s);
  for (int i = 0; i < num_s; ++i) {
    Eigen::Vector2d pos;
    double y_val, curv;
    evalPhase(i * ds, pos, y_val, curv, v_max_grid[i]); // 提取每个点的局部上限
  }

  // 1. 前向积分限制加速 (油门限制)
  std::vector<double> v_fwd(num_s, 0.0);
  v_fwd[0] = v_min;
  for (int i = 1; i < num_s; ++i) {
    v_fwd[i] = std::min(v_max_grid[i], std::sqrt(v_fwd[i-1]*v_fwd[i-1] + 2.0 * a_acc * ds));
  }

  // 2. 后向积分限制减速 (提前刹车)
  std::vector<double> v_bwd(num_s, 0.0);
  v_bwd[num_s - 1] = 0.0; // 确保终点速度降为0，完成平滑停车
  for (int i = num_s - 2; i >= 0; --i) {
    v_bwd[i] = std::min(v_fwd[i], std::sqrt(v_bwd[i+1]*v_bwd[i+1] + 2.0 * a_dec * ds));
  }

  // 3. 滑动窗口平滑 (通过平滑梯形曲线，逼近限制 Jerk 的 S-Curve)
  std::vector<double> v_smooth(num_s, 0.0);
  int window = std::max(1, static_cast<int>(1.0 / ds)); // 1.0米的前后平滑窗口
  for (int i = 0; i < num_s; ++i) {
    double sum = 0.0;
    int count = 0;
    for (int j = std::max(0, i - window/2); j <= std::min(num_s - 1, i + window/2); ++j) {
      sum += v_bwd[j];
      count++;
    }
    const double smoothed_speed = std::max(v_min, sum / count);
    // Preserve the local curvature/yaw-rate constraint after smoothing.
    v_smooth[i] = std::min(v_max_grid[i], smoothed_speed);
  }

  // 4. 时域映射积分
  std::vector<double> t_grid(num_s, 0.0);
  for (int i = 1; i < num_s; ++i) {
    t_grid[i] = t_grid[i-1] + 2.0 * ds / (v_smooth[i] + v_smooth[i-1]);
  }

  double total_time = t_grid.back();
  // 移除强制对齐125s长等待，改为实际运行时间附加3.0s的稳定悬停时间，让无人机能彻底停稳并让仿真及时结束
  trajectory_duration_out = (total_time > 1e-6) ? (total_time + 3.0) : duration_hint;
  double dt_res = trajectory_duration_out / static_cast<double>(sample_num - 1);

  std::vector<QuadrotorSimulator::CarState_t> desired_profile;
  desired_profile.reserve(sample_num);
  int grid_idx = 0;

  for (size_t i = 0; i < sample_num; ++i) {
    const double t_target = i * dt_res;

    while (grid_idx < num_s - 1 && t_grid[grid_idx + 1] < t_target) {
      grid_idx++;
    }

    QuadrotorSimulator::CarState_t point;
    point.setZero();

    if (t_target >= t_grid.back() || grid_idx >= num_s - 1) {
      Eigen::Vector2d pos;
      double yaw_val, curv, limit_v;
      evalPhase(total_length, pos, yaw_val, curv, limit_v);

      point(0) = pos.x();
      point(1) = pos.y();

      Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_val, Eigen::Vector3d::UnitZ()));
      point(6) = q.w();
      point(7) = q.x();
      point(8) = q.y();
      point(9) = q.z();
    } else {
      const double dt_interval = t_grid[grid_idx + 1] - t_grid[grid_idx];
      const double tau =
          (dt_interval > 1e-6) ? (t_target - t_grid[grid_idx]) / dt_interval : 0.0;
      const double s_target = ds * (grid_idx + tau);

      Eigen::Vector2d pos;
      double yaw_val, curv, limit_v;
      evalPhase(s_target, pos, yaw_val, curv, limit_v);

      double v_interp =
          v_smooth[grid_idx] + tau * (v_smooth[grid_idx + 1] - v_smooth[grid_idx]);
      v_interp = std::min(v_interp, limit_v);
      const double a_interp =
          (dt_interval > 1e-6)
              ? (v_smooth[grid_idx + 1] - v_smooth[grid_idx]) / dt_interval
              : 0.0;
      double omega = clampValue(v_interp * curv, -yaw_rate_limit, yaw_rate_limit);

      point(0) = pos.x();
      point(1) = pos.y();
      point(3) = v_interp * std::cos(yaw_val);
      point(4) = v_interp * std::sin(yaw_val);

      Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_val, Eigen::Vector3d::UnitZ()));
      point(6) = q.w();
      point(7) = q.x();
      point(8) = q.y();
      point(9) = q.z();

      point(10) = a_interp * std::cos(yaw_val) - v_interp * omega * std::sin(yaw_val);
      point(11) = a_interp * std::sin(yaw_val) + v_interp * omega * std::cos(yaw_val);
      point(15) = omega;
    }

    desired_profile.push_back(point);
  }

  if (desired_profile.empty()) {
    return result;
  }

  result.reserve(sample_num);
  Eigen::Vector2d pos_actual(desired_profile.front()(0), desired_profile.front()(1));
  double yaw_actual =
      yawFromQuaternion(Eigen::Quaterniond(desired_profile.front()(6),
                                           desired_profile.front()(7),
                                           desired_profile.front()(8),
                                           desired_profile.front()(9)));
  double speed_actual = 0.0;
  double omega_actual = 0.0;
  double a_long_actual = 0.0;
  double alpha_actual = 0.0;

  const double speed_limit = speed;
  const double speed_tau = planner_cfg.speed_tracking_tau;
  const double yaw_tau = planner_cfg.yaw_rate_tracking_tau;
  const double jerk_limit = planner_cfg.longitudinal_jerk_max;
  const double yaw_accel_limit = planner_cfg.yaw_accel_max;
  const double yaw_jerk_limit = planner_cfg.yaw_jerk_max;

  for (size_t i = 0; i < desired_profile.size(); ++i) {
    const auto& desired_point = desired_profile[i];
    const Eigen::Vector2d desired_pos(desired_point(0), desired_point(1));
    const double desired_yaw =
        yawFromQuaternion(Eigen::Quaterniond(desired_point(6),
                                             desired_point(7),
                                             desired_point(8),
                                             desired_point(9)));
    const double desired_speed =
        std::hypot(desired_point(3), desired_point(4));
    const double desired_omega_ff = desired_point(15);

    const Eigen::Vector2d desired_tangent(std::cos(desired_yaw), std::sin(desired_yaw));
    const Eigen::Vector2d desired_normal(-desired_tangent.y(), desired_tangent.x());
    const Eigen::Vector2d position_error = desired_pos - pos_actual;
    const double along_track_error = desired_tangent.dot(position_error);
    const double lateral_error = desired_normal.dot(position_error);
    const double yaw_error = wrapAngle(desired_yaw - yaw_actual);

    const double desired_speed_cmd = clampValue(
        desired_speed + planner_cfg.along_track_feedback_gain * along_track_error,
        0.0,
        speed_limit);
    const double desired_omega_cmd = clampValue(
        desired_omega_ff + planner_cfg.heading_feedback_gain * yaw_error +
            planner_cfg.lateral_feedback_gain * lateral_error,
        -yaw_rate_limit,
        yaw_rate_limit);

    const double target_a_long = clampValue(
        (desired_speed_cmd - speed_actual) / speed_tau,
        -planner_cfg.longitudinal_decel_max,
        planner_cfg.longitudinal_accel_max);
    a_long_actual =
        moveTowards(a_long_actual, target_a_long, jerk_limit * dt_res);
    const double speed_next =
        clampValue(speed_actual + a_long_actual * dt_res, 0.0, speed_limit);

    const double target_alpha = clampValue(
        (desired_omega_cmd - omega_actual) / yaw_tau,
        -yaw_accel_limit,
        yaw_accel_limit);
    alpha_actual =
        moveTowards(alpha_actual, target_alpha, yaw_jerk_limit * dt_res);
    const double omega_next = clampValue(
        omega_actual + alpha_actual * dt_res,
        -yaw_rate_limit,
        yaw_rate_limit);

    const double speed_mid = 0.5 * (speed_actual + speed_next);
    const double omega_mid = 0.5 * (omega_actual + omega_next);
    const double yaw_mid = yaw_actual + 0.5 * omega_mid * dt_res;
    const Eigen::Vector2d forward_mid(std::cos(yaw_mid), std::sin(yaw_mid));
    const Eigen::Vector2d pos_next = pos_actual + speed_mid * dt_res * forward_mid;
    const double yaw_next = wrapAngle(yaw_actual + omega_mid * dt_res);

    QuadrotorSimulator::CarState_t point;
    point.setZero();
    point(0) = pos_next.x();
    point(1) = pos_next.y();
    point(3) = speed_next * std::cos(yaw_next);
    point(4) = speed_next * std::sin(yaw_next);

    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_next, Eigen::Vector3d::UnitZ()));
    point(6) = q.w();
    point(7) = q.x();
    point(8) = q.y();
    point(9) = q.z();
    point(10) =
        a_long_actual * std::cos(yaw_next) - speed_next * omega_next * std::sin(yaw_next);
    point(11) =
        a_long_actual * std::sin(yaw_next) + speed_next * omega_next * std::cos(yaw_next);
    point(15) = omega_next;
    result.push_back(point);

    pos_actual = pos_next;
    yaw_actual = yaw_next;
    speed_actual = speed_next;
    omega_actual = omega_next;
  }

  if (!result.empty()) {
    const Eigen::Vector2d final_goal = polyline.back();
    const Eigen::Vector2d final_pos(result.back()(0), result.back()(1));
    if ((final_pos - final_goal).norm() < 0.05) {
      result.back()(0) = final_goal.x();
      result.back()(1) = final_goal.y();
    }
  }

  return result;

}

std::vector<QuadrotorSimulator::CarState_t>
GenerateFigureEightCarTrajectory(size_t sample_num,
                                 double duration_hint,
                                 double trajectory_laps,
                                 double desired_speed,
                                 double yaw_rate,
                                 const UgvPathPlannerConfig& planner_cfg,
                                 const FigureEightGeometryConfig& geometry_cfg,
                                 double& trajectory_duration_out) {
  std::vector<QuadrotorSimulator::CarState_t> result;
  trajectory_duration_out = 0.0;
  if (sample_num < 2) return result;

  const double speed = std::abs(desired_speed);
  const double omega_limit = std::abs(yaw_rate);
  if (speed <= 1e-6 || omega_limit <= 1e-6) return result;

  // Closed Gerono figure-8 geometry:
  //   x = A sin(phi), y = 0.5 A sin(2 phi).
  // start_phase shifts only where the vehicle begins on the same closed path.
  // The emitted vehicle state is evaluated directly from a smooth phase
  // profile phi(t), so the geometric path remains a standard repeated 8.
  // Only the first point starts at rest and the final point returns to rest;
  // intermediate lap crossings keep the cruise phase rate.
  const double amplitude =
      resolveFigureEightAmplitude(speed, omega_limit, geometry_cfg);
  if (amplitude <= 1e-6) return result;
  const double cruise_phase_rate =
      std::min(speed / (amplitude * kFigureEightGeronoMeanSpeedFactor),
               omega_limit / kFigureEightGeronoMaxYawRateFactor);
  if (cruise_phase_rate <= 1e-9) return result;
  const double lap_count = std::max(1.0, trajectory_laps);
  const double total_phase = lap_count * 2.0 * M_PI;
  const double cruise_phase_duration = total_phase / cruise_phase_rate;
  double ramp_duration = std::min(2.0, 0.25 * cruise_phase_duration);
  double tracking_duration = cruise_phase_duration + ramp_duration;
  double phase_rate = cruise_phase_rate;
  if (duration_hint > 1e-6) {
    tracking_duration = duration_hint;
    ramp_duration = std::min(2.0, 0.25 * tracking_duration);
    if (tracking_duration <= ramp_duration + 1e-6) {
      ramp_duration = 0.25 * tracking_duration;
    }
    phase_rate = total_phase / std::max(1e-6, tracking_duration - ramp_duration);
    if (phase_rate > cruise_phase_rate * 1.01) {
      ROS_WARN_STREAM("figure_eight sim_duration_sec forces phase_rate="
                      << phase_rate << " rad/s above nominal cruise limit "
                      << cruise_phase_rate << " rad/s.");
    }
  }
  const double stop_hold_duration = 3.0;
  const double traj_dur = tracking_duration + stop_hold_duration;
  trajectory_duration_out = traj_dur;

  result.reserve(sample_num);
  const double out_dt = traj_dur / static_cast<double>(sample_num - 1);

  struct FigureEightReference {
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d acceleration = Eigen::Vector2d::Zero();
    double yaw = 0.0;
    double speed = 0.0;
    double omega = 0.0;
  };

  auto smoothStep = [](double u) {
    return u * u * (3.0 - 2.0 * u);
  };
  auto smoothStepDeriv = [](double u) {
    return 6.0 * u * (1.0 - u);
  };

  auto evalPhaseProfile = [&](double t_query,
                              double& phase_offset,
                              double& phase_dot,
                              double& phase_ddot) {
    const double t = std::max(0.0, std::min(t_query, tracking_duration));
    phase_offset = 0.0;
    phase_dot = 0.0;
    phase_ddot = 0.0;

    if (ramp_duration <= 1e-9 || tracking_duration <= 2.0 * ramp_duration) {
      phase_offset = std::min(total_phase, phase_rate * t);
      phase_dot = (t < tracking_duration) ? phase_rate : 0.0;
      return;
    }

    const double decel_start = tracking_duration - ramp_duration;
    if (t < ramp_duration) {
      const double u = t / ramp_duration;
      const double integral = u * u * u - 0.5 * u * u * u * u;
      phase_offset = phase_rate * ramp_duration * integral;
      phase_dot = phase_rate * smoothStep(u);
      phase_ddot = phase_rate * smoothStepDeriv(u) / ramp_duration;
    } else if (t < decel_start) {
      phase_offset = phase_rate * (0.5 * ramp_duration + (t - ramp_duration));
      phase_dot = phase_rate;
      phase_ddot = 0.0;
    } else {
      const double u = (t - decel_start) / ramp_duration;
      const double integral = u - u * u * u + 0.5 * u * u * u * u;
      phase_offset =
          phase_rate * (tracking_duration - 1.5 * ramp_duration +
                        ramp_duration * integral);
      phase_dot = phase_rate * (1.0 - smoothStep(u));
      phase_ddot = -phase_rate * smoothStepDeriv(u) / ramp_duration;
    }

    phase_offset = std::max(0.0, std::min(total_phase, phase_offset));
    if (t_query >= tracking_duration) {
      phase_offset = total_phase;
      phase_dot = 0.0;
      phase_ddot = 0.0;
    }
  };

  auto evalReference = [&](double t_query) {
    double phase_offset = 0.0;
    double phase_dot = 0.0;
    double phase_ddot = 0.0;
    evalPhaseProfile(t_query, phase_offset, phase_dot, phase_ddot);

    double phase = std::fmod(geometry_cfg.start_phase + phase_offset,
                             2.0 * M_PI);
    if (phase < 0.0) {
      phase += 2.0 * M_PI;
    }

    const double sin_phase = std::sin(phase);
    const double cos_phase = std::cos(phase);
    const double sin_2phase = std::sin(2.0 * phase);
    const double cos_2phase = std::cos(2.0 * phase);

    const double x = amplitude * sin_phase;
    const double y = 0.5 * amplitude * sin_2phase;
    const Eigen::Vector2d dpos_dphase(amplitude * cos_phase,
                                      amplitude * cos_2phase);
    const Eigen::Vector2d d2pos_dphase2(-amplitude * sin_phase,
                                        -2.0 * amplitude * sin_2phase);
    const Eigen::Vector2d velocity = dpos_dphase * phase_dot;
    const Eigen::Vector2d acceleration =
        d2pos_dphase2 * phase_dot * phase_dot + dpos_dphase * phase_ddot;
    const double vx = velocity.x();
    const double vy = velocity.y();
    const double ax = acceleration.x();
    const double ay = acceleration.y();
    const double speed_sq = vx * vx + vy * vy;
    const double tangent_norm = dpos_dphase.norm();
    const double theta =
        (speed_sq > 1e-12) ? std::atan2(vy, vx)
                           : ((tangent_norm > 1e-12)
                                  ? std::atan2(dpos_dphase.y(), dpos_dphase.x())
                                  : 0.0);
    const double omega_z =
        (speed_sq > 1e-9) ? ((vx * ay - vy * ax) / speed_sq) : 0.0;

    FigureEightReference ref;
    ref.position = Eigen::Vector2d(x, y);
    ref.velocity = velocity;
    ref.acceleration = acceleration;
    ref.yaw = theta;
    ref.speed = std::sqrt(speed_sq);
    ref.omega = omega_z;
    return ref;
  };

  const FigureEightReference initial_ref = evalReference(0.0);
  auto appendState = [&](const FigureEightReference& ref) {
    QuadrotorSimulator::CarState_t point;
    point.setZero();
    point(0) = ref.position.x();
    point(1) = ref.position.y();
    point(3) = ref.velocity.x();
    point(4) = ref.velocity.y();

    const Eigen::AngleAxisd yaw_rot(ref.yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_WB(yaw_rot);
    q_WB.normalize();
    point(6)  = q_WB.w(); point(7)  = q_WB.x();
    point(8)  = q_WB.y(); point(9)  = q_WB.z();
    point(10) = ref.acceleration.x();
    point(11) = ref.acceleration.y();
    point(15) = ref.omega;
    result.push_back(point);
  };

  appendState(initial_ref);
  for (size_t i = 1; i < sample_num; ++i) {
    const double t_query = i * out_dt;
    appendState(evalReference(t_query));
  }

  ROS_INFO_STREAM("Figure-8 trajectory (standard Gerono geometry + smooth phase profile): amplitude="
                  << amplitude << " m, requested_speed=" << speed
                  << " m/s, max_abs_yaw_rate~=" << omega_limit
                  << ", cruise_phase_rate=" << phase_rate
                  << ", ramp_duration=" << ramp_duration
                  << ", geometry_fixed="
                  << (geometry_cfg.fixed_geometry ? "true" : "false")
                  << ", geometry_reference_v=" << geometry_cfg.reference_v
                  << ", geometry_reference_w=" << geometry_cfg.reference_w
                  << ", start_phase=" << geometry_cfg.start_phase
                  << ", start_position=(" << initial_ref.position.x()
                  << ", " << initial_ref.position.y() << ")"
                  << ", cruise_lap_period=" << (2.0 * M_PI / phase_rate)
                  << " s"
                  << ", laps=" << lap_count
                  << ", tracking_duration=" << tracking_duration << " s"
                  << ", total_duration=" << traj_dur << " s"
                  << ", accel_max=" << planner_cfg.longitudinal_accel_max
                  << ", jerk_max=" << planner_cfg.longitudinal_jerk_max
                  << ", yaw_accel_max=" << planner_cfg.yaw_accel_max
                  << ", yaw_jerk_max=" << planner_cfg.yaw_jerk_max);
  return result;
}

std::vector<QuadrotorSimulator::CarState_t>
GenerateCircularCarTrajectory(size_t sample_num,
                              double duration_hint,
                              double trajectory_laps,
                              double desired_speed,
                              double yaw_rate,
                              double radius_override,
                              double& trajectory_duration_out) {
  std::vector<QuadrotorSimulator::CarState_t> result;
  trajectory_duration_out = 0.0;
  if (sample_num < 2) {
    return result;
  }

  const double speed = std::abs(desired_speed);
  const double omega = std::abs(yaw_rate);
  if (speed <= 1e-6 || omega <= 1e-6) {
    return result;
  }

  // Keep v = R * omega consistent when a scene-specific safe radius is
  // requested.  A zero/non-positive override preserves the historical
  // radius=v/omega behavior.
  const double radius = (radius_override > 1e-6)
                            ? radius_override
                            : (speed / omega);
  const double effective_omega = speed / radius;
  const double lap_count = std::max(1.0, trajectory_laps);
  const double trajectory_duration =
      (duration_hint > 1e-6)
          ? duration_hint
          : (lap_count * 2.0 * M_PI / effective_omega);
  const double dt = trajectory_duration / static_cast<double>(sample_num - 1);
  trajectory_duration_out = trajectory_duration;
  result.reserve(sample_num);

  // Start at the bottom of the circle so the initial heading is +x. This keeps
  // the existing world-frame initial offset x0[0](0) += -r consistent with the
  // default relative reference (-r, 0, z).
  for (size_t i = 0; i < sample_num; ++i) {
    const double t = static_cast<double>(i) * dt;
    const double theta = effective_omega * t;
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);
    const double x = radius * sin_theta;
    const double y = -radius * cos_theta;
    const double vx = speed * cos_theta;
    const double vy = speed * sin_theta;
    const double ax = -speed * effective_omega * sin_theta;
    const double ay = speed * effective_omega * cos_theta;
    const Eigen::AngleAxisd yaw_rot(theta, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_WB(yaw_rot);
    q_WB.normalize();

    QuadrotorSimulator::CarState_t point;
    point.setZero();
    point(0) = x;
    point(1) = y;
    point(2) = 0.0;
    point(3) = vx;
    point(4) = vy;
    point(5) = 0.0;
    point(6) = q_WB.w();
    point(7) = q_WB.x();
    point(8) = q_WB.y();
    point(9) = q_WB.z();
    point(10) = ax;
    point(11) = ay;
    point(12) = 0.0;
    point(13) = 0.0;
    point(14) = 0.0;
    point(15) = effective_omega;
    result.push_back(point);
  }

  return result;
}

int main(int argc, char **argv)
{

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  double r = FLAGS_r;
  double v = FLAGS_v;
  double w = FLAGS_w;

  ros::init(argc, argv, "num_sim_non_one_point_node");
  ros::NodeHandle nh; 
  ros::NodeHandle pnh("~");

  pnh.param("num_uavs", num_uavs, num_uavs);
  if (num_uavs <= 0) {
    ROS_WARN("Parameter num_uavs should be positive. Falling back to 1.");
    num_uavs = 1;
  }

  NonInertialPredictorConfig non_inertial_predictor_cfg;
  non_inertial_predictor_cfg.speed_max = std::max(0.0, std::abs(v));
  non_inertial_predictor_cfg.omega_z_max = std::max(0.0, std::abs(w));
  pnh.param("non_inertial_predictor/warmup_frames",
            non_inertial_predictor_cfg.warmup_frames,
            non_inertial_predictor_cfg.warmup_frames);
  pnh.param("non_inertial_predictor/ema_alpha_min",
            non_inertial_predictor_cfg.ema_alpha_min,
            non_inertial_predictor_cfg.ema_alpha_min);
  pnh.param("non_inertial_predictor/ema_alpha_max",
            non_inertial_predictor_cfg.ema_alpha_max,
            non_inertial_predictor_cfg.ema_alpha_max);
  pnh.param("non_inertial_predictor/var_threshold",
            non_inertial_predictor_cfg.var_threshold,
            non_inertial_predictor_cfg.var_threshold);
  double legacy_gamma = non_inertial_predictor_cfg.gamma_a;
  pnh.param("non_inertial_predictor/gamma", legacy_gamma, legacy_gamma);
  pnh.param("non_inertial_predictor/gamma_a",
            non_inertial_predictor_cfg.gamma_a,
            legacy_gamma);
  pnh.param("non_inertial_predictor/gamma_alpha",
            non_inertial_predictor_cfg.gamma_alpha,
            legacy_gamma);
  pnh.param("non_inertial_predictor/speed_max",
            non_inertial_predictor_cfg.speed_max,
            non_inertial_predictor_cfg.speed_max);
  pnh.param("non_inertial_predictor/omega_z_max",
            non_inertial_predictor_cfg.omega_z_max,
            non_inertial_predictor_cfg.omega_z_max);
  pnh.param("non_inertial_predictor/a_max",
            non_inertial_predictor_cfg.a_max,
            non_inertial_predictor_cfg.a_max);
  pnh.param("non_inertial_predictor/alpha_max",
            non_inertial_predictor_cfg.alpha_max,
            non_inertial_predictor_cfg.alpha_max);
  pnh.param("non_inertial_predictor/exit_history_size",
            non_inertial_predictor_cfg.exit_history_size,
            non_inertial_predictor_cfg.exit_history_size);
  pnh.param("non_inertial_predictor/mode",
            non_inertial_predictor_cfg.trend_mode,
            non_inertial_predictor_cfg.trend_mode);
  pnh.param("non_inertial_predictor/debug_log",
            non_inertial_predictor_cfg.debug_log,
            non_inertial_predictor_cfg.debug_log);
  pnh.param("non_inertial_predictor/trend_window_size",
            non_inertial_predictor_cfg.long_trend.window_size,
            non_inertial_predictor_cfg.long_trend.window_size);
  pnh.param("non_inertial_predictor/trend_alpha",
            non_inertial_predictor_cfg.long_trend.alpha,
            non_inertial_predictor_cfg.long_trend.alpha);
  pnh.param("non_inertial_predictor/trend_gamma_min",
            non_inertial_predictor_cfg.long_trend.gamma_min,
            non_inertial_predictor_cfg.long_trend.gamma_min);
  pnh.param("non_inertial_predictor/trend_gamma_max",
            non_inertial_predictor_cfg.long_trend.gamma_max,
            non_inertial_predictor_cfg.long_trend.gamma_max);
  pnh.param("non_inertial_predictor/trend_acc_threshold",
            non_inertial_predictor_cfg.long_trend.acc_threshold,
            non_inertial_predictor_cfg.long_trend.acc_threshold);
  pnh.param("non_inertial_predictor/trend_acc_floor",
            non_inertial_predictor_cfg.long_trend.acc_floor,
            non_inertial_predictor_cfg.long_trend.acc_floor);
  pnh.param("non_inertial_predictor/trend_lambda",
            non_inertial_predictor_cfg.long_trend.trend_lambda,
            non_inertial_predictor_cfg.long_trend.trend_lambda);
  non_inertial_predictor_cfg.angular_trend = non_inertial_predictor_cfg.long_trend;
  pnh.param("non_inertial_predictor/trend_window_size_alpha",
            non_inertial_predictor_cfg.angular_trend.window_size,
            non_inertial_predictor_cfg.angular_trend.window_size);
  pnh.param("non_inertial_predictor/trend_alpha_alpha",
            non_inertial_predictor_cfg.angular_trend.alpha,
            non_inertial_predictor_cfg.angular_trend.alpha);
  pnh.param("non_inertial_predictor/trend_gamma_min_alpha",
            non_inertial_predictor_cfg.angular_trend.gamma_min,
            non_inertial_predictor_cfg.angular_trend.gamma_min);
  pnh.param("non_inertial_predictor/trend_gamma_max_alpha",
            non_inertial_predictor_cfg.angular_trend.gamma_max,
            non_inertial_predictor_cfg.angular_trend.gamma_max);
  pnh.param("non_inertial_predictor/trend_acc_threshold_alpha",
            non_inertial_predictor_cfg.angular_trend.acc_threshold,
            non_inertial_predictor_cfg.angular_trend.acc_threshold);
  pnh.param("non_inertial_predictor/trend_acc_floor_alpha",
            non_inertial_predictor_cfg.angular_trend.acc_floor,
            non_inertial_predictor_cfg.angular_trend.acc_floor);
  pnh.param("non_inertial_predictor/trend_lambda_alpha",
            non_inertial_predictor_cfg.angular_trend.trend_lambda,
            non_inertial_predictor_cfg.angular_trend.trend_lambda);
  pnh.param("non_inertial_predictor/turn_exit_omega_hold",
            non_inertial_predictor_cfg.turn_exit_omega_hold,
            non_inertial_predictor_cfg.turn_exit_omega_hold);
  pnh.param("non_inertial_predictor/turn_exit_omega_low",
            non_inertial_predictor_cfg.turn_exit_omega_low,
            non_inertial_predictor_cfg.turn_exit_omega_low);
  pnh.param("non_inertial_predictor/accel_exit_a_hold",
            non_inertial_predictor_cfg.accel_exit_a_hold,
            non_inertial_predictor_cfg.accel_exit_a_hold);
  pnh.param("non_inertial_predictor/accel_exit_a_low",
            non_inertial_predictor_cfg.accel_exit_a_low,
            non_inertial_predictor_cfg.accel_exit_a_low);
  pnh.param("non_inertial_predictor/cruise_speed_ratio",
            non_inertial_predictor_cfg.cruise_speed_ratio,
            non_inertial_predictor_cfg.cruise_speed_ratio);
  pnh.param("non_inertial_predictor/stop_speed_threshold",
            non_inertial_predictor_cfg.stop_speed_threshold,
            non_inertial_predictor_cfg.stop_speed_threshold);
  pnh.param("non_inertial_predictor/damped_eta_min",
            non_inertial_predictor_cfg.damped_rollout.eta_min,
            non_inertial_predictor_cfg.damped_rollout.eta_min);
  pnh.param("non_inertial_predictor/damped_eps",
            non_inertial_predictor_cfg.damped_rollout.eps,
            non_inertial_predictor_cfg.damped_rollout.eps);
  pnh.param("non_inertial_predictor/damped_lambda_acc",
            non_inertial_predictor_cfg.damped_rollout.lambda_acc,
            non_inertial_predictor_cfg.damped_rollout.lambda_acc);
  pnh.param("non_inertial_predictor/damped_lambda_omega",
            non_inertial_predictor_cfg.damped_rollout.lambda_omega,
            non_inertial_predictor_cfg.damped_rollout.lambda_omega);
  pnh.param("non_inertial_predictor/damped_lambda_beta",
            non_inertial_predictor_cfg.damped_rollout.lambda_beta,
            non_inertial_predictor_cfg.damped_rollout.lambda_beta);
  pnh.param("non_inertial_predictor/enable_acc_damping",
            non_inertial_predictor_cfg.damped_rollout.enable_acc_damping,
            non_inertial_predictor_cfg.damped_rollout.enable_acc_damping);
  pnh.param("non_inertial_predictor/enable_omega_damping",
            non_inertial_predictor_cfg.damped_rollout.enable_omega_damping,
            non_inertial_predictor_cfg.damped_rollout.enable_omega_damping);
  pnh.param("non_inertial_predictor/enable_beta_decay",
            non_inertial_predictor_cfg.damped_rollout.enable_beta_decay,
            non_inertial_predictor_cfg.damped_rollout.enable_beta_decay);

  std::string world_frame_id("map");
  std::string relative_frame_id("base_link");
  std::string topic_prefix;
  pnh.param("world_frame_id", world_frame_id, world_frame_id);
  pnh.param("relative_frame_id", relative_frame_id, relative_frame_id);
  pnh.param("topic_prefix", topic_prefix, std::string(""));
  acado_mpc_common::configureRosNames(world_frame_id, relative_frame_id,
                                      topic_prefix);

  bool use_dynamic_obs = false;
  pnh.param("use_dynamic_obs", use_dynamic_obs, false);
  bool warm_start = true;
  pnh.param("warm_start", warm_start, true);
  bool metrics_debug = false;
  pnh.param("metrics/debug", metrics_debug, false);
  double kappa = 1.0;
  pnh.param("kappa", kappa, 1.0);
  if (kappa < 0.0) {
    ROS_WARN("Parameter kappa should be non-negative. Using zero.");
    kappa = 0.0;
  }
  const double requested_r = r;
  const double requested_v = v;
  const double requested_w = w;
  // kappa only scales the UGV yaw-rate parameter w. Keep the UAV reference
  // offset r unchanged so kappa no longer acts as a coupled geometry term.
  // Build: catkin_make or catkin build
  // Verify: roslaunch coni_mpc num_sim_non_one_point.launch r:=1.0 v:=0.5 w:=0.25 kappa:=1.5
  w *= kappa;
  std::string metrics_csv;
  pnh.param("metrics_csv", metrics_csv, std::string(""));
  std::string run_tag;
  pnh.param("run_tag", run_tag, std::string(""));
  const std::string run_tag_value =
      run_tag.empty() ? ("run_" + std::to_string(ros::Time::now().toNSec()))
                      : run_tag;
  std::string step_metrics_csv;
  pnh.param("step_metrics_csv", step_metrics_csv, std::string(""));
  if (step_metrics_csv.empty()) {
    step_metrics_csv = deriveStepMetricsCsvPath(metrics_csv);
  }
  std::string ugv_horizon_detail_csv;
  pnh.param("ugv_horizon_detail_csv", ugv_horizon_detail_csv, std::string(""));
  if (ugv_horizon_detail_csv.empty()) {
    ugv_horizon_detail_csv = deriveUgvHorizonDetailCsvPath(step_metrics_csv);
  }
  std::string ugv_horizon_summary_csv;
  pnh.param("ugv_horizon_summary_csv", ugv_horizon_summary_csv, std::string(""));
  if (ugv_horizon_summary_csv.empty()) {
    ugv_horizon_summary_csv = deriveUgvHorizonSummaryCsvPath(step_metrics_csv);
  }
  std::string solver_obstacle_drift_csv;
  pnh.param("solver_obstacle_drift_csv", solver_obstacle_drift_csv, std::string(""));
  if (solver_obstacle_drift_csv.empty()) {
    solver_obstacle_drift_csv = deriveSolverObstacleDriftCsvPath(step_metrics_csv);
  }
  std::string external_avoidance_csv;
  pnh.param("external_avoidance/csv",
            external_avoidance_csv,
            std::string(""));
  if (external_avoidance_csv.empty()) {
    external_avoidance_csv = deriveExternalAvoidanceCsvPath(step_metrics_csv);
  }
  std::string safety_variant;
  pnh.param("safety_variant", safety_variant, std::string("A2_soft_cbf"));
  bool exclude_uav0_from_simulation = true;
  pnh.param("exclude_uav0_from_simulation",
            exclude_uav0_from_simulation,
            true);
  bool exclude_uav0_from_all_metrics = true;
  pnh.param("exclude_uav0_from_all_metrics",
            exclude_uav0_from_all_metrics,
            true);
  const std::string frame_mode_effective = resolveFrameModeEffective(pnh);
  const std::string ugv_rollout_mode = resolveUgvRolloutMode(pnh);
  const bool freeze_ugv_rollout = (ugv_rollout_mode == "frozen");
  ExternalAvoidanceConfig external_avoidance_cfg;
  pnh.param("external_avoidance/mode",
            external_avoidance_cfg.mode,
            external_avoidance_cfg.mode);
  external_avoidance_cfg.mode = toLowerCopy(external_avoidance_cfg.mode);
  if (external_avoidance_cfg.mode == "hrvo" ||
      external_avoidance_cfg.mode == "vo-hrvo" ||
      external_avoidance_cfg.mode == "vo_hrvo" ||
      external_avoidance_cfg.mode == "poly_hrvo" ||
      external_avoidance_cfg.mode == "polytopic-hrvo") {
    external_avoidance_cfg.mode = "polytopic_hrvo";
  }
  if (external_avoidance_cfg.mode != "none" &&
      external_avoidance_cfg.mode != "polytopic_hrvo") {
    ROS_ERROR_STREAM("Unsupported external_avoidance/mode='"
                     << external_avoidance_cfg.mode
                     << "'. Valid values are none and polytopic_hrvo.");
    return 2;
  }
  pnh.param("external_avoidance/time_horizon",
            external_avoidance_cfg.time_horizon,
            external_avoidance_cfg.time_horizon);
  int external_max_neighbors =
      static_cast<int>(external_avoidance_cfg.max_neighbors);
  pnh.param("external_avoidance/max_neighbors",
            external_max_neighbors,
            external_max_neighbors);
  external_avoidance_cfg.max_neighbors =
      static_cast<std::size_t>(std::max(0, external_max_neighbors));
  pnh.param("external_avoidance/safety_margin",
            external_avoidance_cfg.safety_margin,
            external_avoidance_cfg.safety_margin);
  int external_polygon_sides =
      static_cast<int>(external_avoidance_cfg.polygon_sides);
  pnh.param("external_avoidance/polygon_sides",
            external_polygon_sides,
            external_polygon_sides);
  external_avoidance_cfg.polygon_sides =
      static_cast<std::size_t>(std::max(0, external_polygon_sides));
  pnh.param("external_avoidance/velocity_window",
            external_avoidance_cfg.velocity_window,
            external_avoidance_cfg.velocity_window);
  pnh.param("external_avoidance/candidate_resolution",
            external_avoidance_cfg.candidate_resolution,
            external_avoidance_cfg.candidate_resolution);
  pnh.param("external_avoidance/fallback_penalty_weight",
            external_avoidance_cfg.fallback_penalty_weight,
            external_avoidance_cfg.fallback_penalty_weight);
  pnh.param("external_avoidance/preferred_velocity_mode",
            external_avoidance_cfg.preferred_velocity_mode,
            external_avoidance_cfg.preferred_velocity_mode);
  external_avoidance_cfg.preferred_velocity_mode =
      toLowerCopy(external_avoidance_cfg.preferred_velocity_mode);
  if (external_avoidance_cfg.preferred_velocity_mode == "slot_ff" ||
      external_avoidance_cfg.preferred_velocity_mode == "slot-velocity-ff" ||
      external_avoidance_cfg.preferred_velocity_mode == "slot_velocity" ||
      external_avoidance_cfg.preferred_velocity_mode == "slot-velocity") {
    external_avoidance_cfg.preferred_velocity_mode = "slot_velocity_ff";
  }
  pnh.param("external_avoidance/goal_gain",
            external_avoidance_cfg.goal_gain,
            external_avoidance_cfg.goal_gain);
  pnh.param("external_avoidance/velocity_compensation_tau_sec",
            external_avoidance_cfg.velocity_compensation_tau_sec,
            external_avoidance_cfg.velocity_compensation_tau_sec);
  if (external_avoidance_cfg.time_horizon <= 0.0 ||
      external_avoidance_cfg.safety_margin < 0.0 ||
      external_avoidance_cfg.polygon_sides < 3u ||
      external_avoidance_cfg.velocity_window < 0.0 ||
      external_avoidance_cfg.candidate_resolution <= 0.0 ||
      external_avoidance_cfg.fallback_penalty_weight < 0.0 ||
      external_avoidance_cfg.goal_gain < 0.0 ||
      external_avoidance_cfg.velocity_compensation_tau_sec < 0.0 ||
      (external_avoidance_cfg.preferred_velocity_mode != "goal_only" &&
       external_avoidance_cfg.preferred_velocity_mode != "slot_velocity_ff")) {
    ROS_ERROR("Invalid polytopic HRVO parameters: require time_horizon>0, "
              "safety_margin>=0, polygon_sides>=3, velocity_window>=0, "
              "candidate_resolution>0, fallback_penalty_weight>=0, "
              "goal_gain>=0, velocity_compensation_tau_sec>=0, and "
              "preferred_velocity_mode in "
              "{goal_only, slot_velocity_ff}.");
    return 2;
  }
  bool cbf_enabled_param = false;
  bool cbf_use_in_sim_param = false;
  pnh.param("cbf/enabled", cbf_enabled_param, cbf_enabled_param);
  pnh.param("cbf/use_in_sim", cbf_use_in_sim_param, cbf_use_in_sim_param);
  if (external_avoidance_cfg.enabled()) {
    if (std::abs(external_avoidance_cfg.time_horizon - 2.0) > 1.0e-9) {
      ROS_ERROR_STREAM(
          "The formal Comment 3.4 baseline requires external_avoidance/time_horizon=2.0 s; resolved "
          << external_avoidance_cfg.time_horizon);
      return 2;
    }
    if (std::abs(external_avoidance_cfg.safety_margin) > 1.0e-9) {
      ROS_ERROR_STREAM(
          "The formal Comment 3.4 polytopic HRVO baseline uses physical "
          "footprints only and requires external_avoidance/safety_margin=0.0; resolved "
          << external_avoidance_cfg.safety_margin);
      return 2;
    }
    const bool valid_baseline =
        frame_mode_effective == "noninertial" &&
        ugv_rollout_mode == "stage" &&
        safety_variant == "A0_no_cbf" &&
        !cbf_enabled_param &&
        !cbf_use_in_sim_param &&
        !use_dynamic_obs;
    if (!valid_baseline) {
      ROS_ERROR_STREAM(
          "external_avoidance=polytopic_hrvo is isolated to the independent "
          "MG-PolyHRVO no-CBF baseline. Required: frame_mode=noninertial, "
          "ugv_rollout_mode=stage, "
          "safety_variant=A0_no_cbf, cbf/enabled=false, cbf/use_in_sim=false. "
          "The current static-obstacle baseline also requires use_dynamic_obs=false. "
          "Resolved: frame_mode=" << frame_mode_effective
          << " rollout=" << ugv_rollout_mode
          << " safety_variant=" << safety_variant
          << " cbf/enabled=" << cbf_enabled_param
          << " cbf/use_in_sim=" << cbf_use_in_sim_param
          << " use_dynamic_obs=" << use_dynamic_obs);
      return 2;
    }
    ROS_INFO_STREAM(
        "Moving-Goal Polytopic HRVO baseline enabled: time_horizon="
        << external_avoidance_cfg.time_horizon
        << " max_neighbors=" << external_avoidance_cfg.max_neighbors
        << " safety_margin=" << external_avoidance_cfg.safety_margin
        << " polygon_sides=" << external_avoidance_cfg.polygon_sides
        << " velocity_window=" << external_avoidance_cfg.velocity_window
        << " candidate_resolution="
        << external_avoidance_cfg.candidate_resolution
        << " fallback_penalty_weight="
        << external_avoidance_cfg.fallback_penalty_weight
        << " preferred_velocity_mode="
        << external_avoidance_cfg.preferred_velocity_mode
        << " goal_gain=" << external_avoidance_cfg.goal_gain
        << " velocity_compensation_tau_sec="
        << external_avoidance_cfg.velocity_compensation_tau_sec
        << ". Static obstacles use VO_p; reciprocal UAVs use HRVO_p; "
           "ACADO MPC and CBF are bypassed for this baseline.");
  }
  ROS_INFO_STREAM("frame_mode_effective=" << frame_mode_effective
                  << " ugv_rollout_mode=" << ugv_rollout_mode
                  << " safety_variant_effective=" << safety_variant
                  << " exclude_uav0_from_simulation="
                  << (exclude_uav0_from_simulation ? "true" : "false")
                  << " exclude_uav0_from_all_metrics="
                  << (exclude_uav0_from_all_metrics ? "true" : "false"));
  if (!step_metrics_csv.empty()) {
    ROS_INFO_STREAM("step_metrics_csv=" << step_metrics_csv);
  }
  if (!ugv_horizon_detail_csv.empty()) {
    ROS_INFO_STREAM("ugv_horizon_detail_csv=" << ugv_horizon_detail_csv);
  }
  if (!ugv_horizon_summary_csv.empty()) {
    ROS_INFO_STREAM("ugv_horizon_summary_csv=" << ugv_horizon_summary_csv);
  }
  if (!solver_obstacle_drift_csv.empty()) {
    ROS_INFO_STREAM("solver_obstacle_drift_csv=" << solver_obstacle_drift_csv);
  }
  ROS_INFO_STREAM("RViz world view uses fixed frame '"
                  << acado_mpc_common::worldFrameId()
                  << "': the car moves through the environment.");
  ROS_INFO_STREAM("RViz non-inertial view uses fixed frame '"
                  << acado_mpc_common::worldFrameId()
                  << "' with target frame '"
                  << acado_mpc_common::relativeFrameId()
                  << "': the camera follows the car while obstacles stay fixed in"
                     " the world frame.");
  int noise_seed = 1;
  pnh.param("noise_seed", noise_seed, 1);
  double start_z = DEFAULT_START_Z;
  pnh.param("dynamic_obs/start_z", start_z, start_z);
  std::vector<int> active_uavs;
  const bool has_explicit_active_uavs =
      pnh.getParam("active_uavs", active_uavs) && !active_uavs.empty();
  if (!has_explicit_active_uavs) {
    active_uavs = buildDefaultActiveUavs(num_uavs, exclude_uav0_from_simulation);
  }
  std::vector<int> filtered_uavs;
  filtered_uavs.reserve(active_uavs.size());
  std::vector<bool> seen(static_cast<size_t>(std::max(1, num_uavs)), false);
  for (int idx : active_uavs) {
    if (idx < 0 || idx >= num_uavs) {
      continue;
    }
    if (seen[static_cast<size_t>(idx)]) {
      continue;
    }
    if (exclude_uav0_from_simulation && num_uavs > 1 && idx == 0) {
      continue;
    }
    seen[static_cast<size_t>(idx)] = true;
    filtered_uavs.push_back(idx);
  }
  if (filtered_uavs.empty()) {
    filtered_uavs = buildDefaultActiveUavs(num_uavs, exclude_uav0_from_simulation);
    ROS_WARN_STREAM("Resolved active_uavs was empty after filtering; falling back to "
                    << joinIntVector(filtered_uavs));
  }
  active_uavs.swap(filtered_uavs);
  const std::string active_uavs_csv = joinIntVector(active_uavs);
  ROS_INFO_STREAM("simulation active_uavs_csv=" << active_uavs_csv);
  const std::vector<int> aggregate_uavs =
      filterAggregateUavs(active_uavs, exclude_uav0_from_all_metrics);
  const std::string aggregate_uavs_csv = joinIntVector(aggregate_uavs);
  if (exclude_uav0_from_all_metrics) {
    ROS_INFO_STREAM("Aggregate metrics exclude UAV0; aggregate_uavs_csv="
                    << aggregate_uavs_csv);
  }

  double uav_radius = 0.25;
  pnh.param("uav_radius", uav_radius, 0.25);
  if (uav_radius <= 0.0) {
    ROS_WARN("Parameter uav_radius should be positive. Using zero radius for collision checks.");
  }
  double collision_safety_margin = 0.0;
  pnh.param("cbf/safety_margin", collision_safety_margin, 0.0);
  if (collision_safety_margin < 0.0) {
    ROS_WARN("Parameter cbf/safety_margin should be non-negative. Using zero for collision checks.");
    collision_safety_margin = 0.0;
  }
  // This margin belongs only to the terminal physical-contact detector.  It is
  // deliberately independent of cbf/safety_margin so a safety-margin
  // violation is not mislabeled as a physical collision.
  double failure_distance_margin = 0.0;
  pnh.param("failure_distance_margin",
            failure_distance_margin,
            failure_distance_margin);
  if (failure_distance_margin < 0.0) {
    ROS_WARN("Parameter failure_distance_margin should be non-negative. Using zero.");
    failure_distance_margin = 0.0;
  }
  ROS_INFO_STREAM("Physical-contact detector margin="
                  << failure_distance_margin
                  << " m (configured cbf/safety_margin="
                  << collision_safety_margin
                  << " m; inactive in the no-CBF HRVO baseline)");
  bool obstacles_for_car_traj_only = false;
  pnh.param("obstacles_for_car_traj_only",
            obstacles_for_car_traj_only,
            obstacles_for_car_traj_only);
  if (external_avoidance_cfg.enabled() && obstacles_for_car_traj_only) {
    ROS_ERROR("external_avoidance=polytopic_hrvo requires obstacles_for_car_traj_only=false so MG-PolyHRVO receives the physical obstacles.");
    return 2;
  }
  bool publish_obstacle_markers = true;
  pnh.param("publish_obstacle_markers",
            publish_obstacle_markers,
            publish_obstacle_markers);
  double post_collision_hold_sec = 3.0;
  pnh.param("post_collision_hold_sec", post_collision_hold_sec, post_collision_hold_sec);
  if (post_collision_hold_sec < 0.0) {
    ROS_WARN("Parameter post_collision_hold_sec should be non-negative. Using zero.");
    post_collision_hold_sec = 0.0;
  }

  bool lidar_collision_enabled = false;
  pnh.param("lidar_collision/enabled", lidar_collision_enabled, false);
  std::string lidar_collision_cloud_topic;
  pnh.param("lidar_collision/cloud_topic", lidar_collision_cloud_topic,
            std::string(""));
  double lidar_collision_timeout_sec = 0.5;
  pnh.param("lidar_collision/timeout_sec", lidar_collision_timeout_sec, 0.5);
  double lidar_collision_point_margin = 0.0;
  pnh.param("lidar_collision/point_margin", lidar_collision_point_margin, 0.0);
  if (lidar_collision_timeout_sec < 0.0) {
    ROS_WARN("lidar_collision/timeout_sec should be non-negative; using zero.");
    lidar_collision_timeout_sec = 0.0;
  }
  if (lidar_collision_point_margin < 0.0) {
    ROS_WARN("lidar_collision/point_margin should be non-negative; using zero.");
    lidar_collision_point_margin = 0.0;
  }
  LidarCollisionMonitor lidar_collision_monitor;
  lidar_collision_monitor.configure(
      nh, lidar_collision_enabled, lidar_collision_cloud_topic,
      acado_mpc_common::worldFrameId(), lidar_collision_timeout_sec,
      lidar_collision_point_margin);

  ros::Publisher obstacle_marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>(
          acado_mpc_common::resolveTopicName("coni_mpc/static_obstacles"), 1,
          true);
  ros::Publisher collision_marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>(
          acado_mpc_common::resolveTopicName("coni_mpc/collision_debug"), 1,
          true);
  ros::Publisher controller_label_marker_pub =
      nh.advertise<visualization_msgs::Marker>(
          acado_mpc_common::resolveTopicName("coni_mpc/controller_label"), 1,
          true);
  const RandomObstacleConfig obstacle_cfg = loadRandomObstacleConfig(pnh);
  UgvPathPlannerConfig ugv_path_cfg = loadUgvPathPlannerConfig(pnh);
  scaleUgvPathPlannerDynamics(ugv_path_cfg, v, w);
  const FigureEightGeometryConfig fig8_geometry_cfg =
      loadFigureEightGeometryConfig(pnh);
  std::vector<StaticObstacle> obstacles = generateRandomCylinders(obstacle_cfg, uav_radius);
  ROS_INFO_STREAM("Generated " << obstacles.size()
                  << " cylindrical obstacles within ["
                  << obstacle_cfg.x_min << ", " << obstacle_cfg.x_max << "] x ["
                  << obstacle_cfg.y_min << ", " << obstacle_cfg.y_max
                  << "], height in [" << obstacle_cfg.h_min << ", "
                  << obstacle_cfg.h_max << "], radius = " << obstacle_cfg.radius);

  // Append analytically-placed strategic obstacles for the figure-8 trajectory.
  // Their positions use the same resolved Gerono amplitude as the car path.
  {
    std::string car_traj_mode_peek;
    pnh.param("car_trajectory_mode", car_traj_mode_peek,
              std::string("obstacle_aware"));
    std::transform(car_traj_mode_peek.begin(), car_traj_mode_peek.end(),
                   car_traj_mode_peek.begin(),
                   [](unsigned char c) {
                     return static_cast<char>(std::tolower(c));
                   });
    if (car_traj_mode_peek == "circle" ||
        car_traj_mode_peek == "circular") {
      const CircleObstacleConfig circle_obs_cfg =
          loadCircleObstacleConfig(pnh);
      const std::vector<StaticObstacle> ring_obstacles =
          generateCircleRingObstacles(
              v, w, r, uav_radius, circle_obs_cfg);
      obstacles.insert(obstacles.end(), ring_obstacles.begin(),
                       ring_obstacles.end());
    } else if (car_traj_mode_peek == "figure_eight" ||
               car_traj_mode_peek == "figure8") {
      const FigureEightObstacleConfig fig8_obs_cfg =
          loadFigureEightObstacleConfig(pnh);
      const std::vector<StaticObstacle> strategic_obs =
          generateFigureEightStrategicObstacles(
              v, w, r, uav_radius, collision_safety_margin, fig8_geometry_cfg,
              fig8_obs_cfg);
      obstacles.insert(obstacles.end(), strategic_obs.begin(),
                       strategic_obs.end());
    }
  }

  if (!ugv_path_cfg.guide_waypoints.empty()) {
    std::ostringstream oss;
    oss << "UGV guide waypoints:";
    for (size_t idx = 0; idx < ugv_path_cfg.guide_waypoints.size(); ++idx) {
      const Eigen::Vector2d& p = ugv_path_cfg.guide_waypoints[idx];
      oss << " [" << idx << "]=(" << p.x() << ", " << p.y() << ")";
    }
    ROS_INFO_STREAM(oss.str());
  } else if (ugv_path_cfg.use_midpoint_guide) {
    ROS_INFO("UGV path generation will insert the legacy midpoint guide.");
  }
  ROS_INFO_STREAM("UGV dynamics limits after V/W scaling: accel_max="
                  << ugv_path_cfg.longitudinal_accel_max
                  << " m/s^2, decel_max="
                  << ugv_path_cfg.longitudinal_decel_max
                  << " m/s^2, jerk_max="
                  << ugv_path_cfg.longitudinal_jerk_max
                  << " m/s^3, yaw_accel_max="
                  << ugv_path_cfg.yaw_accel_max
                  << " rad/s^2, yaw_jerk_max="
                  << ugv_path_cfg.yaw_jerk_max
                  << " rad/s^3, speed_scale=" << std::abs(v)
                  << ", yaw_scale=" << std::abs(w));
  const double span_x = std::max(0.0, obstacle_cfg.x_max - obstacle_cfg.x_min);
  const double span_y = std::max(0.0, obstacle_cfg.y_max - obstacle_cfg.y_min);
  const double margin_abs = 0.5;
  const double margin_x = std::min(margin_abs, 0.5 * span_x);
  const double margin_y = std::min(margin_abs, 0.5 * span_y);
  const Eigen::Vector2d default_goal_xy(obstacle_cfg.x_max - margin_x,
                                        obstacle_cfg.y_max - margin_y);
  const bool last_guide_is_goal =
      ugv_path_cfg.use_last_guide_as_goal && !ugv_path_cfg.guide_waypoints.empty();
  const Eigen::Vector2d configured_goal_xy =
      last_guide_is_goal ? ugv_path_cfg.guide_waypoints.back() : default_goal_xy;
  ROS_INFO_STREAM("UGV A* connectivity=" << ugv_path_cfg.a_star_connectivity
                  << " last_guide_is_goal=" << std::boolalpha
                  << ugv_path_cfg.use_last_guide_as_goal
                  << " final_goal=(" << configured_goal_xy.x() << ", "
                  << configured_goal_xy.y() << ")"
                  << " shortcut_pruning="
                  << ugv_path_cfg.enable_shortcut_pruning
                  << " endpoint_snapping=" << ugv_path_cfg.enable_endpoint_snapping);

  coni_mpc::NumSimMpc::ObstacleVector static_obstacles_for_mpc;
  static_obstacles_for_mpc.reserve(obstacles.size());
  for (const auto& obstacle : obstacles) {
    Eigen::Matrix<double, 7, 1> entry;
    // Treat cylinder as infinite in height for CBF: only use planar radius.
    const double bounding_radius = obstacle.radius;
    entry << obstacle.position.x(),
             obstacle.position.y(),
             obstacle.position.z(),
             bounding_radius,
             obstacle.velocity.x(),
             obstacle.velocity.y(),
             obstacle.velocity.z();
    static_obstacles_for_mpc.push_back(entry);
  }
  if (obstacles_for_car_traj_only) {
    coni_mpc::NumSimMpc::setStaticObstacles(coni_mpc::NumSimMpc::ObstacleVector());
  } else {
    coni_mpc::NumSimMpc::setStaticObstacles(static_obstacles_for_mpc);
  }

  visualization_msgs::MarkerArray obstacle_markers;
  obstacle_markers.markers.reserve(obstacles.size() * 6);
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const StaticObstacle& obs = obstacles[i];
    const double base_z = std::max(0.0, obs.position.z() - 0.5 * obs.height);
    const double segment_step = std::max(0.1, obs.height / 6.0);
    const int segments = std::max(1, static_cast<int>(std::ceil(obs.height / segment_step)));
    const double segment_height = obs.height / static_cast<double>(segments);
    for (int s = 0; s < segments; ++s) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = acado_mpc_common::worldFrameId();
      // Keep stamp at zero so RViz can always use the latest map->base_link
      // transform when the non-inertial view fixes to RELATIVE_FRAME_ID.
      marker.header.stamp = ros::Time(0);
      marker.ns = "static_obstacles";
      marker.id = static_cast<int>(obstacle_markers.markers.size());
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.frame_locked = true;
      marker.pose.position.x = obs.position.x();
      marker.pose.position.y = obs.position.y();
      marker.pose.position.z = base_z + (s + 0.5) * segment_height;
      marker.pose.orientation.w = 1.0;
      const double diameter = 2.0 * obs.radius;
      marker.scale.x = diameter;
      marker.scale.y = diameter;
      marker.scale.z = segment_height;
      const double t = (segments == 1) ? 0.5 : static_cast<double>(s) / static_cast<double>(segments - 1);
      std_msgs::ColorRGBA color = jetColor(t);
      marker.color = color;
      marker.color.a = 0.9;
      marker.lifetime = ros::Duration(0.0);
      obstacle_markers.markers.push_back(marker);
    }
  }
  if (publish_obstacle_markers && !obstacle_markers.markers.empty()) {
    obstacle_marker_pub.publish(obstacle_markers);
  }

  visualization_msgs::Marker controller_label_marker;
  controller_label_marker.header.frame_id = acado_mpc_common::relativeFrameId();
  controller_label_marker.header.stamp = ros::Time(0);
  controller_label_marker.ns = "controller_label";
  controller_label_marker.id = 0;
  controller_label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  controller_label_marker.action = visualization_msgs::Marker::ADD;
  controller_label_marker.frame_locked = true;
  controller_label_marker.pose.orientation.w = 1.0;
  controller_label_marker.pose.position.x = 0.0;
  controller_label_marker.pose.position.y = -2.6;
  controller_label_marker.pose.position.z = 3.8;
  controller_label_marker.scale.z = 0.55;
  controller_label_marker.color = rgba(0.02, 0.02, 0.02, 1.0);
  controller_label_marker.lifetime = ros::Duration(0.0);
  std::ostringstream controller_label_text;
  controller_label_text << "Controller: "
                        << controllerDisplayName(frame_mode_effective,
                                                 ugv_rollout_mode)
                        << (external_avoidance_cfg.enabled()
                                ? "+Polytopic-HRVO"
                                : "")
                        << "\nV=" << FLAGS_v
                        << " W=" << FLAGS_w
                        << " seed=" << obstacle_cfg.seed
                        << " obs=" << obstacles.size();
  controller_label_marker.text = controller_label_text.str();
  controller_label_marker_pub.publish(controller_label_marker);

  std::vector<std::shared_ptr<QuadrotorSimulator>> simulators;

  std::function<void(double)> dynamic_obstacle_callback;
  if (use_dynamic_obs) {
    std::vector<StaticObstacle> base_obstacles = obstacles;
    std::vector<StaticObstacle> moved = base_obstacles;
    coni_mpc::NumSimMpc::ObstacleVector dyn_obstacles;
    dyn_obstacles.resize(moved.size());
    const double dyn_amp = 0.5;
    const double dyn_freq = 0.2;
    dynamic_obstacle_callback =
        [base_obstacles = std::move(base_obstacles),
         moved = std::move(moved),
         dyn_obstacles = std::move(dyn_obstacles),
         dyn_amp,
         dyn_freq,
         obstacles_for_car_traj_only,
         &simulators](double sim_time) mutable {
          const double dx = dyn_amp * std::sin(dyn_freq * sim_time);
          const double dy = dyn_amp * std::cos(dyn_freq * sim_time);
          const double vx = dyn_amp * dyn_freq * std::cos(dyn_freq * sim_time);
          const double vy = -dyn_amp * dyn_freq * std::sin(dyn_freq * sim_time);
          for (size_t i = 0; i < moved.size(); ++i) {
            moved[i] = base_obstacles[i];
            moved[i].position.x() += dx;
            moved[i].position.y() += dy;
            moved[i].velocity = Eigen::Vector3d(vx, vy, 0.0);
            Eigen::Matrix<double, 7, 1>& entry = dyn_obstacles[i];
            entry << moved[i].position.x(),
                     moved[i].position.y(),
                     moved[i].position.z(),
                     moved[i].radius,
                     moved[i].velocity.x(),
                     moved[i].velocity.y(),
                     moved[i].velocity.z();
          }
          if (!obstacles_for_car_traj_only) {
            coni_mpc::NumSimMpc::setStaticObstacles(dyn_obstacles);
          }
          for (const auto& sim : simulators) {
            if (sim) {
              sim->setObstacles(obstacles_for_car_traj_only ? std::vector<StaticObstacle>() : moved);
            }
          }
        };
  }

std::vector<std::shared_ptr<QuadrotorSystem>> quad_system_ptr(num_uavs);
double tau_v_xy = 0.2;
double tau_v_z = 0.2;
pnh.param("tau_v_xy", tau_v_xy, tau_v_xy);
pnh.param("tau_v_z", tau_v_z, tau_v_z);
ROS_INFO_STREAM("[/num_sim_non_one_point_node] tau_v_xy = " << tau_v_xy
                << ", tau_v_z = " << tau_v_z);
for (int i = 0; i < num_uavs; i++) {
    quad_system_ptr[i] = std::make_shared<QuadrotorSystem>(
        "Quadrotor" + std::to_string(i), tau_v_xy, tau_v_z);
}

std::vector<std::shared_ptr<coni_mpc::NumSimMpc>> control_system_ptr(num_uavs);
for (int i = 0; i < num_uavs; i++) {
    control_system_ptr[i] = std::make_shared<coni_mpc::NumSimMpc>(nh, pnh,i);
}


/*  auto control_system_ptr0 = std::make_shared<coni_mpc::NumSimMpc>(nh, pnh,0);
 auto control_system_ptr1 = std::make_shared<coni_mpc::NumSimMpc>(nh, pnh,1); */
/* auto control_system_ptr2 = std::make_shared<coni_mpc::NumSimMpc>(nh, pnh,2);
 auto control_system_ptr3 = std::make_shared<coni_mpc::NumSimMpc>(nh, pnh,3); */

  double car_trajectory_duration = 0.0;
  double car_radius = 0.3;
  pnh.param("car_radius", car_radius, car_radius);
  if (car_radius < 0.0) {
    ROS_WARN("Parameter car_radius should be non-negative. Using zero.");
    car_radius = 0.0;
  }
  (void)car_radius;
  const std::vector<Eigen::Vector3d> all_planning_uav_reference_offsets =
      loadUavReferenceOffsets(pnh, num_uavs, r, 0.0);
  const std::vector<Eigen::Vector3d> planning_uav_reference_offsets =
      selectActiveEntries(all_planning_uav_reference_offsets, active_uavs);
  std::vector<Eigen::Vector2d> planning_uav_offsets_xy;
  planning_uav_offsets_xy.reserve(planning_uav_reference_offsets.size());
  for (const auto& offset : planning_uav_reference_offsets) {
    planning_uav_offsets_xy.emplace_back(offset.x(), offset.y());
  }
  std::string car_trajectory_mode = "obstacle_aware";
  pnh.param("car_trajectory_mode", car_trajectory_mode, car_trajectory_mode);
  std::transform(car_trajectory_mode.begin(),
                 car_trajectory_mode.end(),
                 car_trajectory_mode.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  double sim_duration_sec = -1.0;
  pnh.param("sim_duration_sec", sim_duration_sec, sim_duration_sec);
  double trajectory_laps = 2.0;
  pnh.param("trajectory_laps", trajectory_laps, trajectory_laps);
  if (trajectory_laps < 1.0) {
    ROS_WARN_STREAM("trajectory_laps=" << trajectory_laps
                    << " is invalid; clamping to 1 lap.");
    trajectory_laps = 1.0;
  }
  double ugv_circle_radius = 0.0;
  pnh.param("ugv_circle_radius", ugv_circle_radius, ugv_circle_radius);
  if (ugv_circle_radius < 0.0) {
    ROS_WARN("ugv_circle_radius should be non-negative; using the radius implied by v/w.");
    ugv_circle_radius = 0.0;
  }
  const double periodic_duration_hint = (sim_duration_sec > 0.0) ? sim_duration_sec : -1.0;
  const double effective_duration = (sim_duration_sec > 0.0) ? sim_duration_sec : DURATION;
  std::vector<QuadrotorSimulator::CarState_t> car_trajectory;
  if (car_trajectory_mode == "circle" || car_trajectory_mode == "circular") {
    car_trajectory = GenerateCircularCarTrajectory(
        10000, periodic_duration_hint, trajectory_laps, v, w,
        ugv_circle_radius, car_trajectory_duration);
    const double generated_circle_radius =
        (ugv_circle_radius > 1e-6)
            ? ugv_circle_radius
            : ((std::abs(w) > 1e-6) ? (std::abs(v) / std::abs(w)) : 0.0);
    const double generated_circle_omega =
        (generated_circle_radius > 1e-6)
            ? (std::abs(v) / generated_circle_radius)
            : 0.0;
    ROS_INFO_STREAM("Using circular car trajectory"
                    << " radius=" << generated_circle_radius
                    << " speed=" << std::abs(v)
                    << " yaw_rate=" << generated_circle_omega
                    << " requested_yaw_rate=" << std::abs(w)
                    << " laps=" << trajectory_laps);
  } else if (car_trajectory_mode == "figure_eight" || car_trajectory_mode == "figure8") {
    car_trajectory = GenerateFigureEightCarTrajectory(
        10000, periodic_duration_hint, trajectory_laps, v, w, ugv_path_cfg,
        fig8_geometry_cfg, car_trajectory_duration);
    ROS_INFO_STREAM("Using standard repeated figure-8 car trajectory"
                    << " amplitude="
                    << resolveFigureEightAmplitude(v, w, fig8_geometry_cfg)
                    << " speed=" << std::abs(v)
                    << " yaw_rate=" << std::abs(w)
                    << " fixed_geometry="
                    << (fig8_geometry_cfg.fixed_geometry ? "true" : "false")
                    << " laps=" << trajectory_laps);
  } else {
    car_trajectory = GenerateCarTrajectory(
        10000, effective_duration, v, w, obstacle_cfg, ugv_path_cfg, obstacles, uav_radius,
        car_radius, planning_uav_offsets_xy,
        collision_safety_margin, car_trajectory_duration);
  }
  if (car_trajectory.empty()) {
    ROS_ERROR("Generated car trajectory is empty, aborting simulation.");
    return 1;
  }
  const double actual_predictor_speed_max =
      computeMaxPlanarSpeed(car_trajectory);
  const double actual_predictor_omega_z_max =
      computeMaxAbsYawRate(car_trajectory);
  non_inertial_predictor_cfg.speed_max = actual_predictor_speed_max;
  non_inertial_predictor_cfg.omega_z_max = actual_predictor_omega_z_max;
  ROS_INFO_STREAM("Non-inertial predictor limits aligned to generated UGV trajectory: "
                  << "speed_max=" << actual_predictor_speed_max
                  << " m/s, omega_z_max=" << actual_predictor_omega_z_max
                  << " rad/s"
                  << " (requested_v=" << requested_v
                  << ", requested_w=" << requested_w << ")");
  const double car_start_x = car_trajectory.empty() ? 0.0 : car_trajectory.front()(0);
  const double car_start_y = car_trajectory.empty() ? 0.0 : car_trajectory.front()(1);
  const double car_start_z = car_trajectory.empty() ? 0.0 : car_trajectory.front()(2);
  const std::string traj_id = computeTrajectoryId(car_trajectory);
  const int seed_env = obstacle_cfg.seed;
  const int seed_traj = seed_env;
  const double max_abs_ugv_yaw_rate = actual_predictor_omega_z_max;
  const double turning_omega_thr = 0.5 * max_abs_ugv_yaw_rate;
  ROS_INFO_STREAM("car_start = (" << car_start_x << ", " << car_start_y << ", " << car_start_z << ")"
                  << ", car_trajectory_duration = " << car_trajectory_duration << " s");
  // In non-inertial mode, keep a configurable conversion from world desired height
  // to relative reference height to avoid frame-chain over-compensation.
  double reference_height_coeff = 1;
  pnh.param("reference_height_coeff", reference_height_coeff, 1.0);
  const double reference_height_rel =
      reference_height_coeff * (start_z - car_start_z);
  ROS_INFO_STREAM("reference_height_rel = " << reference_height_rel
                  << " (coeff=" << reference_height_coeff << ")");
  const std::vector<Eigen::Vector3d> uav_reference_offsets =
      loadUavReferenceOffsets(pnh, num_uavs, r, reference_height_rel);
  for (int i = 0; i < num_uavs; ++i) {
    const Eigen::Vector3d& offset = uav_reference_offsets[static_cast<size_t>(i)];
    ROS_INFO_STREAM("uav_reference_offsets[" << i << "] = ("
                    << offset.x() << ", " << offset.y() << ", " << offset.z()
                    << ")");
  }
  Eigen::Quaterniond car_start_orientation(
      car_trajectory.front()(6), car_trajectory.front()(7), car_trajectory.front()(8),
      car_trajectory.front()(9));
  car_start_orientation.normalize();

  std::vector<acado_mpc_common::RelativeTrajectory> trajectory(num_uavs);
  for (int i = 0; i < num_uavs; i++) {
    const Eigen::Vector3d& offset = uav_reference_offsets[static_cast<size_t>(i)];
    trajectory[static_cast<size_t>(i)] = GenerateOnePointTrajectory(
        control_system_ptr[static_cast<size_t>(i)]->WINDOW_NUM,
        offset.x(), offset.y(), offset.z());
    control_system_ptr[i]->setReferenceTrajectory(trajectory[static_cast<size_t>(i)]);
    control_system_ptr[i]->setImu(Eigen::Vector3d(0.0, 0.0, 9.81));
    control_system_ptr[i]->setTurningOmegaThreshold(turning_omega_thr);
    control_system_ptr[i]->setDebugTrajectoryId(traj_id);
  }
  
  auto plan = [](size_t window_num, double time, double duration, size_t sample_num, 
      const std::vector<RelativeTrajectoryPoint> &trajectory) -> RelativeTrajectory
      {
        RelativeTrajectory result;
        for (size_t i = 0; i < window_num; i++)
          result.points.push_back(trajectory.at(0));
        return result;
      };

using State_t = Eigen::Matrix<double, 10, 1>; 
std::vector<QuadrotorSimulator::State_t> x0(num_uavs);
 for (int i = 0; i < num_uavs; i++) 
  {
    x0[i] = QuadrotorSimulator::State_t::Zero();
  } 
 for (int i = 0; i < num_uavs; i++)
  {
    const size_t idx = static_cast<size_t>(i);
    const Eigen::Vector3d world_offset =
        car_start_orientation * uav_reference_offsets[idx];
    const Eigen::Vector3d world_position(car_start_x, car_start_y, car_start_z);
    const Eigen::Vector3d car_start_velocity(
        car_trajectory.front()(3), car_trajectory.front()(4),
        car_trajectory.front()(5));
    const Eigen::Vector3d car_start_omega_non(
        car_trajectory.front()(13), car_trajectory.front()(14),
        car_trajectory.front()(15));
    const Eigen::Vector3d world_init = world_position + world_offset;
    const Eigen::Vector3d world_init_velocity =
        car_start_velocity +
            car_start_orientation *
            car_start_omega_non.cross(uav_reference_offsets[idx]);
    x0[i](0) = world_init.x();
    x0[i](1) = world_init.y();
    x0[i](2) = world_init.z();
    x0[i](3) = world_init_velocity.x();
    x0[i](4) = world_init_velocity.y();
    x0[i](5) = world_init_velocity.z();
    x0[i](6) = car_start_orientation.w();
    x0[i](7) = car_start_orientation.x();
    x0[i](8) = car_start_orientation.y();
    x0[i](9) = car_start_orientation.z();
    ROS_INFO_STREAM("uav " << i << " initial tracking state: world_p=("
                    << world_init.x() << ", " << world_init.y() << ", "
                    << world_init.z() << ") world_v=("
                    << world_init_velocity.x() << ", "
                    << world_init_velocity.y() << ", "
                    << world_init_velocity.z() << ")");
  }

  const double simulation_duration =
      (car_trajectory_duration > 1e-6) ? car_trajectory_duration : DURATION;
  const size_t car_sample_num =
      car_trajectory.empty() ? 0 : car_trajectory.size();
  const int prediction_steps = static_cast<int>(control_system_ptr[0]->WINDOW_NUM);
  const double prediction_dt_sec = control_system_ptr[0]->getPredictionDt();
  auto car_plan = [prediction_dt_sec](
                       size_t window_num, double time, double duration,
                       size_t sample_num,
                       const std::vector<QuadrotorSimulator::CarState_t>& trajectory)
      -> std::vector<QuadrotorSimulator::CarState_t> {
    (void)sample_num;
    std::vector<QuadrotorSimulator::CarState_t> result;
    result.reserve(window_num);
    for (size_t k = 0; k < window_num; ++k) {
      const double stage_time =
          time + static_cast<double>(k) * prediction_dt_sec;
      result.push_back(sampleCarStateAtTime(trajectory, duration, stage_time));
    }
    return result;
  };

simulators.reserve(num_uavs);
std::atomic<bool> collision_reported(false);
std::atomic<bool> aggregate_collision_reported(false);
std::atomic<bool> collision_marker_published(false);
auto collision_handler =
    [&](int quad_index, const Eigen::Vector3d& quad_position,
        const StaticObstacle& obstacle, double distance) {
      if (!exclude_uav0_from_all_metrics || quad_index != 0) {
        aggregate_collision_reported.store(true);
      }
      bool expected = false;
      if (collision_reported.compare_exchange_strong(expected, true)) {
        const double threshold =
            std::max(0.0, uav_radius) + obstacle.radius +
            failure_distance_margin;
        ROS_ERROR_STREAM("Collision detected for UAV " << quad_index
                         << ": distance=" << distance
                         << " threshold=" << threshold
                         << " drone_pos=(" << quad_position.x() << ", "
                         << quad_position.y() << ", "
                         << quad_position.z() << ")"
                         << " obstacle_pos=(" << obstacle.position.x() << ", "
                         << obstacle.position.y() << ", "
                         << obstacle.position.z() << ")");
        visualization_msgs::MarkerArray collision_markers;
        collision_markers.markers.reserve(4);

        visualization_msgs::Marker drone_marker;
        drone_marker.header.frame_id = acado_mpc_common::worldFrameId();
        drone_marker.header.stamp = ros::Time(0);
        drone_marker.ns = "collision_debug";
        drone_marker.id = 0;
        drone_marker.type = visualization_msgs::Marker::SPHERE;
        drone_marker.action = visualization_msgs::Marker::ADD;
        drone_marker.pose.position.x = quad_position.x();
        drone_marker.pose.position.y = quad_position.y();
        drone_marker.pose.position.z = quad_position.z();
        drone_marker.pose.orientation.w = 1.0;
        drone_marker.scale.x = 0.35;
        drone_marker.scale.y = 0.35;
        drone_marker.scale.z = 0.35;
        drone_marker.color = rgba(1.0, 0.1, 0.1, 0.95);
        drone_marker.lifetime = ros::Duration(0.0);
        collision_markers.markers.push_back(drone_marker);

        visualization_msgs::Marker obstacle_marker = drone_marker;
        obstacle_marker.id = 1;
        obstacle_marker.type = visualization_msgs::Marker::CYLINDER;
        obstacle_marker.pose.position.x = obstacle.position.x();
        obstacle_marker.pose.position.y = obstacle.position.y();
        obstacle_marker.pose.position.z = obstacle.position.z();
        obstacle_marker.scale.x = 2.0 * obstacle.radius;
        obstacle_marker.scale.y = 2.0 * obstacle.radius;
        obstacle_marker.scale.z = std::max(0.2, obstacle.height);
        obstacle_marker.color = rgba(1.0, 0.55, 0.0, 0.7);
        collision_markers.markers.push_back(obstacle_marker);

        visualization_msgs::Marker line_marker = drone_marker;
        line_marker.id = 2;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.scale.x = 0.08;
        line_marker.color = rgba(1.0, 1.0, 0.0, 0.98);
        line_marker.points.clear();
        geometry_msgs::Point p0;
        p0.x = quad_position.x();
        p0.y = quad_position.y();
        p0.z = quad_position.z();
        geometry_msgs::Point p1;
        p1.x = obstacle.position.x();
        p1.y = obstacle.position.y();
        p1.z = quad_position.z();
        line_marker.points.push_back(p0);
        line_marker.points.push_back(p1);
        collision_markers.markers.push_back(line_marker);

        visualization_msgs::Marker text_marker = drone_marker;
        text_marker.id = 3;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.pose.position.x = 0.5 * (quad_position.x() + obstacle.position.x());
        text_marker.pose.position.y = 0.5 * (quad_position.y() + obstacle.position.y());
        text_marker.pose.position.z = std::max(quad_position.z(), obstacle.position.z()) + 0.8;
        text_marker.scale.x = 0.0;
        text_marker.scale.y = 0.0;
        text_marker.scale.z = 0.45;
        text_marker.color = rgba(1.0, 1.0, 1.0, 0.98);
        std::ostringstream collision_text;
        collision_text << "collision uav=" << quad_index
                       << " d=" << std::fixed << std::setprecision(3) << distance
                       << " thr=" << threshold;
        text_marker.text = collision_text.str();
        collision_markers.markers.push_back(text_marker);

        collision_marker_pub.publish(collision_markers);
        collision_marker_published.store(true);
        for (const auto& sim : simulators) {
          if (sim) {
            sim->stop();
          }
        }
      }
    };

  /*   QuadrotorSimulator::State_t x0 = QuadrotorSimulator::State_t::Zero();
  x0(0) = -r;
  x0(2) = START_Z;
  x0(6) = 1.0;
 */
  double sim_system_dt_sec = 0.01;
  double sim_control_dt_sec = 0.01;
  bool sim_realtime_pacing = false;
  double sim_startup_delay_sec = 0.0;
  pnh.param("sim_system_dt_sec", sim_system_dt_sec, sim_system_dt_sec);
  pnh.param("sim_control_dt_sec", sim_control_dt_sec, sim_control_dt_sec);
  pnh.param("sim_realtime_pacing", sim_realtime_pacing, sim_realtime_pacing);
  pnh.param("sim_startup_delay_sec", sim_startup_delay_sec,
             sim_startup_delay_sec);
  sim_startup_delay_sec = std::max(0.0, sim_startup_delay_sec);
  if (sim_system_dt_sec <= 0.0 || sim_control_dt_sec <= 0.0 ||
      sim_system_dt_sec > sim_control_dt_sec) {
    ROS_ERROR_STREAM(
        "Invalid simulation timing: require 0 < sim_system_dt_sec <= "
        "sim_control_dt_sec; resolved system="
        << sim_system_dt_sec << " control=" << sim_control_dt_sec);
    return 2;
  }
  if (external_avoidance_cfg.enabled() &&
      std::abs(sim_control_dt_sec - 0.01) > 1.0e-12) {
    ROS_ERROR_STREAM(
        "The formal MG-PolyHRVO baseline updates its moving goal and velocity "
        "command at 100 Hz (sim_control_dt_sec=0.01); resolved "
        << sim_control_dt_sec);
    return 2;
  }
  ROS_INFO_STREAM("Simulation timing: system_dt=" << sim_system_dt_sec
                  << " s, control_dt=" << sim_control_dt_sec << " s ("
                  << 1.0 / sim_control_dt_sec << " Hz), realtime_pacing="
                  << (sim_realtime_pacing ? "true" : "false")
                  << ", startup_delay=" << sim_startup_delay_sec << " s");

for (int i = 0; i < num_uavs; i++) {
  const std::vector<StaticObstacle> simulator_obstacles =
      obstacles_for_car_traj_only ? std::vector<StaticObstacle>() : obstacles;
  const bool owns_dynamic_obstacle_updates =
      !active_uavs.empty() && i == active_uavs.front();
                      
     auto simulator = std::make_shared<QuadrotorSimulator> (
        sim_system_dt_sec,
        sim_control_dt_sec,
        x0[i],                     // 每个无人机的初始状态
        quad_system_ptr[i],        // 每个无人机的系统指针
        control_system_ptr[i],     // 每个无人机的控制器
        trajectory[i].points,      // 每个无人机的轨迹点
        plan,                      // 轨迹规划函数
        car_trajectory,            // 车辆轨迹
        non_inertial_predictor_cfg,
        car_plan,                  // 车辆规划函数
        1.0,                       // 固定点
        10,                        // 采样点数
        simulation_duration,       // 车辆轨迹时长
        car_sample_num,            // 时间步数
        uav_radius,
        collision_safety_margin,
        failure_distance_margin,
        simulator_obstacles,
        dynamic_obstacle_callback,
        collision_handler,
        i,
        owns_dynamic_obstacle_updates,
        freeze_ugv_rollout,
        external_avoidance_cfg
    );
simulators.push_back(simulator);                      
}

/* simulators[0]->car_prepareControllerIteration( DURATION) ; */
/* simulators[1]->simulate(DURATION, 0); */

  std::vector<int> active_uav_indices;
  active_uav_indices.reserve(active_uavs.size());
  auto start_simulator = [&](int idx) {
    if (idx < 0 || idx >= static_cast<int>(simulators.size()) || !simulators[idx]) {
      ROS_WARN_STREAM("Requested UAV index " << idx << " not available; skipping.");
      return;
    }
    simulators[idx]->initializeSimulation();
    active_uav_indices.push_back(idx);
  };

  ros::WallTime wall_start = ros::WallTime::now();

  for (int idx : active_uavs) {
    start_simulator(idx);
  }

  // Perception-driven runs need wall time for the renderer and DSP pipeline
  // to produce the first risk profiles.  Process callbacks while waiting so
  // a latched/early RiskRegionArray is available to the first MPC iteration.
  if (sim_startup_delay_sec > 0.0) {
    const ros::WallTime startup_begin = ros::WallTime::now();
    while (ros::ok() &&
           (ros::WallTime::now() - startup_begin).toSec() <
               sim_startup_delay_sec) {
      ros::spinOnce();
      ros::WallDuration(0.01).sleep();
    }
  }

  const double scheduler_dt_sec =
      active_uav_indices.empty() ? 0.01
                                 : simulators[static_cast<size_t>(active_uav_indices.front())]
                                       ->getControlDtSec();
  const std::size_t total_scheduler_cycles =
      (scheduler_dt_sec > 1e-9)
          ? static_cast<std::size_t>(
                std::ceil(simulation_duration / scheduler_dt_sec))
          : 0u;
  std::size_t scheduler_cycle_count = 0;
  std::size_t scheduler_overrun_count = 0;
  double scheduler_total_cycle_sec = 0.0;
  double scheduler_max_cycle_sec = 0.0;
  double scheduler_max_overrun_sec = 0.0;
  std::ofstream step_metrics_out;
  if (!step_metrics_csv.empty()) {
    const bool need_step_header = !fileHasData(step_metrics_csv);
    step_metrics_out.open(step_metrics_csv.c_str(),
                          std::ios::out | std::ios::app);
    if (!step_metrics_out.good()) {
      ROS_ERROR_STREAM("Failed to open step metrics CSV: "
                       << step_metrics_csv);
    } else if (need_step_header) {
      writeStepMetricsHeader(step_metrics_out);
    }
  }
  std::ofstream ugv_horizon_detail_out;
  if (!ugv_horizon_detail_csv.empty()) {
    const bool need_detail_header = !fileHasData(ugv_horizon_detail_csv);
    ugv_horizon_detail_out.open(ugv_horizon_detail_csv.c_str(),
                                std::ios::out | std::ios::app);
    if (!ugv_horizon_detail_out.good()) {
      ROS_ERROR_STREAM("Failed to open UGV horizon detail CSV: "
                       << ugv_horizon_detail_csv);
    } else if (need_detail_header) {
      writeUgvHorizonDetailHeader(ugv_horizon_detail_out);
    }
  }
  std::ofstream ugv_horizon_summary_out;
  if (!ugv_horizon_summary_csv.empty()) {
    const bool need_summary_header = !fileHasData(ugv_horizon_summary_csv);
    ugv_horizon_summary_out.open(ugv_horizon_summary_csv.c_str(),
                                 std::ios::out | std::ios::app);
    if (!ugv_horizon_summary_out.good()) {
      ROS_ERROR_STREAM("Failed to open UGV horizon summary CSV: "
                       << ugv_horizon_summary_csv);
    } else if (need_summary_header) {
      writeUgvHorizonSummaryHeader(ugv_horizon_summary_out);
    }
  }
  std::ofstream solver_obstacle_drift_out;
  if (!solver_obstacle_drift_csv.empty()) {
    const bool need_solver_obstacle_header =
        !fileHasData(solver_obstacle_drift_csv);
    solver_obstacle_drift_out.open(solver_obstacle_drift_csv.c_str(),
                                   std::ios::out | std::ios::app);
    if (!solver_obstacle_drift_out.good()) {
      ROS_ERROR_STREAM("Failed to open solver obstacle drift CSV: "
                       << solver_obstacle_drift_csv);
    } else if (need_solver_obstacle_header) {
      writeSolverObstacleDriftHeader(solver_obstacle_drift_out);
    }
  }
  std::ofstream external_avoidance_out;
  if (external_avoidance_cfg.enabled() && !external_avoidance_csv.empty()) {
    const bool need_external_header = !fileHasData(external_avoidance_csv);
    external_avoidance_out.open(external_avoidance_csv.c_str(),
                                std::ios::out | std::ios::app);
    if (!external_avoidance_out.good()) {
      ROS_ERROR_STREAM("Failed to open external VO/HRVO CSV: "
                       << external_avoidance_csv);
    } else if (need_external_header) {
      writeExternalAvoidanceHeader(external_avoidance_out);
    }
  }
  std::vector<std::size_t> step_row_counts(
      static_cast<std::size_t>(std::max(1, num_uavs)), 0);
  std::size_t ugv_horizon_row_count = 0;
  for (std::size_t cycle_idx = 0;
       ros::ok() && cycle_idx < total_scheduler_cycles &&
       !collision_reported.load();
       ++cycle_idx) {
    const auto cycle_start = std::chrono::high_resolution_clock::now();
    // Deliver risk-region and other ROS callbacks before constructing the
    // stage-wise OnlineData for this MPC iteration.
    ros::spinOnce();
    if (lidar_collision_monitor.enabled() && !collision_reported.load()) {
      for (const int idx : active_uav_indices) {
        if (idx < 0 || idx >= static_cast<int>(simulators.size()) ||
            !simulators[idx]) {
          continue;
        }
        const Eigen::Vector3d quad_position =
            simulators[idx]->getWorldPosition();
        Eigen::Vector3d nearest_point = Eigen::Vector3d::Zero();
        double nearest_distance = std::numeric_limits<double>::infinity();
        if (!lidar_collision_monitor.nearestPoint(
                quad_position, nearest_distance, nearest_point)) {
          continue;
        }
        const double collision_threshold =
            std::max(0.0, uav_radius) + failure_distance_margin +
            lidar_collision_monitor.pointMargin();
        // Tangency counts as contact: the UAV radius edge touching a world
        // point on the obstacle surface terminates the simulation.
        if (nearest_distance <= collision_threshold) {
          StaticObstacle lidar_obstacle;
          lidar_obstacle.position = nearest_point;
          lidar_obstacle.velocity = Eigen::Vector3d::Zero();
          // Treat the point-sampling margin as the radius of the synthetic
          // obstacle so the existing collision marker and threshold log show
          // the exact world-coordinate test that triggered termination.
          lidar_obstacle.radius = lidar_collision_monitor.pointMargin();
          lidar_obstacle.height = 0.0;
          collision_handler(idx, quad_position, lidar_obstacle,
                            nearest_distance);
          break;
        }
      }
    }
    if (collision_reported.load()) {
      break;
    }
    const double sim_time =
        static_cast<double>(cycle_idx) * scheduler_dt_sec;
    const ros::Time cycle_stamp = ros::Time::now();
    for (int idx : active_uav_indices) {
      if (idx >= 0 && idx < static_cast<int>(simulators.size()) && simulators[idx]) {
        simulators[idx]->captureCycleQuadOdomSnapshot(idx, cycle_stamp);
      }
    }
    for (int idx : active_uav_indices) {
      if (idx >= 0 && idx < static_cast<int>(simulators.size()) && simulators[idx]) {
        simulators[idx]->prepareControllerIteration(sim_time, idx);
      }
    }
    for (int idx : active_uav_indices) {
      if (idx >= 0 && idx < static_cast<int>(simulators.size()) && simulators[idx]) {
        simulators[idx]->commitCycleQuadOdomSnapshot();
      }
    }
    if (!active_uav_indices.empty()) {
      const int first_idx = active_uav_indices.front();
      if (first_idx >= 0 &&
          first_idx < static_cast<int>(simulators.size()) &&
          simulators[first_idx]) {
        const auto& ugv_snapshot =
            simulators[static_cast<size_t>(first_idx)]
                ->getLastUgvHorizonCompareSnapshot();
        if (ugv_snapshot.valid) {
          if (ugv_horizon_detail_out.good()) {
            writeUgvHorizonDetailRows(ugv_horizon_detail_out,
                                      run_tag_value,
                                      ugv_horizon_row_count,
                                      frame_mode_effective,
                                      ugv_rollout_mode,
                                      ugv_snapshot);
          }
          if (ugv_horizon_summary_out.good()) {
            writeUgvHorizonSummaryRow(ugv_horizon_summary_out,
                                      run_tag_value,
                                      ugv_horizon_row_count,
                                      frame_mode_effective,
                                      ugv_rollout_mode,
                                      ugv_snapshot);
          }
          ugv_horizon_row_count += 1;
        }
      }
    }
    for (int idx : active_uav_indices) {
      if (idx >= 0 && idx < static_cast<int>(simulators.size()) && simulators[idx]) {
        simulators[idx]->finishControllerIteration(sim_time, idx);
        const std::size_t current_step_idx =
            step_row_counts[static_cast<size_t>(idx)];
        if (external_avoidance_out.good()) {
          const auto& external_snapshot =
              simulators[static_cast<size_t>(idx)]
                  ->getLastExternalAvoidanceSnapshot();
          if (external_snapshot.valid) {
            writeExternalAvoidanceRow(external_avoidance_out,
                                      run_tag_value,
                                      idx,
                                      current_step_idx,
                                      external_avoidance_cfg.mode,
                                      external_snapshot);
          }
        }
        if (solver_obstacle_drift_out.good()) {
          const auto& drift_snapshot =
              simulators[static_cast<size_t>(idx)]
                  ->getLastSolverObstacleDriftSnapshot();
          if (drift_snapshot.valid) {
            writeSolverObstacleDriftRows(solver_obstacle_drift_out,
                                         run_tag_value,
                                         current_step_idx,
                                         frame_mode_effective,
                                         ugv_rollout_mode,
                                         drift_snapshot);
          }
        }
        if (step_metrics_out.good() &&
            idx < static_cast<int>(control_system_ptr.size()) &&
            control_system_ptr[static_cast<size_t>(idx)]) {
          const auto step_debug =
              control_system_ptr[static_cast<size_t>(idx)]
                  ->getLastStepDebugSnapshot();
          if (step_debug.valid) {
            writeStepMetricsRow(step_metrics_out,
                                run_tag_value,
                                idx,
                                current_step_idx,
                                sim_time,
                                frame_mode_effective,
                                ugv_rollout_mode,
                                safety_variant,
                                step_debug,
                                simulators[static_cast<size_t>(idx)]
                                    ->getLastAppliedControlWorld(),
                                simulators[static_cast<size_t>(idx)]
                                    ->getTruthMinPlanarSurfaceClearance(),
                                simulators[static_cast<size_t>(idx)]
                                        ->getLastExternalAvoidanceSnapshot()
                                        .valid
                                    ? simulators[static_cast<size_t>(idx)]
                                          ->getLastExternalAvoidanceSnapshot()
                                          .filter_time_ms
                                    : 0.0);
            step_row_counts[static_cast<size_t>(idx)] += 1;
          }
        }
      }
    }
    for (int idx : active_uav_indices) {
      if (idx >= 0 && idx < static_cast<int>(simulators.size()) && simulators[idx]) {
        simulators[idx]->advanceOneControlStep(sim_time);
      }
    }

    // Controller rows above describe the state at the start of this control
    // interval.  If contact occurs during integration, also retain the
    // terminal post-integration state; otherwise the aggregate collision flag
    // can be true while the sampled physical clearance remains positive.
    if (collision_reported.load() && step_metrics_out.good()) {
      const double terminal_sim_time = sim_time + scheduler_dt_sec;
      for (int idx : active_uav_indices) {
        if (idx < 0 || idx >= static_cast<int>(simulators.size()) ||
            !simulators[static_cast<size_t>(idx)] ||
            idx >= static_cast<int>(control_system_ptr.size()) ||
            !control_system_ptr[static_cast<size_t>(idx)]) {
          continue;
        }
        const auto step_debug =
            control_system_ptr[static_cast<size_t>(idx)]
                ->getLastStepDebugSnapshot();
        if (!step_debug.valid) {
          continue;
        }
        const auto& external_snapshot =
            simulators[static_cast<size_t>(idx)]
                ->getLastExternalAvoidanceSnapshot();
        writeStepMetricsRow(
            step_metrics_out,
            run_tag_value,
            idx,
            step_row_counts[static_cast<size_t>(idx)],
            terminal_sim_time,
            frame_mode_effective,
            ugv_rollout_mode,
            safety_variant,
            step_debug,
            simulators[static_cast<size_t>(idx)]->getLastAppliedControlWorld(),
            simulators[static_cast<size_t>(idx)]
                ->getTruthMinPlanarSurfaceClearance(),
            external_snapshot.valid ? external_snapshot.filter_time_ms : 0.0);
        step_row_counts[static_cast<size_t>(idx)] += 1;
      }
    }

    const auto cycle_end = std::chrono::high_resolution_clock::now();
    const double cycle_duration_sec =
        std::chrono::duration<double>(cycle_end - cycle_start).count();
    ++scheduler_cycle_count;
    scheduler_total_cycle_sec += cycle_duration_sec;
    scheduler_max_cycle_sec = std::max(scheduler_max_cycle_sec, cycle_duration_sec);
    if (cycle_duration_sec > scheduler_dt_sec) {
      ++scheduler_overrun_count;
      scheduler_max_overrun_sec =
          std::max(scheduler_max_overrun_sec, cycle_duration_sec - scheduler_dt_sec);
      ROS_WARN_STREAM_THROTTLE(
          1.0,
          "MPC scheduler overrun: cycle=" << cycle_duration_sec * 1000.0
                                           << " ms deadline="
                                           << scheduler_dt_sec * 1000.0
                                           << " ms active_uavs="
                                           << active_uav_indices.size());
    }
    if (sim_realtime_pacing && cycle_duration_sec < scheduler_dt_sec) {
      ros::WallDuration(scheduler_dt_sec - cycle_duration_sec).sleep();
    }
  }

  if (scheduler_cycle_count > 0) {
    ROS_INFO_STREAM("MPC scheduler summary: cycles=" << scheduler_cycle_count
                    << " mean_cycle_ms="
                    << (scheduler_total_cycle_sec / scheduler_cycle_count) * 1000.0
                    << " max_cycle_ms=" << scheduler_max_cycle_sec * 1000.0
                    << " overruns=" << scheduler_overrun_count
                    << " max_overrun_ms=" << scheduler_max_overrun_sec * 1000.0);
  }

  for (int idx : active_uav_indices) {
    if (idx >= 0 && idx < static_cast<int>(simulators.size()) && simulators[idx]) {
      simulators[idx]->stop();
    }
  }

  for (int idx : active_uav_indices) {
    simulators[idx]->finish();
  }

  const double wall_time_sec = (ros::WallTime::now() - wall_start).toSec();
  double actual_sim_time_sec = 0.0;
  for (int idx : active_uav_indices) {
    if (idx >= 0 && idx < static_cast<int>(simulators.size()) && simulators[idx]) {
      actual_sim_time_sec = std::max(actual_sim_time_sec, simulators[idx]->getSimTimeSec());
    }
  }
  if (!metrics_csv.empty()) {
    const bool need_header = !fileHasData(metrics_csv);
    std::ofstream out(metrics_csv.c_str(), std::ios::out | std::ios::app);
    if (!out.good()) {
      ROS_ERROR_STREAM("Failed to open metrics CSV: " << metrics_csv);
    } else {
      if (need_header) {
        writeMetricsHeader(out);
      }
      auto makeEmptySummary = []() {
        coni_mpc::NumSimMpc::MetricsSummary empty;
        empty.samples = 0;
        empty.min_h = std::numeric_limits<double>::quiet_NaN();
        empty.min_cbf = std::numeric_limits<double>::quiet_NaN();
        empty.max_slack = std::numeric_limits<double>::quiet_NaN();
        empty.mean_h = std::numeric_limits<double>::quiet_NaN();
        empty.mean_cbf = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_rms = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_rms_x = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_rms_y = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_rms_z = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_p95 = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_rms_track_only = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_rms_track_only_x = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_rms_track_only_y = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_rms_track_only_z = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_p95_track_only = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_rms_cbf_active = std::numeric_limits<double>::quiet_NaN();
        empty.solver_tracking_rms_avoid = std::numeric_limits<double>::quiet_NaN();
        empty.solver_tracking_rms_avoid_samples = 0;
        empty.fail_rate_turn = std::numeric_limits<double>::quiet_NaN();
        empty.fail_rate_straight = std::numeric_limits<double>::quiet_NaN();
        empty.min_h_turn = std::numeric_limits<double>::quiet_NaN();
        empty.min_h_straight = std::numeric_limits<double>::quiet_NaN();
        empty.slack_sum_turn = std::numeric_limits<double>::quiet_NaN();
        empty.slack_sum_straight = std::numeric_limits<double>::quiet_NaN();
        empty.max_slack_turn = std::numeric_limits<double>::quiet_NaN();
        empty.max_slack_straight = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_rms_all_turn = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_rms_all_straight = std::numeric_limits<double>::quiet_NaN();
        empty.c44_metric = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_rms_high_c44 = std::numeric_limits<double>::quiet_NaN();
        empty.c66_metric = std::numeric_limits<double>::quiet_NaN();
        empty.c88_metric = std::numeric_limits<double>::quiet_NaN();
        empty.min_h_high_c = std::numeric_limits<double>::quiet_NaN();
        empty.slack_sum_high_c = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_rms_high_c = std::numeric_limits<double>::quiet_NaN();
        empty.tracking_rms_high_c88 = std::numeric_limits<double>::quiet_NaN();
        empty.fail_rate_high_c = std::numeric_limits<double>::quiet_NaN();
        empty.solver_min_h = std::numeric_limits<double>::quiet_NaN();
        empty.solver_active_mean_h = std::numeric_limits<double>::quiet_NaN();
        empty.solver_min_planar_clearance =
            std::numeric_limits<double>::quiet_NaN();
        empty.solver_min_h_witness_h = std::numeric_limits<double>::quiet_NaN();
        empty.solver_min_h_witness_planar_clearance =
            std::numeric_limits<double>::quiet_NaN();
        empty.solver_min_h_witness_tracking_error =
            std::numeric_limits<double>::quiet_NaN();
        empty.solver_min_h_witness_delta =
            std::numeric_limits<double>::quiet_NaN();
        empty.solver_min_h_witness_planar_distance =
            std::numeric_limits<double>::quiet_NaN();
        empty.solver_min_h_witness_active_obstacle_key = "none";
        empty.solver_min_cbf = std::numeric_limits<double>::quiet_NaN();
        empty.solver_mean_h = std::numeric_limits<double>::quiet_NaN();
        empty.solver_mean_cbf = std::numeric_limits<double>::quiet_NaN();
        empty.solver_slack_sum = std::numeric_limits<double>::quiet_NaN();
        empty.solver_max_slack = std::numeric_limits<double>::quiet_NaN();
        empty.solver_fail_rate = std::numeric_limits<double>::quiet_NaN();
        empty.solver_fail_rate_turn = std::numeric_limits<double>::quiet_NaN();
        empty.solver_fail_rate_straight = std::numeric_limits<double>::quiet_NaN();
        empty.solver_min_h_turn = std::numeric_limits<double>::quiet_NaN();
        empty.solver_min_h_straight = std::numeric_limits<double>::quiet_NaN();
        empty.solver_slack_sum_turn = std::numeric_limits<double>::quiet_NaN();
        empty.solver_slack_sum_straight = std::numeric_limits<double>::quiet_NaN();
        empty.solver_max_slack_turn = std::numeric_limits<double>::quiet_NaN();
        empty.solver_max_slack_straight = std::numeric_limits<double>::quiet_NaN();
        empty.solver_min_h_high_c = std::numeric_limits<double>::quiet_NaN();
        empty.solver_slack_sum_high_c = std::numeric_limits<double>::quiet_NaN();
        empty.solver_fail_rate_high_c = std::numeric_limits<double>::quiet_NaN();
        empty.diag_min_h_any = std::numeric_limits<double>::quiet_NaN();
        empty.diag_mean_h_any = std::numeric_limits<double>::quiet_NaN();
        empty.diag_min_cbf_any = std::numeric_limits<double>::quiet_NaN();
        empty.diag_mean_cbf_any = std::numeric_limits<double>::quiet_NaN();
        empty.dmin_min = std::numeric_limits<double>::quiet_NaN();
        empty.slack_sum = std::numeric_limits<double>::quiet_NaN();
        empty.solve_time_mean_ms = std::numeric_limits<double>::quiet_NaN();
        empty.solve_time_p95_ms = std::numeric_limits<double>::quiet_NaN();
        empty.solve_time_max_ms = std::numeric_limits<double>::quiet_NaN();
        empty.feedback_time_mean_ms = std::numeric_limits<double>::quiet_NaN();
        empty.feedback_time_p95_ms = std::numeric_limits<double>::quiet_NaN();
        empty.feedback_time_max_ms = std::numeric_limits<double>::quiet_NaN();
        empty.preparation_time_mean_ms = std::numeric_limits<double>::quiet_NaN();
        empty.preparation_time_p95_ms = std::numeric_limits<double>::quiet_NaN();
        empty.preparation_time_max_ms = std::numeric_limits<double>::quiet_NaN();
        empty.fail_rate = std::numeric_limits<double>::quiet_NaN();
        empty.solver_cbf_active_samples = 0;
        empty.solver_track_only_samples = 0;
        empty.cbf_active_samples = 0;
        empty.high_c44_samples = 0;
        empty.high_c_samples = 0;
        empty.high_c88_samples = 0;
        empty.track_only_samples = 0;
        empty.turn_samples = 0;
        empty.straight_samples = 0;
        return empty;
      };

      std::vector<coni_mpc::NumSimMpc::MetricsSummary> summaries(
          static_cast<size_t>(num_uavs), makeEmptySummary());
      for (size_t i = 0; i < control_system_ptr.size(); ++i) {
        const auto& ctrl = control_system_ptr[i];
        if (ctrl) {
          summaries[i] = ctrl->getMetrics();
        }
      }

      std::size_t total_samples = 0;
      double all_min_h = std::numeric_limits<double>::infinity();
      double all_min_cbf = std::numeric_limits<double>::infinity();
      double all_dmin_min = std::numeric_limits<double>::infinity();
      double all_max_slack = 0.0;
      double sum_h = 0.0;
      double sum_cbf = 0.0;
      double tracking_rms_sum_sq = 0.0;
      double tracking_rms_x_sum_sq = 0.0;
      double tracking_rms_y_sum_sq = 0.0;
      double tracking_rms_z_sum_sq = 0.0;
      double tracking_rms_cbf_active_sum_sq = 0.0;
      double solver_tracking_rms_avoid_sum_sq = 0.0;
      double tracking_rms_track_only_x_sum_sq = 0.0;
      double tracking_rms_track_only_y_sum_sq = 0.0;
      double tracking_rms_track_only_z_sum_sq = 0.0;
      double tracking_rms_turn_sum_sq = 0.0;
      double tracking_rms_straight_sum_sq = 0.0;
      double tracking_rms_high_c44_sum_sq = 0.0;
      double tracking_rms_high_c_sum_sq = 0.0;
      double tracking_rms_high_c88_sum_sq = 0.0;
      double slack_sum = 0.0;
      double slack_sum_turn = 0.0;
      double slack_sum_straight = 0.0;
      double slack_sum_high_c = 0.0;
      double diag_sum_h_any = 0.0;
      double diag_sum_cbf_any = 0.0;
      double solve_time_mean_sum = 0.0;
      double solve_time_max = 0.0;
      double feedback_time_mean_sum = 0.0;
      double feedback_time_max = 0.0;
      double preparation_time_mean_sum = 0.0;
      double preparation_time_max = 0.0;
      double tracking_p95_max = -std::numeric_limits<double>::infinity();
      double solve_time_p95_max = -std::numeric_limits<double>::infinity();
      double feedback_time_p95_max = -std::numeric_limits<double>::infinity();
      double preparation_time_p95_max = -std::numeric_limits<double>::infinity();
      double all_min_h_turn = std::numeric_limits<double>::infinity();
      double all_min_h_straight = std::numeric_limits<double>::infinity();
      double all_min_h_high_c = std::numeric_limits<double>::infinity();
      double all_solver_min_h = std::numeric_limits<double>::infinity();
      double all_solver_min_planar_clearance =
          std::numeric_limits<double>::infinity();
      double all_solver_min_h_witness_h =
          std::numeric_limits<double>::quiet_NaN();
      double all_solver_min_h_witness_planar_clearance =
          std::numeric_limits<double>::quiet_NaN();
      double all_solver_min_h_witness_tracking_error =
          std::numeric_limits<double>::quiet_NaN();
      double all_solver_min_h_witness_delta =
          std::numeric_limits<double>::quiet_NaN();
      double all_solver_min_h_witness_planar_distance =
          std::numeric_limits<double>::quiet_NaN();
      std::string all_solver_min_h_witness_active_obstacle_key = "none";
      bool has_solver_min_h_witness = false;
      double all_diag_min_h_any = std::numeric_limits<double>::infinity();
      double all_diag_min_cbf_any = std::numeric_limits<double>::infinity();
      double all_max_slack_turn = 0.0;
      double all_max_slack_straight = 0.0;
      std::vector<double> tracking_errors_all;
      std::vector<double> tracking_errors_track_only_all;
      std::vector<double> solve_time_all;
      std::vector<double> feedback_time_all;
      std::vector<double> preparation_time_all;
      double total_failures = 0.0;
      double total_failures_turn = 0.0;
      double total_failures_straight = 0.0;
      double total_failures_high_c = 0.0;
      std::size_t total_solver_metric_samples = 0;
      std::size_t total_solver_tracking_rms_avoid_samples = 0;
      std::size_t total_cbf_active_samples = 0;
      std::size_t total_solver_track_only_samples = 0;
      std::size_t total_high_c44_samples = 0;
      std::size_t total_high_c_samples = 0;
      std::size_t total_high_c88_samples = 0;
      std::size_t total_turn_samples = 0;
      std::size_t total_straight_samples = 0;
      bool has_tracking_p95 = false;
      bool has_solve_p95 = false;
      bool has_solve_max = false;
      bool has_tracking_rms = false;
      bool has_tracking_rms_cbf_active = false;
      bool has_solver_tracking_rms_avoid = false;
      bool has_tracking_rms_xyz = false;
      bool has_tracking_rms_track_only_xyz = false;
      bool has_solve_mean = false;
      bool has_feedback_mean = false;
      bool has_preparation_mean = false;
      bool has_feedback_p95 = false;
      bool has_preparation_p95 = false;
      bool has_feedback_max = false;
      bool has_preparation_max = false;
      for (int idx : aggregate_uavs) {
        if (idx < 0 || idx >= static_cast<int>(summaries.size())) {
          continue;
        }
        const auto& summary = summaries[static_cast<size_t>(idx)];
        if (summary.samples == 0) {
          continue;
        }
        total_samples += summary.samples;
        total_solver_metric_samples += summary.solver_cbf_active_samples;
        if (std::isfinite(summary.min_h)) {
          all_min_h = std::min(all_min_h, summary.min_h);
        }
        if (std::isfinite(summary.min_cbf)) {
          all_min_cbf = std::min(all_min_cbf, summary.min_cbf);
        }
        if (std::isfinite(summary.solver_min_h)) {
          if (!has_solver_min_h_witness ||
              summary.solver_min_h < all_solver_min_h) {
            all_solver_min_h_witness_h = summary.solver_min_h_witness_h;
            all_solver_min_h_witness_planar_clearance =
                summary.solver_min_h_witness_planar_clearance;
            all_solver_min_h_witness_tracking_error =
                summary.solver_min_h_witness_tracking_error;
            all_solver_min_h_witness_delta =
                summary.solver_min_h_witness_delta;
            all_solver_min_h_witness_planar_distance =
                summary.solver_min_h_witness_planar_distance;
            all_solver_min_h_witness_active_obstacle_key =
                summary.solver_min_h_witness_active_obstacle_key;
            has_solver_min_h_witness = true;
          }
          all_solver_min_h = std::min(all_solver_min_h, summary.solver_min_h);
        }
        if (std::isfinite(summary.solver_min_planar_clearance)) {
          all_solver_min_planar_clearance =
              std::min(all_solver_min_planar_clearance,
                       summary.solver_min_planar_clearance);
        }
        if (std::isfinite(summary.dmin_min)) {
          all_dmin_min = std::min(all_dmin_min, summary.dmin_min);
        }
        if (std::isfinite(summary.max_slack)) {
          all_max_slack = std::max(all_max_slack, summary.max_slack);
        }
        if (std::isfinite(summary.solver_mean_h) &&
            summary.solver_cbf_active_samples > 0) {
          sum_h += summary.solver_mean_h *
                   static_cast<double>(summary.solver_cbf_active_samples);
        }
        if (std::isfinite(summary.solver_mean_cbf) &&
            summary.solver_cbf_active_samples > 0) {
          sum_cbf += summary.solver_mean_cbf *
                     static_cast<double>(summary.solver_cbf_active_samples);
        }
        if (std::isfinite(summary.diag_min_h_any)) {
          all_diag_min_h_any = std::min(all_diag_min_h_any, summary.diag_min_h_any);
        }
        if (std::isfinite(summary.diag_min_cbf_any)) {
          all_diag_min_cbf_any =
              std::min(all_diag_min_cbf_any, summary.diag_min_cbf_any);
        }
        if (std::isfinite(summary.diag_mean_h_any)) {
          diag_sum_h_any +=
              summary.diag_mean_h_any * static_cast<double>(summary.samples);
        }
        if (std::isfinite(summary.diag_mean_cbf_any)) {
          diag_sum_cbf_any +=
              summary.diag_mean_cbf_any * static_cast<double>(summary.samples);
        }
        if (std::isfinite(summary.tracking_rms)) {
          const double steps = static_cast<double>(summary.samples);
          tracking_rms_sum_sq += summary.tracking_rms * summary.tracking_rms * steps;
          has_tracking_rms = true;
        }
        if (std::isfinite(summary.tracking_rms_x) &&
            std::isfinite(summary.tracking_rms_y) &&
            std::isfinite(summary.tracking_rms_z)) {
          const double steps = static_cast<double>(summary.samples);
          tracking_rms_x_sum_sq += summary.tracking_rms_x * summary.tracking_rms_x * steps;
          tracking_rms_y_sum_sq += summary.tracking_rms_y * summary.tracking_rms_y * steps;
          tracking_rms_z_sum_sq += summary.tracking_rms_z * summary.tracking_rms_z * steps;
          has_tracking_rms_xyz = true;
        }
        if (std::isfinite(summary.tracking_p95)) {
          tracking_p95_max = std::max(tracking_p95_max, summary.tracking_p95);
          has_tracking_p95 = true;
        }
        if (summary.cbf_active_samples > 0 &&
            std::isfinite(summary.tracking_rms_cbf_active)) {
          const double steps_cbf = static_cast<double>(summary.cbf_active_samples);
          tracking_rms_cbf_active_sum_sq +=
              summary.tracking_rms_cbf_active * summary.tracking_rms_cbf_active *
              steps_cbf;
          total_cbf_active_samples += summary.cbf_active_samples;
          has_tracking_rms_cbf_active = true;
        }
        if (summary.solver_tracking_rms_avoid_samples > 0 &&
            std::isfinite(summary.solver_tracking_rms_avoid)) {
          const double solver_steps =
              static_cast<double>(summary.solver_tracking_rms_avoid_samples);
          solver_tracking_rms_avoid_sum_sq +=
              summary.solver_tracking_rms_avoid *
              summary.solver_tracking_rms_avoid * solver_steps;
          total_solver_tracking_rms_avoid_samples +=
              summary.solver_tracking_rms_avoid_samples;
          has_solver_tracking_rms_avoid = true;
        }
        if (idx >= 0 && idx < static_cast<int>(control_system_ptr.size()) &&
            control_system_ptr[static_cast<size_t>(idx)]) {
          auto tracking_values =
              control_system_ptr[static_cast<size_t>(idx)]->getTrackingErrors();
          auto tracking_track_only_values =
              control_system_ptr[static_cast<size_t>(idx)]->getTrackingErrorsTrackOnly();
          auto solve_values =
              control_system_ptr[static_cast<size_t>(idx)]->getSolveTimeSeriesMs();
          auto feedback_values =
              control_system_ptr[static_cast<size_t>(idx)]->getFeedbackTimeSeriesMs();
          auto preparation_values =
              control_system_ptr[static_cast<size_t>(idx)]->getPreparationTimeSeriesMs();
          tracking_errors_all.insert(
              tracking_errors_all.end(), tracking_values.begin(), tracking_values.end());
          tracking_errors_track_only_all.insert(
              tracking_errors_track_only_all.end(),
              tracking_track_only_values.begin(),
              tracking_track_only_values.end());
          solve_time_all.insert(
              solve_time_all.end(), solve_values.begin(), solve_values.end());
          feedback_time_all.insert(
              feedback_time_all.end(), feedback_values.begin(), feedback_values.end());
          preparation_time_all.insert(
              preparation_time_all.end(), preparation_values.begin(),
              preparation_values.end());
        }
        if (std::isfinite(summary.slack_sum)) {
          slack_sum += summary.slack_sum;
        }
        if (summary.turn_samples > 0) {
          total_turn_samples += summary.turn_samples;
          if (std::isfinite(summary.tracking_rms_all_turn)) {
            tracking_rms_turn_sum_sq +=
                summary.tracking_rms_all_turn * summary.tracking_rms_all_turn *
                static_cast<double>(summary.turn_samples);
          }
          if (std::isfinite(summary.fail_rate_turn)) {
            total_failures_turn +=
                summary.fail_rate_turn * static_cast<double>(summary.turn_samples);
          }
          if (std::isfinite(summary.min_h_turn)) {
            all_min_h_turn = std::min(all_min_h_turn, summary.min_h_turn);
          }
          if (std::isfinite(summary.slack_sum_turn)) {
            slack_sum_turn += summary.slack_sum_turn;
          }
          if (std::isfinite(summary.max_slack_turn)) {
            all_max_slack_turn = std::max(all_max_slack_turn, summary.max_slack_turn);
          }
        }
        if (summary.straight_samples > 0) {
          total_straight_samples += summary.straight_samples;
          if (std::isfinite(summary.tracking_rms_all_straight)) {
            tracking_rms_straight_sum_sq +=
                summary.tracking_rms_all_straight *
                summary.tracking_rms_all_straight *
                static_cast<double>(summary.straight_samples);
          }
          if (std::isfinite(summary.fail_rate_straight)) {
            total_failures_straight +=
                summary.fail_rate_straight *
                static_cast<double>(summary.straight_samples);
          }
          if (std::isfinite(summary.min_h_straight)) {
            all_min_h_straight =
                std::min(all_min_h_straight, summary.min_h_straight);
          }
          if (std::isfinite(summary.slack_sum_straight)) {
            slack_sum_straight += summary.slack_sum_straight;
          }
          if (std::isfinite(summary.max_slack_straight)) {
            all_max_slack_straight =
                std::max(all_max_slack_straight, summary.max_slack_straight);
          }
        }
        if (summary.high_c_samples > 0) {
          total_high_c_samples += summary.high_c_samples;
          if (std::isfinite(summary.tracking_rms_high_c)) {
            tracking_rms_high_c_sum_sq +=
                summary.tracking_rms_high_c * summary.tracking_rms_high_c *
                static_cast<double>(summary.high_c_samples);
          }
          if (std::isfinite(summary.fail_rate_high_c)) {
            total_failures_high_c +=
                summary.fail_rate_high_c *
                static_cast<double>(summary.high_c_samples);
          }
          if (std::isfinite(summary.min_h_high_c)) {
            all_min_h_high_c = std::min(all_min_h_high_c, summary.min_h_high_c);
          }
          if (std::isfinite(summary.slack_sum_high_c)) {
            slack_sum_high_c += summary.slack_sum_high_c;
          }
        }
        if (summary.high_c44_samples > 0 &&
            std::isfinite(summary.tracking_rms_high_c44)) {
          total_high_c44_samples += summary.high_c44_samples;
          tracking_rms_high_c44_sum_sq +=
              summary.tracking_rms_high_c44 * summary.tracking_rms_high_c44 *
              static_cast<double>(summary.high_c44_samples);
        }
        if (summary.high_c88_samples > 0 &&
            std::isfinite(summary.tracking_rms_high_c88)) {
          total_high_c88_samples += summary.high_c88_samples;
          tracking_rms_high_c88_sum_sq +=
              summary.tracking_rms_high_c88 * summary.tracking_rms_high_c88 *
              static_cast<double>(summary.high_c88_samples);
        }
        total_solver_track_only_samples += summary.solver_track_only_samples;
        if (summary.track_only_samples > 0 &&
            std::isfinite(summary.tracking_rms_track_only_x) &&
            std::isfinite(summary.tracking_rms_track_only_y) &&
            std::isfinite(summary.tracking_rms_track_only_z)) {
          const double steps_track_only = static_cast<double>(summary.track_only_samples);
          tracking_rms_track_only_x_sum_sq +=
              summary.tracking_rms_track_only_x * summary.tracking_rms_track_only_x *
              steps_track_only;
          tracking_rms_track_only_y_sum_sq +=
              summary.tracking_rms_track_only_y * summary.tracking_rms_track_only_y *
              steps_track_only;
          tracking_rms_track_only_z_sum_sq +=
              summary.tracking_rms_track_only_z * summary.tracking_rms_track_only_z *
              steps_track_only;
          has_tracking_rms_track_only_xyz = true;
        }
        if (std::isfinite(summary.solve_time_mean_ms)) {
          solve_time_mean_sum += summary.solve_time_mean_ms * static_cast<double>(summary.samples);
          has_solve_mean = true;
        }
        if (std::isfinite(summary.solve_time_p95_ms)) {
          solve_time_p95_max = std::max(solve_time_p95_max, summary.solve_time_p95_ms);
          has_solve_p95 = true;
        }
        if (std::isfinite(summary.solve_time_max_ms)) {
          solve_time_max = std::max(solve_time_max, summary.solve_time_max_ms);
          has_solve_max = true;
        }
        if (std::isfinite(summary.feedback_time_mean_ms)) {
          feedback_time_mean_sum +=
              summary.feedback_time_mean_ms * static_cast<double>(summary.samples);
          has_feedback_mean = true;
        }
        if (std::isfinite(summary.feedback_time_p95_ms)) {
          feedback_time_p95_max =
              std::max(feedback_time_p95_max, summary.feedback_time_p95_ms);
          has_feedback_p95 = true;
        }
        if (std::isfinite(summary.feedback_time_max_ms)) {
          feedback_time_max =
              std::max(feedback_time_max, summary.feedback_time_max_ms);
          has_feedback_max = true;
        }
        if (std::isfinite(summary.preparation_time_mean_ms)) {
          preparation_time_mean_sum +=
              summary.preparation_time_mean_ms * static_cast<double>(summary.samples);
          has_preparation_mean = true;
        }
        if (std::isfinite(summary.preparation_time_p95_ms)) {
          preparation_time_p95_max = std::max(
              preparation_time_p95_max, summary.preparation_time_p95_ms);
          has_preparation_p95 = true;
        }
        if (std::isfinite(summary.preparation_time_max_ms)) {
          preparation_time_max = std::max(
              preparation_time_max, summary.preparation_time_max_ms);
          has_preparation_max = true;
        }
        if (std::isfinite(summary.fail_rate)) {
          total_failures += summary.fail_rate * static_cast<double>(summary.samples);
        }
      }
      has_tracking_p95 = has_tracking_p95 && !tracking_errors_all.empty();
      has_solve_p95 = has_solve_p95 && !solve_time_all.empty();
      has_feedback_p95 = has_feedback_p95 && !feedback_time_all.empty();
      has_preparation_p95 =
          has_preparation_p95 && !preparation_time_all.empty();

      for (int idx : active_uav_indices) {
        if (idx < 0 || idx >= static_cast<int>(summaries.size())) {
          continue;
        }
        writeMetricsRow(out,
                        run_tag_value,
                        "uav" + std::to_string(idx),
                        frame_mode_effective,
                        ugv_rollout_mode,
                        safety_variant,
                        obstacle_cfg.count,
                        obstacle_cfg.seed,
                        seed_env,
                        seed_traj,
                        traj_id,
                        kappa,
                        requested_r,
                        requested_v,
                        requested_w,
                        use_dynamic_obs,
                        warm_start,
                        start_z,
                        noise_seed,
                        active_uavs_csv,
                        scheduler_dt_sec,
                        prediction_dt_sec,
                        prediction_steps,
                        max_abs_ugv_yaw_rate,
                        turning_omega_thr,
                        metrics_debug,
                        actual_sim_time_sec,
                        wall_time_sec,
                        collision_reported.load(),
                        summaries[static_cast<size_t>(idx)],
                        summaries[static_cast<size_t>(idx)].tracking_p95,
                        summaries[static_cast<size_t>(idx)].solve_time_p95_ms);
      }

      coni_mpc::NumSimMpc::MetricsSummary all_summary;
      all_summary.samples = total_samples;
      if (total_samples == 0) {
        all_summary.min_h = std::numeric_limits<double>::quiet_NaN();
        all_summary.min_cbf = std::numeric_limits<double>::quiet_NaN();
        all_summary.max_slack = std::numeric_limits<double>::quiet_NaN();
        all_summary.mean_h = std::numeric_limits<double>::quiet_NaN();
        all_summary.mean_cbf = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_x = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_y = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_z = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_p95 = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_track_only = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_track_only_x = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_track_only_y = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_track_only_z = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_p95_track_only = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_cbf_active = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_tracking_rms_avoid = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_tracking_rms_avoid_samples = 0;
        all_summary.fail_rate_turn = std::numeric_limits<double>::quiet_NaN();
        all_summary.fail_rate_straight = std::numeric_limits<double>::quiet_NaN();
        all_summary.min_h_turn = std::numeric_limits<double>::quiet_NaN();
        all_summary.min_h_straight = std::numeric_limits<double>::quiet_NaN();
        all_summary.slack_sum_turn = std::numeric_limits<double>::quiet_NaN();
        all_summary.slack_sum_straight = std::numeric_limits<double>::quiet_NaN();
        all_summary.max_slack_turn = std::numeric_limits<double>::quiet_NaN();
        all_summary.max_slack_straight = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_all_turn = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_all_straight = std::numeric_limits<double>::quiet_NaN();
        all_summary.c44_metric = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_high_c44 = std::numeric_limits<double>::quiet_NaN();
        all_summary.c66_metric = std::numeric_limits<double>::quiet_NaN();
        all_summary.c88_metric = std::numeric_limits<double>::quiet_NaN();
        all_summary.min_h_high_c = std::numeric_limits<double>::quiet_NaN();
        all_summary.slack_sum_high_c = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_high_c = std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_high_c88 = std::numeric_limits<double>::quiet_NaN();
        all_summary.fail_rate_high_c = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_min_h = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_active_mean_h = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_min_planar_clearance =
            std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_min_h_witness_h =
            std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_min_h_witness_planar_clearance =
            std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_min_h_witness_tracking_error =
            std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_min_h_witness_delta =
            std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_min_h_witness_planar_distance =
            std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_min_h_witness_active_obstacle_key = "none";
        all_summary.solver_min_cbf = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_mean_h = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_mean_cbf = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_slack_sum = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_max_slack = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_fail_rate = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_fail_rate_turn = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_fail_rate_straight = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_min_h_turn = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_min_h_straight = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_slack_sum_turn = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_slack_sum_straight = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_max_slack_turn = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_max_slack_straight = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_min_h_high_c = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_slack_sum_high_c = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_fail_rate_high_c = std::numeric_limits<double>::quiet_NaN();
        all_summary.diag_min_h_any = std::numeric_limits<double>::quiet_NaN();
        all_summary.diag_mean_h_any = std::numeric_limits<double>::quiet_NaN();
        all_summary.diag_min_cbf_any = std::numeric_limits<double>::quiet_NaN();
        all_summary.diag_mean_cbf_any = std::numeric_limits<double>::quiet_NaN();
        all_summary.dmin_min = std::numeric_limits<double>::quiet_NaN();
        all_summary.slack_sum = std::numeric_limits<double>::quiet_NaN();
        all_summary.solve_time_mean_ms = std::numeric_limits<double>::quiet_NaN();
        all_summary.solve_time_p95_ms = std::numeric_limits<double>::quiet_NaN();
        all_summary.solve_time_max_ms = std::numeric_limits<double>::quiet_NaN();
        all_summary.feedback_time_mean_ms = std::numeric_limits<double>::quiet_NaN();
        all_summary.feedback_time_p95_ms = std::numeric_limits<double>::quiet_NaN();
        all_summary.feedback_time_max_ms = std::numeric_limits<double>::quiet_NaN();
        all_summary.preparation_time_mean_ms = std::numeric_limits<double>::quiet_NaN();
        all_summary.preparation_time_p95_ms = std::numeric_limits<double>::quiet_NaN();
        all_summary.preparation_time_max_ms = std::numeric_limits<double>::quiet_NaN();
        all_summary.fail_rate = std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_cbf_active_samples = 0;
        all_summary.solver_track_only_samples = 0;
        all_summary.cbf_active_samples = 0;
        all_summary.high_c44_samples = 0;
        all_summary.high_c_samples = 0;
        all_summary.high_c88_samples = 0;
        all_summary.track_only_samples = 0;
        all_summary.turn_samples = 0;
        all_summary.straight_samples = 0;
      } else {
        all_summary.min_h = std::isfinite(all_min_h)
                                ? all_min_h
                                : std::numeric_limits<double>::quiet_NaN();
        all_summary.min_cbf = std::isfinite(all_min_cbf)
                                  ? all_min_cbf
                                  : std::numeric_limits<double>::quiet_NaN();
        all_summary.dmin_min = std::isfinite(all_dmin_min)
                                   ? all_dmin_min
                                   : std::numeric_limits<double>::quiet_NaN();
        all_summary.max_slack = all_max_slack;
        all_summary.mean_h =
            (std::isfinite(all_min_h) && total_solver_metric_samples > 0)
                ? (sum_h / static_cast<double>(total_solver_metric_samples))
                : std::numeric_limits<double>::quiet_NaN();
        all_summary.mean_cbf =
            (std::isfinite(all_min_cbf) && total_solver_metric_samples > 0)
                ? (sum_cbf / static_cast<double>(total_solver_metric_samples))
                : std::numeric_limits<double>::quiet_NaN();
        all_summary.diag_min_h_any = std::isfinite(all_diag_min_h_any)
                                         ? all_diag_min_h_any
                                         : std::numeric_limits<double>::quiet_NaN();
        all_summary.diag_min_cbf_any = std::isfinite(all_diag_min_cbf_any)
                                           ? all_diag_min_cbf_any
                                           : std::numeric_limits<double>::quiet_NaN();
        all_summary.diag_mean_h_any =
            std::isfinite(all_diag_min_h_any)
                ? (diag_sum_h_any / static_cast<double>(total_samples))
                : std::numeric_limits<double>::quiet_NaN();
        all_summary.diag_mean_cbf_any =
            std::isfinite(all_diag_min_cbf_any)
                ? (diag_sum_cbf_any / static_cast<double>(total_samples))
                : std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms = has_tracking_rms
                                       ? std::sqrt(tracking_rms_sum_sq /
                                                   static_cast<double>(total_samples))
                                       : std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_x =
            has_tracking_rms_xyz
                ? std::sqrt(tracking_rms_x_sum_sq /
                            static_cast<double>(total_samples))
                : std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_y =
            has_tracking_rms_xyz
                ? std::sqrt(tracking_rms_y_sum_sq /
                            static_cast<double>(total_samples))
                : std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_rms_z =
            has_tracking_rms_xyz
                ? std::sqrt(tracking_rms_z_sum_sq /
                            static_cast<double>(total_samples))
                : std::numeric_limits<double>::quiet_NaN();
        all_summary.tracking_p95 = has_tracking_p95
                                       ? computeP95FromValues(tracking_errors_all)
                                       : std::numeric_limits<double>::quiet_NaN();
        all_summary.cbf_active_samples = total_cbf_active_samples;
        all_summary.solver_cbf_active_samples = total_solver_metric_samples;
        all_summary.solver_tracking_rms_avoid_samples =
            total_solver_tracking_rms_avoid_samples;
        all_summary.tracking_rms_cbf_active =
            (has_tracking_rms_cbf_active && total_cbf_active_samples > 0)
                ? std::sqrt(tracking_rms_cbf_active_sum_sq /
                            static_cast<double>(total_cbf_active_samples))
                : std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_tracking_rms_avoid =
            (has_solver_tracking_rms_avoid &&
             total_solver_tracking_rms_avoid_samples > 0)
                ? std::sqrt(solver_tracking_rms_avoid_sum_sq /
                            static_cast<double>(
                                total_solver_tracking_rms_avoid_samples))
                : std::numeric_limits<double>::quiet_NaN();
        all_summary.track_only_samples = total_solver_track_only_samples;
        all_summary.solver_track_only_samples = total_solver_track_only_samples;
        all_summary.tracking_rms_track_only =
            tracking_errors_track_only_all.empty()
                ? std::numeric_limits<double>::quiet_NaN()
                : std::sqrt(std::inner_product(
                    tracking_errors_track_only_all.begin(),
                    tracking_errors_track_only_all.end(),
                    tracking_errors_track_only_all.begin(), 0.0) /
                            static_cast<double>(tracking_errors_track_only_all.size()));
        all_summary.tracking_p95_track_only =
            tracking_errors_track_only_all.empty()
                ? std::numeric_limits<double>::quiet_NaN()
                : computeP95FromValues(tracking_errors_track_only_all);
        if (all_summary.track_only_samples > 0 && has_tracking_rms_track_only_xyz) {
          const double steps_track_only =
              static_cast<double>(all_summary.track_only_samples);
          all_summary.tracking_rms_track_only_x = std::sqrt(
              tracking_rms_track_only_x_sum_sq / steps_track_only);
          all_summary.tracking_rms_track_only_y = std::sqrt(
              tracking_rms_track_only_y_sum_sq / steps_track_only);
          all_summary.tracking_rms_track_only_z = std::sqrt(
              tracking_rms_track_only_z_sum_sq / steps_track_only);
        } else {
          all_summary.tracking_rms_track_only_x =
              std::numeric_limits<double>::quiet_NaN();
          all_summary.tracking_rms_track_only_y =
              std::numeric_limits<double>::quiet_NaN();
          all_summary.tracking_rms_track_only_z =
              std::numeric_limits<double>::quiet_NaN();
        }
        all_summary.slack_sum = slack_sum;
        all_summary.solver_min_h =
            std::isfinite(all_solver_min_h)
                ? all_solver_min_h
                : std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_active_mean_h =
            (total_solver_metric_samples > 0)
                ? (sum_h / static_cast<double>(total_solver_metric_samples))
                : std::numeric_limits<double>::quiet_NaN();
        all_summary.solver_min_planar_clearance =
            std::isfinite(all_solver_min_planar_clearance)
                ? all_solver_min_planar_clearance
                : std::numeric_limits<double>::quiet_NaN();
        if (has_solver_min_h_witness &&
            std::isfinite(all_summary.solver_min_h)) {
          all_summary.solver_min_h_witness_h = all_solver_min_h_witness_h;
          all_summary.solver_min_h_witness_planar_clearance =
              all_solver_min_h_witness_planar_clearance;
          all_summary.solver_min_h_witness_tracking_error =
              all_solver_min_h_witness_tracking_error;
          all_summary.solver_min_h_witness_delta =
              all_solver_min_h_witness_delta;
          all_summary.solver_min_h_witness_planar_distance =
              all_solver_min_h_witness_planar_distance;
          all_summary.solver_min_h_witness_active_obstacle_key =
              all_solver_min_h_witness_active_obstacle_key;
        } else {
          all_summary.solver_min_h_witness_h =
              std::numeric_limits<double>::quiet_NaN();
          all_summary.solver_min_h_witness_planar_clearance =
              std::numeric_limits<double>::quiet_NaN();
          all_summary.solver_min_h_witness_tracking_error =
              std::numeric_limits<double>::quiet_NaN();
          all_summary.solver_min_h_witness_delta =
              std::numeric_limits<double>::quiet_NaN();
          all_summary.solver_min_h_witness_planar_distance =
              std::numeric_limits<double>::quiet_NaN();
          all_summary.solver_min_h_witness_active_obstacle_key = "none";
        }
        all_summary.solver_min_cbf = all_summary.min_cbf;
        all_summary.solver_mean_h = all_summary.mean_h;
        all_summary.solver_mean_cbf = all_summary.mean_cbf;
        all_summary.solver_slack_sum = slack_sum;
        all_summary.solver_max_slack = all_max_slack;
        all_summary.turn_samples = total_turn_samples;
        all_summary.straight_samples = total_straight_samples;
        all_summary.fail_rate_turn =
            (total_turn_samples == 0)
                ? std::numeric_limits<double>::quiet_NaN()
                : (total_failures_turn /
                   static_cast<double>(total_turn_samples));
        all_summary.fail_rate_straight =
            (total_straight_samples == 0)
                ? std::numeric_limits<double>::quiet_NaN()
                : (total_failures_straight /
                   static_cast<double>(total_straight_samples));
        all_summary.min_h_turn = std::isfinite(all_min_h_turn)
                                     ? all_min_h_turn
                                     : std::numeric_limits<double>::quiet_NaN();
        all_summary.min_h_straight = std::isfinite(all_min_h_straight)
                                         ? all_min_h_straight
                                         : std::numeric_limits<double>::quiet_NaN();
        all_summary.slack_sum_turn =
            (total_turn_samples == 0)
                ? std::numeric_limits<double>::quiet_NaN()
                : slack_sum_turn;
        all_summary.slack_sum_straight =
            (total_straight_samples == 0)
                ? std::numeric_limits<double>::quiet_NaN()
                : slack_sum_straight;
        all_summary.max_slack_turn =
            (total_turn_samples == 0)
                ? std::numeric_limits<double>::quiet_NaN()
                : all_max_slack_turn;
        all_summary.max_slack_straight =
            (total_straight_samples == 0)
                ? std::numeric_limits<double>::quiet_NaN()
                : all_max_slack_straight;
        all_summary.solver_fail_rate_turn = all_summary.fail_rate_turn;
        all_summary.solver_fail_rate_straight = all_summary.fail_rate_straight;
        all_summary.solver_min_h_turn = all_summary.min_h_turn;
        all_summary.solver_min_h_straight = all_summary.min_h_straight;
        all_summary.solver_slack_sum_turn = all_summary.slack_sum_turn;
        all_summary.solver_slack_sum_straight = all_summary.slack_sum_straight;
        all_summary.solver_max_slack_turn = all_summary.max_slack_turn;
        all_summary.solver_max_slack_straight = all_summary.max_slack_straight;
        all_summary.tracking_rms_all_turn =
            (total_turn_samples == 0)
                ? std::numeric_limits<double>::quiet_NaN()
                : std::sqrt(tracking_rms_turn_sum_sq /
                            static_cast<double>(total_turn_samples));
        all_summary.tracking_rms_all_straight =
            (total_straight_samples == 0)
                ? std::numeric_limits<double>::quiet_NaN()
                : std::sqrt(tracking_rms_straight_sum_sq /
                            static_cast<double>(total_straight_samples));
        all_summary.c44_metric = std::numeric_limits<double>::quiet_NaN();
        all_summary.high_c44_samples = total_high_c44_samples;
        all_summary.tracking_rms_high_c44 =
            (total_high_c44_samples == 0)
                ? std::numeric_limits<double>::quiet_NaN()
                : std::sqrt(tracking_rms_high_c44_sum_sq /
                            static_cast<double>(total_high_c44_samples));
        all_summary.c66_metric = std::numeric_limits<double>::quiet_NaN();
        all_summary.high_c_samples = total_high_c_samples;
        all_summary.min_h_high_c =
            (total_high_c_samples == 0 || !std::isfinite(all_min_h_high_c))
                ? std::numeric_limits<double>::quiet_NaN()
                : all_min_h_high_c;
        all_summary.slack_sum_high_c =
            (total_high_c_samples == 0)
                ? std::numeric_limits<double>::quiet_NaN()
                : slack_sum_high_c;
        all_summary.tracking_rms_high_c =
            (total_high_c_samples == 0)
                ? std::numeric_limits<double>::quiet_NaN()
                : std::sqrt(tracking_rms_high_c_sum_sq /
                            static_cast<double>(total_high_c_samples));
        all_summary.fail_rate_high_c =
            (total_high_c_samples == 0)
                ? std::numeric_limits<double>::quiet_NaN()
                : (total_failures_high_c /
                   static_cast<double>(total_high_c_samples));
        all_summary.solver_min_h_high_c = all_summary.min_h_high_c;
        all_summary.solver_slack_sum_high_c = all_summary.slack_sum_high_c;
        all_summary.solver_fail_rate_high_c = all_summary.fail_rate_high_c;
        all_summary.c88_metric = std::numeric_limits<double>::quiet_NaN();
        all_summary.high_c88_samples = total_high_c88_samples;
        all_summary.tracking_rms_high_c88 =
            (total_high_c88_samples == 0)
                ? std::numeric_limits<double>::quiet_NaN()
                : std::sqrt(tracking_rms_high_c88_sum_sq /
                            static_cast<double>(total_high_c88_samples));
        all_summary.solve_time_mean_ms = has_solve_mean
                                             ? (solve_time_mean_sum /
                                                static_cast<double>(total_samples))
                                             : std::numeric_limits<double>::quiet_NaN();
        all_summary.solve_time_p95_ms = has_solve_p95
                                            ? computeP95FromValues(solve_time_all)
                                            : std::numeric_limits<double>::quiet_NaN();
        all_summary.solve_time_max_ms = has_solve_max
                                            ? solve_time_max
                                            : std::numeric_limits<double>::quiet_NaN();
        all_summary.feedback_time_mean_ms = has_feedback_mean
                                                ? (feedback_time_mean_sum /
                                                   static_cast<double>(total_samples))
                                                : std::numeric_limits<double>::quiet_NaN();
        all_summary.feedback_time_p95_ms = has_feedback_p95
                                               ? computeP95FromValues(feedback_time_all)
                                               : std::numeric_limits<double>::quiet_NaN();
        all_summary.feedback_time_max_ms = has_feedback_max
                                               ? feedback_time_max
                                               : std::numeric_limits<double>::quiet_NaN();
        all_summary.preparation_time_mean_ms =
            has_preparation_mean
                ? (preparation_time_mean_sum /
                   static_cast<double>(total_samples))
                : std::numeric_limits<double>::quiet_NaN();
        all_summary.preparation_time_p95_ms =
            has_preparation_p95
                ? computeP95FromValues(preparation_time_all)
                : std::numeric_limits<double>::quiet_NaN();
        all_summary.preparation_time_max_ms =
            has_preparation_max
                ? preparation_time_max
                : std::numeric_limits<double>::quiet_NaN();
        all_summary.fail_rate =
            total_failures / static_cast<double>(total_samples);
        all_summary.solver_fail_rate = all_summary.fail_rate;
      }
      writeMetricsRow(out,
                      run_tag_value,
                      "all",
                      frame_mode_effective,
                      ugv_rollout_mode,
                      safety_variant,
                      obstacle_cfg.count,
                      obstacle_cfg.seed,
                      seed_env,
                      seed_traj,
                      traj_id,
                      kappa,
                      requested_r,
                      requested_v,
                      requested_w,
                      use_dynamic_obs,
                      warm_start,
                      start_z,
                      noise_seed,
                      aggregate_uavs_csv,
                      scheduler_dt_sec,
                      prediction_dt_sec,
                      prediction_steps,
                      max_abs_ugv_yaw_rate,
                      turning_omega_thr,
                      metrics_debug,
                      actual_sim_time_sec,
                      wall_time_sec,
                      aggregate_collision_reported.load(),
                      all_summary,
                      has_tracking_p95 ? tracking_p95_max
                                       : std::numeric_limits<double>::quiet_NaN(),
                      has_solve_p95 ? solve_time_p95_max
                                    : std::numeric_limits<double>::quiet_NaN());
      ROS_INFO_STREAM("Clean solver metrics summary: run_tag=" << run_tag_value
                      << " scope=all"
                      << " aggregate_uavs_csv=" << aggregate_uavs_csv
                      << " solver_tracking_rms_avoid="
                      << fmtDouble(all_summary.solver_tracking_rms_avoid)
                      << " solver_tracking_rms_avoid_samples="
                      << all_summary.solver_tracking_rms_avoid_samples
                      << " solver_min_h="
                      << fmtDouble(all_summary.solver_min_h)
                      << " solver_active_mean_h="
                      << fmtDouble(all_summary.solver_active_mean_h)
                      << " solver_min_planar_clearance="
                      << fmtDouble(all_summary.solver_min_planar_clearance)
                      << " witness_h="
                      << fmtDouble(all_summary.solver_min_h_witness_h)
                      << " witness_planar_clearance="
                      << fmtDouble(
                             all_summary.solver_min_h_witness_planar_clearance)
                      << " witness_e_track="
                      << fmtDouble(
                             all_summary.solver_min_h_witness_tracking_error)
                      << " witness_delta="
                      << fmtDouble(all_summary.solver_min_h_witness_delta)
                      << " witness_planar_distance="
                      << fmtDouble(
                             all_summary.solver_min_h_witness_planar_distance)
                      << " witness_solver_active_obstacle_key="
                      << all_summary.solver_min_h_witness_active_obstacle_key);
    }
  } else {
    ROS_WARN("metrics_csv not set; skipping CSV output.");
  }

  if (collision_reported.load() && post_collision_hold_sec > 1e-6) {
    if (collision_marker_published.load()) {
      ROS_WARN_STREAM("Collision detected; keeping node alive for "
                      << post_collision_hold_sec
                      << " s so RViz can display collision markers.");
    } else {
      ROS_WARN_STREAM("Collision detected; keeping node alive for "
                      << post_collision_hold_sec
                      << " s before shutdown.");
    }
    const ros::WallTime hold_start = ros::WallTime::now();
    while (ros::ok() &&
           (ros::WallTime::now() - hold_start).toSec() < post_collision_hold_sec) {
      ros::spinOnce();
      ros::Duration(0.05).sleep();
    }
  }

  return 0; 
}  


/* try {
std::thread threa0([&]() { simulators[0]->simulate(DURATION, 0); });

if (threa0.joinable())
          threa0.join(); } catch (const std::exception& e) {
                    ROS_ERROR("Exception in simulator %d: %s", 0, e.what());
                }
 */

/* try {
std::thread threa1([&]() { simulators[1]->simulate(DURATION, 0); });

if (threa1.joinable())
          threa1.join(); } catch (const std::exception& e) {
                    ROS_ERROR("Exception in simulator %d: %s", 1, e.what());
                } */
/* std::thread threa1([&]() { simulators[1]->simulate(DURATION, 1); });
std::thread threa2([&]() { simulators[2]->simulate(DURATION, 2); });
std::thread threa3([&]() { simulators[3]->simulate(DURATION, 3); }); */



    /*    std::vector<std::thread> simulator_threads;
    
   // 同时启动所有模拟器
    for (int i = 0; i < 4; ++i) {
        simulator_threads.emplace_back([i, this] {
            simulators[i]->simulate(DURATION, i);
        });
    }
    
    // 等待所有模拟器完成
    for (auto& thread : simulator_threads) {
        if (thread.joinable()) thread.join();
    } */

  







/*   QuadrotorSimulator simulator(0.01, 0.01, x0[0], quad_system_ptr[0], control_system_ptr[0], trajectory[0].points, 
      plan, car_trajectory, car_plan, 
      1.0 ,
      10, 
      DURATION, 10000);
  simulator.simulate(DURATION,0);
  simulator.finish(); */
