/*
 * @Author: Baozhe ZHANG 
 * @Date: 2023-05-22 10:00:03 
 * @Last Modified by: Baozhe ZHANG
 * @Last Modified time: 2023-05-22 12:40:49
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

#ifndef CONI_MPC_NUM_SIM_MPC_H
#define CONI_MPC_NUM_SIM_MPC_H

#include "coni_mpc/mpc_base.h"
#include "coni_mpc/obstacle_sensing_tracker.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <random>
#include <string>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/StdVector>
#include <visualization_msgs/Marker.h>
extern int num_uavs;
namespace coni_mpc
{
    
class NumSimMpc final : public MpcBase 
{

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class FrameMode {
    kNonInertial = 0,
    kInertial = 1,
  };

  using CarState_t = Eigen::Matrix<double, 16, 1>;
  using CarTrajectoryWindow = std::vector<CarState_t>;
  using Obstacle = ObstacleSensingTracker::Obstacle;
  using ObstacleVector = ObstacleSensingTracker::ObstacleVector;
  using ObstacleProfile = acado_mpc::MpcWrapper<double>::ObstacleProfile;
  using ObstacleProfileVector = acado_mpc::MpcWrapper<double>::ObstacleProfileVector;
  using NonInertialProfile = Eigen::Matrix<double, 3, acado_mpc::kSamples + 1>;

  struct MetricsSummary {
    double min_h;
    double min_cbf;
    double max_slack;
    double mean_h;
    double mean_cbf;
    double tracking_rms;
    double tracking_rms_x;
    double tracking_rms_y;
    double tracking_rms_z;
    double tracking_p95;
    double tracking_rms_track_only;
    double tracking_rms_track_only_x;
    double tracking_rms_track_only_y;
    double tracking_rms_track_only_z;
    double tracking_p95_track_only;
    double tracking_rms_cbf_active;
    double solver_tracking_rms_avoid;
    std::size_t solver_tracking_rms_avoid_samples;
    double fail_rate_turn;
    double fail_rate_straight;
    double min_h_turn;
    double min_h_straight;
    double slack_sum_turn;
    double slack_sum_straight;
    double max_slack_turn;
    double max_slack_straight;
    double tracking_rms_all_turn;
    double tracking_rms_all_straight;
    double c44_metric;
    double tracking_rms_high_c44;
    double c66_metric;
    double c88_metric;
    double min_h_high_c;
    double slack_sum_high_c;
    double tracking_rms_high_c;
    double tracking_rms_high_c88;
    double fail_rate_high_c;
    double solver_min_h;
    double solver_active_mean_h;
    double solver_min_planar_clearance;
    double solver_min_h_witness_h;
    double solver_min_h_witness_planar_clearance;
    double solver_min_h_witness_tracking_error;
    double solver_min_h_witness_delta;
    double solver_min_h_witness_planar_distance;
    std::string solver_min_h_witness_active_obstacle_key;
    double solver_min_cbf;
    double solver_mean_h;
    double solver_mean_cbf;
    double solver_slack_sum;
    double solver_max_slack;
    double solver_fail_rate;
    double solver_fail_rate_turn;
    double solver_fail_rate_straight;
    double solver_min_h_turn;
    double solver_min_h_straight;
    double solver_slack_sum_turn;
    double solver_slack_sum_straight;
    double solver_max_slack_turn;
    double solver_max_slack_straight;
    double solver_min_h_high_c;
    double solver_slack_sum_high_c;
    double solver_fail_rate_high_c;
    double diag_min_h_any;
    double diag_mean_h_any;
    double diag_min_cbf_any;
    double diag_mean_cbf_any;
    double dmin_min;
    double truth_static_min_surface_clearance;
    double slack_sum;
    double solve_time_mean_ms;
    double solve_time_p95_ms;
    double solve_time_max_ms;
    double feedback_time_mean_ms;
    double feedback_time_p95_ms;
    double feedback_time_max_ms;
    double preparation_time_mean_ms;
    double preparation_time_p95_ms;
    double preparation_time_max_ms;
    double fail_rate;
    std::size_t solver_cbf_active_samples;
    std::size_t solver_track_only_samples;
    std::size_t cbf_active_samples;
    std::size_t high_c44_samples;
    std::size_t high_c_samples;
    std::size_t high_c88_samples;
    std::size_t track_only_samples;
    std::size_t turn_samples;
    std::size_t straight_samples;
    std::size_t samples;
  };

  // Ground-truth-referenced sensing diagnostics are logged only for
  // evaluation.  They never enter the controller or obstacle tracker.
  struct SensingSafetySnapshot {
    double clearance_threshold_m = 0.215;
    int nearest_truth_obstacle_id = -1;
    double nearest_truth_clearance_m =
        std::numeric_limits<double>::quiet_NaN();
    bool nearest_truth_in_sensor_fov = false;
    bool nearest_truth_current_dropout = false;
    bool nearest_truth_pending_measurement = false;
    bool nearest_truth_track_active = false;
    std::int64_t nearest_truth_track_age_frames = -1;
    double nearest_truth_track_age_sec =
        std::numeric_limits<double>::quiet_NaN();
    std::int64_t nearest_truth_cycles_since_delivery = -1;
    double nearest_truth_sec_since_delivery =
        std::numeric_limits<double>::quiet_NaN();
    std::int64_t nearest_truth_imposed_delay_frames = -1;
    double nearest_truth_imposed_delay_sec =
        std::numeric_limits<double>::quiet_NaN();
    double nearest_truth_last_measurement_error_m =
        std::numeric_limits<double>::quiet_NaN();
    double nearest_truth_track_center_error_m =
        std::numeric_limits<double>::quiet_NaN();
    double nearest_truth_track_covariance_trace =
        std::numeric_limits<double>::quiet_NaN();
    double nearest_truth_track_innovation_norm_m =
        std::numeric_limits<double>::quiet_NaN();
    std::int64_t nearest_truth_track_instance = -1;
    bool nearest_truth_track_reinitialized_this_cycle = false;
    // These are per-cycle indicators.  The offline aggregator counts each
    // contiguous run as one episode.  "Current dropout" records temporal
    // co-occurrence and is not, by itself, a claim of causation.
    bool low_clearance_sample = false;
    bool low_clearance_no_track_sample = false;
    bool low_clearance_current_dropout_sample = false;
    bool low_clearance_pending_no_track_sample = false;
    bool low_clearance_stale_track_sample = false;
  };

  // One evaluation-only row per active static-obstacle track.  Truth fields
  // are used only for simulation diagnostics and never feed the controller.
  struct SensingTrackDiagnostic {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool valid = false;
    bool tracker_online = false;
    std::size_t obstacle_id = 0;
    std::string obstacle_key = "none";
    bool selected_for_hocbf = false;
    bool solver_active = false;
    Eigen::Vector2d truth_position_world =
        Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector2d last_measurement_position_world =
        Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector2d track_position_world =
        Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    double truth_z = std::numeric_limits<double>::quiet_NaN();
    double track_z = std::numeric_limits<double>::quiet_NaN();
    double truth_radius = std::numeric_limits<double>::quiet_NaN();
    double track_radius = std::numeric_limits<double>::quiet_NaN();
    double last_measurement_error_m =
        std::numeric_limits<double>::quiet_NaN();
    double track_center_error_m =
        std::numeric_limits<double>::quiet_NaN();
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    double covariance_trace = std::numeric_limits<double>::quiet_NaN();
    double covariance_max_eigenvalue =
        std::numeric_limits<double>::quiet_NaN();
    bool innovation_valid = false;
    Eigen::Vector2d innovation =
        Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    double innovation_norm_m = std::numeric_limits<double>::quiet_NaN();
    double innovation_covariance_trace =
        std::numeric_limits<double>::quiet_NaN();
    double kalman_gain_trace = std::numeric_limits<double>::quiet_NaN();
    std::int64_t generation_scan_index = -1;
    std::int64_t generation_cycle = -1;
    std::int64_t delivery_cycle = -1;
    std::int64_t update_cycle = -1;
    std::int64_t measurement_age_frames = -1;
    double measurement_age_sec = std::numeric_limits<double>::quiet_NaN();
    std::int64_t cycles_since_delivery = -1;
    double sec_since_delivery = std::numeric_limits<double>::quiet_NaN();
    std::int64_t imposed_delay_frames = -1;
    double imposed_delay_sec = std::numeric_limits<double>::quiet_NaN();
    std::uint64_t track_instance = 0;
    std::uint64_t measurement_update_count = 0;
    bool created_this_cycle = false;
    bool updated_this_cycle = false;
    bool reinitialized_this_cycle = false;
    double truth_surface_clearance_m =
        std::numeric_limits<double>::quiet_NaN();
    double track_surface_clearance_m =
        std::numeric_limits<double>::quiet_NaN();
    double track_surface_error_m =
        std::numeric_limits<double>::quiet_NaN();
  };
  using SensingTrackDiagnosticVector =
      std::vector<SensingTrackDiagnostic,
                  Eigen::aligned_allocator<SensingTrackDiagnostic>>;

  struct StepDebugSnapshot {
    bool valid = false;
    bool cbf_active = false;
    bool solve_ok = true;
    bool turning = false;
    bool solver_cbf_active = false;
    bool solver_fail_flag = false;
    bool solver_track_only = true;
    std::string active_obstacle_key = "none";
    std::string solver_active_obstacle_key = "none";
    Obstacle active_obstacle_world = Obstacle::Zero();
    Obstacle active_obstacle_solver = Obstacle::Zero();
    Eigen::Vector3d truth_uav_position_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d estimated_uav_position_world = Eigen::Vector3d::Zero();
    double uav_position_error_m =
        std::numeric_limits<double>::quiet_NaN();
    SensingTrackDiagnostic solver_active_static_track;
    double solver_surface_error_vs_track_m =
        std::numeric_limits<double>::quiet_NaN();
    double solver_surface_error_vs_truth_m =
        std::numeric_limits<double>::quiet_NaN();
    double solver_h = std::numeric_limits<double>::quiet_NaN();
    double solver_hdot = std::numeric_limits<double>::quiet_NaN();
    double solver_cbf = std::numeric_limits<double>::quiet_NaN();
    double solver_slack = std::numeric_limits<double>::quiet_NaN();
    double solver_planar_distance = std::numeric_limits<double>::quiet_NaN();
    double solver_planar_clearance = std::numeric_limits<double>::quiet_NaN();
    double solver_planar_surface_distance =
        std::numeric_limits<double>::quiet_NaN();
    double feedback_time_ms = std::numeric_limits<double>::quiet_NaN();
    double preparation_time_ms = std::numeric_limits<double>::quiet_NaN();
    double core_time_ms = std::numeric_limits<double>::quiet_NaN();
    double tracking_error = std::numeric_limits<double>::quiet_NaN();
    double diag_min_h_any = std::numeric_limits<double>::quiet_NaN();
    double diag_min_cbf_any = std::numeric_limits<double>::quiet_NaN();
    double min_h = std::numeric_limits<double>::quiet_NaN();
    double min_cbf = std::numeric_limits<double>::quiet_NaN();
    double slack = std::numeric_limits<double>::quiet_NaN();
    double slack_raw = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d omega_non = Eigen::Vector3d::Zero();
    Eigen::Vector3d beta_non = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_car_non = Eigen::Vector3d::Zero();
    Eigen::Vector3d car_a_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d car_omega_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d rot_load_position_non = Eigen::Vector3d::Zero();
    Eigen::Vector3d rot_load_velocity_non = Eigen::Vector3d::Zero();
    Eigen::Vector3d rot_load_omega_non = Eigen::Vector3d::Zero();
    std::string obstacle_sensing_mode = "map";
    bool sensing_scan_triggered = false;
    std::uint64_t sensing_scan_index = 0;
    std::size_t sensing_cast_rays = 0;
    std::size_t sensing_truth_obstacles = 0;
    std::size_t sensing_visible_obstacles = 0;
    std::size_t sensing_generated_measurements = 0;
    std::size_t sensing_dropped_measurements = 0;
    std::size_t sensing_delivered_measurements = 0;
    std::size_t sensing_created_tracks = 0;
    std::size_t sensing_reinitialized_tracks = 0;
    std::size_t sensing_kalman_corrections = 0;
    std::size_t sensing_deleted_tracks = 0;
    std::size_t sensing_active_tracks = 0;
    double truth_static_min_surface_clearance =
        std::numeric_limits<double>::quiet_NaN();
    SensingSafetySnapshot sensing_safety;
    SensingTrackDiagnosticVector sensing_tracks;
  };

  struct SolverObstacleProfileSnapshot {
    bool valid = false;
    int uav_idx = -1;
    std::vector<std::string> keys;
    ObstacleVector world_obstacles;
    ObstacleProfileVector solver_profiles;
  };
  
  NumSimMpc(const ros::NodeHandle &nh, const ros::NodeHandle &pnh,const int id);
  virtual ~NumSimMpc() override = default;
  NumSimMpc(const NumSimMpc &) = delete;
  NumSimMpc &operator=(const NumSimMpc &) = delete;
  NumSimMpc(NumSimMpc &&) = delete;
  NumSimMpc &operator=(NumSimMpc &&) = delete;

  void setImu(const Eigen::Vector3d &imu) { imu_ = imu; };
  void setCarBetaNon(const Eigen::Vector3d &beta_non) { car_beta_non_ = beta_non; };
  void setCarNonInertialStageProfiles(const NonInertialProfile& omega_non_stage_profile,
                                      const NonInertialProfile& beta_non_stage_profile,
                                      const NonInertialProfile& a_car_non_stage_profile) {
    car_omega_non_stage_profile_ = omega_non_stage_profile;
    car_beta_non_stage_profile_ = beta_non_stage_profile;
    car_a_car_non_stage_profile_ = a_car_non_stage_profile;
    has_explicit_noninertial_stage_profiles_ = true;
    has_car_beta_non_stage_profile_ = true;
  }
  void setCarBetaNonStageProfile(const NonInertialProfile& beta_non_stage_profile) {
    car_beta_non_stage_profile_ = beta_non_stage_profile;
    has_car_beta_non_stage_profile_ = true;
  }
  void setTurningOmegaThreshold(double omega_threshold) {
    turning_omega_threshold_ = std::max(0.0, omega_threshold);
  }
  void setDebugNonInertialProfileSimTime(double sim_time) {
    debug_noninertial_profile_sim_time_ = sim_time;
  }
  void setDebugTrajectoryId(const std::string& traj_id) {
    debug_traj_id_ = traj_id;
  }
  void setCarTrajectoryWindow(const CarTrajectoryWindow& car_trajectory_window) {
    car_trajectory_window_ = car_trajectory_window;
  }
  void setObstacleFrameTrajectoryWindow(
      const CarTrajectoryWindow& obstacle_frame_trajectory_window) {
    obstacle_frame_trajectory_window_ = obstacle_frame_trajectory_window;
  }
  const CarTrajectoryWindow& getCarTrajectoryWindow() const {
    return car_trajectory_window_;
  }
  const SolverObstacleProfileSnapshot& getLastSolverObstacleProfileSnapshot() const {
    return last_solver_obstacle_profile_snapshot_;
  }
  void setCarOdom(const nav_msgs::Odometry &car_odom);
  void setQuadOdom(const nav_msgs::Odometry &quad_odom, int i);
  static void setSharedQuadOdom(const nav_msgs::Odometry &quad_odom, int i);
  Eigen::Vector3d getCarPosition() const {
    return Eigen::Vector3d(car_odom_.pose.pose.position.x,
                           car_odom_.pose.pose.position.y,
                           car_odom_.pose.pose.position.z);
  }
  Eigen::Quaterniond getCarOrientation() const {
    return Eigen::Quaterniond(car_odom_.pose.pose.orientation.w,
                              car_odom_.pose.pose.orientation.x,
                              car_odom_.pose.pose.orientation.y,
                              car_odom_.pose.pose.orientation.z);
  }
  Eigen::Vector3d getCarLinearVelocity() const {
    return measured_car_linear_velocity_;
  }
  Eigen::Vector3d getCarAngularVelocity() const {
    return measured_car_angular_velocity_;
  }
  virtual acado_mpc_common::ControlCommand run() override;
  bool isInertialFrame() const { return frame_mode_ == FrameMode::kInertial; }
  bool getLastSolveOk() const { return mpc_controller_.getLastSolveOk(); }
  const std::string& getFrameModeEffective() const { return frame_mode_effective_; }
  const std::string& getSafetyVariant() const { return safety_variant_; }
  const std::string& getObstacleSensingMode() const {
    return obstacle_sensing_mode_;
  }
  double getMaxVelocityXY() const { return mpc_params_.max_v_xy_; }
  double getMaxVelocityZ() const { return mpc_params_.max_v_z_; }
  double getMaxYawRate() const { return mpc_params_.max_yaw_rate_; }
  double getPredictionDt() const { return mpc_controller_.getPredictionDt(); }
  std::vector<double> getTrackingErrors() const { return metrics_tracking_errors_; }
  std::vector<double> getTrackingErrorsTrackOnly() const {
    return metrics_tracking_errors_track_only_;
  }
  std::vector<double> getSolveTimeSeriesMs() const { return metrics_solve_time_ms_; }
  std::vector<double> getFeedbackTimeSeriesMs() const { return metrics_feedback_time_ms_; }
  std::vector<double> getPreparationTimeSeriesMs() const {
    return metrics_preparation_time_ms_;
  }
  StepDebugSnapshot getLastStepDebugSnapshot() const { return prev_step_debug_; }

  static void getSharedPositions(std::vector<Eigen::Vector3d>& positions,
                                 std::vector<Eigen::Vector3d>& velocities,
                                 std::vector<bool>& valid);
  static void setStaticObstacles(
      const ObstacleVector& obstacles,
      const std::vector<double>& obstacle_heights_m = {});
  MetricsSummary getMetrics() const;
 
 private: 
  static std::mutex shared_state_mutex_;
  static std::vector<Eigen::Vector3d> shared_positions_;
  static std::vector<Eigen::Vector3d> shared_velocities_;
  static std::vector<bool> shared_position_valid_;
  static ObstacleVector shared_static_obstacles_;
  static std::vector<double> shared_static_obstacle_heights_m_;

  int  quad_id;
  std::mutex mtx;
  ros::NodeHandle nh_, pnh_;
  Eigen::Vector3d imu_;
  Eigen::Vector3d car_beta_non_;
  NonInertialProfile car_omega_non_stage_profile_;
  NonInertialProfile car_beta_non_stage_profile_;
  NonInertialProfile car_a_car_non_stage_profile_;
  bool has_explicit_noninertial_stage_profiles_;
  bool has_car_beta_non_stage_profile_;
  CarTrajectoryWindow car_trajectory_window_;
  CarTrajectoryWindow obstacle_frame_trajectory_window_;
  nav_msgs::Odometry car_odom_;
  nav_msgs::Odometry quad_odom_;
  nav_msgs::Odometry relative_est_;
  const char *CAR_ODOM_TOPIC = "coni_mpc/car_odom";
  const char *CAR_PATH_TOPIC = "coni_mpc/car_path";
  const char *QUAD_ODOM_TOPIC = "coni_mpc/quad_odom";
  const char *RELATIVE_EST_TOPIC = "coni_mpc/relative_est";
  const char *QUAD_PATH_TOPIC = "coni_mpc/quad_path";
  static constexpr std::size_t kMaxPathLength = 5000;

  // publishers in world frame
  ros::Publisher car_odom_pub_;
  ros::Publisher car_path_pub_;
  ros::Publisher car_path_marker_pub_;
  ros::Publisher car_body_marker_pub_;
  std::vector <ros::Publisher> quad_odom_pub_;
  std::vector <ros::Publisher> quad_path_pub_;
  std::vector <ros::Publisher> quad_path_marker_pub_;
  // relative estimation
/*   ros::Publisher relative_est_pub_; */
   std::vector <ros::Publisher> relative_est_pub_;
  nav_msgs::Path car_path_;
  nav_msgs::Path quad_path_;
  ros::Publisher quad_radius_marker_pub_;
  double car_radius_;
  double uav_radius_;
  ObstacleVector last_obstacles_;
  bool cbf_use_in_sim_;
  FrameMode frame_mode_;
  std::string frame_mode_effective_;
  std::string safety_variant_;
  bool zero_slack_required_;
  using ObstacleSensingStepStats = ObstacleSensingTracker::StepStats;
  std::string obstacle_sensing_mode_;
  bool obstacle_sensing_online_;
  double sensing_range_min_m_;
  double sensing_range_max_m_;
  double sensing_horizontal_fov_rad_;
  double sensing_angular_resolution_rad_;
  std::size_t sensing_minimum_hit_rays_;
  double sensing_control_rate_hz_;
  double sensing_update_rate_hz_;
  double sensing_position_std_m_;
  double sensing_dropout_probability_;
  double sensing_delay_sec_;
  double sensing_hold_sec_;
  double sensing_kf_process_variance_;
  double sensing_safety_clearance_threshold_m_;
  ObstacleSensingTracker obstacle_sensing_tracker_;
  ObstacleSensingStepStats sensing_step_stats_;
  SensingSafetySnapshot sensing_safety_snapshot_;
  SensingTrackDiagnosticVector sensing_track_diagnostics_;
  std::mt19937 noise_rng_;
  std::normal_distribution<double> noise_dist_;
  Eigen::Vector3d measured_car_linear_velocity_;
  Eigen::Vector3d measured_car_angular_velocity_;
  double estimation_position_noise_std_;
  double estimation_relative_velocity_noise_std_;
  double estimation_orientation_noise_std_;
  double estimation_imu_non_noise_std_;
  double car_linear_velocity_noise_std_;
  double car_angular_velocity_noise_std_;
  bool metrics_debug_;
  bool debug_check_noninertial_profile_col0_;
  bool debug_force_zero_a_car_non_profile_;
  double debug_noninertial_profile_sim_time_;
  std::string debug_traj_id_;
  bool warm_start_;
  bool inter_uav_priority_enabled_;
  int obstacle_hysteresis_hold_cycles_;
  int obstacle_hysteresis_cycles_since_switch_;
  double obstacle_hysteresis_min_risk_improvement_;
  double metrics_min_h_;
  double metrics_min_cbf_;
  double metrics_max_slack_;
  double metrics_sum_h_;
  double metrics_sum_cbf_;
  double metrics_tracking_sum_sq_;
  Eigen::Vector3d metrics_tracking_sum_sq_xyz_;
  std::vector<double> metrics_tracking_errors_;
  double metrics_tracking_sum_sq_track_only_;
  Eigen::Vector3d metrics_tracking_sum_sq_track_only_xyz_;
  std::vector<double> metrics_tracking_errors_track_only_;
  double metrics_tracking_sum_sq_cbf_active_;
  double metrics_tracking_sum_sq_solver_cbf_active_;
  double metrics_tracking_sum_sq_turn_;
  double metrics_tracking_sum_sq_straight_;
  double metrics_diag_min_h_any_;
  double metrics_diag_min_cbf_any_;
  double metrics_diag_sum_h_any_;
  double metrics_diag_sum_cbf_any_;
  std::size_t metrics_solver_metric_samples_;
  std::size_t metrics_solver_cbf_active_tracking_samples_;
  std::size_t metrics_solver_track_only_samples_;
  double metrics_solver_min_h_;
  double metrics_solver_min_planar_clearance_;
  double metrics_solver_min_h_witness_h_;
  double metrics_solver_min_h_witness_planar_clearance_;
  double metrics_solver_min_h_witness_tracking_error_;
  double metrics_solver_min_h_witness_delta_;
  double metrics_solver_min_h_witness_planar_distance_;
  std::string metrics_solver_min_h_witness_active_obstacle_key_;
  std::vector<double> metrics_c_values_;
  std::vector<double> metrics_c66_coupled_values_;
  std::vector<double> metrics_step_min_h_;
  std::vector<double> metrics_step_slack_;
  std::vector<double> metrics_step_tracking_error_;
  std::vector<int> metrics_step_fail_flags_;
  double metrics_dmin_min_;
  double metrics_truth_static_min_surface_clearance_;
  double metrics_slack_sum_;
  double metrics_slack_sum_turn_;
  double metrics_slack_sum_straight_;
  double metrics_max_slack_turn_;
  double metrics_max_slack_straight_;
  double metrics_min_h_turn_;
  double metrics_min_h_straight_;
  std::vector<double> metrics_solve_time_ms_;
  double metrics_solve_time_max_ms_;
  std::vector<double> metrics_feedback_time_ms_;
  double metrics_feedback_time_max_ms_;
  std::vector<double> metrics_preparation_time_ms_;
  double metrics_preparation_time_max_ms_;
  std::size_t metrics_failures_;
  std::size_t metrics_failures_turn_;
  std::size_t metrics_failures_straight_;
  std::size_t metrics_cbf_active_samples_;
  std::size_t metrics_turn_samples_;
  std::size_t metrics_straight_samples_;
  std::size_t metrics_samples_;
  double turning_omega_threshold_;
  std::string last_active_obstacle_key_;
  bool last_cbf_active_;
  std::vector<std::string> last_selected_obstacle_keys_;
  SolverObstacleProfileSnapshot last_solver_obstacle_profile_snapshot_;
  std::vector<Eigen::Vector3d> last_predicted_world_positions_;
  bool has_last_predicted_world_positions_;
  StepDebugSnapshot prev_step_debug_;

  void genRelativeEstimate();
  void buildNonInertialDataProfile(NonInertialProfile& omega_non_profile,
                                   NonInertialProfile& beta_non_profile,
                                   NonInertialProfile& a_car_non_profile) const;
  double sampleNormal();
  void resetMetrics();
  void logFailureContext(const StepDebugSnapshot& current_step) const;
  void setEstimationNoise(double p_var, 
                          double v_var, 
                          double ori_var, 
                          double imu_non_var);
  void syncRelativeEstimateMsgFromStateEstimate();
  ObstacleVector buildAvailableStaticObstacles(
      const ObstacleVector& truth_obstacles,
      const std::vector<double>& obstacle_heights_m,
      std::vector<std::string>& obstacle_keys);




}; // class NumSimMpc

} // namespace coni_mpc



#endif // CONI_MPC_NUM_SIM_MPC_H
