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
#include "coni_mpc/RiskRegionArray.h"
#include "coni_mpc/field_hocbf.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <random>
#include <memory>
#include <string>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
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
  using Obstacle = Eigen::Matrix<double, 7, 1>;
  using ObstacleVector = std::vector<Obstacle,
      Eigen::aligned_allocator<Obstacle>>;
  using ObstacleProfile = acado_mpc::MpcWrapper<double>::ObstacleProfile;
  using ObstacleProfileVector = acado_mpc::MpcWrapper<double>::ObstacleProfileVector;
  using RiskRegion = acado_mpc::MpcWrapper<double>::RiskRegion;
  using RiskRegionVector = acado_mpc::MpcWrapper<double>::RiskRegionVector;
  using RiskRegionProfile = acado_mpc::MpcWrapper<double>::RiskRegionProfile;
  using RiskRegionProfileVector = acado_mpc::MpcWrapper<double>::RiskRegionProfileVector;
  using NonInertialProfile = Eigen::Matrix<double, 3, acado_mpc::kSamples + 1>;
  using FieldHocbfProfile = acado_mpc::MpcWrapper<double>::FieldHocbfProfile;

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
  const CarTrajectoryWindow& getCarTrajectoryWindow() const {
    return car_trajectory_window_;
  }
  const SolverObstacleProfileSnapshot& getLastSolverObstacleProfileSnapshot() const {
    return last_solver_obstacle_profile_snapshot_;
  }
  void setCarOdom(const nav_msgs::Odometry &car_odom);
  void setQuadOdom(const nav_msgs::Odometry &quad_odom, int i);
  void riskRegionCallback(const coni_mpc::RiskRegionArray::ConstPtr& message);
  void fieldOccupancyCallback(const sensor_msgs::PointCloud2::ConstPtr& message);
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
  // Return the same noisy state estimate consumed by the current MPC solve,
  // expressed in the world frame.  Reactive safety filters must use this
  // instead of simulator truth so all compared controllers receive the same
  // observation quality.
  void getEstimatedWorldKinematics(Eigen::Vector3d& position_world,
                                   Eigen::Vector3d& velocity_world) const;
  // Update the same noisy estimate and comparable per-step metrics used by
  // the MPC runs, without invoking the ACADO solver.  Reactive baselines use
  // this before computing their own velocity command.
  void runReactiveBaselineStep();
  virtual acado_mpc_common::ControlCommand run() override;
  bool isInertialFrame() const { return frame_mode_ == FrameMode::kInertial; }
  bool getLastSolveOk() const { return mpc_controller_.getLastSolveOk(); }
  const std::string& getFrameModeEffective() const { return frame_mode_effective_; }
  const std::string& getSafetyVariant() const { return safety_variant_; }
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
  static void setStaticObstacles(const ObstacleVector& obstacles);
  MetricsSummary getMetrics() const;
 
 private: 
  static std::mutex shared_state_mutex_;
  static std::vector<Eigen::Vector3d> shared_positions_;
  static std::vector<Eigen::Vector3d> shared_velocities_;
  static std::vector<bool> shared_position_valid_;
  static ObstacleVector shared_static_obstacles_;

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
  ros::Subscriber risk_region_subscriber_;
  std::mutex risk_region_mutex_;
  coni_mpc::RiskRegionArray latest_risk_regions_;
  bool has_latest_risk_regions_ = false;
  bool risk_regions_enabled_ = false;
  // The DSP/risk-region constructor publishes a complete 21-stage horizon
  // less frequently than the MPC loop.  Keep the last complete horizon for
  // a bounded processing gap instead of disabling all barriers at 0.5 s;
  // the profile is shifted by its measured message age before use.
  double risk_regions_timeout_ = 4.0;
  struct FieldSnapshot {
    ros::Time stamp;
    std::string frame_id;
    field_hocbf::Points points;
    std::uint32_t last_occupied_stage = 0;
  };
  ros::Subscriber field_occupancy_subscriber_;
  std::mutex field_snapshot_mutex_;
  std::shared_ptr<const FieldSnapshot> field_snapshot_;
  bool field_hocbf_enabled_ = false;
  // Keep timestamp/stage compensation configurable so the same run can be
  // compared with and without delay alignment. Enabled by default to retain
  // the current behavior.
  bool risk_region_time_alignment_ = true;
  double car_radius_;
  double uav_radius_;
  ObstacleVector last_obstacles_;
  bool cbf_use_in_sim_;
  FrameMode frame_mode_;
  std::string frame_mode_effective_;
  std::string safety_variant_;
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




}; // class NumSimMpc

} // namespace coni_mpc



#endif // CONI_MPC_NUM_SIM_MPC_H
