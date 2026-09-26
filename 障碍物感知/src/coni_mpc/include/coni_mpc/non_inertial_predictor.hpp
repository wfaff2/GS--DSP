#pragma once

#include "coni_mpc/adaptive_trend_gain.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <deque>
#include <mutex>
#include <vector>

namespace coni_mpc {

struct PredictorParams {
  double eta_min = 0.15;
  double eps = 1e-6;

  double lambda_acc = 1.0;
  double lambda_omega = 1.0;
  double lambda_beta = 0.95;

  bool enable_acc_damping = true;
  bool enable_omega_damping = true;
  bool enable_beta_decay = true;
};

struct NonInertialScalarSequence {
  std::vector<double> omega_seq;
  std::vector<double> beta_seq;
  std::vector<double> acc_seq;
};

struct NonInertialVector2Sequence {
  std::vector<double> omega_seq;
  std::vector<double> beta_seq;
  std::vector<Eigen::Vector2d> acc_seq;
};

class NonInertialPredictor {
 public:
  NonInertialPredictor(int horizon_steps,
                       double dt_mpc,
                       int warmup_frames,
                       double ema_alpha_min,
                       double ema_alpha_max,
                       double var_threshold,
                       double gamma_a,
                       double gamma_alpha,
                       double speed_max,
                       double omega_z_max,
                       double a_max,
                       double alpha_max,
                       int exit_history_size,
                       TrendPredictionMode trend_mode,
                       const AdaptiveTrendGainConfig& long_trend_cfg,
                       const AdaptiveTrendGainConfig& angular_trend_cfg,
                       bool debug_log,
                       double turn_exit_omega_hold,
                       double turn_exit_omega_low,
                       double accel_exit_a_hold,
                       double accel_exit_a_low,
                       double cruise_speed_ratio,
                       double stop_speed_threshold);

  // Estimate the current signed longitudinal/angular accelerations from
  // current and previous motion samples, smooth them with a variance-adaptive
  // EMA, then roll out stage-wise omega/beta/a profiles under one of:
  // decay_only / adaptive_gamma (frozen average jerk with exponentially
  // damped acceleration trend toward 0) / constant_jerk.
  void updateAndPredict(const Eigen::Vector3d& v_world_curr,
                        const Eigen::Vector3d& omega_non_curr,
                        const Eigen::Quaterniond& W_q_non_curr,
                        double dt,
                        Eigen::MatrixXd& omega_profile,
                        Eigen::MatrixXd& beta_profile,
                        Eigen::MatrixXd& a_car_profile);

  // Causal horizon-aware damped rollout for S-curve / figure-8 local
  // maneuvers. This is not an optimal predictor; it only suppresses
  // constant-derivative over-extrapolation across an MPC horizon.
  double computeDampingGain(double q,
                            double q_dot,
                            double horizon_time,
                            const PredictorParams& params) const;

  NonInertialScalarSequence generateDampedVaryingCurvatureRollout(
      double dt,
      int Np,
      double acc_t,
      double jerk_t,
      double omega_t,
      double beta_t,
      const PredictorParams& params = PredictorParams()) const;

  NonInertialVector2Sequence generateDampedVaryingCurvatureRollout(
      double dt,
      int Np,
      const Eigen::Vector2d& acc_t,
      const Eigen::Vector2d& jerk_t,
      double omega_t,
      double beta_t,
      const PredictorParams& params = PredictorParams()) const;

 private:
  double clampScalar(double value, double max_abs_value) const;
  double computeAdaptiveAlpha(double variance) const;
  void updateAdaptiveScalar(double raw, double& ema, double& variance) const;
  double meanAbs(const std::deque<double>& history) const;
  bool detectTurnExit(double omega_z_curr, double alpha_z_raw) const;
  bool detectLongitudinalExit(double speed_curr, double a_long_raw) const;
  void appendHistorySample(std::deque<double>& history,
                           double value,
                           int max_size);

  int horizon_steps_;
  double dt_mpc_;
  int warmup_frames_;
  double ema_alpha_min_;
  double ema_alpha_max_;
  double var_threshold_;
  double gamma_a_;
  double gamma_alpha_;
  double speed_max_;
  double omega_z_max_;
  double a_max_;
  double alpha_max_;
  int exit_history_size_;
  TrendPredictionMode trend_mode_;
  AdaptiveTrendGain long_trend_gain_;
  AdaptiveTrendGain angular_trend_gain_;
  bool debug_log_;
  double turn_exit_omega_hold_;
  double turn_exit_omega_low_;
  double accel_exit_a_hold_;
  double accel_exit_a_low_;
  double cruise_speed_ratio_;
  double stop_speed_threshold_;

  std::mutex mutex_;
  bool initialized_;
  std::uint64_t frame_count_;

  double long_speed_prev_;
  double omega_z_prev_;
  double a_long_ema_;
  double alpha_z_ema_;
  double var_a_long_;
  double var_alpha_z_;
  std::deque<double> omega_z_hist_;
  std::deque<double> a_long_hist_;
  std::deque<double> a_long_filtered_hist_;
  std::deque<double> alpha_z_filtered_hist_;
};

}  // namespace coni_mpc
