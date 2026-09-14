#include "coni_mpc/non_inertial_predictor.hpp"

#include <algorithm>
#include <ros/ros.h>

namespace coni_mpc {

NonInertialPredictor::NonInertialPredictor(int horizon_steps,
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
                                           double stop_speed_threshold)
    : horizon_steps_(std::max(0, horizon_steps)),
      dt_mpc_(dt_mpc),
      warmup_frames_(warmup_frames),
      ema_alpha_min_(ema_alpha_min),
      ema_alpha_max_(ema_alpha_max),
      var_threshold_(std::max(1e-6, var_threshold)),
      gamma_a_(gamma_a),
      gamma_alpha_(gamma_alpha),
      speed_max_(std::max(0.0, speed_max)),
      omega_z_max_(std::max(0.0, std::abs(omega_z_max))),
      a_max_(a_max),
      alpha_max_(alpha_max),
      exit_history_size_(std::max(0, exit_history_size)),
      trend_mode_(trend_mode),
      long_trend_gain_(long_trend_cfg),
      angular_trend_gain_(angular_trend_cfg),
      debug_log_(debug_log),
      turn_exit_omega_hold_(std::max(0.0, turn_exit_omega_hold)),
      turn_exit_omega_low_(std::max(0.0, turn_exit_omega_low)),
      accel_exit_a_hold_(std::max(0.0, accel_exit_a_hold)),
      accel_exit_a_low_(std::max(0.0, accel_exit_a_low)),
      cruise_speed_ratio_(std::max(0.0, cruise_speed_ratio)),
      stop_speed_threshold_(std::max(0.0, stop_speed_threshold)),
      initialized_(false),
      frame_count_(0),
      long_speed_prev_(0.0),
      omega_z_prev_(0.0),
      a_long_ema_(0.0),
      alpha_z_ema_(0.0),
      var_a_long_(0.0),
      var_alpha_z_(0.0) {
}

double NonInertialPredictor::clampScalar(double value,
                                         double max_abs_value) const {
  if (max_abs_value <= 1e-9) {
    return 0.0;
  }
  if (value > max_abs_value) {
    return max_abs_value;
  }
  if (value < -max_abs_value) {
    return -max_abs_value;
  }
  return value;
}

double NonInertialPredictor::computeAdaptiveAlpha(double variance) const {
  return ema_alpha_max_ - (ema_alpha_max_ - ema_alpha_min_) *
                              std::exp(-variance / var_threshold_);
}

void NonInertialPredictor::updateAdaptiveScalar(double raw,
                                                double& ema,
                                                double& variance) const {
  const double var_beta = 0.1;
  const double adaptive_alpha = computeAdaptiveAlpha(variance);
  ema = adaptive_alpha * raw + (1.0 - adaptive_alpha) * ema;
  const double residual = raw - ema;
  variance = var_beta * residual * residual + (1.0 - var_beta) * variance;
}

void NonInertialPredictor::appendHistorySample(std::deque<double>& history,
                                               double value,
                                               int max_size) {
  if (max_size <= 0) {
    return;
  }
  history.push_back(value);
  while (static_cast<int>(history.size()) > max_size) {
    history.pop_front();
  }
}

double NonInertialPredictor::meanAbs(const std::deque<double>& history) const {
  if (history.empty()) {
    return 0.0;
  }
  double sum_abs = 0.0;
  for (double value : history) {
    sum_abs += std::abs(value);
  }
  return sum_abs / static_cast<double>(history.size());
}

bool NonInertialPredictor::detectTurnExit(double omega_z_curr,
                                          double alpha_z_raw) const {
  if (exit_history_size_ <= 0 || omega_z_hist_.size() < 2u) {
    return false;
  }

  const double recent_turn_level = meanAbs(omega_z_hist_);
  if (recent_turn_level < turn_exit_omega_hold_) {
    return false;
  }

  if (std::abs(omega_z_curr) > turn_exit_omega_low_) {
    return false;
  }

  const double omega_z_prev = omega_z_hist_.back();
  if (std::abs(omega_z_prev) <= turn_exit_omega_low_) {
    return false;
  }

  const bool braking_previous_turn = alpha_z_raw * omega_z_prev < 0.0;
  const bool crossed_or_released =
      (omega_z_prev * omega_z_curr <= 0.0) ||
      (std::abs(omega_z_curr) <= 0.5 * std::abs(omega_z_prev));
  return braking_previous_turn && crossed_or_released;
}

bool NonInertialPredictor::detectLongitudinalExit(double speed_curr,
                                                  double a_long_raw) const {
  if (exit_history_size_ <= 0 || a_long_hist_.size() < 2u) {
    return false;
  }

  const double recent_accel_level = meanAbs(a_long_hist_);
  if (recent_accel_level < accel_exit_a_hold_) {
    return false;
  }

  const double a_long_prev = a_long_hist_.back();
  const bool sign_flip = a_long_prev * a_long_raw < 0.0;
  const bool settled_near_zero = std::abs(a_long_raw) <= accel_exit_a_low_;
  const bool near_cruise =
      speed_max_ > 1e-9 &&
      speed_curr >= cruise_speed_ratio_ * speed_max_ &&
      a_long_raw > 0.0;
  const bool near_stop =
      speed_curr <= stop_speed_threshold_ && a_long_raw < 0.0;
  return sign_flip || settled_near_zero || near_cruise || near_stop;
}

double NonInertialPredictor::computeDampingGain(
    double q,
    double q_dot,
    double horizon_time,
    const PredictorParams& params) const {
  if (!(std::isfinite(q) && std::isfinite(q_dot) &&
        std::isfinite(horizon_time)) ||
      horizon_time <= params.eps) {
    return 1.0;
  }
  if (q * q_dot >= 0.0) {
    return 1.0;
  }

  const double t_q = std::abs(q) / (std::abs(q_dot) + params.eps);
  const double eta = t_q / horizon_time;
  return std::clamp(eta, params.eta_min, 1.0);
}

NonInertialScalarSequence
NonInertialPredictor::generateDampedVaryingCurvatureRollout(
    double dt,
    int Np,
    double acc_t,
    double jerk_t,
    double omega_t,
    double beta_t,
    const PredictorParams& params) const {
  const int steps = std::max(0, Np);
  const double safe_dt = (std::isfinite(dt) && dt > 0.0) ? dt : 0.0;
  const double horizon_time = static_cast<double>(steps) * safe_dt;

  NonInertialScalarSequence seq;
  seq.acc_seq.assign(static_cast<std::size_t>(steps + 1), 0.0);
  seq.omega_seq.assign(static_cast<std::size_t>(steps + 1), 0.0);
  seq.beta_seq.assign(static_cast<std::size_t>(steps + 1), 0.0);

  seq.acc_seq[0] = std::isfinite(acc_t) ? acc_t : 0.0;
  seq.omega_seq[0] = std::isfinite(omega_t) ? omega_t : 0.0;
  seq.beta_seq[0] = std::isfinite(beta_t) ? beta_t : 0.0;
  const double safe_jerk = std::isfinite(jerk_t) ? jerk_t : 0.0;
  const double safe_beta = std::isfinite(beta_t) ? beta_t : 0.0;

  for (int k = 0; k < steps; ++k) {
    const double eta_acc =
        params.enable_acc_damping
            ? computeDampingGain(seq.acc_seq[static_cast<std::size_t>(k)],
                                 safe_jerk,
                                 horizon_time,
                                 params)
            : 1.0;
    const double eta_omega =
        params.enable_omega_damping
            ? computeDampingGain(seq.omega_seq[static_cast<std::size_t>(k)],
                                 safe_beta,
                                 horizon_time,
                                 params)
            : 1.0;
    seq.acc_seq[static_cast<std::size_t>(k + 1)] =
        params.lambda_acc * seq.acc_seq[static_cast<std::size_t>(k)] +
        eta_acc * safe_jerk * safe_dt;
    seq.omega_seq[static_cast<std::size_t>(k + 1)] =
        params.lambda_omega * seq.omega_seq[static_cast<std::size_t>(k)] +
        eta_omega * safe_beta * safe_dt;
    seq.beta_seq[static_cast<std::size_t>(k + 1)] =
        (params.enable_beta_decay ? params.lambda_beta : 1.0) *
        seq.beta_seq[static_cast<std::size_t>(k)];
  }
  return seq;
}

NonInertialVector2Sequence
NonInertialPredictor::generateDampedVaryingCurvatureRollout(
    double dt,
    int Np,
    const Eigen::Vector2d& acc_t,
    const Eigen::Vector2d& jerk_t,
    double omega_t,
    double beta_t,
    const PredictorParams& params) const {
  const int steps = std::max(0, Np);
  const double safe_dt = (std::isfinite(dt) && dt > 0.0) ? dt : 0.0;
  const double horizon_time = static_cast<double>(steps) * safe_dt;

  NonInertialVector2Sequence seq;
  seq.acc_seq.assign(static_cast<std::size_t>(steps + 1),
                     Eigen::Vector2d::Zero());
  seq.omega_seq.assign(static_cast<std::size_t>(steps + 1), 0.0);
  seq.beta_seq.assign(static_cast<std::size_t>(steps + 1), 0.0);

  seq.acc_seq[0] =
      acc_t.allFinite() ? acc_t : Eigen::Vector2d::Zero();
  seq.omega_seq[0] = std::isfinite(omega_t) ? omega_t : 0.0;
  seq.beta_seq[0] = std::isfinite(beta_t) ? beta_t : 0.0;
  const Eigen::Vector2d safe_jerk =
      jerk_t.allFinite() ? jerk_t : Eigen::Vector2d::Zero();
  const double safe_beta = std::isfinite(beta_t) ? beta_t : 0.0;

  for (int k = 0; k < steps; ++k) {
    Eigen::Vector2d next_acc;
    for (int axis = 0; axis < 2; ++axis) {
      const double eta_acc =
          params.enable_acc_damping
              ? computeDampingGain(
                    seq.acc_seq[static_cast<std::size_t>(k)](axis),
                    safe_jerk(axis),
                    horizon_time,
                    params)
              : 1.0;
      next_acc(axis) =
          params.lambda_acc * seq.acc_seq[static_cast<std::size_t>(k)](axis) +
          eta_acc * safe_jerk(axis) * safe_dt;
    }

    const double eta_omega =
        params.enable_omega_damping
            ? computeDampingGain(seq.omega_seq[static_cast<std::size_t>(k)],
                                 safe_beta,
                                 horizon_time,
                                 params)
            : 1.0;
    seq.acc_seq[static_cast<std::size_t>(k + 1)] = next_acc;
    seq.omega_seq[static_cast<std::size_t>(k + 1)] =
        params.lambda_omega * seq.omega_seq[static_cast<std::size_t>(k)] +
        eta_omega * safe_beta * safe_dt;
    seq.beta_seq[static_cast<std::size_t>(k + 1)] =
        (params.enable_beta_decay ? params.lambda_beta : 1.0) *
        seq.beta_seq[static_cast<std::size_t>(k)];
  }
  return seq;
}

void NonInertialPredictor::updateAndPredict(const Eigen::Vector3d& v_world_curr,
                                            const Eigen::Vector3d& omega_non_curr,
                                            const Eigen::Quaterniond& W_q_non_curr,
                                            double dt,
                                            Eigen::MatrixXd& omega_profile,
                                            Eigen::MatrixXd& beta_profile,
                                            Eigen::MatrixXd& a_car_profile) {
  std::lock_guard<std::mutex> lock(mutex_);

  const int profile_cols = horizon_steps_ + 1;
  if (omega_profile.rows() != 3 || omega_profile.cols() != profile_cols) {
    omega_profile.resize(3, profile_cols);
  }
  if (beta_profile.rows() != 3 || beta_profile.cols() != profile_cols) {
    beta_profile.resize(3, profile_cols);
  }
  if (a_car_profile.rows() != 3 || a_car_profile.cols() != profile_cols) {
    a_car_profile.resize(3, profile_cols);
  }
  omega_profile.setZero();
  beta_profile.setZero();
  a_car_profile.setZero();

  Eigen::Quaterniond W_q_non = W_q_non_curr;
  if (W_q_non.norm() > 1e-9) {
    W_q_non.normalize();
  } else {
    W_q_non = Eigen::Quaterniond::Identity();
  }

  const Eigen::Vector3d v_non_curr = W_q_non.inverse() * v_world_curr;
  const double long_speed_curr = std::max(0.0, v_non_curr.x());
  const double omega_z_curr = omega_non_curr.z();
  auto fillProfiles = [&](double a_long_base,
                          double alpha_z_base,
                          bool frozen_like_warmup,
                          bool turn_exit_detected,
                          bool longitudinal_exit_detected) {
    double current_gamma_a = 1.0;
    double current_gamma_alpha = 1.0;
    double speed_pred = long_speed_curr;
    if (speed_max_ > 1e-9) {
      speed_pred = std::min(speed_pred, speed_max_);
    }
    double omega_z_pred = clampScalar(omega_z_curr, omega_z_max_);
    double a_long_state = clampScalar(a_long_base, a_max_);
    double alpha_z_state = clampScalar(alpha_z_base, alpha_max_);
    if (turn_exit_detected && std::abs(omega_z_pred) <= turn_exit_omega_low_) {
      omega_z_pred = 0.0;
    }
    if (longitudinal_exit_detected) {
      a_long_state = 0.0;
    }
    if (turn_exit_detected) {
      alpha_z_state = 0.0;
    }
    double long_j_avg = 0.0;
    double alpha_j_avg = 0.0;
    double long_gamma = 1.0;
    double alpha_gamma = 1.0;
    double long_damping = 1.0;
    double alpha_damping = 1.0;
    if (trend_mode_ == TrendPredictionMode::kAdaptiveGamma) {
      long_j_avg =
          long_trend_gain_.computeAverageJerk(a_long_filtered_hist_, dt_mpc_);
      alpha_j_avg =
          angular_trend_gain_.computeAverageJerk(alpha_z_filtered_hist_, dt_mpc_);
      if (longitudinal_exit_detected) {
        long_j_avg = 0.0;
      }
      if (turn_exit_detected) {
        alpha_j_avg = 0.0;
      }
    } else if (trend_mode_ == TrendPredictionMode::kConstantJerk) {
      long_j_avg = long_trend_gain_.computeAverageJerk(a_long_filtered_hist_, dt_mpc_);
      alpha_j_avg = angular_trend_gain_.computeAverageJerk(alpha_z_filtered_hist_, dt_mpc_);
    }
    const double frozen_a_long_base = a_long_state;
    const double frozen_alpha_z_base = alpha_z_state;
    double last_a_long_pred = clampScalar(a_long_state, a_max_);
    double last_alpha_z_pred = clampScalar(alpha_z_state, alpha_max_);
    if (trend_mode_ == TrendPredictionMode::kAdaptiveGamma) {
      long_gamma = 1.0;
      alpha_gamma = 1.0;
    }
    for (int stage_idx = 0; stage_idx <= horizon_steps_; ++stage_idx) {
      double a_long_pred = clampScalar(a_long_state, a_max_);
      double alpha_z_pred = clampScalar(alpha_z_state, alpha_max_);
      if (trend_mode_ == TrendPredictionMode::kAdaptiveGamma) {
        a_long_pred = long_trend_gain_.predictFrozenTrend(
            frozen_a_long_base,
            long_j_avg,
            dt_mpc_,
            stage_idx,
            a_max_,
            &long_damping);
        alpha_z_pred = angular_trend_gain_.predictFrozenTrend(
            frozen_alpha_z_base,
            alpha_j_avg,
            dt_mpc_,
            stage_idx,
            alpha_max_,
            &alpha_damping);
      }
      last_a_long_pred = a_long_pred;
      last_alpha_z_pred = alpha_z_pred;
      const Eigen::Vector3d omega_pred(0.0, 0.0, omega_z_pred);
      const Eigen::Vector3d alpha_pred(0.0, 0.0, alpha_z_pred);
      const Eigen::Vector3d a_pred =
          frozen_like_warmup ? Eigen::Vector3d::Zero()
                             : Eigen::Vector3d(a_long_pred,
                                               speed_pred * omega_z_pred,
                                               0.0);

      omega_profile.col(stage_idx) = omega_pred;
      beta_profile.col(stage_idx) = alpha_pred;
      a_car_profile.col(stage_idx) = a_pred;

      if (stage_idx < horizon_steps_) {
        speed_pred = std::max(0.0, speed_pred + a_long_pred * dt_mpc_);
        if (speed_max_ > 1e-9) {
          speed_pred = std::min(speed_pred, speed_max_);
        }
        omega_z_pred += alpha_z_pred * dt_mpc_;
        omega_z_pred = clampScalar(omega_z_pred, omega_z_max_);
        switch (trend_mode_) {
          case TrendPredictionMode::kDecayOnly:
            current_gamma_a *= gamma_a_;
            current_gamma_alpha *= gamma_alpha_;
            a_long_state = clampScalar(a_long_base * current_gamma_a, a_max_);
            alpha_z_state = clampScalar(alpha_z_base * current_gamma_alpha, alpha_max_);
            break;
          case TrendPredictionMode::kAdaptiveGamma:
            break;
          case TrendPredictionMode::kConstantJerk:
            a_long_state = long_trend_gain_.predictNext(
                a_long_state,
                a_long_filtered_hist_,
                dt_mpc_,
                trend_mode_,
                gamma_a_,
                a_max_,
                &long_j_avg,
                &long_gamma);
            alpha_z_state = angular_trend_gain_.predictNext(
                alpha_z_state,
                alpha_z_filtered_hist_,
                dt_mpc_,
                trend_mode_,
                gamma_alpha_,
                alpha_max_,
                &alpha_j_avg,
                &alpha_gamma);
            break;
        }
      }
    }
    if (debug_log_ && !frozen_like_warmup) {
      ROS_DEBUG_STREAM_THROTTLE(
          0.5,
          "[NonInertialPredictor] mode=" << trendPredictionModeName(trend_mode_)
          << " a_k=" << a_long_base
          << " j_avg=" << long_j_avg
          << " trend_lambda=" << long_trend_gain_.trendLambda()
          << " damping=" << long_damping
          << " a_pred=" << last_a_long_pred
          << " v_pred=" << speed_pred
          << " | alpha_k=" << alpha_z_base
          << " j_alpha=" << alpha_j_avg
          << " trend_lambda_alpha=" << angular_trend_gain_.trendLambda()
          << " damping_alpha=" << alpha_damping
          << " alpha_pred=" << last_alpha_z_pred
          << " omega_pred=" << omega_z_pred);
    }
  };

  if (dt <= 1e-6) {
    fillProfiles(0.0, 0.0, true, false, false);
    return;
  }

  if (!initialized_) {
    long_speed_prev_ = long_speed_curr;
    omega_z_prev_ = omega_z_curr;
    a_long_ema_ = 0.0;
    alpha_z_ema_ = 0.0;
    var_a_long_ = 0.0;
    var_alpha_z_ = 0.0;
    initialized_ = true;
    frame_count_ = 1;
    omega_z_hist_.clear();
    a_long_hist_.clear();
    a_long_filtered_hist_.clear();
    alpha_z_filtered_hist_.clear();
    appendHistorySample(omega_z_hist_, omega_z_curr, exit_history_size_);
    appendHistorySample(a_long_hist_, 0.0, exit_history_size_);
    appendHistorySample(a_long_filtered_hist_, 0.0, long_trend_gain_.windowSize());
    appendHistorySample(alpha_z_filtered_hist_, 0.0, angular_trend_gain_.windowSize());
    fillProfiles(0.0, 0.0, true, false, false);
    return;
  }

  const double a_long_raw =
      clampScalar((long_speed_curr - long_speed_prev_) / dt, a_max_);
  const double alpha_z_raw =
      clampScalar((omega_z_curr - omega_z_prev_) / dt, alpha_max_);
  const bool turn_exit_detected = detectTurnExit(omega_z_curr, alpha_z_raw);
  const bool longitudinal_exit_detected =
      detectLongitudinalExit(long_speed_curr, a_long_raw);

  updateAdaptiveScalar(a_long_raw, a_long_ema_, var_a_long_);
  updateAdaptiveScalar(alpha_z_raw, alpha_z_ema_, var_alpha_z_);

  long_speed_prev_ = long_speed_curr;
  omega_z_prev_ = omega_z_curr;
  appendHistorySample(omega_z_hist_, omega_z_curr, exit_history_size_);
  appendHistorySample(a_long_hist_, a_long_raw, exit_history_size_);
  appendHistorySample(a_long_filtered_hist_,
                      a_long_ema_,
                      long_trend_gain_.windowSize());
  appendHistorySample(alpha_z_filtered_hist_,
                      alpha_z_ema_,
                      angular_trend_gain_.windowSize());
  frame_count_++;

  double a_long_base = 0.0;
  double alpha_z_base = 0.0;
  const bool in_warmup =
      frame_count_ <= static_cast<std::uint64_t>(std::max(0, warmup_frames_));
  if (!in_warmup) {
    a_long_base = a_long_ema_;
    alpha_z_base = alpha_z_ema_;
  }
  fillProfiles(a_long_base,
               alpha_z_base,
               in_warmup,
               turn_exit_detected,
               longitudinal_exit_detected);
}

}  // namespace coni_mpc
