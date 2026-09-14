#include "coni_mpc/adaptive_trend_gain.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace coni_mpc {

namespace {

std::string toLowerCopy(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

}  // namespace

const char* trendPredictionModeName(TrendPredictionMode mode) {
  switch (mode) {
    case TrendPredictionMode::kDecayOnly:
      return "decay_only";
    case TrendPredictionMode::kAdaptiveGamma:
      return "adaptive_gamma";
    case TrendPredictionMode::kConstantJerk:
      return "constant_jerk";
  }
  return "decay_only";
}

TrendPredictionMode parseTrendPredictionMode(const std::string& text) {
  const std::string lowered = toLowerCopy(text);
  if (lowered.empty() || lowered == "decay_only" || lowered == "decay" ||
      lowered == "legacy") {
    return TrendPredictionMode::kDecayOnly;
  }
  if (lowered == "adaptive_gamma" || lowered == "adaptive") {
    return TrendPredictionMode::kAdaptiveGamma;
  }
  if (lowered == "constant_jerk" || lowered == "jerk") {
    return TrendPredictionMode::kConstantJerk;
  }
  throw std::invalid_argument("unsupported trend prediction mode: " + text);
}

AdaptiveTrendGain::AdaptiveTrendGain(const AdaptiveTrendGainConfig& cfg)
    : cfg_(cfg) {
  cfg_.window_size = std::max(2, cfg_.window_size);
  cfg_.alpha = std::max(0.0, cfg_.alpha);
  cfg_.gamma_min = std::max(0.0, cfg_.gamma_min);
  cfg_.gamma_max = std::max(cfg_.gamma_min, cfg_.gamma_max);
  cfg_.acc_threshold = std::max(0.0, cfg_.acc_threshold);
  cfg_.acc_floor = std::max(1e-6, cfg_.acc_floor);
  cfg_.trend_lambda = std::max(0.0, cfg_.trend_lambda);
}

double AdaptiveTrendGain::clampAbs(double value, double max_abs_value) {
  if (max_abs_value <= 1e-9) {
    return value;
  }
  return std::max(-max_abs_value, std::min(max_abs_value, value));
}

double AdaptiveTrendGain::computeAverageJerk(
    const std::deque<double>& filtered_acc_history,
    double dt) const {
  if (!(std::isfinite(dt) && dt > 1e-9)) {
    return 0.0;
  }
  if (static_cast<int>(filtered_acc_history.size()) < cfg_.window_size) {
    return 0.0;
  }

  const std::size_t start_idx =
      filtered_acc_history.size() - static_cast<std::size_t>(cfg_.window_size);
  double jerk_sum = 0.0;
  int jerk_count = 0;
  for (std::size_t idx = start_idx + 1u; idx < filtered_acc_history.size(); ++idx) {
    const double prev = filtered_acc_history[idx - 1u];
    const double curr = filtered_acc_history[idx];
    jerk_sum += (curr - prev) / dt;
    ++jerk_count;
  }
  if (jerk_count <= 0) {
    return 0.0;
  }
  return jerk_sum / static_cast<double>(jerk_count);
}

double AdaptiveTrendGain::computeGamma(
    const std::deque<double>& filtered_acc_history,
    double current_accel,
    double dt,
    double* j_avg_out) const {
  const double j_avg = computeAverageJerk(filtered_acc_history, dt);
  if (j_avg_out != nullptr) {
    *j_avg_out = j_avg;
  }
  if (static_cast<int>(filtered_acc_history.size()) < cfg_.window_size) {
    return 1.0;
  }
  if (!std::isfinite(current_accel) || std::abs(current_accel) < cfg_.acc_threshold) {
    return 1.0;
  }

  const double denom = std::max(std::abs(current_accel), cfg_.acc_floor);
  const double gamma =
      1.0 + cfg_.alpha * (j_avg * dt) / denom;
  return std::max(cfg_.gamma_min, std::min(cfg_.gamma_max, gamma));
}

double AdaptiveTrendGain::predictFrozenTrend(
    double initial_accel,
    double frozen_avg_jerk,
    double dt,
    int step_idx,
    double accel_limit,
    double* damping_out) const {
  if (damping_out != nullptr) {
    *damping_out = 1.0;
  }
  if (!(std::isfinite(dt) && dt > 1e-9)) {
    return clampAbs(initial_accel, accel_limit);
  }

  const int clamped_step_idx = std::max(0, step_idx);
  if (clamped_step_idx == 0) {
    return clampAbs(initial_accel, accel_limit);
  }

  const double horizon_time = static_cast<double>(clamped_step_idx) * dt;
  const double damping = std::exp(-cfg_.trend_lambda * horizon_time);

  if (damping_out != nullptr) {
    *damping_out = damping;
  }
  const double trend_accel = initial_accel + frozen_avg_jerk * horizon_time;
  return clampAbs(trend_accel * damping, accel_limit);
}

double AdaptiveTrendGain::predictNext(
    double current_accel,
    const std::deque<double>& filtered_acc_history,
    double dt,
    TrendPredictionMode mode,
    double decay_gamma,
    double accel_limit,
    double* j_avg_out,
    double* gamma_out) const {
  double j_avg = 0.0;
  double gamma = 1.0;
  double next_accel = current_accel;

  switch (mode) {
    case TrendPredictionMode::kDecayOnly:
      gamma = decay_gamma;
      next_accel = current_accel * decay_gamma;
      break;
    case TrendPredictionMode::kAdaptiveGamma:
      j_avg = computeAverageJerk(filtered_acc_history, dt);
      next_accel = predictFrozenTrend(
          current_accel, j_avg, dt, 1, accel_limit, &gamma);
      if (std::isfinite(current_accel) &&
          std::abs(current_accel) > cfg_.acc_floor) {
        gamma = next_accel / current_accel;
      } else {
        gamma = 1.0;
      }
      break;
    case TrendPredictionMode::kConstantJerk:
      j_avg = computeAverageJerk(filtered_acc_history, dt);
      gamma = 1.0;
      next_accel = current_accel + j_avg * dt;
      break;
  }

  if (j_avg_out != nullptr) {
    *j_avg_out = j_avg;
  }
  if (gamma_out != nullptr) {
    *gamma_out = gamma;
  }
  return clampAbs(next_accel, accel_limit);
}

}  // namespace coni_mpc
