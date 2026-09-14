#pragma once

#include <deque>
#include <string>

namespace coni_mpc {

enum class TrendPredictionMode {
  kDecayOnly = 0,
  kAdaptiveGamma,
  kConstantJerk,
};

const char* trendPredictionModeName(TrendPredictionMode mode);
TrendPredictionMode parseTrendPredictionMode(const std::string& text);

struct AdaptiveTrendGainConfig {
  int window_size = 5;
  double alpha = 0.6;
  double gamma_min = 0.9;
  double gamma_max = 1.08;
  double acc_threshold = 0.05;
  double acc_floor = 0.1;
  double trend_lambda = 6.0;
};

class AdaptiveTrendGain {
 public:
  explicit AdaptiveTrendGain(
      const AdaptiveTrendGainConfig& cfg = AdaptiveTrendGainConfig());

  int windowSize() const { return cfg_.window_size; }
  double trendLambda() const { return cfg_.trend_lambda; }

  // Estimate the signed-acceleration trend using the recent filtered
  // acceleration history. The result is an average jerk over the latest N
  // filtered samples and remains causal by only using current/past values.
  double computeAverageJerk(const std::deque<double>& filtered_acc_history,
                            double dt) const;

  // Convert the recent signed-acceleration trend into a dimensionless gain.
  // gamma ~= 1 keeps the current acceleration level, gamma < 1 damps it, and
  // gamma > 1 allows a modest signed increase in the current direction.
  double computeGamma(const std::deque<double>& filtered_acc_history,
                      double current_accel,
                      double dt,
                      double* j_avg_out = nullptr) const;

  // Predict stage-k acceleration from a frozen horizon-start acceleration and
  // a frozen average jerk, then exponentially damp the whole trend toward 0.
  double predictFrozenTrend(double initial_accel,
                            double frozen_avg_jerk,
                            double dt,
                            int step_idx,
                            double accel_limit,
                            double* damping_out = nullptr) const;

  // Predict the next acceleration sample under one of the supported modes.
  double predictNext(double current_accel,
                     const std::deque<double>& filtered_acc_history,
                     double dt,
                     TrendPredictionMode mode,
                     double decay_gamma,
                     double accel_limit,
                     double* j_avg_out = nullptr,
                     double* gamma_out = nullptr) const;

 private:
  static double clampAbs(double value, double max_abs_value);

  AdaptiveTrendGainConfig cfg_;
};

}  // namespace coni_mpc
