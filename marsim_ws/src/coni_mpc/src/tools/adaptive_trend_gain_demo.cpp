#include "coni_mpc/adaptive_trend_gain.hpp"

#include <deque>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

void runScenario(const std::string& name,
                 const std::vector<double>& history,
                 coni_mpc::TrendPredictionMode mode,
                 double decay_gamma) {
  coni_mpc::AdaptiveTrendGainConfig cfg;
  cfg.window_size = 5;
  cfg.alpha = 0.6;
  cfg.gamma_min = 0.90;
  cfg.gamma_max = 1.08;
  cfg.acc_threshold = 0.05;
  cfg.acc_floor = 0.10;
  coni_mpc::AdaptiveTrendGain gain(cfg);

  std::deque<double> hist(history.begin(), history.end());
  const double current_accel = history.back();
  double j_avg = 0.0;
  double gamma = 1.0;
  const double next_accel =
      gain.predictNext(current_accel,
                       hist,
                       0.1,
                       mode,
                       decay_gamma,
                       10.0,
                       &j_avg,
                       &gamma);

  std::cout << "\n[" << name << "] mode=" << coni_mpc::trendPredictionModeName(mode)
            << "\n  a_k    = " << std::fixed << std::setprecision(4) << current_accel
            << "\n  j_avg  = " << j_avg
            << "\n  gamma  = " << gamma
            << "\n  a_next = " << next_accel << "\n";
}

}  // namespace

int main() {
  runScenario("decreasing_accel",
              {1.00, 0.82, 0.67, 0.55, 0.43},
              coni_mpc::TrendPredictionMode::kAdaptiveGamma,
              0.85);
  runScenario("increasing_accel",
              {0.20, 0.34, 0.52, 0.73, 0.95},
              coni_mpc::TrendPredictionMode::kAdaptiveGamma,
              0.85);
  runScenario("zero_crossing",
              {0.16, 0.10, 0.04, -0.01, -0.03},
              coni_mpc::TrendPredictionMode::kAdaptiveGamma,
              0.85);
  runScenario("constant_jerk_demo",
              {0.20, 0.34, 0.52, 0.73, 0.95},
              coni_mpc::TrendPredictionMode::kConstantJerk,
              0.85);
  return 0;
}
