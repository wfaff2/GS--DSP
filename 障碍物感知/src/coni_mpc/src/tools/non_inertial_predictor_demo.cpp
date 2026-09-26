#include "coni_mpc/non_inertial_predictor.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <string>

namespace {

coni_mpc::NonInertialPredictor makePredictor(int np, double dt) {
  const coni_mpc::AdaptiveTrendGainConfig trend_cfg;
  return coni_mpc::NonInertialPredictor(
      np,
      dt,
      0,
      0.2,
      0.8,
      1e-3,
      1.0,
      1.0,
      10.0,
      10.0,
      100.0,
      100.0,
      0,
      coni_mpc::TrendPredictionMode::kDecayOnly,
      trend_cfg,
      trend_cfg,
      false,
      0.0,
      0.0,
      0.0,
      0.0,
      1.0,
      0.0);
}

bool allFinite(const coni_mpc::NonInertialVector2Sequence& seq) {
  for (double v : seq.omega_seq) {
    if (!std::isfinite(v)) return false;
  }
  for (double v : seq.beta_seq) {
    if (!std::isfinite(v)) return false;
  }
  for (const Eigen::Vector2d& v : seq.acc_seq) {
    if (!v.allFinite()) return false;
  }
  return true;
}

void printCase(const std::string& name,
               const coni_mpc::NonInertialPredictor& predictor,
               double dt,
               int np,
               const Eigen::Vector2d& acc,
               const Eigen::Vector2d& jerk,
               double omega,
               double beta) {
  const coni_mpc::PredictorParams params;
  const double horizon_time = static_cast<double>(np) * dt;
  const double eta_acc_x =
      predictor.computeDampingGain(acc.x(), jerk.x(), horizon_time, params);
  const double eta_acc_y =
      predictor.computeDampingGain(acc.y(), jerk.y(), horizon_time, params);
  const double eta_omega =
      predictor.computeDampingGain(omega, beta, horizon_time, params);
  const auto seq = predictor.generateDampedVaryingCurvatureRollout(
      dt, np, acc, jerk, omega, beta, params);
  const std::size_t last = seq.omega_seq.empty() ? 0u : seq.omega_seq.size() - 1u;

  std::cout << name
            << " eta_acc=(" << eta_acc_x << ", " << eta_acc_y << ")"
            << " eta_omega=" << eta_omega
            << " acc_end=(" << seq.acc_seq[last].x() << ", "
            << seq.acc_seq[last].y() << ")"
            << " omega_end=" << seq.omega_seq[last]
            << " beta_end=" << seq.beta_seq[last]
            << " finite=" << (allFinite(seq) ? "true" : "false")
            << '\n';
}

}  // namespace

int main() {
  const double dt = 0.01;
  const int np = 10;
  const auto predictor = makePredictor(np, dt);

  printCase("Case A",
            predictor,
            dt,
            np,
            Eigen::Vector2d(0.2, 0.2),
            Eigen::Vector2d(-4.0, -4.0),
            1.0,
            -8.0);
  printCase("Case B",
            predictor,
            dt,
            np,
            Eigen::Vector2d(0.2, 0.2),
            Eigen::Vector2d(2.0, 2.0),
            1.0,
            3.0);
  printCase("Case C",
            predictor,
            dt,
            np,
            Eigen::Vector2d::Zero(),
            Eigen::Vector2d(2.0, 2.0),
            0.0,
            3.0);
  return 0;
}
