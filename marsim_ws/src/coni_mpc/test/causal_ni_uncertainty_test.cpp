#include "coni_mpc/causal_ni_uncertainty.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <random>

namespace {
namespace ni = coni_mpc::causal_ni;

ni::PredictionSequence zeroPrediction() {
  ni::PredictionSequence prediction;
  prediction.accel_n.fill(Eigen::Vector3d::Zero());
  prediction.omega.fill(0.0);
  prediction.beta.fill(0.0);
  return prediction;
}

ni::PredictorBounds zeroBounds() {
  ni::PredictorBounds bounds;
  bounds.d_accel.fill(0.0);
  bounds.d_omega.fill(0.0);
  bounds.d_beta.fill(0.0);
  bounds.valid = true;
  return bounds;
}

std::array<double, ni::kIntervalCount> filled(double value) {
  std::array<double, ni::kIntervalCount> result;
  result.fill(value);
  return result;
}

Eigen::Vector3d nonInertialTerm(const Eigen::Vector3d& p,
                                const Eigen::Vector3d& v,
                                const Eigen::Vector3d& accel,
                                const Eigen::Vector3d& omega,
                                const Eigen::Vector3d& beta) {
  return -accel - 2.0 * omega.cross(v) - beta.cross(p) -
         omega.cross(omega.cross(p));
}

Eigen::Matrix3d yawRotation(double yaw) {
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

TEST(CausalNiUncertainty, ZeroErrorsForConsistentTuple) {
  ni::PredictionSequence prediction = zeroPrediction();
  prediction.accel_n.fill(Eigen::Vector3d(0.4, -0.2, 0.1));
  prediction.omega.fill(0.3);
  ni::InitialState initial;
  initial.omega = 0.3;

  const ni::TubeResult tube = ni::buildTube(
      prediction, initial, zeroBounds(), filled(2.0), filled(5.0));
  ASSERT_TRUE(tube.valid) << tube.reason;
  for (const ni::SubstepSample& sample : tube.samples) {
    EXPECT_DOUBLE_EQ(sample.r_omega, 0.0);
    EXPECT_DOUBLE_EQ(sample.E_omega, 0.0);
    EXPECT_DOUBLE_EQ(sample.E_theta, 0.0);
    EXPECT_DOUBLE_EQ(sample.E_R, 0.0);
    EXPECT_DOUBLE_EQ(sample.E_Vg, 0.0);
    EXPECT_DOUBLE_EQ(sample.E_G, 0.0);
    EXPECT_DOUBLE_EQ(sample.E_P, 0.0);
    EXPECT_DOUBLE_EQ(sample.E_W, 0.0);
    EXPECT_DOUBLE_EQ(sample.E_q, 0.0);
  }
}

TEST(CausalNiUncertainty, ExactResidualMatchesRandomQuadraticDifference) {
  std::mt19937 rng(1977);
  std::normal_distribution<double> normal(0.0, 0.8);
  auto randomVector = [&]() {
    return Eigen::Vector3d(normal(rng), normal(rng), normal(rng));
  };
  for (int trial = 0; trial < 500; ++trial) {
    const Eigen::Vector3d p = randomVector();
    const Eigen::Vector3d v = randomVector();
    const Eigen::Vector3d accel_hat = randomVector();
    const Eigen::Vector3d accel_error = randomVector();
    const Eigen::Vector3d omega_hat = randomVector();
    const Eigen::Vector3d omega_error = randomVector();
    const Eigen::Vector3d beta_hat = randomVector();
    const Eigen::Vector3d beta_error = randomVector();

    const Eigen::Vector3d direct =
        nonInertialTerm(p, v, accel_hat + accel_error,
                        omega_hat + omega_error, beta_hat + beta_error) -
        nonInertialTerm(p, v, accel_hat, omega_hat, beta_hat);
    const ni::ResidualDiagnostic diagnostic = ni::exactNonInertialResidual(
        p, v, accel_error, omega_hat, omega_error, beta_error);
    EXPECT_NEAR((diagnostic.delta_f - direct).norm(), 0.0, 2.0e-14);
  }
}

TEST(CausalNiUncertainty, NearZeroOmegaUsesStableExactIntegration) {
  ni::PredictionSequence prediction = zeroPrediction();
  prediction.accel_n.fill(Eigen::Vector3d(1.2, -0.7, 0.3));
  prediction.omega.fill(1.0e-12);
  ni::InitialState initial;
  initial.G_world = Eigen::Vector3d(2.0, -1.0, 0.5);
  initial.Vg_world = Eigen::Vector3d(0.4, 0.2, -0.1);
  initial.yaw = 0.6;
  initial.omega = 1.0e-12;

  const ni::TubeResult tube = ni::buildTube(
      prediction, initial, zeroBounds(), filled(0.0), filled(0.0));
  ASSERT_TRUE(tube.valid) << tube.reason;
  const double t = 2.0;
  const Eigen::Vector3d accel_world =
      yawRotation(initial.yaw) * prediction.accel_n[0];
  const Eigen::Vector3d expected_velocity = initial.Vg_world + accel_world * t;
  const Eigen::Vector3d expected_position =
      initial.G_world + initial.Vg_world * t + 0.5 * accel_world * t * t;
  EXPECT_NEAR((tube.samples.back().nominal_Vg_world - expected_velocity).norm(),
              0.0, 5.0e-12);
  EXPECT_NEAR((tube.samples.back().nominal_G_world - expected_position).norm(),
              0.0, 5.0e-12);
}

TEST(CausalNiUncertainty, ConstantYawStageIntegrationMatchesClosedForm) {
  ni::PredictionSequence prediction = zeroPrediction();
  prediction.accel_n.fill(Eigen::Vector3d(0.8, -0.25, 0.2));
  prediction.omega.fill(0.7);
  ni::InitialState initial;
  initial.G_world = Eigen::Vector3d(-0.3, 0.9, 0.4);
  initial.Vg_world = Eigen::Vector3d(0.2, -0.4, 0.1);
  initial.yaw = -0.35;
  initial.omega = 0.7;

  const ni::TubeResult tube = ni::buildTube(
      prediction, initial, zeroBounds(), filled(0.0), filled(0.0));
  ASSERT_TRUE(tube.valid) << tube.reason;
  const double t = 0.1;
  const std::complex<double> z(0.0, prediction.omega[0] * t);
  const std::complex<double> a(prediction.accel_n[0].x(),
                               prediction.accel_n[0].y());
  const std::complex<double> rotation =
      std::exp(std::complex<double>(0.0, initial.yaw));
  const std::complex<double> v0(initial.Vg_world.x(), initial.Vg_world.y());
  const std::complex<double> g0(initial.G_world.x(), initial.G_world.y());
  const std::complex<double> f1 = t * (std::exp(z) - 1.0) / z;
  const std::complex<double> f2 = t * t * (std::exp(z) - 1.0 - z) / (z * z);
  const std::complex<double> expected_v = v0 + rotation * a * f1;
  const std::complex<double> expected_g = g0 + v0 * t + rotation * a * f2;
  const ni::SubstepSample& stage_end = tube.samples[5];
  EXPECT_NEAR(stage_end.nominal_Vg_world.x(), expected_v.real(), 2.0e-14);
  EXPECT_NEAR(stage_end.nominal_Vg_world.y(), expected_v.imag(), 2.0e-14);
  EXPECT_NEAR(stage_end.nominal_G_world.x(), expected_g.real(), 2.0e-14);
  EXPECT_NEAR(stage_end.nominal_G_world.y(), expected_g.imag(), 2.0e-14);
  EXPECT_NEAR(stage_end.nominal_Vg_world.z(), 0.12, 2.0e-14);
  EXPECT_NEAR(stage_end.nominal_G_world.z(), 0.411, 2.0e-14);
}

TEST(CausalNiUncertainty, TubeEnclosesDensePhysicallyConsistentSamples) {
  ni::PredictionSequence prediction = zeroPrediction();
  const double beta = 0.04;
  for (std::size_t j = 0; j < ni::kStageCount; ++j) {
    prediction.accel_n[j] = Eigen::Vector3d(0.3, -0.12, 0.02);
    prediction.omega[j] = 0.2 + beta * 0.1 * static_cast<double>(j);
    prediction.beta[j] = beta;
  }
  ni::InitialState initial;
  initial.G_world = Eigen::Vector3d(0.1, -0.2, 0.0);
  initial.Vg_world = Eigen::Vector3d(0.35, 0.1, 0.0);
  initial.yaw = 0.25;
  initial.omega = prediction.omega[0];
  initial.P_world = Eigen::Vector3d(1.0, 0.5, 0.8);
  initial.W_world = Eigen::Vector3d(0.0, 0.0, 0.0);
  ni::PredictorBounds bounds = zeroBounds();
  bounds.d_accel.fill(0.06);
  bounds.d_omega.fill(0.02);
  const Eigen::Vector3d accel_error(0.03, -0.02, 0.01);
  const Eigen::Vector3d u(0.25, -0.1, 0.0);
  const ni::TubeResult tube = ni::buildTube(
      prediction, initial, bounds, filled(u.norm()), filled(10.0));
  ASSERT_TRUE(tube.valid) << tube.reason;

  Eigen::Vector3d G_true = initial.G_world;
  Eigen::Vector3d Vg_true = initial.Vg_world;
  double yaw_true = initial.yaw;
  Eigen::Vector3d G_nominal = initial.G_world;
  Eigen::Vector3d Vg_nominal = initial.Vg_world;
  double yaw_nominal = initial.yaw;
  Eigen::Vector3d P_true = initial.P_world;
  Eigen::Vector3d W_true = initial.W_world;
  Eigen::Vector3d P_nominal = initial.P_world;
  Eigen::Vector3d W_nominal = initial.W_world;
  const double dt = 1.0e-4;
  const int steps_per_substep = 200;
  for (std::size_t node = 1; node < tube.samples.size(); ++node) {
    for (int n = 0; n < steps_per_substep; ++n) {
      const double time =
          (static_cast<double>(node - 1) * steps_per_substep + n) * dt;
      const std::size_t stage = std::min(
          ni::kIntervalCount - 1,
          static_cast<std::size_t>(time / 0.1));
      const double omega_true = initial.omega + beta * time;
      const Eigen::Matrix3d R_true = yawRotation(yaw_true);
      const Eigen::Vector3d accel_true_world =
          R_true * (prediction.accel_n[stage] + accel_error);
      const Eigen::Matrix3d R_nominal = yawRotation(yaw_nominal);
      // Dense explicit integration is only a test oracle; the certificate is
      // produced analytically by buildTube, not inferred from these samples.
      const Eigen::Vector3d q_true =
          Vg_true + R_true * u +
          omega_true * Eigen::Vector3d::UnitZ().cross(P_true - G_true);
      const Eigen::Vector3d q_nominal =
          Vg_nominal + R_nominal * u +
          prediction.omega[stage] *
              Eigen::Vector3d::UnitZ().cross(P_nominal - G_nominal);
      P_true += dt * W_true;
      W_true += dt * 5.0 * (q_true - W_true);
      P_nominal += dt * W_nominal;
      W_nominal += dt * 5.0 * (q_nominal - W_nominal);
      G_true += dt * Vg_true;
      Vg_true += dt * accel_true_world;
      yaw_true += dt * omega_true;
      G_nominal += dt * Vg_nominal;
      Vg_nominal += dt * R_nominal * prediction.accel_n[stage];
      yaw_nominal += dt * prediction.omega[stage];
    }
    const ni::SubstepSample& sample = tube.samples[node];
    EXPECT_LE((Vg_true - sample.nominal_Vg_world).norm(),
              sample.E_Vg + 2.0e-4);
    EXPECT_LE((G_true - sample.nominal_G_world).norm(), sample.E_G + 2.0e-4);
    EXPECT_LE((P_true - P_nominal).norm(), sample.E_P + 4.0e-3);
    EXPECT_LE((W_true - W_nominal).norm(), sample.E_W + 4.0e-3);
  }
}

TEST(CausalNiUncertainty, InvalidBoundsAndInconsistentInitialRateAreRejected) {
  const ni::PredictionSequence prediction = zeroPrediction();
  ni::InitialState initial;
  ni::PredictorBounds unknown;
  EXPECT_FALSE(ni::buildTube(prediction, initial, unknown, filled(0.0),
                             filled(0.0)).valid);

  ni::PredictorBounds valid = zeroBounds();
  initial.omega = 0.1;
  EXPECT_FALSE(ni::buildTube(prediction, initial, valid, filled(0.0),
                             filled(0.0)).valid);
}

TEST(CausalNiUncertainty, HeldWorldCommandRemovesFutureUgvCoupling) {
  ni::PredictionSequence prediction = zeroPrediction();
  prediction.accel_n.fill(Eigen::Vector3d(0.8, 0.2, 0.0));
  ni::PredictorBounds bounds = zeroBounds();
  bounds.d_accel.fill(0.5);
  bounds.d_omega.fill(0.2);
  bounds.d_beta.fill(0.1);
  ni::TubeConfig config;
  config.fixed_world_command = true;
  const ni::TubeResult tube = ni::buildTube(
      prediction, ni::InitialState(), bounds, filled(2.0), filled(5.0), config);
  ASSERT_TRUE(tube.valid) << tube.reason;
  EXPECT_GT(tube.samples.back().E_G, 0.0);
  EXPECT_DOUBLE_EQ(tube.samples.back().E_P, 0.0);
  EXPECT_DOUBLE_EQ(tube.samples.back().E_W, 0.0);
  EXPECT_DOUBLE_EQ(tube.samples.back().E_q, 0.0);
}

}  // namespace
