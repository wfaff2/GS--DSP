#include "coni_mpc/field_derivative_cert.h"
#include "coni_mpc/recursive_safety_supervisor.h"

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <limits>
#include <random>

namespace {

namespace cert = coni_mpc::field_derivative_cert;
namespace fh = coni_mpc::field_hocbf;
namespace rss = coni_mpc::recursive_safety_supervisor;

fh::KinematicPoint point(const Eigen::Vector3d& position, double occupancy,
                         std::uint64_t id) {
  fh::KinematicPoint result;
  result.position = position;
  result.occupancy = occupancy;
  result.source_id = id;
  return result;
}

cert::FieldSnapshot snapshot() {
  cert::FieldSnapshot result;
  result.id = 42;
  result.persistent_static = true;
  result.coverage_certified = true;
  result.config.sigma = 0.15;
  result.config.occupancy_alpha = 0.7;
  result.points = {
      point(Eigen::Vector3d(-0.5, 0.1, 0.2), 0.4, 1),
      point(Eigen::Vector3d(0.8, -0.3, 0.7), 0.9, 2),
      point(Eigen::Vector3d(1.7, 0.6, -0.2), 0.3, 3),
      point(Eigen::Vector3d(4.0, 0.0, 0.0), 0.8, 4)};
  return result;
}

double spectralNorm(const Eigen::Matrix3d& matrix) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
      0.5 * (matrix + matrix.transpose()));
  return solver.eigenvalues().cwiseAbs().maxCoeff();
}

TEST(FieldDerivativeCert, ExactFieldAgreesWithAuthoritativeC2Evaluator) {
  const cert::FieldSnapshot field = snapshot();
  const Eigen::Vector3d P(0.2, -0.1, 0.35);
  const Eigen::Vector3d W(0.3, -0.2, 0.1);
  const Eigen::Vector3d q(-0.1, 0.25, 0.05);
  const cert::Evaluation evaluated = cert::evaluate(field, P, 0.0);
  ASSERT_TRUE(evaluated.valid) << evaluated.reason;
  EXPECT_TRUE(evaluated.metadata_valid);

  const fh::Constraint baseline = fh::computeConstraint(
      field.points, P, W, Eigen::Vector3d::Zero(), field.config,
      Eigen::Vector3d::Constant(5.0));
  ASSERT_TRUE(baseline.active);
  EXPECT_NEAR(evaluated.h, baseline.h, 2e-13);
  EXPECT_LT((evaluated.g - baseline.gradient).norm(), 2e-13);
  EXPECT_LT((evaluated.H - baseline.hessian).norm(), 2e-12);

  const double psi = cert::exactPsi2(evaluated, W, q, 5.0,
                                     field.config.gamma1,
                                     field.config.gamma2);
  EXPECT_NEAR(psi, baseline.A.dot(q) - baseline.b, 2e-12);
}

TEST(FieldDerivativeCert, AnalyticBallBoundsDominateSampledValues) {
  const cert::FieldSnapshot field = snapshot();
  const Eigen::Vector3d center(0.2, -0.1, 0.35);
  const double radius = 0.24;
  const cert::Evaluation bound = cert::evaluate(field, center, radius);
  ASSERT_TRUE(bound.valid) << bound.reason;

  std::mt19937 generator(9);
  std::normal_distribution<double> normal(0.0, 1.0);
  std::uniform_real_distribution<double> radial(0.0, 1.0);
  for (int sample = 0; sample < 300; ++sample) {
    Eigen::Vector3d direction(normal(generator), normal(generator),
                              normal(generator));
    direction.normalize();
    const Eigen::Vector3d P =
        center + direction * radius * std::cbrt(radial(generator));
    const cert::Evaluation value = cert::evaluate(field, P, 0.0);
    ASSERT_TRUE(value.valid) << value.reason;
    EXPECT_LE(value.g.norm(), bound.G * (1.0 + 1e-12));
    EXPECT_LE(spectralNorm(value.H), bound.H_bound * (1.0 + 1e-12));
    EXPECT_LE(spectralNorm(value.H - bound.H),
              bound.T * (P - center).norm() * (1.0 + 1e-12) + 1e-12);
    EXPECT_LE(value.g.norm(), bound.L * (1.0 + 1e-12));
  }
}

TEST(FieldDerivativeCert, MetadataIsSeparateAndOverflowInvalidatesResult) {
  cert::FieldSnapshot field = snapshot();
  field.coverage_certified = false;
  const cert::Evaluation ordinary =
      cert::evaluate(field, Eigen::Vector3d::Zero(), 0.1);
  EXPECT_TRUE(ordinary.valid) << ordinary.reason;
  EXPECT_FALSE(ordinary.metadata_valid);

  field.config.occupancy_alpha = 1e308;
  field.config.clear_field_value = 100.0;
  const cert::Evaluation overflow =
      cert::evaluate(field, Eigen::Vector3d::Zero(), 0.1);
  EXPECT_FALSE(overflow.valid);
  EXPECT_FALSE(std::isfinite(overflow.L));
}

TEST(FieldDerivativeCert, RobustMarginIsExactlyZeroForZeroErrors) {
  const cert::FieldSnapshot field = snapshot();
  const cert::Evaluation evaluated =
      cert::evaluate(field, Eigen::Vector3d(0.2, -0.1, 0.35), 0.0);
  ASSERT_TRUE(evaluated.valid);
  const cert::RobustMarginResult zero = cert::robustMargin(
      evaluated, Eigen::Vector3d(0.3, 0.1, -0.1),
      Eigen::Vector3d(-0.2, 0.4, 0.2), 0.0, 0.0, 0.0, 5.0, 5.0,
      5.0);
  ASSERT_TRUE(zero.valid) << zero.reason;
  EXPECT_DOUBLE_EQ(zero.rho, 0.0);

  const double E_P = 0.12;
  const double E_W = 0.07;
  const double E_q = 0.09;
  const cert::Evaluation ball = cert::evaluate(
      field, Eigen::Vector3d(0.2, -0.1, 0.35), E_P);
  ASSERT_TRUE(ball.valid);
  const Eigen::Vector3d W(0.3, 0.1, -0.1);
  const Eigen::Vector3d q(-0.2, 0.4, 0.2);
  const cert::RobustMarginResult nonzero = cert::robustMargin(
      ball, W, q, E_P, E_W, E_q, 5.0, 5.0, 5.0);
  ASSERT_TRUE(nonzero.valid) << nonzero.reason;
  const double expected =
      5.0 * (ball.g.norm() * (E_q + E_W) +
             ball.H_bound * E_P * ((q - W).norm() + E_q + E_W)) +
      ball.H_bound * (2.0 * W.norm() * E_W + E_W * E_W) +
      ball.T * E_P * W.squaredNorm() +
      10.0 * (ball.g.norm() * E_W +
              ball.H_bound * E_P * (W.norm() + E_W)) +
      25.0 * ball.G * E_P;
  EXPECT_DOUBLE_EQ(nonzero.rho, expected);

  const cert::RobustMarginResult uncovered = cert::robustMargin(
      ball, W, q, E_P + 0.01, E_W, E_q, 5.0, 5.0, 5.0);
  EXPECT_FALSE(uncovered.valid);
}

TEST(RecursiveSafetySupervisor, ExplicitFreezeAndNonzeroVelocityBrake) {
  cert::FieldSnapshot field;
  field.id = 7;
  field.persistent_static = true;
  field.coverage_certified = true;
  rss::Config config;
  config.v_max_xy = 2.0;
  config.v_max_z = 1.0;
  config.a_max_xy = 4.0;
  config.a_max_z = 3.0;
  rss::Supervisor supervisor(config);

  const Eigen::Vector3d W(0.45, -0.1, 0.2);
  const rss::Output before = supervisor.run(
      field, Eigen::Vector3d::Zero(), W, Eigen::Vector3d::Zero());
  EXPECT_FALSE(before.certified_safe);
  EXPECT_EQ(before.status, rss::Status::kUncertifiedMissingCertificate);
  EXPECT_TRUE(before.q.isZero(0.0));

  std::string reason;
  ASSERT_TRUE(supervisor.freezeCertifiedSnapshot(field, &reason)) << reason;
  const rss::Output brake = supervisor.run(
      field, Eigen::Vector3d::Zero(), W, Eigen::Vector3d::Zero());
  ASSERT_TRUE(brake.certified_safe) << brake.reason;
  EXPECT_FALSE(brake.backup_used);
  EXPECT_TRUE(brake.q.isZero(0.0));
  EXPECT_TRUE(brake.residuals.allCandidateConstraintsNonnegative());
  EXPECT_GE(brake.braking_reserve, config.eta);
}

TEST(RecursiveSafetySupervisor, RayOutputSatisfiesEveryConstraint) {
  cert::FieldSnapshot field;
  field.id = 8;
  field.persistent_static = true;
  field.coverage_certified = true;
  rss::Supervisor supervisor;
  ASSERT_TRUE(supervisor.freezeCertifiedSnapshot(field));

  const Eigen::Vector3d W(0.25, 0.0, 0.1);
  const Eigen::Vector3d proposal(20.0, -15.0, 8.0);
  const rss::Output output = supervisor.run(
      field, Eigen::Vector3d::Zero(), W, proposal);
  ASSERT_TRUE(output.certified_safe) << output.reason;
  EXPECT_TRUE(output.backup_used);
  EXPECT_GE(output.mu, 0.0);
  EXPECT_LT(output.mu, 1.0);
  EXPECT_TRUE(output.q.isApprox(output.mu * proposal, 1e-14));
  EXPECT_TRUE(output.residuals.allCandidateConstraintsNonnegative());
}

TEST(RecursiveSafetySupervisor, NeverClaimsSafetyForMismatchOrOutsideK) {
  cert::FieldSnapshot field;
  field.id = 9;
  field.persistent_static = true;
  field.coverage_certified = true;
  rss::Supervisor supervisor;
  ASSERT_TRUE(supervisor.freezeCertifiedSnapshot(field));

  cert::FieldSnapshot changed = field;
  changed.id = 10;
  const rss::Output mismatch = supervisor.run(
      changed, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.1, 0.0, 0.0));
  EXPECT_FALSE(mismatch.certified_safe);
  EXPECT_TRUE(mismatch.q.isZero(0.0));
  EXPECT_EQ(mismatch.status, rss::Status::kUncertifiedSnapshotMismatch);

  // Reusing an ID cannot mutate the explicitly frozen field certificate.
  cert::FieldSnapshot reused_id = field;
  reused_id.points.push_back(
      point(Eigen::Vector3d::Zero(), 1.0, 100));
  const rss::Output reused = supervisor.run(
      reused_id, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero());
  ASSERT_TRUE(reused.certified_safe) << reused.reason;
  EXPECT_DOUBLE_EQ(reused.h, field.config.clear_field_value);

  const rss::Output outside = supervisor.run(
      field, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.9, 0.0, 0.0),
      Eigen::Vector3d::Zero());
  EXPECT_FALSE(outside.certified_safe);
  EXPECT_TRUE(outside.q.isZero(0.0));
  EXPECT_EQ(outside.status, rss::Status::kUncertifiedOutsideInvariantSet);
}

TEST(WorldPlantConsistency, CurrentFrameCommandCancelsNonInertialTerms) {
  std::mt19937 generator(27);
  std::uniform_real_distribution<double> scalar(-2.0, 2.0);
  constexpr double lambda = 5.0;
  for (int sample = 0; sample < 200; ++sample) {
    const double yaw = scalar(generator);
    const Eigen::Matrix3d R =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d omega(0.0, 0.0, scalar(generator));
    const Eigen::Vector3d beta(0.0, 0.0, scalar(generator));
    const Eigen::Vector3d p(scalar(generator), scalar(generator),
                            scalar(generator));
    const Eigen::Vector3d v(scalar(generator), scalar(generator),
                            scalar(generator));
    const Eigen::Vector3d u(scalar(generator), scalar(generator),
                            scalar(generator));
    const Eigen::Vector3d accel_g_n(scalar(generator), scalar(generator),
                                    scalar(generator));
    const Eigen::Vector3d Vg(scalar(generator), scalar(generator),
                             scalar(generator));

    const Eigen::Vector3d v_dot =
        lambda * (u - v) - accel_g_n - 2.0 * omega.cross(v) -
        beta.cross(p) - omega.cross(omega.cross(p));
    // Differentiate W=Vg+R(v+omega x p), using Rdot*x=R(omega x x).
    const Eigen::Vector3d W_dot =
        R * accel_g_n +
        R * (omega.cross(v + omega.cross(p)) + v_dot +
             beta.cross(p) + omega.cross(v));
    const Eigen::Vector3d W = Vg + R * (v + omega.cross(p));
    const Eigen::Vector3d q = Vg + R * (u + omega.cross(p));
    EXPECT_LT((W_dot - lambda * (q - W)).norm(), 2e-14);
  }
}

TEST(WorldPlantConsistency, ConstantCommandTransitionMatchesMatrixExponential) {
  std::mt19937 generator(31);
  std::uniform_real_distribution<double> scalar(-3.0, 3.0);
  constexpr double tau = 0.2;
  constexpr double dt = 0.02;
  const double alpha = std::exp(-dt / tau);
  const double c = tau * (1.0 - alpha);
  for (int sample = 0; sample < 100; ++sample) {
    const Eigen::Vector3d P(scalar(generator), scalar(generator),
                            scalar(generator));
    const Eigen::Vector3d W(scalar(generator), scalar(generator),
                            scalar(generator));
    const Eigen::Vector3d q(scalar(generator), scalar(generator),
                            scalar(generator));
    const Eigen::Vector3d P_expected = P + c * W + (dt - c) * q;
    const Eigen::Vector3d W_expected = alpha * W + (1.0 - alpha) * q;

    Eigen::Matrix<double, 7, 7> dynamics =
        Eigen::Matrix<double, 7, 7>::Zero();
    dynamics.block<3, 3>(0, 3).setIdentity();
    dynamics.block<3, 3>(3, 3) =
        -(1.0 / tau) * Eigen::Matrix3d::Identity();
    dynamics.block<3, 1>(3, 6) = q / tau;
    Eigen::Matrix<double, 7, 1> state;
    state << P, W, 1.0;
    const Eigen::Matrix<double, 7, 1> propagated =
        (dt * dynamics).exp() * state;
    EXPECT_LT((propagated.head<3>() - P_expected).norm(), 3e-15);
    EXPECT_LT((propagated.segment<3>(3) - W_expected).norm(), 3e-15);
    EXPECT_DOUBLE_EQ(propagated(6), 1.0);
  }
}

}  // namespace
