#include "coni_mpc/field_hocbf.h"

#include <gtest/gtest.h>

namespace {
using coni_mpc::field_hocbf::KinematicPoint;
using coni_mpc::field_hocbf::KinematicPoints;

TEST(FieldHocbf, PruningSeedsThenEnforcesSeparation) {
  KinematicPoints points;
  for (int i = 0; i < 20; ++i) {
    KinematicPoint point;
    point.position = Eigen::Vector3d(0.01 * i, 0.0, 0.0);
    point.occupancy = 0.5;
    points.push_back(point);
  }
  coni_mpc::field_hocbf::Config config;
  config.seed_count = 2;
  const auto selected = coni_mpc::field_hocbf::prune(
      points, Eigen::Vector3d::Zero(), config);
  ASSERT_EQ(selected.size(), 3u);
  EXPECT_DOUBLE_EQ(selected.front().position.x(), 0.0);
  EXPECT_DOUBLE_EQ(selected[1].position.x(), 0.01);
  EXPECT_DOUBLE_EQ(selected.back().position.x(), 0.15);
}

TEST(FieldHocbf, AffineAssemblyMatchesDirectEvaluation) {
  KinematicPoint point;
  point.position = Eigen::Vector3d(0.2, -0.1, 0.0);
  point.velocity = Eigen::Vector3d(0.1, 0.0, 0.0);
  point.acceleration = Eigen::Vector3d(-0.2, 0.1, 0.0);
  point.occupancy = 0.8;
  const Eigen::Vector3d p(0.5, 0.2, 0.0);
  const Eigen::Vector3d v(0.3, -0.2, 0.1);
  const Eigen::Vector3d a_non(0.1, 0.2, -0.1);
  const Eigen::Vector3d command(-0.4, 0.6, 0.2);
  const auto constraint = coni_mpc::field_hocbf::computeConstraint(
      KinematicPoints{point}, p, v, a_non);
  ASSERT_TRUE(constraint.active);

  const coni_mpc::field_hocbf::Config config;
  const double d_safe = config.d_safe;
  const double gamma1 = config.gamma1;
  const double gamma2 = config.gamma2;
  Eigen::Vector3d delta_p = p - point.position;
  delta_p.z() = 0.0;
  Eigen::Vector3d rv = v - point.velocity;
  rv.z() = 0.0;
  Eigen::Vector3d acceleration =
      Eigen::Vector3d::Constant(5.0).cwiseProduct(command - v) + a_non -
      point.acceleration;
  acceleration.z() = 0.0;
  const double r_eq = std::sqrt(delta_p.squaredNorm() + 1.0e-12);
  const double h = r_eq - d_safe;
  const double hdot = delta_p.dot(rv) / r_eq;
  const double hddot =
      (rv.squaredNorm() + delta_p.dot(acceleration) - hdot * hdot) / r_eq;
  EXPECT_DOUBLE_EQ(constraint.A.z(), 0.0);
  EXPECT_NEAR(constraint.h, h, 1e-12);
  EXPECT_NEAR(constraint.A.dot(command) - constraint.b,
              hddot + (gamma1 + gamma2) * hdot + (gamma1 * gamma2) * h, 1e-12);
}

TEST(FieldHocbf, MultiVoxelAnalyticalLieDerivativesMatchFiniteDifferenceAndStayBounded) {
  KinematicPoints points;
  for (int i = 0; i < 8; ++i) {
    KinematicPoint pt;
    pt.position = Eigen::Vector3d(-1.5 + 0.08 * i, 4.45 + 0.05 * i, 2.0);
    pt.velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    pt.acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);
    pt.occupancy = 0.85;
    points.push_back(pt);
  }
  const coni_mpc::field_hocbf::Config config;
  const Eigen::Vector3d p0(-1.33, 4.85, 2.0);
  const Eigen::Vector3d v0(-0.6, 0.4, 0.0);
  const Eigen::Vector3d a_non(0.1, -0.2, 0.0);
  const Eigen::Vector3d u_cmd(-0.2, 0.8, 0.0);
  const Eigen::Vector3d lambda = Eigen::Vector3d::Constant(5.0);

  const auto c0 = coni_mpc::field_hocbf::computeConstraint(
      points, p0, v0, a_non, config, lambda);
  ASSERT_TRUE(c0.active);
  EXPECT_LT(std::abs(c0.b), 50.0);

  const double dt = 1.0e-5;
  const Eigen::Vector3d a0 = lambda.cwiseProduct(u_cmd - v0) + a_non;
  const Eigen::Vector3d p_plus = p0 + v0 * dt + 0.5 * a0 * dt * dt;
  const Eigen::Vector3d p_minus = p0 - v0 * dt + 0.5 * a0 * dt * dt;
  const auto c_plus = coni_mpc::field_hocbf::computeConstraint(
      points, p_plus, v0 + a0 * dt, a_non, config, lambda);
  const auto c_minus = coni_mpc::field_hocbf::computeConstraint(
      points, p_minus, v0 - a0 * dt, a_non, config, lambda);

  const double hdot_fd = (c_plus.h - c_minus.h) / (2.0 * dt);
  const double hddot_fd = (c_plus.h - 2.0 * c0.h + c_minus.h) / (dt * dt);
  const double psi2_fd =
      hddot_fd + (config.gamma1 + config.gamma2) * hdot_fd +
      config.gamma1 * config.gamma2 * c0.h;
  const double psi2_analytic = c0.A.dot(u_cmd) - c0.b;
  EXPECT_NEAR(psi2_analytic, psi2_fd, 1.0e-5);
}
}  // namespace
