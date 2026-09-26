#include "coni_mpc/field_hocbf.h"

#include <gtest/gtest.h>

namespace {
using coni_mpc::field_hocbf::KinematicPoint;
using coni_mpc::field_hocbf::KinematicPoints;

TEST(FieldHocbf, PruningSeedsThenEnforcesSeparation) {
  KinematicPoints points;
  for (int i = 0; i < 15; ++i) {
    KinematicPoint point;
    point.position = Eigen::Vector3d(0.01 * i, 0.0, 0.0);
    point.occupancy = 0.5;
    points.push_back(point);
  }
  const auto selected = coni_mpc::field_hocbf::prune(
      points, Eigen::Vector3d::Zero());
  ASSERT_EQ(selected.size(), 10u);
  EXPECT_DOUBLE_EQ(selected.front().position.x(), 0.0);
  EXPECT_DOUBLE_EQ(selected.back().position.x(), 0.09);
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

  const double sigma = 0.45;
  const double inv_sigma2 = 1.0 / (sigma * sigma);
  const Eigen::Vector3d d = p - point.position;
  const Eigen::Vector3d rv = v - point.velocity;
  const double phi = point.occupancy *
      std::exp(-0.5 * d.squaredNorm() * inv_sigma2);
  const double h = 1.12 - phi;
  const double hdot = phi * d.dot(rv) * inv_sigma2;
  const Eigen::Vector3d acceleration =
      Eigen::Vector3d::Constant(5.0).cwiseProduct(command - v) + a_non;
  const double hddot = phi *
      ((rv.squaredNorm() + d.dot(acceleration - point.acceleration)) *
           inv_sigma2 -
       std::pow(d.dot(rv), 2) * inv_sigma2 * inv_sigma2);
  EXPECT_NEAR(constraint.A.dot(command) - constraint.b,
              hddot + 8.0 * hdot + 16.0 * h, 1e-12);
}
}  // namespace
