#include "coni_mpc/field_hocbf.h"

#include <gtest/gtest.h>

#include <array>
#include <random>

namespace {
namespace fh = coni_mpc::field_hocbf;

fh::KinematicPoint point(const Eigen::Vector3d& p, double occupancy,
                         std::uint64_t id) {
  fh::KinematicPoint result;
  result.position = p;
  result.occupancy = occupancy;
  result.source_id = id;
  return result;
}

TEST(FieldHocbf, SelectionPreservesAllEligibleRowsAndTrue3dRadius) {
  fh::KinematicPoints points;
  for (int i = 0; i < 50; ++i) {
    points.push_back(point(Eigen::Vector3d(0.01 * i, 0.0, 0.0), 0.01, i));
  }
  points.push_back(point(Eigen::Vector3d(0.0, 0.0, 1.9), 0.8, 50));
  points.push_back(point(Eigen::Vector3d(0.0, 0.0, 2.1), 0.8, 51));
  points.push_back(points.front());
  points.back().source_id = 52;
  points.push_back(point(Eigen::Vector3d(0.0, 0.0, 0.0), 0.0, 53));
  fh::PruneStats stats;
  const auto selected = fh::prune(points, Eigen::Vector3d::Zero(), fh::Config(), &stats);
  ASSERT_EQ(selected.size(), 52u);
  EXPECT_EQ(selected.front().source_id, 0u);
  EXPECT_EQ(selected[50].source_id, 50u);
  EXPECT_EQ(selected[51].source_id, 52u);
  EXPECT_EQ(stats.candidate_count, 54u);
  EXPECT_EQ(stats.eligible_count, 52u);
  EXPECT_EQ(stats.selected_count, 52u);
  EXPECT_EQ(stats.invalid_count, 0u);
  EXPECT_TRUE(stats.input_valid);
}

TEST(FieldHocbf, SelectionReportsInvalidRows) {
  auto valid = point(Eigen::Vector3d::Zero(), 0.5, 0);
  auto invalid = point(Eigen::Vector3d::Zero(), -0.1, 1);
  fh::PruneStats stats;
  const auto selected = fh::prune(fh::KinematicPoints{valid, invalid},
                                  Eigen::Vector3d::Zero(), fh::Config(), &stats);
  EXPECT_TRUE(selected.empty());
  EXPECT_EQ(stats.invalid_count, 1u);
  EXPECT_FALSE(stats.input_valid);
  const auto constraint = fh::computeConstraint(
      fh::KinematicPoints{valid, invalid}, Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  EXPECT_FALSE(constraint.active);
}

TEST(FieldHocbf, ConservativeVoxelCertificateAndUnnormalizedLse) {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> coordinate(-2.0, 2.0);
  for (double voxel_size : {0.05, 0.15, 0.30}) {
    fh::Config config;
    config.voxel_size = voxel_size;
    config.uav_radius = 0.21;
    config.distance_margin = 0.17;
    for (int trial = 0; trial < 100; ++trial) {
      const Eigen::Vector3d p(coordinate(rng), coordinate(rng), coordinate(rng));
      fh::KinematicPoints points;
      for (int i = 0; i < 12; ++i) {
        points.push_back(point(Eigen::Vector3d(coordinate(rng), coordinate(rng),
                                               coordinate(rng)),
                               0.05 + 0.07 * i, i));
      }
      const auto c = fh::computeConstraint(points, p, Eigen::Vector3d::Zero(),
                                           Eigen::Vector3d::Zero(), config);
      ASSERT_TRUE(c.active);
      double exact_clearance = std::numeric_limits<double>::infinity();
      const double half = voxel_size * 0.5;
      for (const auto& voxel : points) {
        const Eigen::Array3d outside =
            ((p - voxel.position).array().abs() - half).max(0.0);
        exact_clearance = std::min(exact_clearance,
                                   outside.matrix().norm() - config.uav_radius);
      }
      EXPECT_LE(c.d_min, exact_clearance + 2.0e-6);
      EXPECT_LE(c.h, c.d_min - config.distance_margin + 1.0e-12);
    }
  }
}

TEST(FieldHocbf, DuplicateDenseAndOccupancyWeightsHaveSpecifiedEffects) {
  fh::Config config;
  const Eigen::Vector3d p(0.7, -0.2, 0.4);
  const auto a = point(Eigen::Vector3d::Zero(), 0.1, 1);
  auto b = a;
  b.source_id = 2;
  b.occupancy = 0.9;
  const auto single = fh::computeConstraint(fh::KinematicPoints{a}, p,
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), config);
  const auto duplicate = fh::computeConstraint(fh::KinematicPoints{a, b}, p,
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), config);
  EXPECT_NEAR(duplicate.h, single.h - config.sigma * std::log(2.0), 1e-12);
  EXPECT_NEAR((duplicate.gradient - single.gradient).norm(), 0.0, 1e-12);

  config.occupancy_alpha = 3.0;
  const auto weighted = fh::computeConstraint(fh::KinematicPoints{a, b}, p,
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), config);
  EXPECT_LT(weighted.h, duplicate.h);
}

TEST(FieldHocbf, ExactAndNearVoxelCenterRemainFiniteAndConservative) {
  fh::Config config;
  config.uav_radius = 0.2;
  config.voxel_size = 0.15;
  const auto voxel = point(Eigen::Vector3d::Zero(), 0.7, 1);
  const std::array<Eigen::Vector3d, 2> queries{{
      Eigen::Vector3d::Zero(), Eigen::Vector3d(1e-12, -2e-12, 3e-12)}};
  for (const Eigen::Vector3d& p : queries) {
    const auto c = fh::computeConstraint(fh::KinematicPoints{voxel}, p,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), config);
    ASSERT_TRUE(c.active);
    EXPECT_TRUE(c.gradient.allFinite());
    EXPECT_TRUE(c.hessian.allFinite());
    const double certificate = p.norm() - config.uav_radius -
        std::sqrt(3.0) * config.voxel_size * 0.5;
    EXPECT_LE(c.d_min, certificate + config.distance_epsilon + 1e-15);
  }
}

TEST(FieldHocbf, GradientAndHessianMatchFiniteDifferencesIncludingZ) {
  fh::KinematicPoints points{
      point(Eigen::Vector3d(-0.4, 0.1, 0.2), 0.3, 1),
      point(Eigen::Vector3d(0.8, -0.3, 0.9), 0.9, 2),
      point(Eigen::Vector3d(0.2, 0.7, -0.5), 0.5, 3)};
  fh::Config config;
  config.occupancy_alpha = 1.7;
  const Eigen::Vector3d p(0.3, -0.1, 0.6);
  const auto c = fh::computeConstraint(points, p, Eigen::Vector3d::Zero(),
                                       Eigen::Vector3d::Zero(), config);
  ASSERT_TRUE(c.active);
  const double eps = 1.0e-5;
  Eigen::Vector3d gradient_fd;
  Eigen::Matrix3d hessian_fd;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
    delta(axis) = eps;
    const auto plus = fh::computeConstraint(points, p + delta,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), config);
    const auto minus = fh::computeConstraint(points, p - delta,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), config);
    gradient_fd(axis) = (plus.h - minus.h) / (2.0 * eps);
    hessian_fd.col(axis) = (plus.gradient - minus.gradient) / (2.0 * eps);
  }
  EXPECT_LT((c.gradient - gradient_fd).norm(), 1.0e-8);
  EXPECT_LT((c.hessian - hessian_fd).norm(), 2.0e-7);
  EXPECT_GT(std::abs(c.gradient.z()), 1.0e-3);
}

TEST(FieldHocbf, LieDerivativesAndAffineAssemblyMatchTrajectoryDifference) {
  fh::KinematicPoint a = point(Eigen::Vector3d(-0.2, 0.4, -0.1), 0.4, 1);
  a.velocity = Eigen::Vector3d(0.1, -0.2, 0.05);
  a.acceleration = Eigen::Vector3d(-0.03, 0.02, 0.01);
  fh::KinematicPoint b = point(Eigen::Vector3d(0.7, -0.1, 0.8), 0.8, 2);
  b.velocity = Eigen::Vector3d(-0.05, 0.03, -0.02);
  b.acceleration = Eigen::Vector3d(0.01, -0.04, 0.02);
  fh::KinematicPoints points{a, b};
  const Eigen::Vector3d p(0.4, 0.2, 0.5);
  const Eigen::Vector3d v(0.3, -0.15, 0.2);
  const Eigen::Vector3d a_non(0.08, -0.03, 0.04);
  const Eigen::Vector3d lambda(4.0, 5.0, 6.0);
  const Eigen::Vector3d command(-0.2, 0.4, 0.1);
  fh::Config config;
  config.occupancy_alpha = 0.8;
  const auto c = fh::computeConstraint(points, p, v, a_non, config, lambda);
  ASSERT_TRUE(c.active);
  const Eigen::Vector3d acceleration = lambda.cwiseProduct(command - v) + a_non;
  const double dt = 2.0e-5;
  auto shifted = [&](double t) {
    fh::KinematicPoints moved = points;
    for (std::size_t i = 0; i < moved.size(); ++i) {
      moved[i].position += moved[i].velocity * t +
                           0.5 * moved[i].acceleration * t * t;
      moved[i].velocity += moved[i].acceleration * t;
    }
    return fh::computeConstraint(moved,
        p + v * t + 0.5 * acceleration * t * t,
        v + acceleration * t, a_non, config, lambda);
  };
  const auto plus = shifted(dt);
  const auto minus = shifted(-dt);
  const double hdot_fd = (plus.h - minus.h) / (2.0 * dt);
  const double hddot_fd = (plus.h - 2.0 * c.h + minus.h) / (dt * dt);
  EXPECT_NEAR(c.hdot, hdot_fd, 2.0e-8);
  EXPECT_NEAR(c.A.dot(command) + c.lf2, hddot_fd, 2.0e-5);
  EXPECT_NEAR(c.A.dot(command) - c.b,
              hddot_fd + (config.gamma1 + config.gamma2) * hdot_fd +
                  config.gamma1 * config.gamma2 * c.h,
              2.0e-5);
}

TEST(FieldHocbf, ZOnlySeparationProducesVerticalConstraint) {
  const auto c = fh::computeConstraint(
      fh::KinematicPoints{point(Eigen::Vector3d(0.0, 0.0, -0.5), 0.7, 1)},
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero());
  ASSERT_TRUE(c.active);
  EXPECT_NEAR(c.A.x(), 0.0, 1e-15);
  EXPECT_NEAR(c.A.y(), 0.0, 1e-15);
  EXPECT_GT(c.A.z(), 0.0);
}

}  // namespace
