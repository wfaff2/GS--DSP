#include "coni_mpc/field_hocbf.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

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

double euclideanRadiusForRho(double rho, const fh::Config& config) {
  const double shifted = rho + config.distance_epsilon;
  return std::sqrt(shifted * shifted -
                   config.distance_epsilon * config.distance_epsilon);
}

fh::Constraint atRho(double rho, double occupancy,
                     const fh::Config& config = fh::Config()) {
  return fh::computeConstraint(
      fh::KinematicPoints{point(Eigen::Vector3d::Zero(), occupancy, 1)},
      Eigen::Vector3d(euclideanRadiusForRho(rho, config), 0.0, 0.0),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), config);
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
      Eigen::Vector3d direction(coordinate(rng), coordinate(rng), coordinate(rng));
      direction.normalize();
      const double rho = 0.2 + 1.1 * static_cast<double>(trial) / 99.0;
      const Eigen::Vector3d center =
          p + direction * euclideanRadiusForRho(rho, config);
      fh::KinematicPoints points{point(center, 0.7, trial)};
      const auto c = fh::computeConstraint(points, p, Eigen::Vector3d::Zero(),
                                           Eigen::Vector3d::Zero(), config);
      ASSERT_TRUE(c.active);
      const double half = voxel_size * 0.5;
      const Eigen::Array3d outside =
          ((p - center).array().abs() - half).max(0.0);
      const double exact_clearance =
          outside.matrix().norm() - config.uav_radius;
      const double certified_clearance =
          rho - config.uav_radius - std::sqrt(3.0) * voxel_size * 0.5;
      EXPECT_LE(c.d_min, exact_clearance + 2.0e-6);
      EXPECT_NEAR(c.d_min, certified_clearance, 2.0e-12);
      EXPECT_LE(c.h, certified_clearance - config.distance_margin + 1.0e-12);
    }
  }
}

TEST(FieldHocbf, TrustedPenetrationIsStrictlyUnsafe) {
  std::mt19937 rng(19);
  std::normal_distribution<double> normal(0.0, 1.0);
  fh::Config config;
  const double half_diagonal =
      std::sqrt(3.0) * config.voxel_size * 0.5;
  const double penetration_radius =
      config.uav_radius + half_diagonal + 0.5 * config.distance_margin;
  for (int trial = 0; trial < 100; ++trial) {
    Eigen::Vector3d direction(normal(rng), normal(rng), normal(rng));
    direction.normalize();
    const auto c = fh::computeConstraint(
        fh::KinematicPoints{point(
            direction * euclideanRadiusForRho(penetration_radius, config),
            config.occupancy_full_confidence, trial)},
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), config);
    ASSERT_TRUE(c.active);
    EXPECT_LT(c.h, 0.0);
  }
}

TEST(FieldHocbf, DuplicateDenseAndOccupancyWeightsIncludeClearBackground) {
  fh::Config config;
  const Eigen::Vector3d p(0.7, -0.2, 0.4);
  const auto a = point(Eigen::Vector3d::Zero(), 0.7, 1);
  auto b = a;
  b.source_id = 2;
  const auto single = fh::computeConstraint(fh::KinematicPoints{a}, p,
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), config);
  const auto duplicate = fh::computeConstraint(fh::KinematicPoints{a, b}, p,
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), config);
  const double background =
      std::exp(-config.clear_field_value / config.sigma);
  const double expected_duplicate = -config.sigma * std::log(
      2.0 * std::exp(-single.h / config.sigma) - background);
  EXPECT_NEAR(duplicate.h, expected_duplicate, 1e-12);
  EXPECT_LT(duplicate.h, single.h);

  config.occupancy_alpha = 3.0;
  b.occupancy = 0.9;
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

TEST(FieldHocbf, VanishingOccupancyConvergesSmoothlyToClearBackground) {
  fh::Config config;
  config.sigma = 0.2;
  const double tiny_occupancy = 1.0e-6;
  const auto c = atRho(0.05, tiny_occupancy, config);
  EXPECT_NEAR(c.h, config.clear_field_value, 1.0e-10);
  EXPECT_LT(c.gradient.norm(), 1.0e-8);
  EXPECT_LT(c.hessian.norm(), 1.0e-6);

  const auto zero = atRho(0.05, 0.0, config);
  EXPECT_FALSE(zero.active);
  EXPECT_DOUBLE_EQ(zero.h, config.clear_field_value);
  EXPECT_TRUE(zero.A.isZero(0.0));
}

TEST(FieldHocbf, EmptyAndOutsideKernelAreClearAndInactive) {
  fh::Config config;
  const fh::KinematicPoints outside{
      point(Eigen::Vector3d(config.prune_radius + 0.2, 0.0, 0.0), 0.9, 1)};
  const auto direct = fh::computeConstraint(
      outside, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), config);
  fh::PruneStats stats;
  const auto selected = fh::prune(
      outside, Eigen::Vector3d::Zero(), config, &stats);
  const auto empty = fh::computeConstraint(
      selected, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), config);

  EXPECT_TRUE(selected.empty());
  EXPECT_EQ(stats.eligible_count, 0u);
  for (const auto* c : {&direct, &empty}) {
    EXPECT_FALSE(c->active);
    EXPECT_DOUBLE_EQ(c->h, config.clear_field_value);
    EXPECT_DOUBLE_EQ(c->hdot, 0.0);
    EXPECT_DOUBLE_EQ(c->lf2, 0.0);
    EXPECT_TRUE(c->A.isZero(0.0));
    EXPECT_TRUE(c->gradient.isZero(0.0));
    EXPECT_TRUE(c->hessian.isZero(0.0));
  }
}

TEST(FieldHocbf, AddingZeroSupportPointsDoesNotChangeConstraint) {
  fh::Config config;
  config.sigma = 0.2;
  const auto near = point(Eigen::Vector3d(0.6, -0.1, 0.2), 0.8, 1);
  const auto far = point(Eigen::Vector3d(config.prune_radius + 0.3, 0.0, 0.0),
                         0.9, 2);
  const auto zero = point(Eigen::Vector3d(0.1, 0.0, 0.0), 0.0, 3);
  const Eigen::Vector3d p(0.1, 0.2, -0.1);
  const Eigen::Vector3d v(0.2, -0.1, 0.05);
  const Eigen::Vector3d a_non(-0.02, 0.03, 0.01);
  const auto baseline = fh::computeConstraint(
      fh::KinematicPoints{near}, p, v, a_non, config);
  const auto augmented = fh::computeConstraint(
      fh::KinematicPoints{near, far, zero}, p, v, a_non, config);
  ASSERT_TRUE(baseline.active);
  ASSERT_TRUE(augmented.active);
  EXPECT_DOUBLE_EQ(augmented.h, baseline.h);
  EXPECT_DOUBLE_EQ(augmented.hdot, baseline.hdot);
  EXPECT_DOUBLE_EQ(augmented.lf2, baseline.lf2);
  EXPECT_DOUBLE_EQ(augmented.b, baseline.b);
  EXPECT_TRUE(augmented.A.isApprox(baseline.A, 0.0));
  EXPECT_TRUE(augmented.gradient.isApprox(baseline.gradient, 0.0));
  EXPECT_TRUE(augmented.hessian.isApprox(baseline.hessian, 0.0));
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

TEST(FieldHocbf, SpatialCutoffIsC2AtInnerAndOuterBoundaries) {
  fh::Config config;
  config.sigma = 0.4;
  const auto voxel = point(Eigen::Vector3d::Zero(), 0.8, 1);
  // At the knots the third derivative is only one-sided; use a small centered
  // step to test the C2 quantities without assuming C3 truncation error.
  const double fd_step = 1.0e-5;

  const auto evaluate = [&](const Eigen::Vector3d& p) {
    return fh::computeConstraint(
        fh::KinematicPoints{voxel}, p, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), config);
  };
  const auto check_boundary = [&](double rho, bool outer) {
    const Eigen::Vector3d p(
        euclideanRadiusForRho(rho, config), 0.0, 0.0);
    const auto center = evaluate(p);
    Eigen::Vector3d gradient_fd;
    Eigen::Matrix3d hessian_fd;
    for (int axis = 0; axis < 3; ++axis) {
      Eigen::Vector3d delta = Eigen::Vector3d::Zero();
      delta(axis) = fd_step;
      const auto plus = evaluate(p + delta);
      const auto minus = evaluate(p - delta);
      gradient_fd(axis) = (plus.h - minus.h) / (2.0 * fd_step);
      hessian_fd.col(axis) =
          (plus.gradient - minus.gradient) / (2.0 * fd_step);
    }
    EXPECT_LT((center.gradient - gradient_fd).norm(), 2.0e-6);
    EXPECT_LT((center.hessian - hessian_fd).norm(), 3.0e-4);

    const double rho_delta = 5.0e-5;
    const auto inside = atRho(rho - rho_delta, 0.8, config);
    const auto outside = atRho(rho + rho_delta, 0.8, config);
    EXPECT_LT(std::abs(inside.h - outside.h), 2.0e-4);
    EXPECT_LT((inside.gradient - outside.gradient).norm(), 2.0e-3);
    EXPECT_LT((inside.hessian - outside.hessian).norm(), 2.0e-2);
    if (outer) {
      EXPECT_FALSE(center.active);
      EXPECT_DOUBLE_EQ(center.h, config.clear_field_value);
      EXPECT_TRUE(center.gradient.isZero(0.0));
      EXPECT_TRUE(center.hessian.isZero(0.0));
    } else {
      EXPECT_TRUE(center.active);
    }
  };

  const double inner = config.prune_radius - config.cutoff_width;
  check_boundary(inner, false);
  // Also exercise analytic cutoff derivatives away from either knot.
  check_boundary(inner + 0.37 * config.cutoff_width, false);
  check_boundary(config.prune_radius, true);
}

TEST(FieldHocbf, PruneAndComputeRemainContinuousAcrossOuterCutoff) {
  fh::Config config;
  config.sigma = 0.4;
  const auto voxel = point(Eigen::Vector3d::Zero(), 0.8, 1);
  for (double offset : {-1.0e-4, 0.0, 1.0e-4}) {
    const double rho = config.prune_radius + offset;
    const Eigen::Vector3d p(euclideanRadiusForRho(rho, config), 0.0, 0.0);
    const auto direct = fh::computeConstraint(
        fh::KinematicPoints{voxel}, p, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), config);
    const auto selected = fh::prune(
        fh::KinematicPoints{voxel}, p, config);
    const auto pruned = fh::computeConstraint(
        selected, p, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), config);
    EXPECT_NEAR(pruned.h, direct.h, 1.0e-12);
    EXPECT_LT((pruned.gradient - direct.gradient).norm(), 1.0e-10);
    EXPECT_LT((pruned.hessian - direct.hessian).norm(), 1.0e-8);
    if (offset >= 0.0) {
      EXPECT_TRUE(selected.empty());
      EXPECT_FALSE(pruned.active);
      EXPECT_DOUBLE_EQ(pruned.h, config.clear_field_value);
    } else {
      EXPECT_EQ(selected.size(), 1u);
    }
  }
}

TEST(FieldHocbf, LieDerivativesAndAffineAssemblyMatchTrajectoryDifference) {
  fh::KinematicPoint a = point(Eigen::Vector3d(-0.2, 0.4, -0.1), 0.4, 1);
  a.velocity = Eigen::Vector3d(0.1, -0.2, 0.05);
  a.acceleration = Eigen::Vector3d(-0.03, 0.02, 0.01);
  // This point is in the compact-kernel transition band, so the trajectory
  // difference checks include the cutoff's first and second derivatives.
  fh::KinematicPoint b = point(Eigen::Vector3d(2.1, -0.1, 0.8), 0.8, 2);
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

TEST(FieldHocbf, DistinctHeightLayersKeepTheirTrue3dGeometry) {
  fh::Config config;
  fh::KinematicPoints candidates{
      point(Eigen::Vector3d(0.5, 0.0, 0.2), 0.8, 10),
      point(Eigen::Vector3d(0.5, 0.0, 1.2), 0.8, 11)
  };
  fh::PruneStats stats;
  const auto selected = fh::prune(
      candidates, Eigen::Vector3d::Zero(), config, &stats);
  ASSERT_EQ(selected.size(), 2u);
  EXPECT_EQ(selected.front().source_id, 10u);
  EXPECT_EQ(selected.back().source_id, 11u);

  const auto low = fh::computeConstraint(
      fh::KinematicPoints{candidates[0]}, Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), config);
  const auto high = fh::computeConstraint(
      fh::KinematicPoints{candidates[1]}, Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), config);
  const auto combined = fh::computeConstraint(
      selected, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), config);
  ASSERT_TRUE(low.active);
  ASSERT_TRUE(high.active);
  ASSERT_TRUE(combined.active);
  EXPECT_NE(low.h, high.h);
  EXPECT_GT(std::abs(low.gradient.z()), 1.0e-3);
  EXPECT_GT(std::abs(high.gradient.z()), 1.0e-3);
  EXPECT_FALSE(low.gradient.isApprox(high.gradient, 1.0e-6));
  EXPECT_LT(combined.h, std::min(low.h, high.h));
}

TEST(FieldHocbf, InvalidConfigurationIsRejected) {
  fh::Config base;
  std::vector<fh::Config> invalid;
  auto add = [&](auto mutation) {
    fh::Config config = base;
    mutation(config);
    invalid.push_back(config);
  };
  add([](fh::Config& c) { c.sigma = 0.0; });
  add([](fh::Config& c) { c.distance_margin = -1.0; });
  add([](fh::Config& c) { c.uav_radius = -1.0; });
  add([](fh::Config& c) { c.voxel_size = 0.0; });
  add([](fh::Config& c) { c.distance_epsilon = 0.0; });
  add([](fh::Config& c) { c.occupancy_alpha = -1.0; });
  add([](fh::Config& c) { c.gamma1 = 0.0; });
  add([](fh::Config& c) { c.gamma2 = 0.0; });
  add([](fh::Config& c) { c.prune_radius = 0.0; });
  add([](fh::Config& c) { c.cutoff_width = 0.0; });
  add([](fh::Config& c) { c.cutoff_width = c.prune_radius; });
  add([](fh::Config& c) { c.occupancy_full_confidence = 0.0; });
  add([](fh::Config& c) { c.occupancy_full_confidence = 1.1; });
  add([](fh::Config& c) { c.clear_field_value = 0.0; });
  add([](fh::Config& c) {
    c.clear_field_value = std::numeric_limits<double>::quiet_NaN();
  });

  const fh::KinematicPoints points{
      point(Eigen::Vector3d(0.4, 0.0, 0.0), 0.8, 1)};
  for (const auto& config : invalid) {
    EXPECT_FALSE(fh::validConfig(config));
    const auto c = fh::computeConstraint(
        points, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), config);
    EXPECT_FALSE(c.active);
    EXPECT_TRUE(c.A.isZero(0.0));
  }
}

}  // namespace
