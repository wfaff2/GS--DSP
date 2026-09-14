#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include <gtest/gtest.h>

#include "coni_mpc/depth_cbf/depth_cbf_regressor.h"

namespace coni_mpc {
namespace depth_cbf {
namespace {

PointCloud makePlane(double x) {
  PointCloud cloud;
  for (double y = -1.5; y <= 1.5 + 1e-9; y += 0.05) {
    for (double z = -1.5; z <= 1.5 + 1e-9; z += 0.05) {
      cloud.emplace_back(static_cast<float>(x), static_cast<float>(y),
                         static_cast<float>(z));
    }
  }
  return cloud;
}

PointCloud makeCylinder(double radius) {
  PointCloud cloud;
  constexpr double kPi = 3.14159265358979323846;
  for (int i = 0; i < 720; ++i) {
    const double angle = 2.0 * kPi * i / 720.0;
    for (double z = -1.0; z <= 1.0 + 1e-9; z += 0.05) {
      cloud.emplace_back(static_cast<float>(radius * std::cos(angle)),
                         static_cast<float>(radius * std::sin(angle)),
                         static_cast<float>(z));
    }
  }
  return cloud;
}

PointCloud makeCorner(double wall_coordinate) {
  PointCloud cloud = makePlane(wall_coordinate);
  for (double x = -1.0; x <= 1.5 + 1e-9; x += 0.05) {
    for (double z = -1.5; z <= 1.5 + 1e-9; z += 0.05) {
      cloud.emplace_back(static_cast<float>(x),
                         static_cast<float>(wall_coordinate),
                         static_cast<float>(z));
    }
  }
  return cloud;
}

RegressionConfig testConfig() {
  RegressionConfig config;
  config.query_mesh_half_width_steps = 2;
  config.query_mesh_spacing = 0.05;
  config.d_safe = 0.15;
  config.nearest_neighbor_max_distance = 0.75;
  config.max_condition_number = 1e9;
  config.max_rmse = 0.08;
  config.max_abs_error = 0.25;
  return config;
}

void writeLineCsv(const std::string& path, const PointCloud& cloud,
                  const DepthCbfRegressor& regressor,
                  const LocalBarrier& barrier, int axis) {
  std::ofstream stream(path);
  stream << "offset,h_raw,h_fit,gradient_axis\n";
  for (double offset = -0.2; offset <= 0.2 + 1e-12; offset += 0.01) {
    Eigen::Vector3d point = barrier.center_N;
    point(axis) += offset;
    bool within_support = false;
    const double raw = regressor.rawBarrierValue(
        cloud, point, nullptr, &within_support);
    ASSERT_TRUE(within_support);
    stream << offset << ',' << raw << ',' << barrier.evaluate(point) << ','
           << barrier.gradient(point)(axis) << '\n';
  }
}

void expectDifferentialConsistency(const LocalBarrier& barrier) {
  const Eigen::Vector3d point =
      barrier.center_N + Eigen::Vector3d(0.013, -0.019, 0.011);
  const double step = 1e-5;
  Eigen::Vector3d finite_gradient;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d plus = point;
    Eigen::Vector3d minus = point;
    plus(axis) += step;
    minus(axis) -= step;
    finite_gradient(axis) =
        (barrier.evaluate(plus) - barrier.evaluate(minus)) / (2.0 * step);
  }
  EXPECT_TRUE(finite_gradient.isApprox(barrier.gradient(point), 1e-8));

  Eigen::Matrix3d finite_hessian;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d plus = point;
    Eigen::Vector3d minus = point;
    plus(axis) += step;
    minus(axis) -= step;
    finite_hessian.col(axis) =
        (barrier.gradient(plus) - barrier.gradient(minus)) / (2.0 * step);
  }
  EXPECT_TRUE(finite_hessian.isApprox(barrier.hessian(), 1e-8));
  EXPECT_TRUE(barrier.A.isApprox(barrier.A.transpose(), 1e-12));
}

void fitAndCheck(const PointCloud& cloud, const Eigen::Vector3d& center,
                 const std::string& csv_path, int line_axis,
                 Eigen::Vector3d* gradient) {
  const RegressionConfig config = testConfig();
  DepthCbfRegressor regressor(config);
  LocalBarrier barrier;
  RegressionDiagnostics diagnostics;
  std::string error;
  ASSERT_TRUE(regressor.fit(cloud, center, ros::Time(123.0), &barrier,
                            &diagnostics, &error))
      << error;
  EXPECT_TRUE(barrier.valid);
  EXPECT_EQ(barrier.rank, 10);
  EXPECT_EQ(barrier.center_N, center);
  EXPECT_EQ(barrier.stamp, ros::Time(123.0));
  EXPECT_LT(barrier.rmse, config.max_rmse);
  EXPECT_LT(barrier.max_abs_error, config.max_abs_error);
  EXPECT_EQ(diagnostics.query_points_N.size(), 125U);
  EXPECT_EQ(diagnostics.nearest_points_N.size(), 125U);

  bool within_support = false;
  const double raw_center = regressor.rawBarrierValue(
      cloud, center, nullptr, &within_support);
  ASSERT_TRUE(within_support);
  EXPECT_NEAR(barrier.evaluate(center), raw_center, 0.05);
  std::cout << csv_path << ": rmse=" << barrier.rmse
            << ", max_abs_error=" << barrier.max_abs_error
            << ", condition_number=" << barrier.condition_number
            << ", h_raw_center=" << raw_center
            << ", h_fit_center=" << barrier.evaluate(center) << '\n';
  expectDifferentialConsistency(barrier);
  writeLineCsv(csv_path, cloud, regressor, barrier, line_axis);
  *gradient = barrier.gradient(center);
}

TEST(DepthCbfRegressorTest, PlaneWall) {
  const PointCloud cloud = makePlane(1.0);
  Eigen::Vector3d gradient;
  fitAndCheck(cloud, Eigen::Vector3d(0.6, 0.0, 0.0),
              "/tmp/depth_cbf_plane_line.csv", 0, &gradient);
  EXPECT_LT(gradient.x(), -0.6);
  EXPECT_NEAR(gradient.y(), 0.0, 0.03);
  EXPECT_NEAR(gradient.z(), 0.0, 0.03);
}

TEST(DepthCbfRegressorTest, CylinderSurface) {
  const PointCloud cloud = makeCylinder(1.5);
  Eigen::Vector3d gradient;
  fitAndCheck(cloud, Eigen::Vector3d(1.85, 0.0, 0.0),
              "/tmp/depth_cbf_cylinder_line.csv", 0, &gradient);
  EXPECT_GT(gradient.x(), 0.5);
  EXPECT_NEAR(gradient.y(), 0.0, 0.05);
  EXPECT_NEAR(gradient.z(), 0.0, 0.05);
}

TEST(DepthCbfRegressorTest, Corner) {
  const PointCloud cloud = makeCorner(1.0);
  Eigen::Vector3d gradient;
  fitAndCheck(cloud, Eigen::Vector3d(0.68, 0.68, 0.0),
              "/tmp/depth_cbf_corner_line.csv", 0, &gradient);
  EXPECT_LT(gradient.x(), -0.1);
  EXPECT_LT(gradient.y(), -0.1);
}

TEST(DepthCbfRegressorTest, RejectsMissingQuerySupportWithoutStaleBarrier) {
  RegressionConfig config = testConfig();
  config.nearest_neighbor_max_distance = 0.1;
  DepthCbfRegressor regressor(config);
  LocalBarrier barrier;
  barrier.valid = true;
  barrier.c = 42.0;
  std::string error;
  EXPECT_FALSE(regressor.fit(makePlane(1.0), Eigen::Vector3d::Zero(),
                             ros::Time(5.0), &barrier, nullptr, &error));
  EXPECT_FALSE(barrier.valid);
  EXPECT_DOUBLE_EQ(barrier.c, 0.0);
  EXPECT_FALSE(error.empty());
}

TEST(DepthCbfRegressorTest, ReportsFailureReasonAndStageTiming) {
  DepthCbfRegressor regressor(testConfig());
  LocalBarrier barrier;
  RegressionTiming timing;
  RegressionFailureReason reason = RegressionFailureReason::VALID;
  std::string error;
  EXPECT_FALSE(regressor.fit(PointCloud(), Eigen::Vector3d::Zero(),
                             ros::Time(1.0), &barrier, nullptr, &error,
                             &timing, &reason));
  EXPECT_EQ(reason, RegressionFailureReason::EMPTY_CLOUD);
  EXPECT_STREQ(regressionFailureReasonName(reason), "EMPTY_CLOUD");
  EXPECT_GE(timing.total_ms, 0.0);

  ASSERT_TRUE(regressor.fit(makePlane(1.0), Eigen::Vector3d(0.6, 0.0, 0.0),
                            ros::Time(2.0), &barrier, nullptr, &error,
                            &timing, &reason)) << error;
  EXPECT_EQ(reason, RegressionFailureReason::VALID);
  EXPECT_GT(timing.kdtree_build_ms, 0.0);
  EXPECT_GT(timing.nearest_neighbor_query_ms, 0.0);
  EXPECT_GT(timing.regression_ms, 0.0);
  EXPECT_GE(timing.total_ms, timing.kdtree_build_ms + timing.regression_ms);
}

TEST(DepthCbfRegressorTest, DeterministicRuntimeBenchmark) {
  const PointCloud cloud = makeCylinder(1.5);
  DepthCbfRegressor regressor(testConfig());
  double max_ms = 0.0;
  const int runs = 25;
  const auto begin = std::chrono::steady_clock::now();
  for (int i = 0; i < runs; ++i) {
    LocalBarrier barrier;
    std::string error;
    const auto one_begin = std::chrono::steady_clock::now();
    ASSERT_TRUE(regressor.fit(cloud, Eigen::Vector3d(1.85, 0.0, 0.0),
                              ros::Time(i), &barrier, nullptr, &error))
        << error;
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - one_begin)
            .count();
    max_ms = std::max(max_ms, elapsed_ms);
  }
  const double mean_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - begin)
          .count() /
      runs;
  std::cout << "M2 synthetic cylinder fit runtime: mean=" << mean_ms
            << " ms, max=" << max_ms << " ms\n";
}

}  // namespace
}  // namespace depth_cbf
}  // namespace coni_mpc

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
