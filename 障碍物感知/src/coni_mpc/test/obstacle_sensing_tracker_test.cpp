#include "coni_mpc/obstacle_sensing_tracker.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace coni_mpc {
namespace {

using Tracker = ObstacleSensingTracker;
constexpr double kPi = 3.14159265358979323846;

Tracker::Obstacle makeObstacle(double x, double y, double z = 1.5,
                               double radius = 0.4) {
  Tracker::Obstacle obstacle = Tracker::Obstacle::Zero();
  obstacle << x, y, z, radius, 0.0, 0.0, 0.0;
  return obstacle;
}

Tracker::ObstacleVector oneObstacle(double x, double y, double z = 1.5,
                                    double radius = 0.4) {
  Tracker::ObstacleVector obstacles;
  obstacles.push_back(makeObstacle(x, y, z, radius));
  return obstacles;
}

Tracker::Config onlineConfig() {
  Tracker::Config config;
  config.online = true;
  config.range_min_m = 0.15;
  config.range_max_m = 6.0;
  config.horizontal_fov_rad = 2.0 * kPi;
  config.angular_resolution_rad = 0.9 * kPi / 180.0;
  config.minimum_hit_rays = 3;
  config.control_rate_hz = 100.0;
  config.scan_rate_hz = 10.0;
  config.position_std_m = 0.0;
  config.dropout_probability = 0.0;
  config.delay_sec = 0.0;
  config.hold_sec = 0.3;
  config.kalman_process_variance = 1e-6;
  config.experiment_seed = 12345u;
  config.uav_id = 1u;
  return config;
}

Tracker::StepResult singleFreshStep(const Tracker::Config& config, double x,
                                    double y, double body_yaw = 0.0,
                                    double radius = 0.4) {
  Tracker tracker(config);
  return tracker.step(oneObstacle(x, y, 1.5, radius),
                      Eigen::Vector3d(0.0, 0.0, 2.0), body_yaw,
                      std::vector<double>{3.0});
}

TEST(ObstacleSensingTrackerTest, MapModeReturnsCompleteTruthWithoutNoise) {
  Tracker::Config config = onlineConfig();
  config.online = false;
  config.position_std_m = 10.0;
  config.dropout_probability = 1.0;
  Tracker tracker(config);
  Tracker::ObstacleVector truth;
  truth.push_back(makeObstacle(2.0, 3.0, 1.2, 0.4));
  truth.push_back(makeObstacle(-4.0, 1.0, 1.4, 0.5));

  const auto result =
      tracker.step(truth, Eigen::Vector3d(100.0, 100.0, 2.0), kPi);

  ASSERT_EQ(result.obstacles.size(), truth.size());
  ASSERT_EQ(result.obstacle_keys.size(), truth.size());
  EXPECT_EQ(result.obstacle_keys[0], "static:0");
  EXPECT_EQ(result.obstacle_keys[1], "static:1");
  EXPECT_TRUE(result.obstacles[0].isApprox(truth[0], 0.0));
  EXPECT_TRUE(result.obstacles[1].isApprox(truth[1], 0.0));
  EXPECT_EQ(result.stats.active_tracks, 2u);
  EXPECT_FALSE(result.stats.scan_triggered);
  EXPECT_EQ(tracker.cycle(), 1u);
}

TEST(ObstacleSensingTrackerTest, TriggersTenHertzScanOnHundredHertzControl) {
  Tracker tracker(onlineConfig());
  const auto truth = oneObstacle(5.0, 0.0);

  auto result = tracker.step(truth, Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
                             std::vector<double>{3.0});
  EXPECT_TRUE(result.stats.scan_triggered);
  EXPECT_EQ(result.stats.scan_index, 0u);
  EXPECT_EQ(result.stats.cast_rays, 400u);
  EXPECT_EQ(result.stats.generated_measurements, 1u);

  for (int cycle = 1; cycle < 10; ++cycle) {
    result = tracker.step(truth, Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
                          std::vector<double>{3.0});
    EXPECT_FALSE(result.stats.scan_triggered) << "cycle=" << cycle;
    EXPECT_EQ(result.stats.generated_measurements, 0u) << "cycle=" << cycle;
  }
  result = tracker.step(truth, Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
                        std::vector<double>{3.0});
  EXPECT_TRUE(result.stats.scan_triggered);
  EXPECT_EQ(result.stats.scan_index, 1u);
  EXPECT_EQ(result.stats.generated_measurements, 1u);
  EXPECT_EQ(tracker.scanPeriodControlCycles(), 10u);
}

TEST(ObstacleSensingTrackerTest, UsesInclusiveObstacleSurfaceRangeGate) {
  const Tracker::Config config = onlineConfig();

  EXPECT_EQ(singleFreshStep(config, 0.55, 0.0).stats.visible_obstacles, 1u);
  EXPECT_EQ(singleFreshStep(config, 0.549999, 0.0).stats.visible_obstacles,
            0u);
  EXPECT_EQ(singleFreshStep(config, 6.4, 0.0).stats.visible_obstacles, 1u);
  EXPECT_EQ(singleFreshStep(config, 6.400001, 0.0).stats.visible_obstacles,
            0u);
}

TEST(ObstacleSensingTrackerTest, IsYawInvariantAcrossFullAzimuth) {
  const Tracker::Config config = onlineConfig();
  EXPECT_EQ(singleFreshStep(config, -5.0, 0.0, 0.0).stats.visible_obstacles,
            1u);
  EXPECT_EQ(singleFreshStep(config, -5.0, 0.0, 1.234)
                .stats.visible_obstacles,
            1u);
}

TEST(ObstacleSensingTrackerTest, RequiresHorizontalScanPlaneIntersection) {
  const Tracker::Config config = onlineConfig();
  const auto truth = oneObstacle(5.0, 0.0, 1.5, 0.4);

  Tracker level_tracker(config);
  const auto level = level_tracker.step(
      truth, Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
      std::vector<double>{3.0});
  EXPECT_EQ(level.stats.visible_obstacles, 1u);

  Tracker above_tracker(config);
  const auto above = above_tracker.step(
      truth, Eigen::Vector3d(0.0, 0.0, 3.1), 0.0,
      std::vector<double>{3.0});
  EXPECT_EQ(above.stats.visible_obstacles, 0u);
}

TEST(ObstacleSensingTrackerTest, KeepsOnlyNearestReturnOnEachRay) {
  const Tracker::Config config = onlineConfig();
  Tracker::ObstacleVector aligned;
  aligned.push_back(makeObstacle(2.0, 0.0, 1.5, 0.5));
  aligned.push_back(makeObstacle(4.0, 0.0, 1.5, 0.4));

  Tracker aligned_tracker(config);
  const auto hidden = aligned_tracker.step(
      aligned, Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
      std::vector<double>{3.0, 3.0});
  ASSERT_EQ(hidden.visible_obstacle_ids.size(), 1u);
  EXPECT_EQ(hidden.visible_obstacle_ids[0], 0u);
  ASSERT_EQ(hidden.first_return_hit_counts.size(), 2u);
  EXPECT_GE(hidden.first_return_hit_counts[0], 3u);
  EXPECT_EQ(hidden.first_return_hit_counts[1], 0u);

  aligned[1](1) = 1.0;
  Tracker partial_tracker(config);
  const auto partial = partial_tracker.step(
      aligned, Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
      std::vector<double>{3.0, 3.0});
  ASSERT_EQ(partial.visible_obstacle_ids.size(), 2u);
  EXPECT_EQ(partial.visible_obstacle_ids[0], 0u);
  EXPECT_EQ(partial.visible_obstacle_ids[1], 1u);
}

TEST(ObstacleSensingTrackerTest, RequiresAtLeastThreeUnoccludedRayHits) {
  Tracker::Config three_hit_config = onlineConfig();
  const auto too_small =
      singleFreshStep(three_hit_config, 5.0, 0.0, 0.0, 0.005);
  EXPECT_EQ(too_small.stats.visible_obstacles, 0u);
  ASSERT_EQ(too_small.first_return_hit_counts.size(), 1u);
  EXPECT_LT(too_small.first_return_hit_counts[0], 3u);

  Tracker::Config one_hit_config = three_hit_config;
  one_hit_config.minimum_hit_rays = 1;
  const auto accepted =
      singleFreshStep(one_hit_config, 5.0, 0.0, 0.0, 0.005);
  EXPECT_EQ(accepted.stats.visible_obstacles, 1u);
}

TEST(ObstacleSensingTrackerTest, DeliversAfterExactTimeDelay) {
  for (const double delay_sec : {0.1, 0.3}) {
    Tracker::Config config = onlineConfig();
    config.delay_sec = delay_sec;
    Tracker tracker(config);
    const int delay_cycles = static_cast<int>(std::llround(100.0 * delay_sec));
    Tracker::StepResult result;

    for (int cycle = 0; cycle < delay_cycles; ++cycle) {
      result = tracker.step(oneObstacle(5.0, 0.0),
                            Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
                            std::vector<double>{3.0});
      EXPECT_EQ(result.stats.delivered_measurements, 0u);
      EXPECT_TRUE(result.obstacles.empty());
    }
    result = tracker.step(oneObstacle(5.0, 0.0),
                          Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
                          std::vector<double>{3.0});
    ASSERT_EQ(result.delivered_measurements.size(), 1u);
    EXPECT_EQ(result.delivered_measurements[0].generation_cycle, 0u);
    EXPECT_EQ(result.delivered_measurements[0].delivery_cycle,
              static_cast<std::uint64_t>(delay_cycles));
    EXPECT_EQ(result.stats.created_tracks, 1u);
    EXPECT_EQ(result.stats.active_tracks, 1u);
    ASSERT_EQ(result.tracks.size(), 1u);
    EXPECT_EQ(result.tracks[0].last_generation_cycle, 0u);
    EXPECT_EQ(result.tracks[0].last_delivery_cycle,
              static_cast<std::uint64_t>(delay_cycles));
    EXPECT_EQ(result.tracks[0].last_update_cycle,
              static_cast<std::uint64_t>(delay_cycles));
    EXPECT_EQ(result.tracks[0].last_received_cycle,
              result.tracks[0].last_delivery_cycle);
    EXPECT_EQ(result.tracks[0].track_instance, 1u);
    EXPECT_EQ(result.tracks[0].measurement_update_count, 1u);
    EXPECT_TRUE(result.tracks[0].created_this_cycle);
    EXPECT_TRUE(result.tracks[0].updated_this_cycle);
    EXPECT_FALSE(result.tracks[0].reinitialized_this_cycle);
    EXPECT_FALSE(result.tracks[0].last_innovation_valid);
  }
}

TEST(ObstacleSensingTrackerTest, RecordsKalmanInnovationCovarianceAndGain) {
  Tracker::Config config = onlineConfig();
  config.control_rate_hz = 100.0;
  config.scan_rate_hz = 100.0;
  config.position_std_m = 0.05;
  config.kalman_process_variance = 1e-6;
  Tracker tracker(config);
  const auto truth = oneObstacle(5.0, 0.0);

  const auto initialized = tracker.step(
      truth, Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
      std::vector<double>{3.0});
  ASSERT_EQ(initialized.tracks.size(), 1u);
  ASSERT_EQ(initialized.delivered_measurements.size(), 1u);
  const Eigen::Vector2d prior_position = initialized.tracks[0].position;
  EXPECT_FALSE(initialized.tracks[0].last_innovation_valid);

  const auto corrected = tracker.step(
      truth, Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
      std::vector<double>{3.0});
  ASSERT_EQ(corrected.tracks.size(), 1u);
  ASSERT_EQ(corrected.delivered_measurements.size(), 1u);
  const auto& track = corrected.tracks[0];
  const Eigen::Vector2d expected_innovation =
      corrected.delivered_measurements[0].position - prior_position;
  const double measurement_variance = 0.05 * 0.05;
  const double predicted_variance = measurement_variance + 1e-6;
  const double expected_innovation_variance =
      predicted_variance + measurement_variance;
  const double expected_gain =
      predicted_variance / expected_innovation_variance;

  EXPECT_TRUE(track.last_innovation_valid);
  EXPECT_TRUE(track.last_innovation.isApprox(expected_innovation, 1e-14));
  EXPECT_NEAR(track.last_innovation_covariance(0, 0),
              expected_innovation_variance, 1e-14);
  EXPECT_NEAR(track.last_innovation_covariance(1, 1),
              expected_innovation_variance, 1e-14);
  EXPECT_NEAR(track.last_kalman_gain(0, 0), expected_gain, 1e-14);
  EXPECT_NEAR(track.last_kalman_gain(1, 1), expected_gain, 1e-14);
  EXPECT_EQ(track.last_generation_cycle, 1u);
  EXPECT_EQ(track.last_delivery_cycle, 1u);
  EXPECT_EQ(track.last_update_cycle, 1u);
  EXPECT_EQ(track.measurement_update_count, 2u);
  EXPECT_FALSE(track.created_this_cycle);
  EXPECT_TRUE(track.updated_this_cycle);
  EXPECT_EQ(corrected.stats.kalman_corrections, 1u);
}

TEST(ObstacleSensingTrackerTest, AppliesDropoutOnlyOnScanInstants) {
  Tracker::Config config = onlineConfig();
  config.dropout_probability = 1.0;
  Tracker tracker(config);
  const auto truth = oneObstacle(5.0, 0.0);

  auto result = tracker.step(truth, Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
                             std::vector<double>{3.0});
  EXPECT_TRUE(result.stats.scan_triggered);
  EXPECT_EQ(result.stats.dropped_measurements, 1u);
  for (int cycle = 1; cycle < 10; ++cycle) {
    result = tracker.step(truth, Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
                          std::vector<double>{3.0});
    EXPECT_FALSE(result.stats.scan_triggered);
    EXPECT_EQ(result.stats.dropped_measurements, 0u);
  }
  result = tracker.step(truth, Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
                        std::vector<double>{3.0});
  EXPECT_TRUE(result.stats.scan_triggered);
  EXPECT_EQ(result.stats.dropped_measurements, 1u);
}

TEST(ObstacleSensingTrackerTest, KeyedNoiseSurvivesVisibilityBranchChanges) {
  Tracker::Config config = onlineConfig();
  config.control_rate_hz = 100.0;
  config.scan_rate_hz = 100.0;
  config.position_std_m = 0.05;
  config.experiment_seed = 24680u;
  Tracker first(config);
  Tracker second(config);

  const auto first_scan_a = first.step(
      oneObstacle(5.0, 0.0), Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
      std::vector<double>{3.0});
  const auto first_scan_b = second.step(
      oneObstacle(7.0, 0.0), Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
      std::vector<double>{3.0});
  ASSERT_EQ(first_scan_a.generated_measurements.size(), 1u);
  EXPECT_TRUE(first_scan_b.generated_measurements.empty());

  const auto second_scan_a = first.step(
      oneObstacle(5.0, 0.0), Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
      std::vector<double>{3.0});
  const auto second_scan_b = second.step(
      oneObstacle(5.0, 0.0), Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
      std::vector<double>{3.0});
  ASSERT_EQ(second_scan_a.generated_measurements.size(), 1u);
  ASSERT_EQ(second_scan_b.generated_measurements.size(), 1u);
  EXPECT_DOUBLE_EQ(second_scan_a.generated_measurements[0].position.x(),
                   second_scan_b.generated_measurements[0].position.x());
  EXPECT_DOUBLE_EQ(second_scan_a.generated_measurements[0].position.y(),
                   second_scan_b.generated_measurements[0].position.y());
}

TEST(ObstacleSensingTrackerTest, KeyIncludesExperimentSeedAndUavId) {
  Tracker::Config first_config = onlineConfig();
  first_config.position_std_m = 0.05;
  Tracker::Config other_uav_config = first_config;
  other_uav_config.uav_id += 1;
  Tracker::Config other_seed_config = first_config;
  other_seed_config.experiment_seed += 1;

  const auto first = singleFreshStep(first_config, 5.0, 0.0);
  const auto other_uav = singleFreshStep(other_uav_config, 5.0, 0.0);
  const auto other_seed = singleFreshStep(other_seed_config, 5.0, 0.0);
  ASSERT_EQ(first.generated_measurements.size(), 1u);
  ASSERT_EQ(other_uav.generated_measurements.size(), 1u);
  ASSERT_EQ(other_seed.generated_measurements.size(), 1u);
  EXPECT_FALSE(first.generated_measurements[0].position.isApprox(
      other_uav.generated_measurements[0].position, 0.0));
  EXPECT_FALSE(first.generated_measurements[0].position.isApprox(
      other_seed.generated_measurements[0].position, 0.0));
}

TEST(ObstacleSensingTrackerTest, DegradedDropoutRateIsRepeatable) {
  Tracker::Config config = onlineConfig();
  config.control_rate_hz = 100.0;
  config.scan_rate_hz = 100.0;
  config.position_std_m = 0.05;
  config.dropout_probability = 0.10;
  config.experiment_seed = 98765u;
  Tracker first(config);
  Tracker second(config);
  constexpr int kScans = 10000;
  std::size_t dropped = 0;

  for (int scan = 0; scan < kScans; ++scan) {
    const auto a = first.step(oneObstacle(5.0, 0.0),
                              Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
                              std::vector<double>{3.0});
    const auto b = second.step(oneObstacle(5.0, 0.0),
                               Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
                               std::vector<double>{3.0});
    EXPECT_EQ(a.stats.dropped_measurements, b.stats.dropped_measurements);
    ASSERT_EQ(a.generated_measurements.size(), b.generated_measurements.size());
    if (!a.generated_measurements.empty()) {
      EXPECT_DOUBLE_EQ(a.generated_measurements[0].position.x(),
                       b.generated_measurements[0].position.x());
      EXPECT_DOUBLE_EQ(a.generated_measurements[0].position.y(),
                       b.generated_measurements[0].position.y());
    }
    dropped += a.stats.dropped_measurements;
  }

  const double dropout_rate =
      static_cast<double>(dropped) / static_cast<double>(kScans);
  RecordProperty("observed_dropout_rate", std::to_string(dropout_rate));
  RecordProperty("dropped_count", static_cast<int>(dropped));
  EXPECT_GT(dropout_rate, 0.085);
  EXPECT_LT(dropout_rate, 0.115);
}

TEST(ObstacleSensingTrackerTest, GaussianCenterNoiseHasConfiguredMoments) {
  Tracker::Config config = onlineConfig();
  config.control_rate_hz = 100.0;
  config.scan_rate_hz = 100.0;
  config.position_std_m = 0.05;
  config.experiment_seed = 54321u;
  Tracker tracker(config);
  constexpr int kScans = 20000;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_sq_x = 0.0;
  double sum_sq_y = 0.0;

  for (int scan = 0; scan < kScans; ++scan) {
    const auto result = tracker.step(
        oneObstacle(5.0, 0.0), Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
        std::vector<double>{3.0});
    ASSERT_EQ(result.generated_measurements.size(), 1u);
    const Eigen::Vector2d error =
        result.generated_measurements[0].position - Eigen::Vector2d(5.0, 0.0);
    sum_x += error.x();
    sum_y += error.y();
    sum_sq_x += error.x() * error.x();
    sum_sq_y += error.y() * error.y();
  }

  const double mean_x = sum_x / static_cast<double>(kScans);
  const double mean_y = sum_y / static_cast<double>(kScans);
  const double std_x = std::sqrt(
      sum_sq_x / static_cast<double>(kScans) - mean_x * mean_x);
  const double std_y = std::sqrt(
      sum_sq_y / static_cast<double>(kScans) - mean_y * mean_y);
  RecordProperty("noise_mean_x_m", std::to_string(mean_x));
  RecordProperty("noise_mean_y_m", std::to_string(mean_y));
  RecordProperty("noise_std_x_m", std::to_string(std_x));
  RecordProperty("noise_std_y_m", std::to_string(std_y));
  EXPECT_NEAR(mean_x, 0.0, 0.0015);
  EXPECT_NEAR(mean_y, 0.0, 0.0015);
  EXPECT_NEAR(std_x, 0.05, 0.002);
  EXPECT_NEAR(std_y, 0.05, 0.002);
}

TEST(ObstacleSensingTrackerTest, StaticKalmanFilterConvergesAndStaysFinite) {
  Tracker::Config config = onlineConfig();
  config.position_std_m = 0.05;
  config.kalman_process_variance = 1e-6;
  config.experiment_seed = 112233u;
  Tracker tracker(config);
  const Eigen::Vector2d truth_xy(5.0, 0.0);
  double squared_error_sum = 0.0;
  std::size_t error_samples = 0;
  Tracker::StepResult result;

  for (int cycle = 0; cycle < 4000; ++cycle) {
    result = tracker.step(oneObstacle(truth_xy.x(), truth_xy.y()),
                          Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
                          std::vector<double>{3.0});
    ASSERT_EQ(result.tracks.size(), 1u);
    if (cycle >= 500) {
      squared_error_sum +=
          (result.tracks[0].position - truth_xy).squaredNorm();
      ++error_samples;
    }
  }

  const double filtered_rmse =
      std::sqrt(squared_error_sum / static_cast<double>(error_samples));
  RecordProperty("filtered_position_rmse_m", std::to_string(filtered_rmse));
  EXPECT_LT(filtered_rmse, 0.04);
  const Eigen::Matrix2d covariance = result.tracks[0].covariance;
  RecordProperty("final_covariance_trace",
                 std::to_string(covariance.trace()));
  EXPECT_TRUE(covariance.allFinite());
  EXPECT_NEAR((covariance - covariance.transpose()).norm(), 0.0, 1e-14);
  EXPECT_GT(covariance(0, 0), 0.0);
  EXPECT_GT(covariance(1, 1), 0.0);
  EXPECT_LT(covariance.trace(), 2.0 * 0.05 * 0.05);
}

TEST(ObstacleSensingTrackerTest, HoldsForConfiguredTimeThenDeletesAndReacquires) {
  Tracker::Config config = onlineConfig();
  config.hold_sec = 0.3;
  Tracker tracker(config);
  const auto truth = oneObstacle(5.0, 0.0);

  auto result = tracker.step(truth, Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
                             std::vector<double>{3.0});
  ASSERT_EQ(result.stats.active_tracks, 1u);
  ASSERT_EQ(result.obstacle_keys[0], "static:0");

  for (int cycle = 1; cycle <= 30; ++cycle) {
    result = tracker.step(truth, Eigen::Vector3d(0.0, 0.0, 3.1), 0.0,
                          std::vector<double>{3.0});
    EXPECT_EQ(result.stats.active_tracks, 1u) << "cycle=" << cycle;
    EXPECT_EQ(result.stats.deleted_tracks, 0u) << "cycle=" << cycle;
  }
  result = tracker.step(truth, Eigen::Vector3d(0.0, 0.0, 3.1), 0.0,
                        std::vector<double>{3.0});
  EXPECT_EQ(result.stats.active_tracks, 0u);
  EXPECT_EQ(result.stats.deleted_tracks, 1u);

  for (int cycle = 32; cycle < 40; ++cycle) {
    result = tracker.step(truth, Eigen::Vector3d(0.0, 0.0, 3.1), 0.0,
                          std::vector<double>{3.0});
  }
  result = tracker.step(truth, Eigen::Vector3d(0.0, 0.0, 2.0), 0.0,
                        std::vector<double>{3.0});
  ASSERT_EQ(result.stats.active_tracks, 1u);
  ASSERT_EQ(result.obstacle_keys.size(), 1u);
  EXPECT_EQ(result.obstacle_keys[0], "static:0");
  EXPECT_EQ(result.stats.created_tracks, 1u);
  EXPECT_EQ(result.stats.reinitialized_tracks, 1u);
  ASSERT_EQ(result.tracks.size(), 1u);
  EXPECT_EQ(result.tracks[0].track_instance, 2u);
  EXPECT_EQ(result.tracks[0].measurement_update_count, 1u);
  EXPECT_TRUE(result.tracks[0].created_this_cycle);
  EXPECT_TRUE(result.tracks[0].reinitialized_this_cycle);
  EXPECT_FALSE(result.tracks[0].last_innovation_valid);
  EXPECT_EQ(tracker.holdControlCycles(), 30u);
}

}  // namespace
}  // namespace coni_mpc

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
