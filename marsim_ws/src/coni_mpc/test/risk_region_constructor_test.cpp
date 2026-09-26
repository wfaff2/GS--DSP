#include "coni_mpc/risk_region_constructor.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <vector>

namespace {

std::vector<coni_mpc::RiskVoxel> makeCubeStages(std::size_t stages) {
  std::vector<coni_mpc::RiskVoxel> voxels;
  for (std::size_t stage = 0; stage < stages; ++stage) {
    for (int x = 0; x < 2; ++x) {
      for (int y = 0; y < 2; ++y) {
        for (int z = 0; z < 2; ++z) {
          coni_mpc::RiskVoxel voxel;
          voxel.position = Eigen::Vector3d(x, y, z);
          voxel.weight = 0.4;
          voxel.stage_index = static_cast<std::uint32_t>(stage);
          voxel.prediction_time = 0.1 * static_cast<double>(stage);
          voxels.push_back(voxel);
        }
      }
    }
  }
  return voxels;
}

}  // namespace

TEST(RiskRegionConstructor, UsesConnectedMassAndStagePersistence) {
  coni_mpc::RiskRegionConstructorConfig config;
  config.prediction_stages = 3;
  config.minimum_component_voxels = 1;
  config.minimum_persistent_stages = 2;
  config.persistence_high_confidence = 1.0;
  config.neighbor_support_fraction = 0.15;
  config.neighbor_support_weight = 0.5;
  config.voxel_resolution = 1.0;
  config.risk_radius = 0.0;
  config.max_split_depth = 0;

  const auto regions = coni_mpc::constructRiskRegions(
      makeCubeStages(config.prediction_stages), config);
  ASSERT_EQ(regions.size(), config.prediction_stages);
  for (const auto& region : regions) {
    EXPECT_TRUE(region.valid);
    EXPECT_TRUE(region.center.allFinite());
    EXPECT_TRUE(region.semi_axes.allFinite());
    EXPECT_GT(region.semi_axes.minCoeff(), 0.0);
  }
}

TEST(RiskRegionConstructor, RecursiveSplitKeepsRiskRegionsFinite) {
  coni_mpc::RiskRegionConstructorConfig config;
  config.prediction_stages = 1;
  config.minimum_component_voxels = 1;
  config.minimum_persistent_stages = 1;
  config.voxel_resolution = 1.0;
  config.risk_radius = 0.0;
  config.split_volume_ratio = 1.1;
  config.split_min_volume_reduction = 0.0;
  config.max_split_depth = 1;

  std::vector<coni_mpc::RiskVoxel> voxels;
  for (int x = -2; x <= 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      for (int z = 0; z < 2; ++z) {
        coni_mpc::RiskVoxel voxel;
        voxel.position = Eigen::Vector3d(x, y, z);
        voxel.weight = (x == 0) ? 0.2 : 0.8;
        voxel.stage_index = 0;
        voxels.push_back(voxel);
      }
    }
  }

  const auto regions = coni_mpc::constructRiskRegions(voxels, config);
  ASSERT_FALSE(regions.empty());
  for (const auto& region : regions) {
    EXPECT_TRUE(region.center.allFinite());
    EXPECT_TRUE(region.orientation.allFinite());
    EXPECT_TRUE(region.semi_axes.allFinite());
  }
}
