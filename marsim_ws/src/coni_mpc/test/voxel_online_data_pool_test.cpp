#include "coni_mpc/voxel_online_data_pool.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace {
using coni_mpc::VoxelOnlineDataPool;
using coni_mpc::field_hocbf::KinematicPoint;
using coni_mpc::field_hocbf::KinematicPoints;

KinematicPoint point(double x, double y, double z, double occupancy) {
  KinematicPoint result;
  result.position = Eigen::Vector3d(x, y, z);
  result.occupancy = occupancy;
  return result;
}

TEST(VoxelOnlineDataPool, KeepsDistinctThreeDimensionalKeysAndNegativeFloors) {
  const VoxelOnlineDataPool pool(1.0, 4, 30.0);
  const KinematicPoints raw{
      point(-0.1, 0.1, 0.1, 0.2), point(0.1, -0.1, 0.1, 0.3),
      point(0.1, 0.1, -0.1, 0.4), point(0.1, 0.1, 0.1, 0.5)};

  const std::vector<double> result =
      pool.build(Eigen::Vector3d::Zero(), raw);

  ASSERT_EQ(result.size(), 16u);
  EXPECT_DOUBLE_EQ(result[0], -0.1);
  EXPECT_DOUBLE_EQ(result[1], 0.1);
  EXPECT_DOUBLE_EQ(result[2], 0.1);
  EXPECT_DOUBLE_EQ(result[3], 0.2);
  EXPECT_DOUBLE_EQ(result[4], 0.1);
  EXPECT_DOUBLE_EQ(result[5], -0.1);
  EXPECT_DOUBLE_EQ(result[7], 0.3);
  EXPECT_DOUBLE_EQ(result[8], 0.1);
  EXPECT_DOUBLE_EQ(result[10], -0.1);
  EXPECT_DOUBLE_EQ(result[11], 0.4);
  EXPECT_DOUBLE_EQ(result[12], 0.1);
  EXPECT_DOUBLE_EQ(result[13], 0.1);
  EXPECT_DOUBLE_EQ(result[14], 0.1);
  EXPECT_DOUBLE_EQ(result[15], 0.5);
}

TEST(VoxelOnlineDataPool, UsesUnweightedCentroidAndCapsSummedOccupancy) {
  const VoxelOnlineDataPool pool(1.0, 1, 20.0);
  const KinematicPoints raw{point(0.1, 0.2, 0.3, 0.8),
                            point(0.5, 0.4, 0.7, 0.7)};

  const std::vector<double> result =
      pool.build(Eigen::Vector3d::Zero(), raw);

  ASSERT_EQ(result.size(), 4u);
  EXPECT_DOUBLE_EQ(result[0], 0.3);
  EXPECT_NEAR(result[1], 0.3, 1e-15);
  EXPECT_DOUBLE_EQ(result[2], 0.5);
  EXPECT_DOUBLE_EQ(result[3], 1.0);
}

TEST(VoxelOnlineDataPool, OrdersByDistanceThenGridKeyAndPadsRemainingSlots) {
  const VoxelOnlineDataPool pool(1.0, 4, 9.0);
  const KinematicPoints raw{point(0.2, 2.2, 0.0, 0.4),
                            point(-0.2, -2.2, 0.0, 0.6),
                            point(0.2, 0.0, 0.0, 0.8)};
  const Eigen::Vector3d position(0.0, 0.0, 1.0);

  const std::vector<double> result = pool.build(position, raw);

  ASSERT_EQ(result.size(), 16u);
  EXPECT_DOUBLE_EQ(result[0], 0.2);
  EXPECT_DOUBLE_EQ(result[3], 0.8);
  EXPECT_DOUBLE_EQ(result[4], -0.2);
  EXPECT_DOUBLE_EQ(result[5], -2.2);
  EXPECT_DOUBLE_EQ(result[7], 0.6);
  EXPECT_DOUBLE_EQ(result[8], 0.2);
  EXPECT_DOUBLE_EQ(result[9], 2.2);
  EXPECT_DOUBLE_EQ(result[11], 0.4);
  EXPECT_DOUBLE_EQ(result[12], 9.0);
  EXPECT_DOUBLE_EQ(result[13], 0.0);
  EXPECT_DOUBLE_EQ(result[14], 1.0);
  EXPECT_DOUBLE_EQ(result[15], 0.0);
}

TEST(VoxelOnlineDataPool, PreservesOccupancyAtAndBeyondFormerCutoff) {
  const VoxelOnlineDataPool pool(0.5, 3, 8.0);
  const KinematicPoints raw{point(2.0, 0.0, 0.0, 0.9),
                            point(1.5, 0.0, 0.0, 0.7),
                            point(6.0, 0.0, 0.0, 0.8)};

  const std::vector<double> result =
      pool.build(Eigen::Vector3d::Zero(), raw);

  EXPECT_DOUBLE_EQ(result[3], 0.7);
  EXPECT_DOUBLE_EQ(result[7], 0.9);
  EXPECT_DOUBLE_EQ(result[11], 0.8);
}

TEST(VoxelOnlineDataPool, RejectsInvalidConfigurationAndReferencePosition) {
  EXPECT_THROW(VoxelOnlineDataPool(0.0, 1, 2.0),
               std::invalid_argument);
  EXPECT_THROW(VoxelOnlineDataPool(1.0, 0, 2.0),
               std::invalid_argument);
  EXPECT_THROW(VoxelOnlineDataPool(1.0, 1, 0.0),
               std::invalid_argument);
  EXPECT_THROW(VoxelOnlineDataPool(1.0, 1, -2.0),
               std::invalid_argument);
  EXPECT_THROW(
      VoxelOnlineDataPool(std::numeric_limits<double>::infinity(), 1, 2.0),
      std::invalid_argument);

  const VoxelOnlineDataPool pool(1.0, 1, 2.0);
  EXPECT_THROW(pool.build(
                   Eigen::Vector3d(
                       std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
                   KinematicPoints()),
               std::invalid_argument);
}

TEST(VoxelOnlineDataPool, IgnoresInvalidAndZeroProbabilityPoints) {
  const VoxelOnlineDataPool pool(1.0, 2, 6.0);
  KinematicPoints raw{point(0.1, 0.0, 0.0, 0.0),
                      point(0.2, 0.0, 0.0, -0.1),
                      point(0.3, 0.0, 0.0,
                            std::numeric_limits<double>::quiet_NaN()),
                      point(0.4, 0.0, 0.0, 0.5)};
  raw.push_back(point(std::numeric_limits<double>::infinity(), 0.0, 0.0,
                      0.9));

  const std::vector<double> result =
      pool.build(Eigen::Vector3d::Zero(), raw);

  ASSERT_EQ(result.size(), 8u);
  EXPECT_DOUBLE_EQ(result[0], 0.4);
  EXPECT_DOUBLE_EQ(result[3], 0.5);
  EXPECT_DOUBLE_EQ(result[4], 6.0);
  EXPECT_DOUBLE_EQ(result[5], 0.0);
  EXPECT_DOUBLE_EQ(result[6], 0.0);
  EXPECT_DOUBLE_EQ(result[7], 0.0);
}

}  // namespace
