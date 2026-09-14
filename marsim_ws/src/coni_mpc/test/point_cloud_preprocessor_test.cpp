#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "coni_mpc/depth_cbf/point_cloud_preprocessor.h"

namespace coni_mpc {
namespace depth_cbf {
namespace {

constexpr double kTolerance = 1e-5;

pcl::PointXYZ pclPoint(const Eigen::Vector3d& value) {
  return pcl::PointXYZ(value.x(), value.y(), value.z());
}

void expectVectorNear(const Eigen::Vector3d& actual,
                      const Eigen::Vector3d& expected) {
  EXPECT_NEAR(actual.x(), expected.x(), kTolerance);
  EXPECT_NEAR(actual.y(), expected.y(), kTolerance);
  EXPECT_NEAR(actual.z(), expected.z(), kTolerance);
}

PreprocessorConfig testConfig() {
  PreprocessorConfig config;
  config.min_range = 0.0;
  config.max_range = 100.0;
  config.local_radius = 20.0;
  config.enable_voxel_filter = false;
  config.query_mesh_half_width_steps = 2;
  config.query_mesh_spacing = 0.1;
  config.nearest_neighbor_max_distance = 1.0;
  return config;
}

PointTransformTrace transformKnownWorldPoint(
    const PointCloudPreprocessor& preprocessor,
    const Eigen::Vector3d& obstacle_W,
    const Pose3d& pose_WB,
    const Pose3d& pose_WN) {
  const Eigen::Vector3d q_B =
      pose_WB.rotation_WF.transpose() * (obstacle_W - pose_WB.position_W);
  const Eigen::Vector3d q_L =
      preprocessor.config().rotation_BL.transpose() *
      (q_B - preprocessor.config().translation_BL);
  return preprocessor.transformPoint(q_L, pose_WB, pose_WN);
}

TEST(PointCloudPreprocessorTest, UavRotationDoesNotRotateWorldObstacle) {
  PointCloudPreprocessor preprocessor(testConfig());
  Pose3d pose_WB_1;
  Pose3d pose_WB_2;
  pose_WB_2.rotation_WF =
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Pose3d pose_WN;
  const Eigen::Vector3d obstacle_W(4.0, -1.0, 2.0);

  const auto trace_1 = transformKnownWorldPoint(
      preprocessor, obstacle_W, pose_WB_1, pose_WN);
  const auto trace_2 = transformKnownWorldPoint(
      preprocessor, obstacle_W, pose_WB_2, pose_WN);

  expectVectorNear(trace_1.q_W, obstacle_W);
  expectVectorNear(trace_2.q_W, obstacle_W);
}

TEST(PointCloudPreprocessorTest, UavTranslationKeepsWorldObstacleFixed) {
  PointCloudPreprocessor preprocessor(testConfig());
  Pose3d pose_WB_1;
  Pose3d pose_WB_2;
  pose_WB_2.position_W = Eigen::Vector3d(1.2, -0.7, 0.4);
  Pose3d pose_WN;
  const Eigen::Vector3d obstacle_W(5.0, 2.0, 1.0);

  const auto trace_1 = transformKnownWorldPoint(
      preprocessor, obstacle_W, pose_WB_1, pose_WN);
  const auto trace_2 = transformKnownWorldPoint(
      preprocessor, obstacle_W, pose_WB_2, pose_WN);

  expectVectorNear(trace_1.q_W, obstacle_W);
  expectVectorNear(trace_2.q_W, obstacle_W);
}

TEST(PointCloudPreprocessorTest, UgVmotionProducesExpectedApparentMotionInN) {
  PointCloudPreprocessor preprocessor(testConfig());
  Pose3d pose_WB;
  Pose3d pose_WN_1;
  Pose3d pose_WN_2;
  pose_WN_2.position_W = Eigen::Vector3d(1.0, 2.0, 0.0);
  pose_WN_2.rotation_WF =
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d obstacle_W(4.0, 3.0, 1.0);

  const auto trace_1 = transformKnownWorldPoint(
      preprocessor, obstacle_W, pose_WB, pose_WN_1);
  const auto trace_2 = transformKnownWorldPoint(
      preprocessor, obstacle_W, pose_WB, pose_WN_2);
  const Eigen::Vector3d expected_N_2 =
      pose_WN_2.rotation_WF.transpose() *
      (obstacle_W - pose_WN_2.position_W);

  expectVectorNear(trace_1.q_W, obstacle_W);
  expectVectorNear(trace_2.q_W, obstacle_W);
  expectVectorNear(trace_2.q_N, expected_N_2);
  EXPECT_GT((trace_2.q_N - trace_1.q_N).norm(), 1.0);
}

TEST(PointCloudPreprocessorTest, AppliesExplicitLidarToBodyExtrinsics) {
  PreprocessorConfig config = testConfig();
  config.rotation_BL =
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  config.translation_BL = Eigen::Vector3d(0.2, -0.1, 0.3);
  PointCloudPreprocessor preprocessor(config);
  Pose3d pose_WB;
  pose_WB.position_W = Eigen::Vector3d(1.0, 2.0, 3.0);
  Pose3d pose_WN;

  const Eigen::Vector3d q_L(2.0, 0.0, 1.0);
  const auto trace = preprocessor.transformPoint(q_L, pose_WB, pose_WN);
  expectVectorNear(trace.q_B, Eigen::Vector3d(0.2, 1.9, 1.3));
  expectVectorNear(trace.q_W, Eigen::Vector3d(1.2, 3.9, 4.3));
}

TEST(PointCloudPreprocessorTest, RejectsCropThatCannotSupportQueryMesh) {
  PreprocessorConfig config = testConfig();
  config.local_radius = 1.0;
  config.query_mesh_half_width_steps = 3;
  config.query_mesh_spacing = 0.2;
  config.nearest_neighbor_max_distance = 0.5;
  std::string error;
  std::string warning;
  EXPECT_FALSE(PointCloudPreprocessor::validateConfig(config, &error, &warning));
  EXPECT_NE(error.find("local_radius is too small"), std::string::npos);
}

TEST(PointCloudPreprocessorTest, FiltersRangeAndCropsAroundUavCenterInN) {
  PreprocessorConfig config = testConfig();
  config.min_range = 1.0;
  config.max_range = 5.0;
  config.local_radius = 2.0;
  config.query_mesh_half_width_steps = 1;
  config.query_mesh_spacing = 0.1;
  config.nearest_neighbor_max_distance = 1.0;
  PointCloudPreprocessor preprocessor(config);

  PointCloud cloud_L;
  cloud_L.push_back(pclPoint(Eigen::Vector3d(0.5, 0.0, 0.0)));
  cloud_L.push_back(pclPoint(Eigen::Vector3d(1.5, 0.0, 0.0)));
  cloud_L.push_back(pclPoint(Eigen::Vector3d(4.5, 0.0, 0.0)));
  Pose3d pose_WB;
  Pose3d pose_WN;
  PreprocessedClouds output;
  std::string error;
  ASSERT_TRUE(preprocessor.process(cloud_L, pose_WB, pose_WN, ros::Time(5.0),
                                   &output, &error));
  EXPECT_EQ(output.cloud_W->size(), 2U);
  EXPECT_EQ(output.local_cloud_N->size(), 1U);
  EXPECT_EQ(output.stamp, ros::Time(5.0));
}

}  // namespace
}  // namespace depth_cbf
}  // namespace coni_mpc

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
