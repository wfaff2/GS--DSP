#include "coni_mpc/num_sim_mpc.h"
#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cstring>

int num_uavs = 1;

namespace {
sensor_msgs::PointCloud2Ptr cloud(std::size_t count, double velocity = 0.0) {
  auto message = boost::make_shared<sensor_msgs::PointCloud2>();
  message->header.frame_id = "map";
  message->header.stamp = ros::Time::now();
  sensor_msgs::PointCloud2Modifier modifier(*message);
  modifier.setPointCloud2Fields(8,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "vx", 1, sensor_msgs::PointField::FLOAT32,
      "vy", 1, sensor_msgs::PointField::FLOAT32,
      "vz", 1, sensor_msgs::PointField::FLOAT32,
      "occupancy", 1, sensor_msgs::PointField::FLOAT32,
      "stage_index", 1, sensor_msgs::PointField::UINT32);
  modifier.resize(count * 21);
  for (std::size_t i = 0; i < count * 21; ++i) {
    const std::size_t item = i % count;
    float values[] = {1.20f + .001f * float(item), .01f * float(item % 3),
                      1.90f, float(velocity), float(-velocity), float(velocity), .01f};
    const std::uint32_t stage = i / count;
    std::memcpy(message->data.data() + i * message->point_step, values, sizeof(values));
    std::memcpy(message->data.data() + i * message->point_step + sizeof(values), &stage, sizeof(stage));
  }
  return message;
}

TEST(FieldSnapshot, PreservesSupportAndDistinguishesMissingEmptyInvalidAndStale) {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  coni_mpc::NumSimMpc controller(nh, pnh, 0);
  nav_msgs::Odometry car, quad;
  car.header.frame_id = quad.header.frame_id = "map";
  car.header.stamp = quad.header.stamp = ros::Time::now();
  car.pose.pose.orientation.w = quad.pose.pose.orientation.w = 1.0;
  quad.pose.pose.position.z = 1.0;
  controller.setCarOdom(car);
  controller.setQuadOdom(quad, 0);
  acado_mpc_common::RelativeTrajectoryPoint point;
  point.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  point.velocity.setZero();
  point.orientation = Eigen::Quaterniond::Identity();
  acado_mpc_common::RelativeTrajectory reference(point);
  controller.setReferenceWindow(reference, nullptr);
  controller.run();
  EXPECT_EQ(controller.getLastFieldHocbfDebugProfile()[0].field_status, "missing");

  controller.fieldOccupancyCallback(cloud(52, 1000.0));
  controller.run();
  const auto first = controller.getLastFieldHocbfDebugProfile()[0];
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(first.active);
  EXPECT_EQ(first.field_status, "valid_nonempty");
  EXPECT_EQ(first.candidate_count, 52u);
  EXPECT_EQ(first.eligible_count, 52u);
  EXPECT_EQ(first.selected_count, 52u);
  EXPECT_GT(std::abs(first.A.z()), 1e-5);
  EXPECT_NEAR(first.hdot, 0.0, 1e-10);

  // Opposite particle velocity cannot move a fixed grid center or change its
  // stage-0 derivatives when the physical state remains the same.
  controller.fieldOccupancyCallback(cloud(52, -1000.0));
  controller.run();
  const auto second = controller.getLastFieldHocbfDebugProfile()[0];
  EXPECT_NEAR(second.h, first.h, 1e-10);
  EXPECT_NEAR(second.hdot, first.hdot, 1e-10);

  auto invalid = cloud(1);
  invalid->header.frame_id = "unrecognized_frame";
  controller.fieldOccupancyCallback(invalid);
  controller.run();
  EXPECT_EQ(controller.getLastFieldHocbfDebugProfile()[0].field_status, "invalid");

  controller.fieldOccupancyCallback(cloud(0));
  controller.run();
  const auto empty = controller.getLastFieldHocbfDebugProfile()[0];
  EXPECT_EQ(empty.field_status, "valid_empty");
  EXPECT_TRUE(empty.valid);
  EXPECT_FALSE(empty.active);
  EXPECT_EQ(empty.selected_count, 0u);

  auto stale = cloud(1);
  stale->header.stamp = ros::Time::now() - ros::Duration(1.0);
  controller.fieldOccupancyCallback(stale);
  controller.run();
  EXPECT_EQ(controller.getLastFieldHocbfDebugProfile()[0].field_status, "stale");
  EXPECT_FALSE(controller.getLastFieldHocbfDebugProfile()[0].active);

  auto malformed = cloud(1);
  malformed->point_step = 1;  // Includes a scalar FLOAT32 larger than the row.
  malformed->row_step = malformed->width;
  malformed->data.resize(malformed->row_step);
  controller.fieldOccupancyCallback(malformed);
  controller.run();
  EXPECT_EQ(controller.getLastFieldHocbfDebugProfile()[0].field_status, "invalid");

  auto malformed_empty = cloud(0);
  malformed_empty->fields.front().datatype = 255;
  controller.fieldOccupancyCallback(malformed_empty);
  controller.run();
  EXPECT_EQ(controller.getLastFieldHocbfDebugProfile()[0].field_status, "invalid");

  malformed_empty = cloud(0);
  malformed_empty->fields.front().offset = malformed_empty->point_step;
  controller.fieldOccupancyCallback(malformed_empty);
  controller.run();
  EXPECT_EQ(controller.getLastFieldHocbfDebugProfile()[0].field_status, "invalid");
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "field_snapshot_integration_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
