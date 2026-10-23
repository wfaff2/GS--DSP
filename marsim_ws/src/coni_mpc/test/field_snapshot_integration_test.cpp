#include "coni_mpc/num_sim_mpc.h"
#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cstring>
#include <sstream>

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

ros::NodeHandle caseParameters(const std::string& name) {
  ros::NodeHandle root("~");
  XmlRpc::XmlRpcValue config;
  EXPECT_TRUE(root.getParam("", config));
  ros::NodeHandle pnh(root, name);
  pnh.setParam("", config);
  pnh.setParam("cbf/enabled", false);
  pnh.setParam("cbf/field_detailed_diagnostics", false);
  pnh.setParam("cbf/field_replay_diagnostics", false);
  pnh.setParam("cbf/field_timeout", 3.0);
  pnh.setParam("visualization/path_max_points", 4);
  pnh.setParam("visualization/path_publish_rate_hz", 10.0);
  return pnh;
}

void initializeHover(coni_mpc::NumSimMpc& controller, int vehicle = 0) {
  nav_msgs::Odometry car, quad;
  car.header.frame_id = quad.header.frame_id = "map";
  car.header.stamp = quad.header.stamp = ros::Time::now();
  car.pose.pose.orientation.w = quad.pose.pose.orientation.w = 1.0;
  quad.pose.pose.position.z = 1.0;
  controller.setCarOdom(car);
  controller.setQuadOdom(quad, vehicle);
  acado_mpc_common::RelativeTrajectoryPoint point;
  point.position = Eigen::Vector3d(0, 0, 1);
  point.velocity.setZero();
  point.orientation = Eigen::Quaterniond::Identity();
  const acado_mpc_common::RelativeTrajectory reference(point);
  controller.setReferenceWindow(reference, nullptr);
}

TEST(FieldSnapshot, IndexedSlicesMatchOriginalScanAndDiagnosticsDoNotChangeConstraints) {
  ros::NodeHandle nh;
  auto pnh = caseParameters("indexed_slices");
  coni_mpc::NumSimMpc controller(nh, pnh, 0);
  initializeHover(controller);
  auto message = cloud(6, 7.0);
  const auto sorted_data = message->data;
  coni_mpc::field_hocbf::KinematicPoints flat;
  // Interleave stages in reverse order and include zero rows/empty slices.
  // A per-stage range assumption or a reorder of source IDs would fail this.
  for (std::size_t row = 0; row < 126; ++row) {
    const std::size_t item = row / 21;
    const std::size_t stage = 20 - row % 21;
    auto* data = message->data.data() + row * message->point_step;
    std::memcpy(data, sorted_data.data() + (stage * 6 + item) * message->point_step,
                message->point_step);
    float values[7];
    std::memcpy(values, data, sizeof(values));
    values[0] += .005f * stage;
    if (stage == 7 || stage == 20 || item == 1) values[6] = 0;
    std::memcpy(data, values, sizeof(values));
    if (values[6] == 0) continue;
    coni_mpc::field_hocbf::KinematicPoint p;
    p.position = Eigen::Vector3d(values[0], values[1], values[2]);
    p.published_velocity_diagnostic = Eigen::Vector3d(values[3], values[4], values[5]);
    p.occupancy = values[6];
    p.source_id = row;
    flat.push_back(p);
  }
  coni_mpc::field_hocbf::Config config;
  pnh.getParam("cbf/field_sigma", config.sigma);
  pnh.getParam("cbf/field_distance_margin", config.distance_margin);
  pnh.getParam("cbf/field_voxel_size", config.voxel_size);
  pnh.getParam("cbf/field_distance_epsilon", config.distance_epsilon);
  pnh.getParam("cbf/field_occupancy_weight_alpha", config.occupancy_alpha);
  pnh.getParam("cbf/field_prune_radius", config.prune_radius);
  pnh.getParam("cbf/field_cutoff_width", config.cutoff_width);
  pnh.getParam("cbf/field_occupancy_full_confidence", config.occupancy_full_confidence);
  pnh.getParam("cbf/field_clear_value", config.clear_field_value);
  pnh.getParam("cbf/field_gamma1", config.gamma1);
  pnh.getParam("cbf/field_gamma2", config.gamma2);
  pnh.getParam("uav_radius", config.uav_radius);
  double tau_xy = .2, tau_z = .2;
  pnh.getParam("tau_v_xy", tau_xy);
  pnh.getParam("tau_v_z", tau_z);
  const Eigen::Vector3d lambda(1.0 / tau_xy, 1.0 / tau_xy, 1.0 / tau_z);

  const auto compare_to_scan = [&](bool detailed, bool aged) {
    controller.setFieldDiagnosticsConsumerEnabled(detailed);
    message->header.stamp = ros::Time::now() - ros::Duration(aged ? 2.08 : .08);
    controller.fieldOccupancyCallback(message);
    controller.run();
    const auto profile = controller.getLastFieldHocbfDebugProfile();
    for (int stage = 0; stage <= 20; ++stage) {
      const int source = aged ? 19 : std::min(20, stage + 1);
      coni_mpc::field_hocbf::KinematicPoints old_scan;
      for (const auto& p : flat) {
        std::uint32_t row_stage;
        std::memcpy(&row_stage, message->data.data() + p.source_id * message->point_step + 28, 4);
        if (row_stage == static_cast<std::uint32_t>(source)) old_scan.push_back(p);
      }
      const auto& actual = profile[stage];
      ASSERT_TRUE(actual.valid);
      EXPECT_EQ(actual.candidate_count, old_scan.size());
      coni_mpc::field_hocbf::PruneStats stats;
      const auto selected = coni_mpc::field_hocbf::prune(
          old_scan, actual.nominal_position, config, &stats);
      const auto expected = coni_mpc::field_hocbf::computeConstraint(
          selected, actual.nominal_position, actual.nominal_velocity,
          Eigen::Vector3d::Zero(), config, lambda);
      EXPECT_EQ(actual.active, expected.active);
      EXPECT_EQ(actual.selected_count, selected.size());
      EXPECT_EQ(actual.detailed_diagnostics_enabled, detailed);
      if (expected.active) {
        EXPECT_NEAR(actual.h, expected.h, 1e-10);
        EXPECT_NEAR(actual.hdot, expected.hdot, 1e-10);
        EXPECT_NEAR(actual.b, expected.b, 1e-10);
        EXPECT_TRUE(actual.A.isApprox(expected.A, 1e-12));
        const Eigen::Vector3d k_v =
            (config.gamma1 + config.gamma2) * lambda.cwiseInverse() -
            Eigen::Vector3d::Ones();
        const Eigen::Vector3d k_p =
            (config.gamma1 * config.gamma2) * lambda.cwiseInverse();
        const double profile_b = expected.b + expected.A.dot(
            k_v.cwiseProduct(actual.nominal_velocity) +
            k_p.cwiseProduct(actual.nominal_position));
        EXPECT_NEAR(actual.profile_b, profile_b, 1e-10);
      }
      if (detailed) {
        std::ostringstream ids;
        for (std::size_t i = 0; i < stats.selected_source_ids.size(); ++i) {
          if (i) ids << ';';
          ids << stats.selected_source_ids[i];
        }
        EXPECT_EQ(actual.selected_source_ids, ids.str());
      } else {
        EXPECT_TRUE(actual.selected_source_ids.empty());
        EXPECT_TRUE(std::isnan(actual.selected_relative_speed_max));
      }
    }
  };
  compare_to_scan(false, false);
  compare_to_scan(true, false);
  compare_to_scan(false, true);
  compare_to_scan(true, true);
}

TEST(FieldSnapshot, ReplayAutomaticallyRequestsDetailedDiagnostics) {
  ros::NodeHandle nh;
  auto pnh = caseParameters("replay_consumer");
  pnh.setParam("cbf/field_replay_diagnostics", true);
  coni_mpc::NumSimMpc controller(nh, pnh, 0);
  initializeHover(controller);
  controller.fieldOccupancyCallback(cloud(3));
  controller.run();
  const auto& debug = controller.getLastFieldHocbfDebugProfile()[0];
  EXPECT_TRUE(debug.detailed_diagnostics_enabled);
  EXPECT_FALSE(debug.selected_source_ids.empty());
  std::ostringstream replay;
  EXPECT_TRUE(controller.writeFieldReplayDiagnostics(replay, 0, 0));
  EXPECT_FALSE(replay.str().empty());
}

TEST(FieldSnapshot, StageIndicesAndDiagnosticConsumersAreIsolatedPerVehicle) {
  struct VehicleCountScope {
    int previous = num_uavs;
    VehicleCountScope() { num_uavs = 2; }
    ~VehicleCountScope() { num_uavs = previous; }
  } count_scope;
  ros::NodeHandle nh;
  auto first_params = caseParameters("vehicle_zero");
  auto second_params = caseParameters("vehicle_one");
  coni_mpc::NumSimMpc first(nh, first_params, 0), second(nh, second_params, 1);
  initializeHover(first, 0);
  initializeHover(second, 1);
  second.setFieldDiagnosticsConsumerEnabled(true);
  first.fieldOccupancyCallback(cloud(4));
  second.fieldOccupancyCallback(cloud(9));
  first.run();
  second.run();
  for (int stage = 0; stage <= 20; ++stage) {
    const auto& a = first.getLastFieldHocbfDebugProfile()[stage];
    const auto& b = second.getLastFieldHocbfDebugProfile()[stage];
    EXPECT_EQ(a.candidate_count, 4u);
    EXPECT_EQ(b.candidate_count, 9u);
    EXPECT_TRUE(a.selected_source_ids.empty());
    EXPECT_FALSE(b.selected_source_ids.empty());
    EXPECT_FALSE(a.detailed_diagnostics_enabled);
    EXPECT_TRUE(b.detailed_diagnostics_enabled);
  }
  first.fieldOccupancyCallback(cloud(2));
  first.run();
  EXPECT_EQ(first.getLastFieldHocbfDebugProfile()[0].candidate_count, 2u);
  EXPECT_EQ(second.getLastFieldHocbfDebugProfile()[0].candidate_count, 9u);
}

TEST(FieldSnapshot, CommonControlEpochSurvivesWallDelayAndRejectsFutureInput) {
  ros::NodeHandle nh;
  auto pnh = caseParameters("control_epoch");
  coni_mpc::NumSimMpc controller(nh, pnh, 0);
  initializeHover(controller);
  controller.setFieldDiagnosticsConsumerEnabled(true);
  const ros::Time epoch = ros::Time::now();
  controller.setControlCycleStamp(epoch);
  auto message = cloud(3);
  message->header.stamp = epoch - ros::Duration(.145);
  controller.fieldOccupancyCallback(message);
  ASSERT_TRUE(controller.hasFreshFieldSnapshot(epoch));
  controller.run();
  const auto first = controller.getLastFieldHocbfDebugProfile()[0];
  ASSERT_FALSE(first.selected_source_ids.empty());
  ros::WallDuration(.08).sleep();
  controller.run();
  const auto later = controller.getLastFieldHocbfDebugProfile()[0];
  EXPECT_EQ(later.selected_source_ids, first.selected_source_ids);
  EXPECT_NEAR(later.h, first.h, 1e-12);

  auto future = cloud(8);
  future->header.stamp = epoch + ros::Duration(.1);
  controller.fieldOccupancyCallback(future);
  controller.run();
  EXPECT_EQ(controller.getLastFieldHocbfDebugProfile()[0].candidate_count, 3u);
  EXPECT_FALSE(controller.hasFreshFieldSnapshot(epoch + ros::Duration(4.0)));
  controller.setControlCycleStamp(epoch + ros::Duration(4.0));
  controller.run();
  EXPECT_EQ(controller.getLastFieldHocbfDebugProfile()[0].field_status, "stale");
}

TEST(PathVisualization, PublishedPathsAreBoundedWhileOdometryStaysAtControlRate) {
  ros::NodeHandle nh;
  auto pnh = caseParameters("bounded_paths");
  coni_mpc::NumSimMpc controller(nh, pnh, 0);
  initializeHover(controller);
  int paths = 0, odometry = 0;
  nav_msgs::Path last;
  auto path_sub = nh.subscribe<nav_msgs::Path>(
      acado_mpc_common::resolveTopicName("coni_mpc/quad_path0"), 100,
      [&](const nav_msgs::Path::ConstPtr& p) { ++paths; last = *p; });
  auto odom_sub = nh.subscribe<nav_msgs::Odometry>(
      acado_mpc_common::resolveTopicName("coni_mpc/quad_odom0"), 100,
      [&](const nav_msgs::Odometry::ConstPtr&) { ++odometry; });
  for (int i = 0; i < 100 && path_sub.getNumPublishers() == 0; ++i) {
    ros::spinOnce();
    ros::WallDuration(.005).sleep();
  }
  ASSERT_GT(path_sub.getNumPublishers(), 0u);
  for (int i = 0; i < 20; ++i) {
    nav_msgs::Odometry quad;
    quad.header.frame_id = "map";
    quad.header.stamp = ros::Time::now();
    quad.pose.pose.orientation.w = 1.0;
    quad.pose.pose.position.x = i;
    quad.pose.pose.position.z = 1;
    controller.setQuadOdom(quad, 0);
    controller.runReactiveBaselineStep();
    ros::spinOnce();
    ros::WallDuration(.02).sleep();
  }
  ros::WallDuration(.02).sleep();
  ros::spinOnce();
  EXPECT_GE(odometry, 15);
  EXPECT_GE(paths, 2);
  EXPECT_LE(paths, 7);
  EXPECT_LT(paths, odometry / 2);
  ASSERT_EQ(last.poses.size(), 4u);
  EXPECT_EQ(last.header.frame_id, "map");
  for (std::size_t i = 1; i < last.poses.size(); ++i) {
    EXPECT_GT(last.poses[i].pose.position.x, last.poses[i - 1].pose.position.x);
  }
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "field_snapshot_integration_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
