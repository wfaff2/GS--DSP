#include "coni_mpc/field_replay_diagnostics.h"
#include <gtest/gtest.h>
#include <sstream>

namespace fr = coni_mpc::field_replay;
namespace fh = coni_mpc::field_hocbf;

TEST(FieldReplay, LosslessRoundTripPreservesNonInertialSelectionAndConstraint) {
  fr::SnapshotRecord snapshot;
  snapshot.quad_id = 2;
  snapshot.snapshot_id = 17;
  snapshot.stamp_sec = 100.25;
  snapshot.frame_id = "map";
  snapshot.last_occupied_stage = 1;
  for (int i = 0; i < 4; ++i) {
    fh::Point point;
    point.position_world = Eigen::Vector3d(3.3 + i * .2, -3.7, 2.);
    point.velocity_world = Eigen::Vector3d(.1, -.2, 0.);
    point.occupancy = .4 + i * .1;
    point.prediction_time = .1;
    point.stage_index = 1;
    point.source_id = i;
    snapshot.points.push_back(point);
  }
  fr::CycleRecord cycle;
  cycle.quad_id = 2;
  cycle.snapshot_id = 17;
  cycle.step_idx = 5;
  cycle.field_available = true;
  cycle.field_age = .12;
  cycle.prediction_dt = .1;
  cycle.field_stamp = snapshot.stamp_sec;
  fr::StageRecord stage;
  stage.stage = 0;
  stage.source_stage = 1;
  stage.extrapolation_time = .02;
  stage.car_position = Eigen::Vector3d(3., -4., 0.);
  stage.car_velocity = Eigen::Vector3d(.7, .2, 0.);
  stage.world_q_non = Eigen::Quaterniond(Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitZ()));
  stage.omega_non = Eigen::Vector3d(0., 0., .15);
  stage.beta_non = Eigen::Vector3d(0., 0., .01);
  stage.a_car_non = Eigen::Vector3d(.1, -.03, 0.);
  stage.nominal_position = Eigen::Vector3d(.2, .1, 2.);
  stage.nominal_velocity = Eigen::Vector3d(.3, -.1, 0.);
  stage.lambda = Eigen::Vector3d::Constant(5.);
  fh::PruneStats stats;
  stage.expected = fr::replayStage(snapshot, cycle, stage, &stats);
  ASSERT_TRUE(stage.expected.active);
  stage.selection_stats = stats;
  stage.selected_source_ids = stats.selected_source_ids;
  cycle.stages.push_back(stage);
  std::stringstream bytes(std::ios::in | std::ios::out | std::ios::binary);
  ASSERT_TRUE(fr::writeSnapshot(bytes, snapshot));
  ASSERT_TRUE(fr::writeCycle(bytes, cycle));
  fr::RecordType type;
  fr::SnapshotRecord restored_snapshot;
  fr::CycleRecord restored_cycle;
  ASSERT_TRUE(fr::readNext(bytes, type, restored_snapshot, restored_cycle));
  ASSERT_EQ(type, fr::RecordType::kSnapshot);
  ASSERT_TRUE(fr::readNext(bytes, type, restored_snapshot, restored_cycle));
  ASSERT_EQ(type, fr::RecordType::kCycle);
  ASSERT_EQ(restored_cycle.quad_id, 2);
  ASSERT_EQ(restored_cycle.stages.size(), 1u);
  fh::PruneStats restored_stats;
  const auto result = fr::replayStage(restored_snapshot, restored_cycle,
                                    restored_cycle.stages.front(), &restored_stats);
  EXPECT_EQ(restored_stats.selected_source_ids, stage.selected_source_ids);
  EXPECT_DOUBLE_EQ(result.h, stage.expected.h);
  EXPECT_DOUBLE_EQ(result.hdot, stage.expected.hdot);
  EXPECT_DOUBLE_EQ(result.lf2, stage.expected.lf2);
  EXPECT_DOUBLE_EQ(result.b, stage.expected.b);
  EXPECT_DOUBLE_EQ(result.A.x(), stage.expected.A.x());
  EXPECT_DOUBLE_EQ(result.A.y(), stage.expected.A.y());
  EXPECT_DOUBLE_EQ(result.A.z(), stage.expected.A.z());
  EXPECT_EQ(restored_cycle.stages.front().selection_stats.candidate_count,
            stage.selection_stats.candidate_count);
  EXPECT_EQ((result.gradient - stage.expected.gradient).norm(), 0.0);
  EXPECT_EQ((result.hessian - stage.expected.hessian).norm(), 0.0);
}

TEST(FieldReplay, PublishedVelocityAndExtrapolationDoNotMoveWorldGridCenter) {
  fr::SnapshotRecord snapshot;
  fh::Point point;
  point.position_world = Eigen::Vector3d(2.0, -1.0, 0.7);
  point.velocity_world = Eigen::Vector3d(100.0, -200.0, 50.0);
  point.occupancy = 0.6;
  point.stage_index = 3;
  snapshot.points.push_back(point);
  fr::CycleRecord cycle;
  cycle.inertial_frame = false;
  fr::StageRecord stage;
  stage.source_stage = 3;
  stage.extrapolation_time = 9.0;
  stage.car_position = Eigen::Vector3d(0.4, 0.2, -0.1);
  stage.car_velocity = Eigen::Vector3d(0.3, -0.2, 0.1);
  stage.world_q_non = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.4, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()));
  stage.omega_non = Eigen::Vector3d(0.1, -0.05, 0.2);
  stage.beta_non = Eigen::Vector3d(-0.02, 0.03, 0.01);
  stage.a_car_non = Eigen::Vector3d(0.04, -0.01, 0.02);
  const Eigen::Matrix3d R = stage.world_q_non.inverse().toRotationMatrix();
  const Eigen::Vector3d center_non = R * (point.position_world - stage.car_position);
  stage.nominal_position = center_non + Eigen::Vector3d(0.5, -0.3, 0.4);
  stage.nominal_velocity = Eigen::Vector3d(0.2, 0.1, -0.1);
  stage.lambda = Eigen::Vector3d::Constant(5.0);
  const auto original = fr::replayStage(snapshot, cycle, stage);
  snapshot.points.front().velocity_world = Eigen::Vector3d(-900.0, 20.0, 70.0);
  stage.extrapolation_time = -17.0;
  const auto changed_diagnostic = fr::replayStage(snapshot, cycle, stage);
  ASSERT_TRUE(original.active);
  EXPECT_DOUBLE_EQ(original.h, changed_diagnostic.h);
  EXPECT_DOUBLE_EQ(original.hdot, changed_diagnostic.hdot);
  EXPECT_DOUBLE_EQ(original.lf2, changed_diagnostic.lf2);
  EXPECT_EQ((original.A - changed_diagnostic.A).norm(), 0.0);
}

TEST(FieldReplay, VersionOneHeaderIsRejected) {
  std::stringstream bytes(std::ios::in | std::ios::out | std::ios::binary);
  const std::uint64_t magic = fr::kFileMagic;
  const std::uint32_t version = 1;
  const std::uint32_t type = static_cast<std::uint32_t>(fr::RecordType::kCycle);
  bytes.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
  bytes.write(reinterpret_cast<const char*>(&version), sizeof(version));
  bytes.write(reinterpret_cast<const char*>(&type), sizeof(type));
  bytes.seekg(0);
  fr::RecordType restored_type;
  fr::SnapshotRecord snapshot;
  fr::CycleRecord cycle;
  EXPECT_FALSE(fr::readNext(bytes, restored_type, snapshot, cycle));
  EXPECT_TRUE(bytes.fail());
}

TEST(FieldReplay, FixedWorldCenterNonInertialDerivativesMatchFiniteDifference) {
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -2.0, 1.5).normalized();
  const Eigen::Vector3d omega = 0.35 * axis;
  const Eigen::Vector3d beta = -0.12 * axis;
  const Eigen::Vector3d car0(0.4, -0.3, 0.2);
  const Eigen::Vector3d car_v(0.3, 0.1, -0.15);
  const Eigen::Vector3d car_a(-0.04, 0.03, 0.02);
  const Eigen::Vector3d quad0(1.1, 0.2, 0.8);
  const Eigen::Vector3d quad_v(-0.1, 0.25, 0.12);
  const Eigen::Vector3d quad_a(0.08, -0.02, 0.05);
  const Eigen::Vector3d center_world(-0.2, 0.7, 0.45);

  fr::SnapshotRecord snapshot;
  fh::Point source;
  source.position_world = center_world;
  source.velocity_world = Eigen::Vector3d(90.0, -40.0, 10.0);
  source.occupancy = 0.75;
  source.stage_index = 0;
  snapshot.points.push_back(source);
  fr::CycleRecord cycle;
  cycle.inertial_frame = false;
  fr::StageRecord stage;
  stage.source_stage = 0;
  stage.car_position = car0;
  stage.car_velocity = car_v;
  stage.world_q_non = Eigen::Quaterniond::Identity();
  stage.omega_non = omega;
  stage.beta_non = beta;
  stage.a_car_non = car_a;
  stage.nominal_position = quad0 - car0;
  stage.nominal_velocity = quad_v - car_v - omega.cross(stage.nominal_position);
  stage.lambda = Eigen::Vector3d(4.0, 5.0, 6.0);
  const auto c = fr::replayStage(snapshot, cycle, stage);
  ASSERT_TRUE(c.active);

  const Eigen::Vector3d pddot = quad_a - car_a -
      2.0 * omega.cross(stage.nominal_velocity) -
      beta.cross(stage.nominal_position) -
      omega.cross(omega.cross(stage.nominal_position));
  const Eigen::Vector3d nominal_a_non = -car_a -
      2.0 * omega.cross(stage.nominal_velocity) -
      beta.cross(stage.nominal_position) -
      omega.cross(omega.cross(stage.nominal_position));
  const Eigen::Vector3d command =
      (pddot - nominal_a_non).cwiseQuotient(stage.lambda) +
      stage.nominal_velocity;

  const auto h_at = [&](double t) {
    const double angle = 0.35 * t - 0.5 * 0.12 * t * t;
    const Eigen::Matrix3d R_nw =
        Eigen::AngleAxisd(-angle, axis).toRotationMatrix();
    const Eigen::Vector3d car = car0 + car_v * t + 0.5 * car_a * t * t;
    const Eigen::Vector3d quad = quad0 + quad_v * t + 0.5 * quad_a * t * t;
    fh::KinematicPoint transformed;
    transformed.position = R_nw * (center_world - car);
    transformed.occupancy = source.occupancy;
    return fh::computeConstraint(fh::KinematicPoints{transformed},
        R_nw * (quad - car), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), stage.config, stage.lambda).h;
  };
  const double dt = 1.0e-5;
  const double hdot_fd = (h_at(dt) - h_at(-dt)) / (2.0 * dt);
  const double hddot_fd = (h_at(dt) - 2.0 * h_at(0.0) + h_at(-dt)) /
                           (dt * dt);
  EXPECT_NEAR(c.hdot, hdot_fd, 2e-8);
  EXPECT_NEAR(c.lf2 + c.A.dot(command), hddot_fd, 3e-5);
}

TEST(FieldReplay, EmptyCycleAndTruncatedPayloadAreDistinguishable) {
  fr::CycleRecord cycle;
  cycle.quad_id = 1;
  cycle.step_idx = 9;
  std::stringstream bytes(std::ios::in | std::ios::out | std::ios::binary);
  ASSERT_TRUE(fr::writeCycle(bytes, cycle));
  const std::string payload = bytes.str();
  fr::RecordType type;
  fr::SnapshotRecord snapshot;
  fr::CycleRecord restored;
  ASSERT_TRUE(fr::readNext(bytes, type, snapshot, restored));
  EXPECT_EQ(restored.step_idx, 9u);
  EXPECT_FALSE(restored.field_available);
  EXPECT_TRUE(restored.stages.empty());
  std::stringstream broken(payload.substr(0, payload.size() - 1),
                           std::ios::in | std::ios::binary);
  EXPECT_FALSE(fr::readNext(broken, type, snapshot, restored));
}
