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
  stage.config.seed_count = 1;
  fh::PruneStats stats;
  stage.expected = fr::replayStage(snapshot, cycle, stage, &stats);
  ASSERT_TRUE(stage.expected.active);
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
