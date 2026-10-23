#include "acado_mpc/mpc_controller.h"

#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>

namespace {

using Controller = acado_mpc::MpcController<double>;
using Params = acado_mpc::MpcParams<double>;
using Inputs = Eigen::Matrix<double, acado_mpc::kInputSize,
                             acado_mpc::kSamples>;
using States = Eigen::Matrix<double, acado_mpc::kStateSize,
                             acado_mpc::kSamples + 1>;
using NonInertialProfile = Controller::NonInertialProfile;
using WorldProfile = Controller::WorldVelocityFrameProfile;

class RosEnvironment final : public ::testing::Environment {
 public:
  void SetUp() override {
    ASSERT_TRUE(ros::master::check());
  }
};

::testing::Environment* const kRosEnvironment =
    ::testing::AddGlobalTestEnvironment(new RosEnvironment);

struct RunContext {
  acado_mpc_common::QuadRelativeEstimate estimate;
  acado_mpc_common::RelativeTrajectory reference;
  NonInertialProfile omega = NonInertialProfile::Zero();
  NonInertialProfile beta = NonInertialProfile::Zero();
  NonInertialProfile car_acceleration = NonInertialProfile::Zero();
  Eigen::Vector3d frame_velocity = Eigen::Vector3d::Zero();
  WorldProfile valid_world = WorldProfile::Zero();
  WorldProfile invalid_world = WorldProfile::Zero();

  RunContext()
      : estimate(Eigen::Vector3d(0.0, 0.0, 1.0),
                 Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                 Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                 Eigen::Vector3d::Zero()) {
    acado_mpc_common::RelativeTrajectoryPoint point;
    point.position = Eigen::Vector3d(0.0, 0.0, 1.0);
    point.velocity.setZero();
    point.orientation = Eigen::Quaterniond::Identity();
    reference.points.push_back(point);

    for (int stage = 0; stage <= acado_mpc::kSamples; ++stage) {
      valid_world(0, stage) = 1.0;
      valid_world(4, stage) = 1.0;
      valid_world(8, stage) = 1.0;
    }
    invalid_world = valid_world;
    invalid_world(0, 0) = std::numeric_limits<double>::quiet_NaN();
  }
};

Params loadParams() {
  ros::NodeHandle pnh("~");
  Params params;
  EXPECT_TRUE(params.loadParameters(pnh));
  params.cbf_enabled_ = false;
  params.cbf_use_field_hocbf_ = false;
  params.cbf_obstacles_.clear();
  params.cbf_obstacle_profiles_.clear();
  params.cbf_slack_max_ = 10.0;
  // The controller constructor loaded the same private parameter namespace.
  // Keep repeated fault calls from needlessly reconfiguring the solver.
  params.changed_ = false;
  return params;
}

acado_mpc_common::ControlCommand runWithWorldProfile(
    Controller& controller, Params& params, const RunContext& context,
    const WorldProfile& world_profile, const ros::Time& evaluation_time) {
  return controller.run(
      context.estimate, context.reference, params,
      Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
      &context.omega, &context.beta, &context.car_acceleration,
      &context.frame_velocity, &world_profile, &evaluation_time);
}

ros::Time atSeconds(double seconds) {
  ros::Time result;
  result.fromSec(seconds);
  return result;
}

ros::Time afterMilliseconds(const ros::Time& origin, int milliseconds) {
  return origin + ros::Duration(static_cast<double>(milliseconds) / 1000.0);
}

Inputs distinguishablePlan(double offset) {
  Inputs plan = Inputs::Zero();
  for (int stage = 0; stage < acado_mpc::kSamples; ++stage) {
    plan(acado_mpc::INPUT::kVelCmdX, stage) = offset + 0.01 * stage;
    plan(acado_mpc::INPUT::kVelCmdY, stage) = -offset - 0.02 * stage;
    plan(acado_mpc::INPUT::kVelCmdZ, stage) = 0.1 + 0.005 * stage;
    plan(acado_mpc::INPUT::kYawRate, stage) = -0.2 + 0.01 * stage;
    plan(acado_mpc::INPUT::kSlack0, stage) = 0.01 * stage;
    plan(acado_mpc::INPUT::kSlack1, stage) = 0.02 * stage;
    plan(acado_mpc::INPUT::kSlack2, stage) = 0.03 * stage;
  }
  return plan;
}

void expectCommandMatches(
    const acado_mpc_common::ControlCommand& command,
    const Eigen::Matrix<double, acado_mpc::kInputSize, 1>& input) {
  EXPECT_TRUE(command.velocity_cmd.isApprox(input.head<3>(), 1.0e-12));
  EXPECT_NEAR(command.yaw_rate, input(acado_mpc::INPUT::kYawRate), 1.0e-12);
  EXPECT_NEAR(command.slack,
              input.segment<3>(acado_mpc::INPUT::kSlack0).sum(), 1.0e-12);
}

void expectAnchorMatches(
    const Controller& controller,
    const Eigen::Matrix<double, acado_mpc::kInputSize, 1>& input) {
  EXPECT_TRUE(controller.getSlewRateAnchor().isApprox(input, 1.0e-12));
  EXPECT_NEAR(controller.getLatestSlack(),
              input.segment<3>(acado_mpc::INPUT::kSlack0).sum(), 1.0e-12);
}

std::unique_ptr<Controller> makeController(int id) {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  return std::make_unique<Controller>(
      nh, pnh, id, "route_a/trajectory_predicted_",
      "route_a/trajectory_predicted_world_");
}

TEST(RouteAFallback, FailureBeforeAnySuccessfulPlanReturnsZero) {
  RunContext context;
  Params params = loadParams();
  auto controller = makeController(100);
  const ros::Time evaluation_time = atSeconds(10.0);

  const auto command = runWithWorldProfile(
      *controller, params, context, context.invalid_world, evaluation_time);

  EXPECT_FALSE(controller->getLastSolveOk());
  EXPECT_TRUE(controller->getLastCommandUsedFallback());
  EXPECT_FALSE(controller->getLastFallbackHadPreviousValid());
  EXPECT_EQ(controller->getLastAppliedPlanStage(), -1);
  EXPECT_FALSE(controller->getLastFallbackPlanExhausted());
  EXPECT_EQ(controller->getPlanGeneration(), 0u);
  EXPECT_TRUE(command.velocity_cmd.isZero(0.0));
  EXPECT_DOUBLE_EQ(command.yaw_rate, 0.0);
  EXPECT_DOUBLE_EQ(command.slack, 0.0);
  EXPECT_TRUE(controller->getSlewRateAnchor().isZero(0.0));
}

TEST(RouteAFallback, UsesElapsedPredictionStagesAndKeepsInstancesIndependent) {
  RunContext context;
  Params params_a = loadParams();
  Params params_b = loadParams();
  auto controller_a = makeController(101);
  auto controller_b = makeController(102);
  const ros::Time origin_a = atSeconds(100.0);
  const ros::Time origin_b = atSeconds(200.0);

  const auto first_a = runWithWorldProfile(
      *controller_a, params_a, context, context.valid_world, origin_a);
  ASSERT_TRUE(controller_a->getLastSolveOk());
  ASSERT_FALSE(controller_a->getLastCommandUsedFallback());
  ASSERT_TRUE(first_a.velocity_cmd.allFinite());
  ASSERT_EQ(controller_a->getPlanGeneration(), 1u);
  ASSERT_NEAR(controller_a->getPredictionDt(), 0.1, 1.0e-12);

  const auto first_b = runWithWorldProfile(
      *controller_b, params_b, context, context.valid_world, origin_b);
  ASSERT_TRUE(controller_b->getLastSolveOk());
  ASSERT_TRUE(first_b.velocity_cmd.allFinite());
  ASSERT_EQ(controller_b->getPlanGeneration(), 1u);

  const States states_a = controller_a->getPredictedStates();
  const States states_b = controller_b->getPredictedStates();
  const Inputs plan_a = distinguishablePlan(0.4);
  const Inputs plan_b = distinguishablePlan(1.4);
  controller_a->restoreSuccessfulPrediction(states_a, plan_a, 0.0);
  controller_b->restoreSuccessfulPrediction(states_b, plan_b, 0.0);
  ASSERT_EQ(controller_a->getLastAppliedPlanStage(), 0);
  ASSERT_EQ(controller_b->getLastAppliedPlanStage(), 0);
  ASSERT_EQ(controller_a->getPlanGeneration(), 1u);
  ASSERT_EQ(controller_b->getPlanGeneration(), 1u);

  auto command_a = runWithWorldProfile(
      *controller_a, params_a, context, context.invalid_world,
      afterMilliseconds(origin_a, 20));
  EXPECT_EQ(controller_a->getLastAppliedPlanStage(), 0);
  EXPECT_EQ(controller_b->getLastAppliedPlanStage(), 0);
  expectCommandMatches(command_a, plan_a.col(0));
  expectAnchorMatches(*controller_a, plan_a.col(0));

  // Repeated calls at one logical epoch must not consume the cached plan.
  command_a = runWithWorldProfile(
      *controller_a, params_a, context, context.invalid_world,
      afterMilliseconds(origin_a, 20));
  EXPECT_EQ(controller_a->getLastAppliedPlanStage(), 0);
  expectCommandMatches(command_a, plan_a.col(0));

  const auto command_b = runWithWorldProfile(
      *controller_b, params_b, context, context.invalid_world,
      afterMilliseconds(origin_b, 100));
  EXPECT_EQ(controller_b->getLastAppliedPlanStage(), 1);
  EXPECT_EQ(controller_a->getLastAppliedPlanStage(), 0);
  expectCommandMatches(command_b, plan_b.col(1));
  expectAnchorMatches(*controller_b, plan_b.col(1));

  // Eighteen 20 ms failure cycles cover only 360 ms of a 100 ms-stage plan.
  for (int cycle = 1; cycle <= 18; ++cycle) {
    const int expected_stage = (20 * cycle) / 100;
    command_a = runWithWorldProfile(
        *controller_a, params_a, context, context.invalid_world,
        afterMilliseconds(origin_a, 20 * cycle));
    ASSERT_FALSE(controller_a->getLastSolveOk());
    ASSERT_TRUE(controller_a->getLastCommandUsedFallback());
    ASSERT_TRUE(controller_a->getLastFallbackHadPreviousValid());
    ASSERT_EQ(controller_a->getLastAppliedPlanStage(), expected_stage);
    EXPECT_EQ(controller_a->getPlanGeneration(), 1u);
    expectCommandMatches(command_a, plan_a.col(expected_stage));
    expectAnchorMatches(*controller_a, plan_a.col(expected_stage));
    EXPECT_FALSE(controller_a->getLastFallbackPlanExhausted());
  }

  command_a = runWithWorldProfile(
      *controller_a, params_a, context, context.invalid_world,
      afterMilliseconds(origin_a, 100 * (acado_mpc::kSamples - 1)));
  EXPECT_EQ(controller_a->getLastAppliedPlanStage(), acado_mpc::kSamples - 1);
  EXPECT_TRUE(controller_a->getLastFallbackPlanExhausted());
  expectCommandMatches(command_a, plan_a.col(acado_mpc::kSamples - 1));
  expectAnchorMatches(*controller_a, plan_a.col(acado_mpc::kSamples - 1));

  command_a = runWithWorldProfile(
      *controller_a, params_a, context, context.invalid_world,
      afterMilliseconds(origin_a, 100 * (acado_mpc::kSamples + 5)));
  EXPECT_EQ(controller_a->getLastAppliedPlanStage(), acado_mpc::kSamples - 1);
  EXPECT_TRUE(controller_a->getLastFallbackPlanExhausted());
  expectCommandMatches(command_a, plan_a.col(acado_mpc::kSamples - 1));

  const ros::Time recovery_time =
      afterMilliseconds(origin_a, 100 * (acado_mpc::kSamples + 6));
  const auto recovered = runWithWorldProfile(
      *controller_a, params_a, context, context.valid_world, recovery_time);
  ASSERT_TRUE(controller_a->getLastSolveOk());
  ASSERT_FALSE(controller_a->getLastCommandUsedFallback());
  ASSERT_EQ(controller_a->getLastAppliedPlanStage(), 0);
  ASSERT_FALSE(controller_a->getLastFallbackPlanExhausted());
  ASSERT_EQ(controller_a->getPlanGeneration(), 2u);
  ASSERT_TRUE(recovered.velocity_cmd.allFinite());
  const Inputs replacement = controller_a->getPredictedInputs();

  command_a = runWithWorldProfile(
      *controller_a, params_a, context, context.invalid_world,
      afterMilliseconds(recovery_time, 20));
  EXPECT_EQ(controller_a->getLastAppliedPlanStage(), 0);
  EXPECT_EQ(controller_a->getPlanGeneration(), 2u);
  expectCommandMatches(command_a, replacement.col(0));
  expectAnchorMatches(*controller_a, replacement.col(0));

  EXPECT_EQ(controller_b->getLastAppliedPlanStage(), 1);
  EXPECT_EQ(controller_b->getPlanGeneration(), 1u);
}

TEST(RouteAFallback, ChangesStageAtPredictionBoundaryAndJumpsSkippedCycles) {
  RunContext context;
  Params params = loadParams();
  auto controller = makeController(104);
  const ros::Time origin = atSeconds(300.0);

  ASSERT_TRUE(runWithWorldProfile(
      *controller, params, context, context.valid_world, origin)
                  .velocity_cmd.allFinite());
  const Inputs plan = distinguishablePlan(2.0);
  controller->restoreSuccessfulPrediction(
      controller->getPredictedStates(), plan, 0.0);

  auto command = runWithWorldProfile(
      *controller, params, context, context.invalid_world,
      afterMilliseconds(origin, 99));
  EXPECT_EQ(controller->getLastAppliedPlanStage(), 0);
  expectCommandMatches(command, plan.col(0));

  command = runWithWorldProfile(
      *controller, params, context, context.invalid_world,
      afterMilliseconds(origin, 100));
  EXPECT_EQ(controller->getLastAppliedPlanStage(), 1);
  expectCommandMatches(command, plan.col(1));

  command = runWithWorldProfile(
      *controller, params, context, context.invalid_world,
      afterMilliseconds(origin, 350));
  EXPECT_EQ(controller->getLastAppliedPlanStage(), 3);
  expectCommandMatches(command, plan.col(3));
}

TEST(RouteAFallback, ValidRestorePreservesPlanOriginAndGeneration) {
  RunContext context;
  Params params = loadParams();
  auto controller = makeController(106);
  const ros::Time origin = atSeconds(350.0);

  ASSERT_TRUE(runWithWorldProfile(
      *controller, params, context, context.valid_world, origin)
                  .velocity_cmd.allFinite());
  const States states = controller->getPredictedStates();
  const Inputs restored_plan = distinguishablePlan(2.5);

  auto command = runWithWorldProfile(
      *controller, params, context, context.invalid_world,
      afterMilliseconds(origin, 150));
  ASSERT_EQ(controller->getLastAppliedPlanStage(), 1);
  ASSERT_EQ(controller->getPlanGeneration(), 1u);

  controller->restoreSuccessfulPrediction(states, restored_plan, 0.0);
  EXPECT_EQ(controller->getLastAppliedPlanStage(), 1);
  EXPECT_EQ(controller->getPlanGeneration(), 1u);

  command = runWithWorldProfile(
      *controller, params, context, context.invalid_world,
      afterMilliseconds(origin, 250));
  EXPECT_EQ(controller->getLastAppliedPlanStage(), 2);
  EXPECT_EQ(controller->getPlanGeneration(), 1u);
  expectCommandMatches(command, restored_plan.col(2));
}

TEST(RouteAFallback, SolverRejectionUsesCacheAndInvalidRestoreCannotPolluteIt) {
  RunContext context;
  Params params = loadParams();
  auto controller = makeController(103);
  const ros::Time origin = atSeconds(400.0);

  const auto first = runWithWorldProfile(
      *controller, params, context, context.valid_world, origin);
  ASSERT_TRUE(controller->getLastSolveOk());
  ASSERT_TRUE(first.velocity_cmd.allFinite());
  ASSERT_EQ(controller->getPlanGeneration(), 1u);
  const Inputs plan_p = controller->getPredictedInputs();
  const States states_p = controller->getPredictedStates();

  // At stage 0, v=0 and tau=0.2 with |a_xy|<=10 restricts the nominal
  // command change to 2 m/s. A world-frame origin velocity of 20 m/s cannot
  // simultaneously satisfy the 5 m/s world-speed envelope, so the real ACADO
  // solve must be rejected while all supplied data remain finite.
  WorldProfile infeasible_world = context.valid_world;
  infeasible_world.row(9).setConstant(20.0);
  const auto rejected = runWithWorldProfile(
      *controller, params, context, infeasible_world,
      afterMilliseconds(origin, 20));
  ASSERT_FALSE(controller->getLastSolveOk());
  ASSERT_TRUE(controller->getLastCommandUsedFallback());
  ASSERT_EQ(controller->getLastAppliedPlanStage(), 0);
  ASSERT_EQ(controller->getPlanGeneration(), 1u);
  expectCommandMatches(rejected, plan_p.col(0));
  expectAnchorMatches(*controller, plan_p.col(0));

  Inputs invalid_restore = plan_p;
  invalid_restore(0, 0) = std::numeric_limits<double>::quiet_NaN();
  controller->restoreSuccessfulPrediction(states_p, invalid_restore, 0.0);
  EXPECT_EQ(controller->getLastAppliedPlanStage(), 0);
  EXPECT_EQ(controller->getPlanGeneration(), 1u);

  const auto second_failure = runWithWorldProfile(
      *controller, params, context, context.invalid_world,
      afterMilliseconds(origin, 100));
  ASSERT_FALSE(controller->getLastSolveOk());
  ASSERT_EQ(controller->getLastAppliedPlanStage(), 1);
  ASSERT_EQ(controller->getPlanGeneration(), 1u);
  expectCommandMatches(second_failure, plan_p.col(1));
  expectAnchorMatches(*controller, plan_p.col(1));

  const ros::Time recovery_time = afterMilliseconds(origin, 200);
  const auto recovered = runWithWorldProfile(
      *controller, params, context, context.valid_world, recovery_time);
  ASSERT_TRUE(controller->getLastSolveOk());
  ASSERT_FALSE(controller->getLastCommandUsedFallback());
  ASSERT_EQ(controller->getLastAppliedPlanStage(), 0);
  ASSERT_EQ(controller->getPlanGeneration(), 2u);
  ASSERT_TRUE(recovered.velocity_cmd.allFinite());
  const Inputs plan_q = controller->getPredictedInputs();

  const auto after_recovery_failure = runWithWorldProfile(
      *controller, params, context, context.invalid_world,
      afterMilliseconds(recovery_time, 100));
  ASSERT_FALSE(controller->getLastSolveOk());
  ASSERT_EQ(controller->getLastAppliedPlanStage(), 1);
  ASSERT_EQ(controller->getPlanGeneration(), 2u);
  expectCommandMatches(after_recovery_failure, plan_q.col(1));
  expectAnchorMatches(*controller, plan_q.col(1));
}

TEST(RouteAFallback, ClockRollbackInvalidatesFutureDatedCache) {
  RunContext context;
  Params params = loadParams();
  auto controller = makeController(105);
  const ros::Time origin = atSeconds(500.0);

  ASSERT_TRUE(runWithWorldProfile(
      *controller, params, context, context.valid_world, origin)
                  .velocity_cmd.allFinite());
  ASSERT_EQ(controller->getPlanGeneration(), 1u);

  const auto command = runWithWorldProfile(
      *controller, params, context, context.invalid_world,
      afterMilliseconds(origin, -1));
  EXPECT_TRUE(controller->getLastCommandUsedFallback());
  EXPECT_FALSE(controller->getLastFallbackHadPreviousValid());
  EXPECT_EQ(controller->getLastAppliedPlanStage(), -1);
  EXPECT_EQ(controller->getPlanGeneration(), 1u);
  EXPECT_TRUE(command.velocity_cmd.isZero(0.0));

  // A new finite solve at the reset epoch establishes a fresh cache.
  const ros::Time reset_epoch = afterMilliseconds(origin, -1);
  ASSERT_TRUE(runWithWorldProfile(
      *controller, params, context, context.valid_world, reset_epoch)
                  .velocity_cmd.allFinite());
  EXPECT_EQ(controller->getPlanGeneration(), 2u);
  const Inputs replacement = controller->getPredictedInputs();
  const auto recovered_fallback = runWithWorldProfile(
      *controller, params, context, context.invalid_world,
      afterMilliseconds(reset_epoch, 100));
  EXPECT_EQ(controller->getLastAppliedPlanStage(), 1);
  expectCommandMatches(recovered_fallback, replacement.col(1));
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "route_a_fallback_test",
            ros::init_options::NoSigintHandler);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
