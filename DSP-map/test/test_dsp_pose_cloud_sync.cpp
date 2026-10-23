#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <limits>
#include <thread>

#include "dsp_pose_cloud_sync.h"

namespace {

using Pairer = dsp_sync::TimestampPairer<double, int>;

TEST(DspPoseCloudSync, MatchesExactTimestamp) {
    Pairer pairer(4, std::chrono::milliseconds(50));
    pairer.addPose(2.0, 20.0);
    pairer.addCloud(2.0, 7);
    Pairer::Match match;
    EXPECT_EQ(pairer.waitNext(match), Pairer::WaitEvent::kMatch);
    EXPECT_TRUE(match.exact);
    EXPECT_DOUBLE_EQ(match.pose_before, 20.0);
    EXPECT_EQ(match.cloud, 7);
}

TEST(DspPoseCloudSync, InterpolatesOnlyWithinBracket) {
    Pairer pairer(4, std::chrono::milliseconds(50));
    pairer.addPose(1.0, 10.0);
    pairer.addPose(3.0, 30.0);
    pairer.addCloud(2.0, 8);
    Pairer::Match match;
    ASSERT_EQ(pairer.waitNext(match), Pairer::WaitEvent::kMatch);
    EXPECT_FALSE(match.exact);
    EXPECT_DOUBLE_EQ(match.pose_before, 10.0);
    EXPECT_DOUBLE_EQ(match.pose_after, 30.0);
    EXPECT_DOUBLE_EQ(match.interpolation_factor, 0.5);
}

TEST(DspPoseCloudSync, CloudFirstWaitsForOutOfOrderBracket) {
    Pairer pairer(4, std::chrono::milliseconds(200));
    const auto submission = pairer.addCloud(2.0, 9);
    EXPECT_TRUE(submission.waiting_for_pose);
    auto result = std::async(std::launch::async, [&pairer] {
        Pairer::Match match;
        const auto event = pairer.waitNext(match);
        return std::make_pair(event, match);
    });
    pairer.addPose(3.0, 30.0);
    pairer.addPose(1.0, 10.0);
    const auto output = result.get();
    ASSERT_EQ(output.first, Pairer::WaitEvent::kMatch);
    EXPECT_DOUBLE_EQ(output.second.interpolation_factor, 0.5);
}

TEST(DspPoseCloudSync, DuplicatePoseReplacesSample) {
    Pairer pairer(4, std::chrono::milliseconds(50));
    pairer.addPose(1.0, 10.0);
    pairer.addPose(1.0, 11.0);
    pairer.addCloud(1.0, 1);
    Pairer::Match match;
    ASSERT_EQ(pairer.waitNext(match), Pairer::WaitEvent::kMatch);
    EXPECT_DOUBLE_EQ(match.pose_before, 11.0);
    EXPECT_EQ(pairer.counts().duplicate_poses, 1u);
}

TEST(DspPoseCloudSync, BoundsPoseHistoryAndPendingCloud) {
    Pairer pairer(3, std::chrono::milliseconds(50));
    for(int i = 0; i < 6; ++i) {
        pairer.addPose(static_cast<double>(i), static_cast<double>(i));
    }
    EXPECT_EQ(pairer.poseCount(), 3u);
    EXPECT_EQ(pairer.counts().dropped_poses, 3u);

    pairer.addCloud(10.0, 1);
    const auto second = pairer.addCloud(11.0, 2);
    EXPECT_TRUE(second.replaced_pending_cloud);
    EXPECT_EQ(pairer.counts().replaced_clouds, 1u);
}

TEST(DspPoseCloudSync, OlderOrDuplicateCloudCannotReplaceNewerPendingCloud) {
    Pairer pairer(4, std::chrono::milliseconds(50));
    EXPECT_TRUE(pairer.addCloud(3.0, 30).accepted);
    EXPECT_TRUE(pairer.addCloud(2.0, 20).dropped_nonmonotonic);
    EXPECT_TRUE(pairer.addCloud(3.0, 31).dropped_nonmonotonic);
    pairer.addPose(3.0, 300.0);

    Pairer::Match match;
    ASSERT_EQ(pairer.waitNext(match), Pairer::WaitEvent::kMatch);
    EXPECT_DOUBLE_EQ(match.cloud_stamp, 3.0);
    EXPECT_EQ(match.cloud, 30);
    EXPECT_EQ(pairer.counts().nonmonotonic_clouds, 2u);
}

TEST(DspPoseCloudSync, RepeatedProcessedCloudStampIsDropped) {
    Pairer pairer(4, std::chrono::milliseconds(50));
    pairer.addPose(1.0, 10.0);
    ASSERT_TRUE(pairer.addCloud(1.0, 1).accepted);
    Pairer::Match match;
    ASSERT_EQ(pairer.waitNext(match), Pairer::WaitEvent::kMatch);

    const auto repeated = pairer.addCloud(1.0, 2);
    EXPECT_FALSE(repeated.accepted);
    EXPECT_TRUE(repeated.dropped_nonmonotonic);
    EXPECT_EQ(pairer.counts().nonmonotonic_clouds, 1u);
}

TEST(DspPoseCloudSync, RejectsInvalidAndShutdownCloudSubmissions) {
    Pairer pairer(4, std::chrono::milliseconds(50));
    EXPECT_TRUE(pairer.addCloud(0.0, 1).dropped_invalid_stamp);
    EXPECT_TRUE(pairer.addCloud(
            std::numeric_limits<double>::quiet_NaN(), 2).dropped_invalid_stamp);
    pairer.shutdown();
    EXPECT_TRUE(pairer.addCloud(1.0, 3).dropped_shutdown);
    const auto counts = pairer.counts();
    EXPECT_EQ(counts.invalid_stamp_clouds, 2u);
    EXPECT_EQ(counts.shutdown_clouds, 1u);
}

TEST(DspPoseCloudSync, TimesOutUsingWallClockWithoutFallback) {
    Pairer pairer(4, std::chrono::milliseconds(5));
    pairer.addPose(1.0, 10.0);
    pairer.addCloud(2.0, 3);
    Pairer::Match match;
    EXPECT_EQ(pairer.waitNext(match), Pairer::WaitEvent::kTimedOut);
    EXPECT_EQ(pairer.counts().timed_out_clouds, 1u);
}

TEST(DspPoseCloudSync, ShutdownWakesWaiter) {
    Pairer pairer(4, std::chrono::hours(1));
    auto result = std::async(std::launch::async, [&pairer] {
        Pairer::Match match;
        return pairer.waitNext(match);
    });
    pairer.shutdown();
    EXPECT_EQ(result.get(), Pairer::WaitEvent::kShutdown);
}

}  // namespace

