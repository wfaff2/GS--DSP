#include <gtest/gtest.h>

#include <cmath>

#include "dsp_fov_grid.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;

void expectIndex(const dsp_fov::Vector3f &point, int expected_h) {
    int h = -1;
    int v = -1;
    ASSERT_TRUE(dsp_fov::fullLidarIndex(point, 3, 90, h, v));
    EXPECT_EQ(h, expected_h);
    EXPECT_EQ(v, 15);
}

TEST(DspFovGrid, IndexesAllCardinalDirections) {
    expectIndex({1.0f, 0.0f, 0.0f}, 60);
    expectIndex({0.0f, 1.0f, 0.0f}, 90);
    expectIndex({-1.0f, 0.0f, 0.0f}, 0);
    expectIndex({0.0f, -1.0f, 0.0f}, 30);
}

TEST(DspFovGrid, RearSeamNeighborsWrap) {
    int neighbors[9] = {};
    const int count = dsp_fov::neighborIndices(15, 120, 30, true, neighbors);
    EXPECT_EQ(count, 9);

    bool found_last_horizontal_bin = false;
    for(int i = 0; i < count; ++i) {
        if(neighbors[i] / 30 == 119) {
            found_last_horizontal_bin = true;
        }
    }
    EXPECT_TRUE(found_last_horizontal_bin);
}

TEST(DspFovGrid, UsesInverseSensorRotation) {
    const float half_yaw = 0.25f * kPi;
    const dsp_fov::Vector3f sensor = dsp_fov::worldToSensor(
            {0.0f, 2.0f, 0.0f}, std::cos(half_yaw), 0.0f, 0.0f,
            std::sin(half_yaw));
    EXPECT_NEAR(sensor.x, 2.0f, 1.0e-5f);
    EXPECT_NEAR(sensor.y, 0.0f, 1.0e-5f);

    int h = -1;
    int v = -1;
    ASSERT_TRUE(dsp_fov::fullLidarIndex(sensor, 3, 90, h, v));
    EXPECT_EQ(h, 60);
}

TEST(DspFovGrid, IncludesVerticalEdgesAndRejectsOutside) {
    int h = -1;
    int v = -1;
    ASSERT_TRUE(dsp_fov::fullLidarIndex({1.0f, 0.0f, 1.0f}, 3, 90, h, v));
    EXPECT_EQ(v, 29);
    ASSERT_TRUE(dsp_fov::fullLidarIndex({1.0f, 0.0f, -1.0f}, 3, 90, h, v));
    EXPECT_EQ(v, 0);
    EXPECT_FALSE(dsp_fov::fullLidarIndex({1.0f, 0.0f, 1.01f}, 3, 90, h, v));
    EXPECT_FALSE(dsp_fov::fullLidarIndex({1.0f, 0.0f, -1.01f}, 3, 90, h, v));
}

TEST(DspFovGrid, VerticalBoundaryNeighborsDoNotWrap) {
    int neighbors[9] = {};
    const int count = dsp_fov::neighborIndices(60 * 30, 120, 30, true, neighbors);
    EXPECT_EQ(count, 6);
    for(int i = 0; i < count; ++i) {
        EXPECT_GE(neighbors[i] % 30, 0);
        EXPECT_LE(neighbors[i] % 30, 1);
    }
}

TEST(DspFovGrid, LidarAllowsOneVoxelOfGeometryDiscretization) {
    EXPECT_TRUE(dsp_fov::clusterGeometryWithinLimits(
            2.61354f, 1.21463f, 0.1f, true));
    EXPECT_FALSE(dsp_fov::clusterGeometryWithinLimits(
            3.0f, 1.21463f, 0.1f, true));
}

TEST(DspFovGrid, LegacyCameraKeepsOriginalGeometryLimits) {
    EXPECT_FALSE(dsp_fov::clusterGeometryWithinLimits(
            2.61354f, 1.21463f, 0.1f, false));
    EXPECT_TRUE(dsp_fov::clusterGeometryWithinLimits(
            2.60f, 1.15f, 0.1f, false));
}

}  // namespace

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
