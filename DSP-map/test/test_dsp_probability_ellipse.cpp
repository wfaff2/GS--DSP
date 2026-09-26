#include <gtest/gtest.h>

#include "dsp_probability_ellipse.h"

using dsp_probability::PersistentIdAllocator;
using dsp_probability::ProbabilityEllipseConfig;
using dsp_probability::WeightedMoments;
using dsp_probability::constructProbabilityEllipse;

namespace {

WeightedMoments translatedMoments(double cx, double cy, double vx = 0.0,
                                  double vy = 0.0, double time = 0.0) {
    WeightedMoments moments;
    const double dx[] = {-2.0, -1.0, 1.0, 2.0};
    const double dy[] = {-0.2, 0.2, -0.2, 0.2};
    for(int i = 0; i < 4; ++i) {
        moments.add(cx + vx * time + dx[i], cy + vy * time + dy[i], 1.5, 1.0);
    }
    return moments;
}

TEST(DspProbabilityEllipse, StationaryCenter) {
    const auto result = constructProbabilityEllipse(translatedMoments(3.0, -2.0), {});
    ASSERT_TRUE(result.valid);
    EXPECT_NEAR(result.center.x(), 3.0, 1e-12);
    EXPECT_NEAR(result.center.y(), -2.0, 1e-12);
    EXPECT_NEAR(result.center.z(), 1.5, 1e-12);
}

TEST(DspProbabilityEllipse, ConstantVelocityCenterRelation) {
    const auto initial = constructProbabilityEllipse(translatedMoments(1.0, 2.0), {});
    const auto future = constructProbabilityEllipse(
            translatedMoments(1.0, 2.0, 0.7, -0.3, 2.0), {});
    ASSERT_TRUE(initial.valid && future.valid);
    EXPECT_NEAR(future.center.x(), initial.center.x() + 1.4, 1e-12);
    EXPECT_NEAR(future.center.y(), initial.center.y() - 0.6, 1e-12);
}

TEST(DspProbabilityEllipse, AnisotropicAxesAndYaw) {
    WeightedMoments moments;
    const double c = std::sqrt(0.5);
    for(double s : {-2.0, -1.0, 1.0, 2.0}) {
        moments.add(s * c, s * c, 0.0, 1.0);
    }
    const auto result = constructProbabilityEllipse(moments, {});
    ASSERT_TRUE(result.valid);
    EXPECT_GT(result.semi_major, result.semi_minor);
    EXPECT_NEAR(result.yaw, std::acos(-1.0) / 4.0, 1e-6);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> covariance_solver(result.covariance);
    ASSERT_EQ(covariance_solver.info(), Eigen::Success);
    EXPECT_GT(covariance_solver.eigenvalues().minCoeff(), 0.0);
}

TEST(DspProbabilityEllipse, ProbabilityMassScalingIsMonotonic) {
    ProbabilityEllipseConfig config;
    config.probability_mass = 0.90;
    const auto p90 = constructProbabilityEllipse(translatedMoments(0.0, 0.0), config);
    config.probability_mass = 0.95;
    const auto p95 = constructProbabilityEllipse(translatedMoments(0.0, 0.0), config);
    config.probability_mass = 0.99;
    const auto p99 = constructProbabilityEllipse(translatedMoments(0.0, 0.0), config);
    EXPECT_LT(p90.semi_major, p95.semi_major);
    EXPECT_LT(p95.semi_major, p99.semi_major);
    EXPECT_LT(p90.semi_minor, p95.semi_minor);
    EXPECT_LT(p95.semi_minor, p99.semi_minor);
}

TEST(DspProbabilityEllipse, PersistentIdInheritanceAndAllocation) {
    PersistentIdAllocator allocator(10);
    const auto first = allocator.inheritOrAllocate(-1);
    EXPECT_EQ(first, 10);
    EXPECT_EQ(allocator.inheritOrAllocate(first), 10);
    EXPECT_EQ(allocator.inheritOrAllocate(-1), 11);
}

TEST(DspProbabilityEllipse, TwoObjectsRemainSeparated) {
    const auto first = constructProbabilityEllipse(translatedMoments(-4.0, 1.0), {});
    const auto second = constructProbabilityEllipse(translatedMoments(5.0, -1.0), {});
    ASSERT_TRUE(first.valid && second.valid);
    EXPECT_GT((first.center - second.center).norm(), 8.0);
}

TEST(DspProbabilityEllipse, RejectsNonFiniteSamplesUnderFastMath) {
    WeightedMoments moments;
    EXPECT_FALSE(moments.add(std::numeric_limits<double>::quiet_NaN(),
                             0.0, 0.0, 1.0));
    EXPECT_FALSE(moments.add(0.0, std::numeric_limits<double>::infinity(),
                             0.0, 1.0));
    EXPECT_FALSE(moments.add(0.0, 0.0, 0.0,
                             std::numeric_limits<double>::infinity()));
    EXPECT_EQ(moments.sample_count, 0U);
    EXPECT_DOUBLE_EQ(moments.weight, 0.0);

    ProbabilityEllipseConfig config;
    config.minimum_weight = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(constructProbabilityEllipse(translatedMoments(0.0, 0.0), config).valid);
}

}  // namespace

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
