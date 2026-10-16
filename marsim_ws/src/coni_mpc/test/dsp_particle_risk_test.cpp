#include "coni_mpc/dsp_particle_risk.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace {

using coni_mpc::dsp_particle_risk::Config;
using coni_mpc::dsp_particle_risk::Particle;
using coni_mpc::dsp_particle_risk::ParticleSet;
using coni_mpc::dsp_particle_risk::hardRisk;
using coni_mpc::dsp_particle_risk::predictPosition;
using coni_mpc::dsp_particle_risk::predictPositions;
using coni_mpc::dsp_particle_risk::smoothRisk;

Particle particle(double x, double y, double z, double vx, double vy,
                  double vz, double weight) {
  Particle result;
  result.position = Eigen::Vector3d(x, y, z);
  result.velocity = Eigen::Vector3d(vx, vy, vz);
  result.weight = weight;
  return result;
}

Config config() {
  Config result;
  result.collision_radius = 1.0;
  result.smoothing_width = 0.5;
  return result;
}

double spatialRisk(double x, const ParticleSet& particles, const Config& cfg) {
  return smoothRisk(particles, Eigen::Vector3d(x, 0.0, 0.0), 0.0, cfg);
}

TEST(DspParticleRisk, ConstantVelocityPrediction) {
  const Particle p = particle(1.0, -2.0, 0.5, 0.4, -0.3, 0.2, 0.7);
  const Eigen::Vector3d expected(2.0, -2.75, 1.0);
  EXPECT_NEAR((predictPosition(p, 2.5) - expected).norm(), 0.0, 1e-12);

  const ParticleSet particles{p, particle(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                            0.2)};
  const auto predicted = predictPositions(particles, 1.0);
  ASSERT_EQ(predicted.size(), 2u);
  EXPECT_NEAR((predicted[0] - Eigen::Vector3d(1.4, -2.3, 0.7)).norm(), 0.0,
              1e-12);
  EXPECT_NEAR((predicted[1] - Eigen::Vector3d(-1.0, 1.0, 0.0)).norm(), 0.0,
              1e-12);
}

TEST(DspParticleRisk, HardAndSmoothValuesUseRawWeights) {
  const Config cfg = config();
  const ParticleSet particles{
      particle(0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7),  // inside
      particle(1.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8),  // transition
      particle(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9)};  // outside

  EXPECT_DOUBLE_EQ(hardRisk(particles, Eigen::Vector3d::Zero(), 0.0, cfg),
                   0.7);
  // x=(1.5-1.25)/0.5=0.5 and smootherstep(0.5)=0.5.
  EXPECT_NEAR(smoothRisk(particles, Eigen::Vector3d::Zero(), 0.0, cfg), 1.1,
              1e-12);

  const ParticleSet highMass{particle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7),
                             particle(0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8)};
  EXPECT_DOUBLE_EQ(hardRisk(highMass, Eigen::Vector3d::Zero(), 0.0, cfg),
                   1.5);
}

TEST(DspParticleRisk, StaticAndDynamicParticlesUsePredictedPositions) {
  const Config cfg = config();
  const ParticleSet particles{
      particle(0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4),
      particle(2.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.6)};

  EXPECT_DOUBLE_EQ(hardRisk(particles, Eigen::Vector3d::Zero(), 0.0, cfg),
                   0.4);
  // At t=1 the dynamic particle reaches the physical boundary and is counted.
  EXPECT_DOUBLE_EQ(hardRisk(particles, Eigen::Vector3d::Zero(), 1.0, cfg),
                   1.0);
  EXPECT_DOUBLE_EQ(smoothRisk(particles, Eigen::Vector3d::Zero(), 0.0, cfg),
                   0.4);
}

TEST(DspParticleRisk, SmoothRiskDominatesHardRiskForFixedParticlesAndCV) {
  const Config cfg = config();
  const ParticleSet particles{
      particle(-2.0, 0.0, 0.0, 0.4, 0.0, 0.0, 0.2),
      particle(0.2, 0.1, 0.0, -0.1, 0.0, 0.0, 0.5),
      particle(2.0, 0.0, 0.0, -0.3, 0.0, 0.0, 0.9),
      particle(0.0, 2.0, 0.0, 0.0, -0.4, 0.0, 0.4)};

  for (double time : {0.0, 0.35, 1.0, 2.0}) {
    for (double x : {-1.0, 0.0, 0.8, 1.7}) {
      const Eigen::Vector3d uav(x, 0.0, 0.0);
      EXPECT_GE(smoothRisk(particles, uav, time, cfg),
                hardRisk(particles, uav, time, cfg) - 1e-12);
    }
  }
}

TEST(DspParticleRisk, BoundaryIsC2InSpaceAndTime) {
  const Config cfg = config();
  const ParticleSet static_particle{
      particle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0)};
  const double h = 1e-5;

  auto spatial = [&static_particle, &cfg](double x) {
    return spatialRisk(x, static_particle, cfg);
  };
  const auto spatialAt = [&spatial](double x) { return spatial(x); };
  const double inner = cfg.collision_radius;
  const double outer = cfg.collision_radius + cfg.smoothing_width;
  const auto d1 = [&spatialAt, h](double x) {
    return (spatialAt(x + h) - spatialAt(x - h)) / (2.0 * h);
  };
  const auto d2 = [&spatialAt, h](double x) {
    return (spatialAt(x + h) - 2.0 * spatialAt(x) + spatialAt(x - h)) /
           (h * h);
  };
  EXPECT_NEAR(d1(inner), 0.0, 1e-5);
  EXPECT_NEAR(d2(inner), 0.0, 2e-2);
  EXPECT_NEAR(d1(outer), 0.0, 1e-5);
  EXPECT_NEAR(d2(outer), 0.0, 2e-2);

  // The moving particle crosses the same two boundaries under CV motion.
  const ParticleSet moving_particle{
      particle(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0)};
  auto temporal = [&moving_particle, &cfg](double time) {
    return smoothRisk(moving_particle, Eigen::Vector3d::Zero(), time, cfg);
  };
  const auto t1 = [&temporal, h](double time) {
    return (temporal(time + h) - temporal(time - h)) / (2.0 * h);
  };
  const auto t2 = [&temporal, h](double time) {
    return (temporal(time + h) - 2.0 * temporal(time) +
            temporal(time - h)) /
           (h * h);
  };
  EXPECT_NEAR(t1(inner), 0.0, 1e-5);
  EXPECT_NEAR(t2(inner), 0.0, 2e-2);
  EXPECT_NEAR(t1(outer), 0.0, 1e-5);
  EXPECT_NEAR(t2(outer), 0.0, 2e-2);
}

TEST(DspParticleRisk, InteriorContributionsHaveZeroSpatialGradientSingleAndMultiple) {
  const Config cfg = config();
  const ParticleSet single{
      particle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2)};
  const double h = 1e-4;
  EXPECT_NEAR(spatialRisk(h, single, cfg) - spatialRisk(-h, single, cfg), 0.0,
              1e-14);
  EXPECT_DOUBLE_EQ(spatialRisk(0.2, single, cfg), 1.2);

  const ParticleSet multiple{
      particle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4),
      particle(0.3, 0.2, 0.0, 0.0, 0.0, 0.0, 0.6),
      particle(-0.2, -0.1, 0.1, 0.0, 0.0, 0.0, 0.8)};
  EXPECT_NEAR(spatialRisk(h, multiple, cfg) - spatialRisk(-h, multiple, cfg),
              0.0, 1e-14);
  EXPECT_DOUBLE_EQ(spatialRisk(0.0, multiple, cfg), 1.8);
}

TEST(DspParticleRisk, EmptySetAndInvalidInputsAreHandledExplicitly) {
  const Config cfg = config();
  const ParticleSet empty;
  EXPECT_DOUBLE_EQ(hardRisk(empty, Eigen::Vector3d::Zero(), 0.0, cfg), 0.0);
  EXPECT_DOUBLE_EQ(smoothRisk(empty, Eigen::Vector3d::Zero(), 0.0, cfg), 0.0);
  EXPECT_TRUE(predictPositions(empty, 0.0).empty());

  const Particle valid = particle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1);
  EXPECT_THROW(predictPosition(valid, -1.0), std::invalid_argument);
  EXPECT_THROW(predictPosition(valid, std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
  EXPECT_THROW(smoothRisk(ParticleSet{particle(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           -0.1)},
                          Eigen::Vector3d::Zero(), 0.0, cfg),
               std::invalid_argument);

  Particle nonfinite = valid;
  nonfinite.velocity.x() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(predictPositions(ParticleSet{nonfinite}, 0.0),
               std::invalid_argument);
  EXPECT_THROW(smoothRisk(ParticleSet{valid},
                          Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
                                          0.0, 0.0),
                          0.0, cfg),
               std::invalid_argument);

  Config invalid = cfg;
  invalid.collision_radius = 0.0;
  EXPECT_THROW(hardRisk(ParticleSet{valid}, Eigen::Vector3d::Zero(), 0.0,
                        invalid),
               std::invalid_argument);
  invalid = cfg;
  invalid.smoothing_width = -0.1;
  EXPECT_THROW(smoothRisk(ParticleSet{valid}, Eigen::Vector3d::Zero(), 0.0,
                          invalid),
               std::invalid_argument);
}

}  // namespace
