#include "coni_mpc/dsp_particle_to_grid.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using coni_mpc::dsp_particle_to_grid::GridConfig;
using coni_mpc::dsp_particle_to_grid::GridIndex;
using coni_mpc::dsp_particle_to_grid::NodeValues;
using coni_mpc::dsp_particle_to_grid::Particle;
using coni_mpc::dsp_particle_to_grid::ParticleSet;
using coni_mpc::dsp_particle_to_grid::ParticleToGrid;
using coni_mpc::dsp_particle_to_grid::Projection;

Particle particle(double x, double y, double z, double vx, double vy,
                  double vz, double weight) {
  Particle result;
  result.position = Eigen::Vector3d(x, y, z);
  result.velocity = Eigen::Vector3d(vx, vy, vz);
  result.weight = weight;
  return result;
}

GridConfig gridConfig() {
  GridConfig result;
  result.origin = Eigen::Vector3d(-8.0, -8.0, -8.0);
  result.dimensions = Eigen::Vector3i(32, 32, 32);
  result.resolution = 1.0;
  return result;
}

Eigen::Vector3d world(const GridConfig& config, double x, double y, double z) {
  return config.origin + Eigen::Vector3d(x, y, z) * config.resolution;
}

double massAt(const Projection& projection, const GridIndex& index) {
  const auto it = projection.nodes.find(index);
  return it == projection.nodes.end() ? 0.0 : it->second.mass;
}

double rateAt(const Projection& projection, const GridIndex& index) {
  const auto it = projection.nodes.find(index);
  return it == projection.nodes.end() ? 0.0 : it->second.mass_rate;
}

double accelerationAt(const Projection& projection, const GridIndex& index) {
  const auto it = projection.nodes.find(index);
  return it == projection.nodes.end() ? 0.0
                                      : it->second.mass_acceleration;
}

std::set<GridIndex> unionIndices(const Projection& first,
                                 const Projection& second) {
  std::set<GridIndex> result;
  for (const auto& entry : first.nodes) {
    result.insert(entry.first);
  }
  for (const auto& entry : second.nodes) {
    result.insert(entry.first);
  }
  return result;
}

TEST(DspParticleToGrid, SingleParticleHasAtMost64NodesAndPartitionsUnity) {
  const GridConfig config = gridConfig();
  const ParticleToGrid mapper(config);
  const double weight = 2.3;
  const Projection projection = mapper.project(
      ParticleSet{particle(world(config, 10.25, 11.5, 12.75).x(),
                           world(config, 10.25, 11.5, 12.75).y(),
                           world(config, 10.25, 11.5, 12.75).z(), 0.0, 0.0, 0.0,
                           weight)},
      0.0);

  ASSERT_LE(projection.nodes.size(), 64u);
  ASSERT_EQ(projection.nodes.size(), 64u);
  EXPECT_DOUBLE_EQ(projection.input_mass, weight);
  EXPECT_NEAR(projection.projected_mass, weight, 1e-14);
  for (const auto& entry : projection.nodes) {
    EXPECT_GT(entry.second.mass, 0.0);
  }
  const GridIndex fixed_index{3, 4, 5};
  EXPECT_NEAR((mapper.nodePosition(fixed_index) -
               (config.origin + config.resolution *
                                   Eigen::Vector3d(3.0, 4.0, 5.0)))
                  .norm(),
              0.0, 1e-15);

  // A unit-weight particle directly verifies sum_j N_j = 1.
  const Projection unit = mapper.project(
      ParticleSet{particle(world(config, 10.25, 11.5, 12.75).x(),
                           world(config, 10.25, 11.5, 12.75).y(),
                           world(config, 10.25, 11.5, 12.75).z(), 0.0, 0.0, 0.0,
                           1.0)},
      0.0);
  EXPECT_NEAR(unit.projected_mass, 1.0, 1e-14);
}

TEST(DspParticleToGrid, NodesAreFixedAndMultipleParticlesConserveMass) {
  const GridConfig config = gridConfig();
  const ParticleToGrid mapper(config);
  const ParticleSet particles{
      particle(world(config, 7.2, 8.1, 9.3).x(), world(config, 7.2, 8.1, 9.3).y(),
               world(config, 7.2, 8.1, 9.3).z(), 0.0, 0.0, 0.0, 0.3),
      particle(world(config, 14.4, 9.8, 13.6).x(),
               world(config, 14.4, 9.8, 13.6).y(),
               world(config, 14.4, 9.8, 13.6).z(), 0.2, -0.1, 0.15, 0.8),
      particle(world(config, 18.7, 16.2, 11.1).x(),
               world(config, 18.7, 16.2, 11.1).y(),
               world(config, 18.7, 16.2, 11.1).z(), -0.15, 0.1, -0.05, 1.1)};

  const Projection at_zero = mapper.project(particles, 0.0);
  const Projection at_future = mapper.project(particles, 0.7);
  EXPECT_NEAR(at_zero.input_mass, 2.2, 1e-14);
  EXPECT_NEAR(at_zero.projected_mass, 2.2, 1e-13);
  EXPECT_NEAR(at_future.projected_mass, 2.2, 1e-13);
  double rate_sum = 0.0;
  double acceleration_sum = 0.0;
  for (const auto& entry : at_zero.nodes) {
    rate_sum += entry.second.mass_rate;
    acceleration_sum += entry.second.mass_acceleration;
  }
  EXPECT_NEAR(rate_sum, 0.0, 1e-12);
  EXPECT_NEAR(acceleration_sum, 0.0, 1e-12);
  EXPECT_GT(at_zero.nodes.size(), 0u);
  EXPECT_GT(at_future.nodes.size(), 0u);

  const GridIndex index{12, 13, 14};
  const Eigen::Vector3d expected_position =
      config.origin + config.resolution * Eigen::Vector3d(12.0, 13.0, 14.0);
  EXPECT_NEAR((mapper.nodePosition(index) - expected_position).norm(), 0.0,
              1e-15);
  EXPECT_THROW(mapper.nodePosition(GridIndex{-1, 0, 0}), std::out_of_range);
}

TEST(DspParticleToGrid, StaticAndDynamicParticlesChangeDistributionWithoutChangingMass) {
  const GridConfig config = gridConfig();
  const ParticleToGrid mapper(config);
  const Particle static_particle =
      particle(world(config, 8.25, 9.2, 10.4).x(),
               world(config, 8.25, 9.2, 10.4).y(),
               world(config, 8.25, 9.2, 10.4).z(), 0.0, 0.0, 0.0, 0.6);
  const Particle dynamic_particle =
      particle(world(config, 15.2, 12.3, 14.1).x(),
               world(config, 15.2, 12.3, 14.1).y(),
               world(config, 15.2, 12.3, 14.1).z(), 0.4, -0.2, 0.1, 0.9);

  const Projection static_zero = mapper.project(ParticleSet{static_particle}, 0.0);
  const Projection static_one = mapper.project(ParticleSet{static_particle}, 1.0);
  const Projection mixed_zero =
      mapper.project(ParticleSet{static_particle, dynamic_particle}, 0.0);
  const Projection mixed_one =
      mapper.project(ParticleSet{static_particle, dynamic_particle}, 1.0);

  EXPECT_NEAR(static_zero.projected_mass, 0.6, 1e-14);
  EXPECT_NEAR(static_one.projected_mass, 0.6, 1e-14);
  ASSERT_EQ(static_zero.nodes.size(), static_one.nodes.size());
  for (const auto& entry : static_zero.nodes) {
    const auto other = static_one.nodes.find(entry.first);
    ASSERT_NE(other, static_one.nodes.end());
    EXPECT_DOUBLE_EQ(entry.second.mass, other->second.mass);
    EXPECT_DOUBLE_EQ(entry.second.mass_rate, other->second.mass_rate);
    EXPECT_DOUBLE_EQ(entry.second.mass_acceleration,
                     other->second.mass_acceleration);
  }
  EXPECT_NEAR(mixed_zero.projected_mass, 1.5, 1e-13);
  EXPECT_NEAR(mixed_one.projected_mass, 1.5, 1e-13);

  double distribution_change = 0.0;
  for (const GridIndex& index : unionIndices(mixed_zero, mixed_one)) {
    distribution_change +=
        std::abs(massAt(mixed_zero, index) - massAt(mixed_one, index));
  }
  EXPECT_GT(distribution_change, 1e-4);
}

TEST(DspParticleToGrid, CubicSupportCrossingIsC2AcrossIntegerBoundary) {
  const GridConfig config = gridConfig();
  const ParticleToGrid mapper(config);
  const Particle crossing =
      particle(world(config, 10.8, 10.3, 10.4).x(),
               world(config, 10.8, 10.3, 10.4).y(),
               world(config, 10.8, 10.3, 10.4).z(), 1.0, 0.0, 0.0, 1.7);
  const double crossing_time = 0.2;
  const double epsilon = 1e-5;
  const Projection center = mapper.project(ParticleSet{crossing}, crossing_time);
  const Projection left = mapper.project(
      ParticleSet{crossing}, crossing_time - epsilon);
  const Projection right = mapper.project(
      ParticleSet{crossing}, crossing_time + epsilon);

  std::set<GridIndex> indices = unionIndices(left, right);
  for (const auto& entry : center.nodes) {
    indices.insert(entry.first);
  }
  for (const GridIndex& index : indices) {
    const double center_mass = massAt(center, index);
    const double center_rate = rateAt(center, index);
    const double center_acceleration = accelerationAt(center, index);
    const double left_mass_limit =
        center_mass - epsilon * center_rate +
        0.5 * epsilon * epsilon * center_acceleration;
    const double right_mass_limit =
        center_mass + epsilon * center_rate +
        0.5 * epsilon * epsilon * center_acceleration;
    const double left_rate_limit = center_rate - epsilon * center_acceleration;
    const double right_rate_limit = center_rate + epsilon * center_acceleration;
    EXPECT_NEAR(massAt(left, index), left_mass_limit, 1e-9);
    EXPECT_NEAR(massAt(right, index), right_mass_limit, 1e-9);
    EXPECT_NEAR(rateAt(left, index), left_rate_limit, 1e-7);
    EXPECT_NEAR(rateAt(right, index), right_rate_limit, 1e-7);
    EXPECT_NEAR(accelerationAt(left, index), center_acceleration, 1e-4);
    EXPECT_NEAR(accelerationAt(right, index), center_acceleration, 1e-4);
  }
}

TEST(DspParticleToGrid, AnalyticTimeDerivativesMatchFiniteDifferences) {
  const GridConfig config = gridConfig();
  const ParticleToGrid mapper(config);
  const Particle moving =
      particle(world(config, 10.25, 11.35, 12.4).x(),
               world(config, 10.25, 11.35, 12.4).y(),
               world(config, 10.25, 11.35, 12.4).z(), 0.23, -0.17, 0.11, 1.2);
  const double time = 0.4;
  const double dt = 1e-4;
  const Projection center = mapper.project(ParticleSet{moving}, time);
  const Projection plus = mapper.project(ParticleSet{moving}, time + dt);
  const Projection minus = mapper.project(ParticleSet{moving}, time - dt);

  for (const auto& entry : center.nodes) {
    const GridIndex& index = entry.first;
    const double finite_rate =
        (massAt(plus, index) - massAt(minus, index)) / (2.0 * dt);
    const double finite_acceleration =
        (massAt(plus, index) - 2.0 * massAt(center, index) +
         massAt(minus, index)) /
        (dt * dt);
    EXPECT_NEAR(entry.second.mass_rate, finite_rate, 1e-7);
    EXPECT_NEAR(entry.second.mass_acceleration, finite_acceleration, 1e-5);
  }
}

TEST(DspParticleToGrid, GridNodeAndSupportBoundaryAreExplicit) {
  const GridConfig config = gridConfig();
  const ParticleToGrid mapper(config);

  const Projection exact_node = mapper.project(
      ParticleSet{particle(world(config, 5.0, 5.0, 5.0).x(),
                           world(config, 5.0, 5.0, 5.0).y(),
                           world(config, 5.0, 5.0, 5.0).z(), 0.0, 0.0, 0.0, 1.0)},
      0.0);
  EXPECT_NEAR(exact_node.projected_mass, 1.0, 1e-14);
  EXPECT_EQ(exact_node.nodes.size(), 27u);

  // At the finite map boundary the cubic support needs a node outside the map.
  EXPECT_THROW(mapper.project(
                   ParticleSet{particle(config.origin.x(), config.origin.y(),
                                         config.origin.z(), 0.0, 0.0, 0.0, 1.0)},
                   0.0),
               std::out_of_range);

  // The exact lower edge of the valid support domain is local coordinate 1.
  const Projection support_edge = mapper.project(
      ParticleSet{particle(world(config, 1.0, 1.0, 1.0).x(),
                           world(config, 1.0, 1.0, 1.0).y(),
                           world(config, 1.0, 1.0, 1.0).z(), 0.0, 0.0, 0.0, 1.0)},
      0.0);
  EXPECT_NEAR(support_edge.projected_mass, 1.0, 1e-14);
  EXPECT_THROW(mapper.project(
                   ParticleSet{particle(world(config, 1.0 - 1e-6, 1.0, 1.0).x(),
                                         world(config, 1.0 - 1e-6, 1.0, 1.0).y(),
                                         world(config, 1.0 - 1e-6, 1.0, 1.0).z(),
                                         0.0, 0.0, 0.0, 1.0)},
                   0.0),
               std::out_of_range);
}

TEST(DspParticleToGrid, EmptyZeroWeightAndInvalidInputs) {
  const GridConfig config = gridConfig();
  const ParticleToGrid mapper(config);
  const Projection empty = mapper.project(ParticleSet(), 0.0);
  EXPECT_TRUE(empty.nodes.empty());
  EXPECT_DOUBLE_EQ(empty.input_mass, 0.0);
  EXPECT_DOUBLE_EQ(empty.projected_mass, 0.0);

  const Projection zero = mapper.project(
      ParticleSet{particle(config.origin.x() - 100.0, config.origin.y(),
                           config.origin.z(), 0.0, 0.0, 0.0, 0.0)},
      0.0);
  EXPECT_TRUE(zero.nodes.empty());
  EXPECT_DOUBLE_EQ(zero.projected_mass, 0.0);

  EXPECT_THROW(mapper.project(
                   ParticleSet{particle(world(config, 5.2, 5.2, 5.2).x(),
                                         world(config, 5.2, 5.2, 5.2).y(),
                                         world(config, 5.2, 5.2, 5.2).z(), 0.0,
                                         0.0, 0.0, -0.1)},
                   0.0),
               std::invalid_argument);
  Particle nonfinite = particle(world(config, 5.2, 5.2, 5.2).x(),
                                world(config, 5.2, 5.2, 5.2).y(),
                                world(config, 5.2, 5.2, 5.2).z(), 0.0, 0.0, 0.0,
                                1.0);
  nonfinite.velocity.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(mapper.project(ParticleSet{nonfinite}, 0.0),
               std::invalid_argument);
  EXPECT_THROW(mapper.project(ParticleSet(), -1.0), std::invalid_argument);

  GridConfig invalid = config;
  invalid.resolution = 0.0;
  EXPECT_THROW({ ParticleToGrid invalid_mapper(invalid); },
               std::invalid_argument);
  invalid = config;
  invalid.dimensions.x() = 3;
  EXPECT_THROW({ ParticleToGrid invalid_mapper(invalid); },
               std::invalid_argument);
  invalid = config;
  invalid.origin.x() = std::numeric_limits<double>::infinity();
  EXPECT_THROW({ ParticleToGrid invalid_mapper(invalid); },
               std::invalid_argument);
}

TEST(DspParticleToGrid, BenchmarkScalesWithParticleCountAndReportsNodeCount) {
  GridConfig config;
  config.origin = Eigen::Vector3d(-32.0, -32.0, -32.0);
  config.dimensions = Eigen::Vector3i(128, 128, 128);
  config.resolution = 0.5;
  const ParticleToGrid mapper(config);

  for (const std::size_t count : {1u, 32u, 256u, 1024u}) {
    ParticleSet particles;
    particles.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      const double x = 20.2 + static_cast<double>(i % 17) * 0.37;
      const double y = 21.1 + static_cast<double>((i / 17) % 17) * 0.31;
      const double z = 22.3 + static_cast<double>((i / 289) % 17) * 0.29;
      particles.push_back(particle(world(config, x, y, z).x(),
                                   world(config, x, y, z).y(),
                                   world(config, x, y, z).z(), 0.03, -0.02,
                                   0.01, 0.1 + 0.001 * (i % 7)));
    }

    const auto start = std::chrono::steady_clock::now();
    Projection projection;
    for (int repeat = 0; repeat < 3; ++repeat) {
      projection = mapper.project(particles, 0.35);
    }
    const auto stop = std::chrono::steady_clock::now();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(stop - start).count() / 3.0;
    ASSERT_NE(projection.nodes.size(), 0u);
    EXPECT_NEAR(projection.projected_mass, projection.input_mass, 1e-10);
    ASSERT_GT(elapsed_ms, 0.0);
    std::cout << "[P2G benchmark] particles=" << count
              << " nodes=" << projection.nodes.size()
              << " mean_ms=" << elapsed_ms << "\n";
    RecordProperty("particles_" + std::to_string(count),
                   static_cast<int>(projection.nodes.size()));
    RecordProperty("milliseconds_" + std::to_string(count), elapsed_ms);
  }
}

}  // namespace
