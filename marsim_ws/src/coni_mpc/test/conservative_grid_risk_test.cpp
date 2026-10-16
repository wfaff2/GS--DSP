#include "coni_mpc/conservative_grid_risk.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

namespace {

using GridConfig = coni_mpc::dsp_particle_to_grid::GridConfig;
using GridIndex = coni_mpc::dsp_particle_to_grid::GridIndex;
using GridParticle = coni_mpc::dsp_particle_to_grid::Particle;
using GridParticleSet = coni_mpc::dsp_particle_to_grid::ParticleSet;
using Projection = coni_mpc::dsp_particle_to_grid::Projection;
using Mapper = coni_mpc::dsp_particle_to_grid::ParticleToGrid;
using RiskParticle = coni_mpc::dsp_particle_risk::Particle;
using RiskParticleSet = coni_mpc::dsp_particle_risk::ParticleSet;
using RiskConfig = coni_mpc::dsp_particle_risk::Config;

RiskParticle particle(double x, double y, double z, double vx, double vy,
                      double vz, double weight) {
  RiskParticle result;
  result.position = Eigen::Vector3d(x, y, z);
  result.velocity = Eigen::Vector3d(vx, vy, vz);
  result.weight = weight;
  return result;
}

GridParticle gridParticle(const RiskParticle& source) {
  GridParticle result;
  result.position = source.position;
  result.velocity = source.velocity;
  result.weight = source.weight;
  return result;
}

GridParticleSet gridParticles(const RiskParticleSet& particles) {
  GridParticleSet result;
  result.reserve(particles.size());
  for (const auto& particle : particles) {
    result.push_back(gridParticle(particle));
  }
  return result;
}

GridConfig grid(double resolution) {
  GridConfig result;
  result.origin = Eigen::Vector3d(-32.0, -32.0, -32.0);
  result.dimensions = Eigen::Vector3i::Constant(
      static_cast<int>(64.0 / resolution) + 1);
  result.resolution = resolution;
  return result;
}

RiskConfig riskConfig() {
  RiskConfig result;
  result.collision_radius = 1.0;
  result.smoothing_width = 0.5;
  return result;
}

double smooth(const RiskParticleSet& particles, const Eigen::Vector3d& q,
              double time, const RiskConfig& config) {
  return coni_mpc::dsp_particle_risk::smoothRisk(particles, q, time, config);
}

double hard(const RiskParticleSet& particles, const Eigen::Vector3d& q,
            double time, const RiskConfig& config) {
  return coni_mpc::dsp_particle_risk::hardRisk(particles, q, time, config);
}

double upper(const Mapper& mapper, const RiskParticleSet& particles,
             const Eigen::Vector3d& q, double time, const RiskConfig& config) {
  const Projection projection = mapper.project(gridParticles(particles), time);
  return coni_mpc::conservative_grid_risk::risk(projection, mapper.config(), q,
                                                 config);
}

void expectUpperBound(const RiskParticleSet& particles, const GridConfig& grid,
                      const Eigen::Vector3d& q, double time,
                      const RiskConfig& config) {
  const Mapper mapper(grid);
  const double hard_risk = hard(particles, q, time, config);
  const double smooth_risk = smooth(particles, q, time, config);
  const double grid_risk = upper(mapper, particles, q, time, config);
  EXPECT_LE(hard_risk, smooth_risk + 2e-11);
  EXPECT_LE(smooth_risk, grid_risk + 2e-11)
      << "hard=" << hard_risk << " smooth=" << smooth_risk
      << " grid=" << grid_risk;
}

TEST(ConservativeGridRisk, SameParticlesSatisfyBoundForRepresentativeScenarios) {
  const RiskConfig config = riskConfig();
  const GridConfig grid_config = grid(1.0);
  const RiskParticleSet static_single{
      particle(-0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.8)};
  const RiskParticleSet dense_static{
      particle(-0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4),
      particle(-0.2, 0.4, 0.1, 0.0, 0.0, 0.0, 0.7),
      particle(0.5, -0.3, 0.2, 0.0, 0.0, 0.0, 1.1),
      particle(0.9, 0.2, -0.2, 0.0, 0.0, 0.0, 0.3)};
  const RiskParticleSet dynamic{
      particle(-2.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.9),
      particle(1.5, -0.5, 0.0, -0.3, 0.4, 0.0, 0.6)};
  const RiskParticleSet mixed = dense_static;
  const RiskParticleSet mixed_with_dynamic{
      mixed[0], mixed[1], mixed[2], mixed[3], dynamic[0], dynamic[1]};

  for (const auto& scenario : {static_single, dense_static, dynamic,
                               mixed_with_dynamic}) {
    for (double time : {0.0, 0.35, 1.0}) {
      const std::vector<Eigen::Vector3d> queries{
          Eigen::Vector3d::Zero(), Eigen::Vector3d(2.1, -0.4, 0.7),
          Eigen::Vector3d(-1.4, 1.1, -0.8)};
      for (const Eigen::Vector3d& q : queries) {
        expectUpperBound(scenario, grid_config, q, time, config);
      }
    }
  }
}

TEST(ConservativeGridRisk, RandomParticlesAndQueriesSatisfyBound) {
  std::mt19937 generator(20261009u);
  std::uniform_real_distribution<double> position(-8.0, 8.0);
  std::uniform_real_distribution<double> velocity(-0.8, 0.8);
  std::uniform_real_distribution<double> weight(0.05, 1.5);
  const RiskConfig config = riskConfig();
  RiskParticleSet particles;
  for (int i = 0; i < 60; ++i) {
    particles.push_back(particle(position(generator), position(generator),
                                 position(generator), velocity(generator),
                                 velocity(generator), velocity(generator),
                                 weight(generator)));
  }
  const Mapper mapper(grid(0.5));
  for (int i = 0; i < 100; ++i) {
    const Eigen::Vector3d q(position(generator), position(generator),
                            position(generator));
    const double time = 0.1 * static_cast<double>(i % 10);
    const double smooth_risk = smooth(particles, q, time, config);
    const double grid_risk = upper(mapper, particles, q, time, config);
    EXPECT_LE(smooth_risk, grid_risk + 1e-9);
  }
}

TEST(ConservativeGridRisk, MaxAndNormKinksAreHiddenByCollisionPlateau) {
  const GridConfig grid_config = grid(1.0);
  const RiskConfig config = riskConfig();
  Projection projection;
  projection.input_mass = 1.0;
  projection.projected_mass = 1.0;
  projection.nodes[GridIndex{32, 32, 32}].mass = 1.0;
  const double rho = 2.0 * std::sqrt(3.0) * grid_config.resolution;
  const Eigen::Vector3d node(-32.0, -32.0, -32.0);
  const double h = 1e-5;
  auto value = [&](double distance) {
    return coni_mpc::conservative_grid_risk::evaluate(
               projection, grid_config,
               node + Eigen::Vector3d(distance, 0.0, 0.0), config)
        .value;
  };
  for (double distance : {0.0, rho, rho + config.collision_radius,
                          rho + config.collision_radius + 1e-4,
                          rho + config.collision_radius +
                              config.smoothing_width}) {
    const double first = (value(distance + h) - value(distance - h)) / (2.0 * h);
    const double second = (value(distance + h) - 2.0 * value(distance) +
                           value(distance - h)) /
                          (h * h);
    EXPECT_NEAR(first, 0.0, 2e-5);
    EXPECT_NEAR(second, 0.0, 5e-2);
  }
  // The exact node and d=rho points are in the constant unit plateau, so the
  // analytic derivatives are exactly zero rather than merely finite.
  EXPECT_EQ(coni_mpc::conservative_grid_risk::evaluate(
                projection, grid_config, node, config)
                .gradient.norm(),
            0.0);
  EXPECT_EQ(coni_mpc::conservative_grid_risk::evaluate(
                projection, grid_config,
                node + Eigen::Vector3d(rho, 0.0, 0.0), config)
                .hessian.norm(),
            0.0);
}

TEST(ConservativeGridRisk, AnalyticQAndTimeDerivativesMatchFiniteDifferences) {
  const GridConfig grid_config = grid(0.5);
  const Mapper mapper(grid_config);
  const RiskConfig config = riskConfig();
  const RiskParticleSet particles{
      particle(-1.2, -0.8, 0.4, 0.27, -0.13, 0.19, 1.3),
      particle(2.4, 1.1, -0.7, -0.11, 0.22, 0.09, 0.8)};
  const double time = 0.6;
  const double dt = 1e-4;
  const double dq = 1e-5;
  const Eigen::Vector3d query(2.0, 1.5, 0.9);
  const Projection center = mapper.project(gridParticles(particles), time);
  const Projection plus_t = mapper.project(gridParticles(particles), time + dt);
  const Projection minus_t = mapper.project(gridParticles(particles), time - dt);
  const auto& api = coni_mpc::conservative_grid_risk::evaluate(
      center, grid_config, query, config);
  const auto& plus_api = coni_mpc::conservative_grid_risk::evaluate(
      plus_t, grid_config, query, config);
  const auto& minus_api = coni_mpc::conservative_grid_risk::evaluate(
      minus_t, grid_config, query, config);
  EXPECT_NEAR(api.time_derivative,
              (plus_api.value - minus_api.value) / (2.0 * dt), 2e-6);
  EXPECT_NEAR(api.time_second_derivative,
              (plus_api.value - 2.0 * api.value + minus_api.value) /
                  (dt * dt),
              2e-4);
  EXPECT_NEAR((api.gradient_time_derivative -
               (plus_api.gradient - minus_api.gradient) / (2.0 * dt))
                  .norm(),
              0.0, 2e-6);

  Eigen::Matrix3d finite_hessian;
  finite_hessian.setZero();
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    offset[axis] = dq;
    const auto plus = coni_mpc::conservative_grid_risk::evaluate(
        center, grid_config, query + offset, config);
    const auto minus = coni_mpc::conservative_grid_risk::evaluate(
        center, grid_config, query - offset, config);
    finite_hessian.col(axis) = (plus.gradient - minus.gradient) / (2.0 * dq);
  }
  EXPECT_NEAR((api.hessian - finite_hessian).norm(), 0.0, 2e-4);
  const auto q_plus = coni_mpc::conservative_grid_risk::evaluate(
      center, grid_config, query + Eigen::Vector3d(dq, 0.0, 0.0), config);
  const auto q_minus = coni_mpc::conservative_grid_risk::evaluate(
      center, grid_config, query - Eigen::Vector3d(dq, 0.0, 0.0), config);
  EXPECT_NEAR((api.gradient -
               (q_plus.value - q_minus.value) / (2.0 * dq) *
                   Eigen::Vector3d::UnitX())
                  .x(),
              0.0, 2e-4);
}

TEST(ConservativeGridRisk, TimeC2AcrossParticleGridBoundaryAndSupportOnset) {
  const GridConfig grid_config = grid(1.0);
  const Mapper mapper(grid_config);
  const RiskConfig config = riskConfig();
  // The particle crosses the x=1 grid boundary at t=0.2.  The query is
  // outside the plateau so the test observes the grid-risk value itself.
  const RiskParticleSet particles{
      particle(0.8, 0.2, -0.1, 1.0, 0.0, 0.0, 1.4)};
  const Eigen::Vector3d query(6.0, 0.2, -0.1);
  const double crossing_time = 0.2;
  const double h = 1e-4;
  auto evaluateAt = [&](double time) {
    const Projection projection = mapper.project(gridParticles(particles), time);
    return coni_mpc::conservative_grid_risk::evaluate(
        projection, grid_config, query, config);
  };
  const auto center = evaluateAt(crossing_time);
  const auto left = evaluateAt(crossing_time - h);
  const auto right = evaluateAt(crossing_time + h);
  const double left_rate = (center.value - left.value) / h;
  const double right_rate = (right.value - center.value) / h;
  const double left_acceleration =
      (center.value - 2.0 * left.value + evaluateAt(crossing_time - 2.0 * h).value) /
      (h * h);
  const double right_acceleration =
      (evaluateAt(crossing_time + 2.0 * h).value - 2.0 * right.value +
       center.value) /
      (h * h);
  // One-sided first differences differ by O(h) even for a C2 function.
  EXPECT_NEAR(left_rate, right_rate, 2e-4);
  EXPECT_NEAR(left_acceleration, right_acceleration, 2e-3);
  EXPECT_NEAR(center.time_derivative, (right.value - left.value) / (2.0 * h),
              2e-5);
  EXPECT_NEAR(center.time_second_derivative,
              (right.value - 2.0 * center.value + left.value) / (h * h),
              2e-3);
}

TEST(ConservativeGridRisk, GridResolutionAndTimingReport) {
  const RiskConfig config = riskConfig();
  const RiskParticleSet particles{
      particle(-2.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.7),
      particle(0.3, 0.8, -0.2, 0.0, -0.1, 0.05, 1.1),
      particle(2.2, -1.0, 0.5, -0.2, 0.1, -0.1, 0.5)};
  const Eigen::Vector3d query(0.4, -0.2, 0.3);
  const double threshold = 0.75;
  const double hard_risk = hard(particles, query, 0.7, config);
  const double smooth_risk = smooth(particles, query, 0.7, config);
  std::cout << "\n[conservative_grid_risk] resolution report\n";
  std::cout << "  reference hard=" << hard_risk
            << " smooth=" << smooth_risk << '\n';
  for (double resolution : {1.0, 0.5, 0.25}) {
    const Mapper mapper(grid(resolution));
    const double grid_risk = upper(mapper, particles, query, 0.7, config);
    int smooth_safe = 0;
    int grid_safe = 0;
    for (int i = -80; i <= 80; ++i) {
      const Eigen::Vector3d q(query.x() + 0.05 * i, query.y(), query.z());
      if (smooth(particles, q, 0.7, config) <= threshold) {
        ++smooth_safe;
      }
      if (upper(mapper, particles, q, 0.7, config) <= threshold) {
        ++grid_safe;
      }
    }
    std::cout << "  Delta=" << resolution << " smooth=" << smooth_risk
              << " grid=" << grid_risk
              << " excess=" << grid_risk - smooth_risk
              << " safe_count_difference=" << smooth_safe - grid_safe
              << " extra_excluded_length="
              << 0.05 * static_cast<double>(smooth_safe - grid_safe) << '\n';
  }

  const Mapper mapper(grid(0.5));
  const auto grid_particles = gridParticles(particles);
  constexpr int iterations = 200;
  const auto start = std::chrono::steady_clock::now();
  volatile double sink = 0.0;
  for (int i = 0; i < iterations; ++i) {
    const double time = 0.01 * static_cast<double>(i % 20);
    const Projection projection = mapper.project(grid_particles, time);
    sink += coni_mpc::conservative_grid_risk::risk(
        projection, mapper.config(), query, config);
  }
  const auto end = std::chrono::steady_clock::now();
  const double total_us =
      std::chrono::duration<double, std::micro>(end - start).count();
  const double per_step_us = total_us / iterations;
  std::cout << "[conservative_grid_risk] P2G+risk per prediction step: "
            << per_step_us << " us; 20-step estimate: " << 20.0 * per_step_us
            << " us (sink=" << sink << ")\n";
  EXPECT_TRUE(std::isfinite(per_step_us));
}

}  // namespace
