#include "coni_mpc/dsp_particle_risk.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace coni_mpc {
namespace dsp_particle_risk {
namespace {

void validateConfig(const Config& config) {
  if (!std::isfinite(config.collision_radius) ||
      !std::isfinite(config.smoothing_width) ||
      config.collision_radius <= 0.0 || config.smoothing_width <= 0.0) {
    throw std::invalid_argument(
        "collision_radius and smoothing_width must be finite and positive");
  }
  if (!std::isfinite(config.collision_radius + config.smoothing_width)) {
    throw std::invalid_argument("outer smoothing radius must be finite");
  }
}

void validateTime(double prediction_time) {
  if (!std::isfinite(prediction_time) || prediction_time < 0.0) {
    throw std::invalid_argument(
        "prediction_time must be finite and non-negative");
  }
}

void validateVector(const Eigen::Vector3d& value, const char* name) {
  if (!value.allFinite()) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

void validateParticle(const Particle& particle) {
  validateVector(particle.position, "particle position");
  validateVector(particle.velocity, "particle velocity");
  if (!std::isfinite(particle.weight) || particle.weight < 0.0) {
    throw std::invalid_argument("particle weight must be finite and non-negative");
  }
}

void validateQuery(const ParticleSet& particles,
                   const Eigen::Vector3d& uav_position,
                   double prediction_time,
                   const Config& config) {
  validateConfig(config);
  validateVector(uav_position, "UAV position");
  validateTime(prediction_time);
  for (const Particle& particle : particles) {
    validateParticle(particle);
  }
}

double distanceToUav(const Particle& particle,
                     const Eigen::Vector3d& uav_position,
                     double prediction_time) {
  const Eigen::Vector3d predicted =
      particle.position + particle.velocity * prediction_time;
  const double distance = (predicted - uav_position).norm();
  if (!std::isfinite(distance)) {
    throw std::invalid_argument("predicted particle distance is not finite");
  }
  return distance;
}

double smoothFactor(double distance, const Config& config) {
  if (distance <= config.collision_radius) {
    return 1.0;
  }
  const double outer_radius = config.collision_radius + config.smoothing_width;
  if (distance >= outer_radius) {
    return 0.0;
  }

  // smootherstep(x) = 6x^5 - 15x^4 + 10x^3, with x=1 at the physical
  // boundary and x=0 at the outer boundary.  Its first two derivatives vanish
  // at both ends, so the piecewise radial contribution is C2.
  const double x = (outer_radius - distance) / config.smoothing_width;
  return x * x * x * (x * (x * 6.0 - 15.0) + 10.0);
}

}  // namespace

Eigen::Vector3d predictPosition(const Particle& particle,
                                double prediction_time) {
  validateParticle(particle);
  validateTime(prediction_time);
  const Eigen::Vector3d predicted =
      particle.position + particle.velocity * prediction_time;
  validateVector(predicted, "predicted particle position");
  return predicted;
}

std::vector<Eigen::Vector3d> predictPositions(const ParticleSet& particles,
                                              double prediction_time) {
  validateTime(prediction_time);
  std::vector<Eigen::Vector3d> predicted;
  predicted.reserve(particles.size());
  for (const Particle& particle : particles) {
    predicted.push_back(predictPosition(particle, prediction_time));
  }
  return predicted;
}

double hardRisk(const ParticleSet& particles, const Eigen::Vector3d& uav_position,
                double prediction_time, const Config& config) {
  validateQuery(particles, uav_position, prediction_time, config);
  double risk = 0.0;
  for (const Particle& particle : particles) {
    if (distanceToUav(particle, uav_position, prediction_time) <=
        config.collision_radius) {
      risk += particle.weight;
    }
  }
  return risk;
}

double smoothRisk(const ParticleSet& particles,
                  const Eigen::Vector3d& uav_position,
                  double prediction_time,
                  const Config& config) {
  validateQuery(particles, uav_position, prediction_time, config);
  double risk = 0.0;
  for (const Particle& particle : particles) {
    const double distance =
        distanceToUav(particle, uav_position, prediction_time);
    risk += particle.weight * smoothFactor(distance, config);
  }
  return risk;
}

}  // namespace dsp_particle_risk
}  // namespace coni_mpc
