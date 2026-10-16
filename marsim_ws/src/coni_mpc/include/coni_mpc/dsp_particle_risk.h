#pragma once

#include <Eigen/Core>

#include <vector>

namespace coni_mpc {
namespace dsp_particle_risk {

// A particle carries the native DSP PHD weight.  The weight is retained as a
// mass-like diagnostic quantity; this module does not normalize or interpret
// it as a collision probability.
struct Particle {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double weight = 0.0;
};

using ParticleSet = std::vector<Particle>;

struct Config {
  // Physical collision region: a closed sphere centred at the UAV.
  double collision_radius = 0.5;
  // Width of the outer C2 transition annulus.  It must be strictly positive.
  double smoothing_width = 0.5;
};

// Constant-velocity prediction p(t) = p(0) + v(0) t.
Eigen::Vector3d predictPosition(const Particle& particle,
                                double prediction_time);

std::vector<Eigen::Vector3d> predictPositions(const ParticleSet& particles,
                                              double prediction_time);

// Sum of unmodified PHD weights whose predicted particle centres lie in the
// physical collision sphere.
double hardRisk(const ParticleSet& particles, const Eigen::Vector3d& uav_position,
                double prediction_time, const Config& config = Config());

// Conservative C2 risk: unit contribution in the collision sphere, a
// fifth-order smootherstep transition over the outer annulus, and zero beyond
// it.  The same native PHD weights are summed without normalization.
double smoothRisk(const ParticleSet& particles,
                  const Eigen::Vector3d& uav_position,
                  double prediction_time,
                  const Config& config = Config());

}  // namespace dsp_particle_risk
}  // namespace coni_mpc

