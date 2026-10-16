#pragma once

#include <Eigen/Core>

#include <map>
#include <vector>

namespace coni_mpc {
namespace dsp_particle_to_grid {

// The particle weight is the native DSP PHD weight.  It is accumulated as a
// mass-like quantity and is never normalized or converted to occupancy.
struct Particle {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double weight = 0.0;
};

using ParticleSet = std::vector<Particle>;

// dimensions are node counts along x/y/z.  Node j has fixed position
// origin + resolution * j, independent of all particle states.  Particle and
// grid vectors are required to use the same Cartesian coordinate frame.
struct GridConfig {
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  Eigen::Vector3i dimensions = Eigen::Vector3i::Constant(16);
  double resolution = 1.0;
};

struct GridIndex {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator<(const GridIndex& other) const;
  bool operator==(const GridIndex& other) const;
};

struct NodeValues {
  double mass = 0.0;
  double mass_rate = 0.0;
  double mass_acceleration = 0.0;
};

using SparseGrid = std::map<GridIndex, NodeValues>;

struct Projection {
  SparseGrid nodes;
  double input_mass = 0.0;
  double projected_mass = 0.0;
};

class ParticleToGrid {
 public:
  explicit ParticleToGrid(const GridConfig& config);

  const GridConfig& config() const { return config_; }

  // Project a fixed particle set after CV propagation for prediction_time.
  // A positive-weight particle whose nonzero cubic-B-spline support reaches
  // outside the finite grid throws std::out_of_range; no mass is dropped.
  Projection project(const ParticleSet& particles,
                     double prediction_time) const;

  Eigen::Vector3d nodePosition(const GridIndex& index) const;

 private:
  GridConfig config_;
};

}  // namespace dsp_particle_to_grid
}  // namespace coni_mpc
