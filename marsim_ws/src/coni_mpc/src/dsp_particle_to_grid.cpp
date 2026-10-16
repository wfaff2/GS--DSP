#include "coni_mpc/dsp_particle_to_grid.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace coni_mpc {
namespace dsp_particle_to_grid {
namespace {

struct AxisBasis {
  double value[4];
  double first[4];
  double second[4];
  int base;
};

void validateVector(const Eigen::Vector3d& value, const char* name) {
  if (!value.allFinite()) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

void validateConfig(const GridConfig& config) {
  validateVector(config.origin, "grid origin");
  if (!std::isfinite(config.resolution) || config.resolution <= 0.0) {
    throw std::invalid_argument("grid resolution must be finite and positive");
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (config.dimensions[axis] < 4) {
      throw std::invalid_argument("grid dimensions must be at least four nodes");
    }
    const double last_node =
        config.origin[axis] +
        config.resolution * static_cast<double>(config.dimensions[axis] - 1);
    if (!std::isfinite(last_node)) {
      throw std::invalid_argument("grid node positions must be finite");
    }
  }
}

void validateParticle(const Particle& particle) {
  validateVector(particle.position, "particle position");
  validateVector(particle.velocity, "particle velocity");
  if (!std::isfinite(particle.weight) || particle.weight < 0.0) {
    throw std::invalid_argument(
        "particle weight must be finite and non-negative");
  }
}

void validateTime(double prediction_time) {
  if (!std::isfinite(prediction_time) || prediction_time < 0.0) {
    throw std::invalid_argument(
        "prediction_time must be finite and non-negative");
  }
}

AxisBasis makeAxisBasis(double local_coordinate, double resolution,
                        int dimension, const char* axis_name) {
  if (!std::isfinite(local_coordinate)) {
    throw std::out_of_range(std::string("particle is outside finite ") +
                            axis_name + " grid coordinates");
  }

  // Keep the conversion to int within a range in which all four candidate
  // offsets can still be represented and checked below.
  const double lower = -4.0;
  const double upper = static_cast<double>(dimension) + 4.0;
  if (local_coordinate < lower || local_coordinate > upper) {
    throw std::out_of_range(std::string("particle is outside finite ") +
                            axis_name + " grid support");
  }

  const double floor_coordinate = std::floor(local_coordinate);
  if (floor_coordinate < static_cast<double>(std::numeric_limits<int>::min()) +
          1.0 ||
      floor_coordinate > static_cast<double>(std::numeric_limits<int>::max()) -
          2.0) {
    throw std::out_of_range(std::string("particle is outside finite ") +
                            axis_name + " grid index range");
  }
  const int base = static_cast<int>(floor_coordinate);
  const double u = local_coordinate - floor_coordinate;

  AxisBasis result;
  result.base = base;

  // Cubic cardinal B-spline basis for nodes base-1,...,base+2.  first and
  // second are derivatives with respect to physical distance, not local u.
  result.value[0] = (1.0 - u) * (1.0 - u) * (1.0 - u) / 6.0;
  result.value[1] = (3.0 * u * u * u - 6.0 * u * u + 4.0) / 6.0;
  result.value[2] = (-3.0 * u * u * u + 3.0 * u * u + 3.0 * u + 1.0) /
                    6.0;
  result.value[3] = u * u * u / 6.0;

  result.first[0] = -0.5 * (1.0 - u) * (1.0 - u) / resolution;
  result.first[1] = (1.5 * u * u - 2.0 * u) / resolution;
  result.first[2] = (-1.5 * u * u + u + 0.5) / resolution;
  result.first[3] = 0.5 * u * u / resolution;

  const double inverse_resolution_squared = 1.0 / (resolution * resolution);
  result.second[0] = (1.0 - u) * inverse_resolution_squared;
  result.second[1] = (3.0 * u - 2.0) * inverse_resolution_squared;
  result.second[2] = (-3.0 * u + 1.0) * inverse_resolution_squared;
  result.second[3] = u * inverse_resolution_squared;
  return result;
}

bool isZeroContribution(double value, double first, double second) {
  return value == 0.0 && first == 0.0 && second == 0.0;
}

bool validIndex(const GridIndex& index, const Eigen::Vector3i& dimensions) {
  return index.x >= 0 && index.x < dimensions.x() && index.y >= 0 &&
         index.y < dimensions.y() && index.z >= 0 && index.z < dimensions.z();
}

double sumMass(const SparseGrid& nodes) {
  double result = 0.0;
  for (const auto& entry : nodes) {
    result += entry.second.mass;
  }
  return result;
}

}  // namespace

bool GridIndex::operator<(const GridIndex& other) const {
  if (x != other.x) {
    return x < other.x;
  }
  if (y != other.y) {
    return y < other.y;
  }
  return z < other.z;
}

bool GridIndex::operator==(const GridIndex& other) const {
  return x == other.x && y == other.y && z == other.z;
}

ParticleToGrid::ParticleToGrid(const GridConfig& config) : config_(config) {
  validateConfig(config_);
}

Eigen::Vector3d ParticleToGrid::nodePosition(const GridIndex& index) const {
  if (!validIndex(index, config_.dimensions)) {
    throw std::out_of_range("grid node index is outside the fixed grid");
  }
  return config_.origin +
         config_.resolution *
             Eigen::Vector3d(static_cast<double>(index.x),
                             static_cast<double>(index.y),
                             static_cast<double>(index.z));
}

Projection ParticleToGrid::project(const ParticleSet& particles,
                                    double prediction_time) const {
  validateTime(prediction_time);
  Projection result;

  for (const Particle& particle : particles) {
    validateParticle(particle);
    result.input_mass += particle.weight;
    if (particle.weight == 0.0) {
      continue;
    }

    const Eigen::Vector3d predicted =
        particle.position + particle.velocity * prediction_time;
    validateVector(predicted, "predicted particle position");
    const Eigen::Vector3d local =
        (predicted - config_.origin) / config_.resolution;
    const AxisBasis x = makeAxisBasis(local.x(), config_.resolution,
                                       config_.dimensions.x(), "x");
    const AxisBasis y = makeAxisBasis(local.y(), config_.resolution,
                                       config_.dimensions.y(), "y");
    const AxisBasis z = makeAxisBasis(local.z(), config_.resolution,
                                       config_.dimensions.z(), "z");

    for (int ix = 0; ix < 4; ++ix) {
      for (int iy = 0; iy < 4; ++iy) {
        for (int iz = 0; iz < 4; ++iz) {
          const double value = x.value[ix] * y.value[iy] * z.value[iz];
          const double first_x = x.first[ix] * y.value[iy] * z.value[iz];
          const double first_y = x.value[ix] * y.first[iy] * z.value[iz];
          const double first_z = x.value[ix] * y.value[iy] * z.first[iz];
          const double mass_rate =
              first_x * particle.velocity.x() +
              first_y * particle.velocity.y() +
              first_z * particle.velocity.z();

          const double h_xx = x.second[ix] * y.value[iy] * z.value[iz];
          const double h_yy = x.value[ix] * y.second[iy] * z.value[iz];
          const double h_zz = x.value[ix] * y.value[iy] * z.second[iz];
          const double h_xy = x.first[ix] * y.first[iy] * z.value[iz];
          const double h_xz = x.first[ix] * y.value[iy] * z.first[iz];
          const double h_yz = x.value[ix] * y.first[iy] * z.first[iz];
          const double vx = particle.velocity.x();
          const double vy = particle.velocity.y();
          const double vz = particle.velocity.z();
          const double mass_acceleration =
              vx * vx * h_xx + vy * vy * h_yy + vz * vz * h_zz +
              2.0 * vx * vy * h_xy + 2.0 * vx * vz * h_xz +
              2.0 * vy * vz * h_yz;

          if (isZeroContribution(value, mass_rate, mass_acceleration)) {
            continue;
          }

          const GridIndex index{x.base + ix - 1, y.base + iy - 1,
                                z.base + iz - 1};
          if (!validIndex(index, config_.dimensions)) {
            throw std::out_of_range(
                "nonzero cubic B-spline support reaches outside fixed grid");
          }

          NodeValues& node = result.nodes[index];
          node.mass += particle.weight * value;
          node.mass_rate += particle.weight * mass_rate;
          node.mass_acceleration += particle.weight * mass_acceleration;
        }
      }
    }
  }

  result.projected_mass = sumMass(result.nodes);
  if (!std::isfinite(result.input_mass)) {
    throw std::runtime_error("input mass is not finite");
  }
  if (!std::isfinite(result.projected_mass)) {
    throw std::runtime_error("projected mass is not finite");
  }
  const double mass_tolerance =
      1e-10 * std::max(1.0, std::abs(result.input_mass));
  if (std::abs(result.projected_mass - result.input_mass) >
      mass_tolerance) {
    throw std::runtime_error(
        "projected mass differs from input mass beyond roundoff tolerance");
  }
  return result;
}

}  // namespace dsp_particle_to_grid
}  // namespace coni_mpc
