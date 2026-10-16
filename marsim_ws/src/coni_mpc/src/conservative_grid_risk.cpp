#include "coni_mpc/conservative_grid_risk.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace coni_mpc {
namespace conservative_grid_risk {
namespace {

struct RadialValues {
  double value = 0.0;
  double first = 0.0;
  double second = 0.0;
};

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

void validateGrid(const dsp_particle_to_grid::GridConfig& grid) {
  if (!grid.origin.allFinite() || !std::isfinite(grid.resolution) ||
      grid.resolution <= 0.0) {
    throw std::invalid_argument("grid origin and resolution must be finite");
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (grid.dimensions[axis] < 1) {
      throw std::invalid_argument("grid dimensions must be positive");
    }
    const double last =
        grid.origin[axis] +
        grid.resolution * static_cast<double>(grid.dimensions[axis] - 1);
    if (!std::isfinite(last)) {
      throw std::invalid_argument("grid node positions must be finite");
    }
  }
}

void validateProjection(const dsp_particle_to_grid::Projection& projection,
                        const dsp_particle_to_grid::GridConfig& grid) {
  if (!std::isfinite(projection.input_mass) ||
      !std::isfinite(projection.projected_mass)) {
    throw std::invalid_argument("projection masses must be finite");
  }
  for (const auto& entry : projection.nodes) {
    const auto& index = entry.first;
    if (index.x < 0 || index.x >= grid.dimensions.x() || index.y < 0 ||
        index.y >= grid.dimensions.y() || index.z < 0 ||
        index.z >= grid.dimensions.z()) {
      throw std::out_of_range("projection node is outside fixed grid");
    }
    const auto& values = entry.second;
    if (!std::isfinite(values.mass) || values.mass < 0.0 ||
        !std::isfinite(values.mass_rate) ||
        !std::isfinite(values.mass_acceleration)) {
      throw std::invalid_argument("projection node values must be finite");
    }
  }
}

RadialValues radialProfile(double distance, double rho, const Config& config) {
  const double transition_start = rho + config.collision_radius;
  const double transition_end = transition_start + config.smoothing_width;
  if (distance <= transition_start) {
    // This includes the max(0, d-rho) kink and the physical collision
    // plateau.  Both spatial derivatives are exactly zero here.
    return {1.0, 0.0, 0.0};
  }
  if (distance >= transition_end) {
    return {0.0, 0.0, 0.0};
  }

  const double x = (transition_end - distance) / config.smoothing_width;
  const double x2 = x * x;
  const double value = x2 * x * (x * (x * 6.0 - 15.0) + 10.0);
  const double dg_dx = 30.0 * x2 * (x - 1.0) * (x - 1.0);
  const double d2g_dx2 = 120.0 * x * x * x - 180.0 * x2 + 60.0 * x;
  return {value, -dg_dx / config.smoothing_width,
          d2g_dx2 / (config.smoothing_width * config.smoothing_width)};
}

struct NodeTerms {
  double value = 0.0;
  Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
  Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
};

NodeTerms nodeTerms(const Eigen::Vector3d& query,
                    const Eigen::Vector3d& node,
                    double rho,
                    const Config& config) {
  const Eigen::Vector3d displacement = query - node;
  const double distance = displacement.norm();
  const RadialValues radial = radialProfile(distance, rho, config);
  NodeTerms result;
  result.value = radial.value;
  if (radial.first == 0.0 && radial.second == 0.0) {
    return result;
  }

  // The nonzero derivative region starts strictly outside the node, because
  // collision_radius is positive.  This keeps the radial Hessian well-defined.
  const Eigen::Vector3d unit = displacement / distance;
  result.gradient = radial.first * unit;
  result.hessian = radial.second * (unit * unit.transpose()) +
                   (radial.first / distance) *
                       (Eigen::Matrix3d::Identity() - unit * unit.transpose());
  return result;
}

Eigen::Vector3d nodePosition(const dsp_particle_to_grid::GridConfig& grid,
                             const dsp_particle_to_grid::GridIndex& index) {
  return grid.origin +
         grid.resolution *
             Eigen::Vector3d(static_cast<double>(index.x),
                             static_cast<double>(index.y),
                             static_cast<double>(index.z));
}

}  // namespace

Derivatives evaluate(const dsp_particle_to_grid::Projection& projection,
                     const dsp_particle_to_grid::GridConfig& grid_config,
                     const Eigen::Vector3d& query_position,
                     const Config& config) {
  validateConfig(config);
  validateGrid(grid_config);
  validateProjection(projection, grid_config);
  if (!query_position.allFinite()) {
    throw std::invalid_argument("query position must be finite");
  }

  const double rho = 2.0 * std::sqrt(3.0) * grid_config.resolution;
  Derivatives result;
  for (const auto& entry : projection.nodes) {
    const Eigen::Vector3d node = nodePosition(grid_config, entry.first);
    const NodeTerms terms = nodeTerms(query_position, node, rho, config);
    const auto& values = entry.second;
    result.value += values.mass * terms.value;
    result.gradient += values.mass * terms.gradient;
    result.hessian += values.mass * terms.hessian;
    result.time_derivative += values.mass_rate * terms.value;
    result.time_second_derivative += values.mass_acceleration * terms.value;
    result.gradient_time_derivative += values.mass_rate * terms.gradient;
  }
  return result;
}

double risk(const dsp_particle_to_grid::Projection& projection,
            const dsp_particle_to_grid::GridConfig& grid_config,
            const Eigen::Vector3d& query_position, const Config& config) {
  return evaluate(projection, grid_config, query_position, config).value;
}

Eigen::Vector3d gradient(const dsp_particle_to_grid::Projection& projection,
                         const dsp_particle_to_grid::GridConfig& grid_config,
                         const Eigen::Vector3d& query_position,
                         const Config& config) {
  return evaluate(projection, grid_config, query_position, config).gradient;
}

Eigen::Matrix3d hessian(const dsp_particle_to_grid::Projection& projection,
                        const dsp_particle_to_grid::GridConfig& grid_config,
                        const Eigen::Vector3d& query_position,
                        const Config& config) {
  return evaluate(projection, grid_config, query_position, config).hessian;
}

double timeDerivative(const dsp_particle_to_grid::Projection& projection,
                      const dsp_particle_to_grid::GridConfig& grid_config,
                      const Eigen::Vector3d& query_position,
                      const Config& config) {
  return evaluate(projection, grid_config, query_position, config)
      .time_derivative;
}

double timeSecondDerivative(
    const dsp_particle_to_grid::Projection& projection,
    const dsp_particle_to_grid::GridConfig& grid_config,
    const Eigen::Vector3d& query_position, const Config& config) {
  return evaluate(projection, grid_config, query_position, config)
      .time_second_derivative;
}

Eigen::Vector3d gradientTimeDerivative(
    const dsp_particle_to_grid::Projection& projection,
    const dsp_particle_to_grid::GridConfig& grid_config,
    const Eigen::Vector3d& query_position, const Config& config) {
  return evaluate(projection, grid_config, query_position, config)
      .gradient_time_derivative;
}

}  // namespace conservative_grid_risk
}  // namespace coni_mpc
