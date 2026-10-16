#pragma once

#include "coni_mpc/dsp_particle_risk.h"
#include "coni_mpc/dsp_particle_to_grid.h"

#include <Eigen/Core>

namespace coni_mpc {
namespace conservative_grid_risk {

// Uses the same collision envelope as dsp_particle_risk.  The grid spacing is
// taken from GridConfig::resolution; no particle or controller parameters are
// changed here.
using Config = dsp_particle_risk::Config;

struct Derivatives {
  double value = 0.0;
  Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
  Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
  double time_derivative = 0.0;
  double time_second_derivative = 0.0;
  Eigen::Vector3d gradient_time_derivative = Eigen::Vector3d::Zero();
};

// Evaluate R_grid_upper and its explicit q/t derivatives from one fixed-grid
// P2G projection.  The projection must have been produced with the supplied
// fixed grid and the same prediction time as the requested derivatives.
Derivatives evaluate(const dsp_particle_to_grid::Projection& projection,
                     const dsp_particle_to_grid::GridConfig& grid_config,
                     const Eigen::Vector3d& query_position,
                     const Config& config = Config());

double risk(const dsp_particle_to_grid::Projection& projection,
            const dsp_particle_to_grid::GridConfig& grid_config,
            const Eigen::Vector3d& query_position,
            const Config& config = Config());

Eigen::Vector3d gradient(const dsp_particle_to_grid::Projection& projection,
                         const dsp_particle_to_grid::GridConfig& grid_config,
                         const Eigen::Vector3d& query_position,
                         const Config& config = Config());

Eigen::Matrix3d hessian(const dsp_particle_to_grid::Projection& projection,
                        const dsp_particle_to_grid::GridConfig& grid_config,
                        const Eigen::Vector3d& query_position,
                        const Config& config = Config());

double timeDerivative(const dsp_particle_to_grid::Projection& projection,
                      const dsp_particle_to_grid::GridConfig& grid_config,
                      const Eigen::Vector3d& query_position,
                      const Config& config = Config());

double timeSecondDerivative(
    const dsp_particle_to_grid::Projection& projection,
    const dsp_particle_to_grid::GridConfig& grid_config,
    const Eigen::Vector3d& query_position, const Config& config = Config());

Eigen::Vector3d gradientTimeDerivative(
    const dsp_particle_to_grid::Projection& projection,
    const dsp_particle_to_grid::GridConfig& grid_config,
    const Eigen::Vector3d& query_position, const Config& config = Config());

}  // namespace conservative_grid_risk
}  // namespace coni_mpc
