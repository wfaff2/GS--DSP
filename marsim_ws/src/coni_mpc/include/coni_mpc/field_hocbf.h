#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace coni_mpc {
namespace field_hocbf {

struct Config {
  double sigma = 0.05;
  double distance_margin = 0.05;
  double uav_radius = 0.15;
  double voxel_size = 0.15;
  double distance_epsilon = 1.0e-6;
  double occupancy_alpha = 0.0;
  double gamma1 = 5.0;
  double gamma2 = 5.0;
  double prune_radius = 2.0;
  double cutoff_width = 0.5;
  double occupancy_full_confidence = 0.15;
  double clear_field_value = 1.0;
};

inline bool validConfig(const Config& c) {
  return std::isfinite(c.sigma) && c.sigma > 0.0 &&
         std::isfinite(c.distance_margin) && c.distance_margin >= 0.0 &&
         std::isfinite(c.uav_radius) && c.uav_radius >= 0.0 &&
         std::isfinite(c.voxel_size) && c.voxel_size > 0.0 &&
         std::isfinite(c.distance_epsilon) && c.distance_epsilon > 0.0 &&
         std::isfinite(c.occupancy_alpha) && c.occupancy_alpha >= 0.0 &&
         std::isfinite(c.gamma1) && c.gamma1 > 0.0 &&
         std::isfinite(c.gamma2) && c.gamma2 > 0.0 &&
         std::isfinite(c.prune_radius) && c.prune_radius > 0.0 &&
         std::isfinite(c.cutoff_width) && c.cutoff_width > 0.0 &&
         c.prune_radius - c.cutoff_width >=
             c.uav_radius + std::sqrt(3.0) * c.voxel_size * 0.5 + c.distance_margin &&
         std::isfinite(c.occupancy_full_confidence) &&
         c.occupancy_full_confidence > 0.0 && c.occupancy_full_confidence <= 1.0 &&
         std::isfinite(c.clear_field_value) && c.clear_field_value > 0.0;
}

inline double smootherstep(double x) {
  x = std::max(0.0, std::min(1.0, x));
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

// rho is the regularized 3-D center distance used by the field itself.
// Value and first two derivatives vanish at the outer support boundary.
inline void radialCutoff(double rho, const Config& c,
                         double& value, double& first, double& second) {
  first = second = 0.0;
  if (rho >= c.prune_radius) { value = 0.0; return; }
  if (rho <= c.prune_radius - c.cutoff_width) { value = 1.0; return; }
  const double u = (c.prune_radius - rho) / c.cutoff_width;
  value = smootherstep(u);
  first = -30.0 * u * u * (1.0 - u) * (1.0 - u) / c.cutoff_width;
  second = 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) /
      (c.cutoff_width * c.cutoff_width);
}

struct Point {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
  // Publisher mass-weighted particle velocity: diagnostic only. It must not
  // be used to translate the grid center or form its derivatives.
  Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero();
  double occupancy = 0.0;
  double prediction_time = 0.0;
  std::uint32_t stage_index = 0;
  std::uint64_t source_id = 0;
};
using Points = std::vector<Point, Eigen::aligned_allocator<Point>>;

struct KinematicPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d published_velocity_diagnostic = Eigen::Vector3d::Zero();
  double occupancy = 0.0;
  std::uint64_t source_id = 0;
};
using KinematicPoints =
    std::vector<KinematicPoint, Eigen::aligned_allocator<KinematicPoint>>;

struct PruneStats {
  std::size_t candidate_count = 0;
  std::size_t eligible_count = 0;
  std::size_t selected_count = 0;
  std::size_t invalid_count = 0;
  bool input_valid = true;
  std::vector<std::uint64_t> selected_source_ids;
};

struct Constraint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d A = Eigen::Vector3d::Zero();
  Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
  Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
  double b = 0.0;
  double max_occupancy = 0.0;
  double h = std::numeric_limits<double>::quiet_NaN();
  double hdot = std::numeric_limits<double>::quiet_NaN();
  double lf2 = std::numeric_limits<double>::quiet_NaN();
  double d_min = std::numeric_limits<double>::quiet_NaN();
  double d_softmin = std::numeric_limits<double>::quiet_NaN();
  bool active = false;
};

// Broadphase only. The compact C2 kernel and all its derivatives are zero at
// this boundary. Preserve distinct 3-D voxels; never project height layers.
inline KinematicPoints prune(const KinematicPoints& candidates,
                             const Eigen::Vector3d& nominal_position,
                             const Config& config = Config(),
                             PruneStats* stats = nullptr) {
  PruneStats local;
  local.candidate_count = candidates.size();
  local.input_valid = nominal_position.allFinite() && validConfig(config);
  KinematicPoints selected;
  selected.reserve(candidates.size());
  for (const auto& point : candidates) {
    const bool finite = point.position.allFinite() && point.velocity.allFinite() &&
                        point.acceleration.allFinite() &&
                        std::isfinite(point.occupancy);
    if (!finite || point.occupancy < 0.0 || point.occupancy > 1.0) {
      ++local.invalid_count;
      local.input_valid = false;
      continue;
    }
    if (!(point.occupancy > 0.0) || !nominal_position.allFinite() ||
        !validConfig(config)) {
      continue;
    }
    const Eigen::Vector3d diff = nominal_position - point.position;
    const double rho = std::sqrt(diff.squaredNorm() +
        config.distance_epsilon * config.distance_epsilon) - config.distance_epsilon;
    if (rho < config.prune_radius) {
      ++local.eligible_count;
      selected.push_back(point);
      local.selected_source_ids.push_back(point.source_id);
    }
  }
  if (!local.input_valid) {
    selected.clear();
    local.selected_source_ids.clear();
    local.eligible_count = 0;
  }
  local.selected_count = selected.size();
  if (stats != nullptr) *stats = local;
  return selected;
}

// Confidence-aware 3-D field. Trusted P retains the voxel certificate in the
// violation region; low-P support fades smoothly. Message changes remain a
// separate temporal switching operation. The spatial support boundary is C2.
inline Constraint computeConstraint(
    const KinematicPoints& points, const Eigen::Vector3d& nominal_position,
    const Eigen::Vector3d& nominal_velocity,
    const Eigen::Vector3d& non_inertial_acceleration,
    const Config& config = Config(),
    const Eigen::Vector3d& lambda = Eigen::Vector3d::Constant(5.0)) {
  Constraint result;
  if (!validConfig(config) || !nominal_position.allFinite() ||
      !nominal_velocity.allFinite() || !non_inertial_acceleration.allFinite() ||
      !lambda.allFinite() || (lambda.array() <= 0.0).any()) return result;

  struct Terms {
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Matrix3d center_hessian = Eigen::Matrix3d::Zero();
    Eigen::Vector3d relative_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d relative_drift = Eigen::Vector3d::Zero();
    double cutoff = 0.0, cutoff_first = 0.0, cutoff_second = 0.0;
    double log_amplitude = 0.0;
  };
  std::vector<Terms> terms;
  terms.reserve(points.size());
  double d_min = std::numeric_limits<double>::infinity();
  const double half_diagonal = std::sqrt(3.0) * config.voxel_size * 0.5;
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  const double inv_sigma = 1.0 / config.sigma;
  // Numerical scaling only: its derivative cancels from the log partition.
  double scale = -config.clear_field_value * inv_sigma;
  result.h = config.clear_field_value;
  result.hdot = 0.0;
  result.lf2 = 0.0;
  result.d_softmin = result.h + config.distance_margin;

  for (const auto& point : points) {
    if (!std::isfinite(point.occupancy) || point.occupancy < 0.0 ||
        point.occupancy > 1.0 || !point.position.allFinite() ||
        !point.velocity.allFinite() || !point.acceleration.allFinite()) {
      return Constraint();
    }
    if (!(point.occupancy > 0.0)) continue;
    Terms item;
    const Eigen::Vector3d q = nominal_position - point.position;
    const double s = std::sqrt(q.squaredNorm() +
        config.distance_epsilon * config.distance_epsilon);
    const double rho = s - config.distance_epsilon;
    const double distance = rho - config.uav_radius - half_diagonal;
    d_min = std::min(d_min, distance);
    radialCutoff(rho, config, item.cutoff, item.cutoff_first, item.cutoff_second);
    const double confidence = smootherstep(
        point.occupancy / config.occupancy_full_confidence);
    if (!(confidence > 0.0) || !(item.cutoff > 0.0)) continue;
    item.normal = q / s;
    item.center_hessian = identity / s - (q * q.transpose()) / (s * s * s);
    item.relative_velocity = nominal_velocity - point.velocity;
    item.relative_drift = non_inertial_acceleration -
        lambda.cwiseProduct(nominal_velocity) - point.acceleration;
    item.log_amplitude = std::log(confidence) +
        std::log1p(config.occupancy_alpha * point.occupancy) -
        (distance - config.distance_margin) * inv_sigma;
    scale = std::max(scale, item.log_amplitude);
    result.max_occupancy = std::max(result.max_occupancy, point.occupancy);
    terms.push_back(item);
  }
  if (std::isfinite(d_min)) result.d_min = d_min;
  if (terms.empty()) return result;

  double partition = std::exp(-config.clear_field_value * inv_sigma - scale);
  Eigen::Vector3d partition_gradient = Eigen::Vector3d::Zero();
  Eigen::Matrix3d partition_hessian = Eigen::Matrix3d::Zero();
  double partition_dot = 0.0, partition_ddot = 0.0;
  for (const auto& item : terms) {
    const double amplitude = std::exp(item.log_amplitude - scale);
    const double radial_first = item.cutoff_first - item.cutoff * inv_sigma;
    const double radial_second = item.cutoff_second -
        2.0 * item.cutoff_first * inv_sigma + item.cutoff * inv_sigma * inv_sigma;
    const Eigen::Vector3d kernel_gradient = amplitude * radial_first * item.normal;
    const Eigen::Matrix3d kernel_hessian = amplitude *
        (radial_second * item.normal * item.normal.transpose() +
         radial_first * item.center_hessian);
    partition += amplitude * item.cutoff;
    partition_gradient += kernel_gradient;
    partition_hessian += kernel_hessian;
    partition_dot += kernel_gradient.dot(item.relative_velocity);
    partition_ddot += item.relative_velocity.dot(kernel_hessian * item.relative_velocity) +
        kernel_gradient.dot(item.relative_drift);
  }
  if (!(partition > 0.0) || !std::isfinite(partition)) return Constraint();
  const Eigen::Vector3d scaled_gradient = partition_gradient / partition;
  result.h = -config.sigma * (scale + std::log(partition));
  result.d_softmin = result.h + config.distance_margin;
  result.gradient = -config.sigma * scaled_gradient;
  result.hessian = -config.sigma * (partition_hessian / partition -
      scaled_gradient * scaled_gradient.transpose());
  const double scaled_dot = partition_dot / partition;
  result.hdot = -config.sigma * scaled_dot;
  result.lf2 = -config.sigma * (partition_ddot / partition - scaled_dot * scaled_dot);
  result.A = lambda.cwiseProduct(result.gradient);
  result.b = -(result.lf2 + (config.gamma1 + config.gamma2) * result.hdot +
               config.gamma1 * config.gamma2 * result.h);
  result.active = result.A.allFinite() && result.gradient.allFinite() &&
      result.hessian.allFinite() && std::isfinite(result.b) &&
      std::isfinite(result.h) && std::isfinite(result.hdot) &&
      std::isfinite(result.lf2) && result.max_occupancy > 0.0;
  if (!result.active) { result.A.setZero(); result.b = 0.0; }
  return result;
}

}  // namespace field_hocbf
}  // namespace coni_mpc
