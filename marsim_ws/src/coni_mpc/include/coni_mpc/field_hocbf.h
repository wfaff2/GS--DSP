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
  double sigma = 0.12;
  double distance_margin = 0.20;
  double uav_radius = 0.15;
  double voxel_size = 0.15;
  double distance_epsilon = 1.0e-6;
  double occupancy_alpha = 0.0;
  double gamma1 = 1.2;
  double gamma2 = 4.5;
  double prune_radius = 2.0;
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
         std::isfinite(c.prune_radius) && c.prune_radius > 0.0;
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

// Preserve all eligible rows in source order. The message has no stable voxel
// identity, so duplicate centers are distinct LSE terms and are not removed.
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
    if ((nominal_position - point.position).norm() <= config.prune_radius) {
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

// Three-dimensional conservative voxel-surface smooth-minimum HOCBF. This
// fixed-selected-set calculation is smooth; runtime pruning is separate.
inline Constraint computeConstraint(
    const KinematicPoints& points, const Eigen::Vector3d& nominal_position,
    const Eigen::Vector3d& nominal_velocity,
    const Eigen::Vector3d& non_inertial_acceleration,
    const Config& config = Config(),
    const Eigen::Vector3d& lambda = Eigen::Vector3d::Constant(5.0)) {
  Constraint result;
  if (points.empty() || !validConfig(config) ||
      !nominal_position.allFinite() || !nominal_velocity.allFinite() ||
      !non_inertial_acceleration.allFinite() || !lambda.allFinite() ||
      (lambda.array() <= 0.0).any()) {
    return result;
  }

  struct Terms {
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Matrix3d center_hessian = Eigen::Matrix3d::Zero();
    double occupancy = 0.0;
    double kappa = 0.0;
    double distance = 0.0;
    double hdot = 0.0;
    double hddot_drift = 0.0;
  };
  std::vector<Terms> terms;
  terms.reserve(points.size());
  double d_min = std::numeric_limits<double>::infinity();
  const double half_diagonal = std::sqrt(3.0) * config.voxel_size * 0.5;
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

  for (const auto& point : points) {
    if (!std::isfinite(point.occupancy) || point.occupancy < 0.0 ||
        point.occupancy > 1.0 || !point.position.allFinite() ||
        !point.velocity.allFinite() || !point.acceleration.allFinite()) {
      result.max_occupancy = 0.0;
      return result;
    }
    if (!(point.occupancy > 0.0)) continue;
    Terms item;
    item.occupancy = point.occupancy;
    item.kappa = 1.0 + config.occupancy_alpha * point.occupancy;
    const Eigen::Vector3d q = nominal_position - point.position;
    const double s = std::sqrt(q.squaredNorm() +
                               config.distance_epsilon * config.distance_epsilon);
    item.distance = s - config.distance_epsilon - config.uav_radius - half_diagonal;
    item.normal = q / s;
    item.center_hessian = identity / s - (q * q.transpose()) / (s * s * s);
    const Eigen::Vector3d relative_velocity = nominal_velocity - point.velocity;
    const Eigen::Vector3d relative_drift_acceleration =
        non_inertial_acceleration - lambda.cwiseProduct(nominal_velocity) -
        point.acceleration;
    item.hdot = item.normal.dot(relative_velocity);
    item.hddot_drift =
        relative_velocity.dot(item.center_hessian * relative_velocity) +
        item.normal.dot(relative_drift_acceleration);
    d_min = std::min(d_min, item.distance);
    result.max_occupancy = std::max(result.max_occupancy, item.occupancy);
    terms.push_back(item);
  }
  if (terms.empty() || !std::isfinite(d_min)) {
    result.max_occupancy = 0.0;
    return result;
  }

  const double inv_sigma = 1.0 / config.sigma;
  std::vector<double> unnormalized(terms.size(), 0.0);
  double partition = 0.0;
  for (std::size_t i = 0; i < terms.size(); ++i) {
    unnormalized[i] = terms[i].kappa *
        std::exp(-(terms[i].distance - d_min) * inv_sigma);
    partition += unnormalized[i];
  }
  if (!(partition > 0.0) || !std::isfinite(partition)) {
    result.max_occupancy = 0.0;
    return result;
  }

  result.d_min = d_min;
  result.d_softmin = d_min - config.sigma * std::log(partition);
  result.h = result.d_softmin - config.distance_margin;
  result.hdot = 0.0;
  Eigen::Matrix3d mean_nn = Eigen::Matrix3d::Zero();
  double mean_hddot_drift = 0.0;
  std::vector<double> weights(terms.size(), 0.0);
  for (std::size_t i = 0; i < terms.size(); ++i) {
    const double w = unnormalized[i] / partition;
    weights[i] = w;
    result.gradient += w * terms[i].normal;
    result.hessian += w * terms[i].center_hessian;
    mean_nn += w * (terms[i].normal * terms[i].normal.transpose());
    result.hdot += w * terms[i].hdot;
    mean_hddot_drift += w * terms[i].hddot_drift;
  }
  result.hessian -= inv_sigma *
      (mean_nn - result.gradient * result.gradient.transpose());
  double hdot_variance = 0.0;
  for (std::size_t i = 0; i < terms.size(); ++i) {
    const double delta = terms[i].hdot - result.hdot;
    hdot_variance += weights[i] * delta * delta;
  }
  result.lf2 = mean_hddot_drift - inv_sigma * hdot_variance;
  result.A = lambda.cwiseProduct(result.gradient);
  result.b = -(result.lf2 + (config.gamma1 + config.gamma2) * result.hdot +
               config.gamma1 * config.gamma2 * result.h);
  result.active = result.A.allFinite() && result.gradient.allFinite() &&
                  result.hessian.allFinite() && std::isfinite(result.b) &&
                  std::isfinite(result.h) && std::isfinite(result.hdot) &&
                  std::isfinite(result.lf2) && result.max_occupancy > 0.0;
  if (!result.active) {
    result.A.setZero();
    result.b = 0.0;
  }
  return result;
}

}  // namespace field_hocbf
}  // namespace coni_mpc
