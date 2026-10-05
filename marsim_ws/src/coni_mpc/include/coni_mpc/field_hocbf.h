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
  double d_safe = 0.48;
  double gamma1 = 1.2;
  double gamma2 = 4.5;
  double prune_radius = 1.80;
  double max_vertical_delta = 0.60;
  double min_occupancy = 0.15;
  std::size_t seed_count = 10;
  std::size_t max_count = 35;
  double separation = 0.15;
};

struct Point {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero();
  double occupancy = 0.0;
  double prediction_time = 0.0;
  std::uint32_t stage_index = 0;
  // Index in the published PointCloud2 snapshot.  It is used only for
  // diagnostics, so the controller does not depend on its value.
  std::uint64_t source_id = 0;
};
using Points = std::vector<Point, Eigen::aligned_allocator<Point>>;

struct KinematicPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  double occupancy = 0.0;
  std::uint64_t source_id = 0;
};
using KinematicPoints =
    std::vector<KinematicPoint, Eigen::aligned_allocator<KinematicPoint>>;

struct PruneStats {
  std::size_t candidate_count = 0;
  std::size_t eligible_count = 0;
  std::size_t selected_count = 0;
  std::vector<std::uint64_t> selected_source_ids;
};

struct Constraint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d A = Eigen::Vector3d::Zero();
  double b = 0.0;
  double max_occupancy = 0.0;
  double h = std::numeric_limits<double>::quiet_NaN();
  double hdot = std::numeric_limits<double>::quiet_NaN();
  double lf2 = std::numeric_limits<double>::quiet_NaN();
  double d_min = std::numeric_limits<double>::quiet_NaN();
  double d_softmin = std::numeric_limits<double>::quiet_NaN();
  bool active = false;
};

inline KinematicPoints prune(const KinematicPoints& candidates,
                             const Eigen::Vector3d& nominal_position,
                             const Config& config = Config(),
                             PruneStats* stats = nullptr) {
  if (stats != nullptr) {
    stats->candidate_count = candidates.size();
    stats->eligible_count = 0;
    stats->selected_count = 0;
    stats->selected_source_ids.clear();
  }
  struct Ranked { std::size_t index; double distance; };
  std::vector<Ranked> ranked;
  ranked.reserve(candidates.size());
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const auto& point = candidates[i];
    const Eigen::Vector3d diff = nominal_position - point.position;
    const double vertical_distance = std::abs(diff.z());
    const double planar_distance = diff.head<2>().norm();
    if (point.position.allFinite() && point.velocity.allFinite() &&
        point.acceleration.allFinite() && std::isfinite(point.occupancy) &&
        point.occupancy >= config.min_occupancy &&
        vertical_distance <= config.max_vertical_delta &&
        planar_distance <= config.prune_radius) {
      ranked.push_back({i, planar_distance});
    }
  }
  std::stable_sort(ranked.begin(), ranked.end(),
                   [](const Ranked& a, const Ranked& b) {
                     return a.distance < b.distance;
                   });
  if (stats != nullptr) stats->eligible_count = ranked.size();
  KinematicPoints selected;
  selected.reserve(std::min(config.max_count, ranked.size()));
  for (const Ranked& item : ranked) {
    const auto& point = candidates[item.index];
    bool accept = selected.size() < config.seed_count;
    if (!accept) {
      accept = std::all_of(selected.begin(), selected.end(),
                           [&](const KinematicPoint& other) {
                             return (point.position.head<2>() -
                                     other.position.head<2>()).norm() >=
                                    config.separation;
                           });
    }
    if (accept) selected.push_back(point);
    if (selected.size() >= config.max_count) break;
  }
  if (stats != nullptr) {
    stats->selected_count = selected.size();
    stats->selected_source_ids.reserve(selected.size());
    for (const auto& point : selected) {
      stats->selected_source_ids.push_back(point.source_id);
    }
  }
  return selected;
}

// Relative-degree-two Occupancy-Normalized Log-Mean-Exp Smooth-Minimum HOCBF (LSE-HOCBF).
// 1. Individual voxel planar Euclidean distance:
//      d_m(p) = sqrt(||Delta p_m||^2 + eps^2),  n_m = Delta p_m / d_m
// 2. Occupancy-Normalized Smooth-Minimum barrier:
//      h(p) = -sigma * ln( (sum_m P_m * exp(-d_m(p) / sigma)) / (sum_m P_m) ) - d_safe
// 3. Exact Softmax weights and analytical 1st/2nd Lie derivatives:
//      w_m = P_m * exp(-d_m / sigma) / sum_j P_j * exp(-d_j / sigma)
//      grad_p h = sum_m w_m * n_m
//      hdot     = sum_m w_m * hdot_m
//      Lf^2 h   = sum_m w_m * hddot_m,drift - (1 / sigma) * sum_m w_m * (hdot_m - hdot)^2
inline Constraint computeConstraint(
    const KinematicPoints& points, const Eigen::Vector3d& nominal_position,
    const Eigen::Vector3d& nominal_velocity,
    const Eigen::Vector3d& non_inertial_acceleration,
    const Config& config = Config(),
    const Eigen::Vector3d& lambda = Eigen::Vector3d::Constant(5.0)) {
  Constraint result;
  const double sigma = config.sigma;
  const double d_safe = config.d_safe;
  const double gamma1 = config.gamma1;
  const double gamma2 = config.gamma2;
  if (points.empty() || !(sigma > 0.0) || !nominal_position.allFinite() ||
      !nominal_velocity.allFinite() || !non_inertial_acceleration.allFinite() ||
      !(d_safe >= 0.0) || !(gamma1 >= 0.0) || !(gamma2 >= 0.0)) {
    return result;
  }

  struct VoxelDistanceTerms {
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double P_m = 0.0;
    double d_m = 0.0;
    double hdot_m = 0.0;
    double hddot_drift_m = 0.0;
  };

  std::vector<VoxelDistanceTerms> terms;
  terms.reserve(points.size());
  double d_min = std::numeric_limits<double>::infinity();
  double sum_occupancy = 0.0;

  for (const auto& point : points) {
    if (!(point.occupancy > 0.0) || !std::isfinite(point.occupancy)) continue;
    VoxelDistanceTerms item;
    item.P_m = point.occupancy;

    Eigen::Vector3d delta_p = nominal_position - point.position;
    delta_p.z() = 0.0;
    item.d_m = std::sqrt(delta_p.squaredNorm() + 1.0e-12);
    item.normal = delta_p / item.d_m;

    Eigen::Vector3d v_r = nominal_velocity - point.velocity;
    v_r.z() = 0.0;

    Eigen::Vector3d a_drift =
        non_inertial_acceleration - lambda.cwiseProduct(nominal_velocity) -
        point.acceleration;
    a_drift.z() = 0.0;

    item.hdot_m = item.normal.dot(v_r);
    const double tangential_v2 =
        std::max(0.0, v_r.squaredNorm() - item.hdot_m * item.hdot_m);
    item.hddot_drift_m =
        tangential_v2 / item.d_m + item.normal.dot(a_drift);

    d_min = std::min(d_min, item.d_m);
    sum_occupancy += item.P_m;
    result.max_occupancy = std::max(result.max_occupancy, item.P_m);
    terms.push_back(item);
  }

  if (terms.empty() || !(sum_occupancy > 0.0) || !std::isfinite(d_min)) {
    result.max_occupancy = 0.0;
    return result;
  }

  const double inv_sigma = 1.0 / sigma;
  double shifted_partition_sum = 0.0;
  std::vector<double> shifted_weights(terms.size(), 0.0);
  for (std::size_t m = 0; m < terms.size(); ++m) {
    shifted_weights[m] =
        terms[m].P_m * std::exp(-(terms[m].d_m - d_min) * inv_sigma);
    shifted_partition_sum += shifted_weights[m];
  }

  if (!(shifted_partition_sum > 0.0) || !std::isfinite(shifted_partition_sum)) {
    result.max_occupancy = 0.0;
    return result;
  }

  // Occupancy-normalized smooth minimum distance:
  // d_softmin = d_min - sigma * ln(shifted_partition_sum / sum_occupancy) >= d_min
  const double d_softmin =
      d_min - sigma * std::log(shifted_partition_sum / sum_occupancy);
  result.d_min = d_min;
  result.d_softmin = d_softmin;
  result.h = d_softmin - d_safe;

  Eigen::Vector3d grad_h = Eigen::Vector3d::Zero();
  double hdot = 0.0;
  double hddot_drift_mean = 0.0;
  std::vector<double> weights(terms.size(), 0.0);

  for (std::size_t m = 0; m < terms.size(); ++m) {
    const double w_m = shifted_weights[m] / shifted_partition_sum;
    weights[m] = w_m;
    grad_h += w_m * terms[m].normal;
    hdot += w_m * terms[m].hdot_m;
    hddot_drift_mean += w_m * terms[m].hddot_drift_m;
  }

  double hdot_variance = 0.0;
  for (std::size_t m = 0; m < terms.size(); ++m) {
    const double dhdot = terms[m].hdot_m - hdot;
    hdot_variance += weights[m] * dhdot * dhdot;
  }

  const double lf2 = hddot_drift_mean - inv_sigma * hdot_variance;
  result.hdot = hdot;
  result.lf2 = lf2;
  result.A = lambda.cwiseProduct(grad_h);
  result.b = -(lf2 + (gamma1 + gamma2) * hdot + gamma1 * gamma2 * result.h);
  result.active = result.A.allFinite() && std::isfinite(result.b) &&
                  std::isfinite(result.h) && result.max_occupancy > 0.0;
  if (!result.active) {
    result.A.setZero();
    result.b = 0.0;
  }
  return result;
}

}  // namespace field_hocbf
}  // namespace coni_mpc
