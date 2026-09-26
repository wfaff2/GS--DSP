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

// Field-HOCBF configuration used by the DSP future-occupancy interface.
// Keep these values in one place so the profile generator and its tests do
// not silently drift from the controller design.
constexpr double kSigma = 0.45;
constexpr double kPruneSigmaMultiplier = 3.0;
constexpr double kPruneRadius = kPruneSigmaMultiplier * kSigma;
constexpr double kVmax = 0.3;
constexpr double kDSafe = 1;
constexpr double kGamma1 = 1.5;
constexpr double kGamma2 = 2.0;

struct Point {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero();
  double occupancy = 0.0;
  double prediction_time = 0.0;
  std::uint32_t stage_index = 0;
};
using Points = std::vector<Point, Eigen::aligned_allocator<Point>>;

struct KinematicPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  double occupancy = 0.0;
};
using KinematicPoints =
    std::vector<KinematicPoint, Eigen::aligned_allocator<KinematicPoint>>;

struct Constraint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d A = Eigen::Vector3d::Zero();
  double b = 0.0;
  double max_occupancy = 0.0;
  double h = 0.0;
  bool active = false;
};

inline KinematicPoints prune(const KinematicPoints& candidates,
                             const Eigen::Vector3d& nominal_position,
                             double max_distance = kPruneRadius,
                             double min_occupancy = 0.15,
                             std::size_t seed_count = 10,
                             std::size_t max_count = 35,
                             double separation = 0.15) {
  struct Ranked { std::size_t index; double distance; };
  std::vector<Ranked> ranked;
  ranked.reserve(candidates.size());
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const auto& point = candidates[i];
    const double distance = (point.position - nominal_position).norm();
    if (point.position.allFinite() && point.velocity.allFinite() &&
        point.acceleration.allFinite() && std::isfinite(point.occupancy) &&
        point.occupancy >= min_occupancy && distance <= max_distance) {
      ranked.push_back({i, distance});
    }
  }
  std::stable_sort(ranked.begin(), ranked.end(),
                   [](const Ranked& a, const Ranked& b) {
                     return a.distance < b.distance;
                   });
  KinematicPoints selected;
  selected.reserve(std::min(max_count, ranked.size()));
  for (const Ranked& item : ranked) {
    const auto& point = candidates[item.index];
    bool accept = selected.size() < seed_count;
    if (!accept) {
      accept = std::all_of(selected.begin(), selected.end(),
                           [&](const KinematicPoint& other) {
                             return (point.position - other.position).norm() >= separation;
                           });
    }
    if (accept) selected.push_back(point);
    if (selected.size() >= max_count) break;
  }
  return selected;
}

// Frozen-nominal relative-degree-two field HOCBF. The UAV acceleration model
// is a = Lambda*(v_cmd-v) + a_non_inertial. Thus A*v_cmd + slack >= b.
inline Constraint computeConstraint(
    const KinematicPoints& points, const Eigen::Vector3d& nominal_position,
    const Eigen::Vector3d& nominal_velocity,
    const Eigen::Vector3d& non_inertial_acceleration,
    double sigma = kSigma, double field_max = kVmax,
    double gamma1 = kGamma1, double gamma2 = kGamma2,
    const Eigen::Vector3d& lambda = Eigen::Vector3d::Constant(5.0),
    double d_safe = kDSafe) {
  Constraint result;
  if (points.empty() || !(sigma > 0.0) || !nominal_position.allFinite() ||
      !nominal_velocity.allFinite() || !non_inertial_acceleration.allFinite() ||
      !(d_safe >= 0.0) || !(gamma1 >= 0.0) || !(gamma2 >= 0.0)) {
    return result;
  }
  const double inv_sigma2 = 1.0 / (sigma * sigma);
  double field = 0.0;
  double hdot = 0.0;
  double lf2 = 0.0;
  for (const auto& point : points) {
    const Eigen::Vector3d raw_d = nominal_position - point.position;
    // Signed distance to the inflated voxel shell. The regularized norm
    // keeps the closed form finite at coincidence and preserves the radial
    // first and second derivatives elsewhere.
    const double distance = std::sqrt(raw_d.squaredNorm() + 1.0e-12);
    const Eigen::Vector3d normal = raw_d / distance;
    const double signed_distance = distance - d_safe;
    const Eigen::Vector3d d = signed_distance * normal;
    const Eigen::Vector3d relative_velocity =
        nominal_velocity - point.velocity;
    const double phi = point.occupancy *
        std::exp(-0.5 * d.squaredNorm() * inv_sigma2);
    const double radial_rate = normal.dot(relative_velocity);
    field += phi;
    hdot += phi * signed_distance * radial_rate * inv_sigma2;
    const Eigen::Vector3d drift_acceleration =
        -lambda.cwiseProduct(nominal_velocity) +
        non_inertial_acceleration - point.acceleration;
    const double radial_acceleration =
        (relative_velocity.squaredNorm() - radial_rate * radial_rate) /
            distance +
        normal.dot(drift_acceleration);
    lf2 += phi *
        ((radial_rate * radial_rate + signed_distance * radial_acceleration) *
             inv_sigma2 -
         signed_distance * signed_distance * radial_rate * radial_rate *
             inv_sigma2 * inv_sigma2);
    // d = (||Delta p||-d_safe) Delta p/||Delta p||, so this is the requested
    // (1/sigma^2) sum(phi Delta p^T Lambda) coefficient.
    result.A += phi * inv_sigma2 * lambda.cwiseProduct(d);
    result.max_occupancy = std::max(result.max_occupancy, point.occupancy);
  }
  result.h = field_max - field;
  result.b = -(lf2 + (gamma1 + gamma2) * hdot + gamma1 * gamma2 * result.h);
  result.active = result.A.allFinite() && std::isfinite(result.b) &&
                  result.max_occupancy > 0.0;
  if (!result.active) {
    result.A.setZero();
    result.b = 0.0;
  }
  return result;
}

}  // namespace field_hocbf
}  // namespace coni_mpc
