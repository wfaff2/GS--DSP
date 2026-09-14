#include "coni_mpc/depth_cbf/depth_cbf_regressor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include <Eigen/SVD>
#include <pcl/kdtree/kdtree_flann.h>

namespace coni_mpc {
namespace depth_cbf {
namespace {

using SteadyClock = std::chrono::steady_clock;

double elapsedMs(const SteadyClock::time_point& begin,
                 const SteadyClock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

Eigen::Matrix<double, 1, 10> designRow(const Eigen::Vector3d& xi) {
  Eigen::Matrix<double, 1, 10> row;
  row << xi.x() * xi.x(), xi.y() * xi.y(), xi.z() * xi.z(),
      2.0 * xi.x() * xi.y(), 2.0 * xi.x() * xi.z(),
      2.0 * xi.y() * xi.z(), xi.x(), xi.y(), xi.z(), 1.0;
  return row;
}

void resetBarrier(const Eigen::Vector3d& center_N, const ros::Time& stamp,
                  LocalBarrier* barrier) {
  *barrier = LocalBarrier();
  barrier->center_N = center_N;
  barrier->stamp = stamp;
}

}  // namespace

const char* regressionFailureReasonName(RegressionFailureReason reason) {
  switch (reason) {
    case RegressionFailureReason::VALID: return "VALID";
    case RegressionFailureReason::EMPTY_CLOUD: return "EMPTY_CLOUD";
    case RegressionFailureReason::INSUFFICIENT_SUPPORT:
      return "INSUFFICIENT_SUPPORT";
    case RegressionFailureReason::RANK_DEFICIENT: return "RANK_DEFICIENT";
    case RegressionFailureReason::CONDITION_NUMBER_FAIL:
      return "CONDITION_NUMBER_FAIL";
    case RegressionFailureReason::RMSE_FAIL: return "RMSE_FAIL";
    case RegressionFailureReason::MAX_ERROR_FAIL: return "MAX_ERROR_FAIL";
    case RegressionFailureReason::INVALID_CONFIGURATION:
      return "INVALID_CONFIGURATION";
    case RegressionFailureReason::INTERNAL_ERROR: return "INTERNAL_ERROR";
  }
  return "INTERNAL_ERROR";
}

double LocalBarrier::evaluate(const Eigen::Vector3d& p_N) const {
  const Eigen::Vector3d xi = p_N - center_N;
  return xi.dot(A * xi) + b.dot(xi) + c;
}

Eigen::Vector3d LocalBarrier::gradient(const Eigen::Vector3d& p_N) const {
  return 2.0 * A * (p_N - center_N) + b;
}

Eigen::Matrix3d LocalBarrier::hessian() const { return 2.0 * A; }

DepthCbfRegressor::DepthCbfRegressor(const RegressionConfig& config)
    : config_(config) {}

bool DepthCbfRegressor::validateConfig(const RegressionConfig& config,
                                       std::string* error) {
  if (config.query_mesh_half_width_steps < 1) {
    if (error) *error = "query_mesh_half_width_steps must be >= 1.";
    return false;
  }
  if (!(config.query_mesh_spacing > 0.0)) {
    if (error) *error = "query_mesh_spacing must be positive.";
    return false;
  }
  if (!(config.d_safe >= 0.0)) {
    if (error) *error = "d_safe must be non-negative.";
    return false;
  }
  if (!(config.nearest_neighbor_max_distance > 0.0)) {
    if (error) *error = "nearest_neighbor_max_distance must be positive.";
    return false;
  }
  if (!(config.relative_rank_tolerance > 0.0) ||
      !(config.max_condition_number > 1.0) || !(config.max_rmse >= 0.0) ||
      !(config.max_abs_error >= 0.0)) {
    if (error) *error = "Regression tolerances are invalid.";
    return false;
  }
  return true;
}

bool DepthCbfRegressor::fit(const PointCloud& local_cloud_N,
                            const Eigen::Vector3d& query_center_N,
                            const ros::Time& stamp, LocalBarrier* barrier,
                            RegressionDiagnostics* diagnostics,
                            std::string* error, RegressionTiming* timing,
                            RegressionFailureReason* failure_reason) const {
  const auto total_begin = SteadyClock::now();
  if (timing) *timing = RegressionTiming();
  if (failure_reason) *failure_reason = RegressionFailureReason::INTERNAL_ERROR;
  const auto finish = [&](RegressionFailureReason reason) {
    if (failure_reason) *failure_reason = reason;
    if (timing) timing->total_ms = elapsedMs(total_begin, SteadyClock::now());
  };
  if (!barrier) {
    if (error) *error = "barrier output is null.";
    finish(RegressionFailureReason::INTERNAL_ERROR);
    return false;
  }
  resetBarrier(query_center_N, stamp, barrier);
  if (diagnostics) *diagnostics = RegressionDiagnostics();

  std::string config_error;
  if (!validateConfig(config_, &config_error)) {
    if (error) *error = config_error;
    finish(RegressionFailureReason::INVALID_CONFIGURATION);
    return false;
  }
  if (local_cloud_N.empty()) {
    if (error) *error = "local cloud is empty.";
    finish(RegressionFailureReason::EMPTY_CLOUD);
    return false;
  }

  const auto kdtree_begin = SteadyClock::now();
  PointCloud::Ptr cloud(new PointCloud(local_cloud_N));
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  const auto kdtree_end = SteadyClock::now();
  if (timing) timing->kdtree_build_ms = elapsedMs(kdtree_begin, kdtree_end);

  const int L = config_.query_mesh_half_width_steps;
  const int side = 2 * L + 1;
  const int query_count = side * side * side;
  Eigen::MatrixXd design(query_count, 10);
  Eigen::VectorXd values(query_count);
  std::vector<Eigen::Vector3d> query_points;
  std::vector<Eigen::Vector3d> nearest_points;
  query_points.reserve(query_count);
  nearest_points.reserve(query_count);

  int row = 0;
  const double max_distance_squared =
      config_.nearest_neighbor_max_distance *
      config_.nearest_neighbor_max_distance;
  double query_generation_ms = 0.0;
  double nearest_neighbor_query_ms = 0.0;
  for (int ix = -L; ix <= L; ++ix) {
    for (int iy = -L; iy <= L; ++iy) {
      for (int iz = -L; iz <= L; ++iz) {
        const auto query_begin = SteadyClock::now();
        const Eigen::Vector3d xi =
            config_.query_mesh_spacing * Eigen::Vector3d(ix, iy, iz);
        const Eigen::Vector3d query_N = query_center_N + xi;
        pcl::PointXYZ query_N_pcl;
        query_N_pcl.x = static_cast<float>(query_N.x());
        query_N_pcl.y = static_cast<float>(query_N.y());
        query_N_pcl.z = static_cast<float>(query_N.z());
        query_generation_ms += elapsedMs(query_begin, SteadyClock::now());
        std::vector<int> indices(1);
        std::vector<float> squared_distances(1);
        const auto nearest_begin = SteadyClock::now();
        const int nearest_count = kdtree.nearestKSearch(
            query_N_pcl, 1, indices, squared_distances);
        nearest_neighbor_query_ms +=
            elapsedMs(nearest_begin, SteadyClock::now());
        if (nearest_count != 1 ||
            squared_distances[0] > max_distance_squared) {
          std::ostringstream stream;
          stream << "query mesh point has no surface point within "
                 << config_.nearest_neighbor_max_distance << " m (xi="
                 << xi.transpose();
          if (!squared_distances.empty()) {
            stream << ", nearest_distance="
                   << std::sqrt(static_cast<double>(squared_distances[0]));
          }
          stream << ").";
          if (error) *error = stream.str();
          if (timing) {
            timing->query_generation_ms = query_generation_ms;
            timing->nearest_neighbor_query_ms = nearest_neighbor_query_ms;
          }
          finish(RegressionFailureReason::INSUFFICIENT_SUPPORT);
          return false;
        }
        const pcl::PointXYZ& nearest = cloud->points[indices[0]];
        const Eigen::Vector3d nearest_N(nearest.x, nearest.y, nearest.z);
        design.row(row) = designRow(xi);
        values(row) = static_cast<double>(squared_distances[0]) -
                      config_.d_safe * config_.d_safe;
        query_points.push_back(query_N);
        nearest_points.push_back(nearest_N);
        ++row;
      }
    }
  }

  if (timing) {
    timing->query_generation_ms = query_generation_ms;
    timing->nearest_neighbor_query_ms = nearest_neighbor_query_ms;
  }
  const auto regression_begin = SteadyClock::now();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      design, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular_values = svd.singularValues();
  if (singular_values.size() != 10 || singular_values(0) <= 0.0) {
    if (error) *error = "SVD produced invalid singular values.";
    if (timing) timing->regression_ms =
        elapsedMs(regression_begin, SteadyClock::now());
    finish(RegressionFailureReason::RANK_DEFICIENT);
    return false;
  }
  const double threshold =
      config_.relative_rank_tolerance * singular_values(0);
  int rank = 0;
  for (int i = 0; i < singular_values.size(); ++i) {
    if (singular_values(i) > threshold) ++rank;
  }
  barrier->rank = rank;
  barrier->max_singular_value = singular_values(0);
  barrier->min_singular_value = singular_values(singular_values.size() - 1);
  barrier->condition_number =
      barrier->min_singular_value > 0.0
          ? barrier->max_singular_value / barrier->min_singular_value
          : std::numeric_limits<double>::infinity();
  if (rank != 10) {
    if (error) {
      std::ostringstream stream;
      stream << "rank-deficient quadratic fit: rank=" << rank
             << ", condition_number=" << barrier->condition_number;
      *error = stream.str();
    }
    if (timing) timing->regression_ms =
        elapsedMs(regression_begin, SteadyClock::now());
    finish(RegressionFailureReason::RANK_DEFICIENT);
    return false;
  }
  if (!std::isfinite(barrier->condition_number) ||
      barrier->condition_number > config_.max_condition_number) {
    if (error) {
      std::ostringstream stream;
      stream << "condition-number failure: rank=" << rank
             << ", condition_number=" << barrier->condition_number;
      *error = stream.str();
    }
    if (timing) timing->regression_ms =
        elapsedMs(regression_begin, SteadyClock::now());
    finish(RegressionFailureReason::CONDITION_NUMBER_FAIL);
    return false;
  }

  svd.setThreshold(config_.relative_rank_tolerance);
  const Eigen::VectorXd theta = svd.solve(values);
  barrier->A << theta(0), theta(3), theta(4), theta(3), theta(1), theta(5),
      theta(4), theta(5), theta(2);
  barrier->b = theta.segment<3>(6);
  barrier->c = theta(9);

  const Eigen::VectorXd residual = design * theta - values;
  barrier->rmse = std::sqrt(residual.squaredNorm() / residual.size());
  barrier->max_abs_error = residual.cwiseAbs().maxCoeff();
  if (diagnostics) {
    diagnostics->query_points_N = query_points;
    diagnostics->nearest_points_N = nearest_points;
    diagnostics->raw_values.assign(values.data(), values.data() + values.size());
    const Eigen::VectorXd fitted = design * theta;
    diagnostics->fitted_values.assign(fitted.data(),
                                      fitted.data() + fitted.size());
  }

  barrier->valid = std::isfinite(barrier->rmse) &&
                   std::isfinite(barrier->max_abs_error) &&
                   barrier->rmse <= config_.max_rmse &&
                   barrier->max_abs_error <= config_.max_abs_error;
  if (timing) timing->regression_ms =
      elapsedMs(regression_begin, SteadyClock::now());
  if (!barrier->valid && error) {
    std::ostringstream stream;
    stream << "fit residual exceeds limits: rmse=" << barrier->rmse
           << ", max_abs_error=" << barrier->max_abs_error;
    *error = stream.str();
  }
  if (!barrier->valid) {
    finish(!std::isfinite(barrier->rmse) ||
                   barrier->rmse > config_.max_rmse
               ? RegressionFailureReason::RMSE_FAIL
               : RegressionFailureReason::MAX_ERROR_FAIL);
    return false;
  }
  finish(RegressionFailureReason::VALID);
  return barrier->valid;
}

double DepthCbfRegressor::rawBarrierValue(
    const PointCloud& local_cloud_N, const Eigen::Vector3d& query_point_N,
    Eigen::Vector3d* nearest_point_N, bool* within_support) const {
  if (within_support) *within_support = false;
  if (local_cloud_N.empty()) return std::numeric_limits<double>::infinity();
  PointCloud::Ptr cloud(new PointCloud(local_cloud_N));
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  pcl::PointXYZ query_N_pcl;
  query_N_pcl.x = static_cast<float>(query_point_N.x());
  query_N_pcl.y = static_cast<float>(query_point_N.y());
  query_N_pcl.z = static_cast<float>(query_point_N.z());
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  if (kdtree.nearestKSearch(query_N_pcl, 1, indices,
                           squared_distances) != 1) {
    return std::numeric_limits<double>::infinity();
  }
  if (nearest_point_N) {
    const pcl::PointXYZ& nearest_surface_N_pcl = cloud->points[indices[0]];
    *nearest_point_N = Eigen::Vector3d(nearest_surface_N_pcl.x,
                                      nearest_surface_N_pcl.y,
                                      nearest_surface_N_pcl.z);
  }
  if (within_support) {
    *within_support =
        squared_distances[0] <= config_.nearest_neighbor_max_distance *
                                    config_.nearest_neighbor_max_distance;
  }
  return static_cast<double>(squared_distances[0]) -
         config_.d_safe * config_.d_safe;
}

}  // namespace depth_cbf
}  // namespace coni_mpc
