#pragma once

#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <ros/time.h>

#include "coni_mpc/depth_cbf/point_cloud_preprocessor.h"

namespace coni_mpc {
namespace depth_cbf {

struct LocalBarrier {
  Eigen::Vector3d center_N = Eigen::Vector3d::Zero();
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();
  double c = 0.0;

  double rmse = std::numeric_limits<double>::infinity();
  double max_abs_error = std::numeric_limits<double>::infinity();
  double condition_number = std::numeric_limits<double>::infinity();
  int rank = 0;
  double min_singular_value = 0.0;
  double max_singular_value = 0.0;
  bool valid = false;
  ros::Time stamp;

  double evaluate(const Eigen::Vector3d& p_N) const;
  Eigen::Vector3d gradient(const Eigen::Vector3d& p_N) const;
  Eigen::Matrix3d hessian() const;
};

struct RegressionConfig {
  int query_mesh_half_width_steps = 2;
  double query_mesh_spacing = 0.15;
  double d_safe = 0.15;
  double nearest_neighbor_max_distance = 0.75;
  double relative_rank_tolerance = 1e-10;
  double max_condition_number = 1e8;
  double max_rmse = 0.25;
  double max_abs_error = 0.75;
};

struct RegressionDiagnostics {
  std::vector<Eigen::Vector3d> query_points_N;
  std::vector<Eigen::Vector3d> nearest_points_N;
  std::vector<double> raw_values;
  std::vector<double> fitted_values;
};

enum class RegressionFailureReason {
  VALID = 0,
  EMPTY_CLOUD,
  INSUFFICIENT_SUPPORT,
  RANK_DEFICIENT,
  CONDITION_NUMBER_FAIL,
  RMSE_FAIL,
  MAX_ERROR_FAIL,
  INVALID_CONFIGURATION,
  INTERNAL_ERROR,
};

const char* regressionFailureReasonName(RegressionFailureReason reason);

struct RegressionTiming {
  double kdtree_build_ms = 0.0;
  double query_generation_ms = 0.0;
  double nearest_neighbor_query_ms = 0.0;
  double regression_ms = 0.0;
  double total_ms = 0.0;
};

class DepthCbfRegressor {
 public:
  explicit DepthCbfRegressor(const RegressionConfig& config);

  static bool validateConfig(const RegressionConfig& config,
                             std::string* error);

  bool fit(const PointCloud& local_cloud_N,
           const Eigen::Vector3d& query_center_N,
           const ros::Time& stamp,
           LocalBarrier* barrier,
           RegressionDiagnostics* diagnostics,
           std::string* error,
           RegressionTiming* timing = nullptr,
           RegressionFailureReason* failure_reason = nullptr) const;

  double rawBarrierValue(const PointCloud& local_cloud_N,
                         const Eigen::Vector3d& query_point_N,
                         Eigen::Vector3d* nearest_point_N,
                         bool* within_support) const;

  const RegressionConfig& config() const { return config_; }

 private:
  RegressionConfig config_;
};

}  // namespace depth_cbf
}  // namespace coni_mpc
