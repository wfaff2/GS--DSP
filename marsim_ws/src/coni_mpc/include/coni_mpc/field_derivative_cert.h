#pragma once

#include "coni_mpc/field_hocbf.h"

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstdint>
#include <limits>
#include <string>

namespace coni_mpc {
namespace field_derivative_cert {

// A certificate snapshot contains world-frame voxel centers.  The two
// metadata flags are deliberately independent of mathematical validity: a
// numerically valid field is not necessarily a certified persistent cover.
struct FieldSnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  field_hocbf::KinematicPoints points;
  field_hocbf::Config config;
  std::uint64_t id = 0;
  bool persistent_static = false;
  bool coverage_certified = false;
};

struct Evaluation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = std::numeric_limits<double>::quiet_NaN();
  double h = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d g = Eigen::Vector3d::Zero();
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  double G = std::numeric_limits<double>::quiet_NaN();
  double H_bound = std::numeric_limits<double>::quiet_NaN();
  double T = std::numeric_limits<double>::quiet_NaN();
  double L = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
  bool metadata_valid = false;
  std::string reason;
};

// Evaluates (22), (26), and the analytic ball bounds (27)--(31).  radius is
// E_P and every snapshot point that can intersect R+E_P+epsilon is retained.
Evaluation evaluate(const FieldSnapshot& snapshot, const Eigen::Vector3d& P,
                    double radius);

struct RobustMarginResult {
  double rho = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
  std::string reason;
};

// Exact static-world-field HOCBF expression (32).
double exactPsi2(const Evaluation& field, const Eigen::Vector3d& W,
                 const Eigen::Vector3d& q, double lambda, double gamma1,
                 double gamma2);

// Fully explicit robust loss bound (33).  field must have been evaluated on
// the ball of radius E_P about the same nominal position.
RobustMarginResult robustMargin(const Evaluation& field,
                                const Eigen::Vector3d& W,
                                const Eigen::Vector3d& q, double E_P,
                                double E_W, double E_q, double lambda,
                                double gamma1, double gamma2);

}  // namespace field_derivative_cert
}  // namespace coni_mpc
