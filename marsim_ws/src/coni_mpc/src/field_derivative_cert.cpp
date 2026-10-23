#include "coni_mpc/field_derivative_cert.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace coni_mpc {
namespace field_derivative_cert {
namespace {

constexpr double kCutoffFirstMax = 1.875;
constexpr double kCutoffSecondMax = 15.0;
constexpr double kCutoffThirdMax = 60.0;

bool finiteNonnegative(double x) { return std::isfinite(x) && x >= 0.0; }

double logAdd(double a, double b) {
  if (a == -std::numeric_limits<double>::infinity()) return b;
  if (b == -std::numeric_limits<double>::infinity()) return a;
  const double high = std::max(a, b);
  return high + std::log(std::exp(a - high) + std::exp(b - high));
}

bool expFinite(double x, double* value) {
  if (!std::isfinite(x) || x > std::log(std::numeric_limits<double>::max())) {
    return false;
  }
  *value = std::exp(x);
  return std::isfinite(*value);
}

bool validPoint(const field_hocbf::KinematicPoint& point) {
  return point.position.allFinite() && std::isfinite(point.occupancy) &&
         point.occupancy >= 0.0 && point.occupancy <= 1.0;
}

void cutoffWithThird(double rho, const field_hocbf::Config& config,
                     double* value, double* first, double* second,
                     double* third) {
  field_hocbf::radialCutoff(rho, config, *value, *first, *second);
  *third = 0.0;
  const double inner = config.prune_radius - config.cutoff_width;
  if (rho > inner && rho < config.prune_radius) {
    const double u = (config.prune_radius - rho) / config.cutoff_width;
    *third = -60.0 * (1.0 - 6.0 * u + 6.0 * u * u) /
             (config.cutoff_width * config.cutoff_width *
              config.cutoff_width);
  }
}

struct ExactTerm {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d n = Eigen::Vector3d::Zero();
  Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
  double cutoff = 0.0;
  double cutoff_first = 0.0;
  double cutoff_second = 0.0;
  double log_amplitude = 0.0;
};

}  // namespace

Evaluation evaluate(const FieldSnapshot& snapshot, const Eigen::Vector3d& P,
                    double radius) {
  Evaluation out;
  out.center = P;
  out.radius = radius;
  out.metadata_valid =
      snapshot.persistent_static && snapshot.coverage_certified;
  if (!field_hocbf::validConfig(snapshot.config)) {
    out.reason = "invalid field configuration";
    return out;
  }
  if (!P.allFinite() || !finiteNonnegative(radius)) {
    out.reason = "position or uncertainty radius is invalid";
    return out;
  }
  for (const auto& point : snapshot.points) {
    if (!validPoint(point)) {
      out.reason = "snapshot contains an invalid world voxel";
      return out;
    }
  }

  const field_hocbf::Config& config = snapshot.config;
  const double sigma = config.sigma;
  const double inv_sigma = 1.0 / sigma;
  const double epsilon = config.distance_epsilon;
  const double half_diagonal =
      std::sqrt(3.0) * config.voxel_size * 0.5;
  const double B = config.uav_radius + half_diagonal +
                   config.distance_margin;
  const double inner = config.prune_radius - config.cutoff_width;
  const double log_background = -config.clear_field_value * inv_sigma;
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

  std::vector<ExactTerm, Eigen::aligned_allocator<ExactTerm>> exact_terms;
  exact_terms.reserve(snapshot.points.size());
  double exact_scale = log_background;
  double log_A_sum = -std::numeric_limits<double>::infinity();
  double log_S1 = -std::numeric_limits<double>::infinity();
  double log_S2 = -std::numeric_limits<double>::infinity();
  double log_S3 = -std::numeric_limits<double>::infinity();
  double log_S_lower = log_background;

  for (const auto& point : snapshot.points) {
    if (!(point.occupancy > 0.0)) continue;
    const double confidence = field_hocbf::smootherstep(
        point.occupancy / config.occupancy_full_confidence);
    if (!(confidence > 0.0)) continue;
    const double weight = confidence *
        (1.0 + config.occupancy_alpha * point.occupancy);
    if (!(weight > 0.0) || !std::isfinite(weight)) {
      out.reason = "field weight overflowed";
      return out;
    }
    const double log_weight = std::log(weight);
    log_A_sum = logAdd(log_A_sum, log_weight);

    const Eigen::Vector3d displacement = P - point.position;
    const double distance_to_center = displacement.norm();
    if (!std::isfinite(distance_to_center)) {
      out.reason = "voxel distance overflowed";
      return out;
    }
    const double s = std::sqrt(displacement.squaredNorm() + epsilon * epsilon);
    const double rho = s - epsilon;
    double cutoff = 0.0, cutoff_first = 0.0, cutoff_second = 0.0,
           cutoff_third = 0.0;
    cutoffWithThird(rho, config, &cutoff, &cutoff_first, &cutoff_second,
                    &cutoff_third);
    if (cutoff > 0.0) {
      ExactTerm term;
      term.n = displacement / s;
      term.D = identity / s -
               displacement * displacement.transpose() / (s * s * s);
      term.cutoff = cutoff;
      term.cutoff_first = cutoff_first;
      term.cutoff_second = cutoff_second;
      term.log_amplitude = log_weight - (rho - B) * inv_sigma;
      if (!std::isfinite(term.log_amplitude) || !term.n.allFinite() ||
          !term.D.allFinite()) {
        out.reason = "exact field term overflowed";
        return out;
      }
      exact_scale = std::max(exact_scale, term.log_amplitude);
      exact_terms.push_back(term);
    }

    // Full uncertainty support broadphase required by Section 5.  Points not
    // in this set have identically zero contribution over the whole ball.
    if (!(distance_to_center < config.prune_radius + radius + epsilon)) {
      continue;
    }
    const double r_minus = std::max(0.0, distance_to_center - radius);
    const double r_plus = distance_to_center + radius;
    const double s_minus = std::sqrt(r_minus * r_minus + epsilon * epsilon);
    const double rho_minus = s_minus - epsilon;
    const double rho_plus =
        std::sqrt(r_plus * r_plus + epsilon * epsilon) - epsilon;
    if (!std::isfinite(rho_minus) || !std::isfinite(rho_plus) ||
        !(s_minus > 0.0)) {
      out.reason = "uncertainty interval is invalid";
      return out;
    }

    const bool has_support = rho_minus < config.prune_radius;
    const bool intersects_taper =
        rho_plus > inner && rho_minus < config.prune_radius;
    const double C0 = has_support ? 1.0 : 0.0;
    const double C1 = intersects_taper
                          ? kCutoffFirstMax / config.cutoff_width
                          : 0.0;
    const double C2 = intersects_taper
                          ? kCutoffSecondMax /
                                (config.cutoff_width * config.cutoff_width)
                          : 0.0;
    const double C3 = intersects_taper
                          ? kCutoffThirdMax /
                                (config.cutoff_width * config.cutoff_width *
                                 config.cutoff_width)
                          : 0.0;

    double cutoff_plus = 0.0, ignored_first = 0.0, ignored_second = 0.0;
    field_hocbf::radialCutoff(rho_plus, config, cutoff_plus, ignored_first,
                              ignored_second);
    if (cutoff_plus > 0.0) {
      const double log_lower =
          log_weight + std::log(cutoff_plus) - (rho_plus - B) * inv_sigma;
      if (!std::isfinite(log_lower)) {
        out.reason = "lower partition bound overflowed";
        return out;
      }
      log_S_lower = logAdd(log_S_lower, log_lower);
    }
    if (!has_support) continue;

    const double K1 = C1 + C0 * inv_sigma;
    const double K2 = C2 + 2.0 * C1 * inv_sigma +
                      C0 * inv_sigma * inv_sigma + K1 / s_minus;
    const double K3 =
        C3 + 3.0 * C2 * inv_sigma +
        3.0 * C1 * inv_sigma * inv_sigma +
        C0 * inv_sigma * inv_sigma * inv_sigma +
        3.0 * (C2 + 2.0 * C1 * inv_sigma +
               C0 * inv_sigma * inv_sigma) /
            s_minus +
        6.0 * K1 / (s_minus * s_minus);
    const double log_A_plus = log_weight - (rho_minus - B) * inv_sigma;
    if (!std::isfinite(log_A_plus) || !std::isfinite(K1) ||
        !std::isfinite(K2) || !std::isfinite(K3)) {
      out.reason = "derivative bound term overflowed";
      return out;
    }
    if (K1 > 0.0) log_S1 = logAdd(log_S1, log_A_plus + std::log(K1));
    if (K2 > 0.0) log_S2 = logAdd(log_S2, log_A_plus + std::log(K2));
    if (K3 > 0.0) log_S3 = logAdd(log_S3, log_A_plus + std::log(K3));
  }

  double partition = std::exp(log_background - exact_scale);
  Eigen::Vector3d partition_gradient = Eigen::Vector3d::Zero();
  Eigen::Matrix3d partition_hessian = Eigen::Matrix3d::Zero();
  for (const auto& term : exact_terms) {
    const double amplitude = std::exp(term.log_amplitude - exact_scale);
    const double radial_first =
        term.cutoff_first - term.cutoff * inv_sigma;
    const double radial_second =
        term.cutoff_second - 2.0 * term.cutoff_first * inv_sigma +
        term.cutoff * inv_sigma * inv_sigma;
    partition += amplitude * term.cutoff;
    partition_gradient += amplitude * radial_first * term.n;
    partition_hessian += amplitude *
        (radial_second * term.n * term.n.transpose() +
         radial_first * term.D);
  }
  if (!(partition > 0.0) || !std::isfinite(partition)) {
    out.reason = "exact field partition is invalid";
    return out;
  }
  const Eigen::Vector3d scaled_gradient = partition_gradient / partition;
  out.h = -sigma * (exact_scale + std::log(partition));
  out.g = -sigma * scaled_gradient;
  out.H = -sigma *
          (partition_hessian / partition -
           scaled_gradient * scaled_gradient.transpose());

  const auto ratio = [&](double log_numerator, double* value) {
    if (log_numerator == -std::numeric_limits<double>::infinity()) {
      *value = 0.0;
      return true;
    }
    return expFinite(log_numerator - log_S_lower, value);
  };
  double r1 = 0.0, r2 = 0.0, r3 = 0.0;
  if (!ratio(log_S1, &r1) || !ratio(log_S2, &r2) ||
      !ratio(log_S3, &r3)) {
    out.reason = "derivative bound ratio overflowed";
    return out;
  }
  out.G = sigma * r1;
  out.H_bound = sigma * (r2 + r1 * r1);
  out.T = sigma * (r3 + 3.0 * r2 * r1 + 2.0 * r1 * r1 * r1);

  if (log_A_sum == -std::numeric_limits<double>::infinity()) {
    out.L = 1.0;
  } else {
    const double log_extra =
        std::log(sigma * kCutoffFirstMax / config.cutoff_width) +
        log_A_sum +
        (config.clear_field_value + B - inner) * inv_sigma;
    double extra = 0.0;
    if (!expFinite(log_extra, &extra)) {
      out.reason = "global Lipschitz bound overflowed";
      return out;
    }
    out.L = 1.0 + extra;
  }

  if (!std::isfinite(out.h) || !out.g.allFinite() || !out.H.allFinite() ||
      !finiteNonnegative(out.G) || !finiteNonnegative(out.H_bound) ||
      !finiteNonnegative(out.T) || !std::isfinite(out.L) || out.L < 1.0) {
    out.reason = "field evaluation produced a non-finite certificate";
    return out;
  }
  out.valid = true;
  out.reason = out.metadata_valid ? "valid certified field"
                                  : "math valid; certificate metadata absent";
  return out;
}

double exactPsi2(const Evaluation& field, const Eigen::Vector3d& W,
                 const Eigen::Vector3d& q, double lambda, double gamma1,
                 double gamma2) {
  if (!field.valid || !W.allFinite() || !q.allFinite() ||
      !std::isfinite(lambda) || lambda <= 0.0 ||
      !std::isfinite(gamma1) || gamma1 <= 0.0 ||
      !std::isfinite(gamma2) || gamma2 <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return lambda * field.g.dot(q - W) + W.dot(field.H * W) +
         (gamma1 + gamma2) * field.g.dot(W) +
         gamma1 * gamma2 * field.h;
}

RobustMarginResult robustMargin(const Evaluation& field,
                                const Eigen::Vector3d& W,
                                const Eigen::Vector3d& q, double E_P,
                                double E_W, double E_q, double lambda,
                                double gamma1, double gamma2) {
  RobustMarginResult out;
  if (!field.valid) {
    out.reason = "field derivative certificate is invalid";
    return out;
  }
  if (!W.allFinite() || !q.allFinite() || !finiteNonnegative(E_P) ||
      !finiteNonnegative(E_W) || !finiteNonnegative(E_q) ||
      !std::isfinite(lambda) || lambda <= 0.0 ||
      !std::isfinite(gamma1) || gamma1 <= 0.0 ||
      !std::isfinite(gamma2) || gamma2 <= 0.0) {
    out.reason = "robust margin input is invalid";
    return out;
  }
  if (!std::isfinite(field.radius) || E_P > field.radius) {
    out.reason = "field derivative bounds do not cover E_P";
    return out;
  }
  const double s = gamma1 + gamma2;
  const double p = gamma1 * gamma2;
  const double W_norm = W.norm();
  const double q_minus_W_norm = (q - W).norm();
  const double g_norm = field.g.norm();
  out.rho =
      lambda *
          (g_norm * (E_q + E_W) +
           field.H_bound * E_P *
               (q_minus_W_norm + E_q + E_W)) +
      field.H_bound * (2.0 * W_norm * E_W + E_W * E_W) +
      field.T * E_P * W_norm * W_norm +
      s * (g_norm * E_W +
           field.H_bound * E_P * (W_norm + E_W)) +
      p * field.G * E_P;
  if (!finiteNonnegative(out.rho)) {
    out.reason = "robust margin overflowed";
    return out;
  }
  out.valid = true;
  out.reason = "valid robust margin";
  return out;
}

}  // namespace field_derivative_cert
}  // namespace coni_mpc
