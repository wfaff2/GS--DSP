#include "coni_mpc/causal_ni_uncertainty.h"

#include <unsupported/Eigen/MatrixFunctions>

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <vector>

namespace coni_mpc {
namespace causal_ni {
namespace {

Eigen::Matrix3d crossMatrix(const Eigen::Vector3d& x) {
  Eigen::Matrix3d result;
  result << 0.0, -x.z(), x.y(), x.z(), 0.0, -x.x(), -x.y(), x.x(), 0.0;
  return result;
}

bool finiteNonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool finiteVector(const Eigen::Vector3d& value) { return value.allFinite(); }

std::complex<double> F1(double omega, double duration) {
  const std::complex<double> z(0.0, omega * duration);
  if (std::abs(z) < 1.0e-3) {
    std::complex<double> sum(0.0, 0.0);
    std::complex<double> power(1.0, 0.0);
    double factorial = 1.0;
    for (int n = 0; n <= 8; ++n) {
      factorial *= static_cast<double>(n + 1);
      sum += power / factorial;
      power *= z;
    }
    return duration * sum;
  }
  return duration * (std::exp(z) - std::complex<double>(1.0, 0.0)) / z;
}

std::complex<double> F2(double omega, double duration) {
  const std::complex<double> z(0.0, omega * duration);
  if (std::abs(z) < 1.0e-3) {
    std::complex<double> sum(0.0, 0.0);
    std::complex<double> power(1.0, 0.0);
    double factorial = 1.0;
    for (int n = 0; n <= 8; ++n) {
      factorial *= static_cast<double>(n + 2);
      sum += power / factorial;
      power *= z;
    }
    return duration * duration * sum;
  }
  return duration * duration *
         (std::exp(z) - std::complex<double>(1.0, 0.0) - z) / (z * z);
}

void integrateNominal(double duration, const Eigen::Vector3d& accel_n,
                      double omega, Eigen::Vector3d* G_world,
                      Eigen::Vector3d* Vg_world, double* yaw) {
  const std::complex<double> a(accel_n.x(), accel_n.y());
  const std::complex<double> rotation =
      std::exp(std::complex<double>(0.0, *yaw));
  const std::complex<double> old_velocity(Vg_world->x(), Vg_world->y());
  const std::complex<double> old_position(G_world->x(), G_world->y());
  const std::complex<double> new_position =
      old_position + old_velocity * duration + rotation * a * F2(omega, duration);
  const std::complex<double> new_velocity =
      old_velocity + rotation * a * F1(omega, duration);

  G_world->x() = new_position.real();
  G_world->y() = new_position.imag();
  G_world->z() += Vg_world->z() * duration +
                  0.5 * accel_n.z() * duration * duration;
  Vg_world->x() = new_velocity.real();
  Vg_world->y() = new_velocity.imag();
  Vg_world->z() += accel_n.z() * duration;
  *yaw += omega * duration;
}

double effectiveOmega(double d_omega, double beta_bound_integral,
                      double residual) {
  return std::min(d_omega, beta_bound_integral + std::abs(residual));
}

struct EffectiveOmegaSegment {
  double integral = 0.0;
  double maximum = 0.0;
  double end_value = 0.0;
};

// Integrates min(d_omega, B0+d_beta*t+|r0-beta*t|) exactly. Breakpoints are
// the residual sign change and intersections of each linear piece with the cap.
EffectiveOmegaSegment integrateEffectiveOmega(
    double duration, double d_omega, double B0, double d_beta,
    double r0, double beta) {
  std::vector<double> points{0.0, duration};
  if (beta != 0.0) {
    const double zero = r0 / beta;
    if (zero > 0.0 && zero < duration) points.push_back(zero);
  }
  std::sort(points.begin(), points.end());

  std::vector<double> refined = points;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double a = points[i];
    const double b = points[i + 1];
    const double ha = B0 + d_beta * a + std::abs(r0 - beta * a);
    const double hb = B0 + d_beta * b + std::abs(r0 - beta * b);
    const double slope = (hb - ha) / (b - a);
    if (slope != 0.0) {
      const double crossing = a + (d_omega - ha) / slope;
      if (crossing > a && crossing < b) refined.push_back(crossing);
    }
  }
  std::sort(refined.begin(), refined.end());

  EffectiveOmegaSegment result;
  for (std::size_t i = 0; i + 1 < refined.size(); ++i) {
    const double a = refined[i];
    const double b = refined[i + 1];
    const double ea = effectiveOmega(
        d_omega, B0 + d_beta * a, r0 - beta * a);
    const double eb = effectiveOmega(
        d_omega, B0 + d_beta * b, r0 - beta * b);
    result.integral += 0.5 * (ea + eb) * (b - a);
    result.maximum = std::max(result.maximum, std::max(ea, eb));
  }
  result.end_value = effectiveOmega(
      d_omega, B0 + d_beta * duration, r0 - beta * duration);
  return result;
}

void setFailure(const std::string& reason, TubeResult* result) {
  result->valid = false;
  result->reason = reason;
}

bool validate(const PredictionSequence& prediction, const InitialState& initial,
              const PredictorBounds& bounds,
              const std::array<double, kIntervalCount>& command_bounds,
              const std::array<double, kIntervalCount>& distance_bounds,
              const TubeConfig& config, TubeResult* result) {
  if (!bounds.valid) {
    setFailure("predictor-error bounds are not certified valid", result);
    return false;
  }
  if (config.substeps_per_stage != kDefaultSubstepsPerStage ||
      !finiteNonnegative(config.stage_dt) || config.stage_dt == 0.0 ||
      !finiteNonnegative(config.lambda) || config.lambda == 0.0 ||
      !finiteNonnegative(config.omega0_tolerance)) {
    setFailure("invalid tube configuration", result);
    return false;
  }
  if (!finiteVector(initial.G_world) || !finiteVector(initial.Vg_world) ||
      !finiteVector(initial.P_world) || !finiteVector(initial.W_world) ||
      !std::isfinite(initial.yaw) || !std::isfinite(initial.omega)) {
    setFailure("initial state contains a non-finite value", result);
    return false;
  }
  for (std::size_t i = 0; i < kStageCount; ++i) {
    if (!finiteVector(prediction.accel_n[i]) ||
        !std::isfinite(prediction.omega[i]) ||
        !std::isfinite(prediction.beta[i]) ||
        !finiteNonnegative(bounds.d_accel[i]) ||
        !finiteNonnegative(bounds.d_omega[i]) ||
        !finiteNonnegative(bounds.d_beta[i])) {
      setFailure("prediction or predictor-error bound contains an invalid value", result);
      return false;
    }
  }
  for (std::size_t i = 0; i < kIntervalCount; ++i) {
    if (!finiteNonnegative(command_bounds[i]) ||
        !finiteNonnegative(distance_bounds[i])) {
      setFailure("stage command or distance bound is invalid", result);
      return false;
    }
  }
  if (std::abs(prediction.omega[0] - initial.omega) >
      config.omega0_tolerance) {
    setFailure("prediction omega[0] does not equal the measured current omega", result);
    return false;
  }
  return true;
}

}  // namespace

ResidualDiagnostic exactNonInertialResidual(
    const Eigen::Vector3d& p, const Eigen::Vector3d& v,
    const Eigen::Vector3d& accel_error,
    const Eigen::Vector3d& omega_hat,
    const Eigen::Vector3d& omega_error,
    const Eigen::Vector3d& beta_error) {
  ResidualDiagnostic result;
  result.M_omega = 2.0 * crossMatrix(v) +
                   crossMatrix(omega_hat.cross(p)) +
                   crossMatrix(omega_hat) * crossMatrix(p);
  result.delta_f = -accel_error + result.M_omega * omega_error +
                   crossMatrix(p) * beta_error -
                   omega_error.cross(omega_error.cross(p));
  return result;
}

double directionalResidualUpperBound(
    const Eigen::Vector3d& g, const Eigen::Vector3d& p,
    const Eigen::Vector3d& v, const Eigen::Vector3d& omega_hat,
    double d_accel, double d_omega, double d_beta) {
  if (!finiteNonnegative(d_accel) || !finiteNonnegative(d_omega) ||
      !finiteNonnegative(d_beta) || !g.allFinite() || !p.allFinite() ||
      !v.allFinite() || !omega_hat.allFinite()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const Eigen::Matrix3d M = 2.0 * crossMatrix(v) +
                            crossMatrix(omega_hat.cross(p)) +
                            crossMatrix(omega_hat) * crossMatrix(p);
  return d_accel * g.norm() + d_omega * (M.transpose() * g).norm() +
         d_beta * (crossMatrix(p).transpose() * g).norm() +
         d_omega * d_omega * p.norm() * g.norm();
}

TubeResult buildTube(
    const PredictionSequence& prediction, const InitialState& initial,
    const PredictorBounds& bounds,
    const std::array<double, kIntervalCount>& command_norm_bounds,
    const std::array<double, kIntervalCount>& relative_distance_bounds,
    const TubeConfig& config) {
  TubeResult result;
  if (!validate(prediction, initial, bounds, command_norm_bounds,
                relative_distance_bounds, config, &result)) {
    return result;
  }

  const double substep_dt =
      config.stage_dt / static_cast<double>(config.substeps_per_stage);
  Eigen::Vector3d G = initial.G_world;
  Eigen::Vector3d Vg = initial.Vg_world;
  double yaw = initial.yaw;
  double beta_integral = 0.0;
  double beta_bound_integral = 0.0;
  double E_theta = 0.0;
  double E_Vg = 0.0;
  double E_G = 0.0;

  auto fillKinematicSample = [&](std::size_t sample_index,
                                 std::size_t signal_index) {
    SubstepSample& sample = result.samples[sample_index];
    sample.time = static_cast<double>(sample_index) * substep_dt;
    sample.nominal_G_world = G;
    sample.nominal_Vg_world = Vg;
    sample.nominal_yaw = yaw;
    sample.r_omega = prediction.omega[signal_index] - initial.omega -
                     beta_integral;
    sample.E_omega = effectiveOmega(
        bounds.d_omega[signal_index], beta_bound_integral, sample.r_omega);
    sample.E_theta = E_theta;
    sample.E_R = std::min(2.0, E_theta);
    sample.E_Vg = E_Vg;
    sample.E_G = E_G;
  };
  fillKinematicSample(0, 0);

  for (std::size_t stage = 0; stage < kIntervalCount; ++stage) {
    StageEnvelope& stage_envelope = result.stages[stage];
    stage_envelope.time_begin = static_cast<double>(stage) * config.stage_dt;
    stage_envelope.time_end = stage_envelope.time_begin + config.stage_dt;
    stage_envelope.omega_abs_max =
        std::max(std::abs(prediction.omega[stage]),
                 std::abs(prediction.omega[stage + 1]));

    for (std::size_t local = 0; local < config.substeps_per_stage; ++local) {
      const std::size_t interval_index =
          stage * config.substeps_per_stage + local;
      SubintervalEnvelope& envelope = result.subintervals[interval_index];
      envelope.time_begin = static_cast<double>(interval_index) * substep_dt;
      envelope.time_end = envelope.time_begin + substep_dt;

      const double r_start = prediction.omega[stage] - initial.omega -
                             beta_integral;
      const EffectiveOmegaSegment omega_segment = integrateEffectiveOmega(
          substep_dt, bounds.d_omega[stage], beta_bound_integral,
          bounds.d_beta[stage], r_start, prediction.beta[stage]);
      E_theta += omega_segment.integral;
      const double E_R_end = std::min(2.0, E_theta);

      // Conservative upper comparison quadrature for (15). E_R is
      // nondecreasing, so its right-end value bounds the whole subinterval.
      const double velocity_derivative_bound =
          bounds.d_accel[stage] +
          E_R_end * prediction.accel_n[stage].norm();
      E_Vg += substep_dt * velocity_derivative_bound;
      E_G += substep_dt * E_Vg;

      integrateNominal(substep_dt, prediction.accel_n[stage],
                       prediction.omega[stage], &G, &Vg, &yaw);
      beta_integral += prediction.beta[stage] * substep_dt;
      beta_bound_integral += bounds.d_beta[stage] * substep_dt;

      const std::size_t next_sample = interval_index + 1;
      const bool at_stage_boundary = local + 1 == config.substeps_per_stage;
      const std::size_t next_signal =
          at_stage_boundary ? stage + 1 : stage;
      fillKinematicSample(next_sample, next_signal);

      envelope.E_omega_max = std::max(
          omega_segment.maximum, result.samples[next_sample].E_omega);
      envelope.E_theta_max = E_theta;
      envelope.E_R_max = E_R_end;
      envelope.E_Vg_max = E_Vg;
      envelope.E_G_max = E_G;

      stage_envelope.E_omega_max =
          std::max(stage_envelope.E_omega_max, envelope.E_omega_max);
      stage_envelope.E_theta_max = E_theta;
      stage_envelope.E_R_max = E_R_end;
      stage_envelope.E_Vg_max = E_Vg;
      stage_envelope.E_G_max = E_G;
    }
  }

  // The stage-uniform coefficients are now known. Propagate (20) exactly.
  Eigen::Vector3d reachability_state(0.0, 0.0, 1.0);
  result.samples[0].E_P = 0.0;
  result.samples[0].E_W = 0.0;
  for (std::size_t stage = 0; stage < kIntervalCount; ++stage) {
    StageEnvelope& stage_envelope = result.stages[stage];
    if (config.fixed_world_command) {
      stage_envelope.Omega = 0.0;
      stage_envelope.C = 0.0;
    } else {
      stage_envelope.Omega =
          stage_envelope.omega_abs_max + stage_envelope.E_omega_max;
      stage_envelope.C =
          stage_envelope.E_Vg_max +
          stage_envelope.E_R_max * command_norm_bounds[stage] +
          stage_envelope.E_omega_max * relative_distance_bounds[stage] +
          stage_envelope.Omega * stage_envelope.E_G_max;
    }

    Eigen::Matrix3d comparison = Eigen::Matrix3d::Zero();
    comparison(0, 1) = 1.0;
    comparison(1, 0) = config.lambda * stage_envelope.Omega;
    comparison(1, 1) = -config.lambda;
    comparison(1, 2) = config.lambda * stage_envelope.C;
    const Eigen::Matrix3d transition = (substep_dt * comparison).exp();

    for (std::size_t local = 0; local < config.substeps_per_stage; ++local) {
      const std::size_t interval_index =
          stage * config.substeps_per_stage + local;
      const double E_W_start = reachability_state.y();
      reachability_state = transition * reachability_state;
      const std::size_t next_sample = interval_index + 1;
      SubstepSample& sample = result.samples[next_sample];
      sample.E_P = std::max(0.0, reachability_state.x());
      sample.E_W = std::max(0.0, reachability_state.y());

      SubintervalEnvelope& envelope = result.subintervals[interval_index];
      envelope.E_P_max = sample.E_P;
      // Since F(t)=Omega*E_P(t)+C is nondecreasing, the scalar comparison
      // equation gives this rigorous whole-interval bound even if E_W falls.
      envelope.E_W_max = std::max(
          E_W_start,
          stage_envelope.Omega * envelope.E_P_max + stage_envelope.C);
      envelope.E_q_max =
          stage_envelope.C + stage_envelope.Omega * envelope.E_P_max;
      sample.E_q = envelope.E_q_max;

      stage_envelope.E_P_max = sample.E_P;
      stage_envelope.E_W_max =
          std::max(stage_envelope.E_W_max, envelope.E_W_max);
      stage_envelope.E_q_max =
          std::max(stage_envelope.E_q_max, envelope.E_q_max);
    }
  }

  result.valid = true;
  result.reason = "certified inputs accepted";
  return result;
}

}  // namespace causal_ni
}  // namespace coni_mpc
