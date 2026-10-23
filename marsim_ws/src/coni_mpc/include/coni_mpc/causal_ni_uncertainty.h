#pragma once

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <string>

namespace coni_mpc {
namespace causal_ni {

constexpr std::size_t kStageCount = 21;
constexpr std::size_t kIntervalCount = kStageCount - 1;
constexpr std::size_t kDefaultSubstepsPerStage = 5;
constexpr std::size_t kDefaultSubstepCount =
    kIntervalCount * kDefaultSubstepsPerStage;

// Original MF-MPSC causal non-inertial prediction tuple. Values are held
// constant on each stage interval; element 20 is the terminal node value.
struct PredictionSequence {
  std::array<Eigen::Vector3d, kStageCount> accel_n;
  std::array<double, kStageCount> omega;
  std::array<double, kStageCount> beta;
};

struct InitialState {
  Eigen::Vector3d G_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d Vg_world = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double omega = 0.0;
  Eigen::Vector3d P_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d W_world = Eigen::Vector3d::Zero();
};

// Bounds are deliberately invalid by default. Unknown bounds must never be
// interpreted as zero uncertainty.
struct PredictorBounds {
  std::array<double, kStageCount> d_accel{};
  std::array<double, kStageCount> d_omega{};
  std::array<double, kStageCount> d_beta{};
  bool valid = false;
};

struct TubeConfig {
  double stage_dt = 0.1;
  std::size_t substeps_per_stage = kDefaultSubstepsPerStage;
  double lambda = 5.0;
  double omega0_tolerance = 1.0e-12;
  // Equation (21) applies to a relative-frame command. A command already
  // stored and held in world coordinates has no future UGV coupling.
  bool fixed_world_command = false;
};

struct ResidualDiagnostic {
  Eigen::Matrix3d M_omega = Eigen::Matrix3d::Zero();
  Eigen::Vector3d delta_f = Eigen::Vector3d::Zero();
};

// Exact equation (6), with no small-error approximation.
ResidualDiagnostic exactNonInertialResidual(
    const Eigen::Vector3d& p, const Eigen::Vector3d& v,
    const Eigen::Vector3d& accel_error,
    const Eigen::Vector3d& omega_hat,
    const Eigen::Vector3d& omega_error,
    const Eigen::Vector3d& beta_error);

// Directional scalar certificate in equation (7).
double directionalResidualUpperBound(
    const Eigen::Vector3d& g, const Eigen::Vector3d& p,
    const Eigen::Vector3d& v, const Eigen::Vector3d& omega_hat,
    double d_accel, double d_omega, double d_beta);

struct SubstepSample {
  double time = 0.0;
  Eigen::Vector3d nominal_G_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d nominal_Vg_world = Eigen::Vector3d::Zero();
  double nominal_yaw = 0.0;
  double r_omega = 0.0;
  double E_omega = 0.0;
  double E_theta = 0.0;
  double E_R = 0.0;
  double E_Vg = 0.0;
  double E_G = 0.0;
  double E_P = 0.0;
  double E_W = 0.0;
  double E_q = 0.0;
};

// Uniform upper bounds over one complete 20-ms reachability subinterval.
// E_W_max is a comparison bound and does not assume endpoint monotonicity.
struct SubintervalEnvelope {
  double time_begin = 0.0;
  double time_end = 0.0;
  double E_omega_max = 0.0;
  double E_theta_max = 0.0;
  double E_R_max = 0.0;
  double E_Vg_max = 0.0;
  double E_G_max = 0.0;
  double E_P_max = 0.0;
  double E_W_max = 0.0;
  double E_q_max = 0.0;
};

// Uniform upper bounds over an entire 100-ms MPC stage.
struct StageEnvelope {
  double time_begin = 0.0;
  double time_end = 0.0;
  double E_omega_max = 0.0;
  double E_theta_max = 0.0;
  double E_R_max = 0.0;
  double E_Vg_max = 0.0;
  double E_G_max = 0.0;
  double E_P_max = 0.0;
  double E_W_max = 0.0;
  double E_q_max = 0.0;
  double omega_abs_max = 0.0;
  double Omega = 0.0;
  double C = 0.0;
};

struct TubeResult {
  bool valid = false;
  std::string reason;
  std::array<SubstepSample, kDefaultSubstepCount + 1> samples;
  std::array<SubintervalEnvelope, kDefaultSubstepCount> subintervals;
  std::array<StageEnvelope, kIntervalCount> stages;
};

// command_norm_bounds[j] >= ||u|| and relative_distance_bounds[j] >=
// ||P_hat-G_hat|| on the whole stage. Version 1 fixes five 20-ms substeps;
// other substep counts are rejected so the fixed-size result remains explicit.
TubeResult buildTube(
    const PredictionSequence& prediction, const InitialState& initial,
    const PredictorBounds& bounds,
    const std::array<double, kIntervalCount>& command_norm_bounds,
    const std::array<double, kIntervalCount>& relative_distance_bounds,
    const TubeConfig& config = TubeConfig());

}  // namespace causal_ni
}  // namespace coni_mpc
