#include "coni_mpc/recursive_safety_supervisor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace coni_mpc {
namespace recursive_safety_supervisor {
namespace {

bool finitePositive(double x) { return std::isfinite(x) && x > 0.0; }

bool validConfig(const Config& config) {
  return finitePositive(config.tau) && finitePositive(config.dt) &&
         finitePositive(config.v_max_xy) && finitePositive(config.v_max_z) &&
         finitePositive(config.a_max_xy) && finitePositive(config.a_max_z) &&
         std::isfinite(config.eta) && config.eta >= 0.0 &&
         config.iterations == 30;
}

bool validSnapshotShape(
    const field_derivative_cert::FieldSnapshot& snapshot) {
  if (!field_hocbf::validConfig(snapshot.config)) return false;
  for (const auto& point : snapshot.points) {
    if (!point.position.allFinite() || !std::isfinite(point.occupancy) ||
        point.occupancy < 0.0 || point.occupancy > 1.0) {
      return false;
    }
  }
  return true;
}

Residuals candidateResiduals(
    const Config& config, const field_derivative_cert::Evaluation& field,
    const Eigen::Vector3d& W, const Eigen::Vector3d& q) {
  Residuals residuals;
  const double alpha = std::exp(-config.dt / config.tau);
  const double c = config.tau * (1.0 - alpha);
  const Eigen::Vector3d displacement =
      c * W + (config.dt - c) * q;
  const Eigen::Vector3d W_next = alpha * W + (1.0 - alpha) * q;
  const double brake_v_xy =
      std::min(config.v_max_xy, config.tau * config.a_max_xy);
  const double brake_v_z =
      std::min(config.v_max_z, config.tau * config.a_max_z);

  residuals.invariant =
      field.h - field.L * config.tau * W.norm() - config.eta;
  residuals.current_velocity_xy = brake_v_xy - W.head<2>().norm();
  residuals.current_velocity_z = brake_v_z - std::abs(W.z());
  residuals.transition =
      field.h - config.eta -
      field.L * (displacement.norm() + config.tau * W_next.norm());
  // Evaluate q=0 in its algebraically identical form to the invariant
  // residual.  This avoids turning the theorem's guaranteed candidate into a
  // false numerical failure through c + tau*alpha rounding.
  if (q.isZero(0.0)) residuals.transition = residuals.invariant;
  residuals.intersample =
      field.h - field.L *
                    (c * W.norm() + (config.dt - c) * q.norm());
  residuals.command_xy = config.v_max_xy - q.head<2>().norm();
  residuals.command_z = config.v_max_z - std::abs(q.z());
  residuals.acceleration_xy =
      config.a_max_xy - (q - W).head<2>().norm() / config.tau;
  residuals.acceleration_z =
      config.a_max_z - std::abs(q.z() - W.z()) / config.tau;
  residuals.next_braking_velocity_xy =
      brake_v_xy - W_next.head<2>().norm();
  residuals.next_braking_velocity_z =
      brake_v_z - std::abs(W_next.z());
  return residuals;
}

bool currentStateInInvariantSet(const Residuals& residuals) {
  return std::isfinite(residuals.invariant) && residuals.invariant >= 0.0 &&
         std::isfinite(residuals.current_velocity_xy) &&
         residuals.current_velocity_xy >= 0.0 &&
         std::isfinite(residuals.current_velocity_z) &&
         residuals.current_velocity_z >= 0.0;
}

}  // namespace

const char* statusName(Status status) {
  switch (status) {
    case Status::kCertifiedProposal:
      return "CERTIFIED_PROPOSAL";
    case Status::kCertifiedFiltered:
      return "CERTIFIED_FILTERED";
    case Status::kCertifiedBrake:
      return "CERTIFIED_BRAKE";
    case Status::kUncertifiedMissingCertificate:
      return "UNCERTIFIED_MISSING_CERTIFICATE";
    case Status::kUncertifiedSnapshotMismatch:
      return "UNCERTIFIED_SNAPSHOT_MISMATCH";
    case Status::kUncertifiedInvalidField:
      return "UNCERTIFIED_INVALID_FIELD";
    case Status::kUncertifiedOutsideInvariantSet:
      return "UNCERTIFIED_OUTSIDE_INVARIANT_SET";
    case Status::kInvalidInput:
      return "INVALID_INPUT";
  }
  return "INVALID_INPUT";
}

bool Residuals::allCandidateConstraintsNonnegative() const {
  const double values[] = {transition,
                           intersample,
                           command_xy,
                           command_z,
                           acceleration_xy,
                           acceleration_z,
                           next_braking_velocity_xy,
                           next_braking_velocity_z};
  for (double value : values) {
    if (!std::isfinite(value) || value < 0.0) return false;
  }
  return true;
}

Supervisor::Supervisor(const Config& config) : config_(config) {}

bool Supervisor::freezeCertifiedSnapshot(
    const field_derivative_cert::FieldSnapshot& snapshot,
    std::string* reason) {
  if (!snapshot.persistent_static || !snapshot.coverage_certified) {
    if (reason != nullptr) {
      *reason = "snapshot lacks persistent-static certified-cover metadata";
    }
    return false;
  }
  if (!validSnapshotShape(snapshot)) {
    if (reason != nullptr) *reason = "snapshot field data is invalid";
    return false;
  }
  frozen_snapshot_ = snapshot;
  frozen_snapshot_id_ = snapshot.id;
  frozen_ = true;
  if (reason != nullptr) *reason = "snapshot ID explicitly frozen";
  return true;
}

void Supervisor::clearFrozenSnapshot() {
  frozen_ = false;
  frozen_snapshot_id_ = 0;
  frozen_snapshot_ = field_derivative_cert::FieldSnapshot();
}

Output Supervisor::run(
    const field_derivative_cert::FieldSnapshot& snapshot,
    const Eigen::Vector3d& P, const Eigen::Vector3d& W,
    const Eigen::Vector3d& q_proposal) const {
  Output out;
  if (!validConfig(config_) || !P.allFinite() || !W.allFinite()) {
    out.status = Status::kInvalidInput;
    out.reason = "supervisor configuration, position, or velocity is invalid";
    return out;
  }

  const field_derivative_cert::Evaluation diagnostic_field =
      field_derivative_cert::evaluate(snapshot, P, 0.0);
  if (!frozen_ || !snapshot.persistent_static ||
      !snapshot.coverage_certified) {
    out.h = diagnostic_field.h;
    if (diagnostic_field.valid) {
      out.braking_reserve =
          diagnostic_field.h - diagnostic_field.L * config_.tau * W.norm();
      out.residuals = candidateResiduals(config_, diagnostic_field, W,
                                         Eigen::Vector3d::Zero());
    }
    out.status = Status::kUncertifiedMissingCertificate;
    out.reason = !frozen_
                     ? "no certified snapshot ID was explicitly frozen"
                     : "snapshot lacks persistent-static certified-cover metadata";
    return out;
  }
  if (snapshot.id != frozen_snapshot_id_) {
    out.h = diagnostic_field.h;
    out.status = Status::kUncertifiedSnapshotMismatch;
    out.reason = "incoming snapshot ID differs from the frozen certificate";
    return out;
  }

  // Use the explicitly frozen field data.  The incoming object supplies the
  // ID handshake only; mutation under a reused ID cannot alter a certificate.
  const field_derivative_cert::Evaluation field =
      field_derivative_cert::evaluate(frozen_snapshot_, P, 0.0);
  out.h = field.h;
  if (field.valid) {
    out.braking_reserve = field.h - field.L * config_.tau * W.norm();
    out.residuals = candidateResiduals(config_, field, W,
                                       Eigen::Vector3d::Zero());
  }
  if (!field.valid) {
    out.status = Status::kUncertifiedInvalidField;
    out.reason = field.reason;
    return out;
  }
  if (!field.metadata_valid) {
    out.status = Status::kUncertifiedMissingCertificate;
    out.reason = "frozen snapshot certificate metadata is invalid";
    return out;
  }
  if (!currentStateInInvariantSet(out.residuals)) {
    out.status = Status::kUncertifiedOutsideInvariantSet;
    out.reason = "current state is outside the certified braking invariant set";
    return out;
  }

  const Residuals zero_residuals = out.residuals;
  if (!zero_residuals.allCandidateConstraintsNonnegative()) {
    out.status = Status::kInvalidInput;
    out.reason = "numerical evaluation rejected the guaranteed brake candidate";
    return out;
  }

  if (!q_proposal.allFinite()) {
    out.certified_safe = true;
    out.status = Status::kCertifiedBrake;
    out.reason = "non-finite proposal replaced by certified world braking";
    return out;
  }

  const Residuals proposal_residuals =
      candidateResiduals(config_, field, W, q_proposal);
  if (proposal_residuals.allCandidateConstraintsNonnegative()) {
    out.q = q_proposal;
    out.mu = 1.0;
    out.certified_safe = true;
    out.backup_used = false;
    out.status = Status::kCertifiedProposal;
    out.reason = "proposal satisfies (43)--(45b)";
    out.residuals = proposal_residuals;
    return out;
  }

  double low = 0.0;
  double high = 1.0;
  for (int iteration = 0; iteration < config_.iterations; ++iteration) {
    const double middle = low + 0.5 * (high - low);
    const Eigen::Vector3d candidate = middle * q_proposal;
    const Residuals residuals = candidateResiduals(config_, field, W, candidate);
    if (residuals.allCandidateConstraintsNonnegative()) {
      low = middle;
    } else {
      high = middle;
    }
  }

  double mu = low;
  Eigen::Vector3d q = mu * q_proposal;
  Residuals residuals = candidateResiduals(config_, field, W, q);
  if (!residuals.allCandidateConstraintsNonnegative() && mu > 0.0) {
    mu = std::nextafter(mu, 0.0);
    q = mu * q_proposal;
    residuals = candidateResiduals(config_, field, W, q);
  }
  if (!residuals.allCandidateConstraintsNonnegative()) {
    out.certified_safe = true;
    out.status = Status::kCertifiedBrake;
    out.reason = "ray search numerical failure; certified world braking applied";
    out.residuals = zero_residuals;
    return out;
  }

  out.q = q;
  out.mu = mu;
  out.certified_safe = true;
  out.backup_used = true;
  out.status = mu == 0.0 ? Status::kCertifiedBrake
                         : Status::kCertifiedFiltered;
  out.reason = mu == 0.0 ? "certified world braking applied"
                         : "proposal filtered along the certified brake ray";
  out.residuals = residuals;
  return out;
}

}  // namespace recursive_safety_supervisor
}  // namespace coni_mpc
