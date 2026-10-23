#pragma once

#include "coni_mpc/field_derivative_cert.h"

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>

namespace coni_mpc {
namespace recursive_safety_supervisor {

struct Config {
  double tau = 0.2;
  double dt = 0.02;
  double v_max_xy = 2.0;
  double v_max_z = 1.0;
  double a_max_xy = 4.0;
  double a_max_z = 3.0;
  double eta = 0.0;
  int iterations = 30;
};

enum class Status {
  kCertifiedProposal,
  kCertifiedFiltered,
  kCertifiedBrake,
  kUncertifiedMissingCertificate,
  kUncertifiedSnapshotMismatch,
  kUncertifiedInvalidField,
  kUncertifiedOutsideInvariantSet,
  kInvalidInput
};

const char* statusName(Status status);

struct Residuals {
  double invariant = std::numeric_limits<double>::quiet_NaN();
  double current_velocity_xy = std::numeric_limits<double>::quiet_NaN();
  double current_velocity_z = std::numeric_limits<double>::quiet_NaN();
  double transition = std::numeric_limits<double>::quiet_NaN();
  double intersample = std::numeric_limits<double>::quiet_NaN();
  double command_xy = std::numeric_limits<double>::quiet_NaN();
  double command_z = std::numeric_limits<double>::quiet_NaN();
  double acceleration_xy = std::numeric_limits<double>::quiet_NaN();
  double acceleration_z = std::numeric_limits<double>::quiet_NaN();
  double next_braking_velocity_xy = std::numeric_limits<double>::quiet_NaN();
  double next_braking_velocity_z = std::numeric_limits<double>::quiet_NaN();

  bool allCandidateConstraintsNonnegative() const;
};

struct Output {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d q = Eigen::Vector3d::Zero();
  double mu = 0.0;
  double h = std::numeric_limits<double>::quiet_NaN();
  double braking_reserve = std::numeric_limits<double>::quiet_NaN();
  bool certified_safe = false;
  bool backup_used = true;
  Status status = Status::kInvalidInput;
  std::string reason;
  Residuals residuals;
};

class Supervisor {
 public:
  explicit Supervisor(const Config& config = Config());

  // Freezing is an explicit external decision.  run() never promotes a DSP or
  // other incoming snapshot to a certificate automatically.
  bool freezeCertifiedSnapshot(
      const field_derivative_cert::FieldSnapshot& snapshot,
      std::string* reason = nullptr);
  void clearFrozenSnapshot();
  bool hasFrozenSnapshot() const { return frozen_; }
  std::uint64_t frozenSnapshotId() const { return frozen_snapshot_id_; }

  Output run(const field_derivative_cert::FieldSnapshot& snapshot,
             const Eigen::Vector3d& P, const Eigen::Vector3d& W,
             const Eigen::Vector3d& q_proposal) const;

 private:
  Config config_;
  bool frozen_ = false;
  std::uint64_t frozen_snapshot_id_ = 0;
  field_derivative_cert::FieldSnapshot frozen_snapshot_;
};

}  // namespace recursive_safety_supervisor
}  // namespace coni_mpc
