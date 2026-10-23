#pragma once
#include "coni_mpc/causal_ni_uncertainty.h"
#include "coni_mpc/field_derivative_cert.h"
#include "coni_mpc/recursive_safety_supervisor.h"
#include <ros/ros.h>
#include <fstream>

namespace coni_mpc {
// Runtime adapter; certification metadata is never inferred from DSP occupancy.
class RobustNiRuntime {
 public:
  void initialize(const ros::NodeHandle&, const field_hocbf::Config&);
  bool enabled() const { return enabled_; }
  void recordNominalFrame(int stage,const Eigen::Vector3d& G,
      const Eigen::Vector3d& Vg,const Eigen::Matrix3d& R,double omega);
  Eigen::Vector3d nominalOffsets(int stage, const Eigen::Vector3d& G,
      const Eigen::Vector3d& Vg, const Eigen::Matrix3d& R, double omega,
      const Eigen::Vector3d& p, const Eigen::Vector3d& v,
      const Eigen::Vector3d& u) const;
  void prepare(const causal_ni::PredictionSequence&, const causal_ni::InitialState&,
               const Eigen::MatrixXd& inputs);
  double tighten(int stage, const field_derivative_cert::FieldSnapshot&,
                 const Eigen::Vector3d& P, const Eigen::Vector3d& W,
                 const Eigen::Vector3d& q, double position_offset=0,
                 double velocity_offset=0, double command_offset=0);
  void checkPlan(const Eigen::MatrixXd& states, const Eigen::MatrixXd& inputs,
                 const std::array<field_derivative_cert::FieldSnapshot,21>&);
  Eigen::Vector3d supervise(const Eigen::Vector3d& proposal,
      const Eigen::Vector3d& P, const Eigen::Vector3d& W, double dt,
      double stamp, bool solve_ok, bool fallback, double clearance);
 private:
  bool enabled_=false, plant_valid_=false, plan_valid_=false;
  causal_ni::PredictorBounds bounds_;
  causal_ni::PredictionSequence sequence_{};
  causal_ni::InitialState initial_;
  causal_ni::TubeConfig tube_config_;
  causal_ni::TubeResult tube_;
  recursive_safety_supervisor::Config supervisor_config_;
  recursive_safety_supervisor::Supervisor supervisor_;
  field_derivative_cert::FieldSnapshot fixed_field_;
  std::array<double,21> rho_{}, exact_margin_{};
  std::array<Eigen::Vector3d,21> frame_G_,frame_V_;
  std::array<Eigen::Matrix3d,21> frame_R_;
  std::array<double,21> frame_omega_{};
  std::array<bool,21> frame_valid_{};
  std::ofstream csv_;
};
}
