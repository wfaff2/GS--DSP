#include "coni_mpc/robust_ni_runtime.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <iomanip>
#include <limits>
#include <algorithm>

namespace coni_mpc {
namespace {
bool readBound(const ros::NodeHandle& nh, const char* key, std::array<double,21>& a) {
  XmlRpc::XmlRpcValue v;
  if (!nh.getParam(key,v)) return false;
  auto number=[](XmlRpc::XmlRpcValue& x)->double {
    if(x.getType()==XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(x);
    if(x.getType()==XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(x);
    return std::numeric_limits<double>::quiet_NaN();
  };
  if(v.getType()==XmlRpc::XmlRpcValue::TypeArray) {
    if(v.size()!=21) return false;
    for(int i=0;i<21;++i) a[i]=number(v[i]);
  } else a.fill(number(v));
  for(double x:a) if(!std::isfinite(x)||x<0) return false;
  return true;
}
Eigen::Matrix3d rotation(double yaw) {
  return Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
}
void RobustNiRuntime::initialize(const ros::NodeHandle& nh,const field_hocbf::Config& field) {
  nh.param("robust_ni/enabled",enabled_,false);
  double tau_xy=.2,tau_z=.2;
  nh.param("tau_v_xy",tau_xy,.2);nh.param("tau_v_z",tau_z,.2);
  plant_valid_=std::isfinite(tau_xy)&&tau_xy>0&&tau_xy==tau_z;
  supervisor_config_.tau=tau_xy;
  nh.param("sim_control_dt_sec",supervisor_config_.dt,.02);
  nh.param("max_v_xy",supervisor_config_.v_max_xy,5.);
  nh.param("max_v_z",supervisor_config_.v_max_z,3.);
  nh.param("a_max_xy",supervisor_config_.a_max_xy,6.);
  nh.param("a_max_z",supervisor_config_.a_max_z,4.);
  nh.param("robust_ni/safety_reserve_eta",supervisor_config_.eta,0.);
  nh.param("robust_ni/ray_bisection_iterations",supervisor_config_.iterations,30);
  tube_config_.lambda=plant_valid_?1/tau_xy:5.;
  bounds_.valid=readBound(nh,"robust_ni/delta_a",bounds_.d_accel)&&
      readBound(nh,"robust_ni/delta_omega",bounds_.d_omega)&&
      readBound(nh,"robust_ni/delta_beta",bounds_.d_beta);
  fixed_field_.config=field;
  bool require_static=true;
  nh.param("robust_ni/require_certified_static_field",require_static,true);
  // Turning this flag off does not promote a time-varying map to a proof.
  nh.param("robust_ni/static_field/coverage_asserted",fixed_field_.coverage_certified,false);
  nh.param("robust_ni/static_field/persistent_asserted",fixed_field_.persistent_static,false);
  fixed_field_.coverage_certified= fixed_field_.coverage_certified && require_static;
  int id=0;nh.param("robust_ni/static_field/snapshot_id",id,0);fixed_field_.id=std::max(0,id);
  std::vector<double> flat;
  if(nh.getParam("robust_ni/static_field/voxels_xyzo",flat)&&flat.size()%4==0) {
    for(std::size_t i=0;i<flat.size();i+=4) {
      field_hocbf::KinematicPoint p;p.position=Eigen::Vector3d(flat[i],flat[i+1],flat[i+2]);
      p.occupancy=flat[i+3];fixed_field_.points.push_back(p);
    }
  }
  // An absent list is not a certified empty environment.
  if(flat.empty()) fixed_field_.coverage_certified=false;
  supervisor_=recursive_safety_supervisor::Supervisor(supervisor_config_);
  supervisor_.freezeCertifiedSnapshot(fixed_field_);
  std::string path;nh.param("robust_ni/debug_csv",path,std::string());
  if(enabled_&&!path.empty()) {
    csv_.open(path);csv_<<std::setprecision(17);
    csv_<<"ros_time,stage,pred_a_x,pred_a_y,pred_a_z,pred_omega,pred_beta,current_omega,r_omega,E_P,E_W,E_q,rho_ni,psi2_exact_minus_rho,bounds_valid,tube_valid,plan_exact_margin_valid,h,braking_certificate,mu,q_mpc_x,q_mpc_y,q_mpc_z,q_applied_x,q_applied_y,q_applied_z,certified_safe,backup_used,solve_ok,fallback_used,snapshot_id,geometric_clearance,physical_limits_valid,status\n";
  }
}
void RobustNiRuntime::prepare(const causal_ni::PredictionSequence& seq,
    const causal_ni::InitialState& initial,const Eigen::MatrixXd& inputs) {
  if(!enabled_)return;
  sequence_=seq;initial_=initial;rho_.fill(std::numeric_limits<double>::quiet_NaN());
  frame_valid_.fill(false);
  exact_margin_=rho_;plan_valid_=false;
  tube_=causal_ni::TubeResult();
  if(!bounds_.valid) {tube_.reason="predictor_error_bounds_unavailable";return;}
  if(!initial.G_world.allFinite()||!initial.Vg_world.allFinite()||
      !initial.P_world.allFinite()||!initial.W_world.allFinite()||
      !std::isfinite(initial.yaw)||!std::isfinite(initial.omega)||
      inputs.rows()<3||inputs.cols()<20||!inputs.allFinite()) {
    tube_.reason="invalid_nominal_state_or_control_sequence";return;
  }
  for(int j=0;j<21;++j) if(!seq.accel_n[j].allFinite()||
      !std::isfinite(seq.omega[j])||!std::isfinite(seq.beta[j])) {
    tube_.reason="invalid_causal_sequence";return;
  }
  std::array<double,20> U{},D{};
  double distance=(initial.P_world-initial.G_world).norm();
  double speed=(initial.W_world-initial.Vg_world).norm();
  for(int j=0;j<20;++j) {
    U[j]=(inputs.rows()>=3&&inputs.cols()>j&&inputs.col(j).allFinite())?
        inputs.col(j).head<3>().norm():0.;
    Eigen::Matrix3d m=Eigen::Matrix3d::Zero();m(0,1)=1.;
    m(1,0)=tube_config_.lambda*std::abs(seq.omega[j]);m(1,1)=-tube_config_.lambda;
    m(1,2)=tube_config_.lambda*U[j]+seq.accel_n[j].norm();
    const Eigen::Vector3d next=(.1*m).exp()*Eigen::Vector3d(distance,speed,1.);
    D[j]=std::max(distance,next.x());distance=next.x();speed=next.y();
  }
  tube_=causal_ni::buildTube(seq,initial,bounds_,U,D,tube_config_);
  if(!plant_valid_) {tube_.valid=false;tube_.reason="unequal_or_invalid_time_constants";}
}
void RobustNiRuntime::recordNominalFrame(int j,const Eigen::Vector3d& G,
    const Eigen::Vector3d& Vg,const Eigen::Matrix3d& R,double omega) {
  frame_G_[j]=G;frame_V_[j]=Vg;frame_R_[j]=R;frame_omega_[j]=omega;
  frame_valid_[j]=G.allFinite()&&Vg.allFinite()&&R.allFinite()&&std::isfinite(omega);
}
Eigen::Vector3d RobustNiRuntime::nominalOffsets(int j,const Eigen::Vector3d& G,
    const Eigen::Vector3d& Vg,const Eigen::Matrix3d& R,double omega,
    const Eigen::Vector3d& p,const Eigen::Vector3d& v,const Eigen::Vector3d& u) const {
  if(!tube_.valid)return Eigen::Vector3d::Zero();
  const auto& n=tube_.samples[std::min(j,20)*5];const auto Rhat=rotation(n.nominal_yaw);
  const Eigen::Vector3d spin=Eigen::Vector3d(0,0,omega).cross(p);
  const Eigen::Vector3d nominalspin=Eigen::Vector3d(0,0,sequence_.omega[j]).cross(p);
  return Eigen::Vector3d((G+R*p-n.nominal_G_world-Rhat*p).norm(),
      (Vg+R*(v+spin)-n.nominal_Vg_world-Rhat*(v+nominalspin)).norm(),
      (Vg+R*(u+spin)-n.nominal_Vg_world-Rhat*(u+nominalspin)).norm());
}
double RobustNiRuntime::tighten(int j,const field_derivative_cert::FieldSnapshot& map,
    const Eigen::Vector3d& P,const Eigen::Vector3d& W,const Eigen::Vector3d& q,
    double offsetP,double offsetW,double offsetQ) {
  if(!enabled_||!tube_.valid)return 0.;
  const auto& e=tube_.stages[std::min(j,19)];
  const double ep=e.E_P_max+offsetP,ew=e.E_W_max+offsetW,eq=e.E_q_max+offsetQ;
  const auto eval=field_derivative_cert::evaluate(map,P,ep);
  const auto margin=field_derivative_cert::robustMargin(eval,W,q,ep,ew,eq,
      tube_config_.lambda,map.config.gamma1,map.config.gamma2);
  if(!margin.valid) {rho_[j]=std::numeric_limits<double>::quiet_NaN();return 0.;}
  rho_[j]=margin.rho;
  return margin.rho;
}
void RobustNiRuntime::checkPlan(const Eigen::MatrixXd& states,const Eigen::MatrixXd& inputs,
    const std::array<field_derivative_cert::FieldSnapshot,21>& maps) {
  if(!enabled_)return;
  // Rebuild with the returned control sequence: old warm-start norms do not
  // certify a different newly optimized policy.
  const auto saved_valid=frame_valid_;
  prepare(sequence_,initial_,inputs);
  frame_valid_=saved_valid;
  plan_valid_=tube_.valid && states.rows()>=6 && states.cols()>=21 && inputs.cols()>=20;
  if(!plan_valid_)return;
  for(int j=0;j<20;++j) {
    if(!frame_valid_[j]) {plan_valid_=false;continue;}
    const auto& R=frame_R_[j];
    const Eigen::Vector3d p=states.block<3,1>(0,j),v=states.block<3,1>(3,j),u=inputs.block<3,1>(0,j);
    const Eigen::Vector3d spin=Eigen::Vector3d(0,0,frame_omega_[j]).cross(p);
    const Eigen::Vector3d P=frame_G_[j]+R*p;
    const Eigen::Vector3d W=frame_V_[j]+R*(v+spin);
    const Eigen::Vector3d q=frame_V_[j]+R*(u+spin);
    const auto offsets=nominalOffsets(j,frame_G_[j],frame_V_[j],R,frame_omega_[j],p,v,u);
    const double margin=tighten(j,maps[j],P,W,q,offsets.x(),offsets.y(),offsets.z());
    const auto ev=field_derivative_cert::evaluate(maps[j],P,0.);
    exact_margin_[j]=field_derivative_cert::exactPsi2(ev,W,q,tube_config_.lambda,
        maps[j].config.gamma1,maps[j].config.gamma2)-margin;
    // DSP stages are performance observations, never persistent certificates.
    plan_valid_=plan_valid_&&offsets.isZero(1e-12)&&maps[j].persistent_static&&maps[j].coverage_certified&&
        ev.valid&&std::isfinite(rho_[j])&&std::isfinite(exact_margin_[j])&&exact_margin_[j]>=0.;
  }
}
Eigen::Vector3d RobustNiRuntime::supervise(const Eigen::Vector3d& proposal,
    const Eigen::Vector3d& P,const Eigen::Vector3d& W,double dt,double stamp,
    bool solve_ok,bool fallback,double clearance) {
  if(!enabled_)return proposal;
  auto output=supervisor_.run(fixed_field_,P,W,proposal);
  if(!plant_valid_||std::abs(dt-supervisor_config_.dt)>1e-10) {
    output.q.setZero();output.mu=0;output.certified_safe=false;output.backup_used=true;
    output.reason="plant_or_execution_interval_mismatch";
  }
  if(csv_)for(int j=0;j<20;++j) {
    csv_<<stamp<<','<<j<<','<<sequence_.accel_n[j].x()<<','<<sequence_.accel_n[j].y()<<','<<sequence_.accel_n[j].z()<<','<<sequence_.omega[j]<<','<<sequence_.beta[j]<<','<<initial_.omega<<',';
    const auto& s=tube_.samples[j*5];const auto& e=tube_.stages[j];
    csv_<<s.r_omega<<','<<e.E_P_max<<','<<e.E_W_max<<','<<e.E_q_max<<','<<rho_[j]<<','<<exact_margin_[j]<<','<<bounds_.valid<<','<<tube_.valid<<','<<plan_valid_<<','<<output.h<<','<<output.braking_reserve<<','<<output.mu<<',';
    csv_<<proposal.x()<<','<<proposal.y()<<','<<proposal.z()<<','<<output.q.x()<<','<<output.q.y()<<','<<output.q.z()<<','<<output.certified_safe<<','<<output.backup_used<<','<<solve_ok<<','<<fallback<<','<<fixed_field_.id<<','<<clearance<<','<<output.residuals.allCandidateConstraintsNonnegative()<<','<<output.reason<<'\n';
  }
  ROS_INFO_STREAM_THROTTLE(1.,"[ROBUST_NI] certified_safe="<<output.certified_safe<<" bounds_valid="<<bounds_.valid<<" tube_valid="<<tube_.valid<<" reason="<<output.reason);
  return output.q;
}
}
