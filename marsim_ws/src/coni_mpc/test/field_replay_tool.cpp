#include "coni_mpc/field_replay_diagnostics.h"

#include <cmath>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <utility>

namespace fr = coni_mpc::field_replay;
namespace fh = coni_mpc::field_hocbf;

namespace {
using Key = std::pair<int, std::uint64_t>;

double error(double a, double b) {
  if (std::isnan(a) && std::isnan(b)) return 0.0;
  if (std::isnan(a) || std::isnan(b)) return std::numeric_limits<double>::infinity();
  return std::abs(a - b);
}

double maxConstraintError(const fh::Constraint& a, const fh::Constraint& b) {
  return std::max({(a.A-b.A).cwiseAbs().maxCoeff(), error(a.b,b.b),
                   error(a.h,b.h), error(a.hdot,b.hdot), error(a.lf2,b.lf2),
                   error(a.d_min,b.d_min), error(a.d_softmin,b.d_softmin),
                   error(a.max_occupancy,b.max_occupancy),
                   a.active==b.active ? 0.0 : 1.0});
}

std::string ids(const std::vector<std::uint64_t>& values) {
  std::ostringstream out;
  for (std::size_t i=0;i<values.size();++i) { if(i) out << ';'; out << values[i]; }
  return out.str();
}

void result(std::ostream& out, const fh::Constraint& c) {
  out << ',' << c.active << ',' << c.h << ',' << c.hdot << ',' << c.lf2
      << ',' << c.A.x() << ',' << c.A.y() << ',' << c.A.z() << ',' << c.b;
}

int selfTest() {
  fr::SnapshotRecord source;
  source.quad_id=2; source.snapshot_id=7; source.stamp_sec=12.5; source.frame_id="map";
  fh::Point point; point.position_world=Eigen::Vector3d(1.0,2.0,0.4);
  point.velocity_world=Eigen::Vector3d(0.1,-0.2,0.0); point.occupancy=0.8;
  point.stage_index=0; point.source_id=42; source.points.push_back(point);
  fr::CycleRecord cycle; cycle.quad_id=2; cycle.snapshot_id=7; cycle.field_available=true;
  cycle.inertial_frame=true;
  fr::StageRecord stage; stage.nominal_position=Eigen::Vector3d(1.4,2.1,0.4);
  stage.nominal_velocity=Eigen::Vector3d(0.2,0.0,0.0); stage.lambda=Eigen::Vector3d::Constant(5.0);
  stage.config.prune_radius=2.0; stage.config.max_vertical_delta=1.0;
  fh::PruneStats expected_stats; stage.expected=fr::replayStage(source,cycle,stage,&expected_stats);
  stage.selected_source_ids=expected_stats.selected_source_ids;
  const Eigen::Vector3d kv=(stage.config.gamma1+stage.config.gamma2)*stage.lambda.cwiseInverse()-Eigen::Vector3d::Ones();
  const Eigen::Vector3d kp=(stage.config.gamma1*stage.config.gamma2)*stage.lambda.cwiseInverse();
  stage.profile_b=stage.expected.b+stage.expected.A.dot(kv.cwiseProduct(stage.nominal_velocity)+kp.cwiseProduct(stage.nominal_position));
  cycle.stages.push_back(stage);
  std::stringstream stream(std::ios::in|std::ios::out|std::ios::binary);
  if(!fr::writeSnapshot(stream,source)||!fr::writeCycle(stream,cycle)) return 10;
  stream.seekg(0); fr::RecordType type; fr::SnapshotRecord loaded; fr::CycleRecord loaded_cycle;
  if(!fr::readNext(stream,type,loaded,loaded_cycle)||type!=fr::RecordType::kSnapshot) return 11;
  if(!fr::readNext(stream,type,source,loaded_cycle)||type!=fr::RecordType::kCycle) return 12;
  fh::PruneStats actual_stats;
  const fh::Constraint actual=fr::replayStage(loaded,loaded_cycle,loaded_cycle.stages.front(),&actual_stats);
  if(maxConstraintError(actual,loaded_cycle.stages.front().expected)>1e-14 ||
     actual_stats.selected_source_ids!=loaded_cycle.stages.front().selected_source_ids) return 13;
  std::cout << "field replay serialization/reconstruction self-test passed\n";
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  if(argc==2 && std::string(argv[1])=="--self-test") return selfTest();
  if (argc != 3) {
    std::cerr << "usage: field_replay_tool INPUT.replay.bin OUTPUT.csv\n";
    return 2;
  }
  std::ifstream input(argv[1], std::ios::binary);
  std::ofstream output(argv[2]);
  if (!input || !output) { std::cerr << "failed to open input or output\n"; return 2; }
  output << std::setprecision(17);
  output << "quad_id,step_idx,sim_time,write_ros_time,quad_stamp,car_stamp,field_available,"
            "snapshot_id,field_stamp,field_age,prediction_dt,stage,source_stage,extrapolation_time,"
            "nominal_px,nominal_py,nominal_pz,nominal_vx,nominal_vy,nominal_vz,selected_ids,"
            "ids_match,max_replay_error,profile_b_error,original_active,original_h,original_hdot,original_lf2,"
            "original_Ax,original_Ay,original_Az,original_b,current_est_active,current_est_h,current_est_hdot,"
            "current_est_lf2,current_est_Ax,current_est_Ay,current_est_Az,current_est_b,current_truth_active,"
            "current_truth_h,current_truth_hdot,current_truth_lf2,current_truth_Ax,current_truth_Ay,"
            "current_truth_Az,current_truth_b,uav_truth_fixed_car_active,uav_truth_fixed_car_h,"
            "uav_truth_fixed_car_hdot,uav_truth_fixed_car_lf2,uav_truth_fixed_car_Ax,uav_truth_fixed_car_Ay,"
            "uav_truth_fixed_car_Az,uav_truth_fixed_car_b,truth_quad_px,truth_quad_py,truth_quad_pz,"
            "truth_quad_vx,truth_quad_vy,truth_quad_vz,truth_car_px,truth_car_py,truth_car_pz,"
            "truth_car_vx,truth_car_vy,truth_car_vz,old_field_available,old_field_active,old_field_h,"
            "old_field_hdot,old_field_lf2,old_field_Ax,old_field_Ay,old_field_Az,old_field_b\n";

  std::map<Key, fr::SnapshotRecord> snapshots;
  std::map<int, std::uint64_t> previous_snapshot;
  std::size_t cycles=0, stages=0, failures=0, unavailable=0;
  while (true) {
    if(input.peek()==std::char_traits<char>::eof()) break;
    fr::RecordType type; fr::SnapshotRecord snapshot; fr::CycleRecord cycle;
    if (!fr::readNext(input,type,snapshot,cycle)) {
      if (input.eof()) break;
      std::cerr << "invalid/truncated replay stream after " << cycles << " cycles\n";
      return 3;
    }
    if (type==fr::RecordType::kSnapshot) {
      snapshots[Key(snapshot.quad_id,snapshot.snapshot_id)]=std::move(snapshot);
      continue;
    }
    ++cycles;
    if (!cycle.field_available || cycle.stages.empty()) {
      ++unavailable;
      output << cycle.quad_id << ',' << cycle.step_idx << ',' << cycle.sim_time << ','
             << cycle.write_ros_time << ',' << cycle.quad_stamp << ',' << cycle.car_stamp << ','
             << cycle.field_available << ',' << cycle.snapshot_id << ',' << cycle.field_stamp << ','
             << cycle.field_age << ',' << cycle.prediction_dt
             << ",-1,-1,nan,nan,nan,nan,nan,nan,nan,,1,0,0,0,nan,nan,nan,0,0,0,0,"
                "0,nan,nan,nan,0,0,0,0,0,nan,nan,nan,0,0,0,0,0,nan,nan,nan,0,0,0,0,"
                "nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,0,0,nan,nan,nan,0,0,0,0\n";
      continue;
    }
    const auto sit=snapshots.find(Key(cycle.quad_id,cycle.snapshot_id));
    if (sit==snapshots.end()) { std::cerr << "cycle references missing snapshot\n"; return 3; }
    const fr::SnapshotRecord& current=sit->second;
    const auto old_id_it=previous_snapshot.find(cycle.quad_id);
    const fr::SnapshotRecord* old=nullptr;
    if(old_id_it!=previous_snapshot.end()) {
      const auto oi=snapshots.find(Key(cycle.quad_id,old_id_it->second)); if(oi!=snapshots.end()) old=&oi->second;
    }
    for(const auto& stage:cycle.stages) {
      ++stages;
      fh::PruneStats stats;
      const fh::Constraint replay=fr::replayStage(current,cycle,stage,&stats);
      const double replay_error=maxConstraintError(replay,stage.expected);
      const bool ids_match=stats.selected_source_ids==stage.selected_source_ids;
      double profile_b=0.0;
      if(replay.active) {
        const Eigen::Vector3d kv=(stage.config.gamma1+stage.config.gamma2)*stage.lambda.cwiseInverse()-Eigen::Vector3d::Ones();
        const Eigen::Vector3d kp=(stage.config.gamma1*stage.config.gamma2)*stage.lambda.cwiseInverse();
        profile_b=replay.b+replay.A.dot(kv.cwiseProduct(stage.nominal_velocity)+kp.cwiseProduct(stage.nominal_position));
      }
      const double profile_error=error(profile_b,stage.profile_b);
      if(!ids_match || replay_error>1e-12 || profile_error>1e-12) ++failures;

      fh::Constraint current_est, current_truth, uav_truth_fixed_car, old_field;
      bool has_current_cf=false, has_truth_cf=false, has_fixed_car_cf=false, has_old=false;
      if(stage.stage==0) {
        has_current_cf=true;
        current_est=fr::replayStage(current,cycle,stage,nullptr,&cycle.estimate_position,&cycle.estimate_velocity);
        fr::StageRecord truth_stage=stage;
        truth_stage.car_position=cycle.truth_car_position;
        truth_stage.car_velocity=cycle.truth_car_velocity;
        truth_stage.world_q_non=cycle.truth_world_q_non;
        truth_stage.omega_non=cycle.truth_omega_non;
        Eigen::Vector3d truth_p=cycle.truth_quad_position;
        Eigen::Vector3d truth_v=cycle.truth_quad_velocity;
        if(!cycle.inertial_frame) {
          const Eigen::Matrix3d R_nw=cycle.truth_world_q_non.inverse().toRotationMatrix();
          truth_p=R_nw*(cycle.truth_quad_position-cycle.truth_car_position);
          truth_v=-cycle.truth_omega_non.cross(truth_p)+R_nw*(cycle.truth_quad_velocity-cycle.truth_car_velocity);
        }
        has_truth_cf=true;
        current_truth=fr::replayStage(current,cycle,truth_stage,nullptr,&truth_p,&truth_v);
        Eigen::Vector3d fixed_p=cycle.truth_quad_position;
        Eigen::Vector3d fixed_v=cycle.truth_quad_velocity;
        if(!cycle.inertial_frame) {
          const Eigen::Matrix3d R_nw=stage.world_q_non.inverse().toRotationMatrix();
          fixed_p=R_nw*(cycle.truth_quad_position-stage.car_position);
          fixed_v=-stage.omega_non.cross(fixed_p)+R_nw*(cycle.truth_quad_velocity-stage.car_velocity);
        }
        has_fixed_car_cf=true;
        uav_truth_fixed_car=fr::replayStage(current,cycle,stage,nullptr,&fixed_p,&fixed_v);
        if(old) {
          has_old=true;
          fr::StageRecord old_stage=stage;
          const double query_time=cycle.field_stamp+cycle.field_age;
          const double old_age=std::max(0.0,query_time-old->stamp_sec);
          const int old_horizon=old_age>cycle.prediction_dt*static_cast<double>(cycle.stages.size()-1)
              ? static_cast<int>(old->last_occupied_stage)
              : static_cast<int>(cycle.stages.size()-1);
          const int old_shift=std::max(0,static_cast<int>(std::min(
              static_cast<double>(old_horizon),std::floor(old_age/cycle.prediction_dt+0.5))));
          old_stage.source_stage=static_cast<std::uint32_t>(std::min(
              old_horizon,static_cast<int>(stage.stage)+old_shift));
          old_stage.extrapolation_time=old_age+
              (static_cast<double>(stage.stage)-old_stage.source_stage)*cycle.prediction_dt;
          old_field=fr::replayStage(*old,cycle,old_stage);
        }
      }
      output << cycle.quad_id << ',' << cycle.step_idx << ',' << cycle.sim_time << ',' << cycle.write_ros_time
             << ',' << cycle.quad_stamp << ',' << cycle.car_stamp << ',' << cycle.field_available << ','
             << cycle.snapshot_id << ',' << cycle.field_stamp << ',' << cycle.field_age << ',' << cycle.prediction_dt
             << ',' << stage.stage << ',' << stage.source_stage << ',' << stage.extrapolation_time
             << ',' << stage.nominal_position.x() << ',' << stage.nominal_position.y() << ',' << stage.nominal_position.z()
             << ',' << stage.nominal_velocity.x() << ',' << stage.nominal_velocity.y() << ',' << stage.nominal_velocity.z()
             << ',' << ids(stage.selected_source_ids) << ',' << ids_match << ',' << replay_error << ',' << profile_error;
      result(output,stage.expected);
      if(has_current_cf) result(output,current_est); else output << ",0,nan,nan,nan,0,0,0,0";
      if(has_truth_cf) result(output,current_truth); else output << ",0,nan,nan,nan,0,0,0,0";
      if(has_fixed_car_cf) result(output,uav_truth_fixed_car); else output << ",0,nan,nan,nan,0,0,0,0";
      output << ',' << cycle.truth_quad_position.x() << ',' << cycle.truth_quad_position.y() << ',' << cycle.truth_quad_position.z()
             << ',' << cycle.truth_quad_velocity.x() << ',' << cycle.truth_quad_velocity.y() << ',' << cycle.truth_quad_velocity.z()
             << ',' << cycle.truth_car_position.x() << ',' << cycle.truth_car_position.y() << ',' << cycle.truth_car_position.z()
             << ',' << cycle.truth_car_velocity.x() << ',' << cycle.truth_car_velocity.y() << ',' << cycle.truth_car_velocity.z()
             << ',' << has_old;
      if(has_old) result(output,old_field); else output << ",0,nan,nan,nan,0,0,0,0";
      output << '\n';
    }
    previous_snapshot[cycle.quad_id]=cycle.snapshot_id;
  }
  if(!output) { std::cerr << "CSV write failed\n"; return 4; }
  std::cerr << "cycles=" << cycles << " unavailable=" << unavailable << " stages=" << stages
            << " verification_failures=" << failures << '\n';
  return failures==0 ? 0 : 5;
}
