#pragma once

#include "coni_mpc/field_hocbf.h"

#include <Eigen/Geometry>

#include <cstdint>
#include <cstring>
#include <istream>
#include <limits>
#include <ostream>
#include <string>
#include <vector>

namespace coni_mpc {
namespace field_replay {

// The file is an append-only stream. All scalars are written individually;
// no Eigen object, compiler padding, or platform-dependent size_t is stored.
static constexpr std::uint64_t kFileMagic = 0x3147504c52464843ULL;  // CHFRLPG1
static constexpr std::uint32_t kVersion = 1;
enum class RecordType : std::uint32_t { kSnapshot = 1, kCycle = 2 };

struct SnapshotRecord {
  int quad_id = -1;
  std::uint64_t snapshot_id = 0;
  double stamp_sec = 0.0;  // ROS time carried by the PointCloud2 header.
  std::string frame_id;
  std::uint32_t last_occupied_stage = 0;
  field_hocbf::Points points;
};

struct StageRecord {
  std::uint32_t stage = 0;
  std::uint32_t source_stage = 0;
  double extrapolation_time = 0.0;
  Eigen::Vector3d car_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d car_velocity = Eigen::Vector3d::Zero();
  Eigen::Quaterniond world_q_non = Eigen::Quaterniond::Identity();
  Eigen::Vector3d omega_non = Eigen::Vector3d::Zero();
  Eigen::Vector3d beta_non = Eigen::Vector3d::Zero();
  Eigen::Vector3d a_car_non = Eigen::Vector3d::Zero();
  Eigen::Vector3d nominal_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d nominal_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d lambda = Eigen::Vector3d::Zero();
  field_hocbf::Config config;
  std::vector<std::uint64_t> selected_source_ids;
  field_hocbf::Constraint expected;
  double profile_b = 0.0;
};

struct CycleRecord {
  int quad_id = -1;
  std::uint64_t snapshot_id = 0;
  std::uint64_t step_idx = 0;
  double sim_time = 0.0;       // Caller clock, normally simulation time.
  double write_ros_time = 0.0; // ros::Time::now() at serialization.
  double field_age = 0.0;
  double prediction_dt = 0.0;
  double field_stamp = 0.0;
  double quad_stamp = 0.0;
  double car_stamp = 0.0;
  bool inertial_frame = false;
  bool field_available = false;
  Eigen::Vector3d estimate_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimate_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d truth_quad_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d truth_quad_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d truth_car_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d truth_car_velocity = Eigen::Vector3d::Zero();
  Eigen::Quaterniond truth_world_q_non = Eigen::Quaterniond::Identity();
  Eigen::Vector3d truth_omega_non = Eigen::Vector3d::Zero();
  std::vector<StageRecord> stages;
};

namespace detail {
template <typename T>
inline bool scalar(std::ostream& out, const T& value) {
  out.write(reinterpret_cast<const char*>(&value), sizeof(T));
  return static_cast<bool>(out);
}
template <typename T>
inline bool scalar(std::istream& in, T& value) {
  in.read(reinterpret_cast<char*>(&value), sizeof(T));
  return static_cast<bool>(in);
}
inline bool boolean(std::ostream& out, bool value) {
  const std::uint8_t v = value ? 1 : 0; return scalar(out, v);
}
inline bool boolean(std::istream& in, bool& value) {
  std::uint8_t v = 0; if (!scalar(in, v) || v > 1) return false;
  value = v != 0; return true;
}
inline bool string(std::ostream& out, const std::string& value) {
  const std::uint64_t n = value.size();
  return scalar(out, n) && (out.write(value.data(), n), static_cast<bool>(out));
}
inline bool string(std::istream& in, std::string& value) {
  std::uint64_t n = 0; if (!scalar(in, n) || n > (1ULL << 30)) return false;
  value.resize(static_cast<std::size_t>(n));
  return n == 0 || (in.read(&value[0], n), static_cast<bool>(in));
}
inline bool vec3(std::ostream& out, const Eigen::Vector3d& v) {
  return scalar(out, v.x()) && scalar(out, v.y()) && scalar(out, v.z());
}
inline bool vec3(std::istream& in, Eigen::Vector3d& v) {
  return scalar(in, v.x()) && scalar(in, v.y()) && scalar(in, v.z());
}
inline bool quat(std::ostream& out, const Eigen::Quaterniond& q) {
  return scalar(out, q.w()) && scalar(out, q.x()) && scalar(out, q.y()) && scalar(out, q.z());
}
inline bool quat(std::istream& in, Eigen::Quaterniond& q) {
  double w, x, y, z;
  if (!scalar(in, w) || !scalar(in, x) || !scalar(in, y) || !scalar(in, z)) return false;
  q = Eigen::Quaterniond(w, x, y, z); return true;
}
inline bool config(std::ostream& out, const field_hocbf::Config& c) {
  return scalar(out,c.sigma)&&scalar(out,c.d_safe)&&scalar(out,c.gamma1)&&scalar(out,c.gamma2)&&
      scalar(out,c.prune_radius)&&scalar(out,c.max_vertical_delta)&&scalar(out,c.min_occupancy)&&
      scalar(out,static_cast<std::uint64_t>(c.seed_count))&&
      scalar(out,static_cast<std::uint64_t>(c.max_count))&&scalar(out,c.separation);
}
inline bool config(std::istream& in, field_hocbf::Config& c) {
  std::uint64_t seed=0,max=0;
  if (!scalar(in,c.sigma)||!scalar(in,c.d_safe)||!scalar(in,c.gamma1)||!scalar(in,c.gamma2)||
      !scalar(in,c.prune_radius)||!scalar(in,c.max_vertical_delta)||!scalar(in,c.min_occupancy)||
      !scalar(in,seed)||!scalar(in,max)||!scalar(in,c.separation)) return false;
  c.seed_count=static_cast<std::size_t>(seed); c.max_count=static_cast<std::size_t>(max); return true;
}
inline bool constraint(std::ostream& out, const field_hocbf::Constraint& c) {
  return vec3(out,c.A)&&scalar(out,c.b)&&scalar(out,c.max_occupancy)&&scalar(out,c.h)&&
      scalar(out,c.hdot)&&scalar(out,c.lf2)&&scalar(out,c.d_min)&&scalar(out,c.d_softmin)&&boolean(out,c.active);
}
inline bool constraint(std::istream& in, field_hocbf::Constraint& c) {
  return vec3(in,c.A)&&scalar(in,c.b)&&scalar(in,c.max_occupancy)&&scalar(in,c.h)&&
      scalar(in,c.hdot)&&scalar(in,c.lf2)&&scalar(in,c.d_min)&&scalar(in,c.d_softmin)&&boolean(in,c.active);
}
inline bool header(std::ostream& out, RecordType type) {
  return scalar(out,kFileMagic)&&scalar(out,kVersion)&&scalar(out,static_cast<std::uint32_t>(type));
}
}  // namespace detail

inline bool writeSnapshot(std::ostream& out, const SnapshotRecord& r) {
  if (!detail::header(out,RecordType::kSnapshot)||!detail::scalar(out,static_cast<std::int32_t>(r.quad_id))||
      !detail::scalar(out,r.snapshot_id)||!detail::scalar(out,r.stamp_sec)||!detail::string(out,r.frame_id)||
      !detail::scalar(out,r.last_occupied_stage)||!detail::scalar(out,static_cast<std::uint64_t>(r.points.size()))) return false;
  for (const auto& p:r.points) if (!detail::vec3(out,p.position_world)||!detail::vec3(out,p.velocity_world)||
      !detail::scalar(out,p.occupancy)||!detail::scalar(out,p.prediction_time)||!detail::scalar(out,p.stage_index)||
      !detail::scalar(out,p.source_id)) return false;
  return static_cast<bool>(out);
}

inline bool writeCycle(std::ostream& out, const CycleRecord& r) {
  if (!detail::header(out,RecordType::kCycle)||!detail::scalar(out,static_cast<std::int32_t>(r.quad_id))||
      !detail::scalar(out,r.snapshot_id)||!detail::scalar(out,r.step_idx)||!detail::scalar(out,r.sim_time)||
      !detail::scalar(out,r.write_ros_time)||!detail::scalar(out,r.field_age)||!detail::scalar(out,r.prediction_dt)||
      !detail::scalar(out,r.field_stamp)||!detail::scalar(out,r.quad_stamp)||!detail::scalar(out,r.car_stamp)||
      !detail::boolean(out,r.inertial_frame)||!detail::boolean(out,r.field_available)||
      !detail::vec3(out,r.estimate_position)||!detail::vec3(out,r.estimate_velocity)||
      !detail::vec3(out,r.truth_quad_position)||!detail::vec3(out,r.truth_quad_velocity)||
      !detail::vec3(out,r.truth_car_position)||!detail::vec3(out,r.truth_car_velocity)||
      !detail::quat(out,r.truth_world_q_non)||!detail::vec3(out,r.truth_omega_non)||
      !detail::scalar(out,static_cast<std::uint64_t>(r.stages.size()))) return false;
  for (const auto& s:r.stages) {
    if (!detail::scalar(out,s.stage)||!detail::scalar(out,s.source_stage)||!detail::scalar(out,s.extrapolation_time)||
        !detail::vec3(out,s.car_position)||!detail::vec3(out,s.car_velocity)||!detail::quat(out,s.world_q_non)||
        !detail::vec3(out,s.omega_non)||!detail::vec3(out,s.beta_non)||!detail::vec3(out,s.a_car_non)||
        !detail::vec3(out,s.nominal_position)||!detail::vec3(out,s.nominal_velocity)||!detail::vec3(out,s.lambda)||
        !detail::config(out,s.config)||!detail::scalar(out,static_cast<std::uint64_t>(s.selected_source_ids.size()))) return false;
    for (auto id:s.selected_source_ids) if (!detail::scalar(out,id)) return false;
    if (!detail::constraint(out,s.expected)||!detail::scalar(out,s.profile_b)) return false;
  }
  return static_cast<bool>(out);
}

inline bool readNext(std::istream& in, RecordType& type, SnapshotRecord& snapshot, CycleRecord& cycle) {
  std::uint64_t magic=0; std::uint32_t version=0, raw_type=0;
  if (!detail::scalar(in,magic)) return false;
  if (!detail::scalar(in,version)||!detail::scalar(in,raw_type)||magic!=kFileMagic||version!=kVersion) { in.setstate(std::ios::failbit); return false; }
  type=static_cast<RecordType>(raw_type); std::int32_t qid=-1; std::uint64_t n=0;
  if (type==RecordType::kSnapshot) {
    if (!detail::scalar(in,qid)||!detail::scalar(in,snapshot.snapshot_id)||!detail::scalar(in,snapshot.stamp_sec)||
        !detail::string(in,snapshot.frame_id)||!detail::scalar(in,snapshot.last_occupied_stage)||!detail::scalar(in,n)||n>(1ULL<<32)) return false;
    snapshot.quad_id=qid; snapshot.points.clear(); snapshot.points.reserve(static_cast<std::size_t>(n));
    for(std::uint64_t i=0;i<n;++i){
      field_hocbf::Point p;
      if(!detail::vec3(in,p.position_world)||!detail::vec3(in,p.velocity_world)||
         !detail::scalar(in,p.occupancy)||!detail::scalar(in,p.prediction_time)||
         !detail::scalar(in,p.stage_index)||!detail::scalar(in,p.source_id)) {
        return false;
      }
      snapshot.points.push_back(p);
    }
    return true;
  }
  if (type!=RecordType::kCycle) { in.setstate(std::ios::failbit); return false; }
  if (!detail::scalar(in,qid)||!detail::scalar(in,cycle.snapshot_id)||!detail::scalar(in,cycle.step_idx)||!detail::scalar(in,cycle.sim_time)||
      !detail::scalar(in,cycle.write_ros_time)||!detail::scalar(in,cycle.field_age)||!detail::scalar(in,cycle.prediction_dt)||
      !detail::scalar(in,cycle.field_stamp)||!detail::scalar(in,cycle.quad_stamp)||!detail::scalar(in,cycle.car_stamp)||
      !detail::boolean(in,cycle.inertial_frame)||!detail::boolean(in,cycle.field_available)||
      !detail::vec3(in,cycle.estimate_position)||!detail::vec3(in,cycle.estimate_velocity)||
      !detail::vec3(in,cycle.truth_quad_position)||!detail::vec3(in,cycle.truth_quad_velocity)||
      !detail::vec3(in,cycle.truth_car_position)||!detail::vec3(in,cycle.truth_car_velocity)||
      !detail::quat(in,cycle.truth_world_q_non)||!detail::vec3(in,cycle.truth_omega_non)||!detail::scalar(in,n)||n>(1ULL<<20)) return false;
  cycle.quad_id=qid; cycle.stages.clear(); cycle.stages.resize(static_cast<std::size_t>(n));
  for(auto& s:cycle.stages){ std::uint64_t ids=0; if(!detail::scalar(in,s.stage)||!detail::scalar(in,s.source_stage)||!detail::scalar(in,s.extrapolation_time)||
      !detail::vec3(in,s.car_position)||!detail::vec3(in,s.car_velocity)||!detail::quat(in,s.world_q_non)||!detail::vec3(in,s.omega_non)||
      !detail::vec3(in,s.beta_non)||!detail::vec3(in,s.a_car_non)||!detail::vec3(in,s.nominal_position)||!detail::vec3(in,s.nominal_velocity)||
      !detail::vec3(in,s.lambda)||!detail::config(in,s.config)||!detail::scalar(in,ids)||ids>(1ULL<<32)) return false;
    s.selected_source_ids.resize(static_cast<std::size_t>(ids)); for(auto& id:s.selected_source_ids) if(!detail::scalar(in,id)) return false;
    if(!detail::constraint(in,s.expected)||!detail::scalar(in,s.profile_b)) return false; }
  return true;
}

inline field_hocbf::Constraint replayStage(const SnapshotRecord& snapshot, const CycleRecord& cycle,
                                            const StageRecord& stage, field_hocbf::PruneStats* stats=nullptr,
                                            const Eigen::Vector3d* nominal_position_override=nullptr,
                                            const Eigen::Vector3d* nominal_velocity_override=nullptr) {
  const Eigen::Matrix3d R_nw=stage.world_q_non.inverse().toRotationMatrix();
  field_hocbf::KinematicPoints candidates;
  for(const auto& p:snapshot.points){ if(p.stage_index!=stage.source_stage) continue;
    const Eigen::Vector3d wp=p.position_world+p.velocity_world*stage.extrapolation_time; field_hocbf::KinematicPoint t;
    t.occupancy=p.occupancy; t.source_id=p.source_id;
    if(cycle.inertial_frame){ t.position=wp; t.velocity=p.velocity_world; t.acceleration.setZero(); }
    else { t.position=R_nw*(wp-stage.car_position); t.velocity=R_nw*(p.velocity_world-stage.car_velocity)-stage.omega_non.cross(t.position);
      t.acceleration=-stage.a_car_non-2.0*stage.omega_non.cross(t.velocity)-stage.beta_non.cross(t.position)-stage.omega_non.cross(stage.omega_non.cross(t.position)); }
    candidates.push_back(t); }
  const Eigen::Vector3d& np=nominal_position_override?*nominal_position_override:stage.nominal_position;
  const Eigen::Vector3d& nv=nominal_velocity_override?*nominal_velocity_override:stage.nominal_velocity;
  const auto selected=field_hocbf::prune(candidates,np,stage.config,stats);
  Eigen::Vector3d nominal_a=Eigen::Vector3d::Zero();
  if(!cycle.inertial_frame) nominal_a=-stage.a_car_non-2.0*stage.omega_non.cross(nv)-stage.beta_non.cross(np)-stage.omega_non.cross(stage.omega_non.cross(np));
  return field_hocbf::computeConstraint(selected,np,nv,nominal_a,stage.config,stage.lambda);
}

}  // namespace field_replay
}  // namespace coni_mpc
