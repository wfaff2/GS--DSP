#include "coni_mpc/field_replay_diagnostics.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>

namespace fr = coni_mpc::field_replay;
namespace fh = coni_mpc::field_hocbf;
using Key = std::pair<int, std::uint64_t>;

fh::KinematicPoints transformed(const fr::SnapshotRecord& snapshot,
                               const fr::CycleRecord& cycle,
                               const fr::StageRecord& stage) {
  const Eigen::Matrix3d R = stage.world_q_non.inverse().toRotationMatrix();
  fh::KinematicPoints points;
  for (const auto& p : snapshot.points) {
    if (p.stage_index != stage.source_stage) continue;
    fh::KinematicPoint t;
    const Eigen::Vector3d wp = p.position_world + p.velocity_world * stage.extrapolation_time;
    t.source_id = p.source_id;
    t.occupancy = p.occupancy;
    if (cycle.inertial_frame) {
      t.position = wp; t.velocity = p.velocity_world;
    } else {
      t.position = R * (wp - stage.car_position);
      t.velocity = R * (p.velocity_world - stage.car_velocity) - stage.omega_non.cross(t.position);
      t.acceleration = -stage.a_car_non - 2. * stage.omega_non.cross(t.velocity) -
          stage.beta_non.cross(t.position) - stage.omega_non.cross(stage.omega_non.cross(t.position));
    }
    points.push_back(t);
  }
  return points;
}

fh::Constraint evaluate(const fh::KinematicPoints& points, const fr::CycleRecord& cycle,
                        const fr::StageRecord& stage) {
  Eigen::Vector3d a = Eigen::Vector3d::Zero();
  if (!cycle.inertial_frame) a = -stage.a_car_non - 2. * stage.omega_non.cross(stage.nominal_velocity) -
      stage.beta_non.cross(stage.nominal_position) -
      stage.omega_non.cross(stage.omega_non.cross(stage.nominal_position));
  return fh::computeConstraint(points, stage.nominal_position, stage.nominal_velocity,
                               a, stage.config, stage.lambda);
}

void emit(std::ostream& out, const fr::CycleRecord& cycle, const fr::StageRecord& stage,
          const char* variant, const fh::Constraint& value, std::size_t count,
          std::uint64_t snapshot_id) {
  out << cycle.quad_id << ',' << cycle.step_idx << ',' << cycle.sim_time << ',' << stage.stage
      << ',' << variant << ',' << snapshot_id << ',' << stage.source_stage << ',' << count
      << ',' << value.active << ',' << value.h << ',' << value.hdot << ',' << value.lf2
      << ',' << value.A.x() << ',' << value.A.y() << ',' << value.A.z() << ',' << value.b
      << ',' << value.d_min << '\n';
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: field_replay_event_tool INPUT.bin OUTPUT.csv STEP_IDX [STEP_IDX...]\n";
    return 2;
  }
  std::set<std::uint64_t> events;
  for (int i=3; i<argc; ++i) events.insert(std::stoull(argv[i]));
  std::ifstream in(argv[1], std::ios::binary);
  std::ofstream out(argv[2]);
  if (!in || !out) return 2;
  out << std::setprecision(17)
      << "quad_id,step_idx,sim_time,stage,variant,snapshot_id,source_stage,selected_count,active,h,hdot,lf2,Ax,Ay,Az,b,d_min\n";
  std::map<Key, fr::SnapshotRecord> snapshots;
  std::map<int, fr::CycleRecord> previous;
  std::size_t matched=0;
  while (in.peek() != std::char_traits<char>::eof()) {
    fr::RecordType type; fr::SnapshotRecord snapshot; fr::CycleRecord cycle;
    if (!fr::readNext(in, type, snapshot, cycle)) { std::cerr << "invalid replay stream\n"; return 3; }
    if (type == fr::RecordType::kSnapshot) {
      const Key key(snapshot.quad_id, snapshot.snapshot_id);
      snapshots[key] = std::move(snapshot);
      continue;
    }
    const auto old = previous.find(cycle.quad_id);
    if (events.count(cycle.step_idx) && cycle.field_available && old != previous.end() && old->second.field_available) {
      ++matched;
      const auto& current = snapshots.at(Key(cycle.quad_id, cycle.snapshot_id));
      for (const auto& stage : cycle.stages) {
        const auto& old_stage = old->second.stages.at(stage.stage);
        auto points = transformed(current, cycle, stage);
        const auto selected = fh::prune(points, stage.nominal_position, stage.config);
        const auto original = evaluate(selected, cycle, stage);
        if (std::abs(original.h-stage.expected.h)>1e-12 ||
            std::abs(original.b-stage.expected.b)>1e-12 ||
            (original.A-stage.expected.A).norm()>1e-12) {
          std::cerr << "event transform failed original replay check\n"; return 4;
        }
        emit(out, cycle, stage, "original", original, selected.size(), current.snapshot_id);
        // Keep the previous prediction slice, advance it to the current query
        // time, and hold all current query/state/transform inputs fixed.
        if (cycle.snapshot_id == old->second.snapshot_id) {
          auto held = stage;
          held.source_stage = old_stage.source_stage;
          held.extrapolation_time = cycle.field_age +
              (static_cast<double>(stage.stage)-held.source_stage)*cycle.prediction_dt;
          auto held_points = transformed(current, cycle, held);
          const auto held_selected = fh::prune(held_points, held.nominal_position, held.config);
          emit(out, cycle, held, "previous_slice_current_context",
               evaluate(held_selected, cycle, held), held_selected.size(), current.snapshot_id);
          // Exact selection identity is available only within the same
          // snapshot and prediction slice. Across frames we do not fabricate it.
          if (stage.source_stage == old_stage.source_stage) {
            std::set<std::uint64_t> old_ids(old_stage.selected_source_ids.begin(),
                                             old_stage.selected_source_ids.end());
            fh::KinematicPoints held_selection;
            for (const auto& point : points) {
              if (old_ids.count(point.source_id)) held_selection.push_back(point);
            }
            emit(out, cycle, stage, "previous_selection_current_context",
                 evaluate(held_selection, cycle, stage), held_selection.size(), current.snapshot_id);
          }
        }
      }
    }
    previous[cycle.quad_id] = cycle;
    // Only snapshots referenced by the latest cycle for each UAV are needed.
    for (auto it=snapshots.begin(); it!=snapshots.end();) {
      const auto latest=previous.find(it->first.first);
      if (latest!=previous.end() && latest->second.snapshot_id!=it->first.second) it=snapshots.erase(it);
      else ++it;
    }
  }
  std::cerr << "events_replayed=" << matched << '\n';
  return out.good() ? 0 : 5;
}
