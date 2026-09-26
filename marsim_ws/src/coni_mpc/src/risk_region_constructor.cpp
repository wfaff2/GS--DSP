#include "coni_mpc/risk_region_constructor.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace coni_mpc {
namespace {

struct Cell {
  int x = 0;
  int y = 0;
  int z = 0;
  bool operator==(const Cell& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
  bool operator<(const Cell& other) const {
    return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
  }
};

struct CellHash {
  std::size_t operator()(const Cell& c) const {
    const std::size_t h1 = std::hash<int>()(c.x);
    const std::size_t h2 = std::hash<int>()(c.y);
    const std::size_t h3 = std::hash<int>()(c.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

struct StageCandidate {
  std::uint32_t stage = 0;
  double time = 0.0;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d semi_axes = Eigen::Vector3d::Ones();
  double mass = 0.0;
  double max_probability = 0.0;
  std::uint32_t track = 0;
};

Cell toCell(const Eigen::Vector3d& p, double resolution) {
  return Cell{static_cast<int>(std::floor(p.x() / resolution + 0.5)),
              static_cast<int>(std::floor(p.y() / resolution + 0.5)),
              static_cast<int>(std::floor(p.z() / resolution + 0.5))};
}

std::vector<Cell> neighbors26(const Cell& cell) {
  std::vector<Cell> result;
  result.reserve(26);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx != 0 || dy != 0 || dz != 0) {
          result.push_back(Cell{cell.x + dx, cell.y + dy, cell.z + dz});
        }
      }
    }
  }
  return result;
}

// Fit the fixed-center/fixed-orientation p=4 superellipsoid used by the
// document's minimum-volume step.  In the PCA frame the constraints are
//
//   A*x^4 + B*y^4 + C*z^4 <= 1,  A,B,C > 0,
//
// and minimizing volume is equivalent to maximizing log(A)+log(B)+log(C).
// The objective is monotone in each coordinate, so cyclic exact coordinate
// maximization is a small, deterministic solver for this 3-variable problem.
// It has no external solver dependency and is followed by an explicit
// feasibility check.  The caller supplies points expressed about the fitted
// center in the fixed PCA frame.
bool fitMinimumVolumeP4(const std::vector<Eigen::Vector3d>& local_points,
                        double minimum_axis,
                        Eigen::Vector3d* semi_axes) {
  if (semi_axes == nullptr || local_points.empty() ||
      !std::isfinite(minimum_axis) || minimum_axis <= 0.0) {
    return false;
  }

  constexpr double kExtentEpsilon = 1.0e-9;
  constexpr double kCoefficientEpsilon = 1.0e-18;
  constexpr double kFeasibilityTolerance = 1.0e-10;
  constexpr std::size_t kMaxIterations = 256;

  Eigen::Vector3d extent = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d& point : local_points) {
    if (!point.allFinite()) return false;
    extent = extent.cwiseMax(point.cwiseAbs());
  }
  // If a component is planar/collinear, the minimum-volume problem is
  // unbounded in that direction.  The conservative box fallback handles this
  // degenerate case while preserving the configured minimum axis.
  if ((extent.array() <= kExtentEpsilon).any()) return false;

  Eigen::Vector3d upper = Eigen::Vector3d::Constant(
      1.0 / std::pow(minimum_axis, 4));
  Eigen::Vector3d coefficient = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    const double extent4 = std::pow(extent(axis), 4);
    coefficient(axis) = std::min(upper(axis), 1.0 / (3.0 * extent4));
  }

  auto constraintValue = [&](const Eigen::Vector3d& point) {
    return (coefficient.array() * point.cwiseAbs().array().pow(4)).sum();
  };
  for (std::size_t iteration = 0; iteration < kMaxIterations; ++iteration) {
    const Eigen::Vector3d previous = coefficient;
    for (int axis = 0; axis < 3; ++axis) {
      double bound = upper(axis);
      bool constrained = false;
      for (const Eigen::Vector3d& point : local_points) {
        const Eigen::Vector3d fourth = point.cwiseAbs().array().pow(4);
        const double coefficient_axis = fourth(axis);
        if (coefficient_axis <= kCoefficientEpsilon) continue;
        constrained = true;
        const double other = (coefficient.array() * fourth.array()).sum() -
                             coefficient(axis) * coefficient_axis;
        const double residual = 1.0 - other;
        if (residual < -kFeasibilityTolerance) return false;
        bound = std::min(bound, std::max(0.0, residual) / coefficient_axis);
      }
      if (constrained) {
        if (!std::isfinite(bound) || bound <= kCoefficientEpsilon) {
          return false;
        }
        // Stay just inside the boundary so the next coordinate update cannot
        // become infeasible solely due to floating-point roundoff.
        coefficient(axis) = std::max(kCoefficientEpsilon,
                                     bound * (1.0 - 1.0e-12));
      }
    }

    double max_constraint = 0.0;
    for (const Eigen::Vector3d& point : local_points) {
      max_constraint = std::max(max_constraint, constraintValue(point));
    }
    if (!std::isfinite(max_constraint) ||
        max_constraint > 1.0 + 10.0 * kFeasibilityTolerance) {
      return false;
    }
    const double change = (coefficient - previous).cwiseAbs().maxCoeff();
    const double scale = std::max(1.0, coefficient.cwiseAbs().maxCoeff());
    if (change <= 1.0e-10 * scale) break;
  }

  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(coefficient(axis)) ||
        coefficient(axis) <= kCoefficientEpsilon) {
      return false;
    }
    (*semi_axes)(axis) = std::pow(coefficient(axis), -0.25);
  }
  if (!semi_axes->allFinite()) return false;
  *semi_axes = semi_axes->cwiseMax(
      Eigen::Vector3d::Constant(minimum_axis));

  // Final feasibility check, including the minimum-axis clamp.  A tiny
  // uniform enlargement is enough to absorb roundoff without changing the
  // fitted shape materially.
  double max_constraint = 0.0;
  for (const Eigen::Vector3d& point : local_points) {
    const Eigen::Array3d normalized =
        point.cwiseAbs().array() / semi_axes->array();
    max_constraint = std::max(max_constraint, normalized.pow(4).sum());
  }
  if (!std::isfinite(max_constraint)) return false;
  if (max_constraint > 1.0) {
    (*semi_axes) *= std::pow(max_constraint, 0.25);
  }
  return semi_axes->allFinite();
}

StageCandidate fitCandidate(const std::vector<RiskVoxel>& stage_voxels,
                             const std::vector<std::size_t>& indices,
                             const RiskRegionConstructorConfig& config,
                             bool inflate = true) {
  StageCandidate result;
  result.stage = stage_voxels.at(indices.front()).stage_index;
  result.time = stage_voxels.at(indices.front()).prediction_time;

  std::vector<Eigen::Vector3d> boundary_points;
  boundary_points.reserve(indices.size() * 8);
  result.mass = 0.0;
  const double inflation =
      inflate ? std::max(0.0, config.risk_radius) : 0.0;
  const double half_extent = 0.5 * config.voxel_resolution + inflation;
  for (const std::size_t index : indices) {
    const RiskVoxel& voxel = stage_voxels.at(index);
    result.mass += std::max(0.0, voxel.weight);
    result.max_probability =
        std::max(result.max_probability, std::max(0.0, voxel.weight));
    for (int sx : {-1, 1}) {
      for (int sy : {-1, 1}) {
        for (int sz : {-1, 1}) {
          boundary_points.push_back(
              voxel.position + Eigen::Vector3d(sx * half_extent,
                                                sy * half_extent,
                                                sz * half_extent));
        }
      }
    }
  }
  if (boundary_points.empty() || result.mass <= 0.0) {
    return result;
  }
  // The document specifies geometric PCA over the complete voxel boundary
  // point set.  Occupancy is used for mass retention, not to bias the shape
  // orientation toward one high-probability voxel.
  Eigen::Vector3d boundary_sum = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d& point : boundary_points) boundary_sum += point;
  const Eigen::Vector3d mean =
      boundary_sum / static_cast<double>(boundary_points.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const Eigen::Vector3d& point : boundary_points) {
    const Eigen::Vector3d d = point - mean;
    covariance += d * d.transpose();
  }
  covariance /= static_cast<double>(boundary_points.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return result;
  }
  result.orientation = solver.eigenvectors();
  if (result.orientation.determinant() < 0.0) {
    result.orientation.col(0) *= -1.0;
  }

  Eigen::Vector3d local_min = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d local_max = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());
  for (const Eigen::Vector3d& point : boundary_points) {
    const Eigen::Vector3d local = result.orientation.transpose() * (point - mean);
    local_min = local_min.cwiseMin(local);
    local_max = local_max.cwiseMax(local);
  }
  const Eigen::Vector3d local_center = 0.5 * (local_min + local_max);
  result.center = mean + result.orientation * local_center;

  std::vector<Eigen::Vector3d> local_points;
  local_points.reserve(boundary_points.size());
  for (const Eigen::Vector3d& point : boundary_points) {
    local_points.push_back(
        result.orientation.transpose() * (point - result.center));
  }

  // Step 5: fit the minimum-volume p=4 superellipsoid for the fixed center and
  // PCA orientation.  Degenerate point sets use the old outer-box fallback.
  const bool fitted = fitMinimumVolumeP4(local_points, config.minimum_axis,
                                         &result.semi_axes);
  if (!fitted) {
    result.semi_axes = 0.5 * (local_max - local_min);
    result.semi_axes = result.semi_axes.cwiseMax(
        Eigen::Vector3d::Constant(config.minimum_axis));
    const double p4_value = [&]() {
      double maximum = 0.0;
      for (const Eigen::Vector3d& point : local_points) {
        const Eigen::Array3d normalized =
            point.cwiseAbs().array() / result.semi_axes.array();
        maximum = std::max(maximum, normalized.pow(4).sum());
      }
      return maximum;
    }();
    if (p4_value > 1.0) {
      result.semi_axes *= std::pow(p4_value, 0.25);
    }
  }

  // Each voxel cube is expanded by the configured radius before PCA/fitting.
  // The expanded cube is a conservative axis-aligned enclosure of the exact
  // Euclidean Minkowski sum voxel ⊕ B_3(r), so the fitted region remains safe
  // for spherical UAV geometry without a second, hidden inflation step.
  return result;
}

double superellipsoidVolume(const Eigen::Vector3d& semi_axes) {
  if (!semi_axes.allFinite() || (semi_axes.array() <= 0.0).any()) {
    return std::numeric_limits<double>::infinity();
  }
  // Volume of {sum_i |x_i/a_i|^p <= 1} for p=4 in R^3.
  constexpr double kP = 4.0;
  const double unit_volume =
      8.0 * std::pow(std::tgamma(1.0 + 1.0 / kP), 3.0) /
      std::tgamma(1.0 + 3.0 / kP);
  return unit_volume * semi_axes.prod();
}

bool subsetIsConnected(const std::vector<RiskVoxel>& stage_voxels,
                       const std::vector<std::size_t>& indices,
                       double resolution) {
  if (indices.empty() || !std::isfinite(resolution) || resolution <= 0.0) {
    return false;
  }
  std::unordered_map<Cell, std::size_t, CellHash> cells;
  cells.reserve(indices.size());
  for (const std::size_t index : indices) {
    cells[toCell(stage_voxels.at(index).position, resolution)] = index;
  }
  std::unordered_set<Cell, CellHash> visited;
  std::vector<Cell> queue;
  const Cell seed = toCell(stage_voxels.at(indices.front()).position,
                           resolution);
  queue.push_back(seed);
  visited.insert(seed);
  for (std::size_t q = 0; q < queue.size(); ++q) {
    for (const Cell& neighbour : neighbors26(queue[q])) {
      if (cells.find(neighbour) != cells.end() &&
          visited.insert(neighbour).second) {
        queue.push_back(neighbour);
      }
    }
  }
  return visited.size() == cells.size();
}

std::vector<StageCandidate> splitCandidateRecursive(
    const std::vector<RiskVoxel>& stage_voxels,
    const std::vector<std::size_t>& indices,
    const RiskRegionConstructorConfig& config,
    std::size_t depth) {
  const StageCandidate parent = fitCandidate(stage_voxels, indices, config, true);
  if (indices.size() < 2 * config.minimum_component_voxels ||
      depth >= config.max_split_depth || parent.mass <= 0.0) {
    return {parent};
  }

  const double dilated_voxel_side =
      config.voxel_resolution + 2.0 * std::max(0.0, config.risk_radius);
  const double occupied_volume = static_cast<double>(indices.size()) *
                                 std::pow(dilated_voxel_side, 3.0);
  const double parent_volume = superellipsoidVolume(parent.semi_axes);
  if (!std::isfinite(parent_volume) || occupied_volume <= 0.0 ||
      parent_volume / occupied_volume <= config.split_volume_ratio) {
    return {parent};
  }

  const StageCandidate uninflated = fitCandidate(stage_voxels, indices, config, false);
  double best_volume = parent_volume;
  std::vector<std::size_t> best_left;
  std::vector<std::size_t> best_right;
  for (int axis = 0; axis < 3; ++axis) {
    std::vector<std::pair<double, std::size_t>> projections;
    projections.reserve(indices.size());
    for (const std::size_t index : indices) {
      const Eigen::Vector3d local =
          uninflated.orientation.transpose() *
          (stage_voxels.at(index).position - uninflated.center);
      projections.emplace_back(local(axis), index);
    }
    std::sort(projections.begin(), projections.end());
    if (projections.size() < 2) continue;
    const std::size_t split_at = projections.size() / 2;
    const double cut =
        0.5 * (projections[split_at - 1].first + projections[split_at].first);
    std::vector<std::size_t> left;
    std::vector<std::size_t> right;
    left.reserve(split_at);
    right.reserve(projections.size() - split_at);
    for (const auto& projection : projections) {
      (projection.first <= cut ? left : right).push_back(projection.second);
    }
    if (left.size() < config.minimum_component_voxels ||
        right.size() < config.minimum_component_voxels ||
        !subsetIsConnected(stage_voxels, left, config.voxel_resolution) ||
        !subsetIsConnected(stage_voxels, right, config.voxel_resolution)) {
      continue;
    }
    const StageCandidate left_fit = fitCandidate(stage_voxels, left, config, true);
    const StageCandidate right_fit = fitCandidate(stage_voxels, right, config, true);
    const double volume = superellipsoidVolume(left_fit.semi_axes) +
                          superellipsoidVolume(right_fit.semi_axes);
    if (std::isfinite(volume) && volume < best_volume) {
      best_volume = volume;
      best_left = std::move(left);
      best_right = std::move(right);
    }
  }

  if (best_left.empty() || best_right.empty() ||
      best_volume > parent_volume *
                        (1.0 - std::max(0.0, config.split_min_volume_reduction))) {
    return {parent};
  }

  std::vector<StageCandidate> result;
  std::vector<StageCandidate> left_result = splitCandidateRecursive(
      stage_voxels, best_left, config, depth + 1);
  std::vector<StageCandidate> right_result = splitCandidateRecursive(
      stage_voxels, best_right, config, depth + 1);
  result.reserve(left_result.size() + right_result.size());
  result.insert(result.end(), left_result.begin(), left_result.end());
  result.insert(result.end(), right_result.begin(), right_result.end());
  return result;
}

}  // namespace

std::vector<RiskRegionData> constructRiskRegions(
    const std::vector<RiskVoxel>& input,
    const RiskRegionConstructorConfig& config) {
  std::map<std::uint32_t, std::vector<RiskVoxel>> stages;
  for (const RiskVoxel& voxel : input) {
    if (voxel.stage_index >= config.prediction_stages ||
        !std::isfinite(voxel.weight) || voxel.weight <= config.occupancy_min ||
        !voxel.position.allFinite()) {
      continue;
    }
    stages[voxel.stage_index].push_back(voxel);
  }

  std::vector<std::vector<StageCandidate>> candidates(config.prediction_stages);
  for (auto& stage_entry : stages) {
    auto& stage_voxels = stage_entry.second;
    std::unordered_map<Cell, std::size_t, CellHash> cell_to_index;
    for (std::size_t i = 0; i < stage_voxels.size(); ++i) {
      cell_to_index[toCell(stage_voxels[i].position, config.voxel_resolution)] = i;
    }
    std::vector<double> neighbour_support(stage_voxels.size(), 0.0);
    double maximum_support = 0.0;
    for (std::size_t i = 0; i < stage_voxels.size(); ++i) {
      const Cell cell = toCell(stage_voxels[i].position,
                               config.voxel_resolution);
      for (const Cell& neighbour : neighbors26(cell)) {
        const auto found = cell_to_index.find(neighbour);
        if (found != cell_to_index.end()) {
          neighbour_support[i] +=
              std::max(0.0, stage_voxels[found->second].weight);
        }
      }
      maximum_support = std::max(maximum_support, neighbour_support[i]);
    }
    const double support_weight = std::max(
        0.0, std::min(1.0, config.neighbor_support_weight));
    const double support_fraction = std::max(
        0.0, std::min(1.0, config.neighbor_support_fraction));
    const double support_floor = support_fraction * maximum_support;
    auto priority = [&](std::size_t index) {
      const double support_norm = maximum_support > 0.0
                                      ? neighbour_support[index] / maximum_support
                                      : 0.0;
      const double probability = std::max(0.0, stage_voxels[index].weight);
      // Probability remains the mass being retained; support only orders the
      // connected growth so that isolated high-probability tails are visited
      // after the spatially supported core.
      return probability *
             ((1.0 - support_weight) + support_weight * support_norm);
    };
    std::vector<bool> visited(stage_voxels.size(), false);
    for (std::size_t seed = 0; seed < stage_voxels.size(); ++seed) {
      if (visited[seed]) continue;
      std::vector<std::size_t> component;
      std::vector<std::size_t> queue{seed};
      visited[seed] = true;
      for (std::size_t q = 0; q < queue.size(); ++q) {
        const std::size_t current = queue[q];
        component.push_back(current);
        const Cell cell = toCell(stage_voxels[current].position,
                                 config.voxel_resolution);
        for (const Cell& neighbor : neighbors26(cell)) {
          const auto found = cell_to_index.find(neighbor);
          if (found != cell_to_index.end() && !visited[found->second]) {
            visited[found->second] = true;
            queue.push_back(found->second);
          }
        }
      }
      double mass = 0.0;
      for (const std::size_t i : component) mass += stage_voxels[i].weight;
      if (component.size() < config.minimum_component_voxels ||
          mass < config.minimum_component_mass) {
        continue;
      }
      std::vector<bool> component_member(stage_voxels.size(), false);
      for (const std::size_t index : component) component_member[index] = true;

      // Seed from the high-support core when one exists.  The subsequent
      // priority-queue growth is restricted to graph neighbours, so the 95%
      // mass set remains 26-connected instead of becoming an unconstrained
      // top-probability subset.
      std::size_t seed_index = component.front();
      bool found_core_seed = false;
      for (const std::size_t index : component) {
        const bool core = maximum_support <= 0.0 ||
                          neighbour_support[index] >= support_floor;
        if (!core) continue;
        if (!found_core_seed || priority(index) > priority(seed_index)) {
          seed_index = index;
          found_core_seed = true;
        }
      }
      if (!found_core_seed) {
        for (const std::size_t index : component) {
          if (stage_voxels[index].weight > stage_voxels[seed_index].weight) {
            seed_index = index;
          }
        }
      }

      const double retained_fraction = std::max(
          0.0, std::min(1.0, config.retained_mass));
      const double target = retained_fraction * mass;
      double retained = 0.0;
      std::vector<std::size_t> selected;
      std::vector<bool> selected_mask(stage_voxels.size(), false);
      std::priority_queue<std::pair<double, std::size_t>> frontier;
      frontier.emplace(priority(seed_index), seed_index);
      while (retained < target && selected.size() < component.size()) {
        std::size_t current = stage_voxels.size();
        while (!frontier.empty()) {
          const std::size_t candidate = frontier.top().second;
          frontier.pop();
          if (component_member[candidate] && !selected_mask[candidate]) {
            current = candidate;
            break;
          }
        }
        if (current == stage_voxels.size()) {
          // A valid connected component should always have a frontier.  This
          // fallback keeps the mass invariant robust to duplicate voxel keys.
          double best_priority = -1.0;
          for (const std::size_t index : component) {
            if (!selected_mask[index] && priority(index) > best_priority) {
              best_priority = priority(index);
              current = index;
            }
          }
          if (current == stage_voxels.size()) break;
        }
        selected_mask[current] = true;
        selected.push_back(current);
        retained += std::max(0.0, stage_voxels[current].weight);
        const Cell cell = toCell(stage_voxels[current].position,
                                 config.voxel_resolution);
        for (const Cell& neighbour : neighbors26(cell)) {
          const auto found = cell_to_index.find(neighbour);
          if (found != cell_to_index.end() &&
              component_member[found->second] &&
              !selected_mask[found->second]) {
            frontier.emplace(priority(found->second), found->second);
          }
        }
      }
      if (selected.empty()) {
        continue;
      }
      const std::vector<StageCandidate> split_candidates =
          splitCandidateRecursive(stage_voxels, selected, config, 0);
      for (const StageCandidate& candidate : split_candidates) {
        if (candidate.mass > 0.0) {
          candidates[stage_entry.first].push_back(candidate);
        }
      }
    }
  }

  std::uint32_t next_track = 0;
  std::vector<StageCandidate> previous;
  for (std::uint32_t stage = 0; stage < config.prediction_stages; ++stage) {
    auto& current = candidates[stage];
    std::vector<bool> used(previous.size(), false);
    for (StageCandidate& candidate : current) {
      double best_distance = config.track_match_distance;
      std::size_t best = previous.size();
      for (std::size_t i = 0; i < previous.size(); ++i) {
        if (used[i]) continue;
        const double distance = (candidate.center - previous[i].center).norm();
        if (distance < best_distance) {
          best_distance = distance;
          best = i;
        }
      }
      if (best < previous.size()) {
        candidate.track = previous[best].track;
        used[best] = true;
      } else {
        candidate.track = next_track++;
      }
    }
    previous = current;
  }

  // Use persistence across the internally aligned DSP horizon as a weak
  // temporal denoiser.  The 21 slices are not treated as independent
  // probability observations: a track is simply required to remain present
  // over consecutive stages, while a high-confidence single-stage region is
  // admitted immediately so that a newly appearing pedestrian is not delayed.
  std::map<std::uint32_t, std::vector<std::uint32_t>> track_stages;
  std::map<std::uint32_t, double> track_max_probability;
  for (std::uint32_t stage = 0; stage < config.prediction_stages; ++stage) {
    for (const StageCandidate& candidate : candidates[stage]) {
      track_stages[candidate.track].push_back(stage);
      track_max_probability[candidate.track] = std::max(
          track_max_probability[candidate.track], candidate.max_probability);
    }
  }
  std::map<std::uint32_t, std::size_t> track_longest_run;
  for (auto& entry : track_stages) {
    auto& stages_for_track = entry.second;
    std::sort(stages_for_track.begin(), stages_for_track.end());
    std::size_t longest = 0;
    std::size_t current_run = 0;
    std::uint32_t previous_stage = 0;
    for (const std::uint32_t stage : stages_for_track) {
      if (current_run == 0 || stage == previous_stage + 1) {
        ++current_run;
      } else {
        current_run = 1;
      }
      longest = std::max(longest, current_run);
      previous_stage = stage;
    }
    track_longest_run[entry.first] = longest;
  }
  const std::size_t required_stages =
      std::max<std::size_t>(1, config.minimum_persistent_stages);
  for (std::vector<StageCandidate>& stage_candidates : candidates) {
    stage_candidates.erase(
        std::remove_if(stage_candidates.begin(), stage_candidates.end(),
                       [&](const StageCandidate& candidate) {
                         const bool persistent =
                             track_longest_run[candidate.track] >= required_stages;
                         const bool high_confidence =
                             track_max_probability[candidate.track] >=
                             config.persistence_high_confidence;
                         return !persistent && !high_confidence;
                       }),
        stage_candidates.end());
  }

  std::vector<RiskRegionData> output;
  for (std::uint32_t stage = 0; stage < config.prediction_stages; ++stage) {
    for (const StageCandidate& candidate : candidates[stage]) {
      RiskRegionData region;
      region.track_id = candidate.track;
      region.stage_index = candidate.stage;
      region.prediction_time = candidate.time;
      region.center = candidate.center;
      region.orientation = candidate.orientation;
      region.semi_axes = candidate.semi_axes;
      region.mass = candidate.mass;
      region.max_probability = candidate.max_probability;
      region.valid = true;
      output.push_back(region);
    }
  }

  // Derive center velocity and acceleration from the already predicted region
  // centers. These are data values for the stage-wise HOCBF, not optimization
  // variables and not derivatives of a,b,c or Q.
  std::map<std::uint32_t, std::vector<std::size_t>> by_track;
  for (std::size_t i = 0; i < output.size(); ++i) {
    by_track[output[i].track_id].push_back(i);
  }
  for (auto& track_entry : by_track) {
    auto& indices = track_entry.second;
    std::sort(indices.begin(), indices.end(), [&](std::size_t lhs, std::size_t rhs) {
      return output[lhs].stage_index < output[rhs].stage_index;
    });
    for (std::size_t n = 0; n < indices.size(); ++n) {
      const std::size_t index = indices[n];
      if (indices.size() == 1) continue;
      if (n == 0) {
        const std::size_t next = indices[n + 1];
        output[index].velocity = (output[next].center - output[index].center) /
                                 config.prediction_dt;
      } else if (n + 1 == indices.size()) {
        const std::size_t prev = indices[n - 1];
        output[index].velocity = (output[index].center - output[prev].center) /
                                 config.prediction_dt;
      } else {
        const std::size_t prev = indices[n - 1];
        const std::size_t next = indices[n + 1];
        output[index].velocity = (output[next].center - output[prev].center) /
                                 (2.0 * config.prediction_dt);
      }
    }
    for (std::size_t n = 1; n + 1 < indices.size(); ++n) {
      const std::size_t prev = indices[n - 1];
      const std::size_t current = indices[n];
      const std::size_t next = indices[n + 1];
      output[current].acceleration =
          (output[next].center - 2.0 * output[current].center +
           output[prev].center) /
          (config.prediction_dt * config.prediction_dt);
    }
  }
  return output;
}

}  // namespace coni_mpc
