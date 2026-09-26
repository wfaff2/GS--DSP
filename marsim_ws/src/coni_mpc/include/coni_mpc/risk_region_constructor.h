#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>

namespace coni_mpc {

struct RiskVoxel {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double weight = 0.0;
  std::uint32_t stage_index = 0;
  double prediction_time = 0.0;
};

struct RiskRegionData {
  std::uint32_t track_id = 0;
  std::uint32_t stage_index = 0;
  double prediction_time = 0.0;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d semi_axes = Eigen::Vector3d::Ones();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  double mass = 0.0;
  double max_probability = 0.0;
  bool valid = false;
};

struct RiskRegionConstructorConfig {
  std::uint32_t prediction_stages = 21;
  double prediction_dt = 0.1;
  double voxel_resolution = 0.15;
  double occupancy_min = 1.0e-6;
  double minimum_component_mass = 1.0e-5;
  std::size_t minimum_component_voxels = 2;
  double retained_mass = 0.95;
  // Fraction of the maximum 26-neighbour support used to choose a reliable
  // seed inside each component.  Support is a local consistency score, not a
  // probability, and is deliberately not used as a hard mass denominator.
  double neighbor_support_fraction = 0.15;
  // Weight of normalized neighbourhood support in the connected mass-growth
  // priority.  The remaining weight is the voxel occupancy probability.
  double neighbor_support_weight = 0.5;
  // A region which appears in only one prediction slice is rejected unless it
  // contains a high-confidence voxel.  This is a temporal persistence check
  // across the DSP horizon, not an assumption that the 21 slices are
  // independent observations.
  std::size_t minimum_persistent_stages = 2;
  double persistence_high_confidence = 0.65;
  // Total Minkowski-ball inflation. The ROS node can derive this from the
  // UAV, controller, and perception margins when no explicit override exists.
  double risk_radius = 0.5;
  double track_match_distance = 2.0;
  double minimum_axis = 0.05;
  // Recursive PCA-axis split for non-convex components.  The split is
  // accepted only when it preserves voxel coverage and reduces the total
  // fitted volume by this fraction.
  double split_volume_ratio = 2.0;
  double split_min_volume_reduction = 0.15;
  std::size_t max_split_depth = 2;
};

std::vector<RiskRegionData> constructRiskRegions(
    const std::vector<RiskVoxel>& voxels,
    const RiskRegionConstructorConfig& config = RiskRegionConstructorConfig());

}  // namespace coni_mpc
