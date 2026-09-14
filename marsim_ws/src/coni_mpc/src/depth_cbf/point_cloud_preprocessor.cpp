#include "coni_mpc/depth_cbf/point_cloud_preprocessor.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <pcl/filters/voxel_grid.h>

namespace coni_mpc {
namespace depth_cbf {
namespace {

bool isFinite(const Eigen::Vector3d& value) {
  return value.array().isFinite().all();
}

pcl::PointXYZ toPcl(const Eigen::Vector3d& value) {
  return pcl::PointXYZ(static_cast<float>(value.x()),
                       static_cast<float>(value.y()),
                       static_cast<float>(value.z()));
}

}  // namespace

PointCloudPreprocessor::PointCloudPreprocessor(
    const PreprocessorConfig& config)
    : config_(config) {
  std::string error;
  std::string warning;
  if (!validateConfig(config_, &error, &warning)) {
    throw std::invalid_argument(error);
  }
}

double PointCloudPreprocessor::queryMeshRadius(
    const PreprocessorConfig& config) {
  return std::sqrt(3.0) *
         static_cast<double>(config.query_mesh_half_width_steps) *
         config.query_mesh_spacing;
}

double PointCloudPreprocessor::requiredLocalRadius(
    const PreprocessorConfig& config) {
  return queryMeshRadius(config) + config.nearest_neighbor_max_distance;
}

bool PointCloudPreprocessor::validateConfig(const PreprocessorConfig& config,
                                            std::string* error,
                                            std::string* warning) {
  if (error != nullptr) {
    error->clear();
  }
  if (warning != nullptr) {
    warning->clear();
  }
  auto fail = [error](const std::string& message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };

  if (!(config.min_range >= 0.0 && config.max_range > config.min_range)) {
    return fail("Require 0 <= min_range < max_range.");
  }
  if (!(config.local_radius > 0.0)) {
    return fail("local_radius must be positive.");
  }
  if (config.query_mesh_half_width_steps < 1 ||
      !(config.query_mesh_spacing > 0.0)) {
    return fail("Query mesh requires half_width_steps >= 1 and spacing > 0.");
  }
  if (!(config.nearest_neighbor_max_distance > 0.0)) {
    return fail("nearest_neighbor_max_distance must be positive.");
  }
  if (config.enable_voxel_filter && !(config.voxel_leaf_size > 0.0)) {
    return fail("voxel_leaf_size must be positive when voxel filtering is enabled.");
  }
  if (!config.rotation_BL.array().isFinite().all() ||
      !config.translation_BL.array().isFinite().all()) {
    return fail("LiDAR-to-body extrinsics must be finite.");
  }
  const Eigen::Matrix3d orthogonality =
      config.rotation_BL.transpose() * config.rotation_BL;
  if (!orthogonality.isApprox(Eigen::Matrix3d::Identity(), 1e-8) ||
      std::abs(config.rotation_BL.determinant() - 1.0) > 1e-8) {
    return fail("rotation_BL must be a proper rotation matrix.");
  }

  const double required_radius = requiredLocalRadius(config);
  if (config.local_radius + 1e-12 < required_radius) {
    return fail("local_radius is too small for the query mesh: require " +
                std::to_string(required_radius) +
                " m (= r_mesh + nearest_neighbor_max_distance), got " +
                std::to_string(config.local_radius) + " m.");
  }
  if (warning != nullptr && config.local_radius < 1.1 * required_radius) {
    *warning = "local_radius has less than 10% reserve beyond the required "
               "query-mesh support radius.";
  }
  return true;
}

PointTransformTrace PointCloudPreprocessor::transformPoint(
    const Eigen::Vector3d& q_L,
    const Pose3d& pose_WB,
    const Pose3d& pose_WN) const {
  PointTransformTrace trace;
  trace.q_L = q_L;
  trace.q_B = config_.rotation_BL * trace.q_L + config_.translation_BL;
  trace.q_W = pose_WB.rotation_WF * trace.q_B + pose_WB.position_W;
  trace.q_N = pose_WN.rotation_WF.transpose() *
              (trace.q_W - pose_WN.position_W);
  return trace;
}

bool PointCloudPreprocessor::process(const PointCloud& cloud_L,
                                     const Pose3d& pose_WB,
                                     const Pose3d& pose_WN,
                                     const ros::Time& cloud_stamp,
                                     PreprocessedClouds* output,
                                     std::string* error) const {
  if (output == nullptr) {
    if (error != nullptr) {
      *error = "Output pointer is null.";
    }
    return false;
  }
  if (!pose_WB.rotation_WF.array().isFinite().all() ||
      !pose_WN.rotation_WF.array().isFinite().all() ||
      !isFinite(pose_WB.position_W) || !isFinite(pose_WN.position_W)) {
    if (error != nullptr) {
      *error = "Synchronized UAV/UGV poses contain non-finite values.";
    }
    return false;
  }

  output->cloud_W.reset(new PointCloud);
  output->cloud_N.reset(new PointCloud);
  output->local_cloud_N.reset(new PointCloud);
  output->retained_input_indices.clear();
  output->stamp = cloud_stamp;
  output->query_center_N = pose_WN.rotation_WF.transpose() *
                           (pose_WB.position_W - pose_WN.position_W);

  output->cloud_W->reserve(cloud_L.size());
  output->cloud_N->reserve(cloud_L.size());
  output->retained_input_indices.reserve(cloud_L.size());
  PointCloud::Ptr local_unfiltered_N(new PointCloud);
  local_unfiltered_N->reserve(cloud_L.size());

  const double min_range_squared = config_.min_range * config_.min_range;
  const double max_range_squared = config_.max_range * config_.max_range;
  const double local_radius_squared =
      config_.local_radius * config_.local_radius;

  for (std::size_t input_index = 0; input_index < cloud_L.size();
       ++input_index) {
    const pcl::PointXYZ& pcl_q_L = cloud_L.points[input_index];
    const Eigen::Vector3d q_L(pcl_q_L.x, pcl_q_L.y, pcl_q_L.z);
    if (!isFinite(q_L)) {
      continue;
    }
    const double range_squared = q_L.squaredNorm();
    if (range_squared < min_range_squared ||
        range_squared > max_range_squared) {
      continue;
    }
    const PointTransformTrace trace = transformPoint(q_L, pose_WB, pose_WN);
    if (config_.enable_ground_filter &&
        trace.q_W.z() < config_.ground_min_z_W) {
      continue;
    }

    output->cloud_W->push_back(toPcl(trace.q_W));
    output->cloud_N->push_back(toPcl(trace.q_N));
    output->retained_input_indices.push_back(input_index);
    if ((trace.q_N - output->query_center_N).squaredNorm() <=
        local_radius_squared) {
      local_unfiltered_N->push_back(toPcl(trace.q_N));
    }
  }

  if (config_.enable_voxel_filter && !local_unfiltered_N->empty()) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(local_unfiltered_N);
    const float leaf = static_cast<float>(config_.voxel_leaf_size);
    voxel_filter.setLeafSize(leaf, leaf, leaf);
    voxel_filter.filter(*output->local_cloud_N);
  } else {
    *output->local_cloud_N = *local_unfiltered_N;
  }

  output->cloud_W->width = output->cloud_W->size();
  output->cloud_W->height = 1;
  output->cloud_W->is_dense = false;
  output->cloud_N->width = output->cloud_N->size();
  output->cloud_N->height = 1;
  output->cloud_N->is_dense = false;
  output->local_cloud_N->width = output->local_cloud_N->size();
  output->local_cloud_N->height = 1;
  output->local_cloud_N->is_dense = false;
  return true;
}

}  // namespace depth_cbf
}  // namespace coni_mpc
