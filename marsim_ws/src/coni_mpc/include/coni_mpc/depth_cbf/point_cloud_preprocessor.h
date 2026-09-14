#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/time.h>

namespace coni_mpc {
namespace depth_cbf {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

struct Pose3d {
  Eigen::Vector3d position_W = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation_WF = Eigen::Matrix3d::Identity();
  ros::Time stamp;
};

struct PreprocessorConfig {
  double min_range = 1.0;
  double max_range = 15.0;
  double local_radius = 4.0;
  double voxel_leaf_size = 0.10;
  bool enable_voxel_filter = true;
  bool enable_ground_filter = false;
  double ground_min_z_W = 0.10;

  int query_mesh_half_width_steps = 2;
  double query_mesh_spacing = 0.15;
  double nearest_neighbor_max_distance = 0.75;

  Eigen::Matrix3d rotation_BL = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_BL = Eigen::Vector3d::Zero();
};

struct PointTransformTrace {
  Eigen::Vector3d q_L = Eigen::Vector3d::Zero();
  Eigen::Vector3d q_B = Eigen::Vector3d::Zero();
  Eigen::Vector3d q_W = Eigen::Vector3d::Zero();
  Eigen::Vector3d q_N = Eigen::Vector3d::Zero();
};

struct PreprocessedClouds {
  PointCloud::Ptr cloud_W{new PointCloud};
  PointCloud::Ptr cloud_N{new PointCloud};
  PointCloud::Ptr local_cloud_N{new PointCloud};
  Eigen::Vector3d query_center_N = Eigen::Vector3d::Zero();
  ros::Time stamp;
  std::vector<std::size_t> retained_input_indices;
};

class PointCloudPreprocessor {
 public:
  explicit PointCloudPreprocessor(const PreprocessorConfig& config);

  static double queryMeshRadius(const PreprocessorConfig& config);
  static double requiredLocalRadius(const PreprocessorConfig& config);
  static bool validateConfig(const PreprocessorConfig& config,
                             std::string* error,
                             std::string* warning);

  PointTransformTrace transformPoint(const Eigen::Vector3d& q_L,
                                     const Pose3d& pose_WB,
                                     const Pose3d& pose_WN) const;

  bool process(const PointCloud& cloud_L,
               const Pose3d& pose_WB,
               const Pose3d& pose_WN,
               const ros::Time& cloud_stamp,
               PreprocessedClouds* output,
               std::string* error = nullptr) const;

  const PreprocessorConfig& config() const { return config_; }

 private:
  PreprocessorConfig config_;
};

}  // namespace depth_cbf
}  // namespace coni_mpc
