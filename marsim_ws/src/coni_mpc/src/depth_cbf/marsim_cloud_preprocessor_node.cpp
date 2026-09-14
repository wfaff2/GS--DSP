#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>

#include "coni_mpc/depth_cbf/point_cloud_preprocessor.h"

namespace coni_mpc {
namespace depth_cbf {
namespace {

Pose3d poseFromOdom(const nav_msgs::Odometry& odom) {
  Pose3d pose;
  pose.position_W = Eigen::Vector3d(odom.pose.pose.position.x,
                                    odom.pose.pose.position.y,
                                    odom.pose.pose.position.z);
  Eigen::Quaterniond quaternion_WF(odom.pose.pose.orientation.w,
                                  odom.pose.pose.orientation.x,
                                  odom.pose.pose.orientation.y,
                                  odom.pose.pose.orientation.z);
  if (quaternion_WF.norm() < 1e-12) {
    throw std::runtime_error("Odometry quaternion has zero norm.");
  }
  quaternion_WF.normalize();
  pose.rotation_WF = quaternion_WF.toRotationMatrix();
  pose.stamp = odom.header.stamp;
  return pose;
}

std::vector<double> requireVectorParam(ros::NodeHandle* private_nh,
                                       const std::string& name,
                                       std::size_t expected_size) {
  std::vector<double> values;
  if (!private_nh->getParam(name, values) || values.size() != expected_size) {
    throw std::runtime_error("Parameter ~" + name + " must contain exactly " +
                             std::to_string(expected_size) + " numbers.");
  }
  return values;
}

}  // namespace

class MarsimCloudPreprocessorNode {
 public:
  MarsimCloudPreprocessorNode()
      : nh_(),
        private_nh_("~"),
        cloud_sub_(nh_, readString("cloud_topic",
                                   "/quad0_pcl_render_node/sensor_cloud"),
                   readInt("subscriber_queue_size", 20)),
        quad_odom_sub_(nh_, readString("quad_odom_topic",
                                       "/coni_mpc/quad_odom1"),
                       readInt("subscriber_queue_size", 20)),
        car_odom_sub_(nh_, readString("car_odom_topic",
                                      "/coni_mpc/car_odom"),
                      readInt("subscriber_queue_size", 20)),
        random_engine_(static_cast<std::mt19937::result_type>(
            readInt("debug_random_seed", 7))) {
    const PreprocessorConfig config = loadConfig();
    std::string error;
    std::string warning;
    if (!PointCloudPreprocessor::validateConfig(config, &error, &warning)) {
      ROS_FATAL_STREAM("Depth-CBF preprocessing parameter error: " << error);
      throw std::runtime_error(error);
    }
    if (!warning.empty()) {
      ROS_WARN_STREAM(warning);
    }
    preprocessor_.reset(new PointCloudPreprocessor(config));

    private_nh_.param("max_pose_time_offset_sec", max_pose_time_offset_sec_,
                      0.025);
    private_nh_.param("debug_sample_count", debug_sample_count_, 3);
    private_nh_.param("debug_print_every_n_clouds", debug_print_every_n_clouds_,
                      20);
    private_nh_.param("debug_compare_world_cloud", debug_compare_world_cloud_,
                      true);
    private_nh_.param("world_frame_id", world_frame_id_, std::string("map"));
    private_nh_.param("noninertial_frame_id", noninertial_frame_id_,
                      std::string("base_link"));

    double r_uav = 0.15;
    double delta_surface = 0.0;
    double d_safe = 0.15;
    private_nh_.param("safety/r_uav", r_uav, r_uav);
    private_nh_.param("safety/delta_surface", delta_surface, delta_surface);
    private_nh_.param("safety/d_safe", d_safe, d_safe);
    if (std::abs(d_safe - (r_uav + delta_surface)) > 1e-9) {
      throw std::runtime_error(
          "Require d_safe = r_uav + delta_surface for surface LiDAR points.");
    }
    if (delta_surface <= 0.0) {
      ROS_WARN("delta_surface is a provisional M1/M2 value (<= 0); select its "
               "final value before closed-loop use.");
    }
    if (config.min_range > d_safe) {
      ROS_WARN_STREAM("Livox blind-zone risk: min_range=" << config.min_range
                      << " m is greater than d_safe=" << d_safe
                      << " m. Obstacles entering the blind zone may disappear "
                         "before the safety boundary is reached.");
    }

    cloud_W_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
        "cloud_world", 2);
    cloud_N_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
        "cloud_noninertial", 2);
    local_cloud_N_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
        "local_cloud_noninertial", 2);
    query_center_N_pub_ = private_nh_.advertise<geometry_msgs::PointStamped>(
        "query_center_noninertial", 2);
    sync_diagnostics_pub_ = private_nh_.advertise<std_msgs::Float64MultiArray>(
        "sync_diagnostics", 10);

    const int sync_queue_size = readInt("sync_queue_size", 40);
    SyncPolicy sync_policy(sync_queue_size);
    double sync_slop_sec = 0.025;
    private_nh_.param("sync_slop_sec", sync_slop_sec, sync_slop_sec);
    sync_policy.setMaxIntervalDuration(ros::Duration(sync_slop_sec));
    // Force selection of Synchronizer(const Policy&, ...). Without the const
    // view, ROS Noetic's templated filter constructor treats the policy as a
    // fourth input filter.
    synchronizer_.reset(new Synchronizer(
        static_cast<const SyncPolicy&>(sync_policy), cloud_sub_,
        quad_odom_sub_, car_odom_sub_));
    synchronizer_->registerCallback(
        boost::bind(&MarsimCloudPreprocessorNode::synchronizedCallback, this,
                    _1, _2, _3));

    if (debug_compare_world_cloud_) {
      const std::string debug_topic = readString(
          "debug_world_cloud_topic", "/quad0_pcl_render_node/cloud");
      debug_world_cloud_sub_ =
          nh_.subscribe(debug_topic, 2,
                        &MarsimCloudPreprocessorNode::debugWorldCloudCallback,
                        this);
    }

    ROS_INFO_STREAM(
        "M1 PointCloudPreprocessor ready. Formal input="
        << cloud_sub_.getTopic()
        << "; MARSIM world cloud is debug-only. r_mesh="
        << PointCloudPreprocessor::queryMeshRadius(config)
        << " m, required_local_radius="
        << PointCloudPreprocessor::requiredLocalRadius(config)
        << " m, configured_local_radius=" << config.local_radius << " m.");
  }

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, nav_msgs::Odometry, nav_msgs::Odometry>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  std::string readString(const std::string& name,
                         const std::string& default_value) {
    std::string value = default_value;
    private_nh_.param(name, value, value);
    return value;
  }

  int readInt(const std::string& name, int default_value) {
    int value = default_value;
    private_nh_.param(name, value, value);
    return value;
  }

  PreprocessorConfig loadConfig() {
    PreprocessorConfig config;
    private_nh_.param("filter/min_range", config.min_range, config.min_range);
    private_nh_.param("filter/max_range", config.max_range, config.max_range);
    private_nh_.param("filter/local_radius", config.local_radius,
                      config.local_radius);
    private_nh_.param("filter/voxel_leaf_size", config.voxel_leaf_size,
                      config.voxel_leaf_size);
    private_nh_.param("filter/enable_voxel_filter", config.enable_voxel_filter,
                      config.enable_voxel_filter);
    private_nh_.param("filter/enable_ground_filter",
                      config.enable_ground_filter,
                      config.enable_ground_filter);
    private_nh_.param("filter/ground_min_z_W", config.ground_min_z_W,
                      config.ground_min_z_W);
    private_nh_.param("query_mesh/half_width_steps",
                      config.query_mesh_half_width_steps,
                      config.query_mesh_half_width_steps);
    private_nh_.param("query_mesh/spacing", config.query_mesh_spacing,
                      config.query_mesh_spacing);
    private_nh_.param("query_mesh/nearest_neighbor_max_distance",
                      config.nearest_neighbor_max_distance,
                      config.nearest_neighbor_max_distance);

    const std::vector<double> rotation_values =
        requireVectorParam(&private_nh_, "extrinsics/rotation_BL", 9);
    const std::vector<double> translation_values =
        requireVectorParam(&private_nh_, "extrinsics/translation_BL", 3);
    config.rotation_BL << rotation_values[0], rotation_values[1],
        rotation_values[2], rotation_values[3], rotation_values[4],
        rotation_values[5], rotation_values[6], rotation_values[7],
        rotation_values[8];
    config.translation_BL = Eigen::Vector3d(translation_values[0],
                                            translation_values[1],
                                            translation_values[2]);
    return config;
  }

  void debugWorldCloudCallback(
      const sensor_msgs::PointCloud2ConstPtr& cloud_W_debug_msg) {
    latest_debug_cloud_W_msg_ = cloud_W_debug_msg;
  }

  void synchronizedCallback(
      const sensor_msgs::PointCloud2ConstPtr& cloud_L_msg,
      const nav_msgs::OdometryConstPtr& quad_odom_msg,
      const nav_msgs::OdometryConstPtr& car_odom_msg) {
    const double quad_dt =
        (quad_odom_msg->header.stamp - cloud_L_msg->header.stamp).toSec();
    const double car_dt =
        (car_odom_msg->header.stamp - cloud_L_msg->header.stamp).toSec();
    publishSyncDiagnostics(cloud_L_msg->header.stamp,
                           quad_odom_msg->header.stamp,
                           car_odom_msg->header.stamp, quad_dt, car_dt);
    if (std::abs(quad_dt) > max_pose_time_offset_sec_ ||
        std::abs(car_dt) > max_pose_time_offset_sec_) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "Rejecting cloud: synchronized pose offset exceeds "
                   << max_pose_time_offset_sec_ << " s (quad_dt=" << quad_dt
                   << ", car_dt=" << car_dt << ").");
      return;
    }

    PointCloud cloud_L;
    pcl::fromROSMsg(*cloud_L_msg, cloud_L);
    PreprocessedClouds output;
    std::string error;
    try {
      if (!preprocessor_->process(cloud_L, poseFromOdom(*quad_odom_msg),
                                  poseFromOdom(*car_odom_msg),
                                  cloud_L_msg->header.stamp, &output, &error)) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "Point-cloud preprocessing failed: "
                                           << error);
        return;
      }
    } catch (const std::exception& exception) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Point-cloud preprocessing exception: "
                                         << exception.what());
      return;
    }

    publishCloud(*output.cloud_W, cloud_L_msg->header.stamp, world_frame_id_,
                 &cloud_W_pub_);
    publishCloud(*output.cloud_N, cloud_L_msg->header.stamp,
                 noninertial_frame_id_, &cloud_N_pub_);
    publishCloud(*output.local_cloud_N, cloud_L_msg->header.stamp,
                 noninertial_frame_id_, &local_cloud_N_pub_);

    geometry_msgs::PointStamped center_msg;
    center_msg.header.stamp = cloud_L_msg->header.stamp;
    center_msg.header.frame_id = noninertial_frame_id_;
    center_msg.point.x = output.query_center_N.x();
    center_msg.point.y = output.query_center_N.y();
    center_msg.point.z = output.query_center_N.z();
    query_center_N_pub_.publish(center_msg);

    ++cloud_count_;
    if (debug_print_every_n_clouds_ > 0 &&
        cloud_count_ % static_cast<std::size_t>(debug_print_every_n_clouds_) ==
            0) {
      printTransformSamples(cloud_L, output, cloud_L_msg->header.stamp,
                            quad_dt, car_dt);
    }
  }

  void publishCloud(const PointCloud& cloud,
                    const ros::Time& stamp,
                    const std::string& frame_id,
                    ros::Publisher* publisher) {
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    publisher->publish(message);
  }

  void publishSyncDiagnostics(const ros::Time& cloud_stamp,
                              const ros::Time& quad_stamp,
                              const ros::Time& car_stamp,
                              double quad_dt,
                              double car_dt) {
    std_msgs::Float64MultiArray message;
    message.data = {cloud_stamp.toSec(), quad_stamp.toSec(), car_stamp.toSec(),
                    quad_dt, car_dt};
    sync_diagnostics_pub_.publish(message);
    ROS_INFO_STREAM_THROTTLE(
        2.0, "M1 stamps: cloud=" << cloud_stamp << ", quad_odom=" << quad_stamp
                                  << ", car_odom=" << car_stamp
                                  << ", dt_quad=" << quad_dt
                                  << " s, dt_car=" << car_dt << " s.");
  }

  void printTransformSamples(const PointCloud& cloud_L,
                             const PreprocessedClouds& output,
                             const ros::Time& cloud_stamp,
                             double quad_dt,
                             double car_dt) {
    if (output.cloud_W->empty()) {
      ROS_WARN("No points survived M1 filtering; cannot print transform samples.");
      return;
    }
    const bool debug_cloud_matches =
        latest_debug_cloud_W_msg_ &&
        latest_debug_cloud_W_msg_->header.stamp == cloud_stamp;
    PointCloud debug_cloud_W;
    if (debug_cloud_matches) {
      pcl::fromROSMsg(*latest_debug_cloud_W_msg_, debug_cloud_W);
    }

    std::uniform_int_distribution<std::size_t> distribution(
        0, output.cloud_W->size() - 1);
    const int sample_count =
        std::min<int>(debug_sample_count_, output.cloud_W->size());
    for (int sample = 0; sample < sample_count; ++sample) {
      const std::size_t retained_index = distribution(random_engine_);
      const std::size_t input_index =
          output.retained_input_indices[retained_index];
      const pcl::PointXYZ& q_L_pcl = cloud_L.points[input_index];
      const Eigen::Vector3d q_L(q_L_pcl.x, q_L_pcl.y, q_L_pcl.z);
      // Reconstruct the synchronized poses from the already transformed values
      // is intentionally avoided; all four points were computed in process().
      const pcl::PointXYZ& q_W_pcl = output.cloud_W->points[retained_index];
      const pcl::PointXYZ& q_N_pcl = output.cloud_N->points[retained_index];
      const Eigen::Vector3d q_B =
          preprocessor_->config().rotation_BL * q_L +
          preprocessor_->config().translation_BL;
      ROS_INFO_STREAM("M1 sample input_index=" << input_index
                      << " q^L=" << q_L.transpose()
                      << " q^B=" << q_B.transpose()
                      << " q^W=" << q_W_pcl.x << " " << q_W_pcl.y << " "
                      << q_W_pcl.z << " q^N=" << q_N_pcl.x << " "
                      << q_N_pcl.y << " " << q_N_pcl.z
                      << " dt_quad=" << quad_dt << " dt_car=" << car_dt);
      if (debug_cloud_matches && input_index < debug_cloud_W.size()) {
        const pcl::PointXYZ& q_W_ground_truth =
            debug_cloud_W.points[input_index];
        const Eigen::Vector3d delta_W(
            q_W_pcl.x - q_W_ground_truth.x,
            q_W_pcl.y - q_W_ground_truth.y,
            q_W_pcl.z - q_W_ground_truth.z);
        ROS_INFO_STREAM("M1 debug-only MARSIM /cloud comparison: q^W_gt="
                        << q_W_ground_truth.x << " " << q_W_ground_truth.y
                        << " " << q_W_ground_truth.z
                        << " error_norm=" << delta_W.norm());
      }
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> quad_odom_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> car_odom_sub_;
  std::unique_ptr<Synchronizer> synchronizer_;
  std::unique_ptr<PointCloudPreprocessor> preprocessor_;

  ros::Publisher cloud_W_pub_;
  ros::Publisher cloud_N_pub_;
  ros::Publisher local_cloud_N_pub_;
  ros::Publisher query_center_N_pub_;
  ros::Publisher sync_diagnostics_pub_;
  ros::Subscriber debug_world_cloud_sub_;
  sensor_msgs::PointCloud2ConstPtr latest_debug_cloud_W_msg_;

  double max_pose_time_offset_sec_ = 0.025;
  int debug_sample_count_ = 3;
  int debug_print_every_n_clouds_ = 20;
  bool debug_compare_world_cloud_ = true;
  std::string world_frame_id_ = "map";
  std::string noninertial_frame_id_ = "base_link";
  std::size_t cloud_count_ = 0;
  std::mt19937 random_engine_;
};

}  // namespace depth_cbf
}  // namespace coni_mpc

int main(int argc, char** argv) {
  ros::init(argc, argv, "marsim_cloud_preprocessor");
  try {
    coni_mpc::depth_cbf::MarsimCloudPreprocessorNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL_STREAM("Failed to start M1 cloud preprocessor: "
                     << exception.what());
    return 1;
  }
  return 0;
}
