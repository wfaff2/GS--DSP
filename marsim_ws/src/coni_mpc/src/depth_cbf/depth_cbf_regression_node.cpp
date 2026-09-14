#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include "coni_mpc/LocalBarrier.h"
#include "coni_mpc/depth_cbf/depth_cbf_regressor.h"

namespace coni_mpc {
namespace depth_cbf {
namespace {

geometry_msgs::Point toPoint(const Eigen::Vector3d& value) {
  geometry_msgs::Point point_N_msg;
  point_N_msg.x = value.x();
  point_N_msg.y = value.y();
  point_N_msg.z = value.z();
  return point_N_msg;
}

visualization_msgs::Marker makeMarker(const std_msgs::Header& header,
                                      const std::string& name_space,
                                      int id, int type) {
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = name_space;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 0.9;
  return marker;
}

}  // namespace

class DepthCbfRegressionNode {
 public:
  DepthCbfRegressionNode()
      : nh_(),
        private_nh_("~"),
        cloud_sub_(nh_, readString(
                            "regression/local_cloud_topic",
                            "/depth_cbf_cloud_preprocessor/"
                            "local_cloud_noninertial"),
                   readInt("regression/subscriber_queue_size", 5)),
        center_sub_(nh_, readString(
                             "regression/query_center_topic",
                             "/depth_cbf_cloud_preprocessor/"
                             "query_center_noninertial"),
                    readInt("regression/subscriber_queue_size", 5)) {
    const RegressionConfig config = loadConfig();
    std::string error;
    if (!DepthCbfRegressor::validateConfig(config, &error)) {
      ROS_FATAL_STREAM("Depth-CBF regression parameter error: " << error);
      throw std::runtime_error(error);
    }
    regressor_.reset(new DepthCbfRegressor(config));
    private_nh_.param("noninertial_frame_id", noninertial_frame_id_,
                      std::string("base_link"));
    private_nh_.param("regression/log_every_n_fits", log_every_n_fits_, 20);

    barrier_pub_ = private_nh_.advertise<coni_mpc::LocalBarrier>("barrier", 2);
    marker_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>(
        "markers", 2);

    const int sync_queue = readInt("regression/sync_queue_size", 10);
    synchronizer_.reset(
        new Synchronizer(SyncPolicy(sync_queue), cloud_sub_, center_sub_));
    synchronizer_->registerCallback(
        boost::bind(&DepthCbfRegressionNode::callback, this, _1, _2));

    ROS_INFO_STREAM(
        "M2 Depth-CBF regression ready. Input is M1 local cloud derived from "
        "/quad0_pcl_render_node/sensor_cloud. L="
        << config.query_mesh_half_width_steps
        << ", epsilon=" << config.query_mesh_spacing
        << " m, d_safe=" << config.d_safe
        << " m (surface-point convention). It is not connected to ACADO.");
  }

 private:
  using SyncPolicy = message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2, geometry_msgs::PointStamped>;
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

  RegressionConfig loadConfig() {
    RegressionConfig config;
    private_nh_.param("query_mesh/half_width_steps",
                      config.query_mesh_half_width_steps,
                      config.query_mesh_half_width_steps);
    private_nh_.param("query_mesh/spacing", config.query_mesh_spacing,
                      config.query_mesh_spacing);
    private_nh_.param("query_mesh/nearest_neighbor_max_distance",
                      config.nearest_neighbor_max_distance,
                      config.nearest_neighbor_max_distance);
    private_nh_.param("safety/d_safe", config.d_safe, config.d_safe);
    private_nh_.param("regression/relative_rank_tolerance",
                      config.relative_rank_tolerance,
                      config.relative_rank_tolerance);
    private_nh_.param("regression/max_condition_number",
                      config.max_condition_number,
                      config.max_condition_number);
    private_nh_.param("regression/max_rmse", config.max_rmse,
                      config.max_rmse);
    private_nh_.param("regression/max_abs_error", config.max_abs_error,
                      config.max_abs_error);
    return config;
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                const geometry_msgs::PointStampedConstPtr& center_msg) {
    PointCloud cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    const Eigen::Vector3d center_N(center_msg->point.x, center_msg->point.y,
                                   center_msg->point.z);

    LocalBarrier barrier;
    RegressionDiagnostics diagnostics;
    std::string error;
    const auto begin = std::chrono::steady_clock::now();
    const bool success = regressor_->fit(cloud, center_N,
                                         cloud_msg->header.stamp, &barrier,
                                         &diagnostics, &error);
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin)
            .count();
    max_fit_ms_ = std::max(max_fit_ms_, elapsed_ms);
    mean_fit_ms_ += (elapsed_ms - mean_fit_ms_) /
                    static_cast<double>(fit_count_ + 1);
    ++fit_count_;

    publishBarrier(barrier, diagnostics.query_points_N.size());
    publishMarkers(cloud_msg->header, barrier, diagnostics);

    if (!success) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "M2 invalid barrier at stamp="
                   << cloud_msg->header.stamp << ": " << error
                   << ". No stale barrier is retained.");
      return;
    }

    bool center_supported = false;
    Eigen::Vector3d nearest_center_N;
    const double raw_center = regressor_->rawBarrierValue(
        cloud, center_N, &nearest_center_N, &center_supported);
    if (log_every_n_fits_ > 0 &&
        fit_count_ % static_cast<std::size_t>(log_every_n_fits_) == 0) {
      ROS_INFO_STREAM(
          "M2 fit stamp=" << barrier.stamp << " valid=" << barrier.valid
                          << " queries="
                          << diagnostics.query_points_N.size()
                          << " h_raw(center)=" << raw_center
                          << " h_fit(center)=" << barrier.evaluate(center_N)
                          << " grad(center)="
                          << barrier.gradient(center_N).transpose()
                          << " rmse=" << barrier.rmse
                          << " max_abs_error=" << barrier.max_abs_error
                          << " cond=" << barrier.condition_number
                          << " fit_ms=" << elapsed_ms
                          << " mean_ms=" << mean_fit_ms_
                          << " max_ms=" << max_fit_ms_);
    }
  }

  void publishBarrier(const LocalBarrier& barrier,
                      std::size_t query_count) {
    coni_mpc::LocalBarrier message;
    message.header.stamp = barrier.stamp;
    message.header.frame_id = noninertial_frame_id_;
    message.query_center_N = toPoint(barrier.center_N);
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        message.A_row_major[3 * row + column] = barrier.A(row, column);
      }
    }
    message.b.x = barrier.b.x();
    message.b.y = barrier.b.y();
    message.b.z = barrier.b.z();
    message.c = barrier.c;
    message.rmse = barrier.rmse;
    message.max_abs_error = barrier.max_abs_error;
    message.condition_number = barrier.condition_number;
    message.rank = barrier.rank;
    message.min_singular_value = barrier.min_singular_value;
    message.max_singular_value = barrier.max_singular_value;
    message.query_count = static_cast<uint32_t>(query_count);
    message.valid = barrier.valid;
    barrier_pub_.publish(message);
  }

  void publishMarkers(const std_msgs::Header& input_header,
                      const LocalBarrier& barrier,
                      const RegressionDiagnostics& diagnostics) {
    std_msgs::Header header = input_header;
    header.frame_id = noninertial_frame_id_;
    visualization_msgs::MarkerArray array;

    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    clear.header = header;
    array.markers.push_back(clear);

    auto center = makeMarker(header, "query_center", 0,
                             visualization_msgs::Marker::SPHERE);
    center.pose.position = toPoint(barrier.center_N);
    center.scale.x = center.scale.y = center.scale.z = 0.12;
    center.color.r = barrier.valid ? 0.1 : 1.0;
    center.color.g = barrier.valid ? 1.0 : 0.1;
    center.color.b = 0.1;
    array.markers.push_back(center);

    if (barrier.valid) {
      auto queries = makeMarker(header, "query_mesh", 1,
                                visualization_msgs::Marker::SPHERE_LIST);
      queries.scale.x = queries.scale.y = queries.scale.z = 0.035;
      queries.color.r = 1.0;
      queries.color.g = 0.85;
      queries.color.b = 0.0;
      auto nearest = makeMarker(header, "nearest_surface", 2,
                                visualization_msgs::Marker::SPHERE_LIST);
      nearest.scale.x = nearest.scale.y = nearest.scale.z = 0.045;
      nearest.color.r = 0.0;
      nearest.color.g = 0.7;
      nearest.color.b = 1.0;
      auto links = makeMarker(header, "nearest_links", 3,
                              visualization_msgs::Marker::LINE_LIST);
      links.scale.x = 0.007;
      links.color.r = 0.6;
      links.color.g = 0.6;
      links.color.b = 0.6;
      links.color.a = 0.4;
      for (std::size_t i = 0; i < diagnostics.query_points_N.size(); ++i) {
        const geometry_msgs::Point query =
            toPoint(diagnostics.query_points_N[i]);
        const geometry_msgs::Point surface =
            toPoint(diagnostics.nearest_points_N[i]);
        queries.points.push_back(query);
        nearest.points.push_back(surface);
        links.points.push_back(query);
        links.points.push_back(surface);
      }
      array.markers.push_back(queries);
      array.markers.push_back(nearest);
      array.markers.push_back(links);

      auto current_nearest = makeMarker(
          header, "current_nearest_surface", 4,
          visualization_msgs::Marker::SPHERE);
      current_nearest.scale.x = current_nearest.scale.y =
          current_nearest.scale.z = 0.10;
      current_nearest.color.r = 1.0;
      current_nearest.color.g = 0.15;
      current_nearest.color.b = 0.05;
      const auto center_query = std::min_element(
          diagnostics.query_points_N.begin(),
          diagnostics.query_points_N.end(),
          [&barrier](const Eigen::Vector3d& lhs_N,
                     const Eigen::Vector3d& rhs_N) {
            return (lhs_N - barrier.center_N).squaredNorm() <
                   (rhs_N - barrier.center_N).squaredNorm();
          });
      if (center_query != diagnostics.query_points_N.end()) {
        const std::size_t center_index = static_cast<std::size_t>(
            std::distance(diagnostics.query_points_N.begin(), center_query));
        current_nearest.pose.position =
            toPoint(diagnostics.nearest_points_N[center_index]);
        array.markers.push_back(current_nearest);
      }

      const Eigen::Vector3d gradient = barrier.gradient(barrier.center_N);
      if (gradient.norm() > 1e-9) {
        auto arrow = makeMarker(header, "gradient", 5,
                                visualization_msgs::Marker::ARROW);
        arrow.scale.x = 0.035;
        arrow.scale.y = 0.07;
        arrow.scale.z = 0.10;
        arrow.color.r = 1.0;
        arrow.color.g = 0.1;
        arrow.color.b = 0.8;
        arrow.points.push_back(toPoint(barrier.center_N));
        arrow.points.push_back(
            toPoint(barrier.center_N + 0.5 * gradient.normalized()));
        array.markers.push_back(arrow);
      }
    }
    marker_pub_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Subscriber<geometry_msgs::PointStamped> center_sub_;
  std::unique_ptr<Synchronizer> synchronizer_;
  std::unique_ptr<DepthCbfRegressor> regressor_;
  ros::Publisher barrier_pub_;
  ros::Publisher marker_pub_;
  std::string noninertial_frame_id_;
  int log_every_n_fits_ = 20;
  std::size_t fit_count_ = 0;
  double mean_fit_ms_ = 0.0;
  double max_fit_ms_ = 0.0;
};

}  // namespace depth_cbf
}  // namespace coni_mpc

int main(int argc, char** argv) {
  ros::init(argc, argv, "depth_cbf_regression");
  try {
    coni_mpc::depth_cbf::DepthCbfRegressionNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL_STREAM("Depth-CBF regression startup failed: "
                     << exception.what());
    return 1;
  }
  return 0;
}
