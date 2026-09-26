#include "coni_mpc/risk_region_constructor.h"
#include "coni_mpc/RiskRegionArray.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

class RiskRegionNode {
 public:
  RiskRegionNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    pnh_.param("input_topic", input_topic_, std::string("/my_map/future_occupancy_3d"));
    pnh_.param("output_topic", output_topic_, std::string("/coni_mpc/risk_regions"));
    int prediction_stages = static_cast<int>(config_.prediction_stages);
    pnh_.param("prediction_stages", prediction_stages, prediction_stages);
    config_.prediction_stages = static_cast<std::uint32_t>(std::max(1, prediction_stages));
    pnh_.param("prediction_dt", config_.prediction_dt, config_.prediction_dt);
    pnh_.param("voxel_resolution", config_.voxel_resolution, config_.voxel_resolution);
    pnh_.param("occupancy_min", config_.occupancy_min, config_.occupancy_min);
    pnh_.param("minimum_component_mass", config_.minimum_component_mass,
               config_.minimum_component_mass);
    int minimum_component_voxels = static_cast<int>(config_.minimum_component_voxels);
    pnh_.param("minimum_component_voxels", minimum_component_voxels,
               minimum_component_voxels);
    config_.minimum_component_voxels =
        static_cast<std::size_t>(std::max(1, minimum_component_voxels));
    pnh_.param("retained_mass", config_.retained_mass, config_.retained_mass);
    pnh_.param("neighbor_support_fraction", config_.neighbor_support_fraction,
               config_.neighbor_support_fraction);
    pnh_.param("neighbor_support_weight", config_.neighbor_support_weight,
               config_.neighbor_support_weight);
    int minimum_persistent_stages =
        static_cast<int>(config_.minimum_persistent_stages);
    pnh_.param("minimum_persistent_stages", minimum_persistent_stages,
               minimum_persistent_stages);
    config_.minimum_persistent_stages = static_cast<std::size_t>(
        std::max(1, minimum_persistent_stages));
    pnh_.param("persistence_high_confidence", config_.persistence_high_confidence,
               config_.persistence_high_confidence);
    int temporal_confirmation_frames = 2;
    pnh_.param("temporal_confirmation_frames", temporal_confirmation_frames,
               temporal_confirmation_frames);
    temporal_confirmation_frames_ = static_cast<std::size_t>(
        std::max(1, temporal_confirmation_frames));
    pnh_.param("temporal_track_timeout", temporal_track_timeout_, 0.5);
    pnh_.param("max_input_age_sec", max_input_age_sec_, 0.75);
    pnh_.param("risk_radius", config_.risk_radius, config_.risk_radius);
    double uav_radius = 0.15;
    double control_margin = 0.35;
    double perception_margin = 0.0;
    pnh_.param("uav_radius", uav_radius, uav_radius);
    pnh_.param("control_margin", control_margin, control_margin);
    pnh_.param("perception_margin", perception_margin, perception_margin);
    if (!pnh_.hasParam("risk_radius")) {
      config_.risk_radius = std::max(0.0, uav_radius) +
                            std::max(0.0, control_margin) +
                            std::max(0.0, perception_margin);
    }
    pnh_.param("track_match_distance", config_.track_match_distance,
               config_.track_match_distance);
    pnh_.param("minimum_axis", config_.minimum_axis, config_.minimum_axis);
    pnh_.param("split_volume_ratio", config_.split_volume_ratio,
               config_.split_volume_ratio);
    pnh_.param("split_min_volume_reduction", config_.split_min_volume_reduction,
               config_.split_min_volume_reduction);
    int max_split_depth = static_cast<int>(config_.max_split_depth);
    pnh_.param("max_split_depth", max_split_depth, max_split_depth);
    config_.max_split_depth = static_cast<std::size_t>(std::max(0, max_split_depth));

    publisher_ = nh_.advertise<coni_mpc::RiskRegionArray>(output_topic_, 1, true);
    subscriber_ = nh_.subscribe(input_topic_, 1, &RiskRegionNode::callback, this);
    worker_thread_ = std::thread(&RiskRegionNode::workerLoop, this);
    ROS_INFO_STREAM("RiskRegionConstructor: " << input_topic_ << " -> " << output_topic_
                    << ", stages=" << config_.prediction_stages
                    << ", voxel_resolution=" << config_.voxel_resolution
                    << ", risk_radius=" << config_.risk_radius
                    << ", support_fraction=" << config_.neighbor_support_fraction
                    << ", support_weight=" << config_.neighbor_support_weight
                    << ", persistent_stages=" << config_.minimum_persistent_stages
                    << ", split_volume_ratio=" << config_.split_volume_ratio
                    << ", max_input_age_sec=" << max_input_age_sec_
                    << " (perception_margin=" << perception_margin << ")");
  }

  ~RiskRegionNode() {
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      stop_worker_ = true;
    }
    cloud_condition_.notify_one();
    if (worker_thread_.joinable()) worker_thread_.join();
  }

 private:
  struct TemporalTrack {
    std::uint32_t stable_id = 0;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    ros::Time stamp;
    std::size_t consecutive_hits = 0;
  };

  void applyTemporalPersistence(std::vector<coni_mpc::RiskRegionData>* regions,
                                const ros::Time& stamp) {
    if (regions == nullptr || regions->empty()) return;
    const ros::Time effective_stamp = stamp.isZero() ? ros::Time::now() : stamp;
    std::map<std::uint32_t, std::size_t> representative;
    for (std::size_t i = 0; i < regions->size(); ++i) {
      const auto& region = regions->at(i);
      if (!region.valid) continue;
      const auto found = representative.find(region.track_id);
      if (found == representative.end() ||
          region.stage_index < regions->at(found->second).stage_index) {
        representative[region.track_id] = i;
      }
    }

    std::vector<bool> used(temporal_tracks_.size(), false);
    std::map<std::uint32_t, std::uint32_t> stable_ids;
    std::map<std::uint32_t, bool> confirmed;
    for (const auto& entry : representative) {
      const std::uint32_t local_id = entry.first;
      const auto& representative_region = regions->at(entry.second);
      std::size_t best = temporal_tracks_.size();
      double best_distance = std::max(0.0, config_.track_match_distance);
      for (std::size_t i = 0; i < temporal_tracks_.size(); ++i) {
        if (used[i]) continue;
        if (temporal_track_timeout_ > 0.0 &&
            !temporal_tracks_[i].stamp.isZero() &&
            (effective_stamp - temporal_tracks_[i].stamp).toSec() >
                temporal_track_timeout_) {
          continue;
        }
        const double distance =
            (representative_region.center - temporal_tracks_[i].center).norm();
        if (distance < best_distance) {
          best_distance = distance;
          best = i;
        }
      }
      if (best == temporal_tracks_.size()) {
        TemporalTrack track;
        track.stable_id = next_stable_track_id_++;
        track.center = representative_region.center;
        track.stamp = effective_stamp;
        track.consecutive_hits = 1;
        temporal_tracks_.push_back(track);
        best = temporal_tracks_.size() - 1;
      } else {
        TemporalTrack& track = temporal_tracks_[best];
        track.center = representative_region.center;
        track.stamp = effective_stamp;
        ++track.consecutive_hits;
      }
      used.resize(temporal_tracks_.size(), false);
      used[best] = true;
      const TemporalTrack& track = temporal_tracks_[best];
      stable_ids[local_id] = track.stable_id;
      confirmed[local_id] =
          track.consecutive_hits >= temporal_confirmation_frames_ ||
          representative_region.max_probability >=
              config_.persistence_high_confidence;
    }

    for (auto& region : *regions) {
      const auto stable = stable_ids.find(region.track_id);
      if (stable == stable_ids.end()) {
        region.valid = false;
        continue;
      }
      const bool is_confirmed = confirmed[region.track_id];
      region.track_id = stable->second;
      region.valid = region.valid && is_confirmed;
    }

    temporal_tracks_.erase(
        std::remove_if(temporal_tracks_.begin(), temporal_tracks_.end(),
                       [&](const TemporalTrack& track) {
                         return temporal_track_timeout_ > 0.0 &&
                                !track.stamp.isZero() &&
                                (effective_stamp - track.stamp).toSec() >
                                    temporal_track_timeout_;
                       }),
        temporal_tracks_.end());
  }

  // The LiDAR/DSP callback must stay O(1): fitting can take longer than one
  // sensor period.  Keeping only the newest message prevents ROS callback
  // queue backlog and preserves the timestamp semantics used by the MPC
  // stage-alignment logic.
  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      latest_cloud_ = cloud;
    }
    cloud_condition_.notify_one();
  }

  void workerLoop() {
    while (true) {
      sensor_msgs::PointCloud2ConstPtr cloud;
      {
        std::unique_lock<std::mutex> lock(cloud_mutex_);
        cloud_condition_.wait(lock, [&]() {
          return stop_worker_ || static_cast<bool>(latest_cloud_);
        });
        if (stop_worker_ && !latest_cloud_) break;
        cloud = latest_cloud_;
        latest_cloud_.reset();
      }
      processCloud(cloud);
    }
  }

  void processCloud(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    const double input_age_sec =
        cloud->header.stamp.isZero()
            ? 0.0
            : (ros::Time::now() - cloud->header.stamp).toSec();
    // The fit can take longer than the LiDAR period.  Do not spend another
    // full fit on an already obsolete cloud sitting in the subscriber queue;
    // the queue size is one, so returning here lets the newest cloud be
    // processed next.  This is a latest-sample policy, not a change to the
    // DSP voxel/mass or 95% spatial algorithm.
    if (max_input_age_sec_ > 0.0 && input_age_sec > max_input_age_sec_) {
      ROS_WARN_STREAM_THROTTLE_NAMED(
          0.5, "risk_region_constructor_stale_input",
          "RiskRegionConstructor: dropping stale input age=" << input_age_sec
              << " s (limit=" << max_input_age_sec_ << " s)");
      return;
    }
    const ros::WallTime callback_start = ros::WallTime::now();
    try {
      std::vector<coni_mpc::RiskVoxel> voxels;
      voxels.reserve(static_cast<std::size_t>(cloud->width) * cloud->height);
      sensor_msgs::PointCloud2ConstIterator<float> x_it(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_it(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_it(*cloud, "z");
      sensor_msgs::PointCloud2ConstIterator<float> occupancy_it(*cloud, "occupancy");
      sensor_msgs::PointCloud2ConstIterator<float> time_it(*cloud, "prediction_time");
      sensor_msgs::PointCloud2ConstIterator<std::uint32_t> stage_it(*cloud, "stage_index");
      for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it, ++occupancy_it,
             ++time_it, ++stage_it) {
        coni_mpc::RiskVoxel voxel;
        voxel.position = Eigen::Vector3d(*x_it, *y_it, *z_it);
        voxel.weight = static_cast<double>(*occupancy_it);
        voxel.prediction_time = static_cast<double>(*time_it);
        voxel.stage_index = *stage_it;
        voxels.push_back(voxel);
      }

      std::vector<coni_mpc::RiskRegionData> regions =
          coni_mpc::constructRiskRegions(voxels, config_);
      applyTemporalPersistence(&regions, cloud->header.stamp);
      coni_mpc::RiskRegionArray message;
      message.header = cloud->header;
      message.regions.reserve(regions.size());
      for (const coni_mpc::RiskRegionData& region : regions) {
        coni_mpc::RiskRegion output;
        output.track_id = region.track_id;
        output.stage_index = region.stage_index;
        output.prediction_time = region.prediction_time;
        output.center.x = region.center.x();
        output.center.y = region.center.y();
        output.center.z = region.center.z();
        Eigen::Quaterniond q(region.orientation);
        q.normalize();
        output.orientation.w = q.w();
        output.orientation.x = q.x();
        output.orientation.y = q.y();
        output.orientation.z = q.z();
        output.semi_axes.x = region.semi_axes.x();
        output.semi_axes.y = region.semi_axes.y();
        output.semi_axes.z = region.semi_axes.z();
        output.velocity.x = region.velocity.x();
        output.velocity.y = region.velocity.y();
        output.velocity.z = region.velocity.z();
        output.acceleration.x = region.acceleration.x();
        output.acceleration.y = region.acceleration.y();
        output.acceleration.z = region.acceleration.z();
        output.valid = region.valid;
        message.regions.push_back(output);
      }
      publisher_.publish(message);
      const double callback_wall_sec =
          (ros::WallTime::now() - callback_start).toSec();
      ROS_WARN_STREAM_THROTTLE_NAMED(
          0.5, "risk_region_constructor_timing",
          "RiskRegionConstructor: input_points=" << voxels.size()
              << " output_regions=" << message.regions.size()
              << " callback_wall_sec=" << callback_wall_sec
              << " input_stamp_age_sec="
              << (ros::Time::now() - cloud->header.stamp).toSec());
    } catch (const std::exception& error) {
      ROS_ERROR_THROTTLE(2.0, "RiskRegionConstructor: invalid PointCloud2 fields: %s",
                         error.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::string input_topic_;
  std::string output_topic_;
  coni_mpc::RiskRegionConstructorConfig config_;
  std::size_t temporal_confirmation_frames_ = 2;
  double temporal_track_timeout_ = 0.5;
  double max_input_age_sec_ = 0.75;
  std::vector<TemporalTrack> temporal_tracks_;
  std::uint32_t next_stable_track_id_ = 0;
  std::mutex cloud_mutex_;
  std::condition_variable cloud_condition_;
  sensor_msgs::PointCloud2ConstPtr latest_cloud_;
  bool stop_worker_ = false;
  std::thread worker_thread_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "risk_region_constructor");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  RiskRegionNode node(nh, pnh);
  ros::spin();
  return 0;
}
