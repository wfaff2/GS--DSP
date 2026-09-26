#include "coni_mpc/obstacle_sensing_tracker.h"

#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::uint64_t kNoiseXChannel = 0x6e6f6973655f7801ULL;
constexpr std::uint64_t kNoiseYChannel = 0x6e6f6973655f7902ULL;
constexpr std::uint64_t kDropoutChannel = 0x64726f706f757403ULL;

std::uint64_t splitMix64(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

std::uint64_t keyedHash(std::uint64_t experiment_seed,
                        std::uint32_t uav_id,
                        std::uint64_t scan_index,
                        std::size_t obstacle_id,
                        std::uint64_t channel,
                        std::uint64_t subchannel = 0) {
  std::uint64_t hash = splitMix64(experiment_seed);
  hash = splitMix64(hash ^ static_cast<std::uint64_t>(uav_id));
  hash = splitMix64(hash ^ scan_index);
  hash = splitMix64(hash ^ static_cast<std::uint64_t>(obstacle_id));
  hash = splitMix64(hash ^ channel);
  return splitMix64(hash ^ subchannel);
}

double uniformOpen01(std::uint64_t experiment_seed,
                     std::uint32_t uav_id,
                     std::uint64_t scan_index,
                     std::size_t obstacle_id,
                     std::uint64_t channel,
                     std::uint64_t subchannel = 0) {
  const std::uint64_t bits =
      keyedHash(experiment_seed, uav_id, scan_index, obstacle_id, channel,
                subchannel) >>
      11U;
  constexpr double kInverseTwoTo53 = 1.0 / 9007199254740992.0;
  return (static_cast<double>(bits) + 0.5) * kInverseTwoTo53;
}

double keyedStandardNormal(std::uint64_t experiment_seed,
                           std::uint32_t uav_id,
                           std::uint64_t scan_index,
                           std::size_t obstacle_id,
                           std::uint64_t channel) {
  const double u1 = uniformOpen01(experiment_seed, uav_id, scan_index,
                                  obstacle_id, channel, 0);
  const double u2 = uniformOpen01(experiment_seed, uav_id, scan_index,
                                  obstacle_id, channel, 1);
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}

bool scanPlaneIntersectsCylinder(
    const coni_mpc::ObstacleSensingTracker::Obstacle& obstacle,
    std::size_t obstacle_id,
    double sensor_z,
    const std::vector<double>& obstacle_heights_m) {
  if (obstacle_id >= obstacle_heights_m.size() ||
      !std::isfinite(obstacle_heights_m[obstacle_id]) ||
      obstacle_heights_m[obstacle_id] <= 0.0) {
    return true;
  }
  const double half_height = 0.5 * obstacle_heights_m[obstacle_id];
  const double lower_z = obstacle(2) - half_height;
  const double upper_z = obstacle(2) + half_height;
  return sensor_z >= lower_z && sensor_z <= upper_z;
}

bool rayCircleFirstIntersection(const Eigen::Vector2d& ray_origin,
                                const Eigen::Vector2d& ray_direction,
                                const Eigen::Vector2d& circle_center,
                                double circle_radius,
                                double& intersection_distance) {
  const Eigen::Vector2d delta = circle_center - ray_origin;
  const double projection = delta.dot(ray_direction);
  const double perpendicular_sq =
      std::max(0.0, delta.squaredNorm() - projection * projection);
  const double radius_sq = circle_radius * circle_radius;
  if (perpendicular_sq > radius_sq) {
    return false;
  }
  const double half_chord = std::sqrt(std::max(0.0, radius_sq - perpendicular_sq));
  const double near_distance = projection - half_chord;
  const double far_distance = projection + half_chord;
  if (far_distance < 0.0) {
    return false;
  }
  intersection_distance = near_distance >= 0.0 ? near_distance : far_distance;
  return true;
}

}  // namespace

namespace coni_mpc {

ObstacleSensingTracker::ObstacleSensingTracker() { configure(Config()); }

ObstacleSensingTracker::ObstacleSensingTracker(const Config& config) {
  configure(config);
}

void ObstacleSensingTracker::configure(const Config& config) {
  config_ = config;
  config_.range_min_m = std::max(0.0, config_.range_min_m);
  config_.range_max_m = std::max(config_.range_min_m, config_.range_max_m);
  config_.horizontal_fov_rad =
      std::max(0.0, std::min(2.0 * kPi,
                            config_.horizontal_fov_rad));
  if (!std::isfinite(config_.angular_resolution_rad) ||
      config_.angular_resolution_rad <= 0.0) {
    config_.angular_resolution_rad = 0.9 * kPi / 180.0;
  }
  config_.angular_resolution_rad =
      std::min(2.0 * kPi, config_.angular_resolution_rad);
  config_.minimum_hit_rays = std::max<std::size_t>(1, config_.minimum_hit_rays);
  if (!std::isfinite(config_.control_rate_hz) ||
      config_.control_rate_hz <= 0.0) {
    config_.control_rate_hz = 100.0;
  }
  if (!std::isfinite(config_.scan_rate_hz) || config_.scan_rate_hz <= 0.0) {
    config_.scan_rate_hz = 10.0;
  }
  config_.scan_rate_hz =
      std::min(config_.control_rate_hz, config_.scan_rate_hz);
  config_.position_std_m = std::max(0.0, config_.position_std_m);
  config_.dropout_probability =
      std::max(0.0, std::min(1.0, config_.dropout_probability));
  config_.delay_sec = std::max(0.0, config_.delay_sec);
  config_.hold_sec = std::max(0.0, config_.hold_sec);
  config_.kalman_process_variance =
      std::max(0.0, config_.kalman_process_variance);
  scan_period_control_cycles_ = std::max<std::uint64_t>(
      1, static_cast<std::uint64_t>(
             std::llround(config_.control_rate_hz / config_.scan_rate_hz)));
  delay_control_cycles_ = static_cast<std::uint64_t>(
      std::llround(config_.delay_sec * config_.control_rate_hz));
  hold_control_cycles_ = static_cast<std::uint64_t>(
      std::llround(config_.hold_sec * config_.control_rate_hz));
  reset();
}

void ObstacleSensingTracker::reset() {
  cycle_ = 0;
  scan_index_ = 0;
  pending_measurements_.clear();
  tracks_.clear();
}

ObstacleSensingTracker::StepResult ObstacleSensingTracker::step(
    const ObstacleVector& truth_obstacles,
    const Eigen::Vector3d& uav_position_world,
    double body_yaw_world_rad,
    const std::vector<double>& obstacle_heights_m) {
  StepResult result;
  result.stats.truth_obstacles = truth_obstacles.size();
  result.stats.scan_index = scan_index_;

  if (!config_.online) {
    result.obstacles = truth_obstacles;
    result.obstacle_keys.reserve(truth_obstacles.size());
    for (std::size_t obstacle_id = 0; obstacle_id < truth_obstacles.size();
         ++obstacle_id) {
      result.obstacle_keys.push_back("static:" +
                                     std::to_string(obstacle_id));
      TrackSnapshot snapshot;
      snapshot.obstacle_id = obstacle_id;
      snapshot.position = truth_obstacles[obstacle_id].head<2>();
      snapshot.z = truth_obstacles[obstacle_id](2);
      snapshot.radius = truth_obstacles[obstacle_id](3);
      result.tracks.push_back(snapshot);
    }
    result.stats.active_tracks = truth_obstacles.size();
    ++cycle_;
    return result;
  }

  if (tracks_.size() != truth_obstacles.size()) {
    tracks_.resize(truth_obstacles.size());
  }

  const std::uint64_t current_cycle = cycle_;
  const Eigen::Matrix2d process_covariance =
      config_.kalman_process_variance * Eigen::Matrix2d::Identity();
  const double measurement_variance = std::max(
      config_.position_std_m * config_.position_std_m, 1e-12);
  const Eigen::Matrix2d measurement_covariance =
      measurement_variance * Eigen::Matrix2d::Identity();

  for (auto& track : tracks_) {
    track.created_this_cycle = false;
    track.updated_this_cycle = false;
    track.reinitialized_this_cycle = false;
    if (track.active) {
      track.covariance += process_covariance;
    }
  }

  const bool scan_triggered =
      current_cycle % scan_period_control_cycles_ == 0;
  if (scan_triggered) {
    result.stats.scan_triggered = true;
    result.stats.scan_index = scan_index_;
    const Eigen::Vector2d sensor_xy = uav_position_world.head<2>();
    std::vector<bool> range_and_plane_candidate(truth_obstacles.size(), false);
    for (std::size_t obstacle_id = 0; obstacle_id < truth_obstacles.size();
         ++obstacle_id) {
      const Obstacle& truth = truth_obstacles[obstacle_id];
      const double radius = std::max(0.0, truth(3));
      const double surface_range =
          std::max(0.0, (truth.head<2>() - sensor_xy).norm() - radius);
      range_and_plane_candidate[obstacle_id] =
          surface_range >= config_.range_min_m &&
          surface_range <= config_.range_max_m &&
          scanPlaneIntersectsCylinder(truth, obstacle_id,
                                      uav_position_world.z(),
                                      obstacle_heights_m);
    }

    std::vector<std::size_t> hit_counts(truth_obstacles.size(), 0);
    const std::size_t ray_count = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::llround(
               config_.horizontal_fov_rad / config_.angular_resolution_rad)));
    result.stats.cast_rays = ray_count;
    const double first_ray_angle =
        body_yaw_world_rad - 0.5 * config_.horizontal_fov_rad;
    for (std::size_t ray_index = 0; ray_index < ray_count; ++ray_index) {
      const double ray_angle =
          first_ray_angle +
          static_cast<double>(ray_index) * config_.angular_resolution_rad;
      const Eigen::Vector2d ray_direction(std::cos(ray_angle),
                                          std::sin(ray_angle));
      std::size_t closest_obstacle_id = truth_obstacles.size();
      double closest_intersection = std::numeric_limits<double>::infinity();
      for (std::size_t obstacle_id = 0; obstacle_id < truth_obstacles.size();
           ++obstacle_id) {
        if (!range_and_plane_candidate[obstacle_id]) {
          continue;
        }
        double intersection_distance = 0.0;
        if (!rayCircleFirstIntersection(
                sensor_xy, ray_direction, truth_obstacles[obstacle_id].head<2>(),
                std::max(0.0, truth_obstacles[obstacle_id](3)),
                intersection_distance)) {
          continue;
        }
        if (intersection_distance < closest_intersection) {
          closest_intersection = intersection_distance;
          closest_obstacle_id = obstacle_id;
        }
      }
      if (closest_obstacle_id < hit_counts.size()) {
        ++hit_counts[closest_obstacle_id];
      }
    }
    result.first_return_hit_counts = hit_counts;

    for (std::size_t obstacle_id = 0; obstacle_id < truth_obstacles.size();
         ++obstacle_id) {
      if (hit_counts[obstacle_id] < config_.minimum_hit_rays) {
        continue;
      }
      const Obstacle& truth = truth_obstacles[obstacle_id];
      ++result.stats.visible_obstacles;
      result.visible_obstacle_ids.push_back(obstacle_id);
      const double dropout_draw = uniformOpen01(
          config_.experiment_seed, config_.uav_id, scan_index_, obstacle_id,
          kDropoutChannel);
      if (dropout_draw < config_.dropout_probability) {
        ++result.stats.dropped_measurements;
        result.dropped_obstacle_ids.push_back(obstacle_id);
        continue;
      }

      const double noise_x = config_.position_std_m * keyedStandardNormal(
          config_.experiment_seed, config_.uav_id, scan_index_, obstacle_id,
          kNoiseXChannel);
      const double noise_y = config_.position_std_m * keyedStandardNormal(
          config_.experiment_seed, config_.uav_id, scan_index_, obstacle_id,
          kNoiseYChannel);
      Measurement measurement;
      measurement.obstacle_id = obstacle_id;
      measurement.generation_scan_index = scan_index_;
      measurement.generation_cycle = current_cycle;
      measurement.delivery_cycle = current_cycle + delay_control_cycles_;
      measurement.position =
          truth.head<2>() + Eigen::Vector2d(noise_x, noise_y);
      measurement.z = truth(2);
      measurement.radius = truth(3);
      pending_measurements_.push_back(measurement);
      result.generated_measurements.push_back(measurement);
      ++result.stats.generated_measurements;
    }
    ++scan_index_;
  }

  while (!pending_measurements_.empty() &&
         pending_measurements_.front().delivery_cycle <= current_cycle) {
    const Measurement measurement = pending_measurements_.front();
    pending_measurements_.pop_front();
    if (measurement.obstacle_id >= tracks_.size()) {
      continue;
    }

    Track& track = tracks_[measurement.obstacle_id];
    if (!track.active) {
      const std::uint64_t previous_track_instances = track.track_instance;
      track = Track();
      track.active = true;
      track.position = measurement.position;
      track.covariance = measurement_covariance;
      track.last_measurement_position = measurement.position;
      track.z = measurement.z;
      track.radius = measurement.radius;
      track.last_generation_scan_index = measurement.generation_scan_index;
      track.last_generation_cycle = measurement.generation_cycle;
      track.last_delivery_cycle = measurement.delivery_cycle;
      track.last_update_cycle = current_cycle;
      track.last_received_cycle = current_cycle;
      track.track_instance = previous_track_instances + 1;
      track.measurement_update_count = 1;
      track.last_innovation_valid = false;
      track.created_this_cycle = true;
      track.updated_this_cycle = true;
      track.reinitialized_this_cycle = previous_track_instances > 0;
      ++result.stats.created_tracks;
      if (track.reinitialized_this_cycle) {
        ++result.stats.reinitialized_tracks;
      }
    } else {
      const Eigen::Vector2d innovation =
          measurement.position - track.position;
      const Eigen::Matrix2d innovation_covariance =
          track.covariance + measurement_covariance;
      const Eigen::Matrix2d kalman_gain =
          track.covariance * innovation_covariance.inverse();
      track.position += kalman_gain * innovation;
      track.covariance =
          (Eigen::Matrix2d::Identity() - kalman_gain) * track.covariance;
      track.covariance =
          0.5 * (track.covariance + track.covariance.transpose());
      track.last_measurement_position = measurement.position;
      track.last_innovation = innovation;
      track.last_innovation_covariance = innovation_covariance;
      track.last_kalman_gain = kalman_gain;
      track.z = measurement.z;
      track.radius = measurement.radius;
      track.last_generation_scan_index = measurement.generation_scan_index;
      track.last_generation_cycle = measurement.generation_cycle;
      track.last_delivery_cycle = measurement.delivery_cycle;
      track.last_update_cycle = current_cycle;
      track.last_received_cycle = current_cycle;
      ++track.measurement_update_count;
      track.last_innovation_valid = true;
      track.updated_this_cycle = true;
      ++result.stats.kalman_corrections;
    }
    result.delivered_measurements.push_back(measurement);
    ++result.stats.delivered_measurements;
  }

  for (Track& track : tracks_) {
    if (!track.active) {
      continue;
    }
    const std::uint64_t missed_cycles = current_cycle - track.last_received_cycle;
    if (missed_cycles > hold_control_cycles_) {
      const std::uint64_t track_instance = track.track_instance;
      track = Track();
      track.track_instance = track_instance;
      ++result.stats.deleted_tracks;
    }
  }

  std::vector<bool> has_pending_measurement(truth_obstacles.size(), false);
  for (const Measurement& measurement : pending_measurements_) {
    if (measurement.obstacle_id < has_pending_measurement.size()) {
      has_pending_measurement[measurement.obstacle_id] = true;
    }
  }
  for (std::size_t obstacle_id = 0;
       obstacle_id < has_pending_measurement.size(); ++obstacle_id) {
    if (has_pending_measurement[obstacle_id]) {
      result.pending_obstacle_ids.push_back(obstacle_id);
    }
  }

  result.obstacles.reserve(tracks_.size());
  result.obstacle_keys.reserve(tracks_.size());
  result.tracks.reserve(tracks_.size());
  for (std::size_t obstacle_id = 0; obstacle_id < tracks_.size();
       ++obstacle_id) {
    const Track& track = tracks_[obstacle_id];
    if (!track.active) {
      continue;
    }

    Obstacle obstacle = Obstacle::Zero();
    obstacle << track.position.x(), track.position.y(), track.z, track.radius,
        0.0, 0.0, 0.0;
    result.obstacles.push_back(obstacle);
    result.obstacle_keys.push_back("static:" + std::to_string(obstacle_id));

    TrackSnapshot snapshot;
    snapshot.obstacle_id = obstacle_id;
    snapshot.position = track.position;
    snapshot.covariance = track.covariance;
    snapshot.last_measurement_position = track.last_measurement_position;
    snapshot.last_innovation = track.last_innovation;
    snapshot.last_innovation_covariance =
        track.last_innovation_covariance;
    snapshot.last_kalman_gain = track.last_kalman_gain;
    snapshot.z = track.z;
    snapshot.radius = track.radius;
    snapshot.last_generation_scan_index = track.last_generation_scan_index;
    snapshot.last_generation_cycle = track.last_generation_cycle;
    snapshot.last_delivery_cycle = track.last_delivery_cycle;
    snapshot.last_update_cycle = track.last_update_cycle;
    snapshot.last_received_cycle = track.last_received_cycle;
    snapshot.track_instance = track.track_instance;
    snapshot.measurement_update_count = track.measurement_update_count;
    snapshot.last_innovation_valid = track.last_innovation_valid;
    snapshot.created_this_cycle = track.created_this_cycle;
    snapshot.updated_this_cycle = track.updated_this_cycle;
    snapshot.reinitialized_this_cycle = track.reinitialized_this_cycle;
    result.tracks.push_back(snapshot);
  }
  result.stats.active_tracks = result.obstacles.size();
  ++cycle_;
  return result;
}

}  // namespace coni_mpc
