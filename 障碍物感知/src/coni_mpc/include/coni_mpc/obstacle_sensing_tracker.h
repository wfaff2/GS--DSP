#ifndef CONI_MPC_OBSTACLE_SENSING_TRACKER_H
#define CONI_MPC_OBSTACLE_SENSING_TRACKER_H

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace coni_mpc {

// ROS-independent target-level obstacle sensor and static-target tracker.
// Keeping this module independent makes the sensing assumptions directly
// testable without constructing the MPC controller or starting a ROS master.
class ObstacleSensingTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Obstacle = Eigen::Matrix<double, 7, 1>;
  using ObstacleVector =
      std::vector<Obstacle, Eigen::aligned_allocator<Obstacle>>;

  struct Config {
    bool online = false;
    double range_min_m = 0.15;
    double range_max_m = 6.0;
    double horizontal_fov_rad = 2.0 * 3.14159265358979323846;
    double angular_resolution_rad =
        0.9 * 3.14159265358979323846 / 180.0;
    std::size_t minimum_hit_rays = 3;
    double control_rate_hz = 100.0;
    double scan_rate_hz = 10.0;
    double position_std_m = 0.01;
    double dropout_probability = 0.0;
    double delay_sec = 0.1;
    double hold_sec = 0.3;
    double kalman_process_variance = 1e-6;
    std::uint64_t experiment_seed = 1u;
    std::uint32_t uav_id = 0u;
  };

  struct Measurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::size_t obstacle_id = 0;
    std::uint64_t generation_scan_index = 0;
    std::uint64_t generation_cycle = 0;
    std::uint64_t delivery_cycle = 0;
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    double z = 0.0;
    double radius = 0.0;
  };

  struct TrackSnapshot {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::size_t obstacle_id = 0;
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    Eigen::Vector2d last_measurement_position =
        Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector2d last_innovation =
        Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix2d last_innovation_covariance =
        Eigen::Matrix2d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix2d last_kalman_gain =
        Eigen::Matrix2d::Constant(std::numeric_limits<double>::quiet_NaN());
    double z = 0.0;
    double radius = 0.0;
    std::uint64_t last_generation_scan_index = 0;
    std::uint64_t last_generation_cycle = 0;
    std::uint64_t last_delivery_cycle = 0;
    std::uint64_t last_update_cycle = 0;
    // Kept for backward compatibility.  This is the tracker receipt/update
    // cycle, not the physical generation time of a delayed measurement.
    std::uint64_t last_received_cycle = 0;
    std::uint64_t track_instance = 0;
    std::uint64_t measurement_update_count = 0;
    bool last_innovation_valid = false;
    bool created_this_cycle = false;
    bool updated_this_cycle = false;
    bool reinitialized_this_cycle = false;
  };

  struct StepStats {
    bool scan_triggered = false;
    std::uint64_t scan_index = 0;
    std::size_t cast_rays = 0;
    std::size_t truth_obstacles = 0;
    std::size_t visible_obstacles = 0;
    std::size_t generated_measurements = 0;
    std::size_t dropped_measurements = 0;
    std::size_t delivered_measurements = 0;
    std::size_t created_tracks = 0;
    std::size_t reinitialized_tracks = 0;
    std::size_t kalman_corrections = 0;
    std::size_t deleted_tracks = 0;
    std::size_t active_tracks = 0;
  };

  struct StepResult {
    ObstacleVector obstacles;
    std::vector<std::string> obstacle_keys;
    StepStats stats;
    std::vector<Measurement, Eigen::aligned_allocator<Measurement>>
        generated_measurements;
    std::vector<Measurement, Eigen::aligned_allocator<Measurement>>
        delivered_measurements;
    std::vector<std::size_t> visible_obstacle_ids;
    std::vector<std::size_t> first_return_hit_counts;
    std::vector<std::size_t> dropped_obstacle_ids;
    std::vector<std::size_t> pending_obstacle_ids;
    std::vector<TrackSnapshot, Eigen::aligned_allocator<TrackSnapshot>> tracks;
  };

  ObstacleSensingTracker();
  explicit ObstacleSensingTracker(const Config& config);

  void configure(const Config& config);
  void reset();
  StepResult step(const ObstacleVector& truth_obstacles,
                  const Eigen::Vector3d& uav_position_world,
                  double body_yaw_world_rad,
                  const std::vector<double>& obstacle_heights_m = {});

  const Config& config() const { return config_; }
  std::uint64_t cycle() const { return cycle_; }
  std::uint64_t scanIndex() const { return scan_index_; }
  std::uint64_t scanPeriodControlCycles() const {
    return scan_period_control_cycles_;
  }
  std::uint64_t delayControlCycles() const { return delay_control_cycles_; }
  std::uint64_t holdControlCycles() const { return hold_control_cycles_; }

 private:
  struct Track {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool active = false;
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    Eigen::Vector2d last_measurement_position =
        Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector2d last_innovation =
        Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix2d last_innovation_covariance =
        Eigen::Matrix2d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix2d last_kalman_gain =
        Eigen::Matrix2d::Constant(std::numeric_limits<double>::quiet_NaN());
    double z = 0.0;
    double radius = 0.0;
    std::uint64_t last_generation_scan_index = 0;
    std::uint64_t last_generation_cycle = 0;
    std::uint64_t last_delivery_cycle = 0;
    std::uint64_t last_update_cycle = 0;
    std::uint64_t last_received_cycle = 0;
    std::uint64_t track_instance = 0;
    std::uint64_t measurement_update_count = 0;
    bool last_innovation_valid = false;
    bool created_this_cycle = false;
    bool updated_this_cycle = false;
    bool reinitialized_this_cycle = false;
  };

  using TrackVector = std::vector<Track, Eigen::aligned_allocator<Track>>;
  using MeasurementQueue =
      std::deque<Measurement, Eigen::aligned_allocator<Measurement>>;

  Config config_;
  std::uint64_t cycle_ = 0;
  std::uint64_t scan_index_ = 0;
  std::uint64_t scan_period_control_cycles_ = 10;
  std::uint64_t delay_control_cycles_ = 10;
  std::uint64_t hold_control_cycles_ = 30;
  MeasurementQueue pending_measurements_;
  TrackVector tracks_;
};

}  // namespace coni_mpc

#endif  // CONI_MPC_OBSTACLE_SENSING_TRACKER_H
