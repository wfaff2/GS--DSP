#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>

namespace coni_mpc {

// Display history only. Appending or dropping the oldest sample is O(1).
// Materialize a ROS Path only when a subscribed display is due to update.
class BoundedPathHistory {
 public:
  explicit BoundedPathHistory(std::size_t capacity = 5000)
      : capacity_(std::max<std::size_t>(1, capacity)) {}
  void setCapacity(std::size_t capacity) {
    capacity_ = std::max<std::size_t>(1, capacity);
    while (poses_.size() > capacity_) poses_.pop_front();
  }
  void append(const geometry_msgs::PoseStamped& pose) {
    if (poses_.size() == capacity_) poses_.pop_front();
    poses_.push_back(pose);
  }
  void fill(nav_msgs::Path& path) const {
    path.poses.assign(poses_.begin(), poses_.end());
  }
  const std::deque<geometry_msgs::PoseStamped>& poses() const { return poses_; }

 private:
  std::size_t capacity_;
  std::deque<geometry_msgs::PoseStamped> poses_;
};

class PathVisualizationRate {
 public:
  void setPeriod(double seconds) { period_ = seconds; }
  // The caller supplies monotonic wall time; no change to simulation time.
  bool due(double now) {
    if (!std::isfinite(now)) return false;
    if (!initialized_ || now < last_ || now - last_ + 1e-9 >= period_) {
      initialized_ = true;
      last_ = now;
      return true;
    }
    return false;
  }

 private:
  double period_ = 0.1;
  double last_ = 0.0;
  bool initialized_ = false;
};

}  // namespace coni_mpc
