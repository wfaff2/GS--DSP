#pragma once

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <stdexcept>

namespace num_sim {

// The scheduler is the sole writer. Wall time is used only to pace/wait, never
// to advance this timeline. A nonzero epoch also works with source-stamp checks.
class SimulationClock {
 public:
  SimulationClock(ros::NodeHandle& node, bool enabled) : enabled_(enabled) {
    if (!enabled_) return;
    if (!ros::Time::isSimTime()) {
      throw std::runtime_error("sim_publish_clock requires /use_sim_time=true");
    }
    publisher_ = node.advertise<rosgraph_msgs::Clock>("/clock", 1, true);
    publish(ros::Time(1, 0));
  }

  bool enabled() const { return enabled_; }
  ros::Time now() const { return enabled_ ? stamp_ : ros::Time::now(); }

  void publish(const ros::Time& stamp) {
    if (!enabled_) return;
    if (stamp.isZero() || stamp < stamp_) {
      throw std::runtime_error("Simulation clock must be nonzero and monotone");
    }
    stamp_ = stamp;
    // Do not wait for our own /clock subscription before sampling local state.
    ros::Time::setNow(stamp_);
    rosgraph_msgs::Clock message;
    message.clock = stamp_;
    publisher_.publish(message);
  }

 private:
  bool enabled_;
  ros::Time stamp_;
  ros::Publisher publisher_;
};

}  // namespace num_sim
