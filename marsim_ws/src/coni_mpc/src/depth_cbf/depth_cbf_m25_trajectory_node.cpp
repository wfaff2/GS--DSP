#include <algorithm>
#include <cmath>
#include <string>

#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace {

double smoothStep(double value) {
  value = std::max(0.0, std::min(1.0, value));
  return value * value * (3.0 - 2.0 * value);
}

class TrajectoryPublisher {
 public:
  TrajectoryPublisher() : nh_(), private_nh_("~") {
    private_nh_.param("scene", scene_, std::string("A1_plane"));
    private_nh_.param("motion", motion_, std::string("front"));
    private_nh_.param("duration_sec", duration_sec_, 8.0);
    private_nh_.param("warmup_sec", warmup_sec_, 2.0);
    private_nh_.param("rate_hz", rate_hz_, 100.0);
    private_nh_.param("z", z_, 1.5);
    private_nh_.param("yaw_offset_deg", yaw_offset_deg_, 0.0);
    private_nh_.param("quad_odom_topic", quad_topic_,
                      std::string("/coni_mpc/quad_odom1"));
    private_nh_.param("car_odom_topic", car_topic_,
                      std::string("/coni_mpc/car_odom"));
    quad_pub_ = nh_.advertise<nav_msgs::Odometry>(quad_topic_, 20);
    car_pub_ = nh_.advertise<nav_msgs::Odometry>(car_topic_, 20);
    start_ = ros::WallTime::now();
    timer_ = nh_.createWallTimer(ros::WallDuration(1.0 / rate_hz_),
                                 &TrajectoryPublisher::tick, this);
  }

 private:
  void pose(double u, Eigen::Vector3d* p, double* yaw) const {
    const double s = smoothStep(u);
    *p = Eigen::Vector3d(2.8, 0.0, z_);
    *yaw = 0.0;
    if (scene_ == "A1_plane") {
      if (motion_ == "parallel") {
        *p = Eigen::Vector3d(3.55, -1.5 + 3.0 * s, z_);
      } else if (motion_ == "oblique") {
        *p = Eigen::Vector3d(2.8 + 0.65 * s, -1.3 + 2.6 * s, z_);
      } else if (motion_ == "blind") {
        *p = Eigen::Vector3d(3.55 + 1.15 * s, 0.0, z_);
      } else if (motion_ == "fov") {
        *p = Eigen::Vector3d(3.55, 0.0, z_);
        *yaw = -1.05 + 2.10 * s;
      } else if (motion_ == "switching") {
        *p = Eigen::Vector3d(3.45, -1.5 + 3.0 * s, z_);
        *yaw = 0.22 * std::sin(4.0 * M_PI * s);
      } else {
        *p = Eigen::Vector3d(2.8 + 0.65 * s, 0.0, z_);
      }
    } else if (scene_ == "A2_cylinder") {
      if (motion_ == "fov") {
        *p = Eigen::Vector3d(3.0, 0.0, z_);
        *yaw = -1.40 + 2.80 * s;
      } else if (motion_ == "parallel") {
        *p = Eigen::Vector3d(3.2, -1.6 + 3.2 * s, z_);
        *yaw = std::atan2(-p->y(), 5.0 - p->x());
      } else if (motion_ == "oblique") {
        *p = Eigen::Vector3d(2.7 + 0.7 * s, -1.3 + 1.8 * s, z_);
        *yaw = std::atan2(-p->y(), 5.0 - p->x());
      } else if (motion_ == "blind") {
        *p = Eigen::Vector3d(3.0 + 1.65 * s, 0.0, z_);
      } else if (motion_ == "switching") {
        *p = Eigen::Vector3d(3.15, -1.4 + 2.8 * s, z_);
        *yaw = std::atan2(-p->y(), 5.0 - p->x());
      } else if (motion_ == "switching") {
        *p = Eigen::Vector3d(2.8 + 0.65 * s, 3.45 - 0.65 * s, z_);
        *yaw = M_PI / 4.0;
      } else {
        *p = Eigen::Vector3d(2.7 + 0.7 * s, 0.0, z_);
      }
    } else if (scene_ == "A3_corner") {
      if (motion_ == "parallel") {
        *p = Eigen::Vector3d(3.5, 2.7 + 0.75 * s, z_);
      } else if (motion_ == "front") {
        *p = Eigen::Vector3d(2.7 + 0.75 * s, 3.5, z_);
      } else {
        *p = Eigen::Vector3d(2.7 + 0.75 * s, 2.7 + 0.75 * s, z_);
        *yaw = M_PI / 4.0;
      }
    } else if (scene_ == "A4_switch") {
      *p = Eigen::Vector3d(2.8, -3.0 + 6.0 * s, z_);
      *yaw = std::atan2(-p->y(), 5.0 - p->x());
      if (motion_ == "oblique") p->x() += 0.45 * s;
      if (motion_ == "front") {
        *p = Eigen::Vector3d(2.7 + 0.6 * s, -1.5, z_);
        *yaw = 0.0;
      } else if (motion_ == "parallel") {
        *p = Eigen::Vector3d(3.35, -3.0 + 6.0 * s, z_);
        *yaw = std::atan2(-p->y(), 5.0 - p->x());
      }
    } else {
      // Repeatable small_forest segment within the Mid-360 sensing range.
      *p = Eigen::Vector3d(-0.5 + 1.0 * s, -5.0, z_);
      *yaw = -M_PI / 4.0;
    }
    *yaw += yaw_offset_deg_ * M_PI / 180.0;
  }

  nav_msgs::Odometry makeOdom(const ros::Time& stamp,
                              const Eigen::Vector3d& p, double yaw,
                              const std::string& child) const {
    nav_msgs::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "world";
    msg.child_frame_id = child;
    msg.pose.pose.position.x = p.x();
    msg.pose.pose.position.y = p.y();
    msg.pose.pose.position.z = p.z();
    const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    return msg;
  }

  void tick(const ros::WallTimerEvent&) {
    const double elapsed = (ros::WallTime::now() - start_).toSec();
    const double active = std::max(0.0, elapsed - warmup_sec_);
    const double u = duration_sec_ > 0.0 ? active / duration_sec_ : 1.0;
    Eigen::Vector3d p;
    double yaw = 0.0;
    pose(u, &p, &yaw);
    const ros::Time stamp = ros::Time::now();
    quad_pub_.publish(makeOdom(stamp, p, yaw, "quad0"));
    car_pub_.publish(makeOdom(stamp, Eigen::Vector3d::Zero(), 0.0, "ugv"));
    if (elapsed > warmup_sec_ + duration_sec_ + 0.5) ros::shutdown();
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher quad_pub_, car_pub_;
  ros::WallTimer timer_;
  ros::WallTime start_;
  std::string scene_, motion_, quad_topic_, car_topic_;
  double duration_sec_ = 8.0, warmup_sec_ = 2.0, rate_hz_ = 100.0;
  double z_ = 1.5, yaw_offset_deg_ = 0.0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "depth_cbf_m25_trajectory");
  TrajectoryPublisher node;
  ros::spin();
  return 0;
}
