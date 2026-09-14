//
// Created by baozhe on 22-10-17.
//

/*    coni_mpc
 *    CoNi-MPC: Cooperative Non-inertial Frame Based Model Predictive Control
 *    Copyright (C) 2023 Baozhe Zhang, 
 *    Fast Lab, Huzhou Institute of Zhejiang University
 *  
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


// Reference: rpg_mpc (the original LICENSE is listed below)

/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <acado_mpc/mpc_common.h>

#include <utility>

namespace {
std::string g_relative_frame_id = "base_link";
std::string g_world_frame_id = "map";
std::string g_topic_prefix;

std::string stripOuterSlashes(std::string value) {
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

std::string sanitizeFrameId(std::string value, const std::string& fallback) {
  value = stripOuterSlashes(std::move(value));
  return value.empty() ? fallback : value;
}
}  // namespace

namespace acado_mpc_common
{
void configureRosNames(const std::string& world_frame_id,
                       const std::string& relative_frame_id,
                       const std::string& topic_prefix)
{
  g_world_frame_id = sanitizeFrameId(world_frame_id, "map");
  g_relative_frame_id = sanitizeFrameId(relative_frame_id, "base_link");
  g_topic_prefix = stripOuterSlashes(topic_prefix);
}

const std::string& relativeFrameId()
{
  return g_relative_frame_id;
}

const std::string& worldFrameId()
{
  return g_world_frame_id;
}

const std::string& topicPrefix()
{
  return g_topic_prefix;
}

std::string resolveTopicName(const std::string& topic_name)
{
  const std::string clean_topic = stripOuterSlashes(topic_name);
  if (g_topic_prefix.empty()) {
    return clean_topic;
  }
  if (clean_topic.empty()) {
    return "/" + g_topic_prefix;
  }
  return "/" + g_topic_prefix + "/" + clean_topic;
}

// Implementations
QuadRelativeEstimate::QuadRelativeEstimate() :
    position(Eigen::Vector3d::Zero()),
    velocity(Eigen::Vector3d::Zero()),
    orientation(Eigen::Quaterniond::Identity()),
    a_imu(Eigen::Vector3d(0.0, 0.0, 9.81)),
    omega_non(Eigen::Vector3d::Zero()),
    beta_non(Eigen::Vector3d::Zero())
{
}

QuadRelativeEstimate::QuadRelativeEstimate(const nav_msgs::Odometry &relative_estimate_msg,
                                           const Eigen::Vector3d &a_imu,
                                           const Eigen::Vector3d &omega_non,
                                           const Eigen::Vector3d &beta_non) :
    position(relative_estimate_msg.pose.pose.position.x,
             relative_estimate_msg.pose.pose.position.y,
             relative_estimate_msg.pose.pose.position.z),
    velocity(relative_estimate_msg.twist.twist.linear.x,
             relative_estimate_msg.twist.twist.linear.y,
             relative_estimate_msg.twist.twist.linear.z),
    orientation(relative_estimate_msg.pose.pose.orientation.w,
                relative_estimate_msg.pose.pose.orientation.x,
                relative_estimate_msg.pose.pose.orientation.y,
                relative_estimate_msg.pose.pose.orientation.z),
    a_imu(a_imu),
    omega_non(omega_non), beta_non(beta_non)
{
}

QuadRelativeEstimate::QuadRelativeEstimate(const Eigen::Vector3d &p,
                                           const Eigen::Vector3d &v,
                                           const Eigen::Quaterniond &q,
                                           const Eigen::Vector3d &a_imu,
                                           const Eigen::Vector3d &omega_non,
                                           const Eigen::Vector3d &beta_non) :
    position(p),
    velocity(v),
    orientation(q),
    a_imu(a_imu),
    omega_non(omega_non), beta_non(beta_non)
{
}



QuadRelativeEstimate::~QuadRelativeEstimate()
{
  //
}

RelativeTrajectoryPoint::RelativeTrajectoryPoint() :
    position(Eigen::Vector3d::Zero()),
    velocity(Eigen::Vector3d::Zero()),
    orientation(Eigen::Quaterniond::Identity()),
    a_imu_ref(Eigen::Vector3d(0.0, 0.0, 9.81)),
    omega_non_ref(Eigen::Vector3d::Zero()),
    beta_non_ref(Eigen::Vector3d::Zero())
{
  //
}

RelativeTrajectoryPoint::RelativeTrajectoryPoint(const Eigen::Vector3d &p,
                                                 const Eigen::Vector3d &v,
                                                 const Eigen::Quaterniond &q,
                                                 const acado_mpc_common::QuadRelativeEstimate &relative_estimate) :
    position(p),
    velocity(v),
    orientation(q),
    a_imu_ref(relative_estimate.a_imu),
    omega_non_ref(relative_estimate.omega_non),
    beta_non_ref(relative_estimate.beta_non)
{
}

RelativeTrajectoryPoint::RelativeTrajectoryPoint(const Eigen::Vector3d &p,
                                                 const Eigen::Vector3d &v,
                                                 const Eigen::Quaterniond &q,
                                                 const Eigen::Vector3d &a_imu,
                                                 const Eigen::Vector3d &omega_non,
                                                 const Eigen::Vector3d &beta_non) :
    position(p),
    velocity(v),
    orientation(q),
    a_imu_ref(a_imu),
    omega_non_ref(omega_non),
    beta_non_ref(beta_non)
{
}

RelativeTrajectoryPoint::~RelativeTrajectoryPoint()
{
}

RelativeTrajectory::RelativeTrajectory() :
    points()
{
  //
}

RelativeTrajectory::RelativeTrajectory(
    const acado_mpc_common::RelativeTrajectoryPoint &point) :
    points()
{
  points.push_back(point);
}

RelativeTrajectory::~RelativeTrajectory()
{
  //
}

nav_msgs::Path RelativeTrajectory::toRosPath() const
{
  nav_msgs::Path path_msg;
  ros::Time t = ros::Time::now();
  path_msg.header.stamp = t;
  path_msg.header.frame_id = relativeFrameId();

  geometry_msgs::PoseStamped pose;
  for (const auto &point : points) {
    pose.pose.position.x = point.position.x();
    pose.pose.position.y = point.position.y();
    pose.pose.position.z = point.position.z();
    pose.pose.orientation.w = point.orientation.w();
    pose.pose.orientation.x = point.orientation.x();
    pose.pose.orientation.y = point.orientation.y();
    pose.pose.orientation.z = point.orientation.z();
    path_msg.poses.push_back(pose);
  }
  return path_msg;
}

ControlCommand::ControlCommand() :
    velocity_cmd(Eigen::Vector3d::Zero()),
    yaw_rate(0.0),
    slack(0.0)
{
}

ControlCommand::ControlCommand(const Eigen::Vector3d &velocity_cmd,
                               double yaw_rate,
                               double slack) :
    velocity_cmd(velocity_cmd),
    yaw_rate(yaw_rate),
    slack(slack)
{
}

ControlCommand::~ControlCommand()
{
}

} // namespace acado_mpc_common
