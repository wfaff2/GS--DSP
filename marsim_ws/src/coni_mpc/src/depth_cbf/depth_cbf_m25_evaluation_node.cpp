#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "coni_mpc/depth_cbf/depth_cbf_regressor.h"
#include "coni_mpc/depth_cbf/point_cloud_preprocessor.h"

namespace coni_mpc {
namespace depth_cbf {
namespace {

using Clock = std::chrono::steady_clock;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

double elapsedMs(const Clock::time_point& begin) {
  return std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
}

Pose3d poseFromOdom(const nav_msgs::Odometry& odom) {
  Pose3d result;
  result.position_W = {odom.pose.pose.position.x, odom.pose.pose.position.y,
                       odom.pose.pose.position.z};
  Eigen::Quaterniond q(odom.pose.pose.orientation.w,
                       odom.pose.pose.orientation.x,
                       odom.pose.pose.orientation.y,
                       odom.pose.pose.orientation.z);
  if (q.norm() < 1e-12) throw std::runtime_error("zero-norm odometry quaternion");
  result.rotation_WF = q.normalized().toRotationMatrix();
  result.stamp = odom.header.stamp;
  return result;
}

std::vector<double> requireVector(ros::NodeHandle* nh, const std::string& name,
                                  std::size_t count) {
  std::vector<double> values;
  if (!nh->getParam(name, values) || values.size() != count) {
    throw std::runtime_error("~" + name + " must contain " +
                             std::to_string(count) + " numbers");
  }
  return values;
}

double yawFromOdom(const nav_msgs::Odometry& odom) {
  const auto& o = odom.pose.pose.orientation;
  return std::atan2(2.0 * (o.w * o.z + o.x * o.y),
                    1.0 - 2.0 * (o.y * o.y + o.z * o.z));
}

double angleDeg(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  if (a.norm() < 1e-12 || b.norm() < 1e-12) return kNan;
  const double cosine = std::max(-1.0, std::min(1.0,
      a.normalized().dot(b.normalized())));
  return 180.0 * std::acos(cosine) / M_PI;
}

struct GroundTruth {
  double distance = kNan;
  Eigen::Vector3d nearest_W = Eigen::Vector3d::Constant(kNan);
  int obstacle_id = -1;
};

class M25EvaluatorNode {
 public:
  M25EvaluatorNode()
      : nh_(), pnh_("~"),
        cloud_sub_(nh_, paramString("cloud_topic",
                                    "/quad0_pcl_render_node/sensor_cloud"), 30),
        quad_sub_(nh_, paramString("quad_odom_topic",
                                   "/coni_mpc/quad_odom1"), 50),
        car_sub_(nh_, paramString("car_odom_topic", "/coni_mpc/car_odom"), 50) {
    pnh_.param("m25/scene", scene_, std::string("A1_plane"));
    pnh_.param("m25/motion", motion_, std::string("front"));
    pnh_.param("m25/run_id", run_id_, scene_ + "_" + motion_);
    pnh_.param("m25/result_root", result_root_,
               std::string("results/depth_cbf_m25"));
    pnh_.param("m25/gt_pcd", gt_pcd_path_, std::string(""));
    pnh_.param("m25/max_pose_time_offset_sec", max_pose_offset_, 0.025);
    pnh_.param("m25/root_max_distance", root_max_distance_, 4.0);
    pnh_.param("m25/root_samples", root_samples_, 160);
    pnh_.param("m25/non_conservative_tolerance",
               non_conservative_tolerance_, 1e-3);
    pnh_.param("m25/min_raylength", min_raylength_, 1.0);
    pnh_.param("m25/yaw_fov_deg", yaw_fov_deg_, 360.0);
    pnh_.param("m25/vertical_fov_deg", vertical_fov_deg_, 90.0);

    const PreprocessorConfig pre_cfg = loadPreprocessorConfig();
    const RegressionConfig reg_cfg = loadRegressionConfig();
    std::string error, warning;
    if (!PointCloudPreprocessor::validateConfig(pre_cfg, &error, &warning))
      throw std::runtime_error(error);
    if (!DepthCbfRegressor::validateConfig(reg_cfg, &error))
      throw std::runtime_error(error);
    preprocessor_.reset(new PointCloudPreprocessor(pre_cfg));
    regressor_.reset(new DepthCbfRegressor(reg_cfg));
    if (scene_ == "official") loadOfficialGroundTruth();

    std::filesystem::create_directories(result_root_ + "/frames");
    csv_path_ = result_root_ + "/frames/" + run_id_ + ".csv";
    csv_.open(csv_path_);
    if (!csv_) throw std::runtime_error("cannot open " + csv_path_);
    csv_ << std::setprecision(12);
    writeHeader();

    SyncPolicy policy(80);
    double slop = 0.025;
    pnh_.param("sync_slop_sec", slop, slop);
    policy.setMaxIntervalDuration(ros::Duration(slop));
    sync_.reset(new Synchronizer(static_cast<const SyncPolicy&>(policy),
                                 cloud_sub_, quad_sub_, car_sub_));
    sync_->registerCallback(boost::bind(&M25EvaluatorNode::callback, this,
                                        _1, _2, _3));
    ROS_INFO_STREAM("M2.5 evaluator writes " << csv_path_
                    << "; sole algorithm cloud input=" << cloud_sub_.getTopic()
                    << "; GT mode=" << scene_ << " is evaluation-only.");
  }

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, nav_msgs::Odometry, nav_msgs::Odometry>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  std::string paramString(const std::string& name, const std::string& value) {
    std::string result = value;
    pnh_.param(name, result, result);
    return result;
  }

  PreprocessorConfig loadPreprocessorConfig() {
    PreprocessorConfig c;
    pnh_.param("filter/min_range", c.min_range, c.min_range);
    pnh_.param("filter/max_range", c.max_range, c.max_range);
    pnh_.param("filter/local_radius", c.local_radius, c.local_radius);
    pnh_.param("filter/voxel_leaf_size", c.voxel_leaf_size, c.voxel_leaf_size);
    pnh_.param("filter/enable_voxel_filter", c.enable_voxel_filter,
               c.enable_voxel_filter);
    pnh_.param("filter/enable_ground_filter", c.enable_ground_filter,
               c.enable_ground_filter);
    pnh_.param("filter/ground_min_z_W", c.ground_min_z_W, c.ground_min_z_W);
    pnh_.param("query_mesh/half_width_steps", c.query_mesh_half_width_steps,
               c.query_mesh_half_width_steps);
    pnh_.param("query_mesh/spacing", c.query_mesh_spacing,
               c.query_mesh_spacing);
    pnh_.param("query_mesh/nearest_neighbor_max_distance",
               c.nearest_neighbor_max_distance,
               c.nearest_neighbor_max_distance);
    const auto r = requireVector(&pnh_, "extrinsics/rotation_BL", 9);
    const auto t = requireVector(&pnh_, "extrinsics/translation_BL", 3);
    c.rotation_BL << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
    c.translation_BL = {t[0], t[1], t[2]};
    return c;
  }

  RegressionConfig loadRegressionConfig() {
    RegressionConfig c;
    pnh_.param("query_mesh/half_width_steps", c.query_mesh_half_width_steps,
               c.query_mesh_half_width_steps);
    pnh_.param("query_mesh/spacing", c.query_mesh_spacing,
               c.query_mesh_spacing);
    pnh_.param("query_mesh/nearest_neighbor_max_distance",
               c.nearest_neighbor_max_distance,
               c.nearest_neighbor_max_distance);
    pnh_.param("safety/d_safe", c.d_safe, c.d_safe);
    pnh_.param("regression/relative_rank_tolerance", c.relative_rank_tolerance,
               c.relative_rank_tolerance);
    pnh_.param("regression/max_condition_number", c.max_condition_number,
               c.max_condition_number);
    pnh_.param("regression/max_rmse", c.max_rmse, c.max_rmse);
    pnh_.param("regression/max_abs_error", c.max_abs_error,
               c.max_abs_error);
    return c;
  }

  void loadOfficialGroundTruth() {
    if (gt_pcd_path_.empty() ||
        pcl::io::loadPCDFile<pcl::PointXYZ>(gt_pcd_path_, gt_cloud_) != 0)
      throw std::runtime_error("cannot load official GT PCD: " + gt_pcd_path_);
    gt_tree_.setInputCloud(gt_cloud_.makeShared());
  }

  GroundTruth groundTruth(const Eigen::Vector3d& p) const {
    GroundTruth gt;
    if (scene_ == "A1_plane") {
      gt.nearest_W = {5.0, p.y(), p.z()};
      gt.distance = std::abs(5.0 - p.x());
      gt.obstacle_id = 0;
    } else if (scene_ == "A2_cylinder") {
      gt = cylinderGroundTruth(p, Eigen::Vector2d(5.0, 0.0), 1.0, 0);
    } else if (scene_ == "A3_corner") {
      const double dx = std::abs(5.0 - p.x());
      const double dy = std::abs(5.0 - p.y());
      gt.obstacle_id = dx <= dy ? 0 : 1;
      gt.distance = std::min(dx, dy);
      gt.nearest_W = dx <= dy ? Eigen::Vector3d(5.0, p.y(), p.z())
                              : Eigen::Vector3d(p.x(), 5.0, p.z());
    } else if (scene_ == "A4_switch") {
      const GroundTruth a = cylinderGroundTruth(
          p, Eigen::Vector2d(5.0, -1.5), 0.8, 0);
      const GroundTruth b = cylinderGroundTruth(
          p, Eigen::Vector2d(5.0, 1.5), 0.8, 1);
      gt = a.distance <= b.distance ? a : b;
    } else if (scene_ == "official" && !gt_cloud_.empty()) {
      pcl::PointXYZ q(static_cast<float>(p.x()), static_cast<float>(p.y()),
                      static_cast<float>(p.z()));
      std::vector<int> index(1);
      std::vector<float> d2(1);
      if (gt_tree_.nearestKSearch(q, 1, index, d2) == 1) {
        const auto& n = gt_cloud_.points[index[0]];
        gt.nearest_W = {n.x, n.y, n.z};
        gt.distance = std::sqrt(d2[0]);
        gt.obstacle_id = index[0];
      }
    }
    return gt;
  }

  static GroundTruth cylinderGroundTruth(const Eigen::Vector3d& p,
                                         const Eigen::Vector2d& center,
                                         double radius, int id) {
    GroundTruth gt;
    Eigen::Vector2d radial(p.x() - center.x(), p.y() - center.y());
    const double norm = radial.norm();
    if (norm < 1e-12) radial = Eigen::Vector2d::UnitX();
    else radial /= norm;
    gt.nearest_W = {center.x() + radius * radial.x(),
                    center.y() + radius * radial.y(), p.z()};
    gt.distance = std::abs(norm - radius);
    gt.obstacle_id = id;
    return gt;
  }

  template <typename Function>
  double firstEnteringRoot(Function function) const {
    double previous_s = 0.0;
    double previous_f = function(0.0);
    if (!std::isfinite(previous_f) || previous_f <= 0.0) return kNan;
    for (int i = 1; i <= root_samples_; ++i) {
      const double s = root_max_distance_ * i / root_samples_;
      const double value = function(s);
      if (std::isfinite(value) && value <= 0.0) {
        double low = previous_s, high = s;
        for (int k = 0; k < 50; ++k) {
          const double mid = 0.5 * (low + high);
          const double fmid = function(mid);
          if (!std::isfinite(fmid) || fmid > 0.0) low = mid;
          else high = mid;
        }
        return 0.5 * (low + high);
      }
      if (std::isfinite(value)) {
        previous_s = s;
        previous_f = value;
      }
    }
    return kNan;
  }

  static std::string number(double value) {
    if (!std::isfinite(value)) return "nan";
    std::ostringstream stream;
    stream << std::setprecision(12) << value;
    return stream.str();
  }

  void writeHeader() {
    csv_ << "run_id,scene,motion,frame_index,t_cloud,t_quad,t_car,dt_quad,dt_car,"
            "sync_valid,raw_point_count,world_point_count,local_point_count,"
            "valid,failure_reason,uav_x,uav_y,uav_z,uav_yaw_deg,d_lidar,d_GT,"
            "abs_d_lidar_d_GT,h_raw,h_fit,abs_h_fit_h_raw,qstar_x,qstar_y,qstar_z,"
            "boundary_valid,s_raw,s_fit,signed_boundary_shift,nraw_x,nraw_y,nraw_z,"
            "nfit_x,nfit_y,nfit_z,gradient_angle_deg,raw_gradient_jump_deg,"
            "fit_gradient_jump_deg,nearest_obstacle_id,nearest_changed,rmse,"
            "max_abs_error,condition_number,rank,T_preprocess_ms,T_kdtree_build_ms,"
            "T_query_generation_ms,T_nearest_neighbor_query_ms,T_regression_ms,"
            "T_depth_cbf_total_ms,min_raylength,yaw_fov_deg,vertical_fov_deg,"
            "p_uav_N_x,p_uav_N_y,p_uav_N_z,distance_valid,distance_signed_error,"
            "h_center_valid,h_center_error,gradient_valid,boundary_abs_error,"
            "expected_obstacle_bearing_deg,query_ms,pipeline_ms,cloud_stamp,"
            "quad_stamp,car_stamp,quad_cloud_offset_ms,car_cloud_offset_ms,"
            "non_conservative_tolerance\n";
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                const nav_msgs::OdometryConstPtr& quad_msg,
                const nav_msgs::OdometryConstPtr& car_msg) {
    const auto total_begin = Clock::now();
    const double dt_quad = (quad_msg->header.stamp - cloud_msg->header.stamp).toSec();
    const double dt_car = (car_msg->header.stamp - cloud_msg->header.stamp).toSec();
    const bool sync_valid = std::abs(dt_quad) <= max_pose_offset_ &&
                            std::abs(dt_car) <= max_pose_offset_;
    PointCloud cloud_L;
    pcl::fromROSMsg(*cloud_msg, cloud_L);
    const Pose3d quad_pose = poseFromOdom(*quad_msg);
    const Pose3d car_pose = poseFromOdom(*car_msg);
    const GroundTruth gt = groundTruth(quad_pose.position_W);

    PreprocessedClouds clouds;
    std::string error;
    double preprocess_ms = kNan;
    bool preprocess_ok = false;
    if (sync_valid) {
      const auto begin = Clock::now();
      preprocess_ok = preprocessor_->process(cloud_L, quad_pose, car_pose,
                                             cloud_msg->header.stamp, &clouds,
                                             &error);
      preprocess_ms = elapsedMs(begin);
    }

    LocalBarrier barrier;
    RegressionDiagnostics diagnostics;
    RegressionTiming timing;
    RegressionFailureReason failure = sync_valid
        ? RegressionFailureReason::INTERNAL_ERROR
        : RegressionFailureReason::INTERNAL_ERROR;
    bool valid = false;
    if (sync_valid && preprocess_ok) {
      valid = regressor_->fit(*clouds.local_cloud_N, clouds.query_center_N,
                              cloud_msg->header.stamp, &barrier, &diagnostics,
                              &error, &timing, &failure);
    }
    std::string failure_name = sync_valid ?
        (preprocess_ok ? regressionFailureReasonName(failure) : "EMPTY_CLOUD")
        : "SYNC_REJECT";

    double d_lidar = kNan;
    if (preprocess_ok && !clouds.cloud_W->empty()) {
      d_lidar = std::numeric_limits<double>::infinity();
      for (const auto& point : clouds.cloud_W->points) {
        d_lidar = std::min(d_lidar,
            (Eigen::Vector3d(point.x, point.y, point.z) -
             quad_pose.position_W).norm());
      }
    }

    double h_raw = kNan, h_fit = kNan, s_raw = kNan, s_fit = kNan;
    double shift = kNan, grad_angle = kNan, raw_jump = kNan, fit_jump = kNan;
    Eigen::Vector3d qstar = Eigen::Vector3d::Constant(kNan);
    Eigen::Vector3d nraw = Eigen::Vector3d::Constant(kNan);
    Eigen::Vector3d nfit = Eigen::Vector3d::Constant(kNan);
    bool supported = false;
    bool boundary_valid = false;
    if (preprocess_ok && !clouds.local_cloud_N->empty()) {
      h_raw = regressor_->rawBarrierValue(*clouds.local_cloud_N,
          clouds.query_center_N, &qstar, &supported);
      if (!supported) h_raw = kNan;
    }
    if (valid && supported) {
      h_fit = barrier.evaluate(clouds.query_center_N);
      const Eigen::Vector3d to_obstacle = qstar - clouds.query_center_N;
      if (to_obstacle.norm() > 1e-12) {
        const Eigen::Vector3d direction = to_obstacle.normalized();
        nraw = -direction;
        nfit = barrier.gradient(clouds.query_center_N).normalized();
        s_raw = firstEnteringRoot([&](double s) {
          bool in_support = false;
          return regressor_->rawBarrierValue(*clouds.local_cloud_N,
              clouds.query_center_N + s * direction, nullptr, &in_support);
        });
        s_fit = firstEnteringRoot([&](double s) {
          return barrier.evaluate(clouds.query_center_N + s * direction);
        });
        boundary_valid = std::isfinite(s_raw) && std::isfinite(s_fit);
        if (boundary_valid) shift = s_fit - s_raw;
        grad_angle = angleDeg(nraw, nfit);
        if (have_previous_) {
          raw_jump = angleDeg(previous_nraw_, nraw);
          fit_jump = angleDeg(previous_nfit_, nfit);
        }
        previous_nraw_ = nraw;
        previous_nfit_ = nfit;
        have_previous_ = true;
      }
    }
    const bool nearest_changed = previous_obstacle_id_ >= 0 &&
                                 gt.obstacle_id != previous_obstacle_id_;
    previous_obstacle_id_ = gt.obstacle_id;
    const double depth_cbf_ms = sync_valid && preprocess_ok
        ? timing.total_ms : kNan;
    const double pipeline_ms = sync_valid && preprocess_ok
        ? preprocess_ms + timing.total_ms : elapsedMs(total_begin);
    const double yaw_deg = 180.0 * yawFromOdom(*quad_msg) / M_PI;
    const Eigen::Vector3d p_uav_N =
        car_pose.rotation_WF.transpose() *
        (quad_pose.position_W - car_pose.position_W);
    double expected_bearing = kNan;
    if (std::isfinite(gt.nearest_W.x())) {
      const Eigen::Vector3d delta = gt.nearest_W - quad_pose.position_W;
      expected_bearing = 180.0 * std::atan2(delta.y(), delta.x()) / M_PI - yaw_deg;
      while (expected_bearing > 180.0) expected_bearing -= 360.0;
      while (expected_bearing < -180.0) expected_bearing += 360.0;
    }
    const auto n = [&](double value) { return number(value); };
    csv_ << run_id_ << ',' << scene_ << ',' << motion_ << ',' << frame_index_++
         << ',' << n(cloud_msg->header.stamp.toSec())
         << ',' << n(quad_msg->header.stamp.toSec())
         << ',' << n(car_msg->header.stamp.toSec())
         << ',' << n(dt_quad) << ',' << n(dt_car) << ',' << sync_valid
         << ',' << cloud_L.size() << ',' << (preprocess_ok ? clouds.cloud_W->size() : 0)
         << ',' << (preprocess_ok ? clouds.local_cloud_N->size() : 0)
         << ',' << valid << ',' << failure_name
         << ',' << n(quad_pose.position_W.x()) << ',' << n(quad_pose.position_W.y())
         << ',' << n(quad_pose.position_W.z()) << ',' << n(yaw_deg)
         << ',' << n(d_lidar) << ',' << n(gt.distance)
         << ',' << n(std::abs(d_lidar - gt.distance))
         << ',' << n(h_raw) << ',' << n(h_fit) << ',' << n(std::abs(h_fit - h_raw))
         << ',' << n(qstar.x()) << ',' << n(qstar.y()) << ',' << n(qstar.z())
         << ',' << boundary_valid << ',' << n(s_raw) << ',' << n(s_fit)
         << ',' << n(shift) << ',' << n(nraw.x()) << ',' << n(nraw.y())
         << ',' << n(nraw.z()) << ',' << n(nfit.x()) << ',' << n(nfit.y())
         << ',' << n(nfit.z()) << ',' << n(grad_angle) << ',' << n(raw_jump)
         << ',' << n(fit_jump) << ',' << gt.obstacle_id << ',' << nearest_changed
         << ',' << n(barrier.rmse) << ',' << n(barrier.max_abs_error)
         << ',' << n(barrier.condition_number) << ',' << barrier.rank
         << ',' << n(preprocess_ms) << ',' << n(timing.kdtree_build_ms)
         << ',' << n(timing.query_generation_ms)
         << ',' << n(timing.nearest_neighbor_query_ms)
         << ',' << n(timing.regression_ms) << ',' << n(depth_cbf_ms)
         << ',' << n(min_raylength_) << ',' << n(yaw_fov_deg_)
         << ',' << n(vertical_fov_deg_)
         << ',' << n(p_uav_N.x()) << ',' << n(p_uav_N.y()) << ',' << n(p_uav_N.z())
         << ',' << (std::isfinite(d_lidar) && std::isfinite(gt.distance))
         << ',' << n(d_lidar - gt.distance)
         << ',' << (std::isfinite(h_raw) && std::isfinite(h_fit))
         << ',' << n(h_fit - h_raw)
         << ',' << (std::isfinite(grad_angle)) << ',' << n(std::abs(shift))
         << ',' << n(expected_bearing)
         << ',' << n(timing.query_generation_ms +
                      timing.nearest_neighbor_query_ms)
         << ',' << n(pipeline_ms)
         << ',' << n(cloud_msg->header.stamp.toSec())
         << ',' << n(quad_msg->header.stamp.toSec())
         << ',' << n(car_msg->header.stamp.toSec())
         << ',' << n(1000.0 * dt_quad) << ',' << n(1000.0 * dt_car)
         << ',' << n(non_conservative_tolerance_) << '\n';
    csv_.flush();
  }

  ros::NodeHandle nh_, pnh_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> quad_sub_, car_sub_;
  std::unique_ptr<Synchronizer> sync_;
  std::unique_ptr<PointCloudPreprocessor> preprocessor_;
  std::unique_ptr<DepthCbfRegressor> regressor_;
  PointCloud gt_cloud_;
  mutable pcl::KdTreeFLANN<pcl::PointXYZ> gt_tree_;
  std::ofstream csv_;
  std::string scene_, motion_, run_id_, result_root_, gt_pcd_path_, csv_path_;
  double max_pose_offset_ = 0.025, root_max_distance_ = 4.0;
  double min_raylength_ = 1.0, yaw_fov_deg_ = 360.0, vertical_fov_deg_ = 90.0;
  double non_conservative_tolerance_ = 1e-3;
  int root_samples_ = 160;
  std::size_t frame_index_ = 0;
  bool have_previous_ = false;
  Eigen::Vector3d previous_nraw_, previous_nfit_;
  int previous_obstacle_id_ = -1;
};

}  // namespace
}  // namespace depth_cbf
}  // namespace coni_mpc

int main(int argc, char** argv) {
  ros::init(argc, argv, "depth_cbf_m25_evaluator");
  try {
    coni_mpc::depth_cbf::M25EvaluatorNode node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("M2.5 evaluator failed: " << e.what());
    return 1;
  }
  return 0;
}
