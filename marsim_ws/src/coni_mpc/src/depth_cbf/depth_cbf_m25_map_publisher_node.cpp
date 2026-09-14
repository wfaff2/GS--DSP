#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "depth_cbf_m25_map_publisher");
  ros::NodeHandle private_nh("~");
  std::string map_name;
  private_nh.param("map_name", map_name, std::string(""));
  if (map_name.empty()) {
    ROS_FATAL("~map_name is required");
    return 1;
  }
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_name, cloud_xyz) != 0) {
    ROS_FATAL_STREAM("Cannot load M2.5 map: " << map_name);
    return 1;
  }
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.reserve(cloud_xyz.size());
  for (const auto& point : cloud_xyz.points) {
    pcl::PointXYZI value;
    value.x = point.x;
    value.y = point.y;
    value.z = point.z;
    value.intensity = 1.0f;
    cloud.push_back(value);
  }
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = "world";
  message.header.stamp = ros::Time::now();
  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<sensor_msgs::PointCloud2>(
      "/map_generator/global_cloud", 1, true);
  ros::WallDuration(0.5).sleep();
  publisher.publish(message);
  ROS_INFO_STREAM("Published latched M2.5 map " << map_name
                  << " with " << cloud.size() << " points");
  ros::spin();
  return 0;
}
