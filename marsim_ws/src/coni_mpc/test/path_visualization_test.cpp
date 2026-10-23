#include "coni_mpc/path_visualization.h"
#include <gtest/gtest.h>
#include <limits>

TEST(PathVisualization, BoundedHistoryRetainsLatestSamplesInChronologicalOrder) {
  coni_mpc::BoundedPathHistory history(4), other(2);
  for (int i = 0; i < 12000; ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time(i, 0);
    pose.pose.position.x = i;
    history.append(pose);
    if (i < 2) other.append(pose);
  }
  nav_msgs::Path path;
  path.header.frame_id = "map";
  history.fill(path);
  ASSERT_EQ(path.poses.size(), 4u);
  EXPECT_EQ(path.header.frame_id, "map");
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    EXPECT_EQ(path.poses[i].pose.position.x, 11996 + i);
    EXPECT_EQ(path.poses[i].header.stamp, ros::Time(11996 + i, 0));
  }
  EXPECT_EQ(other.poses().size(), 2u);
  EXPECT_EQ(other.poses().front().pose.position.x, 0);
  history.setCapacity(1);
  EXPECT_EQ(history.poses().size(), 1u);
  EXPECT_EQ(history.poses().front().pose.position.x, 11999);
}

TEST(PathVisualization, TenHertzDisplayDoesNotRequireDroppingControlSamples) {
  coni_mpc::BoundedPathHistory history(100);
  coni_mpc::PathVisualizationRate rate;
  rate.setPeriod(0.1);
  int publications = 0;
  for (int cycle = 0; cycle < 50; ++cycle) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = cycle;
    history.append(pose);
    if (rate.due(cycle * 0.02)) ++publications;
  }
  EXPECT_EQ(history.poses().size(), 50u);
  EXPECT_EQ(publications, 10);
}

TEST(PathVisualization, IndependentInstancesAndTimeRestart) {
  coni_mpc::PathVisualizationRate first, second;
  EXPECT_TRUE(first.due(10.0));
  EXPECT_FALSE(first.due(10.02));
  EXPECT_TRUE(second.due(10.02));
  EXPECT_TRUE(first.due(0.0));
  EXPECT_FALSE(first.due(std::numeric_limits<double>::quiet_NaN()));
}
