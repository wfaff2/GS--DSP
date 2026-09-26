#!/usr/bin/env python3
"""RViz-only solid GT world visualization for MARSIM/LV-DOT experiments."""

import math

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from local_sensing_node.msg import DynamicObstacleStateArray


class WorldVisualizer:
    def __init__(self):
        self.publisher = rospy.Publisher("/box_validation/world_markers", MarkerArray,
                                         queue_size=2, latch=True)
        self.latest_odom = None
        self.latest_obstacles = []
        self.latest_tracked_boxes = []
        self.trail = []
        trajectory = rospy.get_param("/box_validation/uav_trajectory", {})
        self.start_position = trajectory.get("start_position", [3.0, -1.0, 1.5])
        self.end_position = trajectory.get("end_position", [3.0, 1.0, 1.5])
        rospy.Subscriber("/coni_mpc/quad_odom1", Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber("/quad0_pcl_render_node/dynamic_obstacle_states",
                         DynamicObstacleStateArray, self.obstacles_cb, queue_size=20)
        rospy.Subscriber("/onboard_detector/tracked_bboxes", MarkerArray,
                         self.tracked_boxes_cb, queue_size=20)
        rospy.Timer(rospy.Duration(0.05), self.publish)

    def odom_cb(self, message):
        self.latest_odom = message
        position = message.pose.pose.position
        point = Point(x=position.x, y=position.y, z=position.z)
        if not self.trail or ((point.x - self.trail[-1].x) ** 2 +
                              (point.y - self.trail[-1].y) ** 2 +
                              (point.z - self.trail[-1].z) ** 2) > 0.0004:
            self.trail.append(point)
            self.trail = self.trail[-2000:]

    def obstacles_cb(self, message):
        self.latest_obstacles = list(message.obstacles)

    def tracked_boxes_cb(self, message):
        self.latest_tracked_boxes = list(message.markers)

    @staticmethod
    def base(marker_id, namespace, marker_type):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def publish(self, _event):
        markers = MarkerArray()
        clear = self.base(0, "clear", Marker.CUBE)
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        ground = self.base(0, "gt_ground", Marker.CUBE)
        ground.pose.position.z = -0.025
        ground.scale.x, ground.scale.y, ground.scale.z = 12.0, 12.0, 0.05
        ground.color.r, ground.color.g, ground.color.b, ground.color.a = 0.35, 0.38, 0.42, 1.0
        markers.markers.append(ground)

        boundary = self.base(0, "world_boundary", Marker.LINE_LIST)
        boundary.scale.x = 0.025
        boundary.color.r, boundary.color.g, boundary.color.b, boundary.color.a = 0.65, 0.7, 0.75, 0.7
        corners = [(x, y, z) for z in (0.0, 3.0) for y in (-6.0, 6.0) for x in (-6.0, 6.0)]
        edges = [(0, 1), (0, 2), (1, 3), (2, 3), (4, 5), (4, 6),
                 (5, 7), (6, 7), (0, 4), (1, 5), (2, 6), (3, 7)]
        for first, second in edges:
            boundary.points.append(Point(*corners[first]))
            boundary.points.append(Point(*corners[second]))
        markers.markers.append(boundary)

        planned = self.base(0, "planned_uav_path", Marker.LINE_STRIP)
        planned.scale.x = 0.035
        planned.color.r, planned.color.g, planned.color.b, planned.color.a = 0.1, 0.9, 0.25, 0.85
        planned.points = [Point(*self.start_position), Point(*self.end_position)]
        markers.markers.append(planned)

        trail = self.base(0, "actual_uav_path", Marker.LINE_STRIP)
        trail.scale.x = 0.045
        trail.color.r, trail.color.g, trail.color.b, trail.color.a = 0.15, 0.55, 1.0, 1.0
        trail.points = list(self.trail)
        markers.markers.append(trail)

        if self.latest_odom is not None:
            uav = self.base(0, "uav", Marker.CUBE)
            uav.pose = self.latest_odom.pose.pose
            uav.scale.x, uav.scale.y, uav.scale.z = 0.4, 0.4, 0.3
            uav.color.r, uav.color.g, uav.color.b, uav.color.a = 0.05, 0.35, 1.0, 1.0
            markers.markers.append(uav)

            # The current body_to_lidar transform is identity, so the Mid-360
            # optical origin coincides with the UAV odometry position.
            lidar = self.base(0, "mid360_lidar", Marker.CYLINDER)
            lidar.pose = self.latest_odom.pose.pose
            lidar.scale.x, lidar.scale.y, lidar.scale.z = 0.18, 0.18, 0.12
            lidar.color.r, lidar.color.g, lidar.color.b, lidar.color.a = 0.06, 0.06, 0.07, 1.0
            markers.markers.append(lidar)

            lidar_origin = self.base(0, "mid360_origin", Marker.SPHERE)
            lidar_origin.pose.position = self.latest_odom.pose.pose.position
            lidar_origin.scale.x = lidar_origin.scale.y = lidar_origin.scale.z = 0.075
            lidar_origin.color.r, lidar_origin.color.g = 1.0, 0.1
            lidar_origin.color.b, lidar_origin.color.a = 0.05, 1.0
            markers.markers.append(lidar_origin)

            lidar_direction = self.base(0, "mid360_body_x", Marker.ARROW)
            lidar_direction.pose = self.latest_odom.pose.pose
            lidar_direction.scale.x, lidar_direction.scale.y, lidar_direction.scale.z = 0.5, 0.065, 0.065
            lidar_direction.color.r, lidar_direction.color.g = 1.0, 0.85
            lidar_direction.color.b, lidar_direction.color.a = 0.05, 1.0
            markers.markers.append(lidar_direction)

            lidar_label = self.base(0, "mid360_label", Marker.TEXT_VIEW_FACING)
            lidar_label.pose.position = self.latest_odom.pose.pose.position
            lidar_label.pose.position.z += 0.35
            lidar_label.scale.z = 0.18
            lidar_label.color.r = lidar_label.color.g = lidar_label.color.b = 1.0
            lidar_label.color.a = 1.0
            lidar_label.text = "Mid-360 LiDAR"
            markers.markers.append(lidar_label)

            # A compact 360-degree display cue. These rings identify the sensor
            # visually; actual returns are rendered directly from MARSIM /cloud.
            scan_rings = self.base(0, "mid360_scan_indicator", Marker.LINE_LIST)
            scan_rings.scale.x = 0.012
            scan_rings.color.r, scan_rings.color.g = 0.0, 0.85
            scan_rings.color.b, scan_rings.color.a = 1.0, 0.55
            center = self.latest_odom.pose.pose.position
            segments = 72
            for radius in (0.45, 0.8, 1.15):
                for segment in range(segments):
                    first = 2.0 * math.pi * segment / segments
                    second = 2.0 * math.pi * (segment + 1) / segments
                    scan_rings.points.append(
                        Point(center.x + radius * math.cos(first),
                              center.y + radius * math.sin(first), center.z))
                    scan_rings.points.append(
                        Point(center.x + radius * math.cos(second),
                              center.y + radius * math.sin(second), center.z))
            markers.markers.append(scan_rings)

        for index, obstacle in enumerate(self.latest_obstacles):
            box = self.base(index, "full_gt_obstacles", Marker.CUBE)
            box.pose.position = obstacle.position
            box.scale = obstacle.size
            if obstacle.is_dynamic:
                box.color.r, box.color.g, box.color.b = 0.95, 0.2, 0.15
            else:
                box.color.r, box.color.g, box.color.b = 1.0, 0.55, 0.05
            box.color.a = 0.9
            markers.markers.append(box)

            outline = self.base(index, "full_gt_outlines", Marker.CUBE)
            outline.pose.position = obstacle.position
            outline.scale.x = obstacle.size.x + 0.015
            outline.scale.y = obstacle.size.y + 0.015
            outline.scale.z = obstacle.size.z + 0.015
            outline.color.r = outline.color.g = outline.color.b = 0.05
            outline.color.a = 0.25
            markers.markers.append(outline)

        # LV-DOT publishes its final tracked boxes as LINE_LIST markers in the
        # numerically identical map frame. Re-render them in world only for the
        # independent comparison window; the original LV-DOT topic is untouched.
        for index, tracked in enumerate(self.latest_tracked_boxes):
            if tracked.action != Marker.ADD or not tracked.points:
                continue

            red_outline = self.base(index, "lvdot_tracked_box", Marker.LINE_LIST)
            red_outline.pose = tracked.pose
            red_outline.points = list(tracked.points)
            red_outline.scale.x = max(tracked.scale.x, 0.045)
            red_outline.color.r, red_outline.color.g = 1.0, 0.0
            red_outline.color.b, red_outline.color.a = 0.0, 1.0
            markers.markers.append(red_outline)

            x_values = [point.x for point in tracked.points]
            y_values = [point.y for point in tracked.points]
            z_values = [point.z for point in tracked.points]
            size_x = max(x_values) - min(x_values)
            size_y = max(y_values) - min(y_values)
            size_z = max(z_values) - min(z_values)
            if size_x > 0.0 and size_y > 0.0 and size_z > 0.0:
                red_solid = self.base(index, "lvdot_tracked_solid", Marker.CUBE)
                red_solid.pose = tracked.pose
                red_solid.scale.x, red_solid.scale.y, red_solid.scale.z = (
                    size_x, size_y, size_z)
                red_solid.color.r, red_solid.color.g = 1.0, 0.0
                red_solid.color.b, red_solid.color.a = 0.0, 0.22
                markers.markers.append(red_solid)

        self.publisher.publish(markers)


if __name__ == "__main__":
    rospy.init_node("lv_dot_world_visualizer")
    WorldVisualizer()
    rospy.spin()
