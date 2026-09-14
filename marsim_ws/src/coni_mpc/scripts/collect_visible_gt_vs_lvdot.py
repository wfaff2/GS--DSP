#!/usr/bin/env python3
"""Record final MARSIM-visible GT AABB and final LV-DOT tracked AABB."""

import csv
import json
import math
import os
import statistics
import threading

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from scipy.optimize import linear_sum_assignment
from sensor_msgs.msg import PointCloud2

from local_sensing_node.msg import DynamicObstacleStateArray
from onboard_detector.msg import TrackedObstacleArray


FIELDS = [
    "timestamp", "frame_index", "uav_x", "uav_y", "uav_z",
    "full_gt_cx", "full_gt_cy", "full_gt_cz",
    "full_gt_lx", "full_gt_ly", "full_gt_lz",
    "visible_gt_point_count",
    "visible_gt_xmin", "visible_gt_xmax", "visible_gt_ymin", "visible_gt_ymax",
    "visible_gt_zmin", "visible_gt_zmax",
    "visible_gt_cx", "visible_gt_cy", "visible_gt_cz",
    "visible_gt_lx", "visible_gt_ly", "visible_gt_lz",
    "lvdot_detected", "lvdot_tracking_id", "lvdot_slot_id",
    "lvdot_xmin", "lvdot_xmax", "lvdot_ymin", "lvdot_ymax",
    "lvdot_zmin", "lvdot_zmax",
    "lvdot_cx", "lvdot_cy", "lvdot_cz",
    "lvdot_lx", "lvdot_ly", "lvdot_lz",
]


def stamp(header):
    value = header.stamp.to_sec()
    return value if value > 0.0 else rospy.Time.now().to_sec()


def vector(value):
    return np.array([value.x, value.y, value.z], dtype=float)


class TrackIds:
    def __init__(self):
        self.next_id = 0
        self.tracks = {}

    def update(self, estimates, timestamp):
        ids = sorted(self.tracks)
        assigned = {}
        if ids and estimates:
            costs = np.zeros((len(ids), len(estimates)))
            for row, track_id in enumerate(ids):
                prior = self.tracks[track_id]
                dt = max(0.0, timestamp - prior["timestamp"])
                predicted = prior["center"] + prior["velocity"] * dt
                for col, estimate in enumerate(estimates):
                    costs[row, col] = (np.linalg.norm(predicted - estimate["center"]) +
                                       0.1 * np.linalg.norm(prior["size"] - estimate["size"]))
            rows, cols = linear_sum_assignment(costs)
            for row, col in zip(rows, cols):
                if costs[row, col] <= 0.8:
                    assigned[col] = ids[row]
        used = set(assigned.values())
        for track_id in list(ids):
            if track_id not in used:
                self.tracks[track_id]["missed"] += 1
                if self.tracks[track_id]["missed"] > 3:
                    del self.tracks[track_id]
        for index, estimate in enumerate(estimates):
            if index not in assigned:
                assigned[index] = self.next_id
                self.next_id += 1
            self.tracks[assigned[index]] = {
                "center": estimate["center"], "velocity": estimate["velocity"],
                "size": estimate["size"], "timestamp": timestamp, "missed": 0,
            }
        return assigned


class Collector:
    def __init__(self):
        self.root = os.path.abspath(os.path.expanduser(rospy.get_param("~result_root")))
        self.duration = float(rospy.get_param("~duration_sec", 20.0))
        self.warmup = float(rospy.get_param("~warmup_sec", 3.0))
        self.target_frames = int(rospy.get_param("~target_frames", 0))
        self.start = rospy.Time.now().to_sec()
        self.end = self.start + self.warmup + self.duration
        self.full_center = np.array(rospy.get_param("/box_validation/static_box/center"), dtype=float)
        self.full_size = np.array(rospy.get_param("/box_validation/static_box/size"), dtype=float)
        self.lock = threading.RLock()
        self.pending = []
        self.tracked = []
        self.odom = []
        self.gt = []
        self.finished = False
        self.frame_index = 0
        self.previous_stamp = None
        self.dropped_frames = 0
        self.valid_visible_frames = 0
        self.matched_frames = 0
        self.gt_covered_frames = 0
        self.point_counts = []
        self.id_switches = 0
        self.previous_target_track = None
        self.track_ids = TrackIds()
        os.makedirs(self.root, exist_ok=True)
        self.output = os.path.join(self.root, "visible_gt_vs_lvdot_box_raw.csv")
        self.handle = open(self.output, "w", newline="")
        self.writer = csv.DictWriter(self.handle, fieldnames=FIELDS)
        self.writer.writeheader()
        rospy.Subscriber("/quad0_pcl_render_node/visible_gt_points", PointCloud2,
                         self.visible_cb, queue_size=50)
        rospy.Subscriber("/onboard_detector/tracked_obstacles_raw", TrackedObstacleArray,
                         self.tracked_cb, queue_size=50)
        rospy.Subscriber("/coni_mpc/quad_odom1", Odometry, self.odom_cb, queue_size=100)
        rospy.Subscriber("/quad0_pcl_render_node/dynamic_obstacle_states",
                         DynamicObstacleStateArray, self.gt_cb, queue_size=50)
        rospy.Timer(rospy.Duration(0.05), self.timer_cb)
        rospy.on_shutdown(self.close)

    def visible_cb(self, message):
        now = rospy.Time.now().to_sec()
        if self.start + self.warmup <= now and (self.target_frames > 0 or now <= self.end):
            with self.lock:
                self.pending.append((now + 0.25, message))

    def tracked_cb(self, message):
        with self.lock:
            self.tracked.append((stamp(message.header), message))
            self.tracked = self.tracked[-50:]

    def odom_cb(self, message):
        with self.lock:
            self.odom.append((stamp(message.header), message))
            self.odom = self.odom[-100:]

    def gt_cb(self, message):
        with self.lock:
            self.gt.append((stamp(message.header), message))
            self.gt = self.gt[-50:]

    @staticmethod
    def nearest(cache, timestamp, tolerance):
        if not cache:
            return None
        nearest = min(cache, key=lambda item: abs(item[0] - timestamp))
        return nearest[1] if abs(nearest[0] - timestamp) <= tolerance else None

    @staticmethod
    def box_fields(prefix, center, size):
        minimum = center - size / 2.0
        maximum = center + size / 2.0
        return {
            prefix + "_xmin": minimum[0], prefix + "_xmax": maximum[0],
            prefix + "_ymin": minimum[1], prefix + "_ymax": maximum[1],
            prefix + "_zmin": minimum[2], prefix + "_zmax": maximum[2],
            prefix + "_cx": center[0], prefix + "_cy": center[1], prefix + "_cz": center[2],
            prefix + "_lx": size[0], prefix + "_ly": size[1], prefix + "_lz": size[2],
        }

    def record(self, cloud):
        if self.target_frames > 0 and self.frame_index >= self.target_frames:
            return
        timestamp = stamp(cloud.header)
        points = np.array(list(pc2.read_points(cloud, field_names=("x", "y", "z"),
                                               skip_nans=True)), dtype=float)
        count = len(points)
        with self.lock:
            self.frame_index += 1
            if self.previous_stamp is not None:
                self.dropped_frames += max(0, int(round((timestamp - self.previous_stamp) * 10.0)) - 1)
            self.previous_stamp = timestamp
            self.point_counts.append(count)
            row = {field: "" for field in FIELDS}
            row.update({"timestamp": "%.9f" % timestamp, "frame_index": self.frame_index,
                        "visible_gt_point_count": count, "lvdot_detected": 0})
            row.update({"full_gt_cx": self.full_center[0], "full_gt_cy": self.full_center[1],
                        "full_gt_cz": self.full_center[2], "full_gt_lx": self.full_size[0],
                        "full_gt_ly": self.full_size[1], "full_gt_lz": self.full_size[2]})
            if count:
                minimum, maximum = points.min(axis=0), points.max(axis=0)
                row.update(self.box_fields("visible_gt", (minimum + maximum) / 2.0,
                                           maximum - minimum))
                self.valid_visible_frames += 1

            odometry = self.nearest(self.odom, timestamp, 0.15)
            if odometry is not None:
                row.update({"uav_x": odometry.pose.pose.position.x,
                            "uav_y": odometry.pose.pose.position.y,
                            "uav_z": odometry.pose.pose.position.z})
            gt_message = self.nearest(self.gt, timestamp, 0.15)
            if gt_message is not None and len(gt_message.obstacles) == 1:
                self.gt_covered_frames += 1

            tracked_message = self.nearest(self.tracked, timestamp, 0.25)
            estimates = []
            if tracked_message is not None:
                estimates = [{"slot": int(item.slot_id), "center": vector(item.position),
                              "velocity": vector(item.velocity), "size": vector(item.size)}
                             for item in tracked_message.obstacles]
            assignments = self.track_ids.update(estimates, timestamp)
            if count and estimates:
                visible_center = np.array([row["visible_gt_cx"], row["visible_gt_cy"],
                                           row["visible_gt_cz"]])
                distances = [np.linalg.norm(item["center"] - visible_center) for item in estimates]
                index = int(np.argmin(distances))
                if distances[index] <= 1.5:
                    estimate = estimates[index]
                    track_id = assignments[index]
                    row.update({"lvdot_detected": 1, "lvdot_tracking_id": track_id,
                                "lvdot_slot_id": estimate["slot"]})
                    row.update(self.box_fields("lvdot", estimate["center"], estimate["size"]))
                    self.matched_frames += 1
                    if self.previous_target_track is not None and track_id != self.previous_target_track:
                        self.id_switches += 1
                    self.previous_target_track = track_id
            self.writer.writerow(row)
            self.handle.flush()

    def timer_cb(self, _event):
        now = rospy.Time.now().to_sec()
        with self.lock:
            ready = [item for item in self.pending if item[0] <= now]
            self.pending = [item for item in self.pending if item[0] > now]
        for _, cloud in ready:
            self.record(cloud)
        target_complete = self.target_frames > 0 and self.frame_index >= self.target_frames
        timed_complete = self.target_frames <= 0 and now >= self.end + 0.35
        if target_complete or timed_complete:
            with self.lock:
                remaining = self.pending
                self.pending = []
            for _, cloud in remaining:
                self.record(cloud)
            self.finish()

    def finish(self):
        with self.lock:
            if self.finished:
                return
            self.finished = True
            counts = sorted(self.point_counts)
            summary = {
                "total_frames": self.frame_index,
                "target_frames": self.target_frames,
                "valid_visible_gt_frames": self.valid_visible_frames,
                "visible_point_count_min": min(counts) if counts else 0,
                "visible_point_count_median": statistics.median(counts) if counts else 0,
                "visible_point_count_max": max(counts) if counts else 0,
                "lvdot_matched_frames": self.matched_frames,
                "gt_covered_frames": self.gt_covered_frames,
                "gt_coverage_complete": self.gt_covered_frames == self.frame_index,
                "dropped_frames": self.dropped_frames,
                "tracking_id_switches": self.id_switches,
                "full_gt_center": self.full_center.tolist(),
                "full_gt_size": self.full_size.tolist(),
                "visible_gt_source": "MARSIM final polar ray returns labeled by original map point index",
                "sensor_cloud_modified": False,
                "marsim_mid360_parameters": rospy.get_param("/quad0_pcl_render_node", {}),
                "lv_dot_parameters": rospy.get_param("/onboard_detector", {}),
            }
            with open(os.path.join(self.root, "visible_gt_collection_config.json"), "w") as handle:
                json.dump(summary, handle, indent=2, sort_keys=True)
                handle.write("\n")
        rospy.signal_shutdown("visible GT collection complete")

    def close(self):
        if not self.handle.closed:
            self.handle.close()


if __name__ == "__main__":
    rospy.init_node("visible_gt_vs_lvdot_collector")
    Collector()
    rospy.spin()
