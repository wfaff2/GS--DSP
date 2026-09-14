#!/usr/bin/env python3
"""Raw LV-DOT/MARSIM recorder. It intentionally computes no error metrics."""

import csv
import json
import math
import os
import threading

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from scipy.optimize import linear_sum_assignment

from local_sensing_node.msg import DynamicObstacleStateArray
from onboard_detector.msg import TrackedObstacleArray


RAW_FIELDS = [
    "scenario", "frame_index", "timestamp", "source_timestamp", "gt_timestamp",
    "obstacle_id", "is_dynamic", "association_status", "association_distance",
    "tracking_id", "lvdot_slot_id", "est_is_dynamic",
    "est_x", "est_y", "est_z", "est_vx", "est_vy", "est_vz",
    "est_lx", "est_ly", "est_lz",
    "gt_x", "gt_y", "gt_z", "gt_vx", "gt_vy", "gt_vz",
    "gt_lx", "gt_ly", "gt_lz", "gt_geometry_type", "gt_radius",
    "uav_x", "uav_y", "uav_z", "uav_vx", "uav_vy", "uav_vz",
]
EVENT_FIELDS = [
    "timestamp", "scenario", "frame_index", "event_type", "obstacle_id",
    "tracking_id", "lvdot_slot_id", "details",
]


def stamp_sec(header):
    value = header.stamp.to_sec()
    return value if value > 0.0 else rospy.Time.now().to_sec()


def vec3(value):
    return np.array([value.x, value.y, value.z], dtype=float)


class TrackManager:
    """Collector-only persistent IDs; LV-DOT's original slot ID is also retained."""

    def __init__(self, gate=0.8, max_missed=3):
        self.gate = gate
        self.max_missed = max_missed
        self.next_id = 0
        self.tracks = {}

    def update(self, estimates, timestamp):
        track_ids = sorted(self.tracks)
        assigned = {}
        events = []
        if track_ids and estimates:
            cost = np.zeros((len(track_ids), len(estimates)), dtype=float)
            for i, track_id in enumerate(track_ids):
                track = self.tracks[track_id]
                dt = max(0.0, timestamp - track["timestamp"])
                predicted = track["position"] + track["velocity"] * dt
                for j, estimate in enumerate(estimates):
                    center_cost = np.linalg.norm(predicted - estimate["position"])
                    size_cost = np.linalg.norm(track["size"] - estimate["size"])
                    cost[i, j] = center_cost + 0.1 * size_cost
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols):
                if cost[i, j] <= self.gate:
                    assigned[j] = track_ids[i]

        matched_tracks = set(assigned.values())
        for track_id in list(track_ids):
            if track_id not in matched_tracks:
                self.tracks[track_id]["missed"] += 1
                if self.tracks[track_id]["missed"] > self.max_missed:
                    events.append(("track_lost", track_id, "collector track exceeded missed-frame limit"))
                    del self.tracks[track_id]

        for index, estimate in enumerate(estimates):
            if index not in assigned:
                track_id = self.next_id
                self.next_id += 1
                assigned[index] = track_id
                events.append(("track_created", track_id, "collector track created"))
            track_id = assigned[index]
            self.tracks[track_id] = {
                "position": estimate["position"],
                "velocity": estimate["velocity"],
                "size": estimate["size"],
                "timestamp": timestamp,
                "missed": 0,
            }
        return assigned, events


class RawCollector:
    def __init__(self):
        self.scenario = rospy.get_param("~scenario")
        if self.scenario not in ("static", "dynamic", "static_box", "dynamic_box"):
            raise ValueError("unsupported collection scenario")
        self.root = os.path.abspath(os.path.expanduser(rospy.get_param("~collection_root")))
        self.duration = float(rospy.get_param("~duration_sec"))
        self.warmup = float(rospy.get_param("~warmup_sec"))
        self.motion = rospy.get_param("~motion")
        self.map_name = rospy.get_param("~map_name")
        self.expected_gt_count = int(rospy.get_param("~expected_gt_count", 1))
        self.start = rospy.Time.now().to_sec()
        self.end = self.start + self.warmup + self.duration
        self.lock = threading.RLock()
        self.finished = False
        self.latest_odom = None
        self.gt_frames = []
        self.frame_index = 0
        self.previous_source_stamp = None
        self.previous_gt_positions = {}
        self.gt_track_map = {}
        self.track_manager = TrackManager(gate=0.8, max_missed=3)
        self.stats = {
            "frames": 0, "estimated_dropped_frames": 0,
            "unmatched_estimate_rows": 0, "unmatched_gt_rows": 0,
            "matched_rows": 0, "id_switch_events": 0,
            "gt_covered_frames": 0, "gt_missing_frames": 0,
        }
        self.gt_ids = set()
        self.track_ids = set()

        os.makedirs(self.root, exist_ok=True)
        raw_name = rospy.get_param(
            "~raw_filename",
            "static_obstacle_raw.csv" if self.scenario in ("static", "static_box")
            else "dynamic_obstacle_raw.csv")
        self.raw_path = os.path.join(self.root, raw_name)
        self.event_path = os.path.join(self.root, "tracking_events.csv")
        self.raw_handle = open(self.raw_path, "w", newline="")
        self.raw_writer = csv.DictWriter(self.raw_handle, fieldnames=RAW_FIELDS)
        self.raw_writer.writeheader()
        event_exists = os.path.exists(self.event_path) and os.path.getsize(self.event_path) > 0
        self.event_handle = open(self.event_path, "a", newline="")
        self.event_writer = csv.DictWriter(self.event_handle, fieldnames=EVENT_FIELDS)
        if not event_exists:
            self.event_writer.writeheader()

        rospy.Subscriber("/coni_mpc/quad_odom1", Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber("/quad0_pcl_render_node/dynamic_obstacle_states",
                         DynamicObstacleStateArray, self.gt_cb, queue_size=50)
        rospy.Subscriber("/onboard_detector/tracked_obstacles_raw",
                         TrackedObstacleArray, self.estimate_cb, queue_size=50)
        rospy.Timer(rospy.Duration(0.2), self.timer_cb)
        rospy.on_shutdown(self.close_files)
        rospy.loginfo("Raw collector ready: scenario=%s output=%s", self.scenario, self.raw_path)

    def odom_cb(self, msg):
        with self.lock:
            self.latest_odom = msg

    def gt_cb(self, msg):
        timestamp = stamp_sec(msg.header)
        obstacles = []
        for item in msg.obstacles:
            obstacle = {
                "id": str(item.obstacle_id), "is_dynamic": bool(item.is_dynamic),
                "geometry": item.geometry_type, "position": vec3(item.position),
                "velocity": vec3(item.velocity), "size": vec3(item.size), "radius": "",
            }
            prior = self.previous_gt_positions.get(obstacle["id"])
            if prior is not None and np.linalg.norm(prior - obstacle["position"]) > 1.0:
                self.write_event(timestamp, "gt_teleport", obstacle_id=obstacle["id"],
                                 details="same MARSIM ID moved more than 1.0 m between GT messages")
            self.previous_gt_positions[obstacle["id"]] = obstacle["position"]
            obstacles.append(obstacle)
        with self.lock:
            self.gt_frames.append((timestamp, obstacles))
            self.gt_frames = self.gt_frames[-50:]

    def nearest_gt(self, timestamp):
        with self.lock:
            if not self.gt_frames:
                return None, []
            gt_stamp, obstacles = min(self.gt_frames, key=lambda frame: abs(frame[0] - timestamp))
        if abs(gt_stamp - timestamp) > 0.15:
            return gt_stamp, []
        return gt_stamp, obstacles

    def uav_fields(self):
        values = {name: "" for name in
                  ("uav_x", "uav_y", "uav_z", "uav_vx", "uav_vy", "uav_vz")}
        with self.lock:
            msg = self.latest_odom
        if msg is not None:
            values.update({
                "uav_x": msg.pose.pose.position.x, "uav_y": msg.pose.pose.position.y,
                "uav_z": msg.pose.pose.position.z, "uav_vx": msg.twist.twist.linear.x,
                "uav_vy": msg.twist.twist.linear.y, "uav_vz": msg.twist.twist.linear.z,
            })
        return values

    def write_event(self, timestamp, event_type, obstacle_id="", tracking_id="",
                    slot_id="", details=""):
        with self.lock:
            if self.finished:
                return
            self.event_writer.writerow({
                "timestamp": "%.9f" % timestamp, "scenario": self.scenario,
                "frame_index": self.frame_index, "event_type": event_type,
                "obstacle_id": obstacle_id, "tracking_id": tracking_id,
                "lvdot_slot_id": slot_id, "details": details,
            })

    def base_row(self, timestamp, source_timestamp, gt_timestamp):
        row = {field: "" for field in RAW_FIELDS}
        row.update({
            "scenario": self.scenario, "frame_index": self.frame_index,
            "timestamp": "%.9f" % timestamp,
            "source_timestamp": "%.9f" % source_timestamp,
            "gt_timestamp": "%.9f" % gt_timestamp if gt_timestamp is not None else "",
        })
        row.update(self.uav_fields())
        return row

    @staticmethod
    def add_estimate(row, estimate, tracking_id):
        row.update({
            "tracking_id": tracking_id, "lvdot_slot_id": estimate["slot_id"],
            "est_is_dynamic": int(estimate["is_dynamic"]),
            "est_x": estimate["position"][0], "est_y": estimate["position"][1],
            "est_z": estimate["position"][2], "est_vx": estimate["velocity"][0],
            "est_vy": estimate["velocity"][1], "est_vz": estimate["velocity"][2],
            "est_lx": estimate["size"][0], "est_ly": estimate["size"][1],
            "est_lz": estimate["size"][2],
        })

    @staticmethod
    def add_gt(row, gt):
        row.update({
            "obstacle_id": gt["id"], "is_dynamic": int(gt["is_dynamic"]),
            "gt_x": gt["position"][0], "gt_y": gt["position"][1],
            "gt_z": gt["position"][2], "gt_vx": gt["velocity"][0],
            "gt_vy": gt["velocity"][1], "gt_vz": gt["velocity"][2],
            "gt_lx": gt["size"][0], "gt_ly": gt["size"][1], "gt_lz": gt["size"][2],
            "gt_geometry_type": gt["geometry"], "gt_radius": gt["radius"],
        })

    def estimate_cb(self, msg):
        now = rospy.Time.now().to_sec()
        if now < self.start + self.warmup or now > self.end:
            return
        source_timestamp = stamp_sec(msg.header)
        with self.lock:
            if self.finished:
                return
            self.frame_index += 1
            self.stats["frames"] += 1
            if self.previous_source_stamp is not None:
                gap = source_timestamp - self.previous_source_stamp
                missing = max(0, int(round(gap * 10.0)) - 1)
                if missing:
                    self.stats["estimated_dropped_frames"] += missing
                    self.write_event(source_timestamp, "frame_gap",
                                     details="estimated missing frames=%d; source gap=%.6f s" % (missing, gap))
            self.previous_source_stamp = source_timestamp

            estimates = [{
                "slot_id": int(item.slot_id), "is_dynamic": bool(item.is_dynamic),
                "position": vec3(item.position), "velocity": vec3(item.velocity),
                "size": vec3(item.size),
            } for item in msg.obstacles]
            assignments, track_events = self.track_manager.update(estimates, source_timestamp)
            for event_type, track_id, details in track_events:
                self.write_event(source_timestamp, event_type, tracking_id=track_id, details=details)
            self.track_ids.update(assignments.values())

            gt_timestamp, gt_obstacles = self.nearest_gt(source_timestamp)
            coverage_ok = (gt_timestamp is not None and
                           len(gt_obstacles) == self.expected_gt_count)
            if coverage_ok:
                self.stats["gt_covered_frames"] += 1
            else:
                self.stats["gt_missing_frames"] += 1
                self.write_event(source_timestamp, "gt_coverage_failure",
                                 details="expected %d GT boxes, received %d within 0.15 s" %
                                 (self.expected_gt_count, len(gt_obstacles)))
            self.gt_ids.update(gt["id"] for gt in gt_obstacles)
            matched_estimates = set()
            matched_gt = set()
            if estimates and gt_obstacles:
                cost = np.array([[np.linalg.norm(est["position"] - gt["position"])
                                  for gt in gt_obstacles] for est in estimates])
                rows, cols = linear_sum_assignment(cost)
                for est_index, gt_index in zip(rows, cols):
                    if cost[est_index, gt_index] > 1.5:
                        continue
                    matched_estimates.add(est_index)
                    matched_gt.add(gt_index)
                    estimate, gt = estimates[est_index], gt_obstacles[gt_index]
                    track_id = assignments[est_index]
                    row = self.base_row(now, source_timestamp, gt_timestamp)
                    row["association_status"] = "matched"
                    row["association_distance"] = cost[est_index, gt_index]
                    self.add_estimate(row, estimate, track_id)
                    self.add_gt(row, gt)
                    self.raw_writer.writerow(row)
                    self.stats["matched_rows"] += 1
                    previous_track = self.gt_track_map.get(gt["id"])
                    if previous_track is not None and previous_track != track_id:
                        self.stats["id_switch_events"] += 1
                        self.write_event(source_timestamp, "id_switch", gt["id"], track_id,
                                         estimate["slot_id"], "previous tracking_id=%s" % previous_track)
                    self.gt_track_map[gt["id"]] = track_id

            for est_index, estimate in enumerate(estimates):
                if est_index in matched_estimates:
                    continue
                track_id = assignments[est_index]
                row = self.base_row(now, source_timestamp, gt_timestamp)
                row.update({"obstacle_id": "unmatched_est_%s" % track_id,
                            "association_status": "unmatched_estimate"})
                self.add_estimate(row, estimate, track_id)
                self.raw_writer.writerow(row)
                self.stats["unmatched_estimate_rows"] += 1
                self.write_event(source_timestamp, "unmatched_detection",
                                 tracking_id=track_id, slot_id=estimate["slot_id"],
                                 details="no GT within 1.5 m")

            for gt_index, gt in enumerate(gt_obstacles):
                if gt_index in matched_gt:
                    continue
                row = self.base_row(now, source_timestamp, gt_timestamp)
                row["association_status"] = "unmatched_gt"
                self.add_gt(row, gt)
                self.raw_writer.writerow(row)
                self.stats["unmatched_gt_rows"] += 1
                self.write_event(source_timestamp, "unmatched_gt", obstacle_id=gt["id"],
                                 details="no LV-DOT estimate within 1.5 m")
            self.raw_handle.flush()
            self.event_handle.flush()
            if not coverage_ok:
                self.finish()

    def timer_cb(self, _event):
        if rospy.Time.now().to_sec() >= self.end:
            self.finish()

    def write_config(self):
        config_path = os.path.join(self.root, "experiment_config.json")
        try:
            with open(config_path) as handle:
                config = json.load(handle)
        except (IOError, ValueError):
            config = {}
        config.update({
            "purpose": "raw_data_collection_only",
            "analysis_performed": False,
            "acceptance_evaluation_performed": False,
            "gt_coverage_required": True,
            "association_rules": {
                "persistent_tracking_id": "collector-derived Hungarian assignment",
                "upstream_id_preserved_as": "lvdot_slot_id",
                "track_cost": "3D predicted-center distance + 0.1 * 3D size distance",
                "track_gate_m": 0.8, "track_max_missed_frames": 3,
                "estimate_gt_cost": "3D center Euclidean distance",
                "estimate_gt_gate_m": 1.5,
                "gt_time_pairing": "nearest GT timestamp within 0.15 s",
                "unmatched_policy": "write a raw-data row and tracking event; never discard silently",
            },
        })
        run = {
            "launch_commands_file": "launch_commands.txt", "map_name": self.map_name,
            "box_validation_parameters": rospy.get_param("/box_validation", {}),
            "uav_trajectory": {"motion": self.motion, "publisher": "lv_dot_validation_pose.py"},
            "duration_sec": self.duration, "warmup_sec": self.warmup,
            "obstacle_motion": {
                "enabled": bool(rospy.get_param("~dynamic_obstacles")),
                "count": int(rospy.get_param("~dynamic_obstacle_count")),
                "nominal_size_m": float(rospy.get_param("~dynamic_obstacle_size")),
                "nominal_velocity_mps": float(rospy.get_param("~dynamic_obstacle_velocity")),
            },
            "marsim_mid360_parameters": rospy.get_param("/quad0_pcl_render_node", {}),
            "lv_dot_parameters": rospy.get_param("/onboard_detector", {}),
            "collection": dict(self.stats, unique_gt_obstacles=len(self.gt_ids),
                               unique_collector_tracking_ids=len(self.track_ids)),
            "raw_file": os.path.basename(self.raw_path),
        }
        config.setdefault("runs", {})[self.scenario] = run
        config["files"] = sorted(set(config.get("files", []) + [
            os.path.basename(self.raw_path), "tracking_events.csv",
            "experiment_config.json", "launch_commands.txt",
        ]))
        temp_path = config_path + ".tmp"
        with open(temp_path, "w") as handle:
            json.dump(config, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temp_path, config_path)

    def close_files(self):
        with self.lock:
            if not self.raw_handle.closed:
                self.raw_handle.flush()
                self.raw_handle.close()
            if not self.event_handle.closed:
                self.event_handle.flush()
                self.event_handle.close()

    def finish(self):
        with self.lock:
            if self.finished:
                return
            self.write_config()
            self.finished = True
            rospy.loginfo("RAW_COLLECTION_COMPLETE scenario=%s frames=%d",
                          self.scenario, self.stats["frames"])
        rospy.signal_shutdown("raw collection duration complete")


if __name__ == "__main__":
    rospy.init_node("lv_dot_raw_collector")
    collector = RawCollector()
    rospy.spin()
