#!/usr/bin/env python3
import json
import math
import os
import threading

import rosnode
import rospy
from onboard_detector.srv import GetDynamicObstacles, GetDynamicObstaclesRequest
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray


class Counter:
    def __init__(self):
        self.times = []
        self.nonempty = 0
        self.max_items = 0

    def add(self, count):
        self.times.append(rospy.Time.now().to_sec())
        if count > 0:
            self.nonempty += 1
        self.max_items = max(self.max_items, count)

    def rate(self):
        if len(self.times) < 2 or self.times[-1] <= self.times[0]:
            return 0.0
        return (len(self.times) - 1) / (self.times[-1] - self.times[0])

    def result(self):
        return {"messages": len(self.times), "rate_hz": self.rate(),
                "nonempty_messages": self.nonempty, "max_items": self.max_items}


class Validator:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {name: Counter() for name in
                     ("sensor_cloud", "downsampled_cloud", "lidar_bboxes",
                      "tracked_bboxes", "dynamic_bboxes", "dynamic_gt_cloud")}
        rospy.Subscriber("/quad0_pcl_render_node/sensor_cloud", PointCloud2,
                         self.cloud_cb, "sensor_cloud", queue_size=30)
        rospy.Subscriber("/onboard_detector/downsampled_point_cloud", PointCloud2,
                         self.cloud_cb, "downsampled_cloud", queue_size=30)
        rospy.Subscriber("/quad0_pcl_render_node/dyn_cloud", PointCloud2,
                         self.cloud_cb, "dynamic_gt_cloud", queue_size=30)
        for topic, key in (("/onboard_detector/lidar_bboxes", "lidar_bboxes"),
                           ("/onboard_detector/tracked_bboxes", "tracked_bboxes"),
                           ("/onboard_detector/dynamic_bboxes", "dynamic_bboxes")):
            rospy.Subscriber(topic, MarkerArray, self.marker_cb, key, queue_size=30)
        self.service_calls = 0
        self.service_nonempty = 0
        self.service_valid = 0
        self.service_max_obstacles = 0
        self.service_sample = None

    def cloud_cb(self, msg, key):
        with self.lock:
            self.data[key].add(msg.width * msg.height)

    def marker_cb(self, msg, key):
        with self.lock:
            self.data[key].add(len(msg.markers))

    def call_service(self):
        try:
            rospy.wait_for_service("/onboard_detector/get_dynamic_obstacles", timeout=0.5)
            proxy = rospy.ServiceProxy("/onboard_detector/get_dynamic_obstacles",
                                       GetDynamicObstacles)
            req = GetDynamicObstaclesRequest()
            req.current_position.x = 3.0
            req.current_position.y = 0.0
            req.current_position.z = 1.5
            req.range = 15.0
            res = proxy(req)
            self.service_calls += 1
            n = len(res.position)
            same_length = n == len(res.velocity) == len(res.size)
            finite = all(math.isfinite(v) for vec in
                         list(res.position) + list(res.velocity) + list(res.size)
                         for v in (vec.x, vec.y, vec.z))
            positive_size = all(s.x > 0 and s.y > 0 and s.z >= 0 for s in res.size)
            if same_length and finite and positive_size:
                self.service_valid += 1
            if n:
                self.service_nonempty += 1
                self.service_sample = {
                    "position": [[v.x, v.y, v.z] for v in res.position],
                    "velocity": [[v.x, v.y, v.z] for v in res.velocity],
                    "size": [[v.x, v.y, v.z] for v in res.size],
                }
            self.service_max_obstacles = max(self.service_max_obstacles, n)
        except (rospy.ROSException, rospy.ServiceException):
            pass


def runtime_parameters():
    names = {
        "is_360lidar": "/quad0_pcl_render_node/is_360lidar",
        "sensing_rate": "/quad0_pcl_render_node/sensing_rate",
        "vertical_fov": "/quad0_pcl_render_node/vertical_fov",
        "min_raylength": "/quad0_pcl_render_node/min_raylength",
        "use_minicf_pattern": "/quad0_pcl_render_node/use_minicf_pattern",
        "use_os128_pattern": "/quad0_pcl_render_node/use_os128_pattern",
        "lv_dot_cloud_topic": "/onboard_detector/lidar_pointcloud_topic",
        "lv_dot_odom_topic": "/onboard_detector/odom_topic",
        "lv_dot_time_step": "/onboard_detector/time_step",
    }
    return {key: rospy.get_param(name, None) for key, name in names.items()}


def main():
    rospy.init_node("lv_dot_mid360_validator")
    duration = float(rospy.get_param("~duration_sec", 20.0))
    warmup = float(rospy.get_param("~warmup_sec", 3.0))
    expect_dynamic = bool(rospy.get_param("~expect_dynamic", False))
    result_file = os.path.abspath(os.path.expanduser(
        rospy.get_param("~result_file", "results/lv_dot_mid360_validation/summary.json")))
    validator = Validator()
    rospy.sleep(warmup)
    start = rospy.Time.now()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < duration:
        validator.call_service()
        rate.sleep()

    params = runtime_parameters()
    expected = {
        "is_360lidar": 1, "sensing_rate": 10.0, "vertical_fov": 90.0,
        "min_raylength": 0.1, "use_minicf_pattern": 1, "use_os128_pattern": 0,
        "lv_dot_cloud_topic": "/quad0_pcl_render_node/sensor_cloud",
        "lv_dot_odom_topic": "/coni_mpc/quad_odom1", "lv_dot_time_step": 0.1,
    }
    checks = {
        "runtime_parameters": params == expected,
        "mid360_input_present": validator.data["sensor_cloud"].nonempty >= 10,
        "mid360_input_rate_8_to_12_hz": 8.0 <= validator.data["sensor_cloud"].rate() <= 12.0,
        "lv_dot_cloud_output_present": validator.data["downsampled_cloud"].nonempty >= 10,
        "lidar_bbox_detected": validator.data["lidar_bboxes"].nonempty > 0,
        "tracked_bbox_detected": validator.data["tracked_bboxes"].nonempty > 0,
        "service_available_and_well_formed": validator.service_calls > 0 and
                                             validator.service_valid == validator.service_calls,
        "depth_cbf_nodes_absent": not any("depth_cbf" in name for name in rosnode.get_node_names()),
    }
    if expect_dynamic:
        checks["marsim_dynamic_cloud_present"] = validator.data["dynamic_gt_cloud"].nonempty >= 10
        checks["dynamic_bbox_detected"] = validator.data["dynamic_bboxes"].nonempty > 0
        checks["dynamic_service_nonempty"] = validator.service_nonempty > 0
    summary = {
        "passed": all(checks.values()), "expect_dynamic": expect_dynamic,
        "checks": checks, "runtime_parameters": params,
        "topics": {key: value.result() for key, value in validator.data.items()},
        "service": {"calls": validator.service_calls,
                    "valid_calls": validator.service_valid,
                    "nonempty_calls": validator.service_nonempty,
                    "max_obstacles": validator.service_max_obstacles,
                    "sample": validator.service_sample},
    }
    os.makedirs(os.path.dirname(result_file), exist_ok=True)
    with open(result_file, "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
    rospy.loginfo("LV-DOT Mid-360 validation passed=%s result=%s", summary["passed"], result_file)
    for name, passed in checks.items():
        rospy.loginfo("  %-38s %s", name, "PASS" if passed else "FAIL")
    rospy.signal_shutdown("validation complete")


if __name__ == "__main__":
    main()
