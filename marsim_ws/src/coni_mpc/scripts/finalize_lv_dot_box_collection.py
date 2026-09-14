#!/usr/bin/env python3
import argparse
import json
import os
import re


def scan_log(path):
    patterns = [
        ("mutex_shutdown_exception", re.compile(r"boost:+ mutex lock failed", re.I)),
        ("python_exception", re.compile(r"Traceback \(most recent call last\)", re.I)),
        ("ros_error", re.compile(r"\[ERROR\]|\bERROR:\s", re.I)),
    ]
    found = []
    with open(path, errors="replace") as handle:
        for line_number, line in enumerate(handle, 1):
            for name, pattern in patterns:
                if pattern.search(line):
                    found.append({"type": name, "line": line_number,
                                  "text": line.strip()[:500]})
                    break
    return found


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root")
    parser.add_argument("static_exit", type=int)
    parser.add_argument("dynamic_exit", type=int)
    args = parser.parse_args()
    config_path = os.path.join(args.root, "experiment_config.json")
    with open(config_path) as handle:
        config = json.load(handle)
    commands = {}
    with open(os.path.join(args.root, "launch_commands.txt")) as handle:
        for line in handle:
            match = re.match(r"\[(static_box|dynamic_box)\] (.*)", line.rstrip())
            if match:
                commands[match.group(1)] = match.group(2)
    status = {}
    for scenario, exit_code in (("static_box", args.static_exit),
                                ("dynamic_box", args.dynamic_exit)):
        run = config.get("runs", {}).get(scenario, {})
        stats = run.get("collection", {})
        run["launch_command"] = commands.get(scenario, "")
        status[scenario] = {
            "exit_code": exit_code,
            "collector_completed": bool(run),
            "gt_coverage_complete": bool(stats) and
                stats.get("gt_missing_frames") == 0 and
                stats.get("gt_covered_frames") == stats.get("frames"),
            "node_anomalies": scan_log(os.path.join(args.root, "%s_roslaunch.log" % scenario)),
        }
    config["run_status"] = status
    config["files"] = sorted(set(config.get("files", []) + [
        "moving_uav_static_box_raw.csv", "moving_uav_dynamic_box_raw.csv",
        "tracking_events.csv", "experiment_config.json", "launch_commands.txt",
        "static_box_map.pcd", "static_box_roslaunch.log", "dynamic_box_roslaunch.log",
    ]))
    temporary = config_path + ".tmp"
    with open(temporary, "w") as handle:
        json.dump(config, handle, indent=2, sort_keys=True)
        handle.write("\n")
    os.replace(temporary, config_path)


if __name__ == "__main__":
    main()
