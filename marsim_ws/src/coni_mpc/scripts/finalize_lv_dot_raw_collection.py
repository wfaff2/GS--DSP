#!/usr/bin/env python3
"""Record run status and node anomalies; no estimation-error analysis is performed."""

import argparse
import json
import os
import re


PATTERNS = [
    ("process_died", re.compile(r"process\[.*\] has died", re.I)),
    ("mutex_shutdown_exception", re.compile(r"boost:+ mutex lock failed", re.I)),
    ("python_exception", re.compile(r"Traceback \(most recent call last\)", re.I)),
    ("ros_error", re.compile(r"\bERROR\b|\[ERROR\]", re.I)),
]


def anomalies(path):
    found = []
    if not os.path.exists(path):
        return [{"type": "missing_log", "text": os.path.basename(path)}]
    with open(path, errors="replace") as handle:
        for line_number, line in enumerate(handle, 1):
            for name, pattern in PATTERNS:
                if pattern.search(line):
                    found.append({"type": name, "line": line_number, "text": line.strip()[:500]})
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
    command_path = os.path.join(args.root, "launch_commands.txt")
    commands = {}
    if os.path.exists(command_path):
        with open(command_path) as handle:
            for line in handle:
                match = re.match(r"\[(static|dynamic)\] (.*)", line.rstrip("\n"))
                if match:
                    commands[match.group(1)] = match.group(2)
    status = {}
    for scenario, exit_code in (("static", args.static_exit), ("dynamic", args.dynamic_exit)):
        log_name = "%s_roslaunch.log" % scenario
        run_anomalies = anomalies(os.path.join(args.root, log_name))
        raw_name = "%s_obstacle_raw.csv" % scenario
        status[scenario] = {
            "exit_code": exit_code,
            "collector_completed": scenario in config.get("runs", {}),
            "raw_file_exists": os.path.exists(os.path.join(args.root, raw_name)),
            "node_anomalies": run_anomalies,
        }
        config.get("runs", {}).get(scenario, {})["launch_command"] = commands.get(scenario, "")
    config["run_status"] = status
    config["files"] = sorted(set(config.get("files", []) +
                                 ["static_roslaunch.log", "dynamic_roslaunch.log"]))
    temp = config_path + ".tmp"
    with open(temp, "w") as handle:
        json.dump(config, handle, indent=2, sort_keys=True)
        handle.write("\n")
    os.replace(temp, config_path)


if __name__ == "__main__":
    main()
