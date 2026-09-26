#!/usr/bin/env python3
"""
Verify live-compare batches for:
- selected-top3 coverage of the eventual collided static obstacle
- slack growth
- obstacle switch frequency
- scheduler overrun behavior

Typical usage:
  python3 scripts/verify_live_compare_batch.py \
    --reference-batch results/live_compare_rviz_20260313-110916 \
    --target-batch results/live_compare_rviz_20260313-133250
"""

from __future__ import annotations

import argparse
import collections
import csv
import json
import math
import pathlib
import re
import statistics
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from typing import Any


METRICS_MODES = ("noninertial", "inertial")

SCHEDULER_SUMMARY_RE = re.compile(
    r"MPC scheduler summary: cycles=(\d+) mean_cycle_ms=([0-9.]+) "
    r"max_cycle_ms=([0-9.]+) overruns=(\d+) max_overrun_ms=([0-9.]+)"
)
SELECT_RE = re.compile(r"\[(\d+\.\d+)\]: .*?cbf_obstacle_select keys=([^ ]+)")
EST_RE = re.compile(r"\[(\d+\.\d+)\]: .*?\[MPC UAV (\d+)\] est_p=")
SWITCH_RE = re.compile(
    r"\[(\d+\.\d+)\]: .*?\[MPC UAV (\d+)\] active obstacle switched with warm start "
    r".*? curr_key=([^\x1b\r\n ]+)"
)
COLLISION_RE = re.compile(
    r"\[(\d+\.\d+)\]: .*?Collision detected for UAV (\d+): .*? "
    r"obstacle_pos=\(([-0-9.eE+]+), ([-0-9.eE+]+), ([-0-9.eE+]+)\)"
)


@dataclass
class ObstacleConfig:
    count: int
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    h_min: float
    h_max: float
    radius: float
    uav_radius: float


@dataclass
class RunMetrics:
    seed: int
    mode: str
    duration_sec: float
    collision: int
    solver_slack_sum: float
    solver_max_slack: float


@dataclass
class SchedulerSummary:
    cycles: int
    mean_cycle_ms: float
    max_cycle_ms: float
    overruns: int
    max_overrun_ms: float

    @property
    def overrun_rate(self) -> float:
        if self.cycles <= 0:
            return math.nan
        return self.overruns / self.cycles


@dataclass
class CollisionCoverage:
    seed: int
    mode: str
    uav: int
    static_key: str
    map_error: float
    state_time: float | None
    state_age_sec: float | None
    state_source: str | None
    state_keys: list[str] | None
    included: bool | None


def _safe_float(value: str | None) -> float:
    try:
        result = float(value if value is not None else "nan")
    except Exception:
        return math.nan
    return result if math.isfinite(result) else math.nan


def _safe_int(value: str | None) -> int:
    try:
        return int(float(value if value is not None else "nan"))
    except Exception:
        return 0


def _fmt_float(value: float) -> str:
    return "nan" if not math.isfinite(value) else f"{value:.6f}"


def _mean(values: list[float]) -> float:
    values = [v for v in values if math.isfinite(v)]
    if not values:
        return math.nan
    return statistics.mean(values)


def _median(values: list[float]) -> float:
    values = [v for v in values if math.isfinite(v)]
    if not values:
        return math.nan
    return statistics.median(values)


def _count_close(values: list[float], target: float, tol: float = 1e-9) -> int:
    return sum(1 for value in values if math.isfinite(value) and abs(value - target) <= tol)


def _load_yaml_config(yaml_path: pathlib.Path) -> ObstacleConfig:
    default = ObstacleConfig(
        count=100,
        x_min=-10.0,
        x_max=10.0,
        y_min=-10.0,
        y_max=10.0,
        h_min=2.5,
        h_max=3.0,
        radius=0.45,
        uav_radius=0.15,
    )
    if not yaml_path.is_file():
        return default

    try:
        import yaml  # type: ignore
    except Exception:
        return default

    try:
        with yaml_path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return default

    random_obstacles = data.get("random_obstacles") or {}
    x_range = random_obstacles.get("x_range") or [default.x_min, default.x_max]
    y_range = random_obstacles.get("y_range") or [default.y_min, default.y_max]
    h_range = random_obstacles.get("height_range") or [default.h_min, default.h_max]
    return ObstacleConfig(
        count=int(random_obstacles.get("count", default.count)),
        x_min=float(x_range[0]),
        x_max=float(x_range[1]),
        y_min=float(y_range[0]),
        y_max=float(y_range[1]),
        h_min=float(h_range[0]),
        h_max=float(h_range[1]),
        radius=float(random_obstacles.get("radius", default.radius)),
        uav_radius=float(data.get("uav_radius", default.uav_radius)),
    )


def _ensure_obstacle_helper(helper_path: pathlib.Path) -> pathlib.Path:
    if helper_path.is_file():
        return helper_path

    helper_path.parent.mkdir(parents=True, exist_ok=True)
    code = r"""
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

struct Obstacle {
  double x;
  double y;
  double z;
};

int main(int argc, char** argv) {
  if (argc != 11) {
    return 2;
  }
  const int seed = std::stoi(argv[1]);
  const int count = std::stoi(argv[2]);
  const double x_min = std::stod(argv[3]);
  const double x_max = std::stod(argv[4]);
  const double y_min = std::stod(argv[5]);
  const double y_max = std::stod(argv[6]);
  const double h_min = std::stod(argv[7]);
  const double h_max = std::stod(argv[8]);
  const double radius = std::stod(argv[9]);
  const double uav_radius = std::stod(argv[10]);

  std::mt19937 gen(static_cast<uint32_t>(seed));
  std::uniform_real_distribution<double> dist_x(x_min, x_max);
  std::uniform_real_distribution<double> dist_y(y_min, y_max);
  std::uniform_real_distribution<double> dist_h(h_min, h_max);

  std::vector<Obstacle> obstacles;
  obstacles.reserve(static_cast<size_t>(count));
  (void)uav_radius;
  const double min_center_dist = 2.0 * radius + 1.0;
  int attempts = 0;
  const int max_attempts = count * 20;
  while (static_cast<int>(obstacles.size()) < count && attempts < max_attempts) {
    attempts++;
    const double x = dist_x(gen);
    const double y = dist_y(gen);
    const double h = dist_h(gen);
    bool overlaps = false;
    for (const auto& obstacle : obstacles) {
      const double dx = obstacle.x - x;
      const double dy = obstacle.y - y;
      if (std::sqrt(dx * dx + dy * dy) < min_center_dist) {
        overlaps = true;
        break;
      }
    }
    if (overlaps) {
      continue;
    }
    obstacles.push_back({x, y, 0.5 * h});
  }

  std::cout << std::fixed << std::setprecision(8);
  for (size_t idx = 0; idx < obstacles.size(); ++idx) {
    const auto& obstacle = obstacles[idx];
    std::cout << idx << "," << obstacle.x << "," << obstacle.y << "," << obstacle.z << "\n";
  }
  return 0;
}
"""

    with tempfile.NamedTemporaryFile("w", encoding="utf-8", suffix=".cpp", delete=False) as tmp:
        tmp.write(code)
        src_path = pathlib.Path(tmp.name)
    try:
        subprocess.run(
            ["g++", "-std=c++17", "-O2", str(src_path), "-o", str(helper_path)],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    finally:
        src_path.unlink(missing_ok=True)
    return helper_path


def _generate_static_obstacles(
    helper_path: pathlib.Path,
    seed: int,
    obstacle_cfg: ObstacleConfig,
) -> list[tuple[int, float, float, float]]:
    proc = subprocess.run(
        [
            str(helper_path),
            str(seed),
            str(obstacle_cfg.count),
            str(obstacle_cfg.x_min),
            str(obstacle_cfg.x_max),
            str(obstacle_cfg.y_min),
            str(obstacle_cfg.y_max),
            str(obstacle_cfg.h_min),
            str(obstacle_cfg.h_max),
            str(obstacle_cfg.radius),
            str(obstacle_cfg.uav_radius),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    obstacles: list[tuple[int, float, float, float]] = []
    for line in proc.stdout.splitlines():
        if not line.strip():
            continue
        idx_str, x_str, y_str, z_str = line.split(",")
        obstacles.append((int(idx_str), float(x_str), float(y_str), float(z_str)))
    return obstacles


def _nearest_static_key(
    helper_path: pathlib.Path,
    obstacle_cfg: ObstacleConfig,
    cache: dict[int, list[tuple[int, float, float, float]]],
    seed: int,
    x: float,
    y: float,
    z: float,
) -> tuple[str, float]:
    obstacles = cache.setdefault(seed, _generate_static_obstacles(helper_path, seed, obstacle_cfg))
    best_idx = -1
    best_dist = math.inf
    for idx, obs_x, obs_y, obs_z in obstacles:
        dist = math.sqrt((obs_x - x) ** 2 + (obs_y - y) ** 2 + (obs_z - z) ** 2)
        if dist < best_dist:
            best_dist = dist
            best_idx = idx
    return f"static:{best_idx}", best_dist


def _read_all_scope_metrics(metrics_csv: pathlib.Path) -> RunMetrics | None:
    if not metrics_csv.is_file():
        return None

    seed = _safe_int(metrics_csv.parent.name.split("_")[-1])
    mode = metrics_csv.stem.replace("_metrics", "")
    with metrics_csv.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    row = next((entry for entry in rows if entry.get("scope") == "all"), None)
    if row is None:
        return None
    return RunMetrics(
        seed=seed,
        mode=mode,
        duration_sec=_safe_float(row.get("simulation_duration_sec")),
        collision=_safe_int(row.get("collision")),
        solver_slack_sum=_safe_float(row.get("solver_slack_sum")),
        solver_max_slack=_safe_float(row.get("solver_max_slack")),
    )


def _parse_scheduler_summary(log_text: str) -> SchedulerSummary | None:
    match = SCHEDULER_SUMMARY_RE.search(log_text)
    if match is None:
        return None
    return SchedulerSummary(
        cycles=int(match.group(1)),
        mean_cycle_ms=float(match.group(2)),
        max_cycle_ms=float(match.group(3)),
        overruns=int(match.group(4)),
        max_overrun_ms=float(match.group(5)),
    )


def _extract_collision_coverage(
    log_text: str,
    seed: int,
    mode: str,
    helper_path: pathlib.Path,
    obstacle_cfg: ObstacleConfig,
    obstacle_cache: dict[int, list[tuple[int, float, float, float]]],
) -> list[CollisionCoverage]:
    events: list[tuple[float, int, int, str, Any]] = []
    for match in SELECT_RE.finditer(log_text):
        keys = [] if match.group(2) == "none" else match.group(2).split("|")
        events.append((float(match.group(1)), 0, match.start(), "select", keys))
    for match in EST_RE.finditer(log_text):
        events.append((float(match.group(1)), 1, match.start(), "est", int(match.group(2))))
    for match in SWITCH_RE.finditer(log_text):
        keys = [] if match.group(3) == "none" else match.group(3).split("|")
        events.append(
            (float(match.group(1)), 2, match.start(), "switch", (int(match.group(2)), keys))
        )
    for match in COLLISION_RE.finditer(log_text):
        events.append(
            (
                float(match.group(1)),
                3,
                match.start(),
                "collision",
                (
                    int(match.group(2)),
                    float(match.group(3)),
                    float(match.group(4)),
                    float(match.group(5)),
                ),
            )
        )
    events.sort(key=lambda item: (item[0], item[1], item[2]))

    pending_selects: collections.deque[tuple[float, list[str]]] = collections.deque()
    current_keys: dict[int, tuple[float, list[str], str]] = {}
    coverage_rows: list[CollisionCoverage] = []

    # Selection logs do not include quad_id, so pair them with the next est_p line
    # in the same log stream. This matches how the node emits them in practice.
    for timestamp, _, _, kind, payload in events:
        if kind == "select":
            pending_selects.append((timestamp, payload))
            continue
        if kind == "est":
            if pending_selects:
                select_time, keys = pending_selects.popleft()
                current_keys[payload] = (select_time, keys, "selection")
            continue
        if kind == "switch":
            uav, keys = payload
            current_keys[uav] = (timestamp, keys, "switch")
            continue
        if kind != "collision":
            continue

        uav, obs_x, obs_y, obs_z = payload
        static_key, map_error = _nearest_static_key(
            helper_path=helper_path,
            obstacle_cfg=obstacle_cfg,
            cache=obstacle_cache,
            seed=seed,
            x=obs_x,
            y=obs_y,
            z=obs_z,
        )
        snapshot = current_keys.get(uav)
        coverage_rows.append(
            CollisionCoverage(
                seed=seed,
                mode=mode,
                uav=uav,
                static_key=static_key,
                map_error=map_error,
                state_time=None if snapshot is None else snapshot[0],
                state_age_sec=None if snapshot is None else timestamp - snapshot[0],
                state_source=None if snapshot is None else snapshot[2],
                state_keys=None if snapshot is None else snapshot[1],
                included=None if snapshot is None else static_key in snapshot[1],
            )
        )
    return coverage_rows


def _analyze_batch(
    batch_dir: pathlib.Path,
    helper_path: pathlib.Path,
    obstacle_cfg: ObstacleConfig,
) -> dict[str, Any]:
    obstacle_cache: dict[int, list[tuple[int, float, float, float]]] = {}
    per_mode: dict[str, dict[str, Any]] = {
        mode: {
            "metrics": [],
            "switch_count": [],
            "switch_rate_per_sec": [],
            "activation_count": [],
            "activation_rate_per_sec": [],
            "scheduler": [],
            "collisions": [],
        }
        for mode in METRICS_MODES
    }

    for seed_dir in sorted(batch_dir.glob("seed_*")):
        if not seed_dir.is_dir():
            continue
        for mode in METRICS_MODES:
            metrics_csv = seed_dir / f"{mode}_metrics.csv"
            metrics = _read_all_scope_metrics(metrics_csv)
            if metrics is None:
                continue
            log_path = seed_dir / f"{mode}.log"
            log_text = log_path.read_text(encoding="utf-8", errors="ignore") if log_path.is_file() else ""

            switch_count = log_text.count("active obstacle switched with warm start")
            activation_count = log_text.count("cbf activation changed")
            scheduler_summary = _parse_scheduler_summary(log_text)
            collisions = _extract_collision_coverage(
                log_text=log_text,
                seed=metrics.seed,
                mode=mode,
                helper_path=helper_path,
                obstacle_cfg=obstacle_cfg,
                obstacle_cache=obstacle_cache,
            )

            per_mode[mode]["metrics"].append(metrics)
            per_mode[mode]["switch_count"].append(switch_count)
            per_mode[mode]["switch_rate_per_sec"].append(
                switch_count / metrics.duration_sec if metrics.duration_sec > 0.0 else math.nan
            )
            per_mode[mode]["activation_count"].append(activation_count)
            per_mode[mode]["activation_rate_per_sec"].append(
                activation_count / metrics.duration_sec if metrics.duration_sec > 0.0 else math.nan
            )
            if scheduler_summary is not None:
                per_mode[mode]["scheduler"].append(scheduler_summary)
            per_mode[mode]["collisions"].extend(collisions)

    summary: dict[str, Any] = {"batch_dir": str(batch_dir.resolve()), "modes": {}}
    for mode in METRICS_MODES:
        metrics_rows: list[RunMetrics] = per_mode[mode]["metrics"]
        slack_sum_values = [row.solver_slack_sum for row in metrics_rows]
        max_slack_values = [row.solver_max_slack for row in metrics_rows]
        collision_values = [row.collision for row in metrics_rows]
        scheduler_rows: list[SchedulerSummary] = per_mode[mode]["scheduler"]

        summary["modes"][mode] = {
            "run_count": len(metrics_rows),
            "collision_count": sum(collision_values),
            "collision_rate": (
                sum(collision_values) / len(collision_values) if collision_values else math.nan
            ),
            "duration_mean_sec": _mean([row.duration_sec for row in metrics_rows]),
            "solver_slack_sum_mean": _mean(slack_sum_values),
            "solver_slack_sum_median": _median(slack_sum_values),
            "solver_max_slack_mean": _mean(max_slack_values),
            "solver_max_slack_median": _median(max_slack_values),
            "solver_max_slack_eq_10_count": _count_close(max_slack_values, 10.0),
            "switch_count_mean": _mean(per_mode[mode]["switch_count"]),
            "switch_count_median": _median(per_mode[mode]["switch_count"]),
            "switch_rate_per_sec_mean": _mean(per_mode[mode]["switch_rate_per_sec"]),
            "activation_count_mean": _mean(per_mode[mode]["activation_count"]),
            "activation_rate_per_sec_mean": _mean(per_mode[mode]["activation_rate_per_sec"]),
            "scheduler_mean_cycle_ms": _mean([row.mean_cycle_ms for row in scheduler_rows]),
            "scheduler_mean_max_cycle_ms": _mean([row.max_cycle_ms for row in scheduler_rows]),
            "scheduler_mean_overrun_rate": _mean([row.overrun_rate for row in scheduler_rows]),
            "scheduler_mean_max_overrun_ms": _mean(
                [row.max_overrun_ms for row in scheduler_rows]
            ),
            "collision_coverage": per_mode[mode]["collisions"],
        }
    return summary


def _summarize_top3(
    batch_summary: dict[str, Any],
    fresh_window_sec: float,
) -> dict[str, Any]:
    rows: list[CollisionCoverage] = []
    for mode in METRICS_MODES:
        rows.extend(batch_summary["modes"][mode]["collision_coverage"])

    fresh_rows = [
        row
        for row in rows
        if row.state_age_sec is not None and math.isfinite(row.state_age_sec) and row.state_age_sec <= fresh_window_sec
    ]
    stale_rows = [
        row
        for row in rows
        if row.state_age_sec is None or not math.isfinite(row.state_age_sec) or row.state_age_sec > fresh_window_sec
    ]

    def summarize(group: list[CollisionCoverage]) -> dict[str, Any]:
        hits = sum(1 for row in group if row.included is True)
        misses = sum(1 for row in group if row.included is False)
        unknown = sum(1 for row in group if row.included is None)
        return {"total": len(group), "hit": hits, "miss": misses, "unknown": unknown}

    miss_details = [
        {
            "seed": row.seed,
            "mode": row.mode,
            "uav": row.uav,
            "static_key": row.static_key,
            "state_age_sec": row.state_age_sec,
            "state_source": row.state_source,
            "state_keys": row.state_keys,
        }
        for row in rows
        if row.included is False
    ]
    stale_details = [
        {
            "seed": row.seed,
            "mode": row.mode,
            "uav": row.uav,
            "static_key": row.static_key,
            "state_age_sec": row.state_age_sec,
            "state_source": row.state_source,
            "state_keys": row.state_keys,
        }
        for row in stale_rows
    ]
    return {
        "all_collisions": summarize(rows),
        "fresh_window_sec": fresh_window_sec,
        "fresh_collisions": summarize(fresh_rows),
        "stale_collisions": summarize(stale_rows),
        "miss_details": miss_details,
        "stale_details": stale_details,
    }


def _build_report(
    target_summary: dict[str, Any],
    reference_summary: dict[str, Any] | None,
    fresh_window_sec: float,
) -> dict[str, Any]:
    report = {
        "target_batch": target_summary["batch_dir"],
        "reference_batch": None if reference_summary is None else reference_summary["batch_dir"],
        "fresh_window_sec": fresh_window_sec,
        "modes": {},
        "top3": _summarize_top3(target_summary, fresh_window_sec=fresh_window_sec),
    }
    for mode in METRICS_MODES:
        target_mode = target_summary["modes"][mode]
        reference_mode = None if reference_summary is None else reference_summary["modes"][mode]
        report["modes"][mode] = {
            "target": {
                key: value
                for key, value in target_mode.items()
                if key != "collision_coverage"
            },
            "reference": None
            if reference_mode is None
            else {
                key: value
                for key, value in reference_mode.items()
                if key != "collision_coverage"
            },
        }
    return report


def _print_mode_report(mode: str, payload: dict[str, Any]) -> None:
    target = payload["target"]
    reference = payload["reference"]
    print(f"{mode}:")
    print(
        "  target"
        f" runs={target['run_count']}"
        f" collisions={target['collision_count']}/{target['run_count']}"
        f" collision_rate={_fmt_float(target['collision_rate'])}"
    )
    print(
        "  target"
        f" slack_sum_median={_fmt_float(target['solver_slack_sum_median'])}"
        f" slack_sum_mean={_fmt_float(target['solver_slack_sum_mean'])}"
        f" max_slack_median={_fmt_float(target['solver_max_slack_median'])}"
        f" max_slack_eq10={target['solver_max_slack_eq_10_count']}"
    )
    print(
        "  target"
        f" switch_mean={_fmt_float(target['switch_count_mean'])}"
        f" switch_rate_per_s={_fmt_float(target['switch_rate_per_sec_mean'])}"
        f" activation_rate_per_s={_fmt_float(target['activation_rate_per_sec_mean'])}"
    )
    print(
        "  target"
        f" overrun_rate={_fmt_float(target['scheduler_mean_overrun_rate'])}"
        f" mean_cycle_ms={_fmt_float(target['scheduler_mean_cycle_ms'])}"
        f" mean_max_overrun_ms={_fmt_float(target['scheduler_mean_max_overrun_ms'])}"
    )
    if reference is None:
        return
    print(
        "  reference"
        f" runs={reference['run_count']}"
        f" collisions={reference['collision_count']}/{reference['run_count']}"
        f" collision_rate={_fmt_float(reference['collision_rate'])}"
    )
    print(
        "  reference"
        f" slack_sum_median={_fmt_float(reference['solver_slack_sum_median'])}"
        f" slack_sum_mean={_fmt_float(reference['solver_slack_sum_mean'])}"
        f" max_slack_median={_fmt_float(reference['solver_max_slack_median'])}"
        f" max_slack_eq10={reference['solver_max_slack_eq_10_count']}"
    )
    print(
        "  reference"
        f" switch_mean={_fmt_float(reference['switch_count_mean'])}"
        f" switch_rate_per_s={_fmt_float(reference['switch_rate_per_sec_mean'])}"
        f" activation_rate_per_s={_fmt_float(reference['activation_rate_per_sec_mean'])}"
    )
    print(
        "  reference"
        f" overrun_rate={_fmt_float(reference['scheduler_mean_overrun_rate'])}"
        f" mean_cycle_ms={_fmt_float(reference['scheduler_mean_cycle_ms'])}"
        f" mean_max_overrun_ms={_fmt_float(reference['scheduler_mean_max_overrun_ms'])}"
    )


def _print_report(report: dict[str, Any]) -> None:
    print(f"target_batch: {report['target_batch']}")
    if report["reference_batch"] is not None:
        print(f"reference_batch: {report['reference_batch']}")
    print(f"fresh_window_sec: {report['fresh_window_sec']:.3f}")
    print()
    print("mode_summary:")
    for mode in METRICS_MODES:
        _print_mode_report(mode, report["modes"][mode])
    print()
    top3 = report["top3"]
    print("top3_collision_coverage:")
    print(
        "  all"
        f" total={top3['all_collisions']['total']}"
        f" hit={top3['all_collisions']['hit']}"
        f" miss={top3['all_collisions']['miss']}"
        f" unknown={top3['all_collisions']['unknown']}"
    )
    print(
        "  fresh"
        f" total={top3['fresh_collisions']['total']}"
        f" hit={top3['fresh_collisions']['hit']}"
        f" miss={top3['fresh_collisions']['miss']}"
        f" unknown={top3['fresh_collisions']['unknown']}"
    )
    print(
        "  stale"
        f" total={top3['stale_collisions']['total']}"
        f" hit={top3['stale_collisions']['hit']}"
        f" miss={top3['stale_collisions']['miss']}"
        f" unknown={top3['stale_collisions']['unknown']}"
    )
    if top3["miss_details"]:
        print("  misses:")
        for row in top3["miss_details"]:
            print(
                "    "
                f"seed={row['seed']} mode={row['mode']} uav={row['uav']}"
                f" key={row['static_key']}"
                f" age={_fmt_float(_safe_float(str(row['state_age_sec'])))}"
                f" source={row['state_source']}"
                f" keys={row['state_keys']}"
            )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify live-compare batches for top3 coverage, slack, switch, and scheduler overrun."
    )
    parser.add_argument(
        "--target-batch",
        required=True,
        help="Batch directory to verify, e.g. results/live_compare_rviz_20260313-133250",
    )
    parser.add_argument(
        "--reference-batch",
        default=None,
        help="Optional baseline batch directory for side-by-side comparison.",
    )
    parser.add_argument(
        "--yaml",
        default="src/coni_mpc/parameters/num_sim_non_one_point.yaml",
        help="YAML used to recover random obstacle defaults.",
    )
    parser.add_argument(
        "--fresh-window-sec",
        type=float,
        default=2.0,
        help="Selection-state age threshold used to classify fresh vs stale top3 evidence.",
    )
    parser.add_argument(
        "--helper-path",
        default=".cache/verify_live_compare_batch/gen_obstacles_helper",
        help="Path for the small C++ helper used to reproduce static obstacle indices.",
    )
    parser.add_argument(
        "--json-out",
        default=None,
        help="Optional JSON output path.",
    )
    args = parser.parse_args()

    target_batch = pathlib.Path(args.target_batch)
    if not target_batch.is_dir():
        print(f"target batch not found: {target_batch}", file=sys.stderr)
        return 2

    reference_batch = None if args.reference_batch is None else pathlib.Path(args.reference_batch)
    if reference_batch is not None and not reference_batch.is_dir():
        print(f"reference batch not found: {reference_batch}", file=sys.stderr)
        return 2

    obstacle_cfg = _load_yaml_config(pathlib.Path(args.yaml))
    helper_path = _ensure_obstacle_helper(pathlib.Path(args.helper_path))
    target_summary = _analyze_batch(target_batch, helper_path=helper_path, obstacle_cfg=obstacle_cfg)
    reference_summary = (
        None
        if reference_batch is None
        else _analyze_batch(reference_batch, helper_path=helper_path, obstacle_cfg=obstacle_cfg)
    )
    report = _build_report(
        target_summary=target_summary,
        reference_summary=reference_summary,
        fresh_window_sec=args.fresh_window_sec,
    )
    _print_report(report)

    if args.json_out:
        json_path = pathlib.Path(args.json_out)
        json_path.parent.mkdir(parents=True, exist_ok=True)
        with json_path.open("w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
            f.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
