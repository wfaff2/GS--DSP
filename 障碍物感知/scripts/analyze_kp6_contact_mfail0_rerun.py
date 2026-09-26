#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple


CONTROLLERS = ("noninertial_frozen", "noninertial_stage")
CONDITIONS = ("map", "online_nominal", "online_degraded")
EXPECTED_OLD_COUNTS = {
    ("map", "noninertial_frozen"): 66,
    ("map", "noninertial_stage"): 95,
    ("online_nominal", "noninertial_frozen"): 65,
    ("online_nominal", "noninertial_stage"): 96,
    ("online_degraded", "noninertial_frozen"): 69,
    ("online_degraded", "noninertial_stage"): 92,
}


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description="Prepare and analyze the K_P=6 m_fail=0 event-subset rerun."
    )
    subparsers = result.add_subparsers(dest="command", required=True)
    for name in ("prepare", "analyze"):
        sub = subparsers.add_parser(name)
        sub.add_argument("--old-map-root", required=True)
        sub.add_argument("--old-online-root", required=True)
        sub.add_argument("--rerun-root", required=True)
    seeds = subparsers.add_parser("seeds")
    seeds.add_argument("--rerun-root", required=True)
    seeds.add_argument("--condition", choices=CONDITIONS, required=True)
    seeds.add_argument("--controller", choices=CONTROLLERS, required=True)
    return result


def read_csv(path: Path) -> List[Dict[str, str]]:
    if not path.is_file():
        raise FileNotFoundError(path)
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Iterable[Mapping[str, object]], fields: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fields))
        writer.writeheader()
        writer.writerows(rows)


def as_int(value: str) -> int:
    return int(float(value))


def old_map_seeds(root: Path, controller: str) -> List[int]:
    selected: List[int] = []
    for seed in range(1, 101):
        path = (
            root
            / "logs"
            / controller
            / "v_3"
            / "w_3"
            / f"seed_{seed:03d}"
            / "metrics.csv"
        )
        rows = read_csv(path)
        aggregate = next((row for row in rows if row.get("scope") == "all"), None)
        if aggregate is None:
            raise RuntimeError(f"No scope=all row in {path}")
        if as_int(aggregate["collision"]) == 1:
            selected.append(seed)
    return selected


def old_online_seeds(root: Path, condition: str, controller: str) -> List[int]:
    path = root / condition / "tables" / "per_seed_controller_metrics.csv"
    rows = read_csv(path)
    selected = sorted(
        as_int(row["seed"])
        for row in rows
        if row.get("controller") == controller
        and math.isclose(float(row["v"]), 3.0)
        and math.isclose(float(row["w"]), 3.0)
        and as_int(row["collision"]) == 1
    )
    if len(selected) != len(set(selected)):
        raise RuntimeError(f"Duplicate selected seeds in {path}: {controller}")
    return selected


def extract_source_seeds(
    old_map_root: Path, old_online_root: Path
) -> Dict[Tuple[str, str], List[int]]:
    result: Dict[Tuple[str, str], List[int]] = {}
    for condition in CONDITIONS:
        for controller in CONTROLLERS:
            if condition == "map":
                seeds = old_map_seeds(old_map_root, controller)
            else:
                seeds = old_online_seeds(old_online_root, condition, controller)
            expected = EXPECTED_OLD_COUNTS[(condition, controller)]
            if len(seeds) != expected:
                raise RuntimeError(
                    f"Old event count changed for {condition}/{controller}: "
                    f"expected {expected}, found {len(seeds)}"
                )
            result[(condition, controller)] = seeds
    return result


def manifest_path(rerun_root: Path) -> Path:
    return rerun_root / "source_old_event_seed_manifest.csv"


def prepare(args: argparse.Namespace) -> int:
    old_map_root = Path(args.old_map_root).resolve()
    old_online_root = Path(args.old_online_root).resolve()
    rerun_root = Path(args.rerun_root).resolve()
    source = extract_source_seeds(old_map_root, old_online_root)
    rows = []
    for condition in CONDITIONS:
        for controller in CONTROLLERS:
            seeds = source[(condition, controller)]
            rows.append(
                {
                    "condition": condition,
                    "sensing_mode": condition,
                    "controller": controller,
                    "old_event_count": len(seeds),
                    "seeds": ",".join(map(str, seeds)),
                    "source_root": str(
                        old_map_root if condition == "map" else old_online_root
                    ),
                }
            )
    write_csv(
        manifest_path(rerun_root),
        rows,
        (
            "condition",
            "sensing_mode",
            "controller",
            "old_event_count",
            "seeds",
            "source_root",
        ),
    )
    print(f"Prepared {sum(len(value) for value in source.values())} cases")
    for row in rows:
        print(
            f"{row['condition']}/{row['controller']}: "
            f"{row['old_event_count']} cases"
        )
    return 0


def manifest_seeds(rerun_root: Path, condition: str, controller: str) -> List[int]:
    rows = read_csv(manifest_path(rerun_root))
    matches = [
        row
        for row in rows
        if row["condition"] == condition and row["controller"] == controller
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected one manifest row for {condition}/{controller}, found {len(matches)}"
        )
    return [int(item) for item in matches[0]["seeds"].split(",") if item]


def rerun_collision_rows(
    rerun_root: Path, condition: str, controller: str
) -> Dict[int, Dict[str, str]]:
    path = rerun_root / condition / "tables" / "per_seed_controller_metrics.csv"
    rows = [
        row
        for row in read_csv(path)
        if row.get("controller") == controller
        and math.isclose(float(row["v"]), 3.0)
        and math.isclose(float(row["w"]), 3.0)
    ]
    result: Dict[int, Dict[str, str]] = {}
    for row in rows:
        seed = as_int(row["seed"])
        if seed in result:
            raise RuntimeError(f"Duplicate rerun row for {condition}/{controller}/seed {seed}")
        if row.get("status") != "ok":
            raise RuntimeError(
                f"Non-ok rerun row for {condition}/{controller}/seed {seed}: "
                f"{row.get('status')}"
            )
        result[seed] = row
    return result


def rerun_manifest_rows(
    rerun_root: Path, condition: str, controller: str
) -> Dict[int, Dict[str, str]]:
    path = rerun_root / condition / "run_manifest.csv"
    rows = [
        row
        for row in read_csv(path)
        if row.get("controller") == controller
        and math.isclose(float(row["v"]), 3.0)
        and math.isclose(float(row["w"]), 3.0)
    ]
    result: Dict[int, Dict[str, str]] = {}
    for row in rows:
        seed = as_int(row["seed"])
        if seed in result:
            raise RuntimeError(
                f"Duplicate run-manifest row for {condition}/{controller}/seed {seed}"
            )
        result[seed] = row
    return result


def collision_type(rosrun_log: Path, collision: int) -> str:
    text = rosrun_log.read_text(encoding="utf-8", errors="replace")
    matches = re.findall(r"Collision detected.*?threshold=([0-9.]+)", text)
    if collision == 0:
        if matches:
            raise RuntimeError(f"Collision log/metric mismatch in {rosrun_log}")
        return "none"
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected exactly one collision record in {rosrun_log}, found {len(matches)}"
        )
    threshold = float(matches[0])
    if math.isclose(threshold, 0.55):
        return "static_obstacle"
    if math.isclose(threshold, 0.30):
        return "uav_uav"
    raise RuntimeError(f"Unexpected collision threshold {threshold} in {rosrun_log}")


def analyze(args: argparse.Namespace) -> int:
    old_map_root = Path(args.old_map_root).resolve()
    old_online_root = Path(args.old_online_root).resolve()
    rerun_root = Path(args.rerun_root).resolve()
    source = extract_source_seeds(old_map_root, old_online_root)

    detail_rows: List[Dict[str, object]] = []
    summary_rows: List[Dict[str, object]] = []
    new_events: Dict[Tuple[str, str], Dict[int, int]] = {}
    for condition in CONDITIONS:
        for controller in CONTROLLERS:
            expected_seeds = source[(condition, controller)]
            rerun = rerun_collision_rows(rerun_root, condition, controller)
            manifests = rerun_manifest_rows(rerun_root, condition, controller)
            if set(rerun) != set(expected_seeds):
                missing = sorted(set(expected_seeds) - set(rerun))
                extra = sorted(set(rerun) - set(expected_seeds))
                raise RuntimeError(
                    f"Rerun coverage mismatch for {condition}/{controller}: "
                    f"missing={missing}, extra={extra}"
                )
            if set(manifests) != set(expected_seeds):
                raise RuntimeError(
                    f"Run-manifest coverage mismatch for {condition}/{controller}"
                )
            contacts = {seed: as_int(rerun[seed]["collision"]) for seed in expected_seeds}
            contact_types = {
                seed: collision_type(Path(manifests[seed]["rosrun_log"]), contacts[seed])
                for seed in expected_seeds
            }
            new_events[(condition, controller)] = contacts
            count = sum(contacts.values())
            static_count = sum(
                value == "static_obstacle" for value in contact_types.values()
            )
            uav_uav_count = sum(value == "uav_uav" for value in contact_types.values())
            if count != static_count + uav_uav_count:
                raise RuntimeError(f"Collision-type count mismatch for {condition}/{controller}")
            summary_rows.append(
                {
                    "condition": condition,
                    "controller": controller,
                    "old_mfail0p1_event_count": len(expected_seeds),
                    "new_mfail0_contact_count": count,
                    "new_static_obstacle_contact_count": static_count,
                    "new_uav_uav_contact_count": uav_uav_count,
                    "new_contact_rate_denominator": 100,
                    "new_contact_rate": count / 100.0,
                    "old_events_resolved_without_contact": len(expected_seeds) - count,
                }
            )
            for seed in expected_seeds:
                detail_rows.append(
                    {
                        "condition": condition,
                        "controller": controller,
                        "seed": seed,
                        "old_mfail0p1_event": 1,
                        "new_mfail0_contact": contacts[seed],
                        "new_contact_type": contact_types[seed],
                        "metrics_csv": rerun[seed]["metrics_csv"],
                        "rosrun_log": manifests[seed]["rosrun_log"],
                    }
                )

    paired_rows: List[Dict[str, object]] = []
    for condition in CONDITIONS:
        frozen = new_events[(condition, "noninertial_frozen")]
        stage = new_events[(condition, "noninertial_stage")]
        labels = {"both": 0, "frozen_only": 0, "stage_only": 0, "neither": 0}
        for seed in range(1, 101):
            pair = (frozen.get(seed, 0), stage.get(seed, 0))
            label = {
                (1, 1): "both",
                (1, 0): "frozen_only",
                (0, 1): "stage_only",
                (0, 0): "neither",
            }[pair]
            labels[label] += 1
        paired_rows.append({"condition": condition, **labels, "total_seeds": 100})

    tables = rerun_root / "tables"
    write_csv(
        tables / "contact_rate_summary.csv",
        summary_rows,
        (
            "condition",
            "controller",
            "old_mfail0p1_event_count",
            "new_mfail0_contact_count",
            "new_static_obstacle_contact_count",
            "new_uav_uav_contact_count",
            "new_contact_rate_denominator",
            "new_contact_rate",
            "old_events_resolved_without_contact",
        ),
    )
    write_csv(
        tables / "contact_per_rerun_case.csv",
        detail_rows,
        (
            "condition",
            "controller",
            "seed",
            "old_mfail0p1_event",
            "new_mfail0_contact",
            "new_contact_type",
            "metrics_csv",
            "rosrun_log",
        ),
    )
    write_csv(
        tables / "contact_paired_contingency.csv",
        paired_rows,
        ("condition", "both", "frozen_only", "stage_only", "neither", "total_seeds"),
    )

    lines = [
        "# K_P=6 physical-contact rerun (m_fail = 0)",
        "",
        "Rates use the original 100 seeds per condition and controller. Only old ",
        "m_fail=0.1 event-positive cases were rerun; all old event-negative cases ",
        "are necessarily contact-negative under the smaller threshold.",
        "",
        "| Condition | Controller | Old event count | Any contact | Static contact | UAV-UAV contact | Any-contact rate | Resolved |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary_rows:
        lines.append(
            f"| {row['condition']} | {row['controller']} | "
            f"{row['old_mfail0p1_event_count']} | {row['new_mfail0_contact_count']} | "
            f"{row['new_static_obstacle_contact_count']} | "
            f"{row['new_uav_uav_contact_count']} | "
            f"{100.0 * float(row['new_contact_rate']):.1f}% | "
            f"{row['old_events_resolved_without_contact']} |"
        )
    report = rerun_root / "contact_report.md"
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(report)
    for row in summary_rows:
        print(
            f"{row['condition']}/{row['controller']}: "
            f"{row['new_mfail0_contact_count']}/100 "
            f"({100.0 * float(row['new_contact_rate']):.1f}%)"
        )
    return 0


def main() -> int:
    args = parser().parse_args()
    if args.command == "prepare":
        return prepare(args)
    if args.command == "seeds":
        values = manifest_seeds(
            Path(args.rerun_root).resolve(), args.condition, args.controller
        )
        print(",".join(map(str, values)))
        return 0
    if args.command == "analyze":
        return analyze(args)
    raise AssertionError(args.command)


if __name__ == "__main__":
    raise SystemExit(main())
