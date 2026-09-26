#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

STAGE_SOURCE_BASENAMES = [
    "noninertial.log",
    "noninertial_metrics.csv",
    "noninertial_metrics_steps.csv",
    "noninertial_metrics_ugv_horizon_steps.csv",
    "noninertial_metrics_ugv_horizon_summary.csv",
    "noninertial_roscore.log",
]

FROZEN_SOURCE_BASENAMES = [
    "noninertial.log",
    "noninertial_metrics.csv",
    "noninertial_metrics_steps.csv",
    "noninertial_metrics_ugv_horizon_steps.csv",
    "noninertial_metrics_ugv_horizon_summary.csv",
    "noninertial_roscore.log",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Normalize a stage-primary run root into the legacy three-case naming scheme "
            "by copying stage files to noninertial_stage_* and frozen backfill files to "
            "noninertial_*."
        )
    )
    parser.add_argument("--stage-root", required=True, type=Path)
    parser.add_argument("--frozen-root", required=True, type=Path)
    return parser.parse_args()


def stage_dest_name(name: str) -> str:
    return name.replace("noninertial", "noninertial_stage", 1)


def validate_tree(stage_root: Path, frozen_root: Path) -> list[tuple[Path, Path]]:
    pairs: list[tuple[Path, Path]] = []
    missing: list[str] = []
    for stage_seed_dir in sorted(stage_root.rglob("seed_*")):
        if not stage_seed_dir.is_dir():
            continue
        rel = stage_seed_dir.relative_to(stage_root)
        frozen_seed_dir = frozen_root / rel
        if not frozen_seed_dir.is_dir():
            missing.append(f"missing frozen seed dir: {frozen_seed_dir}")
            continue
        for name in STAGE_SOURCE_BASENAMES:
            if not (stage_seed_dir / name).is_file():
                missing.append(f"missing stage file: {stage_seed_dir / name}")
        for name in FROZEN_SOURCE_BASENAMES:
            if not (frozen_seed_dir / name).is_file():
                missing.append(f"missing frozen file: {frozen_seed_dir / name}")
        pairs.append((stage_seed_dir, frozen_seed_dir))

    if missing:
        sample = "\n".join(f"  - {line}" for line in missing[:30])
        if len(missing) > 30:
            sample += f"\n  - ... {len(missing) - 30} more"
        raise SystemExit(f"Merge validation failed:\n{sample}")
    return pairs


def main() -> None:
    args = parse_args()
    stage_root = args.stage_root.resolve()
    frozen_root = args.frozen_root.resolve()
    pairs = validate_tree(stage_root, frozen_root)

    copied = 0
    for stage_seed_dir, frozen_seed_dir in pairs:
        for name in STAGE_SOURCE_BASENAMES:
            shutil.copy2(stage_seed_dir / name, stage_seed_dir / stage_dest_name(name))
            copied += 1
        for name in FROZEN_SOURCE_BASENAMES:
            shutil.copy2(frozen_seed_dir / name, stage_seed_dir / name)
            copied += 1

    print(f"[INFO] merged {len(pairs)} seed directories")
    print(f"[INFO] copied {copied} files")
    print(f"[INFO] stage root normalized at: {stage_root}")


if __name__ == "__main__":
    main()
