#!/usr/bin/env python3
import argparse
import json
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("config")
    parser.add_argument("scenario")
    parser.add_argument("--minimum-frames", type=int, default=20)
    args = parser.parse_args()
    with open(args.config) as handle:
        stats = json.load(handle)["runs"][args.scenario]["collection"]
    complete = (stats["frames"] >= args.minimum_frames and
                stats["gt_missing_frames"] == 0 and
                stats["gt_covered_frames"] == stats["frames"])
    if not complete:
        print("GT coverage preflight failed: %s" % stats, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
