#!/usr/bin/env python3
"""Resolve scene YAML to scalar ROS launch parameters (CLI overrides win)."""

import argparse
import json
import math
import sys
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--scene', default='')
    parser.add_argument('--v', default='')
    parser.add_argument('--w', default='')
    parser.add_argument('--trajectory-laps', default='')
    parser.add_argument('--static-map', default='')
    parser.add_argument('--key', required=True)
    args = parser.parse_args()
    try:
        config = yaml.safe_load(args.config.read_text())
        scene = args.scene or config['scene']
        maps = {
            'figure_eight': 'dsp_figure_eight/six_cylinders.pcd',
            'circle': 'depth_cbf_m25/A2_three_square_prisms_l_wall_near_uav.pcd',
        }
        if scene not in maps:
            raise ValueError('scene must be figure_eight or circle')
        preset = config['scenes'][scene]
        values = {'scene': scene}
        for key, override, default in (
                ('v', args.v, preset['v']), ('w', args.w, preset['w']),
                ('trajectory_laps', args.trajectory_laps, config['trajectory_laps'])):
            value = float(override if override != '' else default)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
            if key == 'trajectory_laps' and value < 1:
                raise ValueError('trajectory_laps must be >= 1')
            values[key] = value
        package = Path(__file__).resolve().parents[1]
        map_name = package / 'test_data' / (args.static_map or maps[scene])
        if not map_name.is_file():
            raise ValueError(f'map does not exist: {map_name}')
        circle = scene == 'circle'
        values.update(map_name=str(map_name), dynobj_enable=int(circle),
                      dynobject_num=2 if circle else 0,
                      deterministic_box_mode=circle,
                      ugv_circle_radius=7.0 if circle else 0.0,
                      uav_reference_offsets=[[0.0, 1.5, 2.0]] if circle else [])
        value = values[args.key]
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        parser.error(f'{args.config}: {error}')
    # JSON lists are valid YAML; scalar output also supports ROS type conversion.
    sys.stdout.write(json.dumps(value) if isinstance(value, (list, bool)) else str(value))


if __name__ == '__main__':
    main()
