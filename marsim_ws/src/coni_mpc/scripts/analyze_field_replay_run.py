#!/usr/bin/env python3
"""Join deterministic field replay with time-aligned simulation truth from bag."""
import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
import rosbag
from scipy.spatial import cKDTree
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir')
    args = parser.parse_args()
    root = Path(args.run_dir).resolve()
    steps = pd.read_csv(root / 'metrics_steps.csv', float_precision='round_trip')
    field = pd.read_csv(root / 'metrics_field_hocbf_steps.csv', float_precision='round_trip')
    replay = pd.read_csv(root / 'replay.csv', float_precision='round_trip')
    stage0 = replay[replay.stage.eq(0)].set_index('step_idx')
    timing = pd.read_csv(root / 'metrics_field_hocbf_steps.csv.replay.bin.timing.csv')
    params = yaml.safe_load((root / 'runtime_params.yaml').read_text())
    radius = params['num_sim_non_one_point_node']['uav_radius']
    odom, dynamic = [], []
    with rosbag.Bag(str(root / 'sensors.bag')) as bag:
        for topic, message, _ in bag.read_messages(topics=[
                '/coni_mpc/quad_odom0', '/quad0_pcl_render_node/dynamic_obstacle_states']):
            stamp = message.header.stamp.to_sec()
            if topic.endswith('quad_odom0'):
                p, v = message.pose.pose.position, message.twist.twist.linear
                odom.append([stamp, p.x, p.y, p.z, v.x, v.y, v.z])
            else:
                for obstacle in message.obstacles:
                    p, v, size = obstacle.position, obstacle.velocity, obstacle.size
                    dynamic.append([stamp, obstacle.obstacle_id, p.x, p.y, p.z,
                                    v.x, v.y, v.z, size.x, size.y, size.z, obstacle.geometry_type])
    truth = pd.DataFrame(odom, columns=['stamp', 'px', 'py', 'pz', 'vx', 'vy', 'vz'])
    assert len(truth) == len(steps), 'bag must have one truth odometry per MPC cycle'
    assert steps.step_idx.is_unique
    truth['step_idx'] = steps.step_idx
    truth['sim_time'] = steps.sim_time
    # ROS time toSec and CSV parsing may differ by one double ULP at epoch scale.
    stamp_error = np.max(np.abs(truth.loc[stage0.index, 'stamp'].to_numpy() - stage0.quad_stamp.to_numpy()))
    assert stamp_error < 1e-6
    values = ['px', 'py', 'pz', 'vx', 'vy', 'vz']
    logged_values = ['truth_quad_' + c for c in values]
    state_error = np.max(np.abs(truth.loc[stage0.index, values].to_numpy() - stage0[logged_values].to_numpy()))
    assert state_error < 1e-12
    velocity = truth[['vx', 'vy', 'vz']].to_numpy()
    truth['horizontal_speed_mps'] = np.linalg.norm(velocity[:, :2], axis=1)
    truth['xy_cycle_average_accel_mps2'] = np.r_[np.nan, np.linalg.norm(
        np.diff(velocity[:, :2], axis=0) / np.diff(truth.sim_time.to_numpy())[:, None], axis=1)]
    commands = steps[['applied_cmd_vx_world', 'applied_cmd_vy_world', 'applied_cmd_vz_world']].to_numpy()
    truth['command_delta_mps'] = np.r_[np.nan, np.linalg.norm(np.diff(commands, axis=0), axis=1)]
    with (root / 'static_scene.pcd').open() as source:
        for line in source:
            if line.startswith('DATA ascii'):
                break
        points = np.loadtxt(source)[:, :3]
    distance, _ = cKDTree(points).query(truth[['px', 'py', 'pz']].to_numpy())
    truth['static_sampled_center_distance_m'] = distance
    truth['static_sampled_clearance_m'] = distance - radius
    dyn = pd.DataFrame(dynamic, columns=['stamp', 'id', 'x', 'y', 'z', 'vx', 'vy', 'vz',
                                        'sx', 'sy', 'sz', 'geometry_type'])
    dyn.to_csv(root / 'dynamic_truth.csv', index=False)
    all_distances = []
    xyz = truth[['px', 'py', 'pz']].to_numpy()
    for obstacle_id, group in dyn.groupby('id'):
        group = group.sort_values('stamp').reset_index(drop=True)
        assert group.geometry_type.eq('box').all(), 'this analysis models published boxes only'
        stamps = group.stamp.to_numpy()
        right = np.clip(np.searchsorted(stamps, truth.stamp), 0, len(stamps)-1)
        left = np.maximum(right-1, 0)
        nearest = np.where(abs(stamps[right]-truth.stamp) < abs(stamps[left]-truth.stamp), right, left)
        selected = group.iloc[nearest]
        dt = truth.stamp.to_numpy() - selected.stamp.to_numpy()
        # Constant velocity interpolation aligns the published obstacle state
        # to each odometry stamp, not to nominal simulation time.
        centers = selected[['x', 'y', 'z']].to_numpy() + selected[['vx', 'vy', 'vz']].to_numpy()*dt[:, None]
        delta = np.abs(xyz-centers) - selected[['sx', 'sy', 'sz']].to_numpy()/2
        sdf = np.linalg.norm(np.maximum(delta, 0), axis=1) + np.minimum(np.max(delta, axis=1), 0)
        truth[f'dynamic_box_{obstacle_id}_modeled_clearance_m'] = sdf-radius
        truth[f'dynamic_box_{obstacle_id}_time_offset_sec'] = dt
        all_distances.append(sdf-radius)
    if all_distances:
        truth['dynamic_box_modeled_clearance_m'] = np.min(all_distances, axis=0)
    truth.to_csv(root / 'truth_states.csv', index=False)
    current = field[field.horizon_idx.eq(0)]
    max_event = int(np.nanargmax(truth.command_delta_mps))
    summary = {
        'mpc_cycles': len(steps), 'field_rows': len(field),
        'solve_successes': int(steps.solve_ok.eq(1).sum()),
        'solver_failure_flags': int(steps.solver_fail_flag.sum()),
        'collision_monitor_flag': int(pd.read_csv(root/'metrics.csv').iloc[0].collision),
        'replay_cycles': int(replay.step_idx.nunique()),
        'replay_available_stages': int(replay.stage.ge(0).sum()),
        'replay_unavailable_cycles': int(replay.stage.eq(-1).sum()),
        'replay_max_error': float(replay.max_replay_error.max()),
        'replay_profile_b_max_error': float(replay.profile_b_error.max()),
        'selected_ids_all_match': bool(replay.ids_match.eq(1).all()),
        'truth_alignment_max_stamp_error_sec': float(stamp_error),
        'truth_alignment_max_state_error': float(state_error),
        'max_command_jump_mps': float(truth.command_delta_mps.max()),
        'max_command_jump_time_sec': float(truth.iloc[max_event].sim_time),
        'max_actual_horizontal_speed_mps': float(truth.horizontal_speed_mps.max()),
        'max_actual_cycle_average_xy_accel_mps2': float(truth.xy_cycle_average_accel_mps2.max()),
        'tracking_rms_m': float(np.sqrt(np.mean(steps.tracking_error**2))),
        'stage0_negative_nominal_h_cycles': int(current.field_h.lt(0).sum()),
        'stage0_min_nominal_h_m': float(current.field_h.min()),
        'stage0_max_slack': float(current.field_slack.max()),
        'all_stages_max_slack': float(field.field_slack.max()),
        'diagnostic_write_mean_ms': float(timing.write_ms.mean()),
        'diagnostic_write_p95_ms': float(timing.write_ms.quantile(.95)),
        'diagnostic_write_max_ms': float(timing.write_ms.max()),
        'diagnostic_writes_all_ok': bool(timing.write_ok.eq(1).all()),
        'static_full_pcd_min_sampled_clearance_m': float(truth.static_sampled_clearance_m.min()),
    }
    (root/'analysis_summary.json').write_text(json.dumps(summary, indent=2))
    selected_steps = sorted(set(range(max_event-3, max_event+4)) | set(range(328, 335)))
    event = steps.set_index('step_idx').loc[selected_steps].copy()
    event = event.join(truth.set_index('step_idx').drop(columns='sim_time'), rsuffix='_truth')
    event = event.join(stage0.drop(columns='sim_time'), rsuffix='_replay')
    event.to_csv(root/'critical_events.csv')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
