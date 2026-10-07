#!/usr/bin/env python3
"""Archive one simulation with field replay data on a dedicated ROS master."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--port', type=int, default=11411)
    parser.add_argument('--duration', type=float, default=30.)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[4]
    package = Path(__file__).resolve().parents[1]
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', args.port)) == 0:
            raise RuntimeError('Dedicated ROS master port is already in use')
    env = dict(os.environ, ROS_MASTER_URI=f'http://127.0.0.1:{args.port}')
    env.pop('ROS_HOSTNAME', None)
    env['ROS_IP'] = '127.0.0.1'
    command = ['roslaunch', '-p', str(args.port), 'coni_mpc', 'dsp_mpc_sim.launch',
               'cbf_use_field_hocbf:=true',
               'field_replay_diagnostics:=true', 'sim_control_dt_sec:=0.02',
               f'sim_duration_sec:={args.duration}', f'metrics_csv:={output / "metrics.csv"}',
               'run_tag:=field_replay_50hz', 'start_rviz:=false']
    metadata = {'schema': 1, 'started_at': datetime.datetime.now().astimezone().isoformat(),
                'command': command, 'ros_master_uri': env['ROS_MASTER_URI'],
                'repo': str(repo), 'duration_sec': args.duration, 'control_dt_sec': .02,
                'rviz_enabled': False, 'status': 'starting'}
    metadata['commit'] = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
    (output / 'source.diff').write_text(subprocess.check_output(['git', '-C', str(repo), 'diff'], text=True))
    (output / 'git_status.txt').write_text(subprocess.check_output(['git', '-C', str(repo), 'status', '--short'], text=True))
    archive = output / 'source'
    for name in ['CMakeLists.txt', 'src/num_sim/num_sim_non_one_point_node.cpp', 'src/coni_mpc/num_sim_mpc.cpp',
                 'include/coni_mpc/num_sim_mpc.h', 'include/coni_mpc/field_hocbf.h',
                 'include/coni_mpc/field_replay_diagnostics.h', 'test/field_replay_tool.cpp',
                 'test/field_replay_diagnostics_test.cpp',
                 'parameters/num_sim_non_one_point.yaml', 'parameters/dsp_mpc_single_obstacles.yaml',
                 'launch/dsp_mpc_sim.launch', 'launch/num_sim_non_one_point.launch',
                 'launch/marsim_depth_cbf_uav1.launch', 'scripts/run_field_replay_experiment.py']:
        dest = archive / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(package / name, dest)
    shutil.copy2(package / 'test_data/depth_cbf_m25/A2_three_square_prisms_l_wall.pcd',
                 output / 'static_scene.pcd')
    binary = repo / 'marsim_ws/devel/lib/coni_mpc/num_sim_non_one_point_node'
    metadata['binary'] = str(binary)
    metadata['binary_mtime_ns'] = binary.stat().st_mtime_ns
    metadata['binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
    metadata_path = output / 'experiment.json'
    metadata_path.write_text(json.dumps(metadata, indent=2))
    launch = bag = None
    try:
        with (output / 'roslaunch.log').open('w') as log, (output / 'rosbag_record.log').open('w') as baglog:
            launch = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 20
            while True:
                if launch.poll() is not None:
                    raise RuntimeError('roslaunch exited before the master started')
                with socket.socket() as probe:
                    ready = probe.connect_ex(('127.0.0.1', args.port)) == 0
                if ready:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError('ROS master startup timed out')
                time.sleep(.2)
            topics = ['/quad0_pcl_render_node/sensor_cloud', '/quad0_pcl_render_node/sensor_pose',
                      '/quad0_pcl_render_node/cloud', '/quad0_pcl_render_node/dynamic_obstacle_states',
                      '/coni_mpc/quad_odom0']
            metadata['bag_topics'] = topics
            bag = subprocess.Popen(['rosbag', 'record', '--lz4', '-O', str(output / 'sensors.bag')] + topics,
                                   env=env, stdout=baglog, stderr=subprocess.STDOUT, start_new_session=True)
            # Allow parameters to load while keeping sensor capture active.
            time.sleep(2)
            subprocess.run(['rosparam', 'dump', str(output / 'runtime_params.yaml')],
                           env=env, stdout=log, stderr=log, check=True)
            metadata['status'] = 'running'
            metadata_path.write_text(json.dumps(metadata, indent=2))
            print(f'RUNNING {output}', flush=True)
            deadline = time.monotonic() + args.duration * 3 + 30
            while not ((output / 'metrics.csv').exists() and (output / 'metrics.csv').stat().st_size > 0):
                if launch.poll() is not None:
                    raise RuntimeError('roslaunch exited without summary metrics')
                if bag.poll() is not None:
                    raise RuntimeError('Sensor bag recorder exited unexpectedly')
                if time.monotonic() > deadline:
                    raise RuntimeError('Simulation did not write summary metrics before timeout')
                time.sleep(.5)
            # The controller closes diagnostic streams during its normal exit.
            time.sleep(1)
            metadata['status'] = 'completed'
    except Exception as error:
        metadata['status'] = 'failed'
        metadata['error'] = str(error)
        raise
    finally:
        stop(bag)
        stop(launch)
        metadata['finished_at'] = datetime.datetime.now().astimezone().isoformat()
        metadata_path.write_text(json.dumps(metadata, indent=2))
    print(f'COMPLETED {output}', flush=True)


if __name__ == '__main__':
    main()
