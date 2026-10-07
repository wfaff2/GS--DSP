# Field-HOCBF diagnostic replay

This optional recorder preserves the original controller, pruning and parameter
values. It is disabled by default (`field_replay_diagnostics:=false`). It records
the raw DSP snapshots actually consumed, then references them from each MPC
cycle. Snapshot/source IDs identify points within a snapshot; they are not
cross-frame physical-object IDs. Exact world positions are retained.

Each stage records the selected prediction slice, extrapolation time, car
transform, non-inertial inputs, nominal state, model/configuration, selected
point order and computed constraint. Each cycle also stores estimated and
synchronized simulation-odometry states. The versioned binary format uses
explicit fixed-size primitive fields, native byte order and IEEE double values;
it does not serialize Eigen or structure padding.

## Build and tests

```bash
cd marsim_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg coni_mpc -j2
cmake --build build --target field_hocbf_test field_replay_diagnostics_test -- -j2
devel/lib/coni_mpc/field_hocbf_test
devel/lib/coni_mpc/field_replay_diagnostics_test
devel/lib/coni_mpc/field_replay_tool --self-test
```

## One archived simulation

From the repository root, after sourcing the workspace:

```bash
source marsim_ws/devel/setup.bash
python3 marsim_ws/src/coni_mpc/scripts/run_field_replay_experiment.py \
  --output /tmp/dsp_field_replay_new --port 11411 --duration 30
```

Use a new output directory. The runner uses a dedicated ROS master, leaves
existing masters alone, disables RViz, runs the 50 Hz Field-HOCBF scenario and
captures sensor clouds/poses, dynamic-obstacle truth and UAV odometry. It archives
source changes, configuration, runtime parameters and the executable fingerprint.
Recording time is measured separately. It can affect wall-time scheduling;
check the timing CSV and scheduler summary before comparing runs.

To enable recording in a manually launched experiment, add
`field_replay_diagnostics:=true` and specify `metrics_csv`. The binary path is
derived as `<metrics_field_hocbf_steps.csv>.replay.bin`. The private
`field_replay_log` parameter can override that path.

## Replay and event comparisons

```bash
devel_bin=marsim_ws/devel/lib/coni_mpc
run_dir=/tmp/dsp_field_replay_new
"$devel_bin/field_replay_tool" \
  "$run_dir/metrics_field_hocbf_steps.csv.replay.bin" "$run_dir/replay.csv"
"$devel_bin/field_replay_event_tool" \
  "$run_dir/metrics_field_hocbf_steps.csv.replay.bin" \
  "$run_dir/event_counterfactuals.csv" 330 1313 1314
python3 marsim_ws/src/coni_mpc/scripts/analyze_field_replay_run.py "$run_dir"
```

The event indices above are examples from the recorded 2026-10-05 run, not
universal event times. Choose indices from the new run's command changes.
The analyzer requires the existing Python `numpy`, `pandas`, `scipy`, `yaml`
and ROS `rosbag` modules.

First require exact reconstruction of selected IDs and constraint values.
The general replay then compares the original nominal query with current
estimated/true queries and compares old/new snapshots at a common evaluation
time. The event tool separately holds the old prediction slice, or holds the
old selection when its identity is unambiguous within the same snapshot/slice.
It verifies its original-event reconstruction before emitting comparisons.

Interpretation limits:

- These tools recompute constraints; they do not replay QP warm starts or
  solve counterfactual control problems.
- Future UAV truth is not available. True-state comparisons apply at stage 0.
- `current_truth` replaces UAV and car/frame odometry. `uav_truth_fixed_car`
  replaces UAV state while retaining the recorded car/frame inputs. Both retain
  the recorded `beta` and `a_car`, which odometry does not directly supply as
  synchronized truth.
- Field distances use selected voxel centers on the nominal query trajectory.
  LiDAR and full-PCD distances use sampled points. Published dynamic boxes are
  interpreted as box models. None alone proves continuous geometric safety.
- True velocity finite differences report cycle-average acceleration, not
  the instantaneous peak acceleration.
