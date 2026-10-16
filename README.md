# GS--DSP
cd ~/桌面/DSP/marsim_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch coni_mpc dsp_mpc_sim.launch




This repository contains the reproducible DSP occupancy prediction, MARSIM
point-cloud renderer, and CoNi-MPC Field-HOCBF simulation path used by
`coni_mpc/launch/dsp_mpc_sim.launch`.

The published source tree keeps the runtime chain:

- `DSP-map`: the DSP occupancy-map node and messages;
- `marsim_ws/src/MARSIM/map_generator`: static map publisher;
- `marsim_ws/src/MARSIM/local_sensing`: CPU point-cloud renderer;
- `marsim_ws/src/MARSIM/Utils/mars_quadrotor_msgs`: renderer message types;
- `marsim_ws/src/coni_mpc`: MPC, Field-HOCBF, launch files, parameters, tests,
  generated ACADO solver sources, and scenario PCD files;
- `marsim_ws/src/no_catkin_compilation/acado-stable`: the vendored qpOASES
  sources required by the generated solver.

Build on Ubuntu with ROS Noetic:

```bash
source /opt/ros/noetic/setup.bash
cd marsim_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make -j2
source devel/setup.bash
```

Run the current one-UAV figure-eight experiment with six static cylinders
(the current LiDAR → DSP → Field-HOCBF control path):

```bash
mkdir -p /tmp/gs_dsp_run
roslaunch coni_mpc dsp_mpc_sim.launch \
  cbf_use_field_hocbf:=true \
  sim_control_dt_sec:=0.02 \
  metrics_csv:=/tmp/gs_dsp_run/metrics.csv \
  run_tag:=figure_eight_50hz
```

For persistent settings, edit `parameters/dsp_mpc_scene.yaml` and restart
with the same `roslaunch coni_mpc dsp_mpc_sim.launch` command:

```yaml
scene: figure_eight  # figure_eight or circle
trajectory_laps: 2.0
scenes:
  figure_eight:
    v: 1.0          # UGV speed limit [m/s]
    w: 1.0          # UGV yaw-rate limit [rad/s]
  circle:
    v: 1.0
    w: 0.135
```

Optional launch arguments temporarily override the YAML:

```bash
# Figure-eight, six static cylinders (default); v in m/s, w in rad/s.
roslaunch coni_mpc dsp_mpc_sim.launch scene:=figure_eight v:=1.0 w:=1.0
# Previous circle, three square prisms, L-wall and two moving boxes.
roslaunch coni_mpc dsp_mpc_sim.launch scene:=circle
```

The figure-eight defaults give about 20 s cruise per lap. Figure-eight speed
is constrained by both limits, so increasing only `v` may not increase speed.
`scene_config:=/path/to/scene.yaml` selects another configuration file.

The figure-eight geometry and cylinder dimensions come from
`parameters/num_sim_non_one_point.yaml`. After changing those geometry
parameters, regenerate the LiDAR map with
`python3 marsim_ws/src/coni_mpc/scripts/generate_dsp_figure_eight_map.py`
from the repository root. `parameters/dsp_mpc_single_obstacles.yaml` controls
only the moving boxes in `scene:=circle`. Leave `sim_duration_sec` unset to retain the nominal trajectory
speed; setting it compresses the configured two laps into that duration.

Build products (`build/`, `devel/`), ROS logs, metrics/results, old duplicate
workspaces, and unrelated detector/algorithm sources are intentionally kept
out of version control. The simulation generates those files locally.
