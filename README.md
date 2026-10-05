# GS--DSP

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

Run the current one-UAV square-prism experiment:

```bash
mkdir -p /tmp/gs_dsp_run
roslaunch coni_mpc dsp_mpc_sim.launch \
  cbf_use_field_hocbf:=true \
  sim_control_dt_sec:=0.02 \
  sim_duration_sec:=30 \
  metrics_csv:=/tmp/gs_dsp_run/metrics.csv \
  run_tag:=square_prism_50hz
```

Build products (`build/`, `devel/`), ROS logs, metrics/results, old duplicate
workspaces, and unrelated detector/algorithm sources are intentionally kept
out of version control. The simulation generates those files locally.
