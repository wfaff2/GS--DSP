# MARSIM Mid-360 to LV-DOT

This is an independent perception path. It does not start or modify the
MPC/CBF controller.

## Runtime graph

`MARSIM Mid-360 sensor_cloud -> upstream LV-DOT detector_node`

Inputs:

- Point cloud: `/quad0_pcl_render_node/sensor_cloud`
- UAV odometry: `/coni_mpc/quad_odom1`

Outputs retained for the later controller-integration stage:

- LiDAR boxes: `/onboard_detector/lidar_bboxes`
- Tracked boxes: `/onboard_detector/tracked_bboxes`
- Dynamic boxes: `/onboard_detector/dynamic_bboxes`
- Position, velocity and 3D size service:
  `/onboard_detector/get_dynamic_obstacles`

The service is the upstream LV-DOT `GetDynamicObstacles` contract. Its response
contains equal-length `geometry_msgs/Vector3[]` arrays named `position`,
`velocity`, and `size`.

## Run

Static detection:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch coni_mpc marsim_lv_dot_uav1.launch
```

Dynamic detection and tracking:

```bash
roslaunch coni_mpc marsim_lv_dot_uav1.launch \
  dynamic_obstacles:=true dynamic_obstacle_count:=20 \
  map_name:='$(find coni_mpc)/test_data/lv_dot_validation_bounds.pcd' \
  result_file:='results/lv_dot_mid360_validation/dynamic_summary.json'
```

## Depth-CBF preservation

All Depth-CBF source, parameters, launch files, and historical results remain
available. The old `marsim_depth_cbf_uav1.launch` now defaults
`start_depth_cbf:=false`. For a legacy M1/M2 reproduction only, explicitly pass
`start_depth_cbf:=true`.

No controller consumes LV-DOT output in this stage.
