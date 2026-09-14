# UAV1 CPU-Livox Depth-CBF M1/M2 report

> Historical validation record: the measurements below were collected with
> the former Livox Avia configuration. The active launch/YAML configuration
> was changed to MARSIM Livox Mid-360 afterward; these historical numbers must
> not be presented as Mid-360 validation results.

Scope: independent M1 point-cloud preprocessing and M2 local quadratic
regression only. The new path is not connected to ACADO, the existing HOCBF,
or `ACADO_NOD=38`.

## A. MARSIM

- Formal algorithm input: `/quad0_pcl_render_node/sensor_cloud`
  (`sensor_msgs/PointCloud2`), observed `frame_id=/sensor`.
- Debug-only ground truth: `/quad0_pcl_render_node/cloud`, observed
  `frame_id=world`. It is used only for same-stamp/same-index `q^W` checking.
  It is never passed to the preprocessor or regressor.
- Renderer odometry: `~odometry` is remapped to
  `/coni_mpc/quad_odom1`. The UGV pose comes from `/coni_mpc/car_odom`.
- CPU renderer: `local_sensing_node/pcl_render_node`, Avia pattern enabled,
  10 Hz, range 1--30 m, yaw FOV 70.4 degrees, vertical FOV 77.2 degrees.
- Static PCD: `map_generator/resource/small_forest01cutoff.pcd` (276,219
  loaded points in the integration run). Dynamic-object generation is disabled.
- LiDAR extrinsic source: the audited CPU renderer's sensor-cloud path uses
  body-to-LiDAR identity rotation and zero translation. The same explicit
  `R_BL=I`, `t_BL=0` are stored in YAML rather than hidden in the callback.
- Launch/config: `launch/marsim_depth_cbf_uav1.launch` and
  `parameters/depth_cbf.yaml`. The launch starts only the static map publisher,
  CPU renderer, M1 node, and M2 node; it does not start MARSIM dynamics/PID.

## B. Frames and transforms

- `F_L`: Livox sensor frame, actual PointCloud2 frame `/sensor`.
- `F_B`: UAV1 body frame, actual vehicle name/frame `uav_1`.
- `F_W`: world frame, configured as `map` (MARSIM debug cloud calls it
  `world`; coordinates were checked numerically).
- `F_N`: UGV-attached non-inertial frame, actual frame `base_link`.

The timestamp-aligned transform chain is

```text
q^B = R_BL q^L + t_BL
q^W = R_WB q^B + p_B^W
q^N = R_WN^T (q^W - p_N^W)
p_c^N = R_WN^T (p_B^W - p_N^W)
```

`PointCloud2`, UAV1 odometry, and UGV odometry are synchronized with a
three-message ApproximateTime policy and then explicitly rejected if either
pose differs from the cloud stamp by more than 0.025 s. Integration samples
showed `cloud_stamp == quad_odom_stamp`; UGV offsets were normally 2--15 us,
with an observed sample maximum of about 38.5 us. The node publishes all three
stamps and both signed offsets on `sync_diagnostics`.

## C. Files

New files:

- `include/coni_mpc/depth_cbf/point_cloud_preprocessor.h`
- `src/depth_cbf/point_cloud_preprocessor.cpp`
- `src/depth_cbf/marsim_cloud_preprocessor_node.cpp`
- `include/coni_mpc/depth_cbf/depth_cbf_regressor.h`
- `src/depth_cbf/depth_cbf_regressor.cpp`
- `src/depth_cbf/depth_cbf_regression_node.cpp`
- `msg/LocalBarrier.msg`
- `test/point_cloud_preprocessor_test.cpp`
- `test/depth_cbf_regressor_test.cpp`
- `parameters/depth_cbf.yaml`
- `launch/marsim_depth_cbf_uav1.launch`
- `rviz/depth_cbf_m1_m2.rviz`
- `DEPTH_CBF_M1_M2_REPORT.md`

Modified build metadata: `CMakeLists.txt`, `package.xml`.

No ACADO generated/model file, existing HOCBF implementation, OnlineData
layout, MPC objective, or controller source was modified.

## D. ROS topics

M1 inputs:

- `/quad0_pcl_render_node/sensor_cloud` -- formal LiDAR input.
- `/coni_mpc/quad_odom1` -- UAV1 pose at the cloud timestamp.
- `/coni_mpc/car_odom` -- UGV/N-frame pose at the cloud timestamp.
- `/quad0_pcl_render_node/cloud` -- optional debug-only world comparison.

M1 outputs:

- `/depth_cbf_cloud_preprocessor/cloud_world`
- `/depth_cbf_cloud_preprocessor/cloud_noninertial`
- `/depth_cbf_cloud_preprocessor/local_cloud_noninertial`
- `/depth_cbf_cloud_preprocessor/query_center_noninertial`
- `/depth_cbf_cloud_preprocessor/sync_diagnostics`

M2 subscribes only to the last two M1 data products. It publishes:

- `/depth_cbf_regression/barrier` (`coni_mpc/LocalBarrier`)
- `/depth_cbf_regression/markers` (`visualization_msgs/MarkerArray`)

Markers include local cloud (RViz PointCloud2 display), query mesh, each
query's nearest surface point, query-to-surface links, current query center,
the current nearest surface point, and the fitted gradient direction.

## E. YAML parameters

Topics/frames and synchronization: `cloud_topic`, `quad_odom_topic`,
`car_odom_topic`, `world_frame_id`, `noninertial_frame_id`, subscriber/sync
queue sizes, `sync_slop_sec=0.025`, and
`max_pose_time_offset_sec=0.025`.

Filters: `min_range=1.0 m`, `max_range=30.0 m`, `local_radius=4.0 m`,
voxel filter enabled with `voxel_leaf_size=0.10 m`, ground filter disabled,
and an explicit optional `ground_min_z_W`.

Extrinsics: row-major `rotation_BL` and `translation_BL`.

Query/regression: `L=2`, `epsilon=0.15 m`, nearest-neighbor fit support
`3.47 m`, relative SVD rank tolerance `1e-10`, maximum condition number
`1e8`, maximum RMSE `0.25`, maximum absolute error `0.75`, and logging/queue
parameters. The support distance is a regression validity parameter, not a
safety radius.

Safety parameters are deliberately separate:

- `r_uav=0.15 m`
- `delta_surface=0.0 m` -- **provisional placeholder; not a final design
  decision**
- `d_safe=0.15 m`, checked at startup as
  `d_safe = r_uav + delta_surface`; no obstacle radius is added.

The query mesh has 125 points and
`r_mesh=sqrt(3)*L*epsilon=0.519615 m`. The final consistency check is
`local_radius=4.0 >= r_mesh+3.47=3.989615 m`. Startup emits a warning because
the remaining crop reserve is less than 10%; an inconsistent value is a fatal
startup error.

## F. Depth-CBF regression

For every synthetic query point, a PCL `KdTreeFLANN` performs a separate
nearest-neighbor query. The raw value is

```text
h_raw(p) = min_q ||p-q||^2 - d_safe^2.
```

The fit uses local coordinates `xi=p^N-p_c^N` and the explicit design row

```text
[dx^2, dy^2, dz^2, 2dxdy, 2dxdz, 2dydz, dx, dy, dz, 1].
```

`Eigen::JacobiSVD` solves the least-squares system without forming an inverse.
Singular values, rank, condition number, RMSE, and maximum absolute residual
are retained. The off-diagonal coefficients map into a symmetric `A`:

```text
h_hat(p) = xi^T A xi + b^T xi + c
grad h_hat = 2 A (p-p_c^N) + b
Hessian h_hat = 2 A.
```

`LocalBarrier` stores `center_N`, `A`, `b`, `c`, residual metrics, condition
diagnostics, validity, and the source cloud stamp. If the cloud is empty, any
query lacks supported surface data, rank is not 10, conditioning is excessive,
or residual limits fail, a new `valid=false` barrier is published and no stale
barrier is retained.

## G. Verification

Build: `catkin_make --pkg coni_mpc -DCATKIN_ENABLE_TESTING=ON -j2` passed.
Because this workspace path contains spaces, the catkin `run_tests` wrapper
cannot locate executables reliably; both gtest binaries were built and run
directly.

M1 deterministic tests: 6/6 passed, covering UAV rotation/world invariance,
UAV translation/world invariance, UGV motion/apparent N-frame motion,
non-identity LiDAR extrinsics, crop-parameter rejection, and range/local crop.
Live MARSIM samples printed `q^L`, `q^B`, `q^W`, `q^N`; computed `q^W`
matched the same-index `/cloud` debug point within approximately
0--4.8e-7 m.

M2 deterministic tests: 5/5 passed and do not require MARSIM.

| Synthetic scene | RMSE | max abs error | h_raw(center) | h_fit(center) |
|---|---:|---:|---:|---:|
| Plane wall | 1.05668e-8 | 1.87329e-8 | 0.1375 | 0.1375 |
| Cylinder surface | 1.23848e-4 | 2.14130e-4 | 0.1000 | 0.100002 |
| Corner / nearest-point switch | 0.00622466 | 0.01298 | 0.0803 | 0.0675 |

Finite-difference tests verified the analytic gradient and Hessian, symmetry
of `A`, expected safe-direction gradient for plane/cylinder/corner, and a
smooth analytic gradient through the corner fit. Line samples were written to
`/tmp/depth_cbf_plane_line.csv`, `/tmp/depth_cbf_cylinder_line.csv`, and
`/tmp/depth_cbf_corner_line.csv`.

The 25-run synthetic-cylinder benchmark measured mean 2.134 ms and maximum
2.188 ms. Live MARSIM log samples were typically about 0.14--0.36 ms, with
running maxima below about 1.0 ms in the inspected run. A 50-message integration
sample contained 50/50 valid barriers. One captured barrier had rank 10,
condition number 26.7222, RMSE 0.0576879, and maximum absolute error 0.167616.

## H. Unresolved items and stop boundary

- `delta_surface` has intentionally not been selected. Therefore the current
  `d_safe=0.15 m` is only a parameter plumbing/test value, not a closed-loop
  safety choice.
- Livox `min_range=1.0 m` is greater than `d_safe=0.15 m` by 0.85 m. The
  renderer parameter was not changed. In a future closed loop, an obstacle can
  enter the sensor blind zone and disappear before the center-to-surface safety
  boundary is reached; no safety claim is possible without resolving this.
- Static-PCD integration can legitimately produce `valid=false` frames when
  the Avia FOV has no local points or the full query mesh lacks supported
  nearest points. The fallback is explicit, but future control integration
  must define how to handle this state.
- The final 3.47 m fit support leaves only about 0.0104 m crop reserve. It is
  mathematically consistent and warned at startup, but should be revisited
  jointly with the query mesh and environment before control integration.
- This round contains static point clouds only: no dynamics, clustering,
  filtering/tracking, future clouds, FAPP, or multi-UAV sensing.

## Final answer and next-stage file map

Yes: the independent path now performs

```text
MARSIM sensor PointCloud2 -> reconstructed world cloud -> N-frame/local cloud
-> Depth-CBF LocalBarrier(A,b,c).
```

It is a tested perception/regression path, not a closed-loop safety controller.
In a separately authorized next stage, the likely touch points are
`acado_model/quadrotor_model_thrustrates.cpp` (symbolic barrier and a redesigned
OnlineData layout/code generation), `include/acado_mpc/mpc_wrapper.h` and
`src/acado_mpc/mpc_wrapper.cpp` (stage-wise data injection),
`include/acado_mpc/mpc_params.h` and `src/acado_mpc/mpc_controller.cpp`
(configuration/plumbing), and `src/coni_mpc/num_sim_mpc.cpp` plus its header
(barrier subscription, freshness/fallback, diagnostics). None of those files
was changed in M1/M2.
