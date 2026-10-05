# Task Log

> Lightweight cross-session engineering state. Use only for substantial multi-step IMPLEMENT / DIAGNOSE / RESEARCH work.

## Current task

- Objective: Integrate DSP future occupancy with the non-inertial UAV MPC/HOCBF.
- Status: risk-region constructor, ROS messages/node, ACADO p=4 HOCBF, and MPC
  subscription/profile path implemented; the one-command MARSIM -> DSP -> risk
  region -> MPC path now builds and passes live topic-rate validation.
- Started: 2026-09-16

## Evidence / observations

- Existing Hungarian current/previous-cluster association now carries a
  monotonic persistent obstacle ID into each dynamic observation and newborn
  particle; static/background particles use ID -1.
- Particle IDs are copied during voxel movement and resampling and cleared on
  removal. Future statistics are keyed directly by `(stage, obstacle_id)`; no
  future DBSCAN or tracker was added.
- Each of the 21 DSP prediction stages accumulates weighted position moments from
  predicted in-map particle support. XY covariance is regularized and converted
  into a configurable Gaussian-moment probability ellipse.
- The covariance represents object extent plus measurement, motion-model, and
  particle/resampling spread. It contains no vehicle radius, controller
  uncertainty, or HOCBF safety inflation.
- DSP dynamic prediction is exported at 21 stages (0.0--2.0 s, 0.1 s spacing)
  on `/my_map/future_occupancy_3d`; its `occupancy` field is the DSP-paper
  voxel occupancy probability in [0,1], while `occupancy_mass` preserves the
  native particle-weight sum for diagnostics and local mass retention.
- `risk_region_constructor_node` clusters 26-connected voxels, retains the
  configured mass, constructs conservative p=4 superellipsoids, associates
  tracks across stages, and publishes `RiskRegionArray`.
- ACADO consumes 22 values per region (center, axes, Q, center velocity,
  center acceleration, active). `NumSimMpc` can subscribe to the region array
  with `cbf/use_risk_regions=true`, transform world regions to the UGV frame,
  and derive stage-wise center derivatives without differentiating fixed axes/Q.
- Typed output is `PredictedEllipseArray` on configurable default topic
  `/my_map/predicted_ellipses`; MarkerArray visualization defaults to
  `/my_map/predicted_ellipse_markers`. Both retain the source-cloud timestamp
  and `map` frame.
- `DSP-map` was linked into `marsim_ws/src` as `dynamic_occpuancy_map`; without
  this package discovery failed even though stale generated artifacts existed.
- The integration launch explicitly sets `/use_sim_time=false`; a stale global
  simulated-time setting had stopped all standalone-simulator timers because no
  `/clock` publisher was present.
- Eigen-aligned STL allocators were added for probability ellipse/moment
  containers. Before this fix `map_sim_example` crashed with SIGSEGV after the
  first point-cloud update.

## Decisions

- Name the feature "DSP obstacle-conditioned moment extraction + probability
  ellipse construction", not traditional ellipse fitting.
- Publish the DSP-paper occupancy probability to the planning-facing
  `occupancy` field and preserve native DSP mass in a separate
  `occupancy_mass` field. Keep legacy sphere obstacles as fallback when
  risk-region mode is off or its message is stale.
- Publish explicit invalid records when an obstacle-stage has insufficient
  support instead of inventing geometry.

## Changed files

- `DSP-map/include/dsp_dynamic.h`
- `DSP-map/include/dsp_probability_ellipse.h`
- `DSP-map/src/map_sim_example.cpp`
- `DSP-map/msg/PredictedEllipse.msg`
- `DSP-map/msg/PredictedEllipseArray.msg`
- `DSP-map/CMakeLists.txt`, `DSP-map/package.xml`
- `DSP-map/test/test_dsp_probability_ellipse.cpp`
- `TASK_LOG.md`
- `marsim_ws/src/coni_mpc/msg/RiskRegion.msg`
- `marsim_ws/src/coni_mpc/msg/RiskRegionArray.msg`
- `marsim_ws/src/coni_mpc/src/risk_region_constructor.cpp`
- `marsim_ws/src/coni_mpc/src/risk_region_constructor_node.cpp`
- `marsim_ws/src/coni_mpc/launch/risk_region_constructor.launch`
- `marsim_ws/src/coni_mpc/launch/dsp_mpc_sim.launch`
- `marsim_ws/src/dynamic_occpuancy_map` (symlink to `DSP-map`)
- `marsim_ws/src/coni_mpc/src/coni_mpc/num_sim_mpc.cpp`
- `marsim_ws/src/coni_mpc/src/acado_mpc/mpc_wrapper.cpp`
- `marsim_ws/src/coni_mpc/acado_model/quadrotor_model_thrustrates.cpp`

## Commands and verification

- `catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES=coni_mpc --pkg coni_mpc
  -j2` passes, including the risk-region node, ACADO wrapper, and numerical
  simulation node.
- Seven gtests pass: stationary center, constant-velocity displacement,
  anisotropic axes/yaw/SPD, probability-mass scaling, ID allocation/inheritance,
  two-object separation, and NaN/Inf rejection under `-ffast-math`.
- `git diff --check` passes and no new code uses the term "ellipse fitting".
- `catkin_make -DCATKIN_WHITELIST_PACKAGES='dynamic_occpuancy_map;coni_mpc' -j2`
  passes.
- With `dsp_mpc_sim.launch start_rviz:=false`, `/quad0_pcl_render_node/sensor_cloud`
  publishes at about 10 Hz, `/my_map/future_occupancy_3d` at about 10 Hz, and
  `/coni_mpc/risk_regions` at about 10 Hz; `rosmsg show coni_mpc/RiskRegionArray`
  succeeds after sourcing `marsim_ws/devel/setup.bash`.

## Unresolved questions

- RViz visual confirmation of the final scene remains a manual check; the live
  ROS topic chain and message type have been validated.
- The constructor now exposes `uav_radius`, `control_margin`, and
  `perception_margin`; the current launch default is 0.15 + 0.35 + 0.0 m,
  so the present independent perception margin is explicitly 0.0 m until it
  is calibrated from LiDAR/DSP data.
- Region fitting now solves the fixed-center/fixed-PCA-frame three-variable
  minimum-volume p=4 fit.  Degenerate point sets use a checked outer-box
  fallback; each voxel cube is expanded by the configured radius before PCA
  and fitting, making the UAV-center Minkowski safety enclosure explicit.
- Neighbour support is a normalized spatial-consistency score used to seed and
  order a 26-connected growth that retains 95% mass; it is not an extra
  probability or an uncalibrated hard cutoff.

## Next action

- Launch `dsp_mpc_sim.launch` for the one-command integration startup, then
  validate stage/frame alignment with a live DSP source.

## 2026-09-19: Single-UAV LiDAR-DSP-CBF simulation scene

### Objective

- Make `dsp_mpc_sim.launch` start one UAV, one static obstacle, and one
  deterministic dynamic obstacle, with obstacles reaching MPC-CBF only through
  the MARSIM LiDAR, DSP occupancy, and risk-region path.

### Changes and decisions

- Bound the MARSIM renderer to `/coni_mpc/quad_odom0`, the odometry actually
  published by the single-UAV numerical simulator.
- Use `A2_cylinder.pcd` as the only static obstacle and a dedicated
  `dsp_mpc_single_obstacles.yaml` as the only dynamic-obstacle definition.
- Disabled the numerical simulator's legacy analytic/random and figure-eight
  obstacle injection; changed the carrier trajectory from figure eight to a
  circle.
- Added real-time pacing, startup delay, and callback processing to the
  numerical simulator so 10 Hz LiDAR/DSP risk messages reach MPC before and
  during optimization.
- RViz now displays the real UAV0 MARSIM scan topic and no longer includes the
  stale camera-cloud display.

### Verification

- `catkin_make --pkg coni_mpc -j2` passes.
- Live isolated-master run measured approximately 10 Hz on
  `/quad0_pcl_render_node/sensor_cloud`, `/my_map/future_occupancy_3d`, and
  `/coni_mpc/risk_regions`; UAV0 odometry was approximately 100 Hz.
- MPC runtime output reported `using ... DSP risk-region tracks`, confirming
  that the CBF path consumed the perception-derived regions.

### Deferred by user

- The support score is enabled in the launch configuration as an ordering
  term; only its numerical weight and the DSP occupancy threshold remain
  data-calibration parameters.

## 2026-09-19: Static-obstacle avoidance timing diagnosis

### Objective

- Explain the repeated late avoidance near the static cylinder at roughly
  7 s and 20 s in `dsp_mpc_sim.launch`.

### Evidence and decision

- An isolated 27 s reproduction was recorded to
  `/tmp/dsp_static_diag.bag.active`; no matching archived run existed.
- The two closest approaches occurred at simulation times 3.516 s and
  16.313 s (about 6.5 s and 19.3 s after including the 3 s startup wait).
  UAV-body clearance to the radius-1 m cylinder was -0.018 m and -0.015 m.
- LiDAR contained thousands of correctly located cylinder returns at both
  approaches. Risk-region messages ran at about 9.94 Hz with 51 ms median
  receipt age, so timestamp delay was not the primary cause.
- Two spatial-interface faults were confirmed. MARSIM publishes the cloud in
  body/LiDAR FLU but publishes `sensor_pose` with the body-to-optical rotation;
  DSP then applies the same optical-to-FLU axis conversion to every point, so
  the effective cloud/pose transform is inconsistent. Separately, DSP exports
  its UAV-centred voxel coordinates with `frame_id=map` without a local-to-world
  transform. Risk regions were consequently absent or several metres from the
  cylinder at closest approach, and MPC reacted only after passing it.

### Next action

- Established one explicit MARSIM-to-DSP frame contract, transformed DSP's
  egocentric output to world coordinates before labeling it `map`, rebuilt, and
  repeated the same two-pass measurement. No CBF gains or DSP probability
  parameters were changed.

### Repair verification

- Added an explicit `body_flu` input mode for the MARSIM path while preserving
  the historical camera-optical default, and made the paired MARSIM pose use
  the same FLU rotation/translation.
- Added the synchronized UAV/map-origin snapshot to occupancy clouds,
  predicted ellipses, and markers before publishing them in `map`.
- The fixed 27 s reproduction `/tmp/dsp_static_fixed.bag` placed stage-0 risk
  centers near `(5,0)` before both passes. The minimum UAV-body clearance was
  approximately `1.56 m`, versus `-0.018 m` and `-0.015 m` before the repair.
- `map_sim_example` and `pcl_render_node` built successfully; launch XML and
  `roslaunch --nodes` checks passed. The remaining scheduler-overrun warnings
  did not produce a collision or alter the coordinate result.

## 2026-09-19: Risk-region selection follows the predicted UAV trajectory

### Objective

- Replace the risk-region `track_id` ordering used by the DSP-CBF path with the
  document-defined trajectory risk ordering.

### Change

- In `src/coni_mpc/num_sim_mpc.cpp`, build all valid stage-wise risk-region
  profiles first, then compute
  `rho_j = min_k(g_j,k(p_uav,k) - 1)` in the solver frame.
- Use the previous successful MPC predicted UAV trajectory when available;
  before the first solve, fall back to the reference window and then the
  current estimated position.
- Sort ascending by `rho_j` and pass the three smallest-margin tracks to
  ACADO. Track ID is used only as a deterministic tie-breaker.

### Verification

- `catkin_make --pkg coni_mpc -j2` passed.
- Launch XML, `roslaunch --nodes`, and source-targeted `git diff --check`
  passed.

## 2026-09-20: Fixed-frame minimum-volume p=4 risk fit

### Change

- Replaced the PCA-bound `3^(1/4)` outer-box axes with a deterministic
  three-variable coordinate maximization of
  `log(A)+log(B)+log(C)` under the document's p=4 boundary constraints.
- Kept the weighted center and PCA orientation fixed, and fit every selected
  voxel-cube corner in that frame.  Degenerate/ill-conditioned components use
  a checked conservative fallback.
- Applied the configured safety radius by expanding each voxel cube before PCA
  and fitting, so the voxel set plus Euclidean safety ball remains contained
  without a hidden post-fit scaling step.

### Verification

- `catkin_make --pkg coni_mpc -j2` passed.
- A replay of `/tmp/dsp_static_fixed.bag` through the rebuilt constructor
  produced non-empty 21-stage `RiskRegionArray` output (63 regions in the
  first populated frame) with finite centers and axes.
- Independent numerical checks recover the theoretical cube solution
  `a=b=c=3^(1/4)s` and all tested boundary constraints are at most one up to
  floating-point tolerance.

## 2026-09-20: DSP-paper voxel occupancy probability export

### Change

- Converted the future voxel mass used by the motion-planning topic with the
  DSP paper's rule
  `P_occ=min(M*(l'/l)^3,1)` for `l<=l'`, otherwise `P_occ=min(M,1)`.
- The active configuration has `l'=0.10 m` and `l=0.15 m`, so the published
  probability is `min(M,1)`.
- `/my_map/future_occupancy_3d/occupancy` now contains the probability; the new
  `occupancy_mass` field retains the raw particle-weight sum. Voxel positions
  remain the DSP voxel centers and the risk constructor consumes the
  probability field.

### Verification

- Rebuilt `dynamic_occpuancy_map` and `coni_mpc` successfully.
- A live synthetic LiDAR/pose stream produced a PointCloud2 with
  `occupancy` and `occupancy_mass`; all sampled probabilities were in `[0,1]`.
  The same stream produced non-empty `RiskRegionArray` output, confirming the
  existing risk node reads `occupancy` as its probability input and ignores the
  diagnostic mass field.
- The one-command `dsp_mpc_sim.launch start_rviz:=false sim_duration_sec:=8`
  path also reported DSP risk-region tracks being consumed by MPC after the
  probability-field change.

## 2026-09-20: World-frame LiDAR collision termination

### Change

- Added an optional world-frame `sensor_msgs/PointCloud2` collision monitor to
  `num_sim_non_one_point_node`.  It subscribes to the MARSIM rendered cloud,
  filters invalid points, builds a PCL KD-tree, and compares each UAV's world
  position against the nearest world-frame obstacle point.
- The DSP launch enables it with `/quad0_pcl_render_node/cloud` and `map` as
  the required frame.  Cloud timeout and an explicit point-sampling margin are
  configurable; the active launch uses zero margin so contact is based on the
  UAV radius only.
- Exact tangency is treated as collision (`distance <= threshold`) for both
  the LiDAR monitor and the existing analytic collision detector.  A hit
  reuses the existing collision marker, stop-all-simulators, metric, and
  three-second RViz hold path.

### Verification

- `catkin_make --pkg coni_mpc -j2` passed after linking the target with PCL.
- `roslaunch coni_mpc dsp_mpc_sim.launch --nodes start_rviz:=false` resolved
  the new parameters and active nodes successfully.
- A short one-command DSP launch resolved
  `lidar_collision/enabled=true`, topic `/quad0_pcl_render_node/cloud`, and
  frame `map`; its simulator completed the configured duration without a
  collision, while DSP and MPC continued to receive the normal streams.

## 2026-09-20: Clear the DSP UGV circle trajectory from the static cylinder

### Change

- Added the optional `ugv_circle_radius` parameter.  A positive override keeps
  the circular trajectory kinematically consistent by using
  `omega=v/radius`; zero preserves the historical `radius=v/w` behavior.
- Updated the obstacle-aware UGV planner's keep-out calculation to include
  `car_radius` (in addition to the UAV radius), so a path is not accepted when
  the UAV center clears an obstacle but the UGV footprint does not.
- For the dedicated collision-test scene, set the single UAV body-frame offset
  to `(0, 1.5, 2.0) m`.  This is an inward radial offset on the circular UGV
  path (not the previous tangential `(-1,0,2)` offset), so the UAV nominal
  circle reaches the cylinder while the UGV remains outside it.
- The DSP scene now uses `ugv_circle_radius=7.0 m`.  Its static cylinder has
  center `(5,0)` and radius `1 m`; the old 6 m circle was tangent at the
  centerline and collided after including `car_radius=0.15 m`.

### Verification

- `catkin_make --pkg coni_mpc -j2` passed.
- A direct launch resolved and logged `radius=7`, `yaw_rate=0.428571`, and
  `car_start=(0,-7,0)`.
- Geometry check gives static-cylinder surface clearance `0.85 m` after the
  0.15 m UGV radius; with the new UAV offset, the UAV-cylinder surface
  clearance is `-0.65 m` (intentional collision-test condition).  The
  deterministic dynamic box remains clear in the configured scene.
- A full `dsp_mpc_sim` run with `cbf_enabled=false`,
  `cbf_use_in_sim=false`, and `sim_duration_sec=10` triggered the LiDAR
  collision exit at `distance=0.140731 <= threshold=0.15`; the UGV stayed on
  the radius-7 path.  The normal DSP launch defaults keep CBF enabled, so that
  mode may avoid the nominal UAV collision instead of triggering the exit.

## 2026-09-20: Support-aware persistence and automatic region splitting

### Objective

- Execute the document's missing front-end steps for heterogeneous static
  obstacles (cylinders, connected boxes, and pedestrians) without introducing
  shape-specific classifiers.

### Changes and decisions

- Added normalized 26-neighbour support and a connected priority-queue growth
  so each selected voxel set preserves at least the configured 95% component
  mass while spatially supported voxels are preferred over isolated tails.
- Added stage-wise continuity confirmation across the 21 DSP slices and a
  second callback-level track confirmation across published map frames. A
  high-confidence new region is admitted immediately; otherwise two
  consecutive frames are required.
- Added recursive PCA-axis bisection. A split is accepted only when both voxel
  subsets are 26-connected and the total fitted p=4 volume decreases by the
  configured fraction. This handles non-convex connected boxes and elongated
  pedestrian/cylinder clouds without shape labels.
- Kept the document's final representation: inflated voxel risk set,
  conservative p=4 superellipsoid, stage-wise center derivatives, UAV-trajectory
  ranking, and the existing three-region ACADO interface.
- Set `safety_variant: A1_hard_cbf` and `cbf/slack_max: 0` in
  `num_sim_non_one_point.yaml`; the runtime already aborts if a nonzero slack
  appears in this variant.

### Verification

- `catkin_make --pkg coni_mpc -j2` passes.
- Added and passed `risk_region_constructor_test` for connected 95%-mass/stage
  persistence and recursive splitting.
- A five-second `dsp_mpc_sim.launch start_rviz:=false` smoke run resolved
  `A1_hard_cbf`, `zero_slack_required=true`, the support/persistence/split
  parameters, and MPC consumed trajectory-ranked DSP risk regions. Shutdown
  emitted the pre-existing map-simulator Boost mutex warning after the
  simulator had already finished cleanly.

## 2026-09-20: Prevent DSP risk-barrier dropouts and stale-stage timing

### Diagnosis and changes

- Confirmed from a clean ROS master that the old `cbf/risk_regions_timeout=0.5`
  disabled all risk constraints whenever the constructor callback age reached
  0.61 s; the UAV then collided at 0.137 m while the constructor was still
  processing a dense cloud.
- Risk-region construction now uses a latest-sample policy: queued clouds older
  than 0.75 s are discarded instead of increasing latency without bound.
- MPC keeps the last complete risk horizon for a bounded four-second gap and
  shifts the DSP stage index by the measured message age, clamping to the last
  predicted stage. This prevents a delayed stage-0 snapshot from being used as
  the current-time obstacle.
- Added callback timing and freshness diagnostics for reproducible diagnosis.

### Verification

- `catkin_make --pkg coni_mpc -j2` passed.
- `run_tests_coni_mpc_gtest_risk_region_constructor_test` passed (2/2).
- Clean `dsp_mpc_sim.launch start_rviz:=false sim_duration_sec:=8` run with
  the dynamic box and static cylinder completed all 800 scheduler cycles with
  no `Collision detected`, no `[MPC][RECOVERY]`, and no risk timeout during
  the simulation. The risk age was time-aligned (including shifts up to 26
  stages) and remained fresh under the four-second bound.

## 2026-09-20: Close the remaining perception-latency collision gap

### Diagnosis and changes

- A long run showed physical LiDAR contact at `distance=0.139222 m` even
  though the MPC solve was successful and `slack=0`. The cause was not a
  soft-CBF relaxation: the DSP fit could be several seconds old while the
  finite 2 s forecast was still treated as a current barrier. The post-contact
  three-second hold also made the RViz scene look frozen.
- The risk-region constructor now keeps only the newest cloud in its ROS
  callback and performs the expensive fit on a worker thread. This prevents
  callback queue starvation without changing the voxel/mass/fit algorithm.
- Risk messages older than the 2 s DSP prediction horizon are rejected; they
  are no longer clamped to the final future slice. The constructor input-age
  bound and MPC timeout were changed to 2 s accordingly.
- Added a relative-degree-one LiDAR CBF safety filter. It projects the already
  computed world velocity only when
  `2(p-o)^T v + alpha (||p-o||^2-r_safe^2) < 0`, with `alpha=8` and
  `r_safe=0.15+0.35=0.50 m`. The terminal contact detector is unchanged and
  still counts exact UAV-radius tangency as collision.

### Verification

- `catkin_make --pkg coni_mpc -j2` passed after the changes.
- An 8 s clean DSP launch completed 800 cycles with no collision or solver
  recovery. The log showed the risk horizon being rejected after 2 s and the
  LiDAR CBF projecting commands at surface distances about 1.00 m and 0.66 m.
- A 20 s clean DSP launch completed all 2000 cycles with no collision and no
  recovery. Mean scheduler time was 3.70 ms; occasional deadline overruns
  were logged but did not stop the simulation.

## 2026-09-21: Remove the nearest-point LiDAR CBF fallback

- Removed the relative-degree-one CBF based on the current nearest point-cloud
  point. It had no future obstacle prediction and its nearest-point selection
  could introduce discontinuous gradients.
- The LiDAR monitor remains only as the terminal physical-contact detector.
  Avoidance is again provided by the DSP future-occupancy risk regions and the
  hard MPC HOCBF; stale risk regions are rejected after the 2 s forecast
  horizon.
- Removed the corresponding runtime parameters and post-MPC velocity
  projection. `catkin_make --pkg coni_mpc -j2` passed after the removal.

## 2026-09-21: Propagate MPC failures and hold the last valid command

- `MpcWrapper` now propagates nonzero ACADO preparation/feedback statuses.
- `MpcController` caches only finite commands from successful solves. On CBF
  update failure, solver failure, or invalid output it reuses the last valid
  command; before the first successful solve it returns a zero command.
- `catkin_make --pkg coni_mpc -j2` passed after the change.

## 2026-09-21: Make cbf/slack_max the sole slack bound

- Removed the `safety_variant`-based zero-slack abort from `NumSimMpc`.
- `cbf/slack_max` now directly determines the slack bounds: zero fixes the
  three slacks to zero, while a positive value permits bounded differential-CBF
  slack and its configured quadratic penalty.
- Kept `safety_variant` as metadata and for the independent A0 no-CBF baseline
  validation; it no longer gates nonzero slack in the MPC controller.
- Updated the default YAML to use `slack_max: 2` with `A2_soft_cbf` as a label.
- `catkin_make --pkg coni_mpc -j2` passed after the change.

## 2026-09-21: Configurable DSP risk-stage time alignment

- Added `cbf/risk_region_time_alignment` (default `true`) so the existing
  age-based DSP stage shift can be enabled or disabled for an A/B comparison.
- Kept risk-region construction, trajectory-based ranking, coordinate
  conversion, and hard HOCBF logic unchanged; runtime logs now report the
  alignment state and applied stage shift.
- `catkin_make --pkg coni_mpc -j2` passed after the change.

## 2026-09-21: Controller-side probabilistic Field-HOCBF

- Extended `/my_map/future_occupancy_3d` with `vx/vy/vz`. Each velocity is the
  probability-mass-weighted mean of the same DSP particles accumulated into
  that future destination voxel; position remains in `map` and stages remain
  0.0--2.0 s at 0.1 s spacing.
- Added a validated, immutable `PointCloud2` snapshot path for the 21-stage DSP
  occupancy field and age/residual-time alignment. Fresh nonempty fields take
  priority over legacy risk regions; stale/empty fields are not retained.
- Added stage-wise world/non-inertial warping, nominal-trajectory pruning, and
  the frozen-nominal affine HOCBF row with probability-scaled slack cost.
- Regenerated ACADO with `ACADO_NOD=85` and one additional path row. A clean
  temporary catkin build passed, and both focused pruning/math tests passed.
  A separate clean CMake build of DSP-map's `map_sim_example` also passed with
  the expanded ten-field voxel message.

### Final Field-HOCBF integration verification

- Field coefficient updates now use the normal prepared-for-next-cycle RTI
  pipeline; only field-mode transitions request a cold solve. Field mode also
  uses an unbounded-above soft slack as specified by the affine HOCBF model.
- `dsp_mpc_sim.launch` defaults to `sim_control_dt_sec=0.02` (50 Hz).
- Rebuilt controller smoke run: field fresh/active, zero ACADO feedback
  failures, zero scheduler overruns over 200 cycles; solve-only timing samples
  were 1.0--2.7 ms. Reported total including preparation reached 3.2 ms,
  which is outside the solve-only budget but did not overrun the 20 ms period.

## 2026-09-22: Strengthen dynamic-obstacle crossing scenario

- The original dynamic box path stayed several metres from the UAV's nominal
  5.5 m-radius orbit during the short validation window.
- Moved the box to `[2.3, -5.8, 1.5]` with velocity `[0.0, 0.8, 0.0]`, producing
  a nominal crossing near `t=1 s` while retaining a vertical overlap with the
  UAV footprint.
- A 50 Hz smoke run showed `field_hocbf fresh=true active=true`, nonzero
  avoidance commands/slack, zero ACADO feedback failures, and zero scheduler
  overruns over 200 cycles.

## 2026-09-22: Align the moving obstacle with the second UAV lap

- The one-shot path had already passed the UAV's nominal orbit before the
  second lap, so its future field could not overlap the UAV arrival time.
- The deterministic box now starts at `[0.0, 8.63, 1.5]` and moves at
  `[0.0, -0.8, 0.0]`.  Accounting for the 3 s sensor warm-up, it reaches the
  UAV's nominal second-lap point near simulation `t=14.7 s` / wall `t=17.7 s`;
  the UGV centerline remains 1.5 m away.
- Replaced duplicate-prone `$(find coni_mpc)` data/config references in the
  DSP launch chain with `$(dirname)`-relative paths, so the current workspace
  cannot silently load the other workspace's obstacle YAML.
- A current-workspace 30 s / 1500-cycle run loaded the new initial/velocity
  values, showed fresh active Field-HOCBF samples and strong avoidance commands
  around the second-lap encounter, with zero ACADO feedback failures and zero
  scheduler overruns.

## 2026-09-22: Expand validation scene to three static and two dynamic obstacles

- Generated `A2_three_cylinders.pcd` by placing three copies of the A2 cylinder
  around the radius-5 m ring; the map publisher confirmed 27,540 points.
- Set the deterministic MARSIM box count to two.  The first box preserves the
  second-lap bottom encounter; the second crosses the UAV's left-side first-lap
  point with a separate constant-velocity track.
- A 22 s Field-HOCBF smoke run loaded `dynobject_num=2`, produced fresh active
  fields and large avoidance commands, and completed 1,100 cycles with zero
  scheduler overruns and no controller failure messages.

## 2026-09-22: Add an irregular static L-shaped wall

- Added a perpendicular two-arm wall near the top of the UAV orbit to the
  static map, while retaining the three ring cylinders and two dynamic boxes.
- The combined `A2_three_cylinders_l_wall.pcd` contains 29,262 points and is
  still published through the same LiDAR/DSP map path.

## 2026-09-22: Non-inertial frozen MPC+CBF baseline check

- Exposed explicit `frame_mode` and `ugv_rollout_mode` launch arguments,
  defaulting to `noninertial` and `frozen`.
- With `cbf_use_field_hocbf=false`, `cbf_use_risk_regions=true`, and the
  current three-cylinder + L-wall + two-dynamic scene, the baseline selected
  the requested modes correctly.
- At 100 Hz it entered repeated ACADO status `-2` failures once dense risk
  regions arrived; at 50 Hz the solver stayed nominal but the UAV contacted
  the A2 cylinder at about 8 s.  This is a baseline limitation in the current
  dense scene, not a Field-HOCBF run result.

## 2026-09-26: Diagnose the reported 160 s Field-HOCBF contact

- Run identity: `roslaunch-jjm-809458.log`, start 10:57:25, current DSP binary,
  field=true, risk=false, RViz=true, duration=300; source d_safe=0.10.
- `rosout.log:953` records contact at epoch 1790391604.092 (about 159 s
  after launch), clearance 0.141861 m versus collision threshold 0.15 m.
- Last logged command, 0.486 s before contact: z=-1, slack=0.0837912.
  Last field sample, 0.761 s before contact: active=true, age=0.121295 s.
  Measured height fell from 1.78931 to 1.19676 m in the preceding second;
  contact height was 0.50916 m. Tracking reference height remained 2 m.
- Across this run: 155 throttled slack samples, max=1.53411; maximum
  logged field age=0.182256 s; five scheduler overruns; no solver failure
  messages found. Field branch still overrides configured slack_max=0
  with infinity. No algorithm or parameter changes made during diagnosis.
- Limitation: no metrics CSV or per-cycle h/A/b/selected voxel records;
  these logs establish contact with active soft constraints, but do not
  identify why the field commanded descent or whether the contact voxel
  was retained. Next diagnosis needs those per-cycle measurements.

## 2026-09-26: Upload the current source snapshot to GitHub

- Requested destination: `wfaff2/GS--DSP`, existing `origin`, branch `main`.
- Include current control/perception sources, launch/configuration files,
  maps and local dependency modifications. Vendor the former LV-DOT and
  vision_msgs gitlinks so their actual sources are available in a clone.
- Add root ignore rules and stop versioning build/devel outputs, caches and
  recorded experiments; all original workspace files remain on disk.
- Pre-push checks: no remaining gitlinks, no files at or above GitHub's
  100 MiB limit, and no matches for common private-key/token patterns.

## 2026-09-26: Diagnose first-obstacle contact in current Field-HOCBF run

- Objective: explain the immediate first-obstacle collision for
  `dsp_mpc_sim.launch` with `cbf_use_field_hocbf=true` and risk regions off.
- Evidence: the launch geometry intentionally places the nominal UAV circle at
  radius 5.5 m through the static obstacle ring; the current rosout then shows
  `field_hocbf available=true active=true`, repeated ACADO status `-2` failures
  and recovery, followed by contact at clearance 0.135067 m versus threshold
  0.15 m.
- Decision: the immediate mechanism is solver failure followed by reuse of the
  previous valid control, not failure to receive the field. Likely upstream
  contributors are the non-monotone Gaussian shell field and an extreme or
  infeasible frozen field constraint; exact status `-2` cause is not logged.
- Changed files: none in source or configuration; this entry only records the
  diagnosis.
- Verification: correlated timestamps in `/home/jjm/.ros/log/cd794b1a-ade9-11f1-aae2-25bc297d88f9/rosout.log`
  with `field_hocbf.h`, `mpc_wrapper.cpp`, and `mpc_controller.cpp`.
- Next action: log field `A/b`, selected voxel count, residual/slack, and solver
  diagnostics; separately test a geometry-safe reference path and a braking
  fallback before changing HOCBF parameters.

## 2026-09-27: Diagnose clustered ACADO infeasibility in latest Field-HOCBF run

- Run identity: `roslaunch-jjm-1868016.log`, 09:57:04--09:58:39,
  Field-HOCBF enabled and risk regions disabled.
- The eight visible `feedbackStep status -2` warnings are one-second-throttled;
  recovery diagnostics show 20 failed solve cycles in eight clusters between
  launch-relative 23.713 s and 84.870 s.
- qpOASES simplified status `-2` means QP infeasible. All recorded predictions,
  inputs, and objectives remained finite, and obstacle-update failures stayed
  at zero; the controller reused its previous valid command after each failure.
- Scheduler overruns occurred at 4.408 s, 5.494 s, and 93.917 s and did not
  coincide with the infeasibility clusters.
- Body yaw is not an input to the Field-HOCBF formula, and the simulated LiDAR
  has a 360-degree horizontal field of view. The run therefore does not support
  body orientation toward a cylinder as the cause. Approaching an obstacle in
  the predicted travel corridor may tighten the field constraint, but the run
  lacks synchronized obstacle identity and per-cycle `A/b/h/hdot`, selected
  voxel, slack-bound, and QP residual records needed to verify that mechanism.
- Changed files: none in source or configuration; this entry records diagnosis.

## 2026-09-27: Load UAV MPC limits and tracking constants from YAML

- Added startup parameters `a_max_xy`/`a_max_z`; existing `max_v_*` and
  `tau_v_*` now feed the same MPC configuration object with finite-positive
  validation.
- Runtime input bounds use YAML `max_v_*`; mixed acceleration path bounds use
  YAML `a_max_*`; the first-order MPC dynamics receive inverse `tau_v_*` through
  two appended ACADO OnlineData entries.
- Regenerated the ACADO solver with `ACADO_NOD=87` and rebuilt `coni_mpc`.
- A 0.5 s one-UAV circular offline smoke run loaded 1.5/1.0 m/s, 6/4 m/s^2,
  and 0.2/0.2 s, solved nominally, completed 50 cycles, and exited cleanly.

## 2026-10-04: Diagnose delayed dynamic-obstacle anticipation

- Objective: explain the reported dynamic encounter around 7--8 s; no controller
  or perception tuning performed during this diagnosis.
- Original run: ROS log `f954ce52-bf97-11f1-be3c-93b24bfac4cc`, started 10:04;
  field available/active with age 0.061--0.141 s and no recorded solver failure
  in the relevant interval. Simulation starts about 3.1 s after node startup.
- Current-config reproductions at commit `a7d66d0` with dirty working tree are
  archived in `/tmp/dsp_anticipation_20261004` and
  `/tmp/dsp_anticipation_sensor_20261004`: source/config snapshots, loaded
  parameters, metadata, logs, capture scripts and timestamped NPZ data.
- The first reproduction contacted dynamic box0 at simulation time about
  6.22 s; static PCD was more than 3 m away. Future stages do propagate motion
  later, but early high-occupancy support near the moving box has zero velocity
  and identical stage 0/10/20 positions. Later dynamic predictions also show
  spurious positive X velocity, despite true X velocity being zero.
- Supported upstream mechanisms: DSP retains a forward 84 x 48 degree FOV
  despite the 360-degree LiDAR. Offline replay of the second capture retained
  zero box points at 0.08/0.48/0.88 s, although LiDAR had observed the box.
  After entry, several box clusters cross max-height 2.60 m or XY-span 1.15 m
  gates and are classified static, whose points/newborn particles get zero
  velocity. Replay approximates PCL and pose synchronization; internal cluster
  features were not directly logged, so exact per-frame classification remains
  a replay-supported conclusion.
- Source audit confirms per-stage DSP data reaches generated ACADO field
  constraints; `frozen` UGV mode still predicts a constant-curvature trajectory.
- Next action if a fix is requested: align DSP observation processing with the
  LiDAR FOV and verify dynamic classification/velocity continuity before
  changing CBF gains; record internal cluster decisions and per-stage barriers
  to validate the repair. User's original run was not exactly replayed.

## 2026-10-04: Repair DSP LiDAR observation and marginal cluster rejection

- Objective: repair the dynamic-anticipation mechanisms identified in the preceding diagnosis.
- Evidence: identical 20 recorded sensor-cloud frames and nearest recorded odometry
  replayed through a freshly compiled pre-change baseline, FOV-only build, and final
  build. In FOV-only replay, 10/13 isolated box candidates at 1.487--3.895 s failed
  max-Z / XY-span gates, leaving no consecutive box matches. Tall early clusters
  still contain merged geometry and remain static.
- Changes: opt-in 360 x 90 degree sensor-frame angular bins, periodic horizontal
  neighbors, and inverse sensor rotation for particles; legacy camera FOV remains
  the default. LiDAR-only cluster limits add one filter voxel in Z and one diagonal
  voxel in XY (2.70 m / 1.2914 m at 0.1 m resolution). Count, center-height, speed
  and CBF settings remain unchanged. Added default-off timestamped gate/match logs.
- Changed files: DSP-map/include/dsp_dynamic.h, new include/dsp_fov_grid.h,
  src/map_sim_example.cpp, CMakeLists.txt, new test/test_dsp_fov_grid.cpp;
  marsim_ws/src/coni_mpc/launch/dsp_mpc_sim.launch enables LiDAR mode and exposes
  dsp_cluster_diagnostics. Existing dirty changes preserved.
- Verification: affected map target built; seven focused geometry/FOV tests passed;
  launch XML and scoped diff check passed. Final replay had 20/20 outputs, 13/13
  isolated candidates and 12 consecutive matches of the same ID from 1.688 s;
  mean estimated vy -0.5938 m/s vs truth -0.60, mean abs vx 0.0531 m/s. Two-second
  field displacement was -1.13 to -1.30 m after 2 s instead of static predictions.
- Closed-loop check: current-config 8.01 s run, 400 UAV samples / 40 field samples,
  no recorded contact or feedback-step failure; sampled box AABB separation minimum
  0.3198 m vs UAV radius 0.15 m. This is a short reproduction, not a full-lap or
  exact replay of the user's original run. Three box speed matches were rejected;
  static fragments/occlusion and matching noise remain possible.
- Archive: /tmp/dsp_anticipation_fix_20261004 contains commit/dirty metadata, input
  and binary hashes, before/FOV-only/after source snapshots, replay/capture scripts,
  NPZ outputs, internal logs, and verification summaries. Particle randomness is
  not bit-identical between runs; poses are approximated by nearest saved odometry.
- Existing limitation: boost mutex-lock exception during shutdown appears in both
  the earlier pre-change reproduction and the new closed-loop run. It was not
  modified in this scoped perception repair. No physical hardware was commanded.
- Next action: none required for this repair; longer evaluation or shutdown-race
  diagnosis is separate work.

## 2026-10-04: Align simulation control to 50 Hz and record validation runs

- Objective: set the numerical simulation control period to 0.02 s (50 Hz) and verify the logging path.
- Changes: `marsim_ws/src/coni_mpc/parameters/num_sim_non_one_point.yaml` and the node default now use `sim_control_dt_sec=0.02`; the external-baseline guard/error text is aligned to 50 Hz. `dsp_mpc_sim.launch` exposes `metrics_csv` and `run_tag` and passes them to the numerical simulator. Existing unrelated dirty changes were preserved.
- Verification: `catkin_make --pkg coni_mpc -j2` passed. Integrated field runs produced 600/1000 rows at median 0.02 s and reported no collision, but `obstacle_count=0` made the solver metrics track-only; they do not establish closed-loop obstacle avoidance. A separate 8 s numerical run with three explicit obstacles activated CBF for 154/550 steps but had 304 solver-failure/recovery steps (`fail_rate=0.552727`), max speed 44.69 m/s and max backward-difference acceleration 311.26 m/s².
- Archives: `/tmp/dsp_50hz_20261004`, `/tmp/dsp_50hz_20261004_long`, `/tmp/dsp_50hz_numeric_20261004`; metadata is in `/tmp/dsp_50hz_20261004/experiment_metadata.json`. No physical hardware was commanded.
- Unresolved: the explicit-obstacle numerical regression is unstable and requires separate solver/constraint diagnosis before claiming normal quadrotor performance or reliable early avoidance.

## 2026-10-04: Align to 100 Hz and instrument per-cycle obstacle/CBF evidence

- Objective: make the numerical control loop 100 Hz and distinguish late CBF activation from a small trajectory deviation.
- Changes: `sim_control_dt_sec` is now 0.01 in the main YAML, DSP integration launch, and numerical node default/external-baseline guard. Per-cycle step CSV now records nominal reference position/velocity, one-step MPC predicted position/velocity, nominal-horizon minimum obstacle distance/clearance, solver distance, `h`, and slack.
- Verification: `catkin_make --pkg coni_mpc -j2` passed. The integrated DSP launch logged `system_dt=0.01`, `control_dt=0.01 (100 Hz)` and 1200 rows at 0.01 s, but had 78 scheduler overruns and still configures `obstacle_count=0`, so its solver obstacle metrics are unavailable.
- Diagnostic run: `/tmp/dsp_100hz_numeric_20261004_record` contains 1100 rows at 0.01 s with three explicit obstacles. CBF became active at 1.16 s; at that first active cycle nominal-horizon minimum distance was 0.295763 m and minimum clearance -0.654237 m, current solver distance was 2.575490 m, `h=5.730650`, and slack=0. All 310 CBF-active rows coincided with solver failures; total solver fail rate was 0.55 and slack remained zero. Actual-to-reference error was already 0.824 m at 1.11 s and 1.039 m at first CBF activation.
- Interpretation: evidence favors a late/failed CBF intervention rather than a subtle nominal-path offset. Because the solver is failing before and throughout activation, this is not a clean proof of the intended CBF law; solver feasibility must be repaired before performance claims.

## 2026-10-04: Diagnose latest 50 Hz field-HOCBF run

- Objective: determine whether the latest `dsp_mpc_sim.launch cbf_use_field_hocbf:=true cbf_use_risk_regions:=false` run failed and identify the mechanism.
- Evidence: latest ROS log `/home/jjm/.ros/log/7111672a-bfcf-11f1-8ab6-0d3644872637` ran about 52.4 s before external shutdown. It contains 50 MPC timing samples, maximum total time 2.6 ms, and zero scheduler-overrun messages. Field-HOCBF was available on 49/50 samples and active on 36.
- Failure: one ACADO `feedbackStep` returned status -2 at 1791103346.540 s. `solver_status=false`, objective/states/inputs were finite, `obstacle_update_fail_count=0`; the controller kept the previous valid command, armed a cold restart, and solved again on the next cycle. The failure context had `cbf_active=false` and zero obstacle state, so this event is not evidence of late CBF activation.
- Other observations: no runtime collision/contact message was found. The MPC node exited 0 during roslaunch shutdown; map/rendering processes returned shutdown-side signals. No metrics CSV was requested, so predicted distance, h, slack, and nominal/actual trajectories cannot be quantitatively evaluated for this run. The launch file does not declare or forward `cbf_use_risk_regions`, so that selector is not verified by this command.
- Decision: classify the run as recovered but not solver-clean; 50 Hz fixed the deadline-overrun issue, while the isolated warm-start/ACADO status failure remains unresolved. Repeat with an explicit metrics archive and natural `sim_duration_sec` completion before changing CBF parameters.

## 2026-10-04: Diagnose apparent late avoidance in short latest run

- Objective: explain the user's observation that the UAV approached an obstacle closely before turning, despite no MPC failure.
- Evidence: latest archive `/home/jjm/.ros/log/626e190e-bfd2-11f1-8ab6-0d3644872637` lasted about 13 s and was externally shut down, so it is not a complete avoidance replay. The log has no solver failure or scheduler overrun. Field-HOCBF was unavailable at 1791104577.493, available but inactive at 4578.497 and 4579.501, then active from 4580.504 onward. Snapshot age stayed 0.12--0.16 s. Active cycles used nonzero corrective commands; slack was nonzero only at 4583.514 and 4587.528 (0.00894 and 0.00592).
- Limitation: this run does not log `h`, predicted obstacle distance/clearance, obstacle state, or actual contact distance. Therefore it cannot prove late perception/prediction. Available evidence favors an already-active constraint whose visible trajectory change is limited by activation threshold, dynamics, and control offset, but the root cause remains unverified.
- Decision: do not tune CBF gains from this short run. A complete run with per-cycle metrics and natural termination is required to distinguish late activation from insufficient trajectory deviation.

## 2026-10-04: Analyze complete 30 s metrics run

- Objective: evaluate the explicit 50 Hz run with `metrics_csv=/tmp/dsp_50hz_latest/metrics.csv`.
- Evidence: `metrics_steps.csv` has 1500 rows from 0.00 to 29.98 s at 0.02 s. All 1500 rows have `solve_ok=1` and `solver_fail_flag=0`; pipeline time is 0.919--3.965 ms. Summary reports collision=0, fail_rate=0, tracking RMS=0.467249 m, p95=1.024215 m, and max tracking error=1.409486 m at sim_time 6.94 s.
- Instrumentation limitation: launch sets `obstacle_count=0`; all rows are `solver_track_only=1`, `solver_cbf_active=0`, `solver_active_obstacle_key=none`, and `solver_h`, predicted obstacle distance/clearance, and truth clearance are NaN. Thus the CSV does not quantify field-HOCBF obstacle avoidance.
- ROS evidence: Field-HOCBF was available after startup and first became active around wall timestamp 1791105168.916 (about 3.0 s after the first MPC sample); it remained active through the interval where tracking error peaked near sim_time 6.94 s. No MPC failure or contact message occurred.
- Decision: this run passes timing/solver health but is inconclusive for “late CBF vs insufficient trajectory deviation.” The visible deviation after field activation is consistent with activation/vehicle-response effects, but field `h`, predicted clearance, and obstacle identity must be logged before changing gains.

## 2026-10-04: Parameter review for earlier Field-HOCBF avoidance

- Objective: identify parameter changes that could improve early avoidance without blind tuning.
- Current configuration is internally consistent on timing: 50 Hz control, 100 Hz simulation, ACADO model dt 0.1 s, 21 prediction samples (2.0 s), field timeout 0.3 s; the complete run had field snapshot age 0.06--0.16 s and no solver failures.
- Main questionable choices: `ugv_rollout_mode=frozen` in a moving non-inertial scene can create frame/obstacle prediction mismatch; `field_prune_radius=1.30 m` may give little reaction time for a moving obstacle; `slack_max=200` is permissive but its effect is unverified; `max_v_xy=1.5 m/s` was reached during the largest tracking deviation. `preview_steps=5` applies to the analytic obstacle branch and is not the active Field-HOCBF path when launch forces `obstacle_count=0`.
- Decision: first add Field-HOCBF-specific h/clearance/candidate metrics, then run one-variable A/B tests in this order: `ugv_rollout_mode=stage`, `field_prune_radius` increase, and only then `field_d_safe`/input authority/slack. Keep alpha/timing unchanged until metrics identify the mechanism.

## 2026-10-04: Add Field-HOCBF horizon diagnostics and validate frozen rollout

- Objective: record Field-HOCBF barrier data per MPC stage before changing rollout or CBF parameters.
- Changes: `field_hocbf::Constraint` now exposes `hdot`, `Lf2`, minimum/smooth distances; `NumSimMpc` retains per-stage Field-HOCBF diagnostics; the simulator writes `*_field_hocbf_steps.csv` with predicted distance, h, hdot, affine coefficients, residual, slack, nominal/predicted horizon states, and current measured state. `ugv_rollout_mode=frozen` and `preview_steps` were unchanged.
- Verification: `catkin_make --pkg coni_mpc -j2` passed. A 30 s 50 Hz run at `/tmp/dsp_field_hocbf_log_20261004` produced 1500 MPC rows and 31500 Field-HOCBF stage rows; all solver rows were successful, with two scheduler overruns. A 10 s repeat at `/tmp/dsp_field_hocbf_residual_20261004` produced 500 MPC rows and 10500 stage rows with residual logging.
- Evidence: the 10 s run had 6069 valid/active field stages, minimum logged h=-0.301542 m, and 3166/5752 finite post-solve affine residuals below zero. Thus the data does not support “h<0 while the field constraint is satisfied”; it shows predicted-horizon h violations together with many residual violations. This identifies a field-constraint consistency/feasibility issue requiring separate diagnosis before adding an explicit h>=0 row.
- Test note: the existing `field_hocbf_test` pruning test still fails (expected 2, got 10) under the repository's current dirty configuration; the two analytical derivative tests pass. This failure predates the diagnostic logging change and was not modified.

## 2026-10-04: Correct Field-HOCBF residual diagnostic and recheck ACADO path row

- Finding: ACADO enforces the profile value `b_profile = b + A*(k_v*nominal_velocity + k_p*nominal_position)`, while the first diagnostic residual subtracted the unshifted `b`. The earlier negative-residual count was therefore a logging error, not evidence that ACADO ignored the path constraint.
- Changes: added `field_b_profile` to the per-stage debug record/CSV and compute residuals against the exact profile value used by ACADO.
- Verification: `catkin_make --pkg coni_mpc -j2` passed. A fresh 10 s, 50 Hz Field-HOCBF run at `/tmp/dsp_field_hocbf_residual_fixed_20261004` produced 500 MPC rows, zero `solve_ok` failures, and 10,500 field-stage rows. Among 5,803 finite active stages, the corrected residual minimum was `-0.0` after CSV precision and no value was below `-1e-4`; minimum logged nominal `h` was `-0.339988`.
- Interpretation: the actual affine ACADO path row is being satisfied in this run. Negative nominal `h` remains a separate discrete-time/barrier-margin question and is not proof of path-row infeasibility. The pruning unit-test mismatch remains independent.

## 2026-10-05: Verify side-pass velocity kick

- Experiment: 10 s, 50 Hz Field-HOCBF run at `/tmp/dsp_field_hocbf_sidekick_20261005`, using the current launch parameters (`field_prune_radius=2.0`, `R_v_xy=100`, `field_seed_count=8`, `slack_max=200`). All 500 MPC rows solved successfully.
- Evidence: applied XY command changed by `1.154 m/s` between 7.06 and 7.08 s and by `1.129 m/s` between 7.20 and 7.22 s, equivalent to command-rate changes of about `58` and `56 m/s^2` over one 20 ms cycle. Around 7.08 s, stage-0 `A` rotated about 11.6 degrees while `b_profile` changed by about `10.8`; around 7.22 s, `A` continued rotating and `b_profile` changed by about `3.6`. At 8.36 s, `A` flipped by about 174 degrees and `d_min` jumped from `0.553` to `1.303 m`, indicating a field/candidate switch, although the largest command jump occurred earlier.
- Interpretation: the side kick is verified as a real command discontinuity. The strongest supported mechanism is unsmoothed stage-wise field/pruning updates combined with no direct inter-cycle velocity-command slew constraint. It is not an MPC failure. Further separation of candidate switching from velocity-estimation noise requires logging selected-point count/identity and obstacle-relative velocity per stage.

## 2026-10-05: Diagnose square-prism Field-HOCBF run

- Experiment: `square_prism_50hz` completed 30 s at `/tmp/dsp_square_prism`, using the new 1 m x 1 m static square-prism map. `metrics_steps.csv` has 1500 rows at 0.02 s; all `solve_ok=1`, all `solver_fail_flag=0`, and summary `collision=0`.
- Evidence: applied command jump peaked at `1.6198 m/s` from 6.38 to 6.40 s (`80.99 m/s^2` equivalent over 20 ms), followed by `1.4706 m/s` at 6.26 s and `1.3225 m/s` at 6.42 s. Stage-0 Field-HOCBF was active on 1144/1500 cycles. At 6.40 s, `d_min=0.0821 m`, `h=-0.2251`, `hdot` changed from `+0.1924` to `-0.2081`, `Lf2` from `-5.1308` to `-0.7518`, and slack changed from `0.1340` to `0.0026`; at 6.42 s `hdot=+0.1926`, `Lf2=-6.3529`, slack=`0.5095`. The field normal changed only about 5.6 degrees over 6.38--6.40 s, so the largest kick is not explained by a simple normal flip alone. Later 149/143 degree normal flips at 7.96/23.16 s while far from the nearest field point show candidate/pruning discontinuities as well.
- Comparison: the previous 10 s cylinder run peaked at `1.1537 m/s` per-cycle command jump; changing to square prisms did not remove the effect and increased the measured peak in this 30 s run.
- Source diagnosis: pruning is nearest-distance stable sorting with `seed_count=8`, separation=`0.15 m`, and no temporal hysteresis (`obstacle_hysteresis_hold_cycles=0`). The LSE-HOCBF `Lf2` contains `-(1/sigma)` times weighted `hdot` variance with `sigma=0.12`, so changes in selected face/corner points or relative-velocity estimates can change the affine row sharply. The MPC limits commanded velocity and modeled acceleration, but has no direct inter-cycle velocity-command slew constraint; `previous_input_` is only stored as an anchor. Scheduler summary reported 8 deadline overruns, maximum cycle `30.538 ms`; ROS field ages were roughly `0.08--0.20 s`, which can worsen the response but is not the primary cause.
- Decision: classify the phenomenon as a real Field-HOCBF/QP command discontinuity caused by unsmoothed field derivatives/candidate selection, amplified by the absent command-rate bound. It is not an MPC failure, and the square geometry alone is not sufficient to explain the 6.40 s kick. Selected-point identity/count and obstacle-relative velocity still need logging to separate map-corner switching from perception update noise before tuning parameters.

## 2026-10-05: Add Field-HOCBF pruning/relative-velocity diagnostics

- Objective: record the evidence needed to distinguish pruning/corner switches from relative-velocity noise without changing control behavior.
- Changes: `KinematicPoint`/field points carry the PointCloud2 snapshot index; `prune()` optionally reports candidate, eligible, and selected counts plus selected source IDs. Per-stage Field-HOCBF CSV now records those counts/IDs and mean/max selected-point relative velocity. Existing h/hdot/Lf2/profile/slack and trajectory columns are unchanged.
- Verification: `catkin_make --pkg coni_mpc -j2` passed and rebuilt `num_sim_non_one_point_node`. Existing `field_hocbf_test` still has its pre-existing pruning expectation failure (expected 2, got 10); the affine and derivative tests pass. The diagnostic change does not alter pruning selection or controller inputs.
- Next action: rerun the square-prism scenario and inspect `metrics_field_hocbf_steps.csv` for selected-ID/count changes aligned with command jumps.

## 2026-10-05: Analyze rerun with Field-HOCBF pruning diagnostics

- Experiment: the command in the user's terminal launched `square_prism_50hz` with `sim_control_dt_sec=0.02` and `sim_duration_sec=30`. The roslaunch log shows natural completion at 10:02:40; `/tmp/dsp_square_prism` contains two concatenated runs with the same tag because the output CSVs were not cleared before rerunning.
- Evidence from the latest 1500-cycle segment: all `solve_ok=1`, all `solver_fail_flag=0`, summary `collision=0`, Field-HOCBF stage-0 active on 1138/1500 cycles. The largest applied-command jump is `0.9453 m/s` over 20 ms at 7.12--7.14 s; other large jumps are `0.9104 m/s` at 6.64--6.66 s and `0.8926 m/s` at 21.68--21.70 s. At the largest jump, `d_min=0.222 m`, `h=-0.075`, slack=`0.058`, selected count changes `13->20`, and selected relative speed reaches `2.016 m/s`.
- Interpretation: the run again has no MPC failure. Large command changes occur while the Field-HOCBF is active and near/inside its barrier margin. Selected counts and source-index sets change around the jumps, supporting a discrete field/QP update mechanism; source IDs are PointCloud2 snapshot-local indices, so cross-frame ID overlap alone cannot prove physical point replacement. The latest logging is sufficient to confirm correlation but not yet to separate geometric corner switching from point-cloud ordering/update noise.
- Next action: use a fresh output directory for the next run, or remove old CSVs first; if root-cause separation is needed, log selected point positions or a frame-stable geometric key in addition to the current snapshot indices.

## 2026-10-05: Publish and verify clean GitHub tree

- Objective: publish only the current DSP -> voxel/Field-HOCBF -> MPC simulation chain and validate it from a fresh GitHub clone.
- Publication: pushed `8104286` to `https://github.com/wfaff2/GS--DSP` on `main` (also retained branch `publish/gs-dsp-clean-20261005`). The published tree keeps `DSP-map`, `coni_mpc`, MARSIM `map_generator/local_sensing/quadrotor_msgs`, the square-prism/cylinder PCD scenarios, and the vendored ACADO/qpOASES sources. It excludes build/devel/results, duplicate `障碍物感知`, LV-DOT/vision detector sources, OSQP copies, old MARSIM simulator packages, and generated binaries/logs.
- Fresh-clone verification: full `catkin_make -j2` passed in `/tmp/gs-dsp-clone-20261005-1791167858`; Field-HOCBF tests passed 3/3 and DSP FOV-grid tests passed 7/7 after correcting stale pruning assertions to the current explicit seed/separation semantics.
- Runtime verification: the clone ran `dsp_mpc_sim.launch` for 10 s at 50 Hz with `run_tag=clone_50hz`. It produced 500 MPC rows and 10,500 Field-HOCBF stage rows; all `solve_ok=1`, all `solver_fail_flag=0`, collision=0, fail_rate=0, and stage-0 Field-HOCBF active on 291/500 cycles. The controller process exited cleanly; MARSIM renderer cleanup emitted a Boost mutex exception only after manual Ctrl-C cleanup.
- Local state: `main` is fast-forwarded to the published commit; `backup/pre-publish-20261005` preserves the pre-publication tip. Build products and local excluded directories remain on disk but are ignored and are not in Git.
