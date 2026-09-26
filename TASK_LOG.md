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
