# vo-polytope algorithm provenance

The C++ implementation in
`src/num_sim/polytopic_vo_hrvo_filter.cpp` ports the core algorithm from the
authors' official open-source repository:

- Repository: https://github.com/HybridRobotics/vo-polytope
- Source file: `vo_polytope/util/reciprocal_vel_obs_polygon.py`
- Referenced master commit: `c20aca0e59b8c11acd6864d5154301d8fcbb9b6f`
- Paper: J. Huang et al., "Velocity Obstacle for Polytopic Collision Avoidance
  for Distributed Multi-Robot Systems," IEEE RA-L, 2023.
- License: MIT (notice reproduced in `LICENSE`).

Preserved behavior:

- the two cone rays are obtained from extrema over all vertex-pair bearings;
- the upstream close-overlap branch is gated by the source-code
  `distance < r + mr + 0.1` test before replacing the cone rays by the
  center-line perpendicular rays;
- static obstacles use ordinary polytopic VO;
- moving peers use polytopic HRVO, while a stopped peer is treated as an
  ordinary VO as in the source code;
- candidates are sampled in a configurable velocity window on a Cartesian
  lattice (published/source default resolution: 0.05 m/s);
- the closest safe candidate to the preferred velocity is selected;
- when no safe sampled candidate exists, the Equation (8) inverse-TTC plus
  preferred-velocity penalty is used.

Necessary experiment-interface adaptations:

- NumPy/Python code was ported to deterministic C++/Eigen for the ROS control
  loop;
- the preferred velocity is produced by the independent moving-goal interface:
  the current formation slot is refreshed at the common control period and is
  optionally combined with the slot-velocity feedforward term;
- first-order velocity-lag compensation is applied after the nominal preferred
  velocity is accepted by VO/HRVO, so acceleration feedforward does not alter
  the original collision-cone construction;
- circular UAV and cylindrical-obstacle footprints are conservatively
  approximated by circumscribed regular polygons;
- the candidate domain uses the same Euclidean UAV planar-speed limit as the
  MF-MPSC controller, instead of a component-wise square bound;
- the exact preferred velocity is always tested before the source velocity
  window lattice, implementing the continuous pass-through implied by the
  paper's Equation (7);
- zero-relative-speed and degenerate-cone divisions are guarded explicitly.

Experiment-only values such as the 2.0 s neighborhood horizon, polygon side
count, safety inflation, velocity window, lattice resolution, and fallback
weight remain ROS parameters and are logged by the simulation.
