Downloaded mesh assets for RViz visualization.

Files:
- `hummingbird.dae`
  - Source: `https://github.com/ethz-asl/rotors_simulator`
  - Upstream path: `rotors_description/meshes/hummingbird.dae`
- `husky_base_link.dae`
  - Source: `https://github.com/husky/husky`
  - Upstream path: `husky_description/meshes/base_link.dae`
- `jackal-base.stl`
  - Source: `https://github.com/jackal/jackal`
  - Upstream path: `jackal_description/meshes/jackal-base.stl`
- `jackal-fender.stl`
  - Source: `https://github.com/jackal/jackal`
  - Upstream path: `jackal_description/meshes/jackal-fender.stl`
- `jackal-wheel.stl`
  - Source: `https://github.com/jackal/jackal`
  - Upstream path: `jackal_description/meshes/jackal-wheel.stl`

Notes:
- `hummingbird.dae` and `husky_base_link.dae` are standalone `.dae` meshes.
- The Jackal body uses three `.stl` parts combined in code: chassis, two fenders, and four wheels.
- A quick scan found no external texture image references inside the `.dae` files.
