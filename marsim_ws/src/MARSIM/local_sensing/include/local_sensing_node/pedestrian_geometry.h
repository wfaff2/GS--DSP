#pragma once

#include <array>

namespace local_sensing_node {
// Upright adult, fixed pose, walking along +Y. Coordinates are relative to
// the bounding-box center. The same ellipsoids drive RViz and LiDAR surfaces.
struct PedestrianPart {
  double x, y, z, sx, sy, sz;
};

inline std::array<PedestrianPart, 6> pedestrianParts(double height) {
  const double s = height / 1.8;
  std::array<PedestrianPart, 6> parts{{
      {0.0, 0.0, 0.75, 0.30, 0.28, 0.30}, // head: top = 1.8 m
      {0.0, 0.0, 0.27, 0.42, 0.28, 0.76}, // torso
      {-0.265, 0.0, 0.20, 0.14, 0.16, 0.72},
      { 0.265, 0.0, 0.20, 0.14, 0.16, 0.72},
      {-0.105, 0.0, -0.48, 0.18, 0.22, 0.84},
      { 0.105, 0.0, -0.48, 0.18, 0.22, 0.84}}};
  for (auto& part : parts) {
    part.x *= s; part.y *= s; part.z *= s;
    part.sx *= s; part.sy *= s; part.sz *= s;
  }
  return parts;
}
} // namespace local_sensing_node
