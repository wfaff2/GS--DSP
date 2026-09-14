#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <limits>
#include <vector>

namespace num_sim {

// World-frame convex polygon used by the polytopic velocity-obstacle filter.
// vertices must be ordered around the boundary. enclosing_radius is the
// maximum center-to-vertex distance and is used only by the published
// time-to-collision fallback cost.
struct PolytopicBody2d {
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  std::vector<Eigen::Vector2d> vertices;
  double enclosing_radius = 0.0;
};

struct PolytopicVoHrvoInput {
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d current_velocity = Eigen::Vector2d::Zero();
  Eigen::Vector2d preferred_velocity = Eigen::Vector2d::Zero();
  std::vector<Eigen::Vector2d> self_vertices;
  double self_enclosing_radius = 0.0;

  // The author's open-source implementation searches a Cartesian velocity
  // lattice in [v_current - velocity_window, v_current + velocity_window].
  double max_speed = 0.0;
  double velocity_window = 0.5;
  double candidate_resolution = 0.05;
  double fallback_penalty_weight = 4.0;

  std::vector<PolytopicBody2d> static_obstacles;
  std::vector<PolytopicBody2d> peers;
};

enum class PolytopicVoHrvoStatus {
  kPreferredVelocitySafe,
  kAdjustedVelocity,
  kPenalizedFallback,
  kInvalidInput,
};

struct PolytopicVoHrvoResult {
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  PolytopicVoHrvoStatus status = PolytopicVoHrvoStatus::kInvalidInput;
  std::size_t candidate_count = 0U;
  std::size_t safe_candidate_count = 0U;
  std::size_t velocity_obstacle_count = 0U;
  std::size_t official_overlap_gate_count = 0U;
  std::size_t official_overlap_collision_count = 0U;
  std::size_t official_overlap_cone_changed_count = 0U;
  std::size_t official_overlap_output_changed_count = 0U;
  double fallback_min_ttc = std::numeric_limits<double>::infinity();
  double official_overlap_output_delta_norm = 0.0;

  bool hasCommand() const {
    return status != PolytopicVoHrvoStatus::kInvalidInput;
  }

  bool selectedVelocityIsSafe() const {
    return status == PolytopicVoHrvoStatus::kPreferredVelocitySafe ||
           status == PolytopicVoHrvoStatus::kAdjustedVelocity;
  }

  bool usedFallback() const {
    return status == PolytopicVoHrvoStatus::kPenalizedFallback;
  }
};

// A C++ port of the core geometry and velocity-selection pipeline in
// HybridRobotics/vo-polytope (RA-L 2023). Static polygons generate VO_p;
// moving peer polygons generate HRVO_p. If no sampled velocity lies outside
// the combined velocity obstacle, the paper's TTC-penalized fallback is used.
class PolytopicVoHrvoFilter {
 public:
  PolytopicVoHrvoResult filter(const PolytopicVoHrvoInput& input) const;

  bool isVelocitySafe(const PolytopicVoHrvoInput& input,
                      const Eigen::Vector2d& velocity) const;
};

// Conservative regular-polygon approximation of a circular horizontal
// footprint. The polygon circumscribes (contains) the circle with the supplied
// radius, so a requested physical clearance is not under-approximated.
std::vector<Eigen::Vector2d> makeCircumscribedRegularPolygon(
    const Eigen::Vector2d& center,
    double circle_radius,
    std::size_t side_count,
    double phase_rad = 0.0);

// Huang et al. Equation (4): l = (v_R,max + v_O,max) * tau.
double polytopicVoNeighborDistance(double controlled_max_speed,
                                  double other_max_speed,
                                  double time_horizon);

}  // namespace num_sim
