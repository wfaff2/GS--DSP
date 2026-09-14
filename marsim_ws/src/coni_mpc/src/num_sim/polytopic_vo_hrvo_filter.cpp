#include "num_sim/polytopic_vo_hrvo_filter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

// Core algorithm ported and adapted from:
//   https://github.com/HybridRobotics/vo-polytope
//   master commit c20aca0e59b8c11acd6864d5154301d8fcbb9b6f
// The upstream project is MIT licensed; its notice is retained under
// third_party/vo_polytope/LICENSE. Adaptations are documented in the adjacent
// README.md.

namespace num_sim {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kGeometryEps = 1.0e-10;
constexpr double kVelocityMergeEps = 1.0e-9;
constexpr double kOfficialOverlapCheckPadding = 0.1;

struct VelocityObstacle {
  Eigen::Vector2d apex = Eigen::Vector2d::Zero();
  Eigen::Vector2d left = Eigen::Vector2d::UnitX();
  Eigen::Vector2d right = Eigen::Vector2d::UnitX();
};

struct Candidate {
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  std::size_t insertion_order = 0U;
};

struct VelocityObstacleBuildStats {
  std::size_t official_overlap_gate_count = 0U;
  std::size_t official_overlap_collision_count = 0U;
  std::size_t official_overlap_cone_changed_count = 0U;
};

double cross2d(const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) {
  return lhs.x() * rhs.y() - lhs.y() * rhs.x();
}

double wrapToPi(double angle) {
  // Match the author's implementation exactly at the branch boundary: +pi
  // remains +pi (it is not remapped to -pi). This matters for the mirrored
  // HRVO passing-side decision in perfectly symmetric head-on cases.
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

Eigen::Vector2d direction(double angle) {
  return Eigen::Vector2d(std::cos(angle), std::sin(angle));
}

Eigen::Vector2d clampSpeed(const Eigen::Vector2d& velocity,
                           double max_speed) {
  const double norm = velocity.norm();
  if (norm > max_speed && norm > kGeometryEps) {
    return velocity * (max_speed / norm);
  }
  return velocity;
}

bool finiteVertices(const std::vector<Eigen::Vector2d>& vertices) {
  if (vertices.size() < 3U) {
    return false;
  }
  for (const auto& vertex : vertices) {
    if (!vertex.allFinite()) {
      return false;
    }
  }
  return true;
}

bool finiteBody(const PolytopicBody2d& body) {
  return body.position.allFinite() && body.velocity.allFinite() &&
         finiteVertices(body.vertices) &&
         std::isfinite(body.enclosing_radius) &&
         body.enclosing_radius >= 0.0;
}

bool finiteInput(const PolytopicVoHrvoInput& input) {
  if (!input.position.allFinite() ||
      !input.current_velocity.allFinite() ||
      !input.preferred_velocity.allFinite() ||
      !finiteVertices(input.self_vertices) ||
      !std::isfinite(input.self_enclosing_radius) ||
      input.self_enclosing_radius < 0.0 ||
      !std::isfinite(input.max_speed) || input.max_speed < 0.0 ||
      !std::isfinite(input.velocity_window) || input.velocity_window < 0.0 ||
      !std::isfinite(input.candidate_resolution) ||
      input.candidate_resolution <= 0.0 ||
      !std::isfinite(input.fallback_penalty_weight) ||
      input.fallback_penalty_weight < 0.0) {
    return false;
  }
  for (const auto& obstacle : input.static_obstacles) {
    if (!finiteBody(obstacle)) {
      return false;
    }
  }
  for (const auto& peer : input.peers) {
    if (!finiteBody(peer)) {
      return false;
    }
  }
  return true;
}

std::pair<double, double> projectPolygon(
    const std::vector<Eigen::Vector2d>& vertices,
    const Eigen::Vector2d& axis) {
  double lower = vertices.front().dot(axis);
  double upper = lower;
  for (std::size_t index = 1U; index < vertices.size(); ++index) {
    const double projection = vertices[index].dot(axis);
    lower = std::min(lower, projection);
    upper = std::max(upper, projection);
  }
  return {lower, upper};
}

bool separatedOnAnyAxis(const std::vector<Eigen::Vector2d>& lhs,
                        const std::vector<Eigen::Vector2d>& rhs,
                        const std::vector<Eigen::Vector2d>& axes_from) {
  for (std::size_t index = 0U; index < axes_from.size(); ++index) {
    const Eigen::Vector2d edge =
        axes_from[(index + 1U) % axes_from.size()] - axes_from[index];
    Eigen::Vector2d axis(-edge.y(), edge.x());
    const double norm = axis.norm();
    if (norm <= kGeometryEps) {
      continue;
    }
    axis /= norm;
    const auto lhs_projection = projectPolygon(lhs, axis);
    const auto rhs_projection = projectPolygon(rhs, axis);
    if (lhs_projection.second < rhs_projection.first - kGeometryEps ||
        rhs_projection.second < lhs_projection.first - kGeometryEps) {
      return true;
    }
  }
  return false;
}

bool polygonsOverlap(const std::vector<Eigen::Vector2d>& lhs,
                     const std::vector<Eigen::Vector2d>& rhs) {
  return !separatedOnAnyAxis(lhs, rhs, lhs) &&
         !separatedOnAnyAxis(lhs, rhs, rhs);
}

bool angleChanged(double lhs, double rhs) {
  return std::abs(wrapToPi(lhs - rhs)) > 10.0 * kGeometryEps;
}

VelocityObstacle buildVelocityObstacleFromAngles(
    const PolytopicVoHrvoInput& input,
    const PolytopicBody2d& body,
    bool reciprocal_peer,
    double left_angle,
    double right_angle) {
  VelocityObstacle vo;
  vo.left = direction(left_angle);
  vo.right = direction(right_angle);

  // The upstream implementation treats a stopped neighboring robot as a
  // non-reciprocating ordinary VO, just like a static obstacle.
  if (!reciprocal_peer || body.velocity.norm() <= kGeometryEps) {
    vo.apex = body.velocity;
    return vo;
  }

  const Eigen::Vector2d rvo_apex =
      0.5 * (input.current_velocity + body.velocity);
  const Eigen::Vector2d vo_apex = body.velocity;
  const Eigen::Vector2d current_from_rvo =
      input.current_velocity - rvo_apex;

  double edge_angle = 0.0;
  double center_line = 0.0;
  if (right_angle - left_angle > kPi) {
    edge_angle = 2.0 * kPi - right_angle + left_angle;
    center_line = wrapToPi(right_angle + 0.5 * edge_angle);
  } else {
    edge_angle = left_angle - right_angle;
    center_line = left_angle - 0.5 * edge_angle;
  }

  const Eigen::Vector2d center_line_vector = direction(center_line);
  const double apex_distance = (rvo_apex - vo_apex).norm();
  const double apex_angle = std::atan2(
      rvo_apex.y() - vo_apex.y(), rvo_apex.x() - vo_apex.x());
  const double numerator = apex_distance * std::sin(left_angle - apex_angle);
  const double denominator = std::sin(kPi - edge_angle);
  if (std::abs(denominator) <= kGeometryEps) {
    vo.apex = rvo_apex;
    return vo;
  }
  const double distance_difference = numerator / denominator;
  if (cross2d(center_line_vector, current_from_rvo) <= 0.0) {
    vo.apex = rvo_apex - distance_difference * vo.right;
  } else {
    vo.apex = vo_apex + distance_difference * vo.right;
  }
  if (!vo.apex.allFinite()) {
    vo.apex = rvo_apex;
  }
  return vo;
}

VelocityObstacle buildPolytopicVelocityObstacle(
    const PolytopicVoHrvoInput& input,
    const PolytopicBody2d& body,
    bool reciprocal_peer,
    bool enable_official_overlap_branch,
    VelocityObstacleBuildStats* stats) {
  double left_angle = -kPi;
  double right_angle = kPi;
  for (const auto& self_vertex : input.self_vertices) {
    for (const auto& other_vertex : body.vertices) {
      const Eigen::Vector2d delta = other_vertex - self_vertex;
      const double angle = std::atan2(delta.y(), delta.x());
      left_angle = std::max(left_angle, angle);
      right_angle = std::min(right_angle, angle);
    }
  }

  // Case (b) in Huang et al.: the cone straddles the -pi/pi branch cut.
  if (left_angle - right_angle > kPi) {
    left_angle = -kPi;
    right_angle = kPi;
    for (const auto& self_vertex : input.self_vertices) {
      for (const auto& other_vertex : body.vertices) {
        const Eigen::Vector2d delta = other_vertex - self_vertex;
        const double angle = std::atan2(delta.y(), delta.x());
        if (left_angle < angle && angle < 0.0) {
          left_angle = angle;
        } else if (0.0 < angle && angle < right_angle) {
          right_angle = angle;
        }
      }
    }
  }

  const double base_left_angle = left_angle;
  const double base_right_angle = right_angle;
  const double center_distance = (body.position - input.position).norm();
  const double official_overlap_check_range =
      input.self_enclosing_radius + body.enclosing_radius +
      kOfficialOverlapCheckPadding;
  if (center_distance < official_overlap_check_range) {
    if (stats != nullptr) {
      ++stats->official_overlap_gate_count;
    }
    if (polygonsOverlap(input.self_vertices, body.vertices)) {
      if (stats != nullptr) {
        ++stats->official_overlap_collision_count;
      }
      if (enable_official_overlap_branch) {
        const Eigen::Vector2d center_delta = body.position - input.position;
        const double center_angle =
            std::atan2(center_delta.y(), center_delta.x());
        left_angle = wrapToPi(center_angle + 0.5 * kPi);
        right_angle = wrapToPi(center_angle - 0.5 * kPi);
      }
    }
  }

  const VelocityObstacle vo = buildVelocityObstacleFromAngles(
      input, body, reciprocal_peer, left_angle, right_angle);
  if (stats != nullptr && enable_official_overlap_branch &&
      (angleChanged(left_angle, base_left_angle) ||
       angleChanged(right_angle, base_right_angle))) {
    const VelocityObstacle base_vo = buildVelocityObstacleFromAngles(
        input, body, reciprocal_peer, base_left_angle, base_right_angle);
    if (angleChanged(left_angle, base_left_angle) ||
        angleChanged(right_angle, base_right_angle) ||
        (vo.apex - base_vo.apex).norm() > 10.0 * kGeometryEps) {
      ++stats->official_overlap_cone_changed_count;
    }
  }
  return vo;
}

std::vector<VelocityObstacle> buildVelocityObstacles(
    const PolytopicVoHrvoInput& input,
    bool enable_official_overlap_branch,
    VelocityObstacleBuildStats* stats = nullptr) {
  std::vector<VelocityObstacle> obstacles;
  obstacles.reserve(input.static_obstacles.size() + input.peers.size());
  for (const auto& obstacle : input.static_obstacles) {
    obstacles.push_back(
        buildPolytopicVelocityObstacle(
            input, obstacle, false, enable_official_overlap_branch, stats));
  }
  for (const auto& peer : input.peers) {
    obstacles.push_back(buildPolytopicVelocityObstacle(
        input, peer, true, enable_official_overlap_branch, stats));
  }
  return obstacles;
}

bool insideVelocityObstacle(const VelocityObstacle& obstacle,
                            const Eigen::Vector2d& velocity) {
  const Eigen::Vector2d line = velocity - obstacle.apex;
  return cross2d(obstacle.left, line) < -kGeometryEps &&
         cross2d(obstacle.right, line) > kGeometryEps;
}

bool safeAgainstAll(const std::vector<VelocityObstacle>& obstacles,
                    const Eigen::Vector2d& velocity,
                    double max_speed) {
  if (!velocity.allFinite() ||
      velocity.norm() > max_speed + 10.0 * kGeometryEps) {
    return false;
  }
  for (const auto& obstacle : obstacles) {
    if (insideVelocityObstacle(obstacle, velocity)) {
      return false;
    }
  }
  return true;
}

void addCandidate(std::vector<Candidate>& candidates,
                  const Eigen::Vector2d& velocity,
                  double max_speed) {
  if (!velocity.allFinite() ||
      velocity.norm() > max_speed + 10.0 * kGeometryEps) {
    return;
  }
  for (const auto& existing : candidates) {
    if ((existing.velocity - velocity).norm() <= kVelocityMergeEps) {
      return;
    }
  }
  candidates.push_back(Candidate{velocity, candidates.size()});
}

std::vector<Candidate> generateCandidates(
    const PolytopicVoHrvoInput& input) {
  std::vector<Candidate> candidates;
  const Eigen::Vector2d bounded_preferred =
      clampSpeed(input.preferred_velocity, input.max_speed);
  // Equation (7) is continuous.  The exact preferred velocity must always be a
  // candidate so a safe moving-goal feedforward command is not rejected merely
  // because it lies outside the local replacement lattice.
  addCandidate(candidates, bounded_preferred, input.max_speed);

  const double lower_x = std::max(-input.max_speed,
                                  input.current_velocity.x() -
                                      input.velocity_window);
  const double upper_x = std::min(input.max_speed,
                                  input.current_velocity.x() +
                                      input.velocity_window);
  const double lower_y = std::max(-input.max_speed,
                                  input.current_velocity.y() -
                                      input.velocity_window);
  const double upper_y = std::min(input.max_speed,
                                  input.current_velocity.y() +
                                      input.velocity_window);

  const std::size_t count_x = static_cast<std::size_t>(std::floor(
      std::max(0.0, upper_x - lower_x) / input.candidate_resolution)) + 1U;
  const std::size_t count_y = static_cast<std::size_t>(std::floor(
      std::max(0.0, upper_y - lower_y) / input.candidate_resolution)) + 1U;
  candidates.reserve(1U + count_x * count_y);
  for (std::size_t ix = 0U; ix < count_x; ++ix) {
    const double vx = lower_x +
                      static_cast<double>(ix) * input.candidate_resolution;
    for (std::size_t iy = 0U; iy < count_y; ++iy) {
      const double vy = lower_y +
                        static_cast<double>(iy) * input.candidate_resolution;
      addCandidate(candidates, Eigen::Vector2d(vx, vy), input.max_speed);
    }
  }
  addCandidate(candidates,
               clampSpeed(input.current_velocity, input.max_speed),
               input.max_speed);
  if (candidates.empty()) {
    addCandidate(candidates, Eigen::Vector2d::Zero(), input.max_speed);
  }
  return candidates;
}

double expectedCollisionTime(const Eigen::Vector2d& relative_position,
                             const Eigen::Vector2d& relative_velocity,
                             double combined_radius) {
  const double a = relative_velocity.squaredNorm();
  const double b = 2.0 * relative_position.dot(relative_velocity);
  const double c = relative_position.squaredNorm() -
                   combined_radius * combined_radius;
  if (c <= 0.0) {
    return 0.0;
  }
  if (a <= kGeometryEps) {
    return std::numeric_limits<double>::infinity();
  }
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double root = std::sqrt(discriminant);
  const double first = (-b - root) / (2.0 * a);
  const double second = (-b + root) / (2.0 * a);
  double collision_time = std::numeric_limits<double>::infinity();
  if (first >= 0.0) {
    collision_time = std::min(collision_time, first);
  }
  if (second >= 0.0) {
    collision_time = std::min(collision_time, second);
  }
  return collision_time;
}

double minimumExpectedCollisionTime(const PolytopicVoHrvoInput& input,
                                    const Eigen::Vector2d& candidate) {
  double minimum = std::numeric_limits<double>::infinity();
  for (const auto& peer : input.peers) {
    const Eigen::Vector2d relative_position = input.position - peer.position;
    // This is the reciprocal collision-time expression used by the author's
    // HRVO implementation for the fallback in Equation (8).
    const Eigen::Vector2d relative_velocity =
        2.0 * candidate - peer.velocity - input.current_velocity;
    minimum = std::min(
        minimum,
        expectedCollisionTime(relative_position,
                              relative_velocity,
                              input.self_enclosing_radius +
                                  peer.enclosing_radius));
  }
  for (const auto& obstacle : input.static_obstacles) {
    minimum = std::min(
        minimum,
        expectedCollisionTime(input.position - obstacle.position,
                              candidate - obstacle.velocity,
                              input.self_enclosing_radius +
                                  obstacle.enclosing_radius));
  }
  return minimum;
}

double fallbackCost(const PolytopicVoHrvoInput& input,
                    const Eigen::Vector2d& candidate,
                    double minimum_ttc) {
  double inverse_ttc = 0.0;
  if (minimum_ttc <= kGeometryEps) {
    inverse_ttc = std::numeric_limits<double>::infinity();
  } else if (std::isfinite(minimum_ttc)) {
    inverse_ttc = 1.0 / minimum_ttc;
  }
  const double collision_penalty =
      input.fallback_penalty_weight <= kGeometryEps
          ? 0.0
          : input.fallback_penalty_weight * inverse_ttc;
  return collision_penalty +
         (candidate - input.preferred_velocity).norm();
}

PolytopicVoHrvoResult filterPolytopicVoHrvo(
    const PolytopicVoHrvoInput& input,
    bool enable_official_overlap_branch,
    bool collect_overlap_stats) {
  PolytopicVoHrvoResult result;
  result.velocity_obstacle_count =
      input.static_obstacles.size() + input.peers.size();
  if (!finiteInput(input)) {
    return result;
  }

  VelocityObstacleBuildStats build_stats;
  VelocityObstacleBuildStats* stats =
      collect_overlap_stats ? &build_stats : nullptr;
  const std::vector<VelocityObstacle> velocity_obstacles =
      buildVelocityObstacles(input, enable_official_overlap_branch, stats);
  if (stats != nullptr) {
    result.official_overlap_gate_count =
        build_stats.official_overlap_gate_count;
    result.official_overlap_collision_count =
        build_stats.official_overlap_collision_count;
    result.official_overlap_cone_changed_count =
        build_stats.official_overlap_cone_changed_count;
  }
  std::vector<Candidate> candidates = generateCandidates(input);
  result.candidate_count = candidates.size();

  const Candidate* best_safe = nullptr;
  double best_safe_cost = std::numeric_limits<double>::infinity();
  for (const auto& candidate : candidates) {
    if (!safeAgainstAll(velocity_obstacles,
                        candidate.velocity,
                        input.max_speed)) {
      continue;
    }
    ++result.safe_candidate_count;
    const double cost =
        (candidate.velocity - input.preferred_velocity).squaredNorm();
    if (best_safe == nullptr || cost < best_safe_cost - kGeometryEps ||
        (std::abs(cost - best_safe_cost) <= kGeometryEps &&
         candidate.insertion_order < best_safe->insertion_order)) {
      best_safe = &candidate;
      best_safe_cost = cost;
    }
  }

  if (best_safe != nullptr) {
    result.velocity = best_safe->velocity;
    result.status =
        (result.velocity - input.preferred_velocity).norm() <=
                10.0 * kGeometryEps
            ? PolytopicVoHrvoStatus::kPreferredVelocitySafe
            : PolytopicVoHrvoStatus::kAdjustedVelocity;
    return result;
  }

  const Candidate* best_fallback = nullptr;
  double best_fallback_cost = std::numeric_limits<double>::infinity();
  double best_fallback_ttc = std::numeric_limits<double>::infinity();
  for (const auto& candidate : candidates) {
    const double minimum_ttc =
        minimumExpectedCollisionTime(input, candidate.velocity);
    const double cost = fallbackCost(input, candidate.velocity, minimum_ttc);
    const bool strictly_better = cost < best_fallback_cost - kGeometryEps;
    const bool both_infinite = std::isinf(cost) &&
                               std::isinf(best_fallback_cost);
    if (best_fallback == nullptr || strictly_better ||
        ((both_infinite || std::abs(cost - best_fallback_cost) <=
                               kGeometryEps) &&
         candidate.insertion_order < best_fallback->insertion_order)) {
      best_fallback = &candidate;
      best_fallback_cost = cost;
      best_fallback_ttc = minimum_ttc;
    }
  }
  if (best_fallback != nullptr) {
    result.velocity = best_fallback->velocity;
    result.fallback_min_ttc = best_fallback_ttc;
    result.status = PolytopicVoHrvoStatus::kPenalizedFallback;
  }
  return result;
}

}  // namespace

PolytopicVoHrvoResult PolytopicVoHrvoFilter::filter(
    const PolytopicVoHrvoInput& input) const {
  PolytopicVoHrvoResult result =
      filterPolytopicVoHrvo(input, true, true);
  if (result.official_overlap_collision_count > 0U && result.hasCommand()) {
    const PolytopicVoHrvoResult branch_disabled =
        filterPolytopicVoHrvo(input, false, false);
    if (branch_disabled.hasCommand()) {
      result.official_overlap_output_delta_norm =
          (result.velocity - branch_disabled.velocity).norm();
      if (result.official_overlap_output_delta_norm >
          10.0 * kVelocityMergeEps) {
        result.official_overlap_output_changed_count = 1U;
      }
    }
  }
  return result;
}

bool PolytopicVoHrvoFilter::isVelocitySafe(
    const PolytopicVoHrvoInput& input,
    const Eigen::Vector2d& velocity) const {
  if (!finiteInput(input)) {
    return false;
  }
  return safeAgainstAll(buildVelocityObstacles(input, true),
                        velocity,
                        input.max_speed);
}

std::vector<Eigen::Vector2d> makeCircumscribedRegularPolygon(
    const Eigen::Vector2d& center,
    double circle_radius,
    std::size_t side_count,
    double phase_rad) {
  std::vector<Eigen::Vector2d> vertices;
  if (!center.allFinite() || !std::isfinite(circle_radius) ||
      circle_radius < 0.0 || side_count < 3U ||
      !std::isfinite(phase_rad)) {
    return vertices;
  }
  const double circumradius =
      circle_radius / std::cos(kPi / static_cast<double>(side_count));
  vertices.reserve(side_count);
  for (std::size_t index = 0U; index < side_count; ++index) {
    const double angle = phase_rad +
                         2.0 * kPi * static_cast<double>(index) /
                             static_cast<double>(side_count);
    vertices.push_back(center + circumradius * direction(angle));
  }
  return vertices;
}

double polytopicVoNeighborDistance(double controlled_max_speed,
                                  double other_max_speed,
                                  double time_horizon) {
  if (!std::isfinite(controlled_max_speed) ||
      !std::isfinite(other_max_speed) || !std::isfinite(time_horizon) ||
      controlled_max_speed < 0.0 || other_max_speed < 0.0 ||
      time_horizon < 0.0) {
    return 0.0;
  }
  return (controlled_max_speed + other_max_speed) * time_horizon;
}

}  // namespace num_sim
