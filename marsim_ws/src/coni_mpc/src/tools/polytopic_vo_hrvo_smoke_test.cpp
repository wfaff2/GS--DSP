#include "num_sim/polytopic_vo_hrvo_filter.h"

#include <Eigen/Core>

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

constexpr std::size_t kPolygonSides = 16U;

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void requireNear(double actual,
                 double expected,
                 double tolerance,
                 const std::string& label) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(label + ": actual=" + std::to_string(actual) +
                             " expected=" + std::to_string(expected));
  }
}

num_sim::PolytopicVoHrvoInput commonInput(
    const Eigen::Vector2d& position,
    const Eigen::Vector2d& current_velocity,
    const Eigen::Vector2d& preferred_velocity) {
  num_sim::PolytopicVoHrvoInput input;
  input.position = position;
  input.current_velocity = current_velocity;
  input.preferred_velocity = preferred_velocity;
  input.max_speed = 1.0;
  input.velocity_window = 0.5;
  input.candidate_resolution = 0.05;
  input.fallback_penalty_weight = 4.0;
  input.self_enclosing_radius =
      0.30 / std::cos(3.14159265358979323846 /
                      static_cast<double>(kPolygonSides));
  input.self_vertices = num_sim::makeCircumscribedRegularPolygon(
      position, 0.30, kPolygonSides);
  return input;
}

num_sim::PolytopicBody2d body(const Eigen::Vector2d& position,
                              const Eigen::Vector2d& velocity,
                              double radius = 0.30) {
  num_sim::PolytopicBody2d output;
  output.position = position;
  output.velocity = velocity;
  output.enclosing_radius =
      radius / std::cos(3.14159265358979323846 /
                        static_cast<double>(kPolygonSides));
  output.vertices = num_sim::makeCircumscribedRegularPolygon(
      position, radius, kPolygonSides);
  return output;
}

void testPolygonAndHorizonParameters() {
  const auto polygon = num_sim::makeCircumscribedRegularPolygon(
      Eigen::Vector2d::Zero(), 0.30, kPolygonSides);
  require(polygon.size() == kPolygonSides,
          "regular polygon has the wrong vertex count");
  double minimum_edge_distance = 1.0e9;
  for (std::size_t index = 0U; index < polygon.size(); ++index) {
    const Eigen::Vector2d start = polygon[index];
    const Eigen::Vector2d end = polygon[(index + 1U) % polygon.size()];
    const Eigen::Vector2d edge = end - start;
    const double distance = std::abs(start.x() * end.y() - start.y() * end.x()) /
                            edge.norm();
    minimum_edge_distance = std::min(minimum_edge_distance, distance);
  }
  requireNear(minimum_edge_distance, 0.30, 1.0e-12,
              "circumscribed polygon inradius");
  requireNear(num_sim::polytopicVoNeighborDistance(1.5, 0.0, 2.0),
              3.0, 1.0e-12, "static VO neighbor range");
  requireNear(num_sim::polytopicVoNeighborDistance(1.5, 1.5, 2.0),
              6.0, 1.0e-12, "peer HRVO neighbor range");
  std::cout << "PASS 1/7 polygon geometry and 2.0 s neighbor horizon\n";
}

void testNoConflictPassThrough() {
  num_sim::PolytopicVoHrvoFilter filter;
  const auto input = commonInput(Eigen::Vector2d(2.0, -1.0),
                                 Eigen::Vector2d(0.2, 0.1),
                                 Eigen::Vector2d(0.6, -0.2));
  const auto result = filter.filter(input);
  require(result.status ==
              num_sim::PolytopicVoHrvoStatus::kPreferredVelocitySafe,
          "no-conflict case did not preserve preferred velocity");
  require((result.velocity - input.preferred_velocity).norm() <= 1.0e-12,
          "no-conflict command changed");
  require(result.safe_candidate_count > 0U,
          "no-conflict case reported no safe candidates");
  std::cout << "PASS 2/7 no-conflict preferred-velocity pass-through\n";
}

void testSafePreferredOutsideVelocityWindowPassThrough() {
  num_sim::PolytopicVoHrvoFilter filter;
  const auto input = commonInput(Eigen::Vector2d::Zero(),
                                 Eigen::Vector2d::Zero(),
                                 Eigen::Vector2d(1.0, 0.0));
  require(!((std::abs(input.preferred_velocity.x() -
                      input.current_velocity.x()) <=
             input.velocity_window) &&
            (std::abs(input.preferred_velocity.y() -
                      input.current_velocity.y()) <=
             input.velocity_window)),
          "test setup did not place preferred velocity outside the lattice");
  const auto result = filter.filter(input);
  require(result.status ==
              num_sim::PolytopicVoHrvoStatus::kPreferredVelocitySafe,
          "safe preferred velocity outside local lattice was not preserved");
  require((result.velocity - input.preferred_velocity).norm() <= 1.0e-12,
          "safe outside-window preferred velocity changed");
  std::cout << "PASS 3/7 safe outside-window preferred velocity pass-through\n";
}

void testSafePeerPreferredOutsideVelocityWindowPassThrough() {
  num_sim::PolytopicVoHrvoFilter filter;
  auto input = commonInput(Eigen::Vector2d(0.235904, -2.020328),
                           Eigen::Vector2d(4.038315, -0.028191),
                           Eigen::Vector2d(3.918460, 0.716591));
  input.max_speed = 5.0;
  input.self_enclosing_radius =
      0.15 / std::cos(3.14159265358979323846 /
                      static_cast<double>(kPolygonSides));
  input.self_vertices = num_sim::makeCircumscribedRegularPolygon(
      input.position, 0.15, kPolygonSides);
  input.peers.push_back(body(Eigen::Vector2d(-0.013892, -0.013022),
                             Eigen::Vector2d(0.002039, -0.049415),
                             0.15));
  input.peers.push_back(body(Eigen::Vector2d(1.138034, -0.881356),
                             Eigen::Vector2d(1.973211, 2.044492),
                             0.15));
  require(!((std::abs(input.preferred_velocity.x() -
                      input.current_velocity.x()) <=
             input.velocity_window) &&
            (std::abs(input.preferred_velocity.y() -
                      input.current_velocity.y()) <=
             input.velocity_window)),
          "peer test setup did not place preferred velocity outside the lattice");
  require(filter.isVelocitySafe(input, input.preferred_velocity),
          "peer test preferred velocity should be HRVO-safe");
  const auto result = filter.filter(input);
  require(result.status ==
              num_sim::PolytopicVoHrvoStatus::kPreferredVelocitySafe,
          "safe peer-aware preferred velocity outside local lattice was not preserved");
  require((result.velocity - input.preferred_velocity).norm() <= 1.0e-12,
          "safe peer-aware outside-window preferred velocity changed");
  std::cout << "PASS 4/7 safe peer-aware outside-window preferred velocity pass-through\n";
}

void testStaticPolytopicVoIntervention() {
  num_sim::PolytopicVoHrvoFilter filter;
  auto input = commonInput(Eigen::Vector2d::Zero(),
                           Eigen::Vector2d(0.7, 0.0),
                           Eigen::Vector2d(1.0, 0.0));
  input.static_obstacles.push_back(
      body(Eigen::Vector2d(1.5, 0.0), Eigen::Vector2d::Zero()));
  const auto result = filter.filter(input);
  require(result.status ==
              num_sim::PolytopicVoHrvoStatus::kAdjustedVelocity,
          "static VO_p case did not choose an adjusted safe velocity");
  require(filter.isVelocitySafe(input, result.velocity),
          "static VO_p selected a velocity inside the cone");
  require(std::abs(result.velocity.y()) > 1.0e-4 ||
              result.velocity.x() < input.preferred_velocity.x() - 1.0e-4,
          "static VO_p did not alter the conflicting command");
  std::cout << "PASS 5/7 static convex-polygon VO_p intervention: v=("
            << result.velocity.x() << ", " << result.velocity.y() << ")\n";
}

void testReciprocalHeadOnHrvo() {
  num_sim::PolytopicVoHrvoFilter filter;
  auto input_a = commonInput(Eigen::Vector2d(-0.75, 0.0),
                             Eigen::Vector2d(0.8, 0.0),
                             Eigen::Vector2d(1.0, 0.0));
  auto input_b = commonInput(Eigen::Vector2d(0.75, 0.0),
                             Eigen::Vector2d(-0.8, 0.0),
                             Eigen::Vector2d(-1.0, 0.0));
  input_a.peers.push_back(body(input_b.position, input_b.current_velocity));
  input_b.peers.push_back(body(input_a.position, input_a.current_velocity));
  const auto result_a = filter.filter(input_a);
  const auto result_b = filter.filter(input_b);
  require(result_a.selectedVelocityIsSafe() &&
              result_b.selectedVelocityIsSafe(),
          "head-on HRVO_p did not produce safe commands for both peers");
  require(result_a.status ==
              num_sim::PolytopicVoHrvoStatus::kAdjustedVelocity &&
              result_b.status ==
                  num_sim::PolytopicVoHrvoStatus::kAdjustedVelocity,
          "head-on HRVO_p did not intervene for both peers");
  require(filter.isVelocitySafe(input_a, result_a.velocity) &&
              filter.isVelocitySafe(input_b, result_b.velocity),
          "head-on HRVO_p output is inside a hybrid reciprocal cone");
  require(result_a.velocity.y() * result_b.velocity.y() < 0.0,
          "head-on HRVO_p peers did not choose reciprocal passing sides");
  require((result_a.velocity + result_b.velocity).norm() <= 1.0e-9,
          "head-on HRVO_p commands are not mirror symmetric");
  requireNear(result_a.velocity.x(), 0.85, 1.0e-12,
              "official-code reference HRVO A vx");
  requireNear(result_a.velocity.y(), 0.40, 1.0e-12,
              "official-code reference HRVO A vy");
  requireNear(result_b.velocity.x(), -0.85, 1.0e-12,
              "official-code reference HRVO B vx");
  requireNear(result_b.velocity.y(), -0.40, 1.0e-12,
              "official-code reference HRVO B vy");
  std::cout << "PASS 6/7 reciprocal head-on HRVO_p: vA=("
            << result_a.velocity.x() << ", " << result_a.velocity.y()
            << "), vB=(" << result_b.velocity.x() << ", "
            << result_b.velocity.y() << ")\n";
}

void testPublishedPenalizedFallback() {
  num_sim::PolytopicVoHrvoFilter filter;
  auto input = commonInput(Eigen::Vector2d::Zero(),
                           Eigen::Vector2d(1.0, 0.0),
                           Eigen::Vector2d(1.0, 0.0));
  input.velocity_window = 0.0;
  input.static_obstacles.push_back(
      body(Eigen::Vector2d(1.5, 0.0), Eigen::Vector2d::Zero()));
  const auto result = filter.filter(input);
  require(result.status ==
              num_sim::PolytopicVoHrvoStatus::kPenalizedFallback,
          "crowded case did not use the Equation (8) fallback");
  require(result.hasCommand() && !result.selectedVelocityIsSafe(),
          "fallback command status is inconsistent");
  require(result.velocity.allFinite(), "fallback command is not finite");
  std::cout << "PASS 7/7 TTC-penalized no-safe-candidate fallback\n";
}

}  // namespace

int main() {
  std::cout << std::fixed << std::setprecision(9);
  try {
    testPolygonAndHorizonParameters();
    testNoConflictPassThrough();
    testSafePreferredOutsideVelocityWindowPassThrough();
    testSafePeerPreferredOutsideVelocityWindowPassThrough();
    testStaticPolytopicVoIntervention();
    testReciprocalHeadOnHrvo();
    testPublishedPenalizedFallback();
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << "\n";
    return EXIT_FAILURE;
  }
  std::cout << "All seven polytopic VO/HRVO correctness tests passed.\n";
  return EXIT_SUCCESS;
}
