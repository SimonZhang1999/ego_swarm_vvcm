#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace legged_swarm_sim
{
constexpr double kPi = 3.14159265358979323846;

inline void validateRobotCount(int robot_count)
{
  if (robot_count < 3 || robot_count > 5)
    throw std::runtime_error("robot_count must be between 3 and 5");
}

inline std::vector<int> boundaryOrder(int robot_count)
{
  validateRobotCount(robot_count);
  if (robot_count == 4)
    return {0, 1, 3, 2};

  std::vector<int> order(robot_count);
  for (int robot_id = 0; robot_id < robot_count; ++robot_id)
    order[robot_id] = robot_id;
  return order;
}

inline std::vector<Eigen::Vector2d> formationVerticesByRobot(
    int robot_count, double length, double width)
{
  validateRobotCount(robot_count);
  if (length <= 0.0 || width <= 0.0)
    throw std::runtime_error("formation dimensions must be positive");

  if (robot_count == 4)
  {
    const double half_length = 0.5 * length;
    const double half_width = 0.5 * width;
    return {
        {-half_length, -half_width},
        {half_length, -half_width},
        {-half_length, half_width},
        {half_length, half_width}};
  }

  std::vector<Eigen::Vector2d> vertices;
  vertices.reserve(robot_count);
  for (int robot_id = 0; robot_id < robot_count; ++robot_id)
  {
    const double angle =
        -0.5 * kPi + 2.0 * kPi * static_cast<double>(robot_id) /
                           static_cast<double>(robot_count);
    vertices.emplace_back(
        0.5 * length * std::cos(angle),
        0.5 * width * std::sin(angle));
  }
  return vertices;
}

inline std::vector<std::pair<int, int>> allPairs(int robot_count)
{
  validateRobotCount(robot_count);
  std::vector<std::pair<int, int>> pairs;
  pairs.reserve(robot_count * (robot_count - 1) / 2);
  for (int id_a = 0; id_a < robot_count; ++id_a)
  {
    for (int id_b = id_a + 1; id_b < robot_count; ++id_b)
      pairs.emplace_back(id_a, id_b);
  }
  return pairs;
}

inline size_t pairIndex(int robot_count, int id_a, int id_b)
{
  validateRobotCount(robot_count);
  if (id_a == id_b || id_a < 0 || id_b < 0 ||
      id_a >= robot_count || id_b >= robot_count)
    throw std::runtime_error("invalid robot pair");
  if (id_a > id_b)
    std::swap(id_a, id_b);
  return static_cast<size_t>(
      id_a * (2 * robot_count - id_a - 1) / 2 + id_b - id_a - 1);
}

inline std::vector<std::pair<int, int>> boundaryPairs(int robot_count)
{
  const auto order = boundaryOrder(robot_count);
  std::vector<std::pair<int, int>> pairs;
  pairs.reserve(order.size());
  for (size_t index = 0; index < order.size(); ++index)
    pairs.emplace_back(order[index], order[(index + 1) % order.size()]);
  return pairs;
}

inline std::vector<std::array<int, 3>> topologyTriples(int robot_count)
{
  const auto order = boundaryOrder(robot_count);
  std::vector<std::array<int, 3>> triples;
  triples.reserve(order.size() * (order.size() - 2));
  for (size_t edge = 0; edge < order.size(); ++edge)
  {
    const int id_a = order[edge];
    const int id_b = order[(edge + 1) % order.size()];
    for (const int id_c : order)
    {
      if (id_c != id_a && id_c != id_b)
        triples.push_back({id_a, id_b, id_c});
    }
  }
  return triples;
}
}  // namespace legged_swarm_sim
