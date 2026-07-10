#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <vector>

namespace legged_swarm_sim
{
inline bool geometricObjectPosition(
    const std::vector<Eigen::Vector2d> &robot_positions,
    const std::vector<Eigen::Vector2d> &nominal_formation_vertices,
    double holding_height, double nominal_sag,
    Eigen::Vector3d &object_position)
{
  if (robot_positions.empty() ||
      robot_positions.size() != nominal_formation_vertices.size() ||
      !std::isfinite(holding_height) || nominal_sag <= 0.0)
    return false;

  Eigen::Vector2d robot_center = Eigen::Vector2d::Zero();
  Eigen::Vector2d nominal_center = Eigen::Vector2d::Zero();
  for (size_t robot_id = 0; robot_id < robot_positions.size(); ++robot_id)
  {
    robot_center += robot_positions[robot_id];
    nominal_center += nominal_formation_vertices[robot_id];
  }
  robot_center /= static_cast<double>(robot_positions.size());
  nominal_center /= static_cast<double>(nominal_formation_vertices.size());

  double vertical_drop = 0.0;
  for (size_t robot_id = 0; robot_id < robot_positions.size(); ++robot_id)
  {
    const double nominal_radius =
        (nominal_formation_vertices[robot_id] - nominal_center).norm();
    const double current_radius =
        (robot_positions[robot_id] - robot_center).norm();
    const double effective_cable_squared =
        nominal_sag * nominal_sag + nominal_radius * nominal_radius;
    vertical_drop += std::sqrt(
        std::max(0.0, effective_cable_squared -
                          current_radius * current_radius));
  }
  vertical_drop /= static_cast<double>(robot_positions.size());

  object_position =
      Eigen::Vector3d(robot_center.x(), robot_center.y(),
                      holding_height - vertical_drop);
  return object_position.allFinite();
}
}  // namespace legged_swarm_sim
