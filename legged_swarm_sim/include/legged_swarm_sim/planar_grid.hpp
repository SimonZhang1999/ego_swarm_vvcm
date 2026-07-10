#ifndef LEGGED_SWARM_SIM__PLANAR_GRID_HPP_
#define LEGGED_SWARM_SIM__PLANAR_GRID_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Eigen>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace legged_swarm_sim
{
class PlanarGrid
{
public:
  PlanarGrid() = default;

  PlanarGrid(double size_x, double size_y, double resolution)
  {
    resetGeometry(size_x, size_y, resolution);
  }

  void resetGeometry(double size_x, double size_y, double resolution)
  {
    size_x_ = size_x;
    size_y_ = size_y;
    resolution_ = resolution;
    width_ = static_cast<int>(std::ceil(size_x_ / resolution_));
    height_ = static_cast<int>(std::ceil(size_y_ / resolution_));
    origin_x_ = -0.5 * size_x_;
    origin_y_ = -0.5 * size_y_;
    occupied_.assign(width_ * height_, 0);
    distance_.assign(width_ * height_, std::numeric_limits<double>::infinity());
    obstacle_height_.assign(
        width_ * height_, -std::numeric_limits<double>::infinity());
  }

  bool setCloud(
      const sensor_msgs::msg::PointCloud2 &cloud, double minimum_z, double maximum_z)
  {
    if (width_ <= 0 || height_ <= 0 || resolution_ <= 0.0)
      return false;

    std::fill(occupied_.begin(), occupied_.end(), 0);
    std::fill(
        obstacle_height_.begin(), obstacle_height_.end(),
        -std::numeric_limits<double>::infinity());
    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
      {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) ||
            !std::isfinite(*iter_z))
          continue;
        int cell_x = 0;
        int cell_y = 0;
        if (!worldToCell(*iter_x, *iter_y, cell_x, cell_y))
          continue;
        obstacle_height_[index(cell_x, cell_y)] =
            std::max(
                obstacle_height_[index(cell_x, cell_y)],
                static_cast<double>(*iter_z));
        if (*iter_z >= minimum_z && *iter_z <= maximum_z)
          occupied_[index(cell_x, cell_y)] = 1;
      }
    }
    catch (const std::runtime_error &)
    {
      return false;
    }

    for (int x = 0; x < width_; ++x)
    {
      occupied_[index(x, 0)] = 1;
      occupied_[index(x, height_ - 1)] = 1;
    }
    for (int y = 0; y < height_; ++y)
    {
      occupied_[index(0, y)] = 1;
      occupied_[index(width_ - 1, y)] = 1;
    }
    buildDistanceField();
    return true;
  }

  bool worldToCell(double x, double y, int &cell_x, int &cell_y) const
  {
    cell_x = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    cell_y = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    return inside(cell_x, cell_y);
  }

  Eigen::Vector2d cellToWorld(int cell_x, int cell_y) const
  {
    return Eigen::Vector2d(
        origin_x_ + (static_cast<double>(cell_x) + 0.5) * resolution_,
        origin_y_ + (static_cast<double>(cell_y) + 0.5) * resolution_);
  }

  bool isFree(const Eigen::Vector2d &position, double clearance) const
  {
    Eigen::Vector2d gradient;
    return distanceAndGradient(position, gradient) >= clearance;
  }

  double distanceAndGradient(
      const Eigen::Vector2d &position, Eigen::Vector2d &gradient) const
  {
    if (distance_.empty())
    {
      gradient.setZero();
      return 0.0;
    }

    const double grid_x = (position.x() - origin_x_) / resolution_ - 0.5;
    const double grid_y = (position.y() - origin_y_) / resolution_ - 0.5;
    if (grid_x < 0.0 || grid_y < 0.0 ||
        grid_x > static_cast<double>(width_ - 1) ||
        grid_y > static_cast<double>(height_ - 1))
    {
      const Eigen::Vector2d clamped(
          std::clamp(position.x(), origin_x_, origin_x_ + size_x_),
          std::clamp(position.y(), origin_y_, origin_y_ + size_y_));
      const Eigen::Vector2d inward = clamped - position;
      gradient = inward.norm() > 1e-9 ? inward.normalized() : Eigen::Vector2d::Zero();
      return 0.0;
    }

    const int x0 = std::clamp(static_cast<int>(std::floor(grid_x)), 0, width_ - 1);
    const int y0 = std::clamp(static_cast<int>(std::floor(grid_y)), 0, height_ - 1);
    const int x1 = std::min(x0 + 1, width_ - 1);
    const int y1 = std::min(y0 + 1, height_ - 1);
    const double alpha_x = std::clamp(grid_x - static_cast<double>(x0), 0.0, 1.0);
    const double alpha_y = std::clamp(grid_y - static_cast<double>(y0), 0.0, 1.0);

    const double d00 = distance_[index(x0, y0)];
    const double d10 = distance_[index(x1, y0)];
    const double d01 = distance_[index(x0, y1)];
    const double d11 = distance_[index(x1, y1)];
    const double lower = (1.0 - alpha_x) * d00 + alpha_x * d10;
    const double upper = (1.0 - alpha_x) * d01 + alpha_x * d11;
    const double value = (1.0 - alpha_y) * lower + alpha_y * upper;

    gradient.x() =
        ((1.0 - alpha_y) * (d10 - d00) + alpha_y * (d11 - d01)) / resolution_;
    gradient.y() =
        ((1.0 - alpha_x) * (d01 - d00) + alpha_x * (d11 - d10)) / resolution_;
    return value;
  }

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  double minX() const { return origin_x_; }
  double maxX() const { return origin_x_ + width_ * resolution_; }
  double minY() const { return origin_y_; }
  double maxY() const { return origin_y_ + height_ * resolution_; }

  double maximumObstacleHeight(
      const Eigen::Vector2d &position, double radius) const
  {
    if (obstacle_height_.empty())
      return -std::numeric_limits<double>::infinity();
    int center_x = 0;
    int center_y = 0;
    if (!worldToCell(position.x(), position.y(), center_x, center_y))
      return std::numeric_limits<double>::infinity();

    const int radius_cells =
        std::max(0, static_cast<int>(std::ceil(radius / resolution_)));
    const double inclusion_radius =
        std::max(0.0, radius) + std::sqrt(0.5) * resolution_;
    double maximum_height = -std::numeric_limits<double>::infinity();
    for (int dy = -radius_cells - 1; dy <= radius_cells + 1; ++dy)
    {
      for (int dx = -radius_cells - 1; dx <= radius_cells + 1; ++dx)
      {
        const int cell_x = center_x + dx;
        const int cell_y = center_y + dy;
        if (!inside(cell_x, cell_y))
          continue;
        if ((cellToWorld(cell_x, cell_y) - position).norm() > inclusion_radius)
          continue;
        maximum_height =
            std::max(maximum_height, obstacle_height_[index(cell_x, cell_y)]);
      }
    }
    return maximum_height;
  }

private:
  struct DistanceNode
  {
    double distance;
    int cell_index;
  };

  struct DistanceNodeGreater
  {
    bool operator()(const DistanceNode &lhs, const DistanceNode &rhs) const
    {
      return lhs.distance > rhs.distance;
    }
  };

  bool inside(int cell_x, int cell_y) const
  {
    return cell_x >= 0 && cell_y >= 0 && cell_x < width_ && cell_y < height_;
  }

  int index(int cell_x, int cell_y) const
  {
    return cell_y * width_ + cell_x;
  }

  std::pair<int, int> fromIndex(int cell_index) const
  {
    return {cell_index % width_, cell_index / width_};
  }

  void buildDistanceField()
  {
    const double infinity = std::numeric_limits<double>::infinity();
    std::fill(distance_.begin(), distance_.end(), infinity);
    std::priority_queue<DistanceNode, std::vector<DistanceNode>, DistanceNodeGreater> queue;
    for (int cell_index = 0; cell_index < static_cast<int>(occupied_.size()); ++cell_index)
    {
      if (occupied_[cell_index] != 0)
      {
        distance_[cell_index] = 0.0;
        queue.push({0.0, cell_index});
      }
    }

    constexpr std::array<std::array<int, 2>, 8> directions = {
        std::array<int, 2>{1, 0}, std::array<int, 2>{-1, 0},
        std::array<int, 2>{0, 1}, std::array<int, 2>{0, -1},
        std::array<int, 2>{1, 1}, std::array<int, 2>{1, -1},
        std::array<int, 2>{-1, 1}, std::array<int, 2>{-1, -1}};
    while (!queue.empty())
    {
      const DistanceNode current = queue.top();
      queue.pop();
      if (current.distance > distance_[current.cell_index] + 1e-9)
        continue;
      const auto [cell_x, cell_y] = fromIndex(current.cell_index);
      for (const auto &direction : directions)
      {
        const int next_x = cell_x + direction[0];
        const int next_y = cell_y + direction[1];
        if (!inside(next_x, next_y))
          continue;
        const double step = resolution_ *
                            (direction[0] != 0 && direction[1] != 0
                                 ? std::sqrt(2.0)
                                 : 1.0);
        const int next_index = index(next_x, next_y);
        const double candidate = current.distance + step;
        if (candidate + 1e-9 < distance_[next_index])
        {
          distance_[next_index] = candidate;
          queue.push({candidate, next_index});
        }
      }
    }
  }

  double size_x_{0.0};
  double size_y_{0.0};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  int width_{0};
  int height_{0};
  std::vector<uint8_t> occupied_;
  std::vector<double> distance_;
  std::vector<double> obstacle_height_;
};
}  // namespace legged_swarm_sim

#endif  // LEGGED_SWARM_SIM__PLANAR_GRID_HPP_
