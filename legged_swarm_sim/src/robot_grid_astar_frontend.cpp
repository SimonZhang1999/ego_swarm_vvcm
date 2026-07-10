#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <legged_swarm_sim/planar_grid.hpp>

#include <Eigen/Eigen>

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

class RobotGridAstarFrontend : public rclcpp::Node
{
public:
  RobotGridAstarFrontend() : Node("robot_grid_astar_frontend")
  {
    robot_id_ = declare_parameter<int>("robot_id", 0);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    fixed_z_ = declare_parameter<double>("fixed_z", 0.35);
    clearance_ = declare_parameter<double>("robot_clearance", 0.58);
    start_recovery_clearance_ =
        declare_parameter<double>("start_recovery_clearance", 0.50);
    start_recovery_radius_ =
        declare_parameter<double>("start_recovery_radius", 1.2);
    map_size_x_ = declare_parameter<double>("map_size_x", 24.0);
    map_size_y_ = declare_parameter<double>("map_size_y", 16.0);
    map_resolution_ = declare_parameter<double>("map_resolution", 0.2);
    collision_min_z_ = declare_parameter<double>("collision_min_z", -0.05);
    collision_max_z_ = declare_parameter<double>("collision_max_z", 0.85);
    max_search_nodes_ = declare_parameter<int>("max_search_nodes", 250000);
    grid_.resetGeometry(map_size_x_, map_size_y_, map_resolution_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/swarm/fused_cloud", rclcpp::QoS(2),
        std::bind(&RobotGridAstarFrontend::cloudCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom_world", 20,
        std::bind(&RobotGridAstarFrontend::odomCallback, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal", 10,
        std::bind(&RobotGridAstarFrontend::goalCallback, this, std::placeholders::_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("frontend_path", 10);
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RobotGridAstarFrontend::tryPlan, this));

    RCLCPP_INFO(
        get_logger(), "robot_%d independent A* frontend ready", robot_id_);
  }

private:
  struct OpenNode
  {
    double total_cost;
    int index;
  };

  struct OpenNodeGreater
  {
    bool operator()(const OpenNode &lhs, const OpenNode &rhs) const
    {
      return lhs.total_cost > rhs.total_cost;
    }
  };

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "fused cloud frame '%s' does not match '%s'",
          msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    if (!grid_.setCloud(*msg, collision_min_z_, collision_max_z_))
    {
      RCLCPP_ERROR(get_logger(), "failed to build frontend occupancy grid");
      return;
    }
    have_map_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_position_ =
        Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
    have_odom_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_)
    {
      RCLCPP_ERROR(
          get_logger(), "goal frame '%s' does not match '%s'",
          msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    pending_goal_ = *msg;
    have_pending_goal_ = true;
    tryPlan();
  }

  void tryPlan()
  {
    if (!have_pending_goal_ || planning_)
      return;
    if (!have_map_ || !have_odom_)
    {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "robot_%d waits for fused map and odometry", robot_id_);
      return;
    }

    planning_ = true;
    const Eigen::Vector2d goal(
        pending_goal_.pose.position.x, pending_goal_.pose.position.y);
    std::vector<Eigen::Vector2d> path;
    size_t expanded_nodes = 0;
    if (!search(current_position_, goal, path, expanded_nodes))
    {
      RCLCPP_ERROR(
          get_logger(),
          "FRONTEND_PLAN_FAILED robot=%d start=(%.2f,%.2f) goal=(%.2f,%.2f)",
          robot_id_, current_position_.x(), current_position_.y(), goal.x(), goal.y());
      planning_ = false;
      return;
    }

    const auto simplified = simplifyPath(path);
    publishPath(simplified, pending_goal_.header.stamp);
    have_pending_goal_ = false;
    planning_ = false;
    RCLCPP_INFO(
        get_logger(),
        "FRONTEND_PLAN_SUCCESS robot=%d points=%zu expanded=%zu",
        robot_id_, simplified.size(), expanded_nodes);
  }

  double obstacleDistance(const Eigen::Vector2d &position) const
  {
    Eigen::Vector2d gradient;
    return grid_.distanceAndGradient(position, gradient);
  }

  bool segmentIsFree(
      const Eigen::Vector2d &start, const Eigen::Vector2d &end,
      double required_clearance) const
  {
    const double length = (end - start).norm();
    const int sample_count = std::max(
        1, static_cast<int>(std::ceil(length / (0.45 * map_resolution_))));
    for (int sample = 0; sample <= sample_count; ++sample)
    {
      const double alpha = static_cast<double>(sample) / sample_count;
      if (!grid_.isFree(start + alpha * (end - start), required_clearance))
        return false;
    }
    return true;
  }

  bool findRecoveryStart(
      const Eigen::Vector2d &start, const Eigen::Vector2d &goal,
      Eigen::Vector2d &recovered_start) const
  {
    int start_x = 0;
    int start_y = 0;
    if (!grid_.worldToCell(start.x(), start.y(), start_x, start_y))
      return false;

    const int radius_cells =
        static_cast<int>(std::ceil(start_recovery_radius_ / map_resolution_));
    double best_score = std::numeric_limits<double>::infinity();
    bool found = false;
    for (int dy = -radius_cells; dy <= radius_cells; ++dy)
    {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx)
      {
        const int cell_x = start_x + dx;
        const int cell_y = start_y + dy;
        if (cell_x < 0 || cell_y < 0 ||
            cell_x >= grid_.width() || cell_y >= grid_.height())
          continue;
        const Eigen::Vector2d candidate = grid_.cellToWorld(cell_x, cell_y);
        const double recovery_distance = (candidate - start).norm();
        if (recovery_distance > start_recovery_radius_ ||
            !grid_.isFree(candidate, clearance_) ||
            !segmentIsFree(start, candidate, start_recovery_clearance_))
          continue;

        const double score =
            recovery_distance + 0.05 * (candidate - goal).norm();
        if (score < best_score)
        {
          best_score = score;
          recovered_start = candidate;
          found = true;
        }
      }
    }
    return found;
  }

  bool search(
      const Eigen::Vector2d &start, const Eigen::Vector2d &goal,
      std::vector<Eigen::Vector2d> &path, size_t &expanded_nodes) const
  {
    Eigen::Vector2d search_start = start;
    int start_x = 0;
    int start_y = 0;
    int goal_x = 0;
    int goal_y = 0;
    const bool start_inside =
        grid_.worldToCell(start.x(), start.y(), start_x, start_y);
    const bool goal_inside =
        grid_.worldToCell(goal.x(), goal.y(), goal_x, goal_y);
    if (!start_inside || !goal_inside)
    {
      RCLCPP_ERROR(
          get_logger(),
          "robot_%d frontend start or goal is outside map "
          "x=[%.2f,%.2f) y=[%.2f,%.2f): "
          "start=(%.2f,%.2f,%s) goal=(%.2f,%.2f,%s)",
          robot_id_, grid_.minX(), grid_.maxX(), grid_.minY(), grid_.maxY(),
          start.x(), start.y(), start_inside ? "inside" : "outside",
          goal.x(), goal.y(), goal_inside ? "inside" : "outside");
      return false;
    }

    const double start_distance = obstacleDistance(start);
    const double goal_distance = obstacleDistance(goal);
    if (start_distance < start_recovery_clearance_)
    {
      RCLCPP_ERROR(
          get_logger(),
          "robot_%d frontend start is in physical collision: distance=%.3f required=%.3f",
          robot_id_, start_distance, start_recovery_clearance_);
      return false;
    }
    if (goal_distance < clearance_)
    {
      RCLCPP_ERROR(
          get_logger(),
          "robot_%d frontend goal has insufficient clearance: distance=%.3f required=%.3f",
          robot_id_, goal_distance, clearance_);
      return false;
    }

    bool used_recovery_start = false;
    if (start_distance < clearance_)
    {
      if (!findRecoveryStart(start, goal, search_start))
      {
        RCLCPP_ERROR(
            get_logger(),
            "robot_%d cannot recover start clearance %.3f -> %.3f within %.2f m",
            robot_id_, start_distance, clearance_, start_recovery_radius_);
        return false;
      }
      used_recovery_start = true;
      grid_.worldToCell(search_start.x(), search_start.y(), start_x, start_y);
      RCLCPP_WARN(
          get_logger(),
          "robot_%d recovers A* start from clearance %.3f at (%.2f,%.2f) "
          "to (%.2f,%.2f)",
          robot_id_, start_distance, start.x(), start.y(),
          search_start.x(), search_start.y());
    }

    const int width = grid_.width();
    const int height = grid_.height();
    const auto to_index = [width](int x, int y) { return y * width + x; };
    const auto from_index = [width](int index)
    {
      return std::pair<int, int>(index % width, index / width);
    };
    const int start_index = to_index(start_x, start_y);
    const int goal_index = to_index(goal_x, goal_y);
    const int cell_count = width * height;
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> g_cost(cell_count, infinity);
    std::vector<int> parent(cell_count, -1);
    std::vector<uint8_t> closed(cell_count, 0);
    std::priority_queue<OpenNode, std::vector<OpenNode>, OpenNodeGreater> open;
    g_cost[start_index] = 0.0;
    open.push({(goal - search_start).norm(), start_index});

    constexpr std::array<std::array<int, 2>, 8> directions = {
        std::array<int, 2>{1, 0}, std::array<int, 2>{-1, 0},
        std::array<int, 2>{0, 1}, std::array<int, 2>{0, -1},
        std::array<int, 2>{1, 1}, std::array<int, 2>{1, -1},
        std::array<int, 2>{-1, 1}, std::array<int, 2>{-1, -1}};

    bool found = false;
    expanded_nodes = 0;
    while (!open.empty() && expanded_nodes < static_cast<size_t>(max_search_nodes_))
    {
      const OpenNode current = open.top();
      open.pop();
      if (closed[current.index] != 0)
        continue;
      closed[current.index] = 1;
      ++expanded_nodes;
      if (current.index == goal_index)
      {
        found = true;
        break;
      }

      const auto [current_x, current_y] = from_index(current.index);
      const Eigen::Vector2d current_position = grid_.cellToWorld(current_x, current_y);
      for (const auto &direction : directions)
      {
        const int next_x = current_x + direction[0];
        const int next_y = current_y + direction[1];
        if (next_x < 0 || next_y < 0 || next_x >= width || next_y >= height)
          continue;
        const int next_index = to_index(next_x, next_y);
        if (closed[next_index] != 0)
          continue;
        const Eigen::Vector2d next_position = grid_.cellToWorld(next_x, next_y);
        if (!segmentIsFree(current_position, next_position, clearance_))
          continue;

        const double step_cost =
            map_resolution_ *
            (direction[0] != 0 && direction[1] != 0 ? std::sqrt(2.0) : 1.0);
        const double candidate = g_cost[current.index] + step_cost;
        if (candidate >= g_cost[next_index])
          continue;
        g_cost[next_index] = candidate;
        parent[next_index] = current.index;
        open.push({candidate + (goal - next_position).norm(), next_index});
      }
    }
    if (!found)
    {
      RCLCPP_ERROR(
          get_logger(),
          "robot_%d A* exhausted search: expanded=%zu limit=%d",
          robot_id_, expanded_nodes, max_search_nodes_);
      return false;
    }

    std::vector<Eigen::Vector2d> reverse_path;
    int current_index = goal_index;
    while (current_index >= 0)
    {
      const auto [cell_x, cell_y] = from_index(current_index);
      reverse_path.push_back(grid_.cellToWorld(cell_x, cell_y));
      if (current_index == start_index)
        break;
      current_index = parent[current_index];
    }
    if (reverse_path.empty() || current_index < 0)
      return false;
    path.assign(reverse_path.rbegin(), reverse_path.rend());
    path.front() = search_start;
    path.back() = goal;
    if (used_recovery_start)
      path.insert(path.begin(), start);
    else
      path.front() = start;
    return true;
  }

  std::vector<Eigen::Vector2d> simplifyPath(
      const std::vector<Eigen::Vector2d> &path) const
  {
    if (path.size() <= 2)
      return path;
    std::vector<Eigen::Vector2d> simplified;
    simplified.push_back(path.front());
    size_t anchor = 0;
    while (anchor + 1 < path.size())
    {
      size_t next = path.size() - 1;
      while (next > anchor + 1 &&
             !segmentIsFree(path[anchor], path[next], clearance_))
        --next;
      simplified.push_back(path[next]);
      anchor = next;
    }
    return simplified;
  }

  void publishPath(
      const std::vector<Eigen::Vector2d> &points,
      const builtin_interfaces::msg::Time &goal_stamp)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = goal_stamp;
    if (path.header.stamp.sec == 0 && path.header.stamp.nanosec == 0)
      path.header.stamp = now();
    path.poses.reserve(points.size());
    for (const auto &point : points)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = fixed_z_;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  int robot_id_{0};
  int max_search_nodes_{250000};
  std::string frame_id_{"world"};
  double fixed_z_{0.35};
  double clearance_{0.58};
  double start_recovery_clearance_{0.50};
  double start_recovery_radius_{1.2};
  double map_size_x_{24.0};
  double map_size_y_{16.0};
  double map_resolution_{0.2};
  double collision_min_z_{-0.05};
  double collision_max_z_{0.85};
  bool have_map_{false};
  bool have_odom_{false};
  bool have_pending_goal_{false};
  bool planning_{false};
  Eigen::Vector2d current_position_{Eigen::Vector2d::Zero()};
  geometry_msgs::msg::PoseStamped pending_goal_;
  legged_swarm_sim::PlanarGrid grid_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotGridAstarFrontend>());
  rclcpp::shutdown();
  return 0;
}
