#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <legged_swarm_sim/formation_geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
double yawFromPose(const geometry_msgs::msg::Pose &pose)
{
  const double siny_cosp = 2.0 * (pose.orientation.w * pose.orientation.z +
                                  pose.orientation.x * pose.orientation.y);
  const double cosy_cosp = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
                                        pose.orientation.z * pose.orientation.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::vector<std::pair<double, double>> formationOffsets(int robot_count, double spacing,
                                                        double length, double width,
                                                        const std::string &mode)
{
  if (mode == "auto" || mode == "polygon")
  {
    std::vector<std::pair<double, double>> offsets;
    for (const auto &vertex :
         legged_swarm_sim::formationVerticesByRobot(
             robot_count, length, width))
      offsets.emplace_back(vertex.x(), vertex.y());
    return offsets;
  }

  std::vector<std::pair<double, double>> offsets;
  offsets.reserve(robot_count);

  if (mode == "rectangle" && robot_count == 4)
  {
    const double hx = 0.5 * length;
    const double hy = 0.5 * width;
    offsets = {{-hx, -hy}, {hx, -hy}, {-hx, hy}, {hx, hy}};
    return offsets;
  }

  if (mode == "square" && robot_count == 4)
  {
    const double h = 0.5 * spacing;
    offsets = {{-h, -h}, {h, -h}, {-h, h}, {h, h}};
    return offsets;
  }

  if (mode == "diamond" && robot_count == 4)
  {
    offsets = {{spacing, 0.0}, {0.0, spacing}, {0.0, -spacing}, {-spacing, 0.0}};
    return offsets;
  }

  for (int i = 0; i < robot_count; ++i)
  {
    offsets.emplace_back(0.0, (static_cast<double>(i) - 0.5 * (robot_count - 1)) * spacing);
  }
  return offsets;
}
}  // namespace

class FormationGoalRelay : public rclcpp::Node
{
public:
  FormationGoalRelay() : Node("formation_goal_relay")
  {
    robot_count_ = declare_parameter<int>("robot_count", 4);
    spacing_ = declare_parameter<double>("formation_spacing", 2.2);
    formation_length_ = declare_parameter<double>("formation_length", 2.4);
    formation_width_ = declare_parameter<double>("formation_width", 2.0);
    formation_mode_ = declare_parameter<std::string>("formation_mode", "auto");
    rotate_with_goal_yaw_ = declare_parameter<bool>("rotate_with_goal_yaw", true);
    goal_z_ = declare_parameter<double>("goal_z", 0.35);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    map_size_x_ = declare_parameter<double>("map_size_x", 24.0);
    map_size_y_ = declare_parameter<double>("map_size_y", 16.0);
    goal_boundary_margin_ =
        declare_parameter<double>("goal_boundary_margin", 0.9);
    legged_swarm_sim::validateRobotCount(robot_count_);
    if (map_size_x_ <= 0.0 || map_size_y_ <= 0.0 ||
        goal_boundary_margin_ < 0.0)
      throw std::runtime_error("invalid formation goal map boundary parameters");
    const bool auto_start = declare_parameter<bool>("auto_start", false);
    const double start_delay = declare_parameter<double>("auto_start_delay", 2.0);
    initial_goal_.pose.position.x = declare_parameter<double>("goal_x", 8.0);
    initial_goal_.pose.position.y = declare_parameter<double>("goal_y", 0.0);
    initial_goal_.pose.position.z = goal_z_;
    initial_goal_.pose.orientation.w = 1.0;
    initial_goal_.header.frame_id = frame_id_;

    for (int i = 0; i < robot_count_; ++i)
    {
      goal_pubs_.push_back(create_publisher<geometry_msgs::msg::PoseStamped>(
          "/robot_" + std::to_string(i) + "/goal", 1));
    }
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/swarm/formation_goal_markers", 1);
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 5,
        std::bind(&FormationGoalRelay::goalCallback, this, std::placeholders::_1));

    if (auto_start)
    {
      auto_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(start_delay)),
          [this]() {
            publishFormation(initial_goal_);
            auto_timer_->cancel();
          });
    }
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    publishFormation(*msg);
  }

  void publishFormation(geometry_msgs::msg::PoseStamped center)
  {
    if (center.header.frame_id.empty())
      center.header.frame_id = frame_id_;
    if (center.pose.position.z < 0.05)
      center.pose.position.z = goal_z_;

    visualization_msgs::msg::MarkerArray markers;
    const auto offsets = formationOffsets(robot_count_, spacing_, formation_length_, formation_width_, formation_mode_);
    const double yaw = rotate_with_goal_yaw_ ? yawFromPose(center.pose) : 0.0;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    std::vector<std::pair<double, double>> rotated_offsets;
    rotated_offsets.reserve(offsets.size());
    double min_offset_x = std::numeric_limits<double>::infinity();
    double max_offset_x = -std::numeric_limits<double>::infinity();
    double min_offset_y = std::numeric_limits<double>::infinity();
    double max_offset_y = -std::numeric_limits<double>::infinity();
    for (const auto &offset : offsets)
    {
      const double rotated_x = c * offset.first - s * offset.second;
      const double rotated_y = s * offset.first + c * offset.second;
      rotated_offsets.emplace_back(rotated_x, rotated_y);
      min_offset_x = std::min(min_offset_x, rotated_x);
      max_offset_x = std::max(max_offset_x, rotated_x);
      min_offset_y = std::min(min_offset_y, rotated_y);
      max_offset_y = std::max(max_offset_y, rotated_y);
    }
    if (rotated_offsets.empty())
    {
      RCLCPP_ERROR(get_logger(), "cannot publish an empty formation");
      return;
    }

    const double center_min_x =
        -0.5 * map_size_x_ + goal_boundary_margin_ - min_offset_x;
    const double center_max_x =
        0.5 * map_size_x_ - goal_boundary_margin_ - max_offset_x;
    const double center_min_y =
        -0.5 * map_size_y_ + goal_boundary_margin_ - min_offset_y;
    const double center_max_y =
        0.5 * map_size_y_ - goal_boundary_margin_ - max_offset_y;
    if (center_min_x > center_max_x || center_min_y > center_max_y)
    {
      RCLCPP_ERROR(
          get_logger(),
          "formation does not fit map %.2f x %.2f with boundary margin %.2f",
          map_size_x_, map_size_y_, goal_boundary_margin_);
      return;
    }
    if (!std::isfinite(center.pose.position.x) ||
        !std::isfinite(center.pose.position.y))
    {
      RCLCPP_ERROR(get_logger(), "formation center goal is not finite");
      return;
    }

    const double requested_x = center.pose.position.x;
    const double requested_y = center.pose.position.y;
    center.pose.position.x =
        std::clamp(requested_x, center_min_x, center_max_x);
    center.pose.position.y =
        std::clamp(requested_y, center_min_y, center_max_y);
    if (std::abs(center.pose.position.x - requested_x) > 1e-9 ||
        std::abs(center.pose.position.y - requested_y) > 1e-9)
    {
      RCLCPP_WARN(
          get_logger(),
          "formation center goal clamped from (%.2f, %.2f) to (%.2f, %.2f); "
          "safe center bounds x=[%.2f, %.2f] y=[%.2f, %.2f]",
          requested_x, requested_y,
          center.pose.position.x, center.pose.position.y,
          center_min_x, center_max_x, center_min_y, center_max_y);
    }

    const auto common_goal_stamp = now();

    for (int i = 0; i < robot_count_; ++i)
    {
      geometry_msgs::msg::PoseStamped goal = center;
      // All independent frontends use this stamp as the centralized batch ID.
      goal.header.stamp = common_goal_stamp;
      goal.pose.position.x += rotated_offsets[i].first;
      goal.pose.position.y += rotated_offsets[i].second;
      goal_pubs_[i]->publish(goal);

      visualization_msgs::msg::Marker marker;
      marker.header = goal.header;
      marker.ns = "formation_goal";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = goal.pose;
      marker.scale.x = 0.35;
      marker.scale.y = 0.35;
      marker.scale.z = 0.35;
      marker.color.r = 0.1f;
      marker.color.g = 0.9f;
      marker.color.b = 0.3f;
      marker.color.a = 0.9f;
      markers.markers.push_back(marker);
    }
    marker_pub_->publish(markers);
    RCLCPP_INFO(get_logger(), "published formation goal around (%.2f, %.2f, %.2f)",
                center.pose.position.x, center.pose.position.y, center.pose.position.z);
  }

  int robot_count_{4};
  double spacing_{2.2};
  double formation_length_{2.4};
  double formation_width_{2.0};
  std::string formation_mode_{"rectangle"};
  bool rotate_with_goal_yaw_{true};
  double goal_z_{0.35};
  std::string frame_id_{"world"};
  double map_size_x_{24.0};
  double map_size_y_{16.0};
  double goal_boundary_margin_{0.9};
  geometry_msgs::msg::PoseStamped initial_goal_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> goal_pubs_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr auto_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FormationGoalRelay>());
  rclcpp::shutdown();
  return 0;
}
