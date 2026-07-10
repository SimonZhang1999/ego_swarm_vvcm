#include <bspline_opt/lbfgs.hpp>
#include <bspline_opt/uniform_bspline.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <traj_utils/msg/bspline.hpp>
#include <traj_utils/msg/multi_bsplines.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <legged_swarm_sim/deformable_sheet_model.hpp>
#include <legged_swarm_sim/formation_geometry.hpp>
#include <legged_swarm_sim/planar_grid.hpp>
#include <VVCM_FK.hpp>

#include <Eigen/Eigen>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
geometry_msgs::msg::Point toPoint(const Eigen::Vector2d &point, double z)
{
  geometry_msgs::msg::Point message;
  message.x = point.x();
  message.y = point.y();
  message.z = z;
  return message;
}

std_msgs::msg::ColorRGBA robotColor(int robot_id, float alpha = 1.0f)
{
  static const std::array<std::array<float, 3>, 5> colors = {
      std::array<float, 3>{0.10f, 0.55f, 0.95f},
      std::array<float, 3>{0.95f, 0.35f, 0.20f},
      std::array<float, 3>{0.20f, 0.70f, 0.35f},
      std::array<float, 3>{0.75f, 0.35f, 0.95f},
      std::array<float, 3>{0.95f, 0.75f, 0.10f}};
  std_msgs::msg::ColorRGBA color;
  color.r = colors[robot_id % colors.size()][0];
  color.g = colors[robot_id % colors.size()][1];
  color.b = colors[robot_id % colors.size()][2];
  color.a = alpha;
  return color;
}

Eigen::Vector2d perpendicular(const Eigen::Vector2d &vector)
{
  return Eigen::Vector2d(-vector.y(), vector.x());
}
}  // namespace

class CentralizedTrajectoryOptimizer : public rclcpp::Node
{
public:
  CentralizedTrajectoryOptimizer() : Node("centralized_trajectory_optimizer")
  {
    robot_count_ = declare_parameter<int>("robot_count", 4);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    fixed_z_ = declare_parameter<double>("fixed_z", 0.35);
    sample_count_ = declare_parameter<int>("sample_count", 48);
    spline_densification_ =
        declare_parameter<int>("spline.densification", 2);
    segment_collision_subsamples_ =
        declare_parameter<int>("spline.segment_collision_subsamples", 3);
    map_size_x_ = declare_parameter<double>("map_size_x", 24.0);
    map_size_y_ = declare_parameter<double>("map_size_y", 16.0);
    map_resolution_ = declare_parameter<double>("map_resolution", 0.2);
    collision_min_z_ = declare_parameter<double>("collision_min_z", -0.05);
    collision_max_z_ = declare_parameter<double>("collision_max_z", 0.85);
    obstacle_clearance_ = declare_parameter<double>("obstacle_clearance", 0.72);
    swarm_clearance_ = declare_parameter<double>("swarm_clearance", 0.90);
    hard_obstacle_clearance_ =
        declare_parameter<double>("hard_obstacle_clearance", 0.50);
    hard_swarm_clearance_ =
        declare_parameter<double>("hard_swarm_clearance", 0.85);
    max_velocity_ = declare_parameter<double>("max_velocity", 2.0);
    max_acceleration_ = declare_parameter<double>("max_acceleration", 4.0);
    start_time_lead_ = declare_parameter<double>("start_time_lead", 0.5);
    fitness_weight_ = declare_parameter<double>("weights.fitness", 1.0);
    smoothness_weight_ = declare_parameter<double>("weights.smoothness", 2.0);
    obstacle_weight_ = declare_parameter<double>("weights.obstacle", 80.0);
    swarm_weight_ = declare_parameter<double>("weights.swarm", 10.0);
    sheet_constraint_enabled_ = declare_parameter<bool>("sheet.enabled", true);
    sheet_length_ = declare_parameter<double>("sheet.length", 3.8);
    sheet_width_ = declare_parameter<double>("sheet.width", 3.4);
    sheet_distance_margin_ =
        declare_parameter<double>("sheet.distance_margin", 0.10);
    sheet_distance_weight_ =
        declare_parameter<double>("weights.sheet_distance", 40.0);
    sheet_lower_bound_weight_ =
        declare_parameter<double>("weights.sheet_lower_bound", 1500.0);
    topology_enabled_ = declare_parameter<bool>("topology.enabled", true);
    topology_weight_ = declare_parameter<double>("topology.weight", 30.0);
    topology_min_directed_distance_ =
        declare_parameter<double>("topology.min_directed_distance", 0.35);
    topology_hard_min_directed_distance_ =
        declare_parameter<double>("topology.hard_min_directed_distance", 0.08);
    object_clearance_enabled_ =
        declare_parameter<bool>("object_clearance.enabled", true);
    rod_base_offset_z_ =
        declare_parameter<double>("object_clearance.rod_base_offset_z", 0.175);
    rod_height_ =
        declare_parameter<double>("object_clearance.rod_height", 2.4);
    object_diameter_ =
        declare_parameter<double>("object_clearance.object_diameter", 0.45);
    object_footprint_radius_ =
        declare_parameter<double>("object_clearance.footprint_radius", 0.35);
    object_obstacle_clearance_ =
        declare_parameter<double>("object_clearance.obstacle_clearance", 0.10);
    object_distance_increment_ =
        declare_parameter<double>("object_clearance.distance_increment", 0.05);
    object_geometric_nominal_sag_ =
        declare_parameter<double>(
            "object_clearance.geometric_nominal_sag", 2.0);
    formation_length_ =
        declare_parameter<double>(
            "object_clearance.nominal_formation_length", 2.4);
    formation_width_ =
        declare_parameter<double>(
            "object_clearance.nominal_formation_width", 2.0);
    object_lower_bound_neighbor_samples_ =
        declare_parameter<int>("object_clearance.neighbor_samples", 2);
    object_clearance_max_iterations_ =
        declare_parameter<int>("object_clearance.max_iterations", 30);
    constraint_repair_max_attempts_ =
        declare_parameter<int>("constraint_repair.max_attempts", 4);
    constraint_repair_weight_multiplier_ =
        declare_parameter<double>("constraint_repair.weight_multiplier", 4.0);
    max_iterations_ = declare_parameter<int>("max_iterations", 250);

    legged_swarm_sim::validateRobotCount(robot_count_);
    if (sample_count_ < 8)
      throw std::runtime_error("sample_count must be at least 8");
    if (spline_densification_ < 1 || spline_densification_ > 10)
      throw std::runtime_error("spline.densification must be in [1, 10]");
    if (segment_collision_subsamples_ < 0 ||
        segment_collision_subsamples_ > 10)
      throw std::runtime_error(
          "spline.segment_collision_subsamples must be in [0, 10]");
    if (sheet_length_ <= 0.0 || sheet_width_ <= 0.0 ||
        sheet_distance_margin_ < 0.0 ||
        sheet_distance_margin_ >= std::min(sheet_length_, sheet_width_))
      throw std::runtime_error("invalid sheet dimensions or distance margin");
    if (sheet_distance_weight_ < 0.0 || sheet_lower_bound_weight_ < 0.0 ||
        topology_weight_ < 0.0 ||
        topology_min_directed_distance_ < topology_hard_min_directed_distance_ ||
        topology_hard_min_directed_distance_ < 0.0)
      throw std::runtime_error("invalid sheet or topology constraint parameters");
    if (rod_height_ <= 0.0 || object_diameter_ <= 0.0 ||
        object_footprint_radius_ < 0.0 || object_obstacle_clearance_ < 0.0 ||
        object_distance_increment_ <= 0.0 ||
        object_geometric_nominal_sag_ <= 0.0 ||
        formation_length_ <= 0.0 || formation_width_ <= 0.0 ||
        object_lower_bound_neighbor_samples_ < 0 ||
        object_clearance_max_iterations_ < 0 ||
        constraint_repair_max_attempts_ < 0 ||
        constraint_repair_weight_multiplier_ <= 1.0)
      throw std::runtime_error("invalid object clearance parameters");
    grid_.resetGeometry(map_size_x_, map_size_y_, map_resolution_);
    boundary_order_ = legged_swarm_sim::boundaryOrder(robot_count_);
    sheet_pairs_ = legged_swarm_sim::allPairs(robot_count_);
    sheet_boundary_pairs_ =
        legged_swarm_sim::boundaryPairs(robot_count_);
    topology_triples_ =
        legged_swarm_sim::topologyTriples(robot_count_);
    sheet_vertices_by_robot_ =
        legged_swarm_sim::formationVerticesByRobot(
            robot_count_, sheet_length_, sheet_width_);
    nominal_formation_vertices_by_robot_ =
        legged_swarm_sim::formationVerticesByRobot(
            robot_count_, formation_length_, formation_width_);
    sheet_pair_limits_.reserve(sheet_pairs_.size());
    for (const auto &pair : sheet_pairs_)
    {
      sheet_pair_limits_.push_back(
          (sheet_vertices_by_robot_[pair.second] -
           sheet_vertices_by_robot_[pair.first])
              .norm());
    }

    VVCM::MatrixXf sheet_vertices(robot_count_, 2);
    for (int row = 0; row < robot_count_; ++row)
    {
      const auto &vertex = sheet_vertices_by_robot_[boundary_order_[row]];
      sheet_vertices(row, 0) = static_cast<float>(vertex.x());
      sheet_vertices(row, 1) = static_cast<float>(vertex.y());
    }
    if (robot_count_ == 4)
    {
      vvcm_solver_ = std::make_unique<VVCM::VVCM_FK>(
          robot_count_, static_cast<float>(holdingHeight()), sheet_vertices);
    }
    distance_lower_bounds_.assign(
        sample_count_, std::vector<double>(sheet_pairs_.size(), 0.0));

    latest_paths_.resize(robot_count_);
    path_goal_stamps_.assign(robot_count_, -1);
    path_subscriptions_.reserve(robot_count_);
    trajectory_publishers_.reserve(robot_count_);
    optimized_path_publishers_.reserve(robot_count_);
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      path_subscriptions_.push_back(create_subscription<nav_msgs::msg::Path>(
          "/robot_" + std::to_string(robot_id) + "/frontend_path", 10,
          [this, robot_id](const nav_msgs::msg::Path::SharedPtr msg)
          {
            pathCallback(robot_id, *msg);
          }));
      trajectory_publishers_.push_back(create_publisher<traj_utils::msg::Bspline>(
          "/drone_" + std::to_string(robot_id) + "_planning/bspline", 10));
      optimized_path_publishers_.push_back(create_publisher<nav_msgs::msg::Path>(
          "/robot_" + std::to_string(robot_id) + "/centralized_path", 10));
    }
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/swarm/fused_cloud", rclcpp::QoS(2),
        std::bind(&CentralizedTrajectoryOptimizer::cloudCallback, this, std::placeholders::_1));
    marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/swarm/centralized_plan", 2);
    sync_pub_ =
        create_publisher<std_msgs::msg::Int64MultiArray>("/swarm/centralized_sync", 10);
    trajectory_batch_pub_ = create_publisher<traj_utils::msg::MultiBsplines>(
        "/swarm/centralized_bspline_batch",
        rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&CentralizedTrajectoryOptimizer::tryOptimize, this));

    RCLCPP_INFO(
        get_logger(),
        "centralized backend ready: robots=%d joint variables=%d "
        "pairs=%zu topology_constraints=%zu sheet=%.2fx%.2f, "
        "topology=%s object_model=%s, no similarity or area cost",
        robot_count_, 2 * robot_count_ * (sample_count_ - 2),
        sheet_pairs_.size(), topology_triples_.size(),
        sheet_length_, sheet_width_,
        topology_enabled_ ? "enabled" : "disabled",
        robot_count_ == 4 ? "VVCM" : "geometric");
  }

private:
  using Path2d = std::vector<Eigen::Vector2d>;

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
      RCLCPP_ERROR(get_logger(), "failed to build centralized optimizer distance field");
      return;
    }
    have_map_ = true;
  }

  static int64_t stampKey(const builtin_interfaces::msg::Time &stamp)
  {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL +
           static_cast<int64_t>(stamp.nanosec);
  }

  void pathCallback(int robot_id, const nav_msgs::msg::Path &path)
  {
    if (path.poses.size() < 2)
    {
      RCLCPP_ERROR(
          get_logger(), "ignore robot_%d frontend path with %zu points",
          robot_id, path.poses.size());
      return;
    }
    if (!path.header.frame_id.empty() && path.header.frame_id != frame_id_)
    {
      RCLCPP_ERROR(
          get_logger(), "robot_%d frontend path frame '%s' does not match '%s'",
          robot_id, path.header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    latest_paths_[robot_id] = path;
    path_goal_stamps_[robot_id] = stampKey(path.header.stamp);
    tryOptimize();
  }

  bool haveCompleteBatch(int64_t &goal_stamp) const
  {
    if (std::any_of(
            path_goal_stamps_.begin(), path_goal_stamps_.end(),
            [](int64_t stamp) { return stamp < 0; }))
      return false;
    goal_stamp = path_goal_stamps_.front();
    return std::all_of(
               path_goal_stamps_.begin(), path_goal_stamps_.end(),
               [goal_stamp](int64_t stamp) { return stamp == goal_stamp; }) &&
           goal_stamp != last_attempted_goal_stamp_;
  }

  Path2d resamplePath(const nav_msgs::msg::Path &path) const
  {
    Path2d source;
    source.reserve(path.poses.size());
    for (const auto &pose : path.poses)
      source.emplace_back(pose.pose.position.x, pose.pose.position.y);

    std::vector<double> cumulative(source.size(), 0.0);
    for (size_t i = 1; i < source.size(); ++i)
      cumulative[i] = cumulative[i - 1] + (source[i] - source[i - 1]).norm();
    const double total_length = cumulative.back();

    Path2d resampled;
    resampled.reserve(sample_count_);
    if (total_length < 1e-9)
    {
      resampled.assign(sample_count_, source.front());
      return resampled;
    }

    size_t source_index = 1;
    for (int sample = 0; sample < sample_count_; ++sample)
    {
      const double target_length =
          total_length * static_cast<double>(sample) / static_cast<double>(sample_count_ - 1);
      while (source_index + 1 < source.size() &&
             cumulative[source_index] < target_length)
        ++source_index;
      const size_t previous = source_index - 1;
      const double segment_length = cumulative[source_index] - cumulative[previous];
      const double alpha =
          segment_length > 1e-9
              ? (target_length - cumulative[previous]) / segment_length
              : 0.0;
      resampled.push_back(
          source[previous] + std::clamp(alpha, 0.0, 1.0) *
                                 (source[source_index] - source[previous]));
    }
    return resampled;
  }

  void prepareReferencePaths()
  {
    reference_paths_.clear();
    reference_paths_.reserve(robot_count_);
    for (const auto &path : latest_paths_)
      reference_paths_.push_back(resamplePath(path));
  }

  int variableIndex(int robot_id, int sample, int dimension) const
  {
    if (sample <= 0 || sample >= sample_count_ - 1)
      return -1;
    return 2 * (robot_id * (sample_count_ - 2) + sample - 1) + dimension;
  }

  Eigen::Vector2d pointFromVariables(
      const double *variables, int robot_id, int sample) const
  {
    if (sample == 0 || sample == sample_count_ - 1)
      return reference_paths_[robot_id][sample];
    const int index = variableIndex(robot_id, sample, 0);
    return Eigen::Vector2d(variables[index], variables[index + 1]);
  }

  void addGradient(
      double *gradient, int robot_id, int sample,
      const Eigen::Vector2d &value) const
  {
    const int index = variableIndex(robot_id, sample, 0);
    if (index < 0)
      return;
    gradient[index] += value.x();
    gradient[index + 1] += value.y();
  }

  double sheetPairLimit(int id_a, int id_b) const
  {
    return sheet_pair_limits_[
        legged_swarm_sim::pairIndex(robot_count_, id_a, id_b)];
  }

  struct DirectedDistance
  {
    double value{0.0};
    Eigen::Vector2d gradient_a{Eigen::Vector2d::Zero()};
    Eigen::Vector2d gradient_b{Eigen::Vector2d::Zero()};
    Eigen::Vector2d gradient_c{Eigen::Vector2d::Zero()};
  };

  static DirectedDistance directedDistance(
      const Eigen::Vector2d &point_a,
      const Eigen::Vector2d &point_b,
      const Eigen::Vector2d &point_c)
  {
    const Eigen::Vector2d edge = point_b - point_a;
    const Eigen::Vector2d relative = point_c - point_a;
    const double edge_length = std::max(1e-6, edge.norm());
    const double numerator = perpendicular(edge).dot(relative);
    const double inverse_length = 1.0 / edge_length;
    const double length_correction =
        numerator / (edge_length * edge_length * edge_length);

    DirectedDistance result;
    result.value = numerator * inverse_length;
    result.gradient_b =
        -perpendicular(relative) * inverse_length -
        length_correction * edge;
    result.gradient_c = perpendicular(edge) * inverse_length;
    result.gradient_a =
        (perpendicular(relative) - perpendicular(edge)) * inverse_length +
        length_correction * edge;
    return result;
  }

  size_t sheetPairIndex(int id_a, int id_b) const
  {
    return legged_swarm_sim::pairIndex(robot_count_, id_a, id_b);
  }

  double holdingHeight() const
  {
    return fixed_z_ + rod_base_offset_z_ + rod_height_;
  }

  bool computeObjectPosition(
      const std::vector<Eigen::Vector2d> &positions,
      Eigen::Vector3d &object_position) const
  {
    if (positions.size() != static_cast<size_t>(robot_count_))
      return false;
    if (robot_count_ != 4)
    {
      return legged_swarm_sim::geometricObjectPosition(
          positions, nominal_formation_vertices_by_robot_, holdingHeight(),
          object_geometric_nominal_sag_, object_position);
    }
    Eigen::Vector2d formation_center = Eigen::Vector2d::Zero();
    for (const auto &position : positions)
      formation_center += position;
    formation_center /= static_cast<double>(positions.size());
    VVCM::MatrixXf robot_positions(robot_count_, 2);
    for (size_t row = 0; row < boundary_order_.size(); ++row)
    {
      robot_positions(static_cast<int>(row), 0) =
          static_cast<float>(
              positions[boundary_order_[row]].x() - formation_center.x());
      robot_positions(static_cast<int>(row), 1) =
          static_cast<float>(
              positions[boundary_order_[row]].y() - formation_center.y());
    }
    vvcm_solver_->zr = static_cast<float>(holdingHeight());
    const auto error = vvcm_solver_->update_stable_solutions(robot_positions);
    if (error != VVCM::VVCM_FK_Error::NoError || vvcm_solver_->Po.empty())
      return false;

    const auto solution = std::min_element(
        vvcm_solver_->Po.begin(), vvcm_solver_->Po.end(),
        [](const VVCM::Vector3f &lhs, const VVCM::Vector3f &rhs)
        {
          return lhs.z() < rhs.z();
        });
    object_position = solution->cast<double>();
    object_position.x() += formation_center.x();
    object_position.y() += formation_center.y();
    return object_position.allFinite();
  }

  double objectObstacleClearance(
      const std::vector<Eigen::Vector2d> &positions,
      Eigen::Vector3d &object_position,
      double &obstacle_height) const
  {
    if (!computeObjectPosition(positions, object_position))
    {
      obstacle_height = std::numeric_limits<double>::infinity();
      return -std::numeric_limits<double>::infinity();
    }
    obstacle_height = grid_.maximumObstacleHeight(
        object_position.head<2>(), object_footprint_radius_);
    if (!std::isfinite(obstacle_height))
      return std::numeric_limits<double>::infinity();
    const double object_bottom =
        object_position.z() - 0.5 * object_diameter_;
    return object_bottom - obstacle_height - object_obstacle_clearance_;
  }

  bool raiseDistanceLowerBounds(const std::vector<int> &violation_samples)
  {
    std::vector<uint8_t> affected(sample_count_, 0);
    for (const int sample : violation_samples)
    {
      for (int offset = -object_lower_bound_neighbor_samples_;
           offset <= object_lower_bound_neighbor_samples_; ++offset)
      {
        const int neighbor = sample + offset;
        if (neighbor > 0 && neighbor < sample_count_ - 1)
          affected[neighbor] = 1;
      }
    }

    bool changed = false;
    bool can_still_adjust = false;
    for (int sample = 1; sample < sample_count_ - 1; ++sample)
    {
      if (affected[sample] == 0)
        continue;
      for (const auto &pair : sheet_boundary_pairs_)
      {
        const int id_a = pair.first;
        const int id_b = pair.second;
        const size_t pair_index = sheetPairIndex(id_a, id_b);
        const double upper =
            sheetPairLimit(id_a, id_b) - sheet_distance_margin_;
        const double distance =
            (optimized_paths_[id_b][sample] -
             optimized_paths_[id_a][sample])
                .norm();
        const double old_lower =
            distance_lower_bounds_[sample][pair_index];
        can_still_adjust =
            can_still_adjust || old_lower < upper - 1e-9;
        const double new_lower = std::min(
            upper,
            std::max(old_lower, distance + object_distance_increment_));
        distance_lower_bounds_[sample][pair_index] = new_lower;
        changed = changed || new_lower > old_lower + 1e-12;
      }
    }
    return changed || can_still_adjust;
  }

  double maximumDistanceLowerUtilization() const
  {
    double maximum_ratio = 0.0;
    const auto &pairs = sheet_pairs_;
    for (int sample = 1; sample < sample_count_ - 1; ++sample)
    {
      for (size_t pair_index = 0; pair_index < pairs.size(); ++pair_index)
      {
        const double upper =
            sheetPairLimit(
                pairs[pair_index].first, pairs[pair_index].second) -
            sheet_distance_margin_;
        maximum_ratio = std::max(
            maximum_ratio,
            distance_lower_bounds_[sample][pair_index] / upper);
      }
    }
    return maximum_ratio;
  }

  void projectVariablesToSheetBounds(std::vector<double> &variables) const
  {
    constexpr int projection_iterations = 40;
    const auto &pairs = sheet_pairs_;
    for (int sample = 1; sample < sample_count_ - 1; ++sample)
    {
      std::vector<Eigen::Vector2d> positions(robot_count_);
      for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
        positions[robot_id] =
            pointFromVariables(variables.data(), robot_id, sample);

      for (int iteration = 0; iteration < projection_iterations; ++iteration)
      {
        double maximum_correction = 0.0;
        for (size_t pair_index = 0; pair_index < pairs.size(); ++pair_index)
        {
          const int id_a = pairs[pair_index].first;
          const int id_b = pairs[pair_index].second;
          Eigen::Vector2d difference = positions[id_b] - positions[id_a];
          double distance = difference.norm();
          Eigen::Vector2d direction;
          if (distance > 1e-8)
          {
            direction = difference / distance;
          }
          else
          {
            direction =
                reference_paths_[id_b][sample] -
                reference_paths_[id_a][sample];
            direction = direction.norm() > 1e-8
                            ? direction.normalized()
                            : Eigen::Vector2d::UnitX();
            distance = 0.0;
          }

          const double lower =
              distance_lower_bounds_[sample][pair_index];
          const double upper =
              sheetPairLimit(id_a, id_b) - sheet_distance_margin_;
          double signed_correction = 0.0;
          if (distance < lower)
            signed_correction = 0.5 * (lower - distance);
          else if (distance > upper)
            signed_correction = -0.5 * (distance - upper);
          if (std::abs(signed_correction) <= 1e-8)
            continue;

          positions[id_a] -= signed_correction * direction;
          positions[id_b] += signed_correction * direction;
          maximum_correction =
              std::max(maximum_correction, std::abs(signed_correction));
        }
        if (maximum_correction < 1e-5)
          break;
      }

      for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
      {
        const int index = variableIndex(robot_id, sample, 0);
        variables[index] = positions[robot_id].x();
        variables[index + 1] = positions[robot_id].y();
      }
    }
  }

  static double evaluateCallback(
      void *instance, const double *variables, double *gradient, const int variable_count)
  {
    return static_cast<CentralizedTrajectoryOptimizer *>(instance)->evaluate(
        variables, gradient, variable_count);
  }

  double evaluate(
      const double *variables, double *gradient, int variable_count) const
  {
    std::fill(gradient, gradient + variable_count, 0.0);
    double cost = 0.0;

    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      for (int sample = 1; sample < sample_count_ - 1; ++sample)
      {
        const Eigen::Vector2d point = pointFromVariables(variables, robot_id, sample);
        const Eigen::Vector2d fitness_error =
            point - reference_paths_[robot_id][sample];
        cost += fitness_weight_ * fitness_error.squaredNorm();
        addGradient(
            gradient, robot_id, sample,
            2.0 * fitness_weight_ * fitness_error);

        Eigen::Vector2d distance_gradient;
        const double obstacle_distance =
            grid_.distanceAndGradient(point, distance_gradient);
        if (obstacle_distance < obstacle_clearance_)
        {
          const double residual = obstacle_clearance_ - obstacle_distance;
          if (distance_gradient.norm() < 0.20)
          {
            const Eigen::Vector2d toward_safe_reference =
                reference_paths_[robot_id][sample] - point;
            if (toward_safe_reference.norm() > 1e-6)
              distance_gradient = toward_safe_reference.normalized();
          }
          cost += active_obstacle_weight_ * residual * residual;
          addGradient(
              gradient, robot_id, sample,
              -2.0 * active_obstacle_weight_ * residual * distance_gradient);
        }
      }

      for (int sample = 0; sample < sample_count_ - 1; ++sample)
      {
        const Eigen::Vector2d point_a =
            pointFromVariables(variables, robot_id, sample);
        const Eigen::Vector2d point_b =
            pointFromVariables(variables, robot_id, sample + 1);
        for (int subdivision = 1;
             subdivision <= segment_collision_subsamples_; ++subdivision)
        {
          const double alpha =
              static_cast<double>(subdivision) /
              static_cast<double>(segment_collision_subsamples_ + 1);
          const Eigen::Vector2d point =
              (1.0 - alpha) * point_a + alpha * point_b;
          Eigen::Vector2d distance_gradient;
          const double obstacle_distance =
              grid_.distanceAndGradient(point, distance_gradient);
          if (obstacle_distance >= obstacle_clearance_)
            continue;

          const double residual =
              obstacle_clearance_ - obstacle_distance;
          if (distance_gradient.norm() < 0.20)
          {
            const Eigen::Vector2d reference_point =
                (1.0 - alpha) * reference_paths_[robot_id][sample] +
                alpha * reference_paths_[robot_id][sample + 1];
            const Eigen::Vector2d toward_reference =
                reference_point - point;
            if (toward_reference.norm() > 1e-6)
              distance_gradient = toward_reference.normalized();
          }
          const double normalization =
              1.0 / static_cast<double>(
                        std::max(1, segment_collision_subsamples_));
          cost += active_obstacle_weight_ * residual * residual *
                  normalization;
          const Eigen::Vector2d point_gradient =
              -2.0 * active_obstacle_weight_ * residual *
              distance_gradient * normalization;
          addGradient(
              gradient, robot_id, sample,
              (1.0 - alpha) * point_gradient);
          addGradient(
              gradient, robot_id, sample + 1,
              alpha * point_gradient);
        }
      }

      for (int sample = 1; sample < sample_count_ - 1; ++sample)
      {
        const Eigen::Vector2d previous =
            pointFromVariables(variables, robot_id, sample - 1);
        const Eigen::Vector2d current =
            pointFromVariables(variables, robot_id, sample);
        const Eigen::Vector2d next =
            pointFromVariables(variables, robot_id, sample + 1);
        const Eigen::Vector2d second_difference = previous - 2.0 * current + next;
        cost += smoothness_weight_ * second_difference.squaredNorm();
        const Eigen::Vector2d common_gradient =
            2.0 * smoothness_weight_ * second_difference;
        addGradient(gradient, robot_id, sample - 1, common_gradient);
        addGradient(gradient, robot_id, sample, -2.0 * common_gradient);
        addGradient(gradient, robot_id, sample + 1, common_gradient);
      }
    }

    for (int sample = 1; sample < sample_count_ - 1; ++sample)
    {
      for (int id_a = 0; id_a < robot_count_; ++id_a)
      {
        for (int id_b = id_a + 1; id_b < robot_count_; ++id_b)
        {
          const Eigen::Vector2d point_a =
              pointFromVariables(variables, id_a, sample);
          const Eigen::Vector2d point_b =
              pointFromVariables(variables, id_b, sample);
          const Eigen::Vector2d difference = point_b - point_a;
          const double distance = difference.norm();
          if (distance < swarm_clearance_)
          {
            const double residual = swarm_clearance_ - distance;
            cost += active_swarm_weight_ * residual * residual;
            Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
            if (distance > 1e-6)
              direction = difference / distance;
            const Eigen::Vector2d pair_gradient =
                2.0 * active_swarm_weight_ * residual * direction;
            addGradient(gradient, id_a, sample, pair_gradient);
            addGradient(gradient, id_b, sample, -pair_gradient);
          }
        }
      }
    }

    if (sheet_constraint_enabled_ && active_sheet_distance_weight_ > 0.0)
    {
      for (int sample = 1; sample < sample_count_ - 1; ++sample)
      {
        for (int id_a = 0; id_a < robot_count_; ++id_a)
        {
          for (int id_b = id_a + 1; id_b < robot_count_; ++id_b)
          {
            const Eigen::Vector2d point_a =
                pointFromVariables(variables, id_a, sample);
            const Eigen::Vector2d point_b =
                pointFromVariables(variables, id_b, sample);
            const Eigen::Vector2d difference = point_b - point_a;
            const double distance = difference.norm();
            const double soft_upper =
                sheetPairLimit(id_a, id_b) - sheet_distance_margin_;
            Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
            if (distance > 1e-6)
              direction = difference / distance;

            if (distance > soft_upper)
            {
              const double residual = distance - soft_upper;
              cost += active_sheet_distance_weight_ * residual * residual;
              const Eigen::Vector2d pair_gradient =
                  2.0 * active_sheet_distance_weight_ * residual * direction;
              addGradient(gradient, id_a, sample, -pair_gradient);
              addGradient(gradient, id_b, sample, pair_gradient);
            }

            const double lower =
                distance_lower_bounds_[sample][sheetPairIndex(id_a, id_b)];
            if (lower > 0.0 && distance < lower)
            {
              const double residual = lower - distance;
              cost += active_sheet_lower_bound_weight_ * residual * residual;
              const Eigen::Vector2d pair_gradient =
                  2.0 * active_sheet_lower_bound_weight_ * residual * direction;
              addGradient(gradient, id_a, sample, pair_gradient);
              addGradient(gradient, id_b, sample, -pair_gradient);
            }
          }
        }
      }
    }

    if (topology_enabled_ && active_topology_weight_ > 0.0)
    {
      for (int sample = 1; sample < sample_count_ - 1; ++sample)
      {
        for (const auto &triple : topology_triples_)
        {
          const DirectedDistance directed = directedDistance(
              pointFromVariables(variables, triple[0], sample),
              pointFromVariables(variables, triple[1], sample),
              pointFromVariables(variables, triple[2], sample));
          if (directed.value >= topology_min_directed_distance_)
            continue;

          const double residual =
              topology_min_directed_distance_ - directed.value;
          cost += active_topology_weight_ * residual * residual;
          const double scale = -2.0 * active_topology_weight_ * residual;
          addGradient(
              gradient, triple[0], sample, scale * directed.gradient_a);
          addGradient(
              gradient, triple[1], sample, scale * directed.gradient_b);
          addGradient(
              gradient, triple[2], sample, scale * directed.gradient_c);
        }
      }
    }

    const double segment_normalization =
        1.0 / static_cast<double>(
                  std::max(1, segment_collision_subsamples_));
    for (int sample = 0; sample < sample_count_ - 1; ++sample)
    {
      for (int subdivision = 1;
           subdivision <= segment_collision_subsamples_; ++subdivision)
      {
        const double alpha =
            static_cast<double>(subdivision) /
            static_cast<double>(segment_collision_subsamples_ + 1);
        std::vector<Eigen::Vector2d> positions(robot_count_);
        for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
        {
          positions[robot_id] =
              (1.0 - alpha) *
                  pointFromVariables(variables, robot_id, sample) +
              alpha *
                  pointFromVariables(variables, robot_id, sample + 1);
        }
        const auto add_interpolated_gradient =
            [this, gradient, sample, alpha](
                int robot_id, const Eigen::Vector2d &value)
            {
              addGradient(
                  gradient, robot_id, sample, (1.0 - alpha) * value);
              addGradient(
                  gradient, robot_id, sample + 1, alpha * value);
            };

        for (int id_a = 0; id_a < robot_count_; ++id_a)
        {
          for (int id_b = id_a + 1; id_b < robot_count_; ++id_b)
          {
            const Eigen::Vector2d difference =
                positions[id_b] - positions[id_a];
            const double distance = difference.norm();
            Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
            if (distance > 1e-6)
              direction = difference / distance;

            if (distance < swarm_clearance_)
            {
              const double residual = swarm_clearance_ - distance;
              cost += active_swarm_weight_ * residual * residual *
                      segment_normalization;
              const Eigen::Vector2d pair_gradient =
                  2.0 * active_swarm_weight_ * residual * direction *
                  segment_normalization;
              add_interpolated_gradient(id_a, pair_gradient);
              add_interpolated_gradient(id_b, -pair_gradient);
            }

            if (!sheet_constraint_enabled_)
              continue;
            const double soft_upper =
                sheetPairLimit(id_a, id_b) - sheet_distance_margin_;
            if (distance > soft_upper)
            {
              const double residual = distance - soft_upper;
              cost += active_sheet_distance_weight_ * residual * residual *
                      segment_normalization;
              const Eigen::Vector2d pair_gradient =
                  2.0 * active_sheet_distance_weight_ * residual *
                  direction * segment_normalization;
              add_interpolated_gradient(id_a, -pair_gradient);
              add_interpolated_gradient(id_b, pair_gradient);
            }

            const size_t pair_index = sheetPairIndex(id_a, id_b);
            const double lower =
                (1.0 - alpha) *
                    distance_lower_bounds_[sample][pair_index] +
                alpha *
                    distance_lower_bounds_[sample + 1][pair_index];
            if (lower > 0.0 && distance < lower)
            {
              const double residual = lower - distance;
              cost += active_sheet_lower_bound_weight_ *
                      residual * residual * segment_normalization;
              const Eigen::Vector2d pair_gradient =
                  2.0 * active_sheet_lower_bound_weight_ * residual *
                  direction * segment_normalization;
              add_interpolated_gradient(id_a, pair_gradient);
              add_interpolated_gradient(id_b, -pair_gradient);
            }
          }
        }

        if (topology_enabled_)
        {
          for (const auto &triple : topology_triples_)
          {
            const DirectedDistance directed = directedDistance(
                positions[triple[0]], positions[triple[1]],
                positions[triple[2]]);
            if (directed.value >= topology_min_directed_distance_)
              continue;

            const double residual =
                topology_min_directed_distance_ - directed.value;
            cost += active_topology_weight_ * residual * residual *
                    segment_normalization;
            const double scale =
                -2.0 * active_topology_weight_ * residual *
                segment_normalization;
            add_interpolated_gradient(
                triple[0], scale * directed.gradient_a);
            add_interpolated_gradient(
                triple[1], scale * directed.gradient_b);
            add_interpolated_gradient(
                triple[2], scale * directed.gradient_c);
          }
        }
      }
    }
    return cost;
  }

  std::vector<double> initialVariables() const
  {
    std::vector<double> variables(2 * robot_count_ * (sample_count_ - 2), 0.0);
    Eigen::Vector2d start_center = Eigen::Vector2d::Zero();
    Eigen::Vector2d goal_center = Eigen::Vector2d::Zero();
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      start_center += reference_paths_[robot_id].front();
      goal_center += reference_paths_[robot_id].back();
    }
    start_center /= static_cast<double>(robot_count_);
    goal_center /= static_cast<double>(robot_count_);

    for (int sample = 1; sample < sample_count_ - 1; ++sample)
    {
      Eigen::Vector2d path_center = Eigen::Vector2d::Zero();
      for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
        path_center += reference_paths_[robot_id][sample];
      path_center /= static_cast<double>(robot_count_);
      const double alpha =
          static_cast<double>(sample) /
          static_cast<double>(sample_count_ - 1);

      for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
      {
        const Eigen::Vector2d relative_offset =
            (1.0 - alpha) *
                (reference_paths_[robot_id].front() - start_center) +
            alpha * (reference_paths_[robot_id].back() - goal_center);
        const Eigen::Vector2d initial_point =
            path_center + relative_offset;
        const int index = variableIndex(robot_id, sample, 0);
        variables[index] = initial_point.x();
        variables[index + 1] = initial_point.y();
      }
    }
    return variables;
  }

  void tryOptimize()
  {
    if (optimizing_ || !have_map_)
      return;
    int64_t goal_stamp = -1;
    if (!haveCompleteBatch(goal_stamp))
      return;

    optimizing_ = true;
    last_attempted_goal_stamp_ = goal_stamp;
    prepareReferencePaths();
    for (auto &bounds : distance_lower_bounds_)
      std::fill(bounds.begin(), bounds.end(), 0.0);
    std::vector<double> variables = initialVariables();
    lbfgs::lbfgs_parameter_t parameters;
    lbfgs::lbfgs_load_default_parameters(&parameters);
    parameters.mem_size = 16;
    parameters.max_iterations = max_iterations_;
    parameters.g_epsilon = 1e-4;
    parameters.max_linesearch = 40;
    active_obstacle_weight_ = obstacle_weight_;
    active_swarm_weight_ = swarm_weight_;
    active_sheet_distance_weight_ = sheet_distance_weight_;
    active_sheet_lower_bound_weight_ = sheet_lower_bound_weight_;
    active_topology_weight_ = topology_weight_;
    const int outer_iteration_limit =
        object_clearance_enabled_ ? object_clearance_max_iterations_ : 0;
    int continuous_repair_attempts = 0;
    for (int outer_iteration = 0;
         outer_iteration <= outer_iteration_limit; ++outer_iteration)
    {
      double final_cost = 0.0;
      int result = 0;
      double minimum_obstacle_distance = 0.0;
      double minimum_swarm_distance = 0.0;
      double minimum_sheet_slack = 0.0;
      double minimum_sheet_lower_slack = 0.0;
      double minimum_topology_directed_distance = 0.0;
      bool sample_constraints_valid = false;
      for (int repair_attempt = 0;
           repair_attempt <= constraint_repair_max_attempts_;
           ++repair_attempt)
      {
        projectVariablesToSheetBounds(variables);
        result = lbfgs::lbfgs_optimize(
            static_cast<int>(variables.size()), variables.data(), &final_cost,
            CentralizedTrajectoryOptimizer::evaluateCallback,
            nullptr, nullptr, this, &parameters);
        const bool usable_early_stop =
            result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
            result == lbfgs::LBFGSERR_ROUNDING_ERROR ||
            result == lbfgs::LBFGSERR_MINIMUMSTEP ||
            result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH;
        if (result < 0 && !usable_early_stop)
        {
          RCLCPP_ERROR(
              get_logger(),
              "CENTRAL_OPT_FAILED outer_iteration=%d repair=%d solver=%d "
              "(%s), cost=%.3f",
              outer_iteration, repair_attempt, result,
              lbfgs::lbfgs_strerror(result), final_cost);
          optimizing_ = false;
          return;
        }
        if (result < 0)
        {
          RCLCPP_WARN(
              get_logger(),
              "central optimizer uses validated early-stop candidate: "
              "outer_iteration=%d repair=%d solver=%d (%s), cost=%.3f",
              outer_iteration, repair_attempt, result,
              lbfgs::lbfgs_strerror(result), final_cost);
        }

        optimized_paths_.assign(robot_count_, Path2d(sample_count_));
        for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
        {
          for (int sample = 0; sample < sample_count_; ++sample)
            optimized_paths_[robot_id][sample] =
                pointFromVariables(variables.data(), robot_id, sample);
        }

        evaluateSolution(
            minimum_obstacle_distance, minimum_swarm_distance,
            minimum_sheet_slack, minimum_sheet_lower_slack,
            minimum_topology_directed_distance);
        const bool obstacle_valid =
            minimum_obstacle_distance >= hard_obstacle_clearance_;
        const bool swarm_valid =
            minimum_swarm_distance >= hard_swarm_clearance_;
        const bool sheet_upper_valid =
            !sheet_constraint_enabled_ || minimum_sheet_slack >= -1e-3;
        const bool sheet_lower_valid =
            !sheet_constraint_enabled_ || minimum_sheet_lower_slack >= -0.03;
        const bool topology_valid =
            !topology_enabled_ ||
            minimum_topology_directed_distance >=
                topology_hard_min_directed_distance_;
        sample_constraints_valid =
            obstacle_valid && swarm_valid && sheet_upper_valid &&
            sheet_lower_valid && topology_valid;
        if (sample_constraints_valid)
          break;

        if (repair_attempt >= constraint_repair_max_attempts_)
        {
          RCLCPP_ERROR(
              get_logger(),
              "CENTRAL_OPT_FAILED unsafe result after repairs: "
              "outer_iteration=%d solver=%d cost=%.3f obstacle=%.3f "
              "swarm=%.3f sheet_slack=%.3f sheet_lower_slack=%.3f "
              "topology_directed=%.3f",
              outer_iteration, result, final_cost,
              minimum_obstacle_distance, minimum_swarm_distance,
              minimum_sheet_slack, minimum_sheet_lower_slack,
              minimum_topology_directed_distance);
          optimizing_ = false;
          return;
        }

        if (!obstacle_valid)
          active_obstacle_weight_ *= constraint_repair_weight_multiplier_;
        if (!swarm_valid)
          active_swarm_weight_ *= constraint_repair_weight_multiplier_;
        if (!sheet_upper_valid)
          active_sheet_distance_weight_ *=
              constraint_repair_weight_multiplier_;
        if (!sheet_lower_valid)
          active_sheet_lower_bound_weight_ *=
              constraint_repair_weight_multiplier_;
        if (!topology_valid)
          active_topology_weight_ *= constraint_repair_weight_multiplier_;
        RCLCPP_WARN(
            get_logger(),
            "CENTRAL_CONSTRAINT_REPAIR outer_iteration=%d attempt=%d "
            "obstacle=%.3f swarm=%.3f sheet=%.3f lower=%.3f topology=%.3f",
            outer_iteration, repair_attempt + 1,
            minimum_obstacle_distance, minimum_swarm_distance,
            minimum_sheet_slack, minimum_sheet_lower_slack,
            minimum_topology_directed_distance);
      }

      double spline_obstacle_distance = 0.0;
      double spline_swarm_distance = 0.0;
      double spline_sheet_slack = 0.0;
      double spline_sheet_lower_slack = 0.0;
      double spline_topology_directed_distance = 0.0;
      double minimum_object_clearance = 0.0;
      std::vector<int> object_violation_samples;
      if (!validateSplineSafety(
              spline_obstacle_distance, spline_swarm_distance,
              spline_sheet_slack, spline_sheet_lower_slack,
              spline_topology_directed_distance, minimum_object_clearance,
              object_violation_samples))
      {
        if (continuous_repair_attempts <
            constraint_repair_max_attempts_)
        {
          if (spline_obstacle_distance < hard_obstacle_clearance_)
            active_obstacle_weight_ *=
                constraint_repair_weight_multiplier_;
          if (spline_swarm_distance < hard_swarm_clearance_)
            active_swarm_weight_ *= constraint_repair_weight_multiplier_;
          if (sheet_constraint_enabled_ && spline_sheet_slack < -1e-3)
            active_sheet_distance_weight_ *=
                constraint_repair_weight_multiplier_;
          if (sheet_constraint_enabled_ &&
              spline_sheet_lower_slack < -0.03)
            active_sheet_lower_bound_weight_ *=
                constraint_repair_weight_multiplier_;
          if (topology_enabled_ &&
              spline_topology_directed_distance <
                  topology_hard_min_directed_distance_)
            active_topology_weight_ *=
                constraint_repair_weight_multiplier_;
          ++continuous_repair_attempts;
          RCLCPP_WARN(
              get_logger(),
              "CENTRAL_CONTINUOUS_REPAIR attempt=%d obstacle=%.3f "
              "swarm=%.3f sheet=%.3f lower=%.3f topology=%.3f",
              continuous_repair_attempts, spline_obstacle_distance,
              spline_swarm_distance, spline_sheet_slack,
              spline_sheet_lower_slack,
              spline_topology_directed_distance);
          --outer_iteration;
          continue;
        }
        RCLCPP_ERROR(
            get_logger(),
            "CENTRAL_OPT_FAILED continuous B-spline is unsafe: obstacle=%.3f "
            "swarm=%.3f sheet_slack=%.3f sheet_lower_slack=%.3f "
            "topology_directed=%.3f",
            spline_obstacle_distance, spline_swarm_distance,
            spline_sheet_slack, spline_sheet_lower_slack,
            spline_topology_directed_distance);
        optimizing_ = false;
        return;
      }

      if (!object_violation_samples.empty())
      {
        if (outer_iteration >= outer_iteration_limit)
        {
          RCLCPP_ERROR(
              get_logger(),
              "CENTRAL_OPT_FAILED object clearance infeasible: "
              "minimum_clearance=%.3f violations=%zu lower_utilization=%.2f "
              "reason=outer iteration limit reached",
              minimum_object_clearance, object_violation_samples.size(),
              maximumDistanceLowerUtilization());
          optimizing_ = false;
          return;
        }
        if (!raiseDistanceLowerBounds(object_violation_samples))
        {
          RCLCPP_ERROR(
              get_logger(),
              "CENTRAL_OPT_FAILED object clearance infeasible: "
              "minimum_clearance=%.3f violations=%zu lower_utilization=1.00 "
              "reason=distance lower bound equals upper bound",
              minimum_object_clearance, object_violation_samples.size());
          optimizing_ = false;
          return;
        }
        RCLCPP_WARN(
            get_logger(),
            "OBJECT_CLEARANCE_REPLAN model=%s iteration=%d violations=%zu "
            "minimum_clearance=%.3f max_lower_utilization=%.2f",
            robot_count_ == 4 ? "VVCM" : "geometric",
            outer_iteration + 1, object_violation_samples.size(),
            minimum_object_clearance, maximumDistanceLowerUtilization());
        continuous_repair_attempts = 0;
        continue;
      }

      publishSolution(
          goal_stamp, final_cost, result, spline_obstacle_distance,
          spline_swarm_distance, spline_sheet_slack,
          spline_topology_directed_distance, minimum_object_clearance,
          outer_iteration);
      last_optimized_goal_stamp_ = goal_stamp;
      optimizing_ = false;
      return;
    }

    optimizing_ = false;
  }

  void evaluateSolution(
      double &minimum_obstacle_distance, double &minimum_swarm_distance,
      double &minimum_sheet_slack,
      double &minimum_sheet_lower_slack,
      double &minimum_topology_directed_distance) const
  {
    minimum_obstacle_distance = std::numeric_limits<double>::infinity();
    minimum_swarm_distance = std::numeric_limits<double>::infinity();
    minimum_sheet_slack = std::numeric_limits<double>::infinity();
    minimum_sheet_lower_slack = std::numeric_limits<double>::infinity();
    minimum_topology_directed_distance =
        std::numeric_limits<double>::infinity();
    for (int sample = 0; sample < sample_count_; ++sample)
    {
      for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
      {
        Eigen::Vector2d gradient;
        minimum_obstacle_distance = std::min(
            minimum_obstacle_distance,
            grid_.distanceAndGradient(optimized_paths_[robot_id][sample], gradient));
      }
      for (int id_a = 0; id_a < robot_count_; ++id_a)
      {
        for (int id_b = id_a + 1; id_b < robot_count_; ++id_b)
        {
          const double distance =
              (optimized_paths_[id_b][sample] -
               optimized_paths_[id_a][sample])
                  .norm();
          minimum_swarm_distance = std::min(
              minimum_swarm_distance, distance);
          minimum_sheet_slack = std::min(
              minimum_sheet_slack,
              sheetPairLimit(id_a, id_b) - distance);
          const double lower =
              distance_lower_bounds_[sample][sheetPairIndex(id_a, id_b)];
          minimum_sheet_lower_slack = std::min(
              minimum_sheet_lower_slack, distance - lower);
        }
      }
      for (const auto &triple : topology_triples_)
      {
        minimum_topology_directed_distance = std::min(
            minimum_topology_directed_distance,
            directedDistance(
                optimized_paths_[triple[0]][sample],
                optimized_paths_[triple[1]][sample],
                optimized_paths_[triple[2]][sample])
                .value);
      }
    }
  }

  Eigen::MatrixXd makeControlPoints(const Path2d &path) const
  {
    constexpr int order = 3;
    std::vector<Eigen::Vector2d> dense_path;
    dense_path.reserve(
        1 + (path.size() - 1) *
                static_cast<size_t>(spline_densification_));
    dense_path.push_back(path.front());
    for (size_t segment = 0; segment + 1 < path.size(); ++segment)
    {
      for (int subdivision = 1;
           subdivision <= spline_densification_; ++subdivision)
      {
        const double alpha =
            static_cast<double>(subdivision) /
            static_cast<double>(spline_densification_);
        dense_path.push_back(
            (1.0 - alpha) * path[segment] +
            alpha * path[segment + 1]);
      }
    }

    Eigen::MatrixXd control_points(3, dense_path.size() + 4);
    int column = 0;
    for (int repeat = 0; repeat < order; ++repeat)
      control_points.col(column++) =
          Eigen::Vector3d(
              dense_path.front().x(), dense_path.front().y(), fixed_z_);
    for (size_t index = 1; index + 1 < dense_path.size(); ++index)
      control_points.col(column++) =
          Eigen::Vector3d(
              dense_path[index].x(), dense_path[index].y(), fixed_z_);
    for (int repeat = 0; repeat < order; ++repeat)
      control_points.col(column++) =
          Eigen::Vector3d(
              dense_path.back().x(), dense_path.back().y(), fixed_z_);
    return control_points;
  }

  double commonInterval(const std::vector<Eigen::MatrixXd> &control_points) const
  {
    double maximum_first_difference = 0.0;
    double maximum_second_difference = 0.0;
    for (const auto &points : control_points)
    {
      for (int column = 0; column + 1 < points.cols(); ++column)
      {
        maximum_first_difference = std::max(
            maximum_first_difference,
            (points.col(column + 1) - points.col(column)).head<2>().norm());
      }
      for (int column = 0; column + 2 < points.cols(); ++column)
      {
        maximum_second_difference = std::max(
            maximum_second_difference,
            (points.col(column + 2) - 2.0 * points.col(column + 1) +
             points.col(column))
                .head<2>()
                .norm());
      }
    }
    return std::max(
        {0.12,
         maximum_first_difference / std::max(0.05, max_velocity_),
         std::sqrt(maximum_second_difference / std::max(0.05, max_acceleration_))});
  }

  bool validateSplineSafety(
      double &minimum_obstacle_distance, double &minimum_swarm_distance,
      double &minimum_sheet_slack, double &minimum_sheet_lower_slack,
      double &minimum_topology_directed_distance,
      double &minimum_object_clearance,
      std::vector<int> &object_violation_samples) const
  {
    std::vector<Eigen::MatrixXd> control_points;
    std::vector<ego_planner::UniformBspline> trajectories;
    control_points.reserve(robot_count_);
    trajectories.reserve(robot_count_);
    for (const auto &path : optimized_paths_)
      control_points.push_back(makeControlPoints(path));
    const double interval = commonInterval(control_points);
    for (const auto &points : control_points)
      trajectories.emplace_back(points, 3, interval);

    const double duration = trajectories.front().getTimeSum();
    minimum_obstacle_distance = std::numeric_limits<double>::infinity();
    minimum_swarm_distance = std::numeric_limits<double>::infinity();
    minimum_sheet_slack = std::numeric_limits<double>::infinity();
    minimum_sheet_lower_slack = std::numeric_limits<double>::infinity();
    minimum_topology_directed_distance =
        std::numeric_limits<double>::infinity();
    minimum_object_clearance = std::numeric_limits<double>::infinity();
    object_violation_samples.clear();
    int closest_obstacle_robot = -1;
    double closest_obstacle_time = 0.0;
    Eigen::Vector2d closest_obstacle_position = Eigen::Vector2d::Zero();
    for (double time = 0.0; time <= duration + 1e-6; time += 0.03)
    {
      std::vector<Eigen::Vector2d> positions(robot_count_);
      for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
      {
        positions[robot_id] =
            trajectories[robot_id].evaluateDeBoorT(std::min(time, duration)).head<2>();
        Eigen::Vector2d gradient;
        const double obstacle_distance =
            grid_.distanceAndGradient(positions[robot_id], gradient);
        if (obstacle_distance < minimum_obstacle_distance)
        {
          minimum_obstacle_distance = obstacle_distance;
          closest_obstacle_robot = robot_id;
          closest_obstacle_time = time;
          closest_obstacle_position = positions[robot_id];
        }
      }
      const double normalized_time =
          duration > 1e-9 ? std::clamp(time / duration, 0.0, 1.0) : 0.0;
      const double sample_position =
          normalized_time * static_cast<double>(sample_count_ - 1);
      const int lower_sample = std::clamp(
          static_cast<int>(std::floor(sample_position)), 0, sample_count_ - 1);
      const int upper_sample = std::min(lower_sample + 1, sample_count_ - 1);
      const double sample_alpha =
          sample_position - static_cast<double>(lower_sample);
      for (int id_a = 0; id_a < robot_count_; ++id_a)
      {
        for (int id_b = id_a + 1; id_b < robot_count_; ++id_b)
        {
          const double distance = (positions[id_b] - positions[id_a]).norm();
          minimum_swarm_distance = std::min(
              minimum_swarm_distance, distance);
          const double pair_limit = sheetPairLimit(id_a, id_b);
          minimum_sheet_slack =
              std::min(minimum_sheet_slack, pair_limit - distance);
          const double lower =
              (1.0 - sample_alpha) *
                  distance_lower_bounds_[lower_sample]
                                        [sheetPairIndex(id_a, id_b)] +
              sample_alpha *
                  distance_lower_bounds_[upper_sample]
                                        [sheetPairIndex(id_a, id_b)];
          minimum_sheet_lower_slack =
              std::min(minimum_sheet_lower_slack, distance - lower);
        }
      }
      for (const auto &triple : topology_triples_)
      {
        minimum_topology_directed_distance = std::min(
            minimum_topology_directed_distance,
            directedDistance(
                positions[triple[0]], positions[triple[1]],
                positions[triple[2]])
                .value);
      }

      if (object_clearance_enabled_)
      {
        Eigen::Vector3d object_position;
        double obstacle_height = 0.0;
        const double clearance = objectObstacleClearance(
            positions, object_position, obstacle_height);
        minimum_object_clearance =
            std::min(minimum_object_clearance, clearance);
        if (clearance < 0.0)
        {
          object_violation_samples.push_back(std::clamp(
              static_cast<int>(std::llround(sample_position)),
              0, sample_count_ - 1));
        }
      }
    }
    const bool safe =
           minimum_obstacle_distance >= hard_obstacle_clearance_ &&
           minimum_swarm_distance >= hard_swarm_clearance_ &&
           (!sheet_constraint_enabled_ ||
            minimum_sheet_slack >= -1e-3) &&
           (!topology_enabled_ ||
            minimum_topology_directed_distance >=
                topology_hard_min_directed_distance_);
    if (!safe)
    {
      RCLCPP_WARN(
          get_logger(),
          "continuous minimum obstacle robot=%d time=%.2f "
          "position=(%.3f,%.3f) distance=%.3f",
          closest_obstacle_robot, closest_obstacle_time,
          closest_obstacle_position.x(), closest_obstacle_position.y(),
          minimum_obstacle_distance);
    }
    return safe;
  }

  void publishSolution(
      int64_t goal_stamp, double final_cost, int solver_result,
      double minimum_obstacle_distance, double minimum_swarm_distance,
      double minimum_sheet_slack,
      double minimum_topology_directed_distance,
      double minimum_object_clearance,
      int object_clearance_iterations)
  {
    std::vector<Eigen::MatrixXd> control_points;
    control_points.reserve(robot_count_);
    for (const auto &path : optimized_paths_)
      control_points.push_back(makeControlPoints(path));
    const double interval = commonInterval(control_points);
    ego_planner::UniformBspline knot_source(control_points.front(), 3, interval);
    const Eigen::VectorXd knots = knot_source.getKnot();
    const double duration = knot_source.getTimeSum();
    const int64_t trajectory_id = ++trajectory_id_;
    const rclcpp::Time start_time =
        rclcpp::Clock(RCL_SYSTEM_TIME).now() +
        rclcpp::Duration::from_seconds(start_time_lead_);
    traj_utils::msg::MultiBsplines trajectory_batch;
    trajectory_batch.drone_id_from = -1;
    trajectory_batch.traj.reserve(robot_count_);

    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      traj_utils::msg::Bspline message;
      message.drone_id = robot_id;
      message.order = 3;
      message.traj_id = trajectory_id;
      message.start_time = start_time;
      message.knots.reserve(knots.size());
      for (int index = 0; index < knots.size(); ++index)
        message.knots.push_back(knots(index));
      message.pos_pts.reserve(control_points[robot_id].cols());
      for (int column = 0; column < control_points[robot_id].cols(); ++column)
      {
        geometry_msgs::msg::Point point;
        point.x = control_points[robot_id](0, column);
        point.y = control_points[robot_id](1, column);
        point.z = fixed_z_;
        message.pos_pts.push_back(point);
      }
      trajectory_publishers_[robot_id]->publish(message);
      trajectory_batch.traj.push_back(message);
      publishOptimizedPath(robot_id, goal_stamp);
    }
    trajectory_batch_pub_->publish(trajectory_batch);

    std_msgs::msg::Int64MultiArray sync;
    sync.data = {
        trajectory_id,
        goal_stamp,
        static_cast<int64_t>(robot_count_),
        static_cast<int64_t>(control_points.front().cols()),
        static_cast<int64_t>(knots.size()),
        static_cast<int64_t>(std::llround(duration * 1000.0))};
    sync_pub_->publish(sync);
    publishMarkers(trajectory_id);

    RCLCPP_INFO(
        get_logger(),
        "CENTRAL_OPT_SUCCESS traj_id=%ld solver=%d cost=%.3f robots=%d "
        "control_points=%ld knots=%ld duration=%.2f obstacle_min=%.3f "
        "swarm_min=%.3f sheet_slack_min=%.3f topology_directed_min=%.3f "
        "object_clearance_min=%.3f clearance_iterations=%d",
        static_cast<long>(trajectory_id), solver_result, final_cost, robot_count_,
        static_cast<long>(control_points.front().cols()),
        static_cast<long>(knots.size()), duration, minimum_obstacle_distance,
        minimum_swarm_distance, minimum_sheet_slack,
        minimum_topology_directed_distance, minimum_object_clearance,
        object_clearance_iterations);
  }

  void publishOptimizedPath(int robot_id, int64_t goal_stamp)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp.sec = static_cast<int32_t>(goal_stamp / 1000000000LL);
    path.header.stamp.nanosec = static_cast<uint32_t>(goal_stamp % 1000000000LL);
    path.poses.reserve(optimized_paths_[robot_id].size());
    for (const auto &point : optimized_paths_[robot_id])
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = toPoint(point, fixed_z_);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    optimized_path_publishers_[robot_id]->publish(path);
  }

  void publishMarkers(int64_t trajectory_id)
  {
    visualization_msgs::msg::MarkerArray markers;
    const auto stamp = now();
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      visualization_msgs::msg::Marker path;
      path.header.frame_id = frame_id_;
      path.header.stamp = stamp;
      path.ns = "centralized_optimized_paths";
      path.id = robot_id;
      path.type = visualization_msgs::msg::Marker::LINE_STRIP;
      path.action = visualization_msgs::msg::Marker::ADD;
      path.pose.orientation.w = 1.0;
      path.scale.x = 0.08;
      path.color = robotColor(robot_id);
      for (const auto &point : optimized_paths_[robot_id])
        path.points.push_back(toPoint(point, fixed_z_ + 0.05));
      markers.markers.push_back(path);
    }

    visualization_msgs::msg::Marker net;
    net.header.frame_id = frame_id_;
    net.header.stamp = stamp;
    net.ns = "centralized_topology_samples";
    net.id = 10;
    net.type = visualization_msgs::msg::Marker::LINE_LIST;
    net.action = visualization_msgs::msg::Marker::ADD;
    net.pose.orientation.w = 1.0;
    net.scale.x = 0.025;
    net.color.r = 0.15f;
    net.color.g = 0.90f;
    net.color.b = 0.85f;
    net.color.a = 0.55f;
    const int stride = std::max(1, sample_count_ / 10);
    for (int sample = 0; sample < sample_count_; sample += stride)
    {
      for (const auto &pair : sheet_pairs_)
      {
        net.points.push_back(
            toPoint(optimized_paths_[pair.first][sample], fixed_z_ + 0.03));
        net.points.push_back(
            toPoint(optimized_paths_[pair.second][sample], fixed_z_ + 0.03));
      }
    }
    markers.markers.push_back(net);

    visualization_msgs::msg::Marker label;
    label.header = net.header;
    label.ns = "centralized_optimizer_label";
    label.id = 11;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.orientation.w = 1.0;
    Eigen::Vector2d goal_center = Eigen::Vector2d::Zero();
    for (const auto &path : optimized_paths_)
      goal_center += path.back();
    goal_center /= static_cast<double>(robot_count_);
    label.pose.position = toPoint(goal_center, fixed_z_ + 0.8);
    label.scale.z = 0.30;
    label.color.r = 1.0f;
    label.color.g = 1.0f;
    label.color.b = 1.0f;
    label.color.a = 1.0f;
    label.text = "centralized optimization " + std::to_string(trajectory_id);
    markers.markers.push_back(label);
    marker_pub_->publish(markers);
  }

  int robot_count_{4};
  int sample_count_{48};
  int spline_densification_{2};
  int segment_collision_subsamples_{3};
  int max_iterations_{250};
  std::string frame_id_{"world"};
  double fixed_z_{0.35};
  double map_size_x_{24.0};
  double map_size_y_{16.0};
  double map_resolution_{0.2};
  double collision_min_z_{-0.05};
  double collision_max_z_{0.85};
  double obstacle_clearance_{0.72};
  double swarm_clearance_{0.90};
  double hard_obstacle_clearance_{0.50};
  double hard_swarm_clearance_{0.85};
  double max_velocity_{2.0};
  double max_acceleration_{4.0};
  double start_time_lead_{0.5};
  double fitness_weight_{1.0};
  double smoothness_weight_{2.0};
  double obstacle_weight_{80.0};
  double swarm_weight_{10.0};
  bool sheet_constraint_enabled_{true};
  double sheet_length_{3.8};
  double sheet_width_{3.4};
  double formation_length_{2.4};
  double formation_width_{2.0};
  double sheet_distance_margin_{0.10};
  double sheet_distance_weight_{40.0};
  double sheet_lower_bound_weight_{1500.0};
  bool topology_enabled_{true};
  double topology_weight_{30.0};
  double topology_min_directed_distance_{0.35};
  double topology_hard_min_directed_distance_{0.08};
  bool object_clearance_enabled_{true};
  double rod_base_offset_z_{0.175};
  double rod_height_{2.4};
  double object_diameter_{0.45};
  double object_footprint_radius_{0.35};
  double object_obstacle_clearance_{0.10};
  double object_distance_increment_{0.05};
  double object_geometric_nominal_sag_{2.0};
  int object_lower_bound_neighbor_samples_{2};
  int object_clearance_max_iterations_{30};
  int constraint_repair_max_attempts_{4};
  double constraint_repair_weight_multiplier_{4.0};
  double active_obstacle_weight_{80.0};
  double active_swarm_weight_{10.0};
  double active_sheet_distance_weight_{40.0};
  double active_sheet_lower_bound_weight_{1500.0};
  double active_topology_weight_{30.0};
  bool have_map_{false};
  bool optimizing_{false};
  int64_t last_attempted_goal_stamp_{-1};
  int64_t last_optimized_goal_stamp_{-1};
  int64_t trajectory_id_{0};

  mutable std::unique_ptr<VVCM::VVCM_FK> vvcm_solver_;
  std::vector<int> boundary_order_;
  std::vector<std::pair<int, int>> sheet_pairs_;
  std::vector<std::pair<int, int>> sheet_boundary_pairs_;
  std::vector<std::array<int, 3>> topology_triples_;
  std::vector<Eigen::Vector2d> sheet_vertices_by_robot_;
  std::vector<Eigen::Vector2d> nominal_formation_vertices_by_robot_;
  std::vector<double> sheet_pair_limits_;
  std::vector<std::vector<double>> distance_lower_bounds_;
  std::vector<nav_msgs::msg::Path> latest_paths_;
  std::vector<int64_t> path_goal_stamps_;
  std::vector<Path2d> reference_paths_;
  std::vector<Path2d> optimized_paths_;
  legged_swarm_sim::PlanarGrid grid_;

  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> path_subscriptions_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::vector<rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr>
      trajectory_publishers_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr>
      optimized_path_publishers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr sync_pub_;
  rclcpp::Publisher<traj_utils::msg::MultiBsplines>::SharedPtr trajectory_batch_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CentralizedTrajectoryOptimizer>());
  rclcpp::shutdown();
  return 0;
}
