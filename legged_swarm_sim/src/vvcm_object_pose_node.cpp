#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <legged_swarm_sim/deformable_sheet_model.hpp>
#include <legged_swarm_sim/formation_geometry.hpp>
#include <VVCM_FK.hpp>

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
geometry_msgs::msg::Point toPoint(const Eigen::Vector3d &point)
{
  geometry_msgs::msg::Point message;
  message.x = point.x();
  message.y = point.y();
  message.z = point.z();
  return message;
}

std_msgs::msg::ColorRGBA color(
    float red, float green, float blue, float alpha = 1.0f)
{
  std_msgs::msg::ColorRGBA result;
  result.r = red;
  result.g = green;
  result.b = blue;
  result.a = alpha;
  return result;
}
}  // namespace

class VvcmObjectPoseNode : public rclcpp::Node
{
public:
  VvcmObjectPoseNode() : Node("vvcm_object_pose_node")
  {
    robot_count_ = declare_parameter<int>("robot_count", 4);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    update_rate_ = declare_parameter<double>("update_rate", 20.0);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);
    max_odom_skew_ = declare_parameter<double>("max_odom_skew", 1e-6);
    sheet_length_ = declare_parameter<double>("sheet.length", 3.8);
    sheet_width_ = declare_parameter<double>("sheet.width", 3.4);
    formation_length_ = declare_parameter<double>("formation.length", 2.4);
    formation_width_ = declare_parameter<double>("formation.width", 2.0);
    rod_base_offset_z_ =
        declare_parameter<double>("rod.base_offset_z", 0.175);
    rod_height_ = declare_parameter<double>("rod.height", 2.4);
    rod_diameter_ = declare_parameter<double>("rod.diameter", 0.08);
    object_diameter_ = declare_parameter<double>("object.diameter", 0.45);
    solver_enabled_ = declare_parameter<bool>("solver.enabled", true);
    fallback_sag_ = declare_parameter<double>("visual.fallback_sag", 2.0);
    cable_width_ = declare_parameter<double>("visual.cable_width", 0.025);
    sheet_edge_width_ =
        declare_parameter<double>("visual.sheet_edge_width", 0.035);
    path_history_length_ =
        declare_parameter<int>("visual.path_history_length", 1200);

    legged_swarm_sim::validateRobotCount(robot_count_);
    if (update_rate_ <= 0.0 || odom_timeout_ <= 0.0 ||
        max_odom_skew_ < 0.0 || sheet_length_ <= 0.0 ||
        sheet_width_ <= 0.0 || formation_length_ <= 0.0 ||
        formation_width_ <= 0.0 || rod_height_ <= 0.0 ||
        rod_diameter_ <= 0.0 || object_diameter_ <= 0.0 ||
        fallback_sag_ <= 0.0 || path_history_length_ < 1)
      throw std::runtime_error("invalid VVCM visualization parameters");

    boundary_order_ = legged_swarm_sim::boundaryOrder(robot_count_);
    sheet_vertices_by_robot_ =
        legged_swarm_sim::formationVerticesByRobot(
            robot_count_, sheet_length_, sheet_width_);
    nominal_formation_vertices_by_robot_ =
        legged_swarm_sim::formationVerticesByRobot(
            robot_count_, formation_length_, formation_width_);
    VVCM::MatrixXf sheet_vertices(robot_count_, 2);
    for (int row = 0; row < robot_count_; ++row)
    {
      const auto &vertex = sheet_vertices_by_robot_[boundary_order_[row]];
      sheet_vertices(row, 0) = static_cast<float>(vertex.x());
      sheet_vertices(row, 1) = static_cast<float>(vertex.y());
    }
    if (solver_enabled_)
    {
      solver_ = std::make_unique<VVCM::VVCM_FK>(
          robot_count_, 0.0F, sheet_vertices);
    }

    latest_odometry_.resize(robot_count_);
    odom_subscriptions_.reserve(robot_count_);
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      odom_subscriptions_.push_back(
          create_subscription<nav_msgs::msg::Odometry>(
              "/robot_" + std::to_string(robot_id) + "/odom_world", 20,
              [this, robot_id](const nav_msgs::msg::Odometry::SharedPtr message)
              {
                latest_odometry_[robot_id] = message;
              }));
    }
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/swarm/vvcm_object_pose", 20);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/swarm/vvcm_markers", 20);
    status_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/swarm/vvcm_status", 20);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/swarm/vvcm_object_path",
        rclcpp::QoS(1).reliable().transient_local());

    const auto period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&VvcmObjectPoseNode::timerCallback, this));
    object_path_.header.frame_id = frame_id_;
    RCLCPP_INFO(
        get_logger(),
        "object and sheet visualizer ready: robots=%d sheet=%.2fx%.2f "
        "rod_height=%.2f solver=%s",
        robot_count_, sheet_length_, sheet_width_, rod_height_,
        solver_enabled_ ? "VVCM" : "geometric_fallback");
  }

private:
  bool odometryReady(rclcpp::Time &common_stamp) const
  {
    int64_t minimum_stamp = std::numeric_limits<int64_t>::max();
    int64_t maximum_stamp = std::numeric_limits<int64_t>::min();
    const rclcpp::Time current_time = now();
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      if (!latest_odometry_[robot_id])
        return false;
      const rclcpp::Time stamp(latest_odometry_[robot_id]->header.stamp);
      if ((current_time - stamp).seconds() > odom_timeout_)
        return false;
      minimum_stamp = std::min(minimum_stamp, stamp.nanoseconds());
      maximum_stamp = std::max(maximum_stamp, stamp.nanoseconds());
    }
    if (static_cast<double>(maximum_stamp - minimum_stamp) * 1e-9 >
        max_odom_skew_)
      return false;
    common_stamp = rclcpp::Time(maximum_stamp);
    return true;
  }

  void timerCallback()
  {
    rclcpp::Time common_stamp(0, 0, get_clock()->get_clock_type());
    if (!odometryReady(common_stamp))
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "VVCM waits for %d synchronized, fresh odometry messages",
          robot_count_);
      return;
    }

    std::vector<Eigen::Vector3d> robot_positions(robot_count_);
    std::vector<Eigen::Vector3d> rod_bases(robot_count_);
    std::vector<Eigen::Vector3d> holding_points(robot_count_);
    double holding_height = 0.0;
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      const auto &position =
          latest_odometry_[robot_id]->pose.pose.position;
      robot_positions[robot_id] =
          Eigen::Vector3d(position.x, position.y, position.z);
      rod_bases[robot_id] =
          robot_positions[robot_id] +
          Eigen::Vector3d(0.0, 0.0, rod_base_offset_z_);
      holding_points[robot_id] =
          rod_bases[robot_id] +
          Eigen::Vector3d(0.0, 0.0, rod_height_);
      holding_height += holding_points[robot_id].z();
    }
    holding_height /= static_cast<double>(robot_count_);

    Eigen::Vector2d formation_center = Eigen::Vector2d::Zero();
    for (const auto &holding_point : holding_points)
      formation_center += holding_point.head<2>();
    formation_center /= static_cast<double>(holding_points.size());

    Eigen::Vector2d nominal_center = Eigen::Vector2d::Zero();
    for (const auto &vertex : sheet_vertices_by_robot_)
      nominal_center += vertex;
    nominal_center /= static_cast<double>(robot_count_);
    Eigen::Matrix2d cross_covariance = Eigen::Matrix2d::Zero();
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      cross_covariance +=
          (robot_positions[robot_id].head<2>() - formation_center) *
          (sheet_vertices_by_robot_[robot_id] - nominal_center).transpose();
    }
    const double rotation_signal =
        cross_covariance(0, 0) + cross_covariance(1, 1);
    const double skew_signal =
        cross_covariance(1, 0) - cross_covariance(0, 1);
    if (std::hypot(rotation_signal, skew_signal) > 1e-9)
      object_yaw_ = std::atan2(skew_signal, rotation_signal);

    std::vector<Eigen::Vector2d> planar_robot_positions;
    planar_robot_positions.reserve(robot_positions.size());
    for (const auto &position : robot_positions)
      planar_robot_positions.push_back(position.head<2>());
    Eigen::Vector3d object_position;
    if (!legged_swarm_sim::geometricObjectPosition(
            planar_robot_positions, nominal_formation_vertices_by_robot_,
            holding_height, fallback_sag_, object_position))
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "cannot compute geometric object visualization");
      return;
    }
    object_position.z() = std::max(
        0.5 * object_diameter_ + 0.05, object_position.z());
    int selected_solution = -1;
    int stable_solution_count = 0;
    bool have_solver_solution = false;
    if (solver_enabled_)
    {
      VVCM::MatrixXf robot_formation(robot_count_, 2);
      for (size_t row = 0; row < boundary_order_.size(); ++row)
      {
        robot_formation(static_cast<int>(row), 0) =
            static_cast<float>(
                holding_points[boundary_order_[row]].x() -
                formation_center.x());
        robot_formation(static_cast<int>(row), 1) =
            static_cast<float>(
                holding_points[boundary_order_[row]].y() -
                formation_center.y());
      }
      solver_->zr = static_cast<float>(holding_height);
      const auto error = solver_->update_stable_solutions(robot_formation);
      if (error == VVCM::VVCM_FK_Error::NoError && !solver_->Po.empty())
      {
        selected_solution =
            static_cast<int>(selectSolution(formation_center));
        stable_solution_count = static_cast<int>(solver_->M);
        object_position =
            solver_->Po[static_cast<size_t>(selected_solution)].cast<double>();
        object_position.x() += formation_center.x();
        object_position.y() += formation_center.y();
        if (!have_previous_solution_)
        {
          RCLCPP_INFO(
              get_logger(),
              "VVCM_OBJECT_POSE_READY stable=%d selected=%d "
              "position=(%.3f,%.3f,%.3f)",
              stable_solution_count, selected_solution, object_position.x(),
              object_position.y(), object_position.z());
        }
        previous_object_position_ = object_position;
        have_previous_solution_ = true;
        have_solver_solution = true;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "VVCM has no stable object solution: error=%d sheet=%.2fx%.2f; "
            "publishing geometric RViz markers only",
            static_cast<int>(error), sheet_length_, sheet_width_);
      }
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.header.stamp = common_stamp;
    pose.pose.position = toPoint(object_position);
    pose.pose.orientation.w = std::cos(0.5 * object_yaw_);
    pose.pose.orientation.z = std::sin(0.5 * object_yaw_);

    publishMarkers(
        common_stamp, rod_bases, holding_points,
        object_position, pose, selected_solution, stable_solution_count,
        have_solver_solution);
    if (!markers_ready_logged_)
    {
      RCLCPP_INFO(
          get_logger(),
          "OBJECT_VISUAL_MARKERS_READY rods=%d sheet_triangles=%d object=1 "
          "mode=%s",
          robot_count_, robot_count_,
          have_solver_solution ? "VVCM" : "geometric_fallback");
      markers_ready_logged_ = true;
    }
    publishStatus(
        holding_height, object_position, selected_solution,
        stable_solution_count);
    if (have_solver_solution)
    {
      pose_pub_->publish(pose);
      publishPath(pose);
    }
  }

  size_t selectSolution(const Eigen::Vector2d &formation_center) const
  {
    size_t selected = 0;
    if (!have_previous_solution_)
    {
      for (size_t index = 1; index < solver_->Po.size(); ++index)
      {
        if (solver_->Po[index].z() < solver_->Po[selected].z())
          selected = index;
      }
      return selected;
    }

    double best_distance = std::numeric_limits<double>::infinity();
    for (size_t index = 0; index < solver_->Po.size(); ++index)
    {
      Eigen::Vector3d candidate = solver_->Po[index].cast<double>();
      candidate.x() += formation_center.x();
      candidate.y() += formation_center.y();
      const double distance =
          (candidate - previous_object_position_).squaredNorm();
      if (distance < best_distance)
      {
        best_distance = distance;
        selected = index;
      }
    }
    return selected;
  }

  void publishMarkers(
      const rclcpp::Time &stamp,
      const std::vector<Eigen::Vector3d> &rod_bases,
      const std::vector<Eigen::Vector3d> &holding_points,
      const Eigen::Vector3d &object_position,
      const geometry_msgs::msg::PoseStamped &object_pose,
      int selected_solution,
      int stable_solution_count,
      bool have_solver_solution)
  {
    visualization_msgs::msg::MarkerArray markers;
    const auto make_header = [this, &stamp]()
    {
      std_msgs::msg::Header header;
      header.frame_id = frame_id_;
      header.stamp = stamp;
      return header;
    };

    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      visualization_msgs::msg::Marker rod;
      rod.header = make_header();
      rod.ns = "vvcm_rods";
      rod.id = robot_id;
      rod.type = visualization_msgs::msg::Marker::CYLINDER;
      rod.action = visualization_msgs::msg::Marker::ADD;
      rod.pose.position = toPoint(
          0.5 * (rod_bases[robot_id] + holding_points[robot_id]));
      rod.pose.orientation.w = 1.0;
      rod.scale.x = rod_diameter_;
      rod.scale.y = rod_diameter_;
      rod.scale.z = rod_height_;
      rod.color = color(0.78f, 0.58f, 0.24f);
      markers.markers.push_back(rod);
    }

    visualization_msgs::msg::Marker holding;
    holding.header = make_header();
    holding.ns = "vvcm_holding_points";
    holding.id = 10;
    holding.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    holding.action = visualization_msgs::msg::Marker::ADD;
    holding.pose.orientation.w = 1.0;
    holding.scale.x = 0.14;
    holding.scale.y = 0.14;
    holding.scale.z = 0.14;
    holding.color = color(1.0f, 0.82f, 0.15f);
    for (const auto &point : holding_points)
      holding.points.push_back(toPoint(point));
    markers.markers.push_back(holding);

    std::vector<int> closed_boundary = boundary_order_;
    closed_boundary.push_back(boundary_order_.front());
    visualization_msgs::msg::Marker boundary;
    boundary.header = make_header();
    boundary.ns = "vvcm_sheet_boundary";
    boundary.id = 11;
    boundary.type = visualization_msgs::msg::Marker::LINE_STRIP;
    boundary.action = visualization_msgs::msg::Marker::ADD;
    boundary.pose.orientation.w = 1.0;
    boundary.scale.x = sheet_edge_width_;
    boundary.color = color(0.15f, 0.80f, 0.95f, 0.95f);
    for (const int robot_id : closed_boundary)
      boundary.points.push_back(toPoint(holding_points[robot_id]));
    markers.markers.push_back(boundary);

    visualization_msgs::msg::Marker cables;
    cables.header = make_header();
    cables.ns = "vvcm_virtual_cables";
    cables.id = 12;
    cables.type = visualization_msgs::msg::Marker::LINE_LIST;
    cables.action = visualization_msgs::msg::Marker::ADD;
    cables.pose.orientation.w = 1.0;
    cables.scale.x = cable_width_;
    cables.color = color(0.95f, 0.50f, 0.10f, 0.95f);
    for (const auto &holding_point : holding_points)
    {
      cables.points.push_back(toPoint(holding_point));
      cables.points.push_back(toPoint(object_position));
    }
    markers.markers.push_back(cables);

    visualization_msgs::msg::Marker sheet;
    sheet.header = make_header();
    sheet.ns = "vvcm_sheet_surface";
    sheet.id = 13;
    sheet.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    sheet.action = visualization_msgs::msg::Marker::ADD;
    sheet.pose.orientation.w = 1.0;
    sheet.scale.x = 1.0;
    sheet.scale.y = 1.0;
    sheet.scale.z = 1.0;
    sheet.color = color(0.10f, 0.65f, 0.90f, 0.18f);
    for (size_t edge = 0; edge + 1 < closed_boundary.size(); ++edge)
    {
      sheet.points.push_back(
          toPoint(holding_points[closed_boundary[edge]]));
      sheet.points.push_back(
          toPoint(holding_points[closed_boundary[edge + 1]]));
      sheet.points.push_back(toPoint(object_position));
    }
    markers.markers.push_back(sheet);

    visualization_msgs::msg::Marker object;
    object.header = make_header();
    object.ns = "vvcm_object";
    object.id = 14;
    object.type = visualization_msgs::msg::Marker::SPHERE;
    object.action = visualization_msgs::msg::Marker::ADD;
    object.pose = object_pose.pose;
    object.scale.x = object_diameter_;
    object.scale.y = object_diameter_;
    object.scale.z = object_diameter_;
    object.color = have_solver_solution
                       ? color(0.95f, 0.18f, 0.15f, 0.95f)
                       : color(1.0f, 0.72f, 0.10f, 0.95f);
    markers.markers.push_back(object);

    visualization_msgs::msg::Marker label;
    label.header = make_header();
    label.ns = "vvcm_object_label";
    label.id = 15;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position = toPoint(
        object_position +
        Eigen::Vector3d(0.0, 0.0, 0.5 * object_diameter_ + 0.25));
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.22;
    label.color = color(1.0f, 1.0f, 1.0f);
    if (have_solver_solution)
    {
      label.text =
          "VVCM object z=" +
          std::to_string(object_position.z()).substr(0, 5) +
          " stable=" + std::to_string(stable_solution_count) +
          " selected=" + std::to_string(selected_solution);
    }
    else
    {
      label.text =
          "object visual estimate z=" +
          std::to_string(object_position.z()).substr(0, 5);
    }
    markers.markers.push_back(label);

    marker_pub_->publish(markers);
  }

  void publishStatus(
      double holding_height, const Eigen::Vector3d &object_position,
      int selected_solution, int stable_solution_count)
  {
    std_msgs::msg::Float64MultiArray status;
    status.data = {
        static_cast<double>(stable_solution_count),
        static_cast<double>(selected_solution),
        sheet_length_,
        sheet_width_,
        holding_height,
        object_position.x(),
        object_position.y(),
        object_position.z()};
    status_pub_->publish(status);
  }

  void publishPath(const geometry_msgs::msg::PoseStamped &pose)
  {
    object_path_.header = pose.header;
    bool append = object_path_.poses.empty();
    if (!append)
    {
      const auto &last = object_path_.poses.back().pose.position;
      append = std::hypot(
                   std::hypot(
                       pose.pose.position.x - last.x,
                       pose.pose.position.y - last.y),
                   pose.pose.position.z - last.z) >
               0.01;
    }
    if (append)
    {
      object_path_.poses.push_back(pose);
      if (object_path_.poses.size() >
          static_cast<size_t>(path_history_length_))
        object_path_.poses.erase(object_path_.poses.begin());
    }
    path_pub_->publish(object_path_);
  }

  int robot_count_{4};
  int path_history_length_{1200};
  std::string frame_id_{"world"};
  double update_rate_{20.0};
  double odom_timeout_{0.5};
  double max_odom_skew_{1e-6};
  double sheet_length_{3.8};
  double sheet_width_{3.4};
  double formation_length_{2.4};
  double formation_width_{2.0};
  double rod_base_offset_z_{0.175};
  double rod_height_{2.4};
  double rod_diameter_{0.08};
  double object_diameter_{0.45};
  double fallback_sag_{2.0};
  double cable_width_{0.025};
  double sheet_edge_width_{0.035};
  double object_yaw_{0.0};
  bool solver_enabled_{true};
  bool have_previous_solution_{false};
  bool markers_ready_logged_{false};
  Eigen::Vector3d previous_object_position_{Eigen::Vector3d::Zero()};
  std::unique_ptr<VVCM::VVCM_FK> solver_;
  std::vector<int> boundary_order_;
  std::vector<Eigen::Vector2d> sheet_vertices_by_robot_;
  std::vector<Eigen::Vector2d> nominal_formation_vertices_by_robot_;
  std::vector<nav_msgs::msg::Odometry::SharedPtr> latest_odometry_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
      odom_subscriptions_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path object_path_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<VvcmObjectPoseNode>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(
        rclcpp::get_logger("vvcm_object_pose_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
