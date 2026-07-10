#include <bspline_opt/uniform_bspline.h>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <traj_utils/msg/bspline.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{
std_msgs::msg::ColorRGBA colorForRobot(int id)
{
  static const double palette[][3] = {
      {0.10, 0.55, 0.95},
      {0.95, 0.35, 0.20},
      {0.20, 0.70, 0.35},
      {0.75, 0.35, 0.95},
      {0.95, 0.70, 0.20},
      {0.10, 0.75, 0.75},
  };
  const auto &c = palette[std::abs(id) % 6];
  std_msgs::msg::ColorRGBA out;
  out.r = static_cast<float>(c[0]);
  out.g = static_cast<float>(c[1]);
  out.b = static_cast<float>(c[2]);
  out.a = 1.0f;
  return out;
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d &p)
{
  geometry_msgs::msg::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}
}  // namespace

class LeggedTrajFollower : public rclcpp::Node
{
public:
  LeggedTrajFollower() : Node("legged_traj_follower")
  {
    robot_id_ = declare_parameter<int>("robot_id", 0);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    const double init_x = declare_parameter<double>("initial_x", 0.0);
    const double init_y = declare_parameter<double>("initial_y", 0.0);
    const double init_z = declare_parameter<double>("initial_z", 0.6);
    body_length_ = declare_parameter<double>("body_length", 0.9);
    body_width_ = declare_parameter<double>("body_width", 0.45);
    body_height_ = declare_parameter<double>("body_height", 0.35);
    planar_mode_ = declare_parameter<bool>("planar_mode", true);
    fixed_z_ = declare_parameter<double>("fixed_z", init_z);
    const double odom_rate = declare_parameter<double>("odom_rate", 50.0);

    pos_ = Eigen::Vector3d(init_x, init_y, init_z);
    vel_.setZero();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom_world", 20);
    body_pub_ = create_publisher<visualization_msgs::msg::Marker>("robot_body_marker", 10);
    path_pub_ = create_publisher<visualization_msgs::msg::Marker>("planned_path_marker", 10);

    traj_sub_ = create_subscription<traj_utils::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&LeggedTrajFollower::bsplineCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, odom_rate));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&LeggedTrajFollower::timerCallback, this));
  }

private:
  void bsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg)
  {
    if (msg->pos_pts.size() < 4 || msg->knots.empty())
    {
      RCLCPP_WARN(get_logger(), "Ignore invalid B-spline: %zu points, %zu knots",
                  msg->pos_pts.size(), msg->knots.size());
      return;
    }

    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
    {
      pos_pts(0, i) = msg->pos_pts[i].x;
      pos_pts(1, i) = msg->pos_pts[i].y;
      pos_pts(2, i) = msg->pos_pts[i].z;
    }

    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i)
    {
      knots(static_cast<int>(i)) = msg->knots[i];
    }

    position_traj_ = ego_planner::UniformBspline(pos_pts, msg->order, 0.1);
    position_traj_.setKnot(knots);
    velocity_traj_ = position_traj_.getDerivative();
    acceleration_traj_ = velocity_traj_.getDerivative();
    traj_duration_ = position_traj_.getTimeSum();
    traj_start_time_ = rclcpp::Time(msg->start_time, RCL_SYSTEM_TIME);
    has_traj_ = true;
    publishPathMarker();

    RCLCPP_INFO(get_logger(), "robot_%d received trajectory %ld, duration %.2fs",
                robot_id_, static_cast<long>(msg->traj_id), traj_duration_);
  }

  void timerCallback()
  {
    if (has_traj_)
    {
      const auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
      const double t_raw = (now - traj_start_time_).seconds();
      const double t = std::clamp(t_raw, 0.0, traj_duration_);
      pos_ = position_traj_.evaluateDeBoorT(t);
      vel_ = t_raw <= traj_duration_ ? velocity_traj_.evaluateDeBoorT(t) : Eigen::Vector3d::Zero();
      if (planar_mode_)
      {
        pos_(2) = fixed_z_;
        vel_(2) = 0.0;
      }
    }

    publishOdom();
    publishBodyMarker();
  }

  void publishOdom()
  {
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = frame_id_;
    odom.header.stamp = now();
    odom.child_frame_id = "robot_" + std::to_string(robot_id_) + "/base";
    odom.pose.pose.position = toPoint(pos_);
    const double yaw = std::abs(vel_.x()) + std::abs(vel_.y()) > 1e-3
                           ? std::atan2(vel_.y(), vel_.x())
                           : last_yaw_;
    last_yaw_ = yaw;
    odom.pose.pose.orientation.w = std::cos(yaw * 0.5);
    odom.pose.pose.orientation.z = std::sin(yaw * 0.5);
    odom.twist.twist.linear.x = vel_.x();
    odom.twist.twist.linear.y = vel_.y();
    odom.twist.twist.linear.z = vel_.z();
    odom_pub_->publish(odom);
  }

  void publishBodyMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "legged_robot_body";
    marker.id = robot_id_;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = toPoint(pos_);
    marker.pose.orientation.w = std::cos(last_yaw_ * 0.5);
    marker.pose.orientation.z = std::sin(last_yaw_ * 0.5);
    marker.scale.x = body_length_;
    marker.scale.y = body_width_;
    marker.scale.z = body_height_;
    marker.color = colorForRobot(robot_id_);
    body_pub_->publish(marker);
  }

  void publishPathMarker()
  {
    if (!has_traj_)
      return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "legged_robot_plan";
    marker.id = robot_id_;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.06;
    marker.color = colorForRobot(robot_id_);

    const double dt = 0.15;
    for (double t = 0.0; t <= traj_duration_ + 1e-6; t += dt)
    {
      marker.points.push_back(toPoint(position_traj_.evaluateDeBoorT(std::min(t, traj_duration_))));
      if (planar_mode_)
      {
        marker.points.back().z = fixed_z_;
      }
    }
    path_pub_->publish(marker);
  }

  int robot_id_{0};
  std::string frame_id_{"world"};
  double body_length_{0.9};
  double body_width_{0.45};
  double body_height_{0.35};
  double traj_duration_{0.0};
  double last_yaw_{0.0};
  bool planar_mode_{true};
  double fixed_z_{0.35};
  bool has_traj_{false};

  Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_{Eigen::Vector3d::Zero()};
  ego_planner::UniformBspline position_traj_;
  ego_planner::UniformBspline velocity_traj_;
  ego_planner::UniformBspline acceleration_traj_;
  rclcpp::Time traj_start_time_{0, 0, RCL_SYSTEM_TIME};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr body_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr traj_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeggedTrajFollower>());
  rclcpp::shutdown();
  return 0;
}
