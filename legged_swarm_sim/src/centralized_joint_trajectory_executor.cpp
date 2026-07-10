#include <bspline_opt/uniform_bspline.h>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <traj_utils/msg/bspline.hpp>
#include <traj_utils/msg/multi_bsplines.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Eigen>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d &point)
{
  geometry_msgs::msg::Point message;
  message.x = point.x();
  message.y = point.y();
  message.z = point.z();
  return message;
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std_msgs::msg::ColorRGBA robotColor(int robot_id)
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
  color.a = 1.0f;
  return color;
}
}  // namespace

class CentralizedJointTrajectoryExecutor : public rclcpp::Node
{
public:
  CentralizedJointTrajectoryExecutor() : Node("centralized_joint_trajectory_executor")
  {
    robot_count_ = declare_parameter<int>("robot_count", 4);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    fixed_z_ = declare_parameter<double>("fixed_z", 0.35);
    body_length_ = declare_parameter<double>("body_length", 0.9);
    body_width_ = declare_parameter<double>("body_width", 0.45);
    body_height_ = declare_parameter<double>("body_height", 0.35);
    minimum_start_delay_ = declare_parameter<double>("minimum_start_delay", 0.25);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 1.2);
    heading_speed_threshold_ =
        declare_parameter<double>("heading_speed_threshold", 0.03);
    allow_reverse_motion_ =
        declare_parameter<bool>("allow_reverse_motion", true);
    const double update_rate = declare_parameter<double>("update_rate", 50.0);
    const std::vector<double> initial_x =
        declare_parameter<std::vector<double>>("initial_x", std::vector<double>{});
    const std::vector<double> initial_y =
        declare_parameter<std::vector<double>>("initial_y", std::vector<double>{});

    if (robot_count_ < 3 || robot_count_ > 5 ||
        initial_x.size() != static_cast<size_t>(robot_count_) ||
        initial_y.size() != static_cast<size_t>(robot_count_))
      throw std::runtime_error(
          "joint executor requires 3-5 matching initial x/y positions");
    if (max_yaw_rate_ <= 0.0 || heading_speed_threshold_ < 0.0 ||
        update_rate <= 0.0)
      throw std::runtime_error("invalid joint executor heading parameters");

    positions_.resize(robot_count_);
    velocities_.assign(robot_count_, Eigen::Vector3d::Zero());
    last_yaws_.assign(robot_count_, 0.0);
    yaw_rates_.assign(robot_count_, 0.0);
    reverse_motion_.assign(robot_count_, false);
    pending_messages_.resize(robot_count_);
    position_trajectories_.resize(robot_count_);
    velocity_trajectories_.resize(robot_count_);
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
      positions_[robot_id] = Eigen::Vector3d(initial_x[robot_id], initial_y[robot_id], fixed_z_);

    body_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/swarm/centralized_robot_bodies", 10);
    path_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/swarm/centralized_robot_planned_paths",
        rclcpp::QoS(1).reliable().transient_local());
    odom_publishers_.reserve(robot_count_);
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      odom_publishers_.push_back(create_publisher<nav_msgs::msg::Odometry>(
          "/robot_" + std::to_string(robot_id) + "/odom_world", 20));
    }
    trajectory_batch_sub_ = create_subscription<traj_utils::msg::MultiBsplines>(
        "/swarm/centralized_bspline_batch",
        rclcpp::QoS(1).reliable().transient_local(),
        std::bind(
            &CentralizedJointTrajectoryExecutor::trajectoryBatchCallback,
            this, std::placeholders::_1));

    update_period_ = 1.0 / update_rate;
    const auto period = std::chrono::duration<double>(update_period_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&CentralizedJointTrajectoryExecutor::timerCallback, this));
    RCLCPP_INFO(
        get_logger(),
        "joint trajectory executor publishes %d robots from one timer",
        robot_count_);
  }

private:
  void trajectoryBatchCallback(
      const traj_utils::msg::MultiBsplines::SharedPtr batch)
  {
    if (batch->traj.size() != static_cast<size_t>(robot_count_))
    {
      RCLCPP_ERROR(
          get_logger(), "JOINT_EXECUTOR_REJECTED batch contains %zu trajectories",
          batch->traj.size());
      return;
    }

    std::fill(pending_messages_.begin(), pending_messages_.end(), nullptr);
    pending_trajectory_id_ = batch->traj.front().traj_id;
    for (const auto &trajectory : batch->traj)
    {
      if (trajectory.drone_id < 0 || trajectory.drone_id >= robot_count_ ||
          pending_messages_[trajectory.drone_id] != nullptr)
      {
        RCLCPP_ERROR(
            get_logger(), "JOINT_EXECUTOR_REJECTED invalid or duplicate drone_id=%d",
            trajectory.drone_id);
        return;
      }
      pending_messages_[trajectory.drone_id] =
          std::make_shared<traj_utils::msg::Bspline>(trajectory);
    }
    acceptPendingBatch();
  }

  bool batchesMatch() const
  {
    const auto &reference = *pending_messages_.front();
    for (int robot_id = 1; robot_id < robot_count_; ++robot_id)
    {
      const auto &candidate = *pending_messages_[robot_id];
      if (candidate.traj_id != reference.traj_id ||
          candidate.order != reference.order ||
          candidate.start_time.sec != reference.start_time.sec ||
          candidate.start_time.nanosec != reference.start_time.nanosec ||
          candidate.pos_pts.size() != reference.pos_pts.size() ||
          candidate.knots.size() != reference.knots.size())
        return false;
      for (size_t knot_index = 0; knot_index < reference.knots.size(); ++knot_index)
      {
        if (std::abs(candidate.knots[knot_index] - reference.knots[knot_index]) > 1e-9)
          return false;
      }
    }
    return true;
  }

  void acceptPendingBatch()
  {
    if (!batchesMatch())
    {
      RCLCPP_ERROR(
          get_logger(),
          "JOINT_EXECUTOR_REJECTED traj_id=%ld because timing or point counts differ",
          static_cast<long>(pending_trajectory_id_));
      return;
    }

    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      const auto &message = *pending_messages_[robot_id];
      Eigen::MatrixXd control_points(3, message.pos_pts.size());
      for (size_t point_index = 0; point_index < message.pos_pts.size(); ++point_index)
      {
        control_points.col(point_index) =
            Eigen::Vector3d(
                message.pos_pts[point_index].x,
                message.pos_pts[point_index].y,
                fixed_z_);
      }
      Eigen::VectorXd knots(message.knots.size());
      for (size_t knot_index = 0; knot_index < message.knots.size(); ++knot_index)
        knots(static_cast<int>(knot_index)) = message.knots[knot_index];

      position_trajectories_[robot_id] =
          ego_planner::UniformBspline(control_points, message.order, 0.1);
      position_trajectories_[robot_id].setKnot(knots);
      velocity_trajectories_[robot_id] =
          position_trajectories_[robot_id].getDerivative();
    }

    selectMotionDirections();
    active_trajectory_id_ = pending_trajectory_id_;
    const rclcpp::Time requested_start(
        pending_messages_.front()->start_time, RCL_SYSTEM_TIME);
    const rclcpp::Time earliest_start =
        rclcpp::Clock(RCL_SYSTEM_TIME).now() +
        rclcpp::Duration::from_seconds(minimum_start_delay_);
    trajectory_start_time_ =
        requested_start < earliest_start ? earliest_start : requested_start;
    trajectory_duration_ = position_trajectories_.front().getTimeSum();
    have_active_trajectory_ = true;
    publishPlannedPaths();
    RCLCPP_INFO(
        get_logger(),
        "JOINT_EXECUTOR_ACCEPTED traj_id=%ld robots=%d control_points=%zu "
        "knots=%zu duration=%.2f",
        static_cast<long>(active_trajectory_id_), robot_count_,
        pending_messages_.front()->pos_pts.size(),
        pending_messages_.front()->knots.size(), trajectory_duration_);
  }

  void timerCallback()
  {
    const rclcpp::Time common_stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    double elapsed = update_period_;
    if (last_update_time_.nanoseconds() > 0)
    {
      const double measured = (common_stamp - last_update_time_).seconds();
      if (measured > 0.0)
        elapsed = std::min(measured, 0.20);
    }
    last_update_time_ = common_stamp;

    if (have_active_trajectory_)
    {
      const double raw_time = (common_stamp - trajectory_start_time_).seconds();
      const double trajectory_time = std::clamp(raw_time, 0.0, trajectory_duration_);
      for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
      {
        positions_[robot_id] =
            position_trajectories_[robot_id].evaluateDeBoorT(trajectory_time);
        velocities_[robot_id] =
            raw_time >= 0.0 && raw_time <= trajectory_duration_
                ? velocity_trajectories_[robot_id].evaluateDeBoorT(trajectory_time)
                : Eigen::Vector3d::Zero();
        positions_[robot_id].z() = fixed_z_;
        velocities_[robot_id].z() = 0.0;
      }
    }

    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      updateYaw(robot_id, elapsed);
      publishOdometry(robot_id, common_stamp);
    }
    publishBodies(common_stamp);
  }

  void selectMotionDirections()
  {
    std::string modes;
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      reverse_motion_[robot_id] = false;
      if (allow_reverse_motion_)
      {
        const double duration =
            position_trajectories_[robot_id].getTimeSum();
        for (double time = 0.0; time <= duration; time += 0.05)
        {
          const Eigen::Vector3d velocity =
              velocity_trajectories_[robot_id].evaluateDeBoorT(time);
          if (std::hypot(velocity.x(), velocity.y()) <=
              heading_speed_threshold_)
            continue;

          const double forward_yaw = std::atan2(velocity.y(), velocity.x());
          const double reverse_yaw = normalizeAngle(forward_yaw + kPi);
          const double forward_error =
              std::abs(normalizeAngle(forward_yaw - last_yaws_[robot_id]));
          const double reverse_error =
              std::abs(normalizeAngle(reverse_yaw - last_yaws_[robot_id]));
          reverse_motion_[robot_id] = reverse_error < forward_error;
          break;
        }
      }
      if (!modes.empty())
        modes += ",";
      modes += reverse_motion_[robot_id] ? "reverse" : "forward";
    }
    RCLCPP_INFO(
        get_logger(),
        "JOINT_EXECUTOR_HEADING_CONTINUITY traj_id=%ld modes=[%s]",
        static_cast<long>(pending_trajectory_id_), modes.c_str());
  }

  void updateYaw(int robot_id, double elapsed)
  {
    yaw_rates_[robot_id] = 0.0;
    const auto &velocity = velocities_[robot_id];
    if (std::hypot(velocity.x(), velocity.y()) <= heading_speed_threshold_)
      return;

    double target_yaw = std::atan2(velocity.y(), velocity.x());
    if (reverse_motion_[robot_id])
      target_yaw = normalizeAngle(target_yaw + kPi);
    const double yaw_error =
        normalizeAngle(target_yaw - last_yaws_[robot_id]);
    const double yaw_step =
        std::clamp(yaw_error, -max_yaw_rate_ * elapsed,
                   max_yaw_rate_ * elapsed);
    last_yaws_[robot_id] =
        normalizeAngle(last_yaws_[robot_id] + yaw_step);
    yaw_rates_[robot_id] = yaw_step / elapsed;
  }

  void publishOdometry(int robot_id, const rclcpp::Time &stamp)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.frame_id = frame_id_;
    odometry.header.stamp = stamp;
    odometry.child_frame_id = "robot_" + std::to_string(robot_id) + "/base";
    odometry.pose.pose.position = toPoint(positions_[robot_id]);
    odometry.pose.pose.orientation.w = std::cos(0.5 * last_yaws_[robot_id]);
    odometry.pose.pose.orientation.z = std::sin(0.5 * last_yaws_[robot_id]);
    odometry.twist.twist.linear.x = velocities_[robot_id].x();
    odometry.twist.twist.linear.y = velocities_[robot_id].y();
    odometry.twist.twist.angular.z = yaw_rates_[robot_id];
    odom_publishers_[robot_id]->publish(odometry);
  }

  visualization_msgs::msg::Marker makeBody(
      int robot_id, const rclcpp::Time &stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "legged_robot_body";
    marker.id = robot_id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = toPoint(positions_[robot_id]);
    marker.pose.orientation.w = std::cos(0.5 * last_yaws_[robot_id]);
    marker.pose.orientation.z = std::sin(0.5 * last_yaws_[robot_id]);
    marker.scale.x = body_length_;
    marker.scale.y = body_width_;
    marker.scale.z = body_height_;
    marker.color = robotColor(robot_id);
    return marker;
  }

  void publishBodies(const rclcpp::Time &stamp)
  {
    visualization_msgs::msg::MarkerArray array;
    array.markers.reserve(robot_count_);
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
      array.markers.push_back(makeBody(robot_id, stamp));
    body_pub_->publish(array);
  }

  void publishPlannedPaths()
  {
    visualization_msgs::msg::MarkerArray array;
    array.markers.reserve(robot_count_);
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = now();
      marker.ns = "joint_executed_plan";
      marker.id = robot_id;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.09;
      marker.color = robotColor(robot_id);
      for (double time = 0.0; time <= trajectory_duration_ + 1e-6; time += 0.10)
      {
        Eigen::Vector3d point =
            position_trajectories_[robot_id].evaluateDeBoorT(
                std::min(time, trajectory_duration_));
        point.z() = fixed_z_;
        marker.points.push_back(toPoint(point));
      }
      array.markers.push_back(marker);
    }
    path_pub_->publish(array);
  }

  int robot_count_{4};
  std::string frame_id_{"world"};
  double fixed_z_{0.35};
  double body_length_{0.9};
  double body_width_{0.45};
  double body_height_{0.35};
  double minimum_start_delay_{0.25};
  double max_yaw_rate_{1.2};
  double heading_speed_threshold_{0.03};
  double update_period_{0.02};
  double trajectory_duration_{0.0};
  int64_t pending_trajectory_id_{-1};
  int64_t active_trajectory_id_{-1};
  bool have_active_trajectory_{false};
  bool allow_reverse_motion_{true};
  rclcpp::Time trajectory_start_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_SYSTEM_TIME};

  std::vector<Eigen::Vector3d> positions_;
  std::vector<Eigen::Vector3d> velocities_;
  std::vector<double> last_yaws_;
  std::vector<double> yaw_rates_;
  std::vector<bool> reverse_motion_;
  std::vector<traj_utils::msg::Bspline::SharedPtr> pending_messages_;
  std::vector<ego_planner::UniformBspline> position_trajectories_;
  std::vector<ego_planner::UniformBspline> velocity_trajectories_;
  rclcpp::Subscription<traj_utils::msg::MultiBsplines>::SharedPtr
      trajectory_batch_sub_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_publishers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr body_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CentralizedJointTrajectoryExecutor>());
  rclcpp::shutdown();
  return 0;
}
