#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class MultiRobotCloudFusion : public rclcpp::Node
{
public:
  MultiRobotCloudFusion() : Node("multi_robot_cloud_fusion")
  {
    robot_count_ = declare_parameter<int>("robot_count", 4);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    cloud_timeout_ = declare_parameter<double>("cloud_timeout", 2.0);
    const double publish_rate = declare_parameter<double>("publish_rate", 5.0);

    robot_clouds_.resize(robot_count_);
    robot_cloud_stamps_.reserve(robot_count_);
    for (int i = 0; i < robot_count_; ++i)
      robot_cloud_stamps_.emplace_back(0, 0, get_clock()->get_clock_type());

    world_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "world_cloud", rclcpp::QoS(2),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
          if (acceptFrame(*msg))
            world_points_ = readPoints(*msg);
        });
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      robot_cloud_subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
          "/robot_" + std::to_string(robot_id) + "/cloud_world",
          rclcpp::SensorDataQoS(),
          [this, robot_id](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
          {
            if (!acceptFrame(*msg))
              return;
            robot_clouds_[robot_id] = readPoints(*msg);
            robot_cloud_stamps_[robot_id] = now();
          }));
    }

    fused_cloud_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/swarm/fused_cloud", 2);
    const auto period = std::chrono::duration<double>(1.0 / std::max(0.5, publish_rate));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&MultiRobotCloudFusion::publishFusedCloud, this));
  }

private:
  bool acceptFrame(const sensor_msgs::msg::PointCloud2 &cloud)
  {
    if (cloud.header.frame_id.empty() || cloud.header.frame_id == frame_id_)
      return true;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "skip cloud in frame '%s'; expected world-frame cloud '%s'",
        cloud.header.frame_id.c_str(), frame_id_.c_str());
    return false;
  }

  std::vector<Eigen::Vector3f> readPoints(
      const sensor_msgs::msg::PointCloud2 &cloud) const
  {
    std::vector<Eigen::Vector3f> points;
    points.reserve(cloud.width * cloud.height);
    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
      {
        if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z))
          points.emplace_back(*iter_x, *iter_y, *iter_z);
      }
    }
    catch (const std::runtime_error &error)
    {
      RCLCPP_ERROR(get_logger(), "invalid PointCloud2: %s", error.what());
    }
    return points;
  }

  void publishFusedCloud()
  {
    size_t point_count = world_points_.size();
    const auto current_time = now();
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      if (robot_cloud_stamps_[robot_id].nanoseconds() > 0 &&
          (current_time - robot_cloud_stamps_[robot_id]).seconds() <= cloud_timeout_)
        point_count += robot_clouds_[robot_id].size();
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_;
    cloud.header.stamp = current_time;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(point_count);
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    const auto append_points =
        [&iter_x, &iter_y, &iter_z](const std::vector<Eigen::Vector3f> &points)
        {
          for (const auto &point : points)
          {
            *iter_x = point.x();
            *iter_y = point.y();
            *iter_z = point.z();
            ++iter_x;
            ++iter_y;
            ++iter_z;
          }
        };
    append_points(world_points_);
    for (int robot_id = 0; robot_id < robot_count_; ++robot_id)
    {
      if (robot_cloud_stamps_[robot_id].nanoseconds() > 0 &&
          (current_time - robot_cloud_stamps_[robot_id]).seconds() <= cloud_timeout_)
        append_points(robot_clouds_[robot_id]);
    }
    fused_cloud_pub_->publish(cloud);
  }

  int robot_count_{4};
  std::string frame_id_{"world"};
  double cloud_timeout_{2.0};
  std::vector<Eigen::Vector3f> world_points_;
  std::vector<std::vector<Eigen::Vector3f>> robot_clouds_;
  std::vector<rclcpp::Time> robot_cloud_stamps_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr world_cloud_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      robot_cloud_subs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiRobotCloudFusion>());
  rclcpp::shutdown();
  return 0;
}
