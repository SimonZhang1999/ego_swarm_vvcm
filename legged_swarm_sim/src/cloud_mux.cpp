#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <array>
#include <string>
#include <vector>

class CloudMux : public rclcpp::Node
{
public:
  CloudMux() : Node("cloud_mux")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    publish_rate_ = declare_parameter<double>("publish_rate", 5.0);

    world_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "world_cloud", 5,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          latest_world_ = *msg;
          has_world_ = true;
        });
    peer_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "peer_obstacles_cloud", 5,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          latest_peer_ = *msg;
          has_peer_ = true;
        });
    merged_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("merged_cloud", 5);

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_)),
        std::bind(&CloudMux::publishMergedCloud, this));
  }

private:
  static void appendPoints(const sensor_msgs::msg::PointCloud2 &cloud, std::vector<std::array<float, 3>> &points)
  {
    if (cloud.width * cloud.height == 0)
      return;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
      points.push_back({*iter_x, *iter_y, *iter_z});
  }

  void publishMergedCloud()
  {
    if (!has_world_ && !has_peer_)
      return;

    std::vector<std::array<float, 3>> points;
    if (has_world_)
      appendPoints(latest_world_, points);
    if (has_peer_)
      appendPoints(latest_peer_, points);

    sensor_msgs::msg::PointCloud2 merged;
    merged.header.frame_id = frame_id_;
    merged.header.stamp = now();
    sensor_msgs::PointCloud2Modifier modifier(merged);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(merged, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(merged, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(merged, "z");
    for (const auto &point : points)
    {
      *iter_x = point[0];
      *iter_y = point[1];
      *iter_z = point[2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    merged_pub_->publish(merged);
  }

  std::string frame_id_{"world"};
  double publish_rate_{5.0};
  bool has_world_{false};
  bool has_peer_{false};
  sensor_msgs::msg::PointCloud2 latest_world_;
  sensor_msgs::msg::PointCloud2 latest_peer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr world_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr peer_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudMux>());
  rclcpp::shutdown();
  return 0;
}
