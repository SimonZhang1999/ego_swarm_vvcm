#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <sstream>
#include <string>
#include <vector>

namespace
{
struct Box
{
  double x;
  double y;
  double z;
  double sx;
  double sy;
  double sz;
};

std::vector<double> parseNumbers(const std::string &text)
{
  std::string cleaned = text;
  for (char &ch : cleaned)
  {
    if (ch == ',' || ch == ';')
      ch = ' ';
  }
  std::istringstream stream(cleaned);
  std::vector<double> values;
  double value = 0.0;
  while (stream >> value)
    values.push_back(value);
  return values;
}

std::vector<Box> parseBoxes(const std::string &text)
{
  const auto values = parseNumbers(text);
  std::vector<Box> boxes;
  for (size_t i = 0; i + 5 < values.size(); i += 6)
  {
    boxes.push_back({values[i], values[i + 1], values[i + 2], values[i + 3], values[i + 4], values[i + 5]});
  }
  return boxes;
}
}  // namespace

class WorldCloudPublisher : public rclcpp::Node
{
public:
  WorldCloudPublisher() : Node("world_cloud_publisher")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    resolution_ = declare_parameter<double>("resolution", 0.2);
    publish_rate_ = declare_parameter<double>("publish_rate", 2.0);
    const std::string default_boxes =
        "-4.8,-2.9,0.45,0.45,1.6,0.9; "
        "-4.8,2.9,0.45,0.45,1.6,0.9; "
        "-1.2,-3.5,0.45,0.45,1.7,0.9; "
        "-1.2,3.5,0.45,0.45,1.7,0.9; "
        "2.2,-2.6,0.45,0.45,1.5,0.9; "
        "2.2,2.6,0.45,0.45,1.5,0.9; "
        "5.3,0.0,0.45,0.45,2.0,0.9; "
        "0.0,-6.4,0.45,13.0,0.35,0.9; "
        "0.0,6.4,0.45,13.0,0.35,0.9";
    boxes_ = parseBoxes(declare_parameter<std::string>("obstacle_boxes", default_boxes));

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 2);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 2);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_)),
        std::bind(&WorldCloudPublisher::timerCallback, this));

    buildCloud();
    RCLCPP_INFO(get_logger(), "world cloud has %zu obstacle boxes and %zu sampled points",
                boxes_.size(), points_.size());
  }

private:
  void buildCloud()
  {
    points_.clear();
    const double step = std::max(0.05, resolution_);
    for (const auto &box : boxes_)
    {
      const double min_x = box.x - box.sx * 0.5;
      const double max_x = box.x + box.sx * 0.5;
      const double min_y = box.y - box.sy * 0.5;
      const double max_y = box.y + box.sy * 0.5;
      const double min_z = box.z - box.sz * 0.5;
      const double max_z = box.z + box.sz * 0.5;

      for (double x = min_x; x <= max_x + 1e-6; x += step)
      {
        for (double y = min_y; y <= max_y + 1e-6; y += step)
        {
          for (double z = min_z; z <= max_z + 1e-6; z += step)
          {
            const bool surface = x <= min_x + step || x >= max_x - step ||
                                 y <= min_y + step || y >= max_y - step ||
                                 z <= min_z + step || z >= max_z - step;
            if (surface)
              points_.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
          }
        }
      }
    }
  }

  void timerCallback()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_;
    cloud.header.stamp = now();
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points_.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto &point : points_)
    {
      *iter_x = point[0];
      *iter_y = point[1];
      *iter_z = point[2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    cloud_pub_->publish(cloud);

    visualization_msgs::msg::MarkerArray markers;
    for (size_t i = 0; i < boxes_.size(); ++i)
    {
      visualization_msgs::msg::Marker marker;
      marker.header = cloud.header;
      marker.ns = "world_obstacles";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = boxes_[i].x;
      marker.pose.position.y = boxes_[i].y;
      marker.pose.position.z = boxes_[i].z;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = boxes_[i].sx;
      marker.scale.y = boxes_[i].sy;
      marker.scale.z = boxes_[i].sz;
      marker.color.r = 0.45f;
      marker.color.g = 0.45f;
      marker.color.b = 0.45f;
      marker.color.a = 0.55f;
      markers.markers.push_back(marker);
    }
    marker_pub_->publish(markers);
  }

  std::string frame_id_{"world"};
  double resolution_{0.2};
  double publish_rate_{2.0};
  std::vector<Box> boxes_;
  std::vector<std::array<float, 3>> points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
