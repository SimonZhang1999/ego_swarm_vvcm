#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <legged_swarm_sim/formation_geometry.hpp>

#include <chrono>
#include <string>
#include <utility>
#include <vector>

class TopologyNetVisualizer : public rclcpp::Node
{
public:
  TopologyNetVisualizer() : Node("topology_net_visualizer")
  {
    robot_count_ = declare_parameter<int>("robot_count", 4);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);
    legged_swarm_sim::validateRobotCount(robot_count_);

    poses_.resize(robot_count_);
    has_pose_.assign(robot_count_, false);
    for (int i = 0; i < robot_count_; ++i)
    {
      odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
          "/robot_" + std::to_string(i) + "/odom_world", 10,
          [this, i](nav_msgs::msg::Odometry::SharedPtr msg) {
            poses_[i] = msg->pose.pose.position;
            has_pose_[i] = true;
          }));
    }

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/swarm/topology_net", 10);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_)),
        std::bind(&TopologyNetVisualizer::publishMarker, this));
  }

private:
  static double signedArea2(const geometry_msgs::msg::Point &a,
                            const geometry_msgs::msg::Point &b,
                            const geometry_msgs::msg::Point &c)
  {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  }

  void publishMarker()
  {
    for (bool has_pose : has_pose_)
    {
      if (!has_pose)
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "topology_net";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.06;
    marker.color.r = 0.1f;
    marker.color.g = 0.75f;
    marker.color.b = 1.0f;
    marker.color.a = 0.95f;

    bool topology_safe = true;
    for (const auto &triple :
         legged_swarm_sim::topologyTriples(robot_count_))
    {
      if (signedArea2(
              poses_[triple[0]], poses_[triple[1]],
              poses_[triple[2]]) <= 0.0)
      {
        topology_safe = false;
        break;
      }
    }
    if (!topology_safe)
    {
      marker.color.r = 1.0f;
      marker.color.g = 0.15f;
      marker.color.b = 0.1f;
    }

    for (const auto &link : legged_swarm_sim::allPairs(robot_count_))
    {
      marker.points.push_back(poses_[link.first]);
      marker.points.push_back(poses_[link.second]);
    }
    marker_pub_->publish(marker);
  }

  int robot_count_{4};
  std::string frame_id_{"world"};
  double publish_rate_{20.0};
  std::vector<geometry_msgs::msg::Point> poses_;
  std::vector<bool> has_pose_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopologyNetVisualizer>());
  rclcpp::shutdown();
  return 0;
}
