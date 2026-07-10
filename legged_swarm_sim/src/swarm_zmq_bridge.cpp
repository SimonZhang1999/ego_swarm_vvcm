#include "swarm.pb.h"

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <traj_utils/msg/bspline.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <zmq.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace
{
uint64_t nowNs()
{
  const auto t = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(t).count();
}

int markerIdFor(const std::string &robot_id)
{
  int value = 0;
  for (char ch : robot_id)
    value = (value * 131 + static_cast<unsigned char>(ch)) & 0x7fffffff;
  return value;
}
}  // namespace

class SwarmZmqBridge : public rclcpp::Node
{
public:
  SwarmZmqBridge()
      : Node("swarm_zmq_bridge"),
        context_(1),
        pub_socket_(context_, zmq::socket_type::pub)
  {
    robot_id_ = declare_parameter<std::string>("robot_id", "robot_0");
    pub_bind_endpoint_ = declare_parameter<std::string>("pub_bind_endpoint", "tcp://*:6200");
    peer_endpoints_ = declare_parameter<std::vector<std::string>>("peer_endpoints", std::vector<std::string>{});
    publish_rate_ = declare_parameter<double>("publish_rate", 5.0);
    safety_radius_ = declare_parameter<double>("safety_radius", 0.7);
    desired_distance_ = declare_parameter<double>("desired_distance", 1.2);
    fixed_z_ = declare_parameter<double>("fixed_z", 0.35);
    peer_obstacle_height_ = declare_parameter<double>("peer_obstacle_height", 0.8);
    peer_obstacle_resolution_ = declare_parameter<double>("peer_obstacle_resolution", 0.2);
    peer_prediction_horizon_ = declare_parameter<double>("peer_prediction_horizon", 3.0);
    peer_predict_trajectory_as_obstacle_ = declare_parameter<bool>("peer_predict_trajectory_as_obstacle", false);
    peer_state_timeout_ = declare_parameter<double>("peer_state_timeout", 1.5);
    include_pointcloud_ = declare_parameter<bool>("include_pointcloud", false);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");

    if (!pub_bind_endpoint_.empty())
      pub_socket_.bind(pub_bind_endpoint_);

    for (const auto &endpoint : peer_endpoints_)
    {
      auto sub = std::make_unique<zmq::socket_t>(context_, zmq::socket_type::sub);
      sub->set(zmq::sockopt::subscribe, "");
      sub->set(zmq::sockopt::linger, 0);
      sub->connect(endpoint);
      sub_sockets_.push_back(std::move(sub));
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom_world", 20,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          latest_odom_ = *msg;
          has_odom_ = true;
        });
    bspline_sub_ = create_subscription<traj_utils::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&SwarmZmqBridge::bsplineCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "world_cloud", 2,
        std::bind(&SwarmZmqBridge::cloudCallback, this, std::placeholders::_1));
    peer_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("peer_state_markers", 10);
    peer_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("peer_obstacles_cloud", 5);

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_)),
        std::bind(&SwarmZmqBridge::timerCallback, this));

    RCLCPP_INFO(get_logger(), "%s ZMQ pub=%s peers=%zu",
                robot_id_.c_str(), pub_bind_endpoint_.c_str(), peer_endpoints_.size());
  }

private:
  void bsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg)
  {
    trajectory_.clear();
    if (msg->pos_pts.empty())
      return;

    const double duration = std::max(0.2, 0.1 * static_cast<double>(msg->pos_pts.size()));
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
    {
      swarm::TrajectoryPoint point;
      point.set_x(msg->pos_pts[i].x);
      point.set_y(msg->pos_pts[i].y);
      point.set_z(msg->pos_pts[i].z);
      point.set_t_from_start(duration * static_cast<double>(i) / std::max<size_t>(1, msg->pos_pts.size() - 1));
      trajectory_.push_back(point);
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!include_pointcloud_)
      return;

    std::vector<float> xyz;
    xyz.reserve(static_cast<size_t>(msg->width) * msg->height * 3);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      xyz.push_back(*iter_x);
      xyz.push_back(*iter_y);
      xyz.push_back(*iter_z);
    }

    const uint32_t count = static_cast<uint32_t>(xyz.size() / 3);
    pointcloud_bin_.resize(sizeof(uint32_t) + xyz.size() * sizeof(float));
    std::memcpy(pointcloud_bin_.data(), &count, sizeof(uint32_t));
    std::memcpy(pointcloud_bin_.data() + sizeof(uint32_t), xyz.data(), xyz.size() * sizeof(float));
  }

  void timerCallback()
  {
    publishLocalState();
    receivePeerStates();
  }

  void publishLocalState()
  {
    if (!has_odom_)
      return;

    swarm::RobotStateAnnounce msg;
    msg.set_robot_id(robot_id_);
    msg.set_timestamp_ns(nowNs());
    msg.set_safety_radius(safety_radius_);
    msg.set_desired_distance(desired_distance_);
    auto *pose = msg.mutable_pose();
    pose->set_x(latest_odom_.pose.pose.position.x);
    pose->set_y(latest_odom_.pose.pose.position.y);
    pose->set_z(latest_odom_.pose.pose.position.z);
    pose->set_qx(latest_odom_.pose.pose.orientation.x);
    pose->set_qy(latest_odom_.pose.pose.orientation.y);
    pose->set_qz(latest_odom_.pose.pose.orientation.z);
    pose->set_qw(latest_odom_.pose.pose.orientation.w);
    for (const auto &point : trajectory_)
      *msg.add_trajectory() = point;
    if (include_pointcloud_ && !pointcloud_bin_.empty())
      msg.set_pointcloud_bin(pointcloud_bin_);

    std::string bytes;
    msg.SerializeToString(&bytes);
    zmq::message_t zmq_msg(bytes.size());
    std::memcpy(zmq_msg.data(), bytes.data(), bytes.size());
    pub_socket_.send(zmq_msg, zmq::send_flags::dontwait);
  }

  void receivePeerStates()
  {
    const uint64_t received_at = nowNs();
    for (auto &socket : sub_sockets_)
    {
      while (true)
      {
        zmq::message_t packet;
        const auto result = socket->recv(packet, zmq::recv_flags::dontwait);
        if (!result)
          break;

        swarm::RobotStateAnnounce state;
        if (!state.ParseFromArray(packet.data(), static_cast<int>(packet.size())) ||
            state.robot_id().empty() || !state.has_pose() || state.robot_id() == robot_id_)
        {
          continue;
        }
        peer_states_[state.robot_id()] = PeerState{state, received_at};
      }
    }

    purgeStalePeers(received_at);

    visualization_msgs::msg::MarkerArray markers;
    peer_points_.clear();
    for (const auto &entry : peer_states_)
    {
      appendPeerMarkers(entry.second.state, markers);
      appendPeerObstaclePoints(entry.second.state);
      appendPointcloudBin(entry.second.state.pointcloud_bin());
    }

    if (!markers.markers.empty())
      peer_marker_pub_->publish(markers);
    publishPeerCloud();
  }

  void appendPeerMarkers(const swarm::RobotStateAnnounce &state, visualization_msgs::msg::MarkerArray &markers)
  {
    const int base_id = markerIdFor(state.robot_id());

    visualization_msgs::msg::Marker body;
    body.header.frame_id = frame_id_;
    body.header.stamp = now();
    body.ns = "zmq_peer_body";
    body.id = base_id;
    body.type = visualization_msgs::msg::Marker::SPHERE;
    body.action = visualization_msgs::msg::Marker::ADD;
    body.pose.position.x = state.pose().x();
    body.pose.position.y = state.pose().y();
    body.pose.position.z = fixed_z_;
    body.pose.orientation.w = 1.0;
    body.scale.x = state.safety_radius() * 2.0;
    body.scale.y = state.safety_radius() * 2.0;
    body.scale.z = peer_obstacle_height_;
    body.color.r = 1.0f;
    body.color.g = 1.0f;
    body.color.b = 0.1f;
    body.color.a = 0.35f;
    markers.markers.push_back(body);

    visualization_msgs::msg::Marker path;
    path.header = body.header;
    path.ns = "zmq_peer_trajectory";
    path.id = base_id + 1;
    path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path.action = visualization_msgs::msg::Marker::ADD;
    path.pose.orientation.w = 1.0;
    path.scale.x = 0.04;
    path.color.r = 1.0f;
    path.color.g = 0.9f;
    path.color.b = 0.1f;
    path.color.a = 0.7f;
    for (const auto &point : state.trajectory())
    {
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = fixed_z_;
      path.points.push_back(p);
    }
    markers.markers.push_back(path);
  }

  void purgeStalePeers(uint64_t now_ns)
  {
    const uint64_t timeout_ns = static_cast<uint64_t>(std::max(0.1, peer_state_timeout_) * 1e9);
    for (auto iter = peer_states_.begin(); iter != peer_states_.end();)
    {
      if (now_ns > iter->second.last_seen_ns && now_ns - iter->second.last_seen_ns > timeout_ns)
        iter = peer_states_.erase(iter);
      else
        ++iter;
    }
  }

  void appendPeerObstaclePoints(const swarm::RobotStateAnnounce &state)
  {
    const double radius = std::max(0.25, state.safety_radius());
    const double z_min = std::max(0.0, fixed_z_ - 0.5 * peer_obstacle_height_);
    const double z_max = fixed_z_ + 0.5 * peer_obstacle_height_;

    appendCylinder(state.pose().x(), state.pose().y(), radius, z_min, z_max);

    if (!peer_predict_trajectory_as_obstacle_)
      return;

    for (const auto &point : state.trajectory())
    {
      if (point.t_from_start() > peer_prediction_horizon_)
        continue;
      appendCylinder(point.x(), point.y(), radius, z_min, z_max);
    }
  }

  void appendCylinder(double cx, double cy, double radius, double z_min, double z_max)
  {
    const double step = std::max(0.05, peer_obstacle_resolution_);
    for (double x = cx - radius; x <= cx + radius + 1e-6; x += step)
    {
      for (double y = cy - radius; y <= cy + radius + 1e-6; y += step)
      {
        const double dx = x - cx;
        const double dy = y - cy;
        if (dx * dx + dy * dy > radius * radius)
          continue;
        for (double z = z_min; z <= z_max + 1e-6; z += step)
        {
          peer_points_.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
        }
      }
    }
  }

  void appendPointcloudBin(const std::string &pointcloud_bin)
  {
    if (pointcloud_bin.size() < sizeof(uint32_t))
      return;

    uint32_t count = 0;
    std::memcpy(&count, pointcloud_bin.data(), sizeof(uint32_t));
    const size_t expected_size = sizeof(uint32_t) + static_cast<size_t>(count) * 3 * sizeof(float);
    if (pointcloud_bin.size() < expected_size)
      return;

    const char *values = pointcloud_bin.data() + sizeof(uint32_t);
    for (uint32_t i = 0; i < count; ++i)
    {
      std::array<float, 3> point{};
      std::memcpy(point.data(), values + static_cast<size_t>(i) * 3 * sizeof(float), 3 * sizeof(float));
      peer_points_.push_back(point);
    }
  }

  void publishPeerCloud()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_;
    cloud.header.stamp = now();
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(peer_points_.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto &point : peer_points_)
    {
      *iter_x = point[0];
      *iter_y = point[1];
      *iter_z = point[2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    peer_cloud_pub_->publish(cloud);
  }

  std::string robot_id_;
  std::string frame_id_;
  std::string pub_bind_endpoint_;
  std::vector<std::string> peer_endpoints_;
  double publish_rate_{5.0};
  double safety_radius_{0.7};
  double desired_distance_{1.2};
  double fixed_z_{0.35};
  double peer_obstacle_height_{0.8};
  double peer_obstacle_resolution_{0.2};
  double peer_prediction_horizon_{3.0};
  bool peer_predict_trajectory_as_obstacle_{false};
  double peer_state_timeout_{1.5};
  bool include_pointcloud_{false};
  bool has_odom_{false};
  nav_msgs::msg::Odometry latest_odom_;
  std::vector<swarm::TrajectoryPoint> trajectory_;
  std::string pointcloud_bin_;
  struct PeerState
  {
    swarm::RobotStateAnnounce state;
    uint64_t last_seen_ns;
  };
  std::map<std::string, PeerState> peer_states_;
  std::vector<std::array<float, 3>> peer_points_;

  zmq::context_t context_;
  zmq::socket_t pub_socket_;
  std::vector<std::unique_ptr<zmq::socket_t>> sub_sockets_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr peer_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr peer_cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwarmZmqBridge>());
  rclcpp::shutdown();
  return 0;
}
