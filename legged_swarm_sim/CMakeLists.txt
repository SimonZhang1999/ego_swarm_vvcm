cmake_minimum_required(VERSION 3.8)
project(legged_swarm_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/swarm.proto)

add_library(swarm_proto STATIC ${PROTO_SRCS})
target_include_directories(swarm_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(swarm_proto PUBLIC protobuf::libprotobuf)

add_library(vvcm_fk STATIC
  third_party/vvcm/src/VVCM_FK.cpp
)
target_include_directories(vvcm_fk PUBLIC
  third_party/vvcm/include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(vvcm_fk PUBLIC Eigen3::Eigen)

add_executable(legged_traj_follower src/legged_traj_follower.cpp)
target_include_directories(legged_traj_follower PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(legged_traj_follower
  rclcpp nav_msgs geometry_msgs visualization_msgs std_msgs traj_utils bspline_opt Eigen3)

add_executable(world_cloud_publisher src/world_cloud_publisher.cpp)
ament_target_dependencies(world_cloud_publisher
  rclcpp sensor_msgs visualization_msgs)

add_executable(cloud_mux src/cloud_mux.cpp)
ament_target_dependencies(cloud_mux
  rclcpp sensor_msgs)

add_executable(formation_goal_relay src/formation_goal_relay.cpp)
target_include_directories(formation_goal_relay PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(formation_goal_relay
  rclcpp geometry_msgs visualization_msgs Eigen3)

add_executable(topology_net_visualizer src/topology_net_visualizer.cpp)
target_include_directories(topology_net_visualizer PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(topology_net_visualizer
  rclcpp nav_msgs geometry_msgs visualization_msgs Eigen3)

add_executable(multi_robot_cloud_fusion src/multi_robot_cloud_fusion.cpp)
target_include_directories(multi_robot_cloud_fusion PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(multi_robot_cloud_fusion
  rclcpp sensor_msgs Eigen3)

add_executable(robot_grid_astar_frontend src/robot_grid_astar_frontend.cpp)
target_include_directories(robot_grid_astar_frontend PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(robot_grid_astar_frontend
  rclcpp nav_msgs geometry_msgs sensor_msgs Eigen3)

add_executable(centralized_trajectory_optimizer src/centralized_trajectory_optimizer.cpp)
target_include_directories(centralized_trajectory_optimizer PRIVATE
  include
  third_party/vvcm/include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(centralized_trajectory_optimizer
  rclcpp nav_msgs geometry_msgs sensor_msgs visualization_msgs std_msgs
  traj_utils bspline_opt Eigen3)
target_link_libraries(centralized_trajectory_optimizer vvcm_fk)

add_executable(centralized_joint_trajectory_executor src/centralized_joint_trajectory_executor.cpp)
target_include_directories(centralized_joint_trajectory_executor PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(centralized_joint_trajectory_executor
  rclcpp nav_msgs geometry_msgs visualization_msgs std_msgs
  traj_utils bspline_opt Eigen3)

add_executable(vvcm_object_pose_node src/vvcm_object_pose_node.cpp)
target_include_directories(vvcm_object_pose_node PRIVATE
  include
  third_party/vvcm/include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(vvcm_object_pose_node
  rclcpp nav_msgs geometry_msgs visualization_msgs std_msgs Eigen3)
target_link_libraries(vvcm_object_pose_node vvcm_fk)

add_executable(swarm_zmq_bridge src/swarm_zmq_bridge.cpp)
target_include_directories(swarm_zmq_bridge PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${ZMQ_INCLUDE_DIRS}
)
ament_target_dependencies(swarm_zmq_bridge
  rclcpp nav_msgs sensor_msgs visualization_msgs traj_utils)
target_link_libraries(swarm_zmq_bridge
  swarm_proto
  ${ZMQ_LIBRARIES}
)

install(TARGETS
  legged_traj_follower
  world_cloud_publisher
  cloud_mux
  formation_goal_relay
  topology_net_visualizer
  multi_robot_cloud_fusion
  robot_grid_astar_frontend
  centralized_trajectory_optimizer
  centralized_joint_trajectory_executor
  vvcm_object_pose_node
  swarm_zmq_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES third_party/vvcm/LICENSE
  DESTINATION share/${PROJECT_NAME}/third_party/vvcm
)

ament_package()
