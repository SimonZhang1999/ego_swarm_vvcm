# Ego-Planner-Swarm-VVCM 

This is a centralized formation trajectory planning project for multiple nonholonomic robots collaboratively transport objects using a deformable sheet in unstructured environments.

<p align="center">
  <img src="https://github.com/SimonZhang1999/ego_swarm_vvcm/blob/main/memo/rviz_demo.gif" alt="rviz_demo" width="800">
</p>

Currently, the project mainly supports:

- Automatic configuration of formations, topics, A* front-ends, centralized optimization variables, and RViz visualization for 3, 4, and 5 robots.
- An independent A* front-end for each robot, with a centralized back-end that synchronously optimizes all robot trajectories.
- Sheet-size constraints, topological directed-distance constraints, inter-robot collision avoidance, obstacle avoidance, and velocity/acceleration constraints.
- VVCM-based object position estimation for any number of robots, with rods, mesh, connection lines, and the object sphere visualized in RViz.
- Object obstacle-clearance iteration: if the object inside the sheet does not have enough height clearance when crossing obstacles, the corresponding lower bounds of edge distances are automatically increased and the optimizer is re-run.
- A reserved ZMQ/protobuf communication bridge for future integration with the localization/point-cloud host and the real robot velocity-control chain.

## 1. Environment Dependencies

Recommended environment:

- Ubuntu 22.04
- ROS 2 Humble
- C++17 compiler
- `colcon`
- Eigen3
- Protobuf
- ZeroMQ
- RViz2

Install common dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  libeigen3-dev \
  libprotobuf-dev protobuf-compiler \
  libzmq3-dev
```

If ROS 2 has already been installed, you can install only the missing development libraries:

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  libeigen3-dev \
  libprotobuf-dev protobuf-compiler \
  libzmq3-dev
```

## 2. Clone the Code

```bash
cd ~
git clone git@github.com:<your_name>/<your_repo>.git legged_swarm_planner_centralized
cd ~/legged_swarm_planner_centralized
```

After publishing the project to GitHub, replace `<your_name>/<your_repo>` with the actual repository address.

## 3. Build

```bash
cd ~/legged_swarm_planner_centralized
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build only the newly added simulation/centralized planning package:

```bash
colcon build --symlink-install --packages-select legged_swarm_sim
source install/setup.bash
```

## 4. Run in RViz

Launch the default 4-robot rectangular formation:

```bash
cd ~/legged_swarm_planner_centralized
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch legged_swarm_sim centralized_swarm_demo.launch.py
```

Use `2D Goal Pose` in RViz to set a target point. The target position will be used as the formation center, and all robots will plan synchronously and move together along the joint trajectory.

Select the number of robots:

```bash
ros2 launch legged_swarm_sim centralized_swarm_demo.launch.py robot_count:=3
ros2 launch legged_swarm_sim centralized_swarm_demo.launch.py robot_count:=4
ros2 launch legged_swarm_sim centralized_swarm_demo.launch.py robot_count:=5
```

Run without RViz and automatically assign a target point for quick planning tests:

```bash
ros2 launch legged_swarm_sim centralized_swarm_demo.launch.py \
  use_rviz:=false auto_start:=true enable_zmq:=false
```

Increase execution speed:

```bash
ros2 launch legged_swarm_sim centralized_swarm_demo.launch.py \
  max_velocity:=2.5 max_acceleration:=5.0
```

Disable topological constraints:

```bash
ros2 launch legged_swarm_sim centralized_swarm_demo.launch.py \
  topology_enabled:=false
```

Disable object obstacle-clearance iteration:

```bash
ros2 launch legged_swarm_sim centralized_swarm_demo.launch.py \
  object_clearance_enabled:=false
```

## 5. Parameter Locations

Main launch parameters are located in:

```text
src/legged_swarm_sim/launch/centralized_swarm_demo.launch.py
```

Centralized optimization parameters are located in:

```text
src/legged_swarm_sim/config/centralized_optimizer.yaml
```

VVCM/object visualization parameters are located in:

```text
src/legged_swarm_sim/config/vvcm.yaml
```

Common parameters:

| Parameter | Default | Description |
|---|---:|---|
| `robot_count` | `4` | Number of robots; currently supports 3, 4, and 5 |
| `formation_length` | `2.4` | Initial formation length |
| `formation_width` | `2.0` | Initial formation width |
| `sheet_length` | `3.8` | Physical mesh/sheet length |
| `sheet_width` | `3.4` | Physical mesh/sheet width |
| `sheet_distance_margin` | `0.10` | Safety margin for the distance upper bound |
| `topology_enabled` | `true` | Whether to enable topological directed-distance constraints |
| `topology_min_directed_distance` | `0.35` | Target distance for the soft topological constraint |
| `topology_hard_min_directed_distance` | `0.08` | Hard safety lower bound for topology |
| `object_clearance_enabled` | `true` | Whether to enable object obstacle-clearance iteration |
| `rod_height` | `2.4` | Length of rods above the robots |
| `object_diameter` | `0.45` | Diameter of the central object sphere |
| `object_obstacle_clearance` | `0.10` | Minimum bottom clearance of the object relative to obstacles |
| `object_distance_increment` | `0.05` | Edge-distance lower-bound increment in each iteration |
| `object_clearance_max_iterations` | `30` | Maximum number of outer iterations for object clearance |
| `object_visual_fallback_sag` | `2.0` | Calibrated sagging amount for the 3/5-robot geometric sagging model |
| `max_velocity` | `2.0` | Maximum trajectory velocity |
| `max_acceleration` | `4.0` | Maximum trajectory acceleration |
| `sample_count` | `48` | Number of synchronized trajectory samples |

Example: increase the mesh size and the rod length for object-height visualization:

```bash
ros2 launch legged_swarm_sim centralized_swarm_demo.launch.py \
  sheet_length:=4.2 sheet_width:=3.8 rod_height:=2.6
```

## 6. Planning Framework

The current centralized version follows this pipeline:

1. `world_cloud_publisher` publishes the known obstacle point cloud in the world frame.
2. `multi_robot_cloud_fusion` fuses the world cloud and the world-frame point cloud from each robot.
3. Each robot runs its own `robot_grid_astar_frontend` and publishes `/robot_i/frontend_path`.
4. `centralized_trajectory_optimizer` waits for all front-end paths corresponding to the same target timestamp.
5. The back-end resamples all robot trajectories to the same number of points and jointly optimizes them in a single L-BFGS problem.
6. After successful optimization, it publishes `/swarm/centralized_bspline_batch`.
7. `centralized_joint_trajectory_executor` executes all robot trajectories synchronously using the same clock.
8. RViz displays the cuboid robot bodies, optimized trajectories, topology net, rods, mesh surface, object sphere, and connection lines.

The front-end and back-end are decoupled through `nav_msgs/msg/Path`. If the A* front-end needs to be replaced or heavily modified later, each robot only needs to keep publishing:

```text
/robot_i/frontend_path
```

and the path `header.stamp` must match the corresponding goal timestamp.

## 7. Main ROS Topics

| Topic | Type / Purpose |
|---|---|
| `/world/cloud` | Obstacle point cloud in the world frame |
| `/swarm/fused_cloud` | Planning point cloud after multi-robot fusion |
| `/robot_i/odom_world` | World-frame odometry of robot i |
| `/robot_i/goal` | Goal point of robot i |
| `/robot_i/frontend_path` | A* front-end path of robot i |
| `/robot_i/centralized_path` | Optimized synchronized path of robot i |
| `/drone_i_planning/bspline` | B-spline output compatible with the original EGO trajectory message |
| `/swarm/centralized_bspline_batch` | Synchronized B-spline batch for all robots |
| `/swarm/centralized_plan` | Centralized trajectory markers |
| `/swarm/centralized_robot_bodies` | Cuboid robot body markers |
| `/swarm/topology_net` | Topology-net visualization |
| `/swarm/vvcm_markers` | Visualization of rods, mesh, connection lines, and object sphere |
| `/swarm/vvcm_object_pose` | VVCM object pose for the 4-robot case |
| `/swarm/vvcm_status` | VVCM/geometric-model status text |

## 8. Real-Robot Communication Interface

During real deployment, the planning host needs two types of communication interfaces:

1. From the perception/localization host to the planning host: receive each robot's world-frame odometry and world-frame point cloud.
2. From the planning host to the robot dogs: send velocity commands or desired states required for trajectory tracking.

A ZMQ/protobuf bridge is reserved in the current project:

```text
src/legged_swarm_sim/src/swarm_zmq_bridge.cpp
src/legged_swarm_sim/proto/swarm.proto
```

Real-robot interface specification:

```text
REAL_ROBOT_COMMUNICATION_SPEC.txt
```

Note: the simulation demo uses `enable_zmq:=false` by default. During real deployment, the simulated world-cloud publisher and the simulated executor's odometry override should be disabled, and the system should instead use real localization feedback and real robot velocity control.

## 9. Acknowledgements

This project is based on and refers to the following open-source projects and works:

- [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm): provides the EGO-Swarm/EGO-Planner planning framework, B-spline trajectory representation, and swarm planning foundation.
- [MorningFrog/VVCM](https://github.com/MorningFrog/VVCM): provides a VVCM forward-kinematics implementation for estimating the 3D position of an object in a flexible mesh/sheet from four robot connection points. This project vendors its source code into `src/legged_swarm_sim/third_party/vvcm` and preserves its Apache-2.0 license.
- IEEE paper: [document 11128313](https://ieeexplore.ieee.org/abstract/document/11128313). This project is inspired by ideas from this paper.

If you use the EGO-Swarm/EGO-Planner or VVCM-related parts of this project, please also comply with the licenses of the corresponding projects and cite the original authors' work in your paper or project.

## 10. Citation

If you find this work useful, please cite ([paper](https://ieeexplore.ieee.org/document/11128313)):

```bibtex
@INPROCEEDINGS{11128313,
  author={Zhang, Weijian and Street, Charlie and Mansouri, Masoumeh},
  booktitle={2025 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Multi-Nonholonomic Robot Object Transportation with Obstacle Crossing Using a Deformable Sheet}, 
  year={2025},
  volume={},
  number={},
  pages={7349-7355},
  keywords={Limiting;Navigation;Transportation;Probabilistic logic;Hardware;Planning;Iterative methods;Robots;Trajectory optimization;Contracts},
  doi={10.1109/ICRA55743.2025.11128313}}
```
