import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_float(context, name):
    return float(LaunchConfiguration(name).perform(context))


def _as_int(context, name):
    return int(LaunchConfiguration(name).perform(context))


def _formation_offsets(robot_count, spacing, length, width, mode):
    if mode == "rectangle" and robot_count == 4:
        hx = 0.5 * length
        hy = 0.5 * width
        return [(-hx, -hy), (hx, -hy), (-hx, hy), (hx, hy)]
    if mode == "square" and robot_count == 4:
        h = 0.5 * spacing
        return [(-h, -h), (h, -h), (-h, h), (h, h)]
    if mode == "diamond" and robot_count == 4:
        return [(spacing, 0.0), (0.0, spacing), (0.0, -spacing), (-spacing, 0.0)]
    return [(0.0, (float(i) - 0.5 * (robot_count - 1)) * spacing) for i in range(robot_count)]


def launch_setup(context, *args, **kwargs):
    robot_count = _as_int(context, "robot_count")
    spacing = _as_float(context, "formation_spacing")
    formation_length = _as_float(context, "formation_length")
    formation_width = _as_float(context, "formation_width")
    formation_mode = LaunchConfiguration("formation_mode").perform(context)
    goal_z = _as_float(context, "goal_z")
    map_size_x = _as_float(context, "map_size_x")
    map_size_y = _as_float(context, "map_size_y")
    map_size_z = _as_float(context, "map_size_z")
    start_x = _as_float(context, "start_x")
    zmq_base_port = _as_int(context, "zmq_base_port")
    topology_config = LaunchConfiguration("topology_config").perform(context).strip()
    topology_config = os.path.expanduser(topology_config) if topology_config else ""
    offsets = _formation_offsets(robot_count, spacing, formation_length, formation_width, formation_mode)

    use_rviz = LaunchConfiguration("use_rviz")
    enable_zmq = LaunchConfiguration("enable_zmq")

    nodes = [
        Node(
            package="legged_swarm_sim",
            executable="world_cloud_publisher",
            name="world_cloud_publisher",
            output="screen",
            remappings=[
                ("world_cloud", "/world/cloud"),
                ("obstacle_markers", "/world/obstacle_markers"),
            ],
            parameters=[
                {"frame_id": "world"},
                {"resolution": 0.2},
            ],
        ),
        Node(
            package="legged_swarm_sim",
            executable="formation_goal_relay",
            name="formation_goal_relay",
            output="screen",
            parameters=[
                {"robot_count": robot_count},
                {"formation_spacing": spacing},
                {"formation_length": formation_length},
                {"formation_width": formation_width},
                {"formation_mode": formation_mode},
                {"goal_x": LaunchConfiguration("goal_x")},
                {"goal_y": LaunchConfiguration("goal_y")},
                {"goal_z": goal_z},
                {"auto_start": LaunchConfiguration("auto_start")},
                {"auto_start_delay": 2.0},
                {"frame_id": "world"},
            ],
        ),
        Node(
            package="legged_swarm_sim",
            executable="topology_net_visualizer",
            name="topology_net_visualizer",
            output="screen",
            parameters=[
                {"robot_count": robot_count},
                {"frame_id": "world"},
                {"publish_rate": 20.0},
            ],
        ),
    ]

    rviz_config = os.path.join(
        get_package_share_directory("legged_swarm_sim"),
        "rviz",
        "legged_swarm.rviz",
    )
    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["--display-config", rviz_config],
            condition=IfCondition(use_rviz),
        )
    )

    for i in range(robot_count):
        init_offset_x, init_offset_y = offsets[i]
        init_x = start_x + init_offset_x
        init_y = init_offset_y
        robot_name = f"robot_{i}"

        nodes.append(
            Node(
                package="legged_swarm_sim",
                executable="cloud_mux",
                name=f"{robot_name}_cloud_mux",
                output="screen",
                remappings=[
                    ("world_cloud", "/world/cloud"),
                    ("peer_obstacles_cloud", f"/{robot_name}/peer_obstacles_cloud"),
                    ("merged_cloud", f"/{robot_name}/grid/input_cloud"),
                ],
                parameters=[
                    {"frame_id": "world"},
                    {"publish_rate": 5.0},
                ],
            )
        )

        planner_params = [
            {"fsm/flight_type": 1},
            {"fsm/thresh_replan_time": 1.0},
            {"fsm/thresh_no_replan_meter": 1.0},
            {"fsm/planning_horizon": 8.0},
            {"fsm/planning_horizen_time": 3.0},
            {"fsm/emergency_time": 1.0},
            {"fsm/default_goal_z": goal_z},
            {"fsm/planar_mode": True},
            {"fsm/realworld_experiment": False},
            {"fsm/fail_safe": True},
            {"fsm/waypoint_num": 1},
            {"fsm/waypoint0_x": LaunchConfiguration("goal_x")},
            {"fsm/waypoint0_y": LaunchConfiguration("goal_y")},
            {"fsm/waypoint0_z": goal_z},
            {"grid_map/resolution": 0.1},
            {"grid_map/map_size_x": map_size_x},
            {"grid_map/map_size_y": map_size_y},
            {"grid_map/map_size_z": map_size_z},
            {"grid_map/local_update_range_x": 10.0},
            {"grid_map/local_update_range_y": 10.0},
            {"grid_map/local_update_range_z": 1.0},
            {"grid_map/obstacles_inflation": 0.35},
            {"grid_map/local_map_margin": 10},
            {"grid_map/ground_height": -0.05},
            {"grid_map/cx": 321.0},
            {"grid_map/cy": 243.0},
            {"grid_map/fx": 387.0},
            {"grid_map/fy": 387.0},
            {"grid_map/use_depth_filter": False},
            {"grid_map/depth_filter_tolerance": 0.15},
            {"grid_map/depth_filter_maxdist": 5.0},
            {"grid_map/depth_filter_mindist": 0.2},
            {"grid_map/depth_filter_margin": 2},
            {"grid_map/k_depth_scaling_factor": 1000.0},
            {"grid_map/skip_pixel": 2},
            {"grid_map/p_hit": 0.65},
            {"grid_map/p_miss": 0.35},
            {"grid_map/p_min": 0.12},
            {"grid_map/p_max": 0.90},
            {"grid_map/p_occ": 0.80},
            {"grid_map/min_ray_length": 0.1},
            {"grid_map/max_ray_length": 6.0},
            {"grid_map/virtual_ceil_height": 0.65},
            {"grid_map/visualization_truncate_height": 0.75},
            {"grid_map/show_occ_time": False},
            {"grid_map/pose_type": 1},
            {"grid_map/frame_id": "world"},
            {"manager/max_vel": 1.2},
            {"manager/max_acc": 1.8},
            {"manager/max_jerk": 4.0},
            {"manager/control_points_distance": 0.35},
            {"manager/feasibility_tolerance": 0.05},
            {"manager/planning_horizon": 8.0},
            {"manager/use_distinctive_trajs": False},
            {"manager/drone_id": i},
            {"optimization/lambda_smooth": 1.0},
            {"optimization/lambda_collision": 0.7},
            {"optimization/lambda_feasibility": 0.1},
            {"optimization/lambda_fitness": 1.0},
            {"optimization/dist0": 0.7},
            {"optimization/swarm_clearance": 0.9},
            {"optimization/max_vel": 1.2},
            {"optimization/max_acc": 1.8},
            {"optimization/planar_mode": True},
            {"optimization/fixed_z": goal_z},
            {"optimization/formation_topology/enabled": LaunchConfiguration("formation_topology_enabled")},
            {"optimization/formation_topology/weight": LaunchConfiguration("formation_topology_weight")},
            {"optimization/formation_topology/edge_weight": LaunchConfiguration("formation_topology_edge_weight")},
            {"optimization/formation_topology/diag_weight": LaunchConfiguration("formation_topology_diag_weight")},
            {"optimization/formation_topology/length": formation_length},
            {"optimization/formation_topology/width": formation_width},
            {"bspline/limit_vel": 1.2},
            {"bspline/limit_acc": 1.8},
            {"bspline/limit_ratio": 1.1},
            {"prediction/obj_num": 0},
            {"prediction/lambda": 1.0},
            {"prediction/predict_rate": 1.0},
        ]
        planner_param_sources = list(planner_params)
        if topology_config:
            planner_param_sources.append(topology_config)

        nodes.append(
            Node(
                package="ego_planner",
                executable="ego_planner_node",
                name=f"{robot_name}_ego_planner",
                output="screen",
                remappings=[
                    ("odom_world", f"/{robot_name}/odom_world"),
                    ("/move_base_simple/goal", f"/{robot_name}/goal"),
                    ("planning/bspline", f"/drone_{i}_planning/bspline"),
                    ("planning/data_display", f"/drone_{i}_planning/data_display"),
                    ("planning/broadcast_bspline_from_planner", "/broadcast_bspline"),
                    ("planning/broadcast_bspline_to_planner", "/broadcast_bspline"),
                    ("goal_point", "/swarm/planner_goal_points"),
                    ("global_list", "/swarm/global_paths"),
                    ("init_list", "/swarm/init_paths"),
                    ("optimal_list", "/swarm/optimal_paths"),
                    ("a_star_list", "/swarm/a_star_paths"),
                    ("grid_map/odom", f"/{robot_name}/odom_world"),
                    ("grid_map/cloud", f"/{robot_name}/grid/input_cloud"),
                    ("grid_map/occupancy_inflate", f"/{robot_name}/grid/occupancy_inflate"),
                    ("grid_map/occupancy", f"/{robot_name}/grid/occupancy"),
                ],
                parameters=planner_param_sources,
            )
        )

        nodes.append(
            Node(
                package="legged_swarm_sim",
                executable="legged_traj_follower",
                name=f"{robot_name}_traj_follower",
                output="screen",
                remappings=[
                    ("planning/bspline", f"/drone_{i}_planning/bspline"),
                    ("odom_world", f"/{robot_name}/odom_world"),
                    ("robot_body_marker", "/swarm/robot_bodies"),
                    ("planned_path_marker", "/swarm/robot_planned_paths"),
                ],
                parameters=[
                    {"robot_id": i},
                    {"initial_x": init_x},
                    {"initial_y": init_y},
                    {"initial_z": goal_z},
                    {"planar_mode": True},
                    {"fixed_z": goal_z},
                    {"body_length": 0.9},
                    {"body_width": 0.45},
                    {"body_height": 0.35},
                    {"odom_rate": 50.0},
                    {"frame_id": "world"},
                ],
            )
        )

        peers = [
            f"tcp://127.0.0.1:{zmq_base_port + j}"
            for j in range(robot_count)
            if j != i
        ]
        nodes.append(
            Node(
                package="legged_swarm_sim",
                executable="swarm_zmq_bridge",
                name=f"{robot_name}_zmq_bridge",
                output="screen",
                condition=IfCondition(enable_zmq),
                remappings=[
                    ("odom_world", f"/{robot_name}/odom_world"),
                    ("planning/bspline", f"/drone_{i}_planning/bspline"),
                    ("world_cloud", "/world/cloud"),
                    ("peer_state_markers", "/swarm/zmq_peer_state_markers"),
                    ("peer_obstacles_cloud", f"/{robot_name}/peer_obstacles_cloud"),
                ],
                parameters=[
                    {"robot_id": robot_name},
                    {"pub_bind_endpoint": f"tcp://*:{zmq_base_port + i}"},
                    {"peer_endpoints": peers},
                    {"publish_rate": 5.0},
                    {"safety_radius": 0.55},
                    {"desired_distance": spacing},
                    {"fixed_z": goal_z},
                    {"peer_obstacle_height": 0.8},
                    {"peer_obstacle_resolution": 0.2},
                    {"peer_prediction_horizon": 3.0},
                    {"peer_predict_trajectory_as_obstacle": LaunchConfiguration("peer_predict_trajectory_as_obstacle")},
                    {"peer_state_timeout": 1.5},
                    {"include_pointcloud": LaunchConfiguration("zmq_send_pointcloud")},
                    {"frame_id": "world"},
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_count", default_value="4"),
            DeclareLaunchArgument("formation_spacing", default_value="2.2"),
            DeclareLaunchArgument("formation_length", default_value="2.4"),
            DeclareLaunchArgument("formation_width", default_value="2.0"),
            DeclareLaunchArgument("formation_mode", default_value="rectangle"),
            DeclareLaunchArgument("start_x", default_value="-8.0"),
            DeclareLaunchArgument("goal_x", default_value="8.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("goal_z", default_value="0.35"),
            DeclareLaunchArgument("map_size_x", default_value="24.0"),
            DeclareLaunchArgument("map_size_y", default_value="16.0"),
            DeclareLaunchArgument("map_size_z", default_value="1.0"),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("enable_zmq", default_value="true"),
            DeclareLaunchArgument("zmq_send_pointcloud", default_value="false"),
            DeclareLaunchArgument("peer_predict_trajectory_as_obstacle", default_value="false"),
            DeclareLaunchArgument("formation_topology_enabled", default_value="false"),
            DeclareLaunchArgument("formation_topology_weight", default_value="0.35"),
            DeclareLaunchArgument("formation_topology_edge_weight", default_value="1.0"),
            DeclareLaunchArgument("formation_topology_diag_weight", default_value="0.25"),
            DeclareLaunchArgument("topology_config", default_value=""),
            DeclareLaunchArgument("zmq_base_port", default_value="6200"),
            OpaqueFunction(function=launch_setup),
        ]
    )
