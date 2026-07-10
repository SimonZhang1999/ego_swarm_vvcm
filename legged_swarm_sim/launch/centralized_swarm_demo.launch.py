import math
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


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        "true",
        "1",
        "yes",
        "on",
    )


def _formation_offsets(robot_count, length, width):
    if robot_count == 4:
        return [
            (-0.5 * length, -0.5 * width),
            (0.5 * length, -0.5 * width),
            (-0.5 * length, 0.5 * width),
            (0.5 * length, 0.5 * width),
        ]

    return [
        (
            0.5
            * length
            * math.cos(-0.5 * math.pi + 2.0 * math.pi * robot_id / robot_count),
            0.5
            * width
            * math.sin(-0.5 * math.pi + 2.0 * math.pi * robot_id / robot_count),
        )
        for robot_id in range(robot_count)
    ]


def launch_setup(context, *args, **kwargs):
    robot_count = _as_int(context, "robot_count")
    if robot_count < 3 or robot_count > 5:
        raise RuntimeError("centralized demo supports robot_count from 3 to 5")

    formation_length = _as_float(context, "formation_length")
    formation_width = _as_float(context, "formation_width")
    goal_z = _as_float(context, "goal_z")
    start_x = _as_float(context, "start_x")
    map_size_x = _as_float(context, "map_size_x")
    map_size_y = _as_float(context, "map_size_y")
    map_resolution = _as_float(context, "map_resolution")
    goal_boundary_margin = _as_float(context, "goal_boundary_margin")
    max_velocity = _as_float(context, "max_velocity")
    max_acceleration = _as_float(context, "max_acceleration")
    max_yaw_rate = _as_float(context, "max_yaw_rate")
    sheet_length = _as_float(context, "sheet_length")
    sheet_width = _as_float(context, "sheet_width")
    sheet_distance_margin = _as_float(context, "sheet_distance_margin")
    rod_base_offset_z = _as_float(context, "rod_base_offset_z")
    rod_height = _as_float(context, "rod_height")
    object_diameter = _as_float(context, "object_diameter")
    object_footprint_radius = _as_float(context, "object_footprint_radius")
    object_obstacle_clearance = _as_float(context, "object_obstacle_clearance")
    object_distance_increment = _as_float(context, "object_distance_increment")
    object_clearance_max_iterations = _as_int(
        context, "object_clearance_max_iterations"
    )
    object_clearance_enabled = _as_bool(context, "object_clearance_enabled")
    vvcm_visualization_enabled = _as_bool(context, "enable_vvcm")
    vvcm_solver_enabled = (
        robot_count == 4 and _as_bool(context, "enable_vvcm_solver")
    )
    zmq_base_port = _as_int(context, "zmq_base_port")
    optimizer_config = LaunchConfiguration("optimizer_config").perform(context).strip()
    optimizer_config = os.path.expanduser(optimizer_config) if optimizer_config else ""
    vvcm_config = LaunchConfiguration("vvcm_config").perform(context).strip()
    vvcm_config = os.path.expanduser(vvcm_config) if vvcm_config else ""
    offsets = _formation_offsets(
        robot_count, formation_length, formation_width
    )

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
            executable="multi_robot_cloud_fusion",
            name="multi_robot_cloud_fusion",
            output="screen",
            remappings=[("world_cloud", "/world/cloud")],
            parameters=[
                {"robot_count": robot_count},
                {"frame_id": "world"},
                {"cloud_timeout": 2.0},
                {"publish_rate": 5.0},
            ],
        ),
        Node(
            package="legged_swarm_sim",
            executable="formation_goal_relay",
            name="formation_goal_relay",
            output="screen",
            parameters=[
                {"robot_count": robot_count},
                {"formation_mode": "auto"},
                {"formation_length": formation_length},
                {"formation_width": formation_width},
                {"rotate_with_goal_yaw": LaunchConfiguration("rotate_formation_with_goal_yaw")},
                {"goal_x": LaunchConfiguration("goal_x")},
                {"goal_y": LaunchConfiguration("goal_y")},
                {"goal_z": goal_z},
                {"map_size_x": map_size_x},
                {"map_size_y": map_size_y},
                {"goal_boundary_margin": goal_boundary_margin},
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

    optimizer_parameters = []
    if optimizer_config:
        optimizer_parameters.append(optimizer_config)
    optimizer_parameters.extend(
      [
        {"robot_count": robot_count},
        {"frame_id": "world"},
        {"fixed_z": goal_z},
        {"sample_count": LaunchConfiguration("sample_count")},
        {"map_size_x": map_size_x},
        {"map_size_y": map_size_y},
        {"map_resolution": map_resolution},
        {"collision_min_z": -0.05},
        {"collision_max_z": 0.85},
        {"obstacle_clearance": 0.72},
        {"swarm_clearance": 0.90},
        {"hard_obstacle_clearance": 0.50},
        {"hard_swarm_clearance": 0.85},
        {"max_velocity": max_velocity},
        {"max_acceleration": max_acceleration},
        {"sheet.enabled": True},
        {"sheet.length": sheet_length},
        {"sheet.width": sheet_width},
        {"sheet.distance_margin": sheet_distance_margin},
        {"topology.enabled": LaunchConfiguration("topology_enabled")},
        {"topology.weight": LaunchConfiguration("topology_weight")},
        {
            "topology.min_directed_distance": LaunchConfiguration(
                "topology_min_directed_distance"
            )
        },
        {
            "topology.hard_min_directed_distance": LaunchConfiguration(
                "topology_hard_min_directed_distance"
            )
        },
        {
            "object_clearance.enabled": object_clearance_enabled
        },
        {"object_clearance.rod_base_offset_z": rod_base_offset_z},
        {"object_clearance.rod_height": rod_height},
        {"object_clearance.object_diameter": object_diameter},
        {"object_clearance.footprint_radius": object_footprint_radius},
        {"object_clearance.obstacle_clearance": object_obstacle_clearance},
        {
            "object_clearance.distance_increment": object_distance_increment
        },
        {
            "object_clearance.geometric_nominal_sag": LaunchConfiguration(
                "object_visual_fallback_sag"
            )
        },
        {
            "object_clearance.nominal_formation_length": formation_length
        },
        {
            "object_clearance.nominal_formation_width": formation_width
        },
        {
            "object_clearance.max_iterations": object_clearance_max_iterations
        },
      ]
    )
    nodes.append(
        Node(
            package="legged_swarm_sim",
            executable="centralized_trajectory_optimizer",
            name="centralized_trajectory_optimizer",
            output="screen",
            parameters=optimizer_parameters,
        )
    )
    vvcm_parameters = []
    if vvcm_config:
        vvcm_parameters.append(vvcm_config)
    vvcm_parameters.append(
        {
            "robot_count": robot_count,
            "frame_id": "world",
            "sheet.length": sheet_length,
            "sheet.width": sheet_width,
            "formation.length": formation_length,
            "formation.width": formation_width,
            "rod.base_offset_z": rod_base_offset_z,
            "rod.height": rod_height,
            "object.diameter": object_diameter,
            "solver.enabled": vvcm_solver_enabled,
            "visual.fallback_sag": LaunchConfiguration(
                "object_visual_fallback_sag"
            ),
        }
    )
    nodes.append(
        Node(
            package="legged_swarm_sim",
            executable="vvcm_object_pose_node",
            name="vvcm_object_pose_node",
            output="screen",
            parameters=vvcm_parameters,
            condition=IfCondition(
                "true" if vvcm_visualization_enabled else "false"
            ),
        )
    )
    nodes.append(
        Node(
            package="legged_swarm_sim",
            executable="centralized_joint_trajectory_executor",
            name="centralized_joint_trajectory_executor",
            output="screen",
            parameters=[
                {"robot_count": robot_count},
                {"frame_id": "world"},
                {"fixed_z": goal_z},
                {"initial_x": [start_x + offset_x for offset_x, _ in offsets]},
                {"initial_y": [offset_y for _, offset_y in offsets]},
                {"body_length": 0.9},
                {"body_width": 0.45},
                {"body_height": 0.35},
                {"update_rate": 50.0},
                {"minimum_start_delay": 0.35},
                {"max_yaw_rate": max_yaw_rate},
                {
                    "allow_reverse_motion": LaunchConfiguration(
                        "allow_reverse_motion"
                    )
                },
            ],
        )
    )

    rviz_config = os.path.join(
        get_package_share_directory("legged_swarm_sim"),
        "rviz",
        "centralized_swarm.rviz",
    )
    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["--display-config", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    for robot_id, _ in enumerate(offsets):
        robot_name = f"robot_{robot_id}"
        nodes.append(
            Node(
                package="legged_swarm_sim",
                executable="robot_grid_astar_frontend",
                name=f"{robot_name}_astar_frontend",
                output="screen",
                remappings=[
                    ("odom_world", f"/{robot_name}/odom_world"),
                    ("goal", f"/{robot_name}/goal"),
                    ("frontend_path", f"/{robot_name}/frontend_path"),
                ],
                parameters=[
                    {"robot_id": robot_id},
                    {"frame_id": "world"},
                    {"fixed_z": goal_z},
                    {"robot_clearance": 0.68},
                    {"start_recovery_clearance": 0.50},
                    {"start_recovery_radius": 1.2},
                    {"map_size_x": map_size_x},
                    {"map_size_y": map_size_y},
                    {"map_resolution": map_resolution},
                    {"collision_min_z": -0.05},
                    {"collision_max_z": 0.85},
                ],
            )
        )
        peers = [
            f"tcp://127.0.0.1:{zmq_base_port + peer_id}"
            for peer_id in range(robot_count)
            if peer_id != robot_id
        ]
        nodes.append(
            Node(
                package="legged_swarm_sim",
                executable="swarm_zmq_bridge",
                name=f"{robot_name}_zmq_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_zmq")),
                remappings=[
                    ("odom_world", f"/{robot_name}/odom_world"),
                    ("planning/bspline", f"/drone_{robot_id}_planning/bspline"),
                    ("world_cloud", "/world/cloud"),
                    ("peer_state_markers", "/swarm/zmq_peer_state_markers"),
                    ("peer_obstacles_cloud", f"/{robot_name}/peer_obstacles_cloud"),
                ],
                parameters=[
                    {"robot_id": robot_name},
                    {"pub_bind_endpoint": f"tcp://*:{zmq_base_port + robot_id}"},
                    {"peer_endpoints": peers},
                    {"publish_rate": 5.0},
                    {"safety_radius": 0.55},
                    {"desired_distance": 2.2},
                    {"fixed_z": goal_z},
                    {"include_pointcloud": False},
                    {"frame_id": "world"},
                ],
            )
        )
    return nodes


def generate_launch_description():
    package_share = get_package_share_directory("legged_swarm_sim")
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_count", default_value="4"),
            DeclareLaunchArgument("formation_length", default_value="2.4"),
            DeclareLaunchArgument("formation_width", default_value="2.0"),
            DeclareLaunchArgument("rotate_formation_with_goal_yaw", default_value="false"),
            DeclareLaunchArgument("start_x", default_value="-8.0"),
            DeclareLaunchArgument("goal_x", default_value="8.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("goal_z", default_value="0.35"),
            DeclareLaunchArgument("map_size_x", default_value="24.0"),
            DeclareLaunchArgument("map_size_y", default_value="16.0"),
            DeclareLaunchArgument("map_resolution", default_value="0.2"),
            DeclareLaunchArgument("goal_boundary_margin", default_value="0.9"),
            DeclareLaunchArgument("sample_count", default_value="48"),
            DeclareLaunchArgument("max_velocity", default_value="2.0"),
            DeclareLaunchArgument("max_acceleration", default_value="4.0"),
            DeclareLaunchArgument("max_yaw_rate", default_value="1.2"),
            DeclareLaunchArgument("allow_reverse_motion", default_value="true"),
            DeclareLaunchArgument("sheet_length", default_value="3.8"),
            DeclareLaunchArgument("sheet_width", default_value="3.4"),
            DeclareLaunchArgument("sheet_distance_margin", default_value="0.10"),
            DeclareLaunchArgument("topology_enabled", default_value="true"),
            DeclareLaunchArgument("topology_weight", default_value="30.0"),
            DeclareLaunchArgument(
                "topology_min_directed_distance", default_value="0.35"
            ),
            DeclareLaunchArgument(
                "topology_hard_min_directed_distance", default_value="0.08"
            ),
            DeclareLaunchArgument("object_clearance_enabled", default_value="true"),
            DeclareLaunchArgument("rod_base_offset_z", default_value="0.175"),
            DeclareLaunchArgument("rod_height", default_value="2.4"),
            DeclareLaunchArgument("object_diameter", default_value="0.45"),
            DeclareLaunchArgument("object_footprint_radius", default_value="0.35"),
            DeclareLaunchArgument("object_obstacle_clearance", default_value="0.10"),
            DeclareLaunchArgument("object_distance_increment", default_value="0.05"),
            DeclareLaunchArgument(
                "object_clearance_max_iterations", default_value="30"
            ),
            DeclareLaunchArgument(
                "optimizer_config",
                default_value=os.path.join(
                    package_share, "config", "centralized_optimizer.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "vvcm_config",
                default_value=os.path.join(package_share, "config", "vvcm.yaml"),
            ),
            DeclareLaunchArgument("enable_vvcm", default_value="true"),
            DeclareLaunchArgument("enable_vvcm_solver", default_value="true"),
            DeclareLaunchArgument(
                "object_visual_fallback_sag", default_value="2.0"
            ),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("enable_zmq", default_value="false"),
            DeclareLaunchArgument("zmq_base_port", default_value="7200"),
            OpaqueFunction(function=launch_setup),
        ]
    )
