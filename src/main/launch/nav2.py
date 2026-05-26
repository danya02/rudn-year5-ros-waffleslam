#!/usr/bin/env python3
"""
Launch file: Nav2 nodes (no docking_server, no route_server).

Assumes Gazebo + a map source (SLAM or map_server) are already running.

Velocity pipeline:
  controller_server → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed
  → collision_monitor → cmd_vel (TwistStamped) → ros_gz_bridge → Gazebo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically activate Nav2 lifecycle nodes",
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("main"), "config", "nav2_params.yaml"]
        ),
        description="Full path to the Nav2 parameters file",
    )

    use_sim_time     = LaunchConfiguration("use_sim_time")
    autostart        = LaunchConfiguration("autostart")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            root_key="",
            param_rewrites={"autostart": autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    nav2_nodes = GroupAction(
        actions=[
            SetParameter("use_sim_time", use_sim_time),

            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[configured_params],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "autostart":   autostart,
                    "node_names": [
                        "controller_server",
                        "smoother_server",
                        "planner_server",
                        "behavior_server",
                        "velocity_smoother",
                        "collision_monitor",
                        "bt_navigator",
                        "waypoint_follower",
                    ],
                }],
            ),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_nav2_params_file,
        nav2_nodes,
    ])
