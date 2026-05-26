#!/usr/bin/env python3
"""
Launch file: custom A* path planner (Rust node from the nav_astar package).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    nav_astar_node = Node(
        package="nav_astar",
        executable="nav_astar",
        name="nav_astar",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        nav_astar_node,
    ])
