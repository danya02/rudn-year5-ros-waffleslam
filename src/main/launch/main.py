#!/usr/bin/env python3
"""
Launch file: full stack (Gazebo + SLAM + Nav2 + custom A* planner).

This is a convenience aggregator. For task-specific runs use the split files:
  - slam.py  : Gazebo + TurtleBot3 + slam_toolbox
  - nav2.py  : Nav2 nodes
  - astar.py : custom A* Rust planner
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(name):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("main"), "launch", name])
        )
    )


def generate_launch_description():
    return LaunchDescription([
        _include("slam.py"),
        _include("nav2.py"),
        _include("astar.py"),
    ])
