#!/usr/bin/env python3
"""
Launch file: TurtleBot3 in Gazebo + slam_toolbox (online_async).

Brings up the simulator and SLAM only — pair with nav2.py and/or astar.py
when those stacks are needed.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_model = DeclareLaunchArgument(
        "turtlebot3_model",
        default_value="waffle",
        description="TurtleBot3 model: burger | waffle | waffle_pi",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock",
    )
    declare_slam_params_file = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("slam_toolbox"), "config", "mapper_params_online_async.yaml"]
        ),
        description="Full path to the SLAM Toolbox parameters file",
    )

    use_sim_time     = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    turtlebot3_model = LaunchConfiguration("turtlebot3_model")

    set_tb3_model_env = SetEnvironmentVariable(
        name="TURTLEBOT3_MODEL", value=turtlebot3_model
    )
    set_logging_env = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")
    # Set our models/ dir first; turtlebot3_gazebo AppendEnvironmentVariable adds
    # its path after, so Gazebo searches ours first and picks our camera-free model.
    set_gz_models = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(get_package_share_directory("main"), "models"),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("turtlebot3_gazebo"), "launch", "turtlebot3_world.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time":     use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    return LaunchDescription([
        declare_model,
        declare_use_sim_time,
        declare_slam_params_file,
        set_tb3_model_env,
        set_logging_env,
        set_gz_models,
        gazebo_launch,
        slam_launch,
    ])
