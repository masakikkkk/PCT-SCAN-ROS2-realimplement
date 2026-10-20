"""Run just the Foxy interfaces; no SLAM or planner is launched."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration("config")
    default = os.path.join(get_package_share_directory("go2_pct_adapter"), "config", "go2.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default),
        Node(package="go2_pct_adapter", executable="state_adapter",
             name="go2_state_adapter", parameters=[config], output="screen"),
        Node(package="go2_pct_adapter", executable="sport_bridge",
             name="go2_sport_bridge", parameters=[config], output="screen"),
    ])
