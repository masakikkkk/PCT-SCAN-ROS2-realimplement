"""Go2 navigation using the existing SLAM's accumulated /uslam/cloud_map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("go2_pct_adapter")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config", "go2.yaml")),
        DeclareLaunchArgument("body_height", default_value="0.4"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "navigation.launch.py")),
            launch_arguments={"map_mode": "online", "config": LaunchConfiguration("config"),
                              "body_height": LaunchConfiguration("body_height")}.items()),
    ])
