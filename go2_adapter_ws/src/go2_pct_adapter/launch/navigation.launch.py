"""Wire the downloaded PCT/SCAN packages directly to the Foxy interfaces.

This requires the upstream packages to have been built for the current ROS
distribution. It does not make their Humble code Foxy-compatible by itself.
"""

import math
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    config = LaunchConfiguration("config").perform(context)
    mode = LaunchConfiguration("map_mode").perform(context)
    if mode not in ("offline", "online"):
        raise RuntimeError("map_mode must be offline or online")
    tomogram_arg = LaunchConfiguration("tomogram_path").perform(context)
    tomogram = os.path.abspath(tomogram_arg) if tomogram_arg else ""
    if mode == "offline" and not os.path.isfile(tomogram):
        raise RuntimeError("tomogram_path must be YOUR mapped environment's tomogram")
    with open(config) as handle:
        settings = yaml.safe_load(handle)
    state = settings["go2_state_adapter"]["ros__parameters"]
    sport = settings["go2_sport_bridge"]["ros__parameters"]
    for name in ("planning_frame", "base_frame"):
        if state[name] != sport[name]:
            raise RuntimeError("State and Sport config disagree on " + name)
    frame = state["planning_frame"]
    height = float(LaunchConfiguration("body_height").perform(context))
    if not math.isfinite(height) or height < 0:
        raise RuntimeError("body_height must be finite and nonnegative")
    scan_share = get_package_share_directory("scan_planner")
    pct_share = get_package_share_directory("pct_planner")
    online = settings.get("go2_online_planner", {}).get("ros__parameters", {})
    return [
        Node(package="go2_pct_adapter", executable="state_adapter",
             name="go2_state_adapter", parameters=[config], output="screen"),
        Node(package="go2_pct_adapter", executable="sport_bridge",
             name="go2_sport_bridge", parameters=[config, {
                 "require_route_valid": True, "require_tracking_valid": True,
             }], output="screen"),
        Node(package="go2_pct_adapter", executable=(
                 "online_pct_bridge" if mode == "online" else "live_pct_bridge"),
             name="pct_scan_bridge", output="screen", parameters=[online if mode == "online" else {}, {
                 "rsg_root": (pct_share if mode == "online" else
                              os.path.dirname(os.path.dirname(tomogram))),
                 "defer_map_loading": mode == "online",
                 "scene_name": "building", "tomogram_path": tomogram,
                 "frame_id": frame, "coord_mode": "identity",
                 "path_topic": "/pct_go2/global_path",
                 "tomogram_topic": "/pct_go2/tomogram",
                 "goal_topic": "/pct_go2/goal",
                 "start_topic": "/pct_go2/unused_manual_start",
                 "clicked_point_topic": "/pct_go2/unused_clicked_point",
                 "plan_on_startup": False, "enable_interactive_markers": False,
                 "publish_tomogram": True, "optimize_path": False,
                 "robot_body_height": height,
                 "robot_odom_timeout_sec": sport["odom_timeout_sec"],
             }]),
        Node(package="scan_planner", executable="scan_planner_node",
             name="scan_planner_node", output="screen", parameters=[
                 os.path.join(scan_share, "config", "planner.yaml"), {
                     "use_sim_time": False, "fsm.navi_mode": 3,
                     "grid_map.frame_id": frame, "grid_map.sensor_type": "lidar",
                     "grid_map.cloud_is_world": state["cloud_is_world"],
                     "grid_map.need_extrinsic": False,
                     "grid_map.body_height": height,
                     "manager.max_vel": sport["max_vx"],
                     "optimization.max_vel": sport["max_vx"],
                 }], remappings=[
                     ("body_pose", "/pct_go2/body_pose"),
                     ("sensor_pose", "/pct_go2/sensor_pose"),
                     ("cloud", "/pct_go2/cloud"),
                     ("initial_path", "/pct_go2/global_path"),
                     ("planning/bspline", "/pct_go2/bspline"),
                 ]),
        Node(package="scan_planner", executable="closed_loop_controller",
             name="closed_loop_controller", output="screen", parameters=[
                 os.path.join(scan_share, "config", "controllers.yaml"), {
                     "use_sim_time": False, "max_vx": sport["max_vx"],
                     "max_vy": sport["max_vy"], "max_vyaw": sport["max_yaw_rate"],
                     "external_execution_control": True,
                     "require_reference_path": True,
                     "expected_frame": frame,
                     "odom_timeout_sec": sport["odom_timeout_sec"],
                     "validate_timestamps": True,
                 }], remappings=[
                     ("body_pose", "/pct_go2/body_pose"),
                     ("planning/bspline", "/pct_go2/bspline"),
                     ("cmd_vel", sport["cmd_topic"]),
                     ("initial_path", "/pct_go2/global_path"),
                     ("planning/tracking_valid", "/pct_go2/tracking_valid"),
                     ("execution_enabled", "/pct_go2/execution_enabled"),
                 ]),
    ]


def generate_launch_description():
    default = os.path.join(get_package_share_directory("go2_pct_adapter"), "config", "go2.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default),
        DeclareLaunchArgument("map_mode", default_value="offline"),
        DeclareLaunchArgument("tomogram_path", default_value=""),
        DeclareLaunchArgument("body_height", default_value="0.4"),
        OpaqueFunction(function=setup),
    ])
