#!/usr/bin/env bash
# Read-only inventory. Run in the existing, correctly sourced Go2 Foxy terminal.
set -u
if ! command -v ros2 >/dev/null 2>&1; then
  echo '请先加载 Go2 现有 Foxy 和宇树消息包环境。' >&2
  exit 2
fi
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
timeout 10s ros2 topic list -t
for topic in /utlidar/robot_odom /uslam/localization/odom \
             /utlidar/cloud_deskewed /utlidar/cloud \
             /api/sport/request /api/sport/response \
             /api/sport_lease/request /api/sport_lease/response /cmd_vel /tf /tf_static; do
  echo "Topic: $topic"
  timeout 5s ros2 topic info "$topic" --verbose || true
done
for schema in nav_msgs/msg/Odometry sensor_msgs/msg/PointCloud2 \
              unitree_api/msg/Request unitree_api/msg/RequestHeader \
              unitree_api/msg/RequestIdentity unitree_api/msg/RequestLease \
              unitree_api/msg/RequestPolicy; do
  echo "Schema: $schema"
  timeout 5s ros2 interface show "$schema" || true
done
