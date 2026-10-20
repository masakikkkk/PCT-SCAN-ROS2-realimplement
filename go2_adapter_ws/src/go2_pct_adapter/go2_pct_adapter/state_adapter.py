"""Validate existing Go2 observations and publish SCAN's private input topics.

No SLAM, TF tree, sensor driver or robot odometry is created here.
"""

import copy
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from .core import finite, normalize_quaternion, rotate, sensor_pose, stamp_is_fresh


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def xyz(value):
    return (value.x, value.y, value.z)


def xyzw(value):
    return (value.x, value.y, value.z, value.w)


class StateAdapter(Node):
    def __init__(self):
        super().__init__("go2_state_adapter")
        defaults = {
            "odom_topic": "/utlidar/robot_odom",
            "cloud_topic": "/utlidar/cloud_deskewed",
            "planning_frame": "odom", "base_frame": "base_link",
            "sensor_frame": "utlidar_lidar", "odom_twist_frame": "child",
            "cloud_is_world": False, "use_tf_extrinsic": True,
            "extrinsics_confirmed": False,
            "body_to_sensor_xyz": [0.0, 0.0, 0.0],
            "body_to_sensor_xyzw": [0.0, 0.0, 0.0, 1.0],
            "max_input_age_sec": 0.25, "max_pose_cloud_skew_sec": 0.10,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        if self.cfg["odom_twist_frame"] not in ("child", "world"):
            raise ValueError("odom_twist_frame must be child or world")
        for name in ("planning_frame", "base_frame", "sensor_frame"):
            if not self.cfg[name]:
                raise ValueError(name + " must not be empty")
        for name in ("max_input_age_sec", "max_pose_cloud_skew_sec"):
            if not math.isfinite(self.cfg[name]) or self.cfg[name] <= 0:
                raise ValueError(name + " must be finite and positive")
        if not self.cfg["use_tf_extrinsic"]:
            if not self.cfg["extrinsics_confirmed"]:
                raise ValueError("Set measured extrinsics and extrinsics_confirmed=true")
            if len(self.cfg["body_to_sensor_xyz"]) != 3:
                raise ValueError("body_to_sensor_xyz must contain three values")
            normalize_quaternion(self.cfg["body_to_sensor_xyzw"])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.body_pub = self.create_publisher(Odometry, "/pct_go2/body_pose", 10)
        self.sensor_pub = self.create_publisher(Odometry, "/pct_go2/sensor_pose", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/pct_go2/cloud", 2)
        self.odom_sub = self.create_subscription(
            Odometry, self.cfg["odom_topic"], self.on_odom, qos_profile_sensor_data)
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.cfg["cloud_topic"], self.on_cloud, qos_profile_sensor_data)
        self.last_sensor = None
        self.last_warning = -math.inf

    def warn(self, reason):
        if time.monotonic() - self.last_warning >= 2.0:
            self.get_logger().warning(reason)
            self.last_warning = time.monotonic()

    def fresh(self, stamp):
        return stamp_is_fresh(self.get_clock().now().nanoseconds * 1e-9,
                              stamp_seconds(stamp), self.cfg["max_input_age_sec"])

    def on_odom(self, msg):
        # Do not silently relabel a lidar/IMU origin as the robot body.
        self.last_sensor = None
        if (msg.header.frame_id != self.cfg["planning_frame"] or
                msg.child_frame_id != self.cfg["base_frame"]):
            self.warn("Odometry frame mismatch: expected planning_frame -> base_frame; "
                      "select/transform the correct body odometry before this adapter")
            return
        if not self.fresh(msg.header.stamp):
            self.warn("Dropping stale/zero/future odometry timestamp")
            return
        position = xyz(msg.pose.pose.position)
        velocity = xyz(msg.twist.twist.linear)
        angular = xyz(msg.twist.twist.angular)
        try:
            orientation = normalize_quaternion(xyzw(msg.pose.pose.orientation))
            if not finite((*position, *velocity, *angular)):
                raise ValueError("non-finite body state")
            if self.cfg["use_tf_extrinsic"]:
                transform = self.buffer.lookup_transform(
                    self.cfg["base_frame"], self.cfg["sensor_frame"],
                    Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.0))
                translation = xyz(transform.transform.translation)
                rotation = xyzw(transform.transform.rotation)
            else:
                translation = self.cfg["body_to_sensor_xyz"]
                rotation = self.cfg["body_to_sensor_xyzw"]
            sensor_position, sensor_orientation = sensor_pose(
                position, orientation, translation, rotation)
        except Exception as exc:
            self.warn("Cannot construct sensor pose: " + str(exc))
            return
        body = copy.deepcopy(msg)
        q = body.pose.pose.orientation
        q.x, q.y, q.z, q.w = orientation
        # SCAN directly uses this velocity in its world-frame trajectory boundary.
        # This is a private planner message, not a replacement for robot odometry.
        if self.cfg["odom_twist_frame"] == "child":
            v = body.twist.twist.linear
            v.x, v.y, v.z = rotate(orientation, velocity)
            a = body.twist.twist.angular
            a.x, a.y, a.z = rotate(orientation, angular)
        sensor = Odometry()
        sensor.header = copy.deepcopy(body.header)
        sensor.child_frame_id = self.cfg["sensor_frame"]
        p, q = sensor.pose.pose.position, sensor.pose.pose.orientation
        p.x, p.y, p.z = sensor_position
        q.x, q.y, q.z, q.w = sensor_orientation
        self.last_sensor = sensor
        self.body_pub.publish(body)
        self.sensor_pub.publish(sensor)

    def on_cloud(self, msg):
        expected = (self.cfg["planning_frame"] if self.cfg["cloud_is_world"]
                    else self.cfg["sensor_frame"])
        if msg.header.frame_id != expected:
            self.warn("Cloud frame mismatch: expected " + expected)
            return
        if (self.last_sensor is None or not self.fresh(msg.header.stamp) or
                not self.fresh(self.last_sensor.header.stamp)):
            self.warn("Cloud is stale or no valid sensor pose is available")
            return
        skew = abs(stamp_seconds(msg.header.stamp) -
                   stamp_seconds(self.last_sensor.header.stamp))
        if skew > self.cfg["max_pose_cloud_skew_sec"]:
            self.warn("Cloud/pose skew exceeds configured tolerance")
            return
        fields = {field.name for field in msg.fields}
        if not {"x", "y", "z"}.issubset(fields) or msg.width * msg.height == 0:
            self.warn("Cloud is empty or missing XYZ fields")
            return
        # Original timestamps, fields and data are preserved. No fake TF is added.
        self.sensor_pub.publish(self.last_sensor)
        self.cloud_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
